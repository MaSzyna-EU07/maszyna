/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the Fields of the Terrain field of work: surfaces cut out of the terrain within an outline, each with a material of its own and
// kept in a list to be changed later, painted into the chunks or drawn as a mesh of their own over a hole; the ground under them
// made wavy (furrows, strips of fields, hills); the slopes of the formation modifiers made natural

#include "stdafx.h"
#include "application/editormode.h"
#include "application/editormodeshared.h"
#include "editor/editorFormat.hpp"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/translation.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <optional>
#include <sstream>
#include <unordered_map>

namespace
{
// the order the painted surfaces go on in: the slopes, then the track bed over them, then the ones drawn by hand over both
int paint_priority(heightmap::surface_patch const &Patch)
{
	if (Patch.name.find(" km ") != std::string::npos)
	{
		if (Patch.name.find(" torowisko ") != std::string::npos)
			return 1;
		if (Patch.name.find(" nasyp ") != std::string::npos || Patch.name.find(" przekop ") != std::string::npos)
			return 0;
	}
	return 2;
}

void reach_of(heightmap::surface_patch const &Patch, glm::dvec2 &Low, glm::dvec2 &High)
{
	Patch.bounds(Low, High);
	auto const margin{Patch.paint ? Patch.feather * 0.5 + Patch.noise + 1.0 : 1.0};
	Low -= glm::dvec2{margin};
	High += glm::dvec2{margin};
}

// every hole of a separate field has the same mark, so closing one field's hole closes it under another field over it as
// well: the separate fields left that overlap the closed outline are cut again (a cut leaves the holes there are as they are)
void recut_separate_over(terrain_streamer &Streamer, std::vector<heightmap::surface_patch> const &Patches, heightmap::surface_patch const &Closed)
{
	glm::dvec2 low;
	glm::dvec2 high;
	Closed.bounds(low, high);
	for (auto const &patch : Patches)
	{
		if (patch.paint || patch.outline.size() < 3)
			continue;
		glm::dvec2 otherlow;
		glm::dvec2 otherhigh;
		patch.bounds(otherlow, otherhigh);
		if (otherlow.x > high.x || otherhigh.x < low.x || otherlow.y > high.y || otherhigh.y < low.y)
			continue;
		Streamer.cut_field(patch.outline, true);
	}
}
} // namespace

namespace
{
// value noise of the waves, 0..1, smooth, the same for the same place: the waves of the ground don't change as it's cut again
double wave_hash(std::int64_t const X, std::int64_t const Z)
{
	auto h{static_cast<std::uint64_t>(X) * 0x9E3779B97F4A7C15ull ^ static_cast<std::uint64_t>(Z) * 0xC2B2AE3D27D4EB4Full};
	h ^= h >> 31;
	h *= 0xBF58476D1CE4E5B9ull;
	h ^= h >> 29;
	return static_cast<double>(h & 0xffffff) / static_cast<double>(0xffffff);
}

double wave_noise(double const X, double const Z)
{
	auto const ix{static_cast<std::int64_t>(std::floor(X))};
	auto const iz{static_cast<std::int64_t>(std::floor(Z))};
	auto const fx{X - std::floor(X)};
	auto const fz{Z - std::floor(Z)};
	auto const sx{fx * fx * (3.0 - 2.0 * fx)};
	auto const sz{fz * fz * (3.0 - 2.0 * fz)};
	auto const a{wave_hash(ix, iz) + (wave_hash(ix + 1, iz) - wave_hash(ix, iz)) * sx};
	auto const b{wave_hash(ix, iz + 1) + (wave_hash(ix + 1, iz + 1) - wave_hash(ix, iz + 1)) * sx};
	return a + (b - a) * sz;
}

// a few octaves of it, 0..1
double wave_fractal(double const X, double const Z, int const Octaves)
{
	double sum{0.0};
	double weight{1.0};
	double total{0.0};
	double scale{1.0};
	for (int i = 0; i < Octaves; ++i)
	{
		sum += wave_noise(X * scale + i * 17.3, Z * scale - i * 9.1) * weight;
		total += weight;
		weight *= 0.5;
		scale *= 2.0;
	}
	return sum / total;
}
} // namespace

void editor_mode::render_fields_ui()
{
	auto &settings{m_field};
	auto const &layers{m_streamer.manifest().layers};
	ImGui::TextWrapped("%s", STR_C("LMB adds a point of the outline, Shift+LMB takes the last one back. The field gets the material chosen in the palette and is "
	                               "kept in the list below, where its material can be changed later"));
	if (false == layers.empty())
	{
		auto const &chosen{layers[std::clamp(m_terrain_layer, 0, static_cast<int>(layers.size()) - 1)]};
		ImGui::Text(STR_C("Material: %s (chosen in the palette above)"), chosen.material.c_str());
	}
	ImGui::Text(STR_C("Outline: %zu points"), m_paint_outline.size());
	int kind{settings.separate ? 1 : 0};
	ImGui::RadioButton(STR_C("Painted into the terrain##fields"), &kind, 0);
	item_tooltip("The material goes into the paint of the chunks: no seam, the edge blends into the ground around; a chunk holds up to 8 materials");
	ImGui::SameLine();
	ImGui::RadioButton(STR_C("A surface of its own##fields"), &kind, 1);
	item_tooltip("The ground is cut away within the outline and the field drawn over the hole as a mesh of its own, following the heights of the terrain:\n"
	             "it doesn't take one of the 8 materials of a chunk, but its edge is sharp and the seam can show");
	settings.separate = kind == 1;
	ImGui::SeparatorText(STR_C("Edge"));
	if (false == settings.separate)
	{
		ImGui::SetNextItemWidth(120.0f);
		ImGui::DragFloat(STR_C("Transition (m)##fields"), &settings.feather, 0.05f, 0.0f, 20.0f, settings.feather > 0.0f ? "%.2f" : STR_C("sharp"));
		settings.feather = std::max(0.0f, settings.feather);
		item_tooltip("Metres across the edge the material passes into the one around over, half of them inside the field.\n"
		             "The paint of the terrain is kept every metre (every 0.5 m in the chunks converted to 0.5 m), narrower comes out in steps");
		ImGui::SetNextItemWidth(120.0f);
		ImGui::DragFloat(STR_C("Noise of the edge (m)##fields"), &settings.noise, 0.05f, 0.0f, 10.0f, settings.noise > 0.0f ? "%.2f" : STR_C("none"));
		settings.noise = std::max(0.0f, settings.noise);
		item_tooltip("How far the edge wanders in and out of the outline, so it isn't drawn with a ruler");
		ImGui::SetNextItemWidth(120.0f);
		ImGui::DragFloat(STR_C("Waves of the noise (m)##fields"), &settings.noise_scale, 0.1f, 0.5f, 50.0f, "%.1f");
		settings.noise_scale = std::max(0.5f, settings.noise_scale);
		item_tooltip("About how long the waves of the edge are: small - frayed, large - gently wavy");
		grass_choice(STR_C("3D grass under it##fields"), settings.grass);
	}
	ImGui::SetNextItemWidth(120.0f);
	ImGui::SliderFloat(STR_C("Texture turned by (deg)##fields"), &m_paint_rotation, -180.0f, 180.0f, "%.0f");
	render_waves_options();
	ImGui::Spacing();
	ImGui::BeginDisabled(m_paint_outline.size() < 3 || layers.empty());
	if (main_button(STR_C("Cut out the field"), ImVec2(-FLT_MIN, tall_button_height())))
		cut_out_field();
	ImGui::EndDisabled();
	if (ImGui::Button(STR_C("Clear the outline##fields"), ImVec2(-FLT_MIN, 0.0f)))
	{
		m_paint_outline.clear();
		m_patch_reoutline.clear(); // the next cut is a new field
	}
	render_patches_list();

	ImGui::SeparatorText(STR_C("Natural slopes"));
	auto &natural{m_natural};
	ImGui::TextWrapped("%s", STR_C("The embankments and the cuttings of the formation modifiers not drawn with a ruler: the formation and the edges of the track bed "
	                               "stay as designed, the slopes and their foot wander, get gentler here and there and meet the ground softly"));
	auto drag = [](char const *Label, double &Value, float Speed, float Low, float High, char const *Format, char const *Tip) {
		float value{static_cast<float>(Value)};
		ImGui::SetNextItemWidth(120.0f);
		if (ImGui::DragFloat(Label, &value, Speed, Low, High, Format))
			Value = std::clamp(static_cast<double>(value), static_cast<double>(Low), static_cast<double>(High));
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", Tip);
	};
	drag(STR_C("Foot wanders by (m)"), natural.wander, 0.05f, 0.0f, 5.0f, "%.2f", STR_C("How far the foot of the slope goes in and out of the line of the slope"));
	drag(STR_C("Waves along the line (m)"), natural.scale, 0.5f, 2.0f, 100.0f, "%.0f", STR_C("About how long the waves of the foot are along the line"));
	drag(STR_C("Gentler by up to"), natural.gentler, 0.01f, 0.0f, 0.6f, "%.2f", STR_C("0.2: the slope 1:1.5 gets as gentle as 1:1.8 here and there, never steeper than designed"));
	drag(STR_C("Soft foot (m/m)"), natural.soften, 0.01f, 0.0f, 2.0f, "%.2f", STR_C("Metres the foot is rounded off over, for a metre of the height of the slope: high embankments meet the ground softer"));
	drag(STR_C("Bumps on the face (m)"), natural.roughness, 0.01f, 0.0f, 0.5f, "%.2f", STR_C("Metres the face of the slope is uneven by, as earth settled for years"));
	int scope{settings.everywhere ? 1 : 0};
	ImGui::RadioButton(STR_C("Around the camera##natural"), &scope, 0);
	ImGui::SameLine();
	ImGui::RadioButton(STR_C("The whole scenery##natural"), &scope, 1);
	settings.everywhere = scope == 1;
	if (false == settings.everywhere)
	{
		ImGui::SameLine();
		ImGui::SetNextItemWidth(90.0f);
		ImGui::DragFloat(STR_C("m##natural"), &settings.radius, 10.0f, 100.0f, 20000.0f, "%.0f");
	}
	ImGui::BeginDisabled(m_streamer.modifiers().empty());
	if (main_button(STR_C("Make the slopes natural"), ImVec2(-FLT_MIN, tall_button_height())))
		naturalise_slopes(true);
	item_tooltip("The terrain under the modifiers is shaped anew; Ctrl+Z takes it back");
	if (ImGui::Button(STR_C("Back to the slopes as designed"), ImVec2(-FLT_MIN, 0.0f)))
		naturalise_slopes(false);
	ImGui::EndDisabled();
}


void editor_mode::cut_out_field()
{
	auto const &layers{m_streamer.manifest().layers};
	if (m_paint_outline.size() < 3 || layers.empty())
		return;
	auto const &settings{m_field};
	auto const &chosen{layers[std::clamp(m_terrain_layer, 0, static_cast<int>(layers.size()) - 1)]};
	heightmap::surface_patch patch;
	patch.material = chosen.material;
	patch.size = chosen.size;
	patch.rotation = static_cast<float>(std::remainder(static_cast<double>(chosen.rotation + m_paint_rotation), 360.0));
	patch.follow = true;
	patch.paint = false == settings.separate;
	patch.feather = settings.feather;
	patch.noise = settings.noise;
	patch.noise_scale = settings.noise_scale;
	patch.grass = settings.grass;
	for (auto const &point : m_paint_outline)
		patch.outline.emplace_back(point.x, point.z);
	auto patches{m_streamer.patches()};
	// the outline of a field put back to be drawn again: that field gets it, the rest of it as it was
	auto const again{std::find_if(patches.begin(), patches.end(), [this](heightmap::surface_patch const &Other) { return Other.name == m_patch_reoutline; })};
	m_patch_reoutline.clear();
	if (again != patches.end())
	{
		auto const old{*again};
		auto const index{static_cast<std::size_t>(again - patches.begin())};
		auto outline{std::move(patch.outline)};
		patch = old;
		patch.outline = std::move(outline);
		terrain_history_step();
		if (m_waves.on && apply_waves(patch.outline))
			forget_ground();
		auto const name{patch.name};
		patches[index] = patch;
		if (false == patch.paint)
		{
			m_streamer.cut_field(old.outline, false);
			recut_separate_over(m_streamer, patches, old); // the others; this one already has its new outline in the list
			m_streamer.cut_field(patch.outline, true);
			m_streamer.patches(std::move(patches));
			forget_ground();
		}
		else
			repaint_patches(std::move(patches), {index}, {old});
		terrain_history_step();
		m_patch_selected = static_cast<int>(index);
		m_paint_outline.clear();
		m_terrain_status = format(STR_C("%s outlined again, Ctrl+Z puts it back"), name.c_str());
		return;
	}
	// a name not taken yet
	for (int number = static_cast<int>(patches.size()) + 1;; ++number)
	{
		patch.name = format(STR_C("field %d"), number);
		if (std::none_of(patches.begin(), patches.end(), [&](heightmap::surface_patch const &Other) { return Other.name == patch.name; }))
			break;
	}
	auto const name{patch.name};
	auto const separate{false == patch.paint};
	// the waves of the ground and the field are one step of the undo
	terrain_history_step();
	if (m_waves.on && apply_waves(patch.outline))
		forget_ground();
	auto const outline{patch.outline};
	patches.push_back(std::move(patch));
	auto const index{patches.size() - 1};
	if (separate)
	{
		// the ground cut away under it, the field drawn over the hole with the heights the terrain has there
		m_streamer.cut_field(outline, true);
		m_streamer.patches(std::move(patches));
		forget_ground();
	}
	else
		repaint_patches(std::move(patches), {index});
	terrain_history_step();
	m_patch_selected = static_cast<int>(index);
	m_paint_outline.clear();
	m_terrain_status = format(STR_C("Cut out as %s with %s, Ctrl+Z puts it back"), name.c_str(), chosen.material.c_str());
}


void editor_mode::render_waves_options()
{
	auto &waves{m_waves};
	ImGui::SeparatorText(STR_C("Waves of the ground"));
	ImGui::Checkbox(STR_C("Make the ground under the surface wavy"), &waves.on);
	item_tooltip("The heights of the terrain within the outline get the waves chosen below, fading in from the edge: a field ploughed into furrows, a meadow of gentle hills, a hollow. Ctrl+Z takes them back with the surface");
	if (false == waves.on)
		return;
	char const *const kinds[]{STR_C("Rolling hills"), STR_C("Furrows of a field"), STR_C("Bumps and molehills"), STR_C("Long waves"), STR_C("A hollow or a mound"),
	                          STR_C("A grid of hummocks"), STR_C("Strips of fields")};
	char const *const tips[]{STR_C("Gentle hills of all sizes, as a meadow or a pasture has them"), STR_C("Ridges and furrows side by side, along the angle below, as a ploughed field or the beds of a garden"),
	                         STR_C("Small mounds scattered about, as molehills or tussocks"), STR_C("Long even waves across the angle below, as a field rolling over a hill or dunes"),
	                         STR_C("The middle lowered (a negative height raises it): a hollow by a pond, a dip in a field, or a mound"),
	                         STR_C("Waves along the angle and across it at once: hummocks in rows, as a drained meadow or an orchard on ridges"),
	                         STR_C("Long strips along the angle, each a little higher or lower than the next, with banks between them: the fields of a village cut up between farms")};
	ImGui::SetNextItemWidth(180.0f);
	if (ImGui::BeginCombo(STR_C("Kind##waves"), kinds[std::clamp(waves.kind, 0, static_cast<int>(terrain_waves::count) - 1)]))
	{
		for (int i = 0; i < terrain_waves::count; ++i)
		{
			if (ImGui::Selectable(kinds[i], i == waves.kind))
			{
				waves.kind = i;
				// the sizes a kind usually has
				static float const heights[]{0.6f, 0.25f, 0.2f, 1.0f, -0.8f, 0.3f, 0.6f};
				static float const lengths[]{40.0f, 1.5f, 3.0f, 60.0f, 0.0f, 6.0f, 25.0f};
				waves.height = heights[i];
				if (lengths[i] > 0.0f)
					waves.length = lengths[i];
			}
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s", tips[i]);
		}
		ImGui::EndCombo();
	}
	ImGui::SetNextItemWidth(120.0f);
	ImGui::DragFloat(STR_C("Height (m)##waves"), &waves.height, 0.01f, -5.0f, 5.0f, "%.2f");
	item_tooltip("From the bottom of a wave to its top; for the hollow how deep its middle goes, negative: a mound");
	if (waves.kind != terrain_waves::hollow)
	{
		ImGui::SetNextItemWidth(120.0f);
		ImGui::DragFloat(STR_C("Length of a wave (m)##waves"), &waves.length, 0.1f, 0.5f, 300.0f, "%.1f");
		waves.length = std::clamp(waves.length, 0.5f, 300.0f);
		item_tooltip("From one top to the next: furrows 0.7-2 m, the beds of a garden 1.5-3 m, hills 20-100 m");
	}
	if (waves.kind == terrain_waves::furrows || waves.kind == terrain_waves::waves || waves.kind == terrain_waves::grid || waves.kind == terrain_waves::strips)
	{
		ImGui::SetNextItemWidth(120.0f);
		ImGui::SliderFloat(STR_C("Direction (deg)##waves"), &waves.angle, -180.0f, 180.0f, "%.0f");
		item_tooltip("Which way the furrows or the ridges of the waves run, counterclockwise from the x axis of the scenery; the texture turned by the same angle lies along them");
		ImGui::SameLine();
		if (ImGui::SmallButton(STR_C("as the texture")))
			waves.angle = m_paint_rotation;
	}
	if (waves.kind != terrain_waves::hollow)
	{
		char const *const shapes[]{STR_C("Rounded"), STR_C("Sharp ridges"), STR_C("Sharp valleys"), STR_C("Terraces"), STR_C("Flat tops")};
		char const *const shape_tips[]{STR_C("Every wave a gentle curve, as before"), STR_C("Tops pointed, sides straight: ploughed ridges, a dune crest, a bank between fields"),
		                               STR_C("Bottoms pointed: drainage ditches, gullies cut by water, the furrow left by a plough"),
		                               STR_C("The slopes broken into flat steps: terraced fields on a hillside, a quarry, the shelves of a vineyard"),
		                               STR_C("Tops and bottoms cut flat with steep sides between: beds of a garden, fields on raised strips, plateaus")};
		ImGui::SetNextItemWidth(180.0f);
		if (ImGui::BeginCombo(STR_C("Profile##waves"), shapes[std::clamp(waves.shape, 0, static_cast<int>(terrain_waves::profiles) - 1)]))
		{
			for (int i = 0; i < terrain_waves::profiles; ++i)
			{
				if (ImGui::Selectable(shapes[i], i == waves.shape))
					waves.shape = i;
				if (ImGui::IsItemHovered())
					ImGui::SetTooltip("%s", shape_tips[i]);
			}
			ImGui::EndCombo();
		}
		item_tooltip("The cross-section of a wave; the rounded one makes the fields look soft, the others give them edges");
		if (waves.shape != terrain_waves::smooth)
		{
			ImGui::SetNextItemWidth(120.0f);
			ImGui::SliderFloat(STR_C("Sharpness##waves"), &waves.sharpness, 0.0f, 1.0f, "%.2f");
			item_tooltip("0: the profile barely changed from the rounded one; 1: its edges and corners as sharp as they get");
		}
		if (waves.shape == terrain_waves::terraces)
		{
			ImGui::SetNextItemWidth(120.0f);
			ImGui::SliderInt(STR_C("Steps##waves"), &waves.steps, 2, 12);
			item_tooltip("How many flat steps a slope is broken into, from the bottom of a wave to its top");
		}
	}
	ImGui::SetNextItemWidth(120.0f);
	ImGui::SliderFloat(STR_C("Roughness##waves"), &waves.roughness, 0.0f, 1.0f, "%.2f");
	item_tooltip("Clods and hummocks of about 3 m over the whole surface, up to 20 cm high: a ploughed field, a trodden pasture.\n"
	             "The terrain under the surface gets a point every metre for them (and for waves shorter than 15 m), the files grow");
	ImGui::SetNextItemWidth(120.0f);
	ImGui::SliderFloat(STR_C("Differences between them##waves"), &waves.variation, 0.0f, 1.0f, "%.2f");
	item_tooltip("0: every wave alike, as drawn; 1: waves of quite different heights and places, as nature has them");
	ImGui::SetNextItemWidth(120.0f);
	ImGui::DragFloat(STR_C("Fading in from the edge (m)##waves"), &waves.edge, 0.1f, 0.0f, 100.0f, "%.1f");
	waves.edge = std::max(0.0f, waves.edge);
	item_tooltip("Metres inside the outline over which the waves grow from nothing, so the ground around meets the surface without a step");
}

bool editor_mode::apply_waves(std::vector<glm::dvec2> const &Outline)
{
	if (Outline.size() < 3 || (std::abs(m_waves.height) < 0.001f && m_waves.roughness < 0.001f))
		return false;
	auto const waves{m_waves};
	glm::dvec2 low{std::numeric_limits<double>::max()}, high{-std::numeric_limits<double>::max()};
	for (auto const &point : Outline)
	{
		low = glm::min(low, point);
		high = glm::max(high, point);
	}
	auto const centre{(low + high) * 0.5};
	auto const radians{glm::radians(static_cast<double>(waves.angle))};
	glm::dvec2 const along{std::cos(radians), std::sin(radians)};
	glm::dvec2 const across{-along.y, along.x};
	auto const length{std::max(0.5, static_cast<double>(waves.length))};
	auto const variation{std::clamp(static_cast<double>(waves.variation), 0.0, 1.0)};
	auto const edge{std::max(0.0, static_cast<double>(waves.edge))};
	// for the hollow: how far in the middle of it is, the most distance from the edge found in the outline
	double deepest{edge};
	auto const inside_by = [&](glm::dvec2 const &Point, double &Distance) {
		bool inside{false};
		auto nearest{std::numeric_limits<double>::max()};
		for (std::size_t i = 0, j = Outline.size() - 1; i < Outline.size(); j = i++)
		{
			auto const &a{Outline[i]};
			auto const &b{Outline[j]};
			if ((a.y > Point.y) != (b.y > Point.y) && Point.x < (b.x - a.x) * (Point.y - a.y) / (b.y - a.y) + a.x)
				inside = !inside;
			auto const run{b - a};
			auto const run2{glm::dot(run, run)};
			auto const t{run2 > 1e-12 ? std::clamp(glm::dot(Point - a, run) / run2, 0.0, 1.0) : 0.0};
			nearest = std::min(nearest, glm::length(Point - (a + run * t)));
		}
		Distance = nearest;
		return inside;
	};
	if (waves.kind == terrain_waves::hollow)
	{
		// a coarse look for the middle, every few metres
		auto const step{std::max(1.0, std::max(high.x - low.x, high.y - low.y) / 60.0)};
		auto const rows{static_cast<int>(std::floor((high.y - low.y) / step))};
		auto const columns{static_cast<int>(std::floor((high.x - low.x) / step))};
		for (int row = 0; row <= rows; ++row)
			for (int column = 0; column <= columns; ++column)
				if (double distance; inside_by({low.x + column * step, low.y + row * step}, distance))
					deepest = std::max(deepest, distance);
	}
	auto const shape = [&](glm::dvec2 const &Point) -> double {
		auto const u{glm::dot(Point - centre, across)}; // across the furrows and the ridges
		auto const v{glm::dot(Point - centre, along)};
		switch (waves.kind)
		{
		case terrain_waves::hills:
			return wave_fractal(Point.x / length, Point.y / length, variation > 0.0 ? 3 : 1) * 2.0 - 1.0;
		case terrain_waves::furrows:
		{
			// the furrows wander a little sideways and change their depth along the field
			auto const wander{(wave_noise(v / (length * 12.0), u / (length * 40.0)) - 0.5) * length * variation};
			auto const depth{1.0 - variation * 0.5 * wave_noise(v / (length * 8.0) + 3.1, u / (length * 8.0))};
			return std::sin(glm::two_pi<double>() * (u + wander) / length) * depth;
		}
		case terrain_waves::bumps:
		{
			auto const n{wave_fractal(Point.x / length, Point.y / length, 2)};
			auto const threshold{0.55 - 0.15 * variation};
			auto const t{std::clamp((n - threshold) / (1.0 - threshold), 0.0, 1.0)};
			return t * t * 2.0; // 0..2 of the half height: the ground between the bumps stays as it is
		}
		case terrain_waves::waves:
		{
			auto const shift{(wave_noise(v / (length * 3.0), 0.37) - 0.5) * length * variation};
			auto const size{1.0 - variation * 0.6 * wave_noise(u / (length * 2.0), v / (length * 2.0) + 5.7)};
			return std::sin(glm::two_pi<double>() * (u + shift) / length) * size;
		}
		case terrain_waves::grid:
		{
			auto const shift{(wave_noise(u / (length * 4.0), v / (length * 4.0) + 1.3) - 0.5) * length * variation};
			auto const a{std::sin(glm::two_pi<double>() * (u + shift) / length)};
			auto const b{std::sin(glm::two_pi<double>() * (v - shift) / length)};
			return a * b;
		}
		case terrain_waves::strips:
		{
			// strips of about the length across, each of its own height, the banks between them as steep as the sharpness
			auto const width{length * (1.0 + variation * 0.6 * (wave_noise(v / (length * 10.0), 2.9) - 0.5))};
			auto const place{u / width};
			auto const strip{std::floor(place)};
			auto const in{place - strip};
			auto const level = [&](double const S) { return wave_noise(S * 0.731 + 0.17, 4.4) * 2.0 - 1.0; };
			auto const bank{0.5 * (0.35 - 0.3 * static_cast<double>(waves.sharpness))}; // share of the strip each side of the edge
			auto const here{level(strip)};
			if (in < bank)
			{
				auto t{(in + bank) / (2.0 * bank)};
				t = t * t * (3.0 - 2.0 * t);
				return level(strip - 1.0) + (here - level(strip - 1.0)) * t;
			}
			if (in > 1.0 - bank)
			{
				auto t{(in - (1.0 - bank)) / (2.0 * bank)};
				t = t * t * (3.0 - 2.0 * t);
				return here + (level(strip + 1.0) - here) * t;
			}
			return here;
		}
		default:
			return 0.0;
		}
	};
	// the cross-section of a wave: the value -1..1 of the shape bent to the profile chosen
	auto const sharp{std::clamp(static_cast<double>(waves.sharpness), 0.0, 1.0)};
	auto const steps{static_cast<double>(std::clamp(waves.steps, 2, 12))};
	auto const profile = [&](double S) -> double {
		S = std::clamp(S, -1.0, 1.0);
		switch (waves.shape)
		{
		case terrain_waves::ridges:
		case terrain_waves::valleys:
		{
			// the sine straightened into a triangle (straight sides, corners at the top and the bottom) as far as it's sharp,
			// then the valleys widened under pointed ridges, or the other way round
			auto const triangle{std::asin(S) * 2.0 / glm::pi<double>()};
			auto const t{(S + (triangle - S) * sharp) * 0.5 + 0.5};
			auto const power{1.0 + 2.0 * sharp};
			auto const bent{waves.shape == terrain_waves::ridges ? std::pow(t, power) : 1.0 - std::pow(1.0 - t, power)};
			return bent * 2.0 - 1.0;
		}
		case terrain_waves::terraces:
		{
			auto const q{(S * 0.5 + 0.5) * steps};
			auto const step{std::floor(q)};
			auto const in{q - step};
			// the ramp between the flat steps narrows with the sharpness
			auto const ramp{std::max(0.04, 1.0 - sharp * 0.92)};
			auto t{std::clamp((in - (1.0 - ramp)) / ramp, 0.0, 1.0)};
			t = t * t * (3.0 - 2.0 * t);
			return std::clamp((step + t) / steps, 0.0, 1.0) * 2.0 - 1.0;
		}
		case terrain_waves::plateaus:
		{
			// cut flat above and below: the sides steeper the sharper
			auto const gain{1.0 + sharp * 6.0};
			return std::tanh(S * gain) / std::tanh(gain);
		}
		default:
			return S;
		}
	};
	auto const rough{std::clamp(static_cast<double>(waves.roughness), 0.0, 1.0)};
	auto const shaped = [&](glm::dvec2 const &Point) -> double {
		double value;
		if (waves.kind == terrain_waves::bumps)
			value = profile(shape(Point) - 1.0) + 1.0; // the bumps stand on untouched ground: shaped from -1 up
		else if (waves.kind == terrain_waves::strips)
			value = shape(Point); // the strips have their own banks
		else
			value = profile(shape(Point));
		return value;
	};
	auto const half{static_cast<double>(waves.height) * 0.5};
	// a grid point every 5 m can't carry waves of 3 m or clods: the chunks under the surface made as fine as they have to be
	// (the shortest wave three points, a clod of 3 m every metre), never coarser than they are
	{
		auto finest{5.0};
		if (waves.kind != terrain_waves::hollow && std::abs(waves.height) >= 0.001f)
			finest = std::min(finest, length / 3.0);
		if (rough > 0.0)
			finest = std::min(finest, 1.0);
		float target{heightmap::spacings.front()};
		for (auto const spacing : heightmap::spacings)
			if (spacing <= finest + 1e-6)
				target = spacing;
		auto const first{heightmap::chunk_at(low.x, low.y)}, last{heightmap::chunk_at(high.x, high.y)};
		bool finer{false};
		for (int cx = std::min(first.first, last.first); cx <= std::max(first.first, last.first); ++cx)
			for (int cz = std::min(first.second, last.second); cz <= std::max(first.second, last.second); ++cz)
			{
				auto const centre{heightmap::chunk_corner({cx, cz}) + glm::dvec2{heightmap::chunk_size * 0.5}};
				auto const *chunk{m_streamer.terrain_at(centre.x, centre.y)};
				if (chunk != nullptr && chunk->data().spacing() > target + 1e-3 && m_streamer.spacing(cx, cz, target))
					finer = true;
			}
		if (finer)
		{
			// as after Point spacing: the tiles of the ground made from the old grid
			forget_ground_in(heightmap::chunk_corner(first), heightmap::chunk_corner(last) + glm::dvec2{heightmap::chunk_size});
		}
	}
	return m_streamer.reshape(low, high, [&](double const X, double const Z, float &Height) {
		glm::dvec2 const point{X, Z};
		double distance;
		if (false == inside_by(point, distance))
			return false;
		auto fade{edge > 0.0 ? std::clamp(distance / edge, 0.0, 1.0) : 1.0};
		fade = fade * fade * (3.0 - 2.0 * fade);
		double offset;
		if (waves.kind == terrain_waves::hollow)
		{
			// lowered most in the middle, a smooth bowl to the edge, a little uneven
			auto t{std::clamp(distance / std::max(deepest, 0.5), 0.0, 1.0)};
			t = t * t * (3.0 - 2.0 * t);
			auto const uneven{1.0 + variation * 0.3 * (wave_noise(X / 10.0, Z / 10.0) * 2.0 - 1.0)};
			offset = -static_cast<double>(waves.height) * t * uneven;
		}
		else
			offset = shaped(point) * half * fade;
		// clods and hummocks of about 3 m, up to 20 cm, whatever the waves (the grid made a point a metre for them)
		if (rough > 0.0)
			offset += (wave_fractal(X / 3.0 + 17.0, Z / 3.0 - 5.0, 3) * 2.0 - 1.0) * 0.2 * rough * fade;
		if (std::abs(offset) < 1e-4)
			return false;
		Height += static_cast<float>(offset);
		return true;
	});
}

bool editor_mode::paint_patch(heightmap::surface_patch const &Patch)
{
	auto const &layers{m_streamer.manifest().layers};
	if (Patch.outline.size() < 3 || layers.empty())
		return false;
	// the material as the palette has it, turned and repeated as the surface wants it: another entry of the palette when it differs
	std::uint16_t layer{0};
	if (false == Patch.material.empty())
		layer = m_streamer.layer(Patch.material, Patch.size, Patch.rotation);
	auto const painted{m_streamer.fill(Patch.outline, Patch.feather, layer, 1.0, Patch.noise, Patch.noise_scale)};
	// the 3D grass: the texture alone wouldn't do, the blades would go on growing over the gravel
	if (Patch.grass >= -1)
	{
		m_streamer.grass(Patch.outline, 0, true);
		if (Patch.grass >= 0 && Patch.grass < static_cast<int>(m_streamer.grass_kinds().size()))
			m_streamer.grass(Patch.outline, static_cast<std::uint16_t>(Patch.grass), false);
	}
	return painted;
}

bool editor_mode::grass_choice(char const *Label, int &Choice)
{
	auto const &kinds{m_streamer.grass_kinds()};
	auto const name = [&](int const Value) -> std::string {
		if (Value == -2)
			return STR("left as it grows");
		if (Value == -1)
			return STR("taken away");
		return Value < static_cast<int>(kinds.size()) ? format(STR_C("grown: %s"), kinds[Value].name.c_str()) : std::string{"?"};
	};
	bool changed{false};
	ImGui::SetNextItemWidth(160.0f);
	if (ImGui::BeginCombo(Label, name(Choice).c_str()))
	{
		for (int value = -2; value < static_cast<int>(kinds.size()); ++value)
			if (ImGui::Selectable(name(value).c_str(), value == Choice))
			{
				Choice = value;
				changed = true;
			}
		ImGui::EndCombo();
	}
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C("The 3D grass under the surface: left as it grows, taken away (gravel, a yard), or another kind of the grass palette grown there instead"));
	return changed;
}

void editor_mode::repaint_patches(std::vector<heightmap::surface_patch> All, std::vector<std::size_t> const &Changed, std::vector<heightmap::surface_patch> const &Cleared)
{
	// the ones changed, and the painted ones over them they touch: the transitions between them made again
	std::vector<char> paint(All.size(), 0);
	double constexpr cell{100.0};
	auto const key = [](double const X, double const Z) {
		return (static_cast<std::int64_t>(std::floor(X / cell)) << 32) ^ (static_cast<std::int64_t>(std::floor(Z / cell)) & 0xffffffff);
	};
	auto const cell_of = [](double const Value) { return static_cast<std::int64_t>(std::floor(Value / cell)); };
	std::unordered_map<std::int64_t, std::vector<std::size_t>> grid;
	if (false == Changed.empty() || false == Cleared.empty())
		for (std::size_t i = 0; i < All.size(); ++i)
		{
			if (false == All[i].paint)
				continue;
			glm::dvec2 low, high;
			reach_of(All[i], low, high);
			for (auto z{cell_of(low.y)}; z <= cell_of(high.y); ++z)
				for (auto x{cell_of(low.x)}; x <= cell_of(high.x); ++x)
					grid[key(static_cast<double>(x) * cell, static_cast<double>(z) * cell)].push_back(i);
		}
	auto const touching = [&](glm::dvec2 const &Low, glm::dvec2 const &High, auto const &Take) {
		for (auto z{cell_of(Low.y)}; z <= cell_of(High.y); ++z)
			for (auto x{cell_of(Low.x)}; x <= cell_of(High.x); ++x)
			{
				auto const lookup{grid.find(key(static_cast<double>(x) * cell, static_cast<double>(z) * cell))};
				if (lookup == grid.end())
					continue;
				for (auto const other : lookup->second)
				{
					glm::dvec2 low, high;
					reach_of(All[other], low, high);
					if (low.x <= High.x && high.x >= Low.x && low.y <= High.y && high.y >= Low.y)
						Take(other);
				}
			}
	};
	for (auto const index : Changed)
	{
		if (index >= All.size() || false == All[index].paint)
			continue;
		paint[index] = 1;
		glm::dvec2 low, high;
		reach_of(All[index], low, high);
		auto const priority{paint_priority(All[index])};
		// over it: higher in the order, or the same and later in the list
		touching(low, high, [&](std::size_t const Other) {
			auto const other{paint_priority(All[Other])};
			if (other > priority || (other == priority && Other > index))
				paint[Other] = 1;
		});
	}
	for (auto const &gone : Cleared)
	{
		glm::dvec2 low, high;
		reach_of(gone, low, high);
		// what was under it shows again; all the painted ones it touched get their edges back
		touching(low, high, [&](std::size_t const Other) { paint[Other] = 1; });
	}
	std::vector<std::size_t> order;
	for (std::size_t i = 0; i < All.size(); ++i)
		if (paint[i] != 0)
			order.push_back(i);
	std::stable_sort(order.begin(), order.end(), [&](std::size_t const A, std::size_t const B) { return paint_priority(All[A]) < paint_priority(All[B]); });
	m_streamer.patches(All);
	for (auto const &gone : Cleared)
	{
		auto base{gone};
		base.material.clear(); // the first material of the palette, as the ground was
		base.size = 0.f;
		base.rotation = 0.f;
		// the grass back as the terrain grows it most: the first kind of the palette, where it was taken away
		base.grass = gone.grass == -2 ? -2 : (m_streamer.grass_kinds().empty() ? -1 : 0);
		paint_patch(base);
	}
	std::size_t failed{0};
	for (auto const index : order)
		if (false == paint_patch(All[index]))
			++failed;
	if (failed > 0)
		WriteLog(format("Editor: %zu surfaces painted nothing: no chunk under them, or no room for another material in a chunk (8 at most)", failed));
	forget_ground();
}

void editor_mode::render_patches_list()
{
    auto const &patches = m_streamer.patches();
    if (patches.empty())
        return;
    ImGui::Text(STR_C("Fields: %zu"), patches.size());
    // a filter by name, for the many cut along the tracks (e.g. "nasyp", "L132 torowisko")
    static char filter[96]{};
    ImGui::SetNextItemWidth(-FLT_MIN);
    ImGui::InputTextWithHint("##patchfilter", STR_C("Filter by name, words apart by spaces..."), filter, sizeof(filter));
    std::vector<std::string> words;
    {
        std::istringstream split{std::string{filter}};
        std::string word;
        while (split >> word)
        {
            std::transform(word.begin(), word.end(), word.begin(), [](unsigned char C) { return static_cast<char>(std::tolower(C)); });
            words.push_back(word);
        }
    }
    // filtered again only when the filter or the list changes: tens of thousands of names a frame cost a frame
    static std::vector<int> shown;
    static std::string shown_filter;
    static void const *shown_revision{nullptr};
    static std::size_t shown_count{0};
    if (shown_filter != filter || shown_revision != static_cast<void const *>(patches.data()) || shown_count != patches.size())
    {
        shown_filter = filter;
        shown_revision = static_cast<void const *>(patches.data());
        shown_count = patches.size();
        shown.clear();
        shown.reserve(patches.size());
        for (int i = 0; i < static_cast<int>(patches.size()); ++i)
        {
            auto name{patches[i].name};
            std::transform(name.begin(), name.end(), name.begin(), [](unsigned char C) { return static_cast<char>(std::tolower(C)); });
            if (std::all_of(words.begin(), words.end(), [&](std::string const &Word) { return name.find(Word) != std::string::npos; }))
                shown.push_back(i);
        }
    }
    if (false == words.empty())
    {
        ImGui::TextDisabled(STR_C("%zu shown"), shown.size());
        auto const &layers = m_streamer.manifest().layers;
        ImGui::SameLine();
        ImGui::BeginDisabled(layers.empty() || shown.empty());
        if (ImGui::SmallButton(STR_C("Material of the palette to all shown")))
        {
            auto const &chosen = layers[std::clamp(m_terrain_layer, 0, static_cast<int>(layers.size()) - 1)];
            auto all{patches};
            std::vector<std::size_t> changed;
            for (auto const index : shown)
            {
                all[index].material = chosen.material;
                all[index].size = chosen.size;
                all[index].rotation = chosen.rotation;
                changed.push_back(static_cast<std::size_t>(index));
            }
            repaint_patches(std::move(all), changed);
            m_terrain_status = format(STR_C("%s given to %zu surfaces, Ctrl+Z takes it back"), chosen.material.c_str(), shown.size());
            ImGui::EndDisabled();
            return;
        }
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
            ImGui::SetTooltip("%s", STR_C("The material chosen in the palette, with its repeat and rotation, to all the surfaces the filter shows"));
    }
    std::optional<std::pair<std::size_t, std::string>> dropped;
    if (ImGui::BeginListBox("##patches", ImVec2(-FLT_MIN, std::min(8.0f, static_cast<float>(shown.size()) + 0.5f) * ImGui::GetTextLineHeightWithSpacing())))
    {
        ImGuiListClipper clipper;
        clipper.Begin(static_cast<int>(shown.size()));
        while (clipper.Step())
        for (int row = clipper.DisplayStart; row < clipper.DisplayEnd; ++row)
        {
            auto const i{static_cast<std::size_t>(shown[row])};
            ImGui::PushID(static_cast<int>(i));
            auto const material = patches[i].material.empty() ? std::string(STR_C("default material")) : patches[i].material;
            if (ImGui::Selectable(format(STR_C("%s, %zu points, %s"), patches[i].name.c_str(), patches[i].outline.size(), material.c_str()).c_str(), static_cast<int>(i) == m_patch_selected))
            {
                if (static_cast<int>(i) != m_patch_selected)
                    m_patch_reoutline.clear(); // another field chosen: the outline put back no longer stands for the one before
                m_patch_selected = static_cast<int>(i);
            }
            // a texture dropped on an entry becomes the material of that surface
            std::string name;
            if (material_drop(name))
                dropped = std::make_pair(i, name);
            ImGui::PopID();
        }
        ImGui::EndListBox();
    }
    if (dropped)
    {
        auto changed{patches};
        changed[dropped->first].material = dropped->second;
        repaint_patches(std::move(changed), {dropped->first});
        m_patch_selected = static_cast<int>(dropped->first);
        return;
    }
    if (m_patch_selected < 0 || m_patch_selected >= static_cast<int>(patches.size()))
        return;
    // the copy edited is kept from frame to frame while a widget is held, so a drag ends with the values dragged to; taken
    // anew from the list otherwise (another field chosen, an undo)
    if (false == ImGui::IsAnyItemActive() || m_patch_edited != m_patch_selected)
    {
        m_patch_edit = patches[m_patch_selected];
        m_patch_edited = m_patch_selected;
    }
    auto &patch{m_patch_edit};
    bool changed{false};
    char buffer[256];
    std::snprintf(buffer, sizeof(buffer), "%s", patch.material.c_str());
    ImGui::SetNextItemWidth(220.0f);
    if (ImGui::InputText(STR_C("Material##patch"), buffer, sizeof(buffer), ImGuiInputTextFlags_EnterReturnsTrue))
        patch.material = buffer, changed = true;
    std::string name;
    if (material_drop(name))
        patch.material = name, changed = true;
    ImGui::SameLine();
    auto const &layers = m_streamer.manifest().layers;
    ImGui::BeginDisabled(layers.empty());
    if (ImGui::Button(STR_C("From the palette##patch")))
    {
        auto const &chosen = layers[std::clamp(m_terrain_layer, 0, static_cast<int>(layers.size()) - 1)];
        patch.material = chosen.material;
        patch.size = chosen.size;
        patch.rotation = chosen.rotation;
        changed = true;
    }
    ImGui::EndDisabled();
    ImGui::SetNextItemWidth(120.0f);
    ImGui::DragFloat(STR_C("Repeat (m)##patch"), &patch.size, 0.1f, 0.0f, 500.0f, patch.size > 0.0f ? "%.1f" : STR_C("of the material"));
    changed |= ImGui::IsItemDeactivatedAfterEdit();
    ImGui::SetNextItemWidth(120.0f);
    ImGui::SliderFloat(STR_C("Turned by (deg)##patch"), &patch.rotation, -180.0f, 180.0f, "%.0f");
    changed |= ImGui::IsItemDeactivatedAfterEdit();
    if (patch.paint)
    {
        ImGui::SetNextItemWidth(120.0f);
        ImGui::DragFloat(STR_C("Transition (m)##patch"), &patch.feather, 0.05f, 0.0f, 20.0f, patch.feather > 0.0f ? "%.2f" : STR_C("sharp"));
        changed |= ImGui::IsItemDeactivatedAfterEdit();
        ImGui::SetNextItemWidth(120.0f);
        ImGui::DragFloat(STR_C("Noise of the edge (m)##patch"), &patch.noise, 0.05f, 0.0f, 10.0f, patch.noise > 0.0f ? "%.2f" : STR_C("none"));
        changed |= ImGui::IsItemDeactivatedAfterEdit();
        ImGui::SetNextItemWidth(120.0f);
        ImGui::DragFloat(STR_C("Waves of the noise (m)##patch"), &patch.noise_scale, 0.1f, 0.5f, 50.0f, "%.1f");
        changed |= ImGui::IsItemDeactivatedAfterEdit();
        changed |= grass_choice(STR_C("3D grass under it##patch"), patch.grass);
    }
    else
    {
        ImGui::SetNextItemWidth(120.0f);
        ImGui::DragFloat(STR_C("Lift (m)##patch"), &patch.lift, 0.01f, -5.0f, 5.0f, "%.2f");
        changed |= ImGui::IsItemDeactivatedAfterEdit();
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("%s", STR_C("Metres the surface lies over the heights taken from the terrain"));
    }
    if (changed)
    {
        patch.size = std::max(0.0f, patch.size);
        patch.feather = std::max(0.0f, patch.feather);
        patch.noise = std::max(0.0f, patch.noise);
        patch.noise_scale = std::max(0.5f, patch.noise_scale);
        auto all{patches};
        all[m_patch_selected] = patch;
        repaint_patches(std::move(all), {static_cast<std::size_t>(m_patch_selected)});
        return;
    }
    if (ImGui::Button(STR_C("Outline it again##patch")))
    {
        m_paint_outline.clear();
        for (auto const &point : patch.outline)
        {
            // on the ground: a field following the terrain has no heights of its own
            double height{0.0};
            if (false == m_streamer.height_at(point.x, point.y, height))
                patch.height(point.x, point.y, height);
            m_paint_outline.emplace_back(point.x, height, point.y);
        }
        // cutting it out again changes this field, not a new one
        m_patch_reoutline = patch.name;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("%s", STR_C("Puts the outline of the field back as the outline being drawn; cut out again, it changes this field"));
    ImGui::SameLine();
    if (ImGui::Button(STR_C("Give back to the terrain##patch")))
    {
        auto const gone = patches[m_patch_selected];
        auto all{patches};
        all.erase(all.begin() + m_patch_selected);
        m_patch_selected = -1;
        if (gone.paint)
        {
            // the first material of the palette over it again, the ones around get their edges back
            repaint_patches(std::move(all), {}, {gone});
            m_terrain_status = STR_C("Surface removed, the ground under it has the first material of the palette again");
            return;
        }
        m_streamer.cut_field(gone.outline, false);
        recut_separate_over(m_streamer, all, gone);
        m_streamer.patches(std::move(all));
        forget_ground();
        m_terrain_status = STR_C("Surface removed, the hole under it closed");
    }
}

void editor_mode::naturalise_slopes(bool const Natural)
{
	auto const &settings{m_field};
	glm::dvec2 const camera{Camera.Pos.x, Camera.Pos.z};
	std::vector<heightmap::modifier> changed;
	for (auto const &modifier : m_streamer.modifiers())
	{
		if (modifier.points.size() < 2)
			continue;
		if (false == settings.everywhere)
		{
			auto const [low, high]{modifier.bounds()};
			if (glm::distance(glm::clamp(camera, low, high), camera) > settings.radius)
				continue;
		}
		auto copy{modifier};
		copy.natural = Natural ? m_natural : heightmap::modifier::natural_shape{};
		changed.push_back(std::move(copy));
	}
	if (changed.empty())
	{
		m_terrain_status = STR_C("No formation modifier there");
		return;
	}
	auto const count{changed.size()};
	m_streamer.modifiers(std::move(changed));
	forget_ground();
	m_terrain_status = format(Natural ? STR_C("The slopes of %zu modifiers made natural, the terrain shaped anew. Ctrl+Z takes it back")
	                                  : STR_C("The slopes of %zu modifiers as designed again. Ctrl+Z takes it back"),
	                          count);
	WriteLog("Editor: " + m_terrain_status);
}
