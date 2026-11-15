/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the heightmap terrain made from the digital terrain model of GUGiK: the scenery placed as its orthophoto is, the heights
// of the model moved up or down to meet the tracks it has already, the chunks around the camera or along the tracks made as
// the model comes in, and the formation of each track with its slopes kept as a modifier of the terrain

#include "stdafx.h"
#include "application/editormode.h"
#include "application/editormodeshared.h"
#include "application/editoruilayer.h"
#include "application/editorprojection.h"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorKilometrage.hpp"
#include "editor/editorSettings.hpp"
#include "model/AnimModel.h"
#include "utilities/Globals.h"
#include "simulation/simulation.h"
#include "utilities/Logs.h"
#include "utilities/translation.h"
#include "world/Track.h"

#include "imgui/imgui.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <set>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace
{

// how many places of the tracks the offset of the model is worked out from at most, each needing its square of the model
std::size_t constexpr fit_places{48};
// the terrain under the tracks: places along them every so many m, the squares they're kept in, how far from the tracks the
// difference at them draws the model; past that the offset of the whole scenery
double constexpr anchor_step{25.0};
double constexpr anchor_cell{100.0};
double constexpr anchor_reach{300.0};
double constexpr anchor_near{5.0};

std::pair<int, int> anchor_cell_of(double const X, double const Z)
{
	return {static_cast<int>(std::floor(X / anchor_cell)), static_cast<int>(std::floor(Z / anchor_cell))};
}

// a path whose ends lie this far apart is an error of the scenery, as a point typed wrong: no grade line, no formation and no
// terrain along it
double constexpr broken_path{5000.0};

bool rail(TTrack const &Track)
{
	if ((Track.iCategoryFlag & 15) != 1 || Track.m_editorremoved || Track.m_paths.empty())
		return false;
	auto const &path{Track.m_paths.front()};
	return geometry::plan_distance(path.points[segment_data::point::start], path.points[segment_data::point::end]) <= broken_path;
}

// a choice of the new scenery as a card: a sketch of the ground a track lies on, the name and what it means; true when clicked.
// the real terrain: hills under an aerial photo, the track along them; none: the flat plane at 0 drawn as a grid
bool ground_card(char const *Id, char const *Title, char const *Text, bool const Hills, bool const Enabled, float const Width)
{
	auto const font{ImGui::GetFontSize()};
	auto const pad{std::round(font * 0.6f)};
	auto const sketch{std::round(font * 4.0f)};
	auto const wrap{Width - 2.0f * pad};
	auto const title{ImGui::CalcTextSize(Title, nullptr, false, wrap)};
	auto const text{ImGui::CalcTextSize(Text, nullptr, false, wrap)};
	auto const height{pad + sketch + pad * 0.6f + title.y + pad * 0.3f + text.y + pad};
	auto const top{ImGui::GetCursorScreenPos()};
	ImGui::BeginDisabled(false == Enabled);
	auto const clicked{ImGui::InvisibleButton(Id, ImVec2(Width, height))};
	ImGui::EndDisabled();
	auto const hovered{Enabled && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)};
	auto *list{ImGui::GetWindowDrawList()};
	ImVec2 const bottom{top.x + Width, top.y + height};
	auto const rounding{std::round(font * 0.4f)};
	auto const accent{family_colour(0)};
	list->AddRectFilled(top, bottom, ImGui::GetColorU32(hovered ? ImGuiCol_FrameBgHovered : ImGuiCol_FrameBg), rounding);
	list->AddRect(top, bottom, hovered ? accent : ImGui::GetColorU32(ImGuiCol_Border), rounding, 0, hovered ? 2.0f : 1.0f);
	auto const alpha{Enabled ? 1.0f : 0.35f};
	auto const tint = [&](ImU32 const Colour) { return (Colour & ~IM_COL32_A_MASK) | (static_cast<ImU32>(((Colour >> IM_COL32_A_SHIFT) & 0xFF) * alpha) << IM_COL32_A_SHIFT); };
	ImVec2 const low{top.x + pad, top.y + pad};
	ImVec2 const high{top.x + Width - pad, top.y + pad + sketch};
	list->AddRectFilled(low, high, tint(IM_COL32(20, 26, 34, 255)), rounding * 0.6f);
	list->PushClipRect(low, high, true);
	int constexpr steps{48};
	auto const x_of = [&](float const Share) { return low.x + (high.x - low.x) * Share; };
	if (Hills)
	{
		auto const ground = [](float const Share) { return 0.45f + 0.18f * std::sin(Share * 7.0f + 0.4f) + 0.08f * std::sin(Share * 17.0f); };
		std::vector<ImVec2> line;
		for (int i = 0; i <= steps; ++i)
		{
			auto const share{static_cast<float>(i) / steps};
			line.emplace_back(x_of(share), low.y + (high.y - low.y) * ground(share));
		}
		// the strips under the line drawn without the soft edges, else seams show between them
		auto const flags{list->Flags};
		list->Flags &= ~ImDrawListFlags_AntiAliasedFill;
		for (int i = 0; i < steps; ++i)
			list->AddQuadFilled(line[i], line[i + 1], ImVec2(line[i + 1].x, high.y), ImVec2(line[i].x, high.y), tint(IM_COL32(70, 110, 55, 200)));
		list->Flags = flags;
		list->AddPolyline(line.data(), static_cast<int>(line.size()), tint(IM_COL32(150, 200, 110, 255)), 0, 1.5f);
		// the track goes along the ground, a little above it on embankments, below it in cuttings
		std::vector<ImVec2> track;
		for (int i = 0; i <= steps; ++i)
		{
			auto const share{static_cast<float>(i) / steps};
			auto const smooth{0.45f + 0.09f * std::sin(share * 7.0f + 0.4f)};
			track.emplace_back(x_of(share), low.y + (high.y - low.y) * smooth - 2.0f);
		}
		list->AddPolyline(track.data(), static_cast<int>(track.size()), tint(IM_COL32(235, 235, 235, 255)), 0, 2.5f);
	}
	else
	{
		auto const horizon{low.y + (high.y - low.y) * 0.35f};
		auto const grid{tint(IM_COL32(120, 130, 150, 120))};
		for (int i = 0; i <= 4; ++i)
		{
			auto const t{static_cast<float>(i) / 4.0f};
			auto const y{horizon + (high.y - horizon) * t * t};
			list->AddLine(ImVec2(low.x, y), ImVec2(high.x, y), grid);
		}
		auto const middle{(low.x + high.x) * 0.5f};
		for (int i = -6; i <= 6; ++i)
			list->AddLine(ImVec2(middle + i * (high.x - low.x) * 0.06f, horizon), ImVec2(middle + i * (high.x - low.x) * 0.22f, high.y), grid);
		auto const level{horizon + (high.y - horizon) * 0.45f};
		list->AddLine(ImVec2(low.x, level), ImVec2(high.x, level), tint(IM_COL32(235, 235, 235, 255)), 2.5f);
	}
	list->PopClipRect();
	auto y{high.y + pad * 0.6f};
	list->AddText(nullptr, 0.0f, ImVec2(low.x, y), tint(ImGui::GetColorU32(ImGuiCol_Text)), Title, nullptr, wrap);
	y += title.y + pad * 0.3f;
	list->AddText(nullptr, 0.0f, ImVec2(low.x, y), tint(ImGui::GetColorU32(ImGuiCol_TextDisabled)), Text, nullptr, wrap);
	return clicked && Enabled;
}

// what a way of putting the tracks and the terrain together does, drawn along a track: onto, the track goes along the hills of
// the model; under, the track stays level and the ground comes to it; only, the hills of the model as they are over a level track
enum class way_sketch { onto, under, only };

// a way of the tracks and the terrain as a card the width of the panel: the sketch on the left, the name and what it's for on
// the right, the one to take first lit green; true when clicked
bool way_card(char const *Id, char const *Title, char const *Text, way_sketch const Sketch, bool const First, bool const Enabled)
{
	auto const font{ImGui::GetFontSize()};
	auto const pad{std::round(font * 0.5f)};
	auto const width{ImGui::GetContentRegionAvail().x};
	ImVec2 const sketch{std::round(font * 4.5f), std::round(font * 2.8f)};
	auto const left{pad + sketch.x + pad};
	auto const wrap{std::max(font * 4.0f, width - left - pad)};
	auto const title{ImGui::CalcTextSize(Title, nullptr, false, wrap)};
	auto const text{ImGui::CalcTextSize(Text, nullptr, false, wrap)};
	auto const height{pad + std::max(sketch.y, title.y + pad * 0.25f + text.y) + pad};
	auto const top{ImGui::GetCursorScreenPos()};
	ImGui::BeginDisabled(false == Enabled);
	auto const clicked{ImGui::InvisibleButton(Id, ImVec2(width, height))};
	ImGui::EndDisabled();
	auto const hovered{Enabled && ImGui::IsItemHovered()};
	auto const held{Enabled && ImGui::IsItemActive()};
	auto *list{ImGui::GetWindowDrawList()};
	ImVec2 const bottom{top.x + width, top.y + height};
	auto const rounding{std::round(font * 0.4f)};
	auto const lit{First && Enabled};
	auto const green{IM_COL32(70, 170, 85, 255)};
	auto const fill{lit ? (held ? IM_COL32(45, 110, 55, 255) : hovered ? IM_COL32(40, 95, 50, 255) : IM_COL32(32, 72, 40, 255))
	                    : ImGui::GetColorU32(held ? ImGuiCol_FrameBgActive : hovered ? ImGuiCol_FrameBgHovered : ImGuiCol_FrameBg)};
	list->AddRectFilled(top, bottom, fill, rounding);
	list->AddRect(top, bottom, lit ? green : hovered ? family_colour(0) : ImGui::GetColorU32(ImGuiCol_Border), rounding, 0, lit || hovered ? 2.0f : 1.0f);
	auto const alpha{Enabled ? 1.0f : 0.35f};
	auto const tint = [&](ImU32 const Colour) { return (Colour & ~IM_COL32_A_MASK) | (static_cast<ImU32>(((Colour >> IM_COL32_A_SHIFT) & 0xFF) * alpha) << IM_COL32_A_SHIFT); };
	ImVec2 const low{top.x + pad, top.y + (height - sketch.y) * 0.5f};
	ImVec2 const high{low.x + sketch.x, low.y + sketch.y};
	list->AddRectFilled(low, high, tint(IM_COL32(20, 26, 34, 255)), rounding * 0.6f);
	list->PushClipRect(low, high, true);
	int constexpr steps{40};
	auto const at = [&](float const Share, float const Height) { return ImVec2(low.x + sketch.x * Share, low.y + sketch.y * Height); };
	auto const hills = [](float const Share) { return 0.52f + 0.2f * std::sin(Share * 6.0f + 0.4f) + 0.07f * std::sin(Share * 15.0f); };
	auto const level{0.5f};
	// the ground as a line and the strips under it, drawn without the soft edges, else seams show between them
	auto const ground = [&](auto const &Height, ImU32 const Colour) {
		std::vector<ImVec2> line;
		for (int i = 0; i <= steps; ++i)
		{
			auto const share{static_cast<float>(i) / steps};
			line.push_back(at(share, Height(share)));
		}
		auto const flags{list->Flags};
		list->Flags &= ~ImDrawListFlags_AntiAliasedFill;
		for (int i = 0; i < steps; ++i)
			list->AddQuadFilled(line[i], line[i + 1], ImVec2(line[i + 1].x, high.y), ImVec2(line[i].x, high.y), tint(Colour));
		list->Flags = flags;
		list->AddPolyline(line.data(), static_cast<int>(line.size()), tint(IM_COL32(150, 200, 110, 255)), 0, 1.5f);
	};
	auto const track = [&](auto const &Height) {
		std::vector<ImVec2> line;
		for (int i = 0; i <= steps; ++i)
		{
			auto const share{static_cast<float>(i) / steps};
			line.push_back(at(share, Height(share)));
			line.back().y -= 2.0f;
		}
		list->AddPolyline(line.data(), static_cast<int>(line.size()), tint(IM_COL32(235, 235, 235, 255)), 0, 2.5f);
	};
	switch (Sketch)
	{
	case way_sketch::onto:
		// the track goes along the ground, a little above it on embankments, below it in cuttings
		ground(hills, IM_COL32(70, 110, 55, 220));
		track([](float const Share) { return 0.52f + 0.1f * std::sin(Share * 6.0f + 0.4f); });
		break;
	case way_sketch::under:
	{
		// the hills of the model faint where they were, the ground drawn up and down to the level track, fading out of it
		std::vector<ImVec2> was;
		for (int i = 0; i <= steps; ++i)
		{
			auto const share{static_cast<float>(i) / steps};
			was.push_back(at(share, hills(share)));
		}
		list->AddPolyline(was.data(), static_cast<int>(was.size()), tint(IM_COL32(150, 200, 110, 90)), 0, 1.0f);
		ground([&](float const Share) { return level + 0.04f + 0.25f * (hills(Share) - level); }, IM_COL32(70, 110, 55, 220));
		track([&](float) { return level; });
		break;
	}
	case way_sketch::only:
		// the level track first, the hills over it: it shows above the hollows and goes into the hills
		track([&](float) { return level; });
		ground(hills, IM_COL32(70, 110, 55, 235));
		break;
	}
	list->PopClipRect();
	auto const x{top.x + left};
	auto y{top.y + (height - (title.y + pad * 0.25f + text.y)) * 0.5f};
	list->AddText(nullptr, 0.0f, ImVec2(x, y), tint(ImGui::GetColorU32(ImGuiCol_Text)), Title, nullptr, wrap);
	y += title.y + pad * 0.25f;
	list->AddText(nullptr, 0.0f, ImVec2(x, y), tint(lit ? IM_COL32(200, 225, 200, 255) : ImGui::GetColorU32(ImGuiCol_TextDisabled)), Text, nullptr, wrap);
	return clicked && Enabled;
}

// m, the squares of the plan the points of the lines are kept in
double constexpr kPlanCell{50.0};

// a bridge or a tunnel keeps the model under it as it is, and a turntable its pit
bool shapes_ground(TTrack const &Track)
{
	return Track.eEnvironment != e_bridge && Track.eEnvironment != e_tunnel && Track.eType != tt_Table;
}

// the name the formation of a track is known by in the terrain, of a track with no name where it lies: two of the same name
// would be one formation
std::string formation_name(TTrack const &Track)
{
	if (false == Track.name().empty() || Track.m_paths.empty())
		return Track.name();
	auto const &points{Track.m_paths.front().points};
	auto const &start{points[segment_data::point::start]};
	auto const &end{points[segment_data::point::end]};
	return format("@%.2f,%.2f>%.2f,%.2f", start.x, start.z, end.x, end.z);
}

// an end of a path no other path is linked to
struct loose_end
{
	TTrack *track{nullptr};
	int path{0};
	bool atend{false};
	glm::dvec3 point{0.0};
	glm::dvec3 outward{0.0}; // off the path along it
};

loose_end end_of(TTrack &Track, int const Path, bool const Atend)
{
	auto const &points{Track.m_paths[Path].points};
	auto const &point{points[Atend ? segment_data::point::end : segment_data::point::start]};
	auto const &control{points[Atend ? segment_data::point::control2 : segment_data::point::control1]};
	auto const &other{points[Atend ? segment_data::point::start : segment_data::point::end]};
	return {&Track, Path, Atend, point, control != glm::dvec3{} ? -control : point - other};
}

// a track runs up to a turntable with its end this near the rim, facing the centre this closely (the cosine), as high as the
// table give or take, the table dropped into its pit too; the model this far past the rim lies beside the pit, past its wall;
// the line keeps level this far off the rim
double constexpr kRimNear{0.3}; // m
double constexpr kRimFacing{0.9};
double constexpr kRimLevel{3.0}; // m
double constexpr kPitWall{1.5}; // m
double constexpr kRunUp{10.0}; // m

bool on_rim(loose_end const &End, glm::dvec3 const &Centre, double const Rim)
{
	auto const outward{geometry::plan_of(End.outward)};
	auto const inward{geometry::plan_of(Centre - End.point)};
	return std::abs(glm::length(inward) - Rim) <= kRimNear && std::abs(End.point.y - Centre.y) <= kRimLevel && glm::length(outward) > 1e-6 &&
	       glm::dot(outward, inward) >= kRimFacing * glm::length(outward) * glm::length(inward);
}

// ends which meet in the plan facing each other, laid apart in height by hand, so the game doesn't join them; farther apart
// than a track over another can be, they lie on two levels
std::vector<std::pair<loose_end, loose_end>> loose_pairs()
{
	double constexpr kLevel{2.0}; // m
	auto const linked = [](TTrack const &Track, int const Path, bool const Atend) {
		if (Track.eType == tt_Switch && Track.SwitchExtension)
			return (Atend ? Track.SwitchExtension->pNexts[Path & 1] : Track.SwitchExtension->pPrevs[Path & 1]) != nullptr;
		return (Atend ? Track.trNext : Track.trPrev) != nullptr;
	};
	std::map<std::pair<int, int>, std::vector<loose_end>> cells;
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || false == rail(*track) || track->Length() < 0.01)
			continue;
		for (int i = 0; i < static_cast<int>(track->m_paths.size()); ++i)
			for (auto const atend : {false, true})
				if (false == linked(*track, i, atend))
				{
					auto const end{end_of(*track, i, atend)};
					cells[{static_cast<int>(std::floor(end.point.x)), static_cast<int>(std::floor(end.point.z))}].push_back(end);
				}
	}
	std::vector<std::pair<loose_end, loose_end>> result;
	std::set<std::tuple<TTrack const *, int, bool>> taken;
	for (auto const &[cell, ends] : cells)
		for (auto const &a : ends)
		{
			if (taken.contains({a.track, a.path, a.atend}))
				continue;
			loose_end const *match{nullptr};
			for (int dx = -1; dx <= 1 && match == nullptr; ++dx)
				for (int dz = -1; dz <= 1 && match == nullptr; ++dz)
				{
					auto const found{cells.find({cell.first + dx, cell.second + dz})};
					if (found == cells.end())
						continue;
					auto const outward{geometry::plan_of(a.outward)};
					for (auto const &b : found->second)
						if (b.track != a.track && false == taken.contains({b.track, b.path, b.atend}) && geometry::plan_distance(a.point, b.point) <= geometry::same_point &&
						    std::abs(a.point.y - b.point.y) <= kLevel && glm::dot(outward, geometry::plan_of(b.outward)) < -0.5 * glm::length(outward) * glm::length(geometry::plan_of(b.outward)))
						{
							match = &b;
							break;
						}
				}
			if (match == nullptr)
				continue;
			taken.insert({a.track, a.path, a.atend});
			taken.insert({match->track, match->path, match->atend});
			result.emplace_back(a, *match);
		}
	return result;
}

} // namespace

std::vector<editor_mode::nmt_place> editor_mode::nmt_track_places(double const Step) const
{
	std::vector<nmt_place> result;
	// a track with no trackbed of its own, as the switches of the older sceneries whose bed is an include, lies in the bed of
	// the others
	std::vector<double> depths;
	for (auto *track : simulation::Paths.sequence())
		if (track != nullptr && rail(*track) && profile_own_bed(*track))
			depths.push_back(profile_ballast(*track).first);
	std::nth_element(depths.begin(), depths.begin() + depths.size() / 2, depths.end());
	auto const usual{depths.empty() ? 0.0 : depths[depths.size() / 2]};
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || false == rail(*track))
			continue;
		auto const rails{static_cast<double>(track->rail_height())};
		for (std::size_t path = 0; path < track->m_paths.size(); ++path)
		{
			// the ballast drawn under the path: a switch has it of the plain track it goes on into, as the renderer lays it
			auto const *source{track};
			if (track->eType == tt_Switch && track->SwitchExtension != nullptr && path < 2)
			{
				auto const *previous{track->SwitchExtension->pPrevs[path]};
				auto const *next{track->SwitchExtension->pNexts[path]};
				source = previous != nullptr && previous->eType != tt_Switch ? previous : next != nullptr && next->eType != tt_Switch ? next : track;
			}
			auto ballast{profile_ballast(*source)};
			if (false == profile_own_bed(*source))
				ballast.first = usual;
			geometry::bezier const curve{track->m_paths[path]};
			auto const &points{track->m_paths[path].points};
			auto const chord{glm::distance(points[segment_data::point::start], points[segment_data::point::end])};
			auto const count{std::max(1, static_cast<int>(std::ceil(chord * 1.1 / Step)))};
			for (int i = 0; i <= count; ++i)
			{
				auto const at{curve.point(static_cast<double>(i) / count)};
				result.push_back({at, at.y - ballast.first, at.y - rails, ballast.second, track, static_cast<int>(path), shapes_ground(*track)});
			}
		}
	}
	// a track passing over another one by more than the clearance, as at a flyover whose bridge is a model: where the slopes of its
	// embankment would bury the track below, it stands on a bridge, and the ground there is the lower track's
	double constexpr clearance{3.0};
	double constexpr cell{20.0};
	auto const key_of = [&](double const X, double const Z) { return (static_cast<std::int64_t>(std::floor(X / cell)) << 32) ^ (static_cast<std::int64_t>(std::floor(Z / cell)) & 0xffffffff); };
	std::unordered_map<std::int64_t, std::vector<std::size_t>> grid;
	double widest{0.0};
	double lowest{std::numeric_limits<double>::max()}, highest{std::numeric_limits<double>::lowest()};
	for (std::size_t i = 0; i < result.size(); ++i)
	{
		grid[key_of(result[i].position.x, result[i].position.z)].push_back(i);
		widest = std::max(widest, result[i].half_width);
		lowest = std::min(lowest, result[i].crown);
		highest = std::max(highest, result[i].formation);
	}
	// the farthest the slopes of an embankment over a track can reach
	auto const reach{2.0 * widest + m_nmt_state.slope * std::max(0.0, highest - lowest)};
	auto const span{static_cast<int>(std::ceil(std::min(reach, 60.0) / cell))};
	std::vector<char> over(result.size(), 0);
	for (std::size_t i = 0; i < result.size(); ++i)
	{
		auto const &place{result[i]};
		if (false == place.shapes)
			continue;
		auto const cx{static_cast<std::int64_t>(std::floor(place.position.x / cell))};
		auto const cz{static_cast<std::int64_t>(std::floor(place.position.z / cell))};
		for (auto dz = -span; dz <= span && 0 == over[i]; ++dz)
			for (auto dx = -span; dx <= span && 0 == over[i]; ++dx)
			{
				auto const found{grid.find(((cx + dx) << 32) ^ ((cz + dz) & 0xffffffff))};
				if (found == grid.end())
					continue;
				for (auto const j : found->second)
				{
					auto const &other{result[j]};
					if (other.track == place.track)
						continue;
					auto const height{place.formation - other.crown};
					if (height < clearance)
						continue;
					auto const distance{glm::distance(glm::dvec2{place.position.x, place.position.z}, glm::dvec2{other.position.x, other.position.z})};
					if (distance <= place.half_width + other.half_width + m_nmt_state.slope * height)
					{
						over[i] = 1;
						break;
					}
				}
			}
	}
	for (std::size_t i = 0; i < result.size(); ++i)
		if (over[i] != 0)
			result[i].shapes = false;
	// nor where the grade lines went over the model falling away by the abutments of a bridge or a viaduct
	double constexpr bridged_near{1.0};
	double constexpr bridged_level{1.0};
	auto const &bridged{m_nmt_state.bridged};
	if (false == bridged.empty())
		for (auto &place : result)
		{
			if (false == place.shapes)
				continue;
			auto const cx{static_cast<int>(std::floor(place.position.x / kPlanCell))};
			auto const cz{static_cast<int>(std::floor(place.position.z / kPlanCell))};
			for (int dx = -1; dx <= 1 && place.shapes; ++dx)
				for (int dz = -1; dz <= 1 && place.shapes; ++dz)
					if (auto const cell{bridged.find({cx + dx, cz + dz})}; cell != bridged.end())
						for (auto const &point : cell->second)
							if (glm::length(glm::dvec2{point.x - place.position.x, point.z - place.position.z}) <= bridged_near && std::abs(point.y - place.position.y) <= bridged_level)
							{
								place.shapes = false;
								break;
							}
		}
	return result;
}

void editor_mode::nmt_locate()
{
	auto const &config{m_orthophoto.settings()};
	m_nmt.place(config.north, config.east);
}

void editor_mode::nmt_fit()
{
	nmt_locate();
	auto &state{m_nmt_state};
	state.error.clear();
	state.under = false;
	state.fit.clear();
	auto const places{nmt_track_places(25.0)};
	std::vector<nmt_place> shaped;
	std::copy_if(places.begin(), places.end(), std::back_inserter(shaped), [](nmt_place const &Place) { return Place.shapes; });
	if (shaped.empty())
	{
		state.offset = 0.0;
		state.fitted = true;
		state.status = STR("No tracks to fit the terrain model to: its heights are taken as they are");
		return;
	}
	// spread over the whole scenery, each place asking for its square of the model
	auto const every{std::max<std::size_t>(1, shaped.size() / fit_places)};
	for (std::size_t i = 0; i < shaped.size(); i += every)
		state.fit.push_back(shaped[i]);
	state.fitting = true;
	state.fetch_most = 0;
	state.status = STR("Fetching the terrain model under the tracks...");
}

void editor_mode::nmt_under_start()
{
	nmt_locate();
	auto &state{m_nmt_state};
	state.error.clear();
	state.fit.clear();
	state.anchors.clear();
	state.anchor_cells.clear();
	for (auto const &place : nmt_track_places(anchor_step))
		if (place.shapes)
			state.fit.push_back(place);
	if (state.fit.empty())
	{
		state.status = STR("No tracks to make the terrain under");
		return;
	}
	state.under = true;
	state.fitting = true;
	state.fetch_most = 0;
	state.result = false;
	state.findings.clear();
	state.status = STR("Fetching the terrain model under the tracks...");
}

void editor_mode::nmt_under_anchors()
{
	auto &state{m_nmt_state};
	struct found
	{
		glm::dvec2 at;
		double difference;
		TTrack const *track;
		int path;
	};
	std::vector<found> places;
	for (auto const &place : state.fit)
		if (double height; m_nmt.height(place.position.x, place.position.z, height))
			places.push_back({{place.position.x, place.position.z}, place.crown - height, place.track, place.path});
	state.anchors.clear();
	state.anchor_cells.clear();
	for (std::size_t i = 0; i < places.size(); ++i)
	{
		// a place where the model is off, as a bridge it doesn't know of or a cutting dug since, doesn't pull the terrain around
		// it: the median of the places around it along the same path
		std::vector<double> around;
		for (auto j{i >= 4 ? i - 4 : 0}; j < std::min(places.size(), i + 5); ++j)
			if (places[j].track == places[i].track && places[j].path == places[i].path)
				around.push_back(places[j].difference);
		std::nth_element(around.begin(), around.begin() + around.size() / 2, around.end());
		state.anchor_cells[anchor_cell_of(places[i].at.x, places[i].at.y)].push_back(state.anchors.size());
		state.anchors.emplace_back(places[i].at.x, places[i].at.y, around[around.size() / 2]);
	}
}

double editor_mode::nmt_offset_at(double const X, double const Z) const
{
	auto const &state{m_nmt_state};
	if (false == state.under || state.anchors.empty())
		return state.offset;
	// the places near weigh the most, down to nothing at the reach; the offset of the scenery weighs as one place at the reach
	auto const cell{anchor_cell_of(X, Z)};
	auto const span{static_cast<int>(std::ceil(anchor_reach / anchor_cell))};
	auto const outermost{1.0 / (anchor_reach * anchor_reach + anchor_near * anchor_near)};
	double sum{state.offset * outermost};
	double weights{outermost};
	for (int dz = -span; dz <= span; ++dz)
		for (int dx = -span; dx <= span; ++dx)
		{
			auto const found{state.anchor_cells.find({cell.first + dx, cell.second + dz})};
			if (found == state.anchor_cells.end())
				continue;
			for (auto const index : found->second)
			{
				auto const &anchor{state.anchors[index]};
				auto const squared{(anchor.x - X) * (anchor.x - X) + (anchor.y - Z) * (anchor.y - Z)};
				if (squared >= anchor_reach * anchor_reach)
					continue;
				auto const weight{1.0 / (squared + anchor_near * anchor_near) - outermost};
				sum += anchor.z * weight;
				weights += weight;
			}
		}
	return sum / weights;
}

void editor_mode::nmt_finish_fit()
{
	auto &state{m_nmt_state};
	std::vector<double> differences;
	for (auto const &place : state.fit)
		if (double height; m_nmt.height(place.position.x, place.position.z, height))
			differences.push_back(place.crown - height);
	state.fitting = false;
	if (differences.empty())
	{
		state.status = STR("The terrain model has no data under the tracks: is the scenery placed where it is?");
		return;
	}
	std::sort(differences.begin(), differences.end());
	state.offset = differences[differences.size() / 2];
	std::vector<double> deviations;
	for (auto const difference : differences)
		deviations.push_back(std::abs(difference - state.offset));
	std::sort(deviations.begin(), deviations.end());
	state.spread = deviations[deviations.size() / 2];
	state.fitted = true;
	state.samples = differences.size();
	state.status = format(STR_P("The tracks lie %+.2f m against the terrain model (%zu place, they differ by %.2f m around it)", "The tracks lie %+.2f m against the terrain model (%zu places, they differ by %.2f m around it)", state.samples), state.offset, state.samples, state.spread);
	if (state.spread > 10.0)
		state.status += STR(". That's far too much: the place of the scenery is likely wrong, check the orthophoto");
	WriteLog("Editor: NMT - " + state.status, logtype::generic);
	if (false == state.under)
		return;
	// the terrain under the tracks: drawn to them place by place, then made along them
	nmt_under_anchors();
	auto const status{state.status};
	nmt_make(nmt_keys_along_tracks(), true);
	state.status = status + "\n" + state.status;
}

std::vector<heightmap::chunk_key> editor_mode::nmt_keys_around() const
{
	std::vector<heightmap::chunk_key> result;
	auto const centre{heightmap::chunk_at(Camera.Pos.x, Camera.Pos.z)};
	auto const first{-(m_terrain_add_count - 1) / 2};
	for (int dz = 0; dz < m_terrain_add_count; ++dz)
		for (int dx = 0; dx < m_terrain_add_count; ++dx)
		{
			heightmap::chunk_key const key{centre.first + first + dx, centre.second + first + dz};
			// none outside the download area
			if (auto const low{heightmap::chunk_corner(key)}; in_download_area(low, low + glm::dvec2{heightmap::chunk_size}))
				result.push_back(key);
		}
	return result;
}

std::vector<heightmap::chunk_key> editor_mode::nmt_keys_along_tracks() const
{
	std::set<heightmap::chunk_key> keys;
	auto const reach{static_cast<int>(std::ceil(m_nmt_state.band / heightmap::chunk_size))};
	for (auto const &place : nmt_track_places(heightmap::chunk_size * 0.5))
	{
		auto const at{heightmap::chunk_at(place.position.x, place.position.z)};
		for (int dz = -reach; dz <= reach; ++dz)
			for (int dx = -reach; dx <= reach; ++dx)
			{
				heightmap::chunk_key const key{at.first + dx, at.second + dz};
				auto const low{heightmap::chunk_corner(key)};
				// the chunk reaches within the band of the track
				glm::dvec2 const nearest{std::clamp(place.position.x, low.x, low.x + heightmap::chunk_size), std::clamp(place.position.z, low.y, low.y + heightmap::chunk_size)};
				if (glm::distance(nearest, glm::dvec2{place.position.x, place.position.z}) <= m_nmt_state.band && in_download_area(low, low + glm::dvec2{heightmap::chunk_size}))
					keys.insert(key);
			}
	}
	return {keys.begin(), keys.end()};
}

void editor_mode::nmt_make(std::vector<heightmap::chunk_key> const &Keys, bool const Reshape)
{
	auto &state{m_nmt_state};
	if (false == ensure_terrain())
	{
		state.status = STR("The heightmap terrain can't be opened");
		return;
	}
	nmt_locate();
	state.given_up.clear();
	std::size_t queued{0};
	for (auto const &key : Keys)
	{
		if (std::find(state.pending.begin(), state.pending.end(), key) != state.pending.end())
			continue;
		// a chunk of flat ground there, as made with no model or no ground under it, gets the ground of the model as a new one would
		if (m_streamer.exists(key))
		{
			if (false == nmt_flat_chunk(key))
				continue;
			state.refill.insert(key);
		}
		state.pending.push_back(key);
		++queued;
	}
	state.made = 0;
	state.empty = 0;
	state.foreign = 0;
	state.fetch_most = 0;
	state.error.clear();
	// the chunks are the last step of the tracks put onto the terrain only after their grade lines
	if (false == Reshape || queued == 0)
		state.run = false;
	if (queued > 0)
	{
		state.grown = false;
		state.reshape = Reshape;
	}
	// the terrain comes with its photos, unless the user turned them off
	if (queued > 0 && false == m_orthophoto.enabled() && false == m_orthophoto_chosen)
		m_orthophoto.enabled(true);
	state.status = queued > 0 ? format(STR_P("%zu chunk to make from the terrain model", "%zu chunks to make from the terrain model", queued), queued) : STR("There's terrain there already");
	// with chunks to make the formations are laid once they're made
	if (Reshape && queued == 0 && state.shape)
	{
		nmt_shape_tracks();
		state.status += format(STR_P("; the formation of %zu path shaped", "; the formation of %zu paths shaped", state.shaped), state.shaped);
	}
}

bool editor_mode::nmt_ready(glm::dvec2 const &Min, glm::dvec2 const &Max)
{
	return m_nmt.request(Min, Max) && false == m_nmt.retrying(Min, Max);
}

std::optional<double> editor_mode::nmt_terrain_offset(std::vector<glm::dvec2> const &Points, bool &Foreign) const
{
	Foreign = false;
	std::vector<double> differences;
	for (auto const &point : Points)
		if (double terrain, model; m_streamer.height_at(point.x, point.y, terrain) && m_nmt.height(point.x, point.y, model))
			differences.push_back(terrain - model);
	if (differences.size() < 5)
		return std::nullopt;
	std::nth_element(differences.begin(), differences.begin() + differences.size() / 2, differences.end());
	auto const median{differences[differences.size() / 2]};
	// the formations of the tracks and the touch-ups of the brushes leave the terrain here and there, the rest keeps to the model
	auto const along{std::count_if(differences.begin(), differences.end(), [&](double const Difference) { return std::abs(Difference - median) <= 0.3; })};
	if (static_cast<double>(along) < 0.7 * static_cast<double>(differences.size()))
	{
		Foreign = true;
		return std::nullopt;
	}
	return median;
}

std::optional<double> editor_mode::nmt_chunk_offset(heightmap::chunk_key const &Key) const
{
	// the one rule for every chunk made from the model where there's no terrain, under the track laid, around the camera in the area or
	// by the buttons: beside terrain made from the model it goes on at its height over the model, so no step is left at the seam;
	// otherwise, beside terrain made by hand or none, at the offset of the scenery. no chunk is left out for what lies beside it
	// the terrain drawn to the tracks place by place (under the tracks) keeps to their heights near it: the height of the chunk beside
	// would carry one offset on along a run of chunks, metres off the tracks
	if (auto const &state{m_nmt_state}; state.under && false == state.anchors.empty())
		return std::nullopt;
	bool foreign{false};
	auto const offset{nmt_seam_offset(Key, foreign)};
	return foreign ? std::nullopt : offset;
}

std::optional<double> editor_mode::nmt_seam_offset(heightmap::chunk_key const &Key, bool &Foreign) const
{
	auto const low{heightmap::chunk_corner(Key)};
	auto const size{heightmap::chunk_size};
	int constexpr steps{16};
	struct side
	{
		int dx;
		int dz;
		glm::dvec2 from;
		glm::dvec2 to;
	};
	side const sides[]{{-1, 0, low, {low.x, low.y + size}}, {1, 0, {low.x + size, low.y}, low + glm::dvec2{size}}, {0, -1, low, {low.x + size, low.y}}, {0, 1, {low.x, low.y + size}, low + glm::dvec2{size}}};
	std::vector<glm::dvec2> points;
	for (auto const &edge : sides)
		if (m_streamer.exists({Key.first + edge.dx, Key.second + edge.dz}))
			for (int i = 0; i <= steps; ++i)
				points.push_back(edge.from + (edge.to - edge.from) * (static_cast<double>(i) / steps) + glm::dvec2{edge.dx, edge.dz} * 0.01);
	return nmt_terrain_offset(points, Foreign);
}

void editor_mode::nmt_track_changed(std::vector<TTrack *> const &Tracks)
{
	auto &state{m_nmt_state};
	if (state.restoring || state.laying || state.stage != nmt_state::automatic::idle || Tracks.empty() || false == m_streamer.active())
		return;
	for (auto *track : Tracks)
		if (track != nullptr)
			state.changed.insert(track);
	state.changed_time = ImGui::GetTime();
}

void editor_mode::nmt_track_gone(TTrack const &Track)
{
	auto &state{m_nmt_state};
	if (state.restoring || state.stage != nmt_state::automatic::idle || Track.name().empty() || false == m_streamer.active())
		return;
	state.gone.insert(Track.name());
	state.changed_time = ImGui::GetTime();
}

void editor_mode::nmt_follow_tracks()
{
	auto &state{m_nmt_state};
	if (state.changed.empty() && state.gone.empty())
		return;
	if (false == state.follow)
	{
		state.changed.clear();
		state.gone.clear();
		return;
	}
	if (state.stage != nmt_state::automatic::idle || state.fitting || ImGui::GetTime() - state.changed_time < 0.3 || ImGui::IsMouseDown(ImGuiMouseButton_Left) || ImGui::IsMouseDown(ImGuiMouseButton_Right))
		return;
	// laying on, from the end of the track laid or dragged from a free end, the terrain waits for the laying to end: shaping it
	// goes through every chunk under the tracks, a stall each click
	if ((m_lay.active && false == m_lay.points.empty()) || m_extend.active)
		return;
	if (false == m_streamer.active())
	{
		state.changed.clear();
		state.gone.clear();
		return;
	}
	auto const &modifiers{m_streamer.modifiers()};
	auto const of = [](heightmap::modifier const &Modifier, std::string const &Track) { return Modifier.name == "nmt " + Track || Modifier.name.starts_with("nmt " + Track + " "); };
	std::vector<std::string> dropped;
	for (auto const &name : state.gone)
		if (simulation::Paths.find(name) == nullptr)
			for (auto const &modifier : modifiers)
				if (of(modifier, name))
					dropped.push_back(modifier.name);
	state.gone.clear();
	// the tracks with a formation take it along; one with none leaves the terrain as it is, until it's drawn to the track
	std::vector<TTrack *> shaped;
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || false == state.changed.contains(track) || track->m_editorremoved || false == rail(*track) || track->name().empty())
			continue;
		if (std::any_of(modifiers.begin(), modifiers.end(), [&](heightmap::modifier const &Modifier) { return of(Modifier, track->name()); }))
			shaped.push_back(track);
	}
	state.changed.clear();
	if (dropped.empty() && shaped.empty())
		return;
	// the terrain follows in the step of the history of the change of the tracks
	terrain_history_step();
	auto const *top{m_history.empty() ? nullptr : m_history.back().terrain.get()};
	m_streamer.remove_modifiers(dropped);
	auto const laid{shaped.empty() ? std::size_t{0} : nmt_shape_tracks(&shaped)};
	if (false == dropped.empty())
		forget_ground();
	terrain_history_step();
	auto const count{m_history.size()};
	auto const of_tracks = [](EditorSnapshot const &Step) { return Step.action == EditorSnapshot::Action::TrackEdit || (Step.action == EditorSnapshot::Action::Terrain && Step.joined); };
	if (count >= 2 && m_history.back().action == EditorSnapshot::Action::Terrain && m_history.back().terrain.get() != top && of_tracks(m_history[count - 2]))
		m_history.back().joined = true;
	WriteLog(format("Editor: NMT - the terrain followed the tracks: %zu formations laid, %zu taken away", laid, dropped.size()), logtype::generic);
}

bool editor_mode::nmt_ahead(double const X, double const Z, double &Height) const
{
	auto const &state{m_nmt_state};
	if (false == state.grow || false == state.ahead.has_value() || false == m_nmt.placed() || false == m_nmt.height(X, Z, Height))
		return false;
	Height += *state.ahead;
	return true;
}

bool editor_mode::nmt_ground(double const X, double const Z, double &Height) const
{
	if (nmt_ahead(X, Z, Height))
		return true;
	if (false == m_nmt.placed() || false == m_nmt.height(X, Z, Height))
		return false;
	Height += nmt_offset_at(X, Z);
	return true;
}

editor_mode::nmt_chunk editor_mode::add_chunk_from_nmt(heightmap::chunk_key const &Key, std::optional<double> const Offset)
{
	auto const low{heightmap::chunk_corner(Key)};
	auto const high{low + glm::dvec2{heightmap::chunk_size}};
	if (false == nmt_ready(low - glm::dvec2{2.0}, high + glm::dvec2{2.0}))
		return nmt_chunk::waiting;
	auto const cells{heightmap::cells_for(m_terrain_spacing)};
	auto const side{static_cast<std::size_t>(cells) + 1};
	auto const step{heightmap::chunk_size / cells};
	std::vector<double> heights(side * side, m_terrain_baseheight);
	std::vector<char> found(side * side, 0);
	// the offset over the chunk, every sixteenth of its side and between them along straight lines: it changes slowly
	int constexpr coarse{16};
	std::array<double, (coarse + 1) * (coarse + 1)> offsets;
	for (int iz = 0; iz <= coarse; ++iz)
		for (int ix = 0; ix <= coarse; ++ix)
			offsets[iz * (coarse + 1) + ix] = Offset.has_value() ? *Offset : nmt_offset_at(low.x + ix * heightmap::chunk_size / coarse, low.y + iz * heightmap::chunk_size / coarse);
	auto const offset_at = [&](double const X, double const Z) {
		auto const u{std::clamp((X - low.x) / heightmap::chunk_size * coarse, 0.0, static_cast<double>(coarse))};
		auto const v{std::clamp((Z - low.y) / heightmap::chunk_size * coarse, 0.0, static_cast<double>(coarse))};
		auto const i{std::min(static_cast<int>(u), coarse - 1)};
		auto const j{std::min(static_cast<int>(v), coarse - 1)};
		auto const fu{u - i};
		auto const fv{v - j};
		auto const at = [&](int const I, int const J) { return offsets[J * (coarse + 1) + I]; };
		return (at(i, j) * (1.0 - fu) + at(i + 1, j) * fu) * (1.0 - fv) + (at(i, j + 1) * (1.0 - fu) + at(i + 1, j + 1) * fu) * fv;
	};
	bool any{false};
	for (std::size_t iz = 0; iz < side; ++iz)
		for (std::size_t ix = 0; ix < side; ++ix)
		{
			auto const index{iz * side + ix};
			if (double height; m_nmt.height(low.x + ix * step, low.y + iz * step, height))
			{
				heights[index] = height + offset_at(low.x + ix * step, low.y + iz * step);
				found[index] = 1;
				any = true;
			}
		}
	if (false == any)
		return nmt_chunk::empty;
	// where the model has no data the chunk goes on level with its nearest point
	fill_from_nearest(heights, found, side);
	auto const height = [&](double const X, double const Z) {
		auto const ix{static_cast<std::size_t>(std::clamp<long>(std::lround((X - low.x) / step), 0, cells))};
		auto const iz{static_cast<std::size_t>(std::clamp<long>(std::lround((Z - low.y) / step), 0, cells))};
		return static_cast<float>(heights[iz * side + ix]);
	};
	if (false == m_streamer.exists(Key))
	{
		m_streamer.add_chunk(Key.first, Key.second, m_terrain_spacing, height);
		return nmt_chunk::made;
	}
	// a chunk of flat ground there already takes the heights of the model under the formations laid on it, which stay: the ground
	// under them shaped, not a touch-up over them
	auto const touchup{m_streamer.touch_up()};
	m_streamer.touch_up(false);
	m_streamer.reshape(low, high, [&](double const X, double const Z, float &Height) {
		Height = height(X, Z);
		return true;
	});
	m_streamer.touch_up(touchup);
	return nmt_chunk::made;
}

bool editor_mode::nmt_flat_chunk(heightmap::chunk_key const &Key) const
{
	auto const middle{heightmap::chunk_corner(Key) + glm::dvec2{0.5 * heightmap::chunk_size}};
	auto const *terrain{m_streamer.terrain_at(middle.x, middle.y)};
	if (terrain == nullptr || terrain->key() != Key)
		return false;
	auto const &data{terrain->data()};
	auto const &ground{data.base.empty() ? data.heights : data.base};
	if (ground.empty())
		return false;
	auto const [lowest, highest]{std::minmax_element(ground.begin(), ground.end())};
	return *highest - *lowest < 0.01f;
}

std::size_t editor_mode::nmt_shape_tracks(std::vector<TTrack *> const *Only)
{
	auto &state{m_nmt_state};
	if (false == ensure_terrain())
		return 0;
	auto const wanted = [&](TTrack const *Track) { return Only == nullptr || std::find(Only->begin(), Only->end(), Track) != Only->end(); };
	// one modifier per path: the formation under its axis and the slopes from its edges to the ground. laid over each other the
	// modifiers keep the highest of the slopes going down and the lowest of those going up, so tracks side by side share one
	// formation instead of a saw of their own ones
	// a path broken by a bridge, as over another track, gets a modifier for each piece on the ground
	std::map<std::tuple<TTrack *, int, int>, heightmap::modifier> modifiers;
	std::map<std::pair<TTrack *, int>, int> pieces;
	double constexpr step{5.0};
	std::pair<TTrack *, int> previous{nullptr, -1};
	bool broken{false};
	std::vector<nmt_place> over;
	for (auto const &place : nmt_track_places(step))
	{
		if (false == wanted(place.track))
			continue;
		std::pair<TTrack *, int> const path{place.track, place.path};
		if (path != previous)
			broken = false;
		previous = path;
		if (false == place.shapes)
		{
			broken = true;
			over.push_back(place);
			continue;
		}
		auto &piece{pieces[path]};
		if (broken)
		{
			++piece;
			broken = false;
		}
		auto &modifier{modifiers[{place.track, place.path, piece}]};
		if (modifier.name.empty())
		{
			modifier.name = "nmt " + formation_name(*place.track) + (place.track->m_paths.size() > 1 ? " " + std::to_string(place.path) : std::string{}) + (piece > 0 ? " #" + std::to_string(piece) : std::string{});
			modifier.slope = state.slope;
			modifier.reach = state.reach;
		}
		modifier.points.push_back({{place.position.x, place.position.z}, place.formation, place.half_width});
	}
	// a formation which ends by a bridge or a tunnel ends square against its abutment, its portal, whichever path goes on over it
	auto const walled = [&](heightmap::corridor_point const &End) {
		return std::any_of(over.begin(), over.end(), [&](nmt_place const &Place) {
			return glm::distance(glm::dvec2{Place.position.x, Place.position.z}, End.position) <= step * 1.5 && std::abs(Place.formation - End.formation) <= 1.0;
		});
	};
	std::vector<heightmap::modifier> laid;
	for (auto &[path, modifier] : modifiers)
		if (modifier.points.size() >= 2)
		{
			modifier.open_start = walled(modifier.points.front());
			modifier.open_end = walled(modifier.points.back());
			laid.push_back(std::move(modifier));
		}
	if (Only != nullptr)
	{
		// the formations as they are already aren't led into the terrain again
		auto const same = [](heightmap::modifier const &A, heightmap::modifier const &B) {
			return A.slope == B.slope && A.reach == B.reach && A.rounding == B.rounding && A.open_start == B.open_start && A.open_end == B.open_end && A.points.size() == B.points.size() &&
			       std::equal(A.points.begin(), A.points.end(), B.points.begin(), [](heightmap::corridor_point const &P, heightmap::corridor_point const &Q) {
				       return glm::distance(P.position, Q.position) < 1e-6 && std::abs(P.formation - Q.formation) < 1e-6 && std::abs(P.half_width - Q.half_width) < 1e-6;
			       });
		};
		auto const &existing{m_streamer.modifiers()};
		// the formations the tracks have, the ones as they are already too: those only aren't led again, nor taken out as gone
		std::unordered_set<std::string> kept;
		for (auto const &modifier : laid)
			kept.insert(modifier.name);
		std::erase_if(laid, [&](heightmap::modifier const &Modifier) {
			auto const found{std::find_if(existing.begin(), existing.end(), [&](heightmap::modifier const &Other) { return Other.name == Modifier.name; })};
			return found != existing.end() && same(*found, Modifier);
		});
		for (auto *track : *Only)
			state.changed.erase(track);
		// the modifiers the tracks don't have any more: of the pieces of a path gone, of the tracks removed
		std::vector<std::string> gone;
		for (auto const *track : *Only)
		{
			if (track == nullptr || track->name().empty())
				continue;
			auto const own{"nmt " + track->name()};
			for (auto const &modifier : m_streamer.modifiers())
				if ((modifier.name == own || modifier.name.starts_with(own + " ")) && false == kept.contains(modifier.name))
					gone.push_back(modifier.name);
		}
		m_streamer.remove_modifiers(gone);
	}
	if (Only == nullptr)
	{
		// the formations of the tracks there are no more, or are on bridges now, and of the tracks with no name laid over each
		// other under one name before
		std::vector<std::string> gone;
		for (auto const &modifier : m_streamer.modifiers())
			if (modifier.name.starts_with("nmt ") && std::none_of(laid.begin(), laid.end(), [&](heightmap::modifier const &Laid) { return Laid.name == modifier.name; }))
				gone.push_back(modifier.name);
		m_streamer.remove_modifiers(gone);
	}
	auto const count{laid.size()};
	m_streamer.modifiers(std::move(laid));
	forget_ground();
	if (Only == nullptr)
	{
		state.shaped = modifiers.size();
		state.changed.clear();
		state.gone.clear();
	}
	return count;
}

void editor_mode::nmt_area_terrain()
{
	auto &area{m_area_terrain};
	auto const &config{m_orthophoto.settings()};
	auto const &state{m_nmt_state};
	// what holds it, logged as it changes: nothing was made and nothing told why
	auto const held = [&](char const *Reason) {
		if (area.held != Reason)
		{
			area.held = Reason;
			WriteLog(std::string{"Editor: NMT - the terrain of the download area "} + (*Reason != '\0' ? std::string{"held: "} + Reason : std::string{"goes on"}), logtype::generic);
		}
	};
	if (false == area.run_only.value_or(EditorSettings.terrain_model().area))
		return held("turned off");
	if (m_orthophoto.area().empty())
		return held("no area");
	if (m_area_paused)
		return held("the area paused");
	if (config.north == 0.0 && config.east == 0.0)
		return held("no origin of the scenery");
	// after the work with the model the user asked for
	if (state.stage != nmt_state::automatic::idle || state.fitting || false == state.pending.empty())
		return held("other work with the model");
	// a stroke of the terrain brush keeps its step of the history open to its end: closed around a chunk, its undo would lose the
	// part brushed before
	if (terrain_brush() && mouseHold)
		return held("a stroke of the terrain brush");
	held("");
	// light: the time after a chunk set by how long it took. not held by slow frames: a large scenery has them all the time, and
	// nothing was made there
	auto const now{ImGui::GetTime()};
	if (now < area.resume)
		return;
	// the chunks of the area in the square around the camera, the nearest first
	int constexpr reach{4};
	auto const centre{heightmap::chunk_at(Camera.Pos.x, Camera.Pos.z)};
	if (false == area.listed || centre != area.centre)
	{
		area.keys.clear();
		area.next = 0;
		area.centre = centre;
		area.listed = true;
		area.waiting = -1.0;
		area.square_made = 0;
		area.took = 0.0;
		area.took_most = 0.0;
		for (int dz = -reach; dz <= reach; ++dz)
			for (int dx = -reach; dx <= reach; ++dx)
			{
				heightmap::chunk_key const key{centre.first + dx, centre.second + dz};
				if (auto const low{heightmap::chunk_corner(key)}; false == area.empty.contains(key) && in_download_area(low, low + glm::dvec2{heightmap::chunk_size}))
					area.keys.push_back(key);
			}
		std::stable_sort(area.keys.begin(), area.keys.end(), [&](heightmap::chunk_key const &Left, heightmap::chunk_key const &Right) {
			auto const distance = [&](heightmap::chunk_key const &Key) { return sq(Key.first - centre.first) + sq(Key.second - centre.second); };
			return distance(Left) < distance(Right);
		});
		auto const missing{std::count_if(area.keys.begin(), area.keys.end(), [&](heightmap::chunk_key const &Key) { return false == m_streamer.active() || false == m_streamer.exists(Key); })};
		// told only where there's terrain to make: the camera crosses a chunk every few seconds
		if (missing > 0)
			WriteLog(format("Editor: NMT - the download area around the camera (chunk %d,%d): %zu chunks in it, %zu with no terrain yet", centre.first, centre.second, area.keys.size(),
			                static_cast<std::size_t>(missing)),
			         logtype::generic);
	}
	// those there already cost a lookup only, a few of them a frame; the terrain made by hand is left as it is
	for (int looked = 0; looked < 8 && area.next < area.keys.size(); ++looked)
	{
		auto const key{area.keys[area.next]};
		if (m_streamer.active() && m_streamer.exists(key))
		{
			++area.next;
			continue;
		}
		if (false == ensure_terrain())
			return;
		nmt_locate();
		auto const low{heightmap::chunk_corner(key)};
		auto const high{low + glm::dvec2{heightmap::chunk_size}};
		// the model under it and a little around, for the heights along the terrain beside: read from the cache or downloaded
		// meanwhile; one that doesn't come is left for the camera's next move
		if (false == nmt_ready(low - glm::dvec2{2.0}, high + glm::dvec2{2.0}))
		{
			if (area.waiting < 0.0)
				area.waiting = now;
			else if (now - area.waiting > 5.0)
			{
				area.waiting = -1.0;
				++area.next;
			}
			return;
		}
		area.waiting = -1.0;
		++area.next;
		auto const started{std::chrono::steady_clock::now()};
		// the editor keeps a step of the history of the terrain open all the time: the chunk isn't a step of it, as the terrain grown
		// under the track laid; the step there is closed before, and a new one opened after
		auto const recording{m_streamer.recording()};
		if (recording)
		{
			terrain_history_step();
			m_streamer.end_change();
		}
		auto const chunk{add_chunk_from_nmt(key, nmt_chunk_offset(key))};
		if (recording)
			m_streamer.begin_change();
		auto const took{std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count()};
		// a chunk at most every 0.1 s, and the editor left at least three times as long for itself as the chunk took
		area.resume = now + std::max(0.1, 3.0 * took);
		area.took += took;
		area.took_most = std::max(area.took_most, took);
		// one outside the region of the scenery isn't added either
		if (chunk == nmt_chunk::made && m_streamer.exists(key))
		{
			++area.made;
			++area.square_made;
			area.refit = true;
			forget_ground_in(low, high);
		}
		else
			area.empty.insert(key);
		return;
	}
	// the square done: the photos laid onto the new ground once, not after each chunk
	if (area.next >= area.keys.size() && std::exchange(area.refit, false))
	{
		m_orthophoto.refit();
		for (auto &layer : m_wms)
			layer->refit();
		WriteLog(format("Editor: NMT - the download area: %zu chunks made around the camera in %.0f ms, the longest %.1f ms, %zu in all", area.square_made, area.took * 1000.0,
		                area.took_most * 1000.0, area.made),
		         logtype::generic);
	}
}

void editor_mode::nmt_update()
{
	auto &state{m_nmt_state};
	// the squares of the model in the download area to the disk cache, as the area promises the model with the photos: the area is in
	// PUWG 1992 already, so it needs neither the model placed (its panel opened) nor the terrain of the scenery begun
	if (auto const &wanted{m_orthophoto.area()}; wanted != m_nmt_prefetched)
	{
		m_nmt_prefetched = wanted;
		m_nmt.prefetch(wanted);
		// the terrain around the camera looked at again within the new area
		m_area_terrain.listed = false;
		m_area_terrain.empty.clear();
	}
	nmt_offer();
	nmt_follow_tracks();
	// the terrain of the download area, when nothing else is done with the model: it tells what holds it
	nmt_area_terrain();
	if (state.stage != nmt_state::automatic::idle)
	{
		nmt_auto_step();
		return;
	}
	if (state.fitting)
	{
		bool all{true};
		for (auto const &place : state.fit)
			all = nmt_ready(glm::dvec2{place.position.x, place.position.z} - glm::dvec2{1.0}, glm::dvec2{place.position.x, place.position.z} + glm::dvec2{1.0}) && all;
		if (all)
			nmt_finish_fit();
		return;
	}
	nmt_grow();
	if (state.pending.empty())
	{
		if (std::exchange(state.camera_check, false) && false == Global.EditorOrtho)
			camera_above_ground();
		return;
	}
	state.camera_check = true;
	// the terrain of the scenery opened, or begun, for the chunks grown under the track laid as well: they don't need terrain there before
	if (false == ensure_terrain())
	{
		state.pending.clear();
		state.refill.clear();
		state.status = STR("The heightmap terrain can't be opened");
		return;
	}
	// a few chunks a frame, those whose model is in. the terrain made ahead of the track laid isn't a step of the history:
	// Ctrl+Z takes back the track, not the ground under it
	auto const unrecorded{state.grown && m_streamer.recording()};
	if (unrecorded)
	{
		terrain_history_step();
		m_streamer.end_change();
	}
	int made{0};
	std::vector<heightmap::chunk_key> changed;
	for (auto it{state.pending.begin()}; it != state.pending.end() && made < 4;)
	{
		auto const refilled{state.refill.contains(*it)};
		if (false == refilled && m_streamer.exists(*it))
		{
			it = state.pending.erase(it);
			continue;
		}
		// the model under it and a little around first: the height along the terrain beside is read from it
		if (auto const low{heightmap::chunk_corner(*it)}; false == nmt_ready(low - glm::dvec2{2.0}, low + glm::dvec2{heightmap::chunk_size + 2.0}))
		{
			++it;
			continue;
		}
		// a new chunk at the height nmt_chunk_offset() gives it; one of flat ground refilled keeps to the offset of the scenery
		auto const offset{refilled ? std::nullopt : nmt_chunk_offset(*it)};
		auto const chunk{add_chunk_from_nmt(*it, offset)};
		if (chunk == nmt_chunk::waiting)
		{
			++it;
			continue;
		}
		// the height the track laid goes on at, over the model: from the terrain grown under it only, not from the buttons' chunks
		if (state.grown && offset.has_value() && chunk == nmt_chunk::made)
			state.ahead = offset;
		if (chunk == nmt_chunk::made)
			changed.push_back(*it);
		state.refill.erase(*it);
		it = state.pending.erase(it);
		++made;
		if (chunk == nmt_chunk::made)
			++state.made;
		else
			++state.empty;
	}
	// the last ones may have got here as chunks the streamer has read in meanwhile, nothing made: all waited for is there all the same,
	// the formations of the tracks led into it
	if (made > 0 || state.pending.empty())
	{
		// the ground gathered again only where the chunks were made: the rest of it is as it was, gathering all of it again frame after
		// frame as the terrain grows around the cursor was felt
		for (auto const &key : changed)
		{
			auto const low{heightmap::chunk_corner(key)};
			forget_ground_in(low, low + glm::dvec2{heightmap::chunk_size});
		}
		state.status = format(STR_C("Chunks made from the terrain model: %zu, waiting for it: %zu"), state.made, state.pending.size());
		if (state.empty > 0)
			state.status += format(STR_P("; %zu chunk not made, the model has no data there", "; %zu chunks not made, the model has no data there", state.empty), state.empty);
		if (state.foreign > 0)
			state.status += format(STR_P("; %zu chunk not made ahead, the terrain beside isn't the real one", "; %zu chunks not made ahead, the terrain beside isn't the real one", state.foreign), state.foreign);
		if (state.pending.empty())
		{
			state.run = false;
			// not after the chunks grown around the cursor: the streamer leads the formations there are into a chunk as it's made, and
			// shaping all of them again took the whole scenery through the terrain, 12 s of a frame each time the laying went on
			if (state.shape && state.reshape && false == state.grown)
			{
				nmt_shape_tracks();
				state.status += format(STR_P("; the formation of %zu path shaped", "; the formation of %zu paths shaped", state.shaped), state.shaped);
			}
			// the photos laid onto the new ground
			m_orthophoto.refit();
			for (auto &layer : m_wms)
				layer->refit();
			first_step_done(first_step::nmt);
		}
	}
	if (unrecorded)
		m_streamer.begin_change();
}

void editor_mode::nmt_grow()
{
	auto &state{m_nmt_state};
	auto const now{ImGui::GetTime()};
	if (false == state.grow || now - state.grow_time < 0.5)
		return;
	state.grow_time = now;
	// around the cursor of the tool laying the tracks, the terrain under them once they're laid: from the first point placed, or as a
	// track is dragged from a free end; not around the camera, nor with the tool merely chosen
	if (false == m_lay.active && false == m_extend.active)
		return;
	// the model placed where the origin of the orthophoto puts the scenery: it was only once the terrain panel had been open, and
	// nothing grew under the track laid before that
	nmt_locate();
	if (false == m_nmt.placed())
		return;
	auto const at{m_extend.active ? m_extend.mouse : m_lay.mouse};
	// nothing asked for outside the download area
	if (false == in_download_area({at.x - 100.0, at.z - 100.0}, {at.x + 100.0, at.z + 100.0}))
		return;
	// the model under it meanwhile, for the height of the first point laid where there's no terrain yet, as over the base map
	m_nmt.request({at.x - 100.0, at.z - 100.0}, {at.x + 100.0, at.z + 100.0});
	auto const laying{m_extend.active || false == m_lay.points.empty()};
	// also in a scenery with no terrain yet, and far from the terrain there is: the chunks are made where the track goes. a scenery
	// with no name yet can't have its terrain: nothing asked for, or it would be asked again and dropped every half a second
	if (false == laying || (false == m_streamer.active() && m_orthophoto_scenery.empty()))
		return;
	glm::dvec2 const cursor{at.x, at.z};
	double constexpr grow_reach{400.0}; // m around the cursor
	auto const reach{static_cast<int>(std::ceil(grow_reach / heightmap::chunk_size))};
	auto const centre{heightmap::chunk_at(at.x, at.z)};
	std::vector<std::pair<double, heightmap::chunk_key>> missing;
	for (int dz = -reach; dz <= reach; ++dz)
		for (int dx = -reach; dx <= reach; ++dx)
		{
			heightmap::chunk_key const key{centre.first + dx, centre.second + dz};
			auto const low{heightmap::chunk_corner(key)};
			glm::dvec2 const nearest{std::clamp(cursor.x, low.x, low.x + heightmap::chunk_size), std::clamp(cursor.y, low.y, low.y + heightmap::chunk_size)};
			auto const distance{glm::distance(nearest, cursor)};
			if (distance <= grow_reach && false == m_streamer.exists(key) && false == state.given_up.contains(key) && std::find(state.pending.begin(), state.pending.end(), key) == state.pending.end() &&
			    in_download_area(low, low + glm::dvec2{heightmap::chunk_size}))
				missing.emplace_back(distance, key);
		}
	if (missing.empty())
		return;
	std::sort(missing.begin(), missing.end());
	if (state.pending.empty())
	{
		state.grown = true;
		state.made = 0;
		state.empty = 0;
		state.foreign = 0;
	}
	for (auto const &chunk : missing)
		state.pending.push_back(chunk.second);
	if (false == m_orthophoto.enabled() && false == m_orthophoto_chosen)
		m_orthophoto.enabled(true);
	WriteLog(format("Editor: NMT - %zu chunks around the cursor to make", missing.size()), logtype::generic);
}

bool editor_mode::nmt_busy() const
{
	auto const &state{m_nmt_state};
	return state.fitting || state.stage != nmt_state::automatic::idle || (false == state.pending.empty() && false == state.grown);
}

void editor_mode::nmt_stop()
{
	auto &state{m_nmt_state};
	if (false == nmt_busy())
		return;
	auto const lines{state.stage == nmt_state::automatic::fitting && state.lines > 0};
	m_nmt.cancel();
	state.fitting = false;
	state.fit.clear();
	state.probes.clear();
	state.left.clear();
	state.done.clear();
	state.laid.clear();
	state.paths.clear();
	state.loose.clear();
	state.turntables.clear();
	state.stage = nmt_state::automatic::idle;
	state.pending.clear();
	state.refill.clear();
	state.run = false;
	// the lines fitted so far stay, and what they found is told
	state.result = state.result || lines;
	m_profile.open = false;
	state.status = format(STR_P("Stopped: %zu chunk made stays", "Stopped: %zu chunks made stay", state.made), state.made);
	if (lines)
		state.status += STR("; so do the grade lines fitted, Ctrl+Z takes them back");
	WriteLog("Editor: NMT - " + state.status, logtype::generic);
}

void editor_mode::render_nmt_progress(bool const Compact)
{
	auto &state{m_nmt_state};
	float fraction{0.0f};
	std::string text;
	// of the tracks put onto the terrain the step too: the model, the grade lines, the chunks
	int step{0};
	if (state.stage == nmt_state::automatic::fetching || state.fitting)
	{
		auto const waiting{m_nmt.waiting()};
		// the squares asked for come in one by one: how many of the most waited for are in; none waited for, only read from the disk
		state.fetch_most = std::max(state.fetch_most, waiting);
		fraction = state.fetch_most > 0 ? 1.0f - static_cast<float>(waiting) / static_cast<float>(state.fetch_most) : -1.0f * static_cast<float>(ImGui::GetTime());
		text = format(STR_P("Terrain model: %zu square left", "Terrain model: %zu squares left", waiting), waiting);
		step = 1;
	}
	else if (state.stage == nmt_state::automatic::fitting && state.tracks > 0)
	{
		fraction = 1.0f - static_cast<float>(state.left.size()) / static_cast<float>(std::max<std::size_t>(state.tracks, state.left.size()));
		text = format(STR_P("Grade lines: %zu fitted, %zu track left", "Grade lines: %zu fitted, %zu tracks left", state.left.size()), state.lines, state.left.size());
		step = 2;
	}
	else if (false == state.pending.empty())
	{
		auto const total{state.made + state.empty + state.pending.size()};
		fraction = static_cast<float>(state.made + state.empty) / static_cast<float>(std::max<std::size_t>(1, total));
		text = format(STR_C("Chunks: %zu of %zu"), state.made + state.empty, total);
		step = 3;
	}
	else
		return;
	if (state.run)
		text = format("%d/3  ", step) + text;
	auto const stoppable{nmt_busy()};
	auto const stop{STR_C("Stop")};
	auto const button{ImGui::CalcTextSize(stop).x + ImGui::GetStyle().FramePadding.x * 2.0f};
	auto const width{Compact ? std::round(ImGui::GetFontSize() * 16.0f) : stoppable ? -(button + ImGui::GetStyle().ItemSpacing.x) : -FLT_MIN};
	ImGui::ProgressBar(fraction, ImVec2(width, Compact ? 0.0f : tall_button_height()), text.c_str());
	if (false == stoppable)
		return;
	ImGui::SameLine();
	if (ImGui::Button(stop, ImVec2(0.0f, Compact ? 0.0f : tall_button_height())))
		nmt_stop();
	item_tooltip("Stops loading: the terrain made so far stays");
}

void editor_mode::nmt_pull_terrain(std::vector<TTrack *> const &Tracks)
{
	auto &state{m_nmt_state};
	std::vector<TTrack *> rails;
	for (auto *track : Tracks)
		if (track != nullptr && false == track->m_editorremoved && rail(*track))
			rails.push_back(track);
	if (rails.empty())
	{
		state.status = STR("No tracks to draw the terrain to");
		return;
	}
	if (false == ensure_terrain())
	{
		state.status = STR("The heightmap terrain can't be opened");
		return;
	}
	terrain_history_step();
	auto const laid{nmt_shape_tracks(&rails)};
	terrain_history_step();
	state.status = format(STR_P("The terrain drawn to %zu track: %zu formations led into it, Ctrl+Z takes it back", "The terrain drawn to %zu tracks: %zu formations led into it, Ctrl+Z takes it back", rails.size()), rails.size(), laid);
	WriteLog("Editor: NMT - " + state.status, logtype::generic);
}

void editor_mode::nmt_restore_terrain(std::vector<TTrack *> const &Tracks)
{
	auto &state{m_nmt_state};
	if (false == m_streamer.active())
		return;
	std::vector<std::string> gone;
	for (auto const *track : Tracks)
	{
		if (track == nullptr)
			continue;
		auto const own{"nmt " + formation_name(*track)};
		for (auto const &modifier : m_streamer.modifiers())
			if ((modifier.name == own || modifier.name.starts_with(own + " ")) && std::find(gone.begin(), gone.end(), modifier.name) == gone.end())
				gone.push_back(modifier.name);
	}
	terrain_history_step();
	m_streamer.remove_modifiers(gone);
	if (false == gone.empty())
		forget_ground();
	terrain_history_step();
	state.status = format(STR_P("%zu formation taken out of the terrain, Ctrl+Z takes it back", "%zu formations taken out of the terrain, Ctrl+Z takes it back", gone.size()), gone.size());
	WriteLog("Editor: NMT - " + state.status, logtype::generic);
}

void editor_mode::render_track_terrain(std::vector<TTrack *> const &Tracks)
{
	auto &state{m_nmt_state};
	// a scenery with no terrain made nor any place on the map has nothing to draw to its tracks
	if (false == m_streamer.active() && false == m_nmt.placed())
		return;
	if (false == ImGui::CollapsingHeader(format("%s###trackterrain", STR_C("Terrain")).c_str()))
		return;
	auto const busy{nmt_busy()};
	auto const spacing{ImGui::GetStyle().ItemSpacing.x};
	auto const half{std::floor((ImGui::GetContentRegionAvail().x - spacing) * 0.5f)};
	{
		std::size_t rails{0}, done{0};
		for (auto const *track : Tracks)
			if (track != nullptr && rail(*track))
			{
				++rails;
				if (grade_fitted(*track))
					++done;
			}
		if (rails == 1)
			ImGui::TextDisabled("%s", done > 0 ? STR_C("Its grade line is fitted to the terrain") : STR_C("Its grade line isn't fitted to the terrain yet"));
		else if (rails > 1)
			ImGui::TextDisabled(STR_C("Grade lines fitted to the terrain: %zu of %zu"), done, rails);
		item_tooltip("A track is fitted when its grade line was fitted to the terrain model or applied with the profile, and it wasn't changed since");
	}
	ImGui::BeginDisabled(busy);
	if (ImGui::Button(STR_C("Draw the terrain to the track"), ImVec2(half, 0.0f)))
		nmt_pull_terrain(Tracks);
	item_tooltip("The formation and the slopes of the selected tracks led into the terrain, down or up to it; elsewhere the terrain stays as it is");
	ImGui::SameLine();
	if (ImGui::Button(STR_C("The whole run"), ImVec2(-FLT_MIN, 0.0f)))
	{
		m_track_set = Tracks;
		track_set_grow();
		select_tracks(m_track_set);
		nmt_pull_terrain(m_track_set);
	}
	item_tooltip("The tracks joined to the selected ones added to the selection, by the rules of the spread, and the terrain drawn to all of them");
	if (ImGui::Button(STR_C("Restore the terrain"), ImVec2(half, 0.0f)))
		nmt_restore_terrain(Tracks);
	item_tooltip("The formations of the selected tracks taken out of the terrain: the ground goes back to how it was before them");
	ImGui::EndDisabled();
	toggle_chips("nmtfollow", {{STR_C("The terrain follows the tracks edited"), &state.follow, "A track with its formation moved or changed takes it along; off: the terrain stays as it is until it's drawn to the tracks again"}});
	ImGui::Separator();
	ImGui::TextDisabled("%s", STR_C("All the tracks of the scenery"));
	render_nmt_all_tracks(false);
	if (busy)
		render_nmt_progress(true);
	else if (false == state.status.empty())
	{
		ImGui::PushTextWrapPos(0.0f);
		ImGui::TextDisabled("%s", state.status.c_str());
		ImGui::PopTextWrapPos();
	}
}

// the tracks of the whole scenery and the terrain model: the way to take first big, what it does told under it, then the other
// two, each with the case it's for; step 3 of the terrain model in the field of the terrain (Terrain_panel), and in the panel of
// the tracks
void editor_mode::render_nmt_all_tracks(bool const Terrain_panel)
{
	auto &state{m_nmt_state};
	auto const busy{nmt_busy()};
	auto const placed{m_nmt.placed()};
	std::size_t unfitted{0}, all{0}, hand{0};
	std::unordered_map<std::string, int> named;
	for (auto const *track : simulation::Paths.sequence())
		if (track != nullptr && false == track->m_editorremoved && rail(*track))
		{
			++all;
			++named[track->name()];
			if (false == grade_fitted(*track))
				++unfitted;
			else if (grade_hand(*track))
				++hand;
		}
	auto const told{[](char const *Text) {
		ImGui::PushTextWrapPos(0.0f);
		ImGui::TextDisabled("%s", Text);
		ImGui::PopTextWrapPos();
	}};
	if (false == placed)
	{
		ImGui::PushTextWrapPos(0.0f);
		ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), "%s", STR_C("First the place of the scenery on the map: F5 Terrain, Terrain model (NMT), step 1"));
		ImGui::PopTextWrapPos();
	}
	// three ways as cards, each with a sketch and the case it's for, the one to take first lit: what to click is seen at a glance.
	// The tracks fitted already stay as they are: a track laid or changed since goes onto the model, all of them only when asked
	auto const ready{false == busy && placed};
	auto const onto_text{unfitted == all ? std::string{STR_C("Recommended: the tracks go down or up onto the real terrain, each line on a grade line within its norms, with the formation and the slopes")}
	                     : unfitted > 0  ? format(STR_C("The tracks laid or changed since the last time go onto the terrain model (%zu of %zu); the ones fitted stay as they are"), unfitted, all)
	                                     : std::string{STR_C("All the tracks are fitted to the terrain; a track changed since goes onto it again")}};
	if (way_card("##nmtonto", STR_C("Tracks onto the terrain"), onto_text.c_str(), way_sketch::onto, true, ready && unfitted > 0))
		nmt_auto_start(true);
	item_tooltip("The terrain model is loaded along the tracks and the tracks go onto it, wherever the scenery was built: the top of the ballast at the height of the model, the grade line of every line fitted to it within the limits of the line, switches and the tracks beside them on one grade line, the formation and the slopes made along them. Where the model is off the tracks keep their line and it's reported. A turntable stays level in its pit at the height of the tracks running up to it, which reach it level. The tracks fitted already, by it or with the profile, and not changed since stay as they are, and the new lines meet them");
	// the fitted ones too, and which are left after it: under the card they belong to
	ImGui::BeginDisabled(false == ready || all == 0);
	if (unfitted < all && ImGui::SmallButton(STR_C("All anew, the fitted ones too")))
		ImGui::OpenPopup("##nmtallanew");
	item_tooltip("All the tracks are fitted to the terrain model again, the ones fitted already too: their grade lines may change; the scenery isn't moved");
	if (ImGui::BeginPopup("##nmtallanew"))
	{
		ImGui::TextUnformatted(format(STR_C("%zu tracks get new grade lines; Ctrl+Z takes it back"), all - (state.redo_hand ? 0 : hand)).c_str());
		if (hand > 0)
			ImGui::Checkbox(format(STR_C("The %zu edited by hand too"), hand).c_str(), &state.redo_hand);
		if (ImGui::Button(STR_C("Fit them anew")))
		{
			nmt_auto_start(false);
			state.redo_hand = false;
			ImGui::CloseCurrentPopup();
		}
		ImGui::SameLine();
		if (ImGui::Button(STR_C("Cancel")))
			ImGui::CloseCurrentPopup();
		ImGui::EndPopup();
	}
	ImGui::EndDisabled();
	if (unfitted > 0 && unfitted < all)
	{
		ImGui::SameLine();
		// which they are, a few after the putting onto the terrain: the ones it couldn't fit
		if (ImGui::SmallButton(format(STR_P("Show the %zu track not fitted", "Show the %zu tracks not fitted", unfitted), unfitted).c_str()))
		{
			std::vector<TTrack *> those;
			for (auto *track : simulation::Paths.sequence())
				if (track != nullptr && rail(*track) && false == grade_fitted(*track))
					those.push_back(track);
			if (false == those.empty())
			{
				select_tracks(those);
				focus_track(*those.front(), 0, geometry::bezier{those.front()->m_paths.front()}.point(0.5));
			}
		}
		item_tooltip("They are selected, the first one in the 3d view");
	}
	ImGui::Spacing();
	if (way_card("##nmtunder", STR_C("Terrain under the tracks"), STR_C("The tracks stay at their heights and the terrain comes up or down to them: for a scenery built level or at heights of its own"), way_sketch::under, false, ready && all > 0))
		nmt_under_start();
	item_tooltip("The tracks aren't touched: the terrain model is loaded along them and drawn to them, at each place by as much as the tracks lie above or below it there, fading out away from them, so its relief stays and the tracks don't end up in deep cuttings; then the formation and the slopes along them. For a scenery built level or at its own heights");
	ImGui::Spacing();
	if (way_card("##nmtonly", STR_C("The terrain of the model only"), STR_C("The real terrain along the tracks as it is; the tracks stay where they are, under it or over it"), way_sketch::only, false, ready))
		nmt_make(nmt_keys_along_tracks());
	item_tooltip("Loads the terrain model as it is along the tracks, with the height offset above; the tracks stay where they are");
	// under the three ways, which stay seen together: what the last of them did
	render_nmt_result();
	std::string twice;
	std::size_t repeated{0};
	for (auto const &[name, count] : named)
		if (count > 1 && false == name.empty() && name != "none")
		{
			if (++repeated <= 12)
				twice += (twice.empty() ? "" : ", ") + name;
		}
	if (repeated > 0)
	{
		told(format(STR_P("%zu track name is used more than once in the scenery", "%zu track names are used more than once in the scenery", repeated), repeated).c_str());
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s%s", twice.c_str(), repeated > 12 ? ", ..." : "");
	}
	if (hand > 0)
		told(format(STR_P("%zu track with its grade line edited by hand stays as it is", "%zu tracks with their grade lines edited by hand stay as they are", hand), hand).c_str());
	if (Terrain_panel)
	{
		ImGui::Spacing();
		told(STR_C("The terrain to some of the tracks only: select them (F3), Terrain in the inspector"));
	}
}

void editor_mode::nmt_note(nmt_state::finding::kind const What, TTrack *From, TTrack *To, double const Length, glm::dvec3 const &Where, std::string Text, double const Size)
{
	// only of the tracks put onto the terrain all together: a track laid onto it is told of as it's laid
	if (m_nmt_state.stage != nmt_state::automatic::fitting)
		return;
	m_nmt_state.findings.push_back({What, From, To, Length, Where, std::move(Text), Size});
}

void editor_mode::nmt_show(nmt_state::finding const &Finding, bool const Profile)
{
	double constexpr reach{10.0}; // m, a track this near the place is the one of the line, if it was split since
	int path{0};
	auto *track{kilometrage::track_near(Finding.where, reach, path)};
	if (track != nullptr)
	{
		select_track(track);
		focus_track(*track, path, Finding.where);
	}
	else
		fly_to(Finding.where, glm::normalize(glm::dvec3{0.0, -0.5, 1.0}), 40.0);
	if (false == Profile)
		return;
	show_track_tab(track_tab::profile);
	auto const alive = [](TTrack const *Track) { return Track != nullptr && false == Track->m_editorremoved; };
	if (alive(Finding.from) && alive(Finding.to))
		profile_open(Finding.from, Finding.to);
	else if (track != nullptr)
		profile_open_run(*track);
}

void editor_mode::render_nmt_result()
{
	auto &state{m_nmt_state};
	if (false == state.result || nmt_busy())
		return;
	using kind = nmt_state::finding::kind;
	auto const count = [&](kind const What) { return static_cast<std::size_t>(std::count_if(state.findings.begin(), state.findings.end(), [&](nmt_state::finding const &Finding) { return Finding.what == What; })); };
	ImVec4 const good{0.45f, 0.85f, 0.45f, 1.0f};
	ImVec4 const bad{1.0f, 0.45f, 0.35f, 1.0f};
	ImVec4 const doubt{1.0f, 0.75f, 0.3f, 1.0f};
	ImVec4 const calm{0.6f, 0.62f, 0.66f, 1.0f};
	auto const refused{count(kind::refused)};
	auto const off{count(kind::off)};
	auto const parted{count(kind::parted)};
	ImGui::Spacing();
	ImGui::TextUnformatted(STR_C("What the last putting onto the terrain did"));
	metric_tiles({{std::to_string(state.lines), STR_C("lines fitted"), ImGui::ColorConvertFloat4ToU32(good)},
	              {std::to_string(refused), STR_C("refused"), ImGui::ColorConvertFloat4ToU32(refused > 0 ? bad : calm)},
	              {std::to_string(off), STR_C("off the line"), ImGui::ColorConvertFloat4ToU32(off > 0 ? doubt : calm)},
	              {std::to_string(parted), STR_C("apart from the model"), ImGui::ColorConvertFloat4ToU32(parted > 0 ? doubt : calm)}});
	if (state.levelled > 0)
		ImGui::TextDisabled(STR_C("Turntables set level: %zu"), state.levelled);
	if (state.findings.empty())
	{
		ImGui::TextColored(good, "%s", STR_C("Nothing to look at: every line lies on the model"));
		return;
	}
	// the refused first, then the paths off their lines, then the lines apart from the model, the farther the sooner
	std::vector<std::size_t> order(state.findings.size());
	for (std::size_t i = 0; i < order.size(); ++i)
		order[i] = i;
	std::stable_sort(order.begin(), order.end(), [&](std::size_t const A, std::size_t const B) {
		auto const &a{state.findings[A]};
		auto const &b{state.findings[B]};
		return a.what != b.what ? a.what < b.what : a.size > b.size;
	});
	auto const rows{std::min<float>(7.0f, static_cast<float>(order.size()))};
	auto const row{ImGui::GetTextLineHeightWithSpacing() * 2.0f};
	ImGui::TextDisabled("%s", STR_C("To look at, each with its place and its profile:"));
	ImGui::BeginChild("##nmtfindings", ImVec2(0.0f, std::round(row * rows + ImGui::GetStyle().WindowPadding.y * 2.0f)), true);
	auto const show{STR_C("Show")};
	auto const profile{STR_C("Profile")};
	ImGuiListClipper clipper;
	clipper.Begin(static_cast<int>(order.size()), row);
	std::optional<std::pair<std::size_t, bool>> clicked;
	while (clipper.Step())
		for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; ++i)
		{
			auto const &finding{state.findings[order[i]]};
			ImGui::PushID(static_cast<int>(order[i]));
			// the buttons first, where the long names of the tracks don't push them out of the panel
			if (ImGui::SmallButton(show))
				clicked = {order[i], false};
			item_tooltip("The place in the 3d view, the track there selected");
			ImGui::SameLine();
			if (ImGui::SmallButton(profile))
				clicked = {order[i], true};
			item_tooltip("The profile of the line opened: its grade line, the terrain model under it and what the checks say");
			ImGui::SameLine();
			auto const colour{finding.what == kind::refused ? bad : doubt};
			auto const *label{finding.what == kind::refused ? STR_C("refused") : finding.what == kind::off ? STR_C("off the line") : STR_C("apart from the model")};
			ImGui::TextColored(colour, "%s", label);
			ImGui::SameLine();
			ImGui::TextUnformatted(finding.text.c_str());
			if (ImGui::IsItemHovered() && false == finding.text.empty())
				ImGui::SetTooltip("%s", finding.text.c_str());
			auto const alive = [](TTrack const *Track) { return Track != nullptr && false == Track->m_editorremoved; };
			auto const ends{alive(finding.from) && alive(finding.to) ? (finding.from == finding.to ? finding.from->name() : finding.from->name() + " .. " + finding.to->name()) : std::string{}};
			ImGui::TextDisabled("%s", format(STR_C("the line %s, %.0f m"), ends.c_str(), finding.length).c_str());
			ImGui::PopID();
		}
	ImGui::EndChild();
	if (clicked)
		nmt_show(state.findings[clicked->first], clicked->second);
}

void editor_mode::draw_nmt_pending() const
{
	auto const &state{m_nmt_state};
	auto const imagery{m_orthophoto.waiting()};
	if (state.pending.empty() && imagery.empty())
		return;
	// where the ground will be: by the terrain there or beside it, else the model with the offset of the scenery, else level
	// with the nearest chunk there is, or the nearest track
	glm::dvec2 const camera{Camera.Pos.x, Camera.Pos.z};
	double fallback{0.0};
	{
		auto nearest{std::numeric_limits<double>::max()};
		std::vector<editor_terrain *> chunks;
		m_streamer.collect(chunks);
		for (auto const *chunk : chunks)
			if (auto const middle{chunk->centre()}; glm::distance(glm::dvec2{middle.x, middle.z}, camera) < nearest)
			{
				nearest = glm::distance(glm::dvec2{middle.x, middle.z}, camera);
				fallback = middle.y;
			}
		if (chunks.empty())
			for (auto const *track : simulation::Paths.sequence())
				if (track != nullptr && false == track->m_paths.empty())
					if (auto const &point{track->m_paths.front().points[segment_data::point::start]}; glm::distance(geometry::plan_of(point), camera) < nearest)
					{
						nearest = glm::distance(geometry::plan_of(point), camera);
						fallback = point.y;
					}
	}
	auto const height = [&](double const X, double const Z, double &Height) {
		if (m_streamer.height_at(X, Z, Height) || nmt_ground(X, Z, Height))
			return true;
		Height = fallback;
		return true;
	};
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList()};
	auto const time{ImGui::GetTime()};
	auto const pulse{static_cast<float>(0.5 + 0.5 * std::sin(time * 4.0))};
	auto const dots{std::string(static_cast<std::size_t>(time * 3.0) % 4, '.')};
	int constexpr steps{8}; // points along each edge, the outline following the ground
	auto const square = [&](glm::dvec2 const &Low, glm::dvec2 const &High, int const Red, int const Green, int const Blue, std::string const &Label) {
		auto const size{High - Low};
		std::vector<ImVec2> outline;
		for (int side = 0; side < 4; ++side)
			for (int i = 0; i < steps; ++i)
			{
				auto const along{static_cast<double>(i) / steps};
				glm::dvec2 const point{side == 0 ? Low + glm::dvec2{size.x * along, 0.0} : side == 1 ? Low + glm::dvec2{size.x, size.y * along} : side == 2 ? High - glm::dvec2{size.x * along, 0.0} : High - glm::dvec2{size.x, size.y * along}};
				double y;
				ImVec2 screen;
				if (false == height(point.x, point.y, y) || false == projection.project({point.x, y + 0.5, point.y}, screen))
					return;
				outline.push_back(screen);
			}
		auto const edge{IM_COL32(Red, Green, Blue, static_cast<int>(140.0f + 100.0f * pulse))};
		drawlist->AddConvexPolyFilled(outline.data(), static_cast<int>(outline.size()), IM_COL32(Red, Green, Blue, static_cast<int>(10.0f + 15.0f * pulse)));
		drawlist->AddPolyline(outline.data(), static_cast<int>(outline.size()), edge, ImDrawFlags_Closed, 2.0f);
		// named where the name fits in
		auto const middle{(Low + High) * 0.5};
		auto const text{Label + dots};
		auto const extent{ImGui::CalcTextSize(text.c_str())};
		auto const [left, right]{std::minmax_element(outline.begin(), outline.end(), [](ImVec2 const &A, ImVec2 const &B) { return A.x < B.x; })};
		double y;
		ImVec2 label;
		if (right->x - left->x > extent.x * 1.1f && height(middle.x, middle.y, y) && projection.project({middle.x, y + 0.5, middle.y}, label))
			drawlist->AddText({label.x - extent.x * 0.5f, label.y - extent.y * 0.5f}, edge, text.c_str());
	};
	// the nearest first, as many as can be drawn without the frame feeling it
	auto const nearest = [&](std::vector<std::pair<glm::dvec2, glm::dvec2>> Areas, std::size_t const Most) {
		auto const away = [&](std::pair<glm::dvec2, glm::dvec2> const &Area) { return glm::distance((Area.first + Area.second) * 0.5, camera); };
		if (Areas.size() > Most)
		{
			std::nth_element(Areas.begin(), Areas.begin() + Most, Areas.end(), [&](auto const &A, auto const &B) { return away(A) < away(B); });
			Areas.resize(Most);
		}
		return Areas;
	};
	for (auto const &[low, high] : nearest(imagery, 64))
		square(low, high, 255, 190, 70, STR("Orthophoto loading"));
	std::vector<std::pair<glm::dvec2, glm::dvec2>> chunks;
	for (auto const &key : state.pending)
		chunks.emplace_back(heightmap::chunk_corner(key), heightmap::chunk_corner(key) + glm::dvec2{heightmap::chunk_size});
	for (auto const &[low, high] : nearest(chunks, 64))
		square(low, high, 90, 170, 255, STR("Terrain model loading"));
}

// ---------------------------------------------------------------------------------------------
// the download area

void editor_mode::area_start()
{
	auto &drawing{m_area_drawing};
	drawing = {};
	drawing.active = true;
	// the area there is taken up to be changed
	for (auto const &point : m_orthophoto.area())
		drawing.points.push_back(m_orthophoto.to_world(point));
	// drawn over the orthophoto of the top view, unless the user turned it off
	if (false == m_orthophoto.enabled() && false == m_orthophoto_chosen)
		m_orthophoto.enabled(true);
	if (false == Global.EditorOrtho)
		toggle_ortho();
	WriteLog("Editor: drawing the download area");
}

void editor_mode::area_finish(bool const Keep)
{
	auto &drawing{m_area_drawing};
	if (Keep && drawing.points.size() >= 3)
	{
		std::vector<glm::dvec2> polygon;
		for (auto const &point : drawing.points)
			polygon.push_back(m_orthophoto.to_puwg(point));
		m_orthophoto.area(std::move(polygon));
		WriteLog(format("Editor: the download area of %zu points, %d tiles of the orthophoto", drawing.points.size(), m_orthophoto.stats().area_tiles));
	}
	drawing = {};
}

glm::dvec3 editor_mode::area_point(glm::dvec2 const &Point) const
{
	double height{Camera.Pos.y - 2.0};
	if (false == Global.EditorOrtho && false == m_streamer.height_at(Point.x, Point.y, height) && false == nmt_ground(Point.x, Point.y, height))
		height = 0.0;
	return {Point.x, height, Point.y};
}

void editor_mode::area_click(bool const Shift)
{
	auto &drawing{m_area_drawing};
	screen_projection const projection;
	// the point under the cursor, within 10 pixels
	int nearest{-1};
	auto best{100.0f};
	for (std::size_t i = 0; i < drawing.points.size(); ++i)
		if (auto const distance{projection.mouse_distance2(area_point(drawing.points[i]))}; distance < best)
		{
			best = distance;
			nearest = static_cast<int>(i);
		}
	if (nearest >= 0)
	{
		if (Shift)
			drawing.points.erase(drawing.points.begin() + nearest);
		else
			drawing.dragged = nearest;
		return;
	}
	if (Shift)
		return;
	auto const cursor{std::as_const(*this).cursor_ground()};
	drawing.points.emplace_back(cursor.x, cursor.z);
}

void editor_mode::area_update()
{
	auto &drawing{m_area_drawing};
	if (false == drawing.active || drawing.dragged < 0)
		return;
	if (false == ImGui::IsMouseDown(0) || drawing.dragged >= static_cast<int>(drawing.points.size()))
	{
		drawing.dragged = -1;
		return;
	}
	auto const cursor{std::as_const(*this).cursor_ground()};
	drawing.points[drawing.dragged] = {cursor.x, cursor.z};
}

void editor_mode::draw_download_area() const
{
	auto const &drawing{m_area_drawing};
	std::vector<glm::dvec2> points;
	if (drawing.active)
		points = drawing.points;
	else
		for (auto const &point : m_orthophoto.area())
			points.push_back(m_orthophoto.to_world(point));
	// shown while it's drawn and with the settings of the orthophoto open
	if (points.empty() || (false == drawing.active && false == orthophoto_settings_open()))
		return;
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList()};
	// the tiles of the area which failed: those in the cache or waiting left clear, the green over all the cached ones got in the way
	if (false == drawing.active && orthophoto_settings_open())
		for (auto const &square : m_orthophoto.area_squares())
		{
			if (square.state != 2)
				continue;
			std::array<ImVec2, 4> corners;
			auto seen{true};
			glm::dvec2 const at[]{square.low, {square.high.x, square.low.y}, square.high, {square.low.x, square.high.y}};
			for (int i = 0; i < 4 && seen; ++i)
				seen = projection.project(area_point(at[i]), corners[i]);
			if (seen)
				drawlist->AddQuadFilled(corners[0], corners[1], corners[2], corners[3], IM_COL32(240, 80, 70, 80));
		}
	auto const edge{IM_COL32(80, 220, 255, 230)};
	auto const closing{IM_COL32(80, 220, 255, drawing.active ? 110 : 230)};
	// the edge back to the first point from three points on
	auto const edges{points.size() >= 3 ? points.size() : points.size() - 1};
	for (std::size_t i = 0; i < edges; ++i)
	{
		auto const last{i + 1 == points.size()};
		projection.line(drawlist, area_point(points[i]), area_point(points[(i + 1) % points.size()]), last ? closing : edge, 2.0f);
	}
	if (false == drawing.active)
		return;
	// the points, the one under the cursor larger, and the line on to the cursor
	ImVec2 screen;
	for (auto const &point : points)
		if (projection.project(area_point(point), screen))
			drawlist->AddCircleFilled(screen, projection.mouse_distance2(area_point(point)) < 100.0f ? 6.0f : 4.0f, edge);
	if (false == points.empty() && drawing.dragged < 0 && false == ImGui::GetIO().WantCaptureMouse)
	{
		auto const cursor{std::as_const(*this).cursor_ground()};
		projection.line(drawlist, area_point(points.back()), area_point({cursor.x, cursor.z}), closing, 1.5f);
	}
}

void editor_mode::area_square(int const Side)
{
	auto const centre{m_orthophoto.to_puwg({Camera.Pos.x, Camera.Pos.z})};
	auto const half{Side * 500.0};
	m_orthophoto.area({{centre.x - half, centre.y - half}, {centre.x + half, centre.y - half}, {centre.x + half, centre.y + half}, {centre.x - half, centre.y + half}});
	WriteLog(format("Editor: the download area a square of %d km around the camera, %d tiles of the orthophoto", Side, m_orthophoto.stats().area_tiles));
}

bool editor_mode::in_download_area(glm::dvec2 const &Low, glm::dvec2 const &High) const
{
	auto const &area{m_orthophoto.area()};
	if (area.empty())
		return true;
	// the square in PUWG: the easting grows to the west of the scenery, the northing with z
	auto const &config{m_orthophoto.settings()};
	return geometry::square_touches(area, config.east - High.x, config.north + Low.y, High.x - Low.x);
}

void editor_mode::render_download_area_ui(editor_orthophoto::statistics const &Stats)
{
	auto &drawing{m_area_drawing};
	ImGui::TextUnformatted(STR_C("Download area"));
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C("A polygon drawn on the map: what the settings ask for inside it, the orthophoto and the terrain model,\n"
		                              "goes to the disk cache in the background, and nothing outside it is downloaded, also around the camera"));
	if (drawing.active)
	{
		ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.3f, 1.0f), "%s", STR_C("Click the points of the area on the map. Enter closes it, Backspace takes the last point back, Esc gives up;\n"
		                                                              "a point dragged moves, Shift+click removes it"));
		ImGui::Text(STR_C("%zu points"), drawing.points.size());
		ImGui::SameLine();
		if (drawing.points.size() >= 3 && ImGui::Button(STR_C("Done")))
			area_finish(true);
		ImGui::SameLine();
		if (ImGui::Button(STR_C("Cancel")))
			area_finish(false);
		return;
	}
	auto const &area{m_orthophoto.area()};
	ImGui::PushTextWrapPos(0.0f);
	ImGui::TextDisabled("%s", area.empty() ? STR_C("None: the photos are downloaded around the camera, wherever it goes")
	                                       : STR_C("The aerial photos and the shape of the ground are downloaded only inside it; outside it there are none"));
	ImGui::PopTextWrapPos();
	if (ImGui::Button(area.empty() ? STR_C("Draw on the map") : STR_C("Draw again")))
		area_start();
	if (false == area.empty())
	{
		ImGui::SameLine();
		if (ImGui::Button(STR_C("Remove")))
			m_orthophoto.area({});
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("No range: the photos around the camera, wherever it goes. The photos downloaded already stay on the disk"));
	}
	// a square around the camera, in place of the one drawn: what the wizard of a new scenery gives. Where the scenery lies on
	// the map must be known for it, or the square would lie anywhere
	auto const &config{m_orthophoto.settings()};
	auto const located{config.north != 0.0 || config.east != 0.0};
	ImGui::BeginDisabled(false == located);
	ImGui::AlignTextToFramePadding();
	ImGui::TextUnformatted(STR_C("Square around the camera:"));
	for (auto const side : {1, 2, 5, 10})
	{
		ImGui::SameLine();
		if (ImGui::SmallButton(format(STR_C("%d x %d km##areasquare"), side, side).c_str()))
		{
			// the area there is, drawn with care, or one this large, replaced only with a yes
			m_area_square = side;
			if (area.empty() && side < 10)
				area_square(side);
			else
				ImGui::OpenPopup("##areasquare");
		}
		if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
			ImGui::SetTooltip("%s", located ? format(STR_C("The range of the photos a square of %d x %d km, the camera in its middle"), side, side).c_str() : STR_C("Enter the origin of this scenery first"));
	}
	ImGui::EndDisabled();
	if (ImGui::BeginPopup("##areasquare"))
	{
		auto const size{static_cast<double>(m_area_square) * m_area_square};
		ImGui::Text(STR_C("The range of the photos a square of %d x %d km around the camera?"), m_area_square, m_area_square);
		if (false == area.empty())
			ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), "%s", STR_C("It replaces the area there is now, which can't be taken back"));
		if (size >= 50.0)
			ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), STR_C("%.0f sq km: the download takes long and needs a lot of disk space"), size);
		if (ImGui::Button(STR_C("Replace")))
		{
			area_square(m_area_square);
			ImGui::CloseCurrentPopup();
		}
		ImGui::SameLine();
		if (ImGui::Button(STR_C("Cancel")))
			ImGui::CloseCurrentPopup();
		ImGui::EndPopup();
	}
	if (area.empty())
		return;
	ImGui::Text(STR_C("Area: %d of %d tiles in the cache, %d failed"), Stats.area_cached, Stats.area_tiles, Stats.area_failed);
	ImGui::SameLine();
	if (ImGui::SmallButton(m_area_paused ? STR_C("Resume") : STR_C("Pause")))
	{
		m_area_paused = false == m_area_paused;
		m_orthophoto.area_paused(m_area_paused);
		m_nmt.prefetch_paused(m_area_paused);
	}
	if (auto const terrain{m_nmt.prefetched()}; terrain.total > 0)
	{
		ImGui::Text(STR_C("Terrain model: %d of %d squares in the cache, %d failed"), terrain.cached, terrain.total, terrain.failed);
		// the ones there are only looked up again
		if (terrain.failed > 0)
		{
			ImGui::SameLine();
			if (ImGui::SmallButton((std::string{STR_C("Retry")} + "##terrainarea").c_str()))
				m_nmt.prefetch(m_nmt_prefetched);
		}
	}
	auto &model{EditorSettings.terrain_model()};
	if (ImGui::Checkbox(STR_C("The terrain of the scenery around the camera"), &model.area))
		EditorSettings.save();
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C("Makes the terrain from the terrain model within the area, around the camera wherever it goes,\n"
		                              "a chunk at a time while the editor is idle. The terrain there is already isn't touched.\n"
		                              "The chunks made wait in memory until the scenery is saved (Ctrl+S)."));
	if (model.area && m_area_terrain.made > 0)
	{
		ImGui::SameLine();
		ImGui::TextDisabled("%s", format(STR_P("%zu chunk made", "%zu chunks made", m_area_terrain.made), m_area_terrain.made).c_str());
	}
}

void editor_mode::render_nmt_banner()
{
	auto const &state{m_nmt_state};
	if (false == nmt_busy() || state.shown >= ImGui::GetFrameCount() - 1)
		return;
	auto const low{editor_ui::view_min()};
	auto const high{editor_ui::view_max()};
	ImGui::SetNextWindowPos(ImVec2((low.x + high.x) * 0.5f, low.y + std::round(ImGui::GetFontSize() * 3.6f)), ImGuiCond_Always, ImVec2(0.5f, 0.0f));
	ImGui::SetNextWindowViewport(ImGui::GetMainViewport()->ID);
	ImGui::SetNextWindowBgAlpha(0.88f);
	if (ImGui::Begin("##nmtbanner", nullptr,
	                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoDocking))
	{
		ImGui::AlignTextToFramePadding();
		ImGui::TextUnformatted(STR_C("Loading the real terrain"));
		ImGui::SameLine();
		render_nmt_progress(true);
		bool again{false};
		if (auto const error{m_nmt.error(&again)}; false == error.empty())
		{
			ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f), "%s", STR(error).c_str());
			if (again)
			{
				ImGui::SameLine();
				if (ImGui::SmallButton(STR_C("Try again")))
					m_nmt.retry();
			}
		}
	}
	ImGui::End();
}

void editor_mode::nmt_check_buried(std::size_t const From)
{
	auto &state{m_nmt_state};
	state.buried.clear();
	state.buried_length = 0.0;
	if (false == EditorSettings.terrain_buried_ask() || false == m_streamer.active())
		return;
	// the top of the rails this near the ground leaves the sleepers and the ballast under it
	double constexpr clearance{0.1};
	double constexpr step{5.0};
	double constexpr noticed{1.0};
	std::vector<std::string> laid;
	std::unordered_set<TTrack const *> seen;
	double hidden{0.0};
	for (auto index{From}; index < m_history.size(); ++index)
		for (auto const *track : m_history[index].created)
		{
			if (track == nullptr || track->m_editorremoved || false == rail(*track) || false == seen.insert(track).second)
				continue;
			laid.push_back(track->name());
			auto const &path{track->m_paths.front()};
			geometry::bezier const curve{path};
			auto const length{geometry::chord_length(path)};
			auto const count{std::max(1, static_cast<int>(std::ceil(length / step)))};
			std::vector<glm::dvec3> along, probes;
			for (int i = 0; i < count; ++i)
			{
				along.push_back(curve.point((i + 0.5) / count));
				probes.emplace_back(along.back().x, 5000.0, along.back().z);
			}
			std::vector<char> found;
			auto const ground{ground_heights(probes, true, &found)};
			for (int i = 0; i < count; ++i)
				if (found[i] != 0 && along[i].y < ground[i] + clearance)
					hidden += length / count;
		}
	if (hidden < noticed)
		return;
	state.buried = std::move(laid);
	state.buried_length = hidden;
}

void editor_mode::render_nmt_buried()
{
	auto &state{m_nmt_state};
	if (state.buried.empty())
		return;
	// gone on to other work the message would only be in the way: the same is done by Terrain in the inspector
	if (current_work_area() == work_area::terrain || (ui()->mode() == nodebank_panel::TRACK && m_track_tab != track_tab::lay && m_track_tab != track_tab::path))
	{
		state.buried.clear();
		return;
	}
	// taken back with Ctrl+Z there's nothing to tell of
	std::vector<TTrack *> tracks;
	for (auto const &name : state.buried)
		if (auto *track{simulation::Paths.find(name)}; track != nullptr && false == track->m_editorremoved)
			tracks.push_back(track);
	if (tracks.empty())
	{
		state.buried.clear();
		return;
	}
	auto const low{editor_ui::view_min()};
	auto const high{editor_ui::view_max()};
	ImGui::SetNextWindowPos(ImVec2((low.x + high.x) * 0.5f, low.y + std::round(ImGui::GetFontSize() * (nmt_busy() ? 7.0f : 3.6f))), ImGuiCond_Always, ImVec2(0.5f, 0.0f));
	ImGui::SetNextWindowViewport(ImGui::GetMainViewport()->ID);
	ImGui::SetNextWindowBgAlpha(0.94f);
	if (ImGui::Begin("##nmtburied", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoDocking))
	{
		ImGui::Text(STR_C("%.0f m of the new track is hidden in the ground, you can't see it there"), state.buried_length);
		ImGui::TextDisabled("%s", STR_C("Only the ground along the track changes; Ctrl+Z takes it back"));
		ImGui::BeginDisabled(nmt_busy());
		if (main_button(STR_C("Fit the ground to the track"), ImVec2(0.0f, tall_button_height())))
		{
			nmt_pull_terrain(tracks);
			ui()->set_status(state.status, false);
			state.buried.clear();
		}
		ImGui::EndDisabled();
		ImGui::SameLine();
		if (ImGui::Button(STR_C("Leave it"), ImVec2(0.0f, tall_button_height())))
			state.buried.clear();
		ImGui::SameLine();
		ImGui::AlignTextToFramePadding();
		bool ask{EditorSettings.terrain_buried_ask()};
		if (ImGui::Checkbox(STR_C("Tell me each time"), &ask))
		{
			EditorSettings.terrain_buried_ask(ask);
			EditorSettings.save();
		}
	}
	ImGui::End();
}

void editor_mode::render_empty_scenery_hint()
{
	// the panel of the empty scenery of the tracks shown, in this frame or the last: it says the same
	if (m_lay.active || m_terrain_open || m_nmt_state.prompt || m_streamer.resident() > 0 || ImGui::IsPopupOpen(nullptr, ImGuiPopupFlags_AnyPopupId) || ImGui::GetFrameCount() - m_empty_hint_frame <= 1)
		return;
	auto const &tracks{simulation::Paths.sequence()};
	if (std::any_of(tracks.begin(), tracks.end(), [](TTrack const *Track) { return Track != nullptr && false == Track->m_editorremoved && rail(*Track); }))
		return;
	auto const low{editor_ui::view_min()};
	auto const high{editor_ui::view_max()};
	auto *list{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
	auto const line = [&](char const *Text, float const Size, float const Y, ImU32 const Color) {
		auto const width{ImGui::GetFont()->CalcTextSizeA(Size, FLT_MAX, 0.0f, Text).x};
		list->AddText(ImGui::GetFont(), Size, ImVec2(std::round((low.x + high.x - width) * 0.5f), Y), Color, Text);
	};
	auto const font{ImGui::GetFontSize()};
	auto const middle{(low.y + high.y) * 0.5f};
	line(STR_C("This scenery is still empty"), font * 1.6f, middle - font * 2.2f, IM_COL32(255, 255, 255, 255));
	line(STR_C("Click Laying on the toolbar or press L, then click the points of the track"), font, middle, IM_COL32(210, 215, 225, 255));
}

void editor_mode::render_outside_area_hint()
{
	auto const &area{m_orthophoto.area()};
	auto const &config{m_orthophoto.settings()};
	// with no place of the scenery on the map the camera is nowhere in it
	if (area.empty() || false == m_orthophoto.enabled() || (config.north == 0.0 && config.east == 0.0))
		return;
	if (geometry::point_in_polygon(area, m_orthophoto.to_puwg({Camera.Pos.x, Camera.Pos.z})))
	{
		m_outside_area_closed = false;
		return;
	}
	// the camera riding a track has its own lines at the bottom of the view, where this one would cover them
	if (m_outside_area_closed || m_area_drawing.active || m_ride.active() || ImGui::IsPopupOpen(nullptr, ImGuiPopupFlags_AnyPopupId))
		return;
	auto const low{editor_ui::view_min()};
	auto const high{editor_ui::view_max()};
	auto const font{ImGui::GetFontSize()};
	// above the strip of the profile and above the row of the keys of the tools at the bottom of the view
	auto const bottom{m_profile.strip_top > low.y + 4.0f * font ? std::min(high.y, m_profile.strip_top) : high.y};
	ImGui::SetNextWindowPos(ImVec2((low.x + high.x) * 0.5f, bottom - std::round(font * 2.8f)), ImGuiCond_Always, ImVec2(0.5f, 1.0f));
	ImGui::SetNextWindowViewport(ImGui::GetMainViewport()->ID);
	ImGui::SetNextWindowBgAlpha(0.88f);
	if (ImGui::Begin("##outsidearea", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoDocking))
	{
		ImGui::AlignTextToFramePadding();
		ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.35f, 1.0f), "%s", STR_C("The camera is outside the range: no aerial photos and no terrain model here"));
		ImGui::SameLine();
		if (ImGui::SmallButton(orthophoto_settings_open() ? STR_C("Draw the range again") : STR_C("Range settings...")))
		{
			if (orthophoto_settings_open())
				area_start();
			else
			{
				ui()->show_layers();
				m_orthophoto_window = true;
			}
		}
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("The range is the download area in Map > Orthophoto settings...; drawn again it can take this place in"));
		ImGui::SameLine();
		if (ImGui::SmallButton("x##outsidearea"))
			m_outside_area_closed = true;
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("Hidden until the camera is back inside the range and out again"));
	}
	ImGui::End();
}

void editor_mode::camera_above_ground()
{
	std::vector<char> found;
	auto const ground{ground_heights({glm::dvec3{Camera.Pos.x, 5000.0, Camera.Pos.z}}, true, &found).front()};
	if (found.front() == 0 || Camera.Pos.y > ground + 2.0)
		return;
	Camera.Pos.y = ground + 40.0;
	// looking down, to see what the view from above showed around it
	Camera.Angle.x = -0.9;
	Camera.Velocity = {};
	m_focus_active = false;
	Global.pCamera = Camera;
}

bool editor_mode::render_nmt_place(bool const Moving, bool const Button)
{
	auto &state{m_nmt_state};
	auto &map{m_nmt_map};
	auto const width{ImGui::GetContentRegionAvail().x};
	auto const height{std::round(std::min(width * 0.65f, ImGui::GetFontSize() * 16.0f))};
	ImGui::TextWrapped("%s", STR_C("Click the middle of the scenery on the map"));
	item_tooltip("The point 0,0,0 of the scenery. The wheel zooms the map, a drag moves it");
	ImGui::BeginChild("##nmtmap", ImVec2(width, height), false, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
	if (map.render({width, height}))
	{
		auto const degrees{editor_geo::to_degrees(map.point())};
		std::snprintf(state.place, sizeof(state.place), "%.6f, %.6f", degrees.x, degrees.y);
	}
	ImGui::EndChild();
	ImGui::SetNextItemWidth(-FLT_MIN);
	glm::dvec2 typed;
	auto const understood{editor_geo::parse_place(state.place, typed)};
	if (ImGui::InputTextWithHint("##nmtplace", STR_C("or type it: 52.2297, 21.0122"), state.place, sizeof(state.place)) && editor_geo::parse_place(state.place, typed))
	{
		map.pick(typed);
		map.show(typed, 8.0);
	}
	item_tooltip("Degrees as the online maps copy them (latitude, longitude), or the X and Y of PUWG 1992, the coordinates of the Polish maps of geoportal.gov.pl");
	if (state.place[0] != '\0' && false == understood)
		ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f), "%s", STR_C("Not a place in Poland: two numbers, degrees or the X and Y of PUWG 1992"));
	else if (map.picked())
		ImGui::TextDisabled(STR_C("X %.0f, Y %.0f in PUWG 1992"), map.point().y, map.point().x);
	if (Moving && m_streamer.resident() > 0)
	{
		ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.95f, 0.75f, 0.3f, 1.0f));
		ImGui::TextWrapped("%s", STR_C("The terrain made already stays as it is"));
		ImGui::PopStyleColor();
	}
	if (false == Button)
		return false;
	auto const cancel{STR_C("Cancel")};
	auto const cancel_width{ImGui::CalcTextSize(cancel).x + ImGui::GetStyle().FramePadding.x * 2.0f};
	bool placed{false};
	ImGui::BeginDisabled(false == map.picked());
	if (main_button(STR_C("The scenery is here"), ImVec2(Moving ? -(cancel_width + ImGui::GetStyle().ItemSpacing.x) : -FLT_MIN, tall_button_height())))
	{
		nmt_place_scenery(map.point());
		placed = true;
	}
	ImGui::EndDisabled();
	if (Moving)
	{
		ImGui::SameLine();
		if (ImGui::Button(cancel, ImVec2(0.0f, tall_button_height())))
			state.moving = false;
	}
	return placed;
}

void editor_mode::render_nmt_ui()
{
	auto &state{m_nmt_state};
	state.shown = ImGui::GetFrameCount();
	nmt_locate();
	auto const busy{nmt_busy()};
	step_heading(1, STR_C("Location"));
	if (false == m_nmt.placed() || state.moving)
	{
		if (render_nmt_place(state.moving, true))
			state.moving = false;
		if (false == m_nmt.placed())
			return;
	}
	else
	{
		auto const &config{m_orthophoto.settings()};
		auto const degrees{editor_geo::to_degrees({config.east, config.north})};
		ImGui::AlignTextToFramePadding();
		ImGui::Text(STR_C("%.5f N, %.5f E"), degrees.x, degrees.y);
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip(STR_C("X %.0f, Y %.0f in PUWG 1992, the coordinates of the Polish maps; the place is shared with the orthophoto"), config.north, config.east);
		ImGui::SameLine();
		ImGui::BeginDisabled(busy);
		if (ImGui::SmallButton(STR_C("Change...")))
		{
			glm::dvec2 const point{config.east, config.north};
			m_nmt_map.pick(point);
			m_nmt_map.show(point, 8.0);
			std::snprintf(state.place, sizeof(state.place), "%.6f, %.6f", degrees.x, degrees.y);
			state.moving = true;
		}
		ImGui::EndDisabled();
	}

	auto const tracks{std::any_of(simulation::Paths.sequence().begin(), simulation::Paths.sequence().end(), [](TTrack const *Track) { return Track != nullptr && rail(*Track); })};
	// the progress shown at the step whose button began it
	auto const tracks_busy{state.stage != nmt_state::automatic::idle || (state.fitting && state.under)};
	if (tracks)
	{
		step_heading(2, STR_C("Tracks and terrain"));
		if (tracks_busy)
			render_nmt_progress(false);
		else
			render_nmt_all_tracks(true);
		ImGui::Spacing();
		if (busy && false == tracks_busy)
			render_nmt_progress(false);
	}
	else
	{
		// no tracks yet: the model as it is around the camera, a square of it
		step_heading(2, STR_C("Load"));
		if (busy)
			render_nmt_progress(false);
		else
		{
			if (main_button(STR_C("Load around the camera"), ImVec2(-FLT_MIN, tall_button_height())))
			{
				if (m_streamer.resident() == 0)
					state.offset = 0.0;
				nmt_make(nmt_keys_around());
			}
			item_tooltip("Loads the terrain model as it is around the camera, with the height offset above; the tracks stay where they are");
			if (begin_fields("##nmtload", 0.5f))
			{
				field_row("Area");
				segmented("nmtarea", m_terrain_add_count, {{2, "0.5 km", "A square of 500 m around the camera"}, {4, "1 km", "A square of 1 km around the camera"}, {8, "2 km", "A square of 2 km around the camera"}});
				ImGui::EndTable();
			}
		}
		ImGui::PushTextWrapPos(0.0f);
		ImGui::TextDisabled("%s", STR_C("A chunk there is already is never made anew"));
		ImGui::PopTextWrapPos();
	}
	if (ImGui::TreeNode(STR_C("Settings")))
	{
		if (toggle_chips("nmtgrow", {{STR_C("Terrain of the model right away while laying"), &state.grow,
		                              "Where the terrain of the scenery ends, the terrain model is made under the tracks as they're laid, around the cursor from the first point placed; "
		                              "the chunks there are stay as they are. Off, it's made only when asked for"}}) >= 0)
		{
			EditorSettings.terrain_while_laying(state.grow);
			EditorSettings.save();
		}
		// with tracks the model as it is is a card of step 2, the band of it and the square around the camera are here
		if (tracks && begin_fields("##nmtoffset", 0.5f))
		{
			field_row("Band along the tracks (m)");
			ImGui::InputDouble("##nmtbandwidth", &state.band, 50.0, 250.0, "%.0f");
			item_tooltip("How far from the tracks, on both sides, the terrain model is loaded");
			field_row("Height offset of the model (m)");
			ImGui::InputDouble("##nmtheightoffset", &state.offset, 0.1, 1.0, "%.2f");
			item_tooltip("Added to the heights of the model, for a scenery built at heights of its own; the tracks put onto the terrain find it by themselves");
			ImGui::TableNextRow();
			ImGui::TableSetColumnIndex(1);
			ImGui::BeginDisabled(busy);
			if (ImGui::Button(STR_C("Fit to the tracks"), ImVec2(-FLT_MIN, 0.0f)))
				nmt_fit();
			ImGui::EndDisabled();
			item_tooltip("Compares the formation of the tracks with the terrain model and moves the model up or down to meet them, as a scenery built at height 0 needs");
			ImGui::EndTable();
		}
		if (tracks)
		{
			ImGui::BeginDisabled(busy);
			if (ImGui::Button(STR_C("Load around the camera"), ImVec2(-FLT_MIN, 0.0f)))
				nmt_make(nmt_keys_around());
			item_tooltip("Loads the terrain model as it is around the camera, with the height offset above; the tracks stay where they are");
			ImGui::EndDisabled();
		}
		ImGui::TreePop();
	}
	state.band = std::clamp(state.band, 50.0, 3000.0);
	ImGui::Spacing();
	// while the work goes on the bar above tells how far it got
	if (false == state.status.empty() && false == busy)
	{
		ImGui::PushTextWrapPos(0.0f);
		ImGui::TextDisabled("%s", state.status.c_str());
		ImGui::PopTextWrapPos();
	}
	if (false == state.error.empty())
	{
		ImGui::PushTextWrapPos(0.0f);
		ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f), "%s", state.error.c_str());
		ImGui::PopTextWrapPos();
	}
	bool again{false};
	if (auto const error{m_nmt.error(&again)}; false == error.empty())
	{
		ImGui::PushTextWrapPos(0.0f);
		ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f), "%s", STR(error).c_str());
		ImGui::PopTextWrapPos();
		if (again && ImGui::Button(STR_C("Try again")))
			m_nmt.retry();
		if (again)
			item_tooltip("The squares of the model the service didn't give are asked for again; it happens by itself every 15 s too");
	}
	ImGui::TextDisabled("%s", STR_C("Terrain model: GUGiK, geoportal.gov.pl"));
}

void editor_mode::nmt_place_scenery(glm::dvec2 const &Point)
{
	auto &state{m_nmt_state};
	auto config{m_orthophoto.settings()};
	config.north = Point.y;
	config.east = Point.x;
	m_orthophoto.settings(config);
	m_orthophoto_origin_edit = {config.north, config.east};
	save_orthophoto_settings();
	nmt_locate();
	// the heights fitted to the tracks at the place before mean nothing here
	state.offset = 0.0;
	state.fitted = false;
	state.under = false;
	state.anchors.clear();
	state.anchor_cells.clear();
}

void editor_mode::nmt_offer()
{
	auto &state{m_nmt_state};
	if (state.offered || false == m_lay.active || false == EditorSettings.terrain_offer())
		return;
	state.offered = true;
	if (m_streamer.resident() > 0 || std::any_of(simulation::Paths.sequence().begin(), simulation::Paths.sequence().end(), [](TTrack const *Track) { return Track != nullptr && rail(*Track); }))
		return;
	nmt_locate();
	state.prompt = true;
}

void editor_mode::nmt_accept_offer()
{
	auto &state{m_nmt_state};
	if (false == m_nmt.placed() && m_nmt_map.picked())
		nmt_place_scenery(m_nmt_map.point());
	// no tracks to fit to: the heights of the model as they are
	state.offset = 0.0;
	nmt_make(nmt_keys_around());
	state.prompt = false;
}

void editor_mode::render_nmt_prompt()
{
	auto &state{m_nmt_state};
	if (false == state.prompt)
		return;
	char const *const title{STR_C("Terrain for the new scenery###nmtoffer")};
	if (false == ImGui::IsPopupOpen(title))
		ImGui::OpenPopup(title);
	ImGui::SetNextWindowSize(ImVec2(520.0f * Global.ui_scale, 0.0f));
	if (false == ImGui::BeginPopupModal(title, nullptr, ImGuiWindowFlags_NoResize))
		return;
	ImGui::TextWrapped("%s", STR_C("The scenery has no terrain yet. Load the terrain model and the aerial photos around here? The tracks laid then go along the real ground."));
	bool const placed{m_nmt.placed()};
	if (false == placed)
		render_nmt_place(false, false);
	auto const known{placed || m_nmt_map.picked()};
	ImGui::Spacing();
	auto const gap{std::round(ImGui::GetFontSize() * 0.6f)};
	auto const width{std::floor((ImGui::GetContentRegionAvail().x - gap) * 0.5f)};
	if (ground_card("##terrain", STR_C("Load the terrain"), STR_C("The terrain model and the aerial photos of geoportal.gov.pl around here; the tracks go along the real ground"), true, known, width))
	{
		nmt_accept_offer();
		ImGui::CloseCurrentPopup();
	}
	if (false == known)
		item_tooltip("Pick where the scenery lies first");
	ImGui::SameLine(0.0f, gap);
	if (ground_card("##flat", STR_C("Without terrain"), STR_C("The tracks lie on the flat plane at the height 0; the terrain can be loaded later in Terrain (F5)"), false, true, width))
	{
		state.prompt = false;
		ImGui::CloseCurrentPopup();
	}
	ImGui::Spacing();
	bool ask{EditorSettings.terrain_offer()};
	if (ImGui::Checkbox(STR_C("Ask with every new scenery"), &ask))
	{
		EditorSettings.terrain_offer(ask);
		EditorSettings.save();
	}
	ImGui::EndPopup();
}

std::size_t editor_mode::nmt_join_ends()
{
	// the loose ends are one joint before the grade lines run over them; of two switches, which can't be changed alone, the
	// lines over them take each other over as they're fitted
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> changed;
	auto const remember = [&](TTrack *Track) {
		if (std::find(changed.begin(), changed.end(), Track) != changed.end())
			return;
		states.emplace_back(Track, editor_track::capture(*Track));
		changed.push_back(Track);
	};
	std::size_t joined{0};
	m_nmt_state.loose.clear();
	for (auto pair : loose_pairs())
	{
		if (pair.first.track->eType != tt_Normal)
			std::swap(pair.first, pair.second);
		auto const &a{pair.first};
		auto const &b{pair.second};
		if (a.track->eType != tt_Normal)
		{
			// two switches: the lines over them meet as they're fitted
			m_nmt_state.loose[{a.track, a.path, a.atend}] = {b.track, b.path, b.atend};
			m_nmt_state.loose[{b.track, b.path, b.atend}] = {a.track, a.path, a.atend};
			continue;
		}
		if (b.track->eType != tt_Normal && false == editor_track::is_supported(*b.track))
			continue;
		auto const height{b.track->eType == tt_Normal ? (a.point.y + b.point.y) * 0.5 : b.point.y};
		for (auto const *side : {&a, &b})
		{
			if (side->track->eType != tt_Normal)
				continue;
			remember(side->track);
			side->track->m_paths[side->path].points[side->atend ? segment_data::point::end : segment_data::point::start].y = height;
		}
		++joined;
	}
	if (changed.empty())
		return 0;
	editor_track::commit(changed);
	push_track_snapshot(std::move(states));
	WriteLog(format("Editor: NMT - %zu joints of tracks laid apart in height joined", joined), logtype::generic);
	return joined;
}

std::vector<editor_mode::nmt_state::turntable> editor_mode::nmt_turntables() const
{
	std::vector<nmt_state::turntable> result;
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->eType != tt_Table || false == rail(*track))
			continue;
		auto const &points{track->m_paths.front().points};
		nmt_state::turntable table;
		table.table = track;
		table.centre = glm::mix(points[segment_data::point::start], points[segment_data::point::end], 0.5);
		table.rim = 0.5 * geometry::plan_distance(points[segment_data::point::start], points[segment_data::point::end]);
		if (table.rim < 1.0)
			continue;
		result.push_back(std::move(table));
	}
	if (result.empty())
		return result;
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->eType == tt_Table || false == rail(*track))
			continue;
		for (int path = 0; path < static_cast<int>(track->m_paths.size()); ++path)
			for (auto const atend : {false, true})
			{
				auto const end{end_of(*track, path, atend)};
				for (auto &table : result)
					if (on_rim(end, table.centre, table.rim))
						table.ends.emplace_back(track, path, atend);
			}
	}
	return result;
}

std::size_t editor_mode::nmt_level_turntables()
{
	auto &state{m_nmt_state};
	state.turntables = nmt_turntables();
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> changed;
	std::vector<std::pair<standing_template, template_item>> moves;
	std::vector<std::pair<TAnimModel *, double>> models;
	std::vector<standing_template> standing;
	if (false == state.turntables.empty())
		standing = standing_templates([](std::string const &) { return true; });
	for (auto &table : state.turntables)
	{
		// the height of the tracks running up to it: of the ones fitted already, which stay as they are, else of the model beside
		// the rim past the wall of the pit, out along each of them
		std::vector<double> fitted, model;
		for (auto const &[track, path, atend] : table.ends)
		{
			auto const end{end_of(*track, path, atend)};
			if (nmt_stays(*track))
				fitted.push_back(end.point.y);
			auto const away{geometry::plan_of(end.point - table.centre) / std::max(1e-6, geometry::plan_distance(end.point, table.centre))};
			for (int metre = 0; metre <= 6; ++metre)
				if (double height; m_nmt.height(end.point.x + away.x * (kPitWall + metre), end.point.z + away.y * (kPitWall + metre), height))
					model.push_back(height + state.offset + track->rail_height());
		}
		auto &heights{fitted.empty() ? model : fitted};
		if (heights.empty())
		{
			WriteLog(format("Editor: NMT - turntable %s left at %.2f m: %s", table.table->name().c_str(), table.centre.y, table.ends.empty() ? "no tracks run up to it" : "no terrain model beside it"), logtype::generic);
			continue;
		}
		std::nth_element(heights.begin(), heights.begin() + heights.size() / 2, heights.end());
		auto const height{heights[heights.size() / 2]};
		auto const rise{height - table.centre.y};
		WriteLog(format("Editor: NMT - turntable %s level at %.2f m, from %.2f m (%+.2f), the height of its %zu tracks %s", table.table->name().c_str(), height, table.centre.y, rise, table.ends.size(),
		                fitted.empty() ? "on the model beside its rim" : "fitted already"),
		         logtype::generic);
		auto &points{table.table->m_paths.front().points};
		table.centre.y = height;
		if (std::abs(points[segment_data::point::start].y - height) < 0.001 && std::abs(points[segment_data::point::end].y - height) < 0.001)
			continue;
		states.emplace_back(table.table, editor_track::capture(*table.table));
		points[segment_data::point::start].y = height;
		points[segment_data::point::end].y = height;
		points[segment_data::point::control1].y = 0.0;
		points[segment_data::point::control2].y = 0.0;
		table.table->m_paths.front().pieces.clear();
		changed.push_back(table.table);
		// its include, the pit and the bridge, by the name of the table in its parameters; a model of its name standing apart
		std::set<scene::instance_handle> included;
		for (auto const &item : standing)
			if (false == item.values.empty() && ToLower(item.values.front()) == table.table->name() && geometry::plan_distance(item.location, table.centre) <= 1.0)
			{
				template_item moved;
				moved.file = item.file;
				moved.location = item.location + glm::dvec3{0.0, rise, 0.0};
				moved.yaw = item.yaw;
				moves.emplace_back(item, moved);
				included.insert(item.instance);
			}
		auto const apart{models.size()};
		if (auto *model{simulation::Instances.find(table.table->name())}; model != nullptr && false == included.contains(model->m_instance) && geometry::plan_distance(model->location(), table.centre) <= table.rim)
			models.emplace_back(model, rise);
		WriteLog(format("Editor: NMT - turntable %s moved %+.2f m with %zu include(s) and %zu model(s) of its name", table.table->name().c_str(), rise, included.size(), models.size() - apart), logtype::generic);
	}
	if (changed.empty())
		return 0;
	editor_track::commit(changed);
	push_track_snapshot(std::move(states));
	if (move_templates(moves) > 0 && m_history.size() >= 2)
		m_history.back().joined = true;
	for (auto const &[model, rise] : models)
	{
		push_snapshot(model, EditorSnapshot::Action::Move);
		m_history.back().joined = true;
		m_editor.translate(model, model->location() + glm::dvec3{0.0, rise, 0.0}, true);
	}
	return changed.size();
}

bool editor_mode::nmt_stays(TTrack const &Track)
{
	auto const &state{m_nmt_state};
	return (state.unfitted && grade_fitted(Track)) || (false == state.redo_hand && grade_hand(Track));
}

void editor_mode::nmt_auto_start(bool const Unfitted)
{
	auto &state{m_nmt_state};
	state.error.clear();
	state.under = false;
	state.unfitted = Unfitted;
	state.result = false;
	state.findings.clear();
	nmt_locate();
	state.probes.clear();
	state.probes_kept = 0;
	// the tracks fitted already, when they stay as they are, tell the offset of the model they were fitted to, which isn't kept
	// from one opening of the scenery to the next: a few hundred of their places are measured too, after the others
	std::vector<nmt_place> kept;
	for (auto const &place : nmt_track_places(20.0))
		if (place.shapes)
			(nmt_stays(*place.track) ? kept : state.probes).push_back(place);
	if (state.probes.empty())
	{
		state.status = Unfitted && false == kept.empty() ? STR("All the tracks are fitted to the terrain already") : STR("No tracks to put onto the terrain");
		return;
	}
	auto const every{std::max<std::size_t>(1, kept.size() / 400)};
	for (std::size_t i = 0; i < kept.size(); i += every)
		state.probes.push_back(kept[i]);
	state.probes_kept = (kept.size() + every - 1) / every;
	state.lines = 0;
	state.refused = 0;
	state.departures = 0;
	state.levelled = 0;
	state.findings.clear();
	state.result = false;
	state.run = true;
	state.fetch_most = 0;
	state.stage = nmt_state::automatic::fetching;
	state.status = STR("Fetching the terrain model under the tracks...");
}

bool editor_mode::nmt_fit_line(TTrack &Track, std::function<bool(TTrack const *)> const &Fitted, bool const Laid)
{
	auto &profile{m_profile};
	auto const &fitted{Fitted};
	// the line runs over the tracks not fitted yet around the track: the ones fitted stay as they are
	auto route{editor_track::run_route(Track, profile.run_length)};
	auto const at{std::find_if(route.spans.begin(), route.spans.end(), [&](auto const &Span) { return Span.track == &Track; })};
	auto const middle{geometry::bezier{Track.m_paths.front()}.point(0.5)};
	if (at == route.spans.end())
	{
		nmt_note(nmt_state::finding::kind::refused, &Track, &Track, Track.Length(), middle, STR("No line of tracks runs over the track"));
		return false;
	}
	auto first{at};
	while (first != route.spans.begin() && false == fitted(std::prev(first)->track))
		--first;
	auto last{at};
	while (std::next(last) != route.spans.end() && false == fitted(std::next(last)->track))
		++last;
	route.spans = {first, std::next(last)};
	profile.open = true;
	profile.status.clear();
	profile.error.clear();
	auto *from{route.spans.front().track};
	auto *to{route.spans.back().track};
	profile_take(std::move(route), from, to);
	if (false == profile.error.empty() || profile.samples.empty())
	{
		nmt_note(nmt_state::finding::kind::refused, from, to, profile.route.length, middle, profile.error.empty() ? STR("The line has no profile") : profile.error);
		return false;
	}
	// an end which goes on into a line fitted already keeps its height and grade; any other end is free, the tracks past it are
	// fitted later and meet this line then
	auto const joint = [&](bool const Atend, double &Height) {
		double grade;
		TTrack *adjoining{nullptr};
		int path{0};
		if (false == editor_track::adjoining_grade(profile.route, Atend, grade, &adjoining, &path) || adjoining == nullptr || false == fitted(adjoining))
			return false;
		auto const &end{Atend ? profile.samples.back().position : profile.samples.front().position};
		auto const &points{adjoining->m_paths[path].points};
		auto const &start{points[segment_data::point::start]};
		auto const &finish{points[segment_data::point::end]};
		Height = geometry::plan_distance(start, end) < geometry::plan_distance(finish, end) ? start.y : finish.y;
		return true;
	};
	profile.context.start_joined = profile.context.start_joined && joint(false, profile.context.start_elevation);
	profile.context.end_joined = profile.context.end_joined && joint(true, profile.context.end_elevation);
	auto const on_route = [&](TTrack const *Other) { return std::any_of(profile.route.spans.begin(), profile.route.spans.end(), [&](auto const &Span) { return Span.track == Other; }); };
	// an end on a switch which meets a switch fitted already unlinked, laid apart in height, takes it over as if it went on into it
	auto const partner_of = [&](editor_track::route_span const &Span, bool const Atend) -> std::optional<loose_end> {
		auto const found{m_nmt_state.loose.find({Span.track, Span.path, Atend})};
		if (found == m_nmt_state.loose.end() || on_route(std::get<0>(found->second)))
			return std::nullopt;
		auto const end{end_of(*std::get<0>(found->second), std::get<1>(found->second), std::get<2>(found->second))};
		if (glm::length(geometry::plan_of(end.outward)) < 1e-6)
			return std::nullopt;
		return end;
	};
	auto const loose_into = [&](editor_track::route_span const &Span, bool const Atend) -> std::optional<loose_end> {
		auto const end{partner_of(Span, Atend)};
		return end.has_value() && fitted(end->track) ? end : std::nullopt;
	};
	// rising off the end met, into the route at its start and against it at its end
	auto const rising = [](loose_end const &End) { return End.outward.y / glm::length(geometry::plan_of(End.outward)); };
	if (false == profile.context.start_joined)
		if (auto const end{loose_into(profile.route.spans.front(), false == profile.route.spans.front().forward)})
		{
			profile.context.start_joined = true;
			profile.context.start_elevation = end->point.y;
			profile.context.start_grade = rising(*end);
		}
	if (false == profile.context.end_joined)
		if (auto const end{loose_into(profile.route.spans.back(), profile.route.spans.back().forward)})
		{
			profile.context.end_joined = true;
			profile.context.end_elevation = end->point.y;
			profile.context.end_grade = -rising(*end);
		}
	// the ground of the line: the terrain model on its axis, the top of the ballast with the rails on it, and the rail tops of
	// the lines fitted already beside it on the same level as the model here, which tell its height the more closely the
	// nearer they are. What doesn't agree with the line, a bridge over it, the ground under a viaduct, the fit lets go of by
	// itself
	double constexpr kModel{0.15}; // m, the tolerance of the model
	double constexpr kRail{0.02}; // m, of a rail top next to the line, growing by kSpread a metre of the distance
	double constexpr kSpread{0.01};
	double constexpr kLevel{0.5}; // m, a rail top farther from the model or from the line lies on another level
	double constexpr kSameLevel{1.0}; // m, tracks authored closer in height lie on one level, farther one over the other
	double constexpr kReach{50.0};
	auto const nearest = [&](glm::dvec3 const &Point, double &Distance) {
		Distance = std::numeric_limits<double>::max();
		double height{0.0};
		auto const cx{static_cast<int>(std::floor(Point.x / kPlanCell))};
		auto const cz{static_cast<int>(std::floor(Point.z / kPlanCell))};
		for (int dx = -1; dx <= 1; ++dx)
			for (int dz = -1; dz <= 1; ++dz)
			{
				auto const cell{m_nmt_state.laid.find({cx + dx, cz + dz})};
				if (cell == m_nmt_state.laid.end())
					continue;
				for (auto const &laid : cell->second)
					if (auto const distance{glm::length(glm::dvec2{laid.position.x - Point.x, laid.position.z - Point.z})}; distance < Distance)
					{
						Distance = distance;
						height = laid.position.y;
					}
			}
		return height;
	};
	// an end of the line, or the other path of a switch of it, which leads through the tracks not fitted yet into a line fitted
	// already, as at a crossover, ties the two where it leaves the line: there the line keeps to the height of the other, on
	// along its grade over the tracks between, give or take what the plain tracks among these can climb with curves of the least
	// radius. A switch lies in its plane and climbs nothing. Where they can climb more than a little, the curves of the norms of
	// the line come first: what it can't keep to, the tracks between take over
	double constexpr kJoin{0.002}; // m
	double constexpr kClimbing{0.05}; // m
	double constexpr kFarthest{200.0};
	profile::ground_fit options;
	// an end on the rim of a turntable runs up to it level at its height, as far as kRunUp off it or half the line, which turns to
	// the ground only past that, not at the edge of the pit; the model in the pit isn't its ground
	auto const first_chainage{profile.samples.front().chainage};
	auto const last_chainage{profile.samples.back().chainage};
	for (auto const atend : {false, true})
	{
		auto const &span{atend ? profile.route.spans.back() : profile.route.spans.front()};
		auto const end{end_of(*span.track, span.path, atend ? span.forward : false == span.forward)};
		auto const table{std::find_if(m_nmt_state.turntables.begin(), m_nmt_state.turntables.end(), [&](nmt_state::turntable const &Table) { return on_rim(end, Table.centre, Table.rim); })};
		if (table == m_nmt_state.turntables.end())
			continue;
		auto const height{table->centre.y};
		auto const level{std::min(kRunUp, 0.5 * (last_chainage - first_chainage))};
		auto const from{atend ? last_chainage - level : first_chainage};
		(atend ? profile.context.end_joined : profile.context.start_joined) = true;
		(atend ? profile.context.end_elevation : profile.context.start_elevation) = height;
		(atend ? profile.context.end_grade : profile.context.start_grade) = 0.0;
		profile.context.switches.push_back({from, from + level, table->table->name()});
		for (int metre = 0; from + metre <= from + level + 1e-6; ++metre)
			options.bounds.push_back({from + metre, height - kJoin, height + kJoin, false});
		WriteLog(format("Editor: NMT line %s runs up to turntable %s level at %.2f m over %.1f m", Track.name().c_str(), table->table->name().c_str(), height, level), logtype::generic);
	}
	auto const in_pit =[&](glm::dvec3 const &Point) {
		return std::any_of(m_nmt_state.turntables.begin(), m_nmt_state.turntables.end(), [&](nmt_state::turntable const &Table) { return geometry::plan_distance(Point, Table.centre) < Table.rim + kPitWall; });
	};
	auto const length_of =[](TTrack const &Track, int const Path) { return Track.eType == tt_Switch ? Track.SwitchExtension->Segments[Path]->GetLength() : Track.Segment->GetLength(); };
	std::vector<editor_track::route_span> exits;
	exits.push_back({profile.route.spans.front().track, profile.route.spans.front().path, false == profile.route.spans.front().forward});
	exits.push_back({profile.route.spans.back().track, profile.route.spans.back().path, profile.route.spans.back().forward});
	for (auto const &span : profile.route.spans)
		if (span.track->eType == tt_Switch && span.track->m_paths.size() >= 2)
			for (auto const outward : {true, false})
				exits.push_back({span.track, 1 - span.path, outward});
	// a switch entered at the far end of a path has the far end of its other path on its plane as high, when as far along it
	auto const frog_beside = [](TTrack const &Switch, int const Path) {
		if (Switch.m_paths.size() < 2)
			return false;
		auto const &entered{Switch.m_paths[Path].points};
		auto const &other{Switch.m_paths[1 - Path].points};
		auto const &origin{entered[segment_data::point::start]};
		auto const &control{entered[segment_data::point::control1]};
		auto const along{geometry::plan_of(control != glm::dvec3{} ? control : entered[segment_data::point::end] - origin)};
		if (glm::length(along) < 1e-6)
			return false;
		auto const ahead = [&](glm::dvec3 const &Point) { return glm::dot(geometry::plan_of(Point - origin), glm::normalize(along)); };
		return std::abs(ahead(entered[segment_data::point::end]) - ahead(other[segment_data::point::end])) <= 0.5;
	};
	for (auto const &exiting : exits)
	{
		auto const &exit{exiting.track->m_paths[exiting.path].points[exiting.forward ? segment_data::point::end : segment_data::point::start]};
		auto const bind = [&](double const Height, double const Between, double const Bending) {
			std::size_t nearest{0};
			for (std::size_t i = 1; i < profile.samples.size(); ++i)
				if (geometry::plan_distance(profile.samples[i].position, exit) < geometry::plan_distance(profile.samples[nearest].position, exit))
					nearest = i;
			auto const climbing{Bending * Between / (4.0 * profile.line.norms.radius_min)};
			options.bounds.push_back({profile.samples[nearest].chainage, Height - kJoin - climbing, Height + kJoin + climbing, climbing >= kClimbing});
		};
		// the tracks it leads into, the nearest first, through both paths of a switch met at its start and across the frog of one
		// met at the end of a path, onto the end of its other path
		struct walk
		{
			editor_track::route_span span;
			double between{0.0}; // m, over the tracks not fitted yet
			double bending{0.0}; // m, of these the plain tracks
		};
		std::vector<walk> open{{exiting, 0.0, 0.0}};
		std::set<std::tuple<TTrack const *, int, bool>> seen;
		while (false == open.empty())
		{
			auto const nearest{std::min_element(open.begin(), open.end(), [](walk const &A, walk const &B) { return A.between < B.between; })};
			auto const step{*nearest};
			open.erase(nearest);
			if (step.between >= kFarthest || false == seen.insert({step.span.track, step.span.path, step.span.forward}).second)
				continue;
			auto onward{editor_track::continuations(step.span)};
			onward.erase(std::remove_if(onward.begin(), onward.end(), [&](editor_track::continuation const &Next) { return on_route(Next.span.track); }), onward.end());
			// met unlinked, laid apart in height: on as if linked
			if (onward.empty())
				if (auto const end{partner_of(step.span, step.span.forward)})
					onward.push_back({{end->track, end->path, false == end->atend}, -rising(*end)});
			for (auto const &[next, grade] : onward)
			{
				auto const &points{next.track->m_paths[next.path].points};
				auto const entry{points[next.forward ? segment_data::point::start : segment_data::point::end].y};
				if (fitted(next.track))
				{
					bind(entry - grade * step.between, step.between, step.bending);
					continue;
				}
				auto const length{length_of(*next.track, next.path)};
				open.push_back({next, step.between + length, step.bending + (next.track->eType == tt_Switch ? 0.0 : length)});
				if (false == next.forward && next.track->eType == tt_Switch && frog_beside(*next.track, next.path))
					open.push_back({{next.track, 1 - next.path, true}, step.between, step.bending});
			}
		}
	}
	// the lines fitted already close beside it on its level, as authored, are ground it never lets go of, however far they are
	// off: side by side the tracks lie on one formation, in one plane, as near as the curves of the norms let them. The nearest
	// on each side. Where they run so near that the timbers carry both, at a switch, a crossover, they are at one height, on
	// any vertical curve the timbers lie on
	double constexpr kNear{8.0}; // m between the axes
	double constexpr kHeld{5.0}; // m, the reach of their rail tops
	double constexpr kTimbers{2.65}; // m between the axes, about the length of a sleeper
	double constexpr kTimbered{0.005}; // m
	// the stretches of the line about a track it crosses on another level, as the model tells it further on: the rails at the height
	// of that track are nothing to the line there, however near and however the file has them
	double constexpr kOtherSide{2.0}; // m
	struct other_level
	{
		TTrack const *track;
		double chainage;
		double reach; // m along the line on both sides
		double height;
	};
	std::vector<other_level> other_levels;
	auto const on_other_level = [&](double const Chainage, double const Height) {
		return std::any_of(other_levels.begin(), other_levels.end(), [&](other_level const &Other) { return std::abs(Chainage - Other.chainage) <= Other.reach && std::abs(Height - Other.height) < kOtherSide; });
	};
	auto const alongside = [&](editor_track::route_sample const &Sample) {
		std::array<std::pair<double, nmt_state::laid_point const *>, 2> sides{std::pair{kNear, nullptr}, std::pair{kNear, nullptr}};
		auto const along{glm::normalize(Sample.direction)};
		auto const cx{static_cast<int>(std::floor(Sample.position.x / kPlanCell))};
		auto const cz{static_cast<int>(std::floor(Sample.position.z / kPlanCell))};
		for (int dx = -1; dx <= 1; ++dx)
			for (int dz = -1; dz <= 1; ++dz)
			{
				auto const cell{m_nmt_state.laid.find({cx + dx, cz + dz})};
				if (cell == m_nmt_state.laid.end())
					continue;
				for (auto const &point : cell->second)
				{
					glm::dvec2 const offset{point.position.x - Sample.position.x, point.position.z - Sample.position.z};
					auto const across{along.x * offset.y - along.y * offset.x};
					if (std::abs(glm::dot(offset, along)) > 1.0 || std::abs(across) >= kNear || std::abs(point.was - Sample.position.y) > kSameLevel || on_other_level(Sample.chainage, point.position.y))
						continue;
					auto &side{sides[across > 0.0 ? 0 : 1]};
					if (std::abs(across) < side.first)
						side = {std::abs(across), &point};
				}
			}
		return sides;
	};
	auto const count{profile.samples.size()};
	std::vector<double> sampled(count), model(count, std::numeric_limits<double>::quiet_NaN());
	std::vector<glm::dvec2> plan(count);
	std::vector<double> chainages, ground, tolerances, reaches;
	// where the line crosses another track at an angle, with no joint with it, the model shows the formation of the one on top and tells
	// nothing of the other: there it's no ground of the line, the height comes from the terrain on both sides, a line low there going under
	double constexpr kCrossedNear{2.0}; // m
	double constexpr kCrossedSine{0.5}; // 30 degrees
	double constexpr kCrossedReach{20.0}; // m along the line on both sides, about what the formation with its slopes takes
	std::vector<char> crossed(count, 0);
	for (std::size_t i = 0; i < count; ++i)
	{
		auto const &sample{profile.samples[i]};
		auto const along{glm::normalize(sample.direction)};
		auto const cx{static_cast<int>(std::floor(sample.position.x / kPlanCell))};
		auto const cz{static_cast<int>(std::floor(sample.position.z / kPlanCell))};
		for (int dx = -1; dx <= 1 && crossed[i] == 0; ++dx)
			for (int dz = -1; dz <= 1 && crossed[i] == 0; ++dz)
				if (auto const cell{m_nmt_state.paths.find({cx + dx, cz + dz})}; cell != m_nmt_state.paths.end())
					for (auto const &point : cell->second)
						if (false == on_route(point.track) && glm::length(glm::dvec2{point.position.x - sample.position.x, point.position.z - sample.position.z}) < kCrossedNear && std::abs(along.x * glm::normalize(point.direction).y - along.y * glm::normalize(point.direction).x) > kCrossedSine)
						{
							crossed[i] = 1;
							break;
						}
	}
	std::vector<char> no_ground(count, 0);
	for (std::size_t i = 0; i < count; ++i)
		if (crossed[i] != 0)
			for (std::size_t j = 0; j < count; ++j)
				if (std::abs(profile.samples[j].chainage - profile.samples[i].chainage) <= kCrossedReach)
					no_ground[j] = 1;
	for (std::size_t i = 0; i < count; ++i)
	{
		auto const &sample{profile.samples[i]};
		auto const *track{sample.span < profile.route.spans.size() ? profile.route.spans[sample.span].track : nullptr};
		sampled[i] = sample.chainage;
		plan[i] = {sample.position.x, sample.position.z};
		if (double height; no_ground[i] == 0 && track != nullptr && shapes_ground(*track) && false == in_pit(sample.position) && m_nmt.height(sample.position.x, sample.position.z, height))
			model[i] = height + m_nmt_state.offset + track->rail_height();
	}
	// a path which the line crosses on its level, as authored, makes a crossing with it: over it the two lie in one plane, the
	// grade doesn't break nor does a vertical curve reach; the line crosses a line fitted already at its height, the rail tops
	// of the two at one
	double constexpr kCrossing{0.75}; // m, the nearest the samples of the two come, a metre apart
	double constexpr kCrossed{0.035}; // the sine of the least angle the axes cross at
	double constexpr kGauge{1.435}; // m
	struct crossing
	{
		double chainage;
		double height;
		double sine;
		double distance;
		TTrack const *track;
		glm::dvec2 at; // where the axes meet
		glm::dvec2 along; // the way of the line there
	};
	auto const crossings_of = [&](std::map<std::pair<int, int>, std::vector<nmt_state::laid_point>> const &Points) {
		std::vector<crossing> found;
		auto const across = [](glm::dvec2 const &A, glm::dvec2 const &B) { return A.x * B.y - A.y * B.x; };
		for (auto const &sample : profile.samples)
		{
			auto const *track{profile.route.spans[sample.span].track};
			auto const cx{static_cast<int>(std::floor(sample.position.x / kPlanCell))};
			auto const cz{static_cast<int>(std::floor(sample.position.z / kPlanCell))};
			auto const along{glm::normalize(sample.direction)};
			for (int dx = -1; dx <= 1; ++dx)
				for (int dz = -1; dz <= 1; ++dz)
				{
					auto const cell{Points.find({cx + dx, cz + dz})};
					if (cell == Points.end())
						continue;
					for (auto const &point : cell->second)
					{
						glm::dvec2 const offset{point.position.x - sample.position.x, point.position.z - sample.position.z};
						auto const other{glm::normalize(point.direction)};
						auto const sine{across(along, other)};
						if (point.track == track || glm::length(offset) > kCrossing || std::abs(point.was - sample.position.y) > kSameLevel || std::abs(sine) < kCrossed)
							continue;
						// where the axes meet
						auto const here{across(offset, other) / sine};
						auto const there{across(offset, along) / sine};
						found.push_back({sample.chainage + here, point.position.y + there * point.grade + point.rails - track->rail_height(), std::abs(sine), glm::length(offset), point.track, glm::dvec2{sample.position.x, sample.position.z} + along * here, along});
					}
				}
		}
		// the nearest samples of each crossing tell it
		std::sort(found.begin(), found.end(), [](crossing const &A, crossing const &B) { return std::tie(A.track, A.chainage) < std::tie(B.track, B.chainage); });
		std::vector<crossing> result;
		for (auto const &one : found)
			if (one.chainage < 0.0 || one.chainage > profile.route.length || on_route(one.track))
				continue;
			else if (result.empty() || result.back().track != one.track || one.chainage - result.back().chainage > 5.0)
				result.push_back(one);
			else if (one.distance < result.back().distance)
				result.back() = one;
		return result;
	};
	// the height of a crossing of a line fitted already is ground the line keeps to as closely as to the switches beside, though
	// not past the curves of the norms: where an end fitted another way is too near, as near as they let it
	double constexpr kCrossingHeight{0.005}; // m
	struct crossing_hold
	{
		double chainage;
		double height;
		double tolerance;
		double reach;
	};
	std::vector<crossing_hold> crossing_holds;
	for (auto const &found : crossings_of(m_nmt_state.laid))
	{
		// the ground on both sides well below the track crossed, or well above it: one goes under the other, however the file has them,
		// which often lays both at one height. It's read past the formation of the track crossed, which takes the longer stretch of the
		// line the lower the angle: nearer the model shows that formation, or the cutting under it, and tells nothing of this line
		double constexpr kFormation{10.0}; // m to the side of the axis of the track crossed
		double constexpr kFormationReach{150.0}; // m along the line at most
		auto const reach{std::clamp(kFormation / found.sine, kCrossedReach, kFormationReach)};
		auto const terrain_towards = [&](int const Way) {
			auto const last{static_cast<std::ptrdiff_t>(count) - 1};
			auto const from{std::min(last, static_cast<std::ptrdiff_t>(std::distance(sampled.begin(), std::lower_bound(sampled.begin(), sampled.end(), found.chainage + Way * reach))))};
			for (auto at{from}; at >= 0 && at <= last; at += Way)
				if (std::isfinite(model[at]))
					return model[at];
			return std::numeric_limits<double>::quiet_NaN();
		};
		auto const before{terrain_towards(-1)};
		auto const after{terrain_towards(1)};
		auto const sides{std::isfinite(before) && std::isfinite(after)};
		auto const above{sides && before > found.height + kOtherSide && after > found.height + kOtherSide};
		auto const below{sides && before < found.height - kOtherSide && after < found.height - kOtherSide};
		if (above || below)
			other_levels.push_back({found.track, found.chainage, std::max(reach, kNear / found.sine), found.height});
		if (above)
			continue;
		if (below)
		{
			// under it far enough for a train and what carries the track above, whatever the model has of the formation of that track
			double constexpr kUnder{6.0}; // m, rail to rail
			double constexpr kUnderReach{30.0}; // m, the ground the line lets go of for it
			for (std::size_t i = 0; i < count; ++i)
				if (std::abs(sampled[i] - found.chainage) <= reach && model[i] > std::max(before, after) + kOtherSide)
					model[i] = std::numeric_limits<double>::quiet_NaN();
			crossing_holds.push_back({found.chainage, std::min(found.height - kUnder, std::max(before, after)), kRail, kUnderReach});
			continue;
		}
		crossing_holds.push_back({found.chainage, found.height, kCrossingHeight, kLevel});
	}
	for (std::size_t i = 0; i < count; ++i)
	{
		auto const &sample{profile.samples[i]};
		auto const *track{sample.span < profile.route.spans.size() ? profile.route.spans[sample.span].track : nullptr};
		chainages.push_back(sample.chainage);
		ground.push_back(model[i]);
		tolerances.push_back(kModel);
		reaches.push_back(kModel);
		double distance;
		auto const beside{nearest(sample.position, distance)};
		if (no_ground[i] == 0 && distance < kReach && (false == std::isfinite(model[i]) || std::abs(beside - model[i]) <= kLevel) && false == on_other_level(sample.chainage, beside))
		{
			chainages.push_back(sample.chainage);
			ground.push_back(beside);
			tolerances.push_back(std::max(kRail, kSpread * distance));
			reaches.push_back(kLevel);
		}
		if (track != nullptr && no_ground[i] == 0)
			for (auto const &[apart, point] : alongside(sample))
				if (point != nullptr)
				{
					chainages.push_back(sample.chainage);
					ground.push_back(point->position.y + point->rails - track->rail_height());
					tolerances.push_back(apart < kTimbers ? kTimbered : std::max(kRail, kSpread * apart));
					reaches.push_back(kHeld);
				}
		// a line fitted already crossing it or beside it on another level, one track going under the other, as authored: the difference
		// stays, whatever the model has there, which knows nothing of the cutting or of the bridge
		double constexpr kOtherLevel{2.5}; // m in the plan
		double constexpr kOtherFarthest{25.0}; // m in height, more isn't a track it has anything to do with
		double constexpr kOtherReach{30.0}; // m, the ground the line lets go of for it
		auto closest{kOtherLevel};
		nmt_state::laid_point const *other{nullptr};
		auto const cx{static_cast<int>(std::floor(sample.position.x / kPlanCell))};
		auto const cz{static_cast<int>(std::floor(sample.position.z / kPlanCell))};
		for (int dx = -1; dx <= 1; ++dx)
			for (int dz = -1; dz <= 1; ++dz)
				if (auto const cell{m_nmt_state.laid.find({cx + dx, cz + dz})}; cell != m_nmt_state.laid.end())
					for (auto const &point : cell->second)
					{
						auto const levels{std::abs(point.was - sample.position.y)};
						auto const distance{glm::length(glm::dvec2{point.position.x - sample.position.x, point.position.z - sample.position.z})};
						if (point.track != track && levels > kSameLevel && levels <= kOtherFarthest && distance < closest && false == on_other_level(sample.chainage, point.position.y))
						{
							closest = distance;
							other = &point;
						}
					}
		if (other != nullptr)
		{
			chainages.push_back(sample.chainage);
			ground.push_back(other->position.y + (sample.position.y - other->was));
			tolerances.push_back(kRail);
			reaches.push_back(kOtherReach);
		}
	}
	if (std::none_of(ground.begin(), ground.end(), [](double const Height) { return std::isfinite(Height); }))
	{
		nmt_note(nmt_state::finding::kind::refused, from, to, profile.route.length, middle, STR("The terrain model has no data under the line"));
		return false;
	}
	// no farther than the path crossed has rails: the middle of a double slip, a few metres crossed at a low angle, keeps to
	// itself, the switches about it have zones of their own
	for (auto const &found : crossings_of(m_nmt_state.paths))
	{
		// a track going over or under isn't crossed on the rails
		if (std::any_of(other_levels.begin(), other_levels.end(), [&](other_level const &Other) { return Other.track == found.track; }))
			continue;
		auto const half{std::min(30.0, kGauge / found.sine + 1.5)};
		auto first{std::numeric_limits<double>::max()};
		auto last{std::numeric_limits<double>::lowest()};
		for (auto const &path : found.track->m_paths)
			for (auto const end : {segment_data::point::start, segment_data::point::end})
			{
				auto const along{glm::dot(glm::dvec2{path.points[end].x, path.points[end].z} - found.at, found.along)};
				first = std::min(first, along);
				last = std::max(last, along);
			}
		auto const from{std::max(-half, first - 1.5)};
		auto const to{std::min(half, last + 1.5)};
		if (from < to)
			profile.context.switches.push_back({found.chainage + from, found.chainage + to, found.track->name()});
	}
	for (auto const &hold : crossing_holds)
	{
		auto const at{static_cast<std::size_t>(std::distance(chainages.begin(), std::upper_bound(chainages.begin(), chainages.end(), hold.chainage)))};
		chainages.insert(chainages.begin() + at, hold.chainage);
		ground.insert(ground.begin() + at, hold.height);
		tolerances.insert(tolerances.begin() + at, hold.tolerance);
		reaches.insert(reaches.begin() + at, hold.reach);
	}
	// the switches of the tracks beside, the ones of the crossovers above all, lie on bearers common with this track or close
	// to it: no break of the grade on them either, so the tracks of the station stay in one plane
	double constexpr kShared{7.0}; // m, about the spacing of the tracks
	for (auto *other : simulation::Paths.sequence())
	{
		if (other == nullptr || other->eType != tt_Switch || other->m_paths.empty() || false == rail(*other))
			continue;
		if (std::any_of(profile.route.spans.begin(), profile.route.spans.end(), [&](auto const &Span) { return Span.track == other; }))
			continue;
		auto const &points{other->m_paths.front().points};
		double from{std::numeric_limits<double>::max()};
		double to{std::numeric_limits<double>::lowest()};
		for (auto const end : {segment_data::point::start, segment_data::point::end})
		{
			auto const &point{points[end]};
			double best{kShared};
			double chainage{0.0};
			for (auto const &sample : profile.samples)
				if (auto const distance{glm::length(glm::dvec2{sample.position.x - point.x, sample.position.z - point.z})}; distance < best)
				{
					best = distance;
					chainage = sample.chainage;
				}
			if (best < kShared)
			{
				from = std::min(from, chainage);
				to = std::max(to, chainage);
			}
		}
		if (from < to)
			profile.context.switches.push_back({from, to, other->name()});
	}
	std::sort(profile.context.switches.begin(), profile.context.switches.end(), [](auto const &Left, auto const &Right) { return Left.from < Right.from; });
	auto settings{profile.line};
	settings.join_start = true;
	settings.join_end = true;
	settings.speed = editor_track::route_speed(profile.route);
	// the grades as steep as the ground has them, up to 50 per mille whatever the norms of the line; the vertical curves of the norms,
	// sharper only on a hump of a marshalling yard
	options.tolerance = kModel;
	options.grade_max = 50.0;
	options.follow_length = 60.0;
	options.follow_height = 4.0;
	options.follow_steepest = 100.0;
	options.follow_tolerance = 0.05;
	options.humps = profile::humps(chainages, ground, profile.context, kModel, tolerances);
	options.hump_radius = 300.0;
	for (auto const &hump : options.humps)
		profile.context.humps.push_back({hump.from, hump.to, {}});
	auto line{profile::fit_ground(chainages, ground, profile.context, settings, options, tolerances, reaches)};
	if (line.points.size() < 2)
	{
		nmt_note(nmt_state::finding::kind::refused, from, to, profile.route.length, middle, STR("No grade line fits the terrain model under the line"));
		return false;
	}
	profile.line = std::move(line);
	profile.selected = -1;
	profile_check();
	auto const fitted_line{profile.line};
	auto const authored{profile.samples};
	// where the model falls away under the line by the abutments, the line goes over on a bridge or a viaduct: its paths there
	// are cut off and set to the bridge, the formation is laid up to it, the ground under it stays as the model has it
	double constexpr kBridged{1.0}; // m
	double constexpr kAbutment{100.0}; // per mille
	// spans a few metres apart are one bridge on its piers; what is shorter than that is a culvert under the embankment
	double constexpr kPier{5.0}; // m
	double constexpr kCulvert{3.0}; // m
	double constexpr kFewest{0.03}; // m, how far a path just laid may part from the line before it's cut
	std::vector<double> heights(count);
	for (std::size_t i = 0; i < count; ++i)
		heights[i] = profile::elevation(fitted_line, sampled[i]);
	std::vector<profile::departure> bridges;
	for (auto const &stretch : Laid ? std::vector<profile::departure>{} : profile::bridged(sampled, heights, model, kBridged, kAbutment))
		if (false == bridges.empty() && stretch.from - bridges.back().to < kPier)
		{
			bridges.back().to = stretch.to;
			bridges.back().depth = std::min(bridges.back().depth, stretch.depth);
		}
		else
			bridges.push_back(stretch);
	bridges.erase(std::remove_if(bridges.begin(), bridges.end(), [&](profile::departure const &Bridge) { return Bridge.to - Bridge.from < kCulvert; }), bridges.end());
	profile.fitting = true;
	profile_apply(false, bridges, Laid ? kFewest : 0.0);
	profile.fitting = false;
	auto const applied{profile.error.empty()};
	// where the line parts from the model: under it a gap, a bridge or a viaduct, over it a wall or a structure above the track
	auto const parted{profile::departures(fitted_line, sampled, model, 2.0 * kModel)};
	if (false == parted.empty())
		++m_nmt_state.departures;
	std::size_t agreeing{0};
	std::size_t modelled{0};
	for (std::size_t i = 0; i < count; ++i)
		if (std::isfinite(model[i]))
		{
			++modelled;
			agreeing += std::abs(profile::elevation(fitted_line, sampled[i]) - model[i]) <= kModel ? 1 : 0;
		}
	// how soon past a switch or its shared timbers the grade changes, the nearest break or start of a curve of the line
	if (applied)
	{
		auto const breaks{profile::breaks(profile.line)};
		double nearest{std::numeric_limits<double>::max()};
		std::string where;
		for (auto const &zone : profile.context.switches)
			for (auto const at : breaks)
			{
				auto const gap{at >= zone.to ? at - zone.to : at <= zone.from ? zone.from - at : -1.0};
				if (gap >= 0.0 && gap < nearest)
				{
					nearest = gap;
					where = format("%.1f m %s %s (%.1f..%.1f), the break at %.1f", gap, at >= zone.to ? "past" : "before", zone.name.c_str(), zone.from, zone.to, at);
				}
			}
		if (false == where.empty())
			WriteLog(format("Editor: NMT line %s .. %s: the grade changes %s", from->name().c_str(), to->name().c_str(), where.c_str()), logtype::generic);
	}
	// where the paths didn't take the line
	double off{0.0};
	std::size_t off_at{0};
	if (applied)
		for (std::size_t i = 0; i < profile.samples.size(); ++i)
			if (auto const departure{std::abs(profile.samples[i].position.y - profile::elevation(profile.line, profile.samples[i].chainage))}; departure > off)
			{
				off = departure;
				off_at = i;
			}
	if (off > 0.05)
	{
		auto const &sample{profile.samples[off_at]};
		auto const *track{profile.route.spans[sample.span].track};
		WriteLog(format("Editor: NMT line %s: the path %s (%s, %.0f m) is off the line by %.3f m at %.0f m", Track.name().c_str(), track->name().c_str(), track->eType == tt_Switch ? "switch" : "track", track->Length(), off, sample.chainage), logtype::generic);
	}
	if (applied)
		for (auto const &hump : options.humps)
			for (auto const &span : profile.route.spans)
				if (span.from < hump.to && span.to > hump.from)
					m_nmt_state.on_hump.insert(span.track);
	if (applied)
		for (auto const &sample : profile.samples)
		{
			auto const *track{sample.span < profile.route.spans.size() ? profile.route.spans[sample.span].track : nullptr};
			m_nmt_state.laid[{static_cast<int>(std::floor(sample.position.x / kPlanCell)), static_cast<int>(std::floor(sample.position.z / kPlanCell))}].push_back(
			    {sample.position, sample.direction, sample.grade, editor_track::sampled_elevation(authored, sample.chainage), track != nullptr ? static_cast<double>(track->rail_height()) : 0.0, track});
		}
	auto const spans{applied ? bridges : std::vector<profile::departure>{}};
	for (auto const &stretch : spans)
		for (std::size_t i = 0; i < count; ++i)
			if (sampled[i] >= stretch.from && sampled[i] <= stretch.to)
				m_nmt_state.bridged[{static_cast<int>(std::floor(plan[i].x / kPlanCell)), static_cast<int>(std::floor(plan[i].y / kPlanCell))}].push_back({plan[i].x, heights[i], plan[i].y});
	// told in the panel too: the line refused and why, the path which didn't take it, the worst place where it parts from the model
	// other than over a bridge found
	auto const place_at = [&](double const Chainage) {
		return std::min_element(profile.samples.begin(), profile.samples.end(), [&](auto const &A, auto const &B) { return std::abs(A.chainage - Chainage) < std::abs(B.chainage - Chainage); })->position;
	};
	if (false == applied)
	{
		auto why{profile.error};
		for (auto const &issue : profile.issues)
			if (issue.error)
			{
				why += (why.empty() ? "" : ": ") + issue.text;
				break;
			}
		nmt_note(nmt_state::finding::kind::refused, from, to, profile.route.length, place_at(0.5 * profile.route.length), why);
	}
	else if (off > 0.05)
	{
		auto const &sample{profile.samples[off_at]};
		nmt_note(nmt_state::finding::kind::off, from, to, profile.route.length, sample.position, format(STR_C("The path %s keeps off the line by %.2f m"), profile.route.spans[sample.span].track->name().c_str(), off), off);
	}
	if (applied)
	{
		// a ditch or a culvert of a few metres under the line, a few decimetres of the model, is no news: what's long and deep is
		double constexpr kTold{0.5}; // m
		double constexpr kToldLength{10.0}; // m
		profile::departure const *worst{nullptr};
		for (auto const &stretch : parted)
			if (std::abs(stretch.depth) >= kTold && stretch.to - stretch.from >= kToldLength &&
			    false == std::any_of(spans.begin(), spans.end(), [&](profile::departure const &Bridge) { return stretch.depth < 0.0 && stretch.from < Bridge.to && stretch.to > Bridge.from; }) &&
			    (worst == nullptr || std::abs(stretch.depth) > std::abs(worst->depth)))
				worst = &stretch;
		if (worst != nullptr)
			nmt_note(nmt_state::finding::kind::parted, from, to, profile.route.length, place_at(0.5 * (worst->from + worst->to)),
			         worst->depth < 0.0 ? format(STR_C("Over a dip of the model %.1f m deep along %.0f m: a bridge, a culvert or the model off"), -worst->depth, worst->to - worst->from)
			                            : format(STR_C("Under the model by %.1f m along %.0f m: a cutting, a wall or the model off"), worst->depth, worst->to - worst->from),
			         std::abs(worst->depth));
	}
	// the line in the log: how much of it lies on the model, where it parts from it, its grades, why it wasn't applied, its
	// course in the plan
	std::string text{format("Editor: NMT line %s: %s .. %s, %.0f m, on the model within %.2f m over %.1f%%, parts from it", applied ? "applied" : "refused", from->name().c_str(), to->name().c_str(), profile.route.length, kModel,
	                        100.0 * static_cast<double>(agreeing) / static_cast<double>(std::max<std::size_t>(1, modelled)))};
	for (auto const &stretch : parted)
		text += format(" %s %.0f..%.0f %.1f m", stretch.depth < 0.0 ? "under" : "over", stretch.from, stretch.to, std::abs(stretch.depth));
	if (false == spans.empty())
	{
		text += ", bridged";
		for (auto const &stretch : spans)
			text += format(" %.0f..%.0f", stretch.from, stretch.to);
	}
	for (auto const &hump : options.humps)
		text += format(", a hump %.0f..%.0f, its crest at %.0f %.1f m high", hump.from, hump.to, hump.crest, hump.height);
	text += ", grades";
	for (std::size_t i = 1; i < fitted_line.points.size(); ++i)
	{
		auto const &a{fitted_line.points[i - 1]};
		auto const &b{fitted_line.points[i]};
		text += format(" %+.2f/%.0f", 1000.0 * (b.elevation - a.elevation) / std::max(1e-6, b.chainage - a.chainage), b.chainage - a.chainage);
	}
	if (false == applied)
	{
		text += "; " + profile.error;
		for (auto const &issue : profile.issues)
			if (issue.error)
				text += " | " + issue.text;
	}
	text += "; plan";
	for (std::size_t i = 0; i < profile.samples.size(); i += 10)
		text += format(" %.1f,%.1f", profile.samples[i].position.x, profile.samples[i].position.z);
	text += format(" %.1f,%.1f", profile.samples.back().position.x, profile.samples.back().position.z);
	WriteLog(text, logtype::generic);
	return applied;
}

void editor_mode::nmt_sample_paths(TTrack const &Track, std::map<std::pair<int, int>, std::vector<nmt_state::laid_point>> &Points)
{
	// the bridge of a turntable turns: it crosses no line nor lies beside one
	if (Track.eType == tt_Table)
		return;
	for (auto const &path : Track.m_paths)
	{
		geometry::bezier const curve{path};
		auto const chord{glm::distance(path.points[segment_data::point::start], path.points[segment_data::point::end])};
		auto const count{std::max(1, static_cast<int>(std::ceil(chord * 1.1)))};
		for (int i = 0; i <= count; ++i)
		{
			auto const t{static_cast<double>(i) / count};
			auto const at{curve.point(t)};
			auto const tangent{curve.first(t)};
			auto const run{glm::length(glm::dvec2{tangent.x, tangent.z})};
			if (run > 1e-9)
				Points[{static_cast<int>(std::floor(at.x / kPlanCell)), static_cast<int>(std::floor(at.z / kPlanCell))}].push_back(
				    {at, glm::dvec2{tangent.x, tangent.z} / run, tangent.y / run, at.y, static_cast<double>(Track.rail_height()), &Track});
		}
	}
}

void editor_mode::nmt_merge_laid(std::unordered_set<TTrack const *> &Laid)
{
	auto const direction = [](segment_data const &Path) {
		auto const control{geometry::plan_of(Path.points[segment_data::point::control1])};
		auto const chord{geometry::plan_of(Path.points[segment_data::point::end] - Path.points[segment_data::point::start])};
		auto const along{glm::length(control) > 1e-6 ? control : chord};
		return along / std::max(1e-12, glm::length(along));
	};
	auto const straight = [&](segment_data const &Path) {
		if (std::abs(Path.radius) > 1e-3f)
			return false;
		auto const along{direction(Path)};
		return std::abs(geometry::cross(along, geometry::plan_of(Path.points[segment_data::point::end] - Path.points[segment_data::point::start]))) < 1e-3
		    && std::abs(geometry::cross(along, geometry::plan_of(Path.points[segment_data::point::control2]))) < 1e-3;
	};
	struct circle
	{
		glm::dvec2 centre{0.0};
		double radius{0.0};
		int side{0};
		double angle{0.0};
	};
	auto const arc = [&](segment_data const &Path) -> std::optional<circle> {
		auto const radius{std::abs(static_cast<double>(Path.radius))};
		if (radius < 1e-3 || glm::length(geometry::plan_of(Path.points[segment_data::point::control1])) < 1e-6)
			return std::nullopt;
		auto const start{geometry::plan_of(Path.points[segment_data::point::start])};
		auto const end{geometry::plan_of(Path.points[segment_data::point::end])};
		auto const along{direction(Path)};
		auto const side{geometry::cross(along, end - start) > 0.0 ? 1 : -1};
		circle result{start + glm::dvec2{-along.y, along.x} * (side * radius), radius, side, 0.0};
		if (std::abs(glm::distance(result.centre, end) - radius) > 0.01)
			return std::nullopt;
		result.angle = std::abs(geometry::signed_angle(start - result.centre, end - result.centre));
		return result;
	};
	auto const alike = [](TTrack const &A, TTrack const &B) {
		return editor_track::velocity(A) == editor_track::velocity(B) && A.eEnvironment == B.eEnvironment && A.iCategoryFlag == B.iCategoryFlag && A.iDamageFlag == B.iDamageFlag;
	};
	editor_track::edit edit;
	for (bool merged{true}; merged;)
	{
		merged = false;
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || track->m_editorremoved || false == Laid.contains(track) || track->eType != tt_Normal || track->m_paths.size() != 1)
				continue;
			auto *next{track->trNext};
			if (next == nullptr || next == track || next->m_editorremoved || false == Laid.contains(next) || next->eType != tt_Normal || next->m_paths.size() != 1 || next->trPrev != track || false == alike(*track, *next))
				continue;
			auto const &first{track->m_paths.front()};
			auto const &second{next->m_paths.front()};
			if (glm::distance(first.points[segment_data::point::end], second.points[segment_data::point::start]) > 1e-3)
				continue;
			auto const length{geometry::bezier(first).plan_length() + geometry::bezier(second).plan_length()};
			if (length > geometry::longest_path)
				continue;
			auto path{first};
			path.pieces.clear();
			if (straight(first) && straight(second) && std::abs(geometry::cross(direction(first), direction(second))) < 1e-5 && glm::dot(direction(first), direction(second)) > 0.0)
			{
				path.points[segment_data::point::control1] = {};
				path.points[segment_data::point::control2] = {};
				path.radius = 0.f;
			}
			else if (auto const a{arc(first)}, b{arc(second)};
			         a && b && a->side == b->side && std::abs(a->radius - b->radius) < 1e-3 * a->radius && glm::distance(a->centre, b->centre) < 0.01 && a->angle + b->angle <= glm::radians(90.0))
			{
				auto const pieces{geometry::arc_pieces(first.points[segment_data::point::start], direction(first), 0.0, a->radius, a->angle + b->angle, a->side)};
				if (pieces.size() != 1)
					continue;
				path.points[segment_data::point::control1] = pieces.front().points[segment_data::point::control1];
				path.points[segment_data::point::control2] = pieces.front().points[segment_data::point::control2];
			}
			else
				continue;
			path.points[segment_data::point::end] = second.points[segment_data::point::end];
			path.rolls[1] = second.rolls[1];
			edit.capture(*track);
			editor_track::retire(*next);
			Laid.erase(next);
			edit.removed.push_back(next);
			track->m_paths.front() = path;
			editor_track::commit({track});
			merged = true;
		}
	}
	if (false == edit.removed.empty())
		push_track_snapshot(std::move(edit.states), {}, std::move(edit.removed));
}

bool editor_mode::nmt_fit_laid(std::size_t const From, glm::dvec3 const End)
{
	auto &state{m_nmt_state};
	if (EditorSettings.heights() != editorSettings::lay_heights::fitted || state.stage != nmt_state::automatic::idle || false == m_nmt.placed() || From >= m_history.size())
		return false;
	std::unordered_set<TTrack const *> laid;
	for (auto step{From}; step < m_history.size(); ++step)
		for (auto const *track : m_history[step].created)
			if (track != nullptr && false == track->m_editorremoved && rail(*track))
				laid.insert(track);
	auto const at_end = [&]() -> TTrack * {
		for (auto *track : simulation::Paths.sequence())
			if (track != nullptr && laid.contains(track) && track->eType == tt_Normal && geometry::plan_distance(track->m_paths.front().points[segment_data::point::end], End) < 0.01)
				return track;
		return nullptr;
	};
	auto const steps{m_history.size()};
	nmt_merge_laid(laid);
	auto *last{at_end()};
	if (last == nullptr)
		return false;
	// the tracks around stay as they are, as lines fitted already: the line keeps level with the ones beside it, the more closely
	// the nearer they are, and crosses the ones it crosses at their height
	double constexpr kAround{60.0}; // m
	glm::dvec2 low{std::numeric_limits<double>::max()};
	glm::dvec2 high{std::numeric_limits<double>::lowest()};
	for (auto const *track : laid)
		for (auto const &path : track->m_paths)
			for (auto const end : {segment_data::point::start, segment_data::point::end})
			{
				low = glm::min(low, geometry::plan_of(path.points[end]));
				high = glm::max(high, geometry::plan_of(path.points[end]));
			}
	low -= kAround;
	high += kAround;
	std::map<std::pair<int, int>, std::vector<nmt_state::laid_point>> around;
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || false == rail(*track) || laid.contains(track))
			continue;
		auto const &points{track->m_paths.front().points};
		auto const a{geometry::plan_of(points[segment_data::point::start])};
		auto const b{geometry::plan_of(points[segment_data::point::end])};
		if (std::max(a.x, b.x) < low.x || std::min(a.x, b.x) > high.x || std::max(a.y, b.y) < low.y || std::min(a.y, b.y) > high.y)
			continue;
		nmt_sample_paths(*track, around);
	}
	// the state of the tracks going onto the terrain lent to the one line, the profile tool left as it was
	auto const profile{m_profile};
	auto fitted_lines{std::exchange(state.laid, around)};
	auto paths{std::exchange(state.paths, std::move(around))};
	auto loose{std::exchange(state.loose, {})};
	auto turntables{std::exchange(state.turntables, nmt_turntables())};
	auto const departures{state.departures};
	auto const applied{nmt_fit_line(*last, [&](TTrack const *Other) { return false == laid.contains(Other); }, true)};
	auto const error{m_profile.error};
	state.laid = std::move(fitted_lines);
	state.paths = std::move(paths);
	state.loose = std::move(loose);
	state.turntables = std::move(turntables);
	state.departures = departures;
	m_profile = profile;
	// taken back with the laying
	for (auto step{steps}; step < m_history.size(); ++step)
		m_history[step].joined = true;
	if (applied)
	{
		laid.clear();
		for (auto step{From}; step < m_history.size(); ++step)
			for (auto const *track : m_history[step].created)
				if (track != nullptr && false == track->m_editorremoved)
					laid.insert(track);
		if (auto *track{at_end()}; track != nullptr)
		{
			m_node = track;
			ui()->set_node(m_node);
		}
	}
	else if (false == error.empty())
		WriteLog("Editor: the track laid not put onto the terrain model - " + error, logtype::generic);
	return applied;
}

namespace
{

// things joined one to another in sets, each set told by its root
template <typename Thing, typename Map = std::unordered_map<Thing, Thing>>
class joined_sets
{
public:
	Thing root(Thing Of) const
	{
		for (auto found{m_parent.find(Of)}; found != m_parent.end() && found->second != Of; found = m_parent.find(Of))
			Of = found->second;
		return Of;
	}
	// the set of the first root goes into the set of the second
	void join_roots(Thing const &A, Thing const &B)
	{
		m_parent[A] = B;
		m_parent.try_emplace(B, B);
	}
	void join(Thing const &A, Thing const &B)
	{
		if (auto const a{root(A)}, b{root(B)}; a != b)
			join_roots(a, b);
	}
	// joined to anything at all
	bool joined(Thing const &Of) const
	{
		return m_parent.contains(Of);
	}

private:
	Map m_parent;
};

// of the least squares the moves of the grade lines shared are: the sum of the unknowns at, each by its factor, is to come to the value,
// as closely as the weight tells; an unknown under 0 ends the ones taken
struct grade_term
{
	std::array<int, 5> at;
	std::array<double, 5> by;
	double value;
	double weight;
};

// the unknowns of the least squares of the terms, by conjugate gradients on the normal equations, with the iterations it took
std::vector<double> grade_moves(std::vector<grade_term> const &Terms, int const Unknowns, int &Iterations)
{
	auto const multiply = [&](std::vector<double> const &In, std::vector<double> &Out) {
		std::fill(Out.begin(), Out.end(), 0.0);
		for (auto const &one : Terms)
		{
			auto sum{0.0};
			for (int k = 0; k < 5 && one.at[k] >= 0; ++k)
				sum += one.by[k] * In[one.at[k]];
			sum *= one.weight;
			for (int k = 0; k < 5 && one.at[k] >= 0; ++k)
				Out[one.at[k]] += one.by[k] * sum;
		}
	};
	std::vector<double> moves(Unknowns, 0.0), residual(Unknowns, 0.0), diagonal(Unknowns, 0.0);
	for (auto const &one : Terms)
		for (int k = 0; k < 5 && one.at[k] >= 0; ++k)
		{
			residual[one.at[k]] += one.weight * one.by[k] * one.value;
			diagonal[one.at[k]] += one.weight * one.by[k] * one.by[k];
		}
	std::vector<double> direction(Unknowns), product(Unknowns);
	auto rho{0.0};
	for (int i = 0; i < Unknowns; ++i)
	{
		direction[i] = residual[i] / diagonal[i];
		rho += residual[i] * direction[i];
	}
	auto const first_rho{std::max(rho, 1e-30)};
	for (Iterations = 0; Iterations < 30000 && rho > 1e-14 * first_rho; ++Iterations)
	{
		multiply(direction, product);
		auto bottom{0.0};
		for (int i = 0; i < Unknowns; ++i)
			bottom += direction[i] * product[i];
		if (bottom <= 0.0)
			break;
		auto const step{rho / bottom};
		auto next{0.0};
		for (int i = 0; i < Unknowns; ++i)
		{
			moves[i] += step * direction[i];
			residual[i] -= step * product[i];
			next += residual[i] * residual[i] / diagonal[i];
		}
		auto const beta{next / rho};
		rho = next;
		for (int i = 0; i < Unknowns; ++i)
			direction[i] = residual[i] / diagonal[i] + beta * direction[i];
	}
	return moves;
}

// the grade a path rises out of an end with, by its control there: a path on a vertical curve rises otherwise than along its chord, a
// long one with the whole curve on it by 16 permille
double rise_out(segment_data const &Path, bool const Atend)
{
	auto const &points{Path.points};
	auto const control{points[Atend ? segment_data::point::control2 : segment_data::point::control1]};
	auto const along{glm::length(geometry::plan_of(control))};
	if (along > 1e-6)
		return control.y / along;
	auto const length{geometry::plan_distance(points[segment_data::point::start], points[segment_data::point::end])};
	return (points[segment_data::point::end].y - points[segment_data::point::start].y) * (Atend ? -1.0 : 1.0) / std::max(1e-9, length);
}

// a path going out of a joint: of which track, the way and the grade it leaves the joint with
struct leaving
{
	TTrack const *track;
	glm::dvec2 way;
	double grade; // rising out of the joint
};

leaving leaving_of(TTrack const *Track, segment_data const &Path, bool const Atend)
{
	auto const &points{Path.points};
	auto const control{geometry::plan_of(points[Atend ? segment_data::point::control2 : segment_data::point::control1])};
	auto const chord{geometry::plan_of(points[segment_data::point::end] - points[segment_data::point::start]) * (Atend ? -1.0 : 1.0)};
	auto const rise{(points[segment_data::point::end].y - points[segment_data::point::start].y) * (Atend ? -1.0 : 1.0)};
	if (glm::length(control) > 1e-6)
		return leaving{Track, glm::normalize(control), points[Atend ? segment_data::point::control2 : segment_data::point::control1].y / glm::length(control)};
	return leaving{Track, chord / std::max(1e-9, glm::length(chord)), rise / std::max(1e-9, glm::length(chord))};
}

// the path on the cubic of these grades along it from end to end, its pieces too
void on_vertical_curve(segment_data &Path, double const First, double const Last)
{
	auto const origin{Path.points[segment_data::point::start]};
	auto const chord{geometry::plan_of(Path.points[segment_data::point::end] - origin)};
	auto const square{std::max(1e-9, glm::dot(chord, chord))};
	auto const length{std::sqrt(square)};
	auto const rise{Path.points[segment_data::point::end].y - origin.y};
	auto const second{3.0 * rise - (2.0 * First + Last) * length};
	auto const third{(First + Last) * length - 2.0 * rise};
	auto const height = [&](double const U) { return origin.y + U * (First * length + U * (second + U * third)); };
	auto const slope = [&](double const U) { return (First * length + U * (2.0 * second + U * 3.0 * third)) / length; };
	auto const lay = [&](segment_data &Part) {
		auto &points{Part.points};
		if (points[segment_data::point::control1] == glm::dvec3{} && points[segment_data::point::control2] == glm::dvec3{})
		{
			points[segment_data::point::control1] = (points[segment_data::point::end] - points[segment_data::point::start]) / 3.0;
			points[segment_data::point::control2] = (points[segment_data::point::start] - points[segment_data::point::end]) / 3.0;
		}
		auto const along = [&](glm::dvec3 const &Point) { return std::clamp(glm::dot(geometry::plan_of(Point - origin), chord) / square, 0.0, 1.0); };
		auto const from{along(points[segment_data::point::start])};
		auto const to{along(points[segment_data::point::end])};
		points[segment_data::point::control1].y = slope(from) * glm::length(geometry::plan_of(points[segment_data::point::control1]));
		points[segment_data::point::control2].y = -slope(to) * glm::length(geometry::plan_of(points[segment_data::point::control2]));
		points[segment_data::point::start].y = height(from);
		points[segment_data::point::end].y = height(to);
	};
	for (auto &piece : Path.pieces)
		lay(piece);
	lay(Path);
}

} // namespace

void editor_mode::nmt_share_grades()
{
	// one grade line for the tracks side by side: over the grade lines fitted one by one, what each joint moves by is worked out for all
	// of them at once. Tracks nearly parallel under 5.1 m apart come to one height, the moves change evenly along a line, so what was fitted stays
	// smooth, and each joint keeps to where it was as far as these let it: a track going away from the others is on its own again.
	// Tracks over 1 m apart in height lie on another level; a long steep one, a hump, stays as it is and tells nothing to those beside it.
	// It's told for a whole run of tracks from switch to switch: one with nothing beside it over 15 m and a quarter of its length has a grade line of its own.
	// The tracks and switches on common timbers lie on one surface whatever else is told of them, a plane tilting along and across as the
	// ground goes, each such group its own; on it they come to one grade line as far as tracks side by side do. A switch lies in its plane
	auto &state{m_nmt_state};
	// nothing fitted now, nothing to share: what stays lies as it was
	if (false == state.common || (false == state.stayed.empty() && state.lines == 0))
		return;
	double constexpr kApart{5.1}; // m between the axes
	double constexpr kParallel{0.985}; // the cosine of the angle of tracks nearly parallel, 10 degrees: the links of a crossover are, a track crossing isn't
	double constexpr kLevel{1.0}; // m, as the lines keep to tracks on another level
	double constexpr kSteep{0.025};
	double constexpr kSteepLength{15.0}; // m, shorter ones are the links of a crossover between tracks at two heights
	double constexpr kSpacing{5.0}; // m between the places looked at on a track
	double constexpr kMoved{0.00001}; // m, any less stays: what a joint moves by goes on all the tracks at it, a millimetre left on a short one is a grade of its own
	// how closely each is kept to, the weights being one over the square
	double constexpr kKeep{0.5}; // m, a joint to where it was
	double constexpr kBeside{0.01}; // m, tracks side by side
	double constexpr kEven{0.0005}; // the change of grade the moves make at a joint
	double constexpr kTimbers{2.65}; // m between the axes, about the length of a sleeper
	double constexpr kOnTimbers{0.001}; // m, how closely tracks on common timbers keep to the surface of their group
	double constexpr kPlane{0.005}; // m, how closely the second path of a switch keeps to the plane of the first
	double constexpr kAway{15.0}; // m of a run with no track beside it, as much as its ends at the switches may part from the others
	double constexpr kBroken{0.015}; // a grade breaking more than this at a joint is to go, a smaller one is the vertical curve fitted
	using key = std::pair<int, int>;
	auto const key_of = [](glm::dvec3 const &Point) { return key{static_cast<int>(std::round(Point.x * 20.0)), static_cast<int>(std::round(Point.z * 20.0))}; };
	std::vector<TTrack *> tracks;
	for (auto *track : simulation::Paths.sequence())
		if (track != nullptr && false == track->m_editorremoved && rail(*track) && track->eType != tt_Table)
			tracks.push_back(track);
	auto const steep = [&](TTrack const &Track) {
		if (state.on_hump.contains(&Track))
			return true;
		return std::any_of(Track.m_paths.begin(), Track.m_paths.end(), [&](segment_data const &Path) {
			auto const &start{Path.points[segment_data::point::start]};
			auto const &end{Path.points[segment_data::point::end]};
			auto const length{geometry::plan_distance(start, end)};
			return length >= kSteepLength && std::abs(end.y - start.y) > kSteep * length;
		});
	};
	struct link
	{
		int to;
		double length;
		glm::dvec2 out;
		double rise; // the grade rising out of the joint along the path, by its control there
	};
	std::map<key, int> indices;
	std::vector<std::vector<link>> links;
	std::vector<char> fixed;
	std::vector<double> heights;
	auto const index_of = [&](glm::dvec3 const &Point) {
		auto const [found, added]{indices.try_emplace(key_of(Point), static_cast<int>(links.size()))};
		if (added)
		{
			links.emplace_back();
			fixed.push_back(0);
			heights.push_back(Point.y);
		}
		return found->second;
	};
	struct place
	{
		glm::dvec3 position;
		glm::dvec2 along;
		double share;
		int first;
		int second;
		TTrack const *track;
		int path{0};
		std::size_t index{0};
		double length{0.0}; // m of the track it stands for, half a step at an end: a joint is shared by the tracks meeting at it
	};
	std::vector<place> all;
	std::vector<place> timber_all; // of all the tracks, the ones held apart too
	std::unordered_set<int> hump_nodes;
	std::unordered_set<int> stayed_nodes;
	for (auto const *track : tracks)
	{
		auto const held{steep(*track)};
		for (auto const &path : track->m_paths)
		{
			auto const &start{path.points[segment_data::point::start]};
			auto const &end{path.points[segment_data::point::end]};
			auto const a{index_of(start)};
			auto const b{index_of(end)};
			auto const chord{geometry::plan_of(end - start)};
			auto const length{glm::length(chord)};
			if (a == b || length < 1e-3)
				continue;
			links[a].push_back({b, length, chord / length, rise_out(path, false)});
			links[b].push_back({a, length, -chord / length, rise_out(path, true)});
			// the joints of a switch keep its heights: a track ending there meets it
			if (track->eType == tt_Switch)
			{
				heights[a] = start.y;
				heights[b] = end.y;
			}
			if (held)
				fixed[a] = fixed[b] = 1;
			if (state.on_hump.contains(track))
				hump_nodes.insert({a, b});
			// what stays where it is holds its ends whatever comes up to them; nothing else moves it
			if (state.stayed.contains(track))
			{
				heights[a] = start.y;
				heights[b] = end.y;
				fixed[a] = fixed[b] = 1;
				stayed_nodes.insert({a, b});
			}
			geometry::bezier const curve{path};
			// no farther apart than the spacing: a track beside is looked for within 0.6 of it along, sparser places leave it unseen
			auto const count{std::max(1, static_cast<int>(std::ceil(length / kSpacing)))};
			for (int i = 0; i <= count; ++i)
			{
				auto const share{static_cast<double>(i) / count};
				auto const tangent{geometry::plan_of(curve.first(share))};
				place const here{curve.point(share), glm::length(tangent) > 1e-9 ? glm::normalize(tangent) : chord / length, share, a, b, track, static_cast<int>(&path - track->m_paths.data()), timber_all.size(), length / count * (i == 0 || i == count ? 0.5 : 1.0)};
				timber_all.push_back(here);
				if (held)
					continue;
				all.push_back(here);
			}
		}
	}
	// the places in squares of a size, the ones around a place looked through for what's beside it
	using cells = std::map<key, std::vector<place const *>>;
	auto const cell_at = [](glm::dvec3 const &Point, double const Size) { return key{static_cast<int>(std::floor(Point.x / Size)), static_cast<int>(std::floor(Point.z / Size))}; };
	auto const cells_of = [&](std::vector<place> const &Places, double const Size, auto const &Taken) {
		cells result;
		for (auto const &here : Places)
			if (Taken(here))
				result[cell_at(here.position, Size)].push_back(&here);
		return result;
	};
	auto const around = [&](cells const &Cells, glm::dvec3 const &Point, double const Size, auto const &Each) {
		auto const cell{cell_at(Point, Size)};
		for (int dx = -1; dx <= 1; ++dx)
			for (int dz = -1; dz <= 1; ++dz)
				if (auto const found{Cells.find({cell.first + dx, cell.second + dz})}; found != Cells.end())
					for (auto const *other : found->second)
						Each(*other);
	};
	auto const every = [](place const &) { return true; };
	auto const places{cells_of(all, kApart, every)};
	auto const timber_places{cells_of(timber_all, kApart, every)};
	std::vector<grade_term> terms;
	auto const nodes{static_cast<int>(links.size())};
	// a dead end holds firmer: nothing past it, the even moves would carry the tilt of the station on along the run as on a lever, the
	// end of a line at the edge of the scenery going up by metres
	double constexpr kDeadEnd{0.05}; // m
	for (int i = 0; i < nodes; ++i)
		terms.push_back({{i, -1, -1, -1}, {1.0, 0.0, 0.0, 0.0}, 0.0, fixed[i] != 0 ? 1e8 : links[i].size() == 1 ? 1.0 / (kDeadEnd * kDeadEnd) : 1.0 / (kKeep * kKeep)});
	for (int i = 0; i < nodes; ++i)
		for (std::size_t a = 0; a < links[i].size(); ++a)
			for (std::size_t b = a + 1; b < links[i].size(); ++b)
				if (glm::dot(links[i][a].out, links[i][b].out) < -0.7)
				{
					auto const in{1.0 / links[i][a].length};
					auto const on{1.0 / links[i][b].length};
					// the grade the joint breaks by, of the paths at it, not of their chords: the chords of a curve on few paths break where
					// the paths don't, and taking that out would break them
					auto const change{links[i][b].rise + links[i][a].rise};
					// by the true lengths, an even move breaks no grade on a short track either; what a track under 0.5 m has of it goes by height, the numbers staying of a size
					auto const scale{std::min(1.0, std::min(links[i][a].length, links[i][b].length) / 0.5)};
					terms.push_back({{links[i][b].to, i, links[i][a].to, -1}, {scale * on, -scale * (on + in), scale * in, 0.0}, std::abs(change) > kBroken ? -scale * change : 0.0, 1.0 / (kEven * kEven)});
				}
	// the runs from switch to switch: plain tracks meeting at a joint nothing else meets at
	std::vector<std::vector<TTrack const *>> at_node(nodes);
	for (auto const *track : tracks)
		for (auto const &path : track->m_paths)
			for (auto const end : {segment_data::point::start, segment_data::point::end})
				at_node[indices.at(key_of(path.points[end]))].push_back(track);
	joined_sets<TTrack const *> runs;
	for (auto const &meeting : at_node)
		if (meeting.size() == 2 && meeting[0] != meeting[1] && meeting[0]->eType == tt_Normal && meeting[1]->eType == tt_Normal)
			runs.join(meeting[0], meeting[1]);
	// who was put on a grade line by hand stays on it, with the others put on it wherever they lie, never on another one so put
	grade_lines_load();
	std::unordered_map<TTrack const *, int> held;
	for (auto const *track : tracks)
	{
		auto const key{grade_line_key(*track)};
		if (auto const found{state.grade_lines.find(key)}; found != state.grade_lines.end() && found->second >= 0 && state.grade_lines_hand.contains(key))
			held[track] = found->second;
	}
	// the nearest place of another track on each side of one, of those taken, under the reach from it: nearly parallel to it, on its
	// level and across the track, not past its end - the next piece of a line isn't beside it
	auto const nearest_of = [&](place const &Here, cells const &Cells, double const Reach, auto const &Taken) {
		std::array<std::pair<double, place const *>, 2> sides{std::pair{Reach, nullptr}, std::pair{Reach, nullptr}};
		around(Cells, Here.position, Reach, [&](place const &Other) {
			if (Other.track == Here.track || Other.first == Here.first || Other.first == Here.second || Other.second == Here.first || Other.second == Here.second || false == Taken(Other))
				return;
			glm::dvec2 const offset{Other.position.x - Here.position.x, Other.position.z - Here.position.z};
			if (std::abs(glm::dot(offset, Here.along)) > 0.6 * kSpacing || std::abs(Other.position.y - Here.position.y) > kLevel || std::abs(glm::dot(Here.along, Other.along)) < kParallel)
				return;
			auto const across{Here.along.x * offset.y - Here.along.y * offset.x};
			auto &side{sides[across > 0.0 ? 0 : 1]};
			if (std::abs(across) < side.first)
				side = {std::abs(across), &Other};
		});
		return sides;
	};
	std::unordered_set<TTrack const *> apart;
	// the nearest track on each side, of the ones not held apart
	auto const nearest_sides = [&](place const &Here) {
		return nearest_of(Here, places, kApart, [&](place const &Other) { return false == apart.contains(runs.root(Other.track)) || held.contains(Other.track); });
	};
	// two places side by side at one height: the joints of each move as the places lie between them
	auto const side_by_side = [&](place const &Here, place const &Other) {
		terms.push_back({{Here.first, Here.second, Other.first, Other.second}, {1.0 - Here.share, Here.share, -(1.0 - Other.share), -Other.share}, Other.position.y - Here.position.y, 1.0 / (kBeside * kBeside)});
	};
	// the run of a track named by a selftest: how long, the places of it with nothing beside, whether it went on its own and on which line
	auto const explain = [&](char const *Stage) {
		for (auto const *track : tracks)
		{
			if (std::string::npos == state.explain.find(" " + track->name() + " "))
				continue;
			auto const *run{runs.root(track)};
			std::size_t members{0};
			auto length{0.0};
			for (auto const *other : tracks)
				if (runs.root(other) == run)
				{
					++members;
					for (auto const &path : other->m_paths)
						length += geometry::plan_distance(path.points[segment_data::point::start], path.points[segment_data::point::end]);
				}
			std::size_t lonely{0};
			std::string where;
			for (auto const &here : all)
				if (runs.root(here.track) == run && here.track->eType == tt_Normal)
					if (auto const sides{nearest_sides(here)}; sides[0].second == nullptr && sides[1].second == nullptr)
					{
						++lonely;
						if (lonely <= 40)
							where += format(" %s(%.0f,%.0f,%.2f)", here.track->name().c_str(), here.position.x, here.position.z, here.position.y);
					}
			auto const found{state.grade_lines.find(grade_line_key(*track))};
			WriteLog(format("Editor: NMT share, %s: %s in a run of %zu tracks over %.0f m from %s, apart %d, line %d, %zu places with nothing beside:%s", Stage, track->name().c_str(), members, length,
			                run->name().c_str(), apart.contains(run) ? 1 : 0, found != state.grade_lines.end() ? found->second : -100, lonely, where.c_str()),
			         logtype::generic);
		}
	};
	{
		// a run beside the others nearly all along stays with them: the switches at its ends often lie a few tens of metres off theirs
		double constexpr kAwayShare{0.25};
		std::unordered_map<TTrack const *, double> alone, whole;
		for (auto const &here : all)
			if (here.track->eType == tt_Normal)
			{
				whole[runs.root(here.track)] += here.length;
				if (auto const sides{nearest_sides(here)}; sides[0].second == nullptr && sides[1].second == nullptr)
					alone[runs.root(here.track)] += here.length;
			}
		explain("alone");
		for (auto const &[run, length] : alone)
			if (length > kAway && length > kAwayShare * whole[run])
				apart.insert(run);
	}
	// who was taken off a grade line by hand stays off them
	auto const taken_off = [&](TTrack const &Track) {
		auto const found{state.grade_lines.find(grade_line_key(Track))};
		return found != state.grade_lines.end() && (found->second == nmt_state::grade_line_off || found->second <= nmt_state::grade_line_off_number);
	};
	std::unordered_set<TTrack const *> off;
	for (auto const *track : tracks)
		if (taken_off(*track))
		{
			off.insert(track);
			apart.insert(runs.root(track));
		}
	std::size_t pairs{0};
	// the tracks on one grade line together, as they're tied side by side and along the runs
	joined_sets<TTrack const *> tied;
	// of a group tied, the grade line one of it was put on by hand: two groups of two such lines don't tie
	std::unordered_map<TTrack const *, int> tied_held(held.begin(), held.end());
	auto const held_of = [&](TTrack const *Root) {
		auto const found{tied_held.find(Root)};
		return found != tied_held.end() ? found->second : -1;
	};
	auto const tie = [&](TTrack const *A, TTrack const *B) {
		auto const *a{tied.root(A)};
		auto const *b{tied.root(B)};
		if (a == b)
			return true;
		auto const of_a{held_of(a)};
		auto const of_b{held_of(b)};
		if (of_a >= 0 && of_b >= 0 && of_a != of_b)
			return false;
		tied.join_roots(a, b);
		if (of_b < 0 && of_a >= 0)
			tied_held[b] = of_a;
		return true;
	};
	{
		std::unordered_map<int, TTrack const *> first;
		for (auto const &[track, line] : held)
			if (auto const [found, added]{first.try_emplace(line, track)}; false == added)
				tie(track, found->second);
	}
	for (auto const &here : all)
	{
		auto const inapart{apart.contains(runs.root(here.track))};
		if (inapart && false == held.contains(here.track))
			continue;
		for (auto const &[distance, other] : nearest_sides(here))
			if (other != nullptr)
			{
				if (false == tie(here.track, other->track))
					continue;
				++pairs;
				if (false == inapart)
					tie(here.track, runs.root(here.track));
				side_by_side(here, *other);
			}
	}
	// the tracks put on one line by hand further apart than the ones side by side, across a platform say, go up and down together too:
	// each place of them with the nearest place of another track of that line on each side
	double constexpr kHandApart{40.0}; // m between the axes
	std::size_t hand_pairs{0};
	if (false == held.empty())
	{
		auto const held_places{cells_of(all, kHandApart, [&](place const &Here) { return held.contains(Here.track); })};
		for (auto const &[cell, those] : held_places)
			for (auto const *here : those)
			{
				auto const line{held.at(here->track)};
				for (auto const &[distance, other] : nearest_of(*here, held_places, kHandApart, [&](place const &Other) { return held.at(Other.track) == line; }))
				{
					// the ones side by side are tied already
					if (other == nullptr || distance < kApart)
						continue;
					++hand_pairs;
					side_by_side(*here, *other);
				}
			}
	}
	// the groups on common timbers: tracks and switches tied by the places under a sleeper's length apart, each group on a plane of its
	// own, whose three numbers are worked out with the moves
	std::size_t on_timbers{0};
	// of a plain track only the stretch under the timbers counts: a long one with switches at both ends doesn't make one group of them
	double constexpr kStretch{40.0}; // m
	using stretch = std::tuple<TTrack const *, int, int>;
	auto const stretch_of = [&](place const &Here) {
		auto const &points{Here.track->m_paths[Here.path].points};
		auto const along{Here.share * geometry::plan_distance(points[segment_data::point::start], points[segment_data::point::end])};
		return stretch{Here.track, Here.track->eType == tt_Switch ? 0 : Here.path, Here.track->eType == tt_Switch ? 0 : static_cast<int>(along / kStretch)};
	};
	joined_sets<stretch, std::map<stretch, stretch>> timbered;
	std::vector<char> on_surface(timber_all.size(), 0);
	for (auto const &here : timber_all)
		around(timber_places, here.position, kApart, [&](place const &Other) {
			if (Other.track <= here.track || (here.track->eType != tt_Switch && Other.track->eType != tt_Switch))
				return;
			glm::dvec2 const offset{Other.position.x - here.position.x, Other.position.z - here.position.z};
			if (glm::length(offset) >= kTimbers || std::abs(glm::dot(offset, here.along)) > 0.6 * kSpacing || std::abs(Other.position.y - here.position.y) > kLevel)
				return;
			++on_timbers;
			on_surface[here.index] = on_surface[Other.index] = 1;
			timbered.join(stretch_of(here), stretch_of(Other));
		});
	state.surface_places.clear();
	std::map<stretch, int> surfaces;
	struct surface_sum
	{
		double count{0.0};
		glm::dvec3 sum{0.0};
	};
	std::vector<surface_sum> sums;
	for (auto const &here : timber_all)
		if (on_surface[here.index] != 0)
		{
			auto const [found, added]{surfaces.try_emplace(timbered.root(stretch_of(here)), static_cast<int>(sums.size()))};
			if (added)
				sums.emplace_back();
			sums[found->second].count += 1.0;
			sums[found->second].sum += here.position;
		}
	double constexpr kSurfaceScale{100.0}; // m, of the tilts worked out, for numbers of a size with the others
	std::unordered_set<TTrack const *> timbered_tracks;
	for (auto const &here : timber_all)
		if (on_surface[here.index] != 0)
		{
			// a steep track holds its heights, but not against the timbers it shares; a hump gives way to them as little as they let it
			double constexpr kHumpKeep{0.05}; // m
			for (auto const node : {here.first, here.second})
				if (false == stayed_nodes.contains(node))
					terms[node].weight = 1.0 / (hump_nodes.contains(node) ? kHumpKeep * kHumpKeep : kKeep * kKeep);
			timbered_tracks.insert(here.track);
			auto const surface{surfaces.at(timbered.root(stretch_of(here)))};
			auto const middle{sums[surface].sum / sums[surface].count};
			auto const level{nodes + 3 * surface};
			terms.push_back({{here.first, here.second, level, level + 1, level + 2}, {1.0 - here.share, here.share, -1.0, -(here.position.x - middle.x) / kSurfaceScale, -(here.position.z - middle.z) / kSurfaceScale}, middle.y - here.position.y, 1.0 / (kOnTimbers * kOnTimbers)});
			state.surface_places.push_back({here.track, here.path, here.share, surface});
		}
	for (std::size_t surface = 0; surface < sums.size(); ++surface)
		for (int k = 0; k < 3; ++k)
			terms.push_back({{nodes + 3 * static_cast<int>(surface) + k, -1, -1, -1, -1}, {1.0, 0.0, 0.0, 0.0, 0.0}, 0.0, 1.0});
	auto const unknowns{nodes + 3 * static_cast<int>(sums.size())};
	// the end of the second path of a switch on the plane of the first
	for (auto const *track : tracks)
	{
		if (track->eType != tt_Switch || track->m_paths.size() < 2)
			continue;
		auto const &main{track->m_paths[0].points};
		auto const &branch{track->m_paths[1].points};
		auto const start{indices.at(key_of(main[segment_data::point::start]))};
		auto const finish{indices.at(key_of(main[segment_data::point::end]))};
		auto const tip{indices.at(key_of(branch[segment_data::point::end]))};
		auto const chord{geometry::plan_of(main[segment_data::point::end] - main[segment_data::point::start])};
		auto const square{glm::dot(chord, chord)};
		if (start == tip || finish == tip || start != indices.at(key_of(branch[segment_data::point::start])) || square < 1e-6)
			continue;
		auto const share{glm::dot(geometry::plan_of(branch[segment_data::point::end] - main[segment_data::point::start]), chord) / square};
		auto const off{heights[tip] - (heights[start] + share * (heights[finish] - heights[start]))};
		terms.push_back({{tip, start, finish, -1}, {1.0, -(1.0 - share), -share, 0.0}, -off, 1.0 / (kPlane * kPlane)});
	}
	// who lies on which, for the map of them: a number of each grade line shared, those on their own and the ones held apart
	state.grade_lines.clear();
	{
		std::unordered_map<TTrack const *, int> numbers;
		std::unordered_map<TTrack const *, std::size_t> sizes;
		for (auto const &here : all)
			if (false == apart.contains(runs.root(here.track)) && tied.joined(here.track))
				++sizes[tied.root(here.track)];
		// the lines put by hand keep their numbers, the others worked out get ones besides them
		std::set<int> kept;
		for (auto const &[track, line] : held)
			kept.insert(line);
		int next{0};
		for (auto const *track : tracks)
		{
			auto line{nmt_state::grade_line_own};
			if (off.contains(track))
				line = nmt_state::grade_line_off;
			else if (auto const found{held.find(track)}; found != held.end())
				line = found->second;
			else if (steep(*track))
				line = nmt_state::grade_line_steep;
			else if (false == apart.contains(runs.root(track)))
			{
				// by the run: a track of it with nothing beside itself lies on the line the rest of the run does
				auto const root{tied.root(runs.root(track))};
				if (auto const by_hand{held_of(root)}; by_hand >= 0)
					line = by_hand;
				else if (sizes.contains(root))
				{
					auto const [number, added]{numbers.try_emplace(root, 0)};
					if (added)
					{
						while (kept.contains(next))
							++next;
						number->second = next++;
					}
					line = number->second;
				}
			}
			state.grade_lines[grade_line_key(*track)] = line;
		}
		WriteLog(format("Editor: NMT - %zu grade lines shared, %zu of them put by hand, their tracks further apart tied at %zu places", numbers.size() + kept.size(), kept.size(), hand_pairs), logtype::generic);
		explain("lines");
		++state.grade_lines_changes;
		grade_lines_store();
	}
	// the least squares of all of it
	int iterations{0};
	auto const moves{grade_moves(terms, unknowns, iterations)};
	// the joints of the runs of the tracks named by a selftest: the height before and the move, held or not
	for (auto const *track : tracks)
		if (std::string::npos != state.explain.find(" " + track->name() + " "))
		{
			std::string joints;
			for (auto const *other : tracks)
				if (runs.root(other) == runs.root(track))
					for (auto const &path : other->m_paths)
					{
						auto const at{indices.at(key_of(path.points[segment_data::point::end]))};
						joints += format(" %s %.2f%+.2f%s", other->name().c_str(), heights[at], moves[at], fixed[at] != 0 ? " held" : "");
					}
			WriteLog(format("Editor: NMT share, moved: %s, the ends of the run:%s", track->name().c_str(), joints.c_str()), logtype::generic);
		}
	// what the end moves by, with what it's off the others at its joint, the switch there first
	auto const move_of = [&](glm::dvec3 const &Point) {
		auto const at{indices.at(key_of(Point))};
		auto const off{heights[at] - Point.y};
		return moves[at] + (std::abs(off) <= 1.0 ? off : 0.0);
	};
	editor_track::edit edit;
	std::vector<TTrack *> moved;
	auto largest{0.0};
	TTrack const *largest_at{nullptr};
	double constexpr kTilted{0.005}; // how far the mean grade at the ends of a path may be from its grade from end to end
	double constexpr kTiltedShortest{5.0}; // m, a shorter one tells little by the grades at its ends
	std::size_t straightened{0};
	for (auto *track : tracks)
	{
		if (state.stayed.contains(track))
			continue;
		auto changed{false};
		for (auto const &path : track->m_paths)
			for (auto const end : {segment_data::point::start, segment_data::point::end})
			{
				if (std::abs(move_of(path.points[end])) > largest)
				{
					largest = std::abs(move_of(path.points[end]));
					largest_at = track;
				}
				changed = changed || std::abs(move_of(path.points[end])) >= kMoved;
			}
		if (false == changed)
			continue;
		edit.capture(*track);
		moved.push_back(track);
		// a switch on common timbers lies straight between its ends, no vertical curve taking it off their surface; the other paths where
		// the grades at their ends, which average to the grade from end to end on a vertical curve, are far from it: the ends were brought
		// to the tracks adjoining with the grades left as they were, and the path waves between them
		auto const settle = [&](segment_data &Part) {
			auto &points{Part.points};
			if (points[segment_data::point::control1] == glm::dvec3{} && points[segment_data::point::control2] == glm::dvec3{})
				return;
			auto const span{geometry::plan_of(points[segment_data::point::end] - points[segment_data::point::start])};
			auto const rise{points[segment_data::point::end].y - points[segment_data::point::start].y};
			auto const tilted = [&] {
				auto const length{glm::length(span)};
				auto const before{glm::length(geometry::plan_of(points[segment_data::point::control1]))};
				auto const after{glm::length(geometry::plan_of(points[segment_data::point::control2]))};
				if (length < kTiltedShortest || before < 1e-3 || after < 1e-3)
					return false;
				return std::abs((points[segment_data::point::control1].y / before - points[segment_data::point::control2].y / after) * 0.5 - rise / length) > kTilted;
			};
			if (track->eType == tt_Switch ? timbered_tracks.contains(track) : tilted())
			{
				auto const grade{rise / std::max(1e-9, glm::dot(span, span))};
				points[segment_data::point::control1].y = grade * glm::dot(geometry::plan_of(points[segment_data::point::control1]), span);
				points[segment_data::point::control2].y = grade * glm::dot(geometry::plan_of(points[segment_data::point::control2]), span);
				straightened += track->eType == tt_Switch ? 0 : 1;
			}
		};
		for (auto &path : track->m_paths)
			move_path_heights(path, move_of(path.points[segment_data::point::start]), move_of(path.points[segment_data::point::end]), settle);
	}
	// a switch lies straight, but a change of grade that needs a curve isn't left bare at its joints: it goes on a vertical curve over the
	// whole of the switch, the ends of the paths taking the grades of the tracks going on from them and staying where they are
	double constexpr kBare{0.002}; // the change of grade which needs a vertical curve
	double constexpr kCrest{2000.0}; // m, the sharpest convex curve a switch may lie on
	std::vector<std::vector<leaving>> leaving_at(nodes);
	for (auto const *track : tracks)
		for (auto const &path : track->m_paths)
		{
			leaving_at[indices.at(key_of(path.points[segment_data::point::start]))].push_back(leaving_of(track, path, false));
			leaving_at[indices.at(key_of(path.points[segment_data::point::end]))].push_back(leaving_of(track, path, true));
		}
	// the grade the end of a path of a switch is to rise out of the joint with, two switches meeting sharing the difference, and how much
	// the grade breaks there now
	struct onward
	{
		double grade;
		double broken;
	};
	auto const going_on = [&](TTrack const *Track, segment_data const &Path, bool const Atend) -> std::optional<onward> {
		auto const own{leaving_of(Track, Path, Atend)};
		leaving const *best{nullptr};
		for (auto const &other : leaving_at[indices.at(key_of(Path.points[Atend ? segment_data::point::end : segment_data::point::start]))])
			if (other.track != Track && glm::dot(own.way, other.way) < -0.9 && (best == nullptr || glm::dot(own.way, other.way) < glm::dot(own.way, best->way)))
				best = &other;
		if (best == nullptr)
			return std::nullopt;
		return onward{best->track->eType == tt_Switch && false == state.stayed.contains(best->track) ? (own.grade - best->grade) * 0.5 : -best->grade, std::abs(own.grade + best->grade)};
	};
	std::size_t curved{0};
	for (auto *track : tracks)
	{
		if (track->eType != tt_Switch || state.stayed.contains(track) || track->m_paths.empty())
			continue;
		auto const before{going_on(track, track->m_paths.front(), false)};
		auto bare{before && before->broken > kBare};
		std::vector<std::optional<onward>> after;
		for (auto const &path : track->m_paths)
		{
			after.push_back(going_on(track, path, true));
			bare = bare || (after.back() && after.back()->broken > kBare);
		}
		if (false == bare)
			continue;
		edit.capture(*track);
		if (std::find(moved.begin(), moved.end(), track) == moved.end())
			moved.push_back(track);
		++curved;
		std::optional<double> blades; // the paths leave the blades with one grade, the first path tells it
		for (std::size_t i = 0; i < track->m_paths.size(); ++i)
		{
			auto &path{track->m_paths[i]};
			auto const length{glm::length(geometry::plan_of(path.points[segment_data::point::end] - path.points[segment_data::point::start]))};
			if (length < 1e-3)
				continue;
			auto const chord{(path.points[segment_data::point::end].y - path.points[segment_data::point::start].y) / length};
			// the grades along the path; an end with no track going on from it makes the curve even
			auto first{before ? before->grade : after[i] ? 2.0 * chord + after[i]->grade : chord};
			auto last{after[i] ? -after[i]->grade : before ? 2.0 * chord - before->grade : chord};
			auto const least_bare = [&](double const Grade) { return Grade + std::clamp(chord - Grade, -kBare, kBare); };
			if (blades)
			{
				// from the grade at the blades one bend reaches only so far by the far end
				first = *blades;
				auto const a{-2.0 * (first - chord)};
				auto const b{-0.5 * (first - chord)};
				auto const bent{chord + std::clamp(last - chord, std::min(a, b), std::max(a, b))};
				last = std::abs(bent - last) <= kBare ? bent : least_bare(last);
			}
			else
			{
				auto const bend_first{(6.0 * chord - 4.0 * first - 2.0 * last) / length};
				auto const bend_last{(-6.0 * chord + 2.0 * first + 4.0 * last) / length};
				// the ends can't be met by one bend: only what a curve is needed for is taken off them, the rest stays at the joints
				if (bend_first * bend_last < 0.0 || std::min(bend_first, bend_last) < -1.0 / kCrest)
				{
					first = least_bare(first);
					last = least_bare(last);
				}
				blades = first;
			}
			on_vertical_curve(path, first, last);
		}
	}
	WriteLog(format("Editor: NMT - one grade line for tracks under %.1f m apart: %zu runs of tracks on their own, %zu places side by side, %zu on common timbers in %zu groups, %d joints, %d iterations, %zu tracks moved, by %.2f m at most (%s), %zu paths straightened, %zu switches on a vertical curve", kApart, apart.size(), pairs, on_timbers, sums.size(), nodes, iterations, moved.size(), largest, largest_at != nullptr ? largest_at->name().c_str() : "-", straightened, curved), logtype::generic);
	if (moved.empty())
		return;
	profile_library_load();
	auto library{m_profiles};
	editor_track::commit(moved);
	push_track_snapshot(std::move(edit.states), {});
	fitted_set(moved);
	// the designs of the lines fitted one by one go with the tracks they moved
	auto const [refitted, failed]{profile_refit_stored({moved.begin(), moved.end()})};
	if (refitted > 0)
		profiles_to_history(std::move(library));
	WriteLog(format("Editor: NMT - %zu designs laid anew onto the tracks moved, %zu left as they were", refitted, failed), logtype::generic);
}

void editor_mode::nmt_auto_step()
{
	auto &state{m_nmt_state};
	if (state.stage == nmt_state::automatic::fetching)
	{
		bool all{true};
		for (auto const &place : state.probes)
			all = nmt_ready(glm::dvec2{place.position.x, place.position.z} - glm::dvec2{1.0}, glm::dvec2{place.position.x, place.position.z} + glm::dvec2{1.0}) && all;
		auto const waiting{m_nmt.waiting()};
		state.status = format(STR_P("Fetching the terrain model under the tracks: %zu square left", "Fetching the terrain model under the tracks: %zu squares left", waiting), waiting);
		if (false == all)
			return;
		// the scenery stays at its own heights, the terrain model meets it: offset by how far the tracks lie from it, so the lines
		// fitted one after another meet at heights near their own; the grade lines then follow the model so offset
		// the places of the tracks fitted already, the last ones, apart
		std::vector<double> differences, kept;
		auto const first_kept{state.probes.size() - std::min(state.probes_kept, state.probes.size())};
		for (std::size_t i = 0; i < state.probes.size(); ++i)
			if (double height; m_nmt.height(state.probes[i].position.x, state.probes[i].position.z, height))
				(i < first_kept ? differences : kept).push_back(height - state.probes[i].crown);
		if (differences.empty())
		{
			state.stage = nmt_state::automatic::idle;
			state.status = STR("The terrain model has no data under the tracks: is the scenery placed where it is?");
			return;
		}
		std::nth_element(differences.begin(), differences.begin() + differences.size() / 2, differences.end());
		auto const lift{differences[differences.size() / 2]};
		if (state.unfitted && false == kept.empty())
		{
			// the tracks fitted already stay where they are, and the model keeps the offset they were fitted to, told by them
			std::nth_element(kept.begin(), kept.begin() + kept.size() / 2, kept.end());
			auto const offset{-kept[kept.size() / 2]};
			if (std::abs(offset - state.offset) > 0.05)
				state.offset = offset;
			WriteLog(format("Editor: NMT - the tracks fitted already lie on the terrain model offset %+.2f m, kept; the ones not fitted yet lie %+.2f m against it", state.offset, -lift - state.offset), logtype::generic);
		}
		else
		{
			// the offset of the run before kept where the tracks fitted to it lie within 5 cm of it: they don't creep a centimetre a run
			auto const offset{std::abs(lift) > 0.05 ? -lift : 0.0};
			if (std::abs(offset - state.offset) > 0.05)
				state.offset = offset;
			WriteLog(format("Editor: NMT - the terrain model offset %+.2f m to the scenery", state.offset), logtype::generic);
		}
		state.fitted = true;
		nmt_join_ends();
		state.levelled = nmt_level_turntables();
		// the longest tracks first: the main lines lead, the sidings join them
		state.left.clear();
		for (auto *track : simulation::Paths.sequence())
			if (track != nullptr && rail(*track) && track->eType != tt_Table && false == nmt_stays(*track))
				state.left.push_back(track);
		std::sort(state.left.begin(), state.left.end(), [](TTrack *Left, TTrack *Right) { return Left->Length() > Right->Length(); });
		state.tracks = state.left.size();
		state.split = 0;
		state.done.clear();
		// the tracks fitted before are lines fitted already: the new ones run up to them and meet them as they are
		state.stayed.clear();
		for (auto *track : simulation::Paths.sequence())
			if (track != nullptr && rail(*track) && nmt_stays(*track))
			{
				state.done.push_back(track);
				state.stayed.insert(track);
			}
		state.laid.clear();
		state.bridged.clear();
		state.on_hump.clear();
		// every path sampled as laid, for the crossings of the lines
		state.paths.clear();
		for (auto *track : simulation::Paths.sequence())
			if (track != nullptr && rail(*track))
				nmt_sample_paths(*track, state.paths);
		state.stage = nmt_state::automatic::fitting;
		return;
	}
	// a few lines a frame: the longest track next to the lines fitted already, so that a line meets the ones it leads into as
	// it's fitted, not when the lines between them come; the longest of all where none is next to them
	for (int line = 0; line < 2 && false == state.left.empty(); ++line)
	{
		std::unordered_set<TTrack const *> const fitted(state.done.begin(), state.done.end());
		auto const touches = [&](TTrack *Track) {
			for (int path = 0; path < static_cast<int>(Track->m_paths.size()); ++path)
				for (auto const forward : {true, false})
				{
					auto const onward{editor_track::continuations({Track, path, forward})};
					if (std::any_of(onward.begin(), onward.end(), [&](editor_track::continuation const &Next) { return fitted.contains(Next.span.track); }))
						return true;
					if (auto const loose{state.loose.find({Track, path, forward})}; loose != state.loose.end() && fitted.contains(std::get<0>(loose->second)))
						return true;
				}
			return false;
		};
		auto const next{std::find_if(state.left.begin(), state.left.end(), touches)};
		if (next != state.left.end())
			std::rotate(state.left.begin(), next, next + 1);
		auto *track{state.left.front()};
		auto const name{track->name()};
		auto const known{simulation::Paths.sequence().size()};
		if (nmt_fit_line(*track, [&](TTrack const *Other) { return fitted.contains(Other); }, false))
			++state.lines;
		else
			++state.refused;
		// the tracks of the line are done, whether it went or not; so is every path made meanwhile, split off by the grade line or
		// by the lines following it over the crossovers: the next line over them would lay them again, and split them again
		std::set<TTrack const *> done{track};
		state.done.push_back(track);
		for (auto const &span : m_profile.route.spans)
			if (done.insert(span.track).second)
				state.done.push_back(span.track);
		auto const &sequence{simulation::Paths.sequence()};
		std::string made;
		for (auto i{known}; i < sequence.size(); ++i)
			if (auto *piece{sequence[i]}; piece != nullptr && done.insert(piece).second)
			{
				state.done.push_back(piece);
				if (made.size() < 300)
					made += ' ' + piece->name();
			}
		auto const added{sequence.size() - known};
		state.split += added;
		if (added > 20)
			WriteLog(format("Editor: NMT line %s made %zu paths:%s", name.c_str(), added, made.c_str()), logtype::generic);
		if (state.split > state.tracks / 2 + 200)
		{
			// something lays the same tracks over and over: stopped before it takes all the memory
			state.left.clear();
			state.error = format(STR_P("Stopped: the grade lines split off %zu path from %zu tracks, more than they should; Ctrl+Z takes the changes back", "Stopped: the grade lines split off %zu paths from %zu tracks, more than they should; Ctrl+Z takes the changes back", state.split), state.split, state.tracks);
			WriteLog("Editor: NMT - " + state.error, logtype::generic);
			break;
		}
		state.left.erase(std::remove_if(state.left.begin(), state.left.end(), [&](TTrack *Track) { return done.contains(Track); }), state.left.end());
	}
	state.status = format(STR_C("Terrain model offset %+.2f m to the scenery; grade lines fitted to the terrain: %zu, refused by the checks: %zu, tracks left: %zu"), state.offset, state.lines, state.refused, state.left.size());
	if (state.departures > 0)
		state.status += format(STR_P("\n%zu line parts from the terrain model by over 0.5 m: the model or the track is off there, the terrain follows the tracks (the log names it)", "\n%zu lines part from the terrain model by over 0.5 m: the model or the track is off there, the terrain follows the tracks (the log names them)", state.departures), state.departures);
	if (state.levelled > 0)
		state.status += format(STR_P("\n%zu turntable set level at the height of the tracks running up to it, which reach it level", "\n%zu turntables set level at the height of the tracks running up to them, which reach them level", state.levelled), state.levelled);
	if (false == state.left.empty())
		return;
	state.result = true;
	// the last line fitted isn't left in the profile, its tracks may be gone: the profile opens on what's selected
	m_profile.open = false;
	m_profile.route = {};
	m_profile.samples.clear();
	m_profile.followed.clear();
	nmt_share_grades();
	state.done.clear();
	state.laid.clear();
	state.paths.clear();
	state.loose.clear();
	state.turntables.clear();
	state.stage = nmt_state::automatic::idle;
	WriteLog("Editor: NMT - " + state.status, logtype::generic);
	nmt_make(nmt_keys_along_tracks(), true);
}
