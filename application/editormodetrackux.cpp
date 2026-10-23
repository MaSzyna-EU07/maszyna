/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/editormode.h"
#include "application/editoruilayer.h"
#include "application/editorprojection.h"
#include "application/editormodeshared.h"
#include "application/editormodetrackshared.h"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorSettings.hpp"
#include "editor/editorSwitchStore.hpp"

#include "utilities/Globals.h"
#include "rendering/renderer.h"
#include "world/Track.h"
#include "utilities/utilities.h"
#include "scene/scene.h"
#include "scene/scenelayers.h"
#include "simulation/simulation.h"
#include "utilities/translation.h"

#include "imgui/imgui.h"
#include "imgui/ImGuizmo.h"
#include <algorithm>
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <array>
#include <functional>
#include <cctype>
#include <chrono>
#include <cmath>
#include <limits>

namespace
{

using geometry::bezier;
using geometry::plan_of;

float const kHoverPixels{12.0f};
float const kFarHoverPixels{16.0f};
double const kLinesFrom{0.5};
double const kHoverLabelDelay{0.35}; // s the cursor rests on a path before its data shows up
double const kFocusDistance{25.0};
auto const kContextClick{std::chrono::milliseconds(350)};
float const kContextTurn{0.02f}; // rad the camera may turn during a click of the right button
std::size_t const kSearchLimit{200};
int const kRailCategory{1};

bool viewport_click()
{
	return false == ImGui::GetIO().MouseDownOwned[0];
}

bool rail(TTrack const &Track)
{
	return (Track.iCategoryFlag & 15) == kRailCategory && false == Track.m_editorremoved && Track.visible();
}

char const *type_of(TTrack const &Track)
{
	switch (Track.eType)
	{
	case tt_Normal: return "normal";
	case tt_Switch: return "switch";
	case tt_Cross: return "cross";
	case tt_Table: return "turntable";
	case tt_Tributary: return "tributary";
	default: return "unknown type";
	}
}

double ortho_metres_per_pixel()
{
	auto const height{ImGui::GetIO().DisplaySize.y};
	return Global.EditorOrtho && height > 0.0f ? 2.0 * static_cast<double>(Global.EditorOrthoExtent) / static_cast<double>(height) : 0.0;
}

// the bands of the colouring of the tracks: up to the value, the colour and its label in the legend
struct colour_band
{
	double below;
	ImU32 colour;
	char const *label;
};
double constexpr kBeyond{std::numeric_limits<double>::max()};
ImU32 const kUncoloured{IM_COL32(200, 200, 200, 170)};
double const kColouredReach{1500.0}; // m around the camera in the 3D view
double const kLabelsTo{1.0}; // m a pixel of the top view, farther out the labels of the tracks are left out
double const kLabelledLength{5.0}; // m, shorter curves get no label
std::vector<colour_band> const kRadiusBands{{300.0, IM_COL32(235, 60, 60, 235), "R < 300 m"}, {600.0, IM_COL32(245, 140, 50, 235), "R 300-600 m"}, {1200.0, IM_COL32(240, 220, 60, 235), "R 600-1200 m"},
                                            {3000.0, IM_COL32(110, 210, 80, 235), "R 1200-3000 m"}, {kBeyond, IM_COL32(70, 190, 235, 235), "R > 3000 m"}};
std::vector<colour_band> const kSpeedBands{{40.5, IM_COL32(235, 60, 60, 235), "<= 40 km/h"}, {60.5, IM_COL32(245, 140, 50, 235), "50-60 km/h"}, {80.5, IM_COL32(240, 220, 60, 235), "70-80 km/h"},
                                           {100.5, IM_COL32(110, 210, 80, 235), "90-100 km/h"}, {120.5, IM_COL32(70, 190, 235, 235), "110-120 km/h"}, {kBeyond, IM_COL32(170, 120, 255, 235), "> 120 km/h"}};
std::vector<colour_band> const kGradeBands{{2.0, kUncoloured, "0-2 \xe2\x80\xb0"}, {5.0, IM_COL32(110, 210, 80, 235), "2-5 \xe2\x80\xb0"}, {10.0, IM_COL32(240, 220, 60, 235), "5-10 \xe2\x80\xb0"},
                                           {20.0, IM_COL32(245, 140, 50, 235), "10-20 \xe2\x80\xb0"}, {kBeyond, IM_COL32(235, 60, 60, 235), "> 20 \xe2\x80\xb0"}};

ImU32 band_colour(std::vector<colour_band> const &Bands, double const Value)
{
	for (auto const &band : Bands)
		if (Value < band.below)
			return band.colour;
	return kUncoloured;
}

bool plan_near(bezier const &Curve, glm::dvec2 const &Point, double const Reach)
{
	auto const centre{(plan_of(Curve.p0) + plan_of(Curve.p1) + plan_of(Curve.p2) + plan_of(Curve.p3)) * 0.25};
	auto const radius{std::max({glm::distance(centre, plan_of(Curve.p0)), glm::distance(centre, plan_of(Curve.p1)), glm::distance(centre, plan_of(Curve.p2)), glm::distance(centre, plan_of(Curve.p3))})};
	return glm::distance(centre, Point) <= radius + Reach;
}

double segment_distance2(glm::vec2 const &Point, glm::vec2 const &A, glm::vec2 const &B, double &T)
{
	auto const ab{B - A};
	auto const length2{glm::dot(ab, ab)};
	T = length2 > 1e-6f ? std::clamp(static_cast<double>(glm::dot(Point - A, ab) / length2), 0.0, 1.0) : 0.0;
	auto const closest{A + ab * static_cast<float>(T)};
	return glm::dot(Point - closest, Point - closest);
}

} // namespace

editor_mode::track_hover editor_mode::track_under_cursor(TTrack const *Only, float const Pixels) const
{
	track_hover result;
	auto const &io{ImGui::GetIO()};
	screen_projection const projection;
	glm::vec2 const mouse{io.MousePos.x, io.MousePos.y};
	auto const metres{ortho_metres_per_pixel()};
	auto const hover{metres >= kLinesFrom ? kFarHoverPixels : kHoverPixels};
	double best{Pixels > 0.0f ? Pixels * Pixels : Only != nullptr ? std::numeric_limits<double>::max() : hover * hover};
	glm::dvec3 origin;
	glm::dvec3 direction;
	projection.ray(io.MousePos, origin, direction);
	bool const cull{metres > 0.0 && Only == nullptr};
	auto const reach{cull ? std::sqrt(best) * metres : 0.0};
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || false == rail(*track) || track->m_editorremoved || (Only != nullptr && track != Only))
			continue;
		for (int i = 0; i < static_cast<int>(track->m_paths.size()); ++i)
		{
			bezier const curve{track->m_paths[i]};
			if (cull && false == plan_near(curve, plan_of(origin), reach))
				continue;
			auto const count{std::clamp(static_cast<int>(curve.plan_length() / 3.0), 6, 64)};
			auto previous{curve.point(0.0)};
			ImVec2 from;
			bool visible{projection.project(previous, from)};
			for (int k = 1; k <= count; ++k)
			{
				auto const next{curve.point(static_cast<double>(k) / count)};
				ImVec2 to;
				bool const shown{projection.project(next, to)};
				if (visible && shown)
				{
					double t;
					auto const distance{segment_distance2(mouse, {from.x, from.y}, {to.x, to.y}, t)};
					if (distance < best)
					{
						best = distance;
						result.track = track;
						result.path = i;
						result.point = glm::mix(previous, next, t);
					}
				}
				previous = next;
				from = to;
				visible = shown;
			}
		}
	}
	return result;
}

void editor_mode::update_track_hover()
{
	if (typed_meaning() == nullptr)
		m_typed.clear();
	bool const blocked{ImGui::GetIO().WantCaptureMouse || mouseHold || m_track_set_using || m_track_box.active || m_input.mouse.button(GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS || m_track_gizmo_using || m_route_dragging || m_extend.active || m_switch.placing ||
	                   m_straights.dragging || m_lay.active};
	if (blocked)
	{
		m_hover = {};
		return;
	}
	auto found{track_under_cursor()};
	found.since = found.track == m_hover.track ? m_hover.since : ImGui::GetTime();
	m_hover = found;
}

void editor_mode::draw_track_lines() const
{
	auto const metres{ortho_metres_per_pixel()};
	auto const colouring{ui()->mode() == nodebank_panel::TRACK ? m_track_colouring : track_colouring::plain};
	// the top view of the rails alone draws them in a colour of their own, as on a plan: both rails where they're apart on the screen
	auto const rails{metres > 0.0 && Global.EditorOrthoTracks > 1 && colouring == track_colouring::plain};
	if (metres < kLinesFrom && colouring == track_colouring::plain && false == rails)
		return;
	auto const &io{ImGui::GetIO()};
	screen_projection const projection;
	glm::dvec2 const centre{Global.pCamera.Pos.x, Global.pCamera.Pos.z};
	auto const reach{Global.EditorOrtho ? 0.5 * metres * std::hypot(static_cast<double>(io.DisplaySize.x), static_cast<double>(io.DisplaySize.y)) : kColouredReach};
	auto const width{rails ? std::max(2.0f, static_cast<float>(0.07 / metres)) : colouring == track_colouring::plain ? 1.5f : 3.0f};
	auto const railcolour{EditorSettings.top_rails()};
	std::vector<glm::dvec3> points;
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || false == rail(*track))
			continue;
		auto const velocity{editor_track::velocity(*track)};
		for (auto const &path : track->m_paths)
		{
			bezier const curve{path};
			if (false == plan_near(curve, centre, reach))
				continue;
			auto colour{rails ? railcolour : IM_COL32(235, 235, 235, 150)};
			if (colouring == track_colouring::radius)
			{
				auto const radius{::track_mode::piece_plan_radius(path)};
				colour = radius >= 0.5 ? band_colour(kRadiusBands, radius) : kUncoloured;
			}
			else if (colouring == track_colouring::speed)
				colour = velocity > 0.0 ? band_colour(kSpeedBands, velocity) : kUncoloured;
			auto const length{curve.plan_length(4)};
			auto const count{std::clamp(static_cast<int>(length / std::max(metres, 0.5) / 8.0), colouring == track_colouring::grade ? std::clamp(static_cast<int>(length / 10.0), 1, 8) : 1, rails ? 128 : 32)};
			if (rails && track->fTrackWidth / metres > 6.0)
			{
				points.clear();
				for (int k = 0; k <= count; ++k)
					points.push_back(curve.point(static_cast<double>(k) / count));
				auto const half{(track->fTrackWidth + 0.07) * 0.5};
				auto const across = [&](std::size_t const K) {
					auto const along{points[std::min(K + 1, points.size() - 1)] - points[K > 0 ? K - 1 : 0]};
					auto const normal{glm::dvec3{-along.z, 0.0, along.x}};
					auto const size{glm::length(normal)};
					return size > 1e-9 ? normal * (half / size) : glm::dvec3{0.0};
				};
				for (auto const side : {-1.0, 1.0})
					for (std::size_t k = 1; k < points.size(); ++k)
						projection.line(drawlist, points[k - 1] + across(k - 1) * side, points[k] + across(k) * side, colour, width);
				continue;
			}
			auto previous{curve.point(0.0)};
			for (int k = 1; k <= count; ++k)
			{
				auto const next{curve.point(static_cast<double>(k) / count)};
				if (colouring == track_colouring::grade)
				{
					auto const run{geometry::plan_distance(previous, next)};
					colour = run > 1e-3 ? band_colour(kGradeBands, std::abs(next.y - previous.y) / run * 1000.0) : kUncoloured;
				}
				projection.line(drawlist, previous, next, colour, width);
				previous = next;
			}
		}
	}
}

void editor_mode::draw_track_labels() const
{
	auto const metres{ortho_metres_per_pixel()};
	if (false == m_track_labels || metres <= 0.0 || metres > kLabelsTo)
		return;
	auto const &io{ImGui::GetIO()};
	screen_projection const projection;
	glm::dvec2 const centre{Global.pCamera.Pos.x, Global.pCamera.Pos.z};
	auto const reach{0.5 * metres * std::hypot(static_cast<double>(io.DisplaySize.x), static_cast<double>(io.DisplaySize.y))};
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
	auto const low{editor_ui::view_min()};
	auto const high{editor_ui::view_max()};
	// a label which would cover another one is left out: the switches go first, then the sharper curves
	std::vector<std::pair<ImVec2, ImVec2>> taken;
	auto const write = [&](glm::dvec3 const &Point, std::string const &Text, ImU32 const Colour) {
		ImVec2 at;
		if (false == projection.project(Point, at))
			return;
		auto const size{ImGui::CalcTextSize(Text.c_str())};
		ImVec2 const from{at.x - size.x * 0.5f - 4.0f, at.y - size.y - 12.0f};
		ImVec2 const to{at.x + size.x * 0.5f + 4.0f, at.y - 8.0f};
		if (from.x < low.x || from.y < low.y || to.x > high.x || to.y > high.y)
			return;
		if (std::any_of(taken.begin(), taken.end(), [&](auto const &Rect) { return from.x < Rect.second.x + 6.0f && to.x > Rect.first.x - 6.0f && from.y < Rect.second.y + 3.0f && to.y > Rect.first.y - 3.0f; }))
			return;
		taken.emplace_back(from, to);
		drawlist->AddRectFilled(from, to, IM_COL32(0, 0, 0, 165), 3.0f);
		drawlist->AddText(ImVec2(from.x + 4.0f, from.y), Colour, Text.c_str());
	};
	struct curve_label
	{
		double radius;
		glm::dvec3 point;
		std::string text;
	};
	std::vector<curve_label> curves;
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || false == rail(*track) || track->m_paths.empty())
			continue;
		bezier const first{track->m_paths.front()};
		if (false == plan_near(first, centre, reach))
			continue;
		if (track->eType == tt_Switch)
		{
			auto text{name_of(*track)};
			if (auto const record{switch_store::of(*track)})
				text += "  " + switch_store::label(*record);
			write(first.point(0.0), text, IM_COL32(255, 220, 120, 255));
			continue;
		}
		auto const radius{::track_mode::piece_plan_radius(track->m_paths.front())};
		if (radius >= 0.5 && first.plan_length() >= kLabelledLength)
			curves.push_back({radius, first.point(0.5), format("R %.0f  L %.1f", radius, first.plan_length())});
	}
	std::sort(curves.begin(), curves.end(), [](curve_label const &A, curve_label const &B) { return A.radius < B.radius; });
	for (auto const &curve : curves)
		write(curve.point, curve.text, IM_COL32(235, 235, 235, 240));
}

void editor_mode::draw_track_legend() const
{
	if (m_track_colouring == track_colouring::plain)
		return;
	auto const &bands{m_track_colouring == track_colouring::radius ? kRadiusBands : m_track_colouring == track_colouring::speed ? kSpeedBands : kGradeBands};
	std::vector<std::pair<ImU32, std::string>> rows;
	for (auto const &band : bands)
		rows.emplace_back(band.colour, band.label);
	if (m_track_colouring == track_colouring::radius)
		rows.emplace_back(kUncoloured, STR("straight"));
	else if (m_track_colouring == track_colouring::speed)
		rows.emplace_back(kUncoloured, STR("no limit"));
	char const *const titles[]{"", "By the radius", "By the speed", "By the grade"};
	std::string const title{STR_C(titles[static_cast<int>(m_track_colouring)])};
	auto const line{ImGui::GetTextLineHeightWithSpacing()};
	auto const swatch{ImGui::GetTextLineHeight()};
	float width{ImGui::CalcTextSize(title.c_str()).x};
	for (auto const &row : rows)
		width = std::max(width, swatch + 6.0f + ImGui::CalcTextSize(row.second.c_str()).x);
	auto const pad{8.0f};
	// in the bottom right corner, over the profile strip when it's open
	auto const bottom{m_profile.strip_top > editor_ui::view_min().y + 4.0f * line ? std::min(editor_ui::view_max().y, m_profile.strip_top) : editor_ui::view_max().y};
	ImVec2 const to{editor_ui::view_max().x - 12.0f, bottom - 12.0f};
	ImVec2 const from{to.x - width - pad * 2.0f, to.y - pad * 2.0f - line * static_cast<float>(rows.size() + 1)};
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
	drawlist->AddRectFilled(from, to, IM_COL32(0, 0, 0, 170), 4.0f);
	ImVec2 at{from.x + pad, from.y + pad};
	drawlist->AddText(at, IM_COL32(255, 255, 255, 235), title.c_str());
	for (auto const &row : rows)
	{
		at.y += line;
		drawlist->AddRectFilled(ImVec2(at.x, at.y + 2.0f), ImVec2(at.x + swatch, at.y + swatch - 2.0f), row.first, 2.0f);
		drawlist->AddText(ImVec2(at.x + swatch + 6.0f, at.y), IM_COL32(235, 235, 235, 235), row.second.c_str());
	}
}

void editor_mode::draw_track_hover() const
{
	using kind = track_intent::kind;
	auto const *track{m_hover.track};
	if (track == nullptr || (m_intent.what != kind::select && m_intent.what != kind::box && m_intent.what != kind::straight_set && m_intent.what != kind::pick_style) || m_intent.track != track)
		return;
	screen_projection const projection;
	if (track != selected_track())
	{
		auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
		for (int i = 0; i < static_cast<int>(track->m_paths.size()); ++i)
		{
			bezier const curve{track->m_paths[i]};
			auto const count{std::clamp(static_cast<int>(curve.plan_length() / 3.0), 6, 64)};
			auto const colour{i == m_hover.path ? IM_COL32(255, 255, 255, 200) : IM_COL32(255, 255, 255, 110)};
			auto previous{curve.point(0.0)};
			for (int k = 1; k <= count; ++k)
			{
				auto const next{curve.point(static_cast<double>(k) / count)};
				projection.line(drawlist, previous, next, colour, i == m_hover.path ? 3.5f : 2.0f);
				previous = next;
			}
		}
	}
}

bool editor_mode::track_gesture_on() const
{
	return mouseHold || m_track_set_using || m_track_box.active || m_input.mouse.button(GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS || m_track_gizmo_using || m_route_dragging || m_extend.active || m_switch.placing || m_straights.dragging || m_straights.tool_mouse ||
	       m_point_drag.active || m_handle_drag.active || m_sweep.dragging || m_turntable.placing || m_signal.placing || m_signal.moving || m_switch.slide.active || ImGuizmo::IsUsing();
}

editor_mode::track_intent editor_mode::track_intent_at(int const Mods)
{
	using kind = track_intent::kind;
	track_intent intent;
	if (ImGui::GetIO().WantCaptureMouse)
		return intent;
	glm::dvec3 const ground{cursor_ground()};
	intent.position = ground;
	bool const shift{(Mods & GLFW_MOD_SHIFT) != 0};
	bool const control{(Mods & GLFW_MOD_CONTROL) != 0};
	bool const alt{(Mods & GLFW_MOD_ALT) != 0};
	auto const make = [&](kind const What, char const *Action) {
		intent.what = What;
		intent.action = Action != nullptr ? STR(Action) : std::string{};
		return intent;
	};
	auto *selected{selected_track()};

	if (m_tape.active)
	{
		intent.position = m_tape.mouse;
		return make(kind::measure, m_tape.points.size() == 1 ? "Click: the end of the measure" : "Click: the start of the measure. M or Esc: done");
	}
	if (m_lay.active)
	{
		auto const &lay{m_lay};
		if (lay.mouse_snap.track != nullptr && (lay.points.empty() || glm::distance(lay.mouse_snap.position, lay.points.front()) > 1.0))
		{
			intent.snap = lay.mouse_snap;
			intent.track = lay.mouse_snap.track;
			intent.position = lay.mouse_snap.position;
			auto const diverging{lay.mouse_snap.track->eType == tt_Switch && lay.mouse_snap.point == editor_track::point_ref{1, editor_track::point_kind::end}};
			if (lay.points.empty() && diverging)
				return make(kind::lay_end, "Click: start from this free end, drag: the curve of the switch goes on");
			return make(kind::lay_end, lay.points.empty() ? "Click: start from this free end, drag: lead a track out of it" : "Click: join this end and lay");
		}
		intent.position = lay.mouse;
		if (lay.points.empty())
			if (auto const under{track_under_cursor()}; under.track != nullptr)
			{
				intent.track = under.track;
				intent.path = under.path;
				if (shift)
					return make(kind::select, "Click: the Parallel tool (N) with this track");
				return make(kind::lay, "Click: start on the ground beside this track. Shift+click: a parallel track to it, T: a switch into it");
			}
		return make(kind::lay, lay.points.empty() ? "Click: start of the new track" : "Click: lay the track to here");
	}
	if (m_track_tab == track_tab::turntable)
		return turntable_intent(ground, shift);
	if (m_track_tab == track_tab::srk && m_srk.drawing())
	{
		bool valid{false};
		intent.track = m_hover.track;
		intent.path = m_hover.path;
		intent.position = m_hover.track != nullptr ? m_hover.point : ground;
		intent.action = m_srk.boundary_hover(intent.position, valid);
		intent.what = kind::srk_boundary;
		return intent;
	}
	if (m_track_tab == track_tab::srk)
	{
		if (auto const hit{signal_hit()}; hit >= 0)
		{
			auto const &standing{m_signal.standing[hit]};
			intent.index = hit;
			intent.position = standing.include.location;
			intent.action = format(STR_C("Click: select the signal %s, drag: move it along the track"), standing.name.c_str());
			intent.what = kind::signal_move;
			return intent;
		}
		if (auto const *armed{signal_armed()}; armed != nullptr)
		{
			signal_spot spot;
			if (m_hover.track != nullptr && signal_spot_at(*m_hover.track, m_hover.path, m_hover.point, cursor_level(m_hover.point.y), 0, 0, *armed, spot))
			{
				intent.track = m_hover.track;
				intent.path = m_hover.path;
				intent.position = spot.position;
				intent.action = signal_intent_label(*armed, spot);
				intent.what = kind::signal_place;
			}
			return intent;
		}
		if (m_hover.track != nullptr)
		{
			intent.track = m_hover.track;
			intent.path = m_hover.path;
			intent.position = m_hover.point;
			return make(kind::select, m_hover.track->eType != tt_Normal ? "Click: the station of this switch" : "Click: the station of the switch nearest along this track");
		}
		return intent;
	}
	if (m_track_tab == track_tab::kilometrage && m_hover.track != nullptr)
	{
		intent.track = m_hover.track;
		intent.path = m_hover.path;
		intent.position = m_hover.point;
		make(kind::kilometrage, "Click: a point of the km here");
		if (auto const km{m_km.readout(m_hover.point)}; false == km.empty())
			intent.action = km + "\n" + intent.action;
		return intent;
	}
	if (m_track_style.picking && (m_track_tab == track_tab::path || m_track_tab == track_tab::turnout))
	{
		intent.track = m_hover.track;
		if (m_hover.track != nullptr)
			intent.position = m_hover.point;
		return make(kind::pick_style, m_hover.track != nullptr ? "Click: take the properties of this track" : "Click a track to take its properties, Esc: cancel");
	}
	if (m_track_tab == track_tab::path)
	{
		if (shift && control && m_hover.track != nullptr && rail(*m_hover.track))
		{
			intent.track = m_hover.track;
			intent.position = m_hover.point;
			auto const &preview{track_spread_preview(*m_hover.track)};
			intent.action = format(in_track_set(m_hover.track) ? STR_C("Click: take out the %zu joined paths, %.0f m") : STR_C("Click: add the %zu joined paths, %.0f m"), preview.size(), m_spread.length);
			intent.what = kind::spread;
			return intent;
		}
		if (shift && false == gizmo_frame::over())
		{
			intent.track = m_hover.track;
			return make(kind::box, m_hover.track == nullptr ? "Drag: the tracks in a box join the selection" : track_selected(m_hover.track) ? "Click: take out of the selection, drag: a box" : "Click: add to the selection, drag: a box");
		}
		if (auto const hit{track_handle_hit()}; hit.valid() && selected != nullptr)
		{
			std::string reason;
			if (editor_track::can_edit_geometry(*selected, reason))
			{
				intent.track = selected;
				intent.point = hit;
				intent.position = editor_track::point_position(*selected, hit);
				if (false == editor_track::is_end(hit.kind))
					return make(kind::point, "Drag: the control vector, the curve follows");
				if (editor_track::is_connected(*selected, hit))
					return make(kind::point, "Drag: the joint, the neighbours follow");
				if (control)
					return make(kind::point, "Drag: the end, dropped on a free end it joins it");
				intent.snap.track = selected;
				intent.snap.point = hit;
				intent.snap.position = intent.position;
				auto const diverging{selected->eType == tt_Switch && hit == editor_track::point_ref{1, editor_track::point_kind::end}};
				return make(kind::extend_end, diverging ? "Drag: the curve of the switch goes on; Ctrl+drag: move the end" : "Drag: a new track out of this end, as in laying; Ctrl+drag: move the end");
			}
		}
		if (control && false == alt && m_hover.track != nullptr && false == gizmo_frame::over())
		{
			intent.track = m_hover.track;
			return make(kind::box, track_selected(m_hover.track) ? "Click: take out of the selection" : "Click: add to the selection");
		}
	}
	if (m_track_tab == track_tab::profile && shift && false == gizmo_frame::over())
	{
		// a box from the empty ground is the route by itself, a click on a track adds it to the route or takes it out
		intent.track = m_hover.track;
		intent.index = m_hover.track == nullptr ? 1 : 0;
		return make(kind::box, m_hover.track == nullptr ? "Drag: the route over the tracks in a box" : track_selected(m_hover.track) ? "Click: take it out of the route, drag: a box" : "Click: add it to the route, drag: a box");
	}
	if (m_track_tab == track_tab::straights)
	{
		auto const &line{current_straight()};
		if (alt)
		{
			intent.track = m_hover.track;
			return m_hover.track != nullptr ? make(kind::straight_set, "Click: move this straight together with the others") : intent;
		}
		if (control && false == line.tracks.empty())
			return make(kind::straight_break, "Press: break the straight here, drag to turn the rest");
		if (shift && false == line.tracks.empty())
			return make(kind::detour, "Click: next point of the shift around an obstacle");
	}
	if (m_track_tab == track_tab::straights && m_straights.tool == 0)
	{
		if (auto const hit{straight_handle_hit()}; hit >= 0)
		{
			auto const &line{current_straight()};
			glm::dvec3 const handles[] = {line.start, line.end, (line.start + line.end) * 0.5};
			intent.index = hit;
			intent.position = handles[hit];
			if (hit == 2)
			{
				auto const offer{straight_parallel_offer(line, {})};
				if (offer.nearby)
				{
					intent.action = format(offer.snaps ? STR_C("Drag: shift the straight sideways — snaps parallel at %.2f m") : STR_C("Drag: shift the straight sideways — close: %.2f m  %.1f°"), offer.spacing, offer.angle);
					intent.what = kind::straight_handle;
					return intent;
				}
			}
			return make(kind::straight_handle, hit == 2 ? "Drag: shift the straight sideways" : "Drag: move this end, the curves at it follow");
		}
	}
	if (m_route.trace.active && route_active())
	{
		intent.index = route_trace_hit();
		if (control && intent.index >= 0)
		{
			intent.position = m_route.trace.points[intent.index];
			return make(kind::route_trace, "Click: take this point out");
		}
		intent.index = -1;
		intent.position = route_trace_cursor();
		return make(kind::route_trace, m_route.trace.points.empty() ? "Click: a point in the middle of the track on the photo. Enter: apply, Esc: drop" : "Click: one more point, the curve follows. Ctrl+click: take a point out");
	}
	if (int vertex, grip; route_hit(vertex, grip))
	{
		intent.index = vertex >= 0 ? vertex : grip;
		intent.position = vertex >= 0 ? route_vertex_position(vertex) : alignment::evaluate(m_route.result, m_route.result.vertex_chainages[grip]).position;
		return make(vertex >= 0 ? kind::route_vertex : kind::route_grip, vertex >= 0 ? "Drag: move this vertex, the curve follows" : "Drag: the radius of this curve");
	}
	if (gizmo_frame::over())
		return make(kind::gizmo, nullptr);
	if (m_track_tab == track_tab::straights)
	{
		auto const &line{current_straight()};
		if (m_straights.tool != 0 && false == line.tracks.empty())
		{
			intent.position = line.start + glm::dvec3{line.direction.x, line.grade, line.direction.y} * std::clamp(line.along(ground), 0.0, line.length);
			return make(kind::straight_tool, m_straights.tool == 1 ? "Press: break the straight here, drag to turn the rest" : "Click: start of the shift");
		}
	}
	if ((m_track_tab == track_tab::turnout || m_track_tab == track_tab::path) && m_hover.track != nullptr && switch_slides(*m_hover.track))
	{
		intent.track = m_hover.track;
		intent.path = m_hover.path;
		intent.position = m_hover.point;
		return make(kind::switch_slide, m_track_tab == track_tab::path ? "Click: select this switch, drag: slide it along the straight" : "Click: edit this switch, drag: slide it along the straight");
	}
	if (selected != nullptr && false == selected->m_paths.empty())
	{
		glm::dvec3 point;
		if (switch_reach(cursor_level(selected->m_paths.front().points[segment_data::point::start].y), point))
		{
			intent.track = selected;
			intent.position = point;
			return make(kind::switch_at, "Press and drag: a switch here, along sets its direction, sideways its side");
		}
	}
	if (m_track_tab == track_tab::lineside && m_fouling.tab().open)
		if (auto const hit{m_fouling.hit()}; hit.has_value())
		{
			intent.index = hit->first;
			intent.path = hit->second ? 1 : 0;
			intent.position = m_fouling.position_of(*hit);
			return make(kind::fouling, hit->second ? "Click: move this W17 to its place" : "Click: put the W17 here");
		}
	if (m_hover.track != nullptr)
	{
		intent.track = m_hover.track;
		intent.path = m_hover.path;
		intent.position = m_hover.point;
		switch (m_track_tab)
		{
		case track_tab::straights:
		case track_tab::route: return make(kind::select, m_hover.track->eType != tt_Switch && editor_track::is_straight(*m_hover.track, m_straights.tolerance) ? "Click: the whole straight through this track" : "Click: the curve of this track with its straights");
		case track_tab::turnout: return make(kind::select, m_hover.track->eType == tt_Switch ? "Click: edit this switch" : m_switch.armed < 0 ? "Click: choose this track, then the type of the switch" : "Click: choose this track, then press and drag on it for the switch");
		case track_tab::profile: return make(kind::select, "Click: the grade line of the whole line through this track; Shift: the route between the tracks");
		case track_tab::lineside: return make(kind::select, "Click: the line the objects go along");
		case track_tab::parallel: return make(kind::select, "Click: the straight the new track goes beside");
		default: return make(kind::select, "Click: select");
		}
	}
	// a drag on the empty ground selects the tracks in a box, a click there clears the selection
	if (m_track_tab == track_tab::path && false == alt)
	{
		intent.index = 1;
		intent.what = kind::box;
	}
	return intent;
}

void editor_mode::update_track_intent()
{
	using kind = track_intent::kind;
	if (track_gesture_on())
	{
		m_intent = {};
		return;
	}
	auto const &io{ImGui::GetIO()};
	m_intent = track_intent_at((io.KeyShift ? GLFW_MOD_SHIFT : 0) | (io.KeyCtrl ? GLFW_MOD_CONTROL : 0) | (io.KeyAlt ? GLFW_MOD_ALT : 0));
	switch (m_intent.what)
	{
	case kind::none:
	case kind::gizmo: break;
	case kind::point:
	case kind::straight_handle:
	case kind::route_vertex:
	case kind::route_grip: ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll); break;
	case kind::box:
		if (m_intent.track != nullptr)
			ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
		break;
	default: ImGui::SetMouseCursor(ImGuiMouseCursor_Hand); break;
	}
}

void editor_mode::tape_update()
{
	auto &tape{m_tape};
	if (false == tape.active || ImGui::GetIO().WantCaptureMouse)
		return;
	glm::vec2 const cursor{ImGui::GetIO().MousePos.x, ImGui::GetIO().MousePos.y};
	auto const ground{cursor_ground()};
	if (auto const end{snap_free_end(ground, cursor, nullptr, kRailCategory, {})}; end.track != nullptr)
		tape.mouse = end.position;
	else if (auto const under{track_under_cursor()}; under.track != nullptr)
		tape.mouse = under.point;
	else
		tape.mouse = ground;
}

std::string editor_mode::tape_readout() const
{
	auto const &tape{m_tape};
	if (tape.points.empty())
		return {};
	auto const &from{tape.points.front()};
	auto const &to{tape.points.size() >= 2 ? tape.points[1] : tape.mouse};
	auto const plan{geometry::plan_distance(from, to)};
	auto const rise{to.y - from.y};
	auto azimuth{glm::degrees(std::atan2(to.x - from.x, to.z - from.z))};
	if (azimuth < 0.0)
		azimuth += 360.0;
	auto text{format(STR_C("%.2f m in plan, %.2f m in space"), plan, glm::distance(from, to))};
	text += "\n" + format(STR_C("height %+.2f m"), rise) + (plan > 0.01 ? format(STR_C("   grade %+.1f per mille"), rise / plan * 1000.0) : std::string{});
	text += "\n" + format(STR_C("azimuth %.1f deg"), azimuth);
	if (tape.points.size() >= 2)
		text += "\n" + STR("Click: a new measure. M or Esc: done");
	return text;
}

void editor_mode::draw_tape() const
{
	auto const &tape{m_tape};
	if (false == tape.active)
		return;
	screen_projection const projection;
	auto *drawlist{ImGui::GetForegroundDrawList(ImGui::GetMainViewport())};
	auto const colour{IM_COL32(255, 230, 60, 255)};
	std::vector<glm::dvec3> points{tape.points};
	if (points.size() < 2)
		points.push_back(tape.mouse);
	for (std::size_t i = 0; i + 1 < points.size(); ++i)
		projection.line(drawlist, points[i], points[i + 1], colour, 2.0f);
	for (auto const &point : points)
	{
		ImVec2 screen;
		if (projection.project(point, screen))
		{
			drawlist->AddCircleFilled(screen, 4.0f, colour);
			drawlist->AddCircle(screen, 7.0f, IM_COL32(0, 0, 0, 200), 12, 1.5f);
		}
	}
}

void editor_mode::track_press(track_intent const &Intent)
{
	using kind = track_intent::kind;
	switch (Intent.what)
	{
	case kind::measure:
		if (m_tape.points.size() >= 2)
			m_tape.points.clear();
		m_tape.points.push_back(m_tape.mouse);
		return;
	case kind::route_trace:
		if (Intent.index >= 0)
			m_route.trace.points.erase(m_route.trace.points.begin() + Intent.index);
		else
			m_route.trace.points.push_back(Intent.position);
		route_trace_fit();
		return;
	case kind::lay_end:
		if (m_lay.points.empty() && start_extend(Intent.snap))
			return;
		[[fallthrough]];
	case kind::lay:
		GfxRenderer->Pick_Node_Callback([this](scene::basic_node * /*node*/) {
			if (viewport_click())
				lay_click();
		});
		return;
	case kind::sweep: sweep_press(); return;
	case kind::box: start_track_box(Intent.index == 1); return;
	case kind::pick_style:
		if (Intent.track != nullptr)
		{
			copy_track_style(*Intent.track);
			m_track_style.picking = false;
			m_track_style.open = true;
		}
		return;
	case kind::spread:
		if (Intent.track != nullptr)
			track_spread_apply(*Intent.track);
		return;
	case kind::point: point_drag_start(Intent.point); return;
	case kind::gizmo: return;
	case kind::straight_set:
	{
		auto *hovered{Intent.track};
		GfxRenderer->Pick_Node_Callback([this, hovered](scene::basic_node *node) {
			auto *track{hovered != nullptr ? hovered : dynamic_cast<TTrack *>(node)};
			if (viewport_click() && track != nullptr)
				toggle_straight_set(*track);
		});
		return;
	}
	case kind::straight_break: start_straight_gesture(1); return;
	case kind::detour: add_detour_point(); return;
	case kind::straight_tool:
		if (m_straights.tool == 1)
			start_straight_gesture(1);
		else
			place_straight_tool();
		return;
	case kind::straight_handle:
		m_straights.handle = Intent.index;
		handle_drag_start(Intent.position);
		return;
	case kind::route_vertex:
		m_route.vertex = Intent.index;
		m_route.grip = -1;
		handle_drag_start(Intent.position);
		return;
	case kind::route_grip:
		m_route.vertex = -1;
		m_route.grip = Intent.index;
		handle_drag_start(Intent.position);
		return;
	case kind::switch_at: start_switch_placement(); return;
	case kind::kilometrage:
		if (Intent.track != nullptr)
			m_km.pick(*Intent.track, Intent.position);
		return;
	case kind::srk_boundary: m_srk.boundary_click(); return;
	case kind::fouling:
		if (Intent.path == 1)
			m_fouling.fix(Intent.index);
		else
			m_fouling.place(Intent.index);
		return;
	case kind::extend_end:
		if (false == start_extend(Intent.snap))
			point_drag_start(Intent.snap.point);
		return;
	case kind::turntable_place:
	case kind::turntable_exit:
	case kind::turntable_fit: turntable_press(Intent); return;
	case kind::signal_place:
	case kind::signal_move: signal_press(Intent); return;
	case kind::switch_slide:
		if (Intent.track != nullptr)
		{
			if (m_track_tab == track_tab::path)
				m_track_set.clear();
			select_track(Intent.track);
			switch_slide_start(*Intent.track);
		}
		return;
	case kind::select:
	case kind::none: track_click_select(Intent.track); return;
	}
}

void editor_mode::track_click_select(TTrack *Hovered)
{
	GfxRenderer->Pick_Node_Callback([this, Hovered](scene::basic_node *node) {
		if (false == viewport_click())
			return;
		auto *track{Hovered != nullptr ? Hovered : dynamic_cast<TTrack *>(node)};
		if (m_track_tab == track_tab::path)
		{
			m_track_set.clear();
			if (track == nullptr)
			{
				m_node = nullptr;
				m_track_point = {};
				ui()->set_node(nullptr);
				return;
			}
		}
		select_track(track != nullptr ? track : node);
		if (m_track_tab == track_tab::lay && track != nullptr)
			show_track_tab(track_tab::parallel);
	});
}

void editor_mode::draw_track_intent() const
{
	using kind = track_intent::kind;
	auto const &intent{m_intent};
	auto const &io{ImGui::GetIO()};
	if (intent.what == kind::none || intent.what == kind::gizmo || io.WantCaptureMouse)
		return;
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
	ImVec2 at;
	if (projection.project(intent.position, at))
	{
		switch (intent.what)
		{
		case kind::point:
		case kind::straight_handle:
		case kind::route_vertex:
		case kind::route_grip: drawlist->AddCircle(at, 13.0f, overlay_color::marked, 24, 2.5f); break;
		case kind::switch_at:
		case kind::straight_tool:
			drawlist->AddCircleFilled(at, 4.0f, overlay_color::grip);
			drawlist->AddCircle(at, 11.0f, overlay_color::grip, 20, 2.0f);
			break;
		case kind::lay_end:
		case kind::extend_end:
		case kind::turntable_fit: drawlist->AddCircleFilled(at, 5.0f, IM_COL32(255, 60, 255, 255)); break;
		case kind::turntable_exit: drawlist->AddCircle(at, 11.0f, overlay_color::grip, 20, 2.0f); break;
		case kind::lay:
		case kind::route_trace: drawlist->AddCircle(at, 6.0f, IM_COL32(255, 255, 255, 200), 12, 1.5f); break;
		default: break;
		}
	}
	if (intent.action.empty() || false == track_readout().empty())
		return;
	std::string details;
	auto const *track{m_hover.track};
	if (intent.what == kind::select && track != nullptr && track == intent.track && ImGui::GetTime() - m_hover.since >= kHoverLabelDelay)
	{
		auto const &path{track->m_paths[std::min<std::size_t>(m_hover.path, track->m_paths.size() - 1)]};
		details = name_of(*track);
		details += "\n" + std::string{STR_C(type_of(*track))};
		if (track->eType == tt_Switch)
			details += m_hover.path == 0 ? STR(" (main)") : STR(" (branch)");
		bezier const curve{path};
		details += format("   L %.1f m", curve.plan_length());
		auto const radius{::track_mode::piece_plan_radius(path)};
		details += radius >= 0.5 ? format("   R %.0f m", radius) : std::string{"   "} + STR_C("straight");
		auto const velocity{editor_track::velocity(*track)};
		details += velocity > 0.0 ? format("   V %.0f km/h", velocity) : std::string{};
		double nearest{std::numeric_limits<double>::max()};
		double at{0.0};
		for (int k = 0; k <= 64; ++k)
			if (auto const distance{geometry::plan_distance(curve.point(k / 64.0), m_hover.point)}; distance < nearest)
			{
				nearest = distance;
				at = k / 64.0;
			}
		auto const tangent{curve.first(at)};
		if (auto const run{glm::length(plan_of(tangent))}; run > 1e-9)
			details += "\n" + format(STR_C("grade %+.1f per mille"), tangent.y / run * 1000.0);
		auto const roll{std::max(std::abs(path.rolls[0]), std::abs(path.rolls[1]))};
		if (roll > 1e-3f)
			details += format(STR_C("   cant %.0f mm"), 1500.0 * std::sin(glm::radians(static_cast<double>(roll))));
	}
	auto *foreground{ImGui::GetForegroundDrawList(ImGui::GetMainViewport())};
	auto const actionsize{ImGui::CalcTextSize(intent.action.c_str())};
	auto const detailsize{details.empty() ? ImVec2(0.0f, 0.0f) : ImGui::CalcTextSize(details.c_str())};
	ImVec2 const size{std::max(actionsize.x, detailsize.x), actionsize.y + (details.empty() ? 0.0f : detailsize.y + 4.0f)};
	ImVec2 position{io.MousePos.x + 18.0f, io.MousePos.y + 18.0f};
	position.x = std::min(position.x, io.DisplaySize.x - size.x - 10.0f);
	position.y = std::min(position.y, io.DisplaySize.y - size.y - 10.0f);
	foreground->AddRectFilled(ImVec2(position.x - 6.0f, position.y - 4.0f), ImVec2(position.x + size.x + 6.0f, position.y + size.y + 4.0f), IM_COL32(0, 0, 0, 170), 4.0f);
	foreground->AddText(position, overlay_color::marked, intent.action.c_str());
	if (false == details.empty())
		foreground->AddText(ImVec2(position.x, position.y + actionsize.y + 4.0f), IM_COL32(255, 255, 255, 235), details.c_str());
}

void editor_mode::track_context_press()
{
	auto &context{m_track_context};
	auto const &io{ImGui::GetIO()};
	context.armed = m_hover.track != nullptr;
	context.target = m_hover;
	context.pressed = std::chrono::steady_clock::now();
	context.angle = Camera.Angle;
	context.mouse = {io.MousePos.x, io.MousePos.y};
}

void editor_mode::track_context_release()
{
	auto &context{m_track_context};
	if (false == context.armed)
		return;
	context.armed = false;
	if (std::chrono::steady_clock::now() - context.pressed < kContextClick && glm::length(Camera.Angle - context.angle) < kContextTurn)
		context.pending = true;
}

void editor_mode::split_track_at(TTrack &Track, glm::dvec3 const &Point)
{
	auto const parameter{editor_track::nearest_parameter(Track, Point)};
	auto const before{editor_track::capture(Track)};
	if (auto *created{editor_track::split_path(Track, parameter)})
	{
		push_track_snapshot({{&Track, before}}, {created});
		straight_refresh();
	}
}

void editor_mode::focus_track(TTrack &Track, int const Path, glm::dvec3 const &Point)
{
	auto const &path{Track.m_paths[std::min<std::size_t>(std::max(Path, 0), Track.m_paths.size() - 1)]};
	bezier const curve{path};
	glm::dvec3 look{curve.first(std::clamp(editor_track::nearest_parameter(Track, Point), 0.0, 1.0))};
	look.y = 0.0;
	look = glm::length(look) > 1e-6 ? glm::normalize(look) : glm::dvec3(0.0, 0.0, 1.0);
	if (glm::dot(Camera.Pos - Point, look) > 0.0)
		look = -look;
	look = glm::normalize(look - glm::dvec3(0.0, 0.25, 0.0));
	fly_to(Point, look, kFocusDistance);
}

void editor_mode::render_track_context()
{
	auto &context{m_track_context};
	auto *track{context.target.track};
	if (context.pending)
	{
		context.pending = false;
		if (track != nullptr && false == track->m_editorremoved)
		{
			// a track of the set keeps the set, the menu acts on all of them
			if (false == (m_track_set.size() >= 2 && in_track_set(track)))
				select_track(track);
			ImGui::OpenPopup("##trackcontext");
			ImGui::SetNextWindowPos(ImVec2(context.mouse.x, context.mouse.y));
		}
	}
	if (false == ImGui::BeginPopup("##trackcontext"))
		return;
	if (track == nullptr || track->m_editorremoved)
	{
		ImGui::CloseCurrentPopup();
		ImGui::EndPopup();
		return;
	}
	auto const point{context.target.point};
	bool const normal{track->eType == tt_Normal && editor_track::is_supported(*track)};
	auto const tracks{selected_tracks()};
	auto const many{tracks.size() >= 2};
	ImGui::TextDisabled("%s  (%s, %.1f m)", name_of(*track).c_str(), STR_C(type_of(*track)), track->Length());
	if (many)
		ImGui::TextColored(ImVec4(1.0f, 0.65f, 0.2f, 1.0f), STR_P("one of %d selected tracks", "one of %d selected tracks", static_cast<long long>(tracks.size())), static_cast<int>(tracks.size()));
	ImGui::Separator();
	if (ImGui::MenuItem(STR_C("Copy its properties"), "Ctrl+Shift+C"))
		copy_track_style(*track);
	auto const paste{many ? format(STR_P("Paste the properties on %d track", "Paste the properties on %d tracks", static_cast<long long>(tracks.size())), static_cast<int>(tracks.size())) : std::string{STR_C("Paste the properties")}};
	if (ImGui::MenuItem(paste.c_str(), "Ctrl+Shift+V", false, false == m_track_style.values.empty()))
		paste_track_style(tracks);
	if (false == m_track_style.values.empty() && ImGui::IsItemHovered())
		ImGui::SetTooltip(STR_C("Properties of %s"), m_track_style.from.c_str());
	if (ImGui::BeginMenu(STR_C("Select alike")))
	{
		char const *const keys[] = {"rails", "trackbed", "speed", "environment", "circuits"};
		char const *const labels[] = {"The same rails", "The same trackbed", "The same speed", "The same environment", "The same track circuits"};
		for (std::size_t i = 0; i < std::size(keys); ++i)
			if (ImGui::MenuItem(STR_C(labels[i])))
			{
				auto const found{similar_tracks(track_property_index(keys[i]), {track})};
				select_tracks(found);
				ui()->set_status(format(STR_P("%s: %d track selected in the scenery", "%s: %d tracks selected in the scenery", static_cast<long long>(found.size())), STR_C(labels[i]), static_cast<int>(found.size())), false);
			}
		ImGui::EndMenu();
	}
	ImGui::Separator();
	if (ImGui::MenuItem(STR_C("Split here"), "K", false, normal))
		split_track_at(*track, point);
	ImGui::Separator();
	if (ImGui::MenuItem(STR_C("Switch"), "T", false, normal || track->eType == tt_Switch))
		show_track_tab(track_tab::turnout);
	if (ImGui::MenuItem(STR_C("Plan"), "G", false, normal))
		show_track_tab(track_tab::straights);
	if (ImGui::MenuItem(STR_C("Parallel track"), "N", false, normal))
	{
		select_track(track);
		show_track_tab(track_tab::parallel);
	}
	if (ImGui::MenuItem(STR_C("Place a vehicle here")))
	{
		show_track_tab(track_tab::lineside);
		m_vehicle.start(*track, point);
	}
	if (ImGui::MenuItem(STR_C("Hectometre posts from here")))
	{
		show_track_tab(track_tab::lineside);
		m_hekto.start(*track, point);
	}
	if (ImGui::MenuItem(STR_C("Fouling point markers"), nullptr, false, track->eType == tt_Switch))
	{
		show_track_tab(track_tab::lineside);
		m_fouling.check_selected_switch();
	}
	if (ImGui::MenuItem(STR_C("Vertical profile of the line"), "P"))
	{
		show_track_tab(track_tab::profile);
		profile_open_run(*track);
	}
	if (ImGui::MenuItem(STR_C("Speed check of the line"), "V"))
	{
		show_track_tab(track_tab::speed);
		m_speed.check_line();
	}
	ImGui::Separator();
	if (ImGui::MenuItem(track_selected(track) ? STR_C("Take out of the selection") : STR_C("Add to the selection"), STR_C("Shift+LMB")))
		track_set_toggle(*track);
	if (ImGui::MenuItem(STR_C("Look at it")))
		focus_track(*track, context.target.path, point);
	if (ImGui::MenuItem(STR_C("Copy the name"), nullptr, false, false == track->name().empty()))
		ImGui::SetClipboardText(track->name().c_str());
	ImGui::Separator();
	if (many)
	{
		if (ImGui::MenuItem(format(STR_P("Delete %d track", "Delete %d tracks", static_cast<long long>(tracks.size())), static_cast<int>(tracks.size())).c_str(), "Del"))
			track_set_delete();
	}
	else if (ImGui::MenuItem(STR_C("Delete"), "Del"))
		delete_selected_track();
	ImGui::EndPopup();
}

void editor_mode::render_track_search()
{
	auto &search{m_track_search};
	if (search.focus)
	{
		ImGui::SetKeyboardFocusHere();
		search.focus = false;
	}
	ImGui::SetNextItemWidth(-1.0f);
	ImGui::InputTextWithHint("##findtrack", STR_C("Find a track by its name (Ctrl+F)"), search.text, sizeof(search.text));
	std::string const text{ToLower(search.text)};
	if (text.empty())
	{
		search.searched.clear();
		search.found.clear();
		return;
	}
	if (text != search.searched)
	{
		search.searched = text;
		search.found.clear();
		search.total = 0;
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || false == rail(*track) || ToLower(track->name()).find(text) == std::string::npos)
				continue;
			++search.total;
			if (search.found.size() < kSearchLimit)
				search.found.push_back(track);
		}
	}
	if (search.found.empty())
	{
		ImGui::TextDisabled("%s", STR_C("No track of that name"));
		return;
	}
	if (search.total > search.found.size())
		ImGui::TextDisabled(STR_C("%zu found, the first %zu listed"), search.total, search.found.size());
	auto const rows{std::min<std::size_t>(search.found.size(), 8)};
	ImGui::BeginChild("##foundtracks", ImVec2(0.0f, ImGui::GetTextLineHeightWithSpacing() * static_cast<float>(rows) + 6.0f), true);
	for (std::size_t i = 0; i < search.found.size(); ++i)
	{
		auto *track{search.found[i]};
		if (track->m_editorremoved)
			continue;
		auto const label{format("%s##found%zu", name_of(*track).c_str(), i)};
		if (ImGui::Selectable(label.c_str(), track == selected_track()))
		{
			m_node = track;
			m_track_point = {};
			ui()->set_node(m_node);
			focus_track(*track, 0, bezier{track->m_paths.front()}.point(0.5));
		}
		ImGui::SameLine(ImGui::GetWindowContentRegionMax().x * 0.6f);
		ImGui::TextDisabled("%s, %.1f m", STR_C(type_of(*track)), track->Length());
	}
	ImGui::EndChild();
}

char const *editor_mode::typed_meaning() const
{
	auto const &straights{m_straights};
	if (straights.dragging)
		return straights.handle == 2 ? "type the shift, + to the right" : "type the length";
	if (straights.tool == 1 && straights.tool_mouse)
		return "type the turn in degrees, + to the right";
	if (m_lay.active && false == m_lay.points.empty())
		return "type the length of the next leg, Enter adds the point";
	if (m_extend.active && (m_track_layout == track_layout::arc_straight || m_track_layout == track_layout::straight_arc))
		return "type the radius of the curve";
	return nullptr;
}

std::vector<double> editor_mode::typed_values() const
{
	std::vector<double> values;
	std::string text{m_typed};
	std::replace(text.begin(), text.end(), ',', '.');
	std::size_t at{0};
	while (at < text.size())
	{
		auto const end{text.find(' ', at)};
		auto const word{text.substr(at, end == std::string::npos ? std::string::npos : end - at)};
		if (false == word.empty() && word != "-" && word != ".")
			values.push_back(std::strtod(word.c_str(), nullptr));
		if (end == std::string::npos)
			break;
		at = end + 1;
	}
	return values;
}

bool editor_mode::typed_key(int const Key)
{
	if (typed_meaning() == nullptr)
	{
		m_typed.clear();
		return false;
	}
	char character{'\0'};
	if (Key >= GLFW_KEY_0 && Key <= GLFW_KEY_9)
		character = static_cast<char>('0' + (Key - GLFW_KEY_0));
	else if (Key >= GLFW_KEY_KP_0 && Key <= GLFW_KEY_KP_9)
		character = static_cast<char>('0' + (Key - GLFW_KEY_KP_0));
	else if (Key == GLFW_KEY_PERIOD || Key == GLFW_KEY_COMMA || Key == GLFW_KEY_KP_DECIMAL)
		character = '.';
	else if (Key == GLFW_KEY_MINUS || Key == GLFW_KEY_KP_SUBTRACT)
		character = '-';
	else if (Key == GLFW_KEY_SPACE)
		character = ' ';
	else if (Key == GLFW_KEY_BACKSPACE && false == m_typed.empty())
	{
		m_typed.pop_back();
		return true;
	}
	else
		return false;
	auto const start{m_typed.empty() || m_typed.back() == ' '};
	if ((character == '-' && false == start) || (character == ' ' && start) || (character == '.' && m_typed.substr(m_typed.rfind(' ') == std::string::npos ? 0 : m_typed.rfind(' ')).find('.') != std::string::npos))
		return true;
	if (m_typed.size() < 24)
		m_typed += character;
	return true;
}

glm::dvec3 editor_mode::typed_lay(glm::dvec3 const &Mouse) const
{
	auto const values{typed_values()};
	if (values.empty() || m_lay.points.empty() || false == (values[0] > 0.0 && values[0] <= 50000.0))
		return Mouse;
	auto const &last{m_lay.points.back()};
	glm::dvec2 heading{Mouse.x - last.x, Mouse.z - last.z};
	if (glm::length(heading) < 1e-6)
		return Mouse;
	heading = glm::normalize(heading) * values[0];
	return {last.x + heading.x, Mouse.y, last.z + heading.y};
}

void editor_mode::typed_turn()
{
	auto &state{m_straights};
	auto const values{typed_values()};
	if (values.empty() || state.tool != 1)
		return;
	auto const reach{std::max(1.0, glm::length(glm::dvec2{state.tool_handle.x - state.tool_point.x, state.tool_handle.z - state.tool_point.z}))};
	auto const heading{geometry::turned(state.tool_line.direction, glm::radians(values[0])) * reach};
	state.tool_handle = {state.tool_point.x + heading.x, state.tool_handle.y, state.tool_point.z + heading.y};
}

bool editor_mode::in_track_set(TTrack const *Track) const
{
	return std::find(m_track_set.begin(), m_track_set.end(), Track) != m_track_set.end();
}

bool editor_mode::track_selected(TTrack const *Track) const
{
	return Track != nullptr && (m_track_set.size() >= 2 ? in_track_set(Track) : Track == m_node);
}

void editor_mode::track_set_prune()
{
	m_track_set.erase(std::remove_if(m_track_set.begin(), m_track_set.end(), [](TTrack const *Track) { return Track == nullptr || Track->m_editorremoved; }), m_track_set.end());
	// the set is the selection only together with the selected track, another one chosen elsewhere ends it
	if (false == m_track_set.empty() && false == in_track_set(selected_track()))
		m_track_set.clear();
}

void editor_mode::track_set_toggle(TTrack &Track)
{
	track_set_prune();
	// the set is the selection only with the selected track in it, a set left from before is forgotten
	if (m_track_set.size() < 2 || false == in_track_set(selected_track()))
		m_track_set.clear();
	if (m_track_set.empty() && &Track == selected_track())
	{
		select_tracks({});
		return;
	}
	if (m_track_set.empty())
		if (auto *selected{selected_track()}; selected != nullptr && selected != &Track)
			m_track_set.push_back(selected);
	if (auto const found{std::find(m_track_set.begin(), m_track_set.end(), &Track)}; found != m_track_set.end())
		m_track_set.erase(found);
	else
		m_track_set.push_back(&Track);
	m_node = m_track_set.empty() ? nullptr : m_track_set.back();
	m_track_point = {};
	ui()->set_node(m_node);
}

void editor_mode::start_track_box(bool const Replace)
{
	auto const &io{ImGui::GetIO()};
	m_track_box.active = true;
	m_track_box.replace = Replace;
	m_track_box.to = {io.MousePos.x, io.MousePos.y};
	m_track_box.from = m_track_box.to;
	m_track_box.hovered = m_hover.track;
}

void editor_mode::finish_track_box()
{
	auto &box{m_track_box};
	if (false == box.active)
		return;
	box.active = false;
	auto const &io{ImGui::GetIO()};
	box.to = {io.MousePos.x, io.MousePos.y};
	if (glm::length(box.to - box.from) < 6.0f)
	{
		// a click on the empty ground: what the renderer finds there, a part of a track seen close up, or nothing
		if (box.replace)
			track_click_select(nullptr);
		else if (box.hovered != nullptr && false == box.hovered->m_editorremoved)
			track_set_toggle(*box.hovered);
		return;
	}
	glm::vec2 const low{std::min(box.from.x, box.to.x), std::min(box.from.y, box.to.y)};
	glm::vec2 const high{std::max(box.from.x, box.to.x), std::max(box.from.y, box.to.y)};
	screen_projection const projection;
	track_set_prune();
	if (m_track_set.size() < 2 || false == in_track_set(selected_track()) || box.replace)
		m_track_set.clear();
	if (m_track_set.empty() && false == box.replace)
		if (auto *selected{selected_track()}; selected != nullptr)
			m_track_set.push_back(selected);
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || false == rail(*track) || in_track_set(track))
			continue;
		bool inside{false};
		for (auto const &path : track->m_paths)
		{
			bezier const curve{path};
			for (int k = 0; k <= 8 && false == inside; ++k)
			{
				ImVec2 screen;
				if (projection.project(curve.point(k / 8.0), screen))
					inside = screen.x >= low.x && screen.x <= high.x && screen.y >= low.y && screen.y <= high.y;
			}
			if (inside)
				break;
		}
		if (inside)
			m_track_set.push_back(track);
	}
	auto const found{m_track_set};
	select_tracks(found);
}

std::vector<TTrack *> editor_mode::track_spread_from(TTrack &Start) const
{
	auto const &rules{m_spread};
	auto const bed = [](TTrack const &Track) { return Track.eType == tt_Switch ? (Track.SwitchExtension ? Track.SwitchExtension->m_material3 : null_handle) : Track.m_material2; };
	auto const startspeed{editor_track::velocity(Start)};
	auto const alike = [&](TTrack const &Track) {
		if (rules.same_rails && Track.m_material1 != Start.m_material1)
			return false;
		if (rules.same_bed && bed(Track) != bed(Start))
			return false;
		if (rules.same_speed && std::abs(editor_track::velocity(Track) - startspeed) > 0.5)
			return false;
		return true;
	};
	auto const exits = [&](TTrack const &Track) {
		std::vector<glm::dvec3> result;
		if (Track.m_paths.empty())
			return result;
		auto const &main{Track.m_paths.front()};
		result = {main.points[segment_data::point::start], main.points[segment_data::point::end]};
		if (Track.eType == tt_Switch && rules.stop == 2)
			for (std::size_t i = 1; i < Track.m_paths.size(); ++i)
				result.push_back(Track.m_paths[i].points[segment_data::point::end]);
		return result;
	};
	std::size_t constexpr limit{20000};
	std::vector<TTrack *> result;
	std::unordered_map<TTrack const *, double> reached{{&Start, 0.0}};
	std::unordered_set<TTrack const *> done;
	using entry = std::pair<double, TTrack *>;
	std::priority_queue<entry, std::vector<entry>, std::greater<entry>> queue;
	queue.push({0.0, &Start});
	while (false == queue.empty() && result.size() < limit)
	{
		auto const [distance, track] = queue.top();
		queue.pop();
		if (false == done.insert(track).second)
			continue;
		result.push_back(track);
		auto const further{distance + track->Length()};
		if (rules.reach > 0.f && further > static_cast<double>(rules.reach))
			continue;
		for (auto const &end : exits(*track))
			for (auto const &[other, point] : editor_track::connected_points(*track, end))
			{
				if (other == nullptr || done.count(other) > 0 || false == rail(*other) || other->eType == tt_Table || false == alike(*other))
					continue;
				if (other->eType == tt_Switch && (rules.stop == 0 || (rules.stop == 1 && point.path != 0)))
					continue;
				auto const known{reached.find(other)};
				if (known != reached.end() && known->second <= further)
					continue;
				reached[other] = further;
				queue.push({further, other});
			}
	}
	return result;
}

std::vector<TTrack *> const &editor_mode::track_spread_preview(TTrack &Start) const
{
	auto &spread{m_spread};
	auto const signature{spread.stop | (spread.same_rails ? 4 : 0) | (spread.same_bed ? 8 : 0) | (spread.same_speed ? 16 : 0) | (static_cast<int>(spread.reach) << 5)};
	if (spread.from != &Start || spread.signature != signature)
	{
		spread.from = &Start;
		spread.signature = signature;
		spread.preview = track_spread_from(Start);
		spread.length = 0.0;
		for (auto const *track : spread.preview)
			spread.length += track->Length();
	}
	return spread.preview;
}

void editor_mode::track_spread_apply(TTrack &Start)
{
	track_set_prune();
	if (m_track_set.size() < 2 || false == in_track_set(selected_track()))
		m_track_set.clear();
	if (m_track_set.empty())
		if (auto *selected{selected_track()}; selected != nullptr && selected != &Start)
			m_track_set.push_back(selected);
	auto const taken{track_spread_from(Start)};
	if (in_track_set(&Start))
	{
		std::unordered_set<TTrack const *> const out(taken.begin(), taken.end());
		m_track_set.erase(std::remove_if(m_track_set.begin(), m_track_set.end(), [&](TTrack const *Track) { return out.count(Track) > 0; }), m_track_set.end());
	}
	else
		for (auto *track : taken)
			if (false == in_track_set(track))
				m_track_set.push_back(track);
	m_spread.from = nullptr;
	auto const found{m_track_set};
	select_tracks(found);
}

void editor_mode::track_set_grow()
{
	track_set_prune();
	auto const reach{m_spread.reach};
	std::vector<TTrack *> added;
	for (auto *track : m_track_set)
	{
		m_spread.reach = std::max(0.01f, static_cast<float>(track->Length()) - 0.01f);
		for (auto *other : track_spread_from(*track))
			if (false == in_track_set(other) && std::find(added.begin(), added.end(), other) == added.end())
				added.push_back(other);
	}
	m_spread.reach = reach;
	m_spread.from = nullptr;
	m_track_set.insert(m_track_set.end(), added.begin(), added.end());
}

void editor_mode::render_track_spread_ui()
{
	auto &spread{m_spread};
	auto const open{ImGui::CollapsingHeader(STR_C("Spread along the joins"))};
	item_tooltip("Ctrl+Shift+LMB on a track adds the tracks joined to it to the selection.\nHolding Ctrl+Shift shows what the click takes; on a track of the selection it takes them out");
	if (false == open)
		return;
	auto const gap{std::round(ImGui::GetFontSize() * 0.3f)};
	auto const width{std::floor((ImGui::GetContentRegionAvail().x - 2.0f * gap) / 3.0f)};
	auto const at = [](ImVec2 const Low, ImVec2 const High, float const X, float const Y) { return ImVec2(Low.x + (High.x - Low.x) * X, Low.y + (High.y - Low.y) * Y); };
	char const *const labels[]{STR_C("Up to a switch"), STR_C("Main track on"), STR_C("Everything joined")};
	char const *const tooltips[]{"Stop before the switches", "Through the switches, along their main track", "Through everything joined"};
	for (int i = 0; i < 3; ++i)
	{
		if (i > 0)
			ImGui::SameLine(0.0f, gap);
		ImGui::PushID(i);
		if (sketch_tile("##spreadstop", labels[i], spread.stop == i, width, [&](ImDrawList &List, ImVec2 Low, ImVec2 High, ImU32 Line, ImU32 Made) {
			    auto const dim{(Line & ~IM_COL32_A_MASK) | IM_COL32(0, 0, 0, 70)};
			    List.AddLine(at(Low, High, 0.0f, 0.75f), at(Low, High, 1.0f, 0.75f), dim, 2.0f);
			    List.AddBezierQuadratic(at(Low, High, 0.45f, 0.75f), at(Low, High, 0.7f, 0.75f), at(Low, High, 1.0f, 0.15f), dim, 2.0f);
			    List.AddLine(at(Low, High, 0.0f, 0.75f), at(Low, High, i == 0 ? 0.4f : 1.0f, 0.75f), Made, 2.5f);
			    if (i == 2)
				    List.AddBezierQuadratic(at(Low, High, 0.45f, 0.75f), at(Low, High, 0.7f, 0.75f), at(Low, High, 1.0f, 0.15f), Made, 2.5f);
			    List.AddCircleFilled(at(Low, High, 0.12f, 0.75f), 3.0f, IM_COL32(230, 90, 70, 255));
		    }))
			spread.stop = i;
		item_tooltip(tooltips[i]);
		ImGui::PopID();
	}
	ImGui::PushID("spread");
	if (ImGui::TreeNode(STR_C("More settings")))
	{
		ImGui::TextDisabled("%s", STR_C("Only to the paths like the clicked one:"));
		toggle_chips("spreadsame", {{STR_C("rails"), &spread.same_rails}, {STR_C("trackbed"), &spread.same_bed}, {STR_C("speed"), &spread.same_speed}});
		if (begin_fields("##spreadfields", 0.55f))
		{
			field_row("Reach along the track (m)");
			ImGui::InputFloat("##spreadreach", &spread.reach, 50.f, 500.f, "%.0f");
			item_tooltip("How far along the tracks the spread goes from the clicked one; 0: no limit");
			ImGui::EndTable();
		}
		ImGui::TreePop();
	}
	ImGui::PopID();
	spread.reach = std::clamp(spread.reach, 0.f, 100000.f);
}

void editor_mode::draw_track_spread() const
{
	if (m_intent.what != track_intent::kind::spread || m_spread.from == nullptr)
		return;
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
	auto const removing{in_track_set(m_spread.from)};
	auto const colour{removing ? IM_COL32(255, 90, 70, 220) : IM_COL32(90, 220, 255, 220)};
	for (auto const *track : m_spread.preview)
	{
		if (track->m_editorremoved)
			continue;
		for (auto const &path : track->m_paths)
		{
			bezier const curve{path};
			auto const count{std::clamp(static_cast<int>(curve.plan_length() / 4.0), 4, 48)};
			auto previous{curve.point(0.0)};
			for (int k = 1; k <= count; ++k)
			{
				auto const next{curve.point(static_cast<double>(k) / count)};
				projection.line(drawlist, previous, next, colour, 2.5f);
				previous = next;
			}
		}
	}
}

void editor_mode::draw_track_set() const
{
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
	if (m_track_box.active)
	{
		auto const &io{ImGui::GetIO()};
		ImVec2 const from{m_track_box.from.x, m_track_box.from.y};
		drawlist->AddRectFilled(from, io.MousePos, IM_COL32(255, 160, 40, 40));
		drawlist->AddRect(from, io.MousePos, IM_COL32(255, 160, 40, 220), 0.0f, 0, 1.5f);
	}
	auto const *listed{m_track_list_hover_frame >= ImGui::GetFrameCount() - 1 ? m_track_list_hover : nullptr};
	if (listed != nullptr && false == listed->m_editorremoved)
		for (auto const &path : listed->m_paths)
		{
			bezier const curve{path};
			auto const count{std::clamp(static_cast<int>(curve.plan_length() / 3.0), 6, 64)};
			auto previous{curve.point(0.0)};
			for (int k = 1; k <= count; ++k)
			{
				auto const next{curve.point(static_cast<double>(k) / count)};
				projection.line(drawlist, previous, next, IM_COL32(255, 255, 255, 240), 6.0f);
				previous = next;
			}
		}
	if (m_track_set.size() < 2 || false == in_track_set(selected_track()))
		return;
	for (auto const *track : m_track_set)
	{
		if (track->m_editorremoved)
			continue;
		for (auto const &path : track->m_paths)
		{
			bezier const curve{path};
			auto const count{std::clamp(static_cast<int>(curve.plan_length() / 3.0), 6, 64)};
			auto previous{curve.point(0.0)};
			for (int k = 1; k <= count; ++k)
			{
				auto const next{curve.point(static_cast<double>(k) / count)};
				projection.line(drawlist, previous, next, IM_COL32(255, 160, 40, 230), 3.0f);
				previous = next;
			}
		}
	}
}

void editor_mode::track_set_delete()
{
	track_set_prune();
	std::vector<TTrack *> removed;
	std::size_t kept{0};
	for (auto *track : m_track_set)
	{
		if (false == track->Dynamics.empty() || false == scene::Layers.editable(track))
		{
			++kept;
			continue;
		}
		editor_track::retire(*track);
		removed.push_back(track);
		if (std::find(m_route.chain.tracks.begin(), m_route.chain.tracks.end(), track) != m_route.chain.tracks.end())
			m_route = {};
	}
	push_track_snapshot({}, {}, removed);
	m_track_set.clear();
	m_node = nullptr;
	ui()->set_node(nullptr);
	m_track_point = {};
	straight_refresh();
	auto status{format(STR_P("%zu track deleted, Ctrl+Z brings it back", "%zu tracks deleted, Ctrl+Z brings them back", static_cast<long long>(removed.size())), removed.size())};
	if (kept > 0)
		status += format(STR_C(", %zu with vehicles on them or in the files which can't be changed are left"), kept);
	ui()->set_status(status, false);
}

void editor_mode::render_track_set_ui()
{
	track_set_prune();
	if (m_track_set.size() < 2 || false == in_track_set(selected_track()))
		return;
	ImGui::TextColored(ImVec4(1.0f, 0.65f, 0.2f, 1.0f), STR_P("%d track selected", "%d tracks selected", static_cast<long long>(m_track_set.size())), static_cast<int>(m_track_set.size()));
	ImGui::PushTextWrapPos(0.0f);
	ImGui::TextDisabled("%s", STR_C("Their properties are in the inspector, the gizmo moves them together"));
	ImGui::PopTextWrapPos();
	m_parallel.render_set_options();
	ImGui::Separator();
}

bool editor_mode::track_set_begin(bool const Rotate)
{
	std::string reason;
	if (false == tracks_editable(m_track_set, reason))
	{
		ui()->set_status(reason, true);
		return false;
	}
	m_track_set_using = true;
	m_track_gizmo_drag = {};
	m_track_gizmo_drag.operation = Rotate ? gizmo_operation::rotate : gizmo_operation::translate;
	m_track_drag = m_track_set;
	m_track_set_joints.clear();
	for (auto *track : m_track_set)
	{
		auto joints{editor_track::joints(*track)};
		joints.erase(std::remove_if(joints.begin(), joints.end(), [&](editor_track::joint const &Joint) { return in_track_set(Joint.other); }), joints.end());
		for (auto const &joint : joints)
			if (std::find(m_track_drag.begin(), m_track_drag.end(), joint.other) == m_track_drag.end())
				m_track_drag.push_back(joint.other);
		m_track_set_joints.emplace_back(track, std::move(joints));
	}
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	for (auto *track : m_track_drag)
		states.emplace_back(track, editor_track::capture(*track));
	push_track_snapshot(std::move(states));
	records_watch_begin();
	track_gizmo_hold();
	return true;
}

void editor_mode::track_set_turn(double const Angle)
{
	auto &drag{m_track_gizmo_drag};
	auto const step{std::remainder(Angle - drag.angle, 2.0 * glm::pi<double>())};
	if (std::abs(step) < 1e-9)
		return;
	drag.angle += step;
	for (auto &entry : m_track_set_joints)
		editor_track::rotate_y(*entry.first, m_track_set_pivot, step);
	for (auto &entry : m_track_set_joints)
		editor_track::follow(*entry.first, entry.second);
	m_track_dirty = true;
}

void editor_mode::track_set_move(glm::dvec3 const &Offset)
{
	for (auto &entry : m_track_set_joints)
	{
		editor_track::translate(*entry.first, Offset);
		editor_track::follow(*entry.first, entry.second);
	}
	m_track_dirty = true;
}

void editor_mode::track_set_end()
{
	track_gizmo_settle();
	m_track_dirty = true;
	commit_track_drag(true);
	records_watch_end();
	m_track_set_using = false;
	m_track_drag.clear();
	m_track_set_joints.clear();
	straight_refresh();
}

glm::dvec3 editor_mode::track_set_centre() const
{
	glm::dvec3 centre{0.0};
	for (auto const *track : m_track_set)
		centre += editor_track::pivot(*track);
	return m_track_set.empty() ? centre : centre / static_cast<double>(m_track_set.size());
}

void editor_mode::render_track_set_gizmo()
{
	track_set_prune();
	if (m_track_set.size() < 2)
	{
		m_track_set_using = false;
		return;
	}
	gizmo_frame const frame;
	if (false == m_track_set_using)
	{
		m_track_set_pivot = track_set_centre();
		m_track_set_gizmo = glm::translate(glm::mat4(1.0f), glm::vec3(m_track_set_pivot - frame.camera));
	}
	auto const rotate{m_track_set_using ? m_track_gizmo_drag.operation == gizmo_operation::rotate : m_gizmo_op == gizmo_operation::rotate};
	glm::mat4 delta(1.0f);
	frame.manipulate(rotate ? ImGuizmo::ROTATE_Y : ImGuizmo::TRANSLATE, m_track_set_gizmo, rotate ? 5.0f : m_gizmo_snap, &delta);
	if (ImGuizmo::IsUsing())
	{
		if (false == m_track_set_using && false == track_set_begin(rotate))
			return;
		if (rotate)
			track_set_turn(std::atan2(static_cast<double>(m_track_set_gizmo[2][0]), static_cast<double>(m_track_set_gizmo[0][0])));
		else if (glm::dvec3 const offset{delta[3]}; glm::length(offset) > 1e-9)
			track_set_move(offset);
		commit_track_drag(false);
	}
	else if (m_track_set_using)
		track_set_end();
}

void editor_mode::render_lineside_ui()
{
	struct tab
	{
		char const *label;
		bool *open;
		bool *expand;
		std::function<void()> render;
	};
	std::array<tab, 3> const tabs{{
	    {"Hectometre posts", &m_hekto.tab().open, &m_hekto.tab().expand, [this]() { m_hekto.render_options(); }},
	    {"Fouling points", &m_fouling.tab().open, &m_fouling.tab().expand, [this]() { m_fouling.render_options(); }},
	    {"Trainset", &m_vehicle.tab().open, &m_vehicle.tab().expand, [this]() { m_vehicle.render_options(); }},
	}};
	if (selected_track() == nullptr)
		ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f), "%s", STR_C("LMB on a track: the line the objects go along"));
	if (false == ImGui::BeginTabBar("##lineside"))
		return;
	for (int i = 0; i < static_cast<int>(tabs.size()); ++i)
	{
		auto const &entry{tabs[i]};
		auto const wanted{*entry.expand};
		*entry.expand = false;
		*entry.open = ImGui::BeginTabItem(STR_C(entry.label), nullptr, wanted ? ImGuiTabItemFlags_SetSelected : 0);
		if (false == *entry.open)
			continue;
		m_lineside_tab = i;
		entry.render();
		ImGui::EndTabItem();
	}
	ImGui::EndTabBar();
}
