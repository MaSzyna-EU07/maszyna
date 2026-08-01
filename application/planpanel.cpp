/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/planpanel.h"

#include "application/editormode.h"
#include "editor/polandmap.h"
#include "editor/turnouts.h"
#include "rendering/renderer.h"
#include "utilities/Globals.h"

#include "maj0sted/web/ribbon.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <numbers>
#include <set>
#include <sstream>

namespace
{

// the plan works in EPSG:2180, the scenery in the engine's world space around its own zero. the
// georeference says which point of the projection that zero stands for; a fictional scenery leaves
// it at the origin, so the two frames differ only in the naming of the axes - easting runs along
// world x, northing against world z
glm::dvec3 plan_to_world(double const X, double const Y)
{
	return {X - Global.scenery_origin.x, 0.0, -(Y - Global.scenery_origin.y)};
}

void world_to_plan(glm::dvec3 const &World, double &X, double &Y)
{
	X = World.x + Global.scenery_origin.x;
	Y = -World.z + Global.scenery_origin.y;
}

// places a world point on screen using the camera of the most recent colour pass. the engine renders
// camera-relative, so the view matrix carries rotation only and the position is subtracted here
bool world_to_screen(glm::dvec3 const &World, ImVec2 &Screen)
{
	auto const relative{glm::vec3(World - GfxRenderer->Camera_Position())};
	auto const clip{GfxRenderer->Camera_Projection_Matrix() * GfxRenderer->Camera_View_Matrix() * glm::vec4(relative, 1.0f)};

	if (clip.w <= 0.0f)
	{
		return false; // behind the camera
	}

	auto const ndc{glm::vec3(clip) / clip.w};
	auto const display{ImGui::GetIO().DisplaySize};
	Screen = {(ndc.x * 0.5f + 0.5f) * display.x, (0.5f - ndc.y * 0.5f) * display.y};

	return true;
}

// ground under the cursor in the top-down plan view. depth picking is useless here: empty scenery
// has nothing to hit, so the last (or zero) offset would pin every click to the same spot and the
// solver would drop the resulting zero-length straight
glm::dvec3 ortho_cursor_world()
{
	auto const width = std::max(1, Global.window_size.x);
	auto const height = std::max(1, Global.window_size.y);
	auto const ndcx = (static_cast<double>(Global.cursor_pos.x) / width) * 2.0 - 1.0;
	auto const ndcy = 1.0 - (static_cast<double>(Global.cursor_pos.y) / height) * 2.0;
	auto const halfheight = static_cast<double>(Global.editor_ortho_extent);
	auto const halfwidth = halfheight * static_cast<double>(width) / static_cast<double>(height);
	// yaw is pinned to north: view x = world x, view y = -world z
	return {Global.pCamera.Pos.x + ndcx * halfwidth, 0.0, Global.pCamera.Pos.z - ndcy * halfheight};
}

ImU32 element_colour(int const Kind, bool const Active)
{
	auto const alpha{Active ? 255 : 210};
	switch (Kind)
	{
	case 1: // arc
		return IM_COL32(255, 190, 90, alpha);
	case 2: // transition curve
		return IM_COL32(140, 245, 150, alpha);
	default: // straight
		return IM_COL32(125, 200, 255, alpha);
	}
}

// one centreline element recovered from a left/right rail pair
// dual rails from a centreline, same 1.5 m gauge as maj0sted::render::RailRenderer
std::vector<maj0sted::web::WebPolyline> rails_from_centreline(std::vector<maj0sted::web::WebPolyline> const &Centre, double const Gauge = 1.5)
{
	std::vector<maj0sted::web::WebPolyline> out;
	auto const half{Gauge * 0.5};
	for (auto const &poly : Centre)
	{
		if (poly.points.size() < 2)
		{
			continue;
		}
		for (double const side : {half, -half})
		{
			maj0sted::web::WebPolyline rail;
			rail.kind = poly.kind;
			rail.length = poly.length;
			rail.radius_start = poly.radius_start;
			rail.radius_end = poly.radius_end;
			rail.points.resize(poly.points.size());
			auto const m{poly.points.size()};
			for (std::size_t i = 0; i < m; ++i)
			{
				double dx{1.0};
				double dy{0.0};
				if (i == 0)
				{
					dx = poly.points[1].x - poly.points[0].x;
					dy = poly.points[1].y - poly.points[0].y;
				}
				else if (i + 1 == m)
				{
					dx = poly.points[i].x - poly.points[i - 1].x;
					dy = poly.points[i].y - poly.points[i - 1].y;
				}
				else
				{
					dx = poly.points[i + 1].x - poly.points[i - 1].x;
					dy = poly.points[i + 1].y - poly.points[i - 1].y;
				}
				auto const len{std::hypot(dx, dy)};
				auto const nx{len > 0.0 ? -dy / len : 0.0};
				auto const ny{len > 0.0 ? dx / len : 0.0};
				rail.points[i] = {poly.points[i].x + nx * side, poly.points[i].y + ny * side};
			}
			out.push_back(std::move(rail));
		}
	}
	return out;
}

void draw_label(ImDrawList *Drawlist, ImVec2 const &Anchor, char const *Text, ImU32 const Colour)
{
	auto const size{ImGui::CalcTextSize(Text)};
	ImVec2 const pad{4.0f, 2.0f};
	ImVec2 const origin{Anchor.x + 6.0f, Anchor.y - size.y * 0.5f};
	Drawlist->AddRectFilled({origin.x - pad.x, origin.y - pad.y}, {origin.x + size.x + pad.x, origin.y + size.y + pad.y}, IM_COL32(15, 20, 32, 210), 3.0f);
	Drawlist->AddRect({origin.x - pad.x, origin.y - pad.y}, {origin.x + size.x + pad.x, origin.y + size.y + pad.y}, Colour, 3.0f);
	Drawlist->AddText(origin, Colour, Text);
}

// leader from the curve midpoint outward, then a boxed label (R= / KP L=)
void draw_curve_callout(ImDrawList *Drawlist, std::vector<ImVec2> const &Screen, char const *Text, ImU32 const Colour)
{
	if (Screen.size() < 2)
	{
		return;
	}
	auto const mid{Screen[Screen.size() / 2]};
	auto const i{static_cast<int>(Screen.size() / 2)};
	auto const a{Screen[std::max(0, i - 1)]};
	auto const c{Screen[std::min(static_cast<int>(Screen.size()) - 1, i + 1)]};
	auto ox{-(c.y - a.y)};
	auto oy{c.x - a.x};
	auto const len{std::hypot(ox, oy)};
	if (len > 1e-3f)
	{
		ox /= len;
		oy /= len;
	}
	else
	{
		ox = 0.0f;
		oy = -1.0f;
	}
	constexpr float kLead = 22.0f;
	ImVec2 const tip{mid.x + ox * kLead, mid.y + oy * kLead};
	Drawlist->AddLine(mid, tip, Colour, 1.2f);
	draw_label(Drawlist, tip, Text, Colour);
}

// short perpendicular tick across the track — marks an arc end / radius break
void draw_station_tick(ImDrawList *Drawlist, ImVec2 const &AlongA, ImVec2 const &AlongB, ImU32 const Colour, float const Half = 8.0f)
{
	auto tx{AlongB.x - AlongA.x};
	auto ty{AlongB.y - AlongA.y};
	auto const tl{std::hypot(tx, ty)};
	if (tl < 1e-3f)
	{
		return;
	}
	tx /= tl;
	ty /= tl;
	ImVec2 const mid{(AlongA.x + AlongB.x) * 0.5f, (AlongA.y + AlongB.y) * 0.5f};
	ImVec2 const px{-ty * Half, tx * Half};
	Drawlist->AddLine({mid.x - px.x, mid.y - px.y}, {mid.x + px.x, mid.y + px.y}, Colour, 2.0f);
}

void draw_arc_end_tick(ImDrawList *Drawlist, std::vector<ImVec2> const &Screen, bool const AtStart, ImU32 const Colour)
{
	if (Screen.size() < 2)
	{
		return;
	}
	if (AtStart)
	{
		draw_station_tick(Drawlist, Screen[0], Screen[1], Colour);
	}
	else
	{
		draw_station_tick(Drawlist, Screen[Screen.size() - 2], Screen[Screen.size() - 1], Colour);
	}
}

double point_segment_distance(double const Px, double const Py, double const Ax, double const Ay, double const Bx, double const By)
{
	auto const dx{Bx - Ax};
	auto const dy{By - Ay};
	auto const len2{dx * dx + dy * dy};
	if (len2 < 1e-18)
	{
		return std::hypot(Px - Ax, Py - Ay);
	}
	auto const t{std::clamp(((Px - Ax) * dx + (Py - Ay) * dy) / len2, 0.0, 1.0)};
	return std::hypot(Px - (Ax + t * dx), Py - (Ay + t * dy));
}

// distance from (X,Y) to the gap track: curve elements plus the two bounding straights
double distance_to_gap_track(maj0sted::web::NiweletaPolys const &Solved, int const Gap, double const X, double const Y)
{
	auto best{std::numeric_limits<double>::infinity()};
	for (auto const &poly : Solved.polylines)
	{
		auto const on_curve{poly.gap == Gap};
		auto const on_straight{poly.straight_index == Gap || poly.straight_index == Gap + 1};
		if ((false == on_curve && false == on_straight) || poly.points.size() < 2)
		{
			continue;
		}
		for (std::size_t i = 1; i < poly.points.size(); ++i)
		{
			best = std::min(best, point_segment_distance(X, Y, poly.points[i - 1].x, poly.points[i - 1].y, poly.points[i].x, poly.points[i].y));
		}
	}
	return best;
}

struct GapSample
{
	double x{0.0};
	double y{0.0};
	double along{0.0};
};

// centreline of a fitted gap, sampled in element order
std::vector<GapSample> sample_gap_centreline(maj0sted::web::NiweletaPolys const &Solved, int const Gap)
{
	std::map<int, maj0sted::web::WebPolyline const *> by_el;
	for (auto const &poly : Solved.polylines)
	{
		if (poly.gap != Gap || poly.element_index < 0 || poly.points.size() < 2)
		{
			continue;
		}
		if (by_el.find(poly.element_index) == by_el.end())
		{
			by_el.emplace(poly.element_index, &poly);
		}
	}
	std::vector<GapSample> out;
	double along{0.0};
	for (auto const &[idx, poly] : by_el)
	{
		(void)idx;
		for (std::size_t i = 0; i < poly->points.size(); ++i)
		{
			auto const &p{poly->points[i]};
			if (false == out.empty())
			{
				auto const &prev{out.back()};
				if (std::hypot(p.x - prev.x, p.y - prev.y) < 1e-9)
				{
					continue;
				}
				along += std::hypot(p.x - prev.x, p.y - prev.y);
			}
			out.push_back({p.x, p.y, along});
		}
	}
	return out;
}

bool project_on_gap(std::vector<GapSample> const &Samples, double const X, double const Y, double &OutStation, double &OutX, double &OutY)
{
	if (Samples.size() < 2)
	{
		return false;
	}
	auto best_d{std::numeric_limits<double>::infinity()};
	double best_s{0.0};
	double best_x{Samples.front().x};
	double best_y{Samples.front().y};
	for (std::size_t i = 1; i < Samples.size(); ++i)
	{
		auto const &a{Samples[i - 1]};
		auto const &b{Samples[i]};
		auto const dx{b.x - a.x};
		auto const dy{b.y - a.y};
		auto const len2{dx * dx + dy * dy};
		auto t{0.0};
		if (len2 > 1e-18)
		{
			t = std::clamp(((X - a.x) * dx + (Y - a.y) * dy) / len2, 0.0, 1.0);
		}
		auto const px{a.x + t * dx};
		auto const py{a.y + t * dy};
		auto const d{std::hypot(X - px, Y - py)};
		if (d < best_d)
		{
			best_d = d;
			best_s = a.along + t * (b.along - a.along);
			best_x = px;
			best_y = py;
		}
	}
	OutStation = best_s;
	OutX = best_x;
	OutY = best_y;
	return std::isfinite(best_d);
}

bool point_at_station(std::vector<GapSample> const &Samples, double Station, double &OutX, double &OutY)
{
	if (Samples.empty())
	{
		return false;
	}
	if (Station <= Samples.front().along)
	{
		OutX = Samples.front().x;
		OutY = Samples.front().y;
		return true;
	}
	if (Station >= Samples.back().along)
	{
		OutX = Samples.back().x;
		OutY = Samples.back().y;
		return true;
	}
	for (std::size_t i = 1; i < Samples.size(); ++i)
	{
		auto const &a{Samples[i - 1]};
		auto const &b{Samples[i]};
		if (Station > b.along)
		{
			continue;
		}
		auto const span{std::max(1e-12, b.along - a.along)};
		auto const t{(Station - a.along) / span};
		OutX = a.x + (b.x - a.x) * t;
		OutY = a.y + (b.y - a.y) * t;
		return true;
	}
	return false;
}

// kind: 0 entry KP end, 1 free-arc end, 2 between-KP end, 3 exit KP start
struct CmpStationHandle
{
	int gap{0};
	int kind{0};
	int arc{0};
	double x{0.0};
	double y{0.0};
};

std::vector<CmpStationHandle> compound_station_handles(maj0sted::web::GapFit const &Fit, maj0sted::web::NiweletaPolys const &Solved, int const Gap)
{
	std::vector<CmpStationHandle> handles;
	if (Fit.mode != 3 || Fit.arcs.empty())
	{
		return handles;
	}
	auto const samples{sample_gap_centreline(Solved, Gap)};
	if (samples.size() < 2)
	{
		return handles;
	}
	auto const total{samples.back().along};

	auto place = [&](int const Kind, int const Arc, double const Station) {
		double x{0.0};
		double y{0.0};
		if (point_at_station(samples, std::clamp(Station, 0.0, total), x, y))
		{
			handles.push_back({Gap, Kind, Arc, x, y});
		}
	};

	auto s{0.0};
	place(0, 0, std::max(0.0, Fit.entry_t)); // entry KP end (at 0 when no entry KP yet)
	s = std::max(0.0, Fit.entry_t);
	for (std::size_t i = 0; i + 1 < Fit.arcs.size(); ++i)
	{
		s += std::max(0.0, Fit.arcs[i].length);
		place(1, static_cast<int>(i), s); // free circular-arc end
		auto const kp{std::max(0.0, Fit.arcs[i].transition_to_next)};
		if (kp > 1e-6)
		{
			s += kp;
			place(2, static_cast<int>(i), s); // between-KP end
		}
	}
	// exit KP start: measured from the end of the solved curve
	place(3, 0, std::max(0.0, total - std::max(0.0, Fit.exit_t)));
	return handles;
}

std::vector<GapSample> sample_draft_centreline(std::vector<maj0sted::web::WebPolyline> const &Polys)
{
	std::vector<GapSample> out;
	double along{0.0};
	for (auto const &poly : Polys)
	{
		for (auto const &p : poly.points)
		{
			if (false == out.empty())
			{
				auto const &prev{out.back()};
				if (std::hypot(p.x - prev.x, p.y - prev.y) < 1e-9)
				{
					continue;
				}
				along += std::hypot(p.x - prev.x, p.y - prev.y);
			}
			out.push_back({p.x, p.y, along});
		}
	}
	return out;
}

// draft stations: every arc has L (including the last), unlike GapFit
std::vector<CmpStationHandle> draft_station_handles(maj0sted::app::BasketDraft const &Draft, std::vector<maj0sted::web::WebPolyline> const &Polys)
{
	std::vector<CmpStationHandle> handles;
	if (false == Draft.active || Draft.arcs.empty())
	{
		return handles;
	}
	auto const samples{sample_draft_centreline(Polys)};
	if (samples.size() < 2)
	{
		return handles;
	}
	auto const total{samples.back().along};
	auto place = [&](int const Kind, int const Arc, double const Station) {
		double x{0.0};
		double y{0.0};
		if (point_at_station(samples, std::clamp(Station, 0.0, total), x, y))
		{
			handles.push_back({-1, Kind, Arc, x, y});
		}
	};
	auto s{std::max(0.0, Draft.entry_t)};
	place(0, 0, s);
	for (std::size_t i = 0; i < Draft.arcs.size(); ++i)
	{
		s += std::max(0.0, Draft.arcs[i].length);
		place(1, static_cast<int>(i), s);
		auto const kp{std::max(0.0, Draft.arcs[i].transition_to_next)};
		if (i + 1 < Draft.arcs.size() && kp > 1e-6)
		{
			s += kp;
			place(2, static_cast<int>(i), s);
		}
	}
	place(3, 0, std::max(0.0, total - std::max(0.0, Draft.exit_t)));
	return handles;
}

} // namespace

plan_panel::plan_panel(std::string const &Name, bool const Isopen) : ui_panel(Name, Isopen)
{
	size_min = {420, 240};
	size_max = {900, 900};
	m_document.niwelety.push_back({"niweleta 1", {}, {}});
	std::snprintf(m_namebuf, sizeof(m_namebuf), "%s", m_document.niwelety.front().name.c_str());
	solve();
}

void plan_panel::update()
{
	// the plan belongs on the scenery itself, so opening the tool puts the viewport into the
	// distortion-free view from above and closing it hands the camera back
	Global.editor_ortho = is_open;
}

void plan_panel::render_contents()
{
	handle_scene();
	draw_on_scene();

	render_toolbar();
	render_draft();
	render_switches();
	render_gaps();
	render_storage();
	render_newmap_dialog();
	render_location_dialog();
}

void plan_panel::render_newmap_dialog()
{
	if (false == ImGui::BeginPopupModal("New map", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
	{
		return;
	}

	ImGui::TextUnformatted("What is this scenery going to be?");
	ImGui::Spacing();

	if (ImGui::Button("Fictional", ImVec2(160.0f, 0.0f)))
	{
		start_map(false, 0.0, 0.0);
		ImGui::CloseCurrentPopup();
	}
	ImGui::SameLine();
	ImGui::TextDisabled("coordinates mean nothing outside the scenery");

	if (ImGui::Button("Real place", ImVec2(160.0f, 0.0f)))
	{
		// start the picker on the middle of the country, showing the whole of it
		m_pickingplace = true;
		m_pickx = 500000.0;
		m_picky = 350000.0;
		m_mapviewx = m_pickx;
		m_mapviewy = m_picky;
		m_mapscale = 0.0;
		ImGui::CloseCurrentPopup();
		ImGui::OpenPopup("Scenery location");
	}
	ImGui::SameLine();
	ImGui::TextDisabled("pin the scenery's zero to a point in Poland");

	ImGui::Spacing();
	if (ImGui::Button("Cancel"))
	{
		ImGui::CloseCurrentPopup();
	}

	ImGui::EndPopup();
}

void plan_panel::render_location_dialog()
{
	if (m_pickingplace)
	{
		// the popup has to be opened from the same stack level it is drawn on
		ImGui::OpenPopup("Scenery location");
		m_pickingplace = false;
	}

	if (false == ImGui::BeginPopupModal("Scenery location", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
	{
		return;
	}

	// the request is one square image, so the view is square too and the pixels map one to one
	int const mappixels = 512;
	ImVec2 const mapsize{static_cast<float>(mappixels), static_cast<float>(mappixels)};
	auto const origin{ImGui::GetCursorScreenPos()};
	ImGui::InvisibleButton("poland_map", mapsize);
	auto const hovered{ImGui::IsItemHovered()};

	// fit the whole country the first time round; afterwards the wheel decides
	if (m_mapscale <= 0.0)
	{
		glm::dvec2 min;
		glm::dvec2 max;
		editor::poland_extent(min, max);
		m_mapviewx = (min.x + max.x) * 0.5;
		m_mapviewy = (min.y + max.y) * 0.5;
		m_mapscale = mapsize.x / std::max({max.x - min.x, max.y - min.y, 1.0});
	}

	ImVec2 const centre{origin.x + mapsize.x * 0.5f, origin.y + mapsize.y * 0.5f};
	auto const to_screen = [&](double const X, double const Y) {
		return ImVec2{centre.x + static_cast<float>((X - m_mapviewx) * m_mapscale), centre.y - static_cast<float>((Y - m_mapviewy) * m_mapscale)};
	};
	auto const to_map = [&](ImVec2 const &Point, double &X, double &Y) {
		X = m_mapviewx + (Point.x - centre.x) / m_mapscale;
		Y = m_mapviewy - (Point.y - centre.y) / m_mapscale;
	};

	auto const mouse{ImGui::GetIO().MousePos};
	if (hovered && ImGui::GetIO().MouseWheel != 0.0f)
	{
		double anchorx{0.0};
		double anchory{0.0};
		to_map(mouse, anchorx, anchory);
		m_mapscale = std::clamp(m_mapscale * std::pow(1.2, ImGui::GetIO().MouseWheel), 1e-5, 1.0);
		m_mapviewx = anchorx - (mouse.x - centre.x) / m_mapscale;
		m_mapviewy = anchory + (mouse.y - centre.y) / m_mapscale;
	}
	if (hovered && ImGui::IsMouseClicked(0))
	{
		to_map(mouse, m_pickx, m_picky);
	}

	auto *drawlist{ImGui::GetWindowDrawList()};
	auto const mapend{ImVec2(origin.x + mapsize.x, origin.y + mapsize.y)};
	drawlist->PushClipRect(origin, mapend, true);
	drawlist->AddRectFilled(origin, mapend, IM_COL32(22, 30, 38, 255));

	// the topographic base map behind everything, asked for at exactly the box on screen
	double viewminx{0.0};
	double viewminy{0.0};
	double viewmaxx{0.0};
	double viewmaxy{0.0};
	to_map(origin, viewminx, viewmaxy);
	to_map(mapend, viewmaxx, viewminy);
	auto const backdrop{m_topomap.texture_for({viewminx, viewminy, viewmaxx, viewmaxy}, mappixels)};
	if (backdrop != 0)
	{
		// the picture on screen is the one that was fetched, which may lag the current view by a
		// request; drawing it against its own box keeps it registered while the new one arrives
		// the image arrives with its first row at the north edge; the vertical texture coordinates are
		// swapped to put that row against the box's northern edge. the orthophoto is handed the same
		// swap - both are placed north-west corner first, so they cannot want different conventions
		auto const &covered{m_topomap.covered()};
		drawlist->AddImage(reinterpret_cast<ImTextureID>(static_cast<intptr_t>(backdrop)), to_screen(covered.min_x, covered.max_y), to_screen(covered.max_x, covered.min_y), ImVec2(0.0f, 1.0f),
		                   ImVec2(1.0f, 0.0f));
	}

	auto const picked{to_screen(m_pickx, m_picky)};
	drawlist->AddCircleFilled(picked, 5.0f, IM_COL32(255, 210, 90, 255));
	drawlist->AddLine(ImVec2(picked.x - 12.0f, picked.y), ImVec2(picked.x + 12.0f, picked.y), IM_COL32(255, 210, 90, 200));
	drawlist->AddLine(ImVec2(picked.x, picked.y - 12.0f), ImVec2(picked.x, picked.y + 12.0f), IM_COL32(255, 210, 90, 200));

	drawlist->PopClipRect();

	ImGui::TextDisabled(m_topomap.loading() ? "fetching the topographic map..." : "click to place the scenery's zero, wheel zooms");

	ImGui::SetNextItemWidth(180.0f);
	ImGui::InputDouble("easting", &m_pickx, 100.0, 1000.0, "%.0f");
	ImGui::SameLine();
	ImGui::SetNextItemWidth(180.0f);
	ImGui::InputDouble("northing", &m_picky, 100.0, 1000.0, "%.0f");

	ImGui::Spacing();
	if (ImGui::Button("Use this place", ImVec2(160.0f, 0.0f)))
	{
		start_map(true, m_pickx, m_picky);
		ImGui::CloseCurrentPopup();
	}
	ImGui::SameLine();
	if (ImGui::Button("Cancel"))
	{
		ImGui::CloseCurrentPopup();
	}

	ImGui::EndPopup();
}

void plan_panel::start_map(bool const Georeferenced, double const Originx, double const Originy)
{
	Global.scenery_georeferenced = Georeferenced;
	Global.scenery_origin = {Originx, Originy};
	m_document.origin_set = true;
	m_document.georeferenced = Georeferenced;
	m_document.origin_x = Originx;
	m_document.origin_y = Originy;

	m_document.niwelety.clear();
	m_document.niwelety.push_back({"niweleta 1", {}, {}});
	m_niweleta = 0;
	m_pending = false;
	clear_selection();
	m_pick_rel = 0;
	m_ask_rel_param = false;
	m_sel_gap = -1;
	m_pick_compound_point = false;
	m_fit_guides.clear();
	solve();

	// the scenery's zero is where the work starts, whichever frame it stands for
	auto &camera{editor_mode::get_camera()};
	camera.Pos.x = 0.0;
	camera.Pos.z = 0.0;

	// a new map starts on empty ground - whatever scenery was loaded has nothing to do with it
	Global.editor_reset_scenery = true;

	m_status = Georeferenced ? "map pinned at " + to_string(Originx, 0) + ", " + to_string(Originy, 0) + " (EPSG:2180)" : "fictional map";
}

void plan_panel::handle_scene()
{
	// clicks that land on a panel belong to the panel, not to the ground under it
	if (ImGui::GetIO().WantCaptureMouse)
	{
		m_dragging = false;
		m_dragging_cmp = false;
		m_dragging_draft = false;
		return;
	}

	double cursorx{0.0};
	double cursory{0.0};
	// the plan view must not lean on the depth buffer: on empty ground it has nothing to read
	world_to_plan(Global.editor_ortho ? ortho_cursor_world() : Global.pCamera.Pos + GfxRenderer->Mouse_Position(), cursorx, cursory);

	auto const mouse{ImGui::GetIO().MousePos};

	if (ImGui::IsMouseDoubleClicked(0) && m_draft_place == 0 && m_anchor_pick == 0 && m_pick_rel == 0 && false == m_pick_compound_point)
	{
		int hit_niw{-1};
		int hit_str{-1};
		if (hit_straight(mouse, hit_niw, hit_str, true) && hit_niw >= 0 && static_cast<std::size_t>(hit_niw) < m_document.niwelety.size())
		{
			m_dragging = false;
			m_dragging_cmp = false;
			m_dragging_draft = false;
			m_niweleta = static_cast<std::size_t>(hit_niw);
			m_sel_straight = hit_str;
			m_sel_end = -1;
			m_pending = false;
			m_draft.active = false;
			std::snprintf(m_namebuf, sizeof(m_namebuf), "%s", m_document.niwelety[m_niweleta].name.c_str());
			m_status = m_document.niwelety[m_niweleta].name;
			return;
		}
	}

	if (ImGui::IsMouseClicked(0))
	{
		m_dragging = false;

		if (m_draft_place == 1)
		{
			int hit_niw{-1};
			int hit_str{-1};
			int hit_end{-1};
			if (hit_endpoint(mouse, hit_niw, hit_str, hit_end))
			{
				m_niweleta = static_cast<std::size_t>(hit_niw);
				start_draft_from_end(static_cast<std::size_t>(hit_str), hit_end);
			}
			else
			{
				m_status = "kliknij koniec prostej";
			}
			return;
		}

		if (m_anchor_pick != 0)
		{
			int hit_niw{-1};
			int hit_str{-1};
			if (hit_straight(mouse, hit_niw, hit_str, false))
			{
				auto *niweleta{current_niweleta()};
				if (niweleta != nullptr && niweleta->straights.size() >= 2)
				{
					auto gap{static_cast<std::size_t>(hit_str)};
					if (gap + 1 >= niweleta->straights.size())
					{
						gap = niweleta->straights.size() - 2;
					}
					m_anchor_pick = 0;
					anchor_draft_to_gap(gap);
					return;
				}
			}
			m_status = "kliknij prostą przy luce";
			return;
		}

		if (m_pick_compound_point)
		{
			m_fit_guides.push_back({cursorx, cursory});
			m_status = std::to_string(m_fit_guides.size()) + " pts";
			return;
		}

		// dropping a switch on a through niweleta
		if (m_pick_switch != 0)
		{
			int hit_niw{-1};
			int hit_str{-1};
			if (hit_straight(mouse, hit_niw, hit_str, true) && hit_niw >= 0 && static_cast<std::size_t>(hit_niw) < m_solved.size())
			{
				add_switch_on(static_cast<std::size_t>(hit_niw), cursorx, cursory);
				m_pick_switch = 0;
			}
			else
			{
				m_status = "kliknij niweletę na wprost";
			}
			return;
		}

		// picking a reference straight for a parallel/skew constraint
		if (m_pick_rel != 0)
		{
			int hit_niw{-1};
			int hit_str{-1};
			if (hit_straight(mouse, hit_niw, hit_str, true) && m_sel_straight >= 0 && m_niweleta < m_document.niwelety.size() &&
			    static_cast<std::size_t>(m_sel_straight) < m_document.niwelety[m_niweleta].straights.size())
			{
				auto &dep{m_document.niwelety[m_niweleta].straights[static_cast<std::size_t>(m_sel_straight)]};
				if (!(hit_niw == static_cast<int>(m_niweleta) && hit_str == m_sel_straight))
				{
					dep.rel_kind = m_pick_rel;
					dep.rel_niw = hit_niw;
					dep.rel_str = hit_str;
					if (m_pick_rel == 1)
					{
						dep.rel_offset = m_ask_rel_value;
						m_status = "parallel → n" + std::to_string(hit_niw + 1) + " s" + std::to_string(hit_str);
					}
					else
					{
						auto const dx{dep.x2 - dep.x1};
						auto const dy{dep.y2 - dep.y1};
						dep.rel_length = std::max(1.0, std::hypot(dx, dy));
						dep.rel_cot = std::abs(m_ask_rel_value);
						dep.rel_side = m_ask_rel_value < 0.0 ? -1 : 1;
						m_status = "skew 1:" + to_string(dep.rel_cot, 0) + " → n" + std::to_string(hit_niw + 1) + " s" + std::to_string(hit_str);
					}
					solve();
				}
			}
			m_pick_rel = 0;
			return;
		}

		int draft_kind{-1};
		int draft_arc{-1};
		if (m_draft.active && hit_draft_station(mouse, draft_kind, draft_arc))
		{
			clear_selection();
			m_dragging_draft = true;
			m_draft_kind = draft_kind;
			m_draft_arc = draft_arc;
			m_drawing = false;
			m_status = draft_kind == 0   ? "KP wejście"
			           : draft_kind == 1 ? "L łuku"
			           : draft_kind == 2 ? "KP między"
			                             : "KP wyjście";
			return;
		}

		int cmp_gap{-1};
		int cmp_kind{-1};
		int cmp_arc{-1};
		if (hit_compound_station(mouse, cmp_gap, cmp_kind, cmp_arc))
		{
			clear_selection();
			m_dragging_cmp = true;
			m_cmp_gap = cmp_gap;
			m_cmp_kind = cmp_kind;
			m_cmp_arc = cmp_arc;
			m_sel_gap = cmp_gap;
			m_drawing = false;
			m_status = cmp_kind == 0   ? "KP wejście"
			           : cmp_kind == 1 ? "L łuku"
			           : cmp_kind == 2 ? "KP między"
			                           : "KP wyjście";
			return;
		}

		int hit_niw{-1};
		int hit_str{-1};
		int hit_end{-1};
		if (hit_endpoint(mouse, hit_niw, hit_str, hit_end))
		{
			m_dragging_cmp = false;
			m_dragging_draft = false;
			m_sel_straight = hit_str;
			m_sel_end = hit_end;
			m_dragging = true;
			auto *niweleta{current_niweleta()};
			if (niweleta != nullptr && static_cast<std::size_t>(hit_str) < niweleta->straights.size())
			{
				auto const &raw{niweleta->straights[static_cast<std::size_t>(hit_str)]};
				m_drag_fixed_x = hit_end == 0 ? raw.x2 : raw.x1;
				m_drag_fixed_y = hit_end == 0 ? raw.y2 : raw.y1;
				auto const gx{hit_end == 0 ? raw.x1 : raw.x2};
				auto const gy{hit_end == 0 ? raw.y1 : raw.y2};
				auto const dl{std::max(1e-9, std::hypot(gx - m_drag_fixed_x, gy - m_drag_fixed_y))};
				m_drag_ux = (gx - m_drag_fixed_x) / dl;
				m_drag_uy = (gy - m_drag_fixed_y) / dl;
			}
		}
		else if (hit_straight(mouse, hit_niw, hit_str, false))
		{
			m_dragging_cmp = false;
			m_dragging_draft = false;
			m_sel_straight = hit_str;
			m_sel_end = -1;
		}
		else if (m_drawing && false == m_draft.active && m_draft_place == 0 && m_anchor_pick == 0)
		{
			clear_selection();
			append_vertex(cursorx, cursory);
		}
		else
		{
			clear_selection();
		}
	}

	if (m_dragging_draft)
	{
		if (ImGui::IsMouseDown(0))
		{
			drag_draft_station(cursorx, cursory);
		}
		else
		{
			m_dragging_draft = false;
		}
	}
	else if (m_dragging_cmp)
	{
		if (ImGui::IsMouseDown(0))
		{
			drag_compound_station(cursorx, cursory);
		}
		else
		{
			m_dragging_cmp = false;
		}
	}
	else if (m_dragging && m_sel_straight >= 0 && m_sel_end >= 0)
	{
		if (ImGui::IsMouseDown(0))
		{
			auto *niweleta{current_niweleta()};
			if (niweleta != nullptr && static_cast<std::size_t>(m_sel_straight) < niweleta->straights.size())
			{
				auto &st{niweleta->straights[static_cast<std::size_t>(m_sel_straight)]};
				double x{cursorx};
				double y{cursory};
				// Shift: slide along the straight's axis — length changes, angle stays
				if (ImGui::GetIO().KeyShift)
				{
					auto const t{std::max(1.0, (x - m_drag_fixed_x) * m_drag_ux + (y - m_drag_fixed_y) * m_drag_uy)};
					x = m_drag_fixed_x + m_drag_ux * t;
					y = m_drag_fixed_y + m_drag_uy * t;
				}
				if (m_sel_end == 0)
				{
					st.x1 = x;
					st.y1 = y;
				}
				else
				{
					st.x2 = x;
					st.y2 = y;
				}
				if (st.rel_kind == 2)
				{
					st.rel_length = std::max(1.0, std::hypot(st.x2 - st.x1, st.y2 - st.y1));
				}
				solve();
			}
		}
		else
		{
			m_dragging = false;
		}
	}
}

void plan_panel::draw_on_scene()
{
	// the background list paints over the rendered scenery but under every window, so the controls
	// stay readable on top of the drawing
	auto *drawlist{ImGui::GetBackgroundDrawList()};

	// orthophoto under everything else. only a georeferenced map knows where on the ground it sits,
	// so a fictional one gets no imagery
	if (m_showortho && Global.scenery_georeferenced)
	{
		double planx{0.0};
		double plany{0.0};
		world_to_plan(Global.pCamera.Pos, planx, plany);

		// yaw is pinned north-up, so the screen is an axis-aligned box in plan space
		auto const display{ImGui::GetIO().DisplaySize};
		auto const halfheight{static_cast<double>(Global.editor_ortho_extent)};
		auto const halfwidth{halfheight * std::max(1.0f, display.x) / std::max(1.0f, display.y)};
		maj0sted::web::TileBBox const view{planx - halfwidth, plany - halfheight, planx + halfwidth, plany + halfheight};

		auto const draw_box = [&](maj0sted::web::TileBBox const &Box, unsigned int const Texture) {
			ImVec2 corners[4];
			auto const ok = world_to_screen(plan_to_world(Box.min_x, Box.max_y), corners[0]) && world_to_screen(plan_to_world(Box.max_x, Box.max_y), corners[1]) &&
			                world_to_screen(plan_to_world(Box.max_x, Box.min_y), corners[2]) && world_to_screen(plan_to_world(Box.min_x, Box.min_y), corners[3]);
			if (false == ok)
			{
				return;
			}
			// north-west corner first; the texture's first row is its northern edge
			drawlist->AddImageQuad(reinterpret_cast<ImTextureID>(static_cast<intptr_t>(Texture)), corners[0], corners[1], corners[2], corners[3], ImVec2(0.0f, 1.0f), ImVec2(1.0f, 1.0f),
			                       ImVec2(1.0f, 0.0f), ImVec2(0.0f, 0.0f));
		};

		// once the view is wider than a kilometre the 100 m orto cells are the wrong scale:
		// switch to a 1 km topo grid that can cover the whole screen
		constexpr double kTopoPreviewMetres = 1000.0;
		auto const span{std::max(halfwidth, halfheight) * 2.0};
		if (span > kTopoPreviewMetres)
		{
			for (auto const &tile : m_topo.collect(view, 80))
			{
				draw_box(tile.box, tile.texture);
			}
		}
		else
		{
			for (auto const &tile : m_ortho.collect(view))
			{
				draw_box(tile.box, tile.texture);
			}
		}
	}

	std::vector<ImVec2> points;
	for (std::size_t n = 0; n < m_solved.size(); ++n)
	{
		auto const active{n == m_niweleta};
		for (auto const &polyline : m_solved[n].polylines)
		{
			if (polyline.points.size() < 2)
			{
				continue;
			}
			points.clear();
			points.reserve(polyline.points.size());
			for (auto const &point : polyline.points)
			{
				ImVec2 screen;
				if (false == world_to_screen(plan_to_world(point.x, point.y), screen))
				{
					continue;
				}
				points.push_back(screen);
			}
			if (points.size() < 2)
			{
				continue;
			}
			drawlist->AddPolyline(points.data(), static_cast<int>(points.size()), element_colour(polyline.kind, active), false, 2.5f);
		}
	}

	// switches: the diverging curve in its own colour, with a marker at the switch point and the frog
	for (auto const &junction : m_junctions)
	{
		if (false == junction.valid)
		{
			continue;
		}
		for (auto const &polyline : junction.polylines)
		{
			if (polyline.points.size() < 2)
			{
				continue;
			}
			points.clear();
			for (auto const &point : polyline.points)
			{
				ImVec2 screen;
				if (world_to_screen(plan_to_world(point.x, point.y), screen))
				{
					points.push_back(screen);
				}
			}
			if (points.size() >= 2)
			{
				drawlist->AddPolyline(points.data(), static_cast<int>(points.size()), IM_COL32(255, 150, 40, 255), false, 2.5f);
			}
		}
		ImVec2 marker;
		if (world_to_screen(plan_to_world(junction.px, junction.py), marker))
		{
			drawlist->AddCircleFilled(marker, 4.0f, IM_COL32(255, 210, 90, 255));
		}
		if (world_to_screen(plan_to_world(junction.fx, junction.fy), marker))
		{
			drawlist->AddCircleFilled(marker, 4.0f, IM_COL32(255, 150, 40, 255));
		}
	}

	// arc / KP annotations: R= and KP labels, plus station ticks at each circular-arc end
	// (and a stronger tick where consecutive arcs of a compound meet with different R)
	if (m_niweleta < m_solved.size())
	{
		auto const &solved{m_solved[m_niweleta]};
		std::set<int> gaps;
		for (auto const &poly : solved.polylines)
		{
			if (poly.gap >= 0)
			{
				gaps.insert(poly.gap);
			}
		}
		for (auto const gap : gaps)
		{
			std::map<int, maj0sted::web::WebPolyline const *> by_el;
			for (auto const &poly : solved.polylines)
			{
				if (poly.gap != gap || poly.points.size() < 2 || poly.element_index < 0)
				{
					continue;
				}
				if (by_el.find(poly.element_index) == by_el.end())
				{
					by_el.emplace(poly.element_index, &poly);
				}
			}

			std::vector<maj0sted::web::WebPolyline const *> arcs;
			for (auto const &[idx, poly] : by_el)
			{
				(void)idx;
				std::vector<ImVec2> screen;
				screen.reserve(poly->points.size());
				for (auto const &point : poly->points)
				{
					ImVec2 s;
					if (world_to_screen(plan_to_world(point.x, point.y), s))
					{
						screen.push_back(s);
					}
				}
				if (poly->kind == 1)
				{
					arcs.push_back(poly);
					auto const r{poly->radius_start > 0.0 ? poly->radius_start : poly->radius_end};
					if (r > 0.0)
					{
						char buf[48];
						std::snprintf(buf, sizeof(buf), "R = %.1f m", r);
						draw_curve_callout(drawlist, screen, buf, IM_COL32(255, 209, 102, 255));
					}
					// ends of this circular arc
					draw_arc_end_tick(drawlist, screen, true, IM_COL32(255, 100, 100, 230));
					draw_arc_end_tick(drawlist, screen, false, IM_COL32(255, 100, 100, 230));
				}
				else if (poly->kind == 2 && poly->length > 0.0)
				{
					char buf[48];
					std::snprintf(buf, sizeof(buf), "KP L = %.1f m", poly->length);
					draw_curve_callout(drawlist, screen, buf, IM_COL32(140, 245, 150, 255));
				}
			}

			// compound: thicker tick where two consecutive arcs meet (radius break)
			for (std::size_t i = 0; i + 1 < arcs.size(); ++i)
			{
				auto const r1{arcs[i]->radius_start > 0.0 ? arcs[i]->radius_start : arcs[i]->radius_end};
				auto const r2{arcs[i + 1]->radius_start > 0.0 ? arcs[i + 1]->radius_start : arcs[i + 1]->radius_end};
				if (std::abs(r1 - r2) < 1e-6)
				{
					continue;
				}
				auto const &a{arcs[i]->points.back()};
				auto const &b{arcs[i + 1]->points.front()};
				ImVec2 sa;
				ImVec2 sb;
				if (world_to_screen(plan_to_world(a.x, a.y), sa) && world_to_screen(plan_to_world(b.x, b.y), sb))
				{
					draw_station_tick(drawlist, sa, sb, IM_COL32(255, 80, 80, 255), 10.0f);
				}
			}
		}
	}

	// independent endpoints of the active niweleta (prefer solved/trimmed positions)
	if (auto const *niweleta{current_niweleta()}; niweleta != nullptr)
	{
		for (std::size_t i = 0; i < niweleta->straights.size(); ++i)
		{
			auto const *st{handle_straight(m_niweleta, i)};
			if (st == nullptr || st->hidden)
			{
				continue;
			}
			auto const selected{static_cast<int>(i) == m_sel_straight};
			for (int end = 0; end < 2; ++end)
			{
				auto const x{end == 0 ? st->x1 : st->x2};
				auto const y{end == 0 ? st->y1 : st->y2};
				ImVec2 screen;
				if (false == world_to_screen(plan_to_world(x, y), screen))
				{
					continue;
				}
				auto const hot{selected && end == m_sel_end};
				drawlist->AddCircleFilled(screen, hot ? 6.0f : 5.0f, hot ? IM_COL32(255, 240, 120, 255) : selected ? IM_COL32(255, 220, 140, 220) : IM_COL32(240, 240, 240, 255));
			}
			if (selected)
			{
				ImVec2 a;
				ImVec2 b;
				if (world_to_screen(plan_to_world(st->x1, st->y1), a) && world_to_screen(plan_to_world(st->x2, st->y2), b))
				{
					drawlist->AddLine(a, b, IM_COL32(255, 240, 120, 200), 3.0f);
				}
			}
		}
	}

	// the corner still waiting for the one that closes its straight, and the rubber band to the cursor
	auto const mouse{ImGui::GetIO().MousePos};
	auto const overscenery{false == ImGui::GetIO().WantCaptureMouse};
	if (m_draft_place == 1 && overscenery)
	{
		drawlist->AddCircleFilled(mouse, 5.0f, IM_COL32(255, 180, 90, 255));
	}
	else if (m_pending)
	{
		ImVec2 screen;
		if (world_to_screen(plan_to_world(m_pendingx, m_pendingy), screen))
		{
			drawlist->AddCircleFilled(screen, 5.0f, IM_COL32(255, 240, 120, 255));
			if (overscenery)
			{
				drawlist->AddLine(screen, mouse, IM_COL32(255, 240, 120, 170), 1.5f);
			}
		}
	}
	else if (overscenery && m_drawing && false == m_dragging && m_draft_place == 0 && m_anchor_pick == 0)
	{
		if (auto const *niweleta{current_niweleta()}; niweleta != nullptr && false == niweleta->straights.empty())
		{
			auto const &last{niweleta->straights.back()};
			ImVec2 screen;
			if (world_to_screen(plan_to_world(last.x2, last.y2), screen))
			{
				drawlist->AddLine(screen, mouse, IM_COL32(255, 240, 120, 170), 1.5f);
			}
		}
	}

	// basket draft: dual rails + same station handles as compound
	if (m_draft.active)
	{
		std::vector<ImVec2> pts;
		for (auto const &polyline : rails_from_centreline(m_draft_polys))
		{
			if (polyline.points.size() < 2)
			{
				continue;
			}
			pts.clear();
			pts.reserve(polyline.points.size());
			for (auto const &point : polyline.points)
			{
				ImVec2 screen;
				if (false == world_to_screen(plan_to_world(point.x, point.y), screen))
				{
					continue;
				}
				pts.push_back(screen);
			}
			if (pts.size() < 2)
			{
				continue;
			}
			drawlist->AddPolyline(pts.data(), static_cast<int>(pts.size()), element_colour(polyline.kind, true), false, 2.5f);
		}
		for (auto const &h : draft_station_handles(m_draft, m_draft_polys))
		{
			ImVec2 screen;
			if (false == world_to_screen(plan_to_world(h.x, h.y), screen))
			{
				continue;
			}
			auto const hot{m_dragging_draft && h.kind == m_draft_kind && h.arc == m_draft_arc};
			auto const kp{h.kind == 0 || h.kind == 2 || h.kind == 3};
			auto const fill{kp ? (hot ? IM_COL32(120, 255, 160, 255) : IM_COL32(90, 230, 140, 230))
			                   : (hot ? IM_COL32(255, 200, 90, 255) : IM_COL32(255, 160, 70, 230))};
			drawlist->AddCircleFilled(screen, hot ? 6.5f : 5.0f, fill);
			drawlist->AddCircle(screen, hot ? 6.5f : 5.0f, IM_COL32(245, 245, 245, 255));
		}
	}

	// draggable compound stations (KP / arc ends) on the active niweleta
	if (auto const *niweleta{current_niweleta()}; niweleta != nullptr && m_niweleta < m_solved.size())
	{
		for (auto const &fit : niweleta->fits)
		{
			if (fit.mode != 3 || fit.arcs.empty())
			{
				continue;
			}
			for (auto const &h : compound_station_handles(fit, m_solved[m_niweleta], fit.gap))
			{
				ImVec2 screen;
				if (false == world_to_screen(plan_to_world(h.x, h.y), screen))
				{
					continue;
				}
				auto const hot{m_dragging_cmp && h.gap == m_cmp_gap && h.kind == m_cmp_kind && h.arc == m_cmp_arc};
				auto const kp{h.kind == 0 || h.kind == 2 || h.kind == 3};
				auto const fill{kp ? (hot ? IM_COL32(120, 255, 160, 255) : IM_COL32(90, 230, 140, 230))
				                   : (hot ? IM_COL32(255, 200, 90, 255) : IM_COL32(255, 160, 70, 230))};
				drawlist->AddCircleFilled(screen, hot ? 6.5f : 5.0f, fill);
				drawlist->AddCircle(screen, hot ? 6.5f : 5.0f, IM_COL32(245, 245, 245, 255));
			}
		}
	}

	// guide points for multi-point compound fit
	for (std::size_t i = 0; i < m_fit_guides.size(); ++i)
	{
		auto const &g{m_fit_guides[i]};
		ImVec2 screen;
		if (false == world_to_screen(plan_to_world(g.x, g.y), screen))
		{
			continue;
		}
		drawlist->AddCircleFilled(screen, 4.5f, IM_COL32(80, 220, 255, 240));
		drawlist->AddCircle(screen, 4.5f, IM_COL32(230, 245, 255, 255));
		char buf[16];
		std::snprintf(buf, sizeof(buf), "%zu", i + 1);
		drawlist->AddText({screen.x + 6.0f, screen.y - 8.0f}, IM_COL32(160, 230, 255, 255), buf);
	}
	if (m_pick_compound_point && overscenery && false == m_fit_guides.empty())
	{
		ImVec2 prev;
		if (world_to_screen(plan_to_world(m_fit_guides.back().x, m_fit_guides.back().y), prev))
		{
			drawlist->AddLine(prev, mouse, IM_COL32(80, 220, 255, 140), 1.2f);
		}
	}
}

void plan_panel::render_draft()
{
	ImGui::Separator();
	if (m_draft_place == 1)
	{
		ImGui::Text("Kosz: kliknij koniec prostej");
		ImGui::SameLine();
		if (ImGui::SmallButton("Anuluj##draftplace"))
		{
			m_draft_place = 0;
			m_status.clear();
		}
		return;
	}

	if (false == m_draft.active)
	{
		if (ImGui::Button("Kosz"))
		{
			begin_draft();
		}
		return;
	}

	ImGui::AlignTextToFramePadding();
	ImGui::TextUnformatted("Kosz");
	ImGui::SameLine();
	bool const left{m_draft.dir >= 0};
	if (ImGui::RadioButton("Left##ddir", left))
	{
		m_draft.dir = 1;
		rebuild_draft_preview();
	}
	ImGui::SameLine();
	if (ImGui::RadioButton("Right##ddir", false == left))
	{
		m_draft.dir = -1;
		rebuild_draft_preview();
	}
	ImGui::SameLine();
	if (ImGui::SmallButton("Anuluj##draft"))
	{
		discard_draft();
		m_status.clear();
	}

	ImGui::Indent();
	ImGui::SetNextItemWidth(100.0f);
	if (ImGui::InputDouble("wejscie##dent", &m_draft.entry_t, 5.0, 20.0, "%.1f"))
	{
		rebuild_draft_preview();
	}
	ImGui::SameLine();
	ImGui::SetNextItemWidth(100.0f);
	if (ImGui::InputDouble("wyjscie##dex", &m_draft.exit_t, 5.0, 20.0, "%.1f"))
	{
		rebuild_draft_preview();
	}

	ImGui::Spacing();
	double station{m_draft.entry_t};
	for (std::size_t i = 0; i < m_draft.arcs.size(); ++i)
	{
		auto &arc{m_draft.arcs[i]};
		ImGui::PushID(static_cast<int>(i) + 9000);
		ImGui::AlignTextToFramePadding();
		ImGui::Text("Luk %d", static_cast<int>(i + 1));
		ImGui::SameLine();
		ImGui::SetNextItemWidth(90.0f);
		if (ImGui::InputDouble("R##dar", &arc.radius, 10.0, 100.0, "%.1f"))
		{
			rebuild_draft_preview();
		}
		ImGui::SameLine();
		ImGui::SetNextItemWidth(90.0f);
		if (ImGui::InputDouble("L##dal", &arc.length, 5.0, 20.0, "%.1f"))
		{
			rebuild_draft_preview();
		}
		ImGui::SameLine();
		ImGui::SetNextItemWidth(80.0f);
		if (ImGui::InputDouble("KP##dat", &arc.transition_to_next, 5.0, 20.0, "%.1f"))
		{
			rebuild_draft_preview();
		}
		station += std::max(0.0, arc.length) + std::max(0.0, arc.transition_to_next);
		ImGui::TextDisabled("~%.0f m", station);
		ImGui::PopID();
	}

	if (ImGui::SmallButton("Add arc##draft"))
	{
		auto const radius{m_draft.arcs.empty() ? 300.0 : std::max(30.0, m_draft.arcs.back().radius * 0.75)};
		if (false == m_draft.arcs.empty() && m_draft.arcs.back().transition_to_next < 1e-6)
		{
			m_draft.arcs.back().transition_to_next = 40.0;
		}
		m_draft.arcs.push_back({radius, 60.0, 0.0});
		m_draft_r = radius;
		m_draft.preview_next_r = radius;
		rebuild_draft_preview();
	}
	if (m_draft.arcs.size() > 1)
	{
		ImGui::SameLine();
		if (ImGui::SmallButton("Remove arc##draft"))
		{
			m_draft.arcs.pop_back();
			if (false == m_draft.arcs.empty())
			{
				m_draft.arcs.back().transition_to_next = 0.0;
			}
			rebuild_draft_preview();
		}
	}

	ImGui::Spacing();
	if (m_draft.reedit_gap >= 0)
	{
		if (ImGui::Button("Zatwierdź##dfin") && false == m_draft.arcs.empty())
		{
			finish_draft();
		}
	}
	else
	{
		ImGui::SetNextItemWidth(80.0f);
		ImGui::InputDouble("L prostej##dfin", &m_draft_finish_straight, 10.0, 50.0, "%.0f");
		if (m_draft_finish_straight < 20.0)
		{
			m_draft_finish_straight = 20.0;
		}
		ImGui::SameLine();
		if (ImGui::Button("Zatwierdź##dfin") && false == m_draft.arcs.empty())
		{
			finish_draft();
		}
	}
	ImGui::Unindent();
}

void plan_panel::render_toolbar()
{
	auto *niweleta{current_niweleta()};

	ImGui::Text("top-down view, %.0f m across", Global.editor_ortho_extent * 2.0f);
	ImGui::TextDisabled("Shift = długość, double-click = inna niweleta");
	if (Global.scenery_georeferenced)
	{
		ImGui::Text("zero at %.0f, %.0f (EPSG:2180)", Global.scenery_origin.x, Global.scenery_origin.y);
	}
	else
	{
		ImGui::TextDisabled("fictional map, no georeference");
	}
	ImGui::Separator();

	if (ImGui::Button("New map"))
	{
		ImGui::OpenPopup("New map");
	}
	ImGui::SameLine();
	if (ImGui::Button("New niweleta"))
	{
		m_document.niwelety.push_back({"niweleta " + std::to_string(m_document.niwelety.size() + 1), {}, {}});
		m_niweleta = m_document.niwelety.size() - 1;
		m_pending = false;
		clear_selection();
		std::snprintf(m_namebuf, sizeof(m_namebuf), "%s", m_document.niwelety[m_niweleta].name.c_str());
		solve();
	}
	ImGui::SameLine();
	if (ImGui::Button("Undo point"))
	{
		drop_last_vertex();
	}
	ImGui::SameLine();
	if (ImGui::Button("Go to plan"))
	{
		go_to_plan();
	}
	ImGui::SameLine();
	ImGui::Checkbox("Draw", &m_drawing);
	if (Global.scenery_georeferenced)
	{
		ImGui::SameLine();
		ImGui::Checkbox("Orthophoto", &m_showortho);
		if (m_showortho)
		{
			auto const pending{static_cast<int>(m_ortho.pending() + m_topo.pending())};
			if (pending > 0)
			{
				ImGui::SameLine();
				ImGui::TextDisabled("(%d tiles on the way)", pending);
			}
		}
	}

	if (m_document.niwelety.size() > 1)
	{
		ImGui::SetNextItemWidth(220.0f);
		if (ImGui::BeginCombo("Edited", m_document.niwelety[m_niweleta].name.c_str()))
		{
			for (std::size_t i = 0; i < m_document.niwelety.size(); ++i)
			{
				if (ImGui::Selectable(m_document.niwelety[i].name.c_str(), i == m_niweleta))
				{
					m_niweleta = i;
					m_pending = false;
					clear_selection();
					std::snprintf(m_namebuf, sizeof(m_namebuf), "%s", m_document.niwelety[m_niweleta].name.c_str());
				}
			}
			ImGui::EndCombo();
		}
	}

	if (niweleta != nullptr)
	{
		ImGui::SetNextItemWidth(220.0f);
		if (ImGui::InputText("Name", m_namebuf, sizeof(m_namebuf)))
		{
			niweleta->name = m_namebuf;
		}
		else if (false == ImGui::IsItemActive() && niweleta->name != m_namebuf)
		{
			std::snprintf(m_namebuf, sizeof(m_namebuf), "%s", niweleta->name.c_str());
		}
		ImGui::SameLine();
		if (m_document.niwelety.size() > 1 && ImGui::Button("Delete niweleta"))
		{
			delete_current_niweleta();
			niweleta = current_niweleta();
		}
	}

	{
		auto const sel_label{m_sel_straight >= 0 ? "  |  selected #" + std::to_string(m_sel_straight) : std::string{}};
		ImGui::Text("straights: %d%s", niweleta != nullptr ? static_cast<int>(niweleta->straights.size()) : 0, sel_label.c_str());
	}

	auto const has_sel{niweleta != nullptr && m_sel_straight >= 0 && static_cast<std::size_t>(m_sel_straight) < niweleta->straights.size()};
	if (has_sel && ImGui::Button("Delete straight"))
	{
		delete_selected_straight();
		niweleta = current_niweleta();
	}
	if (has_sel)
	{
		auto &st{niweleta->straights[static_cast<std::size_t>(m_sel_straight)]};
		ImGui::SameLine();
		if (ImGui::Button(st.rel_kind == 1 ? "Re-pick parallel..." : "Parallel..."))
		{
			m_ask_rel_value = st.rel_kind == 1 ? st.rel_offset : 4.5;
			m_pick_rel = 1;
			m_status = "referencja: parallel";
		}
		ImGui::SameLine();
		if (ImGui::Button(st.rel_kind == 2 ? "Re-pick skew..." : "Skew 1:n..."))
		{
			m_ask_rel_value = st.rel_kind == 2 ? st.rel_cot * (st.rel_side < 0 ? -1.0 : 1.0) : 9.0;
			m_pick_rel = 2;
			m_status = "referencja: skew";
		}
		if (st.rel_kind == 1)
		{
			ImGui::Text("parallel → n%d s%d", st.rel_niw + 1, st.rel_str);
			ImGui::SameLine();
			ImGui::SetNextItemWidth(110.0f);
			if (ImGui::InputDouble("offset m##parlive", &st.rel_offset, 0.1, 1.0, "%.2f"))
			{
				solve();
			}
			ImGui::SameLine();
			if (ImGui::SmallButton("Clear##parlive"))
			{
				st.rel_kind = 0;
				m_status = "cleared parallel";
				solve();
			}
		}
		else if (st.rel_kind == 2)
		{
			auto skew_n{st.rel_cot * (st.rel_side < 0 ? -1.0 : 1.0)};
			ImGui::Text("skew → n%d s%d", st.rel_niw + 1, st.rel_str);
			ImGui::SameLine();
			ImGui::SetNextItemWidth(90.0f);
			if (ImGui::InputDouble("1:n##skewlive", &skew_n, 1.0, 1.0, "%.2f"))
			{
				if (std::abs(skew_n) < 1e-9)
				{
					st.rel_kind = 0;
					m_status = "cleared skew";
				}
				else
				{
					st.rel_cot = std::abs(skew_n);
					st.rel_side = skew_n < 0.0 ? -1 : 1;
				}
				solve();
			}
			ImGui::SameLine();
			ImGui::SetNextItemWidth(90.0f);
			if (ImGui::InputDouble("L##skewlive", &st.rel_length, 5.0, 20.0, "%.1f"))
			{
				if (st.rel_length < 1.0)
				{
					st.rel_length = 1.0;
				}
				solve();
			}
			ImGui::SameLine();
			if (ImGui::SmallButton("Clear##skewlive"))
			{
				st.rel_kind = 0;
				m_status = "cleared skew";
				solve();
			}
		}
	}

	if (m_pick_rel != 0)
	{
		ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.4f, 1.0f), "%s", m_pick_rel == 1 ? "referencja: parallel" : "referencja: skew");
		ImGui::SameLine();
		if (ImGui::SmallButton("Cancel pick"))
		{
			m_pick_rel = 0;
		}
	}
	if (m_pick_compound_point)
	{
		ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.4f, 1.0f), "punkty: %zu", m_fit_guides.size());
		ImGui::SameLine();
		if (ImGui::SmallButton("Fit##cmpick") && false == m_fit_guides.empty())
		{
			fit_compound_to_guides();
			m_pick_compound_point = false;
		}
		ImGui::SameLine();
		if (ImGui::SmallButton("Clear##cmpick"))
		{
			clear_fit_guides();
		}
		ImGui::SameLine();
		if (ImGui::SmallButton("Cancel##cmpick"))
		{
			m_pick_compound_point = false;
			clear_fit_guides();
		}
	}
}

void plan_panel::render_gaps()
{
	auto *niweleta{current_niweleta()};
	if (niweleta == nullptr || niweleta->straights.size() < 2)
	{
		return;
	}

	ImGui::Separator();
	if (false == ImGui::CollapsingHeader("Fits", ImGuiTreeNodeFlags_DefaultOpen))
	{
		return;
	}

	ImGui::BeginChild("plan_gaps", ImVec2(0.0f, 280.0f), false);
	bool dirty{false};
	for (std::size_t gap = 0; gap < niweleta->fits.size(); ++gap)
	{
		auto &fit{niweleta->fits[gap]};
		ImGui::PushID(static_cast<int>(gap));

		auto const selected_gap{m_sel_gap == static_cast<int>(gap)};
		if (ImGui::Selectable(selected_gap ? "gap##sel" : "gap##unsel", selected_gap, ImGuiSelectableFlags_AllowItemOverlap, ImVec2(36.0f, 0.0f)))
		{
			m_sel_gap = static_cast<int>(gap);
		}
		ImGui::SameLine();
		ImGui::Text("%d", static_cast<int>(gap));
		ImGui::SameLine(70.0f);
		ImGui::SetNextItemWidth(170.0f);
		// the library numbers these; 3 is the unified compound, any number of arcs eased by clothoids
		if (ImGui::Combo("##mode", &fit.mode, "none\0arc\0arc + clothoids\0compound\0"))
		{
			m_sel_gap = static_cast<int>(gap);
			if (fit.mode != 0 && fit.radius <= 0.0)
			{
				fit.radius = 300.0; // something drawable to start from, rather than a rejected fit
			}
			if (fit.mode == 3 && fit.arcs.empty())
			{
				// a basket needs at least two arcs to be one; the last absorbs the leftover deflection
				fit.arcs.push_back({600.0, 60.0, 0.0});
				fit.arcs.push_back({300.0, 0.0, 0.0});
			}
			dirty = true;
		}
		if (fit.mode == 1 || fit.mode == 2)
		{
			ImGui::SameLine();
			ImGui::SetNextItemWidth(120.0f);
			if (ImGui::InputDouble("R", &fit.radius, 10.0, 100.0, "%.1f"))
			{
				dirty = true;
			}
		}
		if (fit.mode == 2)
		{
			ImGui::SameLine();
			ImGui::SetNextItemWidth(120.0f);
			if (ImGui::InputDouble("L", &fit.transition, 5.0, 20.0, "%.1f"))
			{
				dirty = true;
			}
		}
		if (fit.mode == 3)
		{
			render_compound(fit, gap, dirty);
		}
		// a fit the library could not produce is simply absent from its answer; say so rather than
		// leaving the parameters looking as though they took effect
		if (fit.mode != 0 && false == fit_applied(gap))
		{
			ImGui::SameLine();
			ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.4f, 1.0f), "does not fit");
		}
		else if (fit.mode != 0 && m_niweleta < m_solved.size())
		{
			auto const &solved{m_solved[m_niweleta]};
			double total{0.0};
			double kp_len{0.0};
			int kp_count{0};
			std::vector<double> radii;
			for (auto const &poly : solved.polylines)
			{
				if (poly.gap != static_cast<int>(gap) || poly.element_index < 0)
				{
					continue;
				}
				// count each element once (rails may duplicate polylines)
				bool seen{false};
				for (auto const &other : solved.polylines)
				{
					if (&other == &poly)
					{
						break;
					}
					if (other.gap == poly.gap && other.element_index == poly.element_index)
					{
						seen = true;
						break;
					}
				}
				if (seen)
				{
					continue;
				}
				total += poly.length;
				if (poly.kind == 2)
				{
					kp_len += poly.length;
					++kp_count;
				}
				if (poly.kind == 1)
				{
					auto const r{poly.radius_start > 0.0 ? poly.radius_start : poly.radius_end};
					if (r > 0.0)
					{
						radii.push_back(r);
					}
				}
			}
			if (false == radii.empty())
			{
				std::string rtxt{"R lukow: "};
				for (std::size_t i = 0; i < radii.size(); ++i)
				{
					if (i > 0)
					{
						rtxt += " / ";
					}
					char piece[32];
					std::snprintf(piece, sizeof(piece), "%.1f", radii[i]);
					rtxt += piece;
				}
				rtxt += " m";
				ImGui::TextDisabled("%s", rtxt.c_str());
			}
			if (kp_count > 0)
			{
				ImGui::TextDisabled("KP: %d  L = %.1f m   (krzywa lacznie %.1f m)", kp_count, kp_len, total);
			}
			else if (total > 0.0)
			{
				ImGui::TextDisabled("krzywa L = %.1f m", total);
			}
		}

		ImGui::PopID();
	}
	ImGui::EndChild();

	if (dirty)
	{
		solve();
	}
}

void plan_panel::render_compound(maj0sted::web::GapFit &Fit, std::size_t const Gap, bool &Dirty)
{
	ImGui::Indent();

	if (ImGui::SmallButton("Kosz##reopen") && false == m_draft.active)
	{
		if (reopen_draft_from_gap(Gap))
		{
			Dirty = false;
			ImGui::Unindent();
			return;
		}
	}

	ImGui::SetNextItemWidth(100.0f);
	if (ImGui::InputDouble("wejscie##ent", &Fit.entry_t, 5.0, 20.0, "%.1f"))
	{
		Dirty = true;
	}
	ImGui::SameLine();
	ImGui::SetNextItemWidth(100.0f);
	if (ImGui::InputDouble("wyjscie##ex", &Fit.exit_t, 5.0, 20.0, "%.1f"))
	{
		Dirty = true;
	}

	ImGui::Spacing();
	double station{Fit.entry_t};
	for (std::size_t i = 0; i < Fit.arcs.size(); ++i)
	{
		auto &arc{Fit.arcs[i]};
		auto const last{i + 1 == Fit.arcs.size()};
		ImGui::PushID(static_cast<int>(i));

		ImGui::AlignTextToFramePadding();
		ImGui::Text("Luk %d", static_cast<int>(i + 1));
		ImGui::SameLine();
		ImGui::SetNextItemWidth(90.0f);
		if (ImGui::InputDouble("R##ar", &arc.radius, 10.0, 100.0, "%.1f"))
		{
			Dirty = true;
		}

		if (false == last)
		{
			ImGui::SameLine();
			ImGui::SetNextItemWidth(90.0f);
			if (ImGui::InputDouble("L##al", &arc.length, 5.0, 20.0, "%.1f"))
			{
				Dirty = true;
			}
			ImGui::SameLine();
			ImGui::SetNextItemWidth(80.0f);
			if (ImGui::InputDouble("KP##at", &arc.transition_to_next, 5.0, 20.0, "%.1f"))
			{
				Dirty = true;
			}
			station += std::max(0.0, arc.length) + std::max(0.0, arc.transition_to_next);
			ImGui::TextDisabled("~%.0f m", station);
		}
		else
		{
			ImGui::SameLine();
			ImGui::TextDisabled("L auto");
		}

		ImGui::PopID();
	}

	if (ImGui::SmallButton("Add arc"))
	{
		auto const radius{Fit.arcs.empty() ? 300.0 : Fit.arcs.back().radius};
		if (false == Fit.arcs.empty() && !(Fit.arcs.back().length > 0.0))
		{
			Fit.arcs.back().length = 60.0;
		}
		Fit.arcs.insert(Fit.arcs.end() - (Fit.arcs.empty() ? 0 : 1), {radius, 60.0, 0.0});
		Dirty = true;
	}
	if (Fit.arcs.size() > 2)
	{
		ImGui::SameLine();
		if (ImGui::SmallButton("Remove arc"))
		{
			Fit.arcs.erase(Fit.arcs.end() - 2);
			Dirty = true;
		}
	}

	ImGui::Spacing();
	if (ImGui::Button("Fit through points"))
	{
		m_sel_gap = static_cast<int>(Gap);
		m_pick_compound_point = true;
		m_pick_rel = 0;
		m_drawing = false;
		m_fit_guides.clear();
		m_status = "click points, then Fit";
	}

	ImGui::Unindent();
}

void plan_panel::clear_fit_guides()
{
	m_fit_guides.clear();
	m_status.clear();
}

void plan_panel::fit_compound_to_guides()
{
	auto *niweleta{current_niweleta()};
	if (niweleta == nullptr || m_sel_gap < 0 || static_cast<std::size_t>(m_sel_gap) >= niweleta->fits.size())
	{
		return;
	}
	if (m_fit_guides.empty())
	{
		return;
	}
	auto const gap{static_cast<std::size_t>(m_sel_gap)};
	if (gap + 1 >= niweleta->straights.size())
	{
		return;
	}
	auto &fit{niweleta->fits[gap]};
	if (fit.mode != 3)
	{
		return;
	}
	if (fit.arcs.empty())
	{
		fit.arcs.push_back({fit.radius > 30.0 ? fit.radius : 300.0, 0.0, 0.0});
	}
	for (auto &arc : fit.arcs)
	{
		if (!(arc.radius >= 30.0))
		{
			arc.radius = 300.0;
		}
	}
	for (std::size_t i = 1; i < fit.arcs.size(); ++i)
	{
		if (std::abs(fit.arcs[i].radius - fit.arcs[i - 1].radius) < 1.0)
		{
			fit.arcs[i].radius = std::max(30.0, fit.arcs[i - 1].radius * 0.75);
		}
	}
	for (std::size_t i = 0; i + 1 < fit.arcs.size(); ++i)
	{
		if (!(fit.arcs[i].length > 0.0))
		{
			fit.arcs[i].length = 60.0;
		}
	}

	auto &entry{niweleta->straights[gap]};
	auto &exit{niweleta->straights[gap + 1]};

	// search window for straight ends: big enough to reach the guide cloud
	double gx{0.0};
	double gy{0.0};
	for (auto const &g : m_fit_guides)
	{
		gx += g.x;
		gy += g.y;
	}
	gx /= static_cast<double>(m_fit_guides.size());
	gy /= static_cast<double>(m_fit_guides.size());
	auto guide_span{80.0};
	for (auto const &g : m_fit_guides)
	{
		guide_span = std::max(guide_span, std::hypot(g.x - gx, g.y - gy) * 1.6);
	}
	auto const near_span{std::max(guide_span, 120.0)};
	auto const far_span{std::max(guide_span * 0.5, 60.0)};

	enum class KnobKind
	{
		EntryT,
		ExitT,
		Radius,
		Length,
		Between,
		EntryNearX,
		EntryNearY,
		ExitNearX,
		ExitNearY,
		EntryFarX,
		EntryFarY,
		ExitFarX,
		ExitFarY
	};
	struct Knob
	{
		KnobKind kind{};
		std::size_t arc{0};
		double lo{0.0};
		double hi{0.0};
		int priority{0};
	};

	std::vector<Knob> knobs;
	knobs.push_back({KnobKind::EntryT, 0, 0.0, std::max(fit.entry_t * 4.0, 180.0), 2});
	knobs.push_back({KnobKind::ExitT, 0, 0.0, std::max(fit.exit_t * 4.0, 180.0), 2});
	for (std::size_t i = 0; i < fit.arcs.size(); ++i)
	{
		auto const &arc{fit.arcs[i]};
		knobs.push_back({KnobKind::Radius, i, 30.0, std::max(arc.radius * 5.0, 1200.0), 3});
		if (i + 1 < fit.arcs.size())
		{
			knobs.push_back({KnobKind::Length, i, 5.0, std::max(arc.length * 6.0, 400.0), 3});
			knobs.push_back({KnobKind::Between, i, 0.0, std::max(arc.transition_to_next * 4.0, 160.0), 2});
		}
	}
	knobs.push_back({KnobKind::EntryNearX, 0, entry.x2 - near_span, entry.x2 + near_span, 4});
	knobs.push_back({KnobKind::EntryNearY, 0, entry.y2 - near_span, entry.y2 + near_span, 4});
	knobs.push_back({KnobKind::ExitNearX, 0, exit.x1 - near_span, exit.x1 + near_span, 4});
	knobs.push_back({KnobKind::ExitNearY, 0, exit.y1 - near_span, exit.y1 + near_span, 4});
	knobs.push_back({KnobKind::EntryFarX, 0, entry.x1 - far_span, entry.x1 + far_span, 1});
	knobs.push_back({KnobKind::EntryFarY, 0, entry.y1 - far_span, entry.y1 + far_span, 1});
	knobs.push_back({KnobKind::ExitFarX, 0, exit.x2 - far_span, exit.x2 + far_span, 1});
	knobs.push_back({KnobKind::ExitFarY, 0, exit.y2 - far_span, exit.y2 + far_span, 1});
	std::stable_sort(knobs.begin(), knobs.end(), [](Knob const &A, Knob const &B) { return A.priority > B.priority; });

	auto trial{m_document};
	auto &trial_niw{trial.niwelety[m_niweleta]};
	auto &trial_fit{trial_niw.fits[gap]};
	auto &trial_entry{trial_niw.straights[gap]};
	auto &trial_exit{trial_niw.straights[gap + 1]};

	auto get_value = [&](Knob const &K) -> double {
		switch (K.kind)
		{
		case KnobKind::EntryT:
			return trial_fit.entry_t;
		case KnobKind::ExitT:
			return trial_fit.exit_t;
		case KnobKind::Radius:
			return trial_fit.arcs[K.arc].radius;
		case KnobKind::Length:
			return trial_fit.arcs[K.arc].length;
		case KnobKind::Between:
			return trial_fit.arcs[K.arc].transition_to_next;
		case KnobKind::EntryNearX:
			return trial_entry.x2;
		case KnobKind::EntryNearY:
			return trial_entry.y2;
		case KnobKind::ExitNearX:
			return trial_exit.x1;
		case KnobKind::ExitNearY:
			return trial_exit.y1;
		case KnobKind::EntryFarX:
			return trial_entry.x1;
		case KnobKind::EntryFarY:
			return trial_entry.y1;
		case KnobKind::ExitFarX:
			return trial_exit.x2;
		case KnobKind::ExitFarY:
			return trial_exit.y2;
		}
		return 0.0;
	};
	auto set_value = [&](Knob const &K, double const V) {
		switch (K.kind)
		{
		case KnobKind::EntryT:
			trial_fit.entry_t = V;
			break;
		case KnobKind::ExitT:
			trial_fit.exit_t = V;
			break;
		case KnobKind::Radius:
			trial_fit.arcs[K.arc].radius = V;
			break;
		case KnobKind::Length:
			trial_fit.arcs[K.arc].length = V;
			break;
		case KnobKind::Between:
			trial_fit.arcs[K.arc].transition_to_next = V;
			break;
		case KnobKind::EntryNearX:
			trial_entry.x2 = V;
			break;
		case KnobKind::EntryNearY:
			trial_entry.y2 = V;
			break;
		case KnobKind::ExitNearX:
			trial_exit.x1 = V;
			break;
		case KnobKind::ExitNearY:
			trial_exit.y1 = V;
			break;
		case KnobKind::EntryFarX:
			trial_entry.x1 = V;
			break;
		case KnobKind::EntryFarY:
			trial_entry.y1 = V;
			break;
		case KnobKind::ExitFarX:
			trial_exit.x2 = V;
			break;
		case KnobKind::ExitFarY:
			trial_exit.y2 = V;
			break;
		}
	};
	auto radii_legal = [&]() {
		for (std::size_t i = 0; i < trial_fit.arcs.size(); ++i)
		{
			if (!(trial_fit.arcs[i].radius >= 30.0))
			{
				return false;
			}
			if (i > 0 && std::abs(trial_fit.arcs[i].radius - trial_fit.arcs[i - 1].radius) < 1.0)
			{
				return false;
			}
		}
		return true;
	};

	auto evaluate = [&](double &OutMaxMiss) {
		OutMaxMiss = std::numeric_limits<double>::infinity();
		if (false == radii_legal())
		{
			return std::numeric_limits<double>::infinity();
		}
		// tiny straights break the fitter
		if (std::hypot(trial_entry.x2 - trial_entry.x1, trial_entry.y2 - trial_entry.y1) < 1.0 ||
		    std::hypot(trial_exit.x2 - trial_exit.x1, trial_exit.y2 - trial_exit.y1) < 1.0)
		{
			return std::numeric_limits<double>::infinity();
		}
		auto const solved{maj0sted::web::solve_project(trial.niwelety)};
		if (m_niweleta >= solved.size())
		{
			return std::numeric_limits<double>::infinity();
		}
		double sum2{0.0};
		double max_miss{0.0};
		for (auto const &g : m_fit_guides)
		{
			auto const d{distance_to_gap_track(solved[m_niweleta], m_sel_gap, g.x, g.y)};
			if (false == std::isfinite(d))
			{
				return std::numeric_limits<double>::infinity();
			}
			sum2 += d * d;
			max_miss = std::max(max_miss, d);
		}
		OutMaxMiss = max_miss;
		return sum2;
	};

	double best_max{0.0};
	auto best_cost{evaluate(best_max)};
	auto best_niw{trial_niw};

	auto scan_knob = [&](Knob const &K, int const Samples) {
		for (int s = 0; s < Samples; ++s)
		{
			auto const v{K.lo + (K.hi - K.lo) * static_cast<double>(s) / static_cast<double>(std::max(1, Samples - 1))};
			set_value(K, v);
			double max_miss{0.0};
			auto const cost{evaluate(max_miss)};
			if (cost < best_cost)
			{
				best_cost = cost;
				best_max = max_miss;
				best_niw = trial_niw;
			}
		}
		trial_niw = best_niw;
	};

	for (int round = 0; round < 3; ++round)
	{
		for (auto const &knob : knobs)
		{
			auto const samples{knob.priority >= 4 ? 22 : (knob.priority >= 3 ? 20 : (knob.priority >= 2 ? 14 : 10))};
			scan_knob(knob, samples);
		}
	}

	for (auto const &knob : knobs)
	{
		trial_niw = best_niw;
		auto const centre{get_value(knob)};
		auto const span{(knob.hi - knob.lo) / 16.0};
		auto const lo{std::max(knob.lo, centre - span)};
		auto const hi{std::min(knob.hi, centre + span)};
		for (int s = 0; s < 14; ++s)
		{
			set_value(knob, lo + (hi - lo) * static_cast<double>(s) / 13.0);
			double max_miss{0.0};
			auto const cost{evaluate(max_miss)};
			if (cost < best_cost)
			{
				best_cost = cost;
				best_max = max_miss;
				best_niw = trial_niw;
			}
		}
	}

	*niweleta = best_niw;
	// drop constraints that would yank the ends back after we moved them on purpose
	niweleta->straights[gap].rel_kind = 0;
	niweleta->straights[gap + 1].rel_kind = 0;
	solve();
	auto const rms{std::sqrt(best_cost / static_cast<double>(m_fit_guides.size()))};
	m_status = "fit " + std::to_string(m_fit_guides.size()) + " pts  rms " + to_string(rms, 2) + "  max " + to_string(best_max, 2) + " m";
}

void plan_panel::render_switches()
{
	if (false == ImGui::CollapsingHeader("Rozjazdy"))
	{
		return;
	}

	// typical Polish turnout templates: pick one to set the crossing mark and radius at once
	auto const &presets{editor::turnout_presets()};
	ImGui::SetNextItemWidth(160.0f);
	if (ImGui::BeginCombo("typ", "wybierz szablon"))
	{
		for (auto const &preset : presets)
		{
			if (ImGui::Selectable(preset.name.c_str()))
			{
				m_sw_crossing = preset.crossing_n;
				m_sw_radius = preset.radius;
			}
		}
		ImGui::EndCombo();
	}
	// each single-arc template spans R*atan(1/n) between its tangent points; shown so its length is known
	ImGui::SameLine();
	ImGui::TextDisabled("dl. ~%.1f m", m_sw_radius * std::atan(1.0 / std::max(1.0, m_sw_crossing)));

	ImGui::SetNextItemWidth(90.0f);
	ImGui::Combo("strona", &m_sw_side, "w lewo\0w prawo\0");
	ImGui::SameLine();
	ImGui::SetNextItemWidth(90.0f);
	ImGui::InputDouble("skos 1:", &m_sw_crossing, 1.0, 1.0, "%.1f");
	ImGui::SameLine();
	ImGui::SetNextItemWidth(90.0f);
	ImGui::InputDouble("R", &m_sw_radius, 10.0, 50.0, "%.0f");
	if (m_sw_crossing < 4.0)
	{
		m_sw_crossing = 4.0;
	}
	if (m_sw_radius < 50.0)
	{
		m_sw_radius = 50.0;
	}

	if (m_pick_switch != 0)
	{
		ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f), "kliknij niweletę na wprost...");
		ImGui::SameLine();
		if (ImGui::Button("anuluj"))
		{
			m_pick_switch = 0;
		}
	}
	else if (ImGui::Button("Dodaj rozjazd"))
	{
		m_pick_switch = 1;
		m_status = "kliknij niweletę na wprost";
	}

	ImGui::TextDisabled("odnoga wychodzi wymuszenie stycznie; luk koszowy zaraz za rozjazdem = luka 0 odnogi");

	// existing switches: the theoretical tangent lengths the solver worked out, plus the internal
	// curve (a plain arc, or a łuk koszowy right inside the switch)
	for (std::size_t i = 0; i < m_document.junctions.size(); ++i)
	{
		auto &junction{m_document.junctions[i]};
		ImGui::PushID(static_cast<int>(i));
		char const *through_name{junction.through >= 0 && static_cast<std::size_t>(junction.through) < m_document.niwelety.size()
		                             ? m_document.niwelety[static_cast<std::size_t>(junction.through)].name.c_str()
		                             : "?"};
		bool const valid{i < m_junctions.size() && m_junctions[i].valid};

		ImGui::AlignTextToFramePadding();
		ImGui::Text("%zu. na %s, 1:%.1f %s", i + 1, through_name, junction.crossing_n, junction.side == 0 ? "L" : "P");
		if (valid)
		{
			ImGui::SameLine();
			ImGui::TextDisabled("dl=%.1f m, styczne t=%.1f / %.1f m", m_junctions[i].length, m_junctions[i].tangent_front, m_junctions[i].tangent_back);
		}
		else
		{
			ImGui::SameLine();
			ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.4f, 1.0f), "nie wpasowany");
		}
		ImGui::SameLine();
		if (ImGui::SmallButton("usuń"))
		{
			m_document.junctions.erase(m_document.junctions.begin() + static_cast<long>(i));
			solve();
			ImGui::PopID();
			break;
		}

		// internal diverging curve of the switch: a single arc, or a basket of arcs
		ImGui::Indent();
		int curve_mode{junction.curve.mode == 3 ? 1 : 0};
		ImGui::SetNextItemWidth(110.0f);
		if (ImGui::Combo("krzywa", &curve_mode, "luk\0koszowy\0"))
		{
			if (curve_mode == 1)
			{
				junction.curve.mode = 3;
				if (junction.curve.arcs.empty())
				{
					// a basket needs at least two arcs; the last absorbs the rest of the turn
					junction.curve.arcs.push_back({junction.curve.radius > 0.0 ? junction.curve.radius * 2.0 : 600.0, 40.0, 0.0});
					junction.curve.arcs.push_back({junction.curve.radius > 0.0 ? junction.curve.radius : 300.0, 0.0, 0.0});
				}
			}
			else
			{
				junction.curve.mode = 1;
			}
			solve();
		}

		if (junction.curve.mode == 3)
		{
			ImGui::SameLine();
			ImGui::SetNextItemWidth(80.0f);
			if (ImGui::InputDouble("wej##je", &junction.curve.entry_t, 5.0, 20.0, "%.1f"))
			{
				solve();
			}
			ImGui::SameLine();
			ImGui::SetNextItemWidth(80.0f);
			if (ImGui::InputDouble("wyj##jx", &junction.curve.exit_t, 5.0, 20.0, "%.1f"))
			{
				solve();
			}
			for (std::size_t a = 0; a < junction.curve.arcs.size(); ++a)
			{
				auto &arc{junction.curve.arcs[a]};
				bool const last{a + 1 == junction.curve.arcs.size()};
				ImGui::PushID(static_cast<int>(a));
				ImGui::AlignTextToFramePadding();
				ImGui::Text("Luk %d", static_cast<int>(a + 1));
				ImGui::SameLine();
				ImGui::SetNextItemWidth(90.0f);
				if (ImGui::InputDouble("R##jar", &arc.radius, 10.0, 100.0, "%.1f"))
				{
					solve();
				}
				if (false == last)
				{
					ImGui::SameLine();
					ImGui::SetNextItemWidth(90.0f);
					if (ImGui::InputDouble("L##jal", &arc.length, 5.0, 20.0, "%.1f"))
					{
						solve();
					}
					ImGui::SameLine();
					ImGui::SetNextItemWidth(80.0f);
					if (ImGui::InputDouble("KP##jat", &arc.transition_to_next, 5.0, 20.0, "%.1f"))
					{
						solve();
					}
				}
				else
				{
					ImGui::SameLine();
					ImGui::TextDisabled("domyka");
				}
				ImGui::PopID();
			}
			if (ImGui::SmallButton("+ luk"))
			{
				junction.curve.arcs.insert(junction.curve.arcs.end() - 1, {junction.curve.arcs.back().radius, 40.0, 0.0});
				solve();
			}
			if (junction.curve.arcs.size() > 2)
			{
				ImGui::SameLine();
				if (ImGui::SmallButton("- luk"))
				{
					junction.curve.arcs.erase(junction.curve.arcs.end() - 2);
					solve();
				}
			}
		}
		else
		{
			ImGui::SameLine();
			ImGui::SetNextItemWidth(90.0f);
			if (ImGui::InputDouble("R##jr", &junction.curve.radius, 10.0, 50.0, "%.0f"))
			{
				solve();
			}
		}

		if (junction.branch >= 0 && static_cast<std::size_t>(junction.branch) < m_document.niwelety.size())
		{
			ImGui::SameLine();
			if (ImGui::SmallButton("edytuj odnoge"))
			{
				m_niweleta = static_cast<std::size_t>(junction.branch);
				clear_selection();
				std::snprintf(m_namebuf, sizeof(m_namebuf), "%s", m_document.niwelety[m_niweleta].name.c_str());
			}
		}
		ImGui::Unindent();
		ImGui::PopID();
	}
}

void plan_panel::add_switch_on(std::size_t const Through, double const Wx, double const Wy)
{
	if (Through >= m_solved.size())
	{
		return;
	}
	auto const &centre{m_solved[Through].centreline};
	if (centre.size() < 2)
	{
		m_status = "niweleta bez osi";
		return;
	}

	// nearest station: walk the centreline, project the click onto each segment
	double best_station{0.0};
	double best_dist{std::numeric_limits<double>::max()};
	double travelled{0.0};
	for (std::size_t i = 0; i + 1 < centre.size(); ++i)
	{
		double const ax{centre[i].x};
		double const ay{centre[i].y};
		double const dx{centre[i + 1].x - ax};
		double const dy{centre[i + 1].y - ay};
		double const seg{std::hypot(dx, dy)};
		if (seg < 1e-9)
		{
			continue;
		}
		double const t{std::clamp(((Wx - ax) * dx + (Wy - ay) * dy) / (seg * seg), 0.0, 1.0)};
		double const px{ax + dx * t};
		double const py{ay + dy * t};
		double const d{std::hypot(Wx - px, Wy - py)};
		if (d < best_dist)
		{
			best_dist = d;
			best_station = travelled + seg * t;
		}
		travelled += seg;
	}

	maj0sted::web::Junction junction;
	junction.through = static_cast<int>(Through);
	junction.station = best_station;
	junction.side = m_sw_side;
	junction.facing = true;
	junction.crossing_n = m_sw_crossing;
	junction.curve.mode = 1; // plain arc; the internal curve can be made compound later
	junction.curve.radius = m_sw_radius;
	junction.branch = static_cast<int>(m_document.niwelety.size());

	// a branch niweleta to draw the diverging track into, pinned to the frog by the solver
	maj0sted::web::NiweletaSpec branch;
	branch.name = "odnoga " + std::to_string(m_document.niwelety.size() + 1);
	branch.straights.push_back({Wx, Wy, Wx + 10.0, Wy, false}); // placeholder, repositioned below
	m_document.niwelety.push_back(std::move(branch));
	m_document.junctions.push_back(junction);

	// solve once to learn where the frog lands, then seed the branch to leave it tangent
	solve();
	if (false == m_junctions.empty() && m_junctions.back().valid)
	{
		auto const &geom{m_junctions.back()};
		auto &seed{m_document.niwelety[static_cast<std::size_t>(junction.branch)].straights.front()};
		seed = {geom.fx, geom.fy, geom.fx + geom.fhx * 50.0, geom.fy + geom.fhy * 50.0, false};
	}

	m_niweleta = static_cast<std::size_t>(junction.branch);
	clear_selection();
	std::snprintf(m_namebuf, sizeof(m_namebuf), "%s", m_document.niwelety[m_niweleta].name.c_str());
	solve();
	m_status = "rozjazd 1:" + to_string(m_sw_crossing, 1) + " dodany";
}

void plan_panel::render_storage()
{
	ImGui::Separator();
	ImGui::SetNextItemWidth(240.0f);
	ImGui::InputText("##path", m_path, sizeof(m_path));
	ImGui::SameLine();
	if (ImGui::Button("Save"))
	{
		double planx{0.0};
		double plany{0.0};
		world_to_plan(editor_mode::get_camera().Pos, planx, plany);
		m_document.view_x = planx;
		m_document.view_y = plany;
		m_document.view_extent = static_cast<double>(Global.editor_ortho_extent);
		m_document.origin_set = true;
		m_document.georeferenced = Global.scenery_georeferenced;
		m_document.origin_x = Global.scenery_origin.x;
		m_document.origin_y = Global.scenery_origin.y;
		m_status = maj0sted::app::save_project(m_document, m_path) ? "saved to " + std::string(m_path) : "could not write " + std::string(m_path);
	}
	ImGui::SameLine();
	if (ImGui::Button("Load"))
	{
		auto const loaded{maj0sted::app::load_project(m_path)};
		if (loaded)
		{
			m_document = *loaded;
			if (m_document.niwelety.empty())
			{
				m_document.niwelety.push_back({"niweleta 1", {}, {}});
			}
			auto looks_puwg = [](maj0sted::app::EditorDocument const &Doc) {
				if (std::abs(Doc.view_x) > 10000.0 || std::abs(Doc.view_y) > 10000.0)
				{
					return true;
				}
				for (auto const &n : Doc.niwelety)
				{
					for (auto const &s : n.straights)
					{
						if (std::abs(s.x1) > 10000.0 || std::abs(s.y1) > 10000.0 || std::abs(s.x2) > 10000.0 || std::abs(s.y2) > 10000.0)
						{
							return true;
						}
					}
				}
				return false;
			};
			if (m_document.origin_set)
			{
				Global.scenery_georeferenced = m_document.georeferenced;
				Global.scenery_origin = {m_document.origin_x, m_document.origin_y};
			}
			if (false == Global.scenery_georeferenced && looks_puwg(m_document))
			{
				Global.scenery_georeferenced = true;
			}
			m_document.origin_set = true;
			m_document.georeferenced = Global.scenery_georeferenced;
			m_document.origin_x = Global.scenery_origin.x;
			m_document.origin_y = Global.scenery_origin.y;
			m_niweleta = 0;
			m_pending = false;
			clear_selection();
			m_pick_rel = 0;
			m_ask_rel_param = false;
			std::snprintf(m_namebuf, sizeof(m_namebuf), "%s", m_document.niwelety[0].name.c_str());
			solve();
			if (m_document.view_extent > 0.0)
			{
				auto &camera{editor_mode::get_camera()};
				auto const centre{plan_to_world(m_document.view_x, m_document.view_y)};
				camera.Pos.x = centre.x;
				camera.Pos.z = centre.z;
				Global.editor_ortho_extent = std::clamp(static_cast<float>(m_document.view_extent), 5.0f, 20000.0f);
			}
			else
			{
				go_to_plan();
			}
			m_status = "loaded " + std::string(m_path);
		}
		else
		{
			m_status = "could not read " + std::string(m_path);
		}
	}
	ImGui::SetNextItemWidth(240.0f);
	ImGui::InputText("##scnpath", m_scn_path, sizeof(m_scn_path));
	ImGui::SameLine();
	if (ImGui::Button("Export SCN"))
	{
		export_scn();
	}
	if (false == m_status.empty())
	{
		ImGui::TextDisabled("%s", m_status.c_str());
	}
}

void plan_panel::export_scn()
{
	solve();
	maj0sted::io::ScnExportOptions opt;
	opt.origin_east = Global.scenery_origin.x;
	opt.origin_north = Global.scenery_origin.y;
	std::ofstream out(m_scn_path, std::ios::binary | std::ios::trunc);
	if (!out)
	{
		m_status = "could not write " + std::string(m_scn_path);
		return;
	}
	auto const result{maj0sted::io::export_scn(m_solved, opt, out)};
	out.flush();
	m_status = out.good() ? ("exported " + std::to_string(result.tracks) + " tracks")
	                      : ("could not write " + std::string(m_scn_path));
}


void plan_panel::solve()
{
	for (auto &niweleta : m_document.niwelety)
	{
		maj0sted::app::align_gap_fits(niweleta);
	}
	maj0sted::app::apply_straight_constraints(m_document.niwelety);
	// switches pin their branches, so the whole layout is solved together
	auto layout{maj0sted::web::solve_layout(m_document.niwelety, m_document.junctions)};
	m_solved = std::move(layout.niwelety);
	m_junctions = std::move(layout.junctions);
}


void plan_panel::apply_constraints()
{
	maj0sted::app::apply_straight_constraints(m_document.niwelety);
}


maj0sted::web::NiweletaSpec *plan_panel::current_niweleta()
{
	return m_niweleta < m_document.niwelety.size() ? &m_document.niwelety[m_niweleta] : nullptr;
}

maj0sted::web::NiweletaSpec const *plan_panel::current_niweleta() const
{
	return m_niweleta < m_document.niwelety.size() ? &m_document.niwelety[m_niweleta] : nullptr;
}

void plan_panel::align_fits()
{
	for (auto &niweleta : m_document.niwelety)
	{
		maj0sted::app::align_gap_fits(niweleta);
	}
}


bool plan_panel::fit_applied(std::size_t const Gap) const
{
	if (m_niweleta >= m_solved.size())
	{
		return false;
	}
	for (auto const &fit : m_solved[m_niweleta].applied_fits)
	{
		if (fit.gap >= 0 && static_cast<std::size_t>(fit.gap) == Gap)
		{
			return true;
		}
	}
	return false;
}

maj0sted::web::StraightSpec const *plan_panel::handle_straight(std::size_t const Niw, std::size_t const Index) const
{
	if (Niw >= m_document.niwelety.size())
	{
		return nullptr;
	}
	auto const &raw{m_document.niwelety[Niw].straights};
	if (Index >= raw.size())
	{
		return nullptr;
	}
	if (Niw < m_solved.size() && Index < m_solved[Niw].rendered_straights.size())
	{
		return &m_solved[Niw].rendered_straights[Index];
	}
	return &raw[Index];
}

void plan_panel::append_vertex(double const X, double const Y)
{
	auto *niweleta{current_niweleta()};
	if (niweleta == nullptr)
	{
		return;
	}

	if (niweleta->straights.empty())
	{
		if (false == m_pending)
		{
			// nothing to attach to yet, so the first corner waits for the one that closes the straight
			m_pending = true;
			m_pendingx = X;
			m_pendingy = Y;
			return;
		}
		m_pending = false;
		niweleta->straights.push_back({m_pendingx, m_pendingy, X, Y, false});
	}
	else
	{
		// seed from the previous end, but the joint is not locked afterwards
		auto const &last{niweleta->straights.back()};
		niweleta->straights.push_back({last.x2, last.y2, X, Y, false});
	}

	solve();
}

void plan_panel::drop_last_vertex()
{
	if (m_pending)
	{
		m_pending = false;
		return;
	}

	auto *niweleta{current_niweleta()};
	if (niweleta == nullptr || niweleta->straights.empty())
	{
		return;
	}

	m_sel_straight = static_cast<int>(niweleta->straights.size() - 1);
	delete_selected_straight();
}

void plan_panel::clear_selection()
{
	m_sel_straight = -1;
	m_sel_end = -1;
	m_dragging = false;
	m_dragging_cmp = false;
	m_cmp_gap = -1;
	m_cmp_kind = -1;
	m_cmp_arc = -1;
	m_dragging_draft = false;
	m_draft_kind = -1;
	m_draft_arc = -1;
}

void plan_panel::delete_selected_straight()
{
	auto *niweleta{current_niweleta()};
	if (niweleta == nullptr || m_sel_straight < 0 || static_cast<std::size_t>(m_sel_straight) >= niweleta->straights.size())
	{
		return;
	}

	auto const sel{m_sel_straight};
	niweleta->straights.erase(niweleta->straights.begin() + sel);

	// gaps touching the removed straight disappear; later gaps shift down
	std::vector<maj0sted::web::GapFit> kept;
	kept.reserve(niweleta->fits.size());
	for (auto const &fit : niweleta->fits)
	{
		if (fit.gap < sel - 1)
		{
			kept.push_back(fit);
		}
		else if (fit.gap >= sel + 1)
		{
			auto copy{fit};
			copy.gap = fit.gap - 1;
			kept.push_back(copy);
		}
	}
	niweleta->fits = std::move(kept);

	// remap constraint references across every niweleta
	for (std::size_t n = 0; n < m_document.niwelety.size(); ++n)
	{
		for (auto &st : m_document.niwelety[n].straights)
		{
			if (st.rel_kind == 0)
			{
				continue;
			}
			if (st.rel_niw == static_cast<int>(m_niweleta))
			{
				if (st.rel_str == sel)
				{
					st.rel_kind = 0;
				}
				else if (st.rel_str > sel)
				{
					--st.rel_str;
				}
			}
		}
	}

	clear_selection();
	solve();
}

void plan_panel::delete_current_niweleta()
{
	if (m_document.niwelety.size() <= 1 || m_niweleta >= m_document.niwelety.size())
	{
		return;
	}

	auto const removed{static_cast<int>(m_niweleta)};
	m_document.niwelety.erase(m_document.niwelety.begin() + static_cast<std::ptrdiff_t>(m_niweleta));

	for (auto &niweleta : m_document.niwelety)
	{
		for (auto &st : niweleta.straights)
		{
			if (st.rel_kind == 0)
			{
				continue;
			}
			if (st.rel_niw == removed)
			{
				st.rel_kind = 0;
			}
			else if (st.rel_niw > removed)
			{
				--st.rel_niw;
			}
		}
	}

	if (m_niweleta >= m_document.niwelety.size())
	{
		m_niweleta = m_document.niwelety.size() - 1;
	}
	m_pending = false;
	clear_selection();
	m_pick_rel = 0;
	m_ask_rel_param = false;
	if (auto *niweleta{current_niweleta()}; niweleta != nullptr)
	{
		std::snprintf(m_namebuf, sizeof(m_namebuf), "%s", niweleta->name.c_str());
	}
	solve();
}

bool plan_panel::hit_compound_station(ImVec2 const &Mouse, int &OutGap, int &OutKind, int &OutArc) const
{
	auto const *niweleta{current_niweleta()};
	if (niweleta == nullptr || m_niweleta >= m_solved.size())
	{
		return false;
	}
	auto best{14.0f};
	bool found{false};
	for (auto const &fit : niweleta->fits)
	{
		if (fit.mode != 3 || fit.arcs.empty())
		{
			continue;
		}
		for (auto const &h : compound_station_handles(fit, m_solved[m_niweleta], fit.gap))
		{
			ImVec2 screen;
			if (false == world_to_screen(plan_to_world(h.x, h.y), screen))
			{
				continue;
			}
			auto const d{std::hypot(screen.x - Mouse.x, screen.y - Mouse.y)};
			if (d <= best)
			{
				best = d;
				OutGap = h.gap;
				OutKind = h.kind;
				OutArc = h.arc;
				found = true;
			}
		}
	}
	return found;
}

void plan_panel::begin_draft()
{
	discard_draft();
	m_draft.reedit_gap = -1;
	m_draft.reedit_backup = {};
	m_draft_place = 1;
	m_anchor_pick = 0;
	m_drawing = false;
	m_pick_compound_point = false;
	m_pick_rel = 0;
	m_dragging_draft = false;
	m_draft_append_kind = 0;
	m_fit_guides.clear();
	m_status = "kliknij koniec prostej";
}

void plan_panel::discard_draft()
{
	auto *niweleta{current_niweleta()};
	maj0sted::app::discard_basket_draft(m_draft, niweleta);
	if (niweleta != nullptr)
	{
		solve();
	}
	m_draft_polys.clear();
	m_draft_place = 0;
	m_anchor_pick = 0;
	m_dragging_draft = false;
	m_draft_kind = -1;
	m_draft_arc = -1;
	m_draft_append_kind = 0;
	m_status.clear();
}


bool plan_panel::reopen_draft_from_gap(std::size_t const Gap)
{
	auto *niweleta{current_niweleta()};
	if (niweleta == nullptr)
	{
		return false;
	}
	auto const *solved{m_niweleta < m_solved.size() ? &m_solved[m_niweleta] : nullptr};
	if (false == maj0sted::app::reopen_basket_draft(m_draft, *niweleta, Gap, solved))
	{
		m_status = "nie compound";
		return false;
	}
	m_draft_place = 0;
	m_anchor_pick = 0;
	m_dragging_draft = false;
	m_drawing = false;
	m_sel_gap = static_cast<int>(Gap);
	m_sel_straight = static_cast<int>(Gap);
	m_sel_end = 1;
	solve();
	rebuild_draft_preview();
	m_status = "luka " + std::to_string(Gap);
	return true;
}


void plan_panel::start_draft_from_end(std::size_t const Str, int const End)
{
	auto *niweleta{current_niweleta()};
	if (niweleta == nullptr || Str >= niweleta->straights.size() || (End != 0 && End != 1))
	{
		return;
	}
	maj0sted::app::start_basket_draft_from_end(m_draft, niweleta->straights[Str], static_cast<int>(Str), End, m_draft_r);
	m_draft_place = 0;
	m_draft_append_kind = 0;
	m_drawing = false;
	m_sel_straight = static_cast<int>(Str);
	m_sel_end = End;
	rebuild_draft_preview();
	m_status.clear();
}


void plan_panel::append_draft_at(double const X, double const Y)
{
	if (false == m_draft.active)
	{
		return;
	}
	double tipx{0.0};
	double tipy{0.0};
	if (false == draft_tip(tipx, tipy))
	{
		return;
	}
	auto const len{std::max(5.0, std::hypot(X - tipx, Y - tipy))};
	m_draft.preview_next_r = std::max(30.0, m_draft_r);

	if (m_draft_append_kind == 0)
	{
		m_draft.arcs.push_back({std::max(30.0, m_draft_r), len, 0.0});
		m_status = "luk L=" + to_string(len, 1) + " R=" + to_string(m_draft_r, 0);
	}
	else if (m_draft_append_kind == 1)
	{
		if (m_draft.arcs.empty())
		{
			m_draft.entry_t = len;
			m_status = "KP wejście " + to_string(len, 1) + " m";
		}
		else
		{
			m_draft.arcs.back().transition_to_next = len;
			m_draft.exit_t = 0.0;
			m_status = "KP między " + to_string(len, 1) + " m";
		}
	}
	else
	{
		if (m_draft.arcs.empty())
		{
			m_status = "najpierw łuk";
			return;
		}
		m_draft.exit_t = len;
		rebuild_draft_preview();
		finish_draft();
		return;
	}
	rebuild_draft_preview();
}

void plan_panel::undo_draft_fragment()
{
	if (false == m_draft.active)
	{
		return;
	}
	if (m_draft.exit_t > 1e-6)
	{
		m_draft.exit_t = 0.0;
	}
	else if (false == m_draft.arcs.empty() && m_draft.arcs.back().transition_to_next > 1e-6)
	{
		m_draft.arcs.back().transition_to_next = 0.0;
	}
	else if (false == m_draft.arcs.empty())
	{
		m_draft.arcs.pop_back();
	}
	else if (m_draft.entry_t > 1e-6)
	{
		m_draft.entry_t = 0.0;
	}
	else
	{
		m_status = "nic do cofniecia";
		return;
	}
	rebuild_draft_preview();
	m_status = "cofnieto fragment";
}

bool plan_panel::draft_tip(double &OutX, double &OutY) const
{
	double az{0.0};
	return draft_end_pose(OutX, OutY, az);
}

bool plan_panel::draft_end_pose(double &OutX, double &OutY, double &OutAz) const
{
	if (false == m_draft.active)
	{
		return false;
	}
	OutAz = m_draft.az0;
	if (false == m_draft_polys.empty() && false == m_draft_polys.back().points.empty())
	{
		auto const &pts{m_draft_polys.back().points};
		OutX = pts.back().x;
		OutY = pts.back().y;
		if (pts.size() >= 2)
		{
			OutAz = std::atan2(pts.back().x - pts[pts.size() - 2].x, pts.back().y - pts[pts.size() - 2].y);
		}
		return true;
	}
	OutX = m_draft.x0;
	OutY = m_draft.y0;
	return true;
}

bool plan_panel::finish_draft()
{
	auto *niweleta{current_niweleta()};
	if (niweleta == nullptr)
	{
		m_status = "brak niwelety";
		return false;
	}
	auto const result{maj0sted::app::finish_basket_draft(m_draft, *niweleta, m_document.niwelety, static_cast<int>(m_niweleta), m_draft_finish_straight)};
	m_status = result.status;
	if (result.ok)
	{
		m_draft_polys.clear();
		m_draft_place = 0;
		m_anchor_pick = 0;
		m_dragging_draft = false;
		solve();
	}
	else
	{
		rebuild_draft_preview();
		solve();
	}
	return result.ok;
}


void plan_panel::rebuild_draft_preview()
{
	m_draft_polys = maj0sted::app::preview_basket_draft(m_draft, m_draft_r);
}


bool plan_panel::hit_draft_station(ImVec2 const &Mouse, int &OutKind, int &OutArc) const
{
	if (false == m_draft.active)
	{
		return false;
	}
	auto best{14.0f};
	bool found{false};
	for (auto const &h : draft_station_handles(m_draft, m_draft_polys))
	{
		ImVec2 screen;
		if (false == world_to_screen(plan_to_world(h.x, h.y), screen))
		{
			continue;
		}
		auto const d{std::hypot(screen.x - Mouse.x, screen.y - Mouse.y)};
		if (d <= best)
		{
			best = d;
			OutKind = h.kind;
			OutArc = h.arc;
			found = true;
		}
	}
	return found;
}

void plan_panel::drag_draft_station(double const X, double const Y)
{
	if (false == m_draft.active || m_draft.arcs.empty())
	{
		return;
	}
	auto const samples{sample_draft_centreline(m_draft_polys)};
	double station{0.0};
	double px{0.0};
	double py{0.0};
	if (false == project_on_gap(samples, X, Y, station, px, py))
	{
		return;
	}
	auto const total{samples.empty() ? 0.0 : samples.back().along};

	auto station_before_arc = [&](int const Arc) {
		auto s{std::max(0.0, m_draft.entry_t)};
		for (int i = 0; i < Arc && i < static_cast<int>(m_draft.arcs.size()); ++i)
		{
			s += std::max(0.0, m_draft.arcs[static_cast<std::size_t>(i)].length);
			if (i + 1 < static_cast<int>(m_draft.arcs.size()))
			{
				s += std::max(0.0, m_draft.arcs[static_cast<std::size_t>(i)].transition_to_next);
			}
		}
		return s;
	};

	constexpr double kMinL = 5.0;
	constexpr double kMinKp = 0.0;

	if (m_draft_kind == 0)
	{
		m_draft.entry_t = std::clamp(station, 0.0, std::max(0.0, total - 20.0));
	}
	else if (m_draft_kind == 1 && m_draft_arc >= 0 && m_draft_arc < static_cast<int>(m_draft.arcs.size()))
	{
		auto const s0{station_before_arc(m_draft_arc)};
		auto &arc{m_draft.arcs[static_cast<std::size_t>(m_draft_arc)]};
		if (ImGui::GetIO().KeyShift && m_draft_arc + 1 < static_cast<int>(m_draft.arcs.size()))
		{
			auto const arc_end{s0 + std::max(kMinL, arc.length)};
			arc.transition_to_next = std::max(kMinKp, station - arc_end);
		}
		else
		{
			arc.length = std::max(kMinL, station - s0);
		}
	}
	else if (m_draft_kind == 2 && m_draft_arc >= 0 && m_draft_arc + 1 < static_cast<int>(m_draft.arcs.size()))
	{
		auto const s0{station_before_arc(m_draft_arc) + std::max(0.0, m_draft.arcs[static_cast<std::size_t>(m_draft_arc)].length)};
		m_draft.arcs[static_cast<std::size_t>(m_draft_arc)].transition_to_next = std::max(kMinKp, station - s0);
	}
	else if (m_draft_kind == 3)
	{
		m_draft.exit_t = std::clamp(total - station, 0.0, std::max(0.0, total - 20.0));
	}

	rebuild_draft_preview();
}

bool plan_panel::anchor_draft_to_gap(std::size_t const Gap)
{
	auto *niweleta{current_niweleta()};
	if (niweleta == nullptr)
	{
		return false;
	}
	if (false == maj0sted::app::anchor_basket_draft(m_draft, *niweleta, Gap))
	{
		m_status = "nie da się (kierunek / L+KP)";
		rebuild_draft_preview();
		solve();
		return false;
	}
	m_draft_polys.clear();
	m_draft_place = 0;
	m_anchor_pick = 0;
	m_dragging_draft = false;
	m_sel_gap = static_cast<int>(Gap);
	solve();
	m_status = "luka " + std::to_string(Gap);
	return true;
}


void plan_panel::drag_compound_station(double const X, double const Y)
{
	auto *niweleta{current_niweleta()};
	if (niweleta == nullptr || m_cmp_gap < 0 || static_cast<std::size_t>(m_cmp_gap) >= niweleta->fits.size() || m_niweleta >= m_solved.size())
	{
		return;
	}
	auto &fit{niweleta->fits[static_cast<std::size_t>(m_cmp_gap)]};
	if (fit.mode != 3 || fit.arcs.empty())
	{
		return;
	}

	auto const samples{sample_gap_centreline(m_solved[m_niweleta], m_cmp_gap)};
	double station{0.0};
	double px{0.0};
	double py{0.0};
	if (false == project_on_gap(samples, X, Y, station, px, py))
	{
		return;
	}
	auto const total{samples.empty() ? 0.0 : samples.back().along};

	if (fit.arcs.empty())
	{
		fit.arcs.push_back({300.0, 0.0, 0.0});
	}
	for (std::size_t i = 0; i + 1 < fit.arcs.size(); ++i)
	{
		if (!(fit.arcs[i].length > 0.0))
		{
			fit.arcs[i].length = 40.0;
		}
	}

	auto station_before_arc = [&](int const Arc) {
		auto s{std::max(0.0, fit.entry_t)};
		for (int i = 0; i < Arc && i + 1 < static_cast<int>(fit.arcs.size()); ++i)
		{
			s += std::max(0.0, fit.arcs[static_cast<std::size_t>(i)].length);
			s += std::max(0.0, fit.arcs[static_cast<std::size_t>(i)].transition_to_next);
		}
		return s;
	};

	constexpr double kMinL = 5.0;
	constexpr double kMinKp = 0.0;

	if (m_cmp_kind == 0)
	{
		// entry KP end: station from the start of the curve
		fit.entry_t = std::clamp(station, 0.0, std::max(0.0, total - 20.0));
	}
	else if (m_cmp_kind == 1 && m_cmp_arc >= 0 && m_cmp_arc + 1 < static_cast<int>(fit.arcs.size()))
	{
		auto const s0{station_before_arc(m_cmp_arc)};
		auto &arc{fit.arcs[static_cast<std::size_t>(m_cmp_arc)]};
		if (ImGui::GetIO().KeyShift)
		{
			// Shift: pull a between-KP out past the arc end (L stays)
			auto const arc_end{s0 + std::max(kMinL, arc.length)};
			arc.transition_to_next = std::max(kMinKp, station - arc_end);
		}
		else
		{
			arc.length = std::max(kMinL, station - s0);
		}
	}
	else if (m_cmp_kind == 2 && m_cmp_arc >= 0 && m_cmp_arc + 1 < static_cast<int>(fit.arcs.size()))
	{
		auto const s0{station_before_arc(m_cmp_arc) + std::max(0.0, fit.arcs[static_cast<std::size_t>(m_cmp_arc)].length)};
		fit.arcs[static_cast<std::size_t>(m_cmp_arc)].transition_to_next = std::max(kMinKp, station - s0);
	}
	else if (m_cmp_kind == 3)
	{
		fit.exit_t = std::clamp(total - station, 0.0, std::max(0.0, total - 20.0));
	}

	solve();
}

bool plan_panel::hit_endpoint(ImVec2 const &Mouse, int &OutNiw, int &OutStr, int &OutEnd) const
{
	auto best{12.0f};
	bool found{false};
	auto const niw{m_niweleta};
	auto const *niweleta{current_niweleta()};
	if (niweleta == nullptr)
	{
		return false;
	}
	for (std::size_t i = 0; i < niweleta->straights.size(); ++i)
	{
		auto const *st{handle_straight(niw, i)};
		if (st == nullptr || st->hidden)
		{
			continue;
		}
		for (int end = 0; end < 2; ++end)
		{
			ImVec2 screen;
			if (false == world_to_screen(plan_to_world(end == 0 ? st->x1 : st->x2, end == 0 ? st->y1 : st->y2), screen))
			{
				continue;
			}
			auto const d{std::hypot(screen.x - Mouse.x, screen.y - Mouse.y)};
			if (d <= best)
			{
				best = d;
				OutNiw = static_cast<int>(niw);
				OutStr = static_cast<int>(i);
				OutEnd = end;
				found = true;
			}
		}
	}
	return found;
}

bool plan_panel::hit_straight(ImVec2 const &Mouse, int &OutNiw, int &OutStr, bool const AllNiwelety) const
{
	auto best{10.0f};
	bool found{false};
	auto const consider = [&](std::size_t const Niw) {
		if (Niw >= m_document.niwelety.size())
		{
			return;
		}
		auto const &straights{m_document.niwelety[Niw].straights};
		for (std::size_t i = 0; i < straights.size(); ++i)
		{
			auto const *st{handle_straight(Niw, i)};
			if (st == nullptr || st->hidden)
			{
				continue;
			}
			ImVec2 a;
			ImVec2 b;
			if (false == world_to_screen(plan_to_world(st->x1, st->y1), a) || false == world_to_screen(plan_to_world(st->x2, st->y2), b))
			{
				continue;
			}
			auto const dx{b.x - a.x};
			auto const dy{b.y - a.y};
			auto const len2{dx * dx + dy * dy};
			if (len2 <= 0.0f)
			{
				continue;
			}
			auto const t{std::clamp(((Mouse.x - a.x) * dx + (Mouse.y - a.y) * dy) / len2, 0.0f, 1.0f)};
			auto const px{a.x + t * dx};
			auto const py{a.y + t * dy};
			auto const d{std::hypot(Mouse.x - px, Mouse.y - py)};
			if (d <= best)
			{
				best = d;
				OutNiw = static_cast<int>(Niw);
				OutStr = static_cast<int>(i);
				found = true;
			}
		}
	};
	if (AllNiwelety)
	{
		for (std::size_t n = 0; n < m_document.niwelety.size(); ++n)
		{
			consider(n);
		}
	}
	else
	{
		consider(m_niweleta);
	}
	return found;
}

void plan_panel::go_to_plan()
{
	auto minx{std::numeric_limits<double>::max()};
	auto miny{std::numeric_limits<double>::max()};
	auto maxx{std::numeric_limits<double>::lowest()};
	auto maxy{std::numeric_limits<double>::lowest()};

	for (auto const &niweleta : m_document.niwelety)
	{
		for (auto const &straight : niweleta.straights)
		{
			minx = std::min({minx, straight.x1, straight.x2});
			maxx = std::max({maxx, straight.x1, straight.x2});
			miny = std::min({miny, straight.y1, straight.y2});
			maxy = std::max({maxy, straight.y1, straight.y2});
		}
	}

	if (minx > maxx)
	{
		return; // nothing drawn yet, so there is nowhere to go
	}

	auto &camera{editor_mode::get_camera()};
	auto const centre{plan_to_world((minx + maxx) * 0.5, (miny + maxy) * 0.5)};
	camera.Pos.x = centre.x;
	camera.Pos.z = centre.z;

	// show the whole drawing with a little room around it
	auto const span{std::max({maxx - minx, maxy - miny, 50.0})};
	Global.editor_ortho_extent = std::clamp(static_cast<float>(span * 0.6), 5.0f, 20000.0f);
}
