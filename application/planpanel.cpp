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
#include "rendering/renderer.h"
#include "utilities/Globals.h"

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

ImU32 element_colour(int const Kind, bool const Active)
{
	auto const alpha{Active ? 255 : 110};
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

} // namespace

plan_panel::plan_panel(std::string const &Name, bool const Isopen) : ui_panel(Name, Isopen)
{
	size_min = {420, 240};
	size_max = {900, 900};
	m_document.niwelety.push_back({"niweleta 1", {}, {}});
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
		auto const &covered{m_topomap.covered()};
		// the image arrives with its first row at the north edge, which is the opposite of what the
		// default texture coordinates assume, so the vertical ones are swapped
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

	m_document.niwelety.clear();
	m_document.niwelety.push_back({"niweleta 1", {}, {}});
	m_niweleta = 0;
	m_pending = false;
	m_draggedvertex = -1;
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
		m_draggedvertex = -1;
		return;
	}

	double cursorx{0.0};
	double cursory{0.0};
	world_to_plan(Global.pCamera.Pos + GfxRenderer->Mouse_Position(), cursorx, cursory);

	auto const mouse{ImGui::GetIO().MousePos};

	// grabbing an existing point comes first, so corners can be nudged into place
	if (ImGui::IsMouseClicked(0))
	{
		m_draggedvertex = -1;
		for (std::size_t i = 0; i < vertex_count(); ++i)
		{
			double x{0.0};
			double y{0.0};
			vertex_position(i, x, y);
			ImVec2 point;
			if (false == world_to_screen(plan_to_world(x, y), point))
			{
				continue;
			}
			if (std::abs(point.x - mouse.x) <= 8.0f && std::abs(point.y - mouse.y) <= 8.0f)
			{
				m_draggedvertex = static_cast<int>(i);
				break;
			}
		}
		if (m_draggedvertex == -1 && m_drawing)
		{
			append_vertex(cursorx, cursory);
		}
	}

	if (m_draggedvertex != -1)
	{
		if (ImGui::IsMouseDown(0))
		{
			move_vertex(static_cast<std::size_t>(m_draggedvertex), cursorx, cursory);
		}
		else
		{
			m_draggedvertex = -1;
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
		auto const centre{Global.pCamera.Pos};
		double planx{0.0};
		double plany{0.0};
		world_to_plan(centre, planx, plany);
		auto const reach{static_cast<double>(Global.editor_ortho_extent) * 1.4};

		for (auto const &tile : m_ortho.collect({planx - reach, plany - reach, planx + reach, plany + reach}))
		{
			ImVec2 corners[4];
			// the tile is a ground rectangle; on screen it is whatever the camera makes of it
			auto const ok = world_to_screen(plan_to_world(tile.box.min_x, tile.box.max_y), corners[0]) &&
			                world_to_screen(plan_to_world(tile.box.max_x, tile.box.max_y), corners[1]) &&
			                world_to_screen(plan_to_world(tile.box.max_x, tile.box.min_y), corners[2]) &&
			                world_to_screen(plan_to_world(tile.box.min_x, tile.box.min_y), corners[3]);
			if (false == ok)
			{
				continue;
			}
			// same north-first row order as the picker's base map, so the same swap applies
			drawlist->AddImageQuad(reinterpret_cast<ImTextureID>(static_cast<intptr_t>(tile.texture)), corners[0], corners[1], corners[2], corners[3], ImVec2(0.0f, 1.0f), ImVec2(1.0f, 1.0f),
			                       ImVec2(1.0f, 0.0f), ImVec2(0.0f, 0.0f));
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
			drawlist->AddPolyline(points.data(), static_cast<int>(points.size()), element_colour(polyline.kind, active), false, active ? 2.5f : 1.5f);
		}
	}

	// editable points of the niweleta being drawn
	for (std::size_t i = 0; i < vertex_count(); ++i)
	{
		double x{0.0};
		double y{0.0};
		vertex_position(i, x, y);
		ImVec2 screen;
		if (false == world_to_screen(plan_to_world(x, y), screen))
		{
			continue;
		}
		drawlist->AddCircleFilled(screen, 5.0f, static_cast<int>(i) == m_draggedvertex ? IM_COL32(255, 240, 120, 255) : IM_COL32(240, 240, 240, 255));
	}

	// the corner still waiting for the one that closes its straight, and the rubber band to the cursor
	auto const mouse{ImGui::GetIO().MousePos};
	auto const overscenery{false == ImGui::GetIO().WantCaptureMouse};
	if (m_pending)
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
	else if (overscenery && m_drawing && m_draggedvertex == -1 && vertex_count() > 0)
	{
		double x{0.0};
		double y{0.0};
		vertex_position(vertex_count() - 1, x, y);
		ImVec2 screen;
		if (world_to_screen(plan_to_world(x, y), screen))
		{
			drawlist->AddLine(screen, mouse, IM_COL32(255, 240, 120, 170), 1.5f);
		}
	}
}

void plan_panel::render_toolbar()
{
	auto *niweleta{current_niweleta()};

	ImGui::Text("top-down view, %.0f m across", Global.editor_ortho_extent * 2.0f);
	ImGui::TextDisabled("click the scenery to lay points, drag a point to move it, wheel zooms");
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
		if (m_showortho && m_ortho.pending() > 0)
		{
			ImGui::SameLine();
			ImGui::TextDisabled("(%d tiles on the way)", static_cast<int>(m_ortho.pending()));
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
				}
			}
			ImGui::EndCombo();
		}
	}

	ImGui::Text("straights: %d, points: %d", niweleta != nullptr ? static_cast<int>(niweleta->straights.size()) : 0, static_cast<int>(vertex_count()));
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

	ImGui::BeginChild("plan_gaps", ImVec2(0.0f, 130.0f), false);
	for (std::size_t gap = 0; gap < niweleta->fits.size(); ++gap)
	{
		auto &fit{niweleta->fits[gap]};
		ImGui::PushID(static_cast<int>(gap));

		ImGui::Text("gap %d", static_cast<int>(gap));
		ImGui::SameLine(70.0f);
		ImGui::SetNextItemWidth(150.0f);
		if (ImGui::Combo("##mode", &fit.mode, "none\0arc\0arc + clothoids\0"))
		{
			if (fit.mode != 0 && fit.radius <= 0.0)
			{
				fit.radius = 300.0; // something drawable to start from, rather than a rejected fit
			}
			solve();
		}
		if (fit.mode != 0)
		{
			ImGui::SameLine();
			ImGui::SetNextItemWidth(120.0f);
			if (ImGui::InputDouble("R", &fit.radius, 10.0, 100.0, "%.1f"))
			{
				solve();
			}
		}
		if (fit.mode == 2)
		{
			ImGui::SameLine();
			ImGui::SetNextItemWidth(120.0f);
			if (ImGui::InputDouble("L", &fit.transition, 5.0, 20.0, "%.1f"))
			{
				solve();
			}
		}
		// a fit the library could not produce is simply absent from its answer; say so rather than
		// leaving the parameters looking as though they took effect
		if (fit.mode != 0 && false == fit_applied(gap))
		{
			ImGui::SameLine();
			ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.4f, 1.0f), "does not fit");
		}

		ImGui::PopID();
	}
	ImGui::EndChild();
}

void plan_panel::render_storage()
{
	ImGui::Separator();
	ImGui::SetNextItemWidth(240.0f);
	ImGui::InputText("##path", m_path, sizeof(m_path));
	ImGui::SameLine();
	if (ImGui::Button("Save"))
	{
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
			m_niweleta = 0;
			m_pending = false;
			m_draggedvertex = -1;
			solve();
			go_to_plan();
			m_status = "loaded " + std::string(m_path);
		}
		else
		{
			m_status = "could not read " + std::string(m_path);
		}
	}
	if (false == m_status.empty())
	{
		ImGui::TextDisabled("%s", m_status.c_str());
	}
}

void plan_panel::solve()
{
	align_fits();
	m_solved = maj0sted::web::solve_project(m_document.niwelety);
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
		auto const gaps{niweleta.straights.empty() ? std::size_t{0} : niweleta.straights.size() - 1};
		std::vector<maj0sted::web::GapFit> slots(gaps);
		for (std::size_t gap = 0; gap < gaps; ++gap)
		{
			slots[gap].gap = static_cast<int>(gap);
		}
		// a document read from disk carries only the gaps that were actually fitted
		for (auto const &fit : niweleta.fits)
		{
			if (fit.gap >= 0 && static_cast<std::size_t>(fit.gap) < gaps)
			{
				slots[fit.gap] = fit;
			}
		}
		niweleta.fits = std::move(slots);
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

std::size_t plan_panel::vertex_count() const
{
	auto const *niweleta{current_niweleta()};
	return (niweleta == nullptr || niweleta->straights.empty()) ? 0 : niweleta->straights.size() + 1;
}

void plan_panel::vertex_position(std::size_t const Index, double &X, double &Y) const
{
	auto const *niweleta{current_niweleta()};
	if (niweleta == nullptr || niweleta->straights.empty() || Index >= niweleta->straights.size() + 1)
	{
		return;
	}
	if (Index == 0)
	{
		X = niweleta->straights.front().x1;
		Y = niweleta->straights.front().y1;
		return;
	}
	X = niweleta->straights[Index - 1].x2;
	Y = niweleta->straights[Index - 1].y2;
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
		auto const &last{niweleta->straights.back()};
		niweleta->straights.push_back({last.x2, last.y2, X, Y, false});
	}

	solve();
}

void plan_panel::move_vertex(std::size_t const Index, double const X, double const Y)
{
	auto *niweleta{current_niweleta()};
	if (niweleta == nullptr || Index >= niweleta->straights.size() + 1)
	{
		return;
	}

	// the point belongs to both straights meeting at it, so both ends follow it
	if (Index > 0)
	{
		niweleta->straights[Index - 1].x2 = X;
		niweleta->straights[Index - 1].y2 = Y;
	}
	if (Index < niweleta->straights.size())
	{
		niweleta->straights[Index].x1 = X;
		niweleta->straights[Index].y1 = Y;
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

	niweleta->straights.pop_back();
	solve();
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
