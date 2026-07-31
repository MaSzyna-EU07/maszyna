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
#include "rendering/renderer.h"
#include "utilities/Globals.h"

namespace
{

// the plan works in the project's cartesian frame, the scenery in the engine's world space. with no
// georeference set the two share an origin, so the mapping is just the naming of the axes: easting
// runs along world x, northing against world z
glm::dvec3 plan_to_world(double const X, double const Y)
{
	return {X, 0.0, -Y};
}

void world_to_plan(glm::dvec3 const &World, double &X, double &Y)
{
	X = World.x;
	Y = -World.z;
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
	ImGui::Separator();

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
