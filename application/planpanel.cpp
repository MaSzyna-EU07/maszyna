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

#include "utilities/Globals.h"

namespace
{

// screen placement of the canvas together with the plan point it is centred on; rebuilt every frame
// from the panel's view state, so the transform never goes stale against a resized window
struct canvas_view
{
	ImVec2 origin{0.f, 0.f};
	ImVec2 size{0.f, 0.f};
	double viewx{0.0};
	double viewy{0.0};
	double scale{1.0};

	ImVec2 centre() const { return {origin.x + size.x * 0.5f, origin.y + size.y * 0.5f}; }

	// northing grows towards the top of the screen, so the vertical axis is flipped
	ImVec2 to_screen(double const X, double const Y) const
	{
		auto const middle{centre()};
		return {middle.x + static_cast<float>((X - viewx) * scale), middle.y - static_cast<float>((Y - viewy) * scale)};
	}

	void to_plan(ImVec2 const &Point, double &X, double &Y) const
	{
		auto const middle{centre()};
		X = viewx + (Point.x - middle.x) / scale;
		Y = viewy - (Point.y - middle.y) / scale;
	}
};

// grid spacing is picked so the cells stay roughly the same size on screen whatever the zoom, and
// always lands on a round number of metres a surveyor would recognise
double grid_step(double const Scale)
{
	auto const target{80.0 / std::max(Scale, 1e-6)}; // metres per cell at the wanted screen size
	auto const magnitude{std::pow(10.0, std::floor(std::log10(std::max(target, 1e-6))))};
	auto const steps = {1.0, 2.0, 5.0, 10.0};
	for (auto const step : steps)
	{
		if (magnitude * step >= target)
		{
			return magnitude * step;
		}
	}
	return magnitude * 10.0;
}

ImU32 element_colour(int const Kind, bool const Active)
{
	auto const alpha{Active ? 255 : 90};
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
	size_min = {520, 460};
	size_max = {2000, 1600};
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
	ImGui::Text("view: top-down, %.0f m across", Global.editor_ortho_extent * 2.0f);
	ImGui::SameLine();
	ImGui::TextDisabled("(wheel over the scenery zooms it)");

	render_toolbar();
	render_canvas();
	render_gaps();
	render_storage();
}

void plan_panel::render_toolbar()
{
	auto *niweleta{current_niweleta()};

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
	if (ImGui::Button("Frame all"))
	{
		frame_all();
	}
	ImGui::SameLine();
	ImGui::Checkbox("Draw", &m_drawing);

	if (m_document.niwelety.size() > 1)
	{
		auto const preview{m_document.niwelety[m_niweleta].name};
		ImGui::SetNextItemWidth(220.0f);
		if (ImGui::BeginCombo("Edited", preview.c_str()))
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
	ImGui::Separator();
}

void plan_panel::render_canvas()
{
	auto const available{ImGui::GetContentRegionAvail()};
	// the tables below the canvas need room of their own, but the canvas takes whatever is left
	ImVec2 const canvassize{std::max(available.x, 240.0f), std::max(available.y - 210.0f, 200.0f)};

	canvas_view view;
	view.origin = ImGui::GetCursorScreenPos();
	view.size = canvassize;
	view.viewx = m_viewx;
	view.viewy = m_viewy;
	view.scale = m_scale;

	ImGui::InvisibleButton("plan_canvas", canvassize);
	auto const hovered{ImGui::IsItemHovered()};
	auto const mouse{ImGui::GetIO().MousePos};

	double mousex{0.0};
	double mousey{0.0};
	view.to_plan(mouse, mousex, mousey);

	// zoom around the cursor, so the ground under it stays put
	if (hovered && ImGui::GetIO().MouseWheel != 0.0f)
	{
		m_scale = std::clamp(m_scale * std::pow(1.15, ImGui::GetIO().MouseWheel), 0.002, 40.0);
		view.scale = m_scale;
		auto const middle{view.centre()};
		m_viewx = mousex - (mouse.x - middle.x) / m_scale;
		m_viewy = mousey + (mouse.y - middle.y) / m_scale;
		view.viewx = m_viewx;
		view.viewy = m_viewy;
	}

	// panning latches on, so the view keeps following the cursor once it leaves the canvas
	if (hovered && ImGui::IsMouseDragging(1))
	{
		m_panning = true;
	}
	if (m_panning)
	{
		if (false == ImGui::IsMouseDragging(1))
		{
			m_panning = false;
		}
		else
		{
			auto const delta{ImGui::GetIO().MouseDelta};
			m_viewx -= delta.x / m_scale;
			m_viewy += delta.y / m_scale;
			view.viewx = m_viewx;
			view.viewy = m_viewy;
		}
	}

	// grab an existing point before laying a new one, so corners can be nudged into place
	if (hovered && ImGui::IsMouseClicked(0))
	{
		m_draggedvertex = -1;
		for (std::size_t i = 0; i < vertex_count(); ++i)
		{
			double x{0.0};
			double y{0.0};
			vertex_position(i, x, y);
			auto const point{view.to_screen(x, y)};
			if (std::abs(point.x - mouse.x) <= 6.0f && std::abs(point.y - mouse.y) <= 6.0f)
			{
				m_draggedvertex = static_cast<int>(i);
				break;
			}
		}
		if (m_draggedvertex == -1 && m_drawing)
		{
			append_vertex(mousex, mousey);
		}
	}
	if (m_draggedvertex != -1)
	{
		if (ImGui::IsMouseDown(0))
		{
			move_vertex(static_cast<std::size_t>(m_draggedvertex), mousex, mousey);
		}
		else
		{
			m_draggedvertex = -1;
		}
	}

	auto *drawlist{ImGui::GetWindowDrawList()};
	auto const canvasend{ImVec2(view.origin.x + canvassize.x, view.origin.y + canvassize.y)};
	drawlist->PushClipRect(view.origin, canvasend, true);
	drawlist->AddRectFilled(view.origin, canvasend, IM_COL32(24, 26, 30, 255));

	// grid, with the project origin picked out
	auto const step{grid_step(m_scale)};
	double left{0.0};
	double top{0.0};
	double right{0.0};
	double bottom{0.0};
	view.to_plan(view.origin, left, top);
	view.to_plan(canvasend, right, bottom);
	for (auto x = std::floor(left / step) * step; x <= right; x += step)
	{
		auto const colour{std::abs(x) < step * 0.5 ? IM_COL32(90, 90, 110, 255) : IM_COL32(48, 50, 58, 255)};
		drawlist->AddLine(view.to_screen(x, top), view.to_screen(x, bottom), colour);
	}
	for (auto y = std::floor(bottom / step) * step; y <= top; y += step)
	{
		auto const colour{std::abs(y) < step * 0.5 ? IM_COL32(90, 90, 110, 255) : IM_COL32(48, 50, 58, 255)};
		drawlist->AddLine(view.to_screen(left, y), view.to_screen(right, y), colour);
	}

	// solved geometry: two rails per centreline, coloured by the kind of element they came from
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
				points.push_back(view.to_screen(point.x, point.y));
			}
			drawlist->AddPolyline(points.data(), static_cast<int>(points.size()), element_colour(polyline.kind, active), false, active ? 2.0f : 1.0f);
		}
	}

	// editable points of the niweleta being drawn, plus the corner still waiting for its partner
	for (std::size_t i = 0; i < vertex_count(); ++i)
	{
		double x{0.0};
		double y{0.0};
		vertex_position(i, x, y);
		auto const point{view.to_screen(x, y)};
		drawlist->AddCircleFilled(point, 4.0f, static_cast<int>(i) == m_draggedvertex ? IM_COL32(255, 240, 120, 255) : IM_COL32(235, 235, 235, 255));
	}
	if (m_pending)
	{
		auto const point{view.to_screen(m_pendingx, m_pendingy)};
		drawlist->AddCircleFilled(point, 4.0f, IM_COL32(255, 240, 120, 255));
		if (hovered)
		{
			drawlist->AddLine(point, mouse, IM_COL32(255, 240, 120, 160));
		}
	}
	else if (hovered && m_drawing && m_draggedvertex == -1 && vertex_count() > 0)
	{
		double x{0.0};
		double y{0.0};
		vertex_position(vertex_count() - 1, x, y);
		drawlist->AddLine(view.to_screen(x, y), mouse, IM_COL32(255, 240, 120, 160));
	}

	drawlist->PopClipRect();

	ImGui::Text("cursor: %.1f, %.1f m   grid: %g m   scale: %.3g px/m", mousex, mousey, step, m_scale);
	ImGui::TextDisabled("left click lays a point, drag a point to move it, right drag pans, wheel zooms");
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

	ImGui::BeginChild("plan_gaps", ImVec2(0.0f, 120.0f), false);
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
	ImGui::SetNextItemWidth(280.0f);
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
			frame_all();
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

void plan_panel::frame_all()
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
		// nothing drawn yet: sit on the project origin at a scale that shows a few hundred metres
		m_viewx = 0.0;
		m_viewy = 0.0;
		m_scale = 2.0;
		return;
	}

	m_viewx = (minx + maxx) * 0.5;
	m_viewy = (miny + maxy) * 0.5;

	auto const span{std::max({maxx - minx, maxy - miny, 50.0})};
	m_scale = std::clamp(400.0 / span, 0.002, 40.0);
}
