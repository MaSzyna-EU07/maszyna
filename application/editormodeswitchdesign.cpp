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
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorSwitchDesign.hpp"

#include "utilities/translation.h"

#include "imgui/imgui.h"
#include <algorithm>
#include <cmath>

std::string switch_name_translated(std::string Title);

namespace
{

enum class part_kind
{
	straight,
	arc,
	transition
};

part_kind kind_of(switch_design::part const &Part)
{
	if (Part.radius1 == 0.0 && Part.radius2 == 0.0)
		return part_kind::straight;
	return Part.radius1 == Part.radius2 ? part_kind::arc : part_kind::transition;
}

void set_kind(switch_design::part &Part, part_kind const Kind)
{
	auto const radius{Part.radius2 != 0.0 ? Part.radius2 : Part.radius1 != 0.0 ? Part.radius1 : 300.0};
	switch (Kind)
	{
	case part_kind::straight:
		Part.radius1 = Part.radius2 = 0.0;
		break;
	case part_kind::arc:
		Part.radius1 = Part.radius2 = radius;
		break;
	case part_kind::transition:
		Part.radius1 = 0.0;
		Part.radius2 = radius;
		break;
	}
}

// which value of the template the mouse is over, for the drawing to show it
namespace value
{
int constexpr radius{1}, ratio{2}, a{3}, b{4}, lead{5}, entry{6}, tip{7}, heel{8}, blade_throw{9}, main_track{10}, stagger{11}, part{100};
}

void mark(int &Pointing, int const Value)
{
	if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled) || ImGui::IsItemActive())
		Pointing = Value;
}

// the parts of the diverging track as a table; true when something changed
bool edit_parts(std::vector<switch_design::part> &Parts, char const *Id, bool const Editable, int &Pointing, int const Pointed)
{
	bool changed{false};
	char const *kinds[] = {STR_C("Straight"), STR_C("Arc"), STR_C("Transition")};
	ImGui::PushID(Id);
	if (false == Editable)
		ImGui::BeginDisabled();
	int erase{-1};
	int up{-1};
	for (int i = 0; i < static_cast<int>(Parts.size()); ++i)
	{
		auto &part{Parts[i]};
		ImGui::PushID(i);
		ImGui::AlignTextToFramePadding();
		ImGui::Text("%d.", i + 1);
		mark(Pointing, Pointed + i);
		ImGui::SameLine();
		auto kind{static_cast<int>(kind_of(part))};
		ImGui::SetNextItemWidth(130.0f);
		if (ImGui::Combo("##kind", &kind, kinds, 3))
		{
			set_kind(part, static_cast<part_kind>(kind));
			changed = true;
		}
		mark(Pointing, Pointed + i);
		ImGui::SameLine();
		ImGui::SetNextItemWidth(90.0f);
		changed |= ImGui::InputDouble(STR_C("m##length"), &part.length, 0.0, 0.0, "%.3f");
		mark(Pointing, Pointed + i);
		item_tooltip("Length of the part, measured along it");
		switch (static_cast<part_kind>(kind))
		{
		case part_kind::arc:
			ImGui::SameLine();
			ImGui::SetNextItemWidth(90.0f);
			if (ImGui::InputDouble(STR_C("R##radius"), &part.radius1, 0.0, 0.0, "%.1f"))
			{
				part.radius2 = part.radius1;
				changed = true;
			}
			mark(Pointing, Pointed + i);
			item_tooltip("Radius of the arc; negative turns back towards the main track");
			if (part.radius1 != 0.0)
			{
				ImGui::SameLine();
				ImGui::TextDisabled("%.3f°", glm::degrees(part.length / part.radius1));
			}
			break;
		case part_kind::transition:
			ImGui::SameLine();
			ImGui::SetNextItemWidth(90.0f);
			changed |= ImGui::InputDouble(STR_C("R from##radius1"), &part.radius1, 0.0, 0.0, "%.1f");
			item_tooltip("Radius at the start of the transition curve, 0: from a straight");
			ImGui::SameLine();
			ImGui::SetNextItemWidth(90.0f);
			changed |= ImGui::InputDouble(STR_C("to##radius2"), &part.radius2, 0.0, 0.0, "%.1f");
			item_tooltip("Radius at the end of the transition curve, 0: into a straight; the curvature changes evenly along it");
			break;
		default:
			break;
		}
		if (Editable)
		{
			ImGui::SameLine();
			if (ImGui::SmallButton("^") && i > 0)
				up = i;
			item_tooltip("Move the part up");
			ImGui::SameLine();
			if (ImGui::SmallButton("x"))
				erase = i;
			item_tooltip("Remove the part");
		}
		ImGui::PopID();
	}
	if (up > 0)
	{
		std::swap(Parts[up], Parts[up - 1]);
		changed = true;
	}
	if (erase >= 0)
	{
		Parts.erase(Parts.begin() + erase);
		changed = true;
	}
	if (Editable)
	{
		auto const radius{Parts.empty() ? 300.0 : std::max({std::abs(Parts.back().radius1), std::abs(Parts.back().radius2), 0.0})};
		if (ImGui::SmallButton(STR_C("+ Straight")))
		{
			Parts.push_back({5.0, 0.0, 0.0});
			changed = true;
		}
		ImGui::SameLine();
		if (ImGui::SmallButton(STR_C("+ Arc")))
		{
			auto const r{radius > 0.0 ? radius : 300.0};
			Parts.push_back({20.0, r, r});
			changed = true;
		}
		ImGui::SameLine();
		if (ImGui::SmallButton(STR_C("+ Transition curve")))
		{
			Parts.push_back({10.0, 0.0, radius > 0.0 ? radius : 300.0});
			changed = true;
		}
	}
	if (false == Editable)
		ImGui::EndDisabled();
	ImGui::PopID();
	return changed;
}

// the switch in its own space with its dimensions, the one of the value pointed at in yellow: the main track along the width of the box
void draw_schematic(switch_design::design const &Entry, switch_design::shape const &Shape, int const Pointed, bool const Stretched)
{
	if (Shape.paths.empty() || Shape.length <= 0.0)
		return;
	float constexpr kRow{17.0f};
	float constexpr kMargin{14.0f};
	auto const rows{2 + (Entry.lead > 0.0 ? 1 : 0) + (Shape.tip > 0.0 ? 1 : 0) + (Entry.heel > 0.0 ? 1 : 0) + (Shape.stagger > 0.0 ? 1 : 0)};
	auto const height{130.0f + rows * kRow};
	auto const width{ImGui::GetContentRegionAvail().x};
	ImGui::BeginChild("##switchshape", ImVec2(0.0f, height), true, ImGuiWindowFlags_NoScrollbar);
	auto *drawlist{ImGui::GetWindowDrawList()};
	auto const origin{ImGui::GetCursorScreenPos()};
	double reach{0.0};
	double across{0.0};
	for (auto const &path : Shape.paths)
	{
		reach = std::max(reach, path.points[segment_data::point::end].z);
		across = std::max(across, std::abs(path.points[segment_data::point::end].x));
	}
	auto const scale{(width - kMargin * 2.0f - 50.0f) / static_cast<float>(std::max(1.0, reach))};
	auto const base{origin.y + height - rows * kRow - 22.0f};
	auto const stretch{Stretched ? std::clamp((base - origin.y - 28.0f) / std::max(0.01f, static_cast<float>(across) * scale), 1.0f, 12.0f) : 1.0f};
	auto const screen = [&](glm::dvec3 const &Point) { return ImVec2{origin.x + kMargin + static_cast<float>(Point.z) * scale, base - static_cast<float>(Point.x) * scale * stretch}; };
	ImU32 constexpr kActive{IM_COL32(255, 225, 60, 255)};
	ImU32 constexpr kPlain{IM_COL32(200, 205, 215, 235)};
	auto const shade = [&](int const Value) { return Pointed == Value ? kActive : kPlain; };
	auto const label = [&](ImVec2 const &At, std::string const &Text, ImU32 const Colour) {
		auto const size{ImGui::CalcTextSize(Text.c_str())};
		ImVec2 const corner{std::clamp(At.x - size.x * 0.5f, origin.x + 2.0f, std::max(origin.x + 2.0f, origin.x + width - size.x - 12.0f)), At.y - size.y * 0.5f};
		drawlist->AddRectFilled({corner.x - 2.0f, corner.y}, {corner.x + size.x + 2.0f, corner.y + size.y}, IM_COL32(18, 22, 28, 235), 2.0f);
		drawlist->AddText(corner, Colour, Text.c_str());
	};
	ImU32 const colours[] = {IM_COL32(200, 200, 200, 255), IM_COL32(90, 200, 255, 255), IM_COL32(200, 200, 200, 255), IM_COL32(255, 170, 60, 255)};
	// the diverging track sampled along its length, for the parts of it
	struct sample
	{
		ImVec2 at;
		double along;
	};
	std::vector<sample> diverging;
	for (std::size_t i = 0; i < Shape.paths.size(); ++i)
	{
		auto const &path{Shape.paths[i]};
		std::vector<segment_data> pieces{path.pieces};
		if (pieces.empty())
			pieces.push_back(path);
		auto along{0.0};
		for (auto const &piece : pieces)
		{
			geometry::bezier const curve{piece};
			auto last{curve.point(0.0)};
			auto previous{screen(last)};
			if (i == 1 && diverging.empty())
				diverging.push_back({previous, 0.0});
			for (int j = 1; j <= 24; ++j)
			{
				auto const point{curve.point(j / 24.0)};
				auto const next{screen(point)};
				drawlist->AddLine(previous, next, colours[i % 4], 2.0f);
				along += glm::distance(point, last);
				if (i == 1)
					diverging.push_back({next, along});
				last = point;
				previous = next;
			}
		}
	}
	if (diverging.empty())
	{
		ImGui::EndChild();
		return;
	}
	// the part of the diverging track pointed at, and where the first arc lies
	auto from{Entry.lead};
	auto arc_at{-1.0};
	auto arc_radius{0.0};
	auto arc_part{-1};
	for (int i = 0; i < static_cast<int>(Entry.parts.size()); ++i)
	{
		auto const &part{Entry.parts[i]};
		if (arc_part < 0 && part.radius1 != 0.0 && part.radius1 == part.radius2)
		{
			arc_at = from + part.length * 0.5;
			arc_radius = part.radius1;
			arc_part = i;
		}
		if (Pointed == value::part + i)
			for (std::size_t k = 1; k < diverging.size(); ++k)
				if (diverging[k].along >= from && diverging[k - 1].along <= from + part.length)
					drawlist->AddLine(diverging[k - 1].at, diverging[k].at, kActive, 4.0f);
		from += part.length;
	}
	auto const start{screen({0.0, 0.0, 0.0})};
	auto const end{diverging.back().at};
	drawlist->AddCircleFilled(start, 4.0f, IM_COL32(255, 255, 255, 255));
	// the tangents meeting at the centre of the switch
	if (Shape.a > 0.0)
	{
		auto const centre{screen({0.0, 0.0, Shape.a})};
		drawlist->AddLine(centre, end, IM_COL32(255, 255, 255, 110), 1.0f);
		drawlist->AddCircle(centre, 4.0f, IM_COL32(255, 255, 255, 220), 0, 1.5f);
	}
	if (arc_at >= 0.0)
	{
		auto const near_arc{std::lower_bound(diverging.begin(), diverging.end(), arc_at, [](sample const &Sample, double const Along) { return Sample.along < Along; })};
		auto const at{near_arc != diverging.end() ? near_arc->at : end};
		auto const active{Pointed == value::radius || Pointed == value::part + arc_part};
		ImVec2 const text{at.x - 30.0f, at.y - 30.0f};
		drawlist->AddLine(at, text, active ? kActive : IM_COL32(200, 205, 215, 140), 1.0f);
		label(text, format("R %.0f", std::abs(arc_radius)), active ? kActive : kPlain);
	}
	if (Shape.ratio > 0.0)
		label({end.x + 8.0f, end.y - 14.0f}, format("1:%.2f", Shape.ratio), shade(value::ratio));
	// how far the end of the diverging track lies from the main one
	{
		ImVec2 const foot{end.x, base};
		drawlist->AddLine(end, foot, IM_COL32(200, 205, 215, 90), 1.0f);
		label({end.x + 26.0f, (end.y + base) * 0.5f}, format("%.2f m", Shape.offset), IM_COL32(200, 205, 215, 170));
	}
	auto row{0};
	auto const dimension = [&](double const First, double const Second, int const Row, std::string const &Text, int const Value) {
		auto const y{base + 16.0f + Row * kRow};
		auto const left{screen({0.0, 0.0, First}).x};
		auto const right{screen({0.0, 0.0, Second}).x};
		auto const colour{shade(Value)};
		drawlist->AddLine({left, y}, {right, y}, colour, Pointed == Value ? 2.0f : 1.0f);
		for (auto const x : {left, right})
		{
			drawlist->AddLine({x, y - 3.0f}, {x, y + 3.0f}, colour, 1.0f);
			drawlist->AddLine({x, base + 2.0f}, {x, y}, Pointed == Value ? kActive : IM_COL32(200, 205, 215, 45), 1.0f);
		}
		label({(left + right) * 0.5f, y}, Text, colour);
	};
	if (Shape.a > 0.0)
		dimension(0.0, Shape.a, row, format("a %.3f", Shape.a), value::a);
	if (Shape.b > 0.0)
		dimension(Shape.a, Shape.a + Shape.b, row, format("b %.3f", Shape.b), value::b);
	++row;
	if (Entry.lead > 0.0)
		dimension(0.0, Entry.lead, row++, format(STR_C("pre-blade %.3f"), Entry.lead), value::lead);
	if (Shape.tip > 0.0)
	{
		auto const at{screen({0.0, 0.0, Shape.tip})};
		drawlist->AddTriangleFilled({at.x, at.y}, {at.x - 4.0f, at.y + 8.0f}, {at.x + 4.0f, at.y + 8.0f}, IM_COL32(120, 255, 120, 220));
		dimension(0.0, Shape.tip, row++, format(STR_C("blade tip %.3f"), Shape.tip), value::tip);
		if (Entry.entry > 0.0 || Pointed == value::entry)
			label({at.x, at.y - 22.0f}, format(STR_C("blade angle %.3f deg"), Entry.entry), shade(value::entry));
		if (Pointed == value::blade_throw)
		{
			drawlist->AddLine({at.x, at.y - 18.0f}, {at.x, at.y}, kActive, 2.0f);
			label({at.x, at.y - 22.0f}, format(STR_C("blade throw %.3f"), Entry.bladethrow > 0.0 ? Entry.bladethrow : 0.160), kActive);
		}
	}
	if (Entry.heel > 0.0)
		dimension(0.0, Entry.heel, row++, format(STR_C("blade heel %.3f"), Entry.heel), value::heel);
	if (Shape.stagger > 0.0)
	{
		drawlist->AddCircleFilled(screen({0.0, 0.0, Shape.stagger}), 4.0f, IM_COL32(255, 170, 60, 255));
		dimension(0.0, Shape.stagger, row++, format(STR_C("second switch %.3f"), Shape.stagger), value::stagger);
	}
	dimension(0.0, Shape.length, row++, format(STR_C("main track %.3f"), Shape.length), value::main_track);
	if (stretch > 1.01f)
		drawlist->AddText({origin.x + 4.0f, origin.y + 2.0f}, IM_COL32(255, 255, 255, 120), format(STR_C("across x%.1f"), stretch).c_str());
	ImGui::EndChild();
}

} // namespace

void editor_mode::switch_designs_load()
{
	auto &designer{m_switch_designer};
	if (designer.loaded)
		return;
	designer.loaded = true;
	designer.catalogue = switch_design::catalogue();
	designer.own = switch_design::own();
}

void editor_mode::switch_templates_refresh(std::string const &Arm)
{
	auto &tool{m_switch};
	std::vector<editor_track::switch_template> scenery;
	for (auto const &entry : tool.templates)
		if (entry.source != nullptr)
			scenery.push_back(entry);
	auto const armed{false == Arm.empty() ? Arm : tool.armed >= 0 && tool.armed < static_cast<int>(tool.templates.size()) ? tool.templates[tool.armed].label : std::string{}};
	tool.templates = editor_track::standard_switch_templates();
	tool.templates.insert(tool.templates.end(), scenery.begin(), scenery.end());
	tool.collected = true;
	for (int i = 0; i < static_cast<int>(tool.templates.size()); ++i)
		if (false == armed.empty() && tool.templates[i].label.rfind(armed, 0) == 0)
		{
			tool.last = i;
			if (tool.armed >= 0)
				tool.armed = i;
			break;
		}
	m_turnout_template = 0;
}

void editor_mode::switch_designer_click(glm::dvec3 const &Point)
{
	m_switch_designer.clicks.push_back(Point);
	switch_designer_measure();
}

// what the points clicked so far make: the shape and the radius keeping closest to the points along the diverging track; too few of them:
// the shape chosen at the radius given
void editor_mode::switch_designer_measure()
{
	auto &designer{m_switch_designer};
	designer.measured_ready = false;
	designer.measured_text.clear();
	if (designer.clicks.size() < 4)
		return;
	std::vector<glm::dvec2> plan;
	for (auto const &click : designer.clicks)
		plan.push_back(geometry::plan_of(click));
	switch_design::measured found;
	std::string problem;
	if (false == switch_design::measure_clicks(plan, found, problem))
	{
		designer.measured_text = STR(problem);
		return;
	}
	switch_design::design entry;
	entry.own = true;
	entry.rail = "S42";
	if (plan.size() >= 7)
	{
		switch_design::clicks_fit fitted;
		char const *const shapes[] = {STR_C("straights"), STR_C("transition curve"), STR_C("two arcs"), STR_C("arc, straight, arc")};
		if (switch_design::fit_clicks(entry, plan, static_cast<switch_design::tangent_fit>(designer.fit_kind), designer.quick_first, fitted, problem))
		{
			designer.quick_radius = std::round(fitted.radius);
			found.radius = 0.0;
			designer.fit_kind = static_cast<int>(fitted.kind);
			designer.measured_text = format(STR_C("Along the points clicked: %s, R %.0f m, off them %.2f m at most"), shapes[static_cast<int>(fitted.kind)], fitted.radius, fitted.worst);
		}
	}
	if (entry.parts.empty())
	{
		if (false == switch_design::fit_tangents(entry, found.radius > 0.0 ? found.radius : designer.quick_radius, found.ratio, found.a, found.b, static_cast<switch_design::tangent_fit>(designer.fit_kind), problem, designer.quick_first))
		{
			designer.measured_text = format(STR_C("Measured a %.2f m, b %.2f m, 1:%.2f: %s"), found.a, found.b, found.ratio, STR(problem).c_str());
			return;
		}
	}
	entry.a = found.a;
	entry.b = found.main - found.a;
	entry.main = found.main;
	if (std::abs(found.past) > 0.5)
		designer.measured_text += (designer.measured_text.empty() ? "" : "\n") + format(STR_C("The end of the diverging track lies %+.2f m along the main track from its end: check points 2 and 3"), found.past);
	designer.quick_a = found.a;
	designer.quick_b = found.b;
	designer.quick_ratio = found.ratio;
	if (found.radius > 0.0)
		designer.quick_radius = found.radius;
	designer.measured = entry;
	designer.measured_ready = true;
}

void editor_mode::switch_designer_finish()
{
	auto &designer{m_switch_designer};
	switch_designs_load();
	switch_designer_measure();
	if (false == designer.measured_ready)
	{
		designer.status = designer.measured_text;
		return;
	}
	auto entry{designer.measured};
	entry.name = format("%s %zu", STR_C("Switch"), designer.own.size() + 1);
	designer.own.push_back(entry);
	designer.from_catalogue = false;
	designer.selected = static_cast<int>(designer.own.size()) - 1;
	designer.dirty = true;
	designer.picking = false;
	first_step_done(first_step::template_clicked);
	designer.status = format(STR_C("Measured a %.2f m, b %.2f m, 1:%.2f"), designer.quick_a, designer.quick_b, designer.quick_ratio);
	if (false == designer.measured_text.empty())
		designer.status += "\n" + designer.measured_text;
}

void editor_mode::draw_switch_designer_clicks() const
{
	auto const &designer{m_switch_designer};
	if (designer.open && designer.picking)
		draw_cursor_ring(designer.clicks.empty() ? cursor_ground() : cursor_level(designer.clicks.front().y));
	if (false == designer.open || designer.clicks.empty())
		return;
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
	auto const &clicks{designer.clicks};
	if (clicks.size() >= 2)
		projection.line(drawlist, clicks[0], clicks[1], IM_COL32(230, 230, 230, 230), 2.0f);
	if (clicks.size() >= 4)
		projection.line(drawlist, clicks[2], clicks[3], IM_COL32(90, 200, 255, 230), 2.0f);
	// the template laid where it was clicked: the one measured so far while clicking, then the one chosen on the list
	if (clicks.size() >= 3 && designer.on_view)
	{
		switch_design::design const *shown{nullptr};
		std::string problem;
		auto ready{false};
		if (designer.picking)
		{
			shown = &designer.measured;
			ready = designer.measured_ready;
		}
		else
		{
			auto const &list{designer.from_catalogue ? designer.catalogue : designer.own};
			if (designer.selected >= 0 && designer.selected < static_cast<int>(list.size()))
			{
				shown = &list[designer.selected];
				ready = true;
			}
		}
		auto const shape{ready ? switch_design::build(*shown, problem) : switch_design::shape{}};
		auto const way{geometry::plan_of(clicks[1] - clicks[0])};
		if (false == shape.paths.empty() && glm::dot(way, way) > 1e-6)
		{
			auto const main{glm::normalize(way)};
			auto const beside{geometry::plan_of(clicks[2] - clicks[0])};
			auto const side{main.x * beside.y - main.y * beside.x < 0.0 ? -1.0 : 1.0};
			glm::dvec2 const across{-main.y * side, main.x * side};
			auto const placed = [&](glm::dvec3 const &Point) {
				auto const at{geometry::plan_of(clicks[0]) + main * Point.z + across * Point.x};
				return glm::dvec3{at.x, clicks[0].y + 0.2, at.y};
			};
			ImU32 const colours[] = {IM_COL32(255, 255, 255, 240), IM_COL32(255, 90, 200, 255), IM_COL32(255, 255, 255, 240), IM_COL32(255, 170, 60, 255)};
			for (std::size_t i = 0; i < shape.paths.size(); ++i)
			{
				std::vector<segment_data> pieces{shape.paths[i].pieces};
				if (pieces.empty())
					pieces.push_back(shape.paths[i]);
				for (auto const &piece : pieces)
				{
					geometry::bezier const curve{piece};
					auto previous{placed(curve.point(0.0))};
					for (int j = 1; j <= 16; ++j)
					{
						auto const next{placed(curve.point(j / 16.0))};
						projection.line(drawlist, previous, next, colours[i % 4], 2.0f);
						previous = next;
					}
				}
			}
			auto const mark_at = [&](double const Along, std::string const &Text) {
				ImVec2 at;
				if (projection.project(placed({0.0, 0.0, Along}), at))
				{
					drawlist->AddCircle(at, 5.0f, IM_COL32(255, 255, 255, 240), 0, 1.5f);
					if (false == Text.empty())
						drawlist->AddText({at.x + 7.0f, at.y + 4.0f}, IM_COL32(255, 255, 255, 240), Text.c_str());
				}
			};
			if (shape.a > 0.0)
				mark_at(shape.a, format("a %.2f", shape.a));
			mark_at(shape.length, format(STR_C("length %.2f m"), shape.length));
		}
	}
	for (std::size_t i = 0; i < clicks.size(); ++i)
	{
		ImVec2 at;
		if (false == projection.project(clicks[i], at))
			continue;
		drawlist->AddCircleFilled(at, 4.0f, i < 4 ? IM_COL32(255, 225, 60, 255) : IM_COL32(120, 255, 120, 255));
		drawlist->AddText({at.x + 6.0f, at.y - 16.0f}, IM_COL32(255, 255, 255, 255), std::to_string(i + 1).c_str());
	}
}

void editor_mode::render_switch_designer()
{
	auto &designer{m_switch_designer};
	draw_switch_designer_clicks();
	if (false == designer.open)
		return;
	first_step_done(first_step::template_open);
	switch_designs_load();
	ImGui::SetNextWindowSize(ImVec2(820.0f, 560.0f), ImGuiCond_FirstUseEver);
	if (false == ImGui::Begin((std::string(STR_C("Switch templates")) + "###switchdesigner").c_str(), &designer.open, ImGuiWindowFlags_NoCollapse))
	{
		ImGui::End();
		return;
	}
	ImGui::BeginChild("##designlist", ImVec2(230.0f, 0.0f), true);
	ImGui::TextDisabled("%s", STR_C("Catalogue (read only)"));
	item_tooltip("The catalogue switches start tangentially at the joint before the blades: without the pre-blade stretch and the angle of the blade, which a copy can be given");
	for (int i = 0; i < static_cast<int>(designer.catalogue.size()); ++i)
		if (ImGui::Selectable((switch_name_translated(designer.catalogue[i].name) + "##catalogue" + std::to_string(i)).c_str(), designer.from_catalogue && designer.selected == i))
		{
			designer.from_catalogue = true;
			designer.selected = i;
		}
	ImGui::Separator();
	ImGui::TextDisabled("%s", STR_C("Own templates"));
	for (int i = 0; i < static_cast<int>(designer.own.size()); ++i)
		if (ImGui::Selectable((designer.own[i].name + "##own" + std::to_string(i)).c_str(), false == designer.from_catalogue && designer.selected == i))
		{
			designer.from_catalogue = false;
			designer.selected = i;
		}
	if (designer.own.empty())
		ImGui::TextDisabled("%s", STR_C("(none yet)"));
	if (ImGui::Button(STR_C("New")))
	{
		switch_design::design entry;
		entry.own = true;
		entry.name = format("%s %zu", STR_C("Switch"), designer.own.size() + 1);
		auto const angle{std::atan(1.0 / 9.0)};
		entry.parts = {{190.0 * angle, 190.0, 190.0}, {6.0, 0.0, 0.0}};
		designer.own.push_back(entry);
		designer.from_catalogue = false;
		designer.selected = static_cast<int>(designer.own.size()) - 1;
		designer.dirty = true;
	}
	item_tooltip("A new own template, an Rz 1:9 R190 to start from");
	if (ImGui::Button(designer.picking ? STR_C("Stop clicking") : STR_C("Click off the view...")))
	{
		designer.picking = false == designer.picking;
		if (designer.picking)
			designer.clicks.clear();
	}
	item_tooltip("Measures a switch off the orthophoto: click its points in the view, in the top view (O) they're best seen. The orthophoto is turned on in the layers");
	if (designer.clicks.size() >= 3)
	{
		ImGui::Checkbox(STR_C("Show on the view"), &designer.on_view);
		item_tooltip("The chosen template drawn in the view where the switch was clicked, as it changes: to be compared with the orthophoto under it");
	}
	if (designer.picking)
	{
		char const *const steps[] = {STR_C("1. The start of the switch"), STR_C("2. The end of the switch on the main track"), STR_C("3. The end of the switch on the diverging track"), STR_C("4. A point further on the diverging track, for its way"), STR_C("5. Points along the diverging track, 3 at least, as laying a curve from clicks (optional)")};
		auto const at{std::min<std::size_t>(designer.clicks.size(), 4)};
		for (std::size_t i = 0; i < 5; ++i)
			if (i == at)
				ImGui::TextColored(ImVec4(1.0f, 0.88f, 0.25f, 1.0f), "%s", steps[i]);
			else
				ImGui::TextDisabled("%s", steps[i]);
		if (at == 4)
			ImGui::TextDisabled(STR_C("On the arc: %zu"), designer.clicks.size() - 4);
		ImGui::TextDisabled("%s", STR_C("Backspace: back, Esc: stop"));
		if (false == m_orthophoto.enabled())
		{
			if (ImGui::SmallButton(STR_C("Orthophoto...")))
				m_orthophoto_window = true;
			item_tooltip("The photo to click the switch off isn't turned on");
		}
		if (ImGui::SmallButton(STR_C("Back")) && false == designer.clicks.empty())
		{
			designer.clicks.pop_back();
			switch_designer_measure();
		}
		if (false == designer.measured_text.empty())
			ImGui::TextWrapped("%s", designer.measured_text.c_str());
		ImGui::SameLine();
		if (designer.clicks.size() >= 4 && ImGui::SmallButton(STR_C("Make the template")))
			switch_designer_finish();
	}
	ImGui::EndChild();
	ImGui::SameLine();
	ImGui::BeginChild("##designedit", ImVec2(0.0f, 0.0f), false);
	auto const &list{designer.from_catalogue ? designer.catalogue : designer.own};
	if (designer.selected < 0 || designer.selected >= static_cast<int>(list.size()))
	{
		ImGui::TextWrapped("%s", STR_C("Choose a template on the left; copy a catalogue one to own templates to change it"));
		item_tooltip("A template is a straight main track and the diverging track from the start of the switch as a chain of straights, arcs and transition curves; a double switch is two switches, the second one starting on the main track of the first");
		ImGui::EndChild();
		ImGui::End();
		return;
	}
	bool const editable{false == designer.from_catalogue};
	auto &entry{designer.from_catalogue ? designer.catalogue[designer.selected] : designer.own[designer.selected]};
	bool changed{false};
	auto pointing{designer.pointed_test};
	if (false == editable)
	{
		if (ImGui::Button(STR_C("Copy to own templates")))
		{
			auto copy{entry};
			copy.own = true;
			copy.note.clear();
			copy.name = switch_name_translated(copy.name) + STR(" (copy)");
			designer.own.push_back(copy);
			designer.from_catalogue = false;
			designer.selected = static_cast<int>(designer.own.size()) - 1;
			designer.dirty = true;
			ImGui::EndChild();
			ImGui::End();
			return;
		}
		ImGui::SameLine();
		ImGui::TextDisabled("%s", STR_C("A catalogue template can't be changed"));
	}
	if (false == editable)
		ImGui::BeginDisabled();
	auto const input = [&](char const *Label, char const *Id, double &Value, char const *Format) {
		field_row(Label);
		return ImGui::InputDouble(Id, &Value, 0.0, 0.0, Format);
	};
	if (begin_fields("##designfields", 0.5f))
	{
		char name[96];
		std::snprintf(name, sizeof(name), "%s", entry.name.c_str());
		field_row("Name##designname");
		if (ImGui::InputText("##designname", name, sizeof(name)))
		{
			entry.name = name;
			changed = true;
		}
		char const *types[] = {STR_C("Rz ordinary"), STR_C("Rpd double, to both sides"), STR_C("Rpj double, to one side"), STR_C("Rps double, symmetric")};
		char const *codes[] = {"Rz", "Rpd", "Rpj", "Rps"};
		int type{entry.type == "Rpd" ? 1 : entry.type == "Rpj" ? 2 : entry.type == "Rps" ? 3 : 0};
		field_row("Type##designtype");
		if (ImGui::Combo("##designtype", &type, types, 4))
		{
			entry.type = codes[type];
			if (type > 0 && entry.stagger <= 0.0)
				entry.stagger = 5.0;
			changed = true;
		}
		item_tooltip("A double switch is laid as two switches: the first one with its diverging track and the main track up to the second one, which starts there with the other diverging track");
		ImGui::EndTable();
	}
	if (false == editable)
		ImGui::EndDisabled();

	{
		std::string problem;
		ImGui::Checkbox(STR_C("Stretch across"), &designer.stretched);
		item_tooltip("Draws the offsets from the main track larger, so the shape of the diverging track can be seen");
		// what's being clicked while it's clicked: the template chosen isn't changed by it
		auto const &drawn{designer.picking && designer.measured_ready ? designer.measured : entry};
		draw_schematic(drawn, switch_design::build(drawn, problem), designer.pointed, designer.stretched);
	}

	ImGui::Separator();
	ImGui::TextDisabled("%s", STR_C("Diverging track from the start of the switch"));
	if (editable && begin_fields("##quickfields", 0.5f))
	{
		input("R##quickradius", "##quickradius", designer.quick_radius, "%.0f");
		mark(pointing, value::radius);
		input("1:n##quickratio", "##quickratio", designer.quick_ratio, "%.2f");
		mark(pointing, value::ratio);
		input("a, m##quicka", "##quicka", designer.quick_a, "%.3f");
		mark(pointing, value::a);
		item_tooltip("From the start to the point where the tangents of both tracks meet; with a b differing the curve is made of two arcs; shorter than the tangent of the arc: the blade meets the stock rail at an angle. 0: the tangent of the arc");
		input("b, m##quickb", "##quickb", designer.quick_b, "%.3f");
		mark(pointing, value::b);
		char const *fits[] = {STR_C("straights"), STR_C("transition curve"), STR_C("two arcs"), STR_C("arc, straight, arc")};
		field_row("Rest of the tangents##quickkind");
		ImGui::Combo("##quickkind", &designer.fit_kind, fits, 4);
		item_tooltip("What makes up where a or b is longer than the tangent of the arc");
		if (designer.fit_kind == static_cast<int>(switch_design::tangent_fit::arc_straight_arc))
		{
			input("R of the first arc##quickfirst", "##quickfirst", designer.quick_first, "%.0f");
			item_tooltip("The radius of the arc before the straight, when it isn't the one after it; 0: the R");
		}
		item_tooltip("From the point where the tangents meet to the end of the switch. 0: the tangent of the arc");
		ImGui::TableNextRow();
		ImGui::TableSetColumnIndex(1);
		if (ImGui::Button(STR_C("Fill in"), ImVec2(-FLT_MIN, 0.0f)) && designer.quick_radius >= 10.0 && designer.quick_ratio > 1.0)
		{
			std::string problem;
			if (switch_design::fit_tangents(entry, designer.quick_radius, designer.quick_ratio, designer.quick_a, designer.quick_b, static_cast<switch_design::tangent_fit>(designer.fit_kind), problem, designer.quick_first))
			{
				designer.status.clear();
				changed = true;
			}
			else
				designer.status = problem;
		}
		item_tooltip("Replaces the parts with the curve from the start of the switch up to the angle 1:n fitted to a and b, R the sharpest of it: two arcs where a and b differ, else one arc with the straights they ask for");
		ImGui::EndTable();
		if (designer.quick_radius >= 10.0 && designer.quick_ratio > 1.0)
			ImGui::TextDisabled(STR_C("The tangent of the arc alone is %.3f m"), designer.quick_radius * std::tan(std::atan(1.0 / designer.quick_ratio) * 0.5));
	}
	changed |= edit_parts(entry.parts, "##parts", editable, pointing, value::part);
	bool const twofold{switch_design::double_switch(entry.type)};
	if (false == editable)
		ImGui::BeginDisabled();
	if (twofold)
	{
		ImGui::Separator();
		if (begin_fields("##staggerfields", 0.5f))
		{
			changed |= input("Second switch starts at, m##stagger", "##stagger", entry.stagger, "%.3f");
			mark(pointing, value::stagger);
			item_tooltip("Distance from the start of the first switch to the start of the second one, along the main track");
			ImGui::EndTable();
		}
		bool own{false == entry.parts2.empty() && entry.type != "Rps"};
		if (entry.type == "Rps")
			ImGui::TextDisabled("%s", STR_C("The second switch is the mirror image of the first one"));
		else if (ImGui::Checkbox(STR_C("The second switch has a diverging track of its own"), &own))
		{
			entry.parts2 = own ? entry.parts : std::vector<switch_design::part>{};
			changed = true;
		}
		if (entry.type != "Rps")
			item_tooltip("Otherwise the second switch diverges the same way as the first, to the other side for Rpd, to the same side for Rpj");
		if (own)
		{
			if (begin_fields("##design2fields", 0.5f))
			{
				auto lead2{entry.lead2 >= 0.0 ? entry.lead2 : entry.lead};
				if (input("Pre-blade stretch, m##designlead2", "##designlead2", lead2, "%.3f"))
				{
					entry.lead2 = std::max(0.0, lead2);
					changed = true;
				}
				auto entry2{entry.entry2 >= 0.0 ? entry.entry2 : entry.entry};
				if (input("Blade angle, deg##designentry2", "##designentry2", entry2, "%.4f"))
				{
					entry.entry2 = std::max(0.0, entry2);
					changed = true;
				}
				auto tip2{entry.tip2 >= 0.0 ? entry.tip2 : entry.tip};
				if (input("Tip of a tangent blade, m##designtip2", "##designtip2", tip2, "%.3f"))
				{
					entry.tip2 = std::max(0.0, tip2);
					changed = true;
				}
				ImGui::EndTable();
			}
			int unused{-1};
			changed |= edit_parts(entry.parts2, "##parts2", editable, unused, value::part);
		}
		else if (entry.lead2 >= 0.0 || entry.entry2 >= 0.0 || entry.tip2 >= 0.0)
		{
			entry.lead2 = entry.entry2 = entry.tip2 = -1.0;
			changed = true;
		}
	}
	if (false == editable)
		ImGui::EndDisabled();
	if (ImGui::TreeNode(STR_C("More settings")))
	{
		if (false == editable)
			ImGui::BeginDisabled();
		if (begin_fields("##designmorefields", 0.5f))
		{
			char rail[32];
			std::snprintf(rail, sizeof(rail), "%s", entry.rail == "-" ? "" : entry.rail.c_str());
			field_row("Rail##designrail");
			if (ImGui::InputText("##designrail", rail, sizeof(rail)))
			{
				entry.rail = rail[0] != '\0' ? std::string{rail} : std::string{"-"};
				changed = true;
			}
			item_tooltip("Rail of the switch kept in its record: 60E1, 49E1...");
			changed |= input("Pre-blade stretch, m##designlead", "##designlead", entry.lead, "%.3f");
			mark(pointing, value::lead);
			item_tooltip("From the start of the switch (the joint before the blades) to the tip of the blade; the diverging track runs along the main one there");
			changed |= input("Blade angle, deg##designentry", "##designentry", entry.entry, "%.4f");
			mark(pointing, value::entry);
			item_tooltip("The angle at which the blade meets the stock rail at its tip: the diverging track starts there at this angle instead of tangentially. Curved blades a fraction of a degree, straight blades 1 to 2 degrees; 0: tangential");
			changed |= input("Tip of a tangent blade, m##designtip", "##designtip", entry.tip, "%.3f");
			mark(pointing, value::tip);
			item_tooltip("With the blade angle 0: the tip of the blade from the start of the switch as its scheme gives it, the diverging track is broken there into pieces; 0: where the blade gets 5 mm thick, less the 125 mm its nose is planed over");
			changed |= input("Blade heel, m##designheel", "##designheel", entry.heel, "%.3f");
			mark(pointing, value::heel);
			item_tooltip("Where the moving blades end, from the start of the switch, as its scheme gives it; the switch keeps it in the scenery. 0: as long as the flangeway of 58 mm behind them needs");
			changed |= input("Blade throw, m##designthrow", "##designthrow", entry.bladethrow, "%.3f");
			mark(pointing, value::blade_throw);
			item_tooltip("How far the tips of the blades move over between the two paths; 0: the usual 0.160 m");
			ImGui::EndTable();
		}
		if (editable)
		{
			ImGui::TextDisabled("%s", STR_C("Straight blade:"));
			if (begin_fields("##bladefields", 0.5f))
			{
				input("l, m##bladelength", "##bladelength", designer.blade_length, "%.2f");
				input("z, m##bladegap", "##bladegap", designer.blade_gap, "%.3f");
				ImGui::TableNextRow();
				ImGui::TableSetColumnIndex(1);
				if (ImGui::Button(STR_C("Angle from l and z"), ImVec2(-FLT_MIN, 0.0f)) && designer.blade_length > 0.1 && designer.blade_gap > 0.0 && designer.blade_gap < designer.blade_length)
				{
					entry.entry = glm::degrees(std::asin(designer.blade_gap / designer.blade_length));
					changed = true;
				}
				item_tooltip("sin angle = z / l: l the length of a straight blade, z the gap between the running edges of the stock rail and the blade at its heel");
				ImGui::EndTable();
			}
		}
		if (begin_fields("##designmainfields", 0.5f))
		{
			changed |= input("Main track, m##designmain", "##designmain", entry.main, "%.3f");
			mark(pointing, value::main_track);
			item_tooltip("Length of the main track, 0: as far as the diverging tracks reach");
			changed |= input("a##designa", "##designa", entry.a, "%.3f");
			mark(pointing, value::a);
			item_tooltip("From the start to the point where the tangents of both tracks meet, as the catalogue gives it; 0: from the geometry");
			changed |= input("b##designb", "##designb", entry.b, "%.3f");
			mark(pointing, value::b);
			item_tooltip("From the point where the tangents meet to the end, as the catalogue gives it; 0: the main track less a");
			ImGui::EndTable();
		}
		if (false == editable)
			ImGui::EndDisabled();
		ImGui::TreePop();
	}

	std::string error;
	auto const shape{switch_design::build(entry, error)};
	ImGui::Separator();
	if (false == error.empty())
		ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.35f, 1.0f), "%s", STR(error).c_str());
	else
	{
		ImGui::Text(STR_C("Angle 1:%.3f (%.4f deg), sharpest R %.1f m, a %.3f m, b %.3f m"), shape.ratio, glm::degrees(std::atan(1.0 / shape.ratio)), shape.radius, shape.a, shape.b);
		if (ImGui::IsItemHovered())
		{
			auto const length{format(STR_C("Length %.3f m, the diverging track ends %.3f m beside the main one, in %zu pieces"), shape.length, shape.offset, std::max<std::size_t>(1, shape.paths[1].pieces.size()))};
			auto const tip{entry.entry > 0.0 ? format(STR_C("Tip of the blade %.3f m from the start, the diverging track leaves there at %.4f deg"), shape.tip, entry.entry)
			                                 : format(STR_C("Tip of the blade %.3f m from the start, the diverging track leaves tangentially"), shape.tip)};
			ImGui::SetTooltip("%s\n%s", length.c_str(), tip.c_str());
		}
	}
	designer.pointed = pointing;
	if (changed)
		designer.dirty = true;
	if (editable)
	{
		if (ImGui::Button(designer.dirty ? STR_C("Save *") : STR_C("Save")))
		{
			std::string problem;
			if (switch_design::save_own(designer.own, problem))
			{
				designer.dirty = false;
				designer.status = STR("Saved, the templates are on the list of the switch tool");
				switch_templates_refresh();
			}
			else
				designer.status = problem;
		}
		item_tooltip("Keeps the own templates in eu07_switch_templates.txt in the configuration directory of the simulator and puts them on the list of the switch tool");
		ImGui::SameLine();
		if (ImGui::Button(STR_C("Delete")))
		{
			designer.own.erase(designer.own.begin() + designer.selected);
			designer.selected = -1;
			designer.dirty = true;
		}
		item_tooltip("Removes the template; Save keeps the change");
	}
	if (error.empty())
	{
		if (editable)
			ImGui::SameLine();
		if (ImGui::Button(STR_C("Place this switch")))
		{
			if (designer.dirty && editable)
			{
				std::string problem;
				if (switch_design::save_own(designer.own, problem))
					designer.dirty = false;
				else
					designer.status = problem;
			}
			switch_templates_refresh(entry.name + "  ");
			show_track_tab(track_tab::turnout);
			arm_switch();
		}
		item_tooltip("Saves the own templates and chooses this one in the switch tool: then click a straight and drag to the side the switch goes");
	}
	if (false == designer.status.empty())
		ImGui::TextWrapped("%s", designer.status.c_str());
	ImGui::EndChild();
	ImGui::End();
}
