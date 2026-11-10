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

// the parts of the diverging track as a table; true when something changed
bool edit_parts(std::vector<switch_design::part> &Parts, char const *Id, bool const Editable)
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
		ImGui::SameLine();
		auto kind{static_cast<int>(kind_of(part))};
		ImGui::SetNextItemWidth(130.0f);
		if (ImGui::Combo("##kind", &kind, kinds, 3))
		{
			set_kind(part, static_cast<part_kind>(kind));
			changed = true;
		}
		ImGui::SameLine();
		ImGui::SetNextItemWidth(90.0f);
		changed |= ImGui::InputDouble(STR_C("m##length"), &part.length, 0.0, 0.0, "%.3f");
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

// plan of the switch in its own space, the main track along the width of the box
void draw_shape(switch_design::shape const &Shape, float const Height, bool const Stretched)
{
	auto const width{ImGui::GetContentRegionAvail().x};
	ImGui::BeginChild("##switchshape", ImVec2(0.0f, Height), true, ImGuiWindowFlags_NoScrollbar);
	auto *drawlist{ImGui::GetWindowDrawList()};
	auto const origin{ImGui::GetCursorScreenPos()};
	if (Shape.paths.empty() || Shape.length <= 0.0)
	{
		ImGui::EndChild();
		return;
	}
	double reach{0.0};
	double across{0.0};
	for (auto const &path : Shape.paths)
	{
		reach = std::max(reach, path.points[segment_data::point::end].z);
		across = std::max(across, std::abs(path.points[segment_data::point::end].x));
	}
	auto const margin{12.0f};
	auto const scale{(width - margin * 2.0f) / static_cast<float>(std::max(1.0, reach))};
	auto const stretch{Stretched ? std::min(8.0f, (Height * 0.5f - margin) / std::max(0.01f, static_cast<float>(across) * scale)) : 1.0f};
	auto const middle{origin.y + Height * 0.5f};
	auto const screen = [&](glm::dvec3 const &Point) { return ImVec2{origin.x + margin + static_cast<float>(Point.z) * scale, middle - static_cast<float>(Point.x) * scale * std::max(1.0f, stretch)}; };
	ImU32 const colours[] = {IM_COL32(200, 200, 200, 255), IM_COL32(90, 200, 255, 255), IM_COL32(200, 200, 200, 255), IM_COL32(255, 170, 60, 255)};
	for (std::size_t i = 0; i < Shape.paths.size(); ++i)
	{
		auto const &path{Shape.paths[i]};
		std::vector<segment_data> pieces{path.pieces};
		if (pieces.empty())
			pieces.push_back(path);
		for (auto const &piece : pieces)
		{
			geometry::bezier const curve{piece};
			auto previous{screen(curve.point(0.0))};
			for (int j = 1; j <= 24; ++j)
			{
				auto const next{screen(curve.point(j / 24.0))};
				drawlist->AddLine(previous, next, colours[i % 4], 2.0f);
				previous = next;
			}
			drawlist->AddCircleFilled(screen(piece.points[segment_data::point::end]), 3.0f, colours[i % 4]);
		}
	}
	drawlist->AddCircleFilled(screen({0.0, 0.0, 0.0}), 4.0f, IM_COL32(255, 255, 255, 255));
	if (Shape.stagger > 0.0)
		drawlist->AddCircleFilled(screen({0.0, 0.0, Shape.stagger}), 4.0f, IM_COL32(255, 170, 60, 255));
	for (auto const tip : {Shape.tip, Shape.stagger > 0.0 ? Shape.tip2 : 0.0})
		if (tip > 0.0)
		{
			auto const at{screen({0.0, 0.0, tip})};
			drawlist->AddTriangleFilled({at.x, at.y}, {at.x - 4.0f, at.y + 8.0f}, {at.x + 4.0f, at.y + 8.0f}, IM_COL32(120, 255, 120, 220));
		}
	if (Shape.a > 0.0)
	{
		auto const at{screen({0.0, 0.0, Shape.a})};
		drawlist->AddLine({at.x, at.y - 6.0f}, {at.x, at.y + 6.0f}, IM_COL32(255, 255, 255, 160), 1.0f);
	}
	if (Stretched && stretch > 1.01f)
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

void editor_mode::render_switch_designer()
{
	auto &designer{m_switch_designer};
	if (false == designer.open)
		return;
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

	ImGui::Separator();
	ImGui::TextDisabled("%s", STR_C("Diverging track from the start of the switch"));
	if (editable && begin_fields("##quickfields", 0.5f))
	{
		input("R##quickradius", "##quickradius", designer.quick_radius, "%.0f");
		input("1:n##quickratio", "##quickratio", designer.quick_ratio, "%.2f");
		input("straight after, m##quickstraight", "##quickstraight", designer.quick_straight, "%.3f");
		ImGui::TableNextRow();
		ImGui::TableSetColumnIndex(1);
		if (ImGui::Button(STR_C("Fill in"), ImVec2(-FLT_MIN, 0.0f)) && designer.quick_radius >= 10.0 && designer.quick_ratio > 1.0)
		{
			auto const angle{std::atan(1.0 / designer.quick_ratio)};
			entry.parts = {{designer.quick_radius * angle, designer.quick_radius, designer.quick_radius}};
			if (designer.quick_straight > 0.01)
				entry.parts.push_back({designer.quick_straight, 0.0, 0.0});
			changed = true;
		}
		item_tooltip("Replaces the parts with an arc of R up to the angle 1:n and the straight after it, the shape of an ordinary switch");
		ImGui::EndTable();
	}
	changed |= edit_parts(entry.parts, "##parts", editable);
	bool const twofold{switch_design::double_switch(entry.type)};
	if (false == editable)
		ImGui::BeginDisabled();
	if (twofold)
	{
		ImGui::Separator();
		if (begin_fields("##staggerfields", 0.5f))
		{
			changed |= input("Second switch starts at, m##stagger", "##stagger", entry.stagger, "%.3f");
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
			changed |= edit_parts(entry.parts2, "##parts2", editable);
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
			item_tooltip("From the start of the switch (the joint before the blades) to the tip of the blade; the diverging track runs along the main one there");
			changed |= input("Blade angle, deg##designentry", "##designentry", entry.entry, "%.4f");
			item_tooltip("The angle at which the blade meets the stock rail at its tip: the diverging track starts there at this angle instead of tangentially. Curved blades a fraction of a degree, straight blades 1 to 2 degrees; 0: tangential");
			changed |= input("Tip of a tangent blade, m##designtip", "##designtip", entry.tip, "%.3f");
			item_tooltip("With the blade angle 0: the tip of the blade from the start of the switch as its scheme gives it, the diverging track is broken there into pieces; 0: where the blade gets 5 mm thick, less the 125 mm its nose is planed over");
			changed |= input("Blade heel, m##designheel", "##designheel", entry.heel, "%.3f");
			item_tooltip("Where the moving blades end, from the start of the switch, as its scheme gives it; the switch keeps it in the scenery. 0: as long as the flangeway of 58 mm behind them needs");
			changed |= input("Blade throw, m##designthrow", "##designthrow", entry.bladethrow, "%.3f");
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
			item_tooltip("Length of the main track, 0: as far as the diverging tracks reach");
			changed |= input("a##designa", "##designa", entry.a, "%.3f");
			item_tooltip("From the start to the point where the tangents of both tracks meet, as the catalogue gives it; 0: from the geometry");
			changed |= input("b##designb", "##designb", entry.b, "%.3f");
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
	ImGui::Checkbox(STR_C("Stretch across"), &designer.stretched);
	item_tooltip("Draws the offsets from the main track larger, so the shape of the diverging track can be seen");
	draw_shape(shape, 150.0f, designer.stretched);

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
