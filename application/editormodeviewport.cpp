/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// over the 3d view: the tools of the field of work along its left edge, the modes of the gizmo at its top, the axes of the
// world and the top view in its top right corner. the same tools in the toolbar under the menu, with their names

#include "stdafx.h"
#include "application/editormode.h"
#include "application/editoruilayer.h"
#include "editor/editorIcons.hpp"
#include "rendering/renderer.h"
#include "utilities/Globals.h"
#include "utilities/translation.h"
#include "imgui/imgui.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <vector>

struct editor_tool_button
{
	editor_icons::icon picture; // icon::count if it has none: then it's only in the toolbar, as its name
	char const *label;
	char const *key; // nullptr if there's none
	char const *tooltip; // nullptr if the label says enough
	bool chosen;
	std::function<void()> action;
	bool group{false}; // a gap before it, it starts another group
	bool enabled{true};
};

namespace
{

using editor_icons::icon;
using tool_button = editor_tool_button;

float scale()
{
	return std::max(1.0f, Global.ui_scale);
}

// side of a button, the icon with some room around it
float button_side()
{
	return editor_icons::size() + 10.0f * scale();
}

// a window of no looks of its own, for the buttons over the 3d view; it takes the mouse only where it lies
bool begin_overlay(char const *Name, ImVec2 const &Position, ImVec2 const &Size)
{
	ImGui::SetNextWindowViewport(ImGui::GetMainViewport()->ID);
	ImGui::SetNextWindowPos(Position);
	ImGui::SetNextWindowSize(Size);
	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
	ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
	ImGui::PushStyleVar(ImGuiStyleVar_WindowMinSize, ImVec2(1.0f, 1.0f));
	auto const flags{ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav |
	                 ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoScrollWithMouse};
	auto const open{ImGui::Begin(Name, nullptr, flags)};
	ImGui::PopStyleVar(3);
	return open;
}

// the name of the tool with its key, and what it does
void button_tooltip(tool_button const &Button)
{
	std::string tip{Button.label};
	if (Button.key != nullptr)
		tip += std::string{"  ["} + Button.key + "]";
	if (Button.tooltip != nullptr)
		tip += std::string{"\n"} + Button.tooltip;
	ImGui::SetTooltip("%s", tip.c_str());
}

// the first letters of the label in place of the icon, if the renderer can't show images
void icon_or_letters(ImDrawList *List, tool_button const &Button, ImVec2 const &Center, ImU32 const Color)
{
	if (editor_icons::draw(List, Button.picture, Center, Color))
		return;
	char const text[]{Button.label[0], (Button.label[0] != '\0' ? Button.label[1] : '\0'), '\0'};
	auto const size{ImGui::CalcTextSize(text)};
	List->AddText(ImVec2(Center.x - size.x * 0.5f, Center.y - size.y * 0.5f), Color, text);
}

// the button with the icon at the place; the label stands in for the icon if the renderer can't show images
void icon_button(tool_button const &Button, ImVec2 const &Position, int const Id)
{
	auto const side{button_side()};
	ImGui::SetCursorScreenPos(Position);
	ImGui::PushID(Id);
	auto const clicked{ImGui::InvisibleButton("##tool", ImVec2(side, side))};
	ImGui::PopID();
	auto const hovered{ImGui::IsItemHovered()};
	auto *list{ImGui::GetWindowDrawList()};
	ImVec2 const end{Position.x + side, Position.y + side};
	auto const rounding{4.0f * scale()};
	auto const background{Button.chosen                 ? ImGui::GetColorU32(ImGuiCol_ButtonActive) :
	                      hovered && Button.enabled ? ImGui::GetColorU32(ImGuiCol_ButtonHovered) :
	                                                    IM_COL32(24, 26, 30, 200)};
	list->AddRectFilled(Position, end, background, rounding);
	if (Button.chosen)
		list->AddRect(Position, end, ImGui::GetColorU32(ImGuiCol_CheckMark), rounding, 0, 1.5f * scale());
	auto const color{Button.enabled ? (Button.chosen || hovered ? IM_COL32(255, 255, 255, 255) : IM_COL32(215, 220, 225, 235)) : IM_COL32(150, 150, 150, 110)};
	icon_or_letters(list, Button, ImVec2(Position.x + side * 0.5f, Position.y + side * 0.5f), color);
	if (hovered)
		button_tooltip(Button);
	if (clicked && Button.enabled && Button.action)
		Button.action();
}

} // namespace

void editor_mode::render_viewport_overlays()
{
	auto const width{editor_ui::view_max().x - editor_ui::view_min().x};
	auto const height{editor_ui::view_max().y - editor_ui::view_min().y};
	// a view squeezed between the docks has no room for them
	if (width < 8.0f * button_side() || height < 6.0f * button_side())
		return;
	render_viewport_tools();
	render_viewport_gizmo();
	render_view_axes();
}

void editor_mode::work_area_tools(std::vector<tool_button> &Buttons)
{
	switch (current_work_area())
	{
	case work_area::surroundings:
	{
		struct mode
		{
			icon picture;
			char const *label;
			char const *key;
			nodebank_panel::edit_mode which;
		};
		mode const modes[] = {{icon::select, STR_C("Select"), "1", nodebank_panel::MODIFY},
		                      {icon::insert, STR_C("Insert"), "2", nodebank_panel::ADD},
		                      {icon::brush, STR_C("Brush"), "3", nodebank_panel::BRUSH},
		                      {icon::area_fill, STR_C("Area fill"), "4", nodebank_panel::FILL},
		                      {icon::copy_to_bank, STR_C("Copy to bank"), "5", nodebank_panel::COPY}};
		for (auto const &entry : modes)
			Buttons.push_back({entry.picture, entry.label, entry.key, nullptr, ui()->mode() == entry.which, [this, which = entry.which]() { choose_edit_mode(which); }});
		break;
	}
	case work_area::tracks:
	{
		auto const vehicle{m_track_tab == track_tab::lineside && m_vehicle.open};
		auto const add = [&](track_tab const Tab, icon const Picture, bool const Group) {
			auto const &modes{track_modes()};
			auto const found{std::find_if(modes.begin(), modes.end(), [&](track_mode const &Mode) { return Mode.tab == Tab; })};
			if (found == modes.end())
				return;
			// a tool chosen again gives way to the select one
			auto const chosen{m_track_tab == Tab && false == (Tab == track_tab::lineside && vehicle)};
			Buttons.push_back({Picture, STR_C(found->label), found->key, STR_C(found->tooltip), chosen, [this, Tab, chosen]() { show_track_tab(chosen ? track_tab::path : Tab); }, Group});
		};
		add(track_tab::path, icon::select, false);
		add(track_tab::lay, icon::lay_track, false);
		add(track_tab::turnout, icon::turnout, false);
		add(track_tab::straights, icon::straight, false);
		add(track_tab::route, icon::curve, false);
		add(track_tab::turntable, icon::turntable, false);
		add(track_tab::signals, icon::signal, true);
		add(track_tab::lineside, icon::objects, false);
		Buttons.push_back({icon::vehicle, STR_C("Vehicle"), nullptr, STR_C("A vehicle to drive, put on the selected track; it isn't written to the scenery files"), vehicle, [this, vehicle]() {
			                   show_track_tab(vehicle ? track_tab::path : track_tab::lineside);
			                   m_vehicle.expand = false == vehicle;
		                   }});
		add(track_tab::profile, icon::profile, true);
		add(track_tab::speed, icon::speed, false);
		add(track_tab::joints, icon::joints, false);
		add(track_tab::infra, icon::infra, false);
		Buttons.push_back({icon::gauge, STR_C("Structure gauge"), nullptr, STR_C("Checks which models enter the structure gauge of the tracks and the clearance over the roads (skrajnia budowli)"), m_gauge.open,
		                   [this]() { m_gauge.open = !m_gauge.open; }});
		break;
	}
	case work_area::roads:
	{
		auto &tool{m_roadtool};
		Buttons.push_back({icon::select, STR_C("Select"), nullptr, STR_C("LMB: a road piece, a junction, a point where pieces meet, a level crossing or a traffic point"), tool.tool == 0, [this]() { road_choose_tool(0); }});
		Buttons.push_back({icon::road, STR_C("Build"), nullptr, STR_C("LMB: start a road, then each next point; on a loose end, the side of a road or a junction it's joined to them"), tool.tool == 1, [this]() { road_choose_tool(1); }});
		Buttons.push_back({icon::place, STR_C("Place"), nullptr, STR_C("Level crossings, and the points where vehicles appear on the roads or are taken off them"), tool.tool == 2, [this]() { road_choose_tool(2); }});
		if (tool.tool == 2)
		{
			// what the place tool puts
			char const *const kinds[] = {STR_C("Level crossing"), STR_C("Spawn point"), STR_C("Removal point"), STR_C("Pedestrian crossing")};
			for (int index = 0; index < static_cast<int>(std::size(kinds)); ++index)
				Buttons.push_back({icon::count, kinds[index], nullptr, nullptr, tool.placekind == index, [&tool, index]() { tool.placekind = index; }, index == 0});
		}
		Buttons.push_back({icon::lanes, STR_C("Show lanes"), nullptr, STR_C("The lanes of the roads drawn as the paths the vehicles take"), tool.lanes, [&tool]() { tool.lanes = !tool.lanes; }, true});
		break;
	}
	case work_area::terrain:
	{
		// what the left button does on the terrain
		auto const choose = [this](terrain_tool const Tool) { return [this, Tool]() { m_terrain_tool = Tool; }; };
		auto const chosen = [this](terrain_tool const Tool) { return m_terrain_tool == Tool; };
		Buttons.push_back({icon::select, STR_C("Select"), nullptr, STR_C("LMB picks the models, as in the surroundings"), chosen(terrain_tool::none), choose(terrain_tool::none)});
		Buttons.push_back({icon::sculpt, STR_C("Sculpt"), nullptr, STR_C("LMB raises the terrain under the brush, Shift+LMB lowers it"), chosen(terrain_tool::sculpt), choose(terrain_tool::sculpt)});
		Buttons.push_back({icon::smooth, STR_C("Smooth"), nullptr, STR_C("LMB evens the terrain out under the brush"), chosen(terrain_tool::smooth), choose(terrain_tool::smooth)});
		Buttons.push_back({icon::profile, STR_C("Level"), nullptr, STR_C("LMB leads the terrain under the brush to the target height, Ctrl+LMB takes the target height from what's under the cursor"),
		                   chosen(terrain_tool::level), choose(terrain_tool::level)});
		Buttons.push_back({icon::brush, STR_C("Paint"), nullptr, STR_C("LMB paints the material chosen in the palette, Shift+LMB the first material of the palette"), chosen(terrain_tool::paint), choose(terrain_tool::paint)});
		Buttons.push_back({icon::chunks, STR_C("Chunks"), nullptr, STR_C("LMB adds a chunk next to the clicked one, Shift+LMB removes it"), chosen(terrain_tool::chunks), choose(terrain_tool::chunks)});
		Buttons.push_back({icon::count, STR_C("Point spacing"), nullptr, STR_C("LMB gives the clicked chunk the point spacing chosen in the tool options"), chosen(terrain_tool::spacing), choose(terrain_tool::spacing)});
		Buttons.push_back({icon::area_fill, STR_C("Water"), nullptr, STR_C("LMB adds a point of the outline of a body of water, Shift+LMB takes the last one back"), chosen(terrain_tool::water), choose(terrain_tool::water)});
		auto const orthophoto{m_orthophoto.enabled()};
		Buttons.push_back({icon::orthophoto, STR_C("Orthophoto"), nullptr, STR_C("Aerial imagery of geoportal.gov.pl under the scenery, laid out by the origin of the scenery"), orthophoto,
		                   [this, orthophoto]() { m_orthophoto.enabled(false == orthophoto); }, true});
		Buttons.push_back({icon::count, STR_C("Orthophoto settings..."), nullptr, nullptr, m_orthophoto_window, [this]() { m_orthophoto_window = !m_orthophoto_window; }});
		break;
	}
	}
}

void editor_mode::render_viewport_tools()
{
	// the tools of the field of work which have icons, as in the toolbar
	std::vector<tool_button> buttons;
	buttons.reserve(24);
	work_area_tools(buttons);
	std::erase_if(buttons, [](tool_button const &Button) { return Button.picture == icon::count; });
	if (buttons.empty())
		return;

	// down the left edge, in more columns if the view is too low for them; the bottom is left to the hints of the tools
	auto const side{button_side()};
	auto const gap{2.0f * scale()};
	auto const groupgap{8.0f * scale()};
	auto const margin{8.0f * scale()};
	ImVec2 const origin{editor_ui::view_min().x + margin, editor_ui::view_min().y + margin};
	auto const bottom{std::max(origin.y + 4.0f * side, editor_ui::view_max().y - 130.0f * scale())};
	std::vector<ImVec2> places;
	places.reserve(buttons.size());
	ImVec2 place{origin};
	for (auto const &button : buttons)
	{
		if (button.group && place.y > origin.y)
			place.y += groupgap - gap;
		if (place.y > origin.y && place.y + side > bottom)
			place = ImVec2(place.x + side + gap, origin.y);
		places.push_back(place);
		place.y += side + gap;
	}
	ImVec2 extent{0.0f, 0.0f};
	for (auto const &at : places)
		extent = ImVec2(std::max(extent.x, at.x + side - origin.x), std::max(extent.y, at.y + side - origin.y));
	auto const pad{3.0f * scale()};
	if (begin_overlay("##viewporttools", ImVec2(origin.x - pad, origin.y - pad), ImVec2(extent.x + 2.0f * pad, extent.y + 2.0f * pad)))
	{
		ImGui::GetWindowDrawList()->AddRectFilled(ImVec2(origin.x - pad, origin.y - pad), ImVec2(origin.x + extent.x + pad, origin.y + extent.y + pad), IM_COL32(16, 18, 21, 140), 6.0f * scale());
		for (std::size_t index = 0; index < buttons.size(); ++index)
			icon_button(buttons[index], places[index], static_cast<int>(index));
	}
	ImGui::End();
}

void editor_mode::render_toolbar_caption(char const *Text)
{
	// as wide in both rows, so the buttons of the rows start one under the other. the rows are drawn with no spacing of items
	auto const margin{std::round(ImGui::GetFontSize() * 0.75f)};
	auto const width{std::max(ImGui::CalcTextSize(STR_C("Work modes")).x, ImGui::CalcTextSize(STR_C("Tools")).x) + 2.0f * margin};
	auto const position{ImGui::GetWindowPos()};
	auto const height{ImGui::GetWindowHeight()};
	auto *list{ImGui::GetWindowDrawList()};
	auto const text{ImGui::CalcTextSize(Text)};
	list->AddText(ImVec2(position.x + margin, std::round(position.y + (height - text.y) * 0.5f)), ImGui::GetColorU32(ImGuiCol_TextDisabled), Text);
	list->AddLine(ImVec2(position.x + width, position.y + height * 0.2f), ImVec2(position.x + width, position.y + height * 0.8f), ImGui::GetColorU32(ImGuiCol_Separator));
	ImGui::SetCursorScreenPos(position);
	ImGui::Dummy(ImVec2(width + margin * 0.5f, height));
	ImGui::SameLine(0.0f, 0.0f);
}

void editor_mode::render_toolbar_tools()
{
	render_toolbar_caption(STR_C("Tools"));
	std::vector<tool_button> buttons;
	buttons.reserve(24);
	work_area_tools(buttons);
	if (buttons.empty())
		return;

	auto const s{scale()};
	auto const iconside{editor_icons::size()};
	auto const height{ImGui::GetWindowHeight()};
	auto const margin{3.0f * s}; // over and under the buttons
	auto const pad{7.0f * s}; // inside the button, at its ends
	auto const spacing{5.0f * s}; // between the icon and the name
	auto const gap{3.0f * s};
	auto const groupgap{15.0f * s};
	auto const buttonheight{height - 2.0f * margin};
	auto const left{ImGui::GetCursorScreenPos().x};
	auto const top{ImGui::GetWindowPos().y + margin};
	auto const right{ImGui::GetWindowPos().x + ImGui::GetWindowWidth() - 4.0f * s};
	// the names next to the icons if there's room for them; if there isn't, the name of the chosen tool only, or none
	enum class names
	{
		all,
		chosen,
		none
	};
	auto const named = [](tool_button const &Button, names const Names) {
		return Button.picture == icon::count || Names == names::all || (Names == names::chosen && Button.chosen);
	};
	auto const width = [&](tool_button const &Button, names const Names) {
		if (false == named(Button, Names))
			return std::max(buttonheight, iconside + pad);
		auto result{2.0f * pad + ImGui::CalcTextSize(Button.label).x};
		if (Button.picture != icon::count)
			result += iconside + spacing;
		return result;
	};
	auto const total = [&](names const Names) {
		auto result{0.0f};
		for (std::size_t index = 0; index < buttons.size(); ++index)
			result += width(buttons[index], Names) + (index == 0 ? 0.0f : buttons[index].group ? groupgap : gap);
		return result;
	};
	auto shown{names::none};
	for (auto const option : {names::all, names::chosen})
		if (left + total(option) <= right)
		{
			shown = option;
			break;
		}

	auto *list{ImGui::GetWindowDrawList()};
	auto const rounding{3.0f * s};
	auto x{left};
	for (std::size_t index = 0; index < buttons.size(); ++index)
	{
		auto const &button{buttons[index]};
		if (index > 0)
		{
			x += button.group ? groupgap : gap;
			if (button.group)
				list->AddLine(ImVec2(std::round(x - groupgap * 0.5f), top + buttonheight * 0.15f), ImVec2(std::round(x - groupgap * 0.5f), top + buttonheight * 0.85f), ImGui::GetColorU32(ImGuiCol_Separator));
		}
		auto const size{ImVec2(width(button, shown), buttonheight)};
		ImVec2 const position{x, top};
		ImGui::SetCursorScreenPos(position);
		ImGui::PushID(static_cast<int>(index));
		auto const clicked{ImGui::InvisibleButton("##tool", size)};
		ImGui::PopID();
		auto const hovered{ImGui::IsItemHovered()};
		ImVec2 const end{position.x + size.x, position.y + size.y};
		// the chosen one stands out as the chosen field of work does, the others as buttons
		auto const background{button.chosen ? ImGui::GetColorU32(ImGuiCol_Header) : hovered && button.enabled ? ImGui::GetColorU32(ImGuiCol_HeaderHovered) : ImGui::GetColorU32(ImGuiCol_FrameBg)};
		list->AddRectFilled(position, end, background, rounding);
		if (button.chosen)
			list->AddRectFilled(ImVec2(position.x, end.y - 2.0f * s), end, ImGui::GetColorU32(ImGuiCol_CheckMark), rounding, ImDrawFlags_RoundCornersBottom);
		auto const color{button.enabled ? ImGui::GetColorU32(ImGuiCol_Text, button.chosen || hovered ? 1.0f : 0.85f) : ImGui::GetColorU32(ImGuiCol_TextDisabled)};
		auto const center{position.y + size.y * 0.5f};
		if (named(button, shown))
		{
			auto textx{position.x + pad};
			if (button.picture != icon::count)
			{
				editor_icons::draw(list, button.picture, ImVec2(textx + iconside * 0.5f, center), color);
				textx += iconside + spacing;
			}
			list->AddText(ImVec2(textx, std::round(center - ImGui::GetFontSize() * 0.5f)), color, button.label);
		}
		else
			icon_or_letters(list, button, ImVec2(position.x + size.x * 0.5f, center), color);
		if (hovered)
			button_tooltip(button);
		if (clicked && button.enabled && button.action)
			button.action();
		x = end.x;
	}
}

void editor_mode::render_viewport_gizmo()
{
	// the gizmo moves the models and the tracks; on the terrain the mouse is the brush's
	if (false == m_gizmo_enabled || current_work_area() == work_area::terrain)
		return;
	auto const tooltip{STR_C("What the handles of the gizmo on the selection do when dragged; Ctrl held snaps the values")};
	std::array<tool_button, 4> const buttons{{
	    {icon::translate, STR_C("Translate"), "Q", tooltip, m_gizmo_op == gizmo_operation::translate, [this]() { m_gizmo_op = gizmo_operation::translate; }},
	    {icon::rotate, STR_C("Rotate"), "W", tooltip, m_gizmo_op == gizmo_operation::rotate, [this]() { m_gizmo_op = gizmo_operation::rotate; }},
	    {icon::scale, STR_C("Scale"), "E", tooltip, m_gizmo_op == gizmo_operation::scale, [this]() { m_gizmo_op = gizmo_operation::scale; }},
	    // the gizmo always scales along the axes of the model
	    {icon::local_space, STR_C("Local space"), "R", STR_C("The handles along the axes of the selected model instead of the axes of the world"), m_gizmo_local || m_gizmo_op == gizmo_operation::scale,
	     [this]() { m_gizmo_local = !m_gizmo_local; }, true, m_gizmo_op != gizmo_operation::scale},
	}};
	// in a row at the top, in the middle
	auto const side{button_side()};
	auto const gap{2.0f * scale()};
	auto const groupgap{8.0f * scale()};
	auto const margin{8.0f * scale()};
	auto const pad{3.0f * scale()};
	auto const width{4.0f * side + 2.0f * gap + groupgap};
	ImVec2 const origin{std::round((editor_ui::view_min().x + editor_ui::view_max().x - width) * 0.5f), editor_ui::view_min().y + margin};
	if (begin_overlay("##viewportgizmo", ImVec2(origin.x - pad, origin.y - pad), ImVec2(width + 2.0f * pad, side + 2.0f * pad)))
	{
		ImGui::GetWindowDrawList()->AddRectFilled(ImVec2(origin.x - pad, origin.y - pad), ImVec2(origin.x + width + pad, origin.y + side + pad), IM_COL32(16, 18, 21, 140), 6.0f * scale());
		auto x{origin.x};
		for (std::size_t index = 0; index < buttons.size(); ++index)
		{
			if (buttons[index].group)
				x += groupgap - gap;
			icon_button(buttons[index], ImVec2(x, origin.y), static_cast<int>(index));
			x += side + gap;
		}
	}
	ImGui::End();
}

void editor_mode::render_view_axes()
{
	// the axes of the world as the camera sees them, as in blender: a click on one looks along it
	auto const s{scale()};
	auto const radius{40.0f * s};
	auto const margin{8.0f * s};
	auto const side{button_side()};
	auto const size{2.0f * radius};
	ImVec2 const origin{editor_ui::view_max().x - margin - size, editor_ui::view_min().y + margin};
	if (begin_overlay("##viewaxes", origin, ImVec2(size, size + 6.0f * s + side)))
	{
		auto *list{ImGui::GetWindowDrawList()};
		ImVec2 const center{origin.x + radius, origin.y + radius};
		ImGui::SetCursorScreenPos(origin);
		ImGui::InvisibleButton("##axes", ImVec2(size, size));
		auto const hovered{ImGui::IsItemHovered()};
		auto const clicked{ImGui::IsItemClicked(ImGuiMouseButton_Left)};
		if (hovered)
			list->AddCircleFilled(center, radius, IM_COL32(255, 255, 255, 28), 48);
		// the view matrix is of the camera turned only, the axes go through it as they are
		glm::mat3 const view{GfxRenderer->Camera_View_Matrix()};
		struct end
		{
			int axis;
			bool positive;
			glm::vec3 seen;
		};
		std::array<end, 6> ends;
		for (int axis = 0; axis < 3; ++axis)
		{
			glm::vec3 direction{0.0f};
			direction[axis] = 1.0f;
			auto const seen{view * direction};
			ends[axis * 2] = {axis, true, seen};
			ends[axis * 2 + 1] = {axis, false, -seen};
		}
		// the farther ones first, the nearer ones over them
		std::sort(ends.begin(), ends.end(), [](end const &Left, end const &Right) { return Left.seen.z < Right.seen.z; });
		ImU32 const colors[]{IM_COL32(230, 70, 80, 255), IM_COL32(130, 200, 40, 255), IM_COL32(60, 135, 235, 255)};
		char const *const names[]{"X", "Y", "Z"};
		auto const reach{radius - 10.0f * s};
		auto const mouse{ImGui::GetIO().MousePos};
		end const *pointed{nullptr};
		auto const place = [&](end const &End) { return ImVec2(center.x + End.seen.x * reach, center.y - End.seen.y * reach); };
		for (auto const &entry : ends)
		{
			auto const at{place(entry)};
			auto const dot{(entry.positive ? 8.0f : 6.0f) * s};
			if (hovered && (mouse.x - at.x) * (mouse.x - at.x) + (mouse.y - at.y) * (mouse.y - at.y) <= (dot + 2.0f * s) * (dot + 2.0f * s))
				pointed = &entry; // the nearest one under the mouse, as it's drawn over the others
		}
		for (auto const &entry : ends)
		{
			auto const at{place(entry)};
			auto const color{colors[entry.axis]};
			if (entry.positive)
			{
				list->AddLine(center, at, color, 2.0f * s);
				list->AddCircleFilled(at, 8.0f * s, color, 24);
				auto const text{ImGui::CalcTextSize(names[entry.axis])};
				list->AddText(ImVec2(at.x - text.x * 0.5f, at.y - text.y * 0.5f), IM_COL32(20, 20, 20, 255), names[entry.axis]);
			}
			else
			{
				list->AddCircleFilled(at, 6.0f * s, (color & ~IM_COL32_A_MASK) | IM_COL32(0, 0, 0, 90), 24);
				list->AddCircle(at, 6.0f * s, color, 24, 1.5f * s);
			}
			if (&entry == pointed)
				list->AddCircle(at, (entry.positive ? 9.5f : 7.5f) * s, IM_COL32(255, 255, 255, 255), 24, 1.5f * s);
		}
		if (pointed != nullptr)
		{
			ImGui::SetTooltip(STR_C("Look along %s%s; from over the scenery it's the top view"), pointed->positive ? "-" : "+", names[pointed->axis]);
			if (clicked)
				look_along_axis(pointed->axis, pointed->positive);
		}
		// the top view, under the axes
		tool_button const ortho{Global.EditorOrtho ? icon::ortho : icon::perspective,
		                        STR_C("Top view, orthographic"),
		                        "O",
		                        STR_C("The view straight down without perspective, as on a map; again for the camera as it was"),
		                        Global.EditorOrtho,
		                        [this]() { toggle_ortho(); }};
		icon_button(ortho, ImVec2(std::round(center.x - side * 0.5f), origin.y + size + 6.0f * s), 0);
	}
	ImGui::End();
}

void editor_mode::look_along_axis(int const Axis, bool const Positive)
{
	m_focus_active = false;
	if (Axis == 1)
	{
		if (Positive)
		{
			// from over the scenery: the top view
			if (false == Global.EditorOrtho)
				toggle_ortho();
			return;
		}
		if (Global.EditorOrtho)
			toggle_ortho();
		Camera.Angle.x = M_PI_2;
		Camera.Angle.z = 0.0;
		return;
	}
	if (Global.EditorOrtho)
		toggle_ortho();
	// from the side of the axis the camera's on, towards the other one; the yaw as the camera works it out of where it looks
	glm::dvec3 where{0.0};
	where[Axis] = Positive ? -1.0 : 1.0;
	Camera.Angle.y = std::atan2(-where.x, -where.z);
	Camera.Angle.x = 0.0;
	Camera.Angle.z = 0.0;
}
