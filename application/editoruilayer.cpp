/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/editoruilayer.h"

#include "utilities/Globals.h"
#include "utilities/utilities.h"
#include "scene/scenenode.h"
#include "scene/scenelayers.h"
#include "scene/scene.h"
#include "rendering/renderer.h"
#include "utilities/translation.h"
#include "imgui/imgui_internal.h"

#include <cstdio>
#include <limits>

glm::vec2 editor_ui::m_viewmin{0.0f};
glm::vec2 editor_ui::m_viewmax{0.0f};

editor_ui::editor_ui()
{

	clear_panels();
	// bind the panels with ui object. maybe not the best place for this but, eh

	add_external_panel(&m_nodebankpanel);
	add_external_panel(&m_layerspanel);
	add_external_panel(&m_includespanel);

	// the menu is a part of the layout of the editor, not a pop-up as in the driver mode
	m_menu_always = true;
	// the window menu of the docks can unpin them
	GImGui->DockNodeWindowMenuHandler = &editor_ui::autohide_menu;
}

void editor_ui::render_dockspace()
{
	// windows of the editor are docked around the 3d view, which is left free in the middle
	auto *viewport{ImGui::GetMainViewport()};
	auto const dockspace{ImGui::GetID("###editordockspace")};
	m_dockspace = dockspace;
	if (m_layoutreset || ImGui::DockBuilderGetNode(dockspace) == nullptr)
	{
		m_layoutreset = false;
		// the windows go to their default docks, none is left unpinned
		m_autohide.clear();
		m_autohideshown = 0;
		ImGui::MarkIniSettingsDirty();
		build_default_layout(dockspace);
	}
	autohide_begin();
	ImGui::DockSpaceOverViewport(dockspace, viewport, ImGuiDockNodeFlags_PassthruCentralNode);
	if (auto const *central{ImGui::DockBuilderGetCentralNode(dockspace)}; central != nullptr)
	{
		m_viewmin = {central->Pos.x, central->Pos.y};
		m_viewmax = {central->Pos.x + central->Size.x, central->Pos.y + central->Size.y};
	}
	else
	{
		m_viewmin = {viewport->WorkPos.x, viewport->WorkPos.y};
		m_viewmax = {viewport->WorkPos.x + viewport->WorkSize.x, viewport->WorkPos.y + viewport->WorkSize.y};
	}
	autohide_place();
}

void editor_ui::build_default_layout(unsigned int const Dockspace)
{
	auto *viewport{ImGui::GetMainViewport()};
	ImGui::DockBuilderRemoveNode(Dockspace);
	ImGui::DockBuilderAddNode(Dockspace, ImGuiDockNodeFlags_DockSpace | ImGuiDockNodeFlags_PassthruCentralNode);
	ImGui::DockBuilderSetNodePos(Dockspace, viewport->WorkPos);
	ImGui::DockBuilderSetNodeSize(Dockspace, viewport->WorkSize);
	// widths in pixels the windows need, as parts of the room there is
	auto const width{std::max(viewport->WorkSize.x, 1.0f)};
	auto const height{std::max(viewport->WorkSize.y, 1.0f)};
	auto const left_width{std::min(400.0f * Global.ui_scale, width * 0.3f)};
	auto const right_width{std::min(460.0f * Global.ui_scale, width * 0.3f)};
	auto const bottom_height{std::min(280.0f * Global.ui_scale, height * 0.35f)};
	ImGuiID center{Dockspace};
	auto const right{ImGui::DockBuilderSplitNode(center, ImGuiDir_Right, right_width / width, nullptr, &center)};
	auto const left{ImGui::DockBuilderSplitNode(center, ImGuiDir_Left, left_width / (width - right_width), nullptr, &center)};
	auto const bottom{ImGui::DockBuilderSplitNode(center, ImGuiDir_Down, bottom_height / height, nullptr, &center)};
	// left: settings of the tool in use above, the scenery below
	ImGuiID lefttop{left};
	auto const leftbottom{ImGui::DockBuilderSplitNode(lefttop, ImGuiDir_Down, 0.5f, nullptr, &lefttop)};
	// windows are found by the stable part of their names, after ###
	ImGui::DockBuilderDockWindow("###tooloptions", lefttop);
	for (auto const *window : {"###scenetree", "###Layers", "###scenehierarchy"})
		ImGui::DockBuilderDockWindow(window, leftbottom);
	for (auto const *window : {"###inspector", "###editorsettings", "###Include database"})
		ImGui::DockBuilderDockWindow(window, right);
	for (auto const *window : {"###Node bank", "###profilestrip", "###trackanalysis", "###structuregauge", "###editorhistory", "###modelsets"})
		ImGui::DockBuilderDockWindow(window, bottom);
	ImGui::DockBuilderFinish(Dockspace);
}

// unpinned docks. imgui has no auto-hide of its own: the windows of an unpinned dock are undocked, kept hidden while they aren't shown,
// and the one shown is placed over the edge of the 3d view its dock was at. a window docked again by hand is pinned again
std::vector<editor_ui::autohide_group> editor_ui::m_autohide;
unsigned int editor_ui::m_dockspace{0};

namespace
{

// imgui math operators aren't defined here, imgui.h is included before imgui_internal.h asks for them
ImVec2 vec_add(ImVec2 const A, ImVec2 const B)
{
	return {A.x + B.x, A.y + B.y};
}
ImVec2 vec_sub(ImVec2 const A, ImVec2 const B)
{
	return {A.x - B.x, A.y - B.y};
}
ImVec2 vec_mul(ImVec2 const A, float const B)
{
	return {A.x * B, A.y * B};
}

bool autohide_vertical(int const Edge)
{
	return Edge == ImGuiDir_Left || Edge == ImGuiDir_Right;
}

// leaf nodes of the dockspace which hold windows, the central node aside
void autohide_leaves(ImGuiDockNode *Node, std::vector<ImGuiDockNode *> &Leaves)
{
	if (Node == nullptr)
		return;
	if (Node->IsLeafNode())
	{
		if (false == Node->IsCentralNode() && Node->Windows.Size > 0)
			Leaves.push_back(Node);
		return;
	}
	autohide_leaves(Node->ChildNodes[0], Leaves);
	autohide_leaves(Node->ChildNodes[1], Leaves);
}

// edge of the central node the node lies at
int autohide_edge(ImGuiDockNode const *Node, ImGuiDockNode const *Central)
{
	if (Central == nullptr)
		return ImGuiDir_Left;
	auto const center{vec_add(Node->Pos, vec_mul(Node->Size, 0.5f))};
	if (center.x < Central->Pos.x)
		return ImGuiDir_Left;
	if (center.x > Central->Pos.x + Central->Size.x)
		return ImGuiDir_Right;
	if (center.y > Central->Pos.y + Central->Size.y)
		return ImGuiDir_Down;
	return ImGuiDir_Up;
}

// pin drawn with primitives, there's no icon font; a pinned one stands, an unpinned one lies
void autohide_draw_pin(ImDrawList *List, ImRect const &Rect, bool const Pinned, bool const Hovered)
{
	auto const color{ImGui::GetColorU32(ImGuiCol_Text)};
	if (Hovered)
		List->AddRectFilled(Rect.Min, Rect.Max, ImGui::GetColorU32(ImGuiCol_ButtonHovered), 2.0f);
	auto const center{Rect.GetCenter()};
	auto const s{Rect.GetWidth()};
	// along the pin: from the head to the point, and across it
	ImVec2 const along{Pinned ? ImVec2(0.0f, 1.0f) : ImVec2(-1.0f, 0.0f)};
	ImVec2 const across{along.y, -along.x};
	auto const head{vec_sub(center, vec_mul(along, s * 0.30f))};
	auto const collar{vec_add(center, vec_mul(along, s * 0.05f))};
	auto const point{vec_add(center, vec_mul(along, s * 0.40f))};
	auto const half{vec_mul(across, s * 0.16f)};
	auto const guard{vec_mul(across, s * 0.30f)};
	// the head and the collar the needle comes out of
	List->AddQuadFilled(vec_sub(head, half), vec_add(head, half), vec_add(collar, half), vec_sub(collar, half), color);
	List->AddLine(vec_sub(collar, guard), vec_add(collar, guard), color, 1.5f);
	List->AddLine(collar, point, color, 1.0f);
}

// text turned to read from the top down, for the strips at the sides
void autohide_vertical_text(ImDrawList *List, ImVec2 const Position, ImU32 const Color, char const *Begin, char const *End)
{
	auto const height{ImGui::GetFontSize()};
	auto const first{List->VtxBuffer.Size};
	// the glyphs are laid out across first, from the left edge of the main window: out of the clip rectangle or the screen
	// they'd be left out, as they would at the right edge
	ImVec2 const origin{ImGui::GetMainViewport()->Pos.x, Position.y};
	List->PushClipRectFullScreen();
	List->AddText(origin, Color, Begin, End);
	List->PopClipRect();
	for (auto index = first; index < List->VtxBuffer.Size; ++index)
	{
		auto &position{List->VtxBuffer[index].pos};
		auto const offset{vec_sub(position, origin)};
		position = ImVec2(Position.x + height - offset.y, Position.y + offset.x);
	}
}

} // namespace

std::size_t editor_ui::autohide_group_of(unsigned int const Window)
{
	for (std::size_t index = 0; index < m_autohide.size(); ++index)
		if (std::find(m_autohide[index].windows.begin(), m_autohide[index].windows.end(), Window) != m_autohide[index].windows.end())
			return index;
	return static_cast<std::size_t>(-1);
}

void editor_ui::autohide_node(ImGuiDockNode *Node)
{
	if (Node == nullptr || Node->Windows.Size == 0 || Node->IsCentralNode())
		return;
	autohide_group group;
	group.edge = autohide_edge(Node, ImGui::DockNodeGetRootNode(Node)->CentralNode);
	group.size = autohide_vertical(group.edge) ? Node->Size.x : Node->Size.y;
	// place in the column of docks at that edge; a dock alone there goes back before the one which comes back to it
	auto const along{autohide_vertical(group.edge) ? ImGuiAxis_Y : ImGuiAxis_X};
	group.first = true;
	if (auto const *parent{Node->ParentNode}; parent != nullptr && parent->SplitAxis == along)
	{
		group.first = parent->ChildNodes[0] == Node;
		group.share = along == ImGuiAxis_Y ? Node->Size.y / std::max(parent->Size.y, 1.0f) : Node->Size.x / std::max(parent->Size.x, 1.0f);
	}
	// docks split along the edge make one column; the docks put back all at once are put back from the outer columns in,
	// so each spans as far as it did
	auto const *column{Node};
	while (column->ParentNode != nullptr && column->ParentNode->SplitAxis == along)
		column = column->ParentNode;
	for (auto const *parent{column->ParentNode}; parent != nullptr; parent = parent->ParentNode)
		++group.depth;
	for (auto *window : Node->Windows)
	{
		group.windows.push_back(window->ID);
		// carried out at the start of the next frame
		ImGui::DockContextQueueUndockWindow(GImGui, window);
	}
	m_autohide.push_back(std::move(group));
	ImGui::MarkIniSettingsDirty();
}

void editor_ui::autohide_pin(std::size_t const Group)
{
	if (Group >= m_autohide.size())
		return;
	auto &group{m_autohide[Group]};
	auto *central{ImGui::DockBuilderGetCentralNode(m_dockspace)};
	ImGuiID target{0};
	if (central != nullptr)
	{
		// the node went with its last window, and its id may have been given to another one since: another dock at that edge
		// gets split, so that docks taken there one by one come back the way they were, or the room of the view does
		std::vector<ImGuiDockNode *> leaves;
		autohide_leaves(ImGui::DockBuilderGetNode(m_dockspace), leaves);
		ImGuiDockNode *neighbour{nullptr};
		for (auto *leaf : leaves)
			if (autohide_edge(leaf, central) == group.edge)
				neighbour = leaf;
		ImGuiID rest{0};
		if (neighbour != nullptr)
		{
			auto const vertical{autohide_vertical(group.edge)};
			auto const direction{vertical ? (group.first ? ImGuiDir_Up : ImGuiDir_Down) : (group.first ? ImGuiDir_Left : ImGuiDir_Right)};
			target = ImGui::DockBuilderSplitNode(neighbour->ID, direction, std::clamp(group.share, 0.15f, 0.85f), nullptr, &rest);
		}
		else
		{
			auto const room{std::max(autohide_vertical(group.edge) ? central->Size.x : central->Size.y, 1.0f)};
			target = ImGui::DockBuilderSplitNode(central->ID, static_cast<ImGuiDir>(group.edge), std::clamp(group.size / room, 0.1f, 0.45f), nullptr, &rest);
		}
	}
	if (target == 0)
		return;
	for (auto const id : group.windows)
		if (auto const *window{ImGui::FindWindowByID(id)}; window != nullptr)
			ImGui::DockBuilderDockWindow(window->Name, target);
	ImGui::DockBuilderFinish(m_dockspace);
	ImGui::MarkIniSettingsDirty();
}

void editor_ui::autohide_show(unsigned int const Window, bool const Focus)
{
	m_autohideshown = Window;
	m_autohideappear = true;
	m_autohidefocus = Focus;
	m_autohideleft = -1.0;
}

void editor_ui::autohide_begin()
{
	auto &g{*GImGui};
	// all docks unpinned, or all pinned back when none is left. the unpinned ones are undocked at the start of the next frame
	auto unpinall{false};
	if (m_dockstoggle)
	{
		m_dockstoggle = false;
		std::vector<ImGuiDockNode *> leaves;
		autohide_leaves(ImGui::DockBuilderGetNode(m_dockspace), leaves);
		unpinall = false == leaves.empty();
		if (false == unpinall)
		{
			for (std::size_t index = 0; index < m_autohide.size(); ++index)
				m_autohidepin.push_back(index);
			std::stable_sort(m_autohidepin.begin(), m_autohidepin.end(), [](std::size_t const Left, std::size_t const Right) { return m_autohide[Left].depth < m_autohide[Right].depth; });
		}
	}
	for (auto const index : m_autohidepin)
		autohide_pin(index);
	m_autohidepin.clear();
	// windows docked again, by the pin or by hand, are pinned
	auto changed{false};
	for (auto &group : m_autohide)
	{
		auto const count{group.windows.size()};
		std::erase_if(group.windows, [](unsigned int const Id) {
			auto const *window{ImGui::FindWindowByID(Id)};
			return window != nullptr && window->DockId != 0;
		});
		changed |= group.windows.size() != count;
	}
	std::erase_if(m_autohide, [](autohide_group const &Group) { return Group.windows.empty(); });
	if (changed)
		ImGui::MarkIniSettingsDirty();
	if (autohide_group_of(m_autohideshown) == static_cast<std::size_t>(-1))
		m_autohideshown = 0;

	// the shown window goes back when the cursor has been away from it for a while, or at once on a click elsewhere;
	// it stays while it's in use: a button held, a field being typed in, a popup open, a drag
	if (auto *shown{m_autohideshown != 0 ? ImGui::FindWindowByID(m_autohideshown) : nullptr}; shown != nullptr && false == m_autohideappear)
	{
		auto const &io{ImGui::GetIO()};
		auto area{shown->Rect()};
		area.Expand(8.0f);
		auto const over{area.Contains(io.MousePos) || m_autohidestrip};
		auto const popup{ImGui::IsPopupOpen("", ImGuiPopupFlags_AnyPopupId | ImGuiPopupFlags_AnyPopupLevel)};
		auto const typing{io.WantTextInput && g.ActiveIdWindow != nullptr && g.ActiveIdWindow->RootWindow == shown};
		auto const busy{ImGui::IsAnyMouseDown() || g.MovingWindow != nullptr || popup || typing || ImGui::IsDragDropActive()};
		if (false == over && false == popup && (ImGui::IsMouseClicked(ImGuiMouseButton_Left) || ImGui::IsMouseClicked(ImGuiMouseButton_Right)))
			m_autohideshown = 0;
		else if (over || busy)
			m_autohideleft = -1.0;
		else if (m_autohideleft < 0.0)
			m_autohideleft = ImGui::GetTime();
		else if (ImGui::GetTime() - m_autohideleft > 0.5)
			m_autohideshown = 0;
	}
	// the rest stay hidden. the count goes down in Begin before it's checked, so it's set to 2 to hide the window in this frame
	for (auto const &group : m_autohide)
		for (auto const id : group.windows)
			if (id != m_autohideshown)
				if (auto *window{ImGui::FindWindowByID(id)}; window != nullptr)
					window->HiddenFramesCanSkipItems = static_cast<ImS8>(std::max<int>(window->HiddenFramesCanSkipItems, 2));
	// after the check above, the windows are still docked until the next frame
	if (unpinall)
	{
		std::vector<ImGuiDockNode *> leaves;
		autohide_leaves(ImGui::DockBuilderGetNode(m_dockspace), leaves);
		for (auto *leaf : leaves)
			autohide_node(leaf);
	}
}

void editor_ui::autohide_place()
{
	auto *shown{m_autohideshown != 0 ? ImGui::FindWindowByID(m_autohideshown) : nullptr};
	if (shown == nullptr)
		return;
	auto &group{m_autohide[autohide_group_of(m_autohideshown)]};
	auto const vertical{autohide_vertical(group.edge)};
	// the size the user gave it by dragging its edge is kept for the next time
	if (false == m_autohideappear)
		group.size = vertical ? shown->SizeFull.x : shown->SizeFull.y;
	auto const room{vertical ? m_viewmax.x - m_viewmin.x : m_viewmax.y - m_viewmin.y};
	auto const size{std::clamp(group.size, std::min(120.0f, room), std::max(room * 0.85f, 1.0f))};
	ImVec2 position{m_viewmin.x, m_viewmin.y};
	ImVec2 extent{vertical ? size : m_viewmax.x - m_viewmin.x, vertical ? m_viewmax.y - m_viewmin.y : size};
	if (group.edge == ImGuiDir_Right)
		position.x = m_viewmax.x - size;
	else if (group.edge == ImGuiDir_Down)
		position.y = m_viewmax.y - size;
	ImGui::SetWindowCollapsed(shown, false, ImGuiCond_Always);
	ImGui::SetWindowSize(shown, extent, ImGuiCond_Always);
	// dragged by its title it may be docked somewhere, it's left to go
	if (GImGui->MovingWindow != shown)
		ImGui::SetWindowPos(shown, position, ImGuiCond_Always);
	if (m_autohideappear)
	{
		m_autohideappear = false;
		ImGui::BringWindowToDisplayFront(shown);
		if (m_autohidefocus)
			ImGui::FocusWindow(shown);
	}
}

void editor_ui::autohide_strips()
{
	auto *viewport{ImGui::GetMainViewport()};
	auto const &style{ImGui::GetStyle()};
	auto const thickness{ImGui::GetFrameHeight()};
	auto const padding{style.FramePadding.x};
	auto hovered{false};
	m_autohidestrip = false;
	for (auto const edge : {ImGuiDir_Left, ImGuiDir_Right, ImGuiDir_Up, ImGuiDir_Down})
	{
		std::vector<ImGuiWindow *> windows;
		for (auto const &group : m_autohide)
		{
			if (group.edge != edge)
				continue;
			for (auto const id : group.windows)
				if (auto *window{ImGui::FindWindowByID(id)}; window != nullptr && window->Active)
					windows.push_back(window);
		}
		if (windows.empty())
			continue;
		auto const vertical{autohide_vertical(edge)};
		ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
		ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
		auto const name{std::string{"##autohidestrip"} + std::to_string(static_cast<int>(edge))};
		if (ImGui::BeginViewportSideBar(name.c_str(), viewport, edge, thickness, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing))
		{
			auto *list{ImGui::GetWindowDrawList()};
			auto position{vec_add(ImGui::GetCursorScreenPos(), vertical ? ImVec2(0.0f, 4.0f) : ImVec2(4.0f, 0.0f))};
			for (auto *window : windows)
			{
				auto const *end{ImGui::FindRenderedTextEnd(window->Name)};
				auto const text{ImGui::CalcTextSize(window->Name, end)};
				ImVec2 const size{vertical ? ImVec2(thickness, text.x + padding * 2.0f) : ImVec2(text.x + padding * 2.0f, thickness)};
				ImGui::SetCursorScreenPos(position);
				ImGui::PushID(static_cast<int>(window->ID));
				ImGui::InvisibleButton("##tab", size);
				ImGui::PopID();
				auto const over{ImGui::IsItemHovered()};
				auto const id{window->ID};
				auto const color{m_autohideshown == id ? ImGuiCol_TabSelected : over ? ImGuiCol_TabHovered : ImGuiCol_Tab};
				list->AddRectFilled(position, vec_add(position, size), ImGui::GetColorU32(color), style.TabRounding);
				if (vertical)
					autohide_vertical_text(list, vec_add(position, ImVec2((thickness - ImGui::GetFontSize()) * 0.5f, padding)), ImGui::GetColorU32(ImGuiCol_Text), window->Name, end);
				else
					list->AddText(vec_add(position, ImVec2(padding, style.FramePadding.y)), ImGui::GetColorU32(ImGuiCol_Text), window->Name, end);
				// a click shows the window and gives it the focus, or puts it away; pointing at the tab for a while shows it too
				if (ImGui::IsItemClicked(ImGuiMouseButton_Left))
				{
					// shown by pointing at it, the click keeps it out and gives it the focus
					if (m_autohideshown == id && m_autohidefocus)
						m_autohideshown = 0;
					else
						autohide_show(id, true);
					m_autohidehovered = id;
					m_autohidehovertime = std::numeric_limits<double>::max();
				}
				else if (over && false == ImGui::IsAnyMouseDown())
				{
					if (m_autohidehovered != id)
					{
						m_autohidehovered = id;
						m_autohidehovertime = ImGui::GetTime();
					}
					else if (m_autohideshown != id && ImGui::GetTime() - m_autohidehovertime >= 0.25)
						autohide_show(id, false);
				}
				if (over)
				{
					hovered = true;
					m_autohidestrip = true;
				}
				position = vec_add(position, vertical ? ImVec2(0.0f, size.y + 4.0f) : ImVec2(size.x + 4.0f, 0.0f));
			}
		}
		ImGui::End();
		ImGui::PopStyleVar(2);
	}
	if (false == hovered)
		m_autohidehovered = 0;
}

void editor_ui::autohide_pins()
{
	auto &g{*GImGui};
	auto const &style{g.Style};
	auto const size{g.FontSize};
	auto const popup{ImGui::IsPopupOpen("", ImGuiPopupFlags_AnyPopupId | ImGuiPopupFlags_AnyPopupLevel)};
	// pins of the docks, in their tab bars by the close button, where the tabs leave room for them; the window menu of a dock
	// (the arrow at the left of its tabs) unpins it too
	std::vector<ImGuiDockNode *> leaves;
	autohide_leaves(ImGui::DockBuilderGetNode(m_dockspace), leaves);
	for (auto *node : leaves)
	{
		if (node->HostWindow == nullptr || false == node->IsVisible || node->IsHiddenTabBar() || node->IsNoTabBar() || node->TabBar == nullptr)
			continue;
		auto const x{node->Pos.x + node->Size.x - style.WindowBorderSize - style.FramePadding.x - (node->HasCloseButton ? size + style.ItemInnerSpacing.x : 0.0f) - size};
		if (node->TabBar->BarRect.Min.x + node->TabBar->WidthAllTabsIdeal + style.ItemInnerSpacing.x > x)
			continue;
		ImRect const rect{ImVec2(x, node->Pos.y + style.FramePadding.y), ImVec2(x + size, node->Pos.y + style.FramePadding.y + size)};
		auto const over{false == popup && g.HoveredWindow != nullptr && g.HoveredWindow->RootWindowDockTree == node->HostWindow->RootWindowDockTree && ImGui::IsMouseHoveringRect(rect.Min, rect.Max, false)};
		auto *list{node->HostWindow->DrawList};
		list->PushClipRect(node->Pos, vec_add(node->Pos, node->Size), false);
		autohide_draw_pin(list, rect, true, over);
		list->PopClipRect();
		if (over)
		{
			ImGui::SetTooltip("%s", STR_C("Unpin: the dock hides at the edge of the window, its tabs show it over the view (Ctrl+Space: all docks)"));
			if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
				autohide_node(node);
		}
	}
	// the pin of the shown window puts its dock back
	auto *shown{m_autohideshown != 0 ? ImGui::FindWindowByID(m_autohideshown) : nullptr};
	if (shown == nullptr || false == shown->Active || shown->Hidden || shown->DockIsActive || (shown->Flags & ImGuiWindowFlags_NoTitleBar) != 0)
		return;
	auto const title{shown->TitleBarRect()};
	auto const x{title.Max.x - style.FramePadding.x - (shown->HasCloseButton ? size + style.ItemInnerSpacing.x : 0.0f) - size};
	ImRect const rect{ImVec2(x, title.Min.y + style.FramePadding.y), ImVec2(x + size, title.Min.y + style.FramePadding.y + size)};
	auto const over{false == popup && g.HoveredWindow == shown && ImGui::IsMouseHoveringRect(rect.Min, rect.Max, false)};
	shown->DrawList->PushClipRect(title.Min, title.Max, false);
	autohide_draw_pin(shown->DrawList, rect, false, over);
	shown->DrawList->PopClipRect();
	if (over)
	{
		ImGui::SetTooltip("%s", STR_C("Pin: the dock goes back to its place"));
		if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
			m_autohidepin.push_back(autohide_group_of(m_autohideshown));
	}
}

void editor_ui::autohide_menu(ImGuiContext *Context, ImGuiDockNode *Node, ImGuiTabBar *TabBar)
{
	ImGui::DockNodeWindowMenuHandler_Default(Context, Node, TabBar);
	if (m_dockspace == 0 || Node == nullptr || Node->IsCentralNode() || ImGui::DockNodeGetRootNode(Node)->ID != m_dockspace)
		return;
	ImGui::Separator();
	if (ImGui::MenuItem(STR_C("Unpin (hide at the edge)")))
		autohide_node(Node);
}

void editor_ui::register_settings()
{
	ImGuiSettingsHandler handler;
	handler.TypeName = "MaSzynaAutoHide";
	handler.TypeHash = ImHashStr(handler.TypeName);
	handler.ClearAllFn = [](ImGuiContext *, ImGuiSettingsHandler *) { m_autohide.clear(); };
	handler.ReadOpenFn = [](ImGuiContext *, ImGuiSettingsHandler *, char const *) -> void * {
		m_autohide.emplace_back();
		return &m_autohide.back();
	};
	handler.ReadLineFn = [](ImGuiContext *, ImGuiSettingsHandler *, void *Entry, char const *Line) {
		auto &group{*static_cast<autohide_group *>(Entry)};
		int edge{0};
		unsigned int id{0};
		// whole numbers only, the decimal point of the locale doesn't matter then
		if (std::sscanf(Line, "Edge=%d", &edge) == 1)
			group.edge = std::clamp(edge, static_cast<int>(ImGuiDir_Left), static_cast<int>(ImGuiDir_Down));
		else if (std::sscanf(Line, "Size=%d", &edge) == 1)
			group.size = static_cast<float>(edge);
		else if (std::sscanf(Line, "First=%d", &edge) == 1)
			group.first = edge != 0;
		else if (std::sscanf(Line, "Share=%d", &edge) == 1)
			group.share = edge / 1000.0f;
		else if (std::sscanf(Line, "Depth=%d", &edge) == 1)
			group.depth = edge;
		else if (std::sscanf(Line, "Window=0x%X", &id) == 1)
			group.windows.push_back(id);
	};
	handler.ApplyAllFn = [](ImGuiContext *, ImGuiSettingsHandler *) {
		std::erase_if(m_autohide, [](autohide_group const &Group) { return Group.windows.empty(); });
	};
	handler.WriteAllFn = [](ImGuiContext *, ImGuiSettingsHandler *Handler, ImGuiTextBuffer *Buffer) {
		for (auto const &group : m_autohide)
		{
			if (group.windows.empty())
				continue;
			Buffer->appendf("[%s][Dock]\nEdge=%d\nSize=%d\nFirst=%d\nShare=%d\nDepth=%d\n", Handler->TypeName, group.edge, static_cast<int>(group.size), group.first ? 1 : 0, static_cast<int>(group.share * 1000.0f), group.depth);
			for (auto const id : group.windows)
				Buffer->appendf("Window=0x%08X\n", id);
			Buffer->append("\n");
		}
	};
	ImGui::AddSettingsHandler(&handler);
}

void editor_ui::render_()
{
	auto *viewport{ImGui::GetMainViewport()};
	auto const flags{ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_MenuBar};
	// toolbar under the menu: the tools of the editor mode
	if (ImGui::BeginViewportSideBar("##editortoolbar", viewport, ImGuiDir_Up, ImGui::GetFrameHeight(), flags))
	{
		if (ImGui::BeginMenuBar())
		{
			if (m_toolbaroptions)
				m_toolbaroptions();
			ImGui::EndMenuBar();
		}
	}
	ImGui::End();
	render_tool_options();
	render_inspector();
	render_scene();
	// status bar: the outcome of the last operation, and the selected node
	if (ImGui::BeginViewportSideBar("##editorstatusbar", viewport, ImGuiDir_Down, ImGui::GetFrameHeight(), flags))
	{
		if (ImGui::BeginMenuBar())
		{
			if (false == m_layerspanel.status.empty())
			{
				if (m_layerspanel.status_error)
					ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.4f, 1.0f), "%s", m_layerspanel.status.c_str());
				else
					ImGui::TextUnformatted(m_layerspanel.status.c_str());
			}
			if (m_node != nullptr)
			{
				auto const name{m_node->name().empty() ? std::string{"(none)"} : m_node->name()};
				auto const label{std::string{STR_C("Selected:")} + " " + name};
				ImGui::SameLine(std::max(ImGui::GetCursorPosX(), ImGui::GetWindowWidth() - ImGui::CalcTextSize(label.c_str()).x - ImGui::GetStyle().ItemSpacing.x * 2.0f));
				ImGui::TextUnformatted(label.c_str());
			}
			ImGui::EndMenuBar();
		}
	}
	ImGui::End();
	// strips of the unpinned docks, inside the toolbar and the status bar; the pins over the tab bars drawn by now
	autohide_strips();
	autohide_pins();
}

// updates state of UI elements
void editor_ui::update()
{

	set_tooltip("");

	if (Global.ControlPicking && DebugModeFlag)
	{
		const auto sceneryNode = GfxRenderer->Pick_Node();
		const std::string content = sceneryNode ? sceneryNode->tooltip() : "";
		set_tooltip(content);
	}

	ui_layer::update();
	m_itempropertiespanel.update(m_node);
	m_functionspanel.update(m_node);
}

void editor_ui::render_mode_options(nodebank_panel::edit_mode const Mode)
{
	switch (Mode)
	{
	case nodebank_panel::MODIFY:
		ImGui::TextDisabled(STR_C("LMB: select node   F: focus   End: drop to ground   Del: delete"));
		break;
	case nodebank_panel::COPY:
		ImGui::TextDisabled(STR_C("LMB: copy the clicked model to the node bank"));
		break;
	case nodebank_panel::ADD:
	{
		ImGui::TextDisabled(STR_C("LMB: insert a model at the cursor"));
		ImGui::Checkbox(STR_C("Random model from set"), &m_insertrandom);
		if (m_insertrandom)
		{
			m_nodebankpanel.set_combo(STR_C("Set##insert"), m_insertset, nullptr);
			auto const count{m_nodebankpanel.set_entries(m_insertset).size()};
			ImGui::TextDisabled(STR_C("%zu templates in set%s"), count, count == 0 ? ", the node bank selection is used" : "");
		}
		else
		{
			ImGui::TextDisabled(STR_C("Inserts the template selected in the node bank"));
		}
		m_functionspanel.render_controls();
		break;
	}
	case nodebank_panel::BRUSH:
		ImGui::TextDisabled(STR_C("Hold LMB: paint models along the cursor path"));
		m_brushobjects.render_options(m_nodebankpanel);
		m_functionspanel.render_controls();
		break;
	case nodebank_panel::FILL:
		if (m_filloptions)
			m_filloptions();
		break;
	default:
		break;
	}
}

void editor_ui::render_tool_options()
{
	if (false == m_tooloptionsopen)
		return;
	ImGui::SetNextWindowSize(ImVec2S(400, 300), ImGuiCond_FirstUseEver);
	auto const current{mode()};
	auto const active{m_workspace == workspace::terrain || (m_workspace == workspace::roads && current != nodebank_panel::TRACK) ? m_workspace : workspace::none};
	// a change of the mode brings the window forward, in case it shares its dock with another one
	auto const shown{active != workspace::none ? 100 + static_cast<int>(active) : static_cast<int>(current)};
	if (shown != m_toolmode)
	{
		m_toolmode = shown;
		ImGui::SetNextWindowFocus();
	}
	if (ImGui::Begin((std::string(STR_C("Tool options")) + "###tooloptions").c_str(), &m_tooloptionsopen, ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoCollapse))
	{
		char const *name{STR_C("Select")};
		switch (current)
		{
		case nodebank_panel::ADD:
			name = STR_C("Insert");
			break;
		case nodebank_panel::BRUSH:
			name = STR_C("Brush");
			break;
		case nodebank_panel::FILL:
			name = STR_C("Area fill");
			break;
		case nodebank_panel::COPY:
			name = STR_C("Copy to bank");
			break;
		case nodebank_panel::TRACK:
			name = STR_C("Tracks");
			break;
		default:
			break;
		}
		if (active == workspace::roads)
			name = "Roads";
		else if (active == workspace::terrain)
			name = STR_C("Terrain");
		ImGui::SeparatorText(name);
		if (active != workspace::none)
		{
			auto const &tools{m_workspacetools[static_cast<std::size_t>(active)]};
			if (tools)
				tools();
		}
		else if (current == nodebank_panel::TRACK)
		{
			if (m_tracktools)
				m_tracktools();
		}
		else
			render_mode_options(current);
		if (ImGui::CollapsingHeader(STR_C("Gizmo"), ImGuiTreeNodeFlags_DefaultOpen))
		{
			if (m_gizmooptions)
				m_gizmooptions();
		}
	}
	ImGui::End();
}

void editor_ui::render_inspector()
{
	if (false == m_inspectoropen)
		return;
	ImGui::SetNextWindowSize(ImVec2S(420, 480), ImGuiCond_FirstUseEver);
	if (ImGui::Begin((std::string(STR_C("Inspector")) + "###inspector").c_str(), &m_inspectoropen, ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoCollapse))
	{
		if (mode() == nodebank_panel::TRACK)
		{
			// in the track mode the selection is a path
			if (m_trackselection)
				m_trackselection();
			ImGui::End();
			return;
		}
		if (auto const &selection{m_workspaceselection[static_cast<std::size_t>(m_workspace)]})
		{
			selection();
			ImGui::End();
			return;
		}
		if (ImGui::CollapsingHeader(STR_C("Node properties"), ImGuiTreeNodeFlags_DefaultOpen))
		{
			ImGui::Indent();
			m_itempropertiespanel.render_body();
			ImGui::Unindent();
		}
		if (ImGui::CollapsingHeader(STR_C("Array")))
		{
			if (m_arrayoptions)
				m_arrayoptions();
		}
		if (m_bendexpand)
		{
			ImGui::SetNextItemOpen(true);
			m_bendexpand = false;
		}
		if (ImGui::CollapsingHeader(STR_C("Bend along the track")))
		{
			if (m_bendoptions)
				m_bendoptions();
		}
	}
	ImGui::End();
}

void editor_ui::scene_rebuild()
{
	m_sceneentries.clear();
	m_sceneentries.reserve(scene::Hierarchy.size());
	for (auto const &entry : scene::Hierarchy)
	{
		if (entry.second == nullptr)
			continue;
		m_sceneentries.push_back({entry.first, entry.second->layer(), entry.second->group(), entry.second->name()});
	}
	std::sort(m_sceneentries.begin(), m_sceneentries.end(), [](scene_entry const &Left, scene_entry const &Right) {
		if (Left.layer != Right.layer)
			return Left.layer < Right.layer;
		if (Left.group != Right.group)
			return Left.group < Right.group;
		return Left.name < Right.name;
	});
	m_scenesize = scene::Hierarchy.size();
	m_scenefilterused = "\x01"; // matches are found again
}

void editor_ui::scene_row(std::size_t const Entry)
{
	auto const &entry{m_sceneentries[Entry]};
	// the node is looked up again, it may have been removed since the list was made
	auto const lookup{scene::Hierarchy.find(entry.uuid)};
	auto *node{lookup != scene::Hierarchy.end() ? lookup->second : nullptr};
	if (node == nullptr)
	{
		ImGui::TextDisabled("%s", entry.name.c_str());
		return;
	}
	auto const label{(node->name().empty() ? std::string{"(none)"} : node->name()) + "##" + entry.uuid};
	if (ImGui::Selectable(label.c_str(), node == m_node, ImGuiSelectableFlags_AllowDoubleClick) && m_sceneselect)
		m_sceneselect(node, ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left));
	if (ImGui::IsItemHovered())
	{
		auto const location{node->location()};
		ImGui::SetTooltip("%s\n%.1f, %.1f, %.1f\n%s", entry.uuid.c_str(), location.x, location.y, location.z, STR_C("Double click: fly to it"));
	}
}

void editor_ui::render_scene()
{
	if (false == m_sceneopen)
		return;
	ImGui::SetNextWindowSize(ImVec2S(400, 420), ImGuiCond_FirstUseEver);
	if (ImGui::Begin((std::string(STR_C("Scene")) + "###scenetree").c_str(), &m_sceneopen, ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoCollapse))
	{
		if (m_scenesize != scene::Hierarchy.size())
			scene_rebuild();
		if (ImGui::Button(STR_C("Refresh")))
			scene_rebuild();
		ImGui::SameLine();
		ImGui::SetNextItemWidth(-FLT_MIN);
		ImGui::InputTextWithHint("##scenefilter", STR_C("Filter: name or uuid"), m_scenefilter, IM_ARRAYSIZE(m_scenefilter));
		ImGui::TextDisabled(STR_C("%zu models"), m_sceneentries.size());
		if (ImGui::BeginChild("##scenelist", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders))
		{
			std::string const filter{m_scenefilter};
			if (false == filter.empty())
			{
				// a flat list of what matches the filter
				if (filter != m_scenefilterused)
				{
					m_scenematches.clear();
					for (std::size_t idx = 0; idx < m_sceneentries.size(); ++idx)
						if (contains(m_sceneentries[idx].name, filter) || contains(m_sceneentries[idx].uuid, filter))
							m_scenematches.push_back(idx);
					m_scenefilterused = filter;
				}
				ImGuiListClipper clipper;
				clipper.Begin(static_cast<int>(m_scenematches.size()));
				while (clipper.Step())
					for (int row = clipper.DisplayStart; row < clipper.DisplayEnd; ++row)
						scene_row(m_scenematches[row]);
			}
			else
			{
				// layers, in them the groups (made by the includes), in these the models
				std::size_t layerstart{0};
				while (layerstart < m_sceneentries.size())
				{
					auto const layer{m_sceneentries[layerstart].layer};
					auto layerend{layerstart};
					while (layerend < m_sceneentries.size() && m_sceneentries[layerend].layer == layer)
						++layerend;
					auto const layername{scene::Layers.valid(layer) ? scene::Layers.layer(layer).name : std::string{STR_C("(no layer)")}};
					auto const layerlabel{layername + " (" + std::to_string(layerend - layerstart) + ")##layer" + std::to_string(layer)};
					if (ImGui::TreeNodeEx(layerlabel.c_str(), m_sceneentries.size() == layerend - layerstart ? ImGuiTreeNodeFlags_DefaultOpen : 0))
					{
						auto groupstart{layerstart};
						while (groupstart < layerend)
						{
							auto const group{m_sceneentries[groupstart].group};
							auto groupend{groupstart};
							while (groupend < layerend && m_sceneentries[groupend].group == group)
								++groupend;
							auto const rows = [&](std::size_t const From, std::size_t const To) {
								ImGuiListClipper clipper;
								clipper.Begin(static_cast<int>(To - From));
								while (clipper.Step())
									for (int row = clipper.DisplayStart; row < clipper.DisplayEnd; ++row)
										scene_row(From + row);
							};
							if (group == null_handle)
							{
								rows(groupstart, groupend);
							}
							else
							{
								auto const grouplabel{std::string{STR_C("Group")} + " " + std::to_string(group) + " (" + std::to_string(groupend - groupstart) + ")##group" + std::to_string(group)};
								if (ImGui::TreeNode(grouplabel.c_str()))
								{
									rows(groupstart, groupend);
									ImGui::TreePop();
								}
							}
							groupstart = groupend;
						}
						ImGui::TreePop();
					}
					layerstart = layerend;
				}
			}
		}
		ImGui::EndChild();
	}
	ImGui::End();
}

void editor_ui::render_menu_contents()
{
	if (ImGui::BeginMenu(STR_C("File")))
	{
		if (ImGui::MenuItem(STR_C("New scenery...")) && m_newscenery)
			m_newscenery();
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("The editor starts again with the wizard of a new scenery: its name and its centre on the map"));
		if (ImGui::MenuItem(STR_C("Open scenery...")) && m_openscenery)
			m_openscenery();
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("The editor starts again with another scenery of the scenery folder"));
		// changes go to the scenery files only if the scenery was loaded with its sources tracked
		auto const editsession{false == scene::Layers.empty()};
		if (ImGui::MenuItem(STR_C("Save"), STR_C("Ctrl+S"), false, editsession) && m_save)
		{
			m_save();
		}
		if (false == editsession && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
		{
			ImGui::SetTooltip("%s", STR_C("Start the simulator with -edit <scenery file> to save changes to the scenery files"));
		}
		if (ImGui::MenuItem(STR_C("Export scenery dump"), STR_C("Ctrl+Shift+F11")) && m_export)
		{
			m_export();
		}
		if (ImGui::IsItemHovered())
		{
			ImGui::SetTooltip("%s", STR_C("Writes the whole scene to _export files next to the scenario, the scenery files are left as they are"));
		}
		ImGui::EndMenu();
	}

	ui_layer::render_menu_contents();

	if (ImGui::BeginMenu(STR_C("Mode windows")))
	{
		if (ImGui::MenuItem(STR_C("Reset window layout")))
			m_layoutreset = true;
		std::vector<ImGuiDockNode *> pinned;
		autohide_leaves(ImGui::DockBuilderGetNode(m_dockspace), pinned);
		if (ImGui::MenuItem(pinned.empty() ? STR_C("Pin the docks back") : STR_C("Unpin all docks"), "Ctrl+Space", false, false == pinned.empty() || false == m_autohide.empty()))
			toggle_docks();
		ImGui::Separator();
		ImGui::MenuItem(STR_C("Tool options"), nullptr, &m_tooloptionsopen);
		ImGui::MenuItem(STR_C("Inspector"), nullptr, &m_inspectoropen);
		ImGui::MenuItem(STR_C("Scene"), nullptr, &m_sceneopen);
		ImGui::MenuItem(STR_C("Node bank"), nullptr, &m_nodebankpanel.is_open);
		ImGui::MenuItem(STR_C("Layers"), nullptr, &m_layerspanel.is_open);
		ImGui::MenuItem(STR_C("Include database"), nullptr, &m_includespanel.is_open);
		ImGui::EndMenu();
	}

	if (m_menuoptions)
		m_menuoptions();
}

void editor_ui::set_file_actions(std::function<void()> Save, std::function<void()> Export)
{
	m_save = Save;
	m_export = std::move(Export);
	m_layerspanel.save = std::move(Save);
}

void editor_ui::set_status(std::string const &Status, bool const Error)
{
	m_layerspanel.status = Status;
	m_layerspanel.status_error = Error;
}

void editor_ui::render_rotation_controls()
{
	m_functionspanel.render_controls();
}

void editor_ui::set_node(scene::basic_node *Node)
{
	m_node = Node;
	m_itempropertiespanel.update(m_node);
	m_functionspanel.update(m_node);
}

void editor_ui::set_include(include_selection *Include)
{
	m_itempropertiespanel.include(Include);
}

void editor_ui::add_node_template(const std::string &desc)
{
	m_nodebankpanel.add_template(desc);
}

std::string const *editor_ui::get_active_node_template(bool bypassRandom)
{
	if (!bypassRandom)
	{
		// random pick from the active tool's set; an empty set falls back to the node bank selection
		std::string const *pick{nullptr};
		if (m_nodebankpanel.mode == nodebank_panel::BRUSH && m_brushobjects.useRandom)
		{
			if (m_brushobjects.source.kind == model_set_ref::source::manual)
				pick = m_brushobjects.Objects.empty() ? nullptr : m_brushobjects.GetRandomObject();
			else
				pick = m_nodebankpanel.random_template(m_brushobjects.source);
		}
		else if (m_nodebankpanel.mode == nodebank_panel::ADD && m_insertrandom)
		{
			pick = m_nodebankpanel.random_template(m_insertset);
		}
		if (pick != nullptr)
			return pick;
	}
	return m_nodebankpanel.get_active_template();
}

nodebank_panel::edit_mode editor_ui::mode()
{
	return m_track ? nodebank_panel::TRACK : m_nodebankpanel.mode;
}
void editor_ui::set_mode(nodebank_panel::edit_mode const Mode)
{
	m_nodebankpanel.mode = Mode;
	m_nodebankpanel.is_open = true;
}
float editor_ui::getSpacing()
{
	return m_brushobjects.spacing;
}

functions_panel::rotation_mode editor_ui::rot_mode()
{
	return m_functionspanel.rot_mode;
}
float editor_ui::rot_val()
{
	return m_functionspanel.rot_value;
}
bool editor_ui::rot_from_last()
{
	return m_functionspanel.rot_from_last;
}