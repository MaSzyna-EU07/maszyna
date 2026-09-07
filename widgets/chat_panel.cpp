/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "widgets/chat_panel.h"

#include "application/application.h"
#include "network/chat.h"
#include "network/entities.h"
#include "utilities/Globals.h"
#include "utilities/translation.h"

namespace
{

// keeps the key that opens the panel from ending up in the line being typed
int filter_input(ImGuiInputTextCallbackData *Data)
{
	if (Data->EventChar == '`' || Data->EventChar == '~')
		return 1;
	return 0;
}

} // namespace

ui::chat_panel::chat_panel()
    : ui_panel(STR_C("Chat"), false)
{
	// both bounds have to be set, or imgui gets a maximum below the minimum
	size_min = {380, 200};
	size_max = {900, 600};
}

void ui::chat_panel::focus_input()
{
	m_focusrequested = true;
}

void ui::chat_panel::submit()
{
	std::string const text(m_input.data());
	m_input.fill('\0');

	if (text.empty())
		return;

	Application.say(text);
}

void ui::chat_panel::render_contents()
{
	if (!network::is_multiplayer())
	{
		ImGui::TextUnformatted(STR_C("This is not a multiplayer session."));
		return;
	}

	auto const &log = network::chat_log();

	// the conversation, with room left underneath for the box it is typed into
	float const lineheight = ImGui::GetTextLineHeightWithSpacing();
	ImGui::BeginChild("chatlog", ImVec2(0.0f, -(lineheight + ImGui::GetStyle().ItemSpacing.y * 2.0f)), false,
	                  ImGuiWindowFlags_HorizontalScrollbar);

	for (auto const &line : log)
	{
		bool const own{line.author == Global.network_peer_id};
		ImGui::TextColored(own ? ImVec4(0.6f, 0.85f, 1.0f, 1.0f) : ImVec4(1.0f, 0.85f, 0.55f, 1.0f), "%s:", line.name.c_str());
		ImGui::SameLine();
		ImGui::TextWrapped("%s", line.text.c_str());
	}

	// follow the conversation, but only when something new has actually arrived - dragging
	// the scrollbar back to read something should not be undone every frame
	if (network::chat_serial() != m_lastserial)
	{
		m_lastserial = network::chat_serial();
		ImGui::SetScrollHereY(1.0f);
	}

	ImGui::EndChild();

	if (m_focusrequested)
	{
		ImGui::SetKeyboardFocusHere();
		m_focusrequested = false;
	}

	ImGui::PushItemWidth(-1.0f);
	if (ImGui::InputText("##chatinput", m_input.data(), m_input.size(),
	                     ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_CallbackCharFilter, filter_input))
	{
		submit();
		// staying in the box is what anybody expects after sending a line
		m_focusrequested = true;
	}
	ImGui::PopItemWidth();

	// while the box has the keyboard, imgui swallows the key that opened the panel, so
	// there has to be another way out of it
	if (ImGui::IsKeyPressed(ImGui::GetIO().KeyMap[ImGuiKey_Escape]))
	{
		is_open = false;
	}
}
