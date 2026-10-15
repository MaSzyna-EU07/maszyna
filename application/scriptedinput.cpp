/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/scriptedinput.h"

#include "application/application.h"
#include "application/uilayer.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/utilities.h"
#include "editor/editorFormat.hpp"
#include "imgui/imgui.h"

#include <cctype>
#include <cstdlib>
#include <fstream>
#include <map>

namespace
{

int key_named(std::string const &Name)
{
	static std::map<std::string, int> const names{{"ctrl", GLFW_KEY_LEFT_CONTROL}, {"shift", GLFW_KEY_LEFT_SHIFT}, {"alt", GLFW_KEY_LEFT_ALT}, {"esc", GLFW_KEY_ESCAPE}, {"enter", GLFW_KEY_ENTER}, {"backspace", GLFW_KEY_BACKSPACE},
	                                              {"del", GLFW_KEY_DELETE}, {"space", GLFW_KEY_SPACE}, {"tab", GLFW_KEY_TAB}};
	if (auto const found{names.find(Name)}; found != names.end())
		return found->second;
	if (Name.size() == 1 && std::isalpha(static_cast<unsigned char>(Name[0])))
		return GLFW_KEY_A + (std::toupper(static_cast<unsigned char>(Name[0])) - 'A');
	if (Name.size() == 1 && std::isdigit(static_cast<unsigned char>(Name[0])))
		return GLFW_KEY_0 + (Name[0] - '0');
	if (Name.size() > 1 && (Name[0] == 'f' || Name[0] == 'F') && std::isdigit(static_cast<unsigned char>(Name[1])))
		return GLFW_KEY_F1 + std::stoi(Name.substr(1)) - 1;
	return GLFW_KEY_UNKNOWN;
}

int modifier_of(int const Key)
{
	return Key == GLFW_KEY_LEFT_CONTROL ? GLFW_MOD_CONTROL : Key == GLFW_KEY_LEFT_SHIFT ? GLFW_MOD_SHIFT : Key == GLFW_KEY_LEFT_ALT ? GLFW_MOD_ALT : 0;
}

} // namespace

void scripted_input::modifiers() const
{
	auto &io{ImGui::GetIO()};
	io.AddKeyEvent(ImGuiMod_Ctrl, (m_mods & GLFW_MOD_CONTROL) != 0);
	io.AddKeyEvent(ImGuiMod_Shift, (m_mods & GLFW_MOD_SHIFT) != 0);
	io.AddKeyEvent(ImGuiMod_Alt, (m_mods & GLFW_MOD_ALT) != 0);
}

void scripted_input::mouse_to(glm::dvec2 const &Screen)
{
	if (false == m_mouse.has_value())
		ui_layer::cursor_enter_callback(1);
	m_mouse = Screen;
	ui_layer::cursor_pos_callback(Screen.x, Screen.y);
	Global.cursor_pos = glm::ivec2(Screen);
	Application.on_cursor_pos(Screen.x, Screen.y);
}

bool scripted_input::dispatch(std::string const &Command, std::istringstream &Words)
{
	if (Command == "mouse")
	{
		glm::dvec2 screen{0.0};
		Words >> screen.x >> screen.y;
		mouse_to(screen);
		WriteLog(format("SELFTEST mouse %.0f %.0f", screen.x, screen.y));
		return true;
	}
	if (Command == "button")
	{
		std::string which;
		std::string action;
		Words >> which >> action;
		auto const button{which == "right" ? GLFW_MOUSE_BUTTON_RIGHT : which == "middle" ? GLFW_MOUSE_BUTTON_MIDDLE : GLFW_MOUSE_BUTTON_LEFT};
		Application.on_mouse_button(button, action == "release" ? GLFW_RELEASE : GLFW_PRESS, m_mods);
		modifiers();
		return true;
	}
	if (Command == "wheel")
	{
		double offset{0.0};
		Words >> offset;
		Application.on_scroll(0.0, offset);
		modifiers();
		return true;
	}
	if (Command == "press" || Command == "release" || Command == "tap")
	{
		std::string name;
		Words >> name;
		auto const key{key_named(name)};
		if (key == GLFW_KEY_UNKNOWN)
		{
			WriteLog("SELFTEST " + Command + ": unknown key " + name);
			return true;
		}
		if (Command != "release")
		{
			m_mods |= modifier_of(key);
			Application.on_key(key, 0, GLFW_PRESS, m_mods);
			modifiers();
		}
		if (Command != "press")
		{
			m_mods &= ~modifier_of(key);
			Application.on_key(key, 0, GLFW_RELEASE, m_mods);
			modifiers();
		}
		return true;
	}
	if (Command == "type")
	{
		std::string text;
		std::getline(Words >> std::ws, text);
		for (std::size_t index = 0; index < text.size();)
		{
			auto const lead{static_cast<unsigned char>(text[index])};
			auto const extra{lead >= 0xF0 ? 3 : lead >= 0xE0 ? 2 : lead >= 0xC0 ? 1 : 0};
			unsigned int code{extra == 0 ? lead : lead & (0x3Fu >> extra)};
			for (int k = 1; k <= extra && index + k < text.size(); ++k)
				code = (code << 6) | (static_cast<unsigned char>(text[index + k]) & 0x3Fu);
			Application.on_char(code);
			index += extra + 1;
		}
		return true;
	}
	if (Command == "softcursor")
	{
		int on{1};
		Words >> on;
		ImGui::GetIO().MouseDrawCursor = on != 0;
		return true;
	}
	return false;
}

void input_script::step()
{
	if (false == m_loaded)
	{
		m_loaded = true;
		auto const *file{std::getenv(m_variable)};
		if (file == nullptr)
			return;
		std::ifstream stream(file);
		std::string line;
		while (std::getline(stream, line))
			if (false == line.empty() && line.front() != '#')
				m_lines.push_back(line);
		WriteLog("SELFTEST: " + std::to_string(m_lines.size()) + " steps from " + file);
	}
	if (m_wait > 0)
	{
		--m_wait;
		return;
	}
	while (m_next < m_lines.size())
	{
		std::istringstream words(m_lines[m_next++]);
		std::string command;
		words >> command;
		if (command == "wait")
		{
			words >> m_wait;
			return;
		}
		if (command == "screenshot")
		{
			Application.queue_screenshot();
			m_wait = 3;
			return;
		}
		if (command == "log")
		{
			std::string text;
			std::getline(words >> std::ws, text);
			WriteLog("SELFTEST " + text);
			continue;
		}
		if (command == "editorscript")
		{
			std::string file;
			words >> file;
#ifdef _WIN32
			_putenv_s("EU07_EDITOR_SELFTEST", file.c_str());
#else
			setenv("EU07_EDITOR_SELFTEST", file.c_str(), 1);
#endif
			WriteLog("SELFTEST editor script " + file);
			continue;
		}
		if (command == "quit")
		{
			if (false == m_quitting)
			{
				WriteLog("SELFTEST done");
				m_quitting = true;
				m_wait = 30;
				--m_next;
				return;
			}
			Application.queue_quit(true);
			return;
		}
		if (false == m_input.dispatch(command, words))
			WriteLog("SELFTEST unknown command " + command);
	}
}
