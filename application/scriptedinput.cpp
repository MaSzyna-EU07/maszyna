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
#include <cstddef>
#include <cstdlib>
#include <format>
#include <fstream>
#include <functional>
#include <map>

namespace
{

int key_named(std::string const &Name)
{
	static std::map<std::string, int, std::less<>> const names{{"ctrl", GLFW_KEY_LEFT_CONTROL}, {"shift", GLFW_KEY_LEFT_SHIFT}, {"alt", GLFW_KEY_LEFT_ALT}, {"esc", GLFW_KEY_ESCAPE}, {"enter", GLFW_KEY_ENTER}, {"backspace", GLFW_KEY_BACKSPACE},
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
	if (Key == GLFW_KEY_LEFT_CONTROL)
		return GLFW_MOD_CONTROL;
	if (Key == GLFW_KEY_LEFT_SHIFT)
		return GLFW_MOD_SHIFT;
	if (Key == GLFW_KEY_LEFT_ALT)
		return GLFW_MOD_ALT;
	return 0;
}

int button_named(std::string const &Name)
{
	if (Name == "right")
		return GLFW_MOUSE_BUTTON_RIGHT;
	if (Name == "middle")
		return GLFW_MOUSE_BUTTON_MIDDLE;
	return GLFW_MOUSE_BUTTON_LEFT;
}

// how many bytes follow the lead one of a utf-8 sequence
int utf8_followers(std::byte const Lead)
{
	if (Lead >= std::byte{0xF0})
		return 3;
	if (Lead >= std::byte{0xE0})
		return 2;
	if (Lead >= std::byte{0xC0})
		return 1;
	return 0;
}

// the code points of a utf-8 text, a sequence the end cuts short taken as far as it goes
std::vector<unsigned int> code_points(std::string const &Text)
{
	std::vector<unsigned int> codes;
	for (std::size_t index = 0; index < Text.size();)
	{
		auto const lead{std::byte{static_cast<unsigned char>(Text[index])}};
		auto const extra{utf8_followers(lead)};
		auto code{std::to_integer<unsigned int>(extra == 0 ? lead : lead & (std::byte{0x3F} >> extra))};
		for (int k = 1; k <= extra && index + k < Text.size(); ++k)
			code = (code << 6) | std::to_integer<unsigned int>(std::byte{static_cast<unsigned char>(Text[index + k])} & std::byte{0x3F});
		codes.push_back(code);
		index += extra + 1;
	}
	return codes;
}

// the steps of a script file, its empty lines and its comments left out
void read_steps(char const *File, std::vector<std::string> &Steps)
{
	std::ifstream stream(File);
	std::string line;
	while (std::getline(stream, line))
		if (false == line.empty() && line.front() != '#')
			Steps.push_back(line);
}

void modifiers(int const Mods)
{
	auto &io{ImGui::GetIO()};
	io.AddKeyEvent(ImGuiMod_Ctrl, (Mods & GLFW_MOD_CONTROL) != 0);
	io.AddKeyEvent(ImGuiMod_Shift, (Mods & GLFW_MOD_SHIFT) != 0);
	io.AddKeyEvent(ImGuiMod_Alt, (Mods & GLFW_MOD_ALT) != 0);
}

} // namespace

char const *selftest_script([[maybe_unused]] char const *Variable)
{
#if WITH_EDITOR_SELFTESTS
	return std::getenv(Variable);
#else
	return nullptr;
#endif
}

void scripted_input::queue(std::function<void()> Event)
{
	s_driving = true;
	m_events.emplace_back(std::move(Event));
}

void scripted_input::deliver()
{
	auto const events{std::move(m_events)};
	m_events.clear();
	for (auto const &pending : events)
		pending();
}

void scripted_input::mouse_to(glm::dvec2 const &Screen)
{
	bool const entered{m_mouse.has_value()};
	m_mouse = Screen;
	queue([Screen, entered]() {
		if (false == entered)
			ui_layer::cursor_enter_callback(1);
		ui_layer::cursor_pos_callback(Screen.x, Screen.y);
		Global.cursor_pos = glm::ivec2(Screen);
		Application.on_cursor_pos(Screen.x, Screen.y);
	});
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
	if (Command == "leave")
	{
		m_mouse.reset();
		queue([]() { ui_layer::cursor_enter_callback(0); });
		WriteLog("SELFTEST leave");
		return true;
	}
	if (Command == "button")
	{
		std::string which;
		std::string action;
		Words >> which >> action;
		auto const button{button_named(which)};
		auto const state{action == "release" ? GLFW_RELEASE : GLFW_PRESS};
		queue([button, state, mods = m_mods]() {
			Application.on_mouse_button(button, state, mods);
			modifiers(mods);
		});
		return true;
	}
	if (Command == "wheel")
	{
		double offset{0.0};
		Words >> offset;
		queue([offset, mods = m_mods]() {
			Application.on_scroll(0.0, offset);
			modifiers(mods);
		});
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
			queue([key, mods = m_mods]() {
				Application.on_key(key, 0, GLFW_PRESS, mods);
				modifiers(mods);
			});
		}
		if (Command != "press")
		{
			m_mods &= ~modifier_of(key);
			queue([key, mods = m_mods]() {
				Application.on_key(key, 0, GLFW_RELEASE, mods);
				modifiers(mods);
			});
		}
		return true;
	}
	if (Command == "type")
	{
		std::string text;
		std::getline(Words >> std::ws, text);
		for (auto const code : code_points(text))
			queue([code]() { Application.on_char(code); });
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
		auto const *file{selftest_script(m_variable)};
		if (file == nullptr)
			return;
		read_steps(file, m_lines);
		WriteLog(std::format("SELFTEST: {} steps from {}", m_lines.size(), file));
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
