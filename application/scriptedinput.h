/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <functional>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <glm/glm.hpp>

// the file of the self-test script the variable names, or none. a build made without WITH_EDITOR_SELFTESTS reads no
// script, so nothing takes over the input of a player
char const *selftest_script(char const *Variable);

class scripted_input
{
  public:
	bool dispatch(std::string const &Command, std::istringstream &Words);
	void deliver();
	void mouse_to(glm::dvec2 const &Screen);
	std::optional<glm::dvec2> const &mouse() const
	{
		return m_mouse;
	}
	static bool driving()
	{
		return s_driving;
	}

  private:
	void queue(std::function<void()> Event);
	std::optional<glm::dvec2> m_mouse;
	int m_mods{0};
	std::vector<std::function<void()>> m_events;
	static inline bool s_driving{false};
};

class input_script
{
  public:
	explicit input_script(char const *Variable) : m_variable{Variable} {}
	void step();
	void deliver()
	{
		m_input.deliver();
	}

  private:
	char const *m_variable;
	bool m_loaded{false};
	std::vector<std::string> m_lines;
	std::size_t m_next{0};
	int m_wait{0};
	bool m_quitting{false};
	scripted_input m_input;
};
