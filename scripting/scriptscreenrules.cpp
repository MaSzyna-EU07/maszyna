/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "scripting/scriptscreenrules.h"

#include "utilities/parser.h"

#include <algorithm>
#include <cmath>

namespace script_screens
{

void screen_config::deserialize(cParser &Input)
{
	while (deserialize_mapping(Input))
	{
		// all work done by while()
	}
}

bool screen_config::deserialize_mapping(cParser &Input)
{
	// token can be a key or block end
	auto const key{Input.getToken<std::string>(true, "\n\r\t  ,;[]")};

	if (key.empty() || key == "}")
	{
		return false;
	}

	if (key == "{")
	{
		script = Input.getToken<std::string>();
	}
	else if (key == "target:")
	{
		target = Input.getToken<std::string>();
	}
	else if (key == "frames:")
	{
		frames = Input.getToken<int>();
	}
	else if (key == "parameters:")
	{
		parameters = Input.getToken<std::string>();
	}
	else if (key == "instance:")
	{
		instancing = Input.getToken<std::string>();
	}
	else if (key == "background:")
	{
		background = Input.getToken<std::string>();
	}
	else
	{
		// a key of a later version: skip its value
		Input.getToken<std::string>();
	}

	return true;
}

frame_schedule::frame_schedule(std::uint64_t const Start, int const Interval, int const Phase)
    : m_interval{std::max(Interval, 1)}, m_next{Start + static_cast<std::uint64_t>(std::max(Phase, 0))}
{
}

int frame_schedule::effective_interval(int const Configured, int const Requested)
{
	return std::max({Configured, Requested, 1});
}

int frame_schedule::phase(std::string_view const Key, int const Interval)
{
	if (Interval <= 1)
	{
		return 0;
	}
	// FNV-1a: the same offset for the same name on every compiler
	std::uint32_t hash{2166136261u};
	for (auto const character : Key)
	{
		hash ^= static_cast<std::uint8_t>(character);
		hash *= 16777619u;
	}
	return static_cast<int>(hash % static_cast<std::uint32_t>(Interval));
}

bool frame_schedule::poll(std::uint64_t const Frame)
{
	if (Frame < m_next)
	{
		return false;
	}
	m_next = Frame + static_cast<std::uint64_t>(m_interval);
	return true;
}

bool is_safe_relative_path(std::string_view const Path)
{
	if (Path.empty() || Path.front() == '/' || Path.front() == '\\' || Path.find(':') != std::string_view::npos)
	{
		return false;
	}
	std::size_t start{0};
	while (start <= Path.size())
	{
		auto const end{std::min(Path.find_first_of("/\\", start), Path.size())};
		if (Path.substr(start, end - start) == "..")
		{
			return false;
		}
		start = end + 1;
	}
	return true;
}

std::optional<std::string> screen_script_path(std::string const &Folder, std::string const &Script)
{
	// as substr_path() tells a folder
	auto path{Script.find('/') == std::string::npos ? Folder + Script : Script};
	if (false == is_safe_relative_path(path))
	{
		return std::nullopt;
	}
	return path;
}

namespace
{

// the end of the set that starts at Start ('['), past its ']'; Pattern.size() for a set that doesn't end. as Lua's
// classEnd: a ']' right after '[' or '[^' is in the set, and '%' escapes the character after it
std::size_t set_end(std::string_view const Pattern, std::size_t Start)
{
	auto idx{Start + 1};
	if (idx < Pattern.size() && Pattern[idx] == '^')
	{
		++idx;
	}
	do
	{
		if (idx >= Pattern.size())
		{
			return Pattern.size();
		}
		if (Pattern[idx++] == '%')
		{
			++idx;
		}
	} while (idx >= Pattern.size() || Pattern[idx] != ']');
	return idx + 1;
}

} // namespace

double lua_pattern_cost(std::string_view const Pattern, std::size_t const Subject, bool const Plain, bool const Anchorable)
{
	auto const length{static_cast<double>(std::max<std::size_t>(Pattern.size(), 1))};
	auto const positions{static_cast<double>(Subject) + 1.0};
	// LuaJIT searches plainly for a pattern without the special characters as well
	if (Plain || Pattern.find_first_of("^$*+?.([%-") == std::string_view::npos)
	{
		return positions * length;
	}
	auto const anchored{Anchorable && Pattern.front() == '^'};
	int repeats{0};
	int optional{0};
	// a repetition that ends the pattern doesn't multiply: past it the match is over
	auto last_repeat{false};
	std::size_t idx{anchored ? 1U : 0U};
	while (idx < Pattern.size())
	{
		auto const character{Pattern[idx]};
		if (character == '(' || character == ')')
		{
			++idx;
			// a ')' after the last repetition keeps it last
			continue;
		}
		last_repeat = false;
		std::size_t end{idx + 1};
		if (character == '%' && idx + 1 < Pattern.size())
		{
			auto const next{Pattern[idx + 1]};
			if (next == 'b')
			{
				// %bxy goes along the subject to the balancing character
				++repeats;
				last_repeat = true;
				idx += 4;
				continue;
			}
			if (next == 'f')
			{
				idx = idx + 2 < Pattern.size() && Pattern[idx + 2] == '[' ? set_end(Pattern, idx + 2) : idx + 2;
				continue;
			}
			if (next >= '0' && next <= '9')
			{
				// a back reference compares the capture, as long as the subject
				++repeats;
				last_repeat = true;
				idx += 2;
				continue;
			}
			end = idx + 2;
		}
		else if (character == '[')
		{
			end = set_end(Pattern, idx);
		}
		if (end < Pattern.size() && (Pattern[end] == '*' || Pattern[end] == '+' || Pattern[end] == '-'))
		{
			++repeats;
			last_repeat = true;
			idx = end + 1;
		}
		else if (end < Pattern.size() && Pattern[end] == '?')
		{
			++optional;
			idx = end + 1;
		}
		else
		{
			idx = end;
		}
	}
	if (last_repeat)
	{
		--repeats;
	}
	auto const starts{anchored ? 1.0 : positions};
	return starts * std::pow(positions, repeats) * std::pow(2.0, optional) * length;
}

} // namespace script_screens
