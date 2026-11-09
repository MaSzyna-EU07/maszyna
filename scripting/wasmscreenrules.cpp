/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "scripting/wasmscreenrules.h"

#include "utilities/parser.h"

#include <algorithm>

namespace wasm_screens
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

} // namespace wasm_screens
