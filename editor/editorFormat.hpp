/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstdio>
#include <string>

// printf-style formatting into a string, for the messages and the data lines of the editor
template <typename... Args>
std::string format(char const *Format, Args... Arguments)
{
	auto const length{std::snprintf(nullptr, 0, Format, Arguments...)};
	if (length <= 0)
		return {};
	std::string text(static_cast<std::size_t>(length) + 1, '\0');
	std::snprintf(text.data(), text.size(), Format, Arguments...);
	text.resize(static_cast<std::size_t>(length));
	return text;
}
