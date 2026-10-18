/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <algorithm>
#include <cstddef>
#include <string>

// what the groups of the self-test commands share, each group in a file of its own
namespace editor_selftest
{

// frames gone through the selftest, for the measurements of perf
inline std::size_t &frames()
{
	static std::size_t count{0};
	return count;
}

inline std::string one_line(std::string Text)
{
	std::replace(Text.begin(), Text.end(), '\n', ';');
	return Text;
}

} // namespace editor_selftest
