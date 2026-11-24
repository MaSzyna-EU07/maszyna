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

// the longest frame since the last perf, and the frames longer than 33 ms
inline double &frame_longest()
{
	static double value{0.0};
	return value;
}

inline std::size_t &frames_slow()
{
	static std::size_t count{0};
	return count;
}

// the expectations of the script checked, and those of them which failed, with the commands no group knew. At quit the run ends
// with failed_exit_code when any failed, for the runner of the scripts (tests/editor/run.py)
inline std::size_t &expectations()
{
	static std::size_t count{0};
	return count;
}

inline std::size_t &failures()
{
	static std::size_t count{0};
	return count;
}

// a command whose subject isn't there, a track name mistyped, fails the run as an expectation does
inline void fail()
{
	++expectations();
	++failures();
}

// apart from the 1 of a start which failed or of an error which ended the editor
inline constexpr int failed_exit_code{2};

inline std::string one_line(std::string Text)
{
	std::replace(Text.begin(), Text.end(), '\n', ';');
	return Text;
}

} // namespace editor_selftest
