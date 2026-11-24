/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstddef>
#include <functional>
#include <vector>
#include <glm/glm.hpp>

// The heights of the points clicked while a track is laid, and where each came from. The terrain is made around the cursor as the track
// is laid: the first point clicked in an empty place has no ground under it yet, its height a guess, and the terrain comes a moment
// later. Such a point takes its height again once the ground under it is there, before the line is laid, and the line shown with it;
// Enter waits a while for the terrain still on its way. A point on a track, or laid already, isn't touched.
namespace click_heights
{

enum class source : char
{
	fixed, // a free end of a track, the end of the line laid last: kept as it is
	ground, // the ground under it: the terrain, the model of it, the ground of the scenery
	none, // no ground there yet when it was clicked: the height a guess, taken again once the ground is there
	level, // the height of the point before it, the heights laid level: it goes along with that one
};

// The points of a guessed height over the ground now take its height: Ground of the index of a point, NaN where there's still none
// under it. The level ones after a point changed go along with it; the ground and the fixed ones are kept. Points beyond the sources
// given are kept. The count of the points whose height changed
std::size_t settle(std::vector<glm::dvec3> &Points, std::vector<source> &Sources, std::function<double(std::size_t)> const &Ground);

// any point of a guessed height left
bool guessed(std::vector<source> const &Sources);
std::size_t guessed_count(std::vector<source> const &Sources);

// a point of a guessed height the terrain under which is on its way (Coming of the index): the line waits for it
bool waiting(std::vector<source> const &Sources, std::function<bool(std::size_t)> const &Coming);

// s, how long Enter waits at most for the terrain under the points: then the line is laid at the heights there are
double constexpr wait_longest{10.0};

// the line waits on: the terrain is still coming under a point, and the wait began less than wait_longest ago
bool wait_on(std::vector<source> const &Sources, std::function<bool(std::size_t)> const &Coming, double Waited);

} // namespace click_heights
