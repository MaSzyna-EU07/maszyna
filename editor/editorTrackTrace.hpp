/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <utility>
#include <vector>

#include <glm/glm.hpp>

class TTrack;

// the track followed the way a train runs it, and the places along it where it comes to a switch: the fouling points the W17
// markers stand at and the signals are measured to
namespace track_trace
{

// m between the axes of the tracks at the fouling point of a switch on the straight (Standardy techniczne PKP PLK, tom II, 6.5)
double constexpr kFoulingSpacing{3.5};

// points along the track from the end of a path on, the way a train goes: through a switch along its main track
std::vector<glm::dvec3> trace(TTrack &Track, int const Path, bool const Forward, double const Limit);
// and Cant: the raise of one rail over the other at each point, m, positive with the cars leaning to the left of the way walked
std::vector<glm::dvec3> trace(TTrack &Track, int const Path, bool const Forward, double const Limit, std::vector<double> &Cant);

// the parameter of the point of a path nearest the point given in the plan
double parameter_of(TTrack const &Track, int const Path, glm::dvec3 const &Point);

// the fouling point of a switch: where its branch is Spacing from its main track, more in the curves under 250 m by the
// widening of the gauge (editorFouling), both followed Limit from its start
struct fouling
{
	glm::dvec3 main{0.0}; // on the axis of the main track
	glm::dvec3 branch{0.0}; // on the axis of the branch
	glm::dvec2 back{0.0, 1.0}; // the plan direction along the branch towards the start of the switch
	double spacing{kFoulingSpacing}; // m between the axes there, with the widenings
};
bool fouling_of(TTrack &Switch, double const Spacing, double const Limit, fouling &Point);

// where a train running along a path from a point of it comes to a switch first: the start of one it runs onto from its start, or
// the fouling point of one it runs onto from the end of a path, which may lie behind the point
struct danger
{
	enum class kind
	{
		none,
		start,
		fouling
	};
	kind what{kind::none};
	TTrack *track{nullptr}; // the switch
	double distance{0.0}; // m along the track from the point to it, below 0 when it lies behind: the point in the fouling zone
	glm::dvec3 point{0.0}; // on the axis of the track the train runs
	double spacing{0.0}; // of a fouling point: m between the axes of the tracks there, the widenings in the curves with it
	// the axis of the track from Behind before the point on, past the place, each point with the path it lies on and the distance
	// from the point to it, below 0 behind
	std::vector<glm::dvec3> line;
	std::vector<double> along;
	std::vector<std::pair<TTrack *, int>> paths;
	// the point of the line Distance from the point, below 0 behind it; Path: the track and the path it lies on
	glm::dvec3 at(double const Distance, std::pair<TTrack *, int> &Path) const;
};
// Forward: the train runs the path from its start to its end. Limit: m it is followed for, Behind: m behind the point
danger danger_ahead(TTrack &Track, int const Path, glm::dvec3 const &Point, bool const Forward, double const Limit, double const Behind = 120.0, double const Spacing = kFoulingSpacing);

} // namespace track_trace
