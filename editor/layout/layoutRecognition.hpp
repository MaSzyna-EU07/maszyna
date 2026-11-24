/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "editor/layout/layoutLine.hpp"
#include "world/Segment.h"

#include <cstddef>
#include <string>
#include <vector>
#include <glm/glm.hpp>

// The recognition of the old tracks (editor-tracks-domain.md §10, "Automatyczne przejęcie odcinka"): a chain of the paths of an old
// scenery, Bezier curves drawn by hand or by older tools, made a line of the domain. The curvature sampled along the arc length, the
// chain cut where it changes (the curves between the straights, the transitions and the arcs inside them, the kinks at the joints), the
// clothoid - arc - clothoid of each curve fitted by least squares to the points of the paths, the cant fitted to their rolls and the grade
// line to their heights; then the deviation of the line from the paths measured densely, in the plan, in the height and in the cant.
// A line the domain can't lay as the paths run (a compound curve, a cant with no transition to ramp it along) is fitted as near as it
// goes and told; the norms aren't asked here.
namespace layout
{

// a path of the chain, run from its start to its end or the other way
struct chain_path
{
	segment_data path;
	bool forward{true};
};

// a point of the chain, every few metres along it and at each joint
struct chain_sample
{
	double s{0.0}; // m along the chain, in the plan
	glm::dvec3 point{0.0};
	glm::dvec2 heading{0.0, 1.0}; // plan, unit, as the chain runs
	double curvature{0.0}; // 1/m, above 0 turning left
	double cant{0.0}; // mm, above 0 leaning left as the chain runs
	std::size_t part{0}; // the path of the chain it lies on
	double joint_turn{0.0}; // rad, at the start of a path: how the heading turns from the end of the path before it
};

// the chain sampled no farther apart than Step along each path, its ends and the joints included
std::vector<chain_sample> sample_chain(std::vector<chain_path> const &Chain, double const Step);
// m in the plan, of each path of the chain
std::vector<double> part_lengths(std::vector<chain_path> const &Chain);

// how near the line has to run to the old tracks for them to be taken over with no question asked
struct takeover_tolerance
{
	double plan{0.02}; // m: the simulator itself joins the ends of the tracks within 2 cm
	double height{0.02}; // m
	double cant{10.0}; // mm: 0.4 degree of the roll, under the half degree many old files round the rolls to
};

// how far the line lies from the old tracks, measured at points of them every half metre: the farthest and the root mean square of the
// distance in the plan, of the height and of the cant, where the farthest is
struct measured_deviation
{
	double plan{0.0};
	double plan_rms{0.0};
	double height{0.0};
	double height_rms{0.0};
	double cant{0.0};
	double cant_rms{0.0};
	glm::dvec3 plan_at{0.0};
	glm::dvec3 height_at{0.0};
	glm::dvec3 cant_at{0.0};
	std::size_t samples{0};

	bool within(takeover_tolerance const &Tolerance) const { return plan <= Tolerance.plan && height <= Tolerance.height && cant <= Tolerance.cant; }
	// the worst of the three in the units of the tolerance
	double share(takeover_tolerance const &Tolerance) const;
	// the farthest of each of both
	void add(measured_deviation const &Other);
};

// the deviation of the line from the samples of the chain: each point of the chain set square onto the line
measured_deviation measure(solved_line const &Line, std::vector<chain_sample> const &Samples);

// an end of the chain held by what goes on from there and stays (a switch, an old track which isn't taken over): the line has to run on
// from it in its heading
struct held_end
{
	bool held{false};
	glm::dvec2 heading{0.0, 1.0}; // into the chain at its start, out of it at its end
};

struct recognition_options
{
	double step{1.0}; // m between the points the curves are fitted to
	double measure_step{0.5}; // m between the points the deviation is measured at
	takeover_tolerance tolerance;
	transition_shape transition{transition_shape::clothoid};
	held_end start;
	held_end end;
};

// a curve found where the curvature of the chain changes: from where to where along the chain (its transitions included), how much
// it turns
struct found_curve
{
	double from{0.0};
	double to{0.0};
	double deflection{0.0}; // rad, above 0 turning left
	double radius{0.0}; // m, of the arc, as first read off the curvature
	double transition_in{0.0};
	double transition_out{0.0};
	bool kink{false}; // the chain turns at a joint with no curve
	bool compound{false}; // arcs of more than one radius with no straight between them
};
// the curves of the chain between its straights, read off the curvature sampled along it (each one a curve of a vertex, a kink a curve
// of the shortest length the domain allows)
std::vector<found_curve> find_curves(std::vector<chain_sample> const &Samples);

struct recognized_line
{
	line_shape shape; // the vertices with their radii, transitions and cants, the grade line; no ids given
	glm::dvec3 start{0.0}; // the ends of the chain
	glm::dvec3 end{0.0};
	solved_line solved; // laid between them
	measured_deviation measured;
	std::vector<double> joints; // the chainage of each joint between the paths along the line, the ends included
	std::vector<std::string> findings; // what the domain lays otherwise than the paths run: kinks, compound curves, cants with no ramp
	std::string error; // why there's no line
	bool valid() const { return error.empty() && solved.valid(); }
	bool silent(takeover_tolerance const &Tolerance) const { return valid() && measured.within(Tolerance); }
};

// the line of the chain: the curves found fitted by least squares, the cant and the grade line fitted, the deviation measured
recognized_line recognize(std::vector<chain_path> const &Chain, recognition_options const &Options = {});

// the cant of the roll of a path, mm, above 0 leaning left as the path runs from its start: on the base of the rail heads
double cant_of(double const Roll);

} // namespace layout
