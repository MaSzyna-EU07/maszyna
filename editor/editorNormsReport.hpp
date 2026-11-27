/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "editor/norms/norms.hpp"

#include <glm/glm.hpp>
#include <string>
#include <utility>
#include <vector>

class TTrack;

// the norms checked on the tracks laid, read only: a line of paths sampled into what the norms describe a line from, the findings set
// back on the paths, their messages and the report written (editor-tracks-domain.md §6, stage E1). Nothing of the tracks is changed
namespace norms_report
{

// a path of a track the line runs along, from its start or from its end
struct span
{
	TTrack *track{nullptr};
	int path{0};
	bool forward{true};
};

struct options
{
	norms::profile_id profile{norms::profile_id::plk};
	norms::line_class line;
	// the speed of each path where it has one, the one of the line class elsewhere; otherwise the one of the line class everywhere
	bool track_speeds{true};
	double step{1.0}; // m between the samples
};

struct sampled_line
{
	std::vector<norms::sample> samples;
	std::vector<std::pair<double, double>> spans; // the chainages each span runs over
};
// the samples along the line: the plan, the curvature signed by the turn, the cant on the base of 1500 mm signed with the side it leans
// to as the line runs, the grade, the speed of the path, what the path is part of (a switch, its diverging track, a crossing); two at
// each joint, the end of the one path and the start of the next
sampled_line sample(std::vector<span> const &Line, double const Step, bool const TrackSpeeds);

struct entry
{
	norms::finding finding;
	norms::element_kind kind{norms::element_kind::straight}; // of the element it's on
	std::string line;
	TTrack *track{nullptr};
	int path{0};
	glm::dvec3 position{0.0};
};

struct checked_line
{
	norms::line described;
	std::vector<entry> entries;
};
// the line checked against the profiles of the areas (the profile of the options where the map has none of its own, Areas being null);
// Own: the index of the span whose findings alone are kept, as for the diverging track of a switch checked along the line it lies in;
// below 0 all of them
checked_line check(std::vector<span> const &Line, options const &Options, norms::area_map const *Areas = nullptr, int const Own = -1);

char const *traffic_name(norms::traffic const Kind);
char const *tier_name(norms::tier const Tier);
char const *parameter_name(norms::parameter const What, norms::element_kind const Kind);
std::string value_text(norms::parameter const What, double const Value);
// "cant ramp 2.7 mm/m, PLK allows at most 2.5 mm/m (tab. 9.12)"
std::string message(norms::finding const &Finding, norms::element_kind const Kind);

// the report written: what it was checked against, then each line, its elements with Elements, and its findings
std::string header(options const &Options);
std::string text(checked_line const &Line, bool const Elements);

} // namespace norms_report
