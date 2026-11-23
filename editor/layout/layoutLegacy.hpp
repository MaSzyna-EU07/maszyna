/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "editor/layout/layoutAnchors.hpp"
#include "editor/layout/layoutCommands.hpp"
#include "editor/layout/layoutRecognition.hpp"
#include "world/Segment.h"

#include <cstddef>
#include <optional>
#include <string>
#include <vector>
#include <glm/glm.hpp>

// The old tracks of a scenery taken over by the domain (editor-tracks-domain.md §10, stage E6): the tracks of the simulator which no line of
// the domain describes, chained the way a train runs them, each chain recognized as a line (layoutRecognition.hpp) and laid by one command
// of the model, the joints of the old tracks kept as hard points of its pieces (line::hard_splits) so their names, events and isolated
// circuits stay where they were. A switch, an old track which isn't taken over, a fork hold the end of a chain: the line runs on from it in
// its heading, the switch staying as it is (a fixed constraint), not cutting the line where nothing else does.
namespace layout
{

// an old track as the text of its entry has it, in the world: its name, its paths, the words of the entry before and after the paths
struct legacy_track
{
	std::string name; // empty for none
	bool is_switch{false};
	std::vector<segment_data> paths; // a switch: the main path, then the diverging one, both from its start
	std::string surface; // the words between the length and the paths: the width, the friction, ... the visibility and the materials
	std::string attributes; // the words after the paths up to endtrack
	std::string file; // where it's written, relative to the scenery folder
	std::size_t line{0};

	// the speed of the track, 0 where its entry has none
	double velocity() const;
};

// a path of a chain of old tracks, run from its start (forward) or from its end
struct legacy_link
{
	std::size_t track{0};
	int path{0};
	bool forward{true};

	bool operator==(legacy_link const &) const = default;
};

// how a chain of old tracks ends: at a free end, held by what goes on from there and stays (a switch, an old track not taken over), or at a
// fork of more tracks than one
enum class legacy_end_kind
{
	free,
	held,
	fork
};

struct legacy_end
{
	legacy_end_kind kind{legacy_end_kind::free};
	glm::dvec2 heading{0.0, 1.0}; // held: into the chain at its start, out of it at its end
	std::optional<std::size_t> by; // the track holding it
};

struct legacy_chain
{
	std::vector<legacy_link> links;
	legacy_end start;
	legacy_end end;
};

// m: the ends of the paths closer than this are joined, as the simulator joins them
inline constexpr double legacy_joint{0.02};

// the chains of the plain tracks of Take (indices of Tracks), each as far as the tracks run on one into the other: a track joined at an end to
// exactly one other plain track of Take goes on into it; a switch, a track not taken, a fork, a free end end it. Each runs from its end the
// smaller in x (then in z) to the other one, whichever track it was found from
std::vector<legacy_chain> walk_chains(std::vector<legacy_track> const &Tracks, std::vector<std::size_t> const &Take);

// the paths of the chain as the recognition takes them
std::vector<chain_path> paths_of(std::vector<legacy_track> const &Tracks, legacy_chain const &Chain);

struct takeover_options
{
	recognition_options recognition;
	norms::line_class category; // of the lines made; the speed the highest of the old tracks, where they have one
};

// a line the takeover makes of a chain of old tracks
struct taken_line
{
	legacy_chain chain;
	recognized_line fit;
	std::vector<hard_split> hard_splits; // the old tracks along it
	norms::line_class category;
	bool silent{false}; // it keeps within the tolerance: taken over with no question
};

struct takeover_plan
{
	std::vector<taken_line> lines;
	takeover_tolerance tolerance;
	std::vector<std::string> notes; // the tracks which can't be taken over, and why
	measured_deviation worst; // the farthest any line of it keeps off its old tracks
	bool silent() const;
	// the command laying the lines
	take_over command() const;
};

// the old tracks of Take chained and recognized as lines; nothing is laid
takeover_plan plan_takeover(std::vector<legacy_track> const &Tracks, std::vector<std::size_t> const &Take, takeover_options const &Options = {});

// what a takeover did: the outcome of the command, the deviation of each line it laid from its old tracks (in the order of the plan), and
// whether it was within the tolerance, which takes it over with no question asked; the anchors of the lineside objects by the lines it
// laid, the doubtful ones to be looked over
struct takeover_result
{
	outcome made;
	std::vector<measured_deviation> deviations;
	measured_deviation worst;
	bool silent{false};
	std::vector<anchor> anchors;
	std::vector<std::size_t> doubtful() const;
};
// the lines of the plan laid by one command of the model, one step to undo; the norms not refusing what the old tracks break (told in
// made.broken). The objects given (those of the scenery) anchored to the lines laid, named by their old tracks, numbered from First
takeover_result take_over_tracks(model &Model, takeover_plan const &Plan, std::vector<lineside_object> const &Objects = {}, std::uint32_t const First = 1);

} // namespace layout
