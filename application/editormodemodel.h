/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// The state of the editing through the model of the track geometry (stage E4), shared by the files of it: editormodemodel.cpp, the project
// and the lines; editormodemodelturnout.cpp, the turnouts; editormodemodeledit.cpp, the grade line, the curves and the parallel track.

#include "application/editormode.h"
#include "editor/project/projectSession.hpp"
#include "editor/project/scenarioIndexCache.hpp"

#include <functional>
#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// what the entry of the track has between its length and its path (the width ... the visibility with the materials); empty where it isn't
// a plain track
std::string model_surface_of(TTrack const &Track);

// what waits for the user's answer in a window of its own
struct model_question
{
	enum class kind
	{
		none,
		hand, // the lines changed by hand: written over, or the change by hand kept
		waiting, // the scenarios using the map being looked through, before the tracks named go
		scenarios // the scenarios naming the tracks: taken away anyway, or not
	};
	kind what{kind::none};
	std::set<layout::line_id> lines;
	std::vector<std::string> names; // the tracks to go
	std::vector<project::scenario_use> users;
	std::function<void(bool)> answer; // true: written over, taken away anyway
	bool rename{false}; // the tracks named are to be renamed, not taken away
	bool opened{false};
};

struct editor_model
{
	project::session session;
	bool tried{false}; // the project opened for the scenery, or failed to
	std::string error;
	std::unordered_map<scene::basic_node const *, editor_mode::managed_track> tracks;
	project::index_cache index;
	std::filesystem::path store; // the file the index is kept in between the sessions, none for the self-tests
	model_question question;
	// the line the route design is open on, its vertices as the model has them
	struct plan_link
	{
		layout::line_id line{layout::line_id::none};
		std::vector<layout::vertex_id> vertices;
		alignment::design original;
		double split{-1.0}; // the chainage the split field holds, below 0 the middle of the line
	} plan;
	std::string message; // what the last command came to: why it was refused, or what the norms tell of it
	bool refused{false};
	std::vector<std::string> notices;
	std::vector<std::pair<TTrack const *, std::string>> written; // by the save under way, the texts written
	std::map<std::string, layout::network> marks; // the self-test's copies of the network, for expect modelsame
	std::string shown; // the message as the user reads it, in the language of the editor
	std::vector<std::string> shown_notices; // the notices so
	std::string proposal; // the solver's best proposal for a crossover refused
	std::pair<std::string, std::string> drag; // the switch dragged on a line of the model as it was tried: its command, why it's refused
	double turnout_move{5.0}; // m, the distance the inspector moves a turnout by
};

