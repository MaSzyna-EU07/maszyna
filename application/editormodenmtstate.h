/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "application/editortool.h"
#include "scene/heightmapterrain.h"
#include "world/Segment.h"

#include <array>
#include <cstddef>
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

class TTrack;

// the offset of the terrain model, worked out from the places of the tracks
struct nmt_fit_state
{
	double offset{0.0}; // added to the heights of the model, so they meet the tracks of the scenery
	bool fitted{false};
	bool fitting{false}; // waiting for the model under the places of the tracks the offset is worked out from
	std::vector<nmt_place> fit;
	double spread{0.0}; // how much the tracks differ from the model about the offset, m
	std::size_t samples{0};
};

// how the terrain is made from the model along the tracks
struct nmt_settings_state
{
	double band{300.0}; // m from the tracks the chunks made along them reach
	bool shape{true}; // the formation and the slopes of the tracks as modifiers of the terrain
	bool grow{true}; // while a track is laid, the terrain is made around the cursor where there's none
	bool follow{true}; // a track with its formation edited takes it along, the formation led anew
	double slope{1.5};
	double reach{60.0};
	bool common{true}; // the tracks nearly parallel under 5.1 m apart lie on one grade line
	bool unfitted{false}; // the tracks put onto the terrain are only the ones whose grade line isn't fitted yet
	bool redo_hand{false}; // all anew takes the grade lines edited by hand too
};

// the chunks of the terrain made from the model, those waiting for it and those given up
struct nmt_growing_state
{
	bool laying{false}; // the tracks committed are being laid: they leave the terrain as it is
	bool grown{false}; // the chunks waiting were all asked for by growing: the formations aren't made anew after them
	bool reshape{false}; // the formations of the tracks laid anew once the chunks waiting are made, as the tracks are put onto the terrain
	std::unordered_set<TTrack const *> stayed; // the tracks this putting onto the terrain leaves as they are
	double grow_time{-10.0};
	std::vector<heightmap::chunk_key> pending; // chunks to make once their model is in
	// of the chunks waiting, those there already with flat ground, made without the model: the model fills their ground
	std::set<heightmap::chunk_key> refill;
	std::set<heightmap::chunk_key> given_up; // chunks the growing gave up, next to terrain not from the model: not asked for again
	std::size_t made{0};
	std::size_t empty{0}; // chunks not made: the model has no data there
	std::size_t foreign{0}; // chunks not made ahead: the terrain beside isn't of the model
	std::optional<double> ahead; // how much the terrain made ahead lies over the model
	bool camera_check{false}; // chunks were made: the 3d camera may be under them now
	std::vector<std::string> buried; // the tracks just laid, when a part of them is hidden in the ground
	double buried_length{0.0}; // m of them hidden
};

// the formations of the tracks changed, which follow them
struct nmt_formation_state
{
	std::set<TTrack *> changed; // tracks whose formations are to follow them
	std::set<std::string, std::less<>> gone; // names of the tracks removed, whose formations go with them
	double changed_time{0.0};
	bool restoring{false}; // the history is taken back or forward: the terrain comes back with it
	std::size_t shaped{0};
};

// the grade lines the tracks lying side by side share
struct nmt_grade_lines_state
{
	static int constexpr grade_line_own{-1};
	static int constexpr grade_line_steep{-2};
	static int constexpr grade_line_off{-3}; // taken off a grade line by hand, stays on its own when the lines are worked out again
	static int constexpr grade_line_off_number{-10}; // this and below: taken off the grade line of this number less this
	std::map<std::string, int, std::less<>> grade_lines; // of each track the number of the grade line it shares, or one of those above
	std::set<std::string, std::less<>> grade_lines_hand; // the tracks put on their grade line by hand: they stay on it when the lines are worked out again
	bool grade_lines_loaded{false};
	bool grade_lines_dirty{false}; // the edits of the tracks carried their lines on: stored once, with the next frame
	std::unordered_map<TTrack const *, std::string> grade_line_keys; // of the tracks changed, the key each had last, to carry its line on to the next
	std::vector<std::pair<std::string, segment_data>> grade_line_replaced; // the first paths of the tracks of a line changed since the last frame: a new track on one takes its line
	std::unordered_set<TTrack const *> on_hump; // the tracks of the humps fitted to the model: no grade line shared levels them
	std::size_t grade_lines_changes{0};
	std::string explain; // a track whose run the next sharing of the grade lines tells of in the log, for a selftest
	struct surface_place
	{
		TTrack const *track;
		int path;
		double share;
		int surface;
	};
	std::vector<surface_place> surface_places; // what lies on the surface of each group on common timbers
};

// the tracks of a scenery put onto the terrain model, a line at a time
struct nmt_automatic_state
{
	// the tracks of a scenery built at any height put onto the terrain model: the model offset to meet the scenery, then the grade
	// line of each line of tracks fitted to the model, the longest first, then the terrain made along them
	enum class automatic
	{
		idle,
		fetching, // the model under the tracks
		fitting_lines // the grade lines, a few each frame
	};
	automatic stage{automatic::idle};
	std::vector<nmt_place> probes; // places of the tracks whose model is waited for
	std::size_t probes_kept{0}; // the last of the probes, on tracks fitted already: they tell the offset the model had
	std::size_t tracks{0}; // rail tracks when the grade lines began, and the paths the grade lines split off since
	std::size_t split{0};
	std::vector<TTrack *> left_to_fit; // tracks whose grade line is still to be fitted
	std::vector<TTrack *> done; // tracks of the lines fitted already: the lines fitted later end on them and don't change them
	// the rail tops of the lines fitted already, in squares of the plan: the tracks of a station lie in one plane, a line laid
	// near them takes their height for its ground too, the more closely the nearer it is, and a line crossing one of them
	// on its level its height on the crossing
	struct laid_point
	{
		glm::dvec3 position{0.0};
		glm::dvec2 direction{0.0, 1.0};
		double grade{0.0};
		double was{0.0}; // the height before the line was fitted
		double rails{0.0}; // the height of the rails of the track
		TTrack const *track{nullptr};
	};
	std::map<std::pair<int, int>, std::vector<laid_point>> laid;
	// the points of all the paths as they were laid, for the crossings
	std::map<std::pair<int, int>, std::vector<laid_point>> paths;
	// ends which meet in the plan unlinked, laid apart in height, that the joining of the ends couldn't join as neither of
	// them can be changed alone, as of two switches: each with the other one, which a line ending there takes over
	std::map<std::tuple<TTrack const *, int, bool>, std::tuple<TTrack *, int, bool>> loose;
	// the points of the lines over a bridge or a viaduct, the model falling away under them: no formation is laid there
	std::map<std::pair<int, int>, std::vector<glm::dvec3>> bridged;
	// a turntable lies level in its pit, which the model has, at the height of the tracks running up to it: it isn't put onto
	// the model, and a line ending on its rim ends there level at its height, turning to the model away from it
	struct turntable
	{
		TTrack *table{nullptr};
		glm::dvec3 centre{0.0}; // at the height of its rails
		double rim{0.0}; // m from the centre
		std::vector<std::tuple<TTrack *, int, bool>> ends; // of the paths running up to it: track, path, at its end
	};
	std::vector<turntable> turntables;
	std::size_t levelled{0}; // turntables moved to the height of their tracks
	std::size_t lines{0}; // grade lines fitted and applied
	std::size_t refused{0}; // grade lines the checks didn't let apply
	std::size_t departures{0}; // grade lines parting from the terrain model by much
};

// what the last putting of the tracks onto the terrain found, and how far the work got
struct nmt_findings_state
{
	// what the last putting of the tracks onto the terrain found, listed in the panel to be looked at: a line the checks
	// refused, a line parting from the model by much, a path which didn't take its line
	struct finding
	{
		enum class kind
		{
			refused,
			off,
			parted
		};
		kind what{kind::refused};
		TTrack *from{nullptr}; // the ends of the line, for its profile
		TTrack *to{nullptr};
		double length{0.0};
		glm::dvec3 where{0.0}; // the place to look at
		std::string text;
		double size{0.0}; // m the line or the path is off by: of one kind the worst told first
	};
	std::vector<finding> findings;
	bool result{false}; // the putting onto the terrain ended: what it did is told in the panel until the next one
	bool run{false}; // the tracks are being put onto the terrain, the chunks after the grade lines too: the steps told
	std::size_t fetch_most{0}; // the most squares of the model waited for since the loading began, for its progress
};

// the terrain made under the tracks as they are, the model drawn to them
struct nmt_under_state
{
	// the terrain made under the tracks as they are: the model drawn to them by the difference found at the places along them,
	// fading out away from them into the offset of the whole scenery
	bool under{false};
	std::vector<glm::dvec3> anchors; // x, z of a place of the tracks, the formation there less the model
	std::map<std::pair<int, int>, std::vector<std::size_t>> anchor_cells; // the anchors in squares of the plan
};

// the terrain from the digital terrain model of GUGiK (editormodenmt.cpp), its parts by what they hold
struct nmt_state : nmt_fit_state, nmt_settings_state, nmt_growing_state, nmt_formation_state, nmt_grade_lines_state, nmt_automatic_state, nmt_findings_state, nmt_under_state
{
	std::string status;
	std::string error;
	bool offered{false}; // the terrain was offered to the scenery begun without it
	bool prompt{false}; // the offer is shown
	bool moving{false}; // the place being picked again
	std::array<char, 96> place{}; // typed: degrees or PUWG 1992
	int shown{-1}; // the frame the terrain panel showed how far the work got
};

// the download area being drawn on the map: its points in the world (x, z), the one dragged
struct area_drawing
{
	bool active{false};
	std::vector<glm::dvec2> points;
	int dragged{-1};
};

// a chunk of the heightmap terrain made from the terrain model
enum class nmt_chunk
{
	waiting, // for its squares of the model
	made,
	empty // the model has no data there
};
