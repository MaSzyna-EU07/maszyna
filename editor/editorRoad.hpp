/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <string>
#include <utility>
#include <vector>
#include <glm/glm.hpp>

#include "world/Road.h"

// editor operations on the roads. a road in the scenery is a string of pieces joined end to end,
// each piece being a road node with lanes and geometry of its own; roads meet at junctions
class editor_road
{
  public:
	// an end of a road piece, or an arm of a junction, nothing is attached to
	struct loose_end
	{
		road_node *road{nullptr};
		junction_node *junction{nullptr};
		bool atend{false}; // of a road: the end of its axis rather than the start
		glm::dvec3 position{0.0};
		glm::dvec3 outwards{0.0}; // direction a road would continue in
		// of a junction arm: lanes of a road leading away from it
		int forward{0};
		int backward{0};
		float width{0.f};

		bool valid() const
		{
			return road != nullptr || junction != nullptr;
		}
	};
	// an end of a road piece
	struct piece_end
	{
		road_node *road{nullptr};
		bool atend{false}; // the end of its axis rather than the start
	};
	// what an operation changed, in the form needed to take it back
	struct record
	{
		std::vector<std::pair<road_node *, road_node::state>> roads; // pieces with what they were like before
		std::vector<road_node *> roads_created;
		std::vector<road_node *> roads_removed;
		std::vector<std::pair<junction_node *, junction_node::state>> junctions; // junctions with what they were like before
		std::vector<junction_node *> junctions_created;
		std::vector<junction_node *> junctions_removed;

		bool empty() const
		{
			return roads.empty() && roads_created.empty() && roads_removed.empty() && junctions.empty() && junctions_created.empty() && junctions_removed.empty();
		}
	};
	// a road led out of the side of another road, or out of a junction between its arms: the junction this takes,
	// and what has to be done with the roads to make room for it
	struct branch
	{
		junction_node *junction{nullptr}; // the junction which gets another arm, nullptr if a new one is made
		junction_node::state layout; // the junction as it's going to be
		std::size_t arm{0}; // the arm of it made for the road
		std::vector<std::pair<road_node *, road_node::state>> changes; // pieces cut short, with their new definitions
		std::vector<road_node::state> added; // part of a piece left past the junction, to be made a piece of its own
		std::vector<road_node *> removed; // pieces which lie within the junction
	};
	// true if the roads can be built and changed at all. optionally explains why they can't
	static bool available(std::string *Reason = nullptr);
	// true if specified road piece or junction can be changed or removed. optionally explains why it can't
	static bool can_edit(road_node const &Road, std::string *Reason = nullptr);
	static bool can_edit(junction_node const &Junction, std::string *Reason = nullptr);
	// true if a road or a junction can be attached to specified loose end. optionally explains why it can't
	static bool can_join(loose_end const &End, std::string *Reason = nullptr);
	// makes road pieces out of provided definitions, with their lanes joined to the neighbours and geometry in the scene
	static std::vector<road_node *> create(std::vector<road_node::state> const &States);
	// replaces definitions of the pieces and makes their lanes and geometry anew
	static void apply(std::vector<std::pair<road_node *, road_node::state>> const &Changes);
	// replaces definitions of the junctions, and makes their geometry and the ways through them anew
	static void apply(std::vector<std::pair<junction_node *, junction_node::state>> const &Changes);
	// takes the pieces out of the scenery. they're kept around, so they can be brought back
	static void remove(std::vector<road_node *> const &Roads);
	static void revive(std::vector<road_node *> const &Roads);
	// the same for the junctions
	static std::vector<junction_node *> create(std::vector<junction_node::state> const &States);
	static void remove(std::vector<junction_node *> const &Junctions);
	static void revive(std::vector<junction_node *> const &Junctions);
	// cuts the piece in two at specified point of its axis. returns: the new piece, holding the part past the cut
	static road_node *split(road_node &Road, double const T);
	// the pieces which make a single road together with specified one, in no particular order
	static std::vector<road_node *> chain(road_node &Road);
	// the piece under specified point, or within specified distance from it
	static road_node *nearest(glm::dvec3 const &Point, double const Margin);
	// the junction specified point lies on
	static junction_node *nearest_junction(glm::dvec3 const &Point);
	// the point where pieces meet or end which is nearest to specified one, within specified distance from it. returns: false if there's none
	static bool nearest_joint(glm::dvec3 const &Point, double const Radius, glm::dvec3 &Joint);
	// ends of the pieces which meet at specified point
	static std::vector<piece_end> ends_at(glm::dvec3 const &Point);
	// junctions with an arm at specified point, each with the number of that arm
	static std::vector<std::pair<junction_node *, std::size_t>> arms_at(glm::dvec3 const &Point);
	// brings what's next to the ends of a piece in line with the piece, after its lanes or its width were changed:
	// a junction gets its arm made for the lanes the piece has now, a neighbouring piece with the same lanes is made as wide
	// as the piece where they meet, and one with other lanes gets a transition cut out of its end - a junction of two roads,
	// which leads the lanes of one to the lanes of the other. what gets done is added to Record, what couldn't be done to Notes
	static void settle(road_node &Road, record &Record, std::string &Notes);
	// width shared by all lanes of the road at specified value of the curve parameter of its axis. returns: false if the lanes differ in width
	static bool lane_width(road_node::state const &State, double const T, float &Width);
	// lays out a junction for a road leaving the side of specified piece at specified value of the curve parameter of its axis.
	// Direction: the way the road leaves; Incoming, Outgoing: lanes of that road leading into the junction and out of it.
	// nothing is changed until the result is carried out. returns: false if it can't be done, with the reason in Error
	static bool branch_from(road_node &Road, double const T, glm::dvec2 const &Direction, int const Incoming, int const Outgoing, float const Width, branch &Result, std::string &Error);
	// the same for a road leaving a junction between its arms, which gives the junction another arm
	static bool branch_from(junction_node &Junction, glm::dvec2 const &Direction, int const Incoming, int const Outgoing, float const Width, branch &Result, std::string &Error);
	// makes the junction laid out for a branch, or gives the existing one its new arm, and cuts the roads to fit. what it does is added to Record.
	// returns: the junction
	static junction_node *carry_out(branch const &Branch, record &Record);
	// value of the curve parameter for the point of the axis nearest to specified point
	static double nearest_parameter(road_node::state const &State, glm::dvec3 const &Point);
	// end of a road with nothing attached, within specified distance from specified point; optionally arms of junctions too
	static loose_end find_end(glm::dvec3 const &Point, double const Radius, bool const Junctions = true);
	// lays out axes of the pieces for a road leading from a point to another. the directions are optional requirements
	// for the road at its ends; without them it starts straight at the target, with them it bends to meet them.
	// returns: the axes in order, or nothing if there's no reasonable road to make, with the reason in Error
	static std::vector<segment_data> plan(glm::dvec3 const &Start, glm::dvec2 const *Direction, glm::dvec3 const &Target, glm::dvec2 const *Enddirection, bool const Straight, std::string &Error);
	// cuts the pieces so none is longer than specified
	static std::vector<segment_data> refine(std::vector<segment_data> const &Pieces, double const Length);
	// gives a string of pieces a smooth profile running through provided heights.
	// Heights: one for the start of each piece and one for the end of the last. these of the ends are kept, the others are evened out:
	// the dips are filled while the tops stay, so the ground the heights were taken from doesn't come through the road.
	// Startgrade, Endgrade: slopes to keep at the ends, to carry on what the road joins; nullptr for none
	static void profile(std::vector<segment_data> &Pieces, std::vector<double> Heights, double const *Startgrade, double const *Endgrade);
	// length of the path seen from above
	static double planar_length(segment_data const &Path);
	// slope of the path at its end, in the direction it's laid out
	static double end_grade(segment_data const &Path);
	// names of materials and images present in the texture folder, to pick from
	static std::vector<std::string> const &materials();

  private:
	// takes down what depends on the listed pieces and junctions, lets the caller change them, then builds it all again
	template <class Change_>
	static void refresh(std::vector<road_node *> const &Roads, std::vector<junction_node *> const &Junctions, Change_ Change);
	static std::vector<road_node *> neighbours(road_node const &Road);
};
