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
// each piece being a road node with lanes and geometry of its own
class editor_road
{
  public:
	// an end of a road piece nothing is attached to
	struct loose_end
	{
		road_node *road{nullptr};
		bool atend{false};
		glm::dvec3 position{0.0};
		glm::dvec3 outwards{0.0}; // direction the road would continue in
	};
	// true if the roads can be built and changed at all. optionally explains why they can't
	static bool available(std::string *Reason = nullptr);
	// true if specified road piece can be changed or removed. optionally explains why it can't
	static bool can_edit(road_node const &Road, std::string *Reason = nullptr);
	// makes road pieces out of provided definitions, with their lanes joined to the neighbours and geometry in the scene
	static std::vector<road_node *> create(std::vector<road_node::state> const &States);
	// replaces definitions of the pieces and makes their lanes and geometry anew
	static void apply(std::vector<std::pair<road_node *, road_node::state>> const &Changes);
	// takes the pieces out of the scenery. they're kept around, so they can be brought back
	static void remove(std::vector<road_node *> const &Roads);
	static void revive(std::vector<road_node *> const &Roads);
	// cuts the piece in two at specified point of its axis. returns: the new piece, holding the part past the cut
	static road_node *split(road_node &Road, double const T);
	// the pieces which make a single road together with specified one, in no particular order
	static std::vector<road_node *> chain(road_node &Road);
	// the piece under specified point, or within specified distance from it
	static road_node *nearest(glm::dvec3 const &Point, double const Margin);
	// value of the curve parameter for the point of the axis nearest to specified point
	static double nearest_parameter(road_node::state const &State, glm::dvec3 const &Point);
	// end of a piece with nothing attached, within specified distance from specified point
	static loose_end find_end(glm::dvec3 const &Point, double const Radius);
	// lays out axes of the pieces for a road leading from a point to another. the directions are optional requirements
	// for the road at its ends; without them it starts straight at the target, with them it bends to meet them.
	// returns: the axes in order, or nothing if there's no reasonable road to make, with the reason in Error
	static std::vector<segment_data> plan(glm::dvec3 const &Start, glm::dvec2 const *Direction, glm::dvec3 const &Target, glm::dvec2 const *Enddirection, bool const Straight, std::string &Error);
	// names of materials and images present in the texture folder, to pick from
	static std::vector<std::string> const &materials();

  private:
	// takes down what depends on shape of the listed pieces, lets the caller change them, then builds it all again
	template <class Change_>
	static void refresh(std::vector<road_node *> const &Roads, Change_ Change);
	static std::vector<road_node *> neighbours(road_node const &Road);
};
