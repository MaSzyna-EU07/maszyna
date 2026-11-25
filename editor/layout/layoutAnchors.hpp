/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "editor/layout/layoutNetwork.hpp"
#include "editor/editorInfra.hpp"

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>
#include <glm/glm.hpp>

// The anchors of the lineside objects (editor-tracks-domain.md §2 "Kotwica", §3 "Otoczenie", §10 "Powiązania otoczenia", stage E6): where an
// object stands along a line of the domain, its chainage, how far to the left of the line (below 0 to the right) and how high above it,
// its turn against the heading of the line there; so when the line changes, the object is put where it stands along the new one. The
// anchors of an old scenery are inferred for all the kinds of objects from where they stand and from the events and the memory cells which
// name the tracks, each with a confidence, the doubtful ones to be looked over; a wire (a traction piece) anchored by its four points and
// bound to the poles its ends hang on. The old binding of the editor (//$b) is the same mechanism along the paths of the tracks.
namespace layout
{

// a lineside object of a scenery, as its node has it
struct lineside_object
{
	std::string kind; // model, traction, memcell, eventlauncher
	std::string name; // of the node, empty for none
	std::string key; // the file of a model, the power supply of a traction, the track of a memory cell; - for none
	infra::category group{infra::category::other};
	glm::dvec3 location{0.0}; // in the world
	double yaw{0.0}; // degrees, of a model
	std::vector<glm::dvec3> points; // a traction: the ends of its wires, as it has them (p1 p2 at the contact wire, p3 p4 above them)
	std::vector<std::string> tracks; // the tracks which name it: a memory cell's own, those whose events act on it (made small)
	std::string file; // where it's written, relative to the scenery folder
	std::size_t line{0};
};

// a point along a line: its chainage, metres to the left of the line (below 0 to the right), above the line
struct anchor_point
{
	double chainage{0.0};
	double offset{0.0};
	double height{0.0};

	bool operator==(anchor_point const &) const = default;
};

struct anchor
{
	std::uint32_t id{0};
	// the object, as it was found: its kind, name, key and the location it stood at when it was anchored (to find it in the scene)
	std::string kind;
	std::string name;
	std::string key{"-"};
	infra::category group{infra::category::other};
	glm::dvec3 location{0.0};
	double yaw{0.0};
	// where it stands along the line
	line_id line{line_id::none};
	anchor_point at;
	std::vector<anchor_point> points; // a wire: its four points
	double turn{0.0}; // degrees, its yaw against the heading of the line there
	double confidence{0.0}; // 0 to 1
	std::vector<std::string> reasons; // what the confidence rests on, or what makes it doubtful
	std::vector<std::uint32_t> poles; // a wire: the anchors of the poles its ends hang on

	bool operator==(anchor const &) const = default;
	// confidence below this: to be looked over by the author before the object follows a change of its line
	static constexpr double doubtful_below{0.6};
	bool doubtful() const { return confidence < doubtful_below; }
};

// the anchors of the objects to the lines of the network, numbered from First: each to the line it stands by, as near as it is and as its
// events and memory cells name its tracks (Tracks: the lines the old tracks of the names, made small, became); the objects standing by no
// line left out. In the order of the objects
std::vector<anchor> infer_anchors(network const &Network, std::vector<lineside_object> const &Objects, std::map<std::string, line_id> const &Tracks, std::uint32_t const First = 1);

// where the anchored object stands along its line now: its location, yaw and points; none where its line is gone
struct placement
{
	glm::dvec3 location{0.0};
	double yaw{0.0};
	std::vector<glm::dvec3> points;
};
std::optional<placement> place(network const &Network, anchor const &Anchor);

// the anchors following a change of the network from Before to After: an anchor whose line is there still keeps its chainage (on the
// length the line has now); one whose line is gone (split, joined, taken by a turnout) is anchored again where it stood before, to the line
// there now. The anchors whose objects move, with where to, in the order of the anchors; the anchors updated
struct moved
{
	std::size_t anchor{0};
	placement to;
};
std::vector<moved> follow(network const &Before, network const &After, std::vector<anchor> &Anchors);

} // namespace layout
