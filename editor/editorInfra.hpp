/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <array>
#include <string>
#include <map>
#include <optional>
#include <vector>
#include <glm/glm.hpp>

#include "world/Segment.h"
#include "scene/scenenode.h"

class TTrack;

// objects which belong to the railway infrastructure (signals, signs, catenary, poles, the logic placed along the
// line) bound to the paths they stand by, so they follow the changes of the track geometry
namespace infra
{

enum class kind
{
	model,
	include,
	traction,
	memcell,
	launcher
};
enum class category
{
	signal,
	sign,
	pole,
	catenary,
	logic,
	other,
	// after the others, so the numbers kept in the scenery files keep their meaning
	equipment, // of the track itself, on its axis: the magnets of the train protection, the plates of a level crossing, the
	           // ballast and the lanterns of the switches
	environment // greenery, people, small architecture: by the track, but not a part of the railway, not bound by default
};
char const *name(kind Kind);
char const *name(category Category);
// the kind of an object by the category in the description of its template, if it names one
std::optional<category> category_named(std::string const &Category);
std::vector<category> const &categories();

// point of a path, measured in the plan from the start of the path
struct station
{
	TTrack *track{nullptr};
	int path{0};
	double s{0.0};
};
struct frame
{
	glm::dvec3 point{0.0};
	glm::dvec2 forward{0.0, 1.0}; // in the plan, x and z
};

double plan_length(segment_data const &Path);
// points of the path, no farther apart in the plan than about specified step; the ends included
std::vector<glm::dvec3> outline(segment_data const &Path, double Step);
frame frame_at(segment_data const &Path, double S);
frame frame_at(station const &Station);
// nearest point of specified path in the plan. returns: distance in the plan; Interior tells the nearest point
// isn't one of the ends of the path
double project(segment_data const &Path, glm::dvec3 const &Point, double &S, bool &Interior);
// nearest point among the paths of the track
double project(TTrack &Track, glm::dvec3 const &Point, station &Station, bool &Interior);
// angle around the vertical axis which turns a model to face specified direction, in degrees
double heading(glm::dvec2 const &Forward);

// location of a point relative to the rail axis: the station, a sideways offset and a height above the rail top
struct anchor
{
	station at;
	double offset{0.0}; // positive to the right of the direction of the path
	double height{0.0};
	glm::dvec3 foot{0.0}; // point of the axis the location was measured from
	// ends and length of the path when the station was measured, to tell whether the path changed since
	glm::dvec3 start{0.0}, end{0.0};
	double length{0.0};
};
anchor make_anchor(station const &At, glm::dvec3 const &Point);
// anchor at the place of the path nearest to the point
anchor make_anchor(TTrack *Track, int const Path, glm::dvec3 const &Point);
// location for the current geometry of the path
glm::dvec3 place(anchor &Anchor);

// rules telling infrastructure apart by the names of the files and nodes
struct rule
{
	category group{category::other};
	std::string keywords; // separated by spaces, matched as parts of the lower case name
};
std::vector<rule> default_rules();
// first rule in the list which matches. returns: category::other if none does
category classify(std::string const &Name, std::vector<rule> const &Rules);

struct binding
{
	kind type{kind::model};
	category group{category::other};
	scene::basic_node *node{nullptr}; // the object, unless it's an include
	scene::instance_handle include{0};
	std::vector<anchor> anchors; // one per point which defines the object: four for a traction piece
	double yaw{0.0}; // rotation of the object relative to the direction of the path
	bool turns{false}; // the object has a rotation which follows the path
	std::string label;
	bool lost{false}; // none of the paths around could take the object over
	// bound by the editor itself when its track was taken for a change, as a signal its path reads: not kept in the scenery
	bool adopted{false};
	glm::dvec3 measured{0.0}; // where an adopted object stood when its track was taken

	bool same(binding const &Other) const
	{
		return type == Other.type && node == Other.node && include == Other.include;
	}
};

// placement of a bound object, to take a change back
struct object_state
{
	binding saved;
	glm::dvec3 location{0.0};
	glm::vec3 angles{0.f};
	std::array<glm::dvec3, 4> points{};
	std::string directive;
	bool moved{false}; // the object followed a change since the state was taken
};

} // namespace infra
