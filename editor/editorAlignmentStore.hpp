/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>
#include <glm/glm.hpp>

#include "editor/editorAlignment.hpp"
#include "scene/scenelayers.h"

class TTrack;

// horizontal alignments designed in the track editor, kept in the scenery files as "//$h" comment lines: the design (its
// vertices) and the elements it makes, so the tracks laid by it are known for straights, transition curves and arcs
namespace alignment_store
{

// an element as the file gives it: radii signed with the turn, 0 for no curvature
struct stored_element
{
	std::string kind; // straight, transition, arc
	int vertex{-1};
	double chainage{0.0};
	double length{0.0};
	double radius_start{0.0};
	double radius_end{0.0};
	double cant{0.0};
};
stored_element describe(alignment::element const &Element);

struct record
{
	scene::layer_handle layer{null_handle};
	std::string source{"designed"};
	double origin{0.0}; // chainage of the start, m
	alignment::design design;
	alignment::result result; // made of the design
	std::vector<stored_element> elements; // as written in the file
	bool consistent{true}; // the design makes the elements written in the file
	glm::dvec2 low{0.0}, high{0.0};
	std::vector<glm::dvec2> samples;
};

struct hit
{
	int record{-1};
	int element{-1};
	double chainage{0.0}; // from the start of the record
	double offset{0.0}; // sideways from its line
};

// the designed vertex a curve made of the tracks follows: its parameters in the direction of the tracks
struct designed_vertex
{
	int record{-1};
	int vertex{-1};
	alignment::vertex shape;
	bool reversed{false};
};

double constexpr tolerance{0.02};

void reset();
void load();
std::vector<record> const &records();
std::optional<hit> at(glm::dvec3 const &Point, double const Reach = tolerance);
// the track lies along a stored alignment, its middle is the hit
std::optional<hit> on_track(TTrack const &Track, double const Reach = tolerance);
// the vertex the tracks follow, if they all lie along one stored alignment and the curve elements under them belong to one vertex
std::optional<designed_vertex> vertex_of(std::vector<std::pair<TTrack *, bool>> const &Run);
// stored alignment with these ends, either way
int find(glm::dvec3 const &Start, glm::dvec3 const &End, bool &Reversed);
// stored alignment near the tracks, whose line they no longer follow; Departure receives the largest distance
int departed(std::vector<TTrack *> const &Tracks, double &Departure);
// keeps the design in the file of the layer, in place of the one with the same ends
int store(alignment::design const &Design, alignment::result const &Result, double const Origin, scene::layer_handle const Layer, std::string const &Source = "designed");
bool follows(alignment::result const &Result, std::vector<TTrack *> const &Tracks);
std::optional<double> chainage_on(alignment::result const &Result, glm::dvec3 const &Point);
void forget(int const Index);
// the point on the line of the stored alignment, if it lies by it within the reach
std::optional<hit> on_record(int const Index, glm::dvec3 const &Point, double const Reach);
// cant of the design at the chainage, mm
double cant_at(int const Index, double const Chainage);
// the stored alignment moved and turned in the plan the way the point and the direction were
void transform(int const Index, glm::dvec3 const &From, glm::dvec2 const &Fromdirection, glm::dvec3 const &To, glm::dvec2 const &Todirection);
// km of the point on the stored alignments, if it lies on one
std::optional<double> chainage_at(glm::dvec3 const &Point);
// the records as they were before the changes made since the last call, if there were any; the current ones become the base
std::optional<std::vector<record>> take_changes();
// the current records become the base, the changes made so far aren't a part of the undo history
void settle();
// puts the records back as they were, for undo and redo
void restore(std::vector<record> Records);
// layer the editor keeps the records of the track in: its own, unless it's a terrain file (.txtf), whose binary version
// (.btf) the editor doesn't read the records from
scene::layer_handle layer_of(TTrack const &Track);

} // namespace alignment_store
