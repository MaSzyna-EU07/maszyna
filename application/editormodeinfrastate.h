/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "editor/editorInfra.hpp"
#include "scene/scenenode.h"

#include <string>
#include <vector>

#include <glm/glm.hpp>

class TTrack;

// infrastructure along the track: objects bound to the paths follow their changes. bindings are kept in the
// scenery files as "//$b" comment lines
struct infra_candidate
{
	infra::binding binding;
	double offset{0.0}; // sideways from the axis
	std::string reason;
	bool chosen{false};
	bool bound{false};
	TTrack *reader{nullptr};
	std::string event; // of that path
	std::string cell;
	bool loose{false};
	TTrack *nearer{nullptr};
	glm::dvec3 nearer_foot{0.0};
	bool unread{false};
};
struct infra_window
{
	bool open{false};
	int scope{1}; // 0: the selected path, 1: the line through it, 2: the route of the vertical profile
	float reach{1500.0f};
	float corridor{8.0f};
	bool follow{true};
	bool show{true};
	std::vector<infra::rule> rules{infra::default_rules()};
	std::vector<TTrack *> tracks; // of the scope
	std::vector<infra_candidate> candidates;
	int filter{0};
	int read{0};
	int overruled{0};
	int unread{0};
	int elsewhere{0};
	int hovered{-1};
	std::string status;
	std::string error;
};
// the point objects which go with a path: the signals and the warning discs it reads, the SHP resonators, and the controls of the
// block sections, the signs and the posts by the path nearest to them
struct point_object
{
	scene::instance_handle include{0};
	glm::dvec3 location{0.0};
	TTrack *reader{nullptr};
	TTrack *nearest{nullptr}; // of one no path reads
	infra::category group{infra::category::other};
};
