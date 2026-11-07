/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "application/editormoderoadstate.h"

#include <string>
#include <vector>

#include <glm/glm.hpp>

// what the road window needs from the editor: its state, the ground under the roads, its history and the material fields
class editor_road_services
{
  public:
	virtual road_tool &road_state() = 0;
	// height of the ground under each of the points, with the roads not taken for the ground. a point with nothing under it keeps
	// its height. Found: set for the points which have the ground under them, nullptr for none
	virtual std::vector<double> ground_heights_at(std::vector<glm::dvec3> const &Points, bool const Fresh, std::vector<char> *Found) const = 0;
	// the changes of the roads, the junctions and the points: one step of the history
	virtual void push_road_snapshot(editor_road::record Record) = 0;
	// a field of the material of a part of a road or of a junction. returns: true if the material was changed
	virtual bool render_road_material(char const *Label, std::string &Value) = 0;

  protected:
	~editor_road_services() = default;
};

// what the build tools (laying track, routes, straights, roads, sweeps, vertical profiles and grade lines, the terrain model, connections and the infrastructure) need from the editor beyond the interfaces of editortool.h; editor_mode implements it
class editor_build_services : public editor_road_services
{
  protected:
	~editor_build_services() = default;
};
