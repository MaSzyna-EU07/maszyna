/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "application/editormodeinfrastate.h"
#include "application/editormodelaystate.h"
#include "application/editormodenmtstate.h"
#include "application/editormodeprofilestate.h"
#include "application/editormoderoadstate.h"
#include "application/editormoderoutestate.h"
#include "application/editormodestraightsstate.h"
#include "application/editormodesweepstate.h"
#include "scene/scenenode.h"

#include <string>
#include <vector>

#include <glm/glm.hpp>

class editor_ui;

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

// what the models laid and bent along the track need from the editor: the state of the one laid, its curve, the steps of the history
// and the model or include selected to be bent
class editor_sweep_services
{
  public:
	virtual sweep_tool &sweep_state() = 0;
	// the curve the model is laid along: the axis of the selected track, once a step of the history
	virtual void sweep_curve() = 0;
	// a model laid along the track made or taken away: one step of the history
	virtual void push_sweep_toggled(sweep_node &Sweep) = 0;
	// a model laid along the track changed from the state Before: one step of the history
	virtual void push_sweep_change(sweep_node &Sweep, sweep_node::state Before) = 0;
	// the model bent made of the source, which is taken out of the scenery: one step of the history
	virtual void push_bend_made(sweep_node &Sweep, bend_source const &Source) = 0;
	// the model or the include selected, as a model to be bent. returns: false if there's none to bend
	virtual bool bend_source_of_selection(bend_source &Source) const = 0;

  protected:
	~editor_sweep_services() = default;
};

// what the build tools (laying track, routes, straights, roads, sweeps, vertical profiles and grade lines, the terrain model, connections and the infrastructure) need from the editor beyond the interfaces of editortool.h; editor_mode implements it
class editor_build_services : public editor_road_services, public editor_sweep_services
{
  public:
	// the panels of the editor
	virtual editor_ui *build_ui() const = 0;
	// the node selected, nullptr for none, and the include selected, 0 for none
	virtual scene::basic_node *current_node() const = 0;
	virtual scene::instance_handle current_instance() const = 0;
	// nothing selected any more: no node, no include, no drag of the node
	virtual void drop_node_selection() = 0;
	virtual bool right_button_held() const = 0;

  protected:
	~editor_build_services() = default;
};
