/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <glm/glm.hpp>

#include <cstddef>
#include <string>
#include <vector>

#include "application/editormoderailstate.h"

class TTrack;

// what the rail tools (the track tab and its panels, the gizmo, track properties and bed, switches, signals, the turntable and the lineside) need from the editor beyond the interfaces of editortool.h; editor_mode implements it
class editor_rail_services
{
  public:
	// the list of the switch tool built again with the own templates, keeping the armed one or arming the one whose label starts with Arm
	virtual void switch_templates_refresh(std::string const &Arm) = 0;
	// the switch tool armed with its template chosen
	virtual void arm_switch_tool() = 0;
	// the point of the ground under the cursor, and the one at the height given
	virtual glm::dvec3 cursor_on_ground() const = 0;
	virtual glm::dvec3 cursor_on_level(double const Height) const = 0;
	// the ring of the cursor drawn at the point
	virtual void draw_ring(glm::dvec3 const &Point) const = 0;
	// the orthophoto is turned on, and its window opened
	virtual bool orthophoto_shown() const = 0;
	virtual void open_orthophoto_window() = 0;
	// a line of the status bar, Warning: in the colour of a warning
	virtual void show_status(std::string const &Text, bool const Warning) = 0;
	// the steps of the history kept
	virtual std::size_t history_size() const = 0;
	virtual void save_scenery() = 0;
	// the picker of a texture of the rails (Kind 0) or of the trackbed (1), in an open combo. returns: true when one is chosen, its name in Chosen
	virtual bool pick_track_texture(int const Kind, std::string const &Current, bool const Mixed, std::string &Chosen) = 0;
	// a material of the texture browser dropped on the last item. returns: true when one was, its name in Name
	virtual bool take_dropped_material(std::string &Name) = 0;
	// the set of the selected tracks when there's one, the selected track otherwise; and the selection made of the tracks given
	virtual std::vector<TTrack *> selected_track_list() = 0;
	virtual void select_track_list(std::vector<TTrack *> const &Tracks) = 0;
	// the tracks joined to the set added to it, the set made of From when there's none, and selected
	virtual void grow_selection(std::vector<TTrack *> const &From) = 0;
	// the track added to the set of the selected ones or taken out of it
	virtual void toggle_in_selection(TTrack &Track) = 0;

  protected:
	~editor_rail_services() = default;
};
