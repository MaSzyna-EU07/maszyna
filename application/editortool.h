/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "editor/editorTrack.hpp"
#include "utilities/Classes.h"

class TTrack;
class editor_nmt;

enum class editor_selftest_result
{
	unhandled,
	handled,
	pause,
	again
};

enum class editor_first_step { tracks, lay, laid, wheel, join, turnout, undo, save, drive, look, top, select, shape, profile, profile_point, profile_apply, terrain, nmt, plan, plan_band, plan_apply, signal, posts, count };

enum class editor_track_tab { straights, route, path, turnout, profile, speed, joints, infra, lay, lineside, turntable, signals, parallel };

struct track_hover
{
	TTrack *track{nullptr};
	int path{0};
	glm::dvec3 point{0.0};
	double since{0.0};
};

struct tool_tab
{
	bool open{false};
	bool expand{false};
};

// scenery templates standing by the track, placed and checked by the editor
struct template_item
{
	std::string file;
	glm::dvec3 location{0.0}; // world space
	double yaw{0.0};
	std::vector<std::string> rest; // parameters after the rotation
	bool described{false};
	std::string track;
	glm::dvec2 tilt{0.0}; // rot.x and rot.z of a described template, degrees
	std::map<std::string, std::string> values;
};
struct standing_template
{
	scene::instance_handle instance{0};
	std::string file; // lower case, forward slashes
	glm::dvec3 location{0.0};
	double yaw{0.0};
	std::vector<std::string> values;
};

struct nmt_place
{
	glm::dvec3 position{0.0}; // on the axis of the path
	double formation{0.0}; // the height of the formation under it, the foot of the ballast
	double crown{0.0}; // the height of the top of the ballast under the rails: what the terrain model measures on a track
	double half_width{3.0}; // from the axis to the foot of the ballast
	TTrack *track{nullptr};
	int path{0};
	bool shapes{true}; // not a bridge nor a tunnel
};

class editor_selection
{
  public:
	virtual TTrack *selected_track() const = 0;
	virtual std::vector<TTrack *> const &track_set() const = 0;
	virtual track_hover const &hovered_track() const = 0;
	virtual void set_current_node(scene::basic_node *Node) = 0;
	virtual void clear_track_point() = 0;

  protected:
	~editor_selection() = default;
};

class editor_view
{
  public:
	virtual glm::dvec3 camera_position() const = 0;
	virtual void fly_to(glm::dvec3 const &Target, glm::dvec3 const &Look, double const Distance, double const Duration = 0.6) = 0;
	virtual void focus_track(TTrack &Track, int const Path, glm::dvec3 const &Point) = 0;
	virtual editor_track_tab current_track_tab() const = 0;
	virtual void show_track_tab(editor_track_tab const Wanted) = 0;

  protected:
	~editor_view() = default;
};

class editor_history
{
  public:
	virtual std::size_t history_size() const = 0;
	virtual void push_track_snapshot(std::vector<std::pair<TTrack *, editor_track::state>> States, std::vector<TTrack *> Created = {}, std::vector<TTrack *> Removed = {}) = 0;

  protected:
	~editor_history() = default;
};

class editor_templates
{
  public:
	virtual std::vector<standing_template> standing_templates(std::function<bool(std::string const &)> const &Accept) const = 0;
	virtual std::size_t place_templates(std::vector<template_item> const &Items, std::string &Error) = 0;
	virtual std::size_t move_templates(std::vector<std::pair<standing_template, template_item>> const &Moves) = 0;
	virtual std::size_t remove_templates(std::vector<scene::instance_handle> const &Instances) = 0;

  protected:
	~editor_templates() = default;
};

class editor_ground
{
  public:
	virtual glm::dvec3 placement_on_ground(glm::dvec3 Location) = 0;
	virtual void nmt_locate() = 0;
	virtual std::vector<nmt_place> nmt_track_places(double const Step) const = 0;
	virtual editor_nmt &terrain_model() = 0;
	virtual double terrain_model_offset() const = 0;
	virtual bool terrain_model_fitted() const = 0;

  protected:
	~editor_ground() = default;
};

class editor_track_design
{
  public:
	virtual bool track_editing() const = 0;
	virtual alignment::limits const &route_norms() const = 0;
	virtual bool digitize_drawn(std::vector<TTrack *> const &Created, bool const Alone = true) = 0;
	virtual void straight_refresh() = 0;

  protected:
	~editor_track_design() = default;
};

class editor_guide
{
  public:
	virtual void first_step_done(editor_first_step const Step) = 0;

  protected:
	~editor_guide() = default;
};

class editor_services : public editor_selection, public editor_view, public editor_history, public editor_templates, public editor_ground, public editor_track_design, public editor_guide
{
  protected:
	~editor_services() = default;
};

class editor_tool
{
  public:
	virtual ~editor_tool() = default;
	virtual void update() {}
	virtual void draw_overlay() const {}
	virtual void render_options() {}
	virtual void render_windows() {}
	virtual void forget(scene::basic_node const *Node) {}
	virtual editor_selftest_result selftest(std::string const &Command, std::istringstream &Words) { return editor_selftest_result::unhandled; }
};
