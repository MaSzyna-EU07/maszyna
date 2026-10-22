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

enum class editor_track_tab { straights, route, path, turnout, profile, speed, joints, infra, lay, lineside, turntable, signals, parallel, srk, kilometrage };

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

// a signal to stand by a track, of the kind of the signal templates: main, shunt...
struct signal_order
{
	TTrack *track{nullptr};
	int path{0};
	glm::dvec3 point{0.0}; // on the axis
	int facing{1}; // the trains it faces run 1: towards the end of the path, -1: towards its start
	int side{1}; // 1: on the right of the path, -1: on the left
	std::string kind;
	std::string name;
	std::string plate; // the name on the plate under the head, when the template has one
	std::string linked{"none"}; // the signal before it, when the template takes one: a block signal shows its aspects by it
	std::string file; // the template, of its kind; empty for the one chosen for the kind
};
struct signal_rename
{
	scene::instance_handle instance{0};
	std::string name;
	std::string plate;
	std::string linked; // the same, empty to keep the one it has
};
struct standing_signal
{
	scene::instance_handle instance{0};
	std::string name;
	std::string kind;
	glm::dvec3 position{0.0};
	double yaw{0.0};
	std::string linked; // of a template that takes one, else empty
	std::string file; // of its template
	TTrack *track{nullptr}; // the trains read it on, nullptr while none does
	int facing{0}; // the trains it faces run 1: towards the end of the path of the track, -1: towards its start, 0 unknown
	std::string read; // its event the trains read, empty for a template without one
};
// a signal template as the signalling picks one: the lights it has and the speeds its events show
struct signal_head
{
	std::string file;
	std::string name;
	std::string kind;
	std::string mount;
	std::string lamps; // its chambers from the top: z green, c red, p orange, b white, n blue, x blind
	std::vector<int> speeds; // of the signal itself, km/h: -1 the highest allowed, 20 the substitute signal
	bool chosen{false}; // the Signals tool places it for its kind
	int aspects{0}; // of a block signal: 3, or 4 with the warning two signals ahead
	std::string lean; // its head leans towards the track: left, standing on the right of it, right, or none
};
// one step of the history of the signalling: signals placed, standing ones renamed, other templates placed with them, the values
// of standing ones replaced
struct signal_changes
{
	std::vector<signal_order> placed;
	std::vector<signal_rename> renamed;
	std::vector<template_item> others;
	std::vector<std::pair<scene::instance_handle, std::vector<std::string>>> values;
	std::vector<std::pair<scene::instance_handle, std::string>> files; // standing signals given another template, of the same kind, their values kept
	std::vector<scene::instance_handle> removed; // includes taken away, the signals among them with their read events
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

inline std::map<std::string, std::string> selftest_values(std::istringstream &Words)
{
	std::map<std::string, std::string> values;
	std::string pair;
	while (Words >> pair)
	{
		auto const equals{pair.find('=')};
		if (equals != std::string::npos)
			values[pair.substr(0, equals)] = pair.substr(equals + 1);
	}
	return values;
}

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
	// changes with each step of the history taken, undone or redone, also once the oldest steps are dropped
	virtual std::size_t history_revision() const = 0;
	virtual void push_track_snapshot(std::vector<std::pair<TTrack *, editor_track::state>> States, std::vector<TTrack *> Created = {}, std::vector<TTrack *> Removed = {}) = 0;
	// the changes of the kilometrage made since the last step: one step of the history
	virtual void push_kilometrage_snapshot() = 0;

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

class editor_signals
{
  public:
	// false while the signal templates are still being looked for
	virtual bool standing_signals(std::vector<standing_signal> &Signals, bool const Wait) = 0;
	// the signals placed by the templates chosen for their kinds and the rest of the changes, with the tracks changed before: one
	// step of the history. returns: how many signals were placed
	virtual std::size_t signal_batch(signal_changes const &Changes, std::vector<std::pair<TTrack *, editor_track::state>> States, std::vector<TTrack *> Created, std::string &Error) = 0;
	// the signal templates with lights, false while they are still being looked for
	virtual bool signal_heads(std::vector<signal_head> &Heads, bool const Wait) = 0;
	// the signals given which no track reads, read again by the track they stand by: one step of the history. returns: how many
	virtual std::size_t signal_reattach(std::vector<scene::instance_handle> const &Instances) = 0;

  protected:
	~editor_signals() = default;
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

class editor_services : public editor_selection, public editor_view, public editor_history, public editor_templates, public editor_signals, public editor_ground, public editor_track_design, public editor_guide
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
