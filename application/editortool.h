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
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "editor/editorTrack.hpp"

class TTrack;
class editor_nmt;
namespace scene
{
class basic_node;
}

enum class editor_selftest_result
{
	unhandled,
	handled,
	pause,
	again
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

class editor_services
{
  public:
	virtual TTrack *selected_track() const = 0;
	virtual bool track_editing() const = 0;
	virtual std::size_t history_size() const = 0;
	virtual glm::dvec3 camera_position() const = 0;
	virtual void fly_to(glm::dvec3 const &Target, glm::dvec3 const &Look, double const Distance, double const Duration = 0.6) = 0;
	virtual void focus_track(TTrack &Track, int const Path, glm::dvec3 const &Point) = 0;
	virtual void set_current_node(scene::basic_node *Node) = 0;
	virtual alignment::limits const &route_norms() const = 0;
	virtual void clear_track_point() = 0;
	virtual void nmt_locate() = 0;
	virtual std::vector<nmt_place> nmt_track_places(double const Step) const = 0;
	virtual editor_nmt &terrain_model() = 0;
	virtual double terrain_model_offset() const = 0;
	virtual bool terrain_model_fitted() const = 0;
	virtual void push_track_snapshot(std::vector<std::pair<TTrack *, editor_track::state>> States, std::vector<TTrack *> Created = {}, std::vector<TTrack *> Removed = {}) = 0;

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
