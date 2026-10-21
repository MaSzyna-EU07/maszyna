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

#include <glm/glm.hpp>

class TTrack;
namespace scene
{
class basic_node;
}

enum class editor_selftest_result
{
	unhandled,
	handled,
	pause
};

class editor_services
{
  public:
	virtual TTrack *selected_track() const = 0;
	virtual bool track_editing() const = 0;
	virtual std::size_t history_size() const = 0;
	virtual glm::dvec3 camera_position() const = 0;
	virtual void fly_to(glm::dvec3 const &Target, glm::dvec3 const &Look, double const Distance, double const Duration = 0.6) = 0;

  protected:
	~editor_services() = default;
};

class editor_tool
{
  public:
	virtual ~editor_tool() = default;
	virtual void update() {}
	virtual void render_windows() {}
	virtual void forget(scene::basic_node const *Node) {}
	virtual editor_selftest_result selftest(std::string const &Command, std::istringstream &Words) { return editor_selftest_result::unhandled; }
};
