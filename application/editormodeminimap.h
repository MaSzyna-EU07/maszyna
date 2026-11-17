/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "application/editortool.h"

#include <string>
#include <vector>

class track_ride;

// the tracks of the scenery seen from above with the camera on them, and the stations the stop indicators (W4) name, to find
// one by its name and go there
class minimap_tool final : public editor_tool
{
  public:
	// a stop of passenger trains, as a W4 marks it
	struct stop
	{
		glm::dvec3 point{0.0};
		TTrack *track{nullptr};
	};
	struct station
	{
		std::string name; // as shown
		std::string key; // lower case, without the polish letters, the stops of the same name gathered by it
		std::vector<stop> stops;
		std::size_t next{0}; // the stop gone to the next time
	};

	minimap_tool(editor_view &View, editor_history &History, track_ride &Ride) : m_view{View}, m_history{History}, m_ride{Ride} {}
	void render_windows() override;
	void forget(scene::basic_node const *Node) override;
	editor_selftest_result selftest(std::string const &Command, std::istringstream &Words) override;
	bool is_open() const { return m_open; }
	void toggle() { m_open = !m_open; }
	// the stations of the scenery, gathered again where the scenery changed
	std::vector<station> const &stations();
	// the camera to the next stop of the station: riding along the track it rides on from there, else it flies over to it
	void go_to(station &Station);
	// the camera to the track nearest to the point of the plan, or above the point where there's none near
	void go_to_point(glm::dvec3 const &Point, double const Reach);

  private:
	// a path of a rail track as the map draws it
	struct line
	{
		TTrack *track{nullptr};
		glm::dvec2 low{0.0};
		glm::dvec2 high{0.0};
		std::vector<glm::dvec2> points; // x, z
	};
	void gather_lines();
	void gather_stations();
	void render_search();
	void render_map(float const Height);
	bool stale(std::size_t &Seen) const;

	editor_view &m_view;
	editor_history &m_history;
	track_ride &m_ride;
	bool m_open{false};
	std::vector<line> m_lines;
	glm::dvec2 m_low{0.0}; // of all the lines
	glm::dvec2 m_high{0.0};
	std::size_t m_lines_seen{~std::size_t{0}};
	std::vector<station> m_stations;
	std::size_t m_stations_seen{~std::size_t{0}};
	char m_search[64]{};
	std::string m_marked; // the key of the station last gone to, drawn larger
	// the map
	glm::dvec2 m_centre{0.0}; // x, z
	double m_scale{4.0}; // m a pixel
	bool m_follow{true}; // the centre goes with the camera
	bool m_fitted{false};
	glm::vec2 m_pressed{0.0f};
	bool m_dragging{false};
};
