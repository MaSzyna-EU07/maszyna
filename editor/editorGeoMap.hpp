/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstdint>
#include <future>
#include <string>
#include <vector>

#include <glm/glm.hpp>

// the place of a scenery in Poland: PUWG 1992 (EPSG:2180), the metres of the Polish maps, as (east, north)
namespace editor_geo
{

glm::dvec2 to_puwg(double const Latitude, double const Longitude);
// latitude, longitude
glm::dvec2 to_degrees(glm::dvec2 const &Point);
// two numbers as a map gives them: degrees (latitude, longitude, as the online maps copy them) or the X and Y of PUWG 1992
// (north, then east, as geoportal.gov.pl shows them); false when they're neither, or the place isn't in Poland
bool parse_place(std::string const &Text, glm::dvec2 &Point);
bool in_poland(glm::dvec2 const &Point);

} // namespace editor_geo

// the map of Poland to pick a place on: the base map of OpenStreetMap from geoportal.gov.pl, the wheel zooms it, a drag moves
// it, a click picks the place
class editor_map
{
  public:
	editor_map() = default;
	~editor_map();
	editor_map(editor_map const &) = delete;
	editor_map &operator=(editor_map const &) = delete;

	// at the cursor, Size big; true when a place was clicked
	bool render(glm::vec2 const &Size);
	void pick(glm::dvec2 const &Point);
	// the map goes there, at most Scale m a pixel
	void show(glm::dvec2 const &Point, double const Scale = 4.0);
	// the map moves and zooms in or out to show the rectangle whole, with a margin around it
	void fit(glm::dvec2 const &Low, glm::dvec2 const &High);
	bool picked() const { return m_picked; }
	glm::dvec2 const &point() const { return m_point; }
	// where the last click was; with picking off a click leaves the place picked as it is, the caller takes the point
	glm::dvec2 const &clicked() const { return m_clicked; }
	void picking(bool const Picking) { m_picking = Picking; }
	// m a pixel, the zoom of the map
	double scale() const { return m_scale; }
	// an area drawn over the map, of PUWG points: closed, it's filled; open, its points are shown, being drawn
	void outline(std::vector<glm::dvec2> Points, bool const Closed)
	{
		m_outline = std::move(Points);
		m_outline_closed = Closed;
	}
	// a line written at the bottom of the map, a word of what a click does there; empty: none
	void hint(std::string Text) { m_hint = std::move(Text); }

  private:
	struct image
	{
		glm::dvec2 low{0.0};
		glm::dvec2 high{0.0};
		int width{0};
		int height{0};
		std::vector<std::uint8_t> pixels;
		std::string error;
	};

	void request(glm::vec2 const &Size);
	void upload();
	void release();

	glm::dvec2 m_centre{515000.0, 460000.0};
	double m_scale{1000.0}; // m a pixel
	bool m_fitted{false};
	double m_changed{0.0};
	bool m_dirty{true};
	glm::vec2 m_pressed{0.f};
	bool m_dragging{false};
	std::future<image> m_request;
	std::uint64_t m_texture{0};
	glm::dvec2 m_image_low{0.0};
	glm::dvec2 m_image_high{0.0};
	std::string m_error;
	bool m_picked{false};
	glm::dvec2 m_point{0.0};
	bool m_picking{true};
	glm::dvec2 m_clicked{0.0};
	glm::vec2 m_size{0.f}; // of the last render
	std::vector<glm::dvec2> m_outline;
	bool m_outline_closed{false};
	std::string m_hint;
};
