/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>
#include <glm/glm.hpp>

// the rules the aerial photo of the editor (editor_orthophoto) goes by, standing on their own: which origin of the scenery is
// taken, whether the photo is on, how the download area grows, when a tile is tried again, when the 4K photos come, and what a
// download that failed is told as
namespace photo_rules
{

// where the origin of the scenery comes from
enum class origin_source
{
	none,
	georeference, // //$g of the scenery file, as Rainsted and the wizard write it
	mark,         // //$o of the scenery, written by the editor
	remembered    // the settings of the editor, per scenery
};

struct origin_choice
{
	glm::dvec2 origin{0.0}; // northing, easting
	origin_source from{origin_source::none};
	// with //$g, how far the other origin known (the one remembered or the one of //$o) lies from it, the farther; 0 when none
	double differs{0.0};
};

// //$g goes first, as the file of the scenery says where it lies: an origin remembered by the editor for a scenery of the same name
// elsewhere, or one written into //$o with it, can't move it. Without //$g the editor's own //$o, then the one remembered. A //$o the
// user aligned by hand (MarkAligned, the word aligned in it) goes before //$g: the user moved the photo off it on purpose
origin_choice choose_origin(std::optional<glm::dvec2> const &Georeference, std::optional<glm::dvec2> const &Mark, std::optional<glm::dvec2> const &Remembered, bool const MarkAligned = false);
// the origins further apart than that are told of
double constexpr origin_tolerance{1.0};

// the photo on or off: as the user left it in this scenery, else as the scenery keeps it (//$o on, the wizard writes it on),
// else off. Chosen: the terrain model doesn't turn it on by itself
struct switched
{
	bool on{false};
	bool chosen{false};
};
switched photo_on(std::optional<bool> const &User, std::optional<bool> const &Scenery);

// a tile which failed is tried again after that many seconds, doubling from 5 s up to 5 min, and never given up for good
double retry_delay(int Failures);
double constexpr retry_most{300.0};

// the 4K photos (6 cm/px) by themselves when the view is zoomed in that far, m a pixel; off again only once zoomed out past the
// second value, so they don't come and go at the edge
double constexpr hires_from{0.15};
double constexpr hires_until{0.25};
bool hires_wanted(double MetresPerPixel, bool Now);

// what a download that failed is told as, in plain words (an English message to be translated); the raw error is in the log
std::string failure_reason(std::string const &Error);
// of these, the failures of the connection and of the server alone, whatever was downloaded (the terrain model too); empty: the
// error isn't one of them
std::string connection_reason(std::string const &Error);

// the download area grown to where the camera or the laying goes: the tiles of the polygon (PUWG easting, northing) and the square of
// Reach tiles around the point, as one outline along the grid of the tiles, its holes filled. Only when the point is within Margin
// tiles of the tiles of the area and the square isn't in it already; not past Limit tiles in all
struct growth
{
	std::vector<glm::dvec2> polygon; // empty: the area stays as it is
	std::size_t added{0};            // tiles
	std::size_t total{0};            // tiles of the area grown, or of the area as it is when it can't grow
	bool limited{false};             // it would grow past the limit
};
growth grow_area(std::vector<glm::dvec2> const &Polygon, glm::dvec2 const &Point, double Tile, int Margin, int Reach, std::size_t Limit);
// the outline of a set of squares of a grid of the size, 4-connected and with no holes, a little inside their edges (Inset m), so
// the squares it touches are the ones of the set and none beside; empty when the set isn't one piece
std::vector<glm::dvec2> squares_outline(std::vector<std::pair<int, int>> const &Squares, double Size, double Inset);

} // namespace photo_rules
