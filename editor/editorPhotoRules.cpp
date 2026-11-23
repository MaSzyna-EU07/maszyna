/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorPhotoRules.hpp"
#include "editor/editorGeometry.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <set>

namespace photo_rules
{

namespace
{

using square = std::pair<int, int>;

bool known(std::optional<glm::dvec2> const &Origin)
{
	return Origin.has_value() && (Origin->x != 0.0 || Origin->y != 0.0);
}

// the squares 4-connected, the holes in them filled; empty when they aren't one piece
std::set<square> filled(std::set<square> const &Squares)
{
	if (Squares.empty())
		return {};
	int constexpr steps[4][2]{{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
	{
		std::set<square> reached{*Squares.begin()};
		std::deque<square> next{*Squares.begin()};
		while (false == next.empty())
		{
			auto const at{next.front()};
			next.pop_front();
			for (auto const &step : steps)
				if (square const beside{at.first + step[0], at.second + step[1]}; Squares.contains(beside) && reached.insert(beside).second)
					next.push_back(beside);
		}
		if (reached.size() != Squares.size())
			return {};
	}
	// the empty squares reached from beyond the edge of the box around them, the others are holes
	square low{*Squares.begin()};
	square high{*Squares.begin()};
	for (auto const &at : Squares)
	{
		low = {std::min(low.first, at.first), std::min(low.second, at.second)};
		high = {std::max(high.first, at.first), std::max(high.second, at.second)};
	}
	low = {low.first - 1, low.second - 1};
	high = {high.first + 1, high.second + 1};
	std::set<square> outside{low};
	std::deque<square> next{low};
	while (false == next.empty())
	{
		auto const at{next.front()};
		next.pop_front();
		for (auto const &step : steps)
		{
			square const beside{at.first + step[0], at.second + step[1]};
			if (beside.first < low.first || beside.second < low.second || beside.first > high.first || beside.second > high.second || Squares.contains(beside))
				continue;
			if (outside.insert(beside).second)
				next.push_back(beside);
		}
	}
	std::set<square> result;
	for (int y = low.second; y <= high.second; ++y)
		for (int x = low.first; x <= high.first; ++x)
			if (false == outside.contains({x, y}))
				result.insert({x, y});
	return result;
}

std::vector<glm::dvec2> outline_of(std::set<square> const &Squares, double const Size, double const Inset)
{
	// the edges along which a square of the set meets one which isn't, the set on their left: counter-clockwise around it
	std::map<square, square> edges; // from a corner to the next one
	auto const add = [&](square const From, square const To) { edges[From] = To; };
	for (auto const &[x, y] : Squares)
	{
		if (false == Squares.contains({x, y - 1}))
			add({x, y}, {x + 1, y});
		if (false == Squares.contains({x + 1, y}))
			add({x + 1, y}, {x + 1, y + 1});
		if (false == Squares.contains({x, y + 1}))
			add({x + 1, y + 1}, {x, y + 1});
		if (false == Squares.contains({x - 1, y}))
			add({x, y + 1}, {x, y});
	}
	if (edges.empty())
		return {};
	std::vector<square> corners;
	auto const first{edges.begin()->first};
	auto at{first};
	do
	{
		corners.push_back(at);
		auto const found{edges.find(at)};
		if (found == edges.end() || corners.size() > edges.size())
			return {};
		at = found->second;
	} while (at != first);
	// one outline goes around all of them, there are no holes
	if (corners.size() != edges.size())
		return {};
	std::vector<glm::dvec2> result;
	auto const count{corners.size()};
	for (std::size_t i = 0; i < count; ++i)
	{
		auto const &previous{corners[(i + count - 1) % count]};
		auto const &here{corners[i]};
		auto const &next{corners[(i + 1) % count]};
		glm::dvec2 const in{static_cast<double>(here.first - previous.first), static_cast<double>(here.second - previous.second)};
		glm::dvec2 const out{static_cast<double>(next.first - here.first), static_cast<double>(next.second - here.second)};
		// the corners along a straight edge left out
		if (in == out)
			continue;
		// the edges in and out moved inwards, onto their left: the corner where they meet then
		glm::dvec2 const left_in{-in.y, in.x};
		glm::dvec2 const left_out{-out.y, out.x};
		result.push_back(glm::dvec2{here.first * Size, here.second * Size} + (left_in + left_out) * Inset);
	}
	return result;
}

} // namespace

origin_choice choose_origin(std::optional<glm::dvec2> const &Georeference, std::optional<glm::dvec2> const &Mark, std::optional<glm::dvec2> const &Remembered)
{
	origin_choice result;
	if (known(Georeference))
	{
		result.origin = *Georeference;
		result.from = origin_source::georeference;
		for (auto const *other : {&Mark, &Remembered})
			if (known(*other))
				result.differs = std::max(result.differs, glm::distance(**other, *Georeference));
		return result;
	}
	if (known(Mark))
	{
		result.origin = *Mark;
		result.from = origin_source::mark;
	}
	else if (known(Remembered))
	{
		result.origin = *Remembered;
		result.from = origin_source::remembered;
	}
	return result;
}

switched photo_on(std::optional<bool> const &User, std::optional<bool> const &Scenery)
{
	if (User.has_value())
		return {*User, true};
	if (Scenery.has_value())
		return {*Scenery, true};
	return {false, false};
}

double retry_delay(int const Failures)
{
	if (Failures <= 0)
		return 0.0;
	return std::min(retry_most, 5.0 * std::pow(2.0, std::min(Failures, 20) - 1));
}

bool hires_wanted(double const MetresPerPixel, bool const Now)
{
	if (MetresPerPixel <= 0.0)
		return false;
	return MetresPerPixel <= (Now ? hires_until : hires_from);
}

std::string failure_reason(std::string const &Error)
{
	auto const has = [&](char const *Text) { return Error.find(Text) != std::string::npos; };
	if (has("12007") || has("resolve") || has("12005"))
		return "no connection to the internet: the name of the server isn't found";
	if (has("12002") || has("imed out") || has("Timeout"))
		return "the server didn't answer in time";
	if (has("12029") || has("12030") || has("12031") || has("12152") || has("onnect"))
		return "the server can't be reached";
	if (has("12175") || has("12157") || has("12045") || has("SSL") || has("certificate"))
		return "the secure connection to the server failed";
	if (has("HTTP 5"))
		return "the server is busy or out of order";
	if (has("HTTP 404"))
		return "the server has no photo of the place";
	if (has("HTTP 4"))
		return "the server refused the request";
	if (has("unexpected response"))
		return "the server sent an error in place of the photo";
	if (has("decoding"))
		return "the photo came damaged";
	if (has("too large") || has("larger"))
		return "the photo came too large";
	return "the download failed";
}

std::vector<glm::dvec2> squares_outline(std::vector<std::pair<int, int>> const &Squares, double const Size, double const Inset)
{
	auto const whole{filled({Squares.begin(), Squares.end()})};
	if (whole.empty())
		return {};
	return outline_of(whole, Size, Inset);
}

growth grow_area(std::vector<glm::dvec2> const &Polygon, glm::dvec2 const &Point, double const Tile, int const Margin, int const Reach, std::size_t const Limit)
{
	growth result;
	if (Polygon.size() < 3 || Tile <= 0.0)
		return result;
	auto const found{geometry::squares_in(Polygon, Tile)};
	std::set<square> const area{found.begin(), found.end()};
	result.total = area.size();
	if (area.empty())
		return result;
	square const centre{static_cast<int>(std::floor(Point.x / Tile)), static_cast<int>(std::floor(Point.y / Tile))};
	// far from the area the point is elsewhere, not at its edge: the area isn't carried after it
	auto nearest{std::numeric_limits<int>::max()};
	for (auto const &at : area)
		nearest = std::min(nearest, std::max(std::abs(at.first - centre.first), std::abs(at.second - centre.second)));
	if (nearest > Margin)
		return result;
	auto grown{area};
	for (int dy = -Reach; dy <= Reach; ++dy)
		for (int dx = -Reach; dx <= Reach; ++dx)
			grown.insert({centre.first + dx, centre.second + dy});
	if (grown.size() == area.size())
		return result;
	auto const whole{filled(grown)};
	if (whole.empty())
		return result;
	if (whole.size() > Limit)
	{
		result.limited = true;
		return result;
	}
	// the polygon touches only the squares of the grid it's drawn over, a little inside their edges
	result.polygon = outline_of(whole, Tile, std::min(1.0, Tile * 0.01));
	if (result.polygon.size() < 3)
	{
		result.polygon.clear();
		return result;
	}
	result.total = whole.size();
	result.added = whole.size() - area.size();
	return result;
}

} // namespace photo_rules
