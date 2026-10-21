/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/editormode.h"
#include "editor/editorGeometry.hpp"
#include "editor/editorGroundFollow.hpp"

#include "utilities/Globals.h"
#include "world/Track.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{

using geometry::plan_distance;
using geometry::plan_of;

} // namespace

glm::dvec3 editor_mode::cursor_terrain(double const Height)
{
	auto point{cursor_level(Height)};
	for (int i = 0; i < 5 && std::isfinite(point.x) && std::isfinite(point.z); ++i)
	{
		std::vector<char> found;
		auto ground{ground_heights({glm::dvec3{point.x, std::max(point.y, static_cast<double>(Global.pCamera.Pos.y)), point.z}}, true, &found).front()};
		if (found[0] == 0 && false == nmt_ahead(point.x, point.z, ground))
			ground = nearby_ground(point);
		if (std::isnan(ground))
			return point;
		auto const next{cursor_level(ground)};
		auto const settled{plan_distance(next, point) < 0.01};
		point = {next.x, ground, next.z};
		if (settled)
			break;
	}
	return point;
}

double editor_mode::nearby_ground(glm::dvec3 const &Point) const
{
	auto const ceiling{std::max(Point.y, static_cast<double>(Global.pCamera.Pos.y))};
	for (double const radius : {10.0, 25.0, 50.0, 100.0, 200.0, 400.0})
	{
		std::vector<glm::dvec3> probes;
		for (int i = 0; i < 12; ++i)
		{
			auto const angle{glm::two_pi<double>() * i / 12.0};
			probes.push_back({Point.x + radius * std::cos(angle), ceiling, Point.z + radius * std::sin(angle)});
		}
		std::vector<char> found;
		auto const heights{ground_heights(probes, true, &found)};
		double sum{0.0};
		int count{0};
		for (std::size_t i = 0; i < probes.size(); ++i)
			if (found[i] != 0)
			{
				sum += heights[i];
				++count;
			}
		if (count > 0)
			return sum / count;
	}
	return std::numeric_limits<double>::quiet_NaN();
}

std::pair<double, double> editor_mode::trackbed(TTrack const *Style)
{
	return Style != nullptr ? profile_ballast(*Style) : std::pair<double, double>{0.2, 0.5 * 1.435 + 0.5 + 1.1};
}

std::vector<double> editor_mode::bed_heights(std::vector<glm::dvec3> const &Points, std::vector<glm::dvec2> const &Directions, TTrack const *Style) const
{
	auto const [depth, half]{trackbed(Style)};
	std::vector<glm::dvec3> probes;
	probes.reserve(Points.size() * 3);
	for (std::size_t i = 0; i < Points.size(); ++i)
	{
		auto const &direction{Directions[i]};
		auto const side{glm::length(direction) > 1e-6 ? glm::normalize(glm::dvec2{-direction.y, direction.x}) * half : glm::dvec2{0.0}};
		probes.push_back(Points[i]);
		probes.push_back(Points[i] + glm::dvec3{side.x, 0.0, side.y});
		probes.push_back(Points[i] - glm::dvec3{side.x, 0.0, side.y});
	}
	std::vector<char> found;
	auto heights{ground_heights(probes, true, &found)};
	for (std::size_t i = 0; i < probes.size(); ++i)
		if (found[i] == 0 && nmt_ahead(probes[i].x, probes[i].z, heights[i]))
			found[i] = 1;
	std::vector<double> result(Points.size(), std::numeric_limits<double>::quiet_NaN());
	for (std::size_t i = 0; i < Points.size(); ++i)
	{
		auto const axis{found[3 * i] != 0 ? heights[3 * i] : std::numeric_limits<double>::quiet_NaN()};
		auto toe{std::numeric_limits<double>::infinity()};
		for (std::size_t j = 3 * i + 1; j < 3 * i + 3; ++j)
			if (found[j] != 0)
				toe = std::min(toe, heights[j]);
		if (std::isfinite(toe))
			result[i] = std::isnan(axis) ? toe + depth : std::max(toe + depth, axis);
		else if (false == std::isnan(axis))
			result[i] = axis + depth;
	}
	return result;
}

double editor_mode::bed_height(glm::dvec3 const &Point, glm::dvec2 const &Direction, TTrack const *Style) const
{
	auto const height{bed_heights({Point}, {Direction}, Style).front()};
	if (false == std::isnan(height))
		return height;
	auto const nearby{nearby_ground(Point)};
	return std::isnan(nearby) ? Point.y : nearby + trackbed(Style).first;
}

void editor_mode::follow_ground(std::vector<segment_data> &Pieces, double const Height0, double const Grade0, double const Height1, double const Grade1, TTrack const *Style) const
{
	ground_follow::follow(Pieces, {Height0, Grade0, Height1, Grade1}, [&](std::vector<glm::dvec3> const &Points, std::vector<glm::dvec2> const &Directions) { return bed_heights(Points, Directions, Style); });
}

glm::dvec3 editor_mode::lay_bed(glm::dvec3 const &Point)
{
	auto const &tool{m_lay};
	TTrack const *style{tool.start.track != nullptr ? tool.start.track : tool.copy_selected ? selected_track() : nullptr};
	return {Point.x, bed_height(Point, tool.points.empty() ? glm::dvec2{0.0} : plan_of(Point - tool.points.back()), style), Point.z};
}
