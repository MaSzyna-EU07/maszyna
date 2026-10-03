/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorInfra.hpp"

#include "editor/editorGeometry.hpp"
#include "world/Track.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>

namespace infra
{

namespace
{

using geometry::bezier;
using geometry::plan_distance;
using geometry::plan_of;

int const kSteps{128};

// arc length in the plan at the evenly spread parameters
std::array<double, kSteps + 1> lengths(bezier const &Curve)
{
	std::array<double, kSteps + 1> result{};
	auto previous{Curve.p0};
	for (int i = 1; i <= kSteps; ++i)
	{
		auto const next{Curve.point(static_cast<double>(i) / kSteps)};
		result[i] = result[i - 1] + plan_distance(previous, next);
		previous = next;
	}
	return result;
}

double parameter(std::array<double, kSteps + 1> const &Lengths, double const S)
{
	if (S <= 0.0)
		return 0.0;
	if (S >= Lengths.back())
		return 1.0;
	auto const next{std::upper_bound(Lengths.begin(), Lengths.end(), S)};
	auto const index{static_cast<int>(std::distance(Lengths.begin(), next)) - 1};
	auto const run{Lengths[index + 1] - Lengths[index]};
	auto const fraction{run > 1e-12 ? (S - Lengths[index]) / run : 0.0};
	return (index + fraction) / kSteps;
}

double length_at(std::array<double, kSteps + 1> const &Lengths, double const T)
{
	auto const position{std::clamp(T, 0.0, 1.0) * kSteps};
	auto const index{std::min(static_cast<int>(position), kSteps - 1)};
	return Lengths[index] + (Lengths[index + 1] - Lengths[index]) * (position - index);
}

glm::dvec2 plan_direction(bezier const &Curve, double const T)
{
	auto tangent{Curve.first(T)};
	glm::dvec2 result{tangent.x, tangent.z};
	if (glm::length(result) < 1e-9)
		result = {Curve.p3.x - Curve.p0.x, Curve.p3.z - Curve.p0.z};
	return glm::length(result) > 1e-9 ? glm::normalize(result) : glm::dvec2{0.0, 1.0};
}

std::string lower(std::string Text)
{
	std::transform(Text.begin(), Text.end(), Text.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
	return Text;
}

} // namespace

char const *name(kind const Kind)
{
	switch (Kind)
	{
	case kind::model: return "model";
	case kind::include: return "include";
	case kind::traction: return "traction";
	case kind::memcell: return "memcell";
	case kind::launcher: return "eventlauncher";
	}
	return "";
}

char const *name(category const Category)
{
	switch (Category)
	{
	case category::signal: return "Signals";
	case category::sign: return "Signs";
	case category::pole: return "Poles and gantries";
	case category::catenary: return "Catenary";
	case category::logic: return "Logic";
	case category::other: return "Other";
	}
	return "";
}

std::vector<category> const &categories()
{
	static std::vector<category> const result{category::signal, category::sign, category::pole, category::catenary, category::logic, category::other};
	return result;
}

double plan_length(segment_data const &Path)
{
	return lengths(bezier{Path}).back();
}

std::vector<glm::dvec3> outline(segment_data const &Path, double const Step)
{
	bezier const shape{Path};
	// the speed along the curve doesn't exceed three times the longest leg of the control polygon
	auto const leg{std::max({plan_distance(shape.p0, shape.p1), plan_distance(shape.p1, shape.p2), plan_distance(shape.p2, shape.p3)})};
	auto const count{std::clamp(static_cast<int>(std::ceil(3.0 * leg / std::max(Step, 0.1))), 1, 4096)};
	std::vector<glm::dvec3> result;
	result.reserve(count + 1);
	for (int i = 0; i <= count; ++i)
		result.push_back(shape.point(static_cast<double>(i) / count));
	return result;
}

frame frame_at(segment_data const &Path, double const S)
{
	bezier const shape{Path};
	auto const t{parameter(lengths(shape), S)};
	return {shape.point(t), plan_direction(shape, t)};
}

frame frame_at(station const &Station)
{
	if (Station.track == nullptr || Station.path < 0 || Station.path >= static_cast<int>(Station.track->m_paths.size()))
		return {};
	return frame_at(Station.track->m_paths[Station.path], Station.s);
}

double project(segment_data const &Path, glm::dvec3 const &Point, double &S, bool &Interior)
{
	bezier const shape{Path};
	auto const table{lengths(shape)};
	auto const distance = [&](double const T) { return plan_distance(shape.point(T), Point); };
	auto best{0};
	auto bestdistance{distance(0.0)};
	for (int i = 1; i <= kSteps; ++i)
	{
		auto const d{distance(static_cast<double>(i) / kSteps)};
		if (d < bestdistance)
		{
			bestdistance = d;
			best = i;
		}
	}
	// golden section search around the nearest sample
	auto low{std::max(0, best - 1) / static_cast<double>(kSteps)};
	auto high{std::min(kSteps, best + 1) / static_cast<double>(kSteps)};
	double const ratio{0.6180339887498949};
	auto a{high - ratio * (high - low)};
	auto b{low + ratio * (high - low)};
	auto da{distance(a)}, db{distance(b)};
	for (int i = 0; i < 40; ++i)
	{
		if (da < db)
		{
			high = b;
			b = a;
			db = da;
			a = high - ratio * (high - low);
			da = distance(a);
		}
		else
		{
			low = a;
			a = b;
			da = db;
			b = low + ratio * (high - low);
			db = distance(b);
		}
	}
	auto t{(low + high) * 0.5};
	if (distance(0.0) <= distance(t))
		t = 0.0;
	if (distance(1.0) <= distance(t))
		t = 1.0;
	S = length_at(table, t);
	Interior = (t > 1e-6 && t < 1.0 - 1e-6);
	return distance(t);
}

double project(TTrack &Track, glm::dvec3 const &Point, station &Station, bool &Interior)
{
	auto best{-1.0};
	for (int i = 0; i < static_cast<int>(Track.m_paths.size()); ++i)
	{
		double s;
		bool interior;
		auto const d{project(Track.m_paths[i], Point, s, interior)};
		// an interior point takes precedence over an end shared with the other path
		if (best < 0.0 || d < best - 1e-6 || (std::abs(d - best) <= 1e-6 && interior && false == Interior))
		{
			best = d;
			Station = {&Track, i, s};
			Interior = interior;
		}
	}
	return best;
}

double heading(glm::dvec2 const &Forward)
{
	return glm::degrees(std::atan2(Forward.x, Forward.y));
}

anchor make_anchor(station const &At, glm::dvec3 const &Point)
{
	anchor result;
	result.at = At;
	auto const axis{frame_at(At)};
	glm::dvec2 const side{axis.forward.y, -axis.forward.x};
	result.offset = glm::dot(plan_of(Point - axis.point), side);
	result.height = Point.y - axis.point.y;
	result.foot = axis.point;
	if (At.track != nullptr && At.path >= 0 && At.path < static_cast<int>(At.track->m_paths.size()))
	{
		auto const &path{At.track->m_paths[At.path]};
		result.start = path.points[segment_data::point::start];
		result.end = path.points[segment_data::point::end];
		result.length = plan_length(path);
	}
	return result;
}

glm::dvec3 place(anchor &Anchor)
{
	auto const axis{frame_at(Anchor.at)};
	Anchor.foot = axis.point;
	glm::dvec2 const side{axis.forward.y, -axis.forward.x};
	return {axis.point.x + side.x * Anchor.offset, axis.point.y + Anchor.height, axis.point.z + side.y * Anchor.offset};
}

std::vector<rule> default_rules()
{
	return {
	    {category::signal, "semafor sem_ sem- tarcza tarcz_ tz_ tm_ tzw powtarz sygnaliz sygnal signal"},
	    {category::sign, "wskaz wsk_ wsk- w4 w8 w11 w24 w27 kilometr hektometr slupek_km znak tablic sign"},
	    {category::catenary, "trakcj siec_ siec- catenary przewod lina_nosna"},
	    {category::pole, "slup maszt wysieg bramk konstrukcj kotw odciag pole gantry"},
	};
}

category classify(std::string const &Name, std::vector<rule> const &Rules)
{
	auto const text{lower(Name)};
	for (auto const &rule : Rules)
	{
		std::istringstream words{lower(rule.keywords)};
		std::string word;
		while (words >> word)
			if (text.find(word) != std::string::npos)
				return rule.group;
	}
	return category::other;
}

} // namespace infra
