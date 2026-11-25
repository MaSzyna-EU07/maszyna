/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "world/CrossingPanels.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace crossing_panels
{

char const *const texture{"przejazdy/cbp"};

double inner_width(int const Gauge)
{
	return Gauge >= 1500 ? 1.380 : 1.300;
}

double edge_drop(int const Height)
{
	return Height >= 160 ? 0.110 : 0.070;
}

namespace
{

// what a face of the cross-section shows, for the texture: the concrete of the top, the steel of an angle, the rest of the concrete
enum class face
{
	top,
	angle,
	side
};
// a corner of the cross-section, with the face from it to the next corner. The corners go round the way the top is crossed to +x first
struct corner
{
	double x;
	double y;
	face next;
};

// an inner panel with its top Half either way of its axis, its bottom as far as it reaches under the slopes of its sides
std::vector<corner> inner_section(double const Half, double const Bottom, int const Height)
{
	auto const h{Height / 1000.0};
	auto const drop{edge_drop(Height)};
	auto const low{std::min(Bottom, Half)};
	return {
	    {-Half, 0.0, face::angle},
	    {-Half + angle_leg, 0.0, face::top},
	    {Half - angle_leg, 0.0, face::angle},
	    {Half, 0.0, face::angle},
	    {Half, -angle_leg, face::side},
	    {Half, -drop, face::side},
	    {low, -h, face::side},
	    {-low, -h, face::side},
	    {-Half, -drop, face::side},
	    {-Half, -angle_leg, face::angle},
	};
}

// an outer panel: its edge by the rail at 0 with the angle, the slope under it, its outer side straight down
std::vector<corner> outer_section(int const Height)
{
	auto const h{Height / 1000.0};
	auto const drop{edge_drop(Height)};
	return {
	    {0.0, 0.0, face::angle},
	    {angle_leg, 0.0, face::top},
	    {outer_width, 0.0, face::side},
	    {outer_width, -h, face::side},
	    {edge_slope, -h, face::side},
	    {0.0, -drop, face::side},
	    {0.0, -angle_leg, face::angle},
	};
}

// the texture of the game laid as its models of CBP have it: the top concrete over the upper 86% of it, the mounting holes 450 mm
// from the ends of a panel, the steel at 0.93-0.98
glm::vec2 uv(face const Face, double const Along, double const Across, double const Middle)
{
	auto const u{static_cast<float>(Along / length)};
	switch (Face)
	{
	case face::top:
		return {u, static_cast<float>(0.433 + 0.852 * (Across - Middle) / 1.16)};
	case face::angle:
		return {u, static_cast<float>(0.93 + 0.05 * std::clamp(Across, 0.0, 1.0))};
	default:
		return {u, static_cast<float>(0.1 + 0.6 * std::clamp(Across, 0.0, 1.0))};
	}
}

void triangle(std::vector<world_vertex> &Out, world_vertex A, world_vertex B, world_vertex C, glm::dvec3 const &Outward)
{
	auto normal{glm::cross(B.position - A.position, C.position - A.position)};
	if (glm::length(normal) < 1e-12)
		return;
	if (glm::dot(normal, Outward) < 0.0)
	{
		std::swap(B, C);
		normal = -normal;
	}
	auto const facing{glm::vec3(glm::normalize(normal))};
	A.normal = B.normal = C.normal = facing;
	Out.push_back(A);
	Out.push_back(B);
	Out.push_back(C);
}

// a panel from Start along z: its cross-section at each station along it, Halves the half-width of its top there (an inner one); the
// top of its ends set back by the joint
void panel(std::vector<world_vertex> &Out, double const Start, std::vector<std::pair<double, double>> const &Stations, bool const Outer, int const Height, int const Gauge, double const Slope = 0.0)
{
	auto const bottom{0.5 * inner_width(Gauge) - edge_slope};
	auto const middle{Outer ? 0.5 * (angle_leg + outer_width) : 0.0};
	std::vector<std::vector<corner>> sections;
	for (auto const &[along, half] : Stations)
		sections.push_back(Outer ? outer_section(Height) : inner_section(half, bottom, Height));
	auto const place = [&](std::size_t const Station, corner const &Corner) {
		auto z{Start + Stations[Station].first};
		// the top and the angle by it set back at the ends, for the joint
		if (Corner.y > -angle_leg - 1e-9)
		{
			if (Station == 0)
				z += joint;
			else if (Station + 1 == Stations.size())
				z -= joint;
		}
		// an outer one bent: its top and all of it rising away from the rail, its sides still upright
		return glm::dvec3{Corner.x, Corner.y + Slope * (Corner.x + outer_gap), z};
	};
	// the faces along the panel
	for (std::size_t k = 0; k + 1 < Stations.size(); ++k)
	{
		auto const &here{sections[k]};
		auto const &there{sections[k + 1]};
		for (std::size_t i = 0; i < here.size(); ++i)
		{
			auto const j{(i + 1) % here.size()};
			auto const a{place(k, here[i])}, b{place(k, here[j])}, c{place(k + 1, there[j])}, d{place(k + 1, there[i])};
			// out of the cross-section, which goes round clockwise seen along +z
			glm::dvec2 const edge{here[j].x - here[i].x, here[j].y - here[i].y};
			glm::dvec3 const outward{-edge.y, edge.x, 0.0};
			auto const kind{here[i].next};
			auto const vertex = [&](glm::dvec3 const &Position, double const Share) {
				auto const across{kind == face::top ? Position.x : Share};
				return world_vertex{Position, glm::vec3{0.f, 1.f, 0.f}, uv(kind, Position.z - Start, across, middle)};
			};
			triangle(Out, vertex(a, 0.0), vertex(b, 1.0), vertex(c, 1.0), outward);
			triangle(Out, vertex(a, 0.0), vertex(c, 1.0), vertex(d, 0.0), outward);
		}
	}
	// the ends
	for (auto const station : {std::size_t{0}, Stations.size() - 1})
	{
		auto const &section{sections[station]};
		glm::dvec3 centre{0.0};
		for (auto const &entry : section)
			centre += place(station, entry);
		centre /= static_cast<double>(section.size());
		glm::dvec3 const outward{0.0, 0.0, station == 0 ? -1.0 : 1.0};
		auto const vertex = [&](glm::dvec3 const &Position) {
			return world_vertex{Position, glm::vec3{0.f, 0.f, 1.f}, glm::vec2{static_cast<float>(0.5 + 0.3 * Position.x), static_cast<float>(0.1 - 2.0 * Position.y)}};
		};
		for (std::size_t i = 0; i < section.size(); ++i)
			triangle(Out, vertex(centre), vertex(place(station, section[i])), vertex(place(station, section[(i + 1) % section.size()])), outward);
	}
}

// the stations of an inner panel: its corners cut at its start, at its end, or neither
std::vector<std::pair<double, double>> inner_stations(int const Gauge, bool const Cutstart, bool const Cutend)
{
	auto const half{0.5 * inner_width(Gauge)};
	std::vector<std::pair<double, double>> result;
	result.emplace_back(0.0, Cutstart ? half - corner_across : half);
	if (Cutstart)
		result.emplace_back(corner_along, half);
	if (Cutend)
		result.emplace_back(length - corner_along, half);
	result.emplace_back(length, Cutend ? half - corner_across : half);
	return result;
}

} // namespace

bool parse(std::string const &Name, spec &Spec)
{
	std::string const prefix{"@cbp/"};
	if (Name.size() <= prefix.size() || Name.compare(0, prefix.size(), prefix) != 0)
		return false;
	auto rest{Name.substr(prefix.size())};
	spec result;
	struct kind_name
	{
		char const *name;
		piece kind;
	};
	// the longer names first: pws before pw
	static kind_name const kinds[]{{"pws", piece::pws}, {"pw", piece::pw}, {"pz", piece::pz}, {"in", piece::inner_row}, {"out", piece::outer_row}};
	bool found{false};
	for (auto const &entry : kinds)
	{
		std::string const word{entry.name};
		if (rest.compare(0, word.size(), word) == 0)
		{
			result.kind = entry.kind;
			rest = rest.substr(word.size());
			found = true;
			break;
		}
	}
	if (false == found)
		return false;
	// <height>-<gauge>[x<count>][s<slope>]
	auto const dash{rest.find('-')};
	if (dash == std::string::npos)
		return false;
	auto const bent{rest.find('s', dash)};
	if (bent != std::string::npos)
	{
		try
		{
			result.slope = std::stoi(rest.substr(bent + 1)) / 10000.0;
		}
		catch (...)
		{
			return false;
		}
		rest = rest.substr(0, bent);
	}
	auto const times{rest.find('x', dash)};
	try
	{
		result.height = std::stoi(rest.substr(0, dash));
		result.gauge = std::stoi(rest.substr(dash + 1, times == std::string::npos ? std::string::npos : times - dash - 1));
		if (times != std::string::npos)
			result.count = std::stoi(rest.substr(times + 1));
	}
	catch (...)
	{
		return false;
	}
	if ((result.height != 140 && result.height != 180) || (result.gauge != 1435 && result.gauge != 1520))
		return false;
	auto const row{result.kind == piece::inner_row || result.kind == piece::outer_row};
	result.count = row ? std::clamp(result.count, result.kind == piece::inner_row ? 2 : 1, most) : 1;
	Spec = result;
	return true;
}

std::string name(spec const &Spec)
{
	std::string kind;
	switch (Spec.kind)
	{
	case piece::pw:
		kind = "pw";
		break;
	case piece::pws:
		kind = "pws";
		break;
	case piece::pz:
		kind = "pz";
		break;
	case piece::inner_row:
		kind = "in";
		break;
	case piece::outer_row:
		kind = "out";
		break;
	}
	auto result{"@cbp/" + kind + std::to_string(Spec.height) + "-" + std::to_string(Spec.gauge)};
	if (Spec.kind == piece::inner_row || Spec.kind == piece::outer_row)
		result += "x" + std::to_string(Spec.count);
	if ((Spec.kind == piece::pz || Spec.kind == piece::outer_row) && std::lround(Spec.slope * 10000.0) != 0)
		result += "s" + std::to_string(std::lround(Spec.slope * 10000.0));
	return result;
}

int height_for(double const Rail)
{
	return Rail >= 0.160 ? 180 : 140;
}

double piece_length(spec const &Spec)
{
	return (Spec.kind == piece::inner_row || Spec.kind == piece::outer_row ? Spec.count : 1) * length;
}

std::vector<world_vertex> geometry(spec const &Spec)
{
	std::vector<world_vertex> result;
	switch (Spec.kind)
	{
	case piece::pw:
		panel(result, 0.0, inner_stations(Spec.gauge, false, false), false, Spec.height, Spec.gauge);
		break;
	case piece::pws:
		panel(result, 0.0, inner_stations(Spec.gauge, false, true), false, Spec.height, Spec.gauge);
		break;
	case piece::pz:
		panel(result, 0.0, {{0.0, 0.0}, {length, 0.0}}, true, Spec.height, Spec.gauge, Spec.slope);
		break;
	case piece::inner_row:
	{
		auto const count{std::max(2, Spec.count)};
		for (int k = 0; k < count; ++k)
			panel(result, k * length, inner_stations(Spec.gauge, k == 0, k + 1 == count), false, Spec.height, Spec.gauge);
		break;
	}
	case piece::outer_row:
		for (int k = 0; k < std::max(1, Spec.count); ++k)
			panel(result, k * length, {{0.0, 0.0}, {length, 0.0}}, true, Spec.height, Spec.gauge, Spec.slope);
		break;
	}
	return result;
}

} // namespace crossing_panels
