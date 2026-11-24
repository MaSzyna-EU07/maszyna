/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/bones.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iterator>
#include <ranges>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/euler_angles.hpp>

namespace bones {

namespace {

bool read_number(std::string const &Word, double &Number)
{
	char *end{nullptr};
	Number = std::strtod(Word.c_str(), &end);
	return end != Word.c_str() && *end == '\0';
}

// the three numbers of the words from the First
bool read_vector(std::vector<std::string> const &Words, std::size_t const First, glm::dvec3 &Vector)
{
	if (Words.size() < First + 3)
	{
		return false;
	}
	return read_number(Words[First], Vector.x) && read_number(Words[First + 1], Vector.y) && read_number(Words[First + 2], Vector.z);
}

std::string read_crank(rig const &, point &Point, std::vector<std::string> const &Words)
{
	Point.kind = point_kind::crank;
	Point.wheel = Words[2];
	return read_vector(Words, 3, Point.rest) ? "" : "crank: the position isn't three numbers";
}

std::string read_slide(rig const &Rig, point &Point, std::vector<std::string> const &Words)
{
	Point.kind = point_kind::slide;
	Point.driver = Rig.find(Words[2]);
	if (Point.driver < 0)
	{
		return "slide: no point " + Words[2] + " defined before it";
	}
	if (Rig.points[Point.driver].kind == point_kind::node)
	{
		return "slide: a node of a chain can't drive it";
	}
	if (false == read_vector(Words, 3, Point.rest) || false == read_vector(Words, 6, Point.guide))
	{
		return "slide: the position or the guide isn't three numbers";
	}
	if (glm::length(Point.guide) < 1e-6)
	{
		return "slide: the guide has no direction";
	}
	Point.guide = glm::normalize(Point.guide);
	return "";
}

std::string read_coupler(rig const &, point &Point, std::vector<std::string> const &Words)
{
	Point.kind = point_kind::coupler;
	if (Words[2] == "front")
	{
		Point.end = 0;
	}
	else if (Words[2] == "rear")
	{
		Point.end = 1;
	}
	else
	{
		return "coupler: the end is front or rear, not " + Words[2];
	}
	return read_vector(Words, 3, Point.rest) ? "" : "coupler: the position isn't three numbers";
}

std::string read_middle(rig const &Rig, point &Point, std::vector<std::string> const &Words)
{
	Point.kind = point_kind::middle;
	Point.driver = Rig.find(Words[2]);
	Point.second = Rig.find(Words[3]);
	if (Point.driver < 0 || Point.second < 0)
	{
		return "mid: the points " + Words[2] + " and " + Words[3] + " are to be defined before it";
	}
	if (Rig.points[Point.driver].kind == point_kind::node || Rig.points[Point.second].kind == point_kind::node)
	{
		return "mid: not between the nodes of a chain";
	}
	Point.rest = 0.5 * (Rig.points[Point.driver].rest + Rig.points[Point.second].rest);
	return "";
}

std::string read_fixed(rig const &, point &Point, std::vector<std::string> const &Words)
{
	Point.kind = point_kind::fixed;
	return read_vector(Words, 2, Point.rest) ? "" : "fixed: the position isn't three numbers";
}

} // namespace

int rig::find(std::string_view const Name) const
{
	auto const found{std::ranges::find(points, Name, &point::name)};
	return found == points.end() ? -1 : static_cast<int>(found - points.begin());
}

int point_words(std::string_view const Kind)
{
	if (Kind == "fixed")
	{
		return 3;
	}
	if (Kind == "crank" || Kind == "coupler")
	{
		return 4;
	}
	if (Kind == "mid")
	{
		return 2;
	}
	if (Kind == "slide")
	{
		return 7;
	}
	return 0;
}

std::string add_point(rig &Rig, std::vector<std::string> const &Words)
{
	if (Words.size() < 2)
	{
		return "a point needs a name and a kind";
	}
	auto const count{point_words(Words[1])};
	if (count == 0)
	{
		return "unknown kind of point: " + Words[1];
	}
	if (Words.size() < static_cast<std::size_t>(count) + 2)
	{
		return "too few words for a point of the kind " + Words[1];
	}
	if (Rig.find(Words[0]) >= 0)
	{
		return "the point " + Words[0] + " is defined twice";
	}
	point point;
	point.name = Words[0];
	std::string error;
	if (Words[1] == "fixed")
	{
		error = read_fixed(Rig, point, Words);
	}
	else if (Words[1] == "crank")
	{
		error = read_crank(Rig, point, Words);
	}
	else if (Words[1] == "slide")
	{
		error = read_slide(Rig, point, Words);
	}
	else if (Words[1] == "mid")
	{
		error = read_middle(Rig, point, Words);
	}
	else
	{
		error = read_coupler(Rig, point, Words);
	}
	if (error.empty())
	{
		Rig.points.push_back(std::move(point));
	}
	return error;
}

std::string add_bone(rig &Rig, std::vector<std::string> const &Words)
{
	if (Words.size() < 3)
	{
		return "a bone needs its submodel, its anchor and the point it faces (or none)";
	}
	bone bone;
	bone.submodel = Words[0];
	bone.anchor = Rig.find(Words[1]);
	if (bone.anchor < 0)
	{
		return "no point " + Words[1] + " for the anchor of the bone " + Words[0];
	}
	if (Words[2] != "none")
	{
		bone.aim = Rig.find(Words[2]);
		if (bone.aim < 0)
		{
			return "no point " + Words[2] + " for the bone " + Words[0] + " to face";
		}
	}
	Rig.bones.push_back(std::move(bone));
	return "";
}

std::string add_chain(rig &Rig, std::vector<std::string> const &Words)
{
	if (Words.size() < 4)
	{
		return "a chain needs its name, its two points and the count of its nodes";
	}
	chain chain;
	chain.name = Words[0];
	chain.start = Rig.find(Words[1]);
	chain.end = Rig.find(Words[2]);
	if (chain.start < 0 || chain.end < 0)
	{
		return "the points " + Words[1] + " and " + Words[2] + " of the chain " + Words[0] + " are to be defined before it";
	}
	auto count{0.0};
	if (false == read_number(Words[3], count) || count < 1.0 || count > 32.0)
	{
		return "the chain " + Words[0] + " needs 1 to 32 nodes";
	}
	auto const nodes{static_cast<std::size_t>(count)};
	std::vector<glm::dvec3> positions{Rig.points[chain.start].rest};
	for (std::size_t node{0}; node < nodes; ++node)
	{
		glm::dvec3 position;
		if (false == read_vector(Words, 4 + 3 * node, position))
		{
			return "the chain " + Words[0] + " has no three numbers for its node " + std::to_string(node + 1);
		}
		positions.push_back(position);
	}
	positions.push_back(Rig.points[chain.end].rest);
	for (std::size_t piece{0}; piece + 1 < positions.size(); ++piece)
	{
		chain.lengths.push_back(glm::length(positions[piece + 1] - positions[piece]));
	}
	for (std::size_t node{0}; node < nodes; ++node)
	{
		auto const name{Words[0] + "." + std::to_string(node + 1)};
		if (Rig.find(name) >= 0)
		{
			return "the point " + name + " is defined twice";
		}
		point point;
		point.name = name;
		point.kind = point_kind::node;
		point.rest = positions[node + 1];
		chain.nodes.push_back(static_cast<int>(Rig.points.size()));
		Rig.points.push_back(std::move(point));
	}
	Rig.chains.push_back(std::move(chain));
	return "";
}

void start_chain(chain_state &State, std::vector<glm::dvec3> const &Nodes, glm::dvec3 const &Start, glm::dvec3 const &End)
{
	State.now = Nodes;
	State.before = Nodes;
	State.start = Start;
	State.end = End;
	State.started = true;
}

namespace {

// the nodes with the ends pinned where they are, each piece of the chain brought back to its length: the points of a
// piece move apart or together, an end pinned taking none of it
void keep_lengths(std::vector<double> const &Lengths, std::vector<glm::dvec3> &Nodes, glm::dvec3 const &Start, glm::dvec3 const &End)
{
	std::vector<glm::dvec3> points{Start};
	points.insert(points.end(), Nodes.begin(), Nodes.end());
	points.push_back(End);
	auto const last{points.size() - 1};
	for (std::size_t piece{0}; piece < Lengths.size() && piece < last; ++piece)
	{
		auto const along{points[piece + 1] - points[piece]};
		auto const length{glm::length(along)};
		if (length < 1e-9)
		{
			continue;
		}
		auto const correction{along * ((length - Lengths[piece]) / length)};
		auto const firstpinned{piece == 0};
		auto const secondpinned{piece + 1 == last};
		if (firstpinned && secondpinned)
		{
			continue;
		}
		if (firstpinned)
		{
			points[piece + 1] -= correction;
		}
		else if (secondpinned)
		{
			points[piece] += correction;
		}
		else
		{
			points[piece] += 0.5 * correction;
			points[piece + 1] -= 0.5 * correction;
		}
	}
	std::ranges::copy(std::ranges::subrange(points.begin() + 1, points.end() - 1), Nodes.begin());
}

} // namespace

void simulate(std::vector<double> const &Lengths, chain_state &State, glm::dvec3 const &Start, glm::dvec3 const &End, double const Deltatime,
              chain_physics const &Physics)
{
	if (Deltatime <= 0.0 || false == State.started)
	{
		return;
	}
	auto const steps{std::clamp(static_cast<int>(std::ceil(Deltatime / Physics.step)), 1, 64)};
	auto const step{Deltatime / steps};
	auto const keep{std::exp(-Physics.damping * step)};
	auto const fromstart{State.start};
	auto const fromend{State.end};
	for (auto substep{1}; substep <= steps; ++substep)
	{
		auto const along{static_cast<double>(substep) / steps};
		auto const start{glm::mix(fromstart, Start, along)};
		auto const end{glm::mix(fromend, End, along)};
		for (std::size_t node{0}; node < State.now.size(); ++node)
		{
			auto const velocity{(State.now[node] - State.before[node]) * keep};
			State.before[node] = State.now[node];
			State.now[node] += velocity + Physics.gravity * (step * step);
		}
		for (auto iteration{0}; iteration < Physics.iterations; ++iteration)
		{
			keep_lengths(Lengths, State.now, start, end);
		}
	}
	State.start = Start;
	State.end = End;
}

glm::dvec3 crank(glm::dvec3 const &Rest, wheel_turn const &Wheel)
{
	auto const turn{glm::angleAxis(glm::radians(Wheel.angle), glm::normalize(Wheel.axis))};
	return Wheel.centre + turn * (Rest - Wheel.centre);
}

glm::dvec3 slide(glm::dvec3 const &Rest, glm::dvec3 const &Guide, glm::dvec3 const &DriverRest, glm::dvec3 const &DriverNow)
{
	// |Rest + s Guide - DriverNow| = |Rest - DriverRest|: s² + 2 b s + c = 0
	auto const length2{glm::dot(Rest - DriverRest, Rest - DriverRest)};
	auto const offset{Rest - DriverNow};
	auto const b{glm::dot(Guide, offset)};
	auto const c{glm::dot(offset, offset) - length2};
	auto const discriminant{b * b - c};
	if (discriminant < 0.0)
	{
		// out of reach of the rod: as close as the guide comes
		return Rest - b * Guide;
	}
	// the root nearer the position as made
	auto const root{std::sqrt(discriminant)};
	auto const shift{std::abs(-b + root) < std::abs(-b - root) ? -b + root : -b - root};
	return Rest + shift * Guide;
}

std::vector<glm::dvec3> pose(rig const &Rig, drive const &Drive)
{
	std::vector<glm::dvec3> positions;
	positions.reserve(Rig.points.size());
	for (std::size_t index{0}; index < Rig.points.size(); ++index)
	{
		auto const &point{Rig.points[index]};
		auto position{point.rest};
		if (point.kind == point_kind::crank && index < Drive.wheels.size())
		{
			position = crank(point.rest, Drive.wheels[index]);
		}
		else if (point.kind == point_kind::slide)
		{
			auto const &driver{Rig.points[point.driver]};
			position = slide(point.rest, point.guide, driver.rest, positions[point.driver]);
		}
		else if (point.kind == point_kind::coupler && index < Drive.couplers.size() && Drive.couplers[index])
		{
			position = point.rest + *Drive.couplers[index];
		}
		else if (point.kind == point_kind::middle)
		{
			position = 0.5 * (positions[point.driver] + positions[point.second]);
		}
		positions.push_back(position);
	}
	return positions;
}

motion bone_motion(glm::dvec3 const &AnchorRest, glm::dvec3 const &AnchorNow, std::optional<glm::dvec3> const &AimRest, std::optional<glm::dvec3> const &AimNow)
{
	motion motion{glm::dmat3{1.0}, AnchorRest, AnchorNow};
	if (false == AimRest.has_value() || false == AimNow.has_value())
	{
		return motion;
	}
	auto const from{*AimRest - AnchorRest};
	auto const to{*AimNow - AnchorNow};
	if (glm::length(from) < 1e-6 || glm::length(to) < 1e-6)
	{
		return motion;
	}
	motion.rotation = glm::mat3_cast(glm::rotation(glm::normalize(from), glm::normalize(to)));
	return motion;
}

animation localise(motion const &Motion, glm::dvec3 const &Origin, glm::dmat3 const &Rotation)
{
	// a point of the submodel x = Origin + Rotation p goes to AnchorNow + R (x - AnchorRest), which is
	// Origin + Rotation (t + Q p) with Q = Rotation^T R Rotation and t = Rotation^T (AnchorNow + R (Origin - AnchorRest) - Origin)
	auto const inverse{glm::transpose(Rotation)};
	auto const shift{Motion.anchor_now + Motion.rotation * (Origin - Motion.anchor_rest) - Origin};
	auto const local{inverse * Motion.rotation * Rotation};
	animation result;
	result.translation = glm::vec3(inverse * shift);
	auto x{0.0};
	auto y{0.0};
	auto z{0.0};
	glm::extractEulerAngleXYZ(glm::dmat4(local), x, y, z);
	result.angles = glm::vec3(glm::degrees(x), glm::degrees(y), glm::degrees(z));
	result.matrix = glm::translate(glm::mat4(1.f), result.translation) * glm::mat4(glm::mat3(local));
	return result;
}

glm::dmat3 model_file_frame()
{
	return glm::dmat3{{-1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {0.0, 1.0, 0.0}};
}

} // namespace bones
