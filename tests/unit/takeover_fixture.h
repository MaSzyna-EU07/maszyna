#pragma once

// old tracks made of a line of the domain, for the tests of their takeover (test_takeover_*.cpp): the line cut at random places, each
// piece one Bezier curve between its ends with the handles of an arc, the joints and the heights shaken a little and the rolls rounded to
// the tenth of a degree, as old sceneries have them

#include "doctest.h"
#include "editor/editorGeometry.hpp"
#include "editor/layout/layoutLine.hpp"
#include "editor/layout/layoutRecognition.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

namespace test::takeover_fixture {

struct noise {
	double plan{0.0}; // m, the joints moved across the line, evenly within this either way
	double height{0.0}; // m, up or down
	bool round_rolls{true}; // to the tenth of a degree
};

struct old_line {
	layout::solved_line line;
	std::vector<layout::chain_path> chain; // as run along the line, some of the paths written the other way round
	std::vector<double> cuts; // the chainages of the joints, the ends included
};

inline layout::vertex vertex_at(glm::dvec2 const &Point, double const Radius, double const In, double const Out, double const Cant) {
	layout::vertex result;
	result.point = Point;
	result.radius = Radius;
	result.transition_in = In;
	result.transition_out = Out;
	result.cant = Cant;
	return result;
}

// the line laid between the ends, its vertices numbered from 1
inline layout::solved_line laid(layout::line_shape Shape, glm::dvec3 const &Start, glm::dvec3 const &End) {
	for (std::size_t k = 0; k < Shape.vertices.size(); ++k) {
		Shape.vertices[k].id = static_cast<layout::vertex_id>(k + 1);
	}
	layout::end_condition start;
	start.position = Start;
	layout::end_condition end;
	end.position = End;
	return layout::solve(Shape, start, end);
}

// the path of the old track from one chainage of the line to another: its ends and the handles of an arc of the turn between their headings
inline segment_data path_between(layout::solved_line const &Line, glm::dvec3 const &From, glm::dvec3 const &To, double const A, double const B, bool const Round) {
	segment_data path;
	path.points[segment_data::point::start] = From;
	path.points[segment_data::point::end] = To;
	auto const t0{glm::normalize(Line.tangent(A))};
	auto const t1{glm::normalize(Line.tangent(B))};
	auto const turn{geometry::signed_angle(t0, t1)};
	auto const chord{geometry::plan_distance(From, To)};
	auto const cosine{std::cos(turn / 4.0)};
	auto const handle{chord / (3.0 * cosine * cosine)};
	if (std::abs(turn) > 1e-9 || std::abs(Line.grade(A) - Line.grade(B)) > 1e-9) {
		path.points[segment_data::point::control1] = glm::dvec3{t0.x * handle, Line.grade(A) * handle, t0.y * handle};
		path.points[segment_data::point::control2] = glm::dvec3{-t1.x * handle, -Line.grade(B) * handle, -t1.y * handle};
	}
	auto const roll = [&](double const S) {
		auto const value{layout::roll_of(Line.cant_at(S))};
		return static_cast<float>(Round ? std::round(value * 10.0) / 10.0 : value);
	};
	path.rolls = {roll(A), roll(B)};
	path.radius = 0.f;
	return path;
}

// the line cut at random chainages Shortest to Longest apart, the seed given; the joints shaken by the noise
inline old_line cut(layout::solved_line const &Line, unsigned const Seed, double const Shortest, double const Longest, noise const &Noise) {
	old_line result;
	result.line = Line;
	std::mt19937 random{Seed};
	std::uniform_real_distribution<double> length{Shortest, Longest};
	std::uniform_real_distribution<double> shake{-1.0, 1.0};
	std::bernoulli_distribution backwards{0.3};
	result.cuts.push_back(0.0);
	while (result.cuts.back() + Longest < Line.length) {
		result.cuts.push_back(result.cuts.back() + length(random));
	}
	if (Line.length - result.cuts.back() < Shortest * 0.5 && result.cuts.size() > 1) {
		result.cuts.pop_back();
	}
	result.cuts.push_back(Line.length);
	std::vector<glm::dvec3> joints;
	for (std::size_t i = 0; i < result.cuts.size(); ++i) {
		auto const s{result.cuts[i]};
		auto point{Line.point(s)};
		auto const across{layout::left_of(glm::normalize(Line.tangent(s)))};
		auto const inner{i > 0 && i + 1 < result.cuts.size()};
		auto const side{inner ? shake(random) * Noise.plan : 0.0};
		auto const up{inner ? shake(random) * Noise.height : 0.0};
		point += glm::dvec3{across.x * side, up, across.y * side};
		joints.push_back(point);
	}
	for (std::size_t i = 0; i + 1 < result.cuts.size(); ++i) {
		auto const path{path_between(Line, joints[i], joints[i + 1], result.cuts[i], result.cuts[i + 1], Noise.round_rolls)};
		if (backwards(random)) {
			result.chain.push_back({geometry::reversed(path), false});
		}
		else {
			result.chain.push_back({path, true});
		}
	}
	return result;
}

// the chain run the other way
inline std::vector<layout::chain_path> turned_round(std::vector<layout::chain_path> Chain) {
	std::ranges::reverse(Chain);
	for (auto &part : Chain) {
		part.forward = false == part.forward;
	}
	return Chain;
}

// the vertex of the recognized line nearest the point, none farther than Within
inline layout::vertex const *nearest(layout::line_shape const &Shape, glm::dvec2 const &Point, double const Within) {
	layout::vertex const *result{nullptr};
	double best{Within};
	for (auto const &vertex : Shape.vertices) {
		if (auto const distance{glm::distance(vertex.point, Point)}; distance < best) {
			best = distance;
			result = &vertex;
		}
	}
	return result;
}

} // namespace test::takeover_fixture
