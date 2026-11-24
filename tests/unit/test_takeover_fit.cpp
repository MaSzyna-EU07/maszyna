// The recognition of old tracks (editor-tracks-domain.md §10, editor-tracks-tests.md §4 "E6", U): lines of the domain cut at random places
// into old tracks, one Bezier curve each, the joints shaken; the curvature sampled, the curves found and fitted by least squares, the
// radii and the transitions read back within a tolerance, the deviation of the line from the tracks measured.

#include "stdafx.h"
#include "doctest.h"
#include "takeover_fixture.h"
#include "editor/layout/layoutLegacy.hpp"

#include <format>
#include <numeric>
#include <random>

namespace {

using test::takeover_fixture::cut;
using test::takeover_fixture::laid;
using test::takeover_fixture::nearest;
using test::takeover_fixture::noise;
using test::takeover_fixture::turned_round;
using test::takeover_fixture::vertex_at;

// three curves: R 800 m with 80 m transitions and 60 mm, R 600 m with 60 and 70 m and 90 mm, R 1500 m with no transitions nor cant
layout::line_shape three_curves() {
	layout::line_shape shape;
	shape.vertices = {vertex_at({600.0, 0.0}, 800.0, 80.0, 80.0, 60.0), vertex_at({1100.0, 300.0}, 600.0, 60.0, 70.0, 90.0), vertex_at({1700.0, 200.0}, 1500.0, 0.0, 0.0, 0.0)};
	shape.grades = {{900.0, 104.0, 10000.0}, {1600.0, 101.0, 8000.0}};
	return shape;
}

glm::dvec3 const kStart{0.0, 100.0, 0.0};
glm::dvec3 const kEnd{2300.0, 102.0, 200.0};

std::string described(layout::recognized_line const &Line) {
	std::string text;
	for (auto const &v : Line.shape.vertices) {
		text += std::format("[{:.2f} {:.2f} R {:.2f} in {:.2f} out {:.2f} D {:.1f}] ", v.point.x, v.point.y, v.radius, v.transition_in, v.transition_out, v.cant);
	}
	text += std::format("plan {:.4f} at {:.1f} {:.1f} rms {:.4f} height {:.4f} cant {:.2f}", Line.measured.plan, Line.measured.plan_at.x, Line.measured.plan_at.z, Line.measured.plan_rms, Line.measured.height, Line.measured.cant);
	for (auto const &finding : Line.findings) {
		text += "; " + finding;
	}
	return text;
}

// a transition found within 3 m of the one laid, or moving the arc by no more than 5 mm from where that one moves it: the shift of a
// transition, L^2 / 24R, tells a short one apart from none by too little to be read through the noise of the joints, and a Bezier curve
// of an old track across the start of an arc eases the curvature in over its length
bool same_transition(double const Found, double const Laid, double const Radius) {
	return std::abs(Found - Laid) < 3.0 || std::abs(Found * Found - Laid * Laid) / (24.0 * Radius) < 0.005;
}

// each vertex of the original line found again: its radius within a percent, its transitions as same_transition() has it, its cant
// within 3 mm
void check_found(layout::line_shape const &Original, layout::recognized_line const &Found, bool const Turned) {
	REQUIRE(Found.valid());
	CHECK(Found.shape.vertices.size() == Original.vertices.size());
	for (auto const &vertex : Original.vertices) {
		CAPTURE(vertex.radius);
		auto const *match{nearest(Found.shape, vertex.point, 5.0)};
		REQUIRE(match != nullptr);
		CHECK(std::abs(match->radius - vertex.radius) < vertex.radius * 0.01); // NOSONAR: REQUIRE above ends the test on none
		CHECK(same_transition(Turned ? match->transition_out : match->transition_in, vertex.transition_in, vertex.radius));
		CHECK(same_transition(Turned ? match->transition_in : match->transition_out, vertex.transition_out, vertex.radius));
		CHECK(std::abs(match->cant - vertex.cant) < 3.0);
	}
}

} // namespace

TEST_CASE("takeover fit: three curves cut at random places, seeds 1101, 2202, 3303") {
	auto const shape{three_curves()};
	auto const line{laid(shape, kStart, kEnd)};
	REQUIRE(line.valid());
	for (unsigned const seed : {1101u, 2202u, 3303u}) {
		CAPTURE(seed);
		auto const old{cut(line, seed, 8.0, 25.0, noise{0.004, 0.004, true})};
		auto const found{layout::recognize(old.chain)};
		CAPTURE(described(found));
		check_found(shape, found, false);
		CHECK(found.measured.within(layout::takeover_tolerance{}));
		CHECK(found.joints.size() == old.cuts.size());
		// the walk the other way round: the same curves, the transitions the other way
		auto const back{layout::recognize(turned_round(old.chain))};
		CAPTURE(described(back));
		check_found(shape, back, true);
		CHECK(back.measured.within(layout::takeover_tolerance{}));
	}
}

TEST_CASE("takeover fit: the same line whichever old track the walk starts from, seeds 4401, 5502, 6603") {
	auto const shape{three_curves()};
	auto const old{cut(laid(shape, kStart, kEnd), 7707u, 8.0, 25.0, noise{0.003, 0.003, true})};
	// the old tracks as a scenery lists them, each written the way it was
	std::vector<layout::legacy_track> tracks;
	for (std::size_t i = 0; i < old.chain.size(); ++i) {
		layout::legacy_track track;
		track.name = std::format("t{}", i);
		track.paths = {old.chain[i].path};
		track.attributes = "velocity 80";
		tracks.push_back(std::move(track));
	}
	auto const take_all = [](std::vector<layout::legacy_track> const &Tracks) {
		std::vector<std::size_t> result(Tracks.size());
		std::iota(result.begin(), result.end(), std::size_t{0});
		return result;
	};
	auto const first{layout::plan_takeover(tracks, take_all(tracks))};
	REQUIRE(first.lines.size() == 1);
	auto const &reference{first.lines.front().fit};
	CAPTURE(described(reference));
	REQUIRE(reference.valid());
	for (unsigned const seed : {4401u, 5502u, 6603u}) {
		CAPTURE(seed);
		// the tracks in another order: the walk starts from another one of them, somewhere along the line
		auto shuffled{tracks};
		std::mt19937 random{seed}; // NOSONAR: seeded for repeatable tests, nothing secret is drawn
		std::ranges::shuffle(shuffled, random);
		auto const plan{layout::plan_takeover(shuffled, take_all(shuffled))};
		REQUIRE(plan.lines.size() == 1);
		auto const &fit{plan.lines.front().fit};
		CHECK(fit.start == reference.start);
		CHECK(fit.end == reference.end);
		REQUIRE(fit.shape.vertices.size() == reference.shape.vertices.size());
		for (std::size_t k = 0; k < fit.shape.vertices.size(); ++k) {
			CHECK(glm::distance(fit.shape.vertices[k].point, reference.shape.vertices[k].point) < 1e-6);
			CHECK(fit.shape.vertices[k].radius == doctest::Approx(reference.shape.vertices[k].radius).epsilon(1e-9));
			CHECK(fit.shape.vertices[k].transition_in == doctest::Approx(reference.shape.vertices[k].transition_in).epsilon(1e-9));
			CHECK(fit.shape.vertices[k].transition_out == doctest::Approx(reference.shape.vertices[k].transition_out).epsilon(1e-9));
		}
		// the old tracks along it the same, by name
		auto const &splits{plan.lines.front().hard_splits};
		auto const &expected{first.lines.front().hard_splits};
		REQUIRE(splits.size() == expected.size());
		for (std::size_t k = 0; k < splits.size(); ++k) {
			CHECK(splits[k].track.name == expected[k].track.name);
			CHECK(splits[k].chainage == doctest::Approx(expected[k].chainage).epsilon(1e-9));
		}
	}
}

TEST_CASE("takeover fit: a kink at a joint told, a curve of a radius of 11 km found where the curvature shows none, a held end kept") {
	// a straight of 300 m turning by 0.6 degree at a joint, a straight of 300 m on
	auto const turn{glm::radians(0.6)};
	glm::dvec2 const along{std::cos(turn), std::sin(turn)};
	segment_data first;
	first.points[segment_data::point::start] = {0.0, 10.0, 0.0};
	first.points[segment_data::point::end] = {300.0, 10.0, 0.0};
	segment_data second;
	second.points[segment_data::point::start] = {300.0, 10.0, 0.0};
	second.points[segment_data::point::end] = {300.0 + along.x * 300.0, 10.0, along.y * 300.0};
	auto const kinked{layout::recognize({{first, true}, {second, true}})};
	REQUIRE(kinked.valid());
	REQUIRE(kinked.shape.vertices.size() == 1);
	CHECK(glm::distance(kinked.shape.vertices.front().point, glm::dvec2{300.0, 0.0}) < 0.01);
	REQUIRE_FALSE(kinked.findings.empty());
	CHECK(kinked.findings.front().find("at a joint") != std::string::npos);
	CHECK(kinked.measured.plan < 0.02);
	// an arc of 11 km over 550 m in pieces of 50 m: 1/11000 is under what the curvature is read as a curve at, the line found keeping off it
	layout::line_shape wide;
	wide.vertices = {vertex_at({300.0, 0.0}, 11000.0, 0.0, 0.0, 0.0)};
	auto const deflection{0.05};
	auto const arc{laid(wide, {0.0, 0.0, 0.0}, {300.0 + std::cos(deflection) * 300.0, 0.0, std::sin(deflection) * 300.0})};
	REQUIRE(arc.valid());
	auto const old{cut(arc, 1212u, 45.0, 50.0, noise{0.0, 0.0, true})};
	auto const found{layout::recognize(old.chain)};
	CAPTURE(described(found));
	REQUIRE(found.shape.vertices.size() == 1);
	CHECK(found.shape.vertices.front().radius == doctest::Approx(11000.0).epsilon(0.01));
	CHECK(found.measured.plan < 0.005);
	// the end held by a switch in a heading the chain itself runs off by 2 mrad: the line ends in it
	layout::recognition_options options;
	options.end = {true, glm::normalize(glm::dvec2{std::cos(deflection + 0.002), std::sin(deflection + 0.002)})};
	auto const held{layout::recognize(old.chain, options)};
	REQUIRE(held.valid());
	CHECK(std::abs(geometry::signed_angle(held.solved.end.heading, options.end.heading)) < 1e-9);
}

