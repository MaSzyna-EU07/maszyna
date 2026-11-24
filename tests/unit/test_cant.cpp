// the cant of the tracks the editor lays and joins: from mm to the roll of the paths and back, signed with the turn, and matched
// across the joints of the tracks
//
// the inspector of the editor shows and takes the cant on a base of 1500 mm through editor_mode::cant_of_roll() and roll_of_cant()
// (application/editormodetrack.cpp). They live in the editor, which the unit tests don't link, so the self-test cases check them
// through the inspector: tests/editor/cases/1_inspector_cant.txt

#include "stdafx.h"
#include "doctest.h"
#include "editor/editorAlignment.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorTrack.hpp"
#include "track_text.h"

#include <cmath>
#include <string>
#include <vector>

namespace {

// a corner of two straights eased by a curve of R 400 with transitions of 60 m and a cant of 80 mm: from the origin along z, then
// along x, to +x for Side 1 and to -x for Side -1
alignment::design corner(double const Side) {
	alignment::design design;
	design.start = {0.0, 0.0, 0.0};
	design.end = {1000.0 * Side, 0.0, 1000.0};
	design.start_direction = {0.0, 1.0};
	design.end_direction = {Side, 0.0};
	alignment::vertex vertex;
	vertex.position = {0.0, 1000.0};
	vertex.radius = 400.0;
	vertex.transition_in = 60.0;
	vertex.transition_out = 60.0;
	vertex.cant = 80.0;
	design.vertices.push_back(vertex);
	return design;
}

// the cant of a roll, mm, as the editor takes it from the paths: on the gauge of the design, a roll above 0 a cant above 0
double cant_of(float const Roll, double const Gauge) {
	return Gauge * std::sin(glm::radians(static_cast<double>(Roll)));
}

// the paths of the line with the chainage of the start of each
struct laid_path {
	segment_data path;
	double from{0.0};
	double to{0.0};
};
std::vector<laid_path> laid(alignment::result const &Line, alignment::design const &Design) {
	std::vector<laid_path> result;
	double chainage{0.0};
	for (auto const &path : alignment::pieces(Line, Design, alignment::minimum_pieces(Line, Design))) {
		auto const length{geometry::bezier{path}.plan_length()};
		result.push_back({path, chainage, chainage + length});
		chainage += length;
	}
	return result;
}

} // namespace

TEST_CASE("cant: the paths of a curve roll by the cant designed on the gauge, mm to degrees and back, signed with the turn") {
	for (auto const side : {1.0, -1.0}) {
		CAPTURE(side);
		auto const design{corner(side)};
		auto const line{alignment::compute(design)};
		REQUIRE(line.valid);
		auto const gauge{design.norms.gauge};
		auto const paths{laid(line, design)};
		REQUIRE_FALSE(paths.empty());
		auto steepest{0.f};
		for (auto const &[path, from, to] : paths) {
			// the cant the rolls at the ends of each path come back to is the one of the line there, with the sign of its turn
			for (auto const &[roll, chainage] : {std::pair{path.rolls[0], from}, std::pair{path.rolls[1], to}}) {
				CAPTURE(chainage);
				auto const station{alignment::evaluate(line, chainage)};
				CHECK(cant_of(roll, gauge) == doctest::Approx(station.turn * station.cant).epsilon(1e-3).scale(1.0));
				if (std::abs(roll) > std::abs(steepest)) {
					steepest = roll;
				}
			}
		}
		// the arc rolls by the cant of the curve: 80 mm on the gauge, the sign of its turn
		CHECK(std::abs(steepest) == doctest::Approx(glm::degrees(std::asin(80.0 / gauge))).epsilon(1e-4));
		CHECK((steepest < 0.f) == (side > 0.0));
	}
	// a curve the other way rolls the other way, by as much
	auto const right{laid(alignment::compute(corner(1.0)), corner(1.0))};
	auto const left{laid(alignment::compute(corner(-1.0)), corner(-1.0))};
	REQUIRE(right.size() == left.size());
	for (std::size_t index = 0; index < right.size(); ++index) {
		CHECK(right[index].path.rolls[0] == doctest::Approx(-left[index].path.rolls[0]));
		CHECK(right[index].path.rolls[1] == doctest::Approx(-left[index].path.rolls[1]));
	}
}

TEST_CASE("cant: at a joint the editor gives the joined track the roll of the track, turned with the way its path goes") {
	test::track_reading const reading;
	using kind = editor_track::point_kind;
	// a straight along z with a ramp of the cant, 1.5 degrees at its start and 2.25 at its end, and one to join to it
	std::string const ramp{"node -1 0 a track normal 10 1.435 0.15 25 20 0 flat unvis 0 0 0 1.5 0 0 0 0 0 0 0 0 10 2.25 0 endtrack\n"};
	auto const joined = [](char const *Name) {
		return std::string{"node -1 0 "} + Name + " track normal 10 1.435 0.15 25 20 0 flat unvis 0 0 10 0 0 0 0 0 0 0 0 0 20 0 0 endtrack\n";
	};
	// the roll of a path at its end, for the travel into the path: as it is at its start, turned at its end, where the path comes out
	auto const track{test::read_track(ramp)};
	CHECK(editor_track::joint_roll(*track, {0, kind::start}) == doctest::Approx(1.5));
	CHECK(editor_track::joint_roll(*track, {0, kind::end}) == doctest::Approx(-2.25));
	CHECK(editor_track::joint_roll(*track, {1, kind::end}) == 0.0); // no such path

	struct joint_case {
		char const *what;
		kind own;
		kind theirs;
		int changed; // the roll of the joined track the joint sets: 0 at its start, 1 at its end
		float roll;
	};
	for (auto const &[what, own, theirs, changed, roll] : {
	         joint_case{"the end to the start, one path on from the other: the same roll", kind::end, kind::start, 0, 2.25f},
	         joint_case{"the end to the end, the paths facing: the roll turned", kind::end, kind::end, 1, -2.25f},
	         joint_case{"the start to the start, the paths back to back: the roll turned", kind::start, kind::start, 0, -1.5f},
	         joint_case{"the start to the end, one path on from the other: the same roll", kind::start, kind::end, 1, 1.5f},
	     }) {
		CAPTURE(what);
		auto const other{test::read_track(joined("b"))};
		editor_track::match_rolls(*track, {{{0, own}, other.get(), {0, theirs}}});
		CHECK(other->m_paths[0].rolls[changed] == roll);
		CHECK(other->m_paths[0].rolls[1 - changed] == 0.f);
		// the cant seen from either side of the joint, for the travel across it, is the same
		CHECK(editor_track::joint_roll(*other, {0, theirs}) == doctest::Approx(-editor_track::joint_roll(*track, {0, own})));
		// and it's what the editor writes
		auto const again{test::read_track(test::written(*other))};
		CHECK(again->m_paths[0].rolls == other->m_paths[0].rolls);
	}
}
