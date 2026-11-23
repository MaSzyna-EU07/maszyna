// the switch tracks the generator makes of the turnouts of the track geometry (editor/layout, editor-tracks-domain.md §5): written and read
// back by the game's own TTrack::Load(), the ends of both paths at the nodes of the legs to the bit, the same numbers as the tracks on the
// other side of them, the pieces of each path within half a millimetre of its line both ways, the velocity as the speed rule gives it, the
// frog and the blades of the type; the diverging track of Rz 60E1-2500-1:26,5 within the chord offsets of Id-4

#include "stdafx.h"
#include "doctest.h"
#include "layout_fixture.h"
#include "track_text.h"
#include "editor/layout/layoutGenerator.hpp"

#include <algorithm>
#include <set>

namespace {

using namespace test::layout_fixture;
using layout::line_id;
using layout::turnout_id;

double constexpr kHalfMillimetre{0.0005};

// the farthest the line lies from the curve drawn: each of its points against the nearest point of the curve, searched on its own (the
// other way round from layout::deviation)
double line_from_curve(layout::solved_line const &Line, layout::drawn_curve const &Curve, double const From, double const To) {
	double worst{0.0};
	int constexpr samples{40};
	for (int i = 0; i <= samples; ++i) {
		auto const s{From + (To - From) * i / samples};
		auto const target{Line.point(s)};
		int constexpr coarse{400};
		double best{0.0};
		double nearest{1e9};
		for (int k = 0; k <= coarse; ++k) {
			auto const t{static_cast<double>(k) / coarse};
			if (auto const d{glm::distance(Curve.point(t), target)}; d < nearest) {
				nearest = d;
				best = t;
			}
		}
		double low{std::max(0.0, best - 1.0 / coarse)};
		double high{std::min(1.0, best + 1.0 / coarse)};
		for (int k = 0; k < 60; ++k) {
			auto const a{high - (high - low) * 0.618};
			auto const b{low + (high - low) * 0.618};
			if (glm::distance(Curve.point(a), target) < glm::distance(Curve.point(b), target)) {
				high = b;
			}
			else {
				low = a;
			}
		}
		worst = std::max(worst, glm::distance(Curve.point(0.5 * (low + high)), target));
	}
	return worst;
}

layout::create_line straight(double const Length = 1000.0) {
	layout::create_line result;
	result.start = {0.0, 100.0, 0.0};
	result.end = {Length, 100.0 + Length * 0.004, 0.0};
	result.category = line_class();
	return result;
}

// a curve of R 1200 m with 60 mm turning left by 30 degrees
layout::create_line bend() {
	layout::create_line result;
	result.start = {0.0, 100.0, 500.0};
	glm::dvec2 const corner{700.0, 500.0};
	auto const turn{3.14159265358979323846 / 6.0};
	auto const end{corner + glm::dvec2{std::cos(turn), std::sin(turn)} * 700.0};
	result.end = {end.x, 100.0, end.y};
	layout::vertex curve;
	curve.point = corner;
	curve.radius = 1200.0;
	curve.cant = 60.0;
	curve.transition_in = curve.transition_out = 70.0;
	result.vertices = {curve};
	result.category = line_class();
	return result;
}

double middle_of_arc(layout::line const &Line) {
	for (auto const &e : Line.solved.elements) {
		if (e.role == layout::element_role::arc) {
			return e.chainage + 0.5 * e.length;
		}
	}
	return 0.0;
}

// the points of a chain of Bezier pieces by the length along it, as the simulator draws them
struct drawn_chain {
	std::vector<glm::dvec3> points;
	std::vector<double> lengths;

	explicit drawn_chain(std::vector<segment_data> const &Pieces) {
		for (auto const &piece : Pieces) {
			auto const curve{layout::drawn(piece)};
			int constexpr steps{2000};
			for (int i = points.empty() ? 0 : 1; i <= steps; ++i) {
				auto const point{curve.point(static_cast<double>(i) / steps)};
				lengths.push_back(points.empty() ? 0.0 : lengths.back() + glm::distance(points.back(), point));
				points.push_back(point);
			}
		}
	}
	double length() const { return lengths.back(); }
	glm::dvec2 at(double const S) const {
		auto const next{static_cast<std::size_t>(std::lower_bound(lengths.begin(), lengths.end(), S) - lengths.begin())};
		auto const i{std::clamp<std::size_t>(next, 1, points.size() - 1)};
		auto const t{(S - lengths[i - 1]) / std::max(1e-12, lengths[i] - lengths[i - 1])};
		return geometry::plan_of(glm::mix(points[i - 1], points[i], std::clamp(t, 0.0, 1.0)));
	}
};

} // namespace

TEST_CASE("layout generator G: switch tracks read back by TTrack::Load: the legs at the nodes to the bit, both paths within 0.5 mm, the velocity") {
	test::track_reading const reading;
	layout::model model;
	REQUIRE(model.execute(straight()).done);
	auto const line{model.state().lines().begin()->first};
	// on a straight with a grade: a 1:12 to the left along the line, a 1:9 to the right against it; one bent outward into R 1200 m
	auto const first{model.execute(layout::insert_turnout{line, 300.0, "Rz 60E1-500-1:12", 1, 1})};
	CAPTURE(reasons_of(first));
	REQUIRE(first.done);
	auto const second{model.execute(layout::insert_turnout{line, 200.0, "Rz 60E1-190-1:9", -1, -1})};
	CAPTURE(reasons_of(second));
	REQUIRE(second.done);
	auto const curved{model.execute(bend())};
	REQUIRE(curved.done);
	auto const bent_line{curved.lines.front()};
	auto const third{model.execute(layout::insert_turnout{bent_line, middle_of_arc(*model.state().find(bent_line)), "Rz 60E1-500-1:12", -1, 1})};
	CAPTURE(reasons_of(third));
	REQUIRE(third.done);
	// and a 1:26.5 with a movable nose on the line going on from the end of the first one
	auto const fourth{model.execute(layout::insert_turnout{first.lines[1], 200.0, "Rz 60E1-2500-1:26,5 movable nose", 1, 1})};
	CAPTURE(reasons_of(fourth));
	REQUIRE(fourth.done);
	auto const &state{model.state()};
	check_invariants(state);
	std::vector<std::string> errors;
	auto const all{layout::generate_all(state, {}, errors)};
	CHECK(errors.empty());
	// a track of a turnout isn't made as a line of its own
	CHECK_FALSE(layout::generate(state, state.find(first.turnouts.front())->diverging).errors.empty());
	double worst{0.0};
	for (auto const &[id, turnout] : state.turnouts()) {
		CAPTURE(layout::number(id));
		auto const made{layout::generate(state, id)};
		REQUIRE(made.errors.empty());
		CHECK(all.find(made.text) != std::string::npos);
		CAPTURE(made.text);
		auto const track{test::read_track(made.text)};
		REQUIRE(track->eType == tt_Switch);
		REQUIRE(track->m_paths.size() == 2);
		CHECK(track->name() == made.name);
		auto const type{layout::turnout_type_of(turnout.type)};
		// the velocity as computed, the frog and the blades of the type
		REQUIRE(track->SwitchExtension != nullptr);
		CHECK(track->SwitchExtension->fVelocity == static_cast<float>(turnout.velocity));
		CHECK(track->SwitchExtension->m_movablenose == type->movable);
		CHECK(track->SwitchExtension->m_frogs);
		// the legs: both paths from the start, the main one to the end of the main track, the diverging one to its end, to the bit
		auto const &start{state.find(turnout.start)->position};
		std::array<glm::dvec3, 2> const ends{state.find(turnout.main_end)->position, state.find(turnout.diverging_end)->position};
		std::array<line_id, 2> const lines{turnout.main, turnout.diverging};
		for (std::size_t i = 0; i < 2; ++i) {
			CAPTURE(i);
			auto const &path{track->m_paths[i]};
			CHECK(path.points[segment_data::point::start] == start);
			CHECK(path.points[segment_data::point::end] == ends[i]);
			// the pieces as written, joined to the bit, within half a millimetre of the line both ways
			auto const &pieces{made.pieces[i]};
			auto const &read{path.pieces.empty() ? std::vector<segment_data>{path} : path.pieces};
			REQUIRE(read.size() == pieces.size());
			CHECK(read.front().points[segment_data::point::start] == start);
			CHECK(read.back().points[segment_data::point::end] == ends[i]);
			auto const &solved{state.find(lines[i])->solved};
			for (std::size_t k = 0; k < read.size(); ++k) {
				CAPTURE(k);
				CHECK(read[k].points == pieces[k].path.points);
				CHECK(read[k].rolls == pieces[k].path.rolls);
				if (k > 0) {
					CHECK(read[k - 1].points[segment_data::point::end] == read[k].points[segment_data::point::start]);
					CHECK(read[k - 1].rolls[1] == read[k].rolls[0]);
				}
				// measured along the line as it runs
				auto const forward{pieces[k].from < pieces[k].to};
				auto const curve{layout::drawn(forward ? read[k] : geometry::reversed(read[k]))};
				auto const from{std::min(pieces[k].from, pieces[k].to)};
				auto const to{std::max(pieces[k].from, pieces[k].to)};
				auto const there{layout::deviation(solved, curve, from, to)};
				auto const back{line_from_curve(solved, curve, from, to)};
				CHECK(there < kHalfMillimetre);
				CHECK(back < kHalfMillimetre);
				worst = std::max({worst, there, back});
			}
		}
		// the diverging track of the radius bent, the main one of the curve it lies in
		auto const sharpest{std::ranges::min(made.pieces[1], {}, [](layout::generated_piece const &Piece) { return Piece.path.radius > 0.f ? Piece.path.radius : 1e9f; }).path.radius};
		CHECK(sharpest == doctest::Approx(std::abs(turnout.diverging_radius)).epsilon(1e-6));
		// the tracks on the other side of the legs have the same numbers there: the place and the roll as each runs
		for (auto const leg : {turnout.start, turnout.main_end}) {
			auto const &node{*state.find(leg)};
			for (auto const &end : node.ends) {
				if (layout::owner_of(state, end.line) != nullptr) {
					continue;
				}
				auto const theirs{layout::generate(state, end.line)};
				REQUIRE(theirs.errors.empty());
				auto const &piece{end.end == layout::line_end::start ? theirs.pieces.front().path : theirs.pieces.back().path};
				auto const index{end.end == layout::line_end::start ? segment_data::point::start : segment_data::point::end};
				CHECK(piece.points[index] == node.position);
				auto const &mine{track->m_paths[0]};
				auto const my_index{leg == turnout.start ? segment_data::point::start : segment_data::point::end};
				CHECK(mine.points[my_index] == node.position);
				// running on from it, the same roll; meeting it head on, the other sign
				auto const their_roll{piece.rolls[end.end == layout::line_end::start ? 0 : 1]};
				auto const my_roll{(mine.pieces.empty() ? mine : (leg == turnout.start ? mine.pieces.front() : mine.pieces.back())).rolls[leg == turnout.start ? 0 : 1]};
				auto const runs_on{(leg == turnout.start) == (end.end == layout::line_end::end)};
				CHECK(my_roll == (runs_on ? their_roll : -their_roll));
			}
		}
	}
	MESSAGE("the switch tracks: the farthest piece " << worst * 1000.0 << " mm from its line");
}

TEST_CASE("layout generator G: the diverging track of Rz 60E1-2500-1:26,5 within the chord offsets of Id-4, 41 to 49 mm in the middle of 30 m") {
	test::track_reading const reading;
	layout::model model;
	REQUIRE(model.execute(straight()).done);
	auto const line{model.state().lines().begin()->first};
	auto const made{model.execute(layout::insert_turnout{line, 300.0, "Rz 60E1-2500-1:26,5 movable nose", -1, 1})};
	CAPTURE(reasons_of(made));
	REQUIRE(made.done);
	auto const &turnout{*model.state().find(made.turnouts.front())};
	// 130 km/h on the diverging track, as Id-4 gives it; the line's 80 written for both tracks
	CHECK(turnout.speed_diverging == 130.0);
	CHECK(turnout.velocity == 80.0);
	auto const track{test::read_track(layout::generate(model.state(), turnout.id).text)};
	REQUIRE(track->m_paths.size() == 2);
	auto const &path{track->m_paths[1]};
	drawn_chain const diverging{path.pieces.empty() ? std::vector<segment_data>{path} : path.pieces};
	CHECK(std::abs(diverging.length() - 2500.0 * std::atan(1.0 / 26.5)) < 0.001);
	double least{1.0};
	double most{0.0};
	for (double s = 0.0; s + 30.0 <= diverging.length(); s += 1.0) {
		auto const a{diverging.at(s)};
		auto const b{diverging.at(s + 30.0)};
		auto const middle{diverging.at(s + 15.0)};
		auto const offset{std::abs(geometry::cross(glm::normalize(b - a), middle - a))};
		least = std::min(least, offset);
		most = std::max(most, offset);
	}
	MESSAGE("Rz 60E1-2500-1:26,5: the offsets of 30 m chords " << least * 1000.0 << " to " << most * 1000.0 << " mm");
	CHECK(least >= 0.041);
	CHECK(most <= 0.049);
	CHECK(most - least < 0.0005);
}
