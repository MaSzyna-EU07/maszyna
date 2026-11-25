// the surface of a road where it crosses the rails (editor/editorCrossing.cpp): over each track in the plane of the heads of its rails,
// across the track with its cant and along it with its grade, the gaps along the rails, the approaches graded in within the regulation,
// the road going under a crossing panel. The tracks are read from the text of their entries, as the scenery loader reads them, and the
// heads are taken from their paths as the rail profiles are turned by the roll about the axis (TTrack::create_track_rail_profile)

#include "stdafx.h"
#include "doctest.h"
#include "editor/editorCrossing.hpp"
#include "world/CrossingPanels.h"
#include "track_text.h"

#include <cmath>
#include <string>
#include <vector>

namespace {

// the roll of the paths for a cant, as the editor makes it: over 1500 mm between the rail axes
double roll_of(double const Cant) {
	return glm::degrees(std::asin(Cant / 1500.0));
}

// a straight track along z at X from Z0 to Z1, the heights at its ends, rolled by the cant at both
std::string straight_track(std::string const &Name, double const X, double const Z0, double const Z1, double const Y0, double const Y1, double const Cant, double const Gauge = 1.435) {
	auto const roll{roll_of(Cant)};
	return "node -1 0 " + Name + " track normal " + std::to_string(Z1 - Z0) + " " + std::to_string(Gauge) + " 0.15 25.0 20 0 flat vis\n"
	       " rail_screw_used1 6 1435mm/tpbps-new2 0.2 0.5 1.1\n" +
	       std::to_string(X) + " " + std::to_string(Y0) + " " + std::to_string(Z0) + " " + std::to_string(roll) + "\n0 0 0\n0 0 0\n" + std::to_string(X) + " " + std::to_string(Y1) + " " +
	       std::to_string(Z1) + " " + std::to_string(roll) + "\n0\nendtrack";
}

// the track where the road crosses it, as the editor takes it (editor_road, rails_crossing())
level_crossing::rail_track crossed(TTrack const &Track, double const Distance, double const Head = 0.072) {
	auto rail{level_crossing::track_at(*Track.CurrentSegment(), Distance)};
	rail.name = Track.name();
	rail.gauge = std::abs(Track.fTrackWidth);
	rail.head = Head;
	return rail;
}

// the top of the head of a rail at a distance along the path, Out from the gauge face: the profile turned by the roll about the axis
glm::dvec3 rail_head(TTrack const &Track, int const Side, double const Distance, double const Out) {
	auto const &segment{*Track.CurrentSegment()};
	glm::dvec3 point, across;
	segment.plan_frame(Distance, point, across);
	auto const roll{static_cast<double>(segment.GetRoll(Distance))};
	auto const lateral{0.5 * std::abs(Track.fTrackWidth) + Out};
	return point + across * (Side * lateral * std::cos(roll)) + glm::dvec3{0.0, -Side * lateral * std::sin(roll), 0.0};
}

// a straight road from one point to another, at a height
level_crossing::road_piece road(glm::dvec3 const &From, glm::dvec3 const &To, double const Halfwidth = 3.5) {
	level_crossing::road_piece piece;
	piece.axis.points[segment_data::point::start] = From;
	piece.axis.points[segment_data::point::end] = To;
	piece.halfwidth = Halfwidth;
	return piece;
}

// the road as laid again: the parts of the pieces before and past the stretch kept, the stretch laid as designed
std::vector<level_crossing::road_piece> relaid(level_crossing::road_piece const &Original, level_crossing::design_result const &Result) {
	std::vector<level_crossing::road_piece> pieces;
	auto const &first{Result.pieces.front()};
	auto const &last{Result.pieces.back()};
	auto const &from{Original.axis.points[segment_data::point::start]};
	auto const &to{Original.axis.points[segment_data::point::end]};
	if (first.from > 0.0)
		pieces.push_back(road(from, glm::mix(from, to, first.from), Original.halfwidth));
	for (auto const &piece : Result.pieces)
		pieces.push_back({piece.axis, Original.halfwidth});
	if (last.to < 1.0)
		pieces.push_back(road(glm::mix(from, to, last.to), to, Original.halfwidth));
	return pieces;
}

} // namespace

TEST_CASE("level crossing: the surface lies in the plane of the heads within 1 mm, with a cant of 60 mm and a grade of 5 per mille") {
	test::track_reading reading;
	for (auto const cant : {60.0, -60.0}) {
		for (auto const angle : {90.0, 70.0}) {
			CAPTURE(cant);
			CAPTURE(angle);
			// 100 m rising by 0.5 m
			auto const track{test::read_track(straight_track("t1", 0.0, 0.0, 100.0, 10.0, 10.5, cant))};
			REQUIRE(track != nullptr);
			// the road crosses at the middle at the angle given, laid 0.3 m under the rails
			auto const s{50.0};
			glm::dvec3 middle, across;
			track->CurrentSegment()->plan_frame(s, middle, across);
			glm::dvec2 const direction{std::sin(glm::radians(angle)), std::cos(glm::radians(angle))};
			auto const from{glm::dvec3{middle.x - direction.x * 60.0, middle.y - 0.3, middle.z - direction.y * 60.0}};
			auto const to{glm::dvec3{middle.x + direction.x * 60.0, middle.y - 0.3, middle.z + direction.y * 60.0}};
			level_crossing::design_input input;
			input.pieces = {road(from, to)};
			input.tracks = {crossed(*track, s)};
			// the plane of the heads measured with a cant a crossing doesn't allow (see the test of the limit below)
			input.limits = false;
			auto const result{level_crossing::design(input)};
			REQUIRE(result.valid);
			CHECK(result.tracks == 1);
			CHECK(result.cant == doctest::Approx(std::abs(cant)).epsilon(0.001));
			// the tops of both heads, at their middle and across their whole tread, along the rails where the road lies over them
			auto checked{0};
			for (auto const side : {-1, 1})
				for (double along = -6.0; along <= 6.0; along += 0.25)
					for (auto const out : {0.016, 0.036, 0.057}) {
						auto const head{rail_head(*track, side, s + along, out)};
						double height{0.0};
						if (false == level_crossing::surface_height(result.pieces, 3.5, {head.x, head.z}, height))
							continue;
						CAPTURE(along);
						CAPTURE(side);
						CHECK(std::abs(height - head.y) <= 0.001);
						++checked;
					}
			CHECK(checked > 20);
		}
	}
}

TEST_CASE("level crossing: the plane of the heads of a track goes through the tops of both rails, turned about the axis by the roll") {
	test::track_reading reading;
	auto const track{test::read_track(straight_track("t1", 5.0, 0.0, 100.0, 0.0, 0.5, 60.0))};
	auto const rail{crossed(*track, 40.0)};
	auto const plane{level_crossing::plane_of(rail)};
	for (auto const side : {-1, 1})
		for (auto const along : {-1.0, 0.0, 2.0}) {
			auto const head{rail_head(*track, side, 40.0 + along, 0.03)};
			CHECK(std::abs(plane.at({head.x, head.z}) - head.y) <= 0.0005);
			auto const model{level_crossing::head_top(rail, side, along, 0.03)};
			CHECK(glm::distance(model, head) <= 0.0005);
		}
	// 60 mm over the 1500 mm between the rail axes
	CHECK(std::abs(rail.cant()) == doctest::Approx(60.0).epsilon(0.001));
}

TEST_CASE("level crossing: the gaps along the rails, the flangeway on the gauge side of the head, for the gauges of 1435 and 1000 mm and three rails") {
	for (auto const gauge : {1.435, 1.0}) {
		for (auto const name : {"60E1", "49E1", "S42"}) {
			CAPTURE(gauge);
			CAPTURE(name);
			auto const *rail{level_crossing::rail_named(name)};
			REQUIRE(rail != nullptr);
			auto const flangeway{level_crossing::flangeway_width(0.0)};
			auto const gaps{level_crossing::rail_gaps(gauge, rail->head, flangeway)};
			auto const &left{gaps[0]};
			auto const &right{gaps[1]};
			CHECK(left.side == -1);
			CHECK(right.side == 1);
			// the gauge faces where the gauge says
			CHECK(right.head_from == doctest::Approx(0.5 * gauge));
			CHECK(left.head_from == doctest::Approx(-0.5 * gauge));
			// the flangeway inside, the head outside of the gauge face
			CHECK(right.flangeway_from == doctest::Approx(0.5 * gauge - 0.067));
			CHECK(right.head_to == doctest::Approx(0.5 * gauge + rail->head));
			CHECK(left.flangeway_from == doctest::Approx(-0.5 * gauge + 0.067));
			CHECK(left.head_to == doctest::Approx(-0.5 * gauge - rail->head));
			CHECK(right.width() == doctest::Approx(0.067 + rail->head));
			CHECK(left.width() == doctest::Approx(right.width()));
			// the surface left between the rails is what's between the flangeways
			CHECK(right.flangeway_from - left.flangeway_from == doctest::Approx(gauge - 0.134));
		}
	}
	CHECK(level_crossing::rail_named("60E1")->head == doctest::Approx(0.072));
	CHECK(level_crossing::rail_named("49E1")->head == doctest::Approx(0.067));
	CHECK(level_crossing::rail_named("S42")->head == doctest::Approx(0.068));
	CHECK(level_crossing::rail_named("S42")->confirmed == false);
	CHECK(level_crossing::rail_named("R65") == nullptr);
	// wider on the tight curves: PLK, STWiORB D.11.01.01 (the 2025 regulation gives no figures)
	CHECK(level_crossing::flangeway_width(0.0) == doctest::Approx(0.067));
	CHECK(level_crossing::flangeway_width(350.0) == doctest::Approx(0.067));
	CHECK(level_crossing::flangeway_width(300.0) == doctest::Approx(0.075));
	CHECK(level_crossing::flangeway_width(-200.0) == doctest::Approx(0.080));
	CHECK(level_crossing::flangeway_gauge_depth(1.435) == doctest::Approx(0.014));
	CHECK(level_crossing::flangeway_gauge_depth(1.0) == doctest::Approx(0.010));
}

TEST_CASE("level crossing: the approaches no steeper than 3% within 20 m of the outer rails (Dz.U. 2025 poz. 1105, annex 5), the road led back to where it was") {
	CHECK(level_crossing::approach_grade == doctest::Approx(0.03));
	test::track_reading reading;
	auto const track{test::read_track(straight_track("t1", 0.0, 0.0, 100.0, 0.0, 0.0, 0.0))};
	for (auto const drop : {0.3, -0.3, 0.0, 0.45}) {
		CAPTURE(drop);
		auto const original{road({-60.0, -drop, 50.0}, {60.0, -drop, 50.0})};
		level_crossing::design_input input;
		input.pieces = {original};
		input.tracks = {crossed(*track, 50.0)};
		auto const result{level_crossing::design(input)};
		REQUIRE(result.valid);
		CHECK(result.steepest <= level_crossing::approach_grade + 1e-6);
		CHECK_FALSE(result.steep_approach);
		// along the road: x from -60; the deck from about x = -1.3 to 1.3, the approaches 30 m on either side of it
		CHECK(result.from > 60.0 - 1.3 - level_crossing::reach - 0.5);
		CHECK(result.to < 60.0 + 1.3 + level_crossing::reach + 0.5);
		double steepest{0.0};
		for (double x = -1.3 - 20.0; x <= 1.3 + 20.0; x += 0.1) {
			if (std::abs(x) < 1.3 + level_crossing::ease + 0.1)
				continue;
			auto const station{x + 60.0 - result.from};
			steepest = std::max(steepest, std::abs(level_crossing::axis_grade(result.pieces, station)));
		}
		CHECK(steepest <= level_crossing::approach_grade + 1e-3);
		// the ends of the stretch meet the road as it was, height and grade
		auto const &first{result.pieces.front().axis};
		auto const &last{result.pieces.back().axis};
		CHECK(first.points[segment_data::point::start].y == doctest::Approx(-drop).epsilon(1e-6));
		CHECK(last.points[segment_data::point::end].y == doctest::Approx(-drop).epsilon(1e-6));
		CHECK(std::abs(level_crossing::axis_grade(result.pieces, 0.05)) <= 1e-3);
		// the pieces joined end to end, no step and no kink between them
		for (std::size_t i = 0; i + 1 < result.pieces.size(); ++i) {
			auto const &a{result.pieces[i].axis};
			auto const &b{result.pieces[i + 1].axis};
			CHECK(glm::distance(a.points[segment_data::point::end], b.points[segment_data::point::start]) <= 1e-6);
			CHECK(a.rolls[1] == doctest::Approx(b.rolls[0]).epsilon(1e-4));
		}
	}
}

TEST_CASE("level crossing: two tracks of different cant side by side, each in its plane, the road straight from the head of one to the head of the other") {
	test::track_reading reading;
	auto const flat{test::read_track(straight_track("p", 120.0, 0.0, 100.0, 0.0, 0.0, 0.0))};
	auto const canted{test::read_track(straight_track("q", 124.5, 0.0, 100.0, 0.0, 0.0, 60.0))};
	level_crossing::design_input input;
	input.pieces = {road({100.0, -0.2, 50.0}, {145.0, -0.2, 50.0})};
	input.tracks = {crossed(*flat, 50.0), crossed(*canted, 50.0)};
	input.limits = false;
	auto const result{level_crossing::design(input)};
	REQUIRE(result.valid);
	CHECK(result.tracks == 2);
	CHECK(result.cant == doctest::Approx(60.0).epsilon(0.001));
	CHECK(result.cant_low == doctest::Approx(0.0).epsilon(0.001));
	for (auto const *track : {flat.get(), canted.get()})
		for (auto const side : {-1, 1})
			for (double along = -3.0; along <= 3.0; along += 0.5) {
				auto const head{rail_head(*track, side, 50.0 + along, 0.036)};
				double height{0.0};
				REQUIRE(level_crossing::surface_height(result.pieces, 3.5, {head.x, head.z}, height));
				CHECK(std::abs(height - head.y) <= 0.001);
			}
	// between the tracks the road goes straight from the field side of the head of the one to the head of the other: no hump, no curve
	// between the outer rails (Dz.U. 2025 poz. 1105, § 30 ust. 3)
	// the field sides of the heads of the rails facing each other: of p the one to +x, of q the one to -x
	auto const p1{rail_head(*flat, 1, 50.0, 0.072)}, p2{rail_head(*flat, -1, 50.0, 0.072)};
	auto const q1{rail_head(*canted, 1, 50.0, 0.072)}, q2{rail_head(*canted, -1, 50.0, 0.072)};
	auto const from{p1.x > p2.x ? p1 : p2}, to{q1.x < q2.x ? q1 : q2};
	auto const a{from.x}, b{to.x};
	auto const ha{from.y}, hb{to.y};
	CHECK(b - a > 2.5);
	for (double x = a + 0.01; x <= b - 0.01; x += 0.05) {
		CAPTURE(x);
		double height{0.0};
		REQUIRE(level_crossing::surface_height(result.pieces, 3.5, {x, 50.0}, height));
		CHECK(std::abs(height - (ha + (hb - ha) * (x - a) / (b - a))) <= 0.001);
	}
}

TEST_CASE("level crossing: laid again on what it laid, the road comes out the same") {
	test::track_reading reading;
	auto const track{test::read_track(straight_track("t1", 0.0, 0.0, 100.0, 0.0, 0.3, 60.0))};
	auto const original{road({-60.0, -0.25, 47.0}, {60.0, -0.25, 47.0})};
	level_crossing::design_input input;
	input.pieces = {original};
	input.tracks = {crossed(*track, 47.0)};
	input.limits = false;
	auto const first{level_crossing::design(input)};
	REQUIRE(first.valid);
	input.pieces = relaid(original, first);
	auto const second{level_crossing::design(input)};
	REQUIRE(second.valid);
	REQUIRE(second.pieces.size() == first.pieces.size());
	for (std::size_t i = 0; i < first.pieces.size(); ++i) {
		CAPTURE(i);
		// each piece of the stretch is a whole piece of the road now
		CHECK(second.pieces[i].from == doctest::Approx(0.0));
		CHECK(second.pieces[i].to == doctest::Approx(1.0));
		for (int p = 0; p < 4; ++p)
			CHECK(glm::distance(second.pieces[i].axis.points[p], first.pieces[i].axis.points[p]) <= 1e-4);
		CHECK(second.pieces[i].axis.rolls[0] == doctest::Approx(first.pieces[i].axis.rolls[0]).epsilon(1e-4));
	}
}

TEST_CASE("level crossing: a crossing panel lying on the track is counted, the road stays at the heads under it and leaves its surface out there") {
	test::track_reading reading;
	auto const track{test::read_track(straight_track("t1", 0.0, 0.0, 100.0, 0.0, 0.0, 0.0))};
	level_crossing::design_input input;
	input.pieces = {road({-60.0, -0.3, 50.0}, {60.0, -0.3, 50.0})};
	input.tracks = {crossed(*track, 50.0)};
	auto const bare{level_crossing::design(input)};
	// a panel 2.6 m along the road and 8 m across it, its top at the heads
	level_crossing::panel panel;
	panel.outline = level_crossing::hull({{-1.3, 46.0}, {1.3, 46.0}, {1.3, 54.0}, {-1.3, 54.0}, {0.0, 50.0}});
	panel.top = input.tracks.front().point.y;
	CHECK(panel.outline.size() == 4);
	input.panels = {panel};
	auto const covered{level_crossing::design(input)};
	REQUIRE(covered.valid);
	CHECK(covered.panels == 1);
	// the road isn't lowered under it: the lanes, which the vehicles go along, stay at the top of the panel
	for (auto const x : {-5.0, -2.0, -1.2, -0.6, 0.0, 0.6, 1.2, 2.0, 5.0}) {
		CAPTURE(x);
		double height{0.0}, before{0.0};
		REQUIRE(level_crossing::surface_height(covered.pieces, 3.5, {x, 50.0}, height));
		REQUIRE(level_crossing::surface_height(bare.pieces, 3.5, {x, 50.0}, before));
		// the deck reaches over the whole panel: the road past it a little different
		CHECK(std::abs(height - before) <= 0.002);
		if (std::abs(x) <= 1.2)
			CHECK(std::abs(height - panel.top) <= 0.001);
	}
	// the pieces of the road under the panel leave their surface out under its outline, as they make their geometry; the ones off it
	// don't, nor do they under a panel lying under the road, or under something standing on it higher than a slab
	int under{0};
	for (auto const &piece : covered.pieces) {
		auto const covers{road_crossing::covers(piece.axis, 3.5, {panel})};
		auto const a{piece.axis.points[segment_data::point::start].x};
		auto const b{piece.axis.points[segment_data::point::end].x};
		auto const over{std::min(a, b) < 1.3 + 3.5 && std::max(a, b) > -1.3 - 3.5};
		if (false == over)
			CHECK(covers.empty());
		under += covers.empty() ? 0 : 1;
	}
	CHECK(under >= 1);
	auto sunk{panel};
	sunk.top -= 0.1;
	auto tall{panel};
	tall.top += 0.5;
	for (auto const &piece : covered.pieces) {
		CHECK(road_crossing::covers(piece.axis, 3.5, {sunk}).empty());
		CHECK(road_crossing::covers(piece.axis, 3.5, {tall}).empty());
	}
	// the outline of the points: what touches it
	CHECK(level_crossing::touches(panel.outline, {0.0, 40.0}, {0.0, 60.0}));
	CHECK(level_crossing::touches(panel.outline, {0.5, 50.0}, {0.5, 50.0}));
	CHECK_FALSE(level_crossing::touches(panel.outline, {2.0, 40.0}, {2.0, 60.0}));
}

TEST_CASE("level crossing: none within a turnout, and none for a road along the track") {
	test::track_reading reading;
	auto const track{test::read_track(straight_track("t1", 0.0, 0.0, 100.0, 0.0, 0.0, 0.0))};
	level_crossing::design_input input;
	input.pieces = {road({-60.0, -0.3, 50.0}, {60.0, -0.3, 50.0})};
	auto rail{crossed(*track, 50.0)};
	rail.turnout = true;
	input.tracks = {rail};
	auto const refused{level_crossing::design(input)};
	CHECK_FALSE(refused.valid);
	CHECK(refused.reason.find("§ 34") != std::string::npos);
	input.tracks = {crossed(*track, 50.0)};
	input.pieces = {road({-0.2, -0.3, 0.0}, {0.3, -0.3, 100.0})};
	auto const along{level_crossing::design(input)};
	CHECK_FALSE(along.valid);
}

TEST_CASE("level crossing: the cant a crossing allows, the road over it at no more than 8% (Dz.U. 2025 poz. 1105, § 31 ust. 4, annex 5), a hard limit") {
	// square to the track, level: a slope of 8%, 119 mm over the 1500 mm between the rail axes; at 60 degrees more, as the road goes along
	// the cant less
	CHECK(level_crossing::allowed_cant(90.0, 0.0, false) == doctest::Approx(119.0));
	CHECK(level_crossing::allowed_cant(60.0, 0.0, false) == doctest::Approx(137.0));
	// the narrow gauge: 2%
	CHECK(level_crossing::allowed_cant(90.0, 0.0, true) == doctest::Approx(29.0));
	// the grade of the track along the road takes its share
	CHECK(level_crossing::allowed_cant(60.0, 0.01, false) < level_crossing::allowed_cant(60.0, 0.0, false));
	CHECK(level_crossing::allowed_cant(90.0, 0.01, false) == doctest::Approx(119.0));
	test::track_reading reading;
	for (auto const [cant, laid] : {std::pair{150.0, false}, std::pair{120.0, false}, std::pair{119.0, true}, std::pair{60.0, true}, std::pair{-150.0, false}}) {
		CAPTURE(cant);
		auto const track{test::read_track(straight_track("t1", 0.0, 0.0, 100.0, 0.0, 0.0, cant))};
		level_crossing::design_input input;
		input.pieces = {road({-60.0, -0.3, 50.0}, {60.0, -0.3, 50.0})};
		input.tracks = {crossed(*track, 50.0)};
		auto const result{level_crossing::design(input)};
		CHECK(result.valid == laid);
		CHECK(result.over.empty() == laid);
		if (false == laid) {
			REQUIRE(result.over.size() == 1);
			CHECK(result.over.front().cant == doctest::Approx(std::abs(cant)).epsilon(0.001));
			CHECK(result.over.front().allowed == doctest::Approx(119.0));
		} else {
			// at the most it allows the road over it isn't steeper than 8%
			CHECK_FALSE(result.steep_cross);
		}
	}
}

TEST_CASE("level crossing: none on a cant ramp (Dz.U. 2025 poz. 1105, § 31 ust. 3), a hard limit") {
	test::track_reading reading;
	// a straight taking a cant of 60 mm on over its 100 m, and one keeping it all the way
	auto const ramp_text{"node -1 0 r1 track normal 100 1.435 0.15 25.0 20 0 flat vis\n rail_screw_used1 6 1435mm/tpbps-new2 0.2 0.5 1.1\n0 0 0 0\n0 0 0\n0 0 0\n0 0 100 " +
	                     std::to_string(roll_of(60.0)) + "\n0\nendtrack"};
	auto const ramp{test::read_track(ramp_text)};
	auto const canted{test::read_track(straight_track("c1", 0.0, 0.0, 100.0, 0.0, 0.0, 60.0))};
	level_crossing::design_input input;
	input.pieces = {road({-60.0, -0.3, 50.0}, {60.0, -0.3, 50.0})};
	input.tracks = {crossed(*ramp, 50.0)};
	CHECK(input.tracks.front().ramp);
	auto const refused{level_crossing::design(input)};
	CHECK_FALSE(refused.valid);
	REQUIRE(refused.on_ramp.size() == 1);
	CHECK(refused.reason.find("§ 31 ust. 3") != std::string::npos);
	input.tracks = {crossed(*canted, 50.0)};
	CHECK_FALSE(input.tracks.front().ramp);
	auto const laid{level_crossing::design(input)};
	CHECK(laid.valid);
	CHECK(laid.on_ramp.empty());
	// measured, the plane of the heads laid whatever the cant: the ramp too
	input.tracks = {crossed(*ramp, 50.0)};
	input.limits = false;
	CHECK(level_crossing::design(input).valid);
}

TEST_CASE("level crossing: the approach of a road ending near the rails within 3% to its end, or the crossing refused (Dz.U. 2025 poz. 1105, annex 5)") {
	test::track_reading reading;
	auto const track{test::read_track(straight_track("t1", 0.0, 0.0, 100.0, 0.0, 0.0, 119.0))};
	for (auto const [drop, laid] : {std::pair{0.2, true}, std::pair{0.0, true}, std::pair{1.5, false}}) {
		CAPTURE(drop);
		// the road ends 12 m from the axis of the track on the one side, at the heads less the drop
		level_crossing::design_input input;
		input.pieces = {road({-12.0, -drop + 0.18, 50.0}, {40.0, 0.0, 50.0})};
		input.tracks = {crossed(*track, 50.0)};
		auto const result{level_crossing::design(input)};
		CHECK(result.valid == laid);
		if (laid) {
			CHECK(result.steepest <= level_crossing::approach_grade + 1e-4);
			// the approach led to the end of the road, at its height
			double height{0.0};
			REQUIRE(level_crossing::surface_height(result.pieces, 3.5, {-11.9, 50.0}, height));
			CHECK(std::abs(height - (-drop + 0.18)) <= 0.01);
		} else
			CHECK(result.reason.find("annex 5") != std::string::npos);
	}
}

TEST_CASE("level crossing: the panels CBP lying on a canted track: the road at their tops over all of their footprints, meeting their edges, and none of it left under them") {
	test::track_reading reading;
	auto const track{test::read_track(straight_track("t1", 0.0, 0.0, 100.0, 0.0, 0.005 * 100.0, 100.0))};
	auto const rail{crossed(*track, 50.0)};
	auto const plane{level_crossing::plane_of(rail)};
	// the rows as a sweep lays them: across the track its right, along it its direction, the top in the plane of the heads
	auto const lay = [&](crossing_panels::spec const &Spec, double const Lateral, int const Side) {
		level_crossing::panel panel;
		std::vector<glm::dvec2> points;
		panel.top = -1e9;
		for (auto const &vertex : crossing_panels::geometry(Spec)) {
			auto const along{vertex.position.z - 4.5};
			auto const across{Lateral + Side * vertex.position.x};
			glm::dvec2 const place{glm::dvec2{rail.point.x, rail.point.z} + rail.along * along + rail.right() * across};
			points.push_back(place);
			panel.top = std::max(panel.top, plane.at(place) + vertex.position.y);
		}
		panel.outline = level_crossing::hull(points);
		return panel;
	};
	auto const outside{0.5 * rail.gauge + rail.head + crossing_panels::outer_gap};
	std::vector<level_crossing::panel> panels{lay({crossing_panels::piece::inner_row, 180, 1435, 3}, 0.0, 1), lay({crossing_panels::piece::outer_row, 180, 1435, 3}, outside, 1),
	                                          lay({crossing_panels::piece::outer_row, 180, 1435, 3}, -outside, -1)};
	level_crossing::design_input input;
	input.pieces = {road({-60.0, -0.3, 50.0}, {60.0, -0.3, 50.0})};
	input.tracks = {rail};
	input.panels = panels;
	auto const result{level_crossing::design(input)};
	REQUIRE(result.valid);
	CHECK(result.panels == 3);
	// the road at the top of the panels on a grid of 2 cm over each footprint within the road, and just past the outer edges
	auto const inside = [](std::vector<glm::dvec2> const &Outline, glm::dvec2 const &Point) {
		for (std::size_t i = 0; i < Outline.size(); ++i) {
			auto const &a{Outline[i]};
			auto const &b{Outline[(i + 1) % Outline.size()]};
			if ((b.x - a.x) * (Point.y - a.y) - (b.y - a.y) * (Point.x - a.x) < 0.0)
				return false;
		}
		return true;
	};
	double worst{0.0};
	int samples{0};
	for (double x = -1.6; x <= 1.6; x += 0.02)
		for (double z = 47.0; z <= 53.0; z += 0.02) {
			glm::dvec2 const point{x, z};
			auto const under{std::any_of(panels.begin(), panels.end(), [&](auto const &Panel) { return inside(Panel.outline, point); })};
			double height{0.0};
			if (false == under || false == level_crossing::surface_height(result.pieces, 3.5, point, height))
				continue;
			worst = std::max(worst, std::abs(height - plane.at(point)));
			++samples;
		}
	CHECK(samples > 1000);
	CHECK(worst <= 0.001);
	for (auto const x : {-1.49, 1.49}) {
		double height{0.0};
		REQUIRE(level_crossing::surface_height(result.pieces, 3.5, {x, 50.0}, height));
		CHECK(std::abs(height - plane.at({x, 50.0})) <= 0.002);
	}
	// the surface of the road as drawn, a grid of triangles of 10 cm at the heights designed, cut along the rails and under the panels as
	// the road cuts itself: no corner of it left within a footprint, so none over a panel or within 1 mm of its top
	std::vector<world_vertex> mesh;
	auto const at = [&](double const X, double const Z) {
		double height{0.0};
		level_crossing::surface_height(result.pieces, 3.5, {X, Z}, height);
		return world_vertex{{X, height, Z}, {0.f, 1.f, 0.f}, {0.f, 0.f}};
	};
	for (double x = -3.0; x < 3.0 - 1e-9; x += 0.1)
		for (double z = 46.6; z < 53.4 - 1e-9; z += 0.1) {
			auto const a{at(x, z)}, b{at(x, z + 0.1)}, c{at(x + 0.1, z + 0.1)}, d{at(x + 0.1, z)};
			mesh.insert(mesh.end(), {a, b, c, a, c, d});
		}
	std::vector<std::vector<glm::dvec2>> covers;
	for (auto const &piece : result.pieces)
		for (auto const &cover : road_crossing::covers(piece.axis, 3.5, panels))
			covers.push_back(cover);
	REQUIRE(covers.size() >= 3);
	road_crossing::cut_gaps(mesh, road_crossing::gaps(input.pieces.front().axis, 3.5, {rail}));
	road_crossing::cut_gaps(mesh, covers);
	int within{0};
	for (std::size_t i = 0; i + 2 < mesh.size(); i += 3) {
		auto const middle{(mesh[i].position + mesh[i + 1].position + mesh[i + 2].position) / 3.0};
		for (auto const &panel : panels)
			if (inside(panel.outline, {middle.x, middle.z}))
				++within;
	}
	CHECK(within == 0);
}
