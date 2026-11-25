// the gaps a road leaves along the rails it crosses with its surface in the plane of their heads (world/RoadCrossing.cpp), as the road
// cuts itself when it makes its geometry, in the editor and in the simulation: which roads are cut, where, and what's left of the triangles

#include "stdafx.h"
#include "doctest.h"
#include "world/RoadCrossing.h"
#include "model/vertex.h"
#include "track_text.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace {

// a straight track along z at X from Z0 to Z1, the heights at its ends, rolled by the cant at both (1500 mm between the rail axes)
std::string straight_track(std::string const &Name, double const X, double const Z0, double const Z1, double const Y0, double const Y1, double const Cant) {
	auto const roll{glm::degrees(std::asin(Cant / 1500.0))};
	return "node -1 0 " + Name + " track normal " + std::to_string(Z1 - Z0) + " 1.435 0.15 25.0 20 0 flat vis\n"
	       " rail_screw_used1 6 1435mm/tpbps-new2 0.2 0.5 1.1\n" +
	       std::to_string(X) + " " + std::to_string(Y0) + " " + std::to_string(Z0) + " " + std::to_string(roll) + "\n0 0 0\n0 0 0\n" + std::to_string(X) + " " + std::to_string(Y1) + " " +
	       std::to_string(Z1) + " " + std::to_string(roll) + "\n0\nendtrack";
}

road_crossing::rail_track crossed(TTrack const &Track, double const Distance) {
	auto rail{road_crossing::track_at(*Track.CurrentSegment(), Distance)};
	rail.name = Track.name();
	rail.gauge = std::abs(Track.fTrackWidth);
	rail.head = 0.072;
	return rail;
}

// a straight road along x at Z, from X0 to X1, its axis at the heights given, rolled by Roll degrees
segment_data road(double const X0, double const X1, double const Z, double const Y0, double const Y1, double const Roll = 0.0) {
	segment_data axis;
	axis.points[segment_data::point::start] = {X0, Y0, Z};
	axis.points[segment_data::point::end] = {X1, Y1, Z};
	axis.rolls = {static_cast<float>(Roll), static_cast<float>(Roll)};
	return axis;
}

// the same road laid in the plane of the heads of the track: its ends on the plane, rolled as the plane leans across it
segment_data road_in(road_crossing::head_plane const &Plane, double const X0, double const X1, double const Z, double const Off = 0.0) {
	// the left of a road going along +x, as the road lays its cross-section
	glm::dvec2 const left{0.0, -1.0};
	return road(X0, X1, Z, Plane.at({X0, Z}) + Off, Plane.at({X1, Z}) + Off, glm::degrees(std::atan(glm::dot(Plane.gradient, left))));
}

// true if the point lies within the convex outline
bool road_crossing_inside(std::vector<glm::dvec2> const &Outline, glm::dvec2 const &Point) {
	bool positive{false}, negative{false};
	for (std::size_t i = 0; i < Outline.size(); ++i) {
		auto const &a{Outline[i]};
		auto const &b{Outline[(i + 1) % Outline.size()]};
		auto const side{(b.x - a.x) * (Point.y - a.y) - (b.y - a.y) * (Point.x - a.x)};
		positive |= side > 0.0;
		negative |= side < 0.0;
	}
	return false == (positive && negative);
}

} // namespace

TEST_CASE("road crossing: a road is cut along the rails where its surface lies in the plane of the heads within 2 mm, and nowhere else") {
	test::track_reading reading;
	auto const canted{test::read_track(straight_track("c", 0.0, 0.0, 100.0, 0.0, 0.5, 60.0))};
	auto const flat{test::read_track(straight_track("f", 10.0, 0.0, 100.0, 0.0, 0.0, 0.0))};
	auto const rail{crossed(*canted, 50.0)};
	auto const plane{road_crossing::plane_of(rail)};
	// laid into the plane of the canted track: its surface through the heads of both rails, both cut
	auto const laid{road_in(plane, -10.0, 10.0, 50.0)};
	for (auto const side : {-1, 1}) {
		auto const head{road_crossing::head_top(rail, side, 0.0, 0.036)};
		double height{0.0};
		REQUIRE(road_crossing::surface_at(laid, 3.5, {head.x, head.z}, height));
		CHECK(std::abs(height - head.y) <= 1e-6);
	}
	std::vector<std::size_t> cut;
	CHECK(road_crossing::gaps(laid, 3.5, {rail}, &cut).size() == 2);
	CHECK(cut == std::vector<std::size_t>{0});
	// 1.5 mm off either way still is, 5 mm off isn't
	CHECK(road_crossing::gaps(road_in(plane, -10.0, 10.0, 50.0, 0.0015), 3.5, {rail}).size() == 2);
	CHECK(road_crossing::gaps(road_in(plane, -10.0, 10.0, 50.0, -0.0015), 3.5, {rail}).size() == 2);
	CHECK(road_crossing::gaps(road_in(plane, -10.0, 10.0, 50.0, 0.005), 3.5, {rail}).empty());
	CHECK(road_crossing::gaps(road_in(plane, -10.0, 10.0, 50.0, -0.005), 3.5, {rail}).empty());
	// the roads of the sceneries before: one under the rails, and a flat one through the axis of the canted track, its heads 27 mm off it
	CHECK(road_crossing::gaps(road(-10.0, 10.0, 50.0, -0.3, -0.3), 3.5, {rail}).empty());
	CHECK(road_crossing::gaps(road(-10.0, 10.0, 50.0, rail.point.y, rail.point.y), 3.5, {rail}).empty());
	// a road passing by, not crossing the track
	CHECK(road_crossing::gaps(road_in(plane, 2.0, 20.0, 50.0), 3.5, {rail}).empty());
	// of the two tracks, only the one it lies on
	auto const level{crossed(*flat, 50.0)};
	cut.clear();
	CHECK(road_crossing::gaps(laid, 3.5, {level, rail}, &cut).size() == 2);
	CHECK(cut == std::vector<std::size_t>{1});
	// a switch: none
	auto turnout{rail};
	turnout.turnout = true;
	CHECK(road_crossing::gaps(laid, 3.5, {turnout}).empty());
}

TEST_CASE("road crossing: the gaps along the rails, the flangeway inside the gauge face and the head outside it, across the whole road") {
	test::track_reading reading;
	auto const flat{test::read_track(straight_track("f", 10.0, 0.0, 100.0, 0.0, 0.0, 0.0))};
	auto const level{crossed(*flat, 50.0)};
	auto const gaps{road_crossing::gaps(road_in(road_crossing::plane_of(level), 0.0, 20.0, 50.0), 3.5, {level})};
	REQUIRE(gaps.size() == 2);
	std::vector<double> lows, highs;
	for (auto const &gap : gaps) {
		double low{1e9}, high{-1e9}, south{1e9}, north{-1e9};
		for (auto const &corner : gap) {
			low = std::min(low, corner.x);
			high = std::max(high, corner.x);
			south = std::min(south, corner.y);
			north = std::max(north, corner.y);
		}
		// the top of the groove of 67 mm of the flangeway on a straight (PLK, STWiORB D.11.01.01), and the head
		// the surface reaches half a millimetre under the field side of the head
		CHECK(high - low == doctest::Approx(road_crossing::groove_top_width(0.067) + level.head - 0.0005).epsilon(1e-6));
		CHECK(south <= 50.0 - 3.5);
		CHECK(north >= 50.0 + 3.5);
		lows.push_back(low);
		highs.push_back(high);
	}
	std::sort(lows.begin(), lows.end());
	std::sort(highs.begin(), highs.end());
	// the right rail of a track going along +z lies at -x: its head out to the field side, its flangeway in to the axis
	CHECK(lows[0] == doctest::Approx(10.0 - 0.7175 - level.head + 0.0005).epsilon(1e-6));
	CHECK(highs[0] == doctest::Approx(10.0 - 0.7175 + road_crossing::groove_top_width(0.067)).epsilon(1e-6));
	CHECK(lows[1] == doctest::Approx(10.0 + 0.7175 - road_crossing::groove_top_width(0.067)).epsilon(1e-6));
	CHECK(highs[1] == doctest::Approx(10.0 + 0.7175 + level.head - 0.0005).epsilon(1e-6));
	// the flangeway of a tight curve is wider: 80 mm under R 250 m
	auto tight{level};
	tight.radius = 200.0;
	auto const wide{road_crossing::gaps(road_in(road_crossing::plane_of(tight), 0.0, 20.0, 50.0), 3.5, {tight})};
	REQUIRE(wide.size() == 2);
	double low{1e9}, high{-1e9};
	for (auto const &corner : wide.front()) {
		low = std::min(low, corner.x);
		high = std::max(high, corner.x);
	}
	CHECK(high - low == doctest::Approx(road_crossing::groove_top_width(0.080) + level.head - 0.0005).epsilon(1e-6));
}

TEST_CASE("road crossing: the gaps left out of the triangles of a surface, what's left covering the rest of it") {
	// a square 4 by 4 m of two triangles, a band 0.2 m wide across it
	std::vector<world_vertex> vertices{
	    {{0.0, 1.0, 0.0}, {0.f, 1.f, 0.f}, {0.f, 0.f}}, {{0.0, 1.0, 4.0}, {0.f, 1.f, 0.f}, {0.f, 1.f}}, {{4.0, 1.0, 4.0}, {0.f, 1.f, 0.f}, {1.f, 1.f}},
	    {{0.0, 1.0, 0.0}, {0.f, 1.f, 0.f}, {0.f, 0.f}}, {{4.0, 1.0, 4.0}, {0.f, 1.f, 0.f}, {1.f, 1.f}}, {{4.0, 1.0, 0.0}, {0.f, 1.f, 0.f}, {1.f, 0.f}}};
	std::vector<std::vector<glm::dvec2>> const gaps{{{1.9, -1.0}, {2.1, -1.0}, {2.1, 5.0}, {1.9, 5.0}}};
	auto const facing{glm::cross(vertices[1].position - vertices[0].position, vertices[2].position - vertices[0].position).y};
	road_crossing::cut_gaps(vertices, gaps);
	REQUIRE(vertices.size() % 3 == 0);
	double area{0.0};
	for (std::size_t i = 0; i < vertices.size(); i += 3) {
		auto const &a{vertices[i].position};
		auto const &b{vertices[i + 1].position};
		auto const &c{vertices[i + 2].position};
		auto const normal{glm::cross(b - a, c - a)};
		area += 0.5 * glm::length(normal);
		// wound the same way as before
		CHECK(normal.y * facing > 0.0);
		// none inside the gap, all on the plane, the texture laid as before
		auto const middle{(a + b + c) / 3.0};
		CHECK((middle.x < 1.9 + 1e-9 || middle.x > 2.1 - 1e-9));
		for (int k = 0; k < 3; ++k) {
			CHECK(vertices[i + k].position.y == doctest::Approx(1.0));
			CHECK(vertices[i + k].texture.x == doctest::Approx(vertices[i + k].position.x / 4.0).epsilon(1e-5));
		}
	}
	CHECK(area == doctest::Approx(16.0 - 0.8).epsilon(1e-9));
	// a wall stays whole
	std::vector<world_vertex> wall{{{2.0, 0.0, 0.0}, {1.f, 0.f, 0.f}, {0.f, 0.f}}, {{2.0, 1.0, 0.0}, {1.f, 0.f, 0.f}, {0.f, 1.f}}, {{2.0, 0.0, 4.0}, {1.f, 0.f, 0.f}, {1.f, 0.f}}};
	road_crossing::cut_gaps(wall, gaps);
	CHECK(wall.size() == 3);
}

TEST_CASE("road crossing: a piece the rails pass under is cut along them, though its axis meets the track on the piece next to it") {
	test::track_reading reading;
	auto const flat{test::read_track(straight_track("f", 10.0, 0.0, 100.0, 0.0, 0.0, 0.0))};
	auto const level{crossed(*flat, 50.0)};
	auto const plane{road_crossing::plane_of(level)};
	// the road along x ends at x 9.5, past the right rail (x 9.28) and short of the axis of the track (x 10)
	auto const piece{road_in(plane, 0.0, 9.5, 50.0)};
	CHECK(road_crossing::crossing_parameter(piece, level) < 0.0);
	// cut along the rail it lies over, the right one
	auto const gaps{road_crossing::gaps(piece, 3.5, {level})};
	REQUIRE(gaps.size() == 1);
	for (auto const &corner : gaps.front())
		CHECK(corner.x < 10.0);
	// and the piece past it, from the axis of the track on, along the left one
	auto const past{road_crossing::gaps(road_in(plane, 9.5, 20.0, 50.0), 3.5, {level})};
	REQUIRE(past.size() == 1);
	for (auto const &corner : past.front())
		CHECK(corner.x > 10.0);
	// a piece the rails don't pass under: none
	CHECK(road_crossing::gaps(road_in(plane, 0.0, 8.5, 50.0), 3.5, {level}).empty());
}

TEST_CASE("road crossing: the gaps along the rails of a curve follow the rails, within a millimetre over the whole road") {
	test::track_reading reading;
	// 30 m of R 300 m turning right from the origin along +z: its end at the angle 0.1 rad, the control vectors 4/3 tan(0.025) R long
	auto const angle{0.1};
	auto const radius{300.0};
	auto const control{4.0 / 3.0 * std::tan(angle / 4.0) * radius};
	glm::dvec3 const end{-radius + radius * std::cos(angle), 0.0, radius * std::sin(angle)};
	glm::dvec3 const direction{-std::sin(angle), 0.0, std::cos(angle)};
	auto const text{"node -1 0 arc track normal 30 1.435 0.15 25.0 20 0 flat vis\n rail_screw_used1 6 1435mm/tpbps-new2 0.2 0.5 1.1\n0 0 0 0\n0 0 " + std::to_string(control) +
	                "\n" + std::to_string(-direction.x * control) + " 0 " + std::to_string(-direction.z * control) + "\n" + std::to_string(end.x) + " 0 " + std::to_string(end.z) +
	                " 0\n" + std::to_string(radius) + "\nendtrack"};
	auto const arc{test::read_track(text)};
	auto const length{arc->CurrentSegment()->GetLength()};
	auto rail{crossed(*arc, 0.5 * length)};
	rail.radius = radius;
	CHECK(rail.curvature == doctest::Approx(1.0 / radius).epsilon(0.05));
	// a road square to the track there, laid into the plane of its heads
	glm::dvec2 const across{rail.right()};
	glm::dvec2 const middle{rail.point.x, rail.point.z};
	segment_data axis;
	axis.points[segment_data::point::start] = {middle.x - across.x * 10.0, rail.point.y, middle.y - across.y * 10.0};
	axis.points[segment_data::point::end] = {middle.x + across.x * 10.0, rail.point.y, middle.y + across.y * 10.0};
	auto const gaps{road_crossing::gaps(axis, 4.0, {rail})};
	REQUIRE(gaps.size() > 2);
	// the middle of each head, along the rails as far as the road reaches, lies in a gap
	auto const &segment{*arc->CurrentSegment()};
	for (double along = -3.9; along <= 3.9; along += 0.3)
		for (auto const side : {-1.0, 1.0}) {
			glm::dvec3 point, right;
			segment.plan_frame(0.5 * length + along, point, right);
			auto const head{glm::dvec2{point.x, point.z} + glm::dvec2{right.x, right.z} * (side * (0.7175 + 0.036))};
			auto const inside{std::any_of(gaps.begin(), gaps.end(), [&](auto const &Gap) { return road_crossing_inside(Gap, head); })};
			CAPTURE(along);
			CAPTURE(side);
			CHECK(inside);
		}
}

TEST_CASE("road crossing: the flangeway a groove, its wall sloping down from the surface to a closed bottom 40 mm under the head") {
	test::track_reading reading;
	auto const flat{test::read_track(straight_track("f", 10.0, 0.0, 100.0, 0.0, 0.0, 0.0))};
	auto const level{crossed(*flat, 50.0)};
	auto const top{level.point.y};
	auto const result{road_crossing::cut(road_in(road_crossing::plane_of(level), 0.0, 20.0, 50.0), 3.5, {level})};
	REQUIRE(result.tracks.size() == 1);
	REQUIRE(false == result.grooves.empty());
	REQUIRE(result.grooves.size() % 3 == 0);
	// at least 38 mm deep (PLK, STWiORB D.11.01.01), the bottom flat
	double lowest{1e9}, highest{-1e9};
	for (auto const &vertex : result.grooves) {
		lowest = std::min(lowest, vertex.position.y);
		highest = std::max(highest, vertex.position.y);
	}
	CHECK(top - lowest == doctest::Approx(road_crossing::groove_depth).epsilon(1e-6));
	CHECK(road_crossing::groove_depth >= 0.038);
	CHECK(highest == doctest::Approx(top).epsilon(1e-9));
	// across the right rail (x 9.2825 its gauge face, the axis of the track at x 10): the edge at the surface, the foot of the wall, the
	// bottom running under the head
	std::vector<double> edges, feet, unders;
	// the ends closed, the faces across the groove at either end
	int ends{0};
	for (std::size_t i = 0; i < result.grooves.size(); i += 3) {
		auto const normal{glm::cross(result.grooves[i + 1].position - result.grooves[i].position, result.grooves[i + 2].position - result.grooves[i].position)};
		if (std::abs(normal.y) < 1e-9 && std::abs(normal.x) < 1e-9)
			++ends;
	}
	CHECK(ends >= 4);
	for (std::size_t i = 0; i + 2 < result.grooves.size(); i += 3) {
		auto const normal{glm::cross(result.grooves[i + 1].position - result.grooves[i].position, result.grooves[i + 2].position - result.grooves[i].position)};
		// the ends aside
		if (std::abs(normal.y) < 1e-9 && std::abs(normal.x) < 1e-9)
			continue;
		for (int k = 0; k < 3; ++k) {
			auto const &vertex{result.grooves[i + k]};
			if (vertex.position.x > 10.0)
				continue;
			auto const lateral{10.0 - vertex.position.x};
			if (std::abs(vertex.position.y - top) < 1e-9)
				edges.push_back(lateral);
			else
				(lateral > 0.7175 - 0.001 ? unders : feet).push_back(lateral);
		}
	}
	REQUIRE(false == edges.empty());
	REQUIRE(false == feet.empty());
	REQUIRE(false == unders.empty());
	auto const width{road_crossing::groove_top_width(0.067)};
	for (auto const lateral : edges)
		CHECK(lateral == doctest::Approx(0.7175 - width).epsilon(1e-6));
	for (auto const lateral : feet)
		CHECK(lateral == doctest::Approx(0.7175 - width + road_crossing::groove_run).epsilon(1e-6));
	for (auto const lateral : unders)
		CHECK(lateral == doctest::Approx(0.7175 + road_crossing::groove_tuck).epsilon(1e-6));
	// 14 mm under the top the groove is as wide as the flangeway
	auto const wall{0.7175 - width + road_crossing::groove_run * 0.014 / road_crossing::groove_depth};
	CHECK(0.7175 - wall == doctest::Approx(0.067).epsilon(1e-6));
	// the triangles face up: the wall towards the rail, the bottom straight up; none of them sideways down; the ends upright
	for (std::size_t i = 0; i < result.grooves.size(); i += 3) {
		auto const &a{result.grooves[i].position};
		auto const &b{result.grooves[i + 1].position};
		auto const &c{result.grooves[i + 2].position};
		CHECK(glm::cross(b - a, c - a).y > -1e-9);
	}
	// along the rail as far as the road reaches over it, 3.5 m either way
	double south{1e9}, north{-1e9};
	for (auto const &vertex : result.grooves) {
		south = std::min(south, vertex.position.z);
		north = std::max(north, vertex.position.z);
	}
	CHECK(south == doctest::Approx(50.0 - 3.5).epsilon(0.002));
	CHECK(north == doctest::Approx(50.0 + 3.5).epsilon(0.002));
}

namespace {

// the highest of the triangles over the point of the plan, as a question for the ground asks for it; false: none is
bool highest_at(std::vector<world_vertex> const &Vertices, double const X, double const Z, double &Height) {
	bool found{false};
	for (std::size_t i = 0; i + 2 < Vertices.size(); i += 3) {
		auto const &a{Vertices[i].position};
		auto const &b{Vertices[i + 1].position};
		auto const &c{Vertices[i + 2].position};
		auto const d{(b.z - c.z) * (a.x - c.x) + (c.x - b.x) * (a.z - c.z)};
		if (std::abs(d) < 1e-12)
			continue;
		auto const u{((b.z - c.z) * (X - c.x) + (c.x - b.x) * (Z - c.z)) / d};
		auto const v{((c.z - a.z) * (X - c.x) + (a.x - c.x) * (Z - c.z)) / d};
		auto const w{1.0 - u - v};
		if (u < -1e-9 || v < -1e-9 || w < -1e-9)
			continue;
		auto const y{u * a.y + v * b.y + w * c.y};
		if (false == found || y > Height)
			Height = y;
		found = true;
	}
	return found;
}

} // namespace

TEST_CASE("road crossing: the height a wheel or a foot gets across the rails has no dip at the grooves, every 5 mm across each rail") {
	test::track_reading reading;
	// a track canted 60 mm on a grade of 5 per mille, a road across it in the plane of its heads
	auto const canted{test::read_track(straight_track("c", 10.0, 0.0, 100.0, 0.0, 0.5, 60.0))};
	auto const rail{crossed(*canted, 50.0)};
	auto const plane{road_crossing::plane_of(rail)};
	auto const axis{road_in(plane, 0.0, 20.0, 50.0)};
	// across each rail: from inside the flangeway to past the field side of the head, every 5 mm
	std::vector<double> across;
	for (int side : {-1, 1})
		for (double offset = 0.55; offset <= 0.95 + 1e-9; offset += 0.005)
			across.push_back(10.0 + side * offset);
	// the vehicles of the road go along its lanes: invisible road tracks the road makes of its axis as it loads (road_node::create_lanes,
	// offset_path), their ends where the surface is, the left of the road and its right. The grooves aren't a part of them
	for (double const offset : {-1.75, 0.0, 1.75}) {
		// the left of a road going along +x is -z
		auto const z{50.0 - offset};
		auto const text{"node -1 0 lane track road 20 3.5 0.2 25 20 0 flat unvis 0 " + std::to_string(plane.at({0.0, z})) + " " + std::to_string(z) + " 0 0 0 0 0 0 0 20 " +
		                std::to_string(plane.at({20.0, z})) + " " + std::to_string(z) + " 0 0 endtrack"};
		auto const lane{test::read_track(text)};
		auto const path{lane->CurrentSegment()};
		REQUIRE(path != nullptr);
		// the lane rises along the road: its length is longer than in the plan
		auto const rise{plane.at({20.0, z}) - plane.at({0.0, z})};
		auto const stretch{std::sqrt(1.0 + rise * rise / 400.0)};
		for (auto const x : across) {
			glm::dvec3 point, sideways;
			path->plan_frame(x * stretch, point, sideways);
			CHECK(point.x == doctest::Approx(x).epsilon(1e-4));
			// on the plane of the heads within 1 mm: on the head over the rail, over the groove as high as the surface by it
			CHECK(std::abs(point.y - plane.at({point.x, point.z})) <= 0.001);
		}
	}
	// what asks the geometry of the road for the height (the editor putting things on the ground): the surface of the road, flat in the
	// plane, as drawn with the gaps cut and the grooves in them, and the surface over the gaps the road keeps for that (road_node::bridges)
	std::vector<world_vertex> surface;
	for (auto const &corner : std::vector<glm::dvec2>{{0.0, 46.5}, {0.0, 53.5}, {20.0, 53.5}, {0.0, 46.5}, {20.0, 53.5}, {20.0, 46.5}})
		surface.push_back({{corner.x, plane.at(corner), corner.y}, {0.f, 1.f, 0.f}, {0.f, 0.f}});
	auto const cut{road_crossing::cut(axis, 3.5, {rail})};
	REQUIRE(cut.holes.size() == 2);
	REQUIRE(false == cut.grooves.empty());
	std::vector<world_vertex> bridges;
	road_crossing::cut_gaps(surface, cut.holes, &bridges);
	REQUIRE(false == bridges.empty());
	auto drawn{surface};
	drawn.insert(drawn.end(), cut.grooves.begin(), cut.grooves.end());
	auto ground{drawn};
	ground.insert(ground.end(), bridges.begin(), bridges.end());
	double deepest{0.0}, dip{0.0};
	for (double const z : {47.0, 50.0, 53.0})
		for (auto const x : across) {
			auto const top{plane.at({x, z})};
			// as drawn the grooves go down beside each rail
			if (double height; highest_at(drawn, x, z, height))
				deepest = std::max(deepest, top - height);
			// as the ground: the surface all the way across, no dip at the grooves, nothing over it
			double height{0.0};
			REQUIRE(highest_at(ground, x, z, height));
			dip = std::max(dip, std::abs(top - height));
		}
	CHECK(deepest == doctest::Approx(road_crossing::groove_depth).epsilon(0.05));
	CHECK(dip <= 0.001);
	// the bridges lie in the gaps alone, on the plane
	for (auto const &vertex : bridges)
		CHECK(std::abs(vertex.position.y - plane.at({vertex.position.x, vertex.position.z})) <= 1e-6);
}
