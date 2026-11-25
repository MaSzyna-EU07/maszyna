// a level crossing as built checked for what shows through it (road_crossing::check): the road laid into the plane of the heads by the
// editor (editor/editorCrossing.cpp), each of its pieces built as road_node::create_shapes builds it (road_crossing::build): cut along the
// rails with the grooves closed, set onto the surface of the crossing where it follows it, the panels CBP laid along the tracks
// (world/CrossingPanels.cpp) with the road left out under their tops, the outer panels between two tracks bent onto the line of the
// road, the corners welded: rays straight down every 2 cm, beside the edges of the road, at slants, down the flangeways. On a single
// track, a double track with its heads in one plane, a crossing at 60 degrees, a crossing on a curve, a double track 5.5 m apart crossed
// at a skew with its tracks canted as the regulation lets them be; the profile between two tracks without a hump, the lanes on the
// surface of the crossing; the break of the grade line over 5% refused, with its two fixes

#include "stdafx.h"
#include "doctest.h"
#include "editor/editorCrossing.hpp"
#include "editor/norms/norms.hpp"
#include "world/CrossingPanels.h"
#include "track_text.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace {

double roll_of(double const Cant) {
	return glm::degrees(std::asin(Cant / 1500.0));
}

// a straight track along z at X from Z0 to Z1, level, its axis at Y, canted
std::string straight(std::string const &Name, double const X, double const Cant, double const Y = 0.0) {
	auto const roll{std::to_string(roll_of(Cant))};
	return "node -1 0 " + Name + " track normal 100 1.435 0.15 25.0 20 0 flat vis\n rail_screw_used1 6 1435mm/tpbps-new2 0.2 0.5 1.1\n" + std::to_string(X) + " " + std::to_string(Y) + " 0 " + roll +
	       "\n0 0 0\n0 0 0\n" + std::to_string(X) + " " + std::to_string(Y) + " 100 " + roll + "\n0\nendtrack";
}

// an arc of R 600 m, 100 m long, from the origin along z bending towards +x, canted
std::string curve(std::string const &Name, double const Cant) {
	auto const roll{std::to_string(roll_of(Cant))};
	auto const angle{100.0 / 600.0};
	auto const control{4.0 / 3.0 * std::tan(angle / 4.0) * 600.0};
	glm::dvec2 const end{600.0 * (1.0 - std::cos(angle)), 600.0 * std::sin(angle)};
	glm::dvec2 const tangent{std::sin(angle), std::cos(angle)};
	return "node -1 0 " + Name + " track normal 100 1.435 0.15 25.0 20 0 flat vis\n rail_screw_used1 6 1435mm/tpbps-new2 0.2 0.5 1.1\n0 0 0 " + roll + "\n0 0 " + std::to_string(control) + "\n" +
	       std::to_string(-tangent.x * control) + " 0 " + std::to_string(-tangent.y * control) + "\n" + std::to_string(end.x) + " 0 " + std::to_string(end.y) + " " + roll + "\n600\nendtrack";
}

// an arc of Radius, 300 m long, from X along z bending towards +x (its centre at X + Radius), canted, its axis at Y: two of them Radius
// and Radius + 5.5 from X and X - 5.5 lie side by side 5.5 m apart, as the tracks sp and sq of the editor's cases
std::string arc(std::string const &Name, double const X, double const Radius, double const Cant, double const Y = 0.0) {
	auto const roll{std::to_string(roll_of(Cant))};
	auto const angle{300.0 / Radius};
	auto const control{4.0 / 3.0 * std::tan(angle / 4.0) * Radius};
	glm::dvec2 const end{X + Radius * (1.0 - std::cos(angle)), Radius * std::sin(angle)};
	glm::dvec2 const tangent{std::sin(angle), std::cos(angle)};
	auto const number = [](double const Value) {
		char text[32];
		std::snprintf(text, sizeof(text), "%.9f", Value);
		return std::string{text};
	};
	return "node -1 0 " + Name + " track normal 300 1.435 0.15 25.0 20 0 flat vis\n rail_screw_used1 6 1435mm/tpbps-new2 0.2 0.5 1.1\n" + number(X) + " " + number(Y) + " 0 " + roll + "\n0 0 " +
	       number(control) + "\n" + number(-tangent.x * control) + " 0 " + number(-tangent.y * control) + "\n" + number(end.x) + " " + number(Y) + " " + number(end.y) + " " + roll + "\n" +
	       number(Radius) + "\nendtrack";
}

// how far along an arc made by arc() the line through Point going Direction crosses it, the crossing nearest the point
double along_arc(double const X, double const Radius, glm::dvec2 const &Point, glm::dvec2 const &Direction) {
	glm::dvec2 const centre{X + Radius, 0.0};
	auto const offset{Point - centre};
	// |offset + direction s| = radius
	auto const b{glm::dot(offset, Direction)};
	auto const c{glm::dot(offset, offset) - Radius * Radius};
	auto const root{std::sqrt(std::max(0.0, b * b - c))};
	auto const s{std::abs(-b - root) < std::abs(-b + root) ? -b - root : -b + root};
	auto const crossing{Point + Direction * s};
	return Radius * std::atan2(crossing.y, centre.x - crossing.x);
}

level_crossing::rail_track crossed(TTrack const &Track, double const Distance) {
	auto rail{level_crossing::track_at(*Track.CurrentSegment(), Distance)};
	rail.name = Track.name();
	rail.source = &Track;
	rail.gauge = std::abs(Track.fTrackWidth);
	rail.head = 0.072;
	if (rail.radius <= 0.0 && std::abs(rail.curvature) > 1e-6)
		rail.radius = 1.0 / std::abs(rail.curvature);
	return rail;
}

level_crossing::road_piece road(glm::dvec2 const &From, glm::dvec2 const &To, double const Height) {
	level_crossing::road_piece piece;
	piece.axis.points[segment_data::point::start] = {From.x, Height, From.y};
	piece.axis.points[segment_data::point::end] = {To.x, Height, To.y};
	piece.halfwidth = 3.5;
	return piece;
}

glm::dvec2 plan(glm::dvec3 const &Point) {
	return {Point.x, Point.z};
}

// the rows of panels CBP of a track as the button of the editor lays them, placed as a sweep bends them along it, their origin at the
// head (anchored at the origin of the model): the inner row centred, the outer rows from outer_gap past the field side of each head,
// bent onto the line to the track next to them. Their triangles, and the panels for the road to leave its surface out under
void lay_panels(level_crossing::rail_track const &Rail, std::vector<level_crossing::rail_track> const &Rails, glm::dvec2 const &Road, std::vector<std::array<glm::dvec3, 3>> &Triangles,
                std::vector<road_crossing::panel> &Panels) {
	auto const sine{std::max(0.2, std::abs(glm::dot(glm::normalize(Road), Rail.right())))};
	auto const reach{3.5 / sine + 0.25};
	auto const count{std::clamp(static_cast<int>(std::ceil((2.0 * reach + 2.0 * (0.5 * crossing_panels::sleeper_spacing + crossing_panels::corner_along)) / crossing_panels::length - 1e-6)), 2,
	                            crossing_panels::most)};
	auto const start{-0.5 * count * crossing_panels::length};
	auto const roll{Rail.roll};
	// placed as a sweep places a model, not turned inside out: the right of a track (rail_track::right) and the x of a model make a mirror
	// image, so a row whose x goes to the right of the track goes along it the other way
	auto const span{0.0 - 2.0 * start};
	auto const place = [&](glm::dvec3 const &Model, double const Lateral, int const Side) {
		glm::dvec2 right;
		auto const along{start + (Side > 0 ? span - Model.z : Model.z)};
		auto const centre{Rail.axis_at(along, right)};
		auto const across{Lateral + Side * Model.x};
		auto const height{Rail.point.y + Rail.grade * along};
		return glm::dvec3{centre.x + right.x * (across * std::cos(roll) + Model.y * std::sin(roll)), height - across * std::sin(roll) + Model.y * std::cos(roll),
		                  centre.y + right.y * (across * std::cos(roll) + Model.y * std::sin(roll))};
	};
	auto const row = [&](crossing_panels::spec const &Spec, double const Lateral, int const Side) {
		auto const vertices{crossing_panels::geometry(Spec)};
		road_crossing::panel panel;
		std::vector<glm::dvec2> points;
		panel.top = -1e9;
		for (std::size_t i = 0; i + 2 < vertices.size(); i += 3) {
			std::array<glm::dvec3, 3> const model{vertices[i].position, vertices[i + 1].position, vertices[i + 2].position};
			std::array<glm::dvec3, 3> const world{place(model[0], Lateral, Side), place(model[1], Lateral, Side), place(model[2], Lateral, Side)};
			Triangles.push_back(world);
			for (auto const &corner : world) {
				points.emplace_back(corner.x, corner.z);
				panel.top = std::max(panel.top, corner.y);
			}
			// its top: the faces up in the space of the model
			auto const normal{glm::cross(model[1] - model[0], model[2] - model[0])};
			if (normal.y > 0.7 * glm::length(normal))
				panel.faces.push_back(world);
		}
		panel.outline = road_crossing::hull(points);
		Panels.push_back(std::move(panel));
	};
	row({crossing_panels::piece::inner_row, 180, 1435, count}, 0.0, 1);
	auto const outside{0.5 * Rail.gauge + Rail.head + crossing_panels::outer_gap};
	for (auto const side : {1, -1}) {
		crossing_panels::spec outer{crossing_panels::piece::outer_row, 180, 1435, count};
		outer.slope = level_crossing::outer_slope(Rail, side, Rails, Road, 3.5);
		row(outer, side * outside, side);
	}
}

struct crossing_built {
	road_crossing::built_crossing built;
	level_crossing::design_result design;
	glm::dvec2 low, high;
	glm::dvec2 direction;
	// the triangles of the surface of each piece of the road, in the order of the pieces along it
	std::vector<std::vector<std::array<glm::dvec3, 3>>> surfaces;
};

// the surface of a straight road piece as the road lays it: a grid of 10 cm along it and across it, at the heights of the piece
std::vector<world_vertex> piece_grid(segment_data const &Axis, double const Halfwidth) {
	std::vector<world_vertex> result;
	auto const from{plan(Axis.points[segment_data::point::start])}, to{plan(Axis.points[segment_data::point::end])};
	auto const length{glm::length(to - from)};
	if (length < 1e-6)
		return result;
	auto const direction{(to - from) / length};
	glm::dvec2 const left{direction.y, -direction.x};
	auto const along{std::max(1, static_cast<int>(std::ceil(length / 0.1 - 1e-9)))};
	auto const across{static_cast<int>(std::ceil(2.0 * Halfwidth / 0.1 - 1e-9))};
	auto const vertex = [&](int const I, int const J) {
		auto const point{from + direction * (length * I / along) + left * (-Halfwidth + 2.0 * Halfwidth * J / across)};
		double y{0.0};
		road_crossing::surface_at(Axis, Halfwidth + 1e-6, point, y);
		return world_vertex{{point.x, y, point.y}, {0.f, 1.f, 0.f}, {0.f, 0.f}};
	};
	for (int i = 0; i < along; ++i)
		for (int j = 0; j < across; ++j) {
			auto const a{vertex(i, j)}, b{vertex(i + 1, j)}, c{vertex(i + 1, j + 1)}, d{vertex(i, j + 1)};
			result.insert(result.end(), {a, b, c, a, c, d});
		}
	return result;
}

// the surface of a straight road piece as the road lays it itself (road_node::create_shapes, loft): one band from the one edge to the
// other, cut across every 10 m along it, at the heights of the piece; an end of it a single edge across the road
std::vector<world_vertex> piece_band(segment_data const &Axis, double const Halfwidth) {
	std::vector<world_vertex> result;
	auto const from{plan(Axis.points[segment_data::point::start])}, to{plan(Axis.points[segment_data::point::end])};
	auto const length{glm::length(to - from)};
	if (length < 1e-6)
		return result;
	auto const direction{(to - from) / length};
	glm::dvec2 const left{direction.y, -direction.x};
	auto const along{std::max(1, static_cast<int>(std::ceil(length / 10.0 - 0.001)))};
	auto const vertex = [&](int const I, double const Side) {
		auto const point{from + direction * (length * I / along) + left * (Side * Halfwidth)};
		double y{0.0};
		road_crossing::surface_at(Axis, Halfwidth + 1e-6, point, y);
		return world_vertex{{point.x, y, point.y}, {0.f, 1.f, 0.f}, {0.f, 0.f}};
	};
	for (int i = 0; i < along; ++i) {
		auto const a{vertex(i, 1.0)}, b{vertex(i, -1.0)}, c{vertex(i + 1, 1.0)}, d{vertex(i + 1, -1.0)};
		result.insert(result.end(), {a, b, c, b, d, c});
	}
	return result;
}

// the road from From to To at Height laid over the tracks as the editor lays it, with the panels CBP on each, and each of its pieces
// built as the road builds itself (road_crossing::build): cut along the rails, onto the surface of the crossing where it follows it,
// under the panels, the grooves and the strips beside the rails past the road, the corners welded. Band: the surface of a piece laid as
// the road lays it, one band across it (piece_band), not a grid of 10 cm
crossing_built build(std::vector<level_crossing::rail_track> const &Rails, glm::dvec2 const &From, glm::dvec2 const &To, double const Height, bool const Panels, bool const Band = false) {
	crossing_built result;
	auto const original{road(From, To, Height)};
	auto const direction{glm::normalize(To - From)};
	result.direction = direction;
	level_crossing::design_input input;
	input.pieces = {original};
	input.tracks = Rails;
	std::vector<std::array<glm::dvec3, 3>> panels;
	std::vector<road_crossing::panel> lying;
	if (Panels)
		for (auto const &rail : Rails)
			lay_panels(rail, Rails, direction, panels, lying);
	input.panels = lying;
	result.design = level_crossing::design(input);
	INFO(result.design.reason);
	REQUIRE(result.design.valid);
	// the road as laid: the parts of the road before and past the stretch laid anew, the pieces laid
	std::vector<std::pair<segment_data, double>> pieces;
	auto const &first{result.design.pieces.front()};
	auto const &last{result.design.pieces.back()};
	auto const at = [&](double const T) { return glm::mix(From, To, T); };
	if (first.from > 1e-9)
		pieces.emplace_back(road(From, at(first.from), Height).axis, 3.5);
	for (auto const &piece : result.design.pieces)
		pieces.emplace_back(piece.axis, 3.5);
	if (last.to < 1.0 - 1e-9)
		pieces.emplace_back(road(at(last.to), To, Height).axis, 3.5);
	result.built.footprint = pieces;
	result.built.tracks = Rails;
	result.built.panels = panels;
	// each piece as it builds itself
	auto const crossing{road_crossing::surface_of(Rails, direction, 3.5)};
	for (auto const &[axis, halfwidth] : pieces) {
		auto surface{Band ? piece_band(axis, halfwidth) : piece_grid(axis, halfwidth)};
		auto const follows{road_crossing::follows(axis, crossing)};
		auto built{road_crossing::build({&surface}, axis, halfwidth, Rails, 4.f, lying, follows ? &crossing : nullptr)};
		road_crossing::weld(surface);
		result.surfaces.emplace_back();
		for (std::size_t i = 0; i + 2 < surface.size(); i += 3)
			result.surfaces.back().push_back({surface[i].position, surface[i + 1].position, surface[i + 2].position});
		surface.insert(surface.end(), built.grooves.begin(), built.grooves.end());
		surface.insert(surface.end(), built.fills.begin(), built.fills.end());
		road_crossing::weld(surface);
		for (std::size_t i = 0; i + 2 < surface.size(); i += 3)
			result.built.road.push_back({surface[i].position, surface[i + 1].position, surface[i + 2].position});
	}
	for (auto const &rail : Rails)
		for (auto const &triangle : road_crossing::rail_heads(rail, -14.0, 14.0))
			result.built.rails.push_back(triangle);
	// the crossing and 6 m on either side of it along the road
	result.low = glm::dvec2{1e9};
	result.high = glm::dvec2{-1e9};
	for (auto const &rail : Rails) {
		for (auto const sign : {-1.0, 1.0}) {
			auto const point{glm::dvec2{rail.point.x, rail.point.z} + direction * (sign * 5.0)};
			result.low = glm::min(result.low, point - glm::dvec2{4.5});
			result.high = glm::max(result.high, point + glm::dvec2{4.5});
		}
	}
	return result;
}

void watertight(crossing_built const &Crossing) {
	std::size_t cracks{0};
	auto const defects{road_crossing::check(Crossing.built, Crossing.low, Crossing.high, 0.02, 20, &cracks)};
	MESSAGE("rays through cracks under 0.02 mm: " << cracks);
	for (auto const &entry : defects) {
		INFO(entry.kind << " at " << entry.point.x << " " << entry.point.y << " " << entry.point.z);
		CHECK(false);
	}
	CHECK(defects.empty());
}

// the top of what's there at a point of the plan: the road or a panel, the highest
bool top_at(crossing_built const &Crossing, glm::dvec2 const &Point, double &Y) {
	auto found{false};
	for (auto const *list : {&Crossing.built.road, &Crossing.built.panels})
		for (auto const &t : *list) {
			auto const d{(t[1].z - t[2].z) * (t[0].x - t[2].x) + (t[2].x - t[1].x) * (t[0].z - t[2].z)};
			if (std::abs(d) < 1e-14)
				continue;
			auto const u{((t[1].z - t[2].z) * (Point.x - t[2].x) + (t[2].x - t[1].x) * (Point.y - t[2].z)) / d};
			auto const v{((t[2].z - t[0].z) * (Point.x - t[2].x) + (t[0].x - t[2].x) * (Point.y - t[2].z)) / d};
			if (u < -1e-9 || v < -1e-9 || 1.0 - u - v < -1e-9)
				continue;
			auto const y{u * t[0].y + v * t[1].y + (1.0 - u - v) * t[2].y};
			if (false == found || y > Y)
				Y = y;
			found = true;
		}
	return found;
}

// between two tracks along z (A the one at the lower x), along lines of the road at the offsets from its axis every 5 cm from the field
// side of the head of the one to the head of the other: the top of what's there (the panels, the road between them) no higher than the
// line straight between them, and no step between neighbours over 2 mm
void no_hump(crossing_built const &Crossing, level_crossing::rail_track const &A, level_crossing::rail_track const &B, glm::dvec2 const &Middle) {
	auto const &direction{Crossing.direction};
	glm::dvec2 const left{direction.y, -direction.x};
	for (auto const offset : {-3.4, -1.75, 0.0, 1.75, 3.4}) {
		auto const origin{Middle + left * offset};
		// where the line passes the field sides of the heads facing each other
		auto const from{(A.point.x + road_crossing::band(A) - origin.x) / direction.x};
		auto const to{(B.point.x - road_crossing::band(B) - origin.x) / direction.x};
		auto const a{origin + direction * from}, b{origin + direction * to};
		auto const ha{level_crossing::plane_of(A).at(a)}, hb{level_crossing::plane_of(B).at(b)};
		auto const grade{(hb - ha) / (to - from)};
		double previous{0.0};
		auto first{true};
		for (double s = from + 0.01; s <= to - 0.01; s += 0.05) {
			CAPTURE(offset);
			CAPTURE(s);
			double y{0.0};
			REQUIRE(top_at(Crossing, origin + direction * s, y));
			CHECK(y <= ha + grade * (s - from) + 0.001);
			if (false == first)
				CHECK(std::abs(y - previous - grade * 0.05) <= 0.002);
			previous = y;
			first = false;
		}
	}
}

// the lanes of the pieces which follow the crossing, a lane each way 1.75 m off the axis, set onto the surface of the crossing at their
// ends as road_node::fit_lanes sets them: along them every 5 cm within 1 cm of it
void lanes_on_surface(crossing_built const &Crossing) {
	auto const surface{road_crossing::surface_of(Crossing.built.tracks, Crossing.direction, 3.5)};
	auto followed{0};
	double worst{0.0};
	for (auto const &[axis, halfwidth] : Crossing.built.footprint) {
		if (false == road_crossing::follows(axis, surface))
			continue;
		++followed;
		auto const &a{axis.points[segment_data::point::start]};
		auto const &b{axis.points[segment_data::point::end]};
		auto const along{glm::normalize(plan(b - a))};
		glm::dvec3 const left{along.y, 0.0, -along.x};
		for (auto const offset : {-1.75, 1.75}) {
			segment_data path;
			path.points[segment_data::point::start] = a + left * offset;
			path.points[segment_data::point::end] = b + left * offset;
			road_crossing::fit(path, surface);
			auto const p0{path.points[segment_data::point::start]};
			auto const p3{path.points[segment_data::point::end]};
			auto const p1{p0 + path.points[segment_data::point::control1]};
			auto const p2{p3 + path.points[segment_data::point::control2]};
			auto const count{std::max(2, static_cast<int>(std::ceil(glm::length(plan(p3 - p0)) / 0.05)))};
			for (int i = 0; i <= count; ++i) {
				auto const t{static_cast<double>(i) / count};
				auto const u{1.0 - t};
				auto const point{u * u * u * p0 + 3.0 * u * u * t * p1 + 3.0 * u * t * t * p2 + t * t * t * p3};
				double height{0.0};
				REQUIRE(surface.at(plan(point), height));
				worst = std::max(worst, std::abs(point.y - height));
			}
		}
	}
	CHECK(followed > 0);
	CHECK(worst <= 0.01);
}

// the pieces of the road meeting at their ends at the same heights: along each joint every centimetre across the road, where the
// surface of both pieces is there, the surface of the one and of the other at the joint within 0.02 mm (the corners are welded to a grid
// of 0.01 mm): the triangle of each a micrometre into it, taken to the joint. Returns the largest difference found
double joints_meet(crossing_built const &Crossing) {
	// the height at Point of the plane of the highest triangle lying flat over Inside
	auto const top = [](std::vector<std::array<glm::dvec3, 3>> const &Surface, glm::dvec2 const &Inside, glm::dvec2 const &Point, double &Y) {
		auto found{false};
		double highest{0.0};
		for (auto const &t : Surface) {
			auto const normal{glm::cross(t[1] - t[0], t[2] - t[0])};
			if (normal.y < 0.9 * glm::length(normal))
				continue;
			auto const d{(t[1].z - t[2].z) * (t[0].x - t[2].x) + (t[2].x - t[1].x) * (t[0].z - t[2].z)};
			if (std::abs(d) < 1e-14)
				continue;
			auto const weights = [&](glm::dvec2 const &P, double &U, double &V) {
				U = ((t[1].z - t[2].z) * (P.x - t[2].x) + (t[2].x - t[1].x) * (P.y - t[2].z)) / d;
				V = ((t[2].z - t[0].z) * (P.x - t[2].x) + (t[0].x - t[2].x) * (P.y - t[2].z)) / d;
			};
			double u{0.0}, v{0.0};
			weights(Inside, u, v);
			if (u < -1e-9 || v < -1e-9 || 1.0 - u - v < -1e-9)
				continue;
			auto const inside{u * t[0].y + v * t[1].y + (1.0 - u - v) * t[2].y};
			if (found && inside <= highest)
				continue;
			highest = inside;
			weights(Point, u, v);
			Y = u * t[0].y + v * t[1].y + (1.0 - u - v) * t[2].y;
			found = true;
		}
		return found;
	};
	auto const &pieces{Crossing.built.footprint};
	REQUIRE(pieces.size() == Crossing.surfaces.size());
	double worst{0.0};
	auto compared{0};
	for (std::size_t i = 0; i + 1 < pieces.size(); ++i) {
		auto const &axis{pieces[i].first};
		auto const joint{plan(axis.points[segment_data::point::end])};
		auto const along{glm::normalize(plan(axis.points[segment_data::point::end] - axis.points[segment_data::point::start]))};
		glm::dvec2 const left{along.y, -along.x};
		for (double s = -3.45; s <= 3.45; s += 0.01) {
			auto const point{joint + left * s};
			double a{0.0}, b{0.0};
			if (false == top(Crossing.surfaces[i], point - along * 0.000001, point, a) || false == top(Crossing.surfaces[i + 1], point + along * 0.000001, point, b))
				continue;
			CAPTURE(i);
			CAPTURE(s);
			CHECK(std::abs(a - b) <= 0.00002);
			worst = std::max(worst, std::abs(a - b));
			++compared;
		}
	}
	CHECK(compared > 100);
	return worst;
}

// the most cant the track norms allow (PLK ST-T1-A6 tab. 9.6, a main track), mm
double norms_cant() {
	norms::query where;
	auto const limits{norms::limits(norms::profile_id::plk, norms::parameter::cant, where)};
	REQUIRE(limits.block.has_value());
	return limits.block->value;
}

} // namespace

TEST_CASE("crossing watertight: the check finds a hole, a road over a panel and a flangeway filled") {
	test::track_reading reading;
	auto const track{test::read_track(straight("t", 0.0, 0.0))};
	auto const rail{crossed(*track, 50.0)};
	auto crossing{build({rail}, {-30.0, 50.0}, {30.0, 50.0}, 0.18, true)};
	// a piece of the road taken out: a hole
	auto damaged{crossing};
	damaged.built.road.erase(std::remove_if(damaged.built.road.begin(), damaged.built.road.end(),
	                                        [](std::array<glm::dvec3, 3> const &T) {
		                                        auto const middle{(T[0] + T[1] + T[2]) / 3.0};
		                                        return middle.x > 3.0 && middle.x < 3.3 && middle.z > 51.0 && middle.z < 51.3 && middle.y > 0.17;
	                                        }),
	                         damaged.built.road.end());
	auto const holes{road_crossing::check(damaged.built, damaged.low, damaged.high, 0.02, 1000)};
	CHECK(std::any_of(holes.begin(), holes.end(), [](road_crossing::defect const &D) { return D.kind == "hole"; }));
	// the road over a panel
	auto over{crossing};
	over.built.road.push_back({glm::dvec3{-0.2, 0.25, 49.8}, glm::dvec3{0.0, 0.25, 50.2}, glm::dvec3{0.2, 0.25, 49.8}});
	auto const covered{road_crossing::check(over.built, over.low, over.high, 0.02, 1000)};
	CHECK(std::any_of(covered.begin(), covered.end(), [](road_crossing::defect const &D) { return D.kind == "over"; }));
	// a block in a flangeway
	auto blocked{crossing};
	auto const centre{road_crossing::head_top(rail, 1, 1.0, -0.03)};
	blocked.built.panels.push_back({centre + glm::dvec3{-0.05, -0.01, -0.3}, centre + glm::dvec3{0.05, -0.01, -0.3}, centre + glm::dvec3{0.0, -0.01, 0.3}});
	auto const filled{road_crossing::check(blocked.built, blocked.low, blocked.high, 0.02, 1000)};
	CHECK(std::any_of(filled.begin(), filled.end(), [](road_crossing::defect const &D) { return D.kind == "flangeway"; }));
}

TEST_CASE("crossing watertight: a single track, level, the road at the heads, the panels CBP on it") {
	test::track_reading reading;
	auto const track{test::read_track(straight("t", 0.0, 0.0))};
	watertight(build({crossed(*track, 50.0)}, {-30.0, 50.0}, {30.0, 50.0}, 0.18, true));
	// and without the panels: the road and its grooves alone
	watertight(build({crossed(*track, 50.0)}, {-30.0, 50.0}, {30.0, 50.0}, 0.18, false));
}

TEST_CASE("crossing watertight: a panel reaching over a short piece of the road by a corner leaves no road over its top") {
	// the top of a panel overlapping a piece only by a sliver at its corner is left out of it, the probes of its corners and middle off it
	road_crossing::panel panel;
	panel.outline = {{0.0, 0.0}, {3.0, 0.0}, {3.0, 0.64}, {0.0, 0.64}};
	panel.top = 0.18;
	panel.faces = {{glm::dvec3{0.0, 0.18, 0.0}, glm::dvec3{3.0, 0.18, 0.0}, glm::dvec3{3.0, 0.18, 0.64}}, {glm::dvec3{0.0, 0.18, 0.0}, glm::dvec3{3.0, 0.18, 0.64}, glm::dvec3{0.0, 0.18, 0.64}}};
	// a piece 0.2 m long at a skew, the corner of its surface reaching 4 cm into the panel over its long side, between the corners and
	// the middle of the side
	segment_data axis;
	glm::dvec2 const direction{std::sin(glm::radians(30.0)), std::cos(glm::radians(30.0))};
	glm::dvec2 const left{direction.y, -direction.x};
	auto const start{glm::dvec2{1.0, 0.60} - left * 3.5};
	axis.points[segment_data::point::start] = {start.x, 0.18, start.y};
	axis.points[segment_data::point::end] = {start.x + direction.x * 0.2, 0.18, start.y + direction.y * 0.2};
	auto const covers{road_crossing::covers(axis, 3.5, {panel})};
	CHECK(false == covers.empty());
	// and a panel well off the piece isn't
	auto away{panel};
	for (auto &corner : away.outline)
		corner.x += 20.0;
	for (auto &face : away.faces)
		for (auto &corner : face)
			corner.x += 20.0;
	CHECK(road_crossing::covers(axis, 3.5, {away}).empty());
}

TEST_CASE("crossing watertight: a double track canted 119 mm with its heads in one plane, the panels CBP on both, no hump between them") {
	test::track_reading reading;
	auto const one{test::read_track(straight("a", 0.0, 119.0))};
	auto const level{test::read_track(straight("b", 4.5, 119.0))};
	auto const a{crossed(*one, 50.0)};
	// the second track at the level of the plane of the first: as the fix of the break levels it
	double raise{0.0};
	int moved{-1};
	REQUIRE(level_crossing::one_plane(a, crossed(*level, 50.0), raise, moved));
	CHECK(moved == 1);
	CHECK(raise == doctest::Approx(4.5 * std::tan(std::asin(0.119 / 1.5))).epsilon(0.01));
	auto const two{test::read_track(straight("b", 4.5, 119.0, raise))};
	std::vector<level_crossing::rail_track> const rails{a, crossed(*two, 50.0)};
	auto const crossing{build(rails, {-35.0, 50.0}, {40.0, 50.0}, 0.18, true)};
	CHECK(crossing.design.largest_break <= 0.0005);
	watertight(crossing);
	no_hump(crossing, rails[0], rails[1], {2.25, 50.0});
}

TEST_CASE("crossing watertight: the grade line breaking over 5% at the heads of two tracks side by side refused, its two fixes (Dz.U. 2025 poz. 1105, annex 5)") {
	test::track_reading reading;
	auto const one{test::read_track(straight("a", 0.0, 119.0))};
	auto const two{test::read_track(straight("b", 4.5, 119.0))};
	std::vector<level_crossing::rail_track> const rails{crossed(*one, 50.0), crossed(*two, 50.0)};
	level_crossing::design_input input;
	input.pieces = {road({-35.0, 50.0}, {40.0, 50.0}, 0.18)};
	input.tracks = rails;
	auto const refused{level_crossing::design(input)};
	CHECK(false == refused.valid);
	CHECK(refused.reason.find("annex 5 pt 3") != std::string::npos);
	REQUIRE(refused.breaks.size() == 1);
	// from 7.9% up the plane of the one to 3.6% down the line to the other
	CHECK(refused.largest_break == doctest::Approx(level_crossing::grade_break(rails[0], rails[1], {1.0, 0.0})).epsilon(0.01));
	CHECK(refused.largest_break > 0.11);
	// the cants of both lowered alike as far as keeps it to 5%
	auto const share{level_crossing::break_share(rails[0], rails[1], {1.0, 0.0})};
	CHECK(share > 0.3);
	CHECK(share < 0.5);
	input.tracks = {level_crossing::with_cant(rails[0], share), level_crossing::with_cant(rails[1], share)};
	auto const lowered{level_crossing::design(input)};
	INFO(lowered.reason);
	REQUIRE(lowered.valid);
	CHECK(lowered.largest_break <= level_crossing::break_limit + 1e-4);
	CHECK(lowered.largest_break >= level_crossing::break_limit - 0.002);
	// cants differing: their levels alone can't put them in one plane
	auto const other{test::read_track(straight("c", 4.5, 60.0))};
	double raise{0.0};
	int moved{-1};
	CHECK(false == level_crossing::one_plane(rails[0], crossed(*other, 50.0), raise, moved));
}

TEST_CASE("crossing watertight: a road crossing a track at 60 degrees, the panels CBP on it") {
	test::track_reading reading;
	auto const track{test::read_track(straight("t", 0.0, 0.0))};
	glm::dvec2 const direction{std::sin(glm::radians(60.0)), std::cos(glm::radians(60.0))};
	glm::dvec2 const middle{0.0, 50.0};
	watertight(build({crossed(*track, 50.0)}, middle - direction * 30.0, middle + direction * 30.0, 0.18, true));
}

TEST_CASE("crossing watertight: a road crossing a track on a curve of R 600 m canted 60 mm, the panels CBP on it") {
	test::track_reading reading;
	auto const track{test::read_track(curve("c", 60.0))};
	auto const rail{crossed(*track, 50.0)};
	glm::dvec2 const point{rail.point.x, rail.point.z};
	auto const direction{rail.right()};
	auto const height{road_crossing::plane_of(rail).at(point)};
	watertight(build({rail}, point - direction * 30.0, point + direction * 30.0, height, true));
}

TEST_CASE("crossing watertight: a double track 5.5 m apart crossed at 60 and 45 degrees, canted as much as the 8% and the 5% of the regulation let it, the tracks at their own heights") {
	test::track_reading reading;
	for (auto const angle : {60.0, 45.0}) {
		CAPTURE(angle);
		// the most cant the road over the crossing at 8% allows at the angle (§ 31 ust. 4), within the most the track norms allow
		auto const most{std::min(level_crossing::allowed_cant(angle, 0.0, false), norms_cant())};
		// 1500 mm * sin(atan(8% / sin(60))) = 138.0 mm, taken down to the whole millimetre; at 45 degrees 168.7 mm, over the 150 of the
		// norms
		CHECK(most == doctest::Approx(angle == 60.0 ? 137.0 : 150.0));
		auto const one{test::read_track(straight("a", 0.0, most))};
		auto const two{test::read_track(straight("b", 5.5, most))};
		glm::dvec2 const direction{std::sin(glm::radians(angle)), std::cos(glm::radians(angle))};
		auto rails{std::vector<level_crossing::rail_track>{crossed(*one, 50.0), crossed(*two, 50.0 + 5.5 / std::tan(glm::radians(angle)))}};
		// the break at the heads of the tracks at their own heights over 5%: their cants lowered alike, the fix of the editor
		auto const share{level_crossing::break_share(rails[0], rails[1], direction)};
		CHECK(share < 1.0);
		auto const cant{std::floor(most * share)};
		auto const lowa{test::read_track(straight("a", 0.0, cant))};
		auto const lowb{test::read_track(straight("b", 5.5, cant))};
		rails = {crossed(*lowa, 50.0), crossed(*lowb, 50.0 + 5.5 / std::tan(glm::radians(angle)))};
		glm::dvec2 const middle{2.75, 50.0 + 2.75 / std::tan(glm::radians(angle))};
		auto const crossing{build(rails, middle - direction * 40.0, middle + direction * 40.0, 0.18, true)};
		CHECK(crossing.design.largest_break <= level_crossing::break_limit + 1e-4);
		watertight(crossing);
		no_hump(crossing, rails[0], rails[1], middle);
		lanes_on_surface(crossing);
	}
}

TEST_CASE("crossing watertight: a double track 5.5 m apart crossed at 45 degrees canted 150 mm with its heads in one plane") {
	test::track_reading reading;
	auto const angle{45.0};
	auto const one{test::read_track(straight("a", 0.0, 150.0))};
	auto const level{test::read_track(straight("b", 5.5, 150.0))};
	auto const along{50.0 + 5.5 / std::tan(glm::radians(angle))};
	double raise{0.0};
	int moved{-1};
	REQUIRE(level_crossing::one_plane(crossed(*one, 50.0), crossed(*level, along), raise, moved));
	auto const two{test::read_track(straight("b", 5.5, 150.0, raise))};
	std::vector<level_crossing::rail_track> const rails{crossed(*one, 50.0), crossed(*two, along)};
	glm::dvec2 const direction{std::sin(glm::radians(angle)), std::cos(glm::radians(angle))};
	glm::dvec2 const middle{2.75, 50.0 + 2.75 / std::tan(glm::radians(angle))};
	auto const crossing{build(rails, middle - direction * 40.0, middle + direction * 40.0, 0.18, true)};
	CHECK(crossing.design.largest_break <= 0.0005);
	watertight(crossing);
	no_hump(crossing, rails[0], rails[1], middle);
	lanes_on_surface(crossing);
}

TEST_CASE("crossing watertight: the double track of the curves i and o of the editor's cases at 119 mm, o brought into the plane of i") {
	test::track_reading reading;
	auto const roll{std::to_string(roll_of(-119.0))};
	auto const text = [&](std::string const &Name, std::string const &Points, std::string const &Radius) {
		return "node -1 0 " + Name + " track normal 30 1.435 0.15 25.0 20 0 flat vis\n rail_screw_used1 6 1435mm/tpbps-new2 0.2 0.5 1.1\n" + Points + "\n" + Radius + "\nendtrack";
	};
	auto const points = [&](double const Raise) {
		auto const y{std::to_string(Raise)};
		return "302.287883 " + y + " 90.335351 " + roll + "\n1.505668 0 9.962387\n-2.001698 0 -9.874685\n307.549754 " + y + " 120.09561 " + roll;
	};
	auto const i4{test::read_track(text("i4", "306.737353 0 89.662879 " + roll + "\n1.494459 0 9.888226\n-1.986797 0 -9.801176\n311.960053 0 119.201598 " + roll, "600"))};
	auto const o4{test::read_track(text("o4", points(0.0), "604.5"))};
	double raise{0.0};
	int moved{-1};
	REQUIRE(level_crossing::one_plane(crossed(*i4, 15.0), crossed(*o4, 15.11), raise, moved));
	CHECK(moved == 1);
	auto const raised{test::read_track(text("o4", points(raise), "604.5"))};
	std::vector<level_crossing::rail_track> const rails{crossed(*i4, 15.0), crossed(*raised, 15.11)};
	// the road long enough for its approaches to come up to o, 0.36 m up
	auto const crossing{build(rails, {363.51, 94.86}, {250.76, 114.79}, 0.0, true)};
	CHECK(crossing.design.largest_break <= 0.002);
	watertight(crossing);
}

TEST_CASE("crossing watertight: the surface between two tracks on curves goes on from the plane of the heads of each without a step") {
	// the tracks sp of the editor's cases: R 2000 m and 2005.5 m, 5.5 m apart, canted 61 mm, crossed at 60 degrees; their arcs leave the
	// straight lines along them by millimetres within the width of the road
	test::track_reading reading;
	auto const one{test::read_track(arc("a", 0.0, 2000.0, 61.0))};
	auto const two{test::read_track(arc("b", -5.5, 2005.5, 61.0))};
	auto const theta{150.0 / 2000.0};
	glm::dvec2 const tangent{std::sin(theta), std::cos(theta)};
	glm::dvec2 const outwards{-std::cos(theta), std::sin(theta)};
	auto const middle{glm::dvec2{2000.0 * (1.0 - std::cos(theta)), 2000.0 * std::sin(theta)} + outwards * 2.75};
	auto const angle{glm::radians(60.0)};
	glm::dvec2 const direction{tangent.x * std::cos(angle) + outwards.x * std::sin(angle), tangent.y * std::cos(angle) + outwards.y * std::sin(angle)};
	std::vector<level_crossing::rail_track> const rails{crossed(*one, along_arc(0.0, 2000.0, middle, direction)), crossed(*two, along_arc(-5.5, 2005.5, middle, direction))};
	auto const surface{road_crossing::surface_of(rails, direction, 3.5)};
	REQUIRE(surface.tracks.size() == 2);
	REQUIRE(road_crossing::facing(rails[0], rails[1], direction, 3.5));
	// across the field side of the heads facing the other track, where the road reaches over it: the same height a hair inside and outside
	double worst{0.0};
	for (std::size_t k = 0; k < 2; ++k) {
		auto const &track{rails[k]};
		auto const &other{rails[1 - k]};
		double along{0.0}, across{0.0};
		road_crossing::track_frame(track, plan(other.point), along, across);
		auto const side{across >= 0.0 ? 1.0 : -1.0};
		for (double s = -5.0; s <= 5.0; s += 0.25) {
			glm::dvec2 right;
			auto const centre{track.axis_at(s, right)};
			auto const band{road_crossing::band(track)};
			double inside{0.0}, outside{0.0};
			REQUIRE(surface.at(centre + right * (side * (band - 1e-7)), inside));
			REQUIRE(surface.at(centre + right * (side * (band + 1e-7)), outside));
			CAPTURE(k);
			CAPTURE(s);
			CHECK(std::abs(inside - outside) <= 1e-6);
			worst = std::max(worst, std::abs(inside - outside));
		}
	}
	MESSAGE("the largest step at the field side of the heads: " << worst * 1000.0 << " mm");
}

TEST_CASE("crossing watertight: the double track sp and sq of the editor's cases on curves of R 2000 m, 5.5 m apart, crossed at 60 and 45 degrees, the road laid as one band across each piece") {
	// the surface of each piece one band across the road, as the road lays it: its ends single edges across, which the cuts along the
	// rails split each piece its own way. Between the two tracks out of parallel (a curve) the surface is twisted; the pieces meeting there
	// meet at the same points of their ends (road_crossing::conform), so no step at a joint for a ray at a slant to slip through
	test::track_reading reading;
	auto const theta{150.0 / 2000.0};
	glm::dvec2 const tangent{std::sin(theta), std::cos(theta)};
	glm::dvec2 const outwards{-std::cos(theta), std::sin(theta)};
	auto const middle{glm::dvec2{2000.0 * (1.0 - std::cos(theta)), 2000.0 * std::sin(theta)} + outwards * 2.75};
	for (auto const degrees : {60.0, 45.0}) {
		CAPTURE(degrees);
		auto const angle{glm::radians(degrees)};
		glm::dvec2 const direction{tangent.x * std::cos(angle) + outwards.x * std::sin(angle), tangent.y * std::cos(angle) + outwards.y * std::sin(angle)};
		auto const at_one{along_arc(0.0, 2000.0, middle, direction)};
		auto const at_two{along_arc(-5.5, 2005.5, middle, direction)};
		std::vector<level_crossing::rail_track> rails;
		if (degrees == 60.0) {
			// 137 mm, the most the 8% allows at 60 degrees, lowered alike on both to keep the break at the heads to 5%, as the fix of the
			// editor lowers them
			auto const one{test::read_track(arc("a", 0.0, 2000.0, 137.0))};
			auto const two{test::read_track(arc("b", -5.5, 2005.5, 137.0))};
			auto const share{level_crossing::break_share(crossed(*one, at_one), crossed(*two, at_two), direction)};
			REQUIRE(share < 1.0);
			auto const cant{std::floor(137.0 * share)};
			auto const lowa{test::read_track(arc("a", 0.0, 2000.0, cant))};
			auto const lowb{test::read_track(arc("b", -5.5, 2005.5, cant))};
			rails = {crossed(*lowa, at_one), crossed(*lowb, at_two)};
		}
		else {
			// 150 mm, the second track brought into the plane of the first by its level, as the fix of the editor levels it
			auto const one{test::read_track(arc("a", 0.0, 2000.0, 150.0))};
			auto const level{test::read_track(arc("b", -5.5, 2005.5, 150.0))};
			double raise{0.0};
			int moved{-1};
			REQUIRE(level_crossing::one_plane(crossed(*one, at_one), crossed(*level, at_two), raise, moved));
			auto const two{test::read_track(arc("b", -5.5, 2005.5, 150.0, moved == 1 ? raise : 0.0))};
			auto const first{moved == 0 ? test::read_track(arc("a", 0.0, 2000.0, 150.0, raise)) : test::read_track(arc("a", 0.0, 2000.0, 150.0))};
			rails = {crossed(*first, at_one), crossed(*two, at_two)};
		}
		auto const height{level_crossing::plane_of(rails[0]).at(middle)};
		auto const crossing{build(rails, middle - direction * 40.0, middle + direction * 40.0, height, true, true)};
		CHECK(crossing.design.largest_break <= level_crossing::break_limit + 1e-4);
		watertight(crossing);
		MESSAGE("the largest step at a joint: " << joints_meet(crossing) * 1000.0 << " mm");
		lanes_on_surface(crossing);
	}
}

TEST_CASE("crossing watertight: a crack the welding of the corners leaves isn't a hole, a gap of a tenth of a millimetre is") {
	// a square of the road 2 m across, level, cut in two along a line with a gap between the halves: a few micrometres, as the corners
	// welded to 0.01 mm leave beside a corner a cut makes on an edge (a T), or a tenth of a millimetre. Along the line the rays straight
	// down meet it (the line through the points of the grid), at 45 degrees or nearly along z, or the rays at 10 degrees along the road
	// pass the surface on it
	auto const halves = [](glm::dvec2 const &Through, glm::dvec2 const &Along, double const Width) {
		std::vector<std::array<glm::dvec3, 3>> result;
		std::vector<glm::dvec2> const square{{-1.0, -1.0}, {1.0, -1.0}, {1.0, 1.0}, {-1.0, 1.0}};
		auto const direction{glm::normalize(Along)};
		for (auto const side : {-1.0, 1.0}) {
			// the part of the square on a side of the line, Width / 2 away from it
			auto const distance = [&](glm::dvec2 const &P) { return side * (direction.x * (P.y - Through.y) - direction.y * (P.x - Through.x)) - 0.5 * Width; };
			std::vector<glm::dvec2> part;
			for (std::size_t i = 0; i < square.size(); ++i) {
				auto const &a{square[i]};
				auto const &b{square[(i + 1) % square.size()]};
				auto const da{distance(a)}, db{distance(b)};
				if (da >= 0.0)
					part.push_back(a);
				if ((da >= 0.0) != (db >= 0.0))
					part.push_back(a + (b - a) * (da / (da - db)));
			}
			for (std::size_t k = 1; k + 1 < part.size(); ++k)
				result.push_back({glm::dvec3{part[0].x, 0.0, part[0].y}, glm::dvec3{part[k].x, 0.0, part[k].y}, glm::dvec3{part[k + 1].x, 0.0, part[k + 1].y}});
		}
		return result;
	};
	auto const built = [&](glm::dvec2 const &Through, glm::dvec2 const &Along, double const Width, bool const Slant) {
		road_crossing::built_crossing crossing;
		crossing.road = halves(Through, Along, Width);
		segment_data axis;
		axis.points[segment_data::point::start] = {-1.0, 0.0, 0.0};
		axis.points[segment_data::point::end] = {1.0, 0.0, 0.0};
		crossing.footprint.emplace_back(axis, 1.0);
		// something for the rays at a slant to look over, deep under the road out of their reach (a ray straight down meets it)
		if (Slant) {
			crossing.rails.push_back({glm::dvec3{-1.0, -1.0, -1.0}, glm::dvec3{1.0, -1.0, -1.0}, glm::dvec3{1.0, -1.0, 1.0}});
			crossing.rails.push_back({glm::dvec3{-1.0, -1.0, -1.0}, glm::dvec3{1.0, -1.0, 1.0}, glm::dvec3{-1.0, -1.0, 1.0}});
		}
		return crossing;
	};
	auto const kinds = [](std::vector<road_crossing::defect> const &Defects, std::string const &Kind) {
		return std::count_if(Defects.begin(), Defects.end(), [&](road_crossing::defect const &D) { return D.kind == Kind; });
	};
	glm::dvec2 const low{-0.5, -0.5}, high{0.5, 0.5};
	// straight down: the line through the points of the grid, at 45 degrees and at 10 degrees off z (the points a ray is moved to across
	// it 0.02 mm along x or z both in it then)
	for (auto const &along : {glm::dvec2{1.0, 1.0}, glm::dvec2{std::sin(glm::radians(10.0)), std::cos(glm::radians(10.0))}}) {
		CAPTURE(along.x);
		std::size_t cracks{0};
		auto const crack{road_crossing::check(built({0.0, 0.0}, along, 0.000005, false), low, high, 0.02, 1000, &cracks)};
		CHECK(crack.empty());
		CHECK(cracks > 0);
		auto const gap{road_crossing::check(built({0.0, 0.0}, along, 0.0001, false), low, high, 0.02, 1000)};
		CHECK(kinds(gap, "hole") > 0);
	}
	// at a slant: a ray at 10 degrees along the road aimed 2 mm under it passes the surface 11.34 mm before the point aimed at, on the
	// line for the points of the grid of 5 cm on the diagonal, the line off the points of the grid straight down
	{
		auto const before{0.002 / std::tan(glm::radians(10.0))};
		std::size_t cracks{0};
		auto const crack{road_crossing::check(built({0.0, before}, {1.0, 1.0}, 0.000005, true), low, high, 0.02, 1000, &cracks)};
		CHECK(crack.empty());
		CHECK(cracks > 0);
		auto const gap{road_crossing::check(built({0.0, before}, {1.0, 1.0}, 0.0001, true), low, high, 0.02, 1000)};
		CHECK(kinds(gap, "slit") > 0);
		CHECK(kinds(gap, "hole") == 0);
	}
}

TEST_CASE("crossing watertight: a skirt hangs from an edge of the road along a panel over a part of its length") {
	// the edge of the surface from z 0 to 2 along x 0, a panel from z 1.5 on beside it: its corner on the edge where the triangle beside
	// wasn't cut at it (a T), the edge along the panel only from there, the middle of the edge off it. The skirt hangs from the edge all
	// the same, so a ray slipping under the edge of the road beside the panel meets it
	std::vector<world_vertex> surface{{{0.0, 0.0, 0.0}, {0.f, 1.f, 0.f}, {0.f, 0.f}}, {{0.0, 0.0, 2.0}, {0.f, 1.f, 0.f}, {0.f, 0.f}}, {{-1.0, 0.0, 1.0}, {0.f, 1.f, 0.f}, {0.f, 0.f}}};
	std::vector<std::vector<glm::dvec2>> const covers{{{0.0, 1.5}, {0.64, 1.5}, {0.64, 4.5}, {0.0, 4.5}}};
	auto const skirt{road_crossing::skirts(surface, covers)};
	auto hangs{false};
	for (std::size_t i = 0; i + 2 < skirt.size(); i += 3) {
		auto top{0};
		for (int k = 0; k < 3; ++k) {
			auto const &p{skirt[i + k].position};
			if (std::abs(p.x) < 1e-9 && std::abs(p.y) < 1e-9 && (std::abs(p.z) < 1e-9 || std::abs(p.z - 2.0) < 1e-9))
				++top;
		}
		hangs = hangs || top == 2;
	}
	CHECK(hangs);
}
