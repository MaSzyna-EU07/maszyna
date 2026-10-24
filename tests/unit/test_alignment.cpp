#include "stdafx.h"
#include "doctest.h"
#include "editor/editorAlignment.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace {

double constexpr pi{3.14159265358979323846};

glm::dvec2 heading(double const Angle) { return {std::cos(Angle), std::sin(Angle)}; }

// straights from the origin along x, bending at the vertices by the deflections given (radians, to the left positive), each leg the
// length given; the vertices clicked as bends of straights
alignment::design straights(std::vector<double> const &Legs, std::vector<double> const &Deflections, double const Speed) {
	alignment::design design;
	design.speed = Speed;
	glm::dvec2 point{0.0};
	double direction{0.0};
	std::vector<glm::dvec2> corners;
	for (std::size_t i = 0; i < Legs.size(); ++i) {
		point += heading(direction) * Legs[i];
		if (i < Deflections.size()) {
			corners.push_back(point);
			direction += Deflections[i];
		}
	}
	design.start = {0.0, 0.0, 0.0};
	design.end = {point.x, 0.0, point.y};
	design.start_direction = heading(0.0);
	design.end_direction = heading(direction);
	for (auto const &corner : corners) {
		alignment::vertex vertex;
		vertex.position = corner;
		design.vertices.push_back(vertex);
	}
	if (design.vertices.size() >= 2) {
		design.vertices.front().offset = Legs.front();
		design.vertices.back().offset = Legs.back();
	}
	return design;
}

// points along the curves of the line every Spacing metres, from 30 m before the first curve to its last one, and two on the straight
// after it, shifted across by up to Noise
std::vector<glm::dvec2> traced(alignment::result const &Line, double const Spacing, double const Noise) {
	auto from{Line.length};
	auto to{0.0};
	for (auto const &e : Line.elements)
		if (e.kind != alignment::element_kind::straight) {
			from = std::min(from, e.chainage);
			to = std::max(to, e.chainage + e.length);
		}
	std::vector<glm::dvec2> points;
	auto const along = [&](double const Chainage, double const Shift) {
		auto const station{alignment::evaluate(Line, Chainage)};
		glm::dvec2 const across{-station.direction.z, station.direction.x};
		return glm::dvec2{station.position.x, station.position.z} + glm::normalize(across) * Shift;
	};
	int count{0};
	for (auto chainage{std::max(0.0, from - 30.0)}; chainage <= to; chainage += Spacing, ++count)
		points.push_back(along(chainage, Noise * (count % 3 == 0 ? 1.0 : count % 3 == 1 ? -0.6 : 0.2)));
	points.push_back(along(to + 30.0, Noise * 0.5));
	points.push_back(along(to + 60.0, -Noise * 0.5));
	return points;
}

// the stretch traced as the lay tool asks for it: from the start of the line along its first straight, to the last point along the
// straight through the last two
alignment::design stretch_base(alignment::design const &Line, std::vector<glm::dvec2> const &Points) {
	alignment::design base{Line};
	base.vertices.clear();
	base.end = {Points.back().x, 0.0, Points.back().y};
	base.end_direction = glm::normalize(Points.back() - Points[Points.size() - 2]);
	return base;
}

} // namespace

TEST_CASE("alignment: a slight bend of straights is eased with two transition curves meeting, with no cant") {
	auto design{straights({500.0, 500.0}, {3.0 * pi / 180.0}, 160.0)};
	std::vector<char> corners{1};
	auto const line{alignment::ease_corners(design, corners, {3000.0, 6000.0})};

	REQUIRE(line.valid);
	REQUIRE(line.curves.size() == 1);
	auto const &curve{line.curves.front()};
	CHECK(curve.spirals);
	CHECK(curve.cant == doctest::Approx(0.0));
	CHECK(curve.arc_length < 0.01);
	CHECK(curve.radius >= 3000.0 - 1e-6);
	CHECK(curve.radius <= 6000.0 + 1e-6);
	CHECK(alignment::meets_norms(curve, design));
	// the bend stays where it was clicked
	CHECK(line.vertices.front().x == doctest::Approx(500.0));
	CHECK(line.vertices.front().y == doctest::Approx(0.0));
}

TEST_CASE("alignment: a bend too slight for transitions long enough at the radii given takes a larger radius") {
	auto design{straights({500.0, 500.0}, {0.2 * pi / 180.0}, 160.0)};
	std::vector<char> corners{1};
	auto const line{alignment::ease_corners(design, corners, {3000.0, 6000.0})};

	REQUIRE(line.valid);
	REQUIRE(line.curves.size() == 1);
	auto const &curve{line.curves.front()};
	CHECK(curve.spirals);
	CHECK(curve.radius > 6000.0);
	CHECK(curve.transition_in >= alignment::transition_length(160.0, curve.radius, 0.0, design.norms) - 1e-6);
	CHECK(alignment::meets_norms(curve, design));
}

TEST_CASE("alignment: a sharp bend between short straights is laid where clicked, an arc with cant below the norms") {
	auto design{straights({120.0, 120.0}, {60.0 * pi / 180.0}, 160.0)};
	std::vector<char> corners{1};
	auto const line{alignment::ease_corners(design, corners, {3000.0, 6000.0})};

	REQUIRE(line.valid);
	REQUIRE(line.curves.size() == 1);
	auto const &curve{line.curves.front()};
	CHECK_FALSE(curve.spirals);
	CHECK(curve.cant > 0.0);
	CHECK(curve.radius < alignment::recommend(160.0, curve.radius, design.norms).radius_min);
	CHECK_FALSE(alignment::meets_norms(curve, design));
	CHECK_FALSE(line.warnings.empty());
	CHECK(curve.tangent_in <= 120.0 + 1e-6);
	CHECK(line.vertices.front().x == doctest::Approx(120.0));
}

TEST_CASE("alignment: two bends share the straight between them") {
	auto design{straights({600.0, 250.0, 600.0}, {4.0 * pi / 180.0, -5.0 * pi / 180.0}, 120.0)};
	std::vector<char> corners{1, 1};
	auto const line{alignment::ease_corners(design, corners, {3000.0, 6000.0})};

	REQUIRE(line.valid);
	REQUIRE(line.curves.size() == 2);
	CHECK(line.curves[0].tangent_out + line.curves[1].tangent_in <= 250.0 + 1e-3);
	CHECK(line.vertices[1].x == doctest::Approx(design.vertices[1].position.x));
	CHECK(line.vertices[1].y == doctest::Approx(design.vertices[1].position.y));
}

TEST_CASE("alignment: two transition curves meet with no arc in a design computed again") {
	auto design{straights({500.0, 500.0}, {3.0 * pi / 180.0}, 120.0)};
	design.vertices.front().spirals = true;
	design.vertices.front().radius = 4000.0;
	design.vertices.front().cant = 60.0;
	auto const line{alignment::compute(design)};

	REQUIRE(line.valid);
	REQUIRE(line.curves.size() == 1);
	auto const &curve{line.curves.front()};
	CHECK(curve.spirals);
	CHECK(curve.cant == doctest::Approx(0.0));
	CHECK(curve.radius == doctest::Approx(4000.0));
	CHECK(curve.transition_in == doctest::Approx(alignment::spirals_length(design.shape, 3.0 * pi / 180.0, 4000.0)));
	// the heading of the line out is that of the straight after the bend
	auto const station{alignment::evaluate(line, line.length)};
	CHECK(std::atan2(station.direction.z, station.direction.x) == doctest::Approx(3.0 * pi / 180.0).epsilon(1e-6));
}

TEST_CASE("alignment: the radius and the transitions of a curve traced along a known one") {
	auto truth{straights({500.0, 500.0}, {25.0 * pi / 180.0}, 120.0)};
	truth.vertices.front().radius = 1000.0;
	truth.vertices.front().transition_in = truth.vertices.front().transition_out = 120.0;
	auto const line{alignment::compute(truth)};
	REQUIRE(line.valid);
	auto const points{traced(line, 20.0, 0.03)};

	auto const fit{alignment::fit_trace_stretch(stretch_base(truth, points), points, 0, true)};

	REQUIRE(fit.valid);
	REQUIRE(fit.vertices.size() == 1);
	CHECK(fit.vertices.front().radius == doctest::Approx(1000.0).epsilon(0.02));
	CHECK(fit.vertices.front().transition_in == doctest::Approx(120.0).epsilon(0.1));
	CHECK(fit.rms < 0.04);
}

TEST_CASE("alignment: a plain arc traced keeps short transitions, not the ones of the norms") {
	auto truth{straights({500.0, 500.0}, {20.0 * pi / 180.0}, 120.0)};
	truth.vertices.front().radius = 800.0;
	auto const line{alignment::compute(truth)};
	REQUIRE(line.valid);
	auto const points{traced(line, 15.0, 0.0)};

	auto const fit{alignment::fit_trace_stretch(stretch_base(truth, points), points, 0, true)};

	REQUIRE(fit.valid);
	REQUIRE(fit.vertices.size() == 1);
	CHECK(fit.vertices.front().radius == doctest::Approx(800.0).epsilon(0.01));
	CHECK(fit.vertices.front().transition_in < 20.0);
	CHECK(fit.rms < 0.02);
}

TEST_CASE("alignment: a curve, a straight and a curve the other way traced as two curves") {
	auto truth{straights({400.0, 700.0, 400.0}, {20.0 * pi / 180.0, -25.0 * pi / 180.0}, 120.0)};
	truth.vertices[0].radius = 1500.0;
	truth.vertices[1].radius = 1000.0;
	for (auto &vertex : truth.vertices)
		vertex.transition_in = vertex.transition_out = 100.0;
	auto const line{alignment::compute(truth)};
	REQUIRE(line.valid);
	REQUIRE(line.curves.size() == 2);
	auto const points{traced(line, 20.0, 0.02)};

	auto const fit{alignment::fit_trace_stretch(stretch_base(truth, points), points, 0, true)};

	REQUIRE(fit.valid);
	REQUIRE(fit.vertices.size() == 2);
	CHECK(fit.vertices[0].radius == doctest::Approx(line.curves[0].radius).epsilon(0.05));
	CHECK(fit.vertices[1].radius == doctest::Approx(line.curves[1].radius).epsilon(0.05));
	CHECK(fit.rms < 0.05);
}

TEST_CASE("alignment: two curves turning the same way with a straight between them traced") {
	auto truth{straights({400.0, 600.0, 400.0}, {15.0 * pi / 180.0, 15.0 * pi / 180.0}, 120.0)};
	truth.vertices[0].radius = 1200.0;
	truth.vertices[1].radius = 900.0;
	for (auto &vertex : truth.vertices)
		vertex.transition_in = vertex.transition_out = 90.0;
	auto const line{alignment::compute(truth)};
	REQUIRE(line.valid);
	REQUIRE(line.curves.size() == 2);
	auto const points{traced(line, 20.0, 0.02)};

	auto const fit{alignment::fit_trace_stretch(stretch_base(truth, points), points, 0, true)};

	REQUIRE(fit.valid);
	REQUIRE(fit.vertices.size() == 2);
	CHECK(fit.vertices[0].radius == doctest::Approx(1200.0).epsilon(0.05));
	CHECK(fit.vertices[1].radius == doctest::Approx(900.0).epsilon(0.05));
	CHECK(fit.rms < 0.04);
}

TEST_CASE("alignment: a compound curve traced is followed closely") {
	auto truth{straights({500.0, 500.0}, {35.0 * pi / 180.0}, 100.0)};
	auto &vertex{truth.vertices.front()};
	vertex.radius = 1200.0;
	vertex.compound = true;
	vertex.share = 0.5;
	vertex.arcs = {{700.0, 60.0, 0.5}};
	vertex.transition_in = vertex.transition_out = 80.0;
	auto const line{alignment::compute(truth)};
	REQUIRE(line.valid);
	auto const points{traced(line, 15.0, 0.02)};

	auto const fit{alignment::fit_trace_stretch(stretch_base(truth, points), points, 0, true)};

	REQUIRE(fit.valid);
	REQUIRE(fit.vertices.size() == 1);
	// the transition between the arcs the one of the norms, 45 m, not the 60 m of the line traced
	auto const &curve{fit.vertices.front()};
	REQUIRE(curve.compound);
	REQUIRE(curve.arcs.size() == 1);
	CHECK(curve.radius == doctest::Approx(1200.0).epsilon(0.05));
	CHECK(curve.arcs.front().radius == doctest::Approx(700.0).epsilon(0.05));
	CHECK(fit.rms < 0.1);
	CHECK(fit.worst < 0.2);
}

TEST_CASE("alignment: each arc of a compound curve has a cant of its own, ramped along the transitions between them") {
	auto design{straights({500.0, 500.0}, {35.0 * pi / 180.0}, 100.0)};
	auto &vertex{design.vertices.front()};
	vertex.radius = 1200.0;
	vertex.compound = true;
	vertex.share = 0.5;
	vertex.arcs = {{700.0, 60.0, 0.5}};
	vertex.transition_in = vertex.transition_out = 80.0;
	vertex.cant = 120.0;

	// until they're set, the cants of the arcs go with their curvature, the cant of the curve on the sharpest one
	auto const proportional{alignment::compound_cants(vertex)};
	REQUIRE(proportional.size() == 2);
	CHECK(proportional[0] == doctest::Approx(70.0));
	CHECK(proportional[1] == doctest::Approx(120.0));

	vertex.first_cant = 40.0;
	vertex.arcs.front().cant = 110.0;
	auto const line{alignment::compute(design)};
	REQUIRE(line.valid);
	REQUIRE(line.curves.size() == 1);
	auto const &curve{line.curves.front()};
	REQUIRE(curve.cants.size() == 2);
	CHECK(curve.cants[0] == doctest::Approx(40.0));
	CHECK(curve.cants[1] == doctest::Approx(110.0));
	CHECK(curve.cant == doctest::Approx(110.0));

	std::vector<alignment::element> pieces;
	for (auto const &e : line.elements)
		if (e.vertex == 0)
			pieces.push_back(e);
	// in, R1, between, R2, out
	REQUIRE(pieces.size() == 5);
	auto const cant = [&](alignment::element const &Element, double const Along) { return alignment::evaluate(line, Element.chainage + Element.length * Along).cant; };
	CHECK(cant(pieces[0], 0.0) == doctest::Approx(0.0).epsilon(1e-6));
	CHECK(cant(pieces[0], 0.5) == doctest::Approx(20.0));
	CHECK(cant(pieces[1], 0.5) == doctest::Approx(40.0));
	CHECK(cant(pieces[2], 0.5) == doctest::Approx(75.0));
	CHECK(cant(pieces[3], 0.5) == doctest::Approx(110.0));
	CHECK(cant(pieces[4], 0.5) == doctest::Approx(55.0));

	// the transition between the arcs of the norms for the change of the cant set
	alignment::recommend_between(vertex, design.speed, design.norms);
	CHECK(vertex.arcs.front().transition == doctest::Approx(alignment::transition_between(design.speed, 1.0 / 1200.0, 40.0, 1.0 / 700.0, 110.0, design.norms)));
}

TEST_CASE("alignment: an arc of a compound curve with no cant gets a longer transition, the curve fitted again to the points it was laid through") {
	auto design{straights({600.0, 600.0}, {35.0 * pi / 180.0}, 100.0)};
	auto &vertex{design.vertices.front()};
	vertex.radius = 1200.0;
	vertex.compound = true;
	vertex.share = 0.5;
	vertex.arcs = {{700.0, 0.0, 0.5}};
	vertex.transition_in = vertex.transition_out = 80.0;
	vertex.cant = 120.0;
	alignment::recommend_between(vertex, design.speed, design.norms);
	auto const line{alignment::compute(design)};
	REQUIRE(line.valid);
	auto const points{traced(line, 15.0, 0.0)};
	auto const before{vertex.arcs.front().transition};

	// the second arc with no cant, the first one keeping its own
	auto lowered{design};
	auto &shape{lowered.vertices.front()};
	shape.first_cant = 70.0;
	shape.arcs.front().cant = 0.0;
	alignment::norm_transitions(shape, lowered.speed, lowered.norms);
	auto const between{shape.arcs.front().transition};
	CHECK(between == doctest::Approx(alignment::transition_between(lowered.speed, 1.0 / 1200.0, 70.0, 1.0 / 700.0, 0.0, lowered.norms)));
	CHECK(between > before);

	auto const departure = [&](alignment::design const &Design) {
		double sum{0.0};
		std::vector<glm::dvec2> plan(points.begin(), points.end());
		for (auto const &projection : alignment::project(alignment::compute(Design), plan))
			sum += projection.offset * projection.offset;
		return std::sqrt(sum / static_cast<double>(points.size()));
	};
	auto const fit{alignment::refit_radii(lowered, 0, points)};
	REQUIRE(fit.valid);
	// the transitions and the cants as they were set, the radii moved to keep to the points
	CHECK(fit.fitted.arcs.front().transition == doctest::Approx(between));
	CHECK(fit.fitted.transition_in == doctest::Approx(shape.transition_in));
	CHECK(fit.fitted.arcs.front().cant == doctest::Approx(0.0));
	CHECK(fit.fitted.first_cant == doctest::Approx(70.0));
	CHECK(fit.rms < departure(lowered));
	CHECK(fit.rms < 0.3);
}

TEST_CASE("alignment: bends clicked close along a curve make one compound curve, not curves with short straights between them") {
	// the points clicked in the scenery abc, at 60 km/h: from a straight along z into a curve to the left
	alignment::design design;
	design.speed = 60.0;
	design.shape = alignment::transition_shape::clothoid;
	design.start = {-399.8169, 0.0, 419.0340};
	design.end = {-665.1078, 0.0, 1138.5287};
	design.start_direction = glm::normalize(glm::dvec2{0.024027152, 0.999711306});
	design.end_direction = glm::normalize(glm::dvec2{-0.831198797, 0.555975323});
	for (auto const &point : std::vector<glm::dvec2>{{-394.5230, 639.2978}, {-395.0216, 680.5362}, {-400.4562, 753.8415}, {-405.1477, 807.6113}, {-410.4658, 847.8606}, {-419.7362, 885.8522},
	                                                 {-435.3280, 924.7706}, {-458.2043, 961.0863}, {-485.8050, 994.2479}, {-531.7847, 1039.0865}, {-565.5341, 1067.6467}, {-637.3993, 1119.9949}}) {
		alignment::vertex vertex;
		vertex.position = point;
		design.vertices.push_back(vertex);
	}
	design.vertices.front().offset = glm::dot(design.vertices.front().position - glm::dvec2{design.start.x, design.start.z}, design.start_direction);
	design.vertices.back().offset = glm::dot(glm::dvec2{design.end.x, design.end.z} - design.vertices.back().position, design.end_direction);
	std::vector<char> corners(design.vertices.size(), 1);

	auto const line{alignment::ease_corners(design, corners, {3000.0, 6000.0})};

	REQUIRE(line.valid);
	auto const shortest{std::max(30.0, design.speed)};
	int straights{0};
	for (std::size_t i = 0; i < line.elements.size(); ++i) {
		auto const &e{line.elements[i]};
		if (e.kind != alignment::element_kind::straight || e.length < 1e-3)
			continue;
		++straights;
		// none short between two curves turning the same way
		if (i > 0 && i + 1 < line.elements.size() && line.elements[i - 1].turn == line.elements[i + 1].turn)
			CHECK(e.length >= shortest - 1e-6);
	}
	// the straight before the curve and the one after it
	CHECK(straights <= 2);
	CHECK(line.curves.size() <= 2);
	// the arcs of a compound curve meet through transition curves, the curvature not jumping from one radius to another
	for (std::size_t i = 0; i + 1 < line.elements.size(); ++i) {
		auto const &a{line.elements[i]};
		auto const &b{line.elements[i + 1]};
		if (a.kind == alignment::element_kind::spiral && b.kind == alignment::element_kind::spiral && a.vertex == b.vertex)
			CHECK(a.curvature_end == doctest::Approx(b.curvature_start).epsilon(1e-6));
	}
	CHECK(design.vertices.size() < 12);
}

TEST_CASE("alignment: a cant ramp moved off the transition curve onto the straight before it") {
	auto design{straights({300.0, 300.0}, {30.0 * pi / 180.0}, 80.0)};
	auto &vertex{design.vertices.front()};
	vertex.radius = 800.0;
	vertex.transition_in = vertex.transition_out = 60.0;
	vertex.cant = 60.0;
	auto const along{alignment::compute(design)};
	REQUIRE(along.valid);
	REQUIRE(along.ramps.size() == 2);
	auto const start{along.ramps.front().from};
	CHECK(along.ramps.front().to - start == doctest::Approx(60.0));
	CHECK(alignment::evaluate(along, start + 30.0).cant == doctest::Approx(30.0));

	// the whole ramp on the straight, the transition with the full cant all along it, as a switch laid on it wants it
	vertex.ramp_in_offset = -40.0;
	vertex.ramp_in_length = 40.0;
	auto const moved{alignment::compute(design)};
	REQUIRE(moved.valid);
	REQUIRE(moved.ramps.size() == 2);
	CHECK(moved.ramps.front().own);
	CHECK(moved.ramps.front().from == doctest::Approx(start - 40.0));
	CHECK(moved.ramps.front().to == doctest::Approx(start));
	CHECK(alignment::evaluate(moved, start - 50.0).cant == doctest::Approx(0.0));
	CHECK(alignment::evaluate(moved, start - 20.0).cant == doctest::Approx(30.0));
	CHECK(alignment::evaluate(moved, start + 10.0).cant == doctest::Approx(60.0));
	CHECK(alignment::evaluate(moved, start + 50.0).cant == doctest::Approx(60.0));
	// on the straight the cant leans the way the curve turns
	CHECK(alignment::evaluate(moved, start - 20.0).turn == alignment::evaluate(moved, start + 100.0).turn);
	// 40 m are too short to raise 60 mm at 80 km/h
	CHECK(std::any_of(moved.warnings.begin(), moved.warnings.end(), [](std::string const &Warning) { return Warning.find("cant ramp of") != std::string::npos; }));

	// the paths end where the ramp starts and ends, their rolls going along it
	auto const paths{alignment::pieces(moved, design, alignment::minimum_pieces(moved, design))};
	auto const meets = [&](double const Chainage) {
		auto const point{alignment::evaluate(moved, Chainage).position};
		return std::any_of(paths.begin(), paths.end(), [&](segment_data const &Path) { return glm::distance(Path.points[segment_data::point::start], point) < 0.05; });
	};
	CHECK(meets(start - 40.0));
	CHECK(meets(start));
	for (std::size_t i = 1; i < paths.size(); ++i)
		CHECK(paths[i].rolls[0] == doctest::Approx(paths[i - 1].rolls[1]).epsilon(1e-4));
	auto const turned{std::max_element(paths.begin(), paths.end(), [](segment_data const &A, segment_data const &B) { return std::abs(A.rolls[1]) < std::abs(B.rolls[1]); })};
	CHECK(std::abs(turned->rolls[1]) == doctest::Approx(glm::degrees(std::asin(60.0 / design.norms.gauge))).epsilon(1e-3));
}
