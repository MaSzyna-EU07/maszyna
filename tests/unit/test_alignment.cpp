#include "stdafx.h"
#include "doctest.h"
#include "editor/editorAlignment.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <random>
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

// a line from the origin along x through segments of the curvature going evenly from the first to the second (to the left positive) over
// the length, as the end it comes to and the heading there; small steps of Simpson
struct line_end {
	glm::dvec2 position{0.0};
	double heading{0.0};
};
line_end walked(std::vector<std::array<double, 3>> const &Segments) {
	line_end end;
	for (auto const &[from, to, length] : Segments) {
		int const steps{std::max(2, static_cast<int>(std::ceil(length / 0.25)) * 2)};
		auto const h{length / steps};
		auto const angle = [&, from = from, to = to, length = length](double const S) { return end.heading + from * S + (to - from) * S * S / (2.0 * length); };
		glm::dvec2 sum{0.0};
		for (int i = 0; i <= steps; ++i) {
			double const weight = (i == 0 || i == steps) ? 1.0 : (i % 2 ? 4.0 : 2.0);
			sum += weight * glm::dvec2{std::cos(angle(i * h)), std::sin(angle(i * h))};
		}
		end.position += sum * (h / 3.0);
		end.heading = angle(length);
	}
	return end;
}

// the curvature of the line at the chainage, positive to the left, from the headings half a metre either side
double curvature_of(alignment::result const &Line, double const Chainage) {
	auto const a{std::clamp(Chainage - 0.25, 0.0, Line.length)};
	auto const b{std::clamp(Chainage + 0.25, 0.0, Line.length)};
	auto const from{alignment::evaluate(Line, a).direction};
	auto const to{alignment::evaluate(Line, b).direction};
	return (std::atan2(to.z, to.x) - std::atan2(from.z, from.x)) / (b - a);
}

// a design from the origin along x, on from a curve to the left of the radius, to the straight the segments end at, 200 m along it; one
// vertex where the straights meet
alignment::design curved_from(double const Radius, std::vector<std::array<double, 3>> const &Segments, double const Speed) {
	auto const end{walked(Segments)};
	alignment::design design;
	design.speed = Speed;
	design.start = {0.0, 0.0, 0.0};
	design.start_direction = heading(0.0);
	auto const beyond{end.position + heading(end.heading) * 200.0};
	design.end = {beyond.x, 0.0, beyond.y};
	design.end_direction = heading(end.heading);
	design.start_radius = Radius;
	design.start_turn = 1;
	design.start_curved = true;
	design.vertices.emplace_back();
	return design;
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

TEST_CASE("alignment: a curve going on at the radius of the curved track before it doesn't change the curvature abruptly") {
	auto const adjoining = [](alignment::result const &Line) {
		return std::any_of(Line.warnings.begin(), Line.warnings.end(), [](std::string const &Warning) { return Warning.find("adjoining track at the start") != std::string::npos; });
	};
	// R 300 with no transition from the very start, as a curve after the diverging track of a switch, R 301 there
	auto const tangent{300.0 * std::tan(10.0 * pi / 180.0)};
	auto design{straights({tangent, 200.0}, {20.0 * pi / 180.0}, 40.0)};
	auto &vertex{design.vertices.front()};
	vertex.radius = 300.0;
	vertex.transition_in = 0.0;
	vertex.transition_out = 0.0;
	design.start_radius = 301.0;
	design.start_turn = 1;
	auto const line{alignment::compute(design)};
	REQUIRE(line.valid);
	REQUIRE_FALSE(line.elements.empty());
	CHECK(line.elements.front().kind == alignment::element_kind::arc);
	CHECK_FALSE(adjoining(line));
	// the track before turning the other way, or the curve starting with a straight: the curvature jumps
	design.start_turn = -1;
	CHECK(adjoining(alignment::compute(design)));
	design.start_turn = 1;
	auto later{straights({tangent + 80.0, 200.0}, {20.0 * pi / 180.0}, 40.0)};
	later.vertices.front() = vertex;
	later.vertices.front().position = {tangent + 80.0, 0.0};
	later.start_radius = 301.0;
	later.start_turn = 1;
	CHECK(adjoining(alignment::compute(later)));
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

TEST_CASE("alignment: a curve laid on from the diverging track of a switch begins at its end with its curvature") {
	auto const adjoining = [](alignment::result const &Line) {
		return std::any_of(Line.warnings.begin(), Line.warnings.end(), [](std::string const &Warning) { return Warning.find("adjoining track at the start") != std::string::npos; });
	};
	// R 300 on for 100 m, then a transition of 40 m out to the straight, at 40 km/h
	auto design{curved_from(300.0, {{1.0 / 300.0, 1.0 / 300.0, 100.0}, {1.0 / 300.0, 0.0, 40.0}}, 40.0)};
	auto &vertex{design.vertices.front()};
	vertex.radius = 1000.0;
	vertex.transition_in = 0.0;
	vertex.transition_out = 40.0;
	auto const line{alignment::compute(design)};
	REQUIRE(line.valid);
	REQUIRE(line.curves.size() == 1);
	auto const &curve{line.curves.front()};
	// the radius is the one which begins the curve at the start, whatever was given
	CHECK(curve.radius == doctest::Approx(300.0).epsilon(1e-3));
	CHECK(curve.curvature_in == doctest::Approx(1.0 / 300.0));
	// no straight before it: the curvature of the switch from the very start, which is where it was
	CHECK(line.elements.front().kind != alignment::element_kind::straight);
	CHECK(curvature_of(line, 0.25) == doctest::Approx(1.0 / 300.0).epsilon(0.01));
	auto const start{alignment::evaluate(line, 0.0)};
	CHECK(std::abs(start.position.x) < 1e-6);
	CHECK(std::abs(start.position.z) < 1e-6);
	CHECK(std::abs(start.direction.z) < 1e-6);
	CHECK_FALSE(adjoining(line));
	CHECK(alignment::meets_norms(curve, design));
	// the line through the end as given
	auto const end{alignment::evaluate(line, line.length)};
	CHECK(glm::distance(glm::dvec2{end.position.x, end.position.z}, glm::dvec2{design.end.x, design.end.z}) < 1e-3);

	// the track before turning the other way: the curve from a straight, the jump of the curvature left to the one laying it, which eases
	// out of the curve there; a design not asked to go on from it, as the plan of a line laid before, warns of it
	auto other{design};
	other.start_turn = -1;
	auto const jump{alignment::compute(other)};
	REQUIRE(jump.valid);
	CHECK(jump.curves.front().curvature_in == doctest::Approx(0.0));
	CHECK_FALSE(adjoining(jump));
	other.start_curved = false;
	CHECK(adjoining(alignment::compute(other)));
}

TEST_CASE("alignment: the cant of the curved track adjoining goes on into the curve with no transition between them") {
	auto design{curved_from(600.0, {{1.0 / 600.0, 1.0 / 600.0, 150.0}, {1.0 / 600.0, 0.0, 80.0}}, 100.0)};
	design.start_cant = 60.0;
	auto &vertex{design.vertices.front()};
	vertex.radius = 600.0;
	vertex.transition_out = 80.0;
	vertex.cant = 100.0;
	auto const line{alignment::compute(design)};
	REQUIRE(line.valid);
	// the cant of the track adjoining from the very start, ramped out along the transition to the straight
	CHECK(alignment::evaluate(line, 0.0).cant == doctest::Approx(60.0));
	CHECK(alignment::evaluate(line, 50.0).cant == doctest::Approx(60.0));
	auto const out{line.elements.back().kind == alignment::element_kind::straight ? line.elements[line.elements.size() - 2] : line.elements.back()};
	CHECK(alignment::evaluate(line, out.chainage + out.length * 0.5).cant == doctest::Approx(30.0).epsilon(0.02));
	CHECK(alignment::evaluate(line, out.chainage + out.length + 1.0).cant == doctest::Approx(0.0));
	REQUIRE(line.ramps.size() == 1);
	CHECK(line.ramps.front().from == doctest::Approx(out.chainage));
	// the paths roll with the cant from the first one
	auto const paths{alignment::pieces(line, design, alignment::minimum_pieces(line, design))};
	REQUIRE_FALSE(paths.empty());
	CHECK(paths.front().rolls[0] == doctest::Approx(glm::degrees(std::asin(60.0 / design.norms.gauge))).epsilon(1e-3));
}

TEST_CASE("alignment: a curve traced on from the diverging track of a switch keeps its curvature from the start") {
	// the curve on the photo: R 300 from the end of the switch for 80 m, a transition of 25 m out, then the straight, at 40 km/h
	auto truth{curved_from(300.0, {{1.0 / 300.0, 1.0 / 300.0, 80.0}, {1.0 / 300.0, 0.0, 25.0}}, 40.0)};
	auto &vertex{truth.vertices.front()};
	vertex.radius = 300.0;
	vertex.transition_out = 25.0;
	auto const line{alignment::compute(truth)};
	REQUIRE(line.valid);
	auto const points{traced(line, 10.0, 0.02)};

	auto const base{stretch_base(truth, points)};
	auto const fit{alignment::fit_trace_stretch(base, points, 0, true)};

	REQUIRE(fit.valid);
	REQUIRE(fit.vertices.size() == 1);
	auto const &curve{fit.vertices.front()};
	CHECK(curve.radius == doctest::Approx(300.0).epsilon(0.02));
	CHECK(curve.transition_in < 8.0);
	CHECK(curve.transition_out == doctest::Approx(25.0).epsilon(0.2));
	CHECK(fit.rms < 0.03);
	// clicked exactly, the diverging track read as R 300.9 as the editor reads a switch: the curve goes on at its radius with no transition
	// between them and with the cant of the switch, none
	auto const exact{traced(line, 10.0, 0.0)};
	auto exactbase{stretch_base(truth, exact)};
	exactbase.start_radius = 300.9;
	auto const along{alignment::fit_trace_stretch(exactbase, exact, 0, true)};
	REQUIRE(along.valid);
	REQUIRE(along.vertices.size() == 1);
	CHECK(along.vertices.front().transition_in == doctest::Approx(0.0));
	CHECK(along.vertices.front().transition_out == doctest::Approx(25.0).epsilon(0.05));
	CHECK(along.vertices.front().radius == doctest::Approx(300.0).epsilon(0.005));
	CHECK(along.vertices.front().cant == doctest::Approx(0.0));
	CHECK(along.rms < 0.005);
	// laid as fitted: the curvature of the switch at the very start
	auto laid{base};
	laid.vertices = fit.vertices;
	auto const shape{alignment::compute(laid)};
	REQUIRE(shape.valid);
	CHECK(shape.elements.front().kind != alignment::element_kind::straight);
	CHECK(curvature_of(shape, 0.25) == doctest::Approx(1.0 / 300.0).epsilon(0.02));

	// from a track turning the other way the curve is fitted from a straight, as before
	auto other{base};
	other.start_turn = -1;
	auto const straight{alignment::fit_trace_stretch(other, points, 0, true)};
	REQUIRE(straight.valid);
	REQUIRE_FALSE(straight.vertices.empty());
	auto apart{other};
	apart.vertices = straight.vertices;
	auto const from{alignment::compute(apart)};
	REQUIRE(from.valid);
	CHECK(from.curves.front().curvature_in == doctest::Approx(0.0));
}

TEST_CASE("alignment: a curve traced on from a curved free end through a transition into a larger radius") {
	// R 300 at the free end, a transition of 30 m into R 500 for 60 m, 40 m out to the straight, at 60 km/h
	auto truth{curved_from(300.0, {{1.0 / 300.0, 1.0 / 500.0, 30.0}, {1.0 / 500.0, 1.0 / 500.0, 60.0}, {1.0 / 500.0, 0.0, 40.0}}, 60.0)};
	auto &vertex{truth.vertices.front()};
	vertex.radius = 500.0;
	vertex.transition_in = 30.0;
	vertex.transition_out = 40.0;
	auto const line{alignment::compute(truth)};
	REQUIRE(line.valid);
	REQUIRE(line.curves.size() == 1);
	CHECK(line.curves.front().radius == doctest::Approx(500.0).epsilon(1e-3));

	// clicked exactly, the curve found again
	auto const exact{traced(line, 10.0, 0.0)};
	auto const fit{alignment::fit_trace_stretch(stretch_base(truth, exact), exact, 0, true)};
	REQUIRE(fit.valid);
	REQUIRE(fit.vertices.size() == 1);
	auto const &curve{fit.vertices.front()};
	CHECK(curve.radius == doctest::Approx(500.0).epsilon(0.01));
	CHECK(curve.transition_in == doctest::Approx(30.0).epsilon(0.05));
	CHECK(curve.transition_out == doctest::Approx(40.0).epsilon(0.05));
	CHECK(fit.rms < 0.01);

	// clicked 2 cm off: the straight after the curve, through the last two points, a little off too, the curve keeps to the points as well
	auto const points{traced(line, 10.0, 0.02)};
	auto const noisy{alignment::fit_trace_stretch(stretch_base(truth, points), points, 0, true)};
	REQUIRE(noisy.valid);
	REQUIRE(noisy.vertices.size() == 1);
	CHECK(noisy.vertices.front().radius == doctest::Approx(500.0).epsilon(0.03));
	CHECK(noisy.rms < 0.025);
}

TEST_CASE("alignment: a bend clicked ahead of a curved free end, turning its way, is eased with a curve going on from it") {
	alignment::design design;
	design.speed = 60.0;
	design.start_direction = heading(0.0);
	design.start_radius = 300.0;
	design.start_turn = 1;
	design.start_curved = true;
	// the bend 150 m ahead, 20 degrees to the left, the end 300 m after it
	alignment::vertex bend;
	bend.position = {150.0, 0.0};
	design.vertices.push_back(bend);
	auto const end{bend.position + heading(20.0 * pi / 180.0) * 300.0};
	design.end = {end.x, 0.0, end.y};
	design.end_direction = heading(20.0 * pi / 180.0);
	std::vector<char> corners{1};
	auto const line{alignment::ease_corners(design, corners, {3000.0, 6000.0})};

	REQUIRE(line.valid);
	REQUIRE(line.curves.size() == 1);
	auto const &curve{line.curves.front()};
	CHECK(line.elements.front().kind != alignment::element_kind::straight);
	CHECK(curvature_of(line, 0.25) == doctest::Approx(1.0 / 300.0).epsilon(0.02));
	// all of the straight before the bend taken by the curve, larger than the one it goes on from, through a transition between them
	CHECK(curve.tangent_in == doctest::Approx(150.0).epsilon(1e-4));
	CHECK(curve.radius > 400.0);
	CHECK(curve.transition_in > 5.0);
	CHECK(alignment::meets_norms(curve, design));
	CHECK(line.vertices.front().x == doctest::Approx(150.0));
	CHECK(std::abs(line.vertices.front().y) < 1e-9);
	// the design keeps the radius it's laid with
	CHECK(design.vertices.front().radius == doctest::Approx(curve.radius).epsilon(1e-6));

	// a second bend after it: the curve going on from the free end keeps its size, the next one makes room for it
	auto two{design};
	two.vertices = {bend, bend};
	two.vertices[1].position = bend.position + heading(20.0 * pi / 180.0) * 220.0;
	auto const after{two.vertices[1].position + heading(-5.0 * pi / 180.0) * 300.0};
	two.end = {after.x, 0.0, after.y};
	two.end_direction = heading(-5.0 * pi / 180.0);
	two.vertices.front().offset = 150.0;
	two.vertices.back().offset = 300.0;
	std::vector<char> both{1, 1};
	auto const pair{alignment::ease_corners(two, both, {3000.0, 6000.0})};
	REQUIRE(pair.valid);
	REQUIRE(pair.curves.size() == 2);
	CHECK(pair.curves[0].radius == doctest::Approx(curve.radius).epsilon(1e-3));
	CHECK(pair.curves[0].tangent_out + pair.curves[1].tangent_in <= 220.0 + 1e-3);
	CHECK(curvature_of(pair, 0.25) == doctest::Approx(1.0 / 300.0).epsilon(0.02));
}

TEST_CASE("alignment: a curve laid into a curved free end ends at it with its curvature") {
	// a straight of 100 m, a transition of 40 m into R 400 to the right, the arc 60 m on to the free end
	auto const end{walked({{0.0, 0.0, 100.0}, {0.0, -1.0 / 400.0, 40.0}, {-1.0 / 400.0, -1.0 / 400.0, 60.0}})};
	alignment::design design;
	design.speed = 80.0;
	design.start_direction = heading(0.0);
	design.end = {end.position.x, 0.0, end.position.y};
	design.end_direction = heading(end.heading);
	design.end_radius = 400.0;
	design.end_turn = -1;
	design.end_curved = true;
	alignment::vertex vertex;
	vertex.radius = 2000.0;
	vertex.transition_in = 40.0;
	design.vertices.push_back(vertex);
	auto const line{alignment::compute(design)};
	REQUIRE(line.valid);
	REQUIRE(line.curves.size() == 1);
	CHECK(line.curves.front().radius == doctest::Approx(400.0).epsilon(1e-3));
	CHECK(line.curves.front().curvature_out == doctest::Approx(1.0 / 400.0));
	CHECK(line.elements.back().kind != alignment::element_kind::straight);
	CHECK(curvature_of(line, line.length - 0.25) == doctest::Approx(-1.0 / 400.0).epsilon(0.01));
	auto const last{alignment::evaluate(line, line.length)};
	CHECK(glm::distance(glm::dvec2{last.position.x, last.position.z}, end.position) < 1e-3);
	CHECK(std::none_of(line.warnings.begin(), line.warnings.end(), [](std::string const &Warning) { return Warning.find("adjoining track at the end") != std::string::npos; }));
}

TEST_CASE("alignment: the pieces of a compound curve are never of no length, whatever its arcs and transitions") {
	std::mt19937 random{20261025};
	auto const uniform = [&](double const Low, double const High) { return std::uniform_real_distribution<double>(Low, High)(random); };
	auto const pick = [&](int const Count) { return std::uniform_int_distribution<int>(0, Count - 1)(random); };
	int bad{0}, valid{0};
	for (int run = 0; run < 4000 && bad < 5; ++run) {
		auto design{straights({600.0 + uniform(0.0, 800.0), 600.0 + uniform(0.0, 800.0)}, {uniform(20.0, 110.0) * pi / 180.0}, 100.0)};
		design.transition_pieces = 1 + pick(2);
		design.arc_piece_angle = pick(2) == 0 ? 10.0 : 2.0;
		auto &vertex{design.vertices.front()};
		vertex.radius = uniform(300.0, 1500.0);
		vertex.transition_in = pick(4) == 0 ? 0.0 : uniform(0.0, 200.0);
		vertex.transition_out = pick(4) == 0 ? 0.0 : uniform(0.0, 200.0);
		vertex.cant = pick(3) == 0 ? 0.0 : uniform(0.0, 150.0);
		vertex.compound = true;
		vertex.share = pick(5) == 0 ? 0.0 : uniform(0.0, 1.5);
		for (int i = 0, count = 1 + pick(2); i < count; ++i)
			vertex.arcs.push_back({uniform(300.0, 1500.0), pick(3) == 0 ? 0.0 : uniform(0.0, 120.0), pick(5) == 0 ? 0.0 : uniform(0.0, 0.5)});
		auto const line{alignment::compute(design)};
		if (false == line.valid)
			continue;
		++valid;
		// as the plan lays them over the paths there, each kept between the joints it had, or all over again
		std::vector<glm::dvec2> joints;
		auto const count{static_cast<std::size_t>(10 + pick(80))};
		for (std::size_t i = 1; i < count; ++i) {
			auto const at{alignment::evaluate(line, line.length * i / count + (pick(2) == 0 ? 0.0 : uniform(-3.0, 3.0)))};
			joints.emplace_back(at.position.x, at.position.z);
		}
		std::vector<std::size_t> intervals;
		for (auto const &paths : {alignment::pieces_at(line, design, count, joints, intervals), alignment::pieces(line, design, count)})
			for (std::size_t i = 0; i < paths.size(); ++i) {
				auto const &path{paths[i]};
				auto const length{glm::distance(path.points[segment_data::point::start], path.points[segment_data::point::end])};
				auto const finite{std::isfinite(length) && std::isfinite(path.points[segment_data::point::control1].x) && std::isfinite(path.points[segment_data::point::control2].x)};
				// the simulation takes a path under 5 cm for one of no length
				if (finite && length >= 0.05)
					continue;
				++bad;
				std::string arcs;
				for (auto const &arc : vertex.arcs)
					arcs += " [R " + std::to_string(arc.radius) + " L " + std::to_string(arc.transition) + " share " + std::to_string(arc.share) + "]";
				MESSAGE("piece " << i << " of " << paths.size() << ": " << length << " m; R " << vertex.radius << " L " << vertex.transition_in << " / " << vertex.transition_out << " cant " << vertex.cant << " share " << vertex.share << arcs << ", count " << count << ", pieces " << design.transition_pieces << " angle " << design.arc_piece_angle);
				break;
			}
	}
	MESSAGE(valid << " valid designs");
	CHECK(bad == 0);
}

namespace {
glm::dvec2 heading_of(double const Angle) { return heading(Angle); }

// points every Spacing metres along a line from the origin along x of segments of the curvature going evenly from the first to the
// second (to the left positive) over the length, and two on the straight after it 30 and 60 m on; the end and the heading there
std::vector<glm::dvec2> walked_points(std::vector<std::array<double, 3>> const &Segments, double const Spacing, line_end &End) {
	std::vector<glm::dvec2> points;
	glm::dvec2 position{0.0};
	double heading{0.0};
	double chainage{0.0};
	auto next{Spacing};
	for (auto const &[from, to, length] : Segments) {
		int const steps{std::max(1, static_cast<int>(std::ceil(length / 0.05)))};
		auto const step{length / steps};
		auto const start{heading};
		auto const angle = [&, from = from, to = to, length = length](double const S) { return start + from * S + (to - from) * S * S / (2.0 * length); };
		for (int i = 0; i < steps; ++i) {
			position += heading_of(angle((i + 0.5) * step)) * step;
			chainage += step;
			if (chainage >= next - 1e-6) {
				points.push_back(position);
				next += Spacing;
			}
		}
		heading = angle(length);
	}
	points.push_back(position + heading_of(heading) * 30.0);
	points.push_back(position + heading_of(heading) * 60.0);
	End.position = position + heading_of(heading) * 60.0;
	End.heading = heading;
	return points;
}
} // namespace

TEST_CASE("alignment: reverse curves traced in the mode of a curve, with no straight between them or with one, two or three curves") {
	double const k{1.0 / 300.0};
	struct shape {
		char const *name;
		std::vector<std::array<double, 3>> segments;
		double speed;
		std::vector<double> radii;
		double noise;
	};
	std::vector<shape> const shapes{
	    {"transitions meeting at the inflection", {{0, 0, 20}, {0, k, 30}, {k, k, 50}, {k, -k, 60}, {-k, -k, 50}, {-k, 0, 30}}, 100.0, {300.0, 300.0}, 0.0},
	    {"the same clicked 5 cm off", {{0, 0, 20}, {0, k, 30}, {k, k, 50}, {k, -k, 60}, {-k, -k, 50}, {-k, 0, 30}}, 60.0, {300.0, 300.0}, 0.05},
	    {"a straight between", {{0, 0, 20}, {0, k, 30}, {k, k, 50}, {k, 0, 30}, {0, 0, 30}, {0, -k, 30}, {-k, -k, 50}, {-k, 0, 30}}, 60.0, {300.0, 300.0}, 0.0},
	    {"arcs meeting", {{0, 0, 20}, {k, k, 60}, {-k, -k, 60}}, 40.0, {300.0, 300.0}, 0.0},
	    {"other radii", {{0, 0, 20}, {0, 0.5 * k, 40}, {0.5 * k, 0.5 * k, 60}, {0.5 * k, -0.75 * k, 70}, {-0.75 * k, -0.75 * k, 40}, {-0.75 * k, 0, 40}}, 80.0, {600.0, 400.0}, 0.0},
	    {"the ends not parallel", {{0, 0, 20}, {0, k, 30}, {k, k, 90}, {k, -k, 60}, {-k, -k, 30}, {-k, 0, 30}}, 60.0, {300.0, 300.0}, 0.0},
	    {"left, right, left", {{0, 0, 20}, {0, k, 30}, {k, k, 40}, {k, -k, 60}, {-k, -k, 40}, {-k, k, 60}, {k, k, 40}, {k, 0, 30}}, 60.0, {300.0, 300.0, 300.0}, 0.0},
	    {"left, right, left clicked 5 cm off", {{0, 0, 20}, {0, k, 30}, {k, k, 40}, {k, -k, 60}, {-k, -k, 40}, {-k, k, 60}, {k, k, 40}, {k, 0, 30}}, 60.0, {300.0, 300.0, 300.0}, 0.05},
	};
	for (auto const &each : shapes) {
		std::string const name{each.name};
		CAPTURE(name);
		line_end end;
		auto points{walked_points(each.segments, 10.0, end)};
		// off to and fro across the way, as clicks are
		for (std::size_t i = 0; i < points.size(); ++i)
			points[i].y += each.noise * (i % 3 == 0 ? 1.0 : i % 3 == 1 ? -0.6 : 0.2);
		alignment::design base;
		base.speed = each.speed;
		base.start = {0.0, 0.0, 0.0};
		base.start_direction = heading(0.0);
		base.end = {end.position.x, 0.0, end.position.y};
		base.end_direction = heading(end.heading);
		auto const fit{alignment::fit_trace_stretch(base, points, 0, true)};
		REQUIRE(fit.valid);
		REQUIRE(fit.vertices.size() == each.radii.size());
		for (std::size_t i = 0; i < each.radii.size(); ++i)
			CHECK(fit.vertices[i].radius == doctest::Approx(each.radii[i]).epsilon(0.03));
		CHECK(fit.rms < 0.02 + each.noise);
		// laid as the design of the curves fitted, with no warning of radii made smaller
		auto laid{base};
		laid.vertices = fit.vertices;
		for (std::size_t i = 1; i + 1 < laid.vertices.size(); ++i)
			laid.vertices[i].position = fit.vertices[i].position;
		auto const line{alignment::compute(laid)};
		CHECK(line.valid);
		CHECK(line.radius_warnings.empty());
	}
}

TEST_CASE("alignment: a straight between two curves turning the same way, traced in the mode of a curve, is laid as a straight") {
	// the points clicked in a scenery, from the start of the straight before the curves on, a curve of about 550 m, about 200 m of
	// straight, a curve of about 550 m turning the same way and the end; once fitted as one compound curve of eight arcs, its straight
	// two arcs of R 10360
	std::vector<glm::dvec2> const clicks{{-1495.4964, 300.4092}, {-1653.8092, 290.0271}, {-1680.8739, 287.7361}, {-1707.4353, 284.2790}, {-1739.5235, 278.3567}, {-1780.0393, 268.1851},
	                                     {-1803.7878, 260.7141}, {-1845.8626, 244.5926}, {-1872.4781, 232.3500}, {-1917.3815, 210.9899}, {-1955.5608, 192.7801}, {-2037.0277, 153.8567},
	                                     {-2053.8111, 145.8379}, {-2077.7168, 132.9264}, {-2087.3488, 127.3010}, {-2097.1702, 121.5480}};
	alignment::design base;
	base.speed = 100.0;
	base.start = {clicks.front().x, 0.0, clicks.front().y};
	base.start_direction = glm::normalize(clicks[1] - clicks[0]);
	base.end = {clicks.back().x, 0.0, clicks.back().y};
	base.end_direction = glm::normalize(clicks.back() - clicks[clicks.size() - 2]);
	std::vector<glm::dvec2> const points(clicks.begin() + 1, clicks.end());
	auto const fit{alignment::fit_trace_stretch(base, points, 0, true)};
	REQUIRE(fit.valid);
	auto laid{base};
	laid.vertices = fit.vertices;
	auto const line{alignment::compute(laid)};
	REQUIRE(line.valid);
	// no arc flatter than R 3000 between the curves, the straight a straight
	for (auto const &element : line.elements)
		if (element.vertex >= 0 && element.radius > 0.0)
			CHECK(element.radius < 3000.0);
	CHECK(line.curves.size() == 2);
	CHECK(fit.worst < 1.0);
}

TEST_CASE("alignment: curves turning the same way with straights between them, traced in the mode of a curve, two or three") {
	double const k{1.0 / 300.0};
	double const m{1.0 / 500.0};
	struct shape {
		char const *name;
		std::vector<std::array<double, 3>> segments;
		std::vector<double> radii;
		double noise;
	};
	std::vector<shape> const shapes{
	    {"two, a straight of 150 m between", {{0, 0, 20}, {0, k, 40}, {k, k, 60}, {k, 0, 40}, {0, 0, 150}, {0, m, 40}, {m, m, 40}, {m, 0, 40}}, {300.0, 500.0}, 0.0},
	    {"the same clicked 5 cm off", {{0, 0, 20}, {0, k, 40}, {k, k, 60}, {k, 0, 40}, {0, 0, 150}, {0, m, 40}, {m, m, 40}, {m, 0, 40}}, {300.0, 500.0}, 0.05},
	    {"three, straights of 120 m between", {{0, 0, 20}, {0, k, 40}, {k, k, 40}, {k, 0, 40}, {0, 0, 120}, {0, m, 40}, {m, m, 60}, {m, 0, 40}, {0, 0, 120}, {0, k, 40}, {k, k, 40}, {k, 0, 40}},
	     {300.0, 500.0, 300.0}, 0.0},
	};
	for (auto const &each : shapes) {
		std::string const name{each.name};
		CAPTURE(name);
		line_end end;
		auto points{walked_points(each.segments, 10.0, end)};
		for (std::size_t i = 0; i < points.size(); ++i)
			points[i].y += each.noise * (i % 3 == 0 ? 1.0 : i % 3 == 1 ? -0.6 : 0.2);
		alignment::design base;
		base.speed = 60.0;
		base.start = {0.0, 0.0, 0.0};
		base.start_direction = heading(0.0);
		base.end = {end.position.x, 0.0, end.position.y};
		base.end_direction = heading(end.heading);
		auto const fit{alignment::fit_trace_stretch(base, points, 0, true)};
		REQUIRE(fit.valid);
		REQUIRE(fit.vertices.size() == each.radii.size());
		for (std::size_t i = 0; i < each.radii.size(); ++i) {
			CHECK_FALSE(fit.vertices[i].compound);
			CHECK(fit.vertices[i].radius == doctest::Approx(each.radii[i]).epsilon(0.03));
		}
		CHECK(fit.rms < 0.02 + each.noise);
	}
}
