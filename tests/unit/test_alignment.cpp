#include "stdafx.h"
#include "doctest.h"
#include "editor/editorAlignment.hpp"

#include <cmath>

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
	auto const line{alignment::ease_corners(design, {1}, {3000.0, 6000.0})};

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
	auto const line{alignment::ease_corners(design, {1}, {3000.0, 6000.0})};

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
	auto const line{alignment::ease_corners(design, {1}, {3000.0, 6000.0})};

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
	auto const line{alignment::ease_corners(design, {1, 1}, {3000.0, 6000.0})};

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
	CHECK(fit.rms < 0.05);
	CHECK(fit.worst < 0.15);
}
