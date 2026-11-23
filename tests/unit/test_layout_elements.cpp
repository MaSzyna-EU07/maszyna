// the elements of the lines of the track geometry (editor/layout, editor-tracks-domain.md §2): the clothoid, the cubic parabola, the
// Bloss curve and the arc — the length, the curvature along, the point and the heading at a distance, the shift of the arc, a point
// offset from the axis — against the series and the formulas of the textbooks; the curve of a vertex between its tangents

#include "stdafx.h"
#include "doctest.h"
#include "editor/layout/layoutLine.hpp"

#include <cmath>

namespace {

using layout::element_kind;

double constexpr kPi{3.14159265358979323846};

glm::dvec2 const kOrigin{0.0, 0.0};
glm::dvec2 const kAlong{1.0, 0.0};

} // namespace

TEST_CASE("layout elements: a clothoid from a straight to R 600 over 120 m, its length, curvature, end and shift by the series") {
	double const R{600.0};
	double const L{120.0};
	auto const e{layout::make_spiral(element_kind::clothoid, kOrigin, kAlong, 0.0, 1.0 / R, L)};
	CHECK(e.length == L);
	CHECK(e.curvature(0.0) == 0.0);
	CHECK(e.curvature(L / 2.0) == doctest::Approx(0.5 / R).epsilon(1e-12));
	CHECK(e.curvature(L) == doctest::Approx(1.0 / R).epsilon(1e-12));
	// x = L - L^5 / (40 A^4) + L^9 / (3456 A^8), y = L^3 / (6 A^2) - L^7 / (336 A^6) + L^11 / (42240 A^10), A^2 = R L
	auto const a2{R * L};
	auto const x{L - std::pow(L, 5) / (40.0 * a2 * a2) + std::pow(L, 9) / (3456.0 * std::pow(a2, 4))};
	auto const y{std::pow(L, 3) / (6.0 * a2) - std::pow(L, 7) / (336.0 * std::pow(a2, 3)) + std::pow(L, 11) / (42240.0 * std::pow(a2, 5))};
	auto const end{e.position(L)};
	CHECK(end.x == doctest::Approx(x).epsilon(1e-9));
	CHECK(end.y == doctest::Approx(y).epsilon(1e-7));
	CHECK(e.local(L).angle == doctest::Approx(L / (2.0 * R)).epsilon(1e-12));
	auto const heading{e.tangent(L)};
	CHECK(std::atan2(heading.y, heading.x) == doctest::Approx(L / (2.0 * R)).epsilon(1e-12));
	// the shift of the arc, p = L^2 / 24R - L^4 / 2688R^3
	CHECK(e.shift() == doctest::Approx(L * L / (24.0 * R) - std::pow(L, 4) / (2688.0 * std::pow(R, 3))).epsilon(1e-5));
	// the same clothoid run out of the curve: the same shift, the curvature falling evenly
	auto const out{layout::make_spiral(element_kind::clothoid, kOrigin, kAlong, -1.0 / R, 0.0, L)};
	CHECK(out.shift() == doctest::Approx(e.shift()).epsilon(1e-12));
	CHECK(out.curvature(L / 4.0) == doctest::Approx(-0.75 / R).epsilon(1e-12));
}

TEST_CASE("layout elements: a clothoid between two arcs turns by the mean of their curvatures") {
	auto const e{layout::make_spiral(element_kind::clothoid, {10.0, 20.0}, glm::normalize(glm::dvec2{1.0, 1.0}), 1.0 / 1000.0, 1.0 / 400.0, 60.0)};
	CHECK(e.local(60.0).angle == doctest::Approx(0.5 * (1.0 / 1000.0 + 1.0 / 400.0) * 60.0).epsilon(1e-12));
	// the point at the distance is the integral of the heading: a few metres at a time by the chord of each
	glm::dvec2 point{10.0, 20.0};
	int constexpr steps{6000};
	for (int i = 0; i < steps; ++i) {
		auto const middle{(i + 0.5) * 60.0 / steps};
		point += e.tangent(middle) * (60.0 / steps);
	}
	CHECK(glm::distance(point, e.position(60.0)) < 1e-6);
}

TEST_CASE("layout elements: a cubic parabola of 100 m to R 500, its end, length, heading, shift and its run out of the curve") {
	double const R{500.0};
	double const X{100.0};
	auto const in{layout::make_parabola(kOrigin, kAlong, X, R, true)};
	// the length along it, X (1 + X^2 / 40R^2 - X^4 / 1152R^4) to the order kept
	CHECK(in.length == doctest::Approx(X * (1.0 + X * X / (40.0 * R * R) - std::pow(X, 4) / (1152.0 * std::pow(R, 4)))).epsilon(1e-8));
	auto const end{in.position(in.length)};
	CHECK(end.x == doctest::Approx(X).epsilon(1e-9));
	CHECK(end.y == doctest::Approx(X * X / (6.0 * R)).epsilon(1e-9));
	CHECK(in.local(in.length).angle == doctest::Approx(std::atan(X / (2.0 * R))).epsilon(1e-12));
	// the curvature at its end falls short of 1/R by the slope there
	CHECK(in.curvature(in.length) == doctest::Approx(1.0 / R / std::pow(1.0 + std::pow(X / (2.0 * R), 2), 1.5)).epsilon(1e-9));
	CHECK(in.curvature(0.0) == 0.0);
	// the shift: X^2 / 24R + 3X^4 / 128R^3 to the second order
	CHECK(in.shift() == doctest::Approx(X * X / (24.0 * R) + 3.0 * std::pow(X, 4) / (128.0 * std::pow(R, 3))).epsilon(1e-3));
	CHECK(in.shift() == doctest::Approx(X * X / (6.0 * R) - R * (1.0 - std::cos(std::atan(X / (2.0 * R))))).epsilon(1e-9));
	// run out of the curve onto the tangent, turning left as well: the mirror of the one into it
	auto const out{layout::make_parabola(kOrigin, kAlong, X, R, false)};
	CHECK(out.length == doctest::Approx(in.length).epsilon(1e-12));
	auto const angle{std::atan(X / (2.0 * R))};
	auto const beyond{out.position(out.length)};
	CHECK(beyond.x == doctest::Approx(X * std::cos(angle) + end.y * std::sin(angle)).epsilon(1e-9));
	CHECK(beyond.y == doctest::Approx(X * std::sin(angle) - end.y * std::cos(angle)).epsilon(1e-9));
	CHECK(out.local(out.length).angle == doctest::Approx(angle).epsilon(1e-12));
	CHECK(out.curvature(0.0) == doctest::Approx(in.curvature(in.length)).epsilon(1e-12));
	CHECK(out.curvature(out.length) == doctest::Approx(0.0));
	// turning right, the same mirrored
	auto const right{layout::make_parabola(kOrigin, kAlong, X, -R, true)};
	CHECK(right.position(right.length).y == doctest::Approx(-end.y).epsilon(1e-12));
	CHECK(right.curvature(right.length) == doctest::Approx(-in.curvature(in.length)).epsilon(1e-12));
}

TEST_CASE("layout elements: a Bloss curve from a straight to R 400 over 100 m: its curvature, heading and a shift of 3/5 of the clothoid's") {
	double const R{400.0};
	double const L{100.0};
	auto const e{layout::make_spiral(element_kind::bloss, kOrigin, kAlong, 0.0, 1.0 / R, L)};
	CHECK(e.curvature(L / 2.0) == doctest::Approx(0.5 / R).epsilon(1e-12));
	CHECK(e.curvature(L / 4.0) == doctest::Approx((3.0 / 16.0 - 2.0 / 64.0) / R).epsilon(1e-12));
	CHECK(e.curvature(L) == doctest::Approx(1.0 / R).epsilon(1e-12));
	// it turns as far as a clothoid of its length
	CHECK(e.local(L).angle == doctest::Approx(L / (2.0 * R)).epsilon(1e-12));
	// the shift of a Bloss curve, L^2 / 40R, is 3/5 of the one of a clothoid, L^2 / 24R
	CHECK(e.shift() == doctest::Approx(L * L / (40.0 * R)).epsilon(1e-3));
	auto const clothoid{layout::make_spiral(element_kind::clothoid, kOrigin, kAlong, 0.0, 1.0 / R, L)};
	CHECK(e.shift() / clothoid.shift() == doctest::Approx(0.6).epsilon(2e-3));
	// its point is the integral of its heading, as for the clothoid
	glm::dvec2 point{0.0};
	int constexpr steps{6000};
	for (int i = 0; i < steps; ++i)
		point += e.tangent((i + 0.5) * L / steps) * (L / steps);
	CHECK(glm::distance(point, e.position(L)) < 1e-6);
}

TEST_CASE("layout elements: an arc keeps its radius and its centre, points offset from it lie on the circles alongside") {
	double const R{300.0};
	auto const e{layout::make_arc({5.0, 7.0}, {0.0, 1.0}, -1.0 / R, 200.0)};
	// turning right from heading +z: the centre to the right, at +x
	glm::dvec2 const centre{5.0 + R, 7.0};
	for (double s = 0.0; s <= 200.0; s += 25.0) {
		CAPTURE(s);
		CHECK(glm::distance(e.position(s), centre) == doctest::Approx(R).epsilon(1e-12));
		CHECK(e.curvature(s) == -1.0 / R);
		// a track 4.5 m to the left runs on the circle 4.5 m larger
		CHECK(glm::distance(e.offset_point(s, 4.5), centre) == doctest::Approx(R + 4.5).epsilon(1e-12));
	}
	CHECK(e.local(200.0).angle == doctest::Approx(-200.0 / R).epsilon(1e-12));
	CHECK(e.shift() == 0.0);
	auto const straight{layout::make_straight({1.0, 2.0}, {0.6, 0.8}, 50.0)};
	CHECK(glm::distance(straight.offset_point(10.0, 2.0), glm::dvec2{1.0 + 6.0 - 1.6, 2.0 + 8.0 + 1.2}) < 1e-12);
}

TEST_CASE("layout elements: the curve of a vertex, R 800 and clothoids of 90 m, by the tangent length of the textbooks") {
	double const R{800.0};
	double const L{90.0};
	double const deflection{0.4};
	auto const fit{layout::fit_curve(layout::transition_shape::clothoid, deflection, R, L, L)};
	REQUIRE(fit.error.empty());
	REQUIRE(fit.elements.size() == 3);
	// T = (R + p) tan(alpha / 2) + x_m, x_m = L / 2 - L^3 / 240 R^2
	auto const p{fit.elements.front().shift()};
	auto const xm{L / 2.0 - std::pow(L, 3) / (240.0 * R * R)};
	CHECK(fit.tangent_in == doctest::Approx((R + p) * std::tan(deflection / 2.0) + xm).epsilon(1e-6));
	CHECK(fit.tangent_out == doctest::Approx(fit.tangent_in).epsilon(1e-9));
	CHECK(fit.arc == doctest::Approx(R * deflection - L).epsilon(1e-12));
	// the elements run on from one another
	for (std::size_t i = 1; i < fit.elements.size(); ++i) {
		auto const &a{fit.elements[i - 1]};
		auto const &b{fit.elements[i]};
		CHECK(glm::distance(a.position(a.length), b.origin) < 1e-12);
		CHECK(a.curvature(a.length) == doctest::Approx(b.curvature(0.0)).epsilon(1e-12));
	}
	// too long transitions for the deflection
	CHECK_FALSE(layout::fit_curve(layout::transition_shape::clothoid, 0.1, R, L, L).error.empty());
	// a curve turning right is the mirror
	auto const right{layout::fit_curve(layout::transition_shape::clothoid, -deflection, R, L, L)};
	CHECK(right.tangent_in == doctest::Approx(fit.tangent_in).epsilon(1e-12));
	CHECK(right.elements.back().position(right.elements.back().length).y == doctest::Approx(-fit.elements.back().position(fit.elements.back().length).y).epsilon(1e-12));
	(void)kPi;
}
