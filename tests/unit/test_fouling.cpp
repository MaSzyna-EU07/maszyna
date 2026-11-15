#include "stdafx.h"
#include "doctest.h"
#include "editor/editorFouling.hpp"
#include "editor/editorGauge.hpp"

#include <cmath>
#include <vector>

namespace {

// the axis from the origin along z, a point each metre; Radius 0 straight, else a curve towards +x (Radius > 0) or -x
std::vector<glm::dvec3> axis(double const Radius, double const Slope = 0.0, double const Length = 150.0) {
	std::vector<glm::dvec3> points;
	auto const metres{static_cast<int>(std::floor(Length + 1e-9))};
	for (int metre = 0; metre <= metres; ++metre) {
		auto const s{static_cast<double>(metre)};
		if (Radius == 0.0) {
			auto const along{s / std::sqrt(1.0 + Slope * Slope)};
			points.emplace_back(along * Slope, 0.0, along);
		}
		else {
			auto const r{std::abs(Radius)};
			auto const side{Radius > 0.0 ? 1.0 : -1.0};
			points.emplace_back(side * r * (1.0 - std::cos(s / r)), 0.0, r * std::sin(s / r));
		}
	}
	return points;
}

} // namespace

TEST_CASE("the widening of the gauge from the radius is that of table 1 of the standard, none from 250 m up") {
	CHECK(gauge::curve_widening(200.0, true) == doctest::Approx(50.0 / 200.0 - 0.185));
	CHECK(gauge::curve_widening(200.0, false) == doctest::Approx(60.0 / 200.0 - 0.225));
	CHECK(gauge::curve_widening(180.0, false) == doctest::Approx(60.0 / 180.0 - 0.225));
	CHECK(gauge::curve_widening(250.0, true) == 0.0);
	CHECK(gauge::curve_widening(300.0, false) == 0.0);
	CHECK(gauge::curve_widening(0.0, false) == 0.0);
	// the table ends at 150 m: the sharper curves get its value there
	CHECK(gauge::curve_widening(40.0, true) == doctest::Approx(50.0 / 150.0 - 0.185));
	CHECK(gauge::curve_widening(100.0, false) == doctest::Approx(60.0 / 150.0 - 0.225));
}

TEST_CASE("a straight branch at 1:9 off a straight track fouls it where the axes are 3.50 m apart") {
	fouling::place place;
	REQUIRE(fouling::find(axis(0.0), axis(0.0, 1.0 / 9.0), fouling::kNominal, place));
	CHECK(place.spacing == doctest::Approx(3.5));
	CHECK(place.main.z == doctest::Approx(31.5).epsilon(0.002));
	CHECK(place.branch.x == doctest::Approx(3.5).epsilon(0.002));
}

TEST_CASE("a branch in a curve of 300 m fouls at 3.50 m, the gauge including the curve") {
	fouling::place place;
	REQUIRE(fouling::find(axis(0.0), axis(300.0), fouling::kNominal, place));
	CHECK(place.spacing == doctest::Approx(3.5));
	CHECK(place.main.z == doctest::Approx(std::sqrt(300.0 * 300.0 - 296.5 * 296.5)).epsilon(0.002));
}

TEST_CASE("a branch in a curve under 250 m fouls farther: the widening on the outer side of its curve is added") {
	// the straight track lies on the outer side of the curve of the branch
	fouling::place place;
	REQUIRE(fouling::find(axis(0.0), axis(200.0), fouling::kNominal, place));
	auto const wanted{3.5 + 60.0 / 200.0 - 0.225};
	CHECK(place.spacing == doctest::Approx(wanted).epsilon(0.001));
	CHECK(place.branch.x == doctest::Approx(wanted).epsilon(0.002));
	CHECK(place.main.z == doctest::Approx(std::sqrt(200.0 * 200.0 - (200.0 - wanted) * (200.0 - wanted))).epsilon(0.002));
}

TEST_CASE("both tracks in curves under 250 m: the widenings of both are summed, each on the side of the other track") {
	// the branch curves sharper, inside the curve of the main track: on the inner side of the main track, which lies on the outer
	// side of the branch
	fouling::place place;
	REQUIRE(fouling::find(axis(200.0), axis(180.0), fouling::kNominal, place));
	auto const wanted{3.5 + (50.0 / 200.0 - 0.185) + (60.0 / 180.0 - 0.225)};
	CHECK(place.spacing == doctest::Approx(wanted).epsilon(0.001));
	// the branch there that far from the axis of the main track, the circle about (200, 0)
	auto const off{200.0 - std::hypot(place.branch.x - 200.0, place.branch.z)};
	CHECK(off == doctest::Approx(wanted).epsilon(0.003));
}

TEST_CASE("the cars of a track on a cant lean towards the other track when its lower rail is on that side, at any radius") {
	// the curves of axis() turn to the right of the way they run, towards +x: the cant of a curve lowers its right rail,
	// negative as widening_at takes it; positive the other way round
	std::vector<double> const cant(200, -0.1);
	std::vector<double> const against(200, 0.1);
	auto const lean{0.1 * fouling::kLeanHeight / 1.5};
	CHECK(fouling::kLeanHeight == doctest::Approx(3.25));
	CHECK(gauge::cant_widening(-0.1, fouling::kLeanHeight) == doctest::Approx(lean));
	fouling::place place;
	// the branch inside the curve of the main track, on its inner side: the main track on the cant leans towards it
	REQUIRE(fouling::find(axis(200.0), axis(180.0), fouling::kNominal, place, cant, {}));
	CHECK(place.spacing == doctest::Approx(3.5 + (50.0 / 200.0 - 0.185) + lean + (60.0 / 180.0 - 0.225)).epsilon(0.001));
	CHECK(place.cant == doctest::Approx(lean).epsilon(0.001));
	// the branch on the cant leans away from the main track, on the outer side of its curve: nothing for it
	REQUIRE(fouling::find(axis(0.0), axis(200.0), fouling::kNominal, place, {}, cant));
	CHECK(place.spacing == doctest::Approx(3.5 + 60.0 / 200.0 - 0.225).epsilon(0.001));
	CHECK(place.cant == doctest::Approx(0.0));
	// a branch on a cant against its curve leans towards the main track on the outer side of the curve: the lean counts
	REQUIRE(fouling::find(axis(0.0), axis(200.0), fouling::kNominal, place, {}, against));
	CHECK(place.spacing == doctest::Approx(3.5 + 60.0 / 200.0 - 0.225 + lean).epsilon(0.001));
	CHECK(place.cant == doctest::Approx(lean).epsilon(0.001));
	// from 250 m up no widening from the radius, but the cars lean on the cant all the same
	REQUIRE(fouling::find(axis(300.0), axis(260.0), fouling::kNominal, place, cant, {}));
	CHECK(place.spacing == doctest::Approx(3.5 + lean).epsilon(0.001));
	// and on the straight: a straight track on a cant leaning towards a straight branch
	std::vector<double> const left(200, 0.1);
	REQUIRE(fouling::find(axis(0.0), axis(0.0, -1.0 / 9.0), fouling::kNominal, place, left, {}));
	CHECK(place.spacing == doctest::Approx(3.5 + lean).epsilon(0.001));
}

TEST_CASE("a straight branch off a track curving away from it: the widening on the outer side of the curve of the main track") {
	fouling::place place;
	REQUIRE(fouling::find(axis(-200.0), axis(0.0, 1.0 / 9.0), fouling::kNominal, place));
	CHECK(place.spacing == doctest::Approx(3.5 + 60.0 / 200.0 - 0.225).epsilon(0.001));
}
