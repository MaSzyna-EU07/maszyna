// the engine of the norms (editor/norms): the formulas of the cant with the examples of ST-T1-A6, the tier of a value, and lines
// described as elements checked: the cant deficiency at the highest speed and the excess at the slow trains, the abrupt changes of the
// cant deficiency of curves turning alike and the other way, taken together over a short element, the rates of change along the ramps
// and the transitions; the profiles of the areas, the stricter one on a boundary; a line read from its samples as elements

#include "stdafx.h"
#include "doctest.h"
#include "editor/norms/norms.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

namespace {

using norms::element_kind;
using norms::parameter;
using norms::profile_id;
using norms::tier;

// a line of elements one after another along x, the curvature signed, positive to the left
class line_builder {
public:
	line_builder &straight(double const Length, double const Speed) { return add(element_kind::straight, Length, 0.0, 0.0, Speed); }
	line_builder &arc(double const Radius, double const Length, double const Speed) { return add(element_kind::arc, Length, 1.0 / Radius, 1.0 / Radius, Speed); }
	line_builder &transition(double const From, double const To, double const Length, double const Speed) {
		return add(element_kind::transition, Length, From != 0.0 ? 1.0 / From : 0.0, To != 0.0 ? 1.0 / To : 0.0, Speed);
	}
	// the cant, mm, from here on evenly to the given at the chainage
	line_builder &cant(double const Chainage, double const Cant) {
		m_line.cant.push_back({Chainage, Cant});
		return *this;
	}
	line_builder &features(unsigned const Features) {
		m_line.elements.back().features = Features;
		return *this;
	}
	norms::line done() {
		if (m_line.grades.empty()) {
			m_line.grades.push_back({0.0, m_end, 0.0, 0.0});
		}
		m_line.elements.front().open_start = true;
		m_line.elements.back().open_end = true;
		return m_line;
	}
	double end() const { return m_end; }

private:
	norms::line m_line;
	double m_end{0.0};

	line_builder &add(element_kind const Kind, double const Length, double const From, double const To, double const Speed) {
		norms::element element;
		element.kind = Kind;
		element.start = m_end;
		element.length = Length;
		element.curvature_start = From;
		element.curvature_end = To;
		element.speed = Speed;
		element.from = {m_end, 0.0};
		element.to = {m_end + Length, 0.0};
		m_line.elements.push_back(element);
		m_end += Length;
		return *this;
	}
};

norms::line_class mixed_main(double const Vmax = 100.0) {
	norms::line_class result;
	result.vmax = Vmax;
	result.kind = norms::traffic::mixed;
	result.track = norms::track_kind::main;
	result.load = 5.0;
	return result;
}

std::vector<norms::finding> evaluate(norms::line const &Line, norms::line_class const &Class, profile_id const Profile = profile_id::plk) {
	norms::area_map areas;
	areas.scenery = Profile;
	return norms::evaluate(Line, Class, areas);
}

std::vector<norms::finding> of(std::vector<norms::finding> const &Findings, parameter const What) {
	std::vector<norms::finding> result;
	std::ranges::copy_if(Findings, std::back_inserter(result), [What](norms::finding const &Finding) { return Finding.what == What; });
	return result;
}

} // namespace

TEST_CASE("norms engine: the cant of ST-T1-A6 załącznik 1, Z1-1, P160 R 2000 m, I 90 mm fast and 20 mm slow") {
	// the highest speed with I 90 mm at the fast trains and 20 mm at the slow ones of 120 km/h: 162 km/h
	CHECK(norms::speed_for_deficiencies(2000.0, 90.0, 20.0, 120.0) == doctest::Approx(162.0).epsilon(0.005));
	CHECK(norms::least_cant(160.0, 2000.0, 90.0) == doctest::Approx(61.0).epsilon(0.005));
	CHECK(norms::least_cant(120.0, 2000.0, 20.0) == doctest::Approx(65.0).epsilon(0.005));
}

TEST_CASE("norms engine: the cant of ST-T1-A6 załącznik 1, Z1-2, M160 R 1200 m, Vmin 80 km/h") {
	CHECK(norms::speed_for_deficiencies(1200.0, 90.0, 20.0, 80.0) == doctest::Approx(116.0).epsilon(0.005));
	// Dmin with Idop 130 mm, written 121 mm (121.7), Dmax with Edop 80 mm 143 mm, Deq 251 mm, Dreg 139 mm
	CHECK(norms::least_cant(160.0, 1200.0, 130.0) == doctest::Approx(121.7).epsilon(0.001));
	CHECK(norms::greatest_cant(80.0, 1200.0, 80.0) == doctest::Approx(143.0).epsilon(0.002));
	CHECK(norms::equilibrium_cant(160.0, 1200.0) == doctest::Approx(251.7).epsilon(0.001));
	CHECK(norms::regular_cant(160.0, 1200.0) == doctest::Approx(139.0).epsilon(0.003));
}

TEST_CASE("norms engine: the regular cant of the compound curve of ST-T1-A6 załącznik 2, 105, 96 and 83 mm, 95 mm on average") {
	auto const first{norms::regular_cant(160.0, 1590.0)};
	auto const second{norms::regular_cant(160.0, 1730.0)};
	auto const third{norms::regular_cant(160.0, 2000.0)};
	CHECK(std::round(first) == 105.0);
	CHECK(std::round(second) == 96.0);
	CHECK(std::round(third) == 83.0);
	CHECK(std::round((first + second + third) / 3.0) == 95.0);
	CHECK(norms::cant_deficiency(160.0, 1590.0, 95.0) == doctest::Approx(11.8 * 160 * 160 / 1590.0 - 95.0));
	CHECK(norms::cant_excess(100.0, 1590.0, 95.0) == doctest::Approx(95.0 - 11.8 * 100 * 100 / 1590.0));
	CHECK(norms::least_radius(160.0, 95.0, 130.0) == doctest::Approx(11.8 * 160 * 160 / 225.0));
	CHECK(norms::greatest_speed(1590.0, 95.0, 130.0) == doctest::Approx(std::sqrt(225.0 * 1590.0 / 11.8)));
}

TEST_CASE("norms engine: a cant gradient of ST-T1-A6 przykład Z3-2 and Z3-3, 2.28 and 2.25 mm/m, passes P1 within P2") {
	norms::query where;
	where.speed = 50.0;
	for (auto const gradient : {2.28, 2.25}) {
		CAPTURE(gradient);
		auto const verdict{norms::assess(parameter::cant_gradient, gradient, norms::limits(profile_id::plk, parameter::cant_gradient, where), norms::hard_limits(parameter::cant_gradient, where))};
		CHECK(verdict.level == tier::warn);
		CHECK(verdict.passed.value == doctest::Approx(2.0));
		CHECK(verdict.passed.from.place == "tab. 9.12");
	}
}

TEST_CASE("norms engine: a value beyond an unconfirmed extended limit warns, beyond a hard one it blocks") {
	norms::query where;
	where.speed = 240.0;
	where.features = norms::feature::switch_track | norms::feature::movable_frog;
	auto const limits{norms::limits(profile_id::plk, parameter::cant_deficiency, where)};
	auto const hard{norms::hard_limits(parameter::cant_deficiency, where)};
	auto const beyond_p2{norms::assess(parameter::cant_deficiency, 120.0, limits, hard)};
	CHECK(beyond_p2.level == tier::warn);
	CHECK_FALSE(beyond_p2.passed.confirmed);
	auto const beyond_tsi{norms::assess(parameter::cant_deficiency, 160.0, limits, hard)};
	CHECK(beyond_tsi.level == tier::block);
	CHECK(beyond_tsi.hard);
	CHECK(beyond_tsi.passed.value == doctest::Approx(153.0));
	// within P0 nothing, between P0 and P1 above the recommended value
	CHECK(norms::assess(parameter::cant_deficiency, 70.0, limits, hard).level == tier::none);
	CHECK(norms::assess(parameter::cant_deficiency, 80.0, limits, hard).level == tier::none);
	norms::query plain;
	plain.speed = 100.0;
	CHECK(norms::assess(parameter::cant_deficiency, 120.0, norms::limits(profile_id::plk, parameter::cant_deficiency, plain), hard).level == tier::prefer);
}

TEST_CASE("norms engine: the cant deficiency at the highest speed and the excess at the slow trains of an arc") {
	// R 600 m, transitions of 60 m, the cant 60 mm: at 100 km/h I = 196.7 - 60 = 136.7 mm, above P1 130, within P2 150
	line_builder builder;
	builder.straight(200, 100).transition(0, 600, 60, 100).arc(600, 300, 100).transition(600, 0, 60, 100).straight(200, 100);
	builder.cant(200, 0).cant(260, 60).cant(560, 60).cant(620, 0);
	auto const findings{evaluate(builder.done(), mixed_main())};
	auto const deficiency{of(findings, parameter::cant_deficiency)};
	REQUIRE_FALSE(deficiency.empty());
	auto const &arc{*std::ranges::find_if(deficiency, [](norms::finding const &Finding) { return Finding.element == 2; })};
	CHECK(arc.value == doctest::Approx(11.8 * 100 * 100 / 600.0 - 60.0));
	CHECK(arc.level == tier::warn);
	CHECK(arc.limit == doctest::Approx(130.0));
	CHECK(arc.from.place == "tab. 9.7");
	CHECK(of(findings, parameter::cant_excess).empty());

	// the cant of 150 mm on it and slow trains of 40 km/h: E = 150 - 31.5 = 118.5 mm, above P2 110 (q <= 5 Tg a year)
	line_builder steep;
	steep.straight(200, 100).transition(0, 600, 100, 100).arc(600, 300, 100).transition(600, 0, 100, 100).straight(200, 100);
	steep.cant(200, 0).cant(300, 150).cant(600, 150).cant(700, 0);
	auto slow{mixed_main()};
	slow.vmin = 40.0;
	auto const excess{of(evaluate(steep.done(), slow), parameter::cant_excess)};
	REQUIRE_FALSE(excess.empty());
	auto const &worst{*std::ranges::max_element(excess, {}, &norms::finding::value)};
	CHECK(worst.value == doctest::Approx(150.0 - 11.8 * 40 * 40 / 600.0));
	CHECK(worst.level == tier::block);
	CHECK(worst.limit == doctest::Approx(110.0));
	CHECK(worst.speed == doctest::Approx(40.0));
}

TEST_CASE("norms engine: the abrupt change of the cant deficiency between arcs turning alike and turning the other way") {
	// R 1000 m straight into R 500 m at 80 km/h with no transition: I 75.5 and 151 mm
	auto const i1{11.8 * 80 * 80 / 1000.0};
	auto const i2{11.8 * 80 * 80 / 500.0};
	line_builder alike;
	alike.straight(100, 80).arc(1000, 200, 80).arc(500, 200, 80).straight(100, 80);
	auto const same{of(evaluate(alike.done(), mixed_main(80)), parameter::deficiency_jump)};
	auto const between = [](std::vector<norms::finding> const &Findings, double const Chainage) {
		return std::ranges::find_if(Findings, [Chainage](norms::finding const &Finding) { return std::abs(Finding.chainage - Chainage) < 1e-6; });
	};
	auto const turning{between(same, 300.0)};
	REQUIRE(turning != same.end());
	CHECK(turning->value == doctest::Approx(i2 - i1));
	CHECK(turning->level == tier::block);

	line_builder reverse;
	reverse.straight(100, 80).arc(1000, 200, 80);
	reverse.arc(-500, 200, 80).straight(100, 80);
	auto const other{of(evaluate(reverse.done(), mixed_main(80)), parameter::deficiency_jump)};
	auto const reversing{between(other, 300.0)};
	REQUIRE(reversing != other.end());
	CHECK(reversing->value == doctest::Approx(i1 + i2));
	// above the 125 mm of the TSI too: blocked by PLK, the stricter
	CHECK(reversing->level == tier::block);
	CHECK(reversing->limit == doctest::Approx(50.0));
}

TEST_CASE("norms engine: the abrupt changes over a straight shorter than tab. 9.16 asks for are taken together, the other way round or longer one by one") {
	// reverse curves of R 400 m at 60 km/h, I 106.2 mm each, with no transitions; the intermediate length at 60 km/h is 20 m
	auto const each{11.8 * 60 * 60 / 400.0};
	auto const jumps = [](double const Between, double const Second) {
		line_builder builder;
		builder.straight(100, 60).arc(400, 80, 60).straight(Between, 60).arc(Second, 80, 60).straight(100, 60);
		return of(evaluate(builder.done(), mixed_main(60)), parameter::deficiency_jump);
	};
	auto const close{jumps(10.0, -400.0)};
	auto const summed{std::ranges::find_if(close, [](norms::finding const &Finding) { return std::abs(Finding.chainage - 180.0) < 1e-6; })};
	REQUIRE(summed != close.end());
	CHECK(summed->value == doctest::Approx(2.0 * each));
	CHECK(summed->length == doctest::Approx(10.0));
	CHECK(summed->level == tier::block);
	// 30 m apart: each by itself
	auto const apart{jumps(30.0, -400.0)};
	for (auto const &finding : apart) {
		CAPTURE(finding.chainage);
		CHECK(finding.value == doctest::Approx(each));
		CHECK(finding.length == 0.0);
	}
	CHECK(apart.size() == 4);
	// turning alike, the deficiency falls and rises again: no monotonic change, each by itself
	for (auto const &finding : jumps(10.0, 400.0)) {
		CHECK(finding.value == doctest::Approx(each));
	}
}

TEST_CASE("norms engine: the gradient and the rate of change of the cant along a ramp, the rate of change of the deficiency along a transition") {
	// 100 mm over 50 m at 120 km/h: 2 mm/m, 66.7 mm/s
	line_builder ramp;
	ramp.straight(300, 120);
	ramp.cant(0, 0).cant(100, 0).cant(150, 100).cant(300, 100);
	auto const ramped{evaluate(ramp.done(), mixed_main(120))};
	auto const gradient{of(ramped, parameter::cant_gradient)};
	REQUIRE(gradient.size() == 1);
	CHECK(gradient.front().value == doctest::Approx(2.0));
	CHECK(gradient.front().level == tier::prefer);
	auto const rate{of(ramped, parameter::cant_rate)};
	REQUIRE(rate.size() == 1);
	CHECK(rate.front().value == doctest::Approx(2.0 * 120 / 3.6));
	CHECK(rate.front().level == tier::block);
	CHECK(rate.front().from.place == "tab. 9.13");
	// a step of the cant: the twist of 15 mm on the base of 5 m, 3 mm/m, beyond the 2.5 mm/m of every profile
	line_builder step;
	step.straight(300, 80);
	step.cant(0, 0).cant(150, 0).cant(150, 15).cant(300, 15);
	auto const stepped{of(evaluate(step.done(), mixed_main(80), profile_id::tsi), parameter::cant_gradient)};
	REQUIRE(stepped.size() == 1);
	CHECK(stepped.front().value == doctest::Approx(3.0));
	CHECK(stepped.front().level == tier::block);
	CHECK(stepped.front().hard);
	// a transition of 40 m to R 500 m with no cant at 80 km/h: 151 mm over 40 m, 83.9 mm/s
	line_builder curve;
	curve.straight(100, 80).transition(0, 500, 40, 80).arc(500, 100, 80).transition(500, 0, 40, 80).straight(100, 80);
	auto const change{of(evaluate(curve.done(), mixed_main(80)), parameter::deficiency_rate)};
	REQUIRE(change.size() == 2);
	CHECK(change.front().value == doctest::Approx(11.8 * 80 * 80 / 500.0 * 80 / (3.6 * 40.0)));
	CHECK(change.front().level == tier::block);
}

TEST_CASE("norms engine: the lengths of the elements, the transition curves and the radius") {
	line_builder builder;
	builder.straight(100, 100).arc(400, 20, 100).straight(28, 100).transition(0, 400, 20, 100).arc(400, 100, 100).transition(400, 0, 60, 100).straight(100, 100);
	builder.cant(0, 0).cant(148, 0).cant(168, 30).cant(268, 30).cant(328, 0);
	auto const findings{evaluate(builder.done(), mixed_main(100))};
	auto const lengths{of(findings, parameter::element_length)};
	// the arc of 20 m below the 25 m of P2, the straight of 28 m below the 35 m of P1
	REQUIRE(lengths.size() == 2);
	CHECK(lengths[0].level == tier::block);
	CHECK(lengths[0].limit == doctest::Approx(25.0));
	CHECK(lengths[1].level == tier::warn);
	CHECK(lengths[1].limit == doctest::Approx(35.0));
	auto const transitions{of(findings, parameter::transition_length)};
	REQUIRE(transitions.size() == 1);
	CHECK(transitions.front().value == doctest::Approx(20.0));
	CHECK(transitions.front().level == tier::warn);
	auto const radii{of(findings, parameter::radius)};
	REQUIRE_FALSE(radii.empty());
	CHECK(radii.front().value == doctest::Approx(400.0));
	CHECK(radii.front().level == tier::prefer);
}

TEST_CASE("norms engine: the vertical curves, a break of the grade and the grade") {
	line_builder builder;
	builder.straight(1000, 100);
	auto line{builder.done()};
	// level, a break to 8 per mille rising, a vertical curve of 40 m back to level (R 5000 m), then level
	line.grades = {{0.0, 300.0, 0.0, 0.0}, {300.0, 200.0, 0.008, 0.008}, {500.0, 40.0, 0.008, 0.0}, {540.0, 460.0, 0.0, 0.0}};
	auto const findings{evaluate(line, mixed_main(100))};
	auto const vertical{of(findings, parameter::vertical_radius)};
	REQUIRE(vertical.size() == 2);
	CHECK(vertical[0].value == 0.0);
	CHECK(vertical[0].level == tier::block);
	CHECK(vertical[0].limit == doctest::Approx(2500.0));
	CHECK(vertical[1].value == doctest::Approx(5000.0));
	CHECK(vertical[1].level == tier::prefer);
	auto const grades{of(findings, parameter::grade)};
	REQUIRE(grades.size() == 1);
	CHECK(grades.front().value == doctest::Approx(8.0));
	CHECK(grades.front().level == tier::prefer);
}

TEST_CASE("norms engine: an element on the boundary of two areas gets the stricter profile, an arc across it a warning") {
	// TSI west of x = 150, PLK elsewhere: a ramp of 2.2 mm/m is nothing to the TSI and a warning to PLK
	norms::area_map areas;
	areas.scenery = profile_id::plk;
	areas.areas.push_back({"west", profile_id::tsi, {{-1000.0, -1000.0}, {150.0, -1000.0}, {150.0, 1000.0}, {-1000.0, 1000.0}}});
	auto const ramp_at = [&areas](double const From) {
		line_builder builder;
		builder.straight(100, 100).straight(100, 100).straight(100, 100);
		builder.cant(0, 0).cant(From, 0).cant(From + 50, 110).cant(300, 110);
		return of(norms::evaluate(builder.done(), mixed_main(100), areas), parameter::cant_gradient);
	};
	// on the element from 0 to 100, all of it in the TSI area
	CHECK(ramp_at(20.0).empty());
	// on the element from 100 to 200, across the boundary: PLK
	auto const across{ramp_at(120.0)};
	REQUIRE(across.size() == 1);
	CHECK(across.front().profile == profile_id::plk);
	CHECK(across.front().level == tier::warn);
	// on the element from 200 to 300, in PLK
	REQUIRE(ramp_at(220.0).size() == 1);

	line_builder curve;
	curve.straight(100, 100).arc(2000, 100, 100).straight(100, 100);
	auto const boundary{of(norms::evaluate(curve.done(), mixed_main(100), areas), parameter::profile_boundary)};
	REQUIRE(boundary.size() == 1);
	CHECK(boundary.front().level == tier::warn);
	CHECK(boundary.front().element == 1);
	// a line of its own wins over the area
	areas.lines.emplace_back("main", profile_id::en13803);
	CHECK(areas.at({0.0, 0.0}, "main") == profile_id::en13803);
	CHECK(areas.at({0.0, 0.0}, "other") == profile_id::tsi);
	CHECK(areas.at({500.0, 0.0}, "other") == profile_id::plk);
}

TEST_CASE("norms engine: a line read from its samples: straights, transitions and arcs, the cant points and the grade line") {
	// samples a metre apart: 100 m straight, a transition of 50 m to R 500 m to the right, an arc of 100 m, then at a joint with no
	// transition a straight of 100 m; the cant 0 - 50 mm along the transition and back to 0 along the first 50 m of the straight; level,
	// and the straight rising 5 per mille from a break at its joint
	std::vector<norms::sample> samples;
	auto const add = [&samples](double const Chainage, double const Curvature, double const Cant, double const Grade) {
		norms::sample sample;
		sample.chainage = Chainage;
		sample.at = {Chainage, 0.0};
		sample.curvature = Curvature;
		sample.cant = Cant;
		sample.grade = Grade;
		sample.speed = 100.0;
		samples.push_back(sample);
	};
	for (int i = 0; i <= 100; ++i) {
		add(i, 0.0, 0.0, 0.0);
	}
	for (int i = 0; i <= 50; ++i) {
		add(100.0 + i, -i / 50.0 / 500.0, -i, 0.0);
	}
	for (int i = 0; i <= 100; ++i) {
		add(150.0 + i, -1.0 / 500.0, -50.0, 0.0);
	}
	for (int i = 0; i <= 100; ++i) {
		add(250.0 + i, 0.0, -50.0 + (i < 50 ? i : 50), 0.005);
	}
	auto const line{norms::describe(samples, "test")};
	REQUIRE(line.elements.size() == 4);
	CHECK(line.elements[0].kind == element_kind::straight);
	CHECK(line.elements[0].length == doctest::Approx(100.0));
	CHECK(line.elements[1].kind == element_kind::transition);
	CHECK(line.elements[1].curvature_end == doctest::Approx(-1.0 / 500.0));
	CHECK(line.elements[2].kind == element_kind::arc);
	CHECK(line.elements[2].length == doctest::Approx(100.0));
	CHECK(line.elements[2].curvature_start == doctest::Approx(-1.0 / 500.0));
	CHECK(line.elements[3].kind == element_kind::straight);
	CHECK(line.elements[0].open_start);
	CHECK(line.elements[3].open_end);
	// the cant: 0 to 100 m, -50 at 150 m, held to 250 m, back to 0 at 300 m
	REQUIRE(line.cant.size() >= 4);
	CHECK(line.cant.front().cant == doctest::Approx(0.0));
	auto const at = [&line](double const Chainage) { return std::ranges::find_if(line.cant, [Chainage](norms::cant_point const &Point) { return std::abs(Point.chainage - Chainage) < 1e-6; }); };
	REQUIRE(at(150.0) != line.cant.end());
	CHECK(at(150.0)->cant == doctest::Approx(-50.0));
	REQUIRE(at(300.0) != line.cant.end());
	CHECK(at(300.0)->cant == doctest::Approx(0.0));
	// the grade: level to 250 m, a break, 5 per mille on
	REQUIRE(line.grades.size() == 2);
	CHECK(line.grades[0].grade_start == 0.0);
	CHECK(line.grades[1].start == doctest::Approx(250.0));
	CHECK(line.grades[1].grade_start == doctest::Approx(0.005));
}
