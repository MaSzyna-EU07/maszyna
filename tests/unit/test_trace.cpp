#include "stdafx.h"
#include "doctest.h"
#include "editor/editorTrace.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <cmath>
#include <random>
#include <string>
#include <vector>

// The line traced on the aerial photo: lines of known straights and curves sampled as the clicks along the middle of the track would be,
// spread across by the noise of the clicks (fixed seeds), traced click after click; what it makes of them
namespace {

double constexpr pi{3.14159265358979323846};

glm::dvec2 heading(double const Degrees) { return {std::cos(Degrees * pi / 180.0), std::sin(Degrees * pi / 180.0)}; }

// a line from the origin along x: legs of straights between the vertices, each turning by its deflection (degrees, left positive)
struct leg {
	double length{0.0};
	double deflection{0.0};
	alignment::vertex curve;
};
alignment::design line_of(std::vector<leg> const &Legs, double const Last, double const Speed) {
	alignment::design design;
	design.speed = Speed;
	glm::dvec2 point{0.0};
	double direction{0.0};
	for (auto const &each : Legs) {
		point += heading(direction) * each.length;
		auto vertex{each.curve};
		vertex.position = point;
		design.vertices.push_back(vertex);
		direction += each.deflection;
	}
	auto const end{point + heading(direction) * Last};
	design.start = {0.0, 0.0, 0.0};
	design.end = {end.x, 0.0, end.y};
	design.start_direction = heading(0.0);
	design.end_direction = heading(direction);
	if (design.vertices.size() >= 2) {
		design.vertices.front().offset = Legs.front().length;
		design.vertices.back().offset = Last;
	}
	return design;
}

alignment::vertex arc(double const Radius, double const Transition, double const Cant = 0.0) {
	alignment::vertex vertex;
	vertex.radius = Radius;
	vertex.transition_in = vertex.transition_out = Transition;
	vertex.cant = Cant;
	return vertex;
}

alignment::vertex compound(std::vector<double> const &Radii, std::vector<double> const &Shares, double const Transition, double const Between) {
	alignment::vertex vertex;
	vertex.radius = Radii.front();
	vertex.compound = true;
	vertex.share = Shares.front();
	for (std::size_t i = 1; i < Radii.size(); ++i)
		vertex.arcs.push_back({Radii[i], Between, Shares[i]});
	vertex.transition_in = vertex.transition_out = Transition;
	return vertex;
}

// the transitions and the cant of the norms at the speed the sharpest arc allows, as a line laid to them has: a compound curve's ends at
// their radii, the cants in proportion to the curvature, the transitions between its arcs of the norms
alignment::vertex normed(alignment::vertex Vertex, double const Speed) {
	alignment::limits const norms;
	auto sharpest{Vertex.radius};
	for (auto const &each : Vertex.arcs)
		sharpest = std::min(sharpest, each.radius);
	auto const allowed{std::min(Speed, alignment::speed_limit(sharpest, norms))};
	auto const cant{alignment::recommend(allowed, sharpest, norms).cant};
	auto const last{Vertex.arcs.empty() ? Vertex.radius : Vertex.arcs.back().radius};
	Vertex.cant = cant;
	Vertex.transition_in = alignment::transition_length(allowed, Vertex.radius, cant * sharpest / Vertex.radius, norms);
	Vertex.transition_out = alignment::transition_length(allowed, last, cant * sharpest / last, norms);
	if (Vertex.compound)
		alignment::recommend_between(Vertex, allowed, norms);
	return Vertex;
}

// the clicks along the line: every Straight metres on its straights, every Curve metres along its curves, from From to To, shifted across
// by the noise of the seed
struct clicks {
	std::vector<glm::dvec2> points;
	std::vector<double> sigmas;
};
clicks sampled(alignment::result const &Line, double const Straight, double const Curve, double const Sigma, unsigned const Seed, double const From = 0.0, double To = -1.0) {
	if (To < 0.0)
		To = Line.length;
	std::mt19937 random{Seed}; // NOSONAR: seeded on purpose, the test is repeated the same
	std::normal_distribution<double> noise{0.0, Sigma};
	auto const curved = [&](double const Chainage) {
		for (auto const &e : Line.elements)
			if (e.kind != alignment::element_kind::straight && Chainage >= e.chainage - 1e-6 && Chainage <= e.chainage + e.length + 1e-6)
				return true;
		return false;
	};
	clicks result;
	for (auto s{From}; s <= To + 1e-6;) {
		auto const station{alignment::evaluate(Line, s)};
		glm::dvec2 const across{-station.direction.z, station.direction.x};
		result.points.push_back(glm::dvec2{station.position.x, station.position.z} + glm::normalize(across) * noise(random));
		result.sigmas.push_back(Sigma);
		s += curved(s) || curved(s + Straight * 0.5) ? Curve : Straight;
	}
	return result;
}

trace::settings settings_at(double const Speed) {
	trace::settings settings;
	settings.speed = Speed;
	return settings;
}

double relative(double const A, double const B) { return std::abs(A - B) / std::abs(B); }

// the radii the line was laid with: those given, where the curves had room enough
std::vector<double> radii_of(alignment::result const &Line, std::size_t const Curve) {
	auto const &curve{Line.curves.at(Curve)};
	std::vector<double> radii{curve.radius};
	radii.insert(radii.end(), curve.radii.begin(), curve.radii.end());
	if (curve.radii.size() > 1)
		radii = curve.radii;
	return radii;
}

alignment::result laid(alignment::design const &Design) {
	auto const line{alignment::compute(Design)};
	REQUIRE(line.valid);
	REQUIRE(line.radius_warnings.empty());
	return line;
}

// the radii of the curves found, sorted per curve as they are
std::string described(trace::result const &Line) {
	std::string text{std::to_string(Line.curves.size()) + " curves:"};
	for (auto const &curve : Line.curves)
		text += " R " + trace::radii_text(curve) + (curve.restricted ? " v " + std::to_string(static_cast<int>(curve.speed)) : "");
	text += ", worst " + std::to_string(Line.worst) + (Line.error.empty() ? "" : ", " + Line.error);
	return text;
}

} // namespace

TEST_CASE("trace: points along a straight make a straight") {
	auto const truth{alignment::compute(line_of({}, 1000.0, 100.0))};
	REQUIRE(truth.valid);
	for (auto const sigma : {0.1, 0.2})
		for (unsigned seed = 1; seed <= 5; ++seed) {
			auto const points{sampled(truth, 150.0, 150.0, sigma, seed)};
			auto const line{trace::traced(points.points, points.sigmas, settings_at(100.0))};
			INFO(described(line));
			REQUIRE(line.valid);
			CHECK(line.curves.empty());
			CHECK(line.worst <= 3.0 * sigma);
		}
}

TEST_CASE("trace: a curve of one radius between straights") {
	auto const truth{alignment::compute(line_of({{500.0, 40.0, normed(arc(800.0, 0.0), 100.0)}}, 500.0, 100.0))};
	REQUIRE(truth.valid);
	for (auto const sigma : {0.1, 0.2})
		for (unsigned seed = 1; seed <= 3; ++seed) {
			auto const points{sampled(truth, 120.0, 40.0, sigma, seed)};
			auto const line{trace::traced(points.points, points.sigmas, settings_at(100.0))};
			INFO(described(line));
			REQUIRE(line.valid);
			REQUIRE(line.curves.size() == 1);
			CHECK(line.curves.front().radii.size() == 1);
			CHECK(relative(line.curves.front().radii.front(), 800.0) < 0.02);
			CHECK_FALSE(line.curves.front().restricted);
		}
}

TEST_CASE("trace: gentle curves of R 8 and 20 km are curves, nearly straight") {
	for (auto const &[radius, length] : {std::pair{8000.0, 300.0}, std::pair{8000.0, 1000.0}, std::pair{20000.0, 600.0}, std::pair{20000.0, 1000.0}}) {
		auto const deflection{length / radius * 180.0 / pi};
		auto const truth{alignment::compute(line_of({{600.0, deflection, arc(radius, 0.0)}}, 600.0, 100.0))};
		REQUIRE(truth.valid);
		for (auto const sigma : {0.1, 0.2})
			for (unsigned seed = 1; seed <= 3; ++seed) {
				auto const points{sampled(truth, 100.0, 100.0, sigma, seed)};
				auto const line{trace::traced(points.points, points.sigmas, settings_at(100.0))};
				INFO("R " << radius << " over " << length << " m, sigma " << sigma << ": " << described(line));
				REQUIRE(line.valid);
				REQUIRE(line.curves.size() == 1);
				CHECK(line.curves.front().nearly_straight);
				CHECK(relative(line.curves.front().sharpest(), radius) < 0.25);
			}
	}
}

TEST_CASE("trace: compound curves of two and three radii") {
	struct case_of {
		std::vector<double> radii;
		std::vector<double> shares;
	};
	for (auto const &shape : {case_of{{1200.0, 700.0}, {0.5, 0.5}}, case_of{{600.0, 400.0}, {0.5, 0.5}}, case_of{{1200.0, 700.0, 1000.0}, {0.35, 0.35, 0.3}}}) {
		auto const truth{laid(line_of({{1200.0, 60.0, normed(compound(shape.radii, shape.shares, 0.0, 0.0), 100.0)}}, 1200.0, 100.0))};
		auto const radii{radii_of(truth, 0)};
		REQUIRE(radii.size() == shape.radii.size());
		for (auto const sigma : {0.1, 0.2})
			for (unsigned seed = 1; seed <= 2; ++seed) {
				// clicked closer along a curve of radii telling apart, as the hint asks: the radius of an arc of 250 m read off points 25 m
				// apart within 0.2 m is known within 1.6 % only
				auto const points{sampled(truth, 120.0, 12.0, sigma, seed)};
				auto const line{trace::traced(points.points, points.sigmas, settings_at(100.0))};
				INFO("sigma " << sigma << ": " << described(line));
				REQUIRE(line.valid);
				REQUIRE(line.curves.size() == 1);
				auto const &found{line.curves.front().radii};
				REQUIRE(found.size() == radii.size());
				for (std::size_t i = 0; i < found.size(); ++i)
					CHECK(relative(found[i], radii[i]) < 0.02);
			}
	}
}

TEST_CASE("trace: an S-curve, the other way after a short straight") {
	auto const truth{laid(line_of({{400.0, 30.0, normed(arc(600.0, 0.0), 100.0)}, {550.0, -35.0, normed(arc(800.0, 0.0), 100.0)}}, 400.0, 100.0))};
	for (auto const sigma : {0.1, 0.2})
		for (unsigned seed = 1; seed <= 3; ++seed) {
			auto const points{sampled(truth, 120.0, 40.0, sigma, seed)};
			auto const line{trace::traced(points.points, points.sigmas, settings_at(100.0))};
			INFO(described(line));
			REQUIRE(line.valid);
			REQUIRE(line.curves.size() == 2);
			CHECK(relative(line.curves[0].sharpest(), 600.0) < 0.02);
			CHECK(relative(line.curves[1].sharpest(), 800.0) < 0.02);
		}
}

TEST_CASE("trace: a line started inside a curve") {
	auto const truth{alignment::compute(line_of({{400.0, 50.0, normed(arc(700.0, 0.0), 100.0)}}, 400.0, 100.0))};
	REQUIRE(truth.valid);
	// from the middle of the curve on
	double middle{0.0};
	for (auto const &e : truth.elements)
		if (e.kind == alignment::element_kind::arc)
			middle = e.chainage + 0.5 * e.length;
	for (auto const sigma : {0.1, 0.2})
		for (unsigned seed = 1; seed <= 3; ++seed) {
			auto const points{sampled(truth, 120.0, 40.0, sigma, seed, middle)};
			auto const line{trace::traced(points.points, points.sigmas, settings_at(100.0))};
			INFO(described(line));
			REQUIRE(line.valid);
			REQUIRE(line.curves.size() == 1);
			CHECK(line.curves.front().inside);
			CHECK(relative(line.curves.front().sharpest(), 700.0) < 0.03);
			// the start heading as the curve has it, not as the first two points: their chord is 0.03 rad off the tangent
			auto const station{alignment::evaluate(truth, middle)};
			auto const truth_heading{std::atan2(station.direction.z, station.direction.x)};
			auto const start{line.design.start_direction};
			CHECK(std::abs(std::remainder(std::atan2(start.y, start.x) - truth_heading, 2.0 * pi)) < 0.008);
		}
}

TEST_CASE("trace: the curves frozen don't change with the clicks after them") {
	auto const truth{laid(line_of({{400.0, 35.0, normed(arc(700.0, 0.0), 100.0)}, {800.0, -30.0, normed(arc(900.0, 0.0), 100.0)}, {800.0, 25.0, normed(arc(1000.0, 0.0), 100.0)}}, 500.0, 100.0))};
	auto const points{sampled(truth, 120.0, 40.0, 0.15, 3)};
	trace::tracer line{settings_at(100.0)};
	std::vector<trace::result> after;
	for (std::size_t i = 0; i < points.points.size(); ++i) {
		line.add(points.points[i], points.sigmas[i]);
		after.push_back(line.now());
	}
	INFO(described(line.now()));
	REQUIRE(line.now().curves.size() == 3);
	CHECK(line.now().frozen >= 2);
	// each curve, once the one after it froze too, stays as it is to the bit
	for (std::size_t c = 0; c + 1 < 3; ++c) {
		std::size_t locked{after.size()};
		for (std::size_t i = 0; i < after.size(); ++i)
			if (after[i].frozen >= c + 2) {
				locked = i;
				break;
			}
		REQUIRE(locked < after.size());
		auto const &then{after[locked].curves[c].vertex};
		for (auto k{locked}; k < after.size(); ++k) {
			auto const &now{after[k].curves[c].vertex};
			CHECK(now.radius == then.radius);
			CHECK(now.transition_in == then.transition_in);
			CHECK(now.transition_out == then.transition_out);
			CHECK(now.position.x == then.position.x);
			CHECK(now.position.y == then.position.y);
		}
		// frozen and only sliding at its end before that: its radius kept from the click it froze at
		std::size_t froze{after.size()};
		for (std::size_t i = 0; i < after.size(); ++i)
			if (after[i].frozen >= c + 1) {
				froze = i;
				break;
			}
		for (auto k{froze}; k < after.size(); ++k)
			CHECK(after[k].curves[c].radii == after[froze].curves[c].radii);
	}
}

TEST_CASE("trace: a point clicked far off leaves the line as it was, told") {
	auto const truth{alignment::compute(line_of({{500.0, 40.0, normed(arc(800.0, 0.0), 100.0)}}, 500.0, 100.0))};
	auto const points{sampled(truth, 120.0, 40.0, 0.1, 1)};
	trace::tracer line{settings_at(100.0)};
	for (std::size_t i = 0; i < 6; ++i)
		line.add(points.points[i], points.sigmas[i]);
	auto const before{line.now()};
	// back across the straight, far off it
	line.add(points.points[2] + glm::dvec2{0.0, 40.0}, 0.1);
	auto const now{line.now()};
	REQUIRE(now.misfit.size() == 7);
	CHECK(now.misfit.back() != 0);
	REQUIRE(now.design.vertices.size() == before.design.vertices.size());
	CHECK(now.design.start.x == before.design.start.x);
	CHECK(now.design.end.x == before.design.end.x);
	CHECK(now.design.end.z == before.design.end.z);
	line.pop();
	CHECK(line.now().misfit == before.misfit);
}

TEST_CASE("trace: the same points make the same line") {
	auto const truth{laid(line_of({{400.0, 35.0, normed(arc(700.0, 0.0), 100.0)}, {900.0, -30.0, normed(compound({900.0, 600.0}, {0.5, 0.5}, 0.0, 0.0), 100.0)}}, 600.0, 100.0))};
	auto const points{sampled(truth, 120.0, 30.0, 0.2, 9)};
	auto const a{trace::traced(points.points, points.sigmas, settings_at(100.0))};
	auto const b{trace::traced(points.points, points.sigmas, settings_at(100.0))};
	REQUIRE(a.design.vertices.size() == b.design.vertices.size());
	for (std::size_t k = 0; k < a.design.vertices.size(); ++k) {
		CHECK(a.design.vertices[k].radius == b.design.vertices[k].radius);
		CHECK(a.design.vertices[k].position == b.design.vertices[k].position);
	}
	CHECK(a.offsets == b.offsets);
}

TEST_CASE("trace: a curve sharper than the norms allow at the speed of the line is slowed down, not refused") {
	auto const truth{alignment::compute(line_of({{400.0, 50.0, arc(300.0, 50.0, 100.0)}}, 400.0, 70.0))};
	REQUIRE(truth.valid);
	auto const points{sampled(truth, 120.0, 25.0, 0.1, 2)};
	auto settings{settings_at(120.0)};
	settings.radius_least = 180.0;
	auto const line{trace::traced(points.points, points.sigmas, settings)};
	INFO(described(line));
	REQUIRE(line.valid);
	REQUIRE(line.curves.size() == 1);
	auto const &curve{line.curves.front()};
	CHECK(curve.restricted);
	CHECK(curve.speed < 120.0);
	CHECK(curve.speed >= 60.0);
	CHECK(std::fmod(curve.speed, 10.0) == doctest::Approx(0.0));
	CHECK_FALSE(line.refused);
	// the norms met at the speed it's laid for
	CHECK(line.design.vertices.front().speed == curve.speed);
	for (auto const &report : line.laid.curves)
		CHECK(alignment::meets_norms(report, line.design));
	// below the least radius of the track: refused
	settings.radius_least = 350.0;
	auto const refused{trace::traced(points.points, points.sigmas, settings)};
	CHECK(refused.refused);
}

TEST_CASE("trace: a click of the worst case, a compound curve of three radii of 60 points, takes 30 ms at most") {
	auto const truth{laid(line_of({{1500.0, 70.0, normed(compound({1200.0, 700.0, 1000.0}, {0.35, 0.35, 0.3}, 0.0, 0.0), 100.0)}}, 1500.0, 100.0))};
	double from{truth.length};
	double to{0.0};
	for (auto const &e : truth.elements)
		if (e.kind != alignment::element_kind::straight) {
			from = std::min(from, e.chainage);
			to = std::max(to, e.chainage + e.length);
		}
	auto const points{sampled(truth, (to - from) / 55.0, (to - from) / 55.0, 0.2, 4, from - 50.0, to + 80.0)};
	REQUIRE(points.points.size() >= 60);
	// over and over, each click its quickest: the same work each time, the machine busy with something else at times (the builds of
	// the other worktrees beside it); done once every click came in within the time, ten times at most
	std::vector<double> quickest(points.points.size(), std::numeric_limits<double>::max());
	std::vector<std::size_t> evaluations(points.points.size(), 0);
	trace::result last;
	auto slowest{std::numeric_limits<double>::max()};
	for (int run = 0; run < 10 && slowest > 30.0; ++run) {
		trace::tracer line{settings_at(100.0)};
		for (std::size_t i = 0; i < points.points.size(); ++i) {
			line.add(points.points[i], points.sigmas[i]);
			quickest[i] = std::min(quickest[i], line.now().milliseconds);
			if (run > 0)
				CHECK(line.now().evaluations == evaluations[i]);
			evaluations[i] = line.now().evaluations;
		}
		last = line.now();
		slowest = *std::max_element(quickest.begin(), quickest.end());
	}
	MESSAGE("slowest click " << slowest << " ms, most evaluations " << *std::max_element(evaluations.begin(), evaluations.end()));
	INFO(described(last));
	CHECK(last.curves.size() == 1);
	CHECK(slowest <= 30.0);
}

TEST_CASE("trace: a run beside a neighbour at a constant spacing is parallel") {
	std::vector<glm::dvec2> neighbour;
	for (int i = 0; i <= 20; ++i)
		neighbour.push_back({i * 50.0, 0.0});
	std::mt19937 random{5}; // NOSONAR: seeded on purpose
	std::normal_distribution<double> noise{0.0, 0.1};
	std::vector<glm::dvec2> points;
	std::vector<double> sigmas;
	for (int i = 1; i < 10; ++i) {
		points.push_back({i * 100.0, 4.75 + noise(random)});
		sigmas.push_back(0.1);
	}
	auto const beside{trace::spacing_to(points, sigmas, neighbour)};
	CHECK(beside.parallel);
	CHECK(beside.mean == doctest::Approx(4.75).epsilon(0.02));
	// one drifting away isn't
	for (std::size_t i = 0; i < points.size(); ++i)
		points[i].y += 0.2 * i;
	CHECK_FALSE(trace::spacing_to(points, sigmas, neighbour).parallel);
}

TEST_CASE("trace: from a free end, straight or curved, and into one") {
	auto const truth{laid(line_of({{500.0, 40.0, normed(arc(800.0, 0.0), 100.0)}}, 600.0, 100.0))};
	auto const points{sampled(truth, 120.0, 40.0, 0.1, 3)};
	// held at the start of the line, the first point there not clicked again
	trace::hold start;
	start.position = {0.0, 0.0};
	start.heading = {1.0, 0.0};
	std::vector<glm::dvec2> after(points.points.begin() + 1, points.points.end());
	std::vector<double> sigmas(points.sigmas.begin() + 1, points.sigmas.end());
	auto const held{trace::traced(after, sigmas, settings_at(100.0), start)};
	INFO(described(held));
	REQUIRE(held.valid);
	REQUIRE(held.curves.size() == 1);
	CHECK(relative(held.curves.front().sharpest(), 800.0) < 0.02);
	CHECK(held.design.start.x == 0.0);
	CHECK(held.design.start.z == 0.0);
	CHECK(held.design.start_direction.x == 1.0);
	// into a free end on the straight after the curve, 100 m past the last point
	trace::tracer line{settings_at(100.0), start};
	for (std::size_t i = 0; i + 2 < after.size(); ++i)
		line.add(after[i], sigmas[i]);
	auto const station{alignment::evaluate(truth, truth.length - 50.0)};
	trace::hold end;
	end.position = {station.position.x, station.position.z};
	end.heading = glm::normalize(glm::dvec2{station.direction.x, station.direction.z});
	auto const into{line.ended(end)};
	INFO(described(into));
	REQUIRE(into.valid);
	REQUIRE(into.curves.size() == 1);
	CHECK(relative(into.curves.front().sharpest(), 800.0) < 0.03);
	CHECK(glm::distance(glm::dvec2{into.design.end.x, into.design.end.z}, end.position) < 1e-6);
	auto const finish{alignment::evaluate(into.laid, into.laid.length)};
	CHECK(glm::distance(glm::dvec2{finish.position.x, finish.position.z}, end.position) < 0.01);
	CHECK(std::abs(glm::dot(glm::normalize(glm::dvec2{finish.direction.x, finish.direction.z}), end.heading)) > 1.0 - 1e-6);
	// from a free end within the curve: the curve goes on from it at its radius
	double middle{0.0};
	for (auto const &e : truth.elements)
		if (e.kind == alignment::element_kind::arc)
			middle = e.chainage + 0.5 * e.length;
	auto const on{alignment::evaluate(truth, middle)};
	trace::hold curved;
	curved.position = {on.position.x, on.position.z};
	curved.heading = glm::normalize(glm::dvec2{on.direction.x, on.direction.z});
	curved.curvature = 1.0 / truth.curves.front().radius;
	curved.cant = truth.curves.front().cant;
	auto const rest{sampled(truth, 120.0, 40.0, 0.1, 4, middle + 40.0)};
	auto const going{trace::traced(rest.points, rest.sigmas, settings_at(100.0), curved)};
	INFO(described(going));
	REQUIRE(going.valid);
	REQUIRE(going.curves.size() == 1);
	CHECK(going.design.start_curved);
	CHECK(relative(going.curves.front().radii.front(), 800.0) < 0.02);
}
