// Stations traced on the aerial photo (editor/editorTraceStation.cpp): the turnout of the catalogue fitted to the points clicked along its
// diverging track, sampled from the turnouts themselves with the noise of the clicks (fixed seeds); the spacing of a track alongside
// another one measured and rounded; a turnout moved along its track to keep the straight the norms ask between turnouts

#include "stdafx.h"
#include "doctest.h"
#include "editor/editorTraceStation.hpp"
#include "editor/editorSwitchDesign.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>
#include <string>
#include <vector>

namespace {

double constexpr pi{3.14159265358979323846};

glm::dvec2 heading(double const Degrees) { return {std::cos(Degrees * pi / 180.0), std::sin(Degrees * pi / 180.0)}; }

glm::dvec2 left(glm::dvec2 const &V) { return {-V.y, V.x}; }

// the plain turnouts of the catalogue of the rail, as the fit takes them: their names, their shapes
struct catalogue_shapes {
	std::vector<std::string> names;
	std::vector<trace_station::turnout_shape> shapes;
	int index(std::string const &Name) const {
		auto const found{std::find(names.begin(), names.end(), Name)};
		return found != names.end() ? static_cast<int>(found - names.begin()) : -1;
	}
};

catalogue_shapes plain_turnouts(std::string const &Rail) {
	catalogue_shapes result;
	for (auto const &design : switch_design::catalogue_entries()) {
		if (design.type != "Rz" || design.rail != Rail || design.movable)
			continue;
		std::string error;
		auto const shape{switch_design::build(design, error)};
		if (shape.paths.size() < 2)
			continue;
		auto const straight_frog{false == design.parts.empty() && design.parts.back().radius1 <= 0.0 && design.parts.back().radius2 <= 0.0};
		result.names.push_back(design.name);
		result.shapes.push_back(trace_station::shape_of(shape.paths[1], shape.length, straight_frog ? 0.0 : 1.0 / shape.radius));
	}
	return result;
}

// the point of the diverging track Along metres from the start of the switch along the main track, or of the straight beyond its end
glm::dvec2 diverging_at(trace_station::turnout_shape const &Shape, double const Along) {
	auto const &line{Shape.diverging};
	if (Along >= line.back().x) {
		auto const last{glm::normalize(line.back() - line[line.size() - 2])};
		return line.back() + last * ((Along - line.back().x) / last.x);
	}
	for (std::size_t i = 0; i + 1 < line.size(); ++i)
		if (Along <= line[i + 1].x) {
			auto const t{(Along - line[i].x) / std::max(1e-9, line[i + 1].x - line[i].x)};
			return line[i] + (line[i + 1] - line[i]) * t;
		}
	return line.back();
}

// a turnout laid on the main track at Start, the way and to the side given, as the clicks along it show it: the first one on the main track
// near its start (Before metres before it), then the diverging track every Spacing metres from 6 m on and the straight beyond it as far as
// Beyond metres past its end, each moved across by the noise
struct clicks {
	std::vector<glm::dvec2> points;
	std::vector<double> sigmas;
};
clicks clicked(trace_station::straight const &Main, trace_station::turnout_shape const &Shape, double const Start, int const Direction, int const Side, double const Before, double const Spacing, double const Beyond,
               double const Sigma, unsigned const Seed) {
	std::mt19937 random{Seed};
	std::normal_distribution<double> noise{0.0, Sigma};
	auto const x{Main.direction * static_cast<double>(Direction)};
	auto const y{left(x) * static_cast<double>(Side)};
	auto const origin{Main.point(Start)};
	clicks result;
	auto const add = [&](glm::dvec2 const &Point) {
		result.points.push_back(Point + glm::dvec2{noise(random), noise(random)});
		result.sigmas.push_back(Sigma);
	};
	add(origin - x * Before);
	for (double along = 6.0; along <= Shape.length + Beyond + 1e-9; along += Spacing) {
		auto const local{diverging_at(Shape, along)};
		add(origin + x * local.x + y * local.y);
	}
	return result;
}

}

TEST_CASE("trace_station: the shapes of the catalogue go along the main track, from its start to its end") {
	auto const turnouts{plain_turnouts("60E1")};
	REQUIRE(turnouts.shapes.size() >= 5);
	for (std::size_t i = 0; i < turnouts.shapes.size(); ++i) {
		CAPTURE(turnouts.names[i]);
		auto const &shape{turnouts.shapes[i]};
		REQUIRE(shape.diverging.size() >= 10);
		CHECK(glm::length(shape.diverging.front()) < 1e-6);
		CHECK(shape.diverging.back().x == doctest::Approx(shape.length).epsilon(0.02));
		CHECK(shape.diverging.back().y > 0.5);
		for (std::size_t k = 1; k < shape.diverging.size(); ++k)
			CHECK(shape.diverging[k].x > shape.diverging[k - 1].x);
	}
}

TEST_CASE("trace_station: the turnout of the catalogue, its start, its way and its side fitted to the clicks along it") {
	auto const turnouts{plain_turnouts("60E1")};
	trace_station::straight main;
	main.origin = {120.0, -40.0};
	main.direction = heading(23.0);
	struct each {
		std::string name;
		int direction;
		int side;
		double spacing;
	};
	unsigned seed{11};
	for (auto const &[name, direction, side, spacing] : std::vector<each>{{"Rz 60E1-190-1:9", 1, 1, 6.0},
	                                                                       {"Rz 60E1-300-1:9", -1, 1, 7.0},
	                                                                       {"Rz 60E1-300-1:9", 1, -1, 7.0},
	                                                                       {"Rz 60E1-500-1:12", 1, -1, 8.0},
	                                                                       {"Rz 60E1-760-1:14", -1, -1, 9.0},
	                                                                       {"Rz 60E1-1200-1:18,5", 1, 1, 10.0}}) {
		for (auto const sigma : {0.05, 0.1}) {
			CAPTURE(name);
			CAPTURE(direction);
			CAPTURE(side);
			CAPTURE(sigma);
			auto const index{turnouts.index(name)};
			REQUIRE(index >= 0);
			auto const &shape{turnouts.shapes[index]};
			auto const start{200.0};
			auto const points{clicked(main, shape, start, direction, side, 1.5, spacing, 12.0, sigma, seed++)};
			// the usual switch of a station preferred, which the clicks have to overrule
			auto const started{std::chrono::steady_clock::now()};
			auto const fit{trace_station::fit_turnout(main, turnouts.shapes, points.points, points.sigmas, turnouts.index("Rz 60E1-300-1:9"))};
			auto const took{std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count()};
			REQUIRE(fit.valid);
			CHECK(turnouts.names[fit.shape] == name);
			CHECK(fit.direction == direction);
			CHECK(fit.side == side);
			CHECK_FALSE(fit.preferred);
			CHECK(std::abs(fit.start - start) <= 1.0);
			CHECK(fit.rms <= 2.5 * sigma);
			CHECK(fit.told >= 2);
			// its end where the turnout ends, heading the way its diverging track leaves
			auto const x{main.direction * static_cast<double>(direction)};
			auto const y{left(x) * static_cast<double>(side)};
			auto const end{main.point(start) + x * shape.diverging.back().x + y * shape.diverging.back().y};
			CHECK(glm::distance(fit.end, end) <= 1.0);
			CHECK(glm::dot(fit.heading, x) > 0.98);
			CHECK(fit.curvature == doctest::Approx(shape.end_curvature * side));
			// the points along the turnout used by it, the ones on the straight beyond it left to the line
			CHECK(fit.used >= 2);
			CHECK(fit.used < points.points.size());
			CHECK(took < 200.0);
		}
	}
}

TEST_CASE("trace_station: with no click off the main track yet the turnout preferred is taken, the way and the side of the next click") {
	auto const turnouts{plain_turnouts("60E1")};
	trace_station::straight main;
	main.direction = heading(-60.0);
	auto const preferred{turnouts.index("Rz 60E1-500-1:12")};
	auto const next{main.point(-14.0, -0.05)};
	auto const fit{trace_station::fit_turnout(main, turnouts.shapes, {main.point(-3.0), next}, {0.1, 0.1}, preferred)};
	REQUIRE(fit.valid);
	CHECK(fit.preferred);
	CHECK(fit.shape == preferred);
	CHECK(fit.direction == -1);
	// a little to the right of the track as it runs, to the left of the way the turnout runs
	CHECK(fit.side == 1);
	CHECK(fit.start == doctest::Approx(-3.0));
	// a single point makes none
	CHECK_FALSE(trace_station::fit_turnout(main, turnouts.shapes, {main.point(-3.0)}, {0.1}, preferred).valid);
}

TEST_CASE("trace_station: the turnout moved along its track keeps its way and its side, its end goes with it") {
	auto const turnouts{plain_turnouts("60E1")};
	trace_station::straight main;
	main.direction = heading(10.0);
	auto const &shape{turnouts.shapes[turnouts.index("Rz 60E1-300-1:9")]};
	auto const points{clicked(main, shape, 50.0, 1, 1, 0.0, 7.0, 12.0, 0.05, 5)};
	auto const fit{trace_station::fit_turnout(main, turnouts.shapes, points.points, points.sigmas, -1)};
	REQUIRE(fit.valid);
	auto const moved{trace_station::turnout_at(main, turnouts.shapes, fit, fit.start + 1.2, points.points)};
	CHECK(moved.shape == fit.shape);
	CHECK(moved.side == fit.side);
	CHECK(moved.direction == fit.direction);
	CHECK(glm::distance(moved.end, fit.end + main.direction * 1.2) < 1e-9);
	CHECK(moved.heading == fit.heading);
}

TEST_CASE("trace_station: the spacing rounded to the usual ones, else to 5 cm") {
	CHECK(trace_station::round_spacing(4.68) == doctest::Approx(4.75));
	CHECK(trace_station::round_spacing(-4.83) == doctest::Approx(-4.75));
	CHECK(trace_station::round_spacing(4.09) == doctest::Approx(4.0));
	CHECK(trace_station::round_spacing(4.52) == doctest::Approx(4.5));
	CHECK(trace_station::round_spacing(5.07) == doctest::Approx(5.0));
	CHECK(trace_station::round_spacing(6.12) == doctest::Approx(6.1));
	CHECK(trace_station::round_spacing(-3.71) == doctest::Approx(-3.7));
}

TEST_CASE("trace_station: a run of the clicks keeping one spacing from a straight track beside it, the clicks before and after it left out") {
	std::vector<glm::dvec2> const track{{0.0, 0.0}, {1000.0, 0.0}};
	std::mt19937 random{7};
	std::normal_distribution<double> noise{0.0, 0.08};
	std::vector<glm::dvec2> points;
	std::vector<double> sigmas;
	// off the track by the turnout and the curve after it, then alongside at 4.73 m, then away again
	for (auto const &[x, y] : std::vector<std::pair<double, double>>{{100.0, 0.0}, {130.0, 1.2}, {160.0, 3.6}}) {
		points.push_back({x, y});
		sigmas.push_back(0.08);
	}
	for (double x = 200.0; x <= 600.0; x += 80.0) {
		points.push_back({x + noise(random), 4.73 + noise(random)});
		sigmas.push_back(0.08);
	}
	points.push_back({650.0, 7.0});
	sigmas.push_back(0.08);
	auto const run{trace_station::find_parallel(points, sigmas, track)};
	REQUIRE(run.found);
	CHECK(run.first == 3);
	CHECK(run.last == 8);
	CHECK(run.measured == doctest::Approx(4.73).epsilon(0.02));
	CHECK(run.spacing == doctest::Approx(4.75));
	// to the right of a track running the other way
	std::vector<glm::dvec2> const back{{1000.0, 0.0}, {0.0, 0.0}};
	auto const other{trace_station::find_parallel(points, sigmas, back)};
	REQUIRE(other.found);
	CHECK(other.spacing == doctest::Approx(-4.75));
	// the run laid exactly alongside
	for (auto i{run.first}; i <= run.last; ++i)
		CHECK(trace_station::abreast_of(track, trace_station::alongside(track, points[i], run.spacing)).lateral == doctest::Approx(4.75).epsilon(1e-9));
	// too short a run, or one too loose for the noise, is none
	CHECK_FALSE(trace_station::find_parallel({{200.0, 4.7}, {210.0, 4.7}, {220.0, 4.7}}, {0.1, 0.1, 0.1}, track).found);
	CHECK_FALSE(trace_station::find_parallel({{200.0, 4.7}, {300.0, 5.4}, {400.0, 4.7}, {500.0, 5.4}}, {0.05, 0.05, 0.05, 0.05}, track).found);
	// on the track itself none either
	CHECK_FALSE(trace_station::find_parallel({{200.0, 0.1}, {300.0, -0.1}, {400.0, 0.05}}, {0.1, 0.1, 0.1}, track).found);
}

TEST_CASE("trace_station: a run alongside a curved track, its arcs concentric") {
	// an arc of R 1000 m turning left, sampled every metre
	double constexpr radius{1000.0};
	std::vector<glm::dvec2> track;
	for (int i = 0; i <= 600; ++i) {
		auto const angle{i / radius};
		track.push_back({radius * std::sin(angle), radius * (1.0 - std::cos(angle))});
	}
	std::mt19937 random{3};
	std::normal_distribution<double> noise{0.0, 0.05};
	std::vector<glm::dvec2> points;
	std::vector<double> sigmas;
	// on its outside, 5.02 m off
	for (double s = 50.0; s <= 550.0; s += 100.0) {
		auto const angle{s / radius};
		auto const at{(radius + 5.02 + noise(random))};
		points.push_back({at * std::sin(angle), radius - at * std::cos(angle)});
		sigmas.push_back(0.05);
	}
	auto const run{trace_station::find_parallel(points, sigmas, track)};
	REQUIRE(run.found);
	CHECK(run.first == 0);
	CHECK(run.last == points.size() - 1);
	CHECK(run.spacing == doctest::Approx(-5.0));
	// moved onto R 1005 m about the same centre
	for (auto const &point : points)
		CHECK(glm::distance(trace_station::alongside(track, point, run.spacing), glm::dvec2{0.0, radius}) == doctest::Approx(radius + 5.0).epsilon(1e-6));
}

TEST_CASE("trace_station: a turnout in a head moved to keep the straight the norms ask from the next one") {
	// a straight of plain track from 0 to 100 m between two turnouts, 6 m asked between them
	auto const nearer{trace_station::place_in_head(4.8, 33.2, 0.0, 100.0, true, true, 6.0)};
	REQUIRE(nearer.fits);
	CHECK(nearer.from == doctest::Approx(6.0));
	CHECK(nearer.moved == doctest::Approx(1.2));
	auto const farther{trace_station::place_in_head(63.0, 33.2, 0.0, 100.0, true, true, 6.0)};
	REQUIRE(farther.fits);
	CHECK(farther.from == doctest::Approx(60.8));
	CHECK(farther.moved == doctest::Approx(-2.2));
	// where the photo put it, the norms kept
	auto const kept{trace_station::place_in_head(30.0, 33.2, 0.0, 100.0, true, true, 6.0)};
	CHECK(kept.fits);
	CHECK(kept.moved == 0.0);
	// no turnout at the end of the straight: no straight asked there
	CHECK(trace_station::place_in_head(-3.0, 33.2, 0.0, 100.0, false, true, 6.0).from == doctest::Approx(0.0));
	// too short a straight for it
	CHECK_FALSE(trace_station::place_in_head(10.0, 33.2, 0.0, 40.0, true, true, 6.0).fits);
}

TEST_CASE("trace_station: the turnouts told apart over many clicks, the start within a metre and a half") {
	auto const turnouts{plain_turnouts("60E1")};
	trace_station::straight main;
	main.origin = {-500.0, 300.0};
	main.direction = heading(-137.0);
	int told{0};
	int placed{0};
	int count{0};
	for (std::size_t index = 0; index < turnouts.shapes.size(); ++index) {
		auto const &shape{turnouts.shapes[index]};
		for (unsigned seed = 100; seed < 112; ++seed) {
			auto const direction{seed % 2 == 0 ? 1 : -1};
			auto const side{seed % 3 == 0 ? -1 : 1};
			auto const points{clicked(main, shape, 80.0, direction, side, 1.0, 7.0, 12.0, 0.1, seed * 31u + static_cast<unsigned>(index))};
			auto const fit{trace_station::fit_turnout(main, turnouts.shapes, points.points, points.sigmas, turnouts.index("Rz 60E1-300-1:9"))};
			++count;
			told += fit.valid && fit.shape == static_cast<int>(index) && fit.direction == direction && fit.side == side ? 1 : 0;
			placed += fit.valid && std::abs(fit.start - 80.0) <= 1.5 ? 1 : 0;
		}
	}
	CAPTURE(count);
	CHECK(told >= count * 9 / 10);
	CHECK(placed >= count * 9 / 10);
}

namespace {

// the track out of the turnout to one alongside the main track at Spacing: the diverging track, the straight beyond it at its angle, a
// curve of R 300 m back to the angle of the main track; its offset from the main track at Along metres from the start of the turnout
double station_offset(trace_station::turnout_shape const &Shape, double const Spacing, double const Along) {
	double constexpr radius{300.0};
	auto const &end{Shape.diverging.back()};
	if (Along <= end.x)
		return diverging_at(Shape, Along).y;
	auto const heading{glm::normalize(end - Shape.diverging[Shape.diverging.size() - 2])};
	auto const angle{std::atan2(heading.y, heading.x)};
	auto const turned{radius * (1.0 - std::cos(angle))};
	auto const bend{end.x + std::max(0.0, Spacing - turned - end.y) / std::tan(angle)};
	if (Along <= bend)
		return end.y + (Along - end.x) * std::tan(angle);
	auto const centre{glm::dvec2{bend + radius * std::sin(angle), Spacing - radius}};
	if (Along >= centre.x)
		return Spacing;
	return centre.y + std::sqrt(radius * radius - (Along - centre.x) * (Along - centre.x));
}

}

TEST_CASE("trace_station: the turnout a line ends with told by the few clicks off the main track and the curve after it") {
	// the line clicked into the turnout from the track alongside: the click on the main track ending it at the tips of the blades, the
	// ones before it only where they're off the main track by 1.5 m and more (nearer they'd end the line), sparsely along the curve after it
	auto const turnouts{plain_turnouts("60E1")};
	trace_station::straight main;
	main.origin = {300.0, 700.0};
	main.direction = heading(71.0);
	int right{0};
	int count{0};
	for (auto const &name : {"Rz 60E1-190-1:9", "Rz 60E1-300-1:9", "Rz 60E1-500-1:12", "Rz 60E1-760-1:14"}) {
		auto const index{turnouts.index(name)};
		REQUIRE(index >= 0);
		auto const &shape{turnouts.shapes[index]};
		for (unsigned seed = 1; seed <= 6; ++seed) {
			CAPTURE(name);
			CAPTURE(seed);
			std::mt19937 random{seed * 977u};
			std::normal_distribution<double> noise{0.0, 0.1};
			auto const direction{seed % 2 == 0 ? 1 : -1};
			auto const side{seed % 3 == 0 ? 1 : -1};
			auto const x{main.direction * static_cast<double>(direction)};
			auto const y{left(x) * static_cast<double>(side)};
			auto const start{120.0};
			std::vector<glm::dvec2> points{main.point(start) - x * 0.5};
			std::vector<double> sigmas{0.1};
			auto first{0.0};
			while (station_offset(shape, 4.75, first) < 1.5)
				first += 0.5;
			for (auto along{first + 0.5}; along <= first + 70.0; along += 10.0) {
				points.push_back(main.point(start) + x * along + y * station_offset(shape, 4.75, along) + glm::dvec2{noise(random), noise(random)});
				sigmas.push_back(0.1);
			}
			auto const fit{trace_station::fit_turnout(main, turnouts.shapes, points, sigmas, turnouts.index("Rz 60E1-190-1:9"))};
			REQUIRE(fit.valid);
			++count;
			right += fit.shape == index && fit.direction == direction && fit.side == side && std::abs(fit.start - start) <= 2.0 ? 1 : 0;
		}
	}
	CAPTURE(right);
	CHECK(right >= count * 5 / 6);
}

TEST_CASE("trace_station: the arc joining the diverging track of a turnout onto the track alongside, its radius fitted to the clicks along it") {
	auto const turnouts{plain_turnouts("60E1")};
	auto const &shape{turnouts.shapes[turnouts.index("Rz 60E1-300-1:9")]};
	// the turnout along x from the origin, to the left; the track alongside 4.75 m off the main one
	auto const &end{shape.diverging.back()};
	auto const heading{glm::normalize(end - shape.diverging[shape.diverging.size() - 2])};
	trace_station::straight const alongside{{0.0, 4.75}, {1.0, 0.0}};
	std::mt19937 random{21};
	std::normal_distribution<double> noise{0.0, 0.05};
	std::vector<glm::dvec2> points;
	std::vector<double> sigmas;
	for (double along = end.x + 4.0; along <= 90.0; along += 5.0) {
		points.push_back({along + noise(random), station_offset(shape, 4.75, along) + noise(random)});
		sigmas.push_back(0.05);
	}
	auto const arc{trace_station::join_alongside(end, heading, alongside, points, sigmas, 500.0)};
	REQUIRE(arc.valid);
	CHECK(arc.radius == doctest::Approx(300.0).epsilon(0.05));
	CHECK(arc.rms < 0.1);
	CHECK(arc.vertex.y == doctest::Approx(4.75));
	// with no clicks along it: the radius preferred, if there's room for it, else as large as there is
	CHECK(trace_station::join_alongside(end, heading, alongside, {}, {}, 300.0).radius == doctest::Approx(300.0));
	auto const room{trace_station::join_alongside(end, heading, alongside, {}, {}, 1.0e6)};
	REQUIRE(room.valid);
	CHECK(room.radius * std::tan(0.5 * std::acos(heading.x)) == doctest::Approx(glm::distance(end, room.vertex)).epsilon(1e-6));
	// running alongside already, no arc
	CHECK_FALSE(trace_station::join_alongside({0.0, 4.75}, {1.0, 0.0}, alongside, {}, {}, 300.0).valid);
}
