#include "stdafx.h"

#include "doctest.h"

#include "editor/editorScatter.hpp"

#include <algorithm>
#include <cmath>
#include <set>

namespace
{

double closest_pair(std::vector<glm::dvec2> const &Points)
{
	double least{1e30};
	for (std::size_t i = 0; i < Points.size(); ++i)
		for (std::size_t j = i + 1; j < Points.size(); ++j)
			least = std::min(least, glm::distance(Points[i], Points[j]));
	return least;
}

} // namespace

TEST_CASE("scatter: the random fill keeps to the circle and the spacing, as dense as asked for")
{
	std::mt19937 random{7}; // NOSONAR(cpp:S2245) a repeatable test, not security
	scatter::dab_settings settings;
	settings.radius = 10.0;
	settings.spacing = 1.5;
	settings.density = 5.0;
	auto const points{scatter::dab({100.0, -50.0}, settings, {}, random)};
	// 5 a 100 m² over 314 m²: 16, the spacing lets them all in
	CHECK(points.size() == 16);
	for (auto const &p : points)
		CHECK(glm::distance(p, glm::dvec2{100.0, -50.0}) <= 10.0 + 1e-9);
	CHECK(closest_pair(points) >= 1.5 - 1e-9);
}

TEST_CASE("scatter: a dab keeps clear of what the stroke put before")
{
	std::mt19937 random{3}; // NOSONAR(cpp:S2245) a repeatable test, not security
	scatter::dab_settings settings;
	settings.radius = 4.0;
	settings.spacing = 2.0;
	settings.density = 30.0;
	auto const first{scatter::dab({0.0, 0.0}, settings, {}, random)};
	auto const second{scatter::dab({2.0, 0.0}, settings, first, random)};
	for (auto const &p : second)
		for (auto const &q : first)
			CHECK(glm::distance(p, q) >= 2.0 - 1e-9);
}

TEST_CASE("scatter: the uniform fill is a grid of the world, a second dab over the first adds only the new points")
{
	std::mt19937 random{1}; // NOSONAR(cpp:S2245) a repeatable test, not security
	scatter::dab_settings settings;
	settings.mode = scatter::fill::uniform;
	settings.radius = 5.0;
	settings.spacing = 2.0;
	auto const first{scatter::dab({0.3, 0.2}, settings, {}, random)};
	CHECK(first.size() > 15);
	CHECK(closest_pair(first) == doctest::Approx(2.0));
	auto taken{first};
	auto const second{scatter::dab({2.3, 0.2}, settings, taken, random)};
	for (auto const &p : second)
		for (auto const &q : first)
			CHECK(glm::distance(p, q) > 1.9);
	// the same place again: nothing new
	taken.insert(taken.end(), second.begin(), second.end());
	CHECK(scatter::dab({0.3, 0.2}, settings, taken, random).empty());
}

TEST_CASE("scatter: the feathered fill thins out towards the edge")
{
	std::mt19937 random{11}; // NOSONAR(cpp:S2245) a repeatable test, not security
	scatter::dab_settings settings;
	settings.mode = scatter::fill::feathered;
	settings.radius = 20.0;
	settings.spacing = 0.1;
	settings.density = 40.0;
	auto const points{scatter::dab({0.0, 0.0}, settings, {}, random)};
	int inner{0};
	int outer{0};
	for (auto const &p : points)
	{
		auto const d{glm::length(p)};
		// rings of equal area: the inner one within r·√½, the outer one beyond it
		if (d < 20.0 * std::sqrt(0.5))
			++inner;
		else
			++outer;
	}
	CHECK(inner > 2 * outer);
}

TEST_CASE("scatter: the turn and the scale of a model")
{
	std::mt19937 random{5}; // NOSONAR(cpp:S2245) a repeatable test, not security
	scatter::look_settings settings;
	settings.turn = scatter::vary::fixed;
	settings.angles = {2.0f, 45.0f, -3.0f};
	settings.size = scatter::vary::fixed;
	settings.scale = {120.0f, 80.0f, 100.0f};
	auto const fixed{scatter::pick(settings, random)};
	CHECK(fixed.angles.y == doctest::Approx(45.0f));
	CHECK(fixed.scale.x == doctest::Approx(1.2f));
	CHECK(fixed.scale.y == doctest::Approx(0.8f));

	settings.turn = scatter::vary::random;
	settings.angle_range = {5.0f, 180.0f, 0.0f};
	settings.size = scatter::vary::random;
	settings.scale_range = {20.0f, 0.0f, 0.0f};
	settings.same_scale = true;
	for (int i = 0; i < 200; ++i)
	{
		auto const drawn{scatter::pick(settings, random)};
		CHECK(std::abs(drawn.angles.x) <= 5.0f);
		CHECK(std::abs(drawn.angles.y) <= 180.0f);
		CHECK(drawn.angles.z == 0.0f);
		CHECK(drawn.scale.x >= 0.8f - 1e-6f);
		CHECK(drawn.scale.x <= 1.2f + 1e-6f);
		CHECK(drawn.scale.y == drawn.scale.x); // one for all the axes
		CHECK(drawn.scale.z == drawn.scale.x);
	}
	// never scaled to nothing
	settings.same_scale = false;
	settings.scale_range = {500.0f, 500.0f, 500.0f};
	for (int i = 0; i < 50; ++i)
	{
		auto const drawn{scatter::pick(settings, random)};
		CHECK(drawn.scale.x >= 0.01f - 1e-6f);
		CHECK(drawn.scale.y >= 0.01f - 1e-6f);
		CHECK(drawn.scale.z >= 0.01f - 1e-6f);
	}
}

TEST_CASE("scatter: a uniform dab too wide for its spacing stops at the limit at once")
{
	std::mt19937 random{2}; // NOSONAR(cpp:S2245) a repeatable test, not security
	scatter::dab_settings settings;
	settings.mode = scatter::fill::uniform;
	settings.radius = 100.0;
	settings.spacing = 0.1;
	// some 3.6 million points of the grid in the circle: cut short at the limit, not gone through
	auto const points{scatter::dab({0.0, 0.0}, settings, {}, random)};
	CHECK(static_cast<int>(points.size()) == scatter::dab_most);
}

TEST_CASE("scatter: the regular fill along a line, in rows, the height of the line followed")
{
	std::mt19937 random{4}; // NOSONAR(cpp:S2245) a repeatable test, not security
	scatter::line_settings settings;
	settings.mode = scatter::line_fill::regular;
	settings.spacing = 5.0;
	// one row on a line of no width, a model every 5 m of 100 m, the ends included
	std::vector<glm::dvec3> const line{{0.0, 10.0, 0.0}, {100.0, 20.0, 0.0}};
	auto const one{scatter::along(line, settings, random)};
	CHECK(one.size() == 21);
	for (auto const &place : one)
	{
		CHECK(place.point.y == doctest::Approx(0.0));
		CHECK(place.side == doctest::Approx(0.0));
		CHECK(place.height == doctest::Approx(10.0 + place.point.x / 10.0));
	}
	// 10 m each way: the rows at 0, ±5 and ±10 m, the ones beyond the edge of the band left out
	settings.width = 10.0;
	auto const band{scatter::along(line, settings, random)};
	for (auto const &place : band)
	{
		CHECK(std::abs(place.point.y) <= 10.0 + 1e-9);
		CHECK(place.side == doctest::Approx(std::abs(place.point.y) / 10.0));
	}
	std::set<long long> rows;
	for (auto const &place : band)
		rows.insert(std::lround(place.point.y));
	CHECK(rows == std::set<long long>{-10, -5, 0, 5, 10});
}

TEST_CASE("scatter: a line bent back on itself puts no two models on one place")
{
	std::mt19937 random{9}; // NOSONAR(cpp:S2245) a repeatable test, not security
	scatter::line_settings settings;
	settings.spacing = 2.0;
	settings.width = 6.0;
	std::vector<glm::dvec3> const line{{0.0, 0.0, 0.0}, {40.0, 0.0, 0.0}, {40.0, 0.0, 3.0}, {0.0, 0.0, 3.0}};
	auto const places{scatter::along(line, settings, random)};
	std::vector<glm::dvec2> points;
	for (auto const &place : places)
		points.push_back(place.point);
	CHECK(false == points.empty());
	CHECK(closest_pair(points) >= 1.0 - 1e-9);
}

TEST_CASE("scatter: the last point of a line given twice puts nothing past its end")
{
	std::mt19937 random{5}; // NOSONAR(cpp:S2245) a repeatable test, not security
	scatter::line_settings settings;
	settings.mode = scatter::line_fill::regular;
	settings.jitter = 0.0;
	settings.spacing = 5.0;
	settings.width = 10.0;
	// a double-click on the end: the piece of no length has no direction, the rows stay to the side of the line
	std::vector<glm::dvec3> const line{{0.0, 0.0, 0.0}, {0.0, 0.0, 100.0}, {0.0, 0.0, 100.0}};
	auto const places{scatter::along(line, settings, random)};
	CHECK(false == places.empty());
	for (auto const &place : places)
	{
		CHECK(std::abs(place.point.x) <= 10.0 + 1e-9);
		CHECK(place.point.y >= -1e-9);
		CHECK(place.point.y <= 100.0 + 1e-9);
	}
}

TEST_CASE("scatter: a regular line too long for its spacing stops at the limit")
{
	std::mt19937 random{6}; // NOSONAR(cpp:S2245) a repeatable test, not security
	scatter::line_settings settings;
	settings.mode = scatter::line_fill::regular;
	settings.jitter = 0.0;
	settings.spacing = 0.1;
	settings.width = 0.0;
	// 10 km a model every 0.1 m: 100001 places, cut short at the limit
	std::vector<glm::dvec3> const line{{0.0, 0.0, 0.0}, {10000.0, 0.0, 0.0}};
	auto const places{scatter::along(line, settings, random)};
	CHECK(static_cast<int>(places.size()) == scatter::line_most);
}

TEST_CASE("scatter: the irregular fill along a line keeps to the band and the spacing, thinner at the edges")
{
	std::mt19937 random{13}; // NOSONAR(cpp:S2245) a repeatable test, not security
	scatter::line_settings settings;
	settings.mode = scatter::line_fill::irregular;
	settings.width = 20.0;
	settings.spacing = 0.5;
	settings.density = 20.0;
	settings.thin_out = true;
	std::vector<glm::dvec3> const line{{0.0, 0.0, 0.0}, {0.0, 0.0, 200.0}};
	auto const places{scatter::along(line, settings, random)};
	CHECK(places.size() > 500);
	int inner{0};
	int outer{0};
	std::vector<glm::dvec2> points;
	for (auto const &place : places)
	{
		CHECK(std::abs(place.point.x) <= 20.0 + 1e-9);
		CHECK(place.point.y >= -1e-9);
		CHECK(place.point.y <= 200.0 + 1e-9);
		if (place.side < 0.5)
			++inner;
		else
			++outer;
		points.push_back(place.point);
	}
	CHECK(inner > 2 * outer);
	CHECK(closest_pair(points) >= 0.5 - 1e-9);
}

TEST_CASE("scatter: the size of a model across the band, full on the line, as set at the edge")
{
	scatter::line_settings settings;
	settings.edge_scale = 40.0;
	CHECK(scatter::side_scale(settings, 0.0) == doctest::Approx(1.0f));
	CHECK(scatter::side_scale(settings, 0.5) == doctest::Approx(0.7f));
	CHECK(scatter::side_scale(settings, 1.0) == doctest::Approx(0.4f));
	settings.edge_scale = 150.0;
	CHECK(scatter::side_scale(settings, 1.0) == doctest::Approx(1.5f));
	// one point is no line
	std::mt19937 random{1}; // NOSONAR(cpp:S2245) a repeatable test, not security
	CHECK(scatter::along({{0.0, 0.0, 0.0}}, settings, random).empty());
}
