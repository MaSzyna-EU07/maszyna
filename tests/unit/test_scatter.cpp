#include "stdafx.h"

#include "doctest.h"

#include "editor/editorScatter.hpp"

#include <algorithm>
#include <cmath>

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
