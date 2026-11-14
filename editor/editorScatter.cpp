#include "editor/editorScatter.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace scatter
{

namespace
{

double constexpr kPi{3.14159265358979323846};

bool clear_of(glm::dvec2 const &Point, double const Spacing, std::vector<glm::dvec2> const &Taken, std::vector<glm::dvec2> const &Made)
{
	auto const least{Spacing * Spacing};
	auto const far_enough = [&](glm::dvec2 const &Other) {
		auto const d{Point - Other};
		return glm::dot(d, d) >= least;
	};
	return std::all_of(Taken.begin(), Taken.end(), far_enough) && std::all_of(Made.begin(), Made.end(), far_enough);
}

// the points of the grid of triangles of the world (rows Spacing·√3/2 apart, every other one shifted by half) in the circle
std::vector<glm::dvec2> grid_points(glm::dvec2 const &Centre, double const Radius, double const Spacing, std::vector<glm::dvec2> const &Taken)
{
	std::vector<glm::dvec2> made;
	auto const row{Spacing * std::sqrt(3.0) * 0.5};
	auto const first_row{static_cast<long long>(std::floor((Centre.y - Radius) / row))};
	auto const last_row{static_cast<long long>(std::ceil((Centre.y + Radius) / row))};
	for (auto r = first_row; r <= last_row; ++r)
	{
		auto const shift{(r % 2 == 0) ? 0.0 : 0.5 * Spacing};
		auto const z{static_cast<double>(r) * row};
		auto const first{static_cast<long long>(std::floor((Centre.x - Radius - shift) / Spacing))};
		auto const last{static_cast<long long>(std::ceil((Centre.x + Radius - shift) / Spacing))};
		for (auto c = first; c <= last; ++c)
		{
			glm::dvec2 const point{static_cast<double>(c) * Spacing + shift, z};
			// a point of the grid put by an earlier dab is there: the same point again, a hair away at most
			if (glm::distance(point, Centre) <= Radius && clear_of(point, 0.5 * Spacing, Taken, made))
				made.push_back(point);
		}
	}
	return made;
}

} // namespace

std::vector<glm::dvec2> dab(glm::dvec2 const &Centre, dab_settings const &Settings, std::vector<glm::dvec2> const &Taken, std::mt19937 &Random) // NOSONAR(cpp:S2245) placement of scenery models, not security
{
	auto const radius{std::max(0.1, Settings.radius)};
	auto const spacing{std::max(0.05, Settings.spacing)};
	// only what the stroke put near this circle matters: a long stroke isn't gone through whole for every point
	std::vector<glm::dvec2> nearby;
	auto const reach{radius + spacing};
	std::copy_if(Taken.begin(), Taken.end(), std::back_inserter(nearby), [&](glm::dvec2 const &Point) { return glm::distance(Point, Centre) <= reach; });
	if (Settings.mode == fill::uniform)
		return grid_points(Centre, radius, spacing, nearby);
	// as many as the density asks for in the circle (no more than a dab can put at once), each tried a number of times before
	// it's given up for the spacing
	int constexpr most{2000};
	auto const wanted{static_cast<int>(std::min<double>(most, std::lround(std::max(0.0, Settings.density) * kPi * radius * radius / 100.0)))};
	std::uniform_real_distribution<double> unit{0.0, 1.0};
	std::vector<glm::dvec2> made;
	int constexpr tries_each{30};
	for (int attempt = 0; attempt < wanted * tries_each && static_cast<int>(made.size()) < wanted; ++attempt)
	{
		// even over the area: the square root of the share of the radius
		auto const share{std::sqrt(unit(Random))};
		auto const angle{2.0 * kPi * unit(Random)};
		if (Settings.mode == fill::feathered)
		{
			// fewer towards the edge: kept with the chance falling from 1 in the middle to 0 at the edge
			auto const keep{(1.0 - share * share) * (1.0 - share * share)};
			if (unit(Random) > keep)
				continue;
		}
		glm::dvec2 const point{Centre.x + radius * share * std::cos(angle), Centre.y + radius * share * std::sin(angle)};
		if (clear_of(point, spacing, nearby, made))
			made.push_back(point);
	}
	return made;
}

double dab_step(dab_settings const &Settings)
{
	return std::max({0.05, Settings.spacing, 0.5 * Settings.radius});
}

look pick(look_settings const &Settings, std::mt19937 &Random) // NOSONAR(cpp:S2245) placement of scenery models, not security
{
	look out;
	auto const spread = [&](float const Range) {
		if (Range <= 0.0f)
			return 0.0f;
		std::uniform_real_distribution<float> each_way{-Range, Range};
		return each_way(Random);
	};
	if (Settings.turn == vary::fixed)
		out.angles = Settings.angles;
	else
		out.angles = {spread(Settings.angle_range.x), spread(Settings.angle_range.y), spread(Settings.angle_range.z)};
	glm::vec3 percent{Settings.scale};
	if (Settings.size == vary::random)
	{
		if (Settings.same_scale)
		{
			auto const common{spread(Settings.scale_range.x)};
			percent = glm::vec3{100.0f + common};
		}
		else
			percent = glm::vec3{100.0f + spread(Settings.scale_range.x), 100.0f + spread(Settings.scale_range.y), 100.0f + spread(Settings.scale_range.z)};
	}
	out.scale = glm::max(percent, glm::vec3{1.0f}) / 100.0f;
	return out;
}

} // namespace scatter
