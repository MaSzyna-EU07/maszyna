#include "stdafx.h"

#include "editor/editorScatter.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <numbers>

namespace scatter
{

namespace
{

bool clear_of(glm::dvec2 const &Point, double const Spacing, std::vector<glm::dvec2> const &Points)
{
	auto const least{Spacing * Spacing};
	return std::ranges::all_of(Points, [&](glm::dvec2 const &Other) {
		auto const d{Point - Other};
		return glm::dot(d, d) >= least;
	});
}

bool clear_of(glm::dvec2 const &Point, double const Spacing, std::vector<glm::dvec2> const &Taken, std::vector<glm::dvec2> const &Made)
{
	return clear_of(Point, Spacing, Taken) && clear_of(Point, Spacing, Made);
}

// one row of the grid of triangles in the circle, added to Made until it holds dab_most points
void grid_row(long long const Row, glm::dvec2 const &Centre, double const Radius, double const Spacing, std::vector<glm::dvec2> const &Taken, std::vector<glm::dvec2> &Made)
{
	auto const row{Spacing * std::sqrt(3.0) * 0.5};
	auto const shift{(Row % 2 == 0) ? 0.0 : 0.5 * Spacing};
	auto const z{static_cast<double>(Row) * row};
	auto const first{static_cast<long long>(std::floor((Centre.x - Radius - shift) / Spacing))};
	auto const last{static_cast<long long>(std::ceil((Centre.x + Radius - shift) / Spacing))};
	for (auto c = first; c <= last && static_cast<int>(Made.size()) < dab_most; ++c)
	{
		glm::dvec2 const point{static_cast<double>(c) * Spacing + shift, z};
		// the points of one dab are Spacing apart by the grid itself; only a point an earlier dab put is there already, the same
		// point again, a hair away at most
		if (glm::distance(point, Centre) <= Radius && clear_of(point, 0.5 * Spacing, Taken))
			Made.push_back(point);
	}
}

// the points of the grid of triangles of the world (rows Spacing·√3/2 apart, every other one shifted by half) in the circle,
// dab_most at most
std::vector<glm::dvec2> grid_points(glm::dvec2 const &Centre, double const Radius, double const Spacing, std::vector<glm::dvec2> const &Taken)
{
	std::vector<glm::dvec2> made;
	auto const row{Spacing * std::sqrt(3.0) * 0.5};
	auto const first_row{static_cast<long long>(std::floor((Centre.y - Radius) / row))};
	auto const last_row{static_cast<long long>(std::ceil((Centre.y + Radius) / row))};
	for (auto r = first_row; r <= last_row && static_cast<int>(made.size()) < dab_most; ++r)
		grid_row(r, Centre, Radius, Spacing, Taken, made);
	return made;
}

// the line drawn, measured along its length: a point at a distance along it and an offset to its side
class polyline
{
  public:
	explicit polyline(std::vector<glm::dvec3> const &Points) : m_points{Points}
	{
		m_lengths.reserve(m_points.size());
		auto total{0.0};
		m_lengths.push_back(total);
		for (std::size_t i = 1; i < m_points.size(); ++i)
		{
			total += glm::distance(flat(m_points[i - 1]), flat(m_points[i]));
			m_lengths.push_back(total);
		}
	}

	double length() const
	{
		return m_lengths.back();
	}

	// the place At metres along the line, Offset metres to its left (negative: to the right)
	line_place at(double const At, double const Offset, double const Width) const
	{
		auto const piece{segment(At)};
		auto const &a{m_points[piece]};
		auto const &b{m_points[piece + 1]};
		auto const span{m_lengths[piece + 1] - m_lengths[piece]};
		auto const t{span > 0.0 ? std::clamp((At - m_lengths[piece]) / span, 0.0, 1.0) : 0.0};
		auto const direction{span > 0.0 ? (flat(b) - flat(a)) / span : glm::dvec2{1.0, 0.0}};
		glm::dvec2 const normal{-direction.y, direction.x};
		line_place place;
		place.point = glm::mix(flat(a), flat(b), t) + normal * Offset;
		place.height = std::lerp(a.y, b.y, t);
		place.side = Width > 0.0 ? std::min(1.0, std::abs(Offset) / Width) : 0.0;
		return place;
	}

  private:
	static glm::dvec2 flat(glm::dvec3 const &Point)
	{
		return {Point.x, Point.z};
	}

	// the piece of the line At metres along it lies on
	std::size_t segment(double const At) const
	{
		auto const next{std::ranges::upper_bound(m_lengths, At)};
		auto const index{static_cast<std::size_t>(std::distance(m_lengths.begin(), next))};
		return std::clamp<std::size_t>(index, 1, m_points.size() - 1) - 1;
	}

	std::vector<glm::dvec3> const &m_points;
	std::vector<double> m_lengths;
};

// the places taken by a line, sorted into square cells as wide as the spacing: a long line with thousands of models isn't gone
// through whole for every new one
class spaced_places
{
  public:
	explicit spaced_places(double const Spacing) : m_spacing{Spacing}
	{
	}

	bool clear(glm::dvec2 const &Point, double const Least) const
	{
		auto const [cx, cz]{cell(Point)};
		for (auto x = cx - 1; x <= cx + 1; ++x)
		{
			for (auto z = cz - 1; z <= cz + 1; ++z)
			{
				if (auto const found{m_cells.find(key(x, z))}; found != m_cells.end() && false == clear_of(Point, Least, found->second))
					return false;
			}
		}
		return true;
	}

	void add(glm::dvec2 const &Point)
	{
		auto const [cx, cz]{cell(Point)};
		m_cells[key(cx, cz)].push_back(Point);
	}

  private:
	std::pair<long long, long long> cell(glm::dvec2 const &Point) const
	{
		return {static_cast<long long>(std::floor(Point.x / m_spacing)), static_cast<long long>(std::floor(Point.y / m_spacing))};
	}

	static long long key(long long const X, long long const Z)
	{
		// the cells of a scenery, tens of kilometres across at most, fit the 32 bits of each half
		return (X << 32) ^ (Z & 0xffffffffLL);
	}

	double m_spacing;
	std::unordered_map<long long, std::vector<glm::dvec2>> m_cells;
};

// the regular fill: rows along the line, each a list of places a spacing apart
struct regular_line
{
	polyline const &line;
	line_settings const &settings;
	double spacing;
	std::mt19937 &random; // NOSONAR(cpp:S2245) placement of scenery models, not security

	void row(long long const Row, spaced_places &Taken, std::vector<line_place> &Made) const
	{
		auto const offset{static_cast<double>(Row) * spacing};
		auto const shift{(Row % 2 == 0) ? 0.0 : 0.5 * spacing};
		auto const jitter{std::clamp(settings.jitter, 0.0, 0.5) * spacing};
		std::uniform_real_distribution each_way{-1.0, 1.0};
		for (auto at = shift; at <= line.length() + 1e-9 && static_cast<int>(Made.size()) < line_most; at += spacing)
		{
			auto const along_off{jitter > 0.0 ? jitter * each_way(random) : 0.0};
			auto const side_off{jitter > 0.0 ? jitter * each_way(random) : 0.0};
			auto place{line.at(std::clamp(at + along_off, 0.0, line.length()), offset + side_off, settings.width)};
			// where the line bends, the rows on the inner side of the bend run into each other: one model of the two
			if (false == Taken.clear(place.point, 0.5 * spacing))
				continue;
			Taken.add(place.point);
			Made.push_back(place);
		}
	}

	std::vector<line_place> make() const
	{
		std::vector<line_place> made;
		spaced_places taken{spacing};
		auto const rows{static_cast<long long>(std::floor(std::max(0.0, settings.width) / spacing + 1e-9))};
		// the row on the line first, then the pairs of rows outwards: cut short by the limit, the band stays even
		row(0, taken, made);
		for (long long r = 1; r <= rows && static_cast<int>(made.size()) < line_most; ++r)
		{
			row(r, taken, made);
			row(-r, taken, made);
		}
		return made;
	}
};

// the irregular fill: random places in the band, the spacing kept
std::vector<line_place> irregular_line(polyline const &Line, line_settings const &Settings, double const Spacing, std::mt19937 &Random) // NOSONAR(cpp:S2245) placement of scenery models, not security
{
	auto const width{std::max(0.0, Settings.width)};
	auto const density{std::max(0.0, Settings.density)};
	// over the area of the band; a line of no width gets the density per spacing of its length
	auto const amount{width > 0.0 ? density * Line.length() * 2.0 * width / 100.0 : density * Line.length() / Spacing};
	auto const wanted{static_cast<int>(std::min(static_cast<double>(line_most), std::round(amount)))};
	std::uniform_real_distribution unit{0.0, 1.0};
	std::vector<line_place> made;
	spaced_places taken{Spacing};
	int constexpr tries_each{30};
	auto attempts{static_cast<long long>(wanted) * tries_each};
	while (attempts > 0 && static_cast<int>(made.size()) < wanted)
	{
		--attempts;
		auto const across{2.0 * unit(Random) - 1.0};
		// fewer towards the edges: kept with the chance falling from 1 on the line to 0 at the edge
		if (Settings.thin_out && unit(Random) > (1.0 - across * across) * (1.0 - across * across))
			continue;
		auto const place{Line.at(Line.length() * unit(Random), width * across, width)};
		if (false == taken.clear(place.point, Spacing))
			continue;
		taken.add(place.point);
		made.push_back(place);
	}
	return made;
}

} // namespace

std::vector<line_place> along(std::vector<glm::dvec3> const &Points, line_settings const &Settings, std::mt19937 &Random) // NOSONAR(cpp:S2245) placement of scenery models, not security
{
	if (Points.size() < 2)
		return {};
	polyline const line{Points};
	if (line.length() <= 0.0)
		return {};
	auto const spacing{std::max(0.1, Settings.spacing)};
	if (Settings.mode == line_fill::irregular)
		return irregular_line(line, Settings, spacing, Random);
	return regular_line{line, Settings, spacing, Random}.make();
}

float side_scale(line_settings const &Settings, double const Side)
{
	auto const edge{std::max(1.0, Settings.edge_scale) / 100.0};
	return static_cast<float>(std::lerp(1.0, edge, std::clamp(Side, 0.0, 1.0)));
}

std::vector<glm::dvec2> dab(glm::dvec2 const &Centre, dab_settings const &Settings, std::vector<glm::dvec2> const &Taken, std::mt19937 &Random) // NOSONAR(cpp:S2245) placement of scenery models, not security
{
	auto const radius{std::max(0.1, Settings.radius)};
	auto const spacing{std::max(0.05, Settings.spacing)};
	// only what the stroke put near this circle matters: a long stroke isn't gone through whole for every point
	std::vector<glm::dvec2> nearby;
	auto const reach{radius + spacing};
	std::ranges::copy_if(Taken, std::back_inserter(nearby), [&](glm::dvec2 const &Point) { return glm::distance(Point, Centre) <= reach; });
	if (Settings.mode == fill::uniform)
		return grid_points(Centre, radius, spacing, nearby);
	// as many as the density asks for in the circle (no more than a dab can put at once), each tried a number of times before
	// it's given up for the spacing
	auto const wanted{std::min(dab_most, static_cast<int>(std::lround(std::max(0.0, Settings.density) * std::numbers::pi * radius * radius / 100.0)))};
	std::uniform_real_distribution unit{0.0, 1.0};
	std::vector<glm::dvec2> made;
	int constexpr tries_each{30};
	auto attempts{wanted * tries_each};
	while (attempts > 0 && static_cast<int>(made.size()) < wanted)
	{
		--attempts;
		// even over the area: the square root of the share of the radius
		auto const share{std::sqrt(unit(Random))};
		auto const angle{2.0 * std::numbers::pi * unit(Random)};
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
