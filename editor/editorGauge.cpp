/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorGauge.hpp"

#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

namespace gauge
{

namespace
{

// the widening runs up ahead of the change: on the inner side from 20 m before it, on the outer side from 26 m
// to 6 m before it (figures 3 to 5 of the standard)
double constexpr inner_lead{20.0};
double constexpr outer_lead{26.0};
double constexpr outer_full{6.0};

struct side_widening
{
	double inner{0.0};
	double outer{0.0};
};

// above the lower part the gauge includes the curves of R >= 250 m already
side_widening upper_widening(double const Radius)
{
	if (Radius >= 250.0)
		return {};
	return {std::max(0.0, 50.0 / Radius - 0.185), std::max(0.0, 60.0 / Radius - 0.225)};
}

side_widening lower_widening(double const Radius)
{
	if (Radius >= 250.0)
		return {3.75 / Radius, 3.75 / Radius};
	return upper_widening(Radius);
}

// values at the points of a route, read as linear between them and continued past the ends
class field
{
public:
	field(std::vector<double> const &Chainage, std::vector<double> Values) : m_chainage{Chainage}, m_values{std::move(Values)}, m_integral(m_values.size(), 0.0)
	{
		for (std::size_t i = 1; i < m_values.size(); ++i)
			m_integral[i] = m_integral[i - 1] + 0.5 * (m_values[i - 1] + m_values[i]) * (m_chainage[i] - m_chainage[i - 1]);
	}

	double mean(double const From, double const To) const
	{
		return (integral(To) - integral(From)) / (To - From);
	}

	// larger of the means over the window placed ahead of the point in both directions
	double ahead(double const Chainage, double const Near, double const Far) const
	{
		return std::max(mean(Chainage + Near, Chainage + Far), mean(Chainage - Far, Chainage - Near));
	}

private:
	double integral(double const Chainage) const
	{
		if (Chainage <= m_chainage.front())
			return m_values.front() * (Chainage - m_chainage.front());
		if (Chainage >= m_chainage.back())
			return m_integral.back() + m_values.back() * (Chainage - m_chainage.back());
		auto const next{static_cast<std::size_t>(std::upper_bound(m_chainage.begin(), m_chainage.end(), Chainage) - m_chainage.begin())};
		auto const i{next - 1};
		auto const span{m_chainage[next] - m_chainage[i]};
		auto const offset{Chainage - m_chainage[i]};
		auto const value{span > 0.0 ? m_values[i] + (m_values[next] - m_values[i]) * offset / span : m_values[i]};
		return m_integral[i] + 0.5 * (m_values[i] + value) * offset;
	}

	std::vector<double> const &m_chainage;
	std::vector<double> m_values;
	std::vector<double> m_integral;
};

// half of the distance between the heads of the rails
double constexpr rail_spread{0.75};

// the point turned by the angle of the sine about the head of the lower rail
place turn(section const &Section, place const &Point, double const Sine)
{
	auto const pivot{Section.tilt > 0.0 ? -rail_spread : rail_spread};
	auto const cosine{std::sqrt(std::max(0.0, 1.0 - Sine * Sine))};
	auto const lateral{Point.lateral - pivot};
	return {pivot + lateral * cosine - Point.height * Sine, lateral * Sine + Point.height * cosine};
}

// lower part common to all gauges, from the axis between the rails: the wheel zone at the running edge (717.5 mm
// from the axis) with the flangeway of the dimension "b" 58 mm wide and 38 mm deep, the dimension "a" of 150 mm
// at the rail top outside of it, then the limit installation gauge up to 380 mm
std::vector<profile::point> const common_lower{{0.6595, 0.055}, {0.6595, -0.038}, {0.7175, -0.038}, {0.7175, 0.0}, {0.8675, 0.0},
                                               {0.8675, 0.055}, {1.211, 0.055},   {1.585, 0.380},   {1.675, 0.380}};

// the wheel zone follows the rails, without the widening in the curves
bool wheel_zone(profile::point const &Point)
{
	return Point.height <= 0.055 + 1e-9 && Point.half_width < 1.0;
}

// names of the kinds in the file
char const *const kind_names[]{"unified", "installation", "road"};

// outline from the points listed top down, in millimetres, as in the tables of the standard, over the lower part
profile installation(std::string const &Name, std::vector<std::pair<int, int>> const &Points)
{
	profile result{Name, kind::installation, common_lower};
	for (auto point = Points.rbegin(); point != Points.rend(); ++point)
		result.outline.push_back({point->second * 0.001, point->first * 0.001});
	return result;
}

} // namespace

std::vector<profile> default_profiles()
{
	// limit installation gauge up to 1170 mm, then the outline of the unified gauge
	auto lower{common_lower};
	lower.insert(lower.end(), {{1.675, 1.170}, {2.000, 1.170}});
	std::vector<profile::point> const gpl1{{2.000, 3.050}, {1.900, 3.850}, {1.800, 4.250}, {1.600, 4.500}, {1.450, 4.632}};
	std::vector<profile::point> const gpl2{{2.000, 3.625}, {1.920, 4.900}};
	std::vector<profile::point> const pantograph{{1.450, 6.600}, {1.150, 6.900}, {0.0, 6.900}};
	auto const make{[&](std::string const &Name, std::vector<std::vector<profile::point>> const &Parts) {
		profile result{Name, kind::unified, lower};
		for (auto const &part : Parts)
			result.outline.insert(result.outline.end(), part.begin(), part.end());
		return result;
	}};
	// tables 3, 5, 7, 9 of the type cards and the drawing of GC, above 380 mm
	return {make("GPL-1", {gpl1, {{1.260, 4.800}, {1.180, 4.850}, {0.0, 4.850}}}),
	        make("GPL-1 + pantograf", {gpl1, pantograph}),
	        make("GPL-2", {gpl2, {{0.0, 4.900}}}),
	        make("GPL-2 + pantograf", {gpl2, {{1.450, 4.900}}, pantograph}),
	        installation("G1 graniczna (GSZ)", {{4375, 0}, {4375, 666}, {4010, 1255}, {3700, 1550}, {3250, 1755}, {1170, 1695}, {1170, 1675}}),
	        installation("G1 nominalna (NSZ)", {{4385, 0}, {4385, 785}, {4010, 1365}, {3700, 1655}, {3250, 1860}, {1170, 1860}, {1170, 1675}}),
	        installation("G2 graniczna (GSZ)", {{4765, 0}, {4765, 940}, {3835, 1600}, {3530, 1765}, {1170, 1695}, {1170, 1675}}),
	        installation("G2 nominalna (NSZ)", {{4770, 0}, {4770, 1060}, {3835, 1710}, {3530, 1870}, {1170, 1870}, {1170, 1675}}),
	        installation("GA graniczna (GSZ)", {{4415, 0}, {4415, 687}, {4080, 1225}, {3808, 1490}, {3250, 1755}, {1170, 1700}, {1170, 1675}}),
	        installation("GA nominalna (NSZ)", {{4425, 0}, {4425, 805}, {4080, 1340}, {3880, 1600}, {3250, 1860}, {1170, 1860}, {1170, 1675}}),
	        installation("GB graniczna (GSZ)", {{4415, 0}, {4415, 687}, {4080, 1495}, {3250, 1755}, {1170, 1700}, {1170, 1675}}),
	        installation("GB nominalna (NSZ)", {{4425, 0}, {4425, 805}, {4110, 1610}, {3250, 1860}, {1170, 1860}, {1170, 1675}}),
	        installation("GC graniczna (GSZ)", {{4800, 0}, {4800, 1693}, {3550, 1765}, {1170, 1765}, {1170, 1675}}),
	        installation("GC nominalna (NSZ)", {{4820, 0}, {4820, 1815}, {3550, 1870}, {1170, 1870}, {1170, 1675}}),
	        {"Droga 4,70 m", kind::road, {{0.5, 0.0}, {0.5, 4.70}}},
	        {"Droga 4,50 m", kind::road, {{0.5, 0.0}, {0.5, 4.50}}}};
}

// one outline per line: the name in quotes, the kind, then pairs of half-width and height
std::vector<profile> load_profiles(std::string const &File)
{
	std::ifstream input{File};
	std::vector<profile> profiles;
	std::string line;
	while (std::getline(input, line))
	{
		std::istringstream stream{line};
		profile profile;
		std::string kind;
		if (false == static_cast<bool>(stream >> std::quoted(profile.name) >> kind))
			continue;
		auto const known{std::find(std::begin(kind_names), std::end(kind_names), kind)};
		if (known == std::end(kind_names))
			continue;
		profile.kind = static_cast<gauge::kind>(known - std::begin(kind_names));
		profile::point point;
		while (stream >> point.half_width >> point.height)
			profile.outline.push_back(point);
		if (profile.outline.size() >= 2)
			profiles.push_back(std::move(profile));
	}
	return profiles.empty() ? default_profiles() : profiles;
}

void save_profiles(std::string const &File, std::vector<profile> const &Profiles)
{
	std::ofstream output{File};
	for (auto const &profile : Profiles)
	{
		output << std::quoted(profile.name) << ' ' << kind_names[static_cast<int>(profile.kind)];
		for (auto const &point : profile.outline)
			output << ' ' << point.half_width << ' ' << point.height;
		output << '\n';
	}
}

std::vector<section> sections(kind const Kind, std::vector<double> const &Chainage, std::vector<double> const &Curvature, std::vector<double> const &Cant)
{
	auto const count{Chainage.size()};
	std::vector<section> result(count);
	if (count == 0 || Kind == kind::road)
		return result;
	// per side, the parts when it's the inner side of the curve and when it's the outer one
	for (int side = 0; side < 2; ++side)
	{
		std::vector<double> upperinner(count), upperouter(count), lowerinner(count), lowerouter(count), cantinner(count), cantouter(count);
		for (std::size_t i = 0; i < count; ++i)
		{
			if (std::abs(Curvature[i]) < 1e-6)
				continue;
			auto const radius{1.0 / std::abs(Curvature[i])};
			bool const inner{(Curvature[i] > 0.0) == (side == 1)};
			auto const upper{Kind == kind::unified ? upper_widening(radius) : lower_widening(radius)};
			auto const lower{lower_widening(radius)};
			(inner ? upperinner : upperouter)[i] = inner ? upper.inner : upper.outer;
			(inner ? lowerinner : lowerouter)[i] = inner ? lower.inner : lower.outer;
			(inner ? cantinner : cantouter)[i] = Cant[i];
		}
		field const fields[]{{Chainage, std::move(upperinner)}, {Chainage, std::move(upperouter)}, {Chainage, std::move(lowerinner)},
		                     {Chainage, std::move(lowerouter)}, {Chainage, std::move(cantinner)},  {Chainage, std::move(cantouter)}};
		for (std::size_t i = 0; i < count; ++i)
		{
			auto const inner{[&](field const &Field) { return Field.ahead(Chainage[i], 0.0, inner_lead); }};
			auto const outer{[&](field const &Field) { return Field.ahead(Chainage[i], outer_full, outer_lead); }};
			result[i].upper[side] = inner(fields[0]) + outer(fields[1]);
			result[i].lower[side] = inner(fields[2]) + outer(fields[3]);
			result[i].cant[side] = inner(fields[4]) - outer(fields[5]);
		}
	}
	return result;
}

place outline_point(profile const &Profile, section const &Section, std::size_t const Index, int const Side)
{
	auto const &point{Profile.outline[Index]};
	// the step at the top of the lower part belongs to it from below
	bool const lower{point.height < lower_part - 1e-9 || (point.height < lower_part + 1e-9 && Index > 0 && Profile.outline[Index - 1].height < lower_part - 1e-9)};
	auto const i{Side > 0 ? 1 : 0};
	auto const widening{wheel_zone(point) ? 0.0 : lower ? Section.lower[i] : Section.upper[i] + Section.cant[i] * point.height / 1.5};
	auto const width{Section.base + point.half_width + widening};
	place const result{Side * width, point.height};
	return lower && Section.tilt != 0.0 ? turn(Section, result, Section.tilt) : result;
}

double intrusion(profile const &Profile, section const &Section, double const Lateral, double const Height)
{
	auto const count{Profile.outline.size()};
	if (count < 2)
		return 0.0;
	// the outline from the bottom right up and down to the bottom left, closed across the bottom
	auto const vertex{[&](std::size_t const Index) {
		return Index < count ? outline_point(Profile, Section, Index, -1) : outline_point(Profile, Section, 2 * count - 1 - Index, 1);
	}};
	bool inside{false};
	auto nearest{std::numeric_limits<double>::max()};
	auto previous{vertex(2 * count - 1)};
	for (std::size_t i = 0; i < 2 * count; ++i)
	{
		auto const current{vertex(i)};
		auto const lateral{current.lateral - previous.lateral};
		auto const height{current.height - previous.height};
		if ((current.height > Height) != (previous.height > Height) && Lateral < previous.lateral + lateral * (Height - previous.height) / height)
			inside = false == inside;
		auto const length{lateral * lateral + height * height};
		auto const t{length > 0.0 ? std::clamp(((Lateral - previous.lateral) * lateral + (Height - previous.height) * height) / length, 0.0, 1.0) : 0.0};
		nearest = std::min(nearest, std::hypot(Lateral - previous.lateral - lateral * t, Height - previous.height - height * t));
		previous = current;
	}
	return inside ? nearest : -nearest;
}

} // namespace gauge
