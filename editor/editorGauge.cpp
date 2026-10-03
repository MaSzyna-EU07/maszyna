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

#include <fstream>
#include <sstream>

namespace gauge
{

std::vector<profile> default_profiles()
{
	std::vector<profile::point> const lower{{1.645, 0.05}, {1.645, 0.30}, {1.720, 0.30}, {1.720, 1.10}, {2.200, 1.10}, {2.200, 3.85}, {1.900, 4.25}};
	profile plain{"A (bez sieci)", lower};
	plain.outline.insert(plain.outline.end(), {{1.400, 4.85}, {0.0, 4.85}});
	profile electrified{"C (sieć trakcyjna)", lower};
	electrified.outline.insert(electrified.outline.end(), {{1.700, 4.85}, {1.400, 5.60}, {1.000, 6.00}, {0.0, 6.00}});
	return {plain, electrified};
}

// one outline per line: the name in quotes, then pairs of half-width and height
std::vector<profile> load_profiles(std::string const &File)
{
	std::ifstream input{File};
	std::vector<profile> profiles;
	std::string line;
	while (std::getline(input, line))
	{
		std::istringstream stream{line};
		profile profile;
		if (false == static_cast<bool>(stream >> std::quoted(profile.name)))
			continue;
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
		output << std::quoted(profile.name);
		for (auto const &point : profile.outline)
			output << ' ' << point.half_width << ' ' << point.height;
		output << '\n';
	}
}

double curve_widening(double const Radius)
{
	// smallest radius of the class and its widening in mm; the radius between the classes takes the larger one
	static constexpr std::pair<double, double> table[]{{3500, 10},  {2500, 15},  {1800, 20},  {1500, 25},  {1200, 30},  {1000, 35},
	                                                   {900, 40},   {800, 45},   {700, 50},   {600, 60},   {500, 75},   {450, 80},
	                                                   {400, 90},   {350, 105},  {300, 120},  {280, 130},  {260, 140},  {250, 145},
	                                                   {240, 150},  {220, 165},  {200, 180},  {190, 190},  {180, 200}};
	if (Radius <= 0.0 || Radius >= 4000.0)
		return 0.0;
	for (auto const &[radius, widening] : table)
		if (Radius >= radius)
			return widening * 0.001;
	return 36.0 / Radius;
}

double cant_widening(double const Cant, double const Height)
{
	return Height * Cant / 1.5;
}

double half_width(profile const &Profile, section const &Section, double const Height, int const Side)
{
	double width{-1.0};
	for (std::size_t i = 1; i < Profile.outline.size(); ++i)
	{
		auto const &a{Profile.outline[i - 1]};
		auto const &b{Profile.outline[i]};
		if (Height < std::min(a.height, b.height) || Height > std::max(a.height, b.height))
			continue;
		auto const span{b.height - a.height};
		width = std::max(width, std::abs(span) < 1e-9 ? std::max(a.half_width, b.half_width) : a.half_width + (b.half_width - a.half_width) * (Height - a.height) / span);
	}
	if (width < 0.0)
		return width;
	width += curve_widening(Section.radius);
	if (Section.inner != 0 && Side == Section.inner)
		width += cant_widening(Section.cant, Height);
	return width;
}

double intrusion(profile const &Profile, section const &Section, double const Lateral, double const Height)
{
	auto const width{half_width(Profile, Section, Height, Lateral >= 0.0 ? 1 : -1)};
	if (width < 0.0)
		return 0.0;
	return width - std::abs(Lateral);
}

} // namespace gauge
