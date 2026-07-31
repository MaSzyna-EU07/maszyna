/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/polandmap.h"

#include "editor/puwg1992.h"

namespace editor
{

namespace
{

// geographic coordinates, degrees, as (latitude, longitude); projected to EPSG:2180 on first use so
// the numbers stay readable and there is a single place where the frame is decided
struct geo_point
{
	double latitude;
	double longitude;
};

// the border walked clockwise from the north-west corner: Baltic coast, the Kaliningrad and
// Lithuanian borders, the Bug, the Carpathians and Sudetes, then the Nysa and Odra back north.
// tens of kilometres of error in places - this is a locator sketch, nothing more
constexpr geo_point kOutline[]{
    {53.93, 14.27}, {54.05, 14.60}, {54.18, 15.58}, {54.42, 16.41}, {54.58, 16.86}, {54.76, 17.55},
    {54.79, 18.40}, {54.60, 18.65}, {54.52, 18.55}, {54.35, 18.95}, {54.37, 19.35}, {54.45, 19.64},
    {54.37, 20.50}, {54.35, 21.50}, {54.36, 22.00}, {54.36, 22.79}, {54.25, 23.20}, {54.15, 23.48},
    {53.95, 23.90}, {53.20, 23.93}, {52.70, 23.85}, {52.30, 23.60}, {52.07, 23.62}, {51.55, 23.55},
    {50.87, 24.15}, {50.72, 24.15}, {50.40, 23.70}, {50.28, 23.50}, {49.80, 22.92}, {49.20, 22.68},
    {49.09, 22.55}, {49.25, 22.00}, {49.42, 21.75}, {49.35, 21.00}, {49.40, 20.50}, {49.20, 20.10},
    {49.40, 19.80}, {49.40, 19.50}, {49.40, 19.20}, {49.51, 18.85}, {49.75, 18.63}, {49.95, 18.05},
    {50.10, 17.70}, {50.31, 17.38}, {50.45, 16.90}, {50.20, 16.60}, {50.40, 16.35}, {50.44, 16.25},
    {50.75, 15.80}, {50.75, 15.55}, {50.85, 15.00}, {50.87, 14.82}, {51.30, 14.95}, {51.60, 14.75},
    {52.00, 14.70}, {52.35, 14.55}, {52.59, 14.65}, {52.85, 14.40}, {53.10, 14.35}, {53.55, 14.30},
    {53.75, 14.28}};

struct named_geo_point
{
	char const *name;
	geo_point position;
};

constexpr named_geo_point kPlaces[]{
    {"Warszawa", {52.23, 21.01}},     {"Krakow", {50.06, 19.94}},       {"Lodz", {51.77, 19.46}},
    {"Wroclaw", {51.11, 17.03}},      {"Poznan", {52.41, 16.93}},       {"Gdansk", {54.35, 18.65}},
    {"Szczecin", {53.43, 14.55}},     {"Bydgoszcz", {53.12, 18.01}},    {"Lublin", {51.25, 22.57}},
    {"Bialystok", {53.13, 23.16}},    {"Katowice", {50.26, 19.02}},     {"Rzeszow", {50.04, 22.00}},
    {"Olsztyn", {53.78, 20.49}},      {"Kielce", {50.87, 20.63}},       {"Zielona Gora", {51.94, 15.51}},
    {"Opole", {50.67, 17.93}},        {"Torun", {53.02, 18.61}},        {"Gorzow Wlkp.", {52.73, 15.24}},
    {"Koszalin", {54.19, 16.18}},     {"Czestochowa", {50.81, 19.12}}};

} // namespace

std::vector<glm::dvec2> const &poland_outline()
{
	static std::vector<glm::dvec2> const outline = []() {
		std::vector<glm::dvec2> points;
		points.reserve(std::size(kOutline));
		for (auto const &point : kOutline)
		{
			points.push_back(puwg1992(point.latitude, point.longitude));
		}
		return points;
	}();

	return outline;
}

std::vector<map_place> const &poland_places()
{
	static std::vector<map_place> const places = []() {
		std::vector<map_place> result;
		result.reserve(std::size(kPlaces));
		for (auto const &place : kPlaces)
		{
			result.push_back({place.name, puwg1992(place.position.latitude, place.position.longitude)});
		}
		return result;
	}();

	return places;
}

} // namespace editor
