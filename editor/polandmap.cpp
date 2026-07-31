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

// the country's corners, wide enough to hold all of it: the Odra in the west, the Bug in the east,
// the Tatras in the south and the coast in the north
constexpr geo_point kSouthWest{49.00, 14.10};
constexpr geo_point kNorthEast{54.90, 24.20};

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

void poland_extent(glm::dvec2 &Min, glm::dvec2 &Max)
{
	// the projection is not axis-aligned with the graticule, so the extreme corners come from
	// projecting all four of them and taking the outermost values
	auto const sw = puwg1992(kSouthWest.latitude, kSouthWest.longitude);
	auto const se = puwg1992(kSouthWest.latitude, kNorthEast.longitude);
	auto const nw = puwg1992(kNorthEast.latitude, kSouthWest.longitude);
	auto const ne = puwg1992(kNorthEast.latitude, kNorthEast.longitude);

	Min = {std::min({sw.x, se.x, nw.x, ne.x}), std::min({sw.y, se.y, nw.y, ne.y})};
	Max = {std::max({sw.x, se.x, nw.x, ne.x}), std::max({sw.y, se.y, nw.y, ne.y})};
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
