/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <array>
#include <span>

namespace climate
{

// a station of the table built into the simulator (environment/climatestations.cpp, made by tools/weather/imgw_normals.py): its
// place and, month by month, the lowest and highest temperature of a day, the humidity, the wind, the precipitation and its days,
// the days with snow lying and its depth, the days with fog and with a storm, and the cloudiness, as climate::month has them
struct station_record
{
	char const *name;
	float latitude;
	float longitude;
	float elevation;
	std::array<std::array<float, 11>, 12> months;
};

std::span<station_record const> builtin_stations();

} // namespace climate
