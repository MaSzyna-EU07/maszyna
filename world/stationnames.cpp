/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "world/stationnames.h"

#include "utilities/utilities.h"

#include <algorithm>

namespace timetable {

std::string station_ascii(std::string Name) {
	win1250_to_ascii(Name);
	return Name;
}

std::string station_key(std::string_view const Name) {
	// the same steps the comparison of a W4 with the timetable always took: the marks off, then the case
	auto key{ToLower(station_ascii(std::string{Name}))};
	std::ranges::replace(key, ' ', '_');
	return key;
}

bool same_station(std::string_view const Left, std::string_view const Right) {
	return station_key(Left) == station_key(Right);
}

bool stop_point_is_for(std::string_view const Text, std::string_view const Station) {
	return Text.starts_with(stop_point_prefix) && same_station(Text.substr(stop_point_prefix.size()), Station);
}

std::string station_caption(std::string const &Name) {
	return data_name_to_utf8(Name, true);
}

} // namespace timetable
