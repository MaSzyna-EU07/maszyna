/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "world/timetablestart.h"

#include <algorithm>
#include <cctype>

namespace timetable {

bool starts_at(std::string_view const Firststation, std::string_view const Relationfrom) {
	// the station names are plain ascii by now, the timetable loader strips the polish letters
	return std::ranges::equal(Firststation, Relationfrom, [](char const Left, char const Right) {
		return std::tolower(static_cast<unsigned char>(Left)) == std::tolower(static_cast<unsigned char>(Right));
	});
}

} // namespace timetable
