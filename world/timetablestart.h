/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <string_view>

// the first row of a timetable
namespace timetable {

// whether the first row of a timetable is the station the train sets off from: it names the station
// the relation starts at. such a row is a stop even with no arrival time given, the train waits there
// for its departure time. the names are compared regardless of case, as the timetables write the
// station of departure in capitals as often as not (ZAGORZ in the rows, Zagorz in the relation)
bool starts_at(std::string_view Firststation, std::string_view Relationfrom);

} // namespace timetable
