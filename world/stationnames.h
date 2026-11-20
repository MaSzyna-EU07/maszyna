/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <string>
#include <string_view>

// the names of the stations (MaSzyna-EU07/maszyna#94): a timetable and the W4 stop points of a scenery name a station
// each in its own file, in windows-1250, and they don't have to spell it the same way (with or without the polish
// letters, in another case). a name has three forms here:
// - as written, with its letters: what the files hold and what the player is shown (station_caption())
// - the ascii one, the polish letters without their marks: what the timetable always kept as the name of a station,
//   what the scenery events, the scripts and the file names of the announcements and destination boards go by
//   (station_ascii())
// - the key two spellings are the same station by (station_key(), same_station())
namespace timetable {

// what a W4 text starts with: "PassengerStopPoint:<station>". in the scenery a "#<part which makes the name of the
// event unique>" follows; the event cuts it as it's read, a memory cell keeps the whole of its text
inline constexpr std::string_view stop_point_prefix{"PassengerStopPoint:"};

// the name without the marks of the polish letters of windows-1250, the rest as written
std::string station_ascii(std::string Name);
// the key of a name: the ascii form in lower case, spaces as underscores
std::string station_key(std::string_view Name);
// whether the two names are spellings of the same station
bool same_station(std::string_view Left, std::string_view Right);
// whether the W4 text, as an event or a memory cell holds it, is for the station of given name, in any spelling of the two
bool stop_point_is_for(std::string_view Text, std::string_view Station);
// the name for the player: utf-8, with its letters, underscores as spaces
std::string station_caption(std::string const &Name);

} // namespace timetable
