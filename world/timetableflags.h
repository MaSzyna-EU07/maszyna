/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstdint>
#include <string>
#include <string_view>

// the way a train is served, set by its timetable for the AI driver (eu07/maszyna#95): trams and underground trains
// open all the doors from the cab, have no doorstep, no conductor, no running brake test. each flag the timetable
// leaves out keeps the behaviour of the vehicles, as before the flags.
//
// the whole train: a row of the first box of the header
//   [ Flagi AI | kierownik=nie,proba_hamulca=nie ]
// from a station on: entries of its station ware (the second line of the station, with R2, ph, @...)
//   [       |     | R2,ph,drzwi_centralne=tak,stopien=nie   1  12.47  |        ]
// a station's entry holds from that station on, until another station changes it
namespace timetable {

enum class tristate : std::uint8_t {
	unset, // left to the vehicles, as before the flags
	yes,
	no
};

struct service_flags {
	tristate central_doors{tristate::unset}; // drzwi_centralne: the driver opens all the doors, of mixed control too
	tristate doorstep{tristate::unset}; // stopien: the doorsteps on (tak) or off (nie) before the doors open
	tristate conductor{tristate::unset}; // kierownik: no conductor (nie), no departure signal, no door kept open for him
	tristate brake_test{tristate::unset}; // proba_hamulca: no running brake test (nie)

	bool operator==(service_flags const &) const = default;
	bool empty() const { return *this == service_flags{}; }
	// the flags Other sets explicitly override these
	void apply(service_flags const &Other);
};

// reads one entry, key=value (tak/nie, 1/0), into Flags. returns false for an entry which isn't a flag, so the caller
// keeps it; a flag with a value it doesn't know is reported to the log and taken as handled
bool read_flag(std::string_view Entry, service_flags &Flags);

// reads the flags out of a station ware list (comma separated entries) into Flags. returns the list without them: the
// list as it was when it had none
std::string extract_flags(std::string const &Ware, service_flags &Flags);

// the timetable entry whose flags hold at the stop the train makes for or stands at: the current one (StationIndex),
// within 0 (the header's flags) and the last station
int stop_flags_entry(int StationIndex, int StationCount);
// the entry whose flags hold on the way: the station the train left last, 0 (the header's flags) before the first one
int run_flags_entry(int StationIndex, int StationCount);

} // namespace timetable
