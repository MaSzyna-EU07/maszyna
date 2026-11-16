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
#include <istream>
#include <string>

#include "world/timetableflags.h"

// the reader of the timetable files (<train>.txt next to the scenery), without the parts of the engine the timetable
// of a running train needs (sounds, scenario time), so it can be tested on its own
namespace Mtable {

static int const MaxTTableSize = 100; // można by to robić dynamicznie

// one station of a timetable, as the file gives it
struct TMTableEntry {
	float km{0.f}; // kilometraz linii
	float vmax{-1.f}; // predkosc rozkladowa przed przystankiem
	std::string StationName{"nowhere"}; // nazwa stacji ('_' zamiast spacji)
	std::string StationWare; // typ i wyposazenie stacji, oddz. przecinkami, bez flag obsługi
	int TrackNo{1}; // ilosc torow szlakowych
	int Ah{-1};
	float Am{-1.f}; // godz. i min. przyjazdu, -1 gdy bez postoju
	int Dh{-1};
	float Dm{-1.f}; // godz. i min. odjazdu
	float tm{0.f}; // czas jazdy do tej stacji w min. (z kolumny)
	bool is_maintenance{false};
	int radio_channel{-1};
	// the service flags which hold from this station on: the header's, changed by this and the earlier stations.
	// entry 0 holds the header's
	timetable::service_flags flags;
};

// what a timetable file holds
struct timetable_data {
	std::string TrainName; // the name the file is looked up by, then the one the file gives
	std::string TrainCategory;
	std::string TrainLabel;
	std::string Relation1;
	std::string Relation2;
	double BrakeRatio{0.0};
	std::string LocSeries;
	double LocLoad{0.0};
	timetable::service_flags Flags; // of the whole train, the row "Flagi" of the header
	int StationCount{0}; // stations 1..StationCount, entry 0 is the technical one
	std::array<TMTableEntry, MaxTTableSize + 1> TimeTable;
};

// conversion errors of read_timetable, besides the ones it always had (-4: no station row where expected, -5: no
// first station of the relation)
int const timetable_truncated{-9}; // the file ends before the bottom line of the table (the reader used to hang)
int const timetable_too_long{-10}; // more stations than MaxTTableSize (the reader used to write past the table)

// reads the timetable from Input into Data. returns the conversion error, 0 when read
int read_timetable(std::istream &Input, timetable_data &Data);

} // namespace Mtable
