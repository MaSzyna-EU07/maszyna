#include "stdafx.h"
#include "doctest.h"

#include "world/timetablestart.h"

// the first row of a timetable is the station of departure, a stop the train waits at for its departure time,
// when it names the station the relation starts at. the loader then takes the departure time for the arrival
// one, which the row usually leaves out; a row without an arrival time is otherwise a station passed through

TEST_CASE("timetable: the first row naming the station of the relation is the station of departure") {
	CHECK(timetable::starts_at("Stacja1", "Stacja1"));
	CHECK(timetable::starts_at("Zagorz", "Zagorz"));
}

TEST_CASE("timetable: the station of departure written in capitals in the rows is still the one of the relation") {
	// linia_107_poludnie.scn, Os33733: ZAGORZ in the rows, Zagorz in the relation (both with the polish letters
	// stripped by the loader). taken for a station passed through, the AI left before its departure time and the
	// train manager never gave the departure signal
	CHECK(timetable::starts_at("ZAGORZ", "Zagorz"));
	CHECK(timetable::starts_at("LEOSIN", "Leosin"));
	CHECK(timetable::starts_at("Warszawa_Wschodnia", "WARSZAWA_WSCHODNIA"));
}

TEST_CASE("timetable: a first row naming another station is not the station of departure") {
	// a timetable which starts on the way, at a station the train passes through
	CHECK_FALSE(timetable::starts_at("Tarnawa_Dolna", "Zagorz"));
	CHECK_FALSE(timetable::starts_at("Nowy_Zagorz", "Zagorz"));
	CHECK_FALSE(timetable::starts_at("Zagorz", "Nowy_Zagorz"));
	// a name which only begins like the other one
	CHECK_FALSE(timetable::starts_at("Zagorz_Towarowy", "Zagorz"));
	CHECK_FALSE(timetable::starts_at("Zagorz", "Zagorz_Towarowy"));
}

TEST_CASE("timetable: the station names are compared as a whole, letters and the rest") {
	CHECK_FALSE(timetable::starts_at("Stacja1", "Stacja2"));
	CHECK_FALSE(timetable::starts_at("Stacja_1", "Stacja-1"));
	CHECK_FALSE(timetable::starts_at("Stacja1", ""));
	CHECK_FALSE(timetable::starts_at("", "Stacja1"));
}
