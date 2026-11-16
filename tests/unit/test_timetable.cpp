#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"

#include "McZapkie/MOVER.h"
#include "utilities/utilities.h"
#include "vehicle/timetableservice.h"
#include "world/timetablereader.h"

#include <algorithm>
#include <filesystem>
#include <format>
#include <fstream>
#include <sstream>

// the timetables of the trains (<train>.txt next to the scenery, world/timetablereader.cpp) and the service flags they
// give the AI driver (eu07/maszyna#95): the reader as it was, the flags of the header and of the stations, and what
// the driver does with them

using timetable::service_flags;
using timetable::tristate;

namespace {

// a timetable the way the game's sceneries have them: the header, three stations, a speed change at the second one
std::string const timetable_text{
    " ___________________________________________________________________\n"
    "[ Rodzaj i numer pociagu                         | Os1234           ]\n"
    "[ Kategoria                                      | Os               ]\n"
    "%FLAGS%"
    "[________________________________________________|__________________]\n"
    "[                                                | Gorna            ]\n"
    "[ Relacja pociagu                                | Dolna            ]\n"
    "[________________________________________________|__________________]\n"
    "[ Wymagany % ciezaru hamujacego                  |  60%             ]\n"
    "[________________________________________________|__________________]\n"
    "[ Seria i obciazenie lokomotywy                  |  EN57  120       ]\n"
    "[________________________________________________|__________________]\n"
    "[ 1.13  | 60  | Gorna                            1    |    |        ]\n"
    "[       |     | %WARE1%                          1  12.47  |        ]\n"
    "[       |     |----------------------------------1---------|--------]\n"
    "[ 5.06  |_____| Srodkowa                         1  12.54  | 7      ]\n"
    "[       | 40  | %WARE2%                          1  12.55  |        ]\n"
    "[       |     |----------------------------------1---------|--------]\n"
    "[ 9.16  |     | Dolna                            2  13.01  | 6      ]\n"
    "[       |     | %WARE3%                          2    |    |        ]\n"
    "[_______|_____|__________________________________2_________|________]\n"};

std::string replaced(std::string Text, std::string const &Key, std::string const &Value) {
	for (auto at{Text.find(Key)}; at != std::string::npos; at = Text.find(Key, at + Value.size())) {
		Text.replace(at, Key.size(), Value);
	}
	return Text;
}

std::string timetable_with(std::string const &Flags, std::string const &Ware1, std::string const &Ware2, std::string const &Ware3) {
	return replaced(replaced(replaced(replaced(timetable_text, "%FLAGS%", Flags), "%WARE1%", Ware1), "%WARE2%", Ware2), "%WARE3%", Ware3);
}

struct read_result {
	int error;
	Mtable::timetable_data data;
};

read_result read(std::string const &Text) {
	std::istringstream input{Text};
	read_result result;
	result.data.TrainName = "file";
	result.error = Mtable::read_timetable(input, result.data);
	return result;
}

service_flags flags(tristate const Doors, tristate const Doorstep, tristate const Conductor, tristate const Braketest) {
	return {Doors, Doorstep, Conductor, Braketest};
}

auto const unset{tristate::unset};
auto const yes{tristate::yes};
auto const no{tristate::no};

// what the timetable gives, but for the service flags: to compare a timetable with flags to one without
std::string contents(Mtable::timetable_data const &Data) {
	auto text{std::format("{}|{}|{}|{}|{}|{}|{}|{}|{}\n", Data.TrainName, Data.TrainCategory, Data.TrainLabel, Data.Relation1, Data.Relation2, Data.BrakeRatio,
	                      Data.LocSeries, Data.LocLoad, Data.StationCount)};
	for (auto i{0}; i <= Data.StationCount; ++i) {
		auto const &row{Data.TimeTable[i]};
		text += std::format("{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}|{}\n", i, row.km, row.vmax, row.StationName, row.StationWare, row.TrackNo, row.Ah, row.Am, row.Dh, row.Dm,
		                    row.tm, row.is_maintenance, row.radio_channel);
	}
	return text;
}

std::string flags_text(service_flags const &Flags) {
	auto const one{[](tristate const Flag) { return Flag == tristate::unset ? '-' : Flag == tristate::yes ? 'y' : 'n'; }};
	return {one(Flags.central_doors), one(Flags.doorstep), one(Flags.conductor), one(Flags.brake_test)};
}

} // namespace

TEST_CASE("timetable: the reader reads a timetable the way it always did") {
	auto const [error, data]{read(timetable_with("", "R2,H", "po", "R3"))};

	CHECK(error == 0);
	CHECK(data.TrainName == "Os1234");
	CHECK(data.TrainCategory == "Os");
	CHECK(data.Relation1 == "Gorna");
	CHECK(data.Relation2 == "Dolna");
	CHECK(data.BrakeRatio == 60.0);
	CHECK(data.LocSeries == "EN57");
	CHECK(data.LocLoad == 120.0);
	REQUIRE(data.StationCount == 3);

	auto const &first{data.TimeTable[1]};
	CHECK(first.StationName == "Gorna");
	CHECK(first.km == doctest::Approx(1.13));
	CHECK(first.Ah == -1); // no arrival: LoadTTfile makes it a stop, the first station being the start of the relation
	CHECK(first.Dh == 12);
	CHECK(first.Dm == 47.f);
	CHECK(first.StationWare == "R2,H");
	CHECK(first.radio_channel == 2);

	auto const &second{data.TimeTable[2]};
	CHECK(second.StationName == "Srodkowa");
	CHECK(second.Ah == 12);
	CHECK(second.Am == 54.f);
	CHECK(second.Dm == 55.f);
	CHECK(second.tm == 7.f);
	CHECK(second.StationWare == "po");
	CHECK(second.radio_channel == -1);

	auto const &last{data.TimeTable[3]};
	CHECK(last.StationName == "Dolna");
	CHECK(last.TrackNo == 2);
	CHECK(last.Ah == 13);
	CHECK(last.Dh == 13); // departure as the arrival, for the delay
	CHECK(last.radio_channel == 3);
	// 60 km/h up to the change at the second station, 40 km/h after it
	CHECK(data.TimeTable[1].vmax == 60.f);
	CHECK(data.TimeTable[2].vmax == 60.f);
	CHECK(data.TimeTable[3].vmax == 40.f);
	// a timetable without flags leaves all of them to the vehicles
	CHECK(data.Flags.empty());
	for (auto i{0}; i <= data.StationCount; ++i) {
		CHECK(data.TimeTable[i].flags.empty());
	}
}

TEST_CASE("timetable: the row Flagi of the header sets the flags of the whole train") {
	auto const [error, data]{read(timetable_with("[ Flagi AI                                      | kierownik=nie, proba_hamulca=nie ]\n", "R2,H", "po", "R3"))};

	CHECK(error == 0);
	CHECK(data.Flags == flags(unset, unset, no, no));
	// every station has them
	for (auto i{0}; i <= data.StationCount; ++i) {
		CAPTURE(i);
		CHECK(data.TimeTable[i].flags == data.Flags);
	}
	// and the rest of the header reads as before
	CHECK(data.TrainName == "Os1234");
	CHECK(data.TrainCategory == "Os");
	CHECK(data.Relation1 == "Gorna");
	CHECK(data.StationCount == 3);
}

TEST_CASE("timetable: a station's flags hold from that station on") {
	auto const [error, data]{read(timetable_with("[ Flagi AI | proba_hamulca=nie ]\n", "R2,H", "po,stopien=nie,drzwi_centralne=tak", "R3,stopien=tak"))};

	CHECK(error == 0);
	CHECK(data.TimeTable[1].flags == flags(unset, unset, unset, no));
	CHECK(data.TimeTable[2].flags == flags(yes, no, unset, no));
	CHECK(data.TimeTable[3].flags == flags(yes, yes, unset, no));
	// the flags are taken out of the station ware, the rest of it stays
	CHECK(data.TimeTable[2].StationWare == "po");
	CHECK(data.TimeTable[3].StationWare == "R3");
	CHECK(data.TimeTable[3].radio_channel == 3);
}

TEST_CASE("timetable: the flags change nothing else the timetable gives") {
	auto const plain{read(timetable_with("", "R2,H", "po", "R3"))};
	auto const flagged{read(timetable_with("[ Flagi AI | kierownik=nie ]\n", "kierownik=tak,R2,H", "po,drzwi_centralne=tak", "R3,stopien=nie,proba_hamulca=nie"))};

	CHECK(flagged.error == plain.error);
	CHECK(contents(flagged.data) == contents(plain.data));
}

TEST_CASE("timetable: entries which aren't flags stay in the station ware") {
	service_flags read_flags;
	SUBCASE("no flags: the list as it was, empty entries too") {
		CHECK(timetable::extract_flags("po,,R2", read_flags) == "po,,R2");
		CHECK(timetable::extract_flags("", read_flags).empty());
		CHECK(read_flags.empty());
	}
	SUBCASE("an unknown key stays, a known one with an unknown value is dropped") {
		CHECK(timetable::extract_flags("po,foo=1,kierownik=moze,R2", read_flags) == "po,foo=1,R2");
		CHECK(read_flags.empty());
	}
	SUBCASE("keys and values in either case, 1 and 0 for tak and nie") {
		CHECK(timetable::extract_flags("Kierownik=NIE,STOPIEN=1,ph", read_flags) == "ph");
		CHECK(read_flags == flags(unset, yes, no, unset));
	}
}

TEST_CASE("timetable: a truncated timetable is an error, not a hang") {
	auto const text{timetable_with("", "R2,H", "po", "R3")};
	// cut in the middle of a station, in the middle of the header, right after the top line: the reader used to go
	// on reading past the end of the file forever
	for (auto const cut : {text.find("Srodkowa"), text.find("Wymagany"), text.find("Rodzaj")}) {
		CAPTURE(cut);
		auto const [error, data]{read(text.substr(0, cut))};
		CHECK(error == Mtable::timetable_truncated);
	}
	// cut between two stations: the row the reader expects isn't there
	auto const [error, data]{read(text.substr(0, text.find("[ 9.16")))};
	CHECK(error == -4);
	CHECK(data.StationCount == 3);
}

TEST_CASE("timetable: a timetable longer than the table is an error") {
	auto const text{timetable_with("", "R2,H", "po", "R3")};
	auto const begin{text.find("[ 5.06")};
	auto const end{text.find("[ 9.16")};
	auto const station{text.substr(begin, end - begin)};
	std::string stations;
	for (auto i{0}; i < Mtable::MaxTTableSize; ++i) {
		stations += station;
	}
	auto const [error, data]{read(text.substr(0, begin) + stations + text.substr(end))};
	CHECK(error == Mtable::timetable_too_long);
	CHECK(data.StationCount == Mtable::MaxTTableSize);
}

TEST_CASE("timetable: the entry whose flags hold at the stop and on the way") {
	// before the timetable is loaded, and with none
	CHECK(timetable::stop_flags_entry(0, 0) == 0);
	CHECK(timetable::run_flags_entry(0, 0) == 0);
	CHECK(timetable::run_flags_entry(1, 0) == 0);
	// making for (or standing at) the first station: its flags at the stop, the header's on the way
	CHECK(timetable::stop_flags_entry(1, 3) == 1);
	CHECK(timetable::run_flags_entry(1, 3) == 0);
	// after the departure from the second station
	CHECK(timetable::stop_flags_entry(3, 3) == 3);
	CHECK(timetable::run_flags_entry(3, 3) == 2);
	// past the end of the timetable
	CHECK(timetable::stop_flags_entry(4, 3) == 3);
	CHECK(timetable::run_flags_entry(5, 3) == 3);
}

TEST_CASE("timetable: the AI opens the doors from the cab as the vehicle and the timetable say") {
	test::vehicle unit{test::definition::load("wagon")};
	auto const central{flags(yes, unset, unset, unset)};
	auto const notcentral{flags(no, unset, unset, unset)};

	unit->Doors.open_control = control_t::driver;
	CHECK(timetable::opens_doors(unit.mover(), {}));
	CHECK(timetable::opens_doors(unit.mover(), notcentral)); // the driver's doors open only from the cab
	unit->Doors.open_control = control_t::conductor;
	CHECK(timetable::opens_doors(unit.mover(), {}));
	// doors of mixed control are left to the passengers, unless the timetable has the driver open them all
	unit->Doors.open_control = control_t::mixed;
	CHECK_FALSE(timetable::opens_doors(unit.mover(), {}));
	CHECK_FALSE(timetable::opens_doors(unit.mover(), notcentral));
	CHECK(timetable::opens_doors(unit.mover(), central));
	// the passengers' own doors and the automatic ones can't be opened from the cab
	unit->Doors.open_control = control_t::passenger;
	CHECK_FALSE(timetable::opens_doors(unit.mover(), central));
	unit->Doors.open_control = control_t::autonomous;
	CHECK_FALSE(timetable::opens_doors(unit.mover(), central));
}

TEST_CASE("timetable: the doorsteps, the conductor and the brake test as the timetable says") {
	// no flags: the doorsteps as they are, the conductor and the brake test as before
	CHECK_FALSE(timetable::doorstep({}).has_value());
	CHECK(timetable::has_conductor({}));
	CHECK(timetable::makes_brake_test({}));

	CHECK(timetable::doorstep(flags(unset, no, unset, unset)) == false);
	CHECK(timetable::doorstep(flags(unset, yes, unset, unset)) == true);
	CHECK_FALSE(timetable::has_conductor(flags(unset, unset, no, unset)));
	CHECK(timetable::has_conductor(flags(unset, unset, yes, unset)));
	CHECK_FALSE(timetable::makes_brake_test(flags(unset, unset, unset, no)));
	CHECK(timetable::makes_brake_test(flags(unset, unset, unset, yes)));
}

// reads every .txt file under the folder given in EU07_UNIT_TESTS_TIMETABLES (the scenery/ of the game), the way the
// game reads a timetable, and checks none of them has flags; EU07_UNIT_TESTS_TIMETABLES_DUMP names a file to write
// what was read to, to compare the reading of two builds
TEST_CASE("timetable: the timetables of the game, when asked for") {
	auto const *source{std::getenv("EU07_UNIT_TESTS_TIMETABLES")};
	if (source == nullptr) {
		return;
	}
	std::vector<std::filesystem::path> files;
	for (auto const &entry : std::filesystem::recursive_directory_iterator{source}) {
		if (entry.is_regular_file() && ToLower(entry.path().extension().string()) == ".txt") {
			files.push_back(entry.path());
		}
	}
	std::ranges::sort(files);
	std::string dump;
	auto timetables{0};
	for (auto const &file : files) {
		auto const name{std::filesystem::relative(file, source).generic_string()};
		CAPTURE(name);
		std::ifstream input{file};
		Mtable::timetable_data data;
		data.TrainName = file.stem().string();
		auto const error{Mtable::read_timetable(input, data)};
		if (error == 0 && data.StationCount > 0) {
			++timetables;
		}
		CHECK(data.Flags.empty());
		for (auto i{0}; i <= data.StationCount; ++i) {
			CHECK(data.TimeTable[i].flags.empty());
		}
		dump += std::format("== {} error {}\n", name, error) + contents(data);
	}
	MESSAGE(std::format("{} files, {} read as timetables", files.size(), timetables));
	if (auto const *output{std::getenv("EU07_UNIT_TESTS_TIMETABLES_DUMP")}; output != nullptr) {
		std::ofstream{output, std::ios::binary} << dump;
	}
}
