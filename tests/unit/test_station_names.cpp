#include "stdafx.h"
#include "doctest.h"

#include "utilities/utilities.h"
#include "world/stationnames.h"
#include "world/timetablereader.h"

#include <sstream>
#include <string>
#include <vector>

// the names of the stations (MaSzyna-EU07/maszyna#94): the timetable keeps a name as its file spells it, with its polish
// letters, for showing; the W4 stop points of the scenery, spelled in their own files, still find their station whatever
// the spelling; the form the announcement files, the scenery events and the scripts get stays the ascii one.
// the texts are windows-1250, written as bytes: the build reads the sources in any code page

using timetable::same_station;
using timetable::station_ascii;
using timetable::station_caption;
using timetable::station_key;
using timetable::stop_point_is_for;

namespace {

// Biała_Podlaska, Łódź_Żabieniec, Częstochowa_Osobowa in windows-1250
std::string const biala{"Bia\xB3" "a_Podlaska"};
std::string const lodz{"\xA3\xF3" "d\x9F_\xAF" "abieniec"};
std::string const czestochowa{"Cz\xEAstochowa_Osobowa"};
// the 18 polish letters in windows-1250, as in test_win1250.cpp
std::string const letters1250{"\xA5\xC6\xCA\xA3\xD1\xD3\x8C\x8F\xAF\xB9\xE6\xEA\xB3\xF1\xF3\x9C\x9F\xBF"};

// a timetable with polish letters in the relation and in the names of the stations
std::string const timetable_text{
    " ___________________________________________________________________\n"
    "[ Rodzaj i numer pociagu                         | Os1234           ]\n"
    "[ Kategoria                                      | Os               ]\n"
    "[________________________________________________|__________________]\n"
    "[                                                | Bia\xB3" "a_Podlaska   ]\n"
    "[ Relacja pociagu                                | \xA3\xF3" "d\x9F_\xAF" "abieniec   ]\n"
    "[________________________________________________|__________________]\n"
    "[ Wymagany % ciezaru hamujacego                  |  60%             ]\n"
    "[________________________________________________|__________________]\n"
    "[ Seria i obciazenie lokomotywy                  |  EN57  120       ]\n"
    "[________________________________________________|__________________]\n"
    "[ 1.13  | 60  | Bia\xB3" "a_Podlaska                   1    |    |        ]\n"
    "[       |     | R2,H                             1  12.47  |        ]\n"
    "[       |     |----------------------------------1---------|--------]\n"
    "[ 5.06  |_____| Cz\xEAstochowa_Osobowa              1  12.54  | 7      ]\n"
    "[       | 40  | po                               1  12.55  |        ]\n"
    "[       |     |----------------------------------1---------|--------]\n"
    "[ 9.16  |     | \xA3\xF3" "d\x9F_\xAF" "abieniec                   2  13.01  | 6      ]\n"
    "[       |     | R3                               2    |    |        ]\n"
    "[_______|_____|__________________________________2_________|________]\n"};

Mtable::timetable_data read(std::string const &Text) {
	std::istringstream input{Text};
	Mtable::timetable_data data;
	data.TrainName = "file";
	REQUIRE(Mtable::read_timetable(input, data) == 0);
	return data;
}

// the text of a W4 as the driver gets it: a putvalues event cuts what follows # when the scenery is read
// (putvalues_event::deserialize_), a memory cell keeps the whole of its text
std::string text_seen(std::string Text, bool const Event) {
	if (auto const hash{Text.find('#')}; Event && hash != std::string::npos) {
		Text.erase(hash);
	}
	return Text;
}

// the comparison of a W4 text with the next stop of the timetable as TController::TableUpdateStopPoint() made it before
// the names kept their letters: the text of a putvalues event lost its polish letters too when the scenery was read;
// 32 is the sizeof( std::string ) the code had for the length
bool matched_before(std::string const &Text, bool const Event, std::string const &Nextstop) {
	auto text{text_seen(Text, Event)};
	if (Event) {
		win1250_to_ascii(text);
	}
	return ToLower(text).compare(19, 32, ToLower(Nextstop)) == 0;
}

bool matched_now(std::string const &Text, bool const Event, std::string const &Nextstop) {
	return stop_point_is_for(text_seen(Text, Event), Nextstop);
}

} // namespace

TEST_CASE("station names: the ascii form takes the marks off the polish letters and nothing else") {
	CHECK(station_ascii(biala) == "Biala_Podlaska");
	CHECK(station_ascii(lodz) == "Lodz_Zabieniec");
	CHECK(station_ascii(czestochowa) == "Czestochowa_Osobowa");
	CHECK(station_ascii(letters1250) == "ACELNOSZZacelnoszz");
	// a name without them stays, the case, the underscores and the spaces included
	CHECK(station_ascii("Warszawa_Zachodnia") == "Warszawa_Zachodnia");
	CHECK(station_ascii("Nowy Targ") == "Nowy Targ");
	// what win1250_to_ascii() always made of a name: letters of other languages stay as they are
	auto ostrava{std::string{"Ostrava_hl.n\xE1" "dra\x9E\xED"}};
	auto const expected{ostrava};
	CHECK(station_ascii(ostrava) == expected);
	win1250_to_ascii(ostrava);
	CHECK(ostrava == expected);
}

TEST_CASE("station names: the file names of a station come from the ascii form, as before") {
	// the announcement of a stop is <scenery>/<name>.ogg or sounds/sip/<name>.ogg, of the name the timetable entry has
	// as StationName; the files have no polish letters in their names
	auto const data{read(timetable_text)};
	CHECK(data.TimeTable[2].StationName + ".ogg" == "Czestochowa_Osobowa.ogg");
	CHECK(data.TimeTable[3].StationName + ".ogg" == "Lodz_Zabieniec.ogg");
	// the destination board is looked up by the last station of the relation, through Bezogonkow()
	CHECK(Bezogonkow(data.Relation2) == "Lodz_Zabieniec");
}

TEST_CASE("station names: a timetable with polish letters keeps them for showing") {
	auto const data{read(timetable_text)};
	REQUIRE(data.StationCount == 3);
	// as written
	CHECK(data.RelationLabel1 == biala);
	CHECK(data.RelationLabel2 == lodz);
	CHECK(data.TimeTable[1].StationLabel == biala);
	CHECK(data.TimeTable[2].StationLabel == czestochowa);
	CHECK(data.TimeTable[3].StationLabel == lodz);
	// what the player sees: utf-8, spaces for the underscores
	CHECK(station_caption(data.TimeTable[1].StationLabel) == "Bia\xC5\x82" "a Podlaska");
	CHECK(station_caption(data.TimeTable[2].StationLabel) == "Cz\xC4\x99stochowa Osobowa");
	CHECK(station_caption(data.TimeTable[3].StationLabel) == "\xC5\x81\xC3\xB3" "d\xC5\xBA \xC5\xBB" "abieniec");
	CHECK(station_caption(data.RelationLabel2) == "\xC5\x81\xC3\xB3" "d\xC5\xBA \xC5\xBB" "abieniec");
	// the rest of the row reads as it did
	CHECK(data.TimeTable[2].StationWare == "po");
	CHECK(data.TimeTable[2].Ah == 12);
	CHECK(data.TimeTable[3].TrackNo == 2);
}

TEST_CASE("station names: the name a timetable is matched and handed over by stays the ascii one") {
	auto const data{read(timetable_text)};
	// TMTableEntry::StationName and the relation: what the W4 comparison, the announcement files, the scenery events
	// (WhoIs) and the python screens get
	CHECK(data.Relation1 == "Biala_Podlaska");
	CHECK(data.Relation2 == "Lodz_Zabieniec");
	CHECK(data.TimeTable[1].StationName == "Biala_Podlaska");
	CHECK(data.TimeTable[2].StationName == "Czestochowa_Osobowa");
	CHECK(data.TimeTable[3].StationName == "Lodz_Zabieniec");
	// the first station is still told to be the start of the relation by the two names being the same
	CHECK(data.TimeTable[1].StationName == data.Relation1);
}

TEST_CASE("station names: a timetable without polish letters has the same name in both forms") {
	auto const plain{station_ascii(timetable_text)};
	auto const data{read(plain)};
	REQUIRE(data.StationCount == 3);
	CHECK(data.RelationLabel1 == data.Relation1);
	CHECK(data.RelationLabel2 == data.Relation2);
	for (auto i{1}; i <= data.StationCount; ++i) {
		CAPTURE(i);
		CHECK(data.TimeTable[i].StationLabel == data.TimeTable[i].StationName);
	}
	// and reads the same as the one with the letters, but for the labels
	auto const lettered{read(timetable_text)};
	for (auto i{0}; i <= data.StationCount; ++i) {
		CAPTURE(i);
		CHECK(data.TimeTable[i].StationName == lettered.TimeTable[i].StationName);
		CHECK(data.TimeTable[i].Dh == lettered.TimeTable[i].Dh);
		CHECK(data.TimeTable[i].Dm == lettered.TimeTable[i].Dm);
	}
	// the entry before the first station is the technical one in both forms
	CHECK(data.TimeTable[0].StationName == "nowhere");
	CHECK(data.TimeTable[0].StationLabel == "nowhere");
}

TEST_CASE("station names: two spellings of a station have one key") {
	// with and without the letters
	CHECK(same_station(biala, "Biala_Podlaska"));
	CHECK(same_station("Biala_Podlaska", biala));
	CHECK(same_station(lodz, "Lodz_Zabieniec"));
	// some of the letters only (linia 107 has the W4 of Rzepedz spelled so)
	CHECK(same_station("Rzep\xEA" "d\x9F", "Rzeped\x9F"));
	// in any case, the letters with marks included (baltyk has BALTYK_GLOWNY in capitals in a timetable)
	CHECK(same_station("BA\xA3TYK_G\xA3\xD3WNY", "Ba\xB3tyk_G\xB3\xF3wny"));
	CHECK(same_station("BA\xA3TYK_G\xA3\xD3WNY", "baltyk_glowny"));
	CHECK(same_station("ZAG\xD3RZ", "Zag\xF3rz"));
	// with spaces or underscores
	CHECK(same_station("Bia\xB3" "a Podlaska", biala));
	CHECK(same_station("Biala Podlaska", biala));
	CHECK(same_station("Biala Podlaska", "biala_podlaska"));
	// the key itself
	CHECK(station_key(biala) == "biala_podlaska");
	CHECK(station_key("Bia\xB3" "a Podlaska") == "biala_podlaska");
	CHECK(station_key(letters1250) == "acelnoszzacelnoszz");
	CHECK(station_key("").empty());
}

TEST_CASE("station names: different stations have different keys") {
	CHECK_FALSE(same_station(biala, "Biala"));
	CHECK_FALSE(same_station("Biala_Podlaska", "Biala_Podlaska_po"));
	CHECK_FALSE(same_station("Lodz_Zabieniec", "Lodz_Kaliska"));
	CHECK_FALSE(same_station("Gorna", "Dolna"));
	CHECK_FALSE(same_station("Gorna", ""));
	// no other separator stands for a space
	CHECK_FALSE(same_station("Biala-Podlaska", "Biala_Podlaska"));
	CHECK_FALSE(same_station("BialaPodlaska", "Biala_Podlaska"));
	// letters of other languages aren't folded, as they never were
	CHECK_FALSE(same_station("Ostrava_hl.n\xE1" "dra\x9E\xED", "Ostrava_hl.nadrazi"));
	CHECK(same_station("Ostrava_hl.n\xE1" "dra\x9E\xED", "ostrava_hl.n\xE1" "dra\x9E\xED"));
}

TEST_CASE("station names: a W4 finds the station of the timetable whatever the two spellings") {
	auto const data{read(timetable_text)};
	// the next stop of the driver is the StationName of the entry
	auto const &nextstop{data.TimeTable[1].StationName};
	// the W4 spelled with the letters, without them, in another case, with a space
	CHECK(stop_point_is_for("PassengerStopPoint:" + biala, nextstop));
	CHECK(stop_point_is_for("PassengerStopPoint:Biala_Podlaska", nextstop));
	CHECK(stop_point_is_for("PassengerStopPoint:BIA\xA3" "A_PODLASKA", nextstop));
	CHECK(stop_point_is_for("PassengerStopPoint:biala_podlaska", nextstop));
	CHECK(stop_point_is_for("PassengerStopPoint:Bia\xB3" "a Podlaska", nextstop));
	CHECK(stop_point_is_for("PassengerStopPoint:Biala Podlaska", nextstop));
	// the usual W4 of a scenery, with the part which makes its event unique: the event cuts it as it's read
	CHECK(matched_now("PassengerStopPoint:" + biala + "#W4-P1-T2-1", true, nextstop));
	CHECK(matched_now("PassengerStopPoint:Biala_Podlaska#W4-1", true, nextstop));
	// and the other way round: the timetable name as written against a W4 without the letters
	CHECK(stop_point_is_for("PassengerStopPoint:Biala_Podlaska", data.TimeTable[1].StationLabel));
	CHECK(stop_point_is_for("PassengerStopPoint:Lodz_Zabieniec", data.TimeTable[3].StationLabel));
	// another station, a text which isn't a W4 one
	CHECK_FALSE(stop_point_is_for("PassengerStopPoint:Biala", nextstop));
	CHECK_FALSE(stop_point_is_for("PassengerStopPoint:Biala_Podlaska_po", nextstop));
	CHECK_FALSE(stop_point_is_for("PassengerStopPoint:" + lodz, nextstop));
	CHECK_FALSE(stop_point_is_for("Biala_Podlaska", nextstop));
	CHECK_FALSE(stop_point_is_for("passengerstoppoint:Biala_Podlaska", nextstop));
	CHECK_FALSE(stop_point_is_for("SetVelocity", nextstop));
	CHECK_FALSE(stop_point_is_for("", nextstop));
	// a memory cell keeps what follows the #: as before, it is a W4 only of a station which has that in its name
	CHECK_FALSE(matched_now("PassengerStopPoint:Biala_Podlaska#W4-1", false, nextstop));
	CHECK(matched_now("PassengerStopPoint:Gorna#1", false, "Gorna#1"));
	CHECK_FALSE(matched_now("PassengerStopPoint:Gorna", false, "Gorna#1"));
	// a W4 without a name is for the end of the timetable, where the next stop has none either
	CHECK(stop_point_is_for("PassengerStopPoint:", ""));
	CHECK_FALSE(stop_point_is_for("PassengerStopPoint:", nextstop));
	CHECK_FALSE(stop_point_is_for("PassengerStopPoint:Biala_Podlaska", ""));
}

TEST_CASE("station names: every W4 the driver matched before is matched now") {
	// the next stops as the timetable gives them (ascii), and the special ones the driver has when there's no station
	std::vector<std::string> const nextstops{"Biala_Podlaska", "Lodz_Zabieniec", "Czestochowa_Osobowa", "BALTYK_GLOWNY", "Gorna", "Gorna#1", "Rzepedz", "", "nowhere", "[End of route]",
	                                         "Ostrava_hl.n\xE1" "dra\x9E\xED"};
	std::vector<std::string> const names{biala, "Biala_Podlaska", "biala_podlaska", "BIA\xA3" "A_PODLASKA", "Bia\xB3" "a Podlaska", lodz, "Lodz_Zabieniec", czestochowa,
	                                     "Ba\xB3tyk_G\xB3\xF3wny", "Rzeped\x9F", "Rzep\xEA" "d\x9F", "Gorna", "Gorna#1", "Dolna", "", "nowhere", "Ostrava_hl.n\xE1" "dra\x9E\xED",
	                                     "Ostrava_hl.nadrazi", "Biala", "Biala_Podlaska_po"};
	auto matches{0};
	auto gained{0};
	for (auto const &nextstop : nextstops) {
		for (auto const &name : names) {
			for (auto const *suffix : {"", "#W4-1"}) {
				auto const text{"PassengerStopPoint:" + name + suffix};
				for (auto const event : {true, false}) {
					CAPTURE(nextstop);
					CAPTURE(text);
					CAPTURE(event);
					auto const before{matched_before(text, event, nextstop)};
					auto const now{matched_now(text, event, nextstop)};
					// nothing is lost
					CHECK((false == before || now));
					// and nothing changes for the spellings the comparison never told apart: a name without polish letters
					// and without spaces
					if (name == station_ascii(name) && name.find(' ') == std::string::npos) {
						CHECK(before == now);
					}
					matches += (now ? 1 : 0);
					gained += (now && false == before ? 1 : 0);
				}
			}
		}
	}
	CHECK(matches > 0);
	// what is new: a memory cell spelled with the polish letters, a space for an underscore
	CHECK(gained > 0);
	CHECK(matched_now("PassengerStopPoint:" + biala, false, "Biala_Podlaska"));
	CHECK_FALSE(matched_before("PassengerStopPoint:" + biala, false, "Biala_Podlaska"));
	CHECK(matched_now("PassengerStopPoint:Biala Podlaska", true, "Biala_Podlaska"));
	CHECK_FALSE(matched_before("PassengerStopPoint:Biala Podlaska", true, "Biala_Podlaska"));
	// and a name longer than the 32 characters the old comparison took of it
	std::string const longname{"Bardzo_Dluga_Nazwa_Stacji_Wezlowej_Osobowej"};
	CHECK(matched_now("PassengerStopPoint:" + longname, true, longname));
	CHECK_FALSE(matched_before("PassengerStopPoint:" + longname, true, longname));
}

TEST_CASE("station names: the caption of a station is its name as written, in utf-8, with spaces") {
	CHECK(station_caption(biala) == "Bia\xC5\x82" "a Podlaska");
	CHECK(station_caption(lodz) == "\xC5\x81\xC3\xB3" "d\xC5\xBA \xC5\xBB" "abieniec");
	CHECK(station_caption("Warszawa_Zachodnia") == "Warszawa Zachodnia");
	CHECK(station_caption("[End of route]") == "[End of route]");
	CHECK(station_caption("").empty());
	// a name already in utf-8 stays
	CHECK(station_caption("Bia\xC5\x82" "a_Podlaska") == "Bia\xC5\x82" "a Podlaska");
}
