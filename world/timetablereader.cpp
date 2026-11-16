/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "world/timetablereader.h"

#include "utilities/Logs.h"
#include "utilities/utilities.h"

namespace Mtable {

namespace {

char const *const hrsd = ".";

// the rest of the row "Flagi" of the header after its label: flag entries up to the closing bracket. returns true when
// a row with no closing bracket ran into the line under the box
bool read_header_flags(std::istream &fin, timetable_data &Data) {
	std::string s;
	do {
		fin >> s;
	} while (!(s == "|" || fin.fail()));
	while (fin >> s) {
		if (s.starts_with('[')) {
			// the next row, or the line under the box
			ErrorLog("Bad timetable: row \"Flagi\" of " + Data.TrainName + " without the closing bracket");
			return contains(s, "_______|");
		}
		auto const closing{s.ends_with(']')};
		for (auto const &entry : Split(closing ? std::string_view{s}.substr(0, s.size() - 1) : std::string_view{s}, ',')) {
			if (false == entry.empty() && false == timetable::read_flag(entry, Data.Flags)) {
				ErrorLog("Bad timetable: unknown flag \"" + entry + "\" in row \"Flagi\" of " + Data.TrainName);
			}
		}
		if (closing) {
			break;
		}
	}
	return false;
}

} // namespace

// the reader as TTrainParameters::LoadTTfile had it, but for: the row "Flagi" of the header and the flags of the
// station ware; the loops end with the end of the file, where they used to spin forever (on a truncated file, or one
// with a number where the reader expects it which isn't one). a file the old reader read gets read the same
int read_timetable(std::istream &fin, timetable_data &Data) {
	std::string lines;
	std::string s;
	bool EndTable{false};
	double vActual{-1};
	int conversionerror{0};
	auto &TimeTable{Data.TimeTable};
	auto &StationCount{Data.StationCount};
	// zapisywanie prędkości maksymalnej do wcześniejszych odcinków
	auto const UpdateVelocity{[&TimeTable, &StationCount](double const Velocity) {
		int i = StationCount;
		while (i >= 0 && TimeTable[i].vmax == -1) {
			TimeTable[i].vmax = static_cast<float>(Velocity); // prędkość dojazdu do przystanku i
			--i; // ewentualnie do poprzedniego też
		}
	}};

	while (fin.good() && !(conversionerror != 0 || EndTable)) {
		std::getline(fin, lines); /*wczytanie linii*/
		if (contains(lines, "___________________")) /*linia pozioma górna*/
		{
			fin >> s;
			if (s == "[") /*lewy pion*/
			{
				fin >> s;
				if (s == "Rodzaj") /*"Rodzaj i numer pociagu"*/
					do {
						fin >> s;
					} while (!(s == "|" || fin.fail())); /*środkowy pion*/
			}
		}
		else if (lines == "") {
			break;
		}
		fin >> s; /*nazwa pociągu*/
		Data.TrainName = s; // nadanie nazwy z pliku TXT (bez ścieżki do pliku)
		while (fin >> s) {
			if (contains(s, "_______|")) {
				break;
			}
			if (s == "Kategoria") {
				do {
					fin >> s;
				} while (!(s == "|" || fin.fail()));
				fin >> Data.TrainCategory;
				continue;
			}
			if (s == "Nazwa") {
				do {
					fin >> s;
				} while (!(s == "|" || fin.fail()));
				fin >> Data.TrainLabel;
				continue;
			}
			if (s == "Flagi") {
				// service flags of the whole train (eu07/maszyna#95)
				if (read_header_flags(fin, Data)) {
					break;
				}
				continue;
			}
		}
		{ /*czytaj naglowek*/
			while (fin >> s) {
				if (s == "[")
					break;
			} /*pierwsza linia z relacją*/
			while (fin >> s) {
				if (s != "|")
					break;
			}
			if (s != "|") {
				Data.Relation1 = s;
				win1250_to_ascii(Data.Relation1);
			}
			else
				conversionerror = -5;
			while (fin >> s) {
				if (s == "Relacja")
					break;
			} /*druga linia z relacją*/
			while (fin >> s) {
				if (s == "|")
					break;
			}
			fin >> Data.Relation2;
			win1250_to_ascii(Data.Relation2);
			while (fin >> s) {
				if (s == "Wymagany")
					break;
			}
			while (fin >> s) {
				if (s == "|" || s == "\n")
					break;
			}
			fin >> s;
			s = s.substr(0, s.find("%"));
			Data.BrakeRatio = atof(s.c_str());
			while (fin >> s) {
				if (s == "Seria")
					break;
			}
			do {
				fin >> s;
			} while (!(s == "|" || fin.fail()));
			fin >> Data.LocSeries;
			fin >> Data.LocLoad; // = s2rE(ReadWord(fin));
			do {
				fin >> s;
			} while (!(contains(s, "[______________") || fin.fail()));
			// the header's flags hold from the start, entry 0 keeps them for the stations to inherit
			TimeTable[0].flags = Data.Flags;
			auto activeradiochannel{-1};
			while (!fin.fail() && !EndTable) {
				++StationCount;
				if (StationCount > MaxTTableSize) {
					--StationCount;
					conversionerror = timetable_too_long;
					break;
				}
				do {
					fin >> s;
				} while (!(s == "[" || fin.fail()));
				TMTableEntry *record = &TimeTable[StationCount];
				{
					if (s == "[")
						fin >> s;
					else
						conversionerror = -4;
					if (false == contains(s, "|")) {
						record->km = static_cast<float>(atof(s.c_str()));
						fin >> s;
					}
					if (contains(s, "|_____|")) /*zmiana predkosci szlakowej*/
						UpdateVelocity(vActual);
					else {
						fin >> s;
						if (false == contains(s, "|"))
							vActual = atof(s.c_str());
					}
					while (false == contains(s, "|") && false == fin.fail())
						fin >> s;
					fin >> record->StationName;
					// get rid of non-ascii chars. TODO: run correct version based on locale
					win1250_to_ascii(record->StationName);
					do {
						fin >> s;
					} while (!(s == "1" || s == "2" || fin.fail()));
					record->TrackNo = atoi(s.c_str());
					fin >> s;
					if (s != "|") {
						if (contains(s, hrsd)) {
							record->Ah = atoi(s.substr(0, s.find(hrsd)).c_str()); // godzina przyjazdu
							record->Am = static_cast<float>(atof(s.substr(s.find(hrsd) + 1, s.length()).c_str())); // minuta przyjazdu
						}
						else {
							record->Ah = TimeTable[StationCount - 1].Ah; // godzina z poprzedniej pozycji
							record->Am = static_cast<float>(atof(s.c_str())); // bo tylko minuty podane
						}
					}
					do {
						fin >> s;
					} while (!(s != "|" || fin.fail()));
					if (s != "]")
						record->tm = static_cast<float>(atof(s.c_str()));
					do {
						fin >> s;
					} while (!(s == "[" || fin.fail()));
					fin >> s;
					if (false == contains(s, "|")) {
						/*tu s moze byc miejscem zmiany predkosci szlakowej*/
						fin >> s;
					}
					if (s.find("|_____|") != std::string::npos) /*zmiana predkosci szlakowej*/
						UpdateVelocity(vActual);
					else {
						fin >> s;
						if (false == contains(s, "|"))
							vActual = atof(s.c_str());
					}
					while (false == contains(s, "|") && false == fin.fail())
						fin >> s;
					// stationware. added fix for empty entry
					fin >> s;
					while (false == (s == "1" || s == "2" || fin.fail())) {
						record->StationWare += s;
						fin >> s;
					}
					// cache relevant station data
					record->is_maintenance = contains(s, "pt");
					// service flags (eu07/maszyna#95): the ones of the station before, changed by the ones given here
					record->flags = TimeTable[StationCount - 1].flags;
					record->StationWare = timetable::extract_flags(record->StationWare, record->flags);
					{
						auto const stationware{Split(record->StationWare, ',')};
						for (auto const &entry : stationware) {
							if (entry.front() != 'R') {
								continue;
							}
							auto const [entrytype, entrynumber]{split_string_and_number(entry)};
							if (entrytype == "R" && entrynumber <= 10) {
								auto const radiochannel{entrynumber};
								if (record->radio_channel == -1 || radiochannel != activeradiochannel) {
									// if the station has more than one radiochannel listed,
									// it generally means we should switch to the one we weren't using so far
									// TODO: reverse this behaviour (keep the channel used so far) once W28 signs are included in the system
									record->radio_channel = radiochannel;
								}
							}
						}
						if (record->radio_channel != -1) {
							activeradiochannel = record->radio_channel;
						}
					}
					record->TrackNo = atoi(s.c_str());
					fin >> s;
					if (s != "|") {
						if (contains(s, hrsd)) {
							record->Dh = atoi(s.substr(0, s.find(hrsd)).c_str()); // godzina odjazdu
							record->Dm = static_cast<float>(atof(s.substr(s.find(hrsd) + 1, s.length()).c_str())); // minuta odjazdu
						}
						else {
							record->Dh = TimeTable[StationCount - 1].Dh; // godzina z poprzedniej pozycji
							record->Dm = static_cast<float>(atof(s.c_str())); // bo tylko minuty podane
						}
					}
					else {
						record->Dh = record->Ah; // odjazd o tej samej, co przyjazd (dla ostatniego też)
						record->Dm = record->Am; // bo są używane do wyliczenia opóźnienia po dojechaniu
					}
					do {
						fin >> s;
					} while (!(s != "|" || fin.fail()));
					if (s != "]")
						record->tm = static_cast<float>(atof(s.c_str()));
					do {
						fin >> s;
					} while (!(contains(s, "[") || fin.fail()));
					if (false == contains(s, "_|_"))
						fin >> s;
					if (false == contains(s, "|")) {
						/*tu s moze byc miejscem zmiany predkosci szlakowej*/
						fin >> s;
					}
					if (contains(s, "|_____|")) /*zmiana predkosci szlakowej*/
						UpdateVelocity(vActual);
					else {
						fin >> s;
						if (false == contains(s, "|"))
							vActual = atof(s.c_str());
					}
					while (false == contains(s, "|") && false == fin.fail())
						fin >> s;
					while (false == contains(s, "]") && false == fin.fail())
						fin >> s;
					if (contains(s, "_|_"))
						EndTable = true;
				} /*timetableline*/
			}
			if (false == EndTable && conversionerror == 0) {
				// the old reader went on reading past the end of the file here, forever
				conversionerror = timetable_truncated;
			}
		}
	} /*while eof*/
	return conversionerror;
}

} // namespace Mtable
