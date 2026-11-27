/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "world/timetableflags.h"

#include "utilities/Logs.h"
#include "utilities/utilities.h"

namespace timetable {

namespace {

void apply_one(tristate &Flag, tristate const Other) {
	if (Other != tristate::unset) {
		Flag = Other;
	}
}

tristate *flag_named(std::string_view const Key, service_flags &Flags) {
	if (Key == "drzwi_centralne") {
		return &Flags.central_doors;
	}
	if (Key == "stopien") {
		return &Flags.doorstep;
	}
	if (Key == "kierownik") {
		return &Flags.conductor;
	}
	if (Key == "proba_hamulca") {
		return &Flags.brake_test;
	}
	return nullptr;
}

} // namespace

void service_flags::apply(service_flags const &Other) {
	apply_one(central_doors, Other.central_doors);
	apply_one(doorstep, Other.doorstep);
	apply_one(conductor, Other.conductor);
	apply_one(brake_test, Other.brake_test);
}

bool read_flag(std::string_view const Entry, service_flags &Flags) {
	auto const separator{Entry.find('=')};
	if (separator == std::string_view::npos) {
		return false;
	}
	auto const key{ToLower(std::string{Entry.substr(0, separator)})};
	auto *flag{flag_named(key, Flags)};
	if (flag == nullptr) {
		return false;
	}
	auto const value{ToLower(std::string{Entry.substr(separator + 1)})};
	if (value == "tak" || value == "1") {
		*flag = tristate::yes;
	}
	else if (value == "nie" || value == "0") {
		*flag = tristate::no;
	}
	else {
		ErrorLog("Bad timetable: unknown value of " + key + " in \"" + std::string{Entry} + "\", expected tak or nie");
	}
	return true;
}

std::string extract_flags(std::string const &Ware, service_flags &Flags) {
	if (false == contains(Ware, '=')) {
		// no flags, the list as it was
		return Ware;
	}
	std::string rest;
	for (auto const &entry : Split(Ware, ',')) {
		if (read_flag(entry, Flags)) {
			continue;
		}
		if (contains(entry, '=')) {
			ErrorLog("Bad timetable: unknown flag \"" + entry + "\" in station ware \"" + Ware + "\"");
		}
		if (false == rest.empty()) {
			rest += ',';
		}
		rest += entry;
	}
	return rest;
}

int stop_flags_entry(int const StationIndex, int const StationCount) {
	return std::clamp(StationIndex, 0, std::max(StationCount, 0));
}

int run_flags_entry(int const StationIndex, int const StationCount) {
	return std::clamp(StationIndex - 1, 0, std::max(StationCount, 0));
}

} // namespace timetable
