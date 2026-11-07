/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/odometer.h"

#include "utilities/Logs.h"
#include "utilities/utilities.h"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace odometer {

namespace {

bool valid(double const Km) {
	return std::isfinite(Km) && Km >= 0.0;
}

// number making up all of the text, surrounding whitespace aside
std::optional<double> parse_number(std::string const &Text) {
	auto const first{Text.find_first_not_of(" \t\r\n")};
	if (first == std::string::npos)
		return std::nullopt;
	auto const last{Text.find_last_not_of(" \t\r\n")};
	auto const number{Text.substr(first, last - first + 1)};
	try {
		std::size_t used{0};
		auto const value{std::stod(number, &used)};
		if (used != number.size() || false == valid(value))
			return std::nullopt;
		return value;
	} catch (std::exception const &) {
		return std::nullopt;
	}
}

} // namespace

void reading::begin(double const Km, origin const Source) {
	km = Km;
	start = Km;
	source = Source;
}

void reading::seed(double const Km) {
	if (source != origin::guessed || false == valid(Km))
		return;
	km += Km - start;
	start = Km;
	source = origin::counter;
}

void reading::run(double const Metres) {
	km += std::abs(Metres) / 1000.0;
}

bool reading::moved() const {
	return km != start;
}

std::size_t store::read(std::istream &Input) {
	std::size_t count{0};
	std::string line;
	while (std::getline(Input, line)) {
		if (false == line.empty() && line.back() == '\r')
			line.pop_back();
		if (line.empty() || line.front() == '#')
			continue;
		// the name goes last, it's the rest of the line after the separator
		auto const separator{line.find_first_of(" \t")};
		if (separator == std::string::npos)
			continue;
		auto const km{parse_number(line.substr(0, separator))};
		auto const name{line.substr(separator + 1)};
		if (false == km.has_value() || name.empty())
			continue;
		m_states[name] = *km;
		++count;
	}
	return count;
}

void store::write(std::ostream &Output) const {
	Output << "# odometer states of the vehicles: <km> <vehicle name>\n";
	Output << std::fixed << std::setprecision(4);
	for (auto const &[name, km] : m_states)
		Output << km << ' ' << name << '\n';
}

std::optional<double> store::find(std::string const &Vehicle) const {
	auto const lookup{m_states.find(Vehicle)};
	if (lookup == m_states.end())
		return std::nullopt;
	return lookup->second;
}

void store::record(std::string const &Vehicle, reading const &Odometer) {
	if (Vehicle.empty() || false == valid(Odometer.km))
		return;
	auto const lookup{m_states.find(Vehicle)};
	if (lookup == m_states.end()) {
		if (false == Odometer.moved())
			return;
		m_states.emplace(Vehicle, Odometer.km);
		m_modified = true;
		return;
	}
	if (lookup->second == Odometer.km)
		return;
	lookup->second = Odometer.km;
	m_modified = true;
}

std::optional<double> read_legacy(std::istream &Input) {
	std::stringstream content;
	content << Input.rdbuf();
	return parse_number(content.str());
}

std::optional<double> legacy_state(std::filesystem::path const &Vehicledirectory, std::string const &Vehicle) {
	if (Vehicle.size() < 2)
		return std::nullopt;
	auto const path{Vehicledirectory / "kilometry" / (Vehicle.substr(0, Vehicle.size() - 1) + "_przebieg.txt")};
	std::error_code error;
	if (false == std::filesystem::is_regular_file(path, error))
		return std::nullopt;
	std::ifstream input{path, std::ios::binary};
	if (false == input.is_open())
		return std::nullopt;
	return read_legacy(input);
}

double guessed_state(std::string const &Vehicle) {
	// FNV-1a, stable between the builds unlike std::hash
	std::uint64_t hash{14695981039346656037ull};
	for (auto const character : Vehicle) {
		hash ^= static_cast<unsigned char>(character);
		hash *= 1099511628211ull;
	}
	return 10000.0 + static_cast<double>(hash % 490000ull);
}

reading starting_state(store const &States, std::string const &Vehicle, std::filesystem::path const &Vehicledirectory) {
	reading result;
	if (auto const saved{States.find(Vehicle)}) {
		result.begin(*saved, origin::saved);
	} else if (auto const legacy{legacy_state(Vehicledirectory, Vehicle)}) {
		result.begin(*legacy, origin::legacy);
	} else {
		result.begin(guessed_state(Vehicle), origin::guessed);
	}
	return result;
}

std::optional<double> counter_seed(float const Offset, float const Scale) {
	if (Offset == 0.f || Scale == 0.f)
		return std::nullopt;
	auto const km{static_cast<double>(Offset) / static_cast<double>(Scale)};
	if (false == valid(km))
		return std::nullopt;
	return km;
}

std::filesystem::path file() {
	auto path{user_config_path("eu07_odometers.txt")};
	return path.empty() ? std::filesystem::path{"eu07_odometers.txt"} : path;
}

store &states() {
	static store odometers = [] {
		store result;
		std::ifstream input{file(), std::ios::binary};
		if (input.is_open()) {
			auto const count{result.read(input)};
			WriteLog("Odometer states of " + std::to_string(count) + " vehicles read from \"" + file().string() + "\"");
		}
		return result;
	}();
	return odometers;
}

bool save() {
	auto &odometers{states()};
	if (false == odometers.modified())
		return true;
	auto const path{file()};
	std::error_code error;
	if (path.has_parent_path())
		std::filesystem::create_directories(path.parent_path(), error);
	// written aside first, a crash in the middle of a write doesn't lose the states
	auto temporary{path};
	temporary += ".tmp";
	{
		std::ofstream output{temporary, std::ios::binary | std::ios::trunc};
		if (false == output.is_open()) {
			ErrorLog("Failed to save the odometer states to \"" + temporary.string() + "\"");
			return false;
		}
		odometers.write(output);
		if (false == output.good()) {
			ErrorLog("Failed to save the odometer states to \"" + temporary.string() + "\"");
			return false;
		}
	}
	std::filesystem::rename(temporary, path, error);
	if (error) {
		ErrorLog("Failed to save the odometer states to \"" + path.string() + "\": " + error.message());
		return false;
	}
	odometers.saved();
	return true;
}

} // namespace odometer
