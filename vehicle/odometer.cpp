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

#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/utilities.h"

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <format>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string_view>
#include <thread>
#include <vector>
#ifdef _WIN32
#include <process.h>
#else
#include <unistd.h>
#endif

namespace odometer {

namespace {

bool valid(double const Km) {
	return std::isfinite(Km) && Km >= 0.0;
}

int process_id() {
#ifdef _WIN32
	return _getpid();
#else
	return static_cast<int>(getpid());
#endif
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

// the comment line which gives the day the states after it were last updated
constexpr std::string_view stamp_prefix{"# updated "};

// a whole number making up all of the text
std::optional<int> parse_integer(std::string_view const Text) {
	if (Text.empty() || Text.front() == '-')
		return std::nullopt;
	int value{0};
	auto const *const end{Text.data() + Text.size()};
	if (auto const [used, error] {std::from_chars(Text.data(), end, value)}; error != std::errc{} || used != end)
		return std::nullopt;
	return value;
}

// day written as yyyy-mm-dd
std::optional<day> parse_day(std::string_view const Text) {
	if (Text.size() != 10 || Text[4] != '-' || Text[7] != '-')
		return std::nullopt;
	auto const year{parse_integer(Text.substr(0, 4))};
	auto const month{parse_integer(Text.substr(5, 2))};
	auto const dayofmonth{parse_integer(Text.substr(8, 2))};
	if (false == (year.has_value() && month.has_value() && dayofmonth.has_value()))
		return std::nullopt;
	std::chrono::year_month_day const date{std::chrono::year{*year}, std::chrono::month{static_cast<unsigned>(*month)},
	                                       std::chrono::day{static_cast<unsigned>(*dayofmonth)}};
	if (false == date.ok())
		return std::nullopt;
	return day{date};
}

std::string format_day(day const Day) {
	std::chrono::year_month_day const date{Day};
	return std::format("{:04}-{:02}-{:02}", static_cast<int>(date.year()), static_cast<unsigned>(date.month()),
	                   static_cast<unsigned>(date.day()));
}

// the day of a stamp line, as written, a day later than Today included. Today for a malformed one: the states after it
// are kept
day stamp_day(std::string_view const Line, day const Today) {
	auto text{Line.substr(stamp_prefix.size())};
	while (false == text.empty() && (text.back() == ' ' || text.back() == '\t'))
		text.remove_suffix(1);
	return parse_day(text).value_or(Today);
}

} // namespace

day today() {
	return std::chrono::floor<std::chrono::days>(std::chrono::system_clock::now());
}

std::chrono::days expiry() {
	return std::chrono::days{std::max(0, Global.odometer_expiry)};
}

void reading::begin(double const Km, origin const Source) {
	km = Km;
	start = Km;
	source = Source;
	guess = Km;
}

void reading::seed(double const Km) {
	if (source != origin::guessed || false == valid(Km))
		return;
	auto const shift{Km - guess};
	km += shift;
	start += shift;
	guess = Km;
	source = origin::counter;
}

void reading::run(double const Metres) {
	km += std::abs(Metres) / 1000.0;
}

bool reading::moved() const {
	return km != start;
}

std::size_t store::read(std::istream &Input, day const Today) {
	std::size_t count{0};
	// the states with no stamp before them, as the builds without the stamps wrote them, count as updated today
	auto stamp{Today};
	std::string line;
	while (std::getline(Input, line)) {
		if (false == line.empty() && line.back() == '\r')
			line.pop_back();
		if (line.starts_with(stamp_prefix))
			stamp = stamp_day(line, Today);
		if (line.empty() || line.front() == '#')
			continue;
		// the name goes last, it's the rest of the line after the separator
		auto const separator{line.find_first_of(" \t")};
		if (separator == std::string::npos)
			continue;
		auto number{line.substr(0, separator)};
		auto const guessed{false == number.empty() && number.front() == '~'};
		if (guessed)
			number.erase(0, 1);
		auto const km{parse_number(number)};
		auto const name{line.substr(separator + 1)};
		if (false == km.has_value() || name.empty())
			continue;
		m_states[name] = state{*km, guessed, stamp};
		m_read_latest = std::max(m_read_latest.value_or(stamp), stamp);
		++count;
	}
	return count;
}

void store::write(std::ostream &Output) const {
	Output << "# odometer states of the vehicles: <km> <vehicle name>, ~<km> for one counted from a made up state\n";
	Output << "# the states after a line \"" << stamp_prefix << "<yyyy-mm-dd>\" were last updated on that day (UTC)\n";
	Output << std::fixed << std::setprecision(4);
	// by the day, then by the name as the map has them
	std::vector<std::map<std::string, state>::value_type const *> entries;
	entries.reserve(m_states.size());
	for (auto const &entry : m_states)
		entries.push_back(&entry);
	std::ranges::stable_sort(entries, {}, [](auto const *Entry) { return Entry->second.updated; });
	std::optional<day> group;
	for (auto const *entry : entries) {
		auto const &written{entry->second};
		if (group != written.updated) {
			group = written.updated;
			Output << stamp_prefix << format_day(written.updated) << '\n';
		}
		Output << (written.guessed ? "~" : "") << written.km << ' ' << entry->first << '\n';
	}
}

std::optional<double> store::find(std::string const &Vehicle) const {
	auto const lookup{m_states.find(Vehicle)};
	if (lookup == m_states.end())
		return std::nullopt;
	return lookup->second.km;
}

std::optional<day> store::updated(std::string const &Vehicle) const {
	auto const lookup{m_states.find(Vehicle)};
	if (lookup == m_states.end())
		return std::nullopt;
	return lookup->second.updated;
}

bool store::guessed(std::string const &Vehicle) const {
	auto const lookup{m_states.find(Vehicle)};
	return lookup != m_states.end() && lookup->second.guessed;
}

bool store::newer(state const &Left, state const &Right) {
	if (Left.guessed != Right.guessed)
		return Right.guessed;
	return Left.km > Right.km;
}

void store::record(std::string const &Vehicle, reading const &Odometer, day const Today) {
	if (Vehicle.empty() || false == valid(Odometer.km))
		return;
	state const recorded{Odometer.km, Odometer.source == origin::guessed, Today};
	auto const lookup{m_states.find(Vehicle)};
	if (lookup == m_states.end()) {
		if (false == Odometer.moved())
			return;
		m_states.emplace(Vehicle, recorded);
		m_modified = true;
		return;
	}
	if (false == newer(recorded, lookup->second))
		return;
	// a clock set back doesn't move the day back
	auto const latest{std::max(lookup->second.updated, Today)};
	lookup->second = recorded;
	lookup->second.updated = latest;
	m_modified = true;
}

bool kept(bool const Drivercab, reading const &Odometer) {
	return Drivercab || Odometer.shown || Odometer.source == origin::counter;
}

void record(store &States, std::string const &Vehicle, bool const Drivercab, reading const &Odometer) {
	if (false == kept(Drivercab, Odometer))
		return;
	States.record(Vehicle, Odometer);
}

void store::merge(store const &Other) {
	for (auto const &[name, other] : Other.m_states) {
		auto const [lookup, added] {m_states.try_emplace(name, other)};
		if (added)
			continue;
		auto &mine{lookup->second};
		auto const latest{std::max(mine.updated, other.updated)};
		if (newer(other, mine))
			mine = other;
		mine.updated = latest;
	}
}

std::size_t store::expire(day const Today, std::chrono::days const Period) {
	if (Period <= std::chrono::days::zero())
		return 0;
	// not from a later Today alone: a clock set forward, or the first save after a break, doesn't drop every state not
	// run in this session
	auto const since{std::min(Today, m_read_latest.value_or(Today))};
	return std::erase_if(m_states, [since, Period](auto const &Entry) { return since - Entry.second.updated > Period; });
}

bool store::holds(store const &Other) const {
	return std::all_of(Other.m_states.begin(), Other.m_states.end(), [this](auto const &Entry) {
		auto const lookup{m_states.find(Entry.first)};
		return lookup != m_states.end() && false == newer(Entry.second, lookup->second);
	});
}

std::optional<double> read_legacy(std::istream &Input) {
	std::stringstream content;
	content << Input.rdbuf();
	return parse_number(content.str());
}

namespace {

std::optional<double> read_legacy_file(std::filesystem::path const &Path) {
	std::error_code error;
	if (false == std::filesystem::is_regular_file(Path, error))
		return std::nullopt;
	std::ifstream input{Path, std::ios::binary};
	if (false == input.is_open())
		return std::nullopt;
	return read_legacy(input);
}

} // namespace

std::optional<double> legacy_state(std::filesystem::path const &Vehicledirectory, std::string const &Vehicle,
                                   std::filesystem::path const &Scriptsdirectory) {
	if (Vehicle.empty())
		return std::nullopt;
	// the renderers of the units drop the letter of the car, the file is the unit's (see the header)
	if (Vehicle.size() > 1) {
		auto const unit{read_legacy_file(Vehicledirectory / "kilometry" / (Vehicle.substr(0, Vehicle.size() - 1) + "_przebieg.txt"))};
		if (unit.has_value())
			return unit;
	}
	// the shared screens of scripts/ keep the whole name
	return read_legacy_file(Scriptsdirectory / "kilometry" / (Vehicle + "_przebieg.txt"));
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

reading starting_state(store const &States, std::string const &Vehicle, std::filesystem::path const &Vehicledirectory,
                       std::filesystem::path const &Scriptsdirectory) {
	reading result;
	if (auto const saved{States.find(Vehicle)}; saved.has_value()) {
		if (States.guessed(Vehicle)) {
			// the vehicle ran with a made up state before any cab with a known one loaded it (as the AI's or loose
			// stock); the made up part of it stays replaceable
			result.begin(*saved, origin::guessed);
			result.guess = std::min(*saved, guessed_state(Vehicle));
		} else {
			result.begin(*saved, origin::saved);
		}
	} else if (auto const legacy{legacy_state(Vehicledirectory, Vehicle, Scriptsdirectory)}; legacy.has_value()) {
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

bool persistent() {
	return false == Global.test.enabled;
}

store &states() {
	if (false == persistent()) {
		// the vehicles of a test run count their odometers as usual, from the guessed or legacy states, with nothing
		// read from or saved to the player's file
		static store scratch;
		return scratch;
	}
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

namespace {

// the states as the file holds them once written, the km rounded to the digits written
store as_written(store const &States, day const Today) {
	std::stringstream text;
	States.write(text);
	store result;
	result.read(text, Today);
	return result;
}

} // namespace

bool save(store &States, std::filesystem::path const &File, day const Today, std::chrono::days const Period) {
	if (false == States.modified() && false == States.unconfirmed())
		return true;
	std::error_code error;
	std::size_t expired{0};
	if (File.has_parent_path())
		std::filesystem::create_directories(File.parent_path(), error);
	// other simulations on the machine (multiplayer, parallel runs) keep their states in the same file: what they
	// saved since this one read it is merged in, not overwritten
	{
		store saved;
		std::ifstream input{File, std::ios::binary};
		if (input.is_open())
			saved.read(input, Today);
		States.merge(saved);
		// the vehicles not run for long, of this simulation and of the others alike, so the file doesn't keep every
		// vehicle ever run. they're logged once the file is written without them: a save that doesn't write it leaves
		// them in it (the next save that writes it drops them again), and a write that fails is tried again
		expired = States.expire(Today, Period);
		// another simulation that read the file before the last save of this one and put its own in place after it
		// dropped the states this one wrote. when the file holds them all, the same or newer, there's nothing to write
		if (false == States.modified()) {
			if (saved.holds(as_written(States, Today))) {
				States.confirmed();
				return true;
			}
			WriteLog("Odometer states saved to \"" + File.string() + "\" are missing from it, saving them again");
		}
	}
	// written aside first, a crash in the middle of a write doesn't lose the states. the file aside is this process'
	// own, the other simulations write theirs
	auto temporary{File};
	temporary += "." + std::to_string(process_id()) + ".tmp";
	{
		std::ofstream output{temporary, std::ios::binary | std::ios::trunc};
		if (false == output.is_open()) {
			ErrorLog("Failed to save the odometer states to \"" + temporary.string() + "\"");
			return false;
		}
		States.write(output);
		if (false == output.good()) {
			output.close();
			std::filesystem::remove(temporary, error);
			ErrorLog("Failed to save the odometer states to \"" + temporary.string() + "\"");
			return false;
		}
	}
	// the file can't be replaced while another simulation is reading it, that takes a moment
	for (auto attempt{0};; ++attempt) {
		std::filesystem::rename(temporary, File, error);
		if (false == static_cast<bool>(error) || attempt == 4)
			break;
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}
	if (error) {
		ErrorLog("Failed to save the odometer states to \"" + File.string() + "\": " + error.message());
		std::error_code ignored;
		std::filesystem::remove(temporary, ignored);
		return false;
	}
	States.saved();
	if (expired > 0)
		WriteLog(std::format("Odometer states of {} vehicles not updated for {} days dropped", expired, Period.count()));
	return true;
}

bool save() {
	if (false == persistent())
		return true;
	return save(states(), file());
}

} // namespace odometer
