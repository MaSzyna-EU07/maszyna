#include "stdafx.h"
#include "golden.h"
#include "doctest.h"

#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

#include "McZapkie/hamulce.h"

namespace test {

namespace channels {

std::vector<channel> motion() {
	return {
	    {"V", [](auto const &M) { return M.V; }},
	    {"DistCounter", [](auto const &M) { return M.DistCounter; }},
	    {"AccS", [](auto const &M) { return M.AccS; }},
	    {"nrot", [](auto const &M) { return M.nrot; }},
	    {"Ft", [](auto const &M) { return M.Ft; }},
	    {"Fb", [](auto const &M) { return M.Fb; }},
	    {"FStand", [](auto const &M) { return M.FStand; }},
	    {"SlippingWheels", [](auto const &M) { return M.SlippingWheels ? 1.0 : 0.0; }},
	};
}

std::vector<channel> traction() {
	return {
	    {"Mains", [](auto const &M) { return M.Mains ? 1.0 : 0.0; }},
	    {"DirActive", [](auto const &M) { return M.DirActive; }},
	    {"MainCtrlPos", [](auto const &M) { return M.MainCtrlPos; }},
	    {"MainCtrlActualPos", [](auto const &M) { return M.MainCtrlActualPos; }},
	    {"ScndCtrlPos", [](auto const &M) { return M.ScndCtrlPos; }},
	    {"ScndCtrlActualPos", [](auto const &M) { return M.ScndCtrlActualPos; }},
	    {"Im", [](auto const &M) { return M.Im; }},
	    {"Itot", [](auto const &M) { return M.Itot; }},
	    {"EngineVoltage", [](auto const &M) { return M.EngineVoltage; }},
	    {"EnginePower", [](auto const &M) { return M.EnginePower; }},
	    {"enrot", [](auto const &M) { return M.enrot; }},
	    {"dizel_fill", [](auto const &M) { return M.dizel_fill; }},
	    {"eimv_pr", [](auto const &M) { return M.eimv_pr; }},
	    {"DynamicBrakeFlag", [](auto const &M) { return M.DynamicBrakeFlag ? 1.0 : 0.0; }},
	};
}

std::vector<channel> brakes() {
	return {
	    {"BrakeCtrlPos", [](auto const &M) { return M.BrakeCtrlPos; }},
	    {"PipePress", [](auto const &M) { return M.PipePress; }},
	    {"BrakePress", [](auto const &M) { return M.BrakePress; }},
	    {"LocBrakePress", [](auto const &M) { return M.LocBrakePress; }},
	    {"BRP", [](auto const &M) { return M.Hamulec->GetBRP(); }},
	    {"VRP", [](auto const &M) { return M.Hamulec->GetVRP(); }},
	    {"CRP", [](auto const &M) { return M.Hamulec->GetCRP(); }},
	};
}

std::vector<channel> air() {
	return {
	    {"Compressor", [](auto const &M) { return M.Compressor; }},
	    {"CompressorFlag", [](auto const &M) { return M.CompressorFlag ? 1.0 : 0.0; }},
	    {"ScndPipePress", [](auto const &M) { return M.ScndPipePress; }},
	};
}

std::vector<channel> handles() {
	std::vector<channel> result{
	    {"BrakeCtrlPosR", [](auto const &M) { return M.BrakeCtrlPosR; }},
	    {"dpMainValve", [](auto const &M) { return M.dpMainValve; }},
	    {"HandleCP", [](auto const &M) { return M.Handle->GetCP(); }},
	    {"HandleRP", [](auto const &M) { return M.Handle->GetRP(); }},
	    {"HandleEP", [](auto const &M) { return M.Handle->GetEP(); }},
	    {"LocalBrakePosA", [](auto const &M) { return M.LocalBrakePosA; }},
	    {"dpLocalValve", [](auto const &M) { return M.dpLocalValve; }},
	};
	for (int i = 0; i < 5; ++i) {
		result.push_back({"HandleSound" + std::to_string(i), [i](auto const &M) { return M.Handle->GetSound(i); }});
	}
	return result;
}

std::vector<channel> doors() {
	std::vector<channel> result{
	    {"DoorsLocked", [](auto const &M) { return M.Doors.is_locked ? 1.0 : 0.0; }},
	};
	for (int side : {side::right, side::left}) {
		auto const name{std::string{side == side::right ? "Right" : "Left"}};
		result.push_back({name + "Position", [side](auto const &M) { return M.Doors.instances[side].position; }});
		result.push_back({name + "Step", [side](auto const &M) { return M.Doors.instances[side].step_position; }});
		result.push_back({name + "Open", [side](auto const &M) { return M.Doors.instances[side].is_open ? 1.0 : 0.0; }});
		result.push_back({name + "Closed", [side](auto const &M) { return M.Doors.instances[side].is_closed ? 1.0 : 0.0; }});
		result.push_back({name + "Permit", [side](auto const &M) { return M.Doors.instances[side].open_permit ? 1.0 : 0.0; }});
	}
	return result;
}

std::vector<channel> parameters() {
	std::vector<channel> result{
	    {"CategoryFlag", [](auto const &M) { return M.CategoryFlag; }},
	    {"TrainType", [](auto const &M) { return M.TrainType; }},
	    {"EngineType", [](auto const &M) { return static_cast<int>(M.EngineType); }},
	    {"Vmax", [](auto const &M) { return M.Vmax; }},
	    {"Mass", [](auto const &M) { return M.Mass; }},
	    {"TotalMass", [](auto const &M) { return M.TotalMass; }},
	    {"Mred", [](auto const &M) { return M.Mred; }},
	    {"Power", [](auto const &M) { return M.Power; }},
	    {"HeatingPower", [](auto const &M) { return M.HeatingPower; }},
	    {"LightPower", [](auto const &M) { return M.LightPower; }},
	    {"BatteryVoltage", [](auto const &M) { return M.BatteryVoltage; }},
	    {"Cx", [](auto const &M) { return M.Cx; }},
	    {"WheelDiameter", [](auto const &M) { return M.WheelDiameter; }},
	    {"TrackW", [](auto const &M) { return M.TrackW; }},
	    {"AxleInertialMoment", [](auto const &M) { return M.AxleInertialMoment; }},
	    {"NPoweredAxles", [](auto const &M) { return M.NPoweredAxles; }},
	    {"NAxles", [](auto const &M) { return M.NAxles; }},
	    {"BearingType", [](auto const &M) { return M.BearingType; }},
	    {"ADist", [](auto const &M) { return M.ADist; }},
	    {"BDist", [](auto const &M) { return M.BDist; }},
	    {"NBpA", [](auto const &M) { return M.NBpA; }},
	    {"SandCapacity", [](auto const &M) { return M.SandCapacity; }},
	    {"BrakeSystem", [](auto const &M) { return static_cast<int>(M.BrakeSystem); }},
	    {"BrakeValve", [](auto const &M) { return static_cast<int>(M.BrakeValve); }},
	    {"BrakeHandle", [](auto const &M) { return static_cast<int>(M.BrakeHandle); }},
	    {"BrakeLocHandle", [](auto const &M) { return static_cast<int>(M.BrakeLocHandle); }},
	    {"LocalBrake", [](auto const &M) { return static_cast<int>(M.LocalBrake); }},
	    {"ASBType", [](auto const &M) { return M.ASBType; }},
	    {"MaxBrakeForce", [](auto const &M) { return M.MaxBrakeForce; }},
	    {"P2FTrans", [](auto const &M) { return M.P2FTrans; }},
	    {"BrakeMethod", [](auto const &M) { return M.BrakeMethod; }},
	    {"HighPipePress", [](auto const &M) { return M.HighPipePress; }},
	    {"LowPipePress", [](auto const &M) { return M.LowPipePress; }},
	    {"DeltaPipePress", [](auto const &M) { return M.DeltaPipePress; }},
	    {"CntrlPipePress", [](auto const &M) { return M.CntrlPipePress; }},
	    {"BrakeVolume", [](auto const &M) { return M.BrakeVolume; }},
	    {"BrakeVVolume", [](auto const &M) { return M.BrakeVVolume; }},
	    {"VeselVolume", [](auto const &M) { return M.VeselVolume; }},
	    {"BrakeCylNo", [](auto const &M) { return M.BrakeCylNo; }},
	    {"BrakeCylRadius", [](auto const &M) { return M.BrakeCylRadius; }},
	    {"BrakeCylDist", [](auto const &M) { return M.BrakeCylDist; }},
	    {"BrakeCylSpring", [](auto const &M) { return M.BrakeCylSpring; }},
	    {"BrakeSlckAdj", [](auto const &M) { return M.BrakeSlckAdj; }},
	    {"BrakeRigEff", [](auto const &M) { return M.BrakeRigEff; }},
	    {"RapidMult", [](auto const &M) { return M.RapidMult; }},
	    {"RapidVel", [](auto const &M) { return M.RapidVel; }},
	    {"BrakeValveSize", [](auto const &M) { return M.BrakeValveSize; }},
	    {"MinCompressor", [](auto const &M) { return M.MinCompressor; }},
	    {"MaxCompressor", [](auto const &M) { return M.MaxCompressor; }},
	    {"CompressorSpeed", [](auto const &M) { return M.CompressorSpeed; }},
	    {"EmergencyValveOn", [](auto const &M) { return M.EmergencyValveOn; }},
	    {"AirLeakRate", [](auto const &M) { return M.AirLeakRate; }},
	    {"BrakeCtrlPosNo", [](auto const &M) { return M.BrakeCtrlPosNo; }},
	    {"MainCtrlPosNo", [](auto const &M) { return M.MainCtrlPosNo; }},
	    {"ScndCtrlPosNo", [](auto const &M) { return M.ScndCtrlPosNo; }},
	    {"LightsPosNo", [](auto const &M) { return M.LightsPosNo; }},
	    {"MBrake", [](auto const &M) { return M.MBrake ? 1.0 : 0.0; }},
	    {"StopBrakeDecc", [](auto const &M) { return M.StopBrakeDecc; }},
	    {"BrakeDelays", [](auto const &M) { return M.BrakeDelays; }},
	    {"BrakeOpModes", [](auto const &M) { return M.BrakeOpModes; }},
	    {"Imin", [](auto const &M) { return M.Imin; }},
	    {"Imax", [](auto const &M) { return M.Imax; }},
	    {"EIMCtrlType", [](auto const &M) { return M.EIMCtrlType; }},
	    {"Volume", [](auto const &M) { return M.Volume; }},
	    {"PantVolume", [](auto const &M) { return M.PantVolume; }},
	};
	for (int i = 0; i < 5; ++i) {
		result.push_back({"MaxBrakePress" + std::to_string(i), [i](auto const &M) { return M.MaxBrakePress[i]; }});
	}
	for (int i = 0; i < 3; ++i) {
		result.push_back({"BrakeCylMult" + std::to_string(i), [i](auto const &M) { return M.BrakeCylMult[i]; }});
	}
	return result;
}

std::vector<channel> operator+(std::vector<channel> Left, std::vector<channel> const &Right) {
	Left.insert(Left.end(), Right.begin(), Right.end());
	return Left;
}

} // namespace channels

namespace {

bool environment(char const *Name) {
	auto const *value{std::getenv(Name)};
	return value != nullptr && value[0] != '\0' && value[0] != '0';
}

std::string number(double Value) {
	// the shortest text which reads back as the same double
	char text[32];
	auto const result{std::to_chars(std::begin(text), std::end(text), Value)};
	return {text, result.ptr};
}

// a line of the file, whichever line ends the checkout gave it
bool read_line(std::ifstream &File, std::string &Line) {
	if (false == static_cast<bool>(std::getline(File, Line))) {
		return false;
	}
	if (false == Line.empty() && Line.back() == '\r') {
		Line.pop_back();
	}
	return true;
}

std::vector<std::string> split(std::string const &Line) {
	std::vector<std::string> fields;
	std::stringstream stream(Line);
	std::string field;
	while (std::getline(stream, field, ',')) {
		fields.push_back(field);
	}
	if (false == Line.empty() && Line.back() == ',') {
		fields.emplace_back();
	}
	return fields;
}

bool same(std::string const &Recorded, std::string const &Now, bool const Exact) {
	if (Recorded == Now) {
		return true;
	}
	if (Exact) {
		return false;
	}
	char *end{nullptr};
	auto const recorded{std::strtod(Recorded.c_str(), &end)};
	auto const now{std::strtod(Now.c_str(), &end)};
	if (std::isnan(recorded) || std::isnan(now)) {
		return std::isnan(recorded) && std::isnan(now);
	}
	return std::abs(recorded - now) <= 1e-9 * std::max(std::abs(recorded), std::abs(now)) + 1e-12;
}

} // namespace

trace::trace(std::string Name, std::vector<channel> Channels, int Every, bool Figures)
    : m_name(std::move(Name)), m_channels(std::move(Channels)), m_every(Every), m_keep_figures(Figures) {}

void trace::sample(double Time, TMoverParameters const &Vehicle) {
	// the invariants on every step, the first time each is broken
	for (auto const &invariant : invariants()) {
		if (std::find(m_broken.begin(), m_broken.end(), invariant.name) != m_broken.end()) {
			continue;
		}
		if (false == invariant.holds(Vehicle, Time, m_sampled ? &m_before : nullptr)) {
			m_broken.push_back(invariant.name);
			char time[32];
			std::snprintf(time, sizeof(time), "%.3f", Time);
			m_violations.push_back("invariant \"" + invariant.name + "\" broken in " + m_name + " at " + time + " s" +
			                       (m_last_event.empty() ? std::string{} : ", after \"" + m_last_event + "\"") + ": " + invariant.explain(Vehicle));
		}
	}
	m_before = {Time, Vehicle.V, Vehicle.DistCounter};
	m_sampled = true;
	m_figures.sample(Time, Vehicle);

	if (m_step++ % m_every != 0) {
		return;
	}
	// the time is a label, summed up from the steps: milliseconds are all it needs
	char time[32];
	std::snprintf(time, sizeof(time), "%.3f", Time);
	std::string row{time};
	for (auto const &channel : m_channels) {
		row += "," + number(channel.value(Vehicle));
	}
	row += "," + m_event;
	m_event.clear();
	m_rows.push_back(row);
}

void trace::mark(std::string const &Event) {
	m_event += (m_event.empty() ? "" : " ") + Event;
	m_last_event = Event;
	m_figures.mark(Event);
}

void trace::check_figures(std::filesystem::path const &Path) const {
	std::ifstream file(Path, std::ios::binary);
	if (false == file.is_open()) {
		FAIL_CHECK("no key figures " << Path.string() << ", run the tests with EU07_UNIT_TESTS_UPDATE_GOLDEN=1 to record them");
		return;
	}
	std::vector<std::string> recorded;
	std::string line;
	while (read_line(file, line)) {
		recorded.push_back(line);
	}
	auto const differences{figures::compare(recorded, m_figures.lines())};
	if (false == differences.empty()) {
		std::string text;
		for (auto const &difference : differences) {
			text += "\n  " + difference;
		}
		FAIL_CHECK("key figures of " << m_name << " differ:" << text);
	}
}

void trace::check() const {
	for (std::size_t i = 0; i < m_violations.size(); ++i) {
		if (false == excused(m_name, m_broken[i])) {
			FAIL_CHECK(m_violations[i]);
		}
	}

	std::string header{"time"};
	for (auto const &channel : m_channels) {
		header += "," + channel.name;
	}
	header += ",event";

	// no spaces in the file names: the build depends on the files, and make doesn't take to them
	auto filename{m_name + ".csv"};
	std::replace(filename.begin(), filename.end(), ' ', '_');
	auto const path{std::filesystem::path{EU07_UNIT_TESTS_DATA} / "golden" / filename};
	auto figurespath{path};
	figurespath.replace_extension(".figures");
	if (environment("EU07_UNIT_TESTS_UPDATE_GOLDEN")) {
		std::filesystem::create_directories(path.parent_path());
		std::ofstream file(path, std::ios::binary);
		file << header << "\n";
		for (auto const &row : m_rows) {
			file << row << "\n";
		}
		if (m_keep_figures) {
			std::ofstream figuresfile(figurespath, std::ios::binary);
			for (auto const &line : m_figures.lines()) {
				figuresfile << line << "\n";
			}
		}
		MESSAGE("recorded the golden trace " << path.string());
		return;
	}

	if (m_keep_figures) {
		check_figures(figurespath);
	}

	// the samples aren't kept in the repository, they're recorded on the code before a change and compared after it
	auto const exact{environment("EU07_UNIT_TESTS_GOLDEN_EXACT")};
	std::ifstream file(path, std::ios::binary);
	if (false == file.is_open()) {
		if (exact) {
			FAIL_CHECK("no golden trace " << path.string() << ", run the tests with EU07_UNIT_TESTS_UPDATE_GOLDEN=1 on the code before the change to record it");
		}
		return;
	}
	std::string line;
	read_line(file, line);
	if (line != header) {
		FAIL_CHECK("golden trace " << m_name << " has other channels: " << line << ", the test records " << header);
		return;
	}
	auto const names{split(header)};
	std::size_t row{0};
	while (read_line(file, line)) {
		if (row >= m_rows.size()) {
			FAIL_CHECK("golden trace " << m_name << " is longer: " << m_rows.size() << " rows now, more recorded");
			return;
		}
		auto const recorded{split(line)};
		auto const now{split(m_rows[row])};
		for (std::size_t field = 0; field < names.size(); ++field) {
			auto const recordedvalue{field < recorded.size() ? recorded[field] : std::string{}};
			auto const nowvalue{field < now.size() ? now[field] : std::string{}};
			if (false == same(recordedvalue, nowvalue, exact || names[field] == "event")) {
				FAIL_CHECK("golden trace " << m_name << " differs first at time " << now.front() << " in " << names[field] << ": " << nowvalue
				                           << " now, " << recordedvalue << " recorded\n  now:      " << m_rows[row] << "\n  recorded: " << line);
				return;
			}
		}
		++row;
	}
	if (row != m_rows.size()) {
		FAIL_CHECK("golden trace " << m_name << " is shorter: " << m_rows.size() << " rows now, " << row << " recorded");
	}
}

} // namespace test
