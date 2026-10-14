#include "stdafx.h"
#include "vehicle.h"

#include <fstream>
#include <stdexcept>

namespace test {

namespace {

struct section_format {
	char separator; // the game's definitions end section names with either '.' or ':'
	char const *table_end{nullptr}; // the line closing the rows of a table section
};

// the sections TMoverParameters::LoadFIZ() reads, see its issection() calls. a section missing here
// would be silently skipped by the loader, and the test would run on the defaults instead
std::map<std::string, section_format> const sections{
    {"Param", {'.'}},
    {"Load", {':'}},
    {"Dimensions", {':'}},
    {"Wheels", {':'}},
    {"Brake", {':'}},
    {"Doors", {':'}},
    {"BuffCoupl", {'.'}},
    {"BuffCoupl1", {'.'}},
    {"BuffCoupl2", {'.'}},
    {"TurboPos", {':'}},
    {"Cntrl", {'.'}},
    {"Blending", {':'}},
    {"DCEMUED", {':'}},
    {"SpringBrake", {':'}},
    {"Light", {':'}},
    {"Headlights", {':'}},
    {"Clima", {':'}},
    {"Power", {':'}},
    {"SpeedControl", {':'}},
    {"Engine", {':'}},
    {"Switches", {':'}},
    {"Security", {':'}},
    {"Circuit", {':'}},
    {"AI", {':'}},
    {"LightsList", {':', "endL"}},
    {"DimmerList", {':', "endDimmerList"}},
    {"WiperList", {':', "endwl"}},
    {"CompressorList", {':', "endCL"}},
};

section_format const &format_of(std::string const &Section) {
	auto const lookup{sections.find(Section)};
	if (lookup == sections.end()) {
		throw std::runtime_error("unknown vehicle definition section " + Section + ", see tests/unit/vehicle.cpp");
	}
	return lookup->second;
}

std::string value_text(nlohmann::ordered_json const &Value) {
	if (Value.is_string()) {
		return Value.get<std::string>();
	}
	if (Value.is_boolean()) {
		return Value.get<bool>() ? "Yes" : "No";
	}
	return Value.dump();
}

} // namespace

definition definition::load(std::string const &Name) {
	auto const path{std::string{EU07_UNIT_TESTS_DATA} + "/" + Name + ".json"};
	std::ifstream file(path);
	if (false == file.is_open()) {
		throw std::runtime_error("no test vehicle definition " + path);
	}
	definition result;
	// not brace-initialised: json{other} makes an array of other with GCC and Clang (the initializer list constructor wins)
	auto const json = nlohmann::ordered_json::parse(file, nullptr, true, true);
	if (false == json.is_object()) {
		throw std::runtime_error("test vehicle definition " + path + " isn't a JSON object");
	}
	for (auto const &[section, keys] : json.items()) {
		// keys starting with _ are comments
		if (section.starts_with('_')) {
			continue;
		}
		format_of(section);
		result.m_sections[section] = keys;
	}
	return result;
}

definition &definition::set(std::string const &Section, std::string const &Key, nlohmann::ordered_json const &Value) {
	format_of(Section);
	m_sections[Section][Key] = Value;
	return *this;
}

definition &definition::remove(std::string const &Section) {
	m_sections.erase(Section);
	return *this;
}

definition &definition::remove(std::string const &Section, std::string const &Key) {
	if (m_sections.contains(Section)) {
		m_sections[Section].erase(Key);
	}
	return *this;
}

std::string definition::text() const {
	std::string text;
	for (auto const &[section, keys] : m_sections.items()) {
		auto const &format{format_of(section)};
		text += section + format.separator;
		for (auto const &[key, value] : keys.items()) {
			if (key != "rows") {
				text += " " + key + "=" + value_text(value);
			}
		}
		text += "\n";
		// table sections give their rows as a list of rows, each a list of values
		if (keys.contains("rows")) {
			if (format.table_end == nullptr) {
				throw std::runtime_error("vehicle definition section " + section + " has no rows");
			}
			for (auto const &row : keys["rows"]) {
				for (auto const &value : row) {
					text += value_text(value) + " ";
				}
				text += "\n";
			}
			text += std::string{format.table_end} + "\n";
		}
	}
	return text;
}

vehicle::vehicle(definition const &Definition, double Velocity, std::string const &Name) {
	// the same steps as TDynamicObject::Init()
	m_mover = std::make_unique<TMoverParameters>(Velocity, Name, Name, 0);
	if (false == m_mover->LoadFIZ_Text(Definition.text())) {
		throw std::runtime_error("the test vehicle definition didn't load");
	}
	m_mover->AssignLoad("", 0.f);
	m_mover->ComputeMass();
	auto const driveractive{Velocity != 0.0};
	if (false == m_mover->CheckLocomotiveParameters(driveractive, Velocity >= 0.0 ? 1 : -1)) {
		throw std::runtime_error("parameters mismatch in the test vehicle definition");
	}
	m_mover->MainCtrlPos = m_mover->MainCtrlNoPowerPos();
	m_mover->LocalBrakePosA = 0.0;
	m_mover->BrakeCtrlPos = static_cast<int>(std::floor(m_mover->Handle->GetPos(driveractive ? bh_RP : bh_NP)));
	m_mover->BrakeLevelSet(m_mover->BrakeCtrlPos);
}

void vehicle::step(track const &Track, double Dt) {
	TTrackShape shape;
	shape.Len = 1000.0;
	shape.dHtrack = Track.gradient;
	TTrackParam param;
	param.Width = 1.435;
	param.friction = Track.friction;
	param.CategoryFlag = 1;
	TTractionParam const traction;
	TLocation const location{0.0, 0.0, 0.0};
	TRotation const rotation{0.0, 0.0, 0.0};
	if (m_pipepressure >= 0.0) {
		m_mover->Pipe->CreatePress(m_pipepressure);
	}
	m_mover->ComputeTotalForce(Dt);
	m_mover->ComputeMovement(Dt, Dt, shape, param, traction, location, rotation);
}

void vehicle::run(double Seconds, track const &Track, double Dt) {
	// whole steps: summing up Dt would make the step count depend on the rounding
	auto const steps{std::lround(Seconds / Dt)};
	for (long i = 0; i < steps; ++i) {
		step(Track, Dt);
	}
}

} // namespace test
