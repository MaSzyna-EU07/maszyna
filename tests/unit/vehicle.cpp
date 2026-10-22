#include "stdafx.h"
#include "vehicle.h"
#include "vehicle/DynObj.h"

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
std::map<std::string, section_format, std::less<>> const sections{
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
		throw definition_error("unknown vehicle definition section " + Section + ", see tests/unit/vehicle.cpp");
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

// the rows of a table section, each a list of values, and the line closing them
std::string table_text(std::string const &Section, section_format const &Format, nlohmann::ordered_json const &Rows) {
	if (Format.table_end == nullptr) {
		throw definition_error("vehicle definition section " + Section + " has no rows");
	}
	std::string text;
	for (auto const &row : Rows) {
		for (auto const &value : row) {
			text += value_text(value) + " ";
		}
		text += "\n";
	}
	return text + Format.table_end + "\n";
}

} // namespace

definition definition::load(std::string const &Name) {
	auto const path{std::string{EU07_UNIT_TESTS_DATA} + "/" + Name + ".json"};
	std::ifstream file(path);
	if (false == file.is_open()) {
		throw definition_error("no test vehicle definition " + path);
	}
	definition result;
	// not brace-initialised: json{other} makes an array of other with GCC and Clang (the initializer list constructor wins)
	auto const json = nlohmann::ordered_json::parse(file, nullptr, true, true);
	if (false == json.is_object()) {
		throw definition_error("test vehicle definition " + path + " isn't a JSON object");
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
		if (keys.contains("rows")) {
			text += table_text(section, format, keys["rows"]);
		}
	}
	return text;
}

vehicle::vehicle(definition const &Definition, double Velocity, std::string const &Name) {
	// the same steps as TDynamicObject::Init()
	m_mover = std::make_unique<TMoverParameters>(Velocity, Name, Name, 0);
	if (false == m_mover->LoadFIZ_Text(Definition.text())) {
		throw definition_error("the test vehicle definition didn't load");
	}
	m_mover->AssignLoad("", 0.f);
	m_mover->ComputeMass();
	auto const driveractive{Velocity != 0.0};
	if (false == m_mover->CheckLocomotiveParameters(driveractive, Velocity >= 0.0 ? 1 : -1)) {
		throw definition_error("parameters mismatch in the test vehicle definition");
	}
	m_mover->MainCtrlPos = m_mover->MainCtrlNoPowerPos();
	m_mover->LocalBrakePosA = 0.0;
	m_mover->BrakeCtrlPos = static_cast<int>(std::floor(m_mover->Handle->GetPos(driveractive ? bh_RP : bh_NP)));
	m_mover->BrakeLevelSet(m_mover->BrakeCtrlPos);
}

void vehicle::step(track const &Track, double Dt) {
	compute_forces(Dt);
	compute_movement(Track, Dt, TLocation{0.0, 0.0, 0.0});
}

void vehicle::compute_forces(double Dt) {
	if (m_pipepressure >= 0.0) {
		m_mover->Pipe->CreatePress(m_pipepressure);
	}
	m_mover->ComputeTotalForce(Dt);
}

double vehicle::compute_movement(track const &Track, double Dt, TLocation const &Location) {
	TTrackShape shape;
	shape.Len = 1000.0;
	shape.dHtrack = Track.gradient;
	TTrackParam param;
	param.Width = 1.435;
	param.friction = Track.friction;
	param.CategoryFlag = 1;
	TTractionParam const traction;
	TRotation const rotation{0.0, 0.0, 0.0};
	return m_mover->ComputeMovement(Dt, Dt, shape, param, traction, Location, rotation);
}

void vehicle::run(double Seconds, track const &Track, double Dt) {
	// whole steps: summing up Dt would make the step count depend on the rounding
	auto const steps{std::lround(Seconds / Dt)};
	for (long i = 0; i < steps; ++i) {
		step(Track, Dt);
	}
}

// the physics reaches the vehicle next to it through TDynamicObject::MoverParameters only (neighbour_data). the
// tests can't make a TDynamicObject, whose constructor and members come with the rest of the engine, so the
// neighbour is a zeroed block of its size with just that pointer set: enough for the physics, which reads nothing
// else of it, and never used as a whole object
static_assert(alignof(TDynamicObject) <= __STDCPP_DEFAULT_NEW_ALIGNMENT__);

namespace {

std::vector<std::byte> make_standin(TMoverParameters &Mover) {
	std::vector<std::byte> standin(sizeof(TDynamicObject)); // zeroed
	reinterpret_cast<TDynamicObject *>(standin.data())->MoverParameters = &Mover; // NOSONAR see above
	return standin;
}

TDynamicObject *as_neighbour(std::byte *Standin) {
	return reinterpret_cast<TDynamicObject *>(Standin); // NOSONAR see above
}

} // namespace

pair::pair(vehicle &First, int FirstEnd, vehicle &Second, int SecondEnd, double Gap) {
	auto const &first{First.mover()};
	auto const &second{Second.mover()};
	// the first vehicle in the middle, its FirstEnd towards the second one
	m_vehicles[0] = {&First, FirstEnd, 0.0, FirstEnd == end::front ? 1.0 : -1.0, make_standin(First.mover())};
	m_vehicles[1] = {&Second, SecondEnd, 0.5 * first.Dim.L + Gap + 0.5 * second.Dim.L, SecondEnd == end::front ? -1.0 : 1.0, make_standin(Second.mover())};
}

pair::~pair() {
	for (auto const &placed : m_vehicles) {
		// the vehicles outlive the pair, they mustn't point at its stand-ins
		for (auto &neighbour : placed.owner->mover().Neighbours) {
			neighbour = neighbour_data();
		}
	}
}

void pair::find_neighbours() {
	// as TDynamicObject::update_neighbours()
	for (auto const &placed : m_vehicles) {
		placed.owner->mover().Loc = TLocation{placed.position, 0.0, 0.0};
	}
	for (int i = 0; i < 2; ++i) {
		auto const &self{m_vehicles[i]};
		auto &other{m_vehicles[1 - i]}; // its stand-in is what this one points at
		auto &mover{self.owner->mover()};
		auto const &othermover{other.owner->mover()};
		auto &neighbour{mover.Neighbours[self.end]};
		neighbour.vehicle = as_neighbour(other.standin.data());
		neighbour.vehicle_end = other.end;
		neighbour.distance = static_cast<float>(TMoverParameters::CouplerDist(&mover, &othermover) - mover.Couplers[self.end].adapter_length -
		                                        othermover.Couplers[other.end].adapter_length);
		mover.Neighbours[1 - self.end] = neighbour_data();
	}
}

void pair::step(track const &Track, double Dt) {
	find_neighbours();
	for (auto const &placed : m_vehicles) {
		placed.owner->compute_forces(Dt);
	}
	for (auto &placed : m_vehicles) {
		placed.position += placed.heading * placed.owner->compute_movement(Track, Dt, TLocation{placed.position, 0.0, 0.0});
	}
}

void pair::run(double Seconds, track const &Track, double Dt) {
	// whole steps, as in vehicle::run()
	auto const steps{std::lround(Seconds / Dt)};
	for (long i = 0; i < steps; ++i) {
		step(Track, Dt);
	}
}

double pair::gap() const {
	auto const &first{m_vehicles[0].owner->mover()};
	auto const &second{m_vehicles[1].owner->mover()};
	return std::abs(m_vehicles[1].position - m_vehicles[0].position) - 0.5 * (first.Dim.L + second.Dim.L);
}

int pair::coupling() const {
	return m_vehicles[0].owner->mover().Couplers[m_vehicles[0].end].CouplingFlag;
}

} // namespace test
