#include "stdafx.h"
#include "vehicle.h"
#include "golden.h"
#include "real_vehicles.h"
#include "fiz.h"

#include <filesystem>
#include <fstream>
#include <stdexcept>

#include "utilities/Globals.h"

namespace test {

namespace {

fiz::section const &format_of(std::string const &Section) {
	auto const section{fiz::named(Section)};
	if (section == nullptr) {
		throw definition_error("vehicle definition section " + Section + " isn't one the loader reads, see tests/unit/fiz.cpp");
	}
	return *section;
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

// the rows of a list section, each a list of values, and the line closing them
std::string table_text(std::string const &Section, fiz::section const &Format, nlohmann::ordered_json const &Rows) {
	if (Format.end == nullptr) {
		throw definition_error("vehicle definition section " + Section + " has no rows");
	}
	std::string text;
	for (auto const &row : Rows) {
		for (auto const &value : row) {
			text += value_text(value) + " ";
		}
		text += "\n";
	}
	return text + Format.end + "\n";
}

nlohmann::ordered_json sections_of(std::string const &Path) {
	std::ifstream file(Path);
	if (false == file.is_open()) {
		throw definition_error("no test vehicle definition " + Path);
	}
	// not brace-initialised: json{other} makes an array of other with GCC and Clang (the initializer list constructor wins)
	auto const json = nlohmann::ordered_json::parse(file, nullptr, true, true);
	if (false == json.is_object()) {
		throw definition_error("test vehicle definition " + Path + " isn't a JSON object");
	}
	auto result = nlohmann::ordered_json::object();
	for (auto const &[section, keys] : json.items()) {
		// keys starting with _ are comments
		if (section.starts_with('_')) {
			continue;
		}
		format_of(section);
		result[section] = keys;
	}
	return result;
}

} // namespace

definition definition::load(std::string const &Name) {
	definition result;
	result.m_sections = sections_of(std::string{EU07_UNIT_TESTS_DATA} + "/" + Name + ".json");
	return result;
}

definition definition::fiz(std::string const &Path) {
	definition result;
	result.m_sections = sections_of(std::string{EU07_UNIT_TESTS_DATA} + "/fiz/" + Path + ".json");
	auto const slash{Path.rfind('/')};
	result.m_typename = slash == std::string::npos ? Path : Path.substr(slash + 1);
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

vehicle::vehicle(definition const &Definition, double Velocity, std::string const &Name, int Cab) : m_cab(Cab) {
	// the same steps as TDynamicObject::Init(), for a vehicle set the usual way round (iDirection 1)
	auto const typename_{Definition.type_name().empty() ? Name : Definition.type_name()};
	m_mover = std::make_unique<TMoverParameters>(Velocity, typename_, Name, Cab);
	if (false == m_mover->LoadFIZ_Text(Definition.text())) {
		throw definition_error("the test vehicle definition didn't load");
	}
	m_mover->AssignLoad("", 0.f);
	m_mover->ComputeMass();
	auto const driveractive{Velocity != 0.0};
	if (false == m_mover->CheckLocomotiveParameters(driveractive, (Velocity > 0 ? 1 : -1) * Cab)) {
		throw definition_error("parameters mismatch in the test vehicle definition");
	}
	m_mover->MainCtrlPos = m_mover->MainCtrlNoPowerPos();
	m_mover->LocalBrakePosA = 0.0;
	if (driveractive) {
		m_mover->BrakeCtrlPos = static_cast<int>(std::floor(m_mover->Handle->GetPos(Cab == 0 ? bh_NP : bh_RP)));
		// a driver keeps a vehicle on standby on its independent brake
		if (Cab != 0 && m_mover->LocalBrake != TLocalBrake::ManualBrake && Velocity < 1.0) {
			m_mover->IncLocalBrakeLevel(LocalBrakePosNo);
			if (m_mover->EIMCtrlEmergency) {
				m_mover->DecLocalBrakeLevel(1);
			}
		}
	}
	else {
		m_mover->BrakeCtrlPos = static_cast<int>(std::floor(m_mover->Handle->GetPos(bh_NP)));
	}
	m_mover->BrakeLevelSet(m_mover->BrakeCtrlPos);
}

void vehicle::step(track const &Track, double Dt) {
	begin_step();
	TLocation const location{0.0, 0.0, 0.0};
	for (int iteration = 1; iteration < Track.iterations; ++iteration) {
		forces(Dt);
		fast_move(Track, Dt, location);
	}
	forces(Dt);
	move(Track, Dt, location, m_mover->PipePress);
}

namespace {

TTrackShape shape_of(track const &Track) {
	TTrackShape shape;
	shape.Len = 1000.0;
	shape.dHtrack = Track.gradient;
	return shape;
}

TTrackParam parameters_of(track const &Track, TMoverParameters const &Mover) {
	TTrackParam param;
	param.Width = Track.width;
	param.friction = Track.friction;
	// the vehicle's own kind of way: a road vehicle on rails derails as soon as it moves
	param.CategoryFlag = Track.category != 0 ? Track.category : (Mover.CategoryFlag != 0 ? Mover.CategoryFlag : 1);
	return param;
}

} // namespace

double vehicle::fast_move(track const &Track, double Dt, TLocation const &Location) {
	// TDynamicObject::FastUpdate(), the iterations of a frame before the last one
	auto param{parameters_of(Track, *m_mover)};
	return m_mover->FastComputeMovement(Dt, shape_of(Track), param, Location, TRotation{0.0, 0.0, 0.0});
}

void vehicle::begin_step() {
	// the conditions the test keeps
	if (m_pipepressure >= 0.0) {
		m_mover->Pipe->CreatePress(m_pipepressure);
	}
	if (m_reservoirpressure >= 0.0 && m_mover->VeselVolume > 0.0) {
		m_mover->CompressedVolume = m_reservoirpressure * m_mover->VeselVolume;
	}
	// vehicle_table::update(), before the forces
	m_mover->ComputeConstans();
}

void vehicle::forces(double Dt) {
	// TDynamicObject::UpdateForce()
	m_mover->ComputeTotalForce(Dt);
}

double vehicle::move(track const &Track, double Dt, TLocation const &Location, double Trainpipe) {
	auto const shape{shape_of(Track)};
	auto const param{parameters_of(Track, *m_mover)};
	TRotation const rotation{0.0, 0.0, 0.0};

	// what TDynamicObject::Update() does before the movement. a raised pantograph touches the wire at once,
	// the game animates its rise
	auto &mover{*m_mover};
	mover.PantFrontVolt = mover.Pantographs[end::front].is_active ? Track.voltage : 0.0;
	mover.PantRearVolt = mover.Pantographs[end::rear].is_active ? Track.voltage : 0.0;
	TTractionParam traction;
	traction.TractionVoltage =
	    mover.EnginePowerSource.SourceType == TPowerSource::CurrentCollector ? std::max(std::abs(mover.PantRearVolt), std::abs(mover.PantFrontVolt)) : 0.95 * mover.EnginePowerSource.MaxVoltage;
	traction.TractionFreq = 0;
	traction.TractionMaxCurrent = 7500;
	traction.TractionResistivity = 0.3;
	mover.PantographVoltage = traction.TractionVoltage;
	if (m_cab != 0) {
		// the driver's own vehicle, which gets the average pressure of the train's brake pipe
		mover.EqvtPipePress = Trainpipe;
		auto const diesel{mover.EngineType == TEngineType::DieselEngine || mover.EngineType == TEngineType::DieselElectric};
		if (diesel && mover.EIMCtrlType > 0) {
			mover.CheckEIMIC(Dt);
			if (mover.SpeedCtrl) {
				mover.CheckSpeedCtrl(Dt);
			}
			mover.eimic_real = std::min(mover.eimic, mover.eimicSpeedCtrl);
		}
		if (mover.EngineType == TEngineType::ElectricInductionMotor) {
			// the share of the electric and pneumatic brake over the train (TDynamicObject's MED) isn't here
			mover.CheckEIMIC(Dt);
			mover.CheckSpeedCtrl(Dt);
			mover.eimic_real = std::min(mover.eimic, mover.eimicSpeedCtrl);
		}
	}

	// the whole frame's time, of all its iterations
	auto const frame{Dt * Track.iterations};
	auto const distance{m_mover->ComputeMovement(Dt, frame, shape, param, traction, Location, rotation)};
	m_time += frame;
	if (m_trace != nullptr) {
		m_trace->sample(m_time, *m_mover);
	}
	return distance;
}

void vehicle::run(double Seconds, track const &Track, double Dt) {
	// whole frames: summing up Dt would make their count depend on the rounding
	auto const steps{std::lround(Seconds / (Dt * Track.iterations))};
	for (long i = 0; i < steps; ++i) {
		step(Track, Dt);
	}
}

void reseed_random() {
	Global.random_engine.seed(1);
	Global.local_random_engine.seed(1);
}

std::vector<std::string> const &real_vehicles() {
	static std::vector<std::string> const paths{[] {
		std::vector<std::string> result;
		auto const root{std::filesystem::path{EU07_UNIT_TESTS_DATA} / "fiz"};
		for (auto const &entry : std::filesystem::recursive_directory_iterator(root)) {
			if (entry.is_regular_file() && entry.path().extension() == ".json") {
				result.push_back(std::filesystem::relative(entry.path(), root).replace_extension().generic_string());
			}
		}
		std::sort(result.begin(), result.end());
		return result;
	}()};
	return paths;
}

} // namespace test
