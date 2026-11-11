/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "simulation/blackboxrecorder.h"

#include "simulation/simulation.h"
#include "simulation/simulationtime.h"
#include "input/command.h"
#include "vehicle/Driver.h"
#include "vehicle/DynObj.h"
#include "vehicle/Train.h"
#include "widgets/map_objects.h"
#include "world/Event.h"
#include "world/MemCell.h"
#include "world/Segment.h"
#include "world/Track.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/utilities.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <ctime>
#include <utility>
#include <filesystem>
#include <format>
#include <limits>

namespace simulation {

blackbox_recorder Blackbox;

namespace {

// [s] between the samples of the user's train, the looks at everything else and the full looks at the vehicles
constexpr double cabinterval{0.1};
constexpr double lookinterval{1.0};
constexpr double slowlook{20.0}; // [ms] a look around taking longer is told in the log, part by part
constexpr double keyframeinterval{60.0};
constexpr double camerainterval{0.5};

// the camera is written again once it moved more than this [m], or turned more than this [rad]
constexpr double cameramove{0.5};
constexpr float cameraturn{0.05f};

// a vehicle is listed again once it moved more than this [m], or changed speed more than this [km/h]
constexpr float listmove{0.1f};
constexpr float listspeed{0.5f};

std::string text_of(std::string_view const Win1250) {
	return win1250_to_utf8(std::string{Win1250});
}

blackbox::driver driver_of(TDynamicObject const &Vehicle) {
	if (Vehicle.Mechanik == nullptr)
		return blackbox::driver::none;
	return Vehicle.Mechanik->AIControllFlag ? blackbox::driver::ai : blackbox::driver::human;
}

// the switches of the user's train: those of its own vehicle, and those of the one with the motors it drives
std::uint32_t devices_of(TMoverParameters const &Occupied, TMoverParameters const &Controlling) {
	namespace device = blackbox::cab_devices;
	auto const on{[](bool const State, std::uint32_t const Bit) { return State ? Bit : 0u; }};
	return on(Occupied.Battery || Controlling.Battery, device::battery) | on(Controlling.Mains, device::main_breaker) |
	       on(Controlling.Pantographs[end::front].is_active, device::pantograph_front) | on(Controlling.Pantographs[end::rear].is_active, device::pantograph_rear) |
	       on(Controlling.ConverterFlag, device::converter) | on(Controlling.CompressorFlag, device::compressor) | on(Controlling.StLinFlag, device::line_contactors) |
	       on(Controlling.FuseFlag, device::overload_relay) | on(Controlling.SandDose, device::sanding) | on(Occupied.WarningSignal != 0, device::horn) |
	       on(false == Occupied.Doors.instances[side::left].is_closed, device::doors_left) |
	       on(false == Occupied.Doors.instances[side::right].is_closed, device::doors_right) | on(Occupied.Radio, device::radio) |
	       on(Controlling.Heating, device::heating) | on(Occupied.SpringBrake.IsActive, device::spring_brake) | on(Controlling.ShuntMode, device::shunt_mode) |
	       // the doors as the cab works them: the permits, the departure signal, the steps and the lock
	       on(Occupied.DepartureSignal, device::departure_signal) | on(Occupied.Doors.instances[side::left].open_permit, device::door_permit_left) |
	       on(Occupied.Doors.instances[side::right].open_permit, device::door_permit_right) | on(Occupied.Doors.step_enabled, device::door_step) |
	       on(Occupied.Doors.lock_enabled, device::door_lock) | on(Occupied.Doors.is_locked, device::doors_locked) |
	       on(Occupied.Hamulec != nullptr && Occupied.Hamulec->Releaser(), device::brake_releaser) | on(Controlling.FuelPump.is_active, device::fuel_pump) |
	       on(Controlling.OilPump.is_active, device::oil_pump) |
	       on(Controlling.MotorBlowers[end::front].is_active || Controlling.MotorBlowers[end::rear].is_active, device::motor_blowers) |
	       on(Controlling.dizel_heat.engine_is_hot, device::engine_hot) | on(Controlling.SpeedCtrl && Controlling.SpeedCtrlUnit.IsActive, device::speed_control);
}

std::uint8_t engine_of(TMoverParameters const &Mover) {
	switch (Mover.EngineType) {
	case TEngineType::ElectricSeriesMotor:
		return blackbox::engines::electric;
	case TEngineType::ElectricInductionMotor:
		return blackbox::engines::electric_induction;
	case TEngineType::DieselEngine:
		return blackbox::engines::diesel;
	case TEngineType::DieselElectric:
		return blackbox::engines::diesel_electric;
	case TEngineType::SteamEngine:
		return blackbox::engines::steam;
	default:
		return blackbox::engines::none;
	}
}

// the lights at an end of the train: of its head or of its tail, at the end of that vehicle turned away from the rest
std::uint16_t outward_lights(TDynamicObject const *Vehicle, bool const Head) {
	if (Vehicle == nullptr || Vehicle->MoverParameters == nullptr)
		return 0;
	auto const along{Vehicle->iDirection > 0};
	return static_cast<std::uint16_t>(Vehicle->MoverParameters->iLights[Head == along ? end::front : end::rear]);
}

// the lamps the vehicle has at that end: the driver checks a pattern against those of it there are
std::uint16_t outward_lamps(TDynamicObject const *Vehicle, bool const Head) {
	if (Vehicle == nullptr)
		return 0;
	auto const along{Vehicle->iDirection > 0};
	return static_cast<std::uint16_t>(Vehicle->LightList(Head == along ? end::front : end::rear));
}

// the scenario file without the starter's $, the folders and the extension, safe for a file name
std::string scenery_stem(std::string_view const Scenery) {
	auto stem{std::filesystem::path{Scenery}.stem().string()};
	if (false == stem.empty() && stem.front() == '$')
		stem.erase(0, 1);
	for (auto &c : stem)
		if (false == (std::isalnum(static_cast<unsigned char>(c)) || c == '-' || c == '_'))
			c = '_';
	return stem.empty() ? "scenery" : stem;
}

std::string local_time(char const *Format) {
	auto const now{std::time(nullptr)};
	std::tm local{};
#ifdef _WIN32
	localtime_s(&local, &now);
#else
	localtime_r(&now, &local);
#endif
	std::array<char, 32> text{};
	std::strftime(text.data(), text.size(), Format, &local);
	return text.data();
}

double second_of_day() {
	auto const &time{simulation::Time.data()};
	return time.wHour * 3600.0 + time.wMinute * 60.0 + simulation::Time.second();
}

// the points along a path, as many as the map of the simulator draws it with
std::vector<glm::vec3> points_of(TSegment const &Segment) {
	auto const steps{std::max(1, static_cast<int>(Segment.RaSegCount() * 0.5))};
	std::vector<glm::vec3> points;
	points.reserve(steps + 1);
	for (int i = 0; i <= steps; ++i)
		points.emplace_back(Segment.FastGetPoint(static_cast<double>(i) / steps));
	return points;
}

// the ways of a switch, one track record each, under the number of the switch
void write_switch(blackbox::writer &Writer, TTrack const &Switch, blackbox::track_data &Data) {
	Data.kind = blackbox::track_kinds::switch_branch;
	for (std::uint8_t branch = 0; branch < 2; ++branch) {
		auto const &segment{Switch.SwitchExtension->Segments[branch]};
		if (segment == nullptr)
			continue;
		Data.branch = branch;
		Data.points = points_of(*segment);
		Writer.track(Data);
	}
}

// the heading [rad, from +z towards +x] of a train going along the segment past the place given, from its first
// end to its second or the other way
float heading_along(TSegment const &Segment, glm::dvec3 const &Location, bool const Forward) {
	auto const steps{std::max(4, Segment.RaSegCount())};
	auto nearest{0};
	auto best{std::numeric_limits<double>::max()};
	for (int i = 0; i <= steps; ++i) {
		auto const distance{glm::distance(Segment.FastGetPoint(static_cast<double>(i) / steps), Location)};
		if (distance < best) {
			best = distance;
			nearest = i;
		}
	}
	auto const from{std::min(nearest, steps - 1)};
	auto way{Segment.FastGetPoint(static_cast<double>(from + 1) / steps) - Segment.FastGetPoint(static_cast<double>(from) / steps)};
	if (false == Forward)
		way = -way;
	return static_cast<float>(std::atan2(way.x, way.z));
}

// whether two vehicles are of one consist, coupled through whatever vehicles stand between them
bool coupled(TDynamicObject const *From, TDynamicObject const *To) {
	if (From == nullptr || To == nullptr)
		return false;
	if (From == To)
		return true;
	for (int start = 0; start < 2; ++start) {
		auto const *vehicle{From};
		auto side{start};
		// going on away from the first vehicle: through the coupler opposite the one each was reached by
		for (int step = 0; step < 1000 && vehicle != nullptr; ++step) {
			auto const *mover{vehicle->MoverParameters};
			auto const &neighbour{mover->Neighbours[side]};
			if (mover->Couplers[side].CouplingFlag == 0 || neighbour.vehicle == nullptr)
				break;
			if (neighbour.vehicle == To)
				return true;
			side = 1 - neighbour.vehicle_end;
			vehicle = neighbour.vehicle;
		}
	}
	return false;
}

blackbox::vehicle_state state_of(TDynamicObject const &Vehicle, std::uint32_t const Id) {
	blackbox::vehicle_state state;
	state.id = Id;
	state.position = glm::vec3{Vehicle.GetPosition()};
	auto const front{Vehicle.VectorFront()};
	state.heading = static_cast<float>(std::atan2(front.x, front.z));
	state.speed_kmh = static_cast<float>(Vehicle.MoverParameters->V * 3.6);
	state.driving = driver_of(Vehicle);
	return state;
}

} // namespace

void blackbox_recorder::update(double const Deltatime) {
	if (false == m_started) {
		if (false == simulation::is_ready)
			return;
		m_started = true;
		if (false == WriteLogFlag || false == Global.BlackBox || Global.editor_session)
			return;
		open();
	}
	if (m_writer == nullptr)
		return;

	std::scoped_lock lock{m_mutex};
	m_time += Deltatime;
	if (m_time >= m_cabdue) {
		write_cab();
		m_cabdue = std::max(m_cabdue + cabinterval, m_time);
	}
	if (m_time >= m_cameradue) {
		write_camera();
		m_cameradue = std::max(m_cameradue + camerainterval, m_time);
	}
	if (m_time >= m_lookdue) {
		auto const keyframe{m_time >= m_keyframedue};
		auto const lookstart{std::chrono::steady_clock::now()};
		look_around(keyframe);
		auto const looktime{std::chrono::steady_clock::now() - lookstart};
		m_looktime += looktime;
		m_looklongest = std::max(m_looklongest, looktime);
		++m_looks;
		if (keyframe)
			m_keyframedue = std::max(m_keyframedue + keyframeinterval, m_time);
		m_lookdue = std::max(m_lookdue + lookinterval, m_time);
		m_file.flush();
	}
}

void blackbox_recorder::close() {
	std::scoped_lock lock{m_mutex};
	m_started = true;
	if (m_writer == nullptr)
		return;
	m_writer->end(m_time);
	m_writer.reset();
	m_file.close();
	using milliseconds = std::chrono::duration<double, std::milli>;
	WriteLog(std::format("Black box: recording closed; {} looks around, {:.3f} ms each on average, the longest {:.3f} ms", m_looks,
	    m_looks > 0 ? milliseconds{m_looktime}.count() / m_looks : 0.0, milliseconds{m_looklongest}.count()));
}

void blackbox_recorder::open() {
	std::scoped_lock lock{m_mutex};

	std::error_code error;
	std::filesystem::create_directories("physicslog", error);
	auto const filename{std::format("physicslog/{}_{}.blackbox", scenery_stem(Global.SceneryFile), local_time("%Y-%m-%d_%H-%M-%S"))};
	m_file.open(filename, std::ios::out | std::ios::binary | std::ios::trunc);
	if (false == m_file.is_open()) {
		ErrorLog(std::format("Black box: can't create \"{}\"", filename));
		return;
	}
	// a buffer of its own, so the file is written to the disk in large pieces, mostly at the flush after a look;
	// given once the file is open and before anything is written: MSVC's file buffer passes over one given earlier
	m_file.rdbuf()->pubsetbuf(m_filebuffer.data(), static_cast<std::streamsize>(m_filebuffer.size()));
	m_writer = std::make_unique<blackbox::writer>(m_file);
	WriteLog(std::format("Black box: recording to \"{}\"", filename));

	auto const &time{simulation::Time.data()};
	m_writer->header();
	m_writer->info({text_of(Global.SceneryFile), local_time("%Y-%m-%dT%H:%M:%S"), static_cast<std::uint16_t>(time.wYear), static_cast<std::uint16_t>(simulation::Time.year_day()),
	    second_of_day(), text_of(Global.local_start_vehicle), text_of(Global.asVersion), static_cast<float>(cabinterval), static_cast<float>(lookinterval),
	    static_cast<float>(keyframeinterval)});
	write_map();
	for (auto *vehicle : simulation::Vehicles.sequence())
		if (vehicle->bEnabled)
			vehicle_id(vehicle);
	// the state at the start, the look listing all of it
	look_around(true);
	m_keyframedue = keyframeinterval;
	m_lookdue = lookinterval;
	m_cabdue = 0.0;
	m_file.flush();
}

// where the stop points read off the tracks are and which way the trains they are for go: the driver reads the events
// of the second end of a track going from its first end to the second, those of the first end going the other way
std::vector<std::pair<glm::dvec3, float>> blackbox_recorder::stop_point_headings() {
	std::vector<std::pair<glm::dvec3, float>> headings;
	for (auto *track : simulation::Paths.sequence()) {
		if (track->Segment == nullptr || track->m_editorremoved)
			continue;
		for (auto const forward : {true, false})
			for (auto const &[name, event] : (forward ? track->m_events2 : track->m_events1))
				if (event != nullptr && event->input_command() == TCommandType::cm_PassengerStopPoint)
					headings.emplace_back(event->input_location(), heading_along(*track->Segment, event->input_location(), forward));
	}
	return headings;
}

void blackbox_recorder::write_map() {
	std::uint32_t index{0};
	for (auto *track : simulation::Paths.sequence()) {
		if ((track->iCategoryFlag & 1) == 0 || track->m_editorremoved)
			continue;
		blackbox::track_data data;
		data.index = index;
		data.name = text_of(track->name());
		data.category = 1;
		if (track->eType == tt_Switch && track->SwitchExtension) {
			write_switch(*m_writer, *track, data);
			m_switches.emplace_back(track, index, -1);
		}
		else if (track->Segment) {
			data.points = points_of(*track->Segment);
			m_writer->track(data);
		}
		++index;
	}
	index = 0;
	for (auto const &object : map::Objects.entries) {
		auto const *semaphore{dynamic_cast<map::semaphore const *>(object.get())};
		if (semaphore == nullptr || semaphore->memcell == nullptr)
			continue;
		auto const location{semaphore->memcell->location()};
		m_writer->signal(index, text_of(semaphore->name), location);
		m_signals.emplace_back(semaphore->memcell, location);
		++index;
	}
	// the stop points (W4) and the horn points (W6): events the trains passing by read from the track, telling where
	// the station they are for is and to sound the horn. a stop point may be a memory cell too
	auto const headings{stop_point_headings()};
	auto const heading_of{[&headings](glm::dvec3 const &Location) {
		// the way the trains it is for go, unless it is on no track or on tracks of both ways
		auto heading{std::numeric_limits<float>::quiet_NaN()};
		for (auto const &[location, way] : headings) {
			if (glm::distance(location, Location) > 0.5)
				continue;
			if (std::isnan(heading))
				heading = way;
			else if (std::abs(std::remainder(heading - way, 2.0f * glm::pi<float>())) > 0.5f)
				return std::numeric_limits<float>::quiet_NaN();
		}
		return heading;
	}};
	constexpr std::string_view stoppoint{"PassengerStopPoint:"};
	std::uint32_t stoppoints{0};
	std::uint32_t hornpoints{0};
	for (auto const *event : simulation::Events.sequence()) {
		auto const *values{dynamic_cast<putvalues_event const *>(event)};
		if (values == nullptr)
			continue;
		auto const &text{values->input_text()};
		if (text.starts_with(stoppoint))
			m_writer->stop_point(stoppoints++, text_of(std::string_view{text}.substr(stoppoint.size())), values->input_location(),
			    static_cast<float>(values->input_value(1)), static_cast<float>(values->input_value(2)), heading_of(values->input_location()));
		else if (text == "Warning_signal")
			m_writer->horn_point(hornpoints++, text_of(values->name()), values->input_location(), static_cast<float>(values->input_value(1)),
			    static_cast<float>(values->input_value(2)));
	}
	for (auto const *cell : simulation::Memory.sequence()) {
		if (cell == nullptr || false == cell->Text().starts_with(stoppoint))
			continue;
		m_writer->stop_point(stoppoints++, text_of(std::string_view{cell->Text()}.substr(stoppoint.size())), cell->location(), static_cast<float>(cell->Value1()),
		    static_cast<float>(cell->Value2()), heading_of(cell->location()));
	}
}

void blackbox_recorder::write_cab() {
	auto *train{simulation::Train};
	if (train == nullptr || train->Dynamic() == nullptr)
		return;
	auto const *vehicle{train->Dynamic()};
	auto const *occupied{vehicle->MoverParameters};
	// the one which drives the consist, as the driving aid tells it
	auto const *owner{vehicle->ctOwner != nullptr ? vehicle->ctOwner : vehicle->Mechanik};
	auto const *controlling{owner != nullptr ? owner->Controlling() : occupied};

	blackbox::cab_sample sample;
	sample.time = m_time;
	sample.vehicle = vehicle_id(vehicle);
	sample.position = vehicle->GetPosition();
	sample.speed_kmh = static_cast<float>(occupied->V * 3.6);
	sample.acceleration = static_cast<float>(controlling->AccS);
	sample.lateral_acceleration = static_cast<float>(occupied->AccN);
	if (owner != nullptr && owner->is_train()) {
		auto const order{owner->order_current()};
		sample.movement = (order & Obey_train) != 0 ? blackbox::movements::train :
		                  (order & (Shunt | Loose_shunt | Connect)) != 0 ? blackbox::movements::shunting : blackbox::movements::other;
		sample.light_hint_head = static_cast<std::int16_t>(owner->light_hint(end::front));
		sample.light_hint_tail = static_cast<std::int16_t>(owner->light_hint(end::rear));
	}
	sample.odometer_km = static_cast<float>(occupied->DistCounter);
	sample.driving = driver_of(*vehicle);
	sample.flags = (FreeFlyModeFlag ? 0 : blackbox::cab_flags::in_cab) | (owner != nullptr && owner->is_active() ? blackbox::cab_flags::ai_valid : 0) |
	               (controlling->SecuritySystem.is_blinking() ? blackbox::cab_flags::alerter_blinking : 0) |
	               (controlling->SecuritySystem.is_beeping() ? blackbox::cab_flags::alerter_beeping : 0) | (controlling->SlippingWheels ? blackbox::cab_flags::wheel_slip : 0) |
	               (occupied->RadioStopFlag ? blackbox::cab_flags::radio_stop : 0) | (occupied->AlarmChainFlag ? blackbox::cab_flags::alarm_chain : 0);
	sample.main_controller = static_cast<std::int8_t>(controlling->MainCtrlPos);
	sample.scnd_controller = static_cast<std::int8_t>(controlling->ScndCtrlPos);
	sample.reverser = static_cast<std::int8_t>(controlling->DirActive);
	sample.train_brake = static_cast<float>(occupied->fBrakeCtrlPos);
	sample.local_brake = static_cast<float>(occupied->LocalBrakePosA);
	sample.brake_cylinder = static_cast<float>(occupied->BrakePress);
	sample.brake_pipe = static_cast<float>(occupied->PipePress);
	sample.motor_current = static_cast<float>(controlling->Im);
	if (owner != nullptr && owner->is_active())
		sample.ai = owner->ai_reference();
	sample.devices = devices_of(*occupied, *controlling);
	sample.main_reservoir = static_cast<float>(controlling->Compressor);
	sample.voltage = static_cast<float>(controlling->EngineVoltage);
	sample.engine_rpm = static_cast<float>(std::abs(controlling->enrot) * 60.0);
	// the lights at the ends of the train: of the vehicles at its ends, at their ends turned away from the rest
	sample.lights_head = outward_lights(owner != nullptr ? owner->Vehicle(end::front) : vehicle, true);
	sample.lights_tail = outward_lights(owner != nullptr ? owner->Vehicle(end::rear) : vehicle, false);
	sample.equipped_head = outward_lamps(owner != nullptr ? owner->Vehicle(end::front) : vehicle, true);
	sample.equipped_tail = outward_lamps(owner != nullptr ? owner->Vehicle(end::rear) : vehicle, false);
	auto const &security{controlling->SecuritySystem};
	namespace flag = blackbox::security_flags;
	sample.security = static_cast<std::uint8_t>((security.is_vigilance_blinking() ? flag::vigilance : 0) | (security.is_cabsignal_blinking() ? flag::cabsignal : 0) |
	                                            (security.is_beeping() ? flag::vigilance_beeping : 0) | (security.is_cabsignal_beeping() ? flag::cabsignal_beeping : 0) |
	                                            (security.is_braking() ? flag::braking : 0));
	sample.cab = static_cast<std::int8_t>(occupied->CabOccupied);
	// what drives the train, the setting of the brakes, the radio, the engine's oil and temperatures
	sample.engine = engine_of(*controlling);
	sample.brake_delay = static_cast<std::uint8_t>(occupied->BrakeDelayFlag);
	// the const one: with no driver the first channel
	sample.radio_channel = static_cast<std::uint8_t>(std::clamp(std::as_const(*train).RadioChannel(), 0, 255));
	sample.driver_valve = occupied->Handle != nullptr ? static_cast<float>(occupied->Handle->GetCP()) : 0.0f;
	sample.feed_pipe = static_cast<float>(occupied->ScndPipePress);
	sample.oil_pressure = controlling->OilPump.pressure;
	sample.water_temperature = controlling->dizel_heat.Twy;
	sample.oil_temperature = controlling->dizel_heat.To;
	sample.engine_temperature = controlling->dizel_heat.Ts;
	sample.speed_control_kmh = controlling->SpeedCtrl ? static_cast<float>(controlling->SpeedCtrlUnit.DesiredVelocity) : 0.0f;
	sample.engine_power = static_cast<float>(controlling->EnginePower);
	sample.hand_brake = static_cast<std::int8_t>(std::clamp(occupied->ManualBrakePos, -128, 127));
	sample.brake_mode = static_cast<std::uint8_t>(occupied->BrakeOpModeFlag);
	sample.horn = static_cast<std::uint8_t>(occupied->WarningSignal);
	sample.integrated_controller = static_cast<float>(controlling->eimic);
	sample.clutch = static_cast<float>(controlling->dizel_engagestate);
	sample.converter_fill = static_cast<float>(controlling->hydro_TC_Fill);
	// the brakes of the whole train: the highest of its cylinders, from the head to the tail
	auto brakes{occupied->BrakePress};
	for (auto const *each{owner != nullptr ? owner->Vehicle(end::front) : nullptr}; each != nullptr; each = each->Next())
		if (each->MoverParameters != nullptr)
			brakes = std::max(brakes, each->MoverParameters->BrakePress);
	sample.consist_brake_cylinder = static_cast<float>(brakes);
	m_writer->cab(sample);
}

void blackbox_recorder::look_around(bool const Keyframe) {
	auto const &time{simulation::Time.data()};
	m_writer->clock(m_time, static_cast<std::uint16_t>(time.wYear), static_cast<std::uint16_t>(simulation::Time.year_day()), second_of_day());

	auto *train{simulation::Train};
	auto const uservehicle{train != nullptr && train->Dynamic() != nullptr ? vehicle_id(train->Dynamic()) : blackbox::no_vehicle};
	auto const userincab{uservehicle != blackbox::no_vehicle && false == FreeFlyModeFlag};
	auto const usercab{uservehicle != blackbox::no_vehicle ? static_cast<std::int8_t>(train->Dynamic()->MoverParameters->CabOccupied) : std::int8_t{0}};
	if (false == m_userknown || uservehicle != m_uservehicle || userincab != m_userincab || usercab != m_usercab) {
		m_writer->user_vehicle(m_time, uservehicle, userincab, usercab);
		m_uservehicle = uservehicle;
		m_userincab = userincab;
		m_usercab = usercab;
		m_userknown = true;
	}
	if (uservehicle != blackbox::no_vehicle)
		look_at_timetable(*train->Dynamic(), uservehicle);

	// each part timed, to tell which one it was when a look takes long
	using clock = std::chrono::steady_clock;
	auto const start{clock::now()};
	look_at_vehicles(Keyframe);
	auto const vehicles{clock::now()};
	look_at_couplers();
	auto const couplers{clock::now()};
	look_at_switches();
	auto const switches{clock::now()};
	look_at_signals();
	auto const signals{clock::now()};
	using milliseconds = std::chrono::duration<double, std::milli>;
	if (milliseconds{signals - start}.count() > slowlook) {
		WriteLog(std::format("Black box: a look around at {:.1f} s took {:.1f} ms: vehicles {:.1f} ({}{}), couplers {:.1f}, switches {:.1f}, signals {:.1f}", m_time,
		    milliseconds{signals - start}.count(), milliseconds{vehicles - start}.count(), m_vehicles.size(), Keyframe ? ", keyframe" : "",
		    milliseconds{couplers - vehicles}.count(), milliseconds{switches - couplers}.count(), milliseconds{signals - switches}.count()));
	}
}

void blackbox_recorder::look_at_couplers() {
	for (auto *vehicle : simulation::Vehicles.sequence()) {
		if (false == vehicle->bEnabled)
			continue;
		auto &entry{m_vehicles[vehicle]};
		auto const *mover{vehicle->MoverParameters};
		for (int side = 0; side < 2; ++side) {
			auto const flags{mover->Couplers[side].CouplingFlag};
			// a neighbour not coupled is only seen on the same track
			auto const *other{flags != 0 ? mover->Neighbours[side].vehicle : nullptr};
			coupler_entry const now{other, other != nullptr ? mover->Neighbours[side].vehicle_end : 0, other != nullptr ? flags : 0};
			auto &last{entry.couplers[side]};
			auto const changed{now.other != last.other || now.other_side != last.other_side || now.flags != last.flags};
			auto const before{last};
			last = now;
			// at the first look only what is coupled; then each change
			if ((entry.couplers_known && false == changed) || (false == entry.couplers_known && now.other == nullptr))
				continue;
			// both vehicles see a coupling change: the one with the lower number tells it
			auto const *counterpart{now.other != nullptr ? now.other : before.other};
			if (counterpart != nullptr) {
				auto const found{m_vehicles.find(counterpart)};
				if (found != m_vehicles.end() && found->second.id < entry.id)
					continue;
			}
			blackbox::coupling_change change;
			change.time = m_time;
			change.vehicle = entry.id;
			change.side = static_cast<std::uint8_t>(side);
			change.other = known_id(now.other);
			change.other_side = static_cast<std::uint8_t>(now.other_side);
			change.flags = static_cast<std::uint8_t>(now.flags);
			change.previous_flags = static_cast<std::uint8_t>(entry.couplers_known ? before.flags : 0);
			change.previous_other = entry.couplers_known ? known_id(before.other) : blackbox::no_vehicle;
			change.position = vehicle->CouplerPosition(static_cast<end>(side));
			m_writer->coupling(change);
		}
		entry.couplers_known = true;
	}
}

void blackbox_recorder::camera(blackbox::camera_state Camera, TDynamicObject const *Owner) {
	std::scoped_lock lock{m_mutex};
	m_camera = Camera;
	m_cameraowner = Owner;
	m_cameraknown = true;
}

void blackbox_recorder::write_camera() {
	if (false == m_cameraknown)
		return;
	auto camera{m_camera};
	camera.time = m_time;
	camera.vehicle = known_id(m_cameraowner);
	auto const same{m_camerawrote && camera.mode == m_camerawritten.mode && camera.view == m_camerawritten.view && camera.vehicle == m_camerawritten.vehicle};
	// In the cab and in the views of the train the camera goes with the train, whose way is written
	// anyway: only a change of the view counts. The free camera is written as it moves and turns.
	auto const attached{camera.mode == blackbox::camera_modes::cab || camera.mode == blackbox::camera_modes::external};
	auto const turned{std::abs(std::remainder(camera.heading - m_camerawritten.heading, 2.0f * glm::pi<float>())) > cameraturn ||
	                  std::abs(camera.pitch - m_camerawritten.pitch) > cameraturn};
	if (same && (attached || (glm::distance(camera.position, m_camerawritten.position) <= cameramove && false == turned)))
		return;
	m_writer->camera(camera);
	m_camerawritten = camera;
	m_camerawrote = true;
}

void blackbox_recorder::look_at_vehicles(bool const Keyframe) {
	std::vector<blackbox::vehicle_state> states;
	for (auto *vehicle : simulation::Vehicles.sequence()) {
		if (false == vehicle->bEnabled)
			continue;
		auto const id{vehicle_id(vehicle)};
		auto &entry{m_vehicles[vehicle]};
		auto const state{state_of(*vehicle, id)};
		auto const changed{false == entry.listed || glm::distance(state.position, entry.last.position) > listmove || std::abs(state.speed_kmh - entry.last.speed_kmh) > listspeed ||
		                   state.driving != entry.last.driving};
		if (Keyframe || changed) {
			states.push_back(state);
			entry.last = state;
			entry.listed = true;
		}
	}
	if (Keyframe || false == states.empty())
		m_writer->vehicles(m_time, Keyframe, states);
}

void blackbox_recorder::look_at_switches() {
	for (auto &entry : m_switches) {
		auto const state{entry.track->GetSwitchState()};
		if (state == entry.state || state < 0)
			continue;
		entry.state = state;
		m_writer->switch_state(m_time, entry.index, static_cast<std::uint8_t>(state));
	}
}

void blackbox_recorder::look_at_signals() {
	for (std::size_t i = 0; i < m_signals.size(); ++i) {
		auto &entry{m_signals[i]};
		auto const speed1{static_cast<float>(entry.cell->Value1())};
		auto const speed2{static_cast<float>(entry.cell->Value2())};
		auto const &command{entry.cell->Text()};
		if (entry.known && speed1 == entry.speed1 && speed2 == entry.speed2 && command == entry.command)
			continue;
		entry.speed1 = speed1;
		entry.speed2 = speed2;
		entry.command = command;
		entry.known = true;
		m_writer->signal_aspect(m_time, static_cast<std::uint32_t>(i), speed1, speed2, text_of(command));
	}
}

void blackbox_recorder::look_at_timetable(TDynamicObject const &Vehicle, std::uint32_t const Id) {
	auto const *owner{Vehicle.ctOwner != nullptr ? Vehicle.ctOwner : Vehicle.Mechanik};
	if (owner == nullptr)
		return;
	auto const &timetable{owner->TrainTimetable()};
	// the user gone over to a vehicle of another consist: its timetable is written even if it has the same name, and so
	// is the one of the train the user comes back to; the other cab or a car of the same train keeps it
	if (false == coupled(&Vehicle, m_timetablevehicle)) {
		m_timetabletrain.clear();
		m_timetablestations = 0;
	}
	m_timetablevehicle = &Vehicle;
	// the timetable is cleared at the end of the route, where the train stays the one it was
	if (timetable.StationCount <= 0 || (timetable.TrainName == m_timetabletrain && timetable.StationCount == m_timetablestations))
		return;
	m_timetabletrain = timetable.TrainName;
	m_timetablestations = timetable.StationCount;
	blackbox::timetable_data data;
	data.time = m_time;
	data.vehicle = Id;
	data.train = text_of(timetable.TrainName);
	data.category = text_of(timetable.TrainCategory);
	data.from = text_of(timetable.Relation1);
	data.to = text_of(timetable.Relation2);
	data.vmax = static_cast<float>(timetable.TTVmax);
	auto const minutes{[](int const Hour, float const Minute) { return Hour >= 0 ? static_cast<float>(Hour * 60) + Minute : -1.0f; }};
	for (int i = 1; i <= timetable.StationCount; ++i) {
		auto const &line{timetable.TimeTable[i]};
		data.rows.push_back({text_of(line.StationName), line.km, line.vmax, minutes(line.Ah, line.Am), minutes(line.Dh, line.Dm), line.is_maintenance});
	}
	m_writer->timetable(data);
}

std::uint32_t blackbox_recorder::vehicle_id(TDynamicObject const *Vehicle) {
	if (auto const lookup{m_vehicles.find(Vehicle)}; lookup != m_vehicles.end())
		return lookup->second.id;

	auto &entry{m_vehicles[Vehicle]};
	entry.id = m_nextvehicle++;
	auto const *mover{Vehicle->MoverParameters};
	auto const *driver{Vehicle->Mechanik};
	auto const train{driver != nullptr && driver->TrainName() != "none" ? driver->TrainName() : std::string{}};
	m_writer->vehicle({entry.id, text_of(Vehicle->name()), text_of(mover->TypeName), text_of(train), static_cast<float>(mover->Dim.L), static_cast<float>(mover->Dim.W),
	    static_cast<std::uint8_t>(mover->CategoryFlag)});
	return entry.id;
}

std::uint32_t blackbox_recorder::known_id(TDynamicObject const *Vehicle) {
	// a vehicle taken off the scenery is gone from the recording: one still pointed at gets no new number
	return Vehicle != nullptr && Vehicle->bEnabled ? vehicle_id(Vehicle) : blackbox::no_vehicle;
}

std::uint32_t blackbox_recorder::signal_at(glm::dvec3 const &Location) const {
	auto const found{std::ranges::find_if(m_signals, [&Location](signal_entry const &Signal) { return glm::distance(Signal.location, Location) < 0.01; })};
	if (found == m_signals.end())
		return blackbox::no_signal;
	return static_cast<std::uint32_t>(std::distance(m_signals.begin(), found));
}

void blackbox_recorder::command(TDynamicObject const *Vehicle, user_command const Command, int const Action, double const Param1, double const Param2) {
	if (Action != GLFW_PRESS && Action != GLFW_RELEASE)
		return;
	std::scoped_lock lock{m_mutex};
	if (m_writer == nullptr)
		return;
	m_writer->command({m_time, Vehicle != nullptr ? vehicle_id(Vehicle) : blackbox::no_vehicle, Action == GLFW_PRESS, Param1, Param2,
	    simulation::Commands_descriptions[static_cast<std::size_t>(Command)].name});
}

void blackbox_recorder::driver_changed(TDynamicObject const *Vehicle, bool const Ai) {
	std::scoped_lock lock{m_mutex};
	if (m_writer == nullptr || Vehicle == nullptr)
		return;
	m_writer->driver_change(m_time, vehicle_id(Vehicle), Ai ? blackbox::driver::ai : blackbox::driver::human);
}

void blackbox_recorder::signal_passed(TDynamicObject const *Vehicle, basic_event const &Event, double const Speed1, double const Speed2, double const Trainspeed, double const Overrun,
    std::uint8_t const Flags) {
	std::scoped_lock lock{m_mutex};
	if (m_writer == nullptr || Vehicle == nullptr)
		return;
	auto const location{Event.input_location()};
	m_writer->signal_passed({m_time, vehicle_id(Vehicle), signal_at(location), text_of(Event.name()), location, static_cast<float>(Speed1), static_cast<float>(Speed2),
	    static_cast<float>(Trainspeed), static_cast<float>(Overrun), Flags});
}

void blackbox_recorder::station(TDynamicObject const *Vehicle, blackbox::station_event Station) {
	std::scoped_lock lock{m_mutex};
	if (m_writer == nullptr || Vehicle == nullptr)
		return;
	// a train given its next timetable may come to the first station of it at once, before the next look: the
	// timetable goes first, or the station would be read against the one before
	// the vehicle the user is in now, which may have changed since the last look
	if (auto const *train{simulation::Train}; m_userknown && train != nullptr && train->Dynamic() != nullptr)
		look_at_timetable(*train->Dynamic(), vehicle_id(train->Dynamic()));
	Station.time = m_time;
	Station.vehicle = vehicle_id(Vehicle);
	Station.station = text_of(Station.station);
	m_writer->station(Station);
}

void blackbox_recorder::horn_requested(TDynamicObject const *Vehicle, double const Duration, double const Tone, glm::dvec3 const &Position) {
	std::scoped_lock lock{m_mutex};
	if (m_writer == nullptr || Vehicle == nullptr)
		return;
	m_writer->horn_request(m_time, vehicle_id(Vehicle), static_cast<float>(Duration), static_cast<float>(Tone), Position);
}

void blackbox_recorder::vehicle_removed(TDynamicObject const *Vehicle) {
	std::scoped_lock lock{m_mutex};
	if (Vehicle == m_timetablevehicle)
		m_timetablevehicle = nullptr;
	if (Vehicle == m_cameraowner)
		m_cameraowner = nullptr;
	auto const lookup{m_vehicles.find(Vehicle)};
	if (lookup == m_vehicles.end())
		return;
	if (m_writer != nullptr)
		m_writer->vehicle_removed(m_time, lookup->second.id);
	// the memory may be given to another vehicle later, which is to get a number of its own
	m_vehicles.erase(lookup);
}

} // namespace simulation
