/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/blackbox.h"

#include <bit>
#include <cstring>
#include <ostream>

namespace blackbox {

// the file is little-endian, written the way the numbers lie in memory
static_assert(std::endian::native == std::endian::little, "the black box writer assumes a little-endian machine");

template <typename Type_>
void writer::payload::raw(Type_ const Value) {
	std::array<char, sizeof(Type_)> bytes;
	std::memcpy(bytes.data(), &Value, sizeof(Type_));
	m_bytes.append(bytes.data(), bytes.size());
}

void writer::payload::u8(std::uint8_t const Value) {
	raw(Value); }

void writer::payload::i8(std::int8_t const Value) {
	raw(Value); }

void writer::payload::u16(std::uint16_t const Value) {
	raw(Value); }

void writer::payload::u32(std::uint32_t const Value) {
	raw(Value); }

void writer::payload::f32(float const Value) {
	raw(Value); }

void writer::payload::f64(double const Value) {
	raw(Value); }

void writer::payload::vec3(glm::vec3 const &Value) {
	f32(Value.x);
	f32(Value.y);
	f32(Value.z);
}

void writer::payload::dvec3(glm::dvec3 const &Value) {
	f64(Value.x);
	f64(Value.y);
	f64(Value.z);
}

void writer::payload::text(std::string_view const Value) {
	auto length{static_cast<std::uint32_t>(Value.size())};
	while (length >= 0x80) {
		u8(static_cast<std::uint8_t>(length | 0x80));
		length >>= 7;
	}
	u8(static_cast<std::uint8_t>(length));
	m_bytes.append(Value);
}

writer::writer(std::ostream &Output) : m_output{Output} {}

writer::payload &writer::begin() {
	m_payload.clear();
	return m_payload;
}

void writer::emit(record const Type) {
	auto const &bytes{m_payload.bytes()};
	std::array<char, record_header_size> head;
	head[0] = static_cast<char>(Type);
	auto const length{static_cast<std::uint32_t>(bytes.size())};
	std::memcpy(head.data() + 1, &length, sizeof(length));
	m_output.write(head.data(), static_cast<std::streamsize>(head.size()));
	m_output.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
	m_size += head.size() + bytes.size();
}

void writer::header() {
	std::array<char, header_size> head{};
	std::memcpy(head.data(), magic.data(), magic.size());
	std::memcpy(head.data() + magic.size(), &format_version, sizeof(format_version));
	// two bytes reserved, zero
	m_output.write(head.data(), static_cast<std::streamsize>(head.size()));
	m_size += head.size();
}

void writer::info(info_data const &Info) {
	auto &out{begin()};
	out.text(Info.scenery);
	out.text(Info.started);
	out.u16(Info.year);
	out.u16(Info.year_day);
	out.f64(Info.second_of_day);
	out.text(Info.start_vehicle);
	out.text(Info.simulator_version);
	out.f32(Info.cab_interval);
	out.f32(Info.vehicles_interval);
	out.f32(Info.keyframe_interval);
	emit(record::info);
}

void writer::track(track_data const &Track) {
	auto &out{begin()};
	out.u32(Track.index);
	out.text(Track.name);
	out.u8(Track.kind);
	out.u8(Track.branch);
	out.u8(Track.category);
	out.u32(static_cast<std::uint32_t>(Track.points.size()));
	for (auto const &point : Track.points)
		out.vec3(point);
	emit(record::track);
}

void writer::signal(std::uint32_t const Index, std::string_view const Name, glm::dvec3 const &Position) {
	auto &out{begin()};
	out.u32(Index);
	out.text(Name);
	out.dvec3(Position);
	emit(record::signal);
}

void writer::vehicle(vehicle_data const &Vehicle) {
	auto &out{begin()};
	out.u32(Vehicle.id);
	out.text(Vehicle.name);
	out.text(Vehicle.type);
	out.text(Vehicle.train);
	out.f32(Vehicle.length);
	out.f32(Vehicle.width);
	out.u8(Vehicle.category);
	emit(record::vehicle);
}

void writer::vehicle_removed(double const Time, std::uint32_t const Id) {
	auto &out{begin()};
	out.f64(Time);
	out.u32(Id);
	emit(record::vehicle_removed);
}

void writer::cab(cab_sample const &Sample) {
	auto &out{begin()};
	out.f64(Sample.time);
	out.u32(Sample.vehicle);
	out.dvec3(Sample.position);
	out.f32(Sample.speed_kmh);
	out.f32(Sample.acceleration);
	out.f32(Sample.odometer_km);
	out.u8(static_cast<std::uint8_t>(Sample.driving));
	out.u8(Sample.flags);
	out.i8(Sample.main_controller);
	out.i8(Sample.scnd_controller);
	out.i8(Sample.reverser);
	out.f32(Sample.train_brake);
	out.f32(Sample.local_brake);
	out.f32(Sample.brake_cylinder);
	out.f32(Sample.brake_pipe);
	out.f32(Sample.motor_current);
	out.f32(Sample.ai.target_kmh);
	out.f32(Sample.ai.next_limit_kmh);
	out.f32(Sample.ai.next_limit_distance);
	out.f32(Sample.ai.acceleration);
	out.f32(Sample.ai.braking_distance);
	out.f32(Sample.ai.timetable_vmax_kmh);
	out.f32(Sample.ai.track_vmax_kmh);
	out.u32(Sample.devices);
	out.f32(Sample.main_reservoir);
	out.f32(Sample.voltage);
	out.f32(Sample.engine_rpm);
	out.u16(Sample.lights_head);
	out.u16(Sample.lights_tail);
	out.i8(Sample.cab);
	out.u8(Sample.security);
	out.f32(Sample.lateral_acceleration);
	out.u8(Sample.movement);
	out.u16(static_cast<std::uint16_t>(Sample.light_hint_head));
	out.u16(static_cast<std::uint16_t>(Sample.light_hint_tail));
	out.u16(Sample.equipped_head);
	out.u16(Sample.equipped_tail);
	out.u8(Sample.engine);
	out.u8(Sample.brake_delay);
	out.u8(Sample.radio_channel);
	out.f32(Sample.driver_valve);
	out.f32(Sample.feed_pipe);
	out.f32(Sample.oil_pressure);
	out.f32(Sample.water_temperature);
	out.f32(Sample.oil_temperature);
	out.f32(Sample.engine_temperature);
	out.f32(Sample.speed_control_kmh);
	out.f32(Sample.engine_power);
	out.i8(Sample.hand_brake);
	out.u8(Sample.brake_mode);
	out.u8(Sample.horn);
	out.f32(Sample.integrated_controller);
	out.f32(Sample.clutch);
	out.f32(Sample.converter_fill);
	out.f32(Sample.consist_brake_cylinder);
	emit(record::cab);
}

void writer::vehicles(double const Time, bool const Keyframe, std::vector<vehicle_state> const &States) {
	auto &out{begin()};
	out.f64(Time);
	out.u8(Keyframe ? 1 : 0);
	out.u32(static_cast<std::uint32_t>(States.size()));
	for (auto const &state : States) {
		out.u32(state.id);
		out.vec3(state.position);
		out.f32(state.heading);
		out.f32(state.speed_kmh);
		out.u8(static_cast<std::uint8_t>(state.driving));
	}
	emit(record::vehicles);
}

void writer::switch_state(double const Time, std::uint32_t const Track, std::uint8_t const State) {
	auto &out{begin()};
	out.f64(Time);
	out.u32(Track);
	out.u8(State);
	emit(record::switch_state);
}

void writer::signal_aspect(double const Time, std::uint32_t const Signal, float const Speed1, float const Speed2, std::string_view const Command) {
	auto &out{begin()};
	out.f64(Time);
	out.u32(Signal);
	out.f32(Speed1);
	out.f32(Speed2);
	out.text(Command);
	emit(record::signal_aspect);
}

void writer::command(command_entry const &Command) {
	auto &out{begin()};
	out.f64(Command.time);
	out.u32(Command.vehicle);
	out.u8(Command.pressed ? 1 : 0);
	out.f64(Command.param1);
	out.f64(Command.param2);
	out.text(Command.name);
	emit(record::command);
}

void writer::driver_change(double const Time, std::uint32_t const Vehicle, driver const Driving) {
	auto &out{begin()};
	out.f64(Time);
	out.u32(Vehicle);
	out.u8(static_cast<std::uint8_t>(Driving));
	emit(record::driver_change);
}

void writer::signal_passed(signal_passing const &Passing) {
	auto &out{begin()};
	out.f64(Passing.time);
	out.u32(Passing.vehicle);
	out.u32(Passing.signal);
	out.text(Passing.event);
	out.dvec3(Passing.position);
	out.f32(Passing.speed1);
	out.f32(Passing.speed2);
	out.f32(Passing.speed_kmh);
	out.f32(Passing.overrun);
	out.u8(Passing.flags);
	emit(record::signal_passed);
}

void writer::user_vehicle(double const Time, std::uint32_t const Vehicle, bool const In_cab, std::int8_t const Cab) {
	auto &out{begin()};
	out.f64(Time);
	out.u32(Vehicle);
	out.u8(In_cab ? 1 : 0);
	out.i8(Cab);
	emit(record::user_vehicle);
}

void writer::stop_point(std::uint32_t const Index, std::string_view const Station, glm::dvec3 const &Position, float const Stop_at, float const Platform,
    float const Heading) {
	auto &out{begin()};
	out.u32(Index);
	out.text(Station);
	out.dvec3(Position);
	out.f32(Stop_at);
	out.f32(Platform);
	out.f32(Heading);
	emit(record::stop_point);
}

void writer::horn_point(std::uint32_t const Index, std::string_view const Name, glm::dvec3 const &Position, float const Duration, float const Tone) {
	auto &out{begin()};
	out.u32(Index);
	out.text(Name);
	out.dvec3(Position);
	out.f32(Duration);
	out.f32(Tone);
	emit(record::horn_point);
}

void writer::timetable(timetable_data const &Timetable) {
	auto &out{begin()};
	out.f64(Timetable.time);
	out.u32(Timetable.vehicle);
	out.text(Timetable.train);
	out.text(Timetable.category);
	out.text(Timetable.from);
	out.text(Timetable.to);
	out.f32(Timetable.vmax);
	out.u32(static_cast<std::uint32_t>(Timetable.rows.size()));
	for (auto const &row : Timetable.rows) {
		out.text(row.station);
		out.f32(row.km);
		out.f32(row.vmax);
		out.f32(row.arrival);
		out.f32(row.departure);
		out.u8(row.maintenance ? 1 : 0);
	}
	emit(record::timetable);
}

void writer::station(station_event const &Station) {
	auto &out{begin()};
	out.f64(Station.time);
	out.u32(Station.vehicle);
	out.u8(Station.kind);
	out.u16(Station.row);
	out.text(Station.station);
	out.dvec3(Station.position);
	out.f32(Station.distance);
	out.f32(Station.stop_at);
	out.f32(Station.train_length);
	out.f32(Station.platform);
	out.f32(Station.arrival);
	out.f32(Station.departure);
	emit(record::station);
}

void writer::horn_request(double const Time, std::uint32_t const Vehicle, float const Duration, float const Tone, glm::dvec3 const &Position) {
	auto &out{begin()};
	out.f64(Time);
	out.u32(Vehicle);
	out.f32(Duration);
	out.f32(Tone);
	out.dvec3(Position);
	emit(record::horn_request);
}

void writer::camera(camera_state const &Camera) {
	auto &out{begin()};
	out.f64(Camera.time);
	out.u8(Camera.mode);
	out.u8(Camera.view);
	out.u32(Camera.vehicle);
	out.dvec3(Camera.position);
	out.f32(Camera.heading);
	out.f32(Camera.pitch);
	emit(record::camera);
}

void writer::coupling(coupling_change const &Change) {
	auto &out{begin()};
	out.f64(Change.time);
	out.u32(Change.vehicle);
	out.u8(Change.side);
	out.u32(Change.other);
	out.u8(Change.other_side);
	out.u8(Change.flags);
	out.u8(Change.previous_flags);
	out.u32(Change.previous_other);
	out.dvec3(Change.position);
	emit(record::coupling);
}

void writer::clock(double const Time, std::uint16_t const Year, std::uint16_t const Year_day, double const Second_of_day) {
	auto &out{begin()};
	out.f64(Time);
	out.u16(Year);
	out.u16(Year_day);
	out.f64(Second_of_day);
	emit(record::clock);
}

void writer::end(double const Time) {
	auto &out{begin()};
	out.f64(Time);
	emit(record::end);
}

} // namespace blackbox
