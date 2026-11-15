#include "stdafx.h"
#include "doctest.h"

#include "vehicle/blackbox.h"

#include <cstring>
#include <sstream>
#include <string_view>

// the black box is read by other programs (the starter, a viewer), from the layout in blackbox_doc.md alone.
// these read what the writer wrote the way such a reader would, so a change of the layout shows up here

namespace {

// reads the numbers the way a .NET BinaryReader does: little-endian, texts after their 7 bit length
class reader {
public:
	explicit reader(std::string_view Data) : m_data{Data} {}

	template <typename Type_>
	Type_ get() {
		REQUIRE(m_position + sizeof(Type_) <= m_data.size());
		Type_ value;
		std::memcpy(&value, m_data.data() + m_position, sizeof(Type_));
		m_position += sizeof(Type_);
		return value;
	}

	std::string text() {
		std::uint32_t length{0};
		int shift{0};
		std::uint8_t part;
		do {
			part = get<std::uint8_t>();
			length |= static_cast<std::uint32_t>(part & 0x7f) << shift;
			shift += 7;
		} while ((part & 0x80) != 0);
		REQUIRE(m_position + length <= m_data.size());
		std::string value{m_data.substr(m_position, length)};
		m_position += length;
		return value;
	}

	std::size_t position() const {
		return m_position; }
	bool done() const {
		return m_position == m_data.size(); }

private:
	std::string_view m_data;
	std::size_t m_position{0};
};

struct record_view {
	blackbox::record type;
	std::string payload;
};

// splits the file into its records, checking the header and that the lengths add up to the whole file
std::vector<record_view> records(std::string const &File) {
	reader file{File};
	REQUIRE(File.size() >= blackbox::header_size);
	CHECK(std::string_view{File.data(), 8} == std::string_view{blackbox::magic.data(), 8});
	file.get<std::array<char, 8>>();
	CHECK(file.get<std::uint16_t>() == blackbox::format_version);
	CHECK(file.get<std::uint16_t>() == 0);
	std::vector<record_view> result;
	while (false == file.done()) {
		auto const type{file.get<std::uint8_t>()};
		auto const length{file.get<std::uint32_t>()};
		REQUIRE(file.position() + length <= File.size());
		result.push_back({static_cast<blackbox::record>(type), File.substr(file.position(), length)});
		for (std::uint32_t i = 0; i < length; ++i)
			file.get<std::uint8_t>();
	}
	return result;
}

blackbox::cab_sample sample() {
	blackbox::cab_sample s;
	s.time = 12.5;
	s.vehicle = 3;
	s.position = {-1234.5, 87.25, 40321.125};
	s.speed_kmh = -42.5f;
	s.acceleration = 0.25f;
	s.odometer_km = 17.75f;
	s.driving = blackbox::driver::human;
	s.flags = blackbox::cab_flags::in_cab | blackbox::cab_flags::ai_valid;
	s.main_controller = 7;
	s.scnd_controller = 2;
	s.reverser = -1;
	s.train_brake = 1.5f;
	s.local_brake = 0.5f;
	s.brake_cylinder = 3.75f;
	s.brake_pipe = 4.5f;
	s.motor_current = 640.0f;
	s.ai = {60.0f, 40.0f, 812.0f, -0.35f, 455.0f, 100.0f, 80.0f};
	s.devices = blackbox::cab_devices::battery | blackbox::cab_devices::pantograph_rear | blackbox::cab_devices::shunt_mode;
	s.main_reservoir = 8.25f;
	s.voltage = 3000.0f;
	s.engine_rpm = 1250.0f;
	s.lights_head = 0x15;
	s.lights_tail = 0x22;
	s.cab = -1;
	s.security = blackbox::security_flags::vigilance | blackbox::security_flags::braking;
	s.lateral_acceleration = -0.75f;
	s.movement = blackbox::movements::shunting;
	s.light_hint_head = -1;
	s.light_hint_tail = 0x22;
	s.equipped_head = 0x1B5;
	s.equipped_tail = 0x22;
	s.engine = blackbox::engines::diesel_electric;
	s.brake_delay = 2;
	s.radio_channel = 3;
	s.driver_valve = 4.75f;
	s.feed_pipe = 8.5f;
	s.oil_pressure = 0.5f;
	s.water_temperature = 78.5f;
	s.oil_temperature = 70.25f;
	s.engine_temperature = 81.0f;
	s.speed_control_kmh = 90.0f;
	s.engine_power = 1250.5f;
	s.hand_brake = 5;
	s.brake_mode = 4;
	s.horn = 2;
	s.integrated_controller = -0.5f;
	s.clutch = 0.75f;
	s.converter_fill = 0.25f;
	s.consist_brake_cylinder = 3.5f;
	return s;
}

} // namespace

TEST_CASE("blackbox: the file starts with the magic and the version, then come records which say how long they are") {
	std::ostringstream file;
	blackbox::writer out{file};
	out.header();
	out.info({"$td.scn", "2026-11-08T15:32:58", 2026, 312, 8 * 3600.0, "ET22-1", "26.1", 0.1f, 1.0f, 60.0f});
	out.track({0, "tor1", blackbox::track_kinds::normal, 0, 1, {{0.f, 0.f, 0.f}, {10.f, 0.5f, 100.f}}});
	out.signal(0, "sem_a", {10.0, 0.0, 100.0});
	out.vehicle({1, "et22-1", "ET22", "12345", 19.24f, 3.0f, 1});
	out.cab(sample());
	out.vehicles(13.0, true, {{1, {1.f, 2.f, 3.f}, 0.5f, 60.f, blackbox::driver::ai}});
	out.switch_state(13.0, 0, 1);
	out.signal_aspect(13.0, 0, 0.0f, 40.0f, "SetVelocity");
	out.command({13.5, 1, true, 1.0, 0.0, "mastercontrollerincrease"});
	out.driver_change(14.0, 1, blackbox::driver::ai);
	out.signal_passed({14.5, 1, 0, "sem_a_sem", {10.0, 0.0, 100.0}, 0.0f, 40.0f, 35.0f, 2.5f, blackbox::passing_flags::at_stop});
	out.user_vehicle(15.0, 1, false, -1);
	out.clock(15.0, 2026, 312, 8 * 3600.0 + 15.0);
	out.vehicle_removed(16.0, 1);
	out.end(16.0);

	auto const file_bytes{file.str()};
	CHECK(file_bytes.size() == out.size());
	auto const all{records(file_bytes)};

	using blackbox::record;
	std::vector<record> const expected{record::info, record::track, record::signal, record::vehicle, record::cab, record::vehicles, record::switch_state, record::signal_aspect,
	    record::command, record::driver_change, record::signal_passed, record::user_vehicle, record::clock, record::vehicle_removed, record::end};
	REQUIRE(all.size() == expected.size());
	for (std::size_t i = 0; i < all.size(); ++i)
		CHECK(all[i].type == expected[i]);

	SUBCASE("the info") {
		reader info{all[0].payload};
		CHECK(info.text() == "$td.scn");
		CHECK(info.text() == "2026-11-08T15:32:58");
		CHECK(info.get<std::uint16_t>() == 2026);
		CHECK(info.get<std::uint16_t>() == 312);
		CHECK(info.get<double>() == 8 * 3600.0);
		CHECK(info.text() == "ET22-1");
		CHECK(info.text() == "26.1");
		CHECK(info.get<float>() == 0.1f);
		CHECK(info.get<float>() == 1.0f);
		CHECK(info.get<float>() == 60.0f);
		CHECK(info.done());
	}
	SUBCASE("a track: its points after their count") {
		reader track{all[1].payload};
		CHECK(track.get<std::uint32_t>() == 0);
		CHECK(track.text() == "tor1");
		CHECK(track.get<std::uint8_t>() == blackbox::track_kinds::normal);
		CHECK(track.get<std::uint8_t>() == 0);
		CHECK(track.get<std::uint8_t>() == 1);
		CHECK(track.get<std::uint32_t>() == 2);
		for (int i = 0; i < 3; ++i)
			CHECK(track.get<float>() == 0.f);
		CHECK(track.get<float>() == 10.f);
		CHECK(track.get<float>() == 0.5f);
		CHECK(track.get<float>() == 100.f);
		CHECK(track.done());
	}
	SUBCASE("a passed signal") {
		reader passed{all[10].payload};
		CHECK(passed.get<double>() == 14.5);
		CHECK(passed.get<std::uint32_t>() == 1);
		CHECK(passed.get<std::uint32_t>() == 0);
		CHECK(passed.text() == "sem_a_sem");
		CHECK(passed.get<double>() == 10.0);
		CHECK(passed.get<double>() == 0.0);
		CHECK(passed.get<double>() == 100.0);
		CHECK(passed.get<float>() == 0.0f);
		CHECK(passed.get<float>() == 40.0f);
		CHECK(passed.get<float>() == 35.0f);
		CHECK(passed.get<float>() == 2.5f);
		CHECK(passed.get<std::uint8_t>() == blackbox::passing_flags::at_stop);
		CHECK(passed.done());
	}
}

TEST_CASE("blackbox: a sample of the user's train has the documented size and order") {
	std::ostringstream file;
	blackbox::writer out{file};
	out.header();
	out.cab(sample());
	auto const all{records(file.str())};
	REQUIRE(all.size() == 1);
	REQUIRE(all[0].payload.size() == blackbox::cab_sample_size);

	reader cab{all[0].payload};
	CHECK(cab.get<double>() == 12.5);
	CHECK(cab.get<std::uint32_t>() == 3);
	CHECK(cab.get<double>() == -1234.5);
	CHECK(cab.get<double>() == 87.25);
	CHECK(cab.get<double>() == 40321.125);
	CHECK(cab.get<float>() == -42.5f);
	CHECK(cab.get<float>() == 0.25f);
	CHECK(cab.get<float>() == 17.75f);
	CHECK(cab.get<std::uint8_t>() == static_cast<std::uint8_t>(blackbox::driver::human));
	CHECK(cab.get<std::uint8_t>() == (blackbox::cab_flags::in_cab | blackbox::cab_flags::ai_valid));
	CHECK(cab.get<std::int8_t>() == 7);
	CHECK(cab.get<std::int8_t>() == 2);
	CHECK(cab.get<std::int8_t>() == -1);
	CHECK(cab.get<float>() == 1.5f);
	CHECK(cab.get<float>() == 0.5f);
	CHECK(cab.get<float>() == 3.75f);
	CHECK(cab.get<float>() == 4.5f);
	CHECK(cab.get<float>() == 640.0f);
	// what the AI would do
	CHECK(cab.get<float>() == 60.0f);
	CHECK(cab.get<float>() == 40.0f);
	CHECK(cab.get<float>() == 812.0f);
	CHECK(cab.get<float>() == -0.35f);
	CHECK(cab.get<float>() == 455.0f);
	CHECK(cab.get<float>() == 100.0f);
	CHECK(cab.get<float>() == 80.0f);
	// what is switched on
	CHECK(cab.get<std::uint32_t>() == 0x8009);
	CHECK(cab.get<float>() == 8.25f);
	CHECK(cab.get<float>() == 3000.0f);
	CHECK(cab.get<float>() == 1250.0f);
	CHECK(cab.get<std::uint16_t>() == 0x15);
	CHECK(cab.get<std::uint16_t>() == 0x22);
	// the cab, the train protection and the lateral acceleration
	CHECK(cab.get<std::int8_t>() == -1);
	CHECK(cab.get<std::uint8_t>() == 0x11);
	CHECK(cab.get<float>() == -0.75f);
	// what the driver goes by, and the lights asked for at the head and the tail
	CHECK(cab.get<std::uint8_t>() == blackbox::movements::shunting);
	CHECK(cab.get<std::int16_t>() == -1);
	CHECK(cab.get<std::int16_t>() == 0x22);
	// the lamps there are at the head and the tail
	CHECK(cab.get<std::uint16_t>() == 0x1B5);
	CHECK(cab.get<std::uint16_t>() == 0x22);
	// the engine, the brakes' setting, the radio, the pressures, the oil and the temperatures, the cruise control, the power
	CHECK(cab.get<std::uint8_t>() == blackbox::engines::diesel_electric);
	CHECK(cab.get<std::uint8_t>() == 2);
	CHECK(cab.get<std::uint8_t>() == 3);
	CHECK(cab.get<float>() == 4.75f);
	CHECK(cab.get<float>() == 8.5f);
	CHECK(cab.get<float>() == 0.5f);
	CHECK(cab.get<float>() == 78.5f);
	CHECK(cab.get<float>() == 70.25f);
	CHECK(cab.get<float>() == 81.0f);
	CHECK(cab.get<float>() == 90.0f);
	CHECK(cab.get<float>() == 1250.5f);
	// the hand brake, the mode of the brakes, the horn, the integrated controller, the clutch, the torque converter
	CHECK(cab.get<std::int8_t>() == 5);
	CHECK(cab.get<std::uint8_t>() == 4);
	CHECK(cab.get<std::uint8_t>() == 2);
	CHECK(cab.get<float>() == -0.5f);
	CHECK(cab.get<float>() == 0.75f);
	CHECK(cab.get<float>() == 0.25f);
	// the highest brake cylinder pressure of the train
	CHECK(cab.get<float>() == 3.5f);
	CHECK(cab.done());
}

TEST_CASE("blackbox: the stations, the stop points and the horn points are written field by field") {
	std::ostringstream file;
	blackbox::writer out{file};
	out.header();
	out.stop_point(2, "Gdynia Gl.", {100.0, 1.5, -200.0}, -15.0f, 250.0f, 1.5f);
	out.horn_point(0, "w6_most", {10.0, 0.0, 20.0}, 2.0f, 1.0f);
	out.timetable({30.0, 4, "61110", "MOJ", "Gdynia", "Hel", 120.0f, {{"Gdynia", 0.0f, -1.0f, -1.0f, 10 * 60 + 5.0f, false}, {"Reda", 21.5f, 100.0f, 10 * 60 + 22.0f, 10 * 60 + 23.0f, true}}});
	out.station({40.5, 4, blackbox::station_kinds::stopped, 2, "Reda", {100.0, 1.5, -200.0}, 3.25f, 15.0f, 66.5f, 250.0f, 10 * 60 + 22.0f, 10 * 60 + 23.0f});
	out.horn_request(41.0, 4, 2.0f, 1.0f, {10.0, 0.0, 20.0});
	out.user_vehicle(42.0, 5, true, 1);
	auto const all{records(file.str())};
	using blackbox::record;
	REQUIRE(all.size() == 6);
	CHECK(all[0].type == record::stop_point);
	CHECK(all[1].type == record::horn_point);
	CHECK(all[2].type == record::timetable);
	CHECK(all[3].type == record::station);
	CHECK(all[4].type == record::horn_request);
	CHECK(all[5].type == record::user_vehicle);

	SUBCASE("a stop point") {
		reader point{all[0].payload};
		CHECK(point.get<std::uint32_t>() == 2);
		CHECK(point.text() == "Gdynia Gl.");
		CHECK(point.get<double>() == 100.0);
		CHECK(point.get<double>() == 1.5);
		CHECK(point.get<double>() == -200.0);
		CHECK(point.get<float>() == -15.0f);
		CHECK(point.get<float>() == 250.0f);
		CHECK(point.get<float>() == 1.5f);
		CHECK(point.done());
	}
	SUBCASE("a horn point") {
		reader point{all[1].payload};
		CHECK(point.get<std::uint32_t>() == 0);
		CHECK(point.text() == "w6_most");
		CHECK(point.get<double>() == 10.0);
		CHECK(point.get<double>() == 0.0);
		CHECK(point.get<double>() == 20.0);
		CHECK(point.get<float>() == 2.0f);
		CHECK(point.get<float>() == 1.0f);
		CHECK(point.done());
	}
	SUBCASE("a timetable: its rows after their count") {
		reader timetable{all[2].payload};
		CHECK(timetable.get<double>() == 30.0);
		CHECK(timetable.get<std::uint32_t>() == 4);
		CHECK(timetable.text() == "61110");
		CHECK(timetable.text() == "MOJ");
		CHECK(timetable.text() == "Gdynia");
		CHECK(timetable.text() == "Hel");
		CHECK(timetable.get<float>() == 120.0f);
		CHECK(timetable.get<std::uint32_t>() == 2);
		CHECK(timetable.text() == "Gdynia");
		CHECK(timetable.get<float>() == 0.0f);
		CHECK(timetable.get<float>() == -1.0f);
		CHECK(timetable.get<float>() == -1.0f);
		CHECK(timetable.get<float>() == 605.0f);
		CHECK(timetable.get<std::uint8_t>() == 0);
		CHECK(timetable.text() == "Reda");
		CHECK(timetable.get<float>() == 21.5f);
		CHECK(timetable.get<float>() == 100.0f);
		CHECK(timetable.get<float>() == 622.0f);
		CHECK(timetable.get<float>() == 623.0f);
		CHECK(timetable.get<std::uint8_t>() == 1);
		CHECK(timetable.done());
	}
	SUBCASE("a train at a station") {
		reader station{all[3].payload};
		CHECK(station.get<double>() == 40.5);
		CHECK(station.get<std::uint32_t>() == 4);
		CHECK(station.get<std::uint8_t>() == blackbox::station_kinds::stopped);
		CHECK(station.get<std::uint16_t>() == 2);
		CHECK(station.text() == "Reda");
		CHECK(station.get<double>() == 100.0);
		CHECK(station.get<double>() == 1.5);
		CHECK(station.get<double>() == -200.0);
		CHECK(station.get<float>() == 3.25f);
		CHECK(station.get<float>() == 15.0f);
		CHECK(station.get<float>() == 66.5f);
		CHECK(station.get<float>() == 250.0f);
		CHECK(station.get<float>() == 622.0f);
		CHECK(station.get<float>() == 623.0f);
		CHECK(station.done());
	}
	SUBCASE("a horn asked for") {
		reader horn{all[4].payload};
		CHECK(horn.get<double>() == 41.0);
		CHECK(horn.get<std::uint32_t>() == 4);
		CHECK(horn.get<float>() == 2.0f);
		CHECK(horn.get<float>() == 1.0f);
		CHECK(horn.get<double>() == 10.0);
		CHECK(horn.get<double>() == 0.0);
		CHECK(horn.get<double>() == 20.0);
		CHECK(horn.done());
	}
	SUBCASE("the user's vehicle, with the cab") {
		reader user{all[5].payload};
		CHECK(user.get<double>() == 42.0);
		CHECK(user.get<std::uint32_t>() == 5);
		CHECK(user.get<std::uint8_t>() == 1);
		CHECK(user.get<std::int8_t>() == 1);
		CHECK(user.done());
	}
}

TEST_CASE("blackbox: the camera and the couplings are written field by field") {
	std::ostringstream file;
	blackbox::writer out{file};
	out.header();
	out.camera({12.5, blackbox::camera_modes::external, 2, 7, {1.0, 2.0, 3.0}, 1.25f, -0.5f});
	out.coupling({13.0, 3, 1, 4, 0, 0x03, 0x01, 0, {5.0, 0.5, -6.0}});
	auto const all{records(file.str())};
	using blackbox::record;
	REQUIRE(all.size() == 2);
	CHECK(all[0].type == record::camera);
	CHECK(all[1].type == record::coupling);

	SUBCASE("the camera") {
		reader camera{all[0].payload};
		CHECK(camera.get<double>() == 12.5);
		CHECK(camera.get<std::uint8_t>() == blackbox::camera_modes::external);
		CHECK(camera.get<std::uint8_t>() == 2);
		CHECK(camera.get<std::uint32_t>() == 7);
		CHECK(camera.get<double>() == 1.0);
		CHECK(camera.get<double>() == 2.0);
		CHECK(camera.get<double>() == 3.0);
		CHECK(camera.get<float>() == 1.25f);
		CHECK(camera.get<float>() == -0.5f);
		CHECK(camera.done());
	}
	SUBCASE("a coupling") {
		reader coupling{all[1].payload};
		CHECK(coupling.get<double>() == 13.0);
		CHECK(coupling.get<std::uint32_t>() == 3);
		CHECK(coupling.get<std::uint8_t>() == 1);
		CHECK(coupling.get<std::uint32_t>() == 4);
		CHECK(coupling.get<std::uint8_t>() == 0);
		CHECK(coupling.get<std::uint8_t>() == 0x03);
		CHECK(coupling.get<std::uint8_t>() == 0x01);
		CHECK(coupling.get<std::uint32_t>() == 0);
		CHECK(coupling.get<double>() == 5.0);
		CHECK(coupling.get<double>() == 0.5);
		CHECK(coupling.get<double>() == -6.0);
		CHECK(coupling.done());
	}
}

TEST_CASE("blackbox: a look at the vehicles takes 25 bytes a vehicle") {
	std::ostringstream file;
	blackbox::writer out{file};
	out.header();
	out.vehicles(3.0, false, {{1, {1.f, 2.f, 3.f}, 0.5f, 60.f, blackbox::driver::ai}, {2, {4.f, 5.f, 6.f}, -0.5f, -20.f, blackbox::driver::none}, {7, {}, 0.f, 0.f, blackbox::driver::human}});
	auto const all{records(file.str())};
	REQUIRE(all.size() == 1);
	CHECK(all[0].payload.size() == 8 + 1 + 4 + 3 * blackbox::vehicle_state_size);

	reader look{all[0].payload};
	CHECK(look.get<double>() == 3.0);
	CHECK(look.get<std::uint8_t>() == 0);
	CHECK(look.get<std::uint32_t>() == 3);
	CHECK(look.get<std::uint32_t>() == 1);
	CHECK(look.get<float>() == 1.f);
	CHECK(look.get<float>() == 2.f);
	CHECK(look.get<float>() == 3.f);
	CHECK(look.get<float>() == 0.5f);
	CHECK(look.get<float>() == 60.f);
	CHECK(look.get<std::uint8_t>() == static_cast<std::uint8_t>(blackbox::driver::ai));
	CHECK(look.get<std::uint32_t>() == 2);
}

TEST_CASE("blackbox: texts are UTF-8 after their length in 7 bit groups, low first") {
	std::ostringstream file;
	blackbox::writer out{file};
	out.header();
	std::string const long_name(200, 'x');
	std::string const polish{"\xc5\xbc\xc3\xb3\xc5\x82w"}; // żółw, 7 bytes
	out.signal(0, long_name, {});
	out.signal(1, polish, {});
	out.signal(2, "", {});
	auto const all{records(file.str())};
	REQUIRE(all.size() == 3);

	// 200 = 0b1'1001000: 0xc8 (low 7 bits and the more flag), then 0x01
	CHECK(static_cast<std::uint8_t>(all[0].payload[4]) == 0xc8);
	CHECK(static_cast<std::uint8_t>(all[0].payload[5]) == 0x01);
	reader first{all[0].payload};
	first.get<std::uint32_t>();
	CHECK(first.text() == long_name);

	CHECK(static_cast<std::uint8_t>(all[1].payload[4]) == 7);
	reader second{all[1].payload};
	second.get<std::uint32_t>();
	CHECK(second.text() == polish);

	CHECK(static_cast<std::uint8_t>(all[2].payload[4]) == 0);
	CHECK(all[2].payload.size() == 4 + 1 + 24);
}
