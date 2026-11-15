/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <string>
#include <string_view>
#include <vector>

#include <glm/glm.hpp>

// the black box: a binary recording of a whole session, from which another program can replay it on the
// map of the scenery. the player's train in detail (with what the AI would have done at the time, every
// command, the autopilot, the signals passed), the other vehicles sparingly, the tracks and the signals
// with the changes of the switches and the aspects. the layout of the file is described in blackbox_doc.md;
// this is only the writer, which doesn't know the simulation, so the tests can check what it writes
namespace blackbox {

inline constexpr std::array<char, 8> magic{'E', 'U', '0', '7', 'B', 'B', 'X', '\0'};
inline constexpr std::uint16_t format_version{1};

// a record is its type, the length of what follows and that many bytes
enum class record : std::uint8_t {
	info = 0x01,
	track = 0x10,
	signal = 0x11,
	vehicle = 0x12,
	vehicle_removed = 0x13,
	stop_point = 0x14,
	horn_point = 0x15,
	cab = 0x20,
	vehicles = 0x21,
	switch_state = 0x22,
	signal_aspect = 0x23,
	timetable = 0x24,
	camera = 0x25,
	command = 0x30,
	driver_change = 0x31,
	signal_passed = 0x32,
	user_vehicle = 0x33,
	clock = 0x34,
	station = 0x35,
	horn_request = 0x36,
	coupling = 0x37,
	end = 0x3f
};

// who drives a vehicle
enum class driver : std::uint8_t { none = 0, human = 1, ai = 2 };

// no vehicle: a command of the simulation, the user out of any vehicle
inline constexpr std::uint32_t no_vehicle{0};
// a signal passed which isn't on the map
inline constexpr std::uint32_t no_signal{0xffffffff};

namespace cab_flags {
inline constexpr std::uint8_t in_cab{0x01}; // the user is in the cab, not looking around outside
inline constexpr std::uint8_t ai_valid{0x02}; // the consist is started, so what the AI would do is worked out
inline constexpr std::uint8_t alerter_blinking{0x04};
inline constexpr std::uint8_t alerter_beeping{0x08};
inline constexpr std::uint8_t wheel_slip{0x10};
inline constexpr std::uint8_t radio_stop{0x20};
inline constexpr std::uint8_t alarm_chain{0x40};
} // namespace cab_flags

// what the driver of the user's train goes by: the lights it is to show depend on it
namespace movements {
inline constexpr std::uint8_t other{0}; // no driver, or standing by
inline constexpr std::uint8_t train{1}; // a train run (Pc 1 or Pc 2 at the head, Pc 5 at the tail)
inline constexpr std::uint8_t shunting{2}; // shunting (Tb 1)
} // namespace movements

// what is switched on in the user's train
namespace cab_devices {
inline constexpr std::uint32_t battery{0x0001};
inline constexpr std::uint32_t main_breaker{0x0002};
inline constexpr std::uint32_t pantograph_front{0x0004};
inline constexpr std::uint32_t pantograph_rear{0x0008};
inline constexpr std::uint32_t converter{0x0010};
inline constexpr std::uint32_t compressor{0x0020};
inline constexpr std::uint32_t line_contactors{0x0040}; // the motors are connected
inline constexpr std::uint32_t overload_relay{0x0080}; // tripped
inline constexpr std::uint32_t sanding{0x0100};
inline constexpr std::uint32_t horn{0x0200};
inline constexpr std::uint32_t doors_left{0x0400}; // open
inline constexpr std::uint32_t doors_right{0x0800}; // open
inline constexpr std::uint32_t radio{0x1000};
inline constexpr std::uint32_t heating{0x2000}; // of the train
inline constexpr std::uint32_t spring_brake{0x4000}; // applied
inline constexpr std::uint32_t shunt_mode{0x8000};
inline constexpr std::uint32_t departure_signal{0x0001'0000}; // sounding
inline constexpr std::uint32_t door_permit_left{0x0002'0000}; // the doors may be opened
inline constexpr std::uint32_t door_permit_right{0x0004'0000};
inline constexpr std::uint32_t door_step{0x0008'0000}; // the steps come out with the doors
inline constexpr std::uint32_t door_lock{0x0010'0000}; // the doors lock above a speed
inline constexpr std::uint32_t doors_locked{0x0020'0000}; // locked now
inline constexpr std::uint32_t brake_releaser{0x0040'0000}; // releasing the brakes of the vehicle
inline constexpr std::uint32_t fuel_pump{0x0080'0000};
inline constexpr std::uint32_t oil_pump{0x0100'0000};
inline constexpr std::uint32_t motor_blowers{0x0200'0000}; // the fans of the traction motors
inline constexpr std::uint32_t engine_hot{0x0400'0000}; // the diesel engine is over its temperature
inline constexpr std::uint32_t speed_control{0x0800'0000}; // the cruise control holds a speed
} // namespace cab_devices

// what drives the vehicle whose motors the cab drives, for a reader to show what matters for it
namespace engines {
inline constexpr std::uint8_t none{0}; // no engine of its own: a draisine, a car
inline constexpr std::uint8_t electric{1}; // series motors
inline constexpr std::uint8_t electric_induction{2}; // induction motors
inline constexpr std::uint8_t diesel{3}; // with a mechanical or a hydraulic transmission
inline constexpr std::uint8_t diesel_electric{4};
inline constexpr std::uint8_t steam{5};
} // namespace engines

// the security system of the user's train: the alerter (CA) and the cab signalling (SHP)
namespace security_flags {
inline constexpr std::uint8_t vigilance{0x01}; // the alerter lamp blinks: the driver has to acknowledge
inline constexpr std::uint8_t cabsignal{0x02}; // the cab signal lamp blinks: an SHP magnet was passed
inline constexpr std::uint8_t vigilance_beeping{0x04};
inline constexpr std::uint8_t cabsignal_beeping{0x08};
inline constexpr std::uint8_t braking{0x10}; // not acknowledged in time: the system brakes the train
} // namespace security_flags

namespace station_kinds {
inline constexpr std::uint8_t stopped{1}; // stood at the stop point of a timetabled stop
inline constexpr std::uint8_t passed{2}; // passed a station where the timetable has no stop
// left the stop point of a timetabled stop behind before it was time to go: without stopping at it, or, after a
// stop told before, leaving before the departure
inline constexpr std::uint8_t missed{3};
inline constexpr std::uint8_t ready{4}; // still at the stop point when it was time to go: the train may leave
} // namespace station_kinds

// where the user looks from
namespace camera_modes {
inline constexpr std::uint8_t cab{0}; // from the cab of the vehicle the user is in
inline constexpr std::uint8_t external{1}; // one of the outside views of the user's train
inline constexpr std::uint8_t free{2}; // the free camera, flying about
inline constexpr std::uint8_t debug{3}; // the debug camera
} // namespace camera_modes

namespace passing_flags {
inline constexpr std::uint8_t shunt{0x01}; // a shunting signal
inline constexpr std::uint8_t at_stop{0x02}; // passed showing stop
inline constexpr std::uint8_t automatic_block{0x04}; // stop of an automatic block signal, which may be passed slowly
} // namespace passing_flags

namespace track_kinds {
inline constexpr std::uint8_t normal{0};
inline constexpr std::uint8_t switch_branch{1};
} // namespace track_kinds

struct info_data {
	std::string scenery; // the scenario file, as the simulator was given it
	std::string started; // local time the recording started at, ISO 8601
	std::uint16_t year{0}; // simulation date and time at the start
	std::uint16_t year_day{0}; // 1..366
	double second_of_day{0.0};
	std::string start_vehicle; // the vehicle the user was to start in, ghostview for none
	std::string simulator_version;
	float cab_interval{0.0f}; // [s] between the samples of the user's train
	float vehicles_interval{0.0f}; // [s] between the looks at the other vehicles
	float keyframe_interval{0.0f}; // [s] between the looks which list all the vehicles
};

// one rail track of the map, or one of the two ways of a switch
struct track_data {
	std::uint32_t index{0};
	std::string name;
	std::uint8_t kind{track_kinds::normal};
	std::uint8_t branch{0}; // the switch position this way is taken in
	std::uint8_t category{1}; // 1 rail
	std::vector<glm::vec3> points; // world frame, y up
};

struct vehicle_data {
	std::uint32_t id{no_vehicle};
	std::string name;
	std::string type;
	std::string train; // train number of its timetable, if it leads one
	float length{0.0f}; // [m]
	float width{0.0f}; // [m]
	std::uint8_t category{1}; // 1 rail, 2 road
};

// what the AI driver would do at this place and speed: the speed it would hold, the next change of the
// speed limit it sees and how hard it would accelerate (negative: brake)
struct ai_reference {
	float target_kmh{0.0f};
	float next_limit_kmh{-1.0f}; // -1 none in sight
	float next_limit_distance{-1.0f}; // [m], -1 none in sight
	float acceleration{0.0f}; // [m/s2]
	float braking_distance{0.0f}; // [m]
	float timetable_vmax_kmh{-1.0f}; // -1 none
	float track_vmax_kmh{-1.0f}; // -1 none
};

// one look at the user's train
struct cab_sample {
	double time{0.0};
	std::uint32_t vehicle{no_vehicle};
	glm::dvec3 position{0.0};
	float speed_kmh{0.0f}; // signed: positive along the front of the vehicle
	float acceleration{0.0f}; // [m/s2]
	float odometer_km{0.0f};
	driver driving{driver::none};
	std::uint8_t flags{0}; // cab_flags
	std::int8_t main_controller{0};
	std::int8_t scnd_controller{0};
	std::int8_t reverser{0};
	float train_brake{0.0f}; // position of the driver's brake valve
	float local_brake{0.0f}; // 0..1
	float brake_cylinder{0.0f}; // [bar]
	float brake_pipe{0.0f}; // [bar]
	float motor_current{0.0f}; // [A]
	ai_reference ai;
	std::uint32_t devices{0}; // cab_devices
	float main_reservoir{0.0f}; // [bar]
	float voltage{0.0f}; // given to the motors [V]
	float engine_rpm{0.0f}; // of the engine or the motors
	std::uint16_t lights_head{0}; // the lights lit at the head of the train, bits of the simulator's light
	std::uint16_t lights_tail{0}; // at its tail: the last vehicle
	std::int8_t cab{0}; // the cab the user is in: 1 front, -1 rear, 0 none
	std::uint8_t security{0}; // security_flags
	float lateral_acceleration{0.0f}; // [m/s2] across the track, in the curves
	std::uint8_t movement{0}; // movements: what the driver of the train goes by, which sets the lights it is to show
	std::int16_t light_hint_head{-1}; // the light pattern the timetable or the scenery asks for at the head, -1 the usual
	std::int16_t light_hint_tail{-1}; // at the tail
	std::uint16_t equipped_head{0}; // the lamps the vehicle at the head has at that end: what of a pattern it can show
	std::uint16_t equipped_tail{0}; // at the tail
	std::uint8_t engine{engines::none}; // engines
	std::uint8_t brake_delay{0}; // the setting of the brakes: 1 G, 2 P, 4 R, 8 Mg, as bits
	std::uint8_t radio_channel{0};
	// what the driver's brake valve shows on its gauge [bar]: the equalizing (control) reservoir of most valves,
	// the time reservoir of the FV4a, which is empty but after an overcharge
	float driver_valve{0.0f};
	float feed_pipe{0.0f}; // the main reservoir pipe along the train [bar]
	float oil_pressure{0.0f}; // of the diesel engine [MPa]
	float water_temperature{0.0f}; // of the diesel engine's cooling [°C]
	float oil_temperature{0.0f}; // [°C]
	float engine_temperature{0.0f}; // [°C]
	float speed_control_kmh{0.0f}; // the speed the cruise control is set to
	float engine_power{0.0f}; // [kW]
	std::int8_t hand_brake{0}; // the position of the hand brake, 0 released
	std::uint8_t brake_mode{0}; // the mode of the brakes: 1 PS, 2 PN, 4 EP, 8 MED, as bits
	std::uint8_t horn{0}; // the horn's tones sounding: 1 low, 2 high, 4 a third
	float integrated_controller{0.0f}; // the one handle of driving and braking of a unit, -1..1
	float clutch{0.0f}; // the clutch of a diesel's gearbox, 0 open .. 1 closed
	float converter_fill{0.0f}; // the filling of a diesel's hydraulic torque converter, 0..1
	float consist_brake_cylinder{0.0f}; // the highest brake cylinder pressure of the vehicles of the train [bar]
};

// a station of the timetable of the user's train; the times in minutes of the day, -1 none
struct timetable_row {
	std::string station;
	float km{0.0f};
	float vmax{-1.0f}; // [km/h] before the station
	float arrival{-1.0f};
	float departure{-1.0f};
	bool maintenance{false}; // a technical stop
};

struct timetable_data {
	double time{0.0};
	std::uint32_t vehicle{no_vehicle};
	std::string train;
	std::string category;
	std::string from;
	std::string to;
	float vmax{-1.0f}; // [km/h] of the train
	std::vector<timetable_row> rows;
};

// a train at a station of its timetable, at its stop point (W4)
struct station_event {
	double time{0.0};
	std::uint32_t vehicle{no_vehicle};
	std::uint8_t kind{station_kinds::stopped};
	std::uint16_t row{0}; // of the timetable, from 1
	std::string station;
	glm::dvec3 position{0.0}; // of the stop point
	float distance{0.0f}; // [m] of the head of the train before the stop point, below 0 past it
	float stop_at{0.0f}; // [m] before the stop point, where its parameters put the head
	float train_length{0.0f}; // [m]
	float platform{0.0f}; // the second parameter of the stop point: above 0 the length of the platform, below 0 for shorter trains only (W32)
	float arrival{-1.0f}; // the timetable's, in minutes of the day, -1 none
	float departure{-1.0f};
};

// one vehicle in a look at the vehicles
// the camera of the user
struct camera_state {
	double time{0.0};
	std::uint8_t mode{camera_modes::free};
	std::uint8_t view{0}; // of an outside view: 0 the front of the train, 1 its rear, 2 a bogie, 3 drive-by
	std::uint32_t vehicle{no_vehicle}; // the vehicle the camera goes with, no_vehicle for none
	glm::dvec3 position{0.0};
	float heading{0.0f}; // [rad] of where it looks, from +z towards +x, as the heading of a vehicle
	float pitch{0.0f}; // [rad] up from the horizon
};

// a coupler of a vehicle coupled, uncoupled, or coupled otherwise (the hoses, the cables)
struct coupling_change {
	double time{0.0};
	std::uint32_t vehicle{no_vehicle};
	std::uint8_t side{0}; // the coupler of the vehicle: 0 the front one, 1 the rear one
	std::uint32_t other{no_vehicle}; // the vehicle coupled to it now, no_vehicle for none
	std::uint8_t other_side{0}; // the coupler of that one
	std::uint8_t flags{0}; // what is coupled now, bits of the simulator's coupling: 1 the coupler, 2 the brake pipe, 4 the control, 8 the power...
	std::uint8_t previous_flags{0}; // and before
	std::uint32_t previous_other{no_vehicle}; // the vehicle coupled before
	glm::dvec3 position{0.0}; // of the coupler
};

struct vehicle_state {
	std::uint32_t id{no_vehicle};
	glm::vec3 position{0.0f};
	float heading{0.0f}; // [rad] of the front of the vehicle, from +z towards +x
	float speed_kmh{0.0f}; // signed: positive along the front of the vehicle
	driver driving{driver::none};
};

struct command_entry {
	double time{0.0};
	std::uint32_t vehicle{no_vehicle}; // no_vehicle for a command of the simulation
	bool pressed{true}; // false: released
	double param1{0.0};
	double param2{0.0};
	std::string name;
};

struct signal_passing {
	double time{0.0};
	std::uint32_t vehicle{no_vehicle};
	std::uint32_t signal{no_signal};
	std::string event; // the event which tells the aspect
	glm::dvec3 position{0.0}; // of the signal
	float speed1{0.0f}; // the aspect when passed: speed at the signal [km/h], 0 stop, -1 no limit
	float speed2{0.0f}; // speed at the next signal
	float speed_kmh{0.0f}; // of the train
	float overrun{0.0f}; // [m] the train had gone past the signal when it was noticed
	std::uint8_t flags{0}; // passing_flags
};

// sizes of the parts of fixed size, as the readers expect them
inline constexpr std::size_t header_size{12};
inline constexpr std::size_t record_header_size{5};
inline constexpr std::size_t cab_sample_size{190};
inline constexpr std::size_t vehicle_state_size{25};

// writes the records to a stream. nothing is written until header() is
class writer {
public:
	explicit writer(std::ostream &Output);

	void header();
	void info(info_data const &Info);
	void track(track_data const &Track);
	void signal(std::uint32_t Index, std::string_view Name, glm::dvec3 const &Position);
	// a stop point (W4): the station it is for and its two parameters
	void stop_point(std::uint32_t Index, std::string_view Station, glm::dvec3 const &Position, float Stop_at, float Platform, float Heading);
	// a place where the trains are told to sound the horn (W6)
	void horn_point(std::uint32_t Index, std::string_view Name, glm::dvec3 const &Position, float Duration, float Tone);
	void timetable(timetable_data const &Timetable);
	void station(station_event const &Station);
	void horn_request(double Time, std::uint32_t Vehicle, float Duration, float Tone, glm::dvec3 const &Position);
	void camera(camera_state const &Camera);
	void coupling(coupling_change const &Change);
	void vehicle(vehicle_data const &Vehicle);
	void vehicle_removed(double Time, std::uint32_t Id);
	void cab(cab_sample const &Sample);
	// Keyframe: the list holds all the vehicles, not only those which changed
	void vehicles(double Time, bool Keyframe, std::vector<vehicle_state> const &States);
	void switch_state(double Time, std::uint32_t Track, std::uint8_t State);
	void signal_aspect(double Time, std::uint32_t Signal, float Speed1, float Speed2, std::string_view Command);
	void command(command_entry const &Command);
	void driver_change(double Time, std::uint32_t Vehicle, driver Driving);
	void signal_passed(signal_passing const &Passing);
	void user_vehicle(double Time, std::uint32_t Vehicle, bool In_cab, std::int8_t Cab);
	void clock(double Time, std::uint16_t Year, std::uint16_t Year_day, double Second_of_day);
	void end(double Time);

	// bytes written so far
	std::uint64_t size() const {
		return m_size; }

private:
	// the payload of a record, assembled before its length is known
	class payload {
	public:
		void u8(std::uint8_t Value);
		void i8(std::int8_t Value);
		void u16(std::uint16_t Value);
		void u32(std::uint32_t Value);
		void f32(float Value);
		void f64(double Value);
		void vec3(glm::vec3 const &Value);
		void dvec3(glm::dvec3 const &Value);
		// UTF-8 text after its length in 7 bit groups, low first: .NET BinaryWriter.Write(string)
		void text(std::string_view Value);
		std::string const &bytes() const {
			return m_bytes; }
		void clear() {
			m_bytes.clear(); }

	private:
		template <typename Type_>
		void raw(Type_ Value);
		std::string m_bytes;
	};

	payload &begin();
	void emit(record Type);

	std::ostream &m_output;
	payload m_payload;
	std::uint64_t m_size{0};
};

} // namespace blackbox
