/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "vehicle/blackbox.h"

#include <array>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

class TDynamicObject;
class TTrack;
class TMemCell;
class basic_event;
enum class user_command;

namespace simulation {

// the black box of the session (see blackbox_doc.md): written along with the physics logs, in driver mode,
// from the moment the scenario is loaded until the simulator closes or the editor is entered
class blackbox_recorder {
public:
	// starts the recording once the scenario is ready, if it is to be made, then records what is due.
	// Deltatime: simulation time passed since the last call
	void update(double Deltatime);
	// writes the end of the recording and closes the file. a recording closed isn't started again
	void close();

	// what happens between the looks, from wherever it happens; the cabs may run on threads of their own.
	// nothing is written unless the recording runs
	void command(TDynamicObject const *Vehicle, user_command Command, int Action, double Param1, double Param2);
	void driver_changed(TDynamicObject const *Vehicle, bool Ai);
	// Event: the one telling the aspect of the signal
	void signal_passed(TDynamicObject const *Vehicle, basic_event const &Event, double Speed1, double Speed2, double Trainspeed, double Overrun, std::uint8_t Flags);
	void vehicle_removed(TDynamicObject const *Vehicle);
	// a train at a station of its timetable; the time and the vehicle are filled in here
	void station(TDynamicObject const *Vehicle, blackbox::station_event Station);
	// a train told to sound the horn (W6), at Position
	void horn_requested(TDynamicObject const *Vehicle, double Duration, double Tone, glm::dvec3 const &Position);
	// where the user looks from, every frame; written when it changes, at most twice a second. Owner: the vehicle the
	// camera goes with; the time and its number are filled in here
	void camera(blackbox::camera_state Camera, TDynamicObject const *Owner);

private:
	struct coupler_entry {
		TDynamicObject const *other{nullptr};
		int other_side{0};
		int flags{0};
	};
	struct vehicle_entry {
		std::uint32_t id{blackbox::no_vehicle};
		blackbox::vehicle_state last; // as listed last
		bool listed{false};
		std::array<coupler_entry, 2> couplers; // as seen last
		bool couplers_known{false};
	};
	struct switch_entry {
		TTrack *track{nullptr};
		std::uint32_t index{0};
		int state{-1};
	};
	struct signal_entry {
		TMemCell const *cell{nullptr};
		glm::dvec3 location{0.0};
		float speed1{0.0f};
		float speed2{0.0f};
		std::string command;
		bool known{false};
	};

	void open();
	void write_map();
	static std::vector<std::pair<glm::dvec3, float>> stop_point_headings();
	void write_cab();
	// the clock, the user's vehicle, the vehicles, the switches and the signals
	void look_around(bool Keyframe);
	void look_at_vehicles(bool Keyframe);
	void look_at_switches();
	void look_at_signals();
	// the couplers of every vehicle against the last look: those coupled at the first look, then the changes
	void look_at_couplers();
	void write_camera();
	// the timetable of the train the user is in, written again when it is another one; none isn't written
	void look_at_timetable(TDynamicObject const &Vehicle, std::uint32_t Id);
	// number of the vehicle, given (and the vehicle written) on first sight
	std::uint32_t vehicle_id(TDynamicObject const *Vehicle);
	std::uint32_t signal_at(glm::dvec3 const &Location) const;

	std::mutex m_mutex;
	std::array<char, 256 * 1024> m_filebuffer{};
	std::ofstream m_file;
	std::unique_ptr<blackbox::writer> m_writer;
	bool m_started{false}; // tried to start, made or not
	double m_time{0.0};
	double m_cabdue{0.0};
	double m_lookdue{0.0};
	double m_keyframedue{0.0};
	std::unordered_map<TDynamicObject const *, vehicle_entry> m_vehicles;
	std::uint32_t m_nextvehicle{1};
	std::vector<switch_entry> m_switches;
	std::vector<signal_entry> m_signals;
	std::uint32_t m_uservehicle{blackbox::no_vehicle};
	bool m_userincab{false};
	std::int8_t m_usercab{0};
	bool m_userknown{false};
	std::string m_timetabletrain;
	TDynamicObject const *m_timetablevehicle{nullptr}; // the user's vehicle at the last look at the timetable
	int m_timetablestations{0};
	blackbox::camera_state m_camera; // as given last
	TDynamicObject const *m_cameraowner{nullptr};
	blackbox::camera_state m_camerawritten; // as written last
	bool m_cameraknown{false};
	bool m_camerawrote{false};
	double m_cameradue{0.0};
	// how long the looks around take, told in the log when the recording ends
	std::chrono::steady_clock::duration m_looktime{};
	std::chrono::steady_clock::duration m_looklongest{};
	int m_looks{0};
};

extern blackbox_recorder Blackbox;

} // namespace simulation
