/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

class basic_event;

// what the driving aid tells the driver: the panel of F1 (imgui) and the hud of RmlUi show the same reading. the limit ahead is
// worked out here from what the driver (TController) knows of the line, apart from the controller, so the rules can be tested
// on their own

namespace driving_aid
{

// what the driver knows of the limits ahead, as TController holds it
struct speed_limit_inputs
{
	double desired{0.0}; // the speed to go at now (VelDesired)
	int schedule{-1}; // the limit of the timetable or of the shunting, -1: none
	double last_speed{0.0}; // the limit after the end of the current one (VelLimitLastDist.first)
	double last_distance{0.0}; // metres to the end of the current limit (VelLimitLastDist.second), below 0: no limit is active
	double next_speed{0.0}; // the speed after the proximity distance (VelNext)
	double proximity{0.0}; // metres to where next_speed applies (ActualProximityDist)
	double obstacle{0.0}; // metres to the vehicle ahead (|TrackObstacle()|)
	double scan_range{0.0}; // metres from which on a limit lies beyond the line scanned (EU07_AI_SPEEDLIMITEXTENDSBEYONDSCANRANGE)
};

struct speed_limit_ahead
{
	int limit{0}; // the next limit, the current one when there's none to tell of
	double distance{0.0}; // metres to it
};

// the next limit worth telling the driver of: the stricter one of the end of the current limit and of the proximity distance,
// none (the current limit) when the driver may not move, when it lies beyond the line scanned or when it's a vehicle ahead
speed_limit_ahead next_speed_limit(speed_limit_inputs const &Inputs);

// what the driving aid shows, read from the vehicle driven
struct reading
{
	// the master controller: its position, or with an integrated controller (eimic) the percent of the power
	int throttle{0};
	// the field shunting, or with an integrated controller the position of the master controller
	int throttle_secondary{0};
	char direction{'N'}; // T: the speed control (tempomat), D: forward, R: backward, N: neutral
	int speed{0}; // km/h, rounded down
	int speed_limit{0}; // km/h the driver may go at now
	speed_limit_ahead ahead;
	double grade{0.0}; // percent, above 0 uphill in the direction of travel
	double brake{0.0}; // the position of the train brake, or with an integrated controller the percent of the braking
	double local_brake{0.0}; // the position of the local (independent) brake
	bool slipping{false};
	double brake_pressure{0.0}; // kPa in the brake cylinders
	double pipe_pressure{0.0}; // kPa in the train pipe
	bool alerter{false}; // the alerter calls, in the lit half of its blink
	bool cab_signal{false}; // the cab signal (SHP) calls
	int stop_time{0}; // seconds left of the exchange of passengers or load
	double obstacle_distance{0.0}; // metres to the vehicle ahead
	bool autopilot_allowed{false}; // the driver may hand the vehicle to the autopilot from the user interface
	bool autopilot{false}; // the autopilot drives
	basic_event const *next_signal{nullptr}; // the event of the nearest signal ahead the driver knows of, none if none
	double next_signal_distance{0.0}; // metres to it
};

// the grade is told of from this many percent on
inline constexpr double grade_shown{0.25};
// the vehicle ahead is told of from this many metres on
inline constexpr double obstacle_shown{75.0};

} // namespace driving_aid
