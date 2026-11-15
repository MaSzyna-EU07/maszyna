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
#include <string>
#include <string_view>

// the criteria of a vehicle leaving the rails, as plain functions of what the vehicle and the track under it are
// like, so the collisions and the buckled track can use them as well as the speed in a curve does
// (physics.derailment new in eu07.ini, see TMoverParameters::check_derailment()). two ways off the rails:
//
// overturning: the lateral acceleration the cant doesn't balance, a = v^2 / R - g * sin(cant), takes the load off the
//   inner wheels; the unloading ratio D = dQ / Q = 2 / s * a / g * (h + s_flex * (h - h_c)) reaches 1 when they lift.
//   s the distance between the contact points of the wheels, h the height of the centre of gravity, s_flex and h_c the
//   flexibility coefficient and the roll centre height of the suspension, which lets the body lean out of the curve
//   (the reference values of the kinematic gauge, EN 15273-2 / UIC 505-1). the body sways on top of that: a lateral
//   vibration of 0.05 g at speed (as Kunieda's overturning formula of the Japanese railways adds one) and what the
//   uneven track adds
// flange climbing: the ratio of the lateral to the vertical force of the flanging wheel, Y / Q, over Nadal's limit
//   (tan d - mu) / (1 + mu * tan d). Y from the force the cant doesn't balance and the sideways part of the forces of
//   the couplings in the curve, shared by the axles, plus the guiding force of the leading axle under its angle of
//   attack; Q the static wheel load, with what the lateral forces at their heights move onto the wheel
//
// either has to hold over a stretch of track, not at an instant: EN 14363 takes the Y / Q of a 2 m sliding window, so
// a peak shorter than that, or the length of a simulation step, doesn't derail the vehicle
namespace derailment {

inline constexpr double gravity{9.81}; // [m/s2]

// the distance between the contact points of the wheels of a wheelset [m]: the gauge and the width of the rail head
// (1.5 m on the standard gauge)
inline constexpr double rail_head_width{0.065};
double contact_spacing(double Gauge);

// the suspension of the body, the reference vehicle of the kinematic gauge (EN 15273-2, UIC 505-1)
struct suspension {
	double flexibility{0.4}; // roll of the body to the angle of the force on it, s0
	double roll_centre{0.5}; // height of the roll centre above the rail [m], h_c0
};

// ---- the centre of gravity

// what the vehicle is, for its centre of gravity when its definition gives none
enum class vehicle_class {
	powered, // a locomotive, a motor car of a unit
	unpowered // a coach, a wagon
};
// height of the centre of gravity of the empty vehicle above the rail [m], from the height of the body: 0.38 of it for
// a locomotive or a motor car, 0.35 for a coach or a wagon (EU07: 1.65 m, the 430W coal wagon: 1.13 m, a 4 m high
// coach: 1.4 m). a body height up to 2 m in the definition is the height of the floor, the older meaning of the key:
// a 4 m high body is taken then
double default_centre_of_gravity(double BodyHeight, vehicle_class Class);
// the load raises it: the load lies on the floor, its centre half way up to the share of the height of the body
// above the floor it fills (the share of the load it can take), and the heights weigh with the masses [m]
double loaded_centre_of_gravity(double EmptyHeight, double EmptyMass, double LoadMass, double LoadShare, double Floor, double BodyHeight);

// ---- overturning

// the lateral acceleration [m/s2] the cant doesn't balance, positive out of the curve: v^2 / R - g sin(cant). the
// sign of the radius and of the cant is left out: the cant lifts the outer rail of a curve. on a straight piece of a
// transition the cant leans the vehicle into the curve to come
double unbalanced_acceleration(double Velocity, double Radius, double Cant);
// D = dQ / Q of the wheels on the inner side, at the lateral acceleration [m/s2]
double unloading_ratio(double Acceleration, double CentreOfGravity, double ContactSpacing, suspension const &Suspension = {});
// the lateral acceleration [m/s2] which lifts the inner wheels off the rail (D = 1)
double overturning_acceleration(double CentreOfGravity, double ContactSpacing, suspension const &Suspension = {});
// lateral vibration of the body on a good track [m/s2], growing with the speed up to 100 km/h
inline constexpr double vibration{0.05 * gravity};
double body_vibration(double Velocity);
// the speed [m/s] at which the vehicle overturns in a curve of the radius [m] and the cant [rad], on a good track
double overturning_speed(double Radius, double Cant, double CentreOfGravity, double Gauge);

// ---- flange climbing

// Nadal's limit of Y / Q for the flange angle [rad] and the friction of the flange on the rail
double nadal_limit(double FlangeAngle, double Friction);
// 70 degrees, the flange of the S1002 and the Polish 1:40 profiles
inline constexpr double flange_angle{1.2217304763960306};
// 1.2: Nadal with 70 degrees and a friction of 0.36 on a dry rail; the limit of EN 14363 for the flange climbing test
// in a tight curve on a twisted track. its 0.8 for the tests on the line is that with a margin for the acceptance of
// a vehicle, and a simulator wants the derailment itself
inline constexpr double climbing_limit{1.2};
// friction of the tread of the inner wheel of the leading axle, the guiding force it gives under the angle of attack
inline constexpr double tread_friction{0.3};
// angle of attack [rad] at which the creep force of the tread saturates
inline constexpr double saturated_attack{0.005};
// height of the buffers and the couplings above the rail [m] (UIC: 940 to 1065 mm)
inline constexpr double buffer_height{1.0};

// one end of the vehicle in a train
struct end_force {
	double compression{0.0}; // force of the coupling along the vehicle, pushing in [N]; negative pulls out
	double other_length{0.0}; // length of the vehicle at that end [m], 0 without one
};

// the vehicle as the flange climbing sees it
struct vehicle_state {
	double mass{0.0}; // with the load, without the rotating masses [kg]
	int axles{4};
	double length{0.0}; // over the buffers [m]
	double axle_base{0.0}; // the wheelbase of a bogie, or of the vehicle without bogies [m]
	double centre_of_gravity{1.5}; // [m]
	std::array<end_force, 2> ends{};
};

// the dtrack_* flags of the track (MOVER.h) the criteria look at
namespace track_damage {
inline constexpr int railwear{2};
inline constexpr int freerail{4};
inline constexpr int thinrail{8};
inline constexpr int railbend{16};
} // namespace track_damage

// the track under the vehicle
struct track_state {
	double radius{0.0}; // [m], 0 on a straight one
	double cant{0.0}; // [rad]
	double gauge{1.435}; // [m]
	double uneven_curvature{0.0}; // the lateral curvature of the unevenness under the vehicle [1/m]
	double uneven_cross{0.0}; // the cross level unevenness of the track [mm]
	int damage{0}; // dtrack_* flags
};

// what the state of the track does to the lateral forces and the wheel loads
struct track_condition {
	double lateral{1.0}; // multiplier of the sway of the vehicle and of the short waves of an uneven track
	double unloading{0.0}; // share of the wheel load the twist of the track (its cross level) takes off
};
track_condition condition_of(track_state const &Track);
// share of the vehicle that follows the short waves of an uneven track: the wheelsets and the bogies, the body not
inline constexpr double uneven_share{0.3};

// the angle [rad] between the vehicle and the line of the force of the coupling at the end, in a curve: half the
// angle between the two vehicles, whose middles are half their lengths apart along the curve
double coupling_angle(double Length, double OtherLength, double Radius);

struct wheel_forces {
	double lateral{0.0}; // the lateral force on the vehicle, positive out of the curve [N]
	double y{0.0}; // lateral force of the flanging wheel [N]
	double q{0.0}; // its vertical force [N]
	// a wheel lifted off the rail with a force on it is past any limit; no force at all (a vehicle without a mass) none
	double ratio() const { return q > 0.0 ? y / q : (y > 0.0 ? 1e9 : 0.0); }
};
// the forces of the flanging wheel of the leading axle, at the speed [m/s]
wheel_forces flange_forces(vehicle_state const &Vehicle, track_state const &Track, double Velocity);
// the lateral acceleration of the body the overturning goes by [m/s2]: what the cant doesn't balance, the sideways
// pull of the couplings, the vibration and the uneven track, by the state of the track
double body_acceleration(vehicle_state const &Vehicle, track_state const &Track, double Velocity);

// ---- the stretch a condition holds over

// the distance [m] a condition has to hold over: the window of EN 14363
inline constexpr double window{2.0};

// the distance travelled while the condition held, without a break. the same distance with any step: a step the
// condition holds at counts with its whole length
class stretch {
public:
	// true once the condition held over the window, given the distance of the step [m]
	bool update(bool Holds, double Distance, double Window = window);
	double distance() const { return m_distance; }
	void reset() { m_distance = 0.0; }

private:
	double m_distance{0.0};
};

enum class outcome {
	none,
	off_rails, // the flange climbed the rail: the vehicle runs off the rails, upright
	overturned // the vehicle fell over out of the curve
};

// what the vehicle came to in a step, with the values the decision went by
struct verdict {
	outcome result{outcome::none};
	double unloading{0.0}; // D
	double flange_ratio{0.0}; // Y / Q
	double acceleration{0.0}; // of the body, [m/s2]
};

// both criteria of a vehicle, step by step
class monitor {
public:
	// the step travelled the distance [m]; the condition at its end goes for the whole of it
	verdict update(vehicle_state const &Vehicle, track_state const &Track, double Velocity, double Distance);
	void reset();

private:
	stretch m_overturning;
	stretch m_climbing;
};

// the friction of a derailed vehicle sliding along [-]: on its wheels on the sleepers and the ballast, or on its side
inline constexpr double off_rails_friction{0.3};
inline constexpr double overturned_friction{0.5};
double sliding_friction(outcome Outcome);

// what the criteria found, for the log: "overturned at 113 km/h on track t1, curve radius 300 m, cant 0 mm: lateral
// acceleration ..., wheel unloading ..., Y/Q ... (mass ..., centre of gravity ..., couplings pushing ... / ... kN,
// track damage ..., cross level ... mm)"
std::string report(verdict const &Verdict, vehicle_state const &Vehicle, track_state const &Track, double Velocity, std::string_view TrackName);

} // namespace derailment
