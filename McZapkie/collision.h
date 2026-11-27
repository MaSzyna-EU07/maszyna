/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <limits>
#include <optional>

#include "couplerphysics.h"
#include "derailment.h"

// what a collision of two vehicles does to them past their buffers (physics.collisions new in eu07.ini, see
// TMoverParameters::CollisionDetect()), as plain functions of the vehicles and the hit:
//
// crush: once the buffers of the pair are pressed all the way, the underframes carry the rest of the hit, up to the
//   force the weaker end gives at. only the energy the hit brings past what the buffers of EN 15551 of both ends take
//   in can crush them (contact_energy): a hit the buffers would take, or the push of a train at a stand, never does. past it the end crushes for good: a short peak of force, then a lower plateau as
//   the structure folds, then it packs down and stiffens (the force-crush curves of the full scale tests of the FRA:
//   Tyrell et al., IMECE2002-33247). the hit is an impulse limited to that force over the step, so the force never
//   goes past it and the depth of the crush is the same with any step; the depth stays with the end
// override: an end crushed deep enough no longer holds the other one down: the lighter of the two vehicles, or the
//   one with its buffers higher (40 mm, the vertical offset of the design collision of EN 15227), climbs onto the
//   other one and leaves the rails; past a deeper crush both do. interlocked anticlimbers hold them until the crush
//   zone of an end is used up
// pushed off the track: the force pressing a vehicle's end acts at an angle to it, the more in a curve and the
//   deeper the end is crushed, which gives a lateral force on its wheels; acting off the height of the centre of the
//   vehicle it lifts its leading wheels. Y / Q over Nadal's limit while the wheels roll on over the window of
//   EN 14363 and the flange climbs the rail (the light wagon between heavy ones goes first, in a curve sooner than
//   on a straight track: EN 15566 takes 250 kN of the pair of buffers in a curve of 150 m as the safe push). a wheel
//   which doesn't roll doesn't climb: Nadal's limit is that of a wheel rolling under its angle of attack, the creep of
//   the flange contact turning the friction upwards; standing, the friction holds the flange down, and with a flange
//   of 70 degrees and a friction over 1 / tan 70 = 0.36 no lateral force gets it up. so a train pushing at a stand,
//   or the short peak of the springs of a hit, never pushes a wheel off. a push that takes the whole load off the
//   leading wheels lifts them off the rail, rolling or not: that holding for a short time is enough
namespace collision {

inline constexpr double gravity{9.81}; // [m/s2]

// ---- the crush of an end

// force-crush characteristic of the underframe at one end of a vehicle
struct crush_curve {
	double peak{3.0e6}; // the force the crushing starts at [N]
	double plateau{2.0e6}; // of the crushing past the onset [N]
	double onset{0.05}; // the crush past which the peak drops to the plateau [m]
	double densify{0.8}; // the crush past which the crushed structure packs down and stiffens [m]
	double hardening{2.0}; // the growth of the force past it, to the plateau for each metre [1/m]
	double limit{1.5}; // the end can't be crushed past it, it's rigid from there [m]

	// the force the end gives at with the crush [N]; past the limit there's no force it gives at
	double force(double Crush) const;
	// the work it takes to crush the end from the one depth to the other one [J]
	double energy(double From, double To) const;
};

// what the vehicle is, for its defaults
enum class vehicle_kind {
	wagon, // a freight wagon
	coach, // a passenger coach, or a car of a unit
	locomotive,
	crash_zone // a vehicle built to EN 15227, with the energy absorbers and the anticlimbers at its ends
};
// the defaults of the kind: a wagon or a coach gives at 1.5 times the force of the proof test of EN 12663 at the
// height of the buffers (2 MN), a locomotive at 4 MN; the plateau is the 2 MN of the crush of the cab car of the FRA
// test. the absorbers of a crash zone give a steady 2 MN over 0.8 m, as the deformation tubes and the crash buffers
// do (2 MN of the crash buffers in service)
crush_curve default_crush(vehicle_kind Kind);

// the buffers of EN 15551, by their category
enum class buffer_category {
	A, // 105 mm, 30 kJ
	B, // 105 mm, 50 kJ
	C, // 105 mm, 70 kJ
	L, // the long stroke, 150 mm
	passenger // 110 mm, of the coaches
};

// the optional keys of the .fiz for an end (BuffCoupl): what's not given comes from the kind of the vehicle
struct end_definition {
	std::optional<double> peak; // FcrushPeak [N]
	std::optional<double> plateau; // FcrushPlateau [N]
	std::optional<double> onset; // CrushOnset [m]
	std::optional<double> densify; // CrushDensify [m]
	std::optional<double> hardening; // CrushHardening [1/m]
	std::optional<double> limit; // CrushLimit [m]
	std::optional<bool> crash_zone; // CrashZone: built to EN 15227
	std::optional<bool> anticlimber; // AntiClimber
	std::optional<double> buffer_height; // BufferHeight, of the empty vehicle above the rail [m]
	std::optional<buffer_category> buffers; // BufferCategory
};
vehicle_kind kind_of(bool Powered, bool Passenger, bool CrashZone);
crush_curve crush_of(end_definition const &Definition, vehicle_kind Kind);
bool anticlimber_of(end_definition const &Definition, vehicle_kind Kind);
// height of the buffers above the rail [m]: UIC 940 to 1065 mm. without a given one, a wagon has them at the top of
// the range empty and lower by 125 mm fully loaded on its springs, a locomotive or a coach at 1040 mm
double buffer_height(end_definition const &Definition, vehicle_kind Kind, double LoadShare);

// ---- the buffers, EN 15551

struct buffer_rating {
	double stroke{0.105}; // [m]
	double energy{30.0e3}; // dynamic energy of one buffer [J]
};
buffer_rating rating(buffer_category Category);
// the spring curve of the two buffers of an end side by side, of the progression given, which takes in their energy
// over the stroke
coupler_physics::spring_curve buffer_curve(buffer_category Category, double Progression);
std::optional<buffer_category> buffer_category_of(char Name);
// the buffers of an end without the category given: of category A, those of a coach of its own category (EN 15551,
// 30 kJ a buffer either way)
buffer_category default_buffers(vehicle_kind Kind);
// the energy the two buffers of an end take in over their stroke, of the category given or the default of the kind [J]
double buffer_capacity(end_definition const &Definition, vehicle_kind Kind);

// vehicles pressed onto each other past the stop of their buffers and the crush of their ends by more than this, more
// than a hit could have taken them in a step, are inside each other: a scenery put them down overlapping. no hit the
// crush could tell anything of, the legacy collision handles them [m]
inline constexpr double inside_margin{0.5};

// ---- the energy of a hit, for the crush

// the energy budget of the crush in a contact of two ends. what a hit brings to the stop of the buffers is the kinetic
// energy of the relative motion of the pair there, and what its buffers took in on the way (the springs of the .fiz,
// which may take less than the buffers of EN 15551 would). the buffers of EN 15551 of both ends take in their
// capacity of it first, the crush may take only the rest:
//   budget = 1/2 mu v^2 - max(0, capacity - absorbed)
// over the steps of a crush the crush done already is in neither term, so the crush of the whole hit never takes more
// than the hit brought past the buffers. the train behind the pair pushing it on through the crush brings in more:
// it's in the velocity the pair closes in with in the next step. the buffers take in in the stroke under way: the
// buffers pressed and the pair closing in. springing back they keep what they still hold, so a train pushing at a
// stand never builds up what it feeds into their damping. the buffers pressed all the way from the start of the
// contact took in nothing: vehicles put down inside each other, or knocked onto each other by a hit elsewhere, bring
// in only the kinetic energy they close in with. nor do the buffers of vehicles found inside each other in the
// contact (inside_margin) take in anything until they part: their springs pressed past the stop and back throw the
// pair about, which is no hit
class contact_energy {
public:
	// the step of the pair: whether its buffers press, and all the way; the closing velocity [m/s] and the force
	// pressing the buffers [N] at the start of the step; how far they're pressed in [m]; the length of the step [s]
	void update(bool Pressed, bool Solid, double Closing, double Force, double Compression, double Dt);
	// the vehicles of the pair are inside each other: what the buffers took in is gone, and they take in nothing more
	// until the contact is over
	void inside();
	// the crush of the hit took the energy [J]
	void crushed(double Work);
	// the energy the crush may take now, closing in at the velocity [m/s] with the reduced mass [kg], the buffers of
	// both ends taking in the capacity [J]; zero or less, the hit stays elastic
	double budget(double Mass, double Closing, double Capacity) const;
	// what the buffers took in in the stroke under way [J], and the crush in the contact
	double absorbed() const { return m_absorbed; }
	double crush_work() const { return m_crushed; }
	void reset();

private:
	double m_absorbed{0.0};
	double m_crushed{0.0};
	bool m_inside{false};
};

// ---- the hit past the buffers

// an end of the pair, as the hit sees it
struct end_state {
	crush_curve curve;
	double crush{0.0}; // the depth it's crushed to already [m]
};

struct hit_input {
	double velocity{0.0}; // the pair closes in on the stop at [m/s]
	double mass1{0.0}; // [kg]
	double mass2{0.0};
	double restitution{0.2}; // coefficient of restitution of the elastic rest of the hit
	double stiffness{1.0e8}; // of the steel parts of both ends in series, past the springs [N/m]
	double springs{0.0}; // the force the springs of the pair carry at the stop over the step [N]
	double dt{0.01}; // the length of the step [s]
	// the closing velocity the pair started the step with, before the forces of the step [m/s]; without it (0), the
	// velocity with what the springs took off it over the step
	double start_velocity{0.0};
	// the energy the crush may take at most [J] (contact_energy::budget()); none left, the hit stays elastic, and the
	// crush stops once it took it, the rest springs back as the buffers would have taken it
	double budget{std::numeric_limits<double>::infinity()};
};

struct hit_result {
	bool crushing{false}; // false: the hit stays elastic, it's the legacy exchange of momentum
	double impulse{0.0}; // the hit adds to the forces of the step, pushing the vehicles apart [N s]
	double velocity{0.0}; // the closing velocity after the step [m/s], negative as they part
	double force{0.0}; // the peak force between the ends, with the springs [N]
	double duration{0.0}; // how long that force acted over the step [s]
	double crush1{0.0}; // added to the end of the first vehicle [m]
	double crush2{0.0}; // of the second one
	double work{0.0}; // the crush took [J]
	bool spent{false}; // the crush stopped at the budget
};

// the hit of the step: the steel takes the hit elastically up to the force the weaker end gives at, past that the end
// crushes at that force until the pair closes in no faster than the steel takes back. the motion over the step is
// integrated in short slices, with the springs slowing the pair as well, so the crush doesn't depend on the step
hit_result hit(end_state const &First, end_state const &Second, hit_input const &Input);

// a crush of an end deeper than this is a damage worth the log [m]
inline constexpr double crush_mark{0.05};
// the draw gear sits behind the headstock: crushed this deep, the coupling of the end is torn off [m]
inline constexpr double draw_gear_crush{0.15};

// ---- override

// the crush of the contact of both ends past which the lighter (or the higher) vehicle climbs onto the other one,
// and past which both leave the rails [m]
inline constexpr double override_crush{0.35};
inline constexpr double override_both{0.6};
// the masses differ enough for the lighter one to climb
inline constexpr double override_mass_ratio{1.3};
// the buffers differ in height enough for the higher one to climb: the vertical offset of the design collision of
// EN 15227 [m]
inline constexpr double override_height{0.04};

struct override_input {
	double crush1{0.0}; // of the end of the first vehicle [m]
	double crush2{0.0};
	double mass1{0.0}; // [kg]
	double mass2{0.0};
	double height1{1.0}; // of its buffers [m]
	double height2{1.0};
	bool anticlimbers{false}; // both ends have them, interlocked
	double zone1{0.0}; // the crush zone of the end with the anticlimber, its densify [m]
	double zone2{0.0};
};
struct override_result {
	bool first{false}; // the first vehicle left the rails
	bool second{false};
};
override_result override_of(override_input const &Input);

// ---- pushed off the track

// the angle of the force at the end of a vehicle to it on a straight track: the play of the wheelsets and of the
// bogies, the buffer heads off the middle [rad]
inline constexpr double straight_angle{0.006};
// the crushed end no longer keeps the ends in line: the angle grows by this share of the crush to the half length
inline constexpr double crush_angle{0.1};
// the height the force acts at off the height it's carried at, the buffers of the pair not at the same height [m]
inline constexpr double eccentricity{0.10};
// the distance the wheels have to roll with Y / Q over the limit for the flange to climb the rail: the window of the
// criteria of derailment [m]
inline constexpr double climb_distance{derailment::window};
// the time the push has to hold the leading wheels off the rail, without any load on them, to lift them over the
// rail [s]
inline constexpr double lift_time{0.05};

struct pushed_vehicle {
	double mass{0.0}; // with the load, without the rotating masses [kg]
	int axles{4};
	double length{0.0}; // over the buffers [m]
	double pivots{0.0}; // the distance of the bogie pivots, of the axles without bogies [m]
	double axle_base{0.0}; // the wheelbase of a bogie, of the vehicle without bogies [m]
};
struct pushed_end {
	double force{0.0}; // pressing the end in [N]
	double other_length{0.0}; // of the vehicle at that end [m]
	double crush{0.0}; // of the end [m]
};
// the forces of the leading wheel at the end pressed in by the force, in a curve of the radius [m] (0 straight): its
// lateral force, and its vertical one, none or less when the push lifts it off the rail
struct pushed_wheel {
	double y{0.0}; // [N]
	double q{0.0}; // [N]
	double ratio() const;
	bool lifted() const { return q <= 0.0 && y > 0.0; }
};
pushed_wheel pushed_forces(pushed_vehicle const &Vehicle, pushed_end const &End, double Radius);
// Y / Q of that wheel
double pushed_ratio(pushed_vehicle const &Vehicle, pushed_end const &End, double Radius);

// the push off the track step by step: how far the wheels rolled with Y / Q over the limit without a break, and how
// long the push held the leading wheels lifted off the rail
class pushed_monitor {
public:
	// true once the flange climbed the rail, given whether Y / Q held over the limit in the step and the distance the
	// wheels rolled while it held [m], and whether the push lifted the wheels and for how long in the step [s]
	bool update(bool Climbs, double Distance, bool Lifted, double Duration);
	void reset();
	double distance() const { return m_climbing.distance(); }
	double time() const { return m_lifted; }

private:
	derailment::stretch m_climbing;
	double m_lifted{0.0};
};

} // namespace collision
