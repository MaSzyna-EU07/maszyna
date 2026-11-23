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
#include <optional>
#include <string_view>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

// a rail vehicle off the rails (physics.derailment new, see McZapkie/derailment.h for when it derails): from then on
// it doesn't follow the track any more, it moves on the ground by itself until the friction stops it.
//
// along: the longitudinal motion stays with the physics of the vehicle (TMoverParameters), which brakes it with the
//   friction() of this one, takes the grade() under it and passes the forces of the couplings on; each step gives
//   the distance travelled along the heading of the vehicle
// the heading: the way it travels; the one the vehicle had when it derailed, turned out of the curve by the sideways
//   speed it left the rail with: none off the end of the track, the speed of the flange climbing over the rail head
//   (or of the wheels lifting over it) otherwise. so in a curve it leaves the track to its outside, on a buckle
//   towards the bulge. from then on the friction of a body sliding askew (more across it than along it) and the
//   sideways part of the forces of its couplings turn it, little at speed
// the yaw: the body turns about its vertical axis by itself, apart from the way it travels: the forces of the
//   couplings at its ends (the push of the buffers of two bodies at an angle to each other, off the middle of the
//   buffer beam) turn it, the friction of the ground under the length of it holds it. pushed from behind, a train
//   of bodies off the track folds up into a zigzag once the bends between them are past what the buffers hold
// sideways: the vehicle slides across its heading only where the ground slopes across it more than the friction
//   holds, down an embankment say
// the height and the attitude: the ground under the supports of the vehicle (the ends of its wheelbase, the wheels
//   of both sides) gives its height, its pitch and its roll; it doesn't sink into the ground, it falls off a step
// tipping over: about the wheels of one side, the body as a rigid one with the lean of its suspension, so it tips at
//   the very lateral acceleration the wheel unloading of McZapkie/derailment.h reaches 1 at. what tips it: the
//   overturning in the curve while its outer wheels still run on the rail (the curve keeps pushing it out as v^2/R,
//   the excess over the limit the faster it tips), the slope of the ground across it, and the friction under it
//   while it slides sideways. once over its tipping angle it falls on its side, at 90 degrees, and stays there
//
// the steps are cut to 0.01 s inside, so the result doesn't depend on the step of the simulation
namespace derail_motion {

inline constexpr double gravity{9.81}; // [m/s2]
// the longest step of the motion [s]
inline constexpr double substep{0.01};

// what put the vehicle off the rails, for the way it leaves the track
enum class cause {
	off_rails, // the flange climbed the rail
	overturned, // the vehicle fell over out of the curve
	end_of_track, // it ran off the end of the track: straight on
	pulled_off, // the coupling of a vehicle off the track pulled it off sideways
	collision // overriding, or pushed off the track in a collision: straight on
};

// ---- the friction [-]

// of a vehicle tipping over on its outer wheels, which still roll on the outer rail
inline constexpr double guided_friction{0.1};
// of the wheels, or of the body on its side, sliding across the heading
inline constexpr double lateral_friction{0.5};
// the friction along the heading: upright on its wheels on the sleepers and the ballast, rising to the friction of a
// vehicle on its side (McZapkie/derailment.h) as it tips over
double sliding_friction(double Tipping, bool Guided);

// ---- the way the vehicle leaves the track

// how far the flange or the wheel goes sideways over the head of the rail before it drops off it [m]
inline constexpr double climb_width{0.1};
// the largest angle the vehicle turns off the track by [rad]
inline constexpr double most_departure{0.2};
// the angle [rad] the heading turns out of the curve by: the sideways speed sqrt(2 a w) the lateral acceleration a
// [m/s2] gives the wheels over the climb width w, against the speed [m/s]. none off the end of the track or in a
// collision
double departure_angle(cause Cause, double Speed, double Acceleration);

// ---- the scatter: what differs from one derailed vehicle to the next

// the ground under it and the way it bites into the ballast: each vehicle draws its own from the seed of the run and its
// name, so a run with the same seed goes the same way and one with another seed another way
struct scatter {
	double departure{0.0}; // added to the angle it leaves the track at [rad], not off the end of the track nor in a collision
	double yaw{0.0}; // its body turned off the way it travels when it leaves the rails [rad], to the left positive
	double yaw_rate{0.0}; // [rad/s] the kick of the leading wheels dropping off the rail head
	double friction_front{1.0}; // the friction under the front half of its body, of the friction of the motion
	double friction_rear{1.0}; // and under the rear half
	double dig_in{-1.0}; // the time after leaving the rails its leading end digs into the ballast [s], -1 never
	double dig_in_time{0.0}; // for how long [s]
	double dig_in_friction{1.0}; // the friction under the leading half then, of the friction of the motion
	int dig_in_side{1}; // the wheel that digs in: 1 the left one of the leading end, -1 the right one
};
// the scatter of a vehicle: departure angle +-0.01 rad, yaw +-0.02 rad, yaw rate +-0.02 rad/s, the friction under each
// half 0.85..1.15 of the friction of the motion, and in two of three vehicles a bite of the leading end 0.2..1.5 s after
// it left the rails, for 0.3..0.8 s, the friction under that half 2..4 times the friction of the motion then
scatter draw_scatter(std::uint32_t Seed);
// the seed of a vehicle: of the seed of the run and its name
std::uint32_t seed_of(std::uint32_t Run, std::string_view Name);

// ---- the buffers and the couplings of two vehicles, one of them off the track at least

// the height of the middle of the buffer beam above the rails [m]: UIC, 940..1065 mm
inline constexpr double buffer_height{1.0};
// half the spacing of the buffers [m]: UIC, 1750 mm
inline constexpr double buffer_half_spacing{0.875};
// the height of a buffer head [m]: two buffers further apart in height than that miss each other
inline constexpr double buffer_head{0.34};
// the bend between two bodies [rad] the push of the buffers goes all onto the inner buffer at: their play, 45 mm of
// difference over the spacing
inline constexpr double buffer_lock{0.05};
// the most a buffer takes [N]: past it, it is pressed solid and gives way, the frames push against each other at
// the middle of the beams
inline constexpr double buffer_limit{1.0e6};

// an end of a vehicle, where it meets the vehicle coupled to it
struct end_point {
	glm::dvec3 point{0.0}; // the middle of the buffer beam, at the height of the buffers
	glm::dvec3 outward{0.0, 0.0, 1.0}; // the way the end faces, level
	// how far the body is rolled [rad]: the buffers of two bodies rolled apart miss each other in height
	double roll{0.0};
};
// a force on a body [N], at the point it acts at
struct end_force {
	glm::dvec3 force{0.0};
	glm::dvec3 point{0.0};
};
// two ends meeting, as the first sees it
struct meeting {
	// apart [m], negative where the buffers press into each other: along the bisector of the bend while they touch,
	// the distance between the middles of the beams once they are apart
	double gap{0.0};
	// the bend: the angle [rad] the second body turns off the axis of the first, to the left of its end positive
	double bend{0.0};
	glm::dvec3 push{0.0, 0.0, 1.0}; // outwards from the first, the bisector of the bend: the buffers push the other way
	glm::dvec3 pull{0.0, 0.0, 1.0}; // the way the coupling draws the first: to the beam of the second
	glm::dvec3 point{0.0}; // the middle of the beam of the first
	// how well the buffers meet in height: 0 they miss each other (one body lower than the other, or rolled against it), to 1
	double overlap{1.0};
};
meeting meet(end_point const &First, end_point const &Second);
// the force on the first end of the meeting, Compression [N] pushing it (negative: the coupling pulling it): through
// the inner buffer of the bend, off the middle of the beam as far as the buffers meet and hold
end_force force_on(meeting const &Meeting, double Compression);

// ---- the bodies of two vehicles running into each other, one of them off the track at least

// how far the buffers and the coupling stand out of the body at each end [m]: the body box is shorter by them, so two
// ends meeting through their buffers (meet(), force_on()) don't touch as bodies until the buffers are pressed in by that
inline constexpr double buffer_length{0.6};
// the least overlap in height [m] two bodies run into each other with: lower, one goes over the other
inline constexpr double height_overlap{0.2};
// the bodies part after a hit with this share of the speed they met at
inline constexpr double contact_restitution{0.2};
// the friction of the side of one body sliding along the other
inline constexpr double contact_friction{0.3};
// the overlap left after a hit, taken out of it by moving the bodies apart: what is over the slop [m], this share of it
// a step
inline constexpr double contact_slop{0.02};
inline constexpr double contact_correction{0.8};

// where a body stands on the ground: the rectangle under its box (the box of the body, the buffers left out, rolled and
// pitched as it lies, seen from above along its level axis), and the height it reaches from and to
struct footprint {
	glm::dvec3 centre{0.0}; // of the rectangle, level
	glm::dvec3 axis{0.0, 0.0, 1.0}; // along the body, level
	double half_length{10.0};
	double half_width{1.5};
	double low{0.0}; // [m] the lowest and the highest point of the box
	double high{4.0};
};
struct pose;
// the footprint of the body of Length (over the buffers), Width and Height (above the rails) in the Pose
footprint footprint_of(pose const &Pose, double Length, double Width, double Height);
// two bodies in each other: how deep (the least the rectangles overlap across any of their sides), the way out of the
// first into the second (level), and the point they touch at
struct contact {
	double depth{0.0};
	glm::dvec3 normal{1.0, 0.0, 0.0};
	glm::dvec3 point{0.0};
};
std::optional<contact> collide(footprint const &First, footprint const &Second);
// a body in a contact, for the impulse: its mass, the velocity of its middle (base) and how fast it turns; on the rails
// it moves only along the track (Guided: along the Guide), not turning
struct contact_body {
	double mass{0.0}; // [kg], none: it doesn't move
	glm::dvec3 base{0.0};
	glm::dvec3 velocity{0.0};
	double yaw_rate{0.0};
	double gyration{30.0}; // the square of its radius of gyration about the vertical [m2]
	bool guided{false};
	glm::dvec3 guide{0.0, 0.0, 1.0};
};
// the velocity of the point of the body [m/s]
glm::dvec3 velocity_at(contact_body const &Body, glm::dvec3 const &Point);
// the impulse [N s] on the second body of the contact (the first gets the opposite one): it stops the two closing in on
// each other at the point, parting them with the restitution, and the friction holds them sliding along each other
glm::dvec3 contact_impulse(contact const &Contact, contact_body const &First, contact_body const &Second);
// the change of the velocity [m/s] and of the yaw rate [rad/s] of the body by the impulse at the point
std::pair<glm::dvec3, double> impulse_change(contact_body const &Body, glm::dvec3 const &Impulse, glm::dvec3 const &Point);

// the distance [m] a vehicle slides from the speed [m/s] with the friction, on the grade (rise per metre along the
// way); none when the grade doesn't let it stop
std::optional<double> stopping_distance(double Speed, double Friction, double Grade = 0.0);

// the Y/Q of the leading wheel of an end of a vehicle which the coupling pulls (or pushes) at the angle [rad] to its
// axis with the force [N]: the part across, taken by the axles of that end (half of them), against the static load of
// a wheel of the vehicle of the mass [kg]
double pull_ratio(double Force, double Angle, double Mass, int Axles);

// ---- the ground

// where the vehicle can find the ground
class ground {
public:
	virtual ~ground() = default;
	// the height of the ground at (X, Z): of the highest surface there at or under Ceiling; none where there is none
	virtual std::optional<double> height(double X, double Z, double Ceiling) const = 0;
};

// level everywhere
class flat_ground final : public ground {
public:
	explicit flat_ground(double Level) : m_level{Level} {}
	std::optional<double> height(double X, double Z, double Ceiling) const override;

private:
	double m_level;
};

// the triangles of the terrain of the scenery, kept in square cells for the lookup
class mesh_ground final : public ground {
public:
	using triangle = std::array<glm::dvec3, 3>;
	// the triangles which reach into the rectangle from Min to Max (x, z), in cells of the size [m]
	mesh_ground(std::vector<triangle> const &Triangles, glm::dvec2 const &Min, glm::dvec2 const &Max, double Cell = 8.0);
	std::optional<double> height(double X, double Z, double Ceiling) const override;
	// whether the point is inside the rectangle the triangles were taken for
	bool covers(double X, double Z) const;
	std::size_t size() const { return m_triangles.size(); }
	glm::dvec2 const &minimum() const { return m_min; }
	glm::dvec2 const &maximum() const { return m_max; }

private:
	std::optional<std::size_t> cell_of(double X, double Z) const;

	std::vector<triangle> m_triangles;
	std::vector<std::vector<std::size_t>> m_cells; // the triangles reaching into each cell
	glm::dvec2 m_min;
	glm::dvec2 m_max;
	double m_cell;
	std::size_t m_columns{0};
	std::size_t m_rows{0};
};

// the bed of the track at the place nearest to the vehicle: the top of the ballast under the rails, level out to the
// edge of the formation, then the slope of the embankment going down
struct formation {
	glm::dvec3 point{0.0}; // on the axis of the track, at the height of the rails
	glm::dvec3 direction{0.0, 0.0, 1.0}; // of the axis
	double depth{0.2}; // of the top of the ballast under the rails [m]
	double half_width{2.0}; // from the axis to the edge of the top [m]
	double slope{1.5}; // run of the slope per metre of fall: 1:1.5, of an embankment
	double height(double X, double Z) const;
};

// the ground as the vehicle finds it: the terrain of the scenery where it has any, level with the top of the ballast
// where it has none (a scenery without terrain), and the bed of the track over it
class site final : public ground {
public:
	explicit site(ground const *Terrain = nullptr, formation const &Bed = {}) : m_terrain{Terrain}, m_bed{Bed} {}
	void terrain(ground const *Terrain) { m_terrain = Terrain; }
	// the bed moves along the track with the vehicle
	void bed(formation const &Bed) { m_bed = Bed; }
	formation const &bed() const { return m_bed; }
	std::optional<double> height(double X, double Z, double Ceiling) const override;

private:
	ground const *m_terrain;
	formation m_bed;
};

// ---- the vehicle

// what of the vehicle the motion takes
struct vehicle_shape {
	double length{20.0}; // over the buffers [m]
	double width{3.0}; // of the body [m]
	double height{4.0}; // of the body above the rail [m]
	double centre_of_gravity{1.5}; // its height above the rail [m]
	double contact_spacing{1.5}; // of the wheels of a wheelset [m]
	double base{10.0}; // between the bogie pivots or the outer axles, which it rests on lengthwise [m]
	// the lean of the body on the suspension, as McZapkie/derailment.h takes it: flexibility and roll centre
	double flexibility{0.4};
	double roll_centre{0.5};
	// [kg] for the forces of the couplings on it; none: they don't move it
	double mass{0.0};
};

// how it left the rails
struct start {
	glm::dvec3 position{0.0}; // of the vehicle, at the height of the rails
	glm::dvec3 front{0.0, 0.0, 1.0}; // the way its front faces
	glm::dvec3 line{0.0, 0.0, 1.0}; // the direction of the track there, to measure the offset across it by
	double speed{0.0}; // along the front [m/s]
	cause reason{cause::off_rails};
	// the outer side of the curve of the line under it: 1 to the left of its front, -1 to the right, 0 straight
	int outward{0};
	double acceleration{0.0}; // the lateral acceleration of the body when it derailed, out of the curve [m/s2]
};

// where the body is and how it lies: the origin of its model, at the height of the rails between its wheels when
// upright, and its axes
struct pose {
	glm::dvec3 position{0.0};
	glm::dvec3 front{0.0, 0.0, 1.0};
	glm::dvec3 left{1.0, 0.0, 0.0};
	glm::dvec3 up{0.0, 1.0, 0.0};
};

class body {
public:
	// the vehicle leaves the rails, set apart from the others by the Scatter
	void begin(start const &Start, vehicle_shape const &Shape, ground const &Ground, scatter const &Scatter = {});
	// the step of Dt [s], in which the vehicle travelled Distance [m] along its heading (forwards along its front) and
	// has the Speed [m/s] along it at its end; the couplings of its ends push or pull it with the Ends through the step
	void step(double Dt, double Distance, double Speed, ground const &Ground, std::array<end_force, 2> const &Ends = {});
	// back on the rails, or never off them
	void reset() { *this = body{}; }

	bool active() const { return m_active; }
	pose const &current() const { return m_pose; }
	// the middle of the vehicle on the ground, under it as it stands (not moved over to the side it tipped to)
	glm::dvec3 const &base() const { return m_position; }
	// the speed along the heading [m/s], and the velocity of the middle, level
	double speed() const { return m_speed; }
	glm::dvec3 velocity() const;
	// the square of the radius of gyration about the vertical [m2]
	double gyration() const { return m_yawgyration; }
	// hit by another body: the velocity and the yaw rate change by these, the heading becomes the way it goes then
	// (turned about where it goes rear first); the speed() after it is the one its vehicle goes on with
	void kick(glm::dvec3 const &Velocity, double YawRate);
	// moved over by the offset (level), out of another body it is in
	void nudge(glm::dvec3 const &Offset);
	// the friction along the heading now, of its weight: of the body sliding along itself, more the more askew it goes,
	// up to the friction across it going sideways
	double friction() const;
	// the rise of the ground per metre along the heading, for the force of the gravity along it
	double grade() const { return m_grade; }
	// how far the body tipped over from its wheels [rad], to the left positive; 90 degrees: it lies on its side
	double tipping() const { return m_side * m_tipping; }
	// the roll of the body against the level [rad], to the left positive: the tipping and the slope across it
	double roll() const;
	bool lying() const { return m_tipping >= half_turn - 1e-9; }
	// still on its outer wheels on the rail, tipping over out of the curve
	bool guided() const { return m_guided; }
	// the distance it went on the ground since it left the rails [m]
	double travelled() const { return m_travelled; }
	// from the place it left the rails at: along the line of the track there, and across it, to the left positive [m]
	double along() const;
	double across() const;
	// the speed it slides with sideways, to the left of its front positive [m/s]
	double sideways() const { return m_lateral; }
	// at rest: not sliding, not tipping, on the ground
	bool at_rest() const;
	// the way it travels, level
	glm::dvec3 const &heading() const { return m_front; }
	// the way the front of its body faces, level
	glm::dvec3 const &axis() const { return m_axis; }
	// the angle [rad] its body is turned by off the line of the track where it left the rails, to the left positive
	double yaw() const;
	// how fast it turns [rad/s], to the left positive
	double yaw_rate() const { return m_yawrate; }
	// how many times the heading was turned about, the body having turned more than a right angle off the way it
	// goes: the heading stays within a right angle of the front of the body, so the speed along the heading (of
	// TMoverParameters, of its couplers at its ends) goes with the body; the speed changes its sign with each
	int reversals() const { return m_reversals; }
	cause reason() const { return m_reason; }
	// the vehicle the motion was begun with
	vehicle_shape const &shape() const { return m_shape; }

	static constexpr double half_turn{1.5707963267948966};

private:
	// the ground under the supports of the vehicle
	struct footing {
		double level{0.0}; // under the middle, not under any support
		double grade{0.0}; // rise per metre forwards
		double cross{0.0}; // rise per metre to the left
	};

	void advance(double Dt, double Distance, double Speed, ground const &Ground);
	footing find_footing(ground const &Ground) const;
	// down to the ground at the Level, or onto it, having travelled Travel [m] over the ground in the step
	void fall(double Dt, double Level, double Travel);
	void slide(double Dt, double Cross);
	void tip(double Dt, double Cross, double Speed);
	// the lateral acceleration [m/s2] on the centre of gravity, to the left positive, in the plane of the ground
	double lateral_load(double Cross, double Speed) const;
	void place();
	// to the left of the body
	glm::dvec3 left_of_heading() const;
	// the friction under the body going along its heading at the Speed and turning at the Yawrate, per its mass: the
	// force [m/s2], the moment about its middle [m2/s2] and the sideways load that trips it over [m/s2]
	struct drag {
		glm::dvec3 force{0.0};
		double moment{0.0};
		double trip{0.0};
	};
	drag friction_under(double Speed, double Yawrate) const;
	// the friction under the front (Front) or the rear half of the body, of the friction of the motion
	double friction_share(bool Front) const;
	// the leading end biting into the ballast now: 1 its front, -1 its rear, 0 none
	int biting() const;
	// the friction along the body and across it
	double friction_along() const;
	double friction_across() const;
	// the body turns, and the way it travels turns, through the substep
	void turn(double Dt, double Speed);

	bool m_active{false};
	cause m_reason{cause::off_rails};
	vehicle_shape m_shape;
	// the geometry of the tipping: half the spacing of the wheels, the height the lateral force leans the body at,
	// the distance of the centre of gravity from the wheels it tips about, the angle it tips over past, and the
	// square of the radius of gyration about them
	double m_pivot{0.75};
	double m_lever{1.8};
	double m_reach{1.9};
	double m_tipangle{0.4};
	double m_gyration{5.0};

	glm::dvec3 m_position{0.0}; // of the middle of the vehicle on the ground (at its base)
	glm::dvec3 m_front{0.0, 0.0, 1.0}; // the heading, level
	glm::dvec3 m_axis{0.0, 0.0, 1.0}; // the front of the body, level
	double m_yawrate{0.0}; // [rad/s]
	double m_yawgyration{30.0}; // the square of the radius of gyration of the body about its vertical axis [m2]
	scatter m_scatter;
	double m_time{0.0}; // since it left the rails [s]
	int m_reversals{0};
	double m_trip{0.0}; // the load across of the friction of the body going askew [m/s2], the way it slides
	// the forces of the couplings through the step: the force, and where it acts, along the body and across it
	struct end_load {
		glm::dvec3 force{0.0};
		double along{0.0};
		double across{0.0};
	};
	std::array<end_load, 2> m_ends{};
	glm::dvec3 m_origin{0.0}; // where it left the rails
	glm::dvec3 m_line{0.0, 0.0, 1.0}; // the direction of the track there, level
	double m_fallspeed{0.0}; // [m/s], downwards negative
	// on the ground, not falling. it leaves the rails above the top of the ballast, and drops onto it
	bool m_grounded{false};
	double m_lateral{0.0}; // [m/s], to the left positive
	double m_grade{0.0}; // along the heading
	double m_pitch{0.0}; // along the body
	double m_cross{0.0};
	int m_side{0}; // the side it tips to: 1 left, -1 right, 0 upright
	double m_tipping{0.0}; // [rad]
	double m_tiprate{0.0}; // [rad/s]
	// overturning in the curve: the lateral acceleration of the curve at the speed it derailed at, which goes with the
	// square of the speed, the side out of the curve, and how long it has been tipping
	bool m_guided{false};
	double m_guideacceleration{0.0};
	double m_guidespeed{0.0};
	int m_outward{0};
	double m_guidetime{0.0};
	double m_speed{0.0};
	double m_travelled{0.0};
	pose m_pose;
};

// the slowest a point of the body slides at [m/s]: the friction under one going slower holds it as much as it goes
inline constexpr double creep_speed{0.05};
// the fastest the body turns about its vertical axis [rad/s]
inline constexpr double most_yaw_rate{2.0};
// the slowest the heading is turned at by the forces across it [m/s]: slower, the vehicle stops where it goes
inline constexpr double turning_speed{0.5};
// the points the friction under the body is taken at, along it
inline constexpr int friction_points{8};

// how long the curve can keep tipping over a vehicle on its outer wheels [s]
inline constexpr double guidance_time{5.0};
// how high a step up the vehicle can climb onto, for the ground under it [m]
inline constexpr double step_up{1.0};

} // namespace derail_motion
