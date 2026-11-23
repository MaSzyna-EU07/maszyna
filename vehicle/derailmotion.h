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
#include <optional>
#include <vector>

#include <glm/glm.hpp>

// a rail vehicle off the rails (physics.derailment new, see McZapkie/derailment.h for when it derails): from then on
// it doesn't follow the track any more, it moves on the ground by itself until the friction stops it.
//
// along: the longitudinal motion stays with the physics of the vehicle (TMoverParameters), which brakes it with the
//   friction() of this one, takes the grade() under it and passes the forces of the couplings on; each step gives
//   the distance travelled along the heading of the vehicle
// the heading: the one the vehicle had when it derailed, turned out of the curve by the sideways speed it left the
//   rail with: none off the end of the track, the speed of the flange climbing over the rail head (or of the wheels
//   lifting over it) otherwise. it stays the heading from then on: the vehicle runs on straight, so in a curve it
//   leaves the track to its outside, on a buckle towards the bulge
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
	// the vehicle leaves the rails
	void begin(start const &Start, vehicle_shape const &Shape, ground const &Ground);
	// the step of Dt [s], in which the vehicle travelled Distance [m] along its heading (forwards along its front) and
	// has the Speed [m/s] along it at its end
	void step(double Dt, double Distance, double Speed, ground const &Ground);
	// back on the rails, or never off them
	void reset() { *this = body{}; }

	bool active() const { return m_active; }
	pose const &current() const { return m_pose; }
	// the friction along the heading now, of its weight
	double friction() const { return sliding_friction(m_tipping, m_guided); }
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
	glm::dvec3 const &heading() const { return m_front; }
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
	glm::dvec3 left_of_heading() const;

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
	glm::dvec3 m_origin{0.0}; // where it left the rails
	glm::dvec3 m_line{0.0, 0.0, 1.0}; // the direction of the track there, level
	double m_fallspeed{0.0}; // [m/s], downwards negative
	// on the ground, not falling. it leaves the rails above the top of the ballast, and drops onto it
	bool m_grounded{false};
	double m_lateral{0.0}; // [m/s], to the left positive
	double m_grade{0.0};
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

// how long the curve can keep tipping over a vehicle on its outer wheels [s]
inline constexpr double guidance_time{5.0};
// how high a step up the vehicle can climb onto, for the ground under it [m]
inline constexpr double step_up{1.0};

} // namespace derail_motion
