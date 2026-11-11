#pragma once

#include <limits>
#include <string_view>

// the physical model of the coupling of two vehicles, used with couplerphysics set in eu07.ini (the legacy model of
// TMoverParameters::CouplerForce() otherwise). each vehicle end brings its buffers and its draw gear: progressive
// springs with friction hysteresis, pressed in series against those of the other end. the coupling has play (slack)
// in tension, or pulls the buffers in when its screw is tightened, and its steel link stretches for good past its
// yield force and breaks at its ultimate one, under a steady pull or the jerk of a run-out of the slack
namespace coupler_physics {

// force-deflection characteristic of the spring device of one vehicle end, its buffers or its draw gear:
//   F(d) = F_end * ((1 - p) * u + p * u^3), u = d / travel, F_end = stiffness * travel
// p (the progression) 0 gives a linear spring; towards 1 it's soft at first and stiffens towards the end of the
// travel, as ring springs and elastomer pads do
struct spring_curve {
	double stiffness{1.0}; // secant stiffness, up to the end of the travel [N/m]
	double travel{0.1}; // [m]
	double progression{0.0}; // 0 to 0.9

	double end_force() const { return stiffness * travel; }
	// force at the deflection [N], the deflection held within the travel
	double force(double Deflection) const;
	// deflection at the force [m]: the end of the travel at the end force and past it
	double deflection(double Force) const;
	// energy the device takes in along the curve up to the deflection [J]
	double energy(double Deflection) const;
};

// the devices of both ends pressed against each other: they carry the same force, their deflections add up. past
// the end of the travel of one, the other one takes the rest. the result doesn't depend on the order of the two
struct series_split {
	double force{0.0}; // [N]
	double first{0.0}; // deflection of the first device [m]
	double second{0.0}; // of the second one [m]
};
series_split series(spring_curve const &First, spring_curve const &Second, double Deflection);

// friction (Coulomb) hysteresis of a spring device: the force follows the loading curve while the deflection grows,
// drops to the unloading curve when it shrinks, and in between moves along a stiff transition line, as the friction
// of ring springs or of the wedges of a draw gear holds the device until the force changes enough
struct hysteresis {
	double force{0.0}; // carried at the last update [N]
	double deflection{0.0}; // at the last update [m]

	// the force at the new deflection, given the force of the loading and of the unloading curve there, and the
	// stiffness of the transition between them [N/m]
	double update(double Deflection, double Loading, double Unloading, double Transition);
	void reset();
};

// steel link of the coupling (the screw coupling and the hook, the knuckle of an automatic coupler): elastic up to
// the yield force, stretched for good past it (the hardening gives the force for each metre of it), broken at the
// ultimate force
struct link_strength {
	double yield{std::numeric_limits<double>::max()}; // [N]
	double ultimate{std::numeric_limits<double>::max()}; // [N]
	double hardening{1.0}; // [N/m]
	bool breakable{false}; // an articulated joint, or an end without a coupler set up, holds whatever
};

// the permanent elongation [m] the link has after carrying the force, given the one it had
double plastic_elongation(link_strength const &Link, double Force, double Elongation);
// peak force [N] of an elastic hit of two masses [kg] at the closing velocity [m/s] on a stop of the stiffness [N/m]:
// the kinetic energy of their relative motion all goes into the stop
double impact_force(double Velocity, double Stiffness, double Mass1, double Mass2);
// spread of the strength of the couplings as made: 1.0 to 1.2 of the rated force, fixed for the vehicle end, and the
// same on every compiler
double strength_scatter(std::string_view Name, int End);

// what the model takes of one vehicle end
struct end_parameters {
	spring_curve buffers;
	spring_curve draw_gear;
	double slack{0.0}; // play of the coupling in tension [m], negative for the buffers pressed in by a tightened screw coupling
	double buffers_unloading{0.5}; // force of the unloading curve to that of the loading one
	double draw_gear_unloading{0.5};
	double damping{0.0}; // viscous damping of the devices, while pressed [N s/m]
	double link_stiffness{1.0}; // of the steel parts of the end, past the travel of the springs [N/m]
	double crush_force{std::numeric_limits<double>::max()}; // a hit on the buffers past their travel with this force crushes them [N]
	link_strength link;
};

// the weaker of the links of the two ends, which gives first; the same whichever end asks
link_strength weaker_link(end_parameters const &First, end_parameters const &Second);

// state of the coupling of two vehicle ends, which both vehicles keep alike: each updates its own copy with the same
// gap, so the copies stay the same
struct pair_state {
	hysteresis draw_gear;
	hysteresis buffers;
	double elongation{0.0}; // permanent, of the link [m]; it adds to the slack until the vehicles are coupled again
	double gap{0.0}; // at the last update [m]
	int stop{0}; // at the last update: 1 the draw gear went solid, -1 the buffers did, 0 neither
	double stop_force{0.0}; // the springs carried at the stop [N]

	// a new coupling of the ends: the link adjusted (a screw coupling tightened anew), the draw gear released
	void couple();
};

struct pair_input {
	double gap{0.0}; // between the ends [m], negative with the buffers pressed in
	double gap_rate{0.0}; // [m/s]
	bool coupled{false};
	bool adapter{false}; // an adapter on either end: the buffers don't meet, the coupler takes the push alone
	bool damage{false}; // the link can stretch and break (crash damage on)
};

struct pair_result {
	double force{0.0}; // between the ends, tension positive [N]
	double tension{0.0}; // carried by the link [N]
	double compression{0.0}; // carried by the buffers [N]
	double buffers_deflection{0.0}; // of the buffers of the first end [m]
	bool overloaded{false}; // the link was pulled past its yield force
	bool broken{false}; // the link broke, it carries nothing now
};

// the force between the first end and the second one at the gap, updating the state of the pair
pair_result update_pair(end_parameters const &First, end_parameters const &Second, pair_state &State, pair_input const &Input);

// what a hit on the stop does to the ends
struct impact_result {
	double force{0.0}; // peak force of the hit [N]
	bool broken{false}; // the link broke under it
	bool crushed{false}; // the buffers or the coupler of an end were crushed by it
};

// the hit on the stop the pair went solid against at the last update (State.stop), at the closing velocity [m/s], of
// the vehicles of the masses [kg]: stretches or breaks the link of a coupled pair, may crush the buffers
impact_result impact(end_parameters const &First, end_parameters const &Second, pair_state &State, double Velocity, double Mass1, double Mass2);

} // namespace coupler_physics
