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

#include "McZapkie/steam/parameters.h"

// Heusinger (Walschaerts) valve gear as a plane linkage, solved exactly from its dimensions, with no
// linearisation: the slip of the die block and the unequal cutoffs of the two ends of the cylinder come out of
// the geometry
//
// the crank angle is counted the way the wheels turn running forwards; at 0 the crank points forwards and the
// piston is at its front dead centre, at pi at the back one. lengths in m, angles in rad
namespace steam::detailed
{

struct valve_position
{
	double valve; // the travel of the valve from its central position, forwards
	double die_block; // along the arc of the link from its pivot, up
	double link; // the angle of the link
	double lever; // the angle of the combination lever
	double crosshead; // x of the crosshead pin
};

// the openings of the ports of a cylinder
struct port_openings
{
	double front_admission;
	double front_exhaust;
	double back_admission;
	double back_exhaust;
};

class valve_gear
{
  public:
	valve_gear(valve_gear_parameters const &Gear, cylinder_parameters const &Cylinders);

	// the piston's way from its front dead centre, 0..stroke, and its derivative by the crank angle
	double piston(double Crank) const;
	double piston_rate(double Crank) const;
	// the linkage at the given angle of the crank and of the lifting arm of the weighshaft. the solution carries
	// on from the last one, so the angles are to change little between the calls; follow() gets there from anywhere
	valve_position solve(double Crank, double Weighshaft);
	// the linkage brought to the given angles from wherever it was, going round with the crank in small steps
	valve_position follow(double Crank, double Weighshaft);
	// forgets the last solution
	void reset();
	port_openings openings(double Valve) const;
	double steam_lap() const;
	double exhaust_lap() const;
	double port_width() const;
	double stroke() const;

  private:
	// the dimensions, in mm as on the drawing
	struct geometry
	{
		double crank;
		double connecting_rod;
		double return_crank;
		double return_crank_angle;
		double eccentric_rod;
		double link_x;
		double link_y;
		double link_foot;
		double link_foot_setback;
		double link_radius;
		double radius_rod;
		double radius_rod_tail;
		double lever_upper;
		double lever_lower;
		double valve_height;
		double valve_centre;
		double union_link;
		double crosshead_arm;
		double weighshaft_x;
		double weighshaft_y;
		double lifting_arm;
		double lifting_link;
		double steam_lap;
		double exhaust_lap;
		double port_width;
	};

	using unknowns = std::array<double, 3>; // x of the valve spindle pin, the angle of the combination lever, the die block on the link

	double crosshead(double Crank) const;
	double link_angle(double Crank);
	unknowns residuals(unknowns const &Unknowns, double Link, double Crosshead, double Weighshaft) const;
	unknowns newton_step(unknowns const &Unknowns, double Link, double Crosshead, double Weighshaft) const;

	geometry m_geometry;
	double m_link{0.0};
	unknowns m_unknowns{};
	bool m_solved{false};
	double m_crank{0.0}; // the angles of the last solution
	double m_weighshaft{0.0};
};

// the angle of the lifting arm with the die block, on the average of a revolution, in the pivot of the link
double mid_gear(valve_gear &Gear);
// the angle of the lifting arm for the given place of the reverser's nut, -1 full reverse gear .. 1 full forward
// gear, and the other way round
double weighshaft_angle(valve_gear_parameters const &Parameters, double MidGear, double Reverser);
double reverser_position(valve_gear_parameters const &Parameters, double MidGear, double Weighshaft);

// the valve events of one setting of the gear, as the valve setting table of a drawing gives them: in per cent of
// the stroke, the lengths in m
struct valve_events
{
	bool forward;
	double die_block_mean;
	double die_block_max;
	double die_block_slip;
	double valve_travel_forward;
	double valve_travel_back;
	// front end of the cylinder, back end
	std::array<double, 2> cutoff;
	std::array<double, 2> release;
	std::array<double, 2> compression;
	std::array<double, 2> admission;
	std::array<double, 2> admission_opening;
	std::array<double, 2> exhaust_opening;
};
valve_events valve_timing(valve_gear &Gear, double Weighshaft, int Steps = 1440);
// the angle of the lifting arm giving the cutoff (per cent) at the front end of the cylinder
double weighshaft_for_cutoff(valve_gear &Gear, double Cutoff, bool Forward);

} // namespace steam::detailed
