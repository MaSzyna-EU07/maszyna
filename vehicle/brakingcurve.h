/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// the deceleration the AI asks for on its way down to a lower speed or a stop ahead, outside of the final
// approach (TController::adjust_desired_speed_for_braking_curve)
namespace braking_curve {

// the deceleration table of the consist brake, by speed (TController::fBrake_a0, fBrake_a1). it is worked out
// for a train only (Obey_train, Bank); in shunting it stays at zero
struct brake_table {
	double const *a0; // deceleration at handle position 1, by speed; a0[1..size]
	double const *a1; // deceleration added by each quarter of a position, by speed
	int size; // entries, up to the vehicle's top speed
	double vmax; // the vehicle's top speed [km/h]

	// deceleration [m/s2] of a full service application at Kmh
	double full_braking(double Kmh) const;
	// whether the table is worked out
	bool known() const;
};

// resultant acceleration [m/s2] to ask for now so that a steady application of the pneumatic brake takes the train
// from Velocity down to Target [km/h] over Distance [m], with the brake force following the speed the way the
// table says it does (cast iron blocks grip harder as the train slows)
double shaped_demand(double Velocity, double Target, double Distance, double Gravity, brake_table const &Table);

// where the train is on its way down to Target
struct approach {
	double speed; // present speed in the direction of travel [km/h]
	double target; // speed to get down to [km/h]
	double distance; // to where the speed is to be down [m]
	double offset; // the braking point that far short of it [m]
	double gravity; // tangential acceleration of the gradient [m/s2], negative uphill
	double acceleration; // present acceleration of the train [m/s2]
	bool brake_applied; // the train brake handle is past running
	double speedup_horizon; // how long the present acceleration lasts while speeding up [s]
	double fill_horizon; // how long the brake takes to fill once applied [s]
};

// acceleration [m/s2] to ask for now, negative to brake. with the worked out table of a pneumatic brake
// (Pneumatictable) the demand follows the brake: planned from the state the train is in once the brake has
// responded, along the shape of its force over speed. without one (nullptr for another brake, or a table left at
// zero, as in shunting) it is the constant deceleration over the distance left
double demand(approach const &Approach, brake_table const *Pneumatictable);

} // namespace braking_curve
