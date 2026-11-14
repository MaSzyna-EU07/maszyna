/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// the brake table the AI keeps of its consist (TController::fBrake_a0, fBrake_a1), as the braking curve reads it.
// entries 1 to size go by speed, up to the vehicle's top speed (entry 0 is the one for the present speed).
// TController::CheckVehicles() zeroes the table and fills it in for a train run only (Obey_train, Bank): in
// shunting, and under Connect, it stays all zeroes
struct consist_brake_table {
	double const *a0; // deceleration at handle position 1, by speed
	double const *a1; // deceleration added by each quarter of a position, by speed
	int size; // entries, up to the vehicle's top speed
	double vmax; // the vehicle's top speed [km/h]

	// deceleration [m/s2] of a full service application at Kmh. a zeroed table reads as 0.01, next to nothing
	double full_braking(double Kmh) const;
	// the table holds the brakes of the consist: it was filled in, not left zeroed. a braking curve shaped by a
	// zeroed table takes the brakes for next to nothing and asks for a few hundredths of a m/s2, which is no
	// braking at all, so the train coasts on to the final approach and can't stop there in time
	bool known() const;
};
