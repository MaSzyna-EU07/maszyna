/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// a standing train starts only with its brakes charged, as a driver checks before setting off: the brake pipe and the
// auxiliary reservoir of every vehicle with a working brake close to the working pressure. released alone isn't enough:
// a train prepared from cold is released at once, its pipe still filling, and has no air to brake with
// (TController::PrepareEngine(), TController::control_brake_charge())
namespace brake_charge {

// [bar] below the working pressure the pipe and the reservoir count as charged: 4.5 of 5.0. the last wagon of a long
// train stays at about 4.6-4.7 for good, from what leaks along the pipe
auto const margin{0.5};
// below this the pipe counts as near empty (after an emergency application): the next start waits for the charge
// whatever came before. well below what a service application leaves: the AI stops at a station with the handle past
// the full service position, the pipe at 3.0-3.5 bar, and wagons coupled from cold come with about 3 bar. waiting there
// only delays the train, and the scenery sets the routes of the others later (calkowo_v2_osoba_jar_poranek, l053_poranek)
auto const empty_level{2.5}; // [bar]
// a charge that rises by less than this in stall_time has stopped (a leak, a hose not connected, a pipe cock shut): the
// train goes all the same, rather than stand for good
auto const stall_gain{0.02}; // [bar]
auto const stall_time{60.0}; // [s]
// standing, the locomotive brake alone has to give this multiple of the grade force to hold the train while it charges
auto const holding_force{1.1};

struct state {
	double best{-1.0}; // the highest charge seen while it still rose [bar], -1 before the first look
	double best_time{0.0}; // when it was seen [s]
	bool stalled{false}; // the charge stopped rising short of charged
	// the train was let go: it stays so until the driver clears this, when the train is to stand again. otherwise a
	// charge rising again as the train starts (the pipe refilling behind the release) would stop it at a crawl
	bool going{false};
};

// Level: the lowest pressure of the brake pipes and auxiliary reservoirs among the vehicles with a working brake [bar];
// Reference: the pressure they charge to [bar]; Time: now [s]; Supplied: there is air to charge with (the engine
// prepared, the main reservoir above the reference): without it the charge can't rise, and that isn't a stall.
// true once the train may start, and from then on
bool charged(state &State, double Level, double Reference, double Time, bool Supplied = true);

// whether the locomotive brake, Locomotivebrakeforce at full application [N], holds the standing train against its
// Gradeforce [N] on its own; when it doesn't, the train brake is applied lightly while the train charges
bool locomotive_brake_holds(double Locomotivebrakeforce, double Gradeforce);

// whether a release of the brakes waits for the charge: standing, a train not charged yet keeps its locomotive brake
// on. the train brake (Trainbrakeapplied) may come off all the same: released, it fills the pipe, which is what charges
// the train. the hint a human driver gets to release the brakes waits with it (driver_hint::waitbrakecharge instead)
bool release_waits(bool Charged, bool Standing, bool Trainbrakeapplied);

} // namespace brake_charge
