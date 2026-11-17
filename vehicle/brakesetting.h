/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <vector>

class TMoverParameters;

// the brake setting (G/P/R/Mg) the virtual wagon inspector (TController::AutoRewident) chooses for every
// vehicle of a train, by the rules of the brake instruction Cw-1 (Mw56), §18 and §19 (eu07/maszyna#350)
namespace brake_setting {

// what the inspector needs to know about a vehicle of the train
struct vehicle {
	int delays{0}; // the settings the vehicle has (BrakeDelays, a combination of bdelay_*)
	bool powered{false}; // a locomotive or a powered unit, as opposed to a wagon
	double length{0.0}; // [m]
	double mass{0.0}; // with its load [kg]
	bool cut_out{false}; // the brake is cut out for a reason of its own (damage, the scenario, the player)
	bool driven{false}; // the vehicle the train is driven from: its brake is never cut out
};

// the kind of train the inspector takes the consist for, and how it sets the brakes
enum class train_setting {
	light_engine, // no wagons: R where there is one, else P
	passenger_r, // passenger train on R, wagons with Mg on R+Mg (Cw-1 §18.2)
	passenger_p, // passenger train on P (Cw-1 §18.3, §18.4)
	freight_p, // freight train on P: locomotives on G, wagons on P (Cw-1 §19.3a)
	freight_gp, // freight train on P: locomotives and the first five wagons on G, the rest on P (Cw-1 §19.3b)
	freight_g // freight train on G (Cw-1 §19.1)
};

// how many wagons next to the locomotive stay on G in a freight_gp train
inline constexpr int freight_gp_wagons_on_g{5};

// the setting choose() gives a vehicle whose brake is to be cut out (Cw-1 §19.9)
inline constexpr int cut_out{-1};

// the train setting by Cw-1. Consist runs from the head of the train to its rear
train_setting choose_train_setting(std::vector<vehicle> const &Consist);

// a passenger train or a light engine, as opposed to a freight train. the inspector of the earlier
// releases tells them apart the same way
constexpr bool passenger(train_setting const Setting) {
	return Setting == train_setting::light_engine || Setting == train_setting::passenger_r || Setting == train_setting::passenger_p;
}

// the brake setting (bdelay_*) for every vehicle of Consist, in the same order, by Cw-1. a vehicle
// without the setting the train needs gets the other one of G and P it has, and 0 (left as it is)
// when it has neither. In a train on G (freight_g) a wagon without G gets cut_out instead: its brake
// is to be cut out (Cw-1 §19.9). A locomotive and the vehicle the train is driven from keep their
// brake working, on P. Train tells the AI runs the consist as a train (runs_as_train()): §19.9 is about
// trains, so in shunting nothing is cut out and a wagon without G brakes on P, which puts back a brake
// the inspector cut out before
std::vector<int> choose(std::vector<vehicle> const &Consist, bool Train);

// the AI runs the consist as a train, rather than shunting it: Obey_train, or Bank, which it drives as a
// train run too (with the braking of a train). Orders is the order of the AI (TOrders)
bool runs_as_train(int Orders);

// the train stands for the inspector: slower than the AI takes for a deliberate movement (EU07_AI_MOVEMENT).
// the AI looks at the train while it still creeps to a stop (at a W4, or switching from shunting to the train
// run), so a stricter threshold would miss those checks; a train really on the run (an order changed mid-run)
// stays above it, and so does one being formed by coupling: the AI drives onto the standing cut faster than
// that and looks at the new train at the first buffer contact. apply() tells of the cut-outs such a look
// left for the next one, at a standstill
bool at_standstill(double Velocity);

// the inspector of the releases before eu07/maszyna#350, kept for ai.brakesetting.cw1 no. the settings
// are requests: a vehicle which doesn't have the one asked for keeps its own
std::vector<int> choose_legacy(std::vector<vehicle> const &Consist);

// the brake of the vehicle is cut out (b_dmg), whoever did it
bool is_cut_out(TMoverParameters const &Vehicle);

// what choose() needs to know of the vehicle. a brake the inspector cut out itself doesn't count as
// cut out: the inspector decides about it anew
vehicle describe(TMoverParameters const &Vehicle, bool Driven);

// the inspector puts the vehicle on Setting, as choose() gives it. cut_out cuts the brake out the way
// the scenery's '0' does: b_dmg, which cuts the distributor off the brake pipe (as the cab's cut-out,
// trainbrakeoperationtoggle, does), and its reservoirs emptied, so the brake releases and stays off.
// The vehicle remembers the inspector did it. A brake is cut out only with the train at a standstill
// (Standstill): on the run the vehicle is left as it is until the inspector looks at the train again
// standing, rather than losing its brake while moving. Any other setting first puts back a brake the
// inspector cut out (it charges from the brake pipe again), then sets the vehicle to it (0: leaves the
// setting as it is). A brake cut out by anyone else stays cut out.
// Returns true when a cut-out was left for later because the train was moving: the inspector has to look
// at the train again once it stands (TController::reinspect_brakes() does)
bool apply(TMoverParameters &Vehicle, int Setting, bool Standstill);

} // namespace brake_setting
