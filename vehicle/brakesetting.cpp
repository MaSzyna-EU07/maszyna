/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/brakesetting.h"

#include "McZapkie/MOVER.h"
#include "McZapkie/hamulce.h"
#include "vehicle/Driver.h"

#include <algorithm>
#include <cmath>

namespace brake_setting {

namespace {

// the limits of Cw-1 §19.2 and §19.3 for the wagons of a freight train (the "skład pociągu", without
// the locomotives)
constexpr double freight_p_max_length{500.0}; // [m]
constexpr double freight_p_max_mass{1300000.0}; // [kg]
constexpr double freight_p_all_max_length{300.0}; // [m]
constexpr double freight_p_all_max_mass{600000.0}; // [kg]

// the most P-braked vehicles a passenger train on R may carry (Cw-1 §18.3, §18.4)
constexpr int passenger_r_max_p_wagons{3};

bool has(int const Delays, int const Setting) {
	return (Delays & Setting) == Setting;
}

// Wanted when the vehicle has it, else Fallback when it has that, else 0
int pick(int const Delays, int const Wanted, int const Fallback) {
	if (has(Delays, Wanted)) {
		return Wanted;
	}
	if (has(Delays, Fallback)) {
		return Fallback;
	}
	return 0;
}

// the wagons of the train by the fastest setting they have
struct wagon_counts {
	int r{0}; // with R: passenger cars of fast trains
	int g{0}; // with G but no R: freight wagons
	int p{0}; // the rest: passenger cars with P only
	int r_cut_out{0}; // of those with R, the ones with their brake cut out
	double length{0.0}; // [m]
	double mass{0.0}; // [kg]

	explicit wagon_counts(std::vector<vehicle> const &Consist) {
		for (auto const &item : Consist) {
			if (item.powered) {
				continue;
			}
			length += item.length;
			mass += item.mass;
			if (has(item.delays, bdelay_R)) {
				++r;
				if (item.cut_out) {
					++r_cut_out;
				}
			}
			else if (has(item.delays, bdelay_G)) {
				++g;
			}
			else {
				++p;
			}
		}
	}

	int wagons() const {
		return r + g + p;
	}
	// a train is a passenger one unless it has at least as many freight wagons as there are others,
	// or four of them (the inspector's rule of the earlier releases, Cw-1 doesn't tell)
	bool passenger() const {
		return g < std::min(4, r + p);
	}
};

int passenger_r_setting(vehicle const &Item) {
	if (false == Item.powered && has(Item.delays, bdelay_R | bdelay_M)) {
		return bdelay_R | bdelay_M;
	}
	return pick(Item.delays, bdelay_R, bdelay_P);
}

} // namespace

train_setting choose_train_setting(std::vector<vehicle> const &Consist) {
	wagon_counts const counts{Consist};
	if (counts.wagons() == 0) {
		return train_setting::light_engine;
	}
	if (counts.passenger()) {
		// wagons without R run on P, and a wagon with its brake cut out counts as one on P (Cw-1 §18.3). there may
		// be no more than three of them, fewer than those on R
		auto const pwagons{counts.g + counts.p + counts.r_cut_out};
		auto const rwagons{counts.r - counts.r_cut_out};
		return pwagons <= passenger_r_max_p_wagons && rwagons > pwagons ? train_setting::passenger_r : train_setting::passenger_p;
	}
	if (counts.length > freight_p_max_length || counts.mass > freight_p_max_mass) {
		return train_setting::freight_g;
	}
	if (counts.length > freight_p_all_max_length || counts.mass > freight_p_all_max_mass) {
		return train_setting::freight_gp;
	}
	return train_setting::freight_p;
}

std::vector<int> choose(std::vector<vehicle> const &Consist, bool const Train) {
	auto const setting{choose_train_setting(Consist)};
	std::vector<int> settings;
	settings.reserve(Consist.size());
	auto wagonsfromhead{0};
	for (auto const &item : Consist) {
		if (false == item.powered) {
			++wagonsfromhead;
		}
		switch (setting) {
			case train_setting::light_engine: {
				settings.emplace_back(pick(item.delays, bdelay_R, bdelay_P));
				break;
			}
			case train_setting::passenger_r: {
				settings.emplace_back(passenger_r_setting(item));
				break;
			}
			case train_setting::passenger_p: {
				settings.emplace_back(pick(item.delays, bdelay_P, bdelay_P));
				break;
			}
			case train_setting::freight_p: {
				settings.emplace_back(item.powered ? pick(item.delays, bdelay_G, bdelay_P) : pick(item.delays, bdelay_P, bdelay_G));
				break;
			}
			case train_setting::freight_gp: {
				// the wagons are counted from the head of the train, past any locomotive in the middle of it.
				// the train runs on P (Cw-1 §19.3b), so §19.9 doesn't apply: a vehicle without G among the ones
				// kept on G brakes on P like the rest of the train, rather than being cut out
				auto const ong{item.powered || wagonsfromhead <= freight_gp_wagons_on_g};
				settings.emplace_back(ong ? pick(item.delays, bdelay_G, bdelay_P) : pick(item.delays, bdelay_P, bdelay_G));
				break;
			}
			case train_setting::freight_g:
			default: {
				if (has(item.delays, bdelay_G)) {
					settings.emplace_back(bdelay_G);
				}
				else if (item.powered || item.driven || item.delays == 0 || false == Train) {
					// a locomotive, or the vehicle the train is driven from, keeps a working brake: on P. a vehicle
					// which tells no settings at all is left alone. a consist being shunted isn't a train (a bank
					// engine's run is) and §19.9 doesn't apply to it: its wagons keep their brakes, on P
					settings.emplace_back(pick(item.delays, bdelay_P, bdelay_P));
				}
				else {
					// a wagon without G (a passenger car with P and R) has its brake cut out (Cw-1 §19.9)
					settings.emplace_back(cut_out);
				}
				break;
			}
		}
	}
	return settings;
}

bool runs_as_train(int const Orders) {
	return (Orders & (Obey_train | Bank)) != 0;
}

bool inspects_train(int const Orders, bool const Coupled) {
	return (Orders & (Shunt | Loose_shunt | Obey_train | Bank)) != 0 || ((Orders & Connect) != 0 && false == Coupled);
}

bool at_standstill(double const Velocity) {
	return std::abs(Velocity) < EU07_AI_MOVEMENT;
}

std::vector<int> choose_legacy(std::vector<vehicle> const &Consist) {
	wagon_counts const counts{Consist};
	auto length{0.0};
	auto mass{0.0};
	for (auto const &item : Consist) {
		length += item.length;
		mass += item.mass;
	}
	auto setting{train_setting::light_engine};
	if (counts.wagons() > 0) {
		if (counts.passenger()) {
			setting = counts.g && counts.r < counts.g + counts.p ? train_setting::passenger_p : train_setting::passenger_r;
		}
		else if (length < 300.0 && mass < 600000.0) {
			setting = train_setting::freight_p;
		}
		else if (length < 500.0 && mass < 1300000.0) {
			setting = train_setting::freight_gp;
		}
		else {
			setting = train_setting::freight_g;
		}
	}
	std::vector<int> settings;
	settings.reserve(Consist.size());
	auto wagonsfromloco{0};
	for (auto const &item : Consist) {
		switch (setting) {
			case train_setting::light_engine:
			case train_setting::passenger_r: {
				settings.emplace_back(has(item.delays, bdelay_R) ? bdelay_R : bdelay_P);
				break;
			}
			case train_setting::passenger_p: {
				settings.emplace_back(bdelay_P);
				break;
			}
			case train_setting::freight_p: {
				settings.emplace_back(item.powered ? bdelay_G : bdelay_P);
				break;
			}
			case train_setting::freight_gp: {
				// the count started anew at every locomotive
				if (item.powered) {
					wagonsfromloco = 0;
					settings.emplace_back(bdelay_G);
				}
				else {
					++wagonsfromloco;
					settings.emplace_back(wagonsfromloco <= freight_gp_wagons_on_g ? bdelay_G : bdelay_P);
				}
				break;
			}
			case train_setting::freight_g:
			default: {
				settings.emplace_back(has(item.delays, bdelay_G) ? bdelay_G : bdelay_P);
				break;
			}
		}
	}
	return settings;
}

bool is_cut_out(TMoverParameters const &Vehicle) {
	return (Vehicle.Hamulec->GetBrakeStatus() & b_dmg) == b_dmg;
}

vehicle describe(TMoverParameters const &Vehicle, bool const Driven) {
	vehicle described;
	described.delays = Vehicle.BrakeDelays;
	// a locomotive has Power > 1, though it may be a dead one at the end of the train
	described.powered = Vehicle.Power >= 1.0;
	described.length = Vehicle.Dim.L;
	described.mass = Vehicle.TotalMass;
	described.cut_out = is_cut_out(Vehicle) && false == Vehicle.BrakeCutOutByInspector;
	described.driven = Driven;
	return described;
}

bool apply(TMoverParameters &Vehicle, int const Setting, bool const Standstill) {
	auto &brake{*Vehicle.Hamulec};
	if (Vehicle.BrakeCutOutByInspector && false == is_cut_out(Vehicle)) {
		// someone put the brake back in the meantime, it's theirs now
		Vehicle.BrakeCutOutByInspector = false;
	}
	if (Setting == cut_out) {
		if (is_cut_out(Vehicle)) {
			// a brake cut out already, by the inspector or for a reason of its own, stays as it is
			return false;
		}
		if (false == Standstill) {
			// on the run the vehicle is left alone, brake and setting, until the inspector looks at the train
			// standing: the caller is told to come back
			return true;
		}
		// close the cut-out cock and empty the brake with the releaser, as the scenery's '0' does: the
		// distributor, cut off the brake pipe, has no air left to brake with and lets the cylinders go
		brake.ForceEmptiness();
		brake.SetBrakeStatus(brake.GetBrakeStatus() | b_dmg);
		Vehicle.BrakeCutOutByInspector = true;
		return false;
	}
	if (Vehicle.BrakeCutOutByInspector) {
		// open the cut-out cock: the distributor charges from the brake pipe again
		brake.SetBrakeStatus(brake.GetBrakeStatus() & ~b_dmg);
		Vehicle.BrakeCutOutByInspector = false;
	}
	if (Setting != 0) {
		Vehicle.BrakeDelaySwitch(Setting);
	}
	return false;
}

} // namespace brake_setting
