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

#include "McZapkie/hamulce.h"

#include <algorithm>

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
		// wagons without R run on P, and there may be no more than three of them, fewer than those on R
		auto const pwagons{counts.g + counts.p};
		return pwagons <= passenger_r_max_p_wagons && counts.r > pwagons ? train_setting::passenger_r : train_setting::passenger_p;
	}
	if (counts.length > freight_p_max_length || counts.mass > freight_p_max_mass) {
		return train_setting::freight_g;
	}
	if (counts.length > freight_p_all_max_length || counts.mass > freight_p_all_max_mass) {
		return train_setting::freight_gp;
	}
	return train_setting::freight_p;
}

std::vector<int> choose(std::vector<vehicle> const &Consist) {
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
				// the wagons are counted from the head of the train, past any locomotive in the middle of it
				auto const ong{item.powered || wagonsfromhead <= freight_gp_wagons_on_g};
				settings.emplace_back(ong ? pick(item.delays, bdelay_G, bdelay_P) : pick(item.delays, bdelay_P, bdelay_G));
				break;
			}
			case train_setting::freight_g:
			default: {
				// NOTE: Cw-1 §19.9 wants the brake of a vehicle without G (a passenger car with P and R only) cut out in a
				// train braked on G. It's put on P instead, as before: a brake cut out by the inspector would have to be
				// remembered and put back when the train is made up again, and the simulator has no such record yet.
				// The same goes for the vehicles kept on G in the long locomotive setting above (freight_gp)
				settings.emplace_back(pick(item.delays, bdelay_G, bdelay_P));
				break;
			}
		}
	}
	return settings;
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

} // namespace brake_setting
