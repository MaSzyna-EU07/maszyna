/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the orders of the AI driver: a temporary operation (preparing the engine, changing the direction, coupling) and the
// way it drives (shunting, a train run), which an order can carry together (changeDirection on top of shunt)

#pragma once

#include "utilities/enumset.h"

enum class DriverOrder : unsigned
{
	waitForOrders = 0, // czekanie na dostarczenie następnych rozkazów
	// operacje tymczasowe
	prepareEngine = 1 << 0, // włączenie silnika
	releaseEngine = 1 << 1, // wyłączenie silnika
	changeDirection = 1 << 2, // zmiana kierunku (bez skanowania sygnalizacji)
	connect = 1 << 3, // podłączanie wagonów (z częściowym skanowaniem sygnalizacji)
	disconnect = 1 << 4, // odłączanie wagonów (bez skanowania sygnalizacji)
	// jazda
	shunt = 1 << 5, // tryb manewrowy
	looseShunt = 1 << 6, // coupling-free shunting mode
	obeyTrain = 1 << 7, // tryb pociągowy
	bank = 1 << 8, // assist mode
	// others
	jumpToFirstOrder = 1 << 9, // zapętlenie do pierwszej pozycji
};
template <> struct IsEnumSetKind<DriverOrder> : std::true_type
{
};
using DriverOrders = EnumSet<DriverOrder>;

// the orders the old numbers of which were from shunt up (32): they move the train, by a way of driving or the jump
inline constexpr DriverOrders movingOrders{DriverOrder::shunt | DriverOrder::looseShunt | DriverOrder::obeyTrain | DriverOrder::bank |
                                           DriverOrder::jumpToFirstOrder};
// the orders the old numbers of which were from obeyTrain up (128): the train runs, not shunting
inline constexpr DriverOrders trainOrders{DriverOrder::obeyTrain | DriverOrder::bank | DriverOrder::jumpToFirstOrder};

// the order as a single value, for a switch: a sum of several of them matches none of the cases
constexpr DriverOrder SingleOrder(DriverOrders const orders)
{
	return static_cast<DriverOrder>(orders.Bits());
}
