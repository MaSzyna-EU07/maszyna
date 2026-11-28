/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the drive of asynchronous motors fed by inverters (ElectricInductionMotor): its definition from the .fiz and its state

#pragma once

#include <array>
#include <map>
#include <vector>

// an inverter of the drive
struct Inverter
{
	double Freal = 0.0;
	double Request = 0.0;
	bool IsActive = true;
	bool Activate = true;
	bool Error = false;
	bool Failure_Drive = false;
	bool Failure_Const = false;
};

struct InductionMotor
{
	// the definition
	std::array<double, 26> config; // the parameters, indexed by eimc_* (eimc before)
	std::map<double, double> maxPowerTable; /*tablica mocy maksymalnej od predkosci*/
	int inverterCount = 0; // number of inverters
	double inverterRatio = 0.0;
	int inverterControlCouplings = 4; // the couplings needed to control the inverters, the number of InvCtrCplFlag of the .fiz
	bool logForce = false; // EIMCLogForce

	// the state
	std::array<double, 21> state; // the values of the drive, indexed by eimv_* (eimv before)
	double forceShare = 0; /*realizowany procent dostepnej sily rozruchu/hamowania*/
	double inverterFrequency{0.0}; // current frequency of power inverters
	std::vector<Inverter> inverters; // all inverters
};
