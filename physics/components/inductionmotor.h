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
#include <string_view>
#include <utility>
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

// the motor itself (the s_ entries of eimc before)
struct InductionMotorWinding
{
	double currentPerSlip = 0.0; // dfic: the torque current per hertz of slip, at full flux
	double maxSlipFrequency = 0.0; // dfmax: the most slip the drive allows [Hz]
	double polePairs = 0.0; // p: the pairs of poles
	double voltsPerHertz = 0.0; // cfu: the voltage per hertz of full flux
	double torquePerCurrent = 0.0; // cim: the torque per unit of flux and torque current
	double fieldCurrentRatio = 0.0; // icif: the field current per unit of torque current
};

// the inverter feeding the motors (the f_ entries of eimc before)
struct InductionMotorConverter
{
	double maxLinkVoltage = 0.0; // Uzmax: the most voltage the inverter gives the motors in traction [V]
	double brakingLinkVoltage = 0.0; // Uzh: the same in electrodynamic braking [V]
	double voltageDrop = 0.0; // DU: the voltage lost in the inverter [V]
	double noLoadCurrent = 0.0; // I0: the current the drive draws with no load [A]
	double voltsPerHertz = 0.0; // fcfu: the voltage per hertz the inverter gives in traction
	double brakingVoltsPerHertz = 0.0; // fcfuH: the same in electrodynamic braking, fcfu without it
};

// what the drive gives in traction (the p_ entries of eimc before)
struct InductionMotorTraction
{
	double startingForce = 0.0; // F0: the force at a standstill [kN]
	double forceDropPerSpeed = 0.0; // a1: how much of the starting force each km/h takes away [kN/(km/h)]
	double maxPower = 0.0; // Pmax: the most power [kW]; follows the table of the most power by the speed, where there is one
	double maxCurrent = 0.0; // Imax: the most current of the motors [A]
};

// the electrodynamic brake of the drive (the p_ entries of eimc before)
struct InductionMotorBraking
{
	double maxForce = 0.0; // Fh: the most braking force [kN]
	double maxPower = 0.0; // Ph: the most braking power [kW]
	double offSpeed = 0.0; // Vh0: below this speed the electrodynamic brake gives nothing [km/h]
	double fullSpeed = 0.0; // Vh1: above this speed the electrodynamic brake gives all it can; between the two it fades [km/h]
	double cylinderPressureGain = 0.0; // abed: the share of the brake cylinder pressure the electrodynamic brake takes; 0: the air brake cuts it off
	double maxEpDeceleration = 1.5; // edep: the most the electro-pneumatic brake of a vehicle decelerates its mass before the force goes elsewhere [m/s2]
};

// the values of the model of the drive in the step (eimv before)
struct InductionMotorState
{
	// the forces [kN]
	double absoluteMaxForce = 0.0; // FMAXMAX: the most force the motors can give at the present speed and voltage
	double maxForce = 0.0; // Fmax: the force at the present setting
	double fullForce = 0.0; // Fful: the force at a full setting
	double force = 0.0; // Fr: the force the motors give
	double forceDemand = 0.0; // Fzad: the share of the force asked of the drive, negative in braking
	// the frequencies
	double slipShare = 0.0; // ks: the force against the most there is, the share of the most slip
	double slipFrequency = 0.0; // df [Hz]
	double supplyFrequency = 0.0; // fp: the frequency the inverter gives the motors [Hz]
	double criticalFrequency = 0.0; // fkr: above this frequency the flux weakens [Hz]
	// the motors
	double linkVoltage = 0.0; // Uzsmax: the most voltage the inverter can give now [V]
	double voltage = 0.0; // U: the voltage of the motors [V]
	double flux = 0.0; // pole: the flux against the full one
	double torqueCurrent = 0.0; // Ic
	double fieldCurrent = 0.0; // If
	double torque = 0.0; // M: of a motor
	double lineCurrent = 0.0; // Ipoj: the current of the vehicle from the line [A]
	double currentLimit = 0.0; // Imax: never set; shown in the debug panel
	// the powers [kW]
	double powerLimit = 0.0; // Pmax: the most power now, of traction or of braking
	double mechanicalPower = 0.0; // Pm
	double electricalPower = 0.0; // Pe
	double efficiency = 0.0; // eta
};

struct InductionMotor
{
	using Listing = std::array<std::pair<std::string_view, double>, 21>;

	// the definition
	InductionMotorWinding motor;
	InductionMotorConverter converter;
	InductionMotorTraction traction;
	InductionMotorBraking braking;
	std::map<double, double> maxPowerTable; /*tablica mocy maksymalnej od predkosci*/
	int inverterCount = 0; // number of inverters
	double inverterRatio = 0.0;
	int inverterControlCouplings = 4; // the couplings needed to control the inverters, the number of InvCtrCplFlag of the .fiz
	bool logForce = false; // EIMCLogForce

	// the state
	InductionMotorState state;
	double forceShare = 0; /*realizowany procent dostepnej sily rozruchu/hamowania*/
	double inverterFrequency{0.0}; // current frequency of power inverters
	std::vector<Inverter> inverters; // all inverters

	// the definition and the state with their labels, for the debug panel
	Listing ConfigListing() const
	{
		return {{{"dfic: ", motor.currentPerSlip},
		         {"dfmax:", motor.maxSlipFrequency},
		         {"p:    ", motor.polePairs},
		         {"scfu: ", motor.voltsPerHertz},
		         {"cim:  ", motor.torquePerCurrent},
		         {"icif: ", motor.fieldCurrentRatio},
		         {"Uzmax:", converter.maxLinkVoltage},
		         {"Uzh:  ", converter.brakingLinkVoltage},
		         {"DU:   ", converter.voltageDrop},
		         {"I0:   ", converter.noLoadCurrent},
		         {"fcfu: ", converter.voltsPerHertz},
		         {"F0:   ", traction.startingForce},
		         {"a1:   ", traction.forceDropPerSpeed},
		         {"Pmax: ", traction.maxPower},
		         {"Fh:   ", braking.maxForce},
		         {"Ph:   ", braking.maxPower},
		         {"Vh0:  ", braking.offSpeed},
		         {"Vh1:  ", braking.fullSpeed},
		         {"Imax: ", traction.maxCurrent},
		         {"abed: ", braking.cylinderPressureGain},
		         {"eped: ", braking.maxEpDeceleration}}};
	}
	Listing StateListing() const
	{
		return {{{"Fkrt:", state.absoluteMaxForce},
		         {"Fmax:", state.maxForce},
		         {"ks:  ", state.slipShare},
		         {"df:  ", state.slipFrequency},
		         {"fp:  ", state.supplyFrequency},
		         {"Us:  ", state.voltage},
		         {"pole:", state.flux},
		         {"Ic:  ", state.torqueCurrent},
		         {"If:  ", state.fieldCurrent},
		         {"M:   ", state.torque},
		         {"Fr:  ", state.force},
		         {"Ipoj:", state.lineCurrent},
		         {"Pm:  ", state.mechanicalPower},
		         {"Pe:  ", state.electricalPower},
		         {"eta: ", state.efficiency},
		         {"fkr: ", state.criticalFrequency},
		         {"Uzsm:", state.linkVoltage},
		         {"Pmax:", state.powerLimit},
		         {"Fzad:", state.forceDemand},
		         {"Imax:", state.currentLimit},
		         {"Fful:", state.fullForce}}};
	}
};
