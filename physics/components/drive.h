/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the drive of a vehicle, by the kind of its engine (EngineType, the Engine section of the .fiz): what each kind of
// drive does in the step of the vehicle, in place of a switch on the engine type in each of those places. the drives
// hold no state of their own yet (it is in TMoverParameters): there is one of each kind, shared by the vehicles

#pragma once

#include "physics/vehicletypes.h"

class TMoverParameters;

// what the step of the traction gives the drive
struct DriveStep
{
	double dt{0.0}; // the length of the step [s]
	double posRatio{0.0}; // the share of the controller positions, of the drive of no engine model (Dumb)
	double revolutions{0.0}; // the revolutions of the engine
	double moment{0.0}; // the moment of the diesel engine
	double transmission{0.0}; // the ratio of the transmission of the diesel engine
};

class TractionDrive
{
public:
	virtual ~TractionDrive() = default;

	// the power of the engine for the controller positions; returns the share of the positions (Dumb), 0 for the others
	virtual double UpdateEnginePower(TMoverParameters &vehicle) const;
	// before the traction force, whether the reverser is set or not
	virtual void BeforeTractionForce(TMoverParameters &vehicle, double dt) const;
	// the traction force of the step (Ft), the reverser set
	virtual void UpdateTractionForce(TMoverParameters &vehicle, DriveStep const &step) const;
	// after the traction force, whether the reverser is set or not
	virtual void AfterTractionForce(TMoverParameters &vehicle) const;
	// the blowers of the starting resistors and the fans of the radiators
	virtual void UpdateVentilators(TMoverParameters &vehicle, double dt) const;
	// the current the given ammeter of the cab shows
	virtual double ShowCurrent(TMoverParameters const &vehicle, int ammeter) const;
	// the revolutions of the engine against the most it gives at the present controller position, 0..1; 1 with no
	// engine of revolutions
	virtual double EngineRpmRatio(TMoverParameters const &vehicle) const;
	// the revolutions of the engine idling and at most [rpm]; the largest double with no engine of revolutions
	virtual double EngineIdleRpm(TMoverParameters const &vehicle) const;
	virtual double EngineMaxRpm(TMoverParameters const &vehicle) const;
	// the revolutions of the engine in the step (enrot); returns the revolutions a diesel-electric engine aims for, 0 for
	// the others. a diesel engine with a mechanical transmission gives its ratio and its moment through the references
	virtual double UpdateEngineRevolutions(TMoverParameters &vehicle, double dt, double &transmission, double &moment) const;
	// a notch up and down of the master controller, as the kind of the engine takes it; return whether it moved
	virtual bool IncreaseMainController(TMoverParameters &vehicle, int speed) const;
	virtual bool DecreaseMainController(TMoverParameters &vehicle, int speed) const;
};

// the drive of the given kind of engine
TractionDrive const &DriveOf(TEngineType type);
