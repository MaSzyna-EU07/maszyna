#pragma once

#include "mover/core.h"

namespace mover
{
struct spring_brake
{
	std::shared_ptr<TReservoir> Cylinder;
	bool Activate{false}; // Input: switching brake on/off in exploitation - main valve/switch
	bool ShuttOff{true}; // Input: shutting brake off during failure - valve in pneumatic container
	bool Release{false}; // Input: emergency releasing rod

	bool IsReady{false}; // Output: readyness to braking - cylinder is armed, spring is tentioned
	bool IsActive{false}; // Output: brake is working
	double SBP{0.0}; // Output: pressure in spring brake cylinder

	bool PNBrakeConnection{false}; // Conf: connection to pneumatic brake cylinders
	double MaxSetPressure{0.0}; // Conf: Maximal pressure for switched off brake
	double ResetPressure{0.0}; // Conf: Pressure for arming brake cylinder
	double MinForcePressure{0.1}; // Conf: Minimal pressure for zero force
	double MaxBrakeForce{0.0}; // Conf: Maximal tension for brake pads/shoes
	double PressureOn{-2.0}; // Conf: Pressure changing ActiveFlag to "On"
	double PressureOff{-1.0}; // Conf: Pressure changing ActiveFlag to "Off"
	double ValveOffArea{0.0}; // Conf: Area of filling valve
	double ValveOnArea{0.0}; // Conf: Area of dumping valve
	double ValvePNBrakeArea{0.0}; // Conf: Area of bypass to brake cylinders

	int MultiTractionCoupler{127}; // Conf: Coupling flag necessary for transmitting the command
};

} // namespace mover
