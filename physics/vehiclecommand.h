/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the commands the vehicles of a train pass to each other through their couplings (the control wire of the multiple
// unit, the brake pipe and the others), and the ones a scenery, a script or the driver gives a vehicle. inside the
// simulation they are VehicleCommand; their names are only read where a command comes as text (events, Lua) and
// written where it is shown (the log, the AI)

#pragma once

#include <optional>
#include <string>
#include <string_view>

enum class VehicleCommand
{
	none, // no command
	unknown, // a command given as text which no vehicle knows: the AI may still read it
	// the controllers
	mainController, // MainCtrl
	secondaryController, // ScndCtrl
	integratedController, // EIMIC: the integrated controller of an induction motor drive, relative traction or braking
	speedControl, // SpeedCntrl
	speedControlParking, // SpeedCtrlUnit.Parking
	// the direction, the automatic start and the current relays
	direction, // Direction
	autoRelaySwitch, // AutoRelaySwitch
	maxCurrentSwitch, // MaxCurrentSwitch
	minCurrentSwitch, // MinCurrentSwitch
	relayReset, // RelayReset
	// the cab
	cabActivation, // CabActivisation
	cabSignal, // CabSignal
	departureSignal, // DepartureSignal
	tractionMode, // TractionMode
	// the brakes
	brake, // Brake
	emergencyBrake, // Emergency_brake
	brakeDelay, // BrakeDelay
	brakeReleaser, // BrakeReleaser
	springBrakeActivate, // SpringBrakeActivate
	epFuseSwitch, // EpFuseSwitch: passed on, but no vehicle acts on it
	sandbox, // Sandbox
	sandboxAutoAllow, // SandboxAutoAllow: passed on, but no vehicle acts on it
	// the compressor and the converter
	compressorSwitch, // CompressorSwitch
	compressorPreset, // CompressorPreset
	converterSwitch, // ConverterSwitch
	// the power
	batterySwitch, // BatterySwitch
	mainSwitch, // MainSwitch
	heatingSwitch, // HeatingSwitch
	gensetSwitch, // GensetSwitch
	pantographValve, // PantValve
	pantographsValve, // PantsValve
	pantographsAllDown, // PantAllDown
	compartmentLightsSwitch, // CompartmentLightsSwitch
	compartmentLightsSwitchOff, // CompartmentLightsSwitchOff
	// the pumps of a diesel engine
	waterPumpBreakerSwitch, // WaterPumpBreakerSwitch
	waterPumpSwitch, // WaterPumpSwitch
	waterPumpSwitchOff, // WaterPumpSwitchOff
	waterHeaterBreakerSwitch, // WaterHeaterBreakerSwitch
	waterHeaterSwitch, // WaterHeaterSwitch
	waterCircuitsLinkSwitch, // WaterCircuitsLinkSwitch
	fuelPumpSwitch, // FuelPumpSwitch
	fuelPumpSwitchOff, // FuelPumpSwitchOff
	oilPumpSwitch, // OilPumpSwitch
	oilPumpSwitchOff, // OilPumpSwitchOff
	// the traction motor blowers
	motorBlowersFrontSwitch, // MotorBlowersFrontSwitch
	motorBlowersFrontSwitchOff, // MotorBlowersFrontSwitchOff
	motorBlowersRearSwitch, // MotorBlowersRearSwitch
	motorBlowersRearSwitchOff, // MotorBlowersRearSwitchOff
	// the doors
	doorOpen, // DoorOpen
	doorClose, // DoorClose
	doorLock, // DoorLock
	doorMode, // DoorMode
	doorPermit, // DoorPermit
	doorStep, // DoorStep
	// the load and the damage
	load, // Load=<name of the load>
	unload, // UnLoad=<name of the load>
	setDamage, // SetDamage
};

// a command read from its text, with the name of the load for Load= and UnLoad=
struct ParsedVehicleCommand
{
	VehicleCommand command{VehicleCommand::none};
	std::string argument; // the name of the load, as extract_value() gives it
};

// the command a text names: none for an empty text, unknown for a text no vehicle knows. the names are the ones the
// scenery files, the scripts and the AI have always used, compared exactly
ParsedVehicleCommand ParseVehicleCommand(std::string const &Text);
// the name of a command, as the scenery files write it; empty for none and unknown
std::string_view VehicleCommandName(VehicleCommand Command);
