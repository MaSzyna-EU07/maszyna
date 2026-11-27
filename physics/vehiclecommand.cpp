/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "physics/vehiclecommand.h"

#include <algorithm>
#include <array>
#include <utility>

#include "utilities/utilities.h"

namespace
{

using enum VehicleCommand;

// the names of the commands, as the scenery files write them
constexpr std::array<std::pair<VehicleCommand, std::string_view>, 59> commandNames{{
    {mainController, "MainCtrl"},
    {secondaryController, "ScndCtrl"},
    {integratedController, "EIMIC"},
    {speedControl, "SpeedCntrl"},
    {speedControlParking, "SpeedCtrlUnit.Parking"},
    {direction, "Direction"},
    {autoRelaySwitch, "AutoRelaySwitch"},
    {maxCurrentSwitch, "MaxCurrentSwitch"},
    {minCurrentSwitch, "MinCurrentSwitch"},
    {relayReset, "RelayReset"},
    {cabActivation, "CabActivisation"},
    {cabSignal, "CabSignal"},
    {departureSignal, "DepartureSignal"},
    {tractionMode, "TractionMode"},
    {brake, "Brake"},
    {emergencyBrake, "Emergency_brake"},
    {brakeDelay, "BrakeDelay"},
    {brakeReleaser, "BrakeReleaser"},
    {springBrakeActivate, "SpringBrakeActivate"},
    {epFuseSwitch, "EpFuseSwitch"},
    {sandbox, "Sandbox"},
    {sandboxAutoAllow, "SandboxAutoAllow"},
    {compressorSwitch, "CompressorSwitch"},
    {compressorPreset, "CompressorPreset"},
    {converterSwitch, "ConverterSwitch"},
    {batterySwitch, "BatterySwitch"},
    {mainSwitch, "MainSwitch"},
    {heatingSwitch, "HeatingSwitch"},
    {gensetSwitch, "GensetSwitch"},
    {pantographValve, "PantValve"},
    {pantographsValve, "PantsValve"},
    {pantographsAllDown, "PantAllDown"},
    {compartmentLightsSwitch, "CompartmentLightsSwitch"},
    {compartmentLightsSwitchOff, "CompartmentLightsSwitchOff"},
    {waterPumpBreakerSwitch, "WaterPumpBreakerSwitch"},
    {waterPumpSwitch, "WaterPumpSwitch"},
    {waterPumpSwitchOff, "WaterPumpSwitchOff"},
    {waterHeaterBreakerSwitch, "WaterHeaterBreakerSwitch"},
    {waterHeaterSwitch, "WaterHeaterSwitch"},
    {waterCircuitsLinkSwitch, "WaterCircuitsLinkSwitch"},
    {fuelPumpSwitch, "FuelPumpSwitch"},
    {fuelPumpSwitchOff, "FuelPumpSwitchOff"},
    {oilPumpSwitch, "OilPumpSwitch"},
    {oilPumpSwitchOff, "OilPumpSwitchOff"},
    {motorBlowersFrontSwitch, "MotorBlowersFrontSwitch"},
    {motorBlowersFrontSwitchOff, "MotorBlowersFrontSwitchOff"},
    {motorBlowersRearSwitch, "MotorBlowersRearSwitch"},
    {motorBlowersRearSwitchOff, "MotorBlowersRearSwitchOff"},
    {doorOpen, "DoorOpen"},
    {doorClose, "DoorClose"},
    {doorLock, "DoorLock"},
    {doorMode, "DoorMode"},
    {doorPermit, "DoorPermit"},
    {doorStep, "DoorStep"},
    {setDamage, "SetDamage"},
    {load, "Load="},
    {unload, "UnLoad="},
    {none, ""},
    {unknown, ""},
}};

} // namespace

ParsedVehicleCommand ParseVehicleCommand(std::string const &Text)
{
	if (Text.empty())
	{
		return {none, {}};
	}
	// Load= and UnLoad= begin the text, the name of the load follows
	if (Text.starts_with("Load="))
	{
		return {load, extract_value("Load", Text)};
	}
	if (Text.starts_with("UnLoad="))
	{
		return {unload, extract_value("UnLoad", Text)};
	}
	auto const named{std::ranges::find(commandNames, std::string_view{Text}, &std::pair<VehicleCommand, std::string_view>::second)};
	if (named == commandNames.end() || named->first == load || named->first == unload)
	{
		return {unknown, {}};
	}
	return {named->first, {}};
}

std::string_view VehicleCommandName(VehicleCommand const Command)
{
	auto const named{std::ranges::find(commandNames, Command, &std::pair<VehicleCommand, std::string_view>::first)};
	return named != commandNames.end() ? named->second : std::string_view{};
}
