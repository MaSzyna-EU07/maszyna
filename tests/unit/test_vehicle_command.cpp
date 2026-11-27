#include "stdafx.h"
#include "doctest.h"

#include <string>

#include "physics/vehiclecommand.h"

// the commands of the vehicles read from their text (events, scripts, the AI) and named for the log: every command
// has the name the scenery files have always used, and reads back as itself

TEST_CASE("vehicle commands read back from their names") {
	using enum VehicleCommand;
	for (auto command{static_cast<int>(mainController)}; command <= static_cast<int>(setDamage); ++command) {
		auto const value{static_cast<VehicleCommand>(command)};
		auto const name{std::string{VehicleCommandName(value)}};
		CAPTURE(name);
		REQUIRE_FALSE(name.empty());
		if (value == load || value == unload) {
			continue; // these carry the name of the load after the =
		}
		CHECK(ParseVehicleCommand(name).command == value);
		CHECK(ParseVehicleCommand(name).argument.empty());
	}
}

TEST_CASE("vehicle commands: the empty text, an unknown one, the load") {
	CHECK(ParseVehicleCommand("").command == VehicleCommand::none);
	CHECK(ParseVehicleCommand("SetVelocity").command == VehicleCommand::unknown); // the AI's, not a vehicle's
	CHECK(ParseVehicleCommand("mainctrl").command == VehicleCommand::unknown); // the names are compared exactly
	CHECK(ParseVehicleCommand("MainCtrl").command == VehicleCommand::mainController);
	CHECK(ParseVehicleCommand("SpeedCtrlUnit.Parking").command == VehicleCommand::speedControlParking);

	auto const load{ParseVehicleCommand("Load=Coal")};
	CHECK(load.command == VehicleCommand::load);
	CHECK(load.argument == "Coal");
	auto const unload{ParseVehicleCommand("UnLoad=passengers")};
	CHECK(unload.command == VehicleCommand::unload);
	CHECK(unload.argument == "passengers");

	CHECK(VehicleCommandName(VehicleCommand::none).empty());
	CHECK(VehicleCommandName(VehicleCommand::unknown).empty());
	CHECK(VehicleCommandName(VehicleCommand::integratedController) == "EIMIC");
}
