#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"
#include "golden.h"
#include "consist.h"
#include "real_vehicles.h"

#include <format>

#include "McZapkie/hamulce.h"

// characterisation of the commands vehicles pass each other along the couplers (RunCommand): what each of them
// does to every real vehicle, recorded in golden traces (data/golden/commands/), how they travel through a train of
// vehicles coupled either way round, and the loading and unloading driven by them

using test::definition;
using test::vehicle;

namespace {

double flag(bool Value) {
	return Value ? 1.0 : 0.0;
}

// what the commands change
std::vector<test::channel> state() {
	std::vector<test::channel> result{
	    {"MainCtrlPos", [](auto const &M) { return M.MainCtrlPos; }},
	    {"ScndCtrlPos", [](auto const &M) { return M.ScndCtrlPos; }},
	    {"LastRelayTime", [](auto const &M) { return M.LastRelayTime; }},
	    {"eimic_real", [](auto const &M) { return M.eimic_real; }},
	    {"TUHEX_StageActual", [](auto const &M) { return M.TUHEX_StageActual; }},
	    {"TUHEX_Active", [](auto const &M) { return flag(M.TUHEX_Active); }},
	    {"DynamicBrakeEMUStatus", [](auto const &M) { return flag(M.DynamicBrakeEMUStatus); }},
	    {"BrakeReleaser", [](auto const &M) { return flag(M.Hamulec->Releaser()); }},
	    {"WaterPumpBreaker", [](auto const &M) { return flag(M.WaterPump.breaker); }},
	    {"WaterPumpEnabled", [](auto const &M) { return flag(M.WaterPump.is_enabled); }},
	    {"WaterPumpDisabled", [](auto const &M) { return flag(M.WaterPump.is_disabled); }},
	    {"WaterHeaterBreaker", [](auto const &M) { return flag(M.WaterHeater.breaker); }},
	    {"WaterHeaterEnabled", [](auto const &M) { return flag(M.WaterHeater.is_enabled); }},
	    {"WaterCircuitsLink", [](auto const &M) { return flag(M.WaterCircuitsLink); }},
	    {"FuelPumpEnabled", [](auto const &M) { return flag(M.FuelPump.is_enabled); }},
	    {"FuelPumpDisabled", [](auto const &M) { return flag(M.FuelPump.is_disabled); }},
	    {"OilPumpEnabled", [](auto const &M) { return flag(M.OilPump.is_enabled); }},
	    {"OilPumpDisabled", [](auto const &M) { return flag(M.OilPump.is_disabled); }},
	    {"CompartmentLightsEnabled", [](auto const &M) { return flag(M.CompartmentLights.is_enabled); }},
	    {"CompartmentLightsDisabled", [](auto const &M) { return flag(M.CompartmentLights.is_disabled); }},
	    {"MainSwitch", [](auto const &M) { return flag(M.Mains); }},
	    {"Heating", [](auto const &M) { return flag(M.Heating); }},
	    {"DirActive", [](auto const &M) { return M.DirActive; }},
	    {"DirAbsolute", [](auto const &M) { return M.DirAbsolute; }},
	    {"CabActive", [](auto const &M) { return M.CabActive; }},
	    {"CabMaster", [](auto const &M) { return flag(M.CabMaster); }},
	    {"AutoRelayFlag", [](auto const &M) { return flag(M.AutoRelayFlag); }},
	    {"ConverterAllow", [](auto const &M) { return flag(M.ConverterAllow); }},
	    {"Battery", [](auto const &M) { return flag(M.Battery); }},
	    {"CompressorAllow", [](auto const &M) { return flag(M.CompressorAllow); }},
	    {"CompressorListPos", [](auto const &M) { return M.CompressorListPos; }},
	    {"DoorLock", [](auto const &M) { return flag(M.Doors.lock_enabled); }},
	    {"DoorStep", [](auto const &M) { return flag(M.Doors.step_enabled); }},
	    {"DoorRemoteOnly", [](auto const &M) { return flag(M.Doors.remote_only); }},
	    {"DepartureSignal", [](auto const &M) { return flag(M.DepartureSignal); }},
	    {"PantsValve", [](auto const &M) { return flag(M.PantsValve.is_enabled) + 2 * flag(M.PantsValve.is_disabled); }},
	    {"MaxCurrentHigh", [](auto const &M) { return flag(M.MotorOverloadRelayHighThreshold); }},
	    {"Imin", [](auto const &M) { return M.Imin; }},
	    {"DamageFlag", [](auto const &M) { return M.DamageFlag; }},
	    {"RadioStop", [](auto const &M) { return flag(M.RadioStopFlag); }},
	    {"BrakeDelayFlag", [](auto const &M) { return M.BrakeDelayFlag; }},
	    {"SandDose", [](auto const &M) { return flag(M.SandDose); }},
	    {"CabSignal", [](auto const &M) { return flag(M.SecuritySystem.is_cabsignal_blinking()); }},
	    {"LoadAmount", [](auto const &M) { return M.LoadAmount; }},
	    {"LoadStatus", [](auto const &M) { return M.LoadStatus; }},
	    {"SpeedCtrlValue", [](auto const &M) { return M.SpeedCtrlValue; }},
	    {"SpeedCtrlParking", [](auto const &M) { return flag(M.SpeedCtrlUnit.Parking); }},
	    {"SpringBrakeActivate", [](auto const &M) { return flag(M.SpringBrake.Activate); }},
	};
	for (int end : {end::front, end::rear}) {
		auto const name{std::string{end == end::front ? "Front" : "Rear"}};
		result.push_back({"MotorBlowers" + name + "Enabled", [end](auto const &M) { return flag(M.MotorBlowers[end].is_enabled); }});
		result.push_back({"MotorBlowers" + name + "Disabled", [end](auto const &M) { return flag(M.MotorBlowers[end].is_disabled); }});
		result.push_back({"Pant" + name + "Valve", [end](auto const &M) { return flag(M.Pantographs[end].valve.is_enabled) + 2 * flag(M.Pantographs[end].valve.is_disabled); }});
	}
	for (int side : {side::right, side::left}) {
		auto const name{std::string{side == side::right ? "Right" : "Left"}};
		result.push_back({name + "DoorPermit", [side](auto const &M) { return flag(M.Doors.instances[side].open_permit); }});
		result.push_back({name + "DoorRemoteOpen", [side](auto const &M) { return flag(M.Doors.instances[side].remote_open); }});
		result.push_back({name + "DoorRemoteClose", [side](auto const &M) { return flag(M.Doors.instances[side].remote_close); }});
	}
	return result;
}

struct command {
	std::string name;
	double value1;
	double value2{1.0}; // the direction it comes from, 1 the front
};

// every command RunCommand knows, with values taking each of its ways, and one it doesn't
std::vector<command> const commands{
    {"MainCtrl", 1},
    {"MainCtrl", 2},
    {"MainCtrl", 0},
    {"MainCtrl", 99},
    {"ScndCtrl", 1},
    {"ScndCtrl", 0},
    {"ScndCtrl", 99},
    {"EIMIC", 0.5},
    {"EIMIC", -0.5},
    {"Brake", 1},
    {"Brake", 2},
    {"Brake", 0},
    {"BrakeReleaser", 1},
    {"BrakeReleaser", 0},
    {"WaterPumpBreakerSwitch", 1},
    {"WaterPumpSwitch", 1},
    {"WaterPumpSwitchOff", 1},
    {"WaterPumpSwitchOff", 0},
    {"WaterPumpSwitch", 0},
    {"WaterPumpBreakerSwitch", 0},
    {"WaterHeaterBreakerSwitch", 1},
    {"WaterHeaterSwitch", 1},
    {"WaterHeaterSwitch", 0},
    {"WaterHeaterBreakerSwitch", 0},
    {"WaterCircuitsLinkSwitch", 1},
    {"WaterCircuitsLinkSwitch", 0},
    {"FuelPumpSwitch", 1},
    {"FuelPumpSwitchOff", 1},
    {"FuelPumpSwitchOff", 0},
    {"FuelPumpSwitch", 0},
    {"OilPumpSwitch", 1},
    {"OilPumpSwitchOff", 1},
    {"OilPumpSwitchOff", 0},
    {"OilPumpSwitch", 0},
    {"MotorBlowersFrontSwitch", 1},
    {"MotorBlowersFrontSwitchOff", 1},
    {"MotorBlowersFrontSwitchOff", 0},
    {"MotorBlowersFrontSwitch", 0},
    {"MotorBlowersRearSwitch", 1},
    {"MotorBlowersRearSwitchOff", 1},
    {"MotorBlowersRearSwitchOff", 0},
    {"MotorBlowersRearSwitch", 0},
    {"CompartmentLightsSwitch", 1},
    {"CompartmentLightsSwitchOff", 1},
    {"CompartmentLightsSwitchOff", 0},
    {"CompartmentLightsSwitch", 0},
    {"BatterySwitch", 1},
    {"ConverterSwitch", 1},
    {"CompressorSwitch", 1},
    {"CompressorPreset", 1},
    {"CompressorPreset", 99},
    {"CompressorPreset", -1},
    {"MainSwitch", 1},
    {"HeatingSwitch", 1},
    {"HeatingSwitch", 0},
    {"Direction", 1},
    {"Direction", -1},
    {"Direction", 0},
    {"CabActivisation", 1, 1},
    {"CabActivisation", 1, -1},
    {"CabActivisation", 0, 1},
    {"AutoRelaySwitch", 1},
    {"AutoRelaySwitch", 0},
    {"RelayReset", 1},
    {"DoorPermit", 1, 1},
    {"DoorPermit", 2, 1},
    {"DoorPermit", -3, 1},
    {"DoorPermit", 1, -1},
    {"DoorPermit", 3, -1},
    {"DoorOpen", 1, 1},
    {"DoorOpen", 2, -1},
    {"DoorClose", 3, 1},
    {"DoorOpen", 3, 1},
    {"DoorClose", 1, -1},
    {"DoorLock", 0},
    {"DoorLock", 1},
    {"DoorStep", 0},
    {"DoorStep", 1},
    {"DoorMode", 1},
    {"DoorMode", 0},
    {"DepartureSignal", 1},
    {"DepartureSignal", 0},
    {"PantValve", 1},
    {"PantValve", 0x80 | 1},
    {"PantValve", 0x40 | 1, -1},
    {"PantValve", 0x80 | 0x40 | 2, -1},
    {"PantValve", 3},
    {"PantValve", 4},
    {"PantValve", 0x80 | 5},
    {"PantValve", 0x80 | 6},
    {"PantsValve", 1},
    {"PantsValve", 2},
    {"PantsValve", 3},
    {"PantsValve", 4},
    {"PantsValve", 5},
    {"PantsValve", 6},
    {"PantAllDown", 1},
    {"PantAllDown", 0},
    {"MaxCurrentSwitch", 1},
    {"MaxCurrentSwitch", 0},
    {"MinCurrentSwitch", 1},
    {"MinCurrentSwitch", 0},
    {"SetDamage", 4, 1},
    {"SetDamage", 4, -1},
    {"SetDamage", 4, 0},
    {"Emergency_brake", 1},
    {"Emergency_brake", 0},
    {"BrakeDelay", 1},
    {"BrakeDelay", 2},
    {"BrakeDelay", 4},
    {"BrakeDelay", 8},
    {"BrakeDelay", 5},
    {"Sandbox", 1},
    {"Sandbox", 0},
    {"CabSignal", 0},
    {"Load=coal", 5, 0},
    {"Load=Coal", 5, 20},
    {"UnLoad=coal", 5, 0},
    {"UnLoad=Coal", 5, 0},
    {"SpeedCntrl", 40.4},
    {"SpeedCtrlUnit.Parking", 1},
    {"SpeedCtrlUnit.Parking", 0},
    {"SpringBrakeActivate", 1},
    {"SpringBrakeActivate", 0},
    {"MainSwitch", 0},
    {"CompressorSwitch", 0},
    {"ConverterSwitch", 0},
    {"BatterySwitch", 0},
    {"SpringBrakeActivate", 1},
    {"NoSuchCommand", 1},
};

} // namespace

TEST_CASE("commands: every command run on every real vehicle") {
	for (auto const &path : test::real_vehicles()) {
		CAPTURE(path);
		test::reseed_random();
		vehicle real{definition::fiz(path), 0.0, "test", 1};
		auto done{false};
		auto channels{state()};
		channels.push_back({"Done", [&done](auto const &) { return flag(done); }});
		test::trace trace{"commands/" + path, channels, 1};
		trace.sample(0.0, real.mover());
		for (std::size_t i = 0; i < commands.size(); ++i) {
			auto const &command{commands[i]};
			trace.mark(std::format("{} {} {}", command.name, command.value1, command.value2));
			done = real->RunCommand(command.name, command.value1, command.value2, coupling::control);
			trace.sample(static_cast<double>(i + 1), real.mover());
		}
		trace.check();
	}
}

namespace {

// three locomotives coupled with the given coupling, the middle one facing backwards, as a train made up of
// whatever came to hand: the commands of the front one go to the rear coupler of the middle one and from its
// front coupler to the front one of the last, which faces backwards from the middle one, so forwards
struct train {
	vehicle front{definition::fiz("4e_v1/4e"), 0.0, "front", 1};
	vehicle middle{definition::fiz("4e_v1/4e"), 0.0, "middle"};
	vehicle last{definition::fiz("4e_v1/4e"), 0.0, "last"};

	explicit train(int Coupling) {
		front->Attach(end::rear, end::rear, &middle.mover(), Coupling, true, false);
		middle->Attach(end::front, end::front, &last.mover(), Coupling, true, false);
	}
};

} // namespace

TEST_CASE("commands: travel along the train, turned round for a vehicle facing backwards") {
	train consist{test::consist::full_coupling};
	// the driver takes the front cab, which the others follow
	REQUIRE(consist.front->CabActivisation());
	CHECK(consist.front->CabActive == 1);
	CHECK(consist.middle->CabActive == -1);
	CHECK(consist.last->CabActive == 1);
	// the command is done in each vehicle and cleared
	CHECK(consist.middle->CommandIn.Command.empty());
	CHECK(consist.last->CommandIn.Command.empty());

	SUBCASE("door permits: the right side of the sender is the left one of a vehicle facing backwards") {
		consist.front->PermitDoors(side::right, true, range_t::consist);
		CHECK(consist.front->Doors.instances[side::right].open_permit);
		CHECK(consist.middle->Doors.instances[side::left].open_permit);
		CHECK_FALSE(consist.middle->Doors.instances[side::right].open_permit);
		CHECK(consist.last->Doors.instances[side::right].open_permit);
		// and revoked the same way
		consist.front->PermitDoors(side::right, false, range_t::consist);
		CHECK_FALSE(consist.middle->Doors.instances[side::left].open_permit);
		CHECK_FALSE(consist.last->Doors.instances[side::right].open_permit);
	}
	SUBCASE("a pantograph valve: the vehicles past one coupled by its control cable take the other pantograph") {
		consist.front->OperatePantographValve(end::front, operation_t::enable, range_t::consist);
		CHECK(consist.front->Pantographs[end::front].valve.is_enabled);
		CHECK(consist.middle->Pantographs[end::front].valve.is_enabled);
		CHECK(consist.last->Pantographs[end::rear].valve.is_enabled);
	}
	SUBCASE("sent backwards, it goes no further than the sender: nothing is coupled at its front") {
		consist.front->RunCommand("DoorLock", 0, -1, coupling::control);
		CHECK_FALSE(consist.front->Doors.lock_enabled);
		CHECK(consist.middle->Doors.lock_enabled);
	}
	SUBCASE("the same command again isn't taken by the next vehicle a second time") {
		CHECK(consist.middle->SetInternalCommand("DoorStep", 0, 1, coupling::control));
		CHECK_FALSE(consist.middle->SetInternalCommand("DoorStep", 0, 1, coupling::control));
		CHECK_FALSE(consist.front->RunInternalCommand()); // nothing to run
	}
}

TEST_CASE("commands: don't pass a coupling without the control cable") {
	train consist{coupling::coupler | coupling::brakehose};
	consist.front->RunCommand("DoorLock", 0, 1, coupling::control);
	CHECK_FALSE(consist.front->Doors.lock_enabled);
	CHECK(consist.middle->Doors.lock_enabled);
	// a command of another coupling goes through it
	consist.front->RunCommand("DoorStep", 0, 1, coupling::brakehose);
	CHECK_FALSE(consist.middle->Doors.step_enabled);
}

TEST_CASE("commands: a wagon loaded and unloaded by commands, which repeat until done") {
	vehicle wagon{definition::fiz("3wl_v2/3wl")};
	REQUIRE(wagon->MaxLoad > 0);
	test::trace trace{"commands/loading", state(), 100};
	wagon.record(trace);

	SUBCASE("loading 12 tonnes of coal") {
		trace.mark("load");
		wagon->SetInternalCommand("Load=Coal", 12, 0, coupling::faux);
		wagon.run(30.0);
		CHECK(wagon->LoadAmount == doctest::Approx(12.0).epsilon(0.1));
		CHECK(wagon->LoadType.name == "coal");
		CHECK(wagon->CommandIn.Command.empty());

		trace.mark("unload");
		wagon->SetInternalCommand("UnLoad=Coal", 20, 0, coupling::faux);
		wagon.run(200.0);
		CHECK(wagon->LoadAmount == doctest::Approx(0.0));
		CHECK(wagon->LoadType.name.empty());
		trace.check();
	}
	SUBCASE("a load the wagon doesn't take stays a command, not done") {
		wagon->SetInternalCommand("Load=passengers", 12, 0, coupling::faux);
		wagon.run(10.0);
		CHECK(wagon->LoadAmount == doctest::Approx(0.0));
		CHECK_FALSE(wagon->CommandIn.Command.empty());
	}
	SUBCASE("away from the place of the command nothing is loaded") {
		wagon->SetInternalCommand("Load=Coal", 12, 5.0, coupling::faux);
		wagon->CommandIn.Location.X = 100.0;
		wagon.run(10.0);
		CHECK(wagon->LoadAmount == doctest::Approx(0.0));
		CHECK(wagon->LastLoadChangeTime == doctest::Approx(0.0));
	}
	SUBCASE("unloading an empty wagon isn't done either") {
		wagon->SetInternalCommand("UnLoad=Coal", 12, 0, coupling::faux);
		wagon.run(10.0);
		CHECK_FALSE(wagon->CommandIn.Command.empty());
	}
}
