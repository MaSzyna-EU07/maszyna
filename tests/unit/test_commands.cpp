#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"
#include "golden.h"
#include "consist.h"
#include "real_vehicles.h"

#include <format>
#include <functional>

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

// a switch of the cab, worked on the driven vehicle
struct action {
	std::string name;
	std::function<bool(TMoverParameters &)> work;
};

// every switch sending its state to the rest of the train, each way and to each range, in the order a crew
// could work them, with the controllers and the direction among them to see what they make of the state
std::vector<action> const actions{
    {"CabActivisation", [](auto &M) { return M.CabActivisation(); }},
    {"BatterySwitch on", [](auto &M) { return M.BatterySwitch(true, range_t::consist); }},
    {"CompartmentLightsSwitch on", [](auto &M) { return M.CompartmentLightsSwitch(true, range_t::consist); }},
    {"CompartmentLightsSwitchOff on", [](auto &M) { return M.CompartmentLightsSwitchOff(true, range_t::consist); }},
    {"CompartmentLightsSwitchOff off", [](auto &M) { return M.CompartmentLightsSwitchOff(false, range_t::unit); }},
    {"WaterPumpBreakerSwitch on", [](auto &M) { return M.WaterPumpBreakerSwitch(true, range_t::consist); }},
    {"WaterPumpSwitch on", [](auto &M) { return M.WaterPumpSwitch(true, range_t::consist); }},
    {"WaterPumpSwitchOff on", [](auto &M) { return M.WaterPumpSwitchOff(true, range_t::unit); }},
    {"WaterPumpSwitchOff off", [](auto &M) { return M.WaterPumpSwitchOff(false, range_t::consist); }},
    {"WaterHeaterBreakerSwitch on", [](auto &M) { return M.WaterHeaterBreakerSwitch(true, range_t::consist); }},
    {"WaterHeaterSwitch on", [](auto &M) { return M.WaterHeaterSwitch(true, range_t::consist); }},
    {"WaterCircuitsLinkSwitch on", [](auto &M) { return M.WaterCircuitsLinkSwitch(true, range_t::consist); }},
    {"FuelPumpSwitch on", [](auto &M) { return M.FuelPumpSwitch(true, range_t::consist); }},
    {"FuelPumpSwitchOff on", [](auto &M) { return M.FuelPumpSwitchOff(true, range_t::unit); }},
    {"FuelPumpSwitchOff off", [](auto &M) { return M.FuelPumpSwitchOff(false, range_t::consist); }},
    {"OilPumpSwitch on", [](auto &M) { return M.OilPumpSwitch(true, range_t::consist); }},
    {"OilPumpSwitchOff on", [](auto &M) { return M.OilPumpSwitchOff(true, range_t::unit); }},
    {"OilPumpSwitchOff off", [](auto &M) { return M.OilPumpSwitchOff(false, range_t::consist); }},
    {"MotorBlowersSwitch front on", [](auto &M) { return M.MotorBlowersSwitch(true, end::front, range_t::consist); }},
    {"MotorBlowersSwitch rear on", [](auto &M) { return M.MotorBlowersSwitch(true, end::rear, range_t::unit); }},
    {"MotorBlowersSwitchOff front on", [](auto &M) { return M.MotorBlowersSwitchOff(true, end::front, range_t::consist); }},
    {"MotorBlowersSwitchOff rear on", [](auto &M) { return M.MotorBlowersSwitchOff(true, end::rear, range_t::unit); }},
    {"MotorBlowersSwitchOff front off", [](auto &M) { return M.MotorBlowersSwitchOff(false, end::front, range_t::consist); }},
    {"MotorBlowersSwitchOff rear off", [](auto &M) { return M.MotorBlowersSwitchOff(false, end::rear, range_t::consist); }},
    {"OperatePantographsValve enable", [](auto &M) { return M.OperatePantographsValve(operation_t::enable, range_t::consist); }},
    {"OperatePantographValve front enable", [](auto &M) { return M.OperatePantographValve(end::front, operation_t::enable, range_t::consist); }},
    {"OperatePantographValve rear enable_on", [](auto &M) { return M.OperatePantographValve(end::rear, operation_t::enable_on, range_t::unit); }},
    {"OperatePantographValve rear enable_off", [](auto &M) { return M.OperatePantographValve(end::rear, operation_t::enable_off, range_t::unit); }},
    {"DirectionForward", [](auto &M) { return M.DirectionForward(); }},
    {"ConverterSwitch on", [](auto &M) { return M.ConverterSwitch(true, range_t::consist); }},
    {"CompressorSwitch on", [](auto &M) { return M.CompressorSwitch(true, range_t::consist); }},
    {"ChangeCompressorPreset 1", [](auto &M) { return M.ChangeCompressorPreset(1, range_t::consist); }},
    {"FuseOn", [](auto &M) { return M.FuseOn(range_t::consist); }},
    {"RelayReset all", [](auto &M) { return M.RelayReset(0xff, range_t::consist); }},
    {"UniversalResetButton 1", [](auto &M) { return M.UniversalResetButton(0, range_t::consist); }},
    {"UniversalResetButton 2", [](auto &M) { return M.UniversalResetButton(1, range_t::unit); }},
    {"MainSwitch on", [](auto &M) { return M.MainSwitch(true, range_t::consist); }},
    {"HeatingSwitch on", [](auto &M) { return M.HeatingSwitch(true, range_t::consist); }},
    {"MaxCurrentSwitch on", [](auto &M) { return M.MaxCurrentSwitch(true, range_t::consist); }},
    {"MinCurrentSwitch on", [](auto &M) { return M.MinCurrentSwitch(true); }},
    {"CurrentSwitch on", [](auto &M) { return M.CurrentSwitch(true); }},
    {"AutoRelaySwitch on", [](auto &M) { return M.AutoRelaySwitch(true); }},
    {"IncMainCtrl 1", [](auto &M) { return M.IncMainCtrl(1); }},
    {"IncMainCtrl 2", [](auto &M) { return M.IncMainCtrl(2); }},
    {"IncScndCtrl 1", [](auto &M) { return M.IncScndCtrl(1); }},
    {"IncScndCtrl 2", [](auto &M) { return M.IncScndCtrl(2); }},
    {"DecScndCtrl 1", [](auto &M) { return M.DecScndCtrl(1); }},
    {"DecScndCtrl 2", [](auto &M) { return M.DecScndCtrl(2); }},
    {"DecMainCtrl 1", [](auto &M) { return M.DecMainCtrl(1); }},
    {"DecMainCtrl 2", [](auto &M) { return M.DecMainCtrl(2); }},
    {"AutoRelaySwitch off", [](auto &M) { return M.AutoRelaySwitch(false); }},
    {"CurrentSwitch off", [](auto &M) { return M.CurrentSwitch(false); }},
    {"MinCurrentSwitch off", [](auto &M) { return M.MinCurrentSwitch(false); }},
    {"MaxCurrentSwitch off", [](auto &M) { return M.MaxCurrentSwitch(false, range_t::unit); }},
    {"DirectionBackward", [](auto &M) { return M.DirectionBackward(); }},
    {"DirectionBackward again", [](auto &M) { return M.DirectionBackward(); }},
    {"DirectionForward", [](auto &M) { return M.DirectionForward(); }},
    {"Sandbox on", [](auto &M) { return M.Sandbox(true, range_t::consist); }},
    {"SandboxManual on", [](auto &M) { return M.SandboxManual(true, range_t::consist); }},
    {"SandboxAuto on", [](auto &M) { return M.SandboxAuto(true, range_t::unit); }},
    {"SandboxAuto off", [](auto &M) { return M.SandboxAuto(false, range_t::consist); }},
    {"SandboxManual off", [](auto &M) { return M.SandboxManual(false, range_t::consist); }},
    {"Sandbox off", [](auto &M) { return M.Sandbox(false, range_t::consist); }},
    {"PermitDoors right", [](auto &M) { return M.PermitDoors(side::right, true, range_t::consist); }},
    {"PermitDoors left", [](auto &M) { return M.PermitDoors(side::left, true, range_t::unit); }},
    {"ChangeDoorPermitPreset +1", [](auto &M) { return M.ChangeDoorPermitPreset(1, range_t::consist); }},
    {"ChangeDoorPermitPreset -1", [](auto &M) { return M.ChangeDoorPermitPreset(-1, range_t::consist); }},
    {"PermitDoorStep off", [](auto &M) { return M.PermitDoorStep(false, range_t::consist); }},
    {"PermitDoorStep on", [](auto &M) { return M.PermitDoorStep(true, range_t::unit); }},
    {"LockDoors off", [](auto &M) { return M.LockDoors(false, range_t::consist); }},
    {"OperateDoors right open local", [](auto &M) { return M.OperateDoors(side::right, true, range_t::local); }},
    {"OperateDoors left open", [](auto &M) { return M.OperateDoors(side::left, true, range_t::consist); }},
    {"OperateDoors right close", [](auto &M) { return M.OperateDoors(side::right, false, range_t::unit); }},
    {"ChangeDoorControlMode remote", [](auto &M) { return M.ChangeDoorControlMode(true, range_t::consist); }},
    {"ChangeDoorControlMode local", [](auto &M) { return M.ChangeDoorControlMode(false, range_t::unit); }},
    {"LockDoors on", [](auto &M) { return M.LockDoors(true, range_t::unit); }},
    {"signal_departure on", [](auto &M) { return M.signal_departure(true, range_t::consist); }},
    {"signal_departure off", [](auto &M) { return M.signal_departure(false, range_t::unit); }},
    {"PermitDoors right off", [](auto &M) { return M.PermitDoors(side::right, false, range_t::consist); }},
    {"HeatingSwitch off", [](auto &M) { return M.HeatingSwitch(false, range_t::unit); }},
    {"MainSwitch off", [](auto &M) { return M.MainSwitch(false, range_t::consist); }},
    {"DropAllPantographs", [](auto &M) { return M.DropAllPantographs(true, range_t::consist); }},
    {"DropAllPantographs released", [](auto &M) { return M.DropAllPantographs(false, range_t::unit); }},
    {"OperatePantographsValve disable", [](auto &M) { return M.OperatePantographsValve(operation_t::disable, range_t::unit); }},
    {"OperatePantographValve front disable_on", [](auto &M) { return M.OperatePantographValve(end::front, operation_t::disable_on, range_t::consist); }},
    {"OperatePantographValve front disable_off", [](auto &M) { return M.OperatePantographValve(end::front, operation_t::disable_off, range_t::consist); }},
    {"OperatePantographValve front none", [](auto &M) { return M.OperatePantographValve(end::front, operation_t::none, range_t::local); }},
    {"CompressorSwitch off", [](auto &M) { return M.CompressorSwitch(false, range_t::unit); }},
    {"ConverterSwitch off", [](auto &M) { return M.ConverterSwitch(false, range_t::unit); }},
    {"WaterCircuitsLinkSwitch off", [](auto &M) { return M.WaterCircuitsLinkSwitch(false, range_t::unit); }},
    {"WaterHeaterSwitch off", [](auto &M) { return M.WaterHeaterSwitch(false, range_t::unit); }},
    {"WaterHeaterBreakerSwitch off", [](auto &M) { return M.WaterHeaterBreakerSwitch(false, range_t::unit); }},
    {"WaterPumpSwitch off", [](auto &M) { return M.WaterPumpSwitch(false, range_t::unit); }},
    {"WaterPumpBreakerSwitch off", [](auto &M) { return M.WaterPumpBreakerSwitch(false, range_t::unit); }},
    {"FuelPumpSwitch off", [](auto &M) { return M.FuelPumpSwitch(false, range_t::unit); }},
    {"OilPumpSwitch off", [](auto &M) { return M.OilPumpSwitch(false, range_t::unit); }},
    {"MotorBlowersSwitch front off", [](auto &M) { return M.MotorBlowersSwitch(false, end::front, range_t::unit); }},
    {"MotorBlowersSwitch rear off", [](auto &M) { return M.MotorBlowersSwitch(false, end::rear, range_t::consist); }},
    {"CompartmentLightsSwitch off", [](auto &M) { return M.CompartmentLightsSwitch(false, range_t::unit); }},
    {"BatterySwitch off", [](auto &M) { return M.BatterySwitch(false, range_t::unit); }},
    {"CabDeactivisation", [](auto &M) { return M.CabDeactivisation(); }},
};

} // namespace

TEST_CASE("commands: every switch of the cab, sent to the next vehicle") {
	for (auto const &path : test::real_vehicles()) {
		CAPTURE(path);
		test::reseed_random();
		vehicle driven{definition::fiz(path), 0.0, "driven", 1};
		vehicle next{definition::fiz(path), 0.0, "next"};
		driven->Attach(end::rear, end::front, &next.mover(), test::consist::full_coupling, true, false);
		auto done{false};
		auto channels{state()};
		for (auto const &channel : state()) {
			channels.push_back({"next." + channel.name, [&next, value = channel.value](auto const &) { return value(next.mover()); }});
		}
		channels.push_back({"Done", [&done](auto const &) { return flag(done); }});
		test::trace trace{"switches/" + path, channels, 1};
		trace.sample(0.0, driven.mover());
		for (std::size_t i = 0; i < actions.size(); ++i) {
			trace.mark(actions[i].name);
			done = actions[i].work(driven.mover());
			trace.sample(static_cast<double>(i + 1), driven.mover());
		}
		trace.check();
	}
}

TEST_CASE("commands: every switch of the cab, worked on a pair standing under the catenary") {
	// the same switches with time for the vehicles to act on them, as a crew works them: the pantographs rise,
	// the main switch closes, the converter and the compressor start, and the controllers take power
	for (auto const &path : test::real_vehicles()) {
		CAPTURE(path);
		test::reseed_random();
		test::consist pair;
		pair.add(definition::fiz(path), 1);
		pair.add(definition::fiz(path));
		auto done{false};
		auto channels{state()};
		channels.insert(channels.end(), {
		                                    {"PantFrontActive", [](auto const &M) { return flag(M.Pantographs[end::front].is_active); }},
		                                    {"PantRearActive", [](auto const &M) { return flag(M.Pantographs[end::rear].is_active); }},
		                                    {"PantPress", [](auto const &M) { return M.PantPress; }},
		                                    {"GroundRelay", [](auto const &M) { return flag(M.GroundRelay); }},
		                                    {"ConverterFlag", [](auto const &M) { return flag(M.ConverterFlag); }},
		                                    {"CompressorFlag", [](auto const &M) { return flag(M.CompressorFlag); }},
		                                    {"Compressor", [](auto const &M) { return M.Compressor; }},
		                                    {"MainCtrlActualPos", [](auto const &M) { return M.MainCtrlActualPos; }},
		                                    {"Itot", [](auto const &M) { return M.Itot; }},
		                                    {"FuseFlag", [](auto const &M) { return flag(M.FuseFlag); }},
		                                    {"EngineRotation", [](auto const &M) { return M.enrot; }},
		                                });
		auto const own{channels.size()};
		for (std::size_t i = 0; i < own; ++i) {
			channels.push_back({"next." + channels[i].name, [&pair, value = channels[i].value](auto const &) { return value(pair[1].mover()); }});
		}
		channels.push_back({"Done", [&done](auto const &) { return flag(done); }});
		test::trace trace{"switches_live/" + path, channels, 1};
		trace.sample(0.0, pair[0].mover());
		test::track const catenary{.voltage = test::catenary_3kv};
		for (std::size_t i = 0; i < actions.size(); ++i) {
			trace.mark(actions[i].name);
			done = actions[i].work(pair[0].mover());
			pair.run(2.0, catenary);
			trace.sample(static_cast<double>(i + 1), pair[0].mover());
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
