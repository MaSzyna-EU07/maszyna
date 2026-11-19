/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "simulation/simulationsnapshotmover.h"

#include "McZapkie/MOVER.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <iterator>

namespace simulation::snapshot {

namespace {

// a plain variable of the physics kept under a key of the entry
template <typename Type_> struct member {
	std::string_view key;
	Type_ TMoverParameters::*variable;
};

// what the vehicle does: its movement, as the physics left it at the end of the step
constexpr std::array motion{
    member<double>{"v", &TMoverParameters::V},
    member<double>{"vel", &TMoverParameters::Vel},
    member<double>{"acc", &TMoverParameters::AccS},
    member<double>{"accv", &TMoverParameters::AccSVBased},
    member<double>{"nrot", &TMoverParameters::nrot},
    member<double>{"distance", &TMoverParameters::DistCounter},
    member<double>{"movelen", &TMoverParameters::dMoveLen},
};

// forces, moments and powers a step reads before it works them out anew, as it left them
constexpr std::array forces{
    member<double>{"enginepower", &TMoverParameters::EnginePower},
    member<double>{"ft", &TMoverParameters::Ft},
    member<double>{"fw", &TMoverParameters::Fw},
    member<double>{"mm", &TMoverParameters::Mm},
    member<double>{"mw", &TMoverParameters::Mw},
    member<double>{"fb", &TMoverParameters::Fb},
    member<double>{"ff", &TMoverParameters::Ff},
    member<double>{"ftrain", &TMoverParameters::FTrain},
    member<double>{"fstand", &TMoverParameters::FStand},
    member<double>{"ftotal", &TMoverParameters::FTotal},
    member<double>{"unitbrakeforce", &TMoverParameters::UnitBrakeForce},
    member<double>{"ntotal", &TMoverParameters::Ntotal},
    member<double>{"totalcurrent", &TMoverParameters::TotalCurrent},
    member<double>{"dl", &TMoverParameters::dL},
    member<double>{"nroteps", &TMoverParameters::nrot_eps},
    member<double>{"accn", &TMoverParameters::AccN},
    member<double>{"accvert", &TMoverParameters::AccVert},
    member<double>{"dizeltorque", &TMoverParameters::dizel_Torque},
    member<double>{"dizelpower", &TMoverParameters::dizel_Power},
    member<double>{"dizeldeltaomega", &TMoverParameters::dizel_engagedeltaomega},
    member<double>{"dizelnold", &TMoverParameters::dizel_n_old},
    member<double>{"dizelnregmin", &TMoverParameters::dizel_nreg_min},
    member<double>{"dizelnregmax", &TMoverParameters::dizel_nreg_max},
    member<double>{"hydrotorquein", &TMoverParameters::hydro_TC_TorqueIn},
    member<double>{"hydrotorqueout", &TMoverParameters::hydro_TC_TorqueOut},
    member<double>{"hydrotmratio", &TMoverParameters::hydro_TC_TMRatio},
    member<double>{"hydrorequest", &TMoverParameters::hydro_TC_Request},
    member<double>{"retarderfill", &TMoverParameters::hydro_R_Fill},
    member<double>{"retardertorque", &TMoverParameters::hydro_R_Torque},
    member<double>{"retarderrequest", &TMoverParameters::hydro_R_Request},
    member<double>{"retardern", &TMoverParameters::hydro_R_n},
    member<double>{"eimvpr", &TMoverParameters::eimv_pr},
    member<double>{"eimicstep", &TMoverParameters::eimic_step_progress},
    member<double>{"eimicspeedctrl", &TMoverParameters::eimicSpeedCtrl},
    member<double>{"eimicspeedctrlintegral", &TMoverParameters::eimicSpeedCtrlIntegral},
    member<double>{"newspeed", &TMoverParameters::NewSpeed},
    member<double>{"speedctrltimer", &TMoverParameters::SpeedCtrlTimer},
    member<double>{"vadd", &TMoverParameters::Vadd},
    member<double>{"medepvctime", &TMoverParameters::MED_EPVC_CurrentTime},
    member<double>{"mededdelay", &TMoverParameters::MED_ED_DelayTimer},
};

// switches, levers and what they've set going
constexpr std::array switches{
    member<bool>{"battery", &TMoverParameters::Battery},
    member<bool>{"mains", &TMoverParameters::Mains},
    member<bool>{"converterallow", &TMoverParameters::ConverterAllow},
    member<bool>{"converterallowlocal", &TMoverParameters::ConverterAllowLocal},
    member<bool>{"converter", &TMoverParameters::ConverterFlag},
    member<bool>{"compressorallow", &TMoverParameters::CompressorAllow},
    member<bool>{"compressorallowlocal", &TMoverParameters::CompressorAllowLocal},
    member<bool>{"compressor", &TMoverParameters::CompressorFlag},
    member<bool>{"compressorlock", &TMoverParameters::CompressorGovernorLock},
    member<bool>{"pantcompressor", &TMoverParameters::PantCompFlag},
    member<bool>{"heatingallow", &TMoverParameters::HeatingAllow},
    member<bool>{"heating", &TMoverParameters::Heating},
    member<bool>{"cabmaster", &TMoverParameters::CabMaster},
    member<bool>{"alarmchain", &TMoverParameters::AlarmChainFlag},
    member<bool>{"radiostop", &TMoverParameters::RadioStopFlag},
    member<bool>{"radio", &TMoverParameters::Radio},
    member<bool>{"radiofuse", &TMoverParameters::RadioFuse},
    member<bool>{"signalling", &TMoverParameters::Signalling},
    member<bool>{"emergencyvalveopen", &TMoverParameters::EmergencyValveOpen},
    member<bool>{"lockpipe", &TMoverParameters::LockPipe},
    member<bool>{"unlockpipe", &TMoverParameters::UnlockPipe},
    member<bool>{"departuresignal", &TMoverParameters::DepartureSignal},
    member<bool>{"sanddoseauto", &TMoverParameters::SandDoseAuto},
    member<bool>{"sanddoseautoallow", &TMoverParameters::SandDoseAutoAllow},
    member<bool>{"inspectorcutout", &TMoverParameters::BrakeCutOutByInspector},
    member<bool>{"physicactivation", &TMoverParameters::PhysicActivation},
    member<bool>{"epfuse", &TMoverParameters::EpFuse},
    member<bool>{"shuntmode", &TMoverParameters::ShuntMode},
    member<bool>{"pantkurek3", &TMoverParameters::bPantKurek3},
    member<bool>{"pantswitch", &TMoverParameters::PantPressSwitchActive},
    member<bool>{"pantlock", &TMoverParameters::PantPressLockActive},
    member<bool>{"pantalldown", &TMoverParameters::PantAllDown},
    member<bool>{"sanddose", &TMoverParameters::SandDose},
    member<bool>{"sanddosemanual", &TMoverParameters::SandDoseManual},
    member<bool>{"dynamicbrake", &TMoverParameters::DynamicBrakeFlag},
    member<bool>{"stlinswitchoff", &TMoverParameters::StLinSwitchOff},
    member<bool>{"power110v", &TMoverParameters::Power110vIsAvailable},
    member<bool>{"power24v", &TMoverParameters::Power24vIsAvailable},
};

// relays and contactors
constexpr std::array relays{
    member<bool>{"fuse", &TMoverParameters::FuseFlag},
    member<bool>{"convovld", &TMoverParameters::ConvOvldFlag},
    member<bool>{"groundrelay", &TMoverParameters::GroundRelay},
    member<bool>{"stlin", &TMoverParameters::StLinFlag},
    member<bool>{"resistors", &TMoverParameters::ResistorsFlag},
    member<bool>{"novoltrelay", &TMoverParameters::NoVoltRelay},
    member<bool>{"overvoltrelay", &TMoverParameters::OvervoltageRelay},
    member<bool>{"pressureswitch", &TMoverParameters::ControlPressureSwitch},
    member<bool>{"delayctrl", &TMoverParameters::DelayCtrlFlag},
    member<bool>{"autorelay", &TMoverParameters::AutoRelayFlag},
    member<bool>{"slipping", &TMoverParameters::SlippingWheels},
    member<bool>{"dizelstartup", &TMoverParameters::dizel_startup},
    member<bool>{"dizelignition", &TMoverParameters::dizel_ignition},
    member<bool>{"dizelspinup", &TMoverParameters::dizel_spinup},
    member<bool>{"hydrolockup", &TMoverParameters::hydro_TC_Lockup},
    member<bool>{"retarderclutch", &TMoverParameters::hydro_R_ClutchActive},
    member<bool>{"dynamicbrakeemu", &TMoverParameters::DynamicBrakeEMUStatus},
    member<bool>{"tuhexreschange", &TMoverParameters::TUHEX_ResChange},
    member<bool>{"tuhexactive", &TMoverParameters::TUHEX_Active},
};

// positions of the controllers
constexpr std::array positions{
    member<int>{"mainctrl", &TMoverParameters::MainCtrlPos},
    member<int>{"scndctrl", &TMoverParameters::ScndCtrlPos},
    member<int>{"mainctrlactual", &TMoverParameters::MainCtrlActualPos},
    member<int>{"scndctrlactual", &TMoverParameters::ScndCtrlActualPos},
    member<int>{"diractive", &TMoverParameters::DirActive},
    member<int>{"dirabsolute", &TMoverParameters::DirAbsolute},
    member<int>{"cabactive", &TMoverParameters::CabActive},
    member<int>{"caboccupied", &TMoverParameters::CabOccupied},
    member<int>{"lightspos", &TMoverParameters::LightsPos},
    member<int>{"compressorlistpos", &TMoverParameters::CompressorListPos},
    member<int>{"wiperswitchpos", &TMoverParameters::wiperSwitchPos},
    member<int>{"pulseforcecount", &TMoverParameters::PulseForceCount},
    member<int>{"loadstatus", &TMoverParameters::LoadStatus},
    member<int>{"brakectrl", &TMoverParameters::BrakeCtrlPos},
    member<int>{"manualbrake", &TMoverParameters::ManualBrakePos},
    member<int>{"brakedelay", &TMoverParameters::BrakeDelayFlag},
    member<int>{"brakeopmode", &TMoverParameters::BrakeOpModeFlag},
    member<int>{"doubletr", &TMoverParameters::DoubleTr},
    member<int>{"damage", &TMoverParameters::DamageFlag},
    member<int>{"enginedamage", &TMoverParameters::EngDmgFlag},
    member<int>{"warningsignal", &TMoverParameters::WarningSignal},
    member<int>{"dimmer", &TMoverParameters::modernDimmerPosition},
    member<int>{"tuhexstage", &TMoverParameters::TUHEX_StageActual},
};

// continuous settings, timers of the devices and the state of the engine
constexpr std::array levels{
    member<double>{"brakectrlr", &TMoverParameters::BrakeCtrlPosR},
    member<double>{"brakectrl2", &TMoverParameters::BrakeCtrlPos2},
    member<double>{"brakectrlf", &TMoverParameters::fBrakeCtrlPos},
    member<double>{"localbrake", &TMoverParameters::LocalBrakePosA},
    member<double>{"localbrakeeim", &TMoverParameters::LocalBrakePosAEIM},
    member<double>{"dynamicbrakectrl", &TMoverParameters::DynamicBrakeCtrlPos},
    member<double>{"eimic", &TMoverParameters::eimic},
    member<double>{"eimicanalog", &TMoverParameters::eimic_analog},
    member<double>{"eimicreal", &TMoverParameters::eimic_real},
    member<double>{"eimlocalbrake", &TMoverParameters::eim_localbrake},
    member<double>{"anpos", &TMoverParameters::AnPos},
    member<double>{"speedctrl", &TMoverParameters::SpeedCtrlValue},
    member<double>{"mainsinit", &TMoverParameters::MainsInitTimeCountdown},
    member<double>{"converterdelay", &TMoverParameters::ConverterStartDelayTimer},
    member<double>{"compressorpowertime", &TMoverParameters::CompressorPowerTime},
    member<double>{"wheelflat", &TMoverParameters::WheelFlat},
    member<double>{"brakeslippingtimer", &TMoverParameters::BrakeSlippingTimer},
    member<double>{"pulseforce", &TMoverParameters::PulseForce},
    member<double>{"pulseforcetimer", &TMoverParameters::PulseForceTimer},
    member<double>{"lastloadchangetime", &TMoverParameters::LastLoadChangeTime},
    member<double>{"brvtimer", &TMoverParameters::BRVtimer},
    member<double>{"epforce", &TMoverParameters::EpForce},
    member<double>{"lastrelaytime", &TMoverParameters::LastRelayTime},
    member<double>{"lastswitchingtime", &TMoverParameters::LastSwitchingTime},
    member<double>{"batteryvoltage", &TMoverParameters::BatteryVoltage},
    member<double>{"power24vvoltage", &TMoverParameters::Power24vVoltage},
    member<double>{"enginevoltage", &TMoverParameters::EngineVoltage},
    member<double>{"enrot", &TMoverParameters::enrot},
    member<double>{"im", &TMoverParameters::Im},
    member<double>{"itot", &TMoverParameters::Itot},
    member<double>{"rventrot", &TMoverParameters::RventRot},
    member<double>{"sand", &TMoverParameters::Sand},
    member<double>{"dizelfill", &TMoverParameters::dizel_fill},
    member<double>{"dizelengage", &TMoverParameters::dizel_engage},
    member<double>{"dizelengagestate", &TMoverParameters::dizel_engagestate},
    member<double>{"dizelgear", &TMoverParameters::dizel_automaticgearstatus},
    member<double>{"dizelfiring", &TMoverParameters::dizel_FiringCountdown},
    member<double>{"hydrofill", &TMoverParameters::hydro_TC_Fill},
    member<double>{"hydronin", &TMoverParameters::hydro_TC_nIn},
    member<double>{"hydronout", &TMoverParameters::hydro_TC_nOut},
    member<double>{"hydrolockuprate", &TMoverParameters::hydro_TC_LockupRate},
    member<double>{"eangle", &TMoverParameters::eAngle},
    member<double>{"fuelrate", &TMoverParameters::dizel_FuelConsumptionActual},
    member<double>{"fuelused", &TMoverParameters::dizel_FuelConsumptedTotal},
};

// the air: pressures and volumes the physics keeps itself. the reservoirs of the brake valve come apart from these
constexpr std::array air{
    member<double>{"pipe", &TMoverParameters::PipePress},
    member<double>{"pipeeqvt", &TMoverParameters::EqvtPipePress},
    member<double>{"cntrlpipe", &TMoverParameters::CntrlPipePress},
    member<double>{"scndpipe", &TMoverParameters::ScndPipePress},
    member<double>{"brakepress", &TMoverParameters::BrakePress},
    member<double>{"locbrakepress", &TMoverParameters::LocBrakePress},
    member<double>{"pipebrakepress", &TMoverParameters::PipeBrakePress},
    member<double>{"volume", &TMoverParameters::Volume},
    member<double>{"compressedvolume", &TMoverParameters::CompressedVolume},
    member<double>{"compressorpress", &TMoverParameters::Compressor},
    member<double>{"limpipe", &TMoverParameters::LimPipePress},
    member<double>{"actflowspeed", &TMoverParameters::ActFlowSpeed},
    member<double>{"pantvolume", &TMoverParameters::PantVolume},
    member<double>{"pantpress", &TMoverParameters::PantPress},
};

template <typename Table_> void capture_members(record &Entry, TMoverParameters const &Mover, Table_ const &Table)
{
	for (auto const &item : Table)
		Entry.set(item.key, Mover.*(item.variable));
}

template <typename Table_> void apply_members(TMoverParameters &Mover, record const &Entry, Table_ const &Table)
{
	for (auto const &item : Table)
		Entry.get(item.key, Mover.*(item.variable));
}

// keys of the parts a vehicle has two of, or a few: "pant0valve", "door1open"
std::string indexed(std::string_view const Part, int const Index, std::string_view const Key)
{
	return std::format("{}{}{}", Part, Index, Key);
}

// the types of the parts are private to the physics, so the helpers below take whatever has the members they use

// the pressures a valve of the driver keeps, under the keys "<name>0", "<name>1"...
void capture_handle(record &Entry, std::string_view const Name, TDriverHandle &Handle)
{
	std::vector<double> values;
	for (auto const *pressure : Handle.pressures())
		values.emplace_back(*pressure);
	Entry.set(Name, numbers_text(values));
	Entry.set(std::format("{}time", Name), (Handle.Time ? 1 : 0) | (Handle.TimeEP ? 2 : 0));
}

void apply_handle(record const &Entry, std::string_view const Name, TDriverHandle &Handle)
{
	auto const pressures{Handle.pressures()};
	// the pressures of another kind of valve mean nothing to this one
	if (auto const values{text_numbers(Entry.text(Name))}; values.size() == pressures.size())
	{
		for (std::size_t index{0}; index < values.size(); ++index)
			*pressures[index] = values[index];
	}
	if (auto time{0}; Entry.get(std::format("{}time", Name), time))
	{
		Handle.Time = (time & 1) != 0;
		Handle.TimeEP = (time & 2) != 0;
	}
}

// the electric circuits of the vehicle, and what it gives and takes through its power couplings: each step starts
// from what the one before left there
void capture_power(record &Entry, TMoverParameters const &Mover)
{
	std::vector<double> circuits;
	for (auto const &[voltage, current] : Mover.PowerCircuits)
	{
		circuits.emplace_back(voltage);
		circuits.emplace_back(current);
	}
	Entry.set("circuits", numbers_text(circuits));
	for (auto side{0}; side < 2; ++side)
	{
		auto const &coupler{Mover.Couplers[side]};
		std::vector<double> values;
		for (auto const *power : {&coupler.power_high, &coupler.power_110v, &coupler.power_24v})
		{
			values.emplace_back(power->current);
			values.emplace_back(power->voltage);
			values.emplace_back(power->is_local ? 1.0 : 0.0);
			values.emplace_back(power->is_live ? 1.0 : 0.0);
		}
		Entry.set(indexed("couplerpower", side, ""), numbers_text(values));
	}
	Entry.set("security", numbers_text(Mover.SecuritySystem.state()));
	// the figures of an induction motor drive, which each step goes on from
	Entry.set("eimv", numbers_text(Mover.eimv));
	// the speed control: 1 active, 2 starting, 4 set by hand, 8 braking in, 16 braking, 32 releasing, 64 standby, 128 parking
	auto const &control{Mover.SpeedCtrlUnit};
	Entry.set("speedctrlstate", (control.IsActive ? 1 : 0) | (control.Start ? 2 : 0) | (control.ManualStateOverride ? 4 : 0) | (control.BrakeIntervention ? 8 : 0) |
	                                (control.BrakeInterventionBraking ? 16 : 0) | (control.BrakeInterventionUnbraking ? 32 : 0) | (control.Standby ? 64 : 0) |
	                                (control.Parking ? 128 : 0));
	Entry.set("speedctrlvelocity", control.DesiredVelocity).set("speedctrlpower", control.DesiredPower);
}

void apply_power(TMoverParameters &Mover, record const &Entry)
{
	if (auto const circuits{text_numbers(Entry.text("circuits"))}; circuits.size() == 2 * Mover.PowerCircuits.size())
	{
		for (std::size_t index{0}; index < Mover.PowerCircuits.size(); ++index)
			Mover.PowerCircuits[index] = {circuits[2 * index], circuits[2 * index + 1]};
	}
	for (auto side{0}; side < 2; ++side)
	{
		auto &coupler{Mover.Couplers[side]};
		auto const values{text_numbers(Entry.text(indexed("couplerpower", side, "")))};
		if (values.size() != 12)
			continue;
		std::size_t index{0};
		for (auto *power : {&coupler.power_high, &coupler.power_110v, &coupler.power_24v})
		{
			power->current = values[index];
			power->voltage = values[index + 1];
			power->is_local = values[index + 2] != 0.0;
			power->is_live = values[index + 3] != 0.0;
			index += 4;
		}
	}
	if (Entry.has("security"))
		Mover.SecuritySystem.state(text_numbers(Entry.text("security")));
	if (auto const drive{text_numbers(Entry.text("eimv"))}; drive.size() == std::size(Mover.eimv))
		std::ranges::copy(drive, std::begin(Mover.eimv));
	auto &control{Mover.SpeedCtrlUnit};
	if (auto state{0}; Entry.get("speedctrlstate", state))
	{
		control.IsActive = (state & 1) != 0;
		control.Start = (state & 2) != 0;
		control.ManualStateOverride = (state & 4) != 0;
		control.BrakeIntervention = (state & 8) != 0;
		control.BrakeInterventionBraking = (state & 16) != 0;
		control.BrakeInterventionUnbraking = (state & 32) != 0;
		control.Standby = (state & 64) != 0;
		control.Parking = (state & 128) != 0;
	}
	Entry.get("speedctrlvelocity", control.DesiredVelocity);
	Entry.get("speedctrlpower", control.DesiredPower);
}

// the three states of a device as one number: 1 allowed to run, 2 told to stop, 4 running
template <typename Device_> int device_state(Device_ const &Device)
{
	return (Device.is_enabled ? 1 : 0) | (Device.is_disabled ? 2 : 0) | (Device.is_active ? 4 : 0);
}

template <typename Device_> void device_state(Device_ &Device, int const State)
{
	Device.is_enabled = (State & 1) != 0;
	Device.is_disabled = (State & 2) != 0;
	Device.is_active = (State & 4) != 0;
}

template <typename Device_> void capture_device(record &Entry, std::string_view const Key, Device_ const &Device)
{
	Entry.set(Key, device_state(Device));
}

template <typename Device_> void apply_device(record const &Entry, std::string_view const Key, Device_ &Device)
{
	if (auto state{0}; Entry.get(Key, state))
		device_state(Device, state);
}

void capture_devices(record &Entry, TMoverParameters const &Mover)
{
	capture_device(Entry, "pantsvalve", Mover.PantsValve);
	for (auto index{0}; index < static_cast<int>(Mover.Pantographs.size()); ++index)
	{
		auto const &pantograph{Mover.Pantographs[index]};
		capture_device(Entry, indexed("pant", index, "valve"), pantograph.valve);
		Entry.set(indexed("pant", index, "up"), pantograph.is_active);
		Entry.set(indexed("pant", index, "voltage"), pantograph.voltage);
	}
	Entry.set("pantfrontvolt", Mover.PantFrontVolt);
	Entry.set("pantrearvolt", Mover.PantRearVolt);
	Entry.set("pantvoltage", Mover.PantographVoltage);
	capture_device(Entry, "fuelpump", Mover.FuelPump);
	capture_device(Entry, "oilpump", Mover.OilPump);
	Entry.set("oilpressure", Mover.OilPump.pressure);
	capture_device(Entry, "waterpump", Mover.WaterPump);
	Entry.set("waterpumpbreaker", Mover.WaterPump.breaker);
	Entry.set("waterheater", (Mover.WaterHeater.is_enabled ? 1 : 0) | (Mover.WaterHeater.is_active ? 4 : 0));
	Entry.set("waterheaterbreaker", Mover.WaterHeater.breaker);
	for (auto index{0}; index < static_cast<int>(Mover.MotorBlowers.size()); ++index)
	{
		capture_device(Entry, indexed("blower", index, ""), Mover.MotorBlowers[index]);
		Entry.set(indexed("blower", index, "rpm"), Mover.MotorBlowers[index].revolutions);
	}
	capture_device(Entry, "compartmentlights", Mover.CompartmentLights);
	Entry.set("lights0", Mover.iLights[end::front]);
	Entry.set("lights1", Mover.iLights[end::rear]);
}

void apply_devices(TMoverParameters &Mover, record const &Entry)
{
	apply_device(Entry, "pantsvalve", Mover.PantsValve);
	for (auto index{0}; index < static_cast<int>(Mover.Pantographs.size()); ++index)
	{
		auto &pantograph{Mover.Pantographs[index]};
		apply_device(Entry, indexed("pant", index, "valve"), pantograph.valve);
		Entry.get(indexed("pant", index, "up"), pantograph.is_active);
		Entry.get(indexed("pant", index, "voltage"), pantograph.voltage);
		// the pantograph is where it's meant to be, there's no sound of it getting there to play
		pantograph.sound_event = pantograph.is_active;
	}
	Entry.get("pantfrontvolt", Mover.PantFrontVolt);
	Entry.get("pantrearvolt", Mover.PantRearVolt);
	Entry.get("pantvoltage", Mover.PantographVoltage);
	apply_device(Entry, "fuelpump", Mover.FuelPump);
	apply_device(Entry, "oilpump", Mover.OilPump);
	Entry.get("oilpressure", Mover.OilPump.pressure);
	apply_device(Entry, "waterpump", Mover.WaterPump);
	Entry.get("waterpumpbreaker", Mover.WaterPump.breaker);
	if (auto state{0}; Entry.get("waterheater", state))
	{
		Mover.WaterHeater.is_enabled = (state & 1) != 0;
		Mover.WaterHeater.is_active = (state & 4) != 0;
	}
	Entry.get("waterheaterbreaker", Mover.WaterHeater.breaker);
	for (auto index{0}; index < static_cast<int>(Mover.MotorBlowers.size()); ++index)
	{
		apply_device(Entry, indexed("blower", index, ""), Mover.MotorBlowers[index]);
		Entry.get(indexed("blower", index, "rpm"), Mover.MotorBlowers[index].revolutions);
	}
	apply_device(Entry, "compartmentlights", Mover.CompartmentLights);
	Entry.get("lights0", Mover.iLights[end::front]);
	Entry.get("lights1", Mover.iLights[end::rear]);
}

// what a door is told to do, as one number: 1 permit, 2 local open, 4 local close, 8 remote open, 16 remote close
template <typename Door_> int door_inputs(Door_ const &Door)
{
	return (Door.open_permit ? 1 : 0) | (Door.local_open ? 2 : 0) | (Door.local_close ? 4 : 0) | (Door.remote_open ? 8 : 0) | (Door.remote_close ? 16 : 0);
}

// what a door does, as one number: 1 closed with its step, 2 the door itself closed, 4 closing, 8 opening, 16 open,
// 32 step folding, 64 step unfolding
template <typename Door_> int door_outputs(Door_ const &Door)
{
	return (Door.is_closed ? 1 : 0) | (Door.is_door_closed ? 2 : 0) | (Door.is_closing ? 4 : 0) | (Door.is_opening ? 8 : 0) | (Door.is_open ? 16 : 0) |
	       (Door.step_folding ? 32 : 0) | (Door.step_unfolding ? 64 : 0);
}

void capture_doors(record &Entry, TMoverParameters const &Mover)
{
	Entry.set("doorlock", Mover.Doors.lock_enabled);
	Entry.set("doorlocked", Mover.Doors.is_locked);
	Entry.set("doorstep", Mover.Doors.step_enabled);
	Entry.set("doorremoteonly", Mover.Doors.remote_only);
	Entry.set("doorpreset", Mover.Doors.permit_preset);
	for (auto index{0}; index < static_cast<int>(Mover.Doors.instances.size()); ++index)
	{
		auto const &door{Mover.Doors.instances[index]};
		Entry.set(indexed("door", index, "in"), door_inputs(door));
		Entry.set(indexed("door", index, "out"), door_outputs(door));
		Entry.set(indexed("door", index, "position"), door.position);
		Entry.set(indexed("door", index, "step"), door.step_position);
		Entry.set(indexed("door", index, "timer"), door.auto_timer);
		Entry.set(indexed("door", index, "closedelay"), door.close_delay);
		Entry.set(indexed("door", index, "opendelay"), door.open_delay);
	}
}

template <typename Door_> void apply_door(Door_ &Door, record const &Entry, int const Index)
{
	if (auto inputs{0}; Entry.get(indexed("door", Index, "in"), inputs))
	{
		Door.open_permit = (inputs & 1) != 0;
		Door.local_open = (inputs & 2) != 0;
		Door.local_close = (inputs & 4) != 0;
		Door.remote_open = (inputs & 8) != 0;
		Door.remote_close = (inputs & 16) != 0;
	}
	if (auto outputs{0}; Entry.get(indexed("door", Index, "out"), outputs))
	{
		Door.is_closed = (outputs & 1) != 0;
		Door.is_door_closed = (outputs & 2) != 0;
		Door.is_closing = (outputs & 4) != 0;
		Door.is_opening = (outputs & 8) != 0;
		Door.is_open = (outputs & 16) != 0;
		Door.step_folding = (outputs & 32) != 0;
		Door.step_unfolding = (outputs & 64) != 0;
	}
	Entry.get(indexed("door", Index, "position"), Door.position);
	Entry.get(indexed("door", Index, "step"), Door.step_position);
	Entry.get(indexed("door", Index, "timer"), Door.auto_timer);
	Entry.get(indexed("door", Index, "closedelay"), Door.close_delay);
	Entry.get(indexed("door", Index, "opendelay"), Door.open_delay);
}

void apply_doors(TMoverParameters &Mover, record const &Entry)
{
	Entry.get("doorlock", Mover.Doors.lock_enabled);
	Entry.get("doorlocked", Mover.Doors.is_locked);
	Entry.get("doorstep", Mover.Doors.step_enabled);
	Entry.get("doorremoteonly", Mover.Doors.remote_only);
	Entry.get("doorpreset", Mover.Doors.permit_preset);
	for (auto index{0}; index < static_cast<int>(Mover.Doors.instances.size()); ++index)
		apply_door(Mover.Doors.instances[index], Entry, index);
}

// the temperatures of a combustion engine and what its cooling is doing
void capture_heat(record &Entry, TMoverParameters const &Mover)
{
	auto const &heat{Mover.dizel_heat};
	Entry.set("heatts", heat.Ts).set("heatto", heat.To).set("heattsr", heat.Tsr).set("heattwy", heat.Twy).set("heattsr2", heat.Tsr2).set("heattwy2", heat.Twy2);
	Entry.set("heatt1", heat.temperatura1).set("heatt2", heat.temperatura2).set("heatrpmw", heat.rpmw).set("heatrpmw2", heat.rpmw2);
	Entry.set("heatshutters1", heat.zaluzje1).set("heatshutters2", heat.zaluzje2).set("heatfault", heat.PA);
	Entry.set("heatrpmwz", heat.rpmwz).set("heatrpmwz2", heat.rpmwz2).set("heatenginehot", heat.engine_is_hot);
	// what each circuit of the cooling tells of its fluid: 1 too cold, 2 too hot, 4 past the cooling point, 8 flowing
	auto index{0};
	for (auto const *circuit : {&heat.water, &heat.water_aux, &heat.oil, &heat.engine})
	{
		Entry.set(indexed("heatcircuit", index++, ""),
		          (circuit->is_cold ? 1 : 0) | (circuit->is_warm ? 2 : 0) | (circuit->is_hot ? 4 : 0) | (circuit->is_flowing ? 8 : 0));
	}
}

void apply_heat(TMoverParameters &Mover, record const &Entry)
{
	auto &heat{Mover.dizel_heat};
	Entry.get("heatts", heat.Ts);
	Entry.get("heatto", heat.To);
	Entry.get("heattsr", heat.Tsr);
	Entry.get("heattwy", heat.Twy);
	Entry.get("heattsr2", heat.Tsr2);
	Entry.get("heattwy2", heat.Twy2);
	Entry.get("heatt1", heat.temperatura1);
	Entry.get("heatt2", heat.temperatura2);
	Entry.get("heatrpmw", heat.rpmw);
	Entry.get("heatrpmw2", heat.rpmw2);
	Entry.get("heatshutters1", heat.zaluzje1);
	Entry.get("heatshutters2", heat.zaluzje2);
	Entry.get("heatfault", heat.PA);
	Entry.get("heatrpmwz", heat.rpmwz);
	Entry.get("heatrpmwz2", heat.rpmwz2);
	Entry.get("heatenginehot", heat.engine_is_hot);
	auto index{0};
	for (auto *circuit : {&heat.water, &heat.water_aux, &heat.oil, &heat.engine})
	{
		if (auto state{0}; Entry.get(indexed("heatcircuit", index++, ""), state))
		{
			circuit->is_cold = (state & 1) != 0;
			circuit->is_warm = (state & 2) != 0;
			circuit->is_hot = (state & 4) != 0;
			circuit->is_flowing = (state & 8) != 0;
		}
	}
}

// the air of a reservoir: what it holds and, when there's any, the flow it has taken since it last acted. a flow
// added late in a step (the feed of the brake reservoir from the main pipe) waits there for the next one
void capture_reservoir(record &Entry, std::string const &Key, TReservoir const &Reservoir)
{
	Entry.set(Key, Reservoir.GetVol());
	if (Reservoir.GetPendingFlow() != 0.0)
		Entry.set(Key + "flow", Reservoir.GetPendingFlow());
}

bool apply_reservoir(TReservoir &Reservoir, record const &Entry, std::string const &Key)
{
	auto volume{0.0};
	if (false == Entry.get(Key, volume))
		return false;
	Reservoir.SetVol(volume, Entry.value(Key + "flow", 0.0));
	return true;
}

// the air of the brake: the pipes of the vehicle, the reservoirs of its valve, the valve of the driver, the spring brake
void capture_brake(record &Entry, TMoverParameters const &Mover)
{
	if (Mover.Pipe != nullptr)
		capture_reservoir(Entry, "pipeair", *Mover.Pipe);
	if (Mover.Pipe2 != nullptr)
		capture_reservoir(Entry, "scndpipeair", *Mover.Pipe2);
	// the brake set for an empty, a part loaded or a loaded vehicle
	Entry.set("brakeload", Mover.LoadFlag);
	if (Mover.Hamulec != nullptr)
	{
		Entry.set("brakestatus", Mover.Hamulec->GetBrakeStatus());
		Entry.set("releaser", Mover.Hamulec->Releaser());
		// the latches of its inner valves, the step of its rapid relay
		Entry.set("brakelatches", numbers_text(Mover.Hamulec->latches()));
		auto index{0};
		for (auto const *reservoir : Mover.Hamulec->reservoirs())
		{
			if (reservoir != nullptr)
				capture_reservoir(Entry, indexed("res", index, ""), *reservoir);
			else
				Entry.set(indexed("res", index, ""), 0.0);
			++index;
		}
	}
	if (Mover.Handle != nullptr)
		capture_handle(Entry, "handle", *Mover.Handle);
	if (Mover.LocHandle != nullptr)
		capture_handle(Entry, "lochandle", *Mover.LocHandle);
	Entry.set("springbrake", Mover.SpringBrake.Activate);
	Entry.set("springbrakeshutoff", Mover.SpringBrake.ShuttOff);
	Entry.set("springbrakerelease", Mover.SpringBrake.Release);
	Entry.set("springbrakeactive", Mover.SpringBrake.IsActive);
	Entry.set("springbrakeready", Mover.SpringBrake.IsReady);
	Entry.set("springbrakepress", Mover.SpringBrake.SBP);
	if (Mover.SpringBrake.Cylinder != nullptr)
		capture_reservoir(Entry, "springbrakevolume", *Mover.SpringBrake.Cylinder);
}

void apply_brake_valve(TBrake &Valve, TMoverParameters const &Mover, record const &Entry)
{
	auto const reservoirs{Valve.reservoirs()};
	// the air of each reservoir, when the entry has it for the reservoirs this kind of valve has; otherwise the valve
	// is set as the scenery sets it at the start, to the pressures of the pipe and of the cylinders
	auto const complete{false == reservoirs.empty() && Entry.has(indexed("res", static_cast<int>(reservoirs.size()) - 1, "")) &&
	                    false == Entry.has(indexed("res", static_cast<int>(reservoirs.size()), ""))};
	if (false == complete)
	{
		if (Entry.has("pipe"))
			Valve.Init(Mover.PipePress, Mover.HighPipePress, Mover.LowPipePress, Mover.BrakePress, Mover.BrakeDelayFlag);
	}
	else
	{
		auto index{0};
		for (auto *reservoir : reservoirs)
		{
			if (reservoir != nullptr)
				apply_reservoir(*reservoir, Entry, indexed("res", index, ""));
			++index;
		}
	}
	if (Entry.has("brakedelay"))
		Valve.SetBDF(Mover.BrakeDelayFlag);
	if (auto status{0}; Entry.get("brakestatus", status))
		Valve.SetBrakeStatus(status);
	// of the same kind of valve only: another kind has other latches
	if (auto const latches{text_numbers(Entry.text("brakelatches"))}; latches.size() == Valve.latches().size())
		Valve.latches(latches);
	if (auto releaser{false}; Entry.get("releaser", releaser) && releaser != Valve.Releaser())
		Valve.Releaser(releaser ? 1 : 0);
}

void apply_brake(TMoverParameters &Mover, record const &Entry)
{
	// the pipes of the vehicle hold the pressures the physics reads them from: by their air, or, for an entry which
	// only gives the pressures, filled to those
	if (Mover.Pipe != nullptr && false == apply_reservoir(*Mover.Pipe, Entry, "pipeair") && Entry.has("pipe"))
	{
		Mover.Pipe->CreatePress(Mover.PipePress);
		Mover.Pipe->Act();
	}
	if (Mover.Pipe2 != nullptr && false == apply_reservoir(*Mover.Pipe2, Entry, "scndpipeair") && Entry.has("scndpipe"))
	{
		Mover.Pipe2->CreatePress(Mover.ScndPipePress);
		Mover.Pipe2->Act();
	}
	if (auto setting{Mover.LoadFlag}; Entry.get("brakeload", setting))
	{
		// by the steps of the lever, which change the ratio of the rigging with it
		auto moved{true};
		while (moved && Mover.LoadFlag != setting)
			moved = Mover.LoadFlag < setting ? Mover.IncBrakeMult() : Mover.DecBrakeMult();
	}
	if (Mover.Hamulec != nullptr)
		apply_brake_valve(*Mover.Hamulec, Mover, Entry);
	if (Mover.Handle != nullptr)
		apply_handle(Entry, "handle", *Mover.Handle);
	if (Mover.LocHandle != nullptr)
		apply_handle(Entry, "lochandle", *Mover.LocHandle);
	Entry.get("springbrake", Mover.SpringBrake.Activate);
	Entry.get("springbrakeshutoff", Mover.SpringBrake.ShuttOff);
	Entry.get("springbrakerelease", Mover.SpringBrake.Release);
	Entry.get("springbrakeactive", Mover.SpringBrake.IsActive);
	Entry.get("springbrakeready", Mover.SpringBrake.IsReady);
	Entry.get("springbrakepress", Mover.SpringBrake.SBP);
	if (Mover.SpringBrake.Cylinder != nullptr)
		apply_reservoir(*Mover.SpringBrake.Cylinder, Entry, "springbrakevolume");
}

// the load changes the mass, and with it what the brakes are set for
void apply_load(TMoverParameters &Mover, record const &Entry)
{
	auto amount{Mover.LoadAmount};
	auto name{Mover.LoadType.name};
	auto const given{Entry.get("load", amount)};
	if (false == Entry.get("loadtype", name) && false == given)
		return;
	if (amount == Mover.LoadAmount && name == Mover.LoadType.name)
		return;
	if (name == Mover.LoadType.name)
	{
		// more or less of what it carries
		Mover.LoadAmount = amount;
		Mover.ComputeMass();
		return;
	}
	// the vehicle is emptied first: it doesn't take a load of another kind on top of the one it has
	Mover.AssignLoad("");
	if (false == name.empty())
	{
		Mover.AssignLoad(name, amount);
		// a load the vehicle isn't made for, as the scenery can give it one all the same, or one with nothing left of it
		if (Mover.LoadType.name != name)
			Mover.LoadType.name = name;
		// AssignLoad() keeps the amount within what the vehicle is rated for, a vehicle loaded over that keeps what it had
		Mover.LoadAmount = amount;
	}
	Mover.ComputeMass();
}

} // namespace

// the track under the vehicle as the last step was given it. the next step works its constants out from this one
// (the resistance of a curve in ComputeConstans()) before it's given the track anew
void capture_running(record &Entry, TMoverParameters const &Mover)
{
	auto const &shape{Mover.RunningShape};
	Entry.set("runningshape",
	          numbers_text(std::array{shape.R, shape.Len, shape.dHtrack, shape.dHrail, shape.uneven_curvature, shape.uneven_cross, shape.buckle_curvature}));
	auto const &track{Mover.RunningTrack};
	Entry.set("runningtrack", numbers_text(std::array{track.Width, track.friction, static_cast<double>(track.CategoryFlag),
	                                                  static_cast<double>(track.QualityFlag), static_cast<double>(track.DamageFlag),
	                                                  std::isfinite(track.Velmax) ? track.Velmax : 0.0}));
	auto const &traction{Mover.RunningTraction};
	Entry.set("runningtraction",
	          numbers_text(std::array{traction.TractionVoltage, traction.TractionFreq, traction.TractionMaxCurrent, traction.TractionResistivity}));
	// where the physics has the vehicle: at the place it started the last step from, a frame behind its body. the gaps
	// between the couplers are measured between these
	if (std::isfinite(Mover.Loc.X) && std::isfinite(Mover.Loc.Y) && std::isfinite(Mover.Loc.Z))
		Entry.set("loc", numbers_text(std::array{Mover.Loc.X, Mover.Loc.Y, Mover.Loc.Z}));
}

void apply_running(TMoverParameters &Mover, record const &Entry)
{
	if (auto const numbers{text_numbers(Entry.text("runningshape"))}; numbers.size() == 7)
	{
		auto &shape{Mover.RunningShape};
		shape.R = numbers[0];
		shape.Len = numbers[1];
		shape.dHtrack = numbers[2];
		shape.dHrail = numbers[3];
		shape.uneven_curvature = numbers[4];
		shape.uneven_cross = numbers[5];
		shape.buckle_curvature = numbers[6];
	}
	if (auto const numbers{text_numbers(Entry.text("runningtrack"))}; numbers.size() == 6)
	{
		auto &track{Mover.RunningTrack};
		track.Width = numbers[0];
		track.friction = numbers[1];
		track.CategoryFlag = static_cast<int>(numbers[2]);
		track.QualityFlag = static_cast<int>(numbers[3]);
		track.DamageFlag = static_cast<int>(numbers[4]);
		track.Velmax = numbers[5];
	}
	if (auto const numbers{text_numbers(Entry.text("loc"))}; numbers.size() == 3)
		Mover.Loc = {numbers[0], numbers[1], numbers[2]};
	if (auto const numbers{text_numbers(Entry.text("runningtraction"))}; numbers.size() == 4)
	{
		auto &traction{Mover.RunningTraction};
		traction.TractionVoltage = numbers[0];
		traction.TractionFreq = numbers[1];
		traction.TractionMaxCurrent = numbers[2];
		traction.TractionResistivity = numbers[3];
	}
}

// what each coupler keeps from the step before: its force and the deflection of the buffers, how long it has been
// overstretched, the collision it's in, and the springs of the physical model of the pair, which give by where they were
void capture_couplers(record &Entry, TMoverParameters const &Mover)
{
	for (auto side{0}; side < 2; ++side)
	{
		auto const &coupler{Mover.Couplers[side]};
		auto const &pair{coupler.physics_state};
		Entry.set(indexed("couplerstate", side, ""),
		          numbers_text(std::array{coupler.CForce, coupler.Dist, static_cast<double>(coupler.stretch_duration), coupler.CheckCollision ? 1.0 : 0.0,
		                                  coupler.crush, coupler.closing, coupler.hit_handled ? 1.0 : 0.0, coupler.hit_force, coupler.hit_duration,
		                                  pair.draw_gear.force, pair.draw_gear.deflection, pair.buffers.force, pair.buffers.deflection, pair.elongation,
		                                  pair.gap, static_cast<double>(pair.stop), pair.stop_force}));
	}
}

void apply_couplers(TMoverParameters &Mover, record const &Entry)
{
	for (auto side{0}; side < 2; ++side)
	{
		auto const values{text_numbers(Entry.text(indexed("couplerstate", side, "")))};
		if (values.size() != 17)
			continue;
		auto &coupler{Mover.Couplers[side]};
		auto &pair{coupler.physics_state};
		coupler.CForce = values[0];
		coupler.Dist = values[1];
		coupler.stretch_duration = static_cast<float>(values[2]);
		coupler.CheckCollision = values[3] != 0.0;
		coupler.crush = values[4];
		coupler.closing = values[5];
		coupler.hit_handled = values[6] != 0.0;
		coupler.hit_force = values[7];
		coupler.hit_duration = values[8];
		pair.draw_gear.force = values[9];
		pair.draw_gear.deflection = values[10];
		pair.buffers.force = values[11];
		pair.buffers.deflection = values[12];
		pair.elongation = values[13];
		pair.gap = values[14];
		pair.stop = static_cast<int>(values[15]);
		pair.stop_force = values[16];
	}
}

void capture_mover(record &Entry, TMoverParameters const &Mover)
{
	Entry.set("load", Mover.LoadAmount);
	Entry.set("loadtype", Mover.LoadType.name);
	capture_members(Entry, Mover, motion);
	capture_members(Entry, Mover, forces);
	capture_members(Entry, Mover, positions);
	capture_members(Entry, Mover, levels);
	capture_members(Entry, Mover, switches);
	capture_members(Entry, Mover, relays);
	capture_devices(Entry, Mover);
	capture_doors(Entry, Mover);
	capture_heat(Entry, Mover);
	capture_members(Entry, Mover, air);
	capture_brake(Entry, Mover);
	capture_power(Entry, Mover);
	capture_running(Entry, Mover);
	capture_couplers(Entry, Mover);
}

void apply_mover(TMoverParameters &Mover, record const &Entry)
{
	apply_load(Mover, Entry);
	apply_members(Mover, Entry, motion);
	apply_members(Mover, Entry, forces);
	apply_members(Mover, Entry, positions);
	apply_members(Mover, Entry, levels);
	apply_members(Mover, Entry, switches);
	apply_members(Mover, Entry, relays);
	apply_devices(Mover, Entry);
	apply_doors(Mover, Entry);
	apply_heat(Mover, Entry);
	apply_members(Mover, Entry, air);
	apply_brake(Mover, Entry);
	apply_power(Mover, Entry);
	apply_running(Mover, Entry);
	apply_couplers(Mover, Entry);
}

} // namespace simulation::snapshot
