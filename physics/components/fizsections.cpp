/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// reading the vehicle definition (.fiz): the sections and their keywords

#include "stdafx.h"
#include "physics/mover.h"
#include "vehicle/DynObj.h"
#include "physics/oerlikonest.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/Timer.h"
#include "utilities/parser.h"
#include "simulation/simulation.h"
#include "vehicle/fieldshunting.h"
#include <algorithm>
#include <charconv>
#include <cmath>
#include <format>
#include <functional>
#include <limits>
#include <utility>
#include <vector>
#include "physics/components/internal.h"


int s2NPW(std::string s)
{ // wylicza ilosc osi napednych z opisu ukladu osi
	const char A = 64;
	int NPW = 0;
	for (char const chentry : s)
	{
		if (chentry >= (char)65 && chentry <= (char)90)
			NPW += chentry - A;
	}
	return NPW;
}

int s2NNW(std::string s)
{ // wylicza ilosc osi nienapedzanych z opisu ukladu osi
	const char Zero = 48;
	int NNW = 0;
	for (char const ch : s)
	{
		if (ch >= (char)49 && ch <= (char)57)
			NNW += ch - Zero;
	}
	return NNW;
}

void TMoverParameters::LoadFIZ_Param(std::string const &line)
{

	extract_value(Mass, "M", line, "0");
	extract_value(Mred, "Mred", line, "0");
	extract_value(Vmax, "Vmax", line, "0");
	extract_value(Power, "PWR", line, "0");
	extract_value(SandCapacity, "SandCap", line, "0");
	extract_value(HeatingPower, "HeatingP", line, "0");
	extract_value(LightPower, "LightP", line, "0");

	{
		std::map<std::string, int, std::less<>> categories{{"train", 1}, {"road", 2}, {"unimog", 3}, {"ship", 4}, {"airplane,", 8}};
		std::string category;
		extract_value(category, "Category", line, "none");
		auto lookup = categories.find(category);
		CategoryFlag = lookup != categories.end() ? lookup->second : 0;
		if (CategoryFlag == 0)
		{
			ErrorLog("Unknown vehicle category: \"" + category + "\".");
		}
	}

	{
		std::map<std::string, train_type, std::less<>> types{
		    {"pseudodiesel", train_type::pseudodiesel},
		    {"ezt", train_type::emu},
		    {"dmu", train_type::dmu},
		    {"sn61", train_type::sn61},
		    {"et22", train_type::et22},
		    {"et40", train_type::et40},
		    {"et41", train_type::et41},
		    {"et42", train_type::et42},
		    {"ep05", train_type::ep05},
		    {"181", train_type::skoda181},
		    {"182", train_type::skoda181} // na razie tak
		};
		std::string type;
		extract_value(type, "Type", line, "none");
		auto lookup = types.find(ToLower(type));
		TrainType = lookup != types.end() ? lookup->second : train_type::generic;
	}

	if (TrainType == train_type::emu)
	{

		IminLo = 1;
		IminHi = 2;
		Imin = 1;
	}
}

void TMoverParameters::LoadFIZ_Load(std::string const &line)
{

	auto const acceptedloads{Split(extract_value("LoadAccepted", line), ',')};

	if (acceptedloads.empty())
	{
		return;
	}

	auto const minoffsets{Split(extract_value("LoadMinOffset", line), ',')};
	auto minoffset{0.f};
	auto minoffsetsiterator{std::begin(minoffsets)};
	// NOTE: last (if any) offset parameter retrieved from the list applies to the remainder of the list
	// TBD, TODO: include other load parameters in this system
	for (auto &load : acceptedloads)
	{
		if (minoffsetsiterator != std::end(minoffsets))
		{
			minoffset = std::stof(*minoffsetsiterator);
			++minoffsetsiterator;
		}
		LoadAttributes.emplace_back(ToLower(load), minoffset);
	}

	extract_value(MaxLoad, "MaxLoad", line, "");
	extract_value(LoadQuantity, "LoadQ", line, "");
	extract_value(OverLoadFactor, "OverLoadFactor", line, "");
	extract_value(LoadSpeed, "LoadSpeed", line, "");
	extract_value(UnLoadSpeed, "UnLoadSpeed", line, "");
}

void TMoverParameters::LoadFIZ_Headlights(std::string const &line)
{
	extract_value(refR, "LampRed", line, "");
	extract_value(refG, "LampGreen", line, "");
	extract_value(refB, "LampBlue", line, "");

	extract_value(dimMultiplier, "DimmedMultiplier", line, "");
	extract_value(normMultiplier, "NormalMultiplier", line, "");
	extract_value(highDimMultiplier, "HighbeamDimmedMultiplier", line, "");
	extract_value(highMultiplier, "HighBeamMultiplier", line, "");
}

void TMoverParameters::LoadFIZ_Dimensions(std::string const &line)
{

	extract_value(Dim.L, "L", line, "");
	extract_value(Dim.H, "H", line, "");
	extract_value(Dim.W, "W", line, "");
	// height of the centre of gravity of the empty vehicle [m], for the criteria of derailment
	extract_value(CentreOfGravity, "CoGH", line, "");

	extract_value(Cx, "Cx", line, "0.3");
	if (Dim.H <= 2.0)
	{
		// gdyby nie było parametru, lepsze to niż zero
		Floor = static_cast<float>(Dim.H);
	}
	else
	{
		// zgodność wsteczna
		Floor = 0.0;
	}
	extract_value(Floor, "Floor", line, "");
}

void TMoverParameters::LoadFIZ_Wheels(std::string const &line)
{

	extract_value(WheelDiameter, "D", line, "");
	WheelDiameterL = WheelDiameter; // gdyby nie było parametru, lepsze to niż zero
	extract_value(WheelDiameterL, "Dl", line, "");
	WheelDiameterT = WheelDiameter; // gdyby nie było parametru, lepsze to niż zero
	extract_value(WheelDiameterT, "Dt", line, "");

	extract_value(TrackW, "Tw", line, "");
	extract_value(AxleInertialMoment, "AIM", line, "");

	extract_value(AxleArangement, "Axle", line, "");
	NPoweredAxles = s2NPW(AxleArangement);
	NAxles = NPoweredAxles + s2NNW(AxleArangement);

	BearingType = extract_value("BearingType", line) == "Roll" ? 1 : 0;

	extract_value(ADist, "Ad", line, "");
	extract_value(BDist, "Bd", line, "");

	if (AxleInertialMoment <= 0.0)
	{
		/*
		        AxleInertialMoment = 1.0;
		*/
		// approximation formula by youby
		auto const k = 472.0; // arbitrary constant
		AxleInertialMoment = k / 4.0 * std::pow(WheelDiameter, 4.0) * NAxles;
		Mred = k * std::pow(WheelDiameter, 2.0) * NAxles;
	}
}

void TMoverParameters::LoadFIZ_Brake(std::string const &line)
{

	extract_value(BrakeValveParams, "BrakeValve", line, "");
	BrakeValveDecode(BrakeValveParams);
	BrakeSubsystemDecode();

	extract_value(NBpA, "NBpA", line, "");
	extract_value(MaxBrakeForce, "MBF", line, "");
	extract_value(BrakeValveSize, "Size", line, "");
	extract_value(TrackBrakeForce, "TBF", line, "");
	TrackBrakeForce *= 1000.0;

	extract_value(MaxBrakePress[3], "MaxBP", line, "");
	if (MaxBrakePress[3] > 0.0)
	{

		extract_value(BrakeCylNo, "BCN", line, "");
		if (BrakeCylNo > 0)
		{

			extract_value(MaxBrakePress[0], "MaxLBP", line, "");
			if (MaxBrakePress[0] < 0.01)
			{
				MaxBrakePress[0] = MaxBrakePress[3];
			}
			extract_value(MaxBrakePress[1], "TareMaxBP", line, "");
			extract_value(MaxBrakePress[2], "MedMaxBP", line, "");
			extract_value(MaxBrakePress[4], "MaxASBP", line, "");
			if (MaxBrakePress[4] < 0.01)
			{
				MaxBrakePress[4] = 0.0;
			}

			extract_value(BrakeCylRadius, "BCR", line, "");
			extract_value(BrakeCylDist, "BCD", line, "");
			extract_value(BrakeCylSpring, "BCS", line, "");
			extract_value(BrakeSlckAdj, "BSA", line, "");
			extract_value(BrakeRigEff, "BRE", line, "1");

			extract_value(BrakeCylMult[0], "BCM", line, "");
			extract_value(BrakeCylMult[1], "BCMlo", line, "");
			extract_value(BrakeCylMult[2], "BCMHi", line, "");

			P2FTrans = 100 * M_PI * std::pow(BrakeCylRadius, 2); // w kN/bar

			if (BrakeCylMult[1] > 0.0 || MaxBrakePress[1] > 0.0)
			{
				LoadFlag = 1;
			}
			else
			{
				LoadFlag = 0;
			}

			BrakeVolume = M_PI * std::pow(BrakeCylRadius, 2) * BrakeCylDist * BrakeCylNo;
			extract_value(BrakeVVolume, "BVV", line, "");

			{
				std::map<std::string, int, std::less<>> brakemethods{{"P10-Bg", bp_P10Bg},  {"P10-Bgu", bp_P10Bgu},  {"FR513", bp_FR513}, {"FR510", bp_FR510},          {"Cosid", bp_Cosid},
				                                        {"P10yBg", bp_P10yBg}, {"P10yBgu", bp_P10yBgu}, {"Disk1", bp_D1},    {"Disk1+Mg", bp_D1 + bp_MHS}, {"Disk2", bp_D2}};
				auto lookup = brakemethods.find(extract_value("BM", line));
				BrakeMethod = lookup != brakemethods.end() ? lookup->second : 0;
			}

			extract_value(RapidMult, "RM", line, "1");
			extract_value(RapidVel, "RV", line, "55");
		}
	}
	else
	{
		// maxbrakepress[3] == 0 or less
		P2FTrans = 0;
	}

	// Ra 2014-07: trochę niedokładności
	// the draws are sequenced: the order of the operands of '-' is unspecified, and GCC and MSVC draw them the other way round
	auto const firstdraw{Random(10)};
	CntrlPipePress = 5 + 0.001 * (Random(10) - firstdraw);
	extract_value(CntrlPipePress, "HiPP", line, "");
	HighPipePress = CntrlPipePress;
	LowPipePress = std::min(HighPipePress, 3.5);
	extract_value(LowPipePress, "LoPP", line, "");
	DeltaPipePress = HighPipePress - LowPipePress;

	extract_value(VeselVolume, "Vv", line, "");
	/*
	    if( VeselVolume == 0.0 ) { VeselVolume = 0.01; }
	*/
	extract_value(MinCompressor, "MinCP", line, "");
	extract_value(MaxCompressor, "MaxCP", line, "");
	extract_value(MinCompressor_cabB, "MinCP_B", line, "");
	extract_value(MaxCompressor_cabB, "MaxCP_B", line, "");
	extract_value(CompressorTankValve, "CompressorTankValve", line, "");
	extract_value(CompressorSpeed, "CompressorSpeed", line, "");
	extract_value(EmergencyValveOff, "MinEVP", line, "");
	extract_value(EmergencyValveOn, "MaxEVP", line, "");
	extract_value(EmergencyValveArea, "EVArea", line, "");
	extract_value(UniversalBrakeButtonFlag[0], "UBB1", line, "");
	extract_value(UniversalBrakeButtonFlag[1], "UBB2", line, "");
	extract_value(UniversalBrakeButtonFlag[2], "UBB3", line, "");
	extract_value(LockPipeOn, "LPOn", line, "-1");
	extract_value(LockPipeOff, "LPOff", line, "-1");
	extract_value(HandleUnlock, "HandlePipeUnlockPos", line, "-3");
	extract_value(EmergencyCutsOffHandle, "EmergencyCutsOffHandle", line, "");
	{
		std::map<std::string, int, std::less<>> compressorpowers{
		    {"Main", 0},
		    // 1: default, powered by converter, with manual state control
		    {"Converter", 2},
		    {"Engine", 3}, // equivalent of 0, TODO: separate 'main' and 'engine' in the code
		    {"Coupler1", 4}, // włączana w silnikowym EZT z przodu
		    {"Coupler2", 5} // włączana w silnikowym EZT z tyłu
		};
		auto lookup = compressorpowers.find(extract_value("CompressorPower", line));
		CompressorPower = lookup != compressorpowers.end() ? lookup->second : 1;
	}
	extract_value(CompressorStartDelay, "CompressorStartDelay", line, "");

	if (true == extract_value(AirLeakRate, "AirLeakRate", line, ""))
	{
		// the parameter is provided in form of a multiplier, where 1.0 means the default rate of 0.01
		AirLeakRate *= 0.01;
	}

	extract_value(ReleaserEnabledOnlyAtNoPowerPos, "ReleaserPowerPosLock", line, EngineType == TEngineType::DieselEngine || EngineType == TEngineType::DieselElectric ? "yes" : "no");

	if (MinCompressor_cabB > 0.0)
	{
		MinCompressor_cabA = MinCompressor;
		CabDependentCompressor = true;
	}
	else
	{
		MinCompressor_cabB = MinCompressor;
	}
	if (MaxCompressor_cabB > 0.0)
	{
		MaxCompressor_cabA = MaxCompressor;
		CabDependentCompressor = true;
	}
	else
	{
		MaxCompressor_cabB = MaxCompressor;
	}
}

void TMoverParameters::LoadFIZ_Doors(std::string const &line)
{

	std::map<std::string, control_t, std::less<>> doorcontrols{
	    {"Passenger", control_t::passenger}, {"AutomaticCtrl", control_t::autonomous}, {"DriverCtrl", control_t::driver}, {"Conductor", control_t::conductor}, {"Mixed", control_t::mixed}};
	// opening method
	{
		auto lookup = doorcontrols.find(extract_value("OpenCtrl", line));
		Doors.open_control = lookup != doorcontrols.end() ? lookup->second : control_t::passenger;
	}
	// closing method
	{
		auto lookup = doorcontrols.find(extract_value("CloseCtrl", line));
		Doors.close_control = lookup != doorcontrols.end() ? lookup->second : control_t::passenger;
	}
	// automatic closing conditions
	extract_value(Doors.auto_duration, "DoorStayOpen", line, "");
	extract_value(Doors.auto_velocity, "DoorAutoCloseVel", line, "");
	extract_value(Doors.auto_include_remote, "DoorAutoCloseRemote", line, "");
	// operation permit
	extract_value(Doors.permit_needed, "DoorNeedPermit", line, "");
	// the door closure lamp (SOD) of the cab signals the doors of the whole consist
	extract_value(Doors.closure_signal, "DoorClosureSignal", line, "");
	{
		auto permitpresets = Split(extract_value("DoorPermitList", line), '|');
		for (auto const &permit : permitpresets)
		{
			Doors.permit_presets.emplace_back(std::stoi(permit));
		}
		if (false == Doors.permit_presets.empty())
		{
			// HACK: legacy position indices start from 1, so we deduct 1 to arrive at proper index into the array
			extract_value(Doors.permit_preset, "DoorPermitListDefault", line, "1");
			Doors.permit_preset = std::min<int>(static_cast<int>(Doors.permit_presets.size()), Doors.permit_preset) - 1;
		}
	}

	extract_value(Doors.open_rate, "OpenSpeed", line, "");
	extract_value(Doors.open_delay, "DoorOpenDelay", line, "");
	extract_value(Doors.close_rate, "CloseSpeed", line, "");
	extract_value(Doors.close_delay, "DoorCloseDelay", line, "");
	extract_value(Doors.range, "DoorMaxShiftL", line, "");
	extract_value(Doors.range, "DoorMaxShiftR", line, "");
	extract_value(Doors.range_out, "DoorMaxShiftPlug", line, "");

	std::map<std::string, int, std::less<>> doortypes{
	    {"Shift", 1},
	    {"Rotate", 2},
	    {"Fold", 3},
	    {"Plug", 4},
	};
	// opening method
	{
		auto lookup = doortypes.find(extract_value("DoorOpenMethod", line));
		Doors.type = lookup != doortypes.end() ? lookup->second : 2; // default type is plain, rotating door
	}

	extract_value(Doors.has_warning, "DoorClosureWarning", line, "");
	extract_value(Doors.has_autowarning, "DoorClosureWarningAuto", line, "");
	extract_value(Doors.has_lock, "DoorBlocked", line, "");
	extract_value(Doors.doorLockSpeed, "DoorLockSpeed", line, "");
	{
		auto const remotedoorcontrol{Doors.open_control == control_t::driver || Doors.open_control == control_t::conductor || Doors.open_control == control_t::mixed};

		extract_value(Doors.voltage, "DoorVoltage", line, remotedoorcontrol ? "24" : "0");
	}

	extract_value(Doors.step_rate, "PlatformSpeed", line, "");
	extract_value(Doors.step_range, "PlatformMaxShift", line, "");

	std::string platformopenmethod;
	extract_value(platformopenmethod, "PlatformOpenMethod", line, "");
	if (platformopenmethod == "Shift")
	{
		Doors.step_type = 1;
	} // przesuw

	extract_value(MirrorMaxShift, "MirrorMaxShift", line, "");
	extract_value(MirrorVelClose, "MirrorVelClose", line, "");

	extract_value(DoorsOpenWithPermitAfter, "DoorOpenWithPermit", line, "");
	extract_value(DoorsPermitLightBlinking, "DoorsPermitLightBlinking", line, "");
}
namespace
{


// the value of an optional key of the line, or the default without it
double optional_value(std::string const &Line, std::string const &Key, double const Default)
{
	auto value{Default};
	extract_value(value, Key, Line, "");
	return value;
}

// the value of an optional key of the line given in kilo-units, in units; or the default (in units) without it
double optional_kilo_value(std::string const &Line, std::string const &Key, double const Default)
{
	auto value{0.0};
	return extract_value(value, Key, Line, "") && value > 0.0 ? value * 1000.0 : Default;
}

// the play of a coupling as made, for each kind: a screw coupling is tightened so that the buffers touch, the knuckles of
// an automatic coupler have about a centimetre between them, a chain hangs loose
double default_slack(TCouplerType const Type)
{
	switch (Type)
	{
	case TCouplerType::Automatic:
		return 0.005;
	case TCouplerType::Chain:
		return 0.05;
	default:
		return 0.0;
	}
}

// the parameters of the physical model of the coupling (couplerphysics in eu07.ini, see couplerphysics.h), taken from
// those of the legacy model, and the optional keys of the .fiz for what the legacy model doesn't have:
//   Slack [m] the play of the coupling of this end, negative for a screw coupling tightened to press the buffers in
//   ProgB, ProgC [-] progression of the buffers and of the draw gear, 0 linear, up to 0.9
//   UnloadB, UnloadC [-] force of the unloading curve of the buffers and of the draw gear to that of the loading one
//   FyieldC [kN] the force past which the link stretches for good; DbreakC [m] the elongation at which it breaks
//   kL [kN/m] stiffness of the steel parts past the travel of the springs; FcrushB [kN] the buffers give under it
//   RestitutionSolid [-] coefficient of restitution of a hit on the springs gone solid
coupler_physics::end_parameters coupler_physics_parameters(TCoupling const &Coupler, std::string const &Line)
{
	coupler_physics::end_parameters parameters;
	parameters.buffers = {Coupler.SpringKB, Coupler.DmaxB, std::clamp(optional_value(Line, "ProgB", 0.5), 0.0, 0.9)};
	parameters.draw_gear = {Coupler.SpringKC, Coupler.DmaxC, std::clamp(optional_value(Line, "ProgC", 0.3), 0.0, 0.9)};
	// only a screw coupling can be tightened to press the buffers in, and not past their travel
	auto const tightest{Coupler.CouplerType == TCouplerType::Screw ? -Coupler.DmaxB : 0.0};
	parameters.slack = std::max(optional_value(Line, "Slack", default_slack(Coupler.CouplerType)), tightest);
	// the legacy model sprang back at beta of the force it was loaded with
	auto const unloading{Coupler.beta > 0.0 && Coupler.beta < 1.0 ? Coupler.beta : 0.5};
	parameters.buffers_unloading = std::clamp(optional_value(Line, "UnloadB", unloading), 0.05, 1.0);
	parameters.draw_gear_unloading = std::clamp(optional_value(Line, "UnloadC", unloading), 0.05, 1.0);
	// as much as the legacy model had
	parameters.damping = CouplerTune * (Coupler.FmaxC + Coupler.FmaxB) * std::max(Coupler.beta, 0.0);

	// FmaxC is the rated breaking force of the coupling
	auto const ultimate{std::max(Coupler.FmaxC, 1.0)};
	auto const yield{std::min(optional_kilo_value(Line, "FyieldC", 0.8 * ultimate), ultimate)};
	auto const elongation{std::max(optional_value(Line, "DbreakC", 0.02), 0.001)};
	parameters.link = {yield, ultimate, std::max((ultimate - yield) / elongation, 1.0), Coupler.CouplerType != TCouplerType::Articulated};
	// the steel parts give a few millimetres under the rated force
	parameters.link_stiffness = optional_kilo_value(Line, "kL", ultimate / 0.003);
	parameters.crush_force = optional_kilo_value(Line, "FcrushB", 2.5 * std::max(Coupler.FmaxB, Coupler.FmaxC));
	parameters.solid_restitution = std::clamp(optional_value(Line, "RestitutionSolid", parameters.solid_restitution), 0.05, 0.95);
	return parameters;
}

void scale_strength(coupler_physics::link_strength &Link, double const Factor)
{
	Link.yield *= Factor;
	Link.ultimate *= Factor;
}

// the optional keys of the collisions past the buffers (physics.collisions new, see collision.h), what's not given
// comes from the kind of the vehicle:
//   FcrushPeak, FcrushPlateau [kN] the force the end starts to crush at, and keeps crushing at
//   CrushOnset, CrushDensify, CrushLimit [m] the crush the peak drops at, it packs down past, it can't go past
//   CrushHardening [1/m] the growth of the force past CrushDensify, to the plateau for each metre
//   CrashZone [yes/no] built to EN 15227: energy absorbers and anticlimbers; AntiClimber [yes/no] on its own
//   BufferHeight [m] of the empty vehicle above the rail
//   BufferCategory [A/B/C/L/P] the buffers of EN 15551 (P those of the coaches) in place of kB and DmaxB
collision::end_definition collision_definition(std::string const &Line)
{
	collision::end_definition definition;
	auto const number = [&Line](std::string const &Key, double const Scale) -> std::optional<double> {
		auto value{0.0};
		if (extract_value(value, Key, Line, "") && value >= 0.0)
		{
			return value * Scale;
		}
		return std::nullopt;
	};
	auto const flag = [&Line](std::string const &Key) -> std::optional<bool> {
		auto value{false};
		if (extract_value(value, Key, Line, ""))
		{
			return value;
		}
		return std::nullopt;
	};
	definition.peak = number("FcrushPeak", 1000.0);
	definition.plateau = number("FcrushPlateau", 1000.0);
	definition.onset = number("CrushOnset", 1.0);
	definition.densify = number("CrushDensify", 1.0);
	definition.hardening = number("CrushHardening", 1.0);
	definition.limit = number("CrushLimit", 1.0);
	definition.crash_zone = flag("CrashZone");
	definition.anticlimber = flag("AntiClimber");
	definition.buffer_height = number("BufferHeight", 1.0);
	return definition;
}

} // namespace

void TMoverParameters::LoadFIZ_BuffCoupl(std::string const &line, int const Index)
{

	TCoupling *thiscoupler;
	if (Index == 2)
	{
		thiscoupler = &Couplers[1];
	}
	else
	{
		thiscoupler = &Couplers[0];
	}

	std::map<std::string, TCouplerType, std::less<>> couplertypes{
	    {"Automatic", TCouplerType::Automatic}, {"Screw", TCouplerType::Screw}, {"Chain", TCouplerType::Chain}, {"Bare", TCouplerType::Bare}, {"Articulated", TCouplerType::Articulated},
	};
	auto lookup = couplertypes.find(extract_value("CType", line));
	thiscoupler->CouplerType = lookup != couplertypes.end() ? lookup->second : TCouplerType::NoCoupler;

	extract_value(thiscoupler->SpringKC, "kC", line, "");
	extract_value(thiscoupler->DmaxC, "DmaxC", line, "");
	extract_value(thiscoupler->FmaxC, "FmaxC", line, "");
	extract_value(thiscoupler->SpringKB, "kB", line, "");
	extract_value(thiscoupler->DmaxB, "DmaxB", line, "");
	extract_value(thiscoupler->FmaxB, "FmaxB", line, "");
	extract_value(thiscoupler->beta, "beta", line, "");
	// the sets are numbers in the definition, the sums of the numbers of the kinds of coupling
	auto automaticflag{thiscoupler->AutomaticCouplingFlag.Bits()};
	extract_value(automaticflag, "AutomaticFlag", line, "");
	thiscoupler->AutomaticCouplingFlag = CouplingSet::FromBits(automaticflag);
	auto allowedflag{thiscoupler->AllowedFlag.Bits()};
	extract_value(allowedflag, "AllowedFlag", line, "");
	// negative: the coupling is permanent besides
	thiscoupler->AllowedFlag = allowedflag < 0 ? CouplingSet::FromBits(-allowedflag) | Coupling::permanent : CouplingSet::FromBits(allowedflag);
	auto powercoupling{thiscoupler->PowerCoupling.Bits()};
	extract_value(powercoupling, "PowerCoupling", line, "");
	thiscoupler->PowerCoupling = CouplingSet::FromBits(powercoupling);
	auto powerflag{thiscoupler->PowerFlag.Bits()};
	extract_value(powerflag, "PowerFlag", line, "");
	thiscoupler->PowerFlag = CouplingSet::FromBits(powerflag);
	extract_value(thiscoupler->control_type, "ControlType", line, "");

	if (thiscoupler->CouplerType != TCouplerType::NoCoupler && thiscoupler->CouplerType != TCouplerType::Bare && thiscoupler->CouplerType != TCouplerType::Articulated)
	{

		thiscoupler->SpringKC *= 1000;
		thiscoupler->FmaxC *= 1000;
		thiscoupler->SpringKB *= 1000;
		thiscoupler->FmaxB *= 1000;
	}
	else if (thiscoupler->CouplerType == TCouplerType::Bare)
	{

		thiscoupler->SpringKC = 50.0 * Mass + Ftmax / 0.05;
		thiscoupler->DmaxC = 0.05;
		thiscoupler->FmaxC = 100.0 * Mass + 2 * Ftmax;
		thiscoupler->SpringKB = 60.0 * Mass + Ftmax / 0.05;
		thiscoupler->DmaxB = 0.05;
		thiscoupler->FmaxB = 50.0 * Mass + 2.0 * Ftmax;
		thiscoupler->beta = 0.3;
	}
	else if (thiscoupler->CouplerType == TCouplerType::Articulated)
	{
		/*
		        coupler->SpringKC = 60.0 * Mass + 1000;
		        coupler->DmaxC = 0.05;
		        coupler->FmaxC = 20000000.0 + 2.0 * Ftmax;
		        coupler->SpringKB = 70.0 * Mass + 1000;
		        coupler->DmaxB = 0.05;
		        coupler->FmaxB = 4000000.0 + 2.0 * Ftmax;
		        coupler->beta = 0.55;
		*/
		thiscoupler->SpringKC = 4500 * 1000;
		thiscoupler->FmaxC = 850 * 1000;
		thiscoupler->SpringKB = 9200 * 1000;
		thiscoupler->FmaxB = 320 * 1000;
		thiscoupler->beta = 0.55;
		// the spring force is taken up to Dmax only, past it the vehicles collide (CollisionDetect()); so the joint
		// carries the full FmaxC either way before it gives. with 0.05 m it carried 225 kN, short of what a loaded
		// articulated wagon in a freight train takes, which then collided at every step (eu07/maszyna#553)
		thiscoupler->DmaxC = std::max(0.05, thiscoupler->FmaxC / thiscoupler->SpringKC);
		thiscoupler->DmaxB = std::max(0.05, thiscoupler->FmaxC / thiscoupler->SpringKB);
	}

	// the buffers of a category of EN 15551 in place of the springs given
	auto const categoryname{extract_value("BufferCategory", line)};
	auto const category{categoryname.empty() ? std::nullopt : collision::buffer_category_of(categoryname.front())};
	auto const buffered{thiscoupler->CouplerType != TCouplerType::NoCoupler && thiscoupler->CouplerType != TCouplerType::Bare && thiscoupler->CouplerType != TCouplerType::Articulated};
	if (category && buffered)
	{
		auto const linear{collision::buffer_curve(*category, 0.0)};
		thiscoupler->SpringKB = linear.stiffness;
		thiscoupler->DmaxB = linear.travel;
	}
	thiscoupler->physics = coupler_physics_parameters(*thiscoupler, line);
	if (category && buffered)
	{
		thiscoupler->physics.buffers = collision::buffer_curve(*category, thiscoupler->physics.buffers.progression);
	}
	thiscoupler->collision = collision_definition(line);
	thiscoupler->collision.buffers = category;

	if (Index == 0)
	{
		// 0 indicates single entry for both couplers
		Couplers[1] = Couplers[0];
	}
	// each coupling as made is a little stronger than its rated force, a different one at each end
	for (int end = end::front; end <= end::rear; ++end)
	{
		if (Index == 0 || end == (Index == 2 ? end::rear : end::front))
		{
			scale_strength(Couplers[end].physics.link, coupler_physics::strength_scatter(Name, end));
		}
	}
}

void TMoverParameters::LoadFIZ_TurboPos(std::string const &Input)
{

	extract_value(TurboTest, "TurboPos", Input, "");
}

void TMoverParameters::LoadFIZ_Cntrl(std::string const &line)
{

	LoadFIZ_CntrlBrakes(line);
	LoadFIZ_CntrlControllers(line);
	LoadFIZ_CntrlStarts(line);
}

// Cntrl.: the brake system, the handles, the brake delays and the local and manual brakes
void TMoverParameters::LoadFIZ_CntrlBrakes(std::string const &line)
{
	{
		using enum TBrakeSystem;
		std::map<std::string, TBrakeSystem, std::less<>> brakesystems{{"Pneumatic", Pneumatic}, {"ElectroPneumatic", ElectroPneumatic}};
		auto lookup = brakesystems.find(extract_value("BrakeSystem", line));
		BrakeSystem = lookup != brakesystems.end() ? lookup->second : Individual;
	}
	if (BrakeSystem != TBrakeSystem::Individual)
	{
		LoadFIZ_CntrlTrainBrake(line);
	} // brakesystem != individual

	// localbrake
	{
		using enum TLocalBrake;
		std::map<std::string, TLocalBrake, std::less<>> localbrakes{{"ManualBrake", ManualBrake}, {"PneumaticBrake", PneumaticBrake}, {"HydraulicBrake", HydraulicBrake}};
		auto lookup = localbrakes.find(extract_value("LocalBrake", line));
		LocalBrake = lookup != localbrakes.end() ? lookup->second : NoBrake;
	}
	// mbrake
	extract_value(MBrake, "ManualBrake", line, "");
}

// Cntrl.: of a vehicle with a train brake, its positions, delays, modes, handles, load weighing and anti-slip brake
void TMoverParameters::LoadFIZ_CntrlTrainBrake(std::string const &line)
{
	using enum TBrakeHandle;

	extract_value(BrakeCtrlPosNo, "BCPN", line, "");
	for (int idx = 0; idx < 4; ++idx)
	{

		extract_value(BrakeDelay[idx], std::format("BDelay{}", idx + 1), line, "");
	}
	// brakedelays, brakedelayflag
	{
		std::map<std::string, int, std::less<>> brakedelays{{"GPR", bdelay_G + bdelay_P + bdelay_R},
		                                       {"PR", bdelay_P + bdelay_R},
		                                       {"GP", bdelay_G + bdelay_P},
		                                       {"R", bdelay_R},
		                                       {"P", bdelay_P},
		                                       {"G", bdelay_G},
		                                       {"GPR+Mg", bdelay_G + bdelay_P + bdelay_R + bdelay_M},
		                                       {"PR+Mg", bdelay_P + bdelay_R + bdelay_M}};
		std::map<std::string, int, std::less<>> brakedelayflags{{"R", bdelay_R}, {"P", bdelay_P}, {"G", bdelay_G}};
		std::string brakedelay;
		extract_value(brakedelay, "BrakeDelays", line, "");
		auto lookup = brakedelays.find(brakedelay);
		BrakeDelays = lookup != brakedelays.end() ? lookup->second : 0;
		lookup = brakedelayflags.find(brakedelay);
		BrakeDelayFlag = lookup != brakedelayflags.end() ? lookup->second : 0;
	}
	// brakeopmode
	{
		std::map<std::string, int, std::less<>> brakeopmodes{{"PN", bom_PS + bom_PN}, {"PNEP", bom_PS + bom_PN + bom_EP}, {"PNEPMED", bom_PS + bom_PN + bom_EP + bom_MED}};
		auto lookup = brakeopmodes.find(extract_value("BrakeOpModes", line));
		BrakeOpModes = lookup != brakeopmodes.end() ? lookup->second : 0;
	}
	// brakehandle
	{
		std::map<std::string, TBrakeHandle, std::less<>> brakehandles{
		    {"FV4a", FV4a},       {"test", testH},    {"D2", D2},      {"MHZ_EN57", MHZ_EN57}, {"MHZ_K5P", MHZ_K5P},
		    {"MHZ_K8P", MHZ_K8P}, {"MHZ_6P", MHZ_6P}, {"M394", M394},  {"Knorr", Knorr},       {"Westinghouse", West},
		    {"FVel6", FVel6},     {"FVE408", FVE408}, {"St113", St113}};
		auto lookup = brakehandles.find(extract_value("BrakeHandle", line));
		BrakeHandle = lookup != brakehandles.end() ? lookup->second : NoHandle;
	}
	extract_value(Handle_AutomaticOverload, "HAO", line, "");
	extract_value(Handle_ManualOverload, "HMO", line, "");
	extract_value(Handle_GenericDoubleParameter1, "HGDP1", line, "");
	extract_value(Handle_GenericDoubleParameter2, "HGDP2", line, "");
	extract_value(Handle_OverloadMaxPressure, "OMP", line, "");
	extract_value(Handle_OverloadPressureDecrease, "OPD", line, "");
	// brakelochandle
	{
		std::map<std::string, TBrakeHandle, std::less<>> locbrakehandles{{"FD1", FD1}, {"Knorr", Knorr}, {"Westinghouse", West}};
		auto lookup = locbrakehandles.find(extract_value("LocBrakeHandle", line));
		BrakeLocHandle = lookup != locbrakehandles.end() ? lookup->second : NoHandle;
	}

	// mbpm
	if (true == extract_value(MBPM, "MaxBPMass", line, ""))
	{
		// NOTE: only convert the value from tons to kilograms if the entry is present in the config file
		MBPM *= 1000.0;
	}

	// asbtype
	LoadFIZ_CntrlAntiSlipBrake(ToLower(extract_value("ASB", line)));
}

// Cntrl.: the kind of the anti-slip brake, by hand or automatic only for a vehicle with a brake handle
void TMoverParameters::LoadFIZ_CntrlAntiSlipBrake(std::string_view const asb)
{
	if (asb == "yes")
	{
		ASBType = 128;
	}
	else if (BrakeCtrlPosNo > 0 && asb == "manual")
	{
		ASBType = 1;
	}
	else if (BrakeCtrlPosNo > 0 && asb == "automatic")
	{
		ASBType = 2;
	}
}

// Cntrl.: the controllers, their delays, the cab and the switches of the driver's desk
void TMoverParameters::LoadFIZ_CntrlControllers(std::string const &line)
{
	// maksymalna predkosc dostepna na tarczce predkosciomierza
	extract_value(maxTachoSpeed, "MaxTachoSpeed", line, "");

	// horns
	{
		using enum horn_supply;
		std::map<std::string, horn_supply, std::less<>> const hornsupplies{{"Pneumatic", air}, {"Electric", electric}};
		auto lookup = hornsupplies.find(extract_value("HornType", line));
		HornSupply = lookup != hornsupplies.end() ? lookup->second : air;
		lookup = hornsupplies.find(extract_value("Horn3Type", line));
		Horn3Supply = lookup != hornsupplies.end() ? lookup->second : HornSupply;
	}

	// dynamicbrake
	{
		std::map<std::string, int, std::less<>> dynamicbrakes{{"Passive", dbrake_passive}, {"Switch", dbrake_switch}, {"Reversal", dbrake_reversal}, {"Automatic", dbrake_automatic}};
		auto lookup = dynamicbrakes.find(extract_value("DynamicBrake", line));
		DynamicBrakeType = lookup != dynamicbrakes.end() ? lookup->second : dbrake_none;
		extract_value(DynamicBrakeAmpmeters, "DBAM", line, "");
	}

	// split ED/pneumatic brake control: when true the local brake operates only on the
	// pneumatic locomotive cylinder while electrodynamic braking is commanded by a
	// separate "dynamicbrakectrl" lever (or the negative range of jointctrl).
	extract_value(SplitEDPneumaticBrake, "SplitEDPneumaticBrake", line, "");
	// number of stepped positions for the dedicated ED brake controller (default 10)
	DynamicBrakeCtrlPosNo = 10;
	extract_value(DynamicBrakeCtrlPosNo, "DBPN", line, "");
	if (DynamicBrakeCtrlPosNo < 1)
	{
		DynamicBrakeCtrlPosNo = 10;
	}

	extract_value(MainCtrlPosNo, "MCPN", line, "");
	extract_value(ScndCtrlPosNo, "SCPN", line, "");
	extract_value(ScndInMain, "SCIM", line, "");
	extract_value(MainCtrlMaxDirChangePos, "DirChangeMaxPos", line, "");

	if (auto const autorelay{ToLower(extract_value("AutoRelay", line))}; autorelay == "optional")
	{
		AutoRelayType = 2;
	}
	else if (autorelay == "yes")
	{
		AutoRelayType = 1;
	}
	else
	{
		AutoRelayType = 0;
	}
	// a key of its own rather than a new AutoRelay value: an exe that doesn't know it reads the line as before, and the value
	// stays Optional, which every check of the switch looks for
	extract_value(FieldShuntSwitch, "FieldShuntSwitch", line, "");

	extract_value(CoupledCtrl, "CoupledCtrl", line, "");
	extract_value(HasCamshaft, "Camshaft", line, "");
	extract_value(EIMCtrlType, "EIMCtrlType", line, "");
	EIMCtrlType = std::clamp(EIMCtrlType, 0, 3);
	extract_value(LocHandleTimeTraxx, "LocalBrakeTraxx", line, "");
	extract_value(EIMCtrlAdditionalZeros, "EIMCtrlAddZeros", line, "");
	extract_value(EIMCtrlEmergency, "EIMCtrlEmergency", line, "");
	// Elf (EIMCtrlType 2): an emergency brake position below the controller's position 0, which shifts the other positions up by one
	extract_value(EIMCtrlMainEmergency, "EIMCtrlMainEmergency", line, "");
	if (auto const elfpositions{4 + EIMCtrlEmergencyPositions()}; EIMCtrlEmergencyPositions() > 0 && MainCtrlPosNo < elfpositions)
	{
		// the old positions in the file: without the fix-up the controller would have no power position, and the
		// reverser would stop working at neutral (DirChangeMaxPos read before, in the old positions)
		MainCtrlMaxDirChangePos += EIMCtrlEmergencyPositions();
		ErrorLog(std::format("Bad fiz: EIMCtrlMainEmergency needs MCPN={} in {}, DirChangeMaxPos taken as {}", elfpositions, TypeName, MainCtrlMaxDirChangePos));
		MainCtrlPosNo = elfpositions;
	}

	extract_value(ScndS, "ScndS", line, ""); // brak pozycji rownoleglej przy niskiej nastawie PSR
	extract_value(ScndCtrlResistiveLock, "ScndCtrlResistiveLock", line, ""); // bocznikowanie tylko na pozycjach bezoporowych

	extract_value(InitialCtrlDelay, "IniCDelay", line, "");
	extract_value(CtrlDelay, "SCDelay", line, "");
	CtrlDownDelay = CtrlDelay; // hunter-101012: jesli nie ma SCDDelay;
	extract_value(CtrlDownDelay, "SCDDelay", line, "");

	// hunter-111012: dla siodemek 303E
	FastSerialCircuit = ToLower(extract_value("FSCircuit", line)) == "yes" ? 1 : 0;
	extract_value(BackwardsBranchesAllowed, "BackwardsBranchesAllowed", line, "");

	extract_value(AutomaticCabActivation, "AutomaticCabActivation", line, "");
	extract_value(InactiveCabFlag, "InactiveCabFlag", line, "");

	extract_value(StopBrakeDecc, "SBD", line, "");
	extract_value(StopBrakeReleaseDelay, "SBDReleaseDelay", line, ""); // the holding brake waits for the power demand this long, s
	extract_value(ReleaseParkingBySpringBrake, "ReleaseParkingBySpringBrake", line, "");
	extract_value(ReleaseParkingBySpringBrakeWhenDoorIsOpen, "ReleaseParkingBySpringBrakeWhenDoorIsOpen", line, "");
	extract_value(SpringBrakeCutsOffDrive, "SpringBrakeCutsOffDrive", line, "");
	extract_value(SpringBrakeDriveEmergencyVel, "SpringBrakeDriveEmergencyVel", line, "");

	extract_value(HideDirStatusWhenMoving, "HideDirStatusWhenMoving", line, "");
	extract_value(HideDirStatusSpeed, "HideDirStatusSpeed", line, "");
	extract_value(isDoubleClickForMeasureNeeded, "DCMB", line, "");
	extract_value(DistanceCounterDoublePressPeriod, "DCDPP", line, "");

	extract_value(isBatteryButtonImpulse, "IBTB", line, "");
	extract_value(shouldHoldBatteryButton, "SBBBH", line, "");
	extract_value(BatteryButtonHoldTime, "BBHT", line, "");
}

// Cntrl.: how the devices start (MainStart, BatteryStart, ...) and their relays
void TMoverParameters::LoadFIZ_CntrlStarts(std::string const &line)
{
	using enum start_t;
	std::map<std::string, start_t, std::less<>> starts{{"Disabled", disabled}, {"Manual", manual},       {"Automatic", automatic}, {"Mixed", manualwithautofallback},
	                                      {"Battery", battery},   {"Converter", converter}, {"Direction", direction}};

	// main circuit
	extract_value(MainsInitTime, "MainInitTime", line, "");
	{
		auto lookup = starts.find(extract_value("MainStart", line));
		MainsStart = lookup != starts.end() ? lookup->second : manual;
	}
	extract_value(PantographsRequireMains, "PantRequiresMain", line, "");
	// the brake pipe pressure of an emergency brake application, which opens the line breaker under power
	extract_value(EmergencyBrakeMainOffPressure, "EmergencyBrakeMainOff", line, "");
	EmergencyBrakeMainOffPressure = std::max(0.0, EmergencyBrakeMainOffPressure);
	// battery
	{
		auto lookup = starts.find(extract_value("BatteryStart", line));
		BatteryStart = lookup != starts.end() ? lookup->second : manual;
	}
	// converter
	{
		auto lookup = starts.find(extract_value("ConverterStart", line));
		ConverterStart = lookup != starts.end() ? lookup->second : manual;
	}
	extract_value(ConverterStartDelay, "ConverterStartDelay", line, "");
	extract_value(ConverterOverloadRelayOffWhenMainIsOff, "ConverterOverloadWhenMainIsOff", line, TrainType == train_type::emu ? "yes" : "no");
	// compressor
	{
		auto lookup = starts.find(extract_value("CompressorStart", line));
		CompressorStart = lookup != starts.end() ? lookup->second : manual;
	}
	// pantograph compressor
	{
		auto lookup = starts.find(extract_value("PantCompressorStart", line));
		PantographCompressorStart = lookup != starts.end() ? lookup->second : manual;
	}
	// pantograph compressor valve
	PantAutoValve = TrainType == train_type::emu; // legacy code behaviour, automatic valve was initially installed in all EMUs
	extract_value(PantAutoValve, "PantAutoValve", line, "");
	// pantographs valve
	{
		auto lookup = starts.find(extract_value("PantEPValveStart", line));
		PantsValve.start_type = lookup != starts.end() ? lookup->second : automatic; // legacy code behaviour, there was no pantographs valve
		extract_value(PantsValve.spring, "PantEPValveSpring", line, "");
	}
	// pantograph valve configuration
	{
		auto lookup = starts.find(extract_value("PantValveStart", line));
		auto valvestarttype = lookup != starts.end() ? lookup->second : manual;
		auto valvespring{true};
		extract_value(valvespring, "PantValveSpring", line, "");
		auto valvesolenoid{true};
		extract_value(valvesolenoid, "PantValveSolenoid", line, "");
		for (auto &pantograph : Pantographs)
		{
			pantograph.valve.spring = valvespring;
			pantograph.valve.solenoid = valvesolenoid;
			pantograph.valve.start_type = valvestarttype;
		}
	}
	// fuel pump
	{
		auto lookup = starts.find(extract_value("FuelStart", line));
		FuelPump.start_type = lookup != starts.end() ? lookup->second : manual;
	}
	// oil pump
	{
		auto lookup = starts.find(extract_value("OilStart", line));
		OilPump.start_type = lookup != starts.end() ? lookup->second : manual;
	}
	// genset of a dual-mode vehicle: Manual (the default) needs GensetSwitch, Automatic runs whenever the selector is on it
	{
		auto lookup = starts.find(extract_value("GensetStart", line));
		Genset.start_type = lookup != starts.end() ? lookup->second : manual;
	}
	// water pump
	{
		auto lookup = starts.find(extract_value("WaterStart", line));
		WaterPump.start_type = lookup != starts.end() ? lookup->second : manual;
	}
	// traction motor fans
	{
		auto lookup = starts.find(extract_value("MotorBlowersStart", line));
		MotorBlowers[end::rear].start_type = lookup != starts.end() ? lookup->second : manual;
		MotorBlowers[end::front].start_type = MotorBlowers[end::rear].start_type;
	}
	// what else the line contactors need to close
	{
		std::map<std::string, line_contactors_interlock, std::less<>> const interlocks{{"Converter", line_contactors_interlock::converter},
		                                                                             {"MotorBlowers", line_contactors_interlock::motor_blowers}};
		auto lookup = interlocks.find(extract_value("LineContactorsInterlock", line));
		LineContactorsInterlock = lookup != interlocks.end() ? lookup->second : line_contactors_interlock::none;
	}
	extract_value(LouvresOpenPressure, "LouvresOpenPressure", line, "");
	// compartment lights
	{
		auto lookup = starts.find(extract_value("CompartmentLightsStart", line));
		CompartmentLights.start_type = lookup != starts.end() ? lookup->second : automatic; // legacy behaviour
	}
	// ground relay
	{
		auto lookup = starts.find(extract_value("GroundRelayStart", line));
		auto const defaultstart{TrainType == train_type::emu ? automatic : manual};
		GroundRelayStart = lookup != starts.end() ? lookup->second : defaultstart;
	}
	// converter overload relay
	{
		auto lookup = starts.find(extract_value("ConverterOverloadRelayStart", line));
		// the relay of a multiple unit activates when the converter is switched off
		auto const defaultstart{TrainType == train_type::emu ? converter : manual};
		ConverterOverloadRelayStart = lookup != starts.end() ? lookup->second : defaultstart;
	}
}

void TMoverParameters::LoadFIZ_Blending(std::string const &line)
{

	extract_value(MED_Vmax, "MED_Vmax", line, std::to_string(Vmax));
	extract_value(MED_Vmin, "MED_Vmin", line, "0");
	extract_value(MED_Vref, "MED_Vref", line, std::to_string(Vmax));
	extract_value(MED_amax, "MED_amax", line, "9.81");
	extract_value(MED_EPVC, "MED_EPVC", line, "");
	extract_value(MED_Ncor, "MED_Ncor", line, "");
	extract_value(MED_MinBrakeReqED, "MED_MinBrakeReqED", line, "");
	extract_value(MED_FrED_factor, "MED_FrEDFactor", line, "");
	extract_value(MED_ED_Delay1, "MED_FirstDelayED", line, "");
	extract_value(MED_ED_Delay2, "MED_ScndDelayED", line, "");
}

void TMoverParameters::LoadFIZ_DCEMUED(std::string const &line)
{

	extract_value(DCEMUED_CC, "CouplerCheck", line, "0");
	extract_value(DCEMUED_EP_max_Vel, "EP_max_Vel", line, "0");
	extract_value(DCEMUED_EP_min_Im, "EP_min_Im", line, "0");
	extract_value(DCEMUED_EP_delay, "EP_delay", line, "0");
}

void TMoverParameters::LoadFIZ_SpringBrake(std::string const &line)
{

	double vol{1.0};
	extract_value(vol, "Volume", line, "1");
	if (!SpringBrake.Cylinder)
		SpringBrake.Cylinder = std::make_shared<TReservoir>();
	SpringBrake.Cylinder->CreateCap(vol);
	extract_value(SpringBrake.MaxBrakeForce, "MBF", line, "");
	extract_value(SpringBrake.MaxSetPressure, "MaxSP", line, "");
	extract_value(SpringBrake.ResetPressure, "ResetP", line, "");
	extract_value(SpringBrake.MinForcePressure, "MinFP", line, "");
	extract_value(SpringBrake.PressureOff, "PressOff", line, "");
	extract_value(SpringBrake.PressureOn, "PressOn", line, "");
	extract_value(SpringBrake.ValveOffArea, "ValveOnArea", line, "");
	extract_value(SpringBrake.ValveOnArea, "ValveOffArea", line, "");
	extract_value(SpringBrake.ValvePNBrakeArea, "ValvePNBArea", line, "");
	SpringBrake.PNBrakeConnection = SpringBrake.ValvePNBrakeArea > 0;
	extract_value(SpringBrake.MultiTractionCoupler, "MTC", line, "");
	SpringBrake.ShuttOff = false;
	SpringBrake.Activate = false;
	SpringBrake.IsReady = true;
}

void TMoverParameters::LoadFIZ_Light(std::string const &line)
{

	LightPowerSource.SourceType = LoadFIZ_SourceDecode(extract_value("Light", line));
	LoadFIZ_PowerParamsDecode(LightPowerSource, "L", line);

	AlterLightPowerSource.SourceType = LoadFIZ_SourceDecode(extract_value("AlterLight", line));
	LoadFIZ_PowerParamsDecode(AlterLightPowerSource, "AlterL", line);

	extract_value(NominalVoltage, "Volt", line, "");
	extract_value(BatteryVoltage, "LMaxVoltage", line, "");
	NominalBatteryVoltage = static_cast<float>(BatteryVoltage);
}

void TMoverParameters::LoadFIZ_Clima(std::string const &line)
{

	HeatingPowerSource.SourceType = LoadFIZ_SourceDecode(extract_value("Heating", line));
	LoadFIZ_PowerParamsDecode(HeatingPowerSource, "H", line);
	AlterHeatPowerSource.SourceType = LoadFIZ_SourceDecode(extract_value("AlterHeating", line));
	LoadFIZ_PowerParamsDecode(AlterHeatPowerSource, "AlterH", line);
}

void TMoverParameters::LoadFIZ_Power(std::string const &Line)
{

	EnginePowerSource.SourceType = LoadFIZ_SourceDecode(extract_value("EnginePower", Line));
	LoadFIZ_PowerParamsDecode(EnginePowerSource, "", Line);
	/*
	    if( ( EnginePowerSource.SourceType == TPowerSource::Generator )
	     && ( EnginePowerSource.GeneratorEngine == TEngineType::WheelsDriven ) ) {
	        // perpetuum mobile?
	        ConversionError = 666;
	    }
	*/
	if (Power == 0.0)
	{
		// jeśli nie ma mocy, np. rozrządcze EZT
		EnginePowerSource.SourceType = TPowerSource::NotDefined;
	}
	else
	{
		LoadFIZ_Genset(Line);
	}

	SystemPowerSource.SourceType = LoadFIZ_SourceDecode(extract_value("SystemPower", Line));
	LoadFIZ_PowerParamsDecode(SystemPowerSource, "", Line);
}

// the genset of a dual-mode vehicle (eu07/maszyna#536), on the Power: line; all optional, no genset without GensetPower
void TMoverParameters::LoadFIZ_Genset(std::string const &Line)
{
	extract_value(Genset.power, "GensetPower", Line, "");
	if (false == Genset.fitted())
	{
		return;
	}
	extract_value(Genset.voltage_nominal, "GensetVoltage", Line, "");
	if (Genset.voltage_nominal <= 0.0)
	{
		Genset.voltage_nominal = EnginePowerSource.MaxVoltage > 0.0 ? 0.65 * EnginePowerSource.MaxVoltage : 3000.0;
	}
	extract_value(Genset.revolutions_idle, "GensetIdleRPM", Line, "600");
	extract_value(Genset.revolutions_max, "GensetMaxRPM", Line, "1800");
	Genset.revolutions_max = std::max(Genset.revolutions_max, 1.0) / 60.0;
	Genset.revolutions_idle = std::clamp(Genset.revolutions_idle / 60.0, 1.0 / 60.0, Genset.revolutions_max);
	extract_value(Genset.start_time, "GensetStartTime", Line, "8");
	extract_value(Genset.spinup_time, "GensetSpinUpTime", Line, "3");
	extract_value(Genset.traction_share, "GensetPmaxRatio", Line, "1");
	Genset.start_time = std::max(Genset.start_time, 0.0);
	Genset.spinup_time = std::max(Genset.spinup_time, 0.01);
	Genset.traction_share = std::clamp(Genset.traction_share, 0.0, 1.0);
}

void TMoverParameters::LoadFIZ_SpeedControl(std::string const &Line)
{
	// speed control
	extract_value(SpeedCtrl, "SpeedCtrl", Line, "");
	if (!SpeedCtrl && EngineType == TEngineType::ElectricInductionMotor && ScndCtrlPosNo > 0) // backward compatibility
		SpeedCtrl = true;
	extract_value(SpeedCtrlDelay, "SpeedCtrlDelay", Line, "");
	SpeedCtrlTypeTime = extract_value("SpeedCtrlType", Line) == "Time" ? true : false;
	extract_value(SpeedCtrlAutoTurnOffFlag, "SpeedCtrlATOF", Line, "");

	auto speedpresets = Split(extract_value("SpeedButtons", Line), '|');
	int speed_no = 0;
	for (auto const &speed : speedpresets)
	{
		SpeedCtrlButtons[speed_no++] = std::stod(speed);
		if (speed_no > 9)
			break;
	}
	extract_value(SpeedCtrlUnit.ManualStateOverride, "OverrideManual", Line, "");
	extract_value(SpeedCtrlUnit.BrakeIntervention, "BrakeIntervention", Line, "");
	extract_value(SpeedCtrlUnit.InitialPower, "InitPwr", Line, "");
	extract_value(SpeedCtrlUnit.FullPowerVelocity, "MaxPwrVel", Line, "");
	extract_value(SpeedCtrlUnit.StartVelocity, "StartVel", Line, "");
	extract_value(SpeedCtrlUnit.VelocityStep, "VelStep", Line, "");
	extract_value(SpeedCtrlUnit.PowerStep, "PwrStep", Line, "");
	extract_value(SpeedCtrlUnit.MinPower, "MinPwr", Line, "");
	extract_value(SpeedCtrlUnit.MaxPower, "MaxPwr", Line, "");
	extract_value(SpeedCtrlUnit.MinVelocity, "MinVel", Line, "");
	extract_value(SpeedCtrlUnit.MaxVelocity, "MaxVel", Line, "");
	extract_value(SpeedCtrlUnit.Offset, "Offset", Line, "");
	extract_value(SpeedCtrlUnit.FactorPpos, "kPpos", Line, "");
	extract_value(SpeedCtrlUnit.FactorPneg, "kPneg", Line, "");
	extract_value(SpeedCtrlUnit.FactorIpos, "kIpos", Line, "");
	extract_value(SpeedCtrlUnit.FactorIneg, "kIneg", Line, "");
	extract_value(SpeedCtrlUnit.BrakeInterventionVel, "BrakeIntMaxVel", Line, "");
	extract_value(SpeedCtrlUnit.PowerUpSpeed, "PowerUpSpeed", Line, "");
	extract_value(SpeedCtrlUnit.PowerDownSpeed, "PowerDownSpeed", Line, "");
}

void TMoverParameters::LoadFIZ_Engine(std::string const &Input)
{

	EngineType = LoadFIZ_EngineDecode(extract_value("EngineType", Input));
	extract_value(JointEngineStart, "JointEngineStart", Input, "");

	if (std::string transmission = extract_value("Trans", Input); false == transmission.empty())
	{
		// transmission type. moved here because more than one engine type has this entry
		auto ratios = Split(transmission, ':'); // e.g. 18:79

		if (ratios.size() != 2)
		{
			ErrorLog("Wrong transmition definition: " + transmission);
		}

		Transmision.NToothM = std::atoi(ratios[0].c_str()); // ToothM to pierwszy czyli 18
		Transmision.NToothW = std::atoi(ratios[1].c_str()); // ToothW to drugi parametr czyli 79

		if (Transmision.NToothM > 0)
			Transmision.Ratio = static_cast<double>(Transmision.NToothW) / Transmision.NToothM;
		else
			Transmision.Ratio = 1.0;
		extract_value(Transmision.Efficiency, "TransEff", Input, "");
	}

	switch (EngineType)
	{

	case TEngineType::ElectricSeriesMotor:
	{

		extract_value(NominalVoltage, "Volt", Input, "");
		extract_value(WindingRes, "WindingRes", Input, "");
		if (WindingRes == 0.0)
		{

			WindingRes = 0.01;
		}
		extract_value(nmax, "nmax", Input, "");
		nmax /= 60.0;
		break;
	}
	case TEngineType::WheelsDriven:
	case TEngineType::Dumb:
	{

		extract_value(Ftmax, "Ftmax", Input, "");
		break;
	}
	case TEngineType::DieselEngine:
	{
		LoadFIZ_EngineDiesel(Input);
		break;
	}
	case TEngineType::DieselElectric:
	{ // youBy
		LoadFIZ_EngineDieselElectric(Input);
		break;
	}
	case TEngineType::ElectricInductionMotor:
	{
		LoadFIZ_EngineInductionMotor(Input);
		break;
	}
	default:
	{
		// nothing here
	}
	} // engine type

	// NOTE: elements shared by both diesel engine variants; crude but, eh
	if (EngineType == TEngineType::DieselEngine || EngineType == TEngineType::DieselElectric)
	{
		LoadFIZ_EngineDieselShared(Input);
	}

	// traction motors
	extract_value(MotorBlowers[end::front].speed, "MotorBlowersSpeed", Input, "");
	extract_value(MotorBlowers[end::front].sustain_time, "MotorBlowersSustainTime", Input, "");
	extract_value(MotorBlowers[end::front].min_start_velocity, "MotorBlowersStartVelocity", Input, "");
	MotorBlowers[end::rear] = MotorBlowers[end::front];
	// pressure switch
	extract_value(HasControlPressureSwitch, "PressureSwitch", Input, TrainType != train_type::emu ? "yes" : "no");
	// its brake cylinder half; the brake pipe one stays
	extract_value(PressureSwitchBrakeCylinder, "PressureSwitchBrakeCylinder", Input, "");
}

// Engine: of a diesel engine, its revolutions, gears, hydraulic transmission and retarder
void TMoverParameters::LoadFIZ_EngineDiesel(std::string const &Input)
{

	extract_value(diesel.minRevolutions, "nmin", Input, "");
	diesel.minRevolutions /= 60.0;
	diesel.governorMinRevolutions = diesel.minRevolutions * 0.98;
	extract_value(diesel.hydraulicMinRevolutions, "nmin_hdrive", Input, "");
	diesel.hydraulicMinRevolutions /= 60.0;
	if (diesel.hydraulicMinRevolutions == 0.0)
	{
		diesel.hydraulicMinRevolutions = diesel.minRevolutions;
	}
	extract_value(diesel.hydraulicMinRevolutionsFactor, "nmin_hdrive_factor", Input, "");
	diesel.hydraulicMinRevolutionsFactor /= 60.0;
	extract_value(diesel.retarderRevolutions, "nmin_retarder", Input, "");
	diesel.retarderRevolutions /= 60.0;
	if (diesel.retarderRevolutions == 0.0)
	{
		diesel.retarderRevolutions = diesel.minRevolutions;
	}
	// TODO: unify naming scheme and sort out which diesel engine params are used where and how
	extract_value(nmax, "nmax", Input, "");
	nmax /= 60.0;
	extract_value(diesel.cutoffRevolutions, "nmax_cutoff", Input, "0.0");
	diesel.cutoffRevolutions /= 60.0;
	extract_value(diesel.governorAcceleration, "nreg_acc", Input, "");
	diesel.governorAcceleration /= 60.0;
	extract_value(diesel.shaftInertia, "AIM", Input, "1.0");
	extract_value(diesel.revolutionsDecreaseRate, "RPMDecRate", Input, "");

	extract_value(engageupspeed, "EUS", Input, "0.5");
	extract_value(engagedownspeed, "EDS", Input, "0.9");

	if (true == extract_value(AnPos, "ShuntMode", Input, ""))
	{
		// dodatkowa przekładnia dla SM03 (2Ls150)
		ShuntModeAllow = true;
		ShuntMode = false;
		if (AnPos < 1.0)
		{
			//"rozruch wysoki" ma dawać większą siłę
			AnPos = 1.0 / AnPos; // im większa liczba, tym wolniej jedzie
		}
	}
	extract_value(torqueConverter.present, "IsTC", Input, "");
	if (true == torqueConverter.present)
	{
		extract_value(torqueConverter.maxTorqueRatio, "TC_TMMax", Input, "");
		extract_value(torqueConverter.couplingPoint, "TC_CP", Input, "");
		extract_value(torqueConverter.lockupTorque, "TC_LT", Input, "");
		extract_value(torqueConverter.lockupRate, "TC_LR", Input, "");
		extract_value(torqueConverter.unlockRate, "TC_ULR", Input, "");
		extract_value(torqueConverter.fillRateIncrease, "TC_FRI", Input, "");
		extract_value(torqueConverter.fillRateDecrease, "TC_FRD", Input, "");
		extract_value(torqueConverter.torqueInIn, "TC_TII", Input, "");
		extract_value(torqueConverter.torqueInOut, "TC_TIO", Input, "");
		extract_value(torqueConverter.torqueOutOut, "TC_TOO", Input, "");
		extract_value(torqueConverter.lockupSpeed, "TC_LS", Input, "");
		extract_value(torqueConverter.unlockSpeed, "TC_ULS", Input, "");
		extract_value(diesel.converterReleaseVelocity, "MaxVelANS", Input, "");

		extract_value(retarder.present, "IsRetarder", Input, "");
		if (true == retarder.present)
		{
			extract_value(retarder.placement, "R_Place", Input, "");
			extract_value(retarder.torqueInIn, "R_TII", Input, "");
			extract_value(retarder.maxTorque, "R_MT", Input, "");
			extract_value(retarder.maxPower, "R_MP", Input, "");
			extract_value(retarder.fillRateIncrease, "R_FRI", Input, "");
			extract_value(retarder.fillRateDecrease, "R_FRD", Input, "");
			extract_value(retarder.minVelocity, "R_MinVel", Input, "");
			extract_value(retarder.engageVelocity, "R_EngageVel", Input, "");
			extract_value(retarder.hasClutch, "R_IsClutch", Input, "");
			extract_value(retarder.clutchSpeed, "R_ClutchSpeed", Input, "");
			extract_value(retarder.withIndividual, "R_WithIndividual", Input, "");
		}
	}
}

// Engine: of a diesel-electric
void TMoverParameters::LoadFIZ_EngineDieselElectric(std::string const &Input)
{

	extract_value(Ftmax, "Ftmax", Input, "");
	Flat = extract_value("Flat", Input) == "1";
	extract_value(Vhyp, "Vhyp", Input, "");
	Vhyp /= 3.6;
	extract_value(Vadd, "Vadd", Input, "");
	Vadd /= 3.6;
	extract_value(PowerCorRatio, "Cr", Input, "");
	extract_value(RelayType, "RelayType", Input, "");
	if (extract_value("ShuntMode", Input) == "1")
	{

		ShuntModeAllow = true;
		ShuntMode = false;
		AnPos = 0.0;
		ImaxHi = 2;
		ImaxLo = 1;
	}
	extract_value(EngineHeatingRPM, "HeatingRPM", Input, "");
	extract_value(diesel.shaftInertia, "AIM", Input, "1.25");
	extract_value(diesel.revolutionsDecreaseRate, "RPMDecRate", Input, "");
	LoadFIZ_DieselElectricDynamicBrake(Input);
}

// Engine: of an induction motor drive
void TMoverParameters::LoadFIZ_EngineInductionMotor(std::string const &Input)
{

	RVentnmax = 1.0;
	extract_value(NominalVoltage, "Volt", Input, "");

	extract_value(inductionMotor.config[eimc_s_dfic], "dfic", Input, "");
	extract_value(inductionMotor.config[eimc_s_dfmax], "dfmax", Input, "");
	extract_value(inductionMotor.config[eimc_s_p], "p", Input, "");
	extract_value(inductionMotor.config[eimc_s_cfu], "cfu", Input, "");
	extract_value(inductionMotor.config[eimc_s_cim], "cim", Input, "");
	extract_value(inductionMotor.config[eimc_s_icif], "icif", Input, "");
	extract_value(inductionMotor.config[eimc_f_Uzmax], "Uzmax", Input, "");
	extract_value(inductionMotor.config[eimc_f_Uzh], "Uzh", Input, "");
	extract_value(inductionMotor.config[eimc_f_DU], "DU", Input, "");
	extract_value(inductionMotor.config[eimc_f_I0], "I0", Input, "");
	extract_value(inductionMotor.config[eimc_f_cfu], "fcfu", Input, "");
	if (false == extract_value(inductionMotor.config[eimc_f_cfuH], "fcfuH", Input, ""))
	{
		// without its own figure the dynamic brake takes the one of the traction
		inductionMotor.config[eimc_f_cfuH] = inductionMotor.config[eimc_f_cfu];
	}
	extract_value(inductionMotor.config[eimc_p_F0], "F0", Input, "");
	extract_value(inductionMotor.config[eimc_p_a1], "a1", Input, "");
	extract_value(inductionMotor.config[eimc_p_Pmax], "Pmax", Input, "");
	extract_value(inductionMotor.config[eimc_p_Fh], "Fh", Input, "");
	extract_value(inductionMotor.config[eimc_p_Ph], "Ph", Input, "");
	extract_value(inductionMotor.config[eimc_p_Vh0], "Vh0", Input, "");
	extract_value(inductionMotor.config[eimc_p_Vh1], "Vh1", Input, "");
	extract_value(inductionMotor.config[eimc_p_Imax], "Imax", Input, "");
	extract_value(inductionMotor.config[eimc_p_abed], "abed", Input, "");
	extract_value(inductionMotor.config[eimc_p_eped], "edep", Input, "");
	extract_value(inductionMotor.logForce, "eimclf", Input, "");
	extract_value(inductionMotor.inverterCount, "InvNo", Input, "");
	extract_value(inductionMotor.inverterControlCouplings, "InvCtrCplFlag", Input, "");
	extract_value(Imaxrpc, "Imaxrpc", Input, "");
	extract_value(BRVto, "BRVto", Input, "");
	extract_value(Flat, "Flat", Input, "");

	if (inductionMotor.config[eimc_p_Pmax] > 0 && Power > 0 && inductionMotor.inverterCount == 0)
	{
		inductionMotor.inverterCount = 1;
	}
	inductionMotor.inverters.resize(inductionMotor.inverterCount);
	/*for (int i = 0; i > inductionMotor.inverterCount; i++)
	{
	    Inverter x;
	    inductionMotor.inverters.emplace_back(x);
	}*/
}

// Engine: what both kinds of diesels have, their heat among it
void TMoverParameters::LoadFIZ_EngineDieselShared(std::string const &Input)
{
	// oil pump
	extract_value(OilPump.pressure_minimum, "OilMinPressure", Input, "");
	extract_value(OilPump.pressure_maximum, "OilMaxPressure", Input, "");
	// the time the starter turns the engine before it fires and smokes (eu07/maszyna#523)
	extract_value(diesel.smokeDelay, "SmokeDelay", Input, "0");
	diesel.smokeDelay = std::max(0.0, diesel.smokeDelay);
	// engine cooling factore
	extract_value(diesel.heat.kw, "HeatKW", Input, "");
	extract_value(diesel.heat.kv, "HeatKV", Input, "");
	extract_value(diesel.heat.kfe, "HeatKFE", Input, "");
	extract_value(diesel.heat.kfs, "HeatKFS", Input, "");
	extract_value(diesel.heat.kfo, "HeatKFO", Input, "");
	extract_value(diesel.heat.kfo2, "HeatKFO2", Input, "");
	// engine cooling systems
	extract_value(diesel.heat.water.config.temp_min, "WaterMinTemperature", Input, "");
	extract_value(diesel.heat.water.config.temp_max, "WaterMaxTemperature", Input, "");
	extract_value(diesel.heat.water.config.temp_flow, "WaterFlowTemperature", Input, "");
	extract_value(diesel.heat.water.config.temp_cooling, "WaterCoolingTemperature", Input, "");
	extract_value(diesel.heat.water.config.shutters, "WaterShutters", Input, "");
	extract_value(diesel.heat.auxiliary_water_circuit, "WaterAuxCircuit", Input, "");
	extract_value(diesel.heat.water_aux.config.temp_min, "WaterAuxMinTemperature", Input, "");
	extract_value(diesel.heat.water_aux.config.temp_max, "WaterAuxMaxTemperature", Input, "");
	extract_value(diesel.heat.water_aux.config.temp_cooling, "WaterAuxCoolingTemperature", Input, "");
	extract_value(diesel.heat.water_aux.config.shutters, "WaterAuxShutters", Input, "");
	extract_value(diesel.heat.oil.config.temp_min, "OilMinTemperature", Input, "");
	extract_value(diesel.heat.oil.config.temp_max, "OilMaxTemperature", Input, "");
	extract_value(diesel.heat.engine_max_temp, "EngineMaxTemperature", Input, "");
	extract_value(diesel.heat.fan_speed, "WaterCoolingFanSpeed", Input, "");
	// water heater
	extract_value(WaterHeater.config.temp_min, "HeaterMinTemperature", Input, "");
	extract_value(WaterHeater.config.temp_max, "HeaterMaxTemperature", Input, "");
	float pf{1235.f};
	extract_value(pf, "NominalCoolingPower", Input, "1235");
	diesel.heat.powerfactor = 1235 / pf;
}

void TMoverParameters::LoadFIZ_Switches(std::string const &Input)
{

	extract_value(PantSwitchType, "Pantograph", Input, "");
	extract_value(ConvSwitchType, "Converter", Input, "");
	extract_value(StLinSwitchType, "MotorConnectors", Input, "");
	HeatingSwitchImpulse = ToLower(extract_value("TrainHeating", Input)) == "impulse";
	// because people can't make up their minds whether it's "impulse" or "Impulse"...
	PantSwitchType = ToLower(PantSwitchType);
	ConvSwitchType = ToLower(ConvSwitchType);
	StLinSwitchType = ToLower(StLinSwitchType);
	// universal reset buttons assignments
	extract_value(UniversalResetButtonFlag[0], "RelayResetButton1", Input, "");
	extract_value(UniversalResetButtonFlag[1], "RelayResetButton2", Input, "");
	extract_value(UniversalResetButtonFlag[2], "RelayResetButton3", Input, "");
	extract_value(enableModernDimmer, "ModernDimmer", Input, "");
	// pantograph presets
	{
		auto &presets{PantsPreset.first};
		extract_value(presets, "PantographPresets", Input, "0|1|3|2");
		std::erase(presets, '|');
	}
}

void TMoverParameters::LoadFIZ_MotorParamTable(std::string const &Input)
{

	switch (EngineType)
	{

	case TEngineType::DieselEngine:
	{

		extract_value(diesel.fullEngageMinVelocity, "minVelfullengage", Input, "");
		extract_value(diesel.clutchDiameter, "engageDia", Input, "");
		extract_value(diesel.clutchMaxForce, "engageMaxForce", Input, "");
		extract_value(diesel.clutchFriction, "engagefriction", Input, "");
		break;
	}
	default:
	{
		// nothing here
	}
	}
}

void TMoverParameters::LoadFIZ_Circuit(std::string const &Input)
{

	extract_value(CircuitRes, "CircuitRes", Input, "");
	extract_value(IminLo, "IminLo", Input, "");
	extract_value(IminHi, "IminHi", Input, "");
	extract_value(ImaxLo, "ImaxLo", Input, "");
	extract_value(ImaxHi, "ImaxHi", Input, "");
	Imin = IminLo;
	Imax = ImaxLo;
	extract_value(TUHEX_Sum, "TUHEX_Sum", Input, "");
	extract_value(TUHEX_Diff, "TUHEX_Diff", Input, "");
	extract_value(TUHEX_MaxIw, "TUHEX_MaxIw", Input, "");
	extract_value(TUHEX_MinIw, "TUHEX_MinIw", Input, "");
	extract_value(TUHEX_Sum1, "TUHEX_Sum1", Input, "");
	extract_value(TUHEX_Sum2, "TUHEX_Sum2", Input, "");
	extract_value(TUHEX_Sum3, "TUHEX_Sum3", Input, "");
	extract_value(TUHEX_Stages, "TUHEX_Stages", Input, "0");
}

void TMoverParameters::LoadFIZ_AI(std::string const &Input)
{

	extract_value(AIHintPantstate, "Pantstate", Input, "");
	extract_value(AIHintShuntPantstate, "ShuntPantstate", Input, "");
	extract_value(AIHintPantUpIfIdle, "IdlePantUp", Input, "");
	extract_value(AIHintLocalBrakeAccFactor, "LocalBrakeAccFactor", Input, "");
}

void TMoverParameters::LoadFIZ_RList(std::string const &Input)
{

	extract_value(RlistSize, "Size", Input, "");

	if (auto const venttype{ToLower(extract_value("RVent", Input))}; venttype == "automatic")
	{

		RVentType = 2;
	}
	else
	{

		RVentType = venttype == "yes" ? 1 : 0;
	}

	if (RVentType > 0)
	{

		extract_value(RVentnmax, "RVentnmax", Input, "");
		RVentnmax /= 60.0;
		extract_value(RVentCutOff, "RVentCutOff", Input, "");
	}
	extract_value(RVentMinI, "RVentMinI", Input, "");
	extract_value(RVentSpeed, "RVentSpeed", Input, "");
	extract_value(DynamicBrakeRes, "DynBrakeRes", Input, "");
	extract_value(DynamicBrakeRes1, "DynBrakeRes1", Input, "");
	extract_value(DynamicBrakeRes2, "DynBrakeRes2", Input, "");
}

// the dynamic brake of a diesel-electric vehicle, from its Engine section: none without EDBrake
void TMoverParameters::LoadFIZ_DieselElectricDynamicBrake(std::string const &Input)
{
	using enum diesel_electric_dynamic_brake::control;
	std::map<std::string, diesel_electric_dynamic_brake::control, std::less<>> const sources{{"Controller", controller}, {"Lever", lever}};
	auto const lookup{sources.find(extract_value("EDBrake", Input))};
	if (lookup == sources.end())
	{
		return;
	}
	auto &brake{DEDynamicBrake};
	brake.source = lookup->second;
	auto force{0.0}; // [kN]
	auto power{0.0}; // [kW]
	extract_value(force, "EDBrakeFmax", Input, "");
	extract_value(power, "EDBrakePmax", Input, "");
	brake.max_force = force * 1000.0;
	brake.max_power = power * 1000.0;
	extract_value(brake.fade_low, "EDBrakeV0", Input, "");
	extract_value(brake.fade_high, "EDBrakeV1", Input, "");
	extract_value(brake.rate, "EDBrakeRate", Input, "");
	extract_value(brake.blending, "EDBrakeBlending", Input, "");
	if (brake.source == lever)
	{
		// the lever of its own, which the commands of the dynamic brake controller move
		SplitEDPneumaticBrake = true;
	}
}

void TMoverParameters::LoadFIZ_UCList(std::string const &Input)
{

	extract_value(UniCtrlListSize, "Size", Input, "");
	extract_value(UniCtrlIntegratedBrakeCtrl, "IntegratedBrake", Input, "");
	extract_value(UniCtrlIntegratedLocalBrakeCtrl, "IntegratedLocBrake", Input, "");
	extract_value(UniCtrlIntegratedBrakePNCtrl, "IntegratedBrakePN", Input, "");
	extract_value(UniCtrlValStep, "CtrlValStep", Input, "");
	UniCtrlValStep = std::max(UniCtrlValStep, 0.0);
}

void TMoverParameters::LoadFIZ_DList(std::string const &Input)
{

	extract_value(diesel.maxMoment, "Mmax", Input, "");
	extract_value(diesel.maxMomentRevolutions, "nMmax", Input, "");
	extract_value(diesel.maxRevolutionsMoment, "Mnmax", Input, "");
	extract_value(diesel.maxRevolutions, "nmax", Input, "");
	extract_value(diesel.nominalFill, "nominalfill", Input, "");
	extract_value(diesel.standingMoment, "Mstand", Input, "");
	extract_value(diesel.nominalFuelConsumptionRate, "NomFuelConsRate", Input, "");

	if (diesel.maxMomentRevolutions == diesel.maxRevolutions)
	{
		// HACK: guard against cases where nMmax == nmax, leading to division by 0 in momentum calculation
		diesel.maxMomentRevolutions = diesel.maxRevolutions - 1.0 / 60.0;
	}

	// Calculation of fuel consumption coefficient for futher calculation
	double dizel_max_power = diesel.maxRevolutions * (diesel.maxRevolutionsMoment - diesel.standingMoment) * M_PI * 2 * 0.001; // power in kW
	double dizel_max_energy = dizel_max_power; // energy per one hour in kWh is equal to power in kW times 1 h
	double fuel_density = 850; // g/l
	diesel.fuelConsumption = diesel.nominalFuelConsumptionRate * dizel_max_energy / fuel_density / diesel.maxRevolutions;
}

void TMoverParameters::LoadFIZ_FFList(std::string const &Input)
{
	extract_value(FFListSize, "Size", Input, "");
}

void TMoverParameters::LoadFIZ_FFEDList(std::string const &Input)
{
	extract_value(FFEDListSize, "Size", Input, "");
}

void TMoverParameters::LoadFIZ_WiperList(std::string const &Input)
{
	extract_value(WiperListSize, "Size", Input, "");
	extract_value(WiperAngle, "Angle", Input, "");
	extract_value(WiperDefaultPos, "Default", Input, "");
}

void TMoverParameters::LoadFIZ_DimmerList(std::string const &Input)
{
	// extract_value(modernWpierListSize, "Size", Input, "");
	extract_value(modernDimmerCanCycle, "Cycle", Input, "");
	extract_value(modernDimmerDefaultPosition, "DefaultPos", Input, "");
}
void TMoverParameters::LoadFIZ_LightsList(std::string const &Input)
{

	extract_value(LightsPosNo, "Size", Input, "");
	extract_value(LightsWrap, "Wrap", Input, "");
	extract_value(LightsDefPos, "Default", Input, "");
}

void TMoverParameters::LoadFIZ_CompressorList(std::string const &Input)
{

	extract_value(CompressorListPosNo, "Size", Input, "");
	extract_value(CompressorListWrap, "Wrap", Input, "");
	extract_value(CompressorListDefPos, "Default", Input, "");
}

void TMoverParameters::LoadFIZ_PowerParamsDecode(TPowerParameters &Powerparameters, std::string const &Prefix, std::string const &Line)
{

	switch (Powerparameters.SourceType)
	{

	case TPowerSource::NotDefined:
	case TPowerSource::InternalSource:
	{

		Powerparameters.PowerType = LoadFIZ_PowerDecode(extract_value(Prefix + "PowerType", Line));
		break;
	}
	case TPowerSource::Transducer:
	{

		extract_value(Powerparameters.Transducer.InputVoltage, Prefix + "TransducerInputV", Line, "");
		break;
	}
	case TPowerSource::Generator:
	{
		// prime mover for the generator
		auto &generatorparameters{Powerparameters.EngineGenerator};

		if (auto const enginetype{LoadFIZ_EngineDecode(extract_value(Prefix + "GeneratorEngine", Line))}; enginetype == TEngineType::Main)
		{
			generatorparameters.engine_revolutions = &enrot;
		}
		else
		{
			// TODO: for engine types other than Main create requested engine object and link to its revolutions
			generatorparameters.engine_revolutions = nullptr;
			generatorparameters.revolutions = 0;
			generatorparameters.voltage = 0;
		}
		// config
		extract_value(generatorparameters.voltage_min, Prefix + "GeneratorMinVoltage", Line, "0");
		extract_value(generatorparameters.voltage_max, Prefix + "GeneratorMaxVoltage", Line, "0");
		// NOTE: for consistency the fiz file specifies  revolutions per minute
		extract_value(generatorparameters.revolutions_min, Prefix + "GeneratorMinRPM", Line, "0");
		extract_value(generatorparameters.revolutions_max, Prefix + "GeneratorMaxRPM", Line, "0");
		generatorparameters.revolutions_min /= 60;
		generatorparameters.revolutions_max /= 60;
		break;
	}
	case TPowerSource::Accumulator:
	{

		extract_value(Powerparameters.RAccumulator.MaxCapacity, Prefix + "Cap", Line, "");
		Powerparameters.RAccumulator.RechargeSource = LoadFIZ_SourceDecode(extract_value(Prefix + "RS", Line));
		break;
	}
	case TPowerSource::CurrentCollector:
	{

		auto &collectorparameters = Powerparameters.CollectorParameters;

		collectorparameters = TCurrentCollector{0, 0, 0, 0, 0, 0, false, 0, 0, 0, false, 0};

		std::string PantType = "";
		extract_value(PantType, "PantType", Line, "");
		if (PantType == "AKP_4E")
			collectorparameters.PantographType = TPantType::AKP_4E;
		if (PantType.size() >= 3 && PantType.compare(0, 3, "DSA") == 0)
			collectorparameters.PantographType = TPantType::DSAx;
		if (PantType == "EC160" || PantType == "EC200")
			collectorparameters.PantographType = TPantType::EC160_200;
		if (PantType == "WBL85")
			collectorparameters.PantographType = TPantType::WBL85;

		extract_value(collectorparameters.CollectorsNo, "CollectorsNo", Line, "");
		extract_value(collectorparameters.MinH, "MinH", Line, "");
		extract_value(collectorparameters.MaxH, "MaxH", Line, "");
		extract_value(collectorparameters.CSW, "CSW", Line, ""); // szerokość części roboczej
		extract_value(collectorparameters.MaxV, "MaxVoltage", Line, "");
		extract_value(collectorparameters.OVP, "OverVoltProt", Line, ""); // przekaźnik nadnapięciowy
		// napięcie rozłączające WS
		collectorparameters.MinV = 0.5 * collectorparameters.MaxV; // gdyby parametr nie podany
		extract_value(collectorparameters.MinV, "MinV", Line, "");
		// napięcie wymagane do załączenia WS
		collectorparameters.InsetV = 0.6 * collectorparameters.MaxV; // gdyby parametr nie podany
		extract_value(collectorparameters.InsetV, "InsetV", Line, "");
		// ciśnienie rozłączające WS
		extract_value(collectorparameters.MinPress, "MinPress", Line, "3.5"); // domyślnie 2 bary do załączenia WS
		// maksymalne ciśnienie za reduktorem
		//            collectorparameters.MaxPress = 5.0 + 0.001 * ( Random( 50 ) - Random( 50 ) );
		extract_value(collectorparameters.MaxPress, "MaxPress", Line, "5.0");
		extract_value(collectorparameters.FakePower, "FakePower", Line, "");
		if (extract_value(collectorparameters.PhysicalLayout, "PhysicalLayout", Line, "3"))
			collectorparameters.CollectorsNo = std::min(collectorparameters.PhysicalLayout, 2);
		break;
	}
	case TPowerSource::PowerCable:
	{

		Powerparameters.RPowerCable.PowerTrans = LoadFIZ_PowerDecode(extract_value(Prefix + "PowerTrans", Line));
		if (Powerparameters.RPowerCable.PowerTrans == TPowerType::SteamPower)
		{

			extract_value(Powerparameters.RPowerCable.SteamPressure, Prefix + "SteamPress", Line, "");
		}
		break;
	}
	case TPowerSource::Heater:
	{
		// jeszcze nie skonczone!
		break;
	}
	default:; // nothing here
	}

	if (Powerparameters.SourceType != TPowerSource::Heater && Powerparameters.SourceType != TPowerSource::InternalSource)
	{

		extract_value(Powerparameters.MaxVoltage, Prefix + "MaxVoltage", Line, "");
		extract_value(Powerparameters.MaxCurrent, Prefix + "MaxCurrent", Line, "");
		extract_value(Powerparameters.IntR, Prefix + "IntR", Line, "");
	}
}

TPowerType TMoverParameters::LoadFIZ_PowerDecode(std::string const &Powertype) const
{
	using enum TPowerType;

	std::map<std::string, TPowerType, std::less<>> powertypes{
	    {"BioPower", BioPower}, {"MechPower", MechPower}, {"ElectricPower", ElectricPower}, {"SteamPower", SteamPower}};
	auto lookup = powertypes.find(Powertype);
	return lookup != powertypes.end() ? lookup->second : NoPower;
}

TPowerSource TMoverParameters::LoadFIZ_SourceDecode(std::string const &Source) const
{
	using enum TPowerSource;

	std::map<std::string, TPowerSource, std::less<>> powersources{{"Transducer", Transducer},   {"Generator", Generator},
	                                                 {"Accu", Accumulator}, // legacy compatibility leftover. TODO: check if we can get rid of it
	                                                 {"Accumulator", Accumulator}, {"CurrentCollector", CurrentCollector},
	                                                 {"PowerCable", PowerCable},   {"Heater", Heater},
	                                                 {"Internal", InternalSource}, {"Main", Main}};
	auto lookup = powersources.find(Source);
	return lookup != powersources.end() ? lookup->second : NotDefined;
}

TEngineType TMoverParameters::LoadFIZ_EngineDecode(std::string const &Engine) const
{
	using enum TEngineType;

	std::map<std::string, TEngineType, std::less<>> enginetypes{{"ElectricSeriesMotor", ElectricSeriesMotor},
	                                               {"DieselEngine", DieselEngine},
	                                               {"SteamEngine", SteamEngine},
	                                               {"WheelsDriven", WheelsDriven},
	                                               {"Dumb", Dumb},
	                                               {"DieselElectric", DieselElectric},
	                                               {"DumbDE", DieselElectric},
	                                               {"ElectricInductionMotor", ElectricInductionMotor},
	                                               {"Main", Main}};
	auto lookup = enginetypes.find(Engine);
	return lookup != enginetypes.end() ? lookup->second : None;
}
