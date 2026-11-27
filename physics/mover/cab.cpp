/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the cab: activation, the direction, the horns, the pullee mode

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
#include "physics/mover/internal.h"


bool TMoverParameters::DirectionForward()
{
	if (false == EIMDirectionChangeAllow())
	{
		return false;
	}

	if (MainCtrlPosNo > 0 && DirActive < 1 && (CabActive != 0 || (InactiveCabFlag & activation::neutraldirection) == 0))
	{
		++DirActive;
		DirAbsolute = DirActive * CabActive;
		SendCtrlToNext(VehicleCommand::direction, DirActive, CabActive);
		return true;
	}
	else if (DirActive == 1 && IsMainCtrlNoPowerPos() && TrainType == train_type::emu && EngineType != TEngineType::ElectricInductionMotor)
		return MinCurrentSwitch(true); //"wysoki rozruch" EN57
	return false;
};

bool TMoverParameters::ChangeCab(int direction)
{ // zmiana kabiny i resetowanie ustawien
	if (std::abs(CabOccupied + direction) < 2)
	{
		CabOccupied = CabOccupied + direction;
		if (BrakeCtrlPosNo > 0 && (BrakeSystem == TBrakeSystem::Pneumatic || BrakeSystem == TBrakeSystem::ElectroPneumatic))
		{
			BrakeLevelSet(Handle->GetPos(bh_NP));
			LimPipePress = PipePress;
			ActFlowSpeed = 0;
		}
		else
			BrakeLevelSet(Handle->GetPos(bh_NP));
		MainCtrlPos = MainCtrlNoPowerPos();
		ScndCtrlPos = 0;
		return true;
	}
	return false;
};

int TMoverParameters::SoundingHorns(int const Signal) const
{
	auto const supplied{[this](horn_supply const Supply) {
		if (Supply == horn_supply::electric)
		{
			return Power24vIsAvailable || Power110vIsAvailable;
		}
		if (VeselVolume > 0.0)
		{
			return Compressor >= HornMinPressure;
		}
		// a vehicle without a main reservoir of its own takes the air from the main pipe, a vehicle without either
		// (a road vehicle, say) has no air to check
		auto const mainpipe{(Couplers[end::front].AllowedFlag | Couplers[end::rear].AllowedFlag).HasAny(Coupling::mainHose)};
		return false == mainpipe || ScndPipePress >= HornMinPressure;
	}};
	auto horns{0};
	if (supplied(HornSupply))
	{
		horns |= Signal & (1 | 2);
	}
	if (supplied(Horn3Supply))
	{
		horns |= Signal & 4;
	}
	return horns;
}

// *************************************************************************************************
// Q: 20160710
// załączenie rozrządu
// *************************************************************************************************
bool TMoverParameters::CabActivisation(bool const Enforce)
{
	bool OK = false;

	OK = Enforce || CabActive == 0; // numer kabiny, z której jest sterowanie
	if (PulleeMode)
	{
		// nobody drives from a vehicle prepared for hauling
		OK = false;
	}
	if (OK)
	{
		CabActive = CabOccupied; // sterowanie jest z kabiny z obsadą
		DirAbsolute = DirActive * CabActive;
		CabMaster = true;
		SecuritySystem.set_enabled(true); // activate the alerter TODO: make it part of control based cab selection
		SendCtrlToNext(VehicleCommand::cabActivation, 1, CabActive);
		SendCtrlToNext(VehicleCommand::direction, DirActive, CabActive);
		if (InactiveCabFlag & activation::springbrakeoff)
		{
			SpringBrakeActivate(false);
		}
	}
	return OK;
}

bool TMoverParameters::CabActivisationAuto(bool const Enforce)
{
	bool OK = AutomaticCabActivation ? CabActivisation(Enforce) : false;
	return OK;
}

// *************************************************************************************************
// Q: 20160710
// wyłączenie rozrządu
// *************************************************************************************************
bool TMoverParameters::CabDeactivisation(bool const Enforce)
{
	bool OK = false;

	OK = Enforce || IsCabMaster(); // o ile obsada jest w kabinie ze sterowaniem
	if (OK)
	{
		if (InactiveCabFlag & activation::springbrakeon)
		{
			SpringBrakeActivate(true);
		}
		if (InactiveCabFlag & activation::pantographsup)
		{
			InactiveCabPantsCheck = true;
		}
		if (InactiveCabFlag & activation::doorpermition)
		{
			PermitDoors(side::right, true, range_t::consist);
			PermitDoors(side::left, true, range_t::consist);
		}
		if (InactiveCabFlag & activation::neutraldirection)
		{
			DirActive = 0;
			SendCtrlToNext(VehicleCommand::direction, 0, CabActive);
		}

		CabActive = 0;
		DirAbsolute = DirActive * CabActive;
		CabMaster = false;
		DepartureSignal = false; // nie buczeć z nieaktywnej kabiny
		SecuritySystem.set_enabled(false); // deactivate alerter TODO: make it part of control based cab selection

		SendCtrlToNext(VehicleCommand::cabActivation, 0, CabOccupied); // CabActive==0!
	}
	return OK;
}

bool TMoverParameters::CabDeactivisationAuto(bool const Enforce)
{
	bool OK = AutomaticCabActivation ? CabDeactivisation(Enforce) : false;
	return OK;
}

// *************************************************************************************************
// pullee mode ("jazda ciągniona", MaSzyna-EU07/maszyna#64): prepares the vehicle for being hauled dead in a
// train, or puts it back in service. while in the mode the vehicle:
// - vents the brake pipe for nothing of its own (inactive cab, vigilance, radio-stop, spring brake), only when
//   the train parts (UpdatePipePressure()),
// - gets no force from its spring brake, whatever air its cylinder has (BrakeForce()),
// - has its driver's valve cut out and its local brake released, whatever the handles show; the distributor
//   stays on the brake pipe, so it brakes and releases with the train,
// - keeps its battery off and takes no cab (BatterySwitch(), CabActivisation() and their commands).
// entering it also lets go of the hand brake and of the spring brake switch.
// returns whether the mode changed; a vehicle driven from can't enter it
// *************************************************************************************************
bool TMoverParameters::PulleeModeSwitch(bool const State)
{
	if (PulleeMode == State || (State && CabActive != 0))
	{
		return false;
	}
	PulleeMode = State;
	if (PulleeMode)
	{
		DecManualBrakeLevel(ManualBrakePosNo);
		SpringBrake.Activate = false;
		SecuritySystem.set_enabled(false);
		Battery = false;
	}
	return true;
}

// *************************************************************************************************
// Q: 20160710
// kierunek do tyłu
// *************************************************************************************************
bool TMoverParameters::DirectionBackward(void)
{
	if (false == EIMDirectionChangeAllow())
	{
		return false;
	}

	if (DirActive == 1 && MainCtrlPos == 0 && TrainType == train_type::emu && EngineType != TEngineType::ElectricInductionMotor && MinCurrentSwitch(false))
	{
		return true;
	}
	if (MainCtrlPosNo > 0 && DirActive > -1 && (CabActive != 0 || (InactiveCabFlag & activation::neutraldirection) == 0))
	{
		if (EngineType == TEngineType::WheelsDriven)
			--CabActive;
		//    else
		--DirActive;
		DirAbsolute = DirActive * CabActive;
		SendCtrlToNext(VehicleCommand::direction, DirActive, CabActive);
		return true;
	}

	return false;
}

bool TMoverParameters::EIMDirectionChangeAllow(void) const
{
	bool OK = false;
	/*
	    // NOTE: disabled while eimic variables aren't immediately synced with master controller changes inside ai module
	    OK = (EngineType != TEngineType::ElectricInductionMotor || ((eimic <= 0) && (eimic_real <= 0) && (Vel < 0.1)));
	*/
	OK = MainCtrlPos <= MainCtrlMaxDirChangePos;
	return OK;
}
