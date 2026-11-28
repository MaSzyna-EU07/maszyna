/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the master and the secondary controllers, the automatic relay and the camshaft, the integrated controller, the speed control

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


// *************************************************************************************************
// queuedEU
// *************************************************************************************************

// *************************************************************************************************
// Q: 20160710
// zwiększenie nastawinika
// *************************************************************************************************

bool TMoverParameters::IncMainCtrl(int CtrlSpeed)
{
	// basic fail conditions:
	if (MainCtrlPosNo <= 0 || CabActive == 0)
	{
		// nie ma sterowania
		return false;
	}
	if (TrainType == train_type::et22 && ScndCtrlPos != 0)
	{
		// w ET22 nie da się kręcić nastawnikiem przy włączonym boczniku
		return false;
	}
	if (TrainType == train_type::emu && DirActive == 0)
	{
		// w EZT nie da się załączyć pozycji bez ustawienia kierunku
		return false;
	}

	bool OK = false;
	if (MainCtrlPos < MainCtrlPosNo)
	{
		OK = IncMainCtrlByEngine(CtrlSpeed);
	}
	else
	{ // MainCtrlPos>=MainCtrlPosNo
		if (true == CoupledCtrl)
		{
			// wspólny wał nastawnika jazdy i bocznikowania
			if (ScndCtrlPos < ScndCtrlPosNo)
			{ // 3<3 -> false
				++ScndCtrlPos;
				OK = true;
			}
			else
			{
				OK = false;
			}
		}
	}

	if (true == OK)
	{
		ScndCtrlResistiveLockCheck(); // e.g. from the last series position on to the resistive parallel ones
		SendCtrlToNext(VehicleCommand::mainController, MainCtrlPosForCoupled(), CabActive); //???
		SendCtrlToNext(VehicleCommand::secondaryController, ScndCtrlPos, CabActive);
	}

	// hunter-101012: poprawka
	// poprzedni warunek byl niezbyt dobry, bo przez to przy trzymaniu +
	// styczniki tkwily na tej samej pozycji (LastRelayTime byl caly czas 0 i rosl
	// po puszczeniu plusa)

	if (OK)
	{
		if (DelayCtrlFlag)
		{
			if (LastRelayTime >= InitialCtrlDelay && MainCtrlPos == 1)
				LastRelayTime = 0;
		}
		else if (LastRelayTime > CtrlDelay)
			LastRelayTime = 0;
	}
	return OK;
}

// a notch up, as the kind of the engine takes it; returns whether the controller moved
bool TMoverParameters::IncMainCtrlByEngine(int const CtrlSpeed)
{
	bool OK = false;
	switch (EngineType)
	{
	case TEngineType::None:
	case TEngineType::Dumb:
	case TEngineType::DieselElectric:
	case TEngineType::ElectricInductionMotor:
	{
		if (CtrlSpeed > 1)
		{
			OK = IncMainCtrl(1) && IncMainCtrl(CtrlSpeed - 1); // a fail will propagate up the recursion chain. should this be || instead?
			break;
		}
		++MainCtrlPos;
		OK = true;
		if (EIMCtrlType == 0 && ( SpeedCtrlAutoTurnOffFlag & 1 ) == 1 && MainCtrlActualPos != MainCtrlPos)
		{
			SpeedCtrlSwitch(false);
			SpeedCtrlUnit.IsActive = false;
		}
		break;
	}

	case TEngineType::ElectricSeriesMotor:
	{
		OK = IncMainCtrlSeriesMotor(CtrlSpeed);
		break;
	}

	case TEngineType::DieselEngine:
	{
		if (CtrlSpeed > 1)
		{
			while (MainCtrlPos < MainCtrlPosNo && IncMainCtrl(1))
			{
				// all work is done in the loop header
			}
		}
		else
		{
			++MainCtrlPos;
		}
		CompressorAllow = MainCtrlPowerPos() > 0;
		OK = true;
		break;
	}

	case TEngineType::WheelsDriven:
	{
		OK = AddPulseForce(CtrlSpeed);
		break;
	}
	default:
		break;
	} // switch EngineType of
	return OK;
}

// a notch up of the series motors' controller: CtrlSpeed over 1 runs it to the first position without resistors;
// returns whether it moved as asked
bool TMoverParameters::IncMainCtrlSeriesMotor(int const CtrlSpeed)
{
	bool OK = false;
	if (DirActive == 0)
	{
		return false;
	}

	if (CtrlSpeed > 1)
	{
		// szybkie przejœcie na bezoporow¹
		if (TrainType == train_type::et40)
		{
			return false; // this means ET40 won't react at all to fast acceleration command. should it issue just IncMainCtrl(1) instead?
		}
		while (RList[MainCtrlPos].R > 0.0 && IncMainCtrl(1))
		{
			// all work is done in the loop header
		}
		OK = false; // shouldn't this be part of the loop above?
		// if (TrainType=train_type::et40)  then
		// while Abs (Im)>IminHi do
		//   dec(MainCtrlPos);
		//  OK:=false ;
	}
	else
	{ // CtrlSpeed == 1
		++MainCtrlPos;
		OK = true;
		if (Imax == ImaxHi && RList[MainCtrlPos].Bn > 1)
		{
			/* NOTE: disabled, relay configuration was moved to compute_movement_
			                            if( true == MaxCurrentSwitch( false )) {
			                                // wylaczanie wysokiego rozruchu
			                                SetFlag( SoundFlag, sound::relay );
			                            }
			*/
			if (TrainType == train_type::et42)
			{
				--MainCtrlPos;
				OK = false;
			}
		}
		//
		// if (TrainType == "et40")
		//  if (Abs(Im) > IminHi)
		//   {
		//    MainCtrlPos--; //Blokada nastawnika po przekroczeniu minimalnego pradu
		//    OK = false;
		//   }
		//}
	}

	if (TrainType == train_type::et42 && true == DynamicBrakeFlag && MainCtrlPos > 20)
	{
		MainCtrlPos = 20;
		OK = false;
	}
	return OK;
}

// *****************************************************************************
// Q: 20160710
// zmniejszenie nastawnika
// *****************************************************************************
bool TMoverParameters::DecMainCtrl(int CtrlSpeed)
{
	bool OK = false;
	// basic fail conditions:
	if (MainCtrlPosNo <= 0 || CabActive == 0)
	{
		// nie ma sterowania
		OK = false;
	}
	else if (CtrlSpeed == 0)
	{
		return false;
	}
	else
	{
		// TBD, TODO: replace with mainctrlpowerpos() check?
		if (MainCtrlPos > 0)
		{
			if (TrainType != train_type::et22 || ScndCtrlPos == 0) // Ra: ET22 blokuje nastawnik przy boczniku
			{
				if (CoupledCtrl && ScndCtrlPos > 0)
				{
					ScndCtrlPos--; // wspolny wal
					OK = true;
				}
				else
					OK = DecMainCtrlByEngine(CtrlSpeed);
			}
		}
		else if (EngineType == TEngineType::WheelsDriven)
			OK = AddPulseForce(-CtrlSpeed);
		else
			OK = false;

		if (OK)
		{
			ScndCtrlResistiveLockCheck(); // the shunt drops when the master controller leaves a resistance-free position
			/*OK:=*/SendCtrlToNext(VehicleCommand::mainController, MainCtrlPosForCoupled(), CabActive); // hmmmm...???!!!
			/*OK:=*/SendCtrlToNext(VehicleCommand::secondaryController, ScndCtrlPos, CabActive);
		}
	}
	// if OK then LastRelayTime:=0;
	// hunter-101012: poprawka
	if (OK)
	{
		if (DelayCtrlFlag)
		{
			if (LastRelayTime >= InitialCtrlDelay)
				LastRelayTime = 0;
		}
		else if (LastRelayTime > CtrlDownDelay)
			LastRelayTime = 0;
	}
	return OK;
}

// a notch down, as the kind of the engine takes it; returns whether the controller moved
bool TMoverParameters::DecMainCtrlByEngine(int const CtrlSpeed)
{
	bool OK = false;
	switch (EngineType)
	{
	case TEngineType::None:
	case TEngineType::Dumb:
	case TEngineType::DieselElectric:
	case TEngineType::ElectricInductionMotor:
	{
		if (CtrlSpeed == 1)
		{
			MainCtrlPos--;
			OK = true;
			if (EIMCtrlType == 0 && (SpeedCtrlAutoTurnOffFlag & 1) == 1 && MainCtrlActualPos != MainCtrlPos)
			{
				SpeedCtrlSwitch(false);
				SpeedCtrlUnit.IsActive = false;
			}
		}
		else if (CtrlSpeed > 1)
			OK = DecMainCtrl(1) && DecMainCtrl(CtrlSpeed - 1); // CtrlSpeed-1);
		break;
	}

	case TEngineType::ElectricSeriesMotor:
	{
		OK = DecMainCtrlSeriesMotor(CtrlSpeed);
		break;
	}

	case TEngineType::DieselEngine:
	{
		if (CtrlSpeed == 1)
		{
			MainCtrlPos--;
			OK = true;
		}
		else if (CtrlSpeed > 1)
		{
			while (MainCtrlPos > 0 || RList[MainCtrlPos].Mn > 0)
				DecMainCtrl(1);
			OK = true;
		}
		break;
	}
	default:
		break;
	} // switch EngineType
	return OK;
}

// a notch down of the series motors' controller: CtrlSpeed over 1 runs it down to the first position without resistors
bool TMoverParameters::DecMainCtrlSeriesMotor(int const CtrlSpeed)
{
	bool OK = false;
	if (CtrlSpeed == 1) /*and (ScndCtrlPos=0)*/
	{
		MainCtrlPos--;
		//                if (MainCtrlPos=0) and (ScndCtrlPos=0) and
		//                (TrainType<>train_type::et40)and(TrainType<>train_type::ep05) then
		//                 StLinFlag:=false;
		//                if (MainCtrlPos=0) and (TrainType<>train_type::et40) and
		//                (TrainType<>train_type::ep05) then
		//                 MainCtrlActualPos:=0; //yBARC: co to tutaj robi? ;)
		OK = true;
	}
	else if (CtrlSpeed > 1) /*and (ScndCtrlPos=0)*/
	{
		OK = true;
		if (RList[MainCtrlPos].R == 0) // Q: tu zrobilem = ;]
			DecMainCtrl(1);
		while (RList[MainCtrlPos].R > 0 && DecMainCtrl(1))
			; // takie chamskie, potem poprawie}
	}
	return OK;
}

bool TMoverParameters::IsMainCtrlActualNoPowerPos() const
{
	// TODO: wrap controller pieces into a class for potential specializations, similar to brake subsystems
	return MainCtrlActualPos <= MainCtrlNoPowerPos();
}

bool TMoverParameters::IsMainCtrlNoPowerPos() const
{
	// TODO: wrap controller pieces into a class for potential specializations, similar to brake subsystems
	return MainCtrlPos <= MainCtrlNoPowerPos();
}

bool TMoverParameters::IsMainCtrlMaxPowerPos() const
{
	// TODO: wrap controller pieces into a class for potential specializations, similar to brake subsystems
	return MainCtrlPos == MainCtrlPosNo;
}

int TMoverParameters::MainCtrlNoPowerPos() const
{

	switch (EIMCtrlType)
	{
	case 1:
	{
		return 3;
	}
	case 2:
	{
		return 3 + EIMCtrlEmergencyPositions();
	}
	case 3:
	{
		return UniCtrlNoPowerPos;
	}
	default:
	{
		return 0;
	}
	}
}

int TMoverParameters::MainCtrlActualPowerPos() const
{

	return MainCtrlActualPos - MainCtrlNoPowerPos();
}

int TMoverParameters::MainCtrlPowerPos() const
{

	return MainCtrlPos - MainCtrlNoPowerPos();
}

int TMoverParameters::EIMCtrlEmergencyPositions() const
{
	return (EIMCtrlType == 2 && EIMCtrlMainEmergency) ? 1 : 0;
}

bool TMoverParameters::IsMainCtrlEmergencyPos() const
{
	return MainCtrlPos < EIMCtrlEmergencyPositions();
}

int TMoverParameters::MainCtrlPosForCoupled() const
{
	// the emergency brake itself reaches the train through the brake pipe, vented by the vehicle driven from
	return std::max(0, MainCtrlPos - EIMCtrlEmergencyPositions());
}

int TMoverParameters::MainCtrlPosFromCoupled(int const Position) const
{
	return Position + EIMCtrlEmergencyPositions();
}

// *************************************************************************************************
// Q: 20160710
// zwiększenie bocznika
// *************************************************************************************************
bool TMoverParameters::IncScndCtrl(int CtrlSpeed)
{
	bool OK = false;

	if (DynamicBrakeFlag && TrainType == train_type::et42 && CabActive != 0 && IsMainCtrlNoPowerPos() && ScndCtrlPos == 0)
	{
		OK = DynamicBrakeSwitch(false);
	}
	else if (ScndCtrlPosNo > 0 && CabActive != 0 && !(TrainType == train_type::et42 && (Imax == ImaxHi || (DynamicBrakeFlag && MainCtrlPowerPos() > 0))))
	{
		//     if (RList[MainCtrlPos].R=0) and (MainCtrlPos>0) and (ScndCtrlPos<ScndCtrlPosNo) and
		//     (not CoupledCtrl) then
		if (ScndCtrlPos < ScndCtrlPosNo && !CoupledCtrl && !IsScndCtrlLockedByMainCtrl())
		{
			// TBD, TODO: refactor this validation as part of relay check routine (currently in tractionforce()
			// TBD, TODO: diesel electric engine utilize scndctrlactualpos like the other types?
			if (EngineType == TEngineType::DieselElectric &&
			    (IsMainCtrlNoPowerPos() || fieldshunting::diesel_electric(AutoRelayType, FieldShuntSwitch, AutoRelayFlag) != fieldshunting::control::manual || ShuntMode || false == Mains))
			{
				OK = false;
			}
			else
			{
				if (CtrlSpeed == 1)
				{
					ScndCtrlPos++;
				}
				else if (CtrlSpeed > 1)
				{
					ScndCtrlPos = ScndCtrlPosNo; // takie chamskie, potem poprawie
				}
				OK = true;
			}
		}
		else // nie mozna zmienic
			OK = false;
		if (OK)
		{
			/*OK:=*/SendCtrlToNext(VehicleCommand::mainController, MainCtrlPosForCoupled(), CabActive); //???
			/*OK:=*/SendCtrlToNext(VehicleCommand::secondaryController, ScndCtrlPos, CabActive);
		}
	}
	else // nie ma sterowania
		OK = false;
	// if OK then LastRelayTime:=0;
	// hunter-101012: poprawka
	if (OK && LastRelayTime > CtrlDelay)
		LastRelayTime = 0;

	if (OK && EngineType == TEngineType::ElectricInductionMotor && ScndCtrlPosNo == 1 && MainCtrlPos > 0)
	{
		SpeedCtrlValue = Vel;
		if (EIMCtrlType == 0 && (SpeedCtrlAutoTurnOffFlag & 1) == 1)
		{
			MainCtrlActualPos = MainCtrlPos;
		}
		SpeedCtrlUnit.IsActive = true;
	}

	if (OK && SpeedCtrl && ScndCtrlPos == 1 && EngineType == TEngineType::DieselEngine)
	{
		// NOTE: round() already adds 0.5, are the ones added here as well correct?
		SpeedCtrlValue = static_cast<double>(Round(Vel));
		SpeedCtrlUnit.IsActive = true;
	}

	return OK;
}

// *************************************************************************************************
// Q: 20160710
// zmniejszenie bocznika
// *************************************************************************************************
bool TMoverParameters::DecScndCtrl(int CtrlSpeed)
{
	bool OK = false;

	if (IsMainCtrlNoPowerPos() && CabActive != 0 && TrainType == train_type::et42 && ScndCtrlPos == 0 && !DynamicBrakeFlag && CtrlSpeed == 1)
	{
		// Ra: AI wywołuje z CtrlSpeed=2 albo gdy ScndCtrlPos>0
		OK = DynamicBrakeSwitch(true);
	}
	else if (ScndCtrlPosNo > 0 && CabActive != 0)
	{
		if (ScndCtrlPos > 0 && !CoupledCtrl && (EngineType != TEngineType::DieselElectric || !AutoRelayFlag) && !IsScndCtrlLockedByMainCtrl())
		{
			if (CtrlSpeed == 1)
			{
				ScndCtrlPos--;
			}
			else if (CtrlSpeed > 1)
			{
				ScndCtrlPos = 0; // takie chamskie, potem poprawie
			}
			OK = true;
		}
		else
			OK = false;
		if (OK)
		{
			/*OK:=*/SendCtrlToNext(VehicleCommand::mainController, MainCtrlPosForCoupled(), CabActive); //???
			/*OK:=*/SendCtrlToNext(VehicleCommand::secondaryController, ScndCtrlPos, CabActive);
		}
	}
	else
		OK = false;
	// if OK then LastRelayTime:=0;
	// hunter-101012: poprawka
	if (OK && LastRelayTime > CtrlDownDelay)
		LastRelayTime = 0;

	if (OK && EngineType == TEngineType::ElectricInductionMotor && ScndCtrlPosNo == 1)
	{
		SpeedCtrlValue = 0;
		SpeedCtrlUnit.IsActive = false;
		if (SpeedCtrlUnit.ManualStateOverride)
		{
			eimic = 0.0;
		}
	}

	if (OK && SpeedCtrl && ScndCtrlPos == 0 && EngineType == TEngineType::DieselEngine)
	{
		SpeedCtrlValue = 0;
		SpeedCtrlUnit.IsActive = false;
		if (SpeedCtrlUnit.ManualStateOverride)
		{
			eimic = 0.0;
		}
	}

	return OK;
}

bool TMoverParameters::IsScndCtrlLockedByMainCtrl() const
{
	// ET21, ET22, EP09: the field shunting engages only on the resistance-free positions of the master controller (eu07/maszyna#402)
	return ScndCtrlResistiveLock && (IsMainCtrlNoPowerPos() || RList[MainCtrlPos].R >= 0.01);
}

void TMoverParameters::ScndCtrlResistiveLockCheck()
{
	// the shunt controller, locked in both directions on zero and the resistive positions, would otherwise stay engaged there once the
	// master controller is taken back (or set from elsewhere, e.g. by the AI or another vehicle of the consist) with the shunt on
	if (ScndCtrlPos > 0 && !CoupledCtrl && IsScndCtrlLockedByMainCtrl())
	{
		ScndCtrlPos = 0;
	}
}

int TMoverParameters::GetVirtualScndPos() const
{
	if (TrainType == train_type::et42 && DynamicBrakeFlag && !ScndCtrlPos)
		return -1;
	return ScndCtrlPos;
}

bool TMoverParameters::IsScndCtrlNoPowerPos() const
{
	// TODO: refine the check on account of potential electric series vehicles with speed control
	return ScndCtrlPos == 0 || true == SpeedCtrl;
}

bool TMoverParameters::IsScndCtrlMaxPowerPos() const
{
	// TODO: refine the check on account of potential electric series vehicles with speed control
	return ScndCtrlPos == ScndCtrlPosNo || true == SpeedCtrl;
}

// *************************************************************************************************
// Q: 20160713
// Włączenie / wyłączenie automatycznego rozruchu
// *************************************************************************************************
bool TMoverParameters::AutoRelaySwitch(bool State)
{
	bool ARS;
	if (AutoRelayType == 2 && AutoRelayFlag != State)
	{
		AutoRelayFlag = State;
		ARS = true;
		SendCtrlToNext(VehicleCommand::autoRelaySwitch, int(State), CabActive);
	}
	else
		ARS = false;

	return ARS;
}

// *************************************************************************************************
// Q: 20160724
// Sprawdzenie warunków pracy automatycznego rozruchu
// *************************************************************************************************

bool TMoverParameters::AutoRelayCheck(void)
{
	bool OK = false; // b:int;

	auto const motorconnectorsoff{false == MotorConnectorsCheck()};

	// Ra 2014-06: dla SN61 nie działa prawidłowo
	// yBARC - rozlaczenie stycznikow liniowych
	if (motorconnectorsoff || (HasCamshaft ? IsMainCtrlActualNoPowerPos() : IsMainCtrlNoPowerPos()))
	{
		StLinFlag = false;
		if (false == DynamicBrakeFlag)
		{
			Im = 0;
			Itot = 0;
			ResistorsFlag = false;
		}
	}

	// sprawdzenie wszystkich warunkow (AutoRelayFlag, AutoSwitch, Im<Imin)
	auto const ARFASI2{(false == AutoRelayFlag || (motor_param(ScndCtrlActualPos).AutoSwitch && abs(Im) < Imin))};
	auto const ARFASI{
	    (false == AutoRelayFlag || (RList[MainCtrlActualPos].AutoSwitch && abs(Im) < Imin) || (!RList[MainCtrlActualPos].AutoSwitch && RList[MainCtrlActualPos].Relay < MainCtrlPos))};
	// brak PSR                   na tej pozycji działa PSR i prąd poniżej progu
	// na tej pozycji nie działa PSR i pozycja walu ponizej
	//                         chodzi w tym wszystkim o to, żeby można było zatrzymać rozruch na
	//                         jakiejś pozycji wpisując Autoswitch=0 i wymuszać
	//                         przejście dalej przez danie nastawnika na dalszą pozycję - tak to do
	//                         tej pory działało i na tym się opiera fizyka ET22-2k
	{
		if (StLinFlag)
		{
			if (RList[MainCtrlActualPos].R == 0 && (ScndCtrlActualPos > 0 || ScndCtrlPos > 0) && (!CoupledCtrl || RList[MainCtrlActualPos].Relay == MainCtrlPos))
			{ // zmieniaj scndctrlactualpos
				OK = AutoRelayShuntStep(ARFASI2);
			}
			else
			{ // zmieniaj mainctrlactualpos
				OK = AutoRelayMainStep(ARFASI);
			}
		}
		else // not StLinFlag
		{
			OK = AutoRelayLineContactorsStep(motorconnectorsoff);
		}
		if (OK)
			LastRelayTime = 0;

		return OK;
	}
}

// a step of the shunt field on a position without resistors, up or down to the second controller
bool TMoverParameters::AutoRelayShuntStep(bool const ARFASI2)
{
	bool OK = false;
		// scnd bez samoczynnego rozruchu
		if (ScndCtrlActualPos < ScndCtrlPos)
		{
			if (LastRelayTime > CtrlDelay && ARFASI2)
			{
				++ScndCtrlActualPos;
				SetFlag(SoundFlag, sound::shuntfield);
				OK = true;
			}
		}
		else if (ScndCtrlActualPos > ScndCtrlPos)
		{
			if (LastRelayTime > CtrlDownDelay && TrainType != train_type::emu)
			{
				--ScndCtrlActualPos;
				SetFlag(SoundFlag, sound::shuntfield);
				OK = true;
			}
		}
		else
			OK = false;
	return OK;
}

// a step of the main controller's relays, up or down to its position, or the shunt field off on the resistors
bool TMoverParameters::AutoRelayMainStep(bool const ARFASI)
{
	bool OK = false;
	// running backwards, the motors don't get joined in more branches than allowed (the series connection
	// alone, usually); the camshaft still goes back down from there (eu07/maszyna#393: it stayed where it
	// was, the line contactors closed, whatever the driver did with the master controller)
	auto const backwardslimit{DirActive < 0 && TrainType != train_type::pseudodiesel && RList[MainCtrlActualPos + 1].Bn > BackwardsBranchesAllowed};
	// main bez samoczynnego rozruchu
	if (false == backwardslimit && MainCtrlActualPos < std::size(RList) - 1 // crude guard against running out of current fixed table
	    && (RList[MainCtrlActualPos].Relay < MainCtrlPos || (RList[MainCtrlActualPos + 1].Relay == MainCtrlPos && MainCtrlActualPos < RlistSize) ||
	        (TrainType == train_type::et22 && DelayCtrlFlag)))
	{
		OK = AutoRelayMainUp(ARFASI);
	}
	else if (RList[MainCtrlActualPos].Relay > MainCtrlPos)
	{
		OK = AutoRelayMainDown();
	}
	else if (RList[MainCtrlActualPos].R > 0 && ScndCtrlActualPos > 0)
	{
		if (LastRelayTime > CtrlDownDelay)
		{
			--ScndCtrlActualPos; // boczniki nie dzialaja na poz. oporowych
			SetFlag(SoundFlag, sound::shuntfield);
			OK = true;
		}
	}
	else
		OK = false;
	return OK;
}

// a step of the main relays up to the controller's position, at once to the parallel position without resistors
// on a fast serial circuit (303E); returns whether the relays moved, never into parallel with the overload relay high
bool TMoverParameters::AutoRelayMainUp(bool const ARFASI)
{
	bool OK = false;
	// prevent switch to parallel mode if motor overload relay is set to high threshold mode
	if (IsMotorOverloadRelayHighThresholdOn() && RList[MainCtrlActualPos + 1].Bn > 1)
	{
		return false;
	}

	if (RList[MainCtrlPos].R == 0 && MainCtrlPos > 0 && MainCtrlPos != MainCtrlPosNo && FastSerialCircuit == 1)
	{
		// szybkie wchodzenie na bezoporowa (303E)
		// MainCtrlActualPos:=MainCtrlPos; //hunter-111012:
		++MainCtrlActualPos;
		if (MainCtrlPos - MainCtrlActualPos == 1)
		{
			// HACK: ensure we play only single sound of basic relays for entire trasition; return false
			// for all but last step despite configuration change, to prevent playback of the basic relay sound
			// TBD, TODO: move the basic sound event here and enable it with call parameter
			OK = true;
		}
		if (RList[MainCtrlActualPos].R == 0)
		{
			SetFlag(SoundFlag, sound::parallel | sound::loud);
			OK = true;
		}
	}
	else if (LastRelayTime > CtrlDelay && ARFASI)
	{
		OK = AutoRelayMainNotchUp();
	}
	return OK;
}

// a notch up of the main relays after their delay, through the group switch of the ET22, and the sounds of the
// parallel positions; returns whether they moved
bool TMoverParameters::AutoRelayMainNotchUp()
{
	bool OK = false;
	// WriteLog("LRT = " + FloatToStr(LastRelayTime) + ", " +
	// FloatToStr(CtrlDelay));
	if (TrainType == train_type::et22 && MainCtrlPos > 1 && (RList[MainCtrlActualPos].Bn < RList[MainCtrlActualPos + 1].Bn || DelayCtrlFlag))
	{
		// et22 z walem grupowym
		if (!DelayCtrlFlag) // najpierw przejscie
		{
			++MainCtrlActualPos;
			DelayCtrlFlag = true; // tryb przejscia
			OK = true;
		}
		else if (LastRelayTime > 4 * CtrlDelay) // przejscie
		{

			DelayCtrlFlag = false;
			OK = true;
		}
		/*
		                            else
		                                ;
		*/
	}
	else // nie ET22 z wałem grupowym
	{
		++MainCtrlActualPos;
		OK = true;
	}
	//---------
	// hunter-111211: poprawki
	if (MainCtrlActualPos > 0)
	{
		if (RList[MainCtrlActualPos].R == 0 && MainCtrlActualPos != MainCtrlPosNo)
		{
			// wejscie na bezoporowa
			SetFlag(SoundFlag, sound::parallel | sound::loud);
		}
		else if (RList[MainCtrlActualPos].R > 0 && RList[MainCtrlActualPos - 1].R == 0)
		{
			// wejscie na drugi uklad
			SetFlag(SoundFlag, sound::parallel);
		}
	}
	return OK;
}

// a step of the main relays down to the controller's position; returns whether they moved
bool TMoverParameters::AutoRelayMainDown()
{
	bool OK = false;
	if (RList[MainCtrlPos].R == 0 && MainCtrlPos > 0 && !(MainCtrlPos == MainCtrlPosNo) && FastSerialCircuit == 1)
	{
		// szybkie wchodzenie na bezoporowa (303E)
		// MainCtrlActualPos:=MainCtrlPos; //hunter-111012:
		--MainCtrlActualPos;
		OK = true;
		if (RList[MainCtrlActualPos].R == 0)
		{
			SetFlag(SoundFlag, sound::parallel);
		}
	}
	else if (LastRelayTime > CtrlDownDelay)
	{
		if (TrainType != train_type::emu) // tutaj powinien być tryb sterowania wałem
		{
			--MainCtrlActualPos;
			OK = true;
		}
		// hunter-111211: poprawki
		if (MainCtrlActualPos > 0 && RList[MainCtrlActualPos].R == 0)
		{
			// dzwieki schodzenia z bezoporowej}
			SetFlag(SoundFlag, sound::parallel);
		}
	}
	return OK;
}

// the line contactors closed at the first position, or, open, the camshaft run back to zero
bool TMoverParameters::AutoRelayLineContactorsStep(bool const motorconnectorsoff)
{
	bool OK = false;
	// ybARC - zalaczenie stycznikow liniowych
	if (false == motorconnectorsoff && MainCtrlActualPos == 0 && (TrainType == train_type::emu || HasCamshaft ? MainCtrlPowerPos() > 0 : MainCtrlPowerPos() == 1))
	{

		DelayCtrlFlag = true;
		if (LastRelayTime >= InitialCtrlDelay)
		{
			StLinFlag = true;
			MainCtrlActualPos = 1;
			DelayCtrlFlag = false;
			SetFlag(SoundFlag, sound::relay | sound::loud);
			OK = true;
		}
	}
	else
	{
		DelayCtrlFlag = false;
	}

	if (false == StLinFlag && (MainCtrlActualPos > 0 || ScndCtrlActualPos > 0))
	{
		OK |= CamshaftRunDown();
	}
	return OK;
}

// the camshaft, the line contactors open, run back to its zero position a step at a time, or at once without one;
// returns whether it moved
bool TMoverParameters::CamshaftRunDown()
{
	bool OK = false;

	if (CoupledCtrl)
	{

		if (TrainType == train_type::emu)
		{
			OK = CamshaftRunDownEMU();
		}
		else if (LastRelayTime > CtrlDownDelay) // wal kulakowy dwukierunkowy
		{
			if (ScndCtrlActualPos > 0)
			{
				--ScndCtrlActualPos;
				SetFlag(SoundFlag, sound::shuntfield);
			}
			else
			{
				--MainCtrlActualPos;
			}
			OK = true;
		}
	}
	else if (HasCamshaft)
	{
		// wal kulakowy dwukierunkowy
		if (LastRelayTime > CtrlDownDelay)
		{
			if (MainCtrlActualPos > 0)
			{
				--MainCtrlActualPos;
			}
			ScndCtrlActualPos = 0;
			OK = true;
		}
	}
	else
	{
		MainCtrlActualPos = 0;
		ScndCtrlActualPos = 0;
		OK = true;
	}
	return OK;
}

// the one way camshaft of the EN57 run on to its end and round to zero; returns whether it moved
bool TMoverParameters::CamshaftRunDownEMU()
{
	bool OK = false;
	// EN57 wal jednokierunkowy calosciowy
	if (MainCtrlActualPos == 1)
	{

		MainCtrlActualPos = 0;
		OK = true;
	}
	else
	{

		if (LastRelayTime > CtrlDownDelay)
		{

			if (MainCtrlActualPos < RlistSize)
			{
				// dojdz do konca
				++MainCtrlActualPos;
			}
			else if (ScndCtrlActualPos < ScndCtrlPosNo)
			{
				// potem boki
				++ScndCtrlActualPos;
				SetFlag(SoundFlag, sound::shuntfield);
			}
			else
			{
				// i sie przewroc na koniec
				MainCtrlActualPos = 0;
				ScndCtrlActualPos = 0;
			}
			OK = true;
		}
	}
	return OK;
}

bool TMoverParameters::MotorConnectorsCheck()
{

	// hunter-111211: wylacznik cisnieniowy
	// (on some vehicles it watches the brake pipe alone)
	ControlPressureSwitch = HasControlPressureSwitch && ((PressureSwitchBrakeCylinder && BrakePress > 2.0) || PipePress < 3.6);

	if (true == ControlPressureSwitch)
	{
		return false;
	}

	// a line breaker closed ahead of the pantographs holds without voltage; what cuts the motors off there is its
	// undervoltage relay, which elsewhere opens the line breaker
	auto const undervoltage{PantographsRequireMains && false == NoVoltRelay};

	auto const connectorsoff{false == Mains || undervoltage || true == FuseFlag || true == StLinSwitchOff || DirActive == 0};

	return false == connectorsoff && LineContactorsInterlockReleased();
}

bool TMoverParameters::LineContactorsInterlockReleased() const
{
	switch (LineContactorsInterlock)
	{
	case line_contactors_interlock::converter:
		// the converter of this very vehicle, not the 110 V of the trainset: a section with its fans off doesn't start
		return ConverterFlag;
	case line_contactors_interlock::motor_blowers:
		return std::ranges::all_of(MotorBlowers, [this](auto const &Blower) {
			if (Blower.min_start_velocity < 0)
			{
				return Blower.is_active;
			}
			// a blower which starts with the speed and the motor current (MotorBlowersStartVelocity) can't run before
			// the contactors close, nor after a stop longer than it keeps running for: it's enough that it's
			// switched on and has power, so it starts once the vehicle moves
			return MotorBlowerPowerIsAvailable(Blower) && false == Blower.is_disabled && (Blower.start_type == start_t::manual ? Blower.is_enabled : true);
		});
	default:
		return true;
	}
}
namespace
{

// the setting of the universal controller one UniCtrlValStep further from Value towards Limit, never past it
double next_uc_step(double const Value, double const Limit, double const Step)
{
	constexpr double epsilon{1e-6};
	if (Limit > Value)
	{
		return std::min((std::floor(Value / Step + epsilon) + 1.0) * Step, Limit);
	}
	return std::max((std::ceil(Value / Step - epsilon) - 1.0) * Step, Limit);
}

// the universal controller with UniCtrlValStep: the setting the continuous ramp got to (Ramped) turned into whole steps.
// a move of the controller onto a position which changes the setting makes one step at once, the ramp then one step each
// time it covers a step's length; the steps stop at the position's set value (or its bound, past which the ramp doesn't go)
double stepped_uc_value(double const Current, double const Ramped, TUniversalCtrl const &Position, double const Step, bool const Moved, double &Progress)
{
	if (Moved)
	{
		Progress = 0.0;
	}
	double const delta{Ramped - Current};
	if (std::abs(delta) < 1e-9)
	{
		Progress = 0.0;
		return Ramped;
	}
	double limit{0.0};
	if (delta > 0.0)
	{
		limit = Position.SetCtrlVal > Current ? Position.SetCtrlVal : Position.MaxCtrlVal;
	}
	else
	{
		limit = Position.SetCtrlVal < Current ? Position.SetCtrlVal : Position.MinCtrlVal;
	}
	Progress = Moved ? Step : Progress + std::abs(delta);
	double value{Current};
	bool reached{std::abs(value - limit) < 1e-9};
	while (!reached && Progress >= Step - 1e-9)
	{
		value = next_uc_step(value, limit, Step);
		Progress -= Step;
		reached = std::abs(value - limit) < 1e-9;
	}
	if (reached)
	{
		Progress = 0.0;
	}
	// added after the move to std::clamp: bounds given the other way round are put in order, as master did it
	auto const [lowest, highest] = std::minmax(Position.MinCtrlVal, Position.MaxCtrlVal);
	return std::clamp(value, lowest, highest);
}

} // namespace

void TMoverParameters::CheckEIMIC(double dt)
{
	double offset = EIMCtrlAdditionalZeros ? 1.0 : 0.0;
	double multiplier = (EIMCtrlEmergency ? 1.0 : 0.0) + offset;
	// gdy SplitEDPneumaticBrake jest wlaczone, sterowanie ujemne (hamowanie ED) bierzemy
	// z osobnego nastawnika DynamicBrakeCtrl, a nie z hamulca pomocniczego (LocalBrake).
	// Dodatkowo zapotrzebowanie ED jest ograniczone przez strefe Vh0..Vh1: powyzej Vh1 - pelny ED,
	// pomiedzy Vh1 a Vh0 - liniowe zanikanie, ponizej Vh0 - zerowe (brak hamowania pneumatycznego
	// jako uzupelnienia, poniewaz LocalBrake jest oddzielny i operuje wylacznie pneumatyka).
	double brakeDemand = SplitEDPneumaticBrake ? DynamicBrakeRatio() : LocalBrakeRatio();
	if (SplitEDPneumaticBrake)
	{
		double const vh0 = inductionMotor.config[eimc_p_Vh0];
		double const vh1 = inductionMotor.config[eimc_p_Vh1];
		if (vh1 > vh0 + 0.001)
		{
			double const vhRamp = std::clamp((Vel - vh0) / (vh1 - vh0), 0.0, 1.0);
			brakeDemand *= vhRamp;
		}
	}
	switch (EIMCtrlType)
	{
	case 0:
		EIMICFromController(brakeDemand, offset, multiplier);
		break;
	case 1:
		EIMICFromSteps(dt);
		break;
	case 2:
		EIMICFromFourPositions(dt);
		break;
	case 3:
		EIMICFromUniversalController(dt, brakeDemand);
		break;
	}
	if (LocHandleTimeTraxx)
	{
		EIMICTraxxLocalBrake(dt);
	}

	EIMICLimit();
}

// the controller (EIMCtrlType 0): the position of the main controller, or the analog one, or the brake asked
void TMoverParameters::EIMICFromController(double const brakeDemand, double const offset, double const multiplier)
{
	eimic = brakeDemand > 0.01 ? -brakeDemand : eimic_analog > 0.01 ? eimic_analog : (double)MainCtrlPos / (double)MainCtrlPosNo;
	if (EIMCtrlAdditionalZeros || EIMCtrlEmergency)
	{
		if (eimic > 0.001)
			eimic = std::max(0.002, eimic * (double)MainCtrlPosNo / ((double)MainCtrlPosNo - offset) - offset / ((double)MainCtrlPosNo - offset));
		if (eimic < -0.001 && BrakeHandle != TBrakeHandle::MHZ_EN57)
			eimic = std::min(-0.002, eimic * (double)LocalBrakePosNo / ((double)LocalBrakePosNo - multiplier) + offset / ((double)LocalBrakePosNo - multiplier));
	}
	if (eimic > 0.001 && SpeedCtrlUnit.IsActive)
		eimic = std::max(eimic, SpeedCtrlUnit.MinPower);
}

// EIMCtrlType 1: the positions B+ to TMax move the value at a steady pace
void TMoverParameters::EIMICFromSteps(double dt)
{
	switch (MainCtrlPos)
	{
	case 0: // B+
		eimic -= std::clamp(1.0 + eimic, 0.0, dt * 0.14); // odejmuj do -1
		break;
	case 1: // B
		eimic -= std::clamp(0.0 + eimic, 0.0, dt * 0.14); // odejmuj do 0
		break;
	case 2: // B-
	case 3: // 0
	case 4: // T-
		eimic -= std::clamp(0.0 + eimic, 0.0, dt * 0.14); // odejmuj do 0
		eimic += std::clamp(0.0 - eimic, 0.0, dt * 0.14); // dodawaj do 0
		break;
	case 5: // T
		eimic += std::clamp(0.0 - eimic, 0.0, dt * 0.14); // dodawaj do 0
		break;
	case 6: // T+
		eimic += std::clamp(1.0 - eimic, 0.0, dt * 0.14); // dodawaj do 1
		break;
	case 7: // TMax
		eimic += std::clamp(1.0 - eimic, 0.0, dt * 0.14); // dodawaj do 1, max
		break;
	default:
		break;
	}
	if (MainCtrlPos >= 3 && eimic < 0)
		eimic = 0;
	if (MainCtrlPos <= 3 && eimic > 0)
		eimic = 0;
}

// EIMCtrlType 2: the positions 0 to 4 move the value towards -1, 0 or 1, from the controller's delay on; the
// emergency brake position below them, where the controller has one, brakes in full at once
void TMoverParameters::EIMICFromFourPositions(double dt)
{
	if (IsMainCtrlEmergencyPos())
	{
		// hamowanie nagle: pelne hamowanie od razu, przewod glowny oproznia UpdatePipePressure()
		eimic = -1.0;
	}
	else if (MainCtrlActualPos != MainCtrlPos || LastRelayTime > InitialCtrlDelay)
	{
		double delta = MainCtrlActualPos == MainCtrlPos ? dt * CtrlDelay : 0.01;
		// the positions below are those of the controller without the emergency brake position
		switch (MainCtrlPos - EIMCtrlEmergencyPositions())
		{
		case 0:
		case 1:
			eimic -= safe_clamp(1.0 + eimic, 0.0, delta); // odejmuj do -1
			if (eimic > 0)
				eimic = 0;
			break;
		case 2:
			eimic -= safe_clamp(0.0 + eimic, 0.0, delta); // odejmuj do 0
			break;
		case 3:
			eimic += safe_clamp(0.0 - eimic, 0.0, delta); // dodawaj do 0
			break;
		case 4:
			eimic += safe_clamp(1.0 - eimic, 0.0, delta); // dodawaj do 1
			if (eimic < 0)
				eimic = 0;
			break;
		default:
			break;
		}
	}
	if (MainCtrlActualPos == MainCtrlPos)
		LastRelayTime += dt;
	else
	{
		LastRelayTime = 0;
		MainCtrlActualPos = MainCtrlPos;
	}
}

// EIMCtrlType 3: the universal controller of UniCtrlList, with the train brake built in or not
void TMoverParameters::EIMICFromUniversalController(double dt, double const brakeDemand)
{
	if (UniCtrlIntegratedBrakePNCtrl)
	{
		UniversalControllerTrainBrake();
	}

	if (MainCtrlActualPos != MainCtrlPos || LastRelayTime > InitialCtrlDelay)
	{
		auto const &position{UniCtrlList[MainCtrlPos]};
		double ramped{eimic};
		ramped -= safe_clamp(-position.SetCtrlVal + ramped, 0.0,
		               MainCtrlActualPos == MainCtrlPos ? dt * position.SpeedDown : sign(position.SpeedDown) * 0.01); // odejmuj do X
		ramped += safe_clamp(position.SetCtrlVal - ramped, 0.0,
		               MainCtrlActualPos == MainCtrlPos ? dt * position.SpeedUp : sign(position.SpeedUp) * 0.01); // dodawaj do X
		ramped = safe_clamp(ramped, position.MinCtrlVal, position.MaxCtrlVal);
		eimic = UniCtrlValStep > 0.0 ? stepped_uc_value(eimic, ramped, position, UniCtrlValStep, MainCtrlActualPos != MainCtrlPos, eimic_step_progress) : ramped;
	}
	if (MainCtrlActualPos == MainCtrlPos)
		LastRelayTime += dt;
	else
	{
		LastRelayTime = 0;
		MainCtrlActualPos = MainCtrlPos;
	}
	if (Hamulec->GetEDBCP() > 0.3 && eimic < 0 && !UniCtrlIntegratedLocalBrakeCtrl) // when braking with pneumatic brake
		eimic = 0; // shut off retarder
	if (UniCtrlIntegratedBrakeCtrl == false && UniCtrlIntegratedLocalBrakeCtrl == false)
	{
		// w trybie SplitEDPneumaticBrake hamowanie ED przychodzi z osobnego nastawnika,
		// LocalBrake operuje wylacznie pneumatycznym hamulcem lokomotywy
		eimic = brakeDemand > 0.01 ? -brakeDemand : eimic;
	}
}

// the train brake built into the universal controller: the handle set to the controller's position, or the
// controller, at a brake handle moved alone, to the nearest position of the handle's
void TMoverParameters::UniversalControllerTrainBrake()
{
	if (UniCtrlList[MainCtrlPos].mode != BrakeCtrlPos && MainCtrlActualPos == MainCtrlPos) // there was no move of controller, but brake only
	{
		if (BrakeCtrlPos < UniCtrlList[MainCtrlPosNo].mode)
			BrakeLevelSet(UniCtrlList[MainCtrlPosNo].mode); // bottom clamping
		if (BrakeCtrlPos > UniCtrlList[0].mode)
			BrakeLevelSet(UniCtrlList[0].mode); // top clamping
		if (IsCabMaster())
		{
			while (BrakeCtrlPos > UniCtrlList[MainCtrlPos].mode)
				DecMainCtrl(1); // find nearest position
			while (BrakeCtrlPos < UniCtrlList[MainCtrlPos].mode)
				IncMainCtrl(1); // find nearest position
		}
	}
	else // controller was moved
		BrakeLevelSet(UniCtrlList[MainCtrlPos].mode);
}

// the local brake of the TRAXX locomotives, by time: released, held or applied, which cuts the drive off
void TMoverParameters::EIMICTraxxLocalBrake(double dt)
{
	if (LocalBrakeRatio() < 0.4) // pozycja 0, odhamowywanie
	{
		eim_localbrake -= dt * 0.17;
	}

	if (LocalBrakeRatio() > 0.6) // pozycja 2, zahamowywanie
	{
		eim_localbrake += dt * 0.17;
		eim_localbrake = std::max(eim_localbrake, BrakePress / MaxBrakePress[0]);
	}
	else // pozycja 1, utrzymanie cisnienia
	{
		if (eim_localbrake < Hamulec->GetEDBCP() / MaxBrakePress[0])
			eim_localbrake = 0;
	}
	eim_localbrake = std::clamp(eim_localbrake, 0.0, 1.0);
	if (eim_localbrake > 0.04 && eimic > 0)
		eimic = 0;
}

// the value limited by the power, the spring brake, the pipe lock, the direction and the doors' permits
void TMoverParameters::EIMICLimit()
{
	auto const eimicpowerenabled{(MainCircuitPowered() || Power == 0.0) && (!SpringBrake.IsActive || !SpringBrakeCutsOffDrive) && !LockPipe && DirAbsolute != 0};
	auto const eimicdoorenabled{(SpringBrake.IsActive && ReleaseParkingBySpringBrakeWhenDoorIsOpen)};
	double eimic_max = 0.0;
	if (Doors.instances[side::left].open_permit == false && Doors.instances[side::right].open_permit == false)
	{
		if (eimicpowerenabled)
		{
			eimic_max = 1.0;
		}
		else
		{
			eimic_max = 0.001;
		}
	}
	else
	{
		if (eimicdoorenabled)
		{
			eimic_max = 0.001;
		}
	}
	eimic = safe_clamp(eimic, -1.0, eimicpowerenabled ? eimic_max : 0.0);
}

void TMoverParameters::CheckSpeedCtrl(double dt)
{
	if (EIMCtrlType == 0)
	{
		SpeedCtrlUnit.DesiredPower = std::max(eimic, 0.0);
	}
	double accfactor = SpeedCtrlUnit.DesiredPower;
	if (EIMCtrlType >= 2)
	{
		if (MainCtrlPos < MainCtrlPosNo - 2)
		{
			SpeedCtrlUnit.Standby = true;
		}
		if (MainCtrlPos > MainCtrlPosNo - 1)
		{
			SpeedCtrlUnit.Standby = false;
		}
		if (!SpeedCtrlUnit.BrakeIntervention && (Hamulec->GetEDBCP() > 0.4 || PipePress < HighPipePress - 0.2))
			SpeedCtrlUnit.Standby = true;
		if (EIMCtrlType >= 3 && UniCtrlList[MainCtrlPos].SpeedUp <= 0)
		{
			accfactor = 0.0;
			eimicSpeedCtrl = 0;
		}
	}
	if (SpeedCtrlUnit.IsActive)
	{ // speed control
		if (true)
		{
			if (!SpeedCtrlUnit.Standby)
			{
				SpeedCtrlRegulator(dt, accfactor);
			}
			else
			{
				eimicSpeedCtrl = 0;
				eimicSpeedCtrlIntegral = 0;
			}
			SpeedCtrlUnit.Parking = Vel == 0.0 && eimic <= 0 && EngineType != TEngineType::ElectricInductionMotor;
			SendCtrlToNext(VehicleCommand::speedControlParking, SpeedCtrlUnit.Parking, CabActive);
		}
		else
		{
			if (Vmax < 250)
				eimicSpeedCtrl = std::clamp(0.5 * (SpeedCtrlValue - Vel), -1.0, 1.0);
			else
				eimicSpeedCtrl = std::clamp(0.5 * (SpeedCtrlValue * 2 - Vel), -1.0, 1.0);
		}
		if ((SpeedCtrlAutoTurnOffFlag & 2) == 2 && Hamulec->GetEDBCP() > 0.25)
		{
			SpeedCtrlSwitch(false);
			SpeedCtrlUnit.IsActive = false;
		}
	}
	else
	{
		eimicSpeedCtrl = 1;
		eimicSpeedCtrlIntegral = 0;
		SpeedCtrlUnit.Parking = false;
		SendCtrlToNext(VehicleCommand::speedControlParking, SpeedCtrlUnit.Parking, CabActive);
	}
}

// the PI regulator of the speed control: the power asked of the drive, or its brake intervention, from the error of
// the speed; accfactor, the most power it may ask
void TMoverParameters::SpeedCtrlRegulator(double dt, double const accfactor)
{
		if (SpeedCtrlUnit.ManualStateOverride && eimic > 0.0009)
			eimic = 1.0;
		double error = std::max(SpeedCtrlValue + SpeedCtrlUnit.Offset, 0.0) - Vel;
		double factorP = error > 0 ? SpeedCtrlUnit.FactorPpos : SpeedCtrlUnit.FactorPneg;
		double eSCP = std::clamp(factorP * error, -1.2, 1.0); // P module
		bool retarder_not_work = EngineType != TEngineType::DieselEngine || Vel < SpeedCtrlUnit.BrakeInterventionVel;
		if (eSCP < -1.0)
		{
			SpeedCtrlUnit.BrakeInterventionBraking = eSCP < -1.1 && retarder_not_work && eimicSpeedCtrl < -0.99 * SpeedCtrlUnit.DesiredPower;
			eSCP = -1.0;
		}
		SpeedCtrlUnit.BrakeInterventionUnbraking = eSCP > 0.0 || Vel == 0.0;
		if (abs(eSCP) < 0.999)
		{
			// TODO: check how to disable integral part when braking in smart way
			// double factorI = eimicSpeedCtrlIntegral >= 0 ? SpeedCtrlUnit.FactorIpos : SpeedCtrlUnit.FactorIneg;
			double factorI = eimicSpeedCtrlIntegral >= 0 ? SpeedCtrlUnit.FactorIpos : SpeedCtrlUnit.FactorIneg;
			eimicSpeedCtrlIntegral = safe_clamp(eimicSpeedCtrlIntegral + factorI * eSCP * dt, -1.0 + eSCP, 1.0 - eSCP);
		}
		else
		{
			eimicSpeedCtrlIntegral = 0;
		}
		auto const DesiredeimicSpeedCtrl{safe_clamp(eimicSpeedCtrlIntegral + eSCP, -SpeedCtrlUnit.DesiredPower, accfactor)};
		eimicSpeedCtrl = safe_clamp(DesiredeimicSpeedCtrl, eimicSpeedCtrl - SpeedCtrlUnit.PowerDownSpeed * dt, eimicSpeedCtrl + SpeedCtrlUnit.PowerUpSpeed * dt);
		if (Vel < SpeedCtrlUnit.FullPowerVelocity)
		{
			eimicSpeedCtrl = std::min(eimicSpeedCtrl, SpeedCtrlUnit.InitialPower);
		}
		if (Vel < SpeedCtrlUnit.StartVelocity && MainCtrlPos < MainCtrlPosNo)
		{
			eimicSpeedCtrl = 0;
			eimic = 0;
		}
}

bool TMoverParameters::IsSpeedCtrlSeparate() const
{
	// no secondary controller, or one of these engines', which shunts the field of the traction motors (eu07/maszyna#69)
	return SpeedCtrl && (ScndCtrlPosNo == 0 || EngineType == TEngineType::ElectricSeriesMotor || EngineType == TEngineType::DieselElectric);
}

bool TMoverParameters::IsSpeedCtrlEngaged() const
{
	return IsSpeedCtrlSeparate() ? SpeedCtrlUnit.IsActive : ScndCtrlPos > 0;
}

bool TMoverParameters::SpeedCtrlSwitch(bool const State)
{
	if (false == IsSpeedCtrlSeparate())
	{
		return State ? IncScndCtrl(1) : DecScndCtrl(2);
	}
	// its own switch, the secondary controller left to the field shunting
	if (State)
	{
		if (CabActive == 0 || SpeedCtrlUnit.IsActive)
		{
			return false;
		}
		SpeedCtrlValue = static_cast<double>(Round(Vel));
		SpeedCtrlUnit.IsActive = true;
		return true;
	}
	auto const wasactive{SpeedCtrlUnit.IsActive};
	SpeedCtrlValue = 0;
	SpeedCtrlUnit.IsActive = false;
	if (wasactive && SpeedCtrlUnit.ManualStateOverride)
	{
		eimic = 0.0;
	}
	return wasactive;
}

void TMoverParameters::SpeedCtrlButton(int button)
{
	if (SpeedCtrl && IsSpeedCtrlEngaged())
	{
		SpeedCtrlValue = SpeedCtrlButtons[button];
	}
}

void TMoverParameters::SpeedCtrlInc()
{
	if (SpeedCtrl && IsSpeedCtrlEngaged())
	{
		double x = floor(SpeedCtrlValue / SpeedCtrlUnit.VelocityStep) + 1.0;
		SpeedCtrlValue = std::min(x * SpeedCtrlUnit.VelocityStep, SpeedCtrlUnit.MaxVelocity);
	}
}

void TMoverParameters::SpeedCtrlDec()
{
	if (SpeedCtrl && IsSpeedCtrlEngaged())
	{
		double x = ceil(SpeedCtrlValue / SpeedCtrlUnit.VelocityStep) - 1.0;
		SpeedCtrlValue = std::max(x * SpeedCtrlUnit.VelocityStep, SpeedCtrlUnit.MinVelocity);
	}
}

bool TMoverParameters::SpeedCtrlPowerInc()
{
	if (!(SpeedCtrl && IsSpeedCtrlEngaged()))
		return false;

	if (SpeedCtrlUnit.DesiredPower == SpeedCtrlUnit.MaxPower)
		return false;

	SpeedCtrlUnit.DesiredPower = std::min(SpeedCtrlUnit.DesiredPower + SpeedCtrlUnit.PowerStep, SpeedCtrlUnit.MaxPower);
	return true;
}

bool TMoverParameters::SpeedCtrlPowerDec()
{
	if (!(SpeedCtrl && IsSpeedCtrlEngaged()))
		return false;

	if (SpeedCtrlUnit.DesiredPower == SpeedCtrlUnit.MinPower)
		return false;

	SpeedCtrlUnit.DesiredPower = std::max(SpeedCtrlUnit.DesiredPower - SpeedCtrlUnit.PowerStep, SpeedCtrlUnit.MinPower);
	return true;
}
