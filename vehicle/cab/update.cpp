/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/
/*
    MaSzyna EU07 locomotive simulator
    Copyright (C) 2001-2004  Marcin Wozniak, Maciej Czapkiewicz and others

*/

// the update of the cab in a step of the simulation

#include "stdafx.h"
#include "vehicle/Train.h"
#include "vehicle/horn.h"
#include "vehicle/linebreaker.h"
#include "vehicle/holdcoillever.h"
#include "vehicle/compartmentlightsswitch.h"
#include "vehicle/odometer.h"
#include "vehicle/mainctrlact.h"
#include "utilities/Globals.h"
#include "simulation/simulation.h"
#include "simulation/blackboxrecorder.h"
#include "world/Event.h"
#include "simulation/simulationtime.h"
#include "vehicle/Camera.h"
#include "utilities/Logs.h"
#include "model/MdlMngr.h"
#include "model/Model3d.h"
#include "utilities/Timer.h"
#include "vehicle/Driver.h"
#include "vehicle/DynObj.h"
#include "world/mtable.h"
#include "Console.h"
#include "application/application.h"
#include "rendering/renderer.h"
#include "physics/mover.h"
#include <future>
#include <thread>
#include <cmath>
#include <algorithm>
#include <format>
#include "vehicle/cab/internal.h"


bool TTrain::Update(double const Deltatime)
{
	// train state update
	// line breaker:
	if (m_linebreakerstate == linebreaker::state::open && true == mvControlled->Mains)
	{
		// crude way to sync state of the linebreaker with ai-issued commands
		m_linebreakerstate = linebreaker::state::closed;
	}
	if (m_linebreakerstate == linebreaker::state::closed && false == (mvControlled->Mains || mvControlled->dizel_startup))
	{
		// crude way to catch cases where the main was knocked out
		// because the state of the line breaker isn't changed to match, we need to do it here manually
		m_linebreakerstate = linebreaker::state::open;
	}
	if (is_hold_coil_lever(ggMainOffButton) && hold_coil_lever::cuts_off(ggMainOffButton.GetDesiredValue()))
	{
		if (true == mvControlled->Mains)
		{
			// the line breaker was closed with the hold coil lever off, by the AI or through the control cable:
			// put the lever back on to match, like the line breaker state above
			ggMainOffButton.UpdateValue(0.0);
		}
		else if (m_linebreakerstate == linebreaker::state::closing)
		{
			// the hold coil was cut off while the closing button was held, the line breaker won't close on its release
			m_linebreakerstate = linebreaker::state::open;
		}
	}

	if ((ggMainButton.SubModel != nullptr && ggMainButton.GetDesiredValue() > 0.95) ||
	    ggMainOnButton.SubModel != nullptr && ggMainOnButton.GetDesiredValue() > 0.95 ||
	    ggIgnitionKey.GetDesiredValue() > 0.95)
	{ // HACK: fallback
		// keep track of period the line breaker button is held down, to determine when/if circuit closes
		if (mvControlled->MainSwitchCheck())
		{
			fMainRelayTimer += static_cast<float>(Deltatime);
		}
	}
	else
	{
		// button isn't down, reset the timer
		fMainRelayTimer = 0.0f;
	}
	if (ggMainOffButton.GetDesiredValue() > 0.95)
	{
		// if the button disconnecting the line breaker is down prevent the timer from accumulating
		fMainRelayTimer = 0.0f;
	}
	if (m_linebreakerstate == linebreaker::state::open && fMainRelayTimer > mvControlled->InitialCtrlDelay)
	{
		// wlaczanie WSa z opoznieniem
		// mark the line breaker as ready to close; for electric series vehicles with impulse switch the setup is completed on button release
		m_linebreakerstate = linebreaker::state::closing;
	}
	// for diesels and/or vehicles with toggle switch setup we complete the engine start here
	// TODO: make it a test for main_on_bt of type push_delayed instead
	if (m_linebreakerstate == linebreaker::state::closing && (ggMainOnButton.SubModel == nullptr || mvControlled->EngineType != TEngineType::ElectricSeriesMotor))
	{
		// try to finalize state change of the line breaker, set the state based on the outcome
		m_linebreakerstate = mvControlled->MainSwitch(true, mvControlled->MainSwitchRange()) ? linebreaker::state::closed : linebreaker::state::open;
	}
	// the switches of the other vehicles in multiple
	update_remote_linebreakers(Deltatime);
	// door permits
	for (auto idx = 0; idx < 2; ++idx)
	{
		auto &doorpermittimer{m_doorpermittimers[idx]};
		if (doorpermittimer < 0.f)
		{
			continue;
		}
		doorpermittimer -= static_cast<float>(Deltatime);
		if (doorpermittimer < 0.f)
		{
			mvOccupied->OperateDoors(static_cast<side>(idx), true);
		}
	}

	// train measurement timer
	if (trainLenghtMeasureTimer >= 0.f)
	{
		trainLenghtMeasureTimer -= static_cast<float>(Deltatime);
		if (trainLenghtMeasureTimer < 0.f)
			trainLenghtMeasureTimer = -1.f;
	}

	// battery timer
	if (fBatteryTimer >= 0.f)
	{
		fBatteryTimer -= static_cast<float>(Deltatime);
		if (fBatteryTimer < 0.f)
			fBatteryTimer = -1.f;
	}

	// helper variables
	if (DynamicObject->Mechanik != nullptr)
	{
		m_doors = DynamicObject->Mechanik->IsAnyDoorOpen[side::right] || DynamicObject->Mechanik->IsAnyDoorOpen[side::left];
		m_doorpermits = DynamicObject->Mechanik->IsAnyDoorPermitActive[side::right] || DynamicObject->Mechanik->IsAnyDoorPermitActive[side::left];
		m_doorspermitleft = mvOccupied->Doors.instances[cab_to_end() == end::front ? side::left : side::right].open_permit &&
		                    (simulation::Time.data().wSecond % 2 < 1 || mvOccupied->DoorsPermitLightBlinking < 1 ||
		                     mvOccupied->DoorsPermitLightBlinking < 2 && DynamicObject->Mechanik->IsAnyDoorOpen[cab_to_end() == end::front ? side::left : side::right] ||
		                     (mvOccupied->DoorsPermitLightBlinking < 3 && DynamicObject->Mechanik->IsAnyDoorOnlyOpen[cab_to_end() == end::front ? side::left : side::right]));
		m_doorspermitright = mvOccupied->Doors.instances[cab_to_end() == end::front ? side::right : side::left].open_permit &&
		                     (simulation::Time.data().wSecond % 2 < 1 || mvOccupied->DoorsPermitLightBlinking < 1 ||
		                      mvOccupied->DoorsPermitLightBlinking < 2 && DynamicObject->Mechanik->IsAnyDoorOpen[cab_to_end() == end::front ? side::right : side::left] ||
		                      (mvOccupied->DoorsPermitLightBlinking < 3 && DynamicObject->Mechanik->IsAnyDoorOnlyOpen[cab_to_end() == end::front ? side::right : side::left]));
	}
	m_dirforward = mvControlled->DirActive > 0;
	m_dirneutral = mvControlled->DirActive == 0;
	m_dirbackward = mvControlled->DirActive < 0;

	// check for received user commands
	// NOTE: this is a temporary arrangement, for the transition period from old command setup to the new one
	// eventually commands are going to be retrieved directly by the vehicle, filtered through active control stand
	// and ultimately executed, provided the stand allows it.
	command_data commanddata;
	while (simulation::Commands.pop(commanddata, static_cast<std::size_t>(command_target::vehicle) | id()))
	{
		simulation::Blackbox.command(DynamicObject, commanddata.command, commanddata.action, commanddata.param1, commanddata.param2);

		auto lookup = m_commandhandlers.find(commanddata.command);
		if (lookup != m_commandhandlers.end())
		{
			// debug data
			if (commanddata.action == GLFW_PRESS)
			{
				WriteLog(mvOccupied->Name + " received command: [" + simulation::Commands_descriptions[static_cast<std::size_t>(commanddata.command)].name + "]");
			}
			// pass the command to the assigned handler
			lookup->second(this, commanddata);
		}
	}

	UpdateCab();

	// nie blokujemy AI
	if (DynamicObject->Mechanik != nullptr && false == DynamicObject->Mechanik->AIControllFlag && (mvOccupied->TrainType == train_type::et40 || mvOccupied->TrainType == train_type::ep05 || mvOccupied->HasCamshaft))
	{
		// dla ET40 i EU05 automatyczne cofanie nastawnika - i tak nie będzie to działać dobrze...
		// TODO: use deltatime to stabilize speed
		/*
		            if( false == (
		                ( input::command == user_command::mastercontrollerset )
		                || ( input::command == user_command::mastercontrollerincrease )
		                || ( input::command == user_command::mastercontrollerdecrease ) ) ) {
		*/
		if (false == (m_mastercontrollerinuse || Global.ctrlState))
		{
			m_mastercontrollerreturndelay -= static_cast<float>(Deltatime);
			if (m_mastercontrollerreturndelay < 0.f)
			{
				m_mastercontrollerreturndelay = EU07_CONTROLLER_BASERETURNDELAY;
				if (mvOccupied->MainCtrlPos > mvOccupied->MainCtrlActualPos)
				{
					mvOccupied->DecMainCtrl(1);
				}
				else if (mvOccupied->MainCtrlPos < mvOccupied->MainCtrlActualPos)
				{
					// Ra 15-01: a to nie miało być tylko cofanie?
					mvOccupied->IncMainCtrl(1);
				}
			}
		}
	}

	// McZapkie: predkosc wyswietlana na tachometrze brana jest z obrotow kol
	auto const maxtacho{3.0};

	double maxSpeed = mvControlled->Vmax * 1.05; // zachowanie starej logiki jak nie ma definicji max tarczki
	if (mvOccupied->maxTachoSpeed != 0)
	{
		maxSpeed = mvOccupied->maxTachoSpeed;
	}
	fTachoVelocity = static_cast<float>(std::min(std::abs(11.31 * mvControlled->WheelDiameter * mvControlled->nrot), maxSpeed));
	{ // skacze osobna zmienna
		float ff = simulation::Time.data().wSecond; // skacze co sekunde - pol sekundy
		// pomiar, pol sekundy ustawienie
		if (ff != fTachoTimer) // jesli w tej sekundzie nie zmienial
		{
			if (fTachoVelocity >= 5) // jedzie
				fTachoVelocityJump = static_cast<float>(fTachoVelocity + (2.0 - LocalRandom(3) + LocalRandom(3)) * 0.5);
			else if (fTachoVelocity < 5 && fTachoVelocity > 1)
				fTachoVelocityJump = static_cast<float>(Random(0, 4)); // tu ma sie bujac jak wariat i zatrzymac na jakiejs predkosci
			// fTachoVelocityJump = 0; // stoi
			fTachoTimer = ff; // juz zmienil
		}
	}
	if (fTachoVelocity > 1) // McZapkie-270503: podkrecanie tachometru
	{
		// szybciej zacznij stukac
		fTachoCount = static_cast<float>(std::min(maxtacho, fTachoCount + Deltatime * 3));
	}
	else if (fTachoCount > 0)
	{
		// schodz powoli - niektore haslery to ze 4 sekundy potrafia stukac
		fTachoCount = static_cast<float>(std::max(0.0, fTachoCount - Deltatime * 0.66));
	}

	// Ra 2014-09: napięcia i prądy muszą być ustalone najpierw, bo wysyłane są ewentualnie na PoKeys
	if (mvControlled->EngineType != TEngineType::DieselElectric && mvControlled->EngineType != TEngineType::ElectricInductionMotor)
	{ // Ra 2014-09: czy taki rozdzia? ma sens?
		fHVoltage = static_cast<float>(std::max(mvControlled->PantographVoltage,
		                     mvControlled->GetTrainsetHighVoltage())); // Winger czy to nie jest zle?
	}
	// *mvControlled->Mains);
	else
	{
		fHVoltage = static_cast<float>(mvControlled->EngineVoltage);
	}
	if (ShowNextCurrent)
	{ // jeśli pokazywać drugi człon
		if (mvSecond)
		{ // o ile jest ten drugi
			fHCurrent[0] = static_cast<float>(mvSecond->ShowCurrent(0) * 1.05);
			fHCurrent[1] = static_cast<float>(mvSecond->ShowCurrent(1) * 1.05);
			fHCurrent[2] = static_cast<float>(mvSecond->ShowCurrent(2) * 1.05);
			fHCurrent[3] = static_cast<float>(mvSecond->ShowCurrent(3) * 1.05);
		}
		else
		{
			fHCurrent[0] = 0.0; // gdy nie ma człona
			fHCurrent[1] = 0.0;
			fHCurrent[2] = 0.0;
			fHCurrent[3] = 0.0;
		}
	}
	else
	{ // normalne pokazywanie
		fHCurrent[0] = static_cast<float>(mvControlled->ShowCurrent(0));
		fHCurrent[1] = static_cast<float>(mvControlled->ShowCurrent(1));
		fHCurrent[2] = static_cast<float>(mvControlled->ShowCurrent(2));
		fHCurrent[3] = static_cast<float>(mvControlled->ShowCurrent(3));
	}

	bool kier = DynamicObject->DirectionGet() * mvOccupied->CabOccupied > 0;
	TDynamicObject *p = DynamicObject->GetFirstDynamic(mvOccupied->CabOccupied < 0 ? end::rear : end::front, Coupling::control);
	int in = 0;
	fEIMParams[0][6] = 0;
	iCarNo = 0;
	iPowerNo = 0;
	iUnitNo = 1;

	for (int i = 0; i < 8; i++)
	{
		bMains[i] = false;
		fCntVol[i] = 0.0f;
		bPants[i][0] = false;
		bPants[i][1] = false;
		bFuse[i] = false;
		bBatt[i] = false;
		bConv[i] = false;
		bComp[i][0] = false;
		bComp[i][1] = false;
		// bComp[i][2] = false;
		// bComp[i][3] = false;
		bHeat[i] = false;
	}
	bCompressors.clear();
	for (int i = 0; i < 20; i++)
	{
		if (p)
		{
			fPress[i][0] = static_cast<float>(p->MoverParameters->BrakePress);
			fPress[i][1] = static_cast<float>(p->MoverParameters->PipePress);
			fPress[i][2] = static_cast<float>(p->MoverParameters->ScndPipePress);
			fPress[i][3] = static_cast<float>(p->MoverParameters->CntrlPipePress);
			fPress[i][4] = static_cast<float>(p->MoverParameters->Hamulec->GetBRP());
			fPress[i][5] = static_cast<float>((p->MoverParameters->TotalMass - p->MoverParameters->Mred) * 0.001);
			fPress[i][6] = static_cast<float>(p->MoverParameters->SpringBrake.SBP);
			bBrakes[i][0] = p->MoverParameters->SpringBrake.IsActive;
			bBrakes[i][1] = p->MoverParameters->SpringBrake.ShuttOff;
			bDoors[i][1] = p->MoverParameters->Doors.instances[side::left].position > 0.f;
			bDoors[i][2] = p->MoverParameters->Doors.instances[side::right].position > 0.f;
			bDoors[i][3] = p->MoverParameters->Doors.instances[side::left].step_position > 0.f;
			bDoors[i][4] = p->MoverParameters->Doors.instances[side::right].step_position > 0.f;
			bDoors[i][0] = bDoors[i][1] || bDoors[i][2];
			iDoorNo[i] = p->iAnimType[ANIM_DOORS];
			iUnits[i] = iUnitNo;
			cCode[i] = p->MoverParameters->TypeName[p->MoverParameters->TypeName.length() - 1];
			asCarName[i] = p->name();
			if (p->MoverParameters->EnginePowerSource.SourceType == TPowerSource::CurrentCollector)
			{
				bPants[iUnitNo - 1][end::front] = bPants[iUnitNo - 1][end::front] || p->MoverParameters->Pantographs[end::front].is_active;
				bPants[iUnitNo - 1][end::rear] = bPants[iUnitNo - 1][end::rear] || p->MoverParameters->Pantographs[end::rear].is_active;
			}
			// TBD, TODO: clean up compressor data arrangement?
			if (iUnitNo <= 8)
			{
				bComp[iUnitNo - 1][0] = bComp[iUnitNo - 1][0] || p->MoverParameters->CompressorAllow || p->MoverParameters->CompressorStart == start_t::automatic;
			}
			if (p->MoverParameters->CompressorSpeed > 0.00001)
			{
				if (iUnitNo <= 8)
				{
					bComp[iUnitNo - 1][1] = bComp[iUnitNo - 1][1] || p->MoverParameters->CompressorFlag;
				}
				bCompressors.emplace_back(p->MoverParameters->CompressorAllow || p->MoverParameters->CompressorStart == start_t::automatic, p->MoverParameters->CompressorFlag, i);
			}
			bSlip[i] = p->MoverParameters->SlippingWheels;
			if (in < 8 && p->MoverParameters->eimc[eimc_p_Pmax] > 1)
			{
				fEIMParams[1 + in][0] = static_cast<float>(p->MoverParameters->eimv[eimv_Fmax]);
				fEIMParams[1 + in][1] = std::max(fEIMParams[1 + in][0], 0.f);
				fEIMParams[1 + in][2] = -std::min(fEIMParams[1 + in][0], 0.f);
				fEIMParams[1 + in][3] = static_cast<float>(p->MoverParameters->eimv[eimv_Fmax] / std::max(p->MoverParameters->eimv[eimv_Fful], 1.));
				fEIMParams[1 + in][4] = std::max(fEIMParams[1 + in][3], 0.f);
				fEIMParams[1 + in][5] = -std::min(fEIMParams[1 + in][3], 0.f);
				fEIMParams[1 + in][6] = static_cast<float>(p->MoverParameters->eimv[eimv_If]);
				fEIMParams[1 + in][7] = static_cast<float>(p->MoverParameters->eimv[eimv_U]);
				fEIMParams[1 + in][8] = static_cast<float>(p->MoverParameters->Itot); // p->MoverParameters->eimv[eimv_Ipoj];
				fEIMParams[1 + in][9] = static_cast<float>(p->MoverParameters->EngineVoltage);
				fEIMParams[0][6] += fEIMParams[1 + in][8];
				bMains[in] = p->MoverParameters->Mains;
				fCntVol[in] = static_cast<float>(p->MoverParameters->BatteryVoltage);
				bFuse[in] = p->MoverParameters->FuseFlag;
				bBatt[in] = p->MoverParameters->Battery;
				bConv[in] = p->MoverParameters->ConverterFlag;
				bHeat[in] = p->MoverParameters->Heating;
				// bComp[in][2] = (p->MoverParameters->CompressorAllow || (p->MoverParameters->CompressorStart == start_t::automatic));
				// bComp[in][3] = (p->MoverParameters->CompressorFlag);
				in++;
				iPowerNo = in;
			}
			if (in < 8 && (p->MoverParameters->EngineType == TEngineType::DieselEngine || p->MoverParameters->EngineType == TEngineType::DieselElectric))
			{
				fDieselParams[1 + in][0] = static_cast<float>(p->MoverParameters->enrot * 60);
				fDieselParams[1 + in][1] = static_cast<float>(p->MoverParameters->nrot);
				fDieselParams[1 + in][2] = static_cast<float>(p->MoverParameters->RList[p->MoverParameters->MainCtrlPos].R);
				fDieselParams[1 + in][3] = static_cast<float>(p->MoverParameters->dizel_fill);
				fDieselParams[1 + in][4] = static_cast<float>(p->MoverParameters->RList[p->MoverParameters->MainCtrlPos].Mn);
				fDieselParams[1 + in][5] = static_cast<float>(p->MoverParameters->dizel_engage);
				fDieselParams[1 + in][6] = p->MoverParameters->dizel_heat.Twy;
				fDieselParams[1 + in][7] = p->MoverParameters->OilPump.pressure;
				fDieselParams[1 + in][8] = p->MoverParameters->dizel_heat.Ts;
				fDieselParams[1 + in][9] = static_cast<float>(p->MoverParameters->hydro_R_Fill);
				bMains[in] = p->MoverParameters->Mains;
				fCntVol[in] = static_cast<float>(p->MoverParameters->BatteryVoltage);
				bFuse[in] = p->MoverParameters->FuseFlag;
				bBatt[in] = p->MoverParameters->Battery;
				bConv[in] = p->MoverParameters->ConverterFlag;
				bHeat[in] = p->MoverParameters->Heating;
				in++;
				iPowerNo = in;
			}
			if ((kier ? p->Next(Coupling::permanent) : p->Prev(Coupling::permanent)) != (kier ? p->Next(Coupling::control) : p->Prev(Coupling::control)))
				iUnitNo++;
			p = kier ? p->Next(Coupling::control) : p->Prev(Coupling::control);
			iCarNo = i + 1;
		}
		else
		{
			fPress[i][0] = 0;
			fPress[i][1] = 0;
			fPress[i][2] = 0;
			fPress[i][3] = 0;
			fPress[i][4] = 0;
			fPress[i][5] = 0;
			bDoors[i][0] = false;
			bDoors[i][1] = false;
			bDoors[i][2] = false;
			bDoors[i][3] = false;
			bDoors[i][4] = false;
			bBrakes[i][0] = false;
			bBrakes[i][1] = false;
			bSlip[i] = false;
			iUnits[i] = 0;
			cCode[i] = 0; //'0';
			asCarName[i] = "";
		}
	}

	//        if (mvControlled == mvOccupied)
	//            fEIMParams[0][3] = mvControlled->eimv[eimv_Fzad]; // procent zadany
	//        else
	//            fEIMParams[0][3] =
	//                mvControlled->eimv[eimv_Fzad] - mvOccupied->LocalBrakeRatio(); // procent zadany
	fEIMParams[0][3] = static_cast<float>(mvOccupied->eimic_real);
	fEIMParams[0][4] = std::max(fEIMParams[0][3], 0.f);
	fEIMParams[0][5] = -std::min(fEIMParams[0][3], 0.f);
	fEIMParams[0][1] = static_cast<float>(fEIMParams[0][4] * mvControlled->eimv[eimv_Fful]);
	fEIMParams[0][2] = static_cast<float>(fEIMParams[0][5] * mvControlled->eimv[eimv_Fful]);
	fEIMParams[0][0] = fEIMParams[0][1] - fEIMParams[0][2];
	fEIMParams[0][7] = 0;
	fEIMParams[0][8] = 0;
	fEIMParams[0][9] = 0;

	for (int i = in; i < 8; i++)
	{
		for (int j = 0; j <= 9; j++)
		{
			fEIMParams[1 + i][j] = 0;
			fDieselParams[1 + i][j] = 0;
		}
	}
#ifdef _WIN32
	if (Global.iFeedbackMode == 4)
	{
		// wykonywać tylko gdy wyprowadzone na pulpit
		// Ra: sterowanie miernikiem: zbiornik główny
		Console::ValueSet(0, mvOccupied->Compressor);
		// Ra: sterowanie miernikiem: przewód główny
		Console::ValueSet(1, mvOccupied->PipePress);
		// Ra: sterowanie miernikiem: cylinder hamulcowy
		Console::ValueSet(2, mvOccupied->BrakePress);
		// woltomierz wysokiego napięcia
		Console::ValueSet(3, fHVoltage);
		// Ra: sterowanie miernikiem: drugi amperomierz
		Console::ValueSet(4, fHCurrent[2]);
		// pierwszy amperomierz; dla EZT prąd całkowity
		Console::ValueSet(5, fHCurrent[mvControlled->TrainType == train_type::emu ? 0 : 1]);
		// Ra: prędkość na pin 43 - wyjście analogowe (to nie jest PWM); skakanie zapewnia mechanika napędu
		Console::ValueSet(6, fTachoVelocity);
	}
#endif
	//------------------
	// hunter-261211: nadmiarowy przetwornicy i ogrzewania
	// Ra 15-01: to musi stąd wylecieć - zależności nie mogą być w kabinie
	if (mvControlled->ConverterFlag == true)
	{
		fConverterTimer += static_cast<float>(Deltatime);
		if (mvControlled->CompressorFlag == true && mvControlled->CompressorPower == 1 && (mvControlled->EngineType == TEngineType::ElectricSeriesMotor || mvControlled->TrainType == train_type::emu) &&
		    DynamicObject->Controller == Humandriver // hunter-110212: poprawka dla EZT
		    && false == DynamicObject->Mechanik->AIControllFlag)
		{ // hunter-091012: poprawka (zmiana warunku z CompressorPower /rozne od 0/ na /rowne 1/)
			if (fConverterTimer < fConverterPrzekaznik)
			{
				mvControlled->ConvOvldFlag = true;
				if (mvControlled->TrainType != train_type::emu)
					mvControlled->MainSwitch(false, mvControlled->TrainType == train_type::emu ? range_t::unit : range_t::local);
			}
			else if (fConverterTimer >= fConverterPrzekaznik)
			{
				// changed switch from always true to take into account state of the compressor switch
				mvControlled->CompressorSwitch(mvControlled->CompressorAllow);
			}
		}
	}
	else
		fConverterTimer = 0;
	//------------------
	auto const lowvoltagepower{mvOccupied->Power24vIsAvailable || mvOccupied->Power110vIsAvailable};

	// youBy - prad w drugim czlonie: galaz lub calosc
	{
		TDynamicObject const *tmp{nullptr};
		if (DynamicObject->NextConnected() && mvControlled->Couplers[end::rear].CouplingFlag.HasAll(Coupling::control) && mvOccupied->CabOccupied == 1)
			tmp = DynamicObject->NextConnected();
		if (DynamicObject->PrevConnected() && mvControlled->Couplers[end::front].CouplingFlag.HasAll(Coupling::control) && mvOccupied->CabOccupied == -1)
			tmp = DynamicObject->PrevConnected();
		if (tmp && tmp->MoverParameters->Power > 0)
		{
			if (ggI1B.SubModel)
			{
				ggI1B.UpdateValue(static_cast<float>(tmp->MoverParameters->ShowCurrent(1)));
				ggI1B.Update();
			}
			if (ggI2B.SubModel)
			{
				ggI2B.UpdateValue(static_cast<float>(tmp->MoverParameters->ShowCurrent(2)));
				ggI2B.Update();
			}
			if (ggI3B.SubModel)
			{
				ggI3B.UpdateValue(static_cast<float>(tmp->MoverParameters->ShowCurrent(3)));
				ggI3B.Update();
			}
			if (ggItotalB.SubModel)
			{
				ggItotalB.UpdateValue(static_cast<float>(tmp->MoverParameters->ShowCurrent(0)));
				ggItotalB.Update();
			}
			if (ggWater1TempB.SubModel)
			{
				ggWater1TempB.UpdateValue(tmp->MoverParameters->dizel_heat.temperatura1);
				ggWater1TempB.Update();
			}
			if (ggOilPressB.SubModel)
			{
				ggOilPressB.UpdateValue(tmp->MoverParameters->OilPump.pressure);
				ggOilPressB.Update();
			}
		}
	}
	// McZapkie-300302: zegarek
	if (ggClockMInd.SubModel)
	{
		ggClockSInd.UpdateValue(simulation::Time.data().wSecond);
		ggClockSInd.Update();
		ggClockMInd.UpdateValue(simulation::Time.data().wMinute);
		ggClockMInd.Update();
		ggClockHInd.UpdateValue(static_cast<float>(simulation::Time.data().wHour + simulation::Time.data().wMinute / 60.0));
		ggClockHInd.Update();
	}

	Cabine[iCabn].Update(lowvoltagepower); // nowy sposób ustawienia animacji
	/*
	    if (ggZbS.SubModel)
	    {
	        ggZbS.UpdateValue(mvOccupied->Handle->GetCP());
	        ggZbS.Update();
	    }
	*/
	// replacement for the above. TODO: move it to a more suitable place
	m_brakehandlecp = mvOccupied->Handle->GetCP();

	// youBy - napiecie na silnikach
	if (ggEngineVoltage.SubModel)
	{
		if (mvControlled->DynamicBrakeFlag)
		{
			ggEngineVoltage.UpdateValue(static_cast<float>(std::abs(mvControlled->Im * 5)));
		}
		else
		{
			int x;
			if (mvControlled->TrainType == train_type::et42 && mvControlled->Imax == mvControlled->ImaxHi)
				x = 1;
			else
				x = 2;
			if (mvControlled->RList[mvControlled->MainCtrlActualPos].Mn > 0 && std::abs(mvControlled->Im) > 0)
			{
				ggEngineVoltage.UpdateValue(static_cast<float>(x * (std::abs(mvControlled->EngineVoltage) - mvControlled->RList[mvControlled->MainCtrlActualPos].R * std::abs(mvControlled->Im)) /
				                            mvControlled->RList[mvControlled->MainCtrlActualPos].Mn));
			}
			else
			{
				ggEngineVoltage.UpdateValue(0);
			}
		}
		ggEngineVoltage.Update();
	}

	// Winger 140404 - woltomierz NN
	if (ggLVoltage.SubModel)
	{
		// NOTE: since we don't have functional converter object, we're faking it here by simple check whether converter is on
		// TODO: implement object-based circuits and power systems model so we can have this working more properly
		ggLVoltage.UpdateValue(static_cast<float>(std::max(mvOccupied->Power110vIsAvailable ? mvOccupied->NominalBatteryVoltage : 0.0, mvOccupied->Power24vIsAvailable ? mvOccupied->BatteryVoltage : 0.0)));
		ggLVoltage.Update();
	}

	if (mvControlled->EngineType == TEngineType::DieselElectric)
	{ // ustawienie zmiennych dla silnika spalinowego
		fEngine[1] = static_cast<float>(mvControlled->ShowEngineRotation(1));
		fEngine[2] = static_cast<float>(mvControlled->ShowEngineRotation(2));
	}

	else if (mvControlled->EngineType == TEngineType::DieselEngine)
	{ // albo dla innego spalinowego
		fEngine[1] = static_cast<float>(mvControlled->ShowEngineRotation(1));
		fEngine[2] = static_cast<float>(mvControlled->ShowEngineRotation(2));
		fEngine[3] = static_cast<float>(mvControlled->ShowEngineRotation(3));
		if (ggMainGearStatus.SubModel)
		{
			if (mvControlled->Mains)
				ggMainGearStatus.UpdateValue(static_cast<float>(1.1 - std::abs(mvControlled->dizel_automaticgearstatus)));
			else
				ggMainGearStatus.UpdateValue(0.0);
			ggMainGearStatus.Update();
		}
		if (ggIgnitionKey.SubModel && ggIgnitionKey.GetDesiredValue() == 0.0)
		{
			ggIgnitionKey.UpdateValue(mvControlled->Mains || mvControlled->dizel_startup || fMainRelayTimer > 0.f ||
			                          (ggMainButton.SubModel != nullptr && ggMainButton.GetDesiredValue() > 0.95) ||
			                          (ggMainOnButton.SubModel != nullptr && ggMainOnButton.GetDesiredValue() > 0.95));
		}
		ggIgnitionKey.Update();
	}

	if (mvControlled->SlippingWheels)
	{
		// Ra 2014-12: lokomotywy 181/182 dostają SlippingWheels po zahamowaniu powyżej 2.85 bara i buczały
		double veldiff = (DynamicObject->GetVelocity() - fTachoVelocity) / mvControlled->Vmax;
		// 1% Vmax rezerwy, żeby 181/182 nie buczały po zahamowaniu, ale to proteza
		if (veldiff < -0.01 && std::abs(mvControlled->Im) > 10.0)
		{
			btLampkaPoslizg.Turn(true);
		}
	}
	else
	{
		btLampkaPoslizg.Turn(false);
	}

	// Lampka pracujacej sprezacki
	if (mvControlled->CompressorFlag || mvOccupied->CompressorFlag)
		btCompressors.Turn(true);
	else
		btCompressors.Turn(false);

	// Lampka zezwolenia na hamowanie ED
	if (mvControlled->EpFuse)
		btEDenabled.Turn(true);
	else
		btEDenabled.Turn(false);

	// Lampka aktywowanej kabiny
	if (mvControlled->CabActive != 0)
	{
		btCabActived.Turn(true);
	}
	else
	{
		btCabActived.Turn(false);
	}

	if (mvControlled->Battery && mvControlled->CabActive != 0)
		btAKLVents.Turn(true);
	else
		btAKLVents.Turn(false);

	if (true == lowvoltagepower)
	{
		// McZapkie-141102: SHP i czuwak, TODO: sygnalizacja kabinowa
		if (mvOccupied->SecuritySystem.is_vigilance_blinking())
		{
			if (fBlinkTimer > fCzuwakBlink)
				fBlinkTimer = -fCzuwakBlink;
			else
				fBlinkTimer += Deltatime;

			btLampkaCzuwaka.Turn(fBlinkTimer > 0);
		}
		else
		{
			fBlinkTimer = 0.0;
			btLampkaCzuwaka.Turn(false);
		}

		btLampkaSHP.Turn(mvOccupied->SecuritySystem.is_cabsignal_blinking());
		btLampkaCzuwakaSHP.Turn(btLampkaSHP.GetValue() || btLampkaCzuwaka.GetValue());

		btLampkaWylSzybki.Turn(m_linebreakerstate == linebreaker::state::closing || mvControlled->Mains);
		btLampkaWylSzybkiOff.Turn(m_linebreakerstate != linebreaker::state::closing && !mvControlled->Mains);
		btLampkaMainBreakerReady.Turn(mvControlled->MainsInitTimeCountdown <= 0.0 && m_linebreakerstate != linebreaker::state::closing && !mvControlled->Mains);
		btLampkaMainBreakerBlinkingIfReady.Turn(m_linebreakerstate == linebreaker::state::closing || mvControlled->Mains || mvControlled->MainsInitTimeCountdown < 0.0 && simulation::Time.data().wMilliseconds > 500);

		btLampkaPrzetw.Turn(mvOccupied->Power110vIsAvailable);
		btLampkaPrzetwOff.Turn(!mvOccupied->Power110vIsAvailable);
		btLampkaPrzetwAOff.Turn(!mvControlled->ConverterFlag);
		btLampkaNadmPrzetw.Turn(Dynamic()->Mechanik ? Dynamic()->Mechanik->IsAnyConverterOverloadRelayOpen : mvControlled->ConvOvldFlag);

		btLampkaOpory.Turn(mvControlled->StLinFlag && mvControlled->ResistorsFlagCheck());
		btLampkaBezoporowa.Turn(mvControlled->ResistorsFlagCheck() || mvControlled->MainCtrlActualPos == 0); // do EU04
		btLampkaStyczn.Turn(mvControlled->LineContactorsLampLit()); // mozna prowadzic rozruch
		btLampkaPrzekRozn.Turn(!mvControlled->GroundRelay && mvControlled->MainCircuitLampsEnabled()); // relay is off and needs a reset
		btLampkaNadmSil.Turn(mvControlled->FuseFlagCheck() && mvControlled->MainCircuitLampsEnabled()); // relay is off and needs a reset
		btLampkaUkrotnienie.Turn(mvControlled->Couplers[mvControlled->CabOccupied == 1 ? rear : front].CouplingFlag.HasAll(Coupling::control));
		btLampkaHamPosp.Turn(TestFlag(mvOccupied->Hamulec->GetBrakeStatus(), 1)); // lampka drugiego stopnia hamowania
		// TODO: youBy wyciągnąć flagę wysokiego stopnia
		btLampkaNadmWent.Turn(mvControlled->RventRot < 5.0 && mvControlled->ResistorsFlagCheck()); // hunter-121211: lampka zanikowo-pradowego wentylatorow
		btLampkaWysRozr.Turn(mvControlled->Imax >= mvControlled->ImaxHi);

		btLampkaNapNastHam.Turn(mvControlled->DirActive != 0); // napiecie na nastawniku hamulcowym
		btLampkaSprezarka.Turn(mvControlled->CompressorFlag); // mutopsitka dziala
		btLampkaSprezarkaOff.Turn(!mvControlled->CompressorFlag);
		btLampkaFuelPumpOff.Turn(!mvControlled->FuelPump.is_active);
		btLampkaOilPump.Turn(mvControlled->OilPump.is_active); // pre-lubrication pump, goes off once the engine is started

		// boczniki
		// Ra: w SU45 boczniki wchodzą na ScndCtrlPos, a nie na ScndCtrlActualPos
		// - pokićkał ktoś?
		int scp = mvControlled->RList[mvControlled->MainCtrlPos].ScndAct;
		scp = scp == 255 ? 0 : scp; // Ra: whatta hella is this?
		btLampkaBoczniki.Turn(!mvControlled->DelayCtrlFlag && mvControlled->ScndCtrlActualPos > 0 || scp > 0);
		btLampkaBocznik1.Turn(!mvControlled->DelayCtrlFlag && mvControlled->ScndCtrlPos > 0 || mvControlled->ScndCtrlActualPos > 0 || scp > 0);
		btLampkaBocznik2.Turn(!mvControlled->DelayCtrlFlag && mvControlled->ScndCtrlPos > 1 || mvControlled->ScndCtrlActualPos > 1 || scp > 1);
		btLampkaBocznik3.Turn(!mvControlled->DelayCtrlFlag && mvControlled->ScndCtrlPos > 2 || mvControlled->ScndCtrlActualPos > 2 || scp > 2);
		btLampkaBocznik4.Turn(!mvControlled->DelayCtrlFlag && mvControlled->ScndCtrlPos > 3 || mvControlled->ScndCtrlActualPos > 3 || scp > 3);

		// biegi dla motoraka
		btLampkaGear1.Turn(mvControlled->ScndCtrlActualPos == 1 && mvControlled->dizel_engagestate > 0); // bieg 1
		btLampkaGear2.Turn(mvControlled->ScndCtrlActualPos == 2 && mvControlled->dizel_engagestate > 0); // bieg 2
		btLampkaHydroLockup.Turn(mvControlled->hydro_TC_Lockup); // zblokowanie skrzyni z silnikiem

		if (mvControlled->Signalling == true)
		{
			if (mvOccupied->BrakePress >= 1.45f)
			{
				btLampkaHamowanie1zes.Turn(true);
			}
			if (mvControlled->BrakePress < 0.75f)
			{
				btLampkaHamowanie1zes.Turn(false);
			}
		}
		else
		{
			btLampkaHamowanie1zes.Turn(false);
		}

		switch (mvControlled->TrainType)
		{
		// zależnie od typu lokomotywy
		case train_type::emu:
		{
			btLampkaHamienie.Turn(mvControlled->BrakePress >= 0.2 && mvControlled->Signalling);
			break;
		}
		case train_type::et41:
		{
			// odhamowanie drugiego członu
			if (mvSecond)
			{
				// bo może komuś przyjść do głowy jeżdżenie jednym członem
				btLampkaHamienie.Turn(mvSecond->BrakePress < 0.4);
			}
			break;
		}
		default:
		{
			btLampkaHamienie.Turn(mvOccupied->BrakePress >= 0.1 || mvControlled->DynamicBrakeFlag);
			btLampkaBrakingOff.Turn(mvOccupied->BrakePress < 0.1 && !mvControlled->DynamicBrakeFlag);
			break;
		}
		}
		// KURS90
		btLampkaMaxSila.Turn(abs(mvControlled->Im) >= 350);
		btLampkaPrzekrMaxSila.Turn(abs(mvControlled->Im) >= 450);
		btLampkaRadio.Turn(mvOccupied->Radio && mvOccupied->RadioFuse);
		btLampkaRadioMessage.Turn(radio_message_played);
		btLampkaRadioStop.Turn(mvOccupied->Radio && mvOccupied->RadioFuse && mvOccupied->RadioStopFlag);
		btLampkaHamulecReczny.Turn(mvOccupied->ManualBrakePos > 0);
		// NBMX wrzesien 2003 - drzwi oraz sygnał odjazdu
		if (DynamicObject->Mechanik != nullptr)
		{
			btLampkaDoorLeft.Turn(DynamicObject->Mechanik->IsAnyDoorOpen[cab_to_end() == end::front ? side::left : side::right]);
			btLampkaDoorRight.Turn(DynamicObject->Mechanik->IsAnyDoorOpen[cab_to_end() == end::front ? side::right : side::left]);
		}
		if (DynamicObject->Mechanik != nullptr)
		{
			// with DoorClosureSignal in the definition of the cab's vehicle, the door closure signalling (SOD) of the whole
			// consist: lit with all doors closed and locked, blinking with them closed but released, dark with a door open.
			// without it, the lock of the vehicle's own doors, as before
			btLampkaBlokadaDrzwi.Turn(mvOccupied->door_closure_lamp_lit(DynamicObject->Mechanik->DoorClosure, simulation::Time.data().wSecond % 2 < 1));
		}
		else
		{
			btLampkaBlokadaDrzwi.Turn(mvOccupied->Doors.is_locked);
		}
		btLampkaDoorLockOff.Turn(false == mvOccupied->Doors.lock_enabled);
		btLampkaDepartureSignal.Turn(mvControlled->DepartureSignal);
		btLampkaNapNastHam.Turn(mvControlled->DirActive != 0 && mvOccupied->EpFuse); // napiecie na nastawniku hamulcowym

		// Wylaczanie lampek kierunku gdy jedziemy
		// Feature uruchamiany z fiz z sekcji Ctrl. wpisem HideDirStatusWhenMoving=Yes (domyslnie No)
		if (mvOccupied->HideDirStatusWhenMoving && // Czy ta funkcja jest w ogole wlaczona
		    mvOccupied->Vel > mvOccupied->HideDirStatusSpeed) // Uzaleznienie od predkosci
		{
			btLampkaForward.Turn(false);
			btLampkaBackward.Turn(false);
			btLampkaNeutral.Turn(false);
		}
		else
		{
			btLampkaForward.Turn(mvControlled->DirActive > 0); // jazda do przodu
			btLampkaBackward.Turn(mvControlled->DirActive < 0); // jazda do tyłu
			btLampkaNeutral.Turn(mvControlled->DirActive == 0); // kierunek neutral
		}

		btLampkaED.Turn(mvControlled->DynamicBrakeFlag); // hamulec ED
		btLampkaBrakeProfileG.Turn(TestFlag(mvOccupied->BrakeDelayFlag, bdelay_G));
		btLampkaBrakeProfileP.Turn(TestFlag(mvOccupied->BrakeDelayFlag, bdelay_P));
		btLampkaBrakeProfileR.Turn(TestFlag(mvOccupied->BrakeDelayFlag, bdelay_R));
		btLampkaSpringBrakeActive.Turn(mvOccupied->SpringBrake.IsActive);
		btLampkaSpringBrakeInactive.Turn(!mvOccupied->SpringBrake.IsActive);
		// light indicators
		// NOTE: sides are hardcoded to deal with setups where single cab is equipped with all indicators
		btLampkaUpperLight.Turn((mvOccupied->iLights[end::front] & light::headlight_upper) != 0);
		btLampkaLeftLight.Turn((mvOccupied->iLights[end::front] & light::headlight_left) != 0);
		btLampkaRightLight.Turn((mvOccupied->iLights[end::front] & light::headlight_right) != 0);
		btLampkaLeftEndLight.Turn((mvOccupied->iLights[end::front] & light::redmarker_left) != 0);
		btLampkaRightEndLight.Turn((mvOccupied->iLights[end::front] & light::redmarker_right) != 0);
		btLampkaRearUpperLight.Turn((mvOccupied->iLights[end::rear] & light::headlight_upper) != 0);
		btLampkaRearLeftLight.Turn((mvOccupied->iLights[end::rear] & light::headlight_left) != 0);
		btLampkaRearRightLight.Turn((mvOccupied->iLights[end::rear] & light::headlight_right) != 0);
		btLampkaRearLeftEndLight.Turn((mvOccupied->iLights[end::rear] & light::redmarker_left) != 0);
		btLampkaRearRightEndLight.Turn((mvOccupied->iLights[end::rear] & light::redmarker_right) != 0);
		// others
		btLampkaMalfunction.Turn(mvControlled->dizel_heat.PA);
		// overheat indicator lamps
		btLampkaOilOverheat.Turn(mvControlled->dizel_heat.oil.is_hot);
		btLampkaWaterOverheat.Turn(mvControlled->dizel_heat.water.is_hot);
		btLampkaWaterAuxOverheat.Turn(mvControlled->dizel_heat.water_aux.is_hot);
		btLampkaEngineOverheat.Turn(mvControlled->dizel_heat.engine_is_hot);
		btLampkaMotorBlowers.Turn(mvControlled->MotorBlowers[end::front].is_active && mvControlled->MotorBlowers[end::rear].is_active);
		btLampkaCoolingFans.Turn(mvControlled->RventRot > 1.0);
		btLampkaLouvresClosed.Turn(mvControlled->LouvresClosed());
		btLampkaTempomat.Turn(mvOccupied->SpeedCtrlUnit.IsActive);
		btLampkaDistanceCounter.Turn(m_distancecounter >= 0.f);
		// universal devices state indicators
		for (auto idx = 0; idx < btUniversals.size(); ++idx)
		{
			btUniversals[idx].Turn(ggUniversals[idx].GetValue() > 0.5);
		}
	}
	else
	{
		// wylaczone
		btLampkaCzuwaka.Turn(false);
		btLampkaSHP.Turn(false);
		btLampkaCzuwakaSHP.Turn(false);
		btLampkaWylSzybki.Turn(false);
		btLampkaWylSzybkiOff.Turn(false);
		btLampkaMainBreakerReady.Turn(false);
		btLampkaMainBreakerBlinkingIfReady.Turn(false);
		btLampkaWysRozr.Turn(false);
		btLampkaOpory.Turn(false);
		btLampkaStyczn.Turn(false);
		btLampkaPrzekRozn.Turn(false);
		btLampkaNadmSil.Turn(false);
		btLampkaUkrotnienie.Turn(false);
		btLampkaHamPosp.Turn(false);
		btLampkaBoczniki.Turn(false);
		btLampkaBocznik1.Turn(false);
		btLampkaBocznik2.Turn(false);
		btLampkaBocznik3.Turn(false);
		btLampkaBocznik4.Turn(false);
		btLampkaGear1.Turn(false);
		btLampkaGear2.Turn(false);
		btLampkaHydroLockup.Turn(false);
		btLampkaNapNastHam.Turn(false);
		btLampkaPrzetw.Turn(false);
		btLampkaPrzetwOff.Turn(false);
		btLampkaPrzetwAOff.Turn(false);
		btLampkaNadmPrzetw.Turn(false);
		btLampkaSprezarka.Turn(false);
		btLampkaSprezarkaOff.Turn(false);
		btLampkaFuelPumpOff.Turn(false);
		btLampkaOilPump.Turn(false);
		btLampkaBezoporowa.Turn(false);
		btLampkaHamowanie1zes.Turn(false);
		btLampkaHamienie.Turn(false);
		btLampkaBrakingOff.Turn(false);
		btLampkaBrakeProfileG.Turn(false);
		btLampkaBrakeProfileP.Turn(false);
		btLampkaBrakeProfileR.Turn(false);
		btLampkaSpringBrakeActive.Turn(false);
		btLampkaSpringBrakeInactive.Turn(false);
		// overheat indicator lamps off
		btLampkaOilOverheat.Turn(false);
		btLampkaWaterOverheat.Turn(false);
		btLampkaWaterAuxOverheat.Turn(false);
		btLampkaEngineOverheat.Turn(false);
		btLampkaMaxSila.Turn(false);
		btLampkaPrzekrMaxSila.Turn(false);
		btLampkaRadio.Turn(false);
		btLampkaRadioMessage.Turn(false);
		btLampkaRadioStop.Turn(false);
		btLampkaHamulecReczny.Turn(false);
		btLampkaDoorLeft.Turn(false);
		btLampkaDoorRight.Turn(false);
		btLampkaBlokadaDrzwi.Turn(false);
		btLampkaDoorLockOff.Turn(false);
		btLampkaDepartureSignal.Turn(false);
		btLampkaNapNastHam.Turn(false);
		btLampkaForward.Turn(false);
		btLampkaBackward.Turn(false);
		btLampkaNeutral.Turn(false);
		btLampkaED.Turn(false);
		// light indicators
		btLampkaUpperLight.Turn(false);
		btLampkaLeftLight.Turn(false);
		btLampkaRightLight.Turn(false);
		btLampkaLeftEndLight.Turn(false);
		btLampkaRightEndLight.Turn(false);
		btLampkaRearUpperLight.Turn(false);
		btLampkaRearLeftLight.Turn(false);
		btLampkaRearRightLight.Turn(false);
		btLampkaRearLeftEndLight.Turn(false);
		btLampkaRearRightEndLight.Turn(false);
		// others
		btLampkaMalfunction.Turn(false);
		btLampkaMotorBlowers.Turn(false);
		btLampkaCoolingFans.Turn(false);
		btLampkaLouvresClosed.Turn(false);
		btLampkaTempomat.Turn(false);
		btLampkaDistanceCounter.Turn(false);
		// universal devices state indicators
		for (auto &universal : btUniversals)
		{
			universal.Turn(false);
		}
	}

	{ // yB - wskazniki drugiego czlonu
		TDynamicObject const *tmp{nullptr}; //=mvControlled->mvSecond; //Ra 2014-07: trzeba to jeszcze wyjąć z kabiny...
		// Ra 2014-07: no nie ma potrzeby szukać tego w każdej klatce
		if (mvControlled->Couplers[1].CouplingFlag.HasAll(Coupling::control) && mvOccupied->CabOccupied > 0)
			tmp = DynamicObject->NextConnected();
		if (mvControlled->Couplers[0].CouplingFlag.HasAll(Coupling::control) && mvOccupied->CabOccupied < 0)
			tmp = DynamicObject->PrevConnected();

		if (tmp)
		{
			if (lowvoltagepower)
			{

				auto const *mover{tmp->MoverParameters};

				btLampkaWylSzybkiB.Turn(mover->Mains);
				btLampkaWylSzybkiBOff.Turn(false == mover->Mains
				                           /*&& ( mover->MainsInitTimeCountdown <= 0.0 )*/
				                           /*&& ( fHVoltage != 0.0 )*/);

				btLampkaOporyB.Turn(mover->ResistorsFlagCheck());
				btLampkaBezoporowaB.Turn(true == mover->ResistorsFlagCheck() || mover->MainCtrlActualPos == 0); // do EU04

				btLampkaStycznB.Turn(mover->LineContactorsLampLit()); // mozna prowadzic rozruch
				// hunter-271211: sygnalizacja poslizgu w pierwszym pojezdzie, gdy wystapi w drugim
				if (mover->SlippingWheels)
				{
					// Ra 2014-12: lokomotywy 181/182 dostają SlippingWheels po zahamowaniu powyżej 2.85 bara i buczały
					auto const veldiff{(DynamicObject->GetVelocity() - fTachoVelocity) / mvControlled->Vmax};
					if (veldiff < -0.01)
					{
						// 1% Vmax rezerwy, żeby 181/182 nie buczały po zahamowaniu, ale to proteza
						auto const lightstate{std::abs(mover->Im) > 10.0};
						btLampkaPoslizg.Turn(btLampkaPoslizg.GetValue() || lightstate);
					}
				}

				btLampkaSprezarkaB.Turn(mover->CompressorFlag); // mutopsitka dziala
				btLampkaSprezarkaBOff.Turn(false == mover->CompressorFlag);
				if (mvControlled->Signalling == true)
				{
					if (mover->BrakePress >= 1.45f)
					{
						btLampkaHamowanie2zes.Turn(true);
					}
					if (mover->BrakePress < 0.75f)
					{
						btLampkaHamowanie2zes.Turn(false);
					}
				}
				else
				{
					btLampkaHamowanie2zes.Turn(false);
				}
				btLampkaNadmPrzetwB.Turn(mover->ConvOvldFlag); // nadmiarowy przetwornicy?
				btLampkaPrzetwB.Turn(mover->ConverterFlag); // zalaczenie przetwornicy
				btLampkaPrzetwBOff.Turn(false == mover->ConverterFlag);
				btLampkaHVoltageB.Turn(mover->NoVoltRelay && mover->OvervoltageRelay);
				btLampkaMalfunctionB.Turn(mover->dizel_heat.PA);
				// motor fuse indicator turns on if the fuse was blown in any unit under control
				if (mover->Mains)
				{
					btLampkaNadmSil.Turn(btLampkaNadmSil.GetValue() || mover->FuseFlagCheck());
				}
			}
			else // wylaczone
			{
				btLampkaWylSzybkiB.Turn(false);
				btLampkaWylSzybkiBOff.Turn(false);
				btLampkaOporyB.Turn(false);
				btLampkaStycznB.Turn(false);
				btLampkaSprezarkaB.Turn(false);
				btLampkaSprezarkaBOff.Turn(false);
				btLampkaBezoporowaB.Turn(false);
				btLampkaHamowanie2zes.Turn(false);
				btLampkaNadmPrzetwB.Turn(false);
				btLampkaPrzetwB.Turn(false);
				btLampkaPrzetwBOff.Turn(false);
				btLampkaHVoltageB.Turn(false);
				btLampkaMalfunctionB.Turn(false);
			}
		}
	}
	// McZapkie-080602: obroty (albo translacje) regulatorow
	if (ggJointCtrl.SubModel != nullptr)
	{
		// joint master controller moves forward to adjust power and backward to adjust brakes
		auto const brakerangemultiplier{/* NOTE: scaling disabled as it was conflicting with associating sounds with control positions
		                                ( mvControlled->CoupledCtrl ?
		                                    mvControlled->MainCtrlPosNo + mvControlled->ScndCtrlPosNo :
		                                    mvControlled->MainCtrlPosNo )
		                                / static_cast<double>(LocalBrakePosNo)
		                                */
		                                1};
		// when SplitEDPneumaticBrake is active the negative range of the joint controller
		// represents the dedicated dynamic-brake lever (DBPN steps), not the pneumatic local brake
		auto const negativePart{mvControlled->SplitEDPneumaticBrake ?
		                            (mvControlled->DynamicBrakeCtrlPos > 0.0 ? mvControlled->DynamicBrakeCtrlPos * mvControlled->DynamicBrakeCtrlPosNo * -1 * brakerangemultiplier : 0.0) :
		                            mvOccupied->LocalBrakePosA > 0.0 ? mvOccupied->LocalBrakePosA * LocalBrakePosNo * -1 * brakerangemultiplier :
		                                                           0.0};
		ggJointCtrl.UpdateValue(negativePart < 0.0        ? static_cast<float>(negativePart) :
		                        mvControlled->CoupledCtrl ? static_cast<float>(double(mvControlled->MainCtrlPos + mvControlled->ScndCtrlPos)) :
		                                                    static_cast<float>(double(mvControlled->MainCtrlPos)),
		                        dsbNastawnikJazdy);
		ggJointCtrl.Update();
	}
	if (ggMainCtrl.SubModel != nullptr)
	{

#ifdef _WIN32
		if (DynamicObject->Mechanik != nullptr && false == DynamicObject->Mechanik->AIControllFlag // nie blokujemy AI
		    && Global.iFeedbackMode == 4 && Global.fCalibrateIn[2][1] != 0.0)
		{

			set_master_controller(Console::AnalogCalibrateGet(2) * mvOccupied->MainCtrlPosNo);
			mvOccupied->eimic_analog = Console::AnalogCalibrateGet(2);
		}
#endif

		if (mvControlled->CoupledCtrl)
		{
			ggMainCtrl.UpdateValue(static_cast<float>(double(mvControlled->MainCtrlPos + mvControlled->ScndCtrlPos)), dsbNastawnikJazdy);
		}
		else
		{
			ggMainCtrl.UpdateValue(static_cast<float>(double(mvControlled->MainCtrlPos)), dsbNastawnikJazdy);
		}
		ggMainCtrl.Update();
	}
	if (ggMainCtrlAct.SubModel != nullptr)
	{
		ggMainCtrlAct.UpdateValue(main_ctrl_act::position(*mvControlled, ggMainCtrlAct.has_scaletable()));
		ggMainCtrlAct.Update();
	}
	if (ggScndCtrl.SubModel != nullptr)
	{
		// Ra: od byte odejmowane boolean i konwertowane potem na double?
		if (false == ggScndCtrl.is_push())
		{
			ggScndCtrl.UpdateValue(static_cast<float>(double(mvControlled->ScndCtrlPos - (mvControlled->TrainType == train_type::et42 && mvControlled->DynamicBrakeFlag))), dsbNastawnikBocz);
		}
		ggScndCtrl.Update();
	}
	if (ggScndCtrlButton.SubModel != nullptr)
	{
		if (ggScndCtrlButton.is_toggle())
		{
			ggScndCtrlButton.UpdateValue(mvControlled->IsSpeedCtrlEngaged() ? 1.f : 0.f, dsbSwitch);
		}
		ggScndCtrlButton.Update(lowvoltagepower);
	}
	if (ggScndCtrlOffButton.SubModel != nullptr)
	{
		ggScndCtrlOffButton.Update(lowvoltagepower);
	}
	if (ggDistanceCounterButton.SubModel != nullptr)
	{
		ggDistanceCounterButton.Update();
	}
	if (ggDirKey.SubModel != nullptr)
	{
		if (mvControlled->TrainType != train_type::emu)
		{
			ggDirKey.UpdateValue(static_cast<float>(double(mvControlled->DirActive)), dsbReverserKey);
		}
		else
		{
			ggDirKey.UpdateValue(static_cast<float>(double(mvControlled->DirActive) + double(mvControlled->Imin == mvControlled->IminHi)), dsbReverserKey);
		}
		ggDirKey.Update();
	}
	if (ggBrakeCtrl.SubModel != nullptr)
	{
#ifdef _WIN32
		if (DynamicObject->Mechanik ? (DynamicObject->Mechanik->AIControllFlag ? false : Global.iFeedbackMode == 4 /*|| (Global.bMWDmasterEnable && Global.bMWDBreakEnable)*/) :
		                              false && Global.fCalibrateIn[0][1] != 0.0) // nie blokujemy AI
		{ // Ra: nie najlepsze miejsce, ale na początek gdzieś to dać trzeba
			// Firleju: dlatego kasujemy i zastepujemy funkcją w Console
			if (mvOccupied->BrakeHandle == TBrakeHandle::FV4a)
			{
				double b = Console::AnalogCalibrateGet(0);
				b = b * 8.0 - 2.0;
				b = safe_clamp(b, -2.0, (double)mvOccupied->BrakeCtrlPosNo); // przycięcie zmiennej do granic
				ggBrakeCtrl.UpdateValue(b); // przesów bez zaokrąglenia
				mvOccupied->BrakeLevelSet(b);
			}
			else if (mvOccupied->BrakeHandle == TBrakeHandle::FVel6) // może można usunąć ograniczenie do FV4a i FVel6?
			{
				double b = Console::AnalogCalibrateGet(0);
				b = b * 7.0 - 1.0;
				b = safe_clamp(b, -1.0, (double)mvOccupied->BrakeCtrlPosNo); // przycięcie zmiennej do granic
				ggBrakeCtrl.UpdateValue(b); // przesów bez zaokrąglenia
				mvOccupied->BrakeLevelSet(b);
			}
			else
			{
				double b = Console::AnalogCalibrateGet(0);
				b = b * (mvOccupied->Handle->GetPos(bh_MAX) - mvOccupied->Handle->GetPos(bh_MIN)) + mvOccupied->Handle->GetPos(bh_MIN);
				b = safe_clamp(b, mvOccupied->Handle->GetPos(bh_MIN), mvOccupied->Handle->GetPos(bh_MAX)); // przycięcie zmiennej do granic
				ggBrakeCtrl.UpdateValue(b); // przesów bez zaokrąglenia
				mvOccupied->BrakeLevelSet(b);
			}
		}
		else
#endif
		{
			// else //standardowa prodedura z kranem powiązanym z klawiaturą
			// ggBrakeCtrl.UpdateValue(double(mvOccupied->BrakeCtrlPos));
			ggBrakeCtrl.UpdateValue(static_cast<float>(mvOccupied->fBrakeCtrlPos));
			ggBrakeCtrl.Update();
		}
	}

	if (ggLocalBrake.SubModel != nullptr)
	{
#ifdef _WIN32
		if (DynamicObject->Mechanik != nullptr && false == DynamicObject->Mechanik->AIControllFlag // nie blokujemy AI
		    && mvOccupied->BrakeLocHandle == TBrakeHandle::FD1 && Global.iFeedbackMode == 4 && Global.fCalibrateIn[0][1] != 0.0)
		{
			// Ra: nie najlepsze miejsce, ale na początek gdzieś to dać trzeba
			// Firleju: dlatego kasujemy i zastepujemy funkcją w Console
			auto const b = std::clamp(Console::AnalogCalibrateGet(1), 0.f, 1.f);
			mvOccupied->LocalBrakePosA = b;
			ggLocalBrake.UpdateValue(b * LocalBrakePosNo);
		}
		else
#endif
		{
			// standardowa prodedura z kranem powiązanym z klawiaturą
			ggLocalBrake.UpdateValue(static_cast<float>(mvOccupied->LocalBrakePosA * LocalBrakePosNo));
		}
		ggLocalBrake.Update();
	}
	ggDirForwardButton.Update(lowvoltagepower);
	ggDirNeutralButton.Update(lowvoltagepower);
	ggDirBackwardButton.Update(lowvoltagepower);
	ggAlarmChain.Update();
	ggBrakeProfileCtrl.Update();
	ggBrakeProfileG.Update();
	ggBrakeProfileR.Update();
	ggBrakeOperationModeCtrl.Update();
	ggWiperSw.Update();
	ggMaxCurrentCtrl.UpdateValue(true == mvControlled->ShuntModeAllow ? (true == mvControlled->ShuntMode ? 1.f : 0.f) : mvControlled->MotorOverloadRelayHighThreshold ? 1.f : 0.f);
	ggMaxCurrentCtrl.Update();
	ggAutoRelayButton.UpdateValue(mvOccupied->AutoRelayFlag ? 1.f : 0.f);
	ggAutoRelayButton.Update();
	// NBMX wrzesien 2003 - drzwi
	ggDoorLeftPermitButton.Update(lowvoltagepower);
	ggDoorRightPermitButton.Update(lowvoltagepower);
	ggDoorPermitPresetButton.Update(lowvoltagepower);
	ggDoorLeftButton.Update(lowvoltagepower);
	ggDoorRightButton.Update(lowvoltagepower);
	ggDoorLeftOnButton.Update(lowvoltagepower);
	ggDoorRightOnButton.Update(lowvoltagepower);
	ggDoorLeftOffButton.Update(lowvoltagepower);
	ggDoorRightOffButton.Update(lowvoltagepower);
	ggDoorAllOnButton.Update(lowvoltagepower);
	ggDoorAllOffButton.Update(lowvoltagepower);
	ggDoorSignallingButton.Update(lowvoltagepower);
	ggDoorStepButton.Update(lowvoltagepower);
	// NBMX dzwignia sprezarki
	ggCompressorButton.Update();
	ggCompressorLocalButton.Update();
	ggCompressorListButton.Update();

	//---------
	// hunter-080812: poprawka na ogrzewanie w elektrykach - usuniete uzaleznienie od przetwornicy
	if (mvControlled->Heating == true && mvControlled->ConvOvldFlag == false)
		btLampkaOgrzewanieSkladu.Turn(true);
	else
		btLampkaOgrzewanieSkladu.Turn(false);

	//----------

	// lights
	auto const lightpower{(InstrumentLightType == 0 ? mvOccupied->Power24vIsAvailable || mvOccupied->Power110vIsAvailable :
	                       InstrumentLightType == 1 ? mvControlled->Mains :
	                       InstrumentLightType == 2 ? mvOccupied->Power110vIsAvailable :
	                       InstrumentLightType == 3 ? mvOccupied->Power24vIsAvailable || mvOccupied->Power110vIsAvailable :
	                       InstrumentLightType == 4 ? mvOccupied->Power24vIsAvailable || mvOccupied->Power110vIsAvailable :
	                                                  false)};
	InstrumentLightActive = InstrumentLightType == 3 ? true : // TODO: link the light state with the state of the master key
	                            InstrumentLightType == 4 ? mvOccupied->iLights[end::front] != 0 || mvOccupied->iLights[end::rear] != 0 :
	                                                       InstrumentLightActive;
	btInstrumentLight.Turn(InstrumentLightActive && lightpower);
	btDashboardLight.Turn(DashboardLightActive && lightpower);
	btTimetableLight.Turn(TimetableLightActive && lightpower);

	// guziki:
	ggMainOffButton.Update();
	ggMainOnButton.Update();
	ggMainButton.Update();
	for (auto &remotelinebreaker : m_remotelinebreakers)
	{
		remotelinebreaker.main_switch.Update();
		remotelinebreaker.on_button.Update();
		remotelinebreaker.off_button.Update();
	}
	ggSecurityResetButton.Update();
	ggSHPResetButton.Update();
	ggReleaserButton.Update();
	ggSpringBrakeOnButton.Update();
	ggSpringBrakeOffButton.Update();
	ggUniveralBrakeButton1.Update();
	ggUniveralBrakeButton2.Update();
	ggUniveralBrakeButton3.Update();
	ggEPFuseButton.Update();
	ggAntiSlipButton.Update();
	ggSandButton.Update();
	ggAutoSandButton.Update();
	ggFuseButton.Update();
	ggConverterFuseButton.Update();
	ggStLinOffButton.Update();
	ggRadioChannelSelector.Update();
	ggRadioChannelPrevious.Update();
	ggRadioChannelNext.Update();
	ggRadioStop.Update();
	ggRadioTest.Update();
	ggRadioCall1.Update();
	ggRadioCall3.Update();
	ggRadioVolumeSelector.Update();
	ggRadioVolumePrevious.Update();
	ggRadioVolumeNext.Update();
	ggRadioOnButton.Update();
	ggRadioOffButton.Update();
	ggRadioFuseOnButton.Update();
	ggRadioFuseOffButton.Update();
	ggDepartureSignalButton.Update();
	/*
	    ggPantFrontButton.Update();
	    ggPantRearButton.Update();
	    ggPantFrontButtonOff.Update();
	    ggPantRearButtonOff.Update();
	*/
	ggPantAllDownButton.Update();
	ggPantSelectedDownButton.Update();
	ggPantSelectedButton.Update();
	ggPantValvesButton.Update();
	ggPantCompressorButton.Update();
	ggPantCompressorValve.Update();

	ggPantValvesOff.Update();
	ggPantValvesUpdate.Update();

	ggLightsButton.Update();
	ggUpperLightButton.Update();
	ggLeftLightButton.Update();
	ggRightLightButton.Update();
	ggLeftEndLightButton.Update();
	ggRightEndLightButton.Update();
	ggModernLightDimSw.Update();
	// hunter-230112
	ggRearUpperLightButton.Update();
	ggRearLeftLightButton.Update();
	ggRearRightLightButton.Update();
	ggRearLeftEndLightButton.Update();
	ggRearRightEndLightButton.Update();
	ggDimHeadlightsButton.Update();
	ggDimHeadlightsButton.Update();
	//------------
	// the converter switches follow the vehicle when something else flips them (the AI driver, the other cab,
	// the pressure switch of an EMU), not only when the cab is entered (set_cab_controls())
	if (ggConverterButton.SubModel != nullptr && mvOccupied->ConvSwitchType != "impulse" && mvOccupied->ConverterStart == start_t::manual)
	{
		ggConverterButton.UpdateValue(mvOccupied->ConverterAllow ? 1.f : 0.f, dsbSwitch);
	}
	ggConverterButton.Update();
	if (ggConverterLocalButton.SubModel != nullptr)
	{
		ggConverterLocalButton.UpdateValue(mvOccupied->ConverterAllowLocal ? 1.f : 0.f, dsbSwitch);
	}
	ggConverterLocalButton.Update();
	ggConverterOffButton.Update();
	ggTrainHeatingButton.Update();
	ggSignallingButton.Update();
	ggNextCurrentButton.Update();
	ggHornButton.Update();
	ggHornLowButton.Update();
	ggHornHighButton.Update();
	ggWhistleButton.Update();
	if (DynamicObject->Mechanik != nullptr)
	{
		ggHelperButton.UpdateValue(static_cast<float>(DynamicObject->Mechanik->HelperState));
	}
	ggHelperButton.Update();

	ggSpeedControlIncreaseButton.Update(lowvoltagepower);
	ggSpeedControlDecreaseButton.Update(lowvoltagepower);
	ggSpeedControlPowerIncreaseButton.Update(lowvoltagepower);
	ggSpeedControlPowerDecreaseButton.Update(lowvoltagepower);
	for (auto &speedctrlbutton : ggSpeedCtrlButtons)
	{
		speedctrlbutton.Update(lowvoltagepower);
	}
	for (auto &universal : ggUniversals)
	{
		universal.Update();
	}
	for (auto &item : ggInverterEnableButtons)
	{
		item.Update();
	}
	for (auto &item : ggInverterDisableButtons)
	{
		item.Update();
	}
	for (auto &item : ggInverterToggleButtons)
	{
		item.Update();
	}
	for (auto &relayresetbutton : ggRelayResetButtons)
	{
		relayresetbutton.Update();
	}
	// hunter-091012
	ggInstrumentLightButton.Update();
	ggDashboardLightButton.Update();
	ggTimetableLightButton.Update();
	ggCabLightDimButton.Update();
	ggCompartmentLightsButton.Update();
	ggCompartmentLightsOnButton.Update();
	ggCompartmentLightsOffButton.Update();
	ggBatteryButton.Update();
	ggBatteryOnButton.Update();
	ggBatteryOffButton.Update();
	if (ggCabActivationButton.SubModel != nullptr && ggCabActivationButton.type() != TGaugeType::push)
	{
		ggCabActivationButton.UpdateValue(mvOccupied->IsCabMaster() ? 1.0 : 0.0);
	}
	ggCabActivationButton.Update();

	ggWaterPumpBreakerButton.Update();
	ggWaterPumpButton.Update();
	ggWaterHeaterBreakerButton.Update();
	ggWaterHeaterButton.Update();
	ggWaterCircuitsLinkButton.Update();
	ggFuelPumpButton.Update();
	ggOilPumpButton.Update();
	ggOilFuelPumpButton.Update();
	ggTractionModeSwitch.Update();
	ggGensetButton.Update();
	ggMotorBlowersFrontButton.Update();
	ggMotorBlowersRearButton.Update();
	ggMotorBlowersAllOffButton.Update();

	// wyprowadzenie sygnałów dla haslera na PoKeys (zaznaczanie na taśmie)
	btHaslerBrakes.Turn(mvOccupied->BrakePress > 0.4); // ciśnienie w cylindrach
	btHaslerCurrent.Turn(mvOccupied->Im != 0.0); // prąd na silnikach

	// calculate current level of interior illumination
	{
		// TODO: organize it along with rest of train update in a more sensible arrangement
		// Ra: uzeleżnic od napięcia w obwodzie sterowania
		// hunter-091012: uzaleznienie jasnosci od przetwornicy
		int cabidx{0};
		for (auto &cab : Cabine)
		{

			auto const cablightlevel = (cab.bLight == false ? 0.f : cab.bLightDim == true ? 0.4f : 1.f) * (mvOccupied->Power110vIsAvailable ? 1.f : 0.5f);

			if (cab.LightLevel != cablightlevel)
			{
				cab.LightLevel = cablightlevel;
				DynamicObject->set_cab_lights(cabidx, cab.LightLevel);
			}
			if (cabidx == iCabn)
			{
				DynamicObject->InteriorLightLevel = cablightlevel;
			}

			++cabidx;
		}
	}

	// anti slip system activation, maintained while the control button is down
	if (mvOccupied->BrakeSystem != TBrakeSystem::ElectroPneumatic && ggAntiSlipButton.GetDesiredValue() > 0.95)
	{
		mvControlled->AntiSlippingBrake();
	}
	// screens

	if (!FreeFlyModeFlag && simulation::Train == this) // don't bother if we're outside
	{
		update_screens(Deltatime);
		update_script_screens();
	}

	// update direction relay
	if (prevBatState != mvOccupied->Power24vIsAvailable)
		SetupDirectionRelays();
	if (prevDirection != mvOccupied->DirActive)
		UpdateDirectionRelays();

	prevBatState = mvOccupied->Power24vIsAvailable;
	prevDirection = mvOccupied->DirActive;

	// sounds
	update_sounds(Deltatime);

	return true; //(DynamicObject->Update(dt));
} // koniec update

void TTrain::UpdateDirectionRelays()
{
	if (mvOccupied->DirActive < 0 && mvOccupied->Power24vIsAvailable) // wstecz
		Dynamic()->sDirectionRelayR.play();
	if (mvOccupied->DirActive == 0 && mvOccupied->Power24vIsAvailable) // neutral
		Dynamic()->sDirectionRelayN.play();
	if (mvOccupied->DirActive > 0 && mvOccupied->Power24vIsAvailable) // przod
		Dynamic()->sDirectionRelayD.play();
}

void TTrain::SetupDirectionRelays()
{
	if (mvOccupied->Power24vIsAvailable)
	{
		if (mvOccupied->DirActive < 0 && mvOccupied->Power24vIsAvailable) // wstecz
			Dynamic()->sDirectionRelayR.play();
		if (mvOccupied->DirActive > 0 && mvOccupied->Power24vIsAvailable) // przod
			Dynamic()->sDirectionRelayD.play();
	}
	else if (mvOccupied->DirActive != 0) // neutral
		Dynamic()->sDirectionRelayN.play();
}
