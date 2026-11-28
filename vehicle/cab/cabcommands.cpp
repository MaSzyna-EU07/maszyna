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

// the commands of the cab itself: the reverser, the battery, its activation, changing it

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


void TTrain::OnCommand_wiperswitchincrease(TTrain *Train, command_data const &Command)
{
	if (Command.action == GLFW_PRESS)
	{
		Train->mvOccupied->wiperSwitchPos++;
		if (Train->mvOccupied->wiperSwitchPos > Train->mvOccupied->WiperListSize - 1)
			Train->mvOccupied->wiperSwitchPos = Train->mvOccupied->WiperListSize - 1;

		// Visual feedback
		Train->ggWiperSw.UpdateValue(static_cast<float>(Train->mvOccupied->wiperSwitchPos), Train->dsbSwitch);
	}
}
void TTrain::OnCommand_wiperswitchdecrease(TTrain *Train, command_data const &Command)
{
	if (Command.action == GLFW_PRESS)
	{
		Train->mvOccupied->wiperSwitchPos--;
		if (Train->mvOccupied->wiperSwitchPos < 0)
			Train->mvOccupied->wiperSwitchPos = 0;

		// visual feedback
		Train->ggWiperSw.UpdateValue(static_cast<float>(Train->mvOccupied->wiperSwitchPos), Train->dsbSwitch);
	}
}

void TTrain::OnCommand_reverserincrease(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{

		// HACK: master controller position isn't set in occupied vehicle in E(D)MUs
		// so we do a manual check in relevant vehicle here
		if (false == Train->mvControlled->EIMDirectionChangeAllow())
		{
			return;
		}

		// aktualizacja skrajnych pojazdów w składzie
		if (Train->mvOccupied->DirectionForward() && Train->mvOccupied->DirActive && Train->DynamicObject->Mechanik)
		{

			Train->DynamicObject->Mechanik->DirectionChange();
		}
	}
}

void TTrain::OnCommand_reverserdecrease(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{

		// HACK: master controller position isn't set in occupied vehicle in E(D)MUs
		// so we do a manual check in relevant vehicle here
		if (false == Train->mvControlled->EIMDirectionChangeAllow())
		{
			return;
		}

		// aktualizacja skrajnych pojazdów w składzie
		if (Train->mvOccupied->DirectionBackward() && Train->mvOccupied->DirActive && Train->DynamicObject->Mechanik)
		{

			Train->DynamicObject->Mechanik->DirectionChange();
		}
	}
}

void TTrain::OnCommand_reverserforwardhigh(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{

		// HACK: master controller position isn't set in occupied vehicle in E(D)MUs
		// so we do a manual check in relevant vehicle here
		if (false == Train->mvControlled->EIMDirectionChangeAllow())
		{
			return;
		}

		// HACK: try to move the reverser one position back, in case it's set to "high forward"
		OnCommand_reverserdecrease(Train, Command);

		if (Train->mvOccupied->DirActive < 1)
		{

			while (Train->mvOccupied->DirActive < 1 && true == Train->mvOccupied->DirectionForward())
			{
				// all work is done in the header
			}
			// aktualizacja skrajnych pojazdów w składzie
			if (Train->mvOccupied->DirActive == 1 && Train->DynamicObject->Mechanik)
			{

				Train->DynamicObject->Mechanik->DirectionChange();
			}
		}
		OnCommand_reverserincrease(Train, Command);
	}
}

void TTrain::OnCommand_reverserforward(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{

		// HACK: master controller position isn't set in occupied vehicle in E(D)MUs
		// so we do a manual check in relevant vehicle here
		if (false == Train->mvControlled->EIMDirectionChangeAllow())
		{
			return;
		}

		// HACK: try to move the reverser one position back, in case it's set to "high forward"
		// OnCommand_reverserdecrease( Train, Command );
		// visual feedback
		Train->ggDirForwardButton.UpdateValue(1.0, Train->dsbSwitch);

		if (Train->mvOccupied->DirActive == 0)
		{

			while (Train->mvOccupied->DirActive < 1 && true == Train->mvOccupied->DirectionForward())
			{
				// all work is done in the header
			}
			// aktualizacja skrajnych pojazdów w składzie
			if (Train->mvOccupied->DirActive == 1 && Train->DynamicObject->Mechanik)
			{

				Train->DynamicObject->Mechanik->DirectionChange();
			}
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// release
		// visual feedback
		Train->ggDirForwardButton.UpdateValue(0.0, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_reverserneutral(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{

		// HACK: master controller position isn't set in occupied vehicle in E(D)MUs
		// so we do a manual check in relevant vehicle here
		if (false == Train->mvControlled->EIMDirectionChangeAllow())
		{
			return;
		}
		// visual feedback
		Train->ggDirNeutralButton.UpdateValue(1.0, Train->dsbSwitch);
		while (Train->mvOccupied->DirActive < 0 && true == Train->mvOccupied->DirectionForward())
		{
			// all work is done in the header
		}
		while (Train->mvOccupied->DirActive > 0 && true == Train->mvOccupied->DirectionBackward())
		{
			// all work is done in the header
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// release
		// visual feedback
		Train->ggDirNeutralButton.UpdateValue(0.0, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_reverserbackward(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{

		// HACK: master controller position isn't set in occupied vehicle in E(D)MUs
		// so we do a manual check in relevant vehicle here
		if (false == Train->mvControlled->EIMDirectionChangeAllow())
		{
			return;
		}
		Train->ggDirBackwardButton.UpdateValue(1.0, Train->dsbSwitch);
		if (Train->mvOccupied->DirActive == 0)
		{

			while (Train->mvOccupied->DirActive > -1 && true == Train->mvOccupied->DirectionBackward())
			{
				// all work is done in the header
			}
			// aktualizacja skrajnych pojazdów w składzie
			if (Train->mvOccupied->DirActive == -1 && Train->DynamicObject->Mechanik)
			{

				Train->DynamicObject->Mechanik->DirectionChange();
			}
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// release
		// visual feedback
		Train->ggDirBackwardButton.UpdateValue(0.0, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_alerteracknowledge(TTrain *Train, command_data const &Command)
{
	if (Command.action == GLFW_PRESS)
	{
		// visual feedback
		Train->ggSecurityResetButton.UpdateValue(1.0, Train->dsbSwitch);

		if (Train->mvOccupied->TrainType == train_type::emu || Train->mvOccupied->DirActive != 0)
			Train->mvOccupied->SecuritySystem.acknowledge_press();
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// visual feedback
		Train->ggSecurityResetButton.UpdateValue(0.0);

		if (Train->mvOccupied->TrainType == train_type::emu || Train->mvOccupied->DirActive != 0)
			Train->mvOccupied->SecuritySystem.acknowledge_release();
	}
}

void TTrain::OnCommand_cabsignalacknowledge(TTrain *Train, command_data const &Command)
{
	// TODO: visual feedback
	if (Command.action == GLFW_PRESS)
	{
		if (Train->mvOccupied->SecuritySystem.has_separate_acknowledge())
		{
			Train->mvOccupied->SecuritySystem.cabsignal_reset();
			Train->ggSHPResetButton.UpdateValue(1.0, Train->dsbSwitch);
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{
		Train->ggSHPResetButton.UpdateValue(0.0);
	}
}

void TTrain::OnCommand_batterytoggle(TTrain *Train, command_data const &Command)
{
	if (Train->allowBatteryToggle || Command.action != GLFW_REPEAT)
	{
		// keep the switch from flipping back and forth if key is held down
		if (false == Train->mvOccupied->Power24vIsAvailable)
		{
			// turn on
			OnCommand_batteryenable(Train, Command);
		}
		else
		{
			// turn off
			OnCommand_batterydisable(Train, Command);
		}
	}
}

void TTrain::OnCommand_batteryenable(TTrain *Train, command_data const &Command)
{
	if (!Train->mvOccupied->isBatteryButtonImpulse)
	{ // regular button behavior
		if (Command.action == GLFW_PRESS)
		{
			// visual feedback
			Train->ggBatteryButton.UpdateValue(1.0f, Train->dsbSwitch);
			Train->ggBatteryOnButton.UpdateValue(1.0f, Train->dsbSwitch);

			Train->mvOccupied->BatterySwitch(true);
			Train->allowBatteryToggle = false;
			// side-effects
			if (Train->mvOccupied->LightsPosNo > 0)
			{
				Train->Dynamic()->SetLights();
			}
		}
		else if (Command.action == GLFW_RELEASE)
		{
			if (Train->ggBatteryButton.type() == TGaugeType::push)
			{
				// return the switch to neutral position
				Train->ggBatteryButton.UpdateValue(0.5f);
			}
			Train->ggBatteryOnButton.UpdateValue(0.0f, Train->dsbSwitch);
			Train->allowBatteryToggle = true;
		}
	}
	else // impulse button behavior
	{
		if (Command.action == GLFW_PRESS)
		{
			if (Train->mvOccupied->shouldHoldBatteryButton)
			{
				// jesli przycisk trzeba przytrzymac
				Train->ggBatteryButton.UpdateValue(1.0f, Train->dsbSwitch);
				Train->ggBatteryOnButton.UpdateValue(1.0f, Train->dsbSwitch);
				Train->fBatteryTimer = Train->mvOccupied->BatteryButtonHoldTime; // start timer
			}
			else
			{
				// jesli przycisk dziala od razu
				Train->mvOccupied->BatterySwitch(true);
				Train->allowBatteryToggle = false;
				// side-effects
				if (Train->mvOccupied->LightsPosNo > 0)
				{
					Train->Dynamic()->SetLights();
				}

				// visual feedback
				Train->ggBatteryButton.UpdateValue(1.0f, Train->dsbSwitch);
				Train->ggBatteryOnButton.UpdateValue(1.0f, Train->dsbSwitch);
			}
		}
		else if (Command.action == GLFW_RELEASE)
		{
			// visual feedback
			Train->ggBatteryButton.UpdateValue(0.0f, Train->dsbSwitch);
			Train->ggBatteryOnButton.UpdateValue(0.0f, Train->dsbSwitch);
			Train->fBatteryTimer = -1.f; //
			Train->allowBatteryToggle = true;
			Train->mvOccupied->batterySwAlreadyFired = false;
		}
		else if (Command.action == GLFW_REPEAT && Train->mvOccupied->shouldHoldBatteryButton)
		{
			// trzymamy przycisk
			if (Train->fBatteryTimer <= 0.0 && Train->mvOccupied->Battery == false && !Train->mvOccupied->batterySwAlreadyFired)
			{
				Train->mvOccupied->BatterySwitch(true);
				Train->mvOccupied->batterySwAlreadyFired = true;

				// side-effects
				if (Train->mvOccupied->LightsPosNo > 0)
				{
					Train->Dynamic()->SetLights();
				}
				Train->allowBatteryToggle = false;
			}
		}
	}
}

void TTrain::OnCommand_batterydisable(TTrain *Train, command_data const &Command)
{
	if (!Train->mvOccupied->isBatteryButtonImpulse)
	{ // regular button behavior
		if (Command.action == GLFW_PRESS)
		{
			// visual feedback
			Train->ggBatteryButton.UpdateValue(0.0f, Train->dsbSwitch);
			Train->ggBatteryOffButton.UpdateValue(1.0f, Train->dsbSwitch);

			Train->mvOccupied->BatterySwitch(false);

			// side-effects
			if (Train->mvOccupied->LightsPosNo > 0)
			{
				Train->Dynamic()->SetLights();
			}
		}
		else if (Command.action == GLFW_RELEASE)
		{
			if (Train->ggBatteryButton.type() == TGaugeType::push)
			{
				// return the switch to neutral position
				Train->ggBatteryButton.UpdateValue(0.5f);
			}
			Train->ggBatteryOffButton.UpdateValue(0.0f, Train->dsbSwitch);
		}
	}
	else // impulse button behavior
	{
		if (Command.action == GLFW_PRESS)
		{
			if (Train->mvOccupied->shouldHoldBatteryButton)
			{
				// jesli przycisk trzeba przytrzymac
				Train->ggBatteryButton.UpdateValue(1.0f, Train->dsbSwitch);
				Train->ggBatteryOffButton.UpdateValue(1.0f, Train->dsbSwitch);
				Train->fBatteryTimer = Train->mvOccupied->BatteryButtonHoldTime; // start timer
			}
			else
			{
				// jesli przycisk dziala od razu
				Train->mvOccupied->BatterySwitch(false);
				Train->allowBatteryToggle = false;

				// side-effects
				if (Train->mvOccupied->LightsPosNo > 0)
				{
					Train->Dynamic()->SetLights();
				}
				// visual feedback
				Train->ggBatteryButton.UpdateValue(1.0f, Train->dsbSwitch);
				Train->ggBatteryOffButton.UpdateValue(1.0f, Train->dsbSwitch);
			}
		}
		else if (Command.action == GLFW_RELEASE)
		{
			// visual feedback
			Train->ggBatteryButton.UpdateValue(0.0f, Train->dsbSwitch);
			Train->ggBatteryOffButton.UpdateValue(0.0f, Train->dsbSwitch);
			Train->allowBatteryToggle = true;
			Train->mvOccupied->batterySwAlreadyFired = false;
		}
		else if (Command.action == GLFW_REPEAT && Train->mvOccupied->shouldHoldBatteryButton)
		{
			// trzymamy przycisk
			if (Train->fBatteryTimer <= 0.0 && Train->mvOccupied->Battery == true && !Train->mvOccupied->batterySwAlreadyFired)
			{
				Train->mvOccupied->BatterySwitch(false);
				Train->mvOccupied->batterySwAlreadyFired = true;

				// side-effects
				if (Train->mvOccupied->LightsPosNo > 0)
				{
					Train->Dynamic()->SetLights();
				}
				Train->allowBatteryToggle = false;
			}
		}
	}
}

void TTrain::OnCommand_cabactivationtoggle(TTrain *Train, command_data const &Command)
{

	if (Command.action != GLFW_REPEAT)
	{
		// keep the switch from flipping back and forth if key is held down
		if (0 == Train->mvOccupied->CabActive)
		{
			// turn on
			OnCommand_cabactivationenable(Train, Command);
		}
		else
		{
			// turn off
			OnCommand_cabactivationdisable(Train, Command);
		}
	}
}

void TTrain::OnCommand_cabactivationenable(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// visual feedback
		if (Train->ggCabActivationButton.type() == TGaugeType::push)
		{
			Train->ggCabActivationButton.UpdateValue(1.0f, Train->dsbSwitch);
		}

		Train->mvOccupied->CabActivisation();

		// side-effects
		if (Train->mvOccupied->LightsPosNo > 0)
		{
			Train->Dynamic()->SetLights();
		}
	}
	else if (Command.action == GLFW_RELEASE && Train->ggCabActivationButton.type() == TGaugeType::push)
	{
		// return the switch to neutral position
		Train->ggCabActivationButton.UpdateValue(0.5f);
	}
}

void TTrain::OnCommand_cabactivationdisable(TTrain *Train, command_data const &Command)
{
	// TBD, TODO: ewentualnie zablokować z FIZ, np. w samochodach się nie odłącza akumulatora
	if (Command.action == GLFW_PRESS)
	{
		// visual feedback
		if (Train->ggCabActivationButton.type() == TGaugeType::push)
		{
			Train->ggCabActivationButton.UpdateValue(0.0f, Train->dsbSwitch);
		}

		Train->mvOccupied->CabDeactivisation();
		if (Train->mvOccupied->LightsPosNo > 0 && Train->mvOccupied->InactiveCabFlag & activation::redmarkers)
		{
			Train->Dynamic()->SetLights();
		}
	}
	else if (Command.action == GLFW_RELEASE && Train->ggCabActivationButton.type() == TGaugeType::push)
	{
		// return the switch to neutral position
		Train->ggCabActivationButton.UpdateValue(0.5f);
	}
}

void TTrain::OnCommand_cabchangeforward(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		if (auto const movedirection{1}; false == Train->CabChange(movedirection))
		{
			auto const exitdirection{(movedirection > 0 ? end::front : end::rear)};
			if (Train->mvOccupied->Couplers[exitdirection].CouplingFlag.HasAll(Coupling::gangway))
			{
				// przejscie do nastepnego pojazdu
				auto *targetvehicle = exitdirection == end::front ? Train->DynamicObject->PrevConnected() : Train->DynamicObject->NextConnected();
				targetvehicle->MoverParameters->CabOccupied = Train->mvOccupied->Neighbours[exitdirection].vehicle_end ? -1 : 1;
				Train->MoveToVehicle(targetvehicle);
			}
		}
		// HACK: match consist door permit state with the preset in the active cab
		if (Train->ggDoorPermitPresetButton.SubModel != nullptr)
		{
			Train->mvOccupied->ChangeDoorPermitPreset(0);
		}
		// HACK: update lights state
		if (Train->mvOccupied->LightsPosNo > 0)
		{
			Train->DynamicObject->SetLights();
		}
	}
}

void TTrain::OnCommand_cabchangebackward(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		if (auto const movedirection{-1}; false == Train->CabChange(movedirection))
		{
			// current vehicle doesn't extend any farther in this direction, check if we there's one connected we can move to
			auto const exitdirection{(movedirection > 0 ? end::front : end::rear)};
			if (Train->mvOccupied->Couplers[exitdirection].CouplingFlag.HasAll(Coupling::gangway))
			{
				// przejscie do nastepnego pojazdu
				auto *targetvehicle = exitdirection == end::front ? Train->DynamicObject->PrevConnected() : Train->DynamicObject->NextConnected();
				targetvehicle->MoverParameters->CabOccupied = Train->mvOccupied->Neighbours[exitdirection].vehicle_end ? -1 : 1;
				Train->MoveToVehicle(targetvehicle);
			}
		}
		// HACK: match consist door permit state with the preset in the active cab
		if (Train->ggDoorPermitPresetButton.SubModel != nullptr)
		{
			Train->mvOccupied->ChangeDoorPermitPreset(0);
		}
		// HACK: update lights state
		if (Train->mvOccupied->LightsPosNo > 0)
		{
			Train->DynamicObject->SetLights();
		}
	}
}

void TTrain::OnCommand_vehiclemoveforwards(TTrain *Train, const command_data &Command)
{
	if (Command.action == GLFW_RELEASE || !DebugModeFlag)
		return;

	Train->DynamicObject->move_set(100.0);
}

void TTrain::OnCommand_vehiclemovebackwards(TTrain *Train, const command_data &Command)
{
	if (Command.action == GLFW_RELEASE || !DebugModeFlag)
		return;

	Train->DynamicObject->move_set(-100.0);
}

void TTrain::OnCommand_vehicleboost(TTrain *Train, const command_data &Command)
{
	if (Command.action == GLFW_RELEASE || !DebugModeFlag)
		return;

	double boost = Command.param1 != 0.0 ? Command.param1 : 2.78;

	if (Train->DynamicObject == nullptr)
	{
		return;
	}

	auto *vehicle{Train->DynamicObject};
	while (vehicle)
	{
		vehicle->MoverParameters->V += vehicle->DirectionGet() * boost;
		vehicle = vehicle->Next(); // pozostałe też
	}
	vehicle = Train->DynamicObject->Prev();
	while (vehicle)
	{
		vehicle->MoverParameters->V += vehicle->DirectionGet() * boost;
		vehicle = vehicle->Prev(); // w drugą stronę też
	}
}
