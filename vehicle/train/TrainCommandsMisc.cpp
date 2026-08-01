/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/Train.h"

#include "world/Event.h"
#include "simulation/simulationtime.h"
#include "model/Model3d.h"
#include "vehicle/Driver.h"
#include "vehicle/DynObj.h"
#include <future>

void TTrain::OnCommand_alerteracknowledge(TTrain *Train, command_data const &Command)
{
	if (Command.action == GLFW_PRESS)
	{
		// visual feedback
		Train->ggSecurityResetButton.UpdateValue(1.0, Train->dsbSwitch);

		if (Train->mvOccupied->TrainType == dt_EZT || Train->mvOccupied->DirActive != 0)
			Train->mvOccupied->SecuritySystem.acknowledge_press();
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// visual feedback
		Train->ggSecurityResetButton.UpdateValue(0.0);

		if (Train->mvOccupied->TrainType == dt_EZT || Train->mvOccupied->DirActive != 0)
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

void TTrain::OnCommand_generictoggle(TTrain *Train, command_data const &Command)
{

	auto const itemindex = static_cast<int>(Command.command) - static_cast<int>(user_command::generictoggle0);
	auto &item = Train->ggUniversals[itemindex];
	/*
	    if( item.SubModel == nullptr ) {
	        if( Command.action == GLFW_PRESS ) {
	            WriteLog( "Train generic item " + std::to_string( itemindex ) + " is missing, or wasn't defined" );
	        }
	        return;
	    }
	*/

	if (Command.action == GLFW_PRESS)
	{

		if (item.type() == TGaugeType::push)
		{
			// impulse switch
			// turn on
			// visual feedback
			item.UpdateValue(1.0);
		}
		else
		{
			// two-state switch
			if (item.GetDesiredValue() < 0.5)
			{
				// turn on
				// visual feedback
				item.UpdateValue(1.0);
			}
			else
			{
				// turn off
				// visual feedback
				item.UpdateValue(0.0);
			}
		}
	}
	else if (Command.action == GLFW_RELEASE && item.type() == TGaugeType::push)
	{

		// impulse switch
		// turn off
		// visual feedback
		item.UpdateValue(0.0);
	}
}

void TTrain::OnCommand_cabchangeforward(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		auto const *owner{(Train->DynamicObject->ctOwner != nullptr ? Train->DynamicObject->ctOwner : Train->DynamicObject->Mechanik)};
		auto const movedirection{1};
		if (false == Train->CabChange(movedirection))
		{
			auto const exitdirection{(movedirection > 0 ? end::front : end::rear)};
			if (TestFlag(Train->mvOccupied->Couplers[exitdirection].CouplingFlag, coupling::gangway))
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
		auto const *owner{(Train->DynamicObject->ctOwner != nullptr ? Train->DynamicObject->ctOwner : Train->DynamicObject->Mechanik)};
		auto const movedirection{-1};
		if (false == Train->CabChange(movedirection))
		{
			// current vehicle doesn't extend any farther in this direction, check if we there's one connected we can move to
			auto const exitdirection{(movedirection > 0 ? end::front : end::rear)};
			if (TestFlag(Train->mvOccupied->Couplers[exitdirection].CouplingFlag, coupling::gangway))
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
