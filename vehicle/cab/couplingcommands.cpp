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

// the commands of the couplings and the departure

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


void TTrain::OnCommand_nearestcarcouplingincrease(TTrain *Train, command_data const &Command)
{

	if (true == Command.freefly && Command.action == GLFW_PRESS)
	{
		// tryb freefly, press only
		auto couplerindex{-1};
		auto *vehicle{Train->DynamicObject->ABuScanNearestObject(Command.location, Train->DynamicObject->GetTrack(), 1, 1500, couplerindex)};
		if (vehicle == nullptr)
			vehicle = Train->DynamicObject->ABuScanNearestObject(Command.location, Train->DynamicObject->GetTrack(), -1, 1500, couplerindex);

		if (couplerindex != -1 && vehicle != nullptr)
		{

			vehicle->couple(couplerindex);
		}
		if (Train->DynamicObject->Mechanik)
		{
			// aktualizacja flag kierunku w składzie
			Train->DynamicObject->Mechanik->CheckVehicles(DriverOrder::connect);
		}
	}
}

void TTrain::OnCommand_nearestcarcouplingdisconnect(TTrain *Train, command_data const &Command)
{

	if (true == Command.freefly && Command.action == GLFW_PRESS)
	{
		// tryb freefly, press only
		auto couplerindex{-1};
		auto *vehicle{Train->DynamicObject->ABuScanNearestObject(Command.location, Train->DynamicObject->GetTrack(), 1, 1500, couplerindex)};
		if (vehicle == nullptr)
			vehicle = Train->DynamicObject->ABuScanNearestObject(Command.location, Train->DynamicObject->GetTrack(), -1, 1500, couplerindex);

		if (couplerindex != -1 && vehicle != nullptr)
		{

			vehicle->uncouple(couplerindex);
		}
		if (Train->DynamicObject->Mechanik)
		{
			// aktualizacja flag kierunku w składzie
			Train->DynamicObject->Mechanik->CheckVehicles(DriverOrder::disconnect);
		}
	}
}

void TTrain::OnCommand_nearestcarcoupleradapterattach(TTrain * /*Train*/, command_data const &Command)
{

	if (true == Command.freefly && Command.action == GLFW_PRESS)
	{
		// tryb freefly, press only
		auto *vehicle{std::get<TDynamicObject *>(simulation::Region->find_vehicle(Command.location, 50, false, true))};
		if (vehicle == nullptr)
		{
			return;
		}

		auto const couplerindex =
		    glm::length2(glm::vec3{vehicle->CouplerPosition(end::front)} - Command.location) < glm::length2(glm::vec3{vehicle->CouplerPosition(end::rear)} - Command.location) ? end::front : end::rear;

		vehicle->attach_coupler_adapter(couplerindex);
	}
}

void TTrain::OnCommand_nearestcarcoupleradapterremove(TTrain * /*Train*/, command_data const &Command)
{

	if (true == Command.freefly && Command.action == GLFW_PRESS)
	{
		// tryb freefly, press only
		auto *vehicle{std::get<TDynamicObject *>(simulation::Region->find_vehicle(Command.location, 50, false, true))};
		if (vehicle == nullptr)
		{
			return;
		}

		auto const couplerindex =
		    glm::length2(glm::vec3{vehicle->CouplerPosition(end::front)} - Command.location) < glm::length2(glm::vec3{vehicle->CouplerPosition(end::rear)} - Command.location) ? end::front : end::rear;

		vehicle->remove_coupler_adapter(couplerindex);
	}
}

void TTrain::OnCommand_occupiedcarcouplingdisconnect(TTrain *Train, command_data const &Command)
{

	//    if( false == Train->m_controlmapper.contains( "couplingdisconnect_sw:" ) ) { return; }

	if (Command.action == GLFW_PRESS)
	{
		// visual feedback
		Train->m_couplingdisconnect = true;

		if (Train->iCabn == 0)
		{
			return;
		}

		if (Train->DynamicObject)
		{
			Train->DynamicObject->uncouple(Train->cab_to_end());
			if (Train->DynamicObject->Mechanik)
			{
				// aktualizacja flag kierunku w składzie
				Train->DynamicObject->Mechanik->CheckVehicles(DriverOrder::disconnect);
			}
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// visual feedback
		Train->m_couplingdisconnect = false;
	}
}

void TTrain::OnCommand_occupiedcarcouplingdisconnectback(TTrain *Train, command_data const &Command)
{

	//    if( false == Train->m_controlmapper.contains( "couplingdisconnect_sw:" ) ) { return; }

	if (Command.action == GLFW_PRESS)
	{
		// visual feedback
		Train->m_couplingdisconnectback = true;

		if (Train->iCabn == 0)
		{
			return;
		}

		if (Train->DynamicObject)
		{
			Train->DynamicObject->uncouple(1 - Train->cab_to_end());
			if (Train->DynamicObject->Mechanik)
			{
				// aktualizacja flag kierunku w składzie
				Train->DynamicObject->Mechanik->CheckVehicles(DriverOrder::disconnect);
			}
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// visual feedback
		Train->m_couplingdisconnectback = false;
	}
}

void TTrain::OnCommand_departureannounce(TTrain *Train, command_data const &Command)
{

	if (Train->ggDepartureSignalButton.SubModel == nullptr)
	{
		if (Command.action == GLFW_PRESS)
		{
			WriteLog("Departure Signal button is missing, or wasn't defined");
		}
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the sound can loop uninterrupted
		if (false == Train->mvOccupied->DepartureSignal)
		{
			// turn on
			Train->mvOccupied->signal_departure(true);
			// visual feedback
			Train->ggDepartureSignalButton.UpdateValue(1.0, Train->dsbSwitch);
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// turn off
		Train->mvOccupied->signal_departure(false);
		// visual feedback
		Train->ggDepartureSignalButton.UpdateValue(0.0, Train->dsbSwitch);
	}
}
