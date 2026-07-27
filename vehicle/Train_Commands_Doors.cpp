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

#include "simulation/simulation.h"
#include "world/Event.h"
#include "simulation/simulationtime.h"
#include "utilities/Logs.h"
#include "model/Model3d.h"
#include "vehicle/Driver.h"
#include "vehicle/DynObj.h"
#include "rendering/renderer.h"
#include <future>

void TTrain::OnCommand_doorlocktoggle(TTrain *Train, command_data const &Command)
{

	if (Train->ggDoorSignallingButton.SubModel == nullptr)
	{
		if (Command.action == GLFW_PRESS)
		{
			WriteLog("Door Lock switch is missing, or wasn't defined");
		}
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the sound can loop uninterrupted
		if (false == Train->mvOccupied->Doors.lock_enabled)
		{
			// turn on
			// TODO: door lock command to send through consist
			Train->mvOccupied->LockDoors(true);
			// visual feedback
			Train->ggDoorSignallingButton.UpdateValue(1.0, Train->dsbSwitch);
		}
		else
		{
			// turn off
			// TODO: door lock command to send through consist
			Train->mvOccupied->LockDoors(false);
			// visual feedback
			Train->ggDoorSignallingButton.UpdateValue(0.0, Train->dsbSwitch);
		}
	}
}

void TTrain::OnCommand_doortoggleleft(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// NOTE: test how the door state check works with consists where the occupied vehicle doesn't have opening doors
		if (false == (Train->ggDoorLeftButton.GetDesiredValue() > 0.5 || Train->ggDoorLeftOnButton.GetDesiredValue() > 0.5))
		{
			// open
			OnCommand_dooropenleft(Train, Command);
		}
		else
		{
			// close
			if (Train->ggDoorAllOffButton.SubModel != nullptr && Train->ggDoorLeftOffButton.SubModel == nullptr)
			{
				// OnCommand_doorcloseall( Train, Command );
				// if two-button setup lacks dedicated closing button require the user to press appropriate button manually
				return;
			}
			else
			{
				OnCommand_doorcloseleft(Train, Command);
			}
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{

		if (true == (Train->ggDoorLeftButton.GetDesiredValue() > 0.5 || Train->ggDoorLeftOnButton.GetDesiredValue() > 0.5))
		{
			// open
			if (Train->mvOccupied->Doors.has_autowarning && Train->mvOccupied->DepartureSignal)
			{
				// complete closing the doors
				if (Train->ggDoorAllOffButton.SubModel != nullptr && Train->ggDoorLeftOffButton.SubModel == nullptr)
				{
					// OnCommand_doorcloseall( Train, Command );
					// if two-button setup lacks dedicated closing button require the user to press appropriate button manually
					return;
				}
				else
				{
					OnCommand_doorcloseleft(Train, Command);
				}
			}
			else
			{
				OnCommand_dooropenleft(Train, Command);
			}
		}
		else
		{
			// close
			if (Train->ggDoorAllOffButton.SubModel != nullptr && Train->ggDoorLeftOffButton.SubModel == nullptr)
			{
				// OnCommand_doorcloseall( Train, Command );
				// if two-button setup lacks dedicated closing button require the user to press appropriate button manually
				return;
			}
			else
			{
				OnCommand_doorcloseleft(Train, Command);
			}
		}
		// visual feedback
		// dedicated closing buttons are presumed to be impulse switches and return automatically to neutral position
		// NOTE: temporary arrangement, can be removed when LD system is in place
		if (Train->ggDoorLeftOffButton.SubModel)
			Train->ggDoorLeftOffButton.UpdateValue(0.0, Train->dsbSwitch);
		if (Train->ggDoorLeftOnButton.SubModel)
			Train->ggDoorLeftOnButton.UpdateValue(0.0, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_doorpermitleft(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_REPEAT)
	{
		return;
	}
	if (false == Train->mvOccupied->Doors.permit_presets.empty())
	{
		return;
	}

	auto const side{(Train->cab_to_end() == end::front ? side::left : side::right)};

	if (Command.action == GLFW_PRESS)
	{

		if (Train->ggDoorLeftPermitButton.is_push())
		{
			// impulse switch
			Train->mvOccupied->PermitDoors(side);
			// visual feedback
			Train->ggDoorLeftPermitButton.UpdateValue(1.0, Train->dsbSwitch);
			// start potential timer for remote door control
			Train->m_doorpermittimers[side] = Train->mvOccupied->DoorsOpenWithPermitAfter;
		}
		else
		{
			// two-state switch
			auto const newstate{!(Train->ggDoorLeftPermitButton.GetDesiredValue() > 0.5)};

			Train->mvOccupied->PermitDoors(side, newstate);
			// visual feedback
			Train->ggDoorLeftPermitButton.UpdateValue(newstate ? 1.0 : 0.0, Train->dsbSwitch);
		}
	}
	else if (Command.action == GLFW_RELEASE && Train->ggDoorLeftPermitButton.is_push())
	{

		// impulse switch
		// visual feedback
		Train->ggDoorLeftPermitButton.UpdateValue(0.0, Train->dsbSwitch);
		// reset potential remote door control timer
		Train->m_doorpermittimers[side] = -1.f;
	}
}

void TTrain::OnCommand_doorpermitright(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_REPEAT)
	{
		return;
	}
	if (false == Train->mvOccupied->Doors.permit_presets.empty())
	{
		return;
	}

	auto const side{(Train->cab_to_end() == end::front ? side::right : side::left)};

	if (Command.action == GLFW_PRESS)
	{

		if (Train->ggDoorRightPermitButton.type() == TGaugeType::push)
		{
			// impulse switch
			Train->mvOccupied->PermitDoors(side);
			// visual feedback
			Train->ggDoorRightPermitButton.UpdateValue(1.0, Train->dsbSwitch);
			// start potential timer for remote door control
			Train->m_doorpermittimers[side] = Train->mvOccupied->DoorsOpenWithPermitAfter;
		}
		else
		{
			// two-state switch
			auto const newstate{!(Train->ggDoorRightPermitButton.GetDesiredValue() > 0.5)};

			Train->mvOccupied->PermitDoors(side, newstate);
			// visual feedback
			Train->ggDoorRightPermitButton.UpdateValue(newstate ? 1.0 : 0.0, Train->dsbSwitch);
		}
	}
	else if (Command.action == GLFW_RELEASE && Train->ggDoorRightPermitButton.type() == TGaugeType::push)
	{

		// impulse switch
		// visual feedback
		Train->ggDoorRightPermitButton.UpdateValue(0.0, Train->dsbSwitch);
		// reset potential remote door control timer
		Train->m_doorpermittimers[side] = -1.f;
	}
}

void TTrain::OnCommand_doorpermitpresetactivatenext(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{

		Train->mvOccupied->ChangeDoorPermitPreset(1);
		// visual feedback
		Train->ggDoorPermitPresetButton.UpdateValue(Train->mvOccupied->Doors.permit_preset, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_doorpermitpresetactivateprevious(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{

		Train->mvOccupied->ChangeDoorPermitPreset(-1);
		// visual feedback
		Train->ggDoorPermitPresetButton.UpdateValue(Train->mvOccupied->Doors.permit_preset, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_dooropenleft(TTrain *Train, command_data const &Command)
{

	auto const remoteopencontrol{Train->mvOccupied->Doors.open_control == control_t::driver || Train->mvOccupied->Doors.open_control == control_t::mixed};

	if (false == remoteopencontrol)
	{
		return;
	}

	if (Train->ggDoorLeftOnButton.SubModel == nullptr && Train->ggDoorLeftButton.SubModel == nullptr)
	{

		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		Train->mvOccupied->OperateDoors(Train->cab_to_end() == end::front ? side::left : side::right, true);
		// visual feedback
		if (Train->ggDoorLeftOnButton.SubModel != nullptr)
		{
			// two separate impulse switches
			Train->ggDoorLeftOnButton.UpdateValue(1.0, Train->dsbSwitch);
		}
		else
		{
			// single two-state switch
			Train->ggDoorLeftButton.UpdateValue(1.0, Train->dsbSwitch);
		}
	}
	else if (Command.action == GLFW_RELEASE && Train->ggDoorLeftOnButton.SubModel != nullptr)
	{
		// visual feedback
		// two separate impulse switches
		Train->ggDoorLeftOnButton.UpdateValue(0.0, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_doorcloseleft(TTrain *Train, command_data const &Command)
{

	auto const remoteclosecontrol{Train->mvOccupied->Doors.close_control == control_t::driver || Train->mvOccupied->Doors.close_control == control_t::mixed};

	if (false == remoteclosecontrol)
	{
		return;
	}

	if (Train->ggDoorLeftOffButton.SubModel == nullptr && Train->ggDoorLeftButton.SubModel == nullptr)
	{

		return;
	}

	if (Command.action == GLFW_PRESS)
	{

		if (Train->mvOccupied->Doors.has_autowarning)
		{
			// automatic departure signal delays actual door closing until the button is released
			Train->mvOccupied->signal_departure(true);
		}
		else
		{
			// TODO: move door opening/closing to the update, so the switch animation doesn't hinge on door working
			Train->mvOccupied->OperateDoors(Train->cab_to_end() == end::front ? side::left : side::right, false);
		}
		// visual feedback
		if (Train->ggDoorLeftOffButton.SubModel != nullptr)
		{
			// two separate switches to open and close the door
			Train->ggDoorLeftOffButton.UpdateValue(1.0, Train->dsbSwitch);
		}
		else
		{
			// single two-state switch
			Train->ggDoorLeftButton.UpdateValue(0.0, Train->dsbSwitch);
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{

		if (Train->mvOccupied->Doors.has_autowarning)
		{
			// automatic departure signal delays actual door closing until the button is released
			Train->mvOccupied->signal_departure(false);
			// now we can actually close the door
			Train->mvOccupied->OperateDoors(Train->cab_to_end() == end::front ? side::left : side::right, false);
		}
		// visual feedback
		// dedicated closing buttons are presumed to be impulse switches and return automatically to neutral position
		if (Train->ggDoorLeftOffButton.SubModel)
			Train->ggDoorLeftOffButton.UpdateValue(0.0, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_doortoggleright(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// NOTE: test how the door state check works with consists where the occupied vehicle doesn't have opening doors
		if (false == (Train->ggDoorRightButton.GetDesiredValue() > 0.5 || Train->ggDoorRightOnButton.GetDesiredValue() > 0.5))
		{
			// open
			OnCommand_dooropenright(Train, Command);
		}
		else
		{
			// close
			if (Train->ggDoorAllOffButton.SubModel != nullptr && Train->ggDoorRightOffButton.SubModel == nullptr)
			{
				// OnCommand_doorcloseall( Train, Command );
				// if two-button setup lacks dedicated closing button require the user to press appropriate button manually
				return;
			}
			else
			{
				OnCommand_doorcloseright(Train, Command);
			}
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{

		if (true == (Train->ggDoorRightButton.GetDesiredValue() > 0.5 || Train->ggDoorRightOnButton.GetDesiredValue() > 0.5))
		{
			// open
			if (Train->mvOccupied->Doors.has_autowarning && Train->mvOccupied->DepartureSignal)
			{
				// complete closing the doors
				if (Train->ggDoorAllOffButton.SubModel != nullptr && Train->ggDoorRightOffButton.SubModel == nullptr)
				{
					// OnCommand_doorcloseall( Train, Command );
					// if two-button setup lacks dedicated closing button require the user to press appropriate button manually
					return;
				}
				else
				{
					OnCommand_doorcloseright(Train, Command);
				}
			}
			else
			{
				OnCommand_dooropenright(Train, Command);
			}
		}
		else
		{
			// close
			if (Train->ggDoorAllOffButton.SubModel != nullptr && Train->ggDoorRightOffButton.SubModel == nullptr)
			{
				// OnCommand_doorcloseall( Train, Command );
				// if two-button setup lacks dedicated closing button require the user to press appropriate button manually
				return;
			}
			else
			{
				OnCommand_doorcloseright(Train, Command);
			}
		}
		// visual feedback
		// dedicated closing buttons are presumed to be impulse switches and return automatically to neutral position
		// NOTE: temporary arrangement, can be removed when LD system is in place
		if (Train->ggDoorRightOffButton.SubModel)
			Train->ggDoorRightOffButton.UpdateValue(0.0, Train->dsbSwitch);
		if (Train->ggDoorRightOnButton.SubModel)
			Train->ggDoorRightOnButton.UpdateValue(0.0, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_dooropenright(TTrain *Train, command_data const &Command)
{

	auto const remoteopencontrol{Train->mvOccupied->Doors.open_control == control_t::driver || Train->mvOccupied->Doors.open_control == control_t::mixed};

	if (false == remoteopencontrol)
	{
		return;
	}

	if (Train->ggDoorRightOnButton.SubModel == nullptr && Train->ggDoorRightButton.SubModel == nullptr)
	{

		return;
	}

	if (Command.action == GLFW_PRESS)
	{

		Train->mvOccupied->OperateDoors(Train->cab_to_end() == end::front ? side::right : side::left, true);
		// visual feedback
		if (Train->ggDoorRightOnButton.SubModel != nullptr)
		{
			// two separate impulse switches
			Train->ggDoorRightOnButton.UpdateValue(1.0, Train->dsbSwitch);
		}
		else
		{
			// single two-state switch
			Train->ggDoorRightButton.UpdateValue(1.0, Train->dsbSwitch);
		}
	}
	else if (Command.action == GLFW_RELEASE && Train->ggDoorRightOnButton.SubModel != nullptr)
	{
		// visual feedback
		// two separate impulse switches
		Train->ggDoorRightOnButton.UpdateValue(0.0, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_doorcloseright(TTrain *Train, command_data const &Command)
{

	auto const remoteclosecontrol{Train->mvOccupied->Doors.close_control == control_t::driver || Train->mvOccupied->Doors.close_control == control_t::mixed};

	if (false == remoteclosecontrol)
	{
		return;
	}

	if (Train->ggDoorRightOffButton.SubModel == nullptr && Train->ggDoorRightButton.SubModel == nullptr)
	{

		return;
	}

	if (Command.action == GLFW_PRESS)
	{

		if (Train->mvOccupied->Doors.has_autowarning)
		{
			// automatic departure signal delays actual door closing until the button is released
			Train->mvOccupied->signal_departure(true);
		}
		else
		{
			Train->mvOccupied->OperateDoors(Train->cab_to_end() == end::front ? side::right : side::left, false);
		}
		// visual feedback
		if (Train->ggDoorRightOffButton.SubModel != nullptr)
		{
			// two separate switches to open and close the door
			Train->ggDoorRightOffButton.UpdateValue(1.0, Train->dsbSwitch);
		}
		else
		{
			// single two-state switch
			Train->ggDoorRightButton.UpdateValue(0.0, Train->dsbSwitch);
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{

		if (Train->mvOccupied->Doors.has_autowarning)
		{
			// automatic departure signal delays actual door closing until the button is released
			Train->mvOccupied->signal_departure(false);
			// now we can actually close the door
			Train->mvOccupied->OperateDoors(Train->cab_to_end() == end::front ? side::right : side::left, false);
		}
		// visual feedback
		// dedicated closing buttons are presumed to be impulse switches and return automatically to neutral position
		if (Train->ggDoorRightOffButton.SubModel)
			Train->ggDoorRightOffButton.UpdateValue(0.0, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_dooropenall(TTrain *Train, command_data const &Command)
{

	auto const remoteopencontrol{Train->mvOccupied->Doors.open_control == control_t::driver || Train->mvOccupied->Doors.open_control == control_t::mixed};

	if (false == remoteopencontrol)
	{
		return;
	}

	if (Train->ggDoorAllOnButton.SubModel == nullptr)
	{
		// TODO: expand definition of cab controls so we can know if the control is present without testing for presence of 3d switch
		if (Command.action == GLFW_PRESS)
		{
			WriteLog("Open All Doors switch is missing, or wasn't defined");
		}
		return;
	}

	if (Command.action == GLFW_PRESS)
	{

		Train->mvOccupied->OperateDoors(side::right, true);
		Train->mvOccupied->OperateDoors(side::left, true);
		// visual feedback
		Train->ggDoorAllOnButton.UpdateValue(1.0, Train->dsbSwitch);
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// visual feedback
		Train->ggDoorAllOnButton.UpdateValue(0.0);
	}
}

void TTrain::OnCommand_doorcloseall(TTrain *Train, command_data const &Command)
{

	auto const remoteclosecontrol{Train->mvOccupied->Doors.close_control == control_t::driver || Train->mvOccupied->Doors.close_control == control_t::mixed};

	if (false == remoteclosecontrol)
	{
		return;
	}

	if (Train->ggDoorAllOffButton.SubModel == nullptr)
	{
		// TODO: expand definition of cab controls so we can know if the control is present without testing for presence of 3d switch
		if (Command.action == GLFW_PRESS)
		{
			WriteLog("Close All Doors switch is missing, or wasn't defined");
		}
		return;
	}

	if (Command.action == GLFW_PRESS)
	{

		if (Train->mvOccupied->Doors.has_autowarning)
		{
			Train->mvOccupied->signal_departure(true);
		}
		if (Train->ggDoorAllOffButton.type() != TGaugeType::push_delayed)
		{
			// delays the action until the button is released
			Train->mvOccupied->OperateDoors(side::right, false);
			Train->mvOccupied->OperateDoors(side::left, false);
		}
		// visual feedback
		Train->ggDoorLeftButton.UpdateValue(0.0, Train->dsbSwitch);
		Train->ggDoorRightButton.UpdateValue(0.0, Train->dsbSwitch);
		if (Train->ggDoorAllOffButton.SubModel)
			Train->ggDoorAllOffButton.UpdateValue(1.0, Train->dsbSwitch);
	}
	else if (Command.action == GLFW_RELEASE)
	{
		if (Train->mvOccupied->Doors.has_autowarning)
		{
			Train->mvOccupied->signal_departure(false);
		}
		if (Train->ggDoorAllOffButton.type() == TGaugeType::push_delayed)
		{
			// now we can actually close the door
			Train->mvOccupied->OperateDoors(side::right, false);
			Train->mvOccupied->OperateDoors(side::left, false);
		}
		// visual feedback
		if (Train->ggDoorAllOffButton.SubModel)
			Train->ggDoorAllOffButton.UpdateValue(0.0);
	}
}

void TTrain::OnCommand_doorsteptoggle(TTrain *Train, command_data const &Command)
{
	// TODO: move logic/visualization code to the gauge, on_command() should return hint whether it should invoke a reaction
	if (Command.action == GLFW_PRESS)
	{
		// effect
		if (false == Train->ggDoorStepButton.is_delayed())
		{
			Train->mvOccupied->PermitDoorStep(false == Train->mvOccupied->Doors.step_enabled);
		}
		// visual feedback
		auto const isactive{(Train->ggDoorStepButton.is_push() // always press push button
		                     || Train->mvOccupied->Doors.step_enabled)}; // for toggle buttons indicate item state
		Train->ggDoorStepButton.UpdateValue(isactive ? 1 : 0);
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// effect
		if (Train->ggDoorStepButton.is_delayed())
		{
			Train->mvOccupied->PermitDoorStep(false == Train->mvOccupied->Doors.step_enabled);
		}
		// visual feedback
		if (Train->ggDoorStepButton.is_push())
		{
			Train->ggDoorStepButton.UpdateValue(0);
		}
	}
}

void TTrain::OnCommand_doormodetoggle(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		Train->mvOccupied->ChangeDoorControlMode(false == Train->mvOccupied->Doors.remote_only);
	}
}

void TTrain::OnCommand_mirrorstoggle(TTrain *Train, command_data const &Command)
{

	if (Command.action != GLFW_PRESS)
	{
		return;
	}

	// only reacting to press, so the sound can loop uninterrupted
	if (false == Train->mvOccupied->MirrorForbidden)
	{
		// turn on
		Train->mvOccupied->MirrorForbidden = true;
	}
	else
	{
		// turn off
		Train->mvOccupied->MirrorForbidden = false;
	}
}

void TTrain::OnCommand_nearestcarcouplingincrease(TTrain *Train, command_data const &Command)
{

	if (true == Command.freefly && Command.action == GLFW_PRESS)
	{
		// tryb freefly, press only
		auto coupler{-1};
		auto *vehicle{Train->DynamicObject->ABuScanNearestObject(Command.location, Train->DynamicObject->GetTrack(), 1, 1500, coupler)};
		if (vehicle == nullptr)
			vehicle = Train->DynamicObject->ABuScanNearestObject(Command.location, Train->DynamicObject->GetTrack(), -1, 1500, coupler);

		if (coupler != -1 && vehicle != nullptr)
		{

			vehicle->couple(coupler);
		}
		if (Train->DynamicObject->Mechanik)
		{
			// aktualizacja flag kierunku w składzie
			Train->DynamicObject->Mechanik->CheckVehicles(Connect);
		}
	}
}

void TTrain::OnCommand_nearestcarcouplingdisconnect(TTrain *Train, command_data const &Command)
{

	if (true == Command.freefly && Command.action == GLFW_PRESS)
	{
		// tryb freefly, press only
		auto coupler{-1};
		auto *vehicle{Train->DynamicObject->ABuScanNearestObject(Command.location, Train->DynamicObject->GetTrack(), 1, 1500, coupler)};
		if (vehicle == nullptr)
			vehicle = Train->DynamicObject->ABuScanNearestObject(Command.location, Train->DynamicObject->GetTrack(), -1, 1500, coupler);

		if (coupler != -1 && vehicle != nullptr)
		{

			vehicle->uncouple(coupler);
		}
		if (Train->DynamicObject->Mechanik)
		{
			// aktualizacja flag kierunku w składzie
			Train->DynamicObject->Mechanik->CheckVehicles(Disconnect);
		}
	}
}

void TTrain::OnCommand_nearestcarcoupleradapterattach(TTrain *Train, command_data const &Command)
{

	if (true == Command.freefly && Command.action == GLFW_PRESS)
	{
		// tryb freefly, press only
		auto *vehicle{std::get<TDynamicObject *>(simulation::Region->find_vehicle(Command.location, 50, false, true))};
		if (vehicle == nullptr)
		{
			return;
		}

		auto const coupler =
		    glm::length2(glm::vec3{vehicle->CouplerPosition(end::front)} - Command.location) < glm::length2(glm::vec3{vehicle->CouplerPosition(end::rear)} - Command.location) ? end::front : end::rear;

		vehicle->attach_coupler_adapter(coupler);
	}
}

void TTrain::OnCommand_nearestcarcoupleradapterremove(TTrain *Train, command_data const &Command)
{

	if (true == Command.freefly && Command.action == GLFW_PRESS)
	{
		// tryb freefly, press only
		auto *vehicle{std::get<TDynamicObject *>(simulation::Region->find_vehicle(Command.location, 50, false, true))};
		if (vehicle == nullptr)
		{
			return;
		}

		auto const coupler =
		    glm::length2(glm::vec3{vehicle->CouplerPosition(end::front)} - Command.location) < glm::length2(glm::vec3{vehicle->CouplerPosition(end::rear)} - Command.location) ? end::front : end::rear;

		vehicle->remove_coupler_adapter(coupler);
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
				Train->DynamicObject->Mechanik->CheckVehicles(Disconnect);
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
				Train->DynamicObject->Mechanik->CheckVehicles(Disconnect);
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
