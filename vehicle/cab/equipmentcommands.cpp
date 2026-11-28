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

// the commands of the heating, the generic switches, the spring brake, the speed control and the inverters

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


void TTrain::OnCommand_heatingtoggle(TTrain *Train, command_data const &Command)
{

	if (Train->ggTrainHeatingButton.SubModel == nullptr)
	{
		if (Command.action == GLFW_PRESS)
		{
			WriteLog("Train Heating switch is missing, or wasn't defined");
		}
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// ignore repeats so the switch doesn't flip back and forth if key is held down
		if (false == Train->mvControlled->HeatingAllow)
		{
			// turn on
			OnCommand_heatingenable(Train, Command);
		}
		else
		{
			// turn off
			OnCommand_heatingdisable(Train, Command);
		}
	}
	else if (Command.action == GLFW_RELEASE && (Train->ggTrainHeatingButton.type() == TGaugeType::push || Train->mvOccupied->HeatingSwitchImpulse))
	{
		// impulse switch
		// visual feedback
		Train->ggTrainHeatingButton.UpdateValue(0.0, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_heatingenable(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{

		Train->mvOccupied->HeatingSwitch(true);
		// visual feedback
		Train->ggTrainHeatingButton.UpdateValue(1.0, Train->dsbSwitch);
	}
	else if (Command.action == GLFW_RELEASE && Train->mvOccupied->HeatingSwitchImpulse)
	{
		// the switch springs back to the middle
		Train->ggTrainHeatingButton.UpdateValue(0.0, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_heatingdisable(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{

		Train->mvOccupied->HeatingSwitch(false);
		// visual feedback: an impulse switch is pushed the other way, down
		auto position{0.0};
		if (Train->mvOccupied->HeatingSwitchImpulse)
		{
			position = -1.0;
		}
		else if (Train->ggTrainHeatingButton.type() == TGaugeType::push)
		{
			position = 1.0;
		}
		Train->ggTrainHeatingButton.UpdateValue(position, Train->dsbSwitch);
	}
	else if (Command.action == GLFW_RELEASE && Train->mvOccupied->HeatingSwitchImpulse)
	{
		Train->ggTrainHeatingButton.UpdateValue(0.0, Train->dsbSwitch);
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
			// two-state switch, or one dragged in between its ends with the mouse: to the end farther from it
			// visual feedback
			item.UpdateValue(universal_drag::toggled(item.GetDesiredValue()));
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

// sets a universal control to the position given in param1, from 0 to 1 (MaSzyna-EU07/maszyna#62): the mouse drags
// the ones with drag: in their definition, and it can be bound to an axis of a gamepad. it stays where it is set
void TTrain::OnCommand_genericset(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_RELEASE)
	{
		return;
	}
	auto const itemindex = static_cast<int>(Command.command) - static_cast<int>(user_command::genericset0);
	auto &item = Train->ggUniversals[itemindex];
	if (item.is_push())
	{
		// an impulse switch springs back, it has no position to be left in
		return;
	}
	// visual feedback
	item.UpdateValue(universal_drag::clamp(Command.param1));
}

universal_drag::direction TTrain::universal_drag_direction(int const Index) const
{

	if (Index < 0 || Index >= static_cast<int>(ggUniversals.size()) || ggUniversals[Index].SubModel == nullptr)
	{
		return universal_drag::direction::none;
	}
	return ggUniversals[Index].drag();
}

float TTrain::universal_position(int const Index) const
{

	if (Index < 0 || Index >= static_cast<int>(ggUniversals.size()))
	{
		return 0.f;
	}
	return ggUniversals[Index].GetDesiredValue();
}

void TTrain::OnCommand_springbraketoggle(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		if (false == Train->mvOccupied->SpringBrake.Activate)
		{
			// turn on
			OnCommand_springbrakeenable(Train, Command);
		}
		else
		{
			// turn off
			OnCommand_springbrakedisable(Train, Command);
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// release
		// visual feedback
		Train->ggSpringBrakeOffButton.UpdateValue(0.0, Train->dsbSwitch);
		Train->ggSpringBrakeOnButton.UpdateValue(0.0, Train->dsbSwitch);
	}
};

void TTrain::OnCommand_springbrakeenable(TTrain *Train, command_data const &Command)
{
	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		Train->mvOccupied->SpringBrakeActivate(true);
		// visual feedback
		Train->ggSpringBrakeOnButton.UpdateValue(1.0, Train->dsbSwitch);
		Train->ggSpringBrakeOffButton.UpdateValue(0.0, Train->dsbSwitch);
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// release
		// visual feedback
		Train->ggSpringBrakeOnButton.UpdateValue(0.0, Train->dsbSwitch);
	}
};

void TTrain::OnCommand_springbrakedisable(TTrain *Train, command_data const &Command)
{
	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		Train->mvOccupied->SpringBrakeActivate(false);
		// visual feedback
		Train->ggSpringBrakeOffButton.UpdateValue(1.0, Train->dsbSwitch);
		Train->ggSpringBrakeOnButton.UpdateValue(0.0, Train->dsbSwitch);
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// release
		// visual feedback
		Train->ggSpringBrakeOffButton.UpdateValue(0.0, Train->dsbSwitch);
	}
};
void TTrain::OnCommand_springbrakeshutofftoggle(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		if (false == Train->mvOccupied->SpringBrake.ShuttOff)
		{
			// turn on
			OnCommand_springbrakeshutoffenable(Train, Command);
		}
		else
		{
			// turn off
			OnCommand_springbrakeshutoffdisable(Train, Command);
		}
	}
};

void TTrain::OnCommand_springbrakeshutoffenable(TTrain *Train, command_data const &Command)
{
	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		Train->mvOccupied->SpringBrakeShutOff(true);
	}
};

void TTrain::OnCommand_springbrakeshutoffdisable(TTrain *Train, command_data const &Command)
{
	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		Train->mvOccupied->SpringBrakeShutOff(false);
	}
};

void TTrain::OnCommand_springbrakerelease(TTrain *Train, command_data const &Command)
{
	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down

		if (auto const *vehicle{Train->find_nearest_consist_vehicle(Command.freefly, Command.location)}; vehicle == nullptr)
		{
			return;
		}
		Train->mvOccupied->SpringBrakeRelease();
	}
};

void TTrain::OnCommand_speedcontrolincrease(TTrain *Train, command_data const &Command)
{
	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		Train->mvOccupied->SpeedCtrlInc();
		// visual feedback
		Train->ggSpeedControlIncreaseButton.UpdateValue(1.0, Train->dsbSwitch);
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// release
		// visual feedback
		Train->ggSpeedControlIncreaseButton.UpdateValue(0.0, Train->dsbSwitch);
	}
};

void TTrain::OnCommand_speedcontroldecrease(TTrain *Train, command_data const &Command)
{
	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		Train->mvOccupied->SpeedCtrlDec();
		// visual feedback
		Train->ggSpeedControlDecreaseButton.UpdateValue(1.0, Train->dsbSwitch);
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// release
		// visual feedback
		Train->ggSpeedControlDecreaseButton.UpdateValue(0.0, Train->dsbSwitch);
	}
};

void TTrain::OnCommand_speedcontrolpowerincrease(TTrain *Train, command_data const &Command)
{
	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		Train->mvOccupied->SpeedCtrlPowerInc();
		// visual feedback
		Train->ggSpeedControlPowerIncreaseButton.UpdateValue(1.0, Train->dsbSwitch);
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// release
		// visual feedback
		Train->ggSpeedControlPowerIncreaseButton.UpdateValue(0.0, Train->dsbSwitch);
	}
};

void TTrain::OnCommand_speedcontrolpowerdecrease(TTrain *Train, command_data const &Command)
{
	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		Train->mvOccupied->SpeedCtrlPowerDec();
		// visual feedback
		Train->ggSpeedControlPowerDecreaseButton.UpdateValue(1.0, Train->dsbSwitch);
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// release
		// visual feedback
		Train->ggSpeedControlPowerDecreaseButton.UpdateValue(0.0, Train->dsbSwitch);
	}
};

void TTrain::OnCommand_speedcontrolbutton(TTrain *Train, command_data const &Command)
{

	auto const itemindex = static_cast<int>(Command.command) - static_cast<int>(user_command::speedcontrolbutton0);

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		Train->mvOccupied->SpeedCtrlButton(itemindex);
		// visual feedback
		Train->ggSpeedCtrlButtons[itemindex].UpdateValue(1.0, Train->dsbSwitch);
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// release
		// visual feedback
		Train->ggSpeedCtrlButtons[itemindex].UpdateValue(0.0, Train->dsbSwitch);
	}
};

void TTrain::OnCommand_inverterenable(TTrain *Train, command_data const &Command)
{

	auto itemindex = static_cast<int>(Command.command) - static_cast<int>(user_command::inverterenable1);
	auto &item = Train->ggInverterEnableButtons[itemindex];

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		bool kier = Train->DynamicObject->DirectionGet() * Train->mvOccupied->CabOccupied > 0;
		auto const flag{CouplingSet::FromBits(Train->DynamicObject->MoverParameters->InverterControlCouplerFlag)};
		TDynamicObject *p = Train->DynamicObject->GetFirstDynamic(Train->mvOccupied->CabOccupied < 0 ? end::rear : end::front, flag);
		while (p)
		{
			if (p->MoverParameters->eimc[eimc_p_Pmax] > 1)
			{
				if (itemindex < p->MoverParameters->InvertersNo)
				{
					p->MoverParameters->Inverters[itemindex].Activate = true;
					break;
				}
				else
				{
					itemindex -= p->MoverParameters->InvertersNo;
				}
			}
			p = kier ? p->Next(flag) : p->Prev(flag);
		}
		// visual feedback
		item.UpdateValue(1.0, Train->dsbSwitch);
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// release
		// visual feedback
		item.UpdateValue(0.0, Train->dsbSwitch);
	}
};

void TTrain::OnCommand_inverterdisable(TTrain *Train, command_data const &Command)
{

	auto itemindex = static_cast<int>(Command.command) - static_cast<int>(user_command::inverterdisable1);
	auto &item = Train->ggInverterDisableButtons[itemindex];

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		bool kier = Train->DynamicObject->DirectionGet() * Train->mvOccupied->CabOccupied > 0;
		auto const flag{CouplingSet::FromBits(Train->DynamicObject->MoverParameters->InverterControlCouplerFlag)};
		TDynamicObject *p = Train->DynamicObject->GetFirstDynamic(Train->mvOccupied->CabOccupied < 0 ? end::rear : end::front, flag);
		while (p)
		{
			if (p->MoverParameters->eimc[eimc_p_Pmax] > 1)
			{
				if (itemindex < p->MoverParameters->InvertersNo)
				{
					p->MoverParameters->Inverters[itemindex].Activate = false;
					break;
				}
				else
				{
					itemindex -= p->MoverParameters->InvertersNo;
				}
			}
			p = kier ? p->Next(flag) : p->Prev(flag);
		}
		// visual feedback
		item.UpdateValue(1.0, Train->dsbSwitch);
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// release
		// visual feedback
		item.UpdateValue(0.0, Train->dsbSwitch);
	}
};

void TTrain::OnCommand_invertertoggle(TTrain *Train, command_data const &Command)
{

	auto itemindex = static_cast<int>(Command.command) - static_cast<int>(user_command::invertertoggle1);
	auto &item = Train->ggInverterToggleButtons[itemindex];

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		bool kier = Train->DynamicObject->DirectionGet() * Train->mvOccupied->CabOccupied > 0;
		auto const flag{CouplingSet::FromBits(Train->DynamicObject->MoverParameters->InverterControlCouplerFlag)};
		TDynamicObject *p = Train->DynamicObject->GetFirstDynamic(Train->mvOccupied->CabOccupied < 0 ? end::rear : end::front, flag);
		while (p)
		{
			if (p->MoverParameters->eimc[eimc_p_Pmax] > 1)
			{
				if (itemindex < p->MoverParameters->InvertersNo)
				{
					p->MoverParameters->Inverters[itemindex].Activate = !p->MoverParameters->Inverters[itemindex].Activate;
					// visual feedback
					item.UpdateValue(p->MoverParameters->Inverters[itemindex].Activate ? 1.0 : 0.0, Train->dsbSwitch);
					break;
				}
				else
				{
					itemindex -= p->MoverParameters->InvertersNo;
				}
			}
			p = kier ? p->Next(flag) : p->Prev(flag);
		}
	}
};
