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

// the commands of the controllers: the master, the secondary and the dynamic brake one, the speed regulator

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


// command handlers
void TTrain::OnCommand_aidriverenable(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// on press
		if (Train->DynamicObject->Mechanik == nullptr)
		{
			return;
		}

		if (true == Train->DynamicObject->Mechanik->AIControllFlag)
		{
			// żeby nie trzeba było rozłączać dla zresetowania
			Train->DynamicObject->Mechanik->TakeControl(false);
		}
		Train->DynamicObject->Mechanik->TakeControl(true);
	}
}

void TTrain::OnCommand_aidriverdisable(TTrain *Train, command_data const &Command)
{

	// on press
	if (Command.action == GLFW_PRESS && Train->DynamicObject->Mechanik)
		Train->DynamicObject->Mechanik->TakeControl(false);
}
auto const EU07_CONTROLLER_KEYBOARDETURNDELAY{1.5f};

void TTrain::OnCommand_jointcontrollerset(TTrain *Train, command_data const &Command)
{

	if (Command.action != GLFW_RELEASE)
	{
		// on press or hold
		// value controls brake in range 0-0.5, master controller in range 0.5-1.0
		if (Command.param1 >= 0.5)
		{
			Train->set_master_controller((Command.param1 * 2 - 1) *
			                             (Train->mvControlled->CoupledCtrl ? Train->mvControlled->MainCtrlPosNo + Train->mvControlled->ScndCtrlPosNo : Train->mvControlled->MainCtrlPosNo));
			Train->m_mastercontrollerinuse = true;
			// when SplitEDPneumaticBrake is active the joint controller's negative range
			// commands the dedicated dynamic-brake lever instead of the local pneumatic brake
			if (Train->mvControlled->SplitEDPneumaticBrake)
			{
				Train->mvControlled->DynamicBrakeLevelSet(0.0);
			}
			else
			{
				Train->mvOccupied->LocalBrakePosA = 0;
			}
		}
		else
		{
			auto const negativeRange{std::clamp(1.0 - Command.param1 * 2, 0.0, 1.0)};
			if (Train->mvControlled->SplitEDPneumaticBrake)
			{
				// negative range of jointctrl drives only ED braking, local pneumatic brake stays untouched.
				// snap to DBPN discrete stops so the lever animates step-by-step
				auto const stepCount{std::max(1, Train->mvControlled->DynamicBrakeCtrlPosNo)};
				auto const snapped{std::round(negativeRange * stepCount) / stepCount};
				Train->mvControlled->DynamicBrakeLevelSet(snapped);
			}
			else
			{
				Train->mvOccupied->LocalBrakePosA = negativeRange;
			}
			if (Train->mvControlled->MainCtrlPowerPos() > 0)
			{
				Train->set_master_controller(Train->mvControlled->MainCtrlNoPowerPos());
			}
		}
	}
	else
	{
		// release
		Train->m_mastercontrollerinuse = false;
		Train->m_mastercontrollerreturndelay = EU07_CONTROLLER_BASERETURNDELAY; // NOTE: keyboard return delay is omitted for other input sources
	}
}

void TTrain::OnCommand_mastercontrollerincrease(TTrain *Train, command_data const &Command)
{

	if (Command.action != GLFW_RELEASE)
	{
		// on press or hold
		auto const splitMode{Train->mvControlled->SplitEDPneumaticBrake};
		// when SplitEDPneumaticBrake is true the joint controller's negative range maps
		// to the dedicated dynamic-brake lever (DynamicBrakeCtrl) rather than to LocalBrake
		auto const negativeRangeActive{splitMode ? Train->mvControlled->DynamicBrakeCtrlPos > 0.0 : Train->mvOccupied->LocalBrakePosA > 0.0};
		if (Train->ggJointCtrl.SubModel != nullptr && negativeRangeActive)
		{
			if (splitMode)
			{
				OnCommand_DynamicBrakeControllerDecrease(Train, Command);
				Train->m_mastercontrollerinuse = true;
			}
			else
			{
				OnCommand_independentbrakedecrease(Train, Command);
			}
		}
		else
		{
			Train->mvControlled->IncMainCtrl(1);
			Train->m_mastercontrollerinuse = true;
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// release
		Train->m_mastercontrollerinuse = false;
		Train->m_mastercontrollerreturndelay = EU07_CONTROLLER_KEYBOARDETURNDELAY + EU07_CONTROLLER_BASERETURNDELAY;
	}
}

void TTrain::OnCommand_mastercontrollerincreasefast(TTrain *Train, command_data const &Command)
{

	if (Command.action != GLFW_RELEASE)
	{
		// on press or hold
		auto const splitMode{Train->mvControlled->SplitEDPneumaticBrake};
		auto const negativeRangeActive{splitMode ? Train->mvControlled->DynamicBrakeCtrlPos > 0.0 : Train->mvOccupied->LocalBrakePosA > 0.0};
		if (Train->ggJointCtrl.SubModel != nullptr && negativeRangeActive)
		{
			if (splitMode)
			{
				OnCommand_DynamicBrakeControllerDecreaseFast(Train, Command);
				Train->m_mastercontrollerinuse = true;
			}
			else
			{
				OnCommand_independentbrakedecreasefast(Train, Command);
			}
		}
		else
		{
			Train->mvControlled->IncMainCtrl(Train->mvControlled->MainCtrlPosNo);
			Train->m_mastercontrollerinuse = true;
		}
	}
	else
	{
		// release
		Train->m_mastercontrollerinuse = false;
		Train->m_mastercontrollerreturndelay = EU07_CONTROLLER_KEYBOARDETURNDELAY + EU07_CONTROLLER_BASERETURNDELAY;
	}
}

void TTrain::OnCommand_mastercontrollerdecrease(TTrain *Train, command_data const &Command)
{

	if (Command.action != GLFW_RELEASE)
	{
		// on press or hold
		auto const splitMode{Train->mvControlled->SplitEDPneumaticBrake};
		if (Train->ggJointCtrl.SubModel != nullptr && Train->mvControlled->IsMainCtrlNoPowerPos())
		{
			// negative range of jointctrl: ED brake when split, otherwise pneumatic local brake
			if (splitMode)
			{
				OnCommand_DynamicBrakeControllerIncrease(Train, Command);
				Train->m_mastercontrollerinuse = true;
			}
			else
			{
				OnCommand_independentbrakeincrease(Train, Command);
			}
		}
		else
		{
			Train->mvControlled->DecMainCtrl(1);
			Train->m_mastercontrollerinuse = true;
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// release
		Train->m_mastercontrollerinuse = false;
		Train->m_mastercontrollerreturndelay = EU07_CONTROLLER_KEYBOARDETURNDELAY + EU07_CONTROLLER_BASERETURNDELAY;
	}
}

void TTrain::OnCommand_mastercontrollerdecreasefast(TTrain *Train, command_data const &Command)
{

	if (Command.action != GLFW_RELEASE)
	{
		// on press or hold
		auto const splitMode{Train->mvControlled->SplitEDPneumaticBrake};
		if (Train->ggJointCtrl.SubModel != nullptr && Train->mvControlled->IsMainCtrlNoPowerPos())
		{
			if (splitMode)
			{
				OnCommand_DynamicBrakeControllerIncreaseFast(Train, Command);
				Train->m_mastercontrollerinuse = true;
			}
			else
			{
				OnCommand_independentbrakeincreasefast(Train, Command);
			}
		}
		else
		{
			Train->mvControlled->DecMainCtrl(Train->mvControlled->MainCtrlPowerPos());
			Train->m_mastercontrollerinuse = true;
		}
	}
	else
	{
		// release
		Train->m_mastercontrollerinuse = false;
		Train->m_mastercontrollerreturndelay = EU07_CONTROLLER_KEYBOARDETURNDELAY + EU07_CONTROLLER_BASERETURNDELAY;
	}
}

void TTrain::OnCommand_mastercontrollerset(TTrain *Train, command_data const &Command)
{

	if (Command.action != GLFW_RELEASE)
	{
		// on press or hold
		Train->set_master_controller(Command.param1);
		Train->m_mastercontrollerinuse = true;
	}
	else
	{
		// release
		Train->m_mastercontrollerinuse = false;
		Train->m_mastercontrollerreturndelay = EU07_CONTROLLER_BASERETURNDELAY; // NOTE: keyboard return delay is omitted for other input sources
	}
}
void TTrain::OnCommand_DynamicBrakeControllerIncrease(TTrain *Train, command_data const &Command)
{
	if (Command.action == GLFW_RELEASE)
	{
		return;
	}
	if (false == Train->mvControlled->SplitEDPneumaticBrake)
	{
		return;
	}

	// step exactly one stop (1 / DBPN) per command, matching the discrete behaviour of jointctrl
	Train->mvControlled->IncDynamicBrakeLevel(1.0f);
}

void TTrain::OnCommand_DynamicBrakeControllerIncreaseFast(TTrain *Train, command_data const &Command)
{
	if (Command.action != GLFW_PRESS)
	{
		return;
	}
	if (false == Train->mvControlled->SplitEDPneumaticBrake)
	{
		return;
	}

	Train->mvControlled->IncDynamicBrakeLevel(static_cast<float>(Train->mvControlled->DynamicBrakeCtrlPosNo));
}

void TTrain::OnCommand_DynamicBrakeControllerDecrease(TTrain *Train, command_data const &Command)
{
	if (Command.action == GLFW_RELEASE)
	{
		return;
	}
	if (false == Train->mvControlled->SplitEDPneumaticBrake)
	{
		return;
	}

	// step exactly one stop (1 / DBPN) per command, matching the discrete behaviour of jointctrl
	Train->mvControlled->DecDynamicBrakeLevel(1.0f);
}

void TTrain::OnCommand_DynamicBrakeControllerDecreaseFast(TTrain *Train, command_data const &Command)
{
	if (Command.action != GLFW_PRESS)
	{
		return;
	}
	if (false == Train->mvControlled->SplitEDPneumaticBrake)
	{
		return;
	}

	Train->mvControlled->DecDynamicBrakeLevel(static_cast<float>(Train->mvControlled->DynamicBrakeCtrlPosNo));
}

void TTrain::OnCommand_DynamicBrakeControllerSet(TTrain *Train, command_data const &Command)
{
	if (Command.action == GLFW_RELEASE)
	{
		return;
	}
	if (false == Train->mvControlled->SplitEDPneumaticBrake)
	{
		return;
	}

	// when input source uses raw 0..1 value, snap to nearest DBPN step
	auto const target{std::clamp(Command.param1, 0.0, 1.0)};
	auto const stepCount{std::max(1, Train->mvControlled->DynamicBrakeCtrlPosNo)};
	auto const snapped{std::round(target * stepCount) / stepCount};
	Train->mvControlled->DynamicBrakeLevelSet(snapped);
}

void TTrain::OnCommand_secondcontrollerincrease(TTrain *Train, command_data const &Command)
{

	if (Train->mvControlled->EngineType == TEngineType::DieselElectric && true == Train->mvControlled->ShuntModeAllow && true == Train->mvControlled->ShuntMode)
	{
		if (Command.action != GLFW_RELEASE)
		{
			Train->mvControlled->AnPos = std::clamp(Train->mvControlled->AnPos + 0.025, 0.0, 1.0);
		}
	}
	else
	{
		// regular mode
		// push or pushtoggle control type
		if (Train->ggScndCtrl.is_push())
		{
			if (Command.action == GLFW_PRESS)
			{
				// activate on press
				Train->mvControlled->IncScndCtrl(1);
			}
		}
		// toggle control type
		else
		{
			if (Command.action != GLFW_RELEASE)
			{
				Train->mvControlled->IncScndCtrl(1);
			}
		}
		// HACK: potentially animate push or pushtoggle control
		if (Train->ggScndCtrl.is_push())
		{
			auto const activeposition{1.f};
			auto const neutralposition{Train->ggScndCtrl.is_toggle() ? 0.5f : 0.f};
			Train->ggScndCtrl.UpdateValue(Command.action == GLFW_RELEASE ? neutralposition : activeposition, Train->dsbSwitch);
		}
		// potentially animate tempomat button, unless the cruise control has its own
		if (Train->ggScndCtrlButton.is_push() && Train->mvControlled->ScndCtrlPos <= 1 && false == Train->mvControlled->IsSpeedCtrlSeparate())
		{
			Train->ggScndCtrlButton.UpdateValue(Command.action == GLFW_RELEASE ? 0.f : 1.f, Train->dsbSwitch);
		}
	}
}

void TTrain::OnCommand_secondcontrollerincreasefast(TTrain *Train, command_data const &Command)
{

	if (Command.action != GLFW_RELEASE)
	{
		// on press or hold
		if (Train->mvControlled->EngineType == TEngineType::DieselElectric && true == Train->mvControlled->ShuntMode)
		{
			Train->mvControlled->AnPos = 1.0;
		}
		else
		{
			Train->mvControlled->IncScndCtrl(2);
		}
	}
}

void TTrain::OnCommand_notchingrelaytoggle(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		if (false == Train->mvOccupied->AutoRelayFlag)
		{
			// turn on
			Train->mvOccupied->AutoRelaySwitch(true);
		}
		else
		{
			// turn off
			Train->mvOccupied->AutoRelaySwitch(false);
		}
		// visual feedback
		Train->ggAutoRelayButton.UpdateValue(Train->mvOccupied->AutoRelayFlag ? 1.f : 0.f, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_tempomattoggle(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_REPEAT)
	{
		return;
	}

	if (Train->ggScndCtrlButton.is_push())
	{
		// impulse switch
		if (Command.action == GLFW_RELEASE)
		{
			// just move the button(s) back to default position
			// visual feedback
			Train->ggScndCtrlButton.UpdateValue(0.0, Train->dsbSwitch);
			Train->ggScndCtrlOffButton.UpdateValue(0.0, Train->dsbSwitch);
			return;
		}
		// glfw_press
		if (false == Train->mvControlled->IsSpeedCtrlEngaged())
		{
			// turn on if it's not active
			Train->mvControlled->SpeedCtrlSwitch(true);
			// visual feedback
			Train->ggScndCtrlButton.UpdateValue(1.0, Train->dsbSwitch);
		}
		else
		{
			// otherwise turn off
			Train->mvControlled->SpeedCtrlSwitch(false);
			// visual feedback
			if (Train->m_controlmapper.contains("tempomatoff_sw:"))
			{
				Train->ggScndCtrlOffButton.UpdateValue(1.0, Train->dsbSwitch);
			}
			else
			{
				Train->ggScndCtrlButton.UpdateValue(1.0, Train->dsbSwitch);
			}
		}
	}
	else
	{
		// two-state switch
		if (Command.action == GLFW_RELEASE)
		{
			return;
		}

		if (false == Train->mvControlled->IsSpeedCtrlEngaged())
		{
			// turn on
			Train->mvControlled->SpeedCtrlSwitch(true);
			// visual feedback
			Train->ggScndCtrlButton.UpdateValue(1.0, Train->dsbSwitch);
		}
		else
		{
			// turn off
			Train->mvControlled->SpeedCtrlSwitch(false);
			// visual feedback
			Train->ggScndCtrlButton.UpdateValue(0.0, Train->dsbSwitch);
		}
	}
}

void TTrain::OnCommand_distancecounteractivate(TTrain *Train, command_data const &Command)
{
	// NOTE: distance meter activation button is presumed to be of impulse type
	if (Command.action == GLFW_PRESS)
	{
		// visual feedback
		Train->ggDistanceCounterButton.UpdateValue(1.0, Train->dsbSwitch);
		// activate or start anew
		if (Train->mvOccupied->isDoubleClickForMeasureNeeded)
		{
			// handler tempomatu dla podwojnego kliku
			if (Train->trainLenghtMeasureTimer >= 0.f) // jesli zdazylismy w czasie sekundy
				Train->m_distancecounter = 0.f; // rozpoczynamy pomiar
			else
				Train->trainLenghtMeasureTimer = Train->mvOccupied->DistanceCounterDoublePressPeriod; // odpalamy zegarek od nowa
		}
		else
		{
			// dla pojedynczego kliku
			Train->m_distancecounter = 0.f;
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// visual feedback
		Train->ggDistanceCounterButton.UpdateValue(0.0, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_mucurrentindicatorothersourceactivate(TTrain *Train, command_data const &Command)
{

	if (Train->ggNextCurrentButton.SubModel == nullptr)
	{
		if (Command.action == GLFW_PRESS)
		{
			WriteLog("Current Indicator Source switch is missing, or wasn't defined");
		}
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// turn on
		Train->ShowNextCurrent = true;
		// visual feedback
		Train->ggNextCurrentButton.UpdateValue(1.0, Train->dsbSwitch);
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// turn off
		Train->ShowNextCurrent = false;
		// visual feedback
		Train->ggNextCurrentButton.UpdateValue(0.0, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_secondcontrollerdecrease(TTrain *Train, command_data const &Command)
{

	if (Train->mvControlled->EngineType == TEngineType::DieselElectric && true == Train->mvControlled->ShuntMode)
	{
		if (Command.action != GLFW_RELEASE)
		{
			Train->mvControlled->AnPos = std::clamp(Train->mvControlled->AnPos - 0.025, 0.0, 1.0);
		}
	}
	else
	{
		// regular mode
		// push or pushtoggle control type
		if (Train->ggScndCtrl.is_push())
		{
			// basic push control can't decrease state, but pushtoggle can
			if (true == Train->ggScndCtrl.is_toggle() && Command.action == GLFW_PRESS)
			{
				// activate on press
				Train->mvControlled->DecScndCtrl(1);
			}
		}
		// toggle control type
		else
		{
			if (Command.action != GLFW_RELEASE)
			{
				Train->mvControlled->DecScndCtrl(1);
			}
		}
		// HACK: potentially animate push or pushtoggle control
		if (Train->ggScndCtrl.is_push())
		{
			auto const activeposition{Train->ggScndCtrl.is_toggle() ? 0.f : 1.f};
			auto const neutralposition{Train->ggScndCtrl.is_toggle() ? 0.5f : 0.f};
			Train->ggScndCtrl.UpdateValue(Command.action == GLFW_RELEASE ? neutralposition : activeposition, Train->dsbSwitch);
		}
		// potentially animate tempomat button, unless the cruise control has its own
		if (Train->ggScndCtrlButton.is_push() && Train->mvControlled->ScndCtrlPos <= 1 && false == Train->mvControlled->IsSpeedCtrlSeparate())
		{
			if (Train->m_controlmapper.contains("tempomatoff_sw:"))
			{
				Train->ggScndCtrlOffButton.UpdateValue(Command.action == GLFW_RELEASE ? 0.f : 1.f, Train->dsbSwitch);
			}
			else
			{
				Train->ggScndCtrlButton.UpdateValue(Command.action == GLFW_RELEASE ? 0.f : 1.f, Train->dsbSwitch);
			}
		}
	}
}

void TTrain::OnCommand_secondcontrollerdecreasefast(TTrain *Train, command_data const &Command)
{

	if (Command.action != GLFW_RELEASE)
	{
		// on press or hold
		if (Train->mvControlled->EngineType == TEngineType::DieselElectric && true == Train->mvControlled->ShuntMode)
		{
			Train->mvControlled->AnPos = 0.0;
		}
		else
		{
			Train->mvControlled->DecScndCtrl(2);
		}
	}
}

void TTrain::OnCommand_secondcontrollerset(TTrain *Train, command_data const &Command)
{

	auto const targetposition{std::min<int>(static_cast<int>(Command.param1), Train->mvControlled->ScndCtrlPosNo)};
	// HACK: potentially animate push or pushtoggle control
	if (Train->ggScndCtrl.is_push())
	{
		auto const activeposition{Train->ggScndCtrl.is_toggle() ?
		                              (targetposition < Train->mvControlled->ScndCtrlPos ? 0.f :
		                               targetposition > Train->mvControlled->ScndCtrlPos ? 1.f :
		                                                                                   Train->ggScndCtrl.GetDesiredValue()) : // leave the control in its current position if it hits the limit
		                              targetposition == 0 ? 0.f :
		                                                    1.f};
		auto const neutralposition{Train->ggScndCtrl.is_toggle() ? 0.5f : 0.f};
		Train->ggScndCtrl.UpdateValue(Command.action == GLFW_RELEASE ? neutralposition : activeposition, Train->dsbSwitch);
	}
	// update control value
	if (Command.action != GLFW_RELEASE)
	{
		// on press or hold
		while (targetposition < Train->mvControlled->GetVirtualScndPos() && true == Train->mvControlled->DecScndCtrl(1))
		{
			// all work is done in the header
		}
		while (targetposition > Train->mvControlled->GetVirtualScndPos() && true == Train->mvControlled->IncScndCtrl(1))
		{
			// all work is done in the header
		}
	}
}
