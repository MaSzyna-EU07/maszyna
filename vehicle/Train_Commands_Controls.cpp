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
#include "utilities/Logs.h"
#include "model/Model3d.h"
#include "vehicle/Driver.h"
#include "vehicle/DynObj.h"
#include <future>
#include <algorithm>

auto const EU07_CONTROLLER_BASERETURNDELAY{0.5f};
auto const EU07_CONTROLLER_KEYBOARDETURNDELAY{1.5f};

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

	if (Command.action == GLFW_PRESS && Train->DynamicObject->Mechanik)
	{
		// on press
		Train->DynamicObject->Mechanik->TakeControl(false);
	}
}

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
			auto const activeposition{Train->ggScndCtrl.is_toggle() ? 1.f : 1.f};
			auto const neutralposition{Train->ggScndCtrl.is_toggle() ? 0.5f : 0.f};
			Train->ggScndCtrl.UpdateValue(Command.action == GLFW_RELEASE ? neutralposition : activeposition, Train->dsbSwitch);
		}
		// potentially animate tempomat button
		if (Train->ggScndCtrlButton.is_push() && Train->mvControlled->ScndCtrlPos <= 1)
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
		if (Train->mvControlled->ScndCtrlPos == 0)
		{
			// turn on if it's not active
			Train->mvControlled->IncScndCtrl(1);
			// visual feedback
			Train->ggScndCtrlButton.UpdateValue(1.0, Train->dsbSwitch);
		}
		else
		{
			// otherwise turn off
			Train->mvControlled->DecScndCtrl(2);
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

		if (Train->mvControlled->ScndCtrlPos == 0)
		{
			// turn on
			Train->mvControlled->IncScndCtrl(1);
			// visual feedback
			Train->ggScndCtrlButton.UpdateValue(1.0, Train->dsbSwitch);
		}
		else
		{
			// turn off
			Train->mvControlled->DecScndCtrl(2);
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
		// potentially animate tempomat button
		if (Train->ggScndCtrlButton.is_push() && Train->mvControlled->ScndCtrlPos <= 1)
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

	auto const targetposition{std::min<int>(Command.param1, Train->mvControlled->ScndCtrlPosNo)};
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

		if (Train->mvOccupied->DirectionForward() && Train->mvOccupied->DirActive && Train->DynamicObject->Mechanik)
		{
			// aktualizacja skrajnych pojazdów w składzie
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

		if (Train->mvOccupied->DirectionBackward() && Train->mvOccupied->DirActive && Train->DynamicObject->Mechanik)
		{
			// aktualizacja skrajnych pojazdów w składzie
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
	auto &item = Train->ggSpeedCtrlButtons[itemindex];

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
