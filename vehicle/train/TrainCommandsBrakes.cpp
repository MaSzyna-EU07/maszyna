/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/train/Train.h"

#include "utilities/Globals.h"
#include "world/Event.h"
#include "simulation/simulationtime.h"
#include "utilities/Logs.h"
#include "model/Model3d.h"
#include "vehicle/Driver.h"
#include "vehicle/DynObj.h"
#include "rendering/renderer.h"
#include <future>
#include <cmath>
#include <algorithm>

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

void TTrain::OnCommand_independentbrakeincrease(TTrain *Train, command_data const &Command)
{

	if (Command.action != GLFW_RELEASE)
	{

		// when SplitEDPneumaticBrake is active the local brake key always operates
		// the pneumatic local brake directly, never piggy-backing on the joint controller
		auto const splitMode{Train->mvControlled->SplitEDPneumaticBrake};
		auto const useStepped{Train->ggJointCtrl.SubModel != nullptr && !splitMode};
		if (Train->mvOccupied->LocalBrake != TLocalBrake::ManualBrake)
		{
			if (false == splitMode && Train->ggJointCtrl.SubModel != nullptr && Train->mvOccupied->MainCtrlPos > 0)
			{
				OnCommand_mastercontrollerdecrease(Train, Command);
			}
			else
			{
				Train->mvOccupied->IncLocalBrakeLevel(useStepped ? 1 : Global.brake_speed * Command.time_delta * LocalBrakePosNo);
				if (useStepped)
				{
					Train->m_mastercontrollerinuse = true;
				}
			}
		}
	}
}

void TTrain::OnCommand_independentbrakeincreasefast(TTrain *Train, command_data const &Command)
{

	if (Command.action != GLFW_RELEASE)
	{

		auto const splitMode{Train->mvControlled->SplitEDPneumaticBrake};
		auto const useJointAnim{Train->ggJointCtrl.SubModel != nullptr && !splitMode};
		if (Train->mvOccupied->LocalBrake != TLocalBrake::ManualBrake)
		{
			if (false == splitMode && Train->ggJointCtrl.SubModel != nullptr && Train->mvOccupied->MainCtrlPos > 0)
			{
				OnCommand_mastercontrollerdecreasefast(Train, Command);
			}
			else
			{
				Train->mvOccupied->IncLocalBrakeLevel(LocalBrakePosNo);
				if (useJointAnim)
				{
					Train->m_mastercontrollerinuse = true;
				}
			}
		}
	}
}

void TTrain::OnCommand_independentbrakedecrease(TTrain *Train, command_data const &Command)
{

	if (Command.action != GLFW_RELEASE)
	{

		auto const splitMode{Train->mvControlled->SplitEDPneumaticBrake};
		auto const useStepped{Train->ggJointCtrl.SubModel != nullptr && !splitMode};
		if (Train->mvOccupied->LocalBrake != TLocalBrake::ManualBrake
		    // Ra 1014-06: AI potrafi zahamować pomocniczym mimo jego braku - odhamować jakoś trzeba
		    // TODO: sort AI out so it doesn't do things it doesn't have equipment for
		    || Train->mvOccupied->LocalBrakePosA > 0)
		{
			if (false == splitMode && Train->ggJointCtrl.SubModel != nullptr && Train->mvOccupied->LocalBrakePosA == 0.0)
			{
				OnCommand_mastercontrollerincrease(Train, Command);
			}
			else
			{
				Train->mvOccupied->DecLocalBrakeLevel(useStepped ? 1 : Global.brake_speed * Command.time_delta * LocalBrakePosNo);
				if (useStepped)
				{
					Train->m_mastercontrollerinuse = true;
				}
			}
		}
	}
}

void TTrain::OnCommand_independentbrakedecreasefast(TTrain *Train, command_data const &Command)
{

	if (Command.action != GLFW_RELEASE)
	{

		auto const splitMode{Train->mvControlled->SplitEDPneumaticBrake};
		auto const useJointAnim{Train->ggJointCtrl.SubModel != nullptr && !splitMode};
		if (Train->mvOccupied->LocalBrake != TLocalBrake::ManualBrake
		    // Ra 1014-06: AI potrafi zahamować pomocniczym mimo jego braku - odhamować jakoś trzeba
		    // TODO: sort AI out so it doesn't do things it doesn't have equipment for
		    || Train->mvOccupied->LocalBrakePosA > 0)
		{
			if (false == splitMode && Train->ggJointCtrl.SubModel != nullptr && Train->mvOccupied->LocalBrakePosA == 0.0)
			{
				OnCommand_mastercontrollerincreasefast(Train, Command);
			}
			else
			{
				Train->mvOccupied->DecLocalBrakeLevel(LocalBrakePosNo);
				if (useJointAnim)
				{
					Train->m_mastercontrollerinuse = true;
				}
			}
		}
	}
}

void TTrain::OnCommand_independentbrakeset(TTrain *Train, command_data const &Command)
{

	if (Command.action != GLFW_RELEASE)
	{

		Train->mvOccupied->LocalBrakePosA = std::clamp(Command.param1, 0.0, 1.0);
	}
	/*
	    Train->mvControlled->LocalBrakePos = (
	        std::round(
	            interpolate<double>(
	                0.0,
	                LocalBrakePosNo,
	                clamp(
	                    Command.param1,
	                    0.0, 1.0 ) ) ) );
	*/
}

void TTrain::OnCommand_independentbrakebailoff(TTrain *Train, command_data const &Command)
{

	if (false == Command.freefly)
	{
		// TODO: check if this set of conditions can be simplified.
		// it'd be more flexible to have an attribute indicating whether bail off position is supported
		if (Train->mvControlled->TrainType != dt_EZT &&
		    (Train->mvControlled->EngineType == TEngineType::ElectricSeriesMotor || Train->mvControlled->EngineType == TEngineType::DieselElectric ||
		     Train->mvControlled->EngineType == TEngineType::ElectricInductionMotor) &&
		    Train->mvOccupied->BrakeCtrlPosNo > 0)
		{

			if (Command.action == GLFW_PRESS)
			{
				// press or hold
				// visual feedback
				Train->ggReleaserButton.UpdateValue(1.0, Train->dsbSwitch);

				Train->mvOccupied->BrakeReleaser(1);
			}
			else if (Command.action == GLFW_RELEASE)
			{
				// release
				// visual feedback
				Train->ggReleaserButton.UpdateValue(0.0, Train->dsbSwitch);

				Train->mvOccupied->BrakeReleaser(0);
			}
		}
	}
	else
	{
		// car brake handling, while in walk mode
		auto *vehicle{Train->find_nearest_consist_vehicle(Command.freefly, Command.location)};
		if (vehicle != nullptr)
		{
			if (Command.action == GLFW_PRESS)
			{
				// press or hold
				vehicle->MoverParameters->BrakeReleaser(1);
			}
			else if (Command.action == GLFW_RELEASE)
			{
				// release
				vehicle->MoverParameters->BrakeReleaser(0);
			}
		}
	}
}

void TTrain::OnCommand_universalbrakebutton1(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// press or hold
		// visual feedback
		Train->ggUniveralBrakeButton1.UpdateValue(1.0, Train->dsbSwitch);

		Train->mvOccupied->UniversalBrakeButton(0, 1);
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// release
		// visual feedback
		Train->ggUniveralBrakeButton1.UpdateValue(0.0, Train->dsbSwitch);

		Train->mvOccupied->UniversalBrakeButton(0, 0);
	}
}

void TTrain::OnCommand_universalbrakebutton2(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// press or hold
		// visual feedback
		Train->ggUniveralBrakeButton2.UpdateValue(1.0, Train->dsbSwitch);

		Train->mvOccupied->UniversalBrakeButton(1, 1);
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// release
		// visual feedback
		Train->ggUniveralBrakeButton2.UpdateValue(0.0, Train->dsbSwitch);

		Train->mvOccupied->UniversalBrakeButton(1, 0);
	}
}

void TTrain::OnCommand_universalbrakebutton3(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// press or hold
		// visual feedback
		Train->ggUniveralBrakeButton3.UpdateValue(1.0, Train->dsbSwitch);

		Train->mvOccupied->UniversalBrakeButton(2, 1);
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// release
		// visual feedback
		Train->ggUniveralBrakeButton3.UpdateValue(0.0, Train->dsbSwitch);

		Train->mvOccupied->UniversalBrakeButton(2, 0);
	}
}

void TTrain::OnCommand_trainbrakeincrease(TTrain *Train, command_data const &Command)
{
	if (Command.action == GLFW_REPEAT && Train->mvOccupied->BrakeHandle == TBrakeHandle::FV4a)
		Train->mvOccupied->BrakeLevelAdd(Global.brake_speed * Command.time_delta * Train->mvOccupied->BrakeCtrlPosNo);
	else if (Command.action == GLFW_PRESS && Train->mvOccupied->BrakeHandle != TBrakeHandle::FV4a)
		Train->set_train_brake(Train->mvOccupied->fBrakeCtrlPos + Global.fBrakeStep);
}

void TTrain::OnCommand_trainbrakedecrease(TTrain *Train, command_data const &Command)
{
	if (Command.action == GLFW_REPEAT && Train->mvOccupied->BrakeHandle == TBrakeHandle::FV4a)
		Train->mvOccupied->BrakeLevelAdd(-Global.brake_speed * Command.time_delta * Train->mvOccupied->BrakeCtrlPosNo);
	else if (Command.action == GLFW_PRESS && Train->mvOccupied->BrakeHandle != TBrakeHandle::FV4a)
		Train->set_train_brake(Train->mvOccupied->fBrakeCtrlPos - Global.fBrakeStep);
	else if (Command.action == GLFW_RELEASE)
	{
		// release
		Train->zero_charging_train_brake();
	}
}

void TTrain::OnCommand_trainbrakeset(TTrain *Train, command_data const &Command)
{

	if (Command.action != GLFW_RELEASE)
	{
		// press or hold
		Train->mvOccupied->BrakeLevelSet(std::lerp(Train->mvOccupied->Handle->GetPos(bh_MIN), Train->mvOccupied->Handle->GetPos(bh_MAX), std::clamp(Command.param1, 0.0, 1.0)));
	}
	else
	{
		// release
		Train->zero_charging_train_brake();
	}
}

void TTrain::OnCommand_trainbrakecharging(TTrain *Train, command_data const &Command)
{

	if (Command.action != GLFW_RELEASE)
	{
		// press or hold
		Train->set_train_brake(-1);
	}
	else
	{
		// release
		Train->zero_charging_train_brake();
	}
}

void TTrain::OnCommand_trainbrakerelease(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{

		Train->set_train_brake(0);
	}
}

void TTrain::OnCommand_trainbrakefirstservice(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{

		Train->set_train_brake(1);
	}
}

void TTrain::OnCommand_trainbrakeservice(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{

		Train->set_train_brake(Train->mvOccupied->BrakeCtrlPosNo / 2 + (Train->mvOccupied->BrakeHandle == TBrakeHandle::FV4a ? 1 : 0));
	}
}

void TTrain::OnCommand_trainbrakefullservice(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{

		Train->set_train_brake(Train->mvOccupied->BrakeCtrlPosNo - 1);
	}
}

void TTrain::OnCommand_trainbrakehandleoff(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{

		Train->set_train_brake(Train->mvOccupied->Handle->GetPos(bh_NP));
	}
}

void TTrain::OnCommand_trainbrakeemergency(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{

		Train->set_train_brake(Train->mvOccupied->Handle->GetPos(bh_EB));
		/*
		        if( Train->mvOccupied->BrakeCtrlPosNo <= 0.1 ) {
		            hamulec bezpieczeństwa dla wagonów
		            Train->mvOccupied->RadioStopFlag = true;
		        }
		*/
	}
}

void TTrain::OnCommand_trainbrakebasepressureincrease(TTrain *Train, command_data const &Command)
{

	if (Command.action != GLFW_RELEASE)
	{

		switch (Train->mvOccupied->BrakeHandle)
		{
		case TBrakeHandle::FV4a:
		{
			Train->mvOccupied->BrakeCtrlPos2 = std::clamp(Train->mvOccupied->BrakeCtrlPos2 - 0.01, -1.5, 2.0);
			break;
		}
		default:
		{
			Train->mvOccupied->BrakeLevelAdd(0.01);
			break;
		}
		}
	}
}

void TTrain::OnCommand_trainbrakebasepressuredecrease(TTrain *Train, command_data const &Command)
{

	if (Command.action != GLFW_RELEASE)
	{

		switch (Train->mvOccupied->BrakeHandle)
		{
		case TBrakeHandle::FV4a:
		{
			Train->mvOccupied->BrakeCtrlPos2 = std::clamp(Train->mvOccupied->BrakeCtrlPos2 + 0.01, -1.5, 2.0);
			break;
		}
		default:
		{
			Train->mvOccupied->BrakeLevelAdd(-0.01);
			break;
		}
		}
	}
}

void TTrain::OnCommand_trainbrakebasepressurereset(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{

		Train->mvOccupied->BrakeCtrlPos2 = 0;
	}
}

void TTrain::OnCommand_trainbrakeoperationtoggle(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{

		auto *vehicle{Train->find_nearest_consist_vehicle(Command.freefly, Command.location)};
		if (vehicle == nullptr)
		{
			return;
		}

		vehicle->MoverParameters->Hamulec->SetBrakeStatus(vehicle->MoverParameters->Hamulec->GetBrakeStatus() ^ b_dmg);
	}
}

void TTrain::OnCommand_manualbrakeincrease(TTrain *Train, command_data const &Command)
{

	if (Command.action != GLFW_RELEASE)
	{

		auto *vehicle{Train->find_nearest_consist_vehicle(Command.freefly, Command.location)};
		if (vehicle == nullptr)
		{
			return;
		}

		if (vehicle->MoverParameters->LocalBrake == TLocalBrake::ManualBrake || vehicle->MoverParameters->MBrake == true)
		{

			vehicle->MoverParameters->IncManualBrakeLevel(1);
		}
	}
}

void TTrain::OnCommand_manualbrakedecrease(TTrain *Train, command_data const &Command)
{

	if (Command.action != GLFW_RELEASE)
	{

		auto *vehicle{Train->find_nearest_consist_vehicle(Command.freefly, Command.location)};
		if (vehicle == nullptr)
		{
			return;
		}

		if (vehicle->MoverParameters->LocalBrake == TLocalBrake::ManualBrake || vehicle->MoverParameters->MBrake == true)
		{

			vehicle->MoverParameters->DecManualBrakeLevel(1);
		}
	}
}

void TTrain::OnCommand_alarmchaintoggle(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{

		if (false == Train->mvOccupied->AlarmChainFlag)
		{
			OnCommand_alarmchainenable(Train, Command);
		}
		else
		{
			OnCommand_alarmchaindisable(Train, Command);
		}
	}
}

void TTrain::OnCommand_alarmchainenable(TTrain *Train, command_data const &Command)
{
	if (Command.action == GLFW_PRESS)
	{

		// pull
		Train->mvOccupied->AlarmChainSwitch(true);
		// visual feedback
		Train->ggAlarmChain.UpdateValue(1.0);
	}
}

void TTrain::OnCommand_alarmchaindisable(TTrain *Train, command_data const &Command)
{
	if (Command.action == GLFW_PRESS)
	{

		// release
		Train->mvOccupied->AlarmChainSwitch(false);
		// visual feedback
		Train->ggAlarmChain.UpdateValue(0.0);
	}
}

void TTrain::OnCommand_wheelspinbrakeactivate(TTrain *Train, command_data const &Command)
{

	// TODO: proper control deviced definition for the interiors, that doesn't hinge of presence of 3d submodels
	if (Train->ggAntiSlipButton.SubModel == nullptr)
	{
		if (Command.action == GLFW_PRESS)
		{
			WriteLog("Wheelspin Brake button is missing, or wasn't defined");
		}
		return;
	}

	if (Train->mvOccupied->BrakeSystem != TBrakeSystem::ElectroPneumatic)
	{
		// standard behaviour
		if (Command.action == GLFW_PRESS)
		{
			// visual feedback
			Train->ggAntiSlipButton.UpdateValue(1.0, Train->dsbSwitch);

			// NOTE: system activation is (repeatedly) done in the train update routine
		}
		else if (Command.action == GLFW_RELEASE)
		{
			// visual feedback
			Train->ggAntiSlipButton.UpdateValue(0.0);
		}
	}
	else
	{
		// electro-pneumatic, custom case
		if (Command.action == GLFW_PRESS)
		{
			// visual feedback
			Train->ggAntiSlipButton.UpdateValue(1.0, Train->dsbPneumaticSwitch);

			if (Train->mvOccupied->BrakeHandle == TBrakeHandle::St113 && Train->mvControlled->EpFuse == true)
			{
				Train->mvOccupied->SwitchEPBrake(1);
			}
		}
		else if (Command.action == GLFW_RELEASE)
		{
			// visual feedback
			Train->ggAntiSlipButton.UpdateValue(0.0);

			Train->mvOccupied->SwitchEPBrake(0);
		}
	}
}

void TTrain::OnCommand_sandboxactivate(TTrain *Train, command_data const &Command)
{

	// TODO: proper control deviced definition for the interiors, that doesn't hinge of presence of 3d submodels
	if (Train->ggSandButton.SubModel == nullptr)
	{
		if (Command.action == GLFW_PRESS)
		{
			WriteLog("Sandbox activation button is missing, or wasn't defined");
		}
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// visual feedback
		Train->ggSandButton.UpdateValue(1.0, Train->dsbSwitch);

		Train->mvControlled->SandboxManual(true);
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// visual feedback
		Train->ggSandButton.UpdateValue(0.0);

		Train->mvControlled->SandboxManual(false);
	}
}

void TTrain::OnCommand_autosandboxtoggle(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		if (false == Train->mvOccupied->SandDoseAutoAllow)
		{
			// turn on
			OnCommand_autosandboxactivate(Train, Command);
		}
		else
		{
			// turn off
			OnCommand_autosandboxdeactivate(Train, Command);
		}
	}
};

void TTrain::OnCommand_autosandboxactivate(TTrain *Train, command_data const &Command)
{
	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		Train->mvOccupied->SandboxAutoAllow(true);
		Train->ggAutoSandButton.UpdateValue(1.0, Train->dsbSwitch);
	}
};

void TTrain::OnCommand_autosandboxdeactivate(TTrain *Train, command_data const &Command)
{
	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		Train->mvOccupied->SandboxAutoAllow(false);
		Train->ggAutoSandButton.UpdateValue(0.0, Train->dsbSwitch);
	}
};

void TTrain::OnCommand_epbrakecontrolenable(TTrain *Train, command_data const &Command)
{
	auto const istoggle{(static_cast<int>(Train->ggEPFuseButton.type()) & static_cast<int>(TGaugeType::toggle)) != 0};
	if (Command.action == GLFW_PRESS && istoggle && Train->mvOccupied->EpFuseSwitch(true))
	{
		// command only works for bistable switch
		// audio feedback
		if (Train->dsbPneumaticSwitch)
		{
			Train->dsbPneumaticSwitch->play();
		}
		Train->ggEPFuseButton.UpdateValue(1.0f, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_epbrakecontroldisable(TTrain *Train, command_data const &Command)
{
	auto const istoggle{(static_cast<int>(Train->ggEPFuseButton.type()) & static_cast<int>(TGaugeType::toggle)) != 0};
	if (Command.action == GLFW_PRESS && istoggle && Train->mvOccupied->EpFuseSwitch(false))
	{
		// command only works for bistable switch
		Train->ggEPFuseButton.UpdateValue(0.0f, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_epbrakecontroltoggle(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_REPEAT)
	{
		return;
	}

	auto const ispush{(static_cast<int>(Train->ggEPFuseButton.type()) & static_cast<int>(TGaugeType::push)) != 0};
	auto const istoggle{(static_cast<int>(Train->ggEPFuseButton.type()) & static_cast<int>(TGaugeType::toggle)) != 0};

	if (Command.action == GLFW_PRESS)
	{
		if (istoggle)
		{
			// switch state
			if (false == Train->mvOccupied->EpFuse)
			{
				// turn on
				if (Train->mvOccupied->EpFuseSwitch(true) && Train->dsbPneumaticSwitch)
				{
					// audio feedback
					Train->dsbPneumaticSwitch->play();
				}
			}
			else
			{
				// turn off
				Train->mvOccupied->EpFuseSwitch(false);
			}
		}
		else if (ispush)
		{
			// potentially turn on
			if (Train->mvOccupied->EpFuseSwitch(true) && Train->dsbPneumaticSwitch)
			{
				// audio feedback
				Train->dsbPneumaticSwitch->play();
			}
		}
		// visual feedback
		Train->ggEPFuseButton.UpdateValue(ispush ? 1.0f : // push or pushtoggle
		                                      Train->mvOccupied->EpFuse ? 1.0f :
		                                                                  0.0f, // toggle
		                                  Train->dsbSwitch);
	}
	else if (Command.action == GLFW_RELEASE && ispush)
	{
		// return the switch to neutral position
		Train->ggEPFuseButton.UpdateValue(0.0f, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_trainbrakeoperationmodeincrease(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS && (Train->mvOccupied->BrakeOpModeFlag << 1 & Train->mvOccupied->BrakeOpModes) != 0)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		// next mode
		Train->mvOccupied->BrakeOpModeFlag <<= 1;
		// visual feedback
		Train->ggBrakeOperationModeCtrl.UpdateValue(Train->mvOccupied->BrakeOpModeFlag > 0 ? std::log2(Train->mvOccupied->BrakeOpModeFlag) : 0); // audio fallback
	}
}

void TTrain::OnCommand_trainbrakeoperationmodedecrease(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS && (Train->mvOccupied->BrakeOpModeFlag >> 1 & Train->mvOccupied->BrakeOpModes) != 0)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		// previous mode
		Train->mvOccupied->BrakeOpModeFlag >>= 1;
		// visual feedback
		Train->ggBrakeOperationModeCtrl.UpdateValue(Train->mvOccupied->BrakeOpModeFlag > 0 ? std::log2(Train->mvOccupied->BrakeOpModeFlag) : 0);
	}
}

void TTrain::OnCommand_brakeactingspeedincrease(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{

		auto *vehicle{Train->find_nearest_consist_vehicle(Command.freefly, Command.location)};
		if (vehicle == nullptr)
		{
			return;
		}

		if ((vehicle->MoverParameters->BrakeDelayFlag & bdelay_M) != 0)
		{
			// can't speed it up any more than this
			return;
		}
		auto const fasterbrakesetting = vehicle->MoverParameters->BrakeDelayFlag < bdelay_R ? vehicle->MoverParameters->BrakeDelayFlag << 1 : vehicle->MoverParameters->BrakeDelayFlag | bdelay_M;

		Train->set_train_brake_speed(vehicle, fasterbrakesetting);
	}
}

void TTrain::OnCommand_brakeactingspeeddecrease(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{

		auto *vehicle{Train->find_nearest_consist_vehicle(Command.freefly, Command.location)};
		if (vehicle == nullptr)
		{
			return;
		}

		if (vehicle->MoverParameters->BrakeDelayFlag == bdelay_G)
		{
			// can't slow it down any more than this
			return;
		}
		auto const slowerbrakesetting = vehicle->MoverParameters->BrakeDelayFlag < bdelay_M ? vehicle->MoverParameters->BrakeDelayFlag >> 1 : vehicle->MoverParameters->BrakeDelayFlag ^ bdelay_M;

		Train->set_train_brake_speed(vehicle, slowerbrakesetting);
	}
}

void TTrain::OnCommand_brakeactingspeedsetcargo(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{

		auto *vehicle{Train->find_nearest_consist_vehicle(Command.freefly, Command.location)};
		if (vehicle == nullptr)
		{
			return;
		}

		Train->set_train_brake_speed(vehicle, bdelay_G);
	}
}

void TTrain::OnCommand_brakeactingspeedsetpassenger(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{

		auto *vehicle{Train->find_nearest_consist_vehicle(Command.freefly, Command.location)};
		if (vehicle == nullptr)
		{
			return;
		}

		Train->set_train_brake_speed(vehicle, bdelay_P);
	}
}

void TTrain::OnCommand_brakeactingspeedsetrapid(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{

		auto *vehicle{Train->find_nearest_consist_vehicle(Command.freefly, Command.location)};
		if (vehicle == nullptr)
		{
			return;
		}

		Train->set_train_brake_speed(vehicle, bdelay_R);
	}
}

void TTrain::OnCommand_brakeloadcompensationincrease(TTrain *Train, command_data const &Command)
{

	if (true == Command.freefly && Command.action == GLFW_PRESS)
	{
		auto *vehicle{Train->find_nearest_consist_vehicle(Command.freefly, Command.location)};
		if (vehicle != nullptr)
		{
			vehicle->MoverParameters->IncBrakeMult();
		}
	}
}

void TTrain::OnCommand_brakeloadcompensationdecrease(TTrain *Train, command_data const &Command)
{

	if (true == Command.freefly && Command.action == GLFW_PRESS)
	{
		auto *vehicle{Train->find_nearest_consist_vehicle(Command.freefly, Command.location)};
		if (vehicle != nullptr)
		{
			vehicle->MoverParameters->DecBrakeMult();
		}
	}
}

void TTrain::OnCommand_mubrakingindicatortoggle(TTrain *Train, command_data const &Command)
{

	if (Train->ggSignallingButton.SubModel == nullptr)
	{
		if (Command.action == GLFW_PRESS)
		{
			WriteLog("Braking Indicator switch is missing, or wasn't defined");
		}
		return;
	}
	if (Train->mvControlled->TrainType != dt_EZT)
	{
		//
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		if (false == Train->mvControlled->Signalling)
		{
			// turn on
			Train->mvControlled->Signalling = true;
			// visual feedback
			Train->ggSignallingButton.UpdateValue(1.0, Train->dsbSwitch);
		}
		else
		{
			// turn off
			Train->mvControlled->Signalling = false;
			// visual feedback
			Train->ggSignallingButton.UpdateValue(0.0, Train->dsbSwitch);
		}
	}
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

		auto *vehicle{Train->find_nearest_consist_vehicle(Command.freefly, Command.location)};
		if (vehicle == nullptr)
		{
			return;
		}
		Train->mvOccupied->SpringBrakeRelease();
	}
};
