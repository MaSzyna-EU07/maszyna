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

#include "simulation/simulation.h"
#include "world/Event.h"
#include "simulation/simulationtime.h"
#include "utilities/Logs.h"
#include "model/Model3d.h"
#include "vehicle/Driver.h"
#include "vehicle/DynObj.h"
#include "rendering/renderer.h"
#include <future>
#include <algorithm>

void TTrain::OnCommand_hornlowactivate(TTrain *Train, command_data const &Command)
{

	if (Train->ggHornButton.SubModel == nullptr && Train->ggHornLowButton.SubModel == nullptr)
	{
		if (Command.action == GLFW_PRESS)
		{
			WriteLog("Horn button is missing, or wasn't defined");
		}
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// only need to react to press, sound will continue until stopped
		if (false == TestFlag(Train->mvOccupied->WarningSignal, 1))
		{
			// turn on
			Train->mvOccupied->WarningSignal |= 1;
			/*
			            if( true == TestFlag( Train->mvOccupied->WarningSignal, 2 ) ) {
			                low and high horn are treated as mutually exclusive
			                Train->mvControlled->WarningSignal &= ~2;
			            }
			*/
			// visual feedback
			Train->ggHornButton.UpdateValue(-1.0);
			Train->ggHornLowButton.UpdateValue(1.0);
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// turn off
		/*
		        NOTE: we turn off both low and high horn, due to unreliability of release event when shift key is involved
		        Train->mvOccupied->WarningSignal &= ~( 1 | 2 );
		*/
		Train->mvOccupied->WarningSignal &= ~1;
		// visual feedback
		Train->ggHornButton.UpdateValue(0.0);
		Train->ggHornLowButton.UpdateValue(0.0);
	}
}

void TTrain::OnCommand_hornhighactivate(TTrain *Train, command_data const &Command)
{

	if (Train->ggHornButton.SubModel == nullptr && Train->ggHornHighButton.SubModel == nullptr)
	{
		if (Command.action == GLFW_PRESS)
		{
			WriteLog("Horn button is missing, or wasn't defined");
		}
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// only need to react to press, sound will continue until stopped
		if (false == TestFlag(Train->mvOccupied->WarningSignal, 2))
		{
			// turn on
			Train->mvOccupied->WarningSignal |= 2;
			/*
			            if( true == TestFlag( Train->mvOccupied->WarningSignal, 1 ) ) {
			                low and high horn are treated as mutually exclusive
			                Train->mvControlled->WarningSignal &= ~1;
			            }
			*/
			// visual feedback
			Train->ggHornButton.UpdateValue(1.0);
			Train->ggHornHighButton.UpdateValue(1.0);
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// turn off
		/*
		        NOTE: we turn off both low and high horn, due to unreliability of release event when shift key is involved
		        Train->mvOccupied->WarningSignal &= ~( 1 | 2 );
		*/
		Train->mvOccupied->WarningSignal &= ~2;
		// visual feedback
		Train->ggHornButton.UpdateValue(0.0);
		Train->ggHornHighButton.UpdateValue(0.0);
	}
}

void TTrain::OnCommand_whistleactivate(TTrain *Train, command_data const &Command)
{

	if (Train->ggWhistleButton.SubModel == nullptr)
	{
		if (Command.action == GLFW_PRESS)
		{
			WriteLog("Whistle button is missing, or wasn't defined");
		}
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// only need to react to press, sound will continue until stopped
		if (false == TestFlag(Train->mvOccupied->WarningSignal, 4))
		{
			// turn on
			Train->mvOccupied->WarningSignal |= 4;
			// visual feedback
			Train->ggWhistleButton.UpdateValue(1.0);
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// turn off
		Train->mvOccupied->WarningSignal &= ~4;
		// visual feedback
		Train->ggWhistleButton.UpdateValue(0.0);
	}
}

void TTrain::OnCommand_radiotoggle(TTrain *Train, command_data const &Command)
{
	if (Command.action != GLFW_PRESS)
	{
		return;
	}

	// NOTE: we ignore the lack of 3d model to allow system reset after receiving radio-stop signal
	/*
	    if( false == Train->m_controlmapper.contains( "radio_sw:" ) ) {
	        return;
	    }
	*/
	// only reacting to press, so the sound can loop uninterrupted
	if (false == Train->mvOccupied->Radio)
	{
		// turn on
		OnCommand_radioenable(Train, Command);
	}
	else
	{
		// turn off
		OnCommand_radiodisable(Train, Command);
	}
}

void TTrain::OnCommand_radioenable(TTrain *Train, command_data const &Command)
{
	if (Command.action != GLFW_PRESS)
	{
		return;
	}

	if (false == Train->mvOccupied->Radio)
	{
		Train->mvOccupied->Radio = true;
	}
}

void TTrain::OnCommand_radiodisable(TTrain *Train, command_data const &Command)
{
	if (Command.action != GLFW_PRESS)
	{
		return;
	}

	if (Train->mvOccupied->Radio)
	{
		Train->mvOccupied->Radio = false;
	}
}

void TTrain::OnCommand_radiochannelincrease(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		command_data newCommand = Command;
		newCommand.param1 = Train->RadioChannel() + 1;
		OnCommand_radiochannelset(Train, newCommand);

		Train->ggRadioChannelNext.UpdateValue(1.0);
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// visual feedback
		Train->ggRadioChannelNext.UpdateValue(0.0);
	}
}

void TTrain::OnCommand_radiochanneldecrease(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		command_data newCommand = Command;
		newCommand.param1 = Train->RadioChannel() - 1;
		OnCommand_radiochannelset(Train, newCommand);

		Train->ggRadioChannelPrevious.UpdateValue(1.0);
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// visual feedback
		Train->ggRadioChannelPrevious.UpdateValue(0.0);
	}
}

void TTrain::OnCommand_radiochannelset(TTrain *Train, command_data const &Command)
{
	if (Command.action != GLFW_RELEASE)
	{
		// on press or hold
		Train->RadioChannel() = std::clamp((int)Command.param1, 1, 10);
		Train->ggRadioChannelSelector.UpdateValue(Train->RadioChannel() - 1);
	}
}

void TTrain::OnCommand_radiostopsend(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		if (true == Train->mvOccupied->Radio && (Train->mvOccupied->Power24vIsAvailable || Train->mvOccupied->Power110vIsAvailable))
		{
			simulation::Region->RadioStop(Train->Dynamic()->GetPosition());
		}
		// visual feedback
		Train->ggRadioStop.UpdateValue(1.0);
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// visual feedback
		Train->ggRadioStop.UpdateValue(0.0);
	}
}

void TTrain::OnCommand_radiostopenable(TTrain *Train, command_data const &Command)
{
	if (Command.action == GLFW_PRESS && Train->ggRadioStop.GetValue() == 0)
	{
		if (true == Train->mvOccupied->Radio && (Train->mvOccupied->Power24vIsAvailable || Train->mvOccupied->Power110vIsAvailable))
		{
			simulation::Region->RadioStop(Train->Dynamic()->GetPosition());
		}
		// visual feedback
		Train->ggRadioStop.UpdateValue(1.0);
	}
}

void TTrain::OnCommand_radiostopdisable(TTrain *Train, command_data const &Command)
{
	if (Command.action == GLFW_PRESS && Train->ggRadioStop.GetValue() > 0)
	{
		// visual feedback
		Train->ggRadioStop.UpdateValue(0.0);
	}
}

void TTrain::OnCommand_radiostoptest(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		if (Train->RadioChannel() == 10 && true == Train->mvOccupied->Radio && (Train->mvOccupied->Power24vIsAvailable || Train->mvOccupied->Power110vIsAvailable))
		{
			Train->Dynamic()->RadioStop();
		}
		// visual feedback
		Train->ggRadioTest.UpdateValue(1.0);
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// visual feedback
		Train->ggRadioTest.UpdateValue(0.0);
	}
}

void TTrain::OnCommand_radiocall1send(TTrain *Train, command_data const &Command)
{
	if (Command.action == GLFW_PRESS)
	{
		if (Train->RadioChannel() != 10 && true == Train->mvOccupied->Radio && (Train->mvOccupied->Power24vIsAvailable || Train->mvOccupied->Power110vIsAvailable))
		{
			simulation::Events.queue_receivers(radio_message::call1, Train->Dynamic()->GetPosition());
		}
		// visual feedback
		Train->ggRadioCall1.UpdateValue(1.0);
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// visual feedback
		Train->ggRadioCall1.UpdateValue(0.0);
	}
}

void TTrain::OnCommand_radiocall3send(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		if (Train->RadioChannel() != 10 && true == Train->mvOccupied->Radio && (Train->mvOccupied->Power24vIsAvailable || Train->mvOccupied->Power110vIsAvailable))
		{
			simulation::Events.queue_receivers(radio_message::call3, Train->Dynamic()->GetPosition());
		}
		// visual feedback
		Train->ggRadioCall3.UpdateValue(1.0);
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// visual feedback
		Train->ggRadioCall3.UpdateValue(0.0);
	}
}

void TTrain::OnCommand_radiovolumeincrease(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		command_data newCommand = Command;
		newCommand.param1 = Train->m_radiovolume + 0.125;
		OnCommand_radiovolumeset(Train, newCommand);
		Train->ggRadioVolumeNext.UpdateValue(1.0);
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// visual feedback
		Train->ggRadioVolumeNext.UpdateValue(0.0);
	}
}

void TTrain::OnCommand_radiovolumedecrease(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		command_data newCommand = Command;
		newCommand.param1 = Train->m_radiovolume - 0.125;
		OnCommand_radiovolumeset(Train, newCommand);
		Train->ggRadioVolumePrevious.UpdateValue(1.0);
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// visual feedback
		Train->ggRadioVolumePrevious.UpdateValue(0.0);
	}
}

void TTrain::OnCommand_radiovolumeset(TTrain *Train, command_data const &Command)
{
	if (Command.action != GLFW_RELEASE)
	{
		// on press or hold
		Train->m_radiovolume = std::clamp(Command.param1, 0.0, 1.0);
		Train->ggRadioVolumeSelector.UpdateValue(Train->m_radiovolume);
		audio::event_volume_change = true;
	}
}
