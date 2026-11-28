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

// the commands of the horns and the radios

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
		if (false == TestFlag(Train->mvOccupied->WarningSignal, horn::low))
		{
			// turn on
			Train->mvOccupied->WarningSignal = horn::pressed(Train->mvOccupied->WarningSignal, horn::low, Train->has_single_horn_lever());
			// visual feedback
			Train->ggHornButton.UpdateValue(-1.0);
			Train->ggHornLowButton.UpdateValue(1.0);
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// turn off
		Train->mvOccupied->WarningSignal = horn::released(Train->mvOccupied->WarningSignal, horn::low);
		// visual feedback
		if (false == (Train->has_single_horn_lever() && TestFlag(Train->mvOccupied->WarningSignal, horn::high)))
		{
			// leave the single lever alone if it was moved to the other tone in the meantime
			Train->ggHornButton.UpdateValue(0.0);
		}
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
		if (false == TestFlag(Train->mvOccupied->WarningSignal, horn::high))
		{
			// turn on
			Train->mvOccupied->WarningSignal = horn::pressed(Train->mvOccupied->WarningSignal, horn::high, Train->has_single_horn_lever());
			// visual feedback
			Train->ggHornButton.UpdateValue(1.0);
			Train->ggHornHighButton.UpdateValue(1.0);
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// turn off
		Train->mvOccupied->WarningSignal = horn::released(Train->mvOccupied->WarningSignal, horn::high);
		// visual feedback
		if (false == (Train->has_single_horn_lever() && TestFlag(Train->mvOccupied->WarningSignal, horn::low)))
		{
			// leave the single lever alone if it was moved to the other tone in the meantime
			Train->ggHornButton.UpdateValue(0.0);
		}
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

	// NOTE: works without the switch in the 3d model too, to allow system reset after receiving radio-stop signal
	// only reacting to press, so the sound can loop uninterrupted
	// NOTE: flips the state directly rather than through radioenable/radiodisable, so it doesn't move the radioon_sw:/radiooff_sw: buttons
	Train->mvOccupied->Radio = (false == Train->mvOccupied->Radio);
}

void TTrain::OnCommand_radioenable(TTrain *Train, command_data const &Command)
{
	if (Command.action == GLFW_PRESS)
	{
		Train->mvOccupied->Radio = true;
		// visual feedback
		if (Train->m_controlmapper.contains("radioon_sw:"))
		{
			Train->ggRadioOnButton.UpdateValue(1.0f, Train->dsbSwitch);
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{
		if (Train->m_controlmapper.contains("radioon_sw:"))
		{
			Train->ggRadioOnButton.UpdateValue(0.0f, Train->dsbSwitch);
		}
	}
}

void TTrain::OnCommand_radiodisable(TTrain *Train, command_data const &Command)
{
	if (Command.action == GLFW_PRESS)
	{
		Train->mvOccupied->Radio = false;
		// visual feedback
		if (Train->m_controlmapper.contains("radiooff_sw:"))
		{
			Train->ggRadioOffButton.UpdateValue(1.0f, Train->dsbSwitch);
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{
		if (Train->m_controlmapper.contains("radiooff_sw:"))
		{
			Train->ggRadioOffButton.UpdateValue(0.0f, Train->dsbSwitch);
		}
	}
}

// the cut-off in the radio's power supply (eu07/maszyna#438); opening it works like switching the radio off, the
// radio-stop flag included
void TTrain::OnCommand_radiofusetoggle(TTrain *Train, command_data const &Command)
{
	if (Command.action != GLFW_PRESS)
	{
		return;
	}

	// NOTE: flips the state directly rather than through radiofuseenable/radiofusedisable, so it doesn't move the
	// radiofuseon_sw:/radiofuseoff_sw: buttons
	Train->mvOccupied->RadioFuse = (false == Train->mvOccupied->RadioFuse);
}

void TTrain::OnCommand_radiofuseenable(TTrain *Train, command_data const &Command)
{
	if (Command.action == GLFW_PRESS)
	{
		Train->mvOccupied->RadioFuse = true;
		// visual feedback
		if (Train->m_controlmapper.contains("radiofuseon_sw:"))
		{
			Train->ggRadioFuseOnButton.UpdateValue(1.0f, Train->dsbSwitch);
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{
		if (Train->m_controlmapper.contains("radiofuseon_sw:"))
		{
			Train->ggRadioFuseOnButton.UpdateValue(0.0f, Train->dsbSwitch);
		}
	}
}

void TTrain::OnCommand_radiofusedisable(TTrain *Train, command_data const &Command)
{
	if (Command.action == GLFW_PRESS)
	{
		Train->mvOccupied->RadioFuse = false;
		// visual feedback
		if (Train->m_controlmapper.contains("radiofuseoff_sw:"))
		{
			Train->ggRadioFuseOffButton.UpdateValue(1.0f, Train->dsbSwitch);
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{
		if (Train->m_controlmapper.contains("radiofuseoff_sw:"))
		{
			Train->ggRadioFuseOffButton.UpdateValue(0.0f, Train->dsbSwitch);
		}
	}
}

void TTrain::OnCommand_radiochannelincrease(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// the channel buttons of a radio switched off or without power do nothing
		if (Train->mvOccupied->RadioPowered())
		{
			command_data newCommand = Command;
			newCommand.param1 = Train->RadioChannel() + 1;
			OnCommand_radiochannelset(Train, newCommand);
		}

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
		if (Train->mvOccupied->RadioPowered())
		{
			command_data newCommand = Command;
			newCommand.param1 = Train->RadioChannel() - 1;
			OnCommand_radiochannelset(Train, newCommand);
		}

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
		Train->ggRadioChannelSelector.UpdateValue(static_cast<float>(Train->RadioChannel() - 1));
	}
}

void TTrain::OnCommand_radiostopsend(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		if (Train->mvOccupied->RadioPowered())
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
		if (Train->mvOccupied->RadioPowered())
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
		if (Train->RadioChannel() == 10 && Train->mvOccupied->RadioPowered())
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
		if (Train->RadioChannel() != 10 && Train->mvOccupied->RadioPowered())
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
		if (Train->RadioChannel() != 10 && Train->mvOccupied->RadioPowered())
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
		// so do the volume buttons
		if (Train->mvOccupied->RadioPowered())
		{
			command_data newCommand = Command;
			newCommand.param1 = Train->m_radiovolume + 0.125;
			OnCommand_radiovolumeset(Train, newCommand);
		}
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
		if (Train->mvOccupied->RadioPowered())
		{
			command_data newCommand = Command;
			newCommand.param1 = Train->m_radiovolume - 0.125;
			OnCommand_radiovolumeset(Train, newCommand);
		}
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
		Train->m_radiovolume = static_cast<float>(std::clamp(Command.param1, 0.0, 1.0));
		Train->ggRadioVolumeSelector.UpdateValue(Train->m_radiovolume);
		audio::event_volume_change = true;
	}
}

void TTrain::OnCommand_fmradiotoggle(TTrain *Train, command_data const &Command)
{
	if (Command.action == GLFW_PRESS && Train->m_fmradiopresent)
	{
		Train->m_fmradiopower = !Train->m_fmradiopower;
	}
}

void TTrain::OnCommand_fmradiovolumeincrease(TTrain *Train, command_data const &Command)
{
	// the knob keeps turning for as long as the control is held, the actual movement takes place in the update
	Train->m_fmradiovolumechange = (Command.action != GLFW_RELEASE && Train->m_fmradiopresent ? 1 : 0);
}

void TTrain::OnCommand_fmradiovolumedecrease(TTrain *Train, command_data const &Command)
{
	Train->m_fmradiovolumechange = (Command.action != GLFW_RELEASE && Train->m_fmradiopresent ? -1 : 0);
}

void TTrain::OnCommand_fmradiovolumeset(TTrain *Train, command_data const &Command)
{
	if (Command.action != GLFW_RELEASE && Train->m_fmradiopresent)
	{
		Train->m_fmradiovolume = std::clamp(static_cast<float>(Command.param1), 0.f, 1.f);
		if (false == Train->m_controlmapper.contains("fmradio_sw:"))
		{
			// receivers without dedicated switch are switched with the volume knob
			Train->m_fmradiopower = Train->m_fmradiovolume > 0.f;
		}
	}
}

void TTrain::OnCommand_fmradiotuneincrease(TTrain *Train, command_data const &Command)
{
	Train->m_fmradiotunechange = (Command.action != GLFW_RELEASE && Train->m_fmradiopresent ? 1 : 0);
}

void TTrain::OnCommand_fmradiotunedecrease(TTrain *Train, command_data const &Command)
{
	Train->m_fmradiotunechange = (Command.action != GLFW_RELEASE && Train->m_fmradiopresent ? -1 : 0);
}

// sets the receiver to frequency specified in MHz
void TTrain::OnCommand_fmradiotuneset(TTrain *Train, command_data const &Command)
{
	if (Command.action != GLFW_RELEASE && Train->m_fmradiopresent)
	{
		Train->m_fmradiodial = std::clamp(100.f * (static_cast<float>(Command.param1) - audio::EU07_FMRADIO_BANDSTART) / (audio::EU07_FMRADIO_BANDEND - audio::EU07_FMRADIO_BANDSTART), 0.f, 100.f);
		Train->m_fmradiotuned = true;
	}
}
