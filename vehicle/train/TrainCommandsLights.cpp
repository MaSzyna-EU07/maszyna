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
#include <algorithm>

void TTrain::OnCommand_wiperswitchincrease(TTrain *Train, command_data const &Command)
{
	if (Command.action == GLFW_PRESS)
	{
		Train->mvOccupied->wiperSwitchPos++;
		if (Train->mvOccupied->wiperSwitchPos > Train->mvOccupied->WiperListSize - 1)
			Train->mvOccupied->wiperSwitchPos = Train->mvOccupied->WiperListSize - 1;

		// Visual feedback
		Train->ggWiperSw.UpdateValue(Train->mvOccupied->wiperSwitchPos, Train->dsbSwitch);
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
		Train->ggWiperSw.UpdateValue(Train->mvOccupied->wiperSwitchPos, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_lightspresetactivatenext(TTrain *Train, command_data const &Command)
{

	if (Train->mvOccupied->LightsPosNo == 0)
	{
		// no preset selector
		return;
	}
	if (Command.action != GLFW_PRESS)
	{
		// one change per key press
		return;
	}

	if (Train->mvOccupied->LightsPos < Train->mvOccupied->LightsPosNo || true == Train->mvOccupied->LightsWrap)
	{
		// active light preset is stored as value in range 1-LigthPosNo
		auto const restartcycle{Train->mvOccupied->LightsPos >= Train->mvOccupied->LightsPosNo};
		Train->mvOccupied->LightsPos = false == restartcycle ? Train->mvOccupied->LightsPos + 1 : 1; // wrap mode

		Train->Dynamic()->SetLights();
		// visual feedback
		if (Train->ggLightsButton.SubModel != nullptr)
		{
			// HACK: skip submodel animation when restarting cycle, since it plays in the 'wrong' direction
			if (false == restartcycle)
			{
				Train->ggLightsButton.UpdateValue(Train->mvOccupied->LightsPos - 1, Train->dsbSwitch);
			}
			else
			{
				Train->ggLightsButton.PutValue(Train->mvOccupied->LightsPos - 1);
			}
		}
	}
}

void TTrain::OnCommand_lightspresetactivateprevious(TTrain *Train, command_data const &Command)
{

	if (Train->mvOccupied->LightsPosNo == 0)
	{
		// no preset selector
		return;
	}
	if (Command.action != GLFW_PRESS)
	{
		// one change per key press
		return;
	}

	if (Train->mvOccupied->LightsPos > 1 || true == Train->mvOccupied->LightsWrap)
	{
		// active light preset is stored as value in range 1-LigthPosNo
		auto const restartcycle{Train->mvOccupied->LightsPos <= 1};
		Train->mvOccupied->LightsPos = false == restartcycle ? Train->mvOccupied->LightsPos - 1 : Train->mvOccupied->LightsPosNo; // wrap mode

		Train->Dynamic()->SetLights();
		// visual feedback
		if (Train->ggLightsButton.SubModel != nullptr)
		{
			// HACK: skip submodel animation when restarting cycle, since it plays in the 'wrong' direction
			if (false == restartcycle)
			{
				Train->ggLightsButton.UpdateValue(Train->mvOccupied->LightsPos - 1, Train->dsbSwitch);
			}
			else
			{
				Train->ggLightsButton.PutValue(Train->mvOccupied->LightsPos - 1);
			}
		}
	}
}

void TTrain::OnCommand_headlighttoggleleft(TTrain *Train, command_data const &Command)
{

	auto const vehicleend{Train->cab_to_end()};

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		if ((Train->mvOccupied->iLights[vehicleend] & light::headlight_left) == 0)
		{
			// turn on
			OnCommand_headlightenableleft(Train, Command);
		}
		else
		{
			// turn off
			OnCommand_headlightdisableleft(Train, Command);
		}
	}
}

void TTrain::OnCommand_lightsset(TTrain *Train, command_data const &Command)
{
	// set custom item in Lights inventory
	Train->mvOccupied->Lights[end::front][17] = Command.param1;
	Train->mvOccupied->Lights[end::rear][17] = Command.param2;
	Train->mvOccupied->LightsPos = 18; // nasza custom pozycja
	Train->Dynamic()->SetLights();
}

void TTrain::OnCommand_headlightenableleft(TTrain *Train, command_data const &Command)
{

	if (Train->mvOccupied->LightsPosNo > 0)
	{
		// lights are controlled by preset selector
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// visual feedback
		Train->ggLeftLightButton.UpdateValue(1.0, Train->dsbSwitch);
		// implementation
		auto const vehicleend{Train->cab_to_end()};

		if ((Train->mvOccupied->iLights[vehicleend] & light::headlight_left) == 0)
		{
			Train->mvOccupied->iLights[vehicleend] ^= light::headlight_left;
		}
		// if the light is controlled by 3-way switch, disable marker light
		if (Train->ggLeftEndLightButton.SubModel == nullptr)
		{
			Train->mvOccupied->iLights[vehicleend] &= ~light::redmarker_left;
		}
	}
}

void TTrain::OnCommand_headlightdisableleft(TTrain *Train, command_data const &Command)
{

	if (Train->mvOccupied->LightsPosNo > 0)
	{
		// lights are controlled by preset selector
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		int const vehicleend{Train->cab_to_end()};

		if ((Train->mvOccupied->iLights[vehicleend] & light::headlight_left) == 0)
		{
			return;
		} // already disabled

		Train->mvOccupied->iLights[vehicleend] ^= light::headlight_left;
		// visual feedback
		Train->ggLeftLightButton.UpdateValue(0.0, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_headlighttoggleright(TTrain *Train, command_data const &Command)
{

	auto const vehicleend{Train->cab_to_end()};

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		if ((Train->mvOccupied->iLights[vehicleend] & light::headlight_right) == 0)
		{
			// turn on
			OnCommand_headlightenableright(Train, Command);
		}
		else
		{
			// turn off
			OnCommand_headlightdisableright(Train, Command);
		}
	}
}

void TTrain::OnCommand_headlightenableright(TTrain *Train, command_data const &Command)
{

	if (Train->mvOccupied->LightsPosNo > 0)
	{
		// lights are controlled by preset selector
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// visual feedback
		Train->ggRightLightButton.UpdateValue(1.0, Train->dsbSwitch);
		// implementation
		auto const vehicleend{Train->cab_to_end()};

		if ((Train->mvOccupied->iLights[vehicleend] & light::headlight_right) == 0)
		{
			Train->mvOccupied->iLights[vehicleend] ^= light::headlight_right;
		}
		// if the light is controlled by 3-way switch, disable marker light
		if (Train->ggRightEndLightButton.SubModel == nullptr)
		{
			Train->mvOccupied->iLights[vehicleend] &= ~light::redmarker_right;
		}
	}
}

void TTrain::OnCommand_headlightdisableright(TTrain *Train, command_data const &Command)
{

	if (Train->mvOccupied->LightsPosNo > 0)
	{
		// lights are controlled by preset selector
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		auto const vehicleend{Train->cab_to_end()};

		if ((Train->mvOccupied->iLights[vehicleend] & light::headlight_right) == 0)
		{
			return;
		} // already disabled

		Train->mvOccupied->iLights[vehicleend] ^= light::headlight_right;
		// visual feedback
		Train->ggRightLightButton.UpdateValue(0.0, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_headlighttoggleupper(TTrain *Train, command_data const &Command)
{

	auto const vehicleend{Train->cab_to_end()};

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		if ((Train->mvOccupied->iLights[vehicleend] & light::headlight_upper) == 0)
		{
			// turn on
			OnCommand_headlightenableupper(Train, Command);
		}
		else
		{
			// turn off
			OnCommand_headlightdisableupper(Train, Command);
		}
	}
}

void TTrain::OnCommand_headlightenableupper(TTrain *Train, command_data const &Command)
{

	if (Train->mvOccupied->LightsPosNo > 0)
	{
		// lights are controlled by preset selector
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		auto const vehicleend{Train->cab_to_end()};

		if ((Train->mvOccupied->iLights[vehicleend] & light::headlight_upper) != 0)
		{
			return;
		} // already enabled

		Train->mvOccupied->iLights[vehicleend] ^= light::headlight_upper;
		// visual feedback
		Train->ggUpperLightButton.UpdateValue(1.0, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_headlightdisableupper(TTrain *Train, command_data const &Command)
{

	if (Train->mvOccupied->LightsPosNo > 0)
	{
		// lights are controlled by preset selector
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		auto const vehicleend{Train->cab_to_end()};

		if ((Train->mvOccupied->iLights[vehicleend] & light::headlight_upper) == 0)
		{
			return;
		} // already disabled

		Train->mvOccupied->iLights[vehicleend] ^= light::headlight_upper;
		// visual feedback
		Train->ggUpperLightButton.UpdateValue(0.0, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_redmarkertoggleleft(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		auto const vehicleend{Train->cab_to_end()};

		if ((Train->mvOccupied->iLights[vehicleend] & light::redmarker_left) == 0)
		{
			// turn on
			OnCommand_redmarkerenableleft(Train, Command);
		}
		else
		{
			// turn off
			OnCommand_redmarkerdisableleft(Train, Command);
		}
	}
}

void TTrain::OnCommand_redmarkerenableleft(TTrain *Train, command_data const &Command)
{

	if (Train->mvOccupied->LightsPosNo > 0)
	{
		// lights are controlled by preset selector
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		auto const vehicleend{Train->cab_to_end()};

		if ((Train->mvOccupied->iLights[vehicleend] & light::redmarker_left) != 0)
		{
			return;
		} // already enabled

		Train->mvOccupied->iLights[vehicleend] ^= light::redmarker_left;
		// visual feedback
		if (Train->ggLeftEndLightButton.SubModel != nullptr)
		{
			Train->ggLeftEndLightButton.UpdateValue(1.0, Train->dsbSwitch);
		}
		else
		{
			// we interpret lack of dedicated switch as a sign the light is controlled with 3-way switch
			// this is crude, but for now will do
			Train->ggLeftLightButton.UpdateValue(-1.0, Train->dsbSwitch);
			// if the light is controlled by 3-way switch, disable the headlight
			Train->mvOccupied->iLights[vehicleend] &= ~light::headlight_left;
		}
	}
}

void TTrain::OnCommand_redmarkerdisableleft(TTrain *Train, command_data const &Command)
{

	if (Train->mvOccupied->LightsPosNo > 0)
	{
		// lights are controlled by preset selector
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		auto const vehicleend{Train->cab_to_end()};

		if ((Train->mvOccupied->iLights[vehicleend] & light::redmarker_left) == 0)
		{
			return;
		} // already disabled

		Train->mvOccupied->iLights[vehicleend] ^= light::redmarker_left;
		// visual feedback
		if (Train->ggLeftEndLightButton.SubModel != nullptr)
		{
			Train->ggLeftEndLightButton.UpdateValue(0.0, Train->dsbSwitch);
		}
		else
		{
			// we interpret lack of dedicated switch as a sign the light is controlled with 3-way switch
			// this is crude, but for now will do
			Train->ggLeftLightButton.UpdateValue(0.0, Train->dsbSwitch);
		}
	}
}

void TTrain::OnCommand_redmarkertoggleright(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		auto const vehicleend{Train->cab_to_end()};

		if ((Train->mvOccupied->iLights[vehicleend] & light::redmarker_right) == 0)
		{
			// turn on
			OnCommand_redmarkerenableright(Train, Command);
		}
		else
		{
			// turn off
			OnCommand_redmarkerdisableright(Train, Command);
		}
	}
}

void TTrain::OnCommand_redmarkerenableright(TTrain *Train, command_data const &Command)
{

	if (Train->mvOccupied->LightsPosNo > 0)
	{
		// lights are controlled by preset selector
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		auto const vehicleend{Train->cab_to_end()};

		if ((Train->mvOccupied->iLights[vehicleend] & light::redmarker_right) != 0)
		{
			return;
		} // already enabled

		Train->mvOccupied->iLights[vehicleend] ^= light::redmarker_right;
		// visual feedback
		if (Train->ggRightEndLightButton.SubModel != nullptr)
		{
			Train->ggRightEndLightButton.UpdateValue(1.0, Train->dsbSwitch);
		}
		else
		{
			// we interpret lack of dedicated switch as a sign the light is controlled with 3-way switch
			// this is crude, but for now will do
			Train->ggRightLightButton.UpdateValue(-1.0, Train->dsbSwitch);
			// if the light is controlled by 3-way switch, disable the headlight
			Train->mvOccupied->iLights[vehicleend] &= ~light::headlight_right;
		}
	}
}

void TTrain::OnCommand_redmarkerdisableright(TTrain *Train, command_data const &Command)
{

	if (Train->mvOccupied->LightsPosNo > 0)
	{
		// lights are controlled by preset selector
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		auto const vehicleend{Train->cab_to_end()};

		if ((Train->mvOccupied->iLights[vehicleend] & light::redmarker_right) == 0)
		{
			return;
		} // already disabled

		Train->mvOccupied->iLights[vehicleend] ^= light::redmarker_right;
		// visual feedback
		if (Train->ggRightEndLightButton.SubModel != nullptr)
		{
			Train->ggRightEndLightButton.UpdateValue(0.0, Train->dsbSwitch);
		}
		else
		{
			// we interpret lack of dedicated switch as a sign the light is controlled with 3-way switch
			// this is crude, but for now will do
			Train->ggRightLightButton.UpdateValue(0.0, Train->dsbSwitch);
		}
	}
}

void TTrain::OnCommand_headlighttogglerearleft(TTrain *Train, command_data const &Command)
{

	if (Train->mvOccupied->LightsPosNo > 0)
	{
		// lights are controlled by preset selector
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// NOTE: we toggle the light on opposite side, as 'rear right' is 'front left' on the rear end etc
		auto const vehicleotherend{(Train->cab_to_end() == end::front ? end::rear : end::front)};
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		if ((Train->mvOccupied->iLights[vehicleotherend] & light::headlight_right) == 0)
		{
			OnCommand_headlightenablerearleft(Train, Command);
		}
		else
		{
			OnCommand_headlightdisablerearleft(Train, Command);
		}
	}
}

void TTrain::OnCommand_headlightenablerearleft(TTrain *Train, command_data const &Command)
{
	if (Train->mvOccupied->LightsPosNo > 0)
	{
		// lights are controlled by preset selector
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		auto const vehicleotherend{(Train->cab_to_end() == end::front ? end::rear : end::front)};
		// already enabled
		if ((Train->mvOccupied->iLights[vehicleotherend] & light::headlight_right) == 0)
		{
			// turn on
			Train->mvOccupied->iLights[vehicleotherend] ^= light::headlight_right;
			// visual feedback
			Train->ggRearLeftLightButton.UpdateValue(1.0, Train->dsbSwitch);
		}
	}
}

void TTrain::OnCommand_headlightdisablerearleft(TTrain *Train, command_data const &Command)
{
	if (Train->mvOccupied->LightsPosNo > 0)
	{
		// lights are controlled by preset selector
		return;
	}
	if (Command.action == GLFW_PRESS)
	{
		auto const vehicleotherend{(Train->cab_to_end() == end::front ? end::rear : end::front)};
		// already disabled
		if ((Train->mvOccupied->iLights[vehicleotherend] & light::headlight_right) == 0)
		{
			return;
		}

		// turn off
		Train->mvOccupied->iLights[vehicleotherend] ^= light::headlight_right;
		// visual feedback
		Train->ggRearLeftLightButton.UpdateValue(0.0, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_headlighttogglerearright(TTrain *Train, command_data const &Command)
{
	if (Command.action == GLFW_PRESS)
	{
		// NOTE: we toggle the light on opposite side, as 'rear right' is 'front left' on the rear end etc
		auto const vehicleotherend{(Train->cab_to_end() == end::front ? end::rear : end::front)};
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		if ((Train->mvOccupied->iLights[vehicleotherend] & light::headlight_left) == 0)
		{
			OnCommand_headlightenablerearright(Train, Command);
		}
		else
		{
			OnCommand_headlightdisablerearright(Train, Command);
		}
	}
}

void TTrain::OnCommand_headlightenablerearright(TTrain *Train, command_data const &Command)
{
	if (Train->mvOccupied->LightsPosNo > 0)
	{
		// lights are controlled by preset selector
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// NOTE: we toggle the light on opposite side, as 'rear right' is 'front left' on the rear end etc
		auto const vehicleotherend{(Train->cab_to_end() == end::front ? end::rear : end::front)};

		if ((Train->mvOccupied->iLights[vehicleotherend] & light::headlight_left) == 0)
		{
			// turn on
			Train->mvOccupied->iLights[vehicleotherend] ^= light::headlight_left;
			// visual feedback
			Train->ggRearRightLightButton.UpdateValue(1.0, Train->dsbSwitch);
		}
	}
}

void TTrain::OnCommand_headlightdisablerearright(TTrain *Train, command_data const &Command)
{
	if (Train->mvOccupied->LightsPosNo > 0)
	{
		// lights are controlled by preset selector
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// NOTE: we toggle the light on opposite side, as 'rear right' is 'front left' on the rear end etc
		auto const vehicleotherend{(Train->cab_to_end() == end::front ? end::rear : end::front)};
		// already disabled
		if ((Train->mvOccupied->iLights[vehicleotherend] & light::headlight_left) == 0)
		{
			return;
		}

		// turn off
		Train->mvOccupied->iLights[vehicleotherend] ^= light::headlight_left;
		// visual feedback
		Train->ggRearRightLightButton.UpdateValue(0.0, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_headlighttogglerearupper(TTrain *Train, command_data const &Command)
{

	if (Train->mvOccupied->LightsPosNo > 0)
	{
		// lights are controlled by preset selector
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		auto const vehicleotherend{(Train->cab_to_end() == end::front ? end::rear : end::front)};
		if ((Train->mvOccupied->iLights[vehicleotherend] & light::headlight_upper) == 0)
		{
			OnCommand_headlightenablerearupper(Train, Command);
		}
		else
		{
			OnCommand_headlightdisablerearupper(Train, Command);
		}
	}
}

void TTrain::OnCommand_headlightenablerearupper(TTrain *Train, command_data const &Command)
{

	if (Train->mvOccupied->LightsPosNo > 0)
	{
		// lights are controlled by preset selector
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		auto const vehicleotherend{(Train->cab_to_end() == end::front ? end::rear : end::front)};
		if ((Train->mvOccupied->iLights[vehicleotherend] & light::headlight_upper) == 0)
		{
			// turn on
			Train->mvOccupied->iLights[vehicleotherend] ^= light::headlight_upper;
			// visual feedback
			Train->ggRearUpperLightButton.UpdateValue(1.0, Train->dsbSwitch);
		}
	}
}

void TTrain::OnCommand_headlightdisablerearupper(TTrain *Train, command_data const &Command)
{

	if (Train->mvOccupied->LightsPosNo > 0)
	{
		// lights are controlled by preset selector
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		auto const vehicleotherend{(Train->cab_to_end() == end::front ? end::rear : end::front)};
		// already disabled?
		if ((Train->mvOccupied->iLights[vehicleotherend] & light::headlight_upper) == 0)
		{
			return;
		}

		// turn off
		Train->mvOccupied->iLights[vehicleotherend] ^= light::headlight_upper;
		// visual feedback
		Train->ggRearUpperLightButton.UpdateValue(0.0, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_modernlightdimmerincrease(TTrain *Train, command_data const &Command)
{
	if (!Train->mvOccupied->enableModernDimmer)
		return; // if modern dimmer is disabled, skip entire command
	if (Command.action == GLFW_PRESS)
	{
		// update modern dimmer state

		auto &dimPos = Train->mvOccupied->modernDimmerPosition;
		auto dimCount = Train->mvOccupied->dimPositions.size();
		if (dimPos + 1 < dimCount)
			dimPos++;
		else if (Train->mvOccupied->modernDimmerCanCycle)
			dimPos = 0; // return to 0
		else
			return; // już na minimum i nie można cyklicznie
		// update lightning
		// Train->Dynamic()->SetLights();
		Train->Dynamic()->SetLightDimmings();

		// visual feedback
		if (Train->ggModernLightDimSw.SubModel != nullptr)
			Train->ggModernLightDimSw.UpdateValue(dimPos, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_modernlightdimmerdecrease(TTrain *Train, command_data const &Command)
{
	if (!Train->mvOccupied->enableModernDimmer)
		return; // jeśli dimmer jest wyłączony, olewamy

	if (Command.action == GLFW_PRESS)
	{
		auto &dimPos = Train->mvOccupied->modernDimmerPosition;
		auto dimCount = Train->mvOccupied->dimPositions.size();

		if (dimCount == 0)
			return;

		if (dimPos > 0)
			dimPos--;
		else if (Train->mvOccupied->modernDimmerCanCycle)
			dimPos = static_cast<int>(dimCount - 1); // ostatnia pozycja
		else
			return; // już na minimum i nie można cyklicznie

		// Train->Dynamic()->SetLights();
		Train->Dynamic()->SetLightDimmings();

		if (Train->ggModernLightDimSw.SubModel != nullptr)
			Train->ggModernLightDimSw.UpdateValue(dimPos, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_redmarkertogglerearleft(TTrain *Train, command_data const &Command)
{
	if (Command.action == GLFW_PRESS)
	{
		// NOTE: we toggle the light on opposite side, as 'rear right' is 'front left' on the rear end etc
		auto const vehicleotherend{(Train->cab_to_end() == end::front ? end::rear : end::front)};
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		if ((Train->mvOccupied->iLights[vehicleotherend] & light::redmarker_right) == 0)
		{
			OnCommand_redmarkerenablerearleft(Train, Command);
		}
		else
		{
			OnCommand_redmarkerdisablerearleft(Train, Command);
		}
	}
}

void TTrain::OnCommand_redmarkerenablerearleft(TTrain *Train, command_data const &Command)
{

	if (Train->mvOccupied->LightsPosNo > 0)
	{
		// lights are controlled by preset selector
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// NOTE: we toggle the light on opposite side, as 'rear right' is 'front left' on the rear end etc
		auto const vehicleotherend{(Train->cab_to_end() == end::front ? end::rear : end::front)};
		if ((Train->mvOccupied->iLights[vehicleotherend] & light::redmarker_right) == 0)
		{
			// turn on
			Train->mvOccupied->iLights[vehicleotherend] ^= light::redmarker_right;
			// visual feedback
			Train->ggRearLeftEndLightButton.UpdateValue(1.0, Train->dsbSwitch);
		}
	}
}

void TTrain::OnCommand_redmarkerdisablerearleft(TTrain *Train, command_data const &Command)
{

	if (Train->mvOccupied->LightsPosNo > 0)
	{
		// lights are controlled by preset selector
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// NOTE: we toggle the light on opposite side, as 'rear right' is 'front left' on the rear end etc
		auto const vehicleotherend{(Train->cab_to_end() == end::front ? end::rear : end::front)};
		if ((Train->mvOccupied->iLights[vehicleotherend] & light::redmarker_right) == 0)
		{
			return;
		}
		// turn off
		Train->mvOccupied->iLights[vehicleotherend] ^= light::redmarker_right;
		// visual feedback
		Train->ggRearLeftEndLightButton.UpdateValue(0.0, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_redmarkertogglerearright(TTrain *Train, command_data const &Command)
{

	if (Train->mvOccupied->LightsPosNo > 0)
	{
		// lights are controlled by preset selector
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// NOTE: we toggle the light on opposite side, as 'rear right' is 'front left' on the rear end etc
		auto const vehicleotherend{(Train->cab_to_end() == end::front ? end::rear : end::front)};
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		if ((Train->mvOccupied->iLights[vehicleotherend] & light::redmarker_left) == 0)
		{
			OnCommand_redmarkerenablerearright(Train, Command);
		}
		else
		{
			OnCommand_redmarkerdisablerearright(Train, Command);
		}
	}
}

void TTrain::OnCommand_redmarkerenablerearright(TTrain *Train, command_data const &Command)
{

	if (Train->mvOccupied->LightsPosNo > 0)
	{
		// lights are controlled by preset selector
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// NOTE: we toggle the light on opposite side, as 'rear right' is 'front left' on the rear end etc
		auto const vehicleotherend{(Train->cab_to_end() == end::front ? end::rear : end::front)};
		if ((Train->mvOccupied->iLights[vehicleotherend] & light::redmarker_left) == 0)
		{
			// turn on
			Train->mvOccupied->iLights[vehicleotherend] ^= light::redmarker_left;
			// visual feedback
			Train->ggRearRightEndLightButton.UpdateValue(1.0, Train->dsbSwitch);
		}
	}
}

void TTrain::OnCommand_redmarkerdisablerearright(TTrain *Train, command_data const &Command)
{

	if (Train->mvOccupied->LightsPosNo > 0)
	{
		// lights are controlled by preset selector
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// NOTE: we toggle the light on opposite side, as 'rear right' is 'front left' on the rear end etc
		auto const vehicleotherend{(Train->cab_to_end() == end::front ? end::rear : end::front)};
		if ((Train->mvOccupied->iLights[vehicleotherend] & light::redmarker_left) == 0)
		{
			return;
		}
		// turn off
		Train->mvOccupied->iLights[vehicleotherend] ^= light::redmarker_left;
		// visual feedback
		Train->ggRearRightEndLightButton.UpdateValue(0.0, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_redmarkerstoggle(TTrain *Train, command_data const &Command)
{

	if (true == Command.freefly && Command.action == GLFW_PRESS)
	{

		auto *vehicle{std::get<TDynamicObject *>(simulation::Region->find_vehicle(Command.location, 10, false, true))};

		if (vehicle == nullptr)
		{
			return;
		}

		auto locationHead = vehicle->HeadPosition() - glm::dvec3(Command.location); // TODO: Maybe command_data should be dvec3?
		auto locationRear = vehicle->RearPosition() - glm::dvec3(Command.location);
		int const CouplNr{std::clamp(vehicle->DirectionGet() * (glm::dot(locationHead, locationHead) > glm::dot(locationRear, locationRear) ? 1 : -1), 0, 1)}; // z [-1,1] zrobić [0,1]

		auto const lightset{light::redmarker_left | light::redmarker_right};

		vehicle->MoverParameters->iLights[CouplNr] = false == TestFlag(vehicle->MoverParameters->iLights[CouplNr], lightset) ?
		                                                 vehicle->MoverParameters->iLights[CouplNr] |= lightset : // turn signals on
		                                                 vehicle->MoverParameters->iLights[CouplNr] ^= lightset; // turn signals off
	}
}

void TTrain::OnCommand_endsignalstoggle(TTrain *Train, command_data const &Command)
{

	if (true == Command.freefly && Command.action == GLFW_PRESS)
	{

		auto *vehicle{std::get<TDynamicObject *>(simulation::Region->find_vehicle(Command.location, 10, false, true))};

		if (vehicle == nullptr)
		{
			return;
		}
		int const CouplNr{
		    std::clamp(vehicle->DirectionGet() * (glm::length2(vehicle->HeadPosition() - glm::dvec3(Command.location)) > glm::length2(vehicle->RearPosition() - glm::dvec3(Command.location)) ? 1 : -1),
		               0, 1)}; // z [-1,1] zrobić [0,1]

		auto const lightset{light::rearendsignals};

		vehicle->MoverParameters->iLights[CouplNr] = false == TestFlag(vehicle->MoverParameters->iLights[CouplNr], lightset) ?
		                                                 vehicle->MoverParameters->iLights[CouplNr] |= lightset : // turn signals on
		                                                 vehicle->MoverParameters->iLights[CouplNr] ^= lightset; // turn signals off
	}
}

void TTrain::OnCommand_headlightsdimtoggle(TTrain *Train, command_data const &Command)
{
	if (Train->DynamicObject->MoverParameters->enableModernDimmer)
		return;
	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		if (Train->DynamicObject->MoverParameters->modernDimmerPosition == 0)
		{
			// turn on
			OnCommand_headlightsdimenable(Train, Command);
		}
		else
		{
			// turn off
			OnCommand_headlightsdimdisable(Train, Command);
		}
	}
}

void TTrain::OnCommand_headlightsdimenable(TTrain *Train, command_data const &Command)
{

	if (Train->DynamicObject->MoverParameters->enableModernDimmer)
		return;
	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		if (Train->ggDimHeadlightsButton.SubModel != nullptr)
		{
			// TODO: proper control deviced definition for the interiors, that doesn't hinge of presence of 3d submodels
			// visual feedback
			Train->ggDimHeadlightsButton.UpdateValue(1.0, Train->dsbSwitch);
		}

		/* to jest stara logika
		if (true == Train->DynamicObject->DimHeadlights)
		{
		    return;
		} already enabled

		Train->DynamicObject->DimHeadlights = true;
		*/
		Train->DynamicObject->MoverParameters->modernDimmerPosition = 1; // ustawiamy modern dimmer na flage przyciemnienia
		Train->DynamicObject->SetLightDimmings();
	}
}

void TTrain::OnCommand_headlightsdimdisable(TTrain *Train, command_data const &Command)
{

	if (Train->DynamicObject->MoverParameters->enableModernDimmer) // nie wiem dlaczego to tak dziala ze jest odwrocona logika
		return;
	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		if (Train->ggDimHeadlightsButton.SubModel != nullptr)
		{
			// TODO: proper control deviced definition for the interiors, that doesn't hinge of presence of 3d submodels
			// visual feedback
			Train->ggDimHeadlightsButton.UpdateValue(0.0, Train->dsbSwitch);
		}

		/* stara logika przyciemniania
		if( false == Train->DynamicObject->DimHeadlights ) { return; } already enabled

		Train->DynamicObject->DimHeadlights = false;

		*/
		Train->DynamicObject->MoverParameters->modernDimmerPosition = 0; // ustawiamy modern dimmer na flage rozjasnienia
		Train->DynamicObject->SetLightDimmings();
	}
}

void TTrain::OnCommand_interiorlighttoggle(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		if (false == Train->Cabine[Train->iCabn].bLight)
		{
			// turn on
			OnCommand_interiorlightenable(Train, Command);
		}
		else
		{
			// turn off
			OnCommand_interiorlightdisable(Train, Command);
		}
	}
}

void TTrain::OnCommand_interiorlightenable(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		if (false == Train->m_controlmapper.contains("cablight_sw:"))
		{
			// TODO: proper control deviced definition for the interiors, that doesn't hinge of presence of 3d submodels
			WriteLog("Interior Light switch is missing, or wasn't defined");
			return;
		}
		// store lighting switch states
		if (false == Train->DynamicObject->JointCabs)
		{
			// vehicles with separate cabs get separate lighting switch states
			Train->Cabine[Train->iCabn].bLight = true;
		}
		else
		{
			// joint virtual cabs share lighting switch states
			for (auto &cab : Train->Cabine)
			{
				cab.bLight = true;
			}
		}
	}
}

void TTrain::OnCommand_interiorlightdisable(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		if (false == Train->m_controlmapper.contains("cablight_sw:"))
		{
			// TODO: proper control deviced definition for the interiors, that doesn't hinge of presence of 3d submodels
			WriteLog("Interior Light switch is missing, or wasn't defined");
			return;
		}
		// store lighting switch states
		if (false == Train->DynamicObject->JointCabs)
		{
			// vehicles with separate cabs get separate lighting switch states
			Train->Cabine[Train->iCabn].bLight = false;
		}
		else
		{
			// joint virtual cabs share lighting switch states
			for (auto &cab : Train->Cabine)
			{
				cab.bLight = false;
			}
		}
	}
}

void TTrain::OnCommand_interiorlightdimtoggle(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		if (false == Train->Cabine[Train->iCabn].bLightDim)
		{
			// turn on
			OnCommand_interiorlightdimenable(Train, Command);
		}
		else
		{
			// turn off
			OnCommand_interiorlightdimdisable(Train, Command);
		}
	}
}

void TTrain::OnCommand_interiorlightdimenable(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		if (Train->ggCabLightDimButton.SubModel == nullptr)
		{
			// TODO: proper control deviced definition for the interiors, that doesn't hinge of presence of 3d submodels
			WriteLog("Dim Interior Light switch is missing, or wasn't defined");
			return;
		}
		// visual feedback
		Train->ggCabLightDimButton.UpdateValue(1.0, Train->dsbSwitch);
		// store lighting switch states
		if (false == Train->DynamicObject->JointCabs)
		{
			// vehicles with separate cabs get separate lighting switch states
			Train->Cabine[Train->iCabn].bLightDim = true;
		}
		else
		{
			// joint virtual cabs share lighting switch states
			for (auto &cab : Train->Cabine)
			{
				cab.bLightDim = true;
			}
		}
	}
}

void TTrain::OnCommand_interiorlightdimdisable(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		if (Train->ggCabLightDimButton.SubModel == nullptr)
		{
			// TODO: proper control deviced definition for the interiors, that doesn't hinge of presence of 3d submodels
			WriteLog("Dim Interior Light switch is missing, or wasn't defined");
			return;
		}
		// visual feedback
		Train->ggCabLightDimButton.UpdateValue(0.0, Train->dsbSwitch);
		// store lighting switch states
		if (false == Train->DynamicObject->JointCabs)
		{
			// vehicles with separate cabs get separate lighting switch states
			Train->Cabine[Train->iCabn].bLightDim = false;
		}
		else
		{
			// joint virtual cabs share lighting switch states
			for (auto &cab : Train->Cabine)
			{
				cab.bLightDim = false;
			}
		}
	}
}

void TTrain::OnCommand_compartmentlightstoggle(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_REPEAT)
	{
		return;
	}

	// keep the switch from flipping back and forth if key is held down
	if (false == Train->mvOccupied->CompartmentLights.is_active && false == Train->mvOccupied->CompartmentLights.is_enabled)
	{
		// turn on
		OnCommand_compartmentlightsenable(Train, Command);
	}
	else
	{
		// turn off
		OnCommand_compartmentlightsdisable(Train, Command);
	}
}

void TTrain::OnCommand_compartmentlightsenable(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		Train->mvOccupied->CompartmentLightsSwitch(true);
		if (Train->m_controlmapper.contains("compartmentlights_sw:"))
		{
			auto const istoggle{(static_cast<int>(Train->ggCompartmentLightsButton.type()) & static_cast<int>(TGaugeType::toggle)) != 0};
			if (istoggle)
			{
				Train->mvOccupied->CompartmentLightsSwitchOff(false);
			}
		}
		// visual feedback
		if (Train->m_controlmapper.contains("compartmentlights_sw:"))
		{
			Train->ggCompartmentLightsButton.UpdateValue(1.0f, Train->dsbSwitch);
		}
		if (Train->m_controlmapper.contains("compartmentlightson_sw:"))
		{
			Train->ggCompartmentLightsOnButton.UpdateValue(1.0f, Train->dsbSwitch);
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{
		if (Train->m_controlmapper.contains("compartmentlights_sw:") && Train->ggCompartmentLightsButton.type() == TGaugeType::push)
		{
			// return the switch to neutral position
			Train->mvOccupied->CompartmentLightsSwitch(false);
			Train->mvOccupied->CompartmentLightsSwitchOff(false);
			Train->ggCompartmentLightsButton.UpdateValue(0.5f);
		}
		if (Train->m_controlmapper.contains("compartmentlightson_sw:"))
		{
			Train->mvOccupied->CompartmentLightsSwitch(false);
			Train->ggCompartmentLightsOnButton.UpdateValue(0.0f, Train->dsbSwitch);
		}
	}
}

void TTrain::OnCommand_compartmentlightsdisable(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		Train->mvOccupied->CompartmentLightsSwitchOff(true);
		if (Train->m_controlmapper.contains("compartmentlights_sw:"))
		{
			auto const istoggle{(static_cast<int>(Train->ggCompartmentLightsButton.type()) & static_cast<int>(TGaugeType::toggle)) != 0};
			if (istoggle)
			{
				Train->mvOccupied->CompartmentLightsSwitch(false);
			}
		}
		// visual feedback
		if (Train->m_controlmapper.contains("compartmentlights_sw:"))
		{
			Train->ggCompartmentLightsButton.UpdateValue(0.0f, Train->dsbSwitch);
		}
		if (Train->m_controlmapper.contains("compartmentlightsoff_sw:"))
		{
			Train->ggCompartmentLightsOffButton.UpdateValue(1.0f, Train->dsbSwitch);
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{
		if (Train->m_controlmapper.contains("compartmentlights_sw:") && Train->ggCompartmentLightsButton.type() == TGaugeType::push)
		{
			// return the switch to neutral position
			Train->mvOccupied->CompartmentLightsSwitch(false);
			Train->mvOccupied->CompartmentLightsSwitchOff(false);
			Train->ggCompartmentLightsButton.UpdateValue(0.5f);
		}
		if (Train->m_controlmapper.contains("compartmentlightsoff_sw:"))
		{
			Train->mvOccupied->CompartmentLightsSwitchOff(false);
			Train->ggCompartmentLightsOffButton.UpdateValue(0.0f, Train->dsbSwitch);
		}
	}
}

void TTrain::OnCommand_instrumentlighttoggle(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		if (false == Train->InstrumentLightActive)
		{
			// turn on
			OnCommand_instrumentlightenable(Train, Command);
		}
		else
		{
			// turn off
			OnCommand_instrumentlightdisable(Train, Command);
		}
	}
}

void TTrain::OnCommand_instrumentlightenable(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		if (Train->ggInstrumentLightButton.SubModel == nullptr)
		{
			// TODO: proper control deviced definition for the interiors, that doesn't hinge of presence of 3d submodels
			WriteLog("Instrument Light switch is missing, or wasn't defined");
			return;
		}
		// visual feedback
		Train->ggInstrumentLightButton.UpdateValue(1.0, Train->dsbSwitch);

		if (true == Train->InstrumentLightActive)
		{
			return;
		} // already enabled

		Train->InstrumentLightActive = true;
	}
}

void TTrain::OnCommand_instrumentlightdisable(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		if (Train->ggInstrumentLightButton.SubModel == nullptr)
		{
			// TODO: proper control deviced definition for the interiors, that doesn't hinge of presence of 3d submodels
			WriteLog("Instrument Light switch is missing, or wasn't defined");
			return;
		}
		// visual feedback
		Train->ggInstrumentLightButton.UpdateValue(0.0, Train->dsbSwitch);

		if (false == Train->InstrumentLightActive)
		{
			return;
		} // already disabled

		Train->InstrumentLightActive = false;
	}
}

void TTrain::OnCommand_dashboardlighttoggle(TTrain *Train, command_data const &Command)
{
	if (false == Train->DashboardLightActive)
	{
		OnCommand_dashboardlightenable(Train, Command);
	}
	else
	{
		OnCommand_dashboardlightdisable(Train, Command);
	}
}

void TTrain::OnCommand_dashboardlightenable(TTrain *Train, command_data const &Command)
{
	// only reacting to press, so the switch doesn't flip back and forth if key is held down
	if (Command.action != GLFW_PRESS)
	{
		return;
	}

	if (Train->ggDashboardLightButton.SubModel == nullptr)
	{
		// TODO: proper control deviced definition for the interiors, that doesn't hinge of presence of 3d submodels
		WriteLog("Dashboard Light switch is missing, or wasn't defined");
		return;
	}

	if (false == Train->DashboardLightActive)
	{
		// turn on
		Train->DashboardLightActive = true;
		// visual feedback
		Train->ggDashboardLightButton.UpdateValue(1.0, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_dashboardlightdisable(TTrain *Train, command_data const &Command)
{
	// only reacting to press, so the switch doesn't flip back and forth if key is held down
	if (Command.action != GLFW_PRESS)
	{
		return;
	}

	if (Train->ggDashboardLightButton.SubModel == nullptr)
	{
		// TODO: proper control deviced definition for the interiors, that doesn't hinge of presence of 3d submodels
		WriteLog("Dashboard Light switch is missing, or wasn't defined");
		return;
	}

	if (Train->DashboardLightActive)
	{
		// turn off
		Train->DashboardLightActive = false;
		// visual feedback
		Train->ggDashboardLightButton.UpdateValue(0.0, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_timetablelighttoggle(TTrain *Train, command_data const &Command)
{
	// only reacting to press, so the switch doesn't flip back and forth if key is held down
	if (Command.action != GLFW_PRESS)
	{
		return;
	}

	if (false == Train->TimetableLightActive)
	{
		OnCommand_timetablelightenable(Train, Command);
	}
	else
	{
		OnCommand_timetablelightdisable(Train, Command);
	}
}

void TTrain::OnCommand_timetablelightenable(TTrain *Train, command_data const &Command)
{
	// only reacting to press, so the switch doesn't flip back and forth if key is held down
	if (Command.action != GLFW_PRESS)
	{
		return;
	}

	if (Train->ggTimetableLightButton.SubModel == nullptr)
	{
		// TODO: proper control deviced definition for the interiors, that doesn't hinge of presence of 3d submodels
		WriteLog("Timetable Light switch is missing, or wasn't defined");
		return;
	}

	if (false == Train->TimetableLightActive)
	{
		// turn on
		Train->TimetableLightActive = true;
		// visual feedback
		Train->ggTimetableLightButton.UpdateValue(1.0, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_timetablelightdisable(TTrain *Train, command_data const &Command)
{
	// only reacting to press, so the switch doesn't flip back and forth if key is held down
	if (Command.action != GLFW_PRESS)
	{
		return;
	}

	if (Train->ggTimetableLightButton.SubModel == nullptr)
	{
		// TODO: proper control deviced definition for the interiors, that doesn't hinge of presence of 3d submodels
		WriteLog("Timetable Light switch is missing, or wasn't defined");
		return;
	}
	if (Train->TimetableLightActive)
	{
		// turn off
		Train->TimetableLightActive = false;
		// visual feedback
		Train->ggTimetableLightButton.UpdateValue(0.0, Train->dsbSwitch);
	}
}
