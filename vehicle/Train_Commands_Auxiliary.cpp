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
#include <future>

void TTrain::OnCommand_fuelpumptoggle(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_REPEAT)
	{
		return;
	}

	if (Train->ggFuelPumpButton.type() == TGaugeType::push)
	{
		// impulse switch
		// currently there's no off button so we always try to turn it on
		OnCommand_fuelpumpenable(Train, Command);
	}
	else
	{
		// two-state switch
		if (Command.action == GLFW_RELEASE)
		{
			return;
		}

		if (false == Train->mvControlled->FuelPump.is_enabled)
		{
			// turn on
			OnCommand_fuelpumpenable(Train, Command);
		}
		else
		{
			// turn off
			OnCommand_fuelpumpdisable(Train, Command);
		}
	}
}

void TTrain::OnCommand_fuelpumpenable(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_REPEAT)
	{
		return;
	}

	if (Train->ggFuelPumpButton.type() == TGaugeType::push)
	{
		// impulse switch
		if (Command.action == GLFW_PRESS)
		{
			// visual feedback
			Train->ggFuelPumpButton.UpdateValue(1.0, Train->dsbSwitch);
			Train->mvControlled->FuelPumpSwitch(true);
		}
		else if (Command.action == GLFW_RELEASE)
		{
			// visual feedback
			Train->ggFuelPumpButton.UpdateValue(0.0, Train->dsbSwitch);
			Train->mvControlled->FuelPumpSwitch(false);
		}
	}
	else
	{
		// two-state switch, only cares about press events
		if (Command.action == GLFW_PRESS)
		{
			// visual feedback
			Train->ggFuelPumpButton.UpdateValue(1.0, Train->dsbSwitch);
			Train->mvControlled->FuelPumpSwitch(true);
			Train->mvControlled->FuelPumpSwitchOff(false);
		}
	}
}

void TTrain::OnCommand_fuelpumpdisable(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_REPEAT)
	{
		return;
	}

	if (Train->ggFuelPumpButton.type() == TGaugeType::push)
	{
		// impulse switch
		// currently there's no disable return type switch
		return;
	}
	else
	{
		// two-state switch, only cares about press events
		if (Command.action == GLFW_PRESS)
		{
			// visual feedback
			Train->ggFuelPumpButton.UpdateValue(0.0, Train->dsbSwitch);
			Train->mvControlled->FuelPumpSwitch(false);
			Train->mvControlled->FuelPumpSwitchOff(true);
		}
	}
}

void TTrain::OnCommand_oilpumptoggle(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_REPEAT)
	{
		return;
	}

	if (Train->ggOilPumpButton.type() == TGaugeType::push)
	{
		// impulse switch
		// currently there's no off button so we always try to turn it on
		OnCommand_oilpumpenable(Train, Command);
	}
	else
	{
		// two-state switch
		if (Command.action == GLFW_RELEASE)
		{
			return;
		}

		if (false == Train->mvControlled->OilPump.is_enabled)
		{
			// turn on
			OnCommand_oilpumpenable(Train, Command);
		}
		else
		{
			// turn off
			OnCommand_oilpumpdisable(Train, Command);
		}
	}
}

void TTrain::OnCommand_oilpumpenable(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_REPEAT)
	{
		return;
	}

	if (Train->ggOilPumpButton.type() == TGaugeType::push)
	{
		// impulse switch
		if (Command.action == GLFW_PRESS)
		{
			// visual feedback
			Train->ggOilPumpButton.UpdateValue(1.0, Train->dsbSwitch);
			Train->mvControlled->OilPumpSwitch(true);
		}
		else if (Command.action == GLFW_RELEASE)
		{
			// visual feedback
			Train->ggOilPumpButton.UpdateValue(0.0, Train->dsbSwitch);
			Train->mvControlled->OilPumpSwitch(false);
		}
	}
	else
	{
		// two-state switch, only cares about press events
		if (Command.action == GLFW_PRESS)
		{
			// visual feedback
			Train->ggOilPumpButton.UpdateValue(1.0, Train->dsbSwitch);
			Train->mvControlled->OilPumpSwitch(true);
			Train->mvControlled->OilPumpSwitchOff(false);
		}
	}
}

void TTrain::OnCommand_oilpumpdisable(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_REPEAT)
	{
		return;
	}

	if (Train->ggOilPumpButton.type() == TGaugeType::push)
	{
		// impulse switch
		// currently there's no disable return type switch
		return;
	}
	else
	{
		// two-state switch, only cares about press events
		if (Command.action == GLFW_PRESS)
		{
			// visual feedback
			Train->ggOilPumpButton.UpdateValue(0.0, Train->dsbSwitch);
			Train->mvControlled->OilPumpSwitch(false);
			Train->mvControlled->OilPumpSwitchOff(true);
		}
	}
}

void TTrain::OnCommand_waterheaterbreakertoggle(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		if (false == Train->mvControlled->WaterHeater.breaker)
		{
			// turn on
			OnCommand_waterheaterbreakerclose(Train, Command);
		}
		else
		{
			// turn off
			OnCommand_waterheaterbreakeropen(Train, Command);
		}
	}
}

void TTrain::OnCommand_waterheaterbreakerclose(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// visual feedback
		Train->ggWaterHeaterBreakerButton.UpdateValue(1.0, Train->dsbSwitch);

		if (true == Train->mvControlled->WaterHeater.breaker)
		{
			return;
		} // already enabled

		Train->mvControlled->WaterHeaterBreakerSwitch(true);
	}
}

void TTrain::OnCommand_waterheaterbreakeropen(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// visual feedback
		Train->ggWaterHeaterBreakerButton.UpdateValue(0.0, Train->dsbSwitch);

		if (false == Train->mvControlled->WaterHeater.breaker)
		{
			return;
		} // already enabled

		Train->mvControlled->WaterHeaterBreakerSwitch(false);
	}
}

void TTrain::OnCommand_waterheatertoggle(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		if (false == Train->mvControlled->WaterHeater.is_enabled)
		{
			// turn on
			OnCommand_waterheaterenable(Train, Command);
		}
		else
		{
			// turn off
			OnCommand_waterheaterdisable(Train, Command);
		}
	}
}

void TTrain::OnCommand_waterheaterenable(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// visual feedback
		Train->ggWaterHeaterButton.UpdateValue(1.0, Train->dsbSwitch);

		if (true == Train->mvControlled->WaterHeater.is_enabled)
		{
			return;
		} // already enabled

		Train->mvControlled->WaterHeaterSwitch(true);
	}
}

void TTrain::OnCommand_waterheaterdisable(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// visual feedback
		Train->ggWaterHeaterButton.UpdateValue(0.0, Train->dsbSwitch);

		if (false == Train->mvControlled->WaterHeater.is_enabled)
		{
			return;
		} // already disabled

		Train->mvControlled->WaterHeaterSwitch(false);
	}
}

void TTrain::OnCommand_waterpumpbreakertoggle(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		if (false == Train->mvControlled->WaterPump.breaker)
		{
			// turn on
			OnCommand_waterpumpbreakerclose(Train, Command);
		}
		else
		{
			// turn off
			OnCommand_waterpumpbreakeropen(Train, Command);
		}
	}
}

void TTrain::OnCommand_waterpumpbreakerclose(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// visual feedback
		Train->ggWaterPumpBreakerButton.UpdateValue(1.0, Train->dsbSwitch);

		if (true == Train->mvControlled->WaterPump.breaker)
		{
			return;
		} // already enabled

		Train->mvControlled->WaterPumpBreakerSwitch(true);
	}
}

void TTrain::OnCommand_waterpumpbreakeropen(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// visual feedback
		Train->ggWaterPumpBreakerButton.UpdateValue(0.0, Train->dsbSwitch);

		if (false == Train->mvControlled->WaterPump.breaker)
		{
			return;
		} // already enabled

		Train->mvControlled->WaterPumpBreakerSwitch(false);
	}
}

void TTrain::OnCommand_waterpumptoggle(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_REPEAT)
	{
		return;
	}

	if (Train->ggWaterPumpButton.type() == TGaugeType::push)
	{
		// impulse switch
		// currently there's no off button so we always try to turn it on
		OnCommand_waterpumpenable(Train, Command);
	}
	else
	{
		// two-state switch
		if (Command.action == GLFW_RELEASE)
		{
			return;
		}

		if (false == Train->mvControlled->WaterPump.is_enabled)
		{
			// turn on
			OnCommand_waterpumpenable(Train, Command);
		}
		else
		{
			// turn off
			OnCommand_waterpumpdisable(Train, Command);
		}
	}
}

void TTrain::OnCommand_waterpumpenable(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_REPEAT)
	{
		return;
	}

	if (Train->ggWaterPumpButton.type() == TGaugeType::push)
	{
		// impulse switch
		if (Command.action == GLFW_PRESS)
		{
			// visual feedback
			Train->ggWaterPumpButton.UpdateValue(1.0, Train->dsbSwitch);
			Train->mvControlled->WaterPumpSwitch(true);
		}
		else if (Command.action == GLFW_RELEASE)
		{
			// visual feedback
			Train->ggWaterPumpButton.UpdateValue(0.0, Train->dsbSwitch);
			Train->mvControlled->WaterPumpSwitch(false);
		}
	}
	else
	{
		// two-state switch, only cares about press events
		if (Command.action == GLFW_PRESS)
		{
			// visual feedback
			Train->ggWaterPumpButton.UpdateValue(1.0, Train->dsbSwitch);
			Train->mvControlled->WaterPumpSwitch(true);
			Train->mvControlled->WaterPumpSwitchOff(false);
		}
	}
}

void TTrain::OnCommand_waterpumpdisable(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_REPEAT)
	{
		return;
	}

	if (Train->ggWaterPumpButton.type() == TGaugeType::push)
	{
		// impulse switch
		// currently there's no disable return type switch
		return;
	}
	else
	{
		// two-state switch, only cares about press events
		if (Command.action == GLFW_PRESS)
		{
			// visual feedback
			Train->ggWaterPumpButton.UpdateValue(0.0, Train->dsbSwitch);
			Train->mvControlled->WaterPumpSwitch(false);
			Train->mvControlled->WaterPumpSwitchOff(true);
		}
	}
}

void TTrain::OnCommand_watercircuitslinktoggle(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		if (false == Train->mvControlled->WaterCircuitsLink)
		{
			// turn on
			OnCommand_watercircuitslinkenable(Train, Command);
		}
		else
		{
			// turn off
			OnCommand_watercircuitslinkdisable(Train, Command);
		}
	}
}

void TTrain::OnCommand_watercircuitslinkenable(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// visual feedback
		Train->ggWaterCircuitsLinkButton.UpdateValue(1.0, Train->dsbSwitch);

		if (true == Train->mvControlled->WaterCircuitsLink)
		{
			return;
		} // already enabled

		Train->mvControlled->WaterCircuitsLinkSwitch(true);
	}
}

void TTrain::OnCommand_watercircuitslinkdisable(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// visual feedback
		Train->ggWaterCircuitsLinkButton.UpdateValue(0.0, Train->dsbSwitch);

		if (false == Train->mvControlled->WaterCircuitsLink)
		{
			return;
		} // already disabled

		Train->mvControlled->WaterCircuitsLinkSwitch(false);
	}
}

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
	else if (Command.action == GLFW_RELEASE && Train->ggTrainHeatingButton.type() == TGaugeType::push)
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
}

void TTrain::OnCommand_heatingdisable(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{

		Train->mvOccupied->HeatingSwitch(false);
		// visual feedback
		Train->ggTrainHeatingButton.UpdateValue(Train->ggTrainHeatingButton.type() == TGaugeType::push ? 1.0 : 0.0, Train->dsbSwitch);
	}
}
