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

// the commands of the line breaker, the traction motors and their relays

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


// whether the cab's main_off_bt is the bistable lever of the line breaker's hold coil (eu07/maszyna#364)
bool is_hold_coil_lever(TGauge const &Button)
{
	return Button.SubModel != nullptr && hold_coil_lever::is_bistable(Button.type_declared(), Button.type() == TGaugeType::toggle);
}

void TTrain::OnCommand_linebreakertoggle(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// press or hold...
		// a single two-state lever stays where it was put, also when the line breaker gets knocked out
		auto const latchinglever{Train->ggMainOnButton.SubModel == nullptr && Train->ggMainOffButton.SubModel == nullptr && Train->ggMainButton.SubModel != nullptr &&
		                         Train->ggMainButton.type() == TGaugeType::toggle};
		switch (linebreaker::pressed(Train->m_linebreakerstate, latchinglever, Train->ggMainButton.GetDesiredValue() > 0.5))
		{
		case linebreaker::toggle::close:
			// ...to close the circuit
			if (is_hold_coil_lever(Train->ggMainOffButton) && hold_coil_lever::cuts_off(Train->ggMainOffButton.GetDesiredValue()))
			{
				// the one key works both controls: the hold coil lever goes back on first, or the line breaker couldn't close
				Train->ggMainOffButton.UpdateValue(0.0, Train->dsbSwitch);
			}
			// NOTE: bit of a dirty shortcut here
			OnCommand_linebreakerclose(Train, Command);
			break;
		case linebreaker::toggle::open:
			// ...to open the circuit, or to put the lever off if the circuit was already opened
			OnCommand_linebreakeropen(Train, Command);
			break;
		default:
			break;
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// release...
		if (Train->ggMainOnButton.SubModel != nullptr || Train->ggMainButton.type() != TGaugeType::toggle)
		{
			// only impulse switches react to release events
			// NOTE: we presume dedicated state switch is of impulse type
			if (Train->m_linebreakerstate == linebreaker::state::open)
			{
				// ...after opening circuit, or holding for too short time to close it
				OnCommand_linebreakeropen(Train, Command);
			}
			else
			{
				// ...after closing the circuit
				// NOTE: bit of a dirty shortcut here
				OnCommand_linebreakerclose(Train, Command);
			}
		}
		// HACK: ignition key ignores lack of submodel, so we can start vehicles without any modeled controls
		Train->ggIgnitionKey.UpdateValue(0.0);
	}
}

void TTrain::OnCommand_linebreakeropen(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// visual feedback
		if (is_hold_coil_lever(Train->ggMainOffButton))
		{
			// the bistable lever goes over to its other position and stays there
			auto const position{hold_coil_lever::operated(Train->ggMainOffButton.GetDesiredValue())};
			Train->ggMainOffButton.UpdateValue(position, Train->dsbSwitch);
			if (false == hold_coil_lever::cuts_off(position))
			{
				// the hold coil is fed again, which by itself doesn't close the line breaker
				return;
			}
		}
		else if (Train->ggMainOffButton.SubModel != nullptr)
		{
			Train->ggMainOffButton.UpdateValue(1.0, Train->dsbSwitch);
		}
		else if (Train->ggMainButton.SubModel != nullptr)
		{
			Train->ggMainButton.UpdateValue(0.0, Train->dsbSwitch);
		}
		else if (Train->ggMainOnButton.SubModel != nullptr && false == Train->mvControlled->MainCircuitOpensWithStarterOnly())
		{
			// NOTE: legacy behaviour, for vehicles equipped only with impulse close switch
			// it doesn't make any real sense to animate this one, but some people can't get over how there's no visual reaction to their keypress
			Train->ggMainOnButton.UpdateValue(1.0, Train->dsbSwitch);
			return;
		}
		// a diesel with only the starter button gets its engine stopped. the button stays put: held down, it would start the engine again
		// play sound immediately when the switch is hit, not after release
		Train->fMainRelayTimer = 0.0f;

		if (Train->m_linebreakerstate == linebreaker::state::open)
		{
			return;
		} // already in the desired state

		if (true == Train->mvControlled->MainSwitch(false, Train->mvControlled->MainSwitchRange()))
		{
			Train->m_linebreakerstate = linebreaker::state::open;
			if (Train->mvControlled->MainSwitchRange() != range_t::local)
			{
				// the engines of the vehicles in multiple stopped along: their two-state switches go off too, left on
				// they'd start them again after the delay
				for (auto &controls : Train->m_remotelinebreakers)
				{
					if (controls.main_switch.SubModel != nullptr && controls.main_switch.type() == TGaugeType::toggle)
					{
						controls.main_switch.UpdateValue(0.0);
					}
				}
			}
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// visual feedback
		// we don't exactly know which of the two buttons was used, so reset both
		// for setup with two separate swiches
		if (Train->ggMainOnButton.SubModel != nullptr)
		{
			Train->ggMainOnButton.UpdateValue(0.0, Train->dsbSwitch);
		}
		if (Train->ggMainOffButton.SubModel != nullptr && false == is_hold_coil_lever(Train->ggMainOffButton))
		{
			// (the bistable hold coil lever stays where it was put)
			Train->ggMainOffButton.UpdateValue(0.0, Train->dsbSwitch);
		}
		// and the two-state switch too, for good measure
		if (Train->ggMainButton.SubModel != nullptr)
		{
			Train->ggMainButton.UpdateValue(Train->ggMainButton.type() != TGaugeType::toggle ? 0.5 : 0.0, Train->dsbSwitch);
		}
	}
}

void TTrain::OnCommand_linebreakerclose(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// visual feedback
		if (Train->ggMainOnButton.SubModel != nullptr)
		{
			// two separate switches to close and break the circuit
			Train->ggMainOnButton.UpdateValue(1.0, Train->dsbSwitch);
		}
		else if (Train->ggMainButton.SubModel != nullptr)
		{
			// single two-state switch
			Train->ggMainButton.UpdateValue(1.0, Train->dsbSwitch);
		}
		else
		{
			// no switch capable of doing the job
			// HACK: ignition key ignores lack of submodel, so we can start vehicles without any modeled controls
			Train->ggIgnitionKey.UpdateValue(1.0);
			return;
		}
		// the actual closing of the line breaker is handled in the train update routine
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// visual feedback
		if (Train->ggMainOnButton.SubModel != nullptr)
		{
			// setup with two separate switches
			Train->ggMainOnButton.UpdateValue(0.0, Train->dsbSwitch);
		}
		else if (Train->ggMainButton.SubModel != nullptr && Train->ggMainButton.type() != TGaugeType::toggle)
		{
			Train->ggMainButton.UpdateValue(0.5, Train->dsbSwitch);
		}

		if (Train->m_linebreakerstate == linebreaker::state::closed)
		{
			return;
		} // already in the desired state

		// we don't need to start the diesel twice, but the other types (with impulse switch setup) still need to be launched
		// NOTE: this behaviour should depend on MainOnButton presence and type_delayed
		// TODO: change it when/if vehicle definition files get their proper switch types
		if (Train->m_linebreakerstate == linebreaker::state::closing && Train->mvControlled->EngineType == TEngineType::ElectricSeriesMotor)
		{
			// try to finalize state change of the line breaker, set the state based on the outcome
			Train->m_linebreakerstate = Train->mvControlled->MainSwitch(true, Train->mvControlled->MainSwitchRange()) ? linebreaker::state::closed : linebreaker::state::open;
		}
		// on button release reset the closing timer
		Train->fMainRelayTimer = 0.0f;
	}
}
namespace
{


// which of the switches of the other vehicles in multiple a command works: 0 the second vehicle's, 1 the third one's
std::size_t remote_linebreaker_index(user_command const Command)
{
	using enum user_command;
	return (Command == linebreakertogglec || Command == linebreakeropenc || Command == linebreakerclosec) ? 1 : 0;
}

// the impulse switches of another vehicle's line breaker spring back when let go, whichever way they were pressed
void release_remote_linebreaker(TTrain::remote_linebreaker &Controls, std::optional<sound_source> &Sound)
{
	if (Controls.on_button.SubModel != nullptr)
	{
		Controls.on_button.UpdateValue(0.0, Sound);
	}
	if (Controls.off_button.SubModel != nullptr)
	{
		Controls.off_button.UpdateValue(0.0, Sound);
	}
	if (Controls.main_switch.SubModel != nullptr && Controls.main_switch.type() != TGaugeType::toggle)
	{
		Controls.main_switch.UpdateValue(0.5, Sound);
	}
	Controls.held_without_switch = false;
}

} // namespace

TMoverParameters *TTrain::remote_engine(std::size_t const Index) const
{
	return engine_start::engine(*mvOccupied, *mvControlled, static_cast<int>(Index) + 2);
}

void TTrain::OnCommand_linebreakertoggleremote(TTrain *Train, command_data const &Command)
{
	auto const index{remote_linebreaker_index(Command.command)};
	auto &controls{Train->m_remotelinebreakers[index]};

	if (Command.action == GLFW_PRESS)
	{
		// a two-state switch goes the other way than it stands, an impulse one the other way than the engine is
		bool close{false};
		if (controls.main_switch.SubModel != nullptr && controls.main_switch.type() == TGaugeType::toggle)
		{
			close = controls.main_switch.GetDesiredValue() < 0.5;
		}
		else
		{
			auto const *engine{Train->remote_engine(index)};
			close = engine == nullptr || false == engine_start::remote_switch::is_running(*engine);
		}
		if (close)
		{
			OnCommand_linebreakercloseremote(Train, Command);
		}
		else
		{
			OnCommand_linebreakeropenremote(Train, Command);
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{
		if (controls.on_button.SubModel != nullptr || controls.main_switch.type() != TGaugeType::toggle)
		{
			// only impulse switches react to release events
			release_remote_linebreaker(controls, Train->dsbSwitch);
		}
		controls.held_without_switch = false;
	}
}

void TTrain::OnCommand_linebreakeropenremote(TTrain *Train, command_data const &Command)
{
	auto const index{remote_linebreaker_index(Command.command)};
	auto &controls{Train->m_remotelinebreakers[index]};

	if (Command.action == GLFW_PRESS)
	{
		// visual feedback
		if (controls.off_button.SubModel != nullptr)
		{
			controls.off_button.UpdateValue(1.0, Train->dsbSwitch);
		}
		else if (controls.main_switch.SubModel != nullptr)
		{
			controls.main_switch.UpdateValue(0.0, Train->dsbSwitch);
		}
		controls.held_without_switch = false;
		if (auto *engine{Train->remote_engine(index)}; engine != nullptr)
		{
			engine_start::remote_switch::open(*engine);
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{
		release_remote_linebreaker(controls, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_linebreakercloseremote(TTrain *Train, command_data const &Command)
{
	auto const index{remote_linebreaker_index(Command.command)};
	auto &controls{Train->m_remotelinebreakers[index]};

	if (Command.action == GLFW_PRESS)
	{
		// visual feedback; the line breaker closes in the train update, once the switch was held long enough
		if (controls.on_button.SubModel != nullptr)
		{
			controls.on_button.UpdateValue(1.0, Train->dsbSwitch);
		}
		else if (controls.main_switch.SubModel != nullptr)
		{
			controls.main_switch.UpdateValue(1.0, Train->dsbSwitch);
		}
		else
		{
			// no switch in the cab, the command alone holds it
			controls.held_without_switch = true;
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{
		release_remote_linebreaker(controls, Train->dsbSwitch);
	}
}

void TTrain::update_remote_linebreakers(double const Deltatime)
{
	for (std::size_t index = 0; index < m_remotelinebreakers.size(); ++index)
	{
		auto &controls{m_remotelinebreakers[index]};
		auto *engine{remote_engine(index)};
		auto const closeheld{controls.held_without_switch || (controls.main_switch.SubModel != nullptr && controls.main_switch.GetDesiredValue() > 0.95) ||
		                     (controls.on_button.SubModel != nullptr && controls.on_button.GetDesiredValue() > 0.95)};
		auto const openheld{controls.off_button.SubModel != nullptr && controls.off_button.GetDesiredValue() > 0.95};
		// as with the cab's own switch, an electric locomotive's impulse switch closes the line breaker when let go
		auto const closesonrelease{engine != nullptr && controls.on_button.SubModel != nullptr && engine->EngineType == TEngineType::ElectricSeriesMotor};
		controls.state.update(engine, closeheld, openheld, Deltatime, closesonrelease);
	}
}

void TTrain::OnCommand_motorconnectorsopen(TTrain *Train, command_data const &Command)
{

	// TODO: don't rely on presense of 3d model to determine presence of the switch
	if (Train->ggStLinOffButton.SubModel == nullptr)
	{
		if (Command.action == GLFW_PRESS)
		{
			WriteLog("Open Motor Power Connectors button is missing, or wasn't defined");
		}
		return;
	}
	// HACK: because we don't have modeled actual circuits this is a simplification of the real mechanics
	// namely, pressing the button will flip it in the entire unit, which isn't exactly physically possible
	if (Command.action == GLFW_PRESS)
	{
		// button works while it's held down but we can ignore repeats
		if (false == Train->mvControlled->StLinSwitchOff)
		{
			// open the connectors
			// visual feedback
			Train->ggStLinOffButton.UpdateValue(1.0, Train->dsbSwitch);

			Train->mvControlled->StLinSwitchOff = true;
			Train->set_paired_open_motor_connectors_button(true);
		}
		else
		{
			// potentially close the connectors
			OnCommand_motorconnectorsclose(Train, Command);
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// button released
		if (Train->mvControlled->StLinSwitchType != "toggle")
		{
			// default button type (impulse) ceases its work on button release
			// visual feedback
			Train->ggStLinOffButton.UpdateValue(0.0, Train->dsbSwitch);

			Train->mvControlled->StLinSwitchOff = false;
			Train->set_paired_open_motor_connectors_button(false);
		}
	}
}

void TTrain::OnCommand_motorconnectorsclose(TTrain *Train, command_data const &Command)
{

	// TODO: don't rely on presense of 3d model to determine presence of the switch
	if (Train->ggStLinOffButton.SubModel == nullptr)
	{
		if (Command.action == GLFW_PRESS)
		{
			WriteLog("Open Motor Power Connectors button is missing, or wasn't defined");
		}
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		if (Train->mvControlled->StLinSwitchType == "toggle")
		{
			// default type of button (impulse) has only one effect on press, but the toggle type can toggle the state
			// visual feedback
			Train->ggStLinOffButton.UpdateValue(0.0, Train->dsbSwitch);
		}

		if (false == Train->mvControlled->StLinSwitchOff)
		{
			return;
		} // already closed

		Train->mvControlled->StLinSwitchOff = false;
		Train->set_paired_open_motor_connectors_button(false);
	}
}

void TTrain::OnCommand_motordisconnect(TTrain *Train, command_data const &Command)
{

	if (Train->mvControlled->TrainType == train_type::emu ? Train->mvControlled != Train->mvOccupied : Train->iCabn != 0)
	{
		// tylko w maszynowym
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		Train->mvControlled->CutOffEngine();
	}
}

void TTrain::OnCommand_motoroverloadrelaythresholdtoggle(TTrain *Train, command_data const &Command)
{
	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		if (true == Train->mvControlled->ShuntModeAllow ? false == Train->mvControlled->ShuntMode : false == Train->mvControlled->MotorOverloadRelayHighThreshold)
		{
			// turn on
			OnCommand_motoroverloadrelaythresholdsethigh(Train, Command);
		}
		else
		{
			// turn off
			OnCommand_motoroverloadrelaythresholdsetlow(Train, Command);
		}
	}
}

void TTrain::OnCommand_motoroverloadrelaythresholdsetlow(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		Train->mvControlled->CurrentSwitch(false);
		// visual feedback
		Train->ggMaxCurrentCtrl.UpdateValue(Train->mvControlled->MotorOverloadRelayHighThreshold ? 1 : 0, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_motoroverloadrelaythresholdsethigh(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		Train->mvControlled->CurrentSwitch(true);
		// visual feedback
		Train->ggMaxCurrentCtrl.UpdateValue(Train->mvControlled->MotorOverloadRelayHighThreshold ? 1 : 0, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_motoroverloadrelayreset(TTrain *Train, command_data const &Command)
{

	if (Train->ggFuseButton.SubModel == nullptr)
	{
		if (Command.action == GLFW_PRESS)
		{
			WriteLog("Motor Overload Relay Reset button is missing, or wasn't defined");
		}
		//        return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// visual feedback
		Train->ggFuseButton.UpdateValue(1.0, Train->dsbSwitch);

		Train->mvControlled->FuseOn();
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// visual feedback
		Train->ggFuseButton.UpdateValue(0.0, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_universalrelayreset(TTrain *Train, command_data const &Command)
{

	auto const itemindex = static_cast<int>(Command.command) - static_cast<int>(user_command::universalrelayreset1);
	auto &item = Train->ggRelayResetButtons[itemindex];

	// NOTE: relay reset switches are impulse-only
	if (Command.action == GLFW_PRESS)
	{
		Train->mvOccupied->UniversalResetButton(itemindex);
		// visual feedback
		item.UpdateValue(1.0);
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// visual feedback
		item.UpdateValue(0.0);
	}
}
