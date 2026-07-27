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

void TTrain::OnCommand_batterytoggle(TTrain *Train, command_data const &Command)
{
	if (Train->allowBatteryToggle || Command.action != GLFW_REPEAT)
	{
		// keep the switch from flipping back and forth if key is held down
		if (false == Train->mvOccupied->Power24vIsAvailable)
		{
			// turn on
			OnCommand_batteryenable(Train, Command);
		}
		else
		{
			// turn off
			OnCommand_batterydisable(Train, Command);
		}
	}
}

void TTrain::OnCommand_batteryenable(TTrain *Train, command_data const &Command)
{
	if (!Train->mvOccupied->isBatteryButtonImpulse)
	{ // regular button behavior
		if (Command.action == GLFW_PRESS)
		{
			// visual feedback
			Train->ggBatteryButton.UpdateValue(1.0f, Train->dsbSwitch);
			Train->ggBatteryOnButton.UpdateValue(1.0f, Train->dsbSwitch);

			Train->mvOccupied->BatterySwitch(true);
			Train->allowBatteryToggle = false;
			// side-effects
			if (Train->mvOccupied->LightsPosNo > 0)
			{
				Train->Dynamic()->SetLights();
			}
		}
		else if (Command.action == GLFW_RELEASE)
		{
			if (Train->ggBatteryButton.type() == TGaugeType::push)
			{
				// return the switch to neutral position
				Train->ggBatteryButton.UpdateValue(0.5f);
			}
			Train->ggBatteryOnButton.UpdateValue(0.0f, Train->dsbSwitch);
			Train->allowBatteryToggle = true;
		}
	}
	else // impulse button behavior
	{
		if (Command.action == GLFW_PRESS)
		{
			if (Train->mvOccupied->shouldHoldBatteryButton)
			{
				// jesli przycisk trzeba przytrzymac
				Train->ggBatteryButton.UpdateValue(1.0f, Train->dsbSwitch);
				Train->ggBatteryOnButton.UpdateValue(1.0f, Train->dsbSwitch);
				Train->fBatteryTimer = Train->mvOccupied->BatteryButtonHoldTime; // start timer
			}
			else
			{
				// jesli przycisk dziala od razu
				Train->mvOccupied->BatterySwitch(true);
				Train->allowBatteryToggle = false;
				// side-effects
				if (Train->mvOccupied->LightsPosNo > 0)
				{
					Train->Dynamic()->SetLights();
				}

				// visual feedback
				Train->ggBatteryButton.UpdateValue(1.0f, Train->dsbSwitch);
				Train->ggBatteryOnButton.UpdateValue(1.0f, Train->dsbSwitch);
			}
		}
		else if (Command.action == GLFW_RELEASE)
		{
			// visual feedback
			Train->ggBatteryButton.UpdateValue(0.0f, Train->dsbSwitch);
			Train->ggBatteryOnButton.UpdateValue(0.0f, Train->dsbSwitch);
			Train->fBatteryTimer = -1.f; //
			Train->allowBatteryToggle = true;
			Train->mvOccupied->batterySwAlreadyFired = false;
		}
		else if (Command.action == GLFW_REPEAT && Train->mvOccupied->shouldHoldBatteryButton && Train->fBatteryTimer <= 0.0 && Train->mvOccupied->Battery == false &&
		         !Train->mvOccupied->batterySwAlreadyFired)
		{
			// trzymamy przycisk
			Train->mvOccupied->BatterySwitch(true);
			Train->mvOccupied->batterySwAlreadyFired = true;

			// side-effects
			if (Train->mvOccupied->LightsPosNo > 0)
			{
				Train->Dynamic()->SetLights();
			}
			Train->allowBatteryToggle = false;
		}
	}
}

void TTrain::OnCommand_batterydisable(TTrain *Train, command_data const &Command)
{
	if (!Train->mvOccupied->isBatteryButtonImpulse)
	{ // regular button behavior
		if (Command.action == GLFW_PRESS)
		{
			// visual feedback
			Train->ggBatteryButton.UpdateValue(0.0f, Train->dsbSwitch);
			Train->ggBatteryOffButton.UpdateValue(1.0f, Train->dsbSwitch);

			Train->mvOccupied->BatterySwitch(false);

			// side-effects
			if (Train->mvOccupied->LightsPosNo > 0)
			{
				Train->Dynamic()->SetLights();
			}
		}
		else if (Command.action == GLFW_RELEASE)
		{
			if (Train->ggBatteryButton.type() == TGaugeType::push)
			{
				// return the switch to neutral position
				Train->ggBatteryButton.UpdateValue(0.5f);
			}
			Train->ggBatteryOffButton.UpdateValue(0.0f, Train->dsbSwitch);
		}
	}
	else // impulse button behavior
	{
		if (Command.action == GLFW_PRESS)
		{
			if (Train->mvOccupied->shouldHoldBatteryButton)
			{
				// jesli przycisk trzeba przytrzymac
				Train->ggBatteryButton.UpdateValue(1.0f, Train->dsbSwitch);
				Train->ggBatteryOffButton.UpdateValue(1.0f, Train->dsbSwitch);
				Train->fBatteryTimer = Train->mvOccupied->BatteryButtonHoldTime; // start timer
			}
			else
			{
				// jesli przycisk dziala od razu
				Train->mvOccupied->BatterySwitch(false);
				Train->allowBatteryToggle = false;

				// side-effects
				if (Train->mvOccupied->LightsPosNo > 0)
				{
					Train->Dynamic()->SetLights();
				}
				// visual feedback
				Train->ggBatteryButton.UpdateValue(1.0f, Train->dsbSwitch);
				Train->ggBatteryOffButton.UpdateValue(1.0f, Train->dsbSwitch);
			}
		}
		else if (Command.action == GLFW_RELEASE)
		{
			// visual feedback
			Train->ggBatteryButton.UpdateValue(0.0f, Train->dsbSwitch);
			Train->ggBatteryOffButton.UpdateValue(0.0f, Train->dsbSwitch);
			Train->allowBatteryToggle = true;
			Train->mvOccupied->batterySwAlreadyFired = false;
		}
		else if (Command.action == GLFW_REPEAT && Train->mvOccupied->shouldHoldBatteryButton && Train->fBatteryTimer <= 0.0 && Train->mvOccupied->Battery == true &&
		         !Train->mvOccupied->batterySwAlreadyFired)
		{
			// trzymamy przycisk
			Train->mvOccupied->BatterySwitch(false);
			Train->mvOccupied->batterySwAlreadyFired = true;

			// side-effects
			if (Train->mvOccupied->LightsPosNo > 0)
			{
				Train->Dynamic()->SetLights();
			}
			Train->allowBatteryToggle = false;
		}
	}
}

void TTrain::OnCommand_cabactivationtoggle(TTrain *Train, command_data const &Command)
{

	if (Command.action != GLFW_REPEAT)
	{
		// keep the switch from flipping back and forth if key is held down
		if (0 == Train->mvOccupied->CabActive)
		{
			// turn on
			OnCommand_cabactivationenable(Train, Command);
		}
		else
		{
			// turn off
			OnCommand_cabactivationdisable(Train, Command);
		}
	}
}

void TTrain::OnCommand_cabactivationenable(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// visual feedback
		if (Train->ggCabActivationButton.type() == TGaugeType::push)
		{
			Train->ggCabActivationButton.UpdateValue(1.0f, Train->dsbSwitch);
		}

		Train->mvOccupied->CabActivisation();

		// side-effects
		if (Train->mvOccupied->LightsPosNo > 0)
		{
			Train->Dynamic()->SetLights();
		}
	}
	else if (Command.action == GLFW_RELEASE && Train->ggCabActivationButton.type() == TGaugeType::push)
	{
		// return the switch to neutral position
		Train->ggCabActivationButton.UpdateValue(0.5f);
	}
}

void TTrain::OnCommand_cabactivationdisable(TTrain *Train, command_data const &Command)
{
	// TBD, TODO: ewentualnie zablokować z FIZ, np. w samochodach się nie odłącza akumulatora
	if (Command.action == GLFW_PRESS)
	{
		// visual feedback
		if (Train->ggCabActivationButton.type() == TGaugeType::push)
		{
			Train->ggCabActivationButton.UpdateValue(0.0f, Train->dsbSwitch);
		}

		Train->mvOccupied->CabDeactivisation();
		if (Train->mvOccupied->LightsPosNo > 0 && Train->mvOccupied->InactiveCabFlag & activation::redmarkers)
		{
			Train->Dynamic()->SetLights();
		}
	}
	else if (Command.action == GLFW_RELEASE && Train->ggCabActivationButton.type() == TGaugeType::push)
	{
		// return the switch to neutral position
		Train->ggCabActivationButton.UpdateValue(0.5f);
	}
}

void TTrain::OnCommand_pantographtogglefront(TTrain *Train, command_data const &Command)
{

	// HACK: presence of pantograph selector prevents manual operation of the individual valves
	if (Train->m_controlmapper.contains("pantselect_sw:"))
	{
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		auto const &pantograph{Train->mvPantographUnit->Pantographs[end::front]};
		auto const state{pantograph.valve.is_enabled || pantograph.is_active}; // fallback for impulse switches
		if (state)
		{
			OnCommand_pantographlowerfront(Train, Command);
		}
		else
		{
			OnCommand_pantographraisefront(Train, Command);
		}
	}
	else if (Command.action == GLFW_RELEASE && Train->mvOccupied->PantSwitchType == "impulse")
	{
		// impulse switches return automatically to neutral position
		auto const ismanual{Train->iCabn == 0};
		Train->mvOccupied->OperatePantographValve(end::front, operation_t::none, ismanual ? range_t::local : range_t::consist);
	}
}

void TTrain::OnCommand_pantographtogglerear(TTrain *Train, command_data const &Command)
{

	// HACK: presence of pantograph selector prevents manual operation of the individual valves
	if (Train->m_controlmapper.contains("pantselect_sw:"))
	{
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		auto const &pantograph{Train->mvPantographUnit->Pantographs[end::rear]};
		auto const state{pantograph.valve.is_enabled || pantograph.is_active}; // fallback for impulse switches
		if (state)
		{
			OnCommand_pantographlowerrear(Train, Command);
		}
		else
		{
			OnCommand_pantographraiserear(Train, Command);
		}
	}
	else if (Command.action == GLFW_RELEASE && Train->mvOccupied->PantSwitchType == "impulse")
	{
		// impulse switches return automatically to neutral position
		auto const ismanual{Train->iCabn == 0};
		Train->mvOccupied->OperatePantographValve(end::rear, operation_t::none, ismanual ? range_t::local : range_t::consist);
	}
}

void TTrain::OnCommand_pantographraisefront(TTrain *Train, command_data const &Command)
{

	// HACK: presence of pantograph selector prevents manual operation of the individual valves
	if (Train->m_controlmapper.contains("pantselect_sw:"))
	{
		return;
	}
	// prevent operation without submodel outside of engine compartment
	if (Train->iCabn != 0 && false == Train->m_controlmapper.contains("pantfront_sw:"))
	{
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		// HACK: don't propagate pantograph commands issued from engine compartment, these are presumed to be manually moved levers
		auto const ismanual{Train->iCabn == 0};
		Train->mvOccupied->OperatePantographValve(end::front, Train->mvOccupied->PantSwitchType == "impulse" ? operation_t::enable_on : operation_t::enable,
		                                          ismanual ? range_t::local : range_t::consist);
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// NOTE: bit of a hax here, we're reusing button reset routine so we don't need a copy in every branch
		OnCommand_pantographtogglefront(Train, Command);
	}
}

void TTrain::OnCommand_pantographraiserear(TTrain *Train, command_data const &Command)
{

	// HACK: presence of pantograph selector prevents manual operation of the individual valves
	if (Train->m_controlmapper.contains("pantselect_sw:"))
	{
		return;
	}
	// prevent operation without submodel outside of engine compartment
	if (Train->iCabn != 0 && false == Train->m_controlmapper.contains("pantrear_sw:"))
	{
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		// HACK: don't propagate pantograph commands issued from engine compartment, these are presumed to be manually moved levers
		auto const ismanual{Train->iCabn == 0};
		Train->mvOccupied->OperatePantographValve(end::rear, Train->mvOccupied->PantSwitchType == "impulse" ? operation_t::enable_on : operation_t::enable,
		                                          ismanual ? range_t::local : range_t::consist);
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// NOTE: bit of a hax here, we're reusing button reset routine so we don't need a copy in every branch
		OnCommand_pantographtogglerear(Train, Command);
	}
}

void TTrain::OnCommand_pantographlowerfront(TTrain *Train, command_data const &Command)
{

	// HACK: presence of pantograph selector prevents manual operation of the individual valves
	if (Train->m_controlmapper.contains("pantselect_sw:"))
	{
		return;
	}
	// prevent operation without submodel outside of engine compartment
	if (Train->iCabn != 0 && false == Train->m_controlmapper.contains(Train->mvOccupied->PantSwitchType == "impulse" ? "pantfrontoff_sw:" : "pantfront_sw:"))
	{
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		// HACK: don't propagate pantograph commands issued from engine compartment, these are presumed to be manually moved levers
		auto const ismanual{Train->iCabn == 0};
		Train->mvOccupied->OperatePantographValve(end::front, Train->mvOccupied->PantSwitchType == "impulse" ? operation_t::disable_on : operation_t::disable,
		                                          ismanual ? range_t::local : range_t::consist);
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// NOTE: bit of a hax here, we're reusing button reset routine so we don't need a copy in every branch
		OnCommand_pantographtogglefront(Train, Command);
	}
}

void TTrain::OnCommand_pantographlowerrear(TTrain *Train, command_data const &Command)
{

	// HACK: presence of pantograph selector prevents manual operation of the individual valves
	if (Train->m_controlmapper.contains("pantselect_sw:"))
	{
		return;
	}

	if (Train->iCabn != 0 && false == Train->m_controlmapper.contains(Train->mvOccupied->PantSwitchType == "impulse" ? "pantrearoff_sw:" : "pantrear_sw:"))
	{
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		// HACK: don't propagate pantograph commands issued from engine compartment, these are presumed to be manually moved levers
		auto const ismanual{Train->iCabn == 0};
		Train->mvOccupied->OperatePantographValve(end::rear, Train->mvOccupied->PantSwitchType == "impulse" ? operation_t::disable_on : operation_t::disable,
		                                          ismanual ? range_t::local : range_t::consist);
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// NOTE: bit of a hax here, we're reusing button reset routine so we don't need a copy in every branch
		OnCommand_pantographtogglerear(Train, Command);
	}
}

void TTrain::OnCommand_pantographlowerall(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_REPEAT)
	{
		return;
	}

	if (Train->ggPantAllDownButton.SubModel == nullptr)
	{
		// TODO: expand definition of cab controls so we can know if the control is present without testing for presence of 3d switch
		if (Command.action == GLFW_PRESS)
		{
			WriteLog("Lower All Pantographs switch is missing, or wasn't defined");
		}
		return;
	}

	if (Train->ggPantAllDownButton.type() == TGaugeType::toggle)
	{
		// two-state switch, only cares about press events
		if (Command.action == GLFW_PRESS)
		{
			Train->mvPantographUnit->DropAllPantographs(false == Train->mvPantographUnit->PantAllDown);
			// visual feedback
			Train->ggPantAllDownButton.UpdateValue(Train->mvPantographUnit->PantAllDown ? 1.0 : 0.0, Train->dsbSwitch);
		}
	}
	else
	{
		// impulse switch
		Train->mvControlled->DropAllPantographs(Command.action == GLFW_PRESS);
		// visual feedback
		Train->ggPantAllDownButton.UpdateValue(Command.action == GLFW_PRESS ? 1.0 : 0.0, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_pantographselectnext(TTrain *Train, command_data const &Command)
{

	if (Command.action != GLFW_PRESS)
	{
		return;
	}

	if (false == Train->m_controlmapper.contains("pantselect_sw:"))
	{
		return;
	}

	Train->change_pantograph_selection(1);
}

void TTrain::OnCommand_pantographselectprevious(TTrain *Train, command_data const &Command)
{

	if (Command.action != GLFW_PRESS)
	{
		return;
	}

	if (false == Train->m_controlmapper.contains("pantselect_sw:"))
	{
		return;
	}

	Train->change_pantograph_selection(-1);
}

void TTrain::OnCommand_pantographtoggleselected(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_REPEAT)
	{
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// recalculate pantograph state (hujhujhuj)
		Train->change_pantograph_selection(1);
		Train->change_pantograph_selection(-1);

		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		auto const state{Train->mvPantographUnit->PantsValve.is_enabled || Train->mvPantographUnit->PantsValve.is_active}; // fallback for impulse switches
		if (state)
		{
			OnCommand_pantographlowerselected(Train, Command);
		}
		else
		{
			OnCommand_pantographraiseselected(Train, Command);
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// impulse switches return automatically to neutral position
		if (Train->m_controlmapper.contains("pantselectedoff_sw:"))
		{
			// two buttons setup
			if (Train->ggPantSelectedButton.type() != TGaugeType::toggle)
			{
				Train->mvOccupied->OperatePantographsValve(operation_t::enable_off);
				// visual feedback
				Train->ggPantSelectedButton.UpdateValue(0.0, Train->dsbSwitch);
			}
			if (Train->ggPantSelectedDownButton.type() != TGaugeType::toggle)
			{
				Train->mvOccupied->OperatePantographsValve(operation_t::disable_off);
				// visual feedback
				Train->ggPantSelectedDownButton.UpdateValue(0.0, Train->dsbSwitch);
			}
		}
		else
		{
			if (Train->ggPantSelectedButton.type() != TGaugeType::toggle)
			{
				// special case, just one impulse switch controlling both states
				// with neutral position mid-way
				Train->mvOccupied->OperatePantographsValve(operation_t::none);
				// visual feedback
				Train->ggPantSelectedButton.UpdateValue(0.5, Train->dsbSwitch);
			}
		}
	}
}

void TTrain::OnCommand_pantographraiseselected(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_REPEAT)
	{
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// raise selected
		Train->mvOccupied->OperatePantographsValve(Train->ggPantSelectedButton.type() != TGaugeType::toggle ? operation_t::enable_on : operation_t::enable);
		// visual feedback
		Train->ggPantSelectedButton.UpdateValue(1.0, Train->dsbSwitch);
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// NOTE: bit of a hax here, we're reusing button reset routine so we don't need a copy in every branch
		OnCommand_pantographtoggleselected(Train, Command);
	}
}

void TTrain::OnCommand_pantographlowerselected(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_REPEAT)
	{
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// lower selected
		Train->mvOccupied->OperatePantographsValve(Train->ggPantSelectedDownButton.type() != TGaugeType::toggle ? operation_t::disable_on : operation_t::disable);
		// visual feedback
		if (Train->m_controlmapper.contains("pantselectedoff_sw:"))
		{
			// two button setup
			Train->ggPantSelectedDownButton.UpdateValue(1.0, Train->dsbSwitch);
		}
		else
		{
			// single button
			Train->ggPantSelectedButton.UpdateValue(0.0, Train->dsbSwitch);
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// NOTE: bit of a hax here, we're reusing button reset routine so we don't need a copy in every branch
		OnCommand_pantographtoggleselected(Train, Command);
	}
}

void TTrain::update_pantograph_valves()
{

	auto const &presets{mvOccupied->PantsPreset.first};
	auto &selection{mvOccupied->PantsPreset.second[cab_to_end()]};

	auto const preset{presets[selection] - '0'};
	auto const swapends{cab_to_end() != end::front};
	// check desired states for both pantographs; value: whether the pantograph should be raised
	auto const frontstate{preset & (swapends ? 2 : 1)};
	auto const rearstate{preset & (swapends ? 1 : 2)};
	mvOccupied->OperatePantographValve(end::front, frontstate ? operation_t::enable : operation_t::disable);
	mvOccupied->OperatePantographValve(end::rear, rearstate ? operation_t::enable : operation_t::disable);
}

void TTrain::change_pantograph_selection(int const Change)
{

	auto const &presets{mvOccupied->PantsPreset.first};
	auto &selection{mvOccupied->PantsPreset.second[cab_to_end()]};
	auto const initialstate{selection};
	selection = std::clamp(selection + Change, 0, std::max<int>(presets.size() - 1, 0));

	if (selection == initialstate)
	{
		return;
	} // no change, nothing to do

	// potentially adjust pantograph valves to match the new state
	if (false == m_controlmapper.contains("pantvalves_sw:"))
	{
		update_pantograph_valves();
	}
}

void TTrain::OnCommand_pantographvalvesupdate(TTrain *Train, command_data const &Command)
{

	bool hasSeparateSwitches = Train->m_controlmapper.contains("pantvalvesupdate_bt:") && Train->m_controlmapper.contains("pantvalvesoff_bt:");

	if (Command.action == GLFW_REPEAT)
	{
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		if (hasSeparateSwitches)
		{
			// implement action
			Train->update_pantograph_valves();
			// visual feedback
			Train->ggPantValvesUpdate.UpdateValue(1.0, Train->dsbSwitch);
		}

		// Old logic to maintain compatibility
		else
		{
			Train->update_pantograph_valves();
			Train->ggPantValvesButton.UpdateValue(1.0, Train->dsbSwitch);
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// visual feedback
		// NOTE: pantvalves_sw: is a specialized button, with no toggle behavior support
		if (hasSeparateSwitches)
			Train->ggPantValvesUpdate.UpdateValue(0.5, Train->dsbSwitch);

		// Old logic to maintain compatibility
		else
			Train->ggPantValvesButton.UpdateValue(0.5, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_pantographvalvesoff(TTrain *Train, command_data const &Command)
{

	bool hasSeparateSwitches = Train->m_controlmapper.contains("pantvalvesupdate_bt:") && Train->m_controlmapper.contains("pantvalvesoff_bt:");

	if (Command.action == GLFW_REPEAT)
	{
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// implement action
		Train->mvOccupied->OperatePantographValve(end::front, operation_t::disable);
		Train->mvOccupied->OperatePantographValve(end::rear, operation_t::disable);
		// visual feedback
		if (hasSeparateSwitches)
			Train->ggPantValvesOff.UpdateValue(1.0, Train->dsbSwitch);
		else
			Train->ggPantValvesButton.UpdateValue(0.0, Train->dsbSwitch);
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// visual feedback
		// NOTE: pantvalves_sw: is a speciali   zed button, with no toggle behavior support
		if (hasSeparateSwitches)
			Train->ggPantValvesOff.UpdateValue(0.f, Train->dsbSwitch);
		else
			Train->ggPantValvesButton.UpdateValue(0.5, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_pantographcompressorvalvetoggle(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// only react to press
		if (Train->mvControlled->bPantKurek3 == false)
		{
			// connect pantographs with primary tank
			OnCommand_pantographcompressorvalveenable(Train, Command);
		}
		else
		{
			// connect pantograps with pantograph compressor
			OnCommand_pantographcompressorvalvedisable(Train, Command);
		}
	}
}

void TTrain::OnCommand_pantographcompressorvalveenable(TTrain *Train, command_data const &Command)
{

	auto const valveispresent{Train->ggPantCompressorValve.SubModel != nullptr || (Train->mvOccupied == Train->mvPantographUnit && Train->iCabn == 0)};

	if (false == valveispresent)
	{
		// tylko w maszynowym, unless actual device is present
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// only react to press
		// connect pantographs with primary tank
		Train->mvControlled->bPantKurek3 = true;
		// visual feedback:
		Train->ggPantCompressorValve.UpdateValue(0.0);
	}
}

void TTrain::OnCommand_pantographcompressorvalvedisable(TTrain *Train, command_data const &Command)
{

	auto const valveispresent{Train->ggPantCompressorValve.SubModel != nullptr || (Train->mvOccupied == Train->mvPantographUnit && Train->iCabn == 0)};

	if (false == valveispresent)
	{
		// tylko w maszynowym, unless actual device is present
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// only react to press
		// connect pantograps with pantograph compressor
		Train->mvControlled->bPantKurek3 = false;
		// visual feedback:
		Train->ggPantCompressorValve.UpdateValue(1.0);
	}
}

void TTrain::OnCommand_pantographcompressoractivate(TTrain *Train, command_data const &Command)
{

	// tylko w maszynowym, unless actual device is present
	auto const switchispresent{Train->m_controlmapper.contains("pantcompressor_sw:") || (Train->mvOccupied == Train->mvPantographUnit && Train->iCabn == 0)};
	if (false == switchispresent)
	{
		return;
	}

	if (Command.action != GLFW_RELEASE)
	{
		// press or hold to activate
		if (Train->mvPantographUnit->PantPress < 4.8 && true == Train->mvPantographUnit->Power24vIsAvailable)
		{
			// needs live power source and low enough pressure to work
			Train->mvPantographUnit->PantCompFlag = true;
		}
		// visual feedback
		Train->ggPantCompressorButton.UpdateValue(1.0);
	}
	else
	{
		// release to disable
		Train->mvPantographUnit->PantCompFlag = false;
		// visual feedback
		Train->ggPantCompressorButton.UpdateValue(0.0);
	}
}

void TTrain::OnCommand_linebreakertoggle(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// press or hold...
		if (Train->m_linebreakerstate == 0)
		{
			// ...to close the circuit
			// NOTE: bit of a dirty shortcut here
			OnCommand_linebreakerclose(Train, Command);
		}
		else if (Train->m_linebreakerstate == 1)
		{
			// ...to open the circuit
			OnCommand_linebreakeropen(Train, Command);
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// release...
		if (Train->ggMainOnButton.SubModel != nullptr || Train->ggMainButton.type() != TGaugeType::toggle)
		{
			// only impulse switches react to release events
			// NOTE: we presume dedicated state switch is of impulse type
			if (Train->m_linebreakerstate == 0)
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
		if (Train->ggMainOffButton.SubModel != nullptr)
		{
			Train->ggMainOffButton.UpdateValue(1.0, Train->dsbSwitch);
		}
		else if (Train->ggMainButton.SubModel != nullptr)
		{
			Train->ggMainButton.UpdateValue(0.0, Train->dsbSwitch);
		}
		else if (Train->ggMainOnButton.SubModel != nullptr)
		{
			// NOTE: legacy behaviour, for vehicles equipped only with impulse close switch
			// it doesn't make any real sense to animate this one, but some people can't get over how there's no visual reaction to their keypress
			Train->ggMainOnButton.UpdateValue(1.0, Train->dsbSwitch);
			return;
		}
		// play sound immediately when the switch is hit, not after release
		Train->fMainRelayTimer = 0.0f;

		if (Train->m_linebreakerstate == 0)
		{
			return;
		} // already in the desired state

		if (true == Train->mvControlled->MainSwitch(false))
		{
			Train->m_linebreakerstate = 0;
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
		if (Train->ggMainOffButton.SubModel != nullptr)
		{
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

		if (Train->m_linebreakerstate == 1)
		{
			return;
		} // already in the desired state

		if (Train->m_linebreakerstate == 2 && Train->mvControlled->EngineType == TEngineType::ElectricSeriesMotor)
		{
			// we don't need to start the diesel twice, but the other types (with impulse switch setup) still need to be launched
			// NOTE: this behaviour should depend on MainOnButton presence and type_delayed
			// TODO: change it when/if vehicle definition files get their proper switch types
			// try to finalize state change of the line breaker, set the state based on the outcome
			Train->m_linebreakerstate = Train->mvControlled->MainSwitch(true) ? 1 : 0;
		}
		// on button release reset the closing timer
		Train->fMainRelayTimer = 0.0f;
	}
}

void TTrain::OnCommand_convertertoggle(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		auto const overloadrelayisopen{(Train->Dynamic()->Mechanik != nullptr ? Train->Dynamic()->Mechanik->IsAnyConverterOverloadRelayOpen : Train->mvOccupied->ConvOvldFlag)};

		if (Train->mvOccupied->ConvSwitchType != "impulse" ? Train->ggConverterButton.GetValue() < 0.5 : false == Train->mvOccupied->Power110vIsAvailable && false == overloadrelayisopen)
		{
			// turn on
			OnCommand_converterenable(Train, Command);
		}
		else
		{
			// turn off
			OnCommand_converterdisable(Train, Command);
		}
	}
	else if (Command.action == GLFW_RELEASE && Train->mvOccupied->ConvSwitchType == "impulse")
	{
		// on button release...
		// ...return switches to start position if applicable
		Train->ggConverterButton.UpdateValue(0.0, Train->dsbSwitch);
		Train->ggConverterOffButton.UpdateValue(0.0, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_converterenable(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// visual feedback
		Train->ggConverterButton.UpdateValue(1.0, Train->dsbSwitch);

		// impulse type switch has no effect if there's no power
		// NOTE: this is most likely setup wrong, but the whole thing is smoke and mirrors anyway
		if (Train->mvOccupied->ConvSwitchType != "impulse" || Train->mvControlled->Mains)
		{
			// won't start if the line breaker button is still held
			Train->mvOccupied->ConverterSwitch(true);
		}
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// potentially reset impulse switch position, using shared code branch
		OnCommand_convertertoggle(Train, Command);
	}
}

void TTrain::OnCommand_converterdisable(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// visual feedback
		Train->ggConverterButton.UpdateValue(0.0, Train->dsbSwitch);
		if (Train->ggConverterOffButton.SubModel != nullptr)
		{
			Train->ggConverterOffButton.UpdateValue(1.0, Train->dsbSwitch);
		}

		Train->mvOccupied->ConverterSwitch(false);
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// potentially reset impulse switch position, using shared code branch
		OnCommand_convertertoggle(Train, Command);
	}
}

void TTrain::OnCommand_convertertogglelocal(TTrain *Train, command_data const &Command)
{

	if (Train->mvOccupied->ConverterStart == start_t::automatic)
	{
		// let the automatic thing do its automatic thing...
		return;
	}
	if (Train->ggConverterLocalButton.SubModel == nullptr)
	{
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		if (false == Train->mvOccupied->ConverterAllowLocal && Train->ggConverterLocalButton.GetValue() < 0.5)
		{
			// turn on
			// visual feedback
			Train->ggConverterLocalButton.UpdateValue(1.0, Train->dsbSwitch);
			// effect
			Train->mvOccupied->ConverterAllowLocal = true;
			/*
			            if( true == Train->mvControlled->ConverterSwitch( true, range::local ) ) {
			                side effects
			                control the compressor, if it's paired with the converter
			                if( Train->mvControlled->CompressorPower == 2 ) {
			                    hunter-091012: tak jest poprawnie
			                    Train->mvControlled->CompressorSwitch( true, range::local );
			                }
			            }
			*/
		}
		else
		{
			// turn off
			//  visual feedback
			Train->ggConverterLocalButton.UpdateValue(0.0, Train->dsbSwitch);
			// effect
			Train->mvOccupied->ConverterAllowLocal = false;
			/*
			            if( true == Train->mvControlled->ConverterSwitch( false, range::local ) ) {
			                side effects
			                control the compressor, if it's paired with the converter
			                if( Train->mvControlled->CompressorPower == 2 ) {
			                    hunter-091012: tak jest poprawnie
			                    Train->mvControlled->CompressorSwitch( false, range::local );
			                }
			                if there's no (low voltage) power source left, drop pantographs
			                if( false == Train->mvControlled->Battery ) {
			                    Train->mvControlled->PantFront( false, range::local );
			                    Train->mvControlled->PantRear( false, range::local );
			                }
			            }
			*/
		}
	}
}

void TTrain::OnCommand_converteroverloadrelayreset(TTrain *Train, command_data const &Command)
{

	if (Train->ggConverterFuseButton.SubModel == nullptr && Command.action == GLFW_PRESS)
	{
		WriteLog("Converter Overload Relay Reset button is missing, or wasn't defined");
		//        return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// visual feedback
		Train->ggConverterFuseButton.UpdateValue(1.0, Train->dsbSwitch);

		Train->mvControlled->RelayReset(relay_t::primaryconverteroverload);
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// visual feedback
		Train->ggConverterFuseButton.UpdateValue(0.0, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_compressortoggle(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		auto const compressorisenabled{(Train->Dynamic()->Mechanik ? Train->Dynamic()->Mechanik->IsAnyCompressorEnabled : Train->mvOccupied->CompressorAllow)};

		if (false == compressorisenabled)
		{
			// turn on
			OnCommand_compressorenable(Train, Command);
		}
		else
		{
			// turn off
			OnCommand_compressordisable(Train, Command);
		}
	}
	/*
	    disabled because we don't have yet support for compressor switch type definition
	    else if( Command.action == GLFW_RELEASE ) {
	        on button release...
	        if( Train->mvOccupied->CompSwitchType == "impulse" ) {
	            ...return switches to start position if applicable
	            Train->ggCompressorButton.UpdateValue( 0.0, Train->dsbSwitch );
	            Train->ggCompressorOffButton.UpdateValue( 0.0, Train->dsbSwitch );
	        }
	    }
	*/
}

void TTrain::OnCommand_compressorenable(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_PRESS)
	{
		// visual feedback
		Train->ggCompressorButton.UpdateValue(1.0, Train->dsbSwitch);

		// impulse type switch has no effect if there's no power
		// NOTE: this is most likely setup wrong, but the whole thing is smoke and mirrors anyway
		//        if( ( Train->mvOccupied->CompSwitchType != "impulse" )
		//         || ( Train->mvControlled->Mains ) ) {

		Train->mvOccupied->CompressorSwitch(true);
		//        }
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// potentially reset impulse switch position, using shared code branch
		OnCommand_compressortoggle(Train, Command);
	}
}

void TTrain::OnCommand_compressordisable(TTrain *Train, command_data const &Command)
{

	if (Train->mvControlled->CompressorPower >= 2)
	{
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// visual feedback
		Train->ggCompressorButton.UpdateValue(0.0, Train->dsbSwitch);
		/*
		        if( Train->ggCompressorOffButton.SubModel != nullptr ) {
		            Train->ggCompressorOffButton.UpdateValue( 1.0, Train->dsbSwitch );
		        }
		*/
		Train->mvOccupied->CompressorSwitch(false);
	}
	else if (Command.action == GLFW_RELEASE)
	{
		// potentially reset impulse switch position, using shared code branch
		OnCommand_compressortoggle(Train, Command);
	}
}

void TTrain::OnCommand_compressortogglelocal(TTrain *Train, command_data const &Command)
{

	if (Train->mvOccupied->CompressorPower >= 2)
	{
		return;
	}
	if (Train->ggCompressorLocalButton.SubModel == nullptr)
	{
		return;
	}

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		if (false == Train->mvOccupied->CompressorAllowLocal)
		{
			// turn on
			// visual feedback
			Train->ggCompressorLocalButton.UpdateValue(1.0, Train->dsbSwitch);
			// effect
			Train->mvOccupied->CompressorAllowLocal = true;
		}
		else
		{
			// turn off
			// visual feedback
			Train->ggCompressorLocalButton.UpdateValue(0.0, Train->dsbSwitch);
			// effect
			Train->mvOccupied->CompressorAllowLocal = false;
		}
	}
}

void TTrain::OnCommand_compressorpresetactivatenext(TTrain *Train, command_data const &Command)
{

	if (Train->mvOccupied->CompressorListPosNo == 0)
	{
		return;
	}
	if (Command.action == GLFW_REPEAT)
	{
		return;
	}

	if (Train->ggCompressorListButton.type() == TGaugeType::push)
	{
		// impulse switch
		if (Train->mvOccupied->CompressorListPosNo < Train->mvOccupied->CompressorListDefPos + 1)
		{
			return;
		}

		Train->mvOccupied->ChangeCompressorPreset(Command.action == GLFW_PRESS ? Train->mvOccupied->CompressorListDefPos + 1 : Train->mvOccupied->CompressorListDefPos);
		// visual feedback
		Train->ggCompressorListButton.UpdateValue(Train->mvOccupied->CompressorListPos - 1, Train->dsbSwitch);
	}
	else
	{
		// multi-state switch
		if (Command.action == GLFW_RELEASE)
		{
			return;
		}

		if (Train->mvOccupied->CompressorListPos < Train->mvOccupied->CompressorListPosNo || true == Train->mvOccupied->CompressorListWrap)
		{
			// active light preset is stored as value in range 1-LigthPosNo
			Train->mvOccupied->ChangeCompressorPreset(Train->mvOccupied->CompressorListPos < Train->mvOccupied->CompressorListPosNo ? Train->mvOccupied->CompressorListPos + 1 : 1); // wrap mode
			// visual feedback
			Train->ggCompressorListButton.UpdateValue(Train->mvOccupied->CompressorListPos - 1, Train->dsbSwitch);
		}
	}
}

void TTrain::OnCommand_compressorpresetactivateprevious(TTrain *Train, command_data const &Command)
{

	if (Train->mvOccupied->CompressorListPosNo == 0)
	{
		return;
	}
	if (Command.action != GLFW_PRESS)
	{
		return;
	} // one change per key press

	if (Train->ggCompressorListButton.type() == TGaugeType::push)
	{
		// impulse switch toggles only between positions 'default' and 'default+1'
		return;
	}

	if (Train->mvOccupied->CompressorListPos > 1 || true == Train->mvOccupied->CompressorListWrap)
	{
		// active light preset is stored as value in range 1-LigthPosNo
		Train->mvOccupied->ChangeCompressorPreset(Train->mvOccupied->CompressorListPos > 1 ? Train->mvOccupied->CompressorListPos - 1 : Train->mvOccupied->CompressorListPosNo); // wrap mode

		// visual feedback
		if (Train->ggCompressorListButton.SubModel != nullptr)
		{
			Train->ggCompressorListButton.UpdateValue(Train->mvOccupied->CompressorListPos - 1, Train->dsbSwitch);
		}
	}
}

void TTrain::OnCommand_compressorpresetactivatedefault(TTrain *Train, command_data const &Command)
{

	if (Train->mvOccupied->CompressorListPosNo == 0)
	{
		return;
	}
	if (Command.action != GLFW_PRESS)
	{
		return;
	} // one change per key press

	Train->mvOccupied->ChangeCompressorPreset(Train->mvOccupied->CompressorListDefPos);

	// visual feedback
	if (Train->ggCompressorListButton.SubModel != nullptr)
	{
		Train->ggCompressorListButton.UpdateValue(Train->mvOccupied->CompressorListPos - 1, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_motorblowerstogglefront(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_REPEAT)
	{
		return;
	}

	if (Train->ggMotorBlowersFrontButton.type() == TGaugeType::push)
	{
		// impulse switch
		// currently there's no off button so we always try to turn it on
		OnCommand_motorblowersenablefront(Train, Command);
	}
	else
	{
		// two-state switch
		if (Command.action == GLFW_RELEASE)
		{
			return;
		}

		if (false == Train->mvControlled->MotorBlowers[end::front].is_enabled)
		{
			// turn on
			OnCommand_motorblowersenablefront(Train, Command);
		}
		else
		{
			// turn off
			OnCommand_motorblowersdisablefront(Train, Command);
		}
	}
}

void TTrain::OnCommand_motorblowersenablefront(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_REPEAT)
	{
		return;
	}

	if (Train->ggMotorBlowersFrontButton.type() == TGaugeType::push)
	{
		// impulse switch
		if (Command.action == GLFW_PRESS)
		{
			// visual feedback
			Train->ggMotorBlowersFrontButton.UpdateValue(1.f, Train->dsbSwitch);
			Train->mvControlled->MotorBlowersSwitch(true, end::front);
		}
		else if (Command.action == GLFW_RELEASE)
		{
			// visual feedback
			Train->ggMotorBlowersFrontButton.UpdateValue(0.f, Train->dsbSwitch);
			Train->mvControlled->MotorBlowersSwitch(false, end::front);
		}
	}
	else
	{
		// two-state switch, only cares about press events
		if (Command.action == GLFW_PRESS)
		{
			// visual feedback
			Train->ggMotorBlowersFrontButton.UpdateValue(1.f, Train->dsbSwitch);
			Train->mvControlled->MotorBlowersSwitch(true, end::front);
			Train->mvControlled->MotorBlowersSwitchOff(false, end::front);
		}
	}
}

void TTrain::OnCommand_motorblowersdisablefront(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_REPEAT)
	{
		return;
	}

	if (Train->ggMotorBlowersFrontButton.type() == TGaugeType::push)
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
			Train->ggMotorBlowersFrontButton.UpdateValue(0.f, Train->dsbSwitch);
			Train->mvControlled->MotorBlowersSwitch(false, end::front);
			Train->mvControlled->MotorBlowersSwitchOff(true, end::front);
		}
	}
}

void TTrain::OnCommand_motorblowerstogglerear(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_REPEAT)
	{
		return;
	}

	if (Train->ggMotorBlowersRearButton.type() == TGaugeType::push)
	{
		// impulse switch
		// currently there's no off button so we always try to turn it on
		OnCommand_motorblowersenablerear(Train, Command);
	}
	else
	{
		// two-state switch
		if (Command.action == GLFW_RELEASE)
		{
			return;
		}

		if (false == Train->mvControlled->MotorBlowers[end::rear].is_enabled)
		{
			// turn on
			OnCommand_motorblowersenablerear(Train, Command);
		}
		else
		{
			// turn off
			OnCommand_motorblowersdisablerear(Train, Command);
		}
	}
}

void TTrain::OnCommand_motorblowersenablerear(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_REPEAT)
	{
		return;
	}

	if (Train->ggMotorBlowersRearButton.type() == TGaugeType::push)
	{
		// impulse switch
		if (Command.action == GLFW_PRESS)
		{
			// visual feedback
			Train->ggMotorBlowersRearButton.UpdateValue(1.f, Train->dsbSwitch);
			Train->mvControlled->MotorBlowersSwitch(true, end::rear);
		}
		else if (Command.action == GLFW_RELEASE)
		{
			// visual feedback
			Train->ggMotorBlowersRearButton.UpdateValue(0.f, Train->dsbSwitch);
			Train->mvControlled->MotorBlowersSwitch(false, end::rear);
		}
	}
	else
	{
		// two-state switch, only cares about press events
		if (Command.action == GLFW_PRESS)
		{
			// visual feedback
			Train->ggMotorBlowersRearButton.UpdateValue(1.f, Train->dsbSwitch);
			Train->mvControlled->MotorBlowersSwitch(true, end::rear);
			Train->mvControlled->MotorBlowersSwitchOff(false, end::rear);
		}
	}
}

void TTrain::OnCommand_motorblowersdisablerear(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_REPEAT)
	{
		return;
	}

	if (Train->ggMotorBlowersRearButton.type() == TGaugeType::push)
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
			Train->ggMotorBlowersRearButton.UpdateValue(0.f, Train->dsbSwitch);
			Train->mvControlled->MotorBlowersSwitch(false, end::rear);
			Train->mvControlled->MotorBlowersSwitchOff(true, end::rear);
		}
	}
}

void TTrain::OnCommand_motorblowersdisableall(TTrain *Train, command_data const &Command)
{

	if (Command.action == GLFW_REPEAT)
	{
		return;
	}

	if (Train->ggMotorBlowersAllOffButton.type() == TGaugeType::push)
	{
		// impulse switch
		if (Command.action == GLFW_PRESS)
		{
			// visual feedback
			Train->ggMotorBlowersAllOffButton.UpdateValue(1.f, Train->dsbSwitch);
			Train->mvControlled->MotorBlowersSwitchOff(true, end::front);
			Train->mvControlled->MotorBlowersSwitchOff(true, end::rear);
		}
		else if (Command.action == GLFW_RELEASE)
		{
			// visual feedback
			Train->ggMotorBlowersAllOffButton.UpdateValue(0.f, Train->dsbSwitch);
			Train->mvControlled->MotorBlowersSwitchOff(false, end::front);
			Train->mvControlled->MotorBlowersSwitchOff(false, end::rear);
		}
	}
	else
	{
		// two-state switch, only cares about press events
		// NOTE: generally this switch doesn't come in two-state form
		if (Command.action == GLFW_PRESS)
		{
			if (Train->ggMotorBlowersAllOffButton.GetDesiredValue() < 0.5f)
			{
				// switch is off, activate
				Train->mvControlled->MotorBlowersSwitchOff(true, end::front);
				Train->mvControlled->MotorBlowersSwitchOff(true, end::rear);
				// visual feedback
				Train->ggMotorBlowersRearButton.UpdateValue(1.f, Train->dsbSwitch);
			}
			else
			{
				// deactivate
				Train->mvControlled->MotorBlowersSwitchOff(false, end::front);
				Train->mvControlled->MotorBlowersSwitchOff(false, end::rear);
				// visual feedback
				Train->ggMotorBlowersRearButton.UpdateValue(0.f, Train->dsbSwitch);
			}
		}
	}
}

void TTrain::OnCommand_coolingfanstoggle(TTrain *Train, command_data const &Command)
{

	if (Command.action != GLFW_PRESS)
	{
		return;
	}

	Train->mvControlled->RVentForceOn = !Train->mvControlled->RVentForceOn;
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
	else if (Command.action == GLFW_RELEASE && Train->mvControlled->StLinSwitchType != "toggle")
	{
		// button released
		// default button type (impulse) ceases its work on button release
		// visual feedback
		Train->ggStLinOffButton.UpdateValue(0.0, Train->dsbSwitch);

		Train->mvControlled->StLinSwitchOff = false;
		Train->set_paired_open_motor_connectors_button(false);
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

	if (Train->mvControlled->TrainType == dt_EZT ? Train->mvControlled != Train->mvOccupied : Train->iCabn != 0)
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

	if (Train->ggFuseButton.SubModel == nullptr && Command.action == GLFW_PRESS)
	{
		WriteLog("Motor Overload Relay Reset button is missing, or wasn't defined");
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

void TTrain::OnCommand_inverterenable(TTrain *Train, command_data const &Command)
{

	auto itemindex = static_cast<int>(Command.command) - static_cast<int>(user_command::inverterenable1);
	auto &item = Train->ggInverterEnableButtons[itemindex];

	if (Command.action == GLFW_PRESS)
	{
		// only reacting to press, so the switch doesn't flip back and forth if key is held down
		bool kier = Train->DynamicObject->DirectionGet() * Train->mvOccupied->CabOccupied > 0;
		int flag = Train->DynamicObject->MoverParameters->InverterControlCouplerFlag;
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
		int flag = Train->DynamicObject->MoverParameters->InverterControlCouplerFlag;
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
		int flag = Train->DynamicObject->MoverParameters->InverterControlCouplerFlag;
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
