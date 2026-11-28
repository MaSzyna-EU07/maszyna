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

// the commands of the pantographs and their compressor

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
	else if (Command.action == GLFW_RELEASE)
	{
		// impulse switches return automatically to neutral position
		if (Train->mvOccupied->PantSwitchType == "impulse")
		{
			auto const ismanual{Train->iCabn == 0};
			Train->mvOccupied->OperatePantographValve(end::front, operation_t::none, ismanual ? range_t::local : range_t::consist);
		}
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
	else if (Command.action == GLFW_RELEASE)
	{
		// impulse switches return automatically to neutral position
		if (Train->mvOccupied->PantSwitchType == "impulse")
		{
			auto const ismanual{Train->iCabn == 0};
			Train->mvOccupied->OperatePantographValve(end::rear, operation_t::none, ismanual ? range_t::local : range_t::consist);
		}
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
	using enum operation_t;

	auto const &presets{mvOccupied->PantsPreset.first};
	auto const &selection{mvOccupied->PantsPreset.second[cab_to_end()]};

	auto const preset{presets[selection] - '0'};
	auto const swapends{cab_to_end() != end::front};
	// check desired states for both pantographs; value: whether the pantograph should be raised
	auto const frontstate{preset & (swapends ? 2 : 1)};
	auto const rearstate{preset & (swapends ? 1 : 2)};
	mvOccupied->OperatePantographValve(end::front, frontstate ? enable : disable);
	mvOccupied->OperatePantographValve(end::rear, rearstate ? enable : disable);
}

void TTrain::change_pantograph_selection(int const Change)
{

	auto const &presets{mvOccupied->PantsPreset.first};
	auto &selection{mvOccupied->PantsPreset.second[cab_to_end()]};
	auto const initialstate{selection};
	selection = std::clamp(selection + Change, 0, std::max<int>(static_cast<int>(presets.size() - 1), 0));

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


	if (auto const valveispresent{Train->ggPantCompressorValve.SubModel != nullptr || (Train->mvOccupied == Train->mvPantographUnit && Train->iCabn == 0)}; false == valveispresent)
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


	if (auto const valveispresent{Train->ggPantCompressorValve.SubModel != nullptr || (Train->mvOccupied == Train->mvPantographUnit && Train->iCabn == 0)}; false == valveispresent)
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
	if (auto const switchispresent{Train->m_controlmapper.contains("pantcompressor_sw:") || (Train->mvOccupied == Train->mvPantographUnit && Train->iCabn == 0)}; false == switchispresent)
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
