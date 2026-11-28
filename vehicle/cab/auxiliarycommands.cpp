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

// the commands of the auxiliaries: the pumps, the genset, the water, the converter, the compressor, the blowers

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

void TTrain::OnCommand_oilfuelpumptoggle(TTrain *Train, command_data const &Command)
{
	if (Command.action == GLFW_REPEAT)
	{
		return;
	}

	auto *vehicle{Train->mvControlled};
	if (Train->ggOilFuelPumpButton.type() == TGaugeType::push)
	{
		// impulse switch, as the separate ones: held to start both pumps
		auto const pressed{Command.action == GLFW_PRESS};
		Train->ggOilFuelPumpButton.UpdateValue(pressed ? 1.0 : 0.0, Train->dsbSwitch);
		vehicle->FuelPumpSwitch(pressed);
		vehicle->OilPumpSwitch(pressed);
		return;
	}
	// two-state switch, only cares about press events
	if (Command.action != GLFW_PRESS)
	{
		return;
	}
	// both pumps go off if both were on, otherwise both go on; a pump started automatically doesn't take part
	auto const enable{false == vehicle->OilFuelPumpsEnabled()};
	Train->ggOilFuelPumpButton.UpdateValue(enable ? 1.0 : 0.0, Train->dsbSwitch);
	vehicle->FuelPumpSwitch(enable);
	vehicle->FuelPumpSwitchOff(false == enable);
	vehicle->OilPumpSwitch(enable);
	vehicle->OilPumpSwitchOff(false == enable);
}

// the traction selector of a dual-mode vehicle (eu07/maszyna#536), a two-state switch
void TTrain::OnCommand_tractionmodetoggle(TTrain *Train, command_data const &Command)
{
	if (Command.action != GLFW_PRESS)
	{
		return;
	}
	if (Train->mvControlled->GensetSelected())
	{
		OnCommand_tractionmodecatenary(Train, Command);
	}
	else
	{
		OnCommand_tractionmodegenset(Train, Command);
	}
}

void TTrain::OnCommand_tractionmodecatenary(TTrain *Train, command_data const &Command)
{
	if (Command.action != GLFW_PRESS)
	{
		return;
	}
	// refused with the line breaker closed: then the switch doesn't move
	Train->mvControlled->TractionModeSwitch(TMoverParameters::traction_mode::catenary, range_t::consist);
	if (false == Train->mvControlled->GensetSelected())
	{
		Train->ggTractionModeSwitch.UpdateValue(0.0, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_tractionmodegenset(TTrain *Train, command_data const &Command)
{
	if (Command.action != GLFW_PRESS)
	{
		return;
	}
	// refused with the line breaker closed, or without a genset: then the switch doesn't move
	Train->mvControlled->TractionModeSwitch(TMoverParameters::traction_mode::genset, range_t::consist);
	if (Train->mvControlled->GensetSelected())
	{
		Train->ggTractionModeSwitch.UpdateValue(1.0, Train->dsbSwitch);
	}
}

// the start request of the genset of a dual-mode vehicle, a two-state switch
void TTrain::OnCommand_gensettoggle(TTrain *Train, command_data const &Command)
{
	if (Command.action != GLFW_PRESS)
	{
		return;
	}
	if (false == Train->mvControlled->Genset.is_enabled)
	{
		OnCommand_gensetenable(Train, Command);
	}
	else
	{
		OnCommand_gensetdisable(Train, Command);
	}
}

void TTrain::OnCommand_gensetenable(TTrain *Train, command_data const &Command)
{
	if (Command.action != GLFW_PRESS)
	{
		return;
	}
	// ignored by a genset which starts on its own: then the switch doesn't move
	Train->mvControlled->GensetSwitch(true, range_t::consist);
	if (Train->mvControlled->Genset.is_enabled)
	{
		Train->ggGensetButton.UpdateValue(1.0, Train->dsbSwitch);
	}
}

void TTrain::OnCommand_gensetdisable(TTrain *Train, command_data const &Command)
{
	if (Command.action != GLFW_PRESS)
	{
		return;
	}
	Train->mvControlled->GensetSwitch(false, range_t::consist);
	if (false == Train->mvControlled->Genset.is_enabled)
	{
		Train->ggGensetButton.UpdateValue(0.0, Train->dsbSwitch);
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
	else if (Command.action == GLFW_RELEASE)
	{
		// on button release...
		if (Train->mvOccupied->ConvSwitchType == "impulse")
		{
			// ...return switches to start position if applicable
			Train->ggConverterButton.UpdateValue(0.0, Train->dsbSwitch);
			Train->ggConverterOffButton.UpdateValue(0.0, Train->dsbSwitch);
		}
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
			                // side effects
			                // control the compressor, if it's paired with the converter
			                if( Train->mvControlled->CompressorPower == 2 ) {
			                    // hunter-091012: tak jest poprawnie
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
			                // side effects
			                // control the compressor, if it's paired with the converter
			                if( Train->mvControlled->CompressorPower == 2 ) {
			                    // hunter-091012: tak jest poprawnie
			                    Train->mvControlled->CompressorSwitch( false, range::local );
			                }
			                // if there's no (low voltage) power source left, drop pantographs
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

	if (Train->ggConverterFuseButton.SubModel == nullptr)
	{
		if (Command.action == GLFW_PRESS)
		{
			WriteLog("Converter Overload Relay Reset button is missing, or wasn't defined");
		}
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
	    // disabled because we don't have yet support for compressor switch type definition
	    else if( Command.action == GLFW_RELEASE ) {
	        // on button release...
	        if( Train->mvOccupied->CompSwitchType == "impulse" ) {
	            // ...return switches to start position if applicable
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
		Train->ggCompressorListButton.UpdateValue(static_cast<float>(Train->mvOccupied->CompressorListPos - 1), Train->dsbSwitch);
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
			Train->ggCompressorListButton.UpdateValue(static_cast<float>(Train->mvOccupied->CompressorListPos - 1), Train->dsbSwitch);
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
			Train->ggCompressorListButton.UpdateValue(static_cast<float>(Train->mvOccupied->CompressorListPos - 1), Train->dsbSwitch);
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
		Train->ggCompressorListButton.UpdateValue(static_cast<float>(Train->mvOccupied->CompressorListPos - 1), Train->dsbSwitch);
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
