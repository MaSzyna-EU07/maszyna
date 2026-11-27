/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the commands passed between the vehicles of a train

#include "stdafx.h"
#include "physics/mover.h"
#include "vehicle/DynObj.h"
#include "physics/oerlikonest.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/Timer.h"
#include "utilities/parser.h"
#include "simulation/simulation.h"
#include "vehicle/fieldshunting.h"
#include <algorithm>
#include <charconv>
#include <cmath>
#include <format>
#include <functional>
#include <limits>
#include <utility>
#include <vector>
#include "physics/components/internal.h"


// *************************************************************************************************
// Q: 20160714
// Wstawia komendę z parametrem, od sprzęgu i w lokalizacji do pojazdu
// *************************************************************************************************
void TMoverParameters::PutCommand(std::string const &NewCommand, double NewValue1, double NewValue2, const TLocation &NewLocation)
{
	auto const parsed{ParseVehicleCommand(NewCommand)};
	CommandIn.Command = parsed.command;
	CommandIn.Text = NewCommand;
	CommandIn.Argument = parsed.argument;
	CommandIn.Value1 = NewValue1;
	CommandIn.Value2 = NewValue2;
	CommandIn.Location = NewLocation;
	// czy uruchomic tu RunInternalCommand? nie wiem
}

// *************************************************************************************************
// Q: 20160714
// Ustawienie komendy wraz z parametrami
// *************************************************************************************************
bool TMoverParameters::SetInternalCommand(VehicleCommand const NewCommand, double NewValue1, double NewValue2, CouplingSet const Couplertype)
{
	bool SIC;
	if (CommandIn.Command == NewCommand && CommandIn.Argument.empty() && CommandIn.Value1 == NewValue1 && CommandIn.Value2 == NewValue2 && CommandIn.couplings == Couplertype)
		SIC = false;
	else
	{
		CommandIn.Command = NewCommand;
		CommandIn.Text = VehicleCommandName(NewCommand);
		CommandIn.Argument.clear();
		CommandIn.Value1 = NewValue1;
		CommandIn.Value2 = NewValue2;
		CommandIn.couplings = Couplertype;
		SIC = true;
		LastLoadChangeTime = 0; // zerowanie czasu (roz)ładowania
	}

	return SIC;
}

// *************************************************************************************************
// Q: 20160714
// wysyłanie komendy w kierunku dir (1=przód, -1=tył) do kolejnego pojazdu (jednego)
// *************************************************************************************************
bool TMoverParameters::SendCtrlToNext(VehicleCommand const CtrlCommand, double const ctrlvalue, double const dir, CouplingSet const Couplertype)
{
	bool OK;
	int d; // numer sprzęgu w kierunku którego wysyłamy

	// Ra: był problem z propagacją, jeśli w składzie jest pojazd wstawiony odwrotnie
	// Ra: problem jest również, jeśli AI będzie na końcu składu
	OK = true; // ( dir != 0 ); // experimentally disabled
	d = (1 + static_cast<int>(Sign(dir))) / 2; // dir=-1=>d=0, dir=1=>d=1 - wysyłanie tylko w tył
	// musi być wybrana niezerowa kabina
	if (OK && Couplers[d].Connected != nullptr && Couplers[d].CouplingFlag.HasAll(Couplertype))
	{

		if (Couplers[d].ConnectedNr != d)
		{
			// jeśli ten nastpęny jest zgodny z aktualnym
			if (Couplers[d].Connected->SetInternalCommand(CtrlCommand, ctrlvalue, dir, Couplertype))
				OK = Couplers[d].Connected->RunInternalCommand() && OK; // tu jest rekurencja
		}
		else
		{
			// jeśli następny jest ustawiony przeciwnie, zmieniamy kierunek
			if (Couplers[d].Connected->SetInternalCommand(CtrlCommand, ctrlvalue, -dir, Couplertype))
				OK = Couplers[d].Connected->RunInternalCommand() && OK; // tu jest rekurencja
		}
	}
	return OK;
}

// *************************************************************************************************
// Q: 20160723
// *************************************************************************************************
// wysłanie komendy otrzymanej z kierunku CValue2 (względem sprzęgów: 1=przod,-1=tył)
// Ra: Jest tu problem z rekurencją. Trzeba by oddzielić wykonywanie komend od mechanizmu
// ich propagacji w składzie. Osobnym problemem może być propagacja tylko w jedną stronę.
// Jeśli jakiś człon jest wstawiony odwrotnie, to również odwrotnie musi wykonywać
// komendy związane z kierunkami (PantFront, PantRear, DoorLeft, DoorRight).
// Komenda musi być zdefiniowana tutaj, a jeśli się wywołuje funkcję, to ona nie może
// sama przesyłać do kolejnych pojazdów. Należy też się zastanowić, czy dla uzyskania
// jakiejś zmiany (np. IncMainCtrl) lepiej wywołać funkcję, czy od razu wysłać komendę.
bool TMoverParameters::RunCommand(VehicleCommand const Command, double CValue1, double CValue2, CouplingSet const Couplertype, std::string_view const Argument)
{
	// the group of the command sets OK; a command none of them knows isn't done
	if (bool OK{false}; RunControllerCommand(Command, CValue1, CValue2, Couplertype, OK) || RunRelaysCommand(Command, CValue1, CValue2, Couplertype, OK) ||
	                    RunCabCommand(Command, CValue1, CValue2, Couplertype, OK) ||
	                    RunBrakeCommand(Command, CValue1, CValue2, Couplertype, OK) ||
	                    RunBrakeEquipmentCommand(Command, CValue1, CValue2, Couplertype, OK) ||
	                    RunAuxiliariesCommand(Command, CValue1, CValue2, Couplertype, OK) ||
	                    RunPumpsCommand(Command, CValue1, CValue2, Couplertype, OK) ||
	                    RunBlowersCommand(Command, CValue1, CValue2, Couplertype, OK) ||
	                    RunPowerCommand(Command, CValue1, CValue2, Couplertype, OK) ||
	                    RunDoorsCommand(Command, CValue1, CValue2, Couplertype, OK) ||
	                    RunLoadCommand(Command, CValue1, CValue2, Argument, OK))
	{
		return OK; // dla true komenda będzie usunięta, dla false wykonana ponownie
	}
	return false;
}

// RunCommand() for the controllers; returns whether the command is one of them
bool TMoverParameters::RunControllerCommand(VehicleCommand const Command, double CValue1, double CValue2, CouplingSet const Couplertype, bool &OK)
{
	if (Command == VehicleCommand::mainController)
	{
		// sent as MainCtrlPosForCoupled(), passed on unchanged
		if (auto const position{MainCtrlPosFromCoupled(static_cast<int>(floor(CValue1)))}; MainCtrlPosNo >= position)
		{
			MainCtrlPos = position;
			if (DelayCtrlFlag)
			{
				if (LastRelayTime >= InitialCtrlDelay && MainCtrlPos == 1)
					LastRelayTime = 0;
			}
			else if (LastRelayTime > CtrlDelay)
				LastRelayTime = 0;
		}
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else if (Command == VehicleCommand::secondaryController)
	{
		if (ScndCtrlPosNo >= floor(CValue1))
		{
			ScndCtrlPos = static_cast<int>(floor(CValue1));
			if (LastRelayTime > CtrlDelay)
				LastRelayTime = 0;
		}
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	/*  else if command='BrakeCtrl' then
	begin
	if BrakeCtrlPosNo>=Trunc(CValue1) then
	begin
	BrakeCtrlPos:=Trunc(CValue1);
	OK:=SendCtrlToNext(command,CValue1,CValue2);
	end;
	end */
	else if (Command == VehicleCommand::integratedController) // ElectricInductionMotor Integrated Control - propulsion and brakes in relative values
	{
		eimic_real = CValue1;
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else
	{
		return false;
	}
	return true;
}

// RunCommand() for the direction, the automatic start and the current relays; returns whether the command is one of them
bool TMoverParameters::RunRelaysCommand(VehicleCommand const Command, double CValue1, double CValue2, CouplingSet const Couplertype, bool &OK)
{
	if (Command == VehicleCommand::direction)
	{
		DirActive = static_cast<int>(floor(CValue1));
		DirAbsolute = DirActive * CabActive;
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else if (Command == VehicleCommand::autoRelaySwitch)
	{
		if (CValue1 == 1 && AutoRelayType == 2)
			AutoRelayFlag = true;
		else
			AutoRelayFlag = false;
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else if (Command == VehicleCommand::maxCurrentSwitch)
	{
		MaxCurrentSwitch(CValue1 == 1, range_t::local);
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else if (Command == VehicleCommand::minCurrentSwitch)
	{
		OK = MinCurrentSwitch(CValue1 == 1);
	}
	/*test komend oddzialywujacych na tabor*/
	else
	{
		return false;
	}
	return true;
}

// RunCommand() for the cab and the speed control; returns whether the command is one of them
bool TMoverParameters::RunCabCommand(VehicleCommand const Command, double CValue1, double CValue2, CouplingSet const Couplertype, bool &OK)
{
	if (Command == VehicleCommand::cabActivation)
	{
		//  OK:=Power>0.01;
		switch (static_cast<int>(CValue1 * CValue2))
		{ // CValue2 ma zmieniany znak przy niezgodności sprzęgów
		case 1:
		{
			CabActive = 1;
			break;
		}
		case -1:
		{
			CabActive = -1;
			break;
		}
		default:
		{
			CabActive = 0; // gdy CValue1==0
			break;
		}
		}
		if (PulleeMode)
		{
			// a vehicle prepared for hauling takes no cab, the command only passes through it
			CabActive = 0;
		}
		DirAbsolute = DirActive * CabActive;
		CabMaster = false;
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else if (Command == VehicleCommand::speedControl)
	{
		if (EngineType == TEngineType::ElectricInductionMotor || SpeedCtrl)
			SpeedCtrlValue = static_cast<int>(round(CValue1));
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else if (Command == VehicleCommand::speedControlParking)
	{
		SpeedCtrlUnit.Parking = static_cast<bool>(CValue1);
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else
	{
		return false;
	}
	return true;
}

// RunCommand() for the train brake, the releaser and the emergency brake; returns whether the command is one of them
bool TMoverParameters::RunBrakeCommand(VehicleCommand const Command, double CValue1, double CValue2, CouplingSet const Couplertype, bool &OK)
{
	if (Command == VehicleCommand::brake) // youBy - jak sie EP hamuje, to trza sygnal wyslac...
	{
		if (CValue1 < 0.001)
			DynamicBrakeEMUStatus = true;
		double temp1 = CValue1;
		if (DCEMUED_EP_max_Vel > 0.001 && Vel > DCEMUED_EP_max_Vel && DynamicBrakeEMUStatus)
			temp1 = 0;
		if (DCEMUED_EP_min_Im > 0.001 && abs(Im) > DCEMUED_EP_min_Im && DynamicBrakeEMUStatus)
			temp1 = 0;
		Hamulec->SetEPS(temp1);
		TUHEX_StageActual = static_cast<int>(CValue1);
		TUHEX_Active = TUHEX_StageActual > 0;
		if (CValue1 < 0.001)
			DynamicBrakeEMUStatus = true;
		// fBrakeCtrlPos:=BrakeCtrlPos; //to powinnno być w jednym miejscu, aktualnie w C++!!!
		BrakePressureActual = BrakePressureTable[BrakeCtrlPos];
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	} // youby - odluzniacz hamulcow, przyda sie
	else if (Command == VehicleCommand::brakeReleaser)
	{
		OK = BrakeReleaser(static_cast<int>(Round(CValue1))); // samo się przesyła dalej
		                                    // OK:=SendCtrlToNext(command,CValue1,CValue2); //to robiło kaskadę 2^n
	}
	else if (Command == VehicleCommand::emergencyBrake)
	{
		if (RadiostopSwitch(floor(CValue1) == 1)) // YB: czy to jest potrzebne?
			OK = true;
		else
			OK = false;
	}
	else
	{
		return false;
	}
	return true;
}

// RunCommand() for the brake delays, the spring brake, the sand and the cab signalling; returns whether the command is one of them
bool TMoverParameters::RunBrakeEquipmentCommand(VehicleCommand const Command, double CValue1, double CValue2, CouplingSet const Couplertype, bool &OK)
{
	if (Command == VehicleCommand::brakeDelay)
	{
		if (auto const brakesetting = static_cast<int>(std::floor(CValue1)); true == Hamulec->SetBDF(brakesetting))
		{
			BrakeDelayFlag = brakesetting;
			OK = true;
		}
		else
		{
			OK = false;
		}
		SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else if (Command == VehicleCommand::springBrakeActivate)
	{
		if (Power24vIsAvailable || Power110vIsAvailable)
		{
			SpringBrake.Activate = CValue1 > 0;
			OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
		}
		else
			OK = true;
	}

	else if (Command == VehicleCommand::sandbox)
	{
		OK = Sandbox(CValue1 == 1, range_t::local);
	}
	else if (Command == VehicleCommand::cabSignal) /*SHP,Indusi*/
	{ // Ra: to powinno działać tylko w członie obsadzonym
		SecuritySystem.set_cabsignal();
		// else OK:=false;
		OK = true; // true, gdy można usunąć komendę
	}
	/*naladunek/rozladunek*/
	// TODO: have these commands leverage load exchange system instead
	// TODO: CValue1 defines amount to load/unload
	else
	{
		return false;
	}
	return true;
}

// RunCommand() for the heating, the converter, the battery and the compressor; returns whether the command is one of them
bool TMoverParameters::RunAuxiliariesCommand(VehicleCommand const Command, double CValue1, double CValue2, CouplingSet const Couplertype, bool &OK)
{
	if (Command == VehicleCommand::heatingSwitch)
	{
		HeatingSwitch_(CValue1 > 0.0);
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else if (Command == VehicleCommand::converterSwitch) /*NBMX*/
	{
		if (ConverterStart == start_t::manual)
		{
			ConverterAllow = CValue1 > 0.0;
		}
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else if (Command == VehicleCommand::batterySwitch) /*NBMX*/
	{
		// the battery of a vehicle prepared for hauling (PulleeMode) stays off
		if (BatteryStart == start_t::manual && (false == PulleeMode || CValue1 <= 0.0))
		{
			Battery = CValue1 > 0.0;
		}
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	//   else if command='EpFuseSwitch' then         {NBMX}
	//   begin
	//     if (CValue1=1) then EpFuse:=true
	//     else if (CValue1=0) then EpFuse:=false;
	//     OK:=SendCtrlToNext(command,CValue1,CValue2);
	//   end
	else if (Command == VehicleCommand::compressorSwitch) /*NBMX*/
	{
		CompressorSwitch(CValue1 == 1, range_t::local);
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else if (Command == VehicleCommand::compressorPreset)
	{
		CompressorListPos = safe_clamp(static_cast<int>(CValue1), 0, CompressorListPosNo);
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else
	{
		return false;
	}
	return true;
}

// RunCommand() for the water, fuel and oil pumps and the water heater; returns whether the command is one of them
bool TMoverParameters::RunPumpsCommand(VehicleCommand const Command, double CValue1, double CValue2, CouplingSet const Couplertype, bool &OK)
{
	if (Command == VehicleCommand::waterPumpBreakerSwitch)
	{
		/*
		        if( FuelPump.start_type != start::automatic ) {
		            // automatic fuel pump ignores 'manual' state commands
		*/
		WaterPump.breaker = CValue1 == 1;
		/*
		        }
		*/
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else if (Command == VehicleCommand::waterPumpSwitch)
	{

		if (WaterPump.start_type != start_t::battery)
		{
			// automatic fuel pump ignores 'manual' state commands
			WaterPump.is_enabled = CValue1 == 1;
		}
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else if (Command == VehicleCommand::waterPumpSwitchOff)
	{

		if (WaterPump.start_type != start_t::battery)
		{
			// automatic fuel pump ignores 'manual' state commands
			WaterPump.is_disabled = CValue1 == 1;
		}
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else if (Command == VehicleCommand::waterHeaterBreakerSwitch)
	{
		/*
		        if( FuelPump.start_type != start::automatic ) {
		            // automatic fuel pump ignores 'manual' state commands
		*/
		WaterHeater.breaker = CValue1 == 1;
		/*
		        }
		*/
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else if (Command == VehicleCommand::waterHeaterSwitch)
	{
		/*
		        if( FuelPump.start_type != start::automatic ) {
		            // automatic fuel pump ignores 'manual' state commands
		*/
		WaterHeater.is_enabled = CValue1 == 1;
		/*
		        }
		*/
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else if (Command == VehicleCommand::waterCircuitsLinkSwitch)
	{
		if (true == dizel_heat.auxiliary_water_circuit)
		{
			// can only link circuits if the vehicle has more than one of them
			WaterCircuitsLink = CValue1 == 1;
		}
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else if (Command == VehicleCommand::fuelPumpSwitch)
	{
		if (FuelPump.start_type != start_t::automatic)
		{
			// automatic fuel pump ignores 'manual' state commands
			FuelPump.is_enabled = CValue1 == 1;
		}
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else if (Command == VehicleCommand::fuelPumpSwitchOff)
	{
		if (FuelPump.start_type != start_t::automatic)
		{
			// automatic fuel pump ignores 'manual' state commands
			FuelPump.is_disabled = CValue1 == 1;
		}
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else if (Command == VehicleCommand::oilPumpSwitch)
	{
		if (OilPump.start_type != start_t::automatic)
		{
			// automatic pump ignores 'manual' state commands
			OilPump.is_enabled = CValue1 == 1;
		}
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else if (Command == VehicleCommand::oilPumpSwitchOff)
	{
		if (OilPump.start_type != start_t::automatic)
		{
			// automatic pump ignores 'manual' state commands
			OilPump.is_disabled = CValue1 == 1;
		}
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else
	{
		return false;
	}
	return true;
}

// RunCommand() for the motor blowers and the compartment lights; returns whether the command is one of them
bool TMoverParameters::RunBlowersCommand(VehicleCommand const Command, double CValue1, double CValue2, CouplingSet const Couplertype, bool &OK)
{
	if (Command == VehicleCommand::motorBlowersFrontSwitch)
	{
		if (MotorBlowers[end::front].start_type != start_t::manual && MotorBlowers[end::front].start_type != start_t::manualwithautofallback)
		{
			// automatic device ignores 'manual' state commands
			MotorBlowers[end::front].is_enabled = CValue1 == 1;
		}
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else if (Command == VehicleCommand::motorBlowersFrontSwitchOff)
	{
		if (MotorBlowers[end::front].start_type != start_t::manual && MotorBlowers[end::front].start_type != start_t::manualwithautofallback)
		{
			// automatic device ignores 'manual' state commands
			MotorBlowers[end::front].is_disabled = CValue1 == 1;
		}
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else if (Command == VehicleCommand::motorBlowersRearSwitch)
	{
		if (MotorBlowers[end::rear].start_type != start_t::manual && MotorBlowers[end::rear].start_type != start_t::manualwithautofallback)
		{
			// automatic device ignores 'manual' state commands
			MotorBlowers[end::rear].is_enabled = CValue1 == 1;
		}
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else if (Command == VehicleCommand::motorBlowersRearSwitchOff)
	{
		if (MotorBlowers[end::rear].start_type != start_t::manual && MotorBlowers[end::rear].start_type != start_t::manualwithautofallback)
		{
			// automatic device ignores 'manual' state commands
			MotorBlowers[end::rear].is_disabled = CValue1 == 1;
		}
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else if (Command == VehicleCommand::compartmentLightsSwitch)
	{

		if (CompartmentLights.start_type != start_t::automatic)
		{
			// automatic lights ignore 'manual' state commands
			CompartmentLights.is_enabled = CValue1 == 1;
		}
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else if (Command == VehicleCommand::compartmentLightsSwitchOff)
	{

		if (CompartmentLights.start_type != start_t::automatic)
		{
			// automatic lights ignore 'manual' state commands
			CompartmentLights.is_disabled = CValue1 == 1;
		}
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else
	{
		return false;
	}
	return true;
}

// RunCommand() for the line breaker, the relays and the pantographs; returns whether the command is one of them
bool TMoverParameters::RunPowerCommand(VehicleCommand const Command, double CValue1, double CValue2, CouplingSet const Couplertype, bool &OK)
{
	if (Command == VehicleCommand::mainSwitch)
	{
		MainSwitch_(CValue1 > 0.0);
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else if (Command == VehicleCommand::relayReset)
	{
		RelayReset(static_cast<int>(CValue1), range_t::local);
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else if (Command == VehicleCommand::pantographValve) // Winger 160204
	{ // Ra: uwzględnić trzeba jeszcze zgodność sprzęgów
		auto const inputend{(static_cast<int>(CValue1) & 0x80) != 0 ? 1 : 0};
		auto const inputcab{(static_cast<int>(CValue1) & 0x40) != 0 ? 1 : 0};
		auto const inputoperation{static_cast<int>(CValue1) & ~(0x80 | 0x40)};
		auto const noswap{TrainType == train_type::emu || TrainType == train_type::et41};
		auto swap{false == noswap && Couplers[CValue2 == -1 ? end::rear : end::front].CouplingFlag.HasAll(Coupling::control)};
		if (auto const reversed{inputcab != (CabActive != -1 ? 1 : 0)}; reversed)
		{
			swap = !swap;
		} // TODO: check whether this part has RL equivalent
		OperatePantographValve(static_cast<end>(swap ? 1 - inputend : inputend), static_cast<operation_t>(inputoperation), range_t::local);
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else if (Command == VehicleCommand::pantographsValve)
	{
		OperatePantographsValve(static_cast<operation_t>(static_cast<int>(CValue1)), range_t::local);
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else if (Command == VehicleCommand::pantographsAllDown)
	{
		DropAllPantographs(CValue1 == 1, range_t::local);
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else if (Command == VehicleCommand::tractionMode)
	{
		TractionModeSwitch(CValue1 > 0.5 ? traction_mode::genset : traction_mode::catenary, range_t::local);
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else if (Command == VehicleCommand::gensetSwitch)
	{
		GensetSwitch(CValue1 > 0.5, range_t::local);
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else
	{
		return false;
	}
	return true;
}

// RunCommand() for the doors and the departure signal; returns whether the command is one of them
bool TMoverParameters::RunDoorsCommand(VehicleCommand const Command, double CValue1, double CValue2, CouplingSet const Couplertype, bool &OK)
{
	if (Command == VehicleCommand::doorPermit)
	{
		DoorPermitCommand(CValue1, CValue2);
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else if (Command == VehicleCommand::doorOpen || Command == VehicleCommand::doorClose) /*NBMX*/
	{ // Ra: uwzględnić trzeba jeszcze zgodność sprzęgów
		DoorRemoteCommand(Command == VehicleCommand::doorOpen, CValue1, CValue2);
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else if (Command == VehicleCommand::doorLock)
	{
		Doors.lock_enabled = CValue1 == 1 ? true : false;
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else if (Command == VehicleCommand::doorStep)
	{
		Doors.step_enabled = CValue1 == 1 ? true : false;
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else if (Command == VehicleCommand::doorMode)
	{
		Doors.remote_only = CValue1 == 1 ? true : false;
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else if (Command == VehicleCommand::departureSignal)
	{
		DepartureSignal = CValue1 == 1 ? true : false;
		OK = SendCtrlToNext(Command, CValue1, CValue2, Couplertype);
	}
	else
	{
		return false;
	}
	return true;
}

// the doors' permits, on the sides the command names: CValue1 the sides, negative to take the permit back, CValue2 the
// direction the command comes from
void TMoverParameters::DoorPermitCommand(double const CValue1, double const CValue2)
{
	auto const leftside{CValue2 > 0 ? 1 : 2};
	if (auto const rightside{3 - leftside}; std::abs(static_cast<int>(CValue1)) & rightside)
	{
		PermitDoors_(side::right, CValue1 > 0);
	}
	if (std::abs(static_cast<int>(CValue1)) & leftside)
	{
		PermitDoors_(side::left, CValue1 > 0);
	}
}

// the doors opened or closed from afar on the sides the command names, unless they're only worked locally
void TMoverParameters::DoorRemoteCommand(bool const Open, double const CValue1, double const CValue2)
{
	using enum control_t;
	if (auto const doorcontrol{Open ? Doors.open_control : Doors.close_control};
	    (doorcontrol != conductor && doorcontrol != driver && doorcontrol != mixed) || (false == Power24vIsAvailable && false == Power110vIsAvailable))
	{
		return;
	}
	auto const leftside{CValue2 > 0 ? 1 : 2};
	if (auto const rightside{3 - leftside}; static_cast<int>(CValue1) & rightside)
	{
		Doors.instances[side::right].remote_open = Open;
		Doors.instances[side::right].remote_close = false == Open;
	}
	if (static_cast<int>(CValue1) & leftside)
	{
		Doors.instances[side::left].remote_open = Open;
		Doors.instances[side::left].remote_close = false == Open;
	}
}

// RunCommand() for the load and the damage; returns whether the command is one of them
bool TMoverParameters::RunLoadCommand(VehicleCommand const Command, double CValue1, double CValue2, std::string_view const Argument, bool &OK)
{
	if (Command == VehicleCommand::load)
	{
		OK = false; // będzie powtarzane aż się załaduje
		if (Vel < 0.1 // tolerance margin for small vehicle movements in the consist
		    && MaxLoad > 0 && LoadAmount < MaxLoad * (1.0 + OverLoadFactor) && Distance(Loc, CommandIn.Location, Dim, Dim) < (CValue2 > 1.0 ? CValue2 : 10.0))
		{ // ten peron/rampa

			auto const loadname{ToLower(std::string{Argument})};
			if (LoadAmount == 0.f)
			{
				AssignLoad(loadname);
			}
			OK = LoadingDone(LoadSpeed,
			                 loadname); // zmienia LoadStatus
		}
		else
		{
			// no loading can be done if conditions aren't met
			LastLoadChangeTime = 0.0;
		}
	}
	else if (Command == VehicleCommand::unload)
	{
		OK = false; // będzie powtarzane aż się rozładuje
		if (Vel < 0.1 // tolerance margin for small vehicle movements in the consist
		    && LoadAmount > 0 // czy jest co rozladowac?
		    && Distance(Loc, CommandIn.Location, Dim, Dim) < (CValue2 > 1.0 ? CValue2 : 10.0))
		{ // ten peron
			/*mozna to rozladowac*/
			OK = LoadingDone(-1.f * LoadSpeed, ToLower(std::string{Argument}));
		}
		else
		{
			// no loading can be done if conditions aren't met
			LastLoadChangeTime = 0.0;
		}
	}
	else if (Command == VehicleCommand::setDamage)
	{
		if (CValue2 == 1)
			OK = SetFlag(DamageFlag, static_cast<int>(floor(CValue1)));
		if (CValue2 == -1)
		{
			auto const derailed{TestFlag(DamageFlag, dtrain_out)};
			OK = SetFlag(DamageFlag, static_cast<int>(-floor(CValue1)));
			// taking the derailment off puts the vehicle back on the rails, with what the criteria of derailment held
			if (derailed && false == TestFlag(DamageFlag, dtrain_out))
				rerail();
		}
	}
	else
	{
		return false;
	}
	return true;
}

// *************************************************************************************************
// Q: 20160714
// Uruchamia funkcję RunCommand aż do skutku. Jeśli będzie pozytywny to kasuje komendę.
// *************************************************************************************************
bool TMoverParameters::RunInternalCommand()
{
	bool OK;

	if (CommandIn.Command != VehicleCommand::none)
	{
		OK = RunCommand(CommandIn.Command, CommandIn.Value1, CommandIn.Value2, CommandIn.couplings, CommandIn.Argument);
		if (OK)
		{

			CommandIn.Command = VehicleCommand::none; // kasowanie bo rozkaz wykonany
			CommandIn.Text.clear();
			CommandIn.Argument.clear();
			CommandIn.Value1 = 0;
			CommandIn.Value2 = 0;
			CommandIn.couplings = {};
			CommandIn.Location.X = 0;
			CommandIn.Location.Y = 0;
			CommandIn.Location.Z = 0;
			switch_physics(true, wake_cause::command);
		}
	}
	else
		OK = false;
	return OK;
}
