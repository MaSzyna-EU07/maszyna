/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the power supply: the batteries, the main switch, the pantographs, the converter, heating, lights, the genset

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


// KURS90 - sprężarka pantografów; Ra 2014-07: teraz jest to zbiornik rozrządu, chociaż to jeszcze nie tak
void TMoverParameters::UpdatePantVolume(double dt)
{
	// check the pantograph compressor while at it
	// TODO: move the check to a separate method
	// automatic start if the pressure is too low
	PantCompFlag |= PantPress < 4.2 && true == (Pantographs[end::front].is_active | Pantographs[end::rear].is_active) // TODO: any_pantograph_is_active method
	                && (PantographCompressorStart == start_t::automatic || PantographCompressorStart == start_t::manualwithautofallback);

	auto const lowvoltagepower{Power24vIsAvailable || Power110vIsAvailable};
	PantCompFlag &= lowvoltagepower;

	if (EnginePowerSource.SourceType == TPowerSource::CurrentCollector // tylko jeśli pantografujący
	    && EnginePowerSource.CollectorParameters.CollectorsNo > 0)
	{
		// Ra 2014-07: zasadniczo, to istnieje zbiornik rozrządu i zbiornik pantografów - na razie mamy razem
		// Ra 2014-07: kurek trójdrogowy łączy spr.pom. z pantografami i wyłącznikiem ciśnieniowym WS
		// Ra 2014-07: zbiornika rozrządu nie pompuje się tu, tylko pantografy; potem można zamknąć
		// WS i odpalić resztę
		if (PantAutoValve ? PantPress < ScndPipePress : bPantKurek3) // kurek zamyka połączenie z ZG
		{ // zbiornik pantografu połączony ze zbiornikiem głównym - małą sprężarką się tego nie napompuje
			// Ra 2013-12: Niebugocław mówi, że w EZT nie ma potrzeby odcinać kurkiem
			PantPress = ScndPipePress;
			// ograniczenie ciśnienia do MaxPress (tylko w pantografach!)
			PantPress = safe_clamp(ScndPipePress, 0.0, EnginePowerSource.CollectorParameters.MaxPress);
			PantVolume = (PantPress + 1.0) * 0.1; // objętość, na wypadek odcięcia kurkiem
		}
		else
		{ // zbiornik główny odcięty, można pompować pantografy
			if (PantCompFlag)
			{
				// włączona mała sprężarka
				PantVolume += dt
				              // Ra 2013-12: Niebugocław mówi, że w EZT nabija 1.5 raz wolniej niż jak było 0.005
				              * (TrainType == train_type::emu ? 0.003 : 0.005) / std::max(1.0, PantPress) * (0.45 - (0.1 / PantVolume / 10 - 0.1)) / 0.45;
			}
			PantPress = safe_clamp(10.0 * PantVolume - 1.0, 0.0, EnginePowerSource.CollectorParameters.MaxPress); // tu by się przydała objętość zbiornika
		}
		if (!PantCompFlag && PantVolume > 0.1)
			PantVolume -= dt * 0.0003 * std::max(1.0, PantPress * 0.5); // nieszczelności: 0.0003=0.3l/s

		PantPressureSwitchCheck();
	}
	else
	{ // a tu coś dla SM42 i SM31, aby pokazywać na manometrze
		PantPress = CntrlPipePress;
	}
};

// the pantographs' pressure switch: low, it opens the line breaker (or locks the converter of a unit) once;
// primed again over 4.6 bar, where the pantographs' compressor stops at 4.8
void TMoverParameters::PantPressureSwitchCheck()
{
	if (PantPress < EnginePowerSource.CollectorParameters.MinPress)
	{
		// 3.5 wg http://www.transportszynowy.pl/eu06-07pneumat.php
		if (true == PantPressSwitchActive)
		{
			PantPressureSwitchTrip();
		}
	}
	else if (PantPress >= 4.6)
	{
		// NOTE: we require active low power source to prime the pressure switch
		// this is a work-around for potential isssues caused by the switch activating on otherwise idle vehicles, but should check whether it's accurate
		if (true == Power24vIsAvailable || true == Power110vIsAvailable)
		{
			// prime the pressure switch
			PantPressSwitchActive = true;
			// turn off the subsystems lock
			PantPressLockActive = false;
		}

		if (PantPress >= 4.8)
		{
			// Winger - automatyczne wylaczanie malej sprezarki
			// TODO: governor lock, disables usage until pressure drop below 3.8 (should really make compressor object we could reuse)
			PantCompFlag = false;
		}
	}
}

// the pantographs' pressure switch tripped by the low pressure: the line breaker opened, unless the voltage comes from
// elsewhere, or the converter of a unit locked; spent until primed again
void TMoverParameters::PantPressureSwitchTrip()
{
	// opuszczenie pantografów przy niskim ciśnieniu
	if (TrainType != train_type::emu)
	{
		// pressure switch safety measure -- open the line breaker, unless there's alternate source of traction voltage
		// (the genset of a dual-mode vehicle, when the selector is on it, doesn't need the pantographs)
		if (GetTrainsetHighVoltage() < EnginePowerSource.CollectorParameters.MinV && false == GensetSelected())
		{
			// TODO: check whether line breaker should be open EMU-wide
			MainSwitch(false, TrainType == train_type::emu ? range_t::unit : range_t::local);
		}
	}
	else
	{
		// specialized variant for EMU -- pwr system disables converter and heating,
		// and prevents their activation until pressure switch is set again
		PantPressLockActive = true;
		// TODO: separate 'heating allowed' from actual heating flag, so we can disable it here without messing up heating toggle
		ConverterSwitch(false, range_t::unit);
	}
	// mark the pressure switch as spent
	PantPressSwitchActive = false;
}

void TMoverParameters::UpdateBatteryVoltage(double dt)
{ // przeliczenie obciążenia baterii
	using enum TEngineType;
	if (BatteryVoltage > 0 && EngineType != DieselEngine && EngineType != WheelsDriven && NominalBatteryVoltage > 0)
	{

		// HACK: allow to draw power also from adjacent converter, applicable for EMUs
		// TODO: expand power cables system to include low voltage power transfers
		// HACK: emulate low voltage generator powered directly by the diesel engine
		auto const converteractive{Power110vIsAvailable || (EngineType == DieselElectric && true == Mains) || (EngineType == DieselEngine && true == Mains)};

		BatteryVoltage -= BatteryDrain(dt, converteractive);
		if (NominalBatteryVoltage / BatteryVoltage > 1.57 && MainSwitch(false) && EngineType != DieselEngine && EngineType != WheelsDriven)
			EventFlag = true; // wywalanie szybkiego z powodu zbyt niskiego napiecia
		if (BatteryVoltage > NominalBatteryVoltage)
			BatteryVoltage = NominalBatteryVoltage; // wstrzymanie ładowania pow. 110V
		if (BatteryVoltage < 0.01)
			BatteryVoltage = 0.01;
	}
	else
	{
		// TODO: check and implement proper way to handle this for diesel engines
		BatteryVoltage = NominalBatteryVoltage;
	}
};

// the drop of the battery's voltage in dt, by what draws from it, or its rise by the converter charging it
double TMoverParameters::BatteryDrain(double dt, bool const converteractive) const
{
	if (NominalBatteryVoltage / BatteryVoltage < 1.22 && Battery)
	{ // 110V
		return BatteryDrainCharged(dt, converteractive);
	}
	if (NominalBatteryVoltage / BatteryVoltage >= 1.22 && Battery)
	{ // 90V
		return BatteryDrainLow(dt, converteractive);
	}
	if (!Battery)
	{
		return BatteryDrainSwitchedOff(dt);
	}
	return 0.0;
}

// the drain of a battery at its voltage, the converter charging it
double TMoverParameters::BatteryDrainCharged(double dt, bool const converteractive) const
{
	double sn1;
	double sn2;
	double sn3;
	double sn4;
	double sn5;
	if (!converteractive)
		sn1 = dt * 2.0; // szybki spadek do ok 90V
	else
		sn1 = 0;
	if (converteractive)
		sn2 = -(dt * 2.0); // szybki wzrost do 110V
	else
		sn2 = 0;
	if (Mains)
		sn3 = dt * 0.05;
	else
		sn3 = 0;
	if (iLights[0] & 63) // 64=blachy, nie ciągną prądu //rozpisać na poszczególne żarówki...
		sn4 = dt * 0.003;
	else
		sn4 = 0;
	if (iLights[1] & 63) // 64=blachy, nie ciągną prądu
		sn5 = dt * 0.001;
	else
		sn5 = 0;
	return sn1 + sn2 + sn3 + sn4 + sn5;
}

// the drain of a battery run down to 90 V, the converter charging it
double TMoverParameters::BatteryDrainLow(double dt, bool const converteractive) const
{
	double sn1;
	double sn2;
	double sn3;
	double sn4;
	double sn5;
	if (PantCompFlag)
		sn1 = dt * 0.0046;
	else
		sn1 = 0;
	if (converteractive)
		sn2 = -(dt * 50); // szybki wzrost do 110V
	else
		sn2 = 0;
	if (Mains)
		sn3 = dt * 0.001;
	else
		sn3 = 0;
	if (iLights[0] & 63) // 64=blachy, nie ciągną prądu
		sn4 = dt * 0.0030;
	else
		sn4 = 0;
	if (iLights[1] & 63) // 64=blachy, nie ciągną prądu
		sn5 = dt * 0.0010;
	else
		sn5 = 0;
	return sn1 + sn2 + sn3 + sn4 + sn5;
}

// the drain of a battery switched off, very slow
double TMoverParameters::BatteryDrainSwitchedOff(double dt) const
{
	double sn1;
	double sn2;
	double sn3;
	double sn4;
	double sn5;
	if (NominalBatteryVoltage / BatteryVoltage < 1.22)
		sn1 = dt * 50;
	else
		sn1 = 0;
	sn2 = dt * 0.000001;
	sn3 = dt * 0.000001;
	sn4 = dt * 0.000001;
	sn5 = dt * 0.000001; // bardzo powolny spadek przy wyłączonych bateriach
	return sn1 + sn2 + sn3 + sn4 + sn5;
}

void TMoverParameters::MainsCheck(double const Deltatime)
{

	//    if( MainsInitTime == 0.0 ) { return; }

	// TBD, TODO: move voltage calculation to separate method and use also in power coupler state calculation?
	auto localvoltage{0.0};
	switch (EnginePowerSource.SourceType)
	{
	case TPowerSource::CurrentCollector:
	{
		localvoltage = std::max(localvoltage, PantographVoltage);
		break;
	}
	case TPowerSource::Accumulator:
	{
		localvoltage = std::max(localvoltage, Power24vVoltage);
		break;
	}
	default:
	{
		break;
	}
	}
	if (GensetSelected())
	{
		// the traction selector of a dual-mode vehicle on the genset isolates the pantographs
		localvoltage = Genset.voltage;
	}
	// a line breaker closed ahead of the pantographs gets ready on the low voltage circuits
	auto const maincircuitpowersupply{std::abs(localvoltage) > 0.1 || GetTrainsetHighVoltage() > 0.1 || (PantographsRequireMains && (Power24vIsAvailable || Power110vIsAvailable))};

	if (true == maincircuitpowersupply)
	{
		// all is well
		if (MainsInitTimeCountdown >= 0.0)
		{
			// NOTE: we ensure main circuit readiness meets condition MainsInitTimeCountdown < 0
			// this allows for simpler rejection of cases where MainsInitTime == 0
			MainsInitTimeCountdown -= Deltatime;
		}
		else
		{
			// optional automatic circuit start
			if (MainsStart != start_t::manual && false == (Mains || dizel_startup))
			{
				MainSwitch(true);
			}
		}
	}
	else
	{
		// no power supply
		MainsInitTimeCountdown = MainsInitTime;
	}

	// an emergency brake application made under power opens the line breaker, where the vehicle has such an interlock
	if (Mains && EmergencyBrakeHoldsMainOff())
	{
		MainSwitch(false, TrainType == train_type::emu ? range_t::unit : range_t::local);
	}
}

void TMoverParameters::LowVoltagePowerCheck(double const /*Deltatime*/)
{

	auto const lowvoltagepower{Power24vIsAvailable || Power110vIsAvailable};

	if (EngineType == TEngineType::ElectricSeriesMotor)
	{
		GroundRelay &= lowvoltagepower;
		if (GroundRelayStart != start_t::manual)
		{
			// NOTE: we're ignoring intricaties of battery and converter types as they're unlikely to be used
			// TODO: generic check method which takes these into account
			GroundRelay |= lowvoltagepower;
		}
	}
}

void TMoverParameters::PowerCouplersCheck(double const /*Deltatime*/, Coupling const kind)
{
	if (kind != Coupling::highVoltage && kind != Coupling::power110v && kind != Coupling::power24v)
	{
		// only power couplings carry voltage
		return;
	}

	auto const localvoltage{LocalPowerVoltage(kind)};

	auto const abslocalvoltage{std::abs(localvoltage)};
	auto const localpowersource{(abslocalvoltage > 1.0)};

	// przekazywanie napiec
	PowerCouplersVoltage(kind, abslocalvoltage, localpowersource);

	// przekazywanie pradow
	PowerCouplersCurrent(kind, localpowersource);
}

// the current the vehicle draws from the circuit of the power coupling given
double *TMoverParameters::PowerCircuitCurrent(Coupling const kind)
{
	switch (kind)
	{
	case Coupling::highVoltage:
		return &TotalCurrent;
	case Coupling::power110v:
		return &PowerCircuits[1].second;
	case Coupling::power24v:
		return &PowerCircuits[0].second;
	default:
		return nullptr;
	}
}

// the voltage of the vehicle's own source for the coupling given; notes the low voltage circuits' too
double TMoverParameters::LocalPowerVoltage(Coupling const kind)
{
	auto localvoltage{0.0};

	// local power sources
	// the results could be kept in the PowerCircuit fields
	switch (kind)
	{

	case Coupling::highVoltage:
	{
		// heating power sources
		if (Heating)
		{
			switch (HeatingPowerSource.SourceType)
			{
			case TPowerSource::Generator:
			{
				localvoltage = HeatingPowerSource.EngineGenerator.voltage - TotalCurrent * 0.02;
				break;
			}
			case TPowerSource::CurrentCollector:
			{
				localvoltage = PantographVoltage;
				break;
			}
			case TPowerSource::Main:
			{
				// HACK: main circuit can be fed through couplers, so we explicitly check pantograph supply here
				// (or the genset's, where the traction selector of a dual-mode vehicle is on it)
				localvoltage = MainCircuitPowered() ? TractionSourceVoltage() : 0.0;
				break;
			}
			default:
			{
				break;
			}
			}
		}
		// high voltage power sources: the pantographs, or the genset of a dual-mode vehicle, by its traction selector
		if (EnginePowerSource.SourceType == TPowerSource::CurrentCollector)
		{
			localvoltage = std::max(localvoltage, TractionSourceVoltage());
		}
		break;
	}

	case Coupling::power110v:
	{
		if (ConverterFlag)
		{
			localvoltage = NominalBatteryVoltage;
		}
		// TBD, TODO: reduce by current draw?
		PowerCircuits[1].first = localvoltage;
		break;
	}

	case Coupling::power24v:
	{
		if (Battery)
		{
			localvoltage = BatteryVoltage;
		}
		// TBD, TODO: reduce by current draw?
		PowerCircuits[0].first = localvoltage;
		break;
	}

	default:
	{
		break;
	}
	}

	return localvoltage;
}

// the voltage passed through the vehicle to the coupler at its other end, from the coupled vehicle or its own source
void TMoverParameters::PowerCouplersVoltage(Coupling const kind, double const abslocalvoltage, bool const localpowersource)
{
	if (Couplers[end::front].power(kind) == nullptr)
	{
		// not a power coupling, there's no voltage to pass
		return;
	}
	for (auto side = 0; side < 2; ++side)
	{

		auto const &thiscoupler{Couplers[side]};
		// NOTE: in the loop we actually update the state of the coupler on the opposite end of the vehicle
		auto &oppositecoupler{Couplers[side == end::front ? end::rear : end::front]};

		bool oppositecouplingispresent;
		bool localpowerexportisenabled;

		switch (kind)
		{

		case Coupling::highVoltage:
		{
			auto const oppositehighvoltagecoupling{oppositecoupler.CouplingFlag.HasAny(Coupling::highVoltage)};
			auto const oppositeheatingcoupling{oppositecoupler.CouplingFlag.HasAny(Coupling::heating)};

			oppositecouplingispresent = oppositehighvoltagecoupling || oppositeheatingcoupling;
			localpowerexportisenabled = oppositehighvoltagecoupling || (oppositeheatingcoupling && localpowersource && Heating);
			break;
		}

		case Coupling::power110v:
		{
			oppositecouplingispresent = oppositecoupler.CouplingFlag.HasAll(oppositecoupler.PowerCoupling) && oppositecoupler.PowerFlag.HasAny(Coupling::power110v);
			localpowerexportisenabled = oppositecouplingispresent;
			break;
		}

		case Coupling::power24v:
		{
			oppositecouplingispresent = oppositecoupler.CouplingFlag.HasAll(oppositecoupler.PowerCoupling) && oppositecoupler.PowerFlag.HasAny(Coupling::power24v);
			localpowerexportisenabled = oppositecouplingispresent;
			break;
		}

		default:
		{
			break;
		}
		}

		auto const *coupling = thiscoupler.power(kind);
		auto *oppositecoupling = oppositecoupler.power(kind);

		// start with base voltage
		oppositecoupling->voltage = abslocalvoltage;
		oppositecoupling->is_live = false;
		oppositecoupling->is_local = localpowersource; // indicate power source
		// draw from external source
		if (thiscoupler.Connected != nullptr)
		{
			auto const &connectedcoupler{thiscoupler.Connected->Couplers[thiscoupler.ConnectedNr]};
			auto const *connectedcoupling = connectedcoupler.power(kind);
			auto const connectedvoltage{(connectedcoupling->is_live ? connectedcoupling->voltage : 0.0)};
			oppositecoupling->voltage = std::max(oppositecoupling->voltage, connectedvoltage - coupling->current * 0.02);
			oppositecoupling->is_live = connectedvoltage > 0.1 && oppositecouplingispresent;
		}
		// draw from local source
		if (localpowersource)
		{
			oppositecoupling->voltage = std::max(oppositecoupling->voltage, abslocalvoltage - coupling->current * 0.02);
			oppositecoupling->is_live |= abslocalvoltage > 0.1 && localpowerexportisenabled;
		}
	}
}

// the current drawn through the couplers, shared by their voltages, or added to the vehicle's own
void TMoverParameters::PowerCouplersCurrent(Coupling const kind, bool const localpowersource)
{
	auto const *frontcoupling{Couplers[end::front].power(kind)};
	auto const *rearcoupling{Couplers[end::rear].power(kind)};
	auto *totalcurrent{PowerCircuitCurrent(kind)};
	if (frontcoupling == nullptr || rearcoupling == nullptr || totalcurrent == nullptr)
	{
		// not a power coupling, there's no current to share
		return;
	}
	auto const couplervoltage{static_cast<int>(frontcoupling->voltage + rearcoupling->voltage)};

	for (auto &thiscoupler : Couplers)
	{
		auto *coupling = thiscoupler.power(kind);

		coupling->current = 0.0;

		if (thiscoupler.Connected == nullptr)
		{
			continue;
		}

		auto const &connectedothercoupler{thiscoupler.Connected->Couplers[thiscoupler.ConnectedNr == end::front ? end::rear : end::front]};
		auto const *connectedothercoupling = connectedothercoupler.power(kind);
		auto const extracurrent = kind == Coupling::highVoltage ? std::abs(Itot) * IsVehicleEIMBrakingFactor() : 0.0;

		if (false == localpowersource)
		{
			// bez napiecia...
			if (couplervoltage != 0.0)
			{
				// ...ale jest cos na sprzegach:
				coupling->current = (*totalcurrent + extracurrent) * coupling->voltage / couplervoltage; // obciążenie rozkladane stosownie do napiec
				if (true == coupling->is_live)
				{
					coupling->current += connectedothercoupling->current;
				}
			}
		}
		else
		{
			if (true == coupling->is_live)
			{
				*totalcurrent += connectedothercoupling->current;
			}
		}
	}
}

// sprawdzanie przetwornicy
void TMoverParameters::ConverterCheck(double const Timestep)
{
	// TODO: move other converter checks here, to have it all in one place for potential device object
	if (ConverterStart != start_t::disabled && ConverterOverloadRelayOffWhenMainIsOff)
	{
		ConvOvldFlag |= !Mains && Power24vIsAvailable;
	}

	switch (ConverterStart)
	{
	case start_t::disabled:
	{
		ConverterAllow = false;
		// NOTE: if there's no converter in vehicle we can end the check here
		return;
	}
	case start_t::automatic:
	{
		ConverterAllow = Mains;
		break;
	}
	case start_t::direction:
	{
		ConverterAllow = DirActive != 0;
	}
	default:
	{
		break;
	}
	}

	if (ConverterAllow && ConverterAllowLocal && false == ConvOvldFlag &&
	    false == PantPressLockActive
	    // HACK: allow carriages to operate converter without (missing) fuse prerequisite
	    && (Power > 1.0 ? MainCircuitPowered() : GetTrainsetHighVoltage() > 0.0))
	{
		// delay timer can be optionally configured, and is set anew whenever converter goes off
		if (ConverterStartDelayTimer <= 0.0)
		{
			ConverterFlag = true;
		}
		else
		{
			ConverterStartDelayTimer -= Timestep;
		}
	}
	else
	{
		ConverterFlag = false;
		ConverterStartDelayTimer = static_cast<double>(ConverterStartDelay);
	}

	if (ConverterOverloadRelayStart == start_t::converter && false == (ConverterAllow && ConverterAllowLocal) && false == TestFlag(EngDmgFlag, 4))
	{
		// reset converter overload relay if the converter was switched off, unless it's damaged
		ConvOvldFlag = false;
	}
};

// heating system status check
void TMoverParameters::HeatingCheck(double const /*Timestep*/)
{

	// update heating devices
	// TBD, TODO: move this to a separate method?
	if (HeatingPowerSource.SourceType == TPowerSource::Generator)
	{
		if (HeatingPowerSource.EngineGenerator.engine_revolutions != nullptr && HeatingPowerSource.EngineGenerator.revolutions_max > 0)
		{

			auto &generator{HeatingPowerSource.EngineGenerator};
			// TBD, TODO: engine-generator transmission
			generator.revolutions = *generator.engine_revolutions;

			auto const absrevolutions{std::abs(generator.revolutions)};
			generator.voltage = (false == HeatingAllow ? 0.0 :
			                                             // TODO: add support for desired voltage selector
			                         absrevolutions < generator.revolutions_min ?
			                                             generator.voltage_min * absrevolutions / generator.revolutions_min :
			                                             //                    absrevolutions > generator.revolutions_max ? generator.voltage_max * absrevolutions / generator.revolutions_max :
			                                             std::lerp(generator.voltage_min, generator.voltage_max,
			                                                         std::clamp((absrevolutions - generator.revolutions_min) / (generator.revolutions_max - generator.revolutions_min), 0.0, 1.0))) *
			                    sign(generator.revolutions);
		}
	}

	// quick check first to avoid unnecessary calls...
	if (false == HeatingAllow)
	{
		Heating = false;
		return;
	}
	// ...detailed check if we're still here
	auto const heatingpowerthreshold{0.1};
	// start with blank slate
	auto voltage{0.0};
	// then try specified power source
	switch (HeatingPowerSource.SourceType)
	{
	case TPowerSource::Generator:
	{
		voltage = HeatingPowerSource.EngineGenerator.voltage;
		break;
	}
	case TPowerSource::CurrentCollector:
	{
		voltage = PantographVoltage;
		break;
	}
	case TPowerSource::PowerCable:
	{
		if (HeatingPowerSource.PowerType == TPowerType::ElectricPower)
		{
			// TBD, TODO: limit input voltage to heating coupling type?
			voltage = GetTrainsetHighVoltage();
		}
		break;
	}
	case TPowerSource::Main:
	{
		voltage = MainCircuitPowered() ? TractionSupplyVoltage() : 0.0;
		break;
	}
	default:
	{
		break;
	}
	}

	Heating = voltage > heatingpowerthreshold;

	if (Heating)
	{
		TotalCurrent += 1000 * HeatingPower / voltage; // heater power cost presumably specified in kilowatts
	}
}

void TMoverParameters::PantographsCheck(double const /*Timestep*/)
{
	using enum start_t;

	{
		auto const &valve{PantsValve};
		auto const lowvoltagepower{valve.solenoid ? Power24vIsAvailable || Power110vIsAvailable : true};
		auto const autostart{valve.start_type == automatic || valve.start_type == manualwithautofallback};
		auto const manualcontrol{valve.start_type == manual || valve.start_type == manualwithautofallback};

		PantsValve.is_active = (valve.spring ? lowvoltagepower : true) // spring actuator needs power to maintain non-default state
		                       && (manualcontrol && lowvoltagepower ? false == valve.is_disabled : true) // needs power to change state
		                       && (valve.is_active || (autostart  ? lowvoltagepower :
		                                               !autostart ? lowvoltagepower && valve.is_enabled :
		                                                            false)); // shouldn't ever get this far but, eh
	}

	size_t pant_id = 0;
	for (auto &pantograph : Pantographs)
	{

		auto &valve{pantograph.valve};
		auto const lowvoltagepower{valve.solenoid ? Power24vIsAvailable || Power110vIsAvailable : true};
		auto const autostart{valve.start_type == automatic || valve.start_type == manualwithautofallback};
		auto const manualcontrol{valve.start_type == manual || valve.start_type == manualwithautofallback};

		valve.is_active = (valve.spring ? lowvoltagepower : true) // spring actuator needs power to maintain non-default state
		                  && (manualcontrol && lowvoltagepower ? false == valve.is_disabled : true) // needs power to change state, without it just pass through
		                  && (manualcontrol && lowvoltagepower ? false == PantAllDown : true) &&
		                  (Mains || false == PantographsRequireMains) // fed through the closed line breaker, where it comes first
		                  && (valve.is_active || (manualcontrol && lowvoltagepower && valve.is_enabled) || (autostart && lowvoltagepower)); // shouldn't ever get this far but, eh

		auto const pantographexists{EnginePowerSource.SourceType == TPowerSource::CurrentCollector && EnginePowerSource.CollectorParameters.PhysicalLayout & 1 << pant_id};

		pantograph.is_active = valve.is_active && PantsValve.is_active && pantographexists;

		pant_id++;
	}
}

// the genset of a dual-mode vehicle: started on request with the battery on, it cranks for start_time, then runs at
// idle and up to full revolutions with the traction demand; stopped, it coasts down. regulated voltage from idle up
void TMoverParameters::GensetCheck(double const Deltatime)
{
	if (false == Genset.fitted())
	{
		return;
	}
	auto &genset{Genset};
	auto const wasactive{genset.is_active};
	auto const lowvoltagepower{Power24vIsAvailable || Power110vIsAvailable};
	auto const requested{lowvoltagepower && false == genset.is_disabled && (genset.start_type == start_t::automatic ? GensetSelected() : genset.is_enabled)};
	if (false == requested)
	{
		genset.is_active = false;
		genset.is_cranking = false;
		genset.timer = 0.0;
	}
	else if (false == genset.is_active)
	{
		genset.is_cranking = true;
		genset.timer += Deltatime;
		if (genset.timer >= genset.start_time)
		{
			genset.is_cranking = false;
			genset.is_active = true;
		}
	}
	// revolutions: the start brings them up to idle in start_time; running they follow the traction demand, stopped
	// they coast down to 0, both with the time constant spinup_time
	if (genset.is_cranking)
	{
		genset.revolutions = std::max(genset.revolutions, genset.revolutions_idle * std::min(1.0, genset.timer / std::max(genset.start_time, 0.01)));
	}
	else if (genset.is_active && false == wasactive)
	{
		genset.revolutions = std::max(genset.revolutions, genset.revolutions_idle);
	}
	else
	{
		auto const demand{GensetSelected() && Mains ? std::clamp(eimv_pr, 0.0, 1.0) : 0.0};
		auto const target{genset.is_active ? genset.revolutions_idle + (genset.revolutions_max - genset.revolutions_idle) * demand : 0.0};
		genset.revolutions += (target - genset.revolutions) * std::min(1.0, Deltatime / genset.spinup_time);
		if (genset.revolutions < 0.001)
		{
			genset.revolutions = 0.0;
		}
	}
	// no excitation while cranking; running or coasting down, regulated from idle up, proportional below it
	genset.voltage = genset.is_cranking ? 0.0 : genset.voltage_nominal * std::clamp(genset.revolutions / genset.revolutions_idle, 0.0, 1.0);

	if (genset.is_active != wasactive)
	{
		WriteLog("Genset of " + Name + (genset.is_active ? ": running" : ": stopped"));
	}
}

void TMoverParameters::LightsCheck(double const /*Timestep*/)
{

	auto &light{CompartmentLights};

	light.is_active =
	    // TODO: bind properly power source when ld is in place
	    (Power24vIsAvailable || Power110vIsAvailable) // power source
	    && false == light.is_disabled && (true == light.is_active || (light.start_type == start_t::manual ? light.is_enabled : true));

	light.intensity = (light.is_active ? 1.0f : 0.0f)
	                  // TODO: bind properly power source when ld is in place
	                  * (Power110vIsAvailable ? 1.0f :
	                     Power24vIsAvailable  ? 0.5f :
	                                            0.0f) *
	                  light.dimming;
}

// *************************************************************************************************
// Q: 20160710
// włączenie / wyłączenie baterii
// *************************************************************************************************
bool TMoverParameters::BatterySwitch(bool State, range_t const Notify)
{
	using enum range_t;
	if (PulleeMode && State)
	{
		// the battery of a vehicle prepared for hauling stays off
		return false;
	}
	auto const initialstate{Battery};

	// Ra: ukrotnienie załączania baterii jest jakąś fikcją...
	if (BatteryStart == start_t::manual)
	{
		Battery = State;
	}

	// switching batteries does not require activation
	if (Notify != local)
	{
		SendCtrlToNext(VehicleCommand::batterySwitch, State ? 1 : 0, 1, Notify == unit ? Coupling::control | Coupling::permanent : Coupling::control);
		SendCtrlToNext(VehicleCommand::batterySwitch, State ? 1 : 0, -1, Notify == unit ? Coupling::control | Coupling::permanent : Coupling::control);
	}

	return Battery != initialstate;
}

bool TMoverParameters::CompartmentLightsSwitch(bool State, range_t const Notify)
{

	if (CompartmentLights.start_type == start_t::automatic)
	{
		// automatic lights ignore 'manual' state commands
		return false;
	}

	bool const initialstate{CompartmentLights.is_enabled};

	CompartmentLights.is_enabled = State;

	if (Notify != range_t::local)
	{
		SendCtrlToNext(VehicleCommand::compartmentLightsSwitch, CompartmentLights.is_enabled ? 1 : 0, CabActive, Notify == range_t::unit ? Coupling::control | Coupling::permanent : Coupling::control);
	}

	return CompartmentLights.is_enabled != initialstate;
}

// water pump state toggle
bool TMoverParameters::CompartmentLightsSwitchOff(bool State, range_t const Notify)
{

	if (CompartmentLights.start_type == start_t::automatic)
	{
		// automatic lights ignore 'manual' state commands
		return false;
	}

	bool const initialstate{CompartmentLights.is_disabled};

	CompartmentLights.is_disabled = State;

	if (Notify != range_t::local)
	{
		SendCtrlToNext(VehicleCommand::compartmentLightsSwitchOff, CompartmentLights.is_disabled ? 1 : 0, CabActive, Notify == range_t::unit ? Coupling::control | Coupling::permanent : Coupling::control);
	}

	return CompartmentLights.is_disabled != initialstate;
}

// *************************************************************************************************
// Q: 20160713
// włączenie / wyłączenie obwodu głownego
// *************************************************************************************************
bool TMoverParameters::MainSwitch(bool const State, range_t const Notify)
{

	bool const initialstate{Mains || dizel_startup};

	MainSwitch_(State);

	if (Notify != range_t::local)
	{
		// pass the command to other vehicles
		// TBD: pass the requested state, or the actual state?
		SendCtrlToNext(VehicleCommand::mainSwitch, State ? 1 : 0, CabActive, Notify == range_t::unit ? Coupling::control | Coupling::permanent : Coupling::control);
	}

	return (Mains || dizel_startup) != initialstate;
}

void TMoverParameters::MainSwitch_(bool const State)
{

	if (Mains == State || MainCtrlPosNo == 0)
	{
		// nothing to do
		return;
	}

	bool const initialstate{Mains};

	if (false == State || true == MainSwitchCheck())
	{

		if (true == State)
		{
			// switch on
			if (EngineType == TEngineType::DieselEngine || EngineType == TEngineType::DieselElectric)
			{
				// launch diesel engine startup procedure
				dizel_startup = true;
			}
			else
			{
				// additional check, as vehicles without pantographs won't fail relay checks earlier
				Mains = true;
			}
		}
		else
		{
			Mains = false;
			// potentially knock out the pumps if their switch doesn't force them on
			WaterPump.is_active &= WaterPump.is_enabled;
			FuelPump.is_active &= FuelPump.is_enabled;
		}

		if (Mains != initialstate)
		{
			LastSwitchingTime = 0;
			Timer::subsystem.switching.note(Timer::switching_counter::mains, this, &TypeName, &Name);
		}
	}
}

bool TMoverParameters::MainSwitchCheck() const
{
	using enum TEngineType;

	// prevent the switch from working if there's no power
	// TODO: consider whether it makes sense for diesel engines and such
	bool powerisavailable{true};

	switch (EngineType)
	{
	case DieselElectric:
	case DieselEngine:
	case Dumb:
	{
		powerisavailable = Power24vIsAvailable;
		break;
	}
	case ElectricSeriesMotor:
	case ElectricInductionMotor:
	{
		// TODO: check whether we can simplify this check and skip the outer EngineType switch
		// closed ahead of the pantographs, the line breaker runs on the low voltage circuits
		powerisavailable = PantographsRequireMains ? Power24vIsAvailable || Power110vIsAvailable : EnginePowerSourceVoltage() > 0.5 * EnginePowerSource.MaxVoltage;
		break;
	}
	default:
	{
		break;
	}
	}
	if (GensetSelected())
	{
		// on the genset of a dual-mode vehicle the line breaker closes only on a running one, whatever the pantographs give
		powerisavailable = Genset.is_active && Genset.voltage >= TractionSupplyMinVoltage();
	}

	return powerisavailable && (ScndCtrlPos == 0 || EngineType == ElectricInductionMotor) && MainsInitTimeCountdown <= 0.0 &&
	       (ConvOvldFlag == false || ConverterOverloadRelayOffWhenMainIsOff) && true == GroundRelay && (true == NoVoltRelay || PantographsRequireMains) && true == OvervoltageRelay &&
	       LastSwitchingTime > CtrlDelay &&
	       (HasCamshaft                       ? IsMainCtrlActualNoPowerPos() :
	        LineBreakerClosesOnlyAtNoPowerPos ? IsMainCtrlNoPowerPos() :
	                                            true) &&
	       false == TestFlag(DamageFlag, dtrain_out) && false == TestFlag(EngDmgFlag, 1) && false == EmergencyBrakeHoldsMainOff();
}

// *************************************************************************************************
// Q: 20160713
// włączenie / wyłączenie przetwornicy
// *************************************************************************************************
bool TMoverParameters::ConverterSwitch(bool State, range_t const Notify)
{

	auto const initialstate{ConverterAllow};

	if (ConverterStart == start_t::manual)
	{
		ConverterAllow = State;
	}

	if (Notify != range_t::local)
	{
		SendCtrlToNext(VehicleCommand::converterSwitch, State ? 1 : 0, CabActive, Notify == range_t::unit ? Coupling::control | Coupling::permanent : Coupling::control);
	}

	return ConverterAllow != initialstate;
}

bool TMoverParameters::HeatingSwitch(bool const State, range_t const Notify)
{

	bool const initialstate{HeatingAllow};

	HeatingSwitch_(State);

	if (Notify != range_t::local)
	{
		// pass the command to other vehicles
		// TBD: pass the requested state, or the actual state?
		SendCtrlToNext(VehicleCommand::heatingSwitch, State ? 1 : 0, CabActive, Notify == range_t::unit ? Coupling::control | Coupling::permanent : Coupling::control);
	}

	return HeatingAllow != initialstate;
}

void TMoverParameters::HeatingSwitch_(bool const State)
{

	// TBD, TODO: activation dependencies?
	HeatingAllow = State;
}

// the start request of the genset of a dual-mode vehicle; one which starts on its own ignores it, but passes it on to
// the cars behind, which may start theirs on request
bool TMoverParameters::GensetSwitch(bool const State, range_t const Notify)
{
	auto const initialstate{Genset.is_enabled};

	if (Genset.start_type != start_t::automatic)
	{
		Genset.is_enabled = State;
	}

	if (Notify != range_t::local)
	{
		SendCtrlToNext(VehicleCommand::gensetSwitch, State ? 1 : 0, CabActive, Notify == range_t::unit ? Coupling::control | Coupling::permanent : Coupling::control);
	}

	return Genset.is_enabled != initialstate;
}

// the voltage of the vehicle's own source the traction selector picks: the catenary through the pantographs, or the
// genset of a dual-mode vehicle
double TMoverParameters::TractionSourceVoltage() const
{
	return GensetSelected() ? Genset.voltage : PantographVoltage;
}

// the voltage the traction circuit gets: from the vehicle's own source, or through the train's high voltage line
double TMoverParameters::TractionSupplyVoltage() const
{
	return std::max(GetTrainsetHighVoltage(), TractionSourceVoltage());
}

// below it the undervoltage trip opens the line breaker
double TMoverParameters::TractionSupplyMinVoltage() const
{
	return GensetSelected() ? 0.5 * Genset.voltage_nominal : EnginePowerSource.CollectorParameters.MinV;
}

// the power the traction may take [kW, at the wheels]: the genset's share at its present revolutions on the genset,
// no limit otherwise
double TMoverParameters::GensetTractionPowerLimit() const
{
	return GensetSelected() ? Genset.available_power() * Genset.traction_share : std::numeric_limits<double>::max();
}

// the current the vehicle draws through its pantographs [A], if they touch the wire: the traction's (none while
// braking on the resistors) and the heating's. on the genset the genset feeds the traction circuit, so only a heating
// fed straight from the pantographs is left
double TMoverParameters::CatenaryCurrent() const
{
	if (GensetSelected())
	{
		return HeatingPowerSource.SourceType == TPowerSource::CurrentCollector ? TotalCurrent : 0.0;
	}
	return (DynamicBrakeFlag && ResistorsFlag ? 0 : std::abs(Itot) * IsVehicleEIMBrakingFactor()) + TotalCurrent;
}

// returns voltage of defined main engine power source
double TMoverParameters::EnginePowerSourceVoltage() const
{

	return EnginePowerSource.SourceType == TPowerSource::CurrentCollector ? TractionSupplyVoltage() :
	       EnginePowerSource.SourceType == TPowerSource::Accumulator      ? Power24vVoltage :
	                                                                        0.0;
}

// whether the main circuit feeds what runs on it: the converter, a compressor, the heating, the motor and resistor
// blowers, the traction motors
bool TMoverParameters::MainCircuitPowered() const
{
	if (false == Mains)
	{
		return false;
	}
	if (false == PantographsRequireMains || EnginePowerSource.SourceType != TPowerSource::CurrentCollector)
	{
		// the closed line breaker says it all: the undervoltage trip opens it without traction voltage, and an engine
		// which runs is its own supply
		return true;
	}
	// closed ahead of the pantographs the line breaker holds without the voltage, which the undervoltage trip isn't
	// there to vouch for
	return EnginePowerSourceVoltage() >= TractionSupplyMinVoltage();
}

bool TMoverParameters::OperatePantographsValve(operation_t const State, range_t const Notify)
{

	if (EnginePowerSource.SourceType == TPowerSource::CurrentCollector && EnginePowerSource.CollectorParameters.CollectorsNo > 0)
	{

		auto &valve{PantsValve};

		switch (State)
		{
		using enum operation_t;
		case none:
		{
			valve.is_enabled = false;
			valve.is_disabled = false;
			break;
		}
		case enable:
		{
			valve.is_enabled = true;
			valve.is_disabled = false;
			break;
		}
		case disable:
		{
			valve.is_enabled = false;
			valve.is_disabled = true;
			break;
		}
		case enable_on:
		{
			valve.is_enabled = true;
			break;
		}
		case enable_off:
		{
			valve.is_enabled = false;
			break;
		}
		case disable_on:
		{
			valve.is_disabled = true;
			break;
		}
		case disable_off:
		{
			valve.is_disabled = false;
			break;
		}
		}
	}

	if (Notify != range_t::local)
	{
		SendCtrlToNext(VehicleCommand::pantographsValve, static_cast<double>(State), CabActive, Notify == range_t::unit ? Coupling::control | Coupling::permanent : Coupling::control);
	}

	return true;
}

bool TMoverParameters::OperatePantographValve(end const End, operation_t const State, range_t const Notify)
{

	if (EnginePowerSource.SourceType == TPowerSource::CurrentCollector && EnginePowerSource.CollectorParameters.CollectorsNo > 0)
	{

		auto &valve{Pantographs[End].valve};

		switch (State)
		{
		using enum operation_t;
		case none:
		{
			valve.is_enabled = false;
			valve.is_disabled = false;
			break;
		}
		case enable:
		{
			valve.is_enabled = true;
			valve.is_disabled = false;
			break;
		}
		case disable:
		{
			valve.is_enabled = false;
			valve.is_disabled = true;
			break;
		}
		case enable_on:
		{
			valve.is_enabled = true;
			break;
		}
		case enable_off:
		{
			valve.is_enabled = false;
			break;
		}
		case disable_on:
		{
			valve.is_disabled = true;
			break;
		}
		case disable_off:
		{
			valve.is_disabled = false;
			break;
		}
		}
	}

	if (Notify != range_t::local)
	{
		SendCtrlToNext(VehicleCommand::pantographValve,
		               // HACK: pack the state, pantograph index and sender cab into 8-bit value
		               // with high bit storing front/rear pantograph, and 7th bit storing sender cab
		               static_cast<double>(0x80 * (End == end::front ? 0 : 1) + 0x40 * (CabActive != -1 ? 1 : 0) + static_cast<int>(State)), CabActive,
		               Notify == range_t::unit ? Coupling::control | Coupling::permanent : Coupling::control);
	}

	return true;
}

bool TMoverParameters::DropAllPantographs(bool const State, range_t const Notify)
{

	auto const initialstate{PantAllDown};

	PantAllDown = State;

	if (Notify != range_t::local)
	{
		SendCtrlToNext(VehicleCommand::pantographsAllDown, State ? 1 : 0, CabActive, Notify == range_t::unit ? Coupling::control | Coupling::permanent : Coupling::control);
	}

	return State != initialstate;
}

// *************************************************************************************************
// Q: 20160709
// Funkcja zwracajaca napiecie dla calego skladu, przydatna dla EZT
// *************************************************************************************************
double TMoverParameters::GetTrainsetVoltage(CouplingSet const couplings) const
{ // ABu: funkcja zwracajaca napiecie dla calego skladu, przydatna dla EZT
	// TBD, TODO: call once per vehicle update, return cached results?
	std::array<double, 2> voltages{0.0, 0.0};
	for (int end = end::front; end <= end::rear; ++end)
	{
		if (Couplers[end].Connected == nullptr)
		{
			continue;
		}
		auto const &thiscoupler{Couplers[end]};
		if (auto const fullcoupling{thiscoupler.CouplingFlag | (thiscoupler.CouplingFlag.HasAll(thiscoupler.PowerCoupling) ? thiscoupler.PowerFlag : CouplingSet{})}; false == fullcoupling.HasAny(couplings))
		{
			continue;
		}
		auto const *connectedpowercoupling = couplings.HasAny(Coupling::highVoltage | Coupling::heating) ? &thiscoupler.Connected->Couplers[thiscoupler.ConnectedNr].power_high :
		                               couplings.HasAny(Coupling::power110v)                         ? &thiscoupler.Connected->Couplers[thiscoupler.ConnectedNr].power_110v :
		                               couplings.HasAny(Coupling::power24v)                          ? &thiscoupler.Connected->Couplers[thiscoupler.ConnectedNr].power_24v :
		                                                                                               nullptr;
		if (connectedpowercoupling != nullptr && connectedpowercoupling->is_live)
		{
			voltages[end] = connectedpowercoupling->voltage;
		}
	}
	return std::max(voltages[end::front], voltages[end::rear]);
}

double TMoverParameters::GetTrainsetHighVoltage() const
{

	return std::max(GetTrainsetVoltage(Coupling::highVoltage), HeatingAllow ? GetTrainsetVoltage(Coupling::heating) : 0.0);
}

int TMoverParameters::clampDimmerPosition(int const Position) const
{
	if (dimPositions.empty())
	{
		return 0;
	}
	return std::clamp(Position, 0, static_cast<int>(dimPositions.size()) - 1);
}

TMoverParameters::dimPosition TMoverParameters::currentDimPosition() const
{
	if (modernDimmerPosition < 0 || modernDimmerPosition >= static_cast<int>(dimPositions.size()))
	{
		return {};
	}
	return dimPositions[modernDimmerPosition];
}

bool TMoverParameters::stepDimmerPosition(int const Step)
{
	if (dimPositions.empty())
	{
		return false;
	}
	auto const last{static_cast<int>(dimPositions.size()) - 1};
	auto const current{clampDimmerPosition(modernDimmerPosition)};
	auto next{current + Step};
	if (next < 0 || next > last)
	{
		if (false == modernDimmerCanCycle)
		{
			next = current;
		}
		else
		{
			next = (next < 0 ? last : 0);
		}
	}
	if (next == modernDimmerPosition)
	{
		return false;
	}
	modernDimmerPosition = next;
	return true;
}

std::optional<lights_preset_cablight> TMoverParameters::LightsPresetCabLight(int const Position) const
{
	if (Position < 1 || Position > static_cast<int>(LightsCabLight.size()))
	{
		return std::nullopt;
	}
	auto const flags{LightsCabLight[Position - 1]};
	if (flags < 0)
	{
		return std::nullopt;
	}
	return lights_preset_cablight{(flags & lights_preset_cablight::light_on) != 0, (flags & lights_preset_cablight::light_dimmed) != 0};
}
