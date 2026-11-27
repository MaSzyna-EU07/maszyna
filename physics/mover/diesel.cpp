/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the diesel engine: its revolutions, the gearbox, the clutch, the hydraulic transmission, its heat and pumps

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
#include "physics/mover/internal.h"


double TMoverParameters::EngineRPMRatio() const
{

	return std::clamp(EngineType == TEngineType::DieselElectric ? 60.0 * std::abs(enrot) / DElist[MainCtrlPosNo].RPM :
	                  EngineType == TEngineType::DieselEngine   ? std::abs(enrot) / nmax :
	                                                              1.0, // shouldn't ever get here but, eh
	             0.0, 1.0);
}

double TMoverParameters::EngineIdleRPM() const
{

	return EngineType == TEngineType::DieselEngine   ? dizel_nmin * 60 :
	       EngineType == TEngineType::DieselElectric ? DElist[MainCtrlNoPowerPos()].RPM :
	                                                   std::numeric_limits<double>::max(); // shouldn't ever get here but, eh
}

double TMoverParameters::EngineMaxRPM() const
{

	return EngineType == TEngineType::DieselEngine   ? dizel_nmax * 60 :
	       EngineType == TEngineType::DieselElectric ? DElist[MainCtrlPosNo].RPM :
	                                                   std::numeric_limits<double>::max(); // shouldn't ever get here but, eh
}

// water pump status check
void TMoverParameters::WaterPumpCheck(double const /*Timestep*/)
{
	// NOTE: breaker override with start type is sm42 specific hack, replace with ability to define the presence of the breaker
	WaterPump.is_active = true == (Power24vIsAvailable || Power110vIsAvailable) && true == WaterPump.breaker && false == WaterPump.is_disabled &&
	                      (true == WaterPump.is_active || true == WaterPump.is_enabled || WaterPump.start_type == start_t::battery);
}

// water heater status check
void TMoverParameters::WaterHeaterCheck(double const /*Timestep*/)
{

	WaterHeater.is_active = false == WaterHeater.is_damaged && true == (Power24vIsAvailable || Power110vIsAvailable) && true == WaterHeater.is_enabled && true == WaterHeater.breaker &&
	                        (WaterHeater.is_active || WaterHeater.config.temp_min < 0 || dizel_heat.temperatura1 < WaterHeater.config.temp_min);

	WaterHeater.is_damaged = true == WaterHeater.is_damaged || (true == WaterHeater.is_active && false == WaterPump.is_active);

	if (WaterHeater.config.temp_max > 0 && dizel_heat.temperatura1 > WaterHeater.config.temp_max)
	{
		WaterHeater.is_active = false;
	}
}

// fuel pump status update
void TMoverParameters::FuelPumpCheck(double const /*Timestep*/)
{
	using enum start_t;

	FuelPump.is_active = true == (Power24vIsAvailable || Power110vIsAvailable) && false == FuelPump.is_disabled &&
	                     (FuelPump.is_active || (FuelPump.start_type == manual                 ? FuelPump.is_enabled :
	                                             FuelPump.start_type == automatic              ? dizel_startup || Mains :
	                                             FuelPump.start_type == manualwithautofallback ? FuelPump.is_enabled || dizel_startup || Mains :
	                                                                                                      false)); // shouldn't ever get this far but, eh
}

// oil pump status update
void TMoverParameters::OilPumpCheck(double const Timestep)
{

	OilPump.is_active = true == (Power24vIsAvailable || Power110vIsAvailable) && false == Mains && false == OilPump.is_disabled &&
	                    (OilPump.is_active || (OilPump.start_type == start_t::manual                 ? OilPump.is_enabled :
	                                           OilPump.start_type == start_t::automatic              ? dizel_startup :
	                                           OilPump.start_type == start_t::manualwithautofallback ? OilPump.is_enabled || dizel_startup :
	                                                                                                   false)); // shouldn't ever get this far but, eh

	auto const minpressure{OilPump.pressure_minimum > 0.f ? OilPump.pressure_minimum : 0.15f}; // arbitrary fallback value

	OilPump.pressure_target = enrot > 0.1 ? std::lerp( minpressure, OilPump.pressure_maximum, static_cast<float>( EngineRPMRatio() ) ) * OilPump.resource_amount :
        true == OilPump.is_active ? std::min( minpressure + 0.1f, OilPump.pressure_maximum ) : // slight pressure margin to give time to switch off the pump and start the engine
        0.f;

	if (OilPump.pressure < OilPump.pressure_target)
	{
		// TODO: scale change rate from 0.01-0.05 with oil/engine temperature/idle time
		OilPump.pressure = std::min<float>(OilPump.pressure_target, static_cast<float>(OilPump.pressure + (enrot > 5.0 ? 0.05 : 0.035) * Timestep));
	}
	if (OilPump.pressure > OilPump.pressure_target)
	{
		OilPump.pressure = std::max<float>(OilPump.pressure_target, static_cast<float>(OilPump.pressure - (enrot > 5.0 ? 0.05 : 0.035) * 0.5 * Timestep));
	}
	OilPump.pressure = std::clamp(OilPump.pressure, 0.f, 1.5f);
}

// water pump breaker state toggle
bool TMoverParameters::WaterPumpBreakerSwitch(bool State, range_t const Notify)
{
	/*
	    if( FuelPump.start_type == start::automatic ) {
	        // automatic fuel pump ignores 'manual' state commands
	        return false;
	    }
	*/
	bool const initialstate{WaterPump.breaker};

	WaterPump.breaker = State;

	if (Notify != range_t::local)
	{
		SendCtrlToNext("WaterPumpBreakerSwitch", WaterPump.breaker ? 1 : 0, CabActive, Notify == range_t::unit ? coupling::control | coupling::permanent : coupling::control);
	}

	return WaterPump.breaker != initialstate;
}

// water pump state toggle
bool TMoverParameters::WaterPumpSwitch(bool State, range_t const Notify)
{

	if (WaterPump.start_type == start_t::battery)
	{
		// automatic fuel pump ignores 'manual' state commands
		return false;
	}

	bool const initialstate{WaterPump.is_enabled};

	WaterPump.is_enabled = State;

	if (Notify != range_t::local)
	{
		SendCtrlToNext("WaterPumpSwitch", WaterPump.is_enabled ? 1 : 0, CabActive, Notify == range_t::unit ? coupling::control | coupling::permanent : coupling::control);
	}

	return WaterPump.is_enabled != initialstate;
}

// water pump state toggle
bool TMoverParameters::WaterPumpSwitchOff(bool State, range_t const Notify)
{

	if (WaterPump.start_type == start_t::battery)
	{
		// automatic fuel pump ignores 'manual' state commands
		return false;
	}

	bool const initialstate{WaterPump.is_disabled};

	WaterPump.is_disabled = State;

	if (Notify != range_t::local)
	{
		SendCtrlToNext("WaterPumpSwitchOff", WaterPump.is_disabled ? 1 : 0, CabActive, Notify == range_t::unit ? coupling::control | coupling::permanent : coupling::control);
	}

	return WaterPump.is_disabled != initialstate;
}

// water heater breaker state toggle
bool TMoverParameters::WaterHeaterBreakerSwitch(bool State, range_t const Notify)
{
	/*
	    if( FuelPump.start_type == start::automatic ) {
	        // automatic fuel pump ignores 'manual' state commands
	        return false;
	    }
	*/
	bool const initialstate{WaterHeater.breaker};

	WaterHeater.breaker = State;

	if (Notify != range_t::local)
	{
		SendCtrlToNext("WaterHeaterBreakerSwitch", WaterHeater.breaker ? 1 : 0, CabActive, Notify == range_t::unit ? coupling::control | coupling::permanent : coupling::control);
	}

	return WaterHeater.breaker != initialstate;
}

// water heater state toggle
bool TMoverParameters::WaterHeaterSwitch(bool State, range_t const Notify)
{
	/*
	    if( FuelPump.start_type == start::automatic ) {
	    // automatic fuel pump ignores 'manual' state commands
	    return false;
	    }
	*/
	bool const initialstate{WaterHeater.is_enabled};

	WaterHeater.is_enabled = State;

	if (Notify != range_t::local)
	{
		SendCtrlToNext("WaterHeaterSwitch", WaterHeater.is_enabled ? 1 : 0, CabActive, Notify == range_t::unit ? coupling::control | coupling::permanent : coupling::control);
	}

	return WaterHeater.is_enabled != initialstate;
}

// water circuits link state toggle
bool TMoverParameters::WaterCircuitsLinkSwitch(bool State, range_t const Notify)
{

	if (false == dizel_heat.auxiliary_water_circuit)
	{
		// can't link the circuits if the vehicle only has one
		return false;
	}

	bool const initialstate{WaterCircuitsLink};

	WaterCircuitsLink = State;

	if (Notify != range_t::local)
	{
		SendCtrlToNext("WaterCircuitsLinkSwitch", WaterCircuitsLink ? 1 : 0, CabActive, Notify == range_t::unit ? coupling::control | coupling::permanent : coupling::control);
	}

	return WaterCircuitsLink != initialstate;
}

// fuel pump state toggle
bool TMoverParameters::FuelPumpSwitch(bool State, range_t const Notify)
{

	if (FuelPump.start_type == start_t::automatic)
	{
		// automatic fuel pump ignores 'manual' state commands
		return false;
	}

	bool const initialstate{FuelPump.is_enabled};

	FuelPump.is_enabled = State;

	if (Notify != range_t::local)
	{
		SendCtrlToNext("FuelPumpSwitch", FuelPump.is_enabled ? 1 : 0, CabActive, Notify == range_t::unit ? coupling::control | coupling::permanent : coupling::control);
	}

	return FuelPump.is_enabled != initialstate;
}

bool TMoverParameters::FuelPumpSwitchOff(bool State, range_t const Notify)
{

	if (FuelPump.start_type == start_t::automatic)
	{
		// automatic fuel pump ignores 'manual' state commands
		return false;
	}

	bool const initialstate{FuelPump.is_disabled};

	FuelPump.is_disabled = State;

	if (Notify != range_t::local)
	{
		SendCtrlToNext("FuelPumpSwitchOff", FuelPump.is_disabled ? 1 : 0, CabActive, Notify == range_t::unit ? coupling::control | coupling::permanent : coupling::control);
	}

	return FuelPump.is_disabled != initialstate;
}

// oil pump state toggle
bool TMoverParameters::OilPumpSwitch(bool State, range_t const Notify)
{

	if (OilPump.start_type == start_t::automatic)
	{
		// automatic pump ignores 'manual' state commands
		return false;
	}

	bool const initialstate{OilPump.is_enabled};

	OilPump.is_enabled = State;

	if (Notify != range_t::local)
	{
		SendCtrlToNext("OilPumpSwitch", OilPump.is_enabled ? 1 : 0, CabActive, Notify == range_t::unit ? coupling::control | coupling::permanent : coupling::control);
	}

	return OilPump.is_enabled != initialstate;
}

bool TMoverParameters::OilPumpSwitchOff(bool State, range_t const Notify)
{

	if (OilPump.start_type == start_t::automatic)
	{
		// automatic pump ignores 'manual' state commands
		return false;
	}

	bool const initialstate{OilPump.is_disabled};

	OilPump.is_disabled = State;

	if (Notify != range_t::local)
	{
		SendCtrlToNext("OilPumpSwitchOff", OilPump.is_disabled ? 1 : 0, CabActive, Notify == range_t::unit ? coupling::control | coupling::permanent : coupling::control);
	}

	return OilPump.is_disabled != initialstate;
}

// the revolutions of the engine; returns those a diesel-electric engine aims at [1/s], 0 for the others.
// for a diesel engine, the ratio of the transmission and the engine's moment
double TMoverParameters::UpdateEngineRevolutions(double dt, double &dtrans, double &dmoment)
{
	double tmp{0.0};
	// youBy
	switch (EngineType)
	{
	case TEngineType::DieselElectric:
	{
		if (true == Mains && true == FuelPump.is_active)
		{
			tmp = DieselElectricTargetRevolutions();
		}
		else
		{
			tmp = 0.0;
			dizel_fill = 0.0;
		}

		if (enrot != tmp)
		{
			enrot = safe_clamp(enrot + dt / dizel_AIM * (enrot < tmp ? 1.0 : -1.0 * dizel_RevolutionsDecreaseRate), // NOTE: revolutions typically drop faster than they rise
			              0.0, std::max(tmp, enrot));
			if (std::abs(tmp - enrot) < 0.001)
			{
				enrot = tmp;
			}
		}
		break;
	}
	case TEngineType::DieselEngine:
	{
		if (ShuntMode) // dodatkowa przekładnia np. dla 2Ls150
			dtrans = AnPos * Transmision.Ratio * motor_param(ScndCtrlActualPos).mIsat;
		else
			dtrans = Transmision.Ratio * motor_param(ScndCtrlActualPos).mIsat;

		dmoment = dizel_Momentum(dizel_fill, dtrans * nrot * DirActive, dt); // oblicza tez enrot
		break;
	}
	default:
	{
		enrot = Transmision.Ratio * nrot;
		break;
	}
	}

	return tmp;
}

// the revolutions a diesel-electric's engine aims at [1/s], by the controller and the heating; the fill shown by the smoke
double TMoverParameters::DieselElectricTargetRevolutions()
{
	double tmp;

	if (EIMCtrlType > 0) // sterowanie cyfrowe
		tmp = (DElist[0].RPM + (DElist[MainCtrlPosNo].RPM - DElist[0].RPM) * std::max(0.0, eimic_real)) / 60.0;
	else
		tmp = DElist[ControlPressureSwitch ? MainCtrlNoPowerPos() : MainCtrlPos].RPM / 60.0;

	if (true == HeatingAllow && HeatingPower > 0 && EngineHeatingRPM > 0)
	{
		// bump engine revolutions up if needed, when heating is on
		tmp = std::max(tmp, std::min(EngineMaxRPM(), EngineHeatingRPM) / 60.0);
	}
	// NOTE: fake dizel_fill calculation for the sake of smoke emitter which uses this parameter to determine smoke opacity
	dizel_fill = std::clamp(0.2 + 0.35 * (tmp - enrot) + 0.5 * (std::abs(Im) / DElist[MainCtrlPosNo].Imax), 0.05, 1.0);
	return tmp;
}

void TMoverParameters::DieselEngineTractionForce(double dt, double const dmoment, double const dtrans)
{
	Mw = dmoment * dtrans * Transmision.Efficiency; // dmoment i dtrans policzone przy okazji enginerotation
	if (hydro_R && hydro_R_Placement == 0)
		Mw -= dizel_MomentumRetarder(nrot * Transmision.Ratio, dt) * Transmision.Ratio * Transmision.Efficiency;
	Fw = Mw * 2.0 / WheelDiameter / NPoweredAxles;
	Ft = Fw * NPoweredAxles; // sila trakcyjna
	Ft = Ft * DirAbsolute; // DirActive*CabActive;
}

// tmp: the revolutions the engine aims at [1/s], see UpdateEngineRevolutions()
void TMoverParameters::DieselElectricTractionForce(double tmp)
{
	double PosRatio;
	//       tmpV:=V*CabActive*DirActive;
	auto const tmpV{nrot * Pirazy2 * 0.5 * WheelDiameter * DirAbsolute}; //*CabActive*DirActive;
	auto tempUmax = 0.0;
	auto tempImax = 0.0;
	// jazda manewrowa
	if (EIMCtrlType > 0) // sterowanie cyfrowe
	{
		tmp = DieselElectricDigitalForce(tmpV, tempUmax, tempImax);
		PosRatio = tmp / DElist[MainCtrlPosNo].GenPower;
	}
	else if (true == ShuntMode)
	{
		DieselElectricShuntForce(tmpV, tmp);
		PosRatio = 1;
	}
	else // jazda ciapongowa
	{
		auto power = Power;
		tempImax = DElist[MainCtrlPos].Imax;
		tempUmax = DElist[MainCtrlPos].Umax;
		if (true == Heating)
		{
			power -= HeatingPower;
		}
		power = std::max(power, 0.0);
		// NOTE: very crude way to approximate power generated at current rpm instead of instant top output
		// NOTE, TODO: doesn't take into account potentially increased revolutions if heating is on, fix it
		auto const currentgenpower{(DElist[MainCtrlPos].RPM > 0 ? DElist[MainCtrlPos].GenPower * (60.0 * enrot / DElist[MainCtrlPos].RPM) : 0.0)};

		tmp = std::min(power, currentgenpower);

		PosRatio = currentgenpower / DElist[MainCtrlPosNo].GenPower;
		// stosunek mocy teraz do mocy max
		// NOTE: Mains in this context is working diesel engine
		if (true == StLinFlag)
			Ft = DieselElectricForce(tmpV, power, PosRatio, tmp);
		else
			Ft = 0; // jak nastawnik na zero, to sila tez zero

		PosRatio = tmp / DElist[MainCtrlPosNo].GenPower;
	}

	if (FuseFlag)
		Ft = 0;
	else
		Ft = Ft * DirAbsolute; // DirActive * CabActive; //zwrot sily i jej wartosc
	Fw = Ft / NPoweredAxles; // sila na obwodzie kola
	Mw = Fw * WheelDiameter / 2.0; // moment na osi kola
	Mm = Mw / Transmision.Ratio; // moment silnika trakcyjnego

	Im = dieselelectric_motor_current(Mm);

	if (ShuntMode)
	{
		DieselElectricShuntPower(tmp);
	}
	else
	{
		DieselElectricGenerator(tmp, tempUmax, tempImax);
	}

	if (Imax > 1 && Im > Imax)
		FuseOff();
	if (FuseFlag)
		EngineVoltage = 0;

	// przekazniki bocznikowania, kazdy inny dla kazdej pozycji
	if (auto const fieldshuntingcontrol{fieldshunting::diesel_electric(AutoRelayType, FieldShuntSwitch, AutoRelayFlag)};
	    false == StLinFlag || ShuntMode || fieldshuntingcontrol == fieldshunting::control::off)
	{
		ScndCtrlPos = 0;
	}
	else if (AutoRelayFlag)
	{
		// the shunting relays set the field shunting (fieldshunting::control::automatic)
		ShuntFieldRelaysCheck(tmpV, PosRatio, tmp);
	}
}

// the tractive force of a diesel-electric shunting, by the generator's table SST and the shunting controller AnPos;
// tmp, the power the generator gives
void TMoverParameters::DieselElectricShuntForce(double const tmpV, double &tmp)
{
	if (true == StLinFlag)
	{
		EngineVoltage = SST[MainCtrlPos].Umax * AnPos + SST[MainCtrlPos].Umin * (1.0 - AnPos);
		// NOTE: very crude way to approximate power generated at current rpm instead of instant top output
		// NOTE, TODO: doesn't take into account potentially increased revolutions if heating is on, fix it
		tmp = EngineRPMRatio() * (SST[MainCtrlPos].Pmax * AnPos) + SST[MainCtrlPos].Pmin * (1.0 - AnPos);
		Ft = tmp * 1000.0 / (abs(tmpV) + 1.6);
	}
	else
	{
		EngineVoltage = 0;
		Ft = 0;
	}
}

// the tractive force of a diesel-electric set by the integrated controller, at the speed tmpV; the generator's
// voltage and current limits too. returns the power the generator gives
double TMoverParameters::DieselElectricDigitalForce(double const tmpV, double &tempUmax, double &tempImax)
{
	auto eimic_positive = std::max(0.0, eimic_real);
	auto const rpmratio{EngineRPMRatio()};
	tempImax = DElist[MainCtrlPosNo].Imax * eimic_positive;
	tempUmax = DElist[MainCtrlPosNo].Umax * std::min(eimic_positive, rpmratio);
	auto const tempPmax = DElist[MainCtrlPosNo].GenPower * std::min(eimic_positive, rpmratio);
	if (true == StLinFlag)
		Ft = DieselElectricForce(tmpV, tempPmax, eimic_positive, tempPmax);
	else
		Ft = 0; // jak nastawnik na zero, to sila tez zero
	return tempPmax;
}

// the tractive force of a diesel-electric at the speed tmpV: on the straight part of its characteristic under the speed
// Vhyp gives at the power LinePower, at the share Ratio of it, on the hyperbola of the power HyperbolaPower over it
double TMoverParameters::DieselElectricForce(double const tmpV, double const LinePower, double const Ratio, double const HyperbolaPower) const
{
	if (tmpV < Vhyp * LinePower / DElist[MainCtrlPosNo].GenPower)
	{
		// czy na czesci prostej, czy na hiperboli
		return (Ftmax - (Ftmax - 1000.0 * DElist[MainCtrlPosNo].GenPower / (Vhyp + Vadd)) * (tmpV / Vhyp) / PowerCorRatio) * Ratio; // posratio - bo sila jakos tam sie rozklada
	}
	// na hiperboli
	// 1.107 - wspolczynnik sredniej nadwyzki Ft w symku nad charakterystyka
	return 1000.0 * HyperbolaPower / (tmpV + Vadd) / PowerCorRatio; // tu jest zawarty stosunek mocy
}

// in the shunting mode, the generator's power limited to tmp, the power the engine gives
void TMoverParameters::DieselElectricShuntPower(double const tmp)
{
	EnginePower = EngineVoltage * Im / 1000.0;
	if (EnginePower > tmp)
	{
		EnginePower = tmp;
		EngineVoltage = EnginePower * 1000.0 / Im;
	}
	if (EnginePower < tmp)
	{
		Ft *= EnginePower / tmp;
	}
}

// the generator's current limited to tempImax and its voltage by its characteristic, under tempUmax and the power tmp
void TMoverParameters::DieselElectricGenerator(double const tmp, double const tempUmax, double const tempImax)
{
	if (abs(Im) > tempImax)
	{ // nie ma nadmiarowego, tylko Imax i zwarcie na pradnicy
		Ft = Ft / Im * tempImax;
		Im = tempImax;
	}

	if (Im > 0)
	{
		// jak pod obciazeniem
		if (true == Flat)
		{
			// ograniczenie napiecia w pradnicy - plaszczak u gory
			EngineVoltage = 1000.0 * tmp / std::abs(Im);
		}
		else
		{
			auto tempMCP = EIMCtrlType > 0 ? 1 + 99 * std::max(1.0, eimic_real) : MainCtrlPos;
			auto tempMCPN = EIMCtrlType > 0 ? 100 : MainCtrlPosNo;
			// charakterystyka pradnicy obcowzbudnej (elipsa) - twierdzenie Pitagorasa
			EngineVoltage = std::sqrt(std::abs(square(tempUmax) - square(tempUmax * Im / tempImax))) * (tempMCP - 1) + (1.0 - Im / tempImax) * tempUmax * (tempMCPN - tempMCP);
			EngineVoltage /= tempMCPN - 1;
			EngineVoltage = safe_clamp(EngineVoltage, Im * 0.05, 1000.0 * tmp / std::abs(Im));
		}
	}

	if (EngineVoltage > tempUmax || Im == 0)
	{
		// gdy wychodzi za duze napiecie albo przy biegu jalowym (jest cos takiego?)
		EngineVoltage = tempUmax * (ConverterFlag ? 1 : 0);
	}

	EnginePower = EngineVoltage * Im / 1000.0;
	/*
	                // power curve drop
	                // NOTE: disabled for the time being due to side-effects
	                if( ( tmpV > 1 ) && ( EnginePower < tmp ) ) {
	                    Ft = std::lerp(
	                        Ft, EnginePower / tmp,
	                        std::clamp( tmpV - 1.0, 0.0, 1.0 ) );
	                }
	*/
}

// the current of a diesel-electric vehicle's traction motors giving the moment [N m] of one motor
double TMoverParameters::dieselelectric_motor_current(double const Moment) const
{
	// with MotorParam[ScndCtrlPos] do
	if (abs(Moment) > motor_param(ScndCtrlPos).fi)
		return NPoweredAxles * abs(abs(Moment) / motor_param(ScndCtrlPos).mfi + motor_param(ScndCtrlPos).mIsat);
	else
		return NPoweredAxles * sqrt(abs(Moment * motor_param(ScndCtrlPos).Isat));
}

// the effort of a diesel-electric vehicle's dynamic brake the driver asks for, 0..1
double TMoverParameters::dieselelectric_dynamic_brake_demand() const
{
	switch (DEDynamicBrake.source)
	{
	case diesel_electric_dynamic_brake::control::controller:
		return std::clamp(-eimic_real, 0.0, 1.0);
	case diesel_electric_dynamic_brake::control::lever:
		return DynamicBrakeRatio();
	default:
		return 0.0;
	}
}

// the force [N] of a diesel-electric vehicle's dynamic brake at full effort at the speed: the most the motors take
// (the current), above the speed where the resistors' power takes over, the power; under EDBrakeV1 it fades, down to
// nothing at EDBrakeV0. a brake of direct current motors, whose force grows with the speed (the voltage they generate
// into the resistors), fades from 0 to the speed of its whole force; an inverter's only at the stop
double TMoverParameters::dieselelectric_dynamic_brake_limit() const
{
	auto const &brake{DEDynamicBrake};
	auto limit{brake.max_force};
	if (brake.max_power > 0.0 && std::abs(V) > 0.0)
	{
		limit = std::min(limit, brake.max_power / std::abs(V));
	}
	if (brake.fade_high > brake.fade_low)
	{
		limit *= std::clamp((Vel - brake.fade_low) / (brake.fade_high - brake.fade_low), 0.0, 1.0);
	}
	else if (Vel < brake.fade_low)
	{
		limit = 0.0;
	}
	return limit;
}

// the dynamic brake of a diesel-electric vehicle, before the drive's force: the effort follows the demand, the force
// is the share of the most at the speed. the motors brake while the engine runs (it excites them, or keeps the
// control of the inverters going), with the direction set, the overload relay closed, no power asked for and the
// wheels rolling; otherwise there's no brake at once
void TMoverParameters::update_dieselelectric_dynamic_brake(double const dt)
{
	auto &brake{DEDynamicBrake};
	auto const nopower{brake.source == diesel_electric_dynamic_brake::control::controller ? eimic_real <= 0.0 : IsMainCtrlNoPowerPos()};
	if (Mains && DirActive != 0 && false == FuseFlag && false == SlippingWheels && nopower)
	{
		brake.level += std::clamp(dieselelectric_dynamic_brake_demand() - brake.level, -brake.rate * dt, brake.rate * dt);
	}
	else
	{
		brake.level = 0.0;
	}
	brake.force = brake.level * dieselelectric_dynamic_brake_limit();
	DynamicBrakeFlag = brake.force > 0.0;
	if (brake.blending)
	{
		// the train brake of the locomotive lets off by the share of the dynamic brake's force (LSt, EStED)
		Hamulec->SetED(brake.max_force > 0.0 ? brake.force / brake.max_force : 0.0);
	}
}

// the force of a diesel-electric vehicle's dynamic brake, against the motion, in place of the drive's one: the motors
// work as generators into the resistors, the ammeter shows their current, the main generator gives nothing. near the
// stop no more than what stops the vehicle within the step dt: with no fade (EDBrakeV0 = EDBrakeV1 = 0) the whole force
// would turn round each time the speed crosses zero, and in FTrain it never lets the stop of ComputeMovement() catch it
void TMoverParameters::apply_dieselelectric_dynamic_brake(double const dt)
{
	auto const force{dt > 0.0 ? std::min(DEDynamicBrake.force, TotalMass * std::abs(V) / dt) : DEDynamicBrake.force};
	Ft = V > 0.0 ? -force : V < 0.0 ? force : 0.0;
	Fw = Ft / NPoweredAxles;
	Mw = Fw * WheelDiameter / 2.0;
	Mm = Mw / Transmision.Ratio;
	Im = dieselelectric_motor_current(Mm);
	EngineVoltage = 0.0;
	EnginePower = 0.0;
}

// *************************************************************************************************
// Q: 20160715
// Zmienia parametr do którego dąży sprzęgło
// *************************************************************************************************
bool TMoverParameters::dizel_EngageSwitch(double state)
{
	if (EngineType == TEngineType::DieselEngine && state <= 1 && state >= 0 && state != dizel_engagestate)
	{
		dizel_engagestate = state;
		return true;
	}
	else
		return false;
}

// *************************************************************************************************
// Q: 20160715
// Zmienia parametr do którego dąży sprzęgło
// *************************************************************************************************
bool TMoverParameters::dizel_EngageChange(double dt)
{
	double engagespeed = 0; // OK:boolean;
	bool DEC;

	DEC = false;
	if (dizel_engage - dizel_engagestate > 0)
		engagespeed = engagedownspeed;
	else
		engagespeed = engageupspeed;
	if (dt > 0.2)
		dt = 0.1;
	if (abs(dizel_engage - dizel_engagestate) < 0.11)
	{
		if (dizel_engage != dizel_engagestate)
		{
			DEC = true;
			dizel_engage = dizel_engagestate;
		}
		// else OK:=false; //już jest false
	}
	else
	{
		dizel_engage = dizel_engage + engagespeed * dt * (dizel_engagestate - dizel_engage);
		// OK:=false;
	}
	// dizel_EngageChange:=OK;
	return DEC;
}

// *************************************************************************************************
// Q: 20160715
// Automatyczna zmiana biegów gdy prędkość przekroczy widełki
// *************************************************************************************************
bool TMoverParameters::dizel_AutoGearCheck(void)
{
	auto OK{false};

	auto const VelUp{(motor_param(ScndCtrlActualPos).mfi0 != 0.0 ?
	                      motor_param(ScndCtrlActualPos).mfi0 + (motor_param(ScndCtrlActualPos).mfi - motor_param(ScndCtrlActualPos).mfi0) * std::max(0.0, eimic_real) :
	                      motor_param(ScndCtrlActualPos).mfi)};

	auto const VelDown{(motor_param(ScndCtrlActualPos).fi0 != 0.0 && eimic_real <= 0.0 ? motor_param(ScndCtrlActualPos).fi0 : motor_param(ScndCtrlActualPos).fi)};

	if (motor_param(ScndCtrlActualPos).AutoSwitch && Mains)
	{
		OK = dizel_GearShiftCheck(VelUp, VelDown);
	}

	if (Mains)
	{
		dizel_ClutchCheck();
	}
	return OK;
}

// the automatic gearbox: a gear up over VelUp, down under VelDown, changed once the clutch is out; returns whether it acted
bool TMoverParameters::dizel_GearShiftCheck(double const VelUp, double const VelDown)
{
	auto OK{false};
	if (RList[MainCtrlPos].Mn == 0 && !hydro_TC)
	{
		if (dizel_engagestate > 0)
			dizel_EngageSwitch(0);
		if (IsMainCtrlNoPowerPos() && ScndCtrlActualPos > 0)
			dizel_automaticgearstatus = -1;
	}
	else
	{
		OK = dizel_GearShiftRequest(VelUp, VelDown);
	}
	if (dizel_engage < 0.1 && dizel_automaticgearstatus != 0)
	{
		if (dizel_automaticgearstatus == 1)
			ScndCtrlActualPos++;
		else
			ScndCtrlActualPos--;
		dizel_automaticgearstatus = 0;
		dizel_EngageSwitch(1.0);
		OK = true;
	}
	return OK;
}

// a gear up asked over VelUp, down under VelDown, where the gearbox shifts by itself; returns whether one is
bool TMoverParameters::dizel_GearShiftRequest(double const VelUp, double const VelDown)
{
	auto OK{false};
	if (motor_param(ScndCtrlActualPos).AutoSwitch && dizel_automaticgearstatus == 0) // sprawdz czy zmienic biegi
	{
		if (Vel > VelUp)
		{
			// shift up
			if (ScndCtrlActualPos < ScndCtrlPosNo)
			{
				dizel_automaticgearstatus = 1;
				OK = true;
			}
		}
		else if (Vel < VelDown)
		{
			// shift down
			if (ScndCtrlActualPos > 0)
			{
				dizel_automaticgearstatus = -1;
				OK = true;
			}
		}
	}
	return OK;
}

// the pressure in the clutch's cylinder, by the controller's position or the integrated controller
void TMoverParameters::dizel_ClutchCheck()
{
	if (EIMCtrlType > 0) // sterowanie komputerowe
	{
		dizel_ClutchByIntegratedController();
	}
	else if (dizel_automaticgearstatus == 0) // ustaw cisnienie w silowniku sprzegla}
		dizel_ClutchByController();
	else
		dizel_EngageSwitch(0.0);
	if (!(motor_param(ScndCtrlActualPos).mIsat > 0))
		dizel_EngageSwitch(0.0); // wylacz sprzeglo na pozycjach neutralnych
	if (!AutoRelayFlag)
		ScndCtrlActualPos = ScndCtrlPos;
}

// the clutch by the controller's position: half or fully in, fully over the speed of the full engagement
void TMoverParameters::dizel_ClutchByController()
{
	switch (RList[MainCtrlPos].Mn)
	{
	case 1:
		dizel_EngageSwitch(0.5);
		break;
	case 2:
		dizel_EngageSwitch(1.0);
		break;
	case 3:
		if (Vel > dizel_minVelfullengage)
			dizel_EngageSwitch(1.0);
		else
			dizel_EngageSwitch(0.5);
		break;
	case 4:
		if (Vel > dizel_minVelfullengage)
			dizel_EngageSwitch(1.0);
		else
			dizel_EngageSwitch(0.66);
		break;
	case 5:
		if (Vel > dizel_minVelfullengage)
			dizel_EngageSwitch(1.0);
		else
			dizel_EngageSwitch(0.35 * (1 + RList[MainCtrlPos].R) * RList[MainCtrlPos].R);
		break;
	default:
		if (hydro_TC && hydro_TC_Fill > 0.01)
			dizel_EngageSwitch(1.0);
		else
			dizel_EngageSwitch(0.0);
	}
}

// the clutch of the integrated controller: in with the converter filled or the power asked, half in for the retarder
void TMoverParameters::dizel_ClutchByIntegratedController()
{
	if (dizel_automaticgearstatus == 0)
	{
		if ((hydro_TC && hydro_TC_Fill > 0.01) || eimic_real > 0.005)
			dizel_EngageSwitch(1.0);
		else if (Vel > hydro_R_EngageVel && hydro_R && hydro_R_Fill > 0.01)
			dizel_EngageSwitch(0.5);
		else
			dizel_EngageSwitch(0.0);
	}
	else
		dizel_EngageSwitch(0.0);
}

// performs diesel engine startup procedure; potentially clears startup switch; returns: true if the engine can be started, false otherwise
bool TMoverParameters::dizel_StartupCheck()
{

	auto engineisready{true}; // make inital optimistic presumption, then watch the reality crush it

	// test the fuel pump
	// TODO: add fuel pressure check
	if (false == FuelPump.is_active || (EngineType == TEngineType::DieselEngine && RList[MainCtrlPos].R == 0.0))
	{
		engineisready = false;
		//        if( FuelPump.start_type == start_t::manual ) {
		// with manual pump control startup procedure is done only once per starter switch press
		dizel_startup = false;
		//        }
	}
	// test the oil pump
	if (false == OilPump.is_active || OilPump.pressure < OilPump.pressure_minimum)
	{
		engineisready = false;
		if (OilPump.start_type == start_t::manual)
		{
			// with manual pump control startup procedure is done only once per starter switch press
			dizel_startup = false;
		}
	}
	// test the water circuits and water temperature
	if (true == dizel_heat.PA)
	{
		engineisready = false;
		// TBD, TODO: reset startup procedure depending on pump and heater control mode
		dizel_startup = false;
	}

	return engineisready;
}

// *************************************************************************************************
// Q: 20160715
// Aktualizacja stanu silnika
// *************************************************************************************************
bool TMoverParameters::dizel_Update(double dt)
{

	WaterPumpCheck(dt);
	WaterHeaterCheck(dt);
	OilPumpCheck(dt);
	FuelPumpCheck(dt);
	// the engine being started fires once the starter has turned it a while; one stopped meanwhile starts over
	dizel_FiringCountdown = (Mains ? std::max(0.0, dizel_FiringCountdown - dt) : 0.0);

	if (true == dizel_startup && true == dizel_StartupCheck())
	{
		dizel_ignition = true;
	}

	if (true == dizel_ignition && LastSwitchingTime >= InitialCtrlDelay)
	{

		dizel_startup = false;
		dizel_ignition = false;
		// TODO: split engine and main circuit state indicator in two separate flags
		LastSwitchingTime = 0;
		Timer::subsystem.switching.note(Timer::switching_counter::diesel, this, &TypeName, &Name);
		Mains = true;
		dizel_spinup = true;
		dizel_FiringCountdown = dizel_SmokeDelay;
		enrot = std::max(enrot,
		                 0.35 * ( // TODO: dac zaleznie od temperatury i baterii
		                            EngineType == TEngineType::DieselEngine ? dizel_nmin : DElist[0].RPM / 60.0));
	}

	dizel_spinup = dizel_spinup && Mains && enrot < 0.95 * (EngineType == TEngineType::DieselEngine ? dizel_nmin : DElist[0].RPM / 60.0);

	if (true == Mains && false == FuelPump.is_active)
	{
		// knock out the engine if the fuel pump isn't feeding it
		// TBD, TODO: grace period before the engine is starved for fuel and knocked out
		// an engine started on its own stops on its own
		MainSwitch(false, MainSwitchRange());
	}

	bool DU{false};

	if (EngineType == TEngineType::DieselEngine)
	{
		dizel_EngageChange(dt);
		DU = dizel_AutoGearCheck();
		double const fillspeed{2};
		dizel_fill = dizel_fill + fillspeed * dt * (dizel_fillcheck(MainCtrlPos, dt) - dizel_fill);
	}

	dizel_Heat(dt);

	return DU;
}

// *************************************************************************************************
// Q: 20160715
// oblicza napelnienie, uzwglednia regulator obrotow
// *************************************************************************************************
double TMoverParameters::dizel_fillcheck(int mcp, double dt)
{
	auto realfill{0.0};

	if (true == Mains && MainCtrlPosNo > 0 && true == FuelPump.is_active)
	{

		if (true == dizel_ignition && LastSwitchingTime >= 0.9 * InitialCtrlDelay)
		{
			// wzbogacenie przy rozruchu
			// NOTE: ignition flag is reset before this code is executed
			// TODO: sort this out
			realfill = 1;
		}
		else
		{
			// napelnienie zalezne od MainCtrlPos
			if (EIMCtrlType > 0)
			{
				realfill = dizel_DigitalFill(dt);
			}
			else
			{
				dizel_nreg_max = dizel_nmax;
				realfill = RList[mcp].R;
			}
		}
		if (dizel_nmax_cutoff > 0)
		{
			realfill = dizel_GovernorFill(mcp, realfill);
		}
	}

	return std::clamp(realfill, 0.0, 1.0);
}

// the fill set by the integrated controller, and the idle and top revolutions the governor keeps to
double TMoverParameters::dizel_DigitalFill(double dt)
{
	double realfill;
		realfill = std::max(0.0, std::min(eimic_real, 1 - motor_param(ScndCtrlActualPos).Isat));
		if (eimic_real > 0.005 && !hydro_TC_Lockup)
		{
			dizel_nreg_min = std::min(dizel_nreg_min + 2.5 * dt, dizel_nmin_hdrive + eimic_real * dizel_nmin_hdrive_factor);
		}
		else
		{
			if (Vel < hydro_R_EngageVel && hydro_R && hydro_R_Fill > 0.01)
				dizel_nreg_min = std::min(dizel_nreg_min + 5.0 * dt, dizel_nmin_retarder);
			else
				dizel_nreg_min = dizel_nmin;
		}
		if (!dizel_vel2nmax_Table.empty() && !hydro_TC_Lockup)
		{
			dizel_nreg_max = std::min(std::min(dizel_nreg_max, enrot) + dizel_nreg_acc * dt, TableInterpolation(dizel_vel2nmax_Table, Vel));
		}
		else
		{
			dizel_nreg_max = dizel_nmax;
		}
	return realfill;
}

// the fill cut off over the revolutions the governor allows at the controller's position, or full under the idle ones
double TMoverParameters::dizel_GovernorFill(int const mcp, double realfill) const
{
	auto nreg{0.0};
	if (EIMCtrlType > 0)
		nreg = eimic_real > 0.005 ? dizel_nreg_max : dizel_nmin;
	else
		nreg = dizel_GovernorRevolutions(mcp, realfill);
	if (enrot > nreg) // nad predkoscia regulatora zeruj dawke
		realfill = 0;
	if (enrot < dizel_nreg_min && RList[mcp].R > 0.001) // jesli ponizej biegu jalowego i niezerowa dawka, to dawaj pelna
		realfill = 1;
	return realfill;
}

// the revolutions the governor allows at the controller's position, the gearbox and the speed; the fill cut off at
// a position the governor doesn't know
double TMoverParameters::dizel_GovernorRevolutions(int const mcp, double &realfill) const
{
	auto nreg{0.0};
	switch (RList[MainCtrlPos].Mn)
	{
	case 0:
	case 1:
		nreg = dizel_nmin;
		break;
	case 2:
		if (dizel_automaticgearstatus == 0 && true /*(!hydro_TC) || (dizel_engage>dizel_fill)*/)
			nreg = dizel_nreg_max;
		else
			nreg = dizel_nmin;
		break;
	case 3:
		if (dizel_automaticgearstatus == 0 && Vel > dizel_minVelfullengage)
			nreg = dizel_nreg_max;
		else
			nreg = dizel_nmin;
		break;
	case 4:
		if (dizel_automaticgearstatus == 0 && Vel > dizel_minVelfullengage)
			nreg = dizel_nmax;
		else
			nreg = dizel_nmin * 0.75 + dizel_nreg_max * 0.25;
		break;
	case 5:
		if (Vel > dizel_minVelfullengage)
			nreg = dizel_nreg_max;
		else
			nreg = dizel_nmin + 0.8 * (dizel_nreg_max - dizel_nmin) * RList[mcp].R;
		break;
	default:
		realfill = 0; // sluczaj
		break;
	}
	return nreg;
}

// *************************************************************************************************
// Q: 20160715
// Oblicza moment siły wytwarzany przez silnik spalinowy
// *************************************************************************************************
double TMoverParameters::dizel_Momentum(double Fill, double n, double dt)
{ // liczy moment sily wytwarzany przez silnik spalinowy}
	double Moment = 0;
	double enMoment = 0;
	double gearMoment = 0;
	double eps = 0;
	double newn = 0;
	double friction = 0;
	double TorqueH = 0;
	double TorqueL = 0;
	double TorqueC = 0;
	n = n * CabActive;
	if (motor_param(ScndCtrlActualPos).mIsat < 0.001 || DirActive == 0)
		n = enrot;
	friction = dizel_engagefriction;
	hydro_TC_nIn = enrot; // wal wejsciowy przetwornika momentu
	hydro_TC_nOut = dizel_n_old; // wal wyjsciowy przetwornika momentu

	if (enrot > 0)
	{
		if (dizel_Momentum_Table.size() > 1)
		{
			Moment = TableInterpolation(dizel_Momentum_Table, enrot) * Fill - dizel_Mstand;
		}
		else
		{
			Moment = (dizel_Mmax - (dizel_Mmax - dizel_Mnmax) * square((enrot - dizel_nMmax) / (dizel_nMmax - dizel_nmax))) * Fill - dizel_Mstand;
		}
		Mm = Moment;
		dizel_FuelConsumptionActual = dizel_FuelConsumption * enrot * Fill;
		dizel_FuelConsumptedTotal += dizel_FuelConsumptionActual * dt / 3600.0;
		if (hydro_R && hydro_R_Placement == 2)
			Moment -= dizel_MomentumRetarder(enrot, dt);
	}
	else
	{
		Moment = -dizel_Mstand;
	}
	if (enrot < dizel_nmin / 10.0 && eAngle < M_PI_2)
	{
		// wstrzymywanie przy malych obrotach
		Moment -= dizel_Mstand;
	}
	if (true == dizel_spinup)
		Moment += dizel_Mstand / (0.3 + std::max(0.0, enrot / dizel_nmin)); // rozrusznik

	dizel_Torque = Moment;

	hydro_TCFillCheck(dt);

	// obliczanie momentow poszczegolnych sprzegiel
	// sprzeglo glowne (skrzynia biegow)
	TorqueC = dizel_engageMaxForce * dizel_engage * dizel_engageDia * friction;

	if (hydro_TC) // jesli hydro
	{
		hydro_TCTorque();
		TorqueH = hydro_TC_TorqueOut;
		TorqueL = hydro_TC_LockupTorque * hydro_TC_LockupRate;
	}
	else
	{
		TorqueH = 0; // brak przetwornika oznacza brak momentu
		TorqueL = 1 + TorqueC * 2; // zabezpieczenie, polaczenie trwale
	}

	// sprawdzanie dociskow poszczegolnych sprzegiel
	if (abs(Moment) > std::min(TorqueC, TorqueL + abs(hydro_TC_TorqueIn)) || abs(dizel_n_old - enrot) > 0.1) // slizga sie z powodu roznic predkosci albo przekroczenia momentu
	{
		dizel_engagedeltaomega = enrot - dizel_n_old;

		if (TorqueC > TorqueL)
		{
			if (TorqueC > TorqueL + abs(TorqueH))
			{
				hydro_TC_nOut = n;
				gearMoment = TorqueL + abs(TorqueH) * sign(dizel_engagedeltaomega);
				enMoment = Moment - (TorqueL + abs(hydro_TC_TorqueIn)) * sign(dizel_engagedeltaomega);
			}
			else
			{
				hydro_TC_nOut = enrot - (n - enrot) * (TorqueC - TorqueL) / TorqueH; // slizganie proporcjonalne, zeby przetwornik nadrabial
				gearMoment = TorqueC * sign(dizel_engagedeltaomega);
				enMoment = Moment - gearMoment;
			}
		}
		else
		{
			hydro_TC_nOut = enrot;
			gearMoment = TorqueC*sign(dizel_engagedeltaomega);
			enMoment = Moment - gearMoment;
		}
		eps = enMoment / dizel_AIM;
		newn = enrot + eps * dt;
		if ((newn - n) * (enrot - dizel_n_old) < 0 && TorqueC > 0.1) // przejscie przez zero - slizgalo sie i przestało
			newn = n;
		if (newn * enrot <= 0 && eps * enrot < 0) // przejscie przez zero obrotow
			newn = 0;
		enrot = newn;
	}
	else // nie slizga sie (jeszcze)
	{
		dizel_engagedeltaomega = 0;
		gearMoment = Moment;
		double enrot_min = enrot - (std::min(TorqueC, TorqueL + abs(hydro_TC_TorqueIn)) - Moment) / dizel_AIM * dt;
		double enrot_max = enrot + (std::min(TorqueC, TorqueL + abs(hydro_TC_TorqueIn)) + Moment) / dizel_AIM * dt;
		enrot = safe_clamp(n, enrot_min, enrot_max);
	}
	if (hydro_R && hydro_R_Placement == 1)
		gearMoment -= dizel_MomentumRetarder(hydro_TC_nOut, dt);

	if (enrot <= 0 && false == dizel_spinup)
	{
		// an engine stalling stops those started together with it, once. one already stopped, from its own switch or
		// another vehicle's, or not started at all, has nothing to stop: it stays quiet, rather than stop every frame
		// the engines started on their own or from the separate switches (eu07/maszyna#80)
		if (true == Mains)
		{
			MainSwitch(false, MainSwitchRange());
		}
		enrot = 0;
	}

	dizel_n_old = n; // obecna predkosc katowa na potrzeby kolejnej klatki

	return gearMoment;
}

// the torque converter filled and emptied, and its lockup clutch engaged and released, by the power and the speed
void TMoverParameters::hydro_TCFillCheck(double dt)
{
	if (hydro_TC) // jesli przetwornik momentu
	{
		// napelnianie przetwornika
		bool IsPower = EIMCtrlType > 0 ? eimic_real > 0.005 : MainCtrlPowerPos() > 0;
		if (IsPower && Mains && enrot > dizel_nmin * 0.9)
			hydro_TC_Fill += hydro_TC_FillRateInc * dt;
		// oproznianie przetwornika
		if ((!IsPower && Vel < dizel_maxVelANS) || !Mains || enrot < dizel_nmin * 0.8)
			hydro_TC_Fill -= hydro_TC_FillRateDec * dt;
		// obcinanie zakresu
		hydro_TC_Fill = std::clamp(hydro_TC_Fill, 0.0, 1.0);

		// blokowanie sprzegla blokującego
		if (Vel > hydro_TC_LockupSpeed && Mains && enrot > 0.9 * dizel_nmin && IsPower)
		{
			hydro_TC_Lockup = true;
			hydro_TC_LockupRate += hydro_TC_FillRateInc * dt;
		}
		// luzowanie sprzegla blokujacego
		if (Vel < (IsPower ? hydro_TC_LockupSpeed : hydro_TC_UnlockSpeed) || !Mains || enrot < 0.8 * dizel_nmin)
		{
			hydro_TC_Lockup = false;
			hydro_TC_LockupRate -= hydro_TC_FillRateDec * dt;
		}
		// obcinanie zakresu
		hydro_TC_LockupRate = std::clamp(hydro_TC_LockupRate, 0.0, 1.0);
	}
	else
	{
		hydro_TC_Fill = 0.0;
		hydro_TC_LockupRate = 0.0;
	}
}

// the torque converter's moments on its input and output shafts, by their revolutions and its fill
void TMoverParameters::hydro_TCTorque()
{
	double HydroTorque = 0;
	HydroTorque += hydro_TC_nIn * hydro_TC_nIn * hydro_TC_TorqueInIn;
	HydroTorque += (hydro_TC_nIn - hydro_TC_nOut) * hydro_TC_TorqueInOut;
	HydroTorque += hydro_TC_nOut * hydro_TC_nOut * hydro_TC_TorqueOutOut;
	double nOut2In = hydro_TC_nOut / std::max(0.01, hydro_TC_nIn);
	if (hydro_TC_Table.size() > 1)
	{
		hydro_TC_TMRatio = TableInterpolation(hydro_TC_Table, nOut2In);
		hydro_TC_TorqueOut = HydroTorque * hydro_TC_Fill * hydro_TC_TMRatio;
		hydro_TC_TorqueIn = HydroTorque * hydro_TC_Fill * std::min(1.0, hydro_TC_TMRatio);
	}
	else
	{
		if (nOut2In < hydro_TC_CouplingPoint)
		{
			hydro_TC_TMRatio = 1 + (hydro_TC_TMMax - 1) * square(1 - nOut2In / hydro_TC_CouplingPoint);
			hydro_TC_TorqueIn = HydroTorque * hydro_TC_Fill;
			hydro_TC_TorqueOut = HydroTorque * hydro_TC_Fill * hydro_TC_TMRatio;
		}
		else
		{
			hydro_TC_TMRatio = (1 - nOut2In) / (1 - hydro_TC_CouplingPoint);
			hydro_TC_TorqueIn = HydroTorque * hydro_TC_Fill * hydro_TC_TMRatio;
			hydro_TC_TorqueOut = HydroTorque * hydro_TC_Fill * hydro_TC_TMRatio;
		}
	}
}

double TMoverParameters::dizel_MomentumRetarder(double n, double dt)
{
	double RetarderRequest = Mains ? std::max(0.0, -eimic_real) : 0;
	if (hydro_R_WithIndividual)
		RetarderRequest = LocalBrakeRatio();
	if (Vel < hydro_R_MinVel)
		RetarderRequest = 0;
	if (hydro_R_Placement == 2 && enrot < dizel_nmin)
	{
		RetarderRequest = 0;
	}

	hydro_R_ClutchActive = !hydro_R_Clutch || RetarderRequest > 0;
	if (!hydro_R_Clutch || (hydro_R_ClutchActive && hydro_R_ClutchSpeed == 0))
	{
		hydro_R_n = n * 60;
	}
	else if (hydro_R_ClutchActive)
	{
		hydro_R_n = sign(n) * std::min(std::abs(hydro_R_n + hydro_R_ClutchSpeed * dt), std::abs(n * 60));
	}
	else
	{
		hydro_R_n = 0;
	}
	n = hydro_R_n / 60.f;

	if (hydro_R_Fill < RetarderRequest) // gdy zadane hamowanie
	{
		hydro_R_Fill = std::min(hydro_R_Fill + hydro_R_FillRateInc * dt, RetarderRequest);
	}
	else
	{
		hydro_R_Fill = std::max(hydro_R_Fill - hydro_R_FillRateDec * dt, RetarderRequest);
	}

	double Moment = hydro_R_MaxTorque;
	if (double pwr = Moment * std::abs(n) * M_PI * 2 * 0.001; pwr > hydro_R_MaxPower)
		Moment = Moment * hydro_R_MaxPower / pwr;
	double moment_in = n * n * hydro_R_TorqueInIn;
	Moment = std::min(moment_in, Moment * hydro_R_Fill);

	hydro_R_Torque = Moment;

	return Moment;
}

// sets component temperatures to specified value
void TMoverParameters::dizel_HeatSet(float const Value)
{

	// TODO: don't include ambient temperature, pull it from environment data instead
	dizel_heat.Te = Value;
	dizel_heat.Ts = Value;
	dizel_heat.To = Value;
	dizel_heat.Tsr = Value;
	dizel_heat.Twy = Value;
	dizel_heat.Tsr2 = Value;
	dizel_heat.Twy2 = Value;
	dizel_heat.temperatura1 = Value;
	dizel_heat.temperatura2 = Value;
}

// calculates diesel engine temperature and heat transfers
// adapted from scripts written by adamst
// NOTE: originally executed twice per second
void TMoverParameters::dizel_Heat(double const dt)
{

	auto const qs{44700.0};
	auto const Cs{11000.0};
	auto const Cw{4.189};
	auto const Co{1.885};
	auto const gwmin{400.0};
	auto const gwmax{4000.0};
	auto const gwmin2{400.0};
	auto const gwmax2{4000.0};

	dizel_heat.Te = Global.AirTemperature;

	auto const engineon{(Mains ? 1 : 0)};
	auto const engineoff{(Mains ? 0 : 1)};
	auto const rpm{enrot * 60};
	// TODO: calculate this once and cache for further use, instead of doing it repeatedly all over the place
	auto const revolutionsfactor{EngineRPMRatio()};
	auto const waterpump{WaterPump.is_active ? 1 : 0};

	auto const gw = engineon * std::lerp(gwmin, gwmax, revolutionsfactor) + waterpump * 1000 + engineoff * 200;
	auto const gw2 = engineon * std::lerp(gwmin2, gwmax2, revolutionsfactor) + waterpump * 1000 + engineoff * 200;
	auto const gwO = std::lerp(gwmin, gwmax, revolutionsfactor);

	dizel_heat.water.is_cold = dizel_heat.water.config.temp_min > 0 && dizel_heat.temperatura1 < dizel_heat.water.config.temp_min - (Mains ? 5 : 0);
	dizel_heat.water.is_hot = dizel_heat.water.config.temp_max > 0 && dizel_heat.temperatura1 > dizel_heat.water.config.temp_max - (dizel_heat.water.is_hot ? 8 : 0);
	dizel_heat.water_aux.is_cold = dizel_heat.water_aux.config.temp_min > 0 && dizel_heat.temperatura2 < dizel_heat.water_aux.config.temp_min - (Mains ? 5 : 0);
	dizel_heat.water_aux.is_hot = dizel_heat.water_aux.config.temp_max > 0 && dizel_heat.temperatura2 > dizel_heat.water_aux.config.temp_max - (dizel_heat.water_aux.is_hot ? 8 : 0);
	dizel_heat.oil.is_cold = dizel_heat.oil.config.temp_min > 0 && dizel_heat.To < dizel_heat.oil.config.temp_min - (Mains ? 5 : 0);
	dizel_heat.oil.is_hot = dizel_heat.oil.config.temp_max > 0 && dizel_heat.To > dizel_heat.oil.config.temp_max - (dizel_heat.oil.is_hot ? 8 : 0);
	// engine overheat check
	dizel_heat.engine_is_hot = dizel_heat.engine_max_temp > 0 && dizel_heat.Ts > dizel_heat.engine_max_temp - (dizel_heat.engine_is_hot ? 8 : 0);

	auto const PT = false == dizel_heat.water.is_cold && false == dizel_heat.water.is_hot && false == dizel_heat.water_aux.is_cold && false == dizel_heat.water_aux.is_hot &&
	                false == dizel_heat.oil.is_cold && false == dizel_heat.oil.is_hot && false == dizel_heat.engine.is_hot /* && ( false == awaria_termostatow ) */
	    /* || PTp */;
	auto const PPT = false == PT /* && ( false == PPTp ) */;
	dizel_heat.PA = /* ( ( !zamkniecie or niedomkniecie ) and !WBD ) || */ PPT /* || nurnik || ( woda < 7 ) */;

	// engine heat transfers
	auto const Ge{engineon * (0.21 * dizel_heat.powerfactor * EnginePower + 12) / 3600};
	// TODO: replace fixed heating power cost with more accurate calculation
	auto const obciazenie{engineon * (dizel_heat.powerfactor * EnginePower / 950 + (Heating ? HeatingPower : 0) + 70)};
	auto const Qd{qs * Ge - obciazenie};
	// silnik oddaje czesc ciepla do wody chlodzacej, a takze pewna niewielka czesc do otoczenia, modyfikowane przez okienko
	auto const Qs{(Qd - dizel_heat.kfs * (dizel_heat.Ts - dizel_heat.Tsr) - dizel_heat.kfe * /* ( 0.3 + 0.7 * ( dizel_heat.okienko ? 1 : 0 ) ) * */ (dizel_heat.Ts - dizel_heat.Te))};
	auto const dTss{Qs / Cs};
	dizel_heat.Ts += static_cast<float>(dTss * dt);

	// oil heat transfers
	// olej oddaje cieplo do wody gdy krazy przez wymiennik ciepla == wlaczona pompka lub silnik
	auto const dTo{(dizel_heat.auxiliary_water_circuit ? (dizel_heat.kfo * (dizel_heat.Ts - dizel_heat.To) - dizel_heat.kfo2 * (dizel_heat.To - dizel_heat.Tsr2)) / (gwO * Co) :
	                                                     (dizel_heat.kfo * (dizel_heat.Ts - dizel_heat.To) - dizel_heat.kfo2 * (dizel_heat.To - dizel_heat.Tsr)) / (gwO * Co))};
	dizel_heat.To += static_cast<float>(dTo * dt);

	// heater
	/*
	    if( typ == "SP45" )
	        Qp = (float)( podgrzewacz and ( true == WaterPump.is_active ) and ( Twy < 55 ) and ( Twy2 < 55 ) ) * 1000;
	    else
	*/
	auto const Qp = (true == WaterHeater.is_active && true == WaterPump.is_active && dizel_heat.Twy < 60 && dizel_heat.Twy2 < 60 ? 1 : 0) * 1000;

	auto const kurek07{1}; // unknown/unimplemented device TBD, TODO: identify and implement?

	if (true == dizel_heat.auxiliary_water_circuit)
	{
		// auxiliary water circuit setup
		dizel_heat.water_aux.is_warm = true == dizel_heat.cooling ||
		                               (true == Mains && BatteryVoltage > 0.75 * NominalBatteryVoltage /* && !bezpompy && !awaria_chlodzenia && !WS10 */
		                                && dizel_heat.water_aux.config.temp_cooling > 0 && dizel_heat.temperatura2 > dizel_heat.water_aux.config.temp_cooling - (dizel_heat.water_aux.is_warm ? 8 : 0));
		auto const PTC2{(dizel_heat.water_aux.is_warm /*or PTC2p*/ ? 1 : 0)};
		dizel_heat.rpmwz2 = static_cast<float>(PTC2 * (dizel_heat.fan_speed >= 0 ? rpm * dizel_heat.fan_speed : dizel_heat.fan_speed * -1));
		dizel_heat.zaluzje2 = dizel_heat.water_aux.config.shutters ? PTC2 == 1 : true; // no shutters is an equivalent to having them open
		auto const zaluzje2{(dizel_heat.zaluzje2 ? 1 : 0)};
		// auxiliary water circuit heat transfer values
		auto const kf2{kurek07 * (dizel_heat.kw * (0.3 + 0.7 * zaluzje2) * dizel_heat.rpmw2 + dizel_heat.kv * (0.3 + 0.7 * zaluzje2) * Vel / 3.6) + 2};
		auto const dTs2{dizel_heat.kfo2 * (dizel_heat.To - dizel_heat.Tsr2) / (gw2 * Cw)};
		// przy otwartym kurku B ma³y obieg jest dogrzewany przez du¿y - stosujemy przy korzystaniu z podgrzewacza oraz w zimie
		auto const Qch2{-kf2 * (dizel_heat.Tsr2 - dizel_heat.Te) + 80 * (true == WaterCircuitsLink ? 1 : 0) * (dizel_heat.Twy - dizel_heat.Tsr2)};
		auto const dTch2{Qch2 / (gw2 * Cw)};
		// auxiliary water circuit heat transfers finalization
		// NOTE: since primary circuit doesn't read data from the auxiliary one, we can pretty safely finalize auxiliary updates before touching the primary circuit
		auto const Twe2{dizel_heat.Twy2 + dTch2 * dt};
		dizel_heat.Twy2 = static_cast<float>(Twe2 + dTs2 * dt);
		dizel_heat.Tsr2 = static_cast<float>(0.5 * (dizel_heat.Twy2 + Twe2));
		dizel_heat.temperatura2 = dizel_heat.Twy2;
	}
	// primary water circuit setup
	dizel_heat.water.is_flowing = dizel_heat.water.config.temp_flow < 0 || dizel_heat.temperatura1 > dizel_heat.water.config.temp_flow - (dizel_heat.water.is_flowing ? 5 : 0);
	auto const obieg{(dizel_heat.water.is_flowing ? 1 : 0)};
	dizel_heat.water.is_warm =
	    true == dizel_heat.cooling || (true == Mains && BatteryVoltage > 0.75 * NominalBatteryVoltage /* && !bezpompy && !awaria_chlodzenia && !WS10 */
	                                   && dizel_heat.water.config.temp_cooling > 0 && dizel_heat.temperatura1 > dizel_heat.water.config.temp_cooling - (dizel_heat.water.is_warm ? 8 : 0));
	auto const PTC1{(dizel_heat.water.is_warm /*or PTC1p*/ ? 1 : 0)};
	dizel_heat.rpmwz = static_cast<float>(PTC1 * (dizel_heat.fan_speed >= 0 ? rpm * dizel_heat.fan_speed : dizel_heat.fan_speed * -1));
	dizel_heat.zaluzje1 = dizel_heat.water.config.shutters ? PTC1 == 1 : true; // no shutters is an equivalent to having them open
	auto const zaluzje1{(dizel_heat.zaluzje1 ? 1 : 0)};
	// primary water circuit heat transfer values
	auto const kf{obieg * kurek07 * (dizel_heat.kw * (0.3 + 0.7 * zaluzje1) * dizel_heat.rpmw + dizel_heat.kv * (0.3 + 0.7 * zaluzje1) * Vel / 3.6 + 3) + 2};
	auto const dTs{(dizel_heat.auxiliary_water_circuit ? dizel_heat.kfs * (dizel_heat.Ts - dizel_heat.Tsr) / (gw * Cw) :
	                                                     (dizel_heat.kfs * (dizel_heat.Ts - dizel_heat.Tsr) + dizel_heat.kfo2 * (dizel_heat.To - dizel_heat.Tsr)) / (gw * Cw))};
	auto const Qch{-kf * (dizel_heat.Tsr - dizel_heat.Te) + Qp};
	auto const dTch{Qch / (gw * Cw)};
	// primary water circuit heat transfers finalization
	auto const Twe{dizel_heat.Twy + dTch * dt};
	dizel_heat.Twy = static_cast<float>(Twe + dTs * dt);
	dizel_heat.Tsr = static_cast<float>(0.5 * (dizel_heat.Twy + Twe));
	dizel_heat.temperatura1 = dizel_heat.Twy;
	/*
	    fuelConsumed = fuelConsumed + ( Ge * 0.5 );

	    while( fuelConsumed >= 0.83 ) {
	        fuelConsumed = fuelConsumed - 0.83;
	        fuelQueue.DestroyProductMatching( null, 1 );
	    }//if

	    if( engineon )
	        temp_turbo = temp_turbo + 0.3 * ( t_pozycja );
	    if( t_pozycja == 0 and cisnienie > 0.04 )
	        temp_turbo = temp_turbo - 1;

	    if( temp_turbo > 400 )
	        temp_turbo = 400;
	    if( temp_turbo < 0 )
	        temp_turbo = 0;

	    if( temp_turbo > 50 and cisnienie < 0.05 )
	        timer_turbo = timer_turbo + 1;

	    if( temp_turbo == 0 )
	        timer_turbo = 0;

	    if( timer_turbo > 360 ) {
	        awaria_turbo = true;
	        timer_turbo = 400;
	    }

	    if( Ts < 50 )
	        p_odpal = 3;
	    if( Ts > 49 and Ts < 76 )
	        p_odpal = 4;
	    if( Ts > 75 )
	        p_odpal = 7;

	    stukanie = stukanie or awaria_oleju;

	    if( awaria_oleju == true and ilosc_oleju > 0 ) {
	        ilosc_oleju = ilosc_oleju - ( 0.002 * rpm / 1500 );
	    }
	    if( awaria_oleju == true and cisnienie < 0.06 )
	        damage = 1;
	*/
}
