/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the traction motors and their forces for each kind of drive, the relays, the fuses, the dynamic brake

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
#include "physics/components/drive.h"


double TableInterpolation(std::map<double, double> &Map, double Parameter)
{
	if (Map.empty())
		return 0.0;
	if (Map.size() == 1)
		return Map.begin()->second;

	auto lower = Map.lower_bound(Parameter);
	auto upper = lower;

	if (lower != Map.begin())
		lower--;
	else
		upper++;

	if (upper == Map.end())
	{
		lower--;
		upper--;
	}
	double ratio = (upper->second - lower->second) / (upper->first - lower->first);
	return lower->second + (Parameter - lower->first) * ratio;
}

// *************************************************************************************************
// Q: 20160716
// Obliczanie natężenie prądu w silnikach
// *************************************************************************************************
double TMoverParameters::Current(double n, double U)
{
	// wazna funkcja - liczy prad plynacy przez silniki polaczone szeregowo lub rownolegle
	// w zaleznosci od polozenia nastawnikow MainCtrl i ScndCtrl oraz predkosci obrotowej n
	// a takze wywala bezpiecznik nadmiarowy gdy za duzy prad lub za male napiecie
	// jest takze mozliwosc uszkodzenia silnika wskutek nietypowych parametrow

	double R;
	double MotorCurrent;
	double Mn; // przujmuje int, ale dla poprawnosci obliczeń
	double Bn;

	UpdateSeriesMotorDynamicBrakeFlag();

	ResistorsFlag = RList[MainCtrlActualPos].R > 0.01; // and (!DelayCtrlFlag)
	ResistorsFlag = ResistorsFlag || (DynamicBrakeFlag == true && DynamicBrakeType == dbrake_automatic);

	if (TrainType == train_type::et22 && DelayCtrlFlag && MainCtrlActualPos > 1)
		Bn = 1.0 - 1.0 / RList[MainCtrlActualPos].Bn;
	else
		Bn = 1; // to jest wykonywane dla EU07

	R = RList[MainCtrlActualPos].R * Bn + CircuitRes;

	if (TrainType != train_type::emu || Imin != IminLo || false == ScndS)
	{
		// yBARC - boczniki na szeregu poprawnie
		Mn = RList[MainCtrlActualPos].Mn; // to jest wykonywane dla EU07
	}
	else
	{
		Mn = RList[MainCtrlActualPos].Mn * RList[MainCtrlActualPos].Bn;
		if (RList[MainCtrlActualPos].Bn > 1)
		{
			R = CircuitRes;
		}
	}

	if (DynamicBrakeFlag && !FuseFlag && DynamicBrakeType == dbrake_automatic && Power110vIsAvailable && Mains) // hamowanie EP09   //TUHEX
	{
		// TODO: zrobic bardziej uniwersalne nie tylko dla EP09
		MotorCurrent = -std::max(motor_param(0).fi * (Vadd / (Vadd + motor_param(0).Isat) - motor_param(0).fi0), 0.) * n * 2.0 / DynamicBrakeRes;
	}
	else if (RList[MainCtrlActualPos].Bn == 0 || false == StLinFlag)
	{
		// wylaczone
		MotorCurrent = 0;
	}
	else
	{ // wlaczone...
		MotorCurrent = SeriesMotorCurrent(n, U, R, Mn);
	}
	//  writepaslog("MotorCurrent   ", FloatToStr(MotorCurrent));

	if (DynamicBrakeType == dbrake_switch && (BrakePress > 2.0 || PipePress < 3.6))
	{
		Im = 0;
		// Im:=0;
		Itot = 0;
	}
	else
		Im = MotorCurrent;

	EnginePower = abs(Itot) * (1 + RList[MainCtrlActualPos].Mn) * abs(U) / 1000.0;

	MotorFailuresCheck(n);

	return Im;
}

// the automatic dynamic brake of the series motors (EP09, the units) on or off, and the distributor told of it
void TMoverParameters::UpdateSeriesMotorDynamicBrakeFlag()
{
	// i dzialanie hamulca ED w EP09
	if (DynamicBrakeType == dbrake_automatic && TrainType != train_type::emu)
	{
		if ((Hamulec->GetEDBCP() < 0.25 && Vadd < 1) || BrakePress > 2.1)
			DynamicBrakeFlag = false;
		else if (BrakePress > 0.25 && Hamulec->GetEDBCP() > 0.25)
			DynamicBrakeFlag = true;
		DynamicBrakeFlag = DynamicBrakeFlag && Power110vIsAvailable;
	}
	if (DynamicBrakeType == dbrake_automatic && TrainType == train_type::emu)
	{
		DynamicBrakeFlag = Power110vIsAvailable && (TUHEX_Active || Vadd > TUHEX_MinIw) && DynamicBrakeEMUStatus;
	}

	// wylacznik cisnieniowy yBARC - to jest chyba niepotrzebne tutaj   Q: no to usuwam...

	// BrakeSubsystem = ss_LSt;
	// if (BrakeSubsystem == ss_LSt) WriteLog("LSt");
	// if (BrakeSubsystem == ss_LSt) // zrobiona funkcja virtualna
	if (DynamicBrakeFlag)
	{
		Hamulec->SetED(abs(Im / 350)); // hamulec ED na EP09 dziala az do zatrzymania lokomotywy
		//-     WriteLog("A");
	}
	else
	{
		Hamulec->SetED(0);
		//-     WriteLog("B");
	}
}

// the current of the series motors connected, at the revolutions n and the voltage U, through the resistance R of
// Mn motors in series
double TMoverParameters::SeriesMotorCurrent(double const n, double const U, double const R, double const Mn) const
{
	double MotorCurrent{0.0};
	double Rz;
	int SP = ScndCtrlActualPos;

	if (ScndCtrlActualPos < 255) // tak smiesznie bede wylaczal
	{
		if (ScndInMain && RList[MainCtrlActualPos].ScndAct != 255)
		{
			SP = RList[MainCtrlActualPos].ScndAct;
		}

		Rz = Mn * WindingRes + R;

		if (DynamicBrakeFlag) // hamowanie
		{
			// if DynamicBrakeType<>dbrake_automatic then
			// MotorCurrent:=-fi*n/Rz  {hamowanie silnikiem na oporach rozruchowych}
			/*               begin
			U:=0;
			Isf:=Isat;
			Delta:=SQR(Isf*Rz+Mn*fi*n-U)+4*U*Isf*Rz;
			MotorCurrent:=(U-Isf*Rz-Mn*fi*n+SQRT(Delta))/(2*Rz)
			end*/
			// only the ET42's switched brake brakes on the starting resistors, otherwise the current is cut off (0)
			if (DynamicBrakeType == dbrake_switch && TrainType == train_type::et42)
			{ // z Megapacka
				Rz = WindingRes + R;
				MotorCurrent = -motor_param(SP).fi * n / Rz; //{hamowanie silnikiem na oporach rozruchowych}
			}
		}
		else
		{
			MotorCurrent = SeriesMotorDrivingCurrent(n, U, Rz, Mn, SP);
		} // else DBF

	} // 255
	else
		MotorCurrent = 0;
	return MotorCurrent;
}

// the current of the series motors driving, at the revolutions n, the voltage U and the resistance Rz of Mn motors in
// series, by the motor parameters of the shunt field SP
double TMoverParameters::SeriesMotorDrivingCurrent(double const n, double const U, double const Rz, double const Mn, int const SP) const
{
	double MotorCurrent;
	double Delta;
	double Isf;
	double U1; // napiecie z korekta
	U1 = U + Mn * n * motor_param(SP).fi0 * motor_param(SP).fi;
	//          writepaslog("U1             ", FloatToStr(U1));
	//          writepaslog("Isat           ", FloatToStr(MotorParam[SP].Isat));
	//          writepaslog("fi             ", FloatToStr(MotorParam[SP].fi));
	Isf = Sign(U1) * motor_param(SP).Isat;
	//          writepaslog("Isf            ", FloatToStr(Isf));
	Delta = square(Isf * Rz + Mn * motor_param(SP).fi * n - U1) + 4.0 * U1 * Isf * Rz; // 105 * 1.67 + Mn * 140.9 * 20.532 - U1
	//          DeltaQ = Isf * Rz + Mn * MotorParam[SP].fi * n - U1 + 4 * U1 * Isf * Rz;
	//          writepaslog("Delta          ", FloatToStr(Delta));
	//          writepaslog("DeltaQ         ", FloatToStr(DeltaQ));
	//          writepaslog("U              ", FloatToStr(U));
	if (MainCircuitPowered())
	{
		if (U > 0)
			MotorCurrent = (U1 - Isf * Rz - Mn * motor_param(SP).fi * n + std::sqrt(Delta)) / (2.0 * Rz);
		else
			MotorCurrent = (U1 - Isf * Rz - Mn * motor_param(SP).fi * n - std::sqrt(Delta)) / (2.0 * Rz);
	}
	else
		MotorCurrent = 0;
	return MotorCurrent;
}

// the line breaker opened, and the motors damaged, by too high revolutions or current
void TMoverParameters::MotorFailuresCheck(double const n)
{
	double MotorCurrent;
	// awarie
	MotorCurrent = abs(Im); // zmienna pomocnicza

	if (MotorCurrent > 0)
	{
		if (FuzzyLogic(abs(n), nmax * 1.1, p_elengproblem) && MainSwitch(false))
			EventFlag = true; /*zbyt duze obroty - wywalanie wskutek ognia okreznego*/
		if (DamageFlag.HasAll(VehicleDamage::engine) && FuzzyLogic(MotorCurrent, (double)ImaxLo / 10.0, p_elengproblem) && MainSwitch(false))
			EventFlag = true; /*uszkodzony silnik (uplywy)*/
		/*       or FuzzyLogic(Abs(U/Mn),2*NominalVoltage,1)) then */ /*poprawic potem*/
		if ((FuzzyLogic(abs(Im), Imax * 2, p_elengproblem) || FuzzyLogic(abs(n), nmax * 1.11, p_elengproblem)) && DamageFlag.Set(VehicleDamage::engine))
			EventFlag = true;
		/*! dorobic grzanie oporow rozruchowych i silnika*/
	}
}

// rozruch wysoki (true) albo niski (false)
bool TMoverParameters::CurrentSwitch(bool const State)
{
	// TBD, TODO: split off shunt mode toggle into a separate command? It doesn't make much sense to have these two together like that
	// the diesels with a shunt mode have no motor overload relay threshold to switch, the switch sets their shunt mode alone.
	// MaxCurrentSwitch() takes any change of the setting, so asked first it took the first move of the switch (eu07/maszyna#497)
	if (true == ShuntModeAllow && (EngineType == TEngineType::DieselEngine || EngineType == TEngineType::DieselElectric))
	{
		// dla 2Ls150 przed ustawieniem kierunku, for SM42/SP42 at the controller's zero position
		if (EngineType == TEngineType::DieselEngine ? DirActive != 0 : false == IsMainCtrlNoPowerPos())
		{
			return false;
		}
		ShuntMode = State;
		return true;
	}

	if (MaxCurrentSwitch(State))
	{
		if (TrainType != train_type::emu)
		{
			MinCurrentSwitch(State);
		}
		return true;
	}

	return false;
};

bool TMoverParameters::IsMotorOverloadRelayHighThresholdOn() const
{

	return ImaxHi > ImaxLo && Imax > ImaxLo;
}

double TMoverParameters::ShowEngineRotation(int VehN) const
{ // Zwraca wartość prędkości obrotowej silnika wybranego pojazdu. Do 3 pojazdów (3×SN61).
	int b;
	switch (VehN)
	{ // numer obrotomierza
	case 1:
		return std::abs(enrot);
	case 2:
		for (b = 0; b <= 1; ++b)
			if (Couplers[b].CouplingFlag.HasAll(Coupling::control) && Couplers[b].Connected->Power > 0.01)
				return fabs(Couplers[b].Connected->enrot);
		break;
	case 3: // to nie uwzględnia ewentualnego odwrócenia pojazdu w środku
		for (b = 0; b <= 1; ++b)
			if (Couplers[b].CouplingFlag.HasAll(Coupling::control) && Couplers[b].Connected->Power > 0.01 && Couplers[b].Connected->Couplers[b].CouplingFlag.HasAll(Coupling::control) && Couplers[b].Connected->Couplers[b].Connected->Power > 0.01)
				return fabs(Couplers[b].Connected->Couplers[b].Connected->enrot);
		break;
	default:
		break;
	}
	return 0.0;
};

void TMoverParameters::MotorBlowersCheck(double const Timestep)
{
	// activation check
	for (auto &blower : MotorBlowers)
	{
		auto disable = blower.is_disabled;
		auto const start{(Vel >= blower.min_start_velocity && std::abs(Im) > 0.5)};
		auto const stop{(Vel < 0.5 && std::abs(Im) < 0.5)};
		if (blower.min_start_velocity >= 0)
		{
			if (stop)
			{
				blower.stop_timer += static_cast<float>(Timestep);
				if (blower.stop_timer > blower.sustain_time)
				{
					disable = true;
				}
			}
			else if (start)
			{
				blower.stop_timer = 0;
			}
			else
			{
				disable |= !blower.is_active;
			}
		}
		blower.is_active =
		    // TODO: bind properly power source when ld is in place
		    MotorBlowerPowerIsAvailable(blower) // power source
		    // breaker condition disabled until it's implemented in the class data
		    //         && ( true == blower.breaker )
		    && false == disable &&
		    (true == blower.is_active || (blower.stop_timer == 0.f // HACK: will be true for blower with exceeded start_velocity, and for one without start_velocity
		                                  && (blower.start_type == start_t::manual ? blower.is_enabled : true)));
	}
	// update
	for (auto &fan : MotorBlowers)
	{

		auto const revolutionstarget{(fan.is_active ? (fan.speed > 0.f ? fan.speed * static_cast<float>(enrot) * 60 : fan.speed * -1) : 0.f)};

		if (std::abs(fan.revolutions - revolutionstarget) < 0.01f)
		{
			fan.revolutions = revolutionstarget;
			continue;
		}
		if (revolutionstarget > 0.f)
		{
			auto const speedincreasecap{std::max(50.f, fan.speed * 0.05f * -1)}; // 5% of fixed revolution speed, or 50
			fan.revolutions += static_cast<float>(std::clamp(revolutionstarget - fan.revolutions, speedincreasecap * -2, speedincreasecap) * Timestep);
		}
		else
		{
			fan.revolutions *= static_cast<float>(std::max(0.0, 1.0 - Timestep));
		}
	}
}

bool TMoverParameters::MotorBlowerPowerIsAvailable(cooling_fan const &Blower) const
{
	switch (Blower.start_type)
	{
	case start_t::battery:
		return Power24vIsAvailable;
	case start_t::converter:
		return Power110vIsAvailable;
	default:
		return MainCircuitPowered();
	}
}

double TMoverParameters::ShowCurrent(int AmpN) const
{ // Odczyt poboru prądu na podanym amperomierzu
	return Drive().ShowCurrent(*this, AmpN);
};

bool TMoverParameters::MotorBlowersSwitch(bool State, end const Side, range_t const Notify)
{

	auto &fan{MotorBlowers[Side]};

	if (fan.start_type != start_t::manual && fan.start_type != start_t::manualwithautofallback)
	{
		// automatic device ignores 'manual' state commands
		return false;
	}

	bool const initialstate{fan.is_enabled};

	fan.is_enabled = State;

	if (Notify != range_t::local)
	{
		SendCtrlToNext(Side == end::front ? VehicleCommand::motorBlowersFrontSwitch : VehicleCommand::motorBlowersRearSwitch, fan.is_enabled ? 1 : 0, CabActive,
		               Notify == range_t::unit ? Coupling::control | Coupling::permanent : Coupling::control);
	}

	return fan.is_enabled != initialstate;
}

bool TMoverParameters::MotorBlowersSwitchOff(bool State, end const Side, range_t const Notify)
{

	auto &fan{MotorBlowers[Side]};

	if (fan.start_type != start_t::manual && fan.start_type != start_t::manualwithautofallback)
	{
		// automatic device ignores 'manual' state commands
		return false;
	}

	bool const initialstate{fan.is_disabled};

	fan.is_disabled = State;

	if (Notify != range_t::local)
	{
		SendCtrlToNext(Side == end::front ? VehicleCommand::motorBlowersFrontSwitchOff : VehicleCommand::motorBlowersRearSwitchOff, fan.is_disabled ? 1 : 0, CabActive,
		               Notify == range_t::unit ? Coupling::control | Coupling::permanent : Coupling::control);
	}

	return fan.is_disabled != initialstate;
}

// the traction selector of a dual-mode vehicle: moves only with the line breaker open, and only where a genset is
bool TMoverParameters::TractionModeSwitch(traction_mode const Mode, range_t const Notify)
{
	auto const initialstate{TractionMode};
	// refused with the line breaker closed, and then not passed on either, so the train stays on one source
	auto const accepted{false == Mains};

	if (Genset.fitted() && accepted)
	{
		TractionMode = Mode;
	}

	if (Notify != range_t::local && accepted)
	{
		SendCtrlToNext(VehicleCommand::tractionMode, Mode == traction_mode::genset ? 1 : 0, CabActive, Notify == range_t::unit ? Coupling::control | Coupling::permanent : Coupling::control);
	}

	return TractionMode != initialstate;
}

// *************************************************************************************************
// zwiększenie nastawy nastawnika hamulca elektrodynamicznego (DynamicBrakeCtrl)
// uzywane gdy SplitEDPneumaticBrake = true
// *************************************************************************************************
bool TMoverParameters::IncDynamicBrakeLevel(float const CtrlSpeed)
{
	if (false == SplitEDPneumaticBrake)
		return false;
	if (DynamicBrakeCtrlPosNo < 1)
		return false;
	if (DynamicBrakeCtrlPos < 1.0)
	{
		DynamicBrakeCtrlPos = std::min(1.0, DynamicBrakeCtrlPos + (double)CtrlSpeed / (double)DynamicBrakeCtrlPosNo);
		return true;
	}
	return false;
}

// *************************************************************************************************
// zmniejszenie nastawy nastawnika hamulca elektrodynamicznego (DynamicBrakeCtrl)
// *************************************************************************************************
bool TMoverParameters::DecDynamicBrakeLevel(float const CtrlSpeed)
{
	if (false == SplitEDPneumaticBrake)
		return false;
	if (DynamicBrakeCtrlPosNo < 1)
		return false;
	if (DynamicBrakeCtrlPos > 0.0)
	{
		DynamicBrakeCtrlPos = std::max(0.0, DynamicBrakeCtrlPos - (double)CtrlSpeed / (double)DynamicBrakeCtrlPosNo);
		return true;
	}
	return false;
}

// *************************************************************************************************
// bezposrednie ustawienie nastawnika hamulca elektrodynamicznego (DynamicBrakeCtrl)
// *************************************************************************************************
bool TMoverParameters::DynamicBrakeLevelSet(double Position)
{
	if (false == SplitEDPneumaticBrake)
		return false;
	DynamicBrakeCtrlPos = std::clamp(Position, 0.0, 1.0);
	return true;
}

// *************************************************************************************************
// odczyt nastawy nastawnika hamulca elektrodynamicznego (DynamicBrakeCtrl)
// zwraca 0 jezeli SplitEDPneumaticBrake nie jest aktywne (dla zachowania starej logiki)
// *************************************************************************************************
double TMoverParameters::DynamicBrakeRatio(void) const
{
	if (false == SplitEDPneumaticBrake)
		return 0.0;
	return std::clamp(DynamicBrakeCtrlPos, 0.0, 1.0);
}

// *************************************************************************************************
// czy hamulec ED jest aktualnie dostepny w oknie predkosci Vh0..Vh1
// *************************************************************************************************
bool TMoverParameters::DynamicBrakeAvailable(void) const
{
	double const vh0{inductionMotor.config[eimc_p_Vh0]};
	if (double const vh1{inductionMotor.config[eimc_p_Vh1]}; vh1 <= 0.001)
	{
		// brak zdefiniowanej strefy - ED dziala zawsze
		return true;
	}
	// strefa wylaczenia jest definiowana przez Vh0 (minimum); powyzej Vh0 ED dziala
	return Vel >= vh0;
}

// *************************************************************************************************
// Q: 20160713
// reczne przelaczanie hamulca elektrodynamicznego
// *************************************************************************************************
bool TMoverParameters::DynamicBrakeSwitch(bool Switch)
{
	bool DBS;

	if (DynamicBrakeType == dbrake_switch && IsMainCtrlNoPowerPos())
	{
		DynamicBrakeFlag = Switch;
		DBS = true;
		for (auto &thiscoupler : Couplers)
			//  with Couplers[b] do
			if (thiscoupler.CouplingFlag.HasAll(Coupling::control))
				thiscoupler.Connected->DynamicBrakeFlag = Switch;
		// end;
		// if (DynamicBrakeType=dbrake_passive) and (TrainType=train_type::et42) then
		// begin
		// DynamicBrakeFlag:=false;
		// DynamicBrakeSwitch:=false;
	}
	else
		DBS = false;

	return DBS;
}

// *************************************************************************************************
// Q: 20160714
// oblicza sile trakcyjna lokomotywy (dla elektrowozu tez calkowity prad)
// *************************************************************************************************
double TMoverParameters::TractionForce(double dt)
{
	Ft = 0;
	double dtrans{0.0};
	double dmoment{0.0};
	auto const revolutions{UpdateEngineRevolutions(dt, dtrans, dmoment)};

	eAngle += enrot * dt;
	if (eAngle > M_PI * 2.0)
		eAngle = std::fmod(eAngle, M_PI * 2.0);
	/*
	    while (eAngle > M_PI * 2.0)
	        // eAngle = Pirazy2 - eAngle; <- ABu: a nie czasem tak, jak nizej?
	        eAngle -= M_PI * 2.0;
	*/
	// hunter-091012: przeniesione z if DirActive<>0 (zeby po zejsciu z kierunku dalej spadala predkosc wentylatorow)
	// wentylatory rozruchowe
	// TBD, TODO: move this to update, it doesn't exactly have much to do with traction
	auto const &drive{Drive()};
	drive.UpdateVentilators(*this, dt);
	auto const PosRatio{drive.UpdateEnginePower(*this)};
	VoltageRelaysCheck();

	drive.BeforeTractionForce(*this, dt);
	if (DirActive != 0)
	{
		drive.UpdateTractionForce(*this, DriveStep{dt, PosRatio, revolutions, dmoment, dtrans});
	}
	drive.AfterTractionForce(*this);

	return Ft;
}

// the starting resistors' blowers, and the diesels' radiator fans
void TMoverParameters::UpdateVentilators(double dt)
{
	Drive().UpdateVentilators(*this, dt);
}

// the starting resistors' blowers of the series motors with the line breaker closed, worked by hand or by the current
void TMoverParameters::UpdateResistorBlowers(double dt)
{
	switch (RVentType)
	{

	case 1:
	{ // manual
		if (true == RVentForceOn || (DirActive != 0 && RList[MainCtrlActualPos].R > RVentCutOff))
		{
			RventRot += (RVentnmax - RventRot) * RVentSpeed * dt;
		}
		else
		{
			RventRot *= std::max(0.0, 1.0 - RVentSpeed * dt);
		}
		break;
	}

	case 2:
	{ // automatic
		auto const motorcurrent{std::min<double>(ImaxHi, std::abs(Im))};
		if (std::abs(Itot) > RVentMinI && RList[MainCtrlActualPos].R > RVentCutOff)
		{

			RventRot += (RVentnmax * std::min(1.0, motorcurrent / NPoweredAxles / RVentMinI) * motorcurrent / ImaxLo - RventRot) * RVentSpeed * dt;
		}
		else if (DynamicBrakeType == dbrake_automatic && true == DynamicBrakeFlag)
		{
			RventRot += (RVentnmax * motorcurrent / ImaxLo - RventRot) * RVentSpeed * dt;
		}
		else if (RVentForceOn)
		{
			RventRot += (RVentnmax - RventRot) * RVentSpeed * dt;
		}
		else
		{
			RventRot *= std::max(0.0, 1.0 - RVentSpeed * dt);
		}
		break;
	}

	default:
	{
		break;
	}
	} // rventtype
}

// the power of a dumb or diesel engine; returns the ratio of the power a dumb engine is set to
double TMoverParameters::UpdateEnginePower()
{
	return Drive().UpdateEnginePower(*this);
}

// the line breaker opened by a voltage out of the motors' range, and the line contactors of a diesel-electric
void TMoverParameters::VoltageRelaysCheck()
{
	switch (EngineType)
	{

	case TEngineType::ElectricSeriesMotor:
	{
		// update the state of voltage relays
		auto const voltage{std::max(GetTrainsetHighVoltage(), PantographVoltage)};
		// where the line breaker holds without voltage (closed ahead of the pantographs) the undervoltage relay opens the
		// line contactors instead, see MotorConnectorsCheck(). it picks up again only with the master controller back
		// where a start begins, as the line breaker it stands in for wouldn't close on a running position either: at
		// zero, or at the first position, which is the first resistance step, where the line contactors close on any
		// start, and where the AI waits for them rather than going back to zero
		auto const novoltrelaypicksup{false == PantographsRequireMains || NoVoltRelay || MainCtrlPowerPos() <= 1};
		NoVoltRelay = EnginePowerSource.SourceType != TPowerSource::CurrentCollector || (voltage >= EnginePowerSource.CollectorParameters.MinV && novoltrelaypicksup);
		OvervoltageRelay =
		    EnginePowerSource.SourceType != TPowerSource::CurrentCollector || voltage <= EnginePowerSource.CollectorParameters.MaxV || false == EnginePowerSource.CollectorParameters.OVP;
		// wywalanie szybkiego z powodu niewłaściwego napięcia
		// (a line breaker closed ahead of the pantographs holds without voltage)
		EventFlag |= true == Mains && ((false == NoVoltRelay && false == PantographsRequireMains) || false == OvervoltageRelay) &&
		             MainSwitch(false, TrainType == train_type::emu ? range_t::unit : range_t::local); // TODO: check whether we need to send this EMU-wide
		break;
	}

	case TEngineType::ElectricInductionMotor:
	{
		// TODO: check if we can use instead the code for electricseriesmotor
		// nie wchodzić w funkcję bez potrzeby
		// (a line breaker closed ahead of the pantographs holds without voltage)
		// (the source by the traction selector of a dual-mode vehicle: the pantographs or the genset)
		if (Mains && ((TractionSupplyVoltage() < TractionSupplyMinVoltage() && false == PantographsRequireMains) ||
			    TractionSupplyVoltage() > EnginePowerSource.CollectorParameters.MaxV + 200))
		{
			MainSwitch(false, TrainType == train_type::emu ? range_t::unit : range_t::local); // TODO: check whether we need to send this EMU-wide
		}
		break;
	}

	case TEngineType::DieselElectric:
	{
		// TODO: move this to the auto relay check when the electric engine code paths are unified
		StLinFlag |= Mains && false == StLinFlag && MainCtrlPowerPos() == 1;
		StLinFlag &= MotorConnectorsCheck();
		StLinFlag &= MainCtrlPowerPos() > 0;

		break;
	}

	default:
	{
		break;
	}
	}
}

void TMoverParameters::DumbTractionForce(double const PosRatio)
{
	if (Mains && CabActive != 0)
	{
		if (Vel > 0.1)
		{
			Ft = std::min(1000.0 * Power / std::abs(V), Ftmax) * PosRatio;
		}
		else
		{
			Ft = Ftmax * PosRatio;
		}
		Ft = Ft * DirAbsolute; // DirActive*CabActive;
	}
	else
	{
		Ft = 0;
	}
}

void TMoverParameters::WheelsDrivenTractionForce(double dt)
{
	if (EnginePowerSource.SourceType == TPowerSource::InternalSource && EnginePowerSource.PowerType == TPowerType::BioPower)
		Ft = Sign(sin(eAngle)) * PulseForce * Transmision.Ratio;
	PulseForceTimer = PulseForceTimer + dt;
	if (PulseForceTimer > CtrlDelay)
	{
		PulseForce = 0;
		if (PulseForceCount > 0)
			PulseForceCount--;
	}
	EnginePower = Ft * (1.0 + Vel);
}

void TMoverParameters::SeriesMotorTractionForce(double dt)
{
	//        enrot:=Transmision.Ratio*nrot;
	// yB: szereg dwoch sekcji w ET42
	if (TrainType == train_type::et42 && Imax == ImaxHi)
		EngineVoltage = EngineVoltage / 2.0;
	Mm = Momentum(Current(enrot, EngineVoltage)); // oblicza tez prad p/slinik

	if (TrainType == train_type::et42)
	{
		if (Imax == ImaxHi)
			EngineVoltage = EngineVoltage * 2;
		if (DynamicBrakeFlag && abs(Im) > 300) // przeiesione do mover.cpp
			FuseOff();
	}
	if (DynamicBrakeType == dbrake_automatic && DynamicBrakeFlag)
	{
		SeriesMotorDynamicBrake(dt);
	}

	if (TrainType == train_type::et22 && DelayCtrlFlag) // szarpanie przy zmianie układu w byku
		Mm = Mm * RList[MainCtrlActualPos].Bn / (RList[MainCtrlActualPos].Bn + 1); // zrobione w momencie, żeby nie dawac elektryki w przeliczaniu sił

	if (abs(Im) > Imax)
		Vhyp += dt; //*(abs(Im) / Imax - 0.9) * 10; // zwieksz czas oddzialywania na PN
	else
		Vhyp = 0;
	if (Vhyp > CtrlDelay / 2) // jesli czas oddzialywania przekroczony
		FuseOff(); // wywalanie bezpiecznika z powodu przetezenia silnikow

	if ((DynamicBrakeType == dbrake_automatic || DynamicBrakeType == dbrake_switch) && DynamicBrakeFlag)
		Itot = Im * 2; // 2x2 silniki w EP09
	else if (TrainType == train_type::emu && Imin == IminLo && ScndS) // yBARC - boczniki na szeregu poprawnie
		Itot = Im;
	else
		Itot = Im * RList[MainCtrlActualPos].Bn; // prad silnika * ilosc galezi
	Mw = Mm * Transmision.Ratio * Transmision.Efficiency;
	Fw = Mw * 2.0 / WheelDiameter;
	Ft = Fw * NPoweredAxles; // sila trakcyjna
}

// the automatic dynamic brake of the series motors: the excitation current, by the stages of the ep brake (TUHEX)
void TMoverParameters::SeriesMotorDynamicBrake(double dt)
{
	if (TUHEX_Stages > 0) // hamowanie wielostopniowe, nadpisuje wartości domyślne
	{
		if (Vel > 100)
			TUHEX_StageActual = std::min(TUHEX_StageActual, 1);
		switch (TUHEX_StageActual)
		{
		case 1:
			TUHEX_Sum = TUHEX_Sum1;
			DynamicBrakeRes = DynamicBrakeRes1;
			break;
		case 2:
			TUHEX_Sum = TUHEX_Sum2;
			DynamicBrakeRes = DynamicBrakeRes1;
			break;
		case 3:
			TUHEX_Sum = TUHEX_Sum3;
			if (Vadd > 0.99 * TUHEX_MaxIw && DynamicBrakeRes == DynamicBrakeRes1)
				TUHEX_ResChange = true;
			if (TUHEX_ResChange && Vadd < 0.5 * TUHEX_MaxIw)
			{
				TUHEX_ResChange = false;
				DynamicBrakeRes = DynamicBrakeRes2;
			}
			break;
		default:
			DynamicBrakeRes = DynamicBrakeRes1;
			TUHEX_Sum = 0;
			break;
		}
	}
	if (Vadd + abs(Im) > TUHEX_Sum + TUHEX_Diff || Hamulec->GetEDBCP() < 0.25 || TUHEX_ResChange || (TUHEX_StageActual == 0 && TUHEX_Stages > 0))
	{
		Vadd -= 500.0 * dt;
		if (Vadd < TUHEX_MinIw)
		{
			Vadd = 0;
			DynamicBrakeFlag = false;
		}
	}
	else if (DynamicBrakeFlag && Vadd + abs(Im) < TUHEX_Sum - TUHEX_Diff)
	{
		Vadd += 70.0 * dt;
		Vadd = std::min(std::max(Vadd, TUHEX_MinIw), TUHEX_MaxIw);
	}
	if (Vadd > 0)
		Mm = MomentumF(Im, Vadd, 0);
}

// the shunt field relays of a diesel-electric, each kind its own conditions on the current, the speed or the
// power (tmp, the power the engine gives)
void TMoverParameters::ShuntFieldRelaysCheck(double const tmpV, double const PosRatio, double const tmp)
{
	auto const shuntfieldstate{ScndCtrlPos};

	switch (RelayType)
	{

	case 0:
	{

		if (ScndCtrlPos < ScndCtrlPosNo && Im <= MPTRelay[ScndCtrlPos].Iup * PosRatio)
		{
			++ScndCtrlPos;
		}
		if (ScndCtrlPos > 0 && Im >= MPTRelay[ScndCtrlPos].Idown * PosRatio)
		{
			--ScndCtrlPos;
		}
		break;
	}
	case 1:
	{

		if (ScndCtrlPos < ScndCtrlPosNo && MPTRelay[ScndCtrlPos].Iup < Vel)
		{
			++ScndCtrlPos;
		}
		if (ScndCtrlPos > 0 && MPTRelay[ScndCtrlPos].Idown > Vel)
		{
			--ScndCtrlPos;
		}
		break;
	}
	case 2:
	{

		ShuntFieldRelaysType2(tmp);
		break;
	}
	case 41:
	{
		ShuntFieldRelaysType41(tmpV);
		break;
	}
	case 45:
	{
		ShuntFieldRelaysType45();
		break;
	}
	case 46:
	{
		ShuntFieldRelaysType46();
		break;
	}
	default:
	{
		break;
	}
	} // switch RelayType

	if (ScndCtrlPos != shuntfieldstate)
	{
		SetFlag(SoundFlag, sound::relay | sound::shuntfield);
	}
}

// the relays of type 2: up by the speed while the engine has power to spare (tmp, the power it gives), down by the current
void TMoverParameters::ShuntFieldRelaysType2(double const tmp)
{
	if (ScndCtrlPos < ScndCtrlPosNo && MPTRelay[ScndCtrlPos].Iup < Vel && EnginePower < tmp * 0.99)
	{
		++ScndCtrlPos;
	}
	if (ScndCtrlPos > 0 && MPTRelay[ScndCtrlPos].Idown < Im)
	{
		--ScndCtrlPos;
	}
}

// the relays of type 41: up by the speed tmpV at full power, the engine slowed down, down by the current
void TMoverParameters::ShuntFieldRelaysType41(double const tmpV)
{
	if (ScndCtrlPos < ScndCtrlPosNo && MainCtrlPos == MainCtrlPosNo && tmpV * 3.6 > MPTRelay[ScndCtrlPos].Iup)
	{
		++ScndCtrlPos;
		enrot = enrot * 0.73;
	}
	if (ScndCtrlPos > 0 && Im > MPTRelay[ScndCtrlPos].Idown)
	{
		--ScndCtrlPos;
	}
}

// the relays of type 45, by the current and the speed: up from the 11th position, at most two under it, none under the 8th
void TMoverParameters::ShuntFieldRelaysType45()
{
	if (ScndCtrlPos < ScndCtrlPosNo && MainCtrlPos >= 11)
	{

		if (Im < MPTRelay[ScndCtrlPos].Iup)
		{
			++ScndCtrlPos;
		}
		// check for cases where the speed drops below threshold for level 2 or 3
		if (ScndCtrlPos > 1 && Vel < MPTRelay[ScndCtrlPos - 1].Idown)
		{
			--ScndCtrlPos;
		}
	}
	// malenie
	if (ScndCtrlPos > 0 && MainCtrlPos < 11)
	{

		if (ScndCtrlPos == 1)
		{
			if (Im > MPTRelay[ScndCtrlPos - 1].Idown)
			{
				--ScndCtrlPos;
			}
		}
		else
		{
			if (Vel < MPTRelay[ScndCtrlPos].Idown)
			{
				--ScndCtrlPos;
			}
		}
	}
	// 3rd level drops with master controller at position lower than 10...
	if (MainCtrlPos < 11)
	{
		ScndCtrlPos = std::min(2, ScndCtrlPos);
	}
	// ...and below position 7 field shunt drops altogether
	if (MainCtrlPos < 8)
	{
		ScndCtrlPos = 0;
	}
	/*
	                            // crude woodward approximation; difference between rpm for consecutive positions is ~5%
	                            // so we get full throttle until ~half way between desired and previous position, or zero on rpm reduction
	                            auto const woodward { std::clamp(
	                                ( DElist[ MainCtrlPos ].RPM / ( enrot * 60.0 ) - 1.0 ) * 50.0,
	                                0.0, 1.0 ) };
	*/
}

// the relays of type 46, by the current and the speed: up from the 12th position, down under it below 50 km/h, at most two
// under the 11th, none under the 8th
void TMoverParameters::ShuntFieldRelaysType46()
{
	// wzrastanie
	if (MainCtrlPos >= 12 && ScndCtrlPos < ScndCtrlPosNo)
	{
		ShuntFieldRelaysType46Up();
	}
	// malenie
	if (MainCtrlPos < 12 && ScndCtrlPos > 0 && Vel < 50.0)
	{
		ShuntFieldRelaysType46Down();
	}
	if (MainCtrlPos < 11)
	{
		ScndCtrlPos = std::min(2, ScndCtrlPos);
	}
	if (MainCtrlPos < 8)
	{
		ScndCtrlPos = 0;
	}
}

// a relay of type 46 down: an even one by the current, an odd one by the current and the speed
void TMoverParameters::ShuntFieldRelaysType46Down()
{
	// above 50 km/h already active shunt field can be maintained until lower controller setting
	if (ScndCtrlPos % 2 == 0)
	{
		if (MPTRelay[ScndCtrlPos].Idown < Im)
		{
			--ScndCtrlPos;
		}
	}
	else
	{
		if (MPTRelay[ScndCtrlPos + 1].Idown < Im && MPTRelay[ScndCtrlPos].Idown > Vel)
		{
			--ScndCtrlPos;
		}
	}
}

// a relay of type 46 up: an even one by the current, an odd one by the current and the speed
void TMoverParameters::ShuntFieldRelaysType46Up()
{
	if (ScndCtrlPos % 2 == 0)
	{
		if (MPTRelay[ScndCtrlPos].Iup > Im)
		{
			++ScndCtrlPos;
		}
	}
	else
	{
		if (MPTRelay[ScndCtrlPos - 1].Iup > Im && MPTRelay[ScndCtrlPos].Iup < Vel)
		{
			++ScndCtrlPos;
		}
	}
}

void TMoverParameters::InductionMotorTractionForce(double dt)
{
	if (MainCircuitPowered() && !SecuritySystem.is_engine_blocked())
	{
		double PosRatio;
		double tmp;
		double ActiveInverters = 0.0;
		for (auto const &inv : inductionMotor.inverters)
		{
			if (inv.IsActive)
				ActiveInverters += 1.0;
		}
		inductionMotor.inverterRatio = ActiveInverters / (double)inductionMotor.inverterCount;
		if (inductionMotor.maxPowerTable.size() > 1)
		{
			inductionMotor.config[eimc_p_Pmax] = TableInterpolation(inductionMotor.maxPowerTable, Vel);
		}
		InductionMotorSpeedControl(dt);
		InductionMotorBrakeBlending(ActiveInverters, PosRatio, tmp);
		// NOTE: the commands to operate the sandbox are likely to conflict with other similar ai decisions
		// TODO: gather these in single place so they can be resolved together
		if (SlippingWheels)
		{
			PosRatio = 0;
			tmp = 10;
			SandboxAuto(true, range_t::unit);
		} // przeciwposlizg
		else
		{
			// switch sandbox off
			SandboxAuto(false, range_t::unit);
		}
		if (ActiveInverters == 0.0)
		{
			PosRatio = 0;
			inductionMotor.forceShare = 0;
		}

		inductionMotor.forceShare += std::max(std::min(PosRatio - inductionMotor.forceShare, 0.02), -0.02) * 12 * tmp /*2{+4*byte(PosRatio<inductionMotor.forceShare)*/ * dt; // wartość zadana/procent czegoś

		InductionMotorDrive(dt);
	} // mains
	else
	{
		InductionMotorOff();
	}
}

// the speed control (tempomat) of an induction motor drive, set by its second controller
void TMoverParameters::InductionMotorSpeedControl(double dt)
{
	// tempomat
	if (ScndCtrlPosNo == 4 && SpeedCtrlTypeTime)
	{
		SpeedControlByTime(dt);
	}
	else if (ScndCtrlPosNo > 1)
	{
		SpeedControlByPosition(dt);
	}
}

// the speed control set by the position of the second controller, after it's been held for its delay
void TMoverParameters::SpeedControlByPosition(double dt)
{
	if (ScndCtrlPos != NewSpeed)
	{

		SpeedCtrlTimer = 0;
		NewSpeed = ScndCtrlPos;
	}
	else
	{
		SpeedCtrlTimer += dt;
		if (SpeedCtrlTimer > SpeedCtrlDelay)
		{
			auto NewSCAP = static_cast<int>((float)ScndCtrlPos / (float)ScndCtrlPosNo * Vmax);
			if (NewSCAP != SpeedCtrlValue)
			{
				SpeedCtrlValue = NewSCAP;
				//								SendCtrlToNext(VehicleCommand::speedControl, SpeedCtrlValue, CabActive);
			}
		}
	}
	SpeedCtrlUnit.IsActive = SpeedCtrlValue > 0;
}

// the speed control of a four position second controller, by time: the speed set down or up by 10 km/h a step, held,
// or off and at the top
void TMoverParameters::SpeedControlByTime(double dt)
{
	SpeedCtrlUnit.IsActive = SpeedCtrlValue > 0;
	switch (ScndCtrlPos)
	{
	case 0:
		NewSpeed = 0;
		SpeedCtrlValue = 0;
		SpeedCtrlTimer = 10;
		break;
	case 1:
		if (SpeedCtrlTimer > SpeedCtrlDelay)
		{
			SpeedCtrlTimer = 0;
			NewSpeed -= 10;
			if (NewSpeed < 0)
				NewSpeed = 0;
		}
		else
			SpeedCtrlTimer += dt;
		break;
	case 2:
		SpeedCtrlTimer = 10;
		SpeedCtrlValue = NewSpeed;
		break;
	case 3:
		if (SpeedCtrlTimer > SpeedCtrlDelay)
		{
			SpeedCtrlTimer = 0;
			NewSpeed += 10;
			if (NewSpeed > Vmax)
				NewSpeed = Vmax;
		}
		else
			SpeedCtrlTimer += dt;
		break;
	case 4:
		NewSpeed = Vmax;
		SpeedCtrlValue = Vmax;
		SpeedCtrlTimer = 10;
		break;
	default:
		break;
	}
}

// whether the induction motors brake, and the share of the electrodynamic and the pneumatic brake; sets the
// force asked of the motors (PosRatio) and how fast the drive follows it (tmp)
void TMoverParameters::InductionMotorBrakeBlending(double const ActiveInverters, double &PosRatio, double &tmp)
{
	InductionMotorDynamicBrakeFlag();
	auto const edBCP{Hamulec->GetEDBCP() * inductionMotor.config[eimc_p_abed]}; // stala napedu
	if (DynamicBrakeFlag)
	{
		PosRatio = InductionMotorBrakeRatio(ActiveInverters, edBCP);
		tmp = 5;
	}
	else
	{
		PosRatio = std::max(eimic_real, 0.);
		inductionMotor.state[eimv_Fzad] = PosRatio;
		if (Flat && inductionMotor.config[eimc_p_F0] * inductionMotor.state[eimv_Fful] > 0)
			PosRatio = std::min(PosRatio * inductionMotor.config[eimc_p_F0] / inductionMotor.state[eimv_Fful], 1.);
		/*                    if (ScndCtrlActualPos > 0) //speed control
		                        if (Vmax < 250)
		                            PosRatio = std::min(PosRatio, std::max(-1, 0.5 * (ScndCtrlActualPos - Vel)));
		                        else
		                            PosRatio =
		                                std::min(PosRatio, std::max(-1, 0.5 * (ScndCtrlActualPos * 2 - Vel))); */
		// PosRatio = 1.0 * (PosRatio * 0 + 1) * PosRatio; // 1 * 1 * PosRatio = PosRatio
		Hamulec->SetED(0);
		//           (Hamulec as TLSt).SetLBP(LocBrakePress);
		tmp = 4; // szybkie malenie, powolne wzrastanie
	}
}

// the electrodynamic brake of the induction motors on: with the doors open or permitted, by the train brake, the local
// brake or its own controller; off without them
void TMoverParameters::InductionMotorDynamicBrakeFlag()
{
	double const edBCP = Hamulec->GetEDBCP();
	auto const localbrakeactive{CabOccupied != 0 && LocHandle->GetCP() > 0.25};
	// gdy SplitEDPneumaticBrake jest aktywne, hamulec pomocniczy (LocalBrake) NIE wymusza
	// zalaczenia hamulca elektrodynamicznego - ED jest sterowany wlasnym nastawnikiem
	auto const dynbrakectrlactive{SplitEDPneumaticBrake && DynamicBrakeRatio() > 0.01 && DynamicBrakeAvailable()};
	auto const localbrakeforED{SplitEDPneumaticBrake ? false : localbrakeactive};
	if (false == Doors.instances[side::left].is_closed || false == Doors.instances[side::right].is_closed ||
	    (Doors.permit_needed && (Doors.instances[side::left].open_permit || Doors.instances[side::right].open_permit)))
	{
		DynamicBrakeFlag = true;
	}
	else if ((edBCP < 0.25 && false == localbrakeforED && false == dynbrakectrlactive && AnPos < 0.01) ||
	         (edBCP < 0.25 && ShuntModeAllow && false == dynbrakectrlactive && LocalBrakePosA < 0.01))
		DynamicBrakeFlag = false;
	else if (BrakePress > 0.25 && edBCP > 0.25 || localbrakeforED || AnPos > 0.02 || dynbrakectrlactive)
		DynamicBrakeFlag = true;
}

// the share of the electrodynamic brake (negative), and the distributor told how much of the air to let out; none
// when the air brake cuts the electrodynamic one off, or an inverter is out
double TMoverParameters::InductionMotorBrakeRatio(double const ActiveInverters, double const edBCP)
{
	double PosRatio;
	// ustalanie współczynnika blendingu do luzowania hamulca PN
	if (inductionMotor.state[eimv_Fmax] * Sign(V) * DirAbsolute < -1)
	{
		PosRatio = -Sign(V) * DirAbsolute * inductionMotor.state[eimv_Fr] / (inductionMotor.config[eimc_p_Fh] * std::max(edBCP, std::max(0.01, Hamulec->GetEDBCP())) / MaxBrakePress[0]);
		PosRatio = std::clamp(PosRatio, 0.0, 1.0);
	}
	else
	{
		PosRatio = 0;
	}
	PosRatio = static_cast<double>(Round(20.0 * PosRatio)) / 20.0; // stopniowanie PN/ED
	if (PosRatio < 19.5 / 20.0)
		PosRatio *= 0.9;
	Hamulec->SetED(std::max(0.0, std::min(PosRatio, 1.0))); // ustalenie stopnia zmniejszenia ciśnienia
	// ustalanie siły hamowania ED
	if (Hamulec->GetEDBCP() > 0.25 && inductionMotor.config[eimc_p_abed] < 0.001 || ActiveInverters < inductionMotor.inverterCount) // jeśli PN wyłącza ED
	{
		PosRatio = 0;
		inductionMotor.state[eimv_Fzad] = 0;
	}
	else
	{
		// strefa wylaczenia ED (Vh0..Vh1) - mnoznik 0..1 zaleznie od predkosci
		double const vhRamp = std::max(0.0, std::min(1.0, (Vel - inductionMotor.config[eimc_p_Vh0]) / (inductionMotor.config[eimc_p_Vh1] - inductionMotor.config[eimc_p_Vh0])));
		if (SplitEDPneumaticBrake)
		{
			// w trybie split LocalBrake nie zaleca ED; uzywany jest DynamicBrakeCtrl
			// oraz zwykle zapotrzebowanie z hamulca zespolonego (edBCP)
			double const edLeverDemand = DynamicBrakeRatio();
			PosRatio = -std::max(std::max(edLeverDemand, std::min(edBCP / MaxBrakePress[0], 1.0)), AnPos) * vhRamp;
			inductionMotor.state[eimv_Fzad] = -std::min(1.0, std::max(edLeverDemand, edBCP / MaxBrakePress[0]));
		}
		else
		{
			PosRatio = -std::max(std::min(edBCP * 1.0 / MaxBrakePress[0], 1.0), AnPos) * vhRamp;
			inductionMotor.state[eimv_Fzad] = -std::min(1.0, std::max(LocalBrakeRatio(), edBCP / MaxBrakePress[0]));
		}
	}
	return PosRatio;
}

// the model of the induction motors and their inverters: voltage, frequency, currents, moment and force
void TMoverParameters::InductionMotorDrive(double dt)
{
	double tmp;
	if (DynamicBrakeFlag)
		tmp = inductionMotor.config[eimc_f_Uzh];
	else
		tmp = inductionMotor.config[eimc_f_Uzmax];

	auto f_cfu{DynamicBrakeFlag ? inductionMotor.config[eimc_f_cfuH] : inductionMotor.config[eimc_f_cfu]};

	inductionMotor.state[eimv_Uzsmax] = std::min(EngineVoltage - inductionMotor.config[eimc_f_DU], tmp);
	inductionMotor.state[eimv_fkr] = inductionMotor.state[eimv_Uzsmax] / f_cfu;
	if (inductionMotor.forceShare < 0)
	{
		inductionMotor.state[eimv_Pmax] = inductionMotor.config[eimc_p_Ph];
	}
	else
	{
		inductionMotor.state[eimv_Pmax] = std::min(inductionMotor.config[eimc_p_Pmax], 0.001 * EngineVoltage * (inductionMotor.config[eimc_p_Imax] - inductionMotor.config[eimc_f_I0]) * Pirazy2 * inductionMotor.config[eimc_s_cim] / inductionMotor.config[eimc_s_p] / inductionMotor.config[eimc_s_cfu]);
		if (GensetSelected())
		{
			// on the genset of a dual-mode vehicle the traction takes no more than the genset gives
			inductionMotor.state[eimv_Pmax] = std::min(inductionMotor.state[eimv_Pmax], GensetTractionPowerLimit());
		}
	}
	inductionMotor.state[eimv_FMAXMAX] = 0.001 *
	                     square(std::min(1.0, inductionMotor.state[eimv_fkr] / std::max(abs(enrot) * inductionMotor.config[eimc_s_p] + inductionMotor.config[eimc_s_dfmax] * inductionMotor.state[eimv_ks], inductionMotor.config[eimc_s_dfmax])) * f_cfu / inductionMotor.config[eimc_s_cfu]) *
	                     (inductionMotor.config[eimc_s_dfmax] * inductionMotor.config[eimc_s_dfic] * inductionMotor.config[eimc_s_cim]) * Transmision.Ratio * NPoweredAxles * 2.0 / WheelDiameter;
	InductionMotorForceLimits();
	for (auto &inv : inductionMotor.inverters)
	{
		inv.Request = inv.IsActive ? inductionMotor.forceShare : 0.0;
		inv.Error = inv.Failure_Const || (inv.Failure_Drive && inv.Request != 0);
		inv.IsActive = inv.Activate && !inv.Error;
	}
	inductionMotor.state[eimv_ks] = inductionMotor.state[eimv_Fr] / inductionMotor.state[eimv_FMAXMAX];
	inductionMotor.state[eimv_df] = inductionMotor.state[eimv_ks] * inductionMotor.config[eimc_s_dfmax];
	inductionMotor.state[eimv_fp] = DirAbsolute * enrot * inductionMotor.config[eimc_s_p] + inductionMotor.state[eimv_df]; // do przemyslenia dzialanie pp z tmpV
	//         inductionMotor.state[eimv_U]:=std::max(inductionMotor.state[eimv_Uzsmax],std::min(inductionMotor.config[eimc_f_cfu]*inductionMotor.state[eimv_fp],inductionMotor.state[eimv_Uzsmax]));
	//         inductionMotor.state[eimv_pole]:=inductionMotor.state[eimv_U]/(inductionMotor.state[eimv_fp]*inductionMotor.config[eimc_s_cfu]);
	if (abs(inductionMotor.state[eimv_fp]) <= inductionMotor.state[eimv_fkr])
		inductionMotor.state[eimv_pole] = f_cfu / inductionMotor.config[eimc_s_cfu];
	else
		inductionMotor.state[eimv_pole] = inductionMotor.state[eimv_Uzsmax] / inductionMotor.config[eimc_s_cfu] / abs(inductionMotor.state[eimv_fp]);
	inductionMotor.state[eimv_U] = inductionMotor.state[eimv_pole] * inductionMotor.state[eimv_fp] * inductionMotor.config[eimc_s_cfu];
	inductionMotor.state[eimv_Ic] = (inductionMotor.state[eimv_fp] - DirAbsolute * enrot * inductionMotor.config[eimc_s_p]) * inductionMotor.config[eimc_s_dfic] * inductionMotor.state[eimv_pole];
	inductionMotor.state[eimv_If] = inductionMotor.state[eimv_Ic] * inductionMotor.config[eimc_s_icif];
	inductionMotor.state[eimv_M] = inductionMotor.state[eimv_pole] * inductionMotor.state[eimv_Ic] * inductionMotor.config[eimc_s_cim];
	inductionMotor.state[eimv_Ipoj] = inductionMotor.state[eimv_Ic] * NPoweredAxles * inductionMotor.inverterRatio * inductionMotor.state[eimv_U] / (EngineVoltage - inductionMotor.config[eimc_f_DU]) + inductionMotor.config[eimc_f_I0];
	inductionMotor.state[eimv_Pm] = DirActive * inductionMotor.state[eimv_M] * NPoweredAxles * inductionMotor.inverterRatio * enrot * Pirazy2 / 1000;
	inductionMotor.state[eimv_Pe] = inductionMotor.state[eimv_Ipoj] * EngineVoltage / 1000;
	inductionMotor.state[eimv_eta] = inductionMotor.state[eimv_Pm] / inductionMotor.state[eimv_Pe];

	Im = inductionMotor.state[eimv_If];
	if (inductionMotor.state[eimv_Ipoj] >= 0)
		Vadd *= 1.0 - 2.0 * dt;
	else if (GensetSelected())
		Vadd = 1.0; // the genset of a dual-mode vehicle takes nothing back: all the braking energy to the resistors
	else if (std::abs(EngineVoltage) < EnginePowerSource.CollectorParameters.MaxV)
		Vadd *= 1.0 - dt;
	else
		Vadd = std::max(Vadd * (1.0 - 0.2 * dt), 0.007 * (std::abs(EngineVoltage) - (EnginePowerSource.CollectorParameters.MaxV - 100)));
	Itot = inductionMotor.state[eimv_Ipoj] * (0.01 + std::min(0.99, 0.99 - Vadd));

	EnginePower = abs(inductionMotor.state[eimv_Ic] * inductionMotor.state[eimv_U] * NPoweredAxles) / 1000;
	// power inverters
	UpdateInverterFrequency();

	Mm = inductionMotor.state[eimv_M] * DirAbsolute;
	Mw = Mm * Transmision.Ratio * Transmision.Efficiency;
	Fw = Mw * 2.0 / WheelDiameter;
	Ft = Fw * NPoweredAxles * inductionMotor.inverterRatio;
	inductionMotor.state[eimv_Fr] = DirAbsolute * Ft / 1000;
}

// the force the motors give at full and at the setting (inductionMotor.forceShare), braking within the power and the force of the
// electrodynamic brake, driving within the power and the starting force
void TMoverParameters::InductionMotorForceLimits()
{
	if (inductionMotor.forceShare < 0)
	{
		inductionMotor.state[eimv_Fful] = std::min(inductionMotor.config[eimc_p_Ph] * 3.6 / (Vel != 0.0 ? Vel : 0.001), std::min(inductionMotor.config[eimc_p_Fh], inductionMotor.state[eimv_FMAXMAX]));
		inductionMotor.state[eimv_Fmax] = -Sign(V) * DirAbsolute*std::min(inductionMotor.config[eimc_p_Ph] * 3.6 / (Vel != 0.0 ? Vel : 0.001), std::min(-inductionMotor.config[eimc_p_Fh] * inductionMotor.forceShare, inductionMotor.state[eimv_FMAXMAX]));
		double pr = inductionMotor.forceShare;
		if (inductionMotor.logForce)
			pr = -log(1 - 4 * pr) / log(5);
		inductionMotor.state[eimv_Fr] = -Sign(V) * DirAbsolute*std::min(inductionMotor.config[eimc_p_Ph] * 3.6 / (Vel != 0.0 ? Vel : 0.001), std::min(-inductionMotor.config[eimc_p_Fh] * pr, inductionMotor.state[eimv_FMAXMAX]));
		if (inductionMotor.inverterRatio < 1.0)
			inductionMotor.state[eimv_Fful] = 0;
		//*std::min(1,(Vel-inductionMotor.config[eimc_p_Vh0])/(inductionMotor.config[eimc_p_Vh1]-inductionMotor.config[eimc_p_Vh0]))
	}
	else
	{
		inductionMotor.state[eimv_Fful] = std::min(std::min(3.6 * inductionMotor.state[eimv_Pmax] / std::max(Vel, 1.), inductionMotor.config[eimc_p_F0] - Vel * inductionMotor.config[eimc_p_a1]), inductionMotor.state[eimv_FMAXMAX]);
		//           if(not Flat)then
		inductionMotor.state[eimv_Fmax] = inductionMotor.state[eimv_Fful] * inductionMotor.forceShare;
		//           else
		//             inductionMotor.state[eimv_Fmax]:=std::min(inductionMotor.config[eimc_p_F0]*inductionMotor.forceShare,inductionMotor.state[eimv_Fful]);
		double pr = inductionMotor.forceShare;
		if (inductionMotor.logForce)
			pr = log(1 + 4 * pr) / log(5);
		inductionMotor.state[eimv_Fr] = inductionMotor.state[eimv_Fful] * pr;
	}
}

// the frequency of the inverters at the motors' slip frequency, from the list of the drive or of its brake
void TMoverParameters::UpdateInverterFrequency()
{
	auto const tmpV{std::abs(inductionMotor.state[eimv_fp])};
	auto const useFFEDList = FFEDListSize > 0 && DynamicBrakeFlag;
	auto const list = useFFEDList ? FFEDlist : FFlist;

	if (auto const listSize = useFFEDList ? FFEDListSize : FFListSize; listSize > 0 && std::abs(inductionMotor.state[eimv_If]) > 1.0 && tmpV > 0.0001)
	{

		int i = 0;
		while (i < listSize - 1 && list[i + 1].v < tmpV)
		{
			++i;
		}
		inductionMotor.inverterFrequency = (tmpV - list[i].v) / std::max(1.0, list[i + 1].v - list[i].v) * (list[i + 1].freq - list[i].freq) + list[i].freq;
	}
	else
	{
		inductionMotor.inverterFrequency = 0.0;
	}
}

// the induction motor drive without power, or blocked by the security system
void TMoverParameters::InductionMotorOff()
{
	for (auto &inv : inductionMotor.inverters)
	{
		inv.Freal = 0.0;
		inv.IsActive = false;
	}
	Im = 0.0;
	Mm = 0.0;
	Mw = 0.0;
	Fw = 0.0;
	Ft = 0.0;
	Itot = 0.0;
	inductionMotor.forceShare = 0.0;
	EnginePower = 0.0;
	{
		for (auto &eimvvalue : inductionMotor.state)
			eimvvalue = 0.0;
	}
	Hamulec->SetED(0.0);
	inductionMotor.inverterFrequency = 0.0; //(Hamulec as TLSt).SetLBP(LocBrakePress);
}

// *************************************************************************************************
// Q: 20160713
// Sprawdzenie bezpiecznika nadmiarowego
// *************************************************************************************************
bool TMoverParameters::FuseFlagCheck(void) const
{
	bool FFC;

	FFC = false;
	if (Power > 0.01)
		FFC = FuseFlag;
	else // pobor pradu jezeli niema mocy
		for (auto &thiscoupler : Couplers)
			if (thiscoupler.CouplingFlag.HasAll(Coupling::control) && thiscoupler.Connected->Power > 0.01)
				FFC = thiscoupler.Connected->FuseFlagCheck();

	return FFC;
}

// *************************************************************************************************
// Q: 20160713
// Załączenie bezpiecznika nadmiarowego
// *************************************************************************************************
bool TMoverParameters::FuseOn(range_t const Notify)
{
	auto const result{RelayReset(relay_t::maincircuitground | relay_t::tractionnmotoroverload, Notify)};

	return result;
}

// *************************************************************************************************
// Q: 20160713
// Wyłączenie bezpiecznika nadmiarowego
// *************************************************************************************************
void TMoverParameters::FuseOff(void)
{
	if (!FuseFlag)
	{
		FuseFlag = true;
		EventFlag = true;
		SetFlag(SoundFlag, sound::relay | sound::loud);
	}
}

// resets relays assigned to specified customizable reset button
bool TMoverParameters::UniversalResetButton(int const Button, range_t const Notify)
{

	if (auto const lowvoltagepower{Power24vIsAvailable || Power110vIsAvailable}; false == lowvoltagepower)
	{
		return false;
	}

	auto const relays{UniversalResetButtonFlag[Button]};
	if (relays == 0)
	{
		return false;
	}

	auto const result{RelayReset(relays, Notify)};

	return result;
}

// resets state of specified relays
bool TMoverParameters::RelayReset(int const Relays, range_t const Notify)
{

	auto const lowvoltagepower{Power24vIsAvailable || Power110vIsAvailable};
	bool reset{false};

	if (TestFlag(Relays, relay_t::maincircuitground) && (EngineType == TEngineType::ElectricSeriesMotor || EngineType == TEngineType::DieselElectric) &&
		    (GroundRelayStart == start_t::manual || GroundRelayStart == start_t::manualwithautofallback) && IsMainCtrlNoPowerPos() && ScndCtrlPos == 0 && DirActive != 0 &&
		    !TestFlag(EngDmgFlag, 1))
	{
		// NOTE: true means the relay is operational
		reset |= !GroundRelay && lowvoltagepower;
		GroundRelay |= lowvoltagepower;
	}

	if (TestFlag(Relays, relay_t::tractionnmotoroverload) && (EngineType == TEngineType::ElectricSeriesMotor || EngineType == TEngineType::DieselElectric) && IsMainCtrlNoPowerPos() && ScndCtrlPos == 0 && DirActive != 0 &&
		    !TestFlag(EngDmgFlag, 1))
	{
		// NOTE: false means the relay is operational
		// TODO: cleanup, flip the FuseFlag code to match other relays
		// TODO: check whether the power is required, TBD, TODO: make it configurable?
		reset |= FuseFlag && lowvoltagepower;
		FuseFlag &= !lowvoltagepower;
	}

	if (TestFlag(Relays, relay_t::primaryconverteroverload) && ConverterOverloadRelayStart == start_t::manual
		    //         && ( false == Mains )
		    && false == ConverterAllow)
	{
		// NOTE: false means the relay is operational
		// TODO: cleanup, flip the FuseFlag code to match other relays
		// TODO: check whether the power is required, TBD, TODO: make it configurable?
		reset |= ConvOvldFlag && lowvoltagepower;
		ConvOvldFlag &= !lowvoltagepower;
	}

	if (reset)
	{
		SetFlag(SoundFlag, sound::relay | sound::loud);
	}

	if (Notify != range_t::local)
	{
		SendCtrlToNext(VehicleCommand::relayReset, Relays, CabActive, Notify == range_t::unit ? Coupling::control | Coupling::permanent : Coupling::control);
	}

	return reset;
}

// *************************************************************************************************
// Q: 20160713
// Przeliczenie prędkości liniowej na obrotową
// *************************************************************************************************
double TMoverParameters::v2n(void)
{
	// przelicza predkosc liniowa na obrotowa
	const double dmgn = 0.5;
	double n;
	double deltan = 0;

	n = V / (M_PI * WheelDiameter); // predkosc obrotowa wynikajaca z liniowej [obr/s]
	deltan = n - nrot; //"pochodna" prędkości obrotowej
	/* if (SlippingWheels)
	    if (std::abs(deltan) < 0.001)
	        SlippingWheels = false; // wygaszenie poslizgu */ //poslizg jest w innym miejscu wygaszany też
	if (SlippingWheels) // nie ma zwiazku z predkoscia liniowa V
	{ // McZapkie-221103: uszkodzenia kol podczas poslizgu
		if (deltan > dmgn && FuzzyLogic(deltan, dmgn, p_slippdmg) && DamageFlag.Set(VehicleDamage::wheelWear)) // podkucie
			EventFlag = true;
		if (deltan < -dmgn && FuzzyLogic(-deltan, dmgn, p_slippdmg) && DamageFlag.Set(VehicleDamage::thinWheel)) // wycieranie sie obreczy
			EventFlag = true;
		n = nrot; // predkosc obrotowa nie zalezy od predkosci liniowej
	}
	return n;
}

// *************************************************************************************************
// Q: 20160714
// Oblicza moment siły wytwarzany przez silnik
// *************************************************************************************************
double TMoverParameters::Momentum(double I) const
{
	// liczy moment sily wytwarzany przez silnik elektryczny}
	int SP;

	SP = ScndCtrlActualPos;
	if (ScndInMain && !(RList[MainCtrlActualPos].ScndAct == 255))
		SP = RList[MainCtrlActualPos].ScndAct;

	//     Momentum:=mfi*I*(1-1.0/(Abs(I)/mIsat+1));
	return motor_param(SP).mfi * I * (abs(I) / (abs(I) + motor_param(SP).mIsat) - motor_param(SP).mfi0);
}

// *************************************************************************************************
// Q: 20160714
// Oblicza moment siły do sterowania wzbudzeniem
// *************************************************************************************************
double TMoverParameters::MomentumF(double I, double Iw, int SCP) const
{
	// umozliwia dokladne sterowanie wzbudzeniem

	return motor_param(SCP).mfi * I * std::max(abs(Iw) / (abs(Iw) + motor_param(SCP).mIsat) - motor_param(SCP).mfi0, 0.);
}

// *************************************************************************************************
// Q: 20160713
// Odłączenie uszkodzonych silników
// *************************************************************************************************
bool TMoverParameters::CutOffEngine(void)
{
	bool COE = false; // Ra: wartość domyślna, sprawdzić to trzeba
	if (NPoweredAxles > 0 && CabActive == 0 && EngineType == TEngineType::ElectricSeriesMotor && DamageFlag.Clear(VehicleDamage::engine))
	{
		NPoweredAxles = NPoweredAxles / 2; // bylo div czyli mod?
		COE = true;
	}
	return COE;
}

// *************************************************************************************************
// Q: 20160713
// Przełączenie wysoki / niski prąd rozruchu
// *************************************************************************************************
bool TMoverParameters::MaxCurrentSwitch(bool State, range_t const Notify)
{
	auto const initialstate{MotorOverloadRelayHighThreshold};

	MotorOverloadRelayHighThreshold = State;

	if (Notify != range_t::local)
	{
		SendCtrlToNext(VehicleCommand::maxCurrentSwitch, State ? 1 : 0, CabActive, Notify == range_t::unit ? Coupling::control | Coupling::permanent : Coupling::control);
	}

	return State != initialstate;
}

// *************************************************************************************************
// Q: 20160713
// Przełączenie wysoki / niski prąd rozruchu automatycznego
// *************************************************************************************************
bool TMoverParameters::MinCurrentSwitch(bool State)
{
	bool MCS = false;
	if ((EngineType == TEngineType::ElectricSeriesMotor && IminHi > IminLo) || (TrainType == train_type::emu && EngineType != TEngineType::ElectricInductionMotor))
	{

		if (State && Imin == IminLo)
		{
			Imin = IminHi;
			MCS = true;
			if (CabActive != 0)
				SendCtrlToNext(VehicleCommand::minCurrentSwitch, 1, CabActive);
		}
		if (!State && Imin == IminHi)
		{
			Imin = IminLo;
			MCS = true;
			if (CabActive != 0)
				SendCtrlToNext(VehicleCommand::minCurrentSwitch, 0, CabActive);
		}
	}
	return MCS;
}

// *************************************************************************************************
// Q: 20160713
// Sprawdzenie wskaźnika jazdy na oporach
// *************************************************************************************************
bool TMoverParameters::ResistorsFlagCheck(void) const
{
	bool RFC = false;

	if (Power > 0.01)
		RFC = ResistorsFlag;
	else // pobor pradu jezeli niema mocy
	{
		for (auto &thiscoupler : Couplers)
			if (thiscoupler.CouplingFlag.HasAll(Coupling::control) && thiscoupler.Connected->Power > 0.01)
				RFC = thiscoupler.Connected->ResistorsFlagCheck();
	}
	return RFC;
}

// *************************************************************************************************
// Q: 20160714
// Zwraca wartość natężenia prądu na wybranym amperomierzu. Podfunkcja do ShowCurrent.
// *************************************************************************************************
double TMoverParameters::ShowCurrentP(int AmpN) const
{
	int Bn;
	bool Grupowy;

	// ClearPendingExceptions;
	Grupowy = DelayCtrlFlag && TrainType == train_type::et22; // przerzucanie walu grupowego w ET22;
	Bn = RList[MainCtrlActualPos].Bn; // ile równoległych gałęzi silników

	if (DynamicBrakeType == dbrake_automatic && DynamicBrakeFlag)
		Bn = DynamicBrakeAmpmeters;
	if (Power > 0.01)
	{
		if (AmpN > 0) // podać prąd w gałęzi
		{
			if (Bn < AmpN || (Grupowy && AmpN == Bn - 1))
				return 0;
			else // normalne podawanie pradu
				return floor(abs(Im));
		}
		else // podać całkowity
			return floor(abs(Itot));
	}
	else // pobor pradu jezeli niema mocy
	{
		int current = 0;
		for (int b = 0; b < 2; b++)
			// with Couplers[b] do
			if (Couplers[b].CouplingFlag.HasAll(Coupling::control) && Couplers[b].Connected->Power > 0.01)
				current = static_cast<int>(Couplers[b].Connected->ShowCurrent(AmpN));
		return current;
	}
}

// the drive of the kind of engine of the vehicle
TractionDrive const &TMoverParameters::Drive() const
{
	return DriveOf(EngineType);
}
