/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the drives of the vehicles, one class for each kind of engine

#include "stdafx.h"
#include "physics/components/drive.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "physics/mover.h"
#include "utilities/utilities.h"

// the defaults: a drive with no engine model and no engine of revolutions does nothing in the steps

double TractionDrive::UpdateEnginePower(TMoverParameters & /*vehicle*/) const
{
	return 0.0;
}

void TractionDrive::BeforeTractionForce(TMoverParameters & /*vehicle*/, double /*dt*/) const
{
	// nothing before the force
}

void TractionDrive::UpdateTractionForce(TMoverParameters & /*vehicle*/, DriveStep const & /*step*/) const
{
	// no force
}

void TractionDrive::AfterTractionForce(TMoverParameters & /*vehicle*/) const
{
	// nothing after the force
}

void TractionDrive::UpdateVentilators(TMoverParameters & /*vehicle*/, double /*dt*/) const
{
	// no ventilators
}

double TractionDrive::ShowCurrent(TMoverParameters const &vehicle, int const ammeter) const
{
	return vehicle.ShowCurrentP(ammeter);
}

double TractionDrive::EngineRpmRatio(TMoverParameters const & /*vehicle*/) const
{
	return 1.0;
}

double TractionDrive::EngineIdleRpm(TMoverParameters const & /*vehicle*/) const
{
	return std::numeric_limits<double>::max();
}

double TractionDrive::EngineMaxRpm(TMoverParameters const & /*vehicle*/) const
{
	return std::numeric_limits<double>::max();
}

double TractionDrive::UpdateEngineRevolutions(TMoverParameters &vehicle, double /*dt*/, double & /*transmission*/, double & /*moment*/) const
{
	vehicle.enrot = vehicle.Transmision.Ratio * vehicle.nrot;
	return 0.0;
}

bool TractionDrive::IncreaseMainController(TMoverParameters & /*vehicle*/, int /*speed*/) const
{
	return false;
}

bool TractionDrive::DecreaseMainController(TMoverParameters & /*vehicle*/, int /*speed*/) const
{
	return false;
}

namespace
{

// the radiator fans of a diesel engine follow their demanded revolutions while the engine runs, and run down without it
void updateRadiatorFans(TMoverParameters &vehicle, double const dt)
{
	auto &heat{vehicle.diesel.heat};
	if (true == vehicle.Mains)
	{
		heat.rpmw += static_cast<float>(std::clamp(heat.rpmwz - heat.rpmw, -100.f, 50.f) * dt);
		heat.rpmw2 += static_cast<float>(std::clamp(heat.rpmwz2 - heat.rpmw2, -100.f, 50.f) * dt);
	}
	else
	{
		heat.rpmw *= static_cast<float>(std::max(0.0, 1.0 - heat.rpmw * dt));
		heat.rpmw2 *= static_cast<float>(std::max(0.0, 1.0 - heat.rpmw2 * dt));
	}
}

// a notch of the master controller of the drives without steps of their own (no engine, Dumb, diesel-electric,
// induction): one position at a time, the speed control let go when the position leaves the one it holds
bool increaseNotch(TMoverParameters &vehicle, int const speed)
{
	if (speed > 1)
	{
		return vehicle.IncMainCtrl(1) && vehicle.IncMainCtrl(speed - 1); // a fail will propagate up the recursion chain. should this be || instead?
	}
	++vehicle.MainCtrlPos;
	if (vehicle.EIMCtrlType == 0 && (vehicle.SpeedCtrlAutoTurnOffFlag & 1) == 1 && vehicle.MainCtrlActualPos != vehicle.MainCtrlPos)
	{
		vehicle.SpeedCtrlSwitch(false);
		vehicle.SpeedCtrlUnit.IsActive = false;
	}
	return true;
}

bool decreaseNotch(TMoverParameters &vehicle, int const speed)
{
	if (speed == 1)
	{
		vehicle.MainCtrlPos--;
		if (vehicle.EIMCtrlType == 0 && (vehicle.SpeedCtrlAutoTurnOffFlag & 1) == 1 && vehicle.MainCtrlActualPos != vehicle.MainCtrlPos)
		{
			vehicle.SpeedCtrlSwitch(false);
			vehicle.SpeedCtrlUnit.IsActive = false;
		}
		return true;
	}
	if (speed > 1)
	{
		return vehicle.DecMainCtrl(1) && vehicle.DecMainCtrl(speed - 1);
	}
	return false;
}

// a vehicle without an engine (a wagon, a control car): its controller still steps, for the vehicles it controls
class NoEngineDrive final : public TractionDrive
{
public:
	bool IncreaseMainController(TMoverParameters &vehicle, int const speed) const override
	{
		return increaseNotch(vehicle, speed);
	}
	bool DecreaseMainController(TMoverParameters &vehicle, int const speed) const override
	{
		return decreaseNotch(vehicle, speed);
	}
};

// an engine with no model at all (steam, Main): nothing happens in the steps
class UnmodelledDrive final : public TractionDrive
{
};

// an engine with no model: the power follows the share of the controller positions
class DumbDrive final : public TractionDrive
{
public:
	double UpdateEnginePower(TMoverParameters &vehicle) const override
	{
		auto const posRatio{(vehicle.MainCtrlPos + vehicle.ScndCtrlPos) / (vehicle.MainCtrlPosNo + vehicle.ScndCtrlPosNo + 0.01)};
		vehicle.EnginePower = /*1000.0 **/ vehicle.Power * posRatio;
		return posRatio;
	}
	void UpdateTractionForce(TMoverParameters &vehicle, DriveStep const &step) const override
	{
		vehicle.DumbTractionForce(step.posRatio);
	}
	bool IncreaseMainController(TMoverParameters &vehicle, int const speed) const override
	{
		return increaseNotch(vehicle, speed);
	}
	bool DecreaseMainController(TMoverParameters &vehicle, int const speed) const override
	{
		return decreaseNotch(vehicle, speed);
	}
};

// a draisine and the like, the wheels driven by hand
class WheelsDrivenDrive final : public TractionDrive
{
public:
	void UpdateTractionForce(TMoverParameters &vehicle, DriveStep const &step) const override
	{
		vehicle.WheelsDrivenTractionForce(step.dt);
	}
	bool IncreaseMainController(TMoverParameters &vehicle, int const speed) const override
	{
		return vehicle.AddPulseForce(speed);
	}
};

// series wound direct current motors, started through resistors
class SeriesMotorDrive final : public TractionDrive
{
public:
	void UpdateTractionForce(TMoverParameters &vehicle, DriveStep const &step) const override
	{
		vehicle.SeriesMotorTractionForce(step.dt);
	}
	bool IncreaseMainController(TMoverParameters &vehicle, int const speed) const override
	{
		return vehicle.IncMainCtrlSeriesMotor(speed);
	}
	bool DecreaseMainController(TMoverParameters &vehicle, int const speed) const override
	{
		return vehicle.DecMainCtrlSeriesMotor(speed);
	}
	void UpdateVentilators(TMoverParameters &vehicle, double const dt) const override
	{
		if (vehicle.MainCircuitPowered())
		{
			vehicle.UpdateResistorBlowers(dt);
		} // mains
		else
		{
			vehicle.RventRot *= std::max(0.0, 1.0 - vehicle.RVentSpeed * dt);
		}
	}
};

// asynchronous motors fed by inverters
class InductionMotorDrive final : public TractionDrive
{
public:
	void UpdateTractionForce(TMoverParameters &vehicle, DriveStep const &step) const override
	{
		vehicle.InductionMotorTractionForce(step.dt);
	}
	bool IncreaseMainController(TMoverParameters &vehicle, int const speed) const override
	{
		return increaseNotch(vehicle, speed);
	}
	bool DecreaseMainController(TMoverParameters &vehicle, int const speed) const override
	{
		return decreaseNotch(vehicle, speed);
	}
	double ShowCurrent(TMoverParameters const &vehicle, int const ammeter) const override
	{
		switch (ammeter)
		{ // do asynchronicznych
		case 1:
			return vehicle.WindingRes * vehicle.Mm / vehicle.Vadd;
		case 2:
			return vehicle.inductionMotor.forceShare * vehicle.WindingRes;
		default:
			return vehicle.ShowCurrentP(ammeter);
		}
	}
};

// a diesel engine driving the wheels through a mechanical or hydraulic transmission
class DieselMechanicalDrive final : public TractionDrive
{
public:
	double UpdateEnginePower(TMoverParameters &vehicle) const override
	{
		vehicle.diesel.power = vehicle.Mm * vehicle.enrot * (2.0 * M_PI / 1000.0);
		vehicle.EnginePower = (vehicle.diesel.standingMoment + vehicle.Mm) * vehicle.enrot * (2.0 * M_PI / 1000.0);
		if (vehicle.MainCtrlPowerPos() > 1)
		{
			// dodatkowe opory z powodu sprezarki}
			//                dmoment -= diesel.standingMoment * ( 0.2 * enrot / diesel.maxRevolutions ); //yB: skąd to w ogóle się bierze?!
		}
		return 0.0;
	}
	void UpdateTractionForce(TMoverParameters &vehicle, DriveStep const &step) const override
	{
		vehicle.DieselEngineTractionForce(step.dt, step.moment, step.transmission);
	}
	double UpdateEngineRevolutions(TMoverParameters &vehicle, double const dt, double &transmission, double &moment) const override
	{
		if (vehicle.ShuntMode) // dodatkowa przekładnia np. dla 2Ls150
			transmission = vehicle.AnPos * vehicle.Transmision.Ratio * vehicle.motor_param(vehicle.ScndCtrlActualPos).mIsat;
		else
			transmission = vehicle.Transmision.Ratio * vehicle.motor_param(vehicle.ScndCtrlActualPos).mIsat;

		moment = vehicle.dizel_Momentum(vehicle.diesel.fill, transmission * vehicle.nrot * vehicle.DirActive, dt); // oblicza tez enrot
		return 0.0;
	}
	bool IncreaseMainController(TMoverParameters &vehicle, int const speed) const override
	{
		if (speed > 1)
		{
			while (vehicle.MainCtrlPos < vehicle.MainCtrlPosNo && vehicle.IncMainCtrl(1))
			{
				// all work is done in the loop header
			}
		}
		else
		{
			++vehicle.MainCtrlPos;
		}
		vehicle.CompressorAllow = vehicle.MainCtrlPowerPos() > 0;
		return true;
	}
	bool DecreaseMainController(TMoverParameters &vehicle, int const speed) const override
	{
		if (speed == 1)
		{
			vehicle.MainCtrlPos--;
			return true;
		}
		if (speed > 1)
		{
			while (vehicle.MainCtrlPos > 0 || vehicle.RList[vehicle.MainCtrlPos].Mn > 0)
				vehicle.DecMainCtrl(1);
			return true;
		}
		return false;
	}
	void UpdateVentilators(TMoverParameters &vehicle, double const dt) const override
	{
		// NOTE: we update only radiator fans, as vehicles with diesel engine don't have other ventilators
		updateRadiatorFans(vehicle, dt);
	}
	double EngineRpmRatio(TMoverParameters const &vehicle) const override
	{
		return std::clamp(std::abs(vehicle.enrot) / vehicle.nmax, 0.0, 1.0);
	}
	double EngineIdleRpm(TMoverParameters const &vehicle) const override
	{
		return vehicle.diesel.minRevolutions * 60;
	}
	double EngineMaxRpm(TMoverParameters const &vehicle) const override
	{
		return vehicle.diesel.maxRevolutions * 60;
	}
};

// a diesel engine driving a generator which feeds the traction motors
class DieselElectricDrive final : public TractionDrive
{
public:
	double UpdateEnginePower(TMoverParameters &vehicle) const override
	{
		vehicle.EnginePower = 0; // the actual calculation is done in two steps later in the method
		return 0.0;
	}
	void BeforeTractionForce(TMoverParameters &vehicle, double const dt) const override
	{
		if (vehicle.DEDynamicBrake.source != TMoverParameters::diesel_electric_dynamic_brake::control::none)
		{
			vehicle.update_dieselelectric_dynamic_brake(dt);
		}
	}
	void UpdateTractionForce(TMoverParameters &vehicle, DriveStep const &step) const override
	{
		vehicle.DieselElectricTractionForce(step.revolutions);
		if (vehicle.DEDynamicBrake.level > 0.0)
		{
			// braking: no power asked for, so the drive above gave no force
			vehicle.apply_dieselelectric_dynamic_brake(step.dt);
		}
	}
	double UpdateEngineRevolutions(TMoverParameters &vehicle, double const dt, double & /*transmission*/, double & /*moment*/) const override
	{
		double target{0.0};
		if (true == vehicle.Mains && true == vehicle.FuelPump.is_active)
		{
			target = vehicle.DieselElectricTargetRevolutions();
		}
		else
		{
			target = 0.0;
			vehicle.diesel.fill = 0.0;
		}

		if (vehicle.enrot != target)
		{
			vehicle.enrot = safe_clamp(vehicle.enrot + dt / vehicle.diesel.shaftInertia * (vehicle.enrot < target ? 1.0 : -1.0 * vehicle.diesel.revolutionsDecreaseRate), // NOTE: revolutions typically drop faster than they rise
			                           0.0, std::max(target, vehicle.enrot));
			if (std::abs(target - vehicle.enrot) < 0.001)
			{
				vehicle.enrot = target;
			}
		}
		return target;
	}
	bool IncreaseMainController(TMoverParameters &vehicle, int const speed) const override
	{
		return increaseNotch(vehicle, speed);
	}
	bool DecreaseMainController(TMoverParameters &vehicle, int const speed) const override
	{
		return decreaseNotch(vehicle, speed);
	}
	void AfterTractionForce(TMoverParameters &vehicle) const override
	{
		// rough approximation of extra effort to overcome friction etc
		vehicle.EnginePower += EngineRpmRatio(vehicle) * 0.15 * vehicle.DElist[vehicle.MainCtrlPosNo].GenPower;
	}
	void UpdateVentilators(TMoverParameters &vehicle, double const dt) const override
	{
		// NOTE: for this type RventRot is the speed of motor blowers; we also update radiator fans while at it
		if (true == vehicle.Mains)
		{
			// TBD, TODO: currently ignores RVentType, fix this?
			vehicle.RventRot += std::clamp(vehicle.enrot - vehicle.RventRot, -100.0, 50.0) * dt;
		}
		else
		{
			vehicle.RventRot *= std::max(0.0, 1.0 - vehicle.RVentSpeed * dt);
		}
		updateRadiatorFans(vehicle, dt);
	}
	double ShowCurrent(TMoverParameters const &vehicle, int const /*ammeter*/) const override
	{
		return std::fabs(vehicle.Im);
	}
	double EngineRpmRatio(TMoverParameters const &vehicle) const override
	{
		return std::clamp(60.0 * std::abs(vehicle.enrot) / vehicle.DElist[vehicle.MainCtrlPosNo].RPM, 0.0, 1.0);
	}
	double EngineIdleRpm(TMoverParameters const &vehicle) const override
	{
		return vehicle.DElist[vehicle.MainCtrlNoPowerPos()].RPM;
	}
	double EngineMaxRpm(TMoverParameters const &vehicle) const override
	{
		return vehicle.DElist[vehicle.MainCtrlPosNo].RPM;
	}
};

NoEngineDrive const noEngineDrive{};
UnmodelledDrive const unmodelledDrive{};
DumbDrive const dumbDrive{};
WheelsDrivenDrive const wheelsDrivenDrive{};
SeriesMotorDrive const seriesMotorDrive{};
InductionMotorDrive const inductionMotorDrive{};
DieselMechanicalDrive const dieselMechanicalDrive{};
DieselElectricDrive const dieselElectricDrive{};

} // namespace

TractionDrive const &DriveOf(TEngineType const type)
{
	switch (type)
	{
	case TEngineType::Dumb:
		return dumbDrive;
	case TEngineType::WheelsDriven:
		return wheelsDrivenDrive;
	case TEngineType::ElectricSeriesMotor:
		return seriesMotorDrive;
	case TEngineType::ElectricInductionMotor:
		return inductionMotorDrive;
	case TEngineType::DieselEngine:
		return dieselMechanicalDrive;
	case TEngineType::DieselElectric:
		return dieselElectricDrive;
	case TEngineType::None:
		return noEngineDrive;
	case TEngineType::SteamEngine:
	case TEngineType::Main:
		return unmodelledDrive;
	}
	return unmodelledDrive;
}
