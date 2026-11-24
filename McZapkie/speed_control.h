#pragma once

#include "core.h"

namespace mover
{
struct speed_control
{
	bool IsActive = false;
	bool Start = false;
	bool ManualStateOverride = true;
	bool BrakeIntervention = false;
	bool BrakeInterventionBraking = false;
	bool BrakeInterventionUnbraking = false;
	bool Standby = false;
	bool Parking = false;
	double InitialPower = 1.0;
	double FullPowerVelocity = -1;
	double StartVelocity = -1;
	double VelocityStep = 5;
	double PowerStep = 0.1;
	double MinPower = 0.0;
	double MaxPower = 1.0;
	double MinVelocity = 0;
	double MaxVelocity = 120;
	double DesiredVelocity = 0;
	double DesiredPower = 1.0;
	double Offset = -0.5;
	double FactorPpos = 0.5;
	double FactorPneg = 0.5;
	double FactorIpos = 0.0;
	double FactorIneg = 0.0;
	double BrakeInterventionVel = 30.0;
	double PowerUpSpeed = 1000;
	double PowerDownSpeed = 1000;
};

} // namespace mover
