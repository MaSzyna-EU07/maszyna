/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "McZapkie/MOVER.h"

// VGRAPH_SHADOW_CHECK: the code of TMoverParameters extracted to the functions shared with the vehicle graph, in its
// original form, working on copies of the state. Mover runs it next to every call of the extracted functions and logs
// the differences, so the extraction can be verified on real vehicles. remove once the reference set of vehicles
// shows no differences
#ifdef VGRAPH_SHADOW_CHECK

#include "utilities/Logs.h"
#include "utilities/utilities.h"

namespace
{

// as in Mover.cpp
inline double square(double val)
{
	return val * val;
}

} // namespace

void TMoverParameters::shadow_compare(char const *What, double const Original, double const Extracted) const
{
	if (Original == Extracted || (std::isnan(Original) && std::isnan(Extracted)))
	{
		return;
	}
	WriteLog("VGRAPH_SHADOW_CHECK: " + Name + ": " + What + " differs: original " + std::to_string(Original) + ", extracted " + std::to_string(Extracted));
}

double TMoverParameters::shadow_series_motor_current(double const U, double const n, double const Mn, double const Rz, int const SP) const
{
	double MotorCurrent;
	double U1 = U + Mn * n * motor_param(SP).fi0 * motor_param(SP).fi;
	double Isf = Sign(U1) * motor_param(SP).Isat;
	double Delta = square(Isf * Rz + Mn * motor_param(SP).fi * n - U1) + 4.0 * U1 * Isf * Rz;
	if (U > 0)
		MotorCurrent = (U1 - Isf * Rz - Mn * motor_param(SP).fi * n + std::sqrt(Delta)) / (2.0 * Rz);
	else
		MotorCurrent = (U1 - Isf * Rz - Mn * motor_param(SP).fi * n - std::sqrt(Delta)) / (2.0 * Rz);
	return MotorCurrent;
}

double TMoverParameters::shadow_brake_rigging_shoe_force(double K, double &Ntotal) const
{
	double u = (BrakePress * P2FTrans - BrakeCylSpring) * BrakeCylMult[0] - BrakeSlckAdj;
	if (u * BrakeRigEff > Ntotal) // histereza na nacisku klockow
		Ntotal = u * BrakeRigEff;
	else
	{
		u = (BrakePress * P2FTrans - BrakeCylSpring) * BrakeCylMult[0] - BrakeSlckAdj;
		if (u * (2.0 - BrakeRigEff) < Ntotal) // histereza na nacisku klockow
			Ntotal = u * (2.0 - BrakeRigEff);
	}

	auto const NBrakeAxles{NAxles};

	if (NBrakeAxles * NBpA > 0)
	{
		if (Ntotal > 0) // nie luz
			K += Ntotal; // w kN
		K *= static_cast<double>(BrakeCylNo) / (NBrakeAxles * static_cast<double>(NBpA)); // w kN na os
	}
	return K;
}

void TMoverParameters::shadow_induction_motor(double const PosRatio, double tmp, double const dt, double *Eimv, double &Eimvpr, double &Vaddcopy, induction_motor_outputs &Output) const
{
	Eimvpr += std::max(std::min(PosRatio - Eimvpr, 0.02), -0.02) * 12 * tmp /*2{+4*byte(PosRatio<Eimvpr)*/ * dt; // wartość zadana/procent czegoś

	if (DynamicBrakeFlag)
		tmp = eimc[eimc_f_Uzh];
	else
		tmp = eimc[eimc_f_Uzmax];

	auto f_cfu{DynamicBrakeFlag ? eimc[eimc_f_cfuH] : eimc[eimc_f_cfu]};

	Eimv[eimv_Uzsmax] = std::min(EngineVoltage - eimc[eimc_f_DU], tmp);
	Eimv[eimv_fkr] = Eimv[eimv_Uzsmax] / f_cfu;
	if (Eimvpr < 0)
	{
		Eimv[eimv_Pmax] = eimc[eimc_p_Ph];
	}
	else
	{
		Eimv[eimv_Pmax] = std::min(eimc[eimc_p_Pmax], 0.001 * EngineVoltage * (eimc[eimc_p_Imax] - eimc[eimc_f_I0]) * Pirazy2 * eimc[eimc_s_cim] / eimc[eimc_s_p] / eimc[eimc_s_cfu]);
	}
	Eimv[eimv_FMAXMAX] = 0.001 *
	                     square(std::min(1.0, Eimv[eimv_fkr] / std::max(abs(enrot) * eimc[eimc_s_p] + eimc[eimc_s_dfmax] * Eimv[eimv_ks], eimc[eimc_s_dfmax])) * f_cfu / eimc[eimc_s_cfu]) *
	                     (eimc[eimc_s_dfmax] * eimc[eimc_s_dfic] * eimc[eimc_s_cim]) * Transmision.Ratio * NPoweredAxles * 2.0 / WheelDiameter;
	if (Eimvpr < 0)
	{
		Eimv[eimv_Fful] = std::min(eimc[eimc_p_Ph] * 3.6 / (Vel != 0.0 ? Vel : 0.001), std::min(eimc[eimc_p_Fh], Eimv[eimv_FMAXMAX]));
		Eimv[eimv_Fmax] = -Sign(V) * DirAbsolute*std::min(eimc[eimc_p_Ph] * 3.6 / (Vel != 0.0 ? Vel : 0.001), std::min(-eimc[eimc_p_Fh] * Eimvpr, Eimv[eimv_FMAXMAX]));
		double pr = Eimvpr;
		if (EIMCLogForce)
			pr = -log(1 - 4 * pr) / log(5);
		Eimv[eimv_Fr] = -Sign(V) * DirAbsolute*std::min(eimc[eimc_p_Ph] * 3.6 / (Vel != 0.0 ? Vel : 0.001), std::min(-eimc[eimc_p_Fh] * pr, Eimv[eimv_FMAXMAX]));
		if (InvertersRatio < 1.0)
			Eimv[eimv_Fful] = 0;
		//*std::min(1,(Vel-eimc[eimc_p_Vh0])/(eimc[eimc_p_Vh1]-eimc[eimc_p_Vh0]))
	}
	else
	{
		Eimv[eimv_Fful] = std::min(std::min(3.6 * Eimv[eimv_Pmax] / std::max(Vel, 1.), eimc[eimc_p_F0] - Vel * eimc[eimc_p_a1]), Eimv[eimv_FMAXMAX]);
		//           if(not Flat)then
		Eimv[eimv_Fmax] = Eimv[eimv_Fful] * Eimvpr;
		//           else
		//             Eimv[eimv_Fmax]:=std::min(eimc[eimc_p_F0]*Eimvpr,Eimv[eimv_Fful]);
		double pr = Eimvpr;
		if (EIMCLogForce)
			pr = log(1 + 4 * pr) / log(5);
		Eimv[eimv_Fr] = Eimv[eimv_Fful] * pr;
	}
	Eimv[eimv_ks] = Eimv[eimv_Fr] / Eimv[eimv_FMAXMAX];
	Eimv[eimv_df] = Eimv[eimv_ks] * eimc[eimc_s_dfmax];
	Eimv[eimv_fp] = DirAbsolute * enrot * eimc[eimc_s_p] + Eimv[eimv_df]; // do przemyslenia dzialanie pp z tmpV
	//         Eimv[eimv_U]:=std::max(Eimv[eimv_Uzsmax],std::min(eimc[eimc_f_cfu]*Eimv[eimv_fp],Eimv[eimv_Uzsmax]));
	//         Eimv[eimv_pole]:=Eimv[eimv_U]/(Eimv[eimv_fp]*eimc[eimc_s_cfu]);
	if (abs(Eimv[eimv_fp]) <= Eimv[eimv_fkr])
		Eimv[eimv_pole] = f_cfu / eimc[eimc_s_cfu];
	else
		Eimv[eimv_pole] = Eimv[eimv_Uzsmax] / eimc[eimc_s_cfu] / abs(Eimv[eimv_fp]);
	Eimv[eimv_U] = Eimv[eimv_pole] * Eimv[eimv_fp] * eimc[eimc_s_cfu];
	Eimv[eimv_Ic] = (Eimv[eimv_fp] - DirAbsolute * enrot * eimc[eimc_s_p]) * eimc[eimc_s_dfic] * Eimv[eimv_pole];
	Eimv[eimv_If] = Eimv[eimv_Ic] * eimc[eimc_s_icif];
	Eimv[eimv_M] = Eimv[eimv_pole] * Eimv[eimv_Ic] * eimc[eimc_s_cim];
	Eimv[eimv_Ipoj] = Eimv[eimv_Ic] * NPoweredAxles * InvertersRatio * Eimv[eimv_U] / (EngineVoltage - eimc[eimc_f_DU]) + eimc[eimc_f_I0];
	Eimv[eimv_Pm] = DirActive * Eimv[eimv_M] * NPoweredAxles * InvertersRatio * enrot * Pirazy2 / 1000;
	Eimv[eimv_Pe] = Eimv[eimv_Ipoj] * EngineVoltage / 1000;
	Eimv[eimv_eta] = Eimv[eimv_Pm] / Eimv[eimv_Pe];

	Output.Im = Eimv[eimv_If];
	if (Eimv[eimv_Ipoj] >= 0)
		Vaddcopy *= 1.0 - 2.0 * dt;
	else if (std::abs(EngineVoltage) < EnginePowerSource.CollectorParameters.MaxV)
		Vaddcopy *= 1.0 - dt;
	else
		Vaddcopy = std::max(Vaddcopy * (1.0 - 0.2 * dt), 0.007 * (std::abs(EngineVoltage) - (EnginePowerSource.CollectorParameters.MaxV - 100)));
	Output.Itot = Eimv[eimv_Ipoj] * (0.01 + std::min(0.99, 0.99 - Vaddcopy));

	Output.EnginePower = abs(Eimv[eimv_Ic] * Eimv[eimv_U] * NPoweredAxles) / 1000;
	Output.Mm = Eimv[eimv_M] * DirAbsolute;
	Output.Mw = Output.Mm * Transmision.Ratio * Transmision.Efficiency;
	Output.Fw = Output.Mw * 2.0 / WheelDiameter;
	Output.Ft = Output.Fw * NPoweredAxles * InvertersRatio;
	Eimv[eimv_Fr] = DirAbsolute * Output.Ft / 1000;
}

#endif
