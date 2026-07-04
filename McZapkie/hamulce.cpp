/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/
/*
MaSzyna EU07 - SPKS
Brakes. Oerlikon ESt.
Copyright (C) 2007-2014 Maciej Cierniak
*/

#include "stdafx.h"
#include "hamulce.h"
#include <typeinfo>
#include "MOVER.h"
#include "utilities/utilities.h"

//---FUNKCJE OGOLNE---

static double const DPL = 0.25;
double const TFV4aM::pos_table[11] = {-2, 6, -1, 0, -2, 1, 4, 6, 0, 0, 0};
double const TMHZ_EN57::pos_table[11] = {-1, 10, -1, 0, 0, 2, 9, 10, 0, 0, 0};
double const TMHZ_K5P::pos_table[11] = {-1, 3, -1, 0, 1, 1, 2, 3, 0, 0, 0};
double const TMHZ_6P::pos_table[11] = {-1, 4, -1, 0, 2, 2, 3, 4, 0, 0, 0};
double const TM394::pos_table[11] = {-1, 5, -1, 0, 1, 2, 4, 5, 0, 0, 0};
double const TH14K1::BPT_K[6][2] = {{10, 0}, {4, 1}, {0, 1}, {4, 0}, {4, -1}, {15, -1}};
double const TH14K1::pos_table[11] = {-1, 4, -1, 0, 1, 2, 3, 4, 0, 0, 0};
double const TSt113::BPT_K[6][2] = {{10, 0}, {4, 1}, {0, 1}, {4, 0}, {4, -1}, {15, -1}};
double const TSt113::BEP_K[7] = {0, -1, 1, 0, 0, 0, 0};
double const TSt113::pos_table[11] = {-1, 5, -1, 0, 2, 3, 4, 5, 0, 0, 1};
double const TFVel6::pos_table[11] = {-1, 6, -1, 0, 6, 4, 4.7, 5, -1, 0, 1};
double const TFVE408::pos_table[11] = {0, 10, 0, 0, 10, 7, 8, 9, 0, 1, 5};

/// <summary>
/// Pressure-ratio helper used by reservoir filling/emptying integrators.
/// Returns the dimensionless driving term (P2 - P1) / (1.13 * PH - PL),
/// where PH/PL are the absolute high/low pressures (with a small safety margin),
/// signed in the direction of P1 -&gt; P2.
/// </summary>
/// <param name="P1">Source pressure [bar].</param>
/// <param name="P2">Destination pressure [bar].</param>
/// <returns>Dimensionless flow driver (positive when P2 &gt; P1).</returns>
double PR(double P1, double P2)
{
	const double PH = std::max(P1, P2) + 0.1;
	const double PL = P1 + P2 - PH + 0.2;
	return (P2 - P1) / (1.13 * PH - PL);
}

/// <summary>
/// Legacy pneumatic flow function. Kept for reference; superseded by <see cref="PF"/>.
/// </summary>
/// <param name="P1">Source pressure [bar].</param>
/// <param name="P2">Destination pressure [bar].</param>
/// <param name="S">Effective orifice cross-section.</param>
/// <returns>Volumetric flow rate (signed).</returns>
double PF_old(double P1, double P2, double S)
{
	const double PH = std::max(P1, P2) + 1;
	const double PL = P1 + P2 - PH + 2;
	if (PH - PL < 0.0001)
		return 0;
	else if (PH - PL < 0.05)
		return 20 * (PH - PL) * (PH + 1) * 222 * S * (P2 - P1) / (1.13 * PH - PL);
	else
		return (PH + 1) * 222 * S * (P2 - P1) / (1.13 * PH - PL);
}

/// <summary>
/// Pneumatic flow rate from one pressure to another through an orifice of area S.
/// Distinguishes choked (PL/PH below 0.5) from subsonic flow and softens the
/// curve near zero pressure delta using a DP-wide ramp to keep the integrator stable.
/// </summary>
/// <param name="P1">Source pressure [bar].</param>
/// <param name="P2">Destination pressure [bar].</param>
/// <param name="S">Effective orifice cross-section.</param>
/// <param name="DP">Soft-clip pressure delta — softens the response for tiny PH-PL differences (default 0.25).</param>
/// <returns>Volumetric flow rate (signed; positive when P2 &gt; P1).</returns>
double PF(double const P1, double const P2, double const S, double const DP)
{
	double const PH = std::max(P1, P2) + 1.0; // wyzsze cisnienie absolutne
	double const PL = P1 + P2 - PH + 2.0; // nizsze cisnienie absolutne
	double const sg = PL / PH; // bezwymiarowy stosunek cisnien
	double const FM = PH * 197.0 * S * Sign(P2 - P1); // najwyzszy mozliwy przeplyw, wraz z kierunkiem
	if (sg > 0.5) // jesli ponizej stosunku krytycznego
		if (PH - PL < DP) // niewielka roznica cisnien
			return (1.0 - sg) / DPL * FM * 2.0 * std::sqrt(DP * (PH - DP));
		//      return 1/DPL*(PH-PL)*fm*2*SQRT((sg)*(1-sg));
		else
			return FM * 2.0 * std::sqrt(sg * (1.0 - sg));
	else // powyzej stosunku krytycznego
		return FM;
}

/// <summary>
/// Variant of <see cref="PF"/> that uses the dimensionless pressure ratio (sg)
/// for the soft-clip threshold instead of an absolute pressure delta.
/// </summary>
/// <param name="P1">Source pressure [bar].</param>
/// <param name="P2">Destination pressure [bar].</param>
/// <param name="S">Effective orifice cross-section.</param>
/// <returns>Volumetric flow rate (signed).</returns>
double PF1(double const P1, double const P2, double const S)
{
	static double const DPS = 0.001;

	double const PH = std::max(P1, P2) + 1.0; // wyzsze cisnienie absolutne
	double const PL = P1 + P2 - PH + 2.0; // nizsze cisnienie absolutne
	double const sg = PL / PH; // bezwymiarowy stosunek cisnien
	double const FM = PH * 197.0 * S * Sign(P2 - P1); // najwyzszy mozliwy przeplyw, wraz z kierunkiem
	if (sg > 0.5) // jesli ponizej stosunku krytycznego
		if (sg < DPS) // niewielka roznica cisnien
			return (1.0 - sg) / DPS * FM * 2.0 * std::sqrt(DPS * (1.0 - DPS));
		else
			return FM * 2.0 * std::sqrt(sg * (1.0 - sg));
	else // powyzej stosunku krytycznego
		return FM;
}

/// <summary>
/// Filling valve flow: flows from PH to PL until PL reaches LIM. The valve
/// throttles smoothly as PL approaches LIM (within DP of the target).
/// Returns 0 once PL is already &gt;= LIM.
/// </summary>
/// <param name="PH">High-pressure source [bar].</param>
/// <param name="PL">Low-pressure side that is being filled [bar].</param>
/// <param name="S">Effective orifice cross-section.</param>
/// <param name="LIM">Target pressure for PL [bar] — flow stops once PL reaches LIM.</param>
/// <param name="DP">Throttling distance from LIM (default 0.1).</param>
/// <returns>Flow rate from PH to PL (positive into PL).</returns>
double PFVa(double PH, double PL, double const S, double LIM, double const DP)
// zawor napelniajacy z PH do PL, PL do LIM
{
	if (LIM > PL)
	{
		LIM = LIM + 1;
		PH = PH + 1; // wyzsze cisnienie absolutne
		PL = PL + 1; // nizsze cisnienie absolutne
		const double sg = std::min(1.0, PL / PH); // bezwymiarowy stosunek cisnien. NOTE: sg is capped at 1 to prevent calculations from going awry. TODO, TBD: log these as errors?
		double FM = PH * 197 * S; // najwyzszy mozliwy przeplyw, wraz z kierunkiem
		if (LIM - PL < DP)
			FM = FM * (LIM - PL) / DP; // jesli jestesmy przy nastawieniu, to zawor sie przymyka
		if (sg > 0.5) // jesli ponizej stosunku krytycznego
			if (PH - PL < DPL) // niewielka roznica cisnien
				return (PH - PL) / DPL * FM * 2 * std::sqrt(sg * (1 - sg)); // BUG: (1-sg) can be < 0, leading to sqrt(-x)
			else
				return FM * 2 * std::sqrt(sg * (1 - sg)); // BUG: (1-sg) can be < 0, leading to sqrt(-x)
		else // powyzej stosunku krytycznego
			return FM;
	}
	else
		return 0;
}

/// <summary>
/// Venting valve flow: flows from PH to PL until PH falls to LIM. The valve
/// throttles smoothly as PH approaches LIM. Returns 0 once PH &lt;= LIM.
/// </summary>
/// <param name="PH">High-pressure side that is being vented [bar].</param>
/// <param name="PL">Low-pressure destination [bar].</param>
/// <param name="S">Effective orifice cross-section.</param>
/// <param name="LIM">Lower bound for PH [bar] — flow stops once PH falls to LIM.</param>
/// <param name="DP">Throttling distance from LIM (default 0.1).</param>
/// <returns>Flow rate from PH to PL.</returns>
double PFVd(double PH, double PL, double const S, double LIM, double const DP)
// zawor wypuszczajacy z PH do PL, PH do LIM
{
	if (LIM < PH)
	{
		LIM = LIM + 1;
		PH = PH + 1.0; // wyzsze cisnienie absolutne
		PL = PL + 1.0; // nizsze cisnienie absolutne
		const double sg = std::min(1.0, PL / PH); // bezwymiarowy stosunek cisnien
		double FM = PH * 197.0 * S; // najwyzszy mozliwy przeplyw, wraz z kierunkiem
		if (PH - LIM < 0.1)
			FM = FM * (PH - LIM) / DP; // jesli jestesmy przy nastawieniu, to zawor sie przymyka
		if (sg > 0.5) // jesli ponizej stosunku krytycznego
			if (PH - PL < DPL) // niewielka roznica cisnien
				return (PH - PL) / DPL * FM * 2.0 * std::sqrt(sg * (1.0 - sg));
			else
				return FM * 2.0 * std::sqrt(sg * (1.0 - sg));
		else // powyzej stosunku krytycznego
			return FM;
	}
	else
		return 0;
}

/// <summary>
/// Returns true when a continuous handle position falls into a unit-wide
/// detent centred on i_pos (i.e. pos is in (i_pos - 0.5, i_pos + 0.5]).
/// </summary>
/// <param name="pos">Continuous handle position.</param>
/// <param name="i_pos">Detent centre to compare against.</param>
/// <returns>True if pos is within ±0.5 of i_pos.</returns>
bool is_EQ(double pos, double i_pos)
{
	return pos <= i_pos + 0.5 && pos > i_pos - 0.5;
}

//---ZBIORNIKI---

/// <summary>
/// Returns absolute (atmospheric) pressure inside the reservoir as 0.1 * Vol / Cap.
/// </summary>
/// <returns>Absolute pressure value.</returns>
double TReservoir::pa()
{
	return 0.1 * Vol / Cap;
}

/// <summary>
/// Returns gauge pressure inside the reservoir as Vol / Cap.
/// </summary>
/// <returns>Pressure in bar.</returns>
double TReservoir::P()
{
	return Vol / Cap;
}

/// <summary>
/// Accumulates a flow delta into the pending volume change for the current step.
/// The change is applied to <c>Vol</c> on the next call to <see cref="Act"/>.
/// </summary>
/// <param name="dv">Flow delta (positive = into reservoir, negative = out).</param>
void TReservoir::Flow(double dv)
{
	dVol = dVol + dv;
}

/// <summary>
/// Commits the pending flow accumulated by <see cref="Flow"/> to <c>Vol</c>
/// (clamping it to non-negative) and resets the pending delta to zero.
/// </summary>
void TReservoir::Act()
{
	Vol = std::max(0.0, Vol + dVol);
	dVol = 0;
}

/// <summary>
/// Sets the reservoir capacity in liters.
/// </summary>
/// <param name="Capacity">Capacity in liters.</param>
void TReservoir::CreateCap(double Capacity)
{
	Cap = Capacity;
}

/// <summary>
/// Initialises the reservoir to a given pressure (Vol = Cap * Press)
/// and clears any pending flow.
/// </summary>
/// <param name="Press">Initial pressure in bar.</param>
void TReservoir::CreatePress(double Press)
{
	Vol = Cap * Press;
	dVol = 0;
}

//---SILOWNIK---
/// <summary>
/// Returns the absolute pressure inside the brake cylinder
/// (gauge pressure from <see cref="P"/> scaled to atm).
/// </summary>
/// <returns>Absolute brake-cylinder pressure.</returns>
double TBrakeCyl::pa()
// var VtoC: real;
{
	//  VtoC:=Vol/Cap;
	return P() * 0.1;
}

/* NOWSZA WERSJA - maksymalne ciśnienie to ok. 4,75 bar, co powoduje
//                 problemy przy rapidzie w lokomotywach, gdyz jest3
//                 osiagany wierzcholek paraboli
function TBrakeCyl.P:real;
var VtoC: real;
begin
  VtoC:=Vol/Cap;
  if VtoC<0.06 then P:=VtoC/4
  else if VtoC>0.88 then P:=0.5+(VtoC-0.88)*1.043-0.064*(VtoC-0.88)*(VtoC-0.88)
  else P:=0.15+0.35/0.82*(VtoC-0.06);
end; */

//(* STARA WERSJA
/// <summary>
/// Returns the gauge pressure inside the brake cylinder using the legacy
/// piston-stroke pressure curve: a low-pressure dead-volume region (VtoC &lt; VS),
/// a linear stroke region, and a saturated region once the piston is fully extended.
/// Includes a div-by-zero guard for vehicles with incomplete definitions.
/// </summary>
/// <returns>Cylinder pressure in bar.</returns>
double TBrakeCyl::P()
{
	static double const VS = 0.005;
	static double const pS = 0.05;
	static double const VD = 1.10;
	static double const cD = 1;
	static double const pD = VD - cD;

	const double VtoC = Cap > 0.0 ? Vol / Cap : 0.0; // stosunek cisnienia do objetosci.
	                                             // Added div/0 trap for vehicles with incomplete definitions (cars etc)
	//  P:=VtoC;
	if (VtoC < VS)
		return VtoC * pS / VS; // objetosc szkodliwa
	else if (VtoC > VD)
		return VtoC - cD; // caly silownik;
	else
		return pS + (VtoC - VS) / (VD - VS) * (pD - pS); // wysuwanie tloka
} //*)

//---HAMULEC---
/*
constructor TBrake.Create(i_mbp, i_bcr, i_bcd, i_brc: real; i_bcn, i_BD, i_mat, i_ba, i_nbpa: int);
begin
  inherited Create;
  MaxBP:=i_mbp;
  BCN:=i_bcn;
  BCA:=i_bcn*i_bcr*i_bcr*pi;
  BA:=i_ba;
  NBpA:=i_nbpa;
  BrakeDelays:=i_BD;

//tworzenie zbiornikow
  BrakeCyl.CreateCap(i_bcd*BCA*1000);
  BrakeRes.CreateCap(i_brc);
  ValveRes.CreateCap(0.2);

//  FM.Free;
//materialy cierne
  case i_mat of
  bp_P10Bg:   FM:=TP10Bg.Create;
  bp_P10Bgu:  FM:=TP10Bgu.Create;
  else //domyslnie
  FM:=TP10.Create;
  end;


end  ; */

/// <summary>
/// Constructs the brake unit: stores parameters, creates the cylinder /
/// auxiliary reservoir / valve pre-chamber, sizes them for the requested
/// 14"-relative scale, and instantiates the friction-material model based on
/// the friction-pair id (with the magnetic-rail flag bp_MHS masked off).
/// </summary>
/// <param name="i_mbp">Maximum brake cylinder pressure [bar].</param>
/// <param name="i_bcr">Brake cylinder radius [m].</param>
/// <param name="i_bcd">Brake cylinder working stroke [m].</param>
/// <param name="i_brc">Auxiliary reservoir capacity [l].</param>
/// <param name="i_bcn">Number of brake cylinders.</param>
/// <param name="i_BD">Available brake delay positions (bdelay_* bitmask).</param>
/// <param name="i_mat">Friction material id (bp_* constant, optionally OR'ed with bp_MHS).</param>
/// <param name="i_ba">Number of braked axles.</param>
/// <param name="i_nbpa">Number of blocks per axle.</param>
TBrake::TBrake(double i_mbp, double i_bcr, double i_bcd, double i_brc, int i_bcn, int i_BD, int i_mat, int i_ba, int i_nbpa)
{
	// inherited:: Create;
	MaxBP = i_mbp;
	BCN = i_bcn;
	BCM = 1;
	BCA = i_bcn * i_bcr * i_bcr * M_PI;
	BA = i_ba;
	NBpA = i_nbpa;
	BrakeDelays = i_BD;
	BrakeDelayFlag = bdelay_P;
	// 210.88
	//  SizeBR:=i_bcn*i_bcr*i_bcr*i_bcd*40.17*MaxBP/(5-MaxBP);  //objetosc ZP w stosunku do cylindra
	//  14" i cisnienia 4.2 atm
	SizeBR = i_brc * 0.0128;
	SizeBC = i_bcn * i_bcr * i_bcr * i_bcd * 210.88 * MaxBP / 4.2; // objetosc CH w stosunku do cylindra 14" i cisnienia 4.2 atm

	BrakeCyl = std::make_shared<TBrakeCyl>();
	BrakeRes = std::make_shared<TReservoir>();
	ValveRes = std::make_shared<TReservoir>();

	// tworzenie zbiornikow
	BrakeCyl->CreateCap(i_bcd * BCA * 1000);
	BrakeRes->CreateCap(i_brc);
	ValveRes->CreateCap(0.25);

	// materialy cierne
	i_mat = i_mat & 255 - bp_MHS;
	switch (i_mat)
	{
	case bp_P10Bg:
		FM = std::make_shared<TP10Bg>();
		break;
	case bp_P10Bgu:
		FM = std::make_shared<TP10Bgu>();
		break;
	case bp_FR513:
		FM = std::make_shared<TFR513>();
		break;
	case bp_FR510:
		FM = std::make_shared<TFR510>();
		break;
	case bp_Cosid:
		FM = std::make_shared<TCosid>();
		break;
	case bp_P10yBg:
		FM = std::make_shared<TP10yBg>();
		break;
	case bp_P10yBgu:
		FM = std::make_shared<TP10yBgu>();
		break;
	case bp_D1:
		FM = std::make_shared<TDisk1>();
		break;
	case bp_D2:
		FM = std::make_shared<TDisk2>();
		break;
	default: // domyslnie
		FM = std::make_shared<TP10>();
	}
}

/// <summary>
/// Default brake initialisation — only stores the requested delay flag.
/// Derived classes typically override to also pre-charge the reservoirs.
/// </summary>
/// <param name="PP">Brake pipe pressure [bar].</param>
/// <param name="HPP">High (control) pressure [bar].</param>
/// <param name="LPP">Low pressure threshold [bar].</param>
/// <param name="BP">Initial brake cylinder pressure [bar].</param>
/// <param name="BDF">Initial brake delay flag.</param>
// inicjalizacja hamulca (stan poczatkowy)
void TBrake::Init(double const PP, double const HPP, double const LPP, double const BP, int const BDF)
{
	BrakeDelayFlag = BDF;
}

/// <summary>
/// Returns the friction coefficient between the blocks and the wheel/disc by
/// delegating to the friction-material model (note: arguments are forwarded
/// in (N, Vel) order to match the friction-material API).
/// </summary>
/// <param name="Vel">Vehicle velocity.</param>
/// <param name="N">Normal force on the block.</param>
/// <returns>Friction coefficient.</returns>
// pobranie wspolczynnika tarcia materialu
double TBrake::GetFC(double const Vel, double const N)
{
	return FM->GetFC(N, Vel);
}

/// <summary>Returns the gauge pressure inside the brake cylinder.</summary>
// cisnienie cylindra hamulcowego
double TBrake::GetBCP()
{
	return BrakeCyl->P();
}

/// <summary>Default ED reference pressure — 0 unless overridden by EP09-style distributors.</summary>
// ciśnienie sterujące hamowaniem elektro-dynamicznym
double TBrake::GetEDBCP()
{
	return 0;
}

/// <summary>Returns the auxiliary reservoir (ZP) pressure.</summary>
// cisnienie zbiornika pomocniczego
double TBrake::GetBRP()
{
	return BrakeRes->P();
}

/// <summary>Returns the valve pre-chamber pressure.</summary>
// cisnienie komory wstepnej
double TBrake::GetVRP()
{
	return ValveRes->P();
}

/// <summary>
/// Default control-reservoir pressure: forwards to the auxiliary reservoir.
/// Distributors with a real ZS override this to return CntrlRes-&gt;P().
/// </summary>
// cisnienie zbiornika sterujacego
double TBrake::GetCRP()
{
	return GetBRP();
}

/// <summary>
/// Default per-step distributor advance — commits any pending flows on the
/// reservoirs and reports zero brake-pipe exchange. Concrete distributors override.
/// </summary>
/// <param name="PP">Brake pipe pressure [bar].</param>
/// <param name="dt">Time step [s].</param>
/// <param name="Vel">Vehicle velocity [m/s].</param>
/// <returns>0 in the default implementation.</returns>
// przeplyw z przewodu glowneg
double TBrake::GetPF(double const PP, double const dt, double const Vel)
{
	ValveRes->Act();
	BrakeCyl->Act();
	BrakeRes->Act();
	return 0;
}

/// <summary>
/// Default high-pressure inflow — 0 (no replenishment from the main line).
/// Distributors with a high-pressure feed override this.
/// </summary>
// przeplyw z przewodu zasilajacego
double TBrake::GetHPFlow(double const HP, double const dt)
{
	return 0;
}

/// <summary>
/// Returns the piston force from the cylinder pressure (BCA * 100 * P).
/// </summary>
double TBrake::GetBCF()
{
	return BCA * 100 * BrakeCyl->P();
}

/// <summary>
/// Sets the brake delay flag (G/P/R/M) only if the requested mode is
/// supported by the vehicle (bit set in BrakeDelays) and differs from the
/// current setting.
/// </summary>
/// <param name="nBDF">Requested delay flag (bdelay_*).</param>
/// <returns>True on accepted change, false otherwise.</returns>
bool TBrake::SetBDF(int const nBDF)
{
	if ((nBDF & BrakeDelays) == nBDF && nBDF != BrakeDelayFlag)
	{
		BrakeDelayFlag = nBDF;
		return true;
	}
	else
		return false;
}

/// <summary>Sets or clears the releaser flag in BrakeStatus.</summary>
/// <param name="state">1 to engage, 0 to disengage.</param>
void TBrake::Releaser(int const state)
{
	BrakeStatus = BrakeStatus & ~b_rls | state * b_rls;
}

/// <summary>Returns true if the releaser flag is currently set in BrakeStatus.</summary>
bool TBrake::Releaser() const
{

	return (BrakeStatus & b_rls) == b_rls;
}

/// <summary>
/// Default EP-state setter — no-op. Overridden by distributors that
/// support the EP brake (TWest, TEStEP1/2, ...).
/// </summary>
/// <param name="nEPS">EP intensity.</param>
void TBrake::SetEPS(double const nEPS) {}

/// <summary>
/// Sets the anti-slip brake state flags. Bit 1 of <paramref name="state"/>
/// drives <c>b_asb</c> (hold), bit 0 drives <c>b_asb_unbrake</c> (release).
/// </summary>
/// <param name="state">Two-bit ASB request.</param>
void TBrake::ASB(int const state)
{ // 255-b_asb(32)
	BrakeStatus = BrakeStatus & ~b_asb | state / 2 * b_asb;
	BrakeStatus = BrakeStatus & ~b_asb_unbrake | state % 2 * b_asb_unbrake;
}

/// <summary>Returns the raw BrakeStatus bitfield.</summary>
int TBrake::GetStatus()
{
	return BrakeStatus;
}

/// <summary>
/// Returns the accumulated SoundFlag bitfield and clears it,
/// so that each event is reported only once.
/// </summary>
int TBrake::GetSoundFlag()
{
	const int result = SoundFlag;
	SoundFlag = 0;
	return result;
}

/// <summary>Sets the anti-slip brake target pressure.</summary>
/// <param name="Press">Pressure [bar].</param>
void TBrake::SetASBP(double const Press)
{
	ASBP = Press;
}

/// <summary>
/// Vents the valve pre-chamber and the auxiliary reservoir to zero
/// (for vehicle reset / decoupling). Derived classes also clear their extra reservoirs.
/// </summary>
void TBrake::ForceEmptiness()
{
	ValveRes->CreatePress(0);
	BrakeRes->CreatePress(0);

	ValveRes->Act();
	BrakeRes->Act();
}

/// <summary>
/// Bleeds a fraction of the air from the auxiliary reservoir and a tiny
/// fraction from the valve pre-chamber to simulate distributed leaks.
/// Note: experimental — currently limited to these two reservoirs.
/// </summary>
/// <param name="Amount">Fraction of pressure to bleed (0..1).</param>
// removes specified amount of air from the reservoirs
// NOTE: experimental feature, for now limited only to brake reservoir
void TBrake::ForceLeak(double const Amount)
{

	BrakeRes->Flow(-Amount * BrakeRes->P());
	ValveRes->Flow(-Amount * ValveRes->P() * 0.01); // this reservoir has hard coded, tiny capacity compared to other parts

	BrakeRes->Act();
	ValveRes->Act();
}

//---WESTINGHOUSE---

/// <summary>
/// Initialises the Westinghouse distributor: pre-charges the valve pre-chamber
/// to PP, the brake cylinder to BP, and the auxiliary reservoir to a midpoint
/// between PP and HPP.
/// </summary>
/// <param name="PP">Brake pipe pressure [bar].</param>
/// <param name="HPP">High (control) pressure [bar].</param>
/// <param name="LPP">Low pressure threshold [bar].</param>
/// <param name="BP">Initial cylinder pressure [bar].</param>
/// <param name="BDF">Initial brake delay flag.</param>
void TWest::Init(double const PP, double const HPP, double const LPP, double const BP, int const BDF)
{
	TBrake::Init(PP, HPP, LPP, BP, BDF);
	ValveRes->CreatePress(PP);
	BrakeCyl->CreatePress(BP);
	BrakeRes->CreatePress(PP / 2 + HPP / 2);
	// BrakeStatus:=3*int(BP>0.1);
}

/// <summary>
/// One-step Westinghouse distributor advance. Drives the b_on/b_hld state
/// machine from the auxiliary-reservoir vs. valve-pre-chamber differential,
/// integrates the auxiliary brake (DCV) and the EP brake against LBP, and
/// computes the resulting flows between brake pipe / pre-chamber / auxiliary
/// reservoir / brake cylinder.
/// </summary>
/// <param name="PP">Brake pipe pressure [bar].</param>
/// <param name="dt">Time step [s].</param>
/// <param name="Vel">Vehicle velocity [m/s].</param>
/// <returns>Net volume exchanged with the brake pipe.</returns>
double TWest::GetPF(double const PP, double const dt, double const Vel)
{
	double dv;
	double dV1;
	double VVP;
	double BVP;
	double CVP;
	double BCP;
	double temp;

	BVP = BrakeRes->P();
	VVP = ValveRes->P();
	CVP = BrakeCyl->P();
	BCP = BrakeCyl->P();

	if ((BrakeStatus & b_hld) == b_hld)
		if (VVP + 0.03 < BVP)
			BrakeStatus |= b_on;
		else if (VVP > BVP + 0.1)
			BrakeStatus &= ~(b_on | b_hld);
		else if (VVP > BVP)
			BrakeStatus &= ~b_on;
		else
			;
	else if (VVP + 0.25 < BVP)
		BrakeStatus |= b_on | b_hld;

	if ((BrakeStatus & b_hld) == b_off && !DCV)
		dv = PF(0, CVP, 0.0068 * SizeBC) * dt;
	else
		dv = 0;
	BrakeCyl->Flow(-dv);

	if (BCP > LBP + 0.01 && DCV)
		dv = PF(0, CVP, 0.1 * SizeBC) * dt;
	else
		dv = 0;
	BrakeCyl->Flow(-dv);

	if ((BrakeStatus & b_rls) == b_rls) // odluzniacz
		dv = PF(0, CVP, 0.1 * SizeBC) * dt;
	else
		dv = 0;
	BrakeCyl->Flow(-dv);

	// hamulec EP
	temp = BVP * int(EPS > 0);
	dv = PF(temp, LBP, 0.0015) * dt * EPS * EPS * int(LBP * EPS < MaxBP * LoadC);
	LBP = LBP - dv;
	dv = 0;

	// przeplyw ZP <-> silowniki
	if ((BrakeStatus & b_on) == b_on && (TareBP < 0.1 || BCP < MaxBP * LoadC))
		if (BVP > LBP)
		{
			DCV = false;
			dv = PF(BVP, CVP, 0.017 * SizeBC) * dt;
		}
		else
			dv = 0;
	else
		dv = 0;
	BrakeRes->Flow(dv);
	BrakeCyl->Flow(-dv);
	if (DCV)
		dVP = PF(LBP, BCP, 0.01 * SizeBC) * dt;
	else
		dVP = 0;
	BrakeCyl->Flow(-dVP);
	if (dVP > 0)
		dVP = 0;
	// przeplyw ZP <-> rozdzielacz
	if ((BrakeStatus & b_hld) == b_off)
		dv = PF(BVP, VVP, 0.0011 * SizeBR) * dt;
	else
		dv = 0;
	BrakeRes->Flow(dv);
	dV1 = dv * 0.95;
	ValveRes->Flow(-0.05 * dv);
	// przeplyw PG <-> rozdzielacz
	dv = PF(PP, VVP, 0.01 * SizeBR) * dt;
	ValveRes->Flow(-dv);

	ValveRes->Act();
	BrakeCyl->Act();
	BrakeRes->Act();
	return dv - dV1;
}

/// <summary>
/// Returns the high-pressure inflow accumulated by the previous GetPF call
/// (the dVP variable) — used by the host to bookkeep main-reservoir consumption.
/// </summary>
double TWest::GetHPFlow(double const HP, double const dt)
{
	return dVP;
}

/// <summary>
/// Sets the auxiliary brake target pressure. If P exceeds the current
/// cylinder pressure, the double check valve (DCV) is engaged so that LBP
/// drives the cylinder.
/// </summary>
/// <param name="P">Auxiliary brake pressure [bar].</param>
void TWest::SetLBP(double const P)
{
	LBP = P;
	if (P > BrakeCyl->P())
		//   begin
		DCV = true;
	//   end
	//  else
	//    LBP:=P;
}

/// <summary>
/// Sets the EP intensity. A positive value engages the DCV; a zero value
/// while the previous EPS was non-zero latches LBP from the cylinder
/// (or clears it when below a threshold), modelling EP-release hysteresis.
/// </summary>
/// <param name="nEPS">New EP intensity.</param>
void TWest::SetEPS(double const nEPS)
{
	double BCP;

	BCP = BrakeCyl->P();
	if (nEPS > 0)
		DCV = true;
	else if (nEPS == 0)
	{
		if (EPS != 0)
		{
			if (LBP > 0.4)
				LBP = BrakeCyl->P();
			if (LBP < 0.15)
				LBP = 0;
		}
	}
	EPS = nEPS;
}

/// <summary>
/// Recomputes the load-weighing coefficient LoadC from the current vehicle
/// mass: linear interpolation between TareBP/MaxBP for masses in [TareM, LoadM],
/// and 1.0 for masses at or above LoadM.
/// </summary>
/// <param name="mass">Current vehicle mass.</param>
void TWest::PLC(double const mass)
{
	LoadC = 1 + int(mass < LoadM) * ((TareBP + (MaxBP - TareBP) * (mass - TareM) / (LoadM - TareM)) / MaxBP - 1);
}

/// <summary>
/// Stores the load-weighing parameters: tare mass, loaded mass and the
/// cylinder pressure that should be reached for the tare mass.
/// </summary>
/// <param name="TM">Tare (empty) mass.</param>
/// <param name="LM">Loaded mass.</param>
/// <param name="TBP">Tare-mass cylinder pressure.</param>
void TWest::SetLP(double const TM, double const LM, double const TBP)
{
	TareM = TM;
	LoadM = LM;
	TareBP = TBP;
}

//---OERLIKON EST4---
/// <summary>
/// Implements the releaser logic: while engaged, vents the control reservoir
/// (CntrlRes) toward the lower of valve pre-chamber / auxiliary reservoir
/// (with a 0.05 bar margin); disengages once CntrlRes has fallen below VVP.
/// </summary>
/// <param name="dt">Time step [s].</param>
void TESt::CheckReleaser(double const dt)
{
	const double VVP = std::min(ValveRes->P(), BrakeRes->P() + 0.05);
	const double CVP = CntrlRes->P() - 0.0;

	// odluzniacz
	if ((BrakeStatus & b_rls) == b_rls)
		if (CVP - VVP < 0)
			BrakeStatus &= ~b_rls;
		else
		{
			CntrlRes->Flow(+PF(CVP, 0, 0.1) * dt);
		}
}

/// <summary>
/// Drives the BrakeStatus state machine (b_on / b_hld) of the ESt main slide
/// valve from the current valve pre-chamber, brake cylinder and control
/// reservoir pressures. Triggers the accelerator (sf_Acc) at the start of
/// braking and the cylinder-vent sound flag (sf_CylU) on release.
/// </summary>
/// <param name="BCP">Brake cylinder (or impulse-chamber) pressure.</param>
/// <param name="dV1">In/out brake-pipe flow correction (unused in this base impl).</param>
void TESt::CheckState(double const BCP, double &dV1)
{

	double const VVP{ValveRes->P()};
	double const BVP{BrakeRes->P()};
	double const CVP{CntrlRes->P()};

	// sprawdzanie stanu
	if (BCP > 0.25)
	{

		if ((BrakeStatus & b_hld) == b_hld)
		{

			if (VVP + 0.003 + BCP / BVM < CVP)
			{
				// hamowanie stopniowe
				BrakeStatus |= b_on;
			}
			else
			{
				if (VVP + BCP / BVM > CVP)
				{
					// zatrzymanie napelaniania
					BrakeStatus &= ~b_on;
				}
				if (VVP - 0.003 + (BCP - 0.1) / BVM > CVP)
				{
					// luzowanie
					BrakeStatus &= ~(b_on | b_hld);
				}
			}
		}
		else
		{

			if (VVP + BCP / BVM < CVP && (CVP - VVP) * BVM > 0.25)
			{
				// zatrzymanie luzowanie
				BrakeStatus |= b_hld;
			}
		}
	}
	else
	{

		if (VVP + 0.1 < CVP)
		{
			// poczatek hamowania
			if ((BrakeStatus & b_hld) == 0)
			{
				// przyspieszacz
				ValveRes->CreatePress(0.02 * VVP);
				SoundFlag |= sf_Acc;
				ValveRes->Act();
			}
			BrakeStatus |= b_on | b_hld;
		}
	}

	if ((BrakeStatus & b_hld) == 0)
	{
		SoundFlag |= sf_CylU;
	}
}

/// <summary>
/// Returns the dimensionless opening factor of the ZS-filling slide valve
/// (control reservoir &lt;-&gt; brake pipe) for the current pre-chamber/auxiliary
/// reservoir state and brake-cylinder pressure.
/// </summary>
/// <param name="BP">Brake cylinder (or impulse) pressure.</param>
/// <returns>Opening coefficient (0 closed, 1 fully open).</returns>
double TESt::CVs(double const BP)
{
	double VVP;
	double BVP;
	double CVP;

	BVP = BrakeRes->P();
	CVP = CntrlRes->P();
	VVP = ValveRes->P();

	// przeplyw ZS <-> PG
	if (VVP < CVP - 0.12 || BVP < CVP - 0.3 || BP > 0.4)
		return 0;
	else if (VVP > CVP + 0.4)
		if (BVP > CVP + 0.2)
			return 0.23;
		else
			return 0.05;
	else if (BVP > CVP - 0.1)
		return 1;
	else
		return 0.3;
}

/// <summary>
/// Returns the dimensionless opening factor of the ZP-filling slide valve
/// (auxiliary reservoir &lt;-&gt; valve pre-chamber) as a function of brake
/// cylinder pressure, distinguishing initial filling, lap and full-pressure regions.
/// </summary>
/// <param name="BCP">Brake cylinder pressure.</param>
/// <returns>Opening coefficient.</returns>
double TESt::BVs(double const BCP)
{
	double VVP;
	double BVP;
	double CVP;

	BVP = BrakeRes->P();
	CVP = CntrlRes->P();
	VVP = ValveRes->P();

	// przeplyw ZP <-> rozdzielacz
	if (BVP < CVP - 0.3)
		return 0.6;
	else if (BCP < 0.5)
		if (VVP > CVP + 0.4)
			return 0.1;
		else
			return 0.3;
	else
		return 0;
}

/// <summary>
/// One-step Oerlikon ESt distributor advance: runs CheckState/CheckReleaser,
/// integrates the ZS &lt;-&gt; PG, ZP &lt;-&gt; cylinder, ZP &lt;-&gt; pre-chamber and
/// PG &lt;-&gt; pre-chamber flows, and returns the net brake-pipe exchange.
/// </summary>
/// <param name="PP">Brake pipe pressure [bar].</param>
/// <param name="dt">Time step [s].</param>
/// <param name="Vel">Vehicle velocity [m/s].</param>
/// <returns>Net volume exchanged with the brake pipe.</returns>
double TESt::GetPF(double const PP, double const dt, double const Vel)
{
	double dv;
	double dV1;
	double temp;
	double VVP;
	double BVP;
	double BCP;
	double CVP;

	BVP = BrakeRes->P();
	VVP = ValveRes->P();
	BCP = BrakeCyl->P();
	CVP = CntrlRes->P() - 0.0;

	dv = 0;
	dV1 = 0;

	// sprawdzanie stanu
	CheckState(BCP, dV1);
	CheckReleaser(dt);

	CVP = CntrlRes->P();
	VVP = ValveRes->P();
	// przeplyw ZS <-> PG
	temp = CVs(BCP);
	dv = PF(CVP, VVP, 0.0015 * temp) * dt;
	CntrlRes->Flow(+dv);
	ValveRes->Flow(-0.04 * dv);
	dV1 = dV1 - 0.96 * dv;

	// luzowanie
	if ((BrakeStatus & b_hld) == b_off)
		dv = PF(0, BCP, 0.0058 * SizeBC) * dt;
	else
		dv = 0;
	BrakeCyl->Flow(-dv);

	// przeplyw ZP <-> silowniki
	if ((BrakeStatus & b_on) == b_on)
		dv = PF(BVP, BCP, 0.016 * SizeBC) * dt;
	else
		dv = 0;
	BrakeRes->Flow(dv);
	BrakeCyl->Flow(-dv);

	// przeplyw ZP <-> rozdzielacz
	temp = BVs(BCP);
	//  if(BrakeStatus and b_hld)=b_off then
	if (VVP - 0.05 > BVP)
		dv = PF(BVP, VVP, 0.02 * SizeBR * temp / 1.87) * dt;
	else
		dv = 0;
	BrakeRes->Flow(dv);
	dV1 = dV1 + dv * 0.96;
	ValveRes->Flow(-0.04 * dv);
	// przeplyw PG <-> rozdzielacz
	dv = PF(PP, VVP, 0.01) * dt;
	ValveRes->Flow(-dv);

	ValveRes->Act();
	BrakeCyl->Act();
	BrakeRes->Act();
	CntrlRes->Act();
	return dv - dV1;
}

/// <summary>
/// Initialises the ESt distributor: pre-charges valve / brake / control
/// reservoirs (the ZS is sized at 15 l), clears BrakeStatus and computes the
/// brake-pipe to brake-cylinder transmission ratio (BVM = MaxBP / (HPP-LPP)).
/// </summary>
/// <param name="PP">Brake pipe pressure [bar].</param>
/// <param name="HPP">High (control) pressure [bar].</param>
/// <param name="LPP">Low pressure threshold [bar].</param>
/// <param name="BP">Initial cylinder pressure [bar].</param>
/// <param name="BDF">Initial brake delay flag.</param>
void TESt::Init(double const PP, double const HPP, double const LPP, double const BP, int const BDF)
{
	TBrake::Init(PP, HPP, LPP, BP, BDF);
	ValveRes->CreatePress(PP);
	BrakeCyl->CreatePress(BP);
	BrakeRes->CreatePress(PP);
	CntrlRes->CreateCap(15);
	CntrlRes->CreatePress(HPP);
	BrakeStatus = b_off;

	BVM = 1.0 / (HPP - LPP) * MaxBP;

	BrakeDelayFlag = BDF;
}

/// <summary>
/// Stub for setting ESt-specific characteristic parameters; reserved for
/// derived variants and currently a no-op.
/// </summary>
/// <param name="i_crc">Characteristic value.</param>
void TESt::EStParams(double const i_crc) {}

/// <summary>Returns the control reservoir (ZS) pressure.</summary>
double TESt::GetCRP()
{
	return CntrlRes->P();
}

/// <summary>
/// Vents the valve, brake and control reservoirs to zero
/// (used on vehicle reset / decoupling).
/// </summary>
void TESt::ForceEmptiness()
{

	ValveRes->CreatePress(0);
	BrakeRes->CreatePress(0);
	CntrlRes->CreatePress(0);

	ValveRes->Act();
	BrakeRes->Act();
	CntrlRes->Act();
}

//---EP2---

/// <summary>
/// Initialises the EP2-equipped distributor: chains TLSt::Init, sizes the
/// 1-litre impulse chamber (ImplsRes), pre-charges it to BP, sets the
/// auxiliary reservoir to PP and locks the brake delay to P.
/// </summary>
/// <param name="PP">Brake pipe pressure [bar].</param>
/// <param name="HPP">High (control) pressure [bar].</param>
/// <param name="LPP">Low pressure threshold [bar].</param>
/// <param name="BP">Initial cylinder pressure [bar].</param>
/// <param name="BDF">Initial brake delay flag (overridden to P).</param>
void TEStEP2::Init(double const PP, double const HPP, double const LPP, double const BP, int const BDF)
{
	TLSt::Init(PP, HPP, LPP, BP, BDF);
	ImplsRes->CreateCap(1);
	ImplsRes->CreatePress(BP);

	BrakeRes->CreatePress(PP);

	BrakeDelayFlag = bdelay_P;
	BrakeDelays = bdelay_P;
}

/// <summary>
/// One-step distributor advance for the ESt + EP2 combination. Drives the
/// pneumatic state machine via the impulse chamber, runs the EP brake
/// integrator (<see cref="EPCalc"/>) and computes the cylinder fill/release
/// against the higher of the impulse-chamber and EP-driven LBP target.
/// </summary>
/// <param name="PP">Brake pipe pressure [bar].</param>
/// <param name="dt">Time step [s].</param>
/// <param name="Vel">Vehicle velocity [m/s].</param>
/// <returns>Net volume exchanged with the brake pipe.</returns>
double TEStEP2::GetPF(double const PP, double const dt, double const Vel)
{
	double result;
	double dv;
	double dV1;
	double temp;
	double VVP;
	double BVP;
	double BCP;
	double CVP;

	BVP = BrakeRes->P();
	VVP = ValveRes->P();
	BCP = ImplsRes->P();
	CVP = CntrlRes->P(); // 110115 - konsultacje warszawa1

	dv = 0;
	dV1 = 0;

	// odluzniacz
	CheckReleaser(dt);

	// sprawdzanie stanu
	if ((BrakeStatus & b_hld) == b_hld && BCP > 0.25)
		if (VVP + 0.003 + BCP / BVM < CVP - 0.12)
			BrakeStatus |= b_on; // hamowanie stopniowe;
		else if (VVP - 0.003 + BCP / BVM > CVP - 0.12)
			BrakeStatus &= ~(b_on | b_hld); // luzowanie;
		else if (VVP + BCP / BVM > CVP - 0.12)
			BrakeStatus &= ~b_on; // zatrzymanie napelaniania;
		else
			;
	else if (VVP + 0.10 < CVP - 0.12 && BCP < 0.25) // poczatek hamowania
	{
		// if ((BrakeStatus & 1) == 0)
		//{
		//    //       ValveRes.CreatePress(0.5*VVP);  //110115 - konsultacje warszawa1
		//    //       SoundFlag:=SoundFlag or sf_Acc;
		//    //       ValveRes.Act;
		//}
		BrakeStatus |= b_on | b_hld;
	}
	else if (VVP + BCP / BVM < CVP - 0.12 && BCP > 0.25) // zatrzymanie luzowanie
		BrakeStatus |= b_hld;

	if ((BrakeStatus & b_hld) == 0)
	{
		SoundFlag |= sf_CylU;
	}

	// przeplyw ZS <-> PG
	if (BVP < CVP - 0.2 || BrakeStatus != b_off || BCP > 0.25)
		temp = 0;
	else if (VVP > CVP + 0.4)
		temp = 0.1;
	else
		temp = 0.5;

	dv = PF(CVP, VVP, 0.0015 * temp / 1.8) * dt;
	CntrlRes->Flow(+dv);
	ValveRes->Flow(-0.04 * dv);
	dV1 = dV1 - 0.96 * dv;

	// hamulec EP
	EPCalc(dt);

	// luzowanie KI
	if ((BrakeStatus & b_hld) == b_off)
		dv = PF(0, BCP, 0.00083) * dt;
	else
		dv = 0;
	ImplsRes->Flow(-dv);
	// przeplyw ZP <-> KI
	if ((BrakeStatus & b_on) == b_on && BCP < MaxBP * LoadC)
		dv = PF(BVP, BCP, 0.0006) * dt;
	else
		dv = 0;
	BrakeRes->Flow(dv);
	ImplsRes->Flow(-dv);
	// przeplyw PG <-> rozdzielacz
	dv = PF(PP, VVP, 0.01 * SizeBR) * dt;
	ValveRes->Flow(-dv);

	result = dv - dV1;

	temp = std::max(BCP, LBP);

	if (ImplsRes->P() > LBP + 0.01)
		LBP = 0;

	// luzowanie CH
	if (BrakeCyl->P() > temp + 0.005 || std::max(ImplsRes->P(), 8 * LBP) < 0.05)
		dv = PF(0, BrakeCyl->P(), 0.25 * SizeBC * (0.01 + (BrakeCyl->P() - temp))) * dt;
	else
		dv = 0;
	BrakeCyl->Flow(-dv);
	// przeplyw ZP <-> CH
	if (BrakeCyl->P() < temp - 0.005 && std::max(ImplsRes->P(), 8 * LBP) > 0.10 && std::max(BCP, LBP) < MaxBP * LoadC)
		dv = PF(BVP, BrakeCyl->P(), 0.35 * SizeBC * (0.01 - (BrakeCyl->P() - temp))) * dt;
	else
		dv = 0;
	BrakeRes->Flow(dv);
	BrakeCyl->Flow(-dv);

	ImplsRes->Act();
	ValveRes->Act();
	BrakeCyl->Act();
	BrakeRes->Act();
	CntrlRes->Act();
	return result;
}

/// <summary>
/// Recomputes the load-weighing coefficient LoadC for the current vehicle
/// mass (linear between TareBP/MaxBP for TareM..LoadM, capped at 1.0).
/// </summary>
/// <param name="mass">Current vehicle mass.</param>
void TEStEP2::PLC(double const mass)
{
	LoadC = 1 + int(mass < LoadM) * ((TareBP + (MaxBP - TareBP) * (mass - TareM) / (LoadM - TareM)) / MaxBP - 1);
}

/// <summary>
/// Sets the EP intensity. When the EP is energised and the pneumatic cylinder
/// pressure exceeds the EP target, latches the EP target up to the cylinder
/// pressure to avoid releasing the pneumatic brake while EP is active.
/// </summary>
/// <param name="nEPS">New EP intensity.</param>
void TEStEP2::SetEPS(double const nEPS)
{
	EPS = nEPS;
	if (EPS > 0 && LBP + 0.01 < BrakeCyl->P())
		LBP = BrakeCyl->P();
}

/// <summary>Stores the load-weighing parameters.</summary>
/// <param name="TM">Tare (empty) mass.</param>
/// <param name="LM">Loaded mass.</param>
/// <param name="TBP">Tare-mass cylinder pressure.</param>
void TEStEP2::SetLP(double const TM, double const LM, double const TBP)
{
	TareM = TM;
	LoadM = LM;
	TareBP = TBP;
}

/// <summary>
/// EP2 EP-flow integrator: drives LBP toward the auxiliary-reservoir pressure
/// when EPS &gt; 0 (apply) or toward 0 when EPS &lt; 0 (release), with a
/// quadratic-in-EPS rate and a load-weighing-aware ceiling at MaxBP * LoadC.
/// </summary>
/// <param name="dt">Time step [s].</param>
void TEStEP2::EPCalc(double dt)
{
	const double temp = BrakeRes->P() * int(EPS > 0);
	const double dv = PF(temp, LBP, 0.00053 + 0.00060 * int(EPS < 0)) * dt * EPS * EPS * int(LBP * EPS < MaxBP * LoadC);
	LBP = LBP - dv;
}

/// <summary>
/// EP1 (proportional) EP-flow integrator: interprets the fractional part of
/// EPS as a continuous EP target and drives LBP toward MaxBP * LoadC * frac
/// (clamped to the auxiliary reservoir pressure), softening the valve
/// opening with a clamped error term S.
/// </summary>
/// <param name="dt">Time step [s].</param>
void TEStEP1::EPCalc(double dt)
{
	const double temp = EPS - std::floor(EPS); // część ułamkowa jest hamulcem EP
	const double LBPLim = std::min(MaxBP * LoadC * temp, BrakeRes->P()); // do czego dążymy
	const double S = 10 * std::clamp(LBPLim - LBP, -0.1, 0.1); // przymykanie zaworku
	const double dv = PF(S > 0 ? BrakeRes->P() : 0, LBP, abs(S) * (0.00053 + 0.00060 * int(S < 0))) * dt; // przepływ
	LBP = LBP - dv;
}

/// <summary>
/// Stores the EP intensity for the EP1 (proportional) variant.
/// The fractional part is read by <see cref="EPCalc"/> as the proportional set-point.
/// </summary>
/// <param name="nEPS">EP intensity.</param>
void TEStEP1::SetEPS(double const nEPS)
{
	EPS = nEPS;
}

//---EST3--

/// <summary>
/// One-step distributor advance for ESt3. Identical structure to TESt::GetPF
/// but with G/P-dependent fill/release curves on the brake cylinder
/// (slower fill/faster release on the goods setting at low cylinder pressures).
/// </summary>
/// <param name="PP">Brake pipe pressure [bar].</param>
/// <param name="dt">Time step [s].</param>
/// <param name="Vel">Vehicle velocity [m/s].</param>
/// <returns>Net volume exchanged with the brake pipe.</returns>
double TESt3::GetPF(double const PP, double const dt, double const Vel)
{
	const double BVP{BrakeRes->P()};
	double VVP{ValveRes->P()};
	const double BCP{BrakeCyl->P()};
	double CVP{CntrlRes->P() - 0.0};

	double dv{0.0};
	double dV1{0.0};

	// sprawdzanie stanu
	CheckState(BCP, dV1);
	CheckReleaser(dt);

	CVP = CntrlRes->P();
	VVP = ValveRes->P();
	// przeplyw ZS <-> PG
	double temp = CVs(BCP);
	dv = PF(CVP, VVP, 0.0015 * temp) * dt;
	CntrlRes->Flow(+dv);
	ValveRes->Flow(-0.04 * dv);
	dV1 = dV1 - 0.96 * dv;

	// luzowanie
	if ((BrakeStatus & b_hld) == b_off)
		dv = PF(0, BCP, 0.0042 * (1.37 - (BrakeDelayFlag == bdelay_G ? 1.0 : 0.0)) * SizeBC) * dt;
	else
		dv = 0;
	BrakeCyl->Flow(-dv);
	// przeplyw ZP <-> silowniki
	if ((BrakeStatus & b_on) == b_on)
		dv = PF(BVP, BCP, 0.017 * (1.00 + (BCP < 0.58 && BrakeDelayFlag == bdelay_G ? 1.0 : 0.0)) * (1.13 - (BCP > 0.60 && BrakeDelayFlag == bdelay_G ? 1.0 : 0.0)) * SizeBC) * dt;
	else
		dv = 0;
	BrakeRes->Flow(dv);
	BrakeCyl->Flow(-dv);
	// przeplyw ZP <-> rozdzielacz
	temp = BVs(BCP);
	if (VVP - 0.05 > BVP)
		dv = PF(BVP, VVP, 0.02 * SizeBR * temp / 1.87) * dt;
	else
		dv = 0;
	BrakeRes->Flow(dv);
	dV1 += dv * 0.96;
	ValveRes->Flow(-0.04 * dv);
	// przeplyw PG <-> rozdzielacz
	dv = PF(PP, VVP, 0.01) * dt;
	ValveRes->Flow(-dv);

	ValveRes->Act();
	BrakeCyl->Act();
	BrakeRes->Act();
	CntrlRes->Act();
	return dv - dV1;
}

//---EST4-RAPID---

/// <summary>
/// One-step distributor advance for ESt4 with the rapid (R) step. Updates the
/// hysteretic <c>RapidStatus</c> latch from speed (R bit set, &gt; 70 km/h or
/// hysteresis above 55 km/h), smooths the rapid coefficient (RapidTemp) and
/// computes the cylinder fill/vent against the impulse-chamber pressure.
/// </summary>
/// <param name="PP">Brake pipe pressure [bar].</param>
/// <param name="dt">Time step [s].</param>
/// <param name="Vel">Vehicle velocity [m/s].</param>
/// <returns>Net volume exchanged with the brake pipe.</returns>
double TESt4R::GetPF(double const PP, double const dt, double const Vel)
{
	double result;
	double dv;
	double dV1;
	double temp;
	double VVP;
	double BVP;
	double BCP;
	double CVP;

	BVP = BrakeRes->P();
	VVP = ValveRes->P();
	BCP = ImplsRes->P();
	CVP = CntrlRes->P();

	dv = 0;
	dV1 = 0;

	// sprawdzanie stanu
	CheckState(BCP, dV1);
	CheckReleaser(dt);

	CVP = CntrlRes->P();
	VVP = ValveRes->P();
	// przeplyw ZS <-> PG
	temp = CVs(BCP);
	dv = PF(CVP, VVP, 0.0015 * temp / 1.8) * dt;
	CntrlRes->Flow(+dv);
	ValveRes->Flow(-0.04 * dv);
	dV1 = dV1 - 0.96 * dv;

	// luzowanie KI
	if ((BrakeStatus & b_hld) == b_off)
		dv = PF(0, BCP, 0.00037 * 1.14 * 15 / 19) * dt;
	else
		dv = 0;
	ImplsRes->Flow(-dv);
	// przeplyw ZP <-> KI
	if ((BrakeStatus & b_on) == b_on)
		dv = PF(BVP, BCP, 0.0014) * dt;
	else
		dv = 0;
	//  BrakeRes->Flow(dV);
	ImplsRes->Flow(-dv);
	// przeplyw ZP <-> rozdzielacz
	temp = BVs(BCP);
	if (BVP < VVP - 0.05) // or((PP<CVP)and(CVP<PP-0.1)
		dv = PF(BVP, VVP, 0.02 * SizeBR * temp / 1.87) * dt;
	else
		dv = 0;
	BrakeRes->Flow(dv);
	dV1 = dV1 + dv * 0.96;
	ValveRes->Flow(-0.04 * dv);
	// przeplyw PG <-> rozdzielacz
	dv = PF(PP, VVP, 0.01 * SizeBR) * dt;
	ValveRes->Flow(-dv);

	result = dv - dV1;

	RapidStatus = BrakeDelayFlag == bdelay_R && ((Vel > 55 && RapidStatus == true) || Vel > 70);

	RapidTemp = RapidTemp + (0.9 * int(RapidStatus) - RapidTemp) * dt / 2;
	temp = 1.9 - RapidTemp;
	if ((BrakeStatus & b_asb) == b_asb)
		temp = 1000;
	// luzowanie CH
	if (BrakeCyl->P() * temp > ImplsRes->P() + 0.005 || ImplsRes->P() < 0.25)
		if ((BrakeStatus & b_asb) == b_asb)
			dv = PFVd(BrakeCyl->P(), 0, 0.115 * SizeBC * 4, ImplsRes->P() / temp) * dt;
		else
			dv = PFVd(BrakeCyl->P(), 0, 0.115 * SizeBC, ImplsRes->P() / temp) * dt;
	//   dV:=PF(0,BrakeCyl.P,0.115*sizeBC/2)*dt
	//   dV:=PFVd(BrakeCyl.P,0,0.015*sizeBC/2,ImplsRes.P/temp)*dt
	else
		dv = 0;
	BrakeCyl->Flow(-dv);
	// przeplyw ZP <-> CH
	if (BrakeCyl->P() * temp < ImplsRes->P() - 0.005 && ImplsRes->P() > 0.3)
		//   dV:=PFVa(BVP,BrakeCyl.P,0.020*sizeBC,ImplsRes.P/temp)*dt
		dv = PFVa(BVP, BrakeCyl->P(), 0.60 * SizeBC, ImplsRes->P() / temp) * dt;
	else
		dv = 0;
	BrakeRes->Flow(-dv);
	BrakeCyl->Flow(+dv);

	ImplsRes->Act();
	ValveRes->Act();
	BrakeCyl->Act();
	BrakeRes->Act();
	CntrlRes->Act();
	return result;
}

/// <summary>
/// Initialises the ESt4R: chains TESt::Init, sizes the impulse chamber to 1 l
/// and pre-charges it to BP, then selects the rapid (R) brake delay.
/// </summary>
/// <param name="PP">Brake pipe pressure [bar].</param>
/// <param name="HPP">High (control) pressure [bar].</param>
/// <param name="LPP">Low pressure threshold [bar].</param>
/// <param name="BP">Initial cylinder pressure [bar].</param>
/// <param name="BDF">Initial brake delay flag (overridden to R).</param>
void TESt4R::Init(double const PP, double const HPP, double const LPP, double const BP, int const BDF)
{
	TESt::Init(PP, HPP, LPP, BP, BDF);
	ImplsRes->CreateCap(1);
	ImplsRes->CreatePress(BP);

	BrakeDelayFlag = bdelay_R;
}

//---EST3/AL2---

/// <summary>
/// One-step distributor advance for ESt3 with AL2 load-weighing equipment.
/// Drives the impulse chamber (KI) with G/P-dependent flow rates and feeds
/// the load relay output (BrakeCyl pressure scaled by LoadC) to the cylinder.
/// </summary>
/// <param name="PP">Brake pipe pressure [bar].</param>
/// <param name="dt">Time step [s].</param>
/// <param name="Vel">Vehicle velocity [m/s].</param>
/// <returns>Net volume exchanged with the brake pipe.</returns>
double TESt3AL2::GetPF(double const PP, double const dt, double const Vel)
{
	double result;
	double dv;
	double dV1;
	double temp;
	double VVP;
	double BVP;
	double BCP;
	double CVP;

	BVP = BrakeRes->P();
	VVP = ValveRes->P();
	BCP = ImplsRes->P();
	CVP = CntrlRes->P() - 0.0;

	dv = 0;
	dV1 = 0;

	// sprawdzanie stanu
	CheckState(BCP, dV1);
	CheckReleaser(dt);

	VVP = ValveRes->P();
	// przeplyw ZS <-> PG
	temp = CVs(BCP);
	dv = PF(CVP, VVP, 0.0015 * temp) * dt;
	CntrlRes->Flow(+dv);
	ValveRes->Flow(-0.04 * dv);
	dV1 = dV1 - 0.96 * dv;

	// luzowanie KI
	if ((BrakeStatus & b_hld) == b_off)
		dv = PF(0, BCP, 0.00017 * (1.37 - int(BrakeDelayFlag == bdelay_G))) * dt;
	else
		dv = 0;
	ImplsRes->Flow(-dv);
	// przeplyw ZP <-> KI
	if ((BrakeStatus & b_on) == b_on && BCP < MaxBP)
		dv = PF(BVP, BCP, 0.0008 * (1 + int(BCP < 0.58 && BrakeDelayFlag == bdelay_G)) * (1.13 - int(BCP > 0.6 && BrakeDelayFlag == bdelay_G))) * dt;
	else
		dv = 0;
	BrakeRes->Flow(dv);
	ImplsRes->Flow(-dv);
	// przeplyw ZP <-> rozdzielacz
	temp = BVs(BCP);
	if (VVP - 0.05 > BVP)
		dv = PF(BVP, VVP, 0.02 * SizeBR * temp / 1.87) * dt;
	else
		dv = 0;
	BrakeRes->Flow(dv);
	dV1 = dV1 + dv * 0.96;
	ValveRes->Flow(-0.04 * dv);
	// przeplyw PG <-> rozdzielacz
	dv = PF(PP, VVP, 0.01) * dt;
	ValveRes->Flow(-dv);
	result = dv - dV1;

	// luzowanie CH
	if (BrakeCyl->P() > ImplsRes->P() * LoadC + 0.005 || ImplsRes->P() < 0.15)
		dv = PF(0, BrakeCyl->P(), 0.015 * SizeBC) * dt;
	else
		dv = 0;
	BrakeCyl->Flow(-dv);

	// przeplyw ZP <-> CH
	if (BrakeCyl->P() < ImplsRes->P() * LoadC - 0.005 && ImplsRes->P() > 0.15)
		dv = PF(BVP, BrakeCyl->P(), 0.020 * SizeBC) * dt;
	else
		dv = 0;
	BrakeRes->Flow(dv);
	BrakeCyl->Flow(-dv);

	ImplsRes->Act();
	ValveRes->Act();
	BrakeCyl->Act();
	BrakeRes->Act();
	CntrlRes->Act();
	return result;
}

/// <summary>Recomputes the load-weighing coefficient LoadC for the current mass.</summary>
/// <param name="mass">Current vehicle mass.</param>
void TESt3AL2::PLC(double const mass)
{
	LoadC = 1 + int(mass < LoadM) * ((TareBP + (MaxBP - TareBP) * (mass - TareM) / (LoadM - TareM)) / MaxBP - 1);
}

/// <summary>Stores the load-weighing parameters (TareM, LoadM, TareBP).</summary>
/// <param name="TM">Tare (empty) mass.</param>
/// <param name="LM">Loaded mass.</param>
/// <param name="TBP">Tare-mass cylinder pressure.</param>
void TESt3AL2::SetLP(double const TM, double const LM, double const TBP)
{
	TareM = TM;
	LoadM = LM;
	TareBP = TBP;
}

/// <summary>
/// Initialises ESt3/AL2: chains TESt::Init and sizes the impulse chamber
/// (KI) to 1 l, pre-charging it to BP.
/// </summary>
/// <param name="PP">Brake pipe pressure [bar].</param>
/// <param name="HPP">High (control) pressure [bar].</param>
/// <param name="LPP">Low pressure threshold [bar].</param>
/// <param name="BP">Initial cylinder pressure [bar].</param>
/// <param name="BDF">Initial brake delay flag.</param>
void TESt3AL2::Init(double const PP, double const HPP, double const LPP, double const BP, int const BDF)
{
	TESt::Init(PP, HPP, LPP, BP, BDF);
	ImplsRes->CreateCap(1);
	ImplsRes->CreatePress(BP);
}

//---LSt---

/// <summary>
/// One-step distributor advance for LSt: locomotive variant of ESt4R.
/// Re-implements the state machine inline (with the universal-button releaser),
/// drives the impulse chamber from CVP/VVP via PF1, applies a smoothed
/// rapid-step (RM) above a velocity threshold, factors in the ED brake
/// release (EDFlag) and the auxiliary brake LBP through a double check valve,
/// and supports the anti-slip release path (b_asb_unbrake / ASBP).
/// </summary>
/// <param name="PP">Brake pipe pressure [bar].</param>
/// <param name="dt">Time step [s].</param>
/// <param name="Vel">Vehicle velocity [m/s].</param>
/// <returns>Net volume exchanged with the brake pipe.</returns>
double TLSt::GetPF(double const PP, double const dt, double const Vel)
{
	double result;

	// ValveRes.CreatePress(LBP);
	// LBP:=0;

	double const BVP{BrakeRes->P()};
	double const VVP{ValveRes->P()};
	double const BCP{ImplsRes->P()};
	double const CVP{CntrlRes->P()};

	double dV{0.0};
	double dV1{0.0};

	// sprawdzanie stanu
	// NOTE: partial copypaste from checkstate() of base class
	// TODO: clean inheritance for checkstate() and checkreleaser() and reuse these instead of manual copypaste
	if ((BrakeStatus & b_hld) == b_hld && BCP > 0.25)
	{
		if (VVP + 0.003 + BCP / BVM < CVP)
		{
			// hamowanie stopniowe
			BrakeStatus |= b_on;
		}
		else if (VVP - 0.003 + (BCP - 0.1) / BVM > CVP)
		{
			// luzowanie
			BrakeStatus &= ~(b_on | b_hld);
		}
		else if (VVP + BCP / BVM > CVP)
		{
			// zatrzymanie napelaniania
			BrakeStatus &= ~b_on;
		}
	}
	else if (VVP + 0.10 < CVP && BCP < 0.25)
	{
		// poczatek hamowania
		if ((BrakeStatus & b_hld) == b_off)
		{
			SoundFlag |= sf_Acc;
		}
		BrakeStatus |= b_on | b_hld;
	}
	else if (VVP + (BCP - 0.1) / BVM < CVP && (CVP - VVP) * BVM > 0.25 && BCP > 0.25)
	{
		// zatrzymanie luzowanie
		BrakeStatus |= b_hld;
	}
	if ((BrakeStatus & b_hld) == 0)
	{
		SoundFlag |= sf_CylU;
	}
	// equivalent of checkreleaser() in the base class?
	const bool is_releasing = BrakeStatus & b_rls || UniversalFlag & TUniversalBrake::ub_Release;
	if (is_releasing)
	{
		if (CVP < 0.0)
		{
			BrakeStatus &= ~b_rls;
		}
		else
		{ // 008
			dV = PF1(CVP, BCP, 0.024) * dt;
			CntrlRes->Flow(dV);
		}
	}

	// przeplyw ZS <-> PG
	double temp;
	if ((CVP - BCP) * BVM > 0.5)
		temp = 0.0;
	else if (VVP > CVP + 0.4)
		temp = 0.5;
	else
		temp = 0.5;

	dV = PF1(CVP, VVP, 0.0015 * temp / 1.8 / 2) * dt;
	CntrlRes->Flow(+dV);
	ValveRes->Flow(-0.04 * dV);
	dV1 = dV1 - 0.96 * dV;

	// luzowanie KI  {G}
	//   if VVP>BCP then
	//    dV:=PF(VVP,BCP,0.00004)*dt
	//   else if (CVP-BCP)<1.5 then
	//    dV:=PF(VVP,BCP,0.00020*(1.33-int((CVP-BCP)*BVM>0.65)))*dt
	//  else dV:=0;      0.00025 P
	/*P*/
	if (VVP > BCP)
	{
		dV = PF(VVP, BCP, 0.00043 * (1.5 - (true == ((CVP - BCP) * BVM > 1.0 && BrakeDelayFlag == bdelay_G) ? 1.0 : 0.0)), 0.1) * dt;
	}
	else if (CVP - BCP < 1.5)
	{
		dV = PF(VVP, BCP, 0.001472 * (1.36 - (true == ((CVP - BCP) * BVM > 1.0 && BrakeDelayFlag == bdelay_G) ? 1.0 : 0.0)), 0.1) * dt;
	}
	else
	{
		dV = 0;
	}

	ImplsRes->Flow(-dV);
	ValveRes->Flow(+dV);
	// przeplyw PG <-> rozdzielacz
	dV = PF(PP, VVP, 0.01, 0.1) * dt;
	ValveRes->Flow(-dV);

	result = dV - dV1;

	//  if Vel>55 then temp:=0.72 else
	//    temp:=1;{R}
	// cisnienie PP
	RapidTemp = RapidTemp + (RM * int(Vel > 55 && BrakeDelayFlag == bdelay_R) - RapidTemp) * dt / 2;
	temp = 1 - RapidTemp;
	if (EDFlag > 0.2)
		temp = 10000;
	double tempasb = 0;
	if ((UniversalFlag & TUniversalBrake::ub_AntiSlipBrake) > 0 || (BrakeStatus & b_asb_unbrake) == b_asb_unbrake)
		tempasb = ASBP;
	// powtarzacz — podwojny zawor zwrotny
	temp = std::max(((CVP - BCP) * BVM + tempasb) / temp, LBP);
	// luzowanie CH
	if (BrakeCyl->P() > temp + 0.005 || temp < 0.28)
		//   dV:=PF(0,BrakeCyl->P(),0.0015*3*sizeBC)*dt
		//   dV:=PF(0,BrakeCyl->P(),0.005*3*sizeBC)*dt
		dV = PFVd(BrakeCyl->P(), 0, 0.005 * 7 * SizeBC, temp) * dt;
	else
		dV = 0;
	BrakeCyl->Flow(-dV);
	// przeplyw ZP <-> CH
	if (BrakeCyl->P() < temp - 0.005 && temp > 0.29)
		//   dV:=PF(BVP,BrakeCyl->P(),0.002*3*sizeBC*2)*dt
		dV = -PFVa(BVP, BrakeCyl->P(), 0.002 * 7 * SizeBC * 2, temp) * dt;
	else
		dV = 0;
	BrakeRes->Flow(dV);
	BrakeCyl->Flow(-dV);

	ImplsRes->Act();
	ValveRes->Act();
	BrakeCyl->Act();
	BrakeRes->Act();
	CntrlRes->Act();
	//  LBP:=ValveRes->P();
	//  ValveRes.CreatePress(ImplsRes->P());
	return result;
}

/// <summary>
/// Initialises the LSt: chains TESt4R::Init, resizes the valve pre-chamber
/// (1 l) and impulse chamber (8 l) for locomotive use, pre-charges
/// pressures, clears the ED-release flag.
/// </summary>
/// <param name="PP">Brake pipe pressure [bar].</param>
/// <param name="HPP">High (control) pressure [bar].</param>
/// <param name="LPP">Low pressure threshold [bar].</param>
/// <param name="BP">Initial cylinder pressure [bar].</param>
/// <param name="BDF">Initial brake delay flag.</param>
void TLSt::Init(double const PP, double const HPP, double const LPP, double const BP, int const BDF)
{
	TESt4R::Init(PP, HPP, LPP, BP, BDF);
	ValveRes->CreateCap(1);
	ImplsRes->CreateCap(8);
	ImplsRes->CreatePress(PP);
	BrakeRes->CreatePress(8);
	ValveRes->CreatePress(PP);

	EDFlag = 0;

	BrakeDelayFlag = BDF;
}

/// <summary>Sets the auxiliary (local) brake target pressure feeding the DCV.</summary>
/// <param name="P">Auxiliary brake pressure [bar].</param>
void TLSt::SetLBP(double const P)
{
	LBP = P;
}

/// <summary>
/// Returns the brake-cylinder reference pressure used by the ED brake
/// controller: (CVP - BCP) * BVM, where BCP is the impulse-chamber pressure.
/// </summary>
double TLSt::GetEDBCP()
{
	double CVP;
	double BCP;

	CVP = CntrlRes->P();
	BCP = ImplsRes->P();
	return (CVP - BCP) * BVM;
}

/// <summary>Sets the ED brake state (intensity, 0..1) used to relax the pneumatic brake.</summary>
/// <param name="EDstate">ED intensity.</param>
void TLSt::SetED(double const EDstate)
{
	EDFlag = EDstate;
}

/// <summary>Sets the rapid-step ratio. RM = 1 - RMR (so RMR=0 disables, RMR&gt;0 enables).</summary>
/// <param name="RMR">Reduction ratio.</param>
void TLSt::SetRM(double const RMR)
{
	RM = 1 - RMR;
}

/// <summary>
/// High-pressure replenishment: takes air only when the source pressure is
/// higher than the auxiliary reservoir, returning a non-positive flow that
/// is added back to BrakeRes.
/// </summary>
/// <param name="HP">High-pressure source [bar].</param>
/// <param name="dt">Time step [s] (folded into the orifice term in this implementation).</param>
/// <returns>Inflow into BrakeRes (negative).</returns>
double TLSt::GetHPFlow(double const HP, double const dt)
{
	double dv;

	dv = std::min(PF(HP, BrakeRes->P(), 0.01 * dt), 0.0);
	BrakeRes->Flow(-dv);
	return dv;
}

//---EStED---

/// <summary>
/// One-step distributor advance for EStED (the EP09 "ESt + ED" valve).
/// Adds the intermediate reservoir (Miedzypoj), the closing-valve memory
/// (Zamykajacy), the accelerator-block latch (Przys_blok), nozzle-tuned
/// flows for ZP/ZS filling and the rapid-step / ED-release / anti-slip
/// path on top of the ESt pneumatics.
/// </summary>
/// <param name="PP">Brake pipe pressure [bar].</param>
/// <param name="dt">Time step [s].</param>
/// <param name="Vel">Vehicle velocity [m/s].</param>
/// <returns>Net volume exchanged with the brake pipe.</returns>
double TEStED::GetPF(double const PP, double const dt, double const Vel)
{
	double dv;
	double dV1;
	double temp;
	double VVP;
	double BVP;
	double BCP;
	double CVP;
	double MPP;
	double nastG;

	BVP = BrakeRes->P();
	VVP = ValveRes->P();
	BCP = ImplsRes->P();
	CVP = CntrlRes->P() - 0.0;
	MPP = Miedzypoj->P();
	dV1 = 0;

	nastG = BrakeDelayFlag & bdelay_G;

	// sprawdzanie stanu
	if (BCP < 0.25 && VVP + 0.08 > CVP)
		Przys_blok = false;

	// sprawdzanie stanu
	if (VVP + 0.002 + BCP / BVM < CVP - 0.05 && Przys_blok)
		BrakeStatus |= b_on | b_hld; // hamowanie stopniowe;
	else if (VVP - 0.002 + (BCP - 0.1) / BVM > CVP - 0.05)
		BrakeStatus &= ~(b_on | b_hld); // luzowanie;
	else if (VVP + BCP / BVM > CVP - 0.05)
		BrakeStatus &= ~b_on; // zatrzymanie napelaniania;
	else if (VVP + (BCP - 0.1) / BVM < CVP - 0.05 && BCP > 0.25) // zatrzymanie luzowania
		BrakeStatus |= b_hld;

	if (VVP + 0.10 < CVP && BCP < 0.25) // poczatek hamowania
		if (!Przys_blok)
		{
			ValveRes->CreatePress(0.75 * VVP);
			SoundFlag |= sf_Acc;
			ValveRes->Act();
			Przys_blok = true;
		}

	if (BCP > 0.5)
		Zamykajacy = true;
	else if (VVP - 0.6 < MPP)
		Zamykajacy = false;

	if ((BrakeStatus & b_rls) == b_rls)
	{
		dv = PF(CVP, BCP, 0.024) * dt;
		CntrlRes->Flow(+dv);
	}

	// luzowanie
	if ((BrakeStatus & b_hld) == b_off)
		dv = PF(0, BCP, Nozzles[3] * nastG + (1 - nastG) * Nozzles[1]) * dt;
	else
		dv = 0;
	ImplsRes->Flow(-dv);
	if ((BrakeStatus & b_on) == b_on && BCP < MaxBP)
		dv = PF(BVP, BCP, Nozzles[2] * (nastG + 2 * int(BCP < 0.8)) + Nozzles[0] * (1 - nastG)) * dt;
	else
		dv = 0;
	ImplsRes->Flow(-dv);
	BrakeRes->Flow(dv);

	// przeplyw testowy miedzypojemnosci
	if (MPP < CVP - 0.3)
		temp = Nozzles[4];
	else if (BCP < 0.5)
		if (Zamykajacy)
			temp = Nozzles[8]; // 1.25;
		else
			temp = Nozzles[7];
	else
		temp = 0;
	dv = PF(MPP, VVP, temp);

	if (MPP < CVP - 0.17)
		temp = 0;
	else if (MPP > CVP - 0.08)
		temp = Nozzles[5];
	else
		temp = Nozzles[6];
	dv = dv + PF(MPP, CVP, temp);

	if (MPP - 0.05 > BVP)
		dv = dv + PF(MPP - 0.05, BVP, Nozzles[10] * nastG + (1 - nastG) * Nozzles[9]);
	if (MPP > VVP)
		dv = dv + PF(MPP, VVP, 0.02);
	Miedzypoj->Flow(dv * dt * 0.15);

	RapidTemp = RapidTemp + (RM * int(Vel > RV && BrakeDelayFlag == bdelay_R) - RapidTemp) * dt / 2;
	temp = std::max(1 - RapidTemp, 0.001);
	//  if EDFlag then temp:=1000;
	//  temp:=temp/(1-);

	// powtarzacz — podwojny zawor zwrotny
	temp = std::max(LoadC * BCP / temp * std::clamp(1 - EDFlag, 0., 1.), LBP);

	if ((UniversalFlag & TUniversalBrake::ub_AntiSlipBrake) > 0)
		temp = std::max(temp, ASBP);

	double speed = 1;
	if (ASBP < 0.1 && (BrakeStatus & b_asb_unbrake) == b_asb_unbrake)
	{
		temp = 0;
		speed = 3;
	}

	if (BrakeCyl->P() > temp)
		dv = -PFVd(BrakeCyl->P(), 0, 0.05 * SizeBC * speed, temp) * dt;
	else if (BrakeCyl->P() < temp && (BrakeStatus & b_asb) == 0)
		dv = PFVa(BVP, BrakeCyl->P(), 0.05 * SizeBC, temp) * dt;
	else
		dv = 0;

	BrakeCyl->Flow(dv);
	if (dv > 0)
		BrakeRes->Flow(-dv);

	// przeplyw ZS <-> PG
	if (MPP < CVP - 0.17)
		temp = 0;
	else if (MPP > CVP - 0.08)
		temp = Nozzles[5];
	else
		temp = Nozzles[6];
	dv = PF(CVP, MPP, temp) * dt;
	CntrlRes->Flow(+dv);
	ValveRes->Flow(-0.02 * dv);
	dV1 = dV1 + 0.98 * dv;

	// przeplyw ZP <-> MPJ
	if (MPP - 0.05 > BVP)
		dv = PF(BVP, MPP - 0.05, Nozzles[10] * nastG + (1 - nastG) * Nozzles[9]) * dt;
	else
		dv = 0;
	BrakeRes->Flow(dv);
	dV1 = dV1 + dv * 0.98;
	ValveRes->Flow(-0.02 * dv);
	// przeplyw PG <-> rozdzielacz
	dv = PF(PP, VVP, 0.005) * dt; // 0.01
	ValveRes->Flow(-dv);

	ValveRes->Act();
	BrakeCyl->Act();
	BrakeRes->Act();
	CntrlRes->Act();
	Miedzypoj->Act();
	ImplsRes->Act();
	return dv - dV1;
}

/// <summary>
/// Initialises the EStED: chains TLSt::Init, sizes Miedzypoj (5 l), pre-charges
/// pressures, computes BVM with a 0.05 bar offset on HPP, configures the
/// 11 internal nozzle cross-sections (squared and scaled to the engine's
/// flow units) and clears the latches.
/// </summary>
/// <param name="PP">Brake pipe pressure [bar].</param>
/// <param name="HPP">High (control) pressure [bar].</param>
/// <param name="LPP">Low pressure threshold [bar].</param>
/// <param name="BP">Initial cylinder pressure [bar].</param>
/// <param name="BDF">Initial brake delay flag.</param>
void TEStED::Init(double const PP, double const HPP, double const LPP, double const BP, int const BDF)
{
	TLSt::Init(PP, HPP, LPP, BP, BDF);
	int i;

	ValveRes->CreatePress(PP);
	BrakeCyl->CreatePress(BP);

	//  CntrlRes:=TReservoir.Create;
	//  CntrlRes.CreateCap(15);
	//  CntrlRes.CreatePress(1*HPP);

	BrakeStatus = BP > 1.0 ? 1 : 0;
	Miedzypoj->CreateCap(5);
	Miedzypoj->CreatePress(PP);

	ImplsRes->CreateCap(1);
	ImplsRes->CreatePress(BP);

	BVM = 1.0 / (HPP - 0.05 - LPP) * MaxBP;

	BrakeDelayFlag = BDF;
	Zamykajacy = false;
	EDFlag = 0;

	Nozzles[0] = 1.250 / 1.7;
	Nozzles[1] = 0.907;
	Nozzles[2] = 0.510 / 1.7;
	Nozzles[3] = 0.524 / 1.17;
	Nozzles[4] = 7.4;
	Nozzles[7] = 5.3;
	Nozzles[8] = 2.5;
	Nozzles[9] = 7.28;
	Nozzles[10] = 2.96;
	Nozzles[5] = 1.1;
	Nozzles[6] = 0.9;

	{
		for (i = 0; i < 11; ++i)
		{
			Nozzles[i] = Nozzles[i] * Nozzles[i] * 3.14159 / 4000;
		}
	}
}

/// <summary>
/// Returns the ED reference pressure for EStED — impulse chamber pressure
/// scaled by the load-weighing coefficient.
/// </summary>
double TEStED::GetEDBCP()
{
	return ImplsRes->P() * LoadC;
}

/// <summary>Recomputes the load-weighing coefficient LoadC for the current mass.</summary>
/// <param name="mass">Current vehicle mass.</param>
void TEStED::PLC(double const mass)
{
	LoadC = 1 + int(mass < LoadM) * ((TareBP + (MaxBP - TareBP) * (mass - TareM) / (LoadM - TareM)) / MaxBP - 1);
}

/// <summary>Stores the load-weighing parameters.</summary>
/// <param name="TM">Tare (empty) mass.</param>
/// <param name="LM">Loaded mass.</param>
/// <param name="TBP">Tare-mass cylinder pressure.</param>
void TEStED::SetLP(double const TM, double const LM, double const TBP)
{
	TareM = TM;
	LoadM = LM;
	TareBP = TBP;
}

//---DAKO CV1---

/// <summary>
/// Drives the BrakeStatus state machine for CV1: handles the releaser
/// auto-disengage (when CVP &lt;= VVP) and the b_on/b_hld transitions
/// from the relations between pre-chamber, cylinder and control reservoir.
/// On the start of braking it returns dV1=1.25 to feed the brake-pipe
/// flow correction.
/// </summary>
/// <param name="BCP">Cylinder pressure.</param>
/// <param name="dV1">In/out brake-pipe flow correction.</param>
void TCV1::CheckState(double const BCP, double &dV1)
{
	double VVP;
	double BVP;
	double CVP;

	BVP = BrakeRes->P();
	VVP = std::min(ValveRes->P(), BVP + 0.05);
	CVP = CntrlRes->P();

	// odluzniacz
	if ((BrakeStatus & b_rls) == b_rls && CVP - VVP < 0)
		BrakeStatus &= ~b_rls;

	// sprawdzanie stanu
	if ((BrakeStatus & b_hld) == b_hld)
		if (VVP + 0.003 + BCP / BVM < CVP)
			BrakeStatus |= b_on; // hamowanie stopniowe;
		else if (VVP - 0.003 + BCP * 1.0 / BVM > CVP)
			BrakeStatus &= ~(b_on | b_hld); // luzowanie;
		else if (VVP + BCP * 1.0 / BVM > CVP)
			BrakeStatus &= ~b_on; // zatrzymanie napelaniania;
		else
			;
	else if (VVP + 0.10 < CVP && BCP < 0.1) // poczatek hamowania
	{
		BrakeStatus |= b_on | b_hld;
		dV1 = 1.25;
	}
	else if (VVP + BCP / BVM < CVP && BCP > 0.25) // zatrzymanie luzowanie
		BrakeStatus |= b_hld;
}

/// <summary>
/// Returns the CV1 ZS-filling slide valve opening factor: closed once any
/// pneumatic braking has been requested (BP &gt; 0.05), otherwise a constant 0.23.
/// </summary>
/// <param name="BP">Cylinder pressure.</param>
/// <returns>Opening coefficient.</returns>
double TCV1::CVs(double const BP)
{
	// przeplyw ZS <-> PG
	if (BP > 0.05)
		return 0;
	else
		return 0.23;
}

/// <summary>
/// Returns the CV1 ZP-filling slide valve opening factor: 1 while the
/// auxiliary reservoir is below the control reservoir (refill), 0 once the
/// brake has been applied, and a small charging value otherwise.
/// </summary>
/// <param name="BCP">Cylinder pressure.</param>
/// <returns>Opening coefficient.</returns>
double TCV1::BVs(double const BCP)
{
	double VVP;
	double BVP;
	double CVP;

	BVP = BrakeRes->P();
	CVP = CntrlRes->P();
	VVP = ValveRes->P();

	// przeplyw ZP <-> rozdzielacz
	if (BVP < CVP - 0.1)
		return 1;
	else if (BCP > 0.05)
		return 0;
	else
		return 0.2 * (1.5 - int(BVP > VVP));
}

/// <summary>
/// One-step distributor advance for the DAKO CV1: runs CheckState, integrates
/// ZS &lt;-&gt; PG, ZP &lt;-&gt; cylinder (with G/P-dependent fill rates), ZP &lt;-&gt;
/// pre-chamber and PG &lt;-&gt; pre-chamber flows.
/// </summary>
/// <param name="PP">Brake pipe pressure [bar].</param>
/// <param name="dt">Time step [s].</param>
/// <param name="Vel">Vehicle velocity [m/s].</param>
/// <returns>Net volume exchanged with the brake pipe.</returns>
double TCV1::GetPF(double const PP, double const dt, double const Vel)
{
	double dv;
	double dV1;
	double temp;
	double VVP;
	double BVP;
	double BCP;
	double CVP;

	BVP = BrakeRes->P();
	VVP = std::min(ValveRes->P(), BVP + 0.05);
	BCP = BrakeCyl->P();
	CVP = CntrlRes->P();

	dv = 0;
	dV1 = 0;

	// sprawdzanie stanu
	CheckState(BCP, dV1);

	VVP = ValveRes->P();
	// przeplyw ZS <-> PG
	temp = CVs(BCP);
	dv = PF(CVP, VVP, 0.0015 * temp) * dt;
	CntrlRes->Flow(+dv);
	ValveRes->Flow(-0.04 * dv);
	dV1 = dV1 - 0.96 * dv;

	// luzowanie
	if ((BrakeStatus & b_hld) == b_off)
		dv = PF(0, BCP, 0.0042 * (1.37 - int(BrakeDelayFlag == bdelay_G)) * SizeBC) * dt;
	else
		dv = 0;
	BrakeCyl->Flow(-dv);

	// przeplyw ZP <-> silowniki
	if ((BrakeStatus & b_on) == b_on)
		dv = PF(BVP, BCP, 0.017 * (1 + int(BCP < 0.58 && BrakeDelayFlag == bdelay_G)) * (1.13 - int(BCP > 0.6 && BrakeDelayFlag == bdelay_G)) * SizeBC) * dt;
	else
		dv = 0;
	BrakeRes->Flow(dv);
	BrakeCyl->Flow(-dv);

	// przeplyw ZP <-> rozdzielacz
	temp = BVs(BCP);
	if (VVP + 0.05 > BVP)
		dv = PF(BVP, VVP, 0.02 * SizeBR * temp / 1.87) * dt;
	else
		dv = 0;
	BrakeRes->Flow(dv);
	dV1 = dV1 + dv * 0.96;
	ValveRes->Flow(-0.04 * dv);
	// przeplyw PG <-> rozdzielacz
	dv = PF(PP, VVP, 0.01) * dt;
	ValveRes->Flow(-dv);

	ValveRes->Act();
	BrakeCyl->Act();
	BrakeRes->Act();
	CntrlRes->Act();
	return dv - dV1;
}

/// <summary>
/// Initialises the CV1: pre-charges valve / brake / control reservoirs (ZS = 15 l),
/// clears BrakeStatus and computes BVM (= MaxBP / (HPP-LPP)).
/// </summary>
/// <param name="PP">Brake pipe pressure [bar].</param>
/// <param name="HPP">High (control) pressure [bar].</param>
/// <param name="LPP">Low pressure threshold [bar].</param>
/// <param name="BP">Initial cylinder pressure [bar].</param>
/// <param name="BDF">Initial brake delay flag.</param>
void TCV1::Init(double const PP, double const HPP, double const LPP, double const BP, int const BDF)
{
	TBrake::Init(PP, HPP, LPP, BP, BDF);
	ValveRes->CreatePress(PP);
	BrakeCyl->CreatePress(BP);
	BrakeRes->CreatePress(PP);
	CntrlRes->CreateCap(15);
	CntrlRes->CreatePress(HPP);
	BrakeStatus = b_off;

	BVM = 1.0 / (HPP - LPP) * MaxBP;

	BrakeDelayFlag = BDF;
}

/// <summary>Returns the control reservoir (ZS) pressure.</summary>
double TCV1::GetCRP()
{
	return CntrlRes->P();
}

/// <summary>Vents valve, brake and control reservoirs to zero.</summary>
void TCV1::ForceEmptiness()
{

	ValveRes->CreatePress(0);
	BrakeRes->CreatePress(0);
	CntrlRes->CreatePress(0);

	ValveRes->Act();
	BrakeRes->Act();
	CntrlRes->Act();
}

//---CV1-L-TR---

/// <summary>Sets the auxiliary (local) brake target pressure feeding the DCV.</summary>
/// <param name="P">Auxiliary brake pressure [bar].</param>
void TCV1L_TR::SetLBP(double const P)
{
	LBP = P;
}

/// <summary>
/// High-pressure replenishment for CV1-L-TR: pulls air into the auxiliary
/// reservoir whenever the source is higher (returns a non-positive flow).
/// </summary>
/// <param name="HP">High-pressure source [bar].</param>
/// <param name="dt">Time step [s].</param>
/// <returns>Inflow into BrakeRes (negative on intake).</returns>
double TCV1L_TR::GetHPFlow(double const HP, double const dt)
{
	double dv;

	dv = PF(HP, BrakeRes->P(), 0.01) * dt;
	dv = std::min(0., dv);
	BrakeRes->Flow(-dv);
	return dv;
}

/// <summary>
/// Initialises CV1-L-TR: chains TCV1::Init and sizes the impulse chamber to
/// 2.5 l, pre-charging it to BP.
/// </summary>
/// <param name="PP">Brake pipe pressure [bar].</param>
/// <param name="HPP">High (control) pressure [bar].</param>
/// <param name="LPP">Low pressure threshold [bar].</param>
/// <param name="BP">Initial cylinder pressure [bar].</param>
/// <param name="BDF">Initial brake delay flag.</param>
void TCV1L_TR::Init(double const PP, double const HPP, double const LPP, double const BP, int const BDF)
{
	TCV1::Init(PP, HPP, LPP, BP, BDF);
	ImplsRes->CreateCap(2.5);
	ImplsRes->CreatePress(BP);
}

/// <summary>
/// One-step distributor advance for CV1-L-TR. Drives the impulse chamber (KI)
/// from the CV1 state machine, then computes cylinder fill/release against
/// the higher of the impulse-chamber and auxiliary-brake (LBP) pressure.
/// </summary>
/// <param name="PP">Brake pipe pressure [bar].</param>
/// <param name="dt">Time step [s].</param>
/// <param name="Vel">Vehicle velocity [m/s].</param>
/// <returns>Net volume exchanged with the brake pipe.</returns>
double TCV1L_TR::GetPF(double const PP, double const dt, double const Vel)
{
	double result;
	double dv;
	double dV1;
	double temp;
	double VVP;
	double BVP;
	double BCP;
	double CVP;

	BVP = BrakeRes->P();
	VVP = std::min(ValveRes->P(), BVP + 0.05);
	BCP = ImplsRes->P();
	CVP = CntrlRes->P();

	dv = 0;
	dV1 = 0;

	// sprawdzanie stanu
	CheckState(BCP, dV1);

	VVP = ValveRes->P();
	// przeplyw ZS <-> PG
	temp = CVs(BCP);
	dv = PF(CVP, VVP, 0.0015 * temp) * dt;
	CntrlRes->Flow(+dv);
	ValveRes->Flow(-0.04 * dv);
	dV1 = dV1 - 0.96 * dv;

	// luzowanie KI
	if ((BrakeStatus & b_hld) == b_off)
		dv = PF(0, BCP, 0.000425 * (1.37 - int(BrakeDelayFlag == bdelay_G))) * dt;
	else
		dv = 0;
	ImplsRes->Flow(-dv);
	// przeplyw ZP <-> KI
	if ((BrakeStatus & b_on) == b_on && BCP < MaxBP)
		dv = PF(BVP, BCP, 0.002 * (1 + int(BCP < 0.58 && BrakeDelayFlag == bdelay_G)) * (1.13 - int(BCP > 0.6 && BrakeDelayFlag == bdelay_G))) * dt;
	else
		dv = 0;
	BrakeRes->Flow(dv);
	ImplsRes->Flow(-dv);

	// przeplyw ZP <-> rozdzielacz
	temp = BVs(BCP);
	if (VVP + 0.05 > BVP)
		dv = PF(BVP, VVP, 0.02 * SizeBR * temp / 1.87) * dt;
	else
		dv = 0;
	BrakeRes->Flow(dv);
	dV1 = dV1 + dv * 0.96;
	ValveRes->Flow(-0.04 * dv);
	// przeplyw PG <-> rozdzielacz
	dv = PF(PP, VVP, 0.01) * dt;
	result = dv - dV1;

	temp = std::max(BCP, LBP);

	// luzowanie CH
	if (BrakeCyl->P() > temp + 0.005 || std::max(ImplsRes->P(), 8 * LBP) < 0.25)
		dv = PF(0, BrakeCyl->P(), 0.015 * SizeBC) * dt;
	else
		dv = 0;
	BrakeCyl->Flow(-dv);

	// przeplyw ZP <-> CH
	if (BrakeCyl->P() < temp - 0.005 && std::max(ImplsRes->P(), 8 * LBP) > 0.3 && std::max(BCP, LBP) < MaxBP)
		dv = PF(BVP, BrakeCyl->P(), 0.020 * SizeBC) * dt;
	else
		dv = 0;
	BrakeRes->Flow(dv);
	BrakeCyl->Flow(-dv);

	ImplsRes->Act();
	ValveRes->Act();
	BrakeCyl->Act();
	BrakeRes->Act();
	CntrlRes->Act();
	return result;
}

//--- KNORR KE ---
/// <summary>
/// Implements the KE releaser logic: while engaged, vents the control
/// reservoir to zero via a 0.1-area orifice; auto-disengages once CVP &lt;= VVP.
/// </summary>
/// <param name="dt">Time step [s].</param>
void TKE::CheckReleaser(double const dt)
{
	double VVP;
	double CVP;

	VVP = ValveRes->P();
	CVP = CntrlRes->P();

	// odluzniacz
	if (true == ((BrakeStatus & b_rls) == b_rls))
		if (CVP - VVP < 0)
			BrakeStatus &= ~b_rls;
		else
			CntrlRes->Flow(+PF(CVP, 0, 0.1) * dt);
}

/// <summary>
/// Drives the KE BrakeStatus state machine from the relations between
/// pre-chamber, impulse-chamber and control reservoir pressures (KE-specific
/// thresholds — pulse on the pre-chamber down to 0.8*VVP and 0.1 bar
/// initial-step). Triggers sf_Acc and sf_CylU sound flags.
/// </summary>
/// <param name="BCP">Cylinder (or impulse-chamber) pressure.</param>
/// <param name="dV1">In/out brake-pipe flow correction (unused here).</param>
void TKE::CheckState(double const BCP, double &dV1)
{
	double VVP;
	double BVP;
	double CVP;

	BVP = BrakeRes->P();
	VVP = ValveRes->P();
	CVP = CntrlRes->P();

	// sprawdzanie stanu
	if (BCP > 0.1)
	{

		if ((BrakeStatus & b_hld) == b_hld)
		{

			if (VVP + 0.003 + BCP / BVM < CVP)
			{
				// hamowanie stopniowe;
				BrakeStatus |= b_on;
			}
			else
			{
				if (VVP + BCP / BVM > CVP)
				{
					// zatrzymanie napelaniania;
					BrakeStatus &= ~b_on;
				}
				if (VVP - 0.003 + BCP / BVM > CVP)
				{
					// luzowanie;
					BrakeStatus &= ~(b_on | b_hld);
				}
			}
		}
		else
		{

			if (VVP + BCP / BVM < CVP && (CVP - VVP) * BVM > 0.25)
			{
				// zatrzymanie luzowanie
				BrakeStatus |= b_hld;
			}
		}
	}
	else
	{

		if (VVP + 0.1 < CVP)
		{
			// poczatek hamowania
			if ((BrakeStatus & b_hld) == 0)
			{
				// przyspieszacz
				ValveRes->CreatePress(0.8 * VVP);
				SoundFlag |= sf_Acc;
				ValveRes->Act();
			}
			BrakeStatus |= b_on | b_hld;
		}
	}

	if ((BrakeStatus & b_hld) == 0)
	{
		SoundFlag |= sf_CylU;
	}
}

/// <summary>
/// Returns the KE ZS-filling slide valve opening factor: closed once
/// any pneumatic braking is requested, slightly attenuated when the
/// pre-chamber is over-pressurised, and a constant 0.23 otherwise.
/// </summary>
/// <param name="BP">Cylinder (or impulse) pressure.</param>
/// <returns>Opening coefficient.</returns>
double TKE::CVs(double const BP)
{
	double VVP;
	double BVP;
	double CVP;

	BVP = BrakeRes->P();
	CVP = CntrlRes->P();
	VVP = ValveRes->P();

	// przeplyw ZS <-> PG
	if (BP > 0.2)
		return 0;
	else if (VVP > CVP + 0.4)
		return 0.05;
	else
		return 0.23;
}

/// <summary>
/// Returns the KE ZP-filling slide valve opening factor: closed when ZP
/// is already above the pre-chamber, fully open when ZP is well below the
/// control reservoir, otherwise a low refill rate (0.13).
/// </summary>
/// <param name="BCP">Impulse-chamber pressure.</param>
/// <returns>Opening coefficient.</returns>
double TKE::BVs(double const BCP)
{
	double VVP;
	double BVP;
	double CVP;

	BVP = BrakeRes->P();
	CVP = CntrlRes->P();
	VVP = ValveRes->P();

	// przeplyw ZP <-> rozdzielacz
	if (BVP > VVP)
		return 0;
	else if (BVP < CVP - 0.3)
		return 0.6;
	else
		return 0.13;
}

/// <summary>
/// One-step distributor advance for the Knorr KE. Drives CheckState/CheckReleaser,
/// integrates ZS &lt;-&gt; PG, ZP &lt;-&gt; impulse chamber and ZP &lt;-&gt; pre-chamber,
/// applies the velocity- and friction-pair-aware rapid step (RM, RV) and
/// the load-relay output to the brake cylinder, including the auxiliary
/// brake (LBP) and anti-slip path (ASBP).
/// </summary>
/// <param name="PP">Brake pipe pressure [bar].</param>
/// <param name="dt">Time step [s].</param>
/// <param name="Vel">Vehicle velocity [m/s].</param>
/// <returns>Net volume exchanged with the brake pipe.</returns>
double TKE::GetPF(double const PP, double const dt, double const Vel)
{
	double dv;
	double dV1;
	double temp;
	double VVP;
	double BVP;
	double BCP;
	double IMP;
	double CVP;

	BVP = BrakeRes->P();
	VVP = ValveRes->P();
	BCP = BrakeCyl->P();
	IMP = ImplsRes->P();
	CVP = CntrlRes->P();

	dv = 0;
	dV1 = 0;

	// sprawdzanie stanu
	CheckState(IMP, dV1);
	CheckReleaser(dt);

	// przeplyw ZS <-> PG
	temp = CVs(IMP);
	dv = PF(CVP, VVP, 0.0015 * temp) * dt;
	CntrlRes->Flow(+dv);
	ValveRes->Flow(-0.04 * dv);
	dV1 = dV1 - 0.96 * dv;

	// luzowanie
	if ((BrakeStatus & b_hld) == b_off)
	{
		if ((BrakeDelayFlag & bdelay_G) == 0)
			temp = 0.283 + 0.139;
		else
			temp = 0.139;
		dv = PF(0, IMP, 0.001 * temp) * dt;
	}
	else
		dv = 0;
	ImplsRes->Flow(-dv);

	// przeplyw ZP <-> silowniki
	if ((BrakeStatus & b_on) == b_on && IMP < MaxBP)
	{
		temp = 0.113;
		if ((BrakeDelayFlag & bdelay_G) == 0)
			temp = temp + 0.636;
		if (BCP < 0.5)
			temp = temp + 0.785;
		dv = PF(BVP, IMP, 0.001 * temp) * dt;
	}
	else
		dv = 0;
	BrakeRes->Flow(dv);
	ImplsRes->Flow(-dv);

	// rapid
	if (!(typeid(*FM) == typeid(TDisk1) || typeid(*FM) == typeid(TDisk2))) // jesli zeliwo to schodz
		RapidStatus = (BrakeDelayFlag & bdelay_R) == bdelay_R && (RV < 0 || (Vel > RV && RapidStatus) || Vel > RV + 20);
	else // jesli tarczowki, to zostan
		RapidStatus = (BrakeDelayFlag & bdelay_R) == bdelay_R;

	//  temp:=1.9-0.9*int(RapidStatus);

	if (RM * RM > 0.001) // jesli jest rapid
		if (RM > 0) // jesli dodatni (naddatek);
			temp = 1 - RM * int(RapidStatus);
		else
			temp = 1 - RM * (1 - int(RapidStatus));
	else
		temp = 1;
	temp = temp / LoadC;
	// luzowanie CH
	//  temp:=std::max(BCP,LBP);
	IMP = std::max(IMP / temp, std::max(LBP, ASBP * int((BrakeStatus & b_asb) == b_asb)));
	if (ASBP < 0.1 && (BrakeStatus & b_asb) == b_asb)
		IMP = 0;
	// luzowanie CH
	if (BCP > IMP + 0.005 || std::max(ImplsRes->P(), 8 * LBP) < 0.25)
		dv = PFVd(BCP, 0, 0.05, IMP) * dt;
	else
		dv = 0;
	BrakeCyl->Flow(-dv);
	if (BCP < IMP - 0.005 && std::max(ImplsRes->P(), 8 * LBP) > 0.3)
		dv = PFVa(BVP, BCP, 0.05, IMP) * dt;
	else
		dv = 0;
	BrakeRes->Flow(-dv);
	BrakeCyl->Flow(+dv);

	// przeplyw ZP <-> rozdzielacz
	temp = BVs(IMP);
	//  if(BrakeStatus and b_hld)=b_off then
	if (IMP < 0.25 || VVP + 0.05 > BVP)
		dv = PF(BVP, VVP, 0.02 * SizeBR * temp / 1.87) * dt;
	else
		dv = 0;
	BrakeRes->Flow(dv);
	dV1 = dV1 + dv * 0.96;
	ValveRes->Flow(-0.04 * dv);
	// przeplyw PG <-> rozdzielacz
	dv = PF(PP, VVP, 0.01) * dt;
	ValveRes->Flow(-dv);

	ValveRes->Act();
	BrakeCyl->Act();
	BrakeRes->Act();
	CntrlRes->Act();
	ImplsRes->Act();
	return dv - dV1;
}

/// <summary>
/// Initialises the KE distributor: pre-charges valve / brake / control /
/// impulse reservoirs (ZS = 5 l, KI = 1 l), clears BrakeStatus and computes
/// BVM (= MaxBP / (HPP-LPP)).
/// </summary>
/// <param name="PP">Brake pipe pressure [bar].</param>
/// <param name="HPP">High (control) pressure [bar].</param>
/// <param name="LPP">Low pressure threshold [bar].</param>
/// <param name="BP">Initial cylinder pressure [bar].</param>
/// <param name="BDF">Initial brake delay flag.</param>
void TKE::Init(double const PP, double const HPP, double const LPP, double const BP, int const BDF)
{
	TBrake::Init(PP, HPP, LPP, BP, BDF);
	ValveRes->CreatePress(PP);
	BrakeCyl->CreatePress(BP);
	BrakeRes->CreatePress(PP);

	CntrlRes->CreateCap(5);
	CntrlRes->CreatePress(HPP);

	ImplsRes->CreateCap(1);
	ImplsRes->CreatePress(BP);

	BrakeStatus = b_off;

	BVM = 1.0 / (HPP - LPP) * MaxBP;

	BrakeDelayFlag = BDF;
}

/// <summary>Returns the control reservoir (ZS) pressure.</summary>
double TKE::GetCRP()
{
	return CntrlRes->P();
}

/// <summary>
/// High-pressure replenishment for KE: pulls air into the auxiliary
/// reservoir from the main reservoir whenever the source is higher.
/// </summary>
/// <param name="HP">High-pressure source [bar].</param>
/// <param name="dt">Time step [s].</param>
/// <returns>Inflow into BrakeRes (negative on intake).</returns>
double TKE::GetHPFlow(double const HP, double const dt)
{
	double dv;

	dv = PF(HP, BrakeRes->P(), 0.01) * dt;
	dv = std::min(0., dv);
	BrakeRes->Flow(-dv);
	return dv;
}

/// <summary>Recomputes the load-weighing coefficient LoadC for the current mass.</summary>
/// <param name="mass">Current vehicle mass.</param>
void TKE::PLC(double const mass)
{
	LoadC = 1 + int(mass < LoadM) * ((TareBP + (MaxBP - TareBP) * (mass - TareM) / (LoadM - TareM)) / MaxBP - 1);
}

/// <summary>Stores the load-weighing parameters.</summary>
/// <param name="TM">Tare (empty) mass.</param>
/// <param name="LM">Loaded mass.</param>
/// <param name="TBP">Tare-mass cylinder pressure.</param>
void TKE::SetLP(double const TM, double const LM, double const TBP)
{
	TareM = TM;
	LoadM = LM;
	TareBP = TBP;
}

/// <summary>Sets the rapid-step ratio. RM = 1 - RMR.</summary>
/// <param name="RMR">Reduction ratio.</param>
void TKE::SetRM(double const RMR)
{
	RM = 1.0 - RMR;
}

/// <summary>Sets the auxiliary (local) brake target pressure.</summary>
/// <param name="P">Auxiliary brake pressure [bar].</param>
void TKE::SetLBP(double const P)
{
	LBP = P;
}

/// <summary>
/// Vents valve, brake, control, impulse and secondary auxiliary reservoirs
/// to zero pressure (used on vehicle reset / decoupling).
/// </summary>
void TKE::ForceEmptiness()
{

	ValveRes->CreatePress(0);
	BrakeRes->CreatePress(0);
	CntrlRes->CreatePress(0);
	ImplsRes->CreatePress(0);
	Brak2Res->CreatePress(0);

	ValveRes->Act();
	BrakeRes->Act();
	CntrlRes->Act();
	ImplsRes->Act();
	Brak2Res->Act();
}

//---KRANY---

/// <summary>Default brake-pipe flow — 0. Concrete handles override.</summary>
double TDriverHandle::GetPF(double const i_bcp, double PP, double HP, double dt, double ep)
{
	return 0;
}

/// <summary>Default initialisation — disables the time / EP-time chambers.</summary>
/// <param name="Press">Initial pressure (unused in the base implementation).</param>
void TDriverHandle::Init(double Press)
{
	Time = false;
	TimeEP = false;
}

/// <summary>Default reductor adjustment — no-op.</summary>
/// <param name="nAdj">Pressure correction.</param>
void TDriverHandle::SetReductor(double nAdj) {}

/// <summary>Default cab-gauge pressure — 0.</summary>
double TDriverHandle::GetCP()
{
	return 0;
}

/// <summary>Default EP intensity — 0.</summary>
double TDriverHandle::GetEP()
{
	return 0;
}

/// <summary>Default regulator pressure — 0.</summary>
double TDriverHandle::GetRP()
{
	return 0;
}

/// <summary>Default per-channel sound magnitude — 0.</summary>
/// <param name="i">Sound channel index.</param>
double TDriverHandle::GetSound(int i)
{
	return 0;
}

/// <summary>Default position lookup — 0 (i.e. unknown).</summary>
/// <param name="i">Function code (bh_*).</param>
double TDriverHandle::GetPos(int i)
{
	return 0;
}

/// <summary>Default EP intensity for a handle position — 0.</summary>
/// <param name="pos">Handle position.</param>
double TDriverHandle::GetEP(double pos)
{
	return 0;
}

/// <summary>
/// Latches the manual overcharge / assimilation button state.
/// </summary>
/// <param name="Active">True while the button is pressed.</param>
void TDriverHandle::OvrldButton(bool Active)
{
	ManualOvrldActive = Active;
}

/// <summary>Stores the universal-button flags (combined ub_* values).</summary>
/// <param name="flag">Universal-button bitfield.</param>
void TDriverHandle::SetUniversalFlag(int flag)
{
	UniversalFlag = flag;
}
//---FV4a---

/// <summary>
/// Computes brake-pipe flow for the classic FV4a handle. Reads target pipe
/// pressure and flow speed from the BPT[] table for the current detent,
/// integrates the control (CP) and reductor (RP) reservoirs, and selects
/// the main-valve flow path based on the handle position (charge stroke at
/// -1, running-position bias at 0, full vent at the maximum detent).
/// </summary>
/// <param name="i_bcp">Continuous handle position.</param>
/// <param name="PP">Brake pipe pressure [bar].</param>
/// <param name="HP">High-pressure source [bar].</param>
/// <param name="dt">Time step [s].</param>
/// <param name="ep">EP / equalising input [bar].</param>
/// <returns>Brake pipe volume change for this step.</returns>
double TFV4a::GetPF(double i_bcp, double PP, double HP, double dt, double ep)
{
	static int const LBDelay = 100;

	ep = PP; // SPKS!!
	double LimPP = std::min(BPT[std::lround(i_bcp) + 2][1], HP);
	const double ActFlowSpeed = BPT[std::lround(i_bcp) + 2][0];

	if (i_bcp == i_bcpno)
		LimPP = 2.9;

	CP = CP + 20 * std::min(std::abs(LimPP - CP), 0.05) * PR(CP, LimPP) * dt / 1;
	RP = RP + 20 * std::min(std::abs(ep - RP), 0.05) * PR(RP, ep) * dt / 2.5;

	LimPP = CP;
	const double dpPipe = std::min(HP, LimPP);

	double dpMainValve = PF(dpPipe, PP, ActFlowSpeed / LBDelay) * dt;
	if (CP > RP + 0.05)
		dpMainValve = PF(std::min(CP + 0.1, HP), PP, 1.1 * ActFlowSpeed / LBDelay) * dt;
	if (CP < RP - 0.05)
		dpMainValve = PF(CP - 0.1, PP, 1.1 * ActFlowSpeed / LBDelay) * dt;

	if (lround(i_bcp) == -1)
	{
		CP = CP + 5 * std::min(std::abs(LimPP - CP), 0.2) * PR(CP, LimPP) * dt / 2;
		if (CP < RP + 0.03)
			if (TP < 5)
				TP = TP + dt;
		//            if(cp+0.03<5.4)then
		if (RP + 0.03 < 5.4 || CP + 0.03 < 5.4) // fala
			dpMainValve = PF(std::min(HP, 17.1), PP, ActFlowSpeed / LBDelay) * dt;
		//              dpMainValve:=20*std::min(abs(ep-7.1),0.05)*PF(HP,pp,ActFlowSpeed/LBDelay)*dt;
		else
		{
			RP = 5.45;
			if (CP < PP - 0.01) //: /34*9
				dpMainValve = PF(dpPipe, PP, ActFlowSpeed / 34 * 9 / LBDelay) * dt;
			else
				dpMainValve = PF(dpPipe, PP, ActFlowSpeed / LBDelay) * dt;
		}
	}

	if (lround(i_bcp) == 0)
	{
		if (TP > 0.1)
		{
			CP = 5 + (TP - 0.1) * 0.08;
			TP = TP - dt / 12 / 2;
		}
		if (CP > RP + 0.1 && CP <= 5)
			dpMainValve = PF(std::min(CP + 0.25, HP), PP, 2 * ActFlowSpeed / LBDelay) * dt;
		else if (CP > 5)
			dpMainValve = PF(std::min(CP, HP), PP, 2 * ActFlowSpeed / LBDelay) * dt;
		else
			dpMainValve = PF(dpPipe, PP, ActFlowSpeed / LBDelay) * dt;
	}

	if (lround(i_bcp) == i_bcpno)
	{
		dpMainValve = PF(0, PP, ActFlowSpeed / LBDelay) * dt;
	}

	return dpMainValve;
}

/// <summary>Initialises CP and RP to the supplied pressure.</summary>
/// <param name="Press">Initial pressure [bar].</param>
void TFV4a::Init(double Press)
{
	CP = Press;
	RP = Press;
}

//---FV4a/M--- nowonapisany kran bez poprawki IC

/// <summary>
/// Computes brake-pipe flow for the FV4a/M handle. Adds a release pressure
/// wave (XP/Fala) and a time-chamber driven overcharge stroke on top of the
/// FV4a logic; clamps i_bcp to the supported range, integrates CP/RP/TP/XP
/// and updates the Sounds[] channels for braking / release / wave / time
/// outflow / emergency.
/// </summary>
/// <param name="i_bcp">Continuous handle position.</param>
/// <param name="PP">Brake pipe pressure [bar].</param>
/// <param name="HP">High-pressure source [bar].</param>
/// <param name="dt">Time step [s].</param>
/// <param name="ep">EP / equalising input [bar].</param>
/// <returns>Brake pipe volume change for this step.</returns>
double TFV4aM::GetPF(double i_bcp, double PP, double HP, double dt, double ep)
{
	int const LBDelay{100};
	double const xpM{0.3}; // mnoznik membrany komory pod

	ep = PP / 2.0 * 1.5 + ep / 2.0 * 0.5; // SPKS!!

	for (int idx = 0; idx < 5; ++idx)
	{
		Sounds[idx] = 0;
	}

	// na wszelki wypadek, zeby nie wyszlo poza zakres
	i_bcp = std::clamp(i_bcp, -1.999, 5.999);

	double DP{0.0};
	if (TP > 0.0)
	{
		// jesli czasowy jest niepusty
		DP = 0.045; // 2.5 w 55 sekund (5,35->5,15 w PG)
		TP -= DP * dt;
		Sounds[s_fv4a_t] = DP;
	}
	else
	{
		//.08
		TP = 0.0;
	}

	if (XP > 0)
	{
		// jesli komora pod niepusta jest niepusty
		DP = 2.5;
		Sounds[s_fv4a_x] = DP * XP;
		XP -= dt * DP * 2.0; // od cisnienia 5 do 0 w 10 sekund ((5-0)*dt/10)
	}
	else
	{
		// jak pusty, to pusty
		XP = 0.0;
	}

	double pom;
	if (EQ(i_bcp, -1.0))
	{
		pom = std::min(HP, 5.4 + RedAdj);
	}
	else
	{
		pom = std::min(CP, HP);
	}

	if (pom > RP + 0.25)
	{
		Fala = true;
	}
	if (Fala)
	{
		if (pom > RP + 0.3)
		{
			XP = XP - 20.0 * PR(pom, XP) * dt;
		}
		else
		{
			Fala = false;
		}
	}

	double LimPP = std::min(LPP_RP(i_bcp) + TP * 0.08 + RedAdj,
	                        HP); // pozycja + czasowy lub zasilanie

	// zbiornik sterujacy
	if (LimPP > CP)
	{
		// podwyzszanie szybkie
		CP += 5.0 * 60.0 * std::min(std::abs(LimPP - CP), 0.05) * PR(CP, LimPP) * dt;
	}
	else
	{
		CP += 13 * std::min(std::abs(LimPP - CP), 0.05) * PR(CP, LimPP) * dt;
	}

	LimPP = pom; // cp
	double const dpPipe = std::min(HP, LimPP + XP * xpM);

	double const ActFlowSpeed = BPT[std::lround(i_bcp) + 2][0];

	double dpMainValve = dpPipe > PP ? -PFVa(HP, PP, ActFlowSpeed / LBDelay, dpPipe, 0.4) : PFVd(PP, 0, ActFlowSpeed / LBDelay, dpPipe, 0.4);

	if (EQ(i_bcp, -1))
	{

		if (TP < 5)
		{
			TP += dt;
		}
		if (TP < 1)
		{
			TP -= 0.5 * dt;
		}
	}

	if (EQ(i_bcp, 0))
	{

		if (TP > 2)
		{
			dpMainValve *= 1.5;
		}
	}

	ep = dpPipe;
	if (EQ(i_bcp, 0) || RP > ep)
	{
		// powolne wzrastanie, ale szybsze na jezdzie;
		RP += PF(RP, ep, 0.0007) * dt;
	}
	else
	{
		// powolne wzrastanie i to bardzo
		RP += PF(RP, ep, 0.000093 / 2 * 2) * dt;
	}
	// jednak trzeba wydluzyc, bo obecnie zle dziala
	if (RP < ep && RP < BPT[std::lround(i_bcpno) + 2][1])
	{
		// jesli jestesmy ponizej cisnienia w sterujacym (2.9 bar)
		// przypisz cisnienie w PG - wydluzanie napelniania o czas potrzebny do napelnienia PG
		RP += PF(RP, CP, 0.005) * dt;
	}

	if (EQ(i_bcp, i_bcpno) || EQ(i_bcp, -2))
	{

		DP = PF(0.0, PP, ActFlowSpeed / LBDelay);
		dpMainValve = DP;
		Sounds[s_fv4a_e] = DP;
		Sounds[s_fv4a_u] = 0.0;
		Sounds[s_fv4a_b] = 0.0;
		Sounds[s_fv4a_x] = 0.0;
	}
	else
	{

		if (dpMainValve > 0.0)
		{
			Sounds[s_fv4a_b] = dpMainValve;
		}
		else
		{
			Sounds[s_fv4a_u] = -dpMainValve;
		}
	}

	return dpMainValve * dt;
}

/// <summary>Initialises CP and RP to the supplied pressure.</summary>
/// <param name="Press">Initial pressure [bar].</param>
void TFV4aM::Init(double Press)
{
	CP = Press;
	RP = Press;
}

/// <summary>Sets the reductor adjustment offset (turning the reductor cap).</summary>
/// <param name="nAdj">Pressure correction [bar].</param>
void TFV4aM::SetReductor(double nAdj)
{
	RedAdj = nAdj;
}

/// <summary>Returns Sounds[i] for valid indices, 0 otherwise.</summary>
/// <param name="i">Sound channel index (s_fv4a_*).</param>
double TFV4aM::GetSound(int i)
{
	if (i > 4)
		return 0;
	else
		return Sounds[i];
}

/// <summary>Returns pos_table[i] (handle position for the requested function code).</summary>
/// <param name="i">Function code (bh_*).</param>
double TFV4aM::GetPos(int i)
{
	return pos_table[i];
}

/// <summary>Returns the time-chamber pressure (TP).</summary>
double TFV4aM::GetCP()
{
	return TP;
}

/// <summary>Returns the regulator target — 5 bar nominal plus TP * 0.08 plus RedAdj.</summary>
double TFV4aM::GetRP()
{
	return 5.0 + TP * 0.08 + RedAdj;
}

/// <summary>
/// Returns the target brake-pipe pressure for a continuous handle position
/// by linearly interpolating column 1 of BPT[] between the two surrounding
/// detents.
/// </summary>
/// <param name="pos">Continuous handle position.</param>
/// <returns>Interpolated target pressure [bar].</returns>
double TFV4aM::LPP_RP(double pos) // cisnienie z zaokraglonej pozycji;
{
	int const i_pos = 2 + std::floor(pos); // zaokraglone w dol

	return BPT[i_pos][1] + (BPT[i_pos + 1][1] - BPT[i_pos][1]) * (pos + 2 - i_pos); // interpolacja liniowa
}
/// <summary>Returns true if pos is within ±0.5 of i_pos (detent test).</summary>
/// <param name="pos">Continuous handle position.</param>
/// <param name="i_pos">Detent centre.</param>
bool TFV4aM::EQ(double pos, double i_pos)
{
	return pos <= i_pos + 0.5 && pos > i_pos - 0.5;
}

//---MHZ_EN57--- manipulator hamulca zespolonego do EN57

/// <summary>
/// Computes brake-pipe flow for the MHZ_EN57 combined handle. Drives CP/TP
/// based on the position via <see cref="LPP_RP"/>, supports auto/manual
/// overcharge (Auto/ManualOvrld and ub_HighPressure / ub_Overload buttons),
/// emits emergency-vent flow at the extreme detents, and exposes EP intensity
/// via the position-encoded RP value.
/// </summary>
/// <param name="i_bcp">Continuous handle position.</param>
/// <param name="PP">Brake pipe pressure [bar].</param>
/// <param name="HP">High-pressure source [bar].</param>
/// <param name="dt">Time step [s].</param>
/// <param name="ep">EP / equalising input [bar].</param>
/// <returns>Brake pipe volume change for this step.</returns>
double TMHZ_EN57::GetPF(double i_bcp, double PP, double HP, double dt, double ep)
{
	static int const LBDelay = 100;

	double LimPP;
	double dpPipe;
	double dpMainValve;
	double ActFlowSpeed;
	double DP;
	double pom;

	for (int idx = 0; idx < 5; ++idx)
	{
		Sounds[idx] = 0;
	}

	DP = 0;

	i_bcp = std::clamp(i_bcp, -0.999, 9.999); // na wszelki wypadek, zeby nie wyszlo poza zakres

	if (TP > 0 && CP > 4.9)
	{
		DP = OverloadPressureDecrease;
		if (EQ(i_bcp, 0))
			TP = TP - DP * dt;
		Sounds[s_fv4a_t] = DP;
	}
	else
	{
		TP = 0;
	}

	LimPP = std::min(LPP_RP(i_bcp) + TP * 0.08 + RedAdj, HP); // pozycja + czasowy lub zasilanie
	ActFlowSpeed = 4;

	double uop = UnbrakeOverPressure; // unbrake over pressure in actual state
	ManualOvrldActive = UniversalFlag & TUniversalBrake::ub_HighPressure; // button is pressed
	if (ManualOvrld && !ManualOvrldActive) // no overpressure for not pressed button if it does not exists
		uop = 0;

	if (EQ(i_bcp, -1) && uop > 0)
		pom = std::min(HP, 5.4 + RedAdj + uop);
	else
		pom = std::min(CP, HP);

	if (LimPP > CP) // podwyzszanie szybkie
		CP = CP + 60 * std::min(abs(LimPP - CP), 0.05) * PR(CP, LimPP) * dt; // zbiornik sterujacy;
	else
		CP = CP + 13 * std::min(abs(LimPP - CP), 0.05) * PR(CP, LimPP) * dt; // zbiornik sterujacy

	LimPP = pom; // cp
	// if (EQ(i_bcp, -1))
	//       dpPipe = HP;
	//   else
	dpPipe = std::min(HP, LimPP);

	if (dpPipe > PP)
		dpMainValve = -PFVa(HP, PP, ActFlowSpeed / LBDelay, dpPipe, 0.4);
	else
		dpMainValve = PFVd(PP, 0, ActFlowSpeed / LBDelay, dpPipe, 0.4);

	if ((EQ(i_bcp, -1) && AutoOvrld) || (i_bcp < 0.5 && UniversalFlag & TUniversalBrake::ub_Overload))
	{
		if (TP < 5)
			TP = TP + dt; // 5/10
		if (TP < OverloadMaxPressure)
			TP = TP - 0.5 * dt; // 5/10
	}

	if (EQ(i_bcp, 10) || EQ(i_bcp, -2))
	{
		DP = PF(0, PP, 2 * ActFlowSpeed / LBDelay);
		dpMainValve = DP;
		Sounds[s_fv4a_e] = DP;
		Sounds[s_fv4a_u] = 0;
		Sounds[s_fv4a_b] = 0;
		Sounds[s_fv4a_x] = 0;
	}
	else
	{
		if (dpMainValve > 0)
			Sounds[s_fv4a_b] = dpMainValve;
		else
			Sounds[s_fv4a_u] = -dpMainValve;
	}

	if (i_bcp < 1.5)
		RP = std::max(0., 0.125 * i_bcp);
	else
		RP = std::min(1., 0.125 * i_bcp - 0.125);

	return dpMainValve * dt;
}

/// <summary>Initialises CP to the supplied pressure.</summary>
/// <param name="Press">Initial pressure [bar].</param>
void TMHZ_EN57::Init(double Press)
{
	CP = Press;
}

/// <summary>Sets the reductor adjustment offset.</summary>
/// <param name="nAdj">Pressure correction [bar].</param>
void TMHZ_EN57::SetReductor(double nAdj)
{
	RedAdj = nAdj;
}

/// <summary>Returns Sounds[i] for valid indices, 0 otherwise.</summary>
/// <param name="i">Sound channel index.</param>
double TMHZ_EN57::GetSound(int i)
{
	if (i > 4)
		return 0;
	else
		return Sounds[i];
}

/// <summary>Returns pos_table[i].</summary>
/// <param name="i">Function code (bh_*).</param>
double TMHZ_EN57::GetPos(int i)
{
	return pos_table[i];
}

/// <summary>Returns RP — the regulator-target value computed during the last GetPF call.</summary>
double TMHZ_EN57::GetCP()
{
	return RP;
}

/// <summary>Returns the regulator target (5 + RedAdj).</summary>
double TMHZ_EN57::GetRP()
{
	return 5.0 + RedAdj;
}

/// <summary>
/// Returns the EP brake intensity for a given handle position
/// (linear ramp 0..1 between positions 0 and 8, zero above 9.5).
/// </summary>
/// <param name="pos">Handle position.</param>
double TMHZ_EN57::GetEP(double pos)
{
	if (pos < 9.5)
		return std::clamp(0.125 * pos, 0., 1.);
	else
		return 0;
}

/// <summary>
/// Returns the target brake-pipe pressure for a given handle position
/// (5 bar at running, gentle pressure reduction past position 0.5, steeper
/// reduction past position 8.5 toward emergency).
/// </summary>
/// <param name="pos">Continuous handle position.</param>
/// <returns>Target brake-pipe pressure [bar].</returns>
double TMHZ_EN57::LPP_RP(double pos) // cisnienie z zaokraglonej pozycji;
{
	if (pos > 8.5)
		return 5.0 - 0.15 * pos - 0.35;
	else if (pos > 0.5)
		return 5.0 - 0.15 * pos - 0.1;
	else
		return 5.0;
}

/// <summary>
/// Configures handle parameters: auto / manual overcharge support, the
/// unbrake over-pressure value (also enables the wave latch), the maximum
/// overcharge pressure and the overcharge decay rate.
/// </summary>
/// <param name="AO">Auto-overcharge enabled.</param>
/// <param name="MO">Manual-overcharge enabled.</param>
/// <param name="OverP">Unbrake over-pressure [bar].</param>
/// <param name="OMP">Overload (assimilation) max pressure.</param>
/// <param name="OPD">Overload pressure decay rate.</param>
void TMHZ_EN57::SetParams(bool AO, bool MO, double OverP, double, double OMP, double OPD)
{
	AutoOvrld = AO;
	ManualOvrld = MO;
	UnbrakeOverPressure = std::max(0.0, OverP);
	Fala = OverP > 0.01;
	OverloadMaxPressure = OMP;
	OverloadPressureDecrease = OPD;
}

/// <summary>Returns true if pos is within ±0.5 of i_pos (detent test).</summary>
bool TMHZ_EN57::EQ(double pos, double i_pos)
{
	return pos <= i_pos + 0.5 && pos > i_pos - 0.5;
}

//---MHZ_K5P--- manipulator hamulca zespolonego Knorr 5-ciopozycyjny

/// <summary>
/// Computes brake-pipe flow for the Knorr 5-position MHZ_K5P handle
/// (release / cut-off / brake / emergency layout). Handles auto / manual
/// overcharge, the optional release wave (Fala) with FillingStrokeFactor,
/// and the emergency vent at position 3.
/// </summary>
/// <param name="i_bcp">Continuous handle position.</param>
/// <param name="PP">Brake pipe pressure [bar].</param>
/// <param name="HP">High-pressure source [bar].</param>
/// <param name="dt">Time step [s].</param>
/// <param name="ep">EP / equalising input [bar].</param>
/// <returns>Brake pipe volume change for this step.</returns>
double TMHZ_K5P::GetPF(double i_bcp, double PP, double HP, double dt, double ep)
{
	static int const LBDelay = 100;

	double LimCP;
	double dpPipe;
	double dpMainValve;
	double ActFlowSpeed;
	double DP;
	double pom;

	for (int idx = 0; idx < 5; ++idx)
	{
		Sounds[idx] = 0;
	}

	DP = 0;

	i_bcp = std::clamp(i_bcp, -0.999, 2.999); // na wszelki wypadek, zeby nie wyszlo poza zakres

	if (TP > 0 && CP > 4.9)
	{
		DP = OverloadPressureDecrease;
		TP = TP - DP * dt;
		Sounds[s_fv4a_t] = DP;
	}
	else
	{
		// TP = 0; //tu nie powinno być nic, ciśnienie zostaje jak było
	}

	if (EQ(i_bcp, 1)) // odcięcie - nie rób nic
		LimCP = CP;
	else if (i_bcp > 1) // hamowanie
		LimCP = 3.4;
	else // luzowanie
		LimCP = 5.0;
	pom = CP;
	LimCP = std::min(LimCP, HP); // pozycja + czasowy lub zasilanie
	ActFlowSpeed = 4;

	if (LimCP > CP) // podwyzszanie szybkie
		CP = CP + 9 * std::min(abs(LimCP - CP), 0.05) * PR(CP, LimCP) * dt; // zbiornik sterujacy;
	else
		CP = CP + 9 * std::min(abs(LimCP - CP), 0.05) * PR(CP, LimCP) * dt; // zbiornik sterujacy

	double uop = UnbrakeOverPressure; // unbrake over pressure in actual state
	ManualOvrldActive = UniversalFlag & TUniversalBrake::ub_HighPressure; // button is pressed
	if (ManualOvrld && !ManualOvrldActive) // no overpressure for not pressed button if it does not exists
		uop = 0;

	dpPipe = std::min(HP, CP + TP + RedAdj);

	if (EQ(i_bcp, -1))
	{
		if (Fala)
		{
			dpPipe = 5.0 + TP + RedAdj + uop;
			ActFlowSpeed = 12;
		}
		else
		{
			ActFlowSpeed *= FillingStrokeFactor;
		}
	}

	if (dpPipe > PP)
		dpMainValve = -PFVa(HP, PP, ActFlowSpeed / LBDelay, dpPipe, 0.4);
	else
		dpMainValve = PFVd(PP, 0, ActFlowSpeed / LBDelay, dpPipe, 0.4);

	if ((EQ(i_bcp, -1) && AutoOvrld) || (i_bcp < 0.5 && UniversalFlag & TUniversalBrake::ub_Overload))
	{
		if (TP < OverloadMaxPressure)
			TP = TP + 0.03 * dt;
	}

	if (EQ(i_bcp, 3))
	{
		DP = PF(0, PP, 2 * ActFlowSpeed / LBDelay);
		dpMainValve = DP;
		Sounds[s_fv4a_e] = DP;
		Sounds[s_fv4a_u] = 0;
		Sounds[s_fv4a_b] = 0;
		Sounds[s_fv4a_x] = 0;
	}
	else
	{
		if (dpMainValve > 0)
			Sounds[s_fv4a_b] = dpMainValve;
		else
			Sounds[s_fv4a_u] = -dpMainValve;
	}

	return dpMainValve * dt;
}

/// <summary>Initialises CP and enables the time / EP-time chambers.</summary>
/// <param name="Press">Initial pressure [bar].</param>
void TMHZ_K5P::Init(double Press)
{
	CP = Press;
	Time = true;
	TimeEP = true;
}

/// <summary>Sets the reductor adjustment offset.</summary>
/// <param name="nAdj">Pressure correction [bar].</param>
void TMHZ_K5P::SetReductor(double nAdj)
{
	RedAdj = nAdj;
}

/// <summary>Returns Sounds[i] for valid indices, 0 otherwise.</summary>
/// <param name="i">Sound channel index.</param>
double TMHZ_K5P::GetSound(int i)
{
	if (i > 4)
		return 0;
	else
		return Sounds[i];
}

/// <summary>Returns pos_table[i].</summary>
/// <param name="i">Function code (bh_*).</param>
double TMHZ_K5P::GetPos(int i)
{
	return pos_table[i];
}

/// <summary>Returns CP.</summary>
double TMHZ_K5P::GetCP()
{
	return CP;
}

/// <summary>Returns the regulator target (5 + TP + RedAdj).</summary>
double TMHZ_K5P::GetRP()
{
	return 5.0 + TP + RedAdj;
}

/// <summary>
/// Configures handle parameters: auto/manual overcharge, unbrake over-pressure
/// (also drives the Fala latch), filling-stroke factor (FSF + 1) and the
/// overcharge max pressure / decay rate.
/// </summary>
/// <param name="AO">Auto-overcharge enabled.</param>
/// <param name="MO">Manual-overcharge enabled.</param>
/// <param name="OverP">Unbrake over-pressure [bar].</param>
/// <param name="FSF">Filling-stroke factor offset (FillingStrokeFactor = 1 + FSF).</param>
/// <param name="OMP">Overload max pressure.</param>
/// <param name="OPD">Overload pressure decay rate.</param>
void TMHZ_K5P::SetParams(bool AO, bool MO, double OverP, double FSF, double OMP, double OPD)
{
	AutoOvrld = AO;
	ManualOvrld = MO;
	UnbrakeOverPressure = std::max(0.0, OverP);
	Fala = OverP > 0.01;
	OverloadMaxPressure = OMP;
	OverloadPressureDecrease = OPD;
	FillingStrokeFactor = 1 + FSF;
}

/// <summary>Returns true if pos is within ±0.5 of i_pos (detent test).</summary>
bool TMHZ_K5P::EQ(double pos, double i_pos)
{
	return pos <= i_pos + 0.5 && pos > i_pos - 0.5;
}

//---MHZ_6P--- manipulator hamulca zespolonego 6-ciopozycyjny

/// <summary>
/// Computes brake-pipe flow for the 6-position MHZ_6P handle. Variant of K5P
/// with one additional detent — detent 4 triggers the emergency vent, detent 2
/// is the cut-off (lap), positions &gt; 2.5 brake, positions &lt; 2 release.
/// </summary>
/// <param name="i_bcp">Continuous handle position.</param>
/// <param name="PP">Brake pipe pressure [bar].</param>
/// <param name="HP">High-pressure source [bar].</param>
/// <param name="dt">Time step [s].</param>
/// <param name="ep">EP / equalising input [bar].</param>
/// <returns>Brake pipe volume change for this step.</returns>
double TMHZ_6P::GetPF(double i_bcp, double PP, double HP, double dt, double ep)
{
	static int const LBDelay = 100;

	double LimCP;
	double dpPipe;
	double dpMainValve;
	double ActFlowSpeed;
	double DP;
	double pom;

	for (int idx = 0; idx < 5; ++idx)
	{
		Sounds[idx] = 0;
	}

	DP = 0;

	i_bcp = std::clamp(i_bcp, -0.999, 3.999); // na wszelki wypadek, zeby nie wyszlo poza zakres

	if (TP > 0 && CP > 4.9)
	{
		DP = OverloadPressureDecrease;
		TP = TP - DP * dt;
		Sounds[s_fv4a_t] = DP;
	}
	else
	{
		// TP = 0; //tu nie powinno być nic, ciśnienie zostaje jak było
	}

	if (EQ(i_bcp, 2)) // odcięcie - nie rób nic
		LimCP = CP;
	else if (i_bcp > 2.5) // hamowanie
		LimCP = 3.4;
	else // luzowanie
		LimCP = 5.0;
	pom = CP;
	LimCP = std::min(LimCP, HP); // pozycja + czasowy lub zasilanie
	ActFlowSpeed = 4;

	if (LimCP > CP) // podwyzszanie szybkie
		CP = CP + 9 * std::min(abs(LimCP - CP), 0.05) * PR(CP, LimCP) * dt; // zbiornik sterujacy;
	else
		CP = CP + 9 * std::min(abs(LimCP - CP), 0.05) * PR(CP, LimCP) * dt; // zbiornik sterujacy

	dpPipe = std::min(HP, CP + TP + RedAdj);

	double uop = UnbrakeOverPressure; // unbrake over pressure in actual state
	ManualOvrldActive = UniversalFlag & TUniversalBrake::ub_HighPressure; // button is pressed
	if (ManualOvrld && !ManualOvrldActive) // no overpressure for not pressed button if it does not exists
		uop = 0;

	if (EQ(i_bcp, -1))
	{
		if (Fala)
		{
			dpPipe = 5.0 + TP + RedAdj + uop;
			ActFlowSpeed = 12;
		}
		else
		{
			ActFlowSpeed *= FillingStrokeFactor;
		}
	}

	if (dpPipe > PP)
		dpMainValve = -PFVa(HP, PP, ActFlowSpeed / LBDelay, dpPipe, 0.4);
	else
		dpMainValve = PFVd(PP, 0, ActFlowSpeed / LBDelay, dpPipe, 0.4);

	if ((EQ(i_bcp, -1) && AutoOvrld) || (i_bcp < 0.5 && UniversalFlag & TUniversalBrake::ub_Overload))
	{
		if (TP < OverloadMaxPressure)
			TP = TP + 0.03 * dt;
	}

	if (EQ(i_bcp, 4))
	{
		DP = PF(0, PP, 2 * ActFlowSpeed / LBDelay);
		dpMainValve = DP;
		Sounds[s_fv4a_e] = DP;
		Sounds[s_fv4a_u] = 0;
		Sounds[s_fv4a_b] = 0;
		Sounds[s_fv4a_x] = 0;
	}
	else
	{
		if (dpMainValve > 0)
			Sounds[s_fv4a_b] = dpMainValve;
		else
			Sounds[s_fv4a_u] = -dpMainValve;
	}

	return dpMainValve * dt;
}

/// <summary>Initialises CP and enables the time / EP-time chambers.</summary>
/// <param name="Press">Initial pressure [bar].</param>
void TMHZ_6P::Init(double Press)
{
	CP = Press;
	Time = true;
	TimeEP = true;
}

/// <summary>Sets the reductor adjustment offset.</summary>
/// <param name="nAdj">Pressure correction [bar].</param>
void TMHZ_6P::SetReductor(double nAdj)
{
	RedAdj = nAdj;
}

/// <summary>Returns Sounds[i] for valid indices, 0 otherwise.</summary>
/// <param name="i">Sound channel index.</param>
double TMHZ_6P::GetSound(int i)
{
	if (i > 4)
		return 0;
	else
		return Sounds[i];
}

/// <summary>Returns pos_table[i].</summary>
/// <param name="i">Function code (bh_*).</param>
double TMHZ_6P::GetPos(int i)
{
	return pos_table[i];
}

/// <summary>Returns CP.</summary>
double TMHZ_6P::GetCP()
{
	return CP;
}

/// <summary>Returns the regulator target (5 + TP + RedAdj).</summary>
double TMHZ_6P::GetRP()
{
	return 5.0 + TP + RedAdj;
}

/// <summary>
/// Configures handle parameters: auto/manual overcharge, unbrake over-pressure
/// (also drives the Fala latch), filling-stroke factor (FSF + 1) and the
/// overcharge max pressure / decay rate.
/// </summary>
/// <param name="AO">Auto-overcharge enabled.</param>
/// <param name="MO">Manual-overcharge enabled.</param>
/// <param name="OverP">Unbrake over-pressure [bar].</param>
/// <param name="FSF">Filling-stroke factor offset.</param>
/// <param name="OMP">Overload max pressure.</param>
/// <param name="OPD">Overload pressure decay rate.</param>
void TMHZ_6P::SetParams(bool AO, bool MO, double OverP, double FSF, double OMP, double OPD)
{
	AutoOvrld = AO;
	ManualOvrld = MO;
	UnbrakeOverPressure = std::max(0.0, OverP);
	Fala = OverP > 0.01;
	OverloadMaxPressure = OMP;
	OverloadPressureDecrease = OPD;
	FillingStrokeFactor = 1 + FSF;
}

/// <summary>Returns true if pos is within ±0.5 of i_pos (detent test).</summary>
bool TMHZ_6P::EQ(double pos, double i_pos)
{
	return pos <= i_pos + 0.5 && pos > i_pos - 0.5;
}

//---M394--- Matrosow

/// <summary>
/// Computes brake-pipe flow for the Matrosow 394 handle. Reads target pipe
/// pressure and flow speed from BPT_394[] for the current detent (with
/// special handling at the running and emergency positions) and integrates
/// the control reservoir CP at a position-dependent rate.
/// </summary>
/// <param name="i_bcp">Continuous handle position.</param>
/// <param name="PP">Brake pipe pressure [bar].</param>
/// <param name="HP">High-pressure source [bar].</param>
/// <param name="dt">Time step [s].</param>
/// <param name="ep">EP / equalising input [bar].</param>
/// <returns>Brake pipe volume change for this step.</returns>
double TM394::GetPF(double i_bcp, double PP, double HP, double dt, double ep)
{
	static int const LBDelay = 65;

	double LimPP;
	double dpPipe;
	double dpMainValve;
	double ActFlowSpeed;
	int BCP;

	BCP = lround(i_bcp);
	if (BCP < -1)
		BCP = 1;

	LimPP = std::min(BPT_394[BCP + 1][1], HP);
	ActFlowSpeed = BPT_394[BCP + 1][0];
	if (BCP == 1 || BCP == i_bcpno)
		LimPP = PP;
	if (BCP == 0)
		LimPP = LimPP + RedAdj;
	if (BCP != 2)
		if (CP < LimPP)
			CP = CP + 4 * std::min(abs(LimPP - CP), 0.05) * PR(CP, LimPP) * dt; // zbiornik sterujacy
		//      cp:=cp+6*(2+int(bcp<0))*std::min(abs(Limpp-cp),0.05)*PR(cp,Limpp)*dt //zbiornik
		//      sterujacy;
		else if (BCP == 0)
			CP = CP - 0.2 * dt / 100;
		else
			CP = CP + 4 * (1 + int(BCP != 3) + int(BCP > 4)) * std::min(abs(LimPP - CP), 0.05) * PR(CP, LimPP) * dt; // zbiornik sterujacy

	LimPP = CP;
	dpPipe = std::min(HP, LimPP);

	//  if(dpPipe>pp)then //napelnianie
	//    dpMainValve:=PF(dpPipe,pp,ActFlowSpeed/LBDelay)*dt
	//  else //spuszczanie
	dpMainValve = PF(dpPipe, PP, ActFlowSpeed / LBDelay) * dt;

	if (BCP == -1)
		dpMainValve = PF(HP, PP, ActFlowSpeed / LBDelay) * dt;

	if (BCP == i_bcpno)
		dpMainValve = PF(0, PP, ActFlowSpeed / LBDelay) * dt;

	return dpMainValve;
}

/// <summary>Initialises CP and enables the time chamber.</summary>
/// <param name="Press">Initial pressure [bar].</param>
void TM394::Init(double Press)
{
	CP = Press;
	Time = true;
}

/// <summary>Sets the reductor adjustment offset.</summary>
/// <param name="nAdj">Pressure correction [bar].</param>
void TM394::SetReductor(double nAdj)
{
	RedAdj = nAdj;
}

/// <summary>Returns CP.</summary>
double TM394::GetCP()
{
	return CP;
}

/// <summary>Returns max(5, CP) + RedAdj as the regulator target.</summary>
double TM394::GetRP()
{
	return std::max(5.0, CP) + RedAdj;
}

/// <summary>Returns pos_table[i].</summary>
/// <param name="i">Function code (bh_*).</param>
double TM394::GetPos(int i)
{
	return pos_table[i];
}

//---H14K1-- Knorr

/// <summary>
/// Computes brake-pipe flow for the Knorr H14K1 auxiliary handle.
/// Picks a target pressure level (high/low/lap) from BPT_K[] depending on the
/// detent, integrates CP toward it and dispatches the resulting flow path
/// (charge / fill toward nominal / vent toward CP / emergency vent at
/// the maximum detent).
/// </summary>
/// <param name="i_bcp">Continuous handle position.</param>
/// <param name="PP">Brake pipe pressure [bar].</param>
/// <param name="HP">High-pressure source [bar].</param>
/// <param name="dt">Time step [s].</param>
/// <param name="ep">EP / equalising input [bar].</param>
/// <returns>Brake pipe volume change for this step.</returns>
double TH14K1::GetPF(double i_bcp, double PP, double HP, double dt, double ep)
{
	int const LBDelay = 100; // szybkosc + zasilanie sterujacego
	//   static double const BPT_K[/*?*/ /*-1..4*/ (4) - (-1) + 1][2] =
	//{ (10, 0), (4, 1), (0, 1), (4, 0), (4, -1), (15, -1) };
	double const NomPress = 5.0;

	int BCP = std::lround(i_bcp);
	if (i_bcp < -1)
	{
		BCP = 1;
	}

	double LimPP = BPT_K[BCP + 1][1];
	if (LimPP < 0.0)
	{
		LimPP = 0.5 * PP;
	}
	else if (LimPP > 0.0)
	{
		LimPP = PP;
	}
	else
	{
		LimPP = CP;
	}
	const double ActFlowSpeed = BPT_K[BCP + 1][0];

	CP = CP + 6 * std::min(std::abs(LimPP - CP), 0.05) * PR(CP, LimPP) * dt; // zbiornik sterujacy

	double dpMainValve = 0.0;

	if (BCP == -1)
		dpMainValve = PF(HP, PP, ActFlowSpeed / LBDelay) * dt;
	if (BCP == 0)
		dpMainValve = -PFVa(HP, PP, ActFlowSpeed / LBDelay, NomPress + RedAdj) * dt;
	if (BCP > 1 && PP > CP)
		dpMainValve = PFVd(PP, 0, ActFlowSpeed / LBDelay, CP) * dt;
	if (BCP == i_bcpno)
		dpMainValve = PF(0, PP, ActFlowSpeed / LBDelay) * dt;

	return dpMainValve;
}

/// <summary>Initialises CP and enables the time / EP-time chambers.</summary>
/// <param name="Press">Initial pressure [bar].</param>
void TH14K1::Init(double Press)
{
	CP = Press;
	Time = true;
	TimeEP = true;
}

/// <summary>Sets the reductor adjustment offset.</summary>
/// <param name="nAdj">Pressure correction [bar].</param>
void TH14K1::SetReductor(double nAdj)
{
	RedAdj = nAdj;
}

/// <summary>Returns CP.</summary>
double TH14K1::GetCP()
{
	return CP;
}

/// <summary>Returns the regulator target (5 + RedAdj).</summary>
double TH14K1::GetRP()
{
	return 5.0 + RedAdj;
}

/// <summary>Returns pos_table[i].</summary>
/// <param name="i">Function code (bh_*).</param>
double TH14K1::GetPos(int i)
{
	return pos_table[i];
}

//---St113-- Knorr EP

/// <summary>
/// Computes brake-pipe flow for the Knorr St113 EP-equipped handle.
/// Reuses the H14K1 flow paths (after collapsing position 1 into the cut-off
/// region) and additionally publishes the EP intensity from BEP_K[].
/// </summary>
/// <param name="i_bcp">Continuous handle position.</param>
/// <param name="PP">Brake pipe pressure [bar].</param>
/// <param name="HP">High-pressure source [bar].</param>
/// <param name="dt">Time step [s].</param>
/// <param name="ep">EP / equalising input [bar].</param>
/// <returns>Brake pipe volume change for this step.</returns>
double TSt113::GetPF(double i_bcp, double PP, double HP, double dt, double ep)
{
	static int const LBDelay = 100; // szybkosc + zasilanie sterujacego
	static double const NomPress = 5.0;

	double LimPP;
	double dpMainValve;
	double ActFlowSpeed;
	int BCP;

	CP = PP;

	BCP = lround(i_bcp);

	EPS = BEP_K[BCP + 1];

	if (BCP > 0)
		BCP = BCP - 1;

	if (BCP < -1)
		BCP = 1;
	LimPP = BPT_K[BCP + 1][1];
	if (LimPP < 0)
		LimPP = 0.5 * PP;
	else if (LimPP > 0)
		LimPP = PP;
	else
		LimPP = CP;
	ActFlowSpeed = BPT_K[BCP + 1][0];

	CP = CP + 6 * std::min(abs(LimPP - CP), 0.05) * PR(CP, LimPP) * dt; // zbiornik sterujacy

	dpMainValve = 0;

	if (BCP == -1)
		dpMainValve = PF(HP, PP, ActFlowSpeed / LBDelay) * dt;
	if (BCP == 0)
		dpMainValve = -PFVa(HP, PP, ActFlowSpeed / LBDelay, NomPress + RedAdj) * dt;
	if (BCP > 1 && PP > CP)
		dpMainValve = PFVd(PP, 0, ActFlowSpeed / LBDelay, CP) * dt;
	if (BCP == i_bcpno)
		dpMainValve = PF(0, PP, ActFlowSpeed / LBDelay) * dt;

	return dpMainValve;
}

/// <summary>Returns CP.</summary>
double TSt113::GetCP()
{
	return CP;
}

/// <summary>Returns the regulator target (5 + RedAdj).</summary>
double TSt113::GetRP()
{
	return 5.0 + RedAdj;
}

/// <summary>Returns the current EP intensity (read from BEP_K via the last GetPF call).</summary>
double TSt113::GetEP()
{
	return EPS;
}

/// <summary>Returns pos_table[i].</summary>
/// <param name="i">Function code (bh_*).</param>
double TSt113::GetPos(int i)
{
	return pos_table[i];
}

/// <summary>Enables the time / EP-time chambers (no pressure init needed).</summary>
/// <param name="Press">Initial pressure (unused).</param>
void TSt113::Init(double Press)
{
	Time = true;
	TimeEP = true;
}

//--- test ---

/// <summary>
/// Computes brake-pipe flow for the test-only handle. Uses BPT[] but
/// overrides the limit pressure for the highest detent (full vent) and
/// for position -1 (raises target to 7 bar).
/// </summary>
/// <param name="i_bcp">Continuous handle position.</param>
/// <param name="PP">Brake pipe pressure [bar].</param>
/// <param name="HP">High-pressure source [bar].</param>
/// <param name="dt">Time step [s].</param>
/// <param name="ep">EP / equalising input [bar].</param>
/// <returns>Brake pipe volume change for this step.</returns>
double Ttest::GetPF(double i_bcp, double PP, double HP, double dt, double ep)
{
	static int const LBDelay = 100;

	double LimPP;
	double dpPipe;
	double dpMainValve;
	double ActFlowSpeed;

	LimPP = BPT[lround(i_bcp) + 2][1];
	ActFlowSpeed = BPT[lround(i_bcp) + 2][0];

	if (i_bcp == i_bcpno)
		LimPP = 0.0;

	if (i_bcp == -1)
		LimPP = 7;

	CP = CP + 20 * std::min(abs(LimPP - CP), 0.05) * PR(CP, LimPP) * dt / 1;

	LimPP = CP;
	dpPipe = std::min(HP, LimPP);

	dpMainValve = PF(dpPipe, PP, ActFlowSpeed / LBDelay) * dt;

	if (lround(i_bcp) == i_bcpno)
	{
		dpMainValve = PF(0, PP, ActFlowSpeed / LBDelay) * dt;
	}

	return dpMainValve;
}

/// <summary>Initialises CP to the supplied pressure.</summary>
/// <param name="Press">Initial pressure [bar].</param>
void Ttest::Init(double Press)
{
	CP = Press;
}

//---FD1---

/// <summary>
/// Computes the auxiliary-brake outflow for the FD1 handle: drives the
/// cylinder pressure BP toward i_bcp * MaxBP at a configurable Speed,
/// using a soft-clamped PF flow with asymmetric fill/release rates.
/// </summary>
/// <param name="i_bcp">Continuous handle position (0..1).</param>
/// <param name="PP">Brake pipe pressure (unused).</param>
/// <param name="HP">High-pressure source [bar].</param>
/// <param name="dt">Time step [s].</param>
/// <param name="ep">EP / equalising input (unused).</param>
/// <returns>Negated cylinder pressure delta for this step.</returns>
double TFD1::GetPF(double i_bcp, double PP, double HP, double dt, double ep)
{
	double DP;
	double temp;

	//  MaxBP:=4;
	//  temp:=std::min(i_bcp*MaxBP,std::min(5.0,HP));
	temp = std::min(i_bcp * MaxBP, HP); // 0011
	DP = 10.0 * std::min(std::abs(temp - BP), 0.1) * PF(temp, BP, 0.0006 * (temp > BP ? 3.0 : 2.0)) * dt * Speed;
	BP = BP - DP;
	return -DP;
}

/// <summary>Initialises MaxBP and the action speed (defaults to 1.0).</summary>
/// <param name="Press">Maximum cylinder pressure [bar].</param>
void TFD1::Init(double Press)
{
	MaxBP = Press;
	Speed = 1.0;
}

/// <summary>Returns the currently commanded cylinder pressure (BP).</summary>
double TFD1::GetCP()
{
	return BP;
}

/// <summary>Sets the action speed multiplier (scales the response time).</summary>
/// <param name="nSpeed">Speed multiplier.</param>
void TFD1::SetSpeed(double nSpeed)
{
	Speed = nSpeed;
}

//---KNORR---

/// <summary>
/// Computes the auxiliary-brake outflow for the Knorr H1405 handle.
/// Above 0.5 the handle drives the cylinder up toward MaxBP (apply); below
/// it the cylinder is vented toward zero. The orifice area depends linearly
/// on the deflection from the lap point.
/// </summary>
/// <param name="i_bcp">Continuous handle position.</param>
/// <param name="PP">Brake pipe pressure clamped to MaxBP.</param>
/// <param name="HP">High-pressure source [bar].</param>
/// <param name="dt">Time step [s].</param>
/// <param name="ep">EP / equalising input (unused).</param>
/// <returns>Negated cylinder pressure delta for this step.</returns>
double TH1405::GetPF(double i_bcp, double PP, double HP, double dt, double ep)
{
	double DP;
	double temp;
	double A;

	PP = std::min(PP, MaxBP);
	if (i_bcp > 0.5)
	{
		temp = std::min(MaxBP, HP);
		A = 2 * (i_bcp - 0.5) * 0.0011;
		BP = std::max(BP, PP);
	}
	else
	{
		temp = 0;
		A = 0.2 * (0.5 - i_bcp) * 0.0033;
		BP = std::min(BP, PP);
	}
	DP = PF(temp, BP, A) * dt;
	BP = BP - DP;
	return -DP;
}

/// <summary>Initialises MaxBP and enables the time chamber.</summary>
/// <param name="Press">Maximum cylinder pressure [bar].</param>
void TH1405::Init(double Press)
{
	MaxBP = Press;
	Time = true;
}

/// <summary>Returns the currently commanded cylinder pressure (BP).</summary>
double TH1405::GetCP()
{
	return BP;
}

//---FVel6---

/// <summary>
/// Computes brake-pipe flow for the FVel6 combined handle. Position-dependent
/// flow speed selects between full release, lap, brake (positions 4.3..4.8)
/// and emergency (above 5.5). Also publishes EP intensity (-1, 0 or 1) and
/// the corresponding sound channels.
/// </summary>
/// <param name="i_bcp">Continuous handle position.</param>
/// <param name="PP">Brake pipe pressure [bar].</param>
/// <param name="HP">High-pressure source [bar].</param>
/// <param name="dt">Time step [s].</param>
/// <param name="ep">EP / equalising input (unused).</param>
/// <returns>Brake pipe volume change for this step.</returns>
double TFVel6::GetPF(double i_bcp, double PP, double HP, double dt, double ep)
{
	static int const LBDelay = 100;

	double LimPP;
	double dpMainValve;
	double ActFlowSpeed;

	CP = PP;

	LimPP = std::min(5. * int(i_bcp < 3.5), HP);
	if (i_bcp >= 3.5 && (i_bcp < 4.3 || i_bcp > 5.5))
		ActFlowSpeed = 0;
	else if (i_bcp > 4.3 && i_bcp < 4.8)
		ActFlowSpeed = 4 * (i_bcp - 4.3); // konsultacje wawa1 - bylo 8;
	else if (i_bcp < 4)
		ActFlowSpeed = 2;
	else
		ActFlowSpeed = 4;
	dpMainValve = PF(LimPP, PP, ActFlowSpeed / LBDelay) * dt;

	Sounds[s_fv4a_e] = 0;
	Sounds[s_fv4a_u] = 0;
	Sounds[s_fv4a_b] = 0;
	if (i_bcp < 3.5)
		Sounds[s_fv4a_u] = -dpMainValve;
	else if (i_bcp < 4.8)
		Sounds[s_fv4a_b] = dpMainValve;
	else if (i_bcp < 5.5)
		Sounds[s_fv4a_e] = dpMainValve;

	if (i_bcp < -0.5)
		EPS = -1;
	else if (i_bcp > 0.5 && i_bcp < 4.7)
		EPS = 1;
	else
		EPS = 0;
	//    EPS:=i_bcp*int(i_bcp<2)
	return dpMainValve;
}

/// <summary>Returns CP.</summary>
double TFVel6::GetCP()
{
	return CP;
}

/// <summary>Returns the constant 5 bar regulator target.</summary>
double TFVel6::GetRP()
{
	return 5.0;
}

/// <summary>Returns the current EP intensity.</summary>
double TFVel6::GetEP()
{
	return EPS;
}

/// <summary>Returns pos_table[i].</summary>
/// <param name="i">Function code (bh_*).</param>
double TFVel6::GetPos(int i)
{
	return pos_table[i];
}

/// <summary>Returns Sounds[i] for valid indices, 0 otherwise.</summary>
/// <param name="i">Sound channel index (0..2).</param>
double TFVel6::GetSound(int i)
{
	if (i > 2)
		return 0;
	else
		return Sounds[i];
}

/// <summary>Enables the time and EP-time chambers (no pressure init).</summary>
/// <param name="Press">Initial pressure (unused).</param>
void TFVel6::Init(double Press)
{
	Time = true;
	TimeEP = true;
}

//---FVE408---

/// <summary>
/// Computes brake-pipe flow for the FVE408 combined handle. Splits the flow
/// behaviour into release (positions &lt; 6.5), brake (7.5..8.5), emergency
/// (8.5..9.5) and locked positions; sets EPS to fixed steps at detents 1..5.
/// </summary>
/// <param name="i_bcp">Continuous handle position.</param>
/// <param name="PP">Brake pipe pressure [bar].</param>
/// <param name="HP">High-pressure source [bar].</param>
/// <param name="dt">Time step [s].</param>
/// <param name="ep">EP / equalising input (unused).</param>
/// <returns>Brake pipe volume change for this step.</returns>
double TFVE408::GetPF(double i_bcp, double PP, double HP, double dt, double ep)
{
	static int const LBDelay = 100;

	double LimPP;
	double dpMainValve;
	double ActFlowSpeed;

	CP = PP;

	LimPP = std::min(5. * int(i_bcp < 6.5), HP);
	if (i_bcp >= 6.5 && (i_bcp < 7.5 || i_bcp > 9.5))
		ActFlowSpeed = 0;
	else if (i_bcp > 7.5 && i_bcp < 8.5)
		ActFlowSpeed = 2; // konsultacje wawa1 - bylo 8;
	else if (i_bcp < 6.5)
		ActFlowSpeed = 2;
	else
		ActFlowSpeed = 4;
	dpMainValve = PF(LimPP, PP, ActFlowSpeed / LBDelay) * dt;

	Sounds[s_fv4a_e] = 0;
	Sounds[s_fv4a_u] = 0;
	Sounds[s_fv4a_b] = 0;
	if (i_bcp < 6.5)
		Sounds[s_fv4a_u] = -dpMainValve;
	else if (i_bcp < 8.5)
		Sounds[s_fv4a_b] = dpMainValve;
	else if (i_bcp < 9.5)
		Sounds[s_fv4a_e] = dpMainValve;

	if (is_EQ(i_bcp, 1))
		EPS = 1.15;
	else if (is_EQ(i_bcp, 2))
		EPS = 1.40;
	else if (is_EQ(i_bcp, 3))
		EPS = 2.64;
	else if (is_EQ(i_bcp, 4))
		EPS = 3.84;
	else if (is_EQ(i_bcp, 5))
		EPS = 3.99;
	else
		EPS = 0;

	return dpMainValve;
}

/// <summary>Returns CP.</summary>
double TFVE408::GetCP()
{
	return CP;
}

/// <summary>Returns the current EP intensity.</summary>
double TFVE408::GetEP()
{
	return EPS;
}

/// <summary>Returns the constant 5 bar regulator target.</summary>
double TFVE408::GetRP()
{
	return 5.0;
}

/// <summary>Returns pos_table[i].</summary>
/// <param name="i">Function code (bh_*).</param>
double TFVE408::GetPos(int i)
{
	return pos_table[i];
}

/// <summary>Returns Sounds[i] for valid indices, 0 otherwise.</summary>
/// <param name="i">Sound channel index (0..2).</param>
double TFVE408::GetSound(int i)
{
	if (i > 2)
		return 0;
	else
		return Sounds[i];
}

/// <summary>Enables the time chamber, disables the EP-time chamber.</summary>
/// <param name="Press">Initial pressure (unused).</param>
void TFVE408::Init(double Press)
{
	Time = true;
	TimeEP = false;
}

// END
