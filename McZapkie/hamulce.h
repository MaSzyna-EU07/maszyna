/*fizyka hamulcow dla symulatora*/

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
    Brakes.
    Copyright (C) 2007-2014 Maciej Cierniak
*/

/*
(C) youBy
Co brakuje:
moze jeszcze jakis SW
*/
/*
Zrobione:
ESt3, ESt3AL2, ESt4R, LSt, FV4a, FD1, EP2, prosty westinghouse
duzo wersji żeliwa
KE
Tarcze od 152A
Magnetyki (implementacja w mover.pas)
Matrosow 394
H14K1 (zasadniczy), H1405 (pomocniczy), St113 (ep)
Knorr/West EP - żeby był
*/

#pragma once

#include "friction.h" // Pascal unit

/// <summary>Number of positions of the local (auxiliary/manual) brake handle.</summary>
static int const LocalBrakePosNo = 10; /*ilosc nastaw hamulca recznego lub pomocniczego*/
/// <summary>Maximum number of positions of the main (train) brake handle.</summary>
static int const MainBrakeMaxPos = 10; /*max. ilosc nastaw hamulca zasadniczego*/

/*nastawy hamulca*/
/// <summary>Brake delay setting flag: G (goods/freight slow).</summary>
static int const bdelay_G = 1; // G
/// <summary>Brake delay setting flag: P (passenger fast).</summary>
static int const bdelay_P = 2; // P
/// <summary>Brake delay setting flag: R (rapid - high braking force).</summary>
static int const bdelay_R = 4; // R
/// <summary>Brake delay setting flag: Mg (magnetic rail brake enabled).</summary>
static int const bdelay_M = 8; // Mg

/*stan hamulca*/
/// <summary>Brake state flag: brake released (no action).</summary>
static int const b_off = 0; // luzowanie
/// <summary>Brake state flag: hold (lap) - keep current cylinder pressure.</summary>
static int const b_hld = 1; // trzymanie
/// <summary>Brake state flag: applying - filling brake cylinder.</summary>
static int const b_on = 2; // napelnianie
/// <summary>Brake state flag: replenishing the auxiliary reservoir.</summary>
static int const b_rfl = 4; // uzupelnianie
/// <summary>Brake state flag: releaser engaged (vents the control reservoir to release brakes).</summary>
static int const b_rls = 8; // odluzniacz
/// <summary>Brake state flag: electro-pneumatic action active.</summary>
static int const b_ep = 16; // elektropneumatyczny
/// <summary>Brake state flag: anti-slip protection holding the brake (preventing further filling).</summary>
static int const b_asb = 32; // przeciwposlizg-wstrzymanie
/// <summary>Brake state flag: anti-slip protection releasing the brake.</summary>
static int const b_asb_unbrake = 64; // przeciwposlizg-luzowanie
/// <summary>Brake state flag: brake disabled / damaged.</summary>
static int const b_dmg = 128; // wylaczony z dzialania

/*uszkodzenia hamulca*/
/// <summary>Brake damage flag: faulty filling phase.</summary>
static int const df_on = 1; // napelnianie
/// <summary>Brake damage flag: faulty release phase.</summary>
static int const df_off = 2; // luzowanie
/// <summary>Brake damage flag: leak in auxiliary reservoir (ZP).</summary>
static int const df_br = 4; // wyplyw z ZP
/// <summary>Brake damage flag: leak in valve pre-chamber.</summary>
static int const df_vv = 8; // wyplyw z komory wstepnej
/// <summary>Brake damage flag: leak in brake cylinder.</summary>
static int const df_bc = 16; // wyplyw z silownika
/// <summary>Brake damage flag: leak in control reservoir (ZS).</summary>
static int const df_cv = 32; // wyplyw z ZS
/// <summary>Brake damage flag: stuck at low load step.</summary>
static int const df_PP = 64; // zawsze niski stopien
/// <summary>Brake damage flag: stuck at high load step.</summary>
static int const df_RR = 128; // zawsze wysoki stopien

/*indeksy dzwiekow FV4a*/
/// <summary>FV4a sound index: braking flow.</summary>
static int const s_fv4a_b = 0; // hamowanie
/// <summary>FV4a sound index: release flow.</summary>
static int const s_fv4a_u = 1; // luzowanie
/// <summary>FV4a sound index: emergency braking flow.</summary>
static int const s_fv4a_e = 2; // hamowanie nagle
/// <summary>FV4a sound index: control wave outflow.</summary>
static int const s_fv4a_x = 3; // wyplyw sterujacego fala
/// <summary>FV4a sound index: timing chamber outflow.</summary>
static int const s_fv4a_t = 4; // wyplyw z czasowego

/*pary cierne*/
/// <summary>Friction pair: P10 (default cast iron).</summary>
static int const bp_P10 = 0;
/// <summary>Friction pair: P10 phosphoric cast iron with Bg block.</summary>
static int const bp_P10Bg = 2; // żeliwo fosforowe P10
/// <summary>Friction pair: P10 phosphoric cast iron with Bgu block.</summary>
static int const bp_P10Bgu = 1;
/// <summary>Friction pair: low-friction composite (b.n.t.) with Bg block.</summary>
static int const bp_LLBg = 4; // komp. b.n.t.
/// <summary>Friction pair: low-friction composite (b.n.t.) with Bgu block.</summary>
static int const bp_LLBgu = 3;
/// <summary>Friction pair: low-friction composite (n.t.) with Bg block.</summary>
static int const bp_LBg = 6; // komp. n.t.
/// <summary>Friction pair: low-friction composite (n.t.) with Bgu block.</summary>
static int const bp_LBgu = 5;
/// <summary>Friction pair: high-friction composite (w.t.) with Bg block.</summary>
static int const bp_KBg = 8; // komp. w.t.
/// <summary>Friction pair: high-friction composite (w.t.) with Bgu block.</summary>
static int const bp_KBgu = 7;
/// <summary>Friction pair: disc brake type 1.</summary>
static int const bp_D1 = 9; // tarcze
/// <summary>Friction pair: disc brake type 2.</summary>
static int const bp_D2 = 10;
/// <summary>Friction pair: Frenoplast FR513 composite.</summary>
static int const bp_FR513 = 11; // Frenoplast FR513
/// <summary>Friction pair: Cosid composite.</summary>
static int const bp_Cosid = 12; // jakistam kompozyt :D
/// <summary>Friction pair: PKP cast iron with Bg block.</summary>
static int const bp_PKPBg = 13; // żeliwo PKP
/// <summary>Friction pair: PKP cast iron with Bgu block.</summary>
static int const bp_PKPBgu = 14;
/// <summary>Friction pair flag: magnetic rail brake (added bitwise to other types).</summary>
static int const bp_MHS = 128; // magnetyczny hamulec szynowy
/// <summary>Friction pair: P10y phosphoric cast iron with Bg block.</summary>
static int const bp_P10yBg = 15; // żeliwo fosforowe P10
/// <summary>Friction pair: P10y phosphoric cast iron with Bgu block.</summary>
static int const bp_P10yBgu = 16;
/// <summary>Friction pair: Frenoplast FR510 composite.</summary>
static int const bp_FR510 = 17; // Frenoplast FR510

/// <summary>Sound flag: accelerator (rapid pressure drop in valve pre-chamber).</summary>
static int const sf_Acc = 1; // przyspieszacz
/// <summary>Sound flag: brake transmission/load relay actuating.</summary>
static int const sf_BR = 2; // przekladnia
/// <summary>Sound flag: brake cylinder filling.</summary>
static int const sf_CylB = 4; // cylinder - napelnianie
/// <summary>Sound flag: brake cylinder venting.</summary>
static int const sf_CylU = 8; // cylinder - oproznianie
/// <summary>Sound flag: releaser actuated.</summary>
static int const sf_rel = 16; // odluzniacz
/// <summary>Sound flag: electro-pneumatic valves switching.</summary>
static int const sf_ep = 32; // zawory ep

/// <summary>Brake handle position index: minimum position.</summary>
static int const bh_MIN = 0; // minimalna pozycja
/// <summary>Brake handle position index: maximum position.</summary>
static int const bh_MAX = 1; // maksymalna pozycja
/// <summary>Brake handle position index: filling stroke (high-pressure overcharge); if not present, equivalent to running.</summary>
static int const bh_FS = 2; // napelnianie uderzeniowe //jesli nie ma, to jazda
/// <summary>Brake handle position index: running (driving) position.</summary>
static int const bh_RP = 3; // jazda
/// <summary>Brake handle position index: cut-off (double traction / neutral).</summary>
static int const bh_NP = 4; // odciecie - podwojna trakcja
/// <summary>Brake handle position index: minimum braking step (lap or first step).</summary>
static int const bh_MB = 5; // odciecie - utrzymanie stopnia hamowania/pierwszy 1 stopien hamowania
/// <summary>Brake handle position index: full service brake.</summary>
static int const bh_FB = 6; // pelne
/// <summary>Brake handle position index: emergency brake.</summary>
static int const bh_EB = 7; // nagle
/// <summary>Brake handle position index: EP release (full release for angular EP).</summary>
static int const bh_EPR = 8; // ep - luzowanie  //pelny luz dla ep kątowego
/// <summary>Brake handle position index: EP hold/lap; if equal to release, button-only EP control.</summary>
static int const bh_EPN = 9; // ep - utrzymanie //jesli rowne luzowaniu, wtedy sterowanie przyciskiem
/// <summary>Brake handle position index: EP brake (full braking for angular EP).</summary>
static int const bh_EPB = 10; // ep - hamowanie  //pelne hamowanie dla ep kątowego

/// <summary>Diameter of the brake pipe (1") used in flow calculations.</summary>
static double const SpgD = 0.7917;
/// <summary>Cross-section of a 1" brake pipe expressed in l/m (used to keep volumes in liters and lengths in meters).</summary>
static double const SpO = 0.5067; // przekroj przewodu 1" w l/m
                                  // wyj: jednostka dosyc dziwna, ale wszystkie obliczenia
                                  // i pojemnosci sa podane w litrach (rozsadne wielkosci)
                                  // zas dlugosc pojazdow jest podana w metrach
                                  // a predkosc przeplywu w m/s                           //3.5
                                  // 7//1.5
//   BPT: array[-2..6] of array [0..1] of real= ((0, 5.0), (14, 5.4), (9, 5.0), (6, 4.6), (9, 4.5), (9, 4.0), (9, 3.5), (9, 2.8), (34, 2.8));
//   BPT: array[-2..6] of array [0..1] of real= ((0, 5.0), (7, 5.0), (2.0, 5.0), (4.5, 4.6), (4.5, 4.2), (4.5, 3.8), (4.5, 3.4), (4.5, 2.8), (8, 2.8));
/// <summary>
/// Brake handle position table for FV4a-family valves (range -2..6).
/// Each row holds {flow speed, target brake pipe pressure [bar]} for the matching position index (offset by +2).
/// </summary>
static double const BPT[9][2] = {{0, 5.0}, {7, 5.0}, {2.0, 5.0}, {4.5, 4.6}, {4.5, 4.2}, {4.5, 3.8}, {4.5, 3.4}, {4.5, 2.8}, {8, 2.8}};
/// <summary>
/// Brake handle position table for the Matrosow 394 valve (range -1..5).
/// Each row holds {flow speed, target brake pipe pressure [bar]} for the matching position index (offset by +1).
/// </summary>
static double const BPT_394[7][2] = {{13, 10.0}, {5, 5.0}, {0, -1}, {5, -1}, {5, 0.0}, {5, 0.0}, {18, 0.0}};
// double *BPT = zero_based_BPT[2]; //tablica pozycji hamulca dla zakresu -2..6
// double *BPT_394 = zero_based_BPT_394[1]; //tablica pozycji hamulca dla zakresu -1..5
//    BPT: array[-2..6] of array [0..1] of real= ((0, 5.0), (12, 5.4), (9, 5.0), (9, 4.6), (9, 4.2), (9, 3.8), (9, 3.4), (9, 2.8), (34, 2.8));
//       BPT: array[-2..6] of array [0..1] of real= ((0, 0),(0, 0),(0, 0),(0, 0),(0, 0),(0, 0),(0, 0),(0, 0),(0, 0));
//  static double const pi = 3.141592653589793; //definicja w mctools

/// <summary>
/// Bit flags for the universal brake button — actions that can be triggered
/// from a single hardware button on the driver's stand. Combined bitwise.
/// </summary>
enum TUniversalBrake // możliwe działania uniwersalnego przycisku hamulca
{ // kolejne flagi
  /// <summary>Releaser (ZR) — vents control reservoir to release brakes on this vehicle.</summary>
	ub_Release = 0x01, // odluźniacz - ZR
	                   /// <summary>Brake pipe unlock / safety brake bridging (vehicle-level).</summary>
	ub_UnlockPipe = 0x02, // odblok PG / mostkowanie hamulca bezpieczeństwa - POJAZD
	                      /// <summary>High-pressure pulse (ZM) — overcharge stroke.</summary>
	ub_HighPressure = 0x04, // impuls wysokiego ciśnienia - ZM
	                        /// <summary>Assimilation / controlled overcharge button (ZM).</summary>
	ub_Overload = 0x08, // przycisk asymilacji / kontrolowanego przeładowania - ZM
	                    /// <summary>Anti-slip light braking button (ZR).</summary>
	ub_AntiSlipBrake = 0x10, // przycisk przyhamowania przeciwposlizgowego - ZR
	                         /// <summary>Reserved sentinel — last bit flag (highest bit).</summary>
	ub_Ostatni = 0x80000000 // ostatnia flaga bitowa
};

/// <summary>
/// Single pneumatic reservoir. Tracks capacity (Cap), current volume (Vol)
/// and the pending flow (dVol) accumulated during a simulation step.
/// </summary>
class TReservoir
{

  protected:
	/// <summary>Reservoir capacity in liters.</summary>
	double Cap{1.0};
	/// <summary>Current air volume contained, scaled by capacity (Vol/Cap == pressure).</summary>
	double Vol{0.0};
	/// <summary>Accumulated flow (in/out) for the current step; applied by <see cref="Act"/>.</summary>
	double dVol{0.0};

  public:
	/// <summary>
	/// Sets reservoir capacity.
	/// </summary>
	/// <param name="Capacity">Capacity in liters.</param>
	void CreateCap(double Capacity);
	/// <summary>
	/// Initialises the reservoir to a given absolute pressure (clears pending flow).
	/// </summary>
	/// <param name="Press">Pressure in bar (atm).</param>
	void CreatePress(double Press);
	/// <summary>
	/// Returns absolute (atmospheric) pressure inside the reservoir.
	/// </summary>
	/// <returns>Pressure value (0.1 * Vol / Cap).</returns>
	virtual double pa();
	/// <summary>
	/// Returns gauge pressure inside the reservoir.
	/// </summary>
	/// <returns>Pressure value (Vol / Cap), in bar.</returns>
	virtual double P();
	/// <summary>
	/// Accumulates a flow into the reservoir for the current step (positive = in, negative = out).
	/// </summary>
	/// <param name="dv">Volume change to add to dVol.</param>
	void Flow(double dv);
	/// <summary>
	/// Applies the accumulated flow (dVol) to the volume and resets dVol.
	/// Volume is clamped to be non-negative.
	/// </summary>
	void Act();

	/// <summary>Default constructor — creates a 1 L reservoir at zero pressure.</summary>
	TReservoir() = default;
};

/// <summary>Pointer typedef for a reservoir instance.</summary>
typedef TReservoir *PReservoir;

/// <summary>
/// Brake cylinder reservoir — overrides pressure functions to model the
/// non-linear behaviour of a piston cylinder (initial dead volume, working stroke,
/// fully extended piston).
/// </summary>
class TBrakeCyl : public TReservoir
{

  public:
	/// <summary>
	/// Returns absolute pressure inside the brake cylinder (P() * 0.1).
	/// </summary>
	/// <returns>Absolute cylinder pressure.</returns>
	virtual double pa() /*override*/;
	/// <summary>
	/// Returns gauge pressure inside the brake cylinder, modelling the
	/// piston-stroke pressure curve (dead volume, linear stroke, fully extended).
	/// </summary>
	/// <returns>Cylinder pressure in bar.</returns>
	virtual double P() /*override*/;
	/// <summary>Default constructor.</summary>
	TBrakeCyl() : TReservoir() {};
};

/// <summary>
/// Base class for the complete pneumatic brake system of a single vehicle.
/// Holds the brake cylinder, auxiliary reservoir, valve pre-chamber and the
/// generic logic shared by every distributor type. Concrete distributor
/// behaviour is implemented in derived classes (TWest, TESt, TLSt, TKE, ...).
/// </summary>
class TBrake
{

  protected:
	/// <summary>Brake cylinder (silownik) — actuator that pushes the brake blocks.</summary>
	std::shared_ptr<TReservoir> BrakeCyl; // silownik
	/// <summary>Auxiliary reservoir (ZP) — local source of compressed air for the cylinder.</summary>
	std::shared_ptr<TReservoir> BrakeRes; // ZP
	/// <summary>Valve pre-chamber (komora wstepna) — small volume connected to the brake pipe / distributor.</summary>
	std::shared_ptr<TReservoir> ValveRes; // komora wstepna
	/// <summary>Number of brake cylinders on the vehicle.</summary>
	int BCN = 0; // ilosc silownikow
	/// <summary>Brake transmission ratio (mechanical leverage between cylinder and shoes).</summary>
	double BCM = 0.0; // przekladnia hamulcowa
	/// <summary>Combined cross-section area of all brake cylinders.</summary>
	double BCA = 0.0; // laczny przekroj silownikow
	/// <summary>Bitfield of available brake delay positions (combination of bdelay_* flags).</summary>
	int BrakeDelays = 0; // dostepne opoznienia
	/// <summary>Currently selected brake delay (one of bdelay_* values).</summary>
	int BrakeDelayFlag = 0; // aktualna nastawa
	/// <summary>Friction material model used for the shoes/discs.</summary>
	std::shared_ptr<TFricMat> FM; // material cierny
	/// <summary>Maximum brake cylinder pressure [bar] in service braking.</summary>
	double MaxBP = 0.0; // najwyzsze cisnienie
	/// <summary>Number of braked axles on the vehicle.</summary>
	int BA = 0; // osie hamowane
	/// <summary>Number of brake blocks per axle.</summary>
	int NBpA = 0; // klocki na os
	/// <summary>Squared size of the auxiliary reservoir relative to a 14" reference cylinder.</summary>
	double SizeBR = 0.0; // rozmiar^2 ZP (w stosunku do 14")
	/// <summary>Squared size of the brake cylinder relative to a 14" reference cylinder.</summary>
	double SizeBC = 0.0; // rozmiar^2 CH (w stosunku do 14")
	/// <summary>True when the double check valve (auxiliary brake -> cylinder) is engaged.</summary>
	bool DCV = false; // podwojny zawor zwrotny
	/// <summary>Anti-slip brake target pressure.</summary>
	double ASBP = 0.0; // cisnienie hamulca pp
	/// <summary>Velocity threshold above which the rapid braking step is enabled.</summary>
	double RV = 0.0; // rapid activation vehicle velocity threshold

	/// <summary>Bitfield with the currently active universal-button actions (see TUniversalBrake).</summary>
	int UniversalFlag = 0; // flaga wcisnietych przyciskow uniwersalnych
	/// <summary>Current brake state (combination of b_off / b_hld / b_on / b_rls / ... flags).</summary>
	int BrakeStatus{b_off}; // flaga stanu
	/// <summary>Bitfield of pending sound events (sf_* flags). Cleared after each read.</summary>
	int SoundFlag = 0;

  public:
	/// <summary>
	/// Builds a brake unit and selects the friction material.
	/// </summary>
	/// <param name="i_mbp">Maximum brake cylinder pressure [bar].</param>
	/// <param name="i_bcr">Brake cylinder radius [m].</param>
	/// <param name="i_bcd">Brake cylinder working stroke [m].</param>
	/// <param name="i_brc">Auxiliary reservoir capacity [l].</param>
	/// <param name="i_bcn">Number of brake cylinders.</param>
	/// <param name="i_BD">Available brake delay positions (bitwise of bdelay_* flags).</param>
	/// <param name="i_mat">Friction material id (one of bp_* constants, optionally OR'ed with bp_MHS).</param>
	/// <param name="i_ba">Number of braked axles.</param>
	/// <param name="i_nbpa">Number of blocks per axle.</param>
	TBrake(double i_mbp, double i_bcr, double i_bcd, double i_brc, int i_bcn, int i_BD, int i_mat, int i_ba, int i_nbpa);
	// maksymalne cisnienie, promien, skok roboczy, pojemnosc ZP, ilosc cylindrow, opoznienia hamulca, material klockow, osie hamowane, klocki na os;
	/// <summary>
	/// Initialises the brake to a given starting state (pressures and delay).
	/// </summary>
	/// <param name="PP">Brake pipe pressure [bar].</param>
	/// <param name="HPP">High (control) pressure [bar].</param>
	/// <param name="LPP">Low pressure threshold [bar].</param>
	/// <param name="BP">Initial brake cylinder pressure [bar].</param>
	/// <param name="BDF">Initial brake delay flag.</param>
	virtual void Init(double PP, double HPP, double LPP, double BP, int BDF); // inicjalizacja hamulca

	/// <summary>
	/// Returns the current friction coefficient between blocks and the wheel/disc.
	/// Delegates to the friction material model.
	/// </summary>
	/// <param name="Vel">Current vehicle velocity [m/s or km/h depending on FM].</param>
	/// <param name="N">Normal force on the block [N].</param>
	/// <returns>Friction coefficient.</returns>
	double GetFC(double Vel, double N); // wspolczynnik tarcia - hamulec wie lepiej
	/// <summary>
	/// Advances the brake distributor for one simulation step and returns the net
	/// volume exchanged with the brake pipe (positive = drawn from the brake pipe).
	/// </summary>
	/// <param name="PP">Brake pipe pressure [bar] at the connection point.</param>
	/// <param name="dt">Time step [s].</param>
	/// <param name="Vel">Vehicle velocity [m/s].</param>
	/// <returns>Volume exchanged between the valve pre-chamber and the brake pipe.</returns>
	virtual double GetPF(double PP, double dt, double Vel); // przeplyw miedzy komora wstepna i PG
	/// <summary>
	/// Returns the piston force produced by the brake cylinder pressure.
	/// </summary>
	/// <returns>Force in arbitrary engine units (BCA * 100 * P).</returns>
	double GetBCF(); // sila tlokowa z tloka
	/// <summary>
	/// Computes the airflow drawn from the high-pressure (8 bar / main) line for one step.
	/// </summary>
	/// <param name="HP">High-pressure source pressure [bar].</param>
	/// <param name="dt">Time step [s].</param>
	/// <returns>Net flow from the high-pressure line.</returns>
	virtual double GetHPFlow(double HP, double dt); // przeplyw - 8 bar
	/// <summary>Returns brake cylinder gauge pressure [bar].</summary>
	double GetBCP(); // cisnienie cylindrow hamulcowych
	/// <summary>
	/// Returns brake cylinder pressure originating only from the pneumatic
	/// (main) brake — used to drive the ED (electrodynamic) brake e.g. in EP09.
	/// </summary>
	virtual double GetEDBCP(); // cisnienie tylko z hamulca zasadniczego, uzywane do hamulca ED w EP09
	/// <summary>Returns auxiliary reservoir (ZP) pressure [bar].</summary>
	double GetBRP(); // cisnienie zbiornika pomocniczego
	/// <summary>Returns valve pre-chamber pressure [bar].</summary>
	double GetVRP(); // cisnienie komory wstepnej rozdzielacza
	/// <summary>Returns control reservoir (ZS) pressure [bar]; defaults to the auxiliary reservoir for valves without a dedicated ZS.</summary>
	virtual double GetCRP(); // cisnienie zbiornika sterujacego
	/// <summary>
	/// Sets the brake delay (G/P/R/M) selector if requested mode is supported.
	/// </summary>
	/// <param name="nBDF">Requested delay flag (bdelay_*).</param>
	/// <returns>True if accepted (mode supported and changed); false otherwise.</returns>
	bool SetBDF(int nBDF); // nastawiacz GPRM
	/// <summary>
	/// Engages or disengages the releaser (odluzniacz), updating the brake state flags.
	/// </summary>
	/// <param name="state">1 to engage releaser, 0 to disengage.</param>
	void Releaser(int state); // odluzniacz
	/// <summary>Returns true if the releaser is currently engaged.</summary>
	bool Releaser() const;
	/// <summary>
	/// Sets the electro-pneumatic state (EP) — strength of the EP brake action.
	/// Default no-op; overridden by EP-capable distributors.
	/// </summary>
	/// <param name="nEPS">EP intensity (typically -1..1).</param>
	virtual void SetEPS(double nEPS); // hamulec EP
	/// <summary>Sets the rapid step ratio. Default no-op; overridden where supported.</summary>
	/// <param name="RMR">Rapid ratio.</param>
	virtual void SetRM(double const RMR) {}; // ustalenie przelozenia rapida
	/// <summary>Sets the velocity threshold for the rapid step.</summary>
	/// <param name="RVR">Velocity threshold (same unit as Vel passed to GetPF).</param>
	virtual void SetRV(double const RVR)
	{
		RV = RVR;
	}; // ustalenie przelozenia rapida
	/// <summary>
	/// Sets the load-weighing parameters (empty/loaded mass, empty-mass cylinder pressure).
	/// Default no-op; overridden where load-weighing is supported.
	/// </summary>
	/// <param name="TM">Tare (empty) mass.</param>
	/// <param name="LM">Loaded mass.</param>
	/// <param name="TBP">Brake cylinder pressure for the tare mass.</param>
	virtual void SetLP(double const TM, double const LM, double const TBP) {}; // parametry przystawki wazacej
	/// <summary>Sets the auxiliary (local) brake target pressure.</summary>
	/// <param name="P">Local brake pressure [bar].</param>
	virtual void SetLBP(double const P) {}; // cisnienie z hamulca pomocniczego
	/// <summary>Updates the load-weighing pressure coefficient based on current vehicle mass.</summary>
	/// <param name="mass">Current vehicle mass.</param>
	virtual void PLC(double const mass) {}; // wspolczynnik cisnienia przystawki wazacej
	/// <summary>
	/// Engages the anti-slip brake function (set hold and/or release flags).
	/// </summary>
	/// <param name="state">Two-bit value: bit1 = hold (b_asb), bit0 = release (b_asb_unbrake).</param>
	void ASB(int state); // hamulec przeciwposlizgowy
	/// <summary>Returns the raw BrakeStatus flags (for sound/visual cues).</summary>
	int GetStatus(); // flaga statusu, moze sie przydac do odglosow
	/// <summary>Sets the anti-slip target pressure.</summary>
	/// <param name="Press">Pressure [bar].</param>
	void SetASBP(double Press); // ustalenie cisnienia pp
	/// <summary>Vents all reservoirs to zero pressure (used when the vehicle is decoupled or reset).</summary>
	virtual void ForceEmptiness();
	/// <summary>
	/// Removes a specified relative amount of air from the reservoirs to simulate leaks.
	/// </summary>
	/// <param name="Amount">Fraction of pressure to bleed (0..1) per call.</param>
	virtual void ForceLeak(double Amount);
	/// <summary>Returns and clears the accumulated SoundFlag bitfield.</summary>
	int GetSoundFlag();
	/// <summary>Returns the current brake status flags.</summary>
	int GetBrakeStatus() const
	{
		return BrakeStatus;
	}
	/// <summary>Overwrites the brake status flags.</summary>
	/// <param name="Status">New BrakeStatus value.</param>
	void SetBrakeStatus(int const Status)
	{
		BrakeStatus = Status;
	}
	/// <summary>
	/// Sets the ED (electrodynamic) brake state, used to release the pneumatic
	/// brake when ED braking is sufficient. Default no-op; overridden where supported.
	/// </summary>
	/// <param name="EDstate">ED brake intensity (0..1).</param>
	virtual void SetED(double const EDstate) {}; // stan hamulca ED do luzowania
	/// <summary>Sets the universal-button flags (see TUniversalBrake).</summary>
	/// <param name="flag">Combined ub_* flags.</param>
	virtual void SetUniversalFlag(int flag)
	{
		UniversalFlag = flag;
	} // przycisk uniwersalny
};

/// <summary>
/// Simple Westinghouse triple-valve distributor with optional EP brake and
/// load-weighing equipment. The auxiliary brake (cab handle) feeds straight
/// into the cylinder via the double check valve (DCV).
/// </summary>
class TWest : public TBrake
{

  private:
	/// <summary>Auxiliary (local) brake pressure [bar].</summary>
	double LBP = 0.0; // cisnienie hamulca pomocniczego
	/// <summary>High-pressure flow drawn during the last step (for the main reservoir bookkeeping).</summary>
	double dVP = 0.0; // pobor powietrza wysokiego cisnienia
	/// <summary>Current EP brake intensity (-1..1).</summary>
	double EPS = 0.0; // stan elektropneumatyka
	/// <summary>Tare (empty) vehicle mass.</summary>
	double TareM = 0.0; // masa proznego
	/// <summary>Loaded vehicle mass.</summary>
	double LoadM = 0.0; // i pelnego
	/// <summary>Cylinder pressure for tare mass.</summary>
	double TareBP = 0.0; // cisnienie dla proznego
	/// <summary>Computed load-weighing coefficient (1.0 for fully loaded).</summary>
	double LoadC = 0.0; // wspolczynnik przystawki wazacej

  public:
	/// <summary>Initialises pressures: PP into ValveRes, BP into BrakeCyl, average of PP/HPP into BrakeRes.</summary>
	/// <param name="PP">Brake pipe pressure.</param>
	/// <param name="HPP">High pressure.</param>
	/// <param name="LPP">Low pressure.</param>
	/// <param name="BP">Initial cylinder pressure.</param>
	/// <param name="BDF">Initial brake delay flag.</param>
	void Init(double PP, double HPP, double LPP, double BP, int BDF) /*override*/;
	/// <summary>Sets the auxiliary brake target pressure and engages the DCV when above cylinder.</summary>
	/// <param name="P">Pressure [bar].</param>
	void SetLBP(double P); // cisnienie z hamulca pomocniczego
	/// <summary>One-step distributor advance (Westinghouse logic).</summary>
	/// <returns>Net flow exchanged with the brake pipe.</returns>
	double GetPF(double PP, double dt, double Vel) /*override*/; // przeplyw miedzy komora wstepna i PG
	/// <summary>Returns the high-pressure flow drawn during the last GetPF step.</summary>
	double GetHPFlow(double HP, double dt) /*override*/;
	/// <summary>Recomputes the load-weighing pressure coefficient for the current mass.</summary>
	/// <param name="mass">Vehicle mass.</param>
	void PLC(double mass); // wspolczynnik cisnienia przystawki wazacej
	/// <summary>Sets the EP brake state and toggles the DCV / latches LBP from cylinder pressure on release.</summary>
	/// <param name="nEPS">New EP intensity.</param>
	void SetEPS(double nEPS) /*override*/; // stan hamulca EP
	/// <summary>Stores the load-weighing parameters (TareM, LoadM, TareBP).</summary>
	void SetLP(double TM, double LM, double TBP); // parametry przystawki wazacej

	/// <summary>Constructs the distributor by forwarding all parameters to TBrake.</summary>
	TWest(double i_mbp, double i_bcr, double i_bcd, double i_brc, int i_bcn, int i_BD, int i_mat, int i_ba, int i_nbpa) : TBrake(i_mbp, i_bcr, i_bcd, i_brc, i_bcn, i_BD, i_mat, i_ba, i_nbpa) {}
};

/// <summary>
/// Oerlikon ESt distributor base. Adds a control reservoir (ZS) and the
/// classical "main slide-valve" logic (CheckState/CheckReleaser/CVs/BVs).
/// Concrete variants (ESt3, ESt3AL2, ESt4R, LSt, EStED, EStEP1/2) refine GetPF.
/// </summary>
class TESt : public TBrake
{

  private:
  protected:
	/// <summary>Control reservoir (ZS) — long-term reference pressure.</summary>
	std::shared_ptr<TReservoir> CntrlRes; // zbiornik sterujący
	/// <summary>Brake-pipe to brake-cylinder transmission ratio (BVM = MaxBP / (HPP-LPP)).</summary>
	double BVM = 0.0; // przelozenie PG-CH

  public:
	/// <summary>Initialises the ESt distributor; sizes the control reservoir (15 l) and computes BVM.</summary>
	void Init(double PP, double HPP, double LPP, double BP, int BDF) /*override*/;
	/// <summary>One-step distributor advance for the ESt baseline.</summary>
	/// <returns>Net flow exchanged with the brake pipe.</returns>
	double GetPF(double PP, double dt, double Vel) /*override*/; // przeplyw miedzy komora wstepna i PG
	/// <summary>Sets ESt-specific characteristic parameters (placeholder; used by some variants).</summary>
	/// <param name="i_crc">Characteristic value.</param>
	static void EStParams(double i_crc); // parametry charakterystyczne dla ESt
	/// <summary>Returns the control reservoir (ZS) pressure.</summary>
	double GetCRP() /*override*/;
	/// <summary>
	/// Updates BrakeStatus (b_on/b_hld) according to the relations between
	/// pre-chamber, cylinder and control reservoir pressures (the main slide valve).
	/// Triggers the accelerator (sf_Acc) at the start of braking.
	/// </summary>
	/// <param name="BCP">Brake cylinder (or impulse) pressure.</param>
	/// <param name="dV1">In/out accumulator for the brake pipe flow correction.</param>
	void CheckState(double BCP, double &dV1); // glowny przyrzad rozrzadczy
	/// <summary>Drives the releaser logic — bleeds the control reservoir while the releaser is engaged.</summary>
	/// <param name="dt">Time step [s].</param>
	void CheckReleaser(double dt); // odluzniacz
	/// <summary>
	/// Returns the effective opening factor of the ZS-filling slide valve as a
	/// function of cylinder pressure and current pre-chamber/reservoir state.
	/// </summary>
	/// <param name="BP">Cylinder pressure (or impulse-chamber pressure).</param>
	/// <returns>Dimensionless opening coefficient.</returns>
	double CVs(double BP); // napelniacz sterujacego
	/// <summary>
	/// Returns the effective opening factor of the auxiliary-reservoir filling
	/// slide valve (ZP &lt;-&gt; pre-chamber path).
	/// </summary>
	/// <param name="BCP">Brake cylinder pressure.</param>
	/// <returns>Dimensionless opening coefficient.</returns>
	double BVs(double BCP); // napelniacz pomocniczego
	/// <summary>Vents the valve, brake and control reservoirs to zero.</summary>
	void ForceEmptiness() /*override*/; // wymuszenie bycia pustym

	/// <summary>Constructs the ESt distributor and creates the control reservoir.</summary>
	TESt(double i_mbp, double i_bcr, double i_bcd, double i_brc, int i_bcn, int i_BD, int i_mat, int i_ba, int i_nbpa) : TBrake(i_mbp, i_bcr, i_bcd, i_brc, i_bcn, i_BD, i_mat, i_ba, i_nbpa)
	{
		CntrlRes = std::make_shared<TReservoir>();
	}
};

/// <summary>
/// ESt3 variant — adjusts the cylinder fill/release rates depending on the
/// G/P delay setting; otherwise keeps the ESt slide-valve logic.
/// </summary>
class TESt3 : public TESt
{

  private:
	// double CylFlowSpeed[2][2]; //zmienna nie uzywana

  public:
	/// <summary>One-step distributor advance for ESt3 (G/P-dependent fill/release curves).</summary>
	/// <returns>Net flow exchanged with the brake pipe.</returns>
	double GetPF(double PP, double dt, double Vel) /*override*/; // przeplyw miedzy komora wstepna i PG

	/// <summary>Constructs the ESt3 distributor.</summary>
	TESt3(double i_mbp, double i_bcr, double i_bcd, double i_brc, int i_bcn, int i_BD, int i_mat, int i_ba, int i_nbpa) : TESt(i_mbp, i_bcr, i_bcd, i_brc, i_bcn, i_BD, i_mat, i_ba, i_nbpa) {}
};

/// <summary>
/// ESt3 with AL2 load-weighing equipment. Adds an impulse chamber and a
/// load-pressure relay that scales the cylinder pressure between empty/loaded.
/// </summary>
class TESt3AL2 : public TESt3
{

  private:
	/// <summary>Impulse chamber (KI) — drives the load relay output to the brake cylinder.</summary>
	std::shared_ptr<TReservoir> ImplsRes; // komora impulsowa
	/// <summary>Tare (empty) vehicle mass.</summary>
	double TareM = 0.0; // masa proznego
	/// <summary>Loaded vehicle mass.</summary>
	double LoadM = 0.0; // i pelnego
	/// <summary>Cylinder pressure for tare mass.</summary>
	double TareBP = 0.0; // cisnienie dla proznego
	/// <summary>Computed load-weighing coefficient.</summary>
	double LoadC = 0.0;

  public:
	/// <summary>Initialises the impulse chamber on top of the ESt initialisation.</summary>
	void Init(double PP, double HPP, double LPP, double BP, int BDF) /*override*/;
	/// <summary>One-step distributor advance for ESt3/AL2 with load-weighing relay.</summary>
	double GetPF(double PP, double dt, double Vel) /*override*/; // przeplyw miedzy komora wstepna i PG
	/// <summary>Recomputes LoadC for the current vehicle mass.</summary>
	void PLC(double mass); // wspolczynnik cisnienia przystawki wazacej
	/// <summary>Stores the load-weighing parameters.</summary>
	void SetLP(double TM, double LM, double TBP); // parametry przystawki wazacej

	/// <summary>Constructs the distributor and creates the impulse chamber.</summary>
	TESt3AL2(double i_mbp, double i_bcr, double i_bcd, double i_brc, int i_bcn, int i_BD, int i_mat, int i_ba, int i_nbpa) : TESt3(i_mbp, i_bcr, i_bcd, i_brc, i_bcn, i_BD, i_mat, i_ba, i_nbpa)
	{
		ImplsRes = std::make_shared<TReservoir>();
	}
};

/// <summary>
/// ESt4 with rapid (R) step. Adds a separately controlled impulse chamber
/// and a velocity-driven rapid factor that boosts cylinder pressure above a
/// configurable speed threshold.
/// </summary>
class TESt4R : public TESt
{

  private:
	/// <summary>Hysteretic rapid-step latch (true while rapid is active).</summary>
	bool RapidStatus = false;

  protected:
	/// <summary>Impulse chamber (KI) — intermediate pressure feeding the load relay.</summary>
	std::shared_ptr<TReservoir> ImplsRes; // komora impulsowa
	/// <summary>Smoothed rapid-step coefficient (current effective ratio).</summary>
	double RapidTemp = 0.0; // aktualne, zmienne przelozenie

  public:
	/// <summary>Initialises the ESt4R; sizes the impulse chamber and selects the R delay.</summary>
	void Init(double PP, double HPP, double LPP, double BP, int BDF) /*override*/;
	/// <summary>One-step distributor advance for ESt4R (rapid step active above velocity threshold).</summary>
	double GetPF(double PP, double dt, double Vel) /*override*/; // przeplyw miedzy komora wstepna i PG

	/// <summary>Constructs the distributor and creates the impulse chamber.</summary>
	TESt4R(double i_mbp, double i_bcr, double i_bcd, double i_brc, int i_bcn, int i_BD, int i_mat, int i_ba, int i_nbpa) : TESt(i_mbp, i_bcr, i_bcd, i_brc, i_bcn, i_BD, i_mat, i_ba, i_nbpa)
	{
		ImplsRes = std::make_shared<TReservoir>();
	}
};

/// <summary>
/// LSt distributor — locomotive variant of ESt4R with a double check valve
/// (auxiliary brake LBP), configurable rapid ratio and ED-brake release input.
/// </summary>
class TLSt : public TESt4R
{

  private:
	// double CylFlowSpeed[2][2]; // zmienna nie używana

  protected:
	/// <summary>Auxiliary (local) brake pressure feeding the DCV.</summary>
	double LBP = 0.0; // cisnienie hamulca pomocniczego
	/// <summary>Rapid step ratio (1 - RMR; see SetRM).</summary>
	double RM = 0.0; // przelozenie rapida
	/// <summary>ED brake state (0..1) — relaxes the pneumatic brake when ED is active.</summary>
	double EDFlag = 0.0; // luzowanie hamulca z powodu zalaczonego ED

  public:
	/// <summary>Initialises the LSt; resizes the valve and impulse reservoirs and presets pressures.</summary>
	void Init(double PP, double HPP, double LPP, double BP, int BDF) /*override*/;
	/// <summary>Sets the auxiliary brake target pressure for the DCV.</summary>
	void SetLBP(double P); // cisnienie z hamulca pomocniczego
	/// <summary>Sets the rapid step ratio (RM = 1 - RMR).</summary>
	/// <param name="RMR">Reduction ratio (0 disables rapid, &gt; 0 enables).</param>
	void SetRM(double RMR); // ustalenie przelozenia rapida
	/// <summary>One-step distributor advance for LSt (DCV + rapid + ED release).</summary>
	double GetPF(double PP, double dt, double Vel) /*override*/; // przeplyw miedzy komora wstepna i PG
	/// <summary>Computes the high-pressure inflow (replenishes the auxiliary reservoir from the main line).</summary>
	double GetHPFlow(double HP, double dt) /*override*/; // przeplyw - 8 bar
	/// <summary>Returns the brake-cylinder reference pressure used by the ED brake controller (CVP-BCP * BVM).</summary>
	virtual double GetEDBCP(); // cisnienie tylko z hamulca zasadniczego, uzywane do hamulca ED w EP09
	/// <summary>Sets the ED brake state used to relax the pneumatic brake.</summary>
	/// <param name="EDstate">ED intensity (0..1).</param>
	virtual void SetED(double EDstate); // stan hamulca ED do luzowania

	/// <summary>Constructs the LSt distributor.</summary>
	TLSt(double i_mbp, double i_bcr, double i_bcd, double i_brc, int i_bcn, int i_BD, int i_mat, int i_ba, int i_nbpa) : TESt4R(i_mbp, i_bcr, i_bcd, i_brc, i_bcn, i_BD, i_mat, i_ba, i_nbpa) {}
};

/// <summary>
/// EStED distributor (used in EP09): ESt4R with a separate load relay,
/// rapid-step control, intermediate reservoir (Miedzypoj) and a fully
/// nozzle-tuned ZP/ZS filling characteristic.
/// </summary>
class TEStED : public TLSt
{ // zawor z EP09 - Est4 z oddzielnym przekladnikiem, kontrola rapidu i takie tam

  private:
	/// <summary>Intermediate (virtual) reservoir used to refill ZP and ZS.</summary>
	std::shared_ptr<TReservoir> Miedzypoj; // pojemnosc posrednia (urojona) do napelniania ZP i ZS
	/// <summary>Effective cross-sections of the internal nozzles (squared diameters).</summary>
	double Nozzles[11]; // dysze
	/// <summary>Closing-valve memory latch.</summary>
	bool Zamykajacy = false; // pamiec zaworka zamykajacego
	/// <summary>Accelerator-block latch (prevents the accelerator from triggering twice).</summary>
	bool Przys_blok = false; // blokada przyspieszacza
	/// <summary>Tare (empty) vehicle mass.</summary>
	double TareM = 0.0; // masa proznego
	/// <summary>Loaded vehicle mass.</summary>
	double LoadM = 0.0; // i pelnego
	/// <summary>Cylinder pressure for tare mass.</summary>
	double TareBP = 0.0; // cisnienie dla proznego
	/// <summary>Computed load-weighing coefficient.</summary>
	double LoadC = 0.0;

  public:
	/// <summary>Initialises the EStED — sets up Miedzypoj, ImplsRes and the nozzle characteristics.</summary>
	void Init(double PP, double HPP, double LPP, double BP, int BDF) /*override*/;
	/// <summary>One-step distributor advance for EStED (full EP09 logic with intermediate reservoir).</summary>
	double GetPF(double PP, double dt, double Vel) /*override*/; // przeplyw miedzy komora wstepna i PG
	/// <summary>Returns ED-brake reference pressure (ImplsRes pressure scaled by load coefficient).</summary>
	double GetEDBCP() /*override*/; // cisnienie tylko z hamulca zasadniczego, uzywane do hamulca ED
	/// <summary>Recomputes LoadC for the current vehicle mass.</summary>
	void PLC(double mass); // wspolczynnik cisnienia przystawki wazacej
	/// <summary>Stores the load-weighing parameters.</summary>
	void SetLP(double TM, double LM, double TBP); // parametry przystawki wazacej

	/// <summary>Constructs the distributor and creates the intermediate reservoir.</summary>
	TEStED(double i_mbp, double i_bcr, double i_bcd, double i_brc, int i_bcn, int i_BD, int i_mat, int i_ba, int i_nbpa) : TLSt(i_mbp, i_bcr, i_bcd, i_brc, i_bcn, i_BD, i_mat, i_ba, i_nbpa)
	{
		Miedzypoj = std::make_shared<TReservoir>();
	}
};

/// <summary>
/// ESt with EP2 electro-pneumatic add-on (continuous EP brake, two-wire
/// driver). Uses the LSt base and supplies its own EP-flow calculation.
/// </summary>
class TEStEP2 : public TLSt
{

  protected:
	/// <summary>Tare (empty) vehicle mass.</summary>
	double TareM = 0.0; // masa proznego
	/// <summary>Loaded vehicle mass.</summary>
	double LoadM = 0.0; // masa pelnego
	/// <summary>Cylinder pressure for tare mass.</summary>
	double TareBP = 0.0; // cisnienie dla proznego
	/// <summary>Computed load-weighing coefficient.</summary>
	double LoadC = 0.0;
	/// <summary>EP intensity (-1..1).</summary>
	double EPS = 0.0;

  public:
	/// <summary>Initialises the EP2-equipped distributor (impulse chamber, P delay).</summary>
	void Init(double PP, double HPP, double LPP, double BP, int BDF) /*override*/; // inicjalizacja
	/// <summary>One-step distributor advance with EP2 EP brake logic.</summary>
	double GetPF(double PP, double dt, double Vel) /*override*/; // przeplyw miedzy komora wstepna i PG
	/// <summary>Recomputes LoadC for the current vehicle mass.</summary>
	void PLC(double mass); // wspolczynnik cisnienia przystawki wazacej
	/// <summary>Sets EP intensity; if EP is active and LBP &lt; cylinder pressure, latches LBP from cylinder.</summary>
	void SetEPS(double nEPS) /*override*/; // stan hamulca EP
	/// <summary>Stores the load-weighing parameters.</summary>
	void SetLP(double TM, double LM, double TBP); // parametry przystawki wazacej
	/// <summary>EP brake flow integration step. Override in EP1 for proportional control.</summary>
	/// <param name="dt">Time step [s].</param>
	void virtual EPCalc(double dt);

	/// <summary>Constructs the EP2 distributor.</summary>
	TEStEP2(double i_mbp, double i_bcr, double i_bcd, double i_brc, int i_bcn, int i_BD, int i_mat, int i_ba, int i_nbpa) : TLSt(i_mbp, i_bcr, i_bcd, i_brc, i_bcn, i_BD, i_mat, i_ba, i_nbpa) {}
};

/// <summary>
/// EP1 variant of the EP brake (continuous, proportional). Reuses EP2's
/// distributor pneumatics but replaces the EP-flow integrator with a
/// proportional model based on the fractional part of EPS.
/// </summary>
class TEStEP1 : public TEStEP2
{

  public:
	/// <summary>Proportional EP flow integration step (uses fractional part of EPS as the EP target).</summary>
	void EPCalc(double dt);
	/// <summary>Stores the EP intensity.</summary>
	/// <param name="nEPS">Target EP value (integer part = direction, fractional part = magnitude).</param>
	void SetEPS(double nEPS) override; // stan hamulca EP

	/// <summary>Constructs the EP1 distributor.</summary>
	TEStEP1(double i_mbp, double i_bcr, double i_bcd, double i_brc, int i_bcn, int i_BD, int i_mat, int i_ba, int i_nbpa) : TEStEP2(i_mbp, i_bcr, i_bcd, i_brc, i_bcn, i_BD, i_mat, i_ba, i_nbpa)
	{
	}
};

/// <summary>
/// DAKO CV1 distributor — Czech triple-valve with control reservoir and a
/// simpler slide-valve characteristic than ESt.
/// </summary>
class TCV1 : public TBrake
{

  private:
	/// <summary>Brake-pipe to brake-cylinder transmission ratio.</summary>
	double BVM = 0.0; // przelozenie PG-CH

  protected:
	/// <summary>Control reservoir (ZS).</summary>
	std::shared_ptr<TReservoir> CntrlRes; // zbiornik sterujący

  public:
	/// <summary>Initialises the CV1 distributor (sizes ZS, sets pressures, computes BVM).</summary>
	void Init(double PP, double HPP, double LPP, double BP, int BDF) /*override*/;
	/// <summary>One-step distributor advance for the CV1 baseline.</summary>
	double GetPF(double PP, double dt, double Vel) /*override*/; // przeplyw miedzy komora wstepna i PG
	/// <summary>Returns the control reservoir (ZS) pressure.</summary>
	double GetCRP() /*override*/;
	/// <summary>Updates BrakeStatus based on pre-chamber/cylinder/control reservoir relations and the releaser.</summary>
	/// <param name="BCP">Cylinder (or impulse) pressure.</param>
	/// <param name="dV1">In/out brake pipe flow correction.</param>
	void CheckState(double BCP, double &dV1);
	/// <summary>Returns the ZS-filling slide valve opening factor for the given cylinder pressure.</summary>
	static double CVs(double BP);
	/// <summary>Returns the ZP-filling slide valve opening factor for the given cylinder pressure.</summary>
	double BVs(double BCP);
	/// <summary>Vents valve, brake and control reservoirs to zero.</summary>
	void ForceEmptiness() /*override*/; // wymuszenie bycia pustym

	/// <summary>Constructs the CV1 distributor and creates the control reservoir.</summary>
	TCV1(double i_mbp, double i_bcr, double i_bcd, double i_brc, int i_bcn, int i_BD, int i_mat, int i_ba, int i_nbpa) : TBrake(i_mbp, i_bcr, i_bcd, i_brc, i_bcn, i_BD, i_mat, i_ba, i_nbpa)
	{
		CntrlRes = std::make_shared<TReservoir>();
	}
};

// class TCV1R : public TCV1

//{
// private:
//	TReservoir *ImplsRes;      //komora impulsowa
//	bool RapidStatus;

// public:
//	//        function GetPF(PP, dt, Vel: real): real; override;     //przeplyw miedzy komora wstepna i PG
//	//        procedure Init(PP, HPP, LPP, BP: real; BDF: int); override;

//	inline TCV1R(double i_mbp, double i_bcr, double i_bcd, double i_brc,
//		int i_bcn, int i_BD, int i_mat, int i_ba, int i_nbpa,
//		double PP, double HPP, double LPP, double BP, int BDF) : TCV1(i_mbp, i_bcr, i_bcd, i_brc, i_bcn
//			, i_BD, i_mat, i_ba, i_nbpa, PP, HPP, LPP, BP, BDF) { }
//};

/// <summary>
/// CV1-L-TR distributor — locomotive variant of CV1 with auxiliary (local)
/// brake input, impulse chamber and a high-pressure replenishing path.
/// </summary>
class TCV1L_TR : public TCV1
{

  private:
	/// <summary>Impulse chamber (KI) — drives the relay output to the brake cylinder.</summary>
	std::shared_ptr<TReservoir> ImplsRes; // komora impulsowa
	/// <summary>Auxiliary (local) brake pressure feeding the DCV.</summary>
	double LBP = 0.0; // cisnienie hamulca pomocniczego

  public:
	/// <summary>Initialises the CV1-L-TR (sizes the impulse chamber on top of CV1::Init).</summary>
	void Init(double PP, double HPP, double LPP, double BP, int BDF) /*override*/;
	/// <summary>One-step distributor advance for CV1-L-TR (impulse chamber + DCV).</summary>
	double GetPF(double PP, double dt, double Vel) /*override*/; // przeplyw miedzy komora wstepna i PG
	/// <summary>Sets the auxiliary brake target pressure for the DCV.</summary>
	void SetLBP(double P); // cisnienie z hamulca pomocniczego
	/// <summary>Computes the high-pressure (8 bar) inflow used to replenish the auxiliary reservoir.</summary>
	double GetHPFlow(double HP, double dt) /*override*/; // przeplyw - 8 bar

	/// <summary>Constructs the CV1-L-TR and creates the impulse chamber.</summary>
	TCV1L_TR(double i_mbp, double i_bcr, double i_bcd, double i_brc, int i_bcn, int i_BD, int i_mat, int i_ba, int i_nbpa) : TCV1(i_mbp, i_bcr, i_bcd, i_brc, i_bcn, i_BD, i_mat, i_ba, i_nbpa)
	{
		ImplsRes = std::make_shared<TReservoir>();
	}
};

/// <summary>
/// Knorr KE (Einheitsbauart) — universal distributor with control reservoir,
/// impulse chamber, optional auxiliary reservoir, rapid step driven by speed
/// and friction-pair type, and load-weighing equipment.
/// </summary>
class TKE : public TBrake
{ // Knorr Einheitsbauart — jeden do wszystkiego

  private:
	/// <summary>Impulse chamber (KI).</summary>
	std::shared_ptr<TReservoir> ImplsRes; // komora impulsowa
	/// <summary>Control reservoir (ZS).</summary>
	std::shared_ptr<TReservoir> CntrlRes; // zbiornik sterujący
	/// <summary>Secondary auxiliary reservoir (ZP2) — used in some variants for additional capacity.</summary>
	std::shared_ptr<TReservoir> Brak2Res; // zbiornik pomocniczy 2
	/// <summary>Hysteretic rapid-step latch.</summary>
	bool RapidStatus = false;
	/// <summary>Brake-pipe to brake-cylinder transmission ratio.</summary>
	double BVM = 0.0; // przelozenie PG-CH
	/// <summary>Tare (empty) vehicle mass.</summary>
	double TareM = 0.0; // masa proznego
	/// <summary>Loaded vehicle mass.</summary>
	double LoadM = 0.0; // masa pelnego
	/// <summary>Cylinder pressure for tare mass.</summary>
	double TareBP = 0.0; // cisnienie dla proznego
	/// <summary>Computed load-weighing coefficient.</summary>
	double LoadC = 0.0; // wspolczynnik zaladowania
	/// <summary>Rapid step coefficient (RM = 1 - RMR).</summary>
	double RM = 0.0; // przelozenie rapida
	/// <summary>Auxiliary (local) brake pressure feeding the DCV.</summary>
	double LBP = 0.0; // cisnienie hamulca pomocniczego

  public:
	/// <summary>Initialises the KE distributor (control / impulse / auxiliary reservoirs and BVM).</summary>
	void Init(double PP, double HPP, double LPP, double BP, int BDF) /*override*/;
	/// <summary>Sets the rapid step ratio (RM = 1 - RMR).</summary>
	void SetRM(double RMR); // ustalenie przelozenia rapida
	/// <summary>One-step distributor advance for the KE distributor.</summary>
	double GetPF(double PP, double dt, double Vel) /*override*/; // przeplyw miedzy komora wstepna i PG
	/// <summary>Computes the high-pressure (8 bar) inflow used to replenish the auxiliary reservoir.</summary>
	double GetHPFlow(double HP, double dt) /*override*/; // przeplyw - 8 bar
	/// <summary>Returns the control reservoir (ZS) pressure.</summary>
	double GetCRP() /*override*/;
	/// <summary>Updates BrakeStatus from cylinder/pre-chamber/control reservoir pressures (KE-specific thresholds).</summary>
	void CheckState(double BCP, double &dV1);
	/// <summary>Drives the releaser logic for KE — bleeds the control reservoir while engaged.</summary>
	void CheckReleaser(double dt); // odluzniacz
	/// <summary>ZS-filling slide valve opening factor for the given cylinder pressure.</summary>
	double CVs(double BP); // napelniacz sterujacego
	/// <summary>ZP-filling slide valve opening factor for the given cylinder pressure.</summary>
	double BVs(double BCP); // napelniacz pomocniczego
	/// <summary>Recomputes LoadC for the current vehicle mass.</summary>
	void PLC(double mass); // wspolczynnik cisnienia przystawki wazacej
	/// <summary>Stores the load-weighing parameters.</summary>
	void SetLP(double TM, double LM, double TBP); // parametry przystawki wazacej
	/// <summary>Sets the auxiliary brake target pressure for the DCV.</summary>
	void SetLBP(double P); // cisnienie z hamulca pomocniczego
	/// <summary>Vents valve, brake, control, impulse and secondary auxiliary reservoirs to zero.</summary>
	void ForceEmptiness() /*override*/; // wymuszenie bycia pustym

	/// <summary>Constructs the KE distributor and creates the control / impulse / secondary reservoirs.</summary>
	TKE(double i_mbp, double i_bcr, double i_bcd, double i_brc, int i_bcn, int i_BD, int i_mat, int i_ba, int i_nbpa) : TBrake(i_mbp, i_bcr, i_bcd, i_brc, i_bcn, i_BD, i_mat, i_ba, i_nbpa)
	{
		ImplsRes = std::make_shared<TReservoir>();
		CntrlRes = std::make_shared<TReservoir>();
		Brak2Res = std::make_shared<TReservoir>();
	}
};

/// <summary>
/// Base class for the driver's brake handle (cab valve / kran maszynisty).
/// Concrete handles (FV4a, FV4a/M, MHZ_*, M394, H14K1, St113, FVel6, ...)
/// implement <see cref="GetPF"/> to compute the brake pipe flow for a given
/// handle position and supply additional gauges (control, equalising, EP).
/// </summary>
class TDriverHandle
{

  protected:
	//        BCP: integer;
	/// <summary>True if the handle automatically overcharges in the high-pressure position (e.g. -1).</summary>
	bool AutoOvrld = false; // czy jest asymilacja automatyczna na pozycji -1
	/// <summary>True if the handle supports manual overcharge/assimilation via a dedicated button.</summary>
	bool ManualOvrld = false; // czy jest asymilacja reczna przyciskiem
	/// <summary>Whether the manual overcharge button is currently pressed.</summary>
	bool ManualOvrldActive = false; // czy jest wcisniety przycisk asymilacji
	/// <summary>Bitfield with the active universal-button actions (see TUniversalBrake).</summary>
	int UniversalFlag = 0; // flaga wcisnietych przyciskow uniwersalnych
	/// <summary>Number of the highest (last) handle position.</summary>
	int i_bcpno = 6;

  public:
	/// <summary>True if the handle has a "time" (delay/charge) chamber.</summary>
	bool Time = false;
	/// <summary>True if the handle has an EP-time chamber.</summary>
	bool TimeEP = false;
	/// <summary>Per-event sound flow magnitudes (indices defined by s_fv4a_*).</summary>
	double Sounds[5]; // wielkosci przeplywow dla dzwiekow

	/// <summary>
	/// Computes the brake pipe flow produced by the handle in one simulation step.
	/// </summary>
	/// <param name="i_bcp">Handle position (continuous; integer values map to detents).</param>
	/// <param name="PP">Brake pipe pressure [bar].</param>
	/// <param name="HP">High-pressure source [bar].</param>
	/// <param name="dt">Time step [s].</param>
	/// <param name="ep">EP / equalising input pressure [bar].</param>
	/// <returns>Volume change of the brake pipe (positive = into pipe).</returns>
	virtual double GetPF(double i_bcp, double PP, double HP, double dt, double ep);
	/// <summary>Initialises internal pressures from the supplied reference.</summary>
	/// <param name="Press">Initial pressure [bar].</param>
	virtual void Init(double Press);
	/// <summary>Returns the control-reservoir / equalising pressure displayed on the cab gauge.</summary>
	virtual double GetCP();
	/// <summary>Returns the EP brake intensity reported by the handle (0 by default).</summary>
	virtual double GetEP();
	/// <summary>Returns the regulator (reduction) pressure target.</summary>
	virtual double GetRP();
	/// <summary>Adjusts the pressure reductor offset (turning the cap of the reductor).</summary>
	/// <param name="nAdj">Pressure correction [bar].</param>
	virtual void SetReductor(double nAdj); // korekcja pozycji reduktora cisnienia
	/// <summary>Returns the requested sound channel magnitude (index 0..4 defined by s_fv4a_*).</summary>
	/// <param name="i">Sound index.</param>
	virtual double GetSound(int i); // pobranie glosnosci wybranego dzwieku
	/// <summary>
	/// Returns the position value (number of detents) for the requested function code.
	/// </summary>
	/// <param name="i">Function index (bh_* constants).</param>
	virtual double GetPos(int i); // pobranie numeru pozycji o zadanym kodzie (funkcji)
	/// <summary>Returns EP brake force at the given handle position.</summary>
	/// <param name="pos">Handle position.</param>
	virtual double GetEP(double pos); // pobranie sily hamulca ep
	/// <summary>
	/// Configures handle-specific behaviour (overcharge mode, filling stroke factor and overcharge dynamics).
	/// </summary>
	/// <param name="AO">Auto-overcharge enabled.</param>
	/// <param name="MO">Manual-overcharge enabled.</param>
	/// <param name="OverP">Unbrake over-pressure [bar].</param>
	/// <param name="OMP">Overload (assimilation) max pressure [bar].</param>
	/// <param name="OPD">Overload pressure decay rate [bar/s].</param>
	virtual void SetParams(bool AO, bool MO, double, double, double OMP, double OPD) {}; // ustawianie jakichs parametrow dla zaworu
	/// <summary>Sets the manual overcharge button state.</summary>
	/// <param name="Active">True while the button is pressed.</param>
	virtual void OvrldButton(bool Active); // przycisk recznego przeladowania/asymilacji
	/// <summary>Stores the universal-button flags (see TUniversalBrake).</summary>
	/// <param name="flag">Combined ub_* flags.</param>
	virtual void SetUniversalFlag(int flag); // przycisk uniwersalny
	/// <summary>Default constructor — clears the Sounds[] array.</summary>
	TDriverHandle()
	{
		memset(Sounds, 0, sizeof(Sounds));
	}
};

/// <summary>
/// FV4a driver's handle (classic Polish PKP cab valve, 6-position).
/// </summary>
class TFV4a : public TDriverHandle
{

  private:
	/// <summary>Control reservoir pressure [bar].</summary>
	double CP = 0.0; // zbiornik sterujący
	/// <summary>Time chamber pressure [bar].</summary>
	double TP = 0.0; // zbiornik czasowy
	/// <summary>Reductor reservoir pressure [bar].</summary>
	double RP = 0.0; // zbiornik redukcyjny

  public:
	/// <summary>Computes brake pipe flow for the FV4a handle (uses BPT[] table).</summary>
	double GetPF(double i_bcp, double PP, double HP, double dt, double ep) /*override*/;
	/// <summary>Initialises CP and RP to the supplied pressure.</summary>
	void Init(double Press) /*override*/;

	/// <summary>Default constructor.</summary>
	TFV4a() : TDriverHandle() {}
};

/// <summary>
/// FV4a/M — modernised FV4a without the IC correction. Adds a reductor air
/// chamber to model the brake pipe pressure wave at the start of a release.
/// </summary>
class TFV4aM : public TDriverHandle
{

  private:
	/// <summary>Control reservoir pressure [bar].</summary>
	double CP = 0.0; // zbiornik sterujący
	/// <summary>Time chamber pressure [bar].</summary>
	double TP = 0.0; // zbiornik czasowy
	/// <summary>Reductor reservoir pressure [bar].</summary>
	double RP = 0.0; // zbiornik redukcyjny
	/// <summary>Reductor air chamber pressure (models the release pressure wave).</summary>
	double XP = 0.0; // komora powietrzna w reduktorze — jest potrzebna do odwzorowania fali
	/// <summary>Reductor adjustment offset (turning the cap).</summary>
	double RedAdj = 0.0; // dostosowanie reduktora cisnienia (krecenie kapturkiem)
	//        Sounds: array[0..4] of real;       //wielkosci przeplywow dla dzwiekow
	/// <summary>True while the release pressure wave is active.</summary>
	bool Fala = false;
	/// <summary>Lookup of bh_* function codes to handle position values.</summary>
	static double const pos_table[11]; // = { -2, 6, -1, 0, -2, 1, 4, 6, 0, 0, 0 };

	/// <summary>
	/// Returns the brake pipe pressure target interpolated from BPT[] for the given handle position.
	/// </summary>
	/// <param name="pos">Handle position.</param>
	static double LPP_RP(double pos);
	/// <summary>Returns true if pos is within ±0.5 of i_pos (detent comparison).</summary>
	static bool EQ(double pos, double i_pos);

  public:
	/// <summary>Computes brake pipe flow for the FV4a/M handle (interpolated BPT, wave modelling, accelerator).</summary>
	double GetPF(double i_bcp, double PP, double HP, double dt, double ep) /*override*/;
	/// <summary>Initialises CP and RP.</summary>
	void Init(double Press) /*override*/;
	/// <summary>Sets the reductor adjustment offset.</summary>
	void SetReductor(double nAdj) /*override*/;
	/// <summary>Returns Sounds[i] (or 0 if i &gt; 4).</summary>
	double GetSound(int i) /*override*/;
	/// <summary>Returns pos_table[i].</summary>
	double GetPos(int i) /*override*/;
	/// <summary>Returns the time chamber pressure (TP).</summary>
	double GetCP();
	/// <summary>Returns the regulator pressure (5 + TP*0.08 + RedAdj).</summary>
	double GetRP();
	/// <summary>Default constructor.</summary>
	TFV4aM() : TDriverHandle() {}
};

/// <summary>
/// MHZ_EN57 — combined brake handle for EN57 EMUs (10-position, EP brake plus pneumatic).
/// </summary>
class TMHZ_EN57 : public TDriverHandle
{

  private:
	/// <summary>Control reservoir pressure [bar].</summary>
	double CP = 0.0; // zbiornik sterujący
	/// <summary>Time chamber pressure [bar].</summary>
	double TP = 0.0; // zbiornik czasowy
	/// <summary>Reductor reservoir pressure [bar].</summary>
	double RP = 0.0; // zbiornik redukcyjny
	/// <summary>Reductor adjustment offset.</summary>
	double RedAdj = 0.0; // dostosowanie reduktora cisnienia (krecenie kapturkiem)
	/// <summary>True while the release pressure wave is active.</summary>
	bool Fala = false;
	/// <summary>Configured unbrake over-pressure [bar].</summary>
	double UnbrakeOverPressure = 0.0;
	/// <summary>Maximum overcharge pressure when assimilating.</summary>
	double OverloadMaxPressure = 1.0; // maksymalne zwiekszenie cisnienia przy asymilacji
	/// <summary>Decay rate of the overcharge pressure.</summary>
	double OverloadPressureDecrease = 0.045; // predkosc spadku cisnienia przy asymilacji
	/// <summary>Lookup of bh_* function codes to handle position values.</summary>
	static double const pos_table[11]; //= { -2, 10, -1, 0, 0, 2, 9, 10, 0, 0, 0 };

	/// <summary>Returns the brake pipe pressure target for the given handle position (piecewise).</summary>
	static double LPP_RP(double pos);
	/// <summary>Returns true if pos is within ±0.5 of i_pos.</summary>
	static bool EQ(double pos, double i_pos);

  public:
	/// <summary>Computes brake pipe flow for MHZ_EN57 (covers handle positions -1..10 with EP/pneumatic mix).</summary>
	double GetPF(double i_bcp, double PP, double HP, double dt, double ep) /*override*/;
	/// <summary>Initialises CP.</summary>
	void Init(double Press) /*override*/;
	/// <summary>Sets the reductor adjustment offset.</summary>
	void SetReductor(double nAdj) /*override*/;
	/// <summary>Returns Sounds[i] (or 0 if i &gt; 4).</summary>
	double GetSound(int i) /*override*/;
	/// <summary>Returns pos_table[i].</summary>
	double GetPos(int i) /*override*/;
	/// <summary>Returns the regulator pressure (RP).</summary>
	double GetCP() /*override*/;
	/// <summary>Returns the regulator target (5 + RedAdj).</summary>
	double GetRP() /*override*/;
	/// <summary>Returns EP brake intensity for the given handle position.</summary>
	double GetEP(double pos);
	/// <summary>Configures handle parameters (auto/manual overcharge, over-pressure, overcharge dynamics).</summary>
	void SetParams(bool AO, bool MO, double OverP, double, double OMP, double OPD);
	/// <summary>Default constructor.</summary>
	TMHZ_EN57(void) : TDriverHandle() {}
};

/// <summary>
/// MHZ_K5P — Knorr 5-position combined brake handle.
/// </summary>
class TMHZ_K5P : public TDriverHandle
{

  private:
	/// <summary>Control reservoir pressure [bar].</summary>
	double CP = 0.0; // zbiornik sterujący
	/// <summary>Time chamber pressure [bar].</summary>
	double TP = 0.0; // zbiornik czasowy
	/// <summary>Reductor reservoir pressure [bar].</summary>
	double RP = 0.0; // zbiornik redukcyjny
	/// <summary>Reductor adjustment offset.</summary>
	double RedAdj = 0.0; // dostosowanie reduktora cisnienia (krecenie kapturkiem)
	/// <summary>True while filling-stroke / release wave is active.</summary>
	bool Fala = false; // czy jest napelnianie uderzeniowe
	/// <summary>Configured unbrake over-pressure [bar].</summary>
	double UnbrakeOverPressure = 0.0;
	/// <summary>Maximum overcharge pressure when assimilating.</summary>
	double OverloadMaxPressure = 1.0; // maksymalne zwiekszenie cisnienia przy asymilacji
	/// <summary>Decay rate of the overcharge pressure.</summary>
	double OverloadPressureDecrease = 0.002; // predkosc spadku cisnienia przy asymilacji
	/// <summary>Filling-stroke valve opening multiplier when no wave is active.</summary>
	double FillingStrokeFactor = 1.0; // mnożnik otwarcia zaworu przy uderzeniowym (bez fali)
	/// <summary>Lookup of bh_* function codes to handle position values.</summary>
	static double const pos_table[11]; //= { -2, 10, -1, 0, 0, 2, 9, 10, 0, 0, 0 };

	/// <summary>Returns true if pos is within ±0.5 of i_pos.</summary>
	static bool EQ(double pos, double i_pos);

  public:
	/// <summary>Computes brake pipe flow for the K5P 5-position handle (release / cut-off / brake / emergency).</summary>
	double GetPF(double i_bcp, double PP, double HP, double dt, double ep) /*override*/;
	/// <summary>Initialises CP and enables the time chambers.</summary>
	void Init(double Press) /*override*/;
	/// <summary>Sets the reductor adjustment offset.</summary>
	void SetReductor(double nAdj) /*override*/;
	/// <summary>Returns Sounds[i] (or 0 if i &gt; 4).</summary>
	double GetSound(int i) /*override*/;
	/// <summary>Returns pos_table[i].</summary>
	double GetPos(int i) /*override*/;
	/// <summary>Returns CP.</summary>
	double GetCP() /*override*/;
	/// <summary>Returns the regulator target (5 + TP + RedAdj).</summary>
	double GetRP() /*override*/;
	/// <summary>Configures handle parameters (auto/manual overcharge, over-pressure, filling-stroke factor, overcharge dynamics).</summary>
	void SetParams(bool AO, bool MO, double, double, double OMP, double OPD); /*ovveride*/

	/// <summary>Default constructor.</summary>
	TMHZ_K5P(void) : TDriverHandle() {}
};

/// <summary>
/// MHZ_6P — 6-position combined brake handle (similar logic to K5P with one more detent).
/// </summary>
class TMHZ_6P : public TDriverHandle
{

  private:
	/// <summary>Control reservoir pressure [bar].</summary>
	double CP = 0.0; // zbiornik sterujący
	/// <summary>Time chamber pressure [bar].</summary>
	double TP = 0.0; // zbiornik czasowy
	/// <summary>Reductor reservoir pressure [bar].</summary>
	double RP = 0.0; // zbiornik redukcyjny
	/// <summary>Reductor adjustment offset.</summary>
	double RedAdj = 0.0; // dostosowanie reduktora cisnienia (krecenie kapturkiem)
	/// <summary>True while filling-stroke / release wave is active.</summary>
	bool Fala = false; // czy jest napelnianie uderzeniowe
	/// <summary>Configured filling-stroke over-pressure value.</summary>
	double UnbrakeOverPressure = 0.0; // wartosc napelniania uderzeniowego
	/// <summary>Maximum overcharge pressure when assimilating.</summary>
	double OverloadMaxPressure = 1.0; // maksymalne zwiekszenie cisnienia przy asymilacji
	/// <summary>Decay rate of the overcharge pressure.</summary>
	double OverloadPressureDecrease = 0.002; // predkosc spadku cisnienia przy asymilacji
	/// <summary>Filling-stroke valve opening multiplier when no wave is active.</summary>
	double FillingStrokeFactor = 1.0; // mnożnik otwarcia zaworu przy uderzeniowym (bez fali)
	/// <summary>Lookup of bh_* function codes to handle position values.</summary>
	static double const pos_table[11]; //= { -2, 10, -1, 0, 0, 2, 9, 10, 0, 0, 0 };

	/// <summary>Returns true if pos is within ±0.5 of i_pos.</summary>
	static bool EQ(double pos, double i_pos);

  public:
	/// <summary>Computes brake pipe flow for the 6P handle.</summary>
	double GetPF(double i_bcp, double PP, double HP, double dt, double ep) /*override*/;
	/// <summary>Initialises CP and enables the time chambers.</summary>
	void Init(double Press) /*override*/;
	/// <summary>Sets the reductor adjustment offset.</summary>
	void SetReductor(double nAdj) /*override*/;
	/// <summary>Returns Sounds[i] (or 0 if i &gt; 4).</summary>
	double GetSound(int i) /*override*/;
	/// <summary>Returns pos_table[i].</summary>
	double GetPos(int i) /*override*/;
	/// <summary>Returns CP.</summary>
	double GetCP() /*override*/;
	/// <summary>Returns the regulator target (5 + TP + RedAdj).</summary>
	double GetRP() /*override*/;
	/// <summary>Configures handle parameters (auto/manual overcharge, over-pressure, filling-stroke factor, overcharge dynamics).</summary>
	void SetParams(bool AO, bool MO, double, double, double OMP, double OPD); /*ovveride*/

	/// <summary>Default constructor.</summary>
	TMHZ_6P(void) : TDriverHandle() {}
};

/*    FBS2= class(TTDriverHandle)
          private
            CP, TP, RP: real;      //zbiornik sterujący, czasowy, redukcyjny
            XP: real;              //komora powietrzna w reduktorze — jest potrzebna do odwzorowania fali
                RedAdj: real;          //dostosowanie reduktora cisnienia (krecenie kapturkiem)
//        Sounds: array[0..4] of real;       //wielkosci przeplywow dla dzwiekow
                Fala: boolean;
          public
                function GetPF(i_bcp:real; pp, hp, dt, ep: real): real; override;
                procedure Init(press: real); override;
                procedure SetReductor(nAdj: real); override;
                function GetSound(i: int): real; override;
                function GetPos(i: int): real; override;
          end;                    */

/*    TD2= class(TTDriverHandle)
              private
                  CP, TP, RP: real;      //zbiornik sterujący, czasowy, redukcyjny
                  XP: real;              //komora powietrzna w reduktorze — jest potrzebna do odwzorowania fali
                RedAdj: real;          //dostosowanie reduktora cisnienia (krecenie kapturkiem)
//        Sounds: array[0..4] of real;       //wielkosci przeplywow dla dzwiekow
                Fala: boolean;
              public
                function GetPF(i_bcp:real; pp, hp, dt, ep: real): real; override;
                procedure Init(press: real); override;
                procedure SetReductor(nAdj: real); override;
                function GetSound(i: int): real; override;
                function GetPos(i: int): real; override;
              end;*/

/// <summary>
/// Matrosow 394 — Russian/Soviet 5-position cab valve. Uses the BPT_394 table.
/// </summary>
class TM394 : public TDriverHandle
{

  private:
	/// <summary>Control reservoir pressure [bar].</summary>
	double CP = 0.0; // zbiornik sterujący, czasowy, redukcyjny
	/// <summary>Reductor adjustment offset.</summary>
	double RedAdj = 0.0; // dostosowanie reduktora cisnienia (krecenie kapturkiem)
	/// <summary>Lookup of bh_* function codes to handle position values.</summary>
	static double const pos_table[11]; // = { -1, 5, -1, 0, 1, 2, 4, 5, 0, 0, 0 };

  public:
	/// <summary>Computes brake pipe flow for the M394 handle (uses BPT_394).</summary>
	double GetPF(double i_bcp, double PP, double HP, double dt, double ep) /*override*/;
	/// <summary>Initialises CP and enables the time chamber.</summary>
	void Init(double Press) /*override*/;
	/// <summary>Sets the reductor adjustment offset.</summary>
	void SetReductor(double nAdj) /*override*/;
	/// <summary>Returns CP.</summary>
	double GetCP() /*override*/;
	/// <summary>Returns max(5, CP) + RedAdj.</summary>
	double GetRP() /*override*/;
	/// <summary>Returns pos_table[i].</summary>
	double GetPos(int i) /*override*/;

	/// <summary>Default constructor — sets the maximum handle position to 5.</summary>
	TM394(void) : TDriverHandle()
	{
		i_bcpno = 5;
	}
};

/// <summary>
/// H14K1 — Knorr auxiliary (independent) brake handle (4-position).
/// </summary>
class TH14K1 : public TDriverHandle
{

  private:
	/// <summary>Position table for the H14K1 (range -1..4): {flow speed, target multiplier}.</summary>
	static double const BPT_K[/*?*/ /*-1..4*/ 4 - -1 + 1][2];
	/// <summary>Lookup of bh_* function codes to handle position values.</summary>
	static double const pos_table[11]; // = {-1, 4, -1, 0, 1, 2, 3, 4, 0, 0, 0};

  protected:
	/// <summary>Control reservoir pressure [bar].</summary>
	double CP = 0.0; // zbiornik sterujący, czasowy, redukcyjny
	/// <summary>Reductor adjustment offset.</summary>
	double RedAdj = 0.0; // dostosowanie reduktora cisnienia (krecenie kapturkiem)

  public:
	/// <summary>Computes brake pipe flow for the H14K1 handle.</summary>
	double GetPF(double i_bcp, double PP, double HP, double dt, double ep) /*override*/;
	/// <summary>Initialises CP and enables the time chambers.</summary>
	void Init(double Press) /*override*/;
	/// <summary>Sets the reductor adjustment offset.</summary>
	void SetReductor(double nAdj) /*override*/;
	/// <summary>Returns CP.</summary>
	double GetCP() /*override*/;
	/// <summary>Returns the regulator target (5 + RedAdj).</summary>
	double GetRP() /*override*/;
	/// <summary>Returns pos_table[i].</summary>
	double GetPos(int i) /*override*/;

	/// <summary>Default constructor — sets the maximum handle position to 4.</summary>
	TH14K1(void) : TDriverHandle()
	{
		i_bcpno = 4;
	}
};

/// <summary>
/// St113 — Knorr EP-equipped 5-position brake handle (extends H14K1 with EP step).
/// </summary>
class TSt113 : public TH14K1
{

  private:
	/// <summary>Current EP intensity reported by the handle.</summary>
	double EPS = 0.0;
	/// <summary>Position table (override of H14K1's, with adjusted parameters).</summary>
	static double const BPT_K[/*?*/ /*-1..4*/ 4 - -1 + 1][2];
	/// <summary>EP table — EP intensity per handle position (range -1..5).</summary>
	static double const BEP_K[/*?*/ /*-1..5*/ 5 - -1 + 1];
	/// <summary>Lookup of bh_* function codes to handle position values.</summary>
	static double const pos_table[11]; // = {-1, 5, -1, 0, 2, 3, 4, 5, 0, 0, 1};
	/// <summary>Local control pressure (mirrors PP).</summary>
	double CP = 0;

  public:
	/// <summary>Computes brake pipe flow for the St113 handle (with EP).</summary>
	double GetPF(double i_bcp, double PP, double HP, double dt, double ep) /*override*/;
	/// <summary>Returns CP.</summary>
	double GetCP() /*override*/;
	/// <summary>Returns the regulator target (5 + RedAdj).</summary>
	double GetRP() /*override*/;
	/// <summary>Returns the current EP intensity.</summary>
	double GetEP() /*override*/;
	/// <summary>Returns pos_table[i].</summary>
	double GetPos(int i) /*override*/;
	/// <summary>Enables the time chambers (no pressure init).</summary>
	void Init(double Press) /*override*/;

	/// <summary>Default constructor.</summary>
	TSt113(void) : TH14K1() {}
};

/// <summary>
/// Test handle — minimal implementation used during development for verifying
/// brake pipe responses with the BPT[] table.
/// </summary>
class Ttest : public TDriverHandle
{

  private:
	/// <summary>Control reservoir pressure [bar].</summary>
	double CP = 0.0;

  public:
	/// <summary>Computes brake pipe flow using the FV4a-style BPT table for testing.</summary>
	double GetPF(double i_bcp, double PP, double HP, double dt, double ep) /*override*/;
	/// <summary>Initialises CP.</summary>
	void Init(double Press) /*override*/;

	/// <summary>Default constructor.</summary>
	Ttest(void) : TDriverHandle() {}
};

/// <summary>
/// FD1 auxiliary brake handle — directly drives the cylinder pressure between
/// 0 and MaxBP based on the handle position (linear scaling with configurable speed).
/// </summary>
class TFD1 : public TDriverHandle
{

  private:
	/// <summary>Maximum cylinder pressure [bar] commandable by this handle.</summary>
	double MaxBP = 0.0; // najwyzsze cisnienie
	/// <summary>Current commanded cylinder pressure [bar].</summary>
	double BP = 0.0; // aktualne cisnienie

  public:
	/// <summary>Action speed multiplier — scales the response time.</summary>
	double Speed = 0.0; // szybkosc dzialania

	/// <summary>Computes the auxiliary brake outflow for this step.</summary>
	double GetPF(double i_bcp, double PP, double HP, double dt, double ep) /*override*/;
	/// <summary>Initialises MaxBP and the action speed.</summary>
	void Init(double Press) /*override*/;
	/// <summary>Returns the currently commanded cylinder pressure (BP).</summary>
	double GetCP() /*override*/;
	/// <summary>Sets the action speed multiplier.</summary>
	void SetSpeed(double nSpeed);
	//        procedure Init(press: real; MaxBP: real); overload;

	/// <summary>Default constructor.</summary>
	TFD1(void) : TDriverHandle() {}
};

/// <summary>
/// H1405 — Knorr auxiliary brake handle (continuous, independent brake).
/// </summary>
class TH1405 : public TDriverHandle
{

  private:
	/// <summary>Maximum cylinder pressure [bar].</summary>
	double MaxBP = 0.0; // najwyzsze cisnienie
	/// <summary>Current commanded cylinder pressure [bar].</summary>
	double BP = 0.0; // aktualne cisnienie

  public:
	/// <summary>Computes the auxiliary brake outflow for this step (proportional to handle deflection).</summary>
	double GetPF(double i_bcp, double PP, double HP, double dt, double ep) /*override*/;
	/// <summary>Initialises MaxBP and enables the time chamber.</summary>
	void Init(double Press) /*override*/;
	/// <summary>Returns the currently commanded cylinder pressure (BP).</summary>
	double GetCP() /*override*/;
	//        procedure Init(press: real; MaxBP: real); overload;

	/// <summary>Default constructor.</summary>
	TH1405(void) : TDriverHandle() {}
};

/// <summary>
/// FVel6 — combined EP + pneumatic brake handle (Czech, 6+1 positions).
/// </summary>
class TFVel6 : public TDriverHandle
{

  private:
	/// <summary>Current EP intensity reported by the handle.</summary>
	double EPS = 0.0;
	/// <summary>Lookup of bh_* function codes to handle position values.</summary>
	static double const pos_table[11]; // = {-1, 6, -1, 0, 6, 4, 4.7, 5, -1, 0, 1};
	/// <summary>Local control pressure (mirrors PP).</summary>
	double CP = 0.0;

  public:
	/// <summary>Computes brake pipe flow for FVel6 (continuous EP brake plus pneumatic emergency).</summary>
	double GetPF(double i_bcp, double PP, double HP, double dt, double ep) /*override*/;
	/// <summary>Returns CP.</summary>
	double GetCP() /*override*/;
	/// <summary>Returns the regulator target (constant 5 bar).</summary>
	double GetRP() /*override*/;
	/// <summary>Returns the current EP intensity.</summary>
	double GetEP() /*override*/;
	/// <summary>Returns pos_table[i].</summary>
	double GetPos(int i) /*override*/;
	/// <summary>Returns Sounds[i] (or 0 if i &gt; 2).</summary>
	double GetSound(int i) /*override*/;
	/// <summary>Enables the time chambers.</summary>
	void Init(double Press) /*override*/;

	/// <summary>Default constructor.</summary>
	TFVel6(void) : TDriverHandle() {}
};

/// <summary>
/// FVE408 — newer combined EP + pneumatic brake handle (10 positions).
/// EP intensity is set to fixed steps for positions 1..5.
/// </summary>
class TFVE408 : public TDriverHandle
{

  private:
	/// <summary>Current EP intensity reported by the handle.</summary>
	double EPS = 0.0;
	/// <summary>Lookup of bh_* function codes to handle position values.</summary>
	static double const pos_table[11]; // = {-1, 6, -1, 0, 6, 4, 4.7, 5, -1, 0, 1};
	/// <summary>Local control pressure (mirrors PP).</summary>
	double CP = 0.0;

  public:
	/// <summary>Computes brake pipe flow for the FVE408 handle.</summary>
	double GetPF(double i_bcp, double PP, double HP, double dt, double ep) /*override*/;
	/// <summary>Returns CP.</summary>
	double GetCP() /*override*/;
	/// <summary>Returns the current EP intensity.</summary>
	double GetEP() /*override*/;
	/// <summary>Returns the regulator target (constant 5 bar).</summary>
	double GetRP() /*override*/;
	/// <summary>Returns pos_table[i].</summary>
	double GetPos(int i) /*override*/;
	/// <summary>Returns Sounds[i] (or 0 if i &gt; 2).</summary>
	double GetSound(int i) /*override*/;
	/// <summary>Enables the time chamber, disables the EP-time chamber.</summary>
	void Init(double Press) /*override*/;

	/// <summary>Default constructor.</summary>
	TFVE408(void) : TDriverHandle() {}
};

/// <summary>
/// Pneumatic flow rate from one pressure to another through an orifice of area S.
/// Models choked vs. subsonic flow (critical pressure ratio = 0.5) and softens
/// the response near zero pressure difference using a DP-wide ramp.
/// </summary>
/// <param name="P1">Source pressure [bar].</param>
/// <param name="P2">Destination pressure [bar].</param>
/// <param name="S">Effective orifice cross-section.</param>
/// <param name="DP">Soft-clip pressure delta — softens flow for tiny PH-PL differences (default 0.25).</param>
/// <returns>Volumetric flow rate (signed; positive = P1-&gt;P2 direction).</returns>
extern double PF(double P1, double P2, double S, double DP = 0.25);
/// <summary>
/// Variant of <see cref="PF"/> that uses the dimensionless pressure ratio (sg)
/// for the soft-clip threshold instead of an absolute pressure delta.
/// </summary>
/// <param name="P1">Source pressure [bar].</param>
/// <param name="P2">Destination pressure [bar].</param>
/// <param name="S">Effective orifice cross-section.</param>
/// <returns>Volumetric flow rate.</returns>
extern double PF1(double P1, double P2, double S);

/// <summary>
/// Filling valve flow: flows from PH to PL until PL reaches LIM. The valve
/// throttles smoothly as PL approaches LIM (within DP of the target).
/// Returns 0 once PL is already &gt;= LIM.
/// </summary>
/// <param name="PH">High-pressure source [bar].</param>
/// <param name="PL">Low-pressure side [bar].</param>
/// <param name="S">Effective orifice cross-section.</param>
/// <param name="LIM">Target pressure for PL [bar].</param>
/// <param name="DP">Throttling distance from LIM (default 0.1).</param>
/// <returns>Flow rate from PH to PL (positive into PL).</returns>
extern double PFVa(double PH, double PL, double S, double LIM, double DP = 0.1); // zawor napelniajacy z PH do PL, PL do LIM
/// <summary>
/// Venting valve flow: flows from PH to PL until PH falls to LIM. The valve
/// throttles smoothly as PH approaches LIM. Returns 0 once PH &lt;= LIM.
/// </summary>
/// <param name="PH">High-pressure side that is being vented [bar].</param>
/// <param name="PL">Low-pressure destination [bar].</param>
/// <param name="S">Effective orifice cross-section.</param>
/// <param name="LIM">Lower bound for PH [bar].</param>
/// <param name="DP">Throttling distance from LIM (default 0.1).</param>
/// <returns>Flow rate from PH to PL.</returns>
extern double PFVd(double PH, double PL, double S, double LIM, double DP = 0.1); // zawor wypuszczajacy z PH do PL, PH do LIM
