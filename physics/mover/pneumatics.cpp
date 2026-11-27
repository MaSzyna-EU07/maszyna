/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the brakes and the air: the brake controls, the pipes, the valves, the compressor, the spring brake

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


// Nastawianie hamulców

void TMoverParameters::BrakeLevelSet(double b)
{ // ustawienie pozycji hamulca na wartość (b) w zakresie od -2 do BrakeCtrlPosNo
	// jedyny dopuszczalny sposób przestawienia hamulca zasadniczego
	if (fBrakeCtrlPos == b)
		return; // nie przeliczać, jak nie ma zmiany
	fBrakeCtrlPos = b;
	if (fBrakeCtrlPos < Handle->GetPos(bh_MIN))
		fBrakeCtrlPos = Handle->GetPos(bh_MIN); // odcięcie
	else if (fBrakeCtrlPos > Handle->GetPos(bh_MAX))
		fBrakeCtrlPos = Handle->GetPos(bh_MAX);
	// TODO: verify whether BrakeCtrlPosR and fBrakeCtrlPos can be rolled into single variable
	BrakeCtrlPosR = fBrakeCtrlPos;
	auto x = static_cast<int>(std::floor(fBrakeCtrlPos)); // jeśli odwołujemy się do BrakeCtrlPos w pośrednich, to musi być
	// obcięte a nie zaokrągone
	while (x > BrakeCtrlPos && BrakeCtrlPos < BrakeCtrlPosNo) // jeśli zwiększyło się o 1
		if (!IncBrakeLevelOld()) // T_MoverParameters::
			break; // wyjście awaryjne
	while (x < BrakeCtrlPos && BrakeCtrlPos >= -1) // jeśli zmniejszyło się o 1
		if (!DecBrakeLevelOld()) // T_MoverParameters::
			break;
	BrakePressureActual = BrakePressureTable[BrakeCtrlPos]; // skopiowanie pozycji
	/*
	//youBy: obawiam sie, ze tutaj to nie dziala :P
	//Ra 2014-03: było tak zrobione, że działało - po każdej zmianie pozycji była wywoływana ta
	funkcja
	// if (BrakeSystem==Pneumatic?BrakeSubsystem==Oerlikon:false) //tylko Oerlikon akceptuje ułamki
	 if(false)
	  if (fBrakeCtrlPos>0.0)
	  {//wartości pośrednie wyliczamy tylko dla hamowania
	   double u=fBrakeCtrlPos-double(x); //ułamek ponad wartość całkowitą
	   if (u>0.0)
	   {//wyliczamy wartości ważone
	    BrakePressureActual.PipePressureVal+=-u*BrakePressureActual.PipePressureVal+u*BrakePressureTable[BrakeCtrlPos+1+2].PipePressureVal;
	    //BrakePressureActual.BrakePressureVal+=-u*BrakePressureActual.BrakePressureVal+u*BrakePressureTable[BrakeCtrlPos+1].BrakePressureVal;
	//to chyba nie będzie tak działać, zwłaszcza w EN57
	    BrakePressureActual.FlowSpeedVal+=-u*BrakePressureActual.FlowSpeedVal+u*BrakePressureTable[BrakeCtrlPos+1+2].FlowSpeedVal;
	   }
	  }
	*/
};

bool TMoverParameters::BrakeLevelAdd(double b)
{ // dodanie wartości (b) do pozycji hamulca (w tym ujemnej)
	// zwraca false, gdy po dodaniu było by poza zakresem
	BrakeLevelSet(fBrakeCtrlPos + b);
	return b > 0.0 ? fBrakeCtrlPos < BrakeCtrlPosNo : BrakeCtrlPos > -1.0; // true, jeśli można kontynuować
};

bool TMoverParameters::IncBrakeLevel()
{ // nowa wersja na użytek AI, false gdy osiągnięto pozycję BrakeCtrlPosNo
	return BrakeLevelAdd(1.0);
};

bool TMoverParameters::DecBrakeLevel()
{
	return BrakeLevelAdd(-1.0);
}; // nowa wersja na użytek AI, false gdy osiągnięto pozycję -1

/* Ukrotnienie EN57:
1 //układ szeregowy
2 //układ równoległy
3 //bocznik 1
4 //bocznik 2
5 //bocznik 3
6 //do przodu
7 //do tyłu
8 //1 przyspieszenie
9 //minus obw. 2 przyspieszenia
10 //jazda na oporach
11 //SHP
12A //podnoszenie pantografu przedniego
12B //podnoszenie pantografu tylnego
13A //opuszczanie pantografu przedniego
13B //opuszczanie wszystkich pantografów
14 //załączenie WS
15 //rozrząd (WS, PSR, wał kułakowy)
16 //odblok PN
18 //sygnalizacja przetwornicy głównej
19 //luzowanie EP
20 //hamowanie EP
21 //rezerwa** (1900+: zamykanie drzwi prawych)
22 //zał. przetwornicy głównej
23 //wył. przetwornicy głównej
24 //zał. przetw. oświetlenia
25 //wył. przetwornicy oświetlenia
26 //sygnalizacja WS
28 //sprężarka
29 //ogrzewanie
30 //rezerwa* (1900+: zamykanie drzwi lewych)
31 //otwieranie drzwi prawych
32H //zadziałanie PN siln. trakcyjnych
33 //sygnał odjazdu
34 //rezerwa (sygnalizacja poślizgu)
35 //otwieranie drzwi lewych
ZN //masa
*/

// *****************************************************************************
// Q: 20160714
// Oblicza iloraz aktualnej pozycji do maksymalnej hamulca pomocnicznego
// *****************************************************************************
double TMoverParameters::LocalBrakeRatio(void) const
{
	double LBR;
	if (BrakeHandle == TBrakeHandle::MHZ_EN57)
		if (BrakeOpModeFlag >= bom_EP)
			LBR = Handle->GetEP(BrakeCtrlPosR);
		else
			LBR = 0;
	else
	{
		if (LocalBrakePosNo > 0)
			LBR = LocalBrakePosA;
		else
			LBR = 0;
	}
	// if (TestFlag(BrakeStatus, b_antislip))
	//   LBR = std::max(LBR, PipeRatio) + 0.4;
	return LBR;
}

// *****************************************************************************
// Q: 20160714
// Oblicza iloraz aktualnej pozycji do maksymalnej hamulca ręcznego
// *****************************************************************************
double TMoverParameters::ManualBrakeRatio(void) const
{
	double MBR;

	if (ManualBrakePosNo > 0)
		MBR = (double)ManualBrakePos / ManualBrakePosNo;
	else
		MBR = 0;
	return MBR;
}

// *****************************************************************************
// Q: 20160713
// Zwraca objętość
// *****************************************************************************
double TMoverParameters::BrakeVP(void) const
{
	if (BrakeVVolume > 0)
		return Volume / (10.0 * BrakeVVolume);
	else
		return 0;
}

// *****************************************************************************
// Q: 20160713
// Zwraca iloraz różnicy między przewodem kontrolnym i głównym oraz DeltaPipePress
// *****************************************************************************
double TMoverParameters::RealPipeRatio(void) const
{
	double rpp;

	if (DeltaPipePress > 0)
		rpp = (CntrlPipePress - PipePress) / DeltaPipePress;
	else
		rpp = 0;
	return rpp;
}

// *****************************************************************************
// Q: 20160713
// Zwraca iloraz ciśnienia w przewodzie do DeltaPipePress
// *****************************************************************************
double TMoverParameters::PipeRatio(void) const
{
	double pr;

	if (DeltaPipePress > 0)
		if (false) // SPKS!! no to jak nie wchodzimy to po co branch?
		{
			if (3.0 * PipePress > HighPipePress + LowPipePress + LowPipePress)
				pr = (HighPipePress - std::min(HighPipePress, PipePress)) / (DeltaPipePress * 4.0 / 3.0);
			else
				pr = (HighPipePress - 1.0 / 3.0 * DeltaPipePress - std::max(LowPipePress, PipePress)) / (DeltaPipePress * 2.0 / 3.0);
			// if (not TestFlag(BrakeStatus, b_Ractive))
			//     and(BrakeMethod and 1 = 0) and TestFlag(BrakeDelays, bdelay_R) and (Power < 1) and
			//         (BrakeCtrlPos < 1) then pr : = std::min(0.5, pr);
			// if (Compressor > 0.5)
			//     then pr : = pr * 1.333; // dziwny rapid wywalamy
		}
		else
			pr = (HighPipePress - std::max(LowPipePress, std::min(HighPipePress, PipePress))) / DeltaPipePress;
	else
		pr = 0;
	return pr;
}

// *************************************************************************************************
// Q: 20160710
// włączenie / wyłączenie hamulca elektro-pneumatycznego
// *************************************************************************************************
bool TMoverParameters::EpFuseSwitch(bool State)
{
	if (EpFuse != State)
	{
		EpFuse = State;
		return true;
	}
	else
		return false;
	// if (EpFuse == true) SendCtrlToNext("EpFuseSwitch", 1, CabActive)
	//  else SendCtrlToNext("EpFuseSwitch", 0, CabActive);
}

// *************************************************************************************************
// yB: 20190906
// włączenie / wyłączenie hamulca sprezynowego
// *************************************************************************************************
bool TMoverParameters::SpringBrakeActivate(bool State)
{
	if (Power24vIsAvailable || Power110vIsAvailable)
	{
		SendCtrlToNext("SpringBrakeActivate", int(State), CabActive, SpringBrake.MultiTractionCoupler);

		if (SpringBrake.Activate != State)
		{
			SpringBrake.Activate = State;
			wake_for_brake();
			return true;
		}
	}
	return false;
}

// *************************************************************************************************
// yB: 20190906
// włączenie / wyłączenie odciecia hamulca sprezynowego
// *************************************************************************************************
bool TMoverParameters::SpringBrakeShutOff(bool State)
{
	if (SpringBrake.ShuttOff != State)
	{
		SpringBrake.ShuttOff = State;
		return true;
	}
	else
		return false;
}

// *************************************************************************************************
// yB: 20190906
// wyluzowanie hamulca sprezynowego
// *************************************************************************************************
bool TMoverParameters::SpringBrakeRelease()
{
	if (SpringBrake.IsReady && SpringBrake.Cylinder->P() < SpringBrake.MinForcePressure)
	{
		SpringBrake.IsReady = false;
		wake_for_brake();
		return true;
	}
	else
		return false;
}

// *************************************************************************************************
// Q: 20160710
// załączenie przycisku przeciwpoślizgowego
// *************************************************************************************************
bool TMoverParameters::AntiSlippingButton(void)
{
	// NOTE: disabled the sandbox part, it's already controlled by another part of the AI routine
	return AntiSlippingBrake() /*|| Sandbox(true)*/;
}

// *************************************************************************************************
// Q: 20160713
// włączenie / wyłączenie sprężarki
// *************************************************************************************************
bool TMoverParameters::CompressorSwitch(bool State, range_t const Notify)
{

	if (CompressorStart != start_t::manual)
	{
		// only pay attention if the compressor can be controlled manually
		return false;
	}

	auto const initialstate{CompressorAllow};

	if (VeselVolume > 0.0 && CompressorSpeed > 0.0)
	{
		CompressorAllow = State;
	}

	if (Notify != range_t::local)
	{
		SendCtrlToNext("CompressorSwitch", State ? 1 : 0, CabActive, Notify == range_t::unit ? coupling::control | coupling::permanent : coupling::control);
	}

	return CompressorAllow != initialstate;
}

bool TMoverParameters::ChangeCompressorPreset(int const State, range_t const Notify)
{

	auto const initialstate{CompressorListPos};

	CompressorListPos = safe_clamp(State, 0, CompressorListPosNo);

	if (Notify != range_t::local)
	{
		SendCtrlToNext("CompressorPreset", State, CabActive, Notify == range_t::unit ? coupling::control | coupling::permanent : coupling::control);
	}

	return CompressorListPos != initialstate;
}

// *************************************************************************************************
// Q: 20160711
// zwiększenie nastawy hamulca
// *************************************************************************************************
bool TMoverParameters::IncBrakeLevelOld(void)
{
	bool IBLO = false;

	if (BrakeCtrlPosNo > 0)
	{
		if (BrakeCtrlPos < BrakeCtrlPosNo)
		{
			++BrakeCtrlPos;
			// youBy: EP po nowemu
			IBLO = true;
			if (BrakePressureActual.PipePressureVal < 0 && BrakePressureTable[BrakeCtrlPos - 1].PipePressureVal > 0)
				LimPipePress = PipePress;
		}
		else
		{
			IBLO = false;
		}
	}

	return IBLO;
}

// *****************************************************************************
// Q: 20160711
// zmniejszenie nastawy hamulca
// *****************************************************************************
bool TMoverParameters::DecBrakeLevelOld(void)
{
	bool DBLO = false;

	if (BrakeCtrlPosNo > 0)
	{
		if (BrakeCtrlPos > (BrakeHandle == TBrakeHandle::FV4a ? -2 : -1))
		{
			--BrakeCtrlPos;
			// youBy: EP po nowemu
			DBLO = true;
			//            if ((BrakePressureTable[BrakeCtrlPos].PipePressureVal<0.0) &&
			//                (BrakePressureTable[BrakeCtrlPos+1].PipePressureVal > 0))
			//                LimPipePress=PipePress;
		}
		else
			DBLO = false;
	}

	return DBLO;
}

// *************************************************************************************************
// Q: 20160711
// zwiększenie nastawy hamulca pomocnicznego
// *************************************************************************************************
bool TMoverParameters::IncLocalBrakeLevel(float const CtrlSpeed)
{
	bool IBL;
	if (LocalBrakePosA < 1.0 /*and (BrakeCtrlPos<1)*/)
	{
		LocalBrakePosA = std::min(1.0, LocalBrakePosA + CtrlSpeed / LocalBrakePosNo);
		wake_for_brake();
		IBL = true;
	}
	else
		IBL = false;

	return IBL;
}

// *************************************************************************************************
// Q: 20160711
// zmniejszenie nastawy hamulca pomocniczego
// *************************************************************************************************
bool TMoverParameters::DecLocalBrakeLevel(float const CtrlSpeed)
{
	bool DBL;
	if (LocalBrakePosA > 0)
	{
		LocalBrakePosA = std::max(0.0, LocalBrakePosA - CtrlSpeed / LocalBrakePosNo);
		wake_for_brake();
		DBL = true;
	}
	else
		DBL = false;

	return DBL;
}

// *************************************************************************************************
// Q: 20160711
// zwiększenie nastawy hamulca ręcznego
// *************************************************************************************************
bool TMoverParameters::IncManualBrakeLevel(int CtrlSpeed)
{
	bool IMBL;
	if (ManualBrakePos < ManualBrakePosNo) /*and (BrakeCtrlPos<1)*/
	{
		while (ManualBrakePos < ManualBrakePosNo && CtrlSpeed > 0)
		{
			ManualBrakePos++;
			CtrlSpeed--;
		}
		wake_for_brake();
		IMBL = true;
	}
	else
		IMBL = false;

	return IMBL;
}

// *************************************************************************************************
// Q: 20160711
// zmniejszenie nastawy hamulca ręcznego
// *************************************************************************************************
bool TMoverParameters::DecManualBrakeLevel(int CtrlSpeed)
{
	bool DMBL;
	if (ManualBrakePos > 0)
	{
		while (CtrlSpeed > 0 && ManualBrakePos > 0)
		{
			ManualBrakePos--;
			CtrlSpeed--;
		}
		wake_for_brake();
		DMBL = true;
	}
	else
		DMBL = false;
	return DMBL;
}

bool TMoverParameters::AlarmChainSwitch(bool const State)
{

	if (AlarmChainFlag != State)
	{
		// simple routine for the time being
		AlarmChainFlag = State;
		return true;
	}
	return false;
}

// *************************************************************************************************
// Q: 20160710
// hamowanie przeciwpoślizgowe
// *************************************************************************************************
bool TMoverParameters::AntiSlippingBrake(void)
{
	bool ASB = false; // Ra: przeniesione z końca
	if (ASBType == 1)
	{
		ASB = true; // SPKS!!
		Hamulec->ASB(1);
		BrakeSlippingTimer = 0;
	}
	return ASB;
}

// *************************************************************************************************
// Q: 20160711
// włączenie / wyłączenie odluźniacza
// *************************************************************************************************
bool TMoverParameters::BrakeReleaser(int state)
{
	bool OK = true; // false tylko jeśli nie uda się wysłać, GF 20161124
	if (state != 0)
	{
		// additional limitations imposed by pressure switch
		if (false == ControlPressureSwitch || false == ReleaserEnabledOnlyAtNoPowerPos || true == IsMainCtrlNoPowerPos())
		{
			Hamulec->Releaser(state);
		}
	}
	else
	{
		Hamulec->Releaser(state);
	}
	if (CabActive != 0) // rekurencyjne wysłanie do następnego
		OK = SendCtrlToNext("BrakeReleaser", state, CabActive);
	return OK;
}

// *************************************************************************************************
// yB: 20160711
// włączenie / wyłączenie uniwersalnego przycisku hamulcowego
// *************************************************************************************************
bool TMoverParameters::UniversalBrakeButton(int button, int state)
{
	bool OK = true; // false tylko jeśli nie uda się wysłać, GF 20161124
	UniversalBrakeButtonActive[button] = state > 0;
	int flag = 0;
	if (Power24vIsAvailable || Power110vIsAvailable)
	{
		for (int i = 0; i < 3; i++)
		{
			flag = flag | (UniversalBrakeButtonActive[i] ? UniversalBrakeButtonFlag[i] : 0);
		}
	}

	Hamulec->SetUniversalFlag(flag);
	Handle->SetUniversalFlag(flag);
	LocHandle->SetUniversalFlag(flag);
	UnlockPipe = (flag & TUniversalBrake::ub_UnlockPipe) > 0;

	// if the releaser can be activated by switch
	if (TestFlag(UniversalBrakeButtonFlag[0] & UniversalBrakeButtonFlag[1] & UniversalBrakeButtonFlag[2], TUniversalBrake::ub_Release))
	{
		BrakeReleaser(TestFlag(flag, TUniversalBrake::ub_Release) ? 1 : 0);
	}
	return OK;
}

// *************************************************************************************************
// Q: 20160711
// włączenie / wyłączenie hamulca elektro-pneumatycznego
// *************************************************************************************************
bool TMoverParameters::SwitchEPBrake(int state)
{
	bool OK;

	OK = false;
	if (BrakeHandle == TBrakeHandle::St113 && CabOccupied != 0)
	{
		// the handle works the EP contactors pneumatically, their supply is cut as the Oerlikon handle's is
		if (state > 0 && EPBrakeControlPowered())
			EpForce = Handle->GetEP(); // TODO: przetlumaczyc
		else
			EpForce = 0;
		Hamulec->SetEPS(EpForce);
		SendCtrlToNext("Brake", EpForce, CabActive);
	}
	//  OK:=SetFlag(BrakeStatus,((2*State-1)*b_epused));
	//  SendCtrlToNext('Brake',(state*(2*BrakeCtrlPos-1)),CabActive);
	return OK;
}

bool TMoverParameters::EPBrakeControlPowered() const
{
	return Power24vIsAvailable && DirActive != 0 && EpFuse;
}

// *************************************************************************************************
// Q: 20160711
// zwiększenie ciśnienia hamowania
// *************************************************************************************************
bool TMoverParameters::IncBrakePress(double &brake, double PressLimit, double dp)
{
	bool IBP;
	//  if (DynamicBrakeType<>dbrake_switch) and (DynamicBrakeType<>dbrake_none) and
	//  ((BrakePress>2.0) or (PipePress<3.7{(LowPipePress+0.5)})) then
	if (DynamicBrakeType != dbrake_switch && DynamicBrakeType != dbrake_none && BrakePress > 2.0 && TrainType != train_type::emu) // yB radzi nie sprawdzać ciśnienia w przewodzie
	// hunter-301211: dla EN57 silnikow nie odlaczamy
	{
		DynamicBrakeFlag = true; // uruchamianie hamulca ED albo odlaczanie silnikow
		if (DynamicBrakeType == dbrake_automatic && abs(Im) > 60) // nie napelniaj wiecej, jak na EP09
			dp = 0.0;
	}
	if (brake + dp < PressLimit)
	{
		brake = brake + dp;
		IBP = true;
	}
	else
	{
		IBP = false;
		brake = PressLimit;
	}
	return IBP;
}

// *************************************************************************************************
// Q: 20160711
// zmniejszenie ciśnienia hamowania
// *************************************************************************************************
bool TMoverParameters::DecBrakePress(double &brake, double PressLimit, double dp)
{
	bool DBP;

	if (brake - dp > PressLimit)
	{
		brake = brake - dp;
		DBP = true;
	}
	else
	{
		DBP = false;
		brake = PressLimit;
	}
	//  if ((DynamicBrakeType != dbrake_switch) && ((BrakePress < 0.1) && (PipePress > 0.45
	//  /*(LowPipePress+0.06)*/ )))
	if (DynamicBrakeType != dbrake_switch && BrakePress < 0.1) // yB radzi nie sprawdzać ciśnienia w przewodzie
		DynamicBrakeFlag = false; // wylaczanie hamulca ED i/albo zalaczanie silnikow

	return DBP;
}

// *************************************************************************************************
// Q: 20160711
// przełączenie nastawy hamulca O/P/T
// *************************************************************************************************
bool TMoverParameters::BrakeDelaySwitch(int BDS)
{
	bool rBDS;
	if (Hamulec->SetBDF(BDS))
	{
		BrakeDelayFlag = BDS;
		rBDS = true;
		Hamulec->SetBrakeStatus(Hamulec->GetBrakeStatus() & ~64);
		// kopowanie nastawy hamulca do kolejnego czlonu - do przemyślenia
		if (CabActive != 0)
			SendCtrlToNext("BrakeDelay", BrakeDelayFlag, CabActive);
	}
	else
		rBDS = false;

	return rBDS;
}

// *************************************************************************************************
// Q: 20160712
// zwiększenie przełożenia hamulca
// *************************************************************************************************
bool TMoverParameters::IncBrakeMult(void)
{
	bool IBM;

	if (LoadFlag > 0 && MBPM < 2 && LoadFlag < 3)
	{
		if (MaxBrakePress[2] > 0 && LoadFlag == 1)
			LoadFlag = 2;
		else
			LoadFlag = 3;
		IBM = true;
		if (BrakeCylMult[2] > 0)
			BrakeCylMult[0] = BrakeCylMult[2];
	}
	else
		IBM = false;

	return IBM;
}

// *************************************************************************************************
// Q: 20160712
// zmniejszenie przełożenia hamulca
// *************************************************************************************************
bool TMoverParameters::DecBrakeMult(void)
{
	bool DBM;
	if (LoadFlag > 1 && MBPM < 2)
	{
		if (MaxBrakePress[2] > 0 && LoadFlag == 3)
			LoadFlag = 2;
		else
			LoadFlag = 1;
		DBM = true;
		if (BrakeCylMult[1] > 0)
			BrakeCylMult[0] = BrakeCylMult[1];
	}
	else
		DBM = false;

	return DBM;
}

// *************************************************************************************************
// Q: 20160712
// zaktualizowanie ciśnienia w hamulcach
// *************************************************************************************************
void TMoverParameters::UpdateBrakePressure(double dt)
{
	// const double LBDelay = 5.0; // stala czasowa hamulca
	// double Rate, Speed, dp, sm;

	dpLocalValve = 0;
	dpBrake = 0;

	Hamulec->ForceLeak(dt * AirLeakRate * 0.25); // fake air leaks from brake system reservoirs

	auto const brakepress{BrakePress};
	BrakePress = Hamulec->GetBCP();
	if (dt > 0.0)
	{
		// averaged over about a second: the valves move the cylinder in pulses, quicker than that within a single step
		auto const rate{std::abs(BrakePress - brakepress) / dt};
		BrakePressRate += (rate - BrakePressRate) * std::min(1.0, dt / 1.0);
	}
	//  BrakePress:=(Hamulec as TEst4).ImplsRes.pa;
	Volume = Hamulec->GetBRP();
}

// *************************************************************************************************
// Q: 20160712
// Obliczanie pracy sprężarki
// *************************************************************************************************
void TMoverParameters::CompressorCheck(double dt)
{

	if (CompressorSpeed == 0.0)
	{
		CompressorAllow = false;
		return;
	}

	if (CabDependentCompressor)
	{
		if (CabActive > 0)
		{
			MinCompressor = MinCompressor_cabA;
			MaxCompressor = MaxCompressor_cabA;
		}
		if (CabActive < 0)
		{
			MinCompressor = MinCompressor_cabB;
			MaxCompressor = MaxCompressor_cabB;
		}
	}

	// EmergencyValve
	EmergencyValveOpen = Compressor > (EmergencyValveOpen ? EmergencyValveOff : EmergencyValveOn);
	if (EmergencyValveOpen)
	{
		auto dV = static_cast<float>(PF(0, Compressor, EmergencyValveArea) * dt);
		CompressedVolume -= dV;
	}

	CompressedVolume = std::max(0.0, CompressedVolume - dt * AirLeakRate * 0.1); // nieszczelności: 0.001=1l/s

	Compressor = CompressedVolume / VeselVolume;

	// assorted operational logic
	auto const MaxCompressorF{CompressorList[TCompressorList::cl_MaxFactor][CompressorListPos] * MaxCompressor};
	auto const MinCompressorF{CompressorList[TCompressorList::cl_MinFactor][CompressorListPos] * MinCompressor};
	auto const CompressorSpeedF{CompressorList[TCompressorList::cl_SpeedFactor][CompressorListPos] * CompressorSpeed};
	// checking the impact on the compressor allowance
	if (auto const AllowFactor{CompressorList[TCompressorList::cl_Allow][CompressorListPos]}; AllowFactor > 0.5)
	{
		CompressorAllow = AllowFactor > 1.5;
	}

	switch (CompressorPower)
	{
	case 2:
	{
		CompressorAllow = ConverterAllow;
		break;
	}
	case 3:
	{
		// HACK: make sure compressor coupled with diesel engine is always ready for work
		CompressorStart = start_t::automatic;
		break;
	}
	default:
	{
		break;
	}
	}

	// the main circuit, also for a compressor driven by the engine, otherwise the low voltage
	auto const compressorpower{CompressorPower == 0 || CompressorPower == 3 ? MainCircuitPowered() : Power110vIsAvailable};
	CompressorPowerTime = compressorpower ? CompressorPowerTime + dt : 0.0;
	auto const compressorready{compressorpower && CompressorPowerTime >= CompressorStartDelay};
	// TBD: split CompressorAllow into separate enable/disable flags, inherit compressor from basic_device
	auto const compressorenable{CompressorAllowLocal && (CompressorStart == start_t::automatic || CompressorAllow)};
	auto const compressordisable{false == compressorenable};

	auto const pressureistoolow{Compressor < MinCompressorF};
	auto const pressureistoohigh{Compressor > MaxCompressorF};

	// TBD, TODO: break the lock with no low voltage power?
	auto const governorlockispresent{MaxCompressorF - MinCompressorF > 0.0001};
	CompressorGovernorLock = governorlockispresent && false == pressureistoolow // unlock if pressure drops below minimal threshold
	                         && (pressureistoohigh || CompressorGovernorLock); // lock if pressure goes above maximum threshold
	// for these multi-unit engines compressors turn off whenever any of them was affected by the governor
	// NOTE: this is crude implementation, limited only to adjacent vehicles
	// TODO: re-implement when a more elegant/flexible system is in place
	auto const coupledgovernorlock{
	    (Couplers[end::rear].Connected != nullptr && true == TestFlag(Couplers[end::rear].CouplingFlag, coupling::permanent) && Couplers[end::rear].Connected->CompressorGovernorLock) ||
	    (Couplers[end::front].Connected != nullptr && true == TestFlag(Couplers[end::front].CouplingFlag, coupling::permanent) && Couplers[end::front].Connected->CompressorGovernorLock)};
	auto const governorlock{CompressorGovernorLock || coupledgovernorlock};

	auto const compressorflag{CompressorFlag};
	CompressorFlag =
	    compressorready && false == compressordisable && (false == governorlock || CompressorPower == 3) && (CompressorFlag || (compressorenable && LastSwitchingTime > CtrlDelay));

	if (CompressorFlag && CompressorFlag != compressorflag)
	{
		// jeśli została załączona to trzeba ograniczyć ponowne włączenie
		LastSwitchingTime = 0;
	}
	if (CompressorFlag != compressorflag)
	{
		Timer::subsystem.switching.note(CompressorFlag ? Timer::switching_counter::compressor_on : Timer::switching_counter::compressor_off, this, &TypeName, &Name);
	}

	if (false == CompressorFlag)
	{
		return;
	}

	// working compressor adds air to the air reservoir
	if (CompressorPower == 3)
	{
		// the compressor is coupled with the diesel engine, engine revolutions affect the output
		CompressedVolume +=
		    CompressorSpeedF * (2.0 * MaxCompressorF - Compressor) / MaxCompressorF * EngineRPMRatio() * dt * (CompressorGovernorLock ? 0.0 : 1.0); // with the lock active air is vented out
	}
	else
	{
		// the compressor is a stand-alone device, working at steady pace
		CompressedVolume += CompressorSpeedF * (2.0 * MaxCompressorF - Compressor) / MaxCompressorF * dt;
	}

	if (pressureistoohigh && (false == governorlockispresent || CompressorPower == 3))
	{
		// vent some air out if there's no governor lock to stop the compressor from exceeding acceptable pressure level
		SetFlag(SoundFlag, sound::relay | sound::loud);
		auto ventedfraction{0.999}; // HACK: drop a tiny bit so the sound doesn't trigger repeatedly
		if (false == governorlockispresent)
		{
			ventedfraction = 0.80; // arbitrary amount
		}
		else if (CompressorTankValve)
		{
			ventedfraction = MinCompressorF / MaxCompressorF; // drop to mincompressor level
		}
		CompressedVolume *= ventedfraction;
		if (false == governorlockispresent || CompressorTankValve)
		{
			CompressorGovernorLock = false;
		}
	}

	// a compressor not driven by the engine doesn't draw power (some 5 A) from the high- or low voltage circuit yet
}

// *************************************************************************************************
// Q: 20160712
// aktualizacja ciśnienia w przewodzie głównym
// *************************************************************************************************
void TMoverParameters::UpdatePipePressure(double dt)
{
	if (PipePress > 1.0)
	{
		Pipe->Flow(-PipePress*AirLeakRate * dt);
		Pipe->Act();
	}

	// double dV;
	// TMoverParameters *c; // T_MoverParameters
	double temp;
	// int b;

	PipePress = Pipe->P();
	//  PPP:=PipePress;

	dpMainValve = 0;

	if (BrakeCtrlPosNo > 1)
	{

		UpdateLocalBrakeValve(dt);
		UpdateMainValve(dt);
	}

	UpdateEmergencyValve(dt);

	// 0.2*Spg
	Pipe->Flow(-dpMainValve);
	Pipe->Flow(-PipePress * 0.001 * dt);
	//      if Heating then
	//        Pipe.Flow(PF(PipePress, 0, d2A(7)) * dt);
	//      if ConverterFlag then
	//        Pipe.Flow(PF(PipePress, 0, d2A(12)) * dt);
	dpMainValve = dpMainValve / (Dim.L * Spg * 20);

	CntrlPipePress = Hamulec->GetVRP(); // ciśnienie komory wstępnej rozdzielacza

	//      if (Hamulec is typeid(TWest)) return 0;

	UpdateDistributorSettings();

	if ((BrakeHandle == TBrakeHandle::FVel6 || BrakeHandle == TBrakeHandle::FVE408) && CabOccupied != 0)
	{
		UpdateEPBrake();
	}
	else if (BrakeHandle == TBrakeHandle::St113 && CabOccupied != 0 && std::abs(EpForce) > 0.001 && !EPBrakeControlPowered())
	{
		// the EP brake switched on with the button drops out with the control's supply
		SwitchEPBrake(0);
	}

	Pipe->Act();
	PipePress = Pipe->P();
	if ((Hamulec->GetBrakeStatus() & b_dmg) == b_dmg) // jesli hamulec wyłączony
		temp = 0.0; // odetnij
	else
		temp = 1.0; // połącz
	Pipe->Flow(temp * Hamulec->GetPF(temp * PipePress, dt, Vel));
	// the air the brake hoses carry to and from the neighbours comes once all vehicles have stepped, UpdatePipeCouplers()
	PipeCouplersPending = true;

	if (ASBType == 128)
		Hamulec->ASB(int(SlippingWheels && Vel > 1) * (1 + 2 * int(nrot_eps < -0.01)));

	dpPipe = 0;

	// yB: jednokrokowe liczenie tego wszystkiego
	Pipe->Act();
	PipePress = Pipe->P();

	dpMainValve = dpMainValve / (100.0 * dt); // normalizacja po czasie do syczenia;

	if (PipePress < -1.0)
	{
		PipePress = -1.0;
		Pipe->CreatePress(-1.0);
		Pipe->Act();
	}

	if (CompressedVolume < 0.0)
		CompressedVolume = 0.0;
}

// the flow of the local brake valve, by the handle, the TRAXX's timed one, the integrated controller or the parking
void TMoverParameters::UpdateLocalBrakeValve(double dt)
{
	// the local brake of a vehicle prepared for hauling (PulleeMode) is held released, whatever asks for it
	if (EngineType != TEngineType::ElectricInductionMotor)
	{
		double lbpa = LocHandleTimeTraxx ? eim_localbrake : LocalBrakePosA;
		if (EIMCtrlType > 0 && UniCtrlIntegratedLocalBrakeCtrl)
		{
			lbpa = std::max(0.0, -eimic_real);
		}
		if (SpeedCtrlUnit.Parking)
		{
			lbpa = std::max(lbpa, StopBrakeDecc);
		}
		dpLocalValve = LocHandle->GetPF(PulleeMode ? 0.0 : std::max(lbpa, LocalBrakePosAEIM), Hamulec->GetBCP(), ScndPipePress, dt, 0);
	}
	else
	{
		// in EIM stock the pneumatic local brake is normally driven only by the MED algorithm
		// (LocalBrakePosAEIM). When SplitEDPneumaticBrake is active the dedicated LocalBrake
		// lever should apply pneumatic pressure on the locomotive directly, bypassing MED.
		// LocHandleTimeTraxx also bypasses MED, but has 3 positions (brake-hold-unbrake).
		double lbpa;
		if (LocHandleTimeTraxx)
		{
			lbpa = eim_localbrake;
		}
		else if (SplitEDPneumaticBrake) // When LocHandleTimeTraxx and SplitEDPneumaticBrake are both active, lbpa is taken from LocHandleTimeTraxx, which is the desired behavior.
		{
			lbpa = LocalBrakePosA;
		}
		else
		{
			lbpa = 0.0;
		}
		dpLocalValve = LocHandle->GetPF(PulleeMode ? 0.0 : std::max(lbpa, LocalBrakePosAEIM), Hamulec->GetBCP(), ScndPipePress, dt, 0);
	}
}

// the lock of the brake pipe and the flow of the driver's handle, from the main reservoir or into the air
void TMoverParameters::UpdateMainValve(double dt)
{
	auto const temp{MainValveSupply()};
	Handle->SetReductor(BrakeCtrlPos2);

	if (PulleeMode)
	{
		// the driver's valve of a vehicle prepared for hauling is cut out, whatever its handle shows
		dpMainValve = 0;
	}
	else if ((BrakeOpModes & bom_PS) == 0 || (CabOccupied != 0 && BrakeOpModeFlag != bom_PS))
	{

		if (BrakeOpModeFlag < bom_EP || Handle->GetPos(bh_EB) - 0.5 < BrakeCtrlPosR || (BrakeHandle != TBrakeHandle::MHZ_EN57 && BrakeHandle != TBrakeHandle::MHZ_K8P))
		{
			dpMainValve = Handle->GetPF(MainValveHandlePosition(), PipePress, temp, dt, EqvtPipePress);
		}
		else
		{
			dpMainValve = Handle->GetPF(0, PipePress, temp, dt, EqvtPipePress);
		}
	}
	else if (BrakeCtrlPos == Handle->GetPos(bh_EB))
	{
		dpMainValve = Handle->GetPF(BrakeCtrlPosR, PipePress, temp, dt, EqvtPipePress);
	}

	if (dpMainValve < 0) // && (PipePressureVal > 0.01)           //50
	{
		if (Compressor > ScndPipePress)
		{
			CompressedVolume = CompressedVolume + dpMainValve / 1500.0;
			Pipe2->Flow(dpMainValve / 3.0);
		}
		else
			Pipe2->Flow(dpMainValve);
	}
}

// the pressure the driver's handle is fed from: the main pipe's, or the brake pipe's own with the pipe locked
double TMoverParameters::MainValveSupply()
{
	LockPipe = PipePress < (LockPipe ? LockPipeOff : LockPipeOn);
	bool lock_new = (LockPipe && !UnlockPipe && BrakeCtrlPosR > HandleUnlock) || (EmergencyCutsOffHandle && EmergencyValveFlow > 0); // new simple codition based on .fiz
	bool lock_old = BrakeHandle == TBrakeHandle::FV4a // old complex condition based on assumptions
	                && PipePress < 2.75 && (Hamulec->GetStatus() & b_rls) == 0 && BrakeSubsystem == TBrakeSubSystem::ss_LSt && TrainType != train_type::emu && !UnlockPipe;

	if (lock_old || lock_new)
	{
		return PipePress + 0.00001;
	}
	return ScndPipePress;
}

// the position the driver's handle works at: its own, or the one the speed control's brake intervention sets
double TMoverParameters::MainValveHandlePosition() const
{
	double pos = BrakeCtrlPosR;
	if (SpeedCtrlUnit.IsActive && SpeedCtrlUnit.BrakeIntervention && !SpeedCtrlUnit.Standby && BrakeCtrlPos != Handle->GetPos(bh_EB))
	{
		pos = Handle->GetPos(bh_NP);
		if (SpeedCtrlUnit.BrakeInterventionBraking)
			pos = Handle->GetPos(bh_FB);
		if (SpeedCtrlUnit.BrakeInterventionUnbraking)
			pos = Handle->GetPos(bh_RP);
	}
	return pos;
}

void TMoverParameters::UpdateEmergencyValve(double dt)
{
	// ulepszony hamulec bezp.
	EmergencyValveFlow = 0.0;


	// EngDmgFlag 32 (load destroyed) used to trigger this too; dropped, it has nothing to do with
	// the emergency brake (a broken coupler sets AlarmChainFlag instead). the Elf's emergency brake position
	// vents the pipe only from the vehicle driven from, whatever the controllers of the others show.
	// a vehicle prepared for hauling (PulleeMode) vents it for nothing of its own, only for the alarm chain,
	// which a train parting sets too
	auto const ownemergency{RadioStopFlag || (EIMCtrlEmergency && LocalBrakePosA >= 1.0) || (IsMainCtrlEmergencyPos() && IsCabMaster()) || SecuritySystem.is_braking() ||
	                        (SpringBrakeDriveEmergencyVel >= 0 && Vel > SpringBrakeDriveEmergencyVel && SpringBrake.IsActive) ||
	                        (0 == CabActive && (InactiveCabFlag & activation::emergencybrake) != 0)};
	if (AlarmChainFlag || (ownemergency && false == PulleeMode))
	{
		EmergencyValveFlow = PF(0, PipePress, 0.15) * dt;
	}
	dpMainValve += EmergencyValveFlow;
}

// what the distributor valve gets of the local brake and the load, by its kind
void TMoverParameters::UpdateDistributorSettings()
{
	switch (BrakeValve)
	{

	case TBrakeValve::K:
	case TBrakeValve::W:
	{

		if (BrakeLocHandle != TBrakeHandle::NoHandle)
		{
			LocBrakePress = LocHandle->GetCP();

			//(Hamulec as TWest).SetLBP(LocBrakePress);
			Hamulec->SetLBP(LocBrakePress);
		}
		if (MBPM < 2)
			//(Hamulec as TWest).PLC(MaxBrakePress[LoadFlag])
			Hamulec->PLC(MaxBrakePress[LoadFlag]);
		else
			//(Hamulec as TWest).PLC(TotalMass);
			Hamulec->PLC(TotalMass - Mred);
		break;
	}

	case TBrakeValve::LSt:
	case TBrakeValve::EStED:
	{

		LocBrakePress = LocHandle->GetCP();
		auto const et41or42 = [](train_type const Type) { return Type == train_type::et41 || Type == train_type::et42; };
		for (auto const &couplerentry : Couplers)
			// nie podoba mi się to rozwiązanie, chyba trzeba
			// dodać jakiś wpis do fizyki na to
			if (et41or42(TrainType) && couplerentry.Connected != nullptr && et41or42(couplerentry.Connected->TrainType) && (couplerentry.CouplingFlag & 36) == 36)
				LocBrakePress = std::max(couplerentry.Connected->LocHandle->GetCP(), LocBrakePress);

		// if ((DynamicBrakeFlag) && (EngineType == ElectricInductionMotor))
		//{
		//     //if (Vel > 10)
		//     //    LocBrakePress = 0;
		//     //else if (Vel > 5)
		//     //    LocBrakePress = (10 - Vel) / 5 * LocBrakePress;
		// }

		//(Hamulec as TLSt).SetLBP(LocBrakePress);
		Hamulec->SetLBP(LocBrakePress);
		if (BrakeValve == TBrakeValve::EStED)
		{
			if (MBPM < 2)
				Hamulec->PLC(MaxBrakePress[LoadFlag]);
			else
				Hamulec->PLC(TotalMass - Mred);
		}
		break;
	}

	case TBrakeValve::CV1_L_TR:
	{
		LocBrakePress = LocHandle->GetCP();
		//(Hamulec as TCV1L_TR).SetLBP(LocBrakePress);
		Hamulec->SetLBP(LocBrakePress);
		break;
	}

	case TBrakeValve::EP2:
	case TBrakeValve::EP1:
	{
		Hamulec->PLC(TotalMass - Mred);
		break;
	}
	case TBrakeValve::ESt3AL2:
	case TBrakeValve::NESt3:
	case TBrakeValve::ESt4:
	case TBrakeValve::ESt3:
	{
		if (MBPM < 2)
			//(Hamulec as TNESt3).PLC(MaxBrakePress[LoadFlag])
			Hamulec->PLC(MaxBrakePress[LoadFlag]);
		else
			//(Hamulec as TNESt3).PLC(TotalMass);
			Hamulec->PLC(TotalMass - Mred);
		LocBrakePress = LocHandle->GetCP();
		//(Hamulec as TNESt3).SetLBP(LocBrakePress);
		Hamulec->SetLBP(LocBrakePress);
		break;
	}
	case TBrakeValve::KE:
	{
		LocBrakePress = LocHandle->GetCP();
		//(Hamulec as TKE).SetLBP(LocBrakePress);
		Hamulec->SetLBP(LocBrakePress);
		if (MBPM < 2)
			//(Hamulec as TKE).PLC(MaxBrakePress[LoadFlag])
			Hamulec->PLC(MaxBrakePress[LoadFlag]);
		else
			//(Hamulec as TKE).PLC(TotalMass);
			Hamulec->PLC(TotalMass - Mred);
		break;
	}
	default:
	{
		// unsupported brake valve type, we should never land here
		//            ErrorLog( "Unsupported brake valve type (" + std::to_string( BrakeValve ) + ") in " + TypeName );
		//            ::PostQuitMessage( 0 );
		break;
	}
	} // switch
}

// the ep brake of the FVel6 and FVE408 handles, passed along the unit
void TMoverParameters::UpdateEPBrake()
{
	if (EPBrakeControlPowered()) // tu powinien byc jeszcze bezpiecznik EP i baterie -
		// temp = (Handle as TFVel6).GetCP
		EpForce = Handle->GetEP();
	else
		EpForce = 0.0;

	DynamicBrakeEMUStatus = EpForce > 0.001 ? Power110vIsAvailable : true;

	double temp1 = EpForce;
	if (DCEMUED_EP_max_Vel > 0.001 && Vel > DCEMUED_EP_max_Vel && DynamicBrakeEMUStatus)
		temp1 = 0;
	if (DCEMUED_EP_min_Im > 0.001 && abs(Im) > DCEMUED_EP_min_Im && DynamicBrakeEMUStatus)
		temp1 = 0;
	Hamulec->SetEPS(temp1);
	TUHEX_StageActual = static_cast<int>(EpForce);
	TUHEX_Active = TUHEX_StageActual > 0;
	// Ra 2014-11: na tym się wysypuje, ale nie wiem, w jakich warunkach
	SendCtrlToNext("Brake", EpForce, CabActive);
}

// *************************************************************************************************
// Q: 20160713
// Aktualizacja ciśnienia w przewodzie zasilającym
// *************************************************************************************************
void TMoverParameters::UpdateScndPipePressure(double dt)
{
	if (ScndPipePress > 1.0)
	{
		Pipe2->Flow(-ScndPipePress*AirLeakRate * dt);
		Pipe2->Act();
	}

	const double Spz = 0.5067;

	UpdateSpringBrake(dt);

	Pipe2->Flow(Hamulec->GetHPFlow(ScndPipePress, dt));
	// NOTE: condition disabled to allow the air flow from the main hose to the main tank as well
	if (/* ( ( Compressor > ScndPipePress ) && ( */ VeselVolume > 0.0 /* ) ) || ( TrainType == train_type::emu ) || ( TrainType == train_type::dmu ) */)
	{
		double const dV = PF(Compressor, ScndPipePress, Spz) * dt;
		CompressedVolume += dV / 1000.0;
		Pipe2->Flow(-dV);
	}
	// the air the main reservoir hoses carry to and from the neighbours comes once all vehicles have stepped, UpdatePipeCouplers()
	ScndPipeCouplersPending = true;
	Pipe2->Act();
	ScndPipePress = Pipe2->P();

	if (ScndPipePress < -1)
	{
		ScndPipePress = -1;
		Pipe2->CreatePress(-1);
		Pipe2->Act();
	}
}

// *************************************************************************************************
// yB: 20190906
// Aktualizacja ciśnienia w hamulcu sprezynowym
// *************************************************************************************************
void TMoverParameters::UpdateSpringBrake(double /*dt*/)
{
	double BP = SpringBrake.PNBrakeConnection ? BrakePress : 0;
	double MSP = SpringBrake.ShuttOff ? 0 : SpringBrake.MaxSetPressure;
	if (!SpringBrake.Activate)
	{
		double desired_press = std::min(std::max(MSP, BP), Pipe2->P());
		double dv = PF(desired_press, SpringBrake.SBP, SpringBrake.ValveOffArea);
		SpringBrake.Cylinder->Flow(-dv);
		Pipe2->Flow(std::max(dv, 0.0));
	}
	else
	{
		double dv = PF(BP, SpringBrake.SBP, SpringBrake.ValveOnArea);
		SpringBrake.Cylinder->Flow(-dv);
	}
	if (SpringBrake.SBP > SpringBrake.ResetPressure)
		SpringBrake.IsReady = true;

	SpringBrake.IsActive = SpringBrake.SBP < (SpringBrake.IsActive ? SpringBrake.PressureOff : SpringBrake.PressureOn);

	SpringBrake.Release = false;
	SpringBrake.Cylinder->Act();
	SpringBrake.SBP = SpringBrake.Cylinder->P();
}
namespace
{

// whether the air flowing to a coupled vehicle along a pipe wakes its physics: any flow did, but a vehicle standing next to one
// whose physics is on gets a trickle of it all the time, as their pressures settle, and each wake kept it on for 5 s. with
// Global.physics_pipewake only a flow which leaves its pressure, with what has flown in while it slept, that much away from the
// pressure of the vehicle the air comes from, or which has changed it by that much since it fell asleep: a pressure followed in
// small steps would never reach the valves of a vehicle asleep otherwise
bool pipe_wakes(double const Flow, double const Pressure, TReservoir const &Pipe)
{
	if (Flow * Flow <= 0.00000000000001)
		return false;
	return (Global.physics_pipewake <= 0.f)
	    || (std::abs(Pressure - Pipe.P_pending()) > Global.physics_pipewake)
	    || (std::abs(Pipe.P_pending() - Pipe.P_applied()) > Global.physics_pipewake);
}

// the pressure in the pipe of a coupled vehicle the flow to it goes by. the pressure the vehicle keeps is the one of its last step:
// with Global.physics_pipewake one asleep can take air for a long time without waking, and the flow, going by that pressure, would
// carry on past the pressure of the vehicle it comes from. so of one asleep, the pressure with what has flown in since
double neighbour_pipe_pressure(TMoverParameters const &Neighbour, double const Pressure, TReservoir const &Pipe)
{
	if ((Global.physics_pipewake <= 0.f) || Neighbour.PhysicActivation)
		return Pressure;
	return Pipe.P_pending();
}

} // namespace

// *************************************************************************************************
// Q: 20160715
// oblicza i zwraca przepływ powietrza pomiędzy pojazdami
// *************************************************************************************************
double TMoverParameters::GetDVc(double dt)
{
	// T_MoverParameters *c;
	TMoverParameters *c;
	double dv1;
	double dv2; // , dV;

	dv1 = 0;
	dv2 = 0;
	// sprzeg 1
	if (Couplers[0].Connected != nullptr && TestFlag(Couplers[0].CouplingFlag, ctrain_pneumatic))
	{ //*0.85
		c = Couplers[0].Connected; // skrot           //0.08           //e/D * L/D = e/D^2 * L
		dv1 = 0.5 * dt * PF(PipePress, neighbour_pipe_pressure(*c, c->PipePress, *c->Pipe), Spg / (1.0 + 0.015 / Spg * Dim.L));
		c->Pipe->Flow(-dv1);
		if (pipe_wakes(dv1, PipePress, *c->Pipe))
			c->switch_physics(true, wake_cause::mainpipe);
	}
	// sprzeg 2
	if (Couplers[1].Connected != nullptr && TestFlag(Couplers[1].CouplingFlag, ctrain_pneumatic))
	{
		c = Couplers[1].Connected; // skrot
		dv2 = 0.5 * dt * PF(PipePress, neighbour_pipe_pressure(*c, c->PipePress, *c->Pipe), Spg / (1.0 + 0.015 / Spg * Dim.L));
		c->Pipe->Flow(-dv2);
		if (pipe_wakes(dv2, PipePress, *c->Pipe))
			c->switch_physics(true, wake_cause::mainpipe);
	}
	// if ((Couplers[1].Connected != NULL) && (Couplers[0].Connected != NULL))
	//     if ((TestFlag(Couplers[0].CouplingFlag, ctrain_pneumatic)) &&
	//         (TestFlag(Couplers[1].CouplingFlag, ctrain_pneumatic)))
	//     {
	//         dV = 0.05 * dt * PF(Couplers[0].Connected->PipePress, Couplers[1].Connected->PipePress,
	//                             (Spg * 0.85) / (1 + 0.03 * Dim.L)) *
	//              0; // ktoś mi powie jaki jest sens tego bloku jeśli przepływ mnożony przez zero?
	//         Couplers[0].Connected->Pipe->Flow(+dV);
	//         Couplers[1].Connected->Pipe->Flow(-dV);
	//     }
	//  suma
	return dv2 + dv1;
}

// *************************************************************************************************
// the air the main reservoir hoses carry between the vehicle's pipe and the neighbours' in dt: the neighbours'
// share is sent to them, the vehicle's own returned
// *************************************************************************************************
double TMoverParameters::GetScndDVc(double dt)
{
	const double Spz = 0.5067;
	TMoverParameters *c;
	double dv1 = 0;
	double dv2 = 0;

	// sprzeg 1
	if (Couplers[0].Connected != nullptr && TestFlag(Couplers[0].CouplingFlag, ctrain_scndpneumatic))
	{
		c = Couplers[0].Connected; // skrot
		dv1 = 0.5 * dt * PF(ScndPipePress, neighbour_pipe_pressure(*c, c->ScndPipePress, *c->Pipe2), Spz * 0.75);
		c->Pipe2->Flow(-dv1);
		if (pipe_wakes(dv1, ScndPipePress, *c->Pipe2))
			c->switch_physics(true, wake_cause::scndpipe);
	}
	// sprzeg 2
	if (Couplers[1].Connected != nullptr && TestFlag(Couplers[1].CouplingFlag, ctrain_scndpneumatic))
	{
		c = Couplers[1].Connected; // skrot
		dv2 = 0.5 * dt * PF(ScndPipePress, neighbour_pipe_pressure(*c, c->ScndPipePress, *c->Pipe2), Spz * 0.75);
		c->Pipe2->Flow(-dv2);
		if (pipe_wakes(dv2, ScndPipePress, *c->Pipe2))
			c->switch_physics(true, wake_cause::scndpipe);
	}
	if (Couplers[1].Connected != nullptr && Couplers[0].Connected != nullptr && TestFlag(Couplers[0].CouplingFlag, ctrain_scndpneumatic) && TestFlag(Couplers[1].CouplingFlag, ctrain_scndpneumatic))
	{
		double const dV = 0.00025 * dt * PF(Couplers[0].Connected->ScndPipePress, Couplers[1].Connected->ScndPipePress, Spz * 0.25);
		Couplers[0].Connected->Pipe2->Flow(+dV);
		Couplers[1].Connected->Pipe2->Flow(-dV);
	}
	return dv1 + dv2;
}
namespace
{


// a pipe along the train, whose air the hoses between the vehicles carry (TMoverParameters::UpdatePipeCouplers())
struct train_pipe
{
	std::shared_ptr<TReservoir> TMoverParameters::*reservoir;
	double TMoverParameters::*pressure;
	bool TMoverParameters::*pending; // the vehicle stepped the pipe, the exchange through its hoses is yet to come
	int hose; // the coupling flag of the pipe's hose
	double (TMoverParameters::*exchange)(double); // the air the hoses carry in a step, the neighbours' share sent to them
	double (*area)(TMoverParameters const &); // the cross-section of the hose, as the exchange takes it
};

// the brake pipe, GetDVc()
train_pipe const brake_pipe{&TMoverParameters::Pipe,
                            &TMoverParameters::PipePress,
                            &TMoverParameters::PipeCouplersPending,
                            ctrain_pneumatic,
                            &TMoverParameters::GetDVc,
                            [](TMoverParameters const &Vehicle) { return Vehicle.Spg / (1.0 + 0.015 / Vehicle.Spg * Vehicle.Dim.L); }};
// the main reservoir pipe, GetScndDVc()
train_pipe const main_reservoir_pipe{&TMoverParameters::Pipe2,
                                     &TMoverParameters::ScndPipePress,
                                     &TMoverParameters::ScndPipeCouplersPending,
                                     ctrain_scndpneumatic,
                                     &TMoverParameters::GetScndDVc,
                                     [](TMoverParameters const &) { return 0.5067 * 0.75; }};

// the number of sub-steps of dt the air exchange through the hoses of the vehicle's pipe needs: in one, the air a
// hose carries stays below what evens out the pipes on its ends
int hose_substeps(TMoverParameters const &Vehicle, train_pipe const &Pipe, double const dt)
{
	auto substeps{1};
	auto const area{Pipe.area(Vehicle)};
	auto const capacity{(Vehicle.*Pipe.reservoir)->GetCap()};
	for (auto const &hose : Vehicle.Couplers)
	{
		if (hose.Connected == nullptr || false == TestFlag(hose.CouplingFlag, Pipe.hose))
		{
			continue;
		}
		auto const &other{*hose.Connected};
		auto const othercapacity{(other.*Pipe.reservoir)->GetCap()};
		// the flow per bar of the difference, at small differences, where it's the largest
		auto const pressure{std::max(Vehicle.*Pipe.pressure, other.*Pipe.pressure)};
		auto const conductance{std::abs(PF(pressure, pressure - 0.01, area)) / 0.01};
		auto const equalising{capacity * othercapacity / (capacity + othercapacity)};
		substeps = std::max(substeps, static_cast<int>(std::ceil(dt * conductance / equalising)));
	}
	return std::min(substeps, 10);
}

// the vehicles of the list which stepped the pipe, each with the number of the train it is in: the ones joined by the hoses
// of the pipe, directly, numbered in the order of their first vehicle in the list. a vehicle which didn't step the pipe
// (its physics sleeps) takes the air sent to it without passing it on, so it ends a train
std::vector<std::size_t> hose_trains(std::vector<TMoverParameters *> const &Vehicles, train_pipe const &Pipe)
{
	auto constexpr none{std::numeric_limits<std::size_t>::max()};
	std::vector<std::size_t> trains(Vehicles.size(), none);
	// the vehicles by their address, for finding the neighbours in the list
	std::vector<std::pair<TMoverParameters const *, std::size_t>> byaddress;
	byaddress.reserve(Vehicles.size());
	for (std::size_t index = 0; index < Vehicles.size(); ++index)
	{
		if (Vehicles[index]->*Pipe.pending)
		{
			byaddress.emplace_back(Vehicles[index], index);
		}
	}
	std::ranges::sort(byaddress);
	auto const find = [&byaddress](TMoverParameters const *Vehicle) {
		auto const found{std::ranges::lower_bound(byaddress, Vehicle, std::less<>{}, &std::pair<TMoverParameters const *, std::size_t>::first)};
		return (found != byaddress.end() && found->first == Vehicle) ? found->second : none;
	};
	std::size_t count{0};
	std::vector<std::size_t> stack;
	for (std::size_t first = 0; first < Vehicles.size(); ++first)
	{
		if (false == Vehicles[first]->*Pipe.pending || trains[first] != none)
		{
			continue;
		}
		trains[first] = count;
		stack.assign(1, first);
		while (false == stack.empty())
		{
			auto const &vehicle{*Vehicles[stack.back()]};
			stack.pop_back();
			for (auto const &hose : vehicle.Couplers)
			{
				if (hose.Connected == nullptr || false == TestFlag(hose.CouplingFlag, Pipe.hose))
				{
					continue;
				}
				if (auto const other{find(hose.Connected)}; other != none && trains[other] == none)
				{
					trains[other] = count;
					stack.push_back(other);
				}
			}
		}
		++count;
	}
	return trains;
}

// the air exchange through the hoses of the pipe, for the vehicles which stepped it since the last call, in the order
// they stepped: each train in as many sub-steps of dt as its vehicle needing the most of them takes, each going through
// its vehicles one after another, as the step of the vehicles does. the air of a train stays in it, so a train comes out
// the same whichever other vehicles are exchanged in the same call: all of them, or the ones of a group of
// vehicle_table::update_steps_parallel()
void exchange_through_hoses(std::vector<TMoverParameters *> const &Vehicles, train_pipe const &Pipe, double const dt)
{
	auto const trains{hose_trains(Vehicles, Pipe)};
	std::vector<int> substeps;
	for (std::size_t index = 0; index < Vehicles.size(); ++index)
	{
		if (false == Vehicles[index]->*Pipe.pending)
		{
			continue;
		}
		if (substeps.size() <= trains[index])
		{
			substeps.resize(trains[index] + 1, 1);
		}
		substeps[trains[index]] = std::max(substeps[trains[index]], hose_substeps(*Vehicles[index], Pipe, dt));
	}
	for (std::size_t train = 0; train < substeps.size(); ++train)
	{
		auto const substep{dt / substeps[train]};
		for (auto i = 0; i < substeps[train]; ++i)
		{
			for (std::size_t index = 0; index < Vehicles.size(); ++index)
			{
				if (false == Vehicles[index]->*Pipe.pending || trains[index] != train)
				{
					continue;
				}
				auto *vehicle{Vehicles[index]};
				auto &pipe{*(vehicle->*Pipe.reservoir)};
				pipe.Act(); // what the neighbours have sent in the meantime
				vehicle->*Pipe.pressure = pipe.P();
				pipe.Flow((vehicle->*Pipe.exchange)(substep));
				pipe.Act();
				vehicle->*Pipe.pressure = pipe.P();
			}
		}
	}
	for (auto *vehicle : Vehicles)
	{
		vehicle->*Pipe.pending = false;
	}
}

} // namespace

// *************************************************************************************************
// the number of sub-steps of dt the air exchange through the brake hoses of the vehicle needs. taken in a single
// step of 0.01 s, the hose between two 14 m freight wagons carries twice the air which evens out their pipes, the
// pipes overshoot each other in turns and the distributors see the pressure up to 0.28 bar off what's in the pipe
// (eu07/maszyna#368)
// *************************************************************************************************
int TMoverParameters::PipeCouplersSubsteps(double const dt) const
{
	return hose_substeps(*this, brake_pipe, dt);
}

// *************************************************************************************************
// the number of sub-steps of dt the air exchange through the main reservoir hoses of the vehicle needs. taken in a
// single step of 0.01 s, the hose carries about 1.5 times the air which evens out the pipes on its ends; along with
// the vehicles' own main reservoirs feeding the pipe in the same step, the pipes of the middle vehicles of a
// multiple unit or of locomotives in multiple traction kept jumping 0.8 bar to and fro between the steps
// *************************************************************************************************
int TMoverParameters::ScndPipeCouplersSubsteps(double const dt) const
{
	return hose_substeps(*this, main_reservoir_pipe, dt);
}

// *************************************************************************************************
// the air exchange through the brake and the main reservoir hoses of the vehicles which stepped their pipes
// (UpdatePipePressure(), UpdateScndPipePressure()) since the last call, each pipe in its own sub-steps
// *************************************************************************************************
void TMoverParameters::UpdatePipeCouplers(std::vector<TMoverParameters *> const &Vehicles, double const dt)
{
	exchange_through_hoses(Vehicles, brake_pipe, dt);
	exchange_through_hoses(Vehicles, main_reservoir_pipe, dt);
}

bool TMoverParameters::UpdateStopBrakeRelease(double const Demand, double const Dt)
{
	if (Demand <= 0.0)
	{
		StopBrakeReleaseTimer = 0.0;
		return false;
	}
	// the step the demand rises above 0 is time 0 of it, so without the delay the brake lets go at once, as it always did
	auto const released{StopBrakeReleaseTimer >= StopBrakeReleaseDelay};
	StopBrakeReleaseTimer += Dt;
	return released;
}
