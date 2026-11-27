/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// setting the vehicle up after reading its definition: the checks, the brake valves, the handles and the pipes

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


// *************************************************************************************************
// Q: 20160717
// *************************************************************************************************

bool TMoverParameters::CheckLocomotiveParameters(bool ReadyFlag, int /*Dir*/)
{
	WriteLog("check locomotive parameters...");
	bool OK = true;

	AutoRelayFlag = AutoRelayType == 1;

	if (NominalBatteryVoltage == 0.0)
	{
		BatteryStart = start_t::disabled;
	}

	Sand = SandCapacity;

	// NOTE: for diesel-powered vehicles we automatically convert legacy "main" power source to more accurate "engine"
	if (CompressorPower == 0 && (EngineType == TEngineType::DieselEngine || EngineType == TEngineType::DieselElectric))
	{
		CompressorPower = 3;
	}

	// for these engine types secondary controller positions index MotorParam table
	if ((EngineType == TEngineType::ElectricSeriesMotor || EngineType == TEngineType::DieselEngine || EngineType == TEngineType::DieselElectric) &&
	    ScndCtrlPosNo > MotorParametersArraySize)
	{
		WriteLog("SCPN=" + std::to_string(ScndCtrlPosNo) + " exceeds motor parameter table size, limited to " + std::to_string(MotorParametersArraySize));
		ScndCtrlPosNo = MotorParametersArraySize;
	}

	// WriteLog("aa = " + AxleArangement + " " + std::string( Pos("o", AxleArangement)) );

	if (contains(AxleArangement, "o") && EngineType == TEngineType::ElectricSeriesMotor)
	{
		// test poprawnosci ilosci osi indywidualnie napedzanych
		OK = RList[1].Bn * RList[1].Mn == NPoweredAxles;
		// WriteLogSS("aa ok", BoolToYN(OK));
	}

	if (BrakeSystem == TBrakeSystem::Individual && BrakeSubsystem != TBrakeSubSystem::ss_None)
		OK = false; //!

	if (BrakeVVolume == 0 && MaxBrakePress[3] > 0 && BrakeSystem != TBrakeSystem::Individual)
	{
		BrakeVVolume = MaxBrakePress[3] / (5.0 - MaxBrakePress[3]) * (BrakeCylRadius * BrakeCylRadius * BrakeCylDist * BrakeCylNo * M_PI) * 1000;
	}
	if (BrakeVVolume == 0.0)
	{
		BrakeVVolume = 0.01;
	}
	// WriteLog("BVV = "  + FloatToStr(BrakeVVolume));

	CreateBrakeValve();

	CreateBrakeHandles();

	CreateBrakePipes();

	if (LightsPosNo > 0)
		LightsPos = LightsDefPos;

	// set default wiper switch position
	wiperSwitchPos = WiperDefaultPos;

	if (CompressorListPosNo > 0)
		CompressorListPos = CompressorListDefPos;

	// the genset of a dual-mode vehicle is a second source of the traction circuit of an induction motor vehicle
	if (Genset.fitted() && (EngineType != TEngineType::ElectricInductionMotor || EnginePowerSource.SourceType != TPowerSource::CurrentCollector))
	{
		WriteLog("GensetPower ignored: a genset works only with EngineType=ElectricInductionMotor and EnginePower=CurrentCollector");
		Genset = genset_device{};
	}

	// NOTE: legacy compatibility behaviour for vehicles without defined heating power source
	if (EnginePowerSource.SourceType == TPowerSource::CurrentCollector && HeatingPowerSource.SourceType == TPowerSource::NotDefined)
	{
		HeatingPowerSource.SourceType = TPowerSource::Main;
	}
	if (HeatingPowerSource.SourceType == TPowerSource::NotDefined && HeatingPower > 0)
	{
		HeatingPowerSource.SourceType = TPowerSource::PowerCable;
		HeatingPowerSource.PowerType = TPowerType::ElectricPower;
	}

	// checking ready flag
	// to dac potem do init
	if (ReadyFlag) // gotowy do drogi
	{
		SetReadyToDepart();
	}
	else
	{ // zahamowany}
		SetBraked();
	}

	ActFlowSpeed = 0.0;
	BrakeCtrlPosR = BrakeCtrlPos;

	if (BrakeLocHandle == TBrakeHandle::Knorr)
		LocalBrakePosA = 0.5;

	Pipe->CreatePress(PipePress);
	Pipe2->CreatePress(ScndPipePress);
	Pipe->Act();
	Pipe2->Act();

	EqvtPipePress = PipePress;

	Handle->Init(PipePress);

	ComputeConstans();

	if (LoadFlag > 0)
	{

		if (LoadAmount < MaxLoad * 0.45)
		{
			IncBrakeMult();
			IncBrakeMult();
			DecBrakeMult(); // TODO: przeinesiono do mover.cpp
			if (LoadAmount < MaxLoad * 0.35)
				DecBrakeMult();
		}
		else
		{
			IncBrakeMult(); // TODO: przeinesiono do mover.cpp
			if (LoadAmount >= MaxLoad * 0.55)
				IncBrakeMult();
		}
	}

	// taki mini automat - powinno byc ladnie dobrze :)
	BrakeDelayFlag = bdelay_P;
	if (TestFlag(BrakeDelays, bdelay_G) && !TestFlag(BrakeDelays, bdelay_R))
		BrakeDelayFlag = bdelay_G;
	if (TestFlag(BrakeDelays, bdelay_R) && !TestFlag(BrakeDelays, bdelay_G))
		BrakeDelayFlag = bdelay_R;
	/*
	// disabled, as test mode is used in specific situations and not really a default
	    if (BrakeOpModes & bom_PS)
	        BrakeOpModeFlag = bom_PS;
	    else
	*/
	BrakeOpModeFlag = bom_PN;

	// yB: jesli pojazdy nie maja zadeklarowanych czasow, to wsadz z przepisow +-16,(6)%
	std::array<int, 8> const DefBrakeTable{15, 4, 25, 25, 13, 3, 12, 2};

	for (int b = 1; b < 4; b++)
	{
		if (BrakeDelay[b] == 0)
			BrakeDelay[b] = DefBrakeTable[b];
		BrakeDelay[b] = BrakeDelay[b] * (2.5 + Random(0.0, 0.2)) / 3.0;
	}

	Hamulec->Init(PipePress, HighPipePress, LowPipePress, BrakePress, BrakeDelayFlag);
	/*
	    ScndPipePress = Compressor;
	*/
	// WriteLogSS("OK=", BoolTo10(OK));
	// WriteLog("");

	if (EIMCtrlType == 3)
	{
		for (auto idx = 0; idx < MainCtrlPosNo; ++idx)
		{
			if (UniCtrlList[idx].MaxCtrlVal > 0.0)
			{
				UniCtrlNoPowerPos = std::max(0, idx - 1);
				break;
			}
		}
	}

	// security system
	// by default place the magnet in the vehicle centre
	if (SecuritySystem.MagnetLocation == 0)
	{
		SecuritySystem.MagnetLocation = Dim.L / 2 - 0.5;
	}
	SecuritySystem.MagnetLocation = safe_clamp(SecuritySystem.MagnetLocation, 0.0, Dim.L);

	return OK;
}

// the distributor valve of the vehicle's kind
void TMoverParameters::CreateBrakeValve()
{
	switch (BrakeValve)
	{
	case TBrakeValve::W:
	case TBrakeValve::K:
	{
		WriteLog("XBT W, K");
		Hamulec = std::make_shared<TWest>(MaxBrakePress[3], BrakeCylRadius, BrakeCylDist, BrakeVVolume, BrakeCylNo, BrakeDelays, BrakeMethod, NAxles, NBpA);
		if (MBPM < 2) // jesli przystawka wazaca
			Hamulec->SetLP(0, MaxBrakePress[3], 0);
		else
			Hamulec->SetLP(Mass, MBPM, MaxBrakePress[1]);
		break;
	}
	case TBrakeValve::KE:
	{
		WriteLog("XBT WKE");
		Hamulec = std::make_shared<TKE>(MaxBrakePress[3], BrakeCylRadius, BrakeCylDist, BrakeVVolume, BrakeCylNo, BrakeDelays, BrakeMethod, NAxles, NBpA);
		Hamulec->SetRM(RapidMult);
		if (MBPM < 2) // jesli przystawka wazaca
			Hamulec->SetLP(0, MaxBrakePress[3], 0);
		else
			Hamulec->SetLP(Mass, MBPM, MaxBrakePress[1]);
		break;
	}
	case TBrakeValve::NESt3:
	case TBrakeValve::ESt3:
	case TBrakeValve::ESt3AL2:
	case TBrakeValve::ESt4:
	{
		WriteLog("XBT NESt3, ESt3, ESt3AL2, ESt4");
		Hamulec = std::make_shared<TNESt3>(MaxBrakePress[3], BrakeCylRadius, BrakeCylDist, BrakeVVolume, BrakeCylNo, BrakeDelays, BrakeMethod, NAxles, NBpA);
		static_cast<TNESt3 *>(Hamulec.get())->SetSize(BrakeValveSize, BrakeValveParams);
		if (MBPM < 2) // jesli przystawka wazaca
			Hamulec->SetLP(0, MaxBrakePress[3], 0);
		else
			Hamulec->SetLP(Mass, MBPM, MaxBrakePress[1]);
		break;
	}
	case TBrakeValve::LSt:
	{
		WriteLog("XBT LSt");
		Hamulec = std::make_shared<TLSt>(MaxBrakePress[3], BrakeCylRadius, BrakeCylDist, BrakeVVolume, BrakeCylNo, BrakeDelays, BrakeMethod, NAxles, NBpA);
		Hamulec->SetRM(RapidMult);
		break;
	}
	case TBrakeValve::EStED:
	{
		WriteLog("XBT EStED");
		Hamulec = std::make_shared<TEStED>(MaxBrakePress[3], BrakeCylRadius, BrakeCylDist, BrakeVVolume, BrakeCylNo, BrakeDelays, BrakeMethod, NAxles, NBpA);
		Hamulec->SetRM(RapidMult);
		if (MBPM < 2)
		{
			// jesli przystawka wazaca
			Hamulec->SetLP(0, MaxBrakePress[3], 0);
		}
		else
		{
			Hamulec->SetLP(Mass, MBPM, MaxBrakePress[1]);
		}
		break;
	}
	case TBrakeValve::EP2:
	{
		WriteLog("XBT EP2");
		Hamulec = std::make_shared<TEStEP2>(MaxBrakePress[3], BrakeCylRadius, BrakeCylDist, BrakeVVolume, BrakeCylNo, BrakeDelays, BrakeMethod, NAxles, NBpA);
		Hamulec->SetLP(Mass, MBPM, MaxBrakePress[1]);
		break;
	}
	case TBrakeValve::EP1:
	{
		WriteLog("XBT EP1");
		Hamulec = std::make_shared<TEStEP1>(MaxBrakePress[3], BrakeCylRadius, BrakeCylDist, BrakeVVolume, BrakeCylNo, BrakeDelays, BrakeMethod, NAxles, NBpA);
		Hamulec->SetLP(Mass, MBPM, MaxBrakePress[1]);
		Hamulec->SetRM(RapidMult);
		break;
	}
	case TBrakeValve::CV1:
	{
		WriteLog("XBT CV1");
		Hamulec = std::make_shared<TCV1>(MaxBrakePress[3], BrakeCylRadius, BrakeCylDist, BrakeVVolume, BrakeCylNo, BrakeDelays, BrakeMethod, NAxles, NBpA);
		break;
	}
	case TBrakeValve::CV1_L_TR:
	{
		WriteLog("XBT CV1_L_T");
		Hamulec = std::make_shared<TCV1L_TR>(MaxBrakePress[3], BrakeCylRadius, BrakeCylDist, BrakeVVolume, BrakeCylNo, BrakeDelays, BrakeMethod, NAxles, NBpA);
		break;
	}
	default:
		Hamulec = std::make_shared<TBrake>(MaxBrakePress[3], BrakeCylRadius, BrakeCylDist, BrakeVVolume, BrakeCylNo, BrakeDelays, BrakeMethod, NAxles, NBpA);
	}

	Hamulec->SetASBP(MaxBrakePress[4]);
	Hamulec->SetRV(RapidVel);
}

// the driver's handle and the local brake handle of the vehicle's kinds
void TMoverParameters::CreateBrakeHandles()
{
	switch (BrakeHandle)
	{
	using enum TBrakeHandle;
	case FV4a:
		Handle = std::make_shared<TFV4aM>();
		break;
	case MHZ_EN57:
	case MHZ_K8P:
		Handle = std::make_shared<TMHZ_EN57>();
		break;
	case FVel6:
		Handle = std::make_shared<TFVel6>();
		break;
	case FVE408:
		Handle = std::make_shared<TFVE408>();
		break;
	case testH:
		Handle = std::make_shared<Ttest>();
		break;
	case M394:
		Handle = std::make_shared<TM394>();
		break;
	case Knorr:
		Handle = std::make_shared<TH14K1>();
		break;
	case St113:
		Handle = std::make_shared<TSt113>();
		break;
	case MHZ_K5P:
		Handle = std::make_shared<TMHZ_K5P>();
		break;
	case MHZ_6P:
		Handle = std::make_shared<TMHZ_6P>();
		break;
	default:
		Handle = std::make_shared<TDriverHandle>();
	}
	Handle->SetParams(Handle_AutomaticOverload, Handle_ManualOverload, Handle_GenericDoubleParameter1, Handle_GenericDoubleParameter2, Handle_OverloadMaxPressure, Handle_OverloadPressureDecrease);

	switch (BrakeLocHandle)
	{
	case TBrakeHandle::FD1:
	{
		LocHandle = std::make_shared<TFD1>();
		LocHandle->Init(MaxBrakePress[0]);
		if (TrainType == train_type::emu)
		{

			dynamic_cast<TFD1 *>(LocHandle.get())->SetSpeed(3.5);
		}
		break;
	}
	case TBrakeHandle::Knorr:
	{
		LocHandle = std::make_shared<TH1405>();
		LocHandle->Init(MaxBrakePress[0]);
		break;
	}
	default:
		LocHandle = std::make_shared<TDriverHandle>();
	}
}

// the brake pipe, the main pipe and the spring brake's cylinder
void TMoverParameters::CreateBrakePipes()
{
	if (true == TestFlag(BrakeDelays, bdelay_G) && (false == TestFlag(BrakeDelays, bdelay_R) || Power > 1.0)) // ustalanie srednicy przewodu glownego (lokomotywa lub napędowy
		Spg = 0.792;
	else
		Spg = 0.507;
	// WriteLog("SPG = " + FloatToStr(Spg));

	Pipe = std::make_shared<TReservoir>();
	Pipe2 = std::make_shared<TReservoir>(); // zabezpieczenie, bo sie PG wywala... :(
	Pipe->CreateCap((std::max(Dim.L, 14.0) + 0.5) * Spg * 1); // dlugosc x przekroj x odejscia i takie tam
	Pipe2->CreateCap((std::max(Dim.L, 14.0) + 0.5) * Spg * 1);
	if (!SpringBrake.Cylinder)
	{
		SpringBrake.Cylinder = std::make_shared<TReservoir>();
		SpringBrake.Cylinder->CreateCap(1);
	}
}

// the pressure of the main pipe at the start: the main reservoir's, or HosePressure fed through a main hose, or none
double TMoverParameters::InitialScndPipePress(double const HosePressure) const
{
	if (VeselVolume > 0.0)
	{
		return CompressedVolume / VeselVolume;
	}
	if ((Couplers[end::front].AllowedFlag & coupling::mainhose) != 0 || (Couplers[end::rear].AllowedFlag & coupling::mainhose) != 0)
	{
		return HosePressure;
	}
	return 0.0;
}

// the air and the controls of a vehicle ready to depart
void TMoverParameters::SetReadyToDepart()
{
	WriteLog("Ready to depart");
	CompressedVolume = VeselVolume * MinCompressor * 9.8 / 10.0;
	ScndPipePress = InitialScndPipePress(5.0);
	PipePress = CntrlPipePress;
	BrakePress = 0.0;
	LocalBrakePosA = 0.0;
	if (CabActive == 0)
		BrakeCtrlPos = static_cast<int>(Handle->GetPos(bh_NP));
	else
		BrakeCtrlPos = static_cast<int>(Handle->GetPos(bh_RP));
	/*
	        // NOTE: disabled and left up to the driver, if there's any
	        MainSwitch( false );
	        PantFront( true );
	        PantRear( true );
	        MainSwitch( true );
	*/
	DirActive = 0; // Dir; //nastawnik kierunkowy - musi być ustawiane osobno!
	DirAbsolute = DirActive * CabActive; // kierunek jazdy względem sprzęgów
	LimPipePress = CntrlPipePress;

	Battery = BatteryStart != start_t::disabled;
}

// the air and the brakes of a vehicle standing braked
void TMoverParameters::SetBraked()
{
	WriteLog("Braked");
	Volume = BrakeVVolume * MaxBrakePress[3];
	CompressedVolume = VeselVolume * MinCompressor * 0.55;
	/*
	        ScndPipePress = 5.1;
	*/
	ScndPipePress = InitialScndPipePress(5.1);
	PipePress = LowPipePress;
	PipeBrakePress = MaxBrakePress[3] * 0.5;
	BrakePress = MaxBrakePress[3] * 0.5;
	LocalBrakePosA = 0.0;
	BrakeCtrlPos = static_cast<int>(Handle->GetPos(bh_NP));
	LimPipePress = LowPipePress;
	if (LocalBrake == TLocalBrake::ManualBrake || MBrake == true)
	{
		IncManualBrakeLevel(ManualBrakePosNo);
	}
	if (SpringBrake.MaxBrakeForce > 0.0)
	{
		SpringBrake.Activate = true;
	}
}
