/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the vehicle as a body: construction, the forces on it, its movement along the track, adhesion and sand

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


std::vector<std::string> const TMoverParameters::eimc_labels = {"dfic: ", "dfmax:", "p:    ", "scfu: ", "cim:  ", "icif: ", "Uzmax:", "Uzh:  ", "DU:   ", "I0:   ", "fcfu: ",
                                                                "F0:   ", "a1:   ", "Pmax: ", "Fh:   ", "Ph:   ", "Vh0:  ", "Vh1:  ", "Imax: ", "abed: ", "eped: "};

std::vector<std::string> const TMoverParameters::eimv_labels = {
    "Fkrt:", "Fmax:", "ks:  ", "df:  ", "fp:  ", "Us:  ", "pole:", "Ic:  ", "If:  ", "M:   ", "Fr:  ", "Ipoj:", "Pm:  ", "Pe:  ", "eta: ", "fkr: ", "Uzsm:", "Pmax:", "Fzad:", "Imax:", "Fful:"};

int DirPatch(int Coupler1, int Coupler2)
{ // poprawka dla liczenia sil przy ustawieniu przeciwnym obiektow
	return Coupler1 != Coupler2 ? 1 : -1;
}
namespace
{


// physics.collisions new: a rail vehicle derailed the legacy way keeps the deceleration of the legacy model, g, as the
// friction of its weight, and the forces of the vehicles coupled to it act on it as on any other: the train behind it
// pushes it on, and it holds the train back. alone it stops as the legacy model has it. the legacy model stops it
// whatever pushes it, and the train behind it stopped at once against it, in a hit no train had
bool legacy_derailed_slides(TMoverParameters const &Vehicle)
{
	return Global.collisions == global_settings::collision_model::physical && TestFlag(Vehicle.CategoryFlag, 1) && TestFlag(Vehicle.DamageFlag, dtrain_out) &&
	       Vehicle.Derailed == derailment::outcome::none;
}

} // namespace

int DirF(int CouplerN)
{
	switch (CouplerN)
	{
	case 0:
		return -1;
	case 1:
		return 1;
	default:
		return 0;
	}
}

// *************************************************************************************************
//  główny konstruktor
// *************************************************************************************************
TMoverParameters::TMoverParameters(double VelInitial, std::string const &TypeNameInit, std::string const &NameInit, int Cab) : TypeName(TypeNameInit), Name(NameInit), CabOccupied(Cab)
{
	WriteLog("------------------------------------------------------");
	WriteLog("init default physic values for " + NameInit + ", [" + TypeNameInit + "]");

	// BrakeLevelSet(-2); //Pascal ustawia na 0, przestawimy na odcięcie (CHK jest jeszcze nie wczytane!)
	iLights[0] = 0;
	iLights[1] = 0; // światła zgaszone

	// inicjalizacja stalych
	for (auto &scheme : RList)
	{
		scheme = TScheme();
	}
	RlistSize = 0;
	for (auto &motorparam : MotorParam)
	{
		motorparam = TMotorParameters();
	}

	for (auto &cablights : Lights)
		for (int k = 0; k < 17; ++k)
			cablights[k] = 0;

	for (auto &compressorrow : CompressorList)
		for (int k = 1; k < 9; ++k)
			compressorrow[k] = 0;
	CompressorList[0][0] = 0.0;
	CompressorList[1][0] = 1.0;
	CompressorList[2][0] = 1.0;
	CompressorList[3][0] = 1.0;

	for (int b = -1; b <= MainBrakeMaxPos; ++b)
	{
		BrakePressureTable[b].PipePressureVal = 0.0;
		BrakePressureTable[b].BrakePressureVal = 0.0;
		BrakePressureTable[b].FlowSpeedVal = 0.0;
	}
	// with BrakePressureTable[-2] do  {pozycja odciecia}
	{
		BrakePressureTable[-2].PipePressureVal = -1.0;
		BrakePressureTable[-2].BrakePressureVal = -1.0;
		BrakePressureTable[-2].FlowSpeedVal = 0.0;
	}
	for (auto &delay : BrakeDelay)
	{
		delay = 0.0;
	}

	for (int b = 0; b < 2; ++b) // Ra: kto tu zrobił "for b:=1 to 2 do" ???
	{
		Couplers[b].CouplerType = TCouplerType::NoCoupler;
		Couplers[b].SpringKB = 1.0;
		Couplers[b].SpringKC = 1.0;
		Couplers[b].DmaxB = 0.1;
		Couplers[b].FmaxB = 1000.0;
		Couplers[b].DmaxC = 0.1;
		Couplers[b].FmaxC = 1000.0;
	}
	for (auto &cylindermult : BrakeCylMult)
	{
		cylindermult = 0.0;
	}

	for (auto &eimcvalue : eimc)
	{
		eimcvalue = 0.0;
	}
	eimc[eimc_p_eped] = 1.5;

	for (auto &couplerentry : Couplers)
	{
		couplerentry.AllowedFlag = 3; // domyślnie hak i hamulec, inne trzeba włączyć jawnie w FIZ
		couplerentry.CouplingFlag = 0;
		couplerentry.Connected = nullptr;
		couplerentry.ConnectedNr = 0; // Ra: to nie ma znaczenia jak nie podłączony
		couplerentry.Render = false;
		couplerentry.CForce = 0.0;
		couplerentry.Dist = 0.0;
		couplerentry.CheckCollision = false;
	}

	for (auto &brakepress : MaxBrakePress)
	{
		brakepress = 0.0;
	}

	Vel = abs(VelInitial);
	V = VelInitial / 3.6;

	for (auto &eimvvalue : eimv)
	{
		eimvvalue = 0.0;
	}

	RunningShape.Len = 1.0;

	RunningTrack.CategoryFlag = CategoryFlag;
	RunningTrack.Width = TrackW;
	RunningTrack.friction = Steel2Steel_friction;
	RunningTrack.QualityFlag = 20;
	RunningTrack.DamageFlag = 0;
	RunningTrack.Velmax = 100.0; // dla uzytku maszynisty w ai_driver}

	RunningTraction.TractionVoltage = 0.0;
	RunningTraction.TractionFreq = 0.0;
	RunningTraction.TractionMaxCurrent = 0.0;
	RunningTraction.TractionResistivity = 1.0;
};

void TMoverParameters::change_train_velocity(double const Change)
{
	// the vehicles coupled to this one in a line, each with the direction of its axis to the axis of this one
	std::vector<std::pair<TMoverParameters *, int>> train{{this, 1}};
	for (std::size_t i = 0; i < train.size(); ++i)
	{
		auto const [vehicle, direction] = train[i];
		for (int end = end::front; end <= end::rear; ++end)
		{
			auto const &coupler{vehicle->Couplers[end]};
			auto *other{coupler.Connected};
			if (other == nullptr || coupler.CouplingFlag == coupling::faux || std::any_of(train.begin(), train.end(), [other](auto const &Entry) { return Entry.first == other; }))
			{
				continue;
			}
			auto const otherdirection{direction * DirPatch(end, coupler.ConnectedNr)};
			other->V += Change * otherdirection;
			train.emplace_back(other, otherdirection);
		}
	}
}

// *************************************************************************************************
// Oblicza przemieszczenie taboru
// *************************************************************************************************
double TMoverParameters::ComputeMovement(double dt, double dt1, const TTrackShape &Shape, TTrackParam const &Track, TTractionParam const &ElectricTraction, TLocation const &NewLoc, TRotation const &NewRot)
{

	if (!TestFlag(DamageFlag, dtrain_out))
	{ // Ra: to przepisywanie tu jest bez sensu
		RunningShape = Shape;
		RunningTrack = Track;
		RunningTraction = ElectricTraction;

		// if (!DynamicBrakeFlag)
		//     RunningTraction.TractionVoltage = ElectricTraction.TractionVoltage /*-
		//                                       abs(ElectricTraction.TractionResistivity *
		//                                           (Itot + HVCouplers[0][0] + HVCouplers[1][0]))*/;
		// else
		//     RunningTraction.TractionVoltage =
		//         ElectricTraction.TractionVoltage /*-
		//         abs(ElectricTraction.TractionResistivity * Itot *
		//             0)*/; // zasadniczo ED oporowe nie zmienia napięcia w sieci
	}
	else if (OffTrack.active())
	{
		// off the track the grade is that of the ground under it, along its heading
		RunningShape.dHtrack = Shape.dHtrack;
	}

	if (CategoryFlag == 4)
		OffsetTrackV = TotalMass / (Dim.L * Dim.W * 1000.0);
	else if (TestFlag(CategoryFlag, 1) && TestFlag(RunningTrack.CategoryFlag, 1) && TestFlag(DamageFlag, dtrain_out))
	{
		OffsetTrackV = -0.2;
		OffsetTrackH = Sign(RunningShape.R) * 0.2;
	}

	// TODO: investigate, seems supplied NewRot is always 0 although the code here suggests some actual values are expected
	Loc = NewLoc;
	Rot = NewRot;

	if (dL == 0) // oblicz przesuniecie}
	{
		auto const AccSprev{AccS};
		// przyspieszenie styczne
		AccS = std::lerp(AccSprev, FTotal / TotalMass, 0.5);
		// std::clamp( dt * 3.0, 0.0, 1.0 ) ); // prawo Newtona ale z wygladzaniem (średnia z poprzednim)

		// a vehicle derailed by the physical criteria slides on with the friction of ComputeTotalForce(), and so does one
		// derailed the legacy way with physics.collisions new
		if (TestFlag(DamageFlag, dtrain_out) && Derailed == derailment::outcome::none && false == legacy_derailed_slides(*this))
			AccS = -Sign(V) * g * 1; // random(0.0, 0.1)
		// stuck in the ballast: the friction holds a derailed vehicle once it stopped, until the forces on it are larger
		auto const stuck{(Derailed != derailment::outcome::none || legacy_derailed_slides(*this)) && V == 0.0 && std::abs(FTrain) <= std::abs(FStand)};
		if (stuck)
			AccS = 0.0;

		// przyspieszenie normalne
		if (abs(Shape.R) > 0.01)
			AccN = square(V) / Shape.R + g * Shape.dHrail / TrackW; // Q: zamieniam SQR() na sqr()
		else
			AccN = g * Shape.dHrail / TrackW;

		// velocity change
		auto const Vprev{V};
		V += (3.0 * AccS - AccSprev) * dt / 2.0; // przyrost predkosci
		if (stuck)
			V = 0.0;
		if (V * Vprev <= 0 && std::abs(FStand) > std::abs(FTrain))
		{
			// tlumienie predkosci przy hamowaniu
			// zahamowany
			V = 0;
		}

		// tangential acceleration, from velocity change
		AccSVBased = std::lerp(AccSVBased, (V - Vprev) / dt, std::clamp(dt * 3.0, 0.0, 1.0));

		// vertical acceleration
		AccVert = std::abs(AccVert) < 0.01 ? 0.0 : AccVert * 0.5;
		// szarpanie
		/*
		#ifdef EU07_USE_FUZZYLOGIC
		        if( FuzzyLogic( ( 10.0 + Track.DamageFlag ) * Mass * Vel / Vmax, 500000.0, p_accn ) ) {
		            // Ra: czemu tu masa bez ładunku?
		            AccV /= ( 2.0 * 0.95 + 2.0 * Random() * 0.1 ); // 95-105% of base modifier (2.0)
		        }
		        else
		#endif
		            AccV = AccV / 2.0;

		        if (AccV > 1.0)
		            AccN += (7.0 - Random(5)) * (100.0 + Track.DamageFlag / 2.0) * AccV / 2000.0;
		*/
		// wykolejanie na luku oraz z braku szyn
		if (TestFlag(CategoryFlag, 1))
		{
			if (TestFlag(Track.DamageFlag, dtrack_norail))
				Derail(END_OF_TRACK);
			// too fast in a curve: the legacy draw in every step (physics.derailment old), or the physical criteria
			// once the step's distance is known, below
			if ((Global.derailment == global_settings::derailment_model::old || Global.derailment == global_settings::derailment_model::monitor) && FuzzyLogic(AccN / g * (1.0 + 0.1 * (Track.DamageFlag & dtrack_freerail)), TrackW / Dim.H, 1))
				Derail(TOO_HIGH_SPEED);
			// wykolejanie na poszerzeniu toru
			if (FuzzyLogic(abs(Track.Width - TrackW), TrackW / 10.0, 1))
				Derail(GAUGE_MISMATCH);
		}
		// wykolejanie wkutek niezgodnosci kategorii toru i pojazdu
		if (!TestFlag(RunningTrack.CategoryFlag, CategoryFlag))
			Derail(WRONG_TRACK_TYPE);

		// dL:=(V+AccS*dt/2)*dt;
		// przyrost dlugosci czyli przesuniecie
		dL = (3.0 * V - Vprev) * dt / 2.0; // metoda Adamsa-Bashfortha}
		if (Global.derailment == global_settings::derailment_model::physical || Global.derailment == global_settings::derailment_model::monitor)
		{
			// over the distance of this step and of the simpler ones of the frame before it
			check_derailment(Shape, Track, derailment_distance + std::abs(dL));
		}
		derailment_distance = 0.0;
		// ale jesli jest kolizja (zas. zach. pedu) to...}
		for (int b = 0; b < 2; b++)
			if (Couplers[b].CheckCollision)
				CollisionDetect(b, dt); // zmienia niejawnie AccS, V !!!
		check_pushed_off(dt, std::abs(dL));

	} // liczone dL, predkosc i przyspieszenie

	auto const d{(EngineType == TEngineType::WheelsDriven ? dL * CabActive : // na chwile dla testu
	                                                        dL)};

	DistCounter += fabs(dL) / 1000.0;
	Odometer.run(dL);
	dL = 0;

	// koniec procedury, tu nastepuja dodatkowe procedury pomocnicze
	compute_movement_(dt);

	// security system
	SecuritySystemCheck(dt1);

	return d;
};

// *************************************************************************************************
// Oblicza przemieszczenie taboru - uproszczona wersja
// *************************************************************************************************

double TMoverParameters::FastComputeMovement(double dt, const TTrackShape & /*Shape*/, TTrackParam & /*Track*/, TLocation const &NewLoc, TRotation const &NewRot)
{
	// T_MoverParameters::FastComputeMovement(dt, Shape, Track, NewLoc, NewRot);

	Loc = NewLoc;
	Rot = NewRot;

	if (dL == 0) // oblicz przesuniecie
	{
		auto const AccSprev{AccS};
		// przyspieszenie styczne
		AccS = std::lerp(AccSprev, FTotal / TotalMass, 0.5);
		// std::clamp( dt * 3.0, 0.0, 1.0 ) ); // prawo Newtona ale z wygladzaniem (średnia z poprzednim)

		if (TestFlag(DamageFlag, dtrain_out) && Derailed == derailment::outcome::none && false == legacy_derailed_slides(*this))
			AccS = -Sign(V) * g * 1; // * random(0.0, 0.1)
		auto const stuck{(Derailed != derailment::outcome::none || legacy_derailed_slides(*this)) && V == 0.0 && std::abs(FTrain) <= std::abs(FStand)};
		if (stuck)
			AccS = 0.0;

		// simple mode skips calculation of normal acceleration

		// velocity change
		auto const Vprev{V};
		V += (3.0 * AccS - AccSprev) * dt / 2.0; // przyrost predkosci
		if (stuck)
			V = 0.0;
		if (V * Vprev <= 0 && std::abs(FStand) > std::abs(FTrain))
		{
			// tlumienie predkosci przy hamowaniu
			// zahamowany
			V = 0;
		}

		// simple mode skips calculation of tangential acceleration

		// simple mode skips calculation of vertical acceleration
		AccVert = 0.0;

		// a vehicle derailed by the physical criteria slides on with the friction of ComputeTotalForce(), and so does one
		// derailed the legacy way with physics.collisions new
		if (true == TestFlag(DamageFlag, dtrain_out) && Derailed == derailment::outcome::none && false == legacy_derailed_slides(*this) && Vel < 1.0)
		{
			V = 0.0;
			AccS = 0.0;
		}

		dL = (3.0 * V - Vprev) * dt / 2.0; // metoda Adamsa-Bashfortha
		// the criteria of derailment go by the distance, checked in the full step which ends the frame
		derailment_distance += std::abs(dL);
		// ale jesli jest kolizja (zas. zach. pedu) to...
		for (int b = 0; b < 2; b++)
			if (Couplers[b].CheckCollision)
				CollisionDetect(b, dt); // zmienia niejawnie AccS, V !!!
		check_pushed_off(dt, std::abs(dL));
	} // liczone dL, predkosc i przyspieszenie

	auto const d{(EngineType == TEngineType::WheelsDriven ? dL * CabActive : // na chwile dla testu
	                                                        dL)};

	DistCounter += fabs(dL) / 1000.0;
	Odometer.run(dL);
	dL = 0;

	// koniec procedury, tu nastepuja dodatkowe procedury pomocnicze
	compute_movement_(dt);

	return d;
};

// updates shared between 'fast' and regular movement computation methods
void TMoverParameters::compute_movement_(double const Deltatime)
{

	// sprawdzanie i ewentualnie wykonywanie->kasowanie poleceń
	if (LoadStatus > 0) // czas doliczamy tylko jeśli trwa (roz)ładowanie
		LastLoadChangeTime += Deltatime; // czas (roz)ładunku

	RunInternalCommand();

	// relay settings
	// adjust motor overload relay threshold
	if (EngineType == TEngineType::ElectricSeriesMotor && ImaxHi > ImaxLo)
	{
		if (MotorOverloadRelayHighThreshold)
		{ // set high threshold
			if (TrainType != train_type::et42 ? RList[MainCtrlPos].Bn < 2 : MainCtrlPos == 0)
			{
				Imax = ImaxHi;
			}
		}
		else
		{ // set low threshold
			if (TrainType != train_type::et42 || MainCtrlPos == 0)
			{
				Imax = ImaxLo;
			}
		}
	}

	// Uproszczona symulacja wentylatorow rezystora hamowania

	// Prad oddawany na rezystor

	// Wlacz wentylator jesli prad rekuperacji przekroczy maksymalny dla pasywnego chlodzenia rezystora
	if (double Irh = abs(eimv[eimv_Pe]) - abs(eimv[eimv_Ipoj]); Irh > Imaxrpc && eimv[eimv_Ipoj] < 0)
	{
		BRVtimer = 0;
		BRVentilators = true;
	}
	else
	{
		BRVtimer += Deltatime;
		if (BRVtimer > BRVto)
			BRVentilators = false;
	}

	// the master controller set directly, not through IncMainCtrl()/DecMainCtrl() (AI, consist): the locked shunt follows it as well
	ScndCtrlResistiveLockCheck();
	// automatyczny rozruch
	if (EngineType == TEngineType::ElectricSeriesMotor && AutoRelayCheck())
	{
		SetFlag(SoundFlag, sound::relay);
	}

	if ((EngineType == TEngineType::DieselEngine || EngineType == TEngineType::DieselElectric) && dizel_Update(Deltatime))
	{
		SetFlag(SoundFlag, sound::relay);
	}

	// TODO: gather and move current calculations to dedicated method
	TotalCurrent = 0;

	// low voltage power sources
	LowVoltagePowerCheck(Deltatime);
	// power sources
	PantographsCheck(Deltatime);
	GensetCheck(Deltatime);
	// main circuit
	MainsCheck(Deltatime);
	// traction motors
	MotorBlowersCheck(Deltatime);
	// uklady hamulcowe:
	ConverterCheck(Deltatime);
	if (VeselVolume > 0)
		Compressor = CompressedVolume / VeselVolume;
	else
	{
		Compressor = 0;
		CompressorFlag = false;
	}
	if (VeselVolume > 0.0)
	{
		// sprężarka musi mieć jakąś niezerową wydajność żeby rozważać jej załączenie i pracę
		CompressorCheck(Deltatime);
	}
	if (Power > 1.0)
	{
		// w rozrządczym nie (jest błąd w FIZ!) - Ra 2014-07: teraz we wszystkich
		UpdatePantVolume(Deltatime); // Ra 2014-07: obsługa zbiornika rozrządu oraz pantografów
	}
	// heating
	HeatingCheck(Deltatime);
	// lighting
	LightsCheck(Deltatime);

	UpdateBrakePressure(Deltatime);
	UpdatePipePressure(Deltatime);
	UpdateBatteryVoltage(Deltatime);
	UpdateScndPipePressure(Deltatime); // druga rurka, youBy

	if ((DCEMUED_CC & 1) != 0 && (Couplers[end::front].CouplingFlag & coupling::control) != 0)
	{
		DynamicBrakeEMUStatus &= Couplers[end::front].Connected->DynamicBrakeEMUStatus;
	}
	if ((DCEMUED_CC & 2) != 0 && (Couplers[end::rear].CouplingFlag & coupling::control) != 0)
	{
		DynamicBrakeEMUStatus &= Couplers[end::rear].Connected->DynamicBrakeEMUStatus;
	}

	if (BrakeSlippingTimer > 0.8 && ASBType != 128)
	{ // ASBSpeed=0.8
		// hamulec antypoślizgowy - wyłączanie
		Hamulec->ASB(0);
	}
	BrakeSlippingTimer += Deltatime;
	// automatic doors
	update_doors(Deltatime);

	m_plc.update(Deltatime);

	PowerCouplersCheck(Deltatime, coupling::highvoltage);
	PowerCouplersCheck(Deltatime, coupling::power110v);
	PowerCouplersCheck(Deltatime, coupling::power24v);

	Power24vVoltage = std::max(PowerCircuits[0].first, GetTrainsetVoltage(coupling::power24v));
	Power24vIsAvailable = Power24vVoltage > 0;
	Power110vIsAvailable = PowerCircuits[1].first > 0 || GetTrainsetVoltage(coupling::power110v) > 0;
}

// *************************************************************************************************
// Q: 20160710
// Siła napędzająca drezynę po naciśnięciu wajhy
// *************************************************************************************************
bool TMoverParameters::AddPulseForce(int Multipler)
{
	bool APF;
	if (EngineType == TEngineType::WheelsDriven && EnginePowerSource.SourceType == TPowerSource::InternalSource && EnginePowerSource.PowerType == TPowerType::BioPower)
	{
		DirActive = CabActive;
		DirAbsolute = DirActive * CabActive;
		if (Vel > 0)
			PulseForce = std::min(1000.0 * Power / (abs(V) + 0.1), Ftmax);
		else
			PulseForce = Ftmax;
		if (PulseForceCount > 1000.0)
			PulseForce = 0;
		else
			PulseForce = PulseForce * Multipler;
		PulseForceCount = PulseForceCount + abs(Multipler);
		APF = PulseForce > 0;
	}
	else
		APF = false;
	return APF;
}

// *************************************************************************************************
// yB: 20190909
// sypanie piasku reczne
// *************************************************************************************************
bool TMoverParameters::SandboxManual(bool const State, range_t const Notify)
{
	bool result{false};

	if (SandDoseManual != State)
	{
		if (SandDoseManual == false)
		{
			// switch on
			if (Sand > 0)
			{
				SandDoseManual = true;
				result = true;
			}
		}
		else
		{
			// switch off
			SandDoseManual = false;
			result = true;
		}
	}

	Sandbox(SandDoseManual || SandDoseAuto, Notify);

	return result;
}

// *************************************************************************************************
// yB: 20190909
// sypanie piasku automatyczne
// *************************************************************************************************
bool TMoverParameters::SandboxAuto(bool const State, range_t const Notify)
{
	bool result{false};
	if (bool NewState = State && SandDoseAutoAllow; SandDoseAuto != NewState)
	{
		if (SandDoseAuto == false)
		{
			// switch on
			if (Sand > 0)
			{
				SandDoseAuto = true;
				result = true;
			}
		}
		else
		{
			// switch off
			SandDoseAuto = false;
			result = true;
		}
	}

	Sandbox(SandDoseManual || SandDoseAuto, Notify);

	return result;
}

// *************************************************************************************************
// Q: 20160713
// sypanie piasku
// *************************************************************************************************
bool TMoverParameters::Sandbox(bool const State, range_t const Notify)
{
	bool result{false};

	if (SandDose != State)
	{
		if (SandDose == false)
		{
			// switch on
			if (Sand > 0 && DirActive != 0)
			{
				SandDose = true;
				result = true;
			}
		}
		else
		{
			// switch off
			SandDose = false;
			result = true;
		}
	}

	if (Notify != range_t::local)
	{
		// if requested pass the command on
		auto const couplingtype = Notify == range_t::unit ? coupling::control | coupling::permanent : coupling::control;

		if (State == true)
		{
			// switch on
			SendCtrlToNext(VehicleCommand::sandbox, 1, CabActive, couplingtype);
		}
		else
		{
			// switch off
			SendCtrlToNext(VehicleCommand::sandbox, 0, CabActive, couplingtype);
		}
	}

	return result;
}

// *************************************************************************************************
// yB: 20190909
// włączenie / wyłączenie automatycznej piasecznicy
// *************************************************************************************************
bool TMoverParameters::SandboxAutoAllow(bool State)
{
	// SendCtrlToNext(VehicleCommand::sandboxAutoAllow, int(State), CabActive, ctrain_controll);

	if (SandDoseAutoAllow != State)
	{
		SandDoseAutoAllow = State;
		return true;
	}
	else
		return false;
}

// *************************************************************************************************
// Q: 20160713
// Obliczenie stałych potrzebnych do dalszych obliczeń
// *************************************************************************************************
void TMoverParameters::ComputeConstans(void)
{
	double BearingF;
	double RollF;
	double HideModifier;
	double Curvature; // Ra 2014-07: odwrotność promienia

	TotalMassxg = TotalMass * g; // TotalMass*g
	BearingF = (DamageFlag & dtrain_bearing) != 0 ? 2.0 : 0;

	HideModifier = 0; // int(Couplers[0].CouplingFlag>0)+int(Couplers[1].CouplingFlag>0);

	if (BearingType == 0)
		RollF = 0.05; // slizgowe
	else
		RollF = 0.015; // toczne
	RollF += BearingF / 200.0;

	//    if (NPoweredAxles > 0)
	//     RollF = RollF * 1.5;    //dodatkowe lozyska silnikow

	if (NPoweredAxles > 0) // drobna optymalka
	{
		RollF += 0.025;
		//       if (Ft * Ft < 1)
		//         HideModifier = HideModifier - 3;
	}
	Ff = TotalMassxg * (BearingF + RollF * V * V / 10.0) / 1000.0;
	// dorobic liczenie temperatury lozyska!
	FrictConst1 = TotalMassxg * RollF / 10000.0;
	// drag calculation
	{
		// NOTE: draft effect of previous vehicle is simplified and doesn't have much to do with reality
		auto const *previousvehicle{Couplers[V >= 0.0 ? end::front : end::rear].Connected};
		auto dragarea{Dim.W * Dim.H};
		if (previousvehicle)
		{
			dragarea = std::max(0.0, dragarea - 0.85 * previousvehicle->Dim.W * previousvehicle->Dim.H);
		}
		FrictConst1 += Cx * dragarea;
	}

	if (CategoryFlag & 1)
	{
		Curvature = RunningShape.R == 0.0 ? // zero oznacza nieskończony promień
		                0.0 :
		                1.0 / std::abs(RunningShape.R);
	}
	else
	{
		// vehicles other than trains don't experience friction against the rail on curves
		Curvature = 0.0;
	}
	// opór składu na łuku (youBy): +(500*TrackW/R)*TotalMassxg*0.001 do FrictConst2s/d
	FrictConst2s = TotalMassxg * (500.0 * TrackW * Curvature + 2.5 - HideModifier + 2 * BearingF / dtrain_bearing) * 0.001;
	FrictConst2d = TotalMassxg * (500.0 * TrackW * Curvature + 2.0 - HideModifier + BearingF / dtrain_bearing) * 0.001;
}

// *************************************************************************************************
// Q: 20160713
// Oblicza masę ładunku
// *************************************************************************************************
void TMoverParameters::ComputeMass()
{
	// Ra: na razie tak, ale nie wszędzie masy wirujące się wliczają
	TotalMass = Mass + Mred;

	if (LoadAmount == 0)
	{
		return;
	}

	// include weight of carried load
	auto loadtypeunitweight{0.f};

	if (ToLower(LoadQuantity) == "tonns")
	{
		loadtypeunitweight = 1000;
	}
	else
	{
		auto const lookup{simulation::Weights.find(LoadType.name)};
		loadtypeunitweight = lookup != simulation::Weights.end() ? lookup->second : 1000.f; // legacy default unit weight value
	}

	TotalMass += LoadAmount * loadtypeunitweight;
}

// *************************************************************************************************
// Q: 20160713
// Obliczanie wypadkowej siły z wszystkich działających sił
// *************************************************************************************************
// TBD, TODO: move some of the calculations out of the method, they're relevant to more than just force calculations
void TMoverParameters::ComputeTotalForce(double dt)
{

	Vel = std::abs(V) * 3.6; // prędkość w km/h

	// McZapkie-031103: sprawdzanie czy warto liczyc fizyke i inne updaty
	// ABu 300105: cos tu mieszalem , dziala teraz troche lepiej, wiec zostawiam
	{
		// asleep, the brake force is the one of the step it fell asleep in: a vehicle whose brakes have changed since, released
		// or applied, wakes for the 5 s of LastSwitchingTime, which take the new force into FStand, and sleeps again if it's
		// still held. none of the causes of the test below, so as "other"
		if (brakes_changed_asleep())
			switch_physics(true, wake_cause::other);

		auto const vehicleisactive{CabActive != 0 || Vel > 0.0001 || accelerating() || LastSwitchingTime < 5 || is_multiple_unit()};

		auto const movingvehicleahead{Neighbours[end::front].vehicle != nullptr &&
		                              (Neighbours[end::front].vehicle->MoverParameters->Vel > 0.0001 || Neighbours[end::front].vehicle->MoverParameters->accelerating())};

		auto const movingvehiclebehind{Neighbours[end::rear].vehicle != nullptr &&
		                               (Neighbours[end::rear].vehicle->MoverParameters->Vel > 0.0001 || Neighbours[end::rear].vehicle->MoverParameters->accelerating())};

		auto const calculatephysics{vehicleisactive || movingvehicleahead || movingvehiclebehind};

		switch_physics(calculatephysics, Vel > 0.0001                     ? wake_cause::speed :
		                                 accelerating()                   ? wake_cause::acceleration :
		                                 vehicleisactive                  ? wake_cause::rest :
		                                                                    wake_cause::neighbour);
	}

	if (false == PhysicActivation)
	{
		return;
	}

	// juz zoptymalizowane:
	FStand = FrictionForce(); // siła oporów ruchu
	if (true == TestFlag(DamageFlag, dtrain_out))
	{
		if (Derailed != derailment::outcome::none)
		{
			// off the rails by the physical criteria: the vehicle slides on the sleepers and the ballast, or on its side;
			// off the track on the ground, as far as it tipped over (physics.derailment new)
			auto const friction{OffTrack.active() ? OffTrack.friction() : derailment::sliding_friction(Derailed)};
			FStand = friction * (TotalMass - Mred) * g;
		}
		else if (legacy_derailed_slides(*this))
		{
			// derailed the legacy way with physics.collisions new: the deceleration of g of the legacy model, as a friction
			FStand = TotalMass * g;
		}
		else
		{
			// HACK: crude way to reduce speed after derailment
			// TBD, TODO: more accurate approach?
			FStand *= 1e20;
		}
	}
	double old_nrot = abs(nrot);
	nrot = v2n(); // przeliczenie prędkości liniowej na obrotową

	if (true == TestFlag(BrakeMethod, bp_MHS) && PipePress < 3.0 // ustawione na sztywno na 3 bar
	    && Vel > 45 && true == TestFlag(BrakeDelayFlag, bdelay_M))
	{
		// doliczenie hamowania hamulcem szynowym
		FStand += TrackBrakeForce;
	}
	// w charakterystykach jest wartość siły hamowania zamiast nacisku

	LastSwitchingTime += dt;
	if (EngineType == TEngineType::ElectricSeriesMotor)
	{
		LastRelayTime += dt;
	}

	if (EngineType == TEngineType::ElectricSeriesMotor) // potem ulepszyc! pantogtrafy!
	{ // Ra 2014-03: uwzględnienie kierunku jazdy w napięciu na silnikach, a powinien być zdefiniowany nawrotnik
		EngineVoltage = Mains ? EnginePowerSourceVoltage() : 0.0;
		if (CabActive == 0)
		{
			EngineVoltage *= DirActive;
		}
		else
		{
			EngineVoltage *= DirAbsolute; // the reverser of the active cab, relative to the couplers
		}
	} // bo nie dzialalo
	else
	{
		EngineVoltage = Power > 1.0 ? TractionSupplyVoltage() : 0.0;
	}

	FTrain = Power > 0 ? TractionForce(dt) : 0;
	double FT_factor = 1.0;
	if (EngineType == TEngineType::ElectricInductionMotor && InvertersRatio > 0.0)
	{
		FT_factor = 1.0 / InvertersRatio;
		FTrain *= FT_factor;
	}

	Fb = BrakeForce(RunningTrack);
	// poslizg
	auto Fwheels{FTrain - Fb * Sign(V)};
	if (Vel > 0.1 // crude trap, to prevent braked stationary vehicles from passing fb > mass * adhesive test
	    && std::abs(Fwheels) > TotalMassxg * Adhesive(RunningTrack.friction))
	{
		SlippingWheels = true;
	}
	if (true == SlippingWheels && Vel < 0.1 // the same crude trap: (nearly) stopped vehicle...
	    && std::abs(nrot * M_PI * WheelDiameter - V) < 0.1 / 3.6) // ...with wheels turning at its speed, i.e. locked wheels at the end of a skid
	{
		// locked wheels never pass the rotation speed test below once the vehicle stops, and a skidding vehicle
		// keeps its braking force capped by the sliding adhesion, so the rest of the consist could push it around
		SlippingWheels = false;
		nrot = V / M_PI / WheelDiameter;
	}
	double temp_nrot = nrot;
	if (true == SlippingWheels)
	{

		temp_nrot = ComputeRotatingWheel(Fwheels - Sign(nrot * M_PI * WheelDiameter - V) * Adhesive(RunningTrack.friction) * TotalMassxg, dt, nrot);
		if (Sign(nrot * M_PI * WheelDiameter - V) * Sign(temp_nrot * M_PI * WheelDiameter - V) < 0)
		{
			SlippingWheels = false;
			temp_nrot = V / M_PI / WheelDiameter;
		}
	}
	if (true == SlippingWheels)
	{
		Fwheels = Sign(temp_nrot * M_PI * WheelDiameter - V) * TotalMassxg * Adhesive(RunningTrack.friction);
		if (Fwheels * Sign(V) > 0)
		{
			FTrain = Fwheels + Fb * Sign(V);
		}
		else if (FTrain * Sign(V) > 0)
		{
			Fb = FTrain * Sign(V) - Fwheels * Sign(V);
		}
		else
		{
			double factor = FTrain - Fb * Sign(V) != 0 ? Fwheels / (FTrain - Fb * Sign(V)) : 1.0;
			Fb *= factor;
			FTrain *= factor;
		}
		if (std::abs(nrot) < 0.1)
		{
			WheelFlat = sqrt(square(WheelFlat) + abs(Fwheels) / NAxles * Vel * 0.000002);
		}

		nrot = temp_nrot;
	}
	nrot_eps = (abs(nrot) - old_nrot) / dt;
	// doliczenie sił z innych pojazdów
	for (int end = end::front; end <= end::rear; ++end)
	{
		// a new step: the hit of the pair not taken by either vehicle yet
		Couplers[end].hit_handled = false;
		Couplers[end].hit_force = 0.0;
		Couplers[end].hit_duration = 0.0;
		Couplers[end].closing = 0.0;
		if (Neighbours[end].vehicle != nullptr)
		{
			// the velocities before the step, the forces of all the vehicles come before their movement
			Couplers[end].closing = -CouplerVelocityDifference(end, *Neighbours[end].vehicle->MoverParameters, Neighbours[end].vehicle_end) * DirF(end);
			Couplers[end].CForce = CouplerForce(end, dt);
			FTrain += Couplers[end].CForce;
			// what the buffers of the pair take in, for the budget of the crush (physics.collisions new and monitor)
			auto &coupler{Couplers[end]};
			coupler.contact.update(coupler.Dist < 0.0, coupler.CheckCollision, coupler.closing, compression_at(coupler, end), std::max(0.0, -coupler.Dist), dt);
		}
		else
		{
			Couplers[end].CForce = 0;
			Couplers[end].contact.reset();
		}
	}

	FStand += Fb;
	// doliczenie składowej stycznej grawitacji
	FTrain /= FT_factor;
	FTrain += TotalMassxg * RunningShape.dHtrack;
	//! niejawne przypisanie zmiennej!
	FTotal = FTrain - Sign(V) * FStand;
}

double TMoverParameters::BrakeForceR(double ratio, double velocity) const
{
	double press = 0;
	ratio = Hamulec->GetBrakeRatio(ratio);
	if (MBPM > 2)
	{
		press = MaxBrakePress[1] + (MaxBrakePress[3] - MaxBrakePress[1]) * std::min(1.0, (TotalMass - Mass) / (MBPM - Mass));
	}
	else
	{
		if (MaxBrakePress[1] > 0.1)
		{
			press = MaxBrakePress[LoadFlag];
		}
		else
		{
			press = MaxBrakePress[3];
			if (DynamicBrakeType == dbrake_automatic)
				ratio = ratio + (1.5 - ratio) * std::min(1.0, Vel * 0.02);
			if (BrakeDelayFlag & bdelay_R && BrakeMethod % 128 != bp_Cosid && BrakeMethod % 128 != bp_D1 && BrakeMethod % 128 != bp_D2 && Power < 1)
				ratio *= Hamulec->GetRapidRatio(velocity);
			if (TrainType == train_type::dmu && velocity < 30.0)
			{
				ratio -= 0.3;
			}
		}
	}
	return BrakeForceP(press * ratio, velocity);
}

double TMoverParameters::BrakeForceP(double press, double velocity) const
{
	double BFP = 0;
	double K = ((press * P2FTrans - BrakeCylSpring) * BrakeCylMult[0] - BrakeSlckAdj) * BrakeRigEff;
	K *= static_cast<double>(BrakeCylNo) / (NAxles * std::max(1, NBpA));
	BFP = Hamulec->GetFC(velocity, K) * K * (NAxles * std::max(1, NBpA)) * 1000;
	return BFP;
}

// *************************************************************************************************
// Q: 20160713
// oblicza siłę na styku koła i szyny
// *************************************************************************************************
double TMoverParameters::BrakeForce(TTrackParam const & /*Track*/)
{

	double K{0};
	double brakeforce{0};

	switch (LocalBrake)
	{
	case TLocalBrake::ManualBrake:
	{
		K = MaxBrakeForce * ManualBrakeRatio();
		break;
	}
	case TLocalBrake::HydraulicBrake:
	{
		K = MaxBrakeForce * LocalBrakeRatio();
		break;
	}
	default:
	{
		break;
	}
	}

	if (MBrake == true)
	{
		K = MaxBrakeForce * ManualBrakeRatio();
	}

	// the spring brake of a vehicle prepared for hauling (PulleeMode) is released by hand: no force, whatever its cylinder has
	if (SpringBrake.IsReady && false == PulleeMode)
		K += std::max(0.0, SpringBrake.MinForcePressure - SpringBrake.Cylinder->P()) * SpringBrake.MaxBrakeForce;

	u = (BrakePress * P2FTrans - BrakeCylSpring) * BrakeCylMult[0] - BrakeSlckAdj;
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
	if (BrakeSystem == TBrakeSystem::Pneumatic || BrakeSystem == TBrakeSystem::ElectroPneumatic)
	{
		u = Hamulec->GetFC(Vel, K);
		UnitBrakeForce = u * K * 1000.0; // sila na jeden klocek w N
	}
	else
		UnitBrakeForce = K * 1000.0;
	//     if (LocalBrake=ManualBrake)or(MBrake=true)) and (BrakePress<0.3) then
	//      Fb:=UnitBrakeForce*NBpA {ham. reczny dziala na jedna os}
	//     else  //yB: to nie do konca ma sens, ponieważ ręczny w wagonie działa na jeden cylinder
	//     hamulcowy/wózek, dlatego potrzebne są oddzielnie liczone osie
	brakeforce = UnitBrakeForce * NBrakeAxles * std::max(1, NBpA);

	//  u:=((BrakePress*P2FTrans)-BrakeCylSpring*BrakeCylMult[BCMFlag]/BrakeCylNo-0.83*BrakeSlckAdj/(BrakeCylNo))*BrakeCylNo;
	// {  end; }
	return brakeforce;
}

// *************************************************************************************************
// Q: 20160713
// Obliczanie siły tarcia
// *************************************************************************************************
double TMoverParameters::FrictionForce() const
{
	double FF = 0;
	// ABu 240205: chyba juz ekstremalnie zoptymalizowana funkcja liczaca sily tarcia
	if (abs(V) > 0.01)
		FF = FrictConst1 * V * V + FrictConst2d;
	else
		FF = FrictConst1 * V * V + FrictConst2s;
	return FF;
}

// *************************************************************************************************
// Q: 20160713
// Oblicza przyczepność
// *************************************************************************************************
double TMoverParameters::Adhesive(double staticfriction) const
{
	double adhesion = 0.0;
	const double adh_factor = 0.25; // współczynnik określający, jak bardzo spada tarcie przy poślizgu
	const double slipfactor = 0.33; // współczynnik określający, jak szybko spada tarcie przy poślizgu
	const double sandfactor = 1.25; // współczynnik określający, jak mocno pomaga piasek
	/*
	    // ABu: male przerobki, tylko czy to da jakikolwiek skutek w FPS?
	    //     w kazdym razie zaciemni kod na pewno :)
	    if (SlippingWheels == false)
	    {
	        if (SandDose)
	            adhesion = (std::max(staticfriction * (100.0 + Vel) / ((50.0 + Vel) * 11.0), 0.048)) *
	                       (11.0 - 2.0 * Random(0.0, 1.0));
	        else
	            adhesion = (staticfriction * (100.0 + Vel) / ((50.0 + Vel) * 10.0)) *
	                       (11.0 - 2.0 * Random(0.0, 1.0));
	    }
	    else
	    {
	        if (SandDose)
	            adhesion = (0.048) * (11.0 - 2.0 * Random(0.0, 1.0));
	        else
	            adhesion = (staticfriction * 0.02) * (11.0 - 2.0 * Random(0.0, 1.0));
	    }
	    //  WriteLog(FloatToStr(adhesive));      // tutaj jest na poziomie 0.2 - 0.3
	    return adhesion;

	    //wersja druga
	    if( true == SlippingWheels ) {

	        if( true == SandDose ) { adhesion = 0.48; }
	        else                   { adhesion = staticfriction * 0.2; }
	    }
	    else {

	        if( true == SandDose ) { adhesion = std::max( staticfriction * ( 100.0 + Vel ) / ( 50.0 + Vel ) * 1.1, 0.48 ); }
	        else                   { adhesion = staticfriction * ( 100.0 + Vel ) / ( 50.0 + Vel ); }
	    }
	//    adhesion *= ( 0.9 + 0.2 * Random() );
	*/
	// wersja3 by youBy - uwzględnia naturalne mikropoślizgi i wpływ piasecznicy, usuwa losowość z pojazdu
	double Vwheels = nrot * M_PI * WheelDiameter; // predkosc liniowa koła wynikająca z obrotowej
	double deltaV = V - Vwheels; // poślizg - różnica prędkości w punkcie styku koła i szyny
	deltaV = std::max(0.0, std::abs(deltaV) - 0.25); // mikropoślizgi do ok. 0,25 m/s nie zrywają przyczepności
	Vwheels = std::abs(Vwheels);
	adhesion = staticfriction * (28 + Vwheels) / (14 + Vwheels) * ((SandDose ? sandfactor : 1) - (1 - adh_factor) * (deltaV / (deltaV + slipfactor)));

	return adhesion;
}

// *************************************************************************************************
// Q: 20160713
// Obliczenie predkości obrotowej kół???
// *************************************************************************************************
double TMoverParameters::ComputeRotatingWheel(double WForce, double dt, double n) const
{
	double newn = 0;
	double eps = 0;
	if (n == 0 && WForce * Sign(V) < 0)
		newn = 0;
	else
	{
		eps = WForce * WheelDiameter / (2.0 * AxleInertialMoment);
		newn = n + eps * dt;
		if (newn * n <= 0 && eps * n < 0)
			newn = 0;
	}
	return newn;
}

// *************************************************************************************************
// Q: 20160713
// Przesuwa pojazd o podaną wartość w bok względem toru (dla samochodów)
// *************************************************************************************************
bool TMoverParameters::ChangeOffsetH(double DeltaOffset)
{
	bool COH = false;
	if (TestFlag(CategoryFlag, 2) && TestFlag(RunningTrack.CategoryFlag, 2))
	{
		OffsetTrackH = OffsetTrackH + DeltaOffset;
		//     if (abs(OffsetTrackH) > (RunningTrack.Width / 1.95 - TrackW / 2.0))
		if (abs(OffsetTrackH) > 0.5 * (RunningTrack.Width - Dim.W) - 0.05) // Ra: może pół pojazdu od brzegu?
			COH = false; // kola na granicy drogi
		else
			COH = true;
	}
	else
		COH = false;

	return COH;
}

// *************************************************************************************************
// Q: 20160713
// Testuje zmienną (narazie tylko 0) i na podstawie uszkodzenia zwraca informację tekstową
// *************************************************************************************************
std::string TMoverParameters::EngineDescription(int what) const
{
	std::string outstr{"OK"};
	if (what == 0)
	{
		if (DamageFlag == 255)
		{
			outstr = "WRECKED";
		}
		else
		{
			if (TestFlag(DamageFlag, dtrain_thinwheel))
			{
				if (Power > 0.1)
					outstr = "Thin wheel";
				else
					outstr = "Load shifted";
			}
			if (WheelFlat > 5.0 || TestFlag(DamageFlag, dtrain_wheelwear))
			{
				outstr = "Wheel wear";
			}
			if (TestFlag(DamageFlag, dtrain_bearing))
			{
				outstr = "Bearing damaged";
			}
			if (TestFlag(DamageFlag, dtrain_coupling))
			{
				outstr = "Coupler broken";
			}
			if (TestFlag(DamageFlag, dtrain_loaddamage))
			{
				if (Power > 0.1)
					outstr = "Ventilator damaged";
				else
					outstr = "Load damaged";
			}
			if (TestFlag(DamageFlag, dtrain_loaddestroyed))
			{
				if (Power > 0.1)
					outstr = "Engine damaged";
				else
					outstr = "LOAD DESTROYED";
			}
			if (TestFlag(DamageFlag, dtrain_axle))
			{
				outstr = "Axle broken";
			}
			if (TestFlag(DamageFlag, dtrain_out))
			{
				outstr = "DERAILED";
			}
		}
	}
	else
	{
		outstr = "Invalid qualifier";
	}
	return outstr;
}

// *************************************************************************************************
// whether the vehicle accelerates, for the test which turns the physics off in vehicles standing still
// *************************************************************************************************
bool TMoverParameters::accelerating() const
{
	if (std::abs(AccS) <= 0.0001)
		return false;
	// standing still and held by the brakes and the resistance, against the slope and the couplers: the test ComputeMovement()
	// sets the speed to zero by. the forces are of the last ComputeTotalForce(), the ones the movement went by. a vehicle whose
	// brake cylinders still fill or release isn't held yet: asleep, it would keep the brake force (and the pressure the AI waits on
	// to set off) it had when it fell asleep. 0.05 bar/s is less than a distributor moves the cylinder when it applies or releases, and
	// more than the leaks take
	auto const held{Global.physics_sleepheld && Vel <= 0.0001 && std::abs(FStand) > std::abs(FTrain) && BrakePressRate <= 0.05};
	return false == held;
}

void TMoverParameters::wake_for_brake()
{
	if (Global.physics_sleepheld)
	{
		switch_physics(true, wake_cause::command);
	}
}

// *************************************************************************************************
// whether the brakes of a vehicle whose physics sleeps have changed since it fell asleep: the pressure in the brake cylinders
// or in the one of the spring brake by more than 0.05 bar, the hand brake by a notch, or the local brake by 1 percent. the brake system
// itself steps on while asleep in the vehicle the user is in (TDynamicObject::Update(), MechInside); the settings change by
// hand or by a command in any vehicle. 0.05 bar is less than what a distributor applying or releasing moves the cylinder by in
// a second, and much less than the pressure the cylinder's return spring takes (BrakeCylSpring, some tenths of a bar), below
// which the cylinder gives no force: a release ends with the vehicle asleep and no force left. a cylinder holding its
// pressure, or settling within that, leaves the vehicle asleep
// *************************************************************************************************
bool TMoverParameters::brakes_changed_asleep() const
{
	if (PhysicActivation)
		return false;
	constexpr double pressure_change{0.05}; // [bar]
	return std::abs(BrakePress - sleep_brakes.cylinder) > pressure_change || std::abs(SpringBrake.SBP - sleep_brakes.spring) > pressure_change ||
	       ManualBrakePos != sleep_brakes.manual || std::abs(LocalBrakePosA - sleep_brakes.local) > 0.01;
}

// *************************************************************************************************
// Kasowanie zmiennych pracy fizyki
// *************************************************************************************************
bool TMoverParameters::switch_physics(bool const State, wake_cause const Cause) // DO PRZETLUMACZENIA NA KONCU
{
	if (PhysicActivation == State)
	{
		return false;
	}

	PhysicActivation = State;
	if (true == State)
	{
		LastSwitchingTime = 0;
		Timer::subsystem.switching.note(static_cast<Timer::switching_counter::source>(Timer::switching_counter::wake + static_cast<std::size_t>(Cause)), this, &TypeName, &Name);
	}
	else
	{
		// what the brake force of the last step came from, which it keeps while asleep (brakes_changed_asleep())
		sleep_brakes = {BrakePress, SpringBrake.SBP, ManualBrakePos, LocalBrakePosA};
	}

	return true;
}

namespace simulation
{

weights_table Weights;

} // namespace simulation
