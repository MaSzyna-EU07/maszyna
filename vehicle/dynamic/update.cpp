/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/
/*
    MaSzyna EU07 locomotive simulator
    Copyright (C) 2001-2004  Marcin Wozniak, Maciej Czapkiewicz and others

*/

// the update of a vehicle in a step of the simulation

#include "stdafx.h"
#include "vehicle/DynObj.h"
#include "physics/mover.h"
#include "vehicle/lightsprogrammer.h"
#include "simulation/simulation.h"
#include "simulation/blackboxrecorder.h"
#include "rendering/lightarray.h"
#include "vehicle/Camera.h"
#include "vehicle/Train.h"
#include "vehicle/odometer.h"
#include "vehicle/pulleemode.h"
#include "vehicle/Driver.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <format>
#include <random>
#include <iterator>
#include <numbers>
#include <optional>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>
#include "utilities/Globals.h"
#include "utilities/Timer.h"
#include "utilities/workerpool.h"
#include "utilities/Logs.h"
#include "utilities/glmHelpers.h"
#include "Console.h"
#include "world/Traction.h"
#include "audio/sound.h"
#include "model/MdlMngr.h"
#include "model/Model3d.h"
#include "vehicle/doorwings.h"
#include "rendering/renderer.h"
#include "editor/editorTerrainStreamer.hpp"
#include "application/uitranscripts.h"
#include "input/messaging.h"
#include "application/application.h"
#include "vehicle/Driver.h"
#include "vehicle/dynamic/internal.h"


bool TDynamicObject::Update(double dt, double dt1)
{
	if (dt1 == 0.0)
        return true; // Ra: pauza
    // physics.derailment new: all of the update of a vehicle off the track, timed; a warning once a second at most when it
    // takes long, to find what makes a derailment slow
    // (no vehicle when it is on the track: nothing is timed then)
    class off_track_watch {
    public:
        off_track_watch( TDynamicObject *Vehicle, double const Frame ) :
            vehicle { Vehicle }, frame { Frame } {
            if( vehicle != nullptr ) {
                started = off_track_clock::now();
            }
        }
        off_track_watch( off_track_watch const & ) = delete;
        off_track_watch &operator=( off_track_watch const & ) = delete;
        ~off_track_watch() {
            if( vehicle == nullptr ) {
                return;
            }
            auto const took { seconds_since( started ) };
            vehicle->m_offtrackupdatedue -= frame;
            if( took > off_track_slow && vehicle->m_offtrackupdatedue <= 0.0 ) {
                vehicle->m_offtrackupdatedue = 1.0;
                ErrorLog( std::format( "derailment: warning: the update of {} off the track took {:.0f} ms in a frame of {:.3f} s", vehicle->MoverParameters->Name, took * 1000.0, frame ) );
            }
        }
    private:
        TDynamicObject *vehicle;
        double frame;
        off_track_clock::time_point started {};
    };
    off_track_watch const watch { MoverParameters->OffTrack.active() ? this : nullptr, dt1 };
    if( m_offtrackterrain != nullptr && false == MoverParameters->OffTrack.active() ) {
        // physics.derailment new: back on the rails (rerail()), the terrain it slid over is let go
        m_offtrackterrain.reset();
        m_offtracksite.terrain( nullptr );
    }
    if (!MoverParameters->PhysicActivation &&
        !MechInside) { // to drugie, bo będąc w maszynowym blokuje się fizyka
        // physics.derailment new: a vehicle off the track stopped along its heading still settles: it slides down a
        // slope, falls over, or comes to rest after the step its physics stopped in
        if( MoverParameters->OffTrack.active() && false == MoverParameters->OffTrack.at_rest() && bEnabled ) {
            m_offtracktime = dt1;
            Move( 0.0 );
            update_off_track( dt1 );
        }
        return true; // McZapkie: wylaczanie fizyki gdy nie potrzeba
    }
    if (!MyTrack)
        return false; // pojazdy postawione na torach portalowych mają MyTrack==NULL
    if (!bEnabled)
        return false; // a normalnie powinny mieć bEnabled==false

    double dDOMoveLen;
    // McZapkie: parametry powinny byc pobierane z toru
    if (Axle0.vAngles.z != Axle1.vAngles.z)
    { // wyliczenie promienia z obrotów osi - modyfikację zgłosił youBy
        ts.R = Axle0.vAngles.z - Axle1.vAngles.z; // różnica może dawać stałą ±M_2PI
        if( ts.R > 15000.0 || ts.R < -15000.0 ) {
            // szkoda czasu na zbyt duże promienie, 4km to promień nie wymagający przechyłki
            ts.R = 0.0;
        }
        else {
            // normalizacja
                 if( ts.R >  M_PI ) { ts.R -= M_2PI; }
            else if( ts.R < -M_PI ) { ts.R += M_2PI; }

            if( ts.R != 0.0 ) {
                // sin(0) results in division by zero
                ts.R = -0.5 * MoverParameters->BDist / sin( ts.R * 0.5 );
            }
        }
    }
    else
        ts.R = 0.0;
    // ts.R=ComputeRadius(Axle1.pPosition,Axle2.pPosition,Axle3.pPosition,Axle0.pPosition);
    // Ra: składową pochylenia wzdłużnego mamy policzoną w jednostkowym wektorze
    // vFront
    ts.Len = 1.0; // std::max(MoverParameters->BDist,MoverParameters->ADist);
    ts.dHtrack = -vFront.y; // Axle1.pPosition.y-Axle0.pPosition.y; //wektor
    // między skrajnymi osiami
    // (!!!odwrotny)
    ts.dHrail = (Axle1.GetRoll() + Axle0.GetRoll()) * 0.5; //średnia przechyłka pudła
    // the uneven track under the outer axles, for the criteria of derailment: its lateral curvature, and its cross level
    ts.uneven_curvature = glm::dot( 0.5 * ( Axle0.UnevenBend() + Axle1.UnevenBend() ), vLeft );
    ts.uneven_cross = MyTrack->uneven_underway().cross;
    // a buckle of the track under the body: the mean of the curvatures at the outer axles, each over the wheelbase of its bogie
    ts.buckle_curvature = 0.5 * ( Axle0.BuckleCurvature( MoverParameters->ADist ) + Axle1.BuckleCurvature( MoverParameters->ADist ) );
    // TTrackParam tp;
    tp.Width = MyTrack->fTrackWidth;
    // McZapkie-250202
    tp.friction = MyTrack->Friction() * Global.fFriction * Global.FrictionWeatherFactor;
    tp.CategoryFlag = MyTrack->iCategoryFlag & 15;
    tp.DamageFlag = MyTrack->iDamageFlag;
    tp.QualityFlag = MyTrack->iQualityFlag;
    tp.Name = &MyTrack->name();

    // couplers
    if( MoverParameters->Couplers[0].CouplingFlag != Coupling::none
     && MoverParameters->Couplers[1].CouplingFlag != Coupling::none ) {
        MoverParameters->InsideConsist = true;
    }
    else {
        MoverParameters->InsideConsist = false;
    }
    if( TestFlag( MoverParameters->AIFlag, sound::attachcoupler ) ) {
        auto *driver{ ctOwner ? ctOwner : Mechanik };
        if( driver != nullptr ) {
            driver->CheckVehicles( DriverOrder::connect );
        }
        ClearFlag( MoverParameters->AIFlag, sound::attachcoupler );
    }

    // napiecie sieci trakcyjnej
    if (MoverParameters->EnginePowerSource.SourceType == TPowerSource::CurrentCollector) { // dla EZT tylko silnikowy

        tmpTraction.TractionVoltage = std::max( std::abs( MoverParameters->PantRearVolt ), std::abs( MoverParameters->PantFrontVolt ) );
        // jeśli brak zasilania dłużej niż 0.2 sekundy (25km/h pod izolatorem daje 0.15s)
        // Ra 2F1H: prowizorka, trzeba przechować napięcie, żeby nie wywalało WS pod izolatorem
        if( tmpTraction.TractionVoltage > 0.0) {
            NoVoltTime = 0;
        }
        else {
            NoVoltTime += dt1;
            if( NoVoltTime <= 0.2 ) {
                tmpTraction.TractionVoltage = MoverParameters->PantographVoltage;
            }
            else {
                // jeśli jedzie
                if( MoverParameters->Vel > 0.5 ) {
                    // Ra 2014-07: doraźna blokada logowania zimnych lokomotyw - zrobić to trzeba inaczej
                    if( MoverParameters->Pantographs[end::front].is_active
                     || MoverParameters->Pantographs[end::rear].is_active ) {
                        if( MoverParameters->Mains // Ra 15-01: logować tylko, jeśli WS załączony
                         && MoverParameters->GetTrainsetHighVoltage() < 0.1f // yB 16-03: i nie jest to asynchron zasilany z daleka
                         && false == MoverParameters->GensetSelected() ) { // nor a dual-mode vehicle running on its genset
                            // Ra 15-01: bezwzględne współrzędne pantografu nie są dostępne więc lepiej się tego nie zaloguje
                            ErrorLog(
                                "Bad traction: " + MoverParameters->Name
                                + " lost power for " + to_string( NoVoltTime, 2 ) + " sec. at "
                                + to_string(vPosition) );
                        }
                    }
                }
            }
        }
    }
    else {
        tmpTraction.TractionVoltage = 0.95 * MoverParameters->EnginePowerSource.MaxVoltage;
    }
    tmpTraction.TractionFreq = 0;
    tmpTraction.TractionMaxCurrent = 7500; // Ra: chyba za dużo? powinno wywalać przy 1500
    tmpTraction.TractionResistivity = 0.3;

    MoverParameters->PantographVoltage = tmpTraction.TractionVoltage;
    // McZapkie: predkosc w torze przekazac do TrackParam
    // McZapkie: Vel ma wymiar [km/h] (absolutny), V ma wymiar [m/s], taka przyjalem notacje
    tp.Velmax = MyTrack->VelocityGet();

    if (Mechanik)
    { // Ra 2F3F: do Driver.cpp to przenieść?
        if( Mechanik->primary() ) {
            MoverParameters->EqvtPipePress = GetEPP(); // srednie cisnienie w PG
        }
		if (Mechanik->primary() &&
            (MoverParameters->EngineType == TEngineType::DieselEngine ||
             MoverParameters->EngineType == TEngineType::DieselElectric)
			&& MoverParameters->EIMCtrlType > 0) {
			MoverParameters->CheckEIMIC(dt1);
			if (MoverParameters->SpeedCtrl)
				MoverParameters->CheckSpeedCtrl(dt1);
			MoverParameters->eimic_real = std::min(MoverParameters->eimic,MoverParameters->eimicSpeedCtrl);
			MoverParameters->SendCtrlToNext(VehicleCommand::integratedController, MoverParameters->eimic_real, MoverParameters->CabActive);
		}
		if( Mechanik->primary()
         && MoverParameters->EngineType == TEngineType::ElectricInductionMotor ) {
            // jesli glowny i z asynchronami, to niech steruje hamulcem i napedem lacznie dla calego pociagu/ezt
			auto const kier = DirectionGet() * MoverParameters->CabOccupied > 0;
            auto FED { 0.0 };
            auto np { 0 };
            auto masa { 0.0 };
            auto FrED { 0.0 };
            auto masamax { 0.0 };
            auto FmaxPN { 0.0 };
			auto FfulED { 0.0 };
			auto FmaxED { 0.0 };
            auto Frj { 0.0 };
            auto osie { 0 };
			// 0a. ustal aktualna nastawe zadania sily napedowej i hamowania
			if( MoverParameters->Power < 1
             && ctOwner != nullptr ) {
				MoverParameters->MainCtrlPos = ctOwner->Controlling()->MainCtrlPos*MoverParameters->MainCtrlPosNo / std::max(1, ctOwner->Controlling()->MainCtrlPosNo);
				MoverParameters->SpeedCtrlValue = ctOwner->Controlling()->SpeedCtrlValue;
                MoverParameters->SpeedCtrlUnit.IsActive = ctOwner->Controlling()->SpeedCtrlUnit.IsActive;
			}
			MoverParameters->CheckEIMIC(dt1);
			MoverParameters->CheckSpeedCtrl(dt1);

			auto eimic = std::min(MoverParameters->eimic, MoverParameters->eimicSpeedCtrl);
			MoverParameters->eimic_real = eimic;
			// Elf: full braking at once in the lowest position, and in the emergency brake position below it
			if (MoverParameters->EIMCtrlType == 2 && MoverParameters->MainCtrlPos <= MoverParameters->EIMCtrlEmergencyPositions())
				eimic = -1.0;
			MoverParameters->SendCtrlToNext(VehicleCommand::integratedController, std::max(0., eimic), MoverParameters->CabActive);
			auto LBR = std::max(-eimic, 0.);
			auto eim_lb = Mechanik->AIControllFlag || !MoverParameters->LocHandleTimeTraxx ? 0 : MoverParameters->eim_localbrake;

			// 1. ustal wymagana sile hamowania calego pociagu
            //   - opoznienie moze byc ustalane na podstawie charakterystyki
            //   - opoznienie moze byc ustalane na podstawie mas i cisnien granicznych
            //

            // 2. ustal mozliwa do realizacji sile hamowania ED
            //   - w szczegolnosci powinien brac pod uwage rozne sily hamowania
            for (TDynamicObject *p = GetFirstDynamic(MoverParameters->CabOccupied < 0 ? 1 : 0, Coupling::control); p;
				kier ? p = p->Next(Coupling::control) : p = p->Prev(Coupling::control))
            {
                ++np;
                masamax +=
                    p->MoverParameters->MBPM
                    + ( p->MoverParameters->MBPM > 1.0 ?
                        0.0 :
                        p->MoverParameters->Mass )
                    + p->MoverParameters->Mred;
                auto const Nmax = ((p->MoverParameters->P2FTrans * p->MoverParameters->MaxBrakePress[0] -
                               p->MoverParameters->BrakeCylSpring) *
                                  p->MoverParameters->BrakeCylMult[0] -
                              p->MoverParameters->BrakeSlckAdj) *
                             p->MoverParameters->BrakeCylNo * p->MoverParameters->BrakeRigEff;
                FmaxPN += Nmax * p->MoverParameters->Hamulec->GetFC(
                              Nmax / (p->MoverParameters->NAxles * p->MoverParameters->NBpA),
                              p->MoverParameters->MED_Vref) *
                          1000; // sila hamowania pn
                FmaxED += p->MoverParameters->Mains && p->MoverParameters->DirActive != 0 && p->MoverParameters->inductionMotor.inverterRatio == 1.0 &&
				                  p->MoverParameters->inductionMotor.braking.maxForce * p->MoverParameters->NPoweredAxles > 0 ?
				              p->MoverParameters->inductionMotor.braking.maxForce * 1000 :
				              0; // chwilowy max ED -> do rozdzialu sil
                FED -= std::min(p->MoverParameters->inductionMotor.state.maxForce, 0.0) *
                       1000; // chwilowy max ED -> do rozdzialu sil
				FfulED = std::min(p->MoverParameters->inductionMotor.state.fullForce, 0.0) *
					1000; // chwilowy max ED -> do rozdzialu sil
				FrED -= std::min(p->MoverParameters->inductionMotor.state.maxForce, 0.0) *
                        1000; // chwilowo realizowane ED -> do pneumatyki
				Frj += std::max(p->MoverParameters->inductionMotor.state.maxForce, 0.0) *
					1000;// chwilowo realizowany napęd -> do utrzymującego
				masa += p->MoverParameters->TotalMass;
				osie += p->MoverParameters->NAxles;
			}
			double RapidMult = 1.0;
			if ((MoverParameters->BrakeDelays & bdelay_P + bdelay_R) == bdelay_P + bdelay_R
				&& MoverParameters->BrakeDelayFlag & bdelay_P)
				RapidMult = MoverParameters->RapidMult;

			auto const amax = RapidMult * std::min(FmaxPN / masamax, MoverParameters->MED_amax);

			Mechanik->fMedAmax = amax;
            auto doorisopen {
                false == MoverParameters->Doors.instances[side::left].is_closed
             || false == MoverParameters->Doors.instances[side::right].is_closed
             || ( MoverParameters->Doors.permit_needed
               && ( MoverParameters->Doors.instances[ side::left ].open_permit
                 || MoverParameters->Doors.instances[ side::right ].open_permit ) ) };
			doorisopen &= !(MoverParameters->ReleaseParkingBySpringBrakeWhenDoorIsOpen && MoverParameters->SpringBrake.IsActive);

            if (MoverParameters->Vel < 0.5 && (eimic < 0 || doorisopen || MoverParameters->Hamulec->GetEDBCP()))
            {
                MoverParameters->ShuntMode = true;
            }
            if (MoverParameters->ShuntMode)
            {
                MoverParameters->ShuntModeAllow = false == doorisopen &&
                                                  LBR < 0.01;
            }
            if( MoverParameters->Vel > 1 && false == doorisopen )
            {
                MoverParameters->ShuntMode = false;
                MoverParameters->ShuntModeAllow = MoverParameters->BrakePress > 0.2 &&
                                                  LBR < 0.01;
            }
            auto Fzad = amax * LBR * masa;
			if ((MoverParameters->BrakeCtrlPos == MoverParameters->Handle->GetPos(bh_EB)
				&& MoverParameters->inductionMotor.braking.cylinderPressureGain < 0.001) ||
                MoverParameters->EmergencyValveFlow > 0)
				Fzad = amax * masa; //pętla bezpieczeństwa - pełne służbowe
            if (MoverParameters->ScndS &&
                MoverParameters->Vel > MoverParameters->inductionMotor.braking.fullSpeed && FmaxED > 0)
            {
                Fzad = std::min(LBR * FmaxED, FfulED);
            }
            // the holding brake lets go once power has been demanded for its delay (SBDReleaseDelay, none by default)
            auto const holdingbrakereleased { MoverParameters->UpdateStopBrakeRelease( eimic, dt1 ) };
            if (MoverParameters->ShuntMode && false == holdingbrakereleased || doorisopen /*||
                (MoverParameters->V * MoverParameters->DirAbsolute < -0.2)*/)
            {
                auto const sbd { ( MoverParameters->SpringBrake.IsActive && MoverParameters->ReleaseParkingBySpringBrake ? 0.0 : MoverParameters->StopBrakeDecc ) };
                Fzad = std::max( Fzad, sbd * masa );
            }
			if (Fzad > 1 && !MEDLogFile.is_open() && MoverParameters->Vel > 1)
			{
                MEDLogFile.open(
                    "MEDLOGS/" + MoverParameters->Name + "_" + std::to_string( ++MEDLogCount ) + ".csv",
                    std::ios::in | std::ios::out | std::ios::trunc );
				MEDLogFile << "t\tVel\tMasa\tOsie\tFmaxPN\tFmaxED\tFfulED\tFrED\tFzad\tFzadED\tFzadPN";
				for(int k=1;k<=np;k++)
				{
					MEDLogFile << "\tBP" << k;
				}
				MEDLogFile << "\n";
				MEDLogFile.flush();
				MEDLogInactiveTime = 0;
				MEDLogTime = 0;
			}
            auto FzadED { 0.0 };
            if( LBR > MoverParameters->MED_MinBrakeReqED
             && ( MoverParameters->BrakeHandle == TBrakeHandle::MHZ_EN57 ?
                    (MoverParameters->BrakeOpModeFlag & bom_MED) != 0 :
                    MoverParameters->EpFuse ) ) {
                FzadED = std::min( Fzad, FmaxED );
            }
			/*/ELF - wdrazanie ED po powrocie na utrzymanie hamowania - do usuniecia
			if (MoverParameters->EIMCtrlType == 2 && MoverParameters->MainCtrlPos < 2 && MoverParameters->eimic > -0.999)
			{
				FzadED = std::min(FzadED, MED_oldFED);
			} //*/
			//opoznienie wdrazania ED
			if (FzadED > MED_oldFED)
			{
				if (MoverParameters->MED_ED_DelayTimer <= 0) {
					MoverParameters->MED_ED_DelayTimer += dt1;
					if (MoverParameters->MED_ED_DelayTimer > 0) {

					}
					else {
						FzadED = std::min(FzadED, MED_oldFED);
					}
				}
				else
				{
					FzadED = std::min(FzadED, MED_oldFED);
					MoverParameters->MED_ED_DelayTimer = FrED > 0 ? -MoverParameters->MED_ED_Delay2 : -MoverParameters->MED_ED_Delay1;
				}
			}
			if (MoverParameters->BrakeCtrlPos == MoverParameters->Handle->GetPos(bh_EB)
				&& MoverParameters->inductionMotor.braking.cylinderPressureGain < 0.001)
				FzadED = 0; //pętla bezpieczeństwa - bez ED
            auto const FzadPN = Fzad - FrED * MoverParameters->MED_FrED_factor;
			std::vector<bool> PrzekrF( np );
			float nPrzekrF = 0;
			bool test = true;
			std::vector<float> FzED( np );
			std::vector<float> FzEP( np );
			std::vector<float> FmaxEP( np );
			// 3. ustaw pojazdom sile hamowania ED
            //   - proporcjonalnie do mozliwosci

            // 4. ustal potrzebne dohamowanie pneumatyczne
            //   - od sily zadanej trzeba odjac realizowana przez ED
            // 5. w razie potrzeby wlacz hamulec utrzymujacy
            //   - gdy zahamowany ma ponizej 2 km/h
            // 6. ustaw pojazdom sile hamowania ep
            //   - proporcjonalnie do masy, do liczby osi, rowne cisnienia - jak
            //   bedzie, tak bedzie dobrze
            float Fpoj = 0; // MoverParameters->CabOccupied < 0
            ////ALGORYTM 2 - KAZDEMU PO ROWNO, ale nie wiecej niz eped * masa
            // 1. najpierw daj kazdemu tyle samo
            int i = 0;
			for (TDynamicObject *p = GetFirstDynamic(MoverParameters->CabOccupied < 0 ? 1 : 0, Coupling::control); p;
				p = kier == true ? p->Next(Coupling::control) : p->Prev(Coupling::control) )
			{
                auto const Nmax = ((p->MoverParameters->P2FTrans * p->MoverParameters->MaxBrakePress[0] -
                               p->MoverParameters->BrakeCylSpring) *
                                  p->MoverParameters->BrakeCylMult[0] -
                              p->MoverParameters->BrakeSlckAdj) *
                             p->MoverParameters->BrakeCylNo * p->MoverParameters->BrakeRigEff;
                FmaxEP[i] = Nmax *
                            p->MoverParameters->Hamulec->GetFC(
                                Nmax / (p->MoverParameters->NAxles * p->MoverParameters->NBpA),
                                p->MoverParameters->MED_Vref) *
                            1000; // sila hamowania pn

                PrzekrF[i] = false;
                FzED[i] = FmaxED > 0 ? FzadED / FmaxED : 0;
                p->MoverParameters->AnPos =
                    MoverParameters->ScndS ? LBR : FzED[i];
                FzEP[ i ] = static_cast<float>( FzadPN * p->MoverParameters->NAxles ) / static_cast<float>( osie );
                ++i;
                p->MoverParameters->ShuntMode = MoverParameters->ShuntMode;
                p->MoverParameters->ShuntModeAllow = MoverParameters->ShuntModeAllow;
            }
            while (test)
            {
                test = false;
                i = 0;
                float przek = 0;
                for (TDynamicObject *p = GetFirstDynamic(MoverParameters->CabOccupied < 0 ? 1 : 0, Coupling::control); p;
                     p = kier == true ? p->Next(Coupling::control) : p->Prev(Coupling::control) )
                {
                    if (FzEP[i] > 0.01 &&
                        FzEP[i] > p->MoverParameters->TotalMass * p->MoverParameters->inductionMotor.braking.maxEpDeceleration + std::min(p->MoverParameters->inductionMotor.state.maxForce, 0.) * 1000 &&
                        !PrzekrF[i])
                    {
                        float przek1 = -std::min(p->MoverParameters->inductionMotor.state.maxForce, 0.) * 1000 +
                                       FzEP[i] -
                                       p->MoverParameters->TotalMass *
                                           p->MoverParameters->inductionMotor.braking.maxEpDeceleration * 0.999;
                        PrzekrF[i] = true;
                        test = true;
                        nPrzekrF++;
                        przek1 = std::min(przek1, FzEP[i]);
                        FzEP[i] -= przek1;
                        if (FzEP[i] < 0)
                            FzEP[i] = 0;
                        przek += przek1;
                    }
                    ++i;
                }
                i = 0;
                przek = przek / (np - nPrzekrF);
                for (TDynamicObject *p = GetFirstDynamic(MoverParameters->CabOccupied < 0 ? 1 : 0, Coupling::control); p;
                     true == kier ? p = p->Next(Coupling::control) : p = p->Prev(Coupling::control))
                {
                    if (!PrzekrF[i])
                    {
                        FzEP[i] += przek;
                    }
                    ++i;
                }
            }
            i = 0;
            for (TDynamicObject *p = GetFirstDynamic(MoverParameters->CabOccupied < 0 ? 1 : 0, Coupling::control); p;
                 true == kier ? p = p->Next(Coupling::control) : p = p->Prev(Coupling::control))
            {
                float Nmax = ((p->MoverParameters->P2FTrans * p->MoverParameters->MaxBrakePress[0] -
                               p->MoverParameters->BrakeCylSpring) *
                                  p->MoverParameters->BrakeCylMult[0] -
                              p->MoverParameters->BrakeSlckAdj) *
                             p->MoverParameters->BrakeCylNo * p->MoverParameters->BrakeRigEff;
				if (FrED > 0.1)
					p->MoverParameters->MED_EPVC_CurrentTime = 0;
				else
					p->MoverParameters->MED_EPVC_CurrentTime += dt1;
				bool EPVC = p->MoverParameters->MED_EPVC && (p->MoverParameters->MED_EPVC_Time < 0 || p->MoverParameters->MED_EPVC_CurrentTime < p->MoverParameters->MED_EPVC_Time);
				float VelC = EPVC ? safe_clamp(p->MoverParameters->Vel, p->MoverParameters->MED_Vmin, p->MoverParameters->MED_Vmax) : p->MoverParameters->MED_Vref;//korekcja EP po prędkości
                float FmaxPoj = Nmax *
					p->MoverParameters->Hamulec->GetFC(
						Nmax / (p->MoverParameters->NAxles * p->MoverParameters->NBpA), VelC) *
					1000; // sila hamowania pn
				p->MoverParameters->LocalBrakePosAEIM = FzEP[i] / FmaxPoj;
				if (p->MoverParameters->LocalBrakePosAEIM>0.009)
					if (p->MoverParameters->P2FTrans * p->MoverParameters->BrakeCylMult[0] *
						p->MoverParameters->MaxBrakePress[0] != 0)
					{
						float x = (p->MoverParameters->BrakeSlckAdj / p->MoverParameters->BrakeCylMult[0] +
							p->MoverParameters->BrakeCylSpring) / (p->MoverParameters->P2FTrans *
								p->MoverParameters->MaxBrakePress[0]);
						p->MoverParameters->LocalBrakePosAEIM = x + (1 - x) * p->MoverParameters->LocalBrakePosAEIM;
					}
					else
						p->MoverParameters->LocalBrakePosAEIM = p->MoverParameters->LocalBrakePosAEIM;
				else
					p->MoverParameters->LocalBrakePosAEIM = 0;
				if (p->MoverParameters->LocHandleTimeTraxx)
				{
					p->MoverParameters->eim_localbrake = eim_lb;
					p->MoverParameters->LocalBrakePosAEIM = std::max(p->MoverParameters->LocalBrakePosAEIM, eim_lb);
				}
				++i;
			}

			MED[0][0] = masa*0.001;
			MED[0][1] = amax;
			MED[0][2] = Fzad*0.001;
			MED[0][3] = FmaxPN*0.001;
			MED[0][4] = FmaxED*0.001;
			MED[0][5] = FrED*0.001;
			MED[0][6] = FzadPN*0.001;
			MED[0][7] = nPrzekrF;

			if (MEDLogFile.is_open())
			{
				MEDLogFile << MEDLogTime << "\t" << MoverParameters->Vel << "\t" << masa*0.001 << "\t" << osie << "\t" << FmaxPN*0.001 << "\t" << FmaxED*0.001 << "\t"
					<< FfulED*0.001 << "\t" << FrED*0.001 << "\t" << Fzad*0.001 << "\t" << FzadED*0.001 << "\t" << FzadPN*0.001;
				for (TDynamicObject *p = GetFirstDynamic(MoverParameters->CabOccupied < 0 ? 1 : 0, Coupling::control); p;
					true == kier ? p = p->Next(Coupling::control) : p = p->Prev(Coupling::control))
				{
					MEDLogFile << "\t" << p->MoverParameters->BrakePress;
				}
				MEDLogFile << "\n";
				if (floor(MEDLogTime + dt1) > floor(MEDLogTime))
				{
					MEDLogFile.flush();
				}
				MEDLogTime += dt1;

				if (MoverParameters->Vel < 0.1 || MoverParameters->MainCtrlPowerPos() > 0)
				{
					MEDLogInactiveTime += dt1;
				}
				else
				{
					MEDLogInactiveTime = 0;
				}
				if (MEDLogInactiveTime > 5)
				{
					MEDLogFile.flush();
					MEDLogFile.close();
				}

			}

			MED_oldFED = FzadED;
        }

        Timer::subsystem.vehicles_ai.start();
        Mechanik->Update(dt1); // przebłyski świadomości AI
        Timer::subsystem.vehicles_ai.stop();
    }

    // fragment "z EXE Kursa"
    if( MoverParameters->Mains ) { // nie wchodzić w funkcję bez potrzeby
        if( false == (MoverParameters->Power24vIsAvailable || MoverParameters->Power110vIsAvailable)
/*
          // NOTE: disabled on account of multi-unit setups, where the unmanned unit wouldn't be affected
            && ( Controller == Humandriver )
*/
            && MoverParameters->EngineType != TEngineType::DieselEngine
            && MoverParameters->EngineType != TEngineType::WheelsDriven ) { // jeśli bateria wyłączona, a nie diesel ani drezyna reczna
            if( MoverParameters->MainSwitch( false, MoverParameters->TrainType == train_type::emu ? range_t::unit : range_t::local ) ) {
                // wyłączyć zasilanie
                // NOTE: we turn off entire EMU, but only the affected unit for other multi-unit consists
                MoverParameters->EventFlag = true;
                // drop pantographs
                // NOTE: this isn't universal behaviour
                // TODO: have this dependant on .fiz-driven flag
                // NOTE: moved to pantspeed calculation part a little later in the function. all remarks and todo still apply
/*
                MoverParameters->PantFront( false, ( MoverParameters->TrainType == train_type::emu ? range::unit : range::local ) );
                MoverParameters->PantRear( false, ( MoverParameters->TrainType == train_type::emu ? range::unit : range::local ) );
*/
            }
        }
    }

    // przekazanie pozycji do fizyki
    // NOTE: coordinate system swap
    // TODO: replace with regular glm vectors
    TLocation const l {
        -vPosition.x,
         vPosition.z,
         vPosition.y };
    TRotation const r {
        0.0,
        0.0,
        modelRot.z };
    // McZapkie-260202 - dMoveLen przyda sie przy stukocie kol
    dDOMoveLen = GetdMoveLen() + MoverParameters->ComputeMovement(dt, dt1, ts, tp, tmpTraction, l, r);
    if( Mechanik ) {
        // dodanie aktualnego przemieszczenia
        Mechanik->MoveDistanceAdd( dDOMoveLen );
    }
    if( simulation::Train != nullptr
     && simulation::Train->Dynamic() == this ) {
        // update distance meter in user-controlled cab
        // TBD: place the meter on mover logic level?
        simulation::Train->add_distance( dDOMoveLen );
    }
    // physics.derailment new: derailed, the vehicle leaves the track and goes on the ground
    if( MoverParameters->leaves_track() ) {
        begin_off_track();
    }
    m_offtracktime = dt1;
    glm::dvec3 old_pos = vPosition;
    Move(dDOMoveLen);
    update_off_track( dt1 );

	m_future_movement = (vPosition - old_pos) / dt1 * Timer::GetDeltaRenderTime();

    if (!bEnabled) // usuwane pojazdy nie mają toru
    { // pojazd do usunięcia
        bDynamicRemove = true; // sprawdzić
        return false;
    }
    ResetdMoveLen();

    // McZapkie-260202
    // tupot mew, tfu, stukot kol:
    if( MyTrack->fSoundDistance != -1 ) {

        if( MyTrack->fSoundDistance != dRailLength ) {
            if( dRailLength > 0.0 ) {
                for( auto &axle : m_axlesounds ) {
                    axle.distance = axle.offset;
                    axle.atjoints = false;
                }
            }
            dRailLength = MyTrack->fSoundDistance;
        }
        if( dRailLength > 0.0 ) {
            if( MoverParameters->Vel > 0 ) {
                // TODO: track quality and/or environment factors as separate subroutine
                auto volume =
                    std::lerp(
                        0.8, 1.2,
                        std::clamp(
                            MyTrack->iQualityFlag / 10.0,
                            0.0, 1.5 ) );
                switch( MyTrack->eEnvironment ) {
                    case e_tunnel: {
                        volume *= 1.3;
                        break;
                    }
                    case  e_bridge: {
                        volume *= 1.5;
                        break;
                    }
                    default: {
                        break;
                    }
                }
                if( dRailLength > 0.0 ) {
                    auto axleindex { 0 };
                    auto const directioninconsist { (
                        ctOwner == nullptr ?
                            1 :
                            ctOwner->Vehicle()->DirectionGet() == DirectionGet() ? 1 : -1 ) };
                    for( auto &axle : m_axlesounds ) {
                        if( axle_passed_joint( axle, dDOMoveLen, directioninconsist )
                         && MoverParameters->Vel > 0.1 ) {
                            // NOTE: for combined clatter sound we supply 1/100th of actual value, as the sound module converts does the opposite, converting received (typically) 0-1 values to 0-100 range
                            auto const frequency = true == axle.clatter.is_combined() ? MoverParameters->Vel * 0.01 : 1.0;
                            axle.clatter
                                .pitch( frequency )
                                .gain( volume )
                                .play();
							// crude bump simulation, drop down on even axles, move back up on
							// the odd ones
							// MoverParameters->AccVert += (MoverParameters->Vel*0.1f) *
							// the arguments keep the order they had up to 26.05.1, 0 in place of the value and the figure in place of the
							// upper limit, so they work out as the old clamp worked them out up to 26.05.1: the headroom is
							// Vmax - (Vel + 0.32 Vmax) whatever its sign, and the bump is capped from above only
							double const headroom = safe_clamp( 0.0, MoverParameters->Vmax, MoverParameters->Vmax - ( MoverParameters->Vel + MoverParameters->Vmax * 0.32f ) );
							double const jolt = headroom * .05f * ( MyTrack->iDamageFlag * 0.25f );
							if (MyTrack->eType == tt_Normal)
							{
								MoverParameters->AccVert += safe_clamp( 0.0, 4.0, jolt );
							}
							else if (MyTrack->eType == tt_Switch)
							{
								double const accHorizontal = safe_clamp( 0.0, 1.0, jolt ) * ((axleindex % 2) != 0 ? 1 : -1);
								MoverParameters->AccS += accHorizontal;
								MoverParameters->AccN += accHorizontal;
								MoverParameters->AccVert += safe_clamp( 0.0, 2.0, jolt );
							}
                        }
                        ++axleindex;
                    }
                }
            }
        }
    }
    // McZapkie-260202 end

    // fragment z EXE Kursa
    /* if (MoverParameters->TrainType==train_type::et42)
         {
         if ((MoverParameters->DynamicBrakeType=dbrake_switch) &&
       ((MoverParameters->BrakePress >
       0.2) || ( MoverParameters->PipePress < 0.36 )))
                      {
                      MoverParameters->StLinFlag=true;
                      }
         else
         if ((MoverParameters->DynamicBrakeType=dbrake_switch) &&
       (MoverParameters->BrakePress <
       0.1))
                      {
                      MoverParameters->StLinFlag=false;

                      }
         }   */

    if (MoverParameters->Vel != 0)
    { // McZapkie-050402: krecenie kolami:
        glm::dvec3 old_wheels = glm::dvec3(dWheelAngle[0], dWheelAngle[1], dWheelAngle[2]);

        dWheelAngle[0] += 114.59155902616464175359630962821 * MoverParameters->V * dt1 /
                          MoverParameters->WheelDiameterL; // przednie toczne
        dWheelAngle[1] += MoverParameters->nrot * dt1 * 360.0; // napędne
        dWheelAngle[2] += 114.59155902616464175359630962821 * MoverParameters->V * dt1 /
                          MoverParameters->WheelDiameterT; // tylne toczne


		m_future_wheels_angle = (glm::dvec3(dWheelAngle[0], dWheelAngle[1], dWheelAngle[2]) - old_wheels) / dt1 * Timer::GetDeltaRenderTime();

        dWheelAngle[0] = clamp_circular( dWheelAngle[0] );
        dWheelAngle[1] = clamp_circular( dWheelAngle[1] );
        dWheelAngle[2] = clamp_circular( dWheelAngle[2] );
    }
    if (pants) // pantograf może być w wagonie kuchennym albo pojeździe rewizyjnym (np. SR61)
    { // przeliczanie kątów dla pantografów
        double k; // tymczasowy kąt
        double PantDiff;
        TAnimPant *p; // wskaźnik do obiektu danych pantografu
        // prąd pobierany przez pojazd z sieci: trakcyjny (on the genset of a dual-mode vehicle none) i TotalCurrent
        // TotalCurrent to bedzie prad nietrakcyjny (niezwiazany z napedem)
        double fCurrent = MoverParameters->CatenaryCurrent();
        // fCurrent+=fabs(MoverParameters->Voltage)*1e-6; //prąd płynący przez woltomierz, rozładowuje kondensator orgromowy 4µF
/*
        double fPantCurrent = fCurrent; // normalnie cały prąd przez jeden pantograf
        if (iAnimType[ANIM_PANTS] > 1) { // a jeśli są dwa pantografy //Ra 1014-11: proteza, trzeba zrobić sensowniej
            if (pants[0].fParamPants->hvPowerWire && pants[1].fParamPants->hvPowerWire) { // i oba podłączone do drutów
                fPantCurrent = fCurrent * 0.5; // to dzielimy prąd równo na oba (trochę bez sensu, ale lepiej tak niż podwoić prąd)
            }
        }
*/
        // test whether more than one pantograph touches the wire
        // NOTE: we're duplicating lot of code from below
        // TODO: clean this up
        auto activepantographs { 0 };
        for( int idx = 0; idx < iAnimType[ ANIM_PANTS ]; ++idx ) {
            auto const *pantograph { pants[ idx ].fParamPants };
            if( Global.bLiveTraction == false ) {
                if( pantograph->PantWys >= 1.2 ) {
                    ++activepantographs;
                }
            }
            else {
                if( pantograph->hvPowerWire != nullptr
                 && true == MoverParameters->Pantographs[end::front].is_active
                 && pantograph->PantTraction - pantograph->PantWys < 0.01 ) { // tolerancja niedolegania
                    ++activepantographs;
                }
            }
        }
        auto const fPantCurrent { fCurrent / std::max( 1, activepantographs ) };

        for (int i = 0; i < iAnimType[ANIM_PANTS]; ++i)
        { // pętla po wszystkich pantografach
            p = pants[i].fParamPants;
            if (p->PantWys < 0)
            { // patograf został połamany, liczony nie będzie
                if (p->fAngleL > p->fAngleL0)
                    p->fAngleL -= 0.2 * dt1; // nieco szybciej niż jak dla opuszczania
                if (p->fAngleL < p->fAngleL0)
                    p->fAngleL = p->fAngleL0; // kąt graniczny
                if (p->fAngleU < M_PI)
                    p->fAngleU += 0.5 * dt1; // górne się musi ruszać szybciej.
                if (p->fAngleU > M_PI)
                    p->fAngleU = M_PI;
                if (i & 1) // zgłoszono, że po połamaniu potrafi zostać zasilanie
                    MoverParameters->PantRearVolt = 0.0;
                else
                    MoverParameters->PantFrontVolt = 0.0;
                continue; // reszta wtedy nie jest wykonywana
            }
            PantDiff = p->PantTraction - p->PantWys; // docelowy-aktualny
            switch (i) // numer pantografu
            { // trzeba usunąć to rozróżnienie
            case 0:
                if (Global.bLiveTraction == false
                 && p->hvPowerWire == nullptr ) {
                    // jeśli nie ma drutu, może pooszukiwać
                    MoverParameters->PantFrontVolt =
                        p->PantWys >= 1.2 ?
                            0.95 * MoverParameters->EnginePowerSource.MaxVoltage :
                            0.0;
                }
                else if( true == MoverParameters->Pantographs[end::front].is_active
                      && PantDiff < 0.01 ) // tolerancja niedolegania
                {
                    if (p->hvPowerWire) {
                        auto const lastvoltage { MoverParameters->PantFrontVolt };
                        // TODO: wyliczyć trzeba prąd przypadający na pantograf i wstawić do GetVoltage()
                        if( lastvoltage == 0.0 ) {
                            // HACK: retrieve the wire voltage for calculations down the road without blowing up the supply
                            MoverParameters->PantFrontVolt = p->hvPowerWire->VoltageGet( MoverParameters->PantographVoltage, 0.0 );
                        }
                        else {
                            MoverParameters->PantFrontVolt = p->hvPowerWire->VoltageGet( MoverParameters->PantographVoltage, fPantCurrent );
                            if( MoverParameters->PantFrontVolt > 0.0 ) {
                                fCurrent -= fPantCurrent; // taki prąd płynie przez powyższy pantograf (unless it doesn't)
                            }
                        }
                        // TODO: refactor reaction to voltage change to mover as sound event for specific pantograph
                        if( lastvoltage == 0.0
                         && MoverParameters->PantFrontVolt > 0.0 ) {
                            for( auto &pantograph : m_pantographsounds ) {
                                if( pantograph.sPantUp.offset().z > 0 ) {
                                    // limit to pantographs located in the front half of the vehicle
                                    pantograph.sPantUp.play( sound_flags::exclusive );
                                }
                            }
                        }
                    }
                    else
                        MoverParameters->PantFrontVolt = 0.0;
                }
                else
                    MoverParameters->PantFrontVolt = 0.0;
                ( fPantCurrent > 0.0 ? MoverParameters->EnergyMeter.first : MoverParameters->EnergyMeter.second ) += MoverParameters->PantRearVolt * fPantCurrent * dt1 / 3600000.0;
                break;
            case 1:
                if( false == Global.bLiveTraction
                 && nullptr == p->hvPowerWire ) {
                    // jeśli nie ma drutu, może pooszukiwać
                    MoverParameters->PantRearVolt =
                        p->PantWys >= 1.2 ?
                            0.95 * MoverParameters->EnginePowerSource.MaxVoltage :
                            0.0;
                }
                else if ( true == MoverParameters->Pantographs[end::rear].is_active
                       && PantDiff < 0.01 )
                {
                    if (p->hvPowerWire) {
                        auto const lastvoltage { MoverParameters->PantRearVolt };
                        // TODO: wyliczyć trzeba prąd przypadający na pantograf i wstawić do GetVoltage()
                        if( lastvoltage == 0.0 ) {
                            // HACK: retrieve the wire voltage for calculations down the road without blowing up the supply
                            MoverParameters->PantRearVolt = p->hvPowerWire->VoltageGet( MoverParameters->PantographVoltage, 0.0 );
                        }
                        else {
                            MoverParameters->PantRearVolt = p->hvPowerWire->VoltageGet( MoverParameters->PantographVoltage, fPantCurrent );
                            if( MoverParameters->PantRearVolt > 0.0 ) {
                                fCurrent -= fPantCurrent; // taki prąd płynie przez powyższy pantograf (unless it doesn't)
                            }
                        }
                        // TODO: refactor reaction to voltage change to mover as sound event for specific pantograph
                        if( lastvoltage == 0.0
                         && MoverParameters->PantRearVolt > 0.0 ) {
                            for( auto &pantograph : m_pantographsounds ) {
                                if( pantograph.sPantUp.offset().z < 0 ) {
                                    // limit to pantographs located in the rear half of the vehicle
                                    pantograph.sPantUp.play( sound_flags::exclusive );
                                }
                            }
                        }
                    }
                    else
                        MoverParameters->PantRearVolt = 0.0;
                }
                else {
//                    Global.iPause ^= 2;
                    MoverParameters->PantRearVolt = 0.0;
                }
                ( fPantCurrent > 0.0 ? MoverParameters->EnergyMeter.first : MoverParameters->EnergyMeter.second ) += MoverParameters->PantFrontVolt * fPantCurrent * dt1 / 3600000.0;
                break;
            } // pozostałe na razie nie obsługiwane
            if( MoverParameters->PantPress > (
                    MoverParameters->TrainType == train_type::emu ?
                        2.45 : // Ra 2013-12: Niebugocław mówi, że w EZT podnoszą się przy 2.5
                        3.45 ) ) {
                // z EXE Kursa
                // Ra: wysokość zależy od ciśnienia !!!
                pantspeedfactor = 0.015 * MoverParameters->PantPress * dt1;
            }
            else {
                pantspeedfactor = 0.0;
            }
            if( false == ( MoverParameters->Power24vIsAvailable || MoverParameters->Power110vIsAvailable ) ) {
                pantspeedfactor = 0.0;
            }
            pantspeedfactor = std::max( 0.0, pantspeedfactor );
            k = p->fAngleL;
            if( pantspeedfactor > 0.0
             && MoverParameters->Pantographs[i].is_active )// jeśli ma być podniesiony
            {
                if (PantDiff > 0.001) // jeśli nie dolega do drutu
                { // jeśli poprzednia wysokość jest mniejsza niż pożądana, zwiększyć kąt dolnego
                    // ramienia zgodnie z ciśnieniem
                    if (pantspeedfactor >
                        0.55 * PantDiff) // 0.55 to około pochodna kąta po wysokości
                        k += 0.55 * PantDiff; // ograniczenie "skoku" w danej klatce
                    else
                        k += pantspeedfactor; // dolne ramię
                    // jeśli przekroczono kąt graniczny, zablokować pantograf
                    // (wymaga interwencji pociągu sieciowego)
                }
                else if (PantDiff < -0.001) {
                    // drut się obniżył albo pantograf podniesiony za wysoko
                    // jeśli wysokość jest zbyt duża, wyznaczyć zmniejszenie kąta
                    // jeśli zmniejszenie kąta jest zbyt duże, przejść do trybu łamania pantografu
                    // if (PantFrontDiff<-0.05) //skok w dół o 5cm daje złąmanie pantografu
                    k += 0.4 * PantDiff; // mniej niż pochodna kąta po wysokości
                } // jeśli wysokość jest dobra, nic więcej nie liczyć
            }
            else
            { // jeśli ma być na dole
                if (k > p->fAngleL0) // jeśli wyżej niż położenie wyjściowe
                    k -= 0.15 * dt1; // ruch w dół
                if (k < p->fAngleL0)
                    k = p->fAngleL0; // położenie minimalne
            }
            if (k != p->fAngleL)
            { //żeby nie liczyć w kilku miejscach ani gdy nie potrzeba
                if (k + p->fAngleU < M_PI)
                { // o ile nie został osiągnięty kąt maksymalny
                    p->fAngleL = k; // zmieniony kąt
                    // wyliczyć kąt górnego ramienia z wzoru (a)cosinusowego
                    //=acos((b*cos()+c)/a)
                    // p->dPantAngleT=acos((1.22*cos(k)+0.535)/1.755); //górne ramię
                    p->fAngleU = acos((p->fLenL1 * cos(k) + p->fHoriz) / p->fLenU1); // górne ramię
                    // wyliczyć aktualną wysokość z wzoru sinusowego
                    // h=a*sin()+b*sin()
                    // wysokość całości
                    p->PantWys = p->fLenL1 * sin(k) + p->fLenU1 * sin(p->fAngleU) + p->fHeight;
                }
            }
        } // koniec pętli po pantografach
        // TBD, TODO: generate sound event during mover update instead?
        if( MoverParameters->Pantographs[end::front].sound_event != MoverParameters->Pantographs[ end::front ].is_active ) {
            if( MoverParameters->Pantographs[ end::front ].is_active ) {
                // pantograph moving up
                // TBD: add a sound?
            }
            else {
                // pantograph dropping
                for( auto &pantograph : m_pantographsounds ) {
                    if( pantograph.sPantDown.offset().z > 0 ) {
                        // limit to pantographs located in the front half of the vehicle
                        pantograph.sPantDown.play( sound_flags::exclusive );
                    }
                }
            }
            MoverParameters->Pantographs[ end::front ].sound_event = MoverParameters->Pantographs[ end::front ].is_active;
        }
        if( MoverParameters->Pantographs[ end::rear ].sound_event != MoverParameters->Pantographs[ end::rear ].is_active ) {
            if( MoverParameters->Pantographs[ end::rear ].is_active ) {
                // pantograph moving up
                // TBD: add a sound?
            }
            else {
                // pantograph dropping
                for( auto &pantograph : m_pantographsounds ) {
                    if( pantograph.sPantDown.offset().z < 0 ) {
                        // limit to pantographs located in the front half of the vehicle
                        pantograph.sPantDown.play( sound_flags::exclusive );
                    }
                }
            }
            MoverParameters->Pantographs[ end::rear ].sound_event = MoverParameters->Pantographs[ end::rear ].is_active;
        }
    }
    else if (MoverParameters->EnginePowerSource.SourceType == TPowerSource::InternalSource)
        if (MoverParameters->EnginePowerSource.PowerType == TPowerType::SteamPower)
        // if (smPatykird1[0])
        { // Ra: animacja rozrządu parowozu, na razie nieoptymalizowane
            /* //Ra: tymczasowo wyłączone ze względu na porządkowanie animacji
               pantografów
               double fi,dx,c2,ka,kc;
               double sin_fi,cos_fi;
               double L1=1.6688888888888889;
               double L2=5.6666666666666667; //2550/450
               double Lc=0.4;
               double L=5.686422222; //2558.89/450
               double G1,G2,G3,ksi,sin_ksi,gam;
               double G1_2,G2_2,G3_2; //kwadraty
               //ruch tłoków oraz korbowodów
               for (int i=0;i<=1;++i)
               {//obie strony w ten sam sposób
                fi=DegToRad(dWheelAngle[1]+(i?pant2x:pant1x)); //kąt obrotu koła dla
               tłoka 1
                sin_fi=sin(fi);
                cos_fi=cos(fi);
                dx=panty*cos_fi+sqrt(panth*panth-panty*panty*sin_fi*sin_fi)-panth;
               //nieoptymalne
                if (smPatykird1[i]) //na razie zabezpieczenie
                 smPatykird1[i]->SetTranslate(float3(dx,0,0));
                ka=-asin(panty/panth)*sin_fi;
                if (smPatykirg1[i]) //na razie zabezpieczenie
                 smPatykirg1[i]->SetRotateXYZ(vector3(RadToDeg(ka),0,0));
                //smPatykirg1[0]->SetRotate(float3(0,1,0),RadToDeg(fi)); //obracamy
                //ruch drążka mimośrodkowego oraz jarzma
                //korzystałem z pliku PDF "mm.pdf" (opis czworoboku
               korbowo-wahaczowego):
                //"MECHANIKA MASZYN. Szkic wykładu i laboratorium komputerowego."
                //Prof. dr hab. inż. Jerzy Zajączkowski, 2007, Politechnika Łódzka
                //L1 - wysokość (w pionie) osi jarzma ponad osią koła
                //L2 - odległość w poziomie osi jarzma od osi koła
                //Lc - długość korby mimośrodu na kole
                //Lr - promień jarzma =1.0 (pozostałe przeliczone proporcjonalnie)
                //L - długość drążka mimośrodowego
                //fi - kąt obrotu koła
                //ksi - kąt obrotu jarzma (od pionu)
                //gam - odchylenie drążka mimośrodowego od poziomu
                //G1=(Lr*Lr+L1*L1+L2*L2+Kc*Lc-L*L-2.0*Lc*L2*cos(fi)+2.0*Lc*L1*sin(fi))/(Lr*Lr);
                //G2=2.0*(L2-Lc*cos(fi))/Lr;
                //G3=2.0*(L1-Lc*sin(fi))/Lr;
                fi=DegToRad(dWheelAngle[1]+(i?pant2x:pant1x)-96.77416667); //kąt
               obrotu koła dla
               tłoka 1
                //1) dla dWheelAngle[1]=0° korba jest w dół, a mimośród w stronę
               jarzma, czyli
               fi=-7°
                //2) dla dWheelAngle[1]=90° korba jest do tyłu, a mimośród w dół,
               czyli fi=83°
                sin_fi=sin(fi);
                cos_fi=cos(fi);
                G1=(1.0+L1*L1+L2*L2+Lc*Lc-L*L-2.0*Lc*L2*cos_fi+2.0*Lc*L1*sin_fi);
                G1_2=G1*G1;
                G2=2.0*(L2-Lc*cos_fi);
                G2_2=G2*G2;
                G3=2.0*(L1-Lc*sin_fi);
                G3_2=G3*G3;
                sin_ksi=(G1*G2-G3*_fm_sqrt(G2_2+G3_2-G1_2))/(G2_2+G3_2); //x1 (minus
               delta)
                ksi=asin(sin_ksi); //kąt jarzma
                if (smPatykirg2[i])
                 smPatykirg2[i]->SetRotateXYZ(vector3(RadToDeg(ksi),0,0)); //obrócenie
               jarzma
                //1) ksi=-23°, gam=
                //2) ksi=10°, gam=
                //gam=acos((L2-sin_ksi-Lc*cos_fi)/L); //kąt od poziomu, liczony
               względem poziomu
                //gam=asin((L1-cos_ksi-Lc*sin_fi)/L); //kąt od poziomu, liczony
               względem pionu
                gam=atan2((L1-cos(ksi)+Lc*sin_fi),(L2-sin_ksi+Lc*cos_fi)); //kąt od
               poziomu
                if (smPatykird2[i]) //na razie zabezpieczenie
                 smPatykird2[i]->SetRotateXYZ(vector3(RadToDeg(-gam-ksi),0,0));
               //obrócenie drążka
               mimośrodowego
               }
            */
        }

	if (MoverParameters->EnginePowerSource.SourceType == TPowerSource::CurrentCollector
	        && MoverParameters->EnginePowerSource.CollectorParameters.FakePower) {
		MoverParameters->PantRearVolt = 0.95 * MoverParameters->EnginePowerSource.MaxVoltage;
		MoverParameters->PantFrontVolt = 0.95 * MoverParameters->EnginePowerSource.MaxVoltage;
	}

    // wipers
	if (dWiperPos.size() > 0) // tylko dla wozow ze zdefiniowanymi wycierakami
    {
		for (int i = 0; i < dWiperPos.size(); i++) // iteracja po kazdej wycieraczce
		{
			bool wipersActive;
			auto const bytesum = MoverParameters->WiperList[MoverParameters->wiperSwitchPos].byteSum;
            // rozroznienie kabinowe
            if (MoverParameters->CabActive == 1)
            {
				wipersActive = (bytesum & 1 << i) != 0 && MoverParameters->Battery;
            }
			else if (MoverParameters->CabActive == -1)
            {
                // odwroconie indexow wycieraczek
				wipersActive = (bytesum & 1 << dWiperPos.size() - 1 - i) != 0 && MoverParameters->Battery;
            }
            else {
				wipersActive = false;
            }

			auto const currentWiperParams = MoverParameters->WiperList[workingSwitchPos[i]];

            wiperOutTimer[i] += dt1; // aktualizujemy zegarek
			wiperParkTimer[i] += dt1; // aktualizujemy zegarek

			if (wipersActive || dWiperPos[i] > 0.0) // zeby wrocily do trybu park
			{
                if (dWiperPos[i] > 0.0 && !wipersActive) // bezwzgledny powrot do zera
                {
					dWiperPos[i] = std::max(0.0, dWiperPos[i] - 1.f / currentWiperParams.WiperSpeed * dt1);
                }
                else {

					if (dWiperPos[i] < 1.0 && !wiperDirection[i] && wiperParkTimer[i] > currentWiperParams.interval)
					// go out
					{
                        if (!sWiperFromParkPlayed)
                        {
							sWiperFromPark.play(sound_flags::exclusive);
							sWiperFromParkPlayed = true;
                        }
						dWiperPos[i] = std::min(1.0, dWiperPos[i] + 1.f / currentWiperParams.WiperSpeed * dt1);
					}
					if (dWiperPos[i] > 0.0 && wiperDirection[i] && wiperOutTimer[i] > currentWiperParams.outBackDelay)
					// return back
					{
                        if (!sWiperToParkPlayed)
                        {
							sWiperToPark.play(sound_flags::exclusive);
							sWiperToParkPlayed = true;
                        }
						dWiperPos[i] = std::max(0.0, dWiperPos[i] - 1.f / currentWiperParams.WiperSpeed * dt1);
					}
					if (dWiperPos[i] >= 1.0) // we reached end
					{
						sWiperFromPark.stop();
						sWiperFromParkPlayed = false;
						wiperParkTimer[i] = 0.0;
						wiperDirection[i] = true; // switch direction
					}
					if (dWiperPos[i] <= 0.0)
					{
						// when in park position
                        sWiperToPark.stop();
						sWiperToParkPlayed = false;
						wiperOutTimer[i] = 0.0;
						wiperDirection[i] = false;
						workingSwitchPos[i] = MoverParameters->wiperSwitchPos; // update configuration
					}

                }
			}
		}
    }



    // mirrors
    auto const mirrorleft { dMirrorMoveL };
    auto const mirrorright { dMirrorMoveR };
    if( MoverParameters->Vel > MoverParameters->MirrorVelClose
		|| MoverParameters->CabActive == 0 && activation::mirrors
		|| MoverParameters->MirrorForbidden ) {
        // automatically fold mirrors when above velocity threshold
        if( dMirrorMoveL > 0.0 ) {
            dMirrorMoveL = std::max(
                0.0,
                dMirrorMoveL - 1.0 * dt1 );
        }
        if( dMirrorMoveR > 0.0 ) {
            dMirrorMoveR = std::max(
                0.0,
                dMirrorMoveR - 1.0 * dt1 );
        }
    }
    else {
        // unfold mirror on the side with open doors, if not moving too fast
        if( dMirrorMoveL < 1.0
         && true == MoverParameters->Doors.instances[side::left].open_permit ) {
            dMirrorMoveL = std::min(
                1.0,
                dMirrorMoveL + 1.0 * dt1 );
        }
        if( dMirrorMoveR < 1.0
         && true == MoverParameters->Doors.instances[side::right].open_permit ) {
            dMirrorMoveR = std::min(
                1.0,
                dMirrorMoveR + 1.0 * dt1 );
        }
    }
    play_mirror_sounds( side::left, m_mirrormotion[ side::left ].update( dMirrorMoveL - mirrorleft ) );
    play_mirror_sounds( side::right, m_mirrormotion[ side::right ].update( dMirrorMoveR - mirrorright ) );

    // compartment lights
    // if the vehicle has a controller, we base the light state on state of the controller otherwise we check the vehicle itself
    if( ctOwner != nullptr  ? SectionLightsActive != MoverParameters->CompartmentLights.is_active :
	    Mechanik != nullptr ? SectionLightsActive != MoverParameters->CompartmentLights.is_active :
	                          SectionLightsActive ) { // without controller switch the lights off
        toggle_lights();
    }

	if (MoverParameters->InactiveCabPantsCheck)
	{
		pants_up();
		MoverParameters->InactiveCabPantsCheck = false;
	}

    if( MoverParameters->LoadStatus ) {
        LoadUpdate(); // zmiana modelu ładunku
    }
    update_exchange( dt );

	return true; // Ra: chyba tak?
}

void TDynamicObject::pants_up()
{
	TDynamicObject *d = this;
	bool isAnyPantUp = false;
	while (d) {
		for (auto &item : d->MoverParameters->Pantographs)
		{
			isAnyPantUp |= item.is_active;
		}
		d = d->Next(Coupling::control); // pozostałe też
	}
	d = Prev(Coupling::control);
	while (d) {
		for (auto &item : d->MoverParameters->Pantographs)
		{
			isAnyPantUp |= item.is_active;
		}
		d = d->Prev(Coupling::control); // w drugą stronę też
	}
	if (isAnyPantUp)
	{
		d = this;
		while (d) {
			d->MoverParameters->OperatePantographValve(end::front, operation_t::enable, range_t::local);
			d->MoverParameters->OperatePantographValve(end::rear, operation_t::enable, range_t::local);
			d = d->Next(Coupling::control); // pozostałe też
		}
		d = Prev(Coupling::control);
		while (d) {
			d->MoverParameters->OperatePantographValve(end::front, operation_t::enable, range_t::local);
			d->MoverParameters->OperatePantographValve(end::rear, operation_t::enable, range_t::local);
			d = d->Prev(Coupling::control); // w drugą stronę też
		}
	}
}

// McZapkie-040402: liczenie pozycji uwzgledniajac wysokosc szyn itp.
// vector3 TDynamicObject::GetPosition()
//{//Ra: pozycja pojazdu jest liczona zaraz po przesunięciu
// return vPosition;
//};

void TDynamicObject::TurnOff()
{ // wyłączenie rysowania submodeli zmiennych dla
    // egemplarza pojazdu
    btnOn = false;
    btCoupler1.TurnOff();
    btCoupler2.TurnOff();
    btCPneumatic1.TurnOff();
    btCPneumatic1r.TurnOff();
    btCPneumatic2.TurnOff();
    btCPneumatic2r.TurnOff();
    btPneumatic1.TurnOff();
    btPneumatic1r.TurnOff();
    btPneumatic2.TurnOff();
    btPneumatic2r.TurnOff();
    btCCtrl1.Turn( false );
    btCCtrl2.Turn( false );
    btCPass1.Turn( false );
    btCPass2.Turn( false );
    m_endsignal12.Turn( false );
    m_endsignal13.Turn( false );
    m_endsignal22.Turn( false );
    m_endsignal23.Turn( false );
    m_endsignals1.Turn( false );
    m_endsignals2.Turn( false );
    m_endtab1.Turn( false );
    m_endtab2.Turn( false );
    m_headlamp11.TurnOff();
    m_headlamp12.TurnOff();
    m_headlamp13.TurnOff();
    m_headlamp21.TurnOff();
    m_headlamp22.TurnOff();
    m_headlamp23.TurnOff();
    m_highbeam12.TurnOff();
    m_highbeam13.TurnOff();
	m_highbeam22.TurnOff();
    m_highbeam23.TurnOff();
    m_headsignal12.TurnOff();
    m_headsignal13.TurnOff();
    m_headsignal22.TurnOff();
    m_headsignal23.TurnOff();
	btMechanik1.Turn( false );
	btMechanik2.Turn( false );
    btShutters1.Turn( false );
    btShutters2.Turn( false );
};

//---------------------------------------------------------------------------
void TDynamicObject::RadioStop()
{ // zatrzymanie pojazdu
    if( Mechanik ) {
        // o ile ktoś go prowadzi
		if( MoverParameters->SecuritySystem.radiostop_available()
         && MoverParameters->Radio
         && MoverParameters->RadioFuse ) {
            // jeśli pojazd ma RadioStop i jest on aktywny
            // HACK cast until math types unification
			glm::dvec3 pos = static_cast<glm::dvec3>(vPosition);
            Mechanik->PutCommand( "Emergency_brake", 1.0, 1.0, &pos, stopRadio );
            // add onscreen notification for human driver
            // TODO: do it selectively for the 'local' driver once the multiplayer is in
            if( false == Mechanik->AIControllFlag ) {
                ui::Transcripts.AddLine( "!! RADIO-STOP !!", 0.0, 10.0, false );
            }
        }
    }
};

//---------------------------------------------------------------------------
void TDynamicObject::Damage(char flag)
{
	if (flag & 1)  //różnicówka nie robi nic
	{
		MoverParameters->MainSwitch(false);
		MoverParameters->FuseOff();
	}
	else
	{
	}

	if (flag & 2)  //usterka sterowania
	{
		MoverParameters->StLinFlag = false;
		if (MoverParameters->InitialCtrlDelay<100000000)
			MoverParameters->InitialCtrlDelay += 100000001;
	}
	else
	{
		if (MoverParameters->InitialCtrlDelay>100000000)
			MoverParameters->InitialCtrlDelay -= 100000001;
	}

	if (flag & 4)  //blokada przetwornicy
	{
        if( MoverParameters->ConverterStart != start_t::disabled ) {
            MoverParameters->ConvOvldFlag = true;
        }
	}
	else
	{
	}

	if (flag & 8)  //blokada sprezarki
	{
		if (MoverParameters->MinCompressor>0)
			MoverParameters->MinCompressor -= 100000001;
		if (MoverParameters->MaxCompressor>0)
			MoverParameters->MaxCompressor -= 100000001;
	}
	else
	{
		if (MoverParameters->MinCompressor<0)
			MoverParameters->MinCompressor += 100000001;
		if (MoverParameters->MaxCompressor<0)
			MoverParameters->MaxCompressor += 100000001;
	}

	if (flag & 16)  //blokada wału
	{
		if (MoverParameters->CtrlDelay<100000000)
			MoverParameters->CtrlDelay += 100000001;
		if (MoverParameters->CtrlDownDelay<100000000)
			MoverParameters->CtrlDownDelay += 100000001;
	}
	else
	{
		if (MoverParameters->CtrlDelay>100000000)
			MoverParameters->CtrlDelay -= 100000001;
		if (MoverParameters->CtrlDownDelay>100000000)
			MoverParameters->CtrlDownDelay -= 100000001;
	}

	if (flag & 32)  //hamowanie nagŁe
	{
	}
	else
	{
	}

	MoverParameters->EngDmgFlag = flag;
};

void TDynamicObject::OverheadTrack(float o)
{ // ewentualne wymuszanie jazdy
    // bezprądowej z powodu informacji
    // w torze
    if (ctOwner) // jeśli ma obiekt nadzorujący
    { // trzeba zaktualizować mapę flag bitowych jazdy bezprądowej
        if (o < 0.0)
        { // normalna jazda po tym torze
            ctOwner->iOverheadZero &= ~iOverheadMask; // zerowanie bitu - może pobierać prąd
            ctOwner->iOverheadDown &= ~iOverheadMask; // zerowanie bitu - może podnieść pantograf
        }
        else if (o > 0.0)
        { // opuszczenie pantografów
            ctOwner->iOverheadZero |= iOverheadMask; // ustawienie bitu - ma jechać bez pobierania prądu
            ctOwner->iOverheadDown |= iOverheadMask; // ustawienie bitu - ma opuścić pantograf
        }
        else
        { // jazda bezprądowa z podniesionym pantografem
            ctOwner->iOverheadZero |=  iOverheadMask; // ustawienie bitu - ma jechać bez pobierania prądu
            ctOwner->iOverheadDown &= ~iOverheadMask; // zerowanie bitu - może podnieść pantograf
        }
    }
};
