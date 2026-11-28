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

// the consist the driver runs: its vehicles, their direction, their brakes, the cab taken

#include "stdafx.h"
#include "vehicle/Driver.h"
#include "physics/mover.h"
#include "utilities/Globals.h"
#include "utilities/translation.h"
#include "utilities/Logs.h"
#include "utilities/physicslog.h"
#include "vehicle/Train.h"
#include "vehicle/brakereleaser.h"
#include "vehicle/brakesetting.h"
#include "vehicle/brakingcurve.h"
#include "vehicle/fieldshunting.h"
#include "vehicle/handover.h"
#include "vehicle/signalpassing.h"
#include "vehicle/timetableservice.h"
#include "world/mtable.h"
#include "world/stationnames.h"
#include "vehicle/DynObj.h"
#include "world/Event.h"
#include "world/MemCell.h"
#include "world/RoadOrder.h"
#include "simulation/simulation.h"
#include "simulation/blackboxrecorder.h"
#include "simulation/simulationtime.h"
#include "simulation/simulationtest.h"
#include "world/Track.h"
#include "world/station.h"
#include "input/keyboardinput.h"
#include <algorithm>
#include <format>
#include <ranges>
#include "vehicle/ai/internal.h"


void TController::Activation()
{ // umieszczenie obsady w odpowiednim członie, wykonywane wyłącznie gdy steruje AI
    iDirection = iDirectionOrder; // kierunek (względem sprzęgów pojazdu z AI) właśnie został ustalony (zmieniony)
    if (iDirection)
    { // jeśli jest ustalony kierunek
        auto *initialvehicle { pVehicle };
/*
        auto const initiallocalbrakelevel { mvOccupied->LocalBrakePosA };
*/
        auto const initialspringbrakestate { mvOccupied->SpringBrake.Activate };
        // switch off tempomat
        SpeedCntrl( 0.0 );
        mvControlling->DecScndCtrl( 2 );
        // reset controls
        ZeroSpeed();
        ZeroDirection();
        mvOccupied->CabOccupied = mvOccupied->CabActive; // użytkownik moze zmienić CabOccupied wychodząc
        mvOccupied->CabDeactivisation(); // tak jest w Train.cpp
        if (pVehicle->MoverParameters->Couplers[iDirectionOrder < 0 ? end::rear : end::front].CouplingFlag.HasAll(Coupling::control)) {
            ZeroLocalBrake();
            if( initialspringbrakestate ) {
                mvOccupied->SpringBrakeActivate( false );
            }
            // odcięcie na zaworze maszynisty, FVel6 po drugiej stronie nie luzuje
            mvOccupied->BrakeLevelSet( mvOccupied->Handle->GetPos(bh_NP) ); // odcięcie na zaworze maszynisty
			BrakeLevelSet( mvOccupied->BrakeCtrlPos ); //ustawienie zmiennej GBH
        }
        // przejście AI na drugą stronę EN57, ET41 itp.
        // TODO: clean this up, there's lot of redundancy with TMoverParameters::ChangeCab() and TTrain::MoveToVehicle()
        {
            int movedirection { ( iDirection < 0 ? end::rear : end::front ) };
            auto *targetvehicle { pVehicle->FirstFind( movedirection, Coupling::control ) };
            if( pVehicle != targetvehicle ) {
                auto *targetvehicledriver { targetvehicle->Mechanik }; // zapamiętanie tego, co ewentualnie tam siedzi, żeby w razie dwóch zamienić miejscami
                // move to the new vehicle
                targetvehicle->Mechanik = this; // na razie bilokacja
                targetvehicle->MoverParameters->SetInternalCommand(VehicleCommand::none, 0, 0); // usunięcie ewentualnie zalegającej komendy (Change_direction?)
                if( targetvehicle->DirectionGet() != pVehicle->DirectionGet() ) {
                    // jeśli są przeciwne do siebie to będziemy jechać w drugą stronę względem zasiedzianego pojazdu
                    iDirection = -iDirection;
                }
                // move the other driver to our old vehicle
                pVehicle->Mechanik = targetvehicledriver; // wsadzamy tego, co ewentualnie był (podwójna trakcja)
                // NOTE: having caboccupied != 0 (by swapping in the driver from the target vehicle) may lead to dysfunctional brake
                // TODO: investigate and resolve if necessary
                pVehicle->MoverParameters->CabOccupied = targetvehicle->MoverParameters->CabOccupied;
                if( pVehicle->Mechanik ) { // not guaranteed, as target vehicle can be empty
                    pVehicle->Mechanik->BrakeLevelSet( pVehicle->MoverParameters->BrakeCtrlPos );
                }
                // finish driver swap
                pVehicle = targetvehicle;
            }
        }
        if (pVehicle != initialvehicle)
        { // jeśli zmieniony został pojazd prowadzony
            ControllingSet(); // utworzenie połączenia do sterowanego pojazdu (może się zmienić) - silnikowy dla EZT
            if( mvOccupied->BrakeCtrlPosNo > 0
             && ( BrakeSystem == TBrakeSystem::Pneumatic
               || BrakeSystem == TBrakeSystem::ElectroPneumatic ) ) {
                mvOccupied->LimPipePress = mvOccupied->PipePress;
                mvOccupied->ActFlowSpeed = 0;
            }
            BrakeLevelSet( mvOccupied->BrakeCtrlPos );
        }
        if( mvControlling->EngineType == TEngineType::DieselEngine && mvControlling->ShuntModeAllow ) {
            // dla 2Ls150 - przed ustawieniem kierunku - można zmienić tryb pracy
			mvControlling->CurrentSwitch(OrderCurrentGet().HasAny(DriverOrder::shunt | DriverOrder::looseShunt) || fMass > 224000.0); // do tego na wzniesieniu może nie dać rady na liniowym
		}
        // Ra: to przełączanie poniżej jest tu bez sensu
        mvOccupied->CabOccupied = iDirection; // aktywacja kabiny w prowadzonym pojeżdzie (silnikowy może być odwrotnie?)
        mvOccupied->CabActivisation(); // uruchomienie kabin w członach
        DirectionForward(true); // nawrotnik do przodu
        mvOccupied->SpringBrakeActivate( initialspringbrakestate );
/*
        // NOTE: this won't restore local brake if the vehicle has integrated local brake control
        // TBD, TODO: fix or let the ai activate the brake again as part of its standard logic?
        if (initiallocalbrakelevel > 0.0) // hamowanie tylko jeśli był wcześniej zahamowany (bo możliwe, że jedzie!)
            mvOccupied->LocalBrakePosA = initiallocalbrakelevel; // zahamuj jak wcześniej
*/
        if( pVehicle != initialvehicle ) {
            auto *train { simulation::Trains.find( initialvehicle->name() ) };
            if( train ) {
                train->MoveToVehicle( pVehicle );
            }
        }

        CheckVehicles(); // sprawdzenie składu, AI zapali światła
        TableClear(); // resetowanie tabelki skanowania torów
    }
};

void TController::AutoRewident()
{ // autorewident: nastawianie hamulców w składzie
    // 1. Zebranie informacji o składzie pociągu, od czoła do końca
    std::vector<brake_setting::vehicle> consist;
    for (auto const *d = pVehicles[0]; d != nullptr; d = d->Next()) {
        consist.emplace_back(brake_setting::describe(*d->MoverParameters, d == pVehicle));
    }
    // 2. Określenie typu pociągu i nastaw: by the brake instruction Cw-1 (eu07/maszyna#350), or as the releases before it did.
    // brakes are cut out (Cw-1 §19.9) only in a train (a bank engine's run included), not in a consist being shunted
    auto const train{brake_setting::runs_as_train(OrderCurrentGet())};
    auto const settings{Global.AIBrakeSettingCw1 ? brake_setting::choose(consist, train) : brake_setting::choose_legacy(consist)};
    // and only at a standstill, the train creeping to a stop included: a check on the run (the order changed mid-run, or
    // the train was just formed by coupling, which the AI does above that speed) leaves them to reinspect_brakes()
    auto const standstill{brake_setting::at_standstill(mvOccupied->Vel)};
    auto reinspect{false};
    // a light engine counts as a passenger train for the braking below
    auto const passengertrain{brake_setting::passenger(brake_setting::choose_train_setting(consist))};
    // 3. Nastawianie
    auto setting{settings.begin()};
    for (auto *d = pVehicles[0]; d != nullptr && setting != settings.end(); d = d->Next(), ++setting) {
        if (false == AIControllFlag && d == pVehicle) {
            // don't touch human-controlled vehicle, but others are free game
            continue;
        }
        // the setting, a brake cut out in a train on G (Cw-1 §19.9) or put back when the train no longer needs it cut out
        if (brake_setting::apply(*d->MoverParameters, *setting, standstill)) {
            // a brake to cut out, with the train moving: left for the next standstill
            reinspect = true;
        }
        // potentially release manual brake
        d->MoverParameters->DecManualBrakeLevel( ManualBrakePosNo );
        d->MoverParameters->SpringBrake.Activate = false;
    }
    BrakeReinspectionPending = reinspect;
    //ustawianie trybu pracy zadajnika hamulca, wystarczy raz po inicjalizacji AI
    if( true == AIControllFlag ) {
        // if a human is in charge leave the brake mode up to them, otherwise do as you like
        for( int i = 1; i <= 8; i *= 2 ) {
            if( ( mvOccupied->BrakeOpModes & i ) > 0 ) {
                mvOccupied->BrakeOpModeFlag = i;
            }
        }
    }

	// teraz zerujemy tabelkę opóźnienia hamowania
	for (int i = 0; i < BrakeAccTableSize; ++i)
	{
		fBrake_a0[i+1] = 0;
		fBrake_a1[i+1] = 0;
	}

    if( OrderCurrentGet() & ( DriverOrder::shunt | DriverOrder::looseShunt ) ) {
        // for uniform behaviour and compatibility with older scenarios set default acceleration table values for shunting
        fAccThreshold = is_emu() ? -0.55 : is_dmu() ? -0.45 : -0.2;
        // HACK: emu with induction motors need to start their braking a bit sooner than the ones with series motors
        if( is_emu()
         && mvControlling->EngineType == TEngineType::ElectricInductionMotor ) {
            fAccThreshold += 0.10;
        }
        fNominalAccThreshold = fAccThreshold;
    }

    BrakeSystem = consist_brake_system();
	// Change the EP fuse state only when AI is in charge, don't fiddle with controls when the user is controlling the vehicle
    if (AIControllFlag) {
	    mvOccupied->EpFuseSwitch(BrakeSystem == TBrakeSystem::ElectroPneumatic);
    }

    if( OrderCurrentGet() & ( DriverOrder::obeyTrain | DriverOrder::bank ) ) {
        // 4. Przeliczanie siły hamowania
        double const velstep = mvOccupied->Vmax * 0.5 / BrakeAccTableSize;
        auto *d = pVehicles[0]; // pojazd na czele składu
	    while (d) {
            // a vehicle with its brake cut out (by the inspector above or for a reason of its own) adds its mass only;
            // the inspector of the earlier releases counted its brake in
            auto const brakes{false == Global.AIBrakeSettingCw1 || false == brake_setting::is_cut_out(*d->MoverParameters)};
            for( int i = 0; i < BrakeAccTableSize && brakes; ++i ) {
                fBrake_a0[ i + 1 ] += d->MoverParameters->BrakeForceR( 0.25, velstep*( 1 + 2 * i ) );
                fBrake_a1[ i + 1 ] += d->MoverParameters->BrakeForceR( 1.00, velstep*( 1 + 2 * i ) );
		    }
		    d = d->Next(); // kolejny pojazd, podłączony od tyłu (licząc od czoła)
	    }
	    for (int i = 0; i < BrakeAccTableSize; ++i)
	    {
		    fBrake_a1[i+1] -= fBrake_a0[i+1];
		    fBrake_a0[i+1] /= fMass;
		    fBrake_a0[i + 1] += 0.001*velstep*(1 + 2 * i);
		    fBrake_a1[i+1] /= 12 * fMass;
	    }

        IsPassengerTrain = is_train() && false == is_emu() && false == is_dmu() && (mvOccupied->BrakeDelayFlag & bdelay_G) == 0;
        IsCargoTrain = is_train() && (mvOccupied->BrakeDelayFlag & bdelay_G) != 0;
        IsHeavyCargoTrain = true == IsCargoTrain && fBrake_a0[1] > 0.4 && iVehicles - ControlledEnginesCount > 0 && fMass / iVehicles > 50000;

        BrakingInitialLevel = IsHeavyCargoTrain ? 1.25 : IsCargoTrain ? 1.25 : 1.00;

        BrakingLevelIncrease = 0.25;

        if( is_emu() ) {
            auto ep_factor { ( BrakeSystem == TBrakeSystem::ElectroPneumatic ? 8 : 4 ) };
            if( mvControlling->EngineType == TEngineType::ElectricInductionMotor ) {
                // HACK: emu with induction motors need to start their braking a bit sooner than the ones with series motors
                fNominalAccThreshold = std::max( -0.60, -fBrake_a0[ BrakeAccTableSize ] - ep_factor * fBrake_a1[ BrakeAccTableSize ] );
            }
            else {
                fNominalAccThreshold = std::max( -0.75, -fBrake_a0[ BrakeAccTableSize ] - ep_factor * fBrake_a1[ BrakeAccTableSize ] );
            }
		    fBrakeReaction = 0.25;
	    }
        else if( is_dmu() ) {
            fNominalAccThreshold = std::max( -0.75, -fBrake_a0[ BrakeAccTableSize ] - 8 * fBrake_a1[ BrakeAccTableSize ] );
            fBrakeReaction = 0.25;
        }
        else if (passengertrain) {
            fNominalAccThreshold = -fBrake_a0[ BrakeAccTableSize ] - 4 * fBrake_a1[ BrakeAccTableSize ];
		    fBrakeReaction = 1.00 + fLength*0.004;
	    }
	    else {
            fNominalAccThreshold = -fBrake_a0[ BrakeAccTableSize ] - 1 * fBrake_a1[ BrakeAccTableSize ];
		    fBrakeReaction = 1.00 + fLength*0.005;
	    }
	    fAccThreshold = fNominalAccThreshold;
    }

}

// a brake the inspector had to cut out (Cw-1 §19.9) while the train was moving gets cut out once the train stands.
// a train formed by coupling is looked at as the buffers meet, above the speed the inspector takes for a standstill,
// and with no stop ahead nothing would look at it standing again (eu07/maszyna#350)
void TController::reinspect_brakes()
{
    if (false == BrakeReinspectionPending) {
        return;
    }
    if (false == brake_setting::runs_as_train(OrderCurrentGet())) {
        // no longer a train: nothing is cut out in a consist being shunted
        BrakeReinspectionPending = false;
        return;
    }
    if (brake_setting::at_standstill(mvOccupied->Vel)) {
        // standing, the inspector leaves nothing for later: this clears the flag
        AutoRewident();
    }
}

int TController::CheckDirection() {

    int d = mvOccupied->DirAbsolute; // który sprzęg jest z przodu
    if( !d ) {
        // jeśli nie ma ustalonego kierunku to jedziemy wg aktualnej kabiny
        d = mvOccupied->CabActive;
    }
    return d;
}

bool TController::CheckVehicles(DriverOrders user)
{ // sprawdzenie stanu posiadanych pojazdów w składzie i zapalenie świateł
    TDynamicObject *p; // roboczy wskaźnik na pojazd
    iVehicles = 0; // ilość pojazdów w składzie
    auto d = CheckDirection();
    d = d >= 0 ? 0 : 1; // kierunek szukania czoła (numer sprzęgu)
    p = pVehicle->FirstFind(d); // pojazd na czele składu
    if (!p)
        p = pVehicle;
    auto *const frontvehicle { p }; // stays the consist head for the rest of this check
    pVehicles[end::front] = frontvehicle;
    // liczenie pojazdów w składzie i ustalenie parametrów
    auto dir = d = 1 - d; // a dalej będziemy zliczać od czoła do tyłu
    fLength = 0.0; // długość składu do badania wyjechania za ograniczenie
    fMass = 0.0; // całkowita masa do liczenia stycznej składowej grawitacji
    fVelMax = -1; // ustalenie prędkości dla składu
    bool main = true; // czy jest głównym sterującym
    drivingFlags.oerlikons = true; // zakładamy, że są same Oerlikony
    // Ra 2014-09: ustawić moveMultiControl, jeśli wszystkie są w ukrotnieniu (i skrajne mają kabinę?)
    while (p)
    { // sprawdzanie, czy jest głównym sterującym, żeby nie było konfliktu
        if( p->Mechanik // jeśli ma obsadę
		    && p->Mechanik != this && p->Mechanik->drivingFlags.primary ) { // ale chodzi o inny pojazd, niż aktualnie sprawdzający
			// a tamten ma priorytet
			// TODO: take into account drivers' operating modes, one or more of them might be on banking duty
			if (drivingFlags.primary && mvOccupied->DirAbsolute && mvOccupied->BrakeCtrlPos >= -1)
			{
				// jeśli rządzi i ma kierunek
				p->Mechanik->primary(false); // dezaktywuje tamtego
				p->Mechanik->ZeroLocalBrake();
				p->MoverParameters->BrakeLevelSet(p->MoverParameters->Handle->GetPos(bh_NP)); // odcięcie na zaworze maszynisty
				p->Mechanik->BrakeLevelSet(p->MoverParameters->BrakeCtrlPos); // ustawienie zmiennej GBH
			}
			else
			{
				main = false; // nici z rządzenia
			}
		}
        ++iVehicles; // jest jeden pojazd więcej
        pVehicles[end::rear] = p; // zapamiętanie ostatniego
        fLength += p->MoverParameters->Dim.L; // dodanie długości pojazdu
        fMass += p->MoverParameters->TotalMass; // dodanie masy łącznie z ładunkiem
        fVelMax = min_speed( fVelMax, p->MoverParameters->Vmax ); // ustalenie maksymalnej prędkości dla składu
        // reset oerlikon brakes consist flag as different type was detected
        if( p->MoverParameters->BrakeSubsystem != TBrakeSubSystem::ss_ESt
         && p->MoverParameters->BrakeSubsystem != TBrakeSubSystem::ss_LSt ) {
            drivingFlags.oerlikons = false;
        }
        p = p->Neighbour(dir); // pojazd podłączony od wskazanej strony
    }
    if( main ) {
        drivingFlags.primary = true; // nie znaleziono innego, można się porządzić
    }

    ControllingSet(); // ustalenie członu do sterowania (może być inny niż zasiedziany)

    if (drivingFlags.primary)
    { // jeśli jest aktywnie prowadzącym pojazd, może zrobić własny porządek
        auto pantmask = 1;
        p = frontvehicle;
        // establish ownership and vehicle order
        while (p)
        {
            if (p->MoverParameters->EnginePowerSource.SourceType == TPowerSource::CurrentCollector)
            { // jeśli pojazd posiada pantograf, to przydzielamy mu maskę, którą będzie informował o jeździe bezprądowej
                p->iOverheadMask = pantmask;
                pantmask = pantmask << 1; // przesunięcie bitów, max. 32 pojazdy z pantografami w składzie
            }

            d = p->DirectionSet(d ? 1 : -1); // zwraca położenie następnego (1=zgodny,0=odwrócony - względem czoła składu)
            p->ctOwner = this; // dominator oznacza swoje terytorium
            p = p->Next(); // pojazd podłączony od tyłu (licząc od czoła)
        }
        // with the order established the virtual train manager can do their work
        p = frontvehicle;
        ControlledEnginesCount = p->MoverParameters->Power > 1.0 ? 1 : 0;
        auto hasheaters { false };
        while (p)
        {
            if( p != pVehicle ) {
                if( false == p->is_connected( pVehicle, Coupling::control ) ) {
                    // NOTE: don't set battery in controllable vehicles, let the user/ai do it explicitly
                    // HACK: wagony muszą mieć baterię załączoną do otwarcia drzwi...
                    p->MoverParameters->BatterySwitch( true );
                    // enable heating and converter in carriages with can be heated
                    if( p->MoverParameters->HeatingPower > 0 ) {
                        p->MoverParameters->HeatingAllow = true;
                        p->MoverParameters->ConverterSwitch( true, range_t::local );
                        hasheaters = true;
                    }
                }
                else {
                    if( p->MoverParameters->Power > 1.0 ) {
                        ++ControlledEnginesCount;
                    }
                }
            }

            if( p->asDestination == "none" ) {
                p->DestinationSet( TrainParams.Relation2, TrainParams.TrainName ); // relacja docelowa, jeśli nie było
            }
            if( AIControllFlag ) { // jeśli prowadzi komputer
                p->RaLightsSet( 0, 0 ); // gasimy światła
            }
            p = p->Next(); // pojazd podłączony od tyłu (licząc od czoła)
        }

        if( user == DriverOrder::connect && true == main ) {
            // HACK: with additional vehicles in the consist ensure all linked vehicles are set to move in the same direction
            if( pVehicle->Prev(Coupling::control) != nullptr
             || pVehicle->Next(Coupling::control) != nullptr ) {
                sync_consist_reversers();
            }
            // potentially sync compartment lighting state for the newly connected vehicles
            if( mvOccupied->CompartmentLights.start_type == start_t::manual ) {
                mvOccupied->CompartmentLightsSwitchOff( mvOccupied->CompartmentLights.is_disabled );
                mvOccupied->CompartmentLightsSwitch( mvOccupied->CompartmentLights.is_enabled );
            }
        }

        // kasowanie pamieci hamowania kontrolnego
        if (OrderCurrentGet() & (DriverOrder::shunt | DriverOrder::looseShunt | DriverOrder::disconnect | DriverOrder::connect | DriverOrder::changeDirection)) {
			DynamicBrakeTest = 0;
            DBT_VelocityBrake
            = DBT_VelocityRelease
            = DBT_VelocityFinish
            = DBT_BrakingTime
            = DBT_ReleasingTime
            = DBT_MidPointAcc
            = 0;
		}

        if( brake_setting::inspects_train( OrderCurrentGet(), user == DriverOrder::connect ) ) {
            // nastawianie hamulca do jazdy pociągowej
            AutoRewident();
/*
            if( ( true == drivingFlags.connect )
                && ( true == OrderCurrentGet().HasAll(DriverOrder::connect) ) ) {
                iCoupler = {}; // dalsza jazda manewrowa już bez łączenia
                drivingFlags.connect = false; // zdjęcie flagi doczepiania
                SetVelocity( 0, 0, stopJoin ); // wyłączyć przyspieszanie
                JumpToNextOrder(); // wykonanie następnej komendy
            }
*/
        }

        // detect push-pull train configurations and mark them accordingly
        if( frontvehicle->is_connected( pVehicles[ end::rear ], Coupling::control ) ) {
            // zmiana czoła przez zmianę kabiny
            drivingFlags.pushPull = true;
        }
        else {
            // zmiana czoła przez manewry
            drivingFlags.pushPull = false;
        }

        // HACK: ensure vehicle lights are active from the beginning, if it had pre-activated battery
        if( mvOccupied->LightsPosNo > 0 ) {
            pVehicle->SetLights();
        }
        if( iEngineActive ) {
            // potentially adjust light state
            control_lights();
            // custom ai action for disconnect mode: switch off lights on disconnected vehicle(s)
            if( is_train() && true == OrderCurrentGet().HasAll(DriverOrder::disconnect) && AIControllFlag ) {
				// światła manewrowe (Tb1) tylko z przodu, aby nie pozostawić odczepionego ze światłem
				if (mvOccupied->DirActive >= 0)
				{ // jak ma kierunek do przodu
					pVehicles[end::rear]->RaLightsSet(-1, 0);
				}
				else
				{ // jak dociska
					frontvehicle->RaLightsSet(0, -1);
				}
			}
            // enable door locks
            cue_action( driver_hint::consistdoorlockson );
            // potentially enable train heating
            {
                auto const ispassengertrain { IsPassengerTrain && iVehicles - ControlledEnginesCount > 0 };
                // TODO: replace connection test with connection check between last engine and first car, specifically
                auto const isheatingcouplingactive { (
                    ControlledEnginesCount == 1 ?
                        frontvehicle->is_connected( pVehicles[ end::rear ], Coupling::heating ) :
                        true ) };
                auto const isheatingneeded {
                    (is_emu() || is_dmu() ? true :
                    false == OrderCurrentGet().HasAny(DriverOrder::obeyTrain | DriverOrder::bank) ? false :
                    false == isheatingcouplingactive ? false :
                    ispassengertrain ? hasheaters :
                    false) };
                if( mvControlling->HeatingAllow != isheatingneeded ) {
                    cue_action(
                        isheatingneeded ?
                            driver_hint::consistheatingon :
                            driver_hint::consistheatingoff );
                }
            }
        }

        if( user == DriverOrder::connect
         || user == DriverOrder::disconnect ) {
            // HACK: force route table update on consist change, new consist length means distances to points of interest are now wrong
            iTableDirection = 0;
        }
    } // blok wykonywany, gdy aktywnie prowadzi
    return true;
}

void TController::Lights(int head, int rear)
{ // zapalenie świateł w skłądzie
    pVehicles[ end::front ]->RaLightsSet(head, -1); // zapalenie przednich w pierwszym
    pVehicles[ end::rear ]->RaLightsSet(-1, rear); // zapalenie końcówek w ostatnim
}

void TController::DirectionInitial()
{ // ustawienie kierunku po wczytaniu trainset (może jechać na wstecznym
    mvOccupied->CabActivisationAuto(); // załączenie rozrządu (wirtualne kabiny)
    if (mvOccupied->Vel > EU07_AI_NOMOVEMENT)
    { // jeśli na starcie jedzie
        iDirection = iDirectionOrder =
            mvOccupied->V > 0 ? 1 : -1; // początkowa prędkość wymusza kierunek jazdy
        DirectionForward(mvOccupied->V * mvOccupied->CabActive >= 0.0); // a dalej ustawienie nawrotnika
    }
    CheckVehicles(); // sprawdzenie świateł oraz skrajnych pojazdów do skanowania
};

void TController::DirectionChange() {

    auto const initialstate { iDirection };
    iDirection = CheckDirection();
    if( iDirection != initialstate ) {
        CheckVehicles( DriverOrder::changeDirection );
    }
}

int TController::OrderDirectionChange(int newdir, TMoverParameters *Vehicle)
{ // zmiana kierunku jazdy, niezależnie od kabiny
    int testd = newdir;
    if (Vehicle->Vel < 0.5)
    { // jeśli prawie stoi, można zmienić kierunek, musi być wykonane dwukrotnie, bo za pierwszym razem daje na zero
        switch (newdir * Vehicle->CabActive)
        { // DirectionBackward() i DirectionForward() to zmiany względem kabiny
        case -1: // if (!Vehicle->DirectionBackward()) testd=0; break;
            DirectionForward(false);
            break;
        case 1: // if (!Vehicle->DirectionForward()) testd=0; break;
            DirectionForward(true);
            break;
        }
        if (testd == 0)
            VelforDriver = -1; // kierunek został zmieniony na żądany, można jechać
    }
    else // jeśli jedzie
        VelforDriver = 0; // ma się zatrzymać w celu zmiany kierunku
    if (Vehicle->DirActive == 0 && VelforDriver < Vehicle->Vel) // Ra: to jest chyba bez sensu
        IncBrake(); // niech hamuje
    if (Vehicle->DirActive == testd * Vehicle->CabActive)
        VelforDriver = -1; // można jechać, bo kierunek jest zgodny z żądanym
    if (is_emu() && Vehicle->DirActive > 0)
		// if () //tylko jeśli jazda pociągowa (tego nie wiemy w momencie odpalania silnika)
		Vehicle->DirectionForward();
	// Ra: z przekazaniem do silnikowego
    return (int)VelforDriver; // zwraca prędkość mechanika
}

void TController::DirectionForward(bool forward)
{ // ustawienie jazdy do przodu dla true i do tyłu dla false (zależy od kabiny)
    ZeroSpeed( true ); // TODO: check if force switch is needed anymore here
    // HACK: make sure the master controller isn't set in position which prevents direction change
    mvControlling->MainCtrlPos = std::min( mvControlling->MainCtrlPos, mvControlling->MainCtrlMaxDirChangePos );

    if( forward ) {
        // do przodu w obecnej kabinie
        while( mvOccupied->DirActive <= 0
            && mvOccupied->DirectionForward() ) {
            // force scan table update
            iTableDirection = 0;
        }
    }
    else {
        // do tyłu w obecnej kabinie
        while( mvOccupied->DirActive >= 0
            && mvOccupied->DirectionBackward() ) {
            // force scan table update
            iTableDirection = 0;
        }
    }
    if( mvOccupied->TrainType == train_type::sn61 ) {
        // specjalnie dla SN61 żeby nie zgasł
        while( mvControlling->RList[mvControlling->MainCtrlPos].Mn == 0
            && mvControlling->IncMainCtrl(1) ) {
            ; // all work is done in the header
        }
    }
}

void TController::ZeroDirection() {

    while( mvOccupied->DirActive > 0 && mvOccupied->DirectionBackward() ) { ; }
    while( mvOccupied->DirActive < 0 && mvOccupied->DirectionForward() ) { ; }
}

void TController::sync_consist_reversers() {

    auto const currentdirection { mvOccupied->DirActive };
    auto const fastforward { is_emu() && mvOccupied->EngineType != TEngineType::ElectricInductionMotor
     && mvOccupied->Imin == mvOccupied->IminHi };

    // move reverser all way in the opposite direction...
    for( auto idx = 0; idx < 3; ++idx ) {
        if( currentdirection >= 0 ) {
            mvOccupied->DirectionBackward();
        }
        else {
            mvOccupied->DirectionForward();

        }
    }
    // ...then restore original setting
    while( mvOccupied->DirActive != currentdirection ) {
        if( false == (
            currentdirection >= 0 ?
                mvOccupied->DirectionForward() :
                mvOccupied->DirectionBackward() ) ) {
            // sanity check for potential endless loop
            break;
        }
    }
    // potentially restore 'fast forward' setting
    if( currentdirection > 0 && true == fastforward ) {
        mvOccupied->DirectionForward();
    }
}

void TController::MoveTo(TDynamicObject *to)
{ // przesunięcie AI do innego pojazdu (przy zmianie kabiny)
    if( to->Mechanik != nullptr
     && to->Mechanik != this ) {
        // ai controller thunderdome, there can be only one
        if( to->Mechanik->AIControllFlag ) {
            if( to->Mechanik->primary() ) {
                // take over boss duties
                primary( true );
            }
            SafeDelete( to->Mechanik );
        }
        else {
            // can't quite delete a human
            if( AIControllFlag ) {
                delete this;
            }
            return;
        }
    }
    pVehicle->Mechanik = nullptr;
    pVehicle = to;
    ControllingSet(); // utworzenie połączenia do sterowanego pojazdu
    pVehicle->Mechanik = this;

};

void TController::ControllingSet()
{ // znajduje człon do sterowania w EZT będzie to silnikowy
    // problematyczne jest sterowanie z członu biernego, dlatego damy AI silnikowy
    // dzięki temu będzie wirtualna kabina w silnikowym, działająca w rozrządczym
    // w plikach FIZ zostały zgubione ujemne maski sprzęgów, stąd problemy z EZT
    if( pVehicle == nullptr ) {
        // a driver of no vehicle has nothing to control
        return;
    }
    mvOccupied = pVehicle->MoverParameters; // domyślny skrót do obiektu parametrów
	BrakeSystem = mvOccupied->BrakeSystem; // domyślny sposób hamowania
    mvControlling = pVehicle->FindPowered()->MoverParameters; // poszukiwanie członu sterowanego
    {
        auto *lookup { pVehicle->FindPantographCarrier() };
        mvPantographUnit = lookup != nullptr ? lookup->MoverParameters : mvControlling;
    }
    BrakeSystem = consist_brake_system();
};

void
TController::determine_consist_state() {
    // ABu-160305 testowanie gotowości do jazdy
    // Ra: przeniesione z DynObj, skład użytkownika też jest testowany, żeby mu przekazać, że ma odhamować

	if (mvOccupied->CabActive == 0 && mvOccupied->Power24vIsAvailable)
	{
		cue_action( driver_hint::cabactivation );
	}
	else if (!mvOccupied->AutomaticCabActivation
			 && ( mvOccupied->CabActive == -mvOccupied->CabOccupied || !mvOccupied->CabMaster || !mvOccupied->Power24vIsAvailable ) )
	{
		cue_action( driver_hint::cabdeactivation );
	}

	int index = double(BrakeAccTableSize) * (mvOccupied->Vel / mvOccupied->Vmax);
	index = std::min(BrakeAccTableSize, std::max(1, index));
	fBrake_a0[0] = fBrake_a0[index];
	fBrake_a1[0] = fBrake_a1[index];

	if (is_emu() || is_dmu()) {
		auto Coeff = std::clamp( mvOccupied->Vel*0.015 , 0.5 , 1.0);
		fAccThreshold = fNominalAccThreshold * Coeff - fBrake_a0[BrakeAccTableSize] * (1.0 - Coeff);
	}

    Ready = true; // wstępnie gotowy
    fReady = 0.0; // założenie, że odhamowany
    IsConsistBraked = mvOccupied->BrakeSystem == TBrakeSystem::ElectroPneumatic && false == ForcePNBrake ? mvOccupied->BrakePress > 2.0 :
	                                                                                                       mvOccupied->PipePress < std::max(3.9, mvOccupied->BrakePressureActual.PipePressureVal) + 0.1;
    fAccGravity = 0.0; // przyspieszenie wynikające z pochylenia
    IsAnyCouplerStretched = false;
    IsAnyDoorOnlyOpen[ side::right ] = IsAnyDoorOnlyOpen[ side::left ] = false;
	IsAnyDoorOpen[ side::right ] = IsAnyDoorOpen[ side::left ] = false;
    IsAnyDoorPermitActive[ side::right ] = IsAnyDoorPermitActive[ side::left ] = false;
    auto doorclosure { door_closure_t::none };
    ConsistShade = 0.0;
    auto *p { pVehicles[ end::front ] }; // pojazd na czole składu
    double dy; // składowa styczna grawitacji, w przedziale <0,1>
    // starting uphill the power is applied against the train brake on purpose, until the power holds the train and
    // then while the brake releases
    auto const heldforstart { bUphillStart };
    auto released { true }; // the brakes are off, the hold of an uphill start not counted
    auto mostapplied { 0.0 }; // the most applied brake, the hold of an uphill start not counted
    auto momentum { 0.0 };
    auto chargelevel { std::numeric_limits<double>::max() }; // the lowest pipe or auxiliary reservoir pressure of a working brake
    while (p)
    { // sprawdzenie odhamowania wszystkich połączonych pojazdów
        auto *vehicle { p->MoverParameters };
        auto const bp { std::max( 0.0, vehicle->BrakePress - ( vehicle->SpeedCtrlUnit.Parking ? vehicle->MaxBrakePress[ 0 ] * vehicle->StopBrakeDecc : 0.0 ) ) };
        momentum += vehicle->TotalMass * vehicle->V * ( p->DirectionGet() == iDirection ? 1 : -1 );
        mostapplied = std::max( bp, mostapplied );
        if( ( vehicle->BrakeSystem != TBrakeSystem::Individual )
         && ( vehicle->MaxBrakePress[ 3 ] > 0.0 )
         && ( false == TestFlag( vehicle->Hamulec->GetBrakeStatus(), b_dmg ) ) ) { // a brake cut out doesn't count
            chargelevel = std::min( { chargelevel, vehicle->PipePress, vehicle->Hamulec->GetBRP() } );
        }
        auto const wasready { Ready };
        if( released || Ready ) {
            // bo jak coś nie odhamowane, to dalej nie ma co sprawdzać
            auto const isbraking { TestFlagAny( vehicle->Hamulec->GetBrakeStatus(), b_hld | b_on ) };
            if( vehicle->Vel < 1.0 ?
                    isbraking || bp > 0.4 : // ensure the brakes are sufficiently released when starting to move
                    // once in motion a brake down to 0.4 bar counts as released (UIC), even if its distributor
                    // is still in the lap position while the brake pipe recharges
                    ( isbraking && bp > 0.4 ) || vehicle->Fb > 10000.0 ) { // 10 kN
                released = false;
                // ready all the same with the power going on against the brake on purpose: at a standstill, and once
                // the power holds the train, while the brake releases; otherwise in motion a braking force left
                // anywhere in the train (a hand brake) stops it as before
                Ready = Ready && heldforstart && ( bUphillRelease || ( vehicle->Vel < 1.0 ) );
            }
        }
        if( wasready ) {
            // Ra: odluźnianie przeładowanych lokomotyw, ciągniętych na zimno - prowizorka...
            if( bp >= 0.4 && (AIControllFlag || Global.AITrainman && mvOccupied->Vel < EU07_AI_NOMOVEMENT && !is_emu() && !is_dmu()) && BrakeCtrlPosition == gbh_RP // jest pozycja jazdy
			    && false == TestFlag(vehicle->Hamulec->GetBrakeStatus(), b_dmg) // brake isn't broken
			    && vehicle->PipePress - mvOccupied->Handle->GetRP() > -0.1 // jeśli ciśnienie jak dla jazdy
			    && vehicle->Hamulec->GetCRP() > vehicle->PipePress + 0.12 ) { // wg UIC określone sztywno na 0.04
				// za dużo w zbiorniku
				// indywidualne luzowanko
				vehicle->BrakeReleaser(1);
			}
			if (bp < 0.1 && (AIControllFlag || Global.AITrainman) && false == TestFlag(vehicle->Hamulec->GetBrakeStatus(), b_dmg) // brake isn't broken
			    && vehicle->Hamulec->GetCRP() < vehicle->PipePress - 0.1) {
				// już nie jest za dużo w zbiorniku
				// koniec indywidualnego luzowanka
				vehicle->BrakeReleaser(0);
			}
        }
        // the brake held for the start counts again once it is being released
        fReady = std::max( ( heldforstart && ( false == bUphillRelease ) ) ? 0.0 : bp, fReady ); // szukanie najbardziej zahamowanego
        if( ( dy = p->VectorFront().y ) != 0.0 ) {
            // istotne tylko dla pojazdów na pochyleniu
            // ciężar razy składowa styczna grawitacji
            fAccGravity -= vehicle->TotalMassxg * dy * ( p->DirectionGet() == iDirection ? 1 : -1 );
        }
        // check coupler state
        IsAnyCouplerStretched =
            IsAnyCouplerStretched
            || vehicle->Couplers[end::front].stretch_duration > 0.0
            || vehicle->Couplers[end::rear].stretch_duration > 0.0;
        // check door state
        {
			auto const switchsides{ p->DirectionGet() != (iDirection == 0 ? mvOccupied->CabOccupied : iDirection) };
            auto const &rightdoor { vehicle->Doors.instances[ ( switchsides ? side::left : side::right ) ] };
            auto const &leftdoor { vehicle->Doors.instances[ ( switchsides ? side::right : side::left ) ] };
            if( vehicle->Doors.close_control != control_t::autonomous ) {
                IsAnyDoorOpen[ side::right ] |= false == rightdoor.is_closed;
                IsAnyDoorOpen[ side::left  ] |= false == leftdoor.is_closed;
            }
			if (vehicle->Doors.close_control != control_t::autonomous) {
				IsAnyDoorOnlyOpen[ side::right ] |= false == rightdoor.is_door_closed;
				IsAnyDoorOnlyOpen[ side::left  ] |= false == leftdoor.is_door_closed;
			}
            if( vehicle->Doors.permit_needed ) {
                IsAnyDoorPermitActive[ side::right ] |= rightdoor.open_permit;
                IsAnyDoorPermitActive[ side::left  ] |= leftdoor.open_permit;
            }
            doorclosure = std::max( doorclosure, vehicle->door_closure() );
        }
        // measure lighting level
        // TBD: apply weight (multiplier) to partially lit vehicles?
        ConsistShade += p->fShade > 0.0 ? p->fShade : 1.0;
        p = p->Next(); // pojazd podłączony z tyłu (patrząc od czoła)
    }
    DoorClosure = doorclosure;
    // calculate average amount of received sunlight
    ConsistShade /= iVehicles;

    // test state of main devices in all vehicles under control
    IsAnyConverterOverloadRelayOpen = false;
    IsAnyMotorOverloadRelayOpen = false;
    IsAnyGroundRelayOpen = false;
    IsAnyLineBreakerOpen = false;
    IsAnyCompressorPresent = false;
    IsAnyCompressorEnabled = false;
    IsAnyCompressorExplicitlyEnabled = false;
    IsAnyConverterPresent = false;
    IsAnyConverterEnabled = false;
    IsAnyConverterExplicitlyEnabled = false;
    // whether the line contactors of a vehicle wait for its own converter (LineContactorsInterlock), off with its local switch
    auto islocalconverterswitchoff { false };

    pVehicle->for_each(
        Coupling::control,
        [this, &islocalconverterswitchoff]( TDynamicObject * Vehicle ) {
            auto const *vehicle { Vehicle->MoverParameters };
            islocalconverterswitchoff |= vehicle->LineContactorsHeldByLocalConverterSwitch();
            IsAnyConverterOverloadRelayOpen |= vehicle->ConvOvldFlag;
            IsAnyMotorOverloadRelayOpen |= vehicle->FuseFlag;
            IsAnyGroundRelayOpen |= !vehicle->GroundRelay;
            IsAnyCompressorPresent |= vehicle->CompressorSpeed > 0.0;
            IsAnyCompressorEnabled |= vehicle->CompressorSpeed > 0.0 ? (vehicle->CompressorAllow || vehicle->CompressorStart == start_t::automatic) && vehicle->CompressorAllowLocal : false;
            IsAnyCompressorExplicitlyEnabled |= vehicle->CompressorSpeed > 0.0 ? vehicle->CompressorAllow && vehicle->CompressorAllowLocal : false;
            IsAnyConverterPresent |= vehicle->ConverterStart != start_t::disabled;
            IsAnyConverterEnabled |= ( vehicle->ConverterAllow || vehicle->ConverterStart == start_t::automatic ) && vehicle->ConverterAllowLocal;
            IsAnyConverterExplicitlyEnabled |= vehicle->ConverterAllow && vehicle->ConverterAllowLocal;
            if( vehicle->Power > 0.01 ) {
                IsAnyLineBreakerOpen |= !vehicle->Mains; } } );

    // siłę generują pojazdy na pochyleniu ale działa ona całość składu, więc a=F/m
    fAccGravity *= iDirection >= 0 ? 1 : -1;
    fAccGravity /= fMass;
    {
        auto absaccs { fAccGravity }; // Ra 2014-03: jesli skład stoi, to działa na niego składowa styczna grawitacji
        if( mvOccupied->Vel > EU07_AI_NOMOVEMENT ) {
            absaccs = 0;
            auto *d = pVehicles[ end::front ]; // pojazd na czele składu
            while( d ) {
                absaccs += d->MoverParameters->TotalMass * d->MoverParameters->AccS * ( d->DirectionGet() == iDirection ? 1 : -1 );
                d = d->Next(); // kolejny pojazd, podłączony od tyłu (licząc od czoła)
            }
            absaccs *= iDirection >= 0 ? 1 : -1;
            absaccs /= fMass;
        }
        AbsAccS = absaccs;
    }
#if LOGVELOCITY
    WriteLog( "Vel=" + to_string( DirectionalVel() ) + ", AbsAccS=" + to_string( AbsAccS ) + ", AccGrav=" + to_string( fAccGravity ) );
#endif

    if( !Ready // v367: jeśli wg powyższych warunków skład nie jest odhamowany
     && fAccGravity < -0.05 // jeśli ma pod górę na tyle, by się stoczyć
     && fReady < 0.8 ) { // delikatniejszy warunek, obejmuje wszystkie wagony
        Ready = true; //żeby uznać za odhamowany
    }
    // the same test without the hold of an uphill start
    IsConsistReleased = released || ( fAccGravity < -0.05 && mostapplied < 0.8 );
    // the frame of the vehicle the AI drives from, as for fAccGravity and AbsAccS
    fConsistVelocity = 3.6 * momentum / std::max( 1.0, fMass ) * ( iDirection >= 0 ? 1 : -1 );
    // standing, the train starts only with its brakes charged: released alone, a train prepared from cold has no air to
    // brake with while its auxiliary reservoirs fill. with the train brake applied (held for an uphill start, a light
    // application while charging) they charge to what the head of the pipe has.
    // only the start the charge is in question for: the first after the train was prepared standing, or one with the
    // pipe near empty (after an emergency application, brake_charge::empty_level). after an ordinary stop the
    // reservoirs refill as the brake releases, and waiting for the last tenths of a bar there only delays the train and
    // those behind it.
    // and only the pneumatic brake: multiple units with the electro-pneumatic one start as before
    if( mvOccupied->Vel > 1.0 ) {
        // the start is made. a shunting engine coupling wagons moves at a walking pace: a higher speed would leave its
        // every start the first, waiting for each cut of wagons placed cold to charge
        bBrakeChargeStart = false;
        bBrakeChargeHold = false;
        bHasRun = true;
    }
    auto const chargeable {
        ( BrakeSystem == TBrakeSystem::Pneumatic )
     && ( mvOccupied->Vel < EU07_AI_NOMOVEMENT )
     && ( chargelevel < std::numeric_limits<double>::max() ) };
    if( chargeable && ( chargelevel < brake_charge::empty_level ) ) {
        bBrakeChargeStart = true;
    }
    if( chargeable && bBrakeChargeStart ) {
        if( ( VelDesired <= 0.0 ) || ( AccDesired <= EU07_AI_NOACCELERATION ) ) {
            // to stand: the next start is checked anew
            BrakeCharge.going = false;
        }
        auto const trainbrakeapplied {
            AIControllFlag ?
                BrakeCtrlPosition > 0.0 :
                // a human driver's handle, which BrakeCtrlPosition doesn't follow
                mvOccupied->fBrakeCtrlPos > mvOccupied->Handle->GetPos( bh_RP ) + 0.2 };
        auto const reference {
            brake_charge::reference( mvOccupied->PipePress, std::min( mvOccupied->Handle->GetRP(), 5.0 ), trainbrakeapplied ) };
        // air to charge with: the engine prepared, the main reservoir above what the pipe charges to, and the driver's
        // brake valve feeding the pipe at the head of the train. a train still being prepared from cold can't charge,
        // nor one whose pipe is locked (unlocked by control_releaser()): it waits without that counting as a stall, held
        // by its brakes as it can't release them either
        auto const supplied {
            iEngineActive
         && ( std::max( mvControlling->Compressor, mvOccupied->Compressor ) > reference )
         && ( mvOccupied->PipePress >= reference - brake_charge::margin ) };
        auto const wasstalled { BrakeCharge.stalled };
        IsConsistCharged = brake_charge::charged( BrakeCharge, chargelevel, reference, ElapsedTime, supplied );
        if( BrakeCharge.stalled && ( false == wasstalled ) ) {
            WriteLog(
                "AI: " + mvOccupied->Name + " stopped charging its brakes at " + to_string( chargelevel, 2 )
                + " bar of " + to_string( reference, 2 ) + ", starting all the same" );
        }
        if( false == IsConsistCharged ) {
            // a human driver is told to wait as the ai does, the locomotive brake applied
            cue_action( driver_hint::waitbrakecharge );
        }
    }
    else {
        IsConsistCharged = true;
        BrakeCharge = {};
    }
    if( AIControllFlag
     && ( false == IsConsistCharged )
     && ( false == heldforstart ) ) {
        Ready = false;
    }
    auto const departblocked {
        AIControllFlag
     && mvOccupied->Vel < EU07_AI_NOMOVEMENT
     && VelDesired > 0.0
     && ( false == Ready || fReady > 0.4 ) };
    if( false == departblocked ) {
        DepartBlockedStart = -1.0;
    }
    else if( DepartBlockedStart < 0.0 ) {
        DepartBlockedStart = ElapsedTime;
    }
    // second pass, for diesel engines verify the (live) engines are fully started
    // TODO: cache presence of diesel engines in the consist, to skip this test if there isn't any
    p = pVehicles[ end::front ]; // pojazd na czole składu
    while( true == Ready
        && p != nullptr ) {

        auto const *vehicle { p->MoverParameters };

        if( has_diesel_engine() ) {

            Ready = vehicle->Vel > EU07_AI_MOVEMENT // already moving
			        || false == vehicle->Mains // deadweight vehicle
			        || vehicle->enrot > 0.8 * (vehicle->EngineType == TEngineType::DieselEngine ? vehicle->dizel_nmin : vehicle->DElist[0].RPM / 60.0);
        }
        p = p->Next(); // pojazd podłączony z tyłu (patrząc od czoła)
    }
    // test consist state for external events and/or weird things human user could've done
    iEngineActive &=
           false == IsAnyConverterOverloadRelayOpen
        && false == IsAnyLineBreakerOpen
        && ( false == IsAnyConverterPresent || true == IsAnyConverterEnabled )
        && ( false == IsAnyCompressorPresent || true == IsAnyCompressorEnabled )
        // the ai goes back to preparing the engine and switches that converter on, the human driver may run with it off
        && ( false == AIControllFlag || false == islocalconverterswitchoff );
}
