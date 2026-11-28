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

// the doors, the stops and the departure

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


// otwieranie/zamykanie drzwi w składzie albo (tylko AI) EZT
void TController::Doors( bool const Open, int const Side ) {
    // otwieranie drzwi
    if( true == Open ) {

        auto const lewe = pVehicle->DirectionGet() > 0 ? 1 : 2;
        auto const prawe = 3 - lewe;
        // doorsteps as the timetable asks, before the doors open (eu07/maszyna#95)
        if( auto const doorstep { timetable::doorstep( m_stopflags ) };
            doorstep.has_value() && mvOccupied->Doors.step_enabled != *doorstep ) {
            cue_action( *doorstep ? driver_hint::doorstepon : driver_hint::doorstepoff );
        }
        // grant door control permission if it's not automatic
        // TBD: stricter requirements?
        if( true == pVehicle->MoverParameters->Doors.permit_needed ) {
            if( Side & prawe ) {
                cue_action( driver_hint::doorrightpermiton );
            }
            if( Side & lewe ) {
                cue_action( driver_hint::doorleftpermiton );
            }
        }
        // consist-wide remote signals to open doors doors
        // NOTE: mixed controls are left up to passengers to open doors by themselves, unless the timetable has
        // the driver open all the doors
        if( true == timetable::opens_doors( *pVehicle->MoverParameters, m_stopflags ) ) {
            if( Side & prawe ) {
                cue_action( driver_hint::doorrightopen );
            }
            if( Side & lewe ) {
                cue_action( driver_hint::doorleftopen );
            }
        }
    }
    // zamykanie drzwi
    else {
        // if the doors are already closed and locked then there's nothing to do
        if( false == doors_permit_active()
         && false == doors_open() ) {
            drivingFlags.doorOpened = false;
        	drivingFlags.departureWarned = false;
            return;
        }

    	if (mvOccupied->Doors.has_warning && !drivingFlags.departureWarned)
    	{
    		if (!mvOccupied->DepartureSignal)
    		{
    			// Not warned yet, not warning yet - start the warning signal.
    			cue_action( driver_hint::departuresignalon ); // załącenie bzyczka
    			fActionTime = Random( -0.8, -0.3 ); // 0.3-0.8 second buzzer
    			return;
    		}
    		// Not warned yet, warning now - stop the warning signal and mark it as done.
    		cue_action( driver_hint::departuresignaloff );
    		fActionTime = Random( -0.2, 0.0 ); // Wait just a bit more before departing
    		drivingFlags.departureWarned = true;
    		return;
    	}

        if( true == doors_open() && (pVehicle->MoverParameters->Doors.close_control == control_t::conductor || pVehicle->MoverParameters->Doors.close_control == control_t::driver ||
		                             pVehicle->MoverParameters->Doors.close_control == control_t::mixed) ) {
            // consist-wide remote signals to close doors
			cue_action(driver_hint::doorrightclose);
			cue_action(driver_hint::doorleftclose);
		}
        if( true == doors_permit_active() ) {
            cue_action( driver_hint::doorrightpermitoff );
            cue_action( driver_hint::doorleftpermitoff );
        }
        // if applicable close manually-operated doors in vehicles which may ignore remote signals
        {
			const auto *vehicle = pVehicles[ end::front ]; // pojazd na czole składu
            while( vehicle != nullptr ) {
                // zamykanie drzwi w pojazdach - flaga zezwolenia była by lepsza
                auto const ismanualdoor {
                    vehicle->MoverParameters->Doors.auto_velocity == -1.f
                 && ( vehicle->MoverParameters->Doors.close_control == control_t::passenger
                   || vehicle->MoverParameters->Doors.close_control == control_t::mixed ) };

                if( true == ismanualdoor
                 && ( vehicle->LoadExchangeTime() == 0.f
                   || vehicle->MoverParameters->Vel > EU07_AI_MOVEMENT ) ) {
                    vehicle->MoverParameters->OperateDoors( side::right, false, range_t::local );
                    vehicle->MoverParameters->OperateDoors( side::left, false, range_t::local );
                }
                vehicle = vehicle->Next(); // pojazd podłączony z tyłu (patrząc od czoła)
            }
        }
		fActionTime = Random(-3.5, -1.0); // 1.0-3.5 sec wait
        drivingFlags.doorOpened = false; // zostały zamknięte - nie wykonywać drugi raz
    	drivingFlags.departureWarned = false;
    }
}

// returns true if any vehicle in the consist has open doors
bool
TController::doors_open() const {

    return IsAnyDoorOpen[side::right] || IsAnyDoorOpen[side::left];
}

bool
TController::doors_permit_active() const {

    return IsAnyDoorPermitActive[side::right] || IsAnyDoorPermitActive[side::left];
}

void
TController::announce( announcement_t const Announcement ) {

    m_lastannouncement = Announcement;

    if( IsCargoTrain ) {
        // cargo trains aren't likely to have announcement systems, so we can skip the procedure altogether
        return;
    }

    auto *vehicle { pVehicles[ end::front ] };
    while( vehicle ) {
        vehicle->announce( Announcement );
        vehicle = vehicle->Next();
    }
}

void
TController::control_doors() {

    if( false == doors_open() ) { return; }
    // jeżeli jedzie
    // nie zamykać drzwi przy drganiach, bo zatrzymanie na W4 akceptuje niewielkie prędkości
    if( mvOccupied->Vel > EU07_AI_MOVEMENT ) {
        Doors( false );
        return;
    }
    // HACK: crude way to deal with automatic door opening on W4 preventing further ride
    // for human-controlled vehicles with no door control and dynamic brake auto-activating with door open
    // TODO: check if this situation still happens and the hack is still needed
    if( false == AIControllFlag && (mvControlling->EngineType == TEngineType::DieselEngine ? mvControlling->RList[mvControlling->MainCtrlPos].Mn > 0 : mvControlling->MainCtrlPowerPos() > 0) ) {
        // for diesel engines react when engine is put past idle revolutions
        // for others straightforward master controller check
		Doors(false);
		return;
	}
}

void
TController::check_departure() {

    if(false == OrderCurrentGet().HasAny( DriverOrder::shunt | DriverOrder::looseShunt | DriverOrder::obeyTrain | DriverOrder::bank | DriverOrder::connect ) ) { return; }
    // odjechać sam może tylko jeśli jest w trybie jazdy
    // automatyczne ruszanie po odstaniu albo spod SBL
    if( VelSignal == 0.0
     && WaitingTime > 0.0
     && mvOccupied->RunningTrack.Velmax != 0.0 ) {
        // jeśli stoi, a upłynął czas oczekiwania i tor ma niezerową prędkość
        if( OrderCurrentGet() & (DriverOrder::shunt | DriverOrder::looseShunt | DriverOrder::obeyTrain | DriverOrder::bank)
         && drivingFlags.stopHere ) {
            // zakaz ruszania z miejsca bez otrzymania wolnej drogi
            WaitingTime = -WaitingExpireTime;
        }
        else {
            if( mvOccupied->CategoryFlag & 1 ) { // jeśli pociąg
                PrepareDirection(); // zmieni ustawiony kierunek
                SetVelocity( 20, 20 ); // jak się nastał, to niech jedzie 20km/h
                WaitingTime = 0.0;
                fWarningDuration = 1.5; // a zatrąbić trochę
            }
            else { // samochód ma stać, aż dostanie odjazd, chyba że stoi przez kolizję
                if( eStopReason == stopBlock
                 && Obstacle.distance > fDriverDist ) {
                    PrepareDirection(); // zmieni ustawiony kierunek
                    SetVelocity( -1, -1 ); // jak się nastał, to niech jedzie
                    WaitingTime = 0.0;
                }
            }
        }
    }
    else if( VelSignal == 0.0 && VelNext > 0.0 && mvOccupied->Vel < 1.0 && (false == iCoupler.Empty() || ( false == drivingFlags.stopHere )) ) {
		// Ra: tu jest coś nie tak, bo bez tego warunku ruszało w manewrowym !!!!
		SetVelocity(VelNext, VelNext, stopSem); // omijanie SBL
	}
}

// verify progress of load exchange
void
TController::check_load_exchange() {

    ExchangeTime = 0.f;
    DoesAnyDoorNeedOpening = false;

    if( fStopTime > 0 ) { return; }

    // czas postoju przed dalszą jazdą (np. na przystanku)
    auto *vehicle { pVehicles[ end::front ] };
    while( vehicle != nullptr ) {
        auto const vehicleexchangetime { vehicle->LoadExchangeTime() };
        DoesAnyDoorNeedOpening |= vehicleexchangetime > 0 && vehicle->LoadExchangeSpeed() == 0;
        ExchangeTime = std::max( ExchangeTime, vehicleexchangetime );
        vehicle = vehicle->Next();
    }

    if( pVehicle->DirectionGet() != m_lastexchangedirection ) {
        // generally means the ai driver moved to the opposite end of the consist
        // TODO: investigate whether user playing with the reverser can mess this up
        auto const left { m_lastexchangedirection > 0 ? 1 : 2 };
        auto const right { 3 - left };
        m_lastexchangeplatforms =
            ( ( m_lastexchangeplatforms & left )  != 0 ? right : 0 )
            + ( ( m_lastexchangeplatforms & right ) != 0 ? left : 0 );
        m_lastexchangedirection = pVehicle->DirectionGet();
    }
    if( false == TrainParams.IsMaintenance()
     && ( false == drivingFlags.doorOpened
       || true == DoesAnyDoorNeedOpening ) ) {
        drivingFlags.doorOpened = true; // nie wykonywać drugi raz
        remove_hint( driver_hint::doorleftopen );
        remove_hint( driver_hint::doorrightopen );
        Doors( true, m_lastexchangeplatforms );
    }
}

void
TController::GuardOpenDoor() {
	if (drivingFlags.guardOpenDoor) {
		auto *vehicle{ pVehicles[end::front] };
		while (vehicle != nullptr && vehicle->MoverParameters->Doors.range == 0) {
			vehicle = vehicle->Next();
		}
		if (vehicle != nullptr && vehicle->MoverParameters->Doors.range > 0) {
			auto const lewe = vehicle->DirectionGet() > 0 ? 1 : 2;
			auto const prawe = 3 - lewe;
			if (m_lastexchangeplatforms & lewe) {
				vehicle->MoverParameters->OperateDoors(side::left, true, range_t::local);
			}
			if (m_lastexchangeplatforms & prawe) {
				vehicle->MoverParameters->OperateDoors(side::right, true, range_t::local);
			}
		}
		else {
			drivingFlags.guardOpenDoor = false;
		}
	}
}
