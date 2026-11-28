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

// the obstacles and the signals behind and ahead of the train

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


basic_event * TController::CheckTrackEventBackward(double fDirection, TTrack *Track, TDynamicObject *Vehicle, int const Eventdirection, end const End)
{ // sprawdzanie eventu w torze, czy jest sygnałowym - skanowanie do tyłu
    // NOTE: this method returns only one event which meets the conditions, due to limitations in the caller
    // TBD, TODO: clean up the caller and return all suitable events, as in theory things will go awry if the track has more than one signal
    auto const dir{ Vehicle->VectorFront() * (double)Vehicle->DirectionGet() };
    auto const pos{ End == end::front ? Vehicle->RearPosition() : Vehicle->HeadPosition() };
    auto const &eventsequence { ( fDirection * Eventdirection > 0 ? Track->m_events2 : Track->m_events1 ) };
    for( auto const &event : eventsequence ) {
        if( event.second != nullptr
         && event.second->m_passive
         && typeid(*event.second) == typeid(getvalues_event) ) {
            // since we're checking for events behind us discard the sources in front of the scanning vehicle
            auto const sl{ event.second->input_location() }; // położenie komórki pamięci
            auto const sem{ sl - pos }; // wektor do komórki pamięci od końca składu
            auto const isahead { dir.x * sem.x + dir.z * sem.z > 0 };
            if( End == end::front ? isahead : !isahead ) {
                // iloczyn skalarny jest ujemny, gdy sygnał stoi z tyłu
                return event.second;
            }
        }
    }
    return nullptr;
};

TTrack * TController::BackwardTraceRoute(double &fDistance, double &fDirection, TDynamicObject *Vehicle, basic_event *&Event, int const Eventdirection, end const End, bool const Untiloccupied )
{ // szukanie sygnalizatora w kierunku przeciwnym jazdy (eventu odczytu komórki pamięci)
    auto *Track = Vehicle->GetTrack();
    double fCurrentDistance = Vehicle->RaTranslationGet(); // aktualna pozycja na torze
    if (fDirection > 0) // jeśli w kierunku Point2 toru
        fCurrentDistance = Track->Length() - fCurrentDistance;
    double s = 0;

    if( Track->VelocityGet() == 0.0 ) { // jak prędkosć 0 albo uszkadza, to nie ma po co skanować
        fDistance = s; // to na tym torze stoimy
        return nullptr; // stop, skanowanie nie dało sensownych rezultatów
    }
    // jak tor zajęty innym składem, to nie ma po co skanować
    if( Untiloccupied && IsOccupiedByAnotherConsist( Track, fCurrentDistance * fDirection ) ) {
        fDistance = s; // to na tym torze stoimy
        return nullptr; // stop, skanowanie nie dało sensownych rezultatów
    }
    if ((Event = CheckTrackEventBackward(fDirection, Track, Vehicle, Eventdirection, End)) != nullptr)
    { // jeśli jest semafor na tym torze
        fDistance = s; // to na tym torze stoimy
        return Track;
    }
    TTrack *pTrackFrom; // odcinek poprzedni, do znajdywania końca dróg
    while (s < fDistance)
    {
        // Track->ScannedFlag=true; //do pokazywania przeskanowanych torów
        pTrackFrom = Track; // zapamiętanie aktualnego odcinka
        s += fCurrentDistance; // doliczenie kolejnego odcinka do przeskanowanej długości
        if (fDirection > 0)
        { // jeśli szukanie od Point1 w kierunku Point2
            if (Track->iNextDirection)
                fDirection = -fDirection;
            Track = Track->CurrentNext(); // może być NULL
        }
        else // if (fDirection<0)
        { // jeśli szukanie od Point2 w kierunku Point1
            if (!Track->iPrevDirection)
                fDirection = -fDirection;
            Track = Track->CurrentPrev(); // może być NULL
        }
        if (Track == pTrackFrom)
            Track = nullptr; // koniec, tak jak dla torów

        // gdy dalej toru nie ma albo zerowa prędkość, albo uszkadza pojazd
        // zwraca NULL, że skanowanie nie dało sensownych rezultatów
        if( Track == nullptr ) {
            fDistance = s;
            return nullptr;
        }
        if( Track->VelocityGet() == 0.0 )
        {
            fDistance = s;
            return nullptr;
        }
        if( Untiloccupied && IsOccupiedByAnotherConsist( Track ) ) {
            fDistance = s;
            return nullptr;
        }
        fCurrentDistance = Track->Length();
        if ((Event = CheckTrackEventBackward(fDirection, Track, Vehicle, Eventdirection, End)) != nullptr)
        { // znaleziony tor z eventem
            fDistance = s;
            return Track;
        }
    }
    Event = nullptr; // jak dojdzie tu, to nie ma semafora
    // zwraca ostatni sprawdzony tor
    fDistance = s;
    return Track;
}

// sprawdzanie zdarzeń semaforów i ograniczeń szlakowych
void TController::SetProximityVelocity( double dist, double vel, glm::dvec3 const *pos )
{ // Ra:przeslanie do AI prędkości
    /*
     //!!!! zastąpić prawidłową reakcją AI na SetProximityVelocity !!!!
     if (vel==0)
     {//jeśli zatrzymanie, to zmniejszamy dystans o 10m
      dist-=10.0;
     };
     if (dist<0.0) dist=0.0;
     if ((vel<0)?true:dist>0.1*(MoverParameters->Vel*MoverParameters->Vel-vel*vel)+50)
     {//jeśli jest dalej od umownej drogi hamowania
    */
    PutCommand( "SetProximityVelocity", dist, vel, pos );
    /*
     }
     else
     {//jeśli jest zagrożenie, że przekroczy
      Mechanik->SetVelocity(floor(0.2*sqrt(dist)+vel),vel,stopError);
     }
     */
}

TCommandType TController::BackwardScan( double const Range )
{ // sprawdzanie zdarzeń semaforów z tyłu pojazdu, zwraca komendę
    // dzięki temu będzie można stawać za wskazanym sygnalizatorem, a zwłaszcza jeśli będzie jazda na kozioł
    // ograniczenia prędkości nie są wtedy istotne, również koniec toru jest do niczego nie przydatny
    // zwraca true, jeśli należy odwrócić kierunek jazdy pojazdu
    if( OrderCurrentGet().Without(DriverOrder::shunt | DriverOrder::looseShunt | DriverOrder::connect) ) {
        // skanowanie sygnałów tylko gdy jedzie w trybie manewrowym albo czeka na rozkazy
        return TCommandType::cm_Unknown;
    }
    // kierunek jazdy względem sprzęgów pojazdu na czele
    int const startdir = -pVehicles[0]->DirectionGet();
    if( startdir == 0 ) {
        // jeśli kabina i kierunek nie jest określony nie robimy nic
        return TCommandType::cm_Unknown;
    }
    // szukamy od pierwszej osi w wybranym kierunku
    double scandir = startdir * pVehicles[0]->RaDirectionGet();

    if( scandir == 0.0 ) { return TCommandType::cm_Unknown; }

    // skanowanie toru w poszukiwaniu eventów GetValues (PutValues nie są przydatne)
    // Ra: przy wstecznym skanowaniu prędkość nie ma znaczenia
    double scandist = Range; // zmodyfikuje na rzeczywiście przeskanowane
    basic_event *e = nullptr; // event potencjalnie od semafora
    // opcjonalnie może być skanowanie od "wskaźnika" z przodu, np. W5, Tm=Ms1, koniec toru wg
    // drugiej osi w kierunku ruchu
    auto const *scantrack{BackwardTraceRoute(scandist, scandir, pVehicles[end::front], e)};
    auto const dir{(double)startdir *
                   pVehicles[end::front]->VectorFront()}; // wektor w kierunku jazdy/szukania

    // jeśli wstecz wykryto koniec toru to raczej nic się nie da w takiej sytuacji zrobić
    if (e == nullptr)
    {
        return TCommandType::cm_Unknown;
    }
    if (typeid(*e) != typeid(getvalues_event))
    {
        return TCommandType::cm_Unknown;
    }

    // jeśli jest jakiś sygnał na widoku
    double scanvel; // prędkość ustawiona semaforem
#if LOGBACKSCAN
    std::string edir{"Backward scan by " + pVehicle->asName + " - " +
                     ((scandir > 0) ? "Event2 " : "Event1 ")};
    edir += "(" + (e->name()) + ")";
#endif
    {
        // najpierw sprawdzamy, czy semafor czy inny znak został przejechany
        auto const sl{e->input_location()}; // położenie komórki pamięci
        auto const pos{pVehicles[end::rear]->RearPosition()}; // pozycja tyłu
        if (auto const sem{sl - pos}; dir.x * sem.x + dir.z * sem.z < 0) // wektor do komórki pamięci od końca składu
        {
            // jeśli został minięty
            // iloczyn skalarny jest ujemny, gdy sygnał stoi z tyłu
#if LOGBACKSCAN
            WriteLog(edir + " - ignored as not passed yet");
#endif
            return TCommandType::cm_Unknown; // nic
        }
        scanvel = e->input_value(1); // prędkość przy tym semaforze
#if LOGBACKSCAN
        // przeliczamy odległość od semafora - potrzebne by były współrzędne początku składu
        scandist = std::max(glm::length(sl - pos) - 2.0, 0.0); // 2m luzu przy manewrach wystarczy, ujemnych nie ma po co wysyłać
#endif
    }

    auto move{false}; // czy AI w trybie manewerowym ma dociągnąć pod S1
    if (e->input_command() == TCommandType::cm_SetVelocity)
    {
        if (scanvel == 0.0 ? OrderCurrentGet() & (DriverOrder::shunt | DriverOrder::looseShunt | DriverOrder::connect) :
                               OrderCurrentGet() & DriverOrder::connect)
        { // przy podczepianiu ignorować wyjazd?
            move = true; // AI w trybie manewerowym ma dociągnąć pod S1
        }
        else
        {
            // jeśli semafor jest daleko, a pojazd jedzie, to informujemy o zmianie prędkości
            // jeśli jedzie manewrowo, musi dostać SetVelocity, żeby sie na pociągowy przełączył
            // w przeciwnym razie ustawiamy prędkość tylko wtedy, gdy ma ruszyć, stanąć albo ma stać
            // ruszać stop trzeba powtarzać, bo inaczej zatrąbi i pojedzie sam
#if LOGBACKSCAN
            if (scandist > fMinProximityDist &&
                mvOccupied->Vel > EU07_AI_NOMOVEMENT && false == OrderCurrentGet().HasAny(DriverOrder::shunt | DriverOrder::looseShunt))
            {
                WriteLog(edir);
            }
            else
            {
                WriteLog(edir + " - [SetVelocity] [" + to_string(scanvel, 2) + "] [" +
                         to_string(e->input_value(2), 2) + "]");
            }
#endif
            return scanvel > 0 ? TCommandType::cm_SetVelocity : TCommandType::cm_Unknown;
        }
    }
    // reakcja AI w trybie manewrowym dodatkowo na sygnały manewrowe
    if ((false == OrderCurrentGet().Empty() ? OrderCurrentGet().HasAny(DriverOrder::shunt | DriverOrder::looseShunt | DriverOrder::connect) : true) && (move || e->input_command() == TCommandType::cm_ShuntVelocity)) // w Wait_for_orders też widzi tarcze
    {
		// jeśli powyżej było SetVelocity 0 0, to dociągamy pod S1
		/*
		            if ((scandist > fMinProximityDist) &&
		                ((mvOccupied->Vel > EU07_AI_NOMOVEMENT) || (scanvel == 0.0)))
		            {
		                // jeśli tarcza jest daleko, to:
		                //- jesli pojazd jedzie, to informujemy o zmianie prędkości
		                //- jeśli stoi, to z własnej inicjatywy może podjechać pod zamkniętą tarczę
		                if (mvOccupied->Vel > EU07_AI_NOMOVEMENT) // tylko jeśli jedzie
		                { // Mechanik->PutCommand("SetProximityVelocity",scandist,scanvel,sl);
		#if LOGBACKSCAN
		                  // WriteLog(edir+"SetProximityVelocity "+AnsiString(scandist)+"
		                    // "+AnsiString(scanvel));
		                    WriteLog(edir);
		#endif
		                    // SetProximityVelocity(scandist,scanvel,&sl);
		                    // jeśli jedzie na W5 albo koniec toru, to można zmienić kierunek
		                    return (drivingFlags.trackEnd) ? TCommandType::cm_ChangeDirection :
		                                                           TCommandType::cm_Unknown;
		                    // TBD: request direction change also if VelNext in current direction is 0,
		                    // while signal behind requests movement?
		                }
		            }
		            else
		            {
		                // ustawiamy prędkość tylko wtedy, gdy ma ruszyć, albo stanąć albo ma stać pod
		                // tarczą stop trzeba powtarzać, bo inaczej zatrąbi i pojedzie sam if
		                // ((MoverParameters->Vel==0.0)||(scanvel==0.0)) //jeśli jedzie lub ma stanąć/stać
		                { // nie dostanie komendy jeśli jedzie i ma jechać
		                  // PutCommand("ShuntVelocity",scanvel,e->Params[9].asMemCell->Value2(),&sl,stopSem);
		#if LOGBACKSCAN
		                    WriteLog(edir + " - [ShuntVelocity] [" + to_string(scanvel, 2) + "] [" +
		                             to_string(e->input_value(2), 2) + "]");
		#endif
		                    return (scanvel > 0 ? TCommandType::cm_ShuntVelocity :
		                                          TCommandType::cm_Unknown);
		                }
		            }
		            // NOTE: dead code due to returns in branching above
		            if ((scanvel != 0.0) && (scandist < 100.0))
		            {
		                // jeśli Tm w odległości do 100m podaje zezwolenie na jazdę, to od razu ją
		                // ignorujemy, aby móc szukać kolejnej eSignSkip=e; //wtedy uznajemy ignorowaną przy
		                // poszukiwaniu nowej
		#if LOGBACKSCAN
		                WriteLog(edir + " - will be ignored due to Ms2");
		#endif
		                return (scanvel > 0 ? TCommandType::cm_ShuntVelocity : TCommandType::cm_Unknown);
		            }
		*/
		return VelNext > 0 ?
                        TCommandType::cm_Unknown : // no need to bother if we can continue in current direction
                        scanvel == 0 ?
                            TCommandType::cm_Unknown : // no need to bother with a stop signal
                            TCommandType::cm_ShuntVelocity; // otherwise report a relevant shunt signal behind
		// if (move?...
    } // if (OrderCurrentGet()==Shunt)

    return e->m_passive && e->is_command() ? TCommandType::cm_Command : TCommandType::cm_Unknown;
};

std::string TController::NextStop() const
{ // informacja o następnym zatrzymaniu, wyświetlane pod [F1]
    if (asNextStop == "[End of route]")
        return ""; // nie zawiera nazwy stacji, gdy dojechał do końca
    // dodać godzinę odjazdu
    // the name the way the timetable spells it, in utf-8 as the user interface takes it
    auto nextstop{ TrainParams.station_caption( asNextStop ) };
    auto const *t{ TrainParams.TimeTable + TrainParams.StationIndex };
    if( t->Ah >= 0 ) {
        // przyjazd
        nextstop += "  przyj." + std::to_string( t->Ah ) + ":"
      + to_minutes_str( t->Am, true, 3 );

    }
    if( t->Dh >= 0 ) {
        // jeśli jest godzina odjazdu
        nextstop += " odj." + std::to_string( t->Dh ) + ":"
      + to_minutes_str( t->Dm, true, 3 );
    }
    return nextstop;
};

void TController::UpdateDelayFlag() {

    if( TrainParams.CheckTrainLatency() < 0.0 ) {
        // odnotowano spóźnienie
        drivingFlags.late = true;
    }
    else {
        // przyjazd o czasie
        drivingFlags.late = false;
    }
}

// returns most recently calculated distance to potential obstacle ahead
double
TController::TrackObstacle() const {

    return Obstacle.distance;
}

// check for potential collisions
void
TController::scan_obstacles( double const Range ) {
    // HACK: vehicle order in the consist is based on intended travel direction
    // if our actual travel direction doesn't match that, we should be scanning from the other end of the consist
    // we cast to int to avoid getting confused by microstutters
    auto *frontvehicle { pVehicles[ ( static_cast<int>( mvOccupied->V ) * iDirection >= 0 ? end::front : end::rear ) ] };

    int routescandirection;
    // for moving vehicle determine heading from velocity; for standing fall back on the set direction
    if( std::abs(frontvehicle->MoverParameters->V) > 0.5 ? // ignore potential micro-stutters in oposite direction during "almost stop"
	        frontvehicle->MoverParameters->V > 0.0 :
	        pVehicle->DirectionGet() == frontvehicle->DirectionGet() ? iDirection >= 0 : iDirection <= 0 ) {
        // towards coupler 0
        routescandirection = end::front;
    }
    else {
        // towards coupler 1
        routescandirection = end::rear;
    }
/*
    if( pVehicle->MoverParameters->CabOccupied < 0 ) {
        // flip the scan direction in the rear cab
        routescandirection ^= routescandirection;
    }
*/
    Obstacle = neighbour_data();
    auto const obstaclescanrange { std::max( is_car() ? 250.0 : 1000.0, Range ) };
    auto const lookup { frontvehicle->find_vehicle( routescandirection, obstaclescanrange ) };

    if( std::get<bool>( lookup ) == true ) {

        Obstacle.vehicle = std::get<TDynamicObject *>( lookup );
        Obstacle.vehicle_end = std::get<int>( lookup );
        Obstacle.distance = std::get<double>( lookup );

        if( Obstacle.distance < ( is_car() ? 50 : 100 ) ) {
            // at short distances (re)calculate range between couplers directly
            Obstacle.distance = TMoverParameters::CouplerDist( frontvehicle->MoverParameters, Obstacle.vehicle->MoverParameters );
        }
    }
}

void
TController::check_route_ahead( double const Range ) {

    auto const comm { TableUpdate( VelDesired, ActualProximityDist, VelNext, AccDesired ) };

    switch (comm) {
    // ustawienie VelSignal - trochę proteza = do przemyślenia
    case TCommandType::cm_Ready: { // W4 zezwolił na jazdę
        // ewentualne doskanowanie trasy za W4, który zezwolił na jazdę
        TableCheck( Range );
        TableUpdate( VelDesired, ActualProximityDist, VelNext, AccDesired ); // aktualizacja po skanowaniu
        if( VelNext == 0.0 ) {
            break; // ale jak coś z przodu zamyka, to ma stać
        }
        if( drivingFlags.stopCloser ) {
            VelSignal = -1.0; // ma czekać na sygnał z sygnalizatora!
        }
        break;
    }
    case TCommandType::cm_SetVelocity: { // od wersji 357 semafor nie budzi wyłączonej lokomotywy
        if( OrderCurrentGet().Without(DriverOrder::shunt | DriverOrder::looseShunt | DriverOrder::obeyTrain | DriverOrder::bank).Empty() && std::fabs(VelSignal) >= 1.0 ) { // jedzie w dowolnym trybie albo Wait_for_orders
			// 0.1 nie wysyła się do samochodow, bo potem nie ruszą
			PutCommand("SetVelocity", VelSignal, VelNext, nullptr); // komenda robi dodatkowe operacje
		}
        break;
    }
    case TCommandType::cm_ShuntVelocity: { // od wersji 357 Tm nie budzi wyłączonej lokomotywy
        if( OrderCurrentGet().Without( DriverOrder::shunt | DriverOrder::looseShunt | DriverOrder::obeyTrain | DriverOrder::bank ).Empty() ) { // jedzie w dowolnym trybie albo Wait_for_orders
            PutCommand( "ShuntVelocity", VelSignal, VelNext, nullptr );
        }
        else if( false == iCoupler.Empty() ) { // jeśli jedzie w celu połączenia
            SetVelocity( VelSignal, VelNext );
        }
        break;
    }
    case TCommandType::cm_Command: { // komenda z komórki
        if( OrderCurrentGet().Without(DriverOrder::shunt | DriverOrder::looseShunt | DriverOrder::obeyTrain | DriverOrder::bank).Empty() && mvOccupied->Vel < 0.1 ) {
            // jedzie w dowolnym trybie albo Wait_for_orders
			// dopiero jak stanie
			/*
			                PutCommand( eSignNext->text(), eSignNext->value( 1 ), eSignNext->value( 2 ), nullptr );
			                eSignNext->StopCommandSent(); // się wykonało już
			*/                      // replacement of the above
			eSignNext->send_command(*this);
		}
        break;
    }
    default: {
        break;
    }
    }
}

void
TController::check_route_behind( double const Range ) {

    if( VelNext != 0.0 ) { return; } // not if we can move ahead
    if( drivingFlags.connect ) { return; } // not if we're in middle of coupling operation

    if( OrderCurrentGet().Without( DriverOrder::shunt | DriverOrder::looseShunt | DriverOrder::connect ).Empty() ) {
        // jedzie w Shunt albo Connect, albo Wait_for_orders
        // w trybie Connect skanować do tyłu tylko jeśli przed kolejnym sygnałem nie ma taboru do podłączenia
        auto const couplinginprogress {
            OrderCurrentGet().HasAll(DriverOrder::connect)
         && ( Obstacle.distance < std::min(2000.0, FirstSemaphorDist)
           || drivingFlags.connect ) };
        if( couplinginprogress ) { return; }

        auto const commandbehind { BackwardScan( Range ) };
        if( commandbehind != TCommandType::cm_Unknown ) {
            // jeśli w drugą można jechać
            // należy sprawdzać odległość od znalezionego sygnalizatora,
            // aby w przypadku prędkości 0.1 wyciągnąć najpierw skład za sygnalizator
            // i dopiero wtedy zmienić kierunek jazdy, oczekując podania prędkości >0.5
            if( commandbehind == TCommandType::cm_Command ) {
                // jeśli komenda Shunt to ją odbierz bez przemieszczania się (np. odczep wagony po dopchnięciu do końca toru)
                drivingFlags.stopHere = true;
            }
            iDirectionOrder = -iDirection; // zmiana kierunku jazdy
            // zmiana kierunku bez psucia kolejnych komend
            OrderCurrentSet( (OrderCurrentGet() | DriverOrder::changeDirection) );
        }
    }
}
