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

// the orders of the driver and the commands it gets from the scenery

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


std::string StopReasonTable[] = {
    // przyczyny zatrzymania ruchu AI
    "", // stopNone, //nie ma powodu - powinien jechać
    "Off", // stopSleep, //nie został odpalony, to nie pojedzie
    "Semaphore", // stopSem, //semafor zamknięty
    "Time", // stopTime, //czekanie na godzinę odjazdu
    "End of track", // stopEnd, //brak dalszej części toru
    "Change direction", // stopDir, //trzeba stanąć, by zmienić kierunek jazdy
    "Joining", // stopJoin, //zatrzymanie przy (p)odczepianiu
    "Block", // stopBlock, //przeszkoda na drodze ruchu
    "A command", // stopComm, //otrzymano taką komendę (niewiadomego pochodzenia)
    "Out of station", // stopOut, //komenda wyjazdu poza stację (raczej nie powinna zatrzymywać!)
    "Radiostop", // stopRadio, //komunikat przekazany radiem (Radiostop)
    "External", // stopExt, //przesłany z zewnątrz
    "Error", // stopError //z powodu błędu w obliczeniu drogi hamowania
};

// zamiana kodu rozkazu na opis
std::string TController::Order2Str(DriverOrders Order) const {

    if( Order.HasAny(DriverOrder::changeDirection)
     && Order != DriverOrder::changeDirection ) {
        // może być nałożona na inną i wtedy ma priorytet
        return
            Order2Str( DriverOrder::changeDirection )
            + " + "
            + Order2Str( Order.Without(DriverOrder::changeDirection) );
    }
    switch( SingleOrder( Order ) ) {
        case DriverOrder::waitForOrders:     return "Wait_for_orders";
        case DriverOrder::prepareEngine:      return "Prepare_engine";
        case DriverOrder::releaseEngine:      return "Release_engine";
        case DriverOrder::changeDirection:    return "Change_direction";
        case DriverOrder::connect:             return "Connect";
        case DriverOrder::disconnect:          return "Disconnect";
        case DriverOrder::shunt:               return "Shunt";
        case DriverOrder::looseShunt:         return "Loose_shunt";
        case DriverOrder::obeyTrain:          return "Obey_train";
        case DriverOrder::bank:                return "Bank";
        case DriverOrder::jumpToFirstOrder: return "Jump_to_first_order";
        default:                  return "Undefined";
    }
}

std::array<char, 64> orderbuffer;

std::string TController::OrderCurrent() const
{ // pobranie aktualnego rozkazu celem wyświetlenia
    auto const order { OrderCurrentGet() };
    // generally prioritize direction change...
    if( order & DriverOrder::changeDirection && false == drivingFlags.connect ) {
        // ...but not in the middle of coupling operation
		return STR("Change direction");
	}

    switch( SingleOrder( order.Without(DriverOrder::changeDirection) ) ) {
        case DriverOrder::waitForOrders: { return STR("Wait for orders"); }
        case DriverOrder::prepareEngine: { return STR("Start the engine"); }
        case DriverOrder::releaseEngine: { return STR("Shut down the engine"); }
        case DriverOrder::changeDirection: { return STR("Change direction"); }
        case DriverOrder::connect: { return STR("Couple to consist ahead"); }
        case DriverOrder::disconnect: {
            if( iVehicleCount < 0 ) {
                // done with uncoupling, order should update shortly
				return STR("Wait for orders");
            }
            // try to provide some task details
            auto const count { iVehicleCount };

            if( iVehicleCount > 1 ) {
                std::snprintf(
                    orderbuffer.data(), orderbuffer.size(),
				    STR_C("the engine plus %d next vehicles"), // TODO: implement plural forms
                    count );
            }
            auto const countstring { (
				count == 0 ? STR("the engine") :
				count == 1 ? STR("the engine plus the next vehicle") :
                orderbuffer.data() ) };

            std::snprintf(
                orderbuffer.data(), orderbuffer.size(),
			    STR_C("Uncouple %s"),
                countstring.c_str() );
            return orderbuffer.data();
        }
	    case DriverOrder::shunt: { return STR("Shunt according to signals"); }
	    case DriverOrder::looseShunt: { return STR("Loose shunt according to signals"); }
	    case DriverOrder::obeyTrain: { return STR("Drive according to signals and timetable"); }
	    case DriverOrder::bank: { return STR("Bank consist ahead"); }
        default: { return{}; }
    }
};

void TController::OrdersClear()
{ // czyszczenie tabeli rozkazów na starcie albo po dojściu do końca
    OrderPos = 0;
    OrderTop = 1; // szczyt stosu rozkazów
    for (int b = 0; b < maxorders; b++)
        OrderList[b] = DriverOrder::waitForOrders;
#if LOGORDERS
    OrdersDump("OrdersClear", false);
#endif
};

void TController::RecognizeCommand()
{ // odczytuje i wykonuje komendę przekazaną lokomotywie
    TCommand *c = &mvOccupied->CommandIn;
    PutCommand(c->Text, c->Value1, c->Value2, c->Location, stopComm);
    c->Command = VehicleCommand::none;
    c->Argument.clear();
    c->Text = ""; // usunięcie obsłużonej komendy
}

void TController::PutCommand(std::string NewCommand, double NewValue1, double NewValue2, const TLocation &NewLocation, TStopReason reason)
{ // wysłanie komendy przez event PutValues, jak pojazd ma obsadę, to wysyła tutaj, a nie do pojazdu bezpośrednio
    // zamiana na współrzędne scenerii
    glm::dvec3 sl { -NewLocation.X, NewLocation.Z, NewLocation.Y };
    if (!PutCommand(NewCommand, NewValue1, NewValue2, &sl, reason))
        mvOccupied->PutCommand(NewCommand, NewValue1, NewValue2, NewLocation);
}

bool TController::PutCommand( std::string NewCommand, double NewValue1, double NewValue2, glm::dvec3 const *NewLocation, TStopReason reason )
{ // analiza komendy
    if (NewCommand == "CabSignal")
    { // SHP wyzwalane jest przez człon z obsadą, ale obsługiwane przez silnikowy
        // nie jest to najlepiej zrobione, ale bez symulacji obwodów lepiej nie będzie
        // Ra 2014-04: jednak przeniosłem do rozrządczego
        mvOccupied->PutCommand(NewCommand, NewValue1, NewValue2, mvOccupied->Loc);
        mvOccupied->RunInternalCommand(); // rozpoznaj komende bo lokomotywa jej nie rozpoznaje
        return true; // załatwione
    }

    if (NewCommand == "Overhead")
    { // informacja o stanie sieci trakcyjnej
        fOverhead1 = NewValue1; // informacja o napięciu w sieci trakcyjnej (0=brak drutu, zatrzymaj!)
        fOverhead2 = NewValue2; // informacja o sposobie jazdy (-1=normalnie, 0=bez prądu, >0=z
        // opuszczonym i ograniczeniem prędkości)
        return true; // załatwione
    }

    if (NewCommand == "Emergency_brake") // wymuszenie zatrzymania, niezależnie kto prowadzi
    { // Ra: no nadal nie jest zbyt pięknie
//        SetVelocity(0, 0, reason);
        mvOccupied->PutCommand("Emergency_brake", 1.0, 1.0, mvOccupied->Loc);
        return true; // załatwione
    }

    if (NewCommand.compare(0, 10, "Timetable:") == 0)
    { // przypisanie nowego rozkładu jazdy, również prowadzonemu przez użytkownika
        NewCommand.erase(0, 10); // zostanie nazwa pliku z rozkładem
#if LOGSTOPS
        WriteLog("New timetable for " + pVehicle->asName + ": " + NewCommand); // informacja
#endif
        TrainParams = TTrainParameters(NewCommand); // rozkład jazdy
        tsGuardSignal = sound_source { sound_placement::internal, 2 * EU07_SOUND_CABCONTROLSCUTOFFRANGE }; // wywalenie kierownika
        m_lastexchangestop.clear();
        m_stopflags = {};
        if (NewCommand != "none")
        {
            if (false == TrainParams.LoadTTfile(
                    std::string(Global.asCurrentSceneryPath.c_str()), floor(NewValue2 + 0.5),
                    NewValue1)) // pierwszy parametr to przesunięcie rozkładu w czasie
            {
                if (ConversionError == -8)
                    ErrorLog("Missed timetable: " + NewCommand);
                WriteLog("Cannot load timetable file " + NewCommand + "\r\nError " +
                         std::to_string(ConversionError) + " in position " + std::to_string(TrainParams.StationCount));
                NewCommand = ""; // puste, dla wymiennej tekstury
            }
            else
            { // inicjacja pierwszego przystanku i pobranie jego nazwy
                // HACK: clear the potentially set door state flag to ensure door get opened if applicable
                if( true == AIControllFlag ) {
                    // simplified door closing procedure, to sync actual door state with the door state flag
                    // NOTE: this may result in visually ugly quick switch between closing and opening the doors, but eh
                    if( pVehicle->MoverParameters->Doors.close_control == control_t::driver
                     || pVehicle->MoverParameters->Doors.close_control == control_t::mixed ) {
                        pVehicle->MoverParameters->OperateDoors( side::right, false );
                        pVehicle->MoverParameters->OperateDoors( side::left, false );
                    }
                    if( pVehicle->MoverParameters->Doors.permit_needed ) {
                        pVehicle->MoverParameters->PermitDoors( side::right, false );
                        pVehicle->MoverParameters->PermitDoors( side::left, false );
                    }
                }
                drivingFlags.doorOpened = false;

                TrainParams.UpdateMTable( simulation::Time, TrainParams.NextStationName );
                TrainParams.StationIndexInc(); // przejście do następnej
                TrainParams.StationStart = TrainParams.StationIndex;
                asNextStop = TrainParams.NextStop();
                // the flags of the first station until the train stops there
                m_stopflags = TrainParams.stop_flags();
                m_lastannouncement = announcement_t::idle;
                drivingFlags.primary = true; // skoro dostał rozkład, to jest teraz głównym
//                NewCommand = Global.asCurrentSceneryPath + NewCommand;
                auto lookup =
                    FileExists(
                        { Global.asCurrentSceneryPath + NewCommand, paths::sounds + NewCommand },
                        { ".ogg", ".flac", ".wav" } );
                if( false == lookup.first.empty() ) {
                    //  wczytanie dźwięku odjazdu podawanego bezpośrenido
                    tsGuardSignal = sound_source( sound_placement::external, 75.f ).deserialize( lookup.first + lookup.second, sound_type::single );
                    iGuardRadio = 0; // nie przez radio
                }
                else {
                    NewCommand += "radio";
                    lookup =
                        FileExists(
                            { Global.asCurrentSceneryPath + NewCommand, paths::sounds + NewCommand },
                            { ".ogg", ".flac", ".wav" } );
                    if( false == lookup.first.empty() ) {
                        //  wczytanie dźwięku odjazdu w wersji radiowej (słychać tylko w kabinie)
                        tsGuardSignal = sound_source( sound_placement::internal, EU07_SOUND_HANDHELDRADIORANGE ).deserialize( lookup.first + lookup.second, sound_type::single );
                        iGuardRadio = iRadioChannel;
                    }
                }
                NewCommand = TrainParams.Relation2; // relacja docelowa z rozkładu
            }
            // jeszcze poustawiać tekstury na wyświetlaczach
            TDynamicObject *p = pVehicles[end::front];
            while (p)
            {
                p->DestinationSet(NewCommand, TrainParams.TrainName); // relacja docelowa
                p = p->Next(); // pojazd podłączony od tyłu (licząc od czoła)
            }
        }
        if (NewLocation) // jeśli podane współrzędne eventu/komórki ustawiającej rozkład (trainset
        // nie podaje)
        {
            auto v = *NewLocation - pVehicle->GetPosition(); // wektor do punktu sterującego
            auto d = pVehicle->VectorFront(); // wektor wskazujący przód
            iDirectionOrder = (v.x * d.x + v.z * d.z) * NewValue1 > 0 ?
                                  1 :
                                  -1; // do przodu, gdy iloczyn skalarny i prędkość dodatnie
            /*
              if (NewValue1!=0.0) //jeśli ma jechać
               if (iDirectionOrder==0) //a kierunek nie był określony (normalnie określany przez
              reardriver/headdriver)
                iDirectionOrder=NewValue1>0?1:-1; //ustalenie kierunku jazdy względem sprzęgów
               else
                if (NewValue1<0) //dla ujemnej prędkości
                 iDirectionOrder=-iDirectionOrder; //ma jechać w drugą stronę
            */
            if (AIControllFlag) // jeśli prowadzi komputer
                Activation(); // umieszczenie obsługi we właściwym członie, ustawienie nawrotnika w
            // przód
        }
        /*
          else
           if (!iDirectionOrder)
          {//jeśli nie ma kierunku, trzeba ustalić
           if (mvOccupied->V!=0.0)
            iDirectionOrder=mvOccupied->V>0?1:-1;
           else
            iDirectionOrder=mvOccupied->CabOccupied;
           if (!iDirectionOrder) iDirectionOrder=1;
          }
        */
        // jeśli wysyłane z Trainset, to wszystko jest już odpowiednio ustawione
        // if (!NewLocation) //jeśli wysyłane z Trainset
        // if (mvOccupied->CabActive*mvOccupied->V*NewValue1<0) //jeśli zadana prędkość niezgodna z
        // aktualnym kierunkiem jazdy
        //  DirectionForward(false); //jedziemy do tyłu (nawrotnik do tyłu)
        // CheckVehicles(); //sprawdzenie składu, AI zapali światła
        TableClear(); // wyczyszczenie tabelki prędkości, bo na nowo trzeba określić kierunek i
        // sprawdzić przystanki
        OrdersInit(fabs(NewValue1)); // ustalenie tabelki komend wg rozkładu oraz prędkości początkowej
        // if (NewValue1!=0.0) if (!AIControllFlag) DirectionForward(NewValue1>0.0); //ustawienie
        // nawrotnika użytkownikowi (propaguje się do członów)
        // if (NewValue1>0)
        // TrainNumber=floor(NewValue1); //i co potem ???
        return true; // załatwione
    }

    if (NewCommand == "SetVelocity")
    {
        if (NewLocation)
            vCommandLocation = *NewLocation;
        if (NewValue1 != 0.0 && OrderCurrentGet() != DriverOrder::obeyTrain)
        { // o ile jazda
            if( false == iEngineActive ) {
                // trzeba odpalić silnik najpierw, światła ustawi
                OrderNext( DriverOrder::prepareEngine );
            }
            OrderNext(DriverOrder::obeyTrain); // to uruchomić jazdę pociągową (od razu albo po odpaleniu silnika
            OrderCheck(); // jeśli jazda pociągowa teraz, to wykonać niezbędne operacje
        }
        if (NewValue1 != 0.0) // jeśli jechać
            drivingFlags.stopHere = false; // podjeżanie do semaforów zezwolone
        else
            drivingFlags.stopHere = true; // stać do momentu podania komendy jazdy
        SetVelocity(NewValue1, NewValue2, reason); // bylo: nic nie rob bo SetVelocity zewnetrznie
        // jest wywolywane przez dynobj.cpp
        return true;
    }

    if( NewCommand == "PointVelocity" ) {
        // read by the speed table scan only; when the cell's event runs, there is nothing to do
        return true;
    }

    if (NewCommand == "SetProximityVelocity")
    {
        /*
          if (SetProximityVelocity(NewValue1,NewValue2))
           if (NewLocation)
            vCommandLocation=*NewLocation;
        */
        return true;
    }

    if (NewCommand == "ShuntVelocity")
    { // uruchomienie jazdy manewrowej bądź zmiana prędkości
        if (NewLocation)
            vCommandLocation = *NewLocation;
        // if (OrderList[OrderPos]=Obey_train) and (NewValue1<>0))
        if (false == iEngineActive)
            OrderNext(DriverOrder::prepareEngine); // trzeba odpalić silnik najpierw
        OrderNext(DriverOrder::shunt); // zamieniamy w aktualnej pozycji, albo dodajey za odpaleniem silnika
        if (NewValue1 != 0.0)
        {
            // if (iVehicleCount>=0) WriteLog("Skasowano ilosć wagonów w ShuntVelocity!");
            iVehicleCount = -2; // wartość neutralna
            CheckVehicles(); // zabrać to do OrderCheck()
        }
        // dla prędkości ujemnej przestawić nawrotnik do tyłu? ale -1=brak ograniczenia !!!!
        drivingFlags.press = false; // bez dociskania
        SetVelocity(NewValue1, NewValue2, reason);
        if (NewValue1 != 0.0)
            drivingFlags.stopHere = false; // podjeżanie do semaforów zezwolone
        else
            drivingFlags.stopHere = true; // ma stać w miejscu
        if (fabs(NewValue1) > 2.0) // o ile wartość jest sensowna (-1 nie jest konkretną wartością)
            fShuntVelocity = fabs(NewValue1); // zapamiętanie obowiązującej prędkości dla manewrów

        return true;
    }

    if (NewCommand == "Wait_for_orders")
    { // oczekiwanie; NewValue1 - czas oczekiwania, -1 = na inną komendę
        if (NewValue1 > 0.0 ? NewValue1 > fStopTime : false)
            fStopTime = NewValue1; // Ra: włączenie czekania bez zmiany komendy
        else
            OrderCurrentSet( DriverOrder::waitForOrders ); // czekanie na komendę (albo dać OrderPos=0)

        return true;
    }

    if (NewCommand == "Prepare_engine")
    { // włączenie albo wyłączenie silnika (w szerokim sensie)
        OrdersClear(); // czyszczenie tabelki rozkazów, aby nic dalej nie robił
        if (NewValue1 == 0.0)
            OrderNext(DriverOrder::releaseEngine); // wyłączyć silnik (przygotować pojazd do jazdy)
        else if (NewValue1 > 0.0)
            OrderNext(DriverOrder::prepareEngine); // odpalić silnik (wyłączyć wszystko, co się da)
        // po załączeniu przejdzie do kolejnej komendy, po wyłączeniu na Wait_for_orders
        return true;
    }

    if (NewCommand == "Change_direction")
    {
        DriverOrders o = OrderCurrentGet(); // co robił przed zmianą kierunku
        if (false == iEngineActive)
            OrderNext(DriverOrder::prepareEngine); // trzeba odpalić silnik najpierw
        if (NewValue1 == 0.0)
            iDirectionOrder = -iDirection; // zmiana na przeciwny niż obecny
        else if (NewLocation) // jeśli podane współrzędne eventu/komórki ustawiającej rozkład
        // (trainset nie podaje)
        {
            auto v = *NewLocation - pVehicle->GetPosition(); // wektor do punktu sterującego
            auto d = pVehicle->VectorFront(); // wektor wskazujący przód
            iDirectionOrder = (v.x * d.x + v.z * d.z) * NewValue1 > 0 ?
                                  1 :
                                  -1; // do przodu, gdy iloczyn skalarny i prędkość dodatnie
            // iDirectionOrder=1; else if (NewValue1<0.0) iDirectionOrder=-1;
        }
        if (

            iDirectionOrder != iDirection)
            OrderNext(DriverOrder::changeDirection); // zadanie komendy do wykonania
        if (o.HasAny(movingOrders)) // jeśli jazda manewrowa albo pociągowa
            OrderNext(o); // to samo robić po zmianie
        else if (!o) // jeśli wcześniej było czekanie
            OrderNext(DriverOrder::shunt); // to dalej jazda manewrowa
        if (mvOccupied->Vel >= EU07_AI_MOVEMENT) // jeśli jedzie
            drivingFlags.startHorn = false; // to bez trąbienia po ruszeniu z zatrzymania
        // Change_direction wykona się samo i następnie przejdzie do kolejnej komendy
        return true;
    }

    if (NewCommand == "Obey_train") {
        if( false == iEngineActive ) {
            OrderNext( DriverOrder::prepareEngine ); // trzeba odpalić silnik najpierw
        }
        OrderNext( DriverOrder::obeyTrain );
        // if (NewValue1>0) TrainNumber=floor(NewValue1); //i co potem ???
        OrderCheck(); // jeśli jazda pociągowa teraz, to wykonać niezbędne operacje

        return true;
    }

    if (NewCommand == "Bank") {
        if( false == iEngineActive ) {
            OrderNext( DriverOrder::prepareEngine ); // trzeba odpalić silnik najpierw
        }
        OrderNext( DriverOrder::bank );
        // if (NewValue1>0) TrainNumber=floor(NewValue1); //i co potem ???
        OrderCheck(); // jeśli jazda pociągowa teraz, to wykonać niezbędne operacje

        return true;
    }

    if( NewCommand == "Shunt" || NewCommand == "Loose_shunt" )
    { // NewValue1 - ilość wagonów (-1=wszystkie); NewValue2: 0=odczep, 1..63=dołącz, -1=bez zmian
        //-3,-y - podłączyć do całego stojącego składu (sprzęgiem y>=1), zmienić kierunek i czekać w trybie pociągowym
        //-2,-y - podłączyć do całego stojącego składu (sprzęgiem y>=1), zmienić kierunek i czekać
        //-2, y - podłączyć do całego stojącego składu (sprzęgiem y>=1) i czekać
        //-1,-y - podłączyć do całego stojącego składu (sprzęgiem y>=1) i jechać w powrotną stronę
        //-1, y - podłączyć do całego stojącego składu (sprzęgiem y>=1) i jechać dalej
        //-1, 0 - tryb manewrowy bez zmian (odczepianie z pozostawieniem wagonów nie ma sensu)
        // 0, 0 - odczepienie lokomotywy
        // x,-y - podłączyć się do składu (sprzęgiem y>=1), a następnie odczepić i zabrać (x) wagonów
        // 1, 0 - odczepienie lokomotywy z jednym wagonem
        drivingFlags.stopHere = false; // podjeżanie do semaforów zezwolone
        if( false == iEngineActive ) {
            OrderNext( DriverOrder::prepareEngine ); // trzeba odpalić silnik najpierw
        }
        if (NewValue2 != 0) // jeśli podany jest sprzęg
        {
            iCoupler = CouplingSet::FromBits( static_cast<int>( std::floor( std::abs( NewValue2 ) ) ) ); // jakim sprzęgiem
            OrderNext(DriverOrder::connect); // najpierw połącz pojazdy
            if (NewValue1 >= 0.0) // jeśli ilość wagonów inna niż wszystkie
            { // to po podpięciu należy się odczepić
                iDirectionOrder = -iDirection; // zmiana na ciągnięcie
                OrderPush(DriverOrder::changeDirection); // najpierw zmień kierunek, bo odczepiamy z tyłu
                OrderPush(DriverOrder::disconnect); // a odczep już po zmianie kierunku
                if( NewValue2 > 0.0
                 && NewCommand == "Loose_shunt" ) {
                    // after decoupling continue pushing in the original direction
                    // NOTE: for backward compatibility this option isn't supported for basic shunting mode
                    iDirectionOrder = iDirection; // back to pushing
                    OrderPush( DriverOrder::changeDirection );
                }
            }
            else if (NewValue2 < 0.0) // jeśli wszystkie, a sprzęg ujemny, to tylko zmiana kierunku po podczepieniu
            { // np. Shunt -1 -3
                iDirectionOrder = -iDirection; // jak się podczepi, to jazda w przeciwną stronę
                OrderNext(DriverOrder::changeDirection);
            }
            WaitingTime = 0.0; // nie ma co dalej czekać, można zatrąbić i jechać, chyba że już jedzie
        }
        else { // if (NewValue2==0.0) //zerowy sprzęg
            if( NewValue1 >= 0.0 ) {
                // jeśli ilość wagonów inna niż wszystkie będzie odczepianie,
                // ale jeśli wagony są z przodu, to trzeba najpierw zmienić kierunek
                if( mvOccupied->Couplers[mvOccupied->DirAbsolute > 0 ? end::rear : end::front].Connected == nullptr // z tyłu nic
                 && mvOccupied->Couplers[mvOccupied->DirAbsolute > 0 ? end::front : end::rear].Connected != nullptr ) { // a z przodu skład
                    iDirectionOrder = -iDirection; // zmiana na ciągnięcie
                    OrderNext( DriverOrder::changeDirection ); // najpierw zmień kierunek (zastąpi Disconnect)
                    OrderPush( DriverOrder::disconnect ); // a odczep już po zmianie kierunku
                }
                else if( mvOccupied->Couplers[ mvOccupied->DirAbsolute > 0 ? end::rear : end::front ].Connected != nullptr ) { // z tyłu coś
                    OrderNext( DriverOrder::disconnect ); // jak ciągnie, to tylko odczep (NewValue1) wagonów
                }
                else {
                    // both couplers empty
                    // NOTE: this condition implements legacy behavior (mis)used by some scenarios to keep vehicle idling without moving
                    SetVelocity( 0, 0, stopJoin );
                    drivingFlags.stopHere = true; // nie podjeżdżać do semafora, jeśli droga nie jest wolna
                }
                WaitingTime = 0.0; // nie ma co dalej czekać, można zatrąbić i jechać, chyba że już jedzie
            }
        }
        if (NewValue1 == -1.0) {
            drivingFlags.stopHere = false; // ma jechać
            WaitingTime = 0.0; // nie ma co dalej czekać, można zatrąbić i jechać
        }
        if( NewValue1 < -1.5 ) { // jeśli -2/-3, czyli czekanie z ruszeniem na sygnał
            drivingFlags.stopHere = true; // nie podjeżdżać do semafora, jeśli droga nie jest wolna
        }
        // (nie dotyczy Connect)
        if( NewValue1 < -2.5 ) { // jeśli -3 to potem jazda pociągowa
            OrderNext( NewCommand == "Shunt" ? DriverOrder::obeyTrain : DriverOrder::bank );
        }
        else { // otherwise continue shunting
            OrderNext( NewCommand == "Shunt" ? DriverOrder::shunt : DriverOrder::looseShunt );
        }
        CheckVehicles(); // sprawdzić światła
        iVehicleCount = std::floor( NewValue1 );

        return true;
    }

    if( NewCommand == "Jump_to_first_order" ) {
        JumpToFirstOrder();
        return true;
    }

    if (NewCommand == "Jump_to_order")
    {
        if( NewValue1 == -1.0 ) {
            JumpToNextOrder();
        }
        else if (NewValue1 >= 0 && NewValue1 < maxorders)
        {
            OrderPos = floor(NewValue1);
            if( !OrderPos ) {
                // zgodność wstecz: dopiero pierwsza uruchamia
                OrderPos = 1;
            }
#if LOGORDERS
            OrdersDump("Jump_to_order");
#endif
        }
        return true;
    }

    if (NewCommand == "Warning_signal")
    {
        if( NewValue1 > 0 && NewValue2 > 0 ) {
            fWarningDuration = NewValue1; // czas trąbienia
            cue_action( driver_hint::hornon, NewValue2 ); // horn combination flag
            simulation::Blackbox.horn_requested(
                pVehicles[ end::front ], NewValue1, NewValue2, NewLocation != nullptr ? *NewLocation : pVehicles[ end::front ]->GetPosition() );
        }
        return true;
    }

    if (NewCommand == "Radio_channel") {
        // wybór kanału radiowego (którego powinien używać AI, ręczny maszynista musi go ustawić sam)
        if (NewValue1 >= 0) {
            // wartości ujemne są zarezerwowane, -1 = nie zmieniać kanału
            if( AIControllFlag ) {
                iRadioChannel = NewValue1;
            }
            if( iGuardRadio ) {
                // kierownikowi też zmienić
                iGuardRadio = NewValue1;
            }
        }
        // NewValue2 może zawierać dodatkowo oczekiwany kod odpowiedzi, np. dla W29 "nawiązać
        // łączność radiową z dyżurnym ruchu odcinkowym"
        return true;
    }

    if( NewCommand == "SetLights" ) {
        // set consist lights pattern hints
        m_lighthints[ end::front ] = static_cast<int>( NewValue1 );
        m_lighthints[ end::rear ] = static_cast<int>( NewValue2 );
        if( true == OrderCurrentGet().HasAll(DriverOrder::obeyTrain) ) {
            // light hints only apply in the obey_train mode
            CheckVehicles();
        }
        return true;
    }

	if (NewCommand == "SetSignal") {
		auto const signal{std::lrint(NewValue1)};

		drivingFlags.signalPc6 = false;
		drivingFlags.signalA1 = false;

		if (NewValue2 > 0.5)
		{
			// another number set the bit of a driving flag of that number before
			drivingFlags.signalPc6 = signal == Signal_Pc6;
			drivingFlags.signalA1 = signal == Signal_A1;
		}
		else
			CheckVehicles(); // restore light state
	}

    return false; // nierozpoznana - wysłać bezpośrednio do pojazdu
};

void TController::JumpToNextOrder( bool const Ignoremergedchangedirection )
{ // wykonanie kolejnej komendy z tablicy rozkazów
    if (auto const currentorder { OrderCurrentGet() }; currentorder != DriverOrder::waitForOrders)
    {
        if( currentorder.HasAny(DriverOrder::changeDirection) // jeśli zmiana kierunku
		    && currentorder != DriverOrder::changeDirection && false == Ignoremergedchangedirection ) { // ale nałożona na coś

			// usunięcie zmiany kierunku z innej komendy
			OrderCurrentSet( currentorder.Without(DriverOrder::changeDirection) );
			OrderCheck();
			return;
		}
        if (OrderPos < maxorders - 1)
            ++OrderPos;
        else
            OrderPos = 0;
    }
    OrderCheck();
#if LOGORDERS
    OrdersDump("JumpToNextOrder"); // normalnie nie ma po co tego wypisywać
#endif
};

void TController::JumpToFirstOrder()
{ // taki relikt
    OrderPos = 1;
    if (OrderTop == 0)
        OrderTop = 1;
    OrderCheck();
#if LOGORDERS
    OrdersDump("JumpToFirstOrder"); // normalnie nie ma po co tego wypisywać
#endif
};

void TController::OrderCheck()
{ // reakcja na zmianę rozkazu

    if( OrderCurrentGet() != DriverOrder::obeyTrain ) {
        // reset light hints
        m_lighthints[ end::front ] = m_lighthints[ end::rear ] = -1;
    }
    if( OrderCurrentGet() & ( DriverOrder::shunt | DriverOrder::looseShunt | DriverOrder::connect | DriverOrder::obeyTrain | DriverOrder::bank ) ) {
        CheckVehicles(); // sprawdzić światła
    }
    if( OrderCurrentGet() & ( DriverOrder::shunt | DriverOrder::looseShunt | DriverOrder::connect ) ) {
        // HACK: ensure consist doors will be closed on departure
        drivingFlags.doorOpened = true;
    }
    auto const currentorder { OrderCurrentGet() };
    if (currentorder & DriverOrder::changeDirection) // może być nałożona na inną i wtedy ma priorytet
        iDirectionOrder = -iDirection; // trzeba zmienić jawnie, bo się nie domyśli
    else if (currentorder == DriverOrder::obeyTrain)
        drivingFlags.stopPoint = true; // W4 są widziane
    else if (currentorder == DriverOrder::disconnect)
        iVehicleCount = iVehicleCount < 0 ? 0 : iVehicleCount; // odczepianie lokomotywy
    else if (currentorder == DriverOrder::connect)
        drivingFlags.stopPoint = false; // podczas jazdy na połączenie nie zwracać uwagi na W4
    else if (currentorder == DriverOrder::waitForOrders)
        OrdersClear(); // czyszczenie rozkazów i przeskok do zerowej pozycji
}

void TController::OrderNext(DriverOrders NewOrder)
{ // ustawienie rozkazu do wykonania jako następny
    if (OrderCurrentGet() == NewOrder)
        return; // jeśli robi to, co trzeba, to koniec
    if (!OrderPos)
        OrderPos = 1; // na pozycji zerowej pozostaje czekanie
    OrderTop = OrderPos; // ale może jest czymś zajęty na razie
    if (NewOrder.HasAny(movingOrders)) // jeśli ma jechać
    { // ale może być zajęty chwilowymi operacjami
        while (OrderTop >= 0 && OrderTop < maxorders - 1
            && OrderList[OrderTop] != DriverOrder::waitForOrders && false == OrderList[OrderTop].HasAny(movingOrders)) // jeśli coś robi
            ++OrderTop; // pomijamy wszystkie tymczasowe prace
    }
    else
    { // jeśli ma ustawioną jazdę, to wyłączamy na rzecz operacji
        while (OrderTop >= 0 && OrderTop < maxorders - 1
            && OrderList[OrderTop] != DriverOrder::waitForOrders && false == OrderList[OrderTop].HasAny(movingOrders) && OrderList[OrderTop] != NewOrder) // jeśli coś robi
            ++OrderTop; // pomijamy wszystkie tymczasowe prace
    }
    if (OrderTop < 0 || OrderTop >= maxorders)
    {
        ErrorLog("Commands overflow: order \"" + Order2Str(NewOrder) + "\" dropped");
        return;
    }
    OrderList[OrderTop++] = NewOrder; // dodanie rozkazu jako następnego
#if LOGORDERS
    OrdersDump( "OrderNext" ); // normalnie nie ma po co tego wypisywać
#endif
}

void TController::OrderPush(DriverOrders NewOrder)
{ // zapisanie na stosie kolejnego rozkazu do wykonania
    if (OrderPos == OrderTop && false == OrderCurrentGet().HasAny(movingOrders)) // jeśli miałby być zapis na aktalnej pozycji
	                                                         // ale nie jedzie
		++OrderTop;
	// niektóre operacje muszą zostać najpierw dokończone => zapis na kolejnej
    if (OrderTop < 0 || OrderTop >= maxorders)
    {
        ErrorLog("Commands overflow: order \"" + Order2Str(NewOrder) + "\" dropped");
        return;
    }
    if (OrderList[OrderTop] != NewOrder) // jeśli jest to samo, to nie dodajemy
        OrderList[OrderTop++] = NewOrder; // dodanie rozkazu na stos
#if LOGORDERS
    OrdersDump( "OrderPush: [" + Order2Str( NewOrder ) + "]" ); // normalnie nie ma po co tego wypisywać
#endif
}

void TController::OrdersDump( std::string const Neworder, bool const Verbose )
{ // wypisanie kolejnych rozkazów do logu
    WriteLog( "New order for [" + pVehicle->asName + "]: " + Neworder );
    if( false == Verbose ) { return; }
    for (int b = 0; b < maxorders; ++b)
    {
        WriteLog( ( OrderPos == b ? ">" : " " ) + (std::to_string(b) + ": " + Order2Str(OrderList[b])));
        if (b && OrderList[b] == DriverOrder::waitForOrders) // z wyjątkiem pierwszej pozycji
		                                          // jeśli końcowa komenda
			break;
		// dalej nie trzeba
    }
};

void TController::OrdersInit(double fVel)
{ // wypełnianie tabelki rozkazów na podstawie rozkładu
    // ustawienie kolejności komend, niezależnie kto prowadzi
    OrdersClear(); // usunięcie poprzedniej tabeli
    OrderPush(DriverOrder::prepareEngine); // najpierw odpalenie silnika
    if (TrainParams.StationCount == 0)
    { // brak rozkładu to jazda manewrowa
//        if (fVel > 0.05) // typowo 0.1 oznacza gotowość do jazdy, 0.01 tylko załączenie silnika
            OrderPush(DriverOrder::shunt); // jeśli nie ma rozkładu, to manewruje
    }
    else
    { // jeśli z rozkładem, to jedzie na szlak
        if (fVel > 0.0 && fVel < 0.02)
            OrderPush(DriverOrder::shunt); // dla prędkości 0.01 włączamy jazdę manewrową
        else if (TrainParams.DirectionChange() ? //  jeśli obrót na pierwszym przystanku
                    (drivingFlags.pushPull ? // SZT również! SN61 zależnie od wagonów...
                        TrainParams.TimeTable[1].StationName == TrainParams.Relation1 :
                        false) :
                    false)
            OrderPush(DriverOrder::shunt); // a teraz start będzie w manewrowym, a tryb pociągowy włączy W4
        else
            // jeśli start z pierwszej stacji i jednocześnie jest na niej zmiana kierunku, to EZT ma mieć Shunt
            OrderPush(DriverOrder::obeyTrain); // dla starych scenerii start w trybie pociagowym
        if (DebugModeFlag) // normalnie nie ma po co tego wypisywać
            WriteLog("/* Timetable: " + TrainParams.ShowRelation());
        TMTableLine *t;
        for (int i = 0; i <= TrainParams.StationCount; ++i)
        {
            t = TrainParams.TimeTable + i;
            if (DebugModeFlag) {
                // normalnie nie ma po co tego wypisywa?
                WriteLog(
                    t->StationName
                    + " " + std::to_string(t->Ah) + ":" + std::to_string(t->Am)
                    + ", " + std::to_string(t->Dh) + ":" + std::to_string(t->Dm)
                    + " " + t->StationWare);
            }
            if ( contains( t->StationWare, '@' ) )
            { // zmiana kierunku i dalsza jazda wg rozk?adu
                if (drivingFlags.pushPull) // SZT również! SN61 zależnie od wagonów...
                { // jeśli skład zespolony, wystarczy zmienić kierunek jazdy
                    OrderPush(DriverOrder::changeDirection); // zmiana kierunku
                }
                else
                { // dla zwykłego składu wagonowego odczepiamy lokomotywę
                    OrderPush(DriverOrder::disconnect); // odczepienie lokomotywy
                    OrderPush(DriverOrder::shunt); // a dalej manewry
                }
                if (i < TrainParams.StationCount) // jak nie ostatnia stacja
                    OrderPush(DriverOrder::obeyTrain); // to dalej wg rozkładu
            }
        }
        if( DebugModeFlag ) {
            // normalnie nie ma po co tego wypisywać
            WriteLog( "*/" );
        }
        OrderPush(DriverOrder::shunt); // po wykonaniu rozkładu przełączy się na manewry
    }
    // McZapkie-100302 - to ma byc wyzwalane ze scenerii
    if( fVel == 0.0 ) {
        // jeśli nie ma prędkości początkowej, to śpi
        SetVelocity( 0, 0, stopSleep );
    }
    else {
        // jeśli podana niezerowa prędkość
        if( fVel >= 1.0
         || fVel < 0.02 ) {
            // jeśli ma jechać - dla 0.01 ma podjechać manewrowo po podaniu sygnału
            // to do następnego W4 ma podjechać blisko
            drivingFlags.stopHere = false;
            drivingFlags.stopCloser = true;
        }
        else {
            // czekać na sygnał
            drivingFlags.stopHere = true;
        }
        JumpToFirstOrder();
        if (fVel >= 1.0) // jeśli ma jechać
            SetVelocity(fVel, -1); // ma ustawić żądaną prędkość
        else
            SetVelocity(0, 0, stopSleep); // prędkość w przedziale (0;1) oznacza, że ma stać
    }
#if LOGORDERS
    OrdersDump( "OrdersInit", DebugModeFlag ); // wypisanie kontrolne tabelki rozkazów
#endif
    // McZapkie! - zeby w ogole AI ruszyl to musi wykonac powyzsze rozkazy
    // Ale mozna by je zapodac ze scenerii
};

std::string TController::StopReasonText() const
{ // informacja tekstowa o przyczynie zatrzymania
    if (eStopReason != 7) // zawalidroga będzie inaczej
        return StopReasonTable[eStopReason];
    else
        return "Blocked by " + pVehicles[0]->PrevAny()->name();
};

//----------------------------------------------------------------------------------------------------------------------
// McZapkie: skanowanie semaforów
// Ra: stare funkcje skanujące, używane podczas manewrów do szukania sygnalizatora z tyłu
//- nie reagują na PutValues, bo nie ma takiej potrzeby
//- rozpoznają tylko zerową prędkość (jako koniec toru i brak podstaw do dalszego skanowania)
//----------------------------------------------------------------------------------------------------------------------

bool TController::IsOccupiedByAnotherConsist( TTrack *Track, double const Distance )
{ // najpierw sprawdzamy, czy na danym torze są pojazdy z innego składu
    if( false == Track->Dynamics.empty() ) {
        for( auto dynamic : Track->Dynamics ) {
            if( dynamic->ctOwner != this ) {
                // jeśli jest jakiś cudzy to tor jest zajęty i skanowanie nie obowiązuje
                if( Distance == 0 ) {
                    return true;
                }
                else {
                    // based on provided position of scanning vehicle and scan direction, filter out vehicles on irrelevant end
                    auto const scandirection { Distance > 0 ? 1 : -1 }; // positive value means scan towards point2 end of the track
                    auto const obstaclelocation { scandirection > 0 ? Track->Length() - dynamic->RaTranslationGet() : dynamic->RaTranslationGet() };
                    // if detected vehicle is closer to the end of the track in scanned direction than we are, it means it's blocking our way
                    if( obstaclelocation < std::abs( Distance ) ) {
                        return true;
                    }
                }
            }
        }
    }
    return false; // wolny
};

void
TController::UpdateChangeDirection() {
    // TODO: rework into driver mode independent routine
    if( false == OrderCurrentGet().HasAll(DriverOrder::changeDirection) ) { return; }

    if( true == AIControllFlag && mvOccupied->Vel < EU07_AI_NOMOVEMENT ) {
        // sprobuj zmienic kierunek (może być zmieszane z jeszcze jakąś komendą)
		// jeśli się zatrzymał, to zmieniamy kierunek jazdy, a nawet kabinę/człon
		Activation(); // ustawienie zadanego wcześniej kierunku i ewentualne przemieszczenie AI
		// Change_direction (tylko dla AI)
    }
    // shared part of the routine, implement if direction matches what was requested
    if( mvOccupied->Vel < EU07_AI_NOMOVEMENT
     && iDirection == iDirectionOrder ) {
        // NOTE: we can't be sure there's a visible signal within scan range after direction change
        // which would normally overwrite the old limit, so we reset signal value manually here
        VelSignal = -1;
        VelSignalNext = -1;
        PrepareEngine();
        JumpToNextOrder(); // następnie robimy, co jest do zrobienia (Shunt albo Obey_train)
        if( OrderCurrentGet() & (DriverOrder::shunt | DriverOrder::looseShunt | DriverOrder::connect) && false == drivingFlags.stopHere ) {
            // jeśli dalej mamy manewry
			// o ile nie ma stać w miejscu,
			// jechać od razu w przeciwną stronę i nie trąbić z tego tytułu:
			drivingFlags.startHorn = false;
			SetVelocity(fShuntVelocity, fShuntVelocity);
		}
    }
}

void
TController::UpdateLooseShunt() {
    // when loose shunting try to detect situations where engaged brakes in a consist to be pushed prevent movement
    // TODO: run also for potential settings-based virtual assistant
    auto const autobrakerelease { ( true == AIControllFlag ) };

    if( autobrakerelease
     && Obstacle.distance < 1.0
     && AccDesired > 0.1
     && mvOccupied->Vel < 1.0 ) {

        auto *vehicle { Obstacle.vehicle };
        auto const direction { ( vehicle->Prev() != nullptr ? end::front : end::rear ) };
        while( vehicle != nullptr ) {
            if( vehicle->MoverParameters->BrakePress > 0.2 ) {
                vehicle->MoverParameters->BrakeLevelSet( 0 ); // hamulec na zero, aby nie hamował
                vehicle->MoverParameters->BrakeReleaser( 1 ); // wyluzuj pojazd, aby dało się dopychać
            }
            // NOTE: we trust the consist to be arranged in a valid chain
            // TBD, TODO: traversal direction validation?
            vehicle = direction == end::front ? vehicle->Prev() : vehicle->Next();
        }
    }
}

void
TController::UpdateObeyTrain() {

    UpdateNextStop();

    if( drivingFlags.guardSignal
     && VelDesired > 0.0 ) {
        // komunikat od kierownika tu, bo musi być wolna droga i odczekany czas stania
        drivingFlags.guardSignal = false; // tylko raz nadać
		drivingFlags.guardOpenDoor = false; // nie trzeba już otwierać drzwi
        if( false == tsGuardSignal.empty() ) {
            tsGuardSignal.stop();
            // w zasadzie to powinien mieć flagę, czy jest dźwiękiem radiowym, czy bezpośrednim
            // albo trzeba zrobić dwa dźwięki, jeden bezpośredni, słyszalny w
            // pobliżu, a drugi radiowy, słyszalny w innych lokomotywach
            // na razie zakładam, że to nie jest dźwięk radiowy, bo trzeba by zrobić
            // obsługę kanałów radiowych itd.
            if( iGuardRadio == 0 ) {
                // jeśli nie przez radio
                tsGuardSignal.owner( pVehicle );
                // place virtual conductor some distance away
                tsGuardSignal.offset( { pVehicle->MoverParameters->Dim.W * -0.75f, 1.7f, std::min( -20.0, -0.2 * fLength ) } );
                tsGuardSignal.play( sound_flags::exclusive );
                // NOTE: we can't rely on is_playing() check as sound playback is based on distance from local camera
                fActionTime = -5.0; // niech trochę potrzyma
            }
            else {
                // radio message
                tsGuardSignal.owner( pVehicle );
/*
                tsGuardSignal.offset( { 0.f, 2.f, pVehicle->MoverParameters->Dim.L * 0.4f * ( pVehicle->MoverParameters->CabOccupied < 0 ? -1 : 1 ) } );
                tsGuardSignal.play( sound_flags::exclusive );
*/
                simulation::radio_message( &tsGuardSignal, iGuardRadio );
                // NOTE: we can't rely on is_playing() check as sound playback is based on distance from local camera
                fActionTime = -5.0; // niech trochę potrzyma
            }
        }
    }
}

void
TController::UpdateConnect() {
    // podłączanie do składu
    if (drivingFlags.connect) {
        // sprzęgi sprawdzamy w pierwszej kolejności, bo jak połączony, to koniec
        auto *vehicle { iCouplingVehicle.value().first };
        auto const couplingend { iCouplingVehicle.value().second };
        auto *vehicleparameters { vehicle->MoverParameters };
        auto const &neighbour { vehicleparameters->Neighbours[ couplingend ] };
        // vehicles which can't share their control get coupled without the control connection (eu07/maszyna#520)
        auto expectedcoupling { iCoupler };
        if( neighbour.vehicle != nullptr ) {
            auto const neighbourend { neighbour.vehicle_end != 2 ? neighbour.vehicle_end : vehicleparameters->Couplers[ couplingend ].ConnectedNr };
            expectedcoupling = vehicleparameters->CompatibleCoupling( couplingend, *neighbour.vehicle->MoverParameters, neighbourend, iCoupler );
        }
        if( vehicleparameters->Couplers[ couplingend ].CouplingFlag != expectedcoupling ) {
            if( neighbour.vehicle != nullptr && neighbour.distance < 10 ) {
				// check whether we need to attach coupler adapter
				auto coupleriscompatible{true};
				auto const &coupler{vehicleparameters->Couplers[couplingend]};
				if (coupler.type() != TCouplerType::Automatic)
				{
					auto const &othercoupler = neighbour.vehicle->MoverParameters->Couplers[(neighbour.vehicle_end != 2 ? neighbour.vehicle_end : coupler.ConnectedNr)];
					if (othercoupler.type() == TCouplerType::Automatic)
					{
						coupleriscompatible = false;
						cue_action(driver_hint::couplingadapterattach, couplingend);
					}
				}
				// próba podczepienia
				if ((AIControllFlag || Global.AITrainman) && coupleriscompatible && neighbour.distance < 2)
				{
					vehicleparameters->Attach(couplingend, neighbour.vehicle_end, neighbour.vehicle->MoverParameters, iCoupler);
				}
			}
        }
        // NOTE: no else as the preceeding block can potentially establish expected connection
        if( vehicleparameters->Couplers[ couplingend ].CouplingFlag == expectedcoupling ) {
            // jeżeli został podłączony
            CheckVehicles( DriverOrder::connect ); // sprawdzić światła nowego składu

            iCoupler = {}; // dalsza jazda manewrowa już bez łączenia
            iCouplingVehicle.reset();
            drivingFlags.connect = false; // zdjęcie flagi doczepiania
            JumpToNextOrder( true ); // wykonanie następnej komendy
        }
    } // moveConnect
    else {
        // początek podczepiania, z wyłączeniem sprawdzania fTrackBlock
        if( Obstacle.distance <= 20.0 && !iCouplingVehicle ) {
			// write down which vehicle should be coupled with the target consist,
			// so we don't lose track of it if the user does something unexpected
			iCouplingVehicle = {pVehicles[end::front], (pVehicles[end::front]->DirectionGet() > 0 ? end::front : end::rear)};
			drivingFlags.connect = true;
		}
    }
}

int
TController::unit_count( int const Threshold ) const {

    auto *vehicle { pVehicle };
    auto unitcount { 1 };
    do {
        auto const decoupledend{ ( vehicle->DirectionGet() > 0 ? // numer sprzęgu od strony czoła składu
            end::rear :
            end::front ) };
        auto const coupling { vehicle->MoverParameters->Couplers[ decoupledend ].CouplingFlag };
        if( coupling == Coupling::none ) {
            break;
        }
        // jeżeli sprzęg zablokowany to liczymy człony jako jeden
        if( false == coupling.HasAny( Coupling::permanent ) ) {
            ++unitcount;
        }
        vehicle = vehicle->Next();
    } while( unitcount < Threshold );

    return unitcount;
}

void
TController::UpdateDisconnect() {

    if( iVehicleCount >= 0 ) {
        // early test for human drivers, who might disregard the proper procedure, or perform it before they receive the order to
        if( false == AIControllFlag && unit_count(iVehicleCount + 2) <= iVehicleCount + 1 ) {
            // iVehicleCount = 0 means the consist should be reduced to (1) leading unit, thus we increase our test values by 1
			iVehicleCount = -2; // odczepiono, co było do odczepienia
			return; // we'll wrap up the procedure on the next update beat
		}
        // regular uncoupling procedure, performed by ai and naively expected from the human drivers
        // 3rd stage: change direction, compress buffers and uncouple
        if( iDirection != iDirectionOrder ) {
            cue_action( driver_hint::mastercontrollersetreverserunlock );
            cue_action( driver_hint::directionother );
        }
        if( drivingFlags.press && iDirection == iDirectionOrder ) {
            if( iVehicleCount >= 0 ) {
                // zmieni się po odczepieniu
                WriteLog( "Uncoupling [" + mvOccupied->Name + "]: actuating and compressing buffers..." );
                cue_action( driver_hint::independentbrakerelease );
                cue_action( driver_hint::bufferscompress );
            }

            WriteLog( "Uncoupling [" + mvOccupied->Name + "]: from " + ( mvOccupied->DirAbsolute > 0 ? "front" : "rear" ) );
            auto *decoupledvehicle { pVehicle }; // pojazd do odczepienia, w (pVehicle) siedzi AI
            int decoupledend; // numer sprzęgu, który sprawdzamy albo odczepiamy
            auto vehiclecount { iVehicleCount }; // ile wagonów ma zostać
            // szukanie pojazdu do odczepienia
            do {
                decoupledend = decoupledvehicle->DirectionGet() > 0 ? // numer sprzęgu od strony czoła składu
                    end::front :
                    end::rear;
                // jeżeli sprzęg zablokowany to liczymy człony jako jeden
                if( decoupledvehicle->MoverParameters->Couplers[ decoupledend ].CouplingFlag & Coupling::permanent ) {
                    ++vehiclecount;
                }
                if( decoupledvehicle != pVehicle ) {
                    decoupledvehicle->MoverParameters->BrakeReleaser( 1 ); // wyluzuj pojazd, aby dało się dopychać
                }
                if( vehiclecount ) { // jeśli jeszcze nie koniec
                    decoupledvehicle = decoupledvehicle->Prev(); // kolejny w stronę czoła składu (licząc od tyłu), bo dociskamy
                    if( decoupledvehicle == nullptr ) {
                        vehiclecount = 0; // nie ma co dalej sprawdzać, odczepianie zakończone
                    }
                }
            } while( vehiclecount-- );
            if( decoupledvehicle == nullptr ) {
                // no target, or already just virtual coupling
                WriteLog( "Uncoupling [" + mvOccupied->Name + "]: didn't find anything to disconnect" );
                iVehicleCount = -2; // odczepiono, co było do odczepienia
            }
            else {
                if( AIControllFlag || Global.AITrainman ) {
                    decoupledvehicle->Dettach( decoupledend );
                }
                // tylko jeśli odepnie
                if( decoupledvehicle->MoverParameters->Couplers[ decoupledend ].CouplingFlag == Coupling::none ) {
                    WriteLog( "Uncoupling [" + mvOccupied->Name + "]: uncoupled" );
                    iVehicleCount = -2;
                    // update trainset state
                    CheckVehicles( DriverOrder::disconnect );
                    // potentially remove coupler adapter
/*
                    if( decoupledvehicle->MoverParameters->Couplers[ decoupledend ].has_adapter() ) {
                        decoupledvehicle->remove_coupler_adapter( decoupledend );
                    }
*/
                    if( pVehicles[ end::front ]->MoverParameters->Couplers[ decoupledend ].has_adapter() ) {
                        cue_action( driver_hint::couplingadapterremove, decoupledend );
                    }
                }
            } // a jak nie, to dociskać dalej
        }
        // 2nd stage: apply consist brakes and change direction
        if( ( false == drivingFlags.press ) ) {
            // store initial consist direction
            if( !iDirectionBackup ) {
                iDirectionBackup = iDirection;
            }
            if( false == IsConsistBraked ) {
                WriteLog( "Uncoupling [" + mvOccupied->Name + "]: applying consist brakes..." );
                cue_action( driver_hint::trainbrakeapply );
            }
            // jeśli w miarę został zahamowany (ciśnienie mniejsze niż podane na pozycji 3, zwyle 0.37)
            else {
                WriteLog( "Uncoupling [" + mvOccupied->Name + "]: direction change" );
/* // TODO: test if this block is needed
                if( BrakeSystem == TBrakeSystem::ElectroPneumatic ) {
                    // wyłączenie EP, gdy wystarczy (może nie być potrzebne, bo na początku jest)
                    mvOccupied->BrakeLevelSet( 0 );
                }
*/
                iDirectionOrder = -iDirection; // zmiana kierunku jazdy na przeciwny (dociskanie)
                drivingFlags.press = true; // następnie będzie dociskanie
            }
        }
    } // odczepianie
    if( iVehicleCount < 0 ) {
        // 4th stage: restore initial direction
        if( iDirectionBackup ) {
            iDirectionOrder = iDirectionBackup.value();
            iDirectionBackup.reset();
        }
        if( iDirection != iDirectionOrder ) {
            WriteLog( "Uncoupling [" + mvOccupied->Name + "]: second direction change" );
            cue_action( driver_hint::mastercontrollersetreverserunlock );
            cue_action( driver_hint::directionother ); // zmiana kierunku jazdy na właściwy
        }
        // 5th stage: clean up and move on to next order
        if( iDirection == iDirectionOrder ) {
            drivingFlags.press = false; // koniec dociskania
            JumpToNextOrder( true ); // zmieni światła
            TableClear(); // skanowanie od nowa
            drivingFlags.startHorn = false; // bez trąbienia przed ruszeniem
        }
    }
}

void
TController::handle_engine() {
    // HACK: activate route scanning if an idling vehicle is activated by a human user
    if( OrderCurrentGet() == DriverOrder::waitForOrders
     && false == iEngineActive
//     && ( false == AIControllFlag )
     && true == mvOccupied->Power24vIsAvailable ) {
        OrderNext( DriverOrder::prepareEngine );
    }
    // basic engine preparation
    if( OrderCurrentGet() == DriverOrder::prepareEngine && PrepareEngine() ) {
		// gotowy do drogi?
		JumpToNextOrder();
	}
    // engine state can potentially deteriorate in one of usual driving modes
    if( OrderCurrentGet() & (DriverOrder::changeDirection | DriverOrder::connect | DriverOrder::disconnect | DriverOrder::shunt | DriverOrder::looseShunt | DriverOrder::obeyTrain | DriverOrder::bank)
     && false == iEngineActive ) {
        // jeśli coś ma robić to niech odpala do skutku
        PrepareEngine();
    }
}

void
TController::handle_orders() {

    switch (SingleOrder(OrderCurrentGet()))
    {
    case DriverOrder::releaseEngine: {
        if( ReleaseEngine() ) { // zdana maszyna?
            JumpToNextOrder();
        }
        break;
    }
    case DriverOrder::obeyTrain: {
        UpdateObeyTrain();
        break;
    }
    case DriverOrder::looseShunt: {
        UpdateLooseShunt();
        break;
    }
    case DriverOrder::connect: {
        UpdateConnect();
        break;
    }
    case DriverOrder::disconnect: {
        UpdateDisconnect();
        break;
    }
    case DriverOrder::jumpToFirstOrder: {
        if( OrderPos > 1 ) {
            OrderPos = 1; // w zerowym zawsze jest czekanie
        }
        else {
            ++OrderPos;
        }
#if LOGORDERS
        OrdersDump("Jump_to_first_order");
#endif
        break;
    }
    default: {
        // special case, change_direction potentially received during partially completed coupling operation
        // this is unlikely to happen for the AI, but can happen for a (slower) human user
        if( drivingFlags.connect ) {
            UpdateConnect();
        }
        break;
    }
    } // switch OrderList[OrderPos]
    UpdateChangeDirection();
}
