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

// a vehicle among the others of its train: the neighbours, the ends, the vehicles found along the couplings

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



//---------------------------------------------------------------------------
TDynamicObject * TDynamicObject::FirstFind(int &coupler_nr, CouplingSet const cf)
{ // szukanie skrajnego połączonego pojazdu w pociagu
    // od strony sprzegu (coupler_nr) obiektu (start)
    TDynamicObject *temp = this;
    for (int i = 0; i < 300; i++) // ograniczenie do 300 na wypadek zapętlenia składu
    {
        if (!temp)
            return nullptr; // Ra: zabezpieczenie przed ewentaulnymi błędami sprzęgów
        if (false == temp->MoverParameters->Couplers[coupler_nr].CouplingFlag.HasAll(cf))
            return temp; // nic nie ma już dalej podłączone sprzęgiem cf
        if (coupler_nr == end::front)
        { // jeżeli szukamy od sprzęgu 0
            if (temp->PrevConnected()) // jeśli mamy coś z przodu
            {
                if (temp->PrevConnectedNo() == end::front) // jeśli pojazd od strony sprzęgu 0 jest odwrócony
                    coupler_nr = 1 - coupler_nr; // to zmieniamy kierunek sprzęgu
                temp = temp->PrevConnected(); // ten jest od strony 0
            }
            else
                return temp; // jeśli jednak z przodu nic nie ma
        }
        else
        {
            if (temp->NextConnected())
            {
                if (temp->NextConnectedNo() == end::rear) // jeśli pojazd od strony sprzęgu 1 jest odwrócony
                    coupler_nr = 1 - coupler_nr; // to zmieniamy kierunek sprzęgu
                temp = temp->NextConnected(); // ten pojazd jest od strony 1
            }
            else
                return temp; // jeśli jednak z tyłu nic nie ma
        }
    }
    return nullptr; // to tylko po wyczerpaniu pętli
};

//---------------------------------------------------------------------------
float TDynamicObject::GetEPP()
{ // szukanie skrajnego połączonego pojazdu w pociagu
    // od strony sprzegu (coupler_nr) obiektu (start)
    TDynamicObject *temp = this;
    int coupler_nr = 0;
    double eq = 0.0;
    double am = 0.0;

    for (int i = 0; i < 300; ++i) // ograniczenie do 300 na wypadek zapętlenia składu
    {
        if (!temp)
            break; // Ra: zabezpieczenie przed ewentaulnymi błędami sprzęgów
        eq += temp->MoverParameters->PipePress * temp->MoverParameters->Dim.L;
        am += temp->MoverParameters->Dim.L;
        if ((temp->MoverParameters->Couplers[coupler_nr].CouplingFlag & Coupling::brakeHose) != Coupling::brakeHose)
            break; // nic nie ma już dalej podłączone
        if (coupler_nr == 0)
        { // jeżeli szukamy od sprzęgu 0
            if (temp->PrevConnected()) // jeśli mamy coś z przodu
            {
                if (temp->PrevConnectedNo() == end::front) // jeśli pojazd od strony sprzęgu 0 jest odwrócony
                    coupler_nr = 1 - coupler_nr; // to zmieniamy kierunek sprzęgu
                temp = temp->PrevConnected(); // ten jest od strony 0
            }
            else
                break; // jeśli jednak z przodu nic nie ma
        }
        else
        {
            if (temp->NextConnected())
            {
                if (temp->NextConnectedNo() == end::rear) // jeśli pojazd od strony sprzęgu 1 jest odwrócony
                    coupler_nr = 1 - coupler_nr; // to zmieniamy kierunek sprzęgu
                temp = temp->NextConnected(); // ten pojazd jest od strony 1
            }
            else
                break; // jeśli jednak z tyłu nic nie ma
        }
    }

    temp = this;
    coupler_nr = 1;
    for (int i = 0; i < 300; i++) // ograniczenie do 300 na wypadek zapętlenia składu
    {
        if (!temp)
            break; // Ra: zabezpieczenie przed ewentaulnymi błędami sprzęgów
        eq += temp->MoverParameters->PipePress * temp->MoverParameters->Dim.L;
        am += temp->MoverParameters->Dim.L;
        if ((temp->MoverParameters->Couplers[coupler_nr].CouplingFlag & Coupling::brakeHose) != Coupling::brakeHose)
            break; // nic nie ma już dalej podłączone
        if (coupler_nr == 0)
        { // jeżeli szukamy od sprzęgu 0
            if (temp->PrevConnected()) // jeśli mamy coś z przodu
            {
                if (temp->PrevConnectedNo() == end::front) // jeśli pojazd od strony sprzęgu 0 jest odwrócony
                    coupler_nr = 1 - coupler_nr; // to zmieniamy kierunek sprzęgu
                temp = temp->PrevConnected(); // ten jest od strony 0
            }
            else
                break; // jeśli jednak z przodu nic nie ma
        }
        else
        {
            if (temp->NextConnected())
            {
                if (temp->NextConnectedNo() == end::rear) // jeśli pojazd od strony sprzęgu 1 jest odwrócony
                    coupler_nr = 1 - coupler_nr; // to zmieniamy kierunek sprzęgu
                temp = temp->NextConnected(); // ten pojazd jest od strony 1
            }
            else
                break; // jeśli jednak z tyłu nic nie ma
        }
    }
    eq -= MoverParameters->PipePress * MoverParameters->Dim.L;
    am -= MoverParameters->Dim.L;
    return eq / am;
};

//---------------------------------------------------------------------------
TDynamicObject * TDynamicObject::GetFirstDynamic(int cpl_type, CouplingSet const cf)
{ // Szukanie skrajnego połączonego pojazdu w pociagu
    // od strony sprzegu (cpl_type) obiektu szukajacego
    // Ra: wystarczy jedna funkcja do szukania w obu kierunkach
    return FirstFind(cpl_type, cf); // używa referencji
};

int TDynamicObject::DirectionSet(int d)
{ // ustawienie kierunku w składzie (wykonuje AI)
    auto const lastdirection { iDirection };
    iDirection = d > 0 ? 1 : 0; // d:1=zgodny,-1=przeciwny; iDirection:1=zgodny,0=przeciwny;

    if( iDirection != lastdirection ) {
        // direction was flipped, switch recorded servicable platform sides for potentially ongoing load exchange
        auto const left { lastdirection > 0 ? 1 : 2 };
        auto const right { 3 - left };
        m_exchange.platforms =
            ( ( m_exchange.platforms & left )  != 0 ? right : 0 )
          + ( ( m_exchange.platforms & right ) != 0 ? left : 0 );
    }

    if (MyTrack)
    { // podczas wczytywania wstawiane jest AI, ale może jeszcze nie być toru
        // AI ustawi kierunek ponownie po uruchomieniu silnika
        update_neighbours();
    }
    // informacja o położeniu następnego
    return 1 - ( iDirection > 0 ? NextConnectedNo() : PrevConnectedNo() );
};

// wskaźnik na poprzedni, nawet wirtualny
TDynamicObject * TDynamicObject::PrevAny() const {
    return MoverParameters->Neighbours[ iDirection ^ 1 ].vehicle;
}
TDynamicObject * TDynamicObject::Prev(CouplingSet const couplings) const {
	return MoverParameters->Couplers[iDirection ^ 1].CouplingFlag.HasAny(couplings) ? MoverParameters->Neighbours[iDirection ^ 1].vehicle : nullptr;// gdy sprzęg wirtualny, to jakby nic nie było
}
TDynamicObject * TDynamicObject::Next(CouplingSet const couplings) const {
	return MoverParameters->Couplers[iDirection].CouplingFlag.HasAny(couplings) ? MoverParameters->Neighbours[iDirection].vehicle : nullptr;// gdy sprzęg wirtualny, to jakby nic nie było
}

// checks whether there's unbroken connection of specified type to specified vehicle
bool
TDynamicObject::is_connected( TDynamicObject const *Vehicle, CouplingSet const couplings ) const {

    auto *vehicle { this };
    if( vehicle == Vehicle ) {
        // edge case, vehicle is always "connected" with itself
        return true;
    }
    // check ahead, it's more likely the "owner" using this method is located there
    while( ( vehicle = vehicle->Prev( couplings ) ) != nullptr ) {
        if( vehicle == Vehicle ) {
            return true;
        }
        if( vehicle == this ) {
            // edge case, looping consist
            return false;
        }
    }
    // start anew in the other direction
    vehicle = this;
    while( ( vehicle = vehicle->Next( couplings ) ) != nullptr ) {
        if( vehicle == Vehicle ) {
            return true;
        }
    }
    // no luck in either direction, give up
    return false;
}

// ustalenie następnego (1) albo poprzedniego (0) w składzie bez względu na prawidłowość iDirection
TDynamicObject *
TDynamicObject::Neighbour(int &dir) {

    auto *neighbour { (
        MoverParameters->Couplers[ dir ].CouplingFlag != Coupling::none ?
            MoverParameters->Neighbours[ dir ].vehicle :
            nullptr ) };
    // nowa wartość
    dir = 1 - MoverParameters->Neighbours[ dir ].vehicle_end;

    return neighbour;
};

// updates potential collision sources
void
TDynamicObject::update_neighbours() {

    for( int end = end::front; end <= end::rear; ++end ) {

        auto &neighbour { MoverParameters->Neighbours[ end ] };
        auto const &coupler { MoverParameters->Couplers[ end ] };

        if( coupler.Connected != nullptr
         && neighbour.vehicle != nullptr ) {
            // physical connection with another vehicle locks down collision source on this end
//            neighbour.vehicle = coupler.Connected;
//            neighbour.vehicle_end = coupler.ConnectedNr;
            neighbour.distance = TMoverParameters::CouplerDist( MoverParameters, coupler.Connected );
            if( MoverParameters->OffTrack.active() || neighbour.vehicle->MoverParameters->OffTrack.active() ) {
                // physics.derailment new: off the track the bodies turn against each other, the ends meet where they are
                // (the distance between the middles less the half lengths would press the buffers of a bent pair)
                neighbour.distance = off_track_gap( end );
            }
            // take into account potential adapters attached to the couplers
            auto const &othercoupler { neighbour.vehicle->MoverParameters->Couplers[ neighbour.vehicle_end ] };
            neighbour.distance -= coupler.adapter_length;
            neighbour.distance -= othercoupler.adapter_length;
        }
        else {
            // if there's no connected vehicle check for potential collision sources in the vicinity
            // NOTE: we perform a new scan on each update to ensure we always locate the nearest potential source
            neighbour = neighbour_data();
            // 10m ~= 140 km/h at 4 fps + safety margin, potential distance between outer axles of involved vehicles
            auto const scanrange { std::max( 10.0, std::abs( MoverParameters->V ) ) + 40.0 };
            // physics.derailment new: a vehicle off the track beside the other one doesn't run into it, the scan goes on
            // past it to the vehicles beyond
            auto const lookup { find_vehicle( end, scanrange, true ) };

            if( false == std::get<bool>( lookup ) ) { continue; }

            neighbour.vehicle = std::get<TDynamicObject *>( lookup );
            neighbour.vehicle_end = std::get<int>( lookup );
            neighbour.distance = std::get<double>( lookup );

            if( neighbour.vehicle
             && neighbour.distance < (neighbour.vehicle->MoverParameters->CategoryFlag == 2 ? 50 : 100) ) {
                // at short distances (re)calculate range between couplers directly
                neighbour.distance = TMoverParameters::CouplerDist( MoverParameters, neighbour.vehicle->MoverParameters );
                if( MoverParameters->OffTrack.active() || neighbour.vehicle->MoverParameters->OffTrack.active() ) {
                    // physics.derailment new: the ends where they are, as for a coupled pair
                    neighbour.distance = off_track_gap( end );
                }
                // take into account potential adapters attached to the couplers
                auto const &othercoupler { neighbour.vehicle->MoverParameters->Couplers[ neighbour.vehicle_end ] };
                neighbour.distance -= coupler.adapter_length;
                neighbour.distance -= othercoupler.adapter_length;
            }
        }
    }
}

bool
TDynamicObject::in_line_with( TDynamicObject const &Other ) const {

    // the middle of the other one off the axis of this one, and this one off the axis of the other one, by less than
    // the half widths of both: their ends can meet
    auto const reach { 0.5 * ( MoverParameters->Dim.W + Other.MoverParameters->Dim.W ) };
    auto const apart { Other.vPosition - vPosition };
    return std::abs( glm::dot( apart, vLeft ) ) < reach && std::abs( glm::dot( apart, Other.vLeft ) ) < reach;
}

// locates potential collision source within specified range, scanning track in specified direction. returns: true if neighbour was located, false otherwise
// NOTE: reuses legacy code. TBD, TODO: review, refactor?
std::tuple<TDynamicObject *, int, double, bool>
TDynamicObject::find_vehicle( int const Direction, double const Distance, bool const InLine ) const {

    auto direction { ( Direction == end::front ? 1 : -1 ) };
    auto const initialdirection { direction }; // zapamiętanie kierunku poszukiwań na torze początkowym, względem sprzęgów

    auto const *track { RaTrackGet() };
    if( RaDirectionGet() < 0 ) {
        // czy oś jest ustawiona w stronę Point1?
        direction = -direction;
    }

    // (teraz względem toru)
    auto const mycoupler { ( initialdirection < 0 ? end::rear : end::front ) }; // numer sprzęgu do podłączenia w obiekcie szukajacym
    auto foundcoupler { -1 }; // numer sprzęgu w znalezionym obiekcie (znaleziony wypełni)
    auto distance { 0.0 }; // przeskanowana odleglość; odległość do zawalidrogi
    auto *foundobject { ABuFindObject( foundcoupler, distance, track, direction, mycoupler, InLine ) }; // zaczynamy szukać na tym samym torze

    if( foundobject == nullptr ) {
        // jeśli nie ma na tym samym, szukamy po okolicy szukanie najblizszego toru z jakims obiektem
        // praktycznie przeklejone z TraceRoute()...
        if (direction >= 0) // uwzględniamy kawalek przeanalizowanego wcześniej toru
            distance = track->Length() - RaTranslationGet(); // odległość osi od Point2 toru
        else
            distance = RaTranslationGet(); // odległość osi od Point1 toru

        while (distance < Distance) {
            if (direction > 0) {
                // w kierunku Point2 toru
                if( track ?
                        track->iNextDirection :
                        false ) {
                    // jeśli następny tor jest podpięty od Point2
                    direction = -direction; // to zmieniamy kierunek szukania na tym torze
                }
                if( track ) {
                    track = track->CurrentNext(); // potem dopiero zmieniamy wskaźnik
                }
            }
            else {
                // w kierunku Point1
                if( track ?
                        !track->iPrevDirection :
                        true ) {
                    // jeśli poprzedni tor nie jest podpięty od Point2
                    direction = -direction; // to zmieniamy kierunek szukania na tym torze
                }
                if( track ) {
                    track = track->CurrentPrev(); // potem dopiero zmieniamy wskaźnik
                }
            }
            if (track) {
                // jesli jest kolejny odcinek toru
                foundobject = ABuFindObject(foundcoupler, distance, track, direction, mycoupler, InLine); // przejrzenie pojazdów tego toru
                if (foundobject) {
                    break;
                }
            }
            else {
                // jeśli toru nie ma, to wychodzimy
                distance = Distance + 1.0; // koniec przeglądania torów
                break;
            }
        }
    } // Koniec szukania najbliższego toru z jakimś obiektem.

    if( foundobject == nullptr ) { return {}; }
/*
    auto const *vehicle { MoverParameters };
    auto const *foundvehicle { foundobject->MoverParameters };
    if( ( false == TestFlag( track->iCategoryFlag, 1 ) )
     && ( distance > 50.0 ) ) {
        // Ra: jeśli dwa samochody się mijają na odcinku przed zawrotką, to odległość między nimi nie może być liczona w linii prostej!
        // NOTE: the distance is approximated, and additionally less accurate for cars heading in opposite direction
        distance -= ( 0.5 * ( vehicle->Dim.L + foundvehicle->Dim.L ) );
    }
    else if( distance < 100.0 ) {
        // at short distances start to calculate range between couplers directly
        // odległość do najbliższego pojazdu w linii prostej
        distance = TMoverParameters::CouplerDist( vehicle, foundvehicle );
    }
*/
    return { foundobject, foundcoupler, distance, true };
}

TDynamicObject * TDynamicObject::FindPowered()
{ // taka proteza:
    // chcę podłączyć kabinę EN57 bezpośrednio z silnikowym, aby nie robić tego przez ukrotnienie
    // drugi silnikowy i tak musi być ukrotniony, podobnie jak kolejna jednostka
    // lepiej by było przesyłać komendy sterowania, co jednak wymaga przebudowy transmisji komend (LD)
    // problem się robi ze światłami, które będą zapalane w silnikowym, ale muszą świecić się w rozrządczych
    // dla EZT światłą czołowe będą "zapalane w silnikowym", ale widziane z rozrządczych
    // również wczytywanie MMD powinno dotyczyć aktualnego członu
    // problematyczna może być kwestia wybranej kabiny (w silnikowym...)
    // jeśli silnikowy będzie zapięty odwrotnie (tzn. -1), to i tak powinno jeździć dobrze
    // również hamowanie wykonuje się zaworem w członie, a nie w silnikowym...
    auto const coupling { MoverParameters->is_multiple_unit() ?
            Coupling::permanent :
            Coupling::control };

    auto *lookup {
        find_vehicle(
            coupling,
            []( TDynamicObject * vehicle ) {
                return vehicle->MoverParameters->Power > 1.0; } ) };

    return lookup != nullptr ? lookup : this; // always return valid vehicle for backward compatibility
}

TDynamicObject *
TDynamicObject::FindPantographCarrier() {

    // try first within a single unit, broaden to all vehicles under our control if first attempt fails
    std::array<Coupling, 2> const couplings = { Coupling::permanent, Coupling::control };

    for( auto const coupling : couplings ) {
        auto *result =
            find_vehicle(
                coupling,
                []( TDynamicObject * vehicle ) {
                    return vehicle->MoverParameters->EnginePowerSource.SourceType == TPowerSource::CurrentCollector && vehicle->MoverParameters->EnginePowerSource.CollectorParameters.CollectorsNo > 0; } );
        if( result != nullptr ) {
            return result;
        }
    }
    // if we're still here, admit failure
    return nullptr;
}

//---------------------------------------------------------------------------

void TDynamicObject::ParamSet(int what, int into)
{ // ustawienie lokalnego parametru (what) na stan (into)
    switch (what & 0xFF00)
    {
    case 0x0100: // to np. są drzwi, bity 0..7 określają numer 1..254 albo maskę
        // dla 8 różnych
        if (what & 1) // na razie mamy lewe oraz prawe, czyli używamy maskę 1=lewe,
            // 2=prawe, 3=wszystkie
            if( MoverParameters->Doors.instances[side::left].is_open )
            { // są otwarte
                if (!into) // jeśli zamykanie
                {
                    // dźwięk zamykania
                }
            }
            else
            { // są zamknięte
                if (into) // jeśli otwieranie
                {
                    // dźwięk otwierania
                }
            }
        if (what & 2) // prawe działają niezależnie od lewych
            if( MoverParameters->Doors.instances[side::right].is_open )
            { // są otwarte
                if (!into) // jeśli zamykanie
                {
                    // dźwięk zamykania
                }
            }
            else
            { // są zamknięte
                if (into) // jeśli otwieranie
                {
                    // dźwięk otwierania
                }
            }
        break;
    }
};

int TDynamicObject::RouteWish(TTrack *tr)
{ // zapytanie do AI, po którym
    // segmencie (-6..6) jechać na
    // skrzyżowaniu (tr)
    return Mechanik ? Mechanik->CrossRoute(tr) : 0; // wg AI albo prosto
};

// the vehicles coupled at the ends, and the ends of theirs facing this one
TDynamicObject *TDynamicObject::NextConnected() { return MoverParameters->Neighbours[ end::rear ].vehicle; }
TDynamicObject *TDynamicObject::PrevConnected() { return MoverParameters->Neighbours[ end::front ].vehicle; }
TDynamicObject *TDynamicObject::NextConnected() const { return MoverParameters->Neighbours[ end::rear ].vehicle; }
TDynamicObject *TDynamicObject::PrevConnected() const { return MoverParameters->Neighbours[ end::front ].vehicle; }
int TDynamicObject::NextConnectedNo() const { return MoverParameters->Neighbours[ end::rear ].vehicle_end; }
int TDynamicObject::PrevConnectedNo() const { return MoverParameters->Neighbours[ end::front ].vehicle_end; }

void TDynamicObject::SetdMoveLen( double dMoveLen ) { MoverParameters->dMoveLen = dMoveLen; }
void TDynamicObject::ResetdMoveLen() { MoverParameters->dMoveLen = 0; }
double TDynamicObject::GetdMoveLen() { return MoverParameters->dMoveLen; }

double TDynamicObject::GetVelocity() const { return MoverParameters->Vel; }
double TDynamicObject::GetLength() const { return MoverParameters->Dim.L; }
double TDynamicObject::GetWidth() const { return MoverParameters->Dim.W; }
