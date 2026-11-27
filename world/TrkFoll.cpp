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
    Copyright (C) 2001-2004  Marcin Wozniak and others

*/

#include "stdafx.h"
#include "world/TrkFoll.h"

#include "simulation/simulation.h"
#include "utilities/Globals.h"
#include "vehicle/DynObj.h"
#include "vehicle/Driver.h"
#include "utilities/Logs.h"

namespace {

// how far the start of a way across a crossroads can be from the end of the path an axle leaves, to be where the axle gets on
constexpr double cross_reach { 1.0 };

// number of the ways across a crossroads; each can be taken either way
int cross_count( TTrack const &Cross ) {

    return Cross.SwitchExtension->iRoads == 4 ? 6 : 3;
}

// where the way of specified code starts, and the direction it leads in there. the code is the number of the way, 1..6,
// negative if the way is taken against its points
bool cross_start( TTrack const &Cross, int const Code, glm::dvec3 &Point, glm::dvec3 &Direction ) {

    if( Code == 0 || std::abs( Code ) > cross_count( Cross ) ) { return false; }
    auto const &segment { Cross.SwitchExtension->Segments[ std::abs( Code ) - 1 ] };
    if( segment == nullptr ) { return false; }
    Point = ( Code > 0 ? segment->FastGetPoint_0() : segment->FastGetPoint_1() );
    Direction = ( Code > 0 ? segment->FastGetPoint( 0.05 ) : segment->FastGetPoint( 0.95 ) ) - Point;
    return true;
}

} // namespace

TTrackFollower::~TTrackFollower()
{
}

bool TTrackFollower::Init(TTrack *pTrack, TDynamicObject *NewOwner, double fDir)
{
    fDirection = fDir;
    Owner = NewOwner;
    // the axle is put on the track, it doesn't come from the one it was on before
    pCurrentSegment = nullptr;
    SetCurrentTrack(pTrack, 0);
    iEventFlag = 3; // na torze startowym również wykonać eventy 1/2
    iEventallFlag = 3;
    if (pCurrentSegment) // && (pCurrentSegment->GetLength()<fFirstDistance))
        return false;
    return true;
}

void TTrackFollower::Reset()
{
	fCurrentDistance = 0.0;
	fDirection = 1.0;
}

TTrack * TTrackFollower::SetCurrentTrack(TTrack *pTrack, int end)
{ // przejechanie na inny odcinkek toru, z ewentualnym rozpruciem
    if (pTrack)
        switch (pTrack->eType)
        {
        case tt_Switch: // jeśli zwrotnica, to przekładamy ją, aby uzyskać dobry segment
        {
            if( pCurrentTrack == nullptr ) {
                break;
            }
            int i = end ? pCurrentTrack->iNextDirection : pCurrentTrack->iPrevDirection;
            if (i > 0) // jeżeli wjazd z ostrza
                pTrack->SwitchForced(i >> 1, Owner); // to przełożenie zwrotnicy - rozprucie!
        }
        break;
        case tt_Cross: // skrzyżowanie trzeba tymczasowo przełączyć, aby wjechać na właściwy tor
        {
            iSegment = Owner->RouteWish(pTrack); // nr segmentu został ustalony podczas skanowania
            // Ra 2014-08: aby ustalić dalszą trasę, należy zapytać AI - trasa jest ustalana podczas
            // skanowania
            // Ra 15-01: zapytanie AI nie wybiera segmentu - kolejny skanujący może przestawić
            // pTrack->CrossSegment(end?pCurrentTrack->iNextDirection:pCurrentTrack->iPrevDirection,i);
            // //ustawienie właściwego wskaźnika
            // powinno zwracać kierunek do zapamiętania, bo segmenty mogą mieć różny kierunek
            // if fDirection=(iSegment>0)?1.0:-1.0; //kierunek na tym segmencie jest ustalany
            // bezpośrednio
            if (iSegment == 0)
            { // to jest błędna sytuacja - generuje pętle zawracające za skrzyżowaniem - ustalić,
                // kiedy powstaje!
                iSegment = 1; // doraźna poprawka
            }
            // the way has to start where the axle gets on. the one the driver picked may not: the driver may have looked
            // again since the other axle got on, the vehicle may have no driver of its own, or it may be going backwards
            iSegment = CrossEntry( *pTrack, iSegment, end );
            if ((end ? iSegment : -iSegment) < 0)
                fDirection = -fDirection; // wtórna zmiana
            pTrack->SwitchForced(abs(iSegment) - 1, nullptr); // wybór zapamiętanego segmentu
        }
        break;
        }
    if (!pTrack) {
        // gdy nie ma toru w kierunku jazdy tworzenie toru wykolejącego na przedłużeniu pCurrentTrack
        pTrack = pCurrentTrack->NullCreate(end);
        if (!end) // jeśli dodana od strony zero, to zmiana kierunku
            fDirection = -fDirection; // wtórna zmiana
    }
    else
    { // najpierw +1, później -1, aby odcinek izolowany wspólny dla tych torów nie wykrył zera
        pTrack->AxleCounter(+1, Owner); // zajęcie nowego toru
        if (pCurrentTrack)
            pCurrentTrack->AxleCounter(-1, Owner); // opuszczenie tamtego toru
    }
    pCurrentTrack = pTrack;
    pCurrentSegment = pCurrentTrack != nullptr ? pCurrentTrack->CurrentSegment() : nullptr;
    if (!pCurrentTrack)
        Error(Owner->MoverParameters->Name + " at NULL track");
    return pCurrentTrack;
};

// picks the way across the crossroads (Cross) for the axle which gets on it by specified end of the path it's on. it's the way
// taken by another axle of the vehicle, or of a vehicle coupled to it, which is on the crossroads already; or the one the driver
// picked (Wish); or, if neither starts where the axle gets on, the one leading the most straight on of the ways which start there
int TTrackFollower::CrossEntry( TTrack const &Cross, int const Wish, int const End ) const {

    if( pCurrentSegment == nullptr || Cross.SwitchExtension == nullptr ) { return Wish; }

    auto const joint { End ? pCurrentSegment->FastGetPoint_1() : pCurrentSegment->FastGetPoint_0() };
    auto const starts_here = [&]( int const Code ) {
        glm::dvec3 point, direction;
        return cross_start( Cross, Code, point, direction ) && glm::distance( point, joint ) < cross_reach; };

    if( Owner != nullptr ) {
        for( auto const *vehicle : { Owner, Owner->Prev(), Owner->Next() } ) {
            if( vehicle == nullptr ) { continue; }
            for( auto const *axle : { &vehicle->Axle0, &vehicle->Axle1 } ) {
                if( axle == this || axle->pCurrentTrack != &Cross || axle->iSegment == 0 ) { continue; }
                if( starts_here( axle->iSegment ) ) { return axle->iSegment; }
                // a vehicle going back over the crossroads gets on the way at its other end
                if( starts_here( -axle->iSegment ) ) { return -axle->iSegment; }
            }
        }
    }
    if( starts_here( Wish ) ) { return Wish; }

    auto const incoming { joint - ( End ? pCurrentSegment->FastGetPoint( 0.95 ) : pCurrentSegment->FastGetPoint( 0.05 ) ) };
    auto nearest { cross_reach };
    for( int way = 1; way <= cross_count( Cross ); ++way ) {
        for( auto const code : { way, -way } ) {
            glm::dvec3 point, direction;
            if( cross_start( Cross, code, point, direction ) ) {
                nearest = std::min( nearest, glm::distance( point, joint ) );
            }
        }
    }
    if( nearest >= cross_reach ) {
        // no way starts there, the crossroads isn't joined with the path; it's left the way it was
        return Wish;
    }
    auto result { Wish };
    auto straightest { -2.0 };
    for( int way = 1; way <= cross_count( Cross ); ++way ) {
        for( auto const code : { way, -way } ) {
            glm::dvec3 point, direction;
            if( false == cross_start( Cross, code, point, direction )
             || glm::distance( point, joint ) > nearest + 0.05 ) {
                continue;
            }
            auto const straight {
                glm::length2( incoming ) > 0.0 && glm::length2( direction ) > 0.0 ?
                    glm::dot( glm::normalize( incoming ), glm::normalize( direction ) ) :
                    -1.0 };
            if( straight > straightest ) {
                straightest = straight;
                result = code;
            }
        }
    }
    return result;
}

bool TTrackFollower::Move(double fDistance, bool bPrimary)
{ // przesuwanie wózka po torach o odległość (fDistance), z wyzwoleniem eventów
    // bPrimary=true - jest pierwszą osią w pojeździe, czyli generuje eventy i przepisuje pojazd
    // Ra: zwraca false, jeśli pojazd ma być usunięty
    auto const ismoving { /* ( std::abs( fDistance ) > 0.01 ) && */ ( Owner->GetVelocity() > 0.01 ) };
    fDistance *= fDirection; // dystans mnożnony przez kierunek
    double s; // roboczy dystans
    double dir; // zapamiętany kierunek do sprawdzenia, czy się zmienił
    bool bCanSkip; // czy przemieścić pojazd na inny tor
    while (true) // pętla wychodzi, gdy przesunięcie wyjdzie zerowe
    { // pętla przesuwająca wózek przez kolejne tory, aż do trafienia w jakiś
        if( pCurrentTrack == nullptr ) { return false; } // nie ma toru, to nie ma przesuwania
        // TODO: refactor following block as track method
        if( pCurrentTrack->m_events ) { // sumaryczna informacja o eventach
            // omijamy cały ten blok, gdy tor nie ma on żadnych eventów (większość nie ma)
            int const eventfilter { (
                false == ismoving ? 0 : // only moving vehicles activate events type 1/2
                false == bPrimary ? 0 : // only primary axle activates events type 1/2
                Owner->ctOwner == nullptr ?
                    ( fDistance > 0 ? 1 : -1 ) : // loose vehicle has no means to determine 'intended' direction so the filter does effectively nothing in such case
                    ( fDirection > 0 ? 1 : -1 ) * Owner->ctOwner->Direction() * ( Owner->ctOwner->Vehicle()->DirectionGet() == Owner->DirectionGet() ? 1 : -1 ) ) };

            if( false == ismoving ) {
                //McZapkie-140602: wyzwalanie zdarzenia gdy pojazd stoi
                if( Owner->Mechanik != nullptr
                 && Owner->Mechanik->primary() ) {
                    // tylko dla jednego członu
                    pCurrentTrack->QueueEvents( pCurrentTrack->m_events0, Owner );
                }
                pCurrentTrack->QueueEvents( pCurrentTrack->m_events0all, Owner );
            }
            else if( fDistance < 0 && eventfilter < 0 ) {
                // event1, eventall1
                if( SetFlag( iEventFlag, -1 ) ) {
                    // zawsze zeruje flagę sprawdzenia, jak mechanik dosiądzie, to się nie wykona
                    if( Owner->Mechanik != nullptr
                     && Owner->Mechanik->primary() ) {
                        // tylko dla jednego członu
                        // McZapkie-280503: wyzwalanie event tylko dla pojazdow z obsada
                        pCurrentTrack->QueueEvents( pCurrentTrack->m_events1, Owner );
                    }
                }
                if( SetFlag( iEventallFlag, -1 ) ) {
                    // McZapkie-280503: wyzwalanie eventall dla wszystkich pojazdow
                    pCurrentTrack->QueueEvents( pCurrentTrack->m_events1all, Owner );
                }
            }
            else if( fDistance > 0 && eventfilter > 0 ) {
                // event2, eventall2
                if( SetFlag( iEventFlag, -2 ) ) {
                    // zawsze ustawia flagę sprawdzenia, jak mechanik dosiądzie, to się nie wykona
                    if( Owner->Mechanik != nullptr
                     && Owner->Mechanik->primary() ) {
                        // tylko dla jednego członu
                         pCurrentTrack->QueueEvents( pCurrentTrack->m_events2, Owner );
                    }
                }
                if( SetFlag( iEventallFlag, -2 ) ) {
                    // sprawdza i zeruje na przyszłość, true jeśli zmieni z 2 na 0
                    pCurrentTrack->QueueEvents( pCurrentTrack->m_events2all, Owner );
                }
            }
        }
        if (!pCurrentSegment) // jeżeli nie ma powiązanego segmentu toru?
            return false;
        // if (fDistance==0.0) return true; //Ra: jak stoi, to chyba dalej nie ma co kombinować?
        s = fCurrentDistance + fDistance; // doliczenie przesunięcia
        // Ra: W Point2 toru może znajdować się "dziura", która zamieni energię kinetyczną
        // ruchu wzdłużnego na energię potencjalną, zamieniającą się potem na energię
        // sprężystości na amortyzatorach. Należałoby we wpisie toru umieścić współczynnik
        // podziału energii kinetycznej.
        // Współczynnik normalnie 1, z dziurą np. 0.99, a -1 będzie oznaczało 100% odbicia (kozioł).
        // Albo w postaci kąta: normalnie 0°, a 180° oznacza 100% odbicia (cosinus powyższego).
        /*
          if (pCurrentTrack->eType==tt_Cross)
          {//zjazdu ze skrzyżowania nie da się określić przez (iPrevDirection) i (iNextDirection)
           //int segment=Owner->RouteWish(pCurrentTrack); //numer segmentu dla skrzyżowań
           //pCurrentTrack->SwitchForced(abs(segment)-1,NULL); //tymczasowo ustawienie tego segmentu
           //pCurrentSegment=pCurrentTrack->CurrentSegment(); //odświeżyć sobie wskaźnik segmentu
          (?)
          }
        */
        if (s < 0)
        { // jeśli przekroczenie toru od strony Point1
            bCanSkip = bPrimary && pCurrentTrack->CheckDynamicObject(Owner);
            if( bCanSkip ) {
                // tylko główna oś przenosi pojazd do innego toru
                // zdejmujemy pojazd z dotychczasowego toru
                Owner->MyTrack->RemoveDynamicObject( Owner );
            }
            dir = fDirection;
            if (pCurrentTrack->eType == tt_Cross)
            {
                if (!SetCurrentTrack(pCurrentTrack->Connected(iSegment, fDirection), 0))
                    return false; // wyjście z błędem
            }
            else if (!SetCurrentTrack(pCurrentTrack->Connected(-1, fDirection), 0)) // ustawia fDirection
                return false; // wyjście z błędem
            if (dir == fDirection) //(pCurrentTrack->iPrevDirection)
            { // gdy kierunek bez zmiany (Point1->Point2)
                fCurrentDistance = pCurrentSegment->GetLength();
                fDistance = s;
            }
            else
            { // gdy zmiana kierunku toru (Point1->Point1)
                fCurrentDistance = 0;
                fDistance = -s;
            }
            if (bCanSkip)
            { // jak główna oś, to dodanie pojazdu do nowego toru
                pCurrentTrack->AddDynamicObject(Owner);
                iEventFlag = 3; // McZapkie-020602: umozliwienie uruchamiania event1,2 po zmianie toru
                iEventallFlag = 3; // McZapkie-280503: jw, dla eventall1,2
                if (!Owner->MyTrack)
                    return false;
            }
            continue;
        }
        else if (s > pCurrentSegment->GetLength())
        { // jeśli przekroczenie toru od strony Point2
            bCanSkip = bPrimary && pCurrentTrack->CheckDynamicObject(Owner);
            if (bCanSkip) // tylko główna oś przenosi pojazd do innego toru
                Owner->MyTrack->RemoveDynamicObject(Owner); // zdejmujemy pojazd z dotychczasowego toru
            fDistance = s - pCurrentSegment->GetLength();
            dir = fDirection;
            if (pCurrentTrack->eType == tt_Cross)
            {
                if (!SetCurrentTrack(pCurrentTrack->Connected(iSegment, fDirection), 1))
                    return false; // wyjście z błędem
            }
            else if (!SetCurrentTrack(pCurrentTrack->Connected(1, fDirection), 1)) // ustawia fDirection
                return false; // wyjście z błędem
            if (dir != fDirection) //(pCurrentTrack->iNextDirection)
            { // gdy zmiana kierunku toru (Point2->Point2)
                fDistance = -fDistance; //(s-pCurrentSegment->GetLength());
                fCurrentDistance = pCurrentSegment->GetLength();
            }
            else // gdy kierunek bez zmiany (Point2->Point1)
                fCurrentDistance = 0;
            if (bCanSkip)
            { // jak główna oś, to dodanie pojazdu do nowego toru
                pCurrentTrack->AddDynamicObject(Owner);
                iEventFlag = 3; // McZapkie-020602: umozliwienie uruchamiania event1,2 po zmianie toru
                iEventallFlag = 3;
                if (!Owner->MyTrack)
                    return false;
            }
            continue;
        }
        else
        { // gdy zostaje na tym samym torze (przesuwanie już nie zmienia toru)
            if (bPrimary)
            { // tylko gdy początkowe ustawienie, dodajemy eventy stania do kolejki
                if (Owner->MoverParameters->CabOccupied != 0) {

                    pCurrentTrack->QueueEvents( pCurrentTrack->m_events1, Owner, -1.0 );
                    pCurrentTrack->QueueEvents( pCurrentTrack->m_events2, Owner, -1.0 );
                }
                pCurrentTrack->QueueEvents( pCurrentTrack->m_events1all, Owner, -1.0 );
                pCurrentTrack->QueueEvents( pCurrentTrack->m_events2all, Owner, -1.0 );
            }
            fCurrentDistance = s;
            return ComputatePosition(); // przeliczenie XYZ, true o ile nie wyjechał na NULL
        }
    }
};

int TTrackFollower::Path() const {

    if( pCurrentTrack == nullptr || pCurrentTrack->SwitchExtension == nullptr ) { return 0; }
    if( pCurrentTrack->eType == tt_Cross ) { return iSegment; }
    if( pCurrentTrack->eType == tt_Switch ) { return pCurrentSegment == pCurrentTrack->SwitchExtension->Segments[ 1 ] ? 1 : 0; }
    return 0;
}

bool TTrackFollower::Place( TTrack *Track, int const Path, double const Distance, double const Direction, int const Eventflags, int const Eventallflags ) {

    if( Track == nullptr ) { return false; }
    // the path of a switch the axle stands on may be other than the one the switch is set to, a crossing has several
    auto segment { Track->CurrentSegment() };
    auto crossingsegment { 0 };
    if( Track->SwitchExtension != nullptr && ( Track->eType == tt_Switch || Track->eType == tt_Cross ) ) {
        auto const index { Track->eType == tt_Switch ? ( Path & 1 ) : std::abs( Path ) - 1 };
        if( index >= 0 && index < 6 && Track->SwitchExtension->Segments[ index ] != nullptr ) {
            segment = Track->SwitchExtension->Segments[ index ];
            crossingsegment = ( Track->eType == tt_Cross ? Path : 0 );
        }
    }
    if( segment == nullptr ) { return false; }

    if( Track != pCurrentTrack ) {
        // najpierw +1, później -1, as in SetCurrentTrack()
        Track->AxleCounter( +1, Owner );
        if( pCurrentTrack != nullptr ) {
            pCurrentTrack->AxleCounter( -1, Owner );
        }
    }
    pCurrentTrack = Track;
    pCurrentSegment = segment;
    iSegment = crossingsegment;
    fDirection = ( Direction < 0.0 ? -1.0 : 1.0 );
    fCurrentDistance = std::clamp( Distance, 0.0, pCurrentSegment->GetLength() );
    iEventFlag = Eventflags;
    iEventallFlag = Eventallflags;
    return ComputatePosition();
}

double TTrackFollower::RailPhase( double const Offset ) const {

    return pCurrentTrack != nullptr ? pCurrentTrack->rail_phase( fCurrentDistance + fDirection * Offset ) : -1.0;
}

double TTrackFollower::BuckleCurvature( double const Base ) const {

    return pCurrentTrack != nullptr ? pCurrentTrack->buckle_curvature( fCurrentDistance, Base ) : 0.0;
}

glm::dvec3 TTrackFollower::PathDirection( double const Reach ) const {

    if( pCurrentSegment == nullptr ) {
        return glm::dvec3 { 0.0 };
    }
    auto const length { pCurrentSegment->GetLength() };
    glm::dvec3 ahead;
    glm::dvec3 behind;
    glm::vec3 rotation;
    pCurrentSegment->RaPositionGet( std::clamp( fCurrentDistance + Reach, 0.0, length ), ahead, rotation );
    pCurrentSegment->RaPositionGet( std::clamp( fCurrentDistance - Reach, 0.0, length ), behind, rotation );
    glm::dvec3 const direction { ( ahead.x - behind.x ) * fDirection, 0.0, ( ahead.z - behind.z ) * fDirection };
    auto const size { glm::length( direction ) };
    return size > 1e-6 ? direction / size : glm::dvec3 { 0.0 };
}

bool TTrackFollower::ComputatePosition()
{ // ustalenie współrzędnych XYZ
    if (pCurrentSegment) // o ile jest tor
    {
        pCurrentSegment->RaPositionGet(fCurrentDistance, pPosition, vAngles);
        if (fDirection < 0) // kąty zależą jeszcze od zwrotu na torze
        { // kąty są w przedziale <-M_PI;M_PI>
            vAngles.x = -vAngles.x; // przechyłka jest w przecinwą stronę
            vAngles.y = -vAngles.y; // pochylenie jest w przecinwą stronę
            vAngles.z +=
                vAngles.z >= M_PI ? -M_PI : M_PI; // ale kierunek w planie jest obrócony o 180°
        }
        if (fOffsetH != 0.0)
        { // jeśli przesunięcie względem osi toru, to je doliczyć
        }
        return true;
    }
    return false;
}
