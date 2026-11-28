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

// scanning the route ahead into the speed table, the signals passed

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

// finds point of specified track nearest to specified event. returns: distance to that point from the specified end of the track
// TODO: move this to file with all generic routines, too easy to forget it's here and it may come useful
double
ProjectEventOnTrack( basic_event const *Event, TTrack const *Track, double const Direction ) {

    auto const segment = Track->CurrentSegment();
    auto const nearestpoint = segment->find_nearest_point( Event->input_location() );
    return Direction > 0 ? nearestpoint * segment->GetLength() : // measure from point1
	                       (1.0 - nearestpoint) * segment->GetLength(); // measure from point2
};

double GetDistanceToEvent(TTrack const *track, basic_event const *event, double scan_dir, double start_dist, int iter = 0, bool back = false)
{
    if( track == nullptr ) { return start_dist; }

    auto const segment = track->CurrentSegment();
    auto const pos_event = event->input_location();
    double len1, len2;
    double sd = scan_dir;
    double seg_len = scan_dir > 0 ? 0.0 : 1.0;
    double const dzielnik = 1.0 / segment->GetLength();// rozdzielczosc mniej wiecej 1m
    int krok = 0; // krok obliczeniowy do sprawdzania czy odwracamy
	len2 = glm::length2(pos_event - segment->FastGetPoint(seg_len));
    do
    {
        len1 = len2;
        seg_len += scan_dir > 0 ? dzielnik : -dzielnik;
		len2 = glm::length2(pos_event - segment->FastGetPoint(seg_len));
        ++krok;
    } while (len1 > len2 && seg_len >= dzielnik && seg_len <= 1.0 - dzielnik);
    //trzeba sprawdzić czy seg_len nie osiągnął skrajnych wartości, bo wtedy
    // trzeba sprawdzić tor obok
    if (1 == krok)
        sd = -sd; // jeśli tylko jeden krok tzn, że event przy poprzednim sprawdzaym torze
    if ((1 == krok || seg_len <= dzielnik || seg_len > 1.0 - dzielnik) && iter < 3)
    { // przejście na inny tor
        track = track->Connected(int(sd), sd);
        start_dist += 1 == krok ? 0 : back ? -segment->GetLength() : segment->GetLength();
        if( track != nullptr
         && track->eType == tt_Cross ) {
            // NOTE: tracing through crossroads currently poses risk of tracing through wrong segment
            // as it's possible to be performerd before setting a route through the crossroads
            // as a stop-gap measure we don't trace through crossroads which should be reasonable in most cases
            // TODO: establish route before the scan, or a way for the function to know which route to pick
            return start_dist;
        }
        else {
            return GetDistanceToEvent( track, event, sd, start_dist, ++iter, 1 == krok ? true : false );
        }
    }
    else
    { // obliczenie mojego toru
        seg_len -= scan_dir > 0 ? dzielnik : -dzielnik; //trzeba wrócić do pozycji len1
        seg_len = scan_dir < 0 ? 1 - seg_len : seg_len;
        seg_len = back ? 1 - seg_len : seg_len; // odwracamy jeśli idzie do tyłu
        start_dist -= back ? segment->GetLength() : 0;
        return start_dist + segment->GetLength() * seg_len;
    }
};
namespace
{


// the low 4 bits of a number as a number -8..7, as the route of a crossing was read from the top of the flags
int signedNibble(unsigned const value)
{
    auto const nibble{static_cast<int>(value & 0xf)};
    return nibble >= 8 ? nibble - 16 : nibble;
}

} // namespace

//---------------------------------------------------------------------------

void TController::TableClear()
{ // wyczyszczenie tablicy
    sSpeedTable.clear();
    iLast = -1;
    iTableDirection = 0; // nieznany
    tLast = nullptr;
    fLastVel = -1.0;
    SemNextIndex = npos;
    SemNextStopIndex = npos;
    eSignSkip = nullptr; // nic nie pomijamy
};

std::vector<basic_event *> TController::CheckTrackEvent( TTrack *Track, double const fDirection ) const
{ // sprawdzanie eventów na podanym torze do podstawowego skanowania
    std::vector<basic_event *> events;
    auto const &eventsequence { ( fDirection > 0 ? Track->m_events2 : Track->m_events1 ) };
    for( auto const &event : eventsequence ) {
        if( event.second != nullptr
         && event.second->m_passive ) {
            events.emplace_back( event.second );
        }
    }
    return events;
}

bool TController::TableAddNew()
{ // zwiększenie użytej tabelki o jeden rekord
    sSpeedTable.emplace_back(); // add a new slot
    iLast = sSpeedTable.size() - 1;
    return true; // false gdy się nałoży
};

bool TController::TableNotFound(basic_event const *Event, double const Distance ) const
{ // sprawdzenie, czy nie został już dodany do tabelki (np. podwójne W4 robi problemy)
    auto lookup =
        std::find_if(
            sSpeedTable.begin(),
            sSpeedTable.end(),
            [Event]( TSpeedPos const &speedpoint ){
                return true == speedpoint.flags.HasAll(SpeedPointFlag::enabled | SpeedPointFlag::event) && speedpoint.evEvent == Event; } );

    if (Global.iWriteLogEnabled & 8
     && lookup != sSpeedTable.end() ) {
        WriteLog( "Speed table for " + OwnerName() + " already contains event " + lookup->evEvent->m_name );
    }

    // ignore duplicates which seem to be reasonably apart from each other, on account of looping tracks
    // NOTE: since supplied distance is only rough approximation of distance to the event, we're using large safety margin
    return lookup == sSpeedTable.end() || Distance - lookup->fDist > 100.0;
};

void TController::TableTraceRoute(double fDistance, TDynamicObject *pVehicle)
{ // skanowanie trajektorii na odległość (fDistance) od (pVehicle) w kierunku przodu składu i
    // uzupełnianie tabelki
    // WriteLog("Starting TableTraceRoute");

    TTrack *pTrack{ nullptr }; // zaczynamy od ostatniego analizowanego toru
    double fTrackLength{ 0.0 }; // długość aktualnego toru (krótsza dla pierwszego)
    double fCurrentDistance{ 0.0 }; // aktualna przeskanowana długość
    double fLastDir{ 0.0 };

    if (iTableDirection != iDirection ) {
        // jeśli zmiana kierunku, zaczynamy od toru ze wskazanym pojazdem
        TableClear();
/*
        // aktualna prędkość // changed to -1 to recognize speed limit, if any
        fLastVel = -1.0;
        sSpeedTable.clear();
        iLast = -1;
        tLast = nullptr; //żaden nie sprawdzony
        SemNextIndex = npos;
        SemNextStopIndex = npos;
*/
        if( VelSignalLast == 0.0 ) {
            // don't allow potential red light overrun keep us from reversing
            VelSignalLast = -1.0;
        }
        iTableDirection = iDirection; // ustalenie w jakim kierunku jest wypełniana tabelka względem pojazdu
        pTrack = pVehicle->GetTrack(); // odcinek, na którym stoi
        fTrackLength = pVehicle->RaTranslationGet(); // pozycja na tym torze (odległość od Point1)
        fLastDir = pVehicle->DirectionGet() * pVehicle->RaDirectionGet(); // ustalenie kierunku skanowania na torze
        if( fLastDir < 0.0 ) {
            // jeśli w kierunku Point2 toru
            fTrackLength = pTrack->Length() - fTrackLength; // przeskanowana zostanie odległość do Point2
        }
        // account for the fact tracing begins from active axle, not the actual front of the vehicle
        // NOTE: position of the couplers is modified by track offset, but the axles ain't, so we need to account for this as well
        fTrackLength -= glm::length(
            pVehicle->AxlePositionGet()
            - pVehicle->RearPosition()
            + pVehicle->VectorLeft() * pVehicle->MoverParameters->OffsetTrackH );
        // aktualna odległość ma być ujemna gdyż jesteśmy na końcu składu
        fCurrentDistance = -fLength - fTrackLength;
        fTrackLength = pTrack->Length(); //skasowanie zmian w zmiennej żeby poprawnie liczyło w dalszych krokach
        MoveDistanceReset(); // AI startuje 1s po zaczęciu jazdy i mógł już coś przejechać
    }
    else {
        if( iTableDirection == 0 ) { return; }
        // NOTE: belt and braces; TablePurger() used to leave these pointing past the end of the table.
        // nothing is known to do that any more, so say so when it happens
        auto const dropstaleindex = [this, fDistance]( std::size_t &Index, std::string_view const Name ) {
            if( Index == npos
             || Index < sSpeedTable.size() ) {
                return;
            }
            ErrorLog( std::format(
                "Speed table for {}: {} {} points past the end of the table ({} records), last track {} at {:.1f} m, scan range {:.1f} m; reset",
                OwnerName(), Name, Index, sSpeedTable.size(),
                ( iLast >= 0 && sSpeedTable[ iLast ].trTrack != nullptr ? sSpeedTable[ iLast ].trTrack->name() : "none" ),
                ( iLast >= 0 ? sSpeedTable[ iLast ].fDist : 0.0 ), fDistance ) );
            Index = npos; };
        dropstaleindex( SemNextIndex, "SemNextIndex" );
        dropstaleindex( SemNextStopIndex, "SemNextStopIndex" );
        // kontynuacja skanowania od ostatnio sprawdzonego toru (w ostatniej pozycji zawsze jest tor)
        if( SemNextStopIndex != npos
         && sSpeedTable[SemNextStopIndex].fVelNext == 0.0 ) {
            // znaleziono semafor lub tarczę lub tor z prędkością zero, trzeba sprawdzić czy to nadał semafor
            // jeśli jest następny semafor to sprawdzamy czy to on nadał zero
            if( OrderCurrentGet() & DriverOrder::obeyTrain
             && sSpeedTable[SemNextStopIndex].flags & SpeedPointFlag::signal ) {
                return;
            }
            else {
                if( false == OrderCurrentGet().HasAny(trainOrders)
                 && sSpeedTable[SemNextStopIndex].flags & (SpeedPointFlag::signal | SpeedPointFlag::shuntSignal | SpeedPointFlag::outsideStation) ) {
                    return;
                }
            }
        }

        auto const &lastspeedpoint = sSpeedTable[ iLast ];
        pTrack = lastspeedpoint.trTrack;
        assert( pTrack != nullptr );
        // flaga ustawiona, gdy Point2 toru jest blizej
        fLastDir = lastspeedpoint.flags.HasAny(SpeedPointFlag::reverse) ? -1.0 : 1.0;
        fCurrentDistance = lastspeedpoint.fDist; // aktualna odleglosc do jego Point1
        fTrackLength = pTrack->Length();
    }

    if( iTableDirection == 0 ) {
        // don't bother
        return;
    }

    while (fCurrentDistance < fDistance)
    {
        if (pTrack != tLast) // ostatni zapisany w tabelce nie był jeszcze sprawdzony
        { // jeśli tor nie był jeszcze sprawdzany
            if( Global.iWriteLogEnabled & 8 ) {
                WriteLog( "Speed table for " + OwnerName() + " tracing through track " + pTrack->name() );
            }

            auto const events { CheckTrackEvent( pTrack, fLastDir ) };
            for( auto *pEvent : events ) {
                if( pEvent != nullptr && TableNotFound(pEvent, fCurrentDistance) ) // jeśli jest semafor na tym torze
                { // trzeba sprawdzić tabelkę, bo dodawanie drugi raz tego samego przystanku nie jest korzystne
					// jeśli nie ma
					TableAddNew(); // zawsze jest true

					if (Global.iWriteLogEnabled & 8)
					{
						WriteLog("Speed table for " + OwnerName() + " found new event, " + pEvent->m_name);
					}
					auto &newspeedpoint = sSpeedTable[iLast];
					/*
					                        if( newspeedpoint.Set(
					                            pEvent,
					                            fCurrentDistance + ProjectEventOnTrack( pEvent, pTrack, fLastDir ),
					                            OrderCurrentGet() ) ) {
					*/
					// an order of a road junction or a level crossing lies on the lane it's on. it's measured along that lane: the search
					// for the events beside the track goes in steps of about a metre, and misplaces the ones on lanes shorter than a few metres
					auto const eventdistance{
						typeid( *pEvent ) == typeid( road_order ) ?
							fCurrentDistance + ProjectEventOnTrack( pEvent, pTrack, fLastDir ) :
							GetDistanceToEvent( pTrack, pEvent, fLastDir, fCurrentDistance ) };
					if (newspeedpoint.Set(pEvent, eventdistance, fLength, OrderCurrentGet(), mvOccupied->Vel < EU07_AI_NOMOVEMENT))
					{

						fDistance = newspeedpoint.fDist; // jeśli sygnał stop, to nie ma potrzeby dalej skanować
						SemNextStopIndex = iLast;
						if (SemNextIndex == npos)
						{
							SemNextIndex = iLast;
						}
						if (Global.iWriteLogEnabled & 8)
						{
							WriteLog("(stop signal from " + (SemNextStopIndex != npos ? sSpeedTable[SemNextStopIndex].GetName() : "unknown semaphor") + ")");
						}
					}
					else
					{
						if (true == newspeedpoint.IsProperSemaphor(OrderCurrentGet()) && SemNextIndex == npos)
						{
							SemNextIndex = iLast; // sprawdzamy czy pierwszy na drodze
						}
						if (Global.iWriteLogEnabled & 8)
						{
							WriteLog("(forward signal for " + (SemNextIndex != npos ? sSpeedTable[SemNextIndex].GetName() : "unknown semaphor") + ")");
						}
					}
				}
            } // event dodajemy najpierw, żeby móc sprawdzić, czy tor został dodany po odczytaniu prędkości następnego

            if( pTrack->VelocityGet() == 0.0 // zatrzymanie
             || pTrack->iAction // jeśli tor ma własności istotne dla skanowania
             || pTrack->VelocityGet() != fLastVel ) // następuje zmiana prędkości
            { // odcinek dodajemy do tabelki, gdy jest istotny dla ruchu
                TableAddNew();
                sSpeedTable[ iLast ].Set(
                    pTrack, fCurrentDistance,
                    fLastDir < 0 ? SpeedPointFlag::enabled | SpeedPointFlag::reverse : SpeedPointFlag::enabled ); // dodanie odcinka do tabelki z flagą kierunku wejścia
                if (pTrack->eType == tt_Cross) {
                    // na skrzyżowaniach trzeba wybrać segment, po którym pojedzie pojazd
                    // dopiero tutaj jest ustalany kierunek segmentu na skrzyżowaniu
/*
                    // NOTE: manual selection disabled due to multiplayer
                    int routewanted;
                    if( false == AIControllFlag ) {
                        routewanted = (
                            input::keys[ GLFW_KEY_LEFT ] != GLFW_RELEASE ? 1 :
                            input::keys[ GLFW_KEY_RIGHT ] != GLFW_RELEASE ? 2 :
                            3 );
                    }
                    else {
                        routewanted = 1 + std::floor( Random( static_cast<double>( pTrack->RouteCount() ) - 0.001 ) );
                    }
*/
                    auto const routewanted { pTrack->RouteDraw() };
                    sSpeedTable[iLast].crossRoute = signedNibble( ( static_cast<unsigned>( sSpeedTable[iLast].crossRoute ) | static_cast<unsigned>( pTrack->CrossSegment(fLastDir < 0 ? tLast->iPrevDirection : tLast->iNextDirection,
					                                                   /*
					                                                                                   iRouteWanted )
					                                                   */
					                                                   routewanted) ) ) & 0xf ); // ostatnie 4 bity pola flag
                    sSpeedTable[iLast].flags = sSpeedTable[iLast].flags.Without(SpeedPointFlag::reverse); // usunięcie flagi kierunku, bo może być błędna
                    if (sSpeedTable[iLast].crossRoute < 0) {
                        sSpeedTable[iLast].flags |= SpeedPointFlag::reverse; // ustawienie flagi kierunku na podstawie wybranego segmentu
                    }
                    if (int(fLastDir) * sSpeedTable[iLast].LegacyFlagsValue() < 0) {
                        fLastDir = -fLastDir;
                    }
/*
                    if (AIControllFlag) {
                        // dla AI na razie losujemy kierunek na kolejnym skrzyżowaniu
                        iRouteWanted = 1 + Random(3);
                    }
*/
                }
            }
            else if ( (pTrack->fRadius != 0.0 // odległość na łuku lepiej aproksymować cięciwami
			          || tLast != nullptr && tLast->fRadius != 0.0) &&
			         TableAddNew()) // koniec łuku też jest istotny
            { // albo dla liczenia odległości przy pomocy cięciw - te usuwać po przejechaniu
			  // dodanie odcinka do tabelki
				sSpeedTable[iLast].Set(pTrack, fCurrentDistance, fLastDir < 0 ? SpeedPointFlag::enabled | SpeedPointFlag::curve | SpeedPointFlag::reverse : SpeedPointFlag::enabled | SpeedPointFlag::curve);
			}
        }

        fCurrentDistance += fTrackLength; // doliczenie kolejnego odcinka do przeskanowanej długości
        tLast = pTrack; // odhaczenie, że sprawdzony
        fLastVel = pTrack->VelocityGet(); // prędkość na poprzednio sprawdzonym odcinku
        pTrack = pTrack->Connected(
            pTrack->eType == tt_Cross ? sSpeedTable[iLast].crossRoute : static_cast<int>(fLastDir),
            fLastDir); // może być NULL

        if (pTrack != nullptr )
        { // jeśli kolejny istnieje
            if( tLast != nullptr && tLast->VelocityGet() > 0 && (tLast->VelocityGet() < pTrack->VelocityGet() || pTrack->VelocityGet() < 0) ) {

				if (iLast != -1 && sSpeedTable[iLast].trTrack == tLast)
				{
					// if the track is already in the table we only need to mark it as relevant
					sSpeedTable[iLast].flags |= SpeedPointFlag::enabled;
				}
				else
				{
					// otherwise add it
					TableAddNew();
					sSpeedTable[iLast].Set(tLast,
					                       fCurrentDistance - fTrackLength, // by now the current distance points to beginning of next track
					                       (fLastDir > 0 ? pTrack->iPrevDirection : pTrack->iNextDirection) ? SpeedPointFlag::enabled : SpeedPointFlag::enabled | SpeedPointFlag::reverse);
				}
			}
            // zwiększenie skanowanej odległości tylko jeśli istnieje dalszy tor
            fTrackLength = pTrack->Length();
        }
        else
        { // definitywny koniec skanowania, chyba że dalej puszczamy samochód po gruncie...
            if( iLast == -1
             || ( false == sSpeedTable[iLast].flags.HasAll(SpeedPointFlag::enabled | SpeedPointFlag::trackEnd)
               && sSpeedTable[iLast].trTrack != tLast ) ) {
                // only if we haven't already marked end of the track and if the new track doesn't duplicate last one
                if( TableAddNew() ) {
                    // zapisanie ostatniego sprawdzonego toru
                    sSpeedTable[iLast].Set(
                        tLast, fCurrentDistance - fTrackLength, // by now the current distance points to beginning of next track,
                        fLastDir < 0 ? SpeedPointFlag::enabled | SpeedPointFlag::trackEnd | SpeedPointFlag::reverse : SpeedPointFlag::enabled | SpeedPointFlag::trackEnd);
                }
            }
            else if( sSpeedTable[iLast].trTrack == tLast && false == TestFlag(sSpeedTable[iLast].trTrack->iCategoryFlag, 0x100) ) {
                // otherwise just mark the last added track as the final one
                // TODO: investigate exactly how we can wind up not marking the last existing track as actual end
				// don't mark portals, as these aren't exactly track ends, but teleport devices
				sSpeedTable[iLast].flags |= SpeedPointFlag::enabled | SpeedPointFlag::trackEnd;
			}
            // to ostatnia pozycja, bo NULL nic nie da, a może się podpiąć obrotnica, czy jakieś transportery
            return;
        }
    }
    if( TableAddNew() ) {
        // zapisanie ostatniego sprawdzonego toru
        sSpeedTable[ iLast ].Set(
            pTrack, fCurrentDistance,
            fLastDir < 0 ? SpeedPointFlag::none | SpeedPointFlag::reverse : SpeedPointFlag::none );
    }
};

void TController::TableCheck(double fDistance)
{ // przeliczenie odległości w tabelce, ewentualnie doskanowanie (bez analizy prędkości itp.)
    if( iTableDirection != iDirection ) {
        // jak zmiana kierunku, to skanujemy od końca składu
        TableTraceRoute( fDistance, pVehicles[ end::rear ] );
        TableSort();
    }
    else if (iTableDirection)
    { // trzeba sprawdzić, czy coś się zmieniło
        auto const distance = MoveDistanceGet();
        for (auto &sp : sSpeedTable) {
            // aktualizacja odległości dla wszystkich pozycji tabeli
            sp.UpdateDistance(distance);
        }
        MoveDistanceReset(); // kasowanie odległości po aktualizacji tabelki
        for( int i = 0; i < iLast; ++i )
        { // aktualizacja rekordów z wyjątkiem ostatniego
            if (sSpeedTable[i].flags & SpeedPointFlag::enabled) // jeśli pozycja istotna
            {
                auto const passedbefore { sSpeedTable[ i ].flags.HasAll(SpeedPointFlag::elapsed) };
                if (sSpeedTable[i].Update())
                {
                    if( Global.iWriteLogEnabled & 8 ) {
                        WriteLog( "Speed table for " + OwnerName() + " detected switch change at " + sSpeedTable[ i ].trTrack->name() + " (generating fresh trace)" );
                    }
                    // usuwamy wszystko za tym torem
                    while (sSpeedTable.back().trTrack != sSpeedTable[i].trTrack)
                    { // usuwamy wszystko dopóki nie trafimy na tą zwrotnicę
                        sSpeedTable.pop_back();
                        --iLast;
                    }
                    // a semaphor recorded beyond the switch went away with the discarded entries
                    if( SemNextIndex != npos
                     && SemNextIndex >= sSpeedTable.size() ) {
                        SemNextIndex = npos;
                    }
                    if( SemNextStopIndex != npos
                     && SemNextStopIndex >= sSpeedTable.size() ) {
                        SemNextStopIndex = npos;
                    }
                    tLast = sSpeedTable[ i ].trTrack;
                    TableTraceRoute( fDistance, pVehicles[ end::rear ] );
                    TableSort();
                    // nie kontynuujemy pętli, trzeba doskanować ciąg dalszy
                    break;
                }
                // a switch is a track, the update of a signal never ends the loop above
                CheckSignalPassed( sSpeedTable[ i ], passedbefore );
                if (sSpeedTable[i].flags & SpeedPointFlag::track) // jeśli odcinek
                {
                    if( sSpeedTable[ i ].fDist + sSpeedTable[ i ].trTrack->Length() < -fLength ) // a skład wyjechał całą długością poza
                    { // degradacja pozycji
						sSpeedTable[i].flags = sSpeedTable[i].flags.Without(SpeedPointFlag::enabled); // nie liczy się
                    }
                    else if( sSpeedTable[i].crossRoute == 0 && ( sSpeedTable[i].flags & ( SpeedPointFlag::elapsed | SpeedPointFlag::switchTrack ) ) == SpeedPointFlag::elapsed && sSpeedTable[i].fVelNext < 0 ) {
                        // jest z tyłu (najechany) i nie jest zwrotnicą ani skrzyżowaniem
						// a nie ma ograniczenia prędkości
						sSpeedTable[i].ResetFlags(); // to nie ma go po co trzymać (odtykacz usunie ze środka)
					}
                }
                else if (sSpeedTable[i].flags & SpeedPointFlag::event && false == sSpeedTable[i].flags.HasAny(SpeedPointFlag::pointSpeed) && sSpeedTable[i].fDist < (typeid(*sSpeedTable[i].evEvent) == typeid(putvalues_event) ? -fLength : 0) && mvOccupied->CategoryFlag == 2 &&
				         sSpeedTable[i].fDist < -0.75) // jeśli event
                {
					// jeśli jest z tyłu
					// pociąg staje zawsze, a samochód tylko jeśli nie przejedzie całą długością (może być zaskoczony zmianą)
					// WriteLog("TableCheck: Event is behind. Delete from table: " + sSpeedTable[i].evEvent->asName);
					sSpeedTable[i].flags = sSpeedTable[i].flags.Without(SpeedPointFlag::enabled); // degradacja pozycji dla samochodu;
					// semafory usuwane tylko przy sprawdzaniu, bo wysyłają komendy
				}
            }
        }
        auto const lastpassedbefore { sSpeedTable[ iLast ].flags.HasAll(SpeedPointFlag::elapsed) };
        sSpeedTable[iLast].Update(); // aktualizacja ostatniego
        // a signal at stop often is the last, the scan doesn't go past it
        CheckSignalPassed( sSpeedTable[ iLast ], lastpassedbefore );
        // WriteLog("TableCheck: Upate last track. Dist=" + AnsiString(sSpeedTable[iLast].fDist));
        if( sSpeedTable[ iLast ].fDist < fDistance ) {
            TableTraceRoute( fDistance, pVehicles[ end::rear ] ); // doskanowanie dalszego odcinka
            TableSort();
        }
        // garbage collection
        TablePurger();
    }
};
namespace
{


// the position is a signal showing stop (SetVelocity 0 0). SetVelocity 0 x on automatic block isn't: it may be passed after a stop
bool is_signal_at_stop( TSpeedPos const &Point )
{
    return Point.flags.HasAll(SpeedPointFlag::event)
        && Point.flags.HasAll(SpeedPointFlag::signal)
        && false == Point.flags.HasAll(SpeedPointFlag::stopOnAutomaticBlock)
        && std::abs( Point.fVelNext ) <= 0.0
        && Point.evEvent != nullptr;
}

// the position is a signal, a main or a shunting one: the aspect comes from its memory cell. putvalues events
// set speed limits by the track, and the flags they get don't make them signals
bool is_signal( TSpeedPos const &Point )
{
    return Point.flags.HasAll(SpeedPointFlag::event)
        && ( Point.flags.HasAll(SpeedPointFlag::signal) || Point.flags.HasAll(SpeedPointFlag::shuntSignal) )
        && Point.evEvent != nullptr
        && typeid( *Point.evEvent ) == typeid( getvalues_event );
}

std::uint8_t passing_flags_of( TSpeedPos const &Point, double const Speedatsignal, bool const Trainmovement )
{
    std::uint8_t flags { 0 };
    auto const shunting { Point.flags.HasAll(SpeedPointFlag::shuntSignal) };
    if( shunting ) {
        flags |= blackbox::passing_flags::shunt;
    }
    // the speeds of the aspects are whole km/h, -1 for no limit; a shunting signal binds only the shunting
    if( std::abs( Speedatsignal ) < 0.5 && ( false == shunting || false == Trainmovement ) ) {
        flags |= ( Point.flags.HasAll(SpeedPointFlag::stopOnAutomaticBlock) ? blackbox::passing_flags::automatic_block : blackbox::passing_flags::at_stop );
    }
    return flags;
}

} // namespace

void TController::CheckSignalPassed( TSpeedPos const &Point, bool const Passedbefore ) const
{
    // not bSeenAhead: found already behind the head, the train stood across it or past it, it didn't pass it
    if( false == signal_passing::passed_now( Passedbefore, Point.flags.HasAll(SpeedPointFlag::elapsed), Point.bSeenAhead )
     || ( mvOccupied->CategoryFlag & 1 ) == 0 ) { // railway only
        return;
    }
    if( is_signal_at_stop( Point ) ) {
        simulation::test().signal_passed_at_danger( OwnerName(), Point.evEvent->m_name );
    }
    if( is_signal( Point ) ) {
        // the aspect the train had of it: the table keeps it once the signal is behind, while the memory cell
        // may already show stop for the train itself, past the signal on the track it guards. the stop of an
        // automatic block signal is kept as the speed allowed after it, with a flag of its own
        auto const speedatsignal { Point.flags.HasAll(SpeedPointFlag::stopOnAutomaticBlock) ? 0.0 : Point.fVelNext };
        simulation::Blackbox.signal_passed(
            pVehicles[ end::front ], *Point.evEvent,
            speedatsignal, Point.evEvent->input_value( 2 ),
            mvOccupied->Vel, -Point.fDist,
            passing_flags_of( Point, speedatsignal, false == OrderCurrentGet().HasAny( DriverOrder::shunt | DriverOrder::looseShunt ) ) );
    }
}

// the margins of the final approach of a train [m], closer than the first of which it stops at once and within the
// second of which it makes its final approach: at a passenger stop, or at a signal and anything else it stops at
std::pair<double, double>
TController::train_proximity_margins( bool const Passengerstop ) const {

    auto minproximity { 0.0 };
    auto maxproximity { 0.0 };
    if( Passengerstop ) {
        // a stop point keeps the margins its W4 parameters (head or middle of the consist,
        // platform length) were set up against
        minproximity = std::clamp(  5 + iVehicles, 10, 15 ) + ( IsCargoTrain ? 10.0 : 0.0 );
        maxproximity = std::clamp( 10 + iVehicles, 15, 40 );
    }
    else {
        // a train stops 8-15 m short of the signal whatever its length; closer than the lower
        // edge it stops at once, the braking aims at the middle of the window, 3.5 m further out
        minproximity = 8.0;
        // the final approach begins far enough out to slow from the coasting speed down to the window,
        // and at the same distance whether the train is moving or standing, so it does not flip
        // between creeping closer and holding still when it comes to a stand short of the window
        maxproximity = 50.0;
    }
    if( IsCargoTrain ) {
        // start the final approach earlier for cargo trains, their brakes react more slowly
        maxproximity += 10.0;
    }
    if( ( Global.FrictionWeatherFactor < 1.f )
     && ( iVehicles > 1 ) ) {
        // take into account weather conditions
        minproximity += 5.0;
        maxproximity += 5.0;
    }
    if( mvOccupied->Vel < 0.1 ) {
        // jak stanie za daleko, to niech nie dociąga paru metrów
        maxproximity = std::max( maxproximity, 50.0 );
    }
    return { minproximity, maxproximity };
}
