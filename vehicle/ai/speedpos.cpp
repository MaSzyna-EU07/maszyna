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

// a position of the speed table: a track, a switch or an event ahead and the speed it asks for

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


TSpeedPos::TSpeedPos(TTrack *track, double dist, SpeedPointFlags flag)
{
    Set(track, dist, flag);
};

TSpeedPos::TSpeedPos(basic_event *event, double dist, double length, DriverOrders order)
{
    Set(event, dist, length, order);
};

void TSpeedPos::Clear()
{
    ResetFlags(); // brak flag to brak reakcji
    fVelNext = -1.0; // prędkość bez ograniczeń
	fSectionVelocityDist = 0.0; //brak długości
    fPointDist = 0.0;
    fDist = 0.0;
    bSeenAhead = false;
    vPos = glm::dvec3(0, 0, 0);
    trTrack = nullptr; // brak wskaźnika
};

void TSpeedPos::CommandCheck()
{ // sprawdzenie typu komendy w evencie i określenie prędkości
    TCommandType command = evEvent->input_command();
    double value1 = evEvent->input_value(1);
    double value2 = evEvent->input_value(2);
    // the point of a PointVelocity limit is taken anew from the current command
    flags = flags.Without(SpeedPointFlag::pointSpeed);
    fPointDist = 0.0;
    switch (command)
    {
    case TCommandType::cm_ShuntVelocity:
        // prędkość manewrową zapisać, najwyżej AI zignoruje przy analizie tabelki
        fVelNext = value1; // powinno być value2, bo druga określa "za"?
        flags |= SpeedPointFlag::shuntSignal;
        break;
    case TCommandType::cm_SetVelocity:
        // w semaforze typu "m" jest ShuntVelocity dla Ms2 i SetVelocity dla S1
        // SetVelocity * 0    -> można jechać, ale stanąć przed
        // SetVelocity 0 20   -> stanąć przed, potem można jechać 20 (SBL)
        // SetVelocity -1 100 -> można jechać, przy następnym ograniczenie (SBL)
        // SetVelocity 40 -1  -> PutValues: jechać 40 aż do minięcia (koniec ograniczenia(
        fVelNext = value1;
        flags = flags.Without((SpeedPointFlag::shuntSignal | SpeedPointFlag::passengerStop | SpeedPointFlag::stopOnAutomaticBlock));
        flags |= SpeedPointFlag::signal;// nie manewrowa, nie przystanek, nie zatrzymać na SBL, ale semafor
        if (value1 == 0.0 && value2 != 0.0) // jeśli pierwsza zerowa
		// a druga nie
		{ // S1 na SBL, można przejechać po zatrzymaniu (tu nie mamy prędkości ani odległości)
			fVelNext = value2; // normalnie będzie zezwolenie na jazdę, aby się usunął z tabelki
			flags |= SpeedPointFlag::stopOnAutomaticBlock; // flaga, że ma zatrzymać; na pewno nie zezwoli na manewry
		}
		break;
    case TCommandType::cm_SectionVelocity:
        // odcinek z ograniczeniem prędkości
        fVelNext = value1;
        fSectionVelocityDist = value2;
        flags |= SpeedPointFlag::sectionSpeed;
        break;
    case TCommandType::cm_RoadVelocity:
        // prędkość drogowa (od tej pory będzie jako domyślna najwyższa)
        fVelNext = value1;
        flags |= SpeedPointFlag::roadSpeed;
        break;
    case TCommandType::cm_PassengerStopPoint:
        // nie ma dostępu do rozkładu
        // przystanek, najwyżej AI zignoruje przy analizie tabelki
        fVelNext = 0.0;
        flags |= SpeedPointFlag::passengerStop; // niestety nie da się w tym miejscu współpracować z rozkładem
/*
        // NOTE: not used for now as it might be unnecessary
        // special case, potentially override any set speed limits if requested
        // NOTE: we test it here because for the time being it's only used for passenger stops
        if( flags.HasAll(spDontApplySpeedLimit) ) {
            fVelNext = -1;
        }
*/
        break;
    case TCommandType::cm_SetProximityVelocity:
        // musi zostać gdyż inaczej nie działają manewry
        fVelNext = -1;
        flags |= SpeedPointFlag::proximitySpeed;
        // fSectionVelocityDist = value2;
        break;
    case TCommandType::cm_PointVelocity:
        // PointVelocity v d: the head has to be at v (-1: no limit) at the point d metres beyond the cell,
        // the limit ends when the head passes that point. the cell is read until the head passes the cell,
        // what it says then holds for the point (the warning disc of a level crossing, Osp1/Osp2)
        fVelNext = value1;
        fPointDist = std::max( value2, 0.0 );
        flags = flags.Without((SpeedPointFlag::shuntSignal | SpeedPointFlag::passengerStop | SpeedPointFlag::stopOnAutomaticBlock | SpeedPointFlag::signal)); // not a signal, even if the cell was one before
        flags |= SpeedPointFlag::pointSpeed;
        break;
    case TCommandType::cm_OutsideStation:
        // w trybie manewrowym: skanować od niej wstecz i stanąć po wyjechaniu za sygnalizator i
        // zmienić kierunek
        // w trybie pociągowym: można przyspieszyć do wskazanej prędkości (po zjechaniu z rozjazdów)
        fVelNext = -1;
        flags |= SpeedPointFlag::outsideStation; // W5
        break;
    case TCommandType::cm_EmergencyBrake:
        fVelNext = -1;
        break;
    case TCommandType::cm_SecuritySystemMagnet:
        fVelNext = -1;
        break;
    case TCommandType::cm_SignalAreaEnd:
        // imposes no speed of its own, it only ends the reduced speed of the signal passed last
        fVelNext = -1;
        flags |= SpeedPointFlag::signalAreaEnd;
        break;
    default:
        // inna komenda w evencie skanowanym powoduje zatrzymanie i wysłanie tej komendy
        // nie manewrowa, nie przystanek, nie zatrzymać na SBL
        // jak nieznana komenda w komórce sygnałowej, to zatrzymujemy
        fVelNext = 0.0;
/*
        fVelNext = (
            evEvent->is_command() ? 0.0 : // ask for a stop if we have a command for the vehicle
            flags.HasAny(( spSemaphor | spShuntSemaphor )) ? fVelNext : // don't change semafor velocity
            -1.0 ); // otherwise don't be a bother
*/
        // TODO: check whether clearing spShuntSemaphor flag doesn't cause problems
        // potentially it can invalidate shunt semaphor used to transmit timetable or similar command
        flags = flags.Without((SpeedPointFlag::shuntSignal | SpeedPointFlag::passengerStop | SpeedPointFlag::stopOnAutomaticBlock));
    }
};

bool TSpeedPos::Update()
{
    if( fDist < 0.0 ) {
        // trzeba zazanaczyć, że minięty
        flags |= SpeedPointFlag::elapsed;
    }
    bSeenAhead = signal_passing::seen_ahead_on_update( fDist, bSeenAhead );
    if (flags & SpeedPointFlag::track) {
        // road/track
        if (trTrack) {
            // może być NULL, jeśli koniec toru (???)
            fVelNext = trTrack->VelocityGet(); // aktualizacja prędkości (może być zmieniana eventem)

            if( trTrack->iCategoryFlag & 1 ) {
                // railways
                if( flags & SpeedPointFlag::switchTrack && (trTrack->GetSwitchState() & 1) != 0 != (flags.HasAny(SpeedPointFlag::switchState)) ) {
                    // jeśli odcinek zmienny
					// czy stan się zmienił?
					// Ra: zakładam, że są tylko 2 możliwe stany
					flags ^= SpeedPointFlag::switchState;
					if (false == flags.HasAny(SpeedPointFlag::elapsed))
					{
						// jeszcze trzeba skanowanie wykonać od tego toru
						// problem jest chyba, jeśli zwrotnica się przełoży zaraz po zjechaniu z niej
						// na Mydelniczce potrafi skanować na wprost mimo pojechania na bok
						return true;
					}
				}
            }
            else {
                // roads and others
                if( crossRoute != 0 && crossRoute != 1 ) { // as the top 4 bits read without the sign: -1 counts as turning
                    // jeśli skrzyżowanie, ograniczyć prędkość przy skręcaniu
					// ±1 to jazda na wprost, ±2 nieby też, ale z przecięciem głównej drogi - chyba że jest równorzędne...
					// TODO: uzależnić prędkość od promienia;
					// albo niech będzie ograniczona w skrzyżowaniu (velocity z ujemną wartością)
					fVelNext = 30.0;
				}
            }
        }
    }
    else if (flags & SpeedPointFlag::event && (false == flags.HasAny(SpeedPointFlag::elapsed) || fVelNext == 0.0)) {
        // jeśli event
		// ignore already passed signals, but keep an eye on overrun stops
		// (a PointVelocity cell is latched when the head passes it, except at 0, which has to be released by the cell)
		// odczyt komórki pamięci najlepiej by było zrobić jako notyfikację,
		// czyli zmiana komórki wywoła jakąś podaną funkcję
		CommandCheck(); // sprawdzenie typu komendy w evencie i określenie prędkości
	}
    return false;
};

std::string TSpeedPos::GetName() const
{
    if( flags & SpeedPointFlag::track ) // jeśli tor
        return trTrack->name();
    else if( flags & SpeedPointFlag::event ) // jeśli event
        return
            evEvent->m_name
            + " [" + std::to_string( static_cast<int>( evEvent->input_value( 1 ) ) )
            + ", " + std::to_string( static_cast<int>( evEvent->input_value( 2 ) ) )
            + "]";
    else
        return "";
}

std::string TSpeedPos::TableText() const
{ // pozycja tabelki pr?dko?ci
    if (flags & SpeedPointFlag::enabled)
    { // o ile pozycja istotna
		return to_hex_str(LegacyFlagsValue(), 8) + "   " + to_string(fDist, 1, 6) +
               "   " + (fVelNext == -1.0 ? "  -" : to_string(static_cast<int>(fVelNext), 0, 3)) + "   " + GetName();
    }
    return "Empty";
}

bool TSpeedPos::IsProperSemaphor(DriverOrders order)
{ // sprawdzenie czy semafor jest zgodny z trybem jazdy
	if (false == order.HasAny(trainOrders)) // Wait_for_orders, Prepare_engine, Change_direction, Connect, Disconnect, Shunt
    {
        if (flags & (SpeedPointFlag::signal | SpeedPointFlag::shuntSignal))
            return true;
        else if (flags & SpeedPointFlag::outsideStation)
            return true;
    }
    else if (order & DriverOrder::obeyTrain && flags & SpeedPointFlag::signal)
    {
		return true;
	}
	return false; // true gdy zatrzymanie, wtedy nie ma po co skanować dalej
}

bool TSpeedPos::Set(basic_event *event, double dist, double length, DriverOrders order, bool const Standing)
{ // zapamiętanie zdarzenia
    fDist = dist;
    // an event already behind the head when found (under a train set across a signal) wasn't passed by it. a moving
    // train may be just passing it, and the distance is found in steps of a metre, so for a moving train an event a
    // little behind the head still counts as seen ahead. a standing train passes nothing: an event found behind its
    // head counts only once an update finds it ahead
    bSeenAhead = signal_passing::seen_ahead_when_found( dist, Standing );
    flags = SpeedPointFlag::event;
    crossRoute = 0;
    evEvent = event;
    vPos = event->input_location(); // współrzędne eventu albo komórki pamięci (zrzutować na tor?)
    // ignore events located behind the consist, but with exception of stop points which may be needed to update freshly received timetable
    if( dist + length < 0
     && event->input_command() != TCommandType::cm_PassengerStopPoint ) {
        return false;
    }
    flags |= SpeedPointFlag::enabled;
    CommandCheck(); // sprawdzenie typu komendy w evencie i określenie prędkości
	// zależnie od trybu sprawdzenie czy jest tutaj gdzieś semafor lub tarcza manewrowa
	// jeśli wskazuje stop wtedy wystawiamy true jako koniec sprawdzania
	// WriteLog("EventSet: Vel=" + AnsiString(fVelNext) + " flags=" + AnsiString(flags) + " order="+AnsiString(order));
	if (false == order.HasAny(trainOrders)) // Wait_for_orders, Prepare_engine, Change_direction, Connect, Disconnect, Shunt
    {
        if (flags & (SpeedPointFlag::signal | SpeedPointFlag::shuntSignal) && fVelNext == 0.0)
            return true;
        else if (flags & SpeedPointFlag::outsideStation)
            return true;
    }
    else if (order & DriverOrder::obeyTrain && flags & SpeedPointFlag::signal && fVelNext == 0.0)
    {
		return true;
	}
    return false; // true gdy zatrzymanie, wtedy nie ma po co skanować dalej
};

void TSpeedPos::Set(TTrack *track, double dist, SpeedPointFlags flag)
{ // zapamiętanie zmiany prędkości w torze
    fDist = dist; // odległość do początku toru
    trTrack = track; // TODO: (t) może być NULL i nie odczytamy końca poprzedniego :/
    if (trTrack)
    {
        crossRoute = 0;
        flags = flag | (trTrack->eType == tt_Normal ? SpeedPointFlag::track : SpeedPointFlag::track | SpeedPointFlag::switchTrack ); // zapamiętanie kierunku wraz z typem
        if (flags & SpeedPointFlag::switchTrack && trTrack->GetSwitchState() & 1)
			flags |= SpeedPointFlag::switchState;
		fVelNext = trTrack->VelocityGet();
        if (trTrack->iDamageFlag & 128)
            fVelNext = 0.0; // jeśli uszkodzony, to też stój
        if (flags & SpeedPointFlag::trackEnd)
            fVelNext = trTrack->iCategoryFlag & 1 ?
                           0.0 :
                           20.0; // jeśli koniec, to pociąg stój, a samochód zwolnij
        vPos =
            flags & SpeedPointFlag::reverse ?
                trTrack->CurrentSegment()->FastGetPoint_1() :
                trTrack->CurrentSegment()->FastGetPoint_0();
    }
};
