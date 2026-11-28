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

// the speed table: what each position of it asks of the driver, the stop points, the speed limits and the signals

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

bool WriteTrainLogFlag = false;
namespace
{

void clear_signal_area_end( TSpeedPos &Point )
{
    if( Point.flags.HasAll(SpeedPointFlag::signalAreaEnd) ) {
        Point.Clear();
    }
}

} // namespace

auto const passengerstopmaxdistance { 400.0 }; // maximum allowed distance between passenger stop point and consist head

TCommandType TController::TableUpdate(double &fVelDes, double &fDist, double &fNext, double &fAcc)
{ // ustalenie parametrów, zwraca typ komendy, jeśli sygnał podaje prędkość do jazdy
    // fVelDes - prędkość zadana
    // fDist - dystans w jakim należy rozważyć ruch
    // fNext - prędkość na końcu tego dystansu
    // fAcc - zalecane przyspieszenie w chwili obecnej - kryterium wyboru dystansu
    auto a { 0.0 }; // przyspieszenie
    auto v { 0.0 }; // prędkość
    auto d { 0.0 }; // droga
    auto d_to_next_sem { 10000.0 }; //ustaiwamy na pewno dalej niż widzi AI
    auto go { TCommandType::cm_Unknown };
    auto speedlimitiscontinuous { true }; // stays true if potential active speed limit is unbroken to the last (relevant) point in scan table
    eSignNext = nullptr;
    IsAtPassengerStop = false;
    bStopTargetIsPassengerStop = false;
    IsScheduledPassengerStopVisible = false;
    mvOccupied->SecuritySystem.set_cabsignal_lock(false);
    // te flagi są ustawiane tutaj, w razie potrzeby
    drivingFlags.trackEnd = false;
    drivingFlags.switchFound = false;
    drivingFlags.signalFound = false;
    drivingFlags.stopPointFound = false;
    SignalAreaEndDist = -1.0;
    update_point_limits();

    for( std::size_t idx = 0; idx < sSpeedTable.size(); ++idx ) {
        // o ile dana pozycja tabelki jest istotna
        if( false == sSpeedTable[ idx ].flags.HasAny(SpeedPointFlag::enabled) ) { continue; }

        auto &point { sSpeedTable[ idx ] };

        if( point.flags.HasAll(SpeedPointFlag::passengerStop) ) {
            if( TableUpdateStopPoint( go, point, d_to_next_sem ) ) { continue; };
        }
        v = point.fVelNext; // odczyt prędkości do zmiennej pomocniczej
        if( point.flags.HasAll(SpeedPointFlag::switchTrack) ) {
            // zwrotnice są usuwane z tabelki dopiero po zjechaniu z nich
            drivingFlags.switchFound = true; // rozjazd z przodu/pod ogranicza np. sens skanowania wstecz
            SwitchClearDist = point.fDist + point.trTrack->Length() + fLength;
        }
        else if ( point.flags.HasAll(SpeedPointFlag::event) && TableUpdateEvent(v, go, point, d_to_next_sem, idx) ) // W4 może się deaktywować
        { // jeżeli event, może być potrzeba wysłania komendy, aby ruszył
			continue;
		}

        auto const railwaytrackend { true == point.flags.HasAll(SpeedPointFlag::trackEnd) && is_train() };
        if( v >= 0.0 // pozycje z prędkością -1 można spokojnie pomijać
         || railwaytrackend ) {

            // a PointVelocity limit lies fPointDist beyond its entry and stays ahead of us until the head passes it
            auto const ispointlimit { point.flags.HasAll(SpeedPointFlag::pointSpeed) };
            d = point.fDist + point.fPointDist;

            if( v >= 0.0 ) {
                // points located in front of us can potentially affect our acceleration and target speed
                if( d > 0.0
                 && ( false == point.flags.HasAll(SpeedPointFlag::elapsed) || ispointlimit ) ) {
                    // sygnał lub ograniczenie z przodu (+32=przejechane)
                    // 2014-02: jeśli stoi, a ma do przejechania kawałek, to niech jedzie
                    if( mvOccupied->Vel < 0.01
                     && true == point.flags.HasAll(SpeedPointFlag::enabled | SpeedPointFlag::event | SpeedPointFlag::passengerStop)
                     && false == IsAtPassengerStop ) {
                        // ma podjechać bliżej - czy na pewno w tym miejscu taki warunek?
                        a = d > passengerstopmaxdistance || drivingFlags.stopCloser ? fAcc : 0.0;
                    }
                    else {
                        // przyspieszenie: ujemne, gdy trzeba hamować
                        if( v >= 0.0 ) {
                            a = ( v * v - mvOccupied->Vel * mvOccupied->Vel ) / ( 25.92 * d );
                            if( mvOccupied->Vel < v
                             || v == 0.0 ) {
                                // if we're going slower than the target velocity and there's enough room for safe stop, speed up
                                auto const brakingdistance { 1.2 * fBrakeDist * braking_distance_multiplier( v ) };
                                if( brakingdistance > 0.0 ) {
                                    // maintain desired acc while we have enough room to brake safely, when close enough start paying attention
                                    // try to make a smooth transition instead of sharp change
                                    a = std::lerp( a, AccPreferred, std::clamp( ( d - brakingdistance ) / brakingdistance, 0.0, 1.0 ) );
                                }
                            }
                            if( d < ( ispointlimit && v > 0.0 ? point_limit_approach() : fMinProximityDist )
                             && v < fVelDes ) {
                                // jak jest już blisko, ograniczenie aktualnej prędkości
                                fVelDes = v;
                            }
                        }
                    }
                }
                // points located behind can affect our current speed, but little else
                else if ( point.flags & SpeedPointFlag::track) // jeśli tor
                { // tor ogranicza prędkość, dopóki cały skład nie przejedzie,
                    if( v >= 1.0 && d + point.trTrack->Length() < -fLength && false == railwaytrackend ) { // EU06 się zawieszało po dojechaniu na koniec toru postojowego
						// zapętlenie, jeśli już wyjechał za ten odcinek
						continue;
					}
                    if( v < fVelDes ) {
                        // ograniczenie aktualnej prędkości aż do wyjechania za ograniczenie
                        fVelDes = v;
                    }
                    if( v < VelLimitLastDist.first ) {
                        VelLimitLastDist.second = d + point.trTrack->Length() + fLength;
                    }
                    else if( VelLimitLastDist.second > 0 && false == point.flags.HasAny((SpeedPointFlag::switchTrack | SpeedPointFlag::passengerStop)) ) { // the speed limit can potentially start afterwards, so don't mark it as broken too soon
						speedlimitiscontinuous = false;
					}
                    if( false == railwaytrackend ) {
                        continue; // i tyle wystarczy
                    }
                }
                else {
                    // event trzyma tylko jeśli VelNext=0, nawet po przejechaniu (nie powinno dotyczyć samochodów?)
                    a = v > 0.0 ?
                        fAcc :
                        mvOccupied->Vel < 0.01 ?
                            0.0 : // already standing still so no need to bother with brakes
                           -2.0; // ruszanie albo hamowanie
                }
            }
            // track can potentially end, which creates another virtual point of interest with speed limit of 0 at the end of it
            // TBD, TODO: when tracing the route create a dedicated table entry for it, to simplify the code?
            if( railwaytrackend ) {
                // if the railway track ends here set the velnext accordingly as well
                // TODO: test this with turntables and such
                auto const stopatendacceleration = -1.0 * mvOccupied->Vel * mvOccupied->Vel / ( 25.92 * ( d + point.trTrack->Length() ) );
                if( stopatendacceleration < a ) {
                    a = stopatendacceleration;
                    v = 0.0;
                    d += point.trTrack->Length();
                    if( d < fMinProximityDist ) {
                        // jak jest już blisko, ograniczenie aktualnej prędkości
                        fVelDes = v;
                    }
                }
            }

            if( a <= fAcc
             && ( v < fNext || fNext < 0 ) ) { // filter out consecutive, farther out blocks with the same speed limit; they'd make us accelerate slower due to their lower a value
                // mniejsze przyspieszenie to mniejsza możliwość rozpędzenia się albo konieczność hamowania
                // jeśli droga wolna, to może być a>1.0 i się tu nie załapuje
                fAcc = a; // zalecane przyspieszenie (nie musi być uwzględniane przez AI)
                fNext = v; // istotna jest prędkość na końcu tego odcinka
                fDist = d; // dlugość odcinka
                bStopTargetIsPassengerStop = ( v == 0.0 ) && ( point.flags.HasAny(SpeedPointFlag::passengerStop) );
            }
/*
            else if ((fAcc > 0) && (v >= 0) && (v > fNext)) {
                // jeśli nie ma wskazań do hamowania, można podać drogę i prędkość na jej końcu
                fNext = v; // istotna jest prędkość na końcu tego odcinka
                fDist = d; // dlugość odcinka (kolejne pozycje mogą wydłużać drogę, jeśli prędkość jest stała)
            }
*/
            // we'll pick first scanned target speed as our goal
            // farther scan table points can override it through previous clause, if they require lower acceleration or speed reduction
            else if( a > 0 && a <= fAcc && v >= 0 && fNext < 0 ) {
                fAcc = a;
                fNext = v;
                fDist = d;
                bStopTargetIsPassengerStop = ( v == 0.0 ) && ( point.flags.HasAny(SpeedPointFlag::passengerStop) );
            }
            // potentially update our current speed limit
            // a point limit binds only the head at its point, it neither extends nor breaks a limit over a stretch of the line
            if( false == ispointlimit
             && v < VelLimitLastDist.first
             && ( d < 0 // the point counts as part of last speed limit either if it's behind us
               || VelLimitLastDist.second > 0 ) ) { // or if we already have the last limit ongoing
                VelLimitLastDist.second = d + fLength;
                if( point.flags.HasAny(SpeedPointFlag::track) ) {
                    VelLimitLastDist.second += point.trTrack->Length();
                }
            }
            else if( false == point.flags.HasAny(( SpeedPointFlag::switchTrack | SpeedPointFlag::passengerStop | SpeedPointFlag::pointSpeed )) ) {
                // if we found a point which isn't part of the current speed limit mark the limit as broken
                // NOTE: we exclude switches from this rule, as speed limits generally extend through these
                speedlimitiscontinuous = false;
            }
        } // if (v>=0.0)
        // finding a point which doesn't impose speed limit means any potential active speed limit can't extend through entire scan table
        // NOTE: we test actual speed value for the given point, as events may receive (v = -1) as a way to disable them in irrelevant modes
        if( point.fVelNext < 0 && false == point.flags.HasAny((SpeedPointFlag::switchTrack | SpeedPointFlag::passengerStop | SpeedPointFlag::pointSpeed)) ) {
            // NOTE: we exclude switches from this rule, as speed limits generally extend through these
            // and point limits, which say nothing of the line they lie on
			speedlimitiscontinuous = false;
		}
        if (fNext >= 0.0)
        { // jeśli ograniczenie
            if( (point.flags & (SpeedPointFlag::enabled | SpeedPointFlag::event)) == (SpeedPointFlag::enabled | SpeedPointFlag::event) && false == point.flags.HasAny(SpeedPointFlag::pointSpeed) && eSignNext == nullptr ) { // tylko sygnał przypisujemy (a point limit isn't one)
				// jeśli jeszcze nic nie zapisane tam
				eSignNext = point.evEvent; // dla informacji
			}
            if( fNext == 0.0 ) {
                break; // nie ma sensu analizować tabelki dalej
            }
        }
    }

    // the point limits latched before, which aren't in the table any more
    apply_point_limits( fVelDes, fDist, fNext, fAcc );

    // jeśli mieliśmy ograniczenie z semafora i nie ma przed nami
    if( VelSignalLast >= 0.0
     && ( false == drivingFlags.signalFound && false == drivingFlags.switchFound && false == drivingFlags.stopPointFound )
       && true == OrderCurrentGet().HasAll(DriverOrder::obeyTrain) ) {
        VelSignalLast = -1.0;
    }
    // if there's unbroken speed limit through our scan table take a note of it
    if( VelLimitLastDist.second > 0
     && speedlimitiscontinuous ) {
        VelLimitLastDist.second = EU07_AI_SPEEDLIMITEXTENDSBEYONDSCANRANGE;
    }
    // take into account the effect switches have on duration of signal-imposed speed limit, in calculation of speed limit end point
    // a reduced speed ends where the interlocking area does, if that is marked, and not past the switches farther on
    if( auto const signallimitend { VelSignalLast > 0.0 && SignalAreaEndDist >= 0.0 ? SignalAreaEndDist : SwitchClearDist };
        VelSignalLast >= 0.0 && signallimitend >= 0.0 ) {
        VelLimitLastDist.second = std::max( VelLimitLastDist.second, signallimitend );
    }

    //analiza spisanych z tabelki ograniczeń i nadpisanie aktualnego
    // if stopped at a valid passenger stop, hold there
    if( true == IsAtPassengerStop && mvOccupied->Vel < 0.01 ) {
        fVelDes = 0.0;
    }
    else {
        fVelDes = min_speed( fVelDes, VelSignalLast );
        fVelDes = min_speed( fVelDes, VelLimitLast );
        fVelDes = min_speed( fVelDes, VelRoad );
        fVelDes = min_speed( fVelDes, VelRestricted );
    }
    // nastepnego semafora albo zwrotnicy to uznajemy, że mijamy W5
    FirstSemaphorDist = d_to_next_sem; // przepisanie znalezionej wartosci do zmiennej
    return go;
};

bool
TController::TableUpdateStopPoint( TCommandType &Command, TSpeedPos &Point, double const Signaldistance ) {
    // stop points are irrelevant when not in one of the basic modes
    if( false == OrderCurrentGet().HasAny( DriverOrder::shunt | DriverOrder::looseShunt | DriverOrder::obeyTrain | DriverOrder::bank ) ) { return true; }
    // jeśli przystanek, trzeba obsłużyć wg rozkładu
    drivingFlags.stopPointFound = true;
    // the W4 and the timetable name the station each in its own file, so the two spellings are compared by their key:
    // with or without the polish letters, in any case, with spaces or underscores
    if( false == timetable::stop_point_is_for( Point.evEvent->input_text(), asNextStop ) )
    { // jeśli nazwa nie jest zgodna
        if( false == IsScheduledPassengerStopVisible // check if our next scheduled stop didn't show up earlier in the scan
         && Point.fDist < 1.15 * fBrakeDist + 300
         && Point.fDist > 0 ) // tylko jeśli W4 jest blisko, przy dwóch może zaczać szaleć
        {
            // porównuje do następnej stacji, więc trzeba przewinąć do poprzedniej
            // nastepnie ustawić następną na aktualną tak żeby prawidłowo ją obsłużył w następnym kroku
            if( true == TrainParams.RewindTimeTable( Point.evEvent->input_text() ) ) {
                asNextStop = TrainParams.NextStop();
                TrainParams.StationStart = TrainParams.StationIndex;
            }
        }
        else if( Point.fDist < -fLength ) {
            // jeśli został przejechany
            Point.ResetFlags(); // to można usunąć (nie mogą być usuwane w skanowaniu)
        }
        return true; // ignorowanie jakby nie było tej pozycji
    }
    else if (drivingFlags.stopPoint) // jeśli pomijanie W4, to nie sprawdza czasu odjazdu
    { // tylko gdy nazwa zatrzymania się zgadza
        if( OrderCurrentGet().HasAny( DriverOrder::obeyTrain | DriverOrder::bank ) ) {
            // check whether the station specifies radio channel change
            // NOTE: we don't do it in shunt mode, as shunting operations tend to use dedicated radio channel
            // NOTE: there's a risk radio channel change was specified by a station which we skipped during timetable rewind
            // we ignore this for the time being as it's not a high priority error
            auto const radiochannel { TrainParams.radio_channel() };
            if( radiochannel > 0 ) {
                if( iGuardRadio != 0 ) {
                    iGuardRadio = radiochannel;
                }
                if( iRadioChannel != radiochannel ) {
                    cue_action( driver_hint::radiochannel, radiochannel );
                }
            }
        }
        IsScheduledPassengerStopVisible = true; // block potential timetable rewind if the next stop shows up later in the scan
        if (false == TrainParams.IsStop())
        { // jeśli nie ma tu postoju
            Point.fVelNext = -1; // maksymalna prędkość w tym miejscu
            // przy 160km/h jedzie 44m/s, to da dokładność rzędu 5 sekund
            if (Point.fDist < passengerstopmaxdistance * 0.5 ) {
                // zaliczamy posterunek w pewnej odległości przed (choć W4 nie zasłania już semafora)
#if LOGSTOPS
                WriteLog(std::format(
                    "{} as {}: at {}:{} passed {}",
                    pVehicle->asName, TrainParams.TrainName, simulation::Time.data().wHour, simulation::Time.data().wMinute, asNextStop)); // informacja
#endif
                // przy jakim dystansie (stanie licznika) ma przesunąć na następny postój
                fLastStopExpDist = mvOccupied->DistCounter + 0.250 + 0.001 * fLength;
                TrainParams.UpdateMTable( simulation::Time, asNextStop );
                record_station( Point, blackbox::station_kinds::passed );
                UpdateDelayFlag();
                TrainParams.StationIndexInc(); // przejście do następnej
                asNextStop = TrainParams.NextStop(); // pobranie kolejnego miejsca zatrzymania
                Point.ResetFlags(); // nie liczy się już
                return true; // table entry recognized and handled
            }
        } // koniec obsługi przelotu na W4
        else {
            // zatrzymanie na W4
            // the margins of a passenger stop, whether the train is already making for it or not: the present ones
            // depend on this very flag, and a check against them would turn it on and off every other scan
            auto const [ stopminproximity, stopmaxproximity ] {
                is_train() ?
                    train_proximity_margins( true ) :
                    std::pair{ fMinProximityDist, fMaxProximityDist } };
            if ( false == Point.bMoved ) {
                // potentially shift the stop point in accordance with its defined parameters
                /*
                // https://rainsted.com/pl/Wersja/18.2.133#Okr.C4.99gi_dla_W4_i_W32
                Pierwszy parametr ujemny - preferowane zatrzymanie czoła składu (np. przed przejściem).
                Pierwszy parametr dodatni - preferowane zatrzymanie środka składu (np. przy wiacie, przejściu podziemnym).
                Drugi parametr ujemny - wskazanie zatrzymania dla krótszych składów (W32).
                Drugi paramer dodatni - długość peronu (W4).
                */
                auto L = 0.0;
				auto Par1 = Point.evEvent->input_value(1);
				auto Par2 = Point.evEvent->input_value(2);
				if (Par2 >= 0 || fLength < -Par2) { //użyj tego W4
					if (Par1 < 0) {
                        L = -Par1;
                    }
                    else {
                        //środek
                        L = Par1 - stopminproximity - fLength * 0.5;
                    }
                    // the train comes to a stand anywhere up to the margin of a passenger stop short of this point, not
                    // that of the signals it may be making for when it first sees the W4, so the tail is to keep that far
                    // inside the platform
					L = std::max(0.0, std::min(L, std::abs(Par2) - stopminproximity - fLength));
					Point.UpdateDistance(L);
					Point.bMoved = true;
					Point.fMoved = L;
                }
                else {
					Point.ResetFlags();
                }
            }
            // for human-driven vehicles discard the stop point if they leave it far enough behind
            if( false == AIControllFlag
             && Point.fDist < -1 * std::max(fLength + 100, 250.0) - Point.fMoved ) {
                record_station( Point, blackbox::station_kinds::missed );
                Point.ResetFlags(); // nie liczy się już zupełnie (nie wyśle SetVelocity)
                Point.fVelNext = -1; // można jechać za W4
                if( Point.fDist <= 0.0 && eSignNext == Point.evEvent ) {
                    // sanity check, if we're held by this stop point, let us go
                    VelSignalLast = -1;
                }
                return true;
            }

            IsAtPassengerStop = Point.fDist <= passengerstopmaxdistance
			                    // Ra 2F1I: odległość plus długość pociągu musi być mniejsza od długości
			                    // peronu, chyba że pociąg jest dłuższy, to wtedy minimalna.
			                    // jeśli długość peronu ((sSpeedTable[i].evEvent->ValueGet(2)) nie podana,
			                    // przyjąć odległość fMinProximityDist
			                    && (drivingFlags.stopCloser ?
			                            Point.fDist + fLength + (Point.fMoved - stopminproximity * 0.5f) <= std::max(std::abs(Point.evEvent->input_value(2)),
			                                                                                                          2.0 * stopmaxproximity + fLength) :
			                            Point.fDist < Signaldistance);

            if( !eSignNext ) {
                //jeśli nie widzi następnego sygnału ustawia dotychczasową
                eSignNext = Point.evEvent;
            }
            if( mvOccupied->Vel > EU07_AI_MOVEMENT ) {
                // jeśli jedzie (nie trzeba czekać, aż się drgania wytłumią - drzwi zamykane od 1.0) to będzie zatrzymanie
                Point.fVelNext = 0;
                // potentially announce pending stop
                if( m_lastannouncement != announcement_t::approaching
                 && Point.fDist < 750
                 && Point.fDist > 250 ) {
                    announce( announcement_t::approaching );
                }
            } else if( true == IsAtPassengerStop ) {
                // jeśli się zatrzymał przy W4, albo stał w momencie zobaczenia W4
                if( !AIControllFlag ) {
                    // w razie przełączenia na AI ma nie podciągać do W4, gdy użytkownik zatrzymał za daleko
                    drivingFlags.stopCloser = false;
                }
                if (TrainParams.UpdateMTable( simulation::Time, asNextStop) ) {
                    // to się wykona tylko raz po zatrzymaniu na W4
                    record_station( Point, blackbox::station_kinds::stopped );
                    // the service flags of this stop hold for its doors and departure
                    m_stopflags = TrainParams.stop_flags();
                    if( TrainParams.StationIndex < TrainParams.StationCount ) {
                        // jeśli są dalsze stacje, bez trąbienia przed odjazdem
                        // also ignore any horn cue that may be potentially set below 1 km/h and before the actual full stop
                        drivingFlags.startHorn = false;
                        drivingFlags.startHornNow = false;
                    }
                    UpdateDelayFlag();

                    // perform loading/unloading
                    // HACK: manual check if we didn't already do load exchange at this stop
                    // TODO: remove the check once the station system is in place
                    if( m_lastexchangestop != asNextStop ) {
                        m_lastexchangestop = asNextStop;
                        m_lastexchangedirection = pVehicle->DirectionGet();
                        m_lastexchangeplatforms = static_cast<int>( std::floor( std::abs( Point.evEvent->input_value( 2 ) ) ) ) % 10;
                        auto const exchangetime { simulation::Station.update_load( pVehicles[ end::front ], TrainParams, m_lastexchangeplatforms ) };
                        WaitingSet( exchangetime );
                        // announce the stop name while at it
                        announce( announcement_t::current );
                        m_makenextstopannouncement = true;
                    }

                    if (TrainParams.DirectionChange()) {
                        // jeśli "@" w rozkładzie, to wykonanie dalszych komend
                        // wykonanie kolejnej komendy, nie dotyczy ostatniej stacji
                        if (drivingFlags.pushPull) {
                            // SN61 ma się też nie ruszać, chyba że ma wagony
                            drivingFlags.stopHere = true; // EZT ma stać przy peronie
                            if (OrderNextGet() != DriverOrder::changeDirection) {
                                OrderPush(DriverOrder::changeDirection); // zmiana kierunku
                                OrderPush(
                                    TrainParams.StationIndex < TrainParams.StationCount ?
                                        DriverOrder::obeyTrain :
                                        DriverOrder::shunt); // to dalej wg rozkładu
                            }
                        }
                        else {
                            // a dla lokomotyw...
                            // pozwolenie na przejechanie za W4 przed czasem i nie ma stać
                            drivingFlags.stopPoint = false;
                            drivingFlags.stopHere = false;
                        }
                        // przejście do kolejnego rozkazu (zmiana kierunku, odczepianie)
                        JumpToNextOrder();
                        // ma nie podjeżdżać pod W4 po przeciwnej stronie
                        drivingFlags.stopCloser = false;
                        // ten W4 nie liczy się już zupełnie (nie wyśle SetVelocity)
                        Point.ResetFlags();
                        // jechać
                        Point.fVelNext = -1;
                        // nie analizować prędkości
                        return true;
                    }
                }

                if (OrderCurrentGet() & ( DriverOrder::shunt | DriverOrder::looseShunt )) {
                    OrderNext(DriverOrder::obeyTrain); // uruchomić jazdę pociągową
                    CheckVehicles(); // zmienić światła
                }

                if (TrainParams.StationIndex < TrainParams.StationCount) {
                    // jeśli są dalsze stacje, czekamy do godziny odjazdu
                    if ( true == IsCargoTrain
                      || true == TrainParams.IsMaintenance()
                      || TrainParams.IsTimeToGo(simulation::Time.data().wHour, simulation::Time.data().wMinute + simulation::Time.data().wSecond * 0.0167) ) {
                        // z dalszą akcją czekamy do godziny odjazdu
                        // cargo trains and passenger trains at maintenance stop don't need to wait
                        IsAtPassengerStop = false;
                        record_station( Point, blackbox::station_kinds::ready );
                        // przy jakim dystansie (stanie licznika) ma przesunąć na następny postój
                        fLastStopExpDist = mvOccupied->DistCounter + 0.050 + 0.001 * fLength;
                        TrainParams.StationIndexInc(); // przejście do następnej
                        asNextStop = TrainParams.NextStop(); // pobranie kolejnego miejsca zatrzymania
#if LOGSTOPS
                        WriteLog(std::format(
                            "{} as {}: at {}:{} next {}",
                            pVehicle->asName, TrainParams.TrainName, simulation::Time.data().wHour, simulation::Time.data().wMinute, asNextStop)); // informacja
#endif
                        // update consist weight, brake settings and ai braking tables
                        // NOTE: this calculation is expected to run after completing loading/unloading
                        CheckVehicles(); // nastawianie hamulca do jazdy pociągowej

                        if( static_cast<int>( std::floor( std::abs( Point.evEvent->input_value( 1 ) ) ) ) % 2 ) {
                            // nie podjeżdżać do semafora, jeśli droga nie jest wolna
                            drivingFlags.stopHere = true;
                        }
                        else {
                            //po czasie jedź dalej
                            drivingFlags.stopHere = false;
                        }
                        drivingFlags.stopCloser = true; // do następnego W4 podjechać blisko (z dociąganiem)
                        Point.ResetFlags(); // nie liczy się już zupełnie (nie wyśle SetVelocity)
                        Point.fVelNext = -1; // można jechać za W4
                        if( Point.fDist <= 0.0 && eSignNext == Point.evEvent ) {
                            // sanity check, if we're held by this stop point, let us go
                            VelSignalLast = -1;
                        }
                        if( Command == TCommandType::cm_Unknown ) { // jeśli nie było komendy wcześniej
                            Command = TCommandType::cm_Ready; // gotów do odjazdu z W4 (semafor może zatrzymać)
                        }
                        if( false == tsGuardSignal.empty()
                         && true == timetable::has_conductor( m_stopflags ) ) {
                            // jeśli mamy głos kierownika, to odegrać
                            drivingFlags.guardSignal = true;
                        }
                        return true; // nie analizować prędkości
                    } // koniec startu z zatrzymania
					else {
						cue_action(driver_hint::waitdeparturetime);
						if (timetable::has_conductor(m_stopflags) // no conductor, no door kept open for him
						 && TrainParams.IsTimeToGo(simulation::Time.data().wHour, simulation::Time.data().wMinute + simulation::Time.data().wSecond*0.0167 + 0.1)) { //if is less than 6 sec to departure
							drivingFlags.guardOpenDoor = true;
							GuardOpenDoor();
						}
					}
                } // koniec obsługi początkowych stacji
                else {
                    // jeśli dojechaliśmy do końca rozkładu
#if LOGSTOPS
                    WriteLog(std::format(
                        "{} as {}: at {}:{} end of route.",
                        pVehicle->asName, TrainParams.TrainName, simulation::Time.data().wHour, simulation::Time.data().wMinute)); // informacja
#endif
                    asNextStop = TrainParams.NextStop(); // informacja o końcu trasy
                    TrainParams.NewName("none"); // czyszczenie nieaktualnego rozkładu
                    // koniec kursu, gasimy wyświetlacze relacji (tablice blaszane zostają)
                    for( auto *vehicle { pVehicles[ end::front ] }; vehicle != nullptr; vehicle = vehicle->Next() ) {
                        vehicle->DestinationClear();
                    }
                    // ma nie podjeżdżać pod W4 i ma je pomijać
                    drivingFlags.stopCloser = false;
                    if( false == drivingFlags.pushPull ) {
                        // if the consist can change direction through a simple cab change it doesn't need fiddling with recognition of passenger stops
                        drivingFlags.stopPoint = false;
                    }
                    fLastStopExpDist = -1.0f; // nie ma rozkładu, nie ma usuwania stacji
                    Point.ResetFlags(); // W4 nie liczy się już (nie wyśle SetVelocity)
                    Point.fVelNext = -1; // można jechać za W4
                    if( Point.fDist <= 0.0 && eSignNext == Point.evEvent ) {
                        // sanity check, if we're held by this stop point, let us go
                        VelSignalLast = -1;
                    }
                    // wykonanie kolejnego rozkazu (Change_direction albo Shunt)
                    JumpToNextOrder();
                    // ma się nie ruszać aż do momentu podania sygnału
                    drivingFlags.stopHere = true;
                    drivingFlags.startHorn = true;
                    return true; // nie analizować prędkości
                } // koniec obsługi ostatniej stacji
            } // vel 0, at passenger stop
            else {
                // HACK: momentarily deactivate W4 to trick the controller into moving closer
                Point.fVelNext = -1;
            } // vel 0, outside of passenger stop
        } // koniec obsługi zatrzymania na W4
    } // koniec warunku pomijania W4 podczas zmiany czoła
    else
    { // skoro pomijanie, to jechać i ignorować W4
        Point.ResetFlags(); // W4 nie liczy się już (nie zatrzymuje jazdy)
        Point.fVelNext = -1;
        return true; // nie analizować prędkości
    }

    return false; //
}

// a road speed sign (W27/W27a) passed by the head stays in the table until the rear is past it too. not for cars: their
// passed events leave the table at the latest when the rear passes them (TableCheck), before the table update sees that
bool
TController::is_road_velocity_held( TSpeedPos const &Point ) const {

    return Point.flags.HasAll(SpeedPointFlag::roadSpeed)
        && mvOccupied->CategoryFlag != 2
        && Point.fDist >= -fLength;
}

// a passed event is handled once and then dropped from the table, except a road speed sign held for the rear
void
TController::TableDropPassedEvent( TSpeedPos &Point ) const {

    if( is_road_velocity_held( Point ) ) {
        Point.flags |= SpeedPointFlag::roadSpeedHeld;
        return;
    }
    Point.Clear();
}

// the lowest road speed of the signs past the given one that the head has passed: held until the rear is past them too,
// or passed in this very tick and not marked as held yet; -1 if there's none
double
TController::held_road_velocity_beyond( TSpeedPos const &Point ) const {

    auto lowest { -1.0 };
    for( auto const &held : sSpeedTable ) {
        if( ( held.flags & ( SpeedPointFlag::enabled | SpeedPointFlag::roadSpeed ) ) != ( SpeedPointFlag::enabled | SpeedPointFlag::roadSpeed )
         || held.fDist >= 0.0
         || held.fDist <= Point.fDist ) {
            continue;
        }
        lowest = min_speed( lowest, held.fVelNext );
    }
    return lowest;
}

// a road speed (W27/W27a) lower than the one in force holds once the head passes the sign, a higher one only once the
// whole train is past it (a car takes it from the head). the first pass after the head handles the sign as before, the
// entry is then only held for the rear: returns true for it, so the rest of the table update leaves it out
bool
TController::TableUpdateRoadVelocity( TSpeedPos &Point ) {

    if( Point.fDist >= 0.0 ) { return false; }

    if( mvOccupied->CategoryFlag == 2 ) {
        VelRoad = Point.fVelNext;
        return false;
    }

    auto const isrearpast { Point.fDist < -fLength };
    if( isrearpast ) {
        // the rear past it, but not past a sign after it the head has passed too: none of those may be exceeded, so a
        // lower one (W27a 60 within a train's length after a 120) stays in force
        VelRoad = min_speed( Point.fVelNext, held_road_velocity_beyond( Point ) );
    }
    else if( Point.fVelNext == min_speed( Point.fVelNext, VelRoad ) ) {
        VelRoad = Point.fVelNext;
    }
    if( false == Point.flags.HasAll(SpeedPointFlag::roadSpeedHeld) ) {
        return false;
    }
    if( isrearpast ) {
        Point.Clear();
    }
    return true;
}

// a reduced speed shown by a signal holds to the end of the interlocking area the signal guards (Ie-1), and that end is
// marked on the main track with a SignalAreaEnd event. on a track without the marker the speed holds as it did before:
// until the next signal, or until no signal, switch or stop point is left in the scan
void
TController::TableUpdateSignalAreaEnd( TSpeedPos &Point ) {

    // the switches lie in front of the marker, so the whole train has left them once its rear end is past it
    if( Point.fDist >= -fLength ) {
        if( SignalAreaEndDist < 0.0
         && OrderCurrentGet().HasAll(DriverOrder::obeyTrain) ) {
            // the table is sorted by distance, so the first marker is the nearest one
            SignalAreaEndDist = Point.fDist + fLength;
        }
        return;
    }

    // in shunting the marker stays, a train order given before the next signal still finds it
    if( false == OrderCurrentGet().HasAll(DriverOrder::obeyTrain) ) { return; }

    // a passed signal at stop (0) keeps holding the train, only a reduced speed ends here
    if( VelSignalLast > 0.0 ) {
        if( Global.iWriteLogEnabled & 8 ) {
            WriteLog( "Speed table update for " + OwnerName() + ", left the interlocking area at " + Point.evEvent->m_name );
        }
        VelSignalLast = -1.0;
    }
    Point.Clear();
}

bool TController::point_limit_latched( basic_event const *Cell ) const {

    return std::any_of(
        m_pointlimits.begin(), m_pointlimits.end(),
        [&]( point_limit const &Limit ) { return Limit.source == Cell; } );
}

double TController::point_limit_end( glm::dvec3 const &Heading ) const {

    auto const origin { pVehicle->GetPosition() };
    auto result { 0.0 };
    auto first { true };
    for( auto const *vehicle : { pVehicles[ end::front ], pVehicles[ end::rear ] } ) {
        if( vehicle == nullptr ) { continue; }
        for( auto const side : { end::front, end::rear } ) {
            auto const along { glm::dot( vehicle->CouplerPosition( side ) - origin, Heading ) };
            result = ( first ? along : std::max( result, along ) );
            first = false;
        }
    }
    return result;
}

void TController::latch_point_limit( TSpeedPos const &Point ) {

    if( point_limit_latched( Point.evEvent ) ) { return; }
    m_pointlimits.push_back(
        point_limit { Point.evEvent, Point.fVelNext, Point.fDist + Point.fPointDist, Point.fPointDist, travel_direction() } );
    if( Global.iWriteLogEnabled & 8 ) {
        WriteLog( "Speed table update for " + OwnerName() + ", latched point limit of " + Point.evEvent->m_name
            + ": " + to_string( Point.fVelNext, 0 ) + " in " + to_string( m_pointlimits.back().distance, 0 ) + " m" );
    }
}

void TController::update_point_limits() {

    if( pVehicle == nullptr ) { return; }
    auto const position { pVehicle->GetPosition() };
    if( m_pointlimitsvehicle == pVehicle ) {
        auto const movement { position - m_pointlimitsposition };
        auto const length { glm::length( movement ) };
        if( length >= 200.0 ) {
            // a longer jump between two updates isn't driving: the consist was moved elsewhere
            m_pointlimits.clear();
        }
        else if( length > 0.0 ) {
            auto const step { movement / length };
            for( auto &limit : m_pointlimits ) {
                if( glm::length( limit.heading ) < 0.5 ) {
                    // the way wasn't known when the limit was latched, the head was passing the cell
                    limit.heading = step;
                }
                auto const along { glm::dot( step, limit.heading ) };
                limit.distance += ( along >= 0.0 ? -length : length );
                // the way towards the point follows the line, step by step; a step far off it
                // (a jolt of a standing vehicle) doesn't turn it
                if( std::abs( along ) > 0.5 ) {
                    limit.heading = ( along >= 0.0 ? step : -step );
                }
            }
        }
    }
    // a change of the vehicle (another cab) starts the measure from its place
    auto const vehiclechanged { m_pointlimitsvehicle != pVehicle };
    auto const previousposition { m_pointlimitsposition };
    m_pointlimitsvehicle = pVehicle;
    m_pointlimitsposition = position;
    // the consist, as its end vehicles and its length
    auto const *front { pVehicles[ end::front ] };
    auto const *rear { pVehicles[ end::rear ] };
    auto const consistchanged {
        false == ( ( front == m_pointlimitsends[ 0 ] && rear == m_pointlimitsends[ 1 ] )
                || ( front == m_pointlimitsends[ 1 ] && rear == m_pointlimitsends[ 0 ] ) )
        || std::abs( fLength - m_pointlimitslength ) > 0.5 };
    m_pointlimitsends[ 0 ] = front;
    m_pointlimitsends[ 1 ] = rear;
    m_pointlimitslength = fLength;
    // vehicles coupled or uncoupled at the end towards the point move the head towards it or away. only a change of
    // the consist counts, as the end measured from the vehicle shifts on curves. another vehicle measured from (a cab
    // change, which may come with a coupling in the same order) moves the end measured before to its place
    for( auto &limit : m_pointlimits ) {
        if( glm::length( limit.heading ) < 0.5 ) { continue; }
        auto const consistend { point_limit_end( limit.heading ) };
        if( limit.endknown && consistchanged ) {
            auto const previousend {
                vehiclechanged ?
                    limit.end + glm::dot( previousposition - position, limit.heading ) :
                    limit.end };
            limit.distance -= consistend - previousend;
        }
        limit.end = consistend;
        limit.endknown = true;
    }

    auto const travel { travel_direction() };
    std::erase_if(
        m_pointlimits,
        [&]( point_limit const &Limit ) {
            // passed by the head
            if( Limit.distance < 0.0 ) {
                if( Global.iWriteLogEnabled & 8 ) {
                    WriteLog( "Speed table update for " + OwnerName() + ", passed point limit of " + Limit.source->m_name );
                }
                return true;
            }
            // the head backed up behind the cell: the speed table reads it again if it is ahead
            if( Limit.distance > Limit.celldistance + 1.0 ) {
                if( Global.iWriteLogEnabled & 8 ) {
                    WriteLog( "Speed table update for " + OwnerName() + ", left behind point limit of " + Limit.source->m_name );
                }
                return true;
            }
            // the cell ahead of the head again, going towards the point after backing up past the cell: it is read anew
            // (not while backing up: the cell may as well be scanned from the other side)
            return glm::dot( travel, Limit.heading ) > 0.0
                && std::any_of(
                sSpeedTable.begin(), sSpeedTable.end(),
                [&]( TSpeedPos const &Point ) {
                    return Point.evEvent == Limit.source
                        && Point.flags.HasAll(SpeedPointFlag::enabled | SpeedPointFlag::event | SpeedPointFlag::pointSpeed)
                        && Point.fDist > 0.0; } ); } );
}

void TController::apply_point_limits( double &Velocity, double &Distance, double &Next, double &Acceleration ) {

    auto const travel { travel_direction() };
    for( auto const &limit : m_pointlimits ) {
        // latched with no limit, or passed (dropped at the next update)
        if( limit.velocity < 0.0 || limit.distance <= 0.0 ) { continue; }
        // going away from the point, or the way isn't known
        if( glm::dot( travel, limit.heading ) <= 0.0 ) { continue; }
        // the same as for a point of the speed table ahead
        auto const v { limit.velocity };
        auto const d { limit.distance };
        auto a { ( v * v - mvOccupied->Vel * mvOccupied->Vel ) / ( 25.92 * d ) };
        if( mvOccupied->Vel < v ) {
            auto const brakingdistance { 1.2 * fBrakeDist * braking_distance_multiplier( v ) };
            if( brakingdistance > 0.0 ) {
                a = std::lerp( a, AccPreferred, std::clamp( ( d - brakingdistance ) / brakingdistance, 0.0, 1.0 ) );
            }
        }
        if( d < point_limit_approach() && v < Velocity ) {
            Velocity = v;
        }
        if( a <= Acceleration
         && ( v < Next || Next < 0 ) ) {
            Acceleration = a;
            Next = v;
            Distance = d;
            bStopTargetIsPassengerStop = false;
        }
    }
}

double TController::point_limit_approach() const {
    // a point limit holds at its point and not a metre beyond, so the braking curve alone leaves a slow consist
    // coasting above it (the deceleration asked for is below what makes the driver brake). within the final
    // approach, and five seconds of the present speed before it, the speed is held down to it already
    return fMaxProximityDist + mvOccupied->Vel / 3.6 * 5.0;
}

glm::dvec3 TController::travel_direction() const {

    if( pVehicle == nullptr || iDirection == 0 ) { return glm::dvec3 { 0.0 }; }
    // the direction is given against the couplers of the vehicle the driver is in, which is the vehicle measured;
    // along its own length it follows the track, where the chord of a long consist on a curve doesn't
    auto const along { pVehicle->CouplerPosition( end::front ) - pVehicle->CouplerPosition( end::rear ) };
    auto const length { glm::length( along ) };
    if( length < 0.001 ) { return glm::dvec3 { 0.0 }; }
    return ( iDirection > 0 ? along : -along ) / length;
}

bool
TController::TableUpdateEvent( double &Velocity, TCommandType &Command, TSpeedPos &Point, double &Signaldistance, int const Pointindex ) {

    if( Point.flags.HasAll(SpeedPointFlag::signalAreaEnd) ) {
        // not a signal: it neither sends a command nor counts as the end of a speed limit in the scan
        TableUpdateSignalAreaEnd( Point );
        return true;
    }

    if( Point.flags.HasAll(SpeedPointFlag::pointSpeed) ) {
        // PointVelocity: no signal, no command, only the limit at its point
        // done once the head passes the point
        if( Point.fDist + Point.fPointDist < 0.0 ) {
            if( Global.iWriteLogEnabled & 8 ) {
                WriteLog( "Speed table update for " + OwnerName() + ", passed point limit of " + Point.GetName() );
            }
            Point.Clear();
            return true;
        }
        if( Point.fDist < 0.0 ) {
            if( point_limit_latched( Point.evEvent ) ) {
                // a table built anew found under the consist a cell whose state holds already
                Point.Clear();
                return true;
            }
            if( Point.fVelNext != 0.0 ) {
                // the head passed the cell: what it says now holds at the point, whatever happens to the table.
                // a stop at 0 stays in the table, where the cell can still release it
                latch_point_limit( Point );
                Point.Clear();
                return true;
            }
        }
        return false;
    }

    // sprawdzanie eventów pasywnych miniętych
    if( Point.fDist < 0.0 ) {
        if( SemNextIndex == Pointindex ) {
            if( Global.iWriteLogEnabled & 8 ) {
                WriteLog( "Speed table update for " + OwnerName() + ", passed semaphor " + sSpeedTable[ SemNextIndex ].GetName() );
            }
            SemNextIndex = npos; // jeśli minęliśmy semafor od ograniczenia to go kasujemy ze zmiennej sprawdzającej dla skanowania w przód
        }
        if( SemNextStopIndex == Pointindex ) {
            if( Global.iWriteLogEnabled & 8 ) {
                WriteLog( "Speed table update for " + OwnerName() + ", passed semaphor " + sSpeedTable[ SemNextStopIndex ].GetName() );
            }
            SemNextStopIndex = npos; // jeśli minęliśmy semafor od ograniczenia to go kasujemy ze zmiennej sprawdzającej dla skanowania w przód
        }
        switch( Point.evEvent->input_command() ) {
            // TBD, TODO: expand emergency_brake handling to a more generic security system signal
            case TCommandType::cm_EmergencyBrake: {
                pVehicle->RadioStop();
                Point.Clear(); // signal received, deactivate
                return true;
            }
            case TCommandType::cm_SecuritySystemMagnet: {
                // NOTE: magnet induction calculation presumes the driver is located in the front vehicle
                // TBD, TODO: take into account actual position of controlled/occupied vehicle in the consist, whichever comes first
                auto const magnetlocation { pVehicles[ end::front ]->MoverParameters->SecuritySystem.MagnetLocation };
                auto const magnetrange { 1.0 };
                auto const ismagnetpassed { Point.fDist < -( magnetlocation + magnetrange ) };
                if( Point.fDist < -magnetlocation && (mvOccupied->Vel > EU07_AI_NOMOVEMENT || false == ismagnetpassed) ) {
                    // NOTE: normally we'd activate the magnet once the leading vehicle passes it
                    // but on a fresh scan after direction change it would be detected as long as it's under consist, and meet the (simple) activation condition
                    // thus we're doing a more precise check in such situation (we presume direction change takes place only if the vehicle is standing still)
				    mvOccupied->SecuritySystem.set_cabsignal_lock(!AIControllFlag); // don't make life difficult for the ai, but a human driver is a fair game
				    PutCommand(Point.evEvent->input_text(), Point.evEvent->input_value(1), Point.evEvent->input_value(2), nullptr);
			    }
                if( ismagnetpassed ) {
                    Point.Clear();
                    mvOccupied->SecuritySystem.set_cabsignal_lock(false);
                }
                return true;
            }
            default: {
                break;
            }
        }
    }
    // check signals ahead
    if( Point.fDist > 0.0 ) {
        // bail out early for signals which activate when passed
        // TBD: make it an event method?
        switch( Point.evEvent->input_command() ) {
            case TCommandType::cm_EmergencyBrake: {
                return true;
            }
            case TCommandType::cm_SecuritySystemMagnet: {
                return true;
            }
            default: {
                break;
            }
        }

        if( Point.IsProperSemaphor( OrderCurrentGet() ) ) {
            // special rule for cars: ignore stop signals at distance too short to come to a stop
            // as trying to stop in such situation is likely to place the car on train tracks
            if( is_car()
             && Point.fVelNext != -1.0
             && Point.fVelNext < 1.0
             && Point.fDist < -0.5 + std::min(fBrakeDist * 0.2, mvOccupied->Vel * 0.2) ) {
                Point.Clear();
                return true;
            }
            // jeśli jest mienięty poprzedni semafor a wcześniej
            // byl nowy to go dorzucamy do zmiennej, żeby cały czas widział najbliższy
            if( SemNextIndex == npos ) { // NOSONAR
                SemNextIndex = Pointindex;
                if( Global.iWriteLogEnabled & 8 ) {
                    WriteLog( "Speed table update for " + OwnerName() + ", next semaphor is " + sSpeedTable[ SemNextIndex ].GetName() );
                }
            }
            if( SemNextStopIndex == npos
             || ( sSpeedTable[SemNextStopIndex].fVelNext != 0
               && Point.fVelNext == 0 ) ) {
                SemNextStopIndex = Pointindex;
            }
        }
    }
    if (Point.flags & SpeedPointFlag::outsideStation)
    { // jeśli W5, to reakcja zależna od trybu jazdy
        if (OrderCurrentGet() & DriverOrder::obeyTrain)
        { // w trybie pociągowym: można przyspieszyć do wskazanej prędkości (po zjechaniu z rozjazdów)
            Velocity = -1.0; // ignorować?
			if ( Point.fDist < 0.0) { // jeśli wskaźnik został minięty
                VelSignalLast = Velocity; //ustawienie prędkości na -1
            }
            else if( ( false == drivingFlags.switchFound ) ) { // jeśli rozjazdy już minięte
                VelSignalLast = Velocity; //!!! to też koniec ograniczenia
            }
        }
        else
        { // w trybie manewrowym: skanować od niego wstecz, stanąć po wyjechaniu za sygnalizator i zmienić kierunek
            Velocity = 0.0; // zmiana kierunku może być podanym sygnałem, ale wypadało by zmienić światło wcześniej
            if( ( false == drivingFlags.switchFound ) ) { // jeśli nie ma rozjazdu
                // check for presence of a signal facing the opposite direction
                // if there's one, we'll want to pass it before changing direction
                basic_event *foundevent = nullptr;
                if( Point.fDist - fMaxProximityDist > 0 ) {
                    auto scandistance{ Point.fDist + fLength - fMaxProximityDist };
                    auto *scanvehicle{ pVehicles[ end::rear ] };
                    auto scandirection{ scanvehicle->DirectionGet() * scanvehicle->RaDirectionGet() };
                    auto *foundtrack = BackwardTraceRoute( scandistance, scandirection, scanvehicle, foundevent, -1, end::front, false );
                }
                if( foundevent == nullptr ) {
                    drivingFlags.trackEnd = true; // to dalsza jazda trwale ograniczona (W5, koniec toru)
                }
            }
        }
    }
    else if ( Point.flags & SpeedPointFlag::stopOnAutomaticBlock) {
        // jeśli S1 na SBL
        if( mvOccupied->Vel < 2.0 && Point.fDist < fMaxProximityDist && Obstacle.distance > 1000 ) {
            // stanąć nie musi, ale zwolnić przynajmniej
            // jest w maksymalnym zasięgu to można go pominąć (wziąć drugą prędkosć)
            // as long as there isn't any obstacle in arbitrary view range
			eSignSkip = Point.evEvent;
			// jazda na widoczność - skanować możliwość kolizji i nie podjeżdżać zbyt blisko
			// usunąć flagę po podjechaniu blisko semafora zezwalającego na jazdę
			// ostrożnie interpretować sygnały - semafor może zezwalać na jazdę pociągu z przodu!
			drivingFlags.visibility = true;
			// store the ordered restricted speed and don't exceed it until the flag is cleared
			VelRestricted = Point.evEvent->input_value(2);
		}
        if( eSignSkip != Point.evEvent ) {
            // jeśli ten SBL nie jest do pominięcia to ma 0 odczytywać
            Velocity = Point.evEvent->input_value( 1 );
            // TODO sprawdzić do której zmiennej jest przypisywane v i zmienić to tutaj
        }
    }
    else if ( Point.IsProperSemaphor(OrderCurrentGet()))
    { // to semaphor
        if( Point.fDist < 0 ) {
            // for human-driven vehicles ignore the signal if it was passed by sufficient distance
            if( false == AIControllFlag
             && Point.fDist < -1 * std::max(fLength + 100, 250.0) ) {
                VelSignal = -1.0;
                Point.Clear();
                return true;
            }
            // for ai-driven vehicles always play by the rules
            else {
                VelSignalLast = Point.fVelNext; //minięty daje prędkość obowiązującą
                // a marker before this signal ends the interlocking area of an earlier signal, not the area of this one
                // (a train longer than the way between the two, or one which started standing over the marker)
                std::ranges::for_each( sSpeedTable | std::views::take( Pointindex ), clear_signal_area_end );
            }
        }
        else {
            //jeśli z przodu to dajemy flagę, że jest
			drivingFlags.signalFound = true;
            Signaldistance = std::min( Point.fDist, Signaldistance );
            // if there's another vehicle closer to the signal, then it's likely its intended recipient
            // HACK: if so, make it a stop point, to prevent non-signals farther down affect us
            auto const isforsomeoneelse { is_train() && Obstacle.distance < Point.fDist };
            if( Point.fDist <= Signaldistance ) {
                if( isforsomeoneelse ) {
                    VelSignalNext = 0.0;
                    Velocity = 0.0;
                }
                else {
                    VelSignalNext = Point.fVelNext;
                    if( Velocity < 0 ) {
                        Velocity = fVelMax;
                        VelSignal = fVelMax;
                    }
                }
            }
        }
    }
    else if ( Point.flags & SpeedPointFlag::roadSpeed)
    { // to W27/W27a
        if( TableUpdateRoadVelocity( Point ) ) {
            return true;
        }
    }
    else if ( Point.flags & SpeedPointFlag::sectionSpeed && Point.fDist < 0)
    { // to W9
		// teraz trzeba sprawdzić inne warunki
		if (Point.fSectionVelocityDist == 0.0)
		{
			if (Global.iWriteLogEnabled & 8)
			{
				WriteLog("TableUpdate: Event is behind. SVD = 0: " + Point.evEvent->m_name);
			}
			Point.Clear(); // jeśli punktowy to kasujemy i nie dajemy ograniczenia na stałe
		}
		else if (Point.fSectionVelocityDist < 0.0)
		{
			// ograniczenie obowiązujące do następnego
			if (Point.fVelNext == min_speed(Point.fVelNext, VelLimitLast) && Point.fVelNext != VelLimitLast)
			{
				// jeśli ograniczenie jest mniejsze niż obecne to obowiązuje od zaraz
				VelLimitLast = Point.fVelNext;
			}
			else if (Point.fDist < -fLength)
			{
				// jeśli większe to musi wyjechać za poprzednie
				VelLimitLast = Point.fVelNext;
				if (Global.iWriteLogEnabled & 8)
				{
					WriteLog("TableUpdate: Event is behind. SVD < 0: " + Point.evEvent->m_name);
				}
				Point.Clear(); // wyjechaliśmy poza poprzednie, można skasować
			}
		}
		else
		{ // jeśli większe to ograniczenie ma swoją długość
			if (Point.fVelNext == min_speed(Point.fVelNext, VelLimitLast) && Point.fVelNext != VelLimitLast)
			{
				// jeśli ograniczenie jest mniejsze niż obecne to obowiązuje od zaraz
				VelLimitLast = Point.fVelNext;
			}
			else if (Point.fDist < -fLength && Point.fVelNext != VelLimitLast)
			{
				// jeśli większe to musi wyjechać za poprzednie
				VelLimitLast = Point.fVelNext;
			}
			else if (Point.fDist < -fLength - Point.fSectionVelocityDist)
			{
				VelLimitLast = -1.0;
				if (Global.iWriteLogEnabled & 8)
				{
					WriteLog("TableUpdate: Event is behind. SVD > 0: " + Point.evEvent->m_name);
				}
				Point.Clear(); // wyjechaliśmy poza poprzednie, można skasować
			}
		}
	}

    //sprawdzenie eventów pasywnych przed nami
    { // zawalidrogi nie ma (albo pojazd jest samochodem), sprawdzić sygnał
        if ( Point.flags & SpeedPointFlag::shuntSignal) // jeśli Tm - w zasadzie to sprawdzić komendę!
        { // jeśli podana prędkość manewrowa
            if( Velocity == 0.0
             && true == OrderCurrentGet().HasAll(DriverOrder::obeyTrain) ) {
                // jeśli tryb pociągowy a tarcze ma ShuntVelocity 0 0
                Velocity = -1; // ignorować, chyba że prędkość stanie się niezerowa
                if( true == Point.flags.HasAll(SpeedPointFlag::elapsed) ) {
                    // a jak przejechana to można usunąć, bo podstawowy automat usuwa tylko niezerowe
                    Point.Clear();
                }
            }
            else if( Command == TCommandType::cm_Unknown && Velocity != 0.0 ) {
                // jeśli jeszcze nie ma komendy
                // komenda jest tylko gdy ma jechać, bo stoi na podstawie tabelki
				// jeśli nie było komendy wcześniej - pierwsza się liczy - ustawianie VelSignal
				Command = TCommandType::cm_ShuntVelocity; // w trybie pociągowym tylko jeśli włącza tryb manewrowy (v!=0.0)
				// Ra 2014-06: (VelSignal) nie może być tu ustawiane, bo Tm może być daleko
				if (VelSignal == 0.0)
				{
					// aby stojący ruszył
					VelSignal = Velocity;
				}
				if (Point.fDist < 0.0)
				{
					// jeśli przejechany
					//!!! ustawienie, gdy przejechany jest lepsze niż wcale, ale to jeszcze nie to
					VelSignal = Velocity;
					// to można usunąć (nie mogą być usuwane w skanowaniu)
					Point.Clear();
				}
			}
        }
        else if( false == Point.flags.HasAny(SpeedPointFlag::sectionSpeed) ) {
            //jeśli jakiś event pasywny ale nie ograniczenie
            if( Command != TCommandType::cm_Unknown )
			{
			}
			else
			{
				// jeśli nie było komendy wcześniej - pierwsza się liczy - ustawianie VelSignal
				if (Velocity < 0.0 || Velocity >= 1.0)
				{
					// bo wartość 0.1 służy do hamowania tylko
					Command = TCommandType::cm_SetVelocity; // może odjechać
					// Ra 2014-06: (VelSignal) nie może być tu ustawiane, bo semafor może być daleko
					// VelSignal=v; //nie do końca tak, to jest druga prędkość; -1 nie wpisywać...
					if (VelSignal == 0.0)
					{
						// aby stojący ruszył
						VelSignal = -1.0;
					}
					if (Point.fDist < 0.0)
					{
						// jeśli przejechany
						VelSignal = Velocity == 0.0 ? 0.0 : -1.0;
						// ustawienie, gdy przejechany jest lepsze niż wcale, ale to jeszcze nie to
						if (Point.flags & SpeedPointFlag::event && (Point.evEvent != eSignSkip || Point.fVelNext != VelRestricted))
						{
							// jeśli event
							// ale inny niż ten, na którym minięto S1, chyba że się już zmieniło
							// sygnał zezwalający na jazdę wyłącza jazdę na widoczność (po S1 na SBL)
							drivingFlags.visibility = false;
							// remove restricted speed
							VelRestricted = -1.0;
						}
						// jeśli nie jest ograniczeniem prędkości to można usunąć
						// (nie mogą być usuwane w skanowaniu)
						TableDropPassedEvent( Point );
					}
				}
				else if (Point.fDist > 0.0 && typeid(*Point.evEvent) == typeid(road_order))
				{
					// a level crossing or a junction which holds the vehicle ahead: the road orders past it, which let the traffic
					// through, aren't for this vehicle yet. without this the first of them would tell the standing vehicle to drive on
					Command = TCommandType::cm_SetProximityVelocity; // taken, and not acted on
				}
				else if (Point.evEvent->is_command())
				{
					// jeśli prędkość jest zerowa, a komórka zawiera komendę
					eSignNext = Point.evEvent; // dla informacji
					// make sure the command isn't for someone else
					// TBD, TODO: revise this check for bank/loose_shunt; we might want to ignore obstacles with no owner in these modes
					if (Point.fDist < Obstacle.distance)
					{
						if (true == drivingFlags.stopHere)
						{
							// jeśli ma stać, dostaje komendę od razu
							Command = TCommandType::cm_Command; // komenda z komórki, do wykonania po zatrzymaniu
						}
						// NOTE: human drivers are likely to stop wherever and expect things to still work, so we give them more leeway
						else if (Point.fDist <= (AIControllFlag ? fMaxProximityDist : std::max(250.0, fMaxProximityDist)))
						{
							// jeśli ma dociągnąć, to niech dociąga
							// (moveStopCloser dotyczy dociągania do W4, nie semafora)
							Command = TCommandType::cm_Command; // komenda z komórki, do wykonania po zatrzymaniu
						}
					}
				}
			}
		}
    } // jeśli nie ma zawalidrogi

    return false;
}

void TController::TablePurger()
{ // odtykacz: usuwa mniej istotne pozycje ze środka tabelki, aby uniknąć zatkania
    //(np. brak ograniczenia pomiędzy zwrotnicami, usunięte sygnały, minięte odcinki łuku)
    if( sSpeedTable.size() < 2 ) {
        return;
    }
    // simplest approach should be good enough for start -- just copy whatever is still relevant, then swap
    auto const isexpendable = []( TSpeedPos const &Speedpoint ) {
        return
            false == Speedpoint.flags.HasAny( SpeedPointFlag::enabled )
         || ( ( Speedpoint.flags & ( SpeedPointFlag::elapsed | SpeedPointFlag::track | SpeedPointFlag::curve | SpeedPointFlag::switchTrack ) ) == ( SpeedPointFlag::elapsed | SpeedPointFlag::track | SpeedPointFlag::curve )
           && Speedpoint.fVelNext < 0.0 ); };
    // do a trial run first, to see if we need to bother at all
    std::size_t trimcount{ 0 };
    for( std::size_t idx = 0; idx < sSpeedTable.size() - 1; ++idx ) {
        // NOTE: we could break out early here, but running through entire thing gives us exact size needed for new table
        if( isexpendable( sSpeedTable[ idx ] ) ) {
            ++trimcount;
        }
    }
    if( trimcount == 0 ) {
        // there'd be no gain, may as well bail
        return;
    }
    std::vector<TSpeedPos> trimmedtable; trimmedtable.reserve( sSpeedTable.size() - trimcount );
    // the indices are only valid for the old table, so the new placements are recorded separately
    // and assigned once the new table is complete
    auto semnextindex { std::size_t( -1 ) };
    auto semnextstopindex { std::size_t( -1 ) };
    auto const trackplacement = [&]( std::size_t const Index, bool const Kept ) {
        // a trimmed point leaves the index invalid, a kept one lands where the new table ends
        if( Index == SemNextIndex ) {
            semnextindex = ( Kept ? trimmedtable.size() : std::size_t( -1 ) );
        }
        if( Index == SemNextStopIndex ) {
            semnextstopindex = ( Kept ? trimmedtable.size() : std::size_t( -1 ) );
        } };

    for( std::size_t idx = 0; idx < sSpeedTable.size() - 1; ++idx ) {
        auto const &speedpoint = sSpeedTable[ idx ];
        auto const kept { false == isexpendable( speedpoint ) };
        trackplacement( idx, kept );
        if( false == kept ) { continue; }
        // we're left with useful speed point record we should copy
        trimmedtable.emplace_back( speedpoint );
    }
    // always copy the last entry; an index pointing at it has to follow it as well
    trackplacement( sSpeedTable.size() - 1, true );
    trimmedtable.emplace_back( sSpeedTable.back() );

    SemNextIndex = semnextindex;
    SemNextStopIndex = semnextstopindex;

    if( Global.iWriteLogEnabled & 8 ) {
        WriteLog( "Speed table garbage collection for " + OwnerName() + " cut away " + std::to_string( trimcount ) + ( trimcount == 1 ? " record" : " records" ) );
    }
    // update the data
    std::swap( sSpeedTable, trimmedtable );
    iLast = sSpeedTable.size() - 1;
};

void TController::TableSort() {

    if( sSpeedTable.size() < 3 ) {
        // we skip last slot and no point in checking if there's only one other entry
        return;
    }
    // the last slot holds the track the scan stopped at and stays where it is.
    // a single bubble pass is not enough: TableTraceRoute() adds the events of a track before the
    // track itself, so a track with k scanned events lands k places too far and needs k passes
    auto const sortedcount { static_cast<std::size_t>( iLast ) };
    auto const inrange = [&]( std::size_t const Index ) {
        return Index < sortedcount; };
    // the placements point at semaphor entries; remember which ones, the sort moves them around
    auto const *semnextevent { inrange( SemNextIndex ) ? sSpeedTable[ SemNextIndex ].evEvent : nullptr };
    auto const *semnextstopevent { inrange( SemNextStopIndex ) ? sSpeedTable[ SemNextStopIndex ].evEvent : nullptr };

    std::stable_sort(
        std::begin( sSpeedTable ), std::begin( sSpeedTable ) + sortedcount,
        []( TSpeedPos const &Left, TSpeedPos const &Right ) {
            return Left.fDist < Right.fDist; } );

    auto const relocate = [&]( basic_event const *Event, std::size_t &Index ) {
        if( Event == nullptr ) { return; }
        auto const lookup {
            std::find_if(
                std::begin( sSpeedTable ), std::begin( sSpeedTable ) + sortedcount,
                [Event]( TSpeedPos const &Point ) {
                    return Point.evEvent == Event; } ) };
        Index = (
            lookup != std::begin( sSpeedTable ) + sortedcount ?
                static_cast<std::size_t>( std::distance( std::begin( sSpeedTable ), lookup ) ) :
                std::size_t( -1 ) );
    };
    relocate( semnextevent, SemNextIndex );
    relocate( semnextstopevent, SemNextStopIndex );
}
