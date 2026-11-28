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

// the AI driver: construction, its update, taking over a vehicle, what it tells about itself

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

const double HardReactionTime = 0.2;
const double EasyAcceleration = 0.85; //[m/ss]
const double HardAcceleration = 9.81;
bool WriteLogFlag = false;

//---------------------------------------------------------------------------

TController::TController(bool AI, TDynamicObject *NewControll, bool InitPsyche, bool Primary) :// czy ma aktywnie prowadzić?
              AIControllFlag( AI ),     pVehicle( NewControll )
{
    if( pVehicle != nullptr ) {
        pVehicles[ end::front ] = pVehicle->GetFirstDynamic( end::front ); // pierwszy w kierunku jazdy (Np. Pc1)
        pVehicles[ end::rear ] = pVehicle->GetFirstDynamic( end::rear ); // ostatni w kierunku jazdy (końcówki)
    }
    else {
        pVehicles[ end::front ] = nullptr;
        pVehicles[ end::rear ] = nullptr;
    }
    ControllingSet(); // utworzenie połączenia do sterowanego pojazdu
    if( mvOccupied != nullptr ) {
        iDirectionOrder = mvOccupied->CabActive; // 1=do przodu (w kierunku sprzęgu 0)
        VehicleName = mvOccupied->Name;

        if( is_car() ) { // samochody: na podst. http://www.prawko-kwartnik.info/hamowanie.html
            // fDriverBraking=0.0065; //mnożone przez (v^2+40*v) [km/h] daje prawie drogę hamowania [m]
            fDriverBraking = 0.03; // coś nie hamują te samochody zbyt dobrze
            fDriverDist = 5.0; // 5m - zachowywany odstęp przed kolizją
            fVelPlus = 10.0; // dopuszczalne przekroczenie prędkości na ograniczeniu bez hamowania
            fVelMinus = 2.0; // margines prędkości powodujący załączenie napędu
        }
        else { // pociągi i statki
            fDriverBraking = 0.06; // mnożone przez (v^2+40*v) [km/h] daje prawie drogę hamowania [m]
            fDriverDist = 50.0; // 50m - zachowywany odstęp przed kolizją
            fVelPlus = 5.0; // dopuszczalne przekroczenie prędkości na ograniczeniu bez hamowania
            fVelMinus = 5.0; // margines prędkości powodujący załączenie napędu
        }

        // fAccThreshold może podlegać uczeniu się - hamowanie powinno być rejestrowane, a potem analizowane
        // próg opóźnienia dla zadziałania hamulca
        fAccThreshold = is_emu() ? -0.55 : is_dmu() ? -0.45 : -0.2;
/*
        // HACK: emu with induction motors need to start their braking a bit sooner than the ones with series motors
        if( ( mvOccupied->TrainType == train_type::emu )
         && ( mvControlling->EngineType == TEngineType::ElectricInductionMotor ) ) {
            fAccThreshold += 0.10;
        }
*/
    }
    // TrainParams=NewTrainParams;
    // if (TrainParams)
    // asNextStop=TrainParams.NextStop();
    // else
    TrainParams = TTrainParameters("none"); // rozkład jazdy
    // OrderCommand="";
    // OrderValue=0;
    OrdersClear();

    if( true == Primary ) {
        drivingFlags.primary = true; // aktywnie prowadzące pojazd
    }

    if( mvOccupied != nullptr ) {
        SetDriverPsyche(); // na końcu, bo wymaga ustawienia zmiennych
    }
    TableClear();

    // the logs of all, begun at once as they always were. one of a train only at its first line (update_logs): every
    // crew is made primary here, and the drivers but one of a train with crews in more vehicles are made passive
    // before it starts, so they get no file with only the header
    if( WriteLogFlag ) {
        begin_physicslog();
    }

    // HACK: give the simulation a small window to potentially replace the AI with a human driver
    fActionTime = -5.0;
};

TController::~TController()
{ // wykopanie mechanika z roboty
};

TBrakeSystem TController::consist_brake_system() const {

    if( mvOccupied->BrakeSystem != TBrakeSystem::ElectroPneumatic ) { return mvOccupied->BrakeSystem; }

    auto isepcapable = true;
    if( pVehicles[ end::front ] != pVehicles[ end::rear ] ) {
        // more detailed version, will use manual braking also for coupled sets of controlled vehicles
        auto *vehicle = pVehicles[ end::front ]; // start from first
        while( true == isepcapable
            && vehicle != nullptr ) {
            // NOTE: we could simplify this by doing only check of the rear coupler, but this can be quite tricky in itself
            // TODO: add easier ways to access front/rear coupler taking into account vehicle's direction
            isepcapable = (vehicle->MoverParameters->Couplers[end::front].Connected == nullptr ||
			               (vehicle->MoverParameters->Couplers[end::front].CouplingFlag & Coupling::control && vehicle->MoverParameters->Couplers[end::front].Connected->Power > -1)) &&
			              (vehicle->MoverParameters->Couplers[end::rear].Connected == nullptr ||
			               (vehicle->MoverParameters->Couplers[end::rear].CouplingFlag & Coupling::control && vehicle->MoverParameters->Couplers[end::rear].Connected->Power > -1));
            vehicle = vehicle->Next(); // kolejny pojazd, podłączony od tyłu (licząc od czoła)
        }
    }
    return isepcapable ? TBrakeSystem::ElectroPneumatic : TBrakeSystem::Pneumatic;
}

std::pair<int, double> TController::next_speed_limit() const {

    auto const speedlimit { static_cast<int>( VelDesired ) };
    auto nextspeedlimit { speedlimit };
    auto nextspeedlimitdistance { std::numeric_limits<double>::max() };
    if( speedlimit == 0 ) {
        // if we aren't allowed to move then any next speed limit is irrelevant
        return { nextspeedlimit, nextspeedlimitdistance };
    }
    // nie przekraczać rozkladowej
    auto const schedulespeedlimit { (
        OrderCurrentGet().HasAny( DriverOrder::obeyTrain | DriverOrder::bank ) && TrainParams.TTVmax > 0.0 ? static_cast<int>( TrainParams.TTVmax ) :
        false == OrderCurrentGet().HasAny( DriverOrder::obeyTrain | DriverOrder::bank ) ? static_cast<int>( fShuntVelocity ) :
            -1 ) };
    // first take note of any speed change which should occur after passing potential current speed limit
    if( VelLimitLastDist.second > 0 ) {
        nextspeedlimit = min_speed( schedulespeedlimit, static_cast<int>( VelLimitLastDist.first ) );
        nextspeedlimitdistance = VelLimitLastDist.second;
    }
    // then take into account speed change ahead, compare it with speed after potentially clearing last limit
    // lower of these two takes priority; otherwise limit lasts at least until potential last limit is cleared
    auto const noactivespeedlimit { VelLimitLastDist.second < 0 };
    if( auto const speedatproximitydistance { min_speed( schedulespeedlimit, static_cast<int>( VelNext ) ) }; speedatproximitydistance == nextspeedlimit ) {
        if( noactivespeedlimit ) {
            nextspeedlimit = speedatproximitydistance;
            nextspeedlimitdistance = ActualProximityDist;
        }
    }
    else if( speedatproximitydistance < nextspeedlimit ) {
        // if the speed limit ahead is more strict than our current limit, it's important enough to report
        if( speedatproximitydistance < VelDesired ) {
            nextspeedlimit = speedatproximitydistance;
            nextspeedlimitdistance = ActualProximityDist;
        }
        // otherwise report it only if it's located after our current (lower) limit ends
        else if( ActualProximityDist > nextspeedlimitdistance ) {
            nextspeedlimit = speedatproximitydistance;
            nextspeedlimitdistance = ActualProximityDist;
        }
    }
    else if( noactivespeedlimit ) { // implicit proximity > last, report only if last limit isn't present
        nextspeedlimit = speedatproximitydistance;
        nextspeedlimitdistance = ActualProximityDist;
    }
    // HACK: if our current speed limit extends beyond our scan range don't display potentially misleading information about its length
    if( nextspeedlimitdistance >= EU07_AI_SPEEDLIMITEXTENDSBEYONDSCANRANGE ) {
        nextspeedlimit = speedlimit;
    }
    // HACK: hide next speed limit if the 'limit' is a vehicle in front of us
    else if( ActualProximityDist == std::abs( TrackObstacle() ) ) {
        nextspeedlimit = speedlimit;
    }
    return { nextspeedlimit, nextspeedlimitdistance };
}

blackbox::ai_reference TController::ai_reference() const {

    blackbox::ai_reference ai;
    ai.target_kmh = static_cast<float>( VelDesired );
    if( auto const [ limit, distance ] { next_speed_limit() }; limit != static_cast<int>( VelDesired ) ) {
        ai.next_limit_kmh = static_cast<float>( limit );
        ai.next_limit_distance = static_cast<float>( distance );
    }
    ai.acceleration = static_cast<float>( AccDesired );
    ai.braking_distance = static_cast<float>( fBrakeDist );
    ai.timetable_vmax_kmh = ( TrainParams.TTVmax > 0.0 ? static_cast<float>( TrainParams.TTVmax ) : -1.0f );
    ai.track_vmax_kmh = static_cast<float>( mvOccupied->RunningTrack.Velmax );
    return ai;
}

void TController::record_station( TSpeedPos const &Point, std::uint8_t const Kind ) const {

    if( Point.evEvent == nullptr ) { return; }
    blackbox::station_event entry;
    entry.kind = Kind;
    auto const row { TrainParams.StationIndex };
    entry.row = static_cast<std::uint16_t>( std::max( 0, row ) );
    entry.station = asNextStop;
    entry.position = Point.evEvent->input_location();
    // the stop point may be moved back from the indicator, to where the head of the train is to stop
    entry.distance = static_cast<float>( Point.fDist + Point.fMoved );
    entry.stop_at = static_cast<float>( Point.fMoved );
    entry.train_length = static_cast<float>( fLength );
    entry.platform = static_cast<float>( Point.evEvent->input_value( 2 ) );
    if( row >= 0 && row <= TrainParams.StationCount ) {
        auto const &line { TrainParams.TimeTable[ row ] };
        entry.arrival = ( line.Ah >= 0 ? static_cast<float>( line.Ah * 60 ) + line.Am : -1.0f );
        entry.departure = ( line.Dh >= 0 ? static_cast<float>( line.Dh * 60 ) + line.Dm : -1.0f );
    }
    simulation::Blackbox.station( pVehicles[ end::front ], std::move( entry ) );
}

void TController::WaitingSet(double Seconds)
{ // ustawienie odczekania po zatrzymaniu (ustawienie w trakcie jazdy zatrzyma)
    fStopTime = -Seconds; // ujemna wartość oznacza oczekiwanie (potem >=0.0)
}

void TController::SetVelocity(double NewVel, double NewVelNext, TStopReason r)
{ // ustawienie nowej prędkości
    WaitingTime = -WaitingExpireTime; // przypisujemy -WaitingExpireTime, a potem porównujemy z zerem
    if (NewVel == 0.0) { // jeśli ma stanąć
        if (r != stopNone) // a jest powód podany
            eStopReason = r; // to zapamiętać nowy powód
    }
    else {
        eStopReason = stopNone; // podana prędkość, to nie ma powodów do stania
        // to całe poniżej to warunki zatrąbienia przed ruszeniem
        if( is_train() // tylko pociągi trąbią (unimogi tylko na torach, więc trzeba raczej sprawdzać tor)
         && OrderCurrentGet().HasAny(DriverOrder::shunt | DriverOrder::looseShunt | DriverOrder::obeyTrain | DriverOrder::bank | DriverOrder::connect | DriverOrder::prepareEngine) // jeśli jedzie w dowolnym trybie
         && drivingFlags.startHorn // jezeli trąbienie włączone
         && ( false == drivingFlags.startHornDone && false == drivingFlags.connect ) // jeśli nie zatrąbione i nie jest to moment podłączania składu
         && mvOccupied->Vel < 1.0 // jesli stoi (na razie, bo chyba powinien też, gdy hamuje przed semaforem)
         && ( NewVel >= 1.0 || NewVel < 0.0 ) ) { // o ile prędkość jest znacząca zatrąb po odhamowaniu
            drivingFlags.startHornNow = true;
        }
    }
    VelSignal = NewVel; // prędkość zezwolona na aktualnym odcinku
    VelNext = NewVelNext; // prędkość przy następnym obiekcie
}

double TController::BrakeAccFactor() const
{
	double Factor = 1.0;

    if( fAccThreshold != 0.0
     && AccDesired < 0.0
     && ( ActualProximityDist > fMinProximityDist
       || mvOccupied->Vel > VelDesired + fVelPlus ) ) {
        Factor += fBrakeReaction * (/*mvOccupied->BrakeCtrlPosR*/ BrakeCtrlPosition < 0.5 ? 1.5 : 1) * mvOccupied->Vel / ( std::max( 0.0, ActualProximityDist ) + 1 ) * ( ( AccDesired - AbsAccS ) / fAccThreshold );
    }
/*
	if (mvOccupied->TrainType == train_type::dmu && mvOccupied->Vel > 40 && VelNext<40)
		Factor *= 1 + 0.25 * ( (1600 - VelNext * VelNext) / (mvOccupied->Vel * mvOccupied->Vel) );
*/
	return Factor;
}

void TController::SetDriverPsyche() {

    if (Psyche == Aggressive && OrderCurrentGet() == DriverOrder::obeyTrain) {
        ReactionTime = HardReactionTime; // w zaleznosci od charakteru maszynisty
        if (is_car()) {
            WaitingExpireTime = 1; // tyle ma czekać samochód, zanim się ruszy
            AccPreferred = 3.0; //[m/ss] agresywny
        }
        else {
            WaitingExpireTime = 61; // tyle ma czekać, zanim się ruszy
            AccPreferred = HardAcceleration; // agresywny
        }
    }
    else {
        ReactionTime = EasyReactionTime; // spokojny
        if (is_car()) {
            WaitingExpireTime = 3; // tyle ma czekać samochód, zanim się ruszy
            AccPreferred = 2.0; //[m/ss]
        }
        else {
            WaitingExpireTime = 65; // tyle ma czekać, zanim się ruszy
            AccPreferred = EasyAcceleration;
        }
    }
}

void
TController::Update( double const Timedelta ) {
    // uruchamiać przynajmniej raz na sekundę
    if( ( false == drivingFlags.primary ) ) { return; } // pasywny nic nie robi
    if( false == simulation::is_ready )       { return; }

    update_timers( Timedelta );
    update_logs( Timedelta );

    auto const reactiontime { std::min( ReactionTime, 2.0 ) };

    if( LastReactionTime < reactiontime ) { return; }
    LastReactionTime -= reactiontime;
/*
    // TBD, TODO: put this in an appropriate place, or get rid of it
    // NOTE: this section moved all cars to the edge of their respective roads
    // it may have some use eventually for collision avoidance,
    // but when enabled all the time it produces silly effect
    if( mvOccupied->Vel > 1.0 ) {
        // przy prowadzeniu samochodu trzeba każdą oś odsuwać oddzielnie, inaczej kicha wychodzi
        if (mvOccupied->CategoryFlag & 2) // jeśli samochód
            // if (fabs(mvOccupied->OffsetTrackH)<mvOccupied->Dim.W) //Ra: szerokość drogi tu powinna być?
            if (!mvOccupied->ChangeOffsetH(-0.01 * mvOccupied->Vel * dt)) // ruch w poprzek drogi
                mvOccupied->ChangeOffsetH(0.01 * mvOccupied->Vel * dt); // Ra: co to miało być, to nie wiem
    }
*/
    update_hints();
    // main ai update routine
    determine_consist_state();
    determine_braking_distance();
    determine_proximity_ranges();
    // vicinity check
    auto const awarenessrange {
        std::max(
            750.0,
            mvOccupied->Vel > EU07_AI_MOVEMENT ?
                400 + fBrakeDist :
                30.0 * fDriverDist ) }; // 1500m dla stojących pociągów;
    if( is_active() ) {
        scan_route( awarenessrange );
    }
    scan_obstacles( awarenessrange );
    // generic actions
    control_security_system( reactiontime );
    if( iEngineActive ) {
        control_wheelslip();
        control_horns( reactiontime );
        control_pantographs();
        if( fActionTime > 0.0 ) {
            // potentially delayed and/or low priority actions
            control_lights();
            control_doors();
            control_compartment_lights();
        }
    }
    CheckTimeControllers();
    // external command actions
    UpdateCommand();
    // mode specific actions
    handle_engine();
    handle_orders();
    // a brake the wagon inspector left to cut out while the train was moving
    reinspect_brakes();
    // situational velocity and acceleration adjustments
    pick_optimal_speed( awarenessrange );
    if( iEngineActive ) {
        control_tractive_and_braking_force();
    }
    SetTimeControllers();
}

//-----------koniec skanowania semaforow

void TController::TakeControl( bool const Aidriver, bool const Forcevehiclecheck )
{ // przejęcie kontroli przez AI albo oddanie
    if (AIControllFlag == Aidriver && !Forcevehiclecheck)
        return; // już jest jak ma być
    if( AIControllFlag != Aidriver ) {
        // the autopilot switched on or off
        simulation::Blackbox.driver_changed( pVehicle, Aidriver );
    }
    if (Aidriver) //żeby nie wykonywać dwa razy
    { // teraz AI prowadzi
        AIControllFlag = AIdriver;
        pVehicle->Controller = AIdriver;
        // the AI doesn't operate the headlight dimmer, drop the state left by the human driver
        // so the lights set by the AI aren't hidden (dimmer in off position) or dimmed
        pVehicle->HeadlightsAoff = false;
        pVehicle->HeadlightsBoff = false;
        pVehicle->DimHeadlights = false;
        pVehicle->HighBeamLights = false;
		control_lights();   // reinicjalizacja swiatel
		mvOccupied->CabActivisation(true);
        iDirection = 0; // kierunek jazdy trzeba dopiero zgadnąć
        TableClear(); // ponowne utworzenie tabelki, bo człowiek mógł pojechać niezgodnie z sygnałami
        if( action() != TAction::actSleep ) {
            // gdy zgaszone światła, flaga podjeżdżania pod semafory pozostaje bez zmiany
            // conditional below disabled to get around the situation where the AI train does nothing ever
            // because it is waiting for orders which don't come until the engine is engaged, i.e. effectively never
            if( OrderCurrentGet() ) {
                // jeśli coś robi
                PrepareEngine(); // niech sprawdzi stan silnika
            }
            else {
                // jeśli nic nie robi
                OrderNext( DriverOrder::prepareEngine );
                if( pVehicle->MoverParameters->iLights[ ( mvOccupied->CabActive < 0 ?
                        end::rear :
                        end::front ) ]
                    & ( light::headlight_left | light::headlight_right | light::headlight_upper ) ) // któreś ze świateł zapalone?
                { // od wersji 357 oczekujemy podania komend dla AI przez scenerię
/*
                    if( pVehicle->MoverParameters->iLights[ mvOccupied->CabActive < 0 ? end::rear : end::front ] & light::headlight_upper ) // górne światło zapalone
                        OrderNext( DriverOrder::obeyTrain ); // jazda pociągowa
                    else
                        OrderNext( DriverOrder::shunt ); // jazda manewrowa
*/
                    if( mvOccupied->Vel >= 1.0 ) // jeśli jedzie (dla 0.1 ma stać)
                        drivingFlags.stopHere = false; // to ma nie czekać na sygnał, tylko jechać
                    else
                        drivingFlags.stopHere = true; // a jak stoi, to niech czeka
                }
            }
            CheckVehicles(); // ustawienie świateł
        }
    }
    else
    { // a teraz użytkownik
		if (eAction == TAction::actSleep || OrderCurrentGet() == DriverOrder::waitForOrders) // gasimy swiatla jesli przejmujemy odstawione
			pVehicle->RaLightsSet(0, 0);
        auto const aiwasdriving { AIControllFlag == AIdriver };
        if( aiwasdriving ) {
            // the horn, sandbox, departure signal and brake releaser the AI may have latched are momentary for the human driver
            handover::release_ai_controls( *mvOccupied, *mvControlling );
            fWarningDuration = 0.0;
            remove_hint( driver_hint::hornon );
            // the AI sets the lights directly, leave the selector in matching position so it doesn't revert them later
            sync_lights_selector();
        }
        AIControllFlag = Humandriver;
        pVehicle->Controller = Humandriver;
        if( aiwasdriving ) {
            // same with the headlight dimmer, which is only applied for the human driver
            sync_lights_dimmer();
        }
        if( eAction == TAction::actSleep ) {
            eAction = TAction::actUnknown;
        }
        if( Forcevehiclecheck ) {
            // update consist ownership and other consist data
            CheckVehicles();
        }
    }
};

Mtable::TTrainParameters const &
TController::TrainTimetable() const {
    return TrainParams;
}

std::string TController::Relation() const
{ // zwraca relację pociągu
    return TrainParams.ShowRelation();
};

const std::string& TController::TrainName() const
{ // zwraca numer pociągu
    return TrainParams.TrainName;
};

int TController::StationCount() const
{ // zwraca ilość stacji (miejsc zatrzymania)
    return TrainParams.StationCount;
};

int TController::StationIndex() const
{ // zwraca indeks aktualnej stacji (miejsca zatrzymania)
    return TrainParams.StationIndex;
};

bool TController::IsStop() const
{ // informuje, czy jest zatrzymanie na najbliższej stacji
    return TrainParams.IsStop();
};

std::string TController::TableText( std::size_t const Index ) const
{ // pozycja tabelki prędkości
    if( Index < sSpeedTable.size() ) {
        return sSpeedTable[ Index ].TableText();
    }
    else {
        return "";
    }
};

int TController::CrossRoute(TTrack *tr)
{ // zwraca numer segmentu dla skrzyżowania (tr)
    // pożądany numer segmentu jest określany podczas skanowania drogi
    // droga powinna być określona sposobem przejazdu przez skrzyżowania albo współrzędnymi miejsca
    // docelowego
    for( std::size_t i = 0; i < sSpeedTable.size(); ++i )
    { // trzeba przejrzeć tabelę skanowania w poszukiwaniu (tr)
        // i jak się znajdzie, to zwrócić zapamiętany numer segmentu i kierunek przejazdu
        // (-6..-1,1..6)
        if( true == sSpeedTable[i].flags.HasAll(SpeedPointFlag::enabled | SpeedPointFlag::track)
         && sSpeedTable[i].trTrack == tr ) {
            // jeśli pozycja odpowiadająca skrzyżowaniu (tr)
            return sSpeedTable[i].crossRoute; // -8..7
        }
    }
    return 0; // nic nie znaleziono?
};
/*
void TController::RouteSwitch(int d)
{ // ustawienie kierunku jazdy z kabiny
    d &= 3;
    if( ( d != 0 )
     && ( iRouteWanted != d ) ) { // nowy kierunek
        iRouteWanted = d; // zapamiętanie
        if( mvOccupied->CategoryFlag & 2 ) {
            // jeśli samochód
            for( std::size_t i = 0; i < sSpeedTable.size(); ++i ) {
                // szukanie pierwszego skrzyżowania i resetowanie kierunku na nim
                if( true == sSpeedTable[ i ].flags.HasAll(spEnabled | spTrack) ) {
                    // jeśli pozycja istotna (1) oraz odcinek (2)
                    if( false == sSpeedTable[ i ].flags.HasAll(spElapsed) ) {
                        // odcinek nie może być miniętym
                        if( sSpeedTable[ i ].trTrack->eType == tt_Cross ) // jeśli skrzyżowanie
                        {
                            while( sSpeedTable.size() >= i ) {
                                // NOTE: we're ignoring semaphor flags and not resetting them like we do for train route trimming
                                // but what if there's street lights?
                                // TODO: investigate
                                sSpeedTable.pop_back();
                            }
                            iLast = sSpeedTable.size();
                        }
                    }
                }
            }
        }
    }
};
*/
std::string TController::OwnerName() const
{
    return pVehicle ? pVehicle->MoverParameters->Name : "none";
};

bool TController::is_train() const {
    return TestFlag( mvOccupied->CategoryFlag, 1 ); }

bool TController::is_car() const {
    return TestFlag( mvOccupied->CategoryFlag, 2 ); }

bool TController::is_emu() const {
    return mvControlling->TrainType == train_type::emu; }

bool TController::is_dmu() const {
    return mvControlling->TrainType == train_type::dmu; }

bool TController::has_diesel_engine() const {
    return mvControlling->EngineType == TEngineType::DieselElectric || mvControlling->EngineType == TEngineType::DieselEngine; }

double TController::DirectionalVel() const {
    return mvOccupied->Vel * sign( iDirection * mvOccupied->V ); }

std::vector<std::string_view> DrivingFlags::ActiveNames() const {
    std::vector<std::pair<bool, std::string_view>> const flags {
        { stopCloser, "StopCloser" }, { stopPoint, "StopPoint" }, { active, "Active" }, { press, "Press" },
        { connect, "Connect" }, { primary, "Primary" }, { late, "Late" }, { stopHere, "StopHere" },
        { startHorn, "StartHorn" }, { startHornNow, "StartHornNow" }, { startHornDone, "StartHornDone" },
        { oerlikons, "Oerlikons" }, { incSpeed, "IncSpeed" }, { trackEnd, "TrackEnd" }, { switchFound, "SwitchFound" },
        { guardSignal, "GuardSignal" }, { visibility, "Visibility" }, { doorOpened, "DoorOpened" },
        { pushPull, "PushPull" }, { signalFound, "SignalFound" }, { stopPointFound, "StopPointFound" },
        { guardOpenDoor, "GuardOpenDoor" }, { departureWarned, "DepartureWarned" } };
    std::vector<std::string_view> names;
    for( auto const &[ set, name ] : flags ) {
        if( set ) {
            names.emplace_back( name );
        }
    }
    return names;
}
