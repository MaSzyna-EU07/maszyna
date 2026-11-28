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

// the speed the driver aims for

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

const double PassengetTrainAcceleration = 0.40;
const double HeavyPassengetTrainAcceleration = 0.20;
const double CargoTrainAcceleration = 0.25;
const double HeavyCargoTrainAcceleration = 0.10;

// modifies brake distance for low target speeds, to ease braking rate in such situations
float
TController::braking_distance_multiplier( float const Targetvelocity ) const {

    auto const frictionmultiplier { 1.f / Global.FrictionWeatherFactor };

    if( Targetvelocity > 65.f ) { return 1.f * frictionmultiplier; }
    if( Targetvelocity < 5.f ) {
        // HACK: engaged automatic transmission means extra/earlier braking effort is needed for the last leg before full stop
        if( is_dmu()
         && mvOccupied->Vel < 40.0
         && Targetvelocity == 0.f ) {
            auto const multiplier { std::clamp( 1.f + iVehicles * 0.5f, 2.f, 4.f ) };
            return std::lerp(multiplier, 1.f, static_cast<float>(mvOccupied->Vel / 40.0)) * frictionmultiplier;
        }
        // HACK: cargo trains or trains going downhill with high braking threshold need more distance to come to a full stop
        if( fBrake_a0[1] > 0.2
         && ( true == IsCargoTrain
           || fAccGravity > 0.025 ) ) {
            return std::lerp(1.f, 2.f, std::clamp((fBrake_a0[1] - 0.2) / 0.2, 0.0, 1.0)) * frictionmultiplier;
        }

        return 1.f * frictionmultiplier;
    }
    // stretch the braking distance up to 3 times; the lower the speed, the greater the stretch
    return std::lerp(3.f, 1.f, (Targetvelocity - 5.f) / 60.f) * frictionmultiplier;
}

double TController::ESMVelocity(bool Main)
{
	double fCurrentCoeff = 0.9;
	double fFrictionCoeff = 0.85;
	double ESMVel = 9999;
	int MCPN = mvControlling->MainCtrlActualPos;
	int SCPN = mvControlling->ScndCtrlActualPos;
	if (Main)
		MCPN += 1;
	else
		SCPN += 1;
	if (mvControlling->RList[MCPN].ScndAct < 255 && mvControlling->ScndCtrlActualPos == 0)
		SCPN = mvControlling->RList[MCPN].ScndAct;
	double FrictionMax = mvControlling->Mass*9.81*mvControlling->Adhesive(mvControlling->RunningTrack.friction)*fFrictionCoeff;
	double IF = mvControlling->Imax;
	double MS = 0;
	double Fmax = 0;
	for (int i = 0; i < 5; i++)
	{
		MS = mvControlling->MomentumF(IF, IF, SCPN);
		Fmax = MS * mvControlling->RList[MCPN].Bn * mvControlling->RList[MCPN].Mn * 2 / mvControlling->WheelDiameter * mvControlling->Transmision.Ratio;
        if( Fmax != 0.0 ) {
            IF = 0.5 * IF * ( 1 + FrictionMax / Fmax );
        }
        else {
            // NOTE: gets trimmed to actual highest acceptable value after the loop
            IF = std::numeric_limits<double>::max();
            break;
        }
	}
	IF = std::min(IF, mvControlling->Imax*fCurrentCoeff);
	double R = mvControlling->RList[MCPN].R + mvControlling->CircuitRes + mvControlling->RList[MCPN].Mn*mvControlling->WindingRes;
	// SCPN may come from the .fiz (RList ScndAct) unchecked: read it with the bounds-safe accessor, as MomentumF() does
	auto const &motorparam{mvControlling->motor_param(SCPN)};
	double pole = motorparam.fi *
		std::max(abs(IF) / (abs(IF) + motorparam.Isat) - motorparam.fi0, 0.0);
	double Us = abs(mvControlling->EngineVoltage) - IF*R;
	double ns = std::max(0.0, Us / (pole*mvControlling->RList[MCPN].Mn));
	ESMVel = ns * mvControlling->WheelDiameter*M_PI*3.6/mvControlling->Transmision.Ratio;
	return ESMVel;
}

// the stages of uncoupling: each one sets the speed for the next
void
TController::set_velocity_for_uncoupling() {

    if( iVehicleCount < 0 ) {
        // 4th stage: restore initial direction
        if( drivingFlags.press ) {
            SetVelocity( 0, 0, stopJoin );
        }
        return;
    }
    // 3rd stage: compress buffers and uncouple
    if( drivingFlags.press && iDirection == iDirectionOrder ) {
        SetVelocity( 2, 0 ); // jazda w ustawionym kierunku z prędkością 2
        return;
    }
    // 1st stage: bring it to stop
    // 2nd stage: apply consist brakes and change direction
    SetVelocity( 0, 0, stopJoin );
}

// uphill, the gradient alone brings a slow train to a stand within metres, far sooner than a released
// brake applies; so the train brake goes on while the train is coming to a stand, stays on while it
// stands, and is applied for real if the train has started to roll back
void
TController::hold_train_coming_to_stand_uphill() {

    if( ( false == is_train() )
     || ( false == OrderCurrentGet().HasAny(DriverOrder::obeyTrain) )
     || ( VelNext != 0.0 )
     || ( ActualProximityDist > fMaxProximityDist )
     || ( fAccGravity >= EU07_AI_UPHILLGRAVITY ) ) {
        return;
    }
    auto const vel { DirectionalVel() };
    if( std::abs( vel ) < EU07_AI_NOMOVEMENT ) {
        VelDesired = 0.0;
        AccDesired = std::min( AccDesired, EU07_AI_NOACCELERATION );
        return;
    }
    if( ( vel < 0.0 )
     || ( ( vel < 5.0 ) && ( AccDesired < 0.0 ) )
     // rolling to a stand on the gradient alone, the brake has to be on its way before the train stops:
     // once the time left to the stand is no longer than half of what the brake takes to apply, apply it; any
     // sooner and the brake takes off more of the distance than the stop was planned with
     || ( ( BrakeSystem == TBrakeSystem::Pneumatic )
       // not for a train that is short of where it is to stop and is to pull up further
       && ( AccDesired <= 0.0 )
       && ( vel / 3.6 / -fAccGravity <= 0.5 * brake_response_horizon( mvOccupied, BrakeSystem, 1.0, fBrake_a0[ 0 ] ) ) ) ) {
        // the gradient alone already slows the train down nearly this much; ask for enough on top of it that
        // the brake is applied for sure, not held back by the margin against small corrections
        AccDesired = std::min( AccDesired, fAccGravity - std::max( 0.15, fBrake_a0[ 0 ] ) - brake_increase_margin() );
    }
}

void
TController::pick_optimal_speed( double const Range ) {

//    if( false == OrderCurrentGet().HasAny( Shunt | Loose_shunt | Obey_train | Bank | Connect | Disconnect | Change_direction ) ) { return; }

    // set initial velocity and acceleration values
    SetDriverPsyche(); // ustawia AccPreferred (potrzebne tu?)
    VelDesired = fVelMax; // wstępnie prędkość maksymalna dla pojazdu(-ów), będzie następnie ograniczana
    AccDesired = AccPreferred; // AccPreferred wynika z osobowości mechanika

    // nie przekraczać rozkladowej
    if( OrderCurrentGet().HasAny(DriverOrder::obeyTrain)
     && TrainParams.TTVmax > 0.0 ) {
        VelDesired =
            min_speed(
                VelDesired,
                TrainParams.TTVmax );
    }

//    VelNext = VelDesired; // maksymalna prędkość wynikająca z innych czynników niż trajektoria ruchu
    // HACK: -1 means we can pick up 0 speed limits in ( point.velnext > velnext ) tests aimed to find highest next speed,
    // but also doubles as 'max speed' in min_speed tests, so it shouldn't interfere if we don't find any speed limits
    VelNext = -1.0;

    // basic velocity and acceleration adjustments
    // jeśli manewry, to ograniczamy prędkość
    if( false == OrderCurrentGet().HasAny( DriverOrder::obeyTrain | DriverOrder::bank ) ) { // spokojne manewry
        SetVelocity( fShuntVelocity, fShuntVelocity );
        VelDesired =
            min_speed(
                VelDesired,
                fShuntVelocity );
    }
    // uncoupling mode changes velocity/acceleration between stages
    if( OrderCurrentGet().HasAny(DriverOrder::disconnect) ) {
        set_velocity_for_uncoupling();
    }

    VelLimitLastDist = { VelDesired, -1 };
    SwitchClearDist = -1;
    ActualProximityDist = Range; // funkcja Update() może pozostawić wartości bez zmian

    // if we're idling bail out early
    if( false == is_active() ) {
        VelDesired = 0.0;
        VelNext = 0.0;
        AccDesired = std::min( AccDesired, EU07_AI_NOACCELERATION );
        return;
    }

    // Ra: odczyt (ActualProximityDist), (VelNext) i (AccPreferred) z tabelki prędkosci
    check_route_ahead( Range );

    check_departure();

    // if ordered to turn off the vehicle, try to stop
    if( true == OrderCurrentGet().HasAll(DriverOrder::releaseEngine) ) {
        SetVelocity( 0, 0, stopSleep );
    }
    // if ordered to change direction, try to stop
    if( true == OrderCurrentGet().HasAll(DriverOrder::changeDirection) ) {
        SetVelocity( 0, 0, stopDir );
    }

    adjust_desired_speed_for_obstacles();
    adjust_desired_speed_for_limits();
    adjust_desired_speed_for_target_speed( Range );
    adjust_desired_speed_for_current_speed();
    adjust_desired_speed_for_braking_test();

    // Ra 2F1I: wyłączyć kiedyś to uśrednianie i przeanalizować skanowanie, czemu migocze
    if( AccDesired > EU07_AI_NOACCELERATION ) { // hamowania lepeiej nie uśredniać
        AccDesired = fAccDesiredAv =
            0.2 * AccDesired +
            0.8 * fAccDesiredAv; // uśrednione, żeby ograniczyć migotanie
    }
    if( VelDesired == 0.0 ) {
        // Ra 2F1J: jeszcze jedna prowizoryczna łatka
        AccDesired = std::min( AccDesired, EU07_AI_NOACCELERATION );
    }

    // last step sanity check, until the whole calculation is straightened out
    AccDesired = std::min( AccDesired, AccPreferred );
    AccDesired = std::clamp(
        AccDesired,
            is_car() ? -2.0 : -0.9,
            is_car() ? 2.0 : 0.9 );

    // done last, so that none of the adjustments above, made for a train moving forward, can undo it
    hold_train_coming_to_stand_uphill();

    // if the route ahead is blocked we might need to head the other way
    check_route_behind( 1000 ); // NOTE: legacy scan range value
}

void
TController::adjust_desired_speed_for_obstacles() {
    // prędkość w kierunku jazdy, ujemna gdy jedzie w przeciwną stronę niż powinien
    auto const vel { DirectionalVel() };

    if (VelDesired < 0.0)
        VelDesired = fVelMax; // bo VelDesired<0 oznacza prędkość maksymalną

    // Ra: jazda na widoczność
    if( Obstacle.distance < 5000 ) {
        // mamy coś z przodu
        // prędkość pojazdu z przodu (zakładając, że jedzie w tę samą stronę!!!)
        auto const k { Obstacle.vehicle->MoverParameters->Vel };

        if( k - vel < 5 ) {
            // porównanie modułów prędkości [km/h]
            // zatroszczyć się trzeba, jeśli tamten nie jedzie znacząco szybciej
            if( Obstacle.distance < ActualProximityDist ) {
                // the vehicle ahead, not the stop found by the scan, is what the train is now to stop short of
                bStopTargetIsPassengerStop = false;
            }
            ActualProximityDist = std::min<double>(
                ActualProximityDist,
                Obstacle.distance );

            if( Obstacle.distance <= (
                mvOccupied->CategoryFlag & 2 ?
                    100.0 : // cars
                    250.0 ) ) { // others
                // regardless of driving mode at close distance take precaution measures:
                // match the other vehicle's speed or slow down if the other vehicle is stopped
                VelDesired =
                    min_speed(
                        VelDesired,
                        std::max(
                            k,
                            mvOccupied->CategoryFlag & 2 ?
                                40.0 : // cars
                                20.0 ) ); // others
                if( vel > VelDesired + fVelPlus ) {
                    // if going too fast force some prompt braking
                    AccPreferred = std::min(
                        mvOccupied->CategoryFlag & 2 ? -0.65 : // cars
					                                                       -0.30, // others
                        AccPreferred );
                }
            }

            double const distance = Obstacle.distance - fMaxProximityDist - fBrakeDist * 1.15; // odległość bezpieczna zależy od prędkości
            if( distance < 0.0 ) {
                // jeśli odległość jest zbyt mała
                if( k < 10.0 ) // k - prędkość tego z przodu
                { // jeśli tamten porusza się z niewielką prędkością albo stoi
                    // keep speed difference within a safer margin
                    VelDesired = std::floor(
                        min_speed(
                            VelDesired,
                            Obstacle.distance > 100 ? k + 20.0 : std::min(8.0, k + 4.0) ) );

                    if( OrderCurrentGet().HasAny( DriverOrder::connect | DriverOrder::looseShunt ) ) {
                        // jeśli spinanie, to jechać dalej
                        AccPreferred = std::min( 0.35, AccPreferred ); // nie hamuj
                        VelNext = std::floor( std::min( 8.0, k + 2.0 ) ); // i pakuj się na tamtego
                    }
                    else {
                        // a normalnie to hamować
                        VelNext = 0.0;
                        if( Obstacle.distance <= fMinProximityDist ) {
                            VelDesired = 0.0;
                        }

                        if( mvOccupied->CategoryFlag & 1
                            && OrderCurrentGet() & DriverOrder::obeyTrain ) {
                            // trains which move normally should try to stop at safe margin
                            ActualProximityDist -= fDriverDist;
                        }
                    }
                }
                else {
                    // jeśli oba jadą, to przyhamuj lekko i ogranicz prędkość
                    if( Obstacle.distance < (
                            mvOccupied->CategoryFlag & 2 ?
                                fMaxProximityDist + 0.5 * vel : // cars
                                2.0 * fMaxProximityDist + 2.0 * vel ) ) { //others
                        // jak tamten jedzie wolniej a jest w drodze hamowania
                        AccPreferred = std::min( -0.9, AccPreferred );
                        VelNext = min_speed( std::round( k ) - 5.0, VelDesired );
                        if( Obstacle.distance <= (
                            mvOccupied->CategoryFlag & 2 ?
                                fMaxProximityDist : // cars
                                2.0 * fMaxProximityDist ) ) { //others
                            // try to force speed change if obstacle is really close
                            VelDesired = VelNext;
                        }
                    }
                }
                ReactionTime = mvOccupied->Vel > EU07_AI_NOMOVEMENT ? 0.1 : // orientuj się, bo jest goraco
				                                                      2.0; // we're already standing still, so take it easy
            }
            else {
                if( OrderCurrentGet() & DriverOrder::connect ) {
                    // if there's something nearby in the connect mode don't speed up too much
                    VelDesired =
                        min_speed(
                            VelDesired,
                            Obstacle.distance > 100 ? 20.0 : 4.0 );
                }
            }
        }
    }
}

void
TController::adjust_desired_speed_for_limits() {
    // speed caps checks
    // sprawdzamy możliwe ograniczenia prędkości
    if( VelSignal >= 0 ) {
        // jeśli skład był zatrzymany na początku i teraz już może jechać
        VelDesired =
            min_speed(
                VelDesired,
                VelSignal );
    }
    if( mvOccupied->RunningTrack.Velmax >= 0 ) {
        // ograniczenie prędkości z trajektorii ruchu
        VelDesired =
            min_speed(
                VelDesired,
                mvOccupied->RunningTrack.Velmax ); // uwaga na ograniczenia szlakowej!
    }
    if( VelforDriver >= 0 ) {
        // tu jest zero przy zmianie kierunku jazdy
        // Ra: tu może być 40, jeśli mechanik nie ma znajomości szlaaku, albo kierowca jeździ 70
        VelDesired =
            min_speed(
                VelDesired,
                VelforDriver );
    }

    // recalculate potential load exchange duration
    check_load_exchange();
    if( ExchangeTime > 0
     || mvOccupied->Vel > 2.0 ) { // HACK: force timer reset if the load exchange is cancelled due to departure
        WaitingSet( ExchangeTime );
    }
    if( fStopTime < 0 ) {
        VelDesired = 0.0; // jak ma czekać, to nie ma jazdy
        cue_action( driver_hint::waitloadexchange );
        return; // speed limit can't get any lower
    }

    if( OrderCurrentGet().HasAny(DriverOrder::shunt | DriverOrder::looseShunt | DriverOrder::obeyTrain | DriverOrder::bank) && drivingFlags.stopHere && mvOccupied->Vel < 0.01 && VelSignalNext == 0.0 ) {
        // w Connect nie, bo moveStopHere odnosi się do stanu po połączeniu
		// jeśli ma czekać na wolną drogę, stoi a wyjazdu nie ma, to ma stać
		VelDesired = 0.0;
		return; // speed limit can't get any lower
	}

    if( OrderCurrentGet() == DriverOrder::waitForOrders ) {
        // wait means sit and wait
        VelDesired = 0.0;
        return; // speed limit can't get any lower
    }
}

// how long the present acceleration of a consist lasts whatever is done with the handle now: half the time
// its pneumatic brake takes to apply (while speeding up) or to release (while slowing down), which in G
// is tens of seconds; never shorter than the look-ahead the AI has always used
double
brake_response_horizon( TMoverParameters const *Vehicle, TBrakeSystem const Brakesystem, double const Acceleration, double const Brakea0 ) {

    auto horizon { ( 1.0 - Brakea0 ) * 30.0 / 3.6 };
    if( Brakesystem == TBrakeSystem::Pneumatic ) {
        // BrakeDelay holds the release and the application time in G, then the same two in P
        auto const delays { Vehicle->BrakeDelayFlag > bdelay_G ? 0 : 2 };
        auto const isrelease { Acceleration < 0.0 };
        horizon = std::max( horizon, 0.5 * Vehicle->BrakeDelay[ delays + ( isrelease ? 0 : 1 ) ] );
    }
    return horizon;
}

// distance [m] in which the gradient alone, with no brake and no power, brings a train rolling uphill at Velocity [km/h] to a stand
double
uphill_stopping_distance( double const Velocity, double const Gravity ) {
    return ( Velocity / 3.6 ) * ( Velocity / 3.6 ) / ( 2.0 * std::max( 0.01, -Gravity ) );
}

// speed [km/h] from which the gradient alone brings a train rolling uphill to a stand in Distance [m]
static double
uphill_coasting_speed( double const Distance, double const Gravity ) {
    return 3.6 * std::sqrt( 2.0 * std::max( 0.01, -Gravity ) * std::max( 0.0, Distance ) );
}

// outside of max safe range: follow the braking curve down to the speed ahead, Vel is the present speed in the
// direction of travel
void
TController::adjust_desired_speed_for_braking_curve( double const Vel ) {

    AccDesired = AccPreferred;
    if( Vel <= min_speed( ActualProximityDist > 10.0 ? 10.0 : 5.0, VelDesired ) ) {
        return;
    }
    // allow to coast at reasonably low speed
    auto const brakingdistance { fBrakeDist * braking_distance_multiplier( VelNext ) };
    if( auto const slowdowndistance { (
            mvOccupied->CategoryFlag == 2 ? // cars can stop on a dime, for bigger vehicles we enforce some minimal braking distance
                brakingdistance :
                std::max(
                    false == OrderCurrentGet().HasAny(DriverOrder::connect) ? 100.0 : 25.0,
                    brakingdistance ) ) };
        brakingdistance + std::max(slowdowndistance, fMaxProximityDist) < ActualProximityDist - fMaxProximityDist ) {
        // don't slow down prematurely; as long as we have room to come to a full stop at a safe distance, we're good
        return;
    }
    // ensure some minimal coasting speed, otherwise a vehicle entering this zone at very low speed will be crawling forever
    auto const brakingpointoffset = (
        ( VelNext == 0.0 ) && is_train() && ( OrderCurrentGet() & DriverOrder::obeyTrain ) ?
            stopping_margin() : // the same stopping point as the final approach
            VelNext * braking_distance_multiplier( static_cast<float>( VelNext ) ) );
    // both plans of a pneumatic brake are made from the brake table of the consist; with none (in shunting it isn't
    // worked out) the braking curve is the constant deceleration over the distance left, as before them
    braking_curve::brake_table const braketable { fBrake_a0, fBrake_a1, BrakeAccTableSize, mvOccupied->Vmax };
    // making for a passenger stop only the time the brake takes to fill: the look-ahead the horizon
    // keeps as its floor was set for the stops at a signal, and a vehicle that brakes with its
    // independent brake, which leaves the train brake handle where it is, kept braking earlier
    // and earlier the whole way in, to crawl the last tens of metres to the stop point
    auto const fillbrakea0 { bStopTargetIsPassengerStop ? 1.0 : fBrake_a0[ 0 ] };
    // once the brake is on, the room it takes to fill is behind; a vehicle running alone brakes with its
    // independent brake and leaves the train brake handle in running, so that brake counts as well
    braking_curve::approach const approach {
        Vel, VelNext, ActualProximityDist, brakingpointoffset, fAccGravity, AbsAccS,
        braking_curve::brake_acting(
            BrakeCtrlPosition,
            // IncLocalBrakeLevel() moves the independent brake handle on a vehicle without one too
            ( mvOccupied->LocalBrake == TLocalBrake::PneumaticBrake || mvOccupied->LocalBrake == TLocalBrake::HydraulicBrake ) ?
                mvOccupied->LocalBrakePosA :
                0.0 ),
        brake_response_horizon( mvOccupied, BrakeSystem, AbsAccS, fBrake_a0[ 0 ] ),
        brake_response_horizon( mvOccupied, BrakeSystem, 1.0, fillbrakea0 ) };
    auto const deceleration { braking_curve::demand( approach, BrakeSystem == TBrakeSystem::Pneumatic ? &braketable : nullptr ) };
    AccDesired = std::min( AccDesired, deceleration ); // najpierw hamuje mocniej, potem zluzuje
    if( Vel <= VelNext ) {
        return;
    }
    if( deceleration < fAccGravity - 0.001 ) {
        // no power while still above the speed being braked for
        AccDesired = std::min( AccDesired, EU07_AI_NOACCELERATION );
    }
    else if( ( fAccGravity < -0.025 )
          && ( OrderCurrentGet().HasAny(DriverOrder::obeyTrain) )
          && ( false == uphill_brake_committed( Vel ) )
          && ( ActualProximityDist - uphill_stopping_distance( Vel, fAccGravity ) > brakingpointoffset ) ) {
        // uphill the gradient alone slows the train down more than it is asked to;
        // keep it rolling at the present speed until that changes
        AccDesired = std::min( 0.0, AccPreferred );
        VelDesired = min_speed( VelDesired, uphill_coasting_speed( ActualProximityDist - brakingpointoffset, fAccGravity ) );
    }
}

// the final approach to a stop, inside the proximity range: Vel is the present speed in the direction of travel
void
TController::adjust_desired_speed_for_final_approach( double const Vel ) {

    if( ( mvOccupied->CategoryFlag & 1 ) == 0 ) {
        // for cars (and others) coast at low speed until we hit min proximity range
        VelDesired = min_speed( VelDesired, 5.0 );
        return;
    }
    // trains
    if( ( OrderCurrentGet() & ( DriverOrder::shunt | DriverOrder::connect ) )
     && ( Obstacle.distance < 50 ) ) {
        // crude detection of edge case, if approaching another vehicle coast slowly until min distance
        // this should allow to bunch up trainsets more on sidings
        VelDesired = min_speed( 5.0, VelDesired );
        return;
    }
    // hamowanie tak, aby stanąć
    if( false == OrderCurrentGet().HasAny(DriverOrder::obeyTrain) ) {
        VelDesired = VelNext;
        AccDesired = ( VelNext * VelNext - Vel * Vel ) / ( 25.92 * ( ActualProximityDist + 0.1 - 0.5*fMinProximityDist ) );
        AccDesired = std::min( AccDesired, fAccThreshold );
        return;
    }
    // a train keeps its desired speed until the lower edge of the stopping window (a desired
    // speed of 0 here asks for hard braking whatever the distance left), aims at the middle
    // of the window, and asks only for what that takes
    auto const stoppingroom { std::max( 0.5, ActualProximityDist - stopping_margin() ) };
    AccDesired = (
        BrakeSystem == TBrakeSystem::Pneumatic ?
            // cast iron blocks grip harder as the train comes to a stand
            braking_curve::shaped_demand(
                Vel, VelNext, stoppingroom, fAccGravity,
                { fBrake_a0, fBrake_a1, BrakeAccTableSize, mvOccupied->Vmax } ) :
            ( VelNext * VelNext - Vel * Vel ) / ( 25.92 * stoppingroom ) );
    if( ( fAccGravity < -0.025 )
     && ( AccDesired >= fAccGravity - 0.001 ) // no brake needed at all
     && ( false == uphill_brake_committed( Vel ) )
     && ( ActualProximityDist - uphill_stopping_distance( Vel, fAccGravity ) > stopping_margin() ) ) {
        // uphill the gradient alone would bring the train to a stand short of the
        // middle of the stopping window; keep it rolling at the present speed until that changes,
        // but no faster than the gradient alone can take off by the middle of the window
        AccDesired = std::min( 0.0, AccPreferred );
        VelDesired = min_speed( VelDesired, uphill_coasting_speed( ActualProximityDist - stopping_margin(), fAccGravity ) );
    }
}

void
TController::adjust_desired_speed_for_target_speed( double const Range ) {
    // ustalanie zadanego przyspieszenia
    //(ActualProximityDist) - odległość do miejsca zmniejszenia prędkości
    //(AccPreferred) - wynika z psychyki oraz uwzglęnia już ewentualne zderzenie z pojazdem z przodu, ujemne gdy należy hamować
    //(AccDesired) - uwzględnia sygnały na drodze ruchu, ujemne gdy należy hamować
    //(fAccGravity) - chwilowe przspieszenie grawitacyjne, ujemne działa przeciwnie do zadanego kierunku jazdy
    //(AbsAccS) - chwilowe przyspieszenie pojazu (uwzględnia grawitację), ujemne działa przeciwnie do zadanego kierunku jazdy
    //(AccDesired) porównujemy z (fAccGravity) albo (AbsAccS)

    // gdy jedzie wolniej niż potrzeba, albo nie ma przeszkód na drodze
    // normalna jazda
    AccDesired = VelDesired != 0.0 ? AccPreferred : -0.01;
    fAccDesiredGravity = 0.0;

    auto const vel { DirectionalVel() };

    if( VelNext >= 0.0
     && ActualProximityDist <= Range
     && vel >= VelNext ) {
        // gdy zbliża się i jest za szybki do nowej prędkości, albo stoi na zatrzymaniu
        if (vel > EU07_AI_NOMOVEMENT) {
            // jeśli jedzie
            if( vel < VelNext
             && ActualProximityDist > fMaxProximityDist * (1.0 + 0.1 * vel) ) {
                // jeśli jedzie wolniej niż można i jest wystarczająco daleko, to można przyspieszyć
                if( AccPreferred > 0.0 ) {
                    // jeśli nie ma zawalidrogi dojedz do semafora/przeszkody
                    AccDesired = AccPreferred;
                }
            }
            else if( ( ActualProximityDist > fMinProximityDist )
                  // only a stop has a final approach of its own inside the proximity range; a lower speed ahead
                  // keeps the braking curve below right up to where it begins, or the train would stop braking
                  // (and even take power) that far out
                  && ( VelNext == 0.0 )
                  && ( ActualProximityDist < fMaxProximityDist ) ) {
                // jedzie szybciej, niż trzeba na końcu ActualProximityDist, ale jeszcze jest daleko
                // jak minął już maksymalny dystans po prostu hamuj (niski stopień)
                // ma stanąć, a jest w drodze hamowania albo ma jechać
                adjust_desired_speed_for_final_approach( vel );
                AccDesired = std::min( AccDesired, AccPreferred );
            }
            else if (ActualProximityDist > fMinProximityDist) {
                // jedzie szybciej, niż trzeba na końcu ActualProximityDist, ale jeszcze jest daleko
                adjust_desired_speed_for_braking_curve( vel );
                AccDesired = std::min( AccDesired, AccPreferred );
            }
            else {
                // jest bliżej niż fMinProximityDist
                // utrzymuj predkosc bo juz blisko
/*
                VelDesired = min_speed( VelDesired, VelNext );
*/
                if( VelNext == 0.0 ) {
                    VelDesired = VelNext;
                }
                else {
                    if( vel <= VelNext + fVelPlus ) {
                        // jeśli niewielkie przekroczenie, ale ma jechać
                        AccDesired = std::max( 0.0, AccPreferred ); // to olej (zacznij luzować)
                    }
                }
                ReactionTime = 0.1; // i orientuj się szybciej
            }
        }
        else {
            // zatrzymany (vel==0.0)
            if( VelNext > 0.0 ) {
                // można jechać
                AccDesired = AccPreferred;
            }
            else {
                // jeśli daleko jechać nie można
                if( ActualProximityDist > (
                        mvOccupied->CategoryFlag & 2 ?
                            fMinProximityDist : // cars
                            fMaxProximityDist ) ) { // trains and others
                    // ale ma kawałek do sygnalizatora
                    if( AccPreferred > 0.0 ) {
                        // dociagnij do semafora;
                        AccDesired = AccPreferred;
                    }
                    else {
                        //stoj
                        VelDesired = 0.0;
                    }
                }
                else {
                    // VelNext=0 i stoi bliżej niż fMaxProximityDist
                    VelDesired = 0.0;
                }
            }
        }
    }
}

void
TController::adjust_desired_speed_for_current_speed() {
    // decisions based on current speed
    auto const vel { DirectionalVel() };

    if( is_train() ) {
        // on flats or uphill we can be less careful
        if( vel > VelDesired ) {
            // jesli jedzie za szybko do AKTUALNEGO
            if( VelDesired == 0.0 ) {
                // jesli stoj, to hamuj, ale i tak juz za pozno :)
                AccDesired = std::min( AccDesired, -0.85 ); // hamuj solidnie
            }
            else {
                // slow down, not full stop
                if( vel > VelDesired + fVelPlus ) {
                    // hamuj tak średnio
                    AccDesired = std::min( AccDesired, -0.05 + fAccThreshold );
                }
                else {
                    // o 5 km/h to olej (zacznij luzować)
                    AccDesired = std::min(
                        AccDesired, // but don't override decceleration for VelNext
                        std::max( 0.0, AccPreferred ) );
                }
            }
        }

        if( fAccGravity > 0.025 ) {
            // going sharply downhill we may need to start braking sooner than usual
            // try to estimate increase of current velocity before engaged brakes start working
            // look ahead for as long as the brake takes to respond, since that is how long the
            // present acceleration will last whatever is done with the handle now; a pneumatic
            // brake applies and releases on its own time scale, which in G is tens of seconds
            auto const horizon { brake_response_horizon( mvOccupied, BrakeSystem, AbsAccS, fBrake_a0[ 0 ] ) };
            auto const speedestimate = vel + 3.6 * horizon * AbsAccS;
            // already below a lower limit ahead, the train has no business speeding up towards it
            auto holdlimit { VelDesired };
            if( ( VelNext > 0.0 )
             && ( VelNext < holdlimit )
             && ( vel <= VelNext + fVelPlus )
             && ( ActualProximityDist < std::max( 2.0 * fBrakeDist, 500.0 ) ) ) {
                holdlimit = VelNext;
            }
            // hold a band just below the limit; the steeper the descent the harder the train pulls
            // away once the brake eases, so the band widens with the gradient (about 3 km/h at 20 per mille)
            // an electro-pneumatic or electric brake responds at once and needs far less room
            auto const holdspeed { holdlimit - (
                BrakeSystem == TBrakeSystem::Pneumatic ?
                    std::clamp( fAccGravity * 15.0, 1.0, 3.0 ) :
                    std::clamp( fAccGravity * 7.5, 1.0, 1.5 ) ) };
            // the predicted difference is to be made up over the time the brake takes to respond, not at once: with a
            // slow brake, a demand sized to remove it at once comes on as a deep application, which then takes as
            // long to let go, and the train ends up well below the speed it was to hold
            auto const holdgain { std::min( 0.04, 1.0 / ( 3.6 * std::max( 1.0, horizon ) ) ) };
            if( ( speedestimate > VelDesired )
             && ( VelDesired == 0.0 ) ) {
                // jesli stoj, to hamuj, ale i tak juz za pozno :)
                AccDesired = std::min( AccDesired, -0.85 ); // hamuj solidnie
            }
            else {
                // one continuous law on both sides of the target, proportional to the predicted
                // difference: a demand that jumps to the braking threshold above the target and to
                // full acceleration below it alternates, with a brake that responds in seconds,
                // between a hard application and a full release
                AccDesired = std::min(
                    AccDesired, // but don't override decceleration for VelNext
                    std::clamp(
                        EU07_AI_NOACCELERATION - holdgain * ( speedestimate - holdspeed ),
                        -0.05 + fAccThreshold,
                        std::max( EU07_AI_NOACCELERATION, AccPreferred ) ) );
                // HACK: for cargo trains with high braking threshold ensure we cross that threshold
                auto const crossthreshold { IsCargoTrain && fBrake_a0[ 0 ] > 0.2 && speedestimate > holdlimit };
                AccDesired -= ( crossthreshold ? std::clamp( fBrake_a0[ 0 ] - 0.2, 0.0, 0.15 ) : 0.0 );
            }
            // final tweaks
            if( vel > EU07_AI_NOMOVEMENT ) {
                // going downhill also take into account impact of gravity
                AccDesired -= fAccGravity;
                // from here on AccDesired is what the brake alone has to deliver, which is what
                // IncBrake() sizes the handle position for; AbsAccS already contains the gravity,
                // so the comparisons with it have to add this part back
                fAccDesiredGravity = fAccGravity;
                // HACK: if the max allowed speed was exceeded something went wrong; brake harder
                AccDesired -= 0.15 * std::clamp( vel - VelDesired, 0.0, 5.0 );
            }
        }
        // HACK: limit acceleration for cargo trains, to reduce probability of breaking couplers on sudden jolts
		auto MaxAcc{ 0.5 * mvOccupied->Couplers[(mvOccupied->DirAbsolute >= 0 ? end::rear : end::front)].FmaxC / fMass };
        if( iVehicles - ControlledEnginesCount > 0 ) {
            MaxAcc *= std::clamp( vel * 0.025, 0.2, 1.0 );
        }
		AccDesired = std::min(AccDesired, safe_clamp(MaxAcc, HeavyCargoTrainAcceleration, AccPreferred));
        // TBD: expand this behaviour to all trains with car(s) exceeding certain weight?
		/*
        if( ( IsPassengerTrain ) && ( iVehicles - ControlledEnginesCount > 0 ) ) {
            AccDesired = std::min( AccDesired, ( iVehicles - ControlledEnginesCount > 8 ? HeavyPassengetTrainAcceleration : PassengetTrainAcceleration ) );
        }
        if( ( IsCargoTrain ) && ( iVehicles - ControlledEnginesCount > 0 ) ) {
            AccDesired = std::min( AccDesired, CargoTrainAcceleration );
        }
        if( ( IsHeavyCargoTrain ) && ( iVehicles - ControlledEnginesCount > 0 ) ) {
            AccDesired = std::min( AccDesired, HeavyCargoTrainAcceleration );
        } */
    }
    else {
        // for cars the older version works better
        if( vel > VelDesired ) {
            // jesli jedzie za szybko do AKTUALNEGO
            if( VelDesired == 0.0 ) {
                // jesli stoj, to hamuj, ale i tak juz za pozno :)
                AccDesired = std::min( AccDesired, -0.9 ); // hamuj solidnie
            }
            else {
                // slow down, not full stop
                if( vel > VelDesired + fVelPlus ) {
                    // hamuj tak średnio
                    AccDesired = std::min( AccDesired, -fBrake_a0[ 0 ] * 0.5 );
                }
                else {
                    // o 5 km/h to olej (zacznij luzować)
                    AccDesired = std::min(
                        AccDesired, // but don't override decceleration for VelNext
                        std::max( 0.0, AccPreferred ) );
                }
            }
        }
    }

    // NOTE: as a stop-gap measure the routine is limited to trains only while car calculations seem off
    if( is_train() && vel < VelDesired &&
	    -AccDesired * BrakeAccFactor() < (false == Ready || VelNext > vel - 40.0 ? fBrake_a0[0] * 0.8 : -fAccThreshold) / (1.2 * braking_distance_multiplier(VelNext)) ) {
		// don't adjust acceleration when going above current target speed
		AccDesired = std::max(EU07_AI_NOACCELERATION, AccDesired);
	}
}

// hamowanie kontrolne
void
TController::adjust_desired_speed_for_braking_test() {

//    if( false == Global.DynamicBrakeTest ) { return; }
    if( DynamicBrakeTest >= 5 ) { return; } // all done
    if( ( DynamicBrakeTest == 0 )
     && ( false == timetable::makes_brake_test( TrainParams.run_flags() ) ) ) { return; } // the timetable says there's none (eu07/maszyna#95)
    if( false == is_train() ) { return; } // not applicable
    if( mvOccupied->BrakeHandle == TBrakeHandle::FVel6 ) { return; } // leads to bugs in DecBrake, disable for now
    if( false == OrderCurrentGet().HasAll(DriverOrder::obeyTrain) ) { return; } // not applicable
    if( false == AIControllFlag ) { return; } // TBD: add notification about braking test and enable it for human driver as well?

    auto const vel { DirectionalVel() };
    // what the route ahead asks for; the test may brake harder than that, never softer
    auto const routeaccdesired { AccDesired };
    auto const routeaccdesiredgravity { fAccDesiredGravity };
    auto const routeveldesired { VelDesired };

    switch( DynamicBrakeTest ) {
        case 0: {
            if( primary()
             && vel < VelDesired
             && fAccGravity >= 0.0 // not if going uphill
             && AccDesired >= EU07_AI_ACCELERATION
             && TrainParams.TTVmax >= 10.0
             && vel > std::min(TrainParams.TTVmax, 60.0) - 2.0 ) {
                DynamicBrakeTest = 1;
                DBT_BrakingTime = ElapsedTime;
                DBT_VelocityBrake = vel;
            }
            break;
        }
        case 1: {
            AccDesired = EU07_AI_BRAKINGTESTACCELERATION;
            fAccDesiredGravity = 0.0;
            VelDesired = DBT_VelocityBrake;
            if( ElapsedTime - DBT_BrakingTime > 1 ) {
                ForcePNBrake = true;
                mvOccupied->EpFuseSwitch( false );
                DynamicBrakeTest = 2;
            }
            break;
        }
        case 2: {
            AccDesired = EU07_AI_BRAKINGTESTACCELERATION;
            fAccDesiredGravity = 0.0;
            VelDesired = DBT_VelocityBrake;
            if( ElapsedTime - DBT_BrakingTime > 2 ) {
                DBT_BrakingTime = ElapsedTime;
                DBT_VelocityBrake = vel;
                DBT_VelocityRelease = vel - 8.0;
                DynamicBrakeTest = 3;
            }
            break;
        }
        case 3: {
            AccDesired = safe_clamp( -AbsAccS, fAccThreshold * 1.01, fAccThreshold * 1.21 );
            fAccDesiredGravity = 0.0;
            VelDesired = DBT_VelocityBrake;
            if( vel <= DBT_VelocityRelease ) {
                DynamicBrakeTest = 4;
                DBT_BrakingTime = ElapsedTime - DBT_BrakingTime;
                DBT_MidPointAcc = AbsAccS;
                DBT_ReleasingTime = ElapsedTime;
            }
            break;
        }
        case 4: {
            if( Ready ) {
                if( BrakeSystem == TBrakeSystem::ElectroPneumatic ) {
                    mvOccupied->EpFuseSwitch( true );
                }
                ForcePNBrake = false;
                DynamicBrakeTest = 5;
                DBT_ReleasingTime = ElapsedTime - DBT_ReleasingTime;
                DBT_VelocityFinish = vel;
            }
            break;
        }
        default: {
            break;
        }
    }
    // a stop or a restriction coming up while the test runs takes precedence
    if( ( DynamicBrakeTest >= 1 )
     && ( DynamicBrakeTest <= 3 )
     && ( routeaccdesired < AccDesired ) ) {
        AccDesired = routeaccdesired;
        fAccDesiredGravity = routeaccdesiredgravity;
        VelDesired = std::min( VelDesired, routeveldesired );
    }
}
