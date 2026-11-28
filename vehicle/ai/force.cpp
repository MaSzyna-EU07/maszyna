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

// the traction and the brakes the driver applies for the speed it aims for

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

const double FSOverchargePipePress = 5.2;
const double FSOverchargeTime = 2.0;
const double FSOverchargeCRP = 0.05;
const double FSMaxTime = 10.0;
const double DepartBlockedFSDelay = 10.0;

void
TController::control_tractive_and_braking_force() {

#if LOGVELOCITY
    WriteLog("Dist=" + FloatToStrF(ActualProximityDist, ffFixed, 7, 1) +
                ", VelDesired=" + FloatToStrF(VelDesired, ffFixed, 7, 1) +
                ", AccDesired=" + FloatToStrF(AccDesired, ffFixed, 7, 3) +
                ", VelSignal=" + AnsiString(VelSignal) + ", VelNext=" +
                AnsiString(VelNext));
#endif

    if (iDriverFailCount > maxdriverfails) {
        Psyche = Easyman;
        if (iDriverFailCount > maxdriverfails * 2)
            SetDriverPsyche();
    }

    control_relays();
    control_motor_connectors();

    // if the radio-stop was issued don't waste effort trying to fight it
    if( true == mvOccupied->RadioStopFlag // radio-stop
     && mvOccupied->Vel > EU07_AI_NOMOVEMENT ) { // and still moving
        cue_action( driver_hint::mastercontrollersetzerospeed ); // just throttle down...
        return; // ...and don't touch any other controls
    }
    // if we're slipping leave the controls alone, there's a separate logic section trying to deal with it
    if( true == mvControlling->SlippingWheels ) {
        return;
    }

    // ensure train brake isn't locked
    if( BrakeCtrlPosition == gbh_NP ) {
        cue_action( driver_hint::trainbrakerelease );
    }
    control_releaser();
    control_main_pipe();

    control_tractive_force();
    control_braking_force();

#if LOGVELOCITY
    WriteLog("BrakePos=" + AnsiString(mvOccupied->BrakeCtrlPos) + ", MainCtrl=" +
                AnsiString(mvControlling->MainCtrlPos));
#endif
}

// the switch of the field shunting of a diesel-electric that has one (AutoRelay=Optional with FieldShuntSwitch=Yes): off,
// the field stays full. checked while running too, since a vehicle taken over with its engine running doesn't go through
// PrepareEngine(). a diesel-electric with AutoRelay=Optional alone is left as before, shunting by hand with the switch off
void TController::control_field_shunting() {

    if( AIControllFlag
     && mvControlling->EngineType == TEngineType::DieselElectric
     && fieldshunting::has_switch( mvControlling->AutoRelayType, mvControlling->FieldShuntSwitch )
     && false == mvControlling->AutoRelayFlag ) {
        mvControlling->AutoRelaySwitch( true );
    }
}

// ensure relays are active
void TController::control_relays() {

    control_field_shunting();

    if( fActionTime < 0.0 ) { return; }

    if( IsAnyMotorOverloadRelayOpen ) {
        cue_action( driver_hint::mastercontrollersetzerospeed );
        cue_action( driver_hint::tractionnmotoroverloadreset );
        ++iDriverFailCount;
    }
    if( IsAnyGroundRelayOpen ) {
        cue_action( driver_hint::mastercontrollersetzerospeed );
        cue_action( driver_hint::maincircuitgroundreset );
        ++iDriverFailCount;
    }
}

void TController::control_motor_connectors() {
    // ensure motor connectors are active
    if( (mvControlling->EngineType == TEngineType::ElectricSeriesMotor || mvControlling->EngineType == TEngineType::DieselElectric || is_emu()) && Need_TryAgain ) {
		// true, jeśli druga pozycja w elektryku nie załapała
		cue_action(driver_hint::mastercontrollersetzerospeed);
		Need_TryAgain = false;
	}
}

void TController::control_tractive_force() {

    control_uphill_start();

    auto const velocity { DirectionalVel() };
    // jeśli przyspieszenie pojazdu jest mniejsze niż żądane oraz...
    if( AccDesired > EU07_AI_NOACCELERATION // don't add power if not asked for actual speed-up
	    && (AbsAccS < AccDesired /*- std::min( 0.05, 0.01 * iDriverFailCount )*/ || mvOccupied->SpeedCtrlUnit.IsActive && velocity < mvOccupied->SpeedCtrlUnit.FullPowerVelocity) &&
	    false == drivingFlags.press &&
	    // starting uphill against the releasing brake, the power that holds the train is enough: the brake coming off
	    // speeds the train up, more power only drags the brake and slips the wheels. nor is a notch past that taken
	    // back: the power got there in one notch from below, and the notch below doesn't hold the train
	    ( false == bUphillRelease || fUphillTractiveForce < uphill_start::holding_force * fUphillHoldingForce ) &&
	    // nor while the train brake is still being applied further to hold the train
	    false == bUphillBrakeTopUp &&
	    velocity < (VelDesired == 1.0 ? // work around for trains getting stuck on tracks with speed limit = 1
	                    VelDesired :
	                    VelDesired - fVelMinus) &&
	    (ActualProximityDist > (is_car() ? fMinProximityDist : // cars are allowed to move within min proximity distance
	                                       fMaxProximityDist) ? // other vehicle types keep wider margin
	         true :
	         velocity + 1.0 < VelNext
	             // uphill, a train told to hold its speed on the way to a stop needs the power to do it
	             || VelNext == 0.0 && OrderCurrentGet().HasAny(DriverOrder::obeyTrain) && AccDesired >= 0.0 && fAccGravity < -0.025 && ActualProximityDist > fMinProximityDist) ) {
        // ...jeśli prędkość w kierunku czoła jest mniejsza od dozwolonej o margines...
		// within proximity margin don't exceed next scheduled speed, outside of the margin accelerate as you like
		// ...to można przyspieszyć
		increase_tractive_force();
	}
    // zmniejszanie predkosci
    // margines dla prędkości jest doliczany tylko jeśli oczekiwana prędkość jest większa od 5km/h
    if( false == drivingFlags.press ) {
		double SpeedCtrlMargin = mvControlling->SpeedCtrlUnit.IsActive && VelDesired > 5 ? 3 : 0;
        // jeśli nie dociskanie
        if( AccDesired <= EU07_AI_NOACCELERATION ) {
            cue_action( driver_hint::mastercontrollersetzerospeed );
        }
        else if( velocity > VelDesired + SpeedCtrlMargin
              || ( fAccGravity < -0.01 ? is_too_fast_uphill() : AbsAccS > AccDesired + 10.05 )
               || IsAnyCouplerStretched ) {
                // jak za bardzo przyspiesza albo prędkość przekroczona
				// dodany wyjatek na "pelna w przod"
                cue_action( driver_hint::tractiveforcedecrease );
        }
    }

    control_handles();
    SpeedSet(); // ciągla regulacja prędkości
}

bool TController::is_too_fast_uphill() const {

    if( AccDesired < 0.0 ) { return true; }
    // holding the speed uphill on the way to a stop asks for no speeding up
    return ( VelNext == 0.0 && ActualProximityDist <= fMaxProximityDist && AbsAccS > AccDesired + 0.05 );
}

// starting uphill a driver of a heavy train keeps the train brake applied, lets the locomotive's own brake off (the
// independent brake, and its share of the train brake with the releaser), applies the power, and releases the train
// brake only once the power holds the train. releasing everything first lets the train roll back before the power is
// there, and holding the train on the locomotive brake is no way out where that brake cuts the power (asynchronous
// drives, the control pressure switch)
void TController::control_uphill_start() {

    auto const velocity { DirectionalVel() };
    if( auto const uphill {
            ( BrakeSystem == TBrakeSystem::Pneumatic )
         && is_train()
         && ( false == is_emu() ) && ( false == is_dmu() ) // a multiple unit has no locomotive to hold a train behind it
         && ( fAccGravity < EU07_AI_UPHILLGRAVITY ) };
        false == uphill ) {
        end_uphill_start();
        return;
    }
    if( false == bUphillStart ) {
        if( ( std::abs( velocity ) < EU07_AI_NOMOVEMENT )
         // the whole train at a stand, not only its head stopped for a moment while the train rolls back
         && ( fConsistVelocity > -0.5 )
         && ( AccDesired > EU07_AI_NOACCELERATION )
         && ( VelDesired > 0.0 )
         && ( BrakeCtrlPosition > 0.0 ) // only with the train brake still applied to hold the train
         && ( IsConsistCharged ) ) { // and charged to what the pipe has, or the brake that holds the train has no air
            bUphillStart = true;
            bUphillRelease = false;
            UphillTopUp = {};
            fUphillStartTime = ElapsedTime;
        }
        else {
            return;
        }
    }
    if( AccDesired <= EU07_AI_NOACCELERATION ) {
        // no longer to start: the standing rules take over
        end_uphill_start();
        return;
    }
    if( fConsistVelocity < -0.5 ) {
        // the train rolls back all the same: the rules for a train slipping back take over and apply the brakes.
        // the train as a whole, not its head: with the locomotive brake off, the locomotive and the first vehicles
        // run back into the slack of a long train still held by its brake
        end_uphill_start();
        return;
    }
    fUphillTractiveForce = 0.0;
    for( auto const *vehicle { pVehicles[ end::front ] }; vehicle != nullptr; vehicle = vehicle->Next() ) {
        // Ft is signed in the vehicle's own frame, and turned into the direction of travel as AbsAccS is
        fUphillTractiveForce += std::max( 0.0, vehicle->MoverParameters->Ft * ( vehicle->DirectionGet() == iDirection ? 1 : -1 ) * ( iDirection >= 0 ? 1 : -1 ) );
    }
    fUphillHoldingForce = fMass * -fAccGravity;
    bUphillBrakeTopUp = false;
    if( false == bUphillRelease ) {
        bUphillBrakeTopUp = uphill_brake_top_up();
        if( bUphillBrakeTopUp ) {
            return;
        }
        if( ( mvOccupied->LocalBrake != TLocalBrake::ManualBrake )
         && ( mvOccupied->LocalBrakePosA > 0.0 ) ) {
            cue_action( driver_hint::independentbrakerelease );
        }
        if( ( velocity > 1.0 )
         || ( fUphillTractiveForce > uphill_start::holding_force * fUphillHoldingForce )
         // a locomotive that can't hold the train on its own power gets the brake off in the end all the same
         || ( ElapsedTime - fUphillStartTime > 45.0 ) ) {
            // the power holds the train: the train brake can be released
            bUphillRelease = true;
            fUphillReleaseTime = ElapsedTime;
        }
        return;
    }
    // the train brake is being released with the power on. the start ends once the brakes are off: ended before,
    // the brake still on takes the power off, the train rolls back and has to be braked to a stand again
    if( IsConsistReleased
     // a brake that doesn't come off (a hand brake) ends it all the same
     || ( ElapsedTime - fUphillReleaseTime > EU07_AI_UPHILLRELEASETIME ) ) {
        end_uphill_start();
    }
}

// the locomotive brake comes off only once the brake of the rest of the train holds the train on its own: a train
// stopped on a light application stands on the locomotive brake too, and rolls back as soon as it is off. until then
// the train brake is applied further, a step at a time; true while it is
bool TController::uphill_brake_top_up() {

    auto trainbrakeforce { 0.0 };
    auto haswagons { false };
    for( auto const *vehicle { pVehicles[ end::front ] }; vehicle != nullptr; vehicle = vehicle->Next() ) {
        if( vehicle->MoverParameters->Power < 0.01 ) {
            trainbrakeforce += vehicle->MoverParameters->Fb;
            haswagons = true;
        }
    }
    if( false == haswagons ) {
        return false;
    }
    auto const action { uphill_start::brake_top_up(
        UphillTopUp, trainbrakeforce, fUphillHoldingForce, BrakeCtrlPosition, fBrakeHeldTime,
        mvOccupied->BrakeDelay[ mvOccupied->BrakeDelayFlag > bdelay_G ? 1 : 3 ] ) };
    if( action == uphill_start::top_up_action::none ) {
        return false;
    }
    // the time for the power to hold the train starts once the brake does
    fUphillStartTime = ElapsedTime;
    if( action == uphill_start::top_up_action::step ) {
        BrakeLevelAdd( BrakingLevelIncrease );
    }
    return true;
}

void TController::end_uphill_start() {

    bUphillStart = false;
    bUphillRelease = false;
    bUphillBrakeTopUp = false;
}

// standing, a train is prepared as a driver does it: the locomotive brake first, then the brake pipe is filled. the pipe
// filled at once releases the distributors while the auxiliary reservoirs are still empty: the train has no brake left,
// and on a gradient rolls away as it releases. where the locomotive brake alone doesn't hold the train, the train brake
// is applied lightly, so the reservoirs charge with the train held. the locomotive brake comes off only once the train
// is charged and to start (DecBrake())
void TController::secure_standing_train() {

    if( BrakeSystem != TBrakeSystem::Pneumatic ) {
        // the electro-pneumatic brake of a multiple unit is released as before: nothing there takes the locomotive
        // brake off again when the unit starts
        cue_action( driver_hint::trainbrakerelease );
        return;
    }
    // the first start of a train that hasn't run yet waits for the brakes to charge. one that has run, prepared again
    // (a change of direction, an engine started anew), has its reservoirs charged from the run and starts as before
    if( false == bHasRun ) {
        bBrakeChargeStart = true;
    }
    if( mvOccupied->LocalBrake != TLocalBrake::ManualBrake ) {
        cue_action( driver_hint::independentbrakeapply );
    }
    if( false == brake_charge::locomotive_brake_holds( locomotive_brake_force(), fMass * fAccGravity ) ) {
        // the first service position: the pipe and the reservoirs charge to a little below the working pressure, the
        // distributors keep a light application. a stronger application already there is left as it is, but not
        // the emergency one or an empty pipe's, which don't charge
        if( AIControllFlag
         && ( ( BrakeCtrlPosition < 1.0 ) || ( BrakeCtrlPosition > uphill_start::top_up_position ) ) ) {
            BrakeLevelSet( 1.0 );
        }
        bBrakeChargeHold = true;
        return;
    }
    bBrakeChargeHold = false;
    cue_action( driver_hint::trainbrakerelease );
}

// the brake force of the locomotive brake at full application [N]. of the occupied vehicle alone: its handle acts on
// no other, not on a second section or a locomotive in multiple control either, let alone one hauled dead
double TController::locomotive_brake_force() const {

    if( ( mvOccupied->LocalBrake == TLocalBrake::ManualBrake )
     || ( mvOccupied->MaxBrakePress[ 0 ] <= 0.0 ) ) {
        return 0.0;
    }
    return brake_charge::locomotive_brake_force(
        mvOccupied->BrakeForceP( mvOccupied->MaxBrakePress[ 0 ], 0.0 ),
        mvOccupied->TotalMassxg );
}

void TController::increase_tractive_force() {

    if( fActionTime < 0.0 )     { return; } // gdy jest nakaz poczekać z jazdą, to nie ruszać
    if( IsAnyCouplerStretched ) { return; } // train is already stretched past its limits, don't pull even harder

    if( mvOccupied->SpringBrake.Activate ) {
        cue_action( driver_hint::springbrakeoff );
    }
    if( VelDesired > 0.0 // to prevent door shuffle on stop
     && ( doors_open() || doors_permit_active() ) ) {
        // zamykanie drzwi - tutaj wykonuje tylko AI (zmienia fActionTime)
        Doors( false );
        // Doors() call can potentially adjust fActionTime
        if( fActionTime < 0.0 ) { return; }
    }
    if( true == mvOccupied->DepartureSignal ) {
        cue_action( driver_hint::departuresignaloff );
    }

    cue_action( driver_hint::tractiveforceincrease );
}

// how far the deceleration may fall short of the demand before the brake is applied harder. approaching a lower
// speed, a pneumatic brake given a full step only once that much is missing acts too late: the demand
// creeps up meanwhile, and in a long G train the next step takes many seconds to act, so half a step is enough
double
TController::brake_increase_margin() const {

    if( ( BrakeSystem == TBrakeSystem::Pneumatic )
     && ( VelNext > 0.0 )
     && ( VelNext < DirectionalVel() ) ) {
        return 0.5 * fBrake_a1[ 0 ];
    }
    return fBrake_a1[ 0 ];
}

// a train coming to a stand with at least part of it on an up gradient: a long train brake released now cannot be
// applied again before the train stops and starts to roll back, so the train brake is not released any further, and the
// locomotive brake, which acts at once, helps to hold the train
bool
TController::uphill_stop_hold() const {

    return (
        ( BrakeSystem == TBrakeSystem::Pneumatic ) && is_train() && ( OrderCurrentGet().HasAny(DriverOrder::obeyTrain) )
     && ( VelNext == 0.0 )
     && ( ActualProximityDist <= fMaxProximityDist )
     && ( fAccGravity < EU07_AI_UPHILLGRAVITY )
     && ( std::abs( DirectionalVel() ) < 10.0 )
     // not while the train is to be driven on
     && ( AccDesired <= 0.0 ) );
}

// uphill with the train brake applied, the gradient brings a slow train to a stand sooner than the brake of the consist
// can be released: letting the train roll on (or pulling up) at this speed only releases the brake for nothing, and the
// train stands with the brake on its way off instead
bool
TController::uphill_brake_committed( double const Velocity ) const {

    return (
        ( BrakeSystem == TBrakeSystem::Pneumatic )
     && ( BrakeCtrlPosition > 0.0 )
     && ( fAccGravity < EU07_AI_UPHILLGRAVITY )
     && ( Velocity / 3.6 / -fAccGravity < brake_response_horizon( mvOccupied, BrakeSystem, -1.0, fBrake_a0[ 0 ] ) ) );
}

void TController::control_braking_force() {

    // don't touch the brakes when compressing buffers during coupling/uncoupling
    if( drivingFlags.press ) { return; }

    auto const velocity { DirectionalVel() };
    // a train slipping back uphill is braked, also when it is to go on: released now it only rolls back further.
    // its head, or the train as a whole: the locomotive may stop for a moment while the rest of the train runs into it
    auto const slippingbackuphill {
        ( fAccGravity < -0.05 )
     && ( std::min( velocity, fConsistVelocity ) < -0.1 )
     && ( false == bUphillStart ) };

    // jeśli przyspieszamy, to nie hamujemy
    if( AccDesired > 0.0 && OrderCurrentGet() != DriverOrder::disconnect // przy odłączaniu nie zwalniamy tu hamulca
	    && false == drivingFlags.press
	    && false == slippingbackuphill ) {
		cue_action(driver_hint::brakingforcesetzero);
	}

    // AbsAccS is the resultant acceleration, gravity included, so it is compared with the
    // resultant the AI asks for, not with the brake-only demand AccDesired holds on a descent
    auto const accdesiredresultant { AccDesired + fAccDesiredGravity };

    // every few seconds, note how far the train falls short of the demand and whether that is getting worse: a brake
    // still filling, or blocks gripping harder as the train slows down, make up a shortfall on their own, but a
    // shortfall that keeps growing at a constant handle means the train is falling further behind its braking curve
    if( ElapsedTime - fShortfallTime >= 3.0 ) {
        auto const shortfall { AbsAccS - accdesiredresultant };
        iShortfallGrowth = (
            ( ( AccDesired < 0.0 ) && ( shortfall > fShortfall + 0.005 ) ) ?
                iShortfallGrowth + 1 :
                0 );
        fShortfall = shortfall;
        fShortfallTime = ElapsedTime;
    }

    if( is_emu() && !ForcePNBrake ) {
        // właściwie, to warunek powinien być na działający EP
        // Ra: to dobrze hamuje EP w EZT
        auto const accthreshold { (
            fAccGravity < 0.025 ? // HACK: when going downhill be more responsive to desired deceleration
                fAccThreshold :
                std::max( -0.2, fAccThreshold ) ) };
		auto const AccMax{ std::min(fBrake_a0[0] + 12 * fBrake_a1[0], mvOccupied->MED_amax) };
		auto const accmargin = AccMax > 1.1 * AccDesired && fAccGravity < 0.025 ?
							0.05 : 0.0;
        if( ( AccDesired < accthreshold // jeśli hamować - u góry ustawia się hamowanie na fAccThreshold
           && ( AbsAccS > accdesiredresultant + accmargin
             || BrakeCtrlPosition < 0 ) )
         || ( fAccGravity < -0.05 && velocity < -0.1 && AccDesired < 0.0 ) ) { // also brake if uphill and slipping back
            // hamować bardziej, gdy aktualne opóźnienie hamowania mniejsze niż (AccDesired)
            cue_action( driver_hint::brakingforceincrease );
        }
        else if( OrderCurrentGet() != DriverOrder::disconnect ) { // przy odłączaniu nie zwalniamy tu hamulca
            if( AbsAccS < accdesiredresultant - 0.05 ) {
                // jeśli opóźnienie większe od wymaganego (z histerezą) luzowanie, gdy za dużo
                // TBD: check if the condition isn't redundant with the DecBrake() code
                if( BrakeCtrlPosition >= 0 && VelDesired > 0.0 ) {
					// sanity check to prevent unintended brake release on sharp slopes
					cue_action(driver_hint::brakingforcedecrease);
				}
            }
            else {
                cue_action( driver_hint::brakingforcelap );
            }
        }
    } // type & dt_ezt
    else {
        // a stara wersja w miarę dobrze działa na składy wagonowe
        if( (accdesiredresultant < fAccGravity - 0.1 && AbsAccS > accdesiredresultant + brake_increase_margin() // regular braking
		     || slippingbackuphill) && // also brake if uphill and slipping back
		    (fBrakeTime < 0.0
		     // the size of the brake demand decides whether to skip the reaction time, so the brake-only part
		     // (on a descent a heavy train needs its brake built up at once, the gravity does not help it slow)
		     || AccDesired < fAccGravity - 0.5 || BrakeCtrlPosition <= 0) ) {
            // u góry ustawia się hamowanie na fAccThreshold
			// jeśli upłynął czas reakcji hamulca, chyba że nagłe albo luzował
			// TODO: check whether brake delay variable still has any purpose
			cue_action(driver_hint::brakingforceincrease,
			           // Ra: ten czas należy zmniejszyć, jeśli czas dojazdu do zatrzymania jest mniejszy
			           (3.0 + 0.5 * ((mvOccupied->BrakeDelayFlag > bdelay_G ? mvOccupied->BrakeDelay[1] : mvOccupied->BrakeDelay[3]) - 3.0)) * 0.5); // Ra: tymczasowo, bo przeżyna S1
		}
        else if( ( BrakeSystem == TBrakeSystem::Pneumatic )
              && ( AccDesired < 0.0 )
              && ( BrakeCtrlPosition > 0.0 )
              && ( fShortfall > 0.02 )
              && ( iShortfallGrowth >= 2 )
              && ( fBrakeHeldTime > brake_response_horizon( mvOccupied, BrakeSystem, 1.0, fBrake_a0[ 0 ] ) ) ) {
            // the train has been falling further behind the demand for a while at a handle left alone for longer than
            // the brake takes to respond: a small step now, while it still is small, rather than the deep one the
            // margin above would allow only once the train is far behind
            cue_action( driver_hint::brakingforceincrease );
            iShortfallGrowth = 0;
        }
        if ( (accdesiredresultant < fAccGravity - 0.05
              || AccDesired >= EU07_AI_NOACCELERATION) // no braking asked for, yet the train is slowing down more than that
            && accdesiredresultant - fBrake_a1[0] * 0.51 - AbsAccS > 0.05 && OrderCurrentGet() != DriverOrder::disconnect && VelDesired > 0.0
            // a human driver standing with the train not charged yet keeps the locomotive brake on, told so by the
            // waitbrakecharge hint; the ai is held back in DecBrake()
            && ( AIControllFlag
              || ( false == brake_charge::release_waits(
                      IsConsistCharged,
                      mvOccupied->Vel < EU07_AI_NOMOVEMENT,
                      mvOccupied->fBrakeCtrlPos > mvOccupied->Handle->GetPos( bh_RP ) + 0.2 ) ) ) ) { // the human driver's handle
            // jak hamuje, to nie tykaj kranu za często
            // yB: luzuje hamulec dopiero przy różnicy opóźnień rzędu 0.2
			// przy odłączaniu nie zwalniamy tu hamulca
			// sanity check to prevent unintended brake release on sharp slopes
			// TODO: check whether brake delay variable still has any purpose
			cue_action(driver_hint::brakingforcedecrease,
			           (mvOccupied->BrakeDelayFlag > bdelay_G ? mvOccupied->BrakeDelay[0] : mvOccupied->BrakeDelay[2]) / 3.0 * 0.5); // Ra: tymczasowo, bo przeżyna S1
		}
        // uphill, the locomotive brake acts at once while the train brake of a long train takes many seconds to apply:
        // put it on as the train comes to a stand, and at once if the train has started to roll back
        if( ( mvOccupied->LocalBrake != TLocalBrake::ManualBrake )
         && ( mvOccupied->LocalBrakePosA < 1.0 )
         && ( ( uphill_stop_hold() && ( std::abs( velocity ) < 3.0 )
             // a train that has slowed down short of where it is to stop and is to pull up further does not get it
             && ( VelDesired <= std::abs( velocity ) + 0.5 )
             && ( ActualProximityDist - uphill_stopping_distance( std::abs( velocity ), fAccGravity ) <= stopping_margin() ) )
           || ( ( BrakeSystem == TBrakeSystem::Pneumatic ) && ( fAccGravity < EU07_AI_UPHILLGRAVITY ) && ( velocity < -0.1 ) && ( AccDesired <= 0.0 ) ) ) ) {
            cue_action( driver_hint::independentbrakeapply );
        }
        // stop-gap measure to ensure cars actually brake to stop even when above calculactions go awry
        // instead of releasing the brakes and creeping into obstacle at 1-2 km/h
        if( is_car() && VelDesired == 0.0 && velocity > VelDesired && ActualProximityDist <= fMinProximityDist && mvOccupied->LocalBrakePosA < 0.01 ) {
			cue_action(driver_hint::brakingforceincrease);
		}
    }

    // odhamowywanie składu po zatrzymaniu i zabezpieczanie lokomotywy
    if( mvOccupied->Vel < EU07_AI_NOMOVEMENT
     && ( VelDesired == 0.0
       || AccDesired <= EU07_AI_NOACCELERATION ) ) {

        if( false == OrderCurrentGet().HasAny(DriverOrder::disconnect | DriverOrder::connect | DriverOrder::changeDirection) // przy (p)odłączaniu nie zwalniamy tu hamulca
         && std::abs(fAccGravity) < 0.01 ) { // only do this on flats, on slopes keep applied the train brake
            apply_independent_brake_only();
        }
        // if told to change direction don't confuse human driver with request to leave applied brake in the cab they're about to leave
        if( OrderCurrentGet().HasAny(DriverOrder::changeDirection) ) {
            cue_action( driver_hint::independentbrakerelease );
        }
    }
}

void TController::apply_independent_brake_only() {

    if( mvOccupied->LocalBrake != TLocalBrake::ManualBrake ) {
        if( false == mvOccupied->ShuntMode // no need for independent brake if autobrake is active
         && is_equal(mvOccupied->fBrakeCtrlPos, mvOccupied->Handle->GetPos(bh_RP), 0.2) ) {
            cue_action( driver_hint::independentbrakeapply );
        }
        else {
            cue_action( driver_hint::trainbrakerelease );
        }
    }
}

void TController::control_releaser() {
    // HACK: don't immediately touch releaser in inert vehicle, in case the simulation is about to replace us with human driver
    if( mvOccupied->Vel < 1.0 && fActionTime < 0.0 ) { return; }
    // TODO: combine all releaser handling in single decision tree instead of having bits all over the place
    // TODO: replace handle and system conditions with flag indicating releaser presence
    if( BrakeSystem != TBrakeSystem::Pneumatic ) { return; }

    auto const hasreleaser {
        false == (is_dmu() || is_emu()) // TODO: a more refined test than rejecting these types wholesale
     && ( mvOccupied->BrakeHandle == TBrakeHandle::FV4a
       || mvOccupied->BrakeHandle == TBrakeHandle::MHZ_6P
       || mvOccupied->BrakeHandle == TBrakeHandle::MHZ_K5P
       || mvOccupied->BrakeHandle == TBrakeHandle::MHZ_K8P
       || mvOccupied->BrakeHandle == TBrakeHandle::M394 ) };

    if( false == hasreleaser ) { return; }

    brake_releaser::conditions state;
    state.acceleration_desired = AccDesired;
    state.pipe_pressure = mvOccupied->PipePress;
    state.cylinder_pressure = mvOccupied->BrakePress;
    state.control_pressure = mvOccupied->Hamulec->GetCRP();
    state.velocity = mvOccupied->Vel;
    state.uphill_start = bUphillStart;
    state.pressing = drivingFlags.press;
    // stage 3 of uncoupling (UpdateDisconnect), from the direction change until the coupler lets go
    state.compressing_buffers =
        state.pressing && OrderCurrentGet().HasAny(DriverOrder::disconnect) && iVehicleCount >= 0 && iDirection == iDirectionOrder;
    // a train prepared standing charges its brakes before its first start. a pipe left below the lock of the driver's
    // brake valve (a train placed cold, as after an emergency application) is unlocked as the train stands, not only
    // once it is to go: else it charges only after the order to go, and starts on what the reservoirs got by then.
    // a handle which has to be moved to unlock is left where it holds the train with the train brake
    state.charging_standing =
        bBrakeChargeStart
     && ( mvOccupied->Vel < EU07_AI_NOMOVEMENT )
     && ( ( mvOccupied->HandleUnlock == -3 ) || ( BrakeCtrlPosition <= 0.0 ) );
    auto const wanted { brake_releaser::decide( state ) };
    auto const actuate { wanted.actuate };
    auto isbrakehandleinrightposition { true };

    // some vehicles require brake handle to be moved to specific position to fill up the brake pipe
    if( wanted.fill_pipe
     && mvOccupied->HandleUnlock != -3 ) {
        cue_action( driver_hint::trainbrakesetpipeunlock );
        isbrakehandleinrightposition = ( BrakeCtrlPosition == mvOccupied->HandleUnlock );
    }

    if( actuate ) {
        // some vehicles may require master controller in neutral position
        cue_action( driver_hint::mastercontrollersetzerospeed );
        if( false == mvOccupied->Hamulec->Releaser()
         && isbrakehandleinrightposition ) {
            cue_action( driver_hint::releaseron );
        }
    }
    else {
        if( true == mvOccupied->Hamulec->Releaser() ) {
            cue_action( driver_hint::releaseroff );
        }
    }
}

void TController::control_main_pipe() {

    // unlocking main pipe
	if( ( AccDesired > -0.03
       // standing to charge before the first start, as in control_releaser()
       || ( bBrakeChargeStart && ( mvOccupied->Vel < EU07_AI_NOMOVEMENT ) ) )
     && true == mvOccupied->LockPipe ) {
		UniversalBrakeButtons |= TUniversalBrake::ub_UnlockPipe;
	}
	else if (false == mvOccupied->LockPipe ) {
		UniversalBrakeButtons &= ~TUniversalBrake::ub_UnlockPipe;
	}

    // napełnianie uderzeniowe
    if( BrakeSystem != TBrakeSystem::Pneumatic ) { return; }

    if( mvOccupied->BrakeHandle == TBrakeHandle::FV4a
     || mvOccupied->BrakeHandle == TBrakeHandle::MHZ_6P
     || mvOccupied->BrakeHandle == TBrakeHandle::MHZ_K5P
     || mvOccupied->BrakeHandle == TBrakeHandle::MHZ_K8P
     || mvOccupied->BrakeHandle == TBrakeHandle::M394 ) {

        auto const departblocked { is_departure_blocked( DepartBlockedFSDelay ) };
        if( (drivingFlags.oerlikons || true == IsCargoTrain || departblocked) && BrakeCtrlPosition == gbh_RP && AbsAccS < 0.03 && AccDesired > -0.03 && VelDesired - mvOccupied->Vel > 2.0 && fReady > 0.35 &&
		    ( mvOccupied->EqvtPipePress < 4.5 || departblocked ) // don't charge with a risk of overcharging the main pipe
		    && mvOccupied->Compressor >= 7.5 // don't charge without sufficient pressure in the tank
		    && BrakeChargingCooldown >= 0.0 // don't charge while cooldown is active
		    && (ActualProximityDist > 100.0 // don't charge if we're about to be braking soon
		        || min_speed(mvOccupied->Vel, VelNext) >= mvOccupied->Vel) ) {

			BrakeLevelSet(gbh_FS);
			// don't charge the brakes too often, or we risk overcharging
			BrakeChargingCooldown = -1 * std::clamp(iVehicles * 3, 30, 90);
		}

        update_brake_charging();

        if( ( mvOccupied->Compressor < 5.0
           || ( BrakeCtrlPosition < gbh_RP
             && ( mvOccupied->EqvtPipePress > (fReady < 0.25 ? 5.1 : 5.2) || is_brake_charging_done() ) ) )
         // the light application a charging train stands on stays, as in DecBrake()
         && ( false == brake_charge::holds_application(
                bBrakeChargeHold, IsConsistCharged, mvOccupied->Vel < EU07_AI_NOMOVEMENT, BrakeCtrlPosition ) ) ) {
            cue_action( driver_hint::trainbrakerelease );
        }
    }
}

void TController::update_brake_charging() {

    if( std::abs( BrakeCtrlPosition - gbh_FS ) >= 0.5 ) {
        BrakeChargingStart = -1.0;
        BrakeChargingPipeOverchargeStart.clear();
        return;
    }
    if( BrakeChargingStart < 0.0 ) {
        BrakeChargingStart = ElapsedTime;
    }
    for( auto const *vehicle { pVehicles[ end::front ] }; vehicle != nullptr; vehicle = vehicle->Next() ) {
        auto const isovercharged {
            vehicle->MoverParameters != mvOccupied
         && vehicle->MoverParameters->PipePress > FSOverchargePipePress };
        if( isovercharged ) {
            BrakeChargingPipeOverchargeStart.try_emplace( vehicle, ElapsedTime );
        }
        else {
            BrakeChargingPipeOverchargeStart.erase( vehicle );
        }
    }
}

bool TController::is_departure_blocked( double const Delay ) const {

    return DepartBlockedStart >= 0.0
        && ElapsedTime - DepartBlockedStart > Delay;
}

double TController::consist_max_crp() const {

    auto result { 0.0 };
    for( auto const *vehicle { pVehicles[ end::front ] }; vehicle != nullptr; vehicle = vehicle->Next() ) {
        result = std::max( result, vehicle->MoverParameters->Hamulec->GetCRP() );
    }
    return result;
}

bool TController::is_brake_charging_done() const {

    if( BrakeChargingStart < 0.0 ) {
        return false;
    }
    return ElapsedTime - BrakeChargingStart > FSMaxTime
        || consist_max_crp() > mvOccupied->HighPipePress + FSOverchargeCRP
        || std::ranges::any_of(
            BrakeChargingPipeOverchargeStart,
            [this]( auto const &Vehicle ) { return ElapsedTime - Vehicle.second > FSOverchargeTime; } );
}

void
TController::UpdateBrakingHelper() {

	if (HelperState > 0 && -AccDesired < fBrake_a0[0] + 2 * fBrake_a1[0] && mvOccupied->Vel > EU07_AI_NOMOVEMENT) {
		HelperState = 0;
	}

    switch( HelperState ) {
        case 0: {
            if( -AccDesired > fBrake_a0[0] + 8 * fBrake_a1[0] ) {
                HelperState = 1;
            }
            break;
        }
        case 1: {
            if( -AccDesired > fBrake_a0[0] + 12 * fBrake_a1[0] ) {
                HelperState = 2;
            }
            break;
        }
        case 2: {
            if( -AccDesired > 0 && -ActualProximityDist > 5 ) {
                HelperState = 3;
            }
            break;
        }
        default: {
            break;
        }
    }
}
