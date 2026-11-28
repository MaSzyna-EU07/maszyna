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

// the controllers and the brake handles the driver works

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


// one step more of the train brake, sized from the consist brake table against the deceleration asked for
bool
TController::increase_train_brake() {

    // dodane dla towarowego
    float pos_corr = 0;

    TDynamicObject *d;
    d = pVehicles[0]; // pojazd na czele składu
    while (d)
    { // przeliczanie dodatkowego potrzebnego spadku ciśnienia
        if( ( d->MoverParameters->Hamulec->GetBrakeStatus() & b_dmg ) == 0 ) {
            pos_corr -= ( std::min( 5.0, d->MoverParameters->Hamulec->GetCRP() ) - 5.0 ) * d->MoverParameters->TotalMass;
        }
        d = d->Next(); // kolejny pojazd, podłączony od tyłu (licząc od czoła)
    }
    pos_corr = pos_corr / fMass * 2.5;

    if (mvOccupied->BrakeHandle == TBrakeHandle::FV4a)
    {
        pos_corr += mvOccupied->Handle->GetCP()*0.2;

    }
    if( double const deltaAcc = -AccDesired*BrakeAccFactor() - (fBrake_a0[0] + 4.0 * (/*GBH mvOccupied->BrakeCtrlPosR*/BrakeCtrlPosition - 1 - pos_corr)*fBrake_a1[0]);
        deltaAcc > brake_increase_margin() )
    {
        if( /*GBH mvOccupied->BrakeCtrlPosR*/BrakeCtrlPosition < 0.1 ) {
            auto const OK { /*mvOccupied->*/BrakeLevelAdd( BrakingInitialLevel ) }; //GBH
            // HACK: stronger braking to overcome SA134 engine behaviour
            if( is_dmu()
             && VelNext == 0.0
             && fBrakeDist < 200.0 ) {
                BrakeLevelAdd(
                    fBrakeDist / ActualProximityDist < 0.8 ?
                        0.5 :
                        1.0 );
            }
            return OK;
        }
        auto const OK { /*mvOccupied->*/BrakeLevelAdd( BrakingLevelIncrease ) }; //GBH
        // brake harder if the acceleration is much higher than desired
        /*if( ( deltaAcc > 2 * fBrake_a1[ 0 ] )
         && ( mvOccupied->BrakeCtrlPosR + BrakingLevelIncrease <= 5.0 ) ) {
            mvOccupied->BrakeLevelAdd( BrakingLevelIncrease );
        }  GBH */
        if( deltaAcc > 2 * fBrake_a1[0]
         && BrakeCtrlPosition + BrakingLevelIncrease <= 5.0 ) {
            /*mvOccupied->*/BrakeLevelAdd( BrakingLevelIncrease );
        }
        return OK;
    }
    if( BrakeCtrlPosition > 0.0
     && AccDesired < 0.0 // only while braking is asked for, not when called for a train slipping back while starting
     && ( fBrakeHeldTime > (
             mvOccupied->BrakeDelayFlag > bdelay_G ?
                 mvOccupied->BrakeDelay[ 1 ] :
                 mvOccupied->BrakeDelay[ 3 ] )
       // a train falling further behind needs no full fill time to be seen not to get enough
       || ( ( fShortfall > 0.02 )
         && ( fBrakeHeldTime > brake_response_horizon( mvOccupied, BrakeSystem, 1.0, fBrake_a0[ 0 ] ) ) ) ) ) {
        // the table says the present position is enough, but the brake has had its full fill
        // time since the handle last moved and the train is still not slowing down as asked:
        // believe the train rather than the table
        return BrakeLevelAdd( BrakingLevelIncrease );
    }
    return false;
}

bool TController::IncBrake()
{ // zwiększenie hamowania
    auto OK { false };
    auto const bs {
        BrakeSystem == TBrakeSystem::ElectroPneumatic && ForcePNBrake ?
            TBrakeSystem::Pneumatic :
            BrakeSystem };
    switch( bs ) {
        case TBrakeSystem::Individual: {
            if( mvOccupied->LocalBrake == TLocalBrake::ManualBrake ) {
                OK = mvOccupied->IncManualBrakeLevel( 1 + static_cast<int>( std::floor( 0.5 + std::fabs( AccDesired ) ) ) );
            }
            else {
                OK = mvOccupied->IncLocalBrakeLevel( std::floor( 1.5 + std::abs( AccDesired ) ) );
            }
            break;
        }
        case TBrakeSystem::Pneumatic: {
			if (bs != mvOccupied->BrakeSystem)
			{
				while (mvOccupied->BrakeOpModeFlag > bom_PN)
				{
					mvOccupied->BrakeOpModeFlag >>= 1;
				}
			}
            // NOTE: can't perform just test whether connected vehicle == nullptr, due to virtual couplers formed with nearby vehicles
            auto standalone { true };
            if( mvOccupied->TrainType == train_type::et41
             || mvOccupied->TrainType == train_type::et42 ) {
                // NOTE: we're doing simplified checks full of presuptions here.
                // they'll break if someone does strange thing like turning around the second unit
                if( mvOccupied->Couplers[end::rear].CouplingFlag & Coupling::permanent
                 && mvOccupied->Couplers[end::rear].Connected->Couplers[end::rear].Connected != nullptr ) {
                    standalone = false;
                }
                if( mvOccupied->Couplers[end::front].CouplingFlag & Coupling::permanent
                 && mvOccupied->Couplers[end::front].Connected->Couplers[end::front].Connected != nullptr ) {
                    standalone = false;
                }
            }
            else if( is_dmu() ) {
                // enforce use of train brake for DMUs
                standalone = false;
            }
            else if( OrderCurrentGet().HasAny(DriverOrder::looseShunt)
                  && mvOccupied->Vel - VelDesired < fVelPlus + fVelMinus ) {
                // try to reduce train brake use in loose shunting mode
                standalone = true;
            }
            else {
/*
                standalone =
                    ( ( mvOccupied->Couplers[ 0 ].CouplingFlag.Empty() )
                   && ( mvOccupied->Couplers[ 1 ].CouplingFlag.Empty() ) );
*/
                if( pVehicles[ end::front ] != pVehicles[ end::rear ] ) {
                    // more detailed version, will use manual braking also for coupled sets of controlled vehicles
                    auto *vehicle = pVehicles[ end::front ]; // start from first
                    while( true == standalone
                        && vehicle != nullptr ) {
                        // NOTE: we could simplify this by doing only check of the rear coupler, but this can be quite tricky in itself
                        // TODO: add easier ways to access front/rear coupler taking into account vehicle's direction
                        standalone =
                            (vehicle->MoverParameters->Couplers[end::front].Connected == nullptr ||
					                  (vehicle->MoverParameters->Couplers[end::front].CouplingFlag & Coupling::control && vehicle->MoverParameters->Couplers[end::front].Connected->Power > 1)) &&
					                 (vehicle->MoverParameters->Couplers[end::rear].Connected == nullptr ||
					                  (vehicle->MoverParameters->Couplers[end::rear].CouplingFlag & Coupling::control && vehicle->MoverParameters->Couplers[end::rear].Connected->Power > 1));
                        vehicle = vehicle->Next(); // kolejny pojazd, podłączony od tyłu (licząc od czoła)
                    }
                }
            }

            //standalone = standalone && ( mvControlling->EIMCtrlType == 0 );

            if( true == standalone && false == ForcePNBrake ) {
                if( mvControlling->EIMCtrlType > 0 ) {
                    OK = IncBrakeEIM();
                }
                else {
                    OK = mvOccupied->IncLocalBrakeLevel(
                        1 + static_cast<int>( std::floor( 0.5 + std::fabs( AccDesired ) ) ) ); // hamowanie lokalnym bo luzem jedzie
                }
            }
            else {
                if( /*GBH mvOccupied->BrakeCtrlPos*/ BrakeCtrlPosition + 1 == gbh_MAX ) {
					if (AccDesired < -1.5) // hamowanie nagle
						OK = /*mvOccupied->*/BrakeLevelAdd(1.0); //GBH
                    else
                        OK = false;
                }
                else {
                    OK = increase_train_brake();
                }
            }
            if( /*GBH mvOccupied->BrakeCtrlPos*/
		        /*GBH mvOccupied->BrakeCtrlPos*/ BrakeCtrlPosition > 0 && mvOccupied->Hamulec->Releaser() ) {
			    mvOccupied->BrakeReleaser(0);
		    }
            break;
        }
        case TBrakeSystem::ElectroPneumatic: {
            while( mvOccupied->BrakeOpModeFlag << 1 <= mvOccupied->BrakeOpModes ) {
                mvOccupied->BrakeOpModeFlag <<= 1;
            }
            if( mvOccupied->EngineType == TEngineType::ElectricInductionMotor ) {
				if (mvOccupied->BrakeHandle == TBrakeHandle::MHZ_EN57) {
					if (mvOccupied->BrakeCtrlPos < mvOccupied->Handle->GetPos(bh_FB))
						OK = mvOccupied->BrakeLevelAdd(1.0);
				}
				else {
					OK = IncBrakeEIM();
				}
            }
            else if( mvOccupied->Handle->TimeEP == false ) {
                auto const initialbrakeposition { mvOccupied->fBrakeCtrlPos };
                auto const AccMax { std::min( fBrake_a0[ 0 ] + 12 * fBrake_a1[ 0 ], mvOccupied->MED_amax ) };
                mvOccupied->BrakeLevelSet(
                    std::lerp(
                        mvOccupied->Handle->GetPos( bh_EPR ),
                        mvOccupied->Handle->GetPos( bh_EPB ),
                        std::clamp( -AccDesired / AccMax * mvOccupied->AIHintLocalBrakeAccFactor, 0.0, 1.0 ) ) );
                OK = mvOccupied->fBrakeCtrlPos != initialbrakeposition;
            }
            else if( mvOccupied->fBrakeCtrlPos != mvOccupied->Handle->GetPos( bh_EPB ) ) {
                mvOccupied->BrakeLevelSet( mvOccupied->Handle->GetPos( bh_EPB ) );
                if( mvOccupied->Handle->GetPos( bh_EPR ) - mvOccupied->Handle->GetPos( bh_EPN ) < 0.1 )
                    mvOccupied->SwitchEPBrake( 1 ); // to nie chce działać
                OK = true;
            }
            else if( ( mvOccupied->Handle->GetPos( bh_EPR ) - mvOccupied->Handle->GetPos( bh_EPN ) < 0.1 )
                  && ( mvOccupied->EpForce <= 0.0 )
                  && ( mvOccupied->EPBrakeControlPowered() ) ) {
                // the handle already in the EP brake position, but the EP brake dropped out with the control's
                // supply (e.g. the EP fuse off in the running brake test): switch it on again
                mvOccupied->SwitchEPBrake( 1 );
                OK = ( mvOccupied->EpForce > 0.0 );
            }
            else
                OK = false;
            break;
        }
        default: { break; }
    }
    return OK;
}

bool TController::IncBrakeEIM()
{ // zwiększenie hamowania
	bool OK = false;
	switch (mvOccupied->EIMCtrlType)
	{
        case 0: {
            if( mvOccupied->MED_amax != 9.81 ) {
                auto const maxpos{mvOccupied->EIMCtrlEmergency ? 0.9 : 1.0 };
                auto const brakelimit{ -2.2 * AccDesired / fMedAmax - 1.0}; //additional limit when hinted is too low
				auto const brakehinted{ -1.0 * mvOccupied->AIHintLocalBrakeAccFactor * AccDesired / fMedAmax }; //preffered by AI
                auto const brakeposition{ maxpos * std::clamp(std::max(brakelimit, brakehinted), 0.0, 1.0)};
                OK = brakeposition != mvOccupied->LocalBrakePosA;
                mvOccupied->LocalBrakePosA = brakeposition;
            }
            else {
                OK = mvOccupied->IncLocalBrakeLevel( 1 );
            }
            break;
        }
        case 1: {
            OK = mvOccupied->MainCtrlPos > 0;
            if( OK )
                mvOccupied->MainCtrlPos = 0;
            break;
        }
        case 2: {
            // the braking position, above the emergency brake one where the controller has it
            auto const brakepos{1 + mvOccupied->EIMCtrlEmergencyPositions()};
            OK = mvOccupied->MainCtrlPos > brakepos;
            if( OK )
                mvOccupied->MainCtrlPos = brakepos;
            break;
        }
        case 3: {
            if( mvOccupied->UniCtrlIntegratedLocalBrakeCtrl ) {
                OK = mvOccupied->MainCtrlPos > 0;
                if( OK )
                    mvOccupied->MainCtrlPos = 0;
            }
            else {
                OK = mvOccupied->IncLocalBrakeLevel( 1 );
            }
            break;
        }
        default: {
            break;
        }
	}
	return OK;
}

// zmniejszenie siły hamowania
bool TController::DecBrake() {

    auto OK { false };
    auto const bs {
        BrakeSystem == TBrakeSystem::ElectroPneumatic && ForcePNBrake ?
            TBrakeSystem::Pneumatic :
            BrakeSystem };
    switch( bs ) {
        case TBrakeSystem::Individual: {
            auto const positionchange { 1 + std::floor( 0.5 + std::abs( AccDesired ) ) };
            OK = mvOccupied->LocalBrake == TLocalBrake::ManualBrake ? mvOccupied->DecManualBrakeLevel(positionchange) : mvOccupied->DecLocalBrakeLevel(positionchange);
            break;
        }
        case TBrakeSystem::Pneumatic: {
            auto deltaAcc { -1.0 };
            if( fBrake_a0[0] != 0.0
             || fBrake_a1[0] != 0.0 ) {
                auto const pos_diff { ( is_dmu() ? 0.25 : 1.0 ) };
                // a distributor applied at all gives at least its minimum stage, about what the first position does, so
                // a position short of the first one brakes no weaker than that
                deltaAcc = -AccDesired * BrakeAccFactor() - ( fBrake_a0[ 0 ] + 4 * std::max( 0.0, BrakeCtrlPosition - pos_diff ) * fBrake_a1[ 0 ] );
            }
            if( bUphillStart && ( false == bUphillRelease ) ) {
                // starting uphill the train brake holds the train until the power does
                break;
            }
            if( ( BrakeCtrlPosition > 0.0 )
             && ( uphill_stop_hold() ) ) {
                // any release now raises the pipe, and a long train releases more than the handle is moved back, too late
                // to apply again before the train stands: keep the brake as it is, and the locomotive brake with it
                break;
            }
            if( brake_charge::holds_application(
                    bBrakeChargeHold, IsConsistCharged, mvOccupied->Vel < EU07_AI_NOMOVEMENT, BrakeCtrlPosition ) ) {
                // the order to go came while the train still charges: released now, the wagons let go as their
                // reservoirs fill and the train stands on a locomotive brake that doesn't hold it. the locomotive
                // brake stays on with it
                break;
            }
		    if (deltaAcc < 0 && /*GBH mvOccupied->BrakeCtrlPosR*/ BrakeCtrlPosition > 0) {
			    OK = /*mvOccupied->*/ BrakeLevelAdd(-0.25);
			    // if ((deltaAcc < 5 * fBrake_a1[0]) && (mvOccupied->BrakeCtrlPosR >= 1.2))
			    //	mvOccupied->BrakeLevelAdd(-1.0);
			    /* if (mvOccupied->BrakeCtrlPosR < 0.74) GBH */
			    if (BrakeCtrlPosition < 0.74)
				    /*mvOccupied->*/ BrakeLevelSet(gbh_RP);
		    }
            // standing, the locomotive brake holds the train until its brakes are charged: off before, a train
            // released on a gradient rolls with no air to brake with. nor through DecBrakeEIM(), which takes it off
            // as well (the locomotive brake of a vehicle without the integrated handle)
            auto const holdlocomotive {
                ( false == IsConsistCharged )
             && ( mvOccupied->Vel < EU07_AI_NOMOVEMENT ) };
            if( ( !OK ) && ( false == holdlocomotive ) ) {
                OK = mvOccupied->DecLocalBrakeLevel(2);
            }
            if( ( !OK ) && ( false == holdlocomotive ) ) {
			    OK = DecBrakeEIM();
		    }
    /*
    // NOTE: disabled, duplicate of AI's behaviour in UpdateSituation()
            if (mvOccupied->PipePress < 3.0)
                Need_BrakeRelease = true;
    */
            break;
        }
        case TBrakeSystem::ElectroPneumatic: {
            if( mvOccupied->EngineType == TEngineType::ElectricInductionMotor ) {
                if( mvOccupied->BrakeHandle == TBrakeHandle::MHZ_EN57 ) {
                    if( mvOccupied->BrakeCtrlPos > mvOccupied->Handle->GetPos( bh_RP ) ) {
                        OK = mvOccupied->BrakeLevelAdd( -1.0 );
                    }
                }
                else {
                    OK = DecBrakeEIM();
                }
            }
            else if( mvOccupied->Handle->TimeEP == false ) {
                auto const initialbrakeposition { mvOccupied->fBrakeCtrlPos };
                auto const AccMax { std::min( fBrake_a0[ 0 ] + 12 * fBrake_a1[ 0 ], mvOccupied->MED_amax ) };
                mvOccupied->BrakeLevelSet(
                    std::lerp(
                        mvOccupied->Handle->GetPos( bh_EPR ),
                        mvOccupied->Handle->GetPos( bh_EPB ),
                        std::clamp( -AccDesired / AccMax * mvOccupied->AIHintLocalBrakeAccFactor, 0.0, 1.0 ) ) );
                OK = mvOccupied->fBrakeCtrlPos != initialbrakeposition;
            }
            else {
				OK = false;
				if (mvOccupied->fBrakeCtrlPos != mvOccupied->Handle->GetPos(bh_EPR)) {
					mvOccupied->BrakeLevelSet(mvOccupied->Handle->GetPos(bh_EPR));
					OK = true;
				}
				if (mvOccupied->Handle->GetPos(bh_EPR) - mvOccupied->Handle->GetPos(bh_EPN) < 0.1) {
					OK = OK || mvOccupied->SwitchEPBrake(1);
				}
            }
            if( !OK ) {
                OK = mvOccupied->DecLocalBrakeLevel( 2 );
            }
            break;
        }
        default: {
            break;
        }
    }
    return OK;
}

void TController::LapBrake() {

    if( mvOccupied->Handle->TimeEP ) {
        if( mvOccupied->Handle->GetPos( bh_EPR ) - mvOccupied->Handle->GetPos( bh_EPN ) < 0.1 ) {
            mvOccupied->SwitchEPBrake( 0 );
        }
        else {
            mvOccupied->BrakeLevelSet( mvOccupied->Handle->GetPos( bh_EPN ) );
        }
    }
}

void TController::ZeroLocalBrake() {

    if( mvOccupied->UniCtrlIntegratedLocalBrakeCtrl ) {
        while( DecBrakeEIM() ) {
            ;
        }
    }
    else {
        mvOccupied->DecLocalBrakeLevel( LocalBrakePosNo );
    }
}

bool TController::DecBrakeEIM()
{ // zmniejszenie siły hamowania
	bool OK = false;
	switch (mvOccupied->EIMCtrlType)
	{
        case 0: {
            if( mvOccupied->MED_amax != 9.81 ) {
                auto const desiredacceleration { ( mvOccupied->Vel > EU07_AI_NOMOVEMENT ? AccDesired : std::max( 0.0, AccDesired ) ) };
                auto const brakeposition { std::clamp( -1.0 * mvOccupied->AIHintLocalBrakeAccFactor * desiredacceleration / mvOccupied->MED_amax, 0.0, 1.0 ) };
                OK = brakeposition != mvOccupied->LocalBrakePosA;
                mvOccupied->LocalBrakePosA = brakeposition;
            }
            else {
                OK = mvOccupied->DecLocalBrakeLevel( 1 );
            }
            break;
        }
        case 1: {
            OK = mvOccupied->MainCtrlPos < 2;
            if( OK )
                mvOccupied->MainCtrlPos = 2;
            break;
        }
        case 2: {
            auto const neutralpos{mvOccupied->MainCtrlNoPowerPos()};
            OK = mvOccupied->MainCtrlPos < neutralpos;
            if( OK )
                mvOccupied->MainCtrlPos = neutralpos;
            break;
        }
        case 3: {
            OK = mvOccupied->MainCtrlPos < mvOccupied->MainCtrlMaxDirChangePos;
            if( OK )
                mvOccupied->MainCtrlPos = mvOccupied->MainCtrlMaxDirChangePos;
            break;
        }
    }
	return OK;
}

bool TController::IncSpeed()
{ // zwiększenie prędkości; zwraca false, jeśli dalej się nie da zwiększać
    auto OK { false };
    switch (mvOccupied->EngineType)
    {
    case TEngineType::None: // McZapkie-041003: wagon sterowniczy
        if (mvControlling->MainCtrlPosNo > 0) // jeśli ma czym kręcić
            drivingFlags.incSpeed = true; // ustawienie flagi jazdy
        return false;
    case TEngineType::ElectricSeriesMotor:
        if (mvPantographUnit->EnginePowerSource.SourceType == TPowerSource::CurrentCollector) // jeśli pantografujący
        {
            if (fOverhead2 >= 0.0) // a jazda bezprądowa ustawiana eventami (albo opuszczenie)
                return false; // to nici z ruszania
            if (iOverheadZero) // jazda bezprądowa z poziomu toru ustawia bity
                return false; // to nici z ruszania
        }
        // TODO: move all fVoltage assignments to a single, more generic place
        if( mvControlling->EnginePowerSource.SourceType == TPowerSource::Accumulator ) {
            fVoltage = mvControlling->BatteryVoltage;
        }
        if (!IsAnyMotorOverloadRelayOpen && !mvControlling->ControlPressureSwitch && (mvControlling->IsMainCtrlNoPowerPos() || mvControlling->StLinFlag) && (Ready || drivingFlags.press)) {
			// youBy polecił dodać 2012-09-08 v367
			// na pozycji 0 przejdzie, a na pozostałych będzie czekać, aż się załączą liniowe (zgaśnie DelayCtrlFlag)
			auto const usehighoverloadrelaythreshold{mvControlling->TrainType != train_type::et42 // ET42 uses these variables for different purpose. TODO: fix this
			                                         && mvControlling->ImaxHi > mvControlling->ImaxLo && mvOccupied->Vel < (mvControlling->IsMotorOverloadRelayHighThresholdOn() ? 30.0 : 20.0) &&
			                                         iVehicles - ControlledEnginesCount > 0
			                                         //                     && ( std::fabs( mvControlling->Im ) > 0.85 * mvControlling->Imax )
			                                         && mvControlling->Imax * mvControlling->EngineVoltage * ControlledEnginesCount /
			                                                    (fMass * (fAccGravity == 0.025 ? -0.01 : // prevent div/0
			                                                                                     fAccGravity - 0.025)) <
			                                                -2.8};
			// use series mode:
			// if high threshold is set for motor overload relay,
			// if the power station is heavily burdened,
			// if it generates enough traction force
			// to build up speed to 30/40 km/h for passenger/cargo train (10 km/h less if going uphill)
			auto const sufficienttractionforce{std::abs(mvControlling->Ft) * ControlledEnginesCount > (IsHeavyCargoTrain ? 75 : 50) * 1000.0};
			auto const sufficientacceleration{AbsAccS >= (IsHeavyCargoTrain ? 0.03 : IsCargoTrain ? 0.06 : 0.09)};
			auto const seriesmodefieldshunting{mvControlling->ScndCtrlPos > 0 && mvControlling->RList[mvControlling->MainCtrlPos].Bn == 1};
			auto const parallelmodefieldshunting{mvControlling->ScndCtrlPos > 0 && mvControlling->RList[mvControlling->MainCtrlPos].Bn > 1};
			auto const minvoltage{(mvPantographUnit->EnginePowerSource.SourceType == TPowerSource::CurrentCollector ? mvPantographUnit->EnginePowerSource.CollectorParameters.MinV : 0.0)};
			auto const maxvoltage{(mvPantographUnit->EnginePowerSource.SourceType == TPowerSource::CurrentCollector ? mvPantographUnit->EnginePowerSource.CollectorParameters.MaxV : 0.0)};
			auto const seriesmodevoltage{std::lerp(minvoltage, maxvoltage, IsHeavyCargoTrain ? 0.35 : 0.40)};
			auto const useseriesmode = mvControlling->Imax > mvControlling->ImaxLo || true == usehighoverloadrelaythreshold || fVoltage < seriesmodevoltage ||
			                           (true == sufficientacceleration && true == sufficienttractionforce &&
			                            mvOccupied->Vel <= (IsCargoTrain ? 40 : 30) + (seriesmodefieldshunting ? 5 : 0) - (fAccGravity < -0.025 ? 10 : 0));
			// when not in series mode use the first available parallel mode configuration until 50/60 km/h for passenger/cargo train
			// (if there's only one parallel mode configuration it'll be used regardless of current speed)
			auto const usefieldshunting = mvControlling->StLinFlag && mvControlling->RList[mvControlling->MainCtrlPos].R < 0.01 && false == mvControlling->IsScndCtrlLockedByMainCtrl() &&
			                              (useseriesmode ? mvControlling->RList[mvControlling->MainCtrlPos].Bn == 1 :
			                               true == sufficientacceleration && true == sufficienttractionforce && mvOccupied->Vel <= (IsCargoTrain ? 60 : 50) + (parallelmodefieldshunting ? 5 : 0) ?
			                                               mvControlling->RList[mvControlling->MainCtrlPos].Bn > 1 :
			                                               mvControlling->MainCtrlPos == mvControlling->MainCtrlPosNo);

			// if needed enable high threshold for overload relay...
			if (mvControlling->TrainType != train_type::et42)
			{ // ET42 uses these variables for different purpose. TODO: fix this
				if (usehighoverloadrelaythreshold)
				{
					if (mvControlling->Imax < mvControlling->ImaxHi)
					{
						// to enable this setting we'll typically need the main controller in series mode (which is not guaranteed)
						if (mvControlling->RList[mvControlling->MainCtrlPos].Bn > 1)
						{
							if (false == mvControlling->IsScndCtrlNoPowerPos())
							{
								mvControlling->DecScndCtrl(2);
							}
							while (false == mvControlling->IsMainCtrlNoPowerPos() && mvControlling->RList[mvControlling->MainCtrlPos].Bn > 1)
							{
								mvControlling->DecMainCtrl(1); // kręcimy nastawnik jazdy o 1 wstecz
							}
						}
						mvControlling->CurrentSwitch(true); // rozruch wysoki (za to może się ślizgać)
					}
				}
				// ...or disable high threshold for overload relay if no longer needed
				else
				{
					if (mvControlling->IsMotorOverloadRelayHighThresholdOn() && std::fabs(mvControlling->Im) < mvControlling->ImaxLo)
					{
						mvControlling->CurrentSwitch(false); // rozruch wysoki wyłącz
					}
				}
			}

			double Vs = 99999;
			if (usefieldshunting ? mvControlling->ScndCtrlPos < mvControlling->ScndCtrlPosNo : mvControlling->MainCtrlPos < mvControlling->MainCtrlPosNo)
			{
				Vs = ESMVelocity(!usefieldshunting);
			}

			if (std::abs(mvControlling->Im) < (fReady < 0.4 ? mvControlling->Imin : mvControlling->IminLo) || mvControlling->Vel > Vs)
			{
				// Ra: wywalał nadmiarowy, bo Im może być ujemne; jak nie odhamowany, to nie przesadzać z prądem
				if (usefieldshunting)
				{
					// to dać bocznik
					// engage the shuntfield only if there's sufficient power margin to draw from
					auto const sufficientpowermargin{fVoltage - seriesmodevoltage > (IsHeavyCargoTrain ? 100.0 : 75.0) * ControlledEnginesCount};

					OK = sufficientpowermargin ? mvControlling->IncScndCtrl(1) : true;
				}
				else
				{
					// jeśli ustawiony bocznik to bocznik na zero po chamsku
					if (mvControlling->ScndCtrlPos)
					{
						mvControlling->DecScndCtrl(2);
					}
					// kręcimy nastawnik jazdy
					// don't draw too much power;
					// keep from dropping into series mode when entering/using parallel mode, and from shutting down in the series mode
					auto const sufficientpowermargin{fVoltage -
					                                     (mvControlling->RList[std::min(mvControlling->MainCtrlPos + 1, mvControlling->MainCtrlPosNo)].Bn == 1 ? minvoltage : seriesmodevoltage) >
					                                 (IsHeavyCargoTrain ? 100.0 : 75.0) * ControlledEnginesCount};

					OK = sufficientpowermargin && false == mvControlling->DelayCtrlFlag ? mvControlling->IncMainCtrl(1) : true;
					// czekaj na 1 pozycji, zanim się nie włączą liniowe
					if (true == mvControlling->StLinFlag)
					{
						drivingFlags.incSpeed = true;
					}
					else
					{
						drivingFlags.incSpeed = false;
					}

					if (mvControlling->Im == 0 && mvControlling->MainCtrlPowerPos() > 1)
					{
						// brak prądu na dalszych pozycjach
						// nie załączona lokomotywa albo wywalił nadmiarowy
						Need_TryAgain = true;
					}
				}
			}
			else
			{
				OK = false;
			}
		}
//        mvControlling->AutoRelayCheck(); // sprawdzenie logiki sterowania
        break;
    case TEngineType::Dumb:
        if (!IsAnyMotorOverloadRelayOpen && (Ready || drivingFlags.press))
		//{(BrakePress<=0.01*MaxBrakePress)}
		{
			OK = IncSpeedEIM();
			if (!OK)
				OK = mvControlling->IncMainCtrl(1);
			if (!OK)
				OK = mvControlling->IncScndCtrl(1);
		}
		break;
    case TEngineType::DieselElectric:
        if (!IsAnyMotorOverloadRelayOpen && !mvControlling->ControlPressureSwitch && (mvControlling->IsMainCtrlNoPowerPos() || mvControlling->StLinFlag) && (Ready || drivingFlags.press)) {
			// youBy polecił dodać 2012-09-08 v367
			// na pozycji 0 przejdzie, a na pozostałych będzie czekać, aż się załączą liniowe (zgaśnie DelayCtrlFlag)
			//{(BrakePress<=0.01*MaxBrakePress)}
			OK = IncSpeedEIM();
			if (!OK)
				OK = mvControlling->IncMainCtrl(1);
			if (!OK)
				OK = mvControlling->IncScndCtrl(1);
		}
        break;
    case TEngineType::ElectricInductionMotor:
        if( !IsAnyMotorOverloadRelayOpen && (Ready || drivingFlags.press || mvOccupied->ShuntMode) ) {
			//{(BrakePress<=0.01*MaxBrakePress)}
			OK = IncSpeedEIM();
			if (mvControlling->SpeedCtrl && mvControlling->Mains)
			{
				// cruise control
				auto const couplinginprogress{true == drivingFlags.connect && true == OrderCurrentGet().HasAll(DriverOrder::connect)};
				auto const SpeedCntrlVel{(ActualProximityDist > std::max(50.0, fMaxProximityDist) || couplinginprogress ? VelDesired : min_speed(VelDesired, VelNext))};
				if (SpeedCntrlVel >= mvControlling->SpeedCtrlUnit.MinVelocity &&
				    SpeedCntrlVel - mvControlling->SpeedCtrlUnit.MinVelocity == quantize(SpeedCntrlVel - mvControlling->SpeedCtrlUnit.MinVelocity, mvControlling->SpeedCtrlUnit.VelocityStep))
				{
					SpeedCntrl(SpeedCntrlVel);
				}
				else if (SpeedCntrlVel > 0.1)
				{
					SpeedCntrl(0.0);
					mvControlling->DecScndCtrl(2);
				}
			}
		}
        break;
    case TEngineType::WheelsDriven:
        if (!mvControlling->CabActive)
            mvControlling->CabActivisation();
        if (sin(mvControlling->eAngle) > 0)
            mvControlling->IncMainCtrl(3 + 3 * floor(0.5 + fabs(AccDesired)));
        else
            mvControlling->DecMainCtrl(3 + 3 * floor(0.5 + fabs(AccDesired)));
        break;
    case TEngineType::DieselEngine:
        if (mvControlling->ShuntModeAllow)
        { // dla 2Ls150 można zmienić tryb pracy, jeśli jest w liniowym i nie daje rady (wymaga zerowania kierunku)
            // mvControlling->ShuntMode=(OrderList[OrderPos]&Shunt)||(fMass>224000.0);
        }
		if (mvControlling->SpeedCtrl && mvControlling->Mains) {// cruise control
            auto const couplinginprogress { true == drivingFlags.connect && true == OrderCurrentGet().HasAll(DriverOrder::connect) };
            auto const SpeedCntrlVel { (
                ActualProximityDist > std::max(50.0, fMaxProximityDist) || couplinginprogress ?
                    VelDesired :
                    min_speed( VelDesired, VelNext ) ) };
            if( SpeedCntrlVel >= mvControlling->SpeedCtrlUnit.MinVelocity
             && SpeedCntrlVel - mvControlling->SpeedCtrlUnit.MinVelocity == quantize(SpeedCntrlVel - mvControlling->SpeedCtrlUnit.MinVelocity, mvControlling->SpeedCtrlUnit.VelocityStep) ) {
                SpeedCntrl(SpeedCntrlVel);
            }
            else if (SpeedCntrlVel > 0.1) {
                SpeedCntrl(0.0);
                mvControlling->DecScndCtrl( 2 );
            }
		}
		if (mvControlling->EIMCtrlType > 0) {
			if (true == Ready)
			{
				bool max = mvControlling->Vel > mvControlling->dizel_minVelfullengage
				    	|| (mvControlling->SpeedCtrl && mvControlling->ScndCtrlPos > 0);
				DizelPercentage = max ? 100 : 1;
			}
			break;
		}
        else {
            if( true == Ready ) {
                if( mvControlling->Vel > mvControlling->dizel_minVelfullengage
                 && mvControlling->RList[mvControlling->MainCtrlPos].Mn > 0 ) {
                    OK = mvControlling->IncMainCtrl( 1 );
                }
                if( mvControlling->RList[ mvControlling->MainCtrlPos ].Mn == 0 ) {
                    OK = mvControlling->IncMainCtrl( 1 );
                }
            }
        }
        // TODO: move this check to a more suitable place, or scrap it
        if( false == mvControlling->Mains ) {
			SpeedCntrl(0.0);
            mvControlling->DecScndCtrl( 2 );
            cue_action( driver_hint::linebreakerclose );
            cue_action( driver_hint::converteron );
            cue_action( driver_hint::compressoron );
        }
		break;
    }
    return OK;
}

void TController::ZeroSpeed( bool const Enforce ) {

    while( DecSpeed( Enforce ) ) {
        ;
    }
}

bool TController::DecSpeed(bool force)
{ // zmniejszenie prędkości (ale nie hamowanie)
    bool OK = false; // domyślnie false, aby wyszło z pętli while
    switch (mvOccupied->EngineType)
    {
    case TEngineType::None: // McZapkie-041003: wagon sterowniczy
        drivingFlags.incSpeed = false; // usunięcie flagi jazdy
        if (force && mvControlling->MainCtrlPosNo > 0) // przy aktywacji kabiny jest potrzeba natychmiastowego wyzerowania
		                                               // McZapkie-041003: wagon sterowniczy, np. EZT
			mvControlling->DecMainCtrl(std::min(mvControlling->MainCtrlPowerPos(), 2));
		//        mvControlling->AutoRelayCheck(); // sprawdzenie logiki sterowania
        return false;
    case TEngineType::ElectricSeriesMotor:
        OK = mvControlling->DecScndCtrl(2); // najpierw bocznik na zero
        if (!OK)
            OK = mvControlling->DecMainCtrl( std::min( mvControlling->MainCtrlPowerPos(), 2 ) );
//        mvControlling->AutoRelayCheck(); // sprawdzenie logiki sterowania
        break;
    case TEngineType::Dumb:
    case TEngineType::DieselElectric:
        OK = mvControlling->DecScndCtrl(2);
        if (!OK)
            OK = mvControlling->DecMainCtrl( std::min( mvControlling->MainCtrlPowerPos(), 2 + mvControlling->MainCtrlPowerPos() / 2 ) );
        break;
	case TEngineType::ElectricInductionMotor:
		OK = DecSpeedEIM();
		break;
    case TEngineType::WheelsDriven:
        if (!mvControlling->CabActive)
            mvControlling->CabActivisation();
        if (sin(mvControlling->eAngle) < 0)
            mvControlling->IncMainCtrl(3 + 3 * floor(0.5 + fabs(AccDesired)));
        else
            mvControlling->DecMainCtrl(3 + 3 * floor(0.5 + fabs(AccDesired)));
        break;
    case TEngineType::DieselEngine:
		if (mvControlling->EIMCtrlType > 0)
		{
			DizelPercentage = 0;
			if (force) {
				SpeedCntrl(0.0); //wylacz od razu tempomat
				mvControlling->DecScndCtrl(2);
			}
			if (VelDesired > 0.1 && mvControlling->SpeedCtrlUnit.MinVelocity > VelDesired) {
				SpeedCntrl(0.0); //wylacz od razu tempomat
				mvControlling->DecScndCtrl(2);
			}
			break;
		}

        if (mvControlling->Vel > mvControlling->dizel_minVelfullengage)
        {
            if (mvControlling->RList[mvControlling->MainCtrlPos].Mn > 0)
                OK = mvControlling->DecMainCtrl(1);
        }
        else {
            while( mvControlling->RList[mvControlling->MainCtrlPos].Mn > 0
                && mvControlling->MainCtrlPowerPos() > 1 ) {
                OK = mvControlling->DecMainCtrl( 1 );
            }
        }
		if (force) { // przy aktywacji kabiny jest potrzeba natychmiastowego wyzerowania
			OK = mvControlling->DecMainCtrl(mvControlling->MainCtrlPowerPos() > 2 ? 2 : 1);
			SpeedCntrl(0.0); //wylacz od razu tempomat
			mvControlling->DecScndCtrl( 2 );
		}
        break;
    }
    return OK;
};

bool TController::IncSpeedEIM() {

    bool OK = false; // domyślnie false, aby wyszło z pętli while
    switch( mvControlling->EIMCtrlType ) {
        case 0:
            OK = mvControlling->IncMainCtrl( 1 );
            break;
        case 1:
            OK = mvControlling->MainCtrlPos < 6;
            if( OK ) {
                mvControlling->MainCtrlPos = 6;
            }
/*
            // TBD, TODO: set position based on desired acceleration?
            OK = mvControlling->MainCtrlPos < mvControlling->MainCtrlPosNo;
            if( OK ) {
                mvControlling->MainCtrlPos = safe_clamp( mvControlling->MainCtrlPos + 1, 6, mvControlling->MainCtrlPosNo );
            }
*/
            break;
        case 2: {
            auto const powerpos{4 + mvControlling->EIMCtrlEmergencyPositions()};
            OK = mvControlling->MainCtrlPos < powerpos;
            if( OK ) {
                mvControlling->MainCtrlPos = powerpos;
            }
            break;
        }
    }
    return OK;
}

bool TController::DecSpeedEIM( int const Amount )
{ // zmniejszenie prędkości (ale nie hamowanie)
	bool OK = false; // domyślnie false, aby wyszło z pętli while
    switch( mvControlling->EIMCtrlType ) {
        case 0: {
            OK = mvControlling->DecMainCtrl( Amount );
            break;
        }
        case 1: {
            OK = mvControlling->MainCtrlPos > 4;
            if( OK ) {
                mvControlling->MainCtrlPos = 4;
            }
            break;
        }
        case 2: {
            if( AccDesired > 0
             && mvControlling->SpeedCtrlUnit.IsActive
             && mvControlling->SpeedCtrlUnit.PowerStep > 0
             && mvControlling->SpeedCtrlUnit.DesiredPower > mvControlling->SpeedCtrlUnit.MinPower ) {
                mvControlling->SpeedCtrlPowerDec();
            }
            else {
                auto const holdpos{2 + mvControlling->EIMCtrlEmergencyPositions()};
                OK = mvControlling->MainCtrlPos > holdpos;
                if( OK ) {
                    mvControlling->MainCtrlPos = holdpos;
                }
            }
            break;
        }
        default: {
            break;
        }
	}
	return OK;
}

void TController::BrakeLevelSet(double b)
{ // ustawienie pozycji hamulca na wartość (b) w zakresie od -2 do BrakeCtrlPosNo
  // jedyny dopuszczalny sposób przestawienia hamulca zasadniczego
	if (BrakeCtrlPosition == b)
		return; // nie przeliczać, jak nie ma zmiany
	BrakeCtrlPosition = std::clamp(b, (double)gbh_MIN, (double)gbh_MAX);
	fBrakeHeldTime = 0.0;
}

bool TController::BrakeLevelAdd(double b)
{ // dodanie wartości (b) do pozycji hamulca (w tym ujemnej)
  // zwraca false, gdy po dodaniu było by poza zakresem
	BrakeLevelSet(BrakeCtrlPosition + b);
	return b > 0.0 ? BrakeCtrlPosition < gbh_MAX :
		BrakeCtrlPosition > -1.0; // true, jeśli można kontynuować
}

// Ra: regulacja prędkości, wykonywana w każdym przebłysku świadomości AI
// ma dokręcać do bezoporowych i zdejmować pozycje w przypadku przekroczenia prądu
void TController::SpeedSet() {

    if( false == AIControllFlag ) { return; }

    switch (mvOccupied->EngineType)
    {
    case TEngineType::None: { // McZapkie-041003: wagon sterowniczy
        if (mvControlling->MainCtrlPosNo > 0)
        { // jeśli ma czym kręcić
            // TODO: sprawdzanie innego czlonu //if (!FuseFlagCheck())
            if( ( false == drivingFlags.incSpeed ) ) {
                // przetok rozłączać od razu (no dependency on fActionTime)
                while( mvControlling->MainCtrlPos
                    && mvControlling->DecMainCtrl(1) ) {
                    ; // na zero
                }
                if( fActionTime >= 0.0 ) {
                    fActionTime = -5.0; // niech trochę potrzyma
                }
//                mvControlling->AutoRelayCheck(); // sprawdzenie logiki sterowania
            }
            else {
                // jak ma jechać
                if( fActionTime < 0.0 ) { break; }
                if( false == Ready )    { break; }

                if( mvOccupied->DirActive > 0 ) {
                    mvOccupied->DirectionForward(); //żeby EN57 jechały na drugiej nastawie
                }

                if( mvControlling->MainCtrlPos > 0
                 && false == mvControlling->StLinFlag ) {
                    // jak niby jedzie, ale ma rozłączone liniowe to na zero i czekać na przewalenie kułakowego
                    mvControlling->DecMainCtrl( 2 );
                }
                else {
                    // ruch nastawnika uzależniony jest od aktualnie ustawionej pozycji
                    switch( mvControlling->MainCtrlPos ) {
                        case 0:
                            if( mvControlling->MainCtrlActualPos ) {
                                // jeśli kułakowy nie jest wyzerowany to czekać na wyzerowanie
                                break;
                            }
                            mvControlling->IncMainCtrl( 1 ); // przetok; bez "break", bo nie ma czekania na 1. pozycji
                        case 1:
                            if( VelDesired >= 20 )
                                mvControlling->IncMainCtrl( 1 ); // szeregowa
                        case 2:
                            if( VelDesired >= 50 )
                                mvControlling->IncMainCtrl( 1 ); // równoległa
                        case 3:
                            if( VelDesired >= 80 )
                                mvControlling->IncMainCtrl( 1 ); // bocznik 1
                        case 4:
                            if( VelDesired >= 90 )
                                mvControlling->IncMainCtrl( 1 ); // bocznik 2
                        case 5:
                            if( VelDesired >= 100 )
                                mvControlling->IncMainCtrl( 1 ); // bocznik 3
                    }
                }
                if( mvControlling->MainCtrlPos ) // jak załączył pozycję
                {
                    fActionTime = -5.0; // niech trochę potrzyma
//                    mvControlling->AutoRelayCheck(); // sprawdzenie logiki sterowania
                }
            }
        }
        break;
    }
    case TEngineType::ElectricSeriesMotor: {
/*
        if (Ready || (drivingFlags.press)) { // o ile może jechać
//            if (fAccGravity < -0.10) // i jedzie pod górę większą niż 10 promil
            { // procedura wjeżdżania na ekstremalne wzniesienia
                if( std::fabs( mvControlling->Im ) > 0.85 * mvControlling->Imax ) { // a prąd jest większy niż 85% nadmiarowego
                    if( mvControlling->Imax * mvControlling->EngineVoltage / ( fMass * fAccGravity ) < -2.8 ) // a na niskim się za szybko nie pojedzie
                    { // włączenie wysokiego rozruchu;
                        // (I*U)[A*V=W=kg*m*m/sss]/(m[kg]*a[m/ss])=v[m/s]; 2.8m/ss=10km/h
                        if( mvControlling->RList[ mvControlling->MainCtrlPos ].Bn > 1 ) { // jeśli jedzie na równoległym, to zbijamy do szeregowego, aby włączyć wysoki rozruch
                            if( mvControlling->ScndCtrlPos > 0 ) { // jeżeli jest bocznik
                                mvControlling->DecScndCtrl( 2 ); // wyłączyć bocznik, bo może blokować skręcenie NJ
                            }
                            do { // skręcanie do bezoporowej na szeregowym
                                mvControlling->DecMainCtrl( 1 ); // kręcimy nastawnik jazdy o 1 wstecz
                            }
                            while( ( false == mvControlling->IsMainCtrlNoPowerPos() )
                                && ( mvControlling->RList[ mvControlling->MainCtrlPos ].Bn > 1 ) ); // oporowa zapętla
                        }
                        if( mvControlling->Imax < mvControlling->ImaxHi ) { // jeśli da się na wysokim
                            mvControlling->CurrentSwitch( true ); // rozruch wysoki (za to może się ślizgać)
                        }
                        if( ReactionTime > 0.1 ) {
                            ReactionTime = 0.1; // orientuj się szybciej
                        }
                    } // if (Im>Imin)
                }
                if( std::fabs( mvControlling->Im ) > 0.96 * mvControlling->Imax ) {// jeśli prąd jest duży (można 690 na 750)
                    if( mvControlling->ScndCtrlPos > 0 ) { // jeżeli jest bocznik
                        mvControlling->DecScndCtrl( 2 ); // zmniejszyć bocznik
                    }
                    else {
                        mvControlling->DecMainCtrl( 1 ); // kręcimy nastawnik jazdy o 1 wstecz
                    }
                }
            }
//            else // gdy nie jedzie ambitnie pod górę
            { // sprawdzenie, czy rozruch wysoki jest potrzebny
                if( ( mvControlling->Imax > mvControlling->ImaxLo )
                 && ( mvOccupied->Vel >= 30.0 ) ) { // jak się rozpędził
//                        if (fAccGravity > -0.02) // a i pochylenie mnijsze niż 2‰
                    mvControlling->CurrentSwitch( false ); // rozruch wysoki wyłącz
                }
            }
        }
*/
        break;
    }
    case TEngineType::Dumb: {
        break;
    }
    case TEngineType::DieselElectric: {
        break;
    }
    case TEngineType::ElectricInductionMotor: {
        break;
    }
    case TEngineType::DieselEngine: {
        // Ra 2014-06: "automatyczna" skrzynia biegów...
        auto const &motorparams { mvControlling->MotorParam[ mvControlling->ScndCtrlPos ] };
        auto const velocity { ( mvControlling->ShuntMode ? mvControlling->AnPos : 1.0 ) * mvControlling->Vel };
        if( false == motorparams.AutoSwitch ) {
            // gdy biegi ręczne
            // jak prędkość większa niż procent maksymalnej na danym biegu, wrzucić wyższy
            if( velocity > 0.75 * motorparams.mfi ) {
                // ...presuming there is a higher gear
                if( mvControlling->ScndCtrlPos < mvControlling->ScndCtrlPosNo ) {
                    mvControlling->DecMainCtrl( 2 );
                    while( mvControlling->IncScndCtrl(1)
                        && mvControlling->MotorParam[mvControlling->ScndCtrlPos].mIsat == 0.0 ) { // jeśli bieg jałowy to kolejny
                        ;
                    }
                }
            }
            // jak prędkość mniejsza niż minimalna na danym biegu, wrzucić niższy
            else if( velocity < motorparams.fi && mvControlling->ScndCtrlPos > 1 ) {
                // ... but ensure we don't switch all way down to 0
				mvControlling->DecMainCtrl(2);
				while (mvControlling->ScndCtrlPos > 1 // repeat the entry check as we're working in a loop
				       && mvControlling->DecScndCtrl(1) && mvControlling->MotorParam[mvControlling->ScndCtrlPos].mIsat == 0.0)
				{ // jeśli bieg jałowy to kolejny
					;
				}
			}
        }
        break;
    }
    default: {
        break;
    }
    } // enginetype
};

void TController::SpeedCntrl(double DesiredSpeed)
{
    if( false == mvControlling->SpeedCtrl ) { return; }

	if (mvControlling->IsSpeedCtrlSeparate())
	{
		// a switch of its own, the secondary controller left to the field shunting (eu07/maszyna#69)
		if (DesiredSpeed < 0.1) {
			DesiredSpeed = 0;
		}
		mvControlling->SpeedCtrlSwitch(DesiredSpeed > 0);
		mvControlling->RunCommand(VehicleCommand::speedControl, DesiredSpeed, mvControlling->CabActive);
	}
	else if (mvControlling->EngineType == TEngineType::DieselEngine)
	{
		if (DesiredSpeed < 0.1) {
			mvControlling->DecScndCtrl(2);
			DesiredSpeed = 0;
        }
		else if (mvControlling->ScndCtrlPos < 1) {
			mvControlling->IncScndCtrl(1);
		}
		mvControlling->RunCommand(VehicleCommand::speedControl, DesiredSpeed, mvControlling->CabActive);
	}
	else
	if (mvControlling->ScndCtrlPosNo == 1)
	{
		mvControlling->IncScndCtrl(1);
		mvControlling->RunCommand(VehicleCommand::speedControl, DesiredSpeed, mvControlling->CabActive);
	}
	else if (mvControlling->ScndCtrlPosNo > 1 && !mvOccupied->SpeedCtrlTypeTime)
	{
		int DesiredPos = 1 + mvControlling->ScndCtrlPosNo * ((DesiredSpeed - 1.0) / mvControlling->Vmax);
        while( mvControlling->ScndCtrlPos > DesiredPos && true == mvControlling->DecScndCtrl(1) ) { ; } // all work is done in the condition loop
        while( mvControlling->ScndCtrlPos < DesiredPos && true == mvControlling->IncScndCtrl(1) ) { ; } // all work is done in the condition loop
	}

    if( mvControlling->SpeedCtrlUnit.PowerStep > 0 && mvControlling->IsSpeedCtrlEngaged() ) {
		while (mvControlling->SpeedCtrlUnit.DesiredPower < mvControlling->SpeedCtrlUnit.MaxPower)
		{
			mvControlling->SpeedCtrlPowerInc();
		}
	}
};

void TController::SetTimeControllers()
{
    // TBD, TODO: rework this method to use hint system and regardless of driver type
    if( false == AIControllFlag || 0 == mvOccupied->CabActive ) { return; }

	//1. Check the type of Main Brake Handle
    if( BrakeSystem == TBrakeSystem::Pneumatic || ForcePNBrake )
    {
		if (mvOccupied->Handle->Time)
		{
            auto const pressuredifference { mvOccupied->Handle->GetCP() - ( mvOccupied->HighPipePress - BrakeCtrlPosition * 0.25*mvOccupied->DeltaPipePress ) };
			if (BrakeCtrlPosition > 0 && pressuredifference > 0.05)
				mvOccupied->BrakeLevelSet(mvOccupied->Handle->GetPos(bh_FB));
			else if (BrakeCtrlPosition > 0 && pressuredifference < -0.05)
				mvOccupied->BrakeLevelSet(mvOccupied->Handle->GetPos(bh_RP));
			else if (BrakeCtrlPosition == 0)
				mvOccupied->BrakeLevelSet(mvOccupied->Handle->GetPos(bh_RP));
			else if (BrakeCtrlPosition == -1)
				mvOccupied->BrakeLevelSet(mvOccupied->Handle->GetPos(bh_FS));
			else if (BrakeCtrlPosition == -2)
				mvOccupied->BrakeLevelSet(mvOccupied->Handle->GetPos(bh_NP));
		}
        if( mvOccupied->BrakeHandle == TBrakeHandle::FV4a ) {
            mvOccupied->BrakeLevelSet( BrakeCtrlPosition );
        }
        if( mvOccupied->BrakeHandle == TBrakeHandle::MHZ_K8P
         || mvOccupied->BrakeHandle == TBrakeHandle::MHZ_EN57 ) {
            if( BrakeCtrlPosition == 0 ) {
                mvOccupied->BrakeLevelSet( mvOccupied->Handle->GetPos( bh_RP ) );
            }
            else if( BrakeCtrlPosition == -1 ) {
                mvOccupied->BrakeLevelSet( mvOccupied->Handle->GetPos( bh_FS ) );
            }
            else if( BrakeCtrlPosition == -2 ) {
                mvOccupied->BrakeLevelSet( mvOccupied->Handle->GetPos( bh_NP ) );
            }
            else if( BrakeCtrlPosition > 4.5 ) {
                mvOccupied->BrakeLevelSet( 10 );
            }
            else if( BrakeCtrlPosition > 3.70 ) {
                mvOccupied->BrakeLevelSet( 9 );
            }
            else {
                mvOccupied->BrakeLevelSet( round( ( BrakeCtrlPosition * 0.4 - 0.1 ) / 0.15 ) );
            }
		}
	}
	//2. Check the type of Secondary Brake Handle

	//3. Check the type od EIMCtrlType
/*
	if (mvOccupied->EIMCtrlType > 0)
	{
		if (mvOccupied->EIMCtrlType == 1) //traxx
		{
			if (mvOccupied->LocalBrakePosA > 0.95) mvOccupied->MainCtrlPos = 0;
		}
		else if (mvOccupied->EIMCtrlType == 2) //elf
		{
			if (mvOccupied->LocalBrakePosA > 0.95) mvOccupied->MainCtrlPos = 1;
		}
	}
*/
    if( mvOccupied->UniCtrlIntegratedLocalBrakeCtrl
     && mvOccupied->LocalBrakePosA > 0.95 ) {
        while( IncBrakeEIM() ) { ; }
    }

	//4. Check Speed Control System
	if (mvOccupied->EngineType == TEngineType::ElectricInductionMotor && mvOccupied->ScndCtrlPosNo > 1 && mvOccupied->SpeedCtrlTypeTime)
	{
		double SpeedCntrlVel =
			ActualProximityDist > std::max(50.0, fMaxProximityDist) ?
			VelDesired :
			min_speed(VelDesired, VelNext);
		SpeedCntrlVel = 10 * std::floor(SpeedCntrlVel*0.1);
		if (mvOccupied->ScndCtrlPosNo == 4)
		{
			if (mvOccupied->NewSpeed + 0.1 < SpeedCntrlVel)
				mvOccupied->ScndCtrlPos = 3;
			if (mvOccupied->NewSpeed - 0.1 > SpeedCntrlVel)
				mvOccupied->ScndCtrlPos = 1;
		}
	}
	//5. Check Main Controller in Dizels
	//5.1. Digital controller in DMUs with hydro
	if (mvControlling->EngineType == TEngineType::DieselEngine && mvControlling->EIMCtrlType == 3)
	{

        DizelPercentage_Speed = DizelPercentage; //wstepnie procenty
		auto MinVel{ std::min(mvControlling->hydro_TC_LockupSpeed, mvControlling->Vmax / 6) }; //minimal velocity
		//when speed controll unit is active - start with the procedure
		if (mvControlling->SpeedCtrl && mvControlling->ScndCtrlPos > 0) {
			if (mvControlling->ScndCtrlPos > 0 && mvControlling->Vel < 1 + mvControlling->SpeedCtrlUnit.StartVelocity && DizelPercentage > 0)
				DizelPercentage_Speed = 101; //keep last position to start
		}
		else if (VelDesired > MinVel) //more power for faster ride
		{
			auto const Factor{ 10 * mvControlling->Vmax / (mvControlling->Vmax + 3 * mvControlling->Vel) };
			auto DesiredPercentage{ std::clamp(
				VelDesired > mvControlling->Vel ? (VelDesired - mvControlling->Vel) / Factor : 0,
				0.0, 1.0) }; //correction for reaching desired velocity
			if (VelDesired < 0.5 * mvControlling->Vmax //low velocity and reaching desired
				&& VelDesired - mvControlling->Vel < 10) {
				DesiredPercentage = std::min(DesiredPercentage, 0.75);
			}
			DizelPercentage_Speed = std::round(DesiredPercentage * DizelPercentage);
		}
		else //slow acceleration during shunting wth minimal velocity
		{
			DizelPercentage_Speed = std::min(DizelPercentage, mvControlling->Vel < 0.99 * VelDesired ? 1 : 0);
		}

        auto const DizelActualPercentage { int(100.4 * mvControlling->eimic_real) };

        auto const PosInc { mvControlling->MainCtrlPosNo };
        auto PosDec { 0 };
        for( int i = PosInc; i >= 0; --i ) {
            if( mvControlling->UniCtrlList[i].SetCtrlVal <= 0
             && mvControlling->UniCtrlList[i].SpeedDown > 0.01 ) {
                PosDec = i;
                break;
            }
        }

        if( std::abs( DizelPercentage_Speed - DizelActualPercentage ) > 0 ) {

            if( PosDec > 0
             && ( DizelActualPercentage - DizelPercentage_Speed > 50
               || ( DizelPercentage_Speed == 0
                 && DizelActualPercentage > 10 ) ) ) {
                //one position earlier should be fast decreasing
                PosDec -= 1;
            }
            auto const DesiredPos { (
                DizelPercentage_Speed > DizelActualPercentage ?
                    PosInc :
                    PosDec ) };
            while( mvControlling->MainCtrlPos > DesiredPos ) { mvControlling->DecMainCtrl( 1 ); }
            while( mvControlling->MainCtrlPos < DesiredPos ) { mvControlling->IncMainCtrl( 1 ); }
		}
		if (BrakeCtrlPosition < 0.1) //jesli nie hamuje
			mvOccupied->BrakeLevelSet(mvControlling->UniCtrlList[mvControlling->MainCtrlPos].mode); //zeby nie bruzdzilo machanie zespolonym
	}
	else
	//5.2. Analog direct controller
	if (mvControlling->EngineType == TEngineType::DieselEngine && mvControlling->Vmax > 30)
	{
		int MaxPos = mvControlling->MainCtrlPosNo;
		int MinPos = MaxPos;
		for (int i = MaxPos; i > 1 && mvControlling->RList[i].Mn > 0; i--) MinPos = i;
		if (MaxPos > MinPos && mvControlling->MainCtrlPos > 0 && AccDesired > 0)
		{
			double Factor = 5 * mvControlling->Vmax / (mvControlling->Vmax + mvControlling->Vel);
			int DesiredPos = MinPos + (MaxPos - MinPos)*(VelDesired > mvControlling->Vel ? (VelDesired - mvControlling->Vel) / Factor : 0);
			if (DesiredPos > MaxPos) DesiredPos = MaxPos;
			if (DesiredPos < MinPos) DesiredPos = MinPos;
			if (!mvControlling->SlippingWheels)
			{
				while (mvControlling->MainCtrlPos > DesiredPos) mvControlling->DecMainCtrl(1);
				if (mvControlling->Vel>mvControlling->dizel_minVelfullengage)
				  while (mvControlling->MainCtrlPos < DesiredPos) mvControlling->IncMainCtrl(1);
			}
		}
	}
    // 5.5 universal control for diesel electric vehicles
	if (mvControlling->EngineType == TEngineType::DieselElectric && mvControlling->EIMCtrlType == 3 && AccDesired >= 0.0 && mvControlling->StLinFlag)
	{
        // NOTE: partial implementation for speedinc/dec
        // TBD, TODO: implement fully?
		auto const PosInc{mvControlling->MainCtrlPosNo};
		auto PosKeep{0};
		auto PosDec{0};
		for (int i = PosInc; i >= 0; --i)
		{
			if (mvControlling->UniCtrlList[i].SetCtrlVal <= 0)
			{
				if (mvControlling->UniCtrlList[i].SpeedDown == 0.0)
				{
					PosKeep = i;
				}
				if (mvControlling->UniCtrlList[i].SpeedDown > 0.01)
				{
					PosDec = i;
					break;
				}
			}
		}

		auto const DesiredPos{(AccDesired > AbsAccS + 0.05 ? PosInc : AccDesired < AbsAccS - 0.05 ? PosDec : PosKeep)};

		while (mvControlling->MainCtrlPos > DesiredPos && mvControlling->DecMainCtrl(1))
		{
			;
		}
		while (mvControlling->MainCtrlPos < DesiredPos && mvControlling->IncMainCtrl(1))
		{
			;
		}
	}
	//6. UniversalBrakeButtons
	//6.1. Checking flags for Over pressure
	if (std::abs(BrakeCtrlPosition - gbh_FS)<0.5) {
		UniversalBrakeButtons |= TUniversalBrake::ub_HighPressure | TUniversalBrake::ub_Overload;
	}
	else {
		UniversalBrakeButtons &= ~(TUniversalBrake::ub_HighPressure | TUniversalBrake::ub_Overload);
	}
	//6.2. Setting buttons
	for (int i = 0; i < 3; i++) {
		mvOccupied->UniversalBrakeButton(i, UniversalBrakeButtons & mvOccupied->UniversalBrakeButtonFlag[i]);
	}
};

void TController::CheckTimeControllers()
{
    // TODO: rework this method to use hint system and regardless of driver type
    if( false == AIControllFlag || 0 == mvControlling->CabActive ) { return; }

	//1. Check the type of Main Brake Handle
    if( BrakeSystem == TBrakeSystem::ElectroPneumatic && mvOccupied->Handle->TimeEP && !ForcePNBrake )
    {
		mvOccupied->BrakeLevelSet(mvOccupied->Handle->GetPos(bh_EPN));
	}
    if( ( BrakeSystem == TBrakeSystem::Pneumatic || ForcePNBrake ) && mvOccupied->Handle->Time )
    {
		if (BrakeCtrlPosition > 0)
			mvOccupied->BrakeLevelSet(mvOccupied->Handle->GetPos(bh_MB));
		else
			mvOccupied->BrakeLevelSet(mvOccupied->Handle->GetPos(bh_RP));
	}
	//2. Check the type of Secondary Brake Handle

	//3. Check the type od EIMCtrlType
	if (mvOccupied->EIMCtrlType > 0)
	{
		if (mvOccupied->EIMCtrlType == 1) //traxx
		{
			if (mvOccupied->MainCtrlPos > 3) mvOccupied->MainCtrlPos = 5;
			if (mvOccupied->MainCtrlPos < 3) mvOccupied->MainCtrlPos = 1;
		}
		else if (mvOccupied->EIMCtrlType == 2) //elf
		{
			auto const emergencypositions{mvOccupied->EIMCtrlEmergencyPositions()};
			if (mvOccupied->eimic > 0) mvOccupied->MainCtrlPos = 3 + emergencypositions;
			if (mvOccupied->eimic < 0) mvOccupied->MainCtrlPos = 2 + emergencypositions;
		}
	}
	//4. Check Speed Control System
	if (mvOccupied->EngineType == TEngineType::ElectricInductionMotor && mvOccupied->ScndCtrlPosNo > 1 && mvOccupied->SpeedCtrlTypeTime && mvOccupied->ScndCtrlPosNo == 4)
	{
		mvOccupied->ScndCtrlPos = 2;
	}

	//5. Check Main Controller in Dizels
	//5.1. Digital controller in DMUs with hydro
	if (mvControlling->EngineType == TEngineType::DieselEngine && mvControlling->EIMCtrlType == 3)
	{
		int DizelActualPercentage = 100.4 * mvControlling->eimic_real;
		int NeutralPos = mvControlling->MainCtrlPosNo - 1; //przedostatnia powinna wstrzymywać - hipoteza robocza
		for (int i = mvControlling->MainCtrlPosNo; i >= 0; i--)
			if (mvControlling->UniCtrlList[i].SetCtrlVal <= 0 && mvControlling->UniCtrlList[i].SpeedDown < 0.01) //niby zero, ale nie zmniejsza procentów
			{
				NeutralPos = i;
				break;
			}
		if (BrakeCtrlPosition < 0.1) //jesli nie hamuje
		{
			if (DizelActualPercentage >= DizelPercentage_Speed && mvControlling->MainCtrlPos > NeutralPos)
				while (mvControlling->MainCtrlPos > NeutralPos) mvControlling->DecMainCtrl(1);
			if (DizelActualPercentage <= DizelPercentage_Speed && mvControlling->MainCtrlPos < NeutralPos)
				while (mvControlling->MainCtrlPos < NeutralPos) mvControlling->IncMainCtrl(1);
			mvOccupied->BrakeLevelSet(mvControlling->UniCtrlList[mvControlling->MainCtrlPos].mode); //zeby nie bruzdzilo machanie zespolonym
		}
	}
};
