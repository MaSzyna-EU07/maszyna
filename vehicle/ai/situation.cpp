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

// what the driver sees and keeps an eye on: the consist, the braking distance, the pantographs, the lights, the horns

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

double const deltalog = 0.05; // przyrost czasu

void
TController::update_timers( double dt ) {

    ElapsedTime += dt;
    WaitingTime += dt;
    fBrakeTime -= dt; // wpisana wartość jest zmniejszana do 0, gdy ujemna należy zmienić nastawę hamulca
    fBrakeHeldTime += dt;
    if( mvOccupied->fBrakeCtrlPos != mvOccupied->Handle->GetPos( bh_FS ) ) {
        // brake charging timeout starts after charging ends
        BrakeChargingCooldown += dt;
    }
    fStopTime += dt; // zliczanie czasu postoju, nie ruszy dopóki ujemne
    fActionTime += dt; // czas używany przy regulacji prędkości i zamykaniu drzwi
    LastReactionTime += dt;
    if( mvOccupied->Vel < 0.05
     && OrderCurrentGet().HasAny(DriverOrder::shunt | DriverOrder::looseShunt | DriverOrder::obeyTrain | DriverOrder::bank) ) {
        IdleTime += dt;
    }
    else {
        IdleTime = 0.0;
    }
    fWarningDuration -= dt; // horn activation while the value is positive
}

void
TController::update_logs( double const dt ) {
    // log vehicle data
    if( false == writes_physicslog() ) {
        // not counted while it isn't logged (the driver of a road vehicle, or no logs on): were the logs turned on,
        // it would write a line every 0.05 s from then on, not one each update to make up for the time before
        LastUpdatedTime = 0.0;
        return;
    }
    LastUpdatedTime += dt;
    if( LastUpdatedTime > deltalog ) {
        // zapis do pliku DAT
        if( false == PhysicsLogBegun ) {
            begin_physicslog();
        }
        PhysicsLog();
        LastUpdatedTime -= deltalog;
    }
}

void
TController::control_wheelslip() {
    // a start that has slipped once keeps the sand on until the train is under way. with the sand off as soon as the
    // wheels grip again, the next notch slips again, and a heavy train on poor rails slips, grips and snatches over
    // and over, and may never get going (#187). a vehicle without sand starts as before
    auto const velocity { std::abs( mvOccupied->Vel ) };
    if( mvControlling->SlippingWheels
     && velocity < EU07_AI_SANDEDSTARTVELOCITY
     && false == mvControlling->IsMainCtrlNoPowerPos()
     && mvControlling->Sand > 0.0 ) {
        bSandedStart = true;
    }
    else if( velocity >= EU07_AI_SANDEDSTARTVELOCITY
          || ( velocity < EU07_AI_NOMOVEMENT && VelDesired == 0.0 ) ) {
        bSandedStart = false;
    }
    // wheel slip
    if( mvControlling->SlippingWheels ) {
        cue_action( driver_hint::sandingon ); // piasku!
        if( false == bSandedStart ) {
            cue_action( driver_hint::tractiveforcedecrease );
            cue_action( driver_hint::antislip );
        }
        else if( ElapsedTime - fSandedStartDecreaseTime >= 1.0 ) {
            // starting, a step down at a time, and no anti-slip brake: taking the power off at once and braking the
            // locomotive brings a heavy train that has just got moving to a stand again, and the start begins anew
            cue_action( driver_hint::tractiveforcedecrease );
            fSandedStartDecreaseTime = ElapsedTime;
        }
        cue_action( driver_hint::brakingforcedecrease );
        ++iDriverFailCount;
    }
    else if( ( std::fabs( mvControlling->Im ) > 0.75 * mvControlling->ImaxHi )
          || ( bSandedStart && false == mvControlling->IsMainCtrlNoPowerPos() ) ) {
        cue_action( driver_hint::sandingon ); // piaskujemy tory, coby się nie ślizgać
    }
    else {
        // deactivate sandbox if we aren't slipping
        if( mvControlling->SandDose ) {
            cue_action( driver_hint::sandingoff );
        }
    }
}

void
TController::control_pantographs() {

    if( mvPantographUnit->EnginePowerSource.SourceType != TPowerSource::CurrentCollector ) { return; }

    if( false == mvPantographUnit->PantAutoValve
     && mvOccupied->ScndPipePress > 4.3 ) {
        // gdy główna sprężarka bezpiecznie nabije ciśnienie to można przestawić kurek na zasilanie pantografów z głównej pneumatyki
        cue_action( driver_hint::pantographairsourcesetmain );
    }

    // uśrednione napięcie sieci: przy spadku poniżej wartości minimalnej opóźnić rozruch o losowy czas
    fVoltage = 0.5 * (fVoltage + std::max( mvControlling->GetTrainsetHighVoltage(), mvControlling->PantographVoltage ) );
    if( fVoltage < mvControlling->EnginePowerSource.CollectorParameters.MinV && fActionTime >= PrepareTime ) {
        // gdy rozłączenie WS z powodu niskiego napięcia
		// jeśli czas oczekiwania nie został ustawiony, losowy czas oczekiwania przed ponownym załączeniem jazdy
		fActionTime = -2.0 - Random(10);
	}

    // the pantograph setup the .fiz suggests for shunting (AI: ShuntPantstate=, eu07/maszyna#461), 0 when there's none
    // or the vehicle isn't shunting
    auto const shuntpantographsetup {
        OrderCurrentGet().HasAny( DriverOrder::shunt | DriverOrder::looseShunt ) ?
            mvOccupied->AIHintShuntPantstate :
            0 };

    // raise/lower pantographs as needed
    auto const useregularpantographlayout {
        pVehicle->Next(Coupling::control) == nullptr // standalone
     || is_emu() // special case
     || mvControlling->TrainType == train_type::et41 }; // special case

    if( mvOccupied->Vel > EU07_AI_NOMOVEMENT ) {

        if( fOverhead2 >= 0.0 || iOverheadZero ) {
            // jeśli jazda bezprądowa albo z opuszczonym pantografem
            cue_action( driver_hint::mastercontrollersetzerospeed );
        }

        if( fOverhead2 > 0.0 || iOverheadDown ) {
            // jazda z opuszczonymi pantografami
            if( mvPantographUnit->Pantographs[ end::front ].is_active ) {
                cue_action( driver_hint::frontpantographvalveoff );
            }
            if( mvPantographUnit->Pantographs[ end::rear ].is_active ) {
                cue_action( driver_hint::rearpantographvalveoff );
            }
        }
        else {
            // jeśli nie trzeba opuszczać pantografów
            // TODO: check if faction alone reduces frequency enough
            if( fActionTime > 0.0 ) {
                if( shuntpantographsetup != 0 ) {
                    // shunting with the pantograph setup the .fiz suggests for it
                    control_pantographs_setup( shuntpantographsetup );
                }
                else if( mvOccupied->AIHintPantstate == 0 ) {
                    // jazda na tylnym
                    if( iDirection >= 0 && useregularpantographlayout ) {
                        // jak jedzie w kierunku sprzęgu 0
                        if( mvPantographUnit->PantRearVolt == 0.0
                            // filter out cases with single _other_ working pantograph so we don't try to raise something we can't
                         && ( mvPantographUnit->PantographVoltage == 0.0
                           || mvPantographUnit->EnginePowerSource.CollectorParameters.CollectorsNo > 1 ) ) {
                            cue_action( driver_hint::rearpantographvalveon );
                        }
                    }
                    else {
                        // jak jedzie w kierunku sprzęgu 0
                        if( mvPantographUnit->PantFrontVolt == 0.0
                            // filter out cases with single _other_ working pantograph so we don't try to raise something we can't
                         && ( mvPantographUnit->PantographVoltage == 0.0
                           || mvPantographUnit->EnginePowerSource.CollectorParameters.CollectorsNo > 1 ) ) {
                            cue_action( driver_hint::frontpantographvalveon );
                        }
                    }

                    if( mvOccupied->Vel > 5 && mvPantographUnit->EnginePowerSource.CollectorParameters.CollectorsNo > 1 ) {
                        // opuszczenie przedniego po rozpędzeniu się o ile jest więcej niż jeden
						if (iDirection >= 0 && useregularpantographlayout) // jak jedzie w kierunku sprzęgu 0
						{ // poczekać na podniesienie tylnego
							if (mvPantographUnit->PantFrontVolt != 0.0 && mvPantographUnit->PantRearVolt != 0.0)
							{ // czy jest napięcie zasilające na tylnym?
								cue_action(driver_hint::frontpantographvalveoff); // opuszcza od sprzęgu 0
							}
						}
						else
						{ // poczekać na podniesienie przedniego
							if (mvPantographUnit->PantRearVolt != 0.0 && mvPantographUnit->PantFrontVolt != 0.0)
							{ // czy jest napięcie zasilające na przednim?
								cue_action(driver_hint::rearpantographvalveoff); // opuszcza od sprzęgu 1
							}
						}
					}

                }
                else {
                    // use suggested pantograph setup
                    if( mvOccupied->Vel > 5 ) {
                        auto const pantographsetup{ mvOccupied->AIHintPantstate };
                        cue_action(
                            pantographsetup & 1 << 0 ? driver_hint::frontpantographvalveon : driver_hint::frontpantographvalveoff );
                        cue_action(
                            pantographsetup & 1 << 1 ? driver_hint::rearpantographvalveon : driver_hint::rearpantographvalveoff );
                    }
                }
            }
        }
    }
    else if( shuntpantographsetup != 0 ) {
        // shunting with the suggested setup, also while standing, and without raising more pantographs when idling
        if( fOverhead2 <= 0.0 && iOverheadDown == 0 ) {
            control_pantographs_setup( shuntpantographsetup );
        }
    }
    else {
        if( !mvOccupied->AIHintPantUpIfIdle || IdleTime <= 45.0 || std::abs(fStopTime) <= 15.0 || mvPantographUnit->EnginePowerSource.CollectorParameters.CollectorsNo <= 1 )
		{
		}
		else
		{
			// spending a longer at a stop, raise also front pantograph
			if (iDirection >= 0 && useregularpantographlayout)
			{
				// jak jedzie w kierunku sprzęgu 0
				if (mvPantographUnit->PantFrontVolt == 0.0)
				{
					cue_action(driver_hint::frontpantographvalveon, 5); // discard the hint if speed exceeds 5 km/h
				}
			}
			else
			{
				if (mvPantographUnit->PantRearVolt == 0.0)
				{
					cue_action(driver_hint::rearpantographvalveon, 5); // discard the hint if speed exceeds 5 km/h
				}
			}
		}
	}
}

// raises the pantographs of the suggested setup (bit 0: front, bit 1: rear) and lowers the others, these only once
// a pantograph of the setup collects the voltage, so the vehicle doesn't lose its power on the way
void
TController::control_pantographs_setup( int const Setup ) {

    auto const front { ( Setup & 1 << 0 ) != 0 };
    auto const rear { ( Setup & 1 << 1 ) != 0 };
    if( front ) {
        cue_action( driver_hint::frontpantographvalveon );
    }
    if( rear ) {
        cue_action( driver_hint::rearpantographvalveon );
    }
    if( auto const setuplive { ( front && mvPantographUnit->PantFrontVolt != 0.0 ) || ( rear && mvPantographUnit->PantRearVolt != 0.0 ) };
        false == setuplive ) {
        return;
    }
    if( false == front ) {
        cue_action( driver_hint::frontpantographvalveoff );
    }
    if( false == rear ) {
        cue_action( driver_hint::rearpantographvalveoff );
    }
}

void
TController::control_horns( double const Timedelta ) {
    // horn control
    if( fWarningDuration > 0.0 ) {
        // jeśli pozostało coś do wytrąbienia trąbienie trwa nadal
        if( mvOccupied->WarningSignal == 0 ) {
            cue_action( driver_hint::hornon, 1 ); // low tone horn by default
        }
    }
    else {
        if( mvOccupied->WarningSignal != 0 ) {
            cue_action( driver_hint::hornoff ); // a tu się kończy
        }
    }
    if( mvOccupied->Vel > EU07_AI_MOVEMENT ) {
        // jesli jedzie, można odblokować trąbienie, bo się wtedy nie włączy
        drivingFlags.startHornDone = false; // zatrąbi dopiero jak następnym razem stanie
        drivingFlags.startHorn = true; // i trąbić przed następnym ruszeniem
    }

    if( true == drivingFlags.startHornNow
     && true == Ready
     && true == iEngineActive
     && mvControlling->MainCtrlPowerPos() > 0 ) {
        // uruchomienie trąbienia przed ruszeniem
        fWarningDuration = 0.3; // czas trąbienia
        cue_action( driver_hint::hornon, pVehicle->iHornWarning ); // wysokość tonu (2=wysoki)
        drivingFlags.startHornDone = true; // nie trąbić aż do ruszenia
        drivingFlags.startHornNow = false; // trąbienie zostało zorganizowane
    }
}

void
TController::control_security_system( double const Timedelta ) {
    if( mvOccupied->SecuritySystem.is_cabsignal_blinking() && mvOccupied->SecuritySystem.has_separate_acknowledge()) {
        // jak zadziałało SHP
        if( false == is_emu()
         && mvOccupied->DirActive == 0 ) {
            cue_action( driver_hint::directionforward );
        }
        cue_action( driver_hint::shpsystemreset ); // to skasuj
        if( BrakeCtrlPosition == 0 // TODO: verify whether it's 0 in all vehicle types
         && AccDesired > 0.0
         && mvOccupied->SecuritySystem.is_braking() ) {
            cue_action( driver_hint::trainbrakerelease );
        }
    }

    if( mvOccupied->SecuritySystem.is_blinking() ) {
        // jak zadziałało CA/SHP
        if( false == is_emu()
         && mvOccupied->DirActive == 0 ) {
            cue_action( driver_hint::directionforward );
        }
        cue_action( driver_hint::securitysystemreset ); // to skasuj
        if( BrakeCtrlPosition == 0 // TODO: verify whether it's 0 in all vehicle types
         && AccDesired > 0.0
         && mvOccupied->SecuritySystem.is_braking() ) {
            cue_action( driver_hint::trainbrakerelease );
        }
    }
    // basic emergency stop handling, while at it
    if( true == mvOccupied->RadioStopFlag // radio-stop
     && mvOccupied->Vel < 0.01 // and actual stop
     && true == mvOccupied->Radio ) { // and we didn't touch the radio yet
        // turning off the radio should reset the flag, during security system check
        if( m_radiocontroltime > 5.0 ) { // arbitrary delay between stop and disabling the radio
            cue_action( driver_hint::radiooff );
        }
        else {
            m_radiocontroltime += Timedelta;
        }
    }
    if( iEngineActive
     && ( false == mvOccupied->Radio || false == mvOccupied->RadioFuse )
     && false == mvOccupied->RadioStopFlag ) {
        // otherwise if it's safe to do so, turn the radio back on
        // arbitrary 5 sec delay before switching radio back on
        m_radiocontroltime = std::min( m_radiocontroltime, 5.0 );
        if( m_radiocontroltime < 0.0 ) {
            // hint at whichever of the two is open, the radio's switch or the cut-off in its power supply
            if( false == mvOccupied->RadioFuse ) {
                cue_action( driver_hint::radiofuseon );
            }
            if( false == mvOccupied->Radio ) {
                cue_action( driver_hint::radioon );
            }
        }
        else {
            m_radiocontroltime -= Timedelta;
        }
    }
}

void
TController::control_handles() {

    switch( mvControlling->EngineType ) {
    case TEngineType::ElectricSeriesMotor: {
        // styczniki liniowe rozłączone    yBARC
        if( false == mvControlling->StLinFlag
         && false == mvControlling->DelayCtrlFlag
         && mvControlling->MainCtrlPowerPos() > 1 ) {
            cue_action( driver_hint::mastercontrollersetzerospeed );
        }
        // if the power station is heavily burdened drop down to series mode to reduce the load
        auto const useseriesmodevoltage {
            std::lerp(
                mvControlling->EnginePowerSource.CollectorParameters.MinV,
                mvControlling->EnginePowerSource.CollectorParameters.MaxV,
                IsHeavyCargoTrain ? 0.35 : 0.40 ) };

        if( fVoltage <= useseriesmodevoltage ) {
            cue_action( driver_hint::mastercontrollersetseriesmode );
        }
        // sanity check
        if( false == Ready
         && mvControlling->MainCtrlPowerPos() > 1 ) {
            cue_action( driver_hint::mastercontrollersetzerospeed );
        }
        break;
    }
    case TEngineType::DieselElectric: {
        // styczniki liniowe rozłączone    yBARC
        if( false == mvControlling->StLinFlag
         && false == mvControlling->DelayCtrlFlag
         && mvControlling->MainCtrlPowerPos() > 1 ) {
            cue_action( driver_hint::mastercontrollersetzerospeed );
        }
        // sanity check
        if( false == Ready
         && mvControlling->MainCtrlPowerPos() > 1 ) {
            cue_action( driver_hint::mastercontrollersetzerospeed );
        }
        break;
    }
    default: {
        break;
    } } // enginetype
}

// moves the light preset selector to the position matching the lights set directly by the AI,
// otherwise the next SetLights() call (e.g. from CheckVehicles()) reverts them to the untouched selector position
void
TController::sync_lights_selector() {

    if( mvOccupied->LightsPosNo <= 0 ) { return; }

    auto const *headvehicle { pVehicles[ end::front ] };
    if( headvehicle == nullptr ) { return; }
    // same choice of the light table as in TDynamicObject::SetLights()
    auto const lightstable { ( mvOccupied->CabOccupied * pVehicle->DirectionGet() >= 0 ? end::front : end::rear ) };
    // same choice of the lit vehicle end as in TDynamicObject::RaLightsSet()
    auto const headend { ( headvehicle->iDirection > 0 ? end::front : end::rear ) };
    auto const position { handover::light_selector_position(
        *mvOccupied, lightstable, headvehicle->MoverParameters->iLights[ headend ], headvehicle->LightList( headend ) ) };
    if( position == mvOccupied->LightsPos ) { return; }

    mvOccupied->LightsPos = position;
    // visual feedback
    auto *train { simulation::Trains.find( pVehicle->name() ) };
    if( train != nullptr ) {
        train->ggLightsButton.PutValue( static_cast<float>( position - 1 ) );
    }
}

// re-applies the headlight dimmer state after the AI handed the vehicle over to the human driver,
// moving the dimmer out of the off position if it'd hide the head lights set by the AI
void
TController::sync_lights_dimmer() {

    if( mvOccupied->dimPositions.empty() ) { return; }

    auto const *headvehicle { pVehicles[ end::front ] };
    auto const headend { ( headvehicle != nullptr && headvehicle->iDirection > 0 ? end::front : end::rear ) };
    auto const headlightson {
        headvehicle != nullptr
     && ( headvehicle->MoverParameters->iLights[ headend ] & ( light::headlight_left | light::headlight_right | light::headlight_upper ) ) != 0 };

    auto const position { handover::dimmer_position( *mvOccupied, headlightson ) };
    if( position != mvOccupied->modernDimmerPosition ) {
        mvOccupied->modernDimmerPosition = position;
        // visual feedback
        auto *train { simulation::Trains.find( pVehicle->name() ) };
        if( train != nullptr ) {
            train->ggModernLightDimSw.PutValue( static_cast<float>( position ) );
        }
    }
    if( position < 0 || position >= static_cast<int>( mvOccupied->dimPositions.size() ) ) { return; } // SetLightDimmings() doesn't check the range
    pVehicle->SetLightDimmings();
}

// activate consist lamp codes
void
TController::control_lights() {

    if( is_train() ) {
        // jeśli jazda pociągowa
        if( true == OrderCurrentGet().HasAll(DriverOrder::obeyTrain) ) {
            // head lights
            if( m_lighthints[ end::front ] == -1 ) {
                cue_action( driver_hint::headcodepc1 );
            }
            else if( m_lighthints[ end::front ] == ( light::redmarker_left | light::headlight_right | light::headlight_upper ) ) {
                cue_action( driver_hint::headcodepc2 );
            }
            else {
                // custom light pattern
                if( AIControllFlag ) {
                    pVehicles[ end::front ]->RaLightsSet( m_lighthints[ end::front ], -1 );
                }
            }
            // tail lights
            if( m_lighthints[ end::rear ] == -1 ) {
                cue_action( driver_hint::headcodepc5 );
            }
            else {
                if( AIControllFlag ) {
                    pVehicles[ end::rear ]->RaLightsSet( -1, m_lighthints[ end::rear ] );
                }
            }
        }
        // for shunting mode
        else if( OrderCurrentGet() & ( DriverOrder::shunt | DriverOrder::looseShunt | DriverOrder::connect ) ) {
            cue_action( driver_hint::headcodetb1 );
        }
    }
    else if( is_car() ) {
        Lights(
            light::headlight_left | light::headlight_right,
            light::redmarker_left | light::redmarker_right );
    }
}

void
TController::control_compartment_lights() {
    // compartment lights
    if( mvOccupied->CompartmentLights.start_type != start_t::manual ) { return; }

    auto const currentlightstate { mvOccupied->CompartmentLights.is_enabled };
    auto const lightlevel { Global.fLuminance * ConsistShade };
    auto const desiredlightstate{ (
        currentlightstate ?
            lightlevel < 0.40 : // turn off if lighting level goes above 0.4
            lightlevel < 0.35 ) }; // turn on if lighting level goes below 0.35
    if( desiredlightstate != currentlightstate ) {
        cue_action( desiredlightstate ? driver_hint::consistlightson : driver_hint::consistlightsoff );
    }
}

void
TController::UpdateCommand() {

    if( mvOccupied->CommandIn.Command != VehicleCommand::none && false == mvOccupied->RunInternalCommand() ) {
		// rozpoznaj komende bo lokomotywa jej nie rozpoznaje
		RecognizeCommand(); // samo czyta komendę wstawioną do pojazdu?
	}
}

void
TController::UpdateNextStop() {

    if( fLastStopExpDist > 0.0
     && mvOccupied->DistCounter > fLastStopExpDist ) {
        // zaktualizować wyświetlanie rozkładu
        TrainParams.StationStart = TrainParams.StationIndex;
        fLastStopExpDist = -1.0; // usunąć licznik
        if( true == m_makenextstopannouncement ) {
            announce( announcement_t::next );
            m_makenextstopannouncement = false; // keep next stop announcements suppressed until another scheduled stop
        }
    }
}

void
TController::determine_braking_distance() {
    // przybliżona droga hamowania
    // NOTE: we're ensuring some minimal braking distance to reduce ai flipping states between starting and braking
    auto const velceil { std::max( 2.0, std::ceil( mvOccupied->Vel ) ) };
    fBrakeDist = fDriverBraking * velceil * ( 40.0 + velceil );
    if( fMass > 1000000.0 ) {
        // korekta dla ciężkich, bo przeżynają - da to coś?
        fBrakeDist *= 2.0;
    }
    if( -fAccThreshold > 0.05
     && mvOccupied->CategoryFlag == 1 ) {
        fBrakeDist = velceil *	velceil / 25.92 / -fAccThreshold;
    }
    if( mvOccupied->BrakeDelayFlag == bdelay_G ) {
        // dla nastawienia G koniecznie należy wydłużyć drogę na czas reakcji
        fBrakeDist += 2 * velceil;
    }
    if( mvOccupied->Vel > 15.0
     && mvControlling->EngineType == TEngineType::ElectricInductionMotor
     && is_emu() ) {
        // HACK: make the induction motor powered EMUs start braking slightly earlier
        fBrakeDist += 10.0;
    }
/*
    // take into account effect of gravity (but to stay on safe side of calculations, only downhill)
    if( fAccGravity > 0.025 ) {
        fBrakeDist *= ( 1.0 + fAccGravity );
        // TBD: use version which shortens route going uphill, too
        //fBrakeDist = std::max( fBrakeDist, fBrakeDist * ( 1.0 + fAccGravity ) );
    }
*/
}

void
TController::scan_route( double const Range ) {

    TableCheck( Range );
}

void
TController::determine_proximity_ranges() {
     // ustalenie dystansów w pozostałych przypadkach
    switch (SingleOrder(OrderCurrentGet()))
    {
    case DriverOrder::connect: {
        // podłączanie do składu
        if (drivingFlags.connect) {
            // jeśli stanął już blisko, unikając zderzenia i można próbować podłączyć
            fMinProximityDist = -1.0;
            fMaxProximityDist =  0.0; //[m] dojechać maksymalnie
            fVelPlus = 1.0; // dopuszczalne przekroczenie prędkości na ograniczeniu bez hamowania
            fVelMinus = 0.5; // margines prędkości powodujący załączenie napędu
        }
        else {
            // jak daleko, to jazda jak dla Shunt na kolizję
            fMinProximityDist = 2.0;
            fMaxProximityDist = 5.0; //[m] w takim przedziale odległości powinien stanąć
            fVelPlus = 2.0; // dopuszczalne przekroczenie prędkości na ograniczeniu bez hamowania
            fVelMinus = 1.0; // margines prędkości powodujący załączenie napędu
        }
        break;
    }
    case DriverOrder::disconnect: {
        // 20.07.03 - manewrowanie wagonami
        fMinProximityDist = 1.0;
        fMaxProximityDist = 10.0; //[m]
        fVelPlus = 1.0; // dopuszczalne przekroczenie prędkości na ograniczeniu bez hamowania
        fVelMinus = 0.5; // margines prędkości powodujący załączenie napędu
        break;
    }
    case DriverOrder::shunt: {
        // na jaką odleglość i z jaką predkością ma podjechać
        // TODO: test if we can use the distances calculation from obey_train
        fMinProximityDist = std::min( 5 + iVehicles, 25 );
        fMaxProximityDist = std::min( 10 + iVehicles, 50 );
        // HACK: modern vehicles might brake slower at low speeds, increase safety margin as crude counter
        if( mvControlling->EIMCtrlType > 0 ) {
            fMinProximityDist += 5.0;
            fMaxProximityDist += 5.0;
        }
        // take into account weather conditions
        if (Global.FrictionWeatherFactor < 1.f
         && iVehicles > 1 ) {
            fMinProximityDist += 5.0;
            fMaxProximityDist += 5.0;
        }
        fVelPlus = 2.0; // dopuszczalne przekroczenie prędkości na ograniczeniu bez hamowania
        // margines prędkości powodujący załączenie napędu
        // były problemy z jazdą np. 3km/h podczas ładowania wagonów
        fVelMinus = std::min( 0.1 * fShuntVelocity, 3.0 );
        break;
    }
    case DriverOrder::looseShunt: {
        fMinProximityDist = -1.0;
        fMaxProximityDist = 0.0; //[m] dojechać maksymalnie
        fVelPlus = 2.0; // dopuszczalne przekroczenie prędkości na ograniczeniu bez hamowania
        fVelMinus = 0.5; // margines prędkości powodujący załączenie napędu
        break;
    }
    case DriverOrder::obeyTrain: {
        // na jaka odleglosc i z jaka predkoscia ma podjechac do przeszkody
        // jeśli pociąg
        if( is_train() ) {
            auto const [ minproximity, maxproximity ] { train_proximity_margins( IsAtPassengerStop ) };
            fMinProximityDist = minproximity;
            fMaxProximityDist = maxproximity;

            if( drivingFlags.late ) {
                // jeśli spóźniony, to gna
                fVelMinus = 1.0;
                // dopuszczalne przekroczenie prędkości na ograniczeniu bez hamowania
                fVelPlus = 5.0;
            }
            else {
                // gdy nie musi się sprężać
                // margines prędkości powodujący załączenie napędu; min 1.0 żeby nie ruszał przy 0.1
                fVelMinus = std::clamp( std::round( 0.05 * VelDesired ), 1.0, 5.0 );
                // normalnie dopuszczalne przekroczenie to 5% prędkości ale nie więcej niż 5km/h
                // bottom margin raised to 2 km/h to give the AI more leeway at low speed limits
                fVelPlus = std::clamp( std::ceil( 0.05 * VelDesired ), 2.0, 5.0 );
            }
        }
        // samochod (sokista też)
        else {
            fMinProximityDist = std::max( 3.5, mvOccupied->Vel * 0.2   );
            fMaxProximityDist = std::max( 9.5, mvOccupied->Vel * 0.375 ); //[m]
            if( Global.FrictionWeatherFactor < 1.f ) {
                // take into account weather conditions
                fMinProximityDist += 0.75;
                fMaxProximityDist += 0.75;
            }
            // margines prędkości powodujący załączenie napędu
            fVelMinus = 2.0;
            // dopuszczalne przekroczenie prędkości na ograniczeniu bez hamowania
            fVelPlus = std::min( 10.0, mvOccupied->Vel * 0.1 );
        }
        break;
    }
    case DriverOrder::bank: {
        // TODO: implement
        break;
    }
    default: {
        fMinProximityDist = 5.0;
        fMaxProximityDist = 10.0; //[m]
        fVelPlus = 2.0; // dopuszczalne przekroczenie prędkości na ograniczeniu bez hamowania
        fVelMinus = 5.0; // margines prędkości powodujący załączenie napędu
    }
    } // switch OrderList[OrderPos]
}
