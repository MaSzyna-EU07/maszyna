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

// preparing the vehicle and shutting it down: the engine, the heating, the direction

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


// closes the line breaker, or starts the engine, the relays reset first
void TController::close_line_breaker()
{
    cue_action( driver_hint::mastercontrollersetzerospeed );
    // consist-wide ground relay reset
    cue_action( driver_hint::maincircuitgroundreset );
    cue_action( driver_hint::tractionnmotoroverloadreset );
    if( mvOccupied->EngineType == TEngineType::DieselEngine ) {
        // specjalnie dla SN61 żeby nie zgasł
        cue_action( driver_hint::mastercontrollersetidle );
    }
    cue_action( driver_hint::linebreakerclose );
}

bool TController::PrepareEngine()
{ // odpalanie silnika
    // HACK: don't immediately activate inert vehicle in case the simulation is about to replace us with human driver
    if( mvOccupied->Vel < 1.0 && fActionTime < 0.0 ) { return false; }

    LastReactionTime = 0.0;
    ReactionTime = mvOccupied->Vel < 5 ? PrepareTime : EasyReactionTime; // react faster with rolling start

    cue_action( driver_hint::batteryon );
	cue_action( driver_hint::cabactivation);
    cue_action( driver_hint::radiofuseon );
    cue_action( driver_hint::radioon );

    if( has_diesel_engine() ) {
        cue_action( driver_hint::oilpumpon );
        PrepareHeating();
        if( true == IsHeatingTemperatureOK ) {
            cue_action( driver_hint::fuelpumpon );
        }
        else {
            cue_action( driver_hint::waittemperaturetoolow );
        }
    }

    if( mvControlling->PantographsRequireMains
     && IsAnyLineBreakerOpen ) {
        // the pantographs rise only through the closed line breaker, which closes without their voltage
        PrepareDirection();
        close_line_breaker();
    }

    auto const sufficientpantographpressure { (
        is_emu() ?
            2.5 : // Ra 2013-12: Niebugocław mówi, że w EZT podnoszą się przy 2.5
            3.5 )
        + 0.1 }; // leeway margin

    if( mvPantographUnit->EnginePowerSource.SourceType == TPowerSource::CurrentCollector
     && mvPantographUnit->EnginePowerSource.CollectorParameters.CollectorsNo > 0 ) {
        // if our pantograph unit isn't a pantograph-devoid fallback
        if (mvPantographUnit->PantPress < sufficientpantographpressure) {
            // załączenie małej sprężarki
            if( false == mvPantographUnit->PantAutoValve ) {
                // odłączenie zbiornika głównego, bo z nim nie da rady napompować
                cue_action( driver_hint::pantographairsourcesetauxiliary );
            }
            cue_action( driver_hint::pantographcompressoron ); // załączenie sprężarki pantografów
            if( mvPantographUnit->PantCompFlag ) {
                cue_action( driver_hint::waitpantographpressuretoolow );
            }
        }
        else {
            // jeżeli jest wystarczające ciśnienie w pantografach
            if( false == mvPantographUnit->bPantKurek3
             || mvPantographUnit->PantPress <= mvPantographUnit->ScndPipePress ) { // kurek przełączony albo główna już pompuje
                cue_action( driver_hint::pantographcompressoroff ); // sprężarkę pantografów można już wyłączyć
            }
        }
        // TODO: make pantograph setup a part of control_pantographs() and call it here instead
        if( fOverhead2 == -1.0 && iOverheadDown == 0 ) {
            cue_action( driver_hint::pantographsvalveon );
            cue_action( driver_hint::frontpantographvalveon, iDirection >= 0 ? 5 : 0 );
            cue_action( driver_hint::rearpantographvalveon, iDirection >= 0 ? 0 : 5 );
        }
    }

    auto const ispoweravailable =
        mvControlling->EnginePowerSource.SourceType != TPowerSource::CurrentCollector
     || std::max(mvControlling->GetTrainsetHighVoltage(), mvControlling->PantographVoltage) > mvControlling->EnginePowerSource.CollectorParameters.MinV;
    // standing with the pantographs up and no voltage, nothing the driver does starts the vehicle. it says so, and keeps
    // trying: towed back under the wire, or with the power switched on, the vehicle gets prepared after all
    report_power_stall( ispoweravailable, sufficientpantographpressure );

    bool isready = false;

    if( IsHeatingTemperatureOK
     && ispoweravailable ) {
        // najpierw ustalamy kierunek, jeśli nie został ustalony
        PrepareDirection();

        if( IsAnyConverterOverloadRelayOpen ) {
            // wywalił bezpiecznik nadmiarowy przetwornicy
            cue_action( driver_hint::compressoroff ); // TODO: discern whether compressor needs converter to operate
            cue_action( driver_hint::converteroff );
            cue_action( driver_hint::primaryconverteroverloadreset ); // reset nadmiarowego
        }
        if( IsAnyLineBreakerOpen ) {
            // activate main circuit or engine
            close_line_breaker();
        }
        else {
            // main circuit or engine is on, set up vehicle devices and controls
            if( false == IsAnyConverterOverloadRelayOpen ) {
                if( IsAnyConverterPresent ) {
                    cue_action( driver_hint::converteron );
                }
                // w EN57 sprężarka w ra jest zasilana z silnikowego
                // TODO: change condition to presence of required voltage type
                if( IsAnyCompressorPresent && IsAnyConverterEnabled ) {
                    cue_action( driver_hint::compressoron );
                }
                if( mvControlling->ScndPipePress < 4.5 && mvControlling->VeselVolume > 0.0 ) {
                    cue_action( driver_hint::waitpressuretoolow );
                }
                // enable motor blowers
                if( mvOccupied->MotorBlowers[ end::front ].speed != 0 ) {
                    cue_action( driver_hint::frontmotorblowerson );
                }
                if( mvOccupied->MotorBlowers[ end::rear ].speed != 0 ) {
                    cue_action( driver_hint::rearmotorblowerson );
                }
            }
            control_field_shunting();
            // sync virtual brake state with the 'real' one
            if( mvOccupied->BrakeHandle != TBrakeHandle::NoHandle ) {
                std::unordered_map<int, int> const brakepositions{
                    { static_cast<int>( mvOccupied->Handle->GetPos( bh_RP ) ), gbh_RP },
                    { static_cast<int>( mvOccupied->Handle->GetPos( bh_NP ) ), gbh_NP },
                    { static_cast<int>( mvOccupied->Handle->GetPos( bh_FS ) ), gbh_FS } };
                auto const lookup{ brakepositions.find( static_cast<int>( mvOccupied->fBrakeCtrlPos ) ) };
                if( lookup != brakepositions.end() ) {
                    BrakeLevelSet( lookup->second ); // GBH
                }
            }
            // set up train brake
            if( mvOccupied->Vel < EU07_AI_NOMOVEMENT ) {
                // standing: the locomotive brake first, then the pipe is filled
                secure_standing_train();
            }
            else if( ( mvOccupied->fBrakeCtrlPos != mvOccupied->Handle->GetPos( bh_RP ) )
                  && ( ( false == mvOccupied->Handle->Time )
                    || ( mvOccupied->Handle->GetCP() < mvOccupied->HighPipePress - 0.05 ) ) ) {
                cue_action( driver_hint::trainbrakerelease );
            }
            // sync spring brake state across consist
            cue_action(
                mvOccupied->SpringBrake.Activate ?
                    driver_hint::springbrakeon :
                    driver_hint::springbrakeoff );
        }
        isready = false == IsAnyConverterOverloadRelayOpen
               && mvOccupied->DirActive != 0
               && false == IsAnyLineBreakerOpen
               && ( false == IsAnyConverterPresent || true == IsAnyConverterEnabled )
               && ( false == IsAnyCompressorPresent || true == IsAnyCompressorEnabled )
               && ( mvControlling->ScndPipePress > 4.5 || mvControlling->VeselVolume == 0.0 );
    }

    if( true == isready ) {
        // jeśli dotychczas spał teraz nie ma powodu do stania
        eAction = TAction::actUnknown;
        if( eStopReason == stopSleep ) {
            eStopReason = stopNone;
        }
        // może skanować sygnały i reagować na komendy
        drivingFlags.active = true;
    }

    iEngineActive = isready;

    return iEngineActive;
}

// tells in the log, once, of standing with the pantographs up and no voltage at them, past the end of the overhead
// wire or under a wire switched off (eu07/maszyna#421). Poweravailable is what PrepareEngine() found, Pantographpressure
// the air the pantographs need to rise
void
TController::report_power_stall( bool const Poweravailable, double const Pantographpressure ) {

    ai_power::state state;
    state.collector = mvControlling->EnginePowerSource.SourceType == TPowerSource::CurrentCollector;
    state.power = Poweravailable;
    state.standing = mvOccupied->Vel < EU07_AI_NOMOVEMENT;
    if( mvPantographUnit->PantPress >= Pantographpressure ) {
        state.pantographs_up =
            mvPantographUnit->Pantographs[ end::front ].is_active
         || mvPantographUnit->Pantographs[ end::rear ].is_active;
    }
    // the pantographs of the vehicle carrying them hold the wire they found above them
    if( auto const *carrier { pVehicle->FindPantographCarrier() };
        carrier != nullptr && carrier->MoverParameters == mvPantographUnit ) {
        auto const pantographcount { std::min<int>( carrier->iAnimType[ ANIM_PANTS ], static_cast<int>( carrier->MoverParameters->Pantographs.size() ) ) };
        for( auto index { 0 }; index < pantographcount; ++index ) {
            if( true == carrier->MoverParameters->Pantographs[ index ].is_active
             && carrier->pants[ index ].fParamPants->hvPowerWire != nullptr ) {
                state.wire = true;
            }
        }
    }
    // a user driving the vehicle sees the voltmeters for themselves
    if( auto const stall { m_powerstall.update( AIControllFlag ? ai_power::check( state ) : ai_power::stall::waiting, ElapsedTime ) };
        stall.has_value() ) {
        ErrorLog( ai_power::message( *stall, pVehicle->asName, pVehicle->GetPosition() ), logtype::traction );
    }
}

// wyłączanie silnika (test wyłączenia, a część wykonawcza tylko jeśli steruje komputer)
bool TController::ReleaseEngine() {
/*
    if( mvOccupied->Vel > 0.01 ) {
        VelDesired = 0.0;
        AccDesired = std::min( AccDesired, -1.25 ); // hamuj solidnie
        if( true == AIControllFlag ) {
            // TBD, TODO: make a dedicated braking procedure out of it for potential reuse
            ZeroSpeed();
            IncBrake();
            ReactionTime = 0.1;
        }
        // don't bother with the rest until we're standing still
        return false;
    }
*/
    // don't bother with the rest until we're standing still
    if( mvOccupied->Vel > EU07_AI_NOMOVEMENT ) { return false; }

    LastReactionTime = 0.0;
    ReactionTime = PrepareTime;

    cue_action( driver_hint::releaseroff );
    // release train brake if on flats...
    if( std::abs( fAccGravity ) < 0.01 ) {
        apply_independent_brake_only();
    }
    // ... but on slopes engage train brake
    else {
//        AccDesired = std::min( AccDesired, -0.9 );
        cue_action( driver_hint::trainbrakeapply );
    }

    cue_action( driver_hint::mastercontrollersetreverserunlock );
    cue_action( driver_hint::directionnone );
    // zamykanie drzwi
    cue_action( driver_hint::doorrightclose );
    cue_action( driver_hint::doorleftclose );
    // heating
    cue_action( driver_hint::consistheatingoff );
    // devices
    cue_action( driver_hint::compressoroff );
    cue_action( driver_hint::converteroff );
    // line breaker/engine
    cue_action( driver_hint::linebreakeropen );
    // pantographs
    cue_action( driver_hint::frontpantographvalveoff );
    cue_action( driver_hint::rearpantographvalveoff );

    if( false == mvControlling->Mains ) {
        // finish vehicle shutdown
        if( has_diesel_engine() ) {
            // heating/cooling subsystem
            cue_action( driver_hint::waterheateroff );
            // optionally turn off the water pump as well
            if( mvControlling->WaterPump.start_type != start_t::battery ) {
                cue_action( driver_hint::waterpumpoff );
            }
            // fuel and oil subsystems
            cue_action( driver_hint::fuelpumpoff );
            cue_action( driver_hint::oilpumpoff );
        }
        // gasimy światła
        cue_action( driver_hint::lightsoff );
        // activate parking brake
        // TBD: do it earlier?
        cue_action( driver_hint::springbrakeon );
        if( mvOccupied->LocalBrake == TLocalBrake::ManualBrake
         || mvOccupied->MBrake == true ) {
            cue_action( driver_hint::manualbrakon );
        }
        // switch off remaining power
        cue_action( driver_hint::radiooff );
        cue_action( driver_hint::batteryoff );
    }

    auto const OK {
               mvOccupied->DirActive == 0
//            && ( false == IsAnyCompressorEnabled )
//            && ( false == IsAnyConverterEnabled )
            && false == mvControlling->Mains
            && false == mvOccupied->Power24vIsAvailable };

    if (OK) {
        // jeśli się zatrzymał
        iEngineActive = false;
        eStopReason = stopSleep; // stoimy z powodu wyłączenia
        eAction = TAction::actSleep; //śpi (wygaszony)

        OrderNext(DriverOrder::waitForOrders); //żeby nie próbował coś robić dalej
        drivingFlags.active = false; // ma nie skanować sygnałów i nie reagować na komendy
        TableClear(); // zapominamy ograniczenia
        VelSignalLast = -1.0;
    }
    return OK;
}

// configures vehicle heating given current situation; returns: true if vehicle can be operated normally, false otherwise
bool
TController::PrepareHeating() {

    switch( mvControlling->EngineType ) {

        case TEngineType::DieselElectric:
        case TEngineType::DieselEngine: {

            auto const &heat { mvControlling->dizel_heat };

            // determine whether there's need to enable the water heater
            // if the heater has configured maximum temperature, it'll disable itself automatically, so we can leave it always running
            // otherwise enable the heater only to maintain minimum required temperature
            auto const lowtemperature { (
                ( heat.water.config.temp_min > 0 && heat.temperatura1 < heat.water.config.temp_min + (mvControlling->WaterHeater.is_active ? 5 : 0) )
             || ( heat.water_aux.config.temp_min > 0 && heat.temperatura2 < heat.water_aux.config.temp_min + (mvControlling->WaterHeater.is_active ? 5 : 0) )
             || ( heat.oil.config.temp_min > 0 && heat.To < heat.oil.config.temp_min + (mvControlling->WaterHeater.is_active ? 5 : 0) ) ) };
            auto const heateron { (
                mvControlling->WaterHeater.config.temp_max > 0
             || true == lowtemperature ) };
            if( true == heateron ) {
                // make sure the water pump is running before enabling the heater
                if( false == mvControlling->WaterPump.is_active ) {
                    cue_action( driver_hint::waterpumpbreakeron );
                    cue_action( driver_hint::waterpumpon );
                }
                if( true == mvControlling->WaterPump.is_active ) {
                    cue_action( driver_hint::waterheaterbreakeron );
                    cue_action( driver_hint::waterheateron );
                    cue_action( driver_hint::watercircuitslinkon );
                }
            }
            else {
                // no need to heat anything up, switch the heater off
                cue_action( driver_hint::watercircuitslinkoff );
                cue_action( driver_hint::waterheateroff );
                cue_action( driver_hint::waterheaterbreakeroff );
                // optionally turn off the water pump as well
                if( mvControlling->WaterPump.start_type != start_t::battery ) {
                    cue_action( driver_hint::waterpumpoff );
                    cue_action( driver_hint::waterpumpbreakeroff );
                }
            }

            IsHeatingTemperatureTooLow = lowtemperature;
            break;
        }
        default: {
            IsHeatingTemperatureTooLow = false;
            break;
        }
    }
    // TBD, TODO: potentially pay attention to too high temperature as well?
    IsHeatingTemperatureOK = false == IsHeatingTemperatureTooLow;

    return IsHeatingTemperatureOK;
}

void
TController::PrepareDirection() {

    if( iDirection == 0 ) {
        // jeśli nie ma ustalonego kierunku
        if( mvOccupied->Vel < EU07_AI_NOMOVEMENT ) { // ustalenie kierunku, gdy stoi
            iDirection = mvOccupied->CabActive; // wg wybranej kabiny
/*
            if( iDirection == 0 ) {
                // jeśli nie ma ustalonego kierunku
                if( mvOccupied->Couplers[ end::rear ].Connected == nullptr ) {
                    // jeśli z tyłu nie ma nic
                    iDirection = -1; // jazda w kierunku sprzęgu 1
                }
                if( mvOccupied->Couplers[ end::front ].Connected == nullptr ) {
                    // jeśli z przodu nie ma nic
                    iDirection = 1; // jazda w kierunku sprzęgu 0
                }
            }
*/
        }
        else {
            // ustalenie kierunku, gdy jedzie
            iDirection = mvOccupied->V >= 0 ? 1 : // jazda w kierunku sprzęgu 0
			                                  -1; // jazda w kierunku sprzęgu 1
        }
    }
    cue_action( driver_hint::mastercontrollersetreverserunlock );
    cue_action(
        iDirection > 0 ?
            driver_hint::directionforward :
            driver_hint::directionbackward );
}
