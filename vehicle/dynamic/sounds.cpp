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

// the sounds of a vehicle

#include "stdafx.h"
#include "vehicle/DynObj.h"
#include "physics/mover.h"
#include "vehicle/lightsprogrammer.h"
#include "simulation/simulation.h"
#include "simulation/blackboxrecorder.h"
#include "rendering/lightarray.h"
#include "vehicle/Camera.h"
#include "vehicle/Train.h"
#include "vehicle/odometer.h"
#include "vehicle/pulleemode.h"
#include "vehicle/Driver.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <format>
#include <random>
#include <iterator>
#include <numbers>
#include <optional>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>
#include "utilities/Globals.h"
#include "utilities/Timer.h"
#include "utilities/workerpool.h"
#include "utilities/Logs.h"
#include "utilities/glmHelpers.h"
#include "Console.h"
#include "world/Traction.h"
#include "audio/sound.h"
#include "model/MdlMngr.h"
#include "model/Model3d.h"
#include "vehicle/doorwings.h"
#include "rendering/renderer.h"
#include "editor/editorTerrainStreamer.hpp"
#include "application/uitranscripts.h"
#include "input/messaging.h"
#include "application/application.h"
#include "vehicle/Driver.h"
#include "vehicle/dynamic/internal.h"


// przeliczanie dźwięków, bo będzie słychać bez wyświetlania sektora z pojazdem
void TDynamicObject::RenderSounds() {

    if( false == simulation::is_ready ) { return; }
    if( Global.iPause != 0 )            { return; }

    if( m_startjoltplayed
     && ( std::abs(MoverParameters->AccSVBased) < 0.01
       || GetVelocity() < 0.01 ) ) {
        // if the vehicle comes to a stop set the movement jolt to play when it starts moving again
        m_startjoltplayed = false;
    }

    auto const dt{ Timer::GetDeltaRenderTime() };
    m_powertrainsounds.render( *MoverParameters, dt );

    auto volume{ 0.0 };
    auto frequency{ 1.0 };

    // NBMX dzwiek przetwornicy
    if( MoverParameters->ConverterFlag ) {
        if( MoverParameters->EngineType == TEngineType::ElectricSeriesMotor ) {
            auto const voltage { std::max( MoverParameters->GetTrainsetHighVoltage(), MoverParameters->PantographVoltage ) };
            if( voltage > 0.0 ) {
                // NOTE: we do sound modulation here to avoid sudden jump on voltage loss
                frequency = voltage / (MoverParameters->NominalVoltage * MoverParameters->RList[MoverParameters->RlistSize].Mn);
                frequency *= sConverter.m_frequencyfactor + sConverter.m_frequencyoffset;
                sConverter.pitch( std::clamp( frequency, 0.75, 1.25 ) ); // arbitrary limits )
            }
        }
        else {
            frequency = sConverter.m_frequencyoffset + sConverter.m_frequencyfactor * frequency;
            sConverter.pitch( std::clamp( frequency, 0.75, 1.25 ) ); // arbitrary limits )
        }
        sConverter.play( sound_flags::exclusive | sound_flags::looping );
    }
    else {
        sConverter.stop();
    }

    // Odtworzenie dzwieku wentylatora rezystora hamowania

	if (MoverParameters->BRVentilators)
	{
		sBRVent.play(sound_flags::exclusive | sound_flags::looping);
	}
	else
	{
		sBRVent.stop();
	}





    if( MoverParameters->CompressorSpeed > 0.0 ) {
        // McZapkie! - dzwiek compressor.wav tylko gdy dziala sprezarka
        if( MoverParameters->CompressorFlag ) {
            // for compressor coupled with the diesel engine sound pitch is driven by engine revolutions
            if( MoverParameters->CompressorPower == 3 ) {
                // presume the compressor sound is recorded for idle revolutions
                // increase the pitch according to increase of engine revolutions
                auto const enginefactor {
                    std::clamp( // try to keep the sound pitch in semi-reasonable range
                        MoverParameters->EngineMaxRPM() / MoverParameters->EngineIdleRPM() * MoverParameters->EngineRPMRatio(),
                        0.5, 2.5 ) };
                sCompressor.pitch( enginefactor );
                sCompressorIdle.pitch( enginefactor );
            }
            if( sCompressorIdle.empty() ) {
                // legacy sound path, if there's no dedicated idle sound
                sCompressor.play( sound_flags::exclusive | sound_flags::looping );
            }
            else {
                // enhanced sound path, with dedicated sound for idling compressor
                if( MoverParameters->CompressorGovernorLock ) {
                    sCompressor.stop();
                    sCompressorIdle.play( sound_flags::exclusive | sound_flags::looping );
                }
                else {
                    sCompressor.play( sound_flags::exclusive | sound_flags::looping );
                    sCompressorIdle.stop();
                }
            }
        }
        else {
            sCompressor.stop();
            sCompressorIdle.stop();
        }
    }

    // Winger 160404 - dzwiek malej sprezarki
    if( MoverParameters->PantCompFlag ) {
        sSmallCompressor.play( sound_flags::exclusive | sound_flags::looping );
    }
    else {
        sSmallCompressor.stop();
    }

    // heater sound
    {
        auto const isdieselenginepowered { MoverParameters->EngineType == TEngineType::DieselElectric || MoverParameters->EngineType == TEngineType::DieselEngine };
        if( true == MoverParameters->Heating
         && ( false == isdieselenginepowered
           || std::abs(MoverParameters->enrot) > 0.01 ) ) {
            sHeater
                .pitch( true == sHeater.is_combined() ?
                    std::abs( MoverParameters->enrot ) * 60.f * 0.01f :
                    1.f )
                .play( sound_flags::exclusive | sound_flags::looping );
        }
        else {
            sHeater.stop();
        }
    }

    // battery sound
    if( MoverParameters->Battery ) {
        m_batterysound.play( sound_flags::exclusive | sound_flags::looping );
    }
    else {
        m_batterysound.stop();
    }

    // brake system and braking sounds:

    // brake cylinder piston
    auto const brakepressureratio { std::max( 0.0, MoverParameters->BrakePress ) / std::max( 1.0, MoverParameters->MaxBrakePress[ 3 ] ) };
    if( m_lastbrakepressure != -1.f ) {
        // HACK: potentially reset playback of opening bookend sounds
        if( false == m_brakecylinderpistonadvance.is_playing() ) {
            m_brakecylinderpistonadvance.stop();
        }
        if( false == m_brakecylinderpistonrecede.is_playing() ) {
            m_brakecylinderpistonrecede.stop();
        }
        // actual sound playback
        auto const quantizedratio { static_cast<int>( 15 * brakepressureratio ) };
        auto const lastbrakepressureratio { std::max( 0.f, m_lastbrakepressure ) / std::max( 1.0, MoverParameters->MaxBrakePress[ 3 ] ) };
        auto const quantizedratiochange { quantizedratio - static_cast<int>( 15 * lastbrakepressureratio ) };
        if( quantizedratiochange > 0 ) {
            m_brakecylinderpistonadvance
                .pitch(
                    true == m_brakecylinderpistonadvance.is_combined() ?
                        quantizedratio * 0.01f :
                        m_brakecylinderpistonadvance.m_frequencyoffset + m_brakecylinderpistonadvance.m_frequencyfactor * 1.f )
                .play();
        }
        else if( quantizedratiochange < 0 ) {
            m_brakecylinderpistonrecede
                .pitch( true == m_brakecylinderpistonrecede.is_combined() ?
                        quantizedratio * 0.01f :
                        m_brakecylinderpistonrecede.m_frequencyoffset + m_brakecylinderpistonrecede.m_frequencyfactor * 1.f )
                .play();
        }
    }

	// epbrake - epcompact
	if (MoverParameters->BrakeSystem == TBrakeSystem::ElectroPneumatic && MoverParameters->LocHandle) {
		auto const epbrakepressureratio{ std::min( std::max( 0.0, MoverParameters->LocHandle->GetCP() ) / std::max( 1.0, MoverParameters->MaxBrakePress[0] ),
												   m_epbrakepressurechangedectimer > -1.0f ?
													   std::max( MoverParameters->LocalBrakePosAEIM, MoverParameters->Hamulec->GetEDBCP() / MoverParameters->MaxBrakePress[3]) :
														1.0 ) };
		if ( m_lastepbrakepressure != -1.f ) {
			// HACK: potentially reset playback of opening bookend sounds
			if ( false == m_epbrakepressureincrease.is_playing() ) {
				m_epbrakepressureincrease.stop();
			}
			if ( false == m_epbrakepressuredecrease.is_playing() ) {
				m_epbrakepressuredecrease.stop();
			}
			m_epbrakepressurechangeinctimer += dt;
			m_epbrakepressurechangedectimer += dt;
			// actual sound playback
			auto const epquantizedratio{ static_cast<int>( 50 * epbrakepressureratio) };
			auto const lastepbrakepressureratio { std::max( 0.f, m_lastepbrakepressure ) / std::max( 1.0, MoverParameters->MaxBrakePress[0] ) };
			auto const epquantizedratiochange { epquantizedratio - static_cast<int>( 50 * lastepbrakepressureratio ) };
			if ( epquantizedratiochange > 0 && m_epbrakepressurechangeinctimer > 0.05f) {
				m_epbrakepressureincrease
					.pitch(
						true == m_epbrakepressureincrease.is_combined() ?
						epquantizedratio * 0.01f :
						m_epbrakepressureincrease.m_frequencyoffset + m_epbrakepressureincrease.m_frequencyfactor * 1.f )
					.play();
				m_epbrakepressurechangeinctimer = 0;
			}
			else if ( epquantizedratiochange < 0 && m_epbrakepressurechangedectimer > 0.3f) {
				m_epbrakepressuredecrease
					.pitch(true == m_epbrakepressuredecrease.is_combined() ?
						-epquantizedratiochange * 0.01f :
						m_epbrakepressuredecrease.m_frequencyoffset + m_epbrakepressuredecrease.m_frequencyfactor * 1.f )
					.play();
				m_epbrakepressurechangedectimer = 0;
			}
		}
		if ( m_epbrakepressurechangeinctimer == 0 || m_epbrakepressurechangedectimer == 0 )
		m_lastepbrakepressure = std::min( MoverParameters->LocHandle->GetCP(),
										  MoverParameters->LocalBrakePosAEIM * std::max( 1.0, MoverParameters->MaxBrakePress[0] ) );
	}

    // emergency brake
    if( MoverParameters->EmergencyValveFlow > 0.025 ) {
        // smooth out air flow rate
        m_emergencybrakeflow = m_emergencybrakeflow == 0.0 ? MoverParameters->EmergencyValveFlow : std::lerp(m_emergencybrakeflow, MoverParameters->EmergencyValveFlow, 0.1);
        // scale volume based on the flow rate and on the pressure in the main pipe
        auto const flowpressure { std::clamp( m_emergencybrakeflow, 0.0, 1.0 ) + std::clamp( 0.1 * MoverParameters->PipePress, 0.0, 0.5 ) };
         m_emergencybrake
            .pitch( m_emergencybrake.m_frequencyoffset + 1.0 * m_emergencybrake.m_frequencyfactor )
            .gain( m_emergencybrake.m_amplitudeoffset + std::clamp( flowpressure, 0.0, 1.0 ) * m_emergencybrake.m_amplitudefactor )
            .play( sound_flags::exclusive | sound_flags::looping );
   }
    else if( MoverParameters->EmergencyValveFlow < 0.015 ) {
        m_emergencybrakeflow = 0.0;
        m_emergencybrake.stop();
    }

    // air release
    if( m_lastbrakepressure != -1.f ) {
        // calculate rate of pressure drop in brake cylinder, once it's been initialized
        auto const brakepressuredifference{ m_lastbrakepressure - MoverParameters->BrakePress };
        m_brakepressurechange = std::lerp( m_brakepressurechange, brakepressuredifference / dt, 0.05f );
    }
    m_lastbrakepressure = MoverParameters->BrakePress;
    // ensure some basic level of volume and scale it up depending on pressure in the cylinder; scale this by the air release rate
    volume = rsUnbrake.m_amplitudefactor * m_brakepressurechange * ( 0.25 + 0.75 * brakepressureratio );
    if( m_brakepressurechange > 0.05 && brakepressureratio > 0.05 ) {
        rsUnbrake
            .gain( volume )
            .play( sound_flags::exclusive | sound_flags::looping );
    }
    else {
        // don't stop the sound too abruptly
        volume = std::max( 0.0, rsUnbrake.gain() - 0.5 * dt );
        rsUnbrake.gain( volume );
        if( volume < 0.05 ) {
            rsUnbrake.stop();
        }
    }

    // Dzwiek odluzniacza
    if( MoverParameters->Hamulec->Releaser() ) {
        sReleaser
            .gain(
                std::clamp(
                    MoverParameters->BrakePress * 1.25, // arbitrary multiplier
                    0., 1. ) )
            .play( sound_flags::exclusive | sound_flags::looping );
    }
    else {
        sReleaser.stop();
    }

    // slipping wheels
    if( MoverParameters->SlippingWheels ) {

        if( MoverParameters->UnitBrakeForce > 100.0
            && GetVelocity() > 1.0 ) {

            auto const velocitydifference{ GetVelocity() / MoverParameters->Vmax };
            rsSlippery
                .gain( rsSlippery.m_amplitudeoffset + rsSlippery.m_amplitudefactor * velocitydifference )
                .play( sound_flags::exclusive | sound_flags::looping );
        }
    }
    else {
        rsSlippery.stop();
    }

    // Dzwiek piasecznicy
    if( MoverParameters->SandDose ) {
        sSand.play( sound_flags::exclusive | sound_flags::looping );
    }
    else {
        sSand.stop();
    }

    // brakes
    auto brakeforceratio{ 0.0 };
    if( //( false == mvOccupied->SlippingWheels ) &&
        MoverParameters->UnitBrakeForce > 10.0
     && MoverParameters->Vel > 0.05 ) {

        brakeforceratio =
            std::clamp(
                MoverParameters->UnitBrakeForce / std::max( 1.0, MoverParameters->BrakeForceR( 1.0, MoverParameters->Vel ) / ( MoverParameters->NAxles * std::max( 1, MoverParameters->NBpA ) ) ),
                0.0, 1.0 );
        rsBrake
            .pitch( rsBrake.m_frequencyoffset + MoverParameters->Vel * rsBrake.m_frequencyfactor )
            .gain( rsBrake.m_amplitudeoffset + std::sqrt( brakeforceratio * std::lerp( 0.4, 1.0, MoverParameters->Vel / (1 + MoverParameters->Vmax) ) ) * rsBrake.m_amplitudefactor )
            .play( sound_flags::exclusive | sound_flags::looping );
    }
    else {
        rsBrake.stop();
    }

    // yB: przyspieszacz (moze zadziala, ale dzwiek juz jest)
    if( true == bBrakeAcc ) {
        if( true == TestFlag( MoverParameters->Hamulec->GetSoundFlag(), sf_Acc ) ) {
            sBrakeAcc.play( sound_flags::exclusive );
        }
    }

    // McZapkie-280302 - pisk mocno zacisnietych hamulcow
    if( MoverParameters->Vel > 2.5 ) {

        volume = rsPisk.m_amplitudeoffset + std::lerp( -1.0, 1.0, brakeforceratio ) * rsPisk.m_amplitudefactor;
        if( volume > 0.075 ) {
            rsPisk
                .pitch(
                    true == rsPisk.is_combined() ?
                        MoverParameters->Vel * 0.01f :
                        rsPisk.m_frequencyoffset + rsPisk.m_frequencyfactor * 1.f )
                .gain( volume )
                .play( sound_flags::exclusive | sound_flags::looping );
        }
    }
    else {
        // don't stop the sound too abruptly
        volume = std::max( 0.0, rsPisk.gain() - rsPisk.gain() * 2.5 * dt );
        rsPisk.gain( volume );
    }
    if( volume < 0.05 ) {
        rsPisk.stop();
    }

    // spring brake
    if( m_springbrakesounds.state != MoverParameters->SpringBrake.Activate ) {
        m_springbrakesounds.state = MoverParameters->SpringBrake.Activate;
        if( m_springbrakesounds.state ) {
            m_springbrakesounds.activate.play( sound_flags::exclusive );
            m_springbrakesounds.release.stop();
        }
        else {
            m_springbrakesounds.activate.stop();
            m_springbrakesounds.release.play( sound_flags::exclusive );
        }
    }

    // other sounds
    // load exchange
    if( MoverParameters->LoadStatus & 1 ) {
        m_exchangesounds.unloading.play( sound_flags::exclusive );
    }
    else {
        m_exchangesounds.unloading.stop();
    }
    if( MoverParameters->LoadStatus & 2 ) {
        m_exchangesounds.loading.play( sound_flags::exclusive );
    }
    else {
        m_exchangesounds.loading.stop();
    }
    // NBMX sygnal odjazdu
    if( MoverParameters->Doors.has_warning ) {
        auto const lowvoltagepower { MoverParameters->Power24vIsAvailable || MoverParameters->Power110vIsAvailable };
        for( auto &doorspeaker : m_doorspeakers ) {
            // TBD, TODO: per-location door state triggers?
            if( MoverParameters->DepartureSignal
             && lowvoltagepower
/*
             || ( ( MoverParameters->DoorCloseCtrl = control::autonomous )
               && ( ( ( false == MoverParameters->DoorLeftOpened )  && ( dDoorMoveL > 0.0 ) )
                 || ( ( false == MoverParameters->DoorRightOpened ) && ( dDoorMoveR > 0.0 ) ) ) )
*/
                 ) {
                // for the autonomous doors play the warning automatically whenever a door is closing
                // MC: pod warunkiem ze jest zdefiniowane w chk
                doorspeaker.departure_signal.play( sound_flags::exclusive | sound_flags::looping );
            }
            else {
                doorspeaker.departure_signal.stop();
            }
        }
    }
    // announcements
    {
        auto const lowvoltagepower { MoverParameters->Power24vIsAvailable || MoverParameters->Power110vIsAvailable };
        if( lowvoltagepower ) {
            // system is powered up, can play queued announcements
            if( false == m_pasystem.announcement_queue.empty()
             && false == m_pasystem.announcement.is_playing() ) {
                // pull first sound from the queue
                m_pasystem.announcement = m_pasystem.announcement_queue.front();
                m_pasystem.announcement.owner( this );
                m_pasystem.announcement.range( 0.5 * MoverParameters->Dim.L * -1 );
                if( m_pasystem.soundproofing ) {
                    if( !m_pasystem.announcement.soundproofing() ) {
                        m_pasystem.announcement.soundproofing() = m_pasystem.soundproofing;
                    }
                }
                m_pasystem.announcement.play();
                m_pasystem.announcement_queue.pop_front();
            }
        }
        else {
            m_pasystem.announcement.stop();
            m_pasystem.announcement_queue.clear();
        }
    }
    // NBMX Obsluga drzwi, MC: zuniwersalnione
    std::array<side, 2> const sides { side::right, side::left };
    for( auto const side : sides ) {

        auto const &door { MoverParameters->Doors.instances[ side ] };

        if( true == door.is_opening ) {
            // door sounds
            if( false == door.step_unfolding // no wait if no doorstep
             || MoverParameters->Doors.step_type == 2 ) { // no wait for rotating doorstep
                if( door.position < 0.5f ) { // safety measure, to keep slightly too short sounds from repeating
                    for( auto &doorsounds : m_doorsounds ) {
                        if( doorsounds.placement == side ) {
                            // determine left side doors from their offset
                            doorsounds.rsDoorOpen.play( sound_flags::exclusive );
                            doorsounds.rsDoorClose.stop();
                        }
                    }
                }
            }
        }
        if( true == door.is_closing ) {
            // door sounds can start playing before the door begins moving but shouldn't cease once the door closes
            if( door.position > 0.f ) {
                for( auto &doorsounds : m_doorsounds ) {
                    if( doorsounds.placement == side ) {
                        // determine left side doors from their offset
                        doorsounds.rsDoorClose.play( sound_flags::exclusive );
                        doorsounds.rsDoorOpen.stop();
                    }
                }
            }
        }
        // doorstep sounds
        if( door.step_unfolding ) {
            for( auto &doorsounds : m_doorsounds ) {
                if( doorsounds.placement == side ) {
                    doorsounds.step_open.play( sound_flags::exclusive );
                    doorsounds.step_close.stop();
                }
            }
        }
        if( door.step_folding ) {
            if( door.step_position < 1.0f ) { // sanity check, the vehicles may keep the doorstep unfolded until the door close
                for( auto &doorsounds : m_doorsounds ) {
                    if( doorsounds.placement == side ) {
                        // determine left side doors from their offset
                        doorsounds.step_close.play( sound_flags::exclusive );
                        doorsounds.step_open.stop();
                    }
                }
            }
        }
    }
    // door locks
    if( MoverParameters->Doors.is_locked != m_doorlocks ) {
        // toggle state of the locks...
        m_doorlocks = !m_doorlocks;
        // ...and play relevant sounds
        for( auto &door : m_doorsounds ) {
            if( m_doorlocks ) {
                door.lock.play( sound_flags::exclusive );
            }
            else {
                door.unlock.play( sound_flags::exclusive );
            }
        }
    }

    // horns
    {
        // for moving vehicle combine regular horn activation flag with emergency brake horn activation flag, if the brake is active;
        // the horns without the air or the voltage they need stay silent
        auto const warningsignal { MoverParameters->SoundingHorns(
            ( MoverParameters->Vel > 0.5 && MoverParameters->AlarmChainFlag ?
                MoverParameters->EmergencyBrakeWarningSignal :
                0 )
            | MoverParameters->WarningSignal ) };

        if( TestFlag( warningsignal, 1 ) ) {
            sHorn1.play( sound_flags::exclusive | sound_flags::looping );
        }
        else {
            sHorn1.stop();
        }
        if( TestFlag( warningsignal, 2 ) ) {
            sHorn2.play( sound_flags::exclusive | sound_flags::looping );
        }
        else {
            sHorn2.stop();
        }
        if( TestFlag( warningsignal, 4 ) ) {
            sHorn3.play( sound_flags::exclusive | sound_flags::looping );
        }
        else {
            sHorn3.stop();
        }
    }
    // szum w czasie jazdy
    if( GetVelocity() > 0.5
#ifdef EU07_SOUND_BOGIESOUNDS
     && false == m_bogiesounds.empty()
#endif
     && ( // compound test whether the vehicle belongs to user-driven consist (as these don't emit outer noise in cab view)
            FreeFlyModeFlag ? true : // in external view all vehicles emit outer noise
            simulation::Train == nullptr ? true : // no player train, e.g. it went through a portal and the view isn't switched yet
            ctOwner == nullptr ? true : // standalone vehicle, can't be part of user-driven train
            ctOwner != simulation::Train->Dynamic()->ctOwner ? true : // confirmed isn't a part of the user-driven train
            Global.CabWindowOpen ? true : // sticking head out we get to hear outer noise
            false ) ) {
#ifdef EU07_SOUND_BOGIESOUNDS
        auto const &bogiesound { m_bogiesounds.front() };
#else
        auto const &bogiesound { m_outernoise };
#endif
        // frequency calculation
        auto const normalizer { (
            true == bogiesound.is_combined() ?
                MoverParameters->Vmax * 0.01f :
                1.f ) };
        frequency =
            bogiesound.m_frequencyoffset
            + bogiesound.m_frequencyfactor * MoverParameters->Vel * normalizer;

        // volume calculation
        volume =
            bogiesound.m_amplitudeoffset
            + bogiesound.m_amplitudefactor * MoverParameters->Vel;
        if( brakeforceratio > 0.0 ) {
            // hamulce wzmagaja halas
            volume *= 1 + 0.125 * brakeforceratio;
        }
        // scale volume by track quality
        // TODO: track quality and/or environment factors as separate subroutine
        volume *=
            std::lerp(
                0.8, 1.2,
                std::clamp(
                    MyTrack->iQualityFlag / 20.0,
                    0.0, 1.0 ) );
        // for single sample sounds muffle the playback at low speeds
        if( false == bogiesound.is_combined() ) {
            volume *=
                std::lerp(
                    0.0, 1.0,
                    std::clamp(
                        MoverParameters->Vel / 40.0,
                        0.0, 1.0 ) );
        }
                        switch( MyTrack->eEnvironment ) {
                    case e_tunnel: {
                        volume *= 1.3;
                        break;
                    }
                    case  e_bridge: {
                        volume *= 1.5;
                        break;
                    }
                    default: {
                        break;
                    }
                }

        if( volume > 0.05 ) {
            // apply calculated parameters to all motor instances
#ifdef EU07_SOUND_BOGIESOUNDS
            for( auto &bogiesound : m_bogiesounds ) {
                bogiesound
                    .pitch( frequency ) // arbitrary limits to prevent the pitch going out of whack
                    .gain( volume )
                    .play( sound_flags::exclusive | sound_flags::looping );
            }
#else
            m_outernoise
                .pitch( frequency ) // arbitrary limits to prevent the pitch going out of whack
                .gain( volume )
                .play( sound_flags::exclusive | sound_flags::looping );
#endif
        }
        else {
            // stop all noise instances
#ifdef EU07_SOUND_BOGIESOUNDS
            for( auto &bogiesound : m_bogiesounds ) {
                bogiesound.stop();
            }
#else
            m_outernoise.stop();
#endif
        }
    }
    else {
        // don't play the optional ending sound if the listener switches views
#ifdef EU07_SOUND_BOGIESOUNDS
        for( auto &bogiesound : m_bogiesounds ) {
            bogiesound.stop( false == FreeFlyModeFlag );
        }
#else
        m_outernoise.stop( false == FreeFlyModeFlag );
#endif
    }
    // flat spot sound
    if( MoverParameters->CategoryFlag == 1 ) {
        // trains only
        if( GetVelocity() > 1.0
         && MoverParameters->WheelFlat > 5.0 ) {
            m_wheelflat
                .pitch( m_wheelflat.m_frequencyoffset + std::abs( MoverParameters->nrot ) * m_wheelflat.m_frequencyfactor )
                .gain( m_wheelflat.m_amplitudeoffset + m_wheelflat.m_amplitudefactor * ( ( 1.0 + MoverParameters->Vel / MoverParameters->Vmax + std::clamp( MoverParameters->WheelFlat / 60.0, 0.0, 1.0 ) ) / 3.0 ) )
                .play( sound_flags::exclusive | sound_flags::looping );
        }
        else {
            m_wheelflat.stop();
        }
    }

    // youBy: dzwiek ostrych lukow i ciasnych zwrotek
    if( MoverParameters->Vel > 5.0
     && ts.R * ts.R > 1.0
     && std::abs(ts.R) < 15000.0 ) {
        // scale volume with curve radius and vehicle speed
        volume =
            std::abs( MoverParameters->AccN ) // * MoverParameters->AccN
            * std::lerp(
                0.5, 1.0,
                std::clamp(
                    MoverParameters->Vel / 40.0,
                    0.0, 1.0 ) )
            * ( MyTrack->eType == tt_Switch && std::abs(ts.R) < 1500.0 ? 100.0 : 1.0 );
    }
    else {
        volume = 0;
    }
    if( volume > 0.05 ) {
        rscurve
            .pitch(
                true == rscurve.is_combined() ?
                    MoverParameters->Vel * 0.01f :
                    rscurve.m_frequencyoffset + rscurve.m_frequencyfactor * 1.f )
            .gain( 2.5 * volume )
            .play( sound_flags::exclusive | sound_flags::looping );
    }
    else {
        rscurve.stop();
    }

    // movement start jolt
    if( false == m_startjoltplayed ) {
        auto const velocity { GetVelocity() };
        if( ( MoverParameters->V > 0.0 ? MoverParameters->AccSVBased > 0.1 : MoverParameters->AccSVBased < 0.1 )
         && velocity > 1.0
         && velocity < 15.0 ) {
            m_startjolt.play( sound_flags::exclusive );
            m_startjoltplayed = true;
        }
    }

    // McZapkie! - to wazne - SoundFlag wystawiane jest przez moje moduly gdy zachodza pewne wydarzenia komentowane dzwiekiem.
    // pneumatic relay
    if( TestFlag( MoverParameters->SoundFlag, sound::pneumatic ) ) {
        dsbPneumaticRelay
            .gain(
                true == TestFlag( MoverParameters->SoundFlag, sound::loud ) ?
                    1.0f :
                    0.8f )
            .play();
    }
    // door permit
    if( TestFlag( MoverParameters->SoundFlag, sound::doorpermit ) ) {
        // NOTE: current implementation doesn't discern between permit for left/right side,
        // which may be undesired in weird setups with doors only on one side
        // TBD, TODO: rework into dedicated sound event flag for each door location instance?
        for( auto &door : m_doorsounds ) {
            door.permit_granted.play( sound_flags::exclusive );
        }
    }

    // couplers
    int couplerindex { 0 };
    for( auto &couplersounds : m_couplersounds ) {

        auto &coupler { MoverParameters->Couplers[ couplerindex ] };

        if( coupler.sounds == sound::none ) {
            ++couplerindex;
            continue;
        }

        if( true == TestFlag( coupler.sounds, sound::bufferclash ) ) {
            // zderzaki uderzaja o siebie
            if( true == TestFlag( coupler.sounds, sound::loud ) ) {
                // loud clash
                if( false == couplersounds.dsbBufferClamp_loud.empty() ) {
                    // dedicated sound for loud clash
                    couplersounds.dsbBufferClamp_loud
                        .gain( 1.f )
                        .play( sound_flags::exclusive );
                }
                else {
                    // fallback on the standard sound
                    couplersounds.dsbBufferClamp
                        .gain( 1.f )
                        .play( sound_flags::exclusive );
                }
            }
            else {
                // basic clash
                couplersounds.dsbBufferClamp
                    .gain( 1.f )
                    .play( sound_flags::exclusive );
            }
        }

        if( true == TestFlag( coupler.sounds, sound::couplerstretch ) ) {
            // sprzegi sie rozciagaja
            if( true == TestFlag( coupler.sounds, sound::loud ) ) {
                // loud stretch
                if( false == couplersounds.dsbCouplerStretch_loud.empty() ) {
                    // dedicated sound for loud stretch
                    couplersounds.dsbCouplerStretch_loud
                        .gain( 1.f )
                        .play( sound_flags::exclusive );
                }
                else {
                    // fallback on the standard sound
                    couplersounds.dsbCouplerStretch
                        .gain( 1.f )
                        .play( sound_flags::exclusive );
                }
            }
            else {
                // basic clash
                couplersounds.dsbCouplerStretch
                    .gain( 1.f )
                    .play( sound_flags::exclusive );
            }
        }
        // attach/detach sounds
        if( ( coupler.sounds & sound::detach ) == 0 ) {
            // potentially added some couplings
            if( ( coupler.sounds & sound::attachcoupler ) != 0 ) {
                couplersounds.attach_coupler.play();
            }
            if( ( coupler.sounds & sound::attachbrakehose ) != 0 ) {
                couplersounds.attach_brakehose.play();
            }
            if( ( coupler.sounds & sound::attachmainhose ) != 0 ) {
                couplersounds.attach_mainhose.play();
            }
            if( ( coupler.sounds & sound::attachcontrol ) != 0 ) {
                couplersounds.attach_control.play();
            }
            if( ( coupler.sounds & sound::attachgangway ) != 0 ) {
                couplersounds.attach_gangway.play();
            }
            if( ( coupler.sounds & sound::attachheating ) != 0 ) {
                couplersounds.attach_heating.play();
            }
        }
        else {
            // potentially removed some couplings
            if( ( coupler.sounds & sound::attachcoupler ) != 0 ) {
                couplersounds.detach_coupler.play();
            }
            if( ( coupler.sounds & sound::attachbrakehose ) != 0 ) {
                couplersounds.detach_brakehose.play();
            }
            if( ( coupler.sounds & sound::attachmainhose ) != 0 ) {
                couplersounds.detach_mainhose.play();
            }
            if( ( coupler.sounds & sound::attachcontrol ) != 0 ) {
                couplersounds.detach_control.play();
            }
            if( ( coupler.sounds & sound::attachgangway ) != 0 ) {
                couplersounds.detach_gangway.play();
            }
            if( ( coupler.sounds & sound::attachheating ) != 0 ) {
                couplersounds.detach_heating.play();
            }
        }
        if( true == TestFlag( coupler.sounds, sound::attachadapter ) ) {
            couplersounds.dsbAdapterAttach.play();
        }
        if( true == TestFlag( coupler.sounds, sound::removeadapter ) ) {
            couplersounds.dsbAdapterRemove.play();
        }

        ++couplerindex;
        coupler.sounds = 0;
    }

    MoverParameters->SoundFlag = 0;
    // McZapkie! - koniec obslugi dzwiekow z mover.pas

    // physics.derailment new: off the track the vehicle scrapes along the ground until it stops, or until it is put back
    // on the rails (rerail()) while still sliding
    if( MoverParameters->OffTrack.active() && false == MoverParameters->OffTrack.at_rest() ) {
        if( false == rsDerailment.is_playing() ) {
            rsDerailment.play( sound_flags::exclusive | sound_flags::looping );
        }
        m_offtrackscraping = true;
    }
    else if( std::exchange( m_offtrackscraping, false ) && rsDerailment.is_playing() ) {
        rsDerailment.stop();
    }

// special events
    if( MoverParameters->EventFlag ) {
        // McZapkie: w razie wykolejenia
        if( true == MoverParameters->DamageFlag.HasAll(VehicleDamage::derailed) ) {
            rsDerailment.play( sound_flags::exclusive );
        }

        MoverParameters->EventFlag = false;
    }
}

void TDynamicObject::play_mirror_sounds( side const Side, mirror_moves::event const Event ) {

    if( Event == mirror_moves::event::none ) { return; }
    // as UpdateMirror(), only the mirrors of the occupied cab move
    if( MoverParameters->CabOccupied == 0 ) { return; }
    auto const cabend { MoverParameters->CabOccupied > 0 ? end::front : end::rear };
    for( auto &mirror : m_mirrorsounds ) {
        if( mirror.placement != Side || mirror.end != cabend ) { continue; }
        auto &sound { Event == mirror_moves::event::unfolding ? mirror.unfold : mirror.fold };
        auto &other { Event == mirror_moves::event::unfolding ? mirror.fold : mirror.unfold };
        other.stop();
        sound.play( sound_flags::exclusive );
    }
}

void
TDynamicObject::powertrain_sounds::position( glm::vec3 const Location ) {

    auto const nullvector { glm::vec3() };
    std::vector<sound_source *> enginesounds = {
        &inverter,
        &motor_relay, &dsbWejscie_na_bezoporow, &motor_parallel, &motor_shuntfield, &rsWentylator,
        &engine, &engine_ignition, &engine_shutdown, &engine_revving, &engine_turbo, &oil_pump, &fuel_pump, &water_pump, &water_heater, &radiator_fan, &radiator_fan_aux,
        &transmission, &rsEngageSlippery, &retarder
    };
    for( auto sound : enginesounds ) {
        if( sound->offset() == nullvector ) {
            sound->offset( Location );
        }
    }
}

void
TDynamicObject::powertrain_sounds::render( TMoverParameters const &Vehicle, double const Deltatime ) {

    if( Vehicle.Power == 0 ) { return; }

    double frequency { 1.0 };
    double volume { 0.0 };

    // oil pump
    if( true == Vehicle.OilPump.is_active ) {
        oil_pump
            .pitch( oil_pump.m_frequencyoffset + oil_pump.m_frequencyfactor * 1.f )
            .gain( oil_pump.m_amplitudeoffset + oil_pump.m_amplitudefactor * 1.f )
            .play( sound_flags::exclusive | sound_flags::looping );
    }
    else {
        oil_pump.stop();
    }
    // fuel pump
    if( true == Vehicle.FuelPump.is_active ) {
        fuel_pump
            .pitch( fuel_pump.m_frequencyoffset + fuel_pump.m_frequencyfactor * 1.f )
            .gain( fuel_pump.m_amplitudeoffset + fuel_pump.m_amplitudefactor * 1.f )
            .play( sound_flags::exclusive | sound_flags::looping );
    }
    else {
        fuel_pump.stop();
    }
    // water pump
    if( true == Vehicle.WaterPump.is_active ) {
        water_pump
            .pitch( water_pump.m_frequencyoffset + water_pump.m_frequencyfactor * 1.f )
            .gain( water_pump.m_amplitudeoffset + water_pump.m_amplitudefactor * 1.f )
            .play( sound_flags::exclusive | sound_flags::looping );
    }
    else {
        water_pump.stop();
    }
    // water heater
    if( true == Vehicle.WaterHeater.is_active ) {
        water_heater
            .pitch( water_heater.m_frequencyoffset + water_heater.m_frequencyfactor * 1.f )
            .gain( water_heater.m_amplitudeoffset + water_heater.m_amplitudefactor * 1.f )
            .play( sound_flags::exclusive | sound_flags::looping );
    }
    else {
        water_heater.stop();
    }
    // engine sounds
    // ignition
    if( engine_state_last != Vehicle.Mains ) {

        if( true == Vehicle.Mains ) {
           // TODO: separate engine and main circuit
           // engine activation
            engine_shutdown.stop();
            engine_ignition
                .pitch( engine_ignition.m_frequencyoffset + engine_ignition.m_frequencyfactor * 1.f )
                .gain( engine_ignition.m_amplitudeoffset + engine_ignition.m_amplitudefactor * 1.f )
                .play( sound_flags::exclusive );
            // main circuit activation
            linebreaker_close
                .pitch( linebreaker_close.m_frequencyoffset + linebreaker_close.m_frequencyfactor * 1.f )
                .gain( linebreaker_close.m_amplitudeoffset + linebreaker_close.m_amplitudefactor * 1.f )
                .play();
        }
        else {
            // engine deactivation
            engine_ignition.stop();
            engine_shutdown.pitch( engine_shutdown.m_frequencyoffset + engine_shutdown.m_frequencyfactor * 1.f )
                .gain( engine_shutdown.m_amplitudeoffset + engine_shutdown.m_amplitudefactor * 1.f )
                .play( sound_flags::exclusive );
            // main circuit deactivation
            linebreaker_open
                .pitch( linebreaker_open.m_frequencyoffset + linebreaker_open.m_frequencyfactor * 1.f )
                .gain( linebreaker_open.m_amplitudeoffset + linebreaker_open.m_amplitudefactor * 1.f )
                .play();
        }
        engine_state_last = Vehicle.Mains;
    }
    // main engine sound
    if( true == Vehicle.Mains ) {

        if( std::abs(Vehicle.enrot) > 0.01
            // McZapkie-280503: zeby dla dumb dzialal silnik na jalowych obrotach
         || Vehicle.EngineType == TEngineType::Dumb ) {

            // frequency calculation
            auto const normalizer { (
                true == engine.is_combined() ?
                    // for combined engine sound we calculate sound point in rpm, to make .mmd files setup easier
                    // NOTE: we supply 1/100th of actual value, as the sound module converts does the opposite, converting received (typically) 0-1 values to 0-100 range
                    60.f * 0.01f :
                    // for legacy single-piece sounds the standard frequency calculation is left untouched
                    1.f ) };
            frequency =
                engine.m_frequencyoffset
                + engine.m_frequencyfactor * std::abs( Vehicle.enrot ) * normalizer;

            if( Vehicle.EngineType == TEngineType::Dumb ) {
                frequency -= 0.2 * Vehicle.EnginePower / ( 1 + Vehicle.Power * 1000 );
            }

            // base volume calculation
            switch( Vehicle.EngineType ) {
                // TODO: check calculated values
                case TEngineType::DieselElectric: {

                    volume =
                        engine.m_amplitudeoffset
                        + engine.m_amplitudefactor * (
                            0.25 * ( Vehicle.EnginePower / Vehicle.Power )
                          + 0.75 * Vehicle.EngineRPMRatio() );
                    break;
                }
                case TEngineType::DieselEngine: {
                    if( Vehicle.enrot > 0.0 ) {
                        volume = Vehicle.EnginePower > 0 ? engine.m_amplitudeoffset + engine.m_amplitudefactor * Vehicle.diesel.fill :
					                                       engine.m_amplitudeoffset + engine.m_amplitudefactor * std::fabs(Vehicle.enrot / Vehicle.diesel.maxRevolutions);
                    }
                    break;
                }
                default: {
                    volume =
                        engine.m_amplitudeoffset
                        + engine.m_amplitudefactor * ( Vehicle.EnginePower + std::fabs( Vehicle.enrot ) * 60.0 );
                    break;
                }
            }

            if( engine_volume >= 0.05 ) {

                auto enginerevvolume { 0.f };
                if( Vehicle.EngineType == TEngineType::DieselElectric
                 || Vehicle.EngineType == TEngineType::DieselEngine ) {
                    // diesel engine revolutions increase; it can potentially decrease volume of base engine sound
                    if( engine_revs_last != -1.f ) {
                        // calculate potential recent increase of engine revolutions
                        auto const revolutionsperminute { Vehicle.enrot * 60 };
                        auto const revolutionsdifference { revolutionsperminute - engine_revs_last };
                        auto const idlerevolutionsthreshold { 1.01 * Vehicle.EngineIdleRPM() };
                        engine_revs_change = std::max( 0.0, engine_revs_change - 2.5 * Deltatime );
                        if( revolutionsperminute > idlerevolutionsthreshold
                         && revolutionsdifference > 1.0 * Deltatime ) {
                            engine_revs_change = std::clamp( engine_revs_change + 5.0 * Deltatime, 0.0, 1.25 );
                        }
                        enginerevvolume = 0.8 * engine_revs_change;
                    }
                    engine_revs_last = Vehicle.enrot * 60;

                    auto const enginerevfrequency { (
                        true == engine_revving.is_combined() ?
                            // if the sound contains multiple samples we reuse rpm-based calculation from the engine
                            frequency :
                            engine_revving.m_frequencyoffset + 1.f * engine_revving.m_frequencyfactor ) };

                    if( enginerevvolume > 0.02 ) {
                        engine_revving
                            .pitch( enginerevfrequency )
                            .gain( enginerevvolume )
                            .play( sound_flags::exclusive );
                    }
                    else {
                        engine_revving.stop();
                    }
                } // diesel engines

                // multi-part revving sound pieces replace base engine sound, single revving simply gets mixed with the base
                auto const enginevolume { (
                    enginerevvolume > 0.02 && true == engine_revving.is_combined() ?
                        std::max( 0.0, engine_volume - enginerevvolume ) :
                        engine_volume ) };
                engine
                    .pitch( frequency )
                    .gain( enginevolume )
                    .play( sound_flags::exclusive | sound_flags::looping );
            } // enginevolume > 0.05
        }
        else {
            engine.stop();
        }
    }

	if (Vehicle.Mains) {
		float power = std::clamp(Vehicle.inductionMotor.forceShare, 0.0, 1.0);

		fake_engine
		        .pitch(fake_engine.m_frequencyoffset + power * fake_engine.m_frequencyfactor)
		        .gain(fake_engine.m_amplitudeoffset + power * fake_engine.m_amplitudefactor)
		        .play(sound_flags::exclusive | sound_flags::looping);
	}
	else {
		fake_engine.stop();
	}

    engine_volume = std::lerp( engine_volume, volume, 0.25 );
    if( engine_volume < 0.05 ) {
        engine.stop();
    }

    // youBy - przenioslem, bo diesel tez moze miec turbo
    if( Vehicle.TurboTest > 0 ) {
        // udawanie turbo:
		auto const pitch_diesel{Vehicle.EngineType == TEngineType::DieselEngine || Vehicle.EngineType == TEngineType::DieselElectric ? Vehicle.enrot / Vehicle.diesel.maxRevolutions * Vehicle.diesel.fill : 1};
        auto const goalpitch { std::max( 0.025, ( /*engine_volume **/ pitch_diesel + engine_turbo.m_frequencyoffset ) * engine_turbo.m_frequencyfactor ) };
        auto const goalvolume { (
            Vehicle.MainCtrlPos >= Vehicle.TurboTest && Vehicle.enrot > 0.1 ?
                std::max( 0.0, ( engine_turbo_pitch + engine_turbo.m_amplitudeoffset ) * engine_turbo.m_amplitudefactor ) :
                0.0 ) };
        auto const currentvolume { engine_turbo.gain() };
        auto const changerate { 0.4 * Deltatime };

        engine_turbo_pitch = engine_turbo_pitch > goalpitch ? std::max(goalpitch, engine_turbo_pitch - changerate * 0.5) : std::min(goalpitch, engine_turbo_pitch + changerate);

        volume = currentvolume > goalvolume ? std::max(goalvolume, currentvolume - changerate) : std::min(goalvolume, currentvolume + changerate);

        engine_turbo
            .pitch( 0.4 + engine_turbo_pitch * 0.4 )
            .gain( volume );

        if( volume > 0.05 ) {
            engine_turbo.play( sound_flags::exclusive | sound_flags::looping );
        }
        else {
            engine_turbo.stop();
        }
    }

    if( Vehicle.diesel.clutchPressure > 0.1 ) {
        if( std::abs( Vehicle.diesel.clutchSlip ) > 0.2 ) {
            frequency = rsEngageSlippery.m_frequencyoffset + rsEngageSlippery.m_frequencyfactor * std::fabs( Vehicle.diesel.clutchSlip );
            volume = rsEngageSlippery.m_amplitudeoffset + rsEngageSlippery.m_amplitudefactor * Vehicle.diesel.clutchPressure;
        }
        else {
            frequency = 1.f; // rsEngageSlippery.FA+0.7*rsEngageSlippery.FM*(fabs(mvControlled->enrot)+mvControlled->nmax);
            volume = Vehicle.diesel.clutchPressure > 0.2 ? rsEngageSlippery.m_amplitudeoffset + 0.2 * rsEngageSlippery.m_amplitudefactor * (Vehicle.enrot / Vehicle.nmax) : 0.f;
        }

        rsEngageSlippery
            .pitch( frequency )
            .gain( volume )
            .play( sound_flags::exclusive | sound_flags::looping );
    }
    else {
        rsEngageSlippery.stop();
    }

	if (Vehicle.retarder.present) {
		float speed = std::abs(Vehicle.retarder.revolutions);

		retarder
			.pitch(retarder.m_frequencyoffset + speed * retarder.m_frequencyfactor)
			.gain(retarder.m_amplitudeoffset + Vehicle.retarder.fill * retarder.m_amplitudefactor);

		if (retarder.gain() > 0.01 && speed > 1 && Vehicle.retarder.clutchActive) {
			retarder.play(sound_flags::exclusive | sound_flags::looping);
		}
		else {
			retarder.stop();
		}
	}

    // motor sounds
    volume = 0.0;
    // generic traction motor sounds
    if( false == motors.empty() ) {

        if( std::abs( Vehicle.nrot ) > 0.01 ) {

            auto const &motor { motors.front() };
            // frequency calculation
            auto normalizer { 1.f };
            if( true == motor.is_combined() ) {
                // for combined motor sound we calculate sound point in rpm, to make .mmd files setup easier
                // NOTE: we supply 1/100th of actual value, as the sound module converts does the opposite, converting received (typically) 0-1 values to 0-100 range
                normalizer = 60.f * 0.01f;
            }
            auto const motorrevolutions { std::abs( Vehicle.nrot ) * Vehicle.Transmision.Ratio };
            frequency =
                motor.m_frequencyoffset
                + motor.m_frequencyfactor * motorrevolutions * normalizer;

            // base volume calculation
            switch( Vehicle.EngineType ) {
                case TEngineType::ElectricInductionMotor: {
                    volume =
                        motor.m_amplitudeoffset
                        + motor.m_amplitudefactor * ( Vehicle.EnginePower + motorrevolutions * 2 );
                    break;
                }
                case TEngineType::ElectricSeriesMotor: {
                    volume =
                        motor.m_amplitudeoffset
                        + motor.m_amplitudefactor * ( Vehicle.EnginePower + motorrevolutions * 60.0 );
                    break;
                }
                default: {
                    volume =
                        motor.m_amplitudeoffset
                        + motor.m_amplitudefactor * motorrevolutions * 60.0;
                    break;
                }
            }

            if( Vehicle.EngineType == TEngineType::ElectricSeriesMotor ) {
                // volume variation
                if( volume < 1.0
                    && Vehicle.EnginePower < 100 ) {

                    auto const volumevariation { LocalRandom( 100 ) * Vehicle.enrot / ( 1 + Vehicle.nmax ) };
                    if( volumevariation < 2 ) {
                        volume += volumevariation / 200;
                    }
                }

                if( Vehicle.DynamicBrakeFlag
                    && Vehicle.EnginePower > 0.1 ) {
                    // Szociu - 29012012 - jeżeli uruchomiony jest hamulec elektrodynamiczny, odtwarzany jest dźwięk silnika
                    volume += 0.8;
                }
            }
            // scale motor volume based on whether they're active
            motor_momentum =
                std::clamp(
                    motor_momentum
                    - 1.0 * Deltatime // smooth out decay
                    + std::abs( Vehicle.Mm ) / 60.0 * Deltatime,
                    0.0, 1.25 );
            volume *= std::max( 0.25f, motor_momentum );
            motor_volume = std::lerp( motor_volume, volume, 0.25 );
            if( motor_volume >= 0.05 ) {
                // apply calculated parameters to all motor instances
                for( auto &motor : motors ) {
                    motor
                        .pitch( frequency )
                        .gain( motor_volume )
                        .play( sound_flags::exclusive | sound_flags::looping );
                }
            }
            else {
                // stop all motor instances
                for( auto &motor : motors ) {
                    motor.stop();
                }
            }
        }
        else {
            // stop all motor instances
            motor_volume = 0.0;
            for( auto &motor : motors ) {
                motor.stop();
            }
        }
    }
    // inverter-specific traction motor sounds
    if( false == acmotors.empty() ) {

        if( Vehicle.EngineType == TEngineType::ElectricInductionMotor ) {
            if( Vehicle.inductionMotor.inverterFrequency > 0.001 ) {

                for( auto &motor : acmotors ) {
                    motor
                        .pitch( motor.m_frequencyoffset + motor.m_frequencyfactor * Vehicle.inductionMotor.inverterFrequency )
                        .gain( motor.m_amplitudeoffset + motor.m_amplitudefactor * std::sqrt( std::abs( Vehicle.inductionMotor.forceShare ) ) )
                        .play( sound_flags::exclusive | sound_flags::looping );
                }
            }
            else {
                for( auto &motor : acmotors ) {
                    motor.stop();
                }
            }
        }
    }
    // motor blowers
    if( false == motorblowers.empty() ) {
        for( auto &blowersound : motorblowers ) {
            // match the motor blower and the sound source based on whether they're located in the front or the back of the vehicle
            auto const &blower { Vehicle.MotorBlowers[ ( blowersound.offset().z > 0 ? end::front : end::rear ) ] };
            // TODO: for the sounds with provided bookends invoke stop() when the stop is triggered and revolutions start dropping, instead of after full stop
            if( blower.revolutions > 1 ) {
                // NOTE: for motorblowers with fixed speed if the sound has defined bookends we skip revolutions-based part of frequency/volume adjustments
                auto const revolutionmodifier { Vehicle.MotorBlowers[end::front].speed < 0.f && blowersound.has_bookends() ? 1.f : blower.revolutions };
                blowersound
                    .pitch(
                        true == blowersound.is_combined() ?
                            blower.revolutions * 0.01f :
                            blowersound.m_frequencyoffset + blowersound.m_frequencyfactor * revolutionmodifier )
                    .gain( blowersound.m_amplitudeoffset + blowersound.m_amplitudefactor * revolutionmodifier )
                    .play( sound_flags::exclusive | sound_flags::looping );
            }
            else {
                blowersound.stop();
            }
        }
    }
    // inverter sounds
    if( Vehicle.EngineType == TEngineType::ElectricInductionMotor ) {
        if( Vehicle.inductionMotor.inverterFrequency > 0.001 ) {

            volume = inverter.m_amplitudeoffset + inverter.m_amplitudefactor * std::sqrt( std::abs( Vehicle.inductionMotor.forceShare) );

            inverter
                .pitch( inverter.m_frequencyoffset + inverter.m_frequencyfactor * Vehicle.inductionMotor.inverterFrequency )
                .gain( volume )
                .play( sound_flags::exclusive | sound_flags::looping );
        }
        else {
            inverter.stop();
        }
    }
    // ventillator sounds
    if( Vehicle.RventRot > 0.1 ) {

        rsWentylator
            .pitch( rsWentylator.m_frequencyoffset + rsWentylator.m_frequencyfactor * Vehicle.RventRot )
            .gain( rsWentylator.m_amplitudeoffset + rsWentylator.m_amplitudefactor * Vehicle.RventRot )
            .play( sound_flags::exclusive | sound_flags::looping );
    }
    else {
        // ...otherwise shut down the sound
        rsWentylator.stop();
    }
    // radiator fan sounds
    if( Vehicle.EngineType == TEngineType::DieselEngine
     || Vehicle.EngineType == TEngineType::DieselElectric ) {

        if( Vehicle.diesel.heat.rpmw > 0.1 ) {
            // NOTE: fan speed tends to max out at ~100 rpm; by default we try to get pitch range of 0.5-1.5 and volume range of 0.5-1.0
            radiator_fan
                .pitch( 0.5 + radiator_fan.m_frequencyoffset + radiator_fan.m_frequencyfactor * Vehicle.diesel.heat.rpmw * 0.01 )
                .gain( 0.5 + radiator_fan.m_amplitudeoffset + 0.5 * radiator_fan.m_amplitudefactor * Vehicle.diesel.heat.rpmw * 0.01 )
                .play( sound_flags::exclusive | sound_flags::looping );
        }
        else {
            // ...otherwise shut down the sound
            radiator_fan.stop();
        }

        if( Vehicle.diesel.heat.rpmw2 > 0.1 ) {
            // NOTE: fan speed tends to max out at ~100 rpm; by default we try to get pitch range of 0.5-1.5 and volume range of 0.5-1.0
            radiator_fan_aux
                .pitch( 0.5 + radiator_fan_aux.m_frequencyoffset + radiator_fan_aux.m_frequencyfactor * Vehicle.diesel.heat.rpmw2 * 0.01 )
                .gain( 0.5 + radiator_fan_aux.m_amplitudeoffset + 0.5 * radiator_fan_aux.m_amplitudefactor * Vehicle.diesel.heat.rpmw2 * 0.01 )
                .play( sound_flags::exclusive | sound_flags::looping );
        }
        else {
            // ...otherwise shut down the sound
            radiator_fan_aux.stop();
        }
    }

    // relay sounds
    auto const soundflags { Vehicle.SoundFlag };
    if( TestFlag( soundflags, sound::relay ) ) {
        // przekaznik - gdy bezpiecznik, automatyczny rozruch itp
        if( true == TestFlag( soundflags, sound::shuntfield ) ) {
            // shunt field
            motor_shuntfield
                .pitch(
                    true == motor_shuntfield.is_combined() ?
                        Vehicle.ScndCtrlActualPos * 0.01f :
                        motor_shuntfield.m_frequencyoffset + motor_shuntfield.m_frequencyfactor * 1.f )
                .gain(
                    motor_shuntfield.m_amplitudeoffset + (
                        true == TestFlag( soundflags, sound::loud ) ?
                            1.0f :
                            0.8f )
                        * motor_shuntfield.m_amplitudefactor )
                .play();
        }
        else if( true == TestFlag( soundflags, sound::parallel ) ) {
            // parallel mode
            if( TestFlag( soundflags, sound::loud ) ) {
                dsbWejscie_na_bezoporow.play();
            }
            else {
                motor_parallel.play();
            }
        }
        else {
            // series mode
            motor_relay
                .pitch(
                    true == motor_relay.is_combined() ?
                        Vehicle.MainCtrlActualPos * 0.01f :
                        motor_relay.m_frequencyoffset + motor_relay.m_frequencyfactor * 1.f )
                .gain(
                    motor_relay.m_amplitudeoffset + (
                        true == TestFlag( soundflags, sound::loud ) ?
                            1.0f :
                            0.8f )
                        * motor_relay.m_amplitudefactor )
                .play();
        }
    }

    if( Vehicle.Vel > 0.1 ) {
        transmission
            .pitch( transmission.m_frequencyoffset + transmission.m_frequencyfactor * Vehicle.Vel )
            .gain( transmission.m_amplitudeoffset + transmission.m_amplitudefactor * Vehicle.Vel )
            .play( sound_flags::exclusive | sound_flags::looping );
    }
    else {
        transmission.stop();
    }
}
