/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/scenarioloadermode.h"

#include "utilities/Globals.h"
#include "simulation/simulation.h"
#include "simulation/simulationtime.h"
#include "simulation/simulationenvironment.h"
#include "application/application.h"
#include "application/scenarioloaderuilayer.h"
#include "rendering/renderer.h"
#include "utilities/Logs.h"
#include "utilities/translation.h"
#include "scene/eu7/eu7_load_stats.h"
#include "scene/eu7/eu7_pack_bench.h"
#include "scene/eu7/eu7_section_stream.h"

namespace {
constexpr std::chrono::seconds kEu7LoadTimeout { 90 };
constexpr float kDeserializeBarShare { 70.f };
constexpr float kPackBarShare { 30.f };
} // namespace

scenarioloader_mode::scenarioloader_mode() {
    m_userinterface = std::make_shared<scenarioloader_ui>();
}

// initializes internal data structures of the mode. returns: true on success, false otherwise
bool scenarioloader_mode::init() {
    // nothing to do here
    return true;
}

void scenarioloader_mode::update_bar_progress( float const progress ) {
    m_bar_progress = std::max( m_bar_progress, std::clamp( progress, 0.f, 100.f ) );
    if( m_userinterface != nullptr ) {
        m_userinterface->set_progress( m_bar_progress, 0.f );
    }
}

// mode-specific update of simulation data. returns: false on error, true otherwise
bool scenarioloader_mode::update() {
	if (!Global.ready_to_load)
		// waiting for network connection
		return true;

	if (!state) {
		WriteLog("using simulation seed: " + std::to_string(Global.random_seed), logtype::generic);
        WriteLog("using simulation starting timestamp: " + std::to_string(Global.starting_timestamp), logtype::generic);

        Application.set_title( Global.AppName + " (" + Global.SceneryFile + ")" );
        WriteLog( "\nLoading scenario \"" + Global.SceneryFile + "\"..." );

		timestart = std::chrono::system_clock::now();
		state = simulation::State.deserialize_begin(Global.SceneryFile);
	}

	try {
		if( m_phase == load_phase::deserialize ) {
			if( simulation::State.deserialize_continue( state ) ) {
				return true;
			}
			m_phase = load_phase::eu7_load;
			m_eu7_load_started = std::chrono::steady_clock::now();
			m_eu7_stream_primed = false;
		}
	}
	catch (invalid_scenery_exception &e) {
		ErrorLog( "Bad init: scenario loading failed" );
		Application.pop_mode();
        return true;
	}

	if( m_phase == load_phase::eu7_load ) {
		if( scene::eu7::section_stream_active() ) {
			simulation::State.drain_deferred_eu7_trainsets( 16.0 );

			auto const position { scene::eu7::stream_loading_position() };

			if( false == m_eu7_stream_primed ) {
				if( scene::eu7::section_stream_needs_bootstrap() ) {
					scene::eu7::kick_section_stream_bootstrap();
				}
				m_eu7_stream_primed = true;
			}

			scene::eu7::drain_section_stream();

			auto const ring_progress {
				scene::eu7::section_stream_ring_progress(
					position,
					scene::eu7::kSectionStreamBootstrapRadiusKm ) };
			update_bar_progress(
				kDeserializeBarShare + ring_progress * kPackBarShare );

			auto const timed_out {
				std::chrono::steady_clock::now() - m_eu7_load_started >= kEu7LoadTimeout };
			auto const bootstrap_ready {
				scene::eu7::section_stream_ready_around(
					position,
					scene::eu7::kSectionStreamBootstrapRadiusKm ) };
			if(
				bootstrap_ready
				|| scene::eu7::section_stream_presentable_around(
					position,
					scene::eu7::kSectionStreamBootstrapRadiusKm ) ) {
				WriteLog( "EU7 PACK: pierścień wokół pozycji startowej gotowy do pokazania" );
				scene::eu7::dismiss_loading_screen();
				m_phase = load_phase::finished;
			}
			else if( timed_out ) {
				ErrorLog(
					"EU7 PACK: timeout ładowania pierścienia wokół kamery — wchodzę w jazdę (ring=" +
					std::to_string( static_cast<int>( ring_progress * 100.f ) ) + "%)" );
				scene::eu7::dismiss_loading_screen();
				m_phase = load_phase::finished;
			}
			else {
				return true;
			}
		}
		else {
			scene::eu7::dismiss_loading_screen();
			m_phase = load_phase::finished;
		}
	}

	WriteLog( "Scenario loading time: " + std::to_string( std::chrono::duration_cast<std::chrono::seconds>( ( std::chrono::system_clock::now() - timestart ) ).count() ) + " seconds" );
	scene::eu7::log_load_stats();
	scene::eu7::log_pack_bench();

	update_bar_progress( 100.f );
	Application.pop_mode();
	Application.push_mode( eu07_application::mode::driver );

    return true;
}

bool scenarioloader_mode::is_command_processor() const {
	return false;
}

// maintenance method, called when the mode is activated
void scenarioloader_mode::enter() {
    // TBD: hide cursor in fullscreen mode?
    Application.set_cursor( GLFW_CURSOR_NORMAL );

    m_phase = load_phase::deserialize;
    m_eu7_stream_primed = false;
    m_bar_progress = 0.f;
    simulation::is_ready = false;

    Application.set_title( Global.AppName + " (" + Global.SceneryFile + ")" );
	m_userinterface->set_progress(STR("Loading scenery"));
}

// maintenance method, called when the mode is deactivated
void scenarioloader_mode::exit() {
    simulation::Time.init( Global.starting_timestamp );
    simulation::Environment.init();
}

void scenarioloader_mode::set_progress(
    float const Progress,
    float const Subtaskprogress ) {
    if( m_phase != load_phase::deserialize ) {
        return;
    }

    auto const parser_progress {
        std::max( Progress, Subtaskprogress ) };
    update_bar_progress( parser_progress * kDeserializeBarShare / 100.f );
}
