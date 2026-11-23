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
#include "simulation/simulationtest.h"
#include "application/application.h"
#include "application/scenarioloaderuilayer.h"
#include "rendering/renderer.h"
#include "utilities/Logs.h"
#include "utilities/translation.h"
#include "audio/audiorenderer.h"

#include <format>

scenarioloader_mode::scenarioloader_mode() {
    m_userinterface = std::make_shared<scenarioloader_ui>();
}

// initializes internal data structures of the mode. returns: true on success, false otherwise
bool scenarioloader_mode::init() {
    // nothing to do here
    return true;
}

// mode-specific update of simulation data. returns: false on error, true otherwise
bool scenarioloader_mode::update() {
	if (!Global.ready_to_load)
		// waiting for network connection
		return true;

	if (m_closing > 0) {
		// the programs of the materials made ready a few at a frame, the loading screen with its bar kept on the screen and the
		// window answering meanwhile; then on to the simulation
		auto const [done, total] { GfxRenderer->Prewarm_Shaders( 50.0 ) };
		if( done < total ) {
			m_userinterface->set_progress( static_cast<float>( done ) * 100.f / static_cast<float>( total ), 0.f );
			return true;
		}
		m_closing = 0;
		Application.pop_mode();
		Application.push_mode( eu07_application::mode::driver );
		return true;
	}

	if (!state) {
		WriteLog("using simulation seed: " + std::to_string(Global.random_seed), logtype::generic);
        WriteLog("using simulation starting timestamp: " + std::to_string(Global.starting_timestamp), logtype::generic);

		// the command line gives the name lowercased, the file may be named in another case, as a scenery made on Windows;
		// the scenery is saved back to its own file then
		if (auto const cased{FileCased(Global.asCurrentSceneryPath + Global.SceneryFile)}; cased.size() > Global.asCurrentSceneryPath.size())
			Global.SceneryFile = cased.substr(Global.asCurrentSceneryPath.size());
        Application.set_title( Global.AppName + " (" + Global.SceneryFile + ")" );
        WriteLog( "\nLoading scenario \"" + Global.SceneryFile + "\"..." );

		timestart = std::chrono::system_clock::now();
		try {
			state = simulation::State.deserialize_begin(Global.SceneryFile);
		}
		// the file it can't read, as the parser and the scenery opening report it; invalid_scenery_exception is one of these, and so is
		// what the file system throws
		catch (std::runtime_error const &e) {
			ErrorLog(std::format("Bad init: scenario \"{}\" couldn't be opened ({})", Global.SceneryFile, e.what()));
			state.reset();
			if (Global.test.enabled) {
				simulation::test().finish(simulation::test_result::error, "scenario couldn't be opened");
				return true;
			}
			Global.SceneryFile.clear();
			Application.pop_mode();
			Application.push_mode(eu07_application::mode::launcher);
			return true;
		}
	}

	try {
		if (simulation::State.deserialize_continue(state))
			return true;
	}
	catch (invalid_scenery_exception &e) {
		// back to the launcher as when the scenario can't be opened, not on to the driver mode with the scenery half loaded
		ErrorLog( "Bad init: scenario loading failed" );
		state.reset();
		if( Global.test.enabled ) {
			// there's nothing to test; the exit code says the run went wrong
			simulation::test().finish( simulation::test_result::error, "scenario loading failed" );
			return true;
		}
		Global.SceneryFile.clear();
		Application.pop_mode();
		Application.push_mode(eu07_application::mode::launcher);
		return true;
	}
	// the loading state holds the scenario file open, which on some systems keeps the file from being replaced
	// when the scenery is saved in the editor. it'd also get in the way of another load
	state.reset();
	// sound files decoded in the background are all ready before the simulation starts
	audio::renderer.complete_buffers();

	WriteLog( "Scenario loading time: " + std::to_string( std::chrono::duration_cast<std::chrono::seconds>( std::chrono::system_clock::now() - timestart ).count() ) + " seconds" );
	// TODO: implement and use next mode cue

	// the loading screen stays, saying the graphics are being prepared, until the programs are ready (above).
	// at the top of the screen: the bottom is taken by the trivia and by the texts some backgrounds carry. the interface
	// of this mode is always the loader's, made in the constructor
	m_userinterface->set_progress(std::string{});
	static_cast<scenarioloader_ui &>(*m_userinterface).set_notice(STR("Preparing the graphics (takes longer at the first start of a new version)"));
	m_closing = 1;

    return true;
}

bool scenarioloader_mode::is_command_processor() const {
	return false;
}

// maintenance method, called when the mode is activated
void scenarioloader_mode::enter() {
    // TBD: hide cursor in fullscreen mode?
    Application.set_cursor( GLFW_CURSOR_NORMAL );

    simulation::is_ready = false;

    Application.set_title( Global.AppName + " (" + Global.SceneryFile + ")" );
	m_userinterface->set_progress(STR("Loading scenery"));
	// the notice of a load before is gone
	static_cast<scenarioloader_ui &>(*m_userinterface).set_notice({});
}

// maintenance method, called when the mode is deactivated
void scenarioloader_mode::exit() {
    simulation::Time.init( Global.starting_timestamp );
    simulation::Environment.init();
}
