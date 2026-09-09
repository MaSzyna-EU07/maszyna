module;
#include <array>
#include <cstddef>
#include <deque>
#include <limits>
#include <mutex>
#include <sstream>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>
#include <ostream>
#include <memory>
#include <GLFW/glfw3.h>
#include <chrono>
#include <string>
#include <typeinfo>
#include "utilities/Globals_macros.h"
#include "utilities/translation_macros.h"

module eu07.application.scenarioloadermode;
import eu07.utilities.classes;
import eu07.utilities.globals;
import eu07.simulation.simulation;
import eu07.simulation.simulationtime;
import eu07.simulation.simulationenvironment;
import eu07.application.application;
import eu07.application.scenarioloaderuilayer;
import eu07.rendering.renderer;
import eu07.utilities.logs;
import eu07.utilities.translation;
/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/



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

	if (!state) {
		WriteLog("using simulation seed: " + std::to_string(Global.random_seed), logtype::generic);
        WriteLog("using simulation starting timestamp: " + std::to_string(Global.starting_timestamp), logtype::generic);

        Application.set_title( Global.AppName + " (" + Global.SceneryFile + ")" );
        WriteLog( "\nLoading scenario \"" + Global.SceneryFile + "\"..." );

		timestart = std::chrono::system_clock::now();
		state = simulation::State.deserialize_begin(Global.SceneryFile);
	}

	try {
		if (simulation::State.deserialize_continue(state))
			return true;
	}
	catch (invalid_scenery_exception &e) {
		ErrorLog( "Bad init: scenario loading failed" );
		Application.pop_mode();
	}

	WriteLog( "Scenario loading time: " + std::to_string( std::chrono::duration_cast<std::chrono::seconds>( std::chrono::system_clock::now() - timestart ).count() ) + " seconds" );
	// TODO: implement and use next mode cue

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

    simulation::is_ready = false;

    Application.set_title( Global.AppName + " (" + Global.SceneryFile + ")" );
	m_userinterface->set_progress(STR("Loading scenery"));
}

// maintenance method, called when the mode is deactivated
void scenarioloader_mode::exit() {
    simulation::Time.init( Global.starting_timestamp );
    simulation::Environment.init();
}
