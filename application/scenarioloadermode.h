/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "application/applicationmode.h"
#include "simulation/simulation.h"

#include <atomic>
#include <chrono>
#include <cstddef>
#include <exception>
#include <memory>
#include <stop_token>
#include <thread>

class scenarioloader_mode : public application_mode {
public:
	// the scenery loaded by a thread of its own (gfx_renderer::Loads_In_Background()), the main thread drawing the loading screen
	struct background_load
	{
		std::jthread thread;
		// set by the thread once it's done; what it threw, if anything
		std::atomic<bool> done{false};
		std::exception_ptr failure;
		// the frames of the loading screen drawn meanwhile, and the longest time between two of them, for the log
		std::size_t frames{0};
		std::chrono::steady_clock::time_point lastframe{std::chrono::steady_clock::now()};
		std::chrono::steady_clock::duration longestframe{};
	};

private:
	std::shared_ptr<simulation::deserializer_state> state;
	std::chrono::system_clock::time_point timestart;
	// the scenery is loaded: the loading screen kept while the renderer prepares the programs (gfx_renderer::Prewarm_Shaders())
	int m_closing{0};
	std::unique_ptr<background_load> m_background;

	// the scenery couldn't be loaded: back to the launcher, or the test ends
	bool loading_failed();
	// the scenery loaded by the thread: false while it's still loading; what the thread threw is thrown again here
	bool background_loaded();

public:
// constructors
    scenarioloader_mode();
    ~scenarioloader_mode() override;
    scenarioloader_mode(scenarioloader_mode const &) = delete;
    scenarioloader_mode &operator=(scenarioloader_mode const &) = delete;
// methods
    // the thread loading the scenery in the background stopped and waited for (the simulator closes), before what it fills goes
    void stop_loading();
    // initializes internal data structures of the mode. returns: true on success, false otherwise
    bool init() override;
    // mode-specific update of simulation data. returns: false on error, true otherwise
    bool update() override;
    // maintenance method, called when the mode is activated
    void enter() override;
    // maintenance method, called when the mode is deactivated
    void exit() override;
    // input handlers
    void on_key( int const Key, int const Scancode, int const Action, int const Mods ) override { ; }
    void on_cursor_pos( double const Horizontal, double const Vertical ) override { ; }
    void on_mouse_button( int const Button, int const Action, int const Mods ) override { ; }
	void on_scroll( double const Xoffset, double const Yoffset ) override { ; }
	void on_window_resize( int w, int h ) override { ; }
    void on_event_poll() override { ; }
    bool is_command_processor() const override;
};
