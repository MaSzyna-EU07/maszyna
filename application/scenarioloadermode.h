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

class scenarioloader_mode : public application_mode {

    enum class load_phase {
        deserialize,
        eu7_load,
        finished
    };

	std::shared_ptr<simulation::deserializer_state> state;
	std::chrono::system_clock::time_point timestart;
    load_phase m_phase { load_phase::deserialize };
    std::chrono::steady_clock::time_point m_eu7_load_started {};
    bool m_eu7_stream_primed { false };
    float m_bar_progress { 0.f };

    void update_bar_progress( float progress );

public:
// constructors
    scenarioloader_mode();
// methods
    // initializes internal data structures of the mode. returns: true on success, false otherwise
    bool init() override;
    // mode-specific update of simulation data. returns: false on error, true otherwise
    bool update() override;
    // maintenance method, called when the mode is activated
    void enter() override;
    // maintenance method, called when the mode is deactivated
    void exit() override;
    void set_progress( float Progress = 0.f, float Subtaskprogress = 0.f ) override;
    // input handlers
    void on_key( int const Key, int const Scancode, int const Action, int const Mods ) override { ; }
    void on_cursor_pos( double const Horizontal, double const Vertical ) override { ; }
    void on_mouse_button( int const Button, int const Action, int const Mods ) override { ; }
	void on_scroll( double const Xoffset, double const Yoffset ) override { ; }
	void on_window_resize( int w, int h ) override { ; }
    void on_event_poll() override { ; }
    bool is_command_processor() const override;
};
