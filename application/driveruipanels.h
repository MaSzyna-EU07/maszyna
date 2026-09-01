/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "application/uilayer.h"
#include "utilities/Classes.h"

// external HUD configuration (hud.ini next to the executable; missing keys keep defaults)
namespace hudcfg {

struct settings {
    bool enabled { true };
    // bottom-right main panel
    int panel_width { 380 };
    int panel_height { 374 };
    int margin { 16 };             // distance from screen right/bottom edges
    float speed_size { 110.0f };   // big speed digits size
    int panel_x { -1 };            // free-position override; -1 = corner anchored
    int panel_y { -1 };
    // top signal strip
    int sig_width { 320 };
    int sig_height { 66 };
    int sig_top { 6 };                 // distance from screen top
    float sig_digit_size { 58.0f };    // limit number size
    float sig_digit_left { 56.0f };    // limit number left offset
    float sig_square_margin { 10.0f }; // signal colour block offset from window edge
    float sig_square_size { 34.0f };   // signal colour block side length
    float sig_text_left { 170.0f };    // right-hand info text column
    float sig_text_top { 16.0f };
    int sig_x { -1 };              // free-position override; -1 = top-centred
    int sig_y { -1 };
};

void load();
void save();
settings const &get();
// shared live visibility (used by the key binding, the Driving Aid checkbox and the HUD panels)
void set_panels( ui_panel *Panel, ui_panel *SignalPanel );
bool visible();
void set_visible( bool Show );
void toggle();
// drag state: while dragging the update() anchor is suspended
bool dragging();
void set_dragging( bool Drag );
// free positioning (called by the drag handles)
void set_panel_pos( int X, int Y );
void set_signal_pos( int X, int Y );

}

class timetable_panel : public ui_expandable_panel {

public:
    timetable_panel( std::string const &Name, bool const Isopen )
        : ui_expandable_panel( Name, Isopen ) {}

    void update() override;
    void render() override;

private:
    // members
    std::array<char, 256> m_buffer;
    std::vector<text_line> m_tablelines;
};

class scenario_panel : public ui_panel {

public:
    scenario_panel( std::string const &Name, bool const Isopen )
        : ui_panel( Name, Isopen ) {}

    void update() override;
    void render() override;

    bool is_expanded{ false };

private:
// members
    std::array<char, 256> m_buffer;
	TDynamicObject const *m_nearest { nullptr };
};

class debug_panel : public ui_panel {

public:
    debug_panel( std::string const &Name, bool const Isopen )
        : ui_panel( Name, Isopen ) {
        m_eventsearch.fill( 0 ); }

    void update() override;
	void render() override;

private:
//  types
    struct input_data {
        TTrain const *train;
        TDynamicObject const *controlled;
        TCamera const *camera;
        TDynamicObject const *vehicle;
        TMoverParameters *mover;
        TController const *mechanik;
    };
// methods
    // generate and send section data to provided output
    void update_section_vehicle( std::vector<text_line> &Output );
    void update_section_engine( std::vector<text_line> &Output );
    void update_section_ai( std::vector<text_line> &Output );
    void update_section_scantable( std::vector<text_line> &Output );
    void update_section_scenario( std::vector<text_line> &Output );
    void update_section_eventqueue( std::vector<text_line> &Output );
    void update_section_powergrid( std::vector<text_line> &Output );
    void update_section_camera( std::vector<text_line> &Output );
    void update_section_renderer( std::vector<text_line> &Output );
#ifdef WITH_UART
    void update_section_uart( std::vector<text_line> &Output );
#endif
    // section update helpers
    std::string update_vehicle_coupler( int const Side );
    std::string update_vehicle_brake() const;
    // renders provided lines, under specified collapsing header
    bool render_section( std::string const &Header, std::vector<text_line> const &Lines );
    bool render_section( std::vector<text_line> const &Lines );
    bool render_section_scenario();
    bool render_section_eventqueue();
    bool render_section_settings();
	bool render_section_developer();
	    // members
    std::array<char, 1024> m_buffer;
    std::array<char, 128> m_eventsearch;
    input_data m_input;
    std::vector<text_line>
        m_vehiclelines,
        m_enginelines,
        m_ailines,
        m_scantablelines,
        m_cameralines,
        m_scenariolines,
        m_eventqueuelines,
        m_powergridlines,
        m_rendererlines,
        m_uartlines;

	double last_time = std::numeric_limits<double>::quiet_NaN();

	struct graph_data
	{
		double last_val = 0.0;
		std::array<float, 150> data = { 0.0f };
		size_t pos = 0;
		float range = 25.0f;

		void update(float data);
		void render();
	};
	graph_data AccN_jerk_graph;
	graph_data AccN_acc_graph;
	float last_AccN;

	std::array<char, 128> queue_event_buf = { 0 };
	std::array<char, 128> queue_event_activator_buf = { 0 };

    bool m_eventqueueactivevehicleonly { false };
};

class transcripts_panel : public ui_panel {

public:
    transcripts_panel( std::string const &Name, bool const Isopen )
        : ui_panel( Name, Isopen ) {}

    void update() override;
	void render() override;
};

// heads-up display: semi-transparent overlay with big speed readout, direction arrows and gradient triangles.
// drawn as a borderless, non-interactive, corner-anchored panel.
class hud_panel : public ui_panel {

public:
    hud_panel( std::string const &Name, bool const Isopen )
        : ui_panel( Name, Isopen )
    {
        no_title_bar = true;
        window_flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar
                     | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing;
    }

    void update() override;
    void render_contents() override;

private:
// members
    glm::vec4 m_speedcolor { 0.70f, 0.88f, 1.00f, 1.00f };
};

// top-of-screen signal preview + speed limit strip; pops and flashes when the limit changes
class hud_signal_panel : public ui_panel {

public:
    hud_signal_panel( std::string const &Name, bool const Isopen )
        : ui_panel( Name, Isopen )
    {
        no_title_bar = true;
        window_flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar
                     | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing;
    }

    void update() override;
    void render_contents() override;

private:
// members
    int m_prevlimit { -1 };
    float m_flash { 0.0f };
};
