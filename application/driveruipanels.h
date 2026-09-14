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
#include "utilities/Globals.h"
#include "model/AnimModel.h"

class TDynamicObject;
class TController;

// HUD overlay configuration; the layout values live in Global.gui_hud (in-code defaults,
// updated at runtime while dragging), while the "gui.hud.enabled" switch alone is read
// from the existing config file (eu07.ini) by the common config parser
namespace hudcfg {

using settings = global_settings::hud_config;

// HUD display modes (F1 cycles): 0 = Standard (default set), 1 = Custom (per-item checkboxes),
// 2 = Off. Both panels can also be switched on/off as groups, but only in Custom mode
enum hud_mode : int {
    standard = 0,
    custom = 1,
    off = 2,
};

// vehicle-dependent capabilities used to grey out items the current vehicle does not have
struct vehicle_caps {
    bool ed        { true };  // dedicated ED handle (SplitEDPneumaticBrake, EP09 excluded)
    bool shunt     { true };  // field-weakening controller (DC series with SCPN)
    bool doors     { true };  // passenger doors (EMU/DMU)
    bool speedctrl { true };  // cruise control (tempomat) fitted
};

// registry of every displayable HUD data item. stable string ids are used for persistence
// (eu07.ini) and as the extension point for future developers: adding a row here makes the
// datum appear in the customisation window and become togglable - nothing else is needed
struct hud_item {
    char const *id;        // stable key; persisted as gui.hud.custom "<id>,<id>,..."
    char const *name;      // display name; STR_C() translation key (falls back to the literal)
    int         group;     // 0 = main panel, 1 = top signal strip, 2 = speed panel
    bool        default_on;
};

settings const &get();
// shared live visibility (used by the key binding, the menu entry and the HUD panels)
void set_panels( ui_panel *Panel, ui_panel *SignalPanel, ui_panel *SpeedPanel );
// the customisation window; F1 entering the Custom mode opens it automatically
void set_custom_window( ui_panel *Panel );
bool visible();
void set_visible( bool Show );
void toggle();
// display modes; F1 cycles, persisted as gui.hud.mode
int mode();
int mode_count();
void set_mode( int Mode );
void cycle_mode();
char const *mode_name( int Mode );
// top strip / main panel / speed panel group switches (only effective in Custom mode)
bool panel_group();
bool strip_group();
bool speed_group();
void set_panel_group( bool On );
void set_strip_group( bool On );
void set_speed_group( bool On );
// mode-name feedback toast in seconds (drawn on the HUD panel while > 0)
float toast();
void update_toast( float DeltaTime );
// item registry lookup / effective visibility for the current mode
hud_item const *item( char const *Id );
int item_count();
hud_item const &item_by_index( int Index );
bool item_visible( char const *Id );
// vehicle-dependent availability (false = the vehicle has no such system):
// e.g. EU07 has no ED handle, so the "Dynamic brake" item is greyed out in the window
bool item_available( char const *Id, vehicle_caps const &Caps );
// Custom-mode checkbox state; writes back to gui.hud.custom
bool custom_checked( char const *Id );
void set_custom_item( char const *Id, bool Checked );
// effective main-panel height, stretched by the amount of visible data (min = speed zone only)
int main_panel_height();
// drag state: while dragging the update() anchor is suspended
bool dragging();
void set_dragging( bool Drag );
// free positioning (called by the drag handles)
void set_panel_pos( int X, int Y );
void set_signal_pos( int X, int Y );
void set_speed_pos( int X, int Y );

}

class drivingaid_panel : public ui_expandable_panel {

public:
    drivingaid_panel( std::string const &Name, bool const Isopen )
        : ui_expandable_panel( Name, Isopen )
    {}

    void update() override;

private:
// members
    std::array<char, 256> m_buffer;
};

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
        window_flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar
                     | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing;
    }

    // custom render: auto-sized by content until the player grabs the resize grip
    void render() override;
    void update() override;
    void render_contents() override;

private:
// members
    glm::vec4 m_speedcolor { 0.70f, 0.88f, 1.00f, 1.00f };
    bool m_manual_size { false }; // true once the player resizes the panel (no auto-height then)
};

// split-out speed panel: big speed digits + direction arrows + gradient triangles
// (kept at the position where the speed zone used to sit, below the data panel);
// resizable via the bottom-right grip, contents scale with the window size
class hud_speed_panel : public ui_panel {

public:
    hud_speed_panel( std::string const &Name, bool const Isopen )
        : ui_panel( Name, Isopen )
    {
        no_title_bar = true;
        window_flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar
                     | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing;
    }

    // custom render: auto-sized until the player grabs the resize grip
    void render() override;
    void update() override;
    void render_contents() override;

private:
// members
    glm::vec4 m_speedcolor { 0.70f, 0.88f, 1.00f, 1.00f };
    bool m_manual_size { false }; // true once the player resizes the panel
};

// top-of-screen signal preview + speed limit strip; pops and flashes when the limit changes
class hud_signal_panel : public ui_panel {

public:
    hud_signal_panel( std::string const &Name, bool const Isopen )
        : ui_panel( Name, Isopen )
    {
        no_title_bar = true;
        window_flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar
                     | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing;
    }

    // custom render: auto-sized by content until the player grabs the resize grip
    void render() override;
    void update() override;
    void render_contents() override;

private:
    // top strip: speed limits / distance / passenger / CA-SHP (AI speed table only)
    // content height for the current display state (limit rows + banners)
    int base_height() const;
// members
    int m_prevlimit { -1 };
    float m_flash { 0.0f };
    bool m_manual_size { false }; // true once the player resizes the strip (no auto-height then)
    // signal display lock (object lock: live re-reads until the signal is passed)
};

// custom HUD configuration window: same style as the other internal windows (draggable,
// closable with the X). closing it does NOT hide the HUD - the overlay keeps showing per
// the checked items until F1 is pressed again
class hud_custom_panel : public ui_panel {

public:
    hud_custom_panel( std::string const &Name, bool const Isopen )
        : ui_panel( Name, Isopen )
    {
        size_min = { 340, 240 };
        size_max = { 560, 1000000 };
    }

    void update() override;
    void render_contents() override;
};
