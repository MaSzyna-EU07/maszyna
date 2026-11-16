/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <optional>

#include "application/uilayer.h"
#include "application/drivingaid.h"
#include "utilities/Classes.h"
#include "rendering/renderer.h"

// what holds the train up, as the driving aid tells it (translated): the exchange of passengers or load, else a vehicle close
// ahead. starts with a space, empty: nothing
std::string driving_aid_hold_up( driving_aid::reading const &Aid );

class drivingaid_panel : public ui_expandable_panel {

public:
    drivingaid_panel( std::string const &Name, bool const Isopen )
        : ui_expandable_panel( Name, Isopen )
    {}

    void update() override;
    void render() override;
    void render_contents() override;

    // what the driving aid shows, read from the vehicle driven; none without one. the hud of RmlUi reads it too
    static std::optional<driving_aid::reading> read();

    // the hud of RmlUi shows the driving aid, the panel stays hidden; is_open and is_expanded still tell what the hud shows
    bool replaced { false };

private:
    // the limit of the timetable or of the shunting the driver keeps to, -1: none
    static int schedule_speed_limit( TController const &Owner );
    static driving_aid::speed_limit_inputs speed_limit_inputs_of( TController const &Owner );
};

// the state of a stop of the timetable worth a colour of its own: the first stop ahead while the passengers or the load are
// exchanged, when the train can depart, or when it stands at the stop
enum class timetable_stop_state { normal, exchange, can_depart, at_stop };

// what the panel of the timetable shows (F2), of the vehicle driven or the one the free camera is at; the hud of RmlUi reads it too
struct timetable_reading {
    struct stop {
        double vmax { 0.0 }; // km/h
        bool vmax_changed { true }; // differs from the one of the stop before
        std::string station; // as the timetable has it: in Windows-1250, with '_' for spaces
        double km { 0.0 }; // 0: not given
        std::string ware; // what the station has (StationWare)
        std::string arrival; // hh:mm, empty when the train passes through
        std::string departure;
        std::string travel; // minutes from the stop before, three characters
        int tracks { 1 }; // of the line to the next stop
        timetable_stop_state state { timetable_stop_state::normal }; // of the first stop ahead
    };
    std::string relation; // as the timetable has it
    std::string train;
    std::string next_stop;
    bool consist { false }; // the mass and the length of the consist are told (a train)
    int mass_specified { 0 }; // t
    int mass_actual { 0 }; // t, the active locomotives taken off
    int length { 0 }; // m
    bool timetable { false }; // the train has a timetable; its stops ahead are in stops
    int delay { 0 }; // minutes the train is late by, as the driver noted it at the last station (CheckTrainLatency)
    std::vector<stop> stops;
};

class timetable_panel : public ui_expandable_panel {

public:
    timetable_panel( std::string const &Name, bool const Isopen )
        : ui_expandable_panel( Name, Isopen ) {}

    void update() override;
    void render() override;

    // what the panel shows, none without a vehicle and its driver
    static std::optional<timetable_reading> read();

    // the hud of RmlUi shows the timetable, the panel stays hidden; is_open and is_expanded still tell what the hud shows
    bool replaced { false };

private:
    static void read_stops( timetable_reading &Reading, TController const &Owner, TDynamicObject const &Vehicle );
    // members
    std::array<char, 256> m_buffer;
    std::vector<text_line> m_tablelines;
};

// what the panel of the scenario shows (F3), of the vehicle driven or the one the free camera is at
struct scenario_reading {
    struct hint {
        std::string text; // translated
        bool done { false };
    };
    std::string task; // the current task of the driver
    std::string assignment; // of the scenario, empty: none
    std::vector<hint> hints;
};

class scenario_panel : public ui_panel {

public:
    scenario_panel( std::string const &Name, bool const Isopen )
        : ui_panel( Name, Isopen ) {}

    void update() override;
    void render() override;

    // what the panel shows, none without a vehicle and its driver
    static std::optional<scenario_reading> read();

    bool is_expanded{ false };
    // the hud of RmlUi shows the scenario, the panel stays hidden; is_open still tells whether the hud shows it
    bool replaced { false };

private:
// members
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
    void update_section_engine( std::vector<text_line> &Output ) const;
    void update_section_ai( std::vector<text_line> &Output ) const;
    void update_section_scantable( std::vector<text_line> &Output ) const;
    void update_section_scenario( std::vector<text_line> &Output ) const;
    void update_section_eventqueue( std::vector<text_line> &Output );
    void update_section_powergrid( std::vector<text_line> &Output ) const;
    void update_section_camera( std::vector<text_line> &Output ) const;
    void update_section_renderer( std::vector<text_line> &Output );
#ifdef WITH_UART
    void update_section_uart( std::vector<text_line> &Output ) const;
#endif
    // section update helpers
    std::string update_vehicle_coupler( int const Side );
    std::string update_vehicle_brake() const;
    // renders provided lines, under specified collapsing header
    bool render_section( std::string const &Header, std::vector<text_line> const &Lines );
    bool render_section( std::vector<text_line> const &Lines ) const;
    bool render_section_scenario();
    bool render_section_eventqueue();
    bool render_section_renderer();
    bool render_section_settings() const;
	bool render_section_developer() const;
	    // members
    std::array<char, 1024> m_buffer;
    std::array<char, 128> m_eventsearch;
    input_data m_input;
    std::vector<text_line> m_vehiclelines;
    std::vector<text_line> m_enginelines;
    std::vector<text_line> m_ailines;
    std::vector<text_line> m_scantablelines;
    std::vector<text_line> m_cameralines;
    std::vector<text_line> m_scenariolines;
    std::vector<text_line> m_eventqueuelines;
    std::vector<text_line> m_powergridlines;
    std::vector<text_line> m_rendererlines;
    std::vector<gfx_renderer::info_section> m_renderersections; // the data of the renderer, each part under a header of its own
    std::vector<text_line> m_uartlines;

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


    bool m_eventqueueactivevehicleonly { false };
};

class transcripts_panel : public ui_panel {

public:
    transcripts_panel( std::string const &Name, bool const Isopen )
        : ui_panel( Name, Isopen ) {}

    void update() override;
	void render() override;
};
