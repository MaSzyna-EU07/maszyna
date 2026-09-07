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
    void update_section_network( std::vector<text_line> &Output );
#ifdef WITH_UART
    void update_section_uart( std::vector<text_line> &Output );
#endif
#ifdef WITH_HARDWARE_PROTOCOL_V2
    void update_section_hardware( std::vector<text_line> &Output );
    // interactive part of the hardware section: connect, disconnect, diagnostics
    void render_section_hardware();
    // message box a controller asked the simulator to show; rendered as its own window
    void render_hardware_prompt();
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
        m_networklines,
        m_uartlines,
        m_hardwarelines;

#ifdef WITH_HARDWARE_PROTOCOL_V2
    // debug panel state for the hardware protocol section
    std::vector<int> m_hardwarefunction; // selected diagnostic function, per link
    std::vector<char const *> m_hardwareports; // port names for the combo, valid for one frame
    std::vector<std::string> m_hardwareportnames;
    int m_hardwareport { -1 };
    int m_hardwarebaud { 4 }; // 115200
#endif

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
	// kilobytes per second in and out, so the cost of a session can be read off a chart
	graph_data net_sent_graph;
	graph_data net_received_graph;
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
