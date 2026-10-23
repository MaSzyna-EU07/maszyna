/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

namespace Timer {

double GetTime();
double GetRenderTime();

double GetDeltaTime();
double GetDeltaRenderTime();

void set_delta_override(double v);

void ResetTimers();

void UpdateTimers(bool pause);

class stopwatch {

public:
// constructors
    stopwatch() = default;
// methods
    void
        start() {
            m_start = std::chrono::steady_clock::now(); }
	std::chrono::duration<float, std::milli>
        stop() {
		    m_last = std::chrono::duration_cast<std::chrono::microseconds>( std::chrono::steady_clock::now() - m_start );
			m_accumulator = 0.95f * m_accumulator + static_cast<float>(m_last.count()) / 1000.f;
			return m_last; }
    float
        average() const {
            return m_accumulator / 20.f;}
	std::chrono::microseconds
	    last() const {
		    return m_last; }

private:
// members
    std::chrono::time_point<std::chrono::steady_clock> m_start { std::chrono::steady_clock::now() };
    float m_accumulator { 1000.f / 30.f * 20.f }; // 20 last samples, initial 'neutral' rate of 30 fps
    std::chrono::microseconds m_last;
};

// time of a part made many times in a frame (for each vehicle, for each step of the physics): the times are added up in the
// frame, and the sums averaged over the frames the way stopwatch averages them. commit() ends the frame
class frame_stopwatch {

public:
    void
        start() {
            m_start = std::chrono::steady_clock::now(); }
    void
        stop() {
            m_frame += std::chrono::duration<float, std::milli>( std::chrono::steady_clock::now() - m_start ).count(); }
    void
        add( float const Milliseconds ) {
            m_frame += Milliseconds; }
    void
        commit() {
            m_accumulator = 0.95f * m_accumulator + m_frame;
            m_frame = 0.f; }
    float
        average() const {
            return m_accumulator / 20.f; }

private:
    std::chrono::time_point<std::chrono::steady_clock> m_start { std::chrono::steady_clock::now() };
    float m_frame { 0.f };
    float m_accumulator { 0.f };
};

struct subsystem_stopwatches {
    stopwatch gfx_total;
    stopwatch gfx_color;
    stopwatch gfx_shadows;
    stopwatch gfx_reflections;
    stopwatch gfx_swap;
    stopwatch gfx_gui;
    stopwatch gfx_animate;
    stopwatch sim_total;
    stopwatch sim_dynamics;
    stopwatch sim_events;
    stopwatch sim_ai;
    // parts of sim_total for the debug panel, measured in driver_mode::update()
    stopwatch sim_scripting; // clocks and the scripting interface
    stopwatch sim_environment; // sky and weather
    stopwatch sim_precipitation;
    stopwatch sim_secondary; // secondary fixed step routines
    stopwatch sim_trains; // cabs of the trains
    stopwatch sim_lights;
    stopwatch sim_commands;
    stopwatch sim_ui; // 50 Hz routines: ui panels, tooltips
    stopwatch sim_camera;
    // parts of sim_dynamics, simulation::state_manager::update() and vehicle_table::update(), made a few times in a frame
    frame_stopwatch sim_animations; // queued animations of the scenery models
    frame_stopwatch sim_powergrid;
    frame_stopwatch sim_roads; // road junctions, level crossings, spawn points of road vehicles
    frame_stopwatch vehicles_prepare; // traction wires, constants, neighbours
    frame_stopwatch vehicles_forces; // forces and the steps of the physics
    frame_stopwatch vehicles_update; // TDynamicObject::Update(), with the ai in it
    frame_stopwatch vehicles_ai; // TController::Update() of the vehicles with a driver
    // the forces and the update by the kind of the vehicle: rail within VEHICLE_NEAR of the camera, rail farther, road within,
    // road farther
    static constexpr double VEHICLE_NEAR { 1000.0 };
    std::array<frame_stopwatch, 4> vehicles_forces_kind;
    std::array<frame_stopwatch, 4> vehicles_update_kind;
    // what the vehicle update went through in the last frame: vehicles, of them enabled, road vehicles, with a driver run by the
    // ai, with the physics active; and the steps of the physics
    struct vehicle_counts {
        unsigned int all { 0 };
        unsigned int enabled { 0 };
        unsigned int road { 0 };
        unsigned int ai { 0 };
        unsigned int active { 0 };
        unsigned int steps { 0 };
        // enabled vehicles of each kind (see vehicles_forces_kind), and of them the ones standing still
        std::array<unsigned int, 4> kind {};
        std::array<unsigned int, 4> standing {};
    };
    vehicle_counts vehicles; // of the frame under way
    vehicle_counts vehicleslast; // of the last frame
    // ends the frame of the frame_stopwatches and of the counts
    void
        commit_frame() {
            for( auto *part : { &sim_animations, &sim_powergrid, &sim_roads, &vehicles_prepare, &vehicles_forces, &vehicles_update, &vehicles_ai } ) {
                part->commit(); }
            for( auto &part : vehicles_forces_kind ) {
                part.commit(); }
            for( auto &part : vehicles_update_kind ) {
                part.commit(); }
            vehicleslast = vehicles;
            vehicles = {}; }
    stopwatch mainloop_total;
    // parts of the main loop outside the simulation and Render(), for the debug panel
    stopwatch loop_input; // events, head tracking
    stopwatch loop_ui_frame; // start of the ui frame
    stopwatch loop_audio; // sounds of the scenery and the audio renderer
    stopwatch loop_particles; // cpu particles
    stopwatch loop_renderer_update; // gfx_renderer::Update()
    stopwatch loop_terrain; // terrain streaming
    stopwatch loop_tasks; // task queue
};

extern subsystem_stopwatches subsystem;

};

//---------------------------------------------------------------------------
