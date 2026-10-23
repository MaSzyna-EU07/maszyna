/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <array>
#include <chrono>
#include <string>
#include <unordered_map>

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

// what resets TMoverParameters::LastSwitchingTime, which keeps the physics of a vehicle on for 5 s: counted over a second,
// with the vehicle which did it most often, for the debug panel
class switching_counter {

public:
    enum source : std::size_t {
        compressor_on = 0,
        compressor_off,
        mains,
        diesel,
        // switch_physics( true ) of a vehicle whose physics was off, by TMoverParameters::wake_cause, in its order
        wake,
        wake_mainpipe,
        wake_scndpipe,
        wake_command,
        wake_speed,
        wake_acceleration,
        wake_neighbour,
        wake_rest,
        count
    };
    struct result {
        std::array<unsigned int, source::count> events {};
        std::string busiest; // type and name of the vehicle with the most events
        unsigned int busiestevents { 0 };
    };
    // Owner: the vehicle, Label: its type and name, read only when the second ends
    void
        note( source const Source, void const *Owner, std::string const *Type, std::string const *Name ) {
            ++m_events[ Source ];
            auto &owner { m_owners[ Owner ] };
            ++owner.events;
            owner.type = Type;
            owner.name = Name; }
    // ends the second when it's over. to be called once a frame, while the vehicles noted in it still exist
    void
        update() {
            auto const now { std::chrono::steady_clock::now() };
            if( now - m_start < std::chrono::seconds( 1 ) ) { return; }
            m_last = {};
            m_last.events = m_events;
            for( auto const &owner : m_owners ) {
                if( owner.second.events > m_last.busiestevents ) {
                    m_last.busiestevents = owner.second.events;
                    m_last.busiest = ( owner.second.type ? *owner.second.type : std::string{} ) + " " + ( owner.second.name ? *owner.second.name : std::string{} ); } }
            m_events = {};
            m_owners.clear();
            m_start = now; }
    // of the last whole second
    result const &
        last() const {
            return m_last; }

private:
    struct owner_data {
        unsigned int events { 0 };
        std::string const *type { nullptr };
        std::string const *name { nullptr };
    };
    std::chrono::time_point<std::chrono::steady_clock> m_start { std::chrono::steady_clock::now() };
    std::array<unsigned int, source::count> m_events {};
    std::unordered_map<void const *, owner_data> m_owners;
    result m_last;
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
        // rail vehicles standing still (below 0.01 km/h) with the physics on, and what keeps it on by the test of
        // TMoverParameters::ComputeTotalForce(); a vehicle can have several of these
        struct awake_reasons {
            unsigned int vehicles { 0 };
            unsigned int cab { 0 }; // CabActive
            unsigned int speed { 0 }; // Vel above 0.0001 km/h
            unsigned int acceleration { 0 }; // AccS above 0.0001 m/s2, and not held by the brakes (TMoverParameters::accelerating())
            unsigned int switching { 0 }; // LastSwitchingTime below 5 s
            unsigned int multipleunit { 0 }; // EZT, DMU
            unsigned int neighbour { 0 }; // a neighbour moving or accelerating
        } awake;
    };
    vehicle_counts vehicles; // of the frame under way
    switching_counter switching;
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
