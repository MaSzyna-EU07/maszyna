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

// every frame advances the clocks by this many seconds, whatever the real time; 0 = real time (test mode)
void set_fixed_step(double v);

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
    // work done in many pieces over a frame (the AI of every vehicle): resume() and pause() around each piece,
    // commit() once a frame to take their sum as the frame's time
    void
        resume() {
            m_start = std::chrono::steady_clock::now(); }
    void
        pause() {
            m_pending += std::chrono::duration_cast<std::chrono::microseconds>( std::chrono::steady_clock::now() - m_start ); }
    void
        commit() {
            m_last = m_pending;
            m_pending = std::chrono::microseconds::zero();
            m_accumulator = 0.95f * m_accumulator + static_cast<float>(m_last.count()) / 1000.f; }

private:
// members
    std::chrono::time_point<std::chrono::steady_clock> m_start { std::chrono::steady_clock::now() };
    float m_accumulator { 1000.f / 30.f * 20.f }; // 20 last samples, initial 'neutral' rate of 30 fps
    std::chrono::microseconds m_last;
    std::chrono::microseconds m_pending { 0 }; // the pieces summed since the last commit()
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
    stopwatch mainloop_total;
    stopwatch mainloop_period; // start to start, the wait for fpslimit included
};

extern subsystem_stopwatches subsystem;

};

//---------------------------------------------------------------------------
