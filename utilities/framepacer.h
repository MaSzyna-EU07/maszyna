/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <chrono>
#include <optional>

// paces the main loop to the frame time set by fpslimit in eu07.ini.
// the frames are scheduled on a fixed cadence rather than by sleeping off what is left of each frame,
// so a sleep that overshoots (the windows timer wakes up in steps of up to ~15.6 ms) is made up by
// a shorter wait in the next frame, and the frame rate averages to the limit.
// all the time arithmetic is done in the clock's integer ticks: a sleep_for() given a float duration
// adds it to now() as float nanoseconds, which after a few hours of system uptime rounds the wake-up
// time to tens of milliseconds and lets the limit fall through (eu07/maszyna#416)
class frame_pacer {

public:
	using clock = std::chrono::steady_clock;

	// called at the end of each frame. returns the point in time the next frame may start at,
	// or nothing when it may start right away
	[[nodiscard]] std::optional<clock::time_point> next_frame(clock::time_point const Now, std::chrono::duration<float> const Frametime) {
		return next_frame(Now, std::chrono::duration_cast<clock::duration>(Frametime));
	}
	[[nodiscard]] std::optional<clock::time_point> next_frame(clock::time_point const Now, clock::duration const Minframetime) {
		if (Minframetime <= clock::duration::zero()) {
			// no limit
			m_framestart.reset();
			return std::nullopt;
		}
		if (!m_framestart) {
			// first paced frame
			m_framestart = Now;
			return std::nullopt;
		}
		auto const nextframe{*m_framestart + Minframetime};
		if (Now >= nextframe + Minframetime) {
			// a whole frame or more behind: start the cadence anew rather than run frames back to back to catch up
			m_framestart = Now;
			return std::nullopt;
		}
		m_framestart = nextframe;
		if (Now >= nextframe) {
			return std::nullopt;
		}
		return nextframe;
	}

private:
	// the scheduled start of the current frame
	std::optional<clock::time_point> m_framestart;
};
