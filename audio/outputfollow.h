/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <algorithm>
#include <chrono>

namespace audio {

// decides when the playback device has to be opened again, so the sound follows the output of the system:
// headphones pulled out or plugged in, another default output chosen, the device taken away and given back.
// holds no OpenAL state of its own: the renderer feeds it the events of the system and the state of the device,
// and does what it answers. kept apart from the renderer so the unit tests can run it
class output_follower {

public:
    using clock = std::chrono::steady_clock;

    // what the system reported since the last update, combined as bits
    enum event : unsigned {
        default_changed = 1u << 0, // another output became the default one
        device_removed = 1u << 1, // an output is gone, possibly the one in use
        device_added = 1u << 2 // an output appeared, possibly the one set in eu07.ini
    };

    enum class action {
        none,
        reopen, // open the output anew: the one set in eu07.ini if there is one and it works, the default one otherwise
        restore_preferred // playing on the default output in place of the one set in eu07.ini: see whether that one is back
    };

    // how often a working device is asked whether it is still connected. the system doesn't report every loss of the
    // output (a stream invalidated without the device being removed), so the events alone leave the game silent
    static constexpr std::chrono::milliseconds check_interval { 1000 };
    // delays between the attempts to open an output while there is none that works
    static constexpr std::chrono::milliseconds retry_min { 500 };
    static constexpr std::chrono::milliseconds retry_max { 4000 };

    // Pinned: a device is set in eu07.ini (sound.openal.renderer); Fallback: it couldn't be opened and the default one plays instead
    void
    start( bool const Pinned, bool const Fallback, clock::time_point const Now ) {
        m_pinned = Pinned;
        m_fallback = Pinned && Fallback;
        m_lost = false;
        m_retrydelay = retry_min;
        m_nextcheck = Now + check_interval;
    }

    // Events: bits of event; Connected: callable telling whether the device still works, called only when a check is due
    template <class Probe_>
    action
    update( unsigned const Events, clock::time_point const Now, Probe_ const &Connected ) {
        if( m_lost ) {
            // no working output: try again when the system reports any change, otherwise after a growing delay
            return ( Events != 0u || Now >= m_nextcheck ) ? action::reopen : action::none;
        }
        if( ( Now >= m_nextcheck ) || ( ( Events & device_removed ) != 0u ) ) {
            m_nextcheck = Now + check_interval;
            if( false == Connected() ) { return action::reopen; }
        }
        if( follows_default() && ( ( Events & default_changed ) != 0u ) ) { return action::reopen; }
        if( m_fallback && ( ( Events & device_added ) != 0u ) ) { return action::restore_preferred; }

        return action::none;
    }

    // an output was opened and works. Fallback: it is the default one, in place of the one set in eu07.ini
    void
    succeeded( bool const Fallback, clock::time_point const Now ) {
        m_fallback = m_pinned && Fallback;
        m_lost = false;
        m_retrydelay = retry_min;
        m_nextcheck = Now + check_interval;
    }

    // no output could be opened. returns true for the first failure in a row, the one worth a line in the log
    bool
    failed( clock::time_point const Now ) {
        auto const first { false == m_lost };
        m_retrydelay = first ? retry_min : std::min( m_retrydelay * 2, retry_max );
        m_lost = true;
        m_nextcheck = Now + m_retrydelay;
        return first;
    }

    bool lost() const { return m_lost; }
    bool fallback() const { return m_fallback; }
    // a device set in eu07.ini is left alone for as long as it works; otherwise the sound goes where the system sends it
    bool follows_default() const { return false == m_pinned || m_fallback; }

private:
    bool m_pinned { false };
    bool m_fallback { false };
    bool m_lost { false }; // the output is gone and none could be opened so far
    std::chrono::milliseconds m_retrydelay { retry_min };
    clock::time_point m_nextcheck {}; // of the connection while the output works, the next attempt to open one while it doesn't
};

} // namespace audio
