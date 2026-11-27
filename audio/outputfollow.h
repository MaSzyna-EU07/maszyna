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
    // delays between the attempts to open an output while there is none that works. every attempt runs on the main
    // thread and may take tens of milliseconds, so after the first minute without an output they thin out
    static constexpr std::chrono::milliseconds retry_min { 500 };
    static constexpr std::chrono::milliseconds retry_max { 4000 };
    static constexpr std::chrono::milliseconds retry_late_after { 60000 };
    static constexpr std::chrono::milliseconds retry_late { 30000 };
    // while there is no output an event of the system has the next attempt made at once, but not sooner than this after
    // the last one: a device coming (a bluetooth headset, say) reports a burst of events over a few frames
    static constexpr std::chrono::milliseconds event_retry_min { 250 };
    // the device set in eu07.ini can be reported before it is ready to play. a move back to it that didn't work out is
    // tried again after 1, 2 and 4 s; then the next arrival of a device is waited for, so a device that never works
    // (taken in exclusive mode by another program, say) doesn't break the sound every few seconds
    static constexpr std::chrono::milliseconds restore_retry_min { 1000 };
    static constexpr int restore_retries { 3 };

    // Pinned: a device is set in eu07.ini (sound.openal.renderer); Fallback: it couldn't be opened and the default one plays instead
    void
    start( bool const Pinned, bool const Fallback, clock::time_point const Now ) {
        m_pinned = Pinned;
        m_fallback = Pinned && Fallback;
        m_lost = false;
        m_retrydelay = retry_min;
        m_nextcheck = Now + check_interval;
        m_restorefailures = 0;
    }

    // Events: bits of event; Connected: callable telling whether the device still works, called only when a check is due
    template <class Probe_>
    action
    update( unsigned const Events, clock::time_point const Now, Probe_ const &Connected ) {
        if( m_lost ) {
            // no working output: whatever the system reports may be the output coming back, so it brings the next attempt
            // forward. otherwise it comes after a growing delay
            if( Events != 0u ) { m_nextcheck = std::min( m_nextcheck, m_lastattempt + event_retry_min ); }
            return ( Now >= m_nextcheck ) ? action::reopen : action::none;
        }
        if( ( Now >= m_nextcheck ) || ( ( Events & device_removed ) != 0u ) ) {
            m_nextcheck = Now + check_interval;
            if( false == Connected() ) { return action::reopen; }
        }
        if( follows_default() && ( ( Events & default_changed ) != 0u ) ) { return action::reopen; }
        if( m_fallback ) {
            if( ( Events & device_added ) != 0u ) {
                // a device that appears may be the one set: a fresh round of attempts
                m_restorefailures = 0;
                return action::restore_preferred;
            }
            if( ( m_restorefailures > 0 ) && ( m_restorefailures <= restore_retries ) && ( Now >= m_nextrestore ) ) {
                return action::restore_preferred;
            }
        }
        return action::none;
    }

    // an output was opened and works. Fallback: it is the default one, in place of the one set in eu07.ini
    void
    succeeded( bool const Fallback, clock::time_point const Now ) {
        m_fallback = m_pinned && Fallback;
        m_lost = false;
        m_retrydelay = retry_min;
        m_nextcheck = Now + check_interval;
        // the default output standing in again doesn't end the round of attempts to move back to the device set
        if( false == m_fallback ) { m_restorefailures = 0; }
    }

    // no output could be opened. returns true for the first failure in a row, the one worth a line in the log
    bool
    failed( clock::time_point const Now ) {
        auto const first { false == m_lost };
        if( first ) {
            m_loststart = Now;
            m_retrydelay = retry_min;
        }
        else if( Now - m_loststart >= retry_late_after ) {
            m_retrydelay = retry_late;
        }
        else {
            m_retrydelay = std::min( m_retrydelay * 2, retry_max );
        }
        m_lost = true;
        m_lastattempt = Now;
        m_nextcheck = Now + m_retrydelay;
        return first;
    }

    // the move back to the device set in eu07.ini didn't work out: the device isn't listed, or it didn't open or start.
    // schedules the next attempt of the round, if there is one left
    void
    restore_failed( clock::time_point const Now ) {
        ++m_restorefailures;
        if( m_restorefailures <= restore_retries ) {
            m_nextrestore = Now + restore_retry_min * ( 1 << ( m_restorefailures - 1 ) );
        }
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
    clock::time_point m_loststart {}; // the first failed attempt of the current loss
    clock::time_point m_lastattempt {}; // the last failed attempt, while the output is lost
    int m_restorefailures { 0 }; // failed moves back to the device set since it last appeared, 0 when none is due
    clock::time_point m_nextrestore {};
};

} // namespace audio
