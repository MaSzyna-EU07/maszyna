/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

module;
#include <cstddef>
#include <string>
#include <vector>

export module eu07.application.shotrunner;
import eu07.glm;

export {

// Drives an unattended sequence of screenshots: place the camera, let the world settle,
// save a frame, move on, quit. Exists so that a visual change can be checked without a
// person sitting at the machine pressing keys, and checked the same way twice.
//
// The runner owns the sequence and its timing and nothing else. It does not know what a
// camera is or how a frame is saved: it reports what should happen next and the caller,
// which does know, carries it out.
class shot_runner {

public:
    struct shot {
        glm::dvec3 position { 0.0, 0.0, 0.0 };
        glm::vec3 angle { 0.f, 0.f, 0.f };  // pitch, yaw, roll in degrees, as the free camera takes them
        std::string label;                   // for the log, so a saved frame can be tied to a viewpoint
    };

    enum class action {
        none,     // nothing to do this frame
        place,    // move the camera to current()
        capture,  // save a frame
        finish    // the sequence is done
    };

    // parses a "x,y,z[,yaw[,pitch]]" viewpoint. returns false on anything malformed,
    // leaving Result untouched
    static bool parse( std::string const &Text, shot &Result );

    void configure( std::vector<shot> Shots, double const Settle, bool const Quit );
    bool active() const { return ( false == m_shots.empty() ) && ( false == m_done ); }
    shot const &current() const { return m_shots[ m_index ]; }

    // advances the sequence by one frame. Deltatime is wall time, so that the settling
    // period covers tiles streaming in rather than a fixed number of frames
    action update( double const Deltatime );

private:
    enum class stage {
        placing,
        settling,
        capturing,
        advancing
    };

    std::vector<shot> m_shots;
    std::size_t m_index { 0 };
    stage m_stage { stage::placing };
    double m_settle { 3.0 };
    double m_elapsed { 0.0 };
    bool m_quit { true };
    bool m_done { false };
};

// the session's sequence, configured from the command line and driven by the driver mode.
// idle and harmless when no viewpoints were given
extern shot_runner Shots;

}  // export
