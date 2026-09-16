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

module eu07.application.shotrunner;
import eu07.glm;
import eu07.utilities.logs;
import eu07.utilities.utilities;

shot_runner Shots;

bool
shot_runner::parse( std::string const &Text, shot &Result ) {

    std::vector<double> fields;
    for( auto const &field : Split( Text, ',' ) ) {
        try {
            fields.push_back( std::stod( field ) );
        }
        catch( ... ) {
            return false;
        }
    }

    if( fields.size() < 3 ) { return false; }

    Result.position = { fields[ 0 ], fields[ 1 ], fields[ 2 ] };
    // the free camera takes its angles as pitch, yaw, roll in radians; a viewpoint is
    // written in degrees, because nobody wants to type radians on a command line
    Result.angle = {
        static_cast<float>( glm::radians( fields.size() > 4 ? fields[ 4 ] : 0.0 ) ),
        static_cast<float>( glm::radians( fields.size() > 3 ? fields[ 3 ] : 0.0 ) ),
        0.f };
    Result.label = Text;
    return true;
}

void
shot_runner::configure( std::vector<shot> Shots, double const Settle, bool const Quit ) {

    m_shots = std::move( Shots );
    m_settle = Settle;
    m_quit = Quit;
    m_index = 0;
    m_stage = stage::placing;
    m_elapsed = 0.0;
    m_done = m_shots.empty();
}

shot_runner::action
shot_runner::update( double const Deltatime ) {

    if( false == active() ) { return action::none; }

    switch( m_stage ) {

        case stage::placing: {
            m_stage = stage::settling;
            m_elapsed = 0.0;
            WriteLog( "Shot " + std::to_string( m_index + 1 ) + "/" + std::to_string( m_shots.size() )
                + ": moving to " + current().label );
            return action::place;
        }

        case stage::settling: {
            m_elapsed += Deltatime;
            if( m_elapsed < m_settle ) { return action::none; }
            m_stage = stage::capturing;
            return action::none;
        }

        case stage::capturing: {
            m_stage = stage::advancing;
            WriteLog( "Shot " + std::to_string( m_index + 1 ) + ": capturing" );
            return action::capture;
        }

        case stage::advancing: {
            // a frame's grace after the capture, since the image is read back and written
            // on another thread and quitting immediately would cut it short
            m_elapsed += Deltatime;
            if( m_elapsed < m_settle + 1.0 ) { return action::none; }
            ++m_index;
            if( m_index < m_shots.size() ) {
                m_stage = stage::placing;
                return action::none;
            }
            m_done = true;
            WriteLog( "Shot sequence finished" );
            return m_quit ? action::finish : action::none;
        }
    }

    return action::none;
}
