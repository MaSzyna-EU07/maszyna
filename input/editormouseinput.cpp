/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "input/editormouseinput.h"

bool
editormouse_input::init() {

    return true;
}

void
editormouse_input::position( double Horizontal, double Vertical ) {

    if( false == m_pickmodepanning ) {
        // even if the view panning isn't active we capture the cursor position in case it does get activated
        m_cursorposition.x = Horizontal;
        m_cursorposition.y = Vertical;
        return;
    }
    if( true == m_pickmodepanning_resync ) {
        // panning just started (cursor may have been grabbed/hidden); reset the reference so the
        // first frame doesn't produce a large, jarring camera jump
        m_cursorposition.x = Horizontal;
        m_cursorposition.y = Vertical;
        m_pickmodepanning_resync = false;
        return;
    }
    glm::dvec2 cursorposition { Horizontal, Vertical };
    auto const viewoffset = cursorposition - m_cursorposition;
    m_relay.post(
        user_command::viewturn,
        viewoffset.x,
        viewoffset.y,
        GLFW_PRESS,
        // TODO: pass correct entity id once the missing systems are in place
	    0 );
    m_cursorposition = cursorposition;
}

void
editormouse_input::button( int const Button, int const Action ) {

    // store key state
    if( Button >= 0 ) {
        m_buttons[ Button ] = Action;
    }

    // right button controls panning
    if( Button == GLFW_MOUSE_BUTTON_RIGHT ) {
        bool const panning = Action == GLFW_PRESS;
        // when panning starts, request a one-frame resync so toggling the cursor grab doesn't jerk the view
        if( panning && false == m_pickmodepanning ) {
            m_pickmodepanning_resync = true;
        }
        m_pickmodepanning = panning;
    }
}

//---------------------------------------------------------------------------
