/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "input/command.h"

class editormouse_input {

public:
// constructors
    editormouse_input() = default;

// methods
    bool
        init();
    void
        position( double Horizontal, double Vertical );
	glm::dvec2
        position() const {
            return m_cursorposition; }
    void
        button( int Button, int Action );
	int
        button( int const Button ) const {
            return m_buttons[ Button ]; }

private:
// members
    command_relay m_relay;
    bool m_pickmodepanning { false }; // indicates mouse is in view panning mode
    bool m_pickmodepanning_resync { false }; // skip the first delta after panning starts (avoids a jump when the cursor is grabbed/hidden)
    glm::dvec2 m_cursorposition { 0.0 }; // stored last cursor position, used for panning
    std::array<int, GLFW_MOUSE_BUTTON_LAST> m_buttons { GLFW_RELEASE };
};

//---------------------------------------------------------------------------
