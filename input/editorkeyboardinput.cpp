/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "input/editorkeyboardinput.h"
#include "editor/editorSettings.hpp"

bool
editorkeyboard_input::init() {

    default_bindings();
    // TODO: re-enable after mode-specific binding import is in place
    // return recall_bindings();
	bind();

    return true;
}

void
editorkeyboard_input::apply_scheme() {

    default_bindings();
    bind();
}

void
editorkeyboard_input::default_bindings() {
    using enum user_command;

    if (EditorSettings.movement() == editorSettings::movement_scheme::legacy) {
        m_bindingsetups = {
            { moveleft, {GLFW_KEY_LEFT, "Move left"} },
            { moveright, {GLFW_KEY_RIGHT, "Move right"} },
            { moveforward, {GLFW_KEY_UP, "Move forwards"} },
            { moveback, {GLFW_KEY_DOWN, "Move backwards"} },
            { moveup, {GLFW_KEY_PAGE_UP, "Move up"} },
            { movedown, {GLFW_KEY_PAGE_DOWN, "Move down"} },
        };
    }
    else {
        m_bindingsetups = {
            { moveleft, {GLFW_KEY_A, "Move left"} },
            { moveright, {GLFW_KEY_D, "Move right"} },
            { moveforward, {GLFW_KEY_W, "Move forwards"} },
            { moveback, {GLFW_KEY_S, "Move backwards"} },
            { moveup, {GLFW_KEY_E, "Move up"} },
            { movedown, {GLFW_KEY_Q, "Move down"} },
        };
    }
}

//---------------------------------------------------------------------------
