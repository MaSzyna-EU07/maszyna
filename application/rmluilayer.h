/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

struct GLFWwindow;
namespace Rml
{
class Context;
}

// RmlUi: the user interface made of documents (rml) and style sheets (rcss), shown over the simulation beside the one of imgui
// when ui.rmlui is on in eu07.ini. it's drawn into the frame of imgui, under its windows, by whichever renderer draws imgui.
// imgui gets the input first, RmlUi what imgui leaves. without ui.rmlui, or built without RmlUi (WITH_RMLUI), none of it runs and
// the functions here do nothing

namespace rmlui
{

// starts RmlUi, after imgui. false: it's off in the settings, wasn't built in or can't run (no font in fonts/)
bool init(GLFWwindow *Window);
// ends RmlUi, while the renderer still holds the images of its textures
void shutdown();
// whether RmlUi runs
bool active();
// the context the documents are shown in, nullptr when RmlUi doesn't run
Rml::Context *context();
// lays the documents out and draws them under the windows of imgui. called in a frame of imgui, by the user interface of a mode
// which shows them
void render();
// the debugger of RmlUi (the elements of the documents, their styles, the log) shown or hidden
void show_debugger(bool Show);
bool debugger_shown();

// the input of glfw, after imgui. true: RmlUi took it, the simulation doesn't get it. the documents get it only while they are
// drawn, the keys only while a control of them has the focus, the mouse only while the cursor is seen
bool on_key(int Key, int Action, int Mods);
bool on_char(unsigned int Codepoint);
bool on_mouse_button(int Button, int Action, int Mods);
bool on_scroll(double Yoffset);
void on_cursor_pos(double X, double Y, bool Visible);
void on_cursor_leave();

} // namespace rmlui
