/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "imgui/imgui.h"

// ImVec2 scaled by the user's ui scale factor.
// Global comes from utilities/Globals_macros.h and resolves at the expansion
// site, so this header must not pull the settings module in itself.
#define ImVec2S(a, b) ImVec2(a * Global.ui_scale, b * Global.ui_scale)
