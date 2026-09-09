/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "glad/glad.h"

#define STRINGIZE_DETAIL(x) #x
#define STRINGIZE(x) STRINGIZE_DETAIL(x)

// drops a marker into the command stream so gDEBugger/apitrace captures show
// which source line issued the calls that follow
#define glDebug(x) if (GLAD_GL_GREMEDY_string_marker) glStringMarkerGREMEDY(0, __FILE__ ":" STRINGIZE(__LINE__) ": " x);
