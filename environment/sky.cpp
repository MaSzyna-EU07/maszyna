/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "environment/sky.h"
#include "utilities/Globals.h"
#include "model/MdlMngr.h"

//---------------------------------------------------------------------------
//GLfloat lightPos[4] = {0.0f, 0.0f, 0.0f, 1.0f};

void TSky::Init() {

    // the clouds are worked out from the weather (environment/weather.h) and drawn by the renderer; the cloud model the sky
    // section of a scenery names (Global.asSky) is no longer loaded
    mdCloud = nullptr;
};

//---------------------------------------------------------------------------
