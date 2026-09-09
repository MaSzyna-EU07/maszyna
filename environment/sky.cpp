module;
#include <array>
#include <chrono>
#include <cstddef>
#include <deque>
#include <limits>
#include <mutex>
#include <sstream>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>
#include <ostream>
#include <string>
#include "utilities/Globals_macros.h"

module eu07.environment.sky;
import eu07.utilities.globals;
import eu07.model.mdlmngr;
/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/


//---------------------------------------------------------------------------
//GLfloat lightPos[4] = {0.0f, 0.0f, 0.0f, 1.0f};

void TSky::Init() {

    if (Global.asSky != "1"
     && Global.asSky != "0" ) {

        mdCloud = TModelsManager::GetModel( Global.asSky );
    }
};

//---------------------------------------------------------------------------
