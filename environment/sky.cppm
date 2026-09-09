/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/
//---------------------------------------------------------------------------
module;
#include <array>
#include <chrono>
#include <cstddef>
#include <deque>
#include <limits>
#include <mutex>
#include <ostream>
#include <sstream>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

export module eu07.environment.sky;
import eu07.simcore;
import eu07.utilities.classes;

export {



class TSky {
public:
    TSky() = default;

    void Init();

public:
    // read directly by the renderers. They used to be friends, but a friend
    // declaration naming a class from another module would require importing it,
    // and that import closes a cycle the module graph does not allow.
    TModel3d *mdCloud { nullptr };
};

//---------------------------------------------------------------------------

}  // export
