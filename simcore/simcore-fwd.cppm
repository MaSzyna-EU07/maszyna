/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// Declarations shared by the simulation core partitions.
//
// These types point at each other -- a vehicle has a driver, a driver has a
// vehicle, a track carries events -- so they cannot live in separate modules: a
// forward declaration may not cross a module boundary. Inside one module it works
// as it always did, which is what this partition is for. It is the old
// utilities/Classes.h, except the declarations now attach to the same module as
// the definitions.

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

export module eu07.simcore:fwd;

export {

class TDynamicObject;
class TController;
class TTrack;
class TTraction;
class TMoverParameters;
class TMemCell;
class TEventLauncher;
class TModel3d;
class TSubModel;
class TAnimModel;
class TAnimContainer;
class basic_event;
class editor_ui;

namespace scene {
class basic_node;
class shape_node;
class basic_cell;
class basic_section;
class basic_region;
}

}  // export
