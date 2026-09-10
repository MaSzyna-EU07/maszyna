#pragma once
#ifdef _WIN32
#include <windows.h>   // before any import: its guard must be set here
#endif


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
import eu07.simcore;
struct MaTopologyUtils {

  void ConvertTopology(gfx::index_array &Indices, gfx::vertex_array &Vertices,
                       int const Typ) const;

};