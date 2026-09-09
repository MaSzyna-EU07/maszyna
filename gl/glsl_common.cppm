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

export module eu07.gl.glsl_common;
import eu07.gl.ubo;
import eu07.utilities.globals;

export {


namespace gl
{
    extern std::string glsl_common;
    void glsl_common_setup();
}

}  // export
