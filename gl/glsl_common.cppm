module;
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
