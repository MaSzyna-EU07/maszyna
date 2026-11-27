#pragma once

#include <array>
#include <string>
#include <vector>

#include "ubo.h"
#include "utilities/Globals.h"

namespace gl
{
    // the constants of the game the shaders take (shaders/ogl46/common.glsl), as #define lines put before the sources
    extern std::string glsl_common;
    void glsl_common_setup();
    // the settings of the game the shaders take (shaders/ogl46/settings.glsl): #define lines of their values for the sources,
    // and the values of the specialization constants of the binaries, in the order of gl::shader_setting
    std::string glsl_settings_defines();
    std::array<GLuint, 6> glsl_settings_values();
    // whether the constants written in common.glsl in Basedir (shaders/ogl46/ when empty) are those of the game: the SPIR-V
    // binaries are compiled with the ones of the file. Differences: what isn't the same
    bool glsl_common_matches(std::string const &Basedir, std::vector<std::string> &Differences);
}
