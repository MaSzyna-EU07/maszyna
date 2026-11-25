#pragma once
// stands in for the precompiled header of the simulator, with what gl/shader.cpp, gl/glsl_common.cpp and the
// headers they include take from it
#define GLM_ENABLE_EXPERIMENTAL
#define GLM_FORCE_CTOR_INIT
#include <glm/glm.hpp>
#include <algorithm>
#include <array>
#include <string>
#include <string_view>
#include <stdexcept>
