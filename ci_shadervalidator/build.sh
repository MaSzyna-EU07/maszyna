#!/bin/sh
# Offline GLSL validator for shaders/ (the same preprocessing as gl::shader in the game, then glslangValidator).
#
# Requirements: g++ with C++20, glslangValidator in PATH (Debian/Ubuntu: apt install glslang-tools),
# submodule ref/glm (git submodule update --init ref/glm).
# Usage (Linux; build.sh works from any directory, validateshaders reads ../shaders/ so it runs from here):
#   ci_shadervalidator/build.sh
#   cd ci_shadervalidator && ./validateshaders
# Every .vert/.frag/.geom/.comp under shaders/ is checked twice (features on and off).
# Exit code is 0 when all shaders pass, 1 otherwise; failing shaders are listed at the end.
#
# The validator's own stdafx.h and utilities/{Globals,Logs}.h stand in for the game's headers,
# so the repository root must not be on the include path.
set -e
cd "$(dirname "$0")"
g++ -std=c++20 -O1 -DSHADERVALIDATOR_STANDALONE ../ref/glad/src/glad.c ../gl/glsl_common.cpp ../gl/shader.cpp main.cpp -I. -I../ref/glad/include -I../ref/glm -o validateshaders
