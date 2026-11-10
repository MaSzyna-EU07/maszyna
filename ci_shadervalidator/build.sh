#!/bin/sh
# Offline validator of the shaders (shaders/ogl46 and shaders/vk14): the same preprocessing as the game
# (rendering/common/shadersource.cpp), then glslangValidator.
#
# Requirements: g++ with C++20, glslangValidator in PATH (Debian/Ubuntu: apt install glslang-tools),
# submodule ref/glm (git submodule update --init ref/glm).
# Usage (Linux; build.sh works from any directory, validateshaders reads ../shaders/ so it runs from here):
#   ci_shadervalidator/build.sh
#   cd ci_shadervalidator && ./validateshaders
# Every .vert/.frag/.geom/.comp of shaders/ogl46 is compiled as the game compiles the source (features on and off, and the
# variant of the multi draws) and to SPIR-V for opengl from the file as it is (as glslc makes the binaries of
# shaders/ogl46bin); the constants written in shaders/ogl46/common.glsl are compared with those of the game. The shaders of
# shaders/vk14 are compiled to SPIR-V for vulkan.
# Exit code is 0 when all shaders pass, 1 otherwise; failing shaders are listed at the end.
#
# The validator's own stdafx.h and utilities/{Globals,Logs}.h stand in for the game's headers: its directory comes first
# on the include path, before the repository root.
set -e
cd "$(dirname "$0")"
g++ -std=c++20 -O1 ../ref/glad/src/glad.c ../gl/glsl_common.cpp ../rendering/common/shadersource.cpp main.cpp -I. -I.. -I../ref/glad/include -I../ref/glm -o validateshaders
