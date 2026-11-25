#!/bin/sh
# builds the shader validator next to this script. it compiles gl/shader.cpp and gl/glsl_common.cpp of the
# simulator against the stand-in headers of this directory (stdafx.h, utilities/Globals.h, utilities/Logs.h)
# and needs glslangValidator on PATH to run. run validateshaders from this directory
set -e
cd "$(dirname "$0")"
${CXX:-g++} -std=c++20 -O2 -DSHADERVALIDATOR_STANDALONE ../ref/glad/src/glad.c ../gl/glsl_common.cpp ../gl/shader.cpp main.cpp -I../ref/glad/include -I../ref/glm -I. -o validateshaders
