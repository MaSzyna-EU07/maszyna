/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// writes the machine to the log at startup, every line starting with "hardware: " (see hardwareinfo.h)
namespace hardware {

// the cpu, the memory and, on Windows, the graphics adapters DXGI lists
void log_system();

// the GPU the current OpenGL context runs on, its memory where the driver tells it, and a warning when it's the
// integrated GPU while the system has a dedicated one. after log_system(), with the context current
void log_gl_context();

} // namespace hardware
