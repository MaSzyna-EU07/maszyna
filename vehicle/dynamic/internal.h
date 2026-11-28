/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/
/*
    MaSzyna EU07 locomotive simulator
    Copyright (C) 2001-2004  Marcin Wozniak, Maciej Czapkiewicz and others

*/

// the constants and helpers the files of the vehicles of the scenery share: only they include this header

#pragma once

#include "vehicle/DynObj.h"
#include "physics/mover.h"
#include "vehicle/lightsprogrammer.h"
#include "simulation/simulation.h"
#include "simulation/blackboxrecorder.h"
#include "rendering/lightarray.h"
#include "vehicle/Camera.h"
#include "vehicle/Train.h"
#include "vehicle/odometer.h"
#include "vehicle/pulleemode.h"
#include "vehicle/Driver.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <format>
#include <random>
#include <iterator>
#include <numbers>
#include <optional>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>
#include "utilities/Globals.h"
#include "utilities/Timer.h"
#include "utilities/workerpool.h"
#include "utilities/Logs.h"
#include "utilities/glmHelpers.h"
#include "Console.h"
#include "world/Traction.h"
#include "audio/sound.h"
#include "model/MdlMngr.h"
#include "model/Model3d.h"
#include "vehicle/doorwings.h"
#include "rendering/renderer.h"
#include "editor/editorTerrainStreamer.hpp"
#include "application/uitranscripts.h"
#include "input/messaging.h"
#include "application/application.h"
#include "vehicle/Driver.h"

// Ra: taki zapis funkcjonuje lepiej, ale może nie jest optymalny
#define vWorldFront glm::vec3(0, 0, 1)
#define vWorldUp glm::vec3(0, 1, 0)
#define vWorldLeft glm::cross(vWorldUp, vWorldFront)

#define M_2PI 6.283185307179586476925286766559;

TSubModel *
GetSubmodelFromName( TModel3d * const Model, std::string const Name );

bool outside( glm::dvec3 const &Centre, double const Radius, glm::dvec2 const &Min, glm::dvec2 const &Max );


// the time spent off the track in a frame which makes it worth a warning [s]
constexpr double off_track_slow { 0.02 };


using off_track_clock = std::chrono::steady_clock;

double seconds_since( off_track_clock::time_point const Start );

