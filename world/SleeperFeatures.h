/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <array>
#include <vector>

#include <glm/glm.hpp>

#include "world/SleeperPaths.h"

// the frogs of the switches and the crossings of the tracks, which the timbers about them turn to
namespace track_sleepers {

// where two paths meet at an angle, the timbers which carry both lie square to the bisector of the angle: past the frog of
// a switch, where the paths go apart, and about the point where two paths cross. the timbers turn to the bisector before
// and back square to the tracks after, by the distance along the bisector from the point
struct feature {
    glm::dvec2 point; // the frog, or where the paths cross, in plan
    glm::dvec2 bisector; // unit
    double sine { 0.0 }; // of half the angle between the paths
    double spacing { 0.0 }; // how far apart the paths run at the point
    double from { 0.0 }; // along the bisector, where the timbers start turning to it
    double full { 0.0 }; // where they're square to it
    double until { 0.0 }; // till where they stay so
    double back { 0.0 }; // where they're square to the tracks again
    std::array<sampled_path const *, 2> paths {};
    std::array<double, 2> at {}; // the distance of the point along each path
    bool frog { false }; // the paths go apart from the point, rather than cross there
    bool grid { false }; // the paths cross so wide the timbers keep square to each
    bool crossover { false }; // the frog of a switch whose diverging track leads to another switch of a track alongside: the timbers keep square to the tracks
};

// the frogs of the switches and the crossings of the paths around
std::vector<feature> find_features( std::vector<sampled_path> const &Around );
// which way the beam at the point lies square to: the path laying it, turned toward the bisectors of the frogs and crossings
// about the point. this depends only on where the point is, so the beams of paths laid one after another turn smoothly
glm::dvec2 beam_direction( glm::dvec3 const &Point, glm::dvec2 const &Forward, std::vector<feature> const &Features );

} // namespace track_sleepers
