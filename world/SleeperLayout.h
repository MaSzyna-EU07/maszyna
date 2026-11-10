/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "world/TrackSleepers.h"

// the beams of a cluster of tracks laid out together
namespace track_sleepers {

// the beams of a cluster of tracks laid together, by the track each is laid with
struct cluster_beams {
    std::uint64_t epoch { 0 };
    std::map<TTrack const *, std::vector<beam>> beams;
};

// the tracks whose timbers are laid together with those of the track: all those reached from it through paths whose plain
// sleepers would meet, in the order of their names
std::vector<TTrack *> cluster_of( TTrack &Track );
// the beams of the tracks of a cluster, laid out together for the layout of the given epoch
std::shared_ptr<cluster_beams> lay_out( std::vector<TTrack *> const &Tracks, std::uint64_t const Epoch );
// how far the paths of the switch stand off one plane where they share the timbers of its panel, and where along the diverging
// path; the switch itself is out of shape for its timbers where it's more than the tolerance
std::pair<double, double> panel_off_plane( TTrack &Track );

} // namespace track_sleepers
