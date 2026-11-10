/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <optional>
#include <vector>

#include "world/TrackSleepers.h"

// the layouts of the clusters of tracks kept till a track changes, and worked out in a thread of their own while the simulation runs
namespace track_sleepers {

// the layouts worked out so far hold no more, as a track changed
void outdate_layouts();
// the beams of the track from the layout of its cluster worked out already for the tracks as they are, none where there's none yet
std::optional<std::vector<beam>> laid_out( TTrack const &Track );
// whether the layouts are worked out in the background
bool in_background();
// the layout of the cluster of the track asked of the thread working them out
void ask_layout( TTrack &Track );
// nothing left to work out in the background nor being worked out, the waiting ones dropped
void settle_background();
// the layout of the cluster kept for the track let go, once the track laid its beams or went far; the cluster stays while
// another track of it keeps it
void forget_layout( TTrack const &Track );

} // namespace track_sleepers
