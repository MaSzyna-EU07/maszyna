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
#include <string>

namespace network
{

// layout of the snapshot container. a reader refuses a blob it does not understand
constexpr uint32_t SNAPSHOT_VERSION = 1;

// the sections a snapshot is built from. a reader skips over any id it does not know,
// which is what makes the format extensible: a peer built before a section existed can
// still read a newer snapshot and simply ignores what it has no use for
enum snapshot_chunk : uint16_t
{
	// tick, clock, weather and the state of the random engine
	SNAPSHOT_SESSION = 1,
	// where every vehicle is and what its controls and brakes are doing
	SNAPSHOT_VEHICLES = 2,
	// who is working which vehicle
	SNAPSHOT_CREWS = 3
	// candidates for the next versions: switches, memcells, the event queue, signals
};

// packs the current state of the world. only ever called on the authority
std::string take_snapshot();

// restores a snapshot handed to us by the authority. returns false when the blob cannot
// be used at all - a version we do not know, or damaged content
bool apply_snapshot(std::string const &Blob);

} // namespace network
