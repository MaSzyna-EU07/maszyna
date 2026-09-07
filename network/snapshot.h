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

#include "network/entities.h"

namespace network
{

// layout of the snapshot container. a reader refuses a blob it does not understand
constexpr uint32_t SNAPSHOT_VERSION = 5;

// the sections a snapshot is built from. a reader skips over any id it does not know,
// which is what makes the format extensible: a peer built before a section existed can
// still read a newer snapshot and simply ignores what it has no use for
enum snapshot_chunk : uint16_t
{
	// tick, clock, weather and (in a full snapshot) the state of the random engine
	SNAPSHOT_SESSION = 1,
	// where every vehicle is, what its controls and appliances are set to, and what its
	// brakes are doing
	SNAPSHOT_VEHICLES = 2,
	// who is working which vehicle
	SNAPSHOT_CREWS = 3,
	// contents of the memory cells - this is what signalling reads
	SNAPSHOT_MEMCELLS = 4,
	// which way the switches are thrown
	SNAPSHOT_SWITCHES = 5
};

// what a peer is meant to do with the blob it received
enum class snapshot_mode
{
	// the peer is coming in cold and takes the world as it is given
	join,
	// routine correction on top of a simulation that is already running
	correction
};

struct snapshot_result
{
	bool ok{false};
	// how far the worst vehicle was from where the authority says it is
	double worst_position_error{0.0};
	uint32_t vehicles{0};
	uint32_t repositioned{0};
};

// packs the state of the world. only ever called on the authority.
// a full snapshot carries everything and is what a joining peer gets; otherwise only what
// has changed since the last one goes out, which is what keeps the routine correction
// stream small
// OwnedOnly limits it to the trains this peer runs itself and leaves out everything that
// is not a vehicle; that is what a client sends upstream about its own train
std::string take_snapshot(bool Full, bool OwnedOnly = false);

// restores state handed to us. vehicles this peer runs itself are never touched - their
// physics is the authority, not the other way round. Owner, when given, limits the update
// to the trains that peer is entitled to move, so a client cannot shove anybody else's
snapshot_result apply_snapshot(std::string const &Blob, snapshot_mode Mode, PeerId Owner = PEER_NONE);

// forgets what was last sent, so that the next delta comes out complete
void reset_snapshot_history();

// remembers the one-shot sound events a vehicle produced. they are set and cleared inside
// a single frame, so by the time an update is put together they would be long gone; this
// is called from the vehicle's own sound handling, just before it wipes them
void note_sound_events(std::string const &Vehicle, int Events);

// same, for the events the brake unit raises. those are only cleared when the vehicle
// happens to have an accelerator sound to play, so what is passed here is peeked rather
// than consumed and only the flags that have just appeared are taken
void note_brake_sound_events(std::string const &Vehicle, int Events);

// re-asserts, on every frame, the physics outputs of the vehicles somebody else is
// running. an update arrives ten times a second; between them the local physics carries
// on computing its own speed and brake force for a train it has no business simulating,
// and the sawtooth that produces is what the brake and squeal sounds were chasing
void hold_remote_state();

} // namespace network
