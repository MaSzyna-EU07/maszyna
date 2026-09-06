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

namespace network
{

// what goes into the digest. bump it whenever the layout changes, so that two builds
// cannot silently compare digests that mean different things
constexpr uint32_t STATE_HASH_VERSION = 1;

// digest of the gameplay critical state of the world, used to notice that two peers have
// drifted apart. deliberately order independent: every vehicle is folded in under its own
// identity and the results are summed, so the order the scenario happened to create them
// in does not matter. values are quantized, so that harmless float noise does not read as
// a desync
//
// version 1 covers vehicle position, speed, direction and the two brake pressures, plus
// the simulation clock. positions of switches, memcells and the event queue are the
// obvious next candidates - they go in when the snapshot work of stage 7 defines them
uint64_t state_hash();

} // namespace network
