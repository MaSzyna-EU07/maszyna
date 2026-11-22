/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cmath>
#include <cstdint>

namespace gfx
{

// how many times a second an arc at a pantograph flickers to another brightness
inline constexpr double ARC_FLICKER_RATE{30.0};

// the brightness of the flicker of an arc at a pantograph, 0.55..1: the billboard of scene_walk::Render_arcs() and the light of
// gfx::local_lights take the same one. worked out of the pantograph (Vehicle, a token of the vehicle such as its address, and
// Pantograph, its number) and of Time, s of the simulation, rather than drawn from the engine of Random(): the network sessions
// keep that one in step across the machines, and the draws of the renderer, as many as the frames and the arcs seen, would set
// them apart. the same at any frame rate, still in a pause
inline float arc_flicker(std::uintptr_t const Vehicle, int const Pantograph, double const Time)
{
	auto const tick{static_cast<std::uint64_t>(static_cast<std::int64_t>(std::floor(Time * ARC_FLICKER_RATE)))};
	// splitmix64 of the pantograph and the tick
	auto value{static_cast<std::uint64_t>(Vehicle) ^ (static_cast<std::uint64_t>(static_cast<std::uint32_t>(Pantograph)) << 56) ^ (tick * 0x9e3779b97f4a7c15ull)};
	value ^= value >> 30;
	value *= 0xbf58476d1ce4e5b9ull;
	value ^= value >> 27;
	value *= 0x94d049bb133111ebull;
	value ^= value >> 31;
	// the top 24 bits as a share, 0..1
	auto const share{static_cast<float>(value >> 40) / static_cast<float>(1u << 24)};
	return 0.55f + 0.45f * share;
}

} // namespace gfx
