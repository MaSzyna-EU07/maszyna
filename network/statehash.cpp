/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "network/statehash.h"

#include "McZapkie/MOVER.h"
#include "simulation/simulation.h"
#include "simulation/simulationtime.h"
#include "vehicle/DynObj.h"

namespace
{

uint64_t const FNV_OFFSET{14695981039346656037ull};
uint64_t const FNV_PRIME{1099511628211ull};

uint64_t fold(uint64_t Hash, uint64_t Value)
{
	for (int i = 0; i < 8; ++i)
	{
		Hash ^= (Value >> (i * 8)) & 0xff;
		Hash *= FNV_PRIME;
	}
	return Hash;
}

uint64_t fold(uint64_t Hash, std::string const &Text)
{
	for (char const character : Text)
	{
		Hash ^= (uint64_t)(unsigned char)character;
		Hash *= FNV_PRIME;
	}
	return Hash;
}

// rounding to a fixed step keeps the digest from reacting to the last bits of a float
int64_t quantize(double Value, double Step)
{
	if (!std::isfinite(Value))
		return std::numeric_limits<int64_t>::min();

	return (int64_t)std::llround(Value / Step);
}

} // namespace

uint64_t network::state_hash()
{
	uint64_t digest{0};
	uint64_t counted{0};

	for (TDynamicObject const *vehicle : simulation::Vehicles.sequence())
	{
		if (vehicle == nullptr || vehicle->MoverParameters == nullptr)
			continue;

		auto const &mover = *vehicle->MoverParameters;
		auto const position = vehicle->GetPosition();

		uint64_t entry{FNV_OFFSET};
		entry = fold(entry, vehicle->name());
		entry = fold(entry, (uint64_t)quantize(position.x, 0.01));
		entry = fold(entry, (uint64_t)quantize(position.y, 0.01));
		entry = fold(entry, (uint64_t)quantize(position.z, 0.01));
		entry = fold(entry, (uint64_t)quantize(mover.V, 0.001));
		entry = fold(entry, (uint64_t)(int64_t)mover.DirActive);
		entry = fold(entry, (uint64_t)quantize(mover.PipePress, 0.001));
		entry = fold(entry, (uint64_t)quantize(mover.BrakePress, 0.001));

		// summing keeps the whole thing independent of iteration order
		digest += entry;
		++counted;
	}

	digest = fold(digest, counted);

	auto const &time = simulation::Time.data();
	digest = fold(digest, (uint64_t)(time.wHour * 3600 + time.wMinute * 60 + time.wSecond));

	return digest;
}
