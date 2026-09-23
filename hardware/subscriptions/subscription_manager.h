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
#include <vector>

#include "hardware/protocol/protocol_defs.h"
#include "hardware/registry/state_registry.h"

// state subscriptions of a single session
// a device receives only the values it asked for, at the rate it asked for; unchanged values within
// the configured deadband generate no traffic at all

namespace hardware
{

// upper bound on subscriptions per session, keeps a misbehaving device from exhausting memory
constexpr std::size_t subscriptions_max = 128;
constexpr std::uint16_t subscription_period_min_ms = 10;
constexpr std::uint16_t subscription_period_max_ms = 60000;

struct subscription_entry
{
	std::uint16_t handle = 0;
	std::size_t symbol_index = 0;
	std::size_t state_index = 0;
	subscription_mode mode = subscription_mode::periodic;
	double period = 0.05;
	double deadband = 0.0;
	double timer = 0.0;
	bool sampled = false;
	double last_numeric = 0.0;
	value_quality last_quality = value_quality::unavailable;
};

struct state_item
{
	std::uint16_t handle = 0;
	state_value value;
};

class subscription_manager
{

public:
// methods
	// registers or updates a subscription, returns the error to be reported back to the device
	error_code subscribe( std::uint16_t const Handle, std::size_t const SymbolIndex, std::size_t const StateIndex, subscription_mode const Mode, std::uint16_t const PeriodMilliseconds, float const Deadband );
	bool unsubscribe( std::uint16_t const Handle );
	void clear();

	std::size_t size() const { return m_entries.size(); }
	std::vector<subscription_entry> const &entries() const { return m_entries; }

	// advances the timers and collects the values which are due for transmission
	void collect( double const Deltatime, state_snapshot const &Snapshot, bool const Everything, std::vector<state_item> &Output );

private:
// members
	std::vector<subscription_entry> m_entries;
};

} // namespace hardware
