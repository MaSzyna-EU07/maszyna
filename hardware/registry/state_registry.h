/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <functional>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware/protocol/protocol_defs.h"
#include "vehicle/Train.h"

// shared TrainState registry
// one description of the simulator state exposed to the outside world, meant to be used by every
// external interface (hardware protocol v2, zmq, python) instead of each of them keeping its own list

namespace hardware
{

// values captured once per simulation frame, so that all consumers see a consistent set
struct state_snapshot
{
	// false when there's no occupied vehicle; state symbols then report UNAVAILABLE
	bool available = false;
	// high voltage circuit is meaningful for this vehicle
	bool electric = false;
	// vehicle is equipped with pantographs
	bool pantographs = false;
	TTrain::state_t train {};
	// last active cab, 0: front, 1: rear. moving to the engine room doesn't change it
	std::uint8_t cab_indicator = 0;
	// changes whenever the controlled vehicle changes, used to notify devices about metadata changes
	std::uint32_t context_id = 0;
	double time_month_of_era = 0.0;
	double time_minute_of_month = 0.0;
	double time_millisecond_of_day = 0.0;
};

struct state_value
{
	value_type type = value_type::float32;
	value_quality quality = value_quality::unavailable;
	double numeric = 0.0;
	std::string text;
};

struct state_descriptor
{
	// registry name without namespace prefix, i.e. "velocity" rather than "state.velocity"
	std::string name;
	value_type type = value_type::float32;
	std::string unit;
	std::optional<double> minimum;
	std::optional<double> maximum;
	std::function<state_value( state_snapshot const & )> getter;
};

class state_registry
{

public:
// methods
	static state_registry const &instance();

	std::vector<state_descriptor> const &entries() const { return m_entries; }
	state_descriptor const *find( std::string const &Name ) const;

	// fills the snapshot from the currently occupied train; simulation thread only
	static void capture( state_snapshot &Snapshot );

private:
// methods
	state_registry();
	void add( std::string Name, value_type const Type, std::string Unit, std::optional<double> const Minimum, std::optional<double> const Maximum, std::function<state_value( state_snapshot const & )> Getter );

// members
	std::vector<state_descriptor> m_entries;
	std::unordered_map<std::string, std::size_t> m_index;
};

} // namespace hardware
