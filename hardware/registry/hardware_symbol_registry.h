/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware/protocol/protocol_defs.h"

// public symbol table of the hardware api
// merges the state and command registries into one namespace-prefixed list; devices refer to symbols
// by these names and receive short, session scoped handles in return

namespace hardware
{

struct symbol_descriptor
{
	// full public name, i.e. "state.velocity" or "command.trainbrakeset"
	std::string name;
	symbol_kind kind = symbol_kind::state;
	value_type type = value_type::float32;
	std::uint8_t access = access_read;
	std::string unit;
	std::optional<double> minimum;
	std::optional<double> maximum;
	// index into the registry the symbol originates from
	std::size_t source_index = 0;
};

class symbol_registry
{

public:
// types
	static constexpr std::size_t npos = std::numeric_limits<std::size_t>::max();

// methods
	static symbol_registry const &instance();

	std::vector<symbol_descriptor> const &entries() const { return m_entries; }
	// returns index of the symbol, or npos if the name isn't known
	std::size_t find( std::string const &Name ) const;

private:
// methods
	symbol_registry();

// members
	std::vector<symbol_descriptor> m_entries;
	std::unordered_map<std::string, std::size_t> m_index;
};

} // namespace hardware
