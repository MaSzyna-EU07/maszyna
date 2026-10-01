/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "hardware/registry/hardware_symbol_registry.h"

#include "hardware/registry/command_registry.h"
#include "hardware/registry/state_registry.h"

namespace hardware
{

symbol_registry const &symbol_registry::instance()
{
	static symbol_registry const registry;
	return registry;
}

std::size_t symbol_registry::find( std::string const &Name ) const
{
	auto const lookup = m_index.find( Name );
	return ( lookup != m_index.end() ? lookup->second : npos );
}

symbol_registry::symbol_registry()
{
	auto const &states = state_registry::instance().entries();
	m_entries.reserve( states.size() + command_registry::instance().entries().size() );

	for( std::size_t index = 0; index < states.size(); ++index )
	{
		auto const &state = states[ index ];
		symbol_descriptor descriptor;
		descriptor.name = "state." + state.name;
		descriptor.kind = symbol_kind::state;
		descriptor.type = state.type;
		descriptor.access = access_read;
		descriptor.unit = state.unit;
		descriptor.minimum = state.minimum;
		descriptor.maximum = state.maximum;
		descriptor.source_index = index;
		m_index.emplace( descriptor.name, m_entries.size() );
		m_entries.emplace_back( std::move( descriptor ) );
	}

	auto const &commands = command_registry::instance().entries();
	for( std::size_t index = 0; index < commands.size(); ++index )
	{
		auto const &command = commands[ index ];
		symbol_descriptor descriptor;
		descriptor.name = "command." + command.name;
		descriptor.kind = symbol_kind::command;
		descriptor.type = command.parameter_type;
		descriptor.access = access_write;
		descriptor.minimum = command.minimum;
		descriptor.maximum = command.maximum;
		descriptor.source_index = index;
		m_index.emplace( descriptor.name, m_entries.size() );
		m_entries.emplace_back( std::move( descriptor ) );
	}
}

} // namespace hardware
