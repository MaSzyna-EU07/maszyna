/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "hardware/registry/command_registry.h"

namespace hardware
{

namespace
{

struct command_metadata
{
	char const *name;
	bool continuous;
	std::optional<double> minimum;
	std::optional<double> maximum;
};

// hardware metadata for the commands which carry an analogue value; everything else is an event command
command_metadata const continuous_commands[] = {
    { "mastercontrollerset", true, 0.0, std::nullopt },
    { "secondcontrollerset", true, 0.0, std::nullopt },
    { "jointcontrollerset", true, 0.0, std::nullopt },
    { "trainbrakeset", true, 0.0, 1.0 },
    { "independentbrakeset", true, 0.0, 1.0 },
    { "dynamicbrakecontrollerset", true, 0.0, 1.0 },
    { "radiovolumeset", true, 0.0, 1.0 },
    { "radiochannelset", true, 1.0, 15.0 }
};

} // anonymous namespace

command_registry const &command_registry::instance()
{
	static command_registry const registry;
	return registry;
}

command_descriptor const *command_registry::find( std::string const &Name ) const
{
	auto const lookup = m_index.find( Name );
	return ( lookup != m_index.end() ? &m_entries[ lookup->second ] : nullptr );
}

command_registry::command_registry()
{
	std::size_t commandid = 0;
	for( auto const &description : simulation::Commands_descriptions )
	{
		command_descriptor descriptor;
		descriptor.name = description.name;
		descriptor.command = static_cast<user_command>( commandid );
		m_index.emplace( descriptor.name, m_entries.size() );
		m_entries.emplace_back( std::move( descriptor ) );
		++commandid;
	}

	for( auto const &metadata : continuous_commands )
	{
		auto const lookup = m_index.find( metadata.name );
		if( lookup == m_index.end() )
		{
			continue;
		}
		auto &descriptor = m_entries[ lookup->second ];
		descriptor.continuous = metadata.continuous;
		descriptor.minimum = metadata.minimum;
		descriptor.maximum = metadata.maximum;
	}

	// protocol level alias: master controller driven with a 0...1 fraction of its range, the way
	// uart v1 does it in percentage mode. it maps onto the regular mastercontrollerset command
	auto const mastercontroller = m_index.find( "mastercontrollerset" );
	if( mastercontroller != m_index.end() )
	{
		command_descriptor descriptor;
		descriptor.name = "mastercontrollersetpercent";
		descriptor.command = m_entries[ mastercontroller->second ].command;
		descriptor.continuous = true;
		descriptor.minimum = 0.0;
		descriptor.maximum = 1.0;
		descriptor.transform = command_transform::mastercontroller_percent;
		m_index.emplace( descriptor.name, m_entries.size() );
		m_entries.emplace_back( std::move( descriptor ) );
	}
}

} // namespace hardware
