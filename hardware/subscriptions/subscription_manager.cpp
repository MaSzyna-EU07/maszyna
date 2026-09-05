/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "hardware/subscriptions/subscription_manager.h"

namespace hardware
{

error_code subscription_manager::subscribe( std::uint16_t const Handle, std::size_t const SymbolIndex, std::size_t const StateIndex, subscription_mode const Mode, std::uint16_t const PeriodMilliseconds, float const Deadband )
{
	if( ( Mode != subscription_mode::periodic ) && ( Mode != subscription_mode::on_change ) && ( Mode != subscription_mode::periodic_or_change ) )
	{
		return error_code::invalid_field;
	}
	if( ( false == std::isfinite( Deadband ) ) || ( Deadband < 0.0f ) )
	{
		return error_code::invalid_field;
	}

	auto const period = std::clamp<std::uint16_t>( ( PeriodMilliseconds > 0 ? PeriodMilliseconds : subscription_period_min_ms ), subscription_period_min_ms, subscription_period_max_ms ) * 0.001;

	auto entry = std::find_if( m_entries.begin(), m_entries.end(), [ Handle ]( subscription_entry const &Entry ) { return Entry.handle == Handle; } );
	if( entry == m_entries.end() )
	{
		if( m_entries.size() >= subscriptions_max )
		{
			return error_code::busy;
		}
		m_entries.emplace_back();
		entry = std::prev( m_entries.end() );
	}

	entry->handle = Handle;
	entry->symbol_index = SymbolIndex;
	entry->state_index = StateIndex;
	entry->mode = Mode;
	entry->period = period;
	entry->deadband = Deadband;
	entry->timer = period;
	entry->sampled = false;

	return error_code::none;
}

bool subscription_manager::unsubscribe( std::uint16_t const Handle )
{
	auto const entry = std::find_if( m_entries.begin(), m_entries.end(), [ Handle ]( subscription_entry const &Entry ) { return Entry.handle == Handle; } );
	if( entry == m_entries.end() )
	{
		return false;
	}
	m_entries.erase( entry );
	return true;
}

void subscription_manager::clear()
{
	m_entries.clear();
}

void subscription_manager::collect( double const Deltatime, state_snapshot const &Snapshot, bool const Everything, std::vector<state_item> &Output )
{
	auto const &states = state_registry::instance().entries();

	for( auto &entry : m_entries )
	{
		if( entry.state_index >= states.size() )
		{
			continue;
		}

		entry.timer += Deltatime;

		auto const value = states[ entry.state_index ].getter( Snapshot );

		bool const periodic = ( ( entry.mode != subscription_mode::on_change ) && ( entry.timer >= entry.period ) );
		bool changed = false;
		if( entry.mode != subscription_mode::periodic )
		{
			changed = (
			    ( false == entry.sampled )
			 || ( value.quality != entry.last_quality )
			 || ( std::abs( value.numeric - entry.last_numeric ) > entry.deadband ) );
		}

		if( ( false == Everything ) && ( false == periodic ) && ( false == changed ) )
		{
			continue;
		}

		entry.timer = 0.0;
		entry.sampled = true;
		entry.last_numeric = value.numeric;
		entry.last_quality = value.quality;

		state_item item;
		item.handle = entry.handle;
		item.value = value;
		Output.emplace_back( std::move( item ) );
	}
}

} // namespace hardware
