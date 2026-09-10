/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "hardware/registry/state_registry.h"

#include "simulation/simulation.h"
#include "simulation/simulationtime.h"

namespace hardware
{

namespace
{

state_value make_value( state_snapshot const &Snapshot, value_type const Type, double const Value, bool const Applicable = true )
{
	state_value result;
	result.type = Type;
	result.numeric = Value;
	result.quality = (
	    false == Snapshot.available ? value_quality::unavailable :
	    false == Applicable ? value_quality::not_applicable :
	    value_quality::valid );
	if( result.quality == value_quality::unavailable )
	{
		// there's nothing to report; a value which merely doesn't apply keeps whatever the
		// simulator holds, the quality flag is what tells the device not to trust it
		result.numeric = 0.0;
	}
	return result;
}

// values which don't depend on the vehicle are always valid
state_value make_global_value( value_type const Type, double const Value )
{
	state_value result;
	result.type = Type;
	result.numeric = Value;
	result.quality = value_quality::valid;
	return result;
}

} // anonymous namespace

state_registry const &state_registry::instance()
{
	static state_registry const registry;
	return registry;
}

state_descriptor const *state_registry::find( std::string const &Name ) const
{
	auto const lookup = m_index.find( Name );
	return ( lookup != m_index.end() ? &m_entries[ lookup->second ] : nullptr );
}

void state_registry::add( std::string Name, value_type const Type, std::string Unit, std::optional<double> const Minimum, std::optional<double> const Maximum, std::function<state_value( state_snapshot const & )> Getter )
{
	m_index.emplace( Name, m_entries.size() );
	state_descriptor descriptor;
	descriptor.name = std::move( Name );
	descriptor.type = Type;
	descriptor.unit = std::move( Unit );
	descriptor.minimum = Minimum;
	descriptor.maximum = Maximum;
	descriptor.getter = std::move( Getter );
	m_entries.emplace_back( std::move( descriptor ) );
}

state_registry::state_registry()
{
	// indicators and switch states
	add( "shp", value_type::boolean, "", {}, {}, []( state_snapshot const &s ) { return make_value( s, value_type::boolean, s.train.shp != 0 ); } );
	add( "alerter", value_type::boolean, "", {}, {}, []( state_snapshot const &s ) { return make_value( s, value_type::boolean, s.train.alerter != 0 ); } );
	add( "alerter_sound", value_type::boolean, "", {}, {}, []( state_snapshot const &s ) { return make_value( s, value_type::boolean, s.train.alerter_sound != 0 ); } );
	add( "radio_stop", value_type::boolean, "", {}, {}, []( state_snapshot const &s ) { return make_value( s, value_type::boolean, s.train.radio_stop != 0 ); } );
	add( "motor_resistors", value_type::boolean, "", {}, {}, []( state_snapshot const &s ) { return make_value( s, value_type::boolean, s.train.motor_resistors != 0 ); } );
	add( "line_breaker", value_type::boolean, "", {}, {}, []( state_snapshot const &s ) { return make_value( s, value_type::boolean, s.train.line_breaker != 0 ); } );
	add( "ground_relay", value_type::boolean, "", {}, {}, []( state_snapshot const &s ) { return make_value( s, value_type::boolean, s.train.ground_relay != 0 ); } );
	add( "motor_overload", value_type::boolean, "", {}, {}, []( state_snapshot const &s ) { return make_value( s, value_type::boolean, s.train.motor_overload != 0 ); } );
	add( "motor_overload_threshold", value_type::boolean, "", {}, {}, []( state_snapshot const &s ) { return make_value( s, value_type::boolean, s.train.motor_overload_threshold != 0 ); } );
	add( "motor_connectors", value_type::boolean, "", {}, {}, []( state_snapshot const &s ) { return make_value( s, value_type::boolean, s.train.motor_connectors != 0 ); } );
	add( "wheelslip", value_type::boolean, "", {}, {}, []( state_snapshot const &s ) { return make_value( s, value_type::boolean, s.train.wheelslip != 0 ); } );
	add( "converter_overload", value_type::boolean, "", {}, {}, []( state_snapshot const &s ) { return make_value( s, value_type::boolean, s.train.converter_overload != 0 ); } );
	add( "converter_off", value_type::boolean, "", {}, {}, []( state_snapshot const &s ) { return make_value( s, value_type::boolean, s.train.converter_off != 0 ); } );
	add( "compressor_overload", value_type::boolean, "", {}, {}, []( state_snapshot const &s ) { return make_value( s, value_type::boolean, s.train.compressor_overload != 0 ); } );
	add( "ventilator_overload", value_type::boolean, "", {}, {}, []( state_snapshot const &s ) { return make_value( s, value_type::boolean, s.train.ventilator_overload != 0 ); } );
	add( "train_heating", value_type::boolean, "", {}, {}, []( state_snapshot const &s ) { return make_value( s, value_type::boolean, s.train.train_heating != 0 ); } );
	add( "coupled_hv_voltage_relays", value_type::boolean, "", {}, {}, []( state_snapshot const &s ) { return make_value( s, value_type::boolean, s.train.coupled_hv_voltage_relays != 0 ); } );
	add( "recorder_braking", value_type::boolean, "", {}, {}, []( state_snapshot const &s ) { return make_value( s, value_type::boolean, s.train.recorder_braking != 0 ); } );
	add( "recorder_power", value_type::boolean, "", {}, {}, []( state_snapshot const &s ) { return make_value( s, value_type::boolean, s.train.recorder_power != 0 ); } );
	add( "springbrake_active", value_type::boolean, "", {}, {}, []( state_snapshot const &s ) { return make_value( s, value_type::boolean, s.train.springbrake_active != 0 ); } );
	add( "epbrake_enabled", value_type::boolean, "", {}, {}, []( state_snapshot const &s ) { return make_value( s, value_type::boolean, s.train.epbrake_enabled != 0 ); } );
	add( "emergencybrake", value_type::boolean, "", {}, {}, []( state_snapshot const &s ) { return make_value( s, value_type::boolean, s.train.emergencybrake != 0 ); } );
	add( "lockpipe", value_type::boolean, "", {}, {}, []( state_snapshot const &s ) { return make_value( s, value_type::boolean, s.train.lockpipe != 0 ); } );
	add( "battery", value_type::boolean, "", {}, {}, []( state_snapshot const &s ) { return make_value( s, value_type::boolean, s.train.battery != 0 ); } );
	add( "radiomessageindicator", value_type::boolean, "", {}, {}, []( state_snapshot const &s ) { return make_value( s, value_type::boolean, s.train.radiomessageindicator ); } );
	add( "dir_forward", value_type::boolean, "", {}, {}, []( state_snapshot const &s ) { return make_value( s, value_type::boolean, s.train.dir_forward != 0 ); } );
	add( "dir_backward", value_type::boolean, "", {}, {}, []( state_snapshot const &s ) { return make_value( s, value_type::boolean, s.train.dir_backward != 0 ); } );
	add( "doorleftallowed", value_type::boolean, "", {}, {}, []( state_snapshot const &s ) { return make_value( s, value_type::boolean, s.train.doorleftallowed != 0 ); } );
	add( "doorleftopened", value_type::boolean, "", {}, {}, []( state_snapshot const &s ) { return make_value( s, value_type::boolean, s.train.doorleftopened != 0 ); } );
	add( "doorrightallowed", value_type::boolean, "", {}, {}, []( state_snapshot const &s ) { return make_value( s, value_type::boolean, s.train.doorrightallowed != 0 ); } );
	add( "doorrightopened", value_type::boolean, "", {}, {}, []( state_snapshot const &s ) { return make_value( s, value_type::boolean, s.train.doorrightopened != 0 ); } );
	add( "doorstepallowed", value_type::boolean, "", {}, {}, []( state_snapshot const &s ) { return make_value( s, value_type::boolean, s.train.doorstepallowed != 0 ); } );

	// enumerations and counters
	add( "cab", value_type::uint8, "", 0.0, 2.0, []( state_snapshot const &s ) { return make_value( s, value_type::uint8, s.train.cab ); } );
	add( "cab_indicator", value_type::uint8, "", 0.0, 1.0, []( state_snapshot const &s ) { return make_value( s, value_type::uint8, s.cab_indicator ); } );
	add( "radio_channel", value_type::uint8, "", 0.0, 15.0, []( state_snapshot const &s ) { return make_value( s, value_type::uint8, s.train.radio_channel ); } );

	// measurements, in engineering units
	add( "velocity", value_type::float32, "km/h", 0.0, 400.0, []( state_snapshot const &s ) { return make_value( s, value_type::float32, s.train.velocity ); } );
	add( "reservoir_pressure", value_type::float32, "bar", 0.0, 16.0, []( state_snapshot const &s ) { return make_value( s, value_type::float32, s.train.reservoir_pressure ); } );
	add( "pipe_pressure", value_type::float32, "bar", 0.0, 16.0, []( state_snapshot const &s ) { return make_value( s, value_type::float32, s.train.pipe_pressure ); } );
	add( "brake_pressure", value_type::float32, "bar", 0.0, 16.0, []( state_snapshot const &s ) { return make_value( s, value_type::float32, s.train.brake_pressure ); } );
	add( "pantograph_pressure", value_type::float32, "bar", 0.0, 16.0, []( state_snapshot const &s ) { return make_value( s, value_type::float32, s.train.pantograph_pressure, s.pantographs ); } );
	add( "hv_voltage", value_type::float32, "V", 0.0, 40000.0, []( state_snapshot const &s ) { return make_value( s, value_type::float32, s.train.hv_voltage, s.electric ); } );
	add( "hv_current_1", value_type::float32, "A", -5000.0, 5000.0, []( state_snapshot const &s ) { return make_value( s, value_type::float32, s.train.hv_current[ 0 ], s.electric ); } );
	add( "hv_current_2", value_type::float32, "A", -5000.0, 5000.0, []( state_snapshot const &s ) { return make_value( s, value_type::float32, s.train.hv_current[ 1 ], s.electric ); } );
	add( "hv_current_3", value_type::float32, "A", -5000.0, 5000.0, []( state_snapshot const &s ) { return make_value( s, value_type::float32, s.train.hv_current[ 2 ], s.electric ); } );
	add( "lv_voltage", value_type::float32, "V", 0.0, 200.0, []( state_snapshot const &s ) { return make_value( s, value_type::float32, s.train.lv_voltage ); } );
	add( "distance", value_type::float64, "m", {}, {}, []( state_snapshot const &s ) { return make_value( s, value_type::float64, s.train.distance ); } );

	// simulation clock, available regardless of the occupied vehicle
	add( "time_month_of_era", value_type::uint32, "", {}, {}, []( state_snapshot const &s ) { return make_global_value( value_type::uint32, s.time_month_of_era ); } );
	add( "time_minute_of_month", value_type::uint32, "", {}, {}, []( state_snapshot const &s ) { return make_global_value( value_type::uint32, s.time_minute_of_month ); } );
	add( "time_millisecond_of_day", value_type::uint32, "", {}, {}, []( state_snapshot const &s ) { return make_global_value( value_type::uint32, s.time_millisecond_of_day ); } );
}

void state_registry::capture( state_snapshot &Snapshot )
{
	SYSTEMTIME const time = simulation::Time.data();
	Snapshot.time_month_of_era = ( time.wYear - 1 ) * 12 + time.wMonth - 1;
	Snapshot.time_minute_of_month = ( time.wDay - 1 ) * 1440 + time.wHour * 60 + time.wMinute;
	Snapshot.time_millisecond_of_day = time.wSecond * 1000 + time.wMilliseconds;

	auto const *train = simulation::Train;
	if( train == nullptr )
	{
		Snapshot.available = false;
		Snapshot.electric = false;
		Snapshot.pantographs = false;
		Snapshot.context_id = 0;
		return;
	}

	Snapshot.available = true;
	Snapshot.train = train->get_state();

	if( Snapshot.train.cab > 0 )
	{
		// NOTE: moving from a cab to the engine room doesn't change the cab indicator
		Snapshot.cab_indicator = Snapshot.train.cab - 1;
	}

	auto const *controlled = train->Controlled();
	auto const enginetype = ( controlled != nullptr ? controlled->EngineType : TEngineType::None );
	Snapshot.electric = ( ( enginetype == TEngineType::ElectricSeriesMotor ) || ( enginetype == TEngineType::ElectricInductionMotor ) );
	Snapshot.pantographs = Snapshot.electric;

	Snapshot.context_id = static_cast<std::uint32_t>( std::hash<std::string>()( train->name() ) );
}

} // namespace hardware
