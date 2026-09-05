/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "hardware/protocol/protocol_session.h"

#include "hardware/registry/command_registry.h"
#include "hardware/registry/hardware_symbol_registry.h"
#include "simulation/simulation.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "vehicle/Train.h"

namespace hardware
{

namespace
{

// how many transaction results are remembered for duplicate detection
constexpr std::size_t transaction_cache_size = 128;
// upper bound on session handles, one per resolved symbol
constexpr std::size_t handles_max = 1024;
// round trip time is measured this often
constexpr double ping_interval = 5.0;

std::uint32_t generate_session_id()
{
	static std::mt19937 generator( static_cast<std::uint32_t>( std::chrono::steady_clock::now().time_since_epoch().count() ) );
	std::uint32_t const value = generator();
	// zero is reserved for "no session"
	return ( value != 0 ? value : 1 );
}

void write_state_value( payload_writer &Writer, state_value const &Value )
{
	switch( Value.type )
	{
		case value_type::boolean:
		case value_type::uint8:
		case value_type::int8:
		case value_type::enumeration:
			Writer.write_uint8( static_cast<std::uint8_t>( Value.numeric ) );
			break;
		case value_type::uint16:
		case value_type::int16:
			Writer.write_uint16( static_cast<std::uint16_t>( Value.numeric ) );
			break;
		case value_type::uint32:
		case value_type::int32:
		case value_type::bitfield:
			Writer.write_uint32( static_cast<std::uint32_t>( Value.numeric ) );
			break;
		case value_type::uint64:
		case value_type::int64:
			Writer.write_uint64( static_cast<std::uint64_t>( Value.numeric ) );
			break;
		case value_type::float64:
			Writer.write_float64( Value.numeric );
			break;
		case value_type::utf8_string:
			Writer.write_string( Value.text );
			break;
		default:
			Writer.write_float32( static_cast<float>( Value.numeric ) );
			break;
	}
}

std::size_t state_value_size( state_value const &Value )
{
	if( Value.type == value_type::utf8_string )
	{
		return 1 + std::min( Value.text.size(), protocol_string_max );
	}
	auto const size = value_type_size( Value.type );
	return ( size > 0 ? size : 4 );
}

int glfw_action( command_action const Action )
{
	switch( Action )
	{
		case command_action::release: return GLFW_RELEASE;
		case command_action::repeat: return GLFW_REPEAT;
		default: return GLFW_PRESS;
	}
}

} // anonymous namespace

protocol_session::protocol_session( session_output &Output, config const &Config ) : m_output( Output ), m_config( Config )
{
	m_framesize = std::clamp<std::size_t>( m_config.frame_size, protocol_frame_size_min, protocol_frame_size_max );
	reset();
}

void protocol_session::reset()
{
	m_state = session_state::waiting_for_hello;
	m_sessionid = 0;
	m_sequence = 0;
	m_lastreceivedsequence = 0;
	m_sequenceseen = false;
	m_framesize = std::clamp<std::size_t>( m_config.frame_size, protocol_frame_size_min, protocol_frame_size_max );
	m_capabilities = 0;
	m_peermajor = 0;
	m_peerminor = 0;

	m_devicename.clear();
	m_devicerole.clear();
	m_deviceid.clear();
	m_firmwareversion.clear();
	m_hardwareversion.clear();

	m_handles.clear();
	m_handlelookup.clear();
	m_subscriptions.clear();
	m_controlsequence.clear();
	m_transactions.clear();
	m_transactionorder.clear();

	m_uptime = 0.0;
	m_lastreceived = 0.0;
	m_heartbeattimer = 0.0;
	m_pingtimer = 0.0;
	m_pingpending = false;
	m_ratewindow = 0.0;
	m_framebudget = m_config.max_frames_per_second;
	m_payloadbudget = m_config.max_payload_bytes_per_second;
	m_commandbudget = m_config.max_commands_per_second;

	m_contextvalid = false;
	m_context = 0;

	m_diagnostics = protocol_diagnostics {};
	m_devicediagnostics = device_diagnostics {};
}

link_state protocol_session::link_condition() const
{
	if( m_state == session_state::waiting_for_hello )
	{
		return link_state::connecting;
	}
	if( m_diagnostics.missed_heartbeats <= 2 )
	{
		return link_state::healthy;
	}
	if( m_diagnostics.missed_heartbeats <= 4 )
	{
		return link_state::degraded;
	}
	return link_state::error;
}

void protocol_session::add_frame_errors( frame_decoder::counters const &Delta )
{
	m_diagnostics.crc_errors += Delta.crc_errors;
	m_diagnostics.frame_errors += Delta.frame_errors;
	m_diagnostics.length_errors += Delta.length_errors;
}

void protocol_session::send( message_type const Type, std::uint8_t const Flags, payload_writer const &Payload )
{
	packet_header header;
	header.flags = Flags;
	header.type = Type;
	header.session_id = m_sessionid;
	header.sequence = m_sequence++;
	header.timestamp_ms = static_cast<std::uint32_t>( m_uptime * 1000.0 );
	header.payload_length = static_cast<std::uint16_t>( Payload.size() );

	m_output.send_packet( header, Payload.buffer() );

	++m_diagnostics.tx_frames;
	m_txrate.add();

	if( true == debug_flags.log_frames )
	{
		WriteLog( "hardware: tx " + std::string( to_string( Type ) ) + ", " + std::to_string( Payload.size() ) + " bytes" );
	}
}

void protocol_session::send_ack( std::uint32_t const Transaction, message_type const Type )
{
	payload_writer writer;
	writer.write_uint32( Transaction );
	writer.write_uint16( static_cast<std::uint16_t>( Type ) );
	send( message_type::ack, packetflag_response, writer );
	++m_diagnostics.acks;
}

void protocol_session::send_nack( std::uint32_t const Transaction, message_type const Type, error_code const Error )
{
	payload_writer writer;
	writer.write_uint32( Transaction );
	writer.write_uint16( static_cast<std::uint16_t>( Type ) );
	writer.write_uint16( static_cast<std::uint16_t>( Error ) );
	send( message_type::nack, packetflag_response | packetflag_error, writer );
	++m_diagnostics.nacks;

	if( true == debug_flags.log_messages )
	{
		WriteLog( "hardware: nack " + std::string( to_string( Type ) ) + ", " + std::string( to_string( Error ) ) );
	}
}

std::size_t protocol_session::symbol_of_handle( std::uint16_t const Handle ) const
{
	if( ( Handle == 0 ) || ( Handle > m_handles.size() ) )
	{
		return symbol_registry::npos;
	}
	return m_handles[ Handle - 1 ];
}

std::uint16_t protocol_session::handle_of_symbol( std::size_t const SymbolIndex )
{
	auto const lookup = m_handlelookup.find( SymbolIndex );
	if( lookup != m_handlelookup.end() )
	{
		return lookup->second;
	}
	if( m_handles.size() >= handles_max )
	{
		return 0;
	}
	m_handles.emplace_back( SymbolIndex );
	auto const handle = static_cast<std::uint16_t>( m_handles.size() );
	m_handlelookup.emplace( SymbolIndex, handle );
	return handle;
}

bool protocol_session::consume_frame_budget( std::size_t const PayloadSize )
{
	if( ( m_framebudget <= 0 ) || ( m_payloadbudget < static_cast<int>( PayloadSize ) ) )
	{
		return false;
	}
	--m_framebudget;
	m_payloadbudget -= static_cast<int>( PayloadSize );
	return true;
}

bool protocol_session::consume_command_budget()
{
	if( m_commandbudget <= 0 )
	{
		return false;
	}
	--m_commandbudget;
	return true;
}

bool protocol_session::remember_transaction( std::uint32_t const Transaction, transaction_result const &Result )
{
	if( Transaction == 0 )
	{
		// transaction id zero means the device doesn't care about duplicate protection
		return false;
	}
	m_transactions[ Transaction ] = Result;
	m_transactionorder.emplace_back( Transaction );
	while( m_transactionorder.size() > transaction_cache_size )
	{
		m_transactions.erase( m_transactionorder.front() );
		m_transactionorder.pop_front();
	}
	return true;
}

protocol_session::transaction_result const *protocol_session::recall_transaction( std::uint32_t const Transaction ) const
{
	if( Transaction == 0 )
	{
		return nullptr;
	}
	auto const lookup = m_transactions.find( Transaction );
	return ( lookup != m_transactions.end() ? &lookup->second : nullptr );
}

void protocol_session::handle_packet( decoded_packet const &Packet )
{
	++m_diagnostics.rx_frames;
	m_rxrate.add();
	m_lastreceived = m_uptime;

	if( false == consume_frame_budget( Packet.payload.size() ) )
	{
		++m_diagnostics.rate_limited;
		return;
	}

	if( Packet.header.protocol_major != protocol_version_major )
	{
		++m_diagnostics.version_errors;
		if( Packet.header.type == message_type::hello )
		{
			send_nack( 0, message_type::hello, error_code::unsupported_version );
		}
		return;
	}

	if( true == m_sequenceseen )
	{
		if( Packet.header.sequence != m_lastreceivedsequence + 1 )
		{
			++m_diagnostics.sequence_gaps;
		}
	}
	m_lastreceivedsequence = Packet.header.sequence;
	m_sequenceseen = true;

	if( true == debug_flags.log_frames )
	{
		WriteLog( "hardware: rx " + std::string( to_string( Packet.header.type ) ) + ", " + std::to_string( Packet.payload.size() ) + " bytes" );
	}

	if( Packet.header.type == message_type::hello )
	{
		handle_hello( Packet );
		return;
	}

	if( Packet.header.type == message_type::ping )
	{
		payload_reader reader( Packet.payload );
		auto const token = reader.read_uint32();
		payload_writer writer;
		writer.write_uint32( token );
		send( message_type::pong, packetflag_response, writer );
		return;
	}

	// everything past this point belongs to an established session
	if( ( m_state == session_state::waiting_for_hello ) || ( Packet.header.session_id != m_sessionid ) )
	{
		send_nack( 0, Packet.header.type, error_code::invalid_session );
		return;
	}

	switch( Packet.header.type )
	{
		case message_type::heartbeat:
			handle_heartbeat( Packet );
			break;

		case message_type::pong:
		{
			payload_reader reader( Packet.payload );
			auto const token = reader.read_uint32();
			if( ( true == m_pingpending ) && ( token == m_pingtoken ) )
			{
				m_diagnostics.round_trip_time_ms = ( m_uptime - m_pingsent ) * 1000.0;
				m_pingpending = false;
			}
			break;
		}

		case message_type::device_ready:
			m_state = session_state::ready;
			if( true == debug_flags.log_messages )
			{
				WriteLog( "hardware: device '" + m_devicename + "' is ready" );
			}
			break;

		case message_type::device_diagnostics:
			handle_device_diagnostics( Packet );
			break;

		case message_type::resolve_symbols:
			handle_resolve_symbols( Packet );
			break;

		case message_type::query_symbol:
			handle_query_symbol( Packet );
			break;

		case message_type::subscribe:
			handle_subscribe( Packet );
			break;

		case message_type::unsubscribe:
			handle_unsubscribe( Packet );
			break;

		case message_type::command_event:
			handle_command_event( Packet );
			break;

		case message_type::control_set:
			handle_control_set( Packet );
			break;

		case message_type::control_claim:
		case message_type::control_release:
		case message_type::property_read:
		case message_type::property_write:
			// reserved for a later revision
			send_nack( 0, Packet.header.type, error_code::unsupported_operation );
			break;

		default:
			++m_diagnostics.unknown_messages;
			if( ( Packet.header.flags & packetflag_ack_required ) != 0 )
			{
				send_nack( 0, Packet.header.type, error_code::unknown_message );
			}
			break;
	}
}

void protocol_session::handle_hello( decoded_packet const &Packet )
{
	payload_reader reader( Packet.payload );

	auto const major = reader.read_uint8();
	auto const minor = reader.read_uint8();
	auto const framesize = reader.read_uint16();
	auto const capabilities = reader.read_uint32();
	auto const name = reader.read_string();
	auto const role = reader.read_string();

	std::string deviceid;
	std::string firmware;
	std::string hardware;
	// the remaining fields are optional, a minimal controller simply doesn't send them
	if( ( true == reader.ok() ) && ( false == reader.exhausted() ) ) { deviceid = reader.read_string(); }
	if( ( true == reader.ok() ) && ( false == reader.exhausted() ) ) { firmware = reader.read_string(); }
	if( ( true == reader.ok() ) && ( false == reader.exhausted() ) ) { hardware = reader.read_string(); }

	if( false == reader.ok() )
	{
		++m_diagnostics.length_errors;
		send_nack( 0, message_type::hello, error_code::invalid_length );
		return;
	}

	if( major != protocol_version_major )
	{
		++m_diagnostics.version_errors;
		send_nack( 0, message_type::hello, error_code::unsupported_version );
		return;
	}

	if( true == name.empty() )
	{
		send_nack( 0, message_type::hello, error_code::invalid_field );
		return;
	}

	// a fresh hello always starts a new session, invalidating whatever the previous one held
	m_handles.clear();
	m_handlelookup.clear();
	m_subscriptions.clear();
	m_controlsequence.clear();
	m_transactions.clear();
	m_transactionorder.clear();

	m_peermajor = major;
	m_peerminor = minor;
	m_capabilities = capabilities;
	m_devicename = name;
	m_devicerole = role;
	m_deviceid = deviceid;
	m_firmwareversion = firmware;
	m_hardwareversion = hardware;

	auto const requested = ( framesize > 0 ? static_cast<std::size_t>( framesize ) : protocol_frame_size_default );
	m_framesize = std::clamp<std::size_t>( std::min( requested, m_config.frame_size ), protocol_frame_size_min, protocol_frame_size_max );

	m_sessionid = generate_session_id();
	m_state = session_state::established;
	m_heartbeattimer = 0.0;
	m_pingtimer = 0.0;
	m_pingpending = false;
	m_contextvalid = false;

	payload_writer writer;
	writer.write_uint8( protocol_version_major );
	writer.write_uint8( protocol_version_minor );
	writer.write_uint32( m_sessionid );
	writer.write_uint16( static_cast<std::uint16_t>( m_framesize ) );
	writer.write_uint16( static_cast<std::uint16_t>( m_config.heartbeat_interval_ms ) );
	writer.write_string( Global.asVersion );
	send( message_type::welcome, packetflag_response, writer );

	WriteLog( "hardware: session " + std::to_string( m_sessionid ) + " established with '" + m_devicename + "' (" + m_devicerole + ")" );
}

void protocol_session::handle_heartbeat( decoded_packet const &Packet )
{
	payload_reader reader( Packet.payload );
	// the fields are informational; a device is free to send an empty heartbeat
	reader.read_uint32();
	reader.read_uint32();
	reader.read_uint16();
}

void protocol_session::handle_device_diagnostics( decoded_packet const &Packet )
{
	payload_reader reader( Packet.payload );

	device_diagnostics diagnostics;
	diagnostics.available = true;
	diagnostics.uptime_ms = reader.read_uint32();
	// every field past the uptime is optional
	if( ( true == reader.ok() ) && ( false == reader.exhausted() ) ) { diagnostics.supply_voltage = reader.read_float32(); }
	if( ( true == reader.ok() ) && ( false == reader.exhausted() ) ) { diagnostics.temperature = reader.read_float32(); }
	if( ( true == reader.ok() ) && ( false == reader.exhausted() ) ) { diagnostics.rx_overflows = reader.read_uint32(); }
	if( ( true == reader.ok() ) && ( false == reader.exhausted() ) ) { diagnostics.tx_overflows = reader.read_uint32(); }
	if( ( true == reader.ok() ) && ( false == reader.exhausted() ) ) { diagnostics.watchdog_resets = reader.read_uint32(); }
	if( ( true == reader.ok() ) && ( false == reader.exhausted() ) ) { diagnostics.firmware_build = reader.read_string(); }

	if( false == reader.ok() )
	{
		++m_diagnostics.length_errors;
		return;
	}

	m_devicediagnostics = diagnostics;
}

void protocol_session::handle_resolve_symbols( decoded_packet const &Packet )
{
	payload_reader reader( Packet.payload );
	auto const transaction = reader.read_uint32();
	auto const count = reader.read_uint8();

	if( ( false == reader.ok() ) || ( count > protocol_symbols_per_message_max ) )
	{
		send_nack( transaction, message_type::resolve_symbols, error_code::invalid_field );
		return;
	}

	auto const &symbols = symbol_registry::instance().entries();

	payload_writer writer;
	writer.write_uint32( transaction );
	writer.write_uint8( count );

	for( std::size_t index = 0; index < count; ++index )
	{
		auto const name = reader.read_string( protocol_symbol_name_max );
		if( false == reader.ok() )
		{
			send_nack( transaction, message_type::resolve_symbols, error_code::invalid_length );
			return;
		}

		auto const symbolindex = symbol_registry::instance().find( name );
		if( symbolindex == symbol_registry::npos )
		{
			++m_diagnostics.unknown_symbols;
			// handle zero tells the device the symbol isn't available in this simulator build
			writer.write_uint16( 0 );
			writer.write_uint8( 0 );
			writer.write_uint8( static_cast<std::uint8_t>( value_type::none ) );
			writer.write_uint8( 0 );
			continue;
		}

		auto const &symbol = symbols[ symbolindex ];
		writer.write_uint16( handle_of_symbol( symbolindex ) );
		writer.write_uint8( static_cast<std::uint8_t>( symbol.kind ) );
		writer.write_uint8( static_cast<std::uint8_t>( symbol.type ) );
		writer.write_uint8( symbol.access );
	}

	send( message_type::symbols_resolved, packetflag_response, writer );
}

void protocol_session::handle_query_symbol( decoded_packet const &Packet )
{
	payload_reader reader( Packet.payload );
	auto const transaction = reader.read_uint32();
	auto const handle = reader.read_uint16();

	if( false == reader.ok() )
	{
		send_nack( transaction, message_type::query_symbol, error_code::invalid_length );
		return;
	}

	auto const symbolindex = symbol_of_handle( handle );
	if( symbolindex == symbol_registry::npos )
	{
		++m_diagnostics.unknown_handles;
		send_nack( transaction, message_type::query_symbol, error_code::unknown_handle );
		return;
	}

	auto const &symbol = symbol_registry::instance().entries()[ symbolindex ];

	payload_writer writer;
	writer.write_uint32( transaction );
	writer.write_uint16( handle );
	writer.write_uint8( static_cast<std::uint8_t>( symbol.kind ) );
	writer.write_uint8( static_cast<std::uint8_t>( symbol.type ) );
	writer.write_uint8( symbol.access );
	writer.write_uint8( static_cast<std::uint8_t>( ( symbol.minimum ? 0x01 : 0x00 ) | ( symbol.maximum ? 0x02 : 0x00 ) ) );
	writer.write_float32( static_cast<float>( symbol.minimum.value_or( 0.0 ) ) );
	writer.write_float32( static_cast<float>( symbol.maximum.value_or( 0.0 ) ) );
	writer.write_string( symbol.name );
	writer.write_string( symbol.unit );
	send( message_type::symbol_info, packetflag_response, writer );
}

void protocol_session::handle_subscribe( decoded_packet const &Packet )
{
	payload_reader reader( Packet.payload );
	auto const transaction = reader.read_uint32();
	auto const count = reader.read_uint8();

	if( ( false == reader.ok() ) || ( count > protocol_symbols_per_message_max ) )
	{
		send_nack( transaction, message_type::subscribe, error_code::invalid_field );
		return;
	}

	auto const &symbols = symbol_registry::instance().entries();

	payload_writer writer;
	writer.write_uint32( transaction );
	writer.write_uint8( count );

	for( std::size_t index = 0; index < count; ++index )
	{
		auto const handle = reader.read_uint16();
		auto const mode = static_cast<subscription_mode>( reader.read_uint8() );
		auto const period = reader.read_uint16();
		auto const deadband = reader.read_float32();

		if( false == reader.ok() )
		{
			send_nack( transaction, message_type::subscribe, error_code::invalid_length );
			return;
		}

		auto result = error_code::none;
		auto const symbolindex = symbol_of_handle( handle );
		if( symbolindex == symbol_registry::npos )
		{
			++m_diagnostics.unknown_handles;
			result = error_code::unknown_handle;
		}
		else if( symbols[ symbolindex ].kind != symbol_kind::state )
		{
			result = error_code::access_denied;
		}
		else
		{
			result = m_subscriptions.subscribe( handle, symbolindex, symbols[ symbolindex ].source_index, mode, period, deadband );
		}

		writer.write_uint16( handle );
		writer.write_uint16( static_cast<std::uint16_t>( result ) );
	}

	send( message_type::subscribe_result, packetflag_response, writer );

	// the device initialises its gauges and indicators from the initial snapshot
	std::vector<state_item> items;
	m_subscriptions.collect( 0.0, m_snapshot, true, items );
	send_state_update( items, true );
}

void protocol_session::handle_unsubscribe( decoded_packet const &Packet )
{
	payload_reader reader( Packet.payload );
	auto const transaction = reader.read_uint32();
	auto const count = reader.read_uint8();

	if( ( false == reader.ok() ) || ( count > protocol_symbols_per_message_max ) )
	{
		send_nack( transaction, message_type::unsubscribe, error_code::invalid_field );
		return;
	}

	for( std::size_t index = 0; index < count; ++index )
	{
		auto const handle = reader.read_uint16();
		if( false == reader.ok() )
		{
			send_nack( transaction, message_type::unsubscribe, error_code::invalid_length );
			return;
		}
		m_subscriptions.unsubscribe( handle );
	}

	send_ack( transaction, message_type::unsubscribe );
}

void protocol_session::handle_command_event( decoded_packet const &Packet )
{
	payload_reader reader( Packet.payload );
	auto const transaction = reader.read_uint32();
	auto const handle = reader.read_uint16();
	auto const action = reader.read_uint8();
	auto const parameter1 = reader.read_float32();
	auto const parameter2 = reader.read_float32();
	auto const entity = reader.read_uint32();

	bool const acknowledge = ( ( Packet.header.flags & packetflag_ack_required ) != 0 );

	if( false == reader.ok() )
	{
		++m_diagnostics.length_errors;
		if( true == acknowledge )
		{
			send_nack( transaction, message_type::command_event, error_code::invalid_length );
		}
		return;
	}

	// a retransmission of an already executed command must not execute it a second time
	if( auto const *previous = recall_transaction( transaction ) )
	{
		++m_diagnostics.duplicate_commands;
		if( true == acknowledge )
		{
			if( true == previous->accepted )
			{
				send_ack( transaction, message_type::command_event );
			}
			else
			{
				send_nack( transaction, message_type::command_event, previous->error );
			}
		}
		return;
	}

	auto result = error_code::none;

	if( m_state != session_state::ready )
	{
		result = error_code::not_ready;
	}
	else if( false == consume_command_budget() )
	{
		++m_diagnostics.rate_limited;
		result = error_code::rate_limited;
	}
	else if( action > static_cast<std::uint8_t>( command_action::repeat ) )
	{
		result = error_code::invalid_field;
	}
	else if( ( false == std::isfinite( parameter1 ) ) || ( false == std::isfinite( parameter2 ) ) )
	{
		result = error_code::invalid_field;
	}
	else
	{
		auto const symbolindex = symbol_of_handle( handle );
		if( symbolindex == symbol_registry::npos )
		{
			++m_diagnostics.unknown_handles;
			result = error_code::unknown_handle;
		}
		else if( symbol_registry::instance().entries()[ symbolindex ].kind != symbol_kind::command )
		{
			result = error_code::access_denied;
		}
		else
		{
			result = execute_command( symbolindex, static_cast<command_action>( action ), parameter1, parameter2, static_cast<std::uint16_t>( entity ) );
		}
	}

	transaction_result const outcome { result == error_code::none, result };
	remember_transaction( transaction, outcome );

	if( true == acknowledge )
	{
		if( true == outcome.accepted )
		{
			send_ack( transaction, message_type::command_event );
		}
		else
		{
			send_nack( transaction, message_type::command_event, result );
		}
	}
}

void protocol_session::handle_control_set( decoded_packet const &Packet )
{
	payload_reader reader( Packet.payload );
	auto const handle = reader.read_uint16();
	auto const sequence = reader.read_uint32();
	auto const value = reader.read_float32();
	auto const entity = reader.read_uint32();

	if( false == reader.ok() )
	{
		++m_diagnostics.length_errors;
		return;
	}

	if( m_state != session_state::ready )
	{
		send_nack( 0, message_type::control_set, error_code::not_ready );
		return;
	}

	// updates are idempotent, so a late one is simply dropped
	auto const previous = m_controlsequence.find( handle );
	if( previous != m_controlsequence.end() )
	{
		if( static_cast<std::int32_t>( sequence - previous->second ) <= 0 )
		{
			return;
		}
	}
	m_controlsequence[ handle ] = sequence;

	auto const symbolindex = symbol_of_handle( handle );
	if( symbolindex == symbol_registry::npos )
	{
		++m_diagnostics.unknown_handles;
		send_nack( 0, message_type::control_set, error_code::unknown_handle );
		return;
	}

	auto const &symbol = symbol_registry::instance().entries()[ symbolindex ];
	if( symbol.kind != symbol_kind::command )
	{
		send_nack( 0, message_type::control_set, error_code::access_denied );
		return;
	}

	auto const &descriptor = command_registry::instance().entries()[ symbol.source_index ];
	if( false == descriptor.continuous )
	{
		send_nack( 0, message_type::control_set, error_code::wrong_type );
		return;
	}

	if( false == std::isfinite( value ) )
	{
		send_nack( 0, message_type::control_set, error_code::invalid_field );
		return;
	}

	// values outside the declared range are rejected rather than silently clipped
	if( ( descriptor.minimum && ( value < *descriptor.minimum ) ) || ( descriptor.maximum && ( value > *descriptor.maximum ) ) )
	{
		send_nack( 0, message_type::control_set, error_code::out_of_range );
		return;
	}

	if( false == consume_command_budget() )
	{
		++m_diagnostics.rate_limited;
		return;
	}

	auto const result = execute_command( symbolindex, command_action::press, value, 0.0, static_cast<std::uint16_t>( entity ) );
	if( result == error_code::none )
	{
		++m_diagnostics.controls_applied;
	}
	else
	{
		send_nack( 0, message_type::control_set, result );
	}
}

error_code protocol_session::execute_command( std::size_t const SymbolIndex, command_action const Action, double const Parameter1, double const Parameter2, std::uint16_t const Recipient )
{
	auto const &symbol = symbol_registry::instance().entries()[ SymbolIndex ];
	auto const &descriptor = command_registry::instance().entries()[ symbol.source_index ];

	if( descriptor.command == user_command::none )
	{
		return error_code::unsupported_operation;
	}

	double parameter = Parameter1;

	if( descriptor.transform == command_transform::mastercontroller_percent )
	{
		auto *train = simulation::Train;
		if( ( train == nullptr ) || ( train->Occupied() == nullptr ) )
		{
			return error_code::not_ready;
		}
		auto const percentage = std::clamp( Parameter1, 0.0, 1.0 );
		auto const positions = train->Occupied()->MainCtrlPosNo;
		parameter = ( percentage > 0.01 ? 1.0 + ( positions - 1 ) * percentage : 0.0 );
		train->Occupied()->eimic_analog = percentage;
	}

	m_relay.post( descriptor.command, parameter, Parameter2, glfw_action( Action ), Recipient );
	++m_diagnostics.commands_executed;

	return error_code::none;
}

void protocol_session::send_state_update( std::vector<state_item> const &Items, bool const Snapshot )
{
	if( true == Items.empty() )
	{
		return;
	}

	std::size_t const budget = ( m_framesize > protocol_header_size + protocol_crc_size ? m_framesize - protocol_header_size - protocol_crc_size : 64 );
	std::uint8_t const flags = ( Snapshot ? packetflag_snapshot : packetflag_none );

	payload_writer writer;
	writer.write_uint16( 0 );
	std::uint16_t count = 0;

	for( auto const &item : Items )
	{
		std::size_t const itemsize = 2 + 1 + 1 + state_value_size( item.value );
		if( ( count > 0 ) && ( writer.size() + itemsize > budget ) )
		{
			writer.patch_uint16( 0, count );
			send( message_type::state_update, flags, writer );
			++m_diagnostics.state_updates;
			writer.clear();
			writer.write_uint16( 0 );
			count = 0;
		}

		writer.write_uint16( item.handle );
		writer.write_uint8( static_cast<std::uint8_t>( item.value.type ) );
		writer.write_uint8( static_cast<std::uint8_t>( item.value.quality ) );
		write_state_value( writer, item.value );
		++count;
	}

	if( count > 0 )
	{
		writer.patch_uint16( 0, count );
		send( message_type::state_update, flags, writer );
		++m_diagnostics.state_updates;
	}
}

void protocol_session::send_symbol_context_changed( std::uint32_t const Context )
{
	payload_writer writer;
	writer.write_uint32( Context );
	send( message_type::symbol_context_changed, packetflag_none, writer );
}

void protocol_session::update( double const Deltatime, state_snapshot const &Snapshot )
{
	m_snapshot = Snapshot;
	m_uptime += Deltatime;

	m_rxrate.update( Deltatime );
	m_txrate.update( Deltatime );
	m_diagnostics.rx_rate = m_rxrate.rate();
	m_diagnostics.tx_rate = m_txrate.rate();
	m_diagnostics.connection_uptime = m_uptime;
	m_diagnostics.last_rx_age = m_uptime - m_lastreceived;

	m_ratewindow += Deltatime;
	if( m_ratewindow >= 1.0 )
	{
		m_ratewindow = 0.0;
		m_framebudget = m_config.max_frames_per_second;
		m_payloadbudget = m_config.max_payload_bytes_per_second;
		m_commandbudget = m_config.max_commands_per_second;
	}

	if( m_state == session_state::waiting_for_hello )
	{
		return;
	}

	double const heartbeatinterval = std::max( 0.05, m_config.heartbeat_interval_ms * 0.001 );
	m_diagnostics.missed_heartbeats = static_cast<int>( m_diagnostics.last_rx_age / heartbeatinterval );

	if( m_diagnostics.missed_heartbeats >= m_config.heartbeat_timeout_count )
	{
		WriteLog( "hardware: device '" + m_devicename + "' stopped responding, session closed" );
		reset();
		return;
	}

	// the metadata of some symbols depends on the controlled vehicle
	if( ( true == Snapshot.available ) && ( ( false == m_contextvalid ) || ( Snapshot.context_id != m_context ) ) )
	{
		bool const notify = m_contextvalid;
		m_context = Snapshot.context_id;
		m_contextvalid = true;
		if( true == notify )
		{
			send_symbol_context_changed( m_context );
		}
	}

	m_heartbeattimer += Deltatime;
	if( m_heartbeattimer >= heartbeatinterval )
	{
		m_heartbeattimer = 0.0;
		payload_writer writer;
		writer.write_uint32( static_cast<std::uint32_t>( m_uptime * 1000.0 ) );
		writer.write_uint32( m_lastreceivedsequence );
		writer.write_uint16( static_cast<std::uint16_t>( m_state ) );
		send( message_type::heartbeat, packetflag_none, writer );
	}

	m_pingtimer += Deltatime;
	if( ( false == m_pingpending ) && ( m_pingtimer >= ping_interval ) )
	{
		m_pingtimer = 0.0;
		m_pingpending = true;
		m_pingsent = m_uptime;
		++m_pingtoken;
		payload_writer writer;
		writer.write_uint32( m_pingtoken );
		send( message_type::ping, packetflag_none, writer );
	}
	else if( ( true == m_pingpending ) && ( m_uptime - m_pingsent > ping_interval ) )
	{
		// the answer never arrived, try again later
		m_pingpending = false;
		m_pingtimer = 0.0;
	}

	std::vector<state_item> items;
	m_subscriptions.collect( Deltatime, Snapshot, false, items );
	send_state_update( items, false );
}

void protocol_session::fill_report( link_report &Report ) const
{
	Report.device_name = m_devicename;
	Report.device_role = m_devicerole;
	Report.device_id = m_deviceid;
	Report.firmware_version = m_firmwareversion;
	Report.session = m_state;
	Report.session_id = m_sessionid;
	Report.protocol_major = m_peermajor;
	Report.protocol_minor = m_peerminor;
	Report.handles = m_handles.size();
	Report.subscriptions = m_subscriptions.size();
	Report.frame_size = m_framesize;
	Report.protocol = m_diagnostics;
	Report.device = m_devicediagnostics;
}

} // namespace hardware
