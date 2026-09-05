/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "hardware/protocol/protocol_defs.h"

namespace hardware
{

char const *to_string( message_type const Type )
{
	switch( Type )
	{
		case message_type::hello: return "HELLO";
		case message_type::welcome: return "WELCOME";
		case message_type::heartbeat: return "HEARTBEAT";
		case message_type::ack: return "ACK";
		case message_type::nack: return "NACK";
		case message_type::ping: return "PING";
		case message_type::pong: return "PONG";
		case message_type::device_ready: return "DEVICE_READY";
		case message_type::device_info: return "DEVICE_INFO";
		case message_type::device_diagnostics: return "DEVICE_DIAGNOSTICS";
		case message_type::resolve_symbols: return "RESOLVE_SYMBOLS";
		case message_type::symbols_resolved: return "SYMBOLS_RESOLVED";
		case message_type::query_symbol: return "QUERY_SYMBOL";
		case message_type::symbol_info: return "SYMBOL_INFO";
		case message_type::symbol_context_changed: return "SYMBOL_CONTEXT_CHANGED";
		case message_type::subscribe: return "SUBSCRIBE";
		case message_type::subscribe_result: return "SUBSCRIBE_RESULT";
		case message_type::unsubscribe: return "UNSUBSCRIBE";
		case message_type::state_update: return "STATE_UPDATE";
		case message_type::command_event: return "COMMAND_EVENT";
		case message_type::control_set: return "CONTROL_SET";
		case message_type::control_claim: return "CONTROL_CLAIM";
		case message_type::control_claim_result: return "CONTROL_CLAIM_RESULT";
		case message_type::control_release: return "CONTROL_RELEASE";
		case message_type::property_read: return "PROPERTY_READ";
		case message_type::property_value: return "PROPERTY_VALUE";
		case message_type::property_write: return "PROPERTY_WRITE";
		default: return "UNKNOWN";
	}
}

char const *to_string( error_code const Code )
{
	switch( Code )
	{
		case error_code::none: return "NONE";
		case error_code::unsupported_version: return "UNSUPPORTED_VERSION";
		case error_code::invalid_session: return "INVALID_SESSION";
		case error_code::invalid_frame: return "INVALID_FRAME";
		case error_code::invalid_length: return "INVALID_LENGTH";
		case error_code::invalid_field: return "INVALID_FIELD";
		case error_code::unknown_message: return "UNKNOWN_MESSAGE";
		case error_code::unknown_symbol: return "UNKNOWN_SYMBOL";
		case error_code::unknown_handle: return "UNKNOWN_HANDLE";
		case error_code::access_denied: return "ACCESS_DENIED";
		case error_code::out_of_range: return "OUT_OF_RANGE";
		case error_code::wrong_type: return "WRONG_TYPE";
		case error_code::not_ready: return "NOT_READY";
		case error_code::busy: return "BUSY";
		case error_code::control_not_owned: return "CONTROL_NOT_OWNED";
		case error_code::rate_limited: return "RATE_LIMITED";
		case error_code::unsupported_operation: return "UNSUPPORTED_OPERATION";
		default: return "INTERNAL_ERROR";
	}
}

char const *to_string( value_quality const Quality )
{
	switch( Quality )
	{
		case value_quality::valid: return "VALID";
		case value_quality::stale: return "STALE";
		case value_quality::unavailable: return "UNAVAILABLE";
		case value_quality::invalid: return "INVALID";
		default: return "NOT_APPLICABLE";
	}
}

char const *to_string( session_state const State )
{
	switch( State )
	{
		case session_state::disconnected: return "DISCONNECTED";
		case session_state::waiting_for_hello: return "WAITING FOR HELLO";
		case session_state::established: return "ESTABLISHED";
		default: return "READY";
	}
}

char const *to_string( link_state const State )
{
	switch( State )
	{
		case link_state::disconnected: return "DISCONNECTED";
		case link_state::connecting: return "CONNECTING";
		case link_state::healthy: return "HEALTHY";
		case link_state::degraded: return "DEGRADED";
		default: return "ERROR";
	}
}

std::size_t value_type_size( value_type const Type )
{
	switch( Type )
	{
		case value_type::boolean:
		case value_type::uint8:
		case value_type::int8:
		case value_type::enumeration:
			return 1;
		case value_type::uint16:
		case value_type::int16:
			return 2;
		case value_type::uint32:
		case value_type::int32:
		case value_type::float32:
		case value_type::bitfield:
			return 4;
		case value_type::uint64:
		case value_type::int64:
		case value_type::float64:
			return 8;
		default:
			// variable length, or nothing at all
			return 0;
	}
}

} // namespace hardware
