/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstddef>
#include <cstdint>

// MaSzyna Hardware Protocol v2 (MHPv2)
// transport-independent protocol definitions shared by every transport (serial, TCP, RS-485)

namespace hardware
{

// protocol version implemented by this build
constexpr std::uint8_t protocol_version_major = 2;
constexpr std::uint8_t protocol_version_minor = 0;

// 'M2', stored little-endian in the frame header
constexpr std::uint16_t protocol_magic = 0x324D;

constexpr std::size_t protocol_header_size = 24;
constexpr std::size_t protocol_crc_size = 4;

constexpr std::size_t protocol_frame_size_min = 64;
constexpr std::size_t protocol_frame_size_default = 1024;
constexpr std::size_t protocol_frame_size_max = 4096;

// safety limits applied to anything arriving from the outside world
constexpr std::size_t protocol_symbol_name_max = 128;
constexpr std::size_t protocol_string_max = 128;
constexpr std::size_t protocol_symbols_per_message_max = 64;

// message identifiers; the numbering is part of the public protocol and must not be reordered
enum class message_type : std::uint16_t
{
	invalid = 0x0000,

	hello = 0x0001,
	welcome = 0x0002,
	heartbeat = 0x0003,

	ack = 0x0004,
	nack = 0x0005,

	ping = 0x0006,
	pong = 0x0007,

	device_ready = 0x0008,

	device_info = 0x0010,
	device_diagnostics = 0x0011,

	resolve_symbols = 0x0020,
	symbols_resolved = 0x0021,
	query_symbol = 0x0022,
	symbol_info = 0x0023,
	symbol_context_changed = 0x0024,

	subscribe = 0x0030,
	subscribe_result = 0x0031,
	unsubscribe = 0x0032,
	state_update = 0x0033,

	command_event = 0x0040,
	control_set = 0x0041,

	control_claim = 0x0042,
	control_claim_result = 0x0043,
	control_release = 0x0044,

	property_read = 0x0050,
	property_value = 0x0051,
	property_write = 0x0052
};

// header flag bits
enum packet_flag : std::uint8_t
{
	packetflag_none = 0x00,
	packetflag_ack_required = 0x01,
	packetflag_response = 0x02,
	packetflag_error = 0x04,
	packetflag_high_priority = 0x08,
	packetflag_snapshot = 0x10
};

// wire data types
enum class value_type : std::uint8_t
{
	none = 0x00,
	boolean = 0x01,
	uint8 = 0x02,
	int8 = 0x03,
	uint16 = 0x04,
	int16 = 0x05,
	uint32 = 0x06,
	int32 = 0x07,
	uint64 = 0x08,
	int64 = 0x09,
	float32 = 0x0A,
	float64 = 0x0B,
	utf8_string = 0x0C,
	byte_array = 0x0D,
	enumeration = 0x0E,
	bitfield = 0x0F
};

// per-value quality indicator, so a device can tell "zero" from "not measured"
enum class value_quality : std::uint8_t
{
	valid = 0,
	stale = 1,
	unavailable = 2,
	invalid = 3,
	not_applicable = 4
};

enum class symbol_kind : std::uint8_t
{
	state = 0,
	command = 1,
	property = 2,
	diagnostic = 3
};

enum access_mode : std::uint8_t
{
	access_read = 0x01,
	access_write = 0x02,
	access_readwrite = 0x03
};

enum class subscription_mode : std::uint8_t
{
	periodic = 0,
	on_change = 1,
	periodic_or_change = 2
};

enum class command_action : std::uint8_t
{
	press = 0,
	release = 1,
	repeat = 2
};

enum class error_code : std::uint16_t
{
	none = 0,

	unsupported_version = 1,
	invalid_session = 2,

	invalid_frame = 3,
	invalid_length = 4,
	invalid_field = 5,

	unknown_message = 6,
	unknown_symbol = 7,
	unknown_handle = 8,

	access_denied = 9,
	out_of_range = 10,
	wrong_type = 11,

	not_ready = 12,
	busy = 13,
	control_not_owned = 14,

	rate_limited = 15,

	unsupported_operation = 16,
	internal_error = 17
};

// capabilities advertised by the device in HELLO
enum capability_flag : std::uint32_t
{
	capability_none = 0x00000000,
	capability_receive_state = 0x00000001,
	capability_send_commands = 0x00000002,
	capability_send_controls = 0x00000004,
	capability_diagnostics = 0x00000008,
	capability_soft_takeover = 0x00000010,
	capability_control_leases = 0x00000020
};

// session progress, mirrors the connection lifecycle of the specification
enum class session_state : std::uint8_t
{
	disconnected = 0,
	waiting_for_hello = 1,
	established = 2,
	ready = 3
};

// aggregated link condition presented in the debug ui
enum class link_state : std::uint8_t
{
	disconnected = 0,
	connecting = 1,
	healthy = 2,
	degraded = 3,
	error = 4
};

char const *to_string( message_type const Type );
char const *to_string( error_code const Code );
char const *to_string( value_quality const Quality );
char const *to_string( session_state const State );
char const *to_string( link_state const State );

// size of a fixed-width type on the wire, 0 for variable-length ones
std::size_t value_type_size( value_type const Type );

} // namespace hardware
