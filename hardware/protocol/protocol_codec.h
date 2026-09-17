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
#include <string>
#include <vector>

#include "hardware/protocol/protocol_defs.h"

// MaSzyna Hardware Protocol v2, frame layer
// raw packet := header | payload | crc32c, transmitted as COBS( packet ) followed by a 0x00 delimiter

namespace hardware
{

struct packet_header
{
	std::uint8_t protocol_major = protocol_version_major;
	std::uint8_t protocol_minor = protocol_version_minor;
	std::uint8_t flags = packetflag_none;
	message_type type = message_type::invalid;
	std::uint32_t session_id = 0;
	std::uint32_t sequence = 0;
	std::uint32_t timestamp_ms = 0;
	std::uint16_t payload_length = 0;
};

struct decoded_packet
{
	packet_header header;
	std::vector<std::uint8_t> payload;
};

// crc32c (castagnoli), the checksum protecting header and payload
std::uint32_t crc32c( std::uint8_t const *Data, std::size_t const Size, std::uint32_t const Seed = 0 );

// consistent overhead byte stuffing; the encoded output never contains a zero byte
void cobs_encode( std::uint8_t const *Data, std::size_t const Size, std::vector<std::uint8_t> &Output );
// returns false if the input isn't a valid cobs sequence
bool cobs_decode( std::uint8_t const *Data, std::size_t const Size, std::vector<std::uint8_t> &Output );

// builds a complete, ready to transmit frame including the trailing delimiter
void encode_frame( packet_header const &Header, std::uint8_t const *Payload, std::size_t const PayloadSize, std::vector<std::uint8_t> &Output );

// bounds checked payload composer; all integers are little-endian, floats are ieee 754
class payload_writer
{

public:
// methods
	void write_bool( bool const Value );
	void write_uint8( std::uint8_t const Value );
	void write_int8( std::int8_t const Value );
	void write_uint16( std::uint16_t const Value );
	void write_int16( std::int16_t const Value );
	void write_uint32( std::uint32_t const Value );
	void write_int32( std::int32_t const Value );
	void write_uint64( std::uint64_t const Value );
	void write_int64( std::int64_t const Value );
	void write_float32( float const Value );
	void write_float64( double const Value );
	// length prefixed (uint8) utf-8 text, truncated to protocol_string_max
	void write_string( std::string const &Value );
	void write_bytes( std::uint8_t const *Data, std::size_t const Size );

	void clear() { m_data.clear(); }
	std::size_t size() const { return m_data.size(); }
	std::uint8_t const *data() const { return m_data.data(); }
	std::vector<std::uint8_t> const &buffer() const { return m_data; }
	// lets the caller patch a count written before the items were emitted
	void patch_uint16( std::size_t const Offset, std::uint16_t const Value );

private:
// members
	std::vector<std::uint8_t> m_data;
};

// bounds checked payload parser; every read past the end sets the error flag and yields a zeroed result
class payload_reader
{

public:
// methods
	payload_reader( std::uint8_t const *Data, std::size_t const Size ) : m_data( Data ), m_size( Size ) {}
	explicit payload_reader( std::vector<std::uint8_t> const &Data ) : m_data( Data.data() ), m_size( Data.size() ) {}

	bool read_bool();
	std::uint8_t read_uint8();
	std::int8_t read_int8();
	std::uint16_t read_uint16();
	std::int16_t read_int16();
	std::uint32_t read_uint32();
	std::int32_t read_int32();
	std::uint64_t read_uint64();
	std::int64_t read_int64();
	float read_float32();
	double read_float64();
	std::string read_string( std::size_t const Sizelimit = protocol_string_max );

	bool ok() const { return !m_error; }
	bool exhausted() const { return m_position >= m_size; }
	std::size_t remaining() const { return ( m_position < m_size ? m_size - m_position : 0 ); }
	void fail() { m_error = true; }

private:
// methods
	bool require( std::size_t const Size );

// members
	std::uint8_t const *m_data = nullptr;
	std::size_t m_size = 0;
	std::size_t m_position = 0;
	bool m_error = false;
};

// stream decoder; recovers from arbitrary corruption at the next delimiter
class frame_decoder
{

public:
// types
	struct counters
	{
		std::uint64_t frames = 0;
		std::uint64_t crc_errors = 0;
		std::uint64_t frame_errors = 0;
		std::uint64_t length_errors = 0;
		std::uint64_t version_errors = 0;
	};

// methods
	void reset();
	void set_frame_size_limit( std::size_t const Limit );
	// consumes raw bytes, appends every complete and valid packet to Output
	void feed( std::uint8_t const *Data, std::size_t const Size, std::vector<decoded_packet> &Output );

	counters const &stats() const { return m_stats; }

private:
// members
	std::vector<std::uint8_t> m_buffer;
	std::vector<std::uint8_t> m_scratch;
	std::size_t m_framesizelimit = protocol_frame_size_default;
	bool m_overrun = false;
	counters m_stats;
};

} // namespace hardware
