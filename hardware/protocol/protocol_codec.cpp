/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "hardware/protocol/protocol_codec.h"

#include <cstring>

namespace hardware
{

namespace
{

// castagnoli polynomial in reflected form
constexpr std::uint32_t crc32c_polynomial = 0x82F63B78u;

std::array<std::uint32_t, 256> const &crc32c_table()
{
	static std::array<std::uint32_t, 256> const table = []()
	{
		std::array<std::uint32_t, 256> result {};
		for( std::uint32_t index = 0; index < 256; ++index )
		{
			std::uint32_t value = index;
			for( int bit = 0; bit < 8; ++bit )
			{
				value = ( value & 1 ? ( value >> 1 ) ^ crc32c_polynomial : value >> 1 );
			}
			result[ index ] = value;
		}
		return result;
	}();

	return table;
}

void append_uint16( std::vector<std::uint8_t> &Buffer, std::uint16_t const Value )
{
	Buffer.emplace_back( static_cast<std::uint8_t>( Value ) );
	Buffer.emplace_back( static_cast<std::uint8_t>( Value >> 8 ) );
}

void append_uint32( std::vector<std::uint8_t> &Buffer, std::uint32_t const Value )
{
	Buffer.emplace_back( static_cast<std::uint8_t>( Value ) );
	Buffer.emplace_back( static_cast<std::uint8_t>( Value >> 8 ) );
	Buffer.emplace_back( static_cast<std::uint8_t>( Value >> 16 ) );
	Buffer.emplace_back( static_cast<std::uint8_t>( Value >> 24 ) );
}

std::uint16_t extract_uint16( std::uint8_t const *Data )
{
	return static_cast<std::uint16_t>( static_cast<std::uint16_t>( Data[ 0 ] ) | static_cast<std::uint16_t>( Data[ 1 ] ) << 8 );
}

std::uint32_t extract_uint32( std::uint8_t const *Data )
{
	return static_cast<std::uint32_t>( Data[ 0 ] ) | static_cast<std::uint32_t>( Data[ 1 ] ) << 8 | static_cast<std::uint32_t>( Data[ 2 ] ) << 16 | static_cast<std::uint32_t>( Data[ 3 ] ) << 24;
}

} // anonymous namespace

std::uint32_t crc32c( std::uint8_t const *Data, std::size_t const Size, std::uint32_t const Seed )
{
	auto const &table = crc32c_table();
	std::uint32_t crc = ~Seed;
	for( std::size_t index = 0; index < Size; ++index )
	{
		crc = table[ ( crc ^ Data[ index ] ) & 0xFF ] ^ ( crc >> 8 );
	}
	return ~crc;
}

void cobs_encode( std::uint8_t const *Data, std::size_t const Size, std::vector<std::uint8_t> &Output )
{
	Output.clear();
	// the first byte holds the distance to the first zero, it's patched once that distance is known
	std::size_t codeposition = 0;
	Output.emplace_back( 0 );
	std::uint8_t code = 1;

	for( std::size_t index = 0; index < Size; ++index )
	{
		if( Data[ index ] != 0 )
		{
			Output.emplace_back( Data[ index ] );
			++code;
			if( code != 0xFF )
			{
				continue;
			}
		}
		Output[ codeposition ] = code;
		codeposition = Output.size();
		Output.emplace_back( 0 );
		code = 1;
	}

	Output[ codeposition ] = code;
}

bool cobs_decode( std::uint8_t const *Data, std::size_t const Size, std::vector<std::uint8_t> &Output )
{
	Output.clear();
	std::size_t index = 0;

	while( index < Size )
	{
		std::uint8_t const code = Data[ index ];
		if( code == 0 )
		{
			// a zero byte can never appear inside an encoded block
			return false;
		}
		++index;
		std::size_t const runlength = static_cast<std::size_t>( code ) - 1;
		if( index + runlength > Size )
		{
			return false;
		}
		Output.insert( Output.end(), Data + index, Data + index + runlength );
		index += runlength;
		if( ( code != 0xFF ) && ( index < Size ) )
		{
			Output.emplace_back( 0 );
		}
	}

	return true;
}

void encode_frame( packet_header const &Header, std::uint8_t const *Payload, std::size_t const PayloadSize, std::vector<std::uint8_t> &Output )
{
	std::vector<std::uint8_t> packet;
	packet.reserve( protocol_header_size + PayloadSize + protocol_crc_size );

	append_uint16( packet, protocol_magic );
	packet.emplace_back( Header.protocol_major );
	packet.emplace_back( Header.protocol_minor );
	packet.emplace_back( Header.flags );
	packet.emplace_back( static_cast<std::uint8_t>( protocol_header_size ) );
	append_uint16( packet, static_cast<std::uint16_t>( Header.type ) );
	append_uint32( packet, Header.session_id );
	append_uint32( packet, Header.sequence );
	append_uint32( packet, Header.timestamp_ms );
	append_uint16( packet, static_cast<std::uint16_t>( PayloadSize ) );
	append_uint16( packet, 0 );

	if( PayloadSize > 0 )
	{
		packet.insert( packet.end(), Payload, Payload + PayloadSize );
	}

	append_uint32( packet, crc32c( packet.data(), packet.size() ) );

	cobs_encode( packet.data(), packet.size(), Output );
	Output.emplace_back( 0 );
}

void payload_writer::write_bool( bool const Value )
{
	m_data.emplace_back( Value ? 1 : 0 );
}

void payload_writer::write_uint8( std::uint8_t const Value )
{
	m_data.emplace_back( Value );
}

void payload_writer::write_int8( std::int8_t const Value )
{
	m_data.emplace_back( static_cast<std::uint8_t>( Value ) );
}

void payload_writer::write_uint16( std::uint16_t const Value )
{
	append_uint16( m_data, Value );
}

void payload_writer::write_int16( std::int16_t const Value )
{
	append_uint16( m_data, static_cast<std::uint16_t>( Value ) );
}

void payload_writer::write_uint32( std::uint32_t const Value )
{
	append_uint32( m_data, Value );
}

void payload_writer::write_int32( std::int32_t const Value )
{
	append_uint32( m_data, static_cast<std::uint32_t>( Value ) );
}

void payload_writer::write_uint64( std::uint64_t const Value )
{
	append_uint32( m_data, static_cast<std::uint32_t>( Value ) );
	append_uint32( m_data, static_cast<std::uint32_t>( Value >> 32 ) );
}

void payload_writer::write_int64( std::int64_t const Value )
{
	write_uint64( static_cast<std::uint64_t>( Value ) );
}

void payload_writer::write_float32( float const Value )
{
	std::uint32_t bits = 0;
	std::memcpy( &bits, &Value, sizeof( bits ) );
	append_uint32( m_data, bits );
}

void payload_writer::write_float64( double const Value )
{
	std::uint64_t bits = 0;
	std::memcpy( &bits, &Value, sizeof( bits ) );
	write_uint64( bits );
}

void payload_writer::write_string( std::string const &Value )
{
	auto const length = std::min( Value.size(), protocol_string_max );
	m_data.emplace_back( static_cast<std::uint8_t>( length ) );
	m_data.insert( m_data.end(), Value.begin(), Value.begin() + length );
}

void payload_writer::write_bytes( std::uint8_t const *Data, std::size_t const Size )
{
	m_data.insert( m_data.end(), Data, Data + Size );
}

void payload_writer::patch_uint16( std::size_t const Offset, std::uint16_t const Value )
{
	if( Offset + 1 >= m_data.size() )
	{
		return;
	}
	m_data[ Offset ] = static_cast<std::uint8_t>( Value );
	m_data[ Offset + 1 ] = static_cast<std::uint8_t>( Value >> 8 );
}

bool payload_reader::require( std::size_t const Size )
{
	if( m_error )
	{
		return false;
	}
	if( m_position + Size > m_size )
	{
		m_error = true;
		return false;
	}
	return true;
}

bool payload_reader::read_bool()
{
	return read_uint8() != 0;
}

std::uint8_t payload_reader::read_uint8()
{
	if( false == require( 1 ) )
	{
		return 0;
	}
	return m_data[ m_position++ ];
}

std::int8_t payload_reader::read_int8()
{
	return static_cast<std::int8_t>( read_uint8() );
}

std::uint16_t payload_reader::read_uint16()
{
	if( false == require( 2 ) )
	{
		return 0;
	}
	auto const value = extract_uint16( m_data + m_position );
	m_position += 2;
	return value;
}

std::int16_t payload_reader::read_int16()
{
	return static_cast<std::int16_t>( read_uint16() );
}

std::uint32_t payload_reader::read_uint32()
{
	if( false == require( 4 ) )
	{
		return 0;
	}
	auto const value = extract_uint32( m_data + m_position );
	m_position += 4;
	return value;
}

std::int32_t payload_reader::read_int32()
{
	return static_cast<std::int32_t>( read_uint32() );
}

std::uint64_t payload_reader::read_uint64()
{
	std::uint64_t const low = read_uint32();
	std::uint64_t const high = read_uint32();
	return low | high << 32;
}

std::int64_t payload_reader::read_int64()
{
	return static_cast<std::int64_t>( read_uint64() );
}

float payload_reader::read_float32()
{
	std::uint32_t const bits = read_uint32();
	float value = 0.0f;
	std::memcpy( &value, &bits, sizeof( value ) );
	return value;
}

double payload_reader::read_float64()
{
	std::uint64_t const bits = read_uint64();
	double value = 0.0;
	std::memcpy( &value, &bits, sizeof( value ) );
	return value;
}

std::string payload_reader::read_string( std::size_t const Sizelimit )
{
	std::size_t const length = read_uint8();
	if( length > Sizelimit )
	{
		m_error = true;
		return {};
	}
	if( false == require( length ) )
	{
		return {};
	}
	std::string value( reinterpret_cast<char const *>( m_data + m_position ), length );
	m_position += length;
	return value;
}

void frame_decoder::reset()
{
	m_buffer.clear();
	m_overrun = false;
}

void frame_decoder::set_frame_size_limit( std::size_t const Limit )
{
	m_framesizelimit = std::clamp<std::size_t>( Limit, protocol_frame_size_min, protocol_frame_size_max );
}

void frame_decoder::feed( std::uint8_t const *Data, std::size_t const Size, std::vector<decoded_packet> &Output )
{
	// worst case cobs expansion, plus the code byte
	std::size_t const encodedlimit = m_framesizelimit + m_framesizelimit / 254 + 2;

	for( std::size_t index = 0; index < Size; ++index )
	{
		std::uint8_t const byte = Data[ index ];

		if( byte == 0 )
		{
			// frame delimiter; anything gathered so far is either a frame or debris to be dropped
			if( true == m_overrun )
			{
				m_overrun = false;
			}
			else if( false == m_buffer.empty() )
			{
				if( true == cobs_decode( m_buffer.data(), m_buffer.size(), m_scratch ) )
				{
					if( m_scratch.size() < protocol_header_size + protocol_crc_size )
					{
						++m_stats.length_errors;
					}
					else if( extract_uint16( m_scratch.data() ) != protocol_magic )
					{
						++m_stats.frame_errors;
					}
					else
					{
						std::size_t const headersize = m_scratch[ 5 ];
						std::size_t const payloadlength = extract_uint16( m_scratch.data() + 20 );
						if( ( headersize < protocol_header_size ) || ( headersize + payloadlength + protocol_crc_size != m_scratch.size() ) )
						{
							++m_stats.length_errors;
						}
						else
						{
							std::uint32_t const expectedcrc = extract_uint32( m_scratch.data() + m_scratch.size() - protocol_crc_size );
							if( expectedcrc != crc32c( m_scratch.data(), m_scratch.size() - protocol_crc_size ) )
							{
								++m_stats.crc_errors;
							}
							else
							{
								decoded_packet packet;
								packet.header.protocol_major = m_scratch[ 2 ];
								packet.header.protocol_minor = m_scratch[ 3 ];
								packet.header.flags = m_scratch[ 4 ];
								packet.header.type = static_cast<message_type>( extract_uint16( m_scratch.data() + 6 ) );
								packet.header.session_id = extract_uint32( m_scratch.data() + 8 );
								packet.header.sequence = extract_uint32( m_scratch.data() + 12 );
								packet.header.timestamp_ms = extract_uint32( m_scratch.data() + 16 );
								packet.header.payload_length = static_cast<std::uint16_t>( payloadlength );
								packet.payload.assign( m_scratch.begin() + headersize, m_scratch.begin() + headersize + payloadlength );
								++m_stats.frames;
								Output.emplace_back( std::move( packet ) );
							}
						}
					}
				}
				else
				{
					++m_stats.frame_errors;
				}
			}
			m_buffer.clear();
			continue;
		}

		if( true == m_overrun )
		{
			// still waiting for the delimiter which ends the oversized frame
			continue;
		}

		m_buffer.emplace_back( byte );

		if( m_buffer.size() > encodedlimit )
		{
			++m_stats.length_errors;
			m_overrun = true;
			m_buffer.clear();
		}
	}
}

} // namespace hardware
