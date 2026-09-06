/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "hardware/transport/serial_transport.h"

namespace hardware
{

serial_transport::serial_transport( std::string Port, int const Baud ) : m_portname( std::move( Port ) ), m_baud( Baud )
{
}

serial_transport::~serial_transport()
{
	disconnect();
}

std::string serial_transport::endpoint() const
{
	return m_portname + " @ " + std::to_string( m_baud );
}

bool serial_transport::connect()
{
	disconnect();

	sp_port *port = nullptr;
	if( sp_get_port_by_name( m_portname.c_str(), &port ) != SP_OK )
	{
		m_lasterror = "port not found";
		return false;
	}

	if( sp_open( port, static_cast<sp_mode>( SP_MODE_READ | SP_MODE_WRITE ) ) != SP_OK )
	{
		m_lasterror = "cannot open port";
		sp_free_port( port );
		return false;
	}

	sp_port_config *config = nullptr;
	if( ( sp_new_config( &config ) != SP_OK )
	 || ( sp_set_config_baudrate( config, m_baud ) != SP_OK )
	 || ( sp_set_config_flowcontrol( config, SP_FLOWCONTROL_NONE ) != SP_OK )
	 || ( sp_set_config_bits( config, 8 ) != SP_OK )
	 || ( sp_set_config_stopbits( config, 1 ) != SP_OK )
	 || ( sp_set_config_parity( config, SP_PARITY_NONE ) != SP_OK )
	 || ( sp_set_config( port, config ) != SP_OK ) )
	{
		m_lasterror = "cannot configure port";
		if( config != nullptr )
		{
			sp_free_config( config );
		}
		sp_close( port );
		sp_free_port( port );
		return false;
	}
	sp_free_config( config );

	if( sp_flush( port, SP_BUF_BOTH ) != SP_OK )
	{
		m_lasterror = "cannot flush port";
		sp_close( port );
		sp_free_port( port );
		return false;
	}

	m_port = port;
	m_lasterror.clear();
	return true;
}

void serial_transport::disconnect()
{
	if( m_port == nullptr )
	{
		return;
	}
	sp_close( m_port );
	sp_free_port( m_port );
	m_port = nullptr;
}

int serial_transport::read( std::uint8_t *Buffer, std::size_t const Size )
{
	if( m_port == nullptr )
	{
		return -1;
	}

	int const waiting = sp_input_waiting( m_port );
	if( waiting < 0 )
	{
		m_lasterror = "read failed";
		return -1;
	}
	if( waiting == 0 )
	{
		return 0;
	}

	int const result = sp_nonblocking_read( m_port, Buffer, std::min( Size, static_cast<std::size_t>( waiting ) ) );
	if( result < 0 )
	{
		m_lasterror = "read failed";
		return -1;
	}
	return result;
}

int serial_transport::write( std::uint8_t const *Data, std::size_t const Size )
{
	if( m_port == nullptr )
	{
		return -1;
	}

	int const result = sp_nonblocking_write( m_port, Data, Size );
	if( result < 0 )
	{
		m_lasterror = "write failed";
		return -1;
	}
	return result;
}

std::vector<std::string> list_serial_ports()
{
	std::vector<std::string> result;
	sp_port **ports = nullptr;
	if( sp_list_ports( &ports ) != SP_OK )
	{
		return result;
	}
	for( int index = 0; ports[ index ] != nullptr; ++index )
	{
		result.emplace_back( sp_get_port_name( ports[ index ] ) );
	}
	sp_free_port_list( ports );
	return result;
}

} // namespace hardware
