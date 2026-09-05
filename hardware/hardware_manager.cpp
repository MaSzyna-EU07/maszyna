/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "hardware/hardware_manager.h"

#include "utilities/Logs.h"

#ifdef WITH_UART
#include "hardware/transport/serial_transport.h"
#endif

namespace hardware
{

namespace
{

// received frames waiting to be picked up by the simulation thread
constexpr std::size_t rx_queue_limit = 256;
// frames waiting to be pushed out by the worker thread
constexpr std::size_t tx_queue_limit = 512;
constexpr std::size_t read_chunk_size = 4096;

} // anonymous namespace

debug_switches debug_flags;

hardware_manager *hardware_manager::s_instance = nullptr;

hardware_link::hardware_link( std::unique_ptr<hardware_transport> Transport, config const &Config ) : m_transport( std::move( Transport ) ), m_config( Config ), m_session( *this, Config )
{
	m_readbuffer.resize( read_chunk_size );
	m_decoder.set_frame_size_limit( m_config.frame_size );
	m_lastconnectattempt = std::chrono::steady_clock::now() - std::chrono::seconds( 10 );

	m_report.transport_kind = m_transport->kind();
	m_report.endpoint = m_transport->endpoint();
	m_transportstatus.endpoint = m_report.endpoint;
}

hardware_link::~hardware_link()
{
	m_transport->disconnect();
}

void hardware_link::send_packet( packet_header const &Header, std::vector<std::uint8_t> const &Payload )
{
	std::vector<std::uint8_t> frame;
	encode_frame( Header, Payload.data(), Payload.size(), frame );

	std::lock_guard<std::mutex> lock( m_mutex );
	if( m_txqueue.size() >= tx_queue_limit )
	{
		// the device isn't keeping up; the oldest update is the least interesting one
		m_txqueue.pop_front();
	}
	m_txqueue.emplace_back( std::move( frame ) );
}

void hardware_link::service()
{
	if( false == m_transport->connected() )
	{
		auto const now = std::chrono::steady_clock::now();
		if( std::chrono::duration<float>( now - m_lastconnectattempt ).count() < m_config.reconnect_interval )
		{
			return;
		}
		m_lastconnectattempt = now;

		if( true == m_transport->connect() )
		{
			m_decoder.reset();
			std::lock_guard<std::mutex> lock( m_mutex );
			m_rxqueue.clear();
			m_txqueue.clear();
			m_transportstatus.connected = true;
			m_transportstatus.last_error.clear();
			++m_transportstatus.reconnects;
		}
		else
		{
			std::lock_guard<std::mutex> lock( m_mutex );
			if( true == m_transportstatus.last_error.empty() )
			{
				++m_transportstatus.transport_errors;
			}
			m_transportstatus.connected = false;
			m_transportstatus.last_error = m_transport->last_error();
		}
		return;
	}

	bool failed = false;

	// inbound
	std::vector<decoded_packet> packets;
	int const received = m_transport->read( m_readbuffer.data(), m_readbuffer.size() );
	if( received < 0 )
	{
		failed = true;
	}
	else if( received > 0 )
	{
		m_decoder.feed( m_readbuffer.data(), static_cast<std::size_t>( received ), packets );
	}

	// outbound; partially written frames are kept until the port accepts the rest
	if( false == failed )
	{
		if( true == m_writebuffer.empty() )
		{
			std::lock_guard<std::mutex> lock( m_mutex );
			while( ( false == m_txqueue.empty() ) && ( m_writebuffer.size() < read_chunk_size ) )
			{
				auto const &frame = m_txqueue.front();
				m_writebuffer.insert( m_writebuffer.end(), frame.begin(), frame.end() );
				m_txqueue.pop_front();
			}
		}

		if( false == m_writebuffer.empty() )
		{
			int const written = m_transport->write( m_writebuffer.data(), m_writebuffer.size() );
			if( written < 0 )
			{
				failed = true;
			}
			else if( written > 0 )
			{
				m_writebuffer.erase( m_writebuffer.begin(), m_writebuffer.begin() + written );
				std::lock_guard<std::mutex> lock( m_mutex );
				m_transportstatus.tx_bytes += written;
			}
		}
	}

	if( true == failed )
	{
		m_transport->disconnect();
		m_writebuffer.clear();
		m_decoder.reset();
		std::lock_guard<std::mutex> lock( m_mutex );
		m_transportstatus.connected = false;
		++m_transportstatus.transport_errors;
		m_transportstatus.last_error = m_transport->last_error();
		m_transportdropped = true;
		m_rxqueue.clear();
		m_txqueue.clear();
		return;
	}

	std::lock_guard<std::mutex> lock( m_mutex );
	if( received > 0 )
	{
		m_transportstatus.rx_bytes += received;
	}
	m_framecounters = m_decoder.stats();
	for( auto &packet : packets )
	{
		if( m_rxqueue.size() >= rx_queue_limit )
		{
			m_rxqueue.pop_front();
		}
		m_rxqueue.emplace_back( std::move( packet ) );
	}
}

void hardware_link::update( double const Deltatime, state_snapshot const &Snapshot )
{
	std::deque<decoded_packet> packets;
	transport_diagnostics status;
	frame_decoder::counters counters;
	bool dropped = false;

	{
		std::lock_guard<std::mutex> lock( m_mutex );
		packets.swap( m_rxqueue );
		status = m_transportstatus;
		counters = m_framecounters;
		dropped = m_transportdropped;
		m_transportdropped = false;
	}

	if( true == dropped )
	{
		m_session.reset();
	}

	frame_decoder::counters delta;
	delta.frames = counters.frames - m_lastframeerrors.frames;
	delta.crc_errors = counters.crc_errors - m_lastframeerrors.crc_errors;
	delta.frame_errors = counters.frame_errors - m_lastframeerrors.frame_errors;
	delta.length_errors = counters.length_errors - m_lastframeerrors.length_errors;
	m_lastframeerrors = counters;
	m_session.add_frame_errors( delta );

	for( auto const &packet : packets )
	{
		m_session.handle_packet( packet );
	}

	m_session.update( Deltatime, Snapshot );

	m_report.transport = status;
	m_report.state = (
	    false == status.connected ? link_state::disconnected :
	    m_session.link_condition() );
	m_session.fill_report( m_report );
}

hardware_manager::hardware_manager( config const &Config ) : m_config( Config )
{
#ifdef WITH_UART
	for( auto const &link : m_config.serial_links )
	{
		m_links.emplace_back( std::make_unique<hardware_link>( std::make_unique<serial_transport>( link.port, link.baud ), m_config ) );
		WriteLog( "hardware: protocol v2 link on " + link.port + " @ " + std::to_string( link.baud ) );
	}
#endif

	debug_flags.log_messages = m_config.debug;
	debug_flags.log_frames = m_config.debug_frames;

	m_reports.resize( m_links.size() );

	if( false == m_links.empty() )
	{
		m_thread = std::thread( &hardware_manager::worker, this );
	}

	s_instance = this;
}

hardware_manager::~hardware_manager()
{
	s_instance = nullptr;
	m_quit = true;
	if( true == m_thread.joinable() )
	{
		m_thread.join();
	}
	m_links.clear();
}

void hardware_manager::worker()
{
	auto const interval = std::chrono::milliseconds( std::max( 1, m_config.poll_interval_ms ) );

	while( false == m_quit )
	{
		for( auto &link : m_links )
		{
			link->service();
		}
		std::this_thread::sleep_for( interval );
	}
}

void hardware_manager::update()
{
	if( true == m_links.empty() )
	{
		return;
	}

	// the input polling code doesn't carry a time step, so the sessions get their own
	auto const now = std::chrono::steady_clock::now();
	double deltatime = 0.0;
	if( true == m_updatestarted )
	{
		deltatime = std::clamp( std::chrono::duration<double>( now - m_lastupdate ).count(), 0.0, 0.5 );
	}
	m_updatestarted = true;
	m_lastupdate = now;

	state_registry::capture( m_snapshot );

	for( std::size_t index = 0; index < m_links.size(); ++index )
	{
		m_links[ index ]->update( deltatime, m_snapshot );
		m_reports[ index ] = m_links[ index ]->report();
	}
}

} // namespace hardware
