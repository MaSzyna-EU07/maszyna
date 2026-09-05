/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "hardware/diagnostics/hardware_diagnostics.h"
#include "hardware/hardware_config.h"
#include "hardware/protocol/protocol_session.h"
#include "hardware/transport/hardware_transport.h"

// owner of every hardware protocol v2 connection, regardless of the transport behind it
//
// threading: a single worker thread does all of the blocking work, that is opening ports, reading
// and writing bytes and turning them into frames. the simulation thread only exchanges ready made
// frames with it through small locked queues, and is the only one which ever touches simulation
// state. a device which stops responding, floods the link or disappears therefore cannot stall the
// simulation

namespace hardware
{

class hardware_link : public session_output
{

public:
// methods
	hardware_link( std::unique_ptr<hardware_transport> Transport, config const &Config );
	~hardware_link() override;

	// worker thread: services the transport
	void service();
	// simulation thread: processes received frames and runs the session
	void update( double const Deltatime, state_snapshot const &Snapshot );
	// simulation thread: the report gathered during the last update
	link_report const &report() const { return m_report; }

	void send_packet( packet_header const &Header, std::vector<std::uint8_t> const &Payload ) override;

private:
// members
	std::unique_ptr<hardware_transport> m_transport;
	config m_config;
	protocol_session m_session;
	link_report m_report;

	// worker thread only
	frame_decoder m_decoder;
	std::vector<std::uint8_t> m_readbuffer;
	std::vector<std::uint8_t> m_writebuffer;
	std::chrono::steady_clock::time_point m_lastconnectattempt;

	// simulation thread only
	frame_decoder::counters m_lastframeerrors;

	// shared
	std::mutex m_mutex;
	std::deque<decoded_packet> m_rxqueue;
	std::deque<std::vector<std::uint8_t>> m_txqueue;
	transport_diagnostics m_transportstatus;
	frame_decoder::counters m_framecounters;
	bool m_transportdropped = false;
};

class hardware_manager
{

public:
// methods
	explicit hardware_manager( config const &Config );
	~hardware_manager();

	// called once per simulation frame from the input polling code
	void update( double const Deltatime );

	std::vector<link_report> const &reports() const { return m_reports; }
	bool active() const { return false == m_links.empty(); }

	// the running instance, if any; used by the debug panel
	static hardware_manager *instance() { return s_instance; }

private:
// methods
	void worker();

// members
	static hardware_manager *s_instance;

	config m_config;
	std::vector<std::unique_ptr<hardware_link>> m_links;
	std::vector<link_report> m_reports;
	state_snapshot m_snapshot;
	std::thread m_thread;
	std::atomic<bool> m_quit { false };
};

} // namespace hardware
