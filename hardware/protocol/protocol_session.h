/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <deque>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware/diagnostics/hardware_diagnostics.h"
#include "hardware/hardware_config.h"
#include "hardware/protocol/protocol_codec.h"
#include "hardware/registry/state_registry.h"
#include "hardware/subscriptions/subscription_manager.h"
#include "input/command.h"

// logical protocol layer of one connection
// runs on the simulation thread; the transport worker only hands over decoded frames, so nothing
// here ever blocks on hardware, and nothing outside here touches simulation state

namespace hardware
{

// sink for packets produced by the session
class session_output
{

public:
// methods
	virtual ~session_output() = default;
	virtual void send_packet( packet_header const &Header, std::vector<std::uint8_t> const &Payload ) = 0;
};

class protocol_session
{

public:
// methods
	protocol_session( session_output &Output, config const &Config );

	// drops the session, called when the transport connection is lost
	void reset();
	// consumes one decoded frame
	void handle_packet( decoded_packet const &Packet );
	// publishes the state of the current simulation frame, before the received frames are processed
	void set_snapshot( state_snapshot const &Snapshot ) { m_snapshot = Snapshot; }
	// periodic work: heartbeats, subscription updates, timeouts
	void update( double const Deltatime, state_snapshot const &Snapshot );
	// copies the current condition into the debug report
	void fill_report( link_report &Report ) const;

	// ---- diagnostics offered by the device, driven from the debug panel ----
	// starts one of the routines the device advertised; false when it isn't available
	bool start_diagnostic( std::uint8_t const Function );
	void cancel_diagnostic();
	// answers the message box the device asked the simulator to show
	void answer_prompt( std::uint8_t const Option );
	bool has_prompt() const { return m_prompt.active; }
	diagnostic_prompt const &prompt() const { return m_prompt; }
	std::vector<diagnostic_function> const &functions() const { return m_functions; }

	session_state state() const { return m_state; }
	link_state link_condition() const;
	protocol_diagnostics const &diagnostics() const { return m_diagnostics; }
	// adds the frame level error counts gathered by the transport worker since the last call
	void add_frame_errors( frame_decoder::counters const &Delta );

private:
// types
	struct transaction_result
	{
		bool accepted = false;
		error_code error = error_code::none;
	};

// methods
	void send( message_type const Type, std::uint8_t const Flags, payload_writer const &Payload );
	void send_ack( std::uint32_t const Transaction, message_type const Type );
	void send_nack( std::uint32_t const Transaction, message_type const Type, error_code const Error );

	void handle_hello( decoded_packet const &Packet );
	void handle_resolve_symbols( decoded_packet const &Packet );
	void handle_query_symbol( decoded_packet const &Packet );
	void handle_subscribe( decoded_packet const &Packet );
	void handle_unsubscribe( decoded_packet const &Packet );
	void handle_command_event( decoded_packet const &Packet );
	void handle_control_set( decoded_packet const &Packet );
	void handle_device_diagnostics( decoded_packet const &Packet );
	void handle_heartbeat( decoded_packet const &Packet );
	void handle_device_log( decoded_packet const &Packet );
	void handle_diagnostic_functions( decoded_packet const &Packet );
	void handle_diagnostic_status( decoded_packet const &Packet );
	void handle_diagnostic_prompt( decoded_packet const &Packet );
	void handle_device_answer( decoded_packet const &Packet, bool const Positive );

	void send_state_update( std::vector<state_item> const &Items, bool const Snapshot );
	void send_symbol_context_changed( std::uint32_t const Context );

	// returns the registry index for a session handle, or symbol_registry::npos
	std::size_t symbol_of_handle( std::uint16_t const Handle ) const;
	std::uint16_t handle_of_symbol( std::size_t const SymbolIndex );

	bool consume_frame_budget( std::size_t const PayloadSize );
	bool consume_command_budget();
	bool remember_transaction( std::uint32_t const Transaction, transaction_result const &Result );
	transaction_result const *recall_transaction( std::uint32_t const Transaction ) const;

	// executes a command, returns the error which should be reported to the device
	error_code execute_command( std::size_t const SymbolIndex, command_action const Action, double const Parameter1, double const Parameter2, std::uint16_t const Recipient );

// members
	session_output &m_output;
	config m_config;
	command_relay m_relay;

	session_state m_state = session_state::waiting_for_hello;
	std::uint32_t m_sessionid = 0;
	std::uint32_t m_sequence = 0;
	std::uint32_t m_lastreceivedsequence = 0;
	// transaction numbering for the few requests the simulator itself makes
	std::uint32_t m_outgoingtransaction = 0;
	bool m_sequenceseen = false;
	std::size_t m_framesize = protocol_frame_size_default;
	std::uint32_t m_capabilities = 0;
	std::uint8_t m_peermajor = 0;
	std::uint8_t m_peerminor = 0;

	std::string m_devicename;
	std::string m_devicerole;
	std::string m_deviceid;
	std::string m_firmwareversion;
	std::string m_hardwareversion;

	std::vector<std::size_t> m_handles;
	std::unordered_map<std::size_t, std::uint16_t> m_handlelookup;
	subscription_manager m_subscriptions;
	// per control sequence counters, used to drop stale continuous updates
	std::unordered_map<std::uint16_t, std::uint32_t> m_controlsequence;

	std::unordered_map<std::uint32_t, transaction_result> m_transactions;
	std::deque<std::uint32_t> m_transactionorder;

	double m_uptime = 0.0;
	double m_lastreceived = 0.0;
	double m_heartbeattimer = 0.0;
	double m_pingtimer = 0.0;
	std::uint32_t m_pingtoken = 0;
	double m_pingsent = 0.0;
	bool m_pingpending = false;

	double m_ratewindow = 0.0;
	int m_framebudget = 0;
	int m_payloadbudget = 0;
	int m_commandbudget = 0;

	std::uint32_t m_context = 0;
	bool m_contextvalid = false;
	// most recent simulator state, kept so that a subscription can be answered right away
	state_snapshot m_snapshot;

	protocol_diagnostics m_diagnostics;
	device_diagnostics m_devicediagnostics;
	std::vector<diagnostic_function> m_functions;
	diagnostic_report m_diagnostic;
	diagnostic_prompt m_prompt;
	std::deque<device_log_entry> m_log;
	rate_counter m_rxrate;
	rate_counter m_txrate;
};

} // namespace hardware
