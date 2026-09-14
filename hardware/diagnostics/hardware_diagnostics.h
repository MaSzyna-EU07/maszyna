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

// diagnostics gathered per connection, presented in the simulator debug panel
// transport and protocol problems are counted separately, as they usually have different causes

namespace hardware
{

// sliding estimate of events per second
class rate_counter
{

public:
// methods
	void add( std::uint64_t const Count = 1 ) { m_accumulator += Count; }
	void update( double const Deltatime );
	float rate() const { return m_rate; }

private:
// members
	std::uint64_t m_accumulator = 0;
	double m_window = 0.0;
	float m_rate = 0.0f;
};

struct transport_diagnostics
{
	bool connected = false;
	std::string endpoint;
	std::string last_error;
	std::uint64_t rx_bytes = 0;
	std::uint64_t tx_bytes = 0;
	std::uint64_t transport_errors = 0;
	std::uint64_t reconnects = 0;
};

struct protocol_diagnostics
{
	std::uint64_t rx_frames = 0;
	std::uint64_t tx_frames = 0;
	std::uint64_t crc_errors = 0;
	std::uint64_t frame_errors = 0;
	std::uint64_t length_errors = 0;
	std::uint64_t version_errors = 0;
	std::uint64_t unknown_messages = 0;
	std::uint64_t unknown_handles = 0;
	std::uint64_t unknown_symbols = 0;
	std::uint64_t duplicate_commands = 0;
	std::uint64_t sequence_gaps = 0;
	std::uint64_t acks = 0;
	std::uint64_t nacks = 0;
	std::uint64_t rate_limited = 0;
	std::uint64_t state_updates = 0;
	std::uint64_t commands_executed = 0;
	std::uint64_t controls_applied = 0;
	// seconds since the last valid frame arrived
	double last_rx_age = 0.0;
	double round_trip_time_ms = 0.0;
	double connection_uptime = 0.0;
	float rx_rate = 0.0f;
	float tx_rate = 0.0f;
	int missed_heartbeats = 0;
};

// optional information reported by the device itself
struct device_diagnostics
{
	bool available = false;
	std::uint32_t uptime_ms = 0;
	float supply_voltage = 0.0f;
	float temperature = 0.0f;
	std::uint32_t rx_overflows = 0;
	std::uint32_t tx_overflows = 0;
	std::uint32_t watchdog_resets = 0;
	std::string firmware_build;
};

// one text line sent by the device
struct device_log_entry
{
	log_severity severity = log_severity::info;
	std::uint32_t device_uptime_ms = 0;
	std::string text;
};

// a diagnostic routine the device offers, listed in the debug panel
struct diagnostic_function
{
	std::uint8_t id = 0;
	std::string name;
};

// progress of the routine currently running on the device
struct diagnostic_report
{
	diagnostic_state state = diagnostic_state::idle;
	std::uint8_t function = 0;
	std::uint8_t progress = 0;
	std::string message;
};

// a message box the device asked the simulator to show
struct diagnostic_prompt
{
	bool active = false;
	std::uint32_t id = 0;
	std::string text;
	std::vector<std::string> options;
};

// everything the debug panel needs about one connection, copied out once per frame
struct link_report
{
	std::string transport_kind;
	std::string endpoint;
	std::string device_name;
	std::string device_role;
	std::string device_id;
	std::string firmware_version;
	link_state state = link_state::disconnected;
	session_state session = session_state::disconnected;
	std::uint32_t session_id = 0;
	std::uint8_t protocol_major = 0;
	std::uint8_t protocol_minor = 0;
	std::size_t handles = 0;
	std::size_t subscriptions = 0;
	std::size_t frame_size = 0;
	// false when the link was switched off from the debug panel, which releases the port
	bool enabled = true;
	// index of the link, as the manager knows it
	std::size_t index = 0;
	transport_diagnostics transport;
	protocol_diagnostics protocol;
	device_diagnostics device;
	std::vector<diagnostic_function> functions;
	diagnostic_report diagnostic;
	diagnostic_prompt prompt;
	std::vector<device_log_entry> log;
};

} // namespace hardware
