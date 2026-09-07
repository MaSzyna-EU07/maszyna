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
#include <string>
#include <vector>

#include "hardware/protocol/protocol_defs.h"

// configuration of the hardware protocol v2, filled from eu07.ini
// the header deliberately doesn't pull in any transport library, so that it can be included by Globals

namespace hardware
{

struct serial_link_config
{
	std::string port;
	int baud = 115200;
};

struct config
{
	// v2 is active when at least one link is configured; uart v1 keeps working independently
	bool enable = false;
	std::vector<serial_link_config> serial_links;

	std::size_t frame_size = protocol_frame_size_default;
	int heartbeat_interval_ms = 500;
	// link is dropped after this many missed heartbeats
	int heartbeat_timeout_count = 5;

	// per connection safety limits, see the rate limiting chapter of the specification
	int max_frames_per_second = 500;
	int max_payload_bytes_per_second = 262144;
	int max_commands_per_second = 100;

	// interval of the worker thread polling loop, in milliseconds
	int poll_interval_ms = 1;
	// delay between reconnection attempts, in seconds
	float reconnect_interval = 1.0f;

	// verbose protocol logging
	bool debug = false;
	// additionally log every frame; very noisy
	bool debug_frames = false;
};

// logging switches which can be flipped at run time from the simulator debug panel
struct debug_switches
{
	std::atomic<bool> log_messages { false };
	std::atomic<bool> log_frames { false };
};

extern debug_switches debug_flags;

} // namespace hardware
