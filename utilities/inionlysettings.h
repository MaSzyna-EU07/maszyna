/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "utilities/Globals.h"

#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

// the settings only eu07.ini sets. a scenery's config block is read by the parser of eu07.ini
// (state_serializer::deserialize_config, global_settings::ConfigParse), and with these keys it would run a program
// (execonexit, extcam.cmd, extcam.rec), write the screenshots or the snapshots elsewhere (screenshotsdir, snapshotsdir), open connections and ports
// (network.server, network.client, zmq.address, motiontelemetry, uart, feedbackmode, feedbackport), switch back on a
// script runtime the player switched off (python.enabled, luascreens.enabled, wasm.enabled), take its own scripts
// out of the sandbox (lua.fulllibraries) or drop the odometer states the player keeps (odometer.expiry). they keep
// what eu07.ini set; a scenery may still switch a runtime off, or keep its scripts in the sandbox when eu07.ini didn't,
// as it always could
class ini_only_settings
{
  public:
	// what Settings holds now, as eu07.ini set it
	explicit ini_only_settings(global_settings const &Settings);

	// puts back into Settings what changed since, and gives the keys of eu07.ini whose settings changed
	std::vector<std::string_view> restore(global_settings &Settings) const;

  private:
	std::string m_exec_on_exit;
	std::string m_screenshot_dir;
	std::string m_snapshot_dir;
	std::string m_extcam_cmd;
	std::string m_extcam_rec;
	std::vector<std::pair<std::string, std::string>> m_network_servers;
	std::optional<std::pair<std::string, std::string>> m_network_client;
#ifdef WITH_ZMQ
	std::string m_zmq_address;
#endif
	motiontelemetry::conf_t m_motiontelemetry;
#ifdef WITH_UART
	uart_input::conf_t m_uart;
#endif
	int m_feedback_mode;
	int m_feedback_port;
	bool m_python_enabled;
	bool m_luascreens_enabled;
	bool m_lua_fulllibraries;
	bool m_wasm_enabled;
	int m_odometer_expiry;
};
