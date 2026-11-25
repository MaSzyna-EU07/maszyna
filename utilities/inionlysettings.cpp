/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "utilities/inionlysettings.h"

namespace
{

// puts Kept back into Current, and adds Key to Changed when they differed
template <typename Type_> void keep(Type_ &Current, Type_ const &Kept, std::string_view const Key, std::vector<std::string_view> &Changed)
{
	if (false == (Current == Kept))
	{
		Current = Kept;
		Changed.push_back(Key);
	}
}

// a switch a scenery may turn off but not on: puts Kept back into Current, and adds Key to Changed, only when the
// scenery turned it on. turned off it stays so, as before these keys were kept
void keep_off(bool &Current, bool const Kept, std::string_view const Key, std::vector<std::string_view> &Changed)
{
	if (Current && false == Kept)
	{
		Current = false;
		Changed.push_back(Key);
	}
}

} // namespace

ini_only_settings::ini_only_settings(global_settings const &Settings)
    : m_exec_on_exit{Settings.exec_on_exit}, m_screenshot_dir{Settings.screenshot_dir}, m_snapshot_dir{Settings.snapshot_dir}, m_extcam_cmd{Settings.extcam_cmd},
      m_extcam_rec{Settings.extcam_rec}, m_network_servers{Settings.network_servers}, m_network_client{Settings.network_client},
#ifdef WITH_ZMQ
      m_zmq_address{Settings.zmq_address},
#endif
      m_motiontelemetry{Settings.motiontelemetry_conf},
#ifdef WITH_UART
      m_uart{Settings.uart_conf},
#endif
      m_feedback_mode{Settings.iFeedbackMode}, m_feedback_port{Settings.iFeedbackPort}, m_python_enabled{Settings.python_enabled},
      m_luascreens_enabled{Settings.luascreens_enabled}, m_lua_fulllibraries{Settings.lua_fulllibraries}, m_wasm_enabled{Settings.wasm_enabled}
{
}

std::vector<std::string_view> ini_only_settings::restore(global_settings &Settings) const
{
	std::vector<std::string_view> changed;
	keep(Settings.exec_on_exit, m_exec_on_exit, "execonexit", changed);
	keep(Settings.screenshot_dir, m_screenshot_dir, "screenshotsdir", changed);
	keep(Settings.snapshot_dir, m_snapshot_dir, "snapshotsdir", changed);
	keep(Settings.extcam_cmd, m_extcam_cmd, "extcam.cmd", changed);
	keep(Settings.extcam_rec, m_extcam_rec, "extcam.rec", changed);
	keep(Settings.network_servers, m_network_servers, "network.server", changed);
	keep(Settings.network_client, m_network_client, "network.client", changed);
#ifdef WITH_ZMQ
	keep(Settings.zmq_address, m_zmq_address, "zmq.address", changed);
#endif
	// the structures as a whole, told apart by what turns them on and where they lead
	{
		auto &current{Settings.motiontelemetry_conf};
		if (current.enable != m_motiontelemetry.enable || current.proto != m_motiontelemetry.proto || current.address != m_motiontelemetry.address ||
		    current.port != m_motiontelemetry.port)
		{
			changed.emplace_back("motiontelemetry");
		}
		current = m_motiontelemetry;
	}
#ifdef WITH_UART
	{
		auto &current{Settings.uart_conf};
		if (current.enable != m_uart.enable || current.port != m_uart.port)
		{
			changed.emplace_back("uart");
		}
		current = m_uart;
	}
#endif
	keep(Settings.iFeedbackMode, m_feedback_mode, "feedbackmode", changed);
	keep(Settings.iFeedbackPort, m_feedback_port, "feedbackport", changed);
	// a scenery may switch a runtime off, and keep its scripts in the sandbox
	keep_off(Settings.python_enabled, m_python_enabled, "python.enabled", changed);
	keep_off(Settings.luascreens_enabled, m_luascreens_enabled, "luascreens.enabled", changed);
	keep_off(Settings.lua_fulllibraries, m_lua_fulllibraries, "lua.fulllibraries", changed);
	keep_off(Settings.wasm_enabled, m_wasm_enabled, "wasm.enabled", changed);
	return changed;
}
