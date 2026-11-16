#include "stdafx.h"
#include "doctest.h"

// the settings only eu07.ini sets (utilities/inionlysettings.h): a config block of a scenery is read by the parser of
// eu07.ini (global_settings::ConfigParse with config_source::scenery), which keeps these as eu07.ini set them, and puts
// them back before FinalizeConfig() acts on any. here the keeping, as ConfigParse does it around the keys of a scenery

#include "utilities/Globals.h"
#include "utilities/inionlysettings.h"

#include <algorithm>
#include <memory>
#include <string_view>
#include <vector>

namespace {

bool listed(std::vector<std::string_view> const &Keys, std::string_view const Key) {
	return std::ranges::find(Keys, Key) != Keys.end();
}

} // namespace

TEST_CASE("ini only settings: a scenery's config block doesn't give its scripts all libraries") {
	auto const settings{std::make_unique<global_settings>()};
	settings->lua_fulllibraries = false;
	ini_only_settings const ini{*settings};
	// "config lua.fulllibraries yes endconfig" in the .scn
	settings->lua_fulllibraries = true;
	auto const changed{ini.restore(*settings)};
	CHECK_FALSE(settings->lua_fulllibraries);
	CHECK(changed == std::vector<std::string_view>{"lua.fulllibraries"});
}

TEST_CASE("ini only settings: what eu07.ini turned on stays on") {
	auto const settings{std::make_unique<global_settings>()};
	settings->lua_fulllibraries = true;
	settings->python_enabled = false;
	ini_only_settings const ini{*settings};
	settings->lua_fulllibraries = false;
	settings->python_enabled = true;
	auto const changed{ini.restore(*settings)};
	CHECK(settings->lua_fulllibraries);
	CHECK_FALSE(settings->python_enabled);
	CHECK(changed.size() == 2);
}

TEST_CASE("ini only settings: a scenery runs no program, opens no connection and switches no runtime back on") {
	auto const settings{std::make_unique<global_settings>()};
	settings->luascreens_enabled = false;
	settings->wasm_enabled = false;
	settings->python_enabled = false;
	ini_only_settings const ini{*settings};

	settings->exec_on_exit = "cmd /c del something";
	settings->extcam_cmd = "evil.exe";
	settings->extcam_rec = "evil.exe";
	settings->screenshot_dir = "C:/Users/Public";
	settings->network_servers.emplace_back("tcp", "0.0.0.0:7424");
	settings->network_client.emplace("tcp", "example.com:7424");
	settings->motiontelemetry_conf.enable = true;
	settings->motiontelemetry_conf.address = "example.com";
	settings->iFeedbackMode = 4;
	settings->iFeedbackPort = 0x378;
	settings->luascreens_enabled = true;
	settings->wasm_enabled = true;
	settings->python_enabled = true;
#ifdef WITH_UART
	settings->uart_conf.enable = true;
	settings->uart_conf.port = "COM1";
#endif
#ifdef WITH_ZMQ
	settings->zmq_address = "tcp://*:5555";
#endif
	// and a key a scenery may set
	settings->fFriction = 0.5;

	auto const changed{ini.restore(*settings)};
	CHECK(settings->exec_on_exit.empty());
	CHECK(settings->extcam_cmd.empty());
	CHECK(settings->extcam_rec.empty());
	CHECK(settings->screenshot_dir.empty());
	CHECK(settings->network_servers.empty());
	CHECK_FALSE(settings->network_client.has_value());
	CHECK_FALSE(settings->motiontelemetry_conf.enable);
	CHECK(settings->motiontelemetry_conf.address.empty());
	CHECK(settings->iFeedbackMode == 1);
	CHECK(settings->iFeedbackPort == 0);
	CHECK_FALSE(settings->luascreens_enabled);
	CHECK_FALSE(settings->wasm_enabled);
	CHECK_FALSE(settings->python_enabled);
	for (auto const *key : {"execonexit", "extcam.cmd", "extcam.rec", "screenshotsdir", "network.server", "network.client", "motiontelemetry",
	                        "feedbackmode", "feedbackport", "luascreens.enabled", "wasm.enabled", "python.enabled"}) {
		CAPTURE(key);
		CHECK(listed(changed, key));
	}
#ifdef WITH_UART
	CHECK_FALSE(settings->uart_conf.enable);
	CHECK(settings->uart_conf.port.empty());
	CHECK(listed(changed, "uart"));
#endif
#ifdef WITH_ZMQ
	CHECK(settings->zmq_address.empty());
	CHECK(listed(changed, "zmq.address"));
#endif
	// the rest is the scenery's to set
	CHECK(settings->fFriction == doctest::Approx(0.5));
}

TEST_CASE("ini only settings: nothing changed, nothing to tell") {
	auto const settings{std::make_unique<global_settings>()};
	settings->exec_on_exit = "notify.exe";
	ini_only_settings const ini{*settings};
	CHECK(ini.restore(*settings).empty());
	CHECK(settings->exec_on_exit == "notify.exe");
}
