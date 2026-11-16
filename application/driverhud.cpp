/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/driverhud.h"

#ifdef WITH_RMLUI

#include <array>
#include <cmath>
#include <format>
#include <optional>
#include <string_view>

#include <RmlUi/Core.h>

#include "application/driveruipanels.h"
#include "application/rmluidocument.h"
#include "application/rmluilayer.h"
#include "input/command.h"
#include "utilities/translation.h"

namespace
{

// staticFiles/ui/driverhud.rml of the sources, written out by CMake as the bytes of driverhud_rml
#include "driverhud_rml.inc"

// the values of the data model of the document, see staticFiles/ui/driverhud.rml
struct hud_values
{
	bool shown{false};
	bool expanded{false};
	int speed{0};
	int limit{0};
	bool next_limit_shown{false};
	int next_limit{0};
	Rml::String next_limit_distance;
	bool grade_shown{false};
	Rml::String grade;
	Rml::String throttle;
	Rml::String direction;
	Rml::String brake;
	bool slipping{false};
	Rml::String brake_pressure;
	Rml::String pipe_pressure;
	bool alerter{false};
	bool cab_signal{false};
	Rml::String alerter_text;
	Rml::String shp_text;
	bool hold_up_shown{false};
	Rml::String hold_up;
	bool autopilot_shown{false};
	bool autopilot{false};
};

void bind(Rml::DataModelConstructor &Constructor, hud_values &Values)
{
	Constructor.Bind("shown", &Values.shown);
	Constructor.Bind("expanded", &Values.expanded);
	Constructor.Bind("speed", &Values.speed);
	Constructor.Bind("limit", &Values.limit);
	Constructor.Bind("next_limit_shown", &Values.next_limit_shown);
	Constructor.Bind("next_limit", &Values.next_limit);
	Constructor.Bind("next_limit_distance", &Values.next_limit_distance);
	Constructor.Bind("grade_shown", &Values.grade_shown);
	Constructor.Bind("grade", &Values.grade);
	Constructor.Bind("throttle", &Values.throttle);
	Constructor.Bind("direction", &Values.direction);
	Constructor.Bind("brake", &Values.brake);
	Constructor.Bind("slipping", &Values.slipping);
	Constructor.Bind("brake_pressure", &Values.brake_pressure);
	Constructor.Bind("pipe_pressure", &Values.pipe_pressure);
	Constructor.Bind("alerter", &Values.alerter);
	Constructor.Bind("cab_signal", &Values.cab_signal);
	Constructor.Bind("alerter_text", &Values.alerter_text);
	Constructor.Bind("shp_text", &Values.shp_text);
	Constructor.Bind("hold_up_shown", &Values.hold_up_shown);
	Constructor.Bind("hold_up", &Values.hold_up);
	Constructor.Bind("autopilot_shown", &Values.autopilot_shown);
	Constructor.Bind("autopilot", &Values.autopilot);
}

// a text of the translations without the spaces it's laid out with in the panel of imgui
Rml::String trimmed(std::string_view const Text)
{
	auto const first{Text.find_first_not_of(' ')};
	if (first == std::string_view::npos)
		return {};
	auto const last{Text.find_last_not_of(' ')};
	return Rml::String{Text.substr(first, last - first + 1)};
}

void fill(hud_values &Values, driving_aid::reading const &Aid)
{
	Values.speed = Aid.speed;
	Values.limit = Aid.speed_limit;
	Values.next_limit_shown = Aid.ahead.limit != Aid.speed_limit;
	Values.next_limit = Aid.ahead.limit;
	Values.next_limit_distance = std::format("{:.1f}", Aid.ahead.distance * 0.001);
	Values.grade_shown = std::abs(Aid.grade) >= driving_aid::grade_shown;
	Values.grade = std::format("{:+.1f}", Aid.grade);
	Values.throttle = std::format("{}+{}", Aid.throttle, Aid.throttle_secondary);
	Values.direction = Rml::String(1, Aid.direction);
	Values.brake = std::format("{:.1f}+{:.0f}", Aid.brake, Aid.local_brake);
	Values.slipping = Aid.slipping;
	Values.brake_pressure = std::format("{:.0f}", Aid.brake_pressure);
	Values.pipe_pressure = std::format("{:.0f}", Aid.pipe_pressure);
	Values.alerter = Aid.alerter;
	Values.cab_signal = Aid.cab_signal;
	// the names the panel of imgui shows them by, of the translations
	Values.alerter_text = trimmed(STR("!ALERTER! "));
	Values.shp_text = trimmed(STR("!SHP!"));
	Values.hold_up = trimmed(driving_aid_hold_up(Aid));
	Values.hold_up_shown = false == Values.hold_up.empty();
	Values.autopilot_shown = Aid.autopilot_allowed;
	Values.autopilot = Aid.autopilot;
}

// the vehicle handed to the autopilot, or taken back from it, as the panel of imgui does it
void toggle_autopilot(bool const Autopilot)
{
	command_relay relay;
	relay.post(Autopilot ? user_command::aidriverdisable : user_command::aidriverenable, 0.0, 0.0, GLFW_PRESS, 0);
}

} // namespace

struct driver_hud::state
{
	hud_values values;
	rmlui::document document;
};

driver_hud::driver_hud() = default;

driver_hud::~driver_hud() = default;

bool driver_hud::open()
{
	auto opened{std::make_unique<state>()};
	auto const bound = [values = &opened->values](Rml::DataModelConstructor &Constructor) {
		bind(Constructor, *values);
		Constructor.BindEventCallback("toggle_autopilot", [values](Rml::DataModelHandle, Rml::Event &, Rml::VariantList const &) { toggle_autopilot(values->autopilot); });
	};
	if (false == opened->document.open("driver_hud", "ui/driverhud.rml", driverhud_rml, bound))
		return false;
	// shown without taking the focus, the keys go on to the simulation
	opened->document.show(false);
	m_state = std::move(opened);
	return true;
}

bool driver_hud::update(bool const Visible, bool const Expanded)
{
	if (m_failed || false == rmlui::active())
		return false;
	if (m_state == nullptr && false == open())
	{
		m_failed = true;
		return false;
	}
	auto &values{m_state->values};
	auto const aid{Visible ? drivingaid_panel::read() : std::nullopt};
	values.shown = aid.has_value();
	values.expanded = Expanded;
	if (aid.has_value())
		fill(values, *aid);
	m_state->document.dirty();
	return true;
}

#else

struct driver_hud::state
{
};

driver_hud::driver_hud() = default;

driver_hud::~driver_hud() = default;

bool driver_hud::open()
{
	return false;
}

bool driver_hud::update(bool, bool)
{
	return false;
}

#endif
