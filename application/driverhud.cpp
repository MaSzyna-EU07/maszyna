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
#include <filesystem>
#include <format>
#include <fstream>
#include <iterator>
#include <optional>
#include <string_view>
#include <utility>
#include <vector>

#include <RmlUi/Core.h>

#include "application/driveruipanels.h"
#include "application/hudlayout.h"
#include "application/rmluidocument.h"
#include "application/rmluilayer.h"
#include "input/command.h"
#include "simulation/simulationtime.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/translation.h"
#include "utilities/utilities.h"

namespace
{

// staticFiles/ui/driverhud.rml of the sources, written out by CMake as the bytes of driverhud_rml
#include "driverhud_rml.inc"

// the parts of the hud the player places: their ids in the document and their names in hud_layout.ini
constexpr std::array part_ids{"aid", "timetable", "scenario"};
// how close an edge of a part snaps to another edge, dp
constexpr float snap_range{12.f};

// a stop of the timetable, as the document lists it
struct stop_row
{
	Rml::String vmax; // empty when it's the same as before
	Rml::String station;
	Rml::String km; // empty when not given
	Rml::String ware;
	Rml::String arrival; // empty when passing through
	Rml::String departure;
	Rml::String travel;
	int tracks{1};
	Rml::String state; // normal, exchange, can_depart or at_stop

	bool operator==(stop_row const &) const = default;
};

// a hint of the scenario
struct hint_row
{
	Rml::String text;
	bool done{false};

	bool operator==(hint_row const &) const = default;
};

// the values of the data model of the document, see staticFiles/ui/driverhud.rml
struct hud_values
{
	bool editing{false};
	// the driving aid
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
	// the timetable
	bool tt_shown{false};
	bool tt_expanded{false};
	Rml::String tt_time;
	Rml::String tt_relation;
	Rml::String tt_next;
	bool tt_consist{false};
	int tt_mass_specified{0};
	int tt_mass_actual{0};
	int tt_length{0};
	bool tt_has_table{false};
	bool tt_late{false};
	Rml::String tt_delay;
	std::vector<stop_row> tt_stops;
	// the scenario
	bool sc_shown{false};
	Rml::String sc_task;
	Rml::String sc_assignment;
	std::vector<hint_row> sc_hints;
};

void bind_aid(Rml::DataModelConstructor &Constructor, rmlui::bound_values<hud_values> &Values)
{
	Values.bind(Constructor, "shown", &hud_values::shown);
	Values.bind(Constructor, "expanded", &hud_values::expanded);
	Values.bind(Constructor, "speed", &hud_values::speed);
	Values.bind(Constructor, "limit", &hud_values::limit);
	Values.bind(Constructor, "next_limit_shown", &hud_values::next_limit_shown);
	Values.bind(Constructor, "next_limit", &hud_values::next_limit);
	Values.bind(Constructor, "next_limit_distance", &hud_values::next_limit_distance);
	Values.bind(Constructor, "grade_shown", &hud_values::grade_shown);
	Values.bind(Constructor, "grade", &hud_values::grade);
	Values.bind(Constructor, "throttle", &hud_values::throttle);
	Values.bind(Constructor, "direction", &hud_values::direction);
	Values.bind(Constructor, "brake", &hud_values::brake);
	Values.bind(Constructor, "slipping", &hud_values::slipping);
	Values.bind(Constructor, "brake_pressure", &hud_values::brake_pressure);
	Values.bind(Constructor, "pipe_pressure", &hud_values::pipe_pressure);
	Values.bind(Constructor, "alerter", &hud_values::alerter);
	Values.bind(Constructor, "cab_signal", &hud_values::cab_signal);
	Values.bind(Constructor, "alerter_text", &hud_values::alerter_text);
	Values.bind(Constructor, "shp_text", &hud_values::shp_text);
	Values.bind(Constructor, "hold_up_shown", &hud_values::hold_up_shown);
	Values.bind(Constructor, "hold_up", &hud_values::hold_up);
	Values.bind(Constructor, "autopilot_shown", &hud_values::autopilot_shown);
	Values.bind(Constructor, "autopilot", &hud_values::autopilot);
}

void bind_timetable(Rml::DataModelConstructor &Constructor, rmlui::bound_values<hud_values> &Values)
{
	if (auto row{Constructor.RegisterStruct<stop_row>()})
	{
		row.RegisterMember("vmax", &stop_row::vmax);
		row.RegisterMember("station", &stop_row::station);
		row.RegisterMember("km", &stop_row::km);
		row.RegisterMember("ware", &stop_row::ware);
		row.RegisterMember("arrival", &stop_row::arrival);
		row.RegisterMember("departure", &stop_row::departure);
		row.RegisterMember("travel", &stop_row::travel);
		row.RegisterMember("tracks", &stop_row::tracks);
		row.RegisterMember("state", &stop_row::state);
	}
	Constructor.RegisterArray<std::vector<stop_row>>();
	Values.bind(Constructor, "tt_shown", &hud_values::tt_shown);
	Values.bind(Constructor, "tt_expanded", &hud_values::tt_expanded);
	Values.bind(Constructor, "tt_time", &hud_values::tt_time);
	Values.bind(Constructor, "tt_relation", &hud_values::tt_relation);
	Values.bind(Constructor, "tt_next", &hud_values::tt_next);
	Values.bind(Constructor, "tt_consist", &hud_values::tt_consist);
	Values.bind(Constructor, "tt_mass_specified", &hud_values::tt_mass_specified);
	Values.bind(Constructor, "tt_mass_actual", &hud_values::tt_mass_actual);
	Values.bind(Constructor, "tt_length", &hud_values::tt_length);
	Values.bind(Constructor, "tt_has_table", &hud_values::tt_has_table);
	Values.bind(Constructor, "tt_late", &hud_values::tt_late);
	Values.bind(Constructor, "tt_delay", &hud_values::tt_delay);
	Values.bind(Constructor, "tt_stops", &hud_values::tt_stops);
}

void bind_scenario(Rml::DataModelConstructor &Constructor, rmlui::bound_values<hud_values> &Values)
{
	if (auto row{Constructor.RegisterStruct<hint_row>()})
	{
		row.RegisterMember("text", &hint_row::text);
		row.RegisterMember("done", &hint_row::done);
	}
	Constructor.RegisterArray<std::vector<hint_row>>();
	Values.bind(Constructor, "sc_shown", &hud_values::sc_shown);
	Values.bind(Constructor, "sc_task", &hud_values::sc_task);
	Values.bind(Constructor, "sc_assignment", &hud_values::sc_assignment);
	Values.bind(Constructor, "sc_hints", &hud_values::sc_hints);
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

void fill_aid(hud_values &Values, driving_aid::reading const &Aid)
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

Rml::String state_name(timetable_stop_state const State)
{
	switch (State)
	{
	case timetable_stop_state::exchange:
		return "exchange";
	case timetable_stop_state::can_depart:
		return "can_depart";
	case timetable_stop_state::at_stop:
		return "at_stop";
	default:
		return "normal";
	}
}

stop_row row_of(timetable_reading::stop const &Stop)
{
	stop_row row;
	row.vmax = Stop.vmax_changed ? to_string(Stop.vmax, 0) : Rml::String{};
	row.station = rmlui::game_text(Stop.station, true);
	row.km = Stop.km > 0.0 ? to_string(Stop.km, 2) : Rml::String{};
	row.ware = rmlui::game_text(Stop.ware, true);
	row.arrival = Stop.arrival;
	row.departure = Stop.departure;
	row.travel = trimmed(Stop.travel);
	row.tracks = Stop.tracks;
	row.state = state_name(Stop.state);
	return row;
}

void fill_timetable(hud_values &Values, timetable_reading const &Timetable)
{
	auto const &time{simulation::Time.data()};
	Values.tt_time = std::format("{}:{:02}:{:02}", time.wHour, time.wMinute, time.wSecond);
	Values.tt_relation = rmlui::game_text(Timetable.relation, true);
	if (false == Values.tt_relation.empty())
		Values.tt_relation += " (" + rmlui::game_text(Timetable.train, true) + ")";
	Values.tt_next = rmlui::game_text(Timetable.next_stop, false);
	Values.tt_consist = Timetable.consist;
	Values.tt_mass_specified = Timetable.mass_specified;
	Values.tt_mass_actual = Timetable.mass_actual;
	Values.tt_length = Timetable.length;
	Values.tt_has_table = Timetable.timetable;
	Values.tt_late = Timetable.delay > 0;
	Values.tt_delay = std::format("+{} min", Timetable.delay);
	Values.tt_stops.clear();
	for (auto const &stop : Timetable.stops)
		Values.tt_stops.push_back(row_of(stop));
}

void fill_scenario(hud_values &Values, scenario_reading const &Scenario)
{
	Values.sc_task = rmlui::game_text(Scenario.task, false);
	Values.sc_assignment = rmlui::game_text(Scenario.assignment, false);
	Values.sc_hints.clear();
	for (auto const &hint : Scenario.hints)
		Values.sc_hints.emplace_back(rmlui::game_text(hint.text, false), hint.done);
}

// the vehicle handed to the autopilot, or taken back from it, as the panel of imgui does it
void toggle_autopilot(bool const Autopilot)
{
	command_relay relay;
	relay.post(Autopilot ? user_command::aidriverdisable : user_command::aidriverenable, 0.0, 0.0, GLFW_PRESS, 0);
}

// the places of the parts, of the profile of the user; none in a test, which leaves the profile alone
std::filesystem::path layout_file()
{
	return Global.test.enabled ? std::filesystem::path{} : user_config_path("hud_layout.ini");
}

hud_layout::size screen_of(Rml::Element const &Element)
{
	auto const dimensions{Element.GetContext()->GetDimensions()};
	return {static_cast<float>(dimensions.x), static_cast<float>(dimensions.y)};
}

hud_layout::rect rect_of(Rml::Element &Element)
{
	auto const offset{Element.GetAbsoluteOffset(Rml::BoxArea::Border)};
	auto const size{Element.GetBox().GetSize(Rml::BoxArea::Border)};
	return {offset.x, offset.y, size.x, size.y};
}

// a part held at the edges by Anchor, in Unit (px while dragged, dp as kept)
void set_anchor(Rml::Element &Part, hud_layout::anchor const &Anchor, std::string_view const Unit)
{
	Part.SetProperty(Anchor.right ? "right" : "left", std::format("{:.1f}{}", Anchor.x, Unit));
	Part.SetProperty(Anchor.right ? "left" : "right", "auto");
	Part.SetProperty(Anchor.bottom ? "bottom" : "top", std::format("{:.1f}{}", Anchor.y, Unit));
	Part.SetProperty(Anchor.bottom ? "top" : "bottom", "auto");
}

// drags the parts of the hud while the player places them, snapping them to the edges of the screen and of the other parts
class part_mover final : public Rml::EventListener
{
  public:
	void ProcessEvent(Rml::Event &Event) override
	{
		auto *part{Event.GetCurrentElement()};
		Rml::Vector2f const mouse{Event.GetParameter<float>("mouse_x", 0.f), Event.GetParameter<float>("mouse_y", 0.f)};
		if (Event.GetId() == Rml::EventId::Dragstart)
		{
			m_grab = mouse - part->GetAbsoluteOffset(Rml::BoxArea::Border);
			return;
		}
		if (Event.GetId() != Rml::EventId::Drag)
			return;
		auto const screen{screen_of(*part)};
		auto wanted{rect_of(*part)};
		wanted.left = mouse.x - m_grab.x;
		wanted.top = mouse.y - m_grab.y;
		auto const placed{hud_layout::snap(wanted, others(*part), screen, snap_range * part->GetContext()->GetDensityIndependentPixelRatio())};
		set_anchor(*part, hud_layout::anchor_of(placed, screen), "px");
		m_moved = true;
	}

	// a part was moved since the last call
	bool take_moved()
	{
		return std::exchange(m_moved, false);
	}

  private:
	// the other parts on screen, which the part snaps to
	static std::vector<hud_layout::rect> others(Rml::Element const &Part)
	{
		std::vector<hud_layout::rect> rects;
		for (auto const *id : part_ids)
		{
			auto *other{Part.GetOwnerDocument()->GetElementById(id)};
			if (other != nullptr && other != &Part && other->IsVisible(true))
				rects.push_back(rect_of(*other));
		}
		return rects;
	}

	Rml::Vector2f m_grab{0.f, 0.f}; // where the part was taken, from its top left corner
	bool m_moved{false};
};

} // namespace

struct driver_hud::state
{
	rmlui::bound_values<hud_values> values;
	part_mover mover;
	// last, the document is closed before the mover it calls goes
	rmlui::document document;
	bool editing{false};
};

driver_hud::driver_hud() = default;

driver_hud::~driver_hud() = default;

bool driver_hud::open()
{
	auto opened{std::make_unique<state>()};
	auto const bound = [values = &opened->values](Rml::DataModelConstructor &Constructor) {
		values->bind(Constructor, "editing", &hud_values::editing);
		bind_aid(Constructor, *values);
		bind_timetable(Constructor, *values);
		bind_scenario(Constructor, *values);
		Constructor.BindEventCallback("toggle_autopilot", [values](Rml::DataModelHandle, Rml::Event &, Rml::VariantList const &) { toggle_autopilot(values->current().autopilot); });
	};
	if (false == opened->document.open("driver_hud", "ui/driverhud.rml", driverhud_rml, bound))
		return false;
	// shown without taking the focus, the keys go on to the simulation
	opened->document.show(false);
	for (auto const *id : part_ids)
	{
		if (auto *part{opened->document.element()->GetElementById(id)}; part != nullptr)
		{
			part->AddEventListener(Rml::EventId::Dragstart, &opened->mover);
			part->AddEventListener(Rml::EventId::Drag, &opened->mover);
		}
	}
	m_state = std::move(opened);
	load_layout();
	return true;
}

void driver_hud::load_layout()
{
	auto const path{layout_file()};
	if (path.empty() || false == std::filesystem::exists(path))
		return;
	std::ifstream input{path, std::ios::binary};
	std::string const text{std::istreambuf_iterator<char>{input}, std::istreambuf_iterator<char>{}};
	for (auto const &[name, anchor] : hud_layout::parse(text))
	{
		if (auto *part{m_state->document.element()->GetElementById(name)}; part != nullptr)
			set_anchor(*part, anchor, "dp");
	}
}

void driver_hud::save_layout()
{
	auto const path{layout_file()};
	if (path.empty())
		return;
	auto *document{m_state->document.element()};
	auto const ratio{document->GetContext()->GetDensityIndependentPixelRatio()};
	hud_layout::layout layout;
	for (auto const *id : part_ids)
	{
		auto *part{document->GetElementById(id)};
		if (part == nullptr || false == part->IsVisible(true))
			continue;
		auto anchor{hud_layout::anchor_of(rect_of(*part), screen_of(*part))};
		anchor.x /= ratio;
		anchor.y /= ratio;
		layout.insert_or_assign(id, anchor);
	}
	std::error_code error;
	std::filesystem::create_directories(path.parent_path(), error);
	std::ofstream output{path, std::ios::binary | std::ios::trunc};
	output << hud_layout::write(layout);
	output.close();
	if (output.fail())
		ErrorLog("rmlui: can't save the layout of the hud to " + path.string());
	else
		WriteLog("rmlui: the layout of the hud saved to " + path.string());
}

void driver_hud::reset_layout()
{
	if (m_state == nullptr)
		return;
	for (auto const *id : part_ids)
	{
		auto *part{m_state->document.element()->GetElementById(id)};
		if (part == nullptr)
			continue;
		for (auto const *side : {"left", "right", "top", "bottom"})
			part->RemoveProperty(side);
	}
	m_state->mover.take_moved();
	if (auto const path{layout_file()}; false == path.empty())
	{
		std::error_code error;
		std::filesystem::remove(path, error);
	}
}

bool driver_hud::update(panels const &Panels)
{
	if (m_failed || false == rmlui::active())
		return false;
	if (m_state == nullptr && false == open())
	{
		m_failed = true;
		return false;
	}
	// the places are kept when the player is done with them
	if (m_state->editing && false == Panels.editing && m_state->mover.take_moved())
		save_layout();
	m_state->editing = Panels.editing;

	// worked out anew from those shown, only what changed is taken into the document
	auto values{m_state->values.current()};
	values.editing = Panels.editing;
	auto const aid{Panels.aid || Panels.editing ? drivingaid_panel::read() : std::nullopt};
	values.shown = aid.has_value() && Panels.aid;
	values.expanded = Panels.aid_expanded;
	if (aid.has_value())
		fill_aid(values, *aid);
	auto const timetable{Panels.timetable || Panels.editing ? timetable_panel::read() : std::nullopt};
	values.tt_shown = timetable.has_value() && Panels.timetable;
	values.tt_expanded = Panels.timetable_expanded;
	if (timetable.has_value())
		fill_timetable(values, *timetable);
	auto const scenario{Panels.scenario || Panels.editing ? scenario_panel::read() : std::nullopt};
	values.sc_shown = scenario.has_value() && Panels.scenario;
	if (scenario.has_value())
		fill_scenario(values, *scenario);
	m_state->values.update(values, m_state->document);
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

void driver_hud::load_layout()
{
	// built without RmlUi
}

void driver_hud::save_layout()
{
	// built without RmlUi
}

void driver_hud::reset_layout()
{
	// built without RmlUi
}

bool driver_hud::update(panels const &)
{
	return false;
}

#endif
