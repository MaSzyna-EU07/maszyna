/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/graph/blocks/cab.h"
#include "vehicle/graph/graph_catalog.h"

#include <algorithm>
#include <cmath>

namespace vgraph
{

namespace
{

std::vector<std::string> split_labels(std::string const &Text)
{
	std::vector<std::string> result;
	if (Text.empty())
	{
		return result;
	}
	std::string::size_type start{0};
	while (true)
	{
		auto const separator{Text.find(';', start)};
		result.emplace_back(Text.substr(start, separator == std::string::npos ? std::string::npos : separator - start));
		if (separator == std::string::npos)
		{
			break;
		}
		start = separator + 1;
	}
	return result;
}

} // namespace

int cab_of(block_setup const &Setup)
{
	auto const index{find_param(Setup.param_defs, "cab")};
	if (index < 0)
	{
		return 0;
	}
	// "any", "1", "2", "0" (the engine room)
	auto const &value{Setup.params[index].text};
	return value == "1" ? 1 : value == "2" ? 2 : value == "0" ? -1 : 0;
}

// cab_switch

cab_switch::cab_switch(block_setup const &Setup)
    : block(Setup), m_behaviour(static_cast<behaviour_type>(choice(Setup, param::behaviour))),
      m_positioncount(std::max(1, static_cast<int>(std::lround(number(Setup, param::positions))))), m_min(number(Setup, param::min)), m_max(number(Setup, param::max)),
      m_spring(boolean(Setup, param::spring)), m_labels(split_labels(text(Setup, param::labels))), m_animation(static_cast<animation_type>(choice(Setup, param::animation))),
      m_step(number(Setup, param::step)), m_offset(number(Setup, param::offset)), m_smoothing(number(Setup, param::smoothing)), m_submodel(text(Setup, param::submodel)),
      m_soundincrease(text(Setup, param::sound_inc)), m_sounddecrease(text(Setup, param::sound_dec)),
      m_soundpositions(text(Setup, param::sound_positions)),
      m_keyincrease(text(Setup, param::key_inc)), m_keydecrease(text(Setup, param::key_dec)), m_command(text(Setup, param::command)), m_hardwarename(text(Setup, param::hw_name)),
      m_cab(cab_of(Setup))
{
	// the default is given as a value, as the author sees it; the switch starts at the position closest to it
	auto const defaultvalue{number(Setup, param::default_value)};
	if (m_positioncount > 1 && m_max != m_min)
	{
		m_defaultposition = clamp_position(static_cast<int>(std::lround((defaultvalue - m_min) / (m_max - m_min) * (m_positioncount - 1))));
	}
	m_position = m_defaultposition;
	m_display = static_cast<float>(m_position);
}

int cab_switch::clamp_position(int const Position) const
{
	return std::clamp(Position, 0, m_positioncount - 1);
}

double cab_switch::value_at(int const Position) const
{
	if (m_positioncount < 2)
	{
		return m_min;
	}
	return m_min + (m_max - m_min) * clamp_position(Position) / (m_positioncount - 1);
}

std::string cab_switch::label(int const Position) const
{
	return (Position >= 0 && Position < static_cast<int>(m_labels.size()) ? m_labels[Position] : std::string{});
}

void cab_switch::update(frame &Frame)
{
	set(Frame, port::value, value_at(m_position));
	set(Frame, port::index, m_position);
	m_display = static_cast<float>(m_position);
}

void cab_switch::increase()
{
	m_position = clamp_position(m_position + 1);
}

void cab_switch::decrease()
{
	m_position = clamp_position(m_position - 1);
}

void cab_switch::toggle()
{
	switch (m_behaviour)
	{
	case behaviour_type::monostable:
	{
		// a momentary switch can't stay toggled; a toggle is treated as pressing it, the release brings it back
		press();
		break;
	}
	default:
	{
		auto const other{m_defaultposition == m_positioncount - 1 ? 0 : m_positioncount - 1};
		m_position = (m_position == m_defaultposition ? other : m_defaultposition);
		break;
	}
	}
}

void cab_switch::press()
{
	switch (m_behaviour)
	{
	case behaviour_type::multi:
	{
		// levers are moved with increase and decrease; pressing them has no meaning
		break;
	}
	case behaviour_type::monostable:
	{
		m_position = (m_defaultposition == m_positioncount - 1 ? 0 : m_positioncount - 1);
		break;
	}
	default:
	{
		toggle();
		break;
	}
	}
}

void cab_switch::release()
{
	if (m_behaviour == behaviour_type::monostable || (m_behaviour == behaviour_type::multi && m_spring))
	{
		m_position = m_defaultposition;
	}
}

void cab_switch::set_position(int const Position)
{
	m_position = clamp_position(Position);
}

void cab_switch::set_value(double const Value)
{
	if (m_positioncount < 2 || m_max == m_min)
	{
		return;
	}
	set_position(static_cast<int>(std::lround((Value - m_min) / (m_max - m_min) * (m_positioncount - 1))));
}

block_type cab_switch::describe()
{
	block_type type;
	type.id = "cab.switch";
	type.category = "cab";
	type.name = "Switch";
	type.description = "Switch, button or lever of the cab. Its behaviour and animation are defined here, and its value can be connected to any block.";
	type.origin = block_origin::cab;
	type.ports = {signal_out("pos", value_type::real), signal_out("index", value_type::integer)};
	type.params = {
	    choice_param("behaviour", {"bistable", "monostable", "multi"}, "bistable"),
	    number_param("positions", 2.0),
	    number_param("min", 0.0),
	    number_param("max", 1.0),
	    number_param("default", 0.0),
	    boolean_param("spring", false),
	    text_param("labels"),
	    choice_param("animation", {"rotate", "move"}, "rotate"),
	    number_param("step", 45.0, "deg/mm"),
	    number_param("offset", 0.0, "deg/mm"),
	    number_param("smoothing", 0.2, "s"),
	    text_param("submodel"),
	    text_param("sound_inc"),
	    text_param("sound_dec"),
	    text_param("sound_positions"),
	    text_param("key_inc"),
	    text_param("key_dec"),
	    text_param("command"),
	    text_param("hw_name"),
	    cab_param(),
	};
	type.create = make_block<cab_switch>;
	return type;
}

// cab_lamp

cab_lamp::cab_lamp(block_setup const &Setup)
    : block(Setup), m_supplied(is_connected(Setup, port::supply)), m_blink(boolean(Setup, param::blink)), m_blinkperiod(std::max(0.01, number(Setup, param::blink_period))),
      m_minvoltage(number(Setup, param::min_voltage)), m_power(std::max(0.0, number(Setup, param::power))), m_submodel(text(Setup, param::submodel)),
      m_soundon(text(Setup, param::sound_on)), m_soundoff(text(Setup, param::sound_off)), m_cab(cab_of(Setup))
{
}

void cab_lamp::update(frame &Frame)
{
	// without low voltage simulation, or without the supply connected, the lamp is assumed to be powered
	auto const lowvoltage{Frame.low_voltage && m_supplied};
	auto const voltage{lowvoltage ? forward_in(Frame, port::supply) : 0.0};
	auto const powered{false == lowvoltage || voltage >= m_minvoltage};
	auto lit{powered && get_bool(Frame, port::on)};
	if (lit && m_blink)
	{
		// blinking follows the time of the graph, so all lamps of the vehicle blink in step
		lit = std::fmod(Frame.time, m_blinkperiod) < 0.5 * m_blinkperiod;
	}
	m_lit = lit;
	if (lowvoltage && m_lit && voltage > 1.0)
	{
		add_back(Frame, port::supply, m_power / voltage);
	}
}

block_type cab_lamp::describe()
{
	block_type type;
	type.id = "fb.lamp";
	type.category = "indicators";
	type.name = "Lamp";
	type.description = "Indicator lamp. With low voltage simulation enabled and the supply connected, it needs enough voltage to light, and draws current.";
	type.origin = block_origin::cab;
	type.ports = {signal_in("on", value_type::boolean), physical_in("supply", domain::low_voltage, true)};
	type.params = {
	    choice_param("color", {"green", "yellow", "red", "white", "blue"}, "green"),
	    boolean_param("blink", false),
	    number_param("blink_period", 1.0, "s"),
	    number_param("min_voltage", 70.0, "V"),
	    number_param("power", 2.0, "W"),
	    text_param("submodel"),
	    text_param("sound_on"),
	    text_param("sound_off"),
	    cab_param(),
	};
	type.create = make_block<cab_lamp>;
	return type;
}

// cab_gauge

cab_gauge::cab_gauge(block_setup const &Setup, bool const Twoneedles)
    : block(Setup), m_twoneedles(Twoneedles), m_min(number(Setup, param::min)), m_max(number(Setup, param::max)), m_anglemin(number(Setup, param::angle_min)),
      m_anglemax(number(Setup, param::angle_max)), m_smoothing(number(Setup, param::smoothing)), m_submodel(text(Setup, param::submodel)),
      m_submodel2(Twoneedles ? text(Setup, param::submodel2) : std::string{}), m_cab(cab_of(Setup))
{
	m_values[0] = m_values[1] = static_cast<float>(std::min(m_min, m_max));
}

void cab_gauge::update(frame &Frame)
{
	// the needle stops at the ends of the scale
	auto const low{std::min(m_min, m_max)};
	auto const high{std::max(m_min, m_max)};
	m_values[0] = static_cast<float>(std::clamp(get(Frame, port::value), low, high));
	if (m_twoneedles)
	{
		m_values[1] = static_cast<float>(std::clamp(get(Frame, port::value2), low, high));
	}
}

block_type cab_gauge::describe(bool const Twoneedles)
{
	block_type type;
	type.category = "indicators";
	type.origin = block_origin::cab;
	if (Twoneedles)
	{
		type.id = "fb.manometer";
		type.name = "Two needle gauge";
		type.description = "Gauge with two needles, e.g. pressure gauge of the brake pipe (black) and the main reservoir (red).";
		type.ports = {signal_in("black", value_type::real, 0.0, "bar"), signal_in("red", value_type::real, 0.0, "bar", true)};
	}
	else
	{
		type.id = "fb.gauge";
		type.name = "Gauge";
		type.description = "Needle gauge: voltmeter, ammeter, speedometer.";
		type.ports = {signal_in("value", value_type::real)};
	}
	type.params = {
	    number_param("min", 0.0),
	    number_param("max", Twoneedles ? 10.0 : 100.0),
	    text_param("unit", Twoneedles ? "bar" : ""),
	    number_param("angle_min", -135.0, "deg"),
	    number_param("angle_max", 135.0, "deg"),
	    number_param("smoothing", 0.3, "s"),
	    text_param("submodel"),
	};
	if (Twoneedles)
	{
		type.params.push_back(text_param("submodel2"));
	}
	type.params.push_back(cab_param());
	if (Twoneedles)
	{
		type.create = [](block_setup const &Setup) -> std::unique_ptr<block> { return std::make_unique<cab_gauge>(Setup, true); };
	}
	else
	{
		type.create = [](block_setup const &Setup) -> std::unique_ptr<block> { return std::make_unique<cab_gauge>(Setup, false); };
	}
	return type;
}

// vehicle_sound

vehicle_sound::vehicle_sound(block_setup const &Setup)
    : block(Setup), m_file(text(Setup, param::file)), m_basevolume(number(Setup, param::volume)), m_basepitch(number(Setup, param::pitch)), m_loop(boolean(Setup, param::loop)),
      m_placement(static_cast<placement_type>(choice(Setup, param::placement))), m_range(number(Setup, param::range)),
      m_offset{number(Setup, param::offset_x), number(Setup, param::offset_y), number(Setup, param::offset_z)}, m_cab(cab_of(Setup))
{
}

void vehicle_sound::update(frame &Frame)
{
	m_playing = get_bool(Frame, port::play);
	m_volume = static_cast<float>(std::max(0.0, m_basevolume * get(Frame, port::volume)));
	m_pitch = static_cast<float>(std::max(0.0, m_basepitch * get(Frame, port::pitch)));
}

char const *vehicle_sound::state_name(std::size_t const Index) const
{
	switch (Index)
	{
	case 0:
		return "playing";
	case 1:
		return "volume";
	default:
		return "pitch";
	}
}

double vehicle_sound::state_value(std::size_t const Index) const
{
	switch (Index)
	{
	case 0:
		return m_playing ? 1.0 : 0.0;
	case 1:
		return m_volume;
	default:
		return m_pitch;
	}
}

block_type vehicle_sound::describe()
{
	block_type type;
	type.id = "fb.sound";
	type.category = "indicators";
	type.name = "Sound";
	type.description = "Sound of the vehicle or of its cab. Volume and pitch are the parameters multiplied by the inputs; looped sounds play while the play input is on.";
	type.origin = block_origin::cab;
	type.ports = {signal_in("play", value_type::boolean), signal_in("volume", value_type::real, 1.0, "", true), signal_in("pitch", value_type::real, 1.0, "", true)};
	type.params = {
	    text_param("file"),
	    number_param("volume", 1.0),
	    number_param("pitch", 1.0),
	    boolean_param("loop", true),
	    choice_param("placement", {"general", "internal", "engine", "external", "external_ambient"}, "internal"),
	    number_param("range", -1.0, "m"),
	    number_param("offset_x", 0.0, "m"),
	    number_param("offset_y", 0.0, "m"),
	    number_param("offset_z", 0.0, "m"),
	    cab_param(),
	};
	type.create = make_block<vehicle_sound>;
	return type;
}

// cab_export

cab_export::cab_export(block_setup const &Setup) : block(Setup), m_name(text(Setup, 0)), m_kind(static_cast<kind_type>(choice(Setup, 1))) {}

void cab_export::update(frame &Frame)
{
	m_value = get(Frame, port::value);
}

block_type cab_export::describe()
{
	block_type type;
	type.id = "fb.export";
	type.category = "indicators";
	type.name = "Screen value";
	type.description = "Publishes the value for the python screens of the cab, as an entry of the train state with the given name.";
	type.origin = block_origin::cab;
	type.ports = {signal_in("value", value_type::real)};
	type.params = {text_param("name"), choice_param("kind", {"float", "int", "bool"}, "float")};
	type.create = make_block<cab_export>;
	return type;
}

void register_cab_blocks(catalog &Catalog)
{
	Catalog.add(cab_switch::describe());
	Catalog.add(cab_lamp::describe());
	Catalog.add(cab_gauge::describe(false));
	Catalog.add(cab_gauge::describe(true));
	Catalog.add(vehicle_sound::describe());
	Catalog.add(cab_export::describe());
}

} // namespace vgraph
