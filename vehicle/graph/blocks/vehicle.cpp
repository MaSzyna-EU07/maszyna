/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/graph/graph_catalog.h"
#include "vehicle/graph/graph_body.h"
#include "McZapkie/MOVER.h"

#include <algorithm>
#include <cmath>

// the vehicle body: its motion, doors, lights, sander and the animated parts of its model, which stay native in the simulator
namespace vgraph
{

namespace
{

TMoverParameters &vehicle(frame const &Frame)
{
	return *Frame.body->mover;
}

block_type make_type(std::string Id, std::string Name, std::string Description)
{
	block_type type;
	type.id = std::move(Id);
	type.category = "vehicle";
	type.name = std::move(Name);
	type.description = std::move(Description);
	type.origin = block_origin::native;
	return type;
}

class veh_state final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			v,
			v_signed,
			accel,
			wheel_rpm,
			slipping,
			cab,
			time
		};
	};

	explicit veh_state(block_setup const &Setup) : block(Setup) {}

	void update(frame &Frame) override
	{
		auto const &body{*Frame.body};
		set(Frame, port::v, std::abs(body.velocity) * 3.6);
		set(Frame, port::v_signed, body.velocity);
		set(Frame, port::accel, body.acceleration);
		set(Frame, port::wheel_rpm, std::abs(body.wheel_speed) * 60.0);
		set_bool(Frame, port::slipping, body.slipping);
		set(Frame, port::cab, body.cab);
		set(Frame, port::time, Frame.time);
	}

	static block_type describe()
	{
		auto type{make_type("veh.state", "Vehicle state",
		                    "Motion of the vehicle: speed (km/h), signed speed (m/s, positive towards the front, cab 1), acceleration, wheel revolutions, the occupied "
		                    "cab (1, -1, 0) and the time of the graph.")};
		type.ports = {signal_out("v", value_type::real, "km/h"),        signal_out("v_signed", value_type::real, "m/s"), signal_out("accel", value_type::real, "m/s2"),
		              signal_out("wheel_rpm", value_type::real, "1/min"), signal_out("slipping", value_type::boolean),      signal_out("cab", value_type::integer),
		              signal_out("time", value_type::real, "s")};
		type.create = make_block<veh_state>;
		return type;
	}
};

// passenger doors, operated by the driver through the graph; the doors of the vehicle body move them and play their sounds
class veh_doors final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			open_left,
			open_right,
			lock,
			closed_left,
			closed_right,
			all_closed
		};
	};

	explicit veh_doors(block_setup const &Setup) : block(Setup), m_lock(is_connected(Setup, port::lock)) {}

	void update(frame &Frame) override
	{
		auto &doors{vehicle(Frame).Doors};
		operate(doors.instances[side::left], get_bool(Frame, port::open_left));
		operate(doors.instances[side::right], get_bool(Frame, port::open_right));
		if (m_lock)
		{
			doors.has_lock = true;
			doors.lock_enabled = get_bool(Frame, port::lock);
		}
		// the state from the last update of the doors; they move after the graph
		auto const closedleft{doors.instances[side::left].is_closed};
		auto const closedright{doors.instances[side::right].is_closed};
		set_bool(Frame, port::closed_left, closedleft);
		set_bool(Frame, port::closed_right, closedright);
		set_bool(Frame, port::all_closed, closedleft && closedright);
	}

	static block_type describe()
	{
		auto type{make_type("veh.doors", "Doors",
		                    "Passenger doors of the vehicle. They open while the open input is on and close when it goes off; lock blocks them above the lock speed.")};
		type.ports = {signal_in("open_left", value_type::boolean, 0.0, "", true),
		              signal_in("open_right", value_type::boolean, 0.0, "", true),
		              signal_in("lock", value_type::boolean, 0.0, "", true),
		              signal_out("closed_left", value_type::boolean),
		              signal_out("closed_right", value_type::boolean),
		              signal_out("all_closed", value_type::boolean)};
		// open_time and close_time are turned into the speeds of the .fiz with the range
		type.params = {
		    number_param("open_time", 2.0, "s"),
		    number_param("close_time", 3.0, "s"),
		    fiz(choice_param("type", {"shift", "rotate", "fold", "plug"}, "rotate"), "Doors:", "DoorOpenMethod"),
		    fiz(number_param("range", 1.0, "m/deg"), "Doors:", "DoorMaxShiftL"),
		    fiz(number_param("range_out", 0.0, "m"), "Doors:", "DoorMaxShiftPlug"),
		    fiz(number_param("open_delay", 0.0, "s"), "Doors:", "DoorOpenDelay"),
		    fiz(number_param("close_delay", 0.0, "s"), "Doors:", "DoorCloseDelay"),
		    fiz(number_param("lock_speed", 10.0, "km/h"), "Doors:", "DoorLockSpeed"),
		};
		type.single = true;
		type.create = make_block<veh_doors>;
		return type;
	}

private:
	template <class Door_> static void operate(Door_ &Door, bool const Open)
	{
		Door.remote_open = Open;
		Door.remote_close = false == Open;
	}

	bool const m_lock;
};

class veh_lights final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			front,
			rear,
			dim
		};
	};

	explicit veh_lights(block_setup const &Setup) : block(Setup) {}

	void update(frame &Frame) override
	{
		auto &body{vehicle(Frame)};
		body.iLights[end::front] = static_cast<int>(std::lround(get(Frame, port::front)));
		body.iLights[end::rear] = static_cast<int>(std::lround(get(Frame, port::rear)));
		Frame.body->headlights_dimmed = get_bool(Frame, port::dim);
	}

	static block_type describe()
	{
		auto type{make_type("veh.lights", "Head and tail lights",
		                    "Lights at the front and the rear end of the vehicle, as bit masks of the lights (1 left, 4 upper, 16 right white; 2 left, 32 right red...).")};
		type.ports = {signal_in("front", value_type::integer, 0.0, "", true), signal_in("rear", value_type::integer, 0.0, "", true), signal_in("dim", value_type::boolean, 0.0, "", true)};
		type.single = true;
		type.create = make_block<veh_lights>;
		return type;
	}
};

class veh_sander final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			on,
			sanding
		};
	};

	explicit veh_sander(block_setup const &Setup) : block(Setup) {}

	void update(frame &Frame) override
	{
		auto &body{vehicle(Frame)};
		auto const on{get_bool(Frame, port::on)};
		if (on != m_on)
		{
			body.Sandbox(on, range_t::local);
			m_on = on;
		}
		set_bool(Frame, port::sanding, body.SandDose);
	}

	static block_type describe()
	{
		auto type{make_type("veh.sander", "Sander", "Sands the rails while on, until the sand (body sand_capacity) runs out.")};
		type.ports = {signal_in("on", value_type::boolean), signal_out("sanding", value_type::boolean)};
		type.single = true;
		type.create = make_block<veh_sander>;
		return type;
	}

private:
	bool m_on{false};
};

// placement of the animated submodels of the vehicle's external model: the model of the vehicle body or its low poly
// interior. the vehicle (DynObj) reads these parameters by their keys and applies the block's state to the submodels
// right before the vehicle is drawn, so keep the keys and the choice ids
param_def model_param()
{
	return choice_param("model", {"vehicle", "lowpoly"}, "vehicle");
}

// submodel of the external model rotated or moved by the value, like the TGauge of a cab control: the input is clamped
// to min..max, followed with the smoothing of TGauge (its friction), and gives offset + scale * value degrees
// (rotation about the submodel's y axis) or millimetres (shift along its z axis)
class veh_anim final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			value,
			anim
		};
	};
	struct param
	{
		enum : std::size_t
		{
			model,
			submodel,
			animation,
			min,
			max,
			scale,
			offset,
			smoothing
		};
	};

	explicit veh_anim(block_setup const &Setup)
	    : block(Setup), m_min(std::min(number(Setup, param::min), number(Setup, param::max))), m_max(std::max(number(Setup, param::min), number(Setup, param::max))),
	      m_scale(number(Setup, param::scale)), m_offset(number(Setup, param::offset)), m_smoothing(std::max(0.0, number(Setup, param::smoothing)))
	{
	}

	void update(frame &Frame) override
	{
		auto const target{std::clamp(get(Frame, port::value), m_min, m_max)};
		if (m_initialized && m_smoothing > 0.0 && Frame.dt < 0.5 * m_smoothing)
		{
			// the same as TGauge::Update
			m_value += Frame.dt * (target - m_value) / m_smoothing;
			if (std::abs(target - m_value) <= 0.0001)
			{
				m_value = target;
			}
		}
		else
		{
			// the first step starts at the input, so the part doesn't move when the vehicle appears
			m_value = target;
			m_initialized = true;
		}
		set(Frame, port::anim, animation());
	}
	std::size_t state_count() const override
	{
		return 2;
	}
	char const *state_name(std::size_t Index) const override
	{
		return Index == 0 ? "value" : "anim";
	}
	double state_value(std::size_t Index) const override
	{
		return Index == 0 ? m_value : animation();
	}

	static block_type describe()
	{
		auto type{make_type("veh.anim", "Vehicle model animation",
		                    "Rotates or moves a submodel of the vehicle's external model (or its low poly interior) by the value, like a cab control: offset + scale * value, "
		                    "in degrees or millimetres, smoothed. Each vehicle shows its own state, although the model is shared.")};
		type.ports = {signal_in("value", value_type::real), signal_out("anim", value_type::real)};
		type.params = {
		    model_param(),
		    text_param("submodel"),
		    choice_param("animation", {"rotate", "move"}, "rotate"),
		    number_param("min", 0.0),
		    number_param("max", 1.0),
		    number_param("scale", 90.0, "deg/mm"),
		    number_param("offset", 0.0, "deg/mm"),
		    number_param("smoothing", 0.5, "s"),
		};
		type.create = make_block<veh_anim>;
		return type;
	}

private:
	double animation() const
	{
		return m_offset + m_scale * m_value;
	}

	double const m_min;
	double const m_max;
	double const m_scale;
	double const m_offset;
	double const m_smoothing;
	double m_value{0.0};
	bool m_initialized{false};
};

// submodel of the external model shown while the input is on, like the TButton of a lamp: submodel visible when on,
// the optional submodel_off visible when off
class veh_visible final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			on,
			shown
		};
	};
	struct param
	{
		enum : std::size_t
		{
			model,
			submodel,
			submodel_off,
			blink,
			blink_period
		};
	};

	explicit veh_visible(block_setup const &Setup) : block(Setup), m_blink(boolean(Setup, param::blink)), m_blinkperiod(std::max(0.01, number(Setup, param::blink_period))) {}

	void update(frame &Frame) override
	{
		auto on{get_bool(Frame, port::on)};
		if (on && m_blink)
		{
			// in step with the lamps of the cab, which blink with the time of the graph too
			on = std::fmod(Frame.time, m_blinkperiod) < 0.5 * m_blinkperiod;
		}
		m_shown = on;
		set_bool(Frame, port::shown, m_shown);
	}
	std::size_t state_count() const override
	{
		return 1;
	}
	char const *state_name(std::size_t) const override
	{
		return "on";
	}
	double state_value(std::size_t) const override
	{
		return m_shown ? 1.0 : 0.0;
	}

	static block_type describe()
	{
		auto type{make_type("veh.visible", "Vehicle model element visibility",
		                    "Shows a submodel of the vehicle's external model (or its low poly interior) while the input is on, and the optional off submodel while "
		                    "it's off, e.g. a lamp, a sign or a gadget. Each vehicle shows its own state, although the model is shared.")};
		type.ports = {signal_in("on", value_type::boolean), signal_out("shown", value_type::boolean)};
		type.params = {
		    model_param(), text_param("submodel"), text_param("submodel_off"), boolean_param("blink", false), number_param("blink_period", 1.0, "s"),
		};
		type.create = make_block<veh_visible>;
		return type;
	}

private:
	bool const m_blink;
	double const m_blinkperiod;
	bool m_shown{false};
};

} // namespace

void register_vehicle_blocks(catalog &Catalog)
{
	Catalog.add(veh_state::describe());
	Catalog.add(veh_doors::describe());
	Catalog.add(veh_lights::describe());
	Catalog.add(veh_sander::describe());
	Catalog.add(veh_anim::describe());
	Catalog.add(veh_visible::describe());
}

} // namespace vgraph
