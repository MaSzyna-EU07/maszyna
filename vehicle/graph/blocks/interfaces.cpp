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
#include "vehicle/graph/graph_instance.h"
#include "McZapkie/MOVER.h"

#include <array>
#include <cmath>
#include <limits>

// standard interfaces: between the vehicle and its neighbours (brake pipe, feed pipe, train heating), and between the
// vehicle and the AI. the neighbours may be .fiz vehicles, so these blocks work through the couplers of the vehicle body
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
	type.category = "interfaces";
	type.name = std::move(Name);
	type.description = std::move(Description);
	type.origin = block_origin::interface_profile;
	return type;
}

param_def side_param()
{
	return choice_param("side", {"front", "rear"}, "front");
}

// end cock of the brake pipe, or of the feed pipe, at one end of the vehicle
class if_cock final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			pipe,
			cock
		};
	};
	struct param
	{
		enum : std::size_t
		{
			side,
			cock
		};
	};

	if_cock(block_setup const &Setup, bool const Brakepipe)
	    : block(Setup), m_brakepipe(Brakepipe), m_side(choice(Setup, param::side) == 0 ? end::front : end::rear), m_switched(is_connected(Setup, port::cock)),
	      m_open(boolean(Setup, param::cock))
	{
	}

	void update(frame &Frame) override
	{
		auto const open{m_switched ? get_bool(Frame, port::cock) : m_open};
		auto &coupler{vehicle(Frame).Couplers[m_side]};
		if (m_brakepipe)
		{
			coupler.BrakePipeCock = open;
		}
		else
		{
			coupler.FeedPipeCock = open;
		}
	}

	static block_type describe(bool const Brakepipe)
	{
		auto type{Brakepipe ? make_type("if.pg", "Brake pipe coupling",
		                                "End cock of the brake pipe at one end of the vehicle. Open (the default), the brake pipe exchanges air with the neighbouring vehicle.")
		                    : make_type("if.pz", "Feed pipe coupling",
		                                "End cock (yellow) of the feed pipe at one end of the vehicle. Closed by default; open, the feed pipe exchanges air with the neighbour.")};
		type.ports = {Brakepipe ? physical_in("pg", domain::brake_pipe) : physical_in("pz", domain::feed_pipe), signal_in("cock", value_type::boolean, 0.0, "", true)};
		type.params = {side_param(), boolean_param("cock", Brakepipe)};
		if (Brakepipe)
		{
			type.create = [](block_setup const &Setup) -> std::unique_ptr<block> { return std::make_unique<if_cock>(Setup, true); };
		}
		else
		{
			type.create = [](block_setup const &Setup) -> std::unique_ptr<block> { return std::make_unique<if_cock>(Setup, false); };
		}
		return type;
	}

private:
	bool const m_brakepipe;
	int const m_side;
	bool const m_switched;
	bool const m_open;
};

// train heating supply through the couplers. the voltage comes from the main circuit of the vehicle (Mains)
class if_heat final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			wn,
			on,
			i
		};
	};

	explicit if_heat(block_setup const &Setup) : block(Setup), m_side(choice(Setup, 0) == 0 ? end::front : end::rear) {}

	void update(frame &Frame) override
	{
		auto &body{vehicle(Frame)};
		auto const voltage{forward_in(Frame, port::wn)};
		auto const on{get_bool(Frame, port::on)};
		body.HeatingAllow = on;
		body.Heating = on && voltage > 0.0;
		auto const current{std::abs(body.Couplers[m_side].power_high.current)};
		add_back(Frame, port::wn, current);
		set(Frame, port::i, current);
	}

	static block_type describe()
	{
		auto type{make_type("if.heat", "Train heating",
		                    "Train heating supply through the coupler at one end: on connects the main circuit (behind the line breaker) to the heating line. i is the "
		                    "current the train draws.")};
		type.ports = {physical_in("wn", domain::high_voltage), signal_in("on", value_type::boolean, 1.0, "", true), signal_out("i", value_type::real, "A")};
		type.params = {side_param()};
		type.create = make_block<if_heat>;
		return type;
	}

private:
	int const m_side;
};

// intents of the AI driver, and the feedback it reads. graph vehicles aren't driven by the AI yet: the outputs stay
// at rest until the AI adapter of the graph vehicles is in place
class if_ai final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			ready,
			i_ratio,
			slip,
			pg,
			zg,
			bc,
			vmax,
			dir,
			traction,
			ed_brake,
			brake,
			emergency,
			local_brake,
			prepare,
			shutdown,
			doors_left,
			doors_right,
			lights,
			horn,
			pantographs,
			sand
		};
	};

	explicit if_ai(block_setup const &Setup) : block(Setup) {}

	void update(frame &Frame) override
	{
		auto const &intents{Frame.body->ai};
		set(Frame, port::dir, intents.dir);
		set(Frame, port::traction, intents.traction);
		set(Frame, port::ed_brake, intents.ed_brake);
		set(Frame, port::brake, intents.brake);
		set_bool(Frame, port::emergency, intents.emergency);
		set(Frame, port::local_brake, intents.local_brake);
		set_bool(Frame, port::prepare, intents.prepare);
		set_bool(Frame, port::shutdown, intents.shutdown);
		set_bool(Frame, port::doors_left, intents.doors_left);
		set_bool(Frame, port::doors_right, intents.doors_right);
		set(Frame, port::lights, intents.lights);
		set_bool(Frame, port::horn, intents.horn);
		set_bool(Frame, port::pantographs, intents.pantographs);
		set_bool(Frame, port::sand, intents.sand);
		auto &feedback{Frame.body->ai_reply};
		feedback.ready = get_bool(Frame, port::ready);
		feedback.i_ratio = get(Frame, port::i_ratio);
		feedback.slip = get_bool(Frame, port::slip);
		feedback.pg = get(Frame, port::pg);
		feedback.zg = get(Frame, port::zg);
		feedback.bc = get(Frame, port::bc);
		feedback.vmax = get(Frame, port::vmax);
	}

	static block_type describe()
	{
		auto type{make_type("if.ai", "AI interface",
		                    "What the AI driver wants (outputs) and what it needs to know (inputs). The vehicle's graph translates the intents to its controls. Not "
		                    "used by the simulator yet: the AI doesn't drive graph vehicles.")};
		type.ports = {signal_in("ready", value_type::boolean, 0.0, "", true),
		              signal_in("i_ratio", value_type::real, 0.0, "", true),
		              signal_in("slip", value_type::boolean, 0.0, "", true),
		              signal_in("pg", value_type::real, 0.0, "bar", true),
		              signal_in("zg", value_type::real, 0.0, "bar", true),
		              signal_in("bc", value_type::real, 0.0, "bar", true),
		              signal_in("vmax", value_type::real, 0.0, "km/h", true),
		              signal_out("dir", value_type::real),
		              signal_out("traction", value_type::real),
		              signal_out("ed_brake", value_type::real),
		              signal_out("brake", value_type::real),
		              signal_out("emergency", value_type::boolean),
		              signal_out("local_brake", value_type::real),
		              signal_out("prepare", value_type::boolean),
		              signal_out("shutdown", value_type::boolean),
		              signal_out("doors_left", value_type::boolean),
		              signal_out("doors_right", value_type::boolean),
		              signal_out("lights", value_type::integer),
		              signal_out("horn", value_type::boolean),
		              signal_out("pantographs", value_type::boolean),
		              signal_out("sand", value_type::boolean)};
		type.single = true;
		type.create = make_block<if_ai>;
		return type;
	}
};

// control line of the multiple unit control: a named train wire shared by the coupled graph vehicles. every vehicle
// publishes its contribution in its body state (body_state::lines); the others read what it published last, so the
// values pass between the vehicles with up to one step of delay. the line ends at a vehicle without a graph, at
// a vehicle without the line, or at a coupling without the required flag
class if_line final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			in,
			active,
			out,
			vehicles,
			drivers
		};
	};
	struct param
	{
		enum : std::size_t
		{
			name,
			combine,
			reverse,
			pair,
			coupling
		};
	};
	enum class combination
	{
		max,
		min,
		sum,
		maxabs,
		any
	};

	explicit if_line(block_setup const &Setup)
	    : block(Setup), m_name(text(Setup, param::name)), m_pair(text(Setup, param::pair).empty() ? m_name : text(Setup, param::pair)),
	      m_combine(static_cast<combination>(std::max(0, choice(Setup, param::combine)))), m_reverse(boolean(Setup, param::reverse)),
	      m_flag(coupling_flag(choice(Setup, param::coupling))), m_drives(is_connected(Setup, port::in))
	{
		m_hints.fill(0);
	}

	void update(frame &Frame) override
	{
		auto &lines{Frame.body->lines};
		if (m_entry < 0)
		{
			// the entry of the line in this vehicle, shared with the other blocks of the same line
			m_entry = find(lines, m_name, 0);
			if (m_entry < 0)
			{
				lines.push_back({m_name, 0.0, false});
				m_entry = static_cast<int>(lines.size()) - 1;
			}
		}
		auto &own{lines[m_entry]};
		if (m_drives)
		{
			own.value = get(Frame, port::in);
			own.active = get_bool(Frame, port::active);
		}
		m_drivers = 0;
		m_vehicles = 1;
		if (own.active)
		{
			add(own.value);
		}
		auto const *body{Frame.body->mover};
		for (int side = 0; side < 2; ++side)
		{
			walk(*body, side, m_hints.data() + side * max_vehicles);
		}
		set(Frame, port::out, m_drivers > 0 ? m_result : 0.0);
		set(Frame, port::vehicles, m_vehicles);
		set(Frame, port::drivers, m_drivers);
	}

	static block_type describe()
	{
		auto type{make_type("if.line", "Train line",
		                    "Named control line of the multiple unit control, shared by the coupled vehicles with the line (the coupling needs the selected flag). out "
		                    "combines the values of the vehicles driving the line (in connected and active set), in the orientation of this vehicle: with reverse the "
		                    "value of a vehicle facing the other way changes sign, and such a vehicle gives its pair line (if set) in place of this one.")};
		type.ports = {signal_in("in", value_type::real, 0.0, "", true), signal_in("active", value_type::boolean, 1.0, "", true), signal_out("out", value_type::real),
		              signal_out("vehicles", value_type::integer), signal_out("drivers", value_type::integer)};
		type.params = {text_param("name", "linia"), choice_param("combine", {"max", "min", "sum", "maxabs", "or"}, "max"), boolean_param("reverse", false), text_param("pair"),
		               choice_param("coupling", {"control", "permanent", "coupler", "highvoltage", "heating", "power110v", "power24v", "power3x400v"}, "control")};
		type.create = make_block<if_line>;
		return type;
	}

private:
	// longest chain of vehicles followed in each direction, also the guard against coupling loops
	static constexpr int max_vehicles{64};

	static int coupling_flag(int const Choice)
	{
		static int const flags[]{coupling::control, coupling::permanent, coupling::coupler, coupling::highvoltage, coupling::heating, coupling::power110v, coupling::power24v,
		                         coupling::power3x400v};
		return flags[std::clamp(Choice, 0, static_cast<int>(std::size(flags)) - 1)];
	}

	// index of the line with the specified name, checking the hint first, or -1
	static int find(std::vector<train_line_value> const &Lines, std::string const &Name, int const Hint)
	{
		if (Hint >= 0 && Hint < static_cast<int>(Lines.size()) && Lines[Hint].name == Name)
		{
			return Hint;
		}
		for (std::size_t idx = 0; idx < Lines.size(); ++idx)
		{
			if (Lines[idx].name == Name)
			{
				return static_cast<int>(idx);
			}
		}
		return -1;
	}

	void add(double const Value)
	{
		if (m_drivers == 0)
		{
			m_result = (m_combine == combination::any ? (Value != 0.0 ? 1.0 : 0.0) : Value);
		}
		else
		{
			switch (m_combine)
			{
			case combination::max:
				m_result = std::max(m_result, Value);
				break;
			case combination::min:
				m_result = std::min(m_result, Value);
				break;
			case combination::sum:
				m_result += Value;
				break;
			case combination::maxabs:
				m_result = (std::abs(Value) > std::abs(m_result) ? Value : m_result);
				break;
			case combination::any:
				m_result = (m_result != 0.0 || Value != 0.0 ? 1.0 : 0.0);
				break;
			}
		}
		++m_drivers;
	}

	// follows the coupled vehicles from the specified side of the vehicle. a neighbour coupled by the same side as the one
	// leading to it (front to front, rear to rear) faces the other way
	void walk(TMoverParameters const &Body, int Side, int *Hints)
	{
		auto const *current{&Body};
		auto facing{1}; // orientation of the current vehicle relative to this one
		for (int count = 0; count < max_vehicles; ++count)
		{
			auto const &coupler{current->Couplers[Side]};
			if (coupler.Connected == nullptr || (coupler.CouplingFlag & m_flag) == 0 || coupler.Connected == &Body)
			{
				return;
			}
			auto const *neighbour{coupler.Connected};
			auto const connectedside{coupler.ConnectedNr};
			facing = (Side != connectedside ? facing : -facing);
			if (neighbour->Graph == nullptr)
			{
				return;
			}
			auto const &lines{neighbour->Graph->body().lines};
			auto const index{find(lines, facing > 0 ? m_name : m_pair, Hints[count])};
			if (index < 0)
			{
				return;
			}
			Hints[count] = index;
			++m_vehicles;
			auto const &line{lines[index]};
			if (line.active)
			{
				add(m_reverse && facing < 0 ? -line.value : line.value);
			}
			current = neighbour;
			Side = 1 - connectedside;
		}
	}

	std::string const m_name;
	std::string const m_pair;
	combination const m_combine;
	bool const m_reverse;
	int const m_flag;
	bool const m_drives;
	int m_entry{-1};
	std::array<int, 2 * max_vehicles> m_hints;
	double m_result{0.0};
	int m_drivers{0};
	int m_vehicles{1};
};

} // namespace

void register_interface_blocks(catalog &Catalog)
{
	Catalog.add(if_cock::describe(true));
	Catalog.add(if_cock::describe(false));
	Catalog.add(if_heat::describe());
	Catalog.add(if_ai::describe());
	Catalog.add(if_line::describe());
}

} // namespace vgraph
