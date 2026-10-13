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

#include <algorithm>
#include <cmath>

// generic logic and arithmetic blocks. boolean signals are stored as 0 and 1, and read as true above 0.5
namespace vgraph
{

namespace
{

// tolerance of the equality comparisons; switch positions and constants are compared, not results of long computations
double constexpr equality_tolerance{1e-6};

block_type make_type(std::string Id, std::string Name, std::string Description)
{
	block_type type;
	type.id = std::move(Id);
	type.category = "logic";
	type.name = std::move(Name);
	type.description = std::move(Description);
	type.origin = block_origin::primitive;
	return type;
}

class logic_compare final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			a,
			q
		};
	};
	struct param
	{
		enum : std::size_t
		{
			op,
			value
		};
	};
	enum class operation
	{
		ne,
		eq,
		lt,
		le,
		gt,
		ge
	};

	explicit logic_compare(block_setup const &Setup) : block(Setup), m_operation(static_cast<operation>(choice(Setup, param::op))), m_value(number(Setup, param::value)) {}

	void update(frame &Frame) override
	{
		auto const a{get(Frame, port::a)};
		auto result{false};
		switch (m_operation)
		{
		case operation::ne:
			result = std::abs(a - m_value) > equality_tolerance;
			break;
		case operation::eq:
			result = std::abs(a - m_value) <= equality_tolerance;
			break;
		case operation::lt:
			result = a < m_value;
			break;
		case operation::le:
			result = a <= m_value + equality_tolerance;
			break;
		case operation::gt:
			result = a > m_value;
			break;
		case operation::ge:
			result = a >= m_value - equality_tolerance;
			break;
		}
		set_bool(Frame, port::q, result);
	}

	static block_type describe()
	{
		auto type{make_type("logic.compare", "Compare", "Compares the input with a constant value.")};
		type.ports = {signal_in("a", value_type::real), signal_out("q", value_type::boolean)};
		type.params = {choice_param("op", {"ne", "eq", "lt", "le", "gt", "ge"}, "ne"), number_param("value", 0.0)};
		type.create = make_block<logic_compare>;
		return type;
	}

private:
	operation const m_operation;
	double const m_value;
};

enum class gate
{
	and_,
	or_,
	xor_,
	nand_,
	nor_
};

class logic_gate final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			a,
			b,
			q
		};
	};

	logic_gate(block_setup const &Setup, gate const Gate) : block(Setup), m_gate(Gate) {}

	void update(frame &Frame) override
	{
		auto const a{get_bool(Frame, port::a)};
		auto const b{get_bool(Frame, port::b)};
		auto result{false};
		switch (m_gate)
		{
		case gate::and_:
			result = a && b;
			break;
		case gate::or_:
			result = a || b;
			break;
		case gate::xor_:
			result = a != b;
			break;
		case gate::nand_:
			result = false == (a && b);
			break;
		case gate::nor_:
			result = false == (a || b);
			break;
		}
		set_bool(Frame, port::q, result);
	}

	template <gate Gate_> static std::unique_ptr<block> create(block_setup const &Setup)
	{
		return std::make_unique<logic_gate>(Setup, Gate_);
	}

	template <gate Gate_> static block_type describe(std::string Id, std::string Name, double const Default)
	{
		// unconnected inputs default to the neutral value of the gate, so a gate can be used with one input
		auto type{make_type(std::move(Id), std::move(Name), "Logic gate.")};
		type.ports = {signal_in("a", value_type::boolean, Default), signal_in("b", value_type::boolean, Default, "", true), signal_out("q", value_type::boolean)};
		type.create = create<Gate_>;
		return type;
	}

private:
	gate const m_gate;
};

class logic_not final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			a,
			q
		};
	};

	explicit logic_not(block_setup const &Setup) : block(Setup) {}

	void update(frame &Frame) override
	{
		set_bool(Frame, port::q, false == get_bool(Frame, port::a));
	}

	static block_type describe()
	{
		auto type{make_type("logic.not", "NOT", "Negation.")};
		type.ports = {signal_in("a", value_type::boolean), signal_out("q", value_type::boolean)};
		type.create = make_block<logic_not>;
		return type;
	}
};

// set-reset latch. it's deferred, so it can close loops, e.g. self-holding contactor circuits
class logic_sr final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			s,
			r,
			q
		};
	};
	struct param
	{
		enum : std::size_t
		{
			reset_priority,
			initial
		};
	};

	explicit logic_sr(block_setup const &Setup) : block(Setup), m_resetpriority(boolean(Setup, param::reset_priority)), m_state(boolean(Setup, param::initial)) {}

	void update(frame &Frame) override
	{
		auto const s{get_bool(Frame, port::s)};
		auto const r{get_bool(Frame, port::r)};
		if (s && r)
		{
			m_state = (false == m_resetpriority);
		}
		else if (s)
		{
			m_state = true;
		}
		else if (r)
		{
			m_state = false;
		}
	}

	void commit(frame &Frame) override
	{
		set_bool(Frame, port::q, m_state);
	}

	std::size_t state_count() const override
	{
		return 1;
	}
	char const *state_name(std::size_t) const override
	{
		return "state";
	}
	double state_value(std::size_t) const override
	{
		return m_state ? 1.0 : 0.0;
	}

	static block_type describe()
	{
		auto type{make_type("logic.sr", "SR latch", "Set sets the output, reset clears it. The output changes in the next step, so the block can close loops.")};
		type.deferred = true;
		type.ports = {signal_in("s", value_type::boolean), signal_in("r", value_type::boolean, 0.0, "", true), signal_out("q", value_type::boolean)};
		type.params = {boolean_param("reset_priority", true), boolean_param("initial", false)};
		type.create = make_block<logic_sr>;
		return type;
	}

private:
	bool const m_resetpriority;
	bool m_state;
};

// on-delay (TON) and off-delay (TOF) timers
class logic_timer final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			a,
			q
		};
	};
	struct param
	{
		enum : std::size_t
		{
			time
		};
	};

	logic_timer(block_setup const &Setup, bool const Ondelay) : block(Setup), m_ondelay(Ondelay), m_delay(std::max(0.0, number(Setup, param::time))) {}

	void update(frame &Frame) override
	{
		auto const a{get_bool(Frame, port::a)};
		if (a == m_ondelay)
		{
			m_elapsed = std::min(m_elapsed + Frame.dt, m_delay);
		}
		else
		{
			m_elapsed = 0.0;
		}
		auto const expired{m_elapsed >= m_delay};
		m_output = (m_ondelay ? (a && expired) : (a || false == expired));
	}

	void commit(frame &Frame) override
	{
		set_bool(Frame, port::q, m_output);
	}

	std::size_t state_count() const override
	{
		return 1;
	}
	char const *state_name(std::size_t) const override
	{
		return "elapsed";
	}
	double state_value(std::size_t) const override
	{
		return m_elapsed;
	}

	template <bool Ondelay_> static std::unique_ptr<block> create(block_setup const &Setup)
	{
		return std::make_unique<logic_timer>(Setup, Ondelay_);
	}

	template <bool Ondelay_> static block_type describe()
	{
		auto type{Ondelay_ ? make_type("logic.ton", "On delay", "The output follows the input switching on after the specified time, and switching off immediately.")
		                   : make_type("logic.tof", "Off delay", "The output follows the input switching on immediately, and switching off after the specified time.")};
		type.deferred = true;
		type.ports = {signal_in("a", value_type::boolean), signal_out("q", value_type::boolean)};
		type.params = {number_param("time", 1.0, "s")};
		type.create = create<Ondelay_>;
		return type;
	}

private:
	bool const m_ondelay;
	double const m_delay;
	// for the on-delay: time the input is on; for the off-delay: time the input is off
	// (both start expired for the off-delay, so the output is off until the input is switched on)
	double m_elapsed{m_ondelay ? 0.0 : m_delay};
	bool m_output{false};
};

// delays the signal by one step; the explicit way to close a loop
class logic_delay final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			a,
			q
		};
	};
	struct param
	{
		enum : std::size_t
		{
			initial
		};
	};

	explicit logic_delay(block_setup const &Setup) : block(Setup), m_value(number(Setup, param::initial)) {}

	void update(frame &Frame) override
	{
		m_value = get(Frame, port::a);
	}

	void commit(frame &Frame) override
	{
		set(Frame, port::q, m_value);
	}

	static block_type describe()
	{
		auto type{make_type("logic.delay", "Unit delay", "Passes the input to the output one step later. Use it to close loops.")};
		type.deferred = true;
		type.ports = {signal_in("a", value_type::real), signal_out("q", value_type::real)};
		type.params = {number_param("initial", 0.0)};
		type.create = make_block<logic_delay>;
		return type;
	}

private:
	double m_value;
};

class logic_hysteresis final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			a,
			q
		};
	};
	struct param
	{
		enum : std::size_t
		{
			lo,
			hi,
			initial
		};
	};

	explicit logic_hysteresis(block_setup const &Setup)
	    : block(Setup), m_low(number(Setup, param::lo)), m_high(number(Setup, param::hi)), m_state(boolean(Setup, param::initial))
	{
	}

	void update(frame &Frame) override
	{
		auto const a{get(Frame, port::a)};
		if (a >= m_high)
		{
			m_state = true;
		}
		else if (a <= m_low)
		{
			m_state = false;
		}
		set_bool(Frame, port::q, m_state);
	}

	static block_type describe()
	{
		auto type{make_type("logic.hyst", "Hysteresis", "Switches on when the input reaches the upper threshold, and off when it drops to the lower one.")};
		type.ports = {signal_in("a", value_type::real), signal_out("q", value_type::boolean)};
		type.params = {number_param("lo", 0.0), number_param("hi", 1.0), boolean_param("initial", false)};
		type.create = make_block<logic_hysteresis>;
		return type;
	}

private:
	double const m_low;
	double const m_high;
	bool m_state;
};

class logic_const final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			q
		};
	};
	struct param
	{
		enum : std::size_t
		{
			value
		};
	};

	explicit logic_const(block_setup const &Setup) : block(Setup), m_value(number(Setup, param::value)) {}

	void update(frame &Frame) override
	{
		set(Frame, port::q, m_value);
	}

	static block_type describe()
	{
		auto type{make_type("logic.const", "Constant", "Constant value.")};
		type.ports = {signal_out("q", value_type::real)};
		type.params = {number_param("value", 1.0)};
		type.create = make_block<logic_const>;
		return type;
	}

private:
	double const m_value;
};

class logic_math final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			a,
			b,
			q
		};
	};
	struct param
	{
		enum : std::size_t
		{
			op
		};
	};
	enum class operation
	{
		add,
		sub,
		mul,
		div,
		min,
		max
	};

	explicit logic_math(block_setup const &Setup) : block(Setup), m_operation(static_cast<operation>(choice(Setup, param::op))) {}

	void update(frame &Frame) override
	{
		auto const a{get(Frame, port::a)};
		auto const b{get(Frame, port::b)};
		auto result{0.0};
		switch (m_operation)
		{
		case operation::add:
			result = a + b;
			break;
		case operation::sub:
			result = a - b;
			break;
		case operation::mul:
			result = a * b;
			break;
		case operation::div:
			// division by zero gives zero rather than infinity, which would spread through the whole graph
			result = (b != 0.0 ? a / b : 0.0);
			break;
		case operation::min:
			result = std::min(a, b);
			break;
		case operation::max:
			result = std::max(a, b);
			break;
		}
		set(Frame, port::q, result);
	}

	static block_type describe()
	{
		auto type{make_type("logic.math", "Arithmetic", "Arithmetic operation on two inputs. Division by zero gives zero.")};
		type.ports = {signal_in("a", value_type::real), signal_in("b", value_type::real, 1.0), signal_out("q", value_type::real)};
		type.params = {choice_param("op", {"add", "sub", "mul", "div", "min", "max"}, "mul")};
		type.create = make_block<logic_math>;
		return type;
	}

private:
	operation const m_operation;
};

class logic_select final : public block
{
public:
	struct port
	{
		enum : std::size_t
		{
			sel,
			a,
			b,
			q
		};
	};

	explicit logic_select(block_setup const &Setup) : block(Setup) {}

	void update(frame &Frame) override
	{
		set(Frame, port::q, get_bool(Frame, port::sel) ? get(Frame, port::b) : get(Frame, port::a));
	}

	static block_type describe()
	{
		auto type{make_type("logic.select", "Select", "Passes input a when sel is off, and input b when sel is on.")};
		type.ports = {signal_in("sel", value_type::boolean), signal_in("a", value_type::real), signal_in("b", value_type::real), signal_out("q", value_type::real)};
		type.create = make_block<logic_select>;
		return type;
	}
};

} // namespace

void register_logic_blocks(catalog &Catalog)
{
	Catalog.add(logic_compare::describe());
	Catalog.add(logic_gate::describe<gate::and_>("logic.and", "AND", 1.0));
	Catalog.add(logic_gate::describe<gate::or_>("logic.or", "OR", 0.0));
	Catalog.add(logic_gate::describe<gate::xor_>("logic.xor", "XOR", 0.0));
	Catalog.add(logic_gate::describe<gate::nand_>("logic.nand", "NAND", 1.0));
	Catalog.add(logic_gate::describe<gate::nor_>("logic.nor", "NOR", 0.0));
	Catalog.add(logic_not::describe());
	Catalog.add(logic_sr::describe());
	Catalog.add(logic_timer::describe<true>());
	Catalog.add(logic_timer::describe<false>());
	Catalog.add(logic_delay::describe());
	Catalog.add(logic_hysteresis::describe());
	Catalog.add(logic_const::describe());
	Catalog.add(logic_math::describe());
	Catalog.add(logic_select::describe());
}

} // namespace vgraph
