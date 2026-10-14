/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "vehicle/graph/graph_types.h"

#include <cstdint>
#include <memory>
#include <vector>

namespace vgraph
{

struct block_type;
struct body_state;
class lua_host;

// state of a physical connection. the output writes the forward value; every connected input adds its share
// of the back value to back_next, which becomes visible to the output in the next step. thanks to this one step lag
// the back channel never constrains the order of evaluation, and the result doesn't depend on the order of the inputs
struct link_state
{
	double forward{0.0};
	double back{0.0};
	double back_next{0.0};
};

// data shared by all blocks of one graph instance during a single step
struct frame
{
	double dt{0.0};
	double time{0.0}; // time elapsed since the start of the instance, after this step
	std::uint64_t step{0}; // number of the step, counted from 1; for the blocks combining their effects on the body (shared_value)
	bool low_voltage{false}; // graph wide switch: simulate the low voltage circuits, or treat them as always powered
	double *signals{nullptr};
	std::uint8_t const *forced{nullptr}; // signal slots with value forced from outside, writes from blocks are ignored
	link_state *links{nullptr};
	body_state *body{nullptr}; // the vehicle body; never null during update
};

// everything a block factory needs to create a block of the given node
struct block_setup
{
	block_type const &type;
	std::vector<port_def> const &ports; // ports of the node: the type's, or resolved for the node (lua blocks)
	std::vector<param_def> const &param_defs; // parameter definitions of the node, as above
	std::vector<param_value> const &params; // in the order of param_defs
	std::vector<int> const &slots; // in the order of ports; signal slot or link index
	std::vector<std::uint8_t> const &connected; // in the order of ports
	std::string const &node_id;
	std::string const &script; // source of a lua block
	lua_host *lua{nullptr}; // lua state of the graph instance; null when the graph has no lua blocks
};

// base of all blocks. a block reads and writes its ports through slot indices resolved when the graph was compiled
class block
{
public:
	explicit block(block_setup const &Setup);
	virtual ~block() = default;

	// called once per step in the order resolved from the connections
	virtual void update(frame &Frame) = 0;
	// called after all blocks were updated, by the blocks of types marked as deferred. a deferred block publishes its
	// outputs here, so they become visible to other blocks in the next step; this is what allows feedback loops
	virtual void commit(frame &) {}
	// called after all blocks were updated and before the deferred ones commit, by the blocks of types marked as late.
	// used by the blocks which have to see the effects of the whole step, e.g. the brake pipe applying all the flows
	virtual void late_update(frame &) {}

	// internal state shown by the inspector and written by the standalone runner, e.g. whether a lamp is lit
	virtual std::size_t state_count() const
	{
		return 0;
	}
	virtual char const *state_name(std::size_t) const
	{
		return "";
	}
	virtual double state_value(std::size_t) const
	{
		return 0.0;
	}

	block_type const &type() const
	{
		return m_type;
	}

protected:
	// signal ports
	double get(frame const &Frame, std::size_t Port) const
	{
		return Frame.signals[m_slots[Port]];
	}
	bool get_bool(frame const &Frame, std::size_t Port) const
	{
		return Frame.signals[m_slots[Port]] > 0.5;
	}
	void set(frame &Frame, std::size_t Port, double const Value) const
	{
		auto const slot{m_slots[Port]};
		if (Frame.forced[slot] == 0)
		{
			Frame.signals[slot] = Value;
		}
	}
	void set_bool(frame &Frame, std::size_t Port, bool const Value) const
	{
		set(Frame, Port, Value ? 1.0 : 0.0);
	}
	// physical inputs
	double forward_in(frame const &Frame, std::size_t Port) const
	{
		return Frame.links[m_slots[Port]].forward;
	}
	void add_back(frame &Frame, std::size_t Port, double const Value) const
	{
		Frame.links[m_slots[Port]].back_next += Value;
	}
	// physical outputs
	void set_forward(frame &Frame, std::size_t Port, double const Value) const
	{
		Frame.links[m_slots[Port]].forward = Value;
	}
	double back_out(frame const &Frame, std::size_t Port) const
	{
		return Frame.links[m_slots[Port]].back;
	}

	// whether the port has a connection; known when the block is created
	static bool is_connected(block_setup const &Setup, std::size_t Port)
	{
		return Setup.connected[Port] != 0;
	}
	// parameters, read when the block is created
	static double number(block_setup const &Setup, std::size_t Param);
	static bool boolean(block_setup const &Setup, std::size_t Param);
	static std::string const &text(block_setup const &Setup, std::size_t Param);
	static std::vector<std::vector<double>> const &table(block_setup const &Setup, std::size_t Param);
	// index of the parameter's value in the list of choices of its definition
	static int choice(block_setup const &Setup, std::size_t Param);

private:
	block_type const &m_type;
	std::vector<int> m_slots;
};

using block_factory = std::unique_ptr<block> (*)(block_setup const &);

template <class Block_> std::unique_ptr<block> make_block(block_setup const &Setup)
{
	return std::make_unique<Block_>(Setup);
}

} // namespace vgraph
