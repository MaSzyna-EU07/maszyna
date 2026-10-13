/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "vehicle/graph/graph_block.h"
#include "vehicle/graph/graph_template.h"

#include <memory>
#include <string_view>
#include <vector>

namespace vgraph
{

// simulation state of the graph of a single vehicle
class graph_instance
{
public:
	explicit graph_instance(std::shared_ptr<graph_template const> Template);

	// advances the graph by the specified time. the order of evaluation is fixed, and there's no other source
	// of time or randomness, so the same inputs always produce the same results
	void update(double Deltatime);

	graph_template const &definition() const
	{
		return *m_template;
	}
	double time() const
	{
		return m_time;
	}

	// block of the node with the specified index (see graph_template::nodes())
	block &node_block(std::size_t Index)
	{
		return *m_blocks[Index];
	}
	block const &node_block(std::size_t Index) const
	{
		return *m_blocks[Index];
	}
	// block of the node with the specified id, or null
	block *find_block(std::string_view Id);

	double signal(int Slot) const
	{
		return m_signals[Slot];
	}
	// keeps the slot at the specified value regardless of what its block writes, until released
	void force(int Slot, double Value);
	void release(int Slot);
	bool is_forced(int Slot) const
	{
		return m_forced[Slot] != 0;
	}

private:
	frame make_frame(double Deltatime);

	std::shared_ptr<graph_template const> m_template;
	std::vector<std::unique_ptr<block>> m_blocks; // in the order of the template's nodes
	std::vector<block *> m_order; // blocks in the order of evaluation
	std::vector<block *> m_deferred; // blocks publishing their outputs after the step
	std::vector<double> m_signals;
	std::vector<std::uint8_t> m_forced;
	std::vector<link_state> m_links;
	double m_time{0.0};
};

} // namespace vgraph
