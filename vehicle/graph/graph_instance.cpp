/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/graph/graph_instance.h"
#include "vehicle/graph/graph_catalog.h"

namespace vgraph
{

graph_instance::graph_instance(std::shared_ptr<graph_template const> Template)
    : m_template(std::move(Template)), m_signals(m_template->signal_defaults()), m_forced(m_signals.size(), 0), m_links(m_template->link_count())
{
	auto const &nodes{m_template->nodes()};
	m_blocks.reserve(nodes.size());
	for (auto const &node : nodes)
	{
		block_setup const setup{*node.type, node.params, node.slots, node.connected};
		m_blocks.emplace_back(node.type->create(setup));
	}
	m_order.reserve(nodes.size());
	for (auto const nodeindex : m_template->order())
	{
		m_order.push_back(m_blocks[nodeindex].get());
		if (nodes[nodeindex].type->deferred)
		{
			m_deferred.push_back(m_blocks[nodeindex].get());
		}
	}
	// deferred blocks publish their initial state, so the blocks they feed don't see zeros in the first step
	auto frame{make_frame(0.0)};
	for (auto *item : m_deferred)
	{
		item->commit(frame);
	}
}

frame graph_instance::make_frame(double const Deltatime)
{
	frame result;
	result.dt = Deltatime;
	result.time = m_time;
	result.low_voltage = m_template->low_voltage_simulation();
	result.signals = m_signals.data();
	result.forced = m_forced.data();
	result.links = m_links.data();
	return result;
}

void graph_instance::update(double const Deltatime)
{
	m_time += Deltatime;
	auto frame{make_frame(Deltatime)};
	for (auto *item : m_order)
	{
		item->update(frame);
	}
	for (auto *item : m_deferred)
	{
		item->commit(frame);
	}
	for (auto &link : m_links)
	{
		link.back = link.back_next;
		link.back_next = 0.0;
	}
}

block *graph_instance::find_block(std::string_view const Id)
{
	auto const nodeindex{m_template->find_node(Id)};
	return (nodeindex >= 0 ? m_blocks[nodeindex].get() : nullptr);
}

void graph_instance::force(int const Slot, double const Value)
{
	m_signals[Slot] = Value;
	m_forced[Slot] = 1;
}

void graph_instance::release(int const Slot)
{
	m_forced[Slot] = 0;
}

} // namespace vgraph
