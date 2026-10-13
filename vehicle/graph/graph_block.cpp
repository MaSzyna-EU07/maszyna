/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/graph/graph_block.h"
#include "vehicle/graph/graph_catalog.h"

namespace vgraph
{

block::block(block_setup const &Setup) : m_type(Setup.type), m_slots(Setup.slots) {}

double block::number(block_setup const &Setup, std::size_t const Param)
{
	return Setup.params[Param].number;
}

bool block::boolean(block_setup const &Setup, std::size_t const Param)
{
	return Setup.params[Param].boolean;
}

std::string const &block::text(block_setup const &Setup, std::size_t const Param)
{
	return Setup.params[Param].text;
}

int block::choice(block_setup const &Setup, std::size_t const Param)
{
	auto const &choices{Setup.type.params[Param].choices};
	auto const &value{Setup.params[Param].text};
	for (std::size_t idx = 0; idx < choices.size(); ++idx)
	{
		if (choices[idx] == value)
		{
			return static_cast<int>(idx);
		}
	}
	// the loader rejects values outside of the list, so this only happens with a broken catalog
	return 0;
}

} // namespace vgraph
