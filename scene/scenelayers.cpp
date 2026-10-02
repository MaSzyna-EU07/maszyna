/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "scene/scenelayers.h"

#include "simulation/simulation.h"
#include "model/AnimModel.h"
#include "world/Track.h"
#include "world/Traction.h"

namespace scene
{

node_layers Layers;

std::size_t basic_layer::item_count() const
{
	return std::accumulate(std::begin(items), std::end(items), std::size_t{0});
}

// indicates start of content of specified scenery file. returns: handle to the layer of the file
layer_handle node_layers::open(std::string const &Name)
{
	// the same file can be included more than once, all its content goes to a single layer then
	auto const lookup{std::find_if(std::begin(m_layers), std::end(m_layers), [&](basic_layer const &Layer) { return Layer.name == Name; })};
	auto layerhandle{static_cast<layer_handle>(std::distance(std::begin(m_layers), lookup) + 1)};
	if (lookup == std::end(m_layers))
	{
		if (m_layers.size() < static_cast<std::size_t>(std::numeric_limits<layer_handle>::max()))
		{
			m_layers.emplace_back();
			m_layers.back().name = Name;
			m_layers.back().parent = handle();
		}
		else
		{
			// out of handles, content of the file is attributed to the parent layer
			layerhandle = handle();
		}
	}
	// NOTE: the stack receives an entry even if the layer wasn't created, to keep it in sync with close() calls
	m_stack.emplace_back(layerhandle);

	return layerhandle;
}

// indicates end of content of the current scenery file. returns: handle to the parent layer, or null_handle if the stack is empty
layer_handle node_layers::close()
{
	if (false == m_stack.empty())
	{
		m_stack.pop_back();
	}
	return handle();
}

void node_layers::clear()
{
	m_layers.clear();
	m_stack.clear();
	m_active = null_handle;
}

int node_layers::depth(layer_handle Layer) const
{
	auto depth{0};
	// NOTE: parent is always created before its children, which rules out cycles
	while (valid(Layer) && layer(Layer).parent != null_handle && layer(Layer).parent < Layer)
	{
		Layer = layer(Layer).parent;
		++depth;
	}
	return depth;
}

void node_layers::count(layer_handle const Layer, layer_item const Item, int const Change)
{
	if (false == valid(Layer))
	{
		return;
	}
	auto &itemcount{layer(Layer).items[static_cast<std::size_t>(Item)]};
	if (Change >= 0)
	{
		itemcount += static_cast<std::size_t>(Change);
	}
	else
	{
		itemcount -= std::min(itemcount, static_cast<std::size_t>(-Change));
	}
}

void node_layers::active(layer_handle const Layer)
{
	if (false == valid(Layer))
	{
		return;
	}
	// new items land in the active layer, they'd be out of reach right away if it was hidden or locked
	visible(Layer, true);
	locked(Layer, false);
	m_active = Layer;
}

// shows or hides nodes of specified layer. visibility defined in the scenery is preserved, hidden layer only overrides it
void node_layers::visible(layer_handle const Layer, bool const Visible)
{
	if (false == valid(Layer) || layer(Layer).visible == Visible)
	{
		return;
	}
	layer(Layer).visible = Visible;

	auto const update_node = [Layer, Visible](basic_node *Node) {
		if (Node == nullptr || Node->layer() != Layer)
		{
			return;
		}
		if (Visible)
		{
			if (Node->m_layerhidden)
			{
				Node->visible(true);
				Node->m_layerhidden = false;
			}
		}
		else if (Node->visible())
		{
			// nodes which are invisible on their own aren't marked, so they stay that way when the layer is shown again
			Node->visible(false);
			Node->m_layerhidden = true;
		}
	};
	// only these node types are drawn by the renderers basing on their visibility flag
	for (auto *instance : simulation::Instances.sequence())
	{
		update_node(instance);
	}
	for (auto *path : simulation::Paths.sequence())
	{
		update_node(path);
	}
	for (auto *traction : simulation::Traction.sequence())
	{
		update_node(traction);
	}
}

void node_layers::locked(layer_handle const Layer, bool const Locked)
{
	if (valid(Layer))
	{
		layer(Layer).locked = Locked;
	}
}

// true if nodes of specified layer can be selected and modified in the editor
bool node_layers::editable(layer_handle const Layer) const
{
	// nodes without a layer (regular load, or created by the simulation) aren't restricted
	return false == valid(Layer) || (layer(Layer).visible && false == layer(Layer).locked);
}

std::vector<layer_handle> node_layers::hidden() const
{
	std::vector<layer_handle> layers;
	for (std::size_t idx = 0; idx < m_layers.size(); ++idx)
	{
		if (false == m_layers[idx].visible)
		{
			layers.emplace_back(static_cast<layer_handle>(idx + 1));
		}
	}
	return layers;
}

void node_layers::show_all()
{
	for (auto const hiddenlayer : hidden())
	{
		visible(hiddenlayer, true);
	}
}

} // namespace scene

//---------------------------------------------------------------------------
