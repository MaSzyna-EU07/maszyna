/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "scene/scenenode.h"

namespace scene
{

// kinds of scenery items a layer keeps count of
enum class layer_item
{
	model,
	track,
	traction,
	powersource,
	memcell,
	launcher,
	event,
	vehicle,
	sound,
	shape,
	lines,
	count_
};

// scenery layer: content of a single scenery file (.scn, .scm, .ctr...) other than an .inc node template
struct basic_layer
{
	std::string name; // file name, relative to the scenery directory
	layer_handle parent{null_handle}; // layer of the file this one was included from
	bool visible{true};
	bool locked{false}; // items of locked layer can't be selected in the editor
	bool binary{false}; // the file wasn't parsed, its content came from binary terrain file
	std::array<std::size_t, static_cast<std::size_t>(layer_item::count_)> items{}; // amount of items of each kind

	std::size_t item_count() const;
};

// holds scenery layers, established during load of scenery opened for editing (-edit)
// NOTE: regular load doesn't create any layers, all nodes are left with null_handle
class node_layers
{
  public:
	// methods
	// indicates start of content of specified scenery file. returns: handle to the layer of the file
	layer_handle open(std::string const &Name);
	// indicates end of content of the current scenery file. returns: handle to the parent layer, or null_handle if the stack is empty
	layer_handle close();
	// returns layer of the scenery file being loaded, or null_handle if there's none
	layer_handle handle() const
	{
		return m_stack.empty() ? null_handle : m_stack.back();
	}
	// removes all layers; layer handles stored in the nodes aren't touched
	void clear();
	bool empty() const
	{
		return m_layers.empty();
	}
	std::size_t size() const
	{
		return m_layers.size();
	}
	bool valid(layer_handle const Layer) const
	{
		return Layer != null_handle && Layer <= m_layers.size();
	}
	// grants direct access to specified layer. NOTE: handle has to be valid
	basic_layer &layer(layer_handle const Layer)
	{
		return m_layers[Layer - 1];
	}
	basic_layer const &layer(layer_handle const Layer) const
	{
		return m_layers[Layer - 1];
	}
	// number of layers between specified layer and the scenario file
	int depth(layer_handle Layer) const;
	// adjusts item count of specified layer
	void count(layer_handle const Layer, layer_item const Item, int const Change = 1);
	// layer receiving the items created in the editor, or null_handle if there's none
	layer_handle active() const
	{
		return m_active;
	}
	// NOTE: active layer is made visible and unlocked
	void active(layer_handle const Layer);
	// shows or hides nodes of specified layer. visibility defined in the scenery is preserved, hidden layer only overrides it
	void visible(layer_handle const Layer, bool const Visible);
	void locked(layer_handle const Layer, bool const Locked);
	// true if nodes of specified layer can be selected and modified in the editor
	bool editable(layer_handle const Layer) const;
	// handles of layers which are currently hidden
	std::vector<layer_handle> hidden() const;
	// restores visibility of all layers
	void show_all();

  private:
	// members
	std::vector<basic_layer> m_layers; // layer handle is index in the vector + 1
	std::vector<layer_handle> m_stack; // helper, layers of scenery files being loaded
	layer_handle m_active{null_handle};
};

extern node_layers Layers;

} // namespace scene

//---------------------------------------------------------------------------
