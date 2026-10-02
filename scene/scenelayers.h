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

#include <filesystem>
#include <limits>
#include <set>

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

// placement applied to scenery statements by the origin, rotate and scale directives in effect
struct layer_context
{
	glm::dvec3 offset{0.0};
	glm::vec3 rotation{0.f};
	glm::vec3 scale{1.f};

	// converts world space location to the one which produces it when loaded with this context in effect
	glm::dvec3 to_local(glm::dvec3 Location) const;
	bool matches(layer_context const &Other) const;
};

// location of a piece of text in a scenery file, as byte offsets
struct source_span
{
	std::streamoff begin{-1};
	std::streamoff end{-1};

	bool valid() const
	{
		return begin >= 0 && end > begin;
	}
};

// include directive bringing a scenery file into another one
struct include_site
{
	layer_handle parent{null_handle}; // layer of the file holding the directive
	source_span span; // location of the whole directive in that file
	bool parameters{false}; // the file is included with parameters
	bool fixed{false}; // the directive can't be rewritten: it's a part of an *.inc template, or its location is unknown
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
	// source data, used by the scenery save
	std::vector<include_site> sites; // include directives bringing the file in; empty for the scenario file
	layer_context context_begin; // placement in effect at the start of the file...
	layer_context context_end; // ...and right after its end
	std::uintmax_t filesize{0}; // state of the file when it was loaded or saved the last time
	std::filesystem::file_time_type filetime{};
	// pending changes, applied by the scenery save
	bool created{false}; // layer was created in the editor, its file doesn't exist yet
	bool editormade{false}; // the file was made by the editor during this session, there's no original content to keep a backup of
	layer_handle anchor{null_handle}; // for created layer: sibling its include directive goes after
	bool removed{false}; // layer is to be dropped from the scenery
	layer_handle merged{null_handle}; // layer which took over the content of this one
	bool dead{false}; // removal or merge of the layer was saved, it's no longer a part of the scenery

	std::size_t item_count() const;
};

// include directive bringing an *.inc template into a scenery layer file
struct include_instance
{
	layer_handle layer{null_handle}; // layer of the file holding the directive
	std::string const *file{nullptr}; // name of the included template
	source_span span; // location of the whole directive in the file; invalid for directive which wasn't saved yet
	std::string directive; // text of the directive, for include placed in the editor and not saved yet
	layer_context context; // placement the directive of include placed in the editor was prepared for
	bool removed{false}; // the include is to be dropped from the scenery
	bool dead{false}; // removal of the include was saved
};

// marks nodes defined by a template which was included in a way the editor doesn't keep track of
instance_handle const untracked_instance{std::numeric_limits<instance_handle>::max()};

// origin of a node defined in a scenery layer file, with the placement it was loaded with
struct node_source
{
	layer_handle layer{null_handle}; // layer of the file holding the node definition
	source_span span; // location of the definition in that file
	layer_context context; // placement in effect at the definition
	glm::dvec3 location{0.0};
	glm::vec3 angles{0.f};
	glm::vec3 scale{1.f};
};

struct save_result
{
	bool success{false};
	std::vector<std::string> files; // names of saved files
	std::string message; // summary, or cause of the failure
};

// holds scenery layers, established during load of scenery opened for editing (-edit)
// NOTE: regular load doesn't create any layers, all nodes are left with null_handle
class node_layers
{
  public:
	// methods
	// indicates start of content of the scenario file. returns: handle to its layer
	layer_handle open(std::string const &Name);
	// indicates start of content of specified scenery file, included by specified directive. returns: handle to the layer of the file
	layer_handle open(std::string const &Name, include_site Site);
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
	// follows layer merges. returns: handle of the layer which currently holds content of specified layer
	layer_handle resolve(layer_handle Layer) const;
	// true if the layer is a part of the scenery and should be presented to the user
	bool listed(layer_handle const Layer) const
	{
		return valid(Layer) && false == layer(Layer).dead && layer(Layer).merged == null_handle;
	}
	// number of layers between specified layer and the scenario file
	int depth(layer_handle Layer) const;
	// handles of the layers to present to the user, each followed by the layers it includes
	std::vector<layer_handle> tree() const;
	// adjusts item count of specified layer
	void count(layer_handle Layer, layer_item const Item, int const Change = 1);
	// layer receiving the items created in the editor, or null_handle if there's none
	layer_handle active() const
	{
		return m_active;
	}
	// NOTE: active layer is made visible and unlocked
	void active(layer_handle Layer);
	// shows or hides nodes of specified layer. visibility defined in the scenery is preserved, hidden layer only overrides it
	void visible(layer_handle Layer, bool const Visible);
	void locked(layer_handle Layer, bool const Locked);
	// true if the file of specified layer can be rewritten with changed nodes. optionally explains why it can't
	bool writable(layer_handle Layer, std::string *Reason = nullptr) const;
	// true if nodes of specified layer can be selected and modified in the editor
	bool editable(layer_handle Layer) const;
	// true if specified node can be selected and modified in the editor. optionally explains why it can't
	bool editable(basic_node const *Node, std::string *Reason = nullptr) const;
	// moves specified node to specified layer
	void move(basic_node *Node, layer_handle Layer);
	// handles of layers which are currently hidden
	std::vector<layer_handle> hidden() const;
	// restores visibility of all layers
	void show_all();

	// scenery load bookkeeping
	// registers *.inc template used by the scenery
	void template_used(std::string const &Name)
	{
		m_templates.emplace(Name);
	}
	std::set<std::string> const &templates() const
	{
		return m_templates;
	}
	// indicates start of content of a template, included by directive at specified location of the layer file being loaded
	void instance_begin(std::string const &File, source_span const &Span);
	// indicates end of content of the template
	void instance_end()
	{
		m_instance = 0;
	}
	// include whose template is being loaded, 0 if there's none
	instance_handle instance() const
	{
		return m_instance;
	}
	// updates placement in effect for the statements which follow
	void context(layer_context const &Context)
	{
		m_context = Context;
	}
	// stores origin of a node defined directly in a layer file, to allow rewriting its definition on save
	void track(basic_node const *Node, source_span const &Span, glm::vec3 const &Angles = glm::vec3{0.f}, glm::vec3 const &Scale = glm::vec3{1.f});
	// indicates specified node was removed from the scene
	void forget(basic_node const *Node);
	// notes the scenery already holds the directive setting up terrain made in the editor
	void terrain_directive(bool const Present)
	{
		m_terraindirective = Present;
	}
	bool terrain_directive() const
	{
		return m_terraindirective;
	}

	// layer management. changes are applied to the scenery files by save()
	// creates empty layer, included from specified one. returns: handle to the new layer, or null_handle with the explanation in Reason
	layer_handle create(std::string Name, layer_handle Parent, std::string &Reason);
	bool can_remove(layer_handle Layer, std::string &Reason) const;
	// marks specified layer and the layers it includes as dropped from the scenery. their files are left on the disk
	bool remove(layer_handle Layer);
	// takes back remove() which wasn't saved yet
	void restore(layer_handle Layer);
	bool can_merge(layer_handle Source, layer_handle Target, std::string &Reason) const;
	// moves content of the source layer to the target layer
	bool merge(layer_handle Source, layer_handle Target);
	// true if there are layer changes awaiting save. NOTE: doesn't account for modified nodes
	bool pending() const;

	// includes of *.inc templates. the editor handles each as a whole: it can be placed and removed, but not taken apart
	// true if specified handle stands for an include the editor keeps track of
	bool tracked(instance_handle const Instance) const
	{
		return Instance != 0 && Instance <= m_instances.size();
	}
	// grants access to specified include. NOTE: the include has to be tracked
	include_instance const &instance(instance_handle const Instance) const
	{
		return m_instances[Instance - 1];
	}
	// true if specified include can be removed in the editor. optionally explains why it can't
	bool removable(instance_handle Instance, std::string *Reason = nullptr) const;
	// drops specified include from the scenery, or brings it back. its nodes are hidden right away, the directive
	// is erased from the layer file on save; what else the template defines stays in the scene until the next load
	void removed(instance_handle Instance, bool Removed);
	// registers include directive made in the editor, to be written to the file of specified layer on save.
	// the directive has to be prepared for the placement in effect at the end of that file. returns: handle to the include
	instance_handle place(layer_handle Layer, std::string const &File, std::string const &Directive);
	// true if specified layer is included, directly or not, by the other one
	bool is_descendant(layer_handle Layer, layer_handle const Ancestor) const;

	// writes changes made in the editor to the scenery files: changed, created and deleted model instances and
	// memory cells, and layer changes. the files are left intact unless the whole operation can be performed.
	// Rootstatements: additional statements to place at the end of the scenario file
	save_result save(std::vector<std::string> const &Rootstatements = {});

  private:
	// types
	struct file_patch;
	struct composition;
	struct save_state;
	// methods
	std::string path(layer_handle const Layer) const;
	void stat(layer_handle const Layer);
	bool compose(save_state &State, layer_handle const Layer, composition &Output, int const Depth) const;
	// members
	std::vector<basic_layer> m_layers; // layer handle is index in the vector + 1
	std::vector<layer_handle> m_stack; // helper, layers of scenery files being loaded
	layer_handle m_active{null_handle};
	layer_context m_context; // placement in effect at the current point of scenery load
	std::unordered_map<basic_node const *, node_source> m_sources; // origins of nodes which can be rewritten on save
	std::vector<std::pair<layer_handle, source_span>> m_erased; // definitions of nodes deleted since the load or the last save
	std::set<std::string> m_templates; // *.inc templates used by the scenery
	std::vector<include_instance> m_instances; // includes of the templates; instance handle is index in the vector + 1
	instance_handle m_instance{0}; // helper, include whose template is being loaded
	bool m_terraindirective{false};
};

extern node_layers Layers;

} // namespace scene

//---------------------------------------------------------------------------
