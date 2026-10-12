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
#include "utilities/Globals.h"
#include "utilities/utilities.h"

#include <array>
#include <fstream>
#include <functional>
#include <iterator>

namespace scene
{

node_layers Layers;

// converts world space location to the one which produces it when loaded with this context in effect
glm::dvec3 layer_context::to_local(glm::dvec3 Location) const
{
	// reverse of the transformation done by the scenery loader: rotation around the vertical axis, then scale, then offset
	Location -= offset;
	Location /= glm::dvec3{scale};
	if (rotation != glm::vec3{0.f})
	{
		Location = glm::rotateY<double>(Location, -glm::radians(static_cast<double>(rotation.y)));
	}
	return Location;
}

glm::dvec3 layer_context::to_world(glm::dvec3 Location) const
{
	if (rotation != glm::vec3{0.f})
	{
		Location = glm::rotateY<double>(Location, glm::radians(static_cast<double>(rotation.y)));
	}
	Location *= glm::dvec3{scale};
	Location += offset;
	return Location;
}

bool layer_context::matches(layer_context const &Other) const
{
	return glm::all(glm::epsilonEqual(offset, Other.offset, 1e-4)) && glm::all(glm::epsilonEqual(rotation, Other.rotation, 1e-4f)) && glm::all(glm::epsilonEqual(scale, Other.scale, 1e-5f));
}

std::size_t basic_layer::item_count() const
{
	return std::accumulate(std::begin(items), std::end(items), std::size_t{0});
}

// indicates start of content of the scenario file. returns: handle to its layer
layer_handle node_layers::open(std::string const &Name)
{
	// the same file can be included more than once, all its content goes to a single layer then
	auto const lookup{std::find_if(std::begin(m_layers), std::end(m_layers), [&](basic_layer const &Layer) { return Layer.name == Name && false == Layer.dead; })};
	auto layerhandle{static_cast<layer_handle>(std::distance(std::begin(m_layers), lookup) + 1)};
	if (lookup == std::end(m_layers))
	{
		if (m_layers.size() < static_cast<std::size_t>(std::numeric_limits<layer_handle>::max()))
		{
			m_layers.emplace_back();
			m_layers.back().name = Name;
			m_layers.back().parent = handle();
			m_layers.back().late = m_initialized;
			stat(layerhandle);
			scan(layerhandle);
		}
		else
		{
			// out of handles, content of the file is attributed to the parent layer
			layerhandle = handle();
		}
	}
	if (valid(layerhandle) && layerhandle != handle())
	{
		layer(layerhandle).context_begin = m_context;
	}
	// NOTE: the stack receives an entry even if the layer wasn't created, to keep it in sync with close() calls
	m_stack.emplace_back(layerhandle);

	return layerhandle;
}

// indicates start of content of specified scenery file, included by specified directive. returns: handle to the layer of the file
layer_handle node_layers::open(std::string const &Name, include_site Site)
{
	Site.parent = handle();
	auto const layerhandle{open(Name)};
	if (valid(layerhandle) && layerhandle != Site.parent)
	{
		layer(layerhandle).sites.emplace_back(Site);
	}
	return layerhandle;
}

// indicates end of content of the current scenery file. returns: handle to the parent layer, or null_handle if the stack is empty
layer_handle node_layers::close()
{
	if (false == m_stack.empty())
	{
		auto const closing{m_stack.back()};
		m_stack.pop_back();
		if (valid(closing) && closing != handle())
		{
			layer(closing).context_end = m_context;
		}
	}
	return handle();
}

void node_layers::clear()
{
	m_layers.clear();
	m_stack.clear();
	m_active = null_handle;
	m_context = layer_context();
	m_sources.clear();
	m_erased.clear();
	m_renamed.clear();
	m_shapes.clear();
	m_shapeserased = false;
	m_templates.clear();
	m_instances.clear();
	m_instance = 0;
	m_initialized = false;
	m_terraindirective = false;
	m_marked.clear();
	m_marking.clear();
}

// indicates the scenario initialization (FirstInit) is being performed
void node_layers::initialization(source_span const &Span, bool const Infile)
{
	if (m_initialized)
	{
		return;
	}
	m_initialized = true;
	if (m_stack.empty() || false == valid(m_stack.back()))
	{
		return;
	}
	// the layer file being loaded holds the directive, or includes the template which does
	auto &holder{layer(m_stack.back())};
	if (Infile)
	{
		// NOTE: scenario without the directive is initialized at its end, with nothing in the file to point at
		std::string text(Span.valid() ? static_cast<std::size_t>(Span.end - Span.begin) : 0, '\0');
		std::ifstream input{path(m_stack.back()), std::ios_base::binary};
		if (text.empty() || false == input.is_open() || false == input.seekg(Span.begin).good() || false == input.read(text.data(), static_cast<std::streamsize>(text.size())).good() ||
		    ToLower(text) != "firstinit")
		{
			return;
		}
		holder.init = Span;
	}
	else if (tracked(m_instance) && instance(m_instance).layer == m_stack.back())
	{
		holder.init = instance(m_instance).span;
	}
	else
	{
		return;
	}
	holder.context_init = m_context;
	// for the files it was included through the initialization takes place at their include directives
	for (auto level{m_stack.size() - 1}; level > 0; --level)
	{
		auto const child{m_stack[level]};
		auto const parent{m_stack[level - 1]};
		if (false == valid(parent) || parent == child || layer(child).sites.empty() || layer(child).sites.back().parent != parent || layer(child).sites.back().fixed ||
		    false == layer(child).sites.back().span.valid())
		{
			break;
		}
		layer(parent).init = layer(child).sites.back().span;
		layer(parent).context_init = layer(child).context_begin;
	}
}

// true if specified layer can receive content added in the editor
bool node_layers::accepts(layer_handle Layer, std::string *Reason) const
{
	Layer = resolve(Layer);
	if (false == valid(Layer) || layer(Layer).removed)
	{
		return false;
	}
	if (false == writable(Layer, Reason))
	{
		return false;
	}
	if (layer(Layer).late)
	{
		if (Reason != nullptr)
		{
			*Reason = "the file is loaded after the scenario initialization (FirstInit), which only vehicles should follow";
		}
		return false;
	}
	return true;
}

// follows layer merges. returns: handle of the layer which currently holds content of specified layer
layer_handle node_layers::resolve(layer_handle Layer) const
{
	// NOTE: merge() rules out cycles, the counter is only a safeguard
	for (auto hops = 0; hops < 64 && valid(Layer) && layer(Layer).merged != null_handle; ++hops)
	{
		Layer = layer(Layer).merged;
	}
	return Layer;
}

int node_layers::depth(layer_handle Layer) const
{
	auto depth{0};
	Layer = resolve(Layer);
	while (valid(Layer) && layer(Layer).parent != null_handle && depth < 64)
	{
		Layer = resolve(layer(Layer).parent);
		++depth;
	}
	return depth;
}

std::vector<layer_handle> node_layers::tree() const
{
	std::vector<layer_handle> order;
	order.reserve(m_layers.size());
	// NOTE: the layers are few, plain scans do
	std::function<void(layer_handle)> const add = [&](layer_handle const Parent) {
		for (std::size_t idx = 0; idx < m_layers.size(); ++idx)
		{
			auto const candidate{static_cast<layer_handle>(idx + 1)};
			if (listed(candidate) && resolve(layer(candidate).parent) == Parent && order.size() < m_layers.size())
			{
				order.emplace_back(candidate);
				add(candidate);
			}
		}
	};
	add(null_handle);
	return order;
}

void node_layers::count(layer_handle Layer, layer_item const Item, int const Change)
{
	Layer = resolve(Layer);
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

void node_layers::active(layer_handle Layer)
{
	Layer = resolve(Layer);
	if (false == accepts(Layer))
	{
		return;
	}
	// new items land in the active layer, they'd be out of reach right away if it was hidden or locked
	visible(Layer, true);
	locked(Layer, false);
	m_active = Layer;
}

// shows or hides nodes of specified layer. visibility defined in the scenery is preserved, hidden layer only overrides it
void node_layers::visible(layer_handle Layer, bool const Visible)
{
	Layer = resolve(Layer);
	if (false == valid(Layer) || layer(Layer).visible == Visible)
	{
		return;
	}
	layer(Layer).visible = Visible;

	auto const update_node = [this, Layer, Visible](basic_node *Node) {
		if (Node == nullptr || resolve(Node->layer()) != Layer)
		{
			return;
		}
		if (Visible)
		{
			// nodes of a removed include stay out of sight, and so do the ones the editor replaced with its own
			if (Node->m_layerhidden && (false == tracked(Node->m_instance) || (false == instance(Node->m_instance).removed && false == stale(Node))))
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

void node_layers::locked(layer_handle Layer, bool const Locked)
{
	Layer = resolve(Layer);
	if (valid(Layer))
	{
		layer(Layer).locked = Locked;
	}
}

// true if the file of specified layer can be rewritten with changed nodes. optionally explains why it can't
bool node_layers::writable(layer_handle Layer, std::string *Reason) const
{
	Layer = resolve(Layer);
	if (false == valid(Layer))
	{
		return false;
	}
	auto const &target{layer(Layer)};
	char const *reason{nullptr};
	if (target.binary)
	{
		reason = "the file is loaded from binary terrain cache";
	}
	else if (target.sites.size() > 1)
	{
		// one definition yields several nodes, there's no telling which of them should be written back
		reason = "the file is included more than once";
	}
	else if (false == target.sites.empty() && target.sites.front().parameters)
	{
		reason = "the file is included with parameters";
	}
	if (reason != nullptr && Reason != nullptr)
	{
		*Reason = reason;
	}
	return reason == nullptr;
}

// true if nodes of specified layer can be selected and modified in the editor
bool node_layers::editable(layer_handle Layer) const
{
	Layer = resolve(Layer);
	// nodes without a layer (regular load, or created by the simulation) aren't restricted
	return false == valid(Layer) || (layer(Layer).visible && false == layer(Layer).locked && false == layer(Layer).removed && writable(Layer));
}

// true if specified node can be selected and modified in the editor. optionally explains why it can't
bool node_layers::editable(basic_node const *Node, std::string *Reason) const
{
	if (Node == nullptr)
	{
		return false;
	}
	auto const nodelayer{resolve(Node->layer())};
	if (false == valid(nodelayer))
	{
		return true;
	}
	std::string reason;
	if (Node->from_template())
	{
		reason = "it's a part of an include template (.inc)";
	}
	else if (false == writable(nodelayer, &reason))
	{
		// reason was filled by the call
	}
	else if (layer(nodelayer).locked)
	{
		reason = "its layer is locked";
	}
	else if (false == layer(nodelayer).visible || layer(nodelayer).removed)
	{
		reason = "its layer is hidden";
	}
	if (false == reason.empty() && Reason != nullptr)
	{
		*Reason = reason;
	}
	return reason.empty();
}

// moves specified node to specified layer
void node_layers::move(basic_node *Node, layer_handle Layer)
{
	Layer = resolve(Layer);
	if (Node == nullptr || false == valid(Layer) || resolve(Node->layer()) == Layer)
	{
		return;
	}
	// NOTE: the only nodes the editor creates and moves between layers are model instances
	count(Node->layer(), layer_item::model, -1);
	Node->layer(Layer);
	count(Layer, layer_item::model, 1);
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
		// layers dropped from the scenery stay out of sight
		if (false == layer(hiddenlayer).removed)
		{
			visible(hiddenlayer, true);
		}
	}
}

// stores origin of a node defined directly in a layer file, to allow rewriting its definition on save
void node_layers::track(basic_node const *Node, source_span const &Span, glm::vec3 const &Angles, glm::vec3 const &Scale)
{
	if (Node == nullptr || Node->from_template() || false == valid(Node->layer()) || false == Span.valid())
	{
		return;
	}
	m_sources[Node] = {Node->layer(), Span, m_context, Node->location(), Angles, Scale};
}

// indicates specified node was removed from the scene
void node_layers::forget(basic_node const *Node)
{
	auto const lookup{m_sources.find(Node)};
	if (lookup == m_sources.end())
	{
		return;
	}
	m_erased.emplace_back(lookup->second.layer, lookup->second.span);
	m_sources.erase(lookup);
	m_renamed.erase(Node);
}

// indicates specified node was given another name in the editor
void node_layers::renamed(basic_node const *Node)
{
	// NOTE: a node which isn't defined by a scenery file yet is written whole on save, along with its name
	if (m_sources.find(Node) != m_sources.end())
	{
		m_renamed.emplace(Node);
	}
}

void node_layers::shape(material_handle const Material, source_span const &Span)
{
	auto const current{handle()};
	if (false == valid(current) || false == Span.valid() || m_instance != 0)
	{
		return;
	}
	m_shapes.push_back({current, Span, Material, false});
}

std::pair<std::size_t, std::size_t> node_layers::erase_shapes(std::set<material_handle> const &Materials, std::set<layer_handle> const *Layers)
{
	std::pair<std::size_t, std::size_t> result{0, 0};
	for (auto &shape : m_shapes)
	{
		if (shape.erased || Materials.count(shape.material) == 0 || (Layers != nullptr && Layers->count(shape.layer) == 0))
		{
			continue;
		}
		if (false == writable(shape.layer))
		{
			++result.second;
			continue;
		}
		shape.erased = true;
		m_erased.emplace_back(shape.layer, shape.span);
		m_shapeserased = true;
		++result.first;
	}
	return result;
}

bool node_layers::is_descendant(layer_handle Layer, layer_handle const Ancestor) const
{
	// NOTE: content of a merged layer, includes of other files among it, belongs to the layer it was merged into
	for (auto hops = 0; hops < 64 && valid(Layer); ++hops)
	{
		Layer = resolve(layer(Layer).parent);
		if (Layer == Ancestor)
		{
			return true;
		}
	}
	return false;
}

std::string node_layers::path(layer_handle const Layer) const
{
	return Global.asCurrentSceneryPath + layer(Layer).name;
}

// stores current state of the layer file, to detect later whether it was changed behind our back
void node_layers::stat(layer_handle const Layer)
{
	std::error_code error;
	auto const filepath{std::filesystem::path(path(Layer))};
	auto const filesize{std::filesystem::file_size(filepath, error)};
	layer(Layer).filesize = error ? 0 : filesize;
	auto const filetime{std::filesystem::last_write_time(filepath, error)};
	layer(Layer).filetime = error ? std::filesystem::file_time_type{} : filetime;
}

// creates empty layer, included from specified one. returns: handle to the new layer, or null_handle with the explanation in Reason
layer_handle node_layers::create(std::string Name, layer_handle Parent, std::string &Reason)
{
	Parent = resolve(Parent);
	if (false == valid(Parent) || layer(Parent).removed || layer(Parent).binary)
	{
		Reason = "the parent layer can't include other files";
		return null_handle;
	}
	if (layer(Parent).late)
	{
		Reason = "the parent layer is loaded after the scenario initialization (FirstInit), which only vehicles should follow";
		return null_handle;
	}
	// file names in the scenery are processed in lower case, with forward slashes
	Name = ToLower(Name);
	replace_slashes(Name);
	Name.erase(0, Name.find_first_not_of(" \t"));
	Name.erase(Name.find_last_not_of(" \t") + 1);
	if (Name.empty())
	{
		Reason = "the layer needs a file name";
		return null_handle;
	}
	if (Name.find_first_of(" \t\";:*?<>|") != std::string::npos || Name.front() == '/' || Name.back() == '/' || contains(Name, ".."))
	{
		Reason = "the file name has to be a plain path inside the scenery directory, without spaces";
		return null_handle;
	}
	if (std::filesystem::path(Name).extension().empty())
	{
		Name += ".scm";
	}
	if (Name.ends_with(".inc"))
	{
		Reason = ".inc files are include templates, not layers";
		return null_handle;
	}
	if (m_layers.size() >= static_cast<std::size_t>(std::numeric_limits<layer_handle>::max()))
	{
		Reason = "too many layers";
		return null_handle;
	}
	auto const taken{std::any_of(std::begin(m_layers), std::end(m_layers), [&](basic_layer const &Layer) { return Layer.name == Name && false == Layer.dead; })};
	if (taken || FileExists(Global.asCurrentSceneryPath + Name))
	{
		Reason = "the file \"" + Name + "\" already exists";
		return null_handle;
	}

	// the include directive goes after the last file included by the parent ahead of the scenario initialization,
	// so the scenery files stay ahead of what follows them in a scenario, like the vehicles
	auto anchor{null_handle};
	std::streamoff anchorend{-1};
	auto const &init{layer(Parent).init};
	for (std::size_t idx = 0; idx < m_layers.size(); ++idx)
	{
		auto const &sibling{m_layers[idx]};
		if (sibling.dead || sibling.created || sibling.sites.size() != 1)
		{
			continue;
		}
		auto const &site{sibling.sites.front()};
		if (site.parent == Parent && false == site.fixed && site.span.end > anchorend && (false == init.valid() || site.span.end <= init.begin))
		{
			anchor = static_cast<layer_handle>(idx + 1);
			anchorend = site.span.end;
		}
	}

	m_layers.emplace_back();
	auto const layerhandle{static_cast<layer_handle>(m_layers.size())};
	auto &created{m_layers.back()};
	created.name = Name;
	created.parent = Parent;
	created.created = true;
	created.editormade = true;
	created.anchor = anchor;
	// placement the new file is going to be loaded with
	created.context_begin = valid(anchor) ? layer(anchor).context_end : layer(Parent).context_insert();
	created.context_end = created.context_begin;

	active(layerhandle);

	return layerhandle;
}

bool node_layers::can_remove(layer_handle Layer, std::string &Reason) const
{
	if (false == listed(Layer) || layer(Layer).removed)
	{
		Reason = "no such layer";
		return false;
	}
	auto const &target{layer(Layer)};
	if (target.created)
	{
		return true;
	}
	if (target.sites.empty())
	{
		Reason = "the scenario file can't be removed";
		return false;
	}
	if (target.binary)
	{
		Reason = "the file is loaded from binary terrain cache";
		return false;
	}
	if (target.sites.size() > 1)
	{
		Reason = "the file is included more than once";
		return false;
	}
	if (target.sites.front().fixed)
	{
		Reason = "the file is included by an include template (.inc)";
		return false;
	}
	return true;
}

// marks specified layer and the layers it includes as dropped from the scenery. their files are left on the disk
bool node_layers::remove(layer_handle Layer)
{
	std::string reason;
	if (false == can_remove(Layer, reason))
	{
		return false;
	}
	for (std::size_t idx = 0; idx < m_layers.size(); ++idx)
	{
		auto const candidate{static_cast<layer_handle>(idx + 1)};
		// layers merged elsewhere keep going, their content is no longer a part of the removed file
		if (false == listed(candidate) || (candidate != Layer && false == is_descendant(candidate, Layer)))
		{
			continue;
		}
		visible(candidate, false);
		layer(candidate).removed = true;
		if (m_active == candidate)
		{
			m_active = null_handle;
		}
	}
	if (m_active == null_handle)
	{
		// fall back on the scenario file, which is always there
		active(1);
	}
	return true;
}

// takes back remove() which wasn't saved yet
void node_layers::restore(layer_handle Layer)
{
	if (false == valid(Layer) || false == layer(Layer).removed || layer(Layer).dead)
	{
		return;
	}
	// a layer can't be brought back without the file which includes it
	auto const parent{resolve(layer(Layer).parent)};
	if (valid(parent) && layer(parent).removed)
	{
		restore(parent);
	}
	for (std::size_t idx = 0; idx < m_layers.size(); ++idx)
	{
		auto const candidate{static_cast<layer_handle>(idx + 1)};
		if (false == listed(candidate) || (candidate != Layer && false == is_descendant(candidate, Layer)))
		{
			continue;
		}
		layer(candidate).removed = false;
		visible(candidate, true);
	}
}

bool node_layers::can_merge(layer_handle Source, layer_handle Target, std::string &Reason) const
{
	if (false == listed(Source) || false == listed(Target) || layer(Source).removed || layer(Target).removed)
	{
		Reason = "no such layer";
		return false;
	}
	if (Source == Target)
	{
		Reason = "pick another layer to merge into";
		return false;
	}
	auto const &source{layer(Source)};
	auto const &target{layer(Target)};
	if (false == writable(Target, &Reason))
	{
		return false;
	}
	if (is_descendant(Target, Source))
	{
		// the merged text would include the file it was put in
		Reason = "the target layer is included by the merged one";
		return false;
	}
	if (target.late != source.late && (source.created || source.sites.empty() || resolve(source.sites.front().parent) != Target))
	{
		// the content would change sides of the scenario initialization
		Reason = (target.late ? "the target layer is loaded after the scenario initialization (FirstInit), which only vehicles should follow" :
		                        "the merged layer is loaded after the scenario initialization (FirstInit), its content has to stay there");
		return false;
	}
	if (false == source.context_insert().matches(target.context_insert()))
	{
		// directives of includes placed in the editor are prepared for the place their layer file receives them at
		auto const unsaved{std::any_of(std::begin(m_instances), std::end(m_instances), [&](include_instance const &Instance) {
			return false == Instance.dead && false == Instance.removed && false == Instance.span.valid() && resolve(Instance.layer) == Source;
		})};
		if (unsaved)
		{
			Reason = "the merged layer has includes which weren't saved yet, save the scenery first";
			return false;
		}
	}
	if (source.created)
	{
		// nothing but the nodes made in the editor to move
		return true;
	}
	if (source.sites.empty())
	{
		Reason = "the scenario file can't be merged into another layer";
		return false;
	}
	if (false == writable(Source, &Reason))
	{
		return false;
	}
	if (source.sites.front().fixed)
	{
		Reason = "the file is included by an include template (.inc)";
		return false;
	}
	if (resolve(source.sites.front().parent) != Target)
	{
		// the text is moved to another file. its statements have to keep their meaning there,
		// and can't leave anything behind for what used to follow it
		if (false == source.context_begin.matches(target.context_insert()) || false == source.context_end.matches(source.context_begin))
		{
			Reason = "the layers are loaded with different origin, rotation or scale";
			return false;
		}
	}
	return true;
}

// moves content of the source layer to the target layer
bool node_layers::merge(layer_handle Source, layer_handle Target)
{
	std::string reason;
	if (false == can_merge(Source, Target, reason))
	{
		return false;
	}
	// start with the same visibility on both sides, as it's kept per node
	visible(Source, true);
	visible(Target, true);
	for (std::size_t idx = 0; idx < layer(Source).items.size(); ++idx)
	{
		layer(Target).items[idx] += layer(Source).items[idx];
	}
	layer(Source).merged = Target;
	if (m_active == Source)
	{
		active(Target);
	}
	return true;
}

// true if there are layer changes awaiting save. NOTE: doesn't account for modified nodes
bool node_layers::pending() const
{
	return false == m_marking.empty() ||
	       std::any_of(std::begin(m_layers), std::end(m_layers), [](basic_layer const &Layer) { return false == Layer.dead && (Layer.created || Layer.removed || Layer.merged != null_handle); });
}

namespace
{
// marks of the lines the editor keeps track of
std::array<char const *, 3> const editormarks{"//$p", "//$b", "//$c"};
} // namespace

void node_layers::scan(layer_handle const Layer)
{
	m_marked.erase(Layer);
	std::ifstream file{path(Layer), std::ios_base::binary};
	if (false == file.is_open())
	{
		return;
	}
	std::string content{std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
	if (content.find("//$") == std::string::npos)
	{
		return;
	}
	std::size_t begin{0};
	while (begin < content.size())
	{
		auto end{content.find('\n', begin)};
		end = (end == std::string::npos ? content.size() : end + 1);
		auto const first{content.find_first_not_of(" \t", begin)};
		if (first < end)
		{
			for (auto const *mark : editormarks)
			{
				auto const length{std::char_traits<char>::length(mark)};
				if (content.compare(first, length, mark) != 0 || (first + length < end && content[first + length] != ' ' && content[first + length] != '\t' && content[first + length] != '\r' && content[first + length] != '\n'))
				{
					continue;
				}
				auto text{content.substr(first + length, end - first - length)};
				while (false == text.empty() && (text.back() == '\n' || text.back() == '\r'))
				{
					text.pop_back();
				}
				text.erase(0, text.find_first_not_of(" \t"));
				m_marked[Layer].push_back({mark, {static_cast<std::streamoff>(begin), static_cast<std::streamoff>(end)}, std::move(text)});
				break;
			}
		}
		begin = end;
	}
}

std::vector<std::string> node_layers::marked(layer_handle const Layer, std::string const &Mark) const
{
	std::vector<std::string> result;
	if (auto const pending{m_marking.find({Layer, Mark})}; pending != m_marking.end())
	{
		return pending->second;
	}
	if (auto const lookup{m_marked.find(Layer)}; lookup != m_marked.end())
	{
		for (auto const &line : lookup->second)
		{
			if (line.mark == Mark)
			{
				result.push_back(line.text);
			}
		}
	}
	return result;
}

std::vector<layer_handle> node_layers::marked_layers(std::string const &Mark) const
{
	std::vector<layer_handle> result;
	for (auto const &entry : m_marked)
	{
		if (listed(entry.first) && std::any_of(entry.second.begin(), entry.second.end(), [&](marked_line const &Line) { return Line.mark == Mark; }))
		{
			result.push_back(entry.first);
		}
	}
	for (auto const &entry : m_marking)
	{
		if (entry.first.second == Mark && std::find(result.begin(), result.end(), entry.first.first) == result.end())
		{
			result.push_back(entry.first.first);
		}
	}
	return result;
}

void node_layers::mark(layer_handle const Layer, std::string const &Mark, std::vector<std::string> Lines)
{
	if (valid(Layer))
	{
		m_marking[{Layer, Mark}] = std::move(Lines);
	}
}

// indicates start of content of a template, included by directive at specified location of the layer file being loaded
void node_layers::instance_begin(std::string const &File, source_span const &Span)
{
	if (m_instances.size() + 1 >= untracked_instance)
	{
		m_instance = untracked_instance;
		return;
	}
	m_instances.emplace_back();
	auto &included{m_instances.back()};
	included.layer = handle();
	included.file = &(*m_templates.emplace(File).first);
	included.span = Span;
	included.context = m_context;
	m_instance = static_cast<instance_handle>(m_instances.size());
}

// true if specified include can be removed in the editor. optionally explains why it can't
bool node_layers::removable(instance_handle Instance, std::string *Reason) const
{
	std::string reason;
	if (false == tracked(Instance) || instance(Instance).dead)
	{
		reason = "it's a part of an include template (.inc)";
	}
	else
	{
		auto const includelayer{resolve(instance(Instance).layer)};
		if (false == valid(includelayer) || layer(includelayer).removed)
		{
			reason = "its layer is no longer a part of the scenery";
		}
		else if (false == writable(includelayer, &reason))
		{
			// reason was filled by the call
		}
		else if (layer(includelayer).locked)
		{
			reason = "its layer is locked";
		}
	}
	if (false == reason.empty() && Reason != nullptr)
	{
		*Reason = reason;
	}
	return reason.empty();
}

// drops specified include from the scenery, or brings it back
void node_layers::removed(instance_handle Instance, bool Removed)
{
	if (false == tracked(Instance) || m_instances[Instance - 1].dead || m_instances[Instance - 1].removed == Removed)
	{
		return;
	}
	m_instances[Instance - 1].removed = Removed;

	auto const includelayer{resolve(m_instances[Instance - 1].layer)};
	auto const layervisible{false == valid(includelayer) || layer(includelayer).visible};
	auto const update_node = [=, this](basic_node *Node, layer_item const Item) {
		if (Node == nullptr || Node->m_instance != Instance || (Item == layer_item::model && stale(Node)))
		{
			return;
		}
		count(Node->layer(), Item, Removed ? -1 : 1);
		if (Removed)
		{
			if (Node->visible())
			{
				Node->visible(false);
				Node->m_layerhidden = true;
			}
		}
		else if (Node->m_layerhidden && layervisible)
		{
			Node->visible(true);
			Node->m_layerhidden = false;
		}
	};
	for (auto *modelinstance : simulation::Instances.sequence())
	{
		update_node(modelinstance, layer_item::model);
	}
	for (auto *path : simulation::Paths.sequence())
	{
		update_node(path, layer_item::track);
	}
	for (auto *traction : simulation::Traction.sequence())
	{
		update_node(traction, layer_item::traction);
	}
}

// registers include directive made in the editor, to be written to the file of specified layer on save
instance_handle node_layers::place(layer_handle Layer, std::string const &File, std::string const &Directive)
{
	Layer = resolve(Layer);
	if (false == accepts(Layer) || m_instances.size() + 1 >= untracked_instance)
	{
		return 0;
	}
	m_instances.emplace_back();
	auto &included{m_instances.back()};
	included.layer = Layer;
	included.file = &(*m_templates.emplace(File).first);
	included.directive = Directive;
	included.context = layer(Layer).context_insert();
	return static_cast<instance_handle>(m_instances.size());
}

// text of the directive of specified include, as changed in the editor or as it stands in the scenery file
std::string node_layers::directive(instance_handle const Instance) const
{
	if (false == tracked(Instance))
	{
		return {};
	}
	auto const &included{instance(Instance)};
	if (false == included.directive.empty() || false == included.span.valid() || false == valid(included.layer))
	{
		return included.directive;
	}
	std::string text(static_cast<std::size_t>(included.span.end - included.span.begin), '\0');
	std::ifstream input{path(included.layer), std::ios_base::binary};
	if (false == input.is_open() || false == input.seekg(included.span.begin).good() || false == input.read(text.data(), static_cast<std::streamsize>(text.size())).good())
	{
		return {};
	}
	return text;
}

// replaces the directive of specified include, the scenery file receives it on save
bool node_layers::modify(instance_handle const Instance, std::string const &Directive)
{
	if (false == tracked(Instance) || m_instances[Instance - 1].dead || Directive.empty())
	{
		return false;
	}
	m_instances[Instance - 1].directive = Directive;
	return true;
}

// takes the models specified include was loaded with out of the scene, once the editor shows its own in their place
void node_layers::rebuilt(instance_handle const Instance)
{
	if (false == tracked(Instance) || m_instances[Instance - 1].rebuilt)
	{
		return;
	}
	auto const removed{m_instances[Instance - 1].removed};
	for (auto *model : simulation::Instances.sequence())
	{
		if (model == nullptr || model->m_instance != Instance || model->m_preview)
		{
			continue;
		}
		// NOTE: the models themselves are left alone, events of the scenery can refer to them
		simulation::Region->erase(model);
		if (false == removed)
		{
			count(model->layer(), layer_item::model, -1);
		}
		if (model->visible())
		{
			model->visible(false);
			model->m_layerhidden = true;
		}
	}
	m_instances[Instance - 1].rebuilt = true;
}

} // namespace scene

//---------------------------------------------------------------------------

std::string scene::node_layers::check_sources() const
{
	std::map<scene::layer_handle, std::string> files;
	std::size_t good{0}, bad{0};
	std::string report;
	for (auto const &[node, source] : m_sources)
	{
		auto &text{files[source.layer]};
		if (text.empty())
		{
			std::ifstream file(path(source.layer), std::ios::binary);
			text.assign(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
		}
		auto const ok{source.span.valid() && static_cast<std::size_t>(source.span.end) <= text.size()};
		std::string piece{ok ? text.substr(source.span.begin, source.span.end - source.span.begin) : std::string{}};
		auto const model{dynamic_cast<TAnimModel const *>(node) != nullptr};
		auto const fine{ok && piece.rfind("node", 0) == 0 && (false == model || (piece.size() >= 8 && piece.compare(piece.size() - 8, 8, "endmodel") == 0))};
		if (fine)
			++good;
		else if (++bad <= 5)
			report += " [" + node->name() + " " + std::to_string(source.span.begin) + "-" + std::to_string(source.span.end) + ": " + piece.substr(0, 60) + " ... " + (piece.size() > 30 ? piece.substr(piece.size() - 30) : std::string{}) + "]"; // NOSONAR
	}
	return std::to_string(good) + " good, " + std::to_string(bad) + " bad" + report;
}
