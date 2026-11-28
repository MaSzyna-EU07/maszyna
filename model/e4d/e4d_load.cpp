/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"

#include "model/Model3d.h"

#include "model/e4d/e4d_build.h"
#include "model/e4d/e4d_io.h"
#include "rendering/renderer.h"
#include "utilities/Logs.h"
#include "utilities/wholefile.h"

#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <atomic>
#include <functional>
#include <optional>
#include <span>
#include <thread>

// the loading of an E4D model into the tree of submodels the rest of the game works with: each node becomes a submodel, with
// what E3D keeps in the submodel taken from the node, its material, its primitive and its components. the result is what
// TModel3d::LoadFromBinFile() makes of the E3D the model was converted from

namespace {

// what E3D keeps in the submodel and E4D in the components of the node
struct legacy_parts {
	std::optional<e4d::payload::light> light;
	std::uint32_t material{e4d::no_index}; // of a node without geometry
	TAnimType animation{TAnimType::at_None};
	bool text{false};
	std::span<std::byte const> stars; // the packed colours, one for each vertex of the mesh
};

legacy_parts parts_of(e4d::model const &Model, e4d::node_record const &Node) {
	legacy_parts parts;
	for (auto idx = Node.component_first; idx < Node.component_first + Node.component_count; ++idx) {
		auto const type{Model.components[idx].type};
		auto const data{Model.component_payload(idx)};
		e4d::payload_reader reader{data};
		if (type == e4d::component::light) {
			if (e4d::payload::light light; reader.get(light))
				parts.light = light;
		}
		else if (type == e4d::component::legacy_animation) {
			if (e4d::payload::legacy_animation animation; reader.get(animation))
				parts.animation = static_cast<TAnimType>(animation.type);
		}
		else if (type == e4d::component::legacy_text)
			parts.text = true;
		else if (type == e4d::component::legacy_material) {
			if (e4d::payload::legacy_material material; reader.get(material))
				parts.material = material.material;
		}
		else if (type == e4d::component::legacy_stars && data.size() >= sizeof(e4d::payload::legacy_stars))
			parts.stars = data.subspan(sizeof(e4d::payload::legacy_stars));
	}
	return parts;
}

glm::vec3 unpack_colour(std::span<std::byte const> const Colours, std::size_t const Index) {
	std::uint32_t packed{0};
	if ((Index + 1) * sizeof(packed) <= Colours.size())
		std::memcpy(&packed, Colours.data() + Index * sizeof(packed), sizeof(packed));
	return glm::vec3{static_cast<float>(packed & 0xFF), static_cast<float>((packed >> 8) & 0xFF), static_cast<float>((packed >> 16) & 0xFF)} / 255.f;
}

glm::vec4 to_vec4(std::array<float, 4> const &Value) {
	return {Value[0], Value[1], Value[2], Value[3]};
}

// the vertices and the indices of a submodel, filled once the submodels are set up
struct geometry_job {
	e4d::primitive_record const *primitive{nullptr};
	std::span<std::byte const> stars;
	TSubModel *submodel{nullptr};
};

// a model with fewer vertices fills its submodels on the thread which loads it: starting threads would cost more
constexpr std::size_t parallel_vertex_threshold{100'000};
constexpr unsigned parallel_thread_limit{8};

// fills the submodels from the nodes of the model
class submodel_builder {
  public:
	submodel_builder(e4d::model const &Model, std::vector<std::string> &Textures, std::vector<std::string> &Names, std::vector<float4x4> &Matrices)
	    : m_model(Model), m_textures(Textures), m_names(Names), m_matrices(Matrices), m_transforms(e4d::local_transforms(Model)) {
		// texture 0 is "none" in the submodels
		m_textures.emplace_back();
	}
	void build(std::uint32_t Index, TSubModel &Submodel);
	// the vertices and the indices of all the submodels, on several threads for a big model
	void fill_geometry();

  private:
	void links(std::uint32_t Index, TSubModel &Submodel) const;
	void transform(std::uint32_t Index, TSubModel &Submodel);
	void material(std::uint32_t Material, TSubModel &Submodel);
	static void light(e4d::payload::light const &Light, TSubModel &Submodel);
	void geometry(e4d::primitive_record const &Primitive, legacy_parts const &Parts, TSubModel &Submodel);
	void fill(geometry_job const &Job) const;
	void indices(e4d::primitive_record const &Primitive, TSubModel &Submodel) const;
	int texture(std::string_view Name);

	e4d::model const &m_model;
	std::vector<std::string> &m_textures;
	std::vector<std::string> &m_names;
	std::vector<float4x4> &m_matrices;
	std::vector<glm::mat4> m_transforms;
	std::vector<geometry_job> m_jobs;
	int m_vertexoffset{0};
	int m_indexoffset{0};

  public:
	int vertex_count{0};
	int index_count{0};
};

void submodel_builder::build(std::uint32_t const Index, TSubModel &Submodel) {
	auto const &node{m_model.nodes[Index]};
	links(Index, Submodel);
	if (node.name != e4d::no_index) {
		Submodel.iName = static_cast<int>(m_names.size());
		m_names.emplace_back(m_model.string(node.name));
	}
	Submodel.iFlags = node.legacy_flags;
	Submodel.fSquareMinDist = node.range_min_sq;
	Submodel.fSquareMaxDist = node.range_max_sq;
	transform(Index, Submodel);
	auto const parts{parts_of(m_model, node)};
	Submodel.b_Anim = parts.animation;
	Submodel.eType = TP_ROTATOR;
	material(parts.material, Submodel);
	if (parts.text)
		Submodel.eType = TP_TEXT;
	if (parts.light) {
		Submodel.eType = TP_FREESPOTLIGHT;
		light(*parts.light, Submodel);
	}
	if (node.mesh >= 0) {
		auto const &mesh{m_model.meshes[node.mesh]};
		if (mesh.primitive_count > 0)
			geometry(m_model.primitives[mesh.primitive_first], parts, Submodel);
	}
	// as in E3D the rotations of the text model were done when it was made
	Submodel.m_rotation_init_done = true;
}

// the submodels lie in the order of the nodes: the first child follows its parent, the next sibling follows the subtree
void submodel_builder::links(std::uint32_t const Index, TSubModel &Submodel) const {
	auto const &node{m_model.nodes[Index]};
	auto const count{static_cast<std::uint32_t>(m_model.nodes.size())};
	Submodel.iChild = (Index + 1 < node.subtree_end) ? static_cast<int>(Index + 1) : 0;
	auto const next{node.subtree_end};
	Submodel.iNext = (next < count && m_model.nodes[next].parent == node.parent) ? static_cast<int>(next) : 0;
}

void submodel_builder::transform(std::uint32_t const Index, TSubModel &Submodel) {
	if ((m_model.nodes[Index].flags & e4d::node_flags::no_transform) != 0) {
		Submodel.iMatrix = -1;
		return;
	}
	Submodel.iMatrix = static_cast<int>(m_matrices.size());
	m_matrices.emplace_back(glm::value_ptr(m_transforms[Index]));
}

int submodel_builder::texture(std::string_view const Name) {
	if (auto const found{std::ranges::find(m_textures, Name)}; found != m_textures.end())
		return static_cast<int>(std::distance(m_textures.begin(), found));
	m_textures.emplace_back(Name);
	return static_cast<int>(m_textures.size() - 1);
}

void submodel_builder::material(std::uint32_t const Material, TSubModel &Submodel) {
	if (Material >= m_model.materials.size())
		return;
	auto const &material{m_model.materials[Material]};
	if (material.replaceable != 0)
		Submodel.iTexture = material.replaceable;
	else if (material.name != e4d::no_index)
		Submodel.iTexture = texture(m_model.string(material.name));
	Submodel.fLight = material.selfillum;
	Submodel.f4Ambient = to_vec4(material.ambient);
	Submodel.f4Diffuse = to_vec4(material.diffuse);
	Submodel.f4Specular = to_vec4(material.specular);
	Submodel.f4Emision = to_vec4(material.emission);
}

void submodel_builder::light(e4d::payload::light const &Light, TSubModel &Submodel) {
	namespace light_flags = e4d::payload::light_flags;
	Submodel.f4Diffuse = to_vec4(Light.color);
	Submodel.fHotspotPower = Light.hotspot_power;
	Submodel.fLightRadius = Light.radius;
	Submodel.fNearAttenStart = Light.near_start;
	Submodel.fNearAttenEnd = Light.near_end;
	Submodel.iFarAttenDecay = Light.far_decay;
	Submodel.fFarDecayRadius = Light.far_radius;
	Submodel.fCosFalloffAngle = Light.cos_falloff;
	Submodel.fCosHotspotAngle = Light.cos_hotspot;
	Submodel.bShadeSelf = (Light.flags & light_flags::shade_self) != 0;
	Submodel.bCastShadows = (Light.flags & light_flags::no_shadows) == 0;
	Submodel.bSoftFalloff = (Light.flags & light_flags::soft_falloff) != 0;
	Submodel.bLightOnly = (Light.flags & light_flags::light_only) != 0;
	Submodel.bUseNearAtten = (Light.flags & light_flags::near_attenuation) != 0;
}

// what the submodel knows of its geometry; the vertices and the indices come later, in fill_geometry()
void submodel_builder::geometry(e4d::primitive_record const &Primitive, legacy_parts const &Parts, TSubModel &Submodel) {
	Submodel.eType = Parts.stars.empty() ? static_cast<int>(Primitive.flags & e4d::primitive_flags::topology_mask) : TP_STARS;
	material(Primitive.material, Submodel);
	auto const indexed{(Primitive.flags & e4d::primitive_flags::indexed) != 0};
	Submodel.m_geometry.vertex_count = static_cast<int>(Primitive.vertex_count);
	Submodel.m_geometry.vertex_offset = m_vertexoffset;
	Submodel.m_geometry.index_count = indexed ? static_cast<int>(Primitive.index_count) : 0;
	Submodel.m_geometry.index_offset = m_indexoffset;
	m_vertexoffset += Submodel.m_geometry.vertex_count;
	m_indexoffset += Submodel.m_geometry.index_count;
	vertex_count += Submodel.m_geometry.vertex_count;
	index_count += Submodel.m_geometry.index_count;
	m_jobs.push_back({&Primitive, Parts.stars, &Submodel});
}

void submodel_builder::fill(geometry_job const &Job) const {
	auto const &primitive{*Job.primitive};
	auto &submodel{*Job.submodel};
	auto const stars{false == Job.stars.empty()};
	auto const userdata{m_model.vertex_userdata.size() == m_model.vertex_count()};
	// each vertex is written once: reserved and appended, without the zeroes resize() would write first
	submodel.Vertices.clear();
	submodel.Vertices.reserve(primitive.vertex_count);
	for (std::uint32_t idx = 0; idx < primitive.vertex_count; ++idx) {
		auto const source{std::size_t{primitive.base_vertex} + idx};
		auto const &position{m_model.positions[source]};
		auto const &attribute{m_model.vertex_attributes[source]};
		auto const frame{e4d::decode_qtangent(attribute.qtangent)};
		gfx::basic_vertex vertex;
		vertex.position = {position[0], position[1], position[2]};
		vertex.normal = stars ? unpack_colour(Job.stars, idx) : frame.normal;
		vertex.tangent = glm::vec4{frame.tangent, frame.handedness};
		vertex.texture = {attribute.uv[0], attribute.uv[1]};
		submodel.Vertices.push_back(vertex);
	}
	if (userdata) {
		submodel.Userdata.clear();
		submodel.Userdata.reserve(primitive.vertex_count);
		for (auto const &data : std::span{m_model.vertex_userdata}.subspan(primitive.base_vertex, primitive.vertex_count))
			submodel.Userdata.push_back({glm::vec4{data[0], data[1], data[2], data[3]}});
	}
	indices(primitive, submodel);
}

void submodel_builder::indices(e4d::primitive_record const &Primitive, TSubModel &Submodel) const {
	if ((Primitive.flags & e4d::primitive_flags::indexed) == 0)
		return;
	if ((Primitive.flags & e4d::primitive_flags::index32) != 0) {
		auto const source{std::span{m_model.indices32}.subspan(Primitive.first_index, Primitive.index_count)};
		Submodel.Indices.assign(source.begin(), source.end());
	}
	else {
		auto const source{std::span{m_model.indices16}.subspan(Primitive.first_index, Primitive.index_count)};
		Submodel.Indices.assign(source.begin(), source.end());
	}
}

void submodel_builder::fill_geometry() {
	auto const threads{static_cast<std::size_t>(vertex_count) < parallel_vertex_threshold ? 1u : std::clamp(std::thread::hardware_concurrency(), 1u, parallel_thread_limit)};
	if (threads <= 1 || m_jobs.size() <= 1) {
		for (auto const &job : m_jobs)
			fill(job);
		return;
	}
	// the biggest first, so the threads end at about the same time
	std::ranges::sort(m_jobs, std::ranges::greater{}, [](geometry_job const &Job) { return Job.primitive->vertex_count; });
	std::atomic<std::size_t> next{0};
	auto const work{[this, &next] {
		while (true) {
			auto const index{next.fetch_add(1)};
			if (index >= m_jobs.size())
				break;
			fill(m_jobs[index]);
		}
	}};
	{
		std::vector<std::jthread> workers;
		workers.reserve(threads - 1);
		for (unsigned idx = 1; idx < threads; ++idx)
			workers.emplace_back(work);
		work();
	} // the workers join here
}

} // namespace

bool TModel3d::LoadFromE4D(std::string const &FileName, bool const dynamic) {
	WriteLog("Loading E4D model data from \"" + FileName + "\"...", logtype::model);
	auto const file{read_whole_file(FileName)};
	if (file.data == nullptr) {
		ErrorLog("Bad model: can't read \"" + FileName + "\"", logtype::model);
		return false;
	}
	e4d::model model;
	// the game has no use for the sections it doesn't know
	if (auto const result{e4d::read(std::as_bytes(std::span{file.data.get(), file.size}), model, {false, false})}; false == result.ok) {
		ErrorLog("Bad model: \"" + FileName + "\" isn't a valid E4D file: " + result.error, logtype::model);
		return false;
	}
	if (model.nodes.empty()) {
		ErrorLog("Bad model: \"" + FileName + "\" has no nodes", logtype::model);
		return false;
	}

	Root = nullptr;
	if (m_geometrybank == null_handle)
		m_geometrybank = GfxRenderer->Create_Bank();
	Textures.clear();
	Names.clear();
	Matrices.clear();
	Matrices.reserve(model.nodes.size());
	iSubModelsCount = static_cast<int>(model.nodes.size());
	Root = new TSubModel[model.nodes.size()];
	submodel_builder builder{model, Textures, Names, Matrices};
	for (std::uint32_t idx = 0; idx < model.nodes.size(); ++idx)
		builder.build(idx, Root[idx]);
	builder.fill_geometry();
	m_vertexcount += builder.vertex_count;
	m_indexcount += builder.index_count;
	// the tangents come with the vertices
	finish_binary(dynamic, true);
	WriteLog("Finished loading E4D model data from \"" + FileName + "\"", logtype::model);
	return true;
}
