/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"

#include "model/e4d/e4d_convert.h"

#include "model/e4d/e4d_build.h"
#include "model/e4d/e4d_dump.h"
#include "model/e4d/e4d_io.h"
#include "model/Model3d.h"
#include "rendering/renderer.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/utilities.h"
#include "utilities/wholefile.h"

#include <algorithm>
#include <chrono>
#include <format>
#include <iostream>
#include <tuple>

namespace e4d {

namespace {

// the E3D kinds of geometry are the primitive types of OpenGL, which the topologies of E4D take over
constexpr int gl_primitive_last{6};

std::uint32_t pack_colour(glm::vec3 const &Colour) {
	auto const channel{[](float const Value) { return static_cast<std::uint32_t>(std::lround(std::clamp(Value, 0.f, 1.f) * 255.f)); }};
	return channel(Colour.r) | (channel(Colour.g) << 8) | (channel(Colour.b) << 16);
}

std::array<float, 4> to_array(glm::vec4 const &Value) {
	return {Value.x, Value.y, Value.z, Value.w};
}

class converter {
  public:
	converter(TModel3d &Source, std::vector<std::string> &Faults) : m_source(Source), m_faults(Faults) {}
	model run();

  private:
	void siblings(TSubModel *First, std::int32_t Parent);
	void node(TSubModel &Submodel, std::int32_t Parent);
	void transform(TSubModel const &Submodel, std::uint32_t Node);
	void components(TSubModel const &Submodel, std::uint32_t Node);
	void geometry(TSubModel const &Submodel, std::uint32_t Node);
	void vertices(TSubModel const &Submodel, gfx::vertex_array const &Vertices, gfx::userdata_array const &Userdata);
	void indices(TSubModel const &Submodel, gfx::index_array const &Indices, primitive_record &Primitive);
	void stars(gfx::vertex_array const &Vertices, std::uint32_t Node);
	std::uint32_t material(TSubModel const &Submodel);
	static bool has_geometry(TSubModel const &Submodel);
	void light_slots();
	bool any_userdata(TSubModel const *First) const;

	TModel3d &m_source;
	std::vector<std::string> &m_faults;
	model m_model;
	bool m_userdata{false};
	std::vector<std::uint32_t> m_stars; // the nodes whose STAR components wait for their meshes
};

bool converter::any_userdata(TSubModel const *First) const {
	for (auto const *submodel{First}; submodel != nullptr; submodel = submodel->Next) {
		if (submodel->m_geometry.handle != null_handle && false == GfxRenderer->UserData(submodel->m_geometry.handle).empty())
			return true;
		if (any_userdata(submodel->Child))
			return true;
	}
	return false;
}

model converter::run() {
	m_userdata = any_userdata(m_source.GetSMRoot());
	siblings(m_source.GetSMRoot(), -1);
	build_name_index(m_model);
	light_slots();
	compute_bounds(m_model);
	return std::move(m_model);
}

void converter::siblings(TSubModel *const First, std::int32_t const Parent) {
	for (auto *submodel{First}; submodel != nullptr; submodel = submodel->Next)
		node(*submodel, Parent);
}

void converter::node(TSubModel &Submodel, std::int32_t const Parent) {
	auto const index{static_cast<std::uint32_t>(m_model.nodes.size())};
	m_model.nodes.emplace_back();
	auto &record{m_model.nodes.back()};
	record.name = Submodel.pName.empty() ? no_index : m_model.intern(Submodel.pName);
	record.parent = Parent;
	record.legacy_flags = Submodel.iFlags;
	record.range_min_sq = Submodel.fSquareMinDist;
	record.range_max_sq = Submodel.fSquareMaxDist;
	if ((Submodel.iFlags & 0x4000) != 0)
		record.flags |= node_flags::animated;
	if (Submodel.iVisible == 0)
		record.flags |= node_flags::hidden;
	transform(Submodel, index);
	// the components of a node lie together, before those of its children
	m_model.nodes[index].component_first = static_cast<std::uint32_t>(m_model.components.size());
	components(Submodel, index);
	geometry(Submodel, index);
	m_model.nodes[index].component_count = static_cast<std::uint32_t>(m_model.components.size()) - m_model.nodes[index].component_first;
	siblings(Submodel.Child, static_cast<std::int32_t>(index));
	m_model.nodes[index].subtree_end = static_cast<std::uint32_t>(m_model.nodes.size());
}

void converter::transform(TSubModel const &Submodel, std::uint32_t const Node) {
	auto const *matrix{Submodel.GetMatrix()};
	if (matrix == nullptr) {
		m_model.nodes[Node].flags |= node_flags::no_transform;
		return;
	}
	auto const transform{glm::make_mat4(matrix->readArray())};
	if (set_transform(m_model.nodes[Node], transform))
		return;
	m_model.nodes[Node].flags |= node_flags::matrix;
	transform_record record;
	record.node = Node;
	std::memcpy(record.matrix.data(), glm::value_ptr(transform), sizeof(float) * 16);
	m_model.transforms.push_back(record);
}

void converter::components(TSubModel const &Submodel, std::uint32_t const Node) {
	if (Submodel.eType == TP_FREESPOTLIGHT) {
		payload::light light;
		light.kind = Submodel.fCosFalloffAngle <= -0.999f ? payload::light_kind::point : payload::light_kind::spot;
		light.flags = static_cast<std::uint8_t>((Submodel.bShadeSelf ? payload::light_flags::shade_self : 0) | (Submodel.bCastShadows ? 0 : payload::light_flags::no_shadows) |
		                                        (Submodel.bSoftFalloff ? payload::light_flags::soft_falloff : 0) | (Submodel.bLightOnly ? payload::light_flags::light_only : 0) |
		                                        (Submodel.bUseNearAtten ? payload::light_flags::near_attenuation : 0));
		light.color = to_array(Submodel.f4Diffuse);
		light.hotspot_power = Submodel.fHotspotPower;
		light.radius = Submodel.fLightRadius;
		light.near_start = Submodel.fNearAttenStart;
		light.near_end = Submodel.fNearAttenEnd;
		light.far_decay = Submodel.iFarAttenDecay;
		light.far_radius = Submodel.fFarDecayRadius;
		light.cos_falloff = Submodel.fCosFalloffAngle;
		light.cos_hotspot = Submodel.fCosHotspotAngle;
		m_model.add_component(component::light, target_kind::node, Node, payload_writer{}.put(light).data());
	}
	if (Submodel.is_emitter())
		m_model.add_component(component::emitter, target_kind::node, Node, payload_writer{}.put(payload::emitter{}).data());
	if (Submodel.b_Anim != TAnimType::at_None)
		m_model.add_component(component::legacy_animation, target_kind::node, Node, payload_writer{}.put(payload::legacy_animation{static_cast<std::uint32_t>(Submodel.b_Anim)}).data());
	if (Submodel.eType == TP_TEXT)
		m_model.add_component(component::legacy_text, target_kind::node, Node, {});
	if (false == has_geometry(Submodel)) {
		// a light point or a rotator: E3D keeps the material and the colours of every submodel, and the lights use them
		auto const index{material(Submodel)};
		m_model.add_component(component::legacy_material, target_kind::node, Node, payload_writer{}.put(payload::legacy_material{index}).data());
	}
}

bool converter::has_geometry(TSubModel const &Submodel) {
	auto const drawable{(Submodel.eType >= 0 && Submodel.eType <= gl_primitive_last) || Submodel.eType == TP_STARS};
	return drawable && Submodel.m_geometry.handle != null_handle && Submodel.m_geometry.vertex_count > 0 && false == GfxRenderer->Vertices(Submodel.m_geometry.handle).empty();
}

std::uint32_t converter::material(TSubModel const &Submodel) {
	material_record record;
	if (Submodel.m_material < 0)
		record.replaceable = std::max(Submodel.m_material, -4);
	else if (false == Submodel.m_materialname.empty())
		record.name = m_model.intern(Submodel.m_materialname);
	record.selfillum = Submodel.fLight;
	record.ambient = to_array(Submodel.f4Ambient);
	record.diffuse = to_array(Submodel.f4Diffuse);
	record.specular = to_array(Submodel.f4Specular);
	record.emission = to_array(Submodel.f4Emision);
	auto const same{[&record](material_record const &Other) { return std::memcmp(&record, &Other, sizeof(material_record)) == 0; }};
	if (auto const found{std::ranges::find_if(m_model.materials, same)}; found != m_model.materials.end())
		return static_cast<std::uint32_t>(std::distance(m_model.materials.begin(), found));
	m_model.materials.push_back(record);
	return static_cast<std::uint32_t>(m_model.materials.size() - 1);
}

void converter::vertices(TSubModel const &Submodel, gfx::vertex_array const &Vertices, gfx::userdata_array const &Userdata) {
	auto const stars{Submodel.eType == TP_STARS};
	for (std::size_t idx = 0; idx < Vertices.size(); ++idx) {
		auto const &vertex{Vertices[idx]};
		m_model.positions.push_back({vertex.position.x, vertex.position.y, vertex.position.z});
		attributes attribute;
		// the normal of a star is its colour, which goes to the STAR component
		if (false == stars)
			attribute.qtangent = encode_qtangent({vertex.normal, glm::vec3{vertex.tangent}, vertex.tangent.w});
		attribute.uv = {vertex.texture.x, vertex.texture.y};
		m_model.vertex_attributes.push_back(attribute);
		if (m_userdata) {
			auto const data{idx < Userdata.size() ? Userdata[idx].data : glm::vec4{0.f, 0.f, 0.f, 1.f}};
			m_model.vertex_userdata.push_back({data.x, data.y, data.z, data.w});
		}
	}
}

void converter::indices(TSubModel const &Submodel, gfx::index_array const &Indices, primitive_record &Primitive) {
	if (auto const last{std::ranges::max(Indices)}; last >= Primitive.vertex_count)
		m_faults.push_back(std::format("submodel \"{}\" has index {} past its {} vertices", Submodel.pName, last, Primitive.vertex_count));
	Primitive.flags |= primitive_flags::indexed;
	if (Primitive.vertex_count <= 65536) {
		Primitive.first_index = static_cast<std::uint32_t>(m_model.indices16.size());
		for (auto const index : Indices)
			m_model.indices16.push_back(static_cast<std::uint16_t>(index));
	}
	else {
		Primitive.flags |= primitive_flags::index32;
		Primitive.first_index = static_cast<std::uint32_t>(m_model.indices32.size());
		m_model.indices32.insert(m_model.indices32.end(), Indices.begin(), Indices.end());
	}
	Primitive.index_count = static_cast<std::uint32_t>(Indices.size());
}

void converter::stars(gfx::vertex_array const &Vertices, std::uint32_t const Node) {
	payload_writer writer;
	writer.put(payload::legacy_stars{static_cast<std::uint32_t>(Vertices.size())});
	for (auto const &vertex : Vertices)
		writer.put(pack_colour(vertex.normal));
	m_model.add_component(component::legacy_stars, target_kind::node, Node, writer.data());
}

void converter::geometry(TSubModel const &Submodel, std::uint32_t const Node) {
	if (false == has_geometry(Submodel))
		return;
	auto const &vertexdata{GfxRenderer->Vertices(Submodel.m_geometry.handle)};
	primitive_record primitive;
	primitive.base_vertex = static_cast<std::uint32_t>(m_model.positions.size());
	primitive.vertex_count = static_cast<std::uint32_t>(vertexdata.size());
	primitive.node = Node;
	primitive.material = material(Submodel);
	auto const kind{Submodel.eType == TP_STARS ? 0u : static_cast<std::uint32_t>(Submodel.eType)};
	primitive.flags = kind | ((Submodel.iFlags & 0x10) != 0 ? primitive_flags::opaque : 0u) | ((Submodel.iFlags & 0x20) != 0 ? primitive_flags::translucent : 0u);
	vertices(Submodel, vertexdata, GfxRenderer->UserData(Submodel.m_geometry.handle));
	if (auto const &indexdata{GfxRenderer->Indices(Submodel.m_geometry.handle)}; false == indexdata.empty())
		indices(Submodel, indexdata, primitive);
	else
		primitive.index_count = primitive.vertex_count;
	mesh_record mesh;
	mesh.primitive_first = static_cast<std::uint32_t>(m_model.primitives.size());
	mesh.primitive_count = 1;
	if (m_userdata)
		mesh.flags |= mesh_flags::userdata;
	m_model.primitives.push_back(primitive);
	m_model.nodes[Node].mesh = static_cast<std::int32_t>(m_model.meshes.size());
	m_model.meshes.push_back(mesh);
	if (Submodel.eType == TP_STARS)
		stars(vertexdata, Node);
}

void converter::light_slots() {
	// the numbered lights the lights event of the scenery switches (TModel3d::instance_parts)
	for (int slot = 0; slot < 8; ++slot) {
		auto const on{find_nodes(m_model, std::format("Light_On{:02}", slot))};
		auto const off{find_nodes(m_model, std::format("Light_Off{:02}", slot))};
		if (on.empty() && off.empty())
			continue;
		payload::light_slot record;
		record.slot = static_cast<std::uint16_t>(slot);
		// as GetFromName, the first submodel of the name counts
		record.on_count = on.empty() ? 0 : 1;
		record.off_count = off.empty() ? 0 : 1;
		payload_writer writer;
		writer.put(record);
		if (false == on.empty())
			writer.put(on.front());
		if (false == off.empty())
			writer.put(off.front());
		m_model.add_component(component::light_slot, target_kind::model, 0, writer.data());
	}
}

std::string extension_of(std::string const &Path) {
	auto const dot{Path.rfind('.')};
	return dot == std::string::npos ? std::string{} : ToLower(Path.substr(dot));
}

// the model loaded the way the converter and the comparison take it: with the null renderer, which keeps the geometry
// Milliseconds gets the time of reading the file into the submodels, without Init()
bool load_source(std::string const &Input, bool const Dynamic, TModel3d &Source, double *Milliseconds = nullptr) {
	auto const start{std::chrono::steady_clock::now()};
	if (extension_of(Input) == ".e3d")
		Source.LoadFromBinFile(Input, Dynamic);
	else if (extension_of(Input) == ".e4d")
		Source.LoadFromE4D(Input, Dynamic);
	else
		Source.LoadFromTextFile(Input, Dynamic);
	if (Milliseconds != nullptr)
		*Milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
	Source.Init();
	if (Source.GetSMRoot() == nullptr) {
		std::cerr << std::format("can't load model \"{}\"\n", Input);
		return false;
	}
	return true;
}

void start_tools(int const Argc, char *Argv[]) {
	Global.iConvertModels = Argc > 6 ? std::stoi(Argv[6]) : 0;
	Global.iWriteLogEnabled = 2;
	GfxRenderer = gfx_renderer_factory::get_instance()->create("null");
}

// the differences between two loaded models, submodel by submodel in the order of the tree
class model_diff {
  public:
	void compare(TSubModel *Left, TSubModel *Right);
	std::vector<std::string> differences;
	std::size_t submodels{0};
	std::size_t vertices{0};
	float matrix{0.f}; // the largest difference of an element of a transform
	float normal{0.f}; // the largest angle between normals, in degrees
	float tangent{0.f}; // the same for the tangents
	std::size_t bent{0}; // the vertices whose normal turned by more than half a degree

  private:
	void fields(TSubModel const &Left, TSubModel const &Right);
	void light(TSubModel const &Left, TSubModel const &Right);
	void geometry(TSubModel const &Left, TSubModel const &Right);
	void vertex(TSubModel const &Left, gfx::basic_vertex const &A, gfx::basic_vertex const &B, std::size_t Index);
	template <typename Type_> void check(TSubModel const &Submodel, std::string_view What, Type_ const &A, Type_ const &B);
	void report(TSubModel const &Submodel, std::string_view What);
};

void model_diff::report(TSubModel const &Submodel, std::string_view const What) {
	if (differences.size() < 50)
		differences.push_back(std::format("submodel {} \"{}\": {}", submodels, Submodel.pName, What));
}

template <typename Type_> void model_diff::check(TSubModel const &Submodel, std::string_view const What, Type_ const &A, Type_ const &B) {
	if (false == (A == B))
		report(Submodel, What);
}

void model_diff::compare(TSubModel *Left, TSubModel *Right) {
	while (Left != nullptr && Right != nullptr) {
		++submodels;
		fields(*Left, *Right);
		compare(Left->Child, Right->Child);
		if ((Left->Child == nullptr) != (Right->Child == nullptr))
			report(*Left, "children differ");
		Left = Left->Next;
		Right = Right->Next;
	}
	if ((Left == nullptr) != (Right == nullptr))
		differences.emplace_back("the trees differ in the number of submodels");
}

void model_diff::fields(TSubModel const &Left, TSubModel const &Right) {
	check(Left, "name", Left.pName, Right.pName);
	check(Left, "type", Left.eType, Right.eType);
	check(Left, "flags", Left.iFlags, Right.iFlags);
	check(Left, "visibility", Left.iVisible, Right.iVisible);
	check(Left, "animation", Left.b_Anim, Right.b_Anim);
	check(Left, "material name", Left.m_materialname, Right.m_materialname);
	// the null renderer gives each fetch of a material a handle of its own, so a fetched material is told by its name
	if (Left.m_material > 0 && Right.m_material > 0)
		check(Left, "material", GfxRenderer->Material(Left.m_material)->GetName(), GfxRenderer->Material(Right.m_material)->GetName());
	else
		check(Left, "material", Left.m_material, Right.m_material);
	check(Left, "selfillum", Left.fLight, Right.fLight);
	check(Left, "colours", std::tie(Left.f4Ambient, Left.f4Diffuse, Left.f4Specular, Left.f4Emision), std::tie(Right.f4Ambient, Right.f4Diffuse, Right.f4Specular, Right.f4Emision));
	check(Left, "ranges", std::tie(Left.fSquareMinDist, Left.fSquareMaxDist), std::tie(Right.fSquareMinDist, Right.fSquareMaxDist));
	check(Left, "normalisation", Left.m_normalizenormals, Right.m_normalizenormals);
	check(Left, "matrix present", Left.GetMatrix() == nullptr, Right.GetMatrix() == nullptr);
	if (Left.GetMatrix() != nullptr && Right.GetMatrix() != nullptr) {
		for (int idx = 0; idx < 16; ++idx)
			matrix = std::max(matrix, std::abs(Left.GetMatrix()->e[idx] - Right.GetMatrix()->e[idx]));
	}
	if (Left.eType == TP_FREESPOTLIGHT)
		light(Left, Right);
	geometry(Left, Right);
}

void model_diff::light(TSubModel const &Left, TSubModel const &Right) {
	check(Left, "light power and size", std::tie(Left.fHotspotPower, Left.fLightRadius), std::tie(Right.fHotspotPower, Right.fLightRadius));
	check(Left, "light attenuation", std::tie(Left.fNearAttenStart, Left.fNearAttenEnd, Left.bUseNearAtten, Left.iFarAttenDecay, Left.fFarDecayRadius),
	    std::tie(Right.fNearAttenStart, Right.fNearAttenEnd, Right.bUseNearAtten, Right.iFarAttenDecay, Right.fFarDecayRadius));
	check(Left, "light cones", std::tie(Left.fCosFalloffAngle, Left.fCosHotspotAngle), std::tie(Right.fCosFalloffAngle, Right.fCosHotspotAngle));
	check(Left, "light flags", std::tie(Left.bShadeSelf, Left.bCastShadows, Left.bSoftFalloff, Left.bLightOnly),
	    std::tie(Right.bShadeSelf, Right.bCastShadows, Right.bSoftFalloff, Right.bLightOnly));
}

float angle_between(glm::vec3 const &A, glm::vec3 const &B) {
	auto const lengths{glm::length(A) * glm::length(B)};
	if (lengths < 1e-12f)
		return glm::length(A - B) < 1e-6f ? 0.f : 180.f;
	return glm::degrees(std::acos(std::clamp(glm::dot(A, B) / lengths, -1.f, 1.f)));
}

void model_diff::vertex(TSubModel const &Left, gfx::basic_vertex const &A, gfx::basic_vertex const &B, std::size_t const Index) {
	if (A.position != B.position || A.texture != B.texture) {
		report(Left, std::format("vertex {} has another position or uv", Index));
		return;
	}
	if (Left.eType == TP_STARS) {
		// the colour of a star, kept in 8 bits a channel
		if (glm::any(glm::greaterThan(glm::abs(A.normal - B.normal), glm::vec3{0.5f / 255.f + 1e-6f})))
			report(Left, std::format("star {} has another colour", Index));
		return;
	}
	auto const turned{angle_between(A.normal, B.normal)};
	normal = std::max(normal, turned);
	if (turned > 0.5f)
		++bent;
	tangent = std::max(tangent, angle_between(glm::vec3{A.tangent}, glm::vec3{B.tangent}));
}

void model_diff::geometry(TSubModel const &Left, TSubModel const &Right) {
	auto const lefthandle{Left.m_geometry.handle};
	auto const righthandle{Right.m_geometry.handle};
	auto const &leftvertices{lefthandle != null_handle ? GfxRenderer->Vertices(lefthandle) : gfx::vertex_array{}};
	auto const &rightvertices{righthandle != null_handle ? GfxRenderer->Vertices(righthandle) : gfx::vertex_array{}};
	auto const drawable{(Left.eType >= 0 && Left.eType <= gl_primitive_last) || Left.eType == TP_STARS};
	if (false == drawable)
		return;
	if (leftvertices.size() != rightvertices.size()) {
		report(Left, std::format("{} vertices against {}", leftvertices.size(), rightvertices.size()));
		return;
	}
	auto const &leftindices{lefthandle != null_handle ? GfxRenderer->Indices(lefthandle) : gfx::index_array{}};
	auto const &rightindices{righthandle != null_handle ? GfxRenderer->Indices(righthandle) : gfx::index_array{}};
	check(Left, "indices", leftindices, rightindices);
	auto const &leftuser{lefthandle != null_handle ? GfxRenderer->UserData(lefthandle) : gfx::userdata_array{}};
	auto const &rightuser{righthandle != null_handle ? GfxRenderer->UserData(righthandle) : gfx::userdata_array{}};
	auto const sameuser{std::ranges::equal(leftuser, rightuser, [](gfx::vertex_userdata const &A, gfx::vertex_userdata const &B) { return A.data == B.data; })};
	// a model with user data gives the submodels without any of their own the default value
	if (false == sameuser && false == leftuser.empty())
		report(Left, "user data");
	for (std::size_t idx = 0; idx < leftvertices.size(); ++idx)
		vertex(Left, leftvertices[idx], rightvertices[idx], idx);
	vertices += leftvertices.size();
}

int command_diff(int const Argc, char *Argv[]) {
	if (Argc < 5) {
		std::cerr << "usage: -e4d diff <model.e3d|model.t3d> <model.e4d> [dynamic 0|1]\n";
		return 2;
	}
	start_tools(Argc, Argv);
	auto const dynamic{Argc > 5 && std::string_view{Argv[5]} == "1"};
	TModel3d source;
	TModel3d converted;
	double sourcetime{0.0};
	double convertedtime{0.0};
	if (false == load_source(Argv[3], dynamic, source, &sourcetime) || false == load_source(Argv[4], dynamic, converted, &convertedtime))
		return 1;
	model_diff diff;
	diff.compare(source.GetSMRoot(), converted.GetSMRoot());
	for (auto const &difference : diff.differences)
		std::cout << difference << '\n';
	std::cout << std::format("{}: {} submodels, {} vertices, {} differences; transforms within {:.2g}, normals within {:.3f} deg ({} past 0.5 deg), tangents within {:.3f} deg\n", Argv[4],
	    diff.submodels, diff.vertices, diff.differences.size(), diff.matrix, diff.normal, diff.bent, diff.tangent);
	// with the files in the system's cache or not, as they came: a run right after another one of the same files finds them there
	std::cout << std::format("load: source {:.2f} ms, e4d {:.2f} ms\n", sourcetime, convertedtime);
	return diff.differences.empty() ? 0 : 1;
}

int command_convert(int const Argc, char *Argv[]) {
	if (Argc < 5) {
		std::cerr << "usage: -e4d convert <model.e3d|model.t3d> <model.e4d> [dynamic 0|1] [convertmodels flags]\n";
		return 2;
	}
	std::string const input{Argv[3]};
	std::string const output{Argv[4]};
	auto const dynamic{Argc > 5 && std::string_view{Argv[5]} == "1"};
	start_tools(Argc, Argv);
	TModel3d source;
	if (false == load_source(input, dynamic, source))
		return 1;
	std::vector<std::string> faults;
	auto converted{convert(source, faults)};
	for (auto const &fault : faults)
		std::cerr << std::format("{}: {}\n", input, fault);
	auto const file{read_whole_file(input)};
	converted.source_hash = hash_bytes(std::as_bytes(std::span{file.data.get(), file.size}));
	converted.edit.push_back({converted.intern(input), converted.intern("eu07 -e4d convert"), {}});
	if (auto const problems{validate(converted)}; false == problems.empty()) {
		for (auto const &problem : problems)
			std::cerr << std::format("the converted model is invalid: {}\n", problem);
		return 1;
	}
	auto const bytes{write(converted)};
	std::ofstream stream{output, std::ios::binary | std::ios::trunc};
	stream.write(reinterpret_cast<char const *>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
	if (false == stream.good()) {
		std::cerr << std::format("can't write \"{}\"\n", output);
		return 1;
	}
	std::cout << std::format("{} -> {}: {} nodes, {} vertices, {} bytes\n", input, output, converted.nodes.size(), converted.vertex_count(), bytes.size());
	return 0;
}

int command_read(int const Argc, char *Argv[], bool const Dump) {
	if (Argc < 4) {
		std::cerr << std::format("usage: -e4d {} <model.e4d>\n", Dump ? "dump" : "validate");
		return 2;
	}
	auto const file{read_whole_file(Argv[3])};
	if (file.data == nullptr) {
		std::cerr << std::format("can't read \"{}\"\n", Argv[3]);
		return 1;
	}
	model loaded;
	auto const result{read(std::as_bytes(std::span{file.data.get(), file.size}), loaded, {true, true})};
	if (false == result.ok) {
		std::cerr << std::format("{}: {}\n", Argv[3], result.error);
		return 1;
	}
	if (Dump)
		std::cout << dump(loaded);
	else
		std::cout << std::format("{}: valid, {} nodes, {} vertices\n", Argv[3], loaded.nodes.size(), loaded.vertex_count());
	return 0;
}

} // namespace

model convert(TModel3d &Source, std::vector<std::string> &Faults) {
	return converter{Source, Faults}.run();
}

int run_command_line(int const Argc, char *Argv[]) {
	std::string_view const command{Argc > 2 ? Argv[2] : ""};
	if (command == "convert")
		return command_convert(Argc, Argv);
	if (command == "dump")
		return command_read(Argc, Argv, true);
	if (command == "validate")
		return command_read(Argc, Argv, false);
	if (command == "diff")
		return command_diff(Argc, Argv);
	std::cerr << "usage: -e4d convert|dump|validate|diff ...\n";
	return 2;
}

} // namespace e4d
