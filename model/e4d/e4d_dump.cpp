/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"

#include "model/e4d/e4d_dump.h"

#include <format>

namespace e4d {

std::string fourcc_name(std::uint32_t const Id) {
	std::string text(4, ' ');
	for (std::size_t idx = 0; idx < 4; ++idx) {
		auto const character{static_cast<char>((Id >> (idx * 8)) & 0xFF)};
		text[idx] = (character >= 32 && character < 127) ? character : '?';
	}
	return text;
}

namespace {

std::string_view topology_name(std::uint32_t const Flags) {
	switch (static_cast<topology>(Flags & primitive_flags::topology_mask)) {
		case topology::points: return "points";
		case topology::lines: return "lines";
		case topology::line_loop: return "line loop";
		case topology::line_strip: return "line strip";
		case topology::triangles: return "triangles";
		case topology::triangle_strip: return "triangle strip";
		case topology::triangle_fan: return "triangle fan";
		default: return "?";
	}
}

std::string quoted(model const &Model, std::uint32_t const Id) {
	return Id == no_index ? std::string{"-"} : std::format("\"{}\"", Model.string(Id));
}

void dump_counts(model const &Model, std::string &Output) {
	Output += std::format("flags: {:#x}, source hash: {:#018x}\n", Model.flags, Model.source_hash);
	Output += std::format("strings: {}, nodes: {} (matrices {}), meshes: {}, primitives: {}, materials: {}\n", Model.strings.size(), Model.nodes.size(), Model.transforms.size(),
	    Model.meshes.size(), Model.primitives.size(), Model.materials.size());
	Output += std::format("vertices: {}, indices: {} x u16 + {} x u32, weights: {} + {} wide, userdata: {}, bindings: {}\n", Model.vertex_count(), Model.indices16.size(),
	    Model.indices32.size(), Model.weights.size(), Model.weights_wide.size(), Model.vertex_userdata.size(), Model.bindings.size());
	Output += std::format("skins: {} ({} joints), vertex groups: {}, components: {} ({} bytes), bounds: {}, name index: {}\n", Model.skins.size(), Model.skin_joints.size(),
	    Model.vertex_groups.size(), Model.components.size(), Model.component_data.size(), Model.bounds.size(), Model.name_index.size());
	for (auto const &section : Model.unknown)
		Output += std::format("unknown section {}: {} bytes\n", fourcc_name(section.id), section.data.size());
}

void dump_primitives(model const &Model, mesh_record const &Mesh, std::string_view const Indent, std::string &Output) {
	for (auto idx = Mesh.primitive_first; idx < Mesh.primitive_first + Mesh.primitive_count && idx < Model.primitives.size(); ++idx) {
		auto const &primitive{Model.primitives[idx]};
		auto const material{primitive.material < Model.materials.size() ? Model.materials[primitive.material] : material_record{}};
		auto const materialname{material.replaceable != 0 ? std::format("skin {}", material.replaceable) : quoted(Model, material.name)};
		Output += std::format("{}  {} {}{}, vertices {}+{}, indices {}+{}, material {}, flags {:#x}\n", Indent, topology_name(primitive.flags),
		    (primitive.flags & primitive_flags::indexed) != 0 ? "indexed" : "plain", (primitive.flags & primitive_flags::index32) != 0 ? " u32" : "", primitive.base_vertex,
		    primitive.vertex_count, primitive.first_index, primitive.index_count, materialname, primitive.flags);
	}
}

void dump_components(model const &Model, std::uint32_t const First, std::uint32_t const Count, std::string_view const Indent, std::string &Output) {
	for (auto idx = First; idx < First + Count && idx < Model.components.size(); ++idx) {
		auto const &component{Model.components[idx]};
		Output += std::format("{}  [{}] {} v{}, {} bytes\n", Indent, idx, fourcc_name(component.type), component.version, component.data_size);
	}
}

void dump_nodes(model const &Model, std::string &Output) {
	std::vector<int> depth(Model.nodes.size(), 0);
	for (std::uint32_t idx = 0; idx < Model.nodes.size(); ++idx) {
		auto const &node{Model.nodes[idx]};
		if (node.parent >= 0 && node.parent < static_cast<std::int64_t>(idx))
			depth[idx] = depth[node.parent] + 1;
		std::string const indent(static_cast<std::size_t>(depth[idx]) * 2, ' ');
		Output += std::format("{}{} {} t({:.4f} {:.4f} {:.4f}) r({:.4f} {:.4f} {:.4f} {:.4f}) s({:.4f} {:.4f} {:.4f}){}{} range {:.1f}-{:.1f} legacy {:#x}\n", indent, idx,
		    quoted(Model, node.name), node.translation[0], node.translation[1], node.translation[2], node.rotation[0], node.rotation[1], node.rotation[2], node.rotation[3],
		    node.scale[0], node.scale[1], node.scale[2], (node.flags & node_flags::matrix) != 0 ? " matrix" : "", (node.flags & node_flags::animated) != 0 ? " animated" : "",
		    std::sqrt(node.range_min_sq), std::sqrt(node.range_max_sq), node.legacy_flags);
		if (node.mesh >= 0 && node.mesh < static_cast<std::int64_t>(Model.meshes.size()))
			dump_primitives(Model, Model.meshes[node.mesh], indent, Output);
		dump_components(Model, node.component_first, node.component_count, indent, Output);
	}
}

void dump_model_components(model const &Model, std::string &Output) {
	for (std::uint32_t idx = 0; idx < Model.components.size(); ++idx) {
		auto const &component{Model.components[idx]};
		if (component.kind == target_kind::model)
			Output += std::format("model component [{}] {} v{}, {} bytes\n", idx, fourcc_name(component.type), component.version, component.data_size);
	}
}

} // namespace

std::string dump(model const &Model) {
	std::string output;
	dump_counts(Model, output);
	dump_nodes(Model, output);
	dump_model_components(Model, output);
	if (false == Model.bounds.empty()) {
		auto const &whole{Model.bounds.back()};
		output += std::format("bounds: ({:.3f} {:.3f} {:.3f}) - ({:.3f} {:.3f} {:.3f}), radius {:.3f}\n", whole.min[0], whole.min[1], whole.min[2], whole.max[0], whole.max[1], whole.max[2],
		    whole.sphere[3]);
	}
	return output;
}

} // namespace e4d
