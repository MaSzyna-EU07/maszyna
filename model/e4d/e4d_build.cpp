/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"

#include "model/e4d/e4d_build.h"

#include <glm/gtc/quaternion.hpp>

#include <limits>
#include <unordered_map>

namespace e4d {

namespace {

constexpr float snorm16_scale{32767.f};
constexpr float minimum_length{1e-12f};

glm::vec3 perpendicular(glm::vec3 const &Normal) {
	// the axis least along the normal gives the steadiest cross product
	auto const absolute{glm::abs(Normal)};
	glm::vec3 axis{1.f, 0.f, 0.f};
	if (absolute.y < absolute.x && absolute.y <= absolute.z)
		axis = {0.f, 1.f, 0.f};
	else if (absolute.z < absolute.x && absolute.z < absolute.y)
		axis = {0.f, 0.f, 1.f};
	return glm::normalize(glm::cross(axis, Normal));
}

std::int16_t pack_snorm16(float const Value) {
	return static_cast<std::int16_t>(std::lround(std::clamp(Value, -1.f, 1.f) * snorm16_scale));
}

glm::mat4 compose(node_record const &Node) {
	glm::quat const rotation{Node.rotation[3], Node.rotation[0], Node.rotation[1], Node.rotation[2]};
	auto transform{glm::mat4_cast(glm::normalize(rotation))};
	transform[0] *= Node.scale[0];
	transform[1] *= Node.scale[1];
	transform[2] *= Node.scale[2];
	transform[3] = glm::vec4{Node.translation[0], Node.translation[1], Node.translation[2], 1.f};
	return transform;
}

std::unordered_map<std::uint32_t, std::size_t> matrix_lookup(model const &Model) {
	std::unordered_map<std::uint32_t, std::size_t> lookup;
	for (std::size_t idx = 0; idx < Model.transforms.size(); ++idx)
		lookup.try_emplace(Model.transforms[idx].node, idx);
	return lookup;
}

glm::mat4 node_transform(model const &Model, std::uint32_t const Node, std::unordered_map<std::uint32_t, std::size_t> const &Matrices) {
	auto const &node{Model.nodes[Node]};
	if ((node.flags & node_flags::matrix) != 0) {
		if (auto const lookup{Matrices.find(Node)}; lookup != Matrices.end()) {
			glm::mat4 matrix;
			std::memcpy(&matrix[0][0], Model.transforms[lookup->second].matrix.data(), sizeof(float) * 16);
			return matrix;
		}
	}
	return compose(node);
}

std::string lowercase(std::string_view const Text) {
	std::string result{Text};
	for (auto &character : result) {
		if (character >= 'A' && character <= 'Z')
			character = static_cast<char>(character - 'A' + 'a');
	}
	return result;
}

struct box {
	glm::vec3 min{std::numeric_limits<float>::max()};
	glm::vec3 max{std::numeric_limits<float>::lowest()};
	bool empty() const {
		return min.x > max.x;
	}
	void add(glm::vec3 const &Point) {
		min = glm::min(min, Point);
		max = glm::max(max, Point);
	}
	glm::vec3 centre() const {
		return empty() ? glm::vec3{0.f} : (min + max) * 0.5f;
	}
};

glm::vec3 to_vec3(position const &Position) {
	return {Position[0], Position[1], Position[2]};
}

// the sphere around the centre of the box of the vertices, reaching the farthest of them
std::array<float, 4> sphere_of(model const &Model, std::size_t const First, std::size_t const Count, box &Box) {
	for (auto idx = First; idx < First + Count; ++idx)
		Box.add(to_vec3(Model.positions[idx]));
	auto const centre{Box.centre()};
	float radius{0.f};
	for (auto idx = First; idx < First + Count; ++idx)
		radius = std::max(radius, glm::length(to_vec3(Model.positions[idx]) - centre));
	return {centre.x, centre.y, centre.z, radius};
}

bounds_record to_bounds(box const &Box, float const Radius) {
	bounds_record bounds;
	if (Box.empty())
		return bounds;
	auto const centre{Box.centre()};
	bounds.min = {Box.min.x, Box.min.y, Box.min.z};
	bounds.max = {Box.max.x, Box.max.y, Box.max.z};
	bounds.sphere = {centre.x, centre.y, centre.z, Radius};
	return bounds;
}

} // namespace

std::array<std::int16_t, 4> encode_qtangent(tangent_frame const &Frame) {
	auto const normal{glm::dot(Frame.normal, Frame.normal) > minimum_length ? glm::normalize(Frame.normal) : glm::vec3{0.f, 0.f, 1.f}};
	auto tangent{Frame.tangent - normal * glm::dot(normal, Frame.tangent)};
	tangent = glm::dot(tangent, tangent) > minimum_length ? glm::normalize(tangent) : perpendicular(normal);
	auto const bitangent{glm::cross(normal, tangent)};
	auto rotation{glm::normalize(glm::quat_cast(glm::mat3{tangent, bitangent, normal}))};
	if (rotation.w < 0.f)
		rotation = -rotation;
	// w keeps its sign through the packing, the sign of the bitangent rides on it
	constexpr float bias{1.f / snorm16_scale};
	if (rotation.w < bias) {
		auto const scale{std::sqrt(1.f - bias * bias) / std::max(glm::length(glm::vec3{rotation.x, rotation.y, rotation.z}), minimum_length)};
		rotation = glm::quat{bias, rotation.x * scale, rotation.y * scale, rotation.z * scale};
	}
	if (Frame.handedness < 0.f)
		rotation = -rotation;
	return {pack_snorm16(rotation.x), pack_snorm16(rotation.y), pack_snorm16(rotation.z), pack_snorm16(rotation.w)};
}

tangent_frame decode_qtangent(std::array<std::int16_t, 4> const &Packed) {
	// the first and the third column of the rotation matrix of the quaternion, which 2 / |q|^2 makes unit without a square root:
	// what glm::mat3_cast(glm::normalize(q)) gives, for the two columns needed, at a part of the cost (this runs for every vertex)
	auto const x{Packed[0] / snorm16_scale};
	auto const y{Packed[1] / snorm16_scale};
	auto const z{Packed[2] / snorm16_scale};
	auto const w{Packed[3] / snorm16_scale};
	tangent_frame frame;
	frame.handedness = w < 0.f ? -1.f : 1.f;
	auto const norm{x * x + y * y + z * z + w * w};
	if (norm < 1e-12f)
		return frame;
	auto const s{2.f / norm};
	frame.tangent = {1.f - s * (y * y + z * z), s * (x * y + w * z), s * (x * z - w * y)};
	frame.normal = {s * (x * z + w * y), s * (y * z - w * x), 1.f - s * (x * x + y * y)};
	return frame;
}

glm::mat4 local_transform(model const &Model, std::uint32_t const Node) {
	return node_transform(Model, Node, matrix_lookup(Model));
}

std::vector<glm::mat4> local_transforms(model const &Model) {
	auto const matrices{matrix_lookup(Model)};
	std::vector<glm::mat4> transforms(Model.nodes.size());
	for (std::uint32_t idx = 0; idx < Model.nodes.size(); ++idx)
		transforms[idx] = node_transform(Model, idx, matrices);
	return transforms;
}

std::vector<glm::mat4> model_transforms(model const &Model) {
	auto const matrices{matrix_lookup(Model)};
	std::vector<glm::mat4> transforms(Model.nodes.size());
	for (std::uint32_t idx = 0; idx < Model.nodes.size(); ++idx) {
		auto const local{node_transform(Model, idx, matrices)};
		auto const parent{Model.nodes[idx].parent};
		transforms[idx] = (parent >= 0 && parent < static_cast<std::int64_t>(idx)) ? transforms[parent] * local : local;
	}
	return transforms;
}

bool set_transform(node_record &Node, glm::mat4 const &Matrix, float const Tolerance) {
	glm::vec3 scale{glm::length(glm::vec3{Matrix[0]}), glm::length(glm::vec3{Matrix[1]}), glm::length(glm::vec3{Matrix[2]})};
	glm::mat3 rotation{1.f};
	if (scale.x > minimum_length && scale.y > minimum_length && scale.z > minimum_length) {
		rotation = glm::mat3{glm::vec3{Matrix[0]} / scale.x, glm::vec3{Matrix[1]} / scale.y, glm::vec3{Matrix[2]} / scale.z};
		// a mirror goes into the scale, the rotation stays proper
		if (glm::determinant(rotation) < 0.f) {
			scale.x = -scale.x;
			rotation[0] = -rotation[0];
		}
	}
	auto const quaternion{glm::normalize(glm::quat_cast(rotation))};
	Node.translation = {Matrix[3].x, Matrix[3].y, Matrix[3].z};
	Node.rotation = {quaternion.x, quaternion.y, quaternion.z, quaternion.w};
	Node.scale = {scale.x, scale.y, scale.z};
	auto const composed{compose(Node)};
	for (int column = 0; column < 4; ++column) {
		for (int row = 0; row < 4; ++row) {
			auto const original{Matrix[column][row]};
			if (std::abs(composed[column][row] - original) > Tolerance * std::max(1.f, std::abs(original)))
				return false;
		}
	}
	return true;
}

void build_name_index(model &Model) {
	Model.name_index.clear();
	for (std::uint32_t idx = 0; idx < Model.nodes.size(); ++idx) {
		if (auto const name{Model.nodes[idx].name}; name != no_index && name < Model.strings.size())
			Model.name_index.push_back({hash_name(Model.strings[name]), idx, 0});
	}
	std::ranges::sort(Model.name_index, [](name_index_record const &Left, name_index_record const &Right) { return Left.hash != Right.hash ? Left.hash < Right.hash : Left.node < Right.node; });
}

std::vector<std::uint32_t> find_nodes(model const &Model, std::string_view const Name) {
	auto const key{lowercase(Name)};
	std::vector<std::uint32_t> found;
	auto const matches{[&Model, &key](std::uint32_t const Node) { return lowercase(Model.string(Model.nodes[Node].name)) == key; }};
	if (Model.name_index.empty()) {
		for (std::uint32_t idx = 0; idx < Model.nodes.size(); ++idx) {
			if (Model.nodes[idx].name != no_index && matches(idx))
				found.push_back(idx);
		}
		return found;
	}
	auto const hash{hash_name(Name)};
	auto const range{std::ranges::equal_range(Model.name_index, hash, {}, &name_index_record::hash)};
	for (auto const &entry : range) {
		if (matches(entry.node))
			found.push_back(entry.node);
	}
	return found;
}

void compute_bounds(model &Model) {
	std::vector<box> boxes(Model.nodes.size());
	std::vector<float> radii(Model.nodes.size(), 0.f);
	for (auto &primitive : Model.primitives) {
		if (primitive.node >= Model.nodes.size() || primitive.vertex_count == 0)
			continue;
		box own;
		primitive.sphere = sphere_of(Model, primitive.base_vertex, primitive.vertex_count, own);
		auto &nodebox{boxes[primitive.node]};
		nodebox.add(own.min);
		nodebox.add(own.max);
	}
	// the radius of the node around the centre of its box, from all its vertices
	for (auto const &primitive : Model.primitives) {
		if (primitive.node >= Model.nodes.size() || primitive.vertex_count == 0)
			continue;
		auto const centre{boxes[primitive.node].centre()};
		auto &radius{radii[primitive.node]};
		for (auto idx = std::size_t{primitive.base_vertex}; idx < std::size_t{primitive.base_vertex} + primitive.vertex_count; ++idx)
			radius = std::max(radius, glm::length(to_vec3(Model.positions[idx]) - centre));
	}
	Model.bounds.clear();
	Model.bounds.reserve(Model.nodes.size() + 1);
	auto const transforms{model_transforms(Model)};
	box whole;
	for (std::size_t idx = 0; idx < Model.nodes.size(); ++idx) {
		Model.bounds.push_back(to_bounds(boxes[idx], radii[idx]));
		if (boxes[idx].empty())
			continue;
		for (int corner = 0; corner < 8; ++corner) {
			glm::vec3 const point{(corner & 1) ? boxes[idx].max.x : boxes[idx].min.x, (corner & 2) ? boxes[idx].max.y : boxes[idx].min.y, (corner & 4) ? boxes[idx].max.z : boxes[idx].min.z};
			whole.add(glm::vec3{transforms[idx] * glm::vec4{point, 1.f}});
		}
	}
	Model.bounds.push_back(to_bounds(whole, whole.empty() ? 0.f : glm::length(whole.max - whole.centre())));
}

} // namespace e4d
