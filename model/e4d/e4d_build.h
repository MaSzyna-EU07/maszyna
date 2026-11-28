/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// the helpers which make and read the data of an E4D model: the tangent frames, the transforms of the nodes, the index of
// the names and the bounds

#include "model/e4d/e4d_model.h"

#include <glm/glm.hpp>

namespace e4d {

struct tangent_frame {
	glm::vec3 normal{0.f, 0.f, 1.f};
	glm::vec3 tangent{1.f, 0.f, 0.f};
	float handedness{1.f}; // -1 where the bitangent is flipped (mirrored texture)
};

// the snorm16 quaternion of the frame. a degenerate normal or tangent gets an arbitrary perpendicular one
std::array<std::int16_t, 4> encode_qtangent(tangent_frame const &Frame);
tangent_frame decode_qtangent(std::array<std::int16_t, 4> const &Packed);

// the transform of the node within its parent: the matrix of XFRM, or T * R * S
glm::mat4 local_transform(model const &Model, std::uint32_t Node);
// the transforms of all nodes within their parents
std::vector<glm::mat4> local_transforms(model const &Model);
// the transforms of all nodes within the model, in one pass (the parents come first)
std::vector<glm::mat4> model_transforms(model const &Model);

// fills the TRS fields of the node from the matrix. returns false when T * R * S doesn't give the matrix back (shear),
// and the matrix has to be kept in XFRM
bool set_transform(node_record &Node, glm::mat4 const &Matrix, float Tolerance = 1e-5f);

// the sorted index of the names of the nodes
void build_name_index(model &Model);
// the nodes with the name, case-insensitive, in the order of the nodes
std::vector<std::uint32_t> find_nodes(model const &Model, std::string_view Name);

// the bounds of every node (its own geometry, in its space) and of the whole model, and the spheres of the primitives
void compute_bounds(model &Model);

} // namespace e4d
