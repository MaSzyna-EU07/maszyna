/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// the records of the E4D model format, as they lie in the file. docs/e4d-format.md describes them; the sizes and offsets
// below are the format, and the static_asserts keep the two the same

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace e4d {

static_assert(std::endian::native == std::endian::little, "the E4D records are read and uploaded to the gpu as they lie in the file, little-endian");

constexpr std::uint32_t fourcc(std::string_view const Code) {
	return static_cast<std::uint32_t>(static_cast<unsigned char>(Code[0])) | (static_cast<std::uint32_t>(static_cast<unsigned char>(Code[1])) << 8) |
	       (static_cast<std::uint32_t>(static_cast<unsigned char>(Code[2])) << 16) | (static_cast<std::uint32_t>(static_cast<unsigned char>(Code[3])) << 24);
}

constexpr std::uint16_t version_major{1};
constexpr std::uint16_t version_minor{0};
constexpr std::uint32_t no_index{0xFFFFFFFFu}; // no string, node, component and the like
constexpr std::size_t gpu_alignment{64}; // the vertex and index streams
constexpr std::size_t section_alignment{16}; // every other section

// the sections
namespace section {
constexpr std::uint32_t strings{fourcc("STRS")};
constexpr std::uint32_t name_index{fourcc("NIDX")};
constexpr std::uint32_t nodes{fourcc("NODE")};
constexpr std::uint32_t transforms{fourcc("XFRM")};
constexpr std::uint32_t meshes{fourcc("MESH")};
constexpr std::uint32_t primitives{fourcc("PRIM")};
constexpr std::uint32_t positions{fourcc("VPOS")};
constexpr std::uint32_t attributes{fourcc("VATT")};
constexpr std::uint32_t skin_weights{fourcc("VSKN")};
constexpr std::uint32_t skin_weights_wide{fourcc("VSKW")};
constexpr std::uint32_t userdata{fourcc("VUSR")};
constexpr std::uint32_t bindings{fourcc("VBND")};
constexpr std::uint32_t indices16{fourcc("IDX2")};
constexpr std::uint32_t indices32{fourcc("IDX4")};
constexpr std::uint32_t materials{fourcc("MATL")};
constexpr std::uint32_t skins{fourcc("SKIN")};
constexpr std::uint32_t skin_joints{fourcc("SKJT")};
constexpr std::uint32_t vertex_groups{fourcc("VGRP")};
constexpr std::uint32_t vertex_group_items{fourcc("VGIT")};
constexpr std::uint32_t vertex_group_weights{fourcc("VGWT")};
constexpr std::uint32_t components{fourcc("CMPT")};
constexpr std::uint32_t component_data{fourcc("CDAT")};
constexpr std::uint32_t bounds{fourcc("BNDS")};
constexpr std::uint32_t edit{fourcc("EDIT")};
} // namespace section

// the components
namespace component {
constexpr std::uint32_t properties{fourcc("PROP")};
constexpr std::uint32_t socket{fourcc("SOCK")};
constexpr std::uint32_t joint{fourcc("JOIN")};
constexpr std::uint32_t spring{fourcc("SPRG")};
constexpr std::uint32_t chain{fourcc("CHAN")};
constexpr std::uint32_t variants{fourcc("VARI")};
constexpr std::uint32_t channel{fourcc("CHNL")};
constexpr std::uint32_t light_slot{fourcc("LSLT")};
constexpr std::uint32_t light{fourcc("LGHT")};
constexpr std::uint32_t emitter{fourcc("EMIT")};
constexpr std::uint32_t model_reference{fourcc("MREF")};
constexpr std::uint32_t collision{fourcc("COLL")};
constexpr std::uint32_t legacy_animation{fourcc("LANM")};
constexpr std::uint32_t legacy_text{fourcc("LTXT")};
constexpr std::uint32_t legacy_stars{fourcc("STAR")};
} // namespace component

struct file_header {
	std::array<char, 4> magic{'E', '4', 'D', '\0'};
	std::uint16_t major{version_major};
	std::uint16_t minor{version_minor};
	std::uint32_t flags{0};
	std::uint32_t section_count{0};
	std::uint64_t toc_offset{0};
	std::uint64_t file_size{0};
	std::uint64_t content_hash{0}; // FNV-1a 64 of the bytes after the header
	std::uint64_t source_hash{0}; // of the file the model was made from, so the tools know when to make it again
	std::array<std::uint8_t, 16> reserved{};
};
static_assert(sizeof(file_header) == 64);
static_assert(offsetof(file_header, toc_offset) == 16);

namespace file_flags {
constexpr std::uint32_t optimized{0x1}; // the index order was optimised; a file straight from the exporter has it clear
}

struct section_entry {
	std::uint32_t id{0};
	std::uint32_t flags{0};
	std::uint64_t offset{0};
	std::uint64_t stored_size{0}; // in the file
	std::uint64_t size{0}; // unpacked
};
static_assert(sizeof(section_entry) == 32);

namespace section_flags {
constexpr std::uint32_t required{0x1}; // a reader which doesn't know the section refuses the file
constexpr std::uint32_t zstd{0x2};
}

// the head of a section holding a table of records. the stride lets a later version add fields at the end of a record
struct table_header {
	std::uint32_t count{0};
	std::uint32_t stride{0};
};
static_assert(sizeof(table_header) == 8);

struct name_index_record {
	std::uint64_t hash{0}; // FNV-1a 64 of the name in ASCII lowercase
	std::uint32_t node{0};
	std::uint32_t reserved{0};
};
static_assert(sizeof(name_index_record) == 16);

namespace node_flags {
constexpr std::uint32_t animated{0x1}; // moved at run time, keeps its own transform
constexpr std::uint32_t hidden{0x2}; // not drawn until something shows it
constexpr std::uint32_t matrix{0x4}; // the transform is the matrix in XFRM, the TRS fields are only an approximation
}

struct node_record {
	std::uint32_t name{no_index};
	std::int32_t parent{-1};
	std::uint32_t subtree_end{0}; // one past the last node of the subtree (nodes are in depth-first order)
	std::uint32_t flags{0};
	std::array<float, 3> translation{0.f, 0.f, 0.f};
	std::array<float, 4> rotation{0.f, 0.f, 0.f, 1.f}; // quaternion x, y, z, w
	std::array<float, 3> scale{1.f, 1.f, 1.f};
	std::int32_t mesh{-1};
	std::uint32_t component_first{0};
	std::uint32_t component_count{0};
	float range_min_sq{0.f}; // squared distance from the camera from which it is drawn
	float range_max_sq{0.f}; // squared distance up to which it is drawn; 0 means no limit
	std::uint32_t legacy_flags{0}; // iFlags of the E3D submodel it came from, 0 for a model made anew
};
static_assert(sizeof(node_record) == 80);
static_assert(offsetof(node_record, translation) == 16);
static_assert(offsetof(node_record, mesh) == 56);

struct transform_record {
	std::uint32_t node{0};
	std::array<std::uint32_t, 3> reserved{};
	std::array<float, 16> matrix{}; // column-major
};
static_assert(sizeof(transform_record) == 80);

namespace mesh_flags {
constexpr std::uint32_t userdata{0x1}; // the model of E3D had user data for the vertices of this mesh (or of another)
}

struct mesh_record {
	std::uint32_t primitive_first{0};
	std::uint32_t primitive_count{0};
	std::int32_t skin{-1};
	std::uint32_t flags{0};
};
static_assert(sizeof(mesh_record) == 16);

enum class topology : std::uint8_t {
	points = 0,
	lines = 1,
	line_loop = 2,
	line_strip = 3,
	triangles = 4,
	triangle_strip = 5,
	triangle_fan = 6
};

namespace primitive_flags {
constexpr std::uint32_t topology_mask{0xF};
constexpr std::uint32_t indexed{0x10};
constexpr std::uint32_t index32{0x20}; // the indices are in IDX4, else in IDX2
constexpr std::uint32_t opaque{0x40}; // drawn in the opaque pass
constexpr std::uint32_t translucent{0x80}; // drawn in the translucent pass
constexpr std::uint32_t no_shadow{0x100};
// the vertices of the primitive follow the joints named in VBND, each with a weight of 1: parts of several nodes merged
// into one draw. the joints are those of the skin of the primitive; the vertex shader takes the matrix, without skinning
constexpr std::uint32_t rigid_joints{0x200};
// the vertices of the primitive are lit, dimmed or hidden by the light slots named in VBND
constexpr std::uint32_t light_slots{0x400};
constexpr std::uint32_t lod_shift{16}; // bits 16-23: level of detail, 0 is the most detailed
constexpr std::uint32_t lod_mask{0xFF0000};
} // namespace primitive_flags

struct primitive_record {
	std::uint32_t first_index{0}; // in IDX2 or IDX4, in elements; for a primitive without indices, the first vertex past base_vertex
	std::uint32_t index_count{0}; // for a primitive without indices, the count of vertices drawn
	std::uint32_t base_vertex{0}; // into the vertex streams
	std::uint32_t vertex_count{0};
	std::uint32_t material{no_index};
	std::uint32_t node{0};
	std::int32_t skin{-1};
	std::uint32_t flags{0};
	std::array<float, 4> sphere{0.f, 0.f, 0.f, 0.f}; // centre and radius, in the space of the node
};
static_assert(sizeof(primitive_record) == 48);

// the vertex streams. every stream present has one element per vertex of the file
using position = std::array<float, 3>;
static_assert(sizeof(position) == 12);

struct attributes {
	std::array<std::int16_t, 4> qtangent{0, 0, 0, 32767}; // snorm16 quaternion of the tangent frame; w < 0 flips the bitangent
	std::array<float, 2> uv{0.f, 0.f};
};
static_assert(sizeof(attributes) == 16);

struct skin_weights {
	std::array<std::uint8_t, 4> joints{}; // into the joints of the skin of the mesh
	std::array<std::uint8_t, 4> weights{}; // unorm8, summing to 255
};
static_assert(sizeof(skin_weights) == 8);

struct skin_weights_wide {
	std::array<std::uint16_t, 4> joints{};
	std::array<std::uint16_t, 4> weights{}; // unorm16, summing to 65535
};
static_assert(sizeof(skin_weights_wide) == 16);

using userdata = std::array<float, 4>;
static_assert(sizeof(userdata) == 16);

// what a vertex of a merged primitive belongs to
enum class slot_role : std::uint8_t {
	none = 0, // not tied to a light slot
	lit = 1, // shown, and glowing, while the slot is lit (the Light_On## of E3D)
	dark = 2 // shown while the slot is dark (the Light_Off## of E3D)
};

struct vertex_binding {
	std::uint8_t joint{0}; // with primitive_flags::rigid_joints: into the joints of the skin of the primitive
	std::uint8_t slot{0}; // with primitive_flags::light_slots: the light slot (the slot of an LSLT)
	slot_role role{slot_role::none};
	std::uint8_t reserved{0}; // 0; a later version may use it, a reader ignores it
};
static_assert(sizeof(vertex_binding) == 4);

namespace material_flags {
constexpr std::uint32_t translucent{0x1}; // the replaceable skin decides the pass (bits 0-3 of the E3D flags)
}

struct material_record {
	std::uint32_t name{no_index}; // the .mat file or texture; no_index draws in the colours below only
	std::int32_t replaceable{0}; // 0, or the replaceable skin -1..-4 used instead of the name
	std::uint32_t flags{0};
	float selfillum{-1.f}; // the light level below which the material glows (fLight of E3D)
	std::array<float, 4> ambient{1.f, 1.f, 1.f, 1.f};
	std::array<float, 4> diffuse{1.f, 1.f, 1.f, 1.f};
	std::array<float, 4> specular{0.f, 0.f, 0.f, 1.f};
	std::array<float, 4> emission{1.f, 1.f, 1.f, 1.f};
};
static_assert(sizeof(material_record) == 80);

struct skin_record {
	std::uint32_t joint_first{0}; // in SKJT
	std::uint32_t joint_count{0};
	std::int32_t root{-1}; // the node the skin hangs from
	std::uint32_t reserved{0};
};
static_assert(sizeof(skin_record) == 16);

struct skin_joint_record {
	std::uint32_t node{0};
	std::array<std::uint32_t, 3> reserved{};
	std::array<float, 12> inverse_bind{}; // 3x4, column-major: four columns of three
};
static_assert(sizeof(skin_joint_record) == 64);

namespace vertex_group_flags {
constexpr std::uint32_t ranges{0x1}; // the items are pairs (first vertex, count), else single vertices
constexpr std::uint32_t weighted{0x2}; // VGWT holds an unorm8 weight for each item
} // namespace vertex_group_flags

struct vertex_group_record {
	std::uint32_t name{no_index};
	std::uint32_t mesh{0};
	std::uint32_t item_first{0}; // in VGIT (and VGWT)
	std::uint32_t item_count{0};
	std::uint32_t flags{0};
	std::uint32_t component_first{0};
	std::uint32_t component_count{0};
	std::uint32_t reserved{0};
};
static_assert(sizeof(vertex_group_record) == 32);

enum class target_kind : std::uint8_t {
	model = 0,
	node = 1,
	mesh = 2,
	vertex_group = 3
};

struct component_record {
	std::uint32_t type{0};
	std::uint16_t version{1};
	target_kind kind{target_kind::node};
	std::uint8_t flags{0};
	std::uint32_t target{0};
	std::uint32_t data_offset{0}; // in CDAT, 4-byte aligned
	std::uint32_t data_size{0};
};
static_assert(sizeof(component_record) == 20);

struct bounds_record {
	std::array<float, 3> min{0.f, 0.f, 0.f};
	std::array<float, 3> max{0.f, 0.f, 0.f};
	std::array<float, 4> sphere{0.f, 0.f, 0.f, 0.f};
};
static_assert(sizeof(bounds_record) == 40);

// the payloads of the components, version 1. a payload may be longer than the struct, the rest is read by the
// variable-length parts described with each type
namespace payload {

enum class value_type : std::uint8_t {
	boolean = 0,
	integer = 1,
	real = 2,
	vec2 = 3,
	vec3 = 4,
	vec4 = 5,
	string = 6, // string id in value[0]
	node = 7, // node index in value[0]
	component = 8 // component index in value[0]
};

struct property {
	std::uint32_t key{no_index};
	value_type type{value_type::real};
	std::array<std::uint8_t, 3> reserved{};
	std::array<std::uint32_t, 4> value{}; // the bits of the value; a real is a float, an integer an int32
};
static_assert(sizeof(property) == 24);
// PROP: std::uint32_t count, then count property records

enum class side : std::uint8_t {
	none = 0,
	front = 1,
	rear = 2
};

struct socket {
	std::uint32_t role{no_index};
	side end{side::none};
	std::uint8_t reserved{0};
	std::uint16_t index{0};
	std::array<float, 3> position{0.f, 0.f, 0.f}; // in the space of the node
	std::array<float, 4> rotation{0.f, 0.f, 0.f, 1.f};
};
static_assert(sizeof(socket) == 36);

enum class joint_kind : std::uint8_t {
	fixed = 0,
	hinge = 1,
	ball = 2,
	slider = 3,
	screw = 4
};

struct joint {
	joint_kind kind{joint_kind::hinge};
	std::uint8_t limited{0};
	std::uint16_t reserved{0};
	std::uint32_t role{no_index};
	std::array<float, 3> axis{1.f, 0.f, 0.f}; // in the space of the node
	float minimum{0.f}; // radians or metres
	float maximum{0.f};
	float rest{0.f};
	float pitch{0.f}; // metres per turn of a screw
};
static_assert(sizeof(joint) == 36);

enum class spring_mode : std::uint8_t {
	follow = 0,
	stretch = 1,
	aim = 2,
	pendulum = 3
};

struct spring {
	spring_mode mode{spring_mode::follow};
	std::uint8_t sideways{0}; // a pendulum may swing sideways too
	std::uint16_t reserved{0};
	std::uint32_t anchor_a{no_index}; // node
	std::uint32_t socket_a{no_index}; // component, or no_index for the origin of the node
	std::uint32_t anchor_b{no_index};
	std::uint32_t socket_b{no_index};
	std::array<float, 3> axis{0.f, 1.f, 0.f};
	float rest_length{0.f};
	float travel_min{0.f};
	float travel_max{0.f};
	float pendulum_length{0.f};
	float pendulum_damping{0.f};
};
static_assert(sizeof(spring) == 52);

enum class chain_kind : std::uint8_t {
	hose_brake = 0,
	hose_main = 1,
	cable = 2,
	bellows = 3,
	other = 4
};

struct chain {
	chain_kind kind{chain_kind::other};
	std::uint8_t reserved{0};
	std::uint16_t joint_count{0};
	std::int32_t skin{-1};
	float min_bend_radius{0.f};
	float stiffness{0.f};
	float head_mass{0.f};
	float head_length{0.f};
	std::uint32_t end_a{no_index}; // socket components
	std::uint32_t end_b{no_index};
};
static_assert(sizeof(chain) == 32);
// CHAN: the chain, then joint_count node indices (std::uint32_t), then joint_count - 1 rest lengths of the segments (float)

struct variants {
	std::uint32_t name{no_index};
	std::uint16_t state_count{0};
	std::uint16_t default_state{0};
};
static_assert(sizeof(variants) == 8);
struct variant_state {
	std::uint32_t name{no_index};
	std::uint32_t node_first{0}; // in the node list after the states
	std::uint32_t node_count{0};
};
static_assert(sizeof(variant_state) == 12);
// VARI: the variants, then state_count variant_state records, then the node list (std::uint32_t)

enum class binding_kind : std::uint8_t {
	joint = 0,
	clip = 1,
	variants = 2,
	light_slot = 3
};

struct channel {
	std::uint32_t name{no_index};
	float minimum{0.f};
	float maximum{1.f};
	float initial{0.f};
	std::uint16_t binding_count{0};
	std::uint16_t reserved{0};
};
static_assert(sizeof(channel) == 20);
struct channel_binding {
	binding_kind kind{binding_kind::joint};
	std::array<std::uint8_t, 3> reserved{};
	std::uint32_t target{no_index}; // component
	float scale{1.f}; // target value = channel value * scale + offset
	float offset{0.f};
};
static_assert(sizeof(channel_binding) == 16);
// CHNL: the channel, then binding_count channel_binding records

struct light_slot {
	std::uint16_t slot{0};
	std::uint16_t reserved{0};
	float initial{0.f}; // the state, coded as the values of the lights event
	std::array<float, 3> color{-1.f, -1.f, -1.f}; // negative: the colour of the light sources
	std::uint16_t on_count{0};
	std::uint16_t off_count{0};
	std::uint16_t light_count{0};
	std::uint16_t reserved2{0};
};
static_assert(sizeof(light_slot) == 28);
// LSLT: the slot, then on_count nodes shown when lit, off_count nodes shown when dark, light_count LGHT components (all std::uint32_t)

enum class light_kind : std::uint8_t {
	point = 0,
	spot = 1
};

namespace light_flags {
constexpr std::uint8_t shade_self{0x1};
constexpr std::uint8_t no_shadows{0x2};
constexpr std::uint8_t soft_falloff{0x4};
constexpr std::uint8_t light_only{0x8};
constexpr std::uint8_t near_attenuation{0x10};
} // namespace light_flags

struct light {
	light_kind kind{light_kind::spot};
	std::uint8_t flags{0};
	std::uint16_t reserved{0};
	std::array<float, 4> color{1.f, 1.f, 1.f, 1.f};
	float hotspot_power{1.f};
	float radius{0.f};
	float near_start{40.f};
	float near_end{80.f};
	std::int32_t far_decay{0};
	float far_radius{100.f};
	float cos_falloff{0.5f};
	float cos_hotspot{0.3f};
};
static_assert(sizeof(light) == 52);

enum class emitter_kind : std::uint8_t {
	smoke = 0,
	steam = 1,
	sparks = 2
};

struct emitter {
	emitter_kind kind{emitter_kind::smoke};
	std::array<std::uint8_t, 3> reserved{};
	std::array<float, 3> direction{0.f, 1.f, 0.f};
};
static_assert(sizeof(emitter) == 16);

struct model_reference {
	std::uint32_t path{no_index};
	std::uint32_t socket{no_index}; // component; no_index for the origin of the node
	std::uint32_t skin{no_index}; // string, the skin of the model put in
};
static_assert(sizeof(model_reference) == 12);

enum class shape : std::uint8_t {
	box = 0,
	sphere = 1,
	capsule = 2
};

struct collision {
	shape kind{shape::box};
	std::array<std::uint8_t, 3> reserved{};
	std::uint32_t role{no_index};
	std::array<float, 3> centre{0.f, 0.f, 0.f};
	std::array<float, 4> rotation{0.f, 0.f, 0.f, 1.f};
	std::array<float, 3> size{0.f, 0.f, 0.f}; // half extents; a sphere uses x, a capsule x (radius) and y (half length)
};
static_assert(sizeof(collision) == 48);

struct legacy_animation {
	std::uint32_t type{0}; // the TAnimType of E3D
};
static_assert(sizeof(legacy_animation) == 4);

struct legacy_stars {
	std::uint32_t count{0};
};
static_assert(sizeof(legacy_stars) == 4);
// STAR: the count, then count packed RGB colours (std::uint32_t), one for each vertex of the mesh of the node

} // namespace payload

} // namespace e4d
