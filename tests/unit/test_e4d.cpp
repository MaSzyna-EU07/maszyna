#include "stdafx.h"

#include "doctest.h"

#include "model/e4d/e4d_build.h"
#include "model/e4d/e4d_dump.h"
#include "model/e4d/e4d_io.h"

#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <functional>
#include <random>

namespace {

// a small model with something in every section: a root, a body with a quad and a light, a hose skinned to two joints
e4d::model sample_model() {
	e4d::model model;
	auto const node{[&model](std::string_view const Name, std::int32_t const Parent) {
		e4d::node_record record;
		record.name = model.intern(Name);
		record.parent = Parent;
		model.nodes.push_back(record);
		return static_cast<std::uint32_t>(model.nodes.size() - 1);
	}};
	auto const root{node("root", -1)};
	auto const body{node("Body", static_cast<std::int32_t>(root))};
	auto const hose{node("hose", static_cast<std::int32_t>(root))};
	auto const joint0{node("hose_j0", static_cast<std::int32_t>(hose))};
	auto const joint1{node("hose_j1", static_cast<std::int32_t>(hose))};
	for (std::uint32_t idx = 0; idx < model.nodes.size(); ++idx)
		model.nodes[idx].subtree_end = idx + 1;
	model.nodes[root].subtree_end = 5;
	model.nodes[hose].subtree_end = 5;
	model.nodes[body].translation = {1.f, 2.f, 3.f};

	e4d::material_record material;
	material.name = model.intern("pkp/body.mat");
	model.materials.push_back(material);
	e4d::material_record skin;
	skin.replaceable = -1;
	model.materials.push_back(skin);

	// the quad of the body, indexed
	for (auto const &corner : std::array<glm::vec2, 4>{glm::vec2{0.f, 0.f}, {1.f, 0.f}, {1.f, 1.f}, {0.f, 1.f}}) {
		model.positions.push_back({corner.x, corner.y, 0.f});
		e4d::attributes attribute;
		attribute.qtangent = e4d::encode_qtangent({{0.f, 0.f, 1.f}, {1.f, 0.f, 0.f}, 1.f});
		attribute.uv = {corner.x, corner.y};
		model.vertex_attributes.push_back(attribute);
		model.weights.push_back({});
	}
	model.indices16 = {0, 1, 2, 0, 2, 3};
	e4d::primitive_record quad;
	quad.index_count = 6;
	quad.vertex_count = 4;
	quad.node = body;
	quad.material = 0;
	quad.flags = static_cast<std::uint32_t>(e4d::topology::triangles) | e4d::primitive_flags::indexed | e4d::primitive_flags::opaque;
	model.primitives.push_back(quad);
	model.meshes.push_back({0, 1, -1, 0});
	model.nodes[body].mesh = 0;

	// the hose: a line of three vertices, the middle one shared by both joints
	std::array<std::array<std::uint8_t, 4>, 3> const hoseweights{{{255, 0, 0, 0}, {128, 127, 0, 0}, {0, 255, 0, 0}}};
	for (int idx = 0; idx < 3; ++idx) {
		model.positions.push_back({0.f, 0.f, static_cast<float>(idx)});
		model.vertex_attributes.push_back({});
		e4d::skin_weights weights;
		weights.joints = {0, 1, 0, 0};
		weights.weights = hoseweights[idx];
		model.weights.push_back(weights);
	}
	e4d::primitive_record line;
	line.base_vertex = 4;
	line.vertex_count = 3;
	line.index_count = 3;
	line.node = hose;
	line.material = 1;
	line.skin = 0;
	line.flags = static_cast<std::uint32_t>(e4d::topology::line_strip);
	model.primitives.push_back(line);
	model.meshes.push_back({1, 1, 0, 0});
	model.nodes[hose].mesh = 1;
	model.skins.push_back({0, 2, static_cast<std::int32_t>(hose), 0});
	e4d::skin_joint_record first;
	first.node = joint0;
	first.inverse_bind = {1.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f};
	auto second{first};
	second.node = joint1;
	model.skin_joints = {first, second};

	// the components of the body (a socket and a light), then the chain of the hose, then a light slot of the model
	model.nodes[body].component_first = 0;
	e4d::payload::socket socket;
	socket.role = model.intern("hook");
	socket.end = e4d::payload::side::front;
	model.add_component(e4d::component::socket, e4d::target_kind::node, body, e4d::payload_writer{}.put(socket).data());
	auto const light{model.add_component(e4d::component::light, e4d::target_kind::node, body, e4d::payload_writer{}.put(e4d::payload::light{}).data())};
	model.nodes[body].component_count = 2;
	model.nodes[hose].component_first = 2;
	e4d::payload::chain chain;
	chain.kind = e4d::payload::chain_kind::hose_brake;
	chain.joint_count = 2;
	chain.skin = 0;
	chain.end_a = 0;
	model.add_component(e4d::component::chain, e4d::target_kind::node, hose, e4d::payload_writer{}.put(chain).put(joint0).put(joint1).put(0.5f).data());
	model.nodes[hose].component_count = 1;
	e4d::payload::light_slot slot;
	slot.on_count = 1;
	slot.light_count = 1;
	model.add_component(e4d::component::light_slot, e4d::target_kind::model, 0, e4d::payload_writer{}.put(slot).put(body).put(light).data());

	// a vertex group of the hose with metadata, and an unknown section of a later version
	model.vertex_group_items = {4, 2};
	model.vertex_groups.push_back({model.intern("rigid head"), 1, 0, 1, e4d::vertex_group_flags::ranges, 4, 1, 0});
	e4d::payload::property property;
	property.key = model.intern("rigid");
	property.type = e4d::payload::value_type::boolean;
	property.value[0] = 1;
	model.add_component(e4d::component::properties, e4d::target_kind::vertex_group, 0, e4d::payload_writer{}.put(std::uint32_t{1}).put(property).data());
	model.unknown.push_back({e4d::fourcc("ZZZZ"), 0, {std::byte{1}, std::byte{2}, std::byte{3}}});

	e4d::build_name_index(model);
	e4d::compute_bounds(model);
	return model;
}

std::vector<std::byte> sample_file(bool const Compress = true) {
	return e4d::write(sample_model(), {Compress, 9});
}

e4d::read_result read_file(std::vector<std::byte> const &File, e4d::model &Model) {
	return e4d::read(File, Model, {true, true});
}

template <typename Type_> Type_ get_at(std::vector<std::byte> const &File, std::size_t const Offset) {
	Type_ value;
	std::memcpy(&value, File.data() + Offset, sizeof(Type_));
	return value;
}

template <typename Type_> void put_at(std::vector<std::byte> &File, std::size_t const Offset, Type_ const &Value) {
	std::memcpy(File.data() + Offset, &Value, sizeof(Type_));
}

// the entry of the section in the list of the file
std::size_t entry_offset(std::vector<std::byte> const &File, std::uint32_t const Id) {
	auto const header{get_at<e4d::file_header>(File, 0)};
	for (std::size_t idx = 0; idx < header.section_count; ++idx) {
		auto const offset{header.toc_offset + idx * sizeof(e4d::section_entry)};
		if (get_at<e4d::section_entry>(File, offset).id == Id)
			return offset;
	}
	FAIL("no such section");
	return 0;
}

// the hash of the content made right again after a change, so the reader looks at the change itself
void rehash(std::vector<std::byte> &File) {
	auto header{get_at<e4d::file_header>(File, 0)};
	header.content_hash = e4d::hash_bytes(std::span<std::byte const>{File}.subspan(sizeof(e4d::file_header)));
	put_at(File, 0, header);
}

} // namespace

TEST_CASE("e4d: a model written and read back is the same")
{
	auto const original{sample_model()};
	REQUIRE(e4d::validate(original).empty());
	for (auto const compress : {false, true}) {
		CAPTURE(compress);
		auto const file{e4d::write(original, {compress, 9})};
		e4d::model loaded;
		auto const result{read_file(file, loaded)};
		REQUIRE_MESSAGE(result.ok, result.error);
		CHECK(e4d::dump(loaded) == e4d::dump(original));
		CHECK(loaded.strings == original.strings);
		CHECK(loaded.component_data == original.component_data);
		CHECK(loaded.indices16 == original.indices16);
		CHECK(loaded.weights.size() == original.weights.size());
		CHECK(std::memcmp(loaded.nodes.data(), original.nodes.data(), original.nodes.size() * sizeof(e4d::node_record)) == 0);
		REQUIRE(loaded.unknown.size() == 1);
		CHECK(loaded.unknown.front().data == original.unknown.front().data);
	}
}

TEST_CASE("e4d: the sections lie where the format says")
{
	auto const file{sample_file(false)};
	auto const header{get_at<e4d::file_header>(file, 0)};
	CHECK(header.file_size == file.size());
	CHECK(header.toc_offset == sizeof(e4d::file_header));
	for (std::size_t idx = 0; idx < header.section_count; ++idx) {
		auto const entry{get_at<e4d::section_entry>(file, header.toc_offset + idx * sizeof(e4d::section_entry))};
		auto const stream{entry.id == e4d::section::positions || entry.id == e4d::section::attributes || entry.id == e4d::section::indices16};
		CHECK(entry.offset % (stream ? e4d::gpu_alignment : e4d::section_alignment) == 0);
	}
	// the strings of the model
	auto const strings{get_at<e4d::section_entry>(file, entry_offset(file, e4d::section::strings))};
	CHECK((strings.flags & e4d::section_flags::required) != 0);
	CHECK(get_at<std::uint32_t>(file, strings.offset) == sample_model().strings.size());
}

TEST_CASE("e4d: a table with longer records of a later version is read by its known part")
{
	auto const model{sample_model()};
	auto file{e4d::write(model, {false, 0})};
	// the nodes written again with 16 more bytes each, in a new section at the end of the file
	std::vector<std::byte> table;
	auto const header{e4d::table_header{static_cast<std::uint32_t>(model.nodes.size()), static_cast<std::uint32_t>(sizeof(e4d::node_record) + 16)}};
	table.resize(sizeof(header) + header.count * header.stride, std::byte{0x5A});
	std::memcpy(table.data(), &header, sizeof(header));
	for (std::size_t idx = 0; idx < model.nodes.size(); ++idx)
		std::memcpy(table.data() + sizeof(header) + idx * header.stride, &model.nodes[idx], sizeof(e4d::node_record));
	auto const entryoffset{entry_offset(file, e4d::section::nodes)};
	auto entry{get_at<e4d::section_entry>(file, entryoffset)};
	while (file.size() % e4d::section_alignment != 0)
		file.push_back(std::byte{0});
	entry.offset = file.size();
	entry.size = entry.stored_size = table.size();
	file.insert(file.end(), table.begin(), table.end());
	put_at(file, entryoffset, entry);
	auto fileheader{get_at<e4d::file_header>(file, 0)};
	fileheader.file_size = file.size();
	put_at(file, 0, fileheader);
	rehash(file);

	e4d::model loaded;
	auto const result{read_file(file, loaded)};
	REQUIRE_MESSAGE(result.ok, result.error);
	CHECK(std::memcmp(loaded.nodes.data(), model.nodes.data(), model.nodes.size() * sizeof(e4d::node_record)) == 0);
}

TEST_CASE("e4d: broken files are refused with a reason, and leave the model empty")
{
	auto const good{sample_file(false)};
	struct breakage {
		std::string_view what;
		std::function<void(std::vector<std::byte> &)> apply;
		std::string_view reason;
	};
	auto const section{[&good](std::uint32_t const Id) { return get_at<e4d::section_entry>(good, entry_offset(good, Id)); }};
	std::vector<breakage> const breakages{
	    {"cut short", [](std::vector<std::byte> &File) { File.resize(File.size() / 2); }, "cut short"},
	    {"no magic", [](std::vector<std::byte> &File) { File[0] = std::byte{'X'}; }, "isn't an E4D"},
	    {"a later major version", [](std::vector<std::byte> &File) { put_at(File, 4, std::uint16_t{2}); }, "version 2"},
	    {"the content changed", [](std::vector<std::byte> &File) { File.back() ^= std::byte{0xFF}; }, "hash"},
	    {"a section past the end",
	        [&good](std::vector<std::byte> &File) {
		        auto const offset{entry_offset(good, e4d::section::nodes)};
		        auto entry{get_at<e4d::section_entry>(File, offset)};
		        entry.offset = File.size();
		        put_at(File, offset, entry);
		        rehash(File);
	        },
	        "outside the file"},
	    {"a section twice",
	        [&good](std::vector<std::byte> &File) {
		        put_at(File, entry_offset(good, e4d::section::bounds), get_at<e4d::section_entry>(File, entry_offset(good, e4d::section::nodes)));
		        rehash(File);
	        },
	        "twice"},
	    {"an unknown required section",
	        [&good](std::vector<std::byte> &File) {
		        auto const offset{entry_offset(good, e4d::fourcc("ZZZZ"))};
		        auto entry{get_at<e4d::section_entry>(File, offset)};
		        entry.flags |= e4d::section_flags::required;
		        put_at(File, offset, entry);
		        rehash(File);
	        },
	        "doesn't know"},
	    {"an index past the vertices",
	        [&section](std::vector<std::byte> &File) {
		        put_at(File, section(e4d::section::indices16).offset, std::uint16_t{9});
		        rehash(File);
	        },
	        "past its 4 vertices"},
	    {"a parent after its child",
	        [&section](std::vector<std::byte> &File) {
		        put_at(File, section(e4d::section::nodes).offset + sizeof(e4d::table_header) + offsetof(e4d::node_record, parent), std::int32_t{3});
		        rehash(File);
	        },
	        "isn't before it"},
	    {"a joint the skin doesn't have",
	        [&section](std::vector<std::byte> &File) {
		        put_at(File, section(e4d::section::skin_weights).offset + 4 * sizeof(e4d::skin_weights), std::uint8_t{7});
		        rehash(File);
	        },
	        "joint its skin doesn't have"},
	    {"a light slot naming a node which isn't there",
	        [&section](std::vector<std::byte> &File) {
		        // the slot is the last but one component; its node list follows the fixed part
		        auto const table{section(e4d::section::components).offset + sizeof(e4d::table_header)};
		        auto const record{get_at<e4d::component_record>(File, table + 3 * sizeof(e4d::component_record))};
		        REQUIRE(record.type == e4d::component::light_slot);
		        put_at(File, section(e4d::section::component_data).offset + record.data_offset + sizeof(e4d::payload::light_slot), std::uint32_t{99});
		        rehash(File);
	        },
	        "don't hold together"},
	    {"a record too short for its table",
	        [&section](std::vector<std::byte> &File) {
		        put_at(File, section(e4d::section::primitives).offset + 4, std::uint32_t{20});
		        rehash(File);
	        },
	        "records of 20 bytes"},
	};
	for (auto const &breakage : breakages) {
		CAPTURE(breakage.what);
		auto file{good};
		breakage.apply(file);
		auto model{sample_model()};
		auto const result{read_file(file, model)};
		CHECK_FALSE(result.ok);
		CHECK_MESSAGE(result.error.find(breakage.reason) != std::string::npos, result.error);
		CHECK(model.nodes.empty());
	}
}

namespace {

// the sample with one more primitive: two quads of two nodes merged into one draw, the second a lamp of light slot 3
e4d::model merged_model() {
	auto model{sample_model()};
	auto const first{static_cast<std::uint32_t>(model.positions.size())};
	for (std::uint8_t part = 0; part < 2; ++part) {
		for (auto const &corner : std::array<glm::vec2, 4>{glm::vec2{0.f, 0.f}, {1.f, 0.f}, {1.f, 1.f}, {0.f, 1.f}}) {
			model.positions.push_back({corner.x, corner.y, static_cast<float>(part)});
			model.vertex_attributes.push_back({});
			model.weights.push_back({});
		}
	}
	model.bindings.resize(model.positions.size());
	for (std::uint32_t idx = first + 4; idx < first + 8; ++idx)
		model.bindings[idx] = {1, 3, e4d::slot_role::lit, 0};
	for (std::uint16_t part = 0; part < 2; ++part) {
		for (std::uint16_t const corner : {0, 1, 2, 0, 2, 3})
			model.indices16.push_back(static_cast<std::uint16_t>(part * 4 + corner));
	}
	e4d::primitive_record merged;
	merged.first_index = 6; // past the quad of the sample; its hose has no indices
	merged.index_count = 12;
	merged.base_vertex = first;
	merged.vertex_count = 8;
	merged.material = 0;
	merged.node = 0;
	merged.skin = 1;
	merged.flags = static_cast<std::uint32_t>(e4d::topology::triangles) | e4d::primitive_flags::indexed | e4d::primitive_flags::opaque |
	               e4d::primitive_flags::rigid_joints | e4d::primitive_flags::light_slots;
	model.primitives.push_back(merged);
	model.meshes.push_back({2, 1, 1, 0});
	model.nodes[0].mesh = 2;
	// the joints of the merged parts: the body and the hose, each in its place at bind time
	model.skins.push_back({2, 2, 0, 0});
	auto body{model.skin_joints.front()};
	body.node = 1;
	auto hose{body};
	hose.node = 2;
	model.skin_joints.push_back(body);
	model.skin_joints.push_back(hose);
	e4d::compute_bounds(model);
	return model;
}

} // namespace

TEST_CASE("e4d: parts of several nodes and light slots merged into one primitive")
{
	auto const original{merged_model()};
	auto const faults{e4d::validate(original)};
	REQUIRE_MESSAGE(faults.empty(), (faults.empty() ? std::string{} : faults.front()));
	REQUIRE(original.indices16.size() == 18);
	auto const file{e4d::write(original, {false, 0})};
	// the merged primitive draws wrong without VBND, so a reader which doesn't know it refuses the file
	auto const bindings{get_at<e4d::section_entry>(file, entry_offset(file, e4d::section::bindings))};
	CHECK((bindings.flags & e4d::section_flags::required) != 0);
	CHECK(bindings.offset % e4d::gpu_alignment == 0);
	e4d::model loaded;
	auto const result{read_file(file, loaded)};
	REQUIRE_MESSAGE(result.ok, result.error);
	REQUIRE(loaded.bindings.size() == original.bindings.size());
	CHECK(std::memcmp(loaded.bindings.data(), original.bindings.data(), original.bindings.size() * sizeof(e4d::vertex_binding)) == 0);
	CHECK(e4d::dump(loaded) == e4d::dump(original));

	struct breakage {
		std::string_view what;
		std::function<void(e4d::model &)> apply;
		std::string_view reason;
	};
	std::vector<breakage> const breakages{
	    {"a joint the skin doesn't have", [](e4d::model &Model) { Model.bindings.back().joint = 2; }, "joint its skin doesn't have"},
	    {"an unknown role", [](e4d::model &Model) { Model.bindings.back().role = static_cast<e4d::slot_role>(3); }, "unknown light slot role"},
	    {"no skin", [](e4d::model &Model) { Model.primitives.back().skin = -1; }, "no skin naming them"},
	    {"no VBND", [](e4d::model &Model) { Model.bindings.clear(); }, "no VBND"},
	    {"VBND of other length", [](e4d::model &Model) { Model.bindings.pop_back(); }, "VBND has"},
	};
	for (auto const &breakage : breakages) {
		CAPTURE(breakage.what);
		auto model{original};
		breakage.apply(model);
		auto const problems{e4d::validate(model)};
		REQUIRE_FALSE(problems.empty());
		CHECK(std::ranges::any_of(problems, [&breakage](std::string const &Problem) { return Problem.find(breakage.reason) != std::string::npos; }));
	}
}

TEST_CASE("e4d: a file with random bytes changed is read or refused, never more")
{
	for (auto const compress : {false, true}) {
		auto const good{sample_file(compress)};
		std::mt19937 random{compress ? 7u : 11u};
		std::uniform_int_distribution<std::size_t> position{0, good.size() - 1};
		std::uniform_int_distribution<int> value{0, 255};
		for (int attempt = 0; attempt < 3000; ++attempt) {
			auto file{good};
			auto const changes{1 + attempt % 4};
			for (int change = 0; change < changes; ++change)
				file[position(random)] = static_cast<std::byte>(value(random));
			// without the hash, so the changes reach the sections
			e4d::model model;
			auto const result{e4d::read(file, model, {false, true})};
			if (result.ok)
				CHECK(e4d::validate(model).empty());
			else
				CHECK(model.nodes.empty());
		}
	}
}

TEST_CASE("e4d: the tangent frame survives its packing")
{
	std::mt19937 random{3};
	std::uniform_real_distribution<float> coordinate{-1.f, 1.f};
	for (int attempt = 0; attempt < 2000; ++attempt) {
		glm::vec3 normal{coordinate(random), coordinate(random), coordinate(random)};
		glm::vec3 tangent{coordinate(random), coordinate(random), coordinate(random)};
		if (glm::length(normal) < 0.1f || glm::length(glm::cross(normal, tangent)) < 0.1f)
			continue;
		normal = glm::normalize(normal);
		tangent = glm::normalize(tangent - normal * glm::dot(normal, tangent));
		auto const handedness{attempt % 2 == 0 ? 1.f : -1.f};
		auto const frame{e4d::decode_qtangent(e4d::encode_qtangent({normal, tangent, handedness}))};
		CHECK(glm::dot(frame.normal, normal) > 0.99999f);
		CHECK(glm::dot(frame.tangent, tangent) > 0.99999f);
		CHECK(frame.handedness == handedness);
	}
	// the frames where w of the quaternion is near zero (a half turn) keep their handedness too
	for (auto const handedness : {1.f, -1.f}) {
		auto const frame{e4d::decode_qtangent(e4d::encode_qtangent({{0.f, 0.f, -1.f}, {-1.f, 0.f, 0.f}, handedness}))};
		CHECK(frame.handedness == handedness);
		CHECK(glm::dot(frame.normal, glm::vec3{0.f, 0.f, -1.f}) > 0.9999f);
	}
	// a degenerate frame (no normal, a tangent along it) gets a valid one
	auto const frame{e4d::decode_qtangent(e4d::encode_qtangent({{0.f, 0.f, 0.f}, {0.f, 0.f, 1.f}, 1.f}))};
	CHECK(std::abs(glm::dot(frame.normal, frame.tangent)) < 1e-3f);
}

TEST_CASE("e4d: a transform becomes TRS, a sheared one stays a matrix")
{
	e4d::node_record node;
	auto const trs{glm::translate(glm::mat4{1.f}, glm::vec3{1.f, -2.f, 3.f}) * glm::mat4_cast(glm::angleAxis(0.7f, glm::normalize(glm::vec3{1.f, 2.f, 3.f}))) *
	               glm::scale(glm::mat4{1.f}, glm::vec3{2.f, 0.5f, 1.f})};
	CHECK(e4d::set_transform(node, trs));
	CHECK(node.translation[1] == doctest::Approx(-2.f));
	// a mirror is kept as a negative scale
	CHECK(e4d::set_transform(node, glm::scale(glm::mat4{1.f}, glm::vec3{-1.f, 1.f, 1.f})));
	CHECK(node.scale[0] == doctest::Approx(-1.f));
	auto shear{glm::mat4{1.f}};
	shear[1][0] = 0.5f;
	CHECK_FALSE(e4d::set_transform(node, shear));

	// the transforms of the model follow the parents
	auto model{sample_model()};
	auto const transforms{e4d::model_transforms(model)};
	CHECK(transforms[1][3].x == doctest::Approx(1.f));
	CHECK(transforms[3] == glm::mat4{1.f});
}

TEST_CASE("e4d: the nodes are found by name regardless of case")
{
	auto const model{sample_model()};
	auto const found{e4d::find_nodes(model, "BODY")};
	REQUIRE(found.size() == 1);
	CHECK(found.front() == 1);
	CHECK(e4d::find_nodes(model, "hose_J1") == std::vector<std::uint32_t>{4});
	CHECK(e4d::find_nodes(model, "nothing").empty());
	// without the index the same, by a pass over the nodes
	auto unindexed{model};
	unindexed.name_index.clear();
	CHECK(e4d::find_nodes(unindexed, "Hose") == std::vector<std::uint32_t>{2});
}

TEST_CASE("e4d: the bounds hold the geometry of each node and of the whole model")
{
	auto const model{sample_model()};
	REQUIRE(model.bounds.size() == model.nodes.size() + 1);
	auto const &body{model.bounds[1]};
	CHECK(body.min == std::array<float, 3>{0.f, 0.f, 0.f});
	CHECK(body.max == std::array<float, 3>{1.f, 1.f, 0.f});
	CHECK(body.sphere[3] == doctest::Approx(std::sqrt(0.5f)));
	// the quad moved by its node, and the hose along z
	auto const &whole{model.bounds.back()};
	CHECK(whole.min[0] == doctest::Approx(0.f));
	CHECK(whole.max[0] == doctest::Approx(2.f));
	CHECK(whole.max[2] == doctest::Approx(3.f));
	CHECK(model.primitives[1].sphere[2] == doctest::Approx(1.f));
}
