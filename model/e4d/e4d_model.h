/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// an E4D model in memory: the tables of the file, as the writer takes them and the reader gives them back

#include "model/e4d/e4d_format.h"

#include <algorithm>
#include <cstring>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace e4d {

// a section the reader didn't know, kept so the tools write it back unchanged
struct unknown_section {
	std::uint32_t id{0};
	std::uint32_t flags{0};
	std::vector<std::byte> data;
};

struct edit_record {
	std::uint32_t source{no_index};
	std::uint32_t tool{no_index};
	std::array<std::uint32_t, 2> reserved{};
};
static_assert(sizeof(edit_record) == 16);

struct model {
	std::uint32_t flags{0};
	std::uint64_t source_hash{0};

	std::vector<std::string> strings;
	std::vector<name_index_record> name_index;
	std::vector<node_record> nodes;
	std::vector<transform_record> transforms;
	std::vector<mesh_record> meshes;
	std::vector<primitive_record> primitives;
	std::vector<position> positions;
	std::vector<attributes> vertex_attributes;
	std::vector<skin_weights> weights;
	std::vector<skin_weights_wide> weights_wide;
	std::vector<userdata> vertex_userdata;
	std::vector<vertex_binding> bindings;
	std::vector<std::uint16_t> indices16;
	std::vector<std::uint32_t> indices32;
	std::vector<material_record> materials;
	std::vector<skin_record> skins;
	std::vector<skin_joint_record> skin_joints;
	std::vector<vertex_group_record> vertex_groups;
	std::vector<std::uint32_t> vertex_group_items;
	std::vector<std::uint8_t> vertex_group_weights;
	std::vector<component_record> components;
	std::vector<std::byte> component_data;
	std::vector<bounds_record> bounds;
	std::vector<edit_record> edit;
	std::vector<unknown_section> unknown;

	// the id of the string, adding it when it isn't there yet
	std::uint32_t intern(std::string_view Text);
	// the string of the id, empty for no_index or an id out of range
	std::string_view string(std::uint32_t Id) const;
	// adds a component with the given payload and returns its index
	std::uint32_t add_component(std::uint32_t Type, target_kind Kind, std::uint32_t Target, std::span<std::byte const> Data, std::uint16_t Version = 1);
	// the payload of the component
	std::span<std::byte const> component_payload(std::uint32_t Component) const;
	std::size_t vertex_count() const {
		return positions.size();
	}

  private:
	std::unordered_map<std::string, std::uint32_t> m_stringids; // filled by intern() on first use
};

// builds the payload of a component: the fixed part, then the variable lists
class payload_writer {
  public:
	template <typename Type_> payload_writer &put(Type_ const &Value) {
		static_assert(std::is_trivially_copyable_v<Type_>);
		auto const offset{m_data.size()};
		m_data.resize(offset + sizeof(Type_));
		std::memcpy(m_data.data() + offset, &Value, sizeof(Type_));
		return *this;
	}
	template <typename Type_> payload_writer &put_all(std::span<Type_ const> Values) {
		for (auto const &value : Values)
			put(value);
		return *this;
	}
	std::span<std::byte const> data() const {
		return m_data;
	}

  private:
	std::vector<std::byte> m_data;
};

// reads the payload of a component. a read past the end leaves the value as it was and marks the reader failed
class payload_reader {
  public:
	explicit payload_reader(std::span<std::byte const> Data) : m_data(Data) {}
	template <typename Type_> bool get(Type_ &Value) {
		static_assert(std::is_trivially_copyable_v<Type_>);
		if (m_failed || m_data.size() - m_offset < sizeof(Type_)) {
			m_failed = true;
			return false;
		}
		std::memcpy(&Value, m_data.data() + m_offset, sizeof(Type_));
		m_offset += sizeof(Type_);
		return true;
	}
	bool failed() const {
		return m_failed;
	}
	std::size_t remaining() const {
		return m_failed ? 0 : m_data.size() - std::min(m_offset, m_data.size());
	}

  private:
	std::span<std::byte const> m_data;
	std::size_t m_offset{0};
	bool m_failed{false};
};

// FNV-1a 64
std::uint64_t hash_bytes(std::span<std::byte const> Data, std::uint64_t Seed = 0xcbf29ce484222325ull);
// FNV-1a 64 of the name in ASCII lowercase, the key of the name index
std::uint64_t hash_name(std::string_view Name);

} // namespace e4d
