/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

namespace scene::eu7 {

struct Eu7Vec4 {
    float x = 0.f;
    float y = 0.f;
    float z = 0.f;
    float w = 1.f;
};

struct Eu7WorldVertex {
    glm::dvec3 position{};
    glm::vec3 normal{};
    double u = 0.0;
    double v = 0.0;
};

struct Eu7LightingData {
    Eu7Vec4 diffuse{ 0.8f, 0.8f, 0.8f, 1.f };
    Eu7Vec4 ambient{ 0.2f, 0.2f, 0.2f, 1.f };
    Eu7Vec4 specular{ 0.f, 0.f, 0.f, 1.f };
};

struct Eu7BoundingArea {
    glm::dvec3 center{};
    float radius = -1.f;
};

struct Eu7TransformContext {
    std::vector<glm::dvec3> origin_stack;
    std::vector<glm::dvec3> scale_stack;
    glm::dvec3 rotation{};
    std::size_t group_depth = 0;
};

struct Eu7BasicNode {
    Eu7BoundingArea area;
    double range_squared_min = 0.0;
    double range_squared_max = std::numeric_limits<double>::max();
    bool visible = true;
    std::string name;
    std::string node_type;
    std::size_t group_handle = 0;
    bool group_valid = false;
    Eu7TransformContext transform;
};

struct Eu7SegmentPath {
    glm::dvec3 p_start{};
    double roll_start = 0.0;
    glm::dvec3 cp_out{};
    glm::dvec3 cp_in{};
    glm::dvec3 p_end{};
    double roll_end = 0.0;
    double radius = 0.0;
};

enum class Eu7TrackType : std::uint8_t {
    Unknown,
    Normal,
    Switch,
    Table,
    Cross,
    Tributary,
};

enum class Eu7TrackCategory : std::uint8_t {
    Rail = 1,
    Road = 2,
    Water = 4,
};

enum class Eu7TrackEnvironment : std::int8_t {
    Unknown = -1,
    Flat = 0,
    Mountains,
    Canyon,
    Tunnel,
    Bridge,
    Bank,
};

struct Eu7TrackVisibility {
    std::string material1;
    float tex_length = 4.f;
    std::string material2;
    float tex_height1 = 0.f;
    float tex_width = 0.f;
    float tex_slope = 0.f;
};

struct Eu7Track {
    Eu7BasicNode node;
    Eu7TrackType track_type = Eu7TrackType::Unknown;
    Eu7TrackCategory category = Eu7TrackCategory::Rail;
    float length = 0.f;
    float track_width = 0.f;
    float friction = 0.f;
    float sound_distance = 0.f;
    int quality_flag = 0;
    int damage_flag = 0;
    Eu7TrackEnvironment environment = Eu7TrackEnvironment::Unknown;
    std::optional<Eu7TrackVisibility> visibility;
    std::vector<Eu7SegmentPath> paths;
    std::vector<std::pair<std::string, std::string>> tail_keywords;
};

enum class Eu7TractionWireMaterial : std::uint8_t {
    None = 0,
    Copper = 1,
    Aluminium = 2,
};

struct Eu7Traction {
    Eu7BasicNode node;
    std::string power_supply_name;
    float nominal_voltage = 0.f;
    float max_current = 0.f;
    float resistivity_ohm_per_m = 0.f;
    double resistivity_legacy = 0.0;
    std::string material_raw = "cu";
    Eu7TractionWireMaterial material = Eu7TractionWireMaterial::Copper;
    float wire_thickness = 0.f;
    int damage_flag = 0;
    glm::dvec3 wire_p1{};
    glm::dvec3 wire_p2{};
    glm::dvec3 wire_p3{};
    glm::dvec3 wire_p4{};
    double min_height = 0.0;
    double segment_length = 0.0;
    int wire_count = 0;
    float wire_offset = 0.f;
    std::optional<std::string> parallel_name;
};

enum class Eu7PowerSourceModifier : std::uint8_t {
    None,
    Recuperation,
    Section,
};

struct Eu7TractionPowerSource {
    Eu7BasicNode node;
    glm::dvec3 position{};
    float nominal_voltage = 0.f;
    float voltage_frequency = 0.f;
    double internal_resistance_legacy = 0.2;
    float internal_resistance = 0.2f;
    float max_output_current = 0.f;
    float fast_fuse_timeout = 0.f;
    float fast_fuse_repetition = 0.f;
    float slow_fuse_timeout = 0.f;
    Eu7PowerSourceModifier modifier = Eu7PowerSourceModifier::None;
};

struct Eu7Shape {
    Eu7BasicNode node;
    bool translucent = false;
    std::string material_path;
    Eu7LightingData lighting;
    glm::dvec3 origin{};
    std::vector<Eu7WorldVertex> vertices;
};

struct Eu7Lines {
    Eu7BasicNode node;
    Eu7LightingData lighting;
    float line_width = 1.f;
    glm::dvec3 origin{};
    std::vector<Eu7WorldVertex> vertices;
};

struct Eu7Model {
    Eu7BasicNode node;
    glm::dvec3 location{};
    glm::dvec3 angles{};
    glm::dvec3 scale{ 1.0, 1.0, 1.0 };
    std::string model_file;
    std::string texture_file;
    std::vector<float> light_states;
    std::vector<std::uint32_t> light_colors;
    bool transition = true;
    bool is_terrain = false;
};

[[nodiscard]] inline std::string
pack_nodedata_cache_key( Eu7Model const &model ) {
    return model.model_file + '\x1f' + model.texture_file + '\x1f'
        + std::to_string( model.node.range_squared_min ) + '\x1f'
        + std::to_string( model.node.range_squared_max ) + '\x1f'
        + ( model.is_terrain ? '1' : '0' );
}

// Wspolna definicja modelu (EU7B v8 PROT) — bez transformacji instancji.
struct Eu7ModelPrototype {
    Eu7BasicNode node;
    std::string model_file;
    std::string texture_file;
    std::vector<float> light_states;
    std::vector<std::uint32_t> light_colors;
    bool transition = true;
    bool is_terrain = false;
};

struct Eu7MemCell {
    Eu7BasicNode node;
    std::string text;
    double value1 = 0.0;
    double value2 = 0.0;
    std::optional<std::string> track_name;
};

struct Eu7EventLauncherCondition {
    std::string memcell_name;
    std::string compare_text;
    double compare_value1 = 0.0;
    double compare_value2 = 0.0;
    int check_mask = 0;
};

struct Eu7EventLauncher {
    Eu7BasicNode node;
    glm::dvec3 location{};
    double radius_squared = 0.0;
    std::string activation_key_raw;
    int activation_key = 0;
    double delta_time = -1.0;
    std::string event1_name;
    std::string event2_name;
    std::optional<Eu7EventLauncherCondition> condition;
    bool train_triggered = false;
    int launch_hour = -1;
    int launch_minute = -1;
};

struct Eu7Dynamic {
    Eu7BasicNode node;
    std::string data_folder;
    std::string skin_file;
    std::string mmd_file;
    std::string track_name;
    double offset = -1.0;
    std::string driver_type;
    int coupling = 3;
    std::string coupling_raw = "3";
    std::string coupling_params;
    float velocity = 0.f;
    int load_count = 0;
    std::string load_type;
    std::optional<std::string> destination;
    std::optional<std::size_t> trainset_index;
};

struct Eu7Sound {
    Eu7BasicNode node;
    glm::dvec3 location{};
    std::string wav_file;
};

enum class Eu7EventType : std::uint8_t {
    AddValues,
    UpdateValues,
    CopyValues,
    GetValues,
    PutValues,
    Whois,
    LogValues,
    Multiple,
    Switch,
    TrackVel,
    Sound,
    Texture,
    Animation,
    Lights,
    Voltage,
    Visible,
    Friction,
    Message,
    Unknown,
};

struct Eu7Event {
    std::string name;
    Eu7EventType type = Eu7EventType::Unknown;
    double delay = 0.0;
    std::vector<std::string> targets;
    double delay_random = 0.0;
    double delay_departure = 0.0;
    bool ignored = false;
    bool passive = false;
    std::vector<std::pair<std::string, std::string>> payload;
};

struct Eu7Trainset {
    std::string name;
    std::string track;
    float offset = 0.f;
    float velocity = 0.f;
    std::unordered_map<std::string, std::string> assignment;
    std::vector<std::size_t> vehicle_indices;
    std::vector<int> couplings;
    std::size_t driver_index = static_cast<std::size_t>( -1 );
};

struct Eu7IncludePlacement {
    std::uint8_t origin_x_param = 0;
    std::uint8_t origin_y_param = 0;
    std::uint8_t origin_z_param = 0;
    std::uint8_t rotation_y_param = 0;

    [[nodiscard]] bool
    empty() const noexcept {
        return origin_x_param == 0 && origin_y_param == 0 && origin_z_param == 0 &&
            rotation_y_param == 0;
    }
};

struct Eu7Include {
    std::uint32_t source_line = 0;
    std::string source_path;
    std::string binary_path;
    std::vector<std::string> parameters;
    Eu7TransformContext site_transform;
};

struct Eu7Scene {
    std::vector<Eu7Track> tracks;
    std::vector<Eu7Traction> traction;
    std::vector<Eu7TractionPowerSource> power_sources;
    std::vector<Eu7Shape> shapes;
    std::vector<Eu7Shape> terrain_shapes;
    std::vector<Eu7Lines> lines;
    std::vector<Eu7Model> models;
    std::vector<Eu7MemCell> memcells;
    std::vector<Eu7EventLauncher> event_launchers;
    std::vector<Eu7Dynamic> dynamics;
    std::vector<Eu7Sound> sounds;
    std::vector<Eu7Trainset> trainsets;
    std::vector<Eu7Event> events;
    std::uint32_t first_init_count = 0;
};

// Wpis indeksu PACK: sekcja 1 km x 1 km -> offset w payload chunku PACK.
struct Eu7PackIndexEntry {
    std::uint16_t row = 0;
    std::uint16_t column = 0;
    std::uint32_t model_count = 0;
    std::uint64_t pack_offset = 0;
};

// Stan odczytu sekcji PACK z istream (uzywany wewnetrznie przez reader).
struct Eu7PackSectionCursor {
    std::uint32_t solo_remaining { 0 };
    std::uint32_t inst_remaining { 0 };
    std::uint32_t models_read { 0 };
    std::uint32_t model_total { 0 };
    std::uint8_t section_format { 0 };
    bool header_parsed { false };
    std::vector<std::string> unique_meshes;
    std::vector<std::string> unique_textures;
    std::uint32_t chunk_count { 0 };
    std::vector<std::uint32_t> chunk_model_counts;
    std::vector<std::uint32_t> chunk_byte_offsets;
};

struct Eu7PackSectionLoad {
    std::vector<Eu7Model> models;
    std::vector<std::string> unique_meshes;
    std::vector<std::string> unique_textures;
};

struct Eu7PackSectionChunkLoad {
    std::vector<Eu7Model> models;
    std::vector<std::string> unique_meshes;
    std::vector<std::string> unique_textures;
    std::uint32_t chunk_count { 1 };
    std::uint32_t chunk_index { 0 };
};

struct Eu7PackCatalog {
    std::vector<Eu7PackIndexEntry> entries;
    std::unordered_map<std::uint32_t, std::size_t> index_by_section;
    std::uint64_t pack_payload_size = 0;
};

struct Eu7Module {
    std::vector<Eu7Include> includes;
    Eu7Scene scene;
    Eu7IncludePlacement include_placement;
    Eu7PackCatalog pack_catalog;
    std::vector<Eu7ModelPrototype> model_prototypes;
    std::vector<std::string> strings;
    std::string source_path;
    std::uint64_t pack_payload_offset = 0;
    std::uint32_t version = 0;
    bool has_terrain_chunk = false;
    bool has_pack_chunk = false;
};

} // namespace scene::eu7
