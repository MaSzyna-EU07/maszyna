/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "scene/eu7/eu7_reader.h"

#include "scene/eu7/eu7_chunks.h"
#include "scene/sn_utils.h"
#include "utilities/utilities.h"

#include <fstream>
#include <limits>
#include <stdexcept>

namespace scene::eu7 {
namespace {

constexpr std::uint8_t kNodeFlagHasName { 1u << 0 };
constexpr std::uint8_t kNodeFlagHasRangeMin { 1u << 1 };
constexpr std::uint8_t kNodeFlagHasRangeMax { 1u << 2 };
constexpr std::uint8_t kNodeFlagHasBounds { 1u << 3 };
constexpr std::uint8_t kNodeFlagHasGroup { 1u << 4 };
constexpr std::uint8_t kNodeFlagHasTransform { 1u << 5 };
constexpr std::uint8_t kNodeFlagNotVisible { 1u << 6 };

constexpr std::uint8_t kTerrFlagTranslucent { 1u << 0 };
constexpr std::uint8_t kTerrFlagNonDefaultLighting { 1u << 1 };
constexpr std::uint8_t kTerrFlagBatched { 1u << 2 };
constexpr std::size_t kTerrVertsPerRecord { 3 };

constexpr std::uint8_t kTrackTailCustom { 255 };
constexpr std::uint32_t kMaxTrackPaths { 65536 };
constexpr std::uint32_t kMaxTrackTailKeywords { 256 };

class StringTable {
public:
    static constexpr std::uint32_t kEmpty { 0xFFFFFFFFu };

    StringTable() = default;

    void
    load( std::vector<std::string> strings ) {
        strings_ = std::move( strings );
    }

    explicit StringTable( std::vector<std::string> const &strings )
        : strings_ { strings } {}

    [[nodiscard]] std::string const &
    resolve( std::uint32_t const index ) const {
        static std::string const empty;
        if( index == kEmpty || index >= strings_.size() ) {
            return empty;
        }
        return strings_[ index ];
    }

    [[nodiscard]] std::vector<std::string> const &
    strings() const noexcept {
        return strings_;
    }

private:
    std::vector<std::string> strings_;
};

[[nodiscard]] std::int16_t
read_i16( std::istream &input ) {
    return static_cast<std::int16_t>( sn_utils::ld_uint16( input ) );
}

[[nodiscard]] float
half_to_float( std::uint16_t const value ) {
    const std::uint32_t sign { static_cast<std::uint32_t>( value & 0x8000u ) << 16u };
    const std::uint32_t exponent { ( value & 0x7C00u ) >> 10u };
    std::uint32_t mantissa { value & 0x03FFu };

    std::uint32_t bits { 0 };
    if( exponent == 0 ) {
        if( mantissa == 0 ) {
            bits = sign;
        }
        else {
            std::uint32_t exp { 127 - 15 - 1 };
            while( ( mantissa & 0x400u ) == 0 ) {
                mantissa <<= 1u;
                --exp;
            }
            mantissa &= 0x3FFu;
            bits = sign | ( exp << 23u ) | ( mantissa << 13u );
        }
    }
    else if( exponent == 31 ) {
        bits = sign | 0x7F800000u | ( mantissa << 13u );
    }
    else {
        bits = sign | ( ( exponent + 127 - 15 ) << 23u ) | ( mantissa << 13u );
    }

    float result { 0.f };
    std::memcpy( &result, &bits, sizeof( result ) );
    return result;
}

[[nodiscard]] float
snorm16_to_float( std::int16_t const value ) {
    return static_cast<float>( value ) * ( 1.f / 32767.f );
}

// EU7B v4/v5: io::writeF64/writeVec3 zapisuja float32 na dysku.
[[nodiscard]] double
read_f64_disk( std::istream &input ) {
    return static_cast<double>( sn_utils::ld_float32( input ) );
}

[[nodiscard]] glm::dvec3
read_vec3( std::istream &input ) {
    return {
        static_cast<double>( sn_utils::ld_float32( input ) ),
        static_cast<double>( sn_utils::ld_float32( input ) ),
        static_cast<double>( sn_utils::ld_float32( input ) ) };
}

[[nodiscard]] Eu7WorldVertex
read_packed_vertex( std::istream &input ) {
    Eu7WorldVertex vertex;
    vertex.position = read_vec3( input );
    vertex.normal.x = snorm16_to_float( read_i16( input ) );
    vertex.normal.y = snorm16_to_float( read_i16( input ) );
    vertex.normal.z = snorm16_to_float( read_i16( input ) );
    vertex.u = half_to_float( sn_utils::ld_uint16( input ) );
    vertex.v = half_to_float( sn_utils::ld_uint16( input ) );
    return vertex;
}

[[nodiscard]] Eu7LightingData
read_lighting_block( std::istream &input ) {
    Eu7LightingData lighting;
    lighting.diffuse.x = sn_utils::ld_float32( input );
    lighting.diffuse.y = sn_utils::ld_float32( input );
    lighting.diffuse.z = sn_utils::ld_float32( input );
    lighting.diffuse.w = sn_utils::ld_float32( input );
    lighting.ambient.x = sn_utils::ld_float32( input );
    lighting.ambient.y = sn_utils::ld_float32( input );
    lighting.ambient.z = sn_utils::ld_float32( input );
    lighting.ambient.w = sn_utils::ld_float32( input );
    lighting.specular.x = sn_utils::ld_float32( input );
    lighting.specular.y = sn_utils::ld_float32( input );
    lighting.specular.z = sn_utils::ld_float32( input );
    lighting.specular.w = sn_utils::ld_float32( input );
    return lighting;
}

[[nodiscard]] Eu7TransformContext
read_transform_context( std::istream &input ) {
    Eu7TransformContext transform;
    const std::uint8_t origin_count { sn_utils::d_uint8( input ) };
    transform.origin_stack.reserve( origin_count );
    for( std::uint8_t i { 0 }; i < origin_count; ++i ) {
        transform.origin_stack.push_back( read_vec3( input ) );
    }
    const std::uint8_t scale_count { sn_utils::d_uint8( input ) };
    transform.scale_stack.reserve( scale_count );
    for( std::uint8_t i { 0 }; i < scale_count; ++i ) {
        transform.scale_stack.push_back( read_vec3( input ) );
    }
    transform.rotation = read_vec3( input );
    transform.group_depth = sn_utils::d_uint8( input );
    return transform;
}

[[nodiscard]] Eu7BasicNode
read_slim_node( std::istream &input, StringTable const &table, std::string_view const implied_type ) {
    Eu7BasicNode node;
    node.node_type = std::string( implied_type );
    const std::uint8_t flags { sn_utils::d_uint8( input ) };
    if( ( flags & kNodeFlagHasName ) != 0 ) {
        node.name = table.resolve( sn_utils::ld_uint32( input ) );
    }
    if( ( flags & kNodeFlagHasRangeMin ) != 0 ) {
        node.range_squared_min = read_f64_disk( input );
    }
    if( ( flags & kNodeFlagHasRangeMax ) != 0 ) {
        node.range_squared_max = read_f64_disk( input );
    }
    else {
        node.range_squared_max = std::numeric_limits<double>::max();
    }
    if( ( flags & kNodeFlagHasBounds ) != 0 ) {
        node.area.center = read_vec3( input );
        node.area.radius = sn_utils::ld_float32( input );
    }
    if( ( flags & kNodeFlagHasGroup ) != 0 ) {
        node.group_valid = true;
        node.group_handle = sn_utils::ld_uint32( input );
    }
    if( ( flags & kNodeFlagHasTransform ) != 0 ) {
        node.transform = read_transform_context( input );
    }
    node.visible = ( flags & kNodeFlagNotVisible ) == 0;
    return node;
}

[[nodiscard]] std::string
read_length_string( std::istream &input ) {
    const std::uint32_t length { sn_utils::ld_uint32( input ) };
    std::string text( length, '\0' );
    if( length > 0 ) {
        input.read( text.data(), static_cast<std::streamsize>( length ) );
    }
    return text;
}

void
read_strs_chunk( std::istream &input, StringTable &table ) {
    const std::uint32_t count { sn_utils::ld_uint32( input ) };
    std::vector<std::string> strings;
    strings.reserve( count );
    for( std::uint32_t i { 0 }; i < count; ++i ) {
        strings.push_back( read_length_string( input ) );
    }
    table.load( std::move( strings ) );
}

[[nodiscard]] std::string_view
mesh_subtype_name( std::uint8_t const code ) noexcept {
    switch( code ) {
    case 1:
        return "triangle_strip";
    case 2:
        return "triangle_fan";
    default:
        return "triangles";
    }
}

[[nodiscard]] std::string_view
line_subtype_name( std::uint8_t const code ) noexcept {
    switch( code ) {
    case 1:
        return "line_strip";
    case 2:
        return "line_loop";
    default:
        return "lines";
    }
}

[[nodiscard]] std::string
track_tail_keyword_name( std::uint8_t const code ) {
    switch( code ) {
    case 1:
        return "event0";
    case 2:
        return "eventall0";
    case 3:
        return "event1";
    case 4:
        return "eventall1";
    case 5:
        return "event2";
    case 6:
        return "eventall2";
    case 7:
        return "velocity";
    case 8:
        return "isolated";
    case 9:
        return "overhead";
    case 10:
        return "vradius";
    case 11:
        return "railprofile";
    case 12:
        return "trackbed";
    case 13:
        return "friction";
    case 14:
        return "fouling1";
    case 15:
        return "fouling2";
    case 16:
        return "sleepermodel";
    case 17:
        return "angle1";
    case 18:
        return "angle2";
    default:
        return {};
    }
}

[[nodiscard]] std::pair<std::string, std::string>
read_track_tail_entry( std::istream &input, StringTable const &table ) {
    const std::uint8_t code { sn_utils::d_uint8( input ) };
    std::string key;
    if( code == kTrackTailCustom ) {
        key = table.resolve( sn_utils::ld_uint32( input ) );
    }
    else {
        key = track_tail_keyword_name( code );
    }
    return { std::move( key ), table.resolve( sn_utils::ld_uint32( input ) ) };
}

[[nodiscard]] Eu7SegmentPath
read_segment_path( std::istream &input ) {
    Eu7SegmentPath seg;
    seg.p_start = read_vec3( input );
    seg.roll_start = read_f64_disk( input );
    seg.cp_out = read_vec3( input );
    seg.cp_in = read_vec3( input );
    seg.p_end = read_vec3( input );
    seg.roll_end = read_f64_disk( input );
    seg.radius = read_f64_disk( input );
    return seg;
}

[[nodiscard]] Eu7TrackVisibility
read_track_visibility( std::istream &input, StringTable const &table ) {
    Eu7TrackVisibility vis;
    vis.material1 = table.resolve( sn_utils::ld_uint32( input ) );
    vis.tex_length = sn_utils::ld_float32( input );
    vis.material2 = table.resolve( sn_utils::ld_uint32( input ) );
    vis.tex_height1 = sn_utils::ld_float32( input );
    vis.tex_width = sn_utils::ld_float32( input );
    vis.tex_slope = sn_utils::ld_float32( input );
    return vis;
}

[[nodiscard]] Eu7Track
read_runtime_track( std::istream &input, StringTable const &table ) {
    Eu7Track track;
    track.node = read_slim_node( input, table, "track" );
    track.track_type = static_cast<Eu7TrackType>( sn_utils::d_uint8( input ) );
    track.category = static_cast<Eu7TrackCategory>( sn_utils::d_uint8( input ) );
    track.length = sn_utils::ld_float32( input );
    track.track_width = sn_utils::ld_float32( input );
    track.friction = sn_utils::ld_float32( input );
    track.sound_distance = sn_utils::ld_float32( input );
    track.quality_flag = sn_utils::ld_int32( input );
    track.damage_flag = sn_utils::ld_int32( input );
    track.environment = static_cast<Eu7TrackEnvironment>( static_cast<int>( sn_utils::d_uint8( input ) ) - 1 );
    if( sn_utils::d_uint8( input ) != 0 ) {
        track.visibility = read_track_visibility( input, table );
    }
    const std::uint32_t path_count { sn_utils::ld_uint32( input ) };
    if( path_count > kMaxTrackPaths ) {
        throw std::runtime_error(
            "EU7B TRAK: path_count=" + std::to_string( path_count ) + " (uszkodzony tor \"" +
            track.node.name + "\")" );
    }
    track.paths.reserve( path_count );
    for( std::uint32_t i { 0 }; i < path_count; ++i ) {
        track.paths.push_back( read_segment_path( input ) );
    }
    const std::uint32_t tail_count { sn_utils::ld_uint32( input ) };
    if( tail_count > kMaxTrackTailKeywords ) {
        throw std::runtime_error(
            "EU7B TRAK: tail_count=" + std::to_string( tail_count ) + " (uszkodzony tor \"" +
            track.node.name + "\")" );
    }
    track.tail_keywords.reserve( tail_count );
    for( std::uint32_t i { 0 }; i < tail_count; ++i ) {
        track.tail_keywords.push_back( read_track_tail_entry( input, table ) );
    }
    return track;
}

[[nodiscard]] Eu7Shape
read_runtime_shape( std::istream &input, StringTable const &table ) {
    Eu7Shape shape;
    const std::uint8_t subtype { sn_utils::d_uint8( input ) };
    shape.node = read_slim_node( input, table, mesh_subtype_name( subtype ) );
    shape.translucent = sn_utils::d_uint8( input ) != 0;
    shape.material_path = table.resolve( sn_utils::ld_uint32( input ) );
    if( sn_utils::d_uint8( input ) != 0 ) {
        shape.lighting = read_lighting_block( input );
    }
    shape.origin = read_vec3( input );
    const std::uint32_t vertex_count { sn_utils::ld_uint32( input ) };
    shape.vertices.reserve( vertex_count );
    for( std::uint32_t i { 0 }; i < vertex_count; ++i ) {
        shape.vertices.push_back( read_packed_vertex( input ) );
    }
    return shape;
}

[[nodiscard]] Eu7Lines
read_runtime_lines( std::istream &input, StringTable const &table ) {
    Eu7Lines lines;
    const std::uint8_t subtype { sn_utils::d_uint8( input ) };
    lines.node = read_slim_node( input, table, line_subtype_name( subtype ) );
    if( sn_utils::d_uint8( input ) != 0 ) {
        lines.lighting = read_lighting_block( input );
    }
    lines.line_width = sn_utils::ld_float32( input );
    lines.origin = read_vec3( input );
    const std::uint32_t vertex_count { sn_utils::ld_uint32( input ) };
    lines.vertices.reserve( vertex_count );
    for( std::uint32_t i { 0 }; i < vertex_count; ++i ) {
        Eu7WorldVertex vertex;
        vertex.position = read_vec3( input );
        lines.vertices.push_back( vertex );
    }
    return lines;
}

[[nodiscard]] Eu7Model
read_runtime_model( std::istream &input, StringTable const &table ) {
    Eu7Model model;
    model.node = read_slim_node( input, table, "model" );
    model.is_terrain = sn_utils::d_uint8( input ) != 0;
    model.transition = sn_utils::d_uint8( input ) != 0;
    model.location = read_vec3( input );
    model.angles = read_vec3( input );
    model.scale = read_vec3( input );
    model.model_file = table.resolve( sn_utils::ld_uint32( input ) );
    model.texture_file = table.resolve( sn_utils::ld_uint32( input ) );
    const std::uint32_t light_count { sn_utils::ld_uint32( input ) };
    model.light_states.resize( light_count );
    for( std::uint32_t i { 0 }; i < light_count; ++i ) {
        model.light_states[ i ] = sn_utils::ld_float32( input );
    }
    const std::uint32_t color_count { sn_utils::ld_uint32( input ) };
    model.light_colors.resize( color_count );
    for( std::uint32_t i { 0 }; i < color_count; ++i ) {
        model.light_colors[ i ] = sn_utils::ld_uint32( input );
    }
    return model;
}

[[nodiscard]] Eu7ModelPrototype
read_runtime_prototype( std::istream &input, StringTable const &table ) {
    Eu7ModelPrototype proto;
    proto.node = read_slim_node( input, table, "model" );
    proto.is_terrain = sn_utils::d_uint8( input ) != 0;
    proto.transition = sn_utils::d_uint8( input ) != 0;
    proto.model_file = table.resolve( sn_utils::ld_uint32( input ) );
    proto.texture_file = table.resolve( sn_utils::ld_uint32( input ) );
    const std::uint32_t light_count { sn_utils::ld_uint32( input ) };
    proto.light_states.resize( light_count );
    for( std::uint32_t i { 0 }; i < light_count; ++i ) {
        proto.light_states[ i ] = sn_utils::ld_float32( input );
    }
    const std::uint32_t color_count { sn_utils::ld_uint32( input ) };
    proto.light_colors.resize( color_count );
    for( std::uint32_t i { 0 }; i < color_count; ++i ) {
        proto.light_colors[ i ] = sn_utils::ld_uint32( input );
    }
    return proto;
}

void
read_prot_chunk( std::istream &input, StringTable const &strings, Eu7Module &module ) {
    const std::uint32_t count { sn_utils::ld_uint32( input ) };
    module.model_prototypes.clear();
    module.model_prototypes.reserve( count );
    for( std::uint32_t i { 0 }; i < count; ++i ) {
        module.model_prototypes.push_back( read_runtime_prototype( input, strings ) );
    }
}

[[nodiscard]] Eu7Model
expand_prototype_instance(
    Eu7ModelPrototype const &proto,
    glm::dvec3 const &location,
    glm::dvec3 const &angles,
    glm::dvec3 const &scale,
    std::string const &name ) {
    Eu7Model model;
    model.node = proto.node;
    model.node.name = name;
    model.node.transform = {};
    model.location = location;
    model.angles = angles;
    model.scale = scale;
    model.model_file = proto.model_file;
    model.texture_file = proto.texture_file;
    model.light_states = proto.light_states;
    model.light_colors = proto.light_colors;
    model.transition = proto.transition;
    model.is_terrain = proto.is_terrain;
    return model;
}

void
parse_pack_section_header(
    Eu7Module const &module,
    std::istream &input,
    Eu7PackIndexEntry const &entry,
    Eu7PackSectionCursor &cursor ) {
    cursor = {};
    input.seekg(
        static_cast<std::streamoff>( module.pack_payload_offset + entry.pack_offset ) );

    const auto peek { input.peek() };
    const std::uint8_t first_byte {
        peek == EOF ? std::uint8_t { 0 } : static_cast<std::uint8_t>( peek ) };

    if( first_byte == kPackSectionFormatV8 ) {
        cursor.section_format = sn_utils::d_uint8( input );
        const std::uint32_t solo_total { sn_utils::ld_uint32( input ) };
        const std::uint32_t inst_total { sn_utils::ld_uint32( input ) };
        cursor.model_total = solo_total + inst_total;
        cursor.solo_remaining = solo_total;
        cursor.inst_remaining = inst_total;
    }
    else if( first_byte == 2 ) {
        throw std::runtime_error(
            "EU7 PACK: format v9 nieobslugiwany — przebake plik .eu7" );
    }
    else {
        cursor.model_total = entry.model_count;
        cursor.solo_remaining = entry.model_count;
        cursor.inst_remaining = 0;
    }

    cursor.models_read = 0;
    cursor.header_parsed = true;
}

[[nodiscard]] std::vector<Eu7Model>
read_pack_models_chunk_impl(
    Eu7Module const &module,
    std::istream &input,
    Eu7PackSectionCursor &cursor,
    std::size_t const max_count,
    StringTable const &strings ) {
    if( false == cursor.header_parsed || max_count == 0 ) {
        return {};
    }

    std::vector<Eu7Model> models;
    auto const remaining {
        static_cast<std::size_t>( cursor.solo_remaining ) +
        static_cast<std::size_t>( cursor.inst_remaining ) };
    models.reserve( std::min( max_count, remaining ) );

    while(
        models.size() < max_count &&
        ( cursor.solo_remaining > 0 || cursor.inst_remaining > 0 ) ) {
        if( cursor.solo_remaining > 0 ) {
            auto model { read_runtime_model( input, strings ) };
            model.node.transform = {};
            models.push_back( std::move( model ) );
            --cursor.solo_remaining;
        }
        else {
            const std::uint32_t proto_id { sn_utils::ld_uint32( input ) };
            if( proto_id >= module.model_prototypes.size() ) {
                throw std::runtime_error(
                    "EU7 PACK v8: proto_id " + std::to_string( proto_id ) + " poza zakresem PROT (" +
                    std::to_string( module.model_prototypes.size() ) + ")" );
            }
            auto const location { read_vec3( input ) };
            auto const angles { read_vec3( input ) };
            auto const scale { read_vec3( input ) };
            auto const name { strings.resolve( sn_utils::ld_uint32( input ) ) };
            models.push_back( expand_prototype_instance(
                module.model_prototypes[ proto_id ],
                location,
                angles,
                scale,
                name ) );
            --cursor.inst_remaining;
        }
        ++cursor.models_read;
    }

    return models;
}

[[nodiscard]] Eu7Traction
read_runtime_traction( std::istream &input, StringTable const &table ) {
    Eu7Traction traction;
    traction.node = read_slim_node( input, table, "traction" );
    traction.power_supply_name = table.resolve( sn_utils::ld_uint32( input ) );
    traction.material = static_cast<Eu7TractionWireMaterial>( sn_utils::d_uint8( input ) );
    traction.nominal_voltage = sn_utils::ld_float32( input );
    traction.max_current = sn_utils::ld_float32( input );
    traction.resistivity_ohm_per_m = sn_utils::ld_float32( input );
    traction.resistivity_legacy = read_f64_disk( input );
    traction.material_raw = table.resolve( sn_utils::ld_uint32( input ) );
    traction.wire_thickness = sn_utils::ld_float32( input );
    traction.damage_flag = sn_utils::ld_int32( input );
    traction.wire_p1 = read_vec3( input );
    traction.wire_p2 = read_vec3( input );
    traction.wire_p3 = read_vec3( input );
    traction.wire_p4 = read_vec3( input );
    traction.min_height = read_f64_disk( input );
    traction.segment_length = read_f64_disk( input );
    traction.wire_count = sn_utils::ld_int32( input );
    traction.wire_offset = sn_utils::ld_float32( input );
    if( sn_utils::d_uint8( input ) != 0 ) {
        auto const parallel { table.resolve( sn_utils::ld_uint32( input ) ) };
        if( !parallel.empty() ) {
            traction.parallel_name = parallel;
        }
    }
    return traction;
}

[[nodiscard]] Eu7TractionPowerSource
read_runtime_power_source( std::istream &input, StringTable const &table ) {
    Eu7TractionPowerSource source;
    source.node = read_slim_node( input, table, "tractionpowersource" );
    source.position = read_vec3( input );
    source.node.area.center = source.position;
    source.nominal_voltage = sn_utils::ld_float32( input );
    source.voltage_frequency = sn_utils::ld_float32( input );
    source.internal_resistance_legacy = read_f64_disk( input );
    source.internal_resistance = sn_utils::ld_float32( input );
    source.max_output_current = sn_utils::ld_float32( input );
    source.fast_fuse_timeout = sn_utils::ld_float32( input );
    source.fast_fuse_repetition = sn_utils::ld_float32( input );
    source.slow_fuse_timeout = sn_utils::ld_float32( input );
    source.modifier = static_cast<Eu7PowerSourceModifier>( sn_utils::d_uint8( input ) );
    return source;
}

[[nodiscard]] Eu7MemCell
read_runtime_memcell( std::istream &input, StringTable const &table ) {
    Eu7MemCell cell;
    cell.node = read_slim_node( input, table, "memcell" );
    cell.text = table.resolve( sn_utils::ld_uint32( input ) );
    cell.value1 = read_f64_disk( input );
    cell.value2 = read_f64_disk( input );
    auto const track { table.resolve( sn_utils::ld_uint32( input ) ) };
    if( !track.empty() ) {
        cell.track_name = track;
    }
    return cell;
}

[[nodiscard]] Eu7EventLauncher
read_runtime_launcher( std::istream &input, StringTable const &table ) {
    Eu7EventLauncher launcher;
    launcher.node = read_slim_node( input, table, "eventlauncher" );
    launcher.location = read_vec3( input );
    launcher.radius_squared = read_f64_disk( input );
    launcher.activation_key_raw = table.resolve( sn_utils::ld_uint32( input ) );
    launcher.activation_key = sn_utils::ld_int32( input );
    launcher.delta_time = read_f64_disk( input );
    launcher.event1_name = table.resolve( sn_utils::ld_uint32( input ) );
    launcher.event2_name = table.resolve( sn_utils::ld_uint32( input ) );
    launcher.launch_hour = sn_utils::ld_int32( input );
    launcher.launch_minute = sn_utils::ld_int32( input );
    if( sn_utils::d_uint8( input ) != 0 ) {
        Eu7EventLauncherCondition cond;
        cond.memcell_name = table.resolve( sn_utils::ld_uint32( input ) );
        cond.compare_text = table.resolve( sn_utils::ld_uint32( input ) );
        cond.compare_value1 = read_f64_disk( input );
        cond.compare_value2 = read_f64_disk( input );
        cond.check_mask = sn_utils::ld_int32( input );
        launcher.condition = cond;
    }
    launcher.train_triggered = sn_utils::d_uint8( input ) != 0;
    return launcher;
}

[[nodiscard]] Eu7Dynamic
read_runtime_dynamic( std::istream &input, StringTable const &table ) {
    Eu7Dynamic vehicle;
    vehicle.node = read_slim_node( input, table, "dynamic" );
    vehicle.data_folder = table.resolve( sn_utils::ld_uint32( input ) );
    vehicle.skin_file = table.resolve( sn_utils::ld_uint32( input ) );
    vehicle.mmd_file = table.resolve( sn_utils::ld_uint32( input ) );
    vehicle.track_name = table.resolve( sn_utils::ld_uint32( input ) );
    vehicle.driver_type = table.resolve( sn_utils::ld_uint32( input ) );
    vehicle.load_type = table.resolve( sn_utils::ld_uint32( input ) );
    vehicle.coupling_params = table.resolve( sn_utils::ld_uint32( input ) );
    vehicle.coupling_raw = table.resolve( sn_utils::ld_uint32( input ) );
    vehicle.offset = read_f64_disk( input );
    vehicle.coupling = sn_utils::ld_int32( input );
    if( vehicle.coupling_raw.empty() ) {
        if( !vehicle.coupling_params.empty() ) {
            vehicle.coupling_raw = std::to_string( vehicle.coupling ) + "." + vehicle.coupling_params;
        }
        else {
            vehicle.coupling_raw = std::to_string( vehicle.coupling );
        }
    }
    vehicle.load_count = sn_utils::ld_int32( input );
    vehicle.velocity = sn_utils::ld_float32( input );
    if( sn_utils::d_uint8( input ) != 0 ) {
        vehicle.destination = table.resolve( sn_utils::ld_uint32( input ) );
    }
    if( sn_utils::d_uint8( input ) != 0 ) {
        vehicle.trainset_index = sn_utils::ld_uint32( input );
    }
    return vehicle;
}

[[nodiscard]] Eu7Sound
read_runtime_sound( std::istream &input, StringTable const &table ) {
    Eu7Sound sound;
    sound.node = read_slim_node( input, table, "sound" );
    sound.location = read_vec3( input );
    sound.wav_file = table.resolve( sn_utils::ld_uint32( input ) );
    return sound;
}

[[nodiscard]] Eu7Trainset
read_runtime_trainset( std::istream &input, StringTable const &table ) {
    Eu7Trainset trainset;
    trainset.name = table.resolve( sn_utils::ld_uint32( input ) );
    trainset.track = table.resolve( sn_utils::ld_uint32( input ) );
    trainset.offset = sn_utils::ld_float32( input );
    trainset.velocity = sn_utils::ld_float32( input );
    const std::uint32_t assignment_count { sn_utils::ld_uint32( input ) };
    for( std::uint32_t i { 0 }; i < assignment_count; ++i ) {
        auto const key { table.resolve( sn_utils::ld_uint32( input ) ) };
        auto const value { table.resolve( sn_utils::ld_uint32( input ) ) };
        trainset.assignment.emplace( key, value );
    }
    const std::uint32_t vehicle_count { sn_utils::ld_uint32( input ) };
    trainset.vehicle_indices.reserve( vehicle_count );
    for( std::uint32_t i { 0 }; i < vehicle_count; ++i ) {
        trainset.vehicle_indices.push_back( sn_utils::ld_uint32( input ) );
    }
    const std::uint32_t coupling_count { sn_utils::ld_uint32( input ) };
    trainset.couplings.reserve( coupling_count );
    for( std::uint32_t i { 0 }; i < coupling_count; ++i ) {
        trainset.couplings.push_back( sn_utils::ld_int32( input ) );
    }
    trainset.driver_index = sn_utils::ld_uint32( input );
    return trainset;
}

[[nodiscard]] Eu7Event
read_runtime_event( std::istream &input, StringTable const &table ) {
    Eu7Event event;
    event.name = table.resolve( sn_utils::ld_uint32( input ) );
    event.type = static_cast<Eu7EventType>( sn_utils::d_uint8( input ) );
    event.delay = read_f64_disk( input );
    const std::uint32_t target_count { sn_utils::ld_uint32( input ) };
    event.targets.reserve( target_count );
    for( std::uint32_t i { 0 }; i < target_count; ++i ) {
        event.targets.push_back( table.resolve( sn_utils::ld_uint32( input ) ) );
    }
    event.delay_random = read_f64_disk( input );
    event.delay_departure = read_f64_disk( input );
    event.ignored = sn_utils::d_uint8( input ) != 0;
    event.passive = sn_utils::d_uint8( input ) != 0;
    const std::uint32_t payload_count { sn_utils::ld_uint32( input ) };
    event.payload.reserve( payload_count );
    for( std::uint32_t i { 0 }; i < payload_count; ++i ) {
        event.payload.emplace_back(
            table.resolve( sn_utils::ld_uint32( input ) ),
            table.resolve( sn_utils::ld_uint32( input ) ) );
    }
    return event;
}

[[nodiscard]] Eu7Shape
make_terr_shape( std::string const &material, std::uint8_t const flags, Eu7LightingData const &lighting ) {
    Eu7Shape shape;
    shape.node.node_type = "triangles";
    shape.node.range_squared_min = 0.0;
    shape.node.range_squared_max = std::numeric_limits<double>::max();
    shape.node.visible = true;
    shape.material_path = material;
    shape.translucent = ( flags & kTerrFlagTranslucent ) != 0;
    shape.lighting = lighting;
    return shape;
}

void
read_terr_chunk( std::istream &input, StringTable const &table, Eu7Scene &scene ) {
    const std::uint8_t flags { sn_utils::d_uint8( input ) };
    const std::string material { table.resolve( sn_utils::ld_uint32( input ) ) };

    Eu7LightingData lighting;
    if( ( flags & kTerrFlagNonDefaultLighting ) != 0 ) {
        lighting = read_lighting_block( input );
    }

    if( ( flags & kTerrFlagBatched ) != 0 ) {
        const std::uint32_t batch_count { sn_utils::ld_uint32( input ) };
        scene.terrain_shapes.reserve( scene.terrain_shapes.size() + batch_count );
        for( std::uint32_t i { 0 }; i < batch_count; ++i ) {
            (void)sn_utils::ld_int32( input );
            (void)sn_utils::ld_int32( input );
            Eu7Shape shape { make_terr_shape( material, flags, lighting ) };
            const std::uint32_t vertex_count { sn_utils::ld_uint32( input ) };
            if( vertex_count % kTerrVertsPerRecord != 0 ) {
                throw std::runtime_error( "EU7B TERR: liczba wierzcholkow musi byc wielokrotnoscia 3" );
            }
            shape.vertices.resize( vertex_count );
            for( std::uint32_t v { 0 }; v < vertex_count; ++v ) {
                shape.vertices[ v ] = read_packed_vertex( input );
            }
            scene.terrain_shapes.push_back( std::move( shape ) );
        }
        return;
    }

    const std::uint32_t count { sn_utils::ld_uint32( input ) };
    scene.terrain_shapes.reserve( scene.terrain_shapes.size() + count );
    for( std::uint32_t i { 0 }; i < count; ++i ) {
        Eu7Shape shape { make_terr_shape( material, flags, lighting ) };
        shape.vertices.resize( kTerrVertsPerRecord );
        for( std::size_t v { 0 }; v < kTerrVertsPerRecord; ++v ) {
            shape.vertices[ v ] = read_packed_vertex( input );
        }
        scene.terrain_shapes.push_back( std::move( shape ) );
    }
}

void
build_pack_section_index( Eu7PackCatalog &catalog ) {
    catalog.index_by_section.clear();
    catalog.index_by_section.reserve( catalog.entries.size() );
    for( std::size_t i { 0 }; i < catalog.entries.size(); ++i ) {
        auto const &entry { catalog.entries[ i ] };
        auto const key {
            ( static_cast<std::uint32_t>( entry.row ) << 16 ) |
            static_cast<std::uint32_t>( entry.column ) };
        catalog.index_by_section.emplace( key, i );
    }
}

void
read_chunk_payload(
    std::uint32_t const chunk_type,
    std::istream &input,
    StringTable &strings,
    Eu7Module &module,
    std::uint32_t const file_version ) {
    if( chunk_type == kChunkStrs ) {
        read_strs_chunk( input, strings );
    }
    else if( chunk_type == kChunkIncl ) {
        const std::uint32_t count { sn_utils::ld_uint32( input ) };
        module.includes.reserve( count );
        for( std::uint32_t i { 0 }; i < count; ++i ) {
            Eu7Include inc;
            inc.source_line = sn_utils::ld_uint32( input );
            inc.source_path = strings.resolve( sn_utils::ld_uint32( input ) );
            inc.binary_path = strings.resolve( sn_utils::ld_uint32( input ) );
            const std::uint32_t param_count { sn_utils::ld_uint32( input ) };
            inc.parameters.reserve( param_count );
            for( std::uint32_t p { 0 }; p < param_count; ++p ) {
                inc.parameters.push_back( strings.resolve( sn_utils::ld_uint32( input ) ) );
            }
            if( file_version >= kEu7VersionV6 ) {
                inc.site_transform = read_transform_context( input );
            }
            module.includes.push_back( std::move( inc ) );
        }
    }
    else if( chunk_type == kChunkTrak ) {
        const std::uint32_t count { sn_utils::ld_uint32( input ) };
        module.scene.tracks.reserve( count );
        for( std::uint32_t i { 0 }; i < count; ++i ) {
            module.scene.tracks.push_back( read_runtime_track( input, strings ) );
        }
    }
    else if( chunk_type == kChunkTerr ) {
        module.has_terrain_chunk = true;
        read_terr_chunk( input, strings, module.scene );
    }
    else if( chunk_type == kChunkMesh ) {
        const std::uint32_t count { sn_utils::ld_uint32( input ) };
        module.scene.shapes.reserve( module.scene.shapes.size() + count );
        for( std::uint32_t i { 0 }; i < count; ++i ) {
            module.scene.shapes.push_back( read_runtime_shape( input, strings ) );
        }
    }
    else if( chunk_type == kChunkLine ) {
        const std::uint32_t count { sn_utils::ld_uint32( input ) };
        module.scene.lines.reserve( count );
        for( std::uint32_t i { 0 }; i < count; ++i ) {
            module.scene.lines.push_back( read_runtime_lines( input, strings ) );
        }
    }
    else if( chunk_type == kChunkModl ) {
        const std::uint32_t count { sn_utils::ld_uint32( input ) };
        module.scene.models.reserve( count );
        for( std::uint32_t i { 0 }; i < count; ++i ) {
            module.scene.models.push_back( read_runtime_model( input, strings ) );
        }
    }
    else if( chunk_type == kChunkTrac ) {
        const std::uint32_t count { sn_utils::ld_uint32( input ) };
        module.scene.traction.reserve( count );
        for( std::uint32_t i { 0 }; i < count; ++i ) {
            module.scene.traction.push_back( read_runtime_traction( input, strings ) );
        }
    }
    else if( chunk_type == kChunkPwrs ) {
        const std::uint32_t count { sn_utils::ld_uint32( input ) };
        module.scene.power_sources.reserve( count );
        for( std::uint32_t i { 0 }; i < count; ++i ) {
            module.scene.power_sources.push_back( read_runtime_power_source( input, strings ) );
        }
    }
    else if( chunk_type == kChunkMemc ) {
        const std::uint32_t count { sn_utils::ld_uint32( input ) };
        module.scene.memcells.reserve( count );
        for( std::uint32_t i { 0 }; i < count; ++i ) {
            module.scene.memcells.push_back( read_runtime_memcell( input, strings ) );
        }
    }
    else if( chunk_type == kChunkLaun ) {
        const std::uint32_t count { sn_utils::ld_uint32( input ) };
        module.scene.event_launchers.reserve( count );
        for( std::uint32_t i { 0 }; i < count; ++i ) {
            module.scene.event_launchers.push_back( read_runtime_launcher( input, strings ) );
        }
    }
    else if( chunk_type == kChunkDynm ) {
        const std::uint32_t count { sn_utils::ld_uint32( input ) };
        module.scene.dynamics.reserve( count );
        for( std::uint32_t i { 0 }; i < count; ++i ) {
            module.scene.dynamics.push_back( read_runtime_dynamic( input, strings ) );
        }
    }
    else if( chunk_type == kChunkSond ) {
        const std::uint32_t count { sn_utils::ld_uint32( input ) };
        module.scene.sounds.reserve( count );
        for( std::uint32_t i { 0 }; i < count; ++i ) {
            module.scene.sounds.push_back( read_runtime_sound( input, strings ) );
        }
    }
    else if( chunk_type == kChunkTrset ) {
        const std::uint32_t count { sn_utils::ld_uint32( input ) };
        module.scene.trainsets.reserve( count );
        for( std::uint32_t i { 0 }; i < count; ++i ) {
            module.scene.trainsets.push_back( read_runtime_trainset( input, strings ) );
        }
    }
    else if( chunk_type == kChunkEvnt ) {
        const std::uint32_t count { sn_utils::ld_uint32( input ) };
        module.scene.events.reserve( count );
        for( std::uint32_t i { 0 }; i < count; ++i ) {
            module.scene.events.push_back( read_runtime_event( input, strings ) );
        }
    }
    else if( chunk_type == kChunkFint ) {
        module.scene.first_init_count = sn_utils::ld_uint32( input );
    }
    else if( chunk_type == kChunkPlac ) {
        module.include_placement.origin_x_param = sn_utils::d_uint8( input );
        module.include_placement.origin_y_param = sn_utils::d_uint8( input );
        module.include_placement.origin_z_param = sn_utils::d_uint8( input );
        module.include_placement.rotation_y_param = sn_utils::d_uint8( input );
    }
    else if( chunk_type == kChunkPidx ) {
        const std::uint32_t count { sn_utils::ld_uint32( input ) };
        module.pack_catalog.entries.clear();
        module.pack_catalog.entries.reserve( count );
        for( std::uint32_t i { 0 }; i < count; ++i ) {
            Eu7PackIndexEntry entry;
            entry.row = sn_utils::ld_uint16( input );
            entry.column = sn_utils::ld_uint16( input );
            entry.model_count = sn_utils::ld_uint32( input );
            entry.pack_offset = sn_utils::ld_uint64( input );
            module.pack_catalog.entries.push_back( entry );
        }
        build_pack_section_index( module.pack_catalog );
    }
    else if( chunk_type == kChunkProt ) {
        read_prot_chunk( input, strings, module );
    }
}

[[nodiscard]] std::optional<Eu7PackIndexEntry>
find_pack_entry_impl( Eu7Module const &module, int const row, int const column ) {
    if( row < 0 || row > 0xFFFF || column < 0 || column > 0xFFFF ) {
        return std::nullopt;
    }

    if( false == module.pack_catalog.index_by_section.empty() ) {
        auto const key {
            ( static_cast<std::uint32_t>( row ) << 16 ) |
            static_cast<std::uint32_t>( column ) };
        auto const found { module.pack_catalog.index_by_section.find( key ) };
        if( found == module.pack_catalog.index_by_section.end() ) {
            return std::nullopt;
        }
        return module.pack_catalog.entries[ found->second ];
    }

    for( auto const &entry : module.pack_catalog.entries ) {
        if( entry.row == static_cast<std::uint16_t>( row ) &&
            entry.column == static_cast<std::uint16_t>( column ) ) {
            return entry;
        }
    }
    return std::nullopt;
}

[[nodiscard]] std::vector<Eu7Model>
read_pack_section_impl( Eu7Module const &module, int const row, int const column ) {
    if( false == module.has_pack_chunk || module.source_path.empty() ) {
        return {};
    }

    auto const entry { find_pack_entry_impl( module, row, column ) };
    if( false == entry.has_value() || entry->model_count == 0 ) {
        return {};
    }

    std::ifstream input { module.source_path, std::ios::binary };
    if( !input ) {
        throw std::runtime_error(
            "EU7 PACK: nie mozna otworzyc \"" + module.source_path + "\"" );
    }

    StringTable const strings { module.strings };
    Eu7PackSectionCursor header;
    parse_pack_section_header( module, input, *entry, header );

    return read_pack_models_chunk_impl(
        module, input, header, std::numeric_limits<std::size_t>::max(), strings );
}

} // namespace

bool
is_valid_eu7b_file( std::string const &path ) {
    if( false == FileExists( path ) ) {
        return false;
    }
    std::ifstream input { path, std::ios::binary };
    if( !input ) {
        return false;
    }
    if( sn_utils::ld_uint32( input ) != kEu7Magic ) {
        return false;
    }
    const std::uint32_t version { sn_utils::ld_uint32( input ) };
    return (
        version == kEu7VersionV4 || version == kEu7VersionV5 || version == kEu7VersionV6 ||
        version == kEu7VersionV7 || version == kEu7VersionV8 );
}

Eu7Module
read_module( std::string const &path ) {
    std::ifstream input { path, std::ios::binary };
    if( !input ) {
        throw std::runtime_error( "EU7: nie mozna otworzyc \"" + path + "\"" );
    }

    if( sn_utils::ld_uint32( input ) != kEu7Magic ) {
        throw std::runtime_error( "EU7B: zly magic w \"" + path + "\"" );
    }

    Eu7Module module;
    module.version = sn_utils::ld_uint32( input );
    if(
        module.version != kEu7VersionV4 && module.version != kEu7VersionV5 &&
        module.version != kEu7VersionV6 && module.version != kEu7VersionV7 &&
        module.version != kEu7VersionV8 ) {
        throw std::runtime_error(
            "EU7B: nieobslugiwana wersja " + std::to_string( module.version ) + " w \"" + path + "\"" );
    }

    StringTable strings;

    while( input.peek() != EOF ) {
        const std::uint32_t chunk_type { sn_utils::ld_uint32( input ) };
        const std::uint32_t chunk_size { sn_utils::ld_uint32( input ) };
        if( chunk_size < 8 ) {
            throw std::runtime_error( "EU7B: uszkodzony chunk w \"" + path + "\"" );
        }
        const auto chunk_start { input.tellg() };
        const std::streamoff payload_size {
            static_cast<std::streamoff>( chunk_size ) - static_cast<std::streamoff>( 8 ) };

        if( chunk_type == kChunkPack ) {
            module.has_pack_chunk = true;
            module.pack_catalog.pack_payload_size = static_cast<std::uint64_t>( payload_size );
            module.pack_payload_offset = static_cast<std::uint64_t>( chunk_start );
            input.seekg( chunk_start + payload_size );
            continue;
        }

        read_chunk_payload( chunk_type, input, strings, module, module.version );

        input.seekg( chunk_start + payload_size );
    }

    module.strings = strings.strings();
    module.source_path = path;
    return module;
}

std::optional<Eu7PackIndexEntry>
find_pack_entry( Eu7Module const &module, int const row, int const column ) {
    return find_pack_entry_impl( module, row, column );
}

std::vector<Eu7Model>
read_pack_section( Eu7Module const &module, int const row, int const column ) {
    return read_pack_section_impl( module, row, column );
}

void
seek_pack_section(
    Eu7Module const &module,
    std::istream &input,
    Eu7PackIndexEntry const &entry,
    Eu7PackSectionCursor &cursor ) {
    if( false == module.has_pack_chunk ) {
        cursor = {};
        return;
    }
    parse_pack_section_header( module, input, entry, cursor );
}

std::vector<Eu7Model>
read_pack_models_chunk(
    Eu7Module const &module,
    std::istream &input,
    Eu7PackSectionCursor &cursor,
    std::size_t const max_count ) {
    StringTable const strings { module.strings };
    return read_pack_models_chunk_impl( module, input, cursor, max_count, strings );
}

void
resume_pack_section(
    Eu7Module const &module,
    std::istream &input,
    Eu7PackIndexEntry const &entry,
    std::uint64_t const resume_byte_offset,
    Eu7PackSectionCursor const resume_cursor,
    Eu7PackSectionCursor &cursor ) {
    if( false == module.has_pack_chunk ) {
        cursor = {};
        return;
    }

    input.seekg(
        static_cast<std::streamoff>(
            module.pack_payload_offset + entry.pack_offset + resume_byte_offset ) );
    cursor = resume_cursor;
}

} // namespace scene::eu7
