/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "scene/eu7/eu7_loader.h"

#include "scene/eu7/eu7_apply.h"
#include "scene/eu7/eu7_emit.h"
#include "scene/eu7/eu7_load_stats.h"
#include "scene/eu7/eu7_pack_bench.h"
#include "scene/eu7/eu7_reader.h"
#include "scene/eu7/eu7_section_stream.h"
#include "scene/eu7/eu7_parameters.h"
#include "scene/eu7/eu7_transform.h"
#include "scene/eu7/v2/eu7v2_bake.h"
#include "scene/eu7/v2/eu7v2_format.h"
#include "scene/eu7/v2/eu7v2_load.h"
#include "scene/scene.h"
#include "scene/scenenode.h"
#include "rendering/renderer.h"
#include "simulation/simulation.h"
#include "simulation/simulationstateserializer.h"
#include "scene/sn_utils.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/utilities.h"

#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace scene::eu7 {

namespace {

std::unordered_map<std::string, Eu7Module> g_module_file_cache;
bool g_packed_root_active { false };

// --- eu7v2 experiment: lossy lean-format transcode/load ---------------------
// Plik towarzyszacy obok legacy .eu7: <eu7>.v2
[[nodiscard]] std::string
eu7v2_sidecar_path( std::string const &eu7_path ) {
    return eu7_path + ".v2";
}

// .v2 jest swiezy gdy istnieje i nie jest starszy niz zrodlowy .eu7.
[[nodiscard]] bool
eu7v2_sidecar_fresh( std::string const &eu7_path, std::string const &v2_path ) {
    std::error_code ec;
    if( false == std::filesystem::exists( v2_path, ec ) ) {
        return false;
    }
    auto const v2_time { std::filesystem::last_write_time( v2_path, ec ) };
    if( ec ) {
        return false;
    }
    auto const src_time { std::filesystem::last_write_time( eu7_path, ec ) };
    if( ec ) {
        return false;
    }
    return v2_time >= src_time;
}

// Odczyt modulu z chudego .v2. Stratny: odtwarza wylacznie scene (PROT/INST/MESH
// + rekordy), bez includes/pack/prototypow/flag. Zwraca false przy bledzie.
[[nodiscard]] bool
read_eu7v2_module( std::string const &v2_path, Eu7Module &out ) {
    std::ifstream input { v2_path, std::ios::binary };
    if( !input ) {
        return false;
    }
    std::vector<std::uint8_t> bytes(
        ( std::istreambuf_iterator<char>( input ) ),
        std::istreambuf_iterator<char>() );
    if( bytes.empty() ) {
        return false;
    }
    Eu7Scene scene;
    if( false == eu7v2::load_scene( bytes.data(), bytes.size(), scene ) ) {
        return false;
    }
    out = Eu7Module{};
    out.scene = std::move( scene );
    return true;
}

// True gdy plik zaczyna sie magiem eu7v2 ('E','U','7','C').
[[nodiscard]] bool
file_is_eu7v2( std::string const &path ) {
    std::ifstream input { path, std::ios::binary };
    if( !input ) {
        return false;
    }
    char magic[ 4 ] { 0, 0, 0, 0 };
    input.read( magic, 4 );
    if( input.gcount() != 4 ) {
        return false;
    }
    return magic[ 0 ] == 'E' && magic[ 1 ] == 'U' && magic[ 2 ] == '7' && magic[ 3 ] == 'C';
}

// Pelny modul z formatu .eu7v2 (lossless): scena + includes + placement + flagi.
[[nodiscard]] bool
read_eu7v2_module_full( std::string const &path, Eu7Module &out ) {
    std::ifstream input { path, std::ios::binary };
    if( !input ) {
        return false;
    }
    std::vector<std::uint8_t> bytes(
        ( std::istreambuf_iterator<char>( input ) ),
        std::istreambuf_iterator<char>() );
    if( bytes.empty() ) {
        return false;
    }
    out = Eu7Module{};
    return eu7v2::load_module( bytes.data(), bytes.size(), out );
}

// Zapis chudego .v2 obok legacy .eu7 (best-effort, ciche niepowodzenie).
void
write_eu7v2_module( std::string const &v2_path, Eu7Module const &module ) {
    auto const bytes { eu7v2::bake_scene( module.scene ) };
    std::ofstream output { v2_path, std::ios::binary | std::ios::trunc };
    if( !output ) {
        return;
    }
    output.write(
        reinterpret_cast<char const *>( bytes.data() ),
        static_cast<std::streamsize>( bytes.size() ) );
}

// Odczyt modulu z transkodem eu7v2 gdy Global.eu7v2_runtime wlaczone:
//  - jest swiezy .v2  -> czytaj z niego (szybka, stratna sciezka),
//  - brak/nieswiezy   -> czytaj legacy i (dla nie-PACK) zapisz .v2 na nastepny raz.
[[nodiscard]] Eu7Module
read_module_maybe_v2( std::string const &resolved ) {
    if( false == Global.eu7v2_runtime ) {
        return read_module( resolved );
    }
    // Primary path: the module file itself is a native .eu7v2 container
    // (text -> eu7v2 bake, no legacy .eu7). Load it losslessly.
    if( file_is_eu7v2( resolved ) ) {
        Eu7Module v2_module;
        if( read_eu7v2_module_full( resolved, v2_module ) ) {
            ++load_stats().module_v2_loaded;
            return v2_module;
        }
        throw std::runtime_error( "EU7v2: uszkodzony modul \"" + resolved + "\"" );
    }
    // Legacy .eu7 with optional ".v2" sidecar transcode (older experiment).
    auto const v2_path { eu7v2_sidecar_path( resolved ) };
    if( eu7v2_sidecar_fresh( resolved, v2_path ) ) {
        Eu7Module v2_module;
        if( read_eu7v2_module( v2_path, v2_module ) ) {
            ++load_stats().module_v2_loaded;
            return v2_module;
        }
        WriteLog( "EU7v2: nieudany odczyt \"" + v2_path + "\", fallback legacy .eu7" );
    }
    auto legacy { read_module( resolved ) };
    if( false == legacy.has_pack_chunk ) {
        write_eu7v2_module( v2_path, legacy );
        ++load_stats().module_v2_baked;
    }
    return legacy;
}

void
apply_material( shape_node::shapenode_data &data, std::string material_name ) {
    replace_slashes( material_name );
    data.material = GfxRenderer->Fetch_Material( material_name );

    auto const texturehandle {
        data.material != null_handle ?
            GfxRenderer->Material( data.material )->GetTexture( 0 ) :
            null_handle };
    auto const &texture {
        texturehandle ?
            GfxRenderer->Texture( texturehandle ) :
            *ITexture::null_texture() };
    if( texturehandle != null_handle ) {
        data.translucent = (
            contains( texture.get_name(), '@' ) && texture.get_has_alpha() );
    }
    else {
        data.translucent = false;
    }
}

void
finalize_shape_bounds( shape_node &shape, shape_node::shapenode_data &data ) {
    if( data.vertices.empty() ) {
        return;
    }
    data.area.center = glm::dvec3( 0.0 );
    for( auto const &vertex : data.vertices ) {
        data.area.center += vertex.position;
    }
    data.area.center /=
        static_cast<double>( data.vertices.size() );
    shape.invalidate_radius();
}

} // namespace

shape_node
build_shape_node( Eu7Shape const &source ) {
    shape_node shape;
    auto &data { shape.m_data };
    data.rangesquared_min = source.node.range_squared_min;
    data.rangesquared_max = source.node.range_squared_max;
    data.visible = source.node.visible;
    data.origin = source.origin;
    data.lighting.diffuse = glm::vec4(
        source.lighting.diffuse.x, source.lighting.diffuse.y,
        source.lighting.diffuse.z, source.lighting.diffuse.w );
    data.lighting.ambient = glm::vec4(
        source.lighting.ambient.x, source.lighting.ambient.y,
        source.lighting.ambient.z, source.lighting.ambient.w );
    data.lighting.specular = glm::vec4(
        source.lighting.specular.x, source.lighting.specular.y,
        source.lighting.specular.z, source.lighting.specular.w );
    if( source.translucent ) {
        data.translucent = true;
    }
    apply_material( data, source.material_path );
    data.vertices.resize( source.vertices.size() );
    for( std::size_t i { 0 }; i < source.vertices.size(); ++i ) {
        auto const &src { source.vertices[ i ] };
        world_vertex &dst { data.vertices[ i ] };
        dst.position = src.position;
        dst.normal = src.normal;
        dst.texture = glm::vec2( static_cast<float>( src.u ), static_cast<float>( src.v ) );
    }
    finalize_shape_bounds( shape, data );
    return shape;
}

lines_node
build_lines_node( Eu7Lines const &source ) {
    lines_node node;
    auto &data { node.m_data };
    data.rangesquared_min = source.node.range_squared_min;
    data.rangesquared_max = source.node.range_squared_max;
    data.visible = source.node.visible;
    data.line_width = source.line_width;
    data.lighting.diffuse = glm::vec4(
        source.lighting.diffuse.x, source.lighting.diffuse.y,
        source.lighting.diffuse.z, source.lighting.diffuse.w );
    data.lighting.ambient = glm::vec4(
        source.lighting.ambient.x, source.lighting.ambient.y,
        source.lighting.ambient.z, source.lighting.ambient.w );
    data.lighting.specular = glm::vec4(
        source.lighting.specular.x, source.lighting.specular.y,
        source.lighting.specular.z, source.lighting.specular.w );
    data.origin = source.origin;
    data.vertices.resize( source.vertices.size() );
    for( std::size_t i { 0 }; i < source.vertices.size(); ++i ) {
        data.vertices[ i ].position = source.vertices[ i ].position;
    }
    node.m_name = source.node.name;
    return node;
}

namespace {

constexpr std::uint32_t kMagic { MAKE_ID4( 'E', 'U', '7', 'B' ) };
constexpr std::uint32_t kVersionV4 { 4 };
constexpr std::uint32_t kVersionV5 { 5 };
constexpr std::uint32_t kVersionV6 { 6 };
constexpr std::uint32_t kVersionV7 { 7 };
constexpr std::uint32_t kVersionV8 { 8 };
constexpr std::uint32_t kChunkTerr { MAKE_ID4( 'T', 'E', 'R', 'R' ) };

std::unordered_set<std::string> g_load_session;

[[nodiscard]] std::string
scenery_relative_file( std::string const &resolved ) {
    if( resolved.starts_with( Global.asCurrentSceneryPath ) ) {
        return resolved.substr( Global.asCurrentSceneryPath.size() );
    }
    return resolved;
}

[[nodiscard]] std::string
include_binary_path( Eu7Include const &include ) {
    if( !include.binary_path.empty() ) {
        return resolve_scenery_path( include.binary_path );
    }
    if( !include.source_path.empty() ) {
        return binary_path( include.source_path );
    }
    return {};
}

void
evict_non_pack_modules_from_cache( std::string const &pack_root_resolved ) {
    for( auto it { g_module_file_cache.begin() }; it != g_module_file_cache.end(); ) {
        if( it->first != pack_root_resolved ) {
            it = g_module_file_cache.erase( it );
        }
        else {
            ++it;
        }
    }
}

bool
load_module_recursive(
    std::string const &path,
    simulation::state_serializer &serializer,
    std::unordered_set<std::string> &loaded,
    Eu7TransformContext const &include_prefix = {},
    std::vector<std::string> const &include_parameters = {} ) {
    ++load_stats().module_visits;

    auto const resolved { resolve_scenery_path( path ) };
    auto const load_key { module_load_key( resolved, include_parameters ) };
    if( loaded.contains( load_key ) ) {
        ++load_stats().module_deduped;
        return true;
    }
    if( false == probe_file( resolved ) ) {
        return false;
    }

    Eu7Module const *template_module { nullptr };
    Eu7Module owned_module;
    try {
        auto const cached { g_module_file_cache.find( resolved ) };
        if( cached != g_module_file_cache.end() ) {
            ++load_stats().module_cache_hit;
            template_module = &cached->second;
        }
        else {
            ScopedTimer const read_timer { load_stats().read_ms };
            owned_module = read_module_maybe_v2( resolved );
            ++load_stats().module_read;
            auto const emplaced { g_module_file_cache.emplace( resolved, std::move( owned_module ) ) };
            template_module = &emplaced.first->second;
        }
    }
    catch( std::exception const &ex ) {
        ErrorLog( std::string{ "EU7: blad odczytu \"" + resolved + "\": " } + ex.what() );
        return false;
    }

    if( g_packed_root_active && is_model_only_module( *template_module ) ) {
        loaded.insert( load_key );
        ++load_stats().pack_skipped_includes;
        return true;
    }

    loaded.insert( load_key );
    ++load_stats().module_applied;

    if(
        false == g_packed_root_active &&
        include_parameters.empty() &&
        transform_is_empty( include_prefix ) &&
        template_module->has_pack_chunk ) {
        g_packed_root_active = true;
    }

    std::vector<Eu7Include> fallback_includes;
    for( auto const &include : template_module->includes ) {
        auto const child { include_binary_path( include ) };
        auto const ref {
            include.source_path.empty() ? include.binary_path : include.source_path };
        bool child_loaded { false };
        if( !child.empty() && should_use_binary_module( ref ) ) {
            child_loaded = load_module_recursive(
                child,
                serializer,
                loaded,
                include.site_transform,
                include.parameters );
        }
        if( child_loaded ) {
            continue;
        }
        if( !child.empty() ) {
            WriteLog(
                "EU7 include niedostepny, fallback SCM: " + include_text_path( include ) );
        }
        fallback_includes.push_back( include );
    }

    scene::scratch_data scratch;
    auto const current_relative { scenery_relative_file( resolved ) };
    if( false == fallback_includes.empty() ) {
        ScopedTimer const scm_timer { load_stats().scm_fallback_ms };
        load_stats().scm_fallback += fallback_includes.size();
        for( auto const &include : fallback_includes ) {
            serializer.deserialize_include_file(
                include_text_path( include ),
                current_relative,
                include.parameters,
                scratch );
        }
    }

    if(
        fallback_includes.empty() &&
        is_model_only_module( *template_module ) ) {
        ++load_stats().model_fast_path;

        std::vector<Eu7Model> models { template_module->scene.models };
        {
            ScopedTimer const place_timer { load_stats().place_fast_ms };
            if( false == include_parameters.empty() ) {
                apply_include_parameters_to_models( models, include_parameters );
            }
            if( false == transform_is_empty( include_prefix ) ) {
                compose_models_with_prefix( models, include_prefix );
            }
            apply_include_placement_to_models(
                models, template_module->include_placement, include_parameters );
        }
        serializer.apply_eu7_models( models, scratch );
    }
    else {
        ++load_stats().module_full_path;

        Eu7Module module { *template_module };
        {
            ScopedTimer const place_timer { load_stats().place_full_ms };
            if( false == include_parameters.empty() ) {
                apply_include_parameters_to_scene( module.scene, include_parameters );
            }

            if( false == transform_is_empty( include_prefix ) ) {
                compose_scene_with_include_prefix( module.scene, include_prefix );
            }

            apply_include_placement_to_scene(
                module.scene, module.include_placement, include_parameters );
        }

        apply_module( module, serializer, scratch );
    }

    return true;
}

[[nodiscard]] bool
file_has_terr_chunk( std::string const &path ) {
    std::ifstream input { path, std::ios::binary };
    if( !input ) {
        return false;
    }

    if( sn_utils::ld_uint32( input ) != kMagic ) {
        return false;
    }
    const std::uint32_t version { sn_utils::ld_uint32( input ) };
    if(
        version != kVersionV4 && version != kVersionV5 && version != kVersionV6 &&
        version != kVersionV7 && version != kVersionV8 ) {
        return false;
    }

    while( input.peek() != EOF ) {
        const std::uint32_t chunk_type { sn_utils::ld_uint32( input ) };
        const std::uint32_t chunk_size { sn_utils::ld_uint32( input ) };
        if( chunk_size < 8 ) {
            return false;
        }
        if( chunk_type == kChunkTerr ) {
            return true;
        }
        input.seekg( static_cast<std::streamoff>( chunk_size - 8 ), std::ios::cur );
    }
    return false;
}

} // namespace

std::string
resolve_scenery_path( std::string const &reference ) {
    auto path { reference };
    while( false == path.empty() && path[ 0 ] == '$' ) {
        path.erase( 0, 1 );
    }
    replace_slashes( path );
    if( path.starts_with( Global.asCurrentSceneryPath ) ) {
        return path;
    }
    return Global.asCurrentSceneryPath + path;
}

std::string
binary_path( std::string const &reference ) {
    auto path { resolve_scenery_path( reference ) };
    if( Global.eu7v2_runtime ) {
        return eu7v2::binary_path_from_text( path ).generic_string();
    }
    char const *const ext { ".eu7" };
    if( path.ends_with( ".scm" ) || path.ends_with( ".sbt" ) || path.ends_with( ".inc" ) ||
        path.ends_with( ".scn" ) ) {
        path.replace( path.size() - 4, 4, ext );
    }
    else if( false == path.ends_with( ".eu7" ) ) {
        erase_extension( path );
        path += ext;
    }
    return path;
}

std::string
resolve_parser_include_path(
    std::string const &parser_path,
    std::string const &current_file,
    std::string const &include_reference ) {
    auto reference { include_reference };
    while( false == reference.empty() && reference[ 0 ] == '$' ) {
        reference.erase( 0, 1 );
    }
    replace_slashes( reference );
    if( reference.starts_with( Global.asCurrentSceneryPath ) ) {
        return reference;
    }
    if( reference.find( '/' ) != std::string::npos ) {
        return parser_path + reference;
    }
    if( false == current_file.empty() ) {
        auto const slash { current_file.find_last_of( '/' ) };
        if( slash != std::string::npos ) {
            return parser_path + current_file.substr( 0, slash + 1 ) + reference;
        }
    }
    return parser_path + reference;
}

std::string
include_eu7_path(
    std::string const &parser_path,
    std::string const &current_file,
    std::string const &include_reference ) {
    return binary_path( resolve_parser_include_path( parser_path, current_file, include_reference ) );
}

std::string
terrain_binary_path( std::string const &terrain_reference ) {
    return binary_path( terrain_reference );
}

bool
probe_file( std::string const &path ) {
    if( false == FileExists( path ) ) {
        return false;
    }
    if( Global.eu7v2_runtime && file_is_eu7v2( path ) ) {
        return true;
    }
    return is_valid_eu7b_file( path );
}

bool
probe_baked_scenario( std::string const &scenario_file ) {
    return probe_file( resolve_scenery_path( binary_path( scenario_file ) ) );
}

bool
is_text_module_extension( std::string const &path ) {
    return path.ends_with( ".scn" ) || path.ends_with( ".scm" ) ||
        path.ends_with( ".inc" ) || path.ends_with( ".sbt" );
}

std::string
text_source_path( std::string const &reference ) {
    auto const resolved { resolve_scenery_path( reference ) };
    if( is_text_module_extension( resolved ) ) {
        return resolved;
    }
    if( resolved.ends_with( ".eu7" ) ) {
        auto stem { resolved };
        stem.erase( stem.size() - 4 );
        for( auto const *ext : { ".scm", ".scn", ".inc", ".sbt" } ) {
            auto const candidate { stem + ext };
            if( FileExists( candidate ) ) {
                return candidate;
            }
        }
    }
    return {};
}

bool
should_use_binary_module( std::string const &reference ) {
    return probe_file( binary_path( reference ) );
}

bool
probe_terrain_file( std::string const &path ) {
    return FileExists( path ) && file_has_terr_chunk( path );
}

bool
is_scenario_terrain( std::string const &scenario_file ) {
    auto stem { scenario_file };
    while( false == stem.empty() && stem[ 0 ] == '$' ) {
        stem.erase( 0, 1 );
    }
    erase_extension( stem );
    return probe_terrain_file( resolve_scenery_path( stem + ".eu7" ) );
}

bool
try_load_scenario_terrain( basic_region &region, std::string const &scenario_file ) {
    auto stem { scenario_file };
    while( false == stem.empty() && stem[ 0 ] == '$' ) {
        stem.erase( 0, 1 );
    }
    erase_extension( stem );
    return load_terrain( region, resolve_scenery_path( stem + ".eu7" ) );
}

void
insert_terrain_shapes( basic_region &region, Eu7Module const &module ) {
    scene::scratch_data scratch;
    std::size_t shape_count { 0 };
    std::size_t vertex_count { 0 };
    for( auto const &source : module.scene.terrain_shapes ) {
        shape_node shape { build_shape_node( source ) };
        vertex_count += source.vertices.size();
        region.insert( shape, scratch, false );
        ++shape_count;
    }
    if( shape_count > 0 ) {
        WriteLog(
            "EU7 terrain shapes: count=" + std::to_string( shape_count ) +
            " vertices=" + std::to_string( vertex_count ) );
    }
}

bool
load_terrain( basic_region &region, std::string const &path ) {
    if( false == probe_terrain_file( path ) ) {
        return false;
    }

    try {
        auto const module { read_module( path ) };
        if( module.scene.terrain_shapes.empty() ) {
            ErrorLog( "EU7: brak chunka TERR w \"" + path + "\"" );
            return false;
        }
        insert_terrain_shapes( region, module );
        WriteLog( "EU7 terrain loaded: " + path );
        return true;
    }
    catch( std::exception const &ex ) {
        ErrorLog( std::string{ "EU7: blad terenu \"" + path + "\": " } + ex.what() );
        return false;
    }
}

void
begin_load_session() {
    g_load_session.clear();
    g_packed_root_active = false;
    reset_section_stream();
    g_module_file_cache.clear();
    reset_load_stats();
    reset_pack_bench();
}

bool
is_module_loaded( std::string const &path ) {
    return g_load_session.contains( resolve_scenery_path( path ) );
}

bool
pack_scenery_active() {
    return g_packed_root_active;
}

bool
load_module( std::string const &path, simulation::state_serializer &serializer ) {
    if( simulation::Region == nullptr ) {
        ErrorLog( "EU7: Region nie jest zainicjalizowany" );
        return false;
    }
    auto const resolved { resolve_scenery_path( path ) };
    auto const ok { load_module_recursive( path, serializer, g_load_session ) };
    if( ok ) {
        log_load_stats();
        if( auto const cached { g_module_file_cache.find( resolved ) };
            cached != g_module_file_cache.end() && cached->second.has_pack_chunk ) {
            init_section_stream( cached->second, resolved, serializer );
            prime_section_stream( cached->second );
            evict_non_pack_modules_from_cache( resolved );
        }
        else {
            evict_non_pack_modules_from_cache( {} );
        }
    }
    return ok;
}

} // namespace scene::eu7
