/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

module;
#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <system_error>
#include <unordered_map>
#include <vector>
#include "utilities/Globals_macros.h"
#include "scene/pakformat.h"
#include "scene/quantizedmeshcooker.h"
#include "scene/quantizedmeshreader.h"

module eu07.simulation.terrainbake;
import eu07.utilities.logs;
import eu07.utilities.globals;
import eu07.utilities.utilities;
import eu07.model.vertex;
import eu07.simulation.cookdeps;

namespace simulation::terrainbake {

namespace {

// the most triangles a tile of any level is allowed. the finest level is the ground as the scenery
// drew it and is not cut down, so this is a ceiling for the coarser ones rather than a target
constexpr std::size_t tilebudget { 8192 };

// the note written beside the tiles: which nodes they stand for
constexpr char notemagic[ 8 ] { 'E', 'U', '0', '7', 'T', 'N', 'O', 'D' };
constexpr std::uint32_t noteversion { 1 };
// a node with no mask: the tiles stand for every triangle of it
constexpr std::uint32_t allshown { 0xffffffffu };

struct note_header {
    char magic[ 8 ];
    std::uint32_t version;
    std::uint32_t rules;
    std::uint64_t count;
    std::uint64_t maskbytes;
};

struct note_record {
    std::uint64_t key;
    std::uint32_t triangles;
    // where this node's bits start in the mask blob, one per triangle, set where the scenery
    // still draws it. allshown when none of them is drawn any more
    std::uint32_t mask;
};

// one node as it went into the cook: where its triangles sit in the cook's numbering
struct baked_node {
    std::uint64_t key { 0 };
    std::uint32_t first { 0 };
    std::uint32_t count { 0 };
};

struct {
    // cooking
    bool collecting { false };
    quantizedmesh::cooker cooker;
    std::vector<baked_node> nodes;
    std::uint32_t triangles { 0 };
    std::size_t objects { 0 };
    std::size_t objecttriangles { 0 };
    std::chrono::steady_clock::time_point started;
    // reading a cook from an earlier run
    std::unordered_map<std::uint64_t, note_record> note;
    std::vector<std::uint8_t> masks;
    std::size_t skipped { 0 }, filtered { 0 }, drawn { 0 };
} state;

// beside the scenery, under its name: the same place the heightfield used to be cooked to
std::string archive_path( std::string const &Sceneryfile )  { return scenery_sidecar( Sceneryfile, "_terrain.pak" ); }
std::string note_path( std::string const &Sceneryfile )     { return scenery_sidecar( Sceneryfile, "_terrain.nodes" ); }
std::string manifest_path( std::string const &Sceneryfile ) { return scenery_sidecar( Sceneryfile, "_terrain.deps" ); }
std::string scratch_path( std::string const &Sceneryfile )  { return scenery_sidecar( Sceneryfile, "_terrain.cook" ); }

void
discard( std::string const &Sceneryfile ) {

    std::error_code error;
    for( auto const &path : {
            archive_path( Sceneryfile ), note_path( Sceneryfile ),
            manifest_path( Sceneryfile ), scratch_path( Sceneryfile ) } ) {
        std::filesystem::remove( path, error );
    }
}

bool
load_note( std::string const &Path ) {

    std::ifstream file( Path, std::ios::binary );
    if( false == file.good() ) { return false; }
    note_header header {};
    file.read( reinterpret_cast<char *>( &header ), sizeof( header ) );
    if( false == file.good() ) { return false; }
    if( 0 != std::memcmp( header.magic, notemagic, sizeof( notemagic ) ) ) { return false; }
    if( header.version != noteversion ) { return false; }
    if( header.rules != quantizedmesh::cook_rules ) { return false; }

    state.note.reserve( header.count );
    for( std::uint64_t index = 0; index < header.count; ++index ) {
        note_record record {};
        file.read( reinterpret_cast<char *>( &record ), sizeof( record ) );
        if( false == file.good() ) { state.note.clear(); return false; }
        state.note.emplace( record.key, record );
    }
    state.masks.resize( header.maskbytes );
    if( header.maskbytes > 0 ) {
        file.read( reinterpret_cast<char *>( state.masks.data() ), static_cast<std::streamsize>( header.maskbytes ) );
    }
    if( false == file.good() ) { state.note.clear(); state.masks.clear(); return false; }
    return true;
}

bool
save_note( std::string const &Path, std::vector<std::uint32_t> const &Refusals ) {

    // the cook refuses a triangle that has no ground in it - a wall - and the scenery goes on
    // drawing that one. the note says so per node, as a bit each
    std::vector<note_record> records;
    std::vector<std::uint8_t> masks;
    records.reserve( state.nodes.size() );

    auto refusal { Refusals.begin() };
    for( auto const &node : state.nodes ) {
        while( ( refusal != Refusals.end() ) && ( *refusal < node.first ) ) { ++refusal; }
        auto const last { node.first + node.count };
        auto const firstrefusal { refusal };
        std::size_t refused { 0 };
        for( auto walk = refusal; ( walk != Refusals.end() ) && ( *walk < last ); ++walk ) { ++refused; }
        if( refused == 0 ) {
            records.push_back( { node.key, node.count, allshown } );
            continue;
        }
        if( refused == node.count ) {
            // nothing of this node is in the tiles: it is drawn as written, and a node the note
            // says nothing about is exactly that
            continue;
        }
        auto const at { static_cast<std::uint32_t>( masks.size() ) };
        masks.resize( masks.size() + ( node.count + 7 ) / 8, 0 );
        for( auto walk = firstrefusal; ( walk != Refusals.end() ) && ( *walk < last ); ++walk ) {
            auto const bit { *walk - node.first };
            masks[ at + bit / 8 ] |= static_cast<std::uint8_t>( 1u << ( bit % 8 ) );
        }
        records.push_back( { node.key, node.count, at } );
    }

    std::ofstream file( Path, std::ios::binary | std::ios::trunc );
    if( false == file.good() ) { return false; }
    note_header header {};
    std::memcpy( header.magic, notemagic, sizeof( notemagic ) );
    header.version = noteversion;
    header.rules = quantizedmesh::cook_rules;
    header.count = records.size();
    header.maskbytes = masks.size();
    file.write( reinterpret_cast<char const *>( &header ), sizeof( header ) );
    for( auto const &record : records ) {
        file.write( reinterpret_cast<char const *>( &record ), sizeof( record ) );
    }
    if( false == masks.empty() ) {
        file.write( reinterpret_cast<char const *>( masks.data() ), static_cast<std::streamsize>( masks.size() ) );
    }
    return file.good();
}

// the key a node is known by. what it is made of, where it stands, and where it is written: the
// manifest vouches for the content of the file, so its name is enough here
std::uint64_t
node_key(
    std::string const &File, std::size_t const Line, std::vector<std::string> const &Parameters,
    std::string_view const Type, glm::dvec3 const &Offset, glm::vec3 const &Rotation ) {

    std::uint64_t hash { 0xcbf29ce484222325ull };
    auto const mix {
        [ &hash ]( void const *Data, std::size_t const Size ) {
            auto const *bytes { static_cast<std::uint8_t const *>( Data ) };
            for( std::size_t index = 0; index < Size; ++index ) {
                hash ^= bytes[ index ];
                hash *= 0x100000001b3ull;
            } } };
    auto const text {
        [ &mix ]( std::string_view const Text ) {
            mix( Text.data(), Text.size() );
            mix( "\n", 1 ); } };

    text( File );
    auto const line { static_cast<std::uint64_t>( Line ) };
    mix( &line, sizeof( line ) );
    for( auto const &parameter : Parameters ) { text( parameter ); }
    text( Type );
    mix( &Offset.x, sizeof( Offset.x ) ); mix( &Offset.y, sizeof( Offset.y ) ); mix( &Offset.z, sizeof( Offset.z ) );
    mix( &Rotation.x, sizeof( Rotation.x ) ); mix( &Rotation.y, sizeof( Rotation.y ) ); mix( &Rotation.z, sizeof( Rotation.z ) );
    return hash;
}

bool planning() { return false == state.note.empty(); }
// collecting, and not overruled by a terrain the scenery named part way in
bool baking()   { return state.collecting && Global.terrain_heightfields.empty(); }

node_plan
plan( std::uint64_t const Key ) {

    node_plan result;
    auto const lookup { state.note.find( Key ) };
    if( lookup == state.note.end() ) {
        ++state.drawn;
        return result;
    }
    auto const &record { lookup->second };
    if( record.mask == allshown ) {
        ++state.skipped;
        result.what = verdict::skip;
        return result;
    }
    ++state.filtered;
    result.what = verdict::filter;
    result.drawn.resize( record.triangles, false );
    for( std::uint32_t index = 0; index < record.triangles; ++index ) {
        auto const byte { record.mask + index / 8u };
        result.drawn[ index ] =
            ( byte < state.masks.size() ) && ( ( ( state.masks[ byte ] >> ( index % 8u ) ) & 1u ) != 0 );
    }
    return result;
}

} // anonymous namespace

void
begin( std::string const &Sceneryfile ) {

    state.collecting = false;
    state.nodes.clear();
    state.triangles = 0;
    state.objects = 0;
    state.objecttriangles = 0;
    state.note.clear();
    state.masks.clear();
    state.skipped = state.filtered = state.drawn = 0;

    if( false == Global.terrain_heightfields.empty() ) {
        // the scenery names a terrain of its own, which wins over anything cooked beside it
        return;
    }
    if( false == Global.bake_terrain ) {
        // neither cook nor pick one up: the triangles as written
        return;
    }

    auto const archive { archive_path( Sceneryfile ) };
    if( true == FileExists( note_path( Sceneryfile ) ) ) {
        // a cook is a cache. one this build cannot read, one made under other rules about what
        // counts as ground, or one made from a scenery that has changed since, is thrown away
        // and cooked again rather than used
        quantizedmesh::reader probe;
        auto const usable {
            ( true == cookdeps::current( manifest_path( Sceneryfile ) ) )
         && ( true == probe.open( archive ) )
         && ( true == load_note( note_path( Sceneryfile ) ) ) };
        if( true == usable ) {
            Global.terrain_heightfields.push_back( archive );
            WriteLog(
                "Terrain cook: using tiles cooked earlier, " + std::to_string( probe.table().levels )
                + " levels over " + std::to_string( static_cast<int>( probe.table().side ) ) + " m, "
                + std::to_string( state.note.size() ) + " triangle nodes stood for" );
            return;
        }
        WriteLog( "Terrain cook: the tiles beside this scenery are out of date, cooking them again" );
        state.note.clear();
        state.masks.clear();
        discard( Sceneryfile );
    }

    if( false == state.cooker.begin( scratch_path( Sceneryfile ), tilebudget ) ) {
        ErrorLog( "Terrain cook: cannot write \"" + scratch_path( Sceneryfile ) + "\", not cooking" );
        return;
    }
    state.cooker.quiet();
    WriteLog( "Terrain cook: no tiles for this scenery, cooking them. This load will be slow; the next will not" );
    state.collecting = true;
    state.started = std::chrono::steady_clock::now();
}

bool
collecting() {

    return state.collecting;
}

node_decision
examine(
    std::string const &File, std::size_t const Line, std::vector<std::string> const &Parameters,
    std::string_view const Type, glm::dvec3 const &Offset, glm::vec3 const &Rotation, bool const Worldspace ) {

    node_decision decision;
    if( ( true == baking() ) || ( true == planning() ) ) {
        decision.key = node_key( File, Line, Parameters, Type, Offset, Rotation );
        // only what the scenery drew in world space: a node under origin, rotate or scale is an
        // object standing on the ground rather than the ground itself
        decision.bake = baking() && Worldspace;
        if( false == decision.bake ) {
            decision.plan = plan( decision.key );
            decision.skip = ( decision.plan.what == verdict::skip );
        }
        return decision;
    }
    // a terrain the scenery names itself replaces the world space triangles outright, as it
    // always has
    decision.skip = ( true == Worldspace ) && ( false == Global.terrain_heightfields.empty() );
    return decision;
}

// What is ground and what merely stands on it.
//
// The cook lays a triangle whichever way it faces: the side of an embankment or a cutting stands
// upright and is ground all the same, and leaving those out is what opens a hole along every bank.
// So orientation cannot be the test. Two things the scenery says about a node serve instead.
//
// A node under origin, rotate or scale is an object the scenery placed, not ground it drew, and
// examine() keeps those out. And a material with an alpha channel is a tree, a bush, a fence or a
// catenary mast: the ground is opaque. That matters twice over - the terrain is drawn without an
// alpha test, so a billboard would come out as a full sheet, and every level above the finest is one
// simplified surface, which cannot be fitted through the ground and through what stands on it at
// once.
void
add(
    std::uint64_t const Key, std::vector<world_vertex> const &Vertices, std::string_view const Material,
    bool const Translucent ) {

    if( false == state.collecting ) { return; }

    auto const count { static_cast<std::uint32_t>( Vertices.size() / 3 ) };
    if( count == 0 ) { return; }
    if( true == Translucent ) {
        // no record of it, and a node the note says nothing about is one the scenery draws as written
        ++state.objects;
        state.objecttriangles += count;
        return;
    }
    state.nodes.push_back( { Key, state.triangles, count } );

    auto const corner {
        []( world_vertex const &Vertex ) {
            return quantizedmesh::cooker::corner {
                Vertex.position.x, Vertex.position.y, Vertex.position.z,
                Vertex.texture.x, Vertex.texture.y }; } };
    for( std::uint32_t index = 0; index < count; ++index ) {
        state.cooker.add(
            corner( Vertices[ index * 3u + 0 ] ),
            corner( Vertices[ index * 3u + 1 ] ),
            corner( Vertices[ index * 3u + 2 ] ),
            Material );
    }
    state.triangles += count;
}

void
finish( std::string const &Sceneryfile, std::vector<std::string> const &Included ) {

    if( true == planning() ) {
        WriteLog(
            "Terrain cook: " + std::to_string( state.skipped ) + " triangle nodes left to the tiles, "
            + std::to_string( state.filtered ) + " drawn in part, " + std::to_string( state.drawn )
            + " drawn as written" );
        // an include added or taken away since the cook: the nodes of a new file were drawn as
        // geometry this time, since none of their keys is known, but the next start cooks again
        if( false == cookdeps::lists( manifest_path( Sceneryfile ), Included ) ) {
            WriteLog( "Terrain cook: the scenery includes different files than when it was cooked; cooking again on the next start" );
            discard( Sceneryfile );
        }
        return;
    }
    if( false == state.collecting ) { return; }
    state.collecting = false;

    if( false == Global.terrain_heightfields.empty() ) {
        WriteLog( "Terrain cook: the scenery names a terrain of its own, nothing cooked" );
        return;
    }
    if( state.triangles == 0 ) {
        WriteLog( "Terrain cook: no ground in this scenery, nothing cooked" );
        return;
    }

    // the cook itself, apart from the load that fed it: what a second start saves is the cook
    auto const cookstarted { std::chrono::steady_clock::now() };
    auto const archive { archive_path( Sceneryfile ) };
    pak::writer writer;
    if( false == writer.open( archive ) ) {
        ErrorLog( "Terrain cook: cannot write \"" + archive + "\"" );
        return;
    }

    auto written { true };
    quantizedmesh::terrain_table table;
    auto const sink {
        [ &writer, &written ]( quantizedmesh::tile_address const &Tile, std::vector<std::uint8_t> const &Bytes ) {
            written = writer.add( quantizedmesh::tile_name( Tile ), Bytes ) && written; } };

    if( ( false == state.cooker.finish( sink, table ) ) || ( false == written ) ) {
        ErrorLog( "Terrain cook: cooking failed, no tiles written" );
        writer.close();
        discard( Sceneryfile );
        return;
    }
    if( false == writer.add( quantizedmesh::tablename, quantizedmesh::write_table( table ) ) ) {
        written = false;
    }
    if( ( false == writer.close() ) || ( false == written ) ) {
        ErrorLog( "Terrain cook: writing \"" + archive + "\" failed" );
        discard( Sceneryfile );
        return;
    }

    auto const &report { state.cooker.result() };
    if( false == save_note( note_path( Sceneryfile ), state.cooker.refusals() ) ) {
        ErrorLog( "Terrain cook: cannot write \"" + note_path( Sceneryfile ) + "\"" );
        discard( Sceneryfile );
        return;
    }
    if( false == cookdeps::write( manifest_path( Sceneryfile ), Included ) ) {
        discard( Sceneryfile );
        return;
    }

    auto const now { std::chrono::steady_clock::now() };
    auto const cooking { std::chrono::duration<double>( now - cookstarted ).count() };
    auto const collecting { std::chrono::duration<double>( now - state.started ).count() };
    WriteLog(
        "Terrain cook: " + std::to_string( state.triangles ) + " triangles of ground in "
        + std::to_string( state.nodes.size() ) + " nodes, " + std::to_string( state.objecttriangles )
        + " triangles in " + std::to_string( state.objects ) + " nodes left standing on it, into "
        + std::to_string( report.tiles ) + " tiles of " + std::to_string( static_cast<int>( state.cooker.finesttile() ) )
        + " m over " + std::to_string( report.levels ) + " levels, "
        + std::to_string( report.triangles ) + " triangles and " + std::to_string( report.vertices )
        + " vertices written, " + std::to_string( report.refused ) + " triangles left to the scenery, "
        + std::to_string( writer.bytes() / 1024 ) + " kB, cooked in "
        + std::to_string( static_cast<int>( cooking ) ) + " s of a "
        + std::to_string( static_cast<int>( collecting ) ) + " s load" );

    Global.terrain_heightfields.push_back( archive );
}

} // namespace simulation::terrainbake
