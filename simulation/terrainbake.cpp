/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

module;
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
#include "scene/terraincooker.h"
#include "scene/heightfieldreader.h"

module eu07.simulation.terrainbake;
import eu07.utilities.logs;
import eu07.utilities.globals;
import eu07.utilities.utilities;
import eu07.simulation.loadprofile;

namespace simulation::terrainbake {

namespace {

// triangles are buffered until there are enough vertices to say what the grid step is, and
// only then rasterised. the step has to be known before the first sample is placed, and it
// is a property of the whole terrain rather than of the first triangle that turns up
constexpr std::size_t samplegoal { 200000 };

// the node table written beside a baked heightfield
constexpr char tablemagic[ 8 ] { 'E', 'U', '0', '7', 'H', 'F', 'N', 'D' };
// 2: nodes keyed by file, line, include parameters and placement rather than by their text
constexpr std::uint32_t tableversion { 2 };
constexpr std::uint32_t allshown { 0xffffffffu };

struct table_header {
    char magic[ 8 ];
    std::uint32_t version;
    std::uint32_t selection;
    std::uint64_t count;
    std::uint64_t maskbytes;
};

struct table_record {
    std::uint64_t key;
    std::uint32_t triangles;
    // offset into the mask blob, one bit per triangle set where it is still drawn; allshown
    // when the heightfield shows every triangle and there is no mask
    std::uint32_t mask;
};

struct file_stamp {
    std::uint64_t size { 0 };
    std::int64_t time { 0 };
};

struct buffered {
    terrain::vertex v[ 3 ];
    std::uint32_t node;   // the node it came from, which holds its material
    std::uint32_t id;
};

// one node as baked: where its triangles sit in the cooker's numbering
struct baked_node {
    std::uint64_t key;
    std::uint32_t first;
    std::uint32_t count;
    // kept once per node rather than once per buffered triangle
    std::string material;
};

struct {
    // baking
    bool collecting { false };
    bool configured { false };
    terrain::cooker cooker;
    std::vector<buffered> buffer;
    std::vector<baked_node> nodes;
    std::uint32_t triangles { 0 };
    std::chrono::steady_clock::time_point started;
    // planning
    std::unordered_map<std::uint64_t, table_record> table;
    std::vector<std::uint8_t> masks;
    std::size_t skipped { 0 }, filtered { 0 }, drawn { 0 };
    // size and modification time of every file a key was taken in, looked up once per load
    std::unordered_map<std::string, file_stamp> files;
} state;

std::string
heightfield_path( std::string const &Sceneryfile ) {

    return scenery_sidecar( Sceneryfile, ".ehf" );
}

std::string
table_path( std::string const &Heightfield ) {

    return Heightfield + ".nodes";
}

bool
load_table( std::string const &Path ) {

    std::ifstream input( Path, std::ios::binary );
    if( false == input.good() ) { return false; }

    table_header header {};
    input.read( reinterpret_cast<char *>( &header ), sizeof( header ) );
    if( ( false == static_cast<bool>( input ) )
     || ( 0 != std::memcmp( header.magic, tablemagic, sizeof( tablemagic ) ) )
     || ( header.version != tableversion )
     || ( header.selection != heightfield::selection_rules ) ) {
        return false;
    }

    std::vector<table_record> records( header.count );
    input.read( reinterpret_cast<char *>( records.data() ), static_cast<std::streamsize>( header.count * sizeof( table_record ) ) );
    state.masks.resize( header.maskbytes );
    input.read( reinterpret_cast<char *>( state.masks.data() ), static_cast<std::streamsize>( header.maskbytes ) );
    if( false == static_cast<bool>( input ) ) { return false; }

    state.table.reserve( records.size() );
    for( auto const &record : records ) {
        state.table.emplace( record.key, record );
    }
    return true;
}

void
write_table( std::string const &Path ) {

    // a key met more than once is the same line of the same file, included with the same
    // parameters at the same place - the same surface: a triangle counts as shown if any of
    // the copies is
    std::unordered_map<std::uint64_t, std::vector<bool>> shown;
    for( auto const &node : state.nodes ) {
        auto &entry { shown[ node.key ] };
        entry.resize( node.count, false );
        for( std::uint32_t index = 0; index < node.count; ++index ) {
            if( true == state.cooker.represented( node.first + index ) ) { entry[ index ] = true; }
        }
    }

    std::vector<table_record> records;
    std::vector<std::uint8_t> masks;
    for( auto const &node : state.nodes ) {
        auto const lookup { shown.find( node.key ) };
        if( lookup == shown.end() ) { continue; } // written already
        auto const &entry { lookup->second };
        std::uint32_t count { 0 };
        for( auto const flag : entry ) { count += ( flag ? 1 : 0 ); }
        if( count > 0 ) {
            table_record record { node.key, node.count, allshown };
            if( count < node.count ) {
                record.mask = static_cast<std::uint32_t>( masks.size() );
                auto const bytes { ( node.count + 7u ) / 8u };
                masks.resize( masks.size() + bytes, 0u );
                for( std::uint32_t index = 0; index < node.count; ++index ) {
                    if( false == entry[ index ] ) {
                        masks[ record.mask + index / 8u ] |= static_cast<std::uint8_t>( 1u << ( index % 8u ) );
                    }
                }
            }
            records.emplace_back( record );
        }
        shown.erase( lookup );
    }

    table_header header {};
    std::memcpy( header.magic, tablemagic, sizeof( tablemagic ) );
    header.version = tableversion;
    header.selection = heightfield::selection_rules;
    header.count = records.size();
    header.maskbytes = masks.size();

    std::ofstream output( Path, std::ios::binary | std::ios::trunc );
    output.write( reinterpret_cast<char const *>( &header ), sizeof( header ) );
    output.write( reinterpret_cast<char const *>( records.data() ), static_cast<std::streamsize>( records.size() * sizeof( table_record ) ) );
    output.write( reinterpret_cast<char const *>( masks.data() ), static_cast<std::streamsize>( masks.size() ) );

    std::size_t full { 0 };
    for( auto const &record : records ) { full += ( record.mask == allshown ? 1 : 0 ); }
    WriteLog(
        "Terrain bake: " + std::to_string( records.size() ) + " of " + std::to_string( state.nodes.size() )
        + " triangle nodes are shown by the heightfield, " + std::to_string( full ) + " of them entirely" );
}

void
rasterize( buffered const &Triangle ) {

    state.cooker.rasterize(
        Triangle.v[ 0 ], Triangle.v[ 1 ], Triangle.v[ 2 ], state.nodes[ Triangle.node ].material, Triangle.id );
}

// works the grid step out of what has been buffered, then puts the buffer through
void
configure_and_drain() {

    std::vector<terrain::vertex> sample;
    sample.reserve( state.buffer.size() * 3 );
    for( auto const &triangle : state.buffer ) {
        sample.emplace_back( triangle.v[ 0 ] );
        sample.emplace_back( triangle.v[ 1 ] );
        sample.emplace_back( triangle.v[ 2 ] );
    }

    auto const step { terrain::detect_step( sample ) };
    WriteLog( "Terrain bake: grid step " + std::to_string( step ) + " m, from " + std::to_string( sample.size() ) + " sampled vertices" );

    if( false == loadprofile::enabled() ) {
        // the per-material survival table is what a bad bake is diagnosed from, so it is
        // printed when the load is being measured anyway
        state.cooker.quiet();
    }
    state.cooker.configure( step, true );
    state.configured = true;

    for( auto const &triangle : state.buffer ) {
        rasterize( triangle );
    }
    state.buffer.clear();
    state.buffer.shrink_to_fit();
}

} // anonymous namespace

namespace {

std::uint64_t
node_key(
    std::string const &File, std::size_t const Line, std::vector<std::string> const &Parameters,
    std::string_view const Type, glm::dvec3 const &Offset, glm::vec3 const &Rotation ) {

    // fnv-1a. stable across runs and builds, which a std::hash is not required to be
    std::uint64_t hash { 0xcbf29ce484222325ull };
    auto const mix {
        [ &hash ]( void const *Data, std::size_t const Size ) {
            auto const *bytes { static_cast<unsigned char const *>( Data ) };
            for( std::size_t index = 0; index < Size; ++index ) {
                hash ^= bytes[ index ];
                hash *= 0x100000001b3ull;
            } } };
    auto const text {
        [ &mix ]( std::string_view const Text ) {
            mix( Text.data(), Text.size() );
            mix( "\n", 1 ); } };

    // what the file is now: an edit anywhere in it changes every key it holds
    auto lookup { state.files.find( File ) };
    if( lookup == state.files.end() ) {
        std::error_code error;
        file_stamp stamp {};
        auto const size { std::filesystem::file_size( File, error ) };
        stamp.size = ( error ? 0 : static_cast<std::uint64_t>( size ) );
        auto const time { std::filesystem::last_write_time( File, error ) };
        stamp.time = ( error ? 0 : static_cast<std::int64_t>( time.time_since_epoch().count() ) );
        lookup = state.files.emplace( File, stamp ).first;
    }

    text( File );
    mix( &lookup->second.size, sizeof( lookup->second.size ) );
    mix( &lookup->second.time, sizeof( lookup->second.time ) );
    auto const line { static_cast<std::uint64_t>( Line ) };
    mix( &line, sizeof( line ) );
    for( auto const &parameter : Parameters ) { text( parameter ); }
    text( Type );
    mix( &Offset.x, sizeof( Offset.x ) ); mix( &Offset.y, sizeof( Offset.y ) ); mix( &Offset.z, sizeof( Offset.z ) );
    mix( &Rotation.x, sizeof( Rotation.x ) ); mix( &Rotation.y, sizeof( Rotation.y ) ); mix( &Rotation.z, sizeof( Rotation.z ) );
    return hash;
}

} // anonymous namespace

void
begin( std::string const &Sceneryfile ) {

    state = {};

    if( false == Global.terrain_heightfields.empty() ) {
        // the scenery names its own, which always wins over anything baked beside it
        return;
    }
    if( false == Global.bake_terrain ) {
        // neither bake nor pick one up: the triangles as written
        return;
    }

    auto const path { heightfield_path( Sceneryfile ) };
    std::error_code error;
    if( true == FileExists( path ) ) {
        // a baked file is a cache: one this build cannot read, one cooked under older rules
        // about what counts as terrain, or one whose node table is missing, is thrown away
        // and baked again rather than used
        heightfield::reader probe;
        if( ( true == probe.open( path ) )
         && ( probe.selection() == heightfield::selection_rules )
         && ( true == load_table( table_path( path ) ) ) ) {
            Global.terrain_heightfields.push_back( path );
            WriteLog( "Terrain bake: using \"" + path + "\", baked earlier, " + std::to_string( state.table.size() ) + " triangle nodes replaced by it" );
            return;
        }
        WriteLog( "Terrain bake: \"" + path + "\" was baked under different rules, baking it again" );
        probe.close();
        state.table.clear();
        state.masks.clear();
        std::filesystem::remove( path, error );
        std::filesystem::remove( table_path( path ), error );
    }

    WriteLog( "Terrain bake: no heightfield for this scenery, baking one. This load will be slow; the next will not" );
    state.collecting = true;
    state.started = std::chrono::steady_clock::now();
}

namespace {

bool
planning() {

    return false == state.table.empty();
}

node_plan
plan( std::uint64_t const Key ) {

    node_plan result;
    auto const lookup { state.table.find( Key ) };
    if( lookup == state.table.end() ) {
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
        result.drawn[ index ] = ( byte < state.masks.size() ) && ( ( state.masks[ byte ] >> ( index % 8u ) ) & 1u ) != 0;
    }
    return result;
}

// collecting for a bake, and not overruled by a heightfield the scenery named part way in
bool
baking() {

    return state.collecting && Global.terrain_heightfields.empty();
}

} // anonymous namespace

node_decision
examine(
    std::string const &File, std::size_t const Line, std::vector<std::string> const &Parameters,
    std::string_view const Type, glm::dvec3 const &Offset, glm::vec3 const &Rotation, bool const Worldspace ) {

    node_decision decision;
    if( ( true == baking() ) || ( true == planning() ) ) {
        decision.key = node_key( File, Line, Parameters, Type, Offset, Rotation );
        decision.bake = baking();
        if( false == decision.bake ) {
            decision.plan = plan( decision.key );
            decision.skip = ( decision.plan.what == verdict::skip );
        }
        return decision;
    }
    // a heightfield the scenery names itself, cooked by hand, replaces the world space
    // triangles outright, as it always has
    decision.skip = ( true == Worldspace ) && ( false == Global.terrain_heightfields.empty() );
    return decision;
}

void
add( std::uint64_t const Key, std::vector<world_vertex> const &Vertices, std::string_view const Material ) {

    if( false == state.collecting ) { return; }

    auto const count { static_cast<std::uint32_t>( Vertices.size() / 3 ) };
    auto const node { static_cast<std::uint32_t>( state.nodes.size() ) };
    state.nodes.push_back( { Key, state.triangles, count, std::string { Material } } );

    for( std::uint32_t triangle = 0; triangle < count; ++triangle ) {
        auto const vertex = [ & ]( std::size_t const Index ) {
            auto const &source { Vertices[ triangle * 3u + Index ] };
            return terrain::vertex{ source.position.x, source.position.y, source.position.z, source.texture.x, source.texture.y }; };
        buffered entry { { vertex( 0 ), vertex( 1 ), vertex( 2 ) }, node, state.triangles + triangle };

        if( true == state.configured ) {
            rasterize( entry );
            continue;
        }
        state.buffer.emplace_back( entry );
    }
    state.triangles += count;

    if( ( false == state.configured ) && ( state.buffer.size() * 3 >= samplegoal ) ) {
        configure_and_drain();
    }
}

void
finish( std::string const &Sceneryfile ) {

    if( true == planning() ) {
        WriteLog(
            "Terrain bake: " + std::to_string( state.skipped ) + " triangle nodes left to the heightfield, "
            + std::to_string( state.filtered ) + " drawn in part, " + std::to_string( state.drawn ) + " drawn as written" );
    }
    if( false == state.collecting ) { return; }

    if( false == Global.terrain_heightfields.empty() ) {
        // the scenery named its own heightfield part way through, and that one wins
        WriteLog( "Terrain bake: the scenery names its own heightfield, nothing baked" );
        state = {};
        return;
    }

    if( false == state.configured ) {
        // a scenery with less terrain than the sample goal never triggered the drain
        if( true == state.buffer.empty() ) {
            WriteLog( "Terrain bake: the scenery holds no triangles, nothing baked" );
            state.collecting = false;
            return;
        }
        configure_and_drain();
    }

    auto const path { heightfield_path( Sceneryfile ) };
    state.cooker.finish( path, {} );
    write_table( table_path( path ) );

    std::error_code error;
    auto const bytes { std::filesystem::file_size( path, error ) };
    auto const seconds {
        std::chrono::duration<double>( std::chrono::steady_clock::now() - state.started ).count() };
    WriteLog(
        "Terrain bake: wrote \"" + path + "\" from " + std::to_string( state.triangles )
        + " triangles in " + std::to_string( seconds ) + " s"
        + ( error ? std::string() : ", " + std::to_string( bytes / 1048576 ) + " MB" ) );

    // the triangles of this load were all drawn; the table applies from the next one
    state.collecting = false;
    state.buffer.clear();
    state.nodes.clear();
}

} // namespace simulation::terrainbake
