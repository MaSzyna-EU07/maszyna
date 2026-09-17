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
#include <array>
#include <cmath>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <memory>
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

// vertices kept from the start of the scenery to tell the grid step from
constexpr std::size_t samplegoal { 200000 };

// Terrain comes in two kinds: modelled by hand with triangles hundreds of metres long, or taken
// from an elevation model with small triangles along the line. One grid step for both either
// loses the detail or samples a field of large triangles every metre. So a bake writes several
// heightfields at steps a factor apart. A triangle's own field is the coarsest that still puts
// samplesperedge samples along its longest edge, and it is laid into that field and every
// coarser one: the coarsest field then holds all the ground, as the layers of a level of
// detail scheme do, and a triangle at the edge of a finer field - next to ground that field
// does not hold - can still be replaced by a coarser one lying under it. Where two fields
// hold the same ground the finer one wins the depth test. A flat triangle sampled coarsely is
// the same plane; what a coarser step gives up is the exact line of the creases between
// triangles, which a hand modelled terrain does not have to spare anyway.
constexpr std::uint32_t fieldcount { 3 };
constexpr double fieldstride { 4.0 };
constexpr double samplesperedge { 16.0 };

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

// one triangle as it waits in the spill file for the passes: its corners in world space with
// their texture coordinates, the node it came from, which holds its material, and its number
struct spilled {
    double corners[ 15 ];
    std::uint32_t node;
    std::uint32_t id;
};

// one node as baked: where its triangles sit in the cooker's numbering
struct baked_node {
    std::uint64_t key;
    std::uint32_t first;
    std::uint32_t count;
    // kept once per node rather than once per spilled triangle
    std::string material;
};

struct {
    // baking. triangles go to a spill file as the scenery is parsed; the heightfield is made
    // from it once everything is in
    bool collecting { false };
    std::ofstream spill;
    std::string spillpath;
    std::vector<terrain::vertex> sample;
    std::vector<baked_node> nodes;
    std::uint32_t triangles { 0 };
    // per triangle, once baked: whether the heightfield shows it
    std::vector<std::uint8_t> shown;
    std::chrono::steady_clock::time_point started;
    // planning
    std::unordered_map<std::uint64_t, table_record> table;
    std::vector<std::uint8_t> masks;
    std::size_t skipped { 0 }, filtered { 0 }, drawn { 0 };
    // size and modification time of every file a key was taken in, looked up once per load
    std::unordered_map<std::string, file_stamp> files;
} state;

// where a field is written: the finest beside the scenery as <scenario>.ehf, coarser ones as
// <scenario>.<field>.ehf
std::string
heightfield_path( std::string const &Sceneryfile, std::uint32_t const Field = 0 ) {

    return scenery_sidecar( Sceneryfile, Field == 0 ? std::string( ".ehf" ) : "." + std::to_string( Field ) + ".ehf" );
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
            if( state.shown[ node.first + index ] != 0 ) { entry[ index ] = true; }
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

// the field a triangle belongs to, given the finest step
std::uint32_t
field_of( terrain::vertex const &A, terrain::vertex const &B, terrain::vertex const &C, double const Basestep ) {

    auto const longest { std::max( { terrain::planar_length( A, B ), terrain::planar_length( B, C ), terrain::planar_length( C, A ) } ) };
    std::uint32_t field { 0 };
    auto step { Basestep };
    while( ( field + 1 < fieldcount ) && ( longest / ( step * fieldstride ) >= samplesperedge ) ) {
        step *= fieldstride;
        ++field;
    }
    return field;
}

using fields = std::array<std::unique_ptr<terrain::cooker>, fieldcount>;

// one pass over the spill file: lays every triangle into fresh cookers for its own field and
// the coarser ones, wherever Keep has the bit of that field set
fields
lay( std::string const &Spillpath, double const Basestep, std::vector<std::uint8_t> const &Keep,
     std::vector<std::uint8_t> *Ownfield = nullptr ) {

    fields result;
    auto step { Basestep };
    for( auto &cooker : result ) {
        cooker = std::make_unique<terrain::cooker>();
        if( false == loadprofile::enabled() ) {
            // the per-material survival table is what a bad bake is diagnosed from, so it is
            // printed when the load is being measured anyway
            cooker->quiet();
        }
        cooker->configure( step, true );
        step *= fieldstride;
    }

    std::ifstream input( Spillpath, std::ios::binary );
    std::vector<spilled> chunk( 4096 );
    while( input ) {
        input.read( reinterpret_cast<char *>( chunk.data() ), static_cast<std::streamsize>( chunk.size() * sizeof( spilled ) ) );
        auto const count { static_cast<std::size_t>( input.gcount() ) / sizeof( spilled ) };
        for( std::size_t index = 0; index < count; ++index ) {
            auto const &triangle { chunk[ index ] };
            if( Keep[ triangle.id ] == 0 ) { continue; }
            auto const corner = [ &triangle ]( std::size_t const Corner ) {
                auto const *values { triangle.corners + Corner * 5 };
                return terrain::vertex { values[ 0 ], values[ 1 ], values[ 2 ], values[ 3 ], values[ 4 ] }; };
            auto const a { corner( 0 ) }, b { corner( 1 ) }, c { corner( 2 ) };
            auto const own { field_of( a, b, c, Basestep ) };
            if( Ownfield != nullptr ) { ( *Ownfield )[ triangle.id ] = static_cast<std::uint8_t>( own ); }
            for( auto field { own }; field < fieldcount; ++field ) {
                if( ( Keep[ triangle.id ] & ( 1u << field ) ) == 0 ) { continue; }
                result[ field ]->rasterize( a, b, c, state.nodes[ triangle.node ].material, triangle.id );
            }
        }
    }
    return result;
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
    if( true == FileExists( table_path( path ) ) ) {
        // a bake is a cache: one this build cannot read, one cooked under older rules about
        // what counts as terrain, or one missing a part, is thrown away and baked again
        // rather than used
        std::vector<std::string> baked;
        auto usable { true == load_table( table_path( path ) ) };
        for( std::uint32_t field = 0; ( field < fieldcount ) && ( true == usable ); ++field ) {
            auto const fieldpath { heightfield_path( Sceneryfile, field ) };
            if( false == FileExists( fieldpath ) ) { continue; }
            heightfield::reader probe;
            usable = ( true == probe.open( fieldpath ) ) && ( probe.selection() == heightfield::selection_rules );
            baked.push_back( fieldpath );
        }
        if( ( true == usable ) && ( false == baked.empty() ) ) {
            for( auto const &fieldpath : baked ) {
                Global.terrain_heightfields.push_back( fieldpath );
            }
            WriteLog(
                "Terrain bake: using " + std::to_string( baked.size() ) + " heightfields baked earlier, "
                + std::to_string( state.table.size() ) + " triangle nodes replaced by them" );
            return;
        }
        WriteLog( "Terrain bake: the heightfields beside this scenery were baked under different rules, baking them again" );
        state.table.clear();
        state.masks.clear();
        for( std::uint32_t field = 0; field < fieldcount; ++field ) {
            std::filesystem::remove( heightfield_path( Sceneryfile, field ), error );
        }
        std::filesystem::remove( table_path( path ), error );
    }

    // the triangles wait on disk beside the heightfield rather than in memory: a large scenery
    // holds millions of them, and the bake reads them several times
    state.spillpath = path + ".bake";
    state.spill.open( state.spillpath, std::ios::binary | std::ios::trunc );
    if( false == state.spill.good() ) {
        ErrorLog( "Terrain bake: cannot write \"" + state.spillpath + "\", not baking" );
        return;
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

    spilled triangle {};
    triangle.node = node;
    for( std::uint32_t index = 0; index < count; ++index ) {
        for( std::size_t corner = 0; corner < 3; ++corner ) {
            auto const &source { Vertices[ index * 3u + corner ] };
            auto *values { triangle.corners + corner * 5 };
            values[ 0 ] = source.position.x;
            values[ 1 ] = source.position.y;
            values[ 2 ] = source.position.z;
            values[ 3 ] = source.texture.x;
            values[ 4 ] = source.texture.y;
            // the grid step is a property of the whole terrain; the first vertices that turn up
            // are enough to tell it
            if( state.sample.size() < samplegoal ) {
                state.sample.push_back( { values[ 0 ], values[ 1 ], values[ 2 ], values[ 3 ], values[ 4 ] } );
            }
        }
        triangle.id = state.triangles + index;
        state.spill.write( reinterpret_cast<char const *>( &triangle ), sizeof( triangle ) );
    }
    state.triangles += count;
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
        state.spill.close();
        std::error_code error;
        std::filesystem::remove( state.spillpath, error );
        state = {};
        return;
    }

    state.spill.close();
    auto const cleanup = []() {
        std::error_code error;
        std::filesystem::remove( state.spillpath, error );
        state = {};
    };
    if( state.triangles == 0 ) {
        WriteLog( "Terrain bake: the scenery holds no triangles, nothing baked" );
        cleanup();
        return;
    }

    auto const step { terrain::detect_step( state.sample ) };
    WriteLog( "Terrain bake: grid step " + std::to_string( step ) + " m, from " + std::to_string( state.sample.size() ) + " sampled vertices" );

    // The heightfield should hold the triangles it replaces and nothing else: anything it
    // holds that is also drawn as geometry is drawn twice, and a roof or a bridge deck left in
    // it bends the ground. But a replaced triangle is only drawn whole if every cell around it
    // is, and those cells have corners owned by neighbours that are not replaced - the edge of
    // the ground, a slope too steep, a patch too small. So the first pass lays everything and
    // decides what is replaced; the second lays the replaced triangles together with the
    // neighbours that support them, and that is what gets written. The replaced triangles
    // stay replaced in it: their supports are all there, and they have fewer rivals than
    // before. The supports are drawn both ways, and the depth offset lets the geometry win.
    std::vector<std::uint8_t> ownfield( state.triangles, 0u );
    auto first { lay( state.spillpath, step, std::vector<std::uint8_t>( state.triangles, 0xffu ), &ownfield ) };
    // per triangle, the fields it goes into on the second pass: those that show it, and those
    // where it supports a triangle they show
    std::vector<std::uint8_t> keep( state.triangles, 0u );
    state.shown.assign( state.triangles, 0u );
    for( std::uint32_t field = 0; field < fieldcount; ++field ) {
        first[ field ]->settle();
        for( std::uint32_t id = 0; id < state.triangles; ++id ) {
            if( true == first[ field ]->represented( id ) ) {
                state.shown[ id ] = 1u;
                keep[ id ] |= static_cast<std::uint8_t>( 1u << field );
            }
        }
    }
    // a triangle no field shows, counted by the reason its own field gives
    terrain::cooker::refusals refused;
    for( std::uint32_t id = 0; id < state.triangles; ++id ) {
        if( state.shown[ id ] == 0 ) { first[ ownfield[ id ] ]->refusal_of( id, refused ); }
    }
    WriteLog(
        "Terrain bake: not replaced - " + std::to_string( refused.upright ) + " standing on edge, "
        + std::to_string( refused.unreached ) + " too small, " + std::to_string( refused.outvoted ) + " under other surfaces, "
        + std::to_string( refused.bordering ) + " next to a hole" );
    std::vector<std::uint8_t> support( state.triangles, 0u );
    for( std::uint32_t field = 0; field < fieldcount; ++field ) {
        std::fill( support.begin(), support.end(), 0u );
        first[ field ]->mark_support( support );
        first[ field ].reset();
        for( std::uint32_t id = 0; id < state.triangles; ++id ) {
            if( support[ id ] != 0 ) { keep[ id ] |= static_cast<std::uint8_t>( 1u << field ); }
        }
    }
    auto const shown { static_cast<std::size_t>( std::count( state.shown.begin(), state.shown.end(), 1u ) ) };
    std::size_t supports { 0 };
    for( std::uint32_t id = 0; id < state.triangles; ++id ) {
        if( ( keep[ id ] != 0 ) && ( state.shown[ id ] == 0 ) ) { ++supports; }
    }

    auto const second { lay( state.spillpath, step, keep ) };
    std::string written;
    std::error_code error;
    std::uintmax_t bytes { 0 };
    for( std::uint32_t field = 0; field < fieldcount; ++field ) {
        auto const path { heightfield_path( Sceneryfile, field ) };
        std::filesystem::remove( path, error );
        if( true == second[ field ]->empty() ) { continue; }
        second[ field ]->finish( path, {} );
        auto const size { std::filesystem::file_size( path, error ) };
        bytes += ( error ? 0 : size );
        written +=
            ( written.empty() ? "" : ", " ) + ( "\"" + path + "\" at " )
            + std::to_string( step * std::pow( fieldstride, field ) ) + " m";
    }
    write_table( table_path( heightfield_path( Sceneryfile ) ) );

    auto const seconds {
        std::chrono::duration<double>( std::chrono::steady_clock::now() - state.started ).count() };
    WriteLog(
        "Terrain bake: wrote " + written + "; " + std::to_string( shown ) + " of " + std::to_string( state.triangles )
        + " triangles replaced and " + std::to_string( supports ) + " more supporting them, "
        + std::to_string( seconds ) + " s, " + std::to_string( bytes / 1048576 ) + " MB" );

    // the triangles of this load were all drawn; the table applies from the next one
    cleanup();
}

} // namespace simulation::terrainbake
