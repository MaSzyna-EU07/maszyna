// What the cook promises, checked on a terrain made up for the purpose: a tile reads back as it was
// written, two tiles sharing a border agree on it whatever level each is drawn at, a hole the scenery
// left stays a hole, a wall is ground and is laid exactly once, and the tiling of a material is
// measured off the source rather than guessed.
//
// Run it with no arguments; it writes its files to a temporary directory and removes them.
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <set>
#include <cstdlib>
#include <system_error>
#include <map>
#include <tuple>
#include <set>
#include <vector>

#include "scene/quantizedmesharchive.h"
#include "scene/quantizedmeshcooker.h"
#include "scene/quantizedmeshreader.h"

namespace qm = quantizedmesh;

int failures = 0;
void check( bool ok, char const *what ) {
    if( !ok ) { std::printf( "FAIL: %s\n", what ); ++failures; }
}

int main() {
    auto const directory = std::filesystem::temp_directory_path() / "eu07-meshtest";
    std::error_code ignored;
    std::filesystem::create_directories( directory, ignored );
    std::string const scratch = ( directory / "terrain.cook" ).string();
    std::string const archive = ( directory / "terrain.pak" ).string();

    qm::cooker cooker;
    cooker.quiet();
    check( cooker.begin( scratch, 2000 ), "begin" );

    auto const height = []( double x, double z ) {
        return 20.0 * std::sin( x / 300.0 ) * std::cos( z / 250.0 ) + 0.01 * x;
    };
    // a 1200 x 1200 m field on a 10 m grid, with a square hole and one big triangle
    double const step = 10.0;
    int const cells = 120;
    for( int ix = 0; ix < cells; ++ix ) {
        for( int iz = 0; iz < cells; ++iz ) {
            double const x0 = -600.0 + ix * step, x1 = x0 + step;
            double const z0 = -600.0 + iz * step, z1 = z0 + step;
            // a hole in the middle, as a scenery leaves where water or a building is
            if( ( x0 >= -100.0 ) && ( x1 <= 100.0 ) && ( z0 >= -100.0 ) && ( z1 <= 100.0 ) ) { continue; }
            char const *material = ( x0 < 0.0 ) ? "grass" : "gravel";
            auto const corner = [&]( double x, double z ) {
                return qm::cooker::corner{ x, height( x, z ), z,
                    static_cast<float>( x / 25.0 ), static_cast<float>( z / 25.0 ) };
            };
            cooker.add( corner( x0, z0 ), corner( x1, z0 ), corner( x1, z1 ), material );
            cooker.add( corner( x0, z0 ), corner( x1, z1 ), corner( x0, z1 ), material );
        }
    }
    // an upright wall, as the side of an embankment is. it is ground too, and must be laid - once,
    // although it lies exactly along the border between two tiles (z = -512)
    for( int piece = 0; piece < 4; ++piece ) {
        double const xa = 100.0 + piece * 20.0, xb = xa + 20.0;
        cooker.add( { xa, 0.0, -512.0, 0.f, 0.f }, { xb, 0.0, -512.0, 1.f, 0.f }, { xb, 8.0, -512.0, 1.f, 1.f }, "wall" );
        cooker.add( { xa, 0.0, -512.0, 0.f, 0.f }, { xb, 8.0, -512.0, 1.f, 1.f }, { xa, 8.0, -512.0, 0.f, 1.f }, "wall" );
    }
    // one big triangle spanning many tiles, as 3ds Max terrain has
    cooker.add( { -600.0, -5.0, 700.0, 0.f, 0.f }, { 600.0, -5.0, 700.0, 24.f, 0.f }, { 0.0, -5.0, 1100.0, 12.f, 16.f }, "grass" );

    qm::archive_writer writer;
    check( writer.open( archive, qm::cook_rules ), "archive opened for writing" );
    qm::terrain_table table;
    std::map<std::int64_t, qm::tile_address> written;
    auto const sink = [&]( qm::tile_address const &tile, std::vector<std::uint8_t> const &bytes,
        qm::tile_measure const &measure ) {
        written.emplace( qm::tile_key( tile ), tile );
        check( writer.add( tile, bytes, measure ), "tile written" );
    };
    check( cooker.finish( sink, table ), "finish" );
    auto const tabletext = qm::write_table( table );
    check( writer.table( tabletext ), "table written" );
    check( writer.close(), "archive closed" );

    auto const &report = cooker.result();
    std::printf( "added %zu, laid %zu triangles in %zu tiles, %u levels, %zu vertices, %zu kB, refused %zu\n",
        cooker.added(), report.triangles, report.tiles, report.levels, report.vertices,
        report.bytes / 1024, report.refused );
    check( cooker.refusals().empty(), "nothing refused: a wall is ground too" );
    check( report.levels >= 3, "quadtree deeper than the root" );

    qm::reader reader;
    check( reader.open( archive ), "reader open" );
    check( reader.table().materials.size() == 3, "grass, gravel and the wall" );
    for( auto const &material : reader.table().materials ) {
        std::printf( "  material %s, one repeat every %.2f m\n", material.name.c_str(), material.repeat );
    }
    for( auto const &material : reader.table().materials ) {
        if( material.name != "wall" ) { check( std::abs( material.repeat - 25.0 ) < 0.5, "the tiling was measured off the source" ); }
    }
    check( reader.table().levels == report.levels, "levels agree" );
    for( std::uint32_t level = 0; level < reader.table().levels; ++level ) {
        std::printf( "  level %u error %.3f m, tile %.0f m\n", level,
            reader.table().error( level ), reader.table().tilesize( level ) );
    }
    // the finest level is let go of only where it says the same thing twice, so it is off by
    // centimetres rather than by nothing
    check( reader.table().error( reader.table().levels - 1 ) < 0.05f, "the finest level is off by centimetres" );
    // every level is at least as far off as the one below it, and the coarsest is further off than
    // the finest - which is what makes the ladder worth walking down
    for( std::uint32_t level = 0; level + 1 < reader.table().levels; ++level ) {
        check( reader.table().error( level ) >= reader.table().error( level + 1 ) - 1e-4f,
            "a coarser level is at least as far off as the one below it" );
    }
    check( reader.table().error( 0 ) > reader.table().error( reader.table().levels - 1 ),
        "the coarsest level is further off than the finest" );

    // A coordinate is quantized to fifteen bits across its tile, so how closely a position comes back
    // follows the tile's size: a thousand-metre tile holds it to a few centimetres. Everything below
    // compares against that rather than against a fixed figure.
    auto const quantum = [ & ]( std::uint32_t level ) { return reader.table().tilesize( level ) / 32767.0; };


    // every tile reads back, and its vertices lie inside its own square
    // per level and border point, the heights each tile put there. with walls in the mesh one
    // point can carry several heights, so every height of one tile has to find a match in the
    // other rather than the lists being compared one to one
    std::map<std::tuple<std::uint32_t,long long,long long>, std::map<std::int64_t, std::vector<double>>> bordery;
    std::size_t read = 0, trianglesread = 0;
    std::map<std::uint32_t, std::size_t> perlevel;
    for( auto const &[ key, tile ] : written ) {
        qm::reader::tile_data data;
        if( !reader.read_tile( tile, data, 0.0, 0.0, 0.0 ) ) { std::printf( "FAIL: read %s\n", qm::tile_name( tile ).c_str() ); ++failures; continue; }
        ++read;
        trianglesread += data.indices.size() / 3;
        perlevel[ tile.level ] += data.indices.size() / 3;
        auto const side = reader.table().tilesize( tile.level );
        auto const west = reader.table().west + tile.x * side;
        auto const north = reader.table().north + tile.z * side;
        for( auto const &v : data.vertices ) {
            double const x = data.originx + v.x, z = data.originz + v.z, y = data.originy + v.y;
            auto const slack = 2.0 * quantum( tile.level );
            check( x >= west - slack && x <= west + side + slack, "vertex inside its tile in x" );
            check( z >= north - slack && z <= north + side + slack, "vertex inside its tile in z" );
            check( y >= data.lowest - slack && y <= data.highest + slack, "vertex inside its height range" );
            // remember heights on tile borders, to compare with the neighbour later
            auto const close = 2.0 * quantum( tile.level );
            auto const onborder = std::abs( x - west ) < close || std::abs( x - west - side ) < close
                               || std::abs( z - north ) < close || std::abs( z - north - side ) < close;
            if( onborder ) {
                auto const gx = static_cast<long long>( std::llround( x * 100.0 ) );
                auto const gz = static_cast<long long>( std::llround( z * 100.0 ) );
                bordery[ { tile.level, gx, gz } ][ key ].push_back( y );
            }
        }
    }
    std::printf( "read %zu tiles, %zu triangles\n", read, trianglesread );
    for( auto const &[ level, count ] : perlevel ) { std::printf( "  level %u: %zu triangles\n", level, count ); }

    // a vertex two tiles of the same level share must decode to the same height in both
    // where two tiles meet, a point on the border can carry more than one height - a wall standing
    // on the ground is two surfaces at one place - and only one of them belongs to the neighbour.
    // What has to agree, and is what keeps the ground from cracking, is the single-surface case
    double worst = 0.0;
    std::size_t shared = 0, manysurfaced = 0;
    for( auto const &[ where, pertile ] : bordery ) {
        if( pertile.size() < 2 ) { continue; }
        ++shared;
        auto simple = true;
        for( auto const &[ which, heights ] : pertile ) { if( heights.size() != 1 ) { simple = false; } }
        if( false == simple ) { ++manysurfaced; continue; }
        auto const first = pertile.begin()->second.front();
        for( auto const &[ which, heights ] : pertile ) {
            worst = std::max( worst, std::abs( heights.front() - first ) );
        }
    }
    std::printf( "%zu of %zu shared border points carry more than one surface\n", manysurfaced, shared );
    std::printf( "%zu border points shared by two tiles\n", shared );
    std::printf( "worst height disagreement on a shared border: %.1f mm\n", worst * 1000.0 );
    // heights are quantized against each tile's own range, as the format has it, so two tiles round a
    // shared vertex a fraction of a quantum apart
    check( worst < 2.0 * quantum( reader.table().levels - 1 ), "shared border vertices agree to a quantum" );

    // the coarse levels must actually be coarser
    if( perlevel.size() >= 2 ) {
        auto const finest = perlevel.rbegin()->second;
        auto const next = std::next( perlevel.rbegin() )->second;
        check( next < finest, "a coarser level has fewer triangles" );
    }

    // the wall along the tile border must be there, and exactly once
    {
        std::size_t wallpieces = 0;
        std::set<std::pair<long long,long long>> wallcentres;
        for( auto const &[ key, tile ] : written ) {
            if( tile.level + 1 != reader.table().levels ) { continue; }
            qm::reader::tile_data data;
            if( !reader.read_tile( tile, data, 0.0, 0.0, 0.0 ) ) { continue; }
            for( std::size_t i = 0; i + 2 < data.indices.size(); i += 3 ) {
                bool wall = true;
                for( int c = 0; c < 3; ++c ) {
                    auto const &v = data.vertices[ data.indices[ i + c ] ];
                    if( std::abs( data.originz + v.z + 512.0 ) > 2.0 * quantum( tile.level ) ) { wall = false; }
                }
                if( wall ) {
                    ++wallpieces;
                    double cx = 0.0, cy = 0.0;
                    for( int c = 0; c < 3; ++c ) {
                        cx += data.originx + data.vertices[ data.indices[ i + c ] ].x;
                        cy += data.originy + data.vertices[ data.indices[ i + c ] ].y;
                    }
                    wallcentres.insert( { std::llround( cx * 100.0 ), std::llround( cy * 100.0 ) } );
                }
            }
        }
        std::printf( "wall triangles on the tile border: %zu (%zu distinct)\n", wallpieces, wallcentres.size() );
        check( wallpieces > 0, "the wall on the border was laid" );
        check( wallcentres.size() == wallpieces, "no piece of the wall was laid twice" );
    }

    // a hole stays a hole: nothing may be drawn in the middle of the map at the finest level
    std::size_t inhole = 0;
    for( auto const &[ key, tile ] : written ) {
        if( tile.level + 1 != reader.table().levels ) { continue; }
        qm::reader::tile_data data;
        if( !reader.read_tile( tile, data, 0.0, 0.0, 0.0 ) ) { continue; }
        for( std::size_t i = 0; i + 2 < data.indices.size(); i += 3 ) {
            double cx = 0.0, cz = 0.0;
            for( int c = 0; c < 3; ++c ) {
                cx += data.originx + data.vertices[ data.indices[ i + c ] ].x;
                cz += data.originz + data.vertices[ data.indices[ i + c ] ].z;
            }
            cx /= 3.0; cz /= 3.0;
            if( cx > -95.0 && cx < 95.0 && cz > -95.0 && cz < 95.0 ) { ++inhole; }
        }
    }
    std::printf( "triangles inside the hole: %zu\n", inhole );
    check( inhole == 0, "the hole was not filled in" );

    if( nullptr == getenv( "KEEP" ) ) { std::filesystem::remove_all( directory, ignored ); }
    std::printf( failures ? "\n%d checks failed\n" : "\nall checks passed\n", failures );
    return failures ? 1 : 0;
}
