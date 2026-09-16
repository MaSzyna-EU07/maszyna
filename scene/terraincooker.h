/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <climits>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>
#include <zstd.h>

#include "scene/heightfieldformat.h"

// Rasterising a scenery's terrain triangles into a tiled heightfield.
//
// Both the standalone terraincook tool and the engine cook with this, which is the point of
// it being here: a heightfield baked on first run has to come out identical to one baked by
// the tool, and the only way to be sure of that is for there to be one implementation.
//
// The cooker knows nothing about where triangles come from. Feed it rasterize() per triangle
// and call finish(); the tool reads them out of scenery text, the engine hands them over as
// it parses.
namespace terrain {


// candidate grid steps tried when the step is not given; the coarsest one that still holds
// for practically every vertex names the grid (see detect_step).
// the range has to reach far past a corridor's metre grid: a regional elevation model
// is sampled every hundred metres, and a candidate list that stops at ten would report
// it as a ten metre grid, because every point of a 100 m grid sits on a 10 m one too
constexpr double gridsteps[] { 0.5, 1.0, 2.0, 2.5, 4.0, 5.0, 10.0, 20.0, 25.0, 50.0, 100.0, 200.0 };
// how far off a grid line a coordinate may sit and still count as on it
constexpr double gridepsilon { 0.01 };
// height difference at one grid point above which the samples are treated as a genuine
// conflict - two surfaces stacked over each other, which a heightfield cannot express
constexpr double conflictepsilon { 0.25 };
// share of sampled vertices a candidate step has to cover to count as the base grid
constexpr double gridthreshold { 99.0 };

struct vertex {
    double x, y, z;
    double u { 0.0 }, v { 0.0 }; // texture coordinates, kept to measure material scale
};

// length of the segment between two vertices as seen from above
inline double
planar_length( vertex const &From, vertex const &To ) {
    return std::sqrt( ( To.x - From.x ) * ( To.x - From.x ) + ( To.z - From.z ) * ( To.z - From.z ) );
}

// whether a coordinate lies on a grid of the given step
inline bool
on_grid( double const Value, double const Step ) {
    return std::fabs( Value / Step - std::round( Value / Step ) ) * Step <= gridepsilon;
}

// writes a greyscale preview. Values holds Width * Height samples, absent ones marked
// by the sentinel; those come out black
inline void
writepreview(
    std::filesystem::path const &Path,
    std::vector<float> const &Values,
    std::size_t const Width, std::size_t const Height,
    float const Sentinel ) {

    auto minimum { 1e30f }, maximum { -1e30f };
    for( auto const value : Values ) {
        if( value == Sentinel ) { continue; }
        minimum = std::min( minimum, value );
        maximum = std::max( maximum, value );
    }
    if( minimum > maximum ) { return; }
    auto const range { std::max( 1e-6f, maximum - minimum ) };

    std::ofstream file( Path, std::ios::binary );
    file << "P5\n" << Width << " " << Height << "\n255\n";
    std::vector<unsigned char> row( Width );
    for( auto y { 0u }; y < Height; ++y ) {
        for( auto x { 0u }; x < Width; ++x ) {
            auto const value { Values[ y * Width + x ] };
            row[ x ] = (
                value == Sentinel ?
                    0u :
                    static_cast<unsigned char>( 1.0f + 254.0f * ( value - minimum ) / range ) );
        }
        file.write( reinterpret_cast<char const *>( row.data() ), static_cast<std::streamsize>( Width ) );
    }
}

class cooker {

public:
    // samples along a tile side, shared edge excluded; with a 2 m grid this is a 256 m tile
    static constexpr std::uint32_t tilesamples { 128 };
    static constexpr std::uint32_t tilestride { heightfield::samples_at_level( tilesamples, 0 ) };
    // mip chain down to a 5 sample side, which is where a tile stops being worth its entry
    static constexpr std::uint32_t miplevels { heightfield::level_count( tilesamples, 5 ) };
    // triangles larger than this are not terrain: the water plane spans the whole map
    static constexpr double maxedge { 200.0 };
    // two surfaces closer than this are the same ground, one laid over the other, rather
    // than a deck carried above it. a kerb or a slab stands centimetres proud; a viaduct
    // stands metres
    static constexpr double surfaceepsilon { 0.5 };

    // stop printing the cooking report. the tool wants the tables; the engine, baking on
    // a first run, wants one line in its own log instead
    void
        quiet() { m_verbose = false; }

    void
        configure( double const Step, bool const Compress ) {
            m_step = Step;
            m_compress = Compress; }

    // triangle identity for representation tracking. callers that do not track it pass
    // nothing and get notriangle, which is never reported as represented
    static constexpr std::uint32_t notriangle { 0xffffffffu };

    void
        rasterize( vertex const &A, vertex const &B, vertex const &C, std::string_view const Material,
                   std::uint32_t const Triangle = notriangle ) {

            m_current = Triangle;
            if( Triangle != notriangle ) {
                if( Triangle >= m_triangleclaimed.size() ) {
                    auto const size { static_cast<std::size_t>( Triangle ) + 1u };
                    m_triangleclaimed.resize( size, 0u );
                    m_trianglekept.resize( size, 0u );
                    m_trianglerejected.resize( size, 0u );
                }
            }

            vertex const points[ 3 ] { A, B, C };

            for( auto index { 0u }; index < 3u; ++index ) {
                if( planar_length( points[ index ], points[ ( index + 1 ) % 3 ] ) > maxedge ) {
                    ++m_oversized;
                    reject();
                    return;
                }
            }

            // barycentric setup in the horizontal plane; a triangle standing on its edge
            // covers no samples and would divide by zero
            auto const area { ( B.x - A.x ) * ( C.z - A.z ) - ( C.x - A.x ) * ( B.z - A.z ) };
            if( std::abs( area ) < 1e-9 ) { ++m_degenerate; reject(); return; }
            auto const inversearea { 1.0 / area };
            // a palette slot is taken only by a material that reaches this far, so triangles
            // standing on their edge do not fill the palette
            auto const material { material_index( Material ) };
            measure_scale( material, points );

            auto const firstcolumn { static_cast<std::int64_t>( std::ceil( std::min( { A.x, B.x, C.x } ) / m_step ) ) };
            auto const lastcolumn { static_cast<std::int64_t>( std::floor( std::max( { A.x, B.x, C.x } ) / m_step ) ) };
            auto const firstrow { static_cast<std::int64_t>( std::ceil( std::min( { A.z, B.z, C.z } ) / m_step ) ) };
            auto const lastrow { static_cast<std::int64_t>( std::floor( std::max( { A.z, B.z, C.z } ) / m_step ) ) };

            std::size_t claimed { 0 };
            for( auto row { firstrow }; row <= lastrow; ++row ) {
                for( auto column { firstcolumn }; column <= lastcolumn; ++column ) {

                    auto const x { column * m_step };
                    auto const z { row * m_step };
                    auto const weightb { ( ( x - A.x ) * ( C.z - A.z ) - ( C.x - A.x ) * ( z - A.z ) ) * inversearea };
                    auto const weightc { ( ( B.x - A.x ) * ( z - A.z ) - ( x - A.x ) * ( B.z - A.z ) ) * inversearea };
                    auto const weighta { 1.0 - weightb - weightc };
                    constexpr double slack { -1e-9 };
                    if( ( weighta < slack ) || ( weightb < slack ) || ( weightc < slack ) ) { continue; }

                    ++claimed;
                    store( column, row, weighta * A.y + weightb * B.y + weightc * C.y, material );
                }
            }

            // every sample this triangle wanted, whether or not it ended up keeping it.
            // comparing this with what the finished grid holds says how much of a
            // material was lost to triangles stacked over it
            if( m_current != notriangle ) { m_triangleclaimed[ m_current ] += static_cast<std::uint32_t>( claimed ); }
            tally( m_materialclaimed, material, claimed );
            if( material >= m_materialwhere.size() ) { m_materialwhere.resize( material + 1u, { 0.0, 0.0, false } ); }
            if( false == m_materialwhere[ material ].seen ) {
                // one place the material was laid, so a report line can be looked at
                m_materialwhere[ material ] = { ( A.x + B.x + C.x ) / 3.0, ( A.z + B.z + C.z ) / 3.0, true };
            }
            if( claimed == 0 ) {
                // smaller than the gap between samples: it left no trace at all
                tally( m_materialmissed, material, 1 );
                ++m_toosmall;
            }
        }

    // whether the heightfield shows this triangle well enough for it not to be drawn on its
    // own: it was laid at all, it reached at least one sample, and it still holds nearly
    // every sample it wanted once everything else has been laid. valid after finish()
    bool
        represented( std::uint32_t const Triangle ) const {
            if( Triangle >= m_triangleclaimed.size() ) { return false; }
            if( m_trianglerejected[ Triangle ] != 0 ) { return false; }
            auto const claimed { m_triangleclaimed[ Triangle ] };
            if( claimed == 0 ) { return false; }
            return static_cast<double>( m_trianglekept[ Triangle ] ) >= representedshare * static_cast<double>( claimed ); }

    void
        finish( std::filesystem::path const &Path, std::filesystem::path const &Previewpath ) {

            if( true == m_tiles.empty() ) {
                report( "   nothing to cook\n" );
                return; }

            // every sample counted once, in the tile that owns it rather than in the
            // neighbours carrying a copy of the shared edge
            for( auto const & [ key, entry ] : m_tiles ) {
                for( std::uint32_t row = 0; row < tilesamples; ++row ) {
                    for( std::uint32_t column = 0; column < tilesamples; ++column ) {
                        auto const owner { entry.owner[ row * tilestride + column ] };
                        if( ( owner == notriangle ) || ( owner >= m_trianglekept.size() ) ) { continue; }
                        ++m_trianglekept[ owner ];
                        // the heightfield is drawn only between samples that all carry ground,
                        // so a triangle reaching past the last full cell leaves a strip nothing
                        // covers. one holding a sample next to a hole keeps being drawn itself
                        auto const globalcolumn { static_cast<std::int64_t>( entry.x ) * tilesamples + column };
                        auto const globalrow { static_cast<std::int64_t>( entry.z ) * tilesamples + row };
                        if( m_trianglerejected[ owner ] != 0 ) { continue; }
                        if( false == drawn_around( globalcolumn, globalrow ) ) {
                            m_trianglerejected[ owner ] = 1u;
                        }
                    }
                }
            }

            auto const scale { std::max( 1.0, m_maxheight - m_minheight ) / heightfield::heightrange };

            std::vector<tile const *> ordered;
            ordered.reserve( m_tiles.size() );
            for( auto const & [ key, entry ] : m_tiles ) { ordered.push_back( &entry ); }
            std::sort( ordered.begin(), ordered.end(),
                []( tile const *Left, tile const *Right ) {
                    return heightfield::morton( Left->x, Left->z ) < heightfield::morton( Right->x, Right->z ); } );

            std::ofstream file( Path, std::ios::binary );
            if( false == file.good() ) {
                report( "   cannot write %s\n", Path.string().c_str() );
                return; }

            heightfield::file_header header {};
            std::memcpy( header.magic, heightfield::magic, sizeof( header.magic ) );
            header.version = heightfield::version;
            header.tilesamples = tilesamples;
            header.gridstep = static_cast<float>( m_step );
            header.heightbias = static_cast<float>( m_minheight );
            header.heightscale = static_cast<float>( scale );
            header.tilecount = static_cast<std::uint32_t>( ordered.size() );
            header.materialcount = static_cast<std::uint32_t>( m_materials.size() );
            header.miplevels = miplevels;
            header.selection = heightfield::selection_rules;
            header.minx = header.minz = INT32_MAX;
            header.maxx = header.maxz = INT32_MIN;
            for( auto const *entry : ordered ) {
                header.minx = std::min( header.minx, entry->x ); header.maxx = std::max( header.maxx, entry->x );
                header.minz = std::min( header.minz, entry->z ); header.maxz = std::max( header.maxz, entry->z );
            }

            std::size_t materialbytes { 0 };
            for( auto const &name : m_materials ) {
                materialbytes += sizeof( std::uint16_t ) + name.size() + sizeof( float );
            }
            auto const directorybytes { ordered.size() * sizeof( heightfield::tile_entry ) };
            auto const payloadstart { sizeof( heightfield::file_header ) + directorybytes + materialbytes };

            // the payloads are encoded up front: the directory records where each one
            // lands and how large it ended up, so it cannot be written before they exist
            std::vector<heightfield::tile_entry> directory;
            std::vector<std::vector<std::uint8_t>> payloads;
            directory.reserve( ordered.size() );
            payloads.reserve( ordered.size() );

            auto offset { payloadstart };
            auto const plainsize { heightfield::tile_bytes( tilesamples, miplevels ) };
            std::vector<std::uint8_t> plain( plainsize );

            for( auto const *entry : ordered ) {

                encode( *entry, scale, plain );
                auto payload { m_compress ? compress( plain ) : plain };
                m_storedbytes += payload.size();
                m_plainbytes += plain.size();

                heightfield::tile_entry record {};
                record.x = entry->x;
                record.z = entry->z;
                record.offset = offset;
                record.storedsize = static_cast<std::uint32_t>( payload.size() );
                record.plainsize = static_cast<std::uint32_t>( plain.size() );
                record.minheight = entry->minheight;
                record.maxheight = entry->maxheight;
                record.coverage = static_cast<std::uint32_t>( entry->coverage );
                record.tilecodec = static_cast<std::uint16_t>(
                    m_compress ? heightfield::codec::zstd : heightfield::codec::raw );
                offset += payload.size();

                directory.push_back( record );
                payloads.push_back( std::move( payload ) );
            }

            file.write( reinterpret_cast<char const *>( &header ), sizeof( header ) );
            file.write( reinterpret_cast<char const *>( directory.data() ), static_cast<std::streamsize>( directorybytes ) );
            for( std::size_t index = 0; index < m_materials.size(); ++index ) {
                auto const &name { m_materials[ index ] };
                std::uint16_t const length { static_cast<std::uint16_t>( name.size() ) };
                file.write( reinterpret_cast<char const *>( &length ), sizeof( length ) );
                file.write( name.data(), static_cast<std::streamsize>( name.size() ) );
                auto const scale { static_cast<float>( material_scale( index ) ) };
                file.write( reinterpret_cast<char const *>( &scale ), sizeof( scale ) );
            }
            for( auto const &payload : payloads ) {
                file.write( reinterpret_cast<char const *>( payload.data() ), static_cast<std::streamsize>( payload.size() ) );
            }

            file.close();

            report( Path, header, offset );
            if( false == Previewpath.empty() ) {
                writecookedpreview( Previewpath, header );
            }
        }

private:
    struct tile {
        std::int32_t x { 0 }, z { 0 };
        std::vector<float> height;
        std::vector<std::uint8_t> material;
        std::vector<std::uint8_t> covered;
        // which triangle the sample came from, so that once everything is laid each
        // triangle can be asked whether the heightfield still shows it
        std::vector<std::uint32_t> owner;
        float minheight { 1e30f }, maxheight { -1e30f };
        std::size_t coverage { 0 };
    };

    // the palette slot of a material, allocated on first use. looked up by view, so a
    // material already known costs no allocation
    std::uint8_t
        material_index( std::string_view const Name ) {
            auto const lookup { m_materialindex.find( Name ) };
            if( lookup != m_materialindex.end() ) { return lookup->second; }
            if( m_materials.size() >= 255 ) { ++m_materialoverflow; return 255; }
            auto const index { static_cast<std::uint8_t>( m_materials.size() ) };
            m_materials.emplace_back( Name );
            m_materialindex.emplace( std::string { Name }, index );
            return index; }

    // the source carries texture coordinates, so the tiling of every material can be
    // measured instead of guessed: the ratio of world distance to texture distance along
    // a triangle edge is how many metres one repeat covers. edges too short or with no
    // texture movement say nothing and are left out
    void
        measure_scale( std::uint8_t const Material, vertex const ( &Points )[ 3 ] ) {

            if( Material >= m_scalesums.size() ) {
                m_scalesums.resize( Material + 1, 0.0 );
                m_scalesquares.resize( Material + 1, 0.0 );
                m_scalecounts.resize( Material + 1, 0 );
            }
            for( auto index { 0u }; index < 3u; ++index ) {
                auto const &from { Points[ index ] };
                auto const &to { Points[ ( index + 1 ) % 3 ] };
                auto const world { planar_length( from, to ) };
                auto const texture {
                    std::sqrt( ( to.u - from.u ) * ( to.u - from.u ) + ( to.v - from.v ) * ( to.v - from.v ) ) };
                if( ( world < 0.5 ) || ( texture < 1e-4 ) ) { continue; }
                auto const ratio { world / texture };
                m_scalesums[ Material ] += ratio;
                m_scalesquares[ Material ] += ratio * ratio;
                ++m_scalecounts[ Material ];
            }
        }

    void
        store(
            std::int64_t const Column, std::int64_t const Row, double const Height,
            std::uint8_t const Material ) {

            // a sample on a tile boundary belongs to both neighbours. writing it to each
            // of them is what makes the shared edge identical rather than merely close
            auto const basex { static_cast<std::int32_t>( heightfield::floor_div( Column, tilesamples ) ) };
            auto const basez { static_cast<std::int32_t>( heightfield::floor_div( Row, tilesamples ) ) };
            auto const onxedge { Column == static_cast<std::int64_t>( basex ) * tilesamples };
            auto const onzedge { Row == static_cast<std::int64_t>( basez ) * tilesamples };

            for( auto duplicate { 0u }; duplicate < 4u; ++duplicate ) {

                if( ( ( duplicate & 1u ) != 0 ) && ( false == onxedge ) ) { continue; }
                if( ( ( duplicate & 2u ) != 0 ) && ( false == onzedge ) ) { continue; }

                auto const tilex { basex - ( ( duplicate & 1u ) != 0 ? 1 : 0 ) };
                auto const tilez { basez - ( ( duplicate & 2u ) != 0 ? 1 : 0 ) };
                auto const column { static_cast<std::size_t>( Column - static_cast<std::int64_t>( tilex ) * tilesamples ) };
                auto const row { static_cast<std::size_t>( Row - static_cast<std::int64_t>( tilez ) * tilesamples ) };
                if( ( column >= tilestride ) || ( row >= tilestride ) ) { continue; }

                auto &target { tileat( tilex, tilez ) };
                auto const index { row * tilestride + column };
                if( target.covered[ index ] == 0 ) {
                    target.covered[ index ] = 1;
                    target.height[ index ] = static_cast<float>( Height );
                    target.material[ index ] = Material;
                    target.owner[ index ] = m_current;
                    ++target.coverage;
                    ++m_covered;
                }
                else {
                    auto const difference { Height - static_cast<double>( target.height[ index ] ) };
                    if( std::abs( difference ) > conflictepsilon ) { ++m_stacked; }

                    if( std::abs( difference ) <= surfaceepsilon ) {
                        // near enough to be the same ground: a slab, a path, a patch of
                        // gravel laid over it. what is laid on top is what one sees, so
                        // the upper surface takes the sample. comparing heights rather
                        // than arrival order keeps the result independent of file order
                        if( difference > 0.0 ) {
                            target.height[ index ] = static_cast<float>( Height );
                            target.material[ index ] = Material;
                            target.owner[ index ] = m_current;
                        }
                        else if( ( std::abs( difference ) <= coincident )
                              && ( duplicate == 0u )
                              && ( m_current != notriangle ) ) {
                            // two triangles meeting along an edge both claim the samples on
                            // it, at the same height. the one that arrived second shows there
                            // just as much as the first, so it is credited with the sample
                            ++m_trianglekept[ m_current ];
                        }
                    }
                    else if( difference < 0.0 ) {
                        // far apart: one of them is carried above the ground on a
                        // structure, and the ground is the lower of the two
                        target.height[ index ] = static_cast<float>( Height );
                        target.material[ index ] = Material;
                        target.owner[ index ] = m_current;
                    }
                }
                target.minheight = std::min( target.minheight, static_cast<float>( Height ) );
                target.maxheight = std::max( target.maxheight, static_cast<float>( Height ) );
            }
            m_minheight = std::min( m_minheight, Height );
            m_maxheight = std::max( m_maxheight, Height );
        }

    // the tile at the given coordinates, created on first use. the last one returned is
    // remembered: consecutive samples nearly always fall in the same tile, and a map's
    // elements stay where they are when it grows
    tile &
        tileat( std::int32_t const X, std::int32_t const Z ) {
            auto const key { heightfield::tile_key( X, Z ) };
            if( ( m_lasttile != nullptr ) && ( m_lasttilekey == key ) ) { return *m_lasttile; }
            m_lasttilekey = key;
            auto const lookup { m_tiles.find( key ) };
            if( lookup != m_tiles.end() ) { return *( m_lasttile = &lookup->second ); }
            tile fresh;
            fresh.x = X;
            fresh.z = Z;
            fresh.height.assign( tilestride * tilestride, 0.0f );
            fresh.material.assign( tilestride * tilestride, 0u );
            fresh.covered.assign( tilestride * tilestride, 0u );
            fresh.owner.assign( tilestride * tilestride, notriangle );
            return *( m_lasttile = &m_tiles.emplace( key, std::move( fresh ) ).first->second ); }

    // lays the mip chain of one tile into the buffer, finest level first.
    // coarse levels are point sampled rather than averaged, deliberately: every level
    // keeps the samples that sit on the tile boundary exactly as its neighbour has them,
    // so the seam between two tiles stays identical at every level instead of needing
    // skirts to hide a mismatch. the cost is aliasing on distant terrain
    void
        encode( tile const &Tile, double const Scale, std::vector<std::uint8_t> &Buffer ) const {

            auto *cursor { Buffer.data() };
            for( auto level { 0u }; level < miplevels; ++level ) {

                auto const side { heightfield::samples_at_level( tilesamples, level ) };
                auto const stepping { std::size_t { 1 } << level };
                auto *heights { reinterpret_cast<std::uint16_t *>( cursor ) };
                auto *materials { cursor + static_cast<std::size_t>( side ) * side * sizeof( std::uint16_t ) };

                for( auto row { 0u }; row < side; ++row ) {
                    for( auto column { 0u }; column < side; ++column ) {
                        auto const source { ( row * stepping ) * tilestride + ( column * stepping ) };
                        auto const target { static_cast<std::size_t>( row ) * side + column };
                        heights[ target ] = (
                            Tile.covered[ source ] == 0 ?
                                heightfield::nodata :
                                static_cast<std::uint16_t>( ( Tile.height[ source ] - m_minheight ) / Scale ) );
                        materials[ target ] = Tile.material[ source ];
                    }
                }
                cursor += heightfield::level_bytes( tilesamples, level );
            }
        }

    std::vector<std::uint8_t>
        compress( std::vector<std::uint8_t> const &Plain ) {

            // one context for the whole cook: creating one per tile costs more than a tile
            if( m_context == nullptr ) { m_context.reset( ZSTD_createCCtx() ); }
            std::vector<std::uint8_t> encoded( ZSTD_compressBound( Plain.size() ) );
            auto const written {
                ZSTD_compressCCtx( m_context.get(), encoded.data(), encoded.size(), Plain.data(), Plain.size(), compressionlevel ) };
            if( 0 != ZSTD_isError( written ) ) {
                // a tile that will not compress is still a valid tile; it just stays raw
                return Plain;
            }
            encoded.resize( written );
            return encoded;
        }

    void
        report(
            std::filesystem::path const &Path,
            heightfield::file_header const &Header,
            std::size_t const Bytes ) const {

            auto const tilearea {
                ( static_cast<std::int64_t>( Header.maxx ) - Header.minx + 1 )
              * ( static_cast<std::int64_t>( Header.maxz ) - Header.minz + 1 ) };
            auto const storedsamples {
                static_cast<std::size_t>( Header.tilecount ) * tilestride * tilestride };

            report( "\n-- cooked heightfield\n" );
            report( "   grid step            %12.2f m\n", m_step );
            report( "   tiles                %12u  (%u x %u samples, %.0f m each)\n",
                Header.tilecount, tilestride, tilestride, tilesamples * m_step );
            report( "   mip levels           %12u  (finest %u, coarsest %u samples)\n",
                miplevels, tilestride, heightfield::samples_at_level( tilesamples, miplevels - 1 ) );
            report( "   tile span            %12d .. %d x   %d .. %d z\n",
                Header.minx, Header.maxx, Header.minz, Header.maxz );
            report( "   occupancy            %12.1f %%  of the tile bounding box\n",
                100.0 * static_cast<double>( Header.tilecount ) / static_cast<double>( tilearea ) );
            report( "   samples with data    %12zu  %6.2f %% of the finest level\n",
                m_covered, 100.0 * static_cast<double>( m_covered ) / static_cast<double>( storedsamples ) );
            report( "   height range         %12.2f .. %.2f m  (%.4f m per unit)\n",
                m_minheight, m_maxheight, Header.heightscale );
            report( "   materials            %12zu%s\n", m_materials.size(),
                m_materialoverflow > 0 ? "  (palette full, some triangles share the last slot)" : "" );
            auto const coverage { material_coverage() };
            report( "\n-- what survived, per material\n" );
            report( "     %-28s %10s %10s %8s %8s  %s\n", "material", "claimed", "kept", "kept %", "vanished", "seen at x,z" );
            for( std::size_t index = 0; index < m_materials.size(); ++index ) {
                auto const claimed { index < m_materialclaimed.size() ? m_materialclaimed[ index ] : 0 };
                auto const kept { index < coverage.size() ? coverage[ index ] : 0 };
                auto const missed { index < m_materialmissed.size() ? m_materialmissed[ index ] : 0 };
                auto const where { index < m_materialwhere.size() ? m_materialwhere[ index ] : place{} };
                report( "     %-28s %10zu %10zu %7.1f %% %8zu  %.0f,%.0f\n",
                    m_materials[ index ].c_str(), claimed, kept,
                    claimed == 0 ? 0.0 : 100.0 * static_cast<double>( kept ) / static_cast<double>( claimed ),
                    missed, where.x, where.z );
            }
            report( "   triangles too small to reach a sample: %zu\n", m_toosmall );

            report( "\n-- material tiling\n" );
            for( std::size_t index = 0; index < m_materials.size(); ++index ) {
                report( "     %-28s %6.2f m per repeat, spread %5.1f %%%s\n",
                    m_materials[ index ].c_str(), material_scale( index ), 100.0 * scale_spread( index ),
                    ( index < m_scalecounts.size() ) && ( m_scalecounts[ index ] > 0 ) ? "" : "  (assumed)" );
            }
            report( "   oversized skipped    %12zu  (edges over %.0f m, the water plane)\n", m_oversized, maxedge );
            report( "   degenerate skipped   %12zu\n", m_degenerate );
            report( "   stacked samples      %12zu  (surfaces more than %.2f m apart)\n", m_stacked, conflictepsilon );
            if( true == m_compress ) {
                report( "   payload              %12.1f MB from %.1f MB  (%.2fx, zstd level %d)\n",
                    m_storedbytes / 1048576.0, m_plainbytes / 1048576.0,
                    m_plainbytes > 0 ? static_cast<double>( m_plainbytes ) / static_cast<double>( m_storedbytes ) : 0.0,
                    compressionlevel );
            }
            report( "   file size            %12.1f MB  -> %s\n", Bytes / 1048576.0, Path.string().c_str() );
        }

    void
        writecookedpreview( std::filesystem::path const &Path, heightfield::file_header const &Header ) const {

            auto const width { static_cast<std::size_t>( Header.maxx - Header.minx + 1 ) };
            auto const height { static_cast<std::size_t>( Header.maxz - Header.minz + 1 ) };
            // one preview pixel per coarsest mip sample, capped so the image stays viewable
            auto pixelspertile { heightfield::samples_at_level( tilesamples, miplevels - 1 ) - 1 };
            while( ( pixelspertile > 1 ) && ( std::max( width, height ) * pixelspertile > 2048 ) ) { pixelspertile /= 2; }
            auto const previewwidth { width * pixelspertile };
            auto const previewheight { height * pixelspertile };
            auto const samplesper { tilesamples / pixelspertile };
            constexpr float sentinel { -1e30f };
            std::vector<float> preview( previewwidth * previewheight, sentinel );

            for( auto const & [ key, tile ] : m_tiles ) {
                for( auto y { 0u }; y < pixelspertile; ++y ) {
                    for( auto x { 0u }; x < pixelspertile; ++x ) {
                        auto sum { 0.0 };
                        std::size_t count { 0 };
                        for( auto sy { 0u }; sy < samplesper; ++sy ) {
                            for( auto sx { 0u }; sx < samplesper; ++sx ) {
                                auto const index { ( y * samplesper + sy ) * tilestride + ( x * samplesper + sx ) };
                                if( tile.covered[ index ] == 0 ) { continue; }
                                sum += tile.height[ index ];
                                ++count;
                            }
                        }
                        if( count == 0 ) { continue; }
                        auto const column { static_cast<std::size_t>( tile.x - Header.minx ) * pixelspertile + x };
                        auto const row { static_cast<std::size_t>( tile.z - Header.minz ) * pixelspertile + y };
                        if( ( column >= previewwidth ) || ( row >= previewheight ) ) { continue; }
                        preview[ row * previewwidth + column ] = static_cast<float>( sum / static_cast<double>( count ) );
                    }
                }
            }
            writepreview( Path, preview, previewwidth, previewheight, sentinel );
        }

    // metres of ground per texture repeat; materials the measurement never caught fall
    // back to something plausible rather than to zero, which would divide in the shader
    double
        material_scale( std::size_t const Index ) const {
            if( ( Index >= m_scalecounts.size() ) || ( m_scalecounts[ Index ] == 0 ) ) { return 4.0; }
            return m_scalesums[ Index ] / static_cast<double>( m_scalecounts[ Index ] ); }

    // how much the measured tiling varies within one material, relative to its mean.
    // a single global scale per texture is only faithful where this is small; a material
    // the source scaled differently from place to place cannot be served by one number
    double
        scale_spread( std::size_t const Index ) const {
            if( ( Index >= m_scalecounts.size() ) || ( m_scalecounts[ Index ] < 2 ) ) { return 0.0; }
            auto const count { static_cast<double>( m_scalecounts[ Index ] ) };
            auto const mean { m_scalesums[ Index ] / count };
            auto const variance { std::max( 0.0, m_scalesquares[ Index ] / count - mean * mean ) };
            return mean > 1e-6 ? std::sqrt( variance ) / mean : 0.0; }

    static void
        tally( std::vector<std::size_t> &Counters, std::uint8_t const Material, std::size_t const Amount ) {
            if( Material >= Counters.size() ) { Counters.resize( Material + 1, 0 ); }
            Counters[ Material ] += Amount; }

    // what the finished grid actually holds, counted once the last triangle is in
    std::vector<std::size_t>
        material_coverage() const {
            std::vector<std::size_t> counters( m_materials.size(), 0 );
            for( auto const & [ key, tile ] : m_tiles ) {
                // only the samples a tile owns, so the shared edge is not counted twice
                for( std::uint32_t row = 0; row < tilesamples; ++row ) {
                    for( std::uint32_t column = 0; column < tilesamples; ++column ) {
                        auto const index { row * tilestride + column };
                        if( tile.covered[ index ] == 0 ) { continue; }
                        auto const material { tile.material[ index ] };
                        if( material < counters.size() ) { ++counters[ material ]; }
                    }
                }
            }
            return counters; }

    template <typename... Arguments_>
    void
        report( char const *Format, Arguments_... Values ) const {
            if( true == m_verbose ) { std::printf( Format, Values... ); } }

    static constexpr int compressionlevel { 9 };
    bool m_verbose { true };

    double m_step { 2.0 };
    bool m_compress { true };
    std::unordered_map<std::int64_t, tile> m_tiles;
    std::vector<std::string> m_materials;
    // looked up by string_view, so finding a material does not build a string
    struct name_hash {
        using is_transparent = void;
        std::size_t operator()( std::string_view const Name ) const { return std::hash<std::string_view>{}( Name ); } };
    std::unordered_map<std::string, std::uint8_t, name_hash, std::equal_to<>> m_materialindex;
    double m_minheight { 1e30 }, m_maxheight { -1e30 };
    std::size_t m_covered { 0 }, m_stacked { 0 }, m_oversized { 0 }, m_degenerate { 0 };
    std::size_t m_materialoverflow { 0 };

    // whether the sample at a grid position carries ground, wherever it is stored. the last
    // tile looked in is remembered, since neighbouring samples nearly always share one
    bool
        covered_at( std::int64_t const Column, std::int64_t const Row ) const {
            auto const tilex { static_cast<std::int32_t>( heightfield::floor_div( Column, tilesamples ) ) };
            auto const tilez { static_cast<std::int32_t>( heightfield::floor_div( Row, tilesamples ) ) };
            auto const tilekey { heightfield::tile_key( tilex, tilez ) };
            if( ( m_lastcovered == nullptr ) || ( m_lastcoveredkey != tilekey ) ) {
                auto const lookup { m_tiles.find( tilekey ) };
                if( lookup == m_tiles.end() ) { return false; }
                m_lastcovered = &lookup->second;
                m_lastcoveredkey = tilekey;
            }
            auto const column { static_cast<std::size_t>( Column - static_cast<std::int64_t>( tilex ) * tilesamples ) };
            auto const row { static_cast<std::size_t>( Row - static_cast<std::int64_t>( tilez ) * tilesamples ) };
            return m_lastcovered->covered[ row * tilestride + column ] != 0; }

    // whether the ground around a sample is drawn at every level a triangle node is likely to
    // be seen at. the heightfield draws a cell only when all four of its corners carry ground,
    // and a coarser level has larger cells, so a sample can be drawn up close and fall into a
    // hole further out. corners are checked on every cell touching the sample, diagonals too
    bool
        drawn_around( std::int64_t const Column, std::int64_t const Row ) const {
            for( std::uint32_t level = 0; level < checkedlevels; ++level ) {
                auto const stride { static_cast<std::int64_t>( 1 ) << level };
                auto const basecolumn { heightfield::floor_div( Column, stride ) * stride };
                auto const baserow { heightfield::floor_div( Row, stride ) * stride };
                // a sample on a cell line touches the cells either side of it
                auto const firstcolumn { basecolumn == Column ? basecolumn - stride : basecolumn };
                auto const firstrow { baserow == Row ? baserow - stride : baserow };
                for( auto row { firstrow }; row <= baserow + stride; row += stride ) {
                    for( auto column { firstcolumn }; column <= basecolumn + stride; column += stride ) {
                        if( false == covered_at( column, row ) ) { return false; }
                    }
                }
            }
            return true; }

    void
        reject() {
            if( m_current != notriangle ) { m_trianglerejected[ m_current ] = 1u; } }

    // levels up to 2048 m, the farthest a triangle node of the terrain is normally drawn
    static constexpr std::uint32_t checkedlevels { 3 };
    mutable tile const *m_lastcovered { nullptr };
    mutable std::int64_t m_lastcoveredkey { 0 };

    // heights this close are one surface rather than two
    static constexpr double coincident { 0.05 };
    // share of its samples a triangle has to keep to count as shown by the heightfield
    static constexpr double representedshare { 0.9 };

    std::uint32_t m_current { notriangle };
    std::vector<std::uint32_t> m_triangleclaimed, m_trianglekept;
    std::vector<std::uint8_t> m_trianglerejected;
    std::vector<std::size_t> m_materialclaimed, m_materialmissed;
    struct place { double x { 0.0 }, z { 0.0 }; bool seen { false }; };
    std::vector<place> m_materialwhere;
    std::size_t m_toosmall { 0 };
    std::vector<double> m_scalesums, m_scalesquares;
    std::vector<std::size_t> m_scalecounts;
    std::size_t m_storedbytes { 0 }, m_plainbytes { 0 };
    tile *m_lasttile { nullptr };
    std::int64_t m_lasttilekey { 0 };
    struct context_release {
        void operator()( ZSTD_CCtx *Context ) const { ZSTD_freeCCtx( Context ); } };
    std::unique_ptr<ZSTD_CCtx, context_release> m_context;
};

// the coarsest candidate step that still holds for practically every sampled vertex.
// every vertex sitting on a 2 m grid also sits on a 1 m and a 0.5 m one, so the finest
// candidate always scores best and says nothing - it is the coarsest that names the grid
// Report is called with each candidate and the share of vertices on it, for a caller that
// wants to show the working
template <typename Sample_, typename Report_>
double
detect_step( Sample_ const &Sample, Report_ &&Report ) {

    auto best { 0.0 };
    for( auto const candidate : gridsteps ) {
        std::size_t hits { 0 };
        for( auto const &point : Sample ) {
            if( on_grid( point.x, candidate ) && on_grid( point.z, candidate ) ) { ++hits; }
        }
        auto const share { std::empty( Sample ) ? 0.0 : 100.0 * static_cast<double>( hits ) / static_cast<double>( std::size( Sample ) ) };
        Report( candidate, share );
        if( share >= gridthreshold ) { best = candidate; }
    }
    return ( best > 0.0 ? best : 1.0 );
}

template <typename Sample_>
double
detect_step( Sample_ const &Sample ) {
    return detect_step( Sample, []( double, double ) {} );
}

} // namespace terrain
