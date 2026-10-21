/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"

#include "world/TrackSleepers.h"
#include "world/SleeperGeometry.h"
#include "world/SleeperLayout.h"
#include "world/SleeperPaths.h"
#include "world/SleeperRules.h"
#include "world/SleeperWorker.h"
#include "world/Track.h"
#include "simulation/simulation.h"
#include "scene/scene.h"
#include "utilities/utilities.h"

#include <glm/gtc/matrix_transform.hpp>

#include <map>
#include <set>

namespace track_sleepers {

bool bedded( TTrack const &Track ) {

    if( ( Track.iCategoryFlag & 15 ) != 1 || Track.m_editorremoved ) {
        return false;
    }
    switch( Track.eType ) {
        case tt_Normal: {
            if( Track.m_material2 == null_handle || Track.Segment == nullptr ) {
                return false;
            }
            break;
        }
        case tt_Switch: {
            // the switch trackbed takes the ballast of a plain track joined to it
            auto const ballast = [&]( TTrack const *Other ) {
                return Other != nullptr && Other->eType == tt_Normal && Other->m_material2 != null_handle; };
            if( false == Global.CreateSwitchTrackbeds || Track.SwitchExtension == nullptr
             || ( Track.SwitchExtension->m_material3 == null_handle && false == ballast( Track.trPrev ) && false == ballast( Track.trNext ) ) ) {
                return false;
            }
            break;
        }
        default: {
            return false;
        }
    }
    return true;
}

bool lays( TTrack const &Track ) {

    // the model of the track itself (timbermodel) is enough, without sleepers3d in eu07.ini
    return bedded( Track ) && timber_model( Track ) != nullptr;
}

std::vector<std::pair<double, double>> shared_zones( TTrack const &Track, int const Path ) {

    std::vector<std::pair<double, double>> result;
    if( false == bedded( Track ) || Path < 0 || Path >= path_count( Track ) ) {
        return result;
    }
    auto const around { paths_around( const_cast<TTrack &>( Track ), true ) };
    auto const found { std::find_if( around.begin(), around.end(), [&]( sampled_path const &Sampled ) { return Sampled.track == &Track && Sampled.path == Path; } ) };
    if( found == around.end() ) {
        return result;
    }
    auto const &path { *found };
    // where another path runs so near that the timbers carry both
    for( auto const &stretch : stretches_where( path, [&]( glm::dvec3 const &Point, glm::dvec2 const &Direction ) {
            return std::any_of( around.begin(), around.end(), [&]( sampled_path const &Other ) {
                return &Other != &path && shares_with( path, Point, Direction, Other ); } ); } ) ) {
        result.emplace_back( stretch.x, stretch.y );
    }
    return result;
}

std::pair<double, double> shared_zone( TTrack const &Track, int const Path ) {

    auto const zones { shared_zones( Track, Path ) };
    if( zones.empty() ) {
        return { 0.0, 0.0 };
    }
    return { zones.front().first, zones.back().second };
}

void lay( TTrack &Track ) {

    auto &laid { Track.m_sleepers3d };
    std::vector<beam> placed;
    if( laid.active && in_background() && timber_model( Track ) != nullptr ) {
        // worked out in the background; till then the track keeps what it has, and tries again
        auto beams { laid_out( Track ) };
        if( false == beams.has_value() ) {
            ask_layout( Track );
            return;
        }
        placed = std::move( *beams );
    }
    laid.dirty = false;
    for( auto &instances : laid.instances ) {
        instances.clear();
    }
    laid.whole.clear();
    laid.plain.clear();
    laid.beams.clear();
    laid.chunks.clear();
    laid.timbers = timber_model( Track );
    laid.plains = plain_model( Track );
    if( false == laid.active || laid.timbers == nullptr ) {
        return;
    }
    auto const &parts { *laid.timbers };
    if( auto const panel { panel_off_plane( Track ) }; panel.first > coplanar_tolerance ) {
        WriteLog(
            "sleepers3d: switch \"" + Track.name() + "\" has its paths " + to_string( panel.first * 1000.0, 1 ) + " mm off one plane "
            + to_string( panel.second, 2 ) + " m along, where they share the timbers of its panel" );
    }
    laid.laying = true;
    laid.beams = ( in_background() ? std::move( placed ) : place( Track ) );
    laid.laying = false;
    auto const &beams { laid.beams };

    auto const partlength = [&]( int const Part ) {
        return static_cast<double>( parts.spans[ Part ].y - parts.spans[ Part ].x ); };
    auto const beginlength { partlength( part_begin ) };
    auto const mainlength { std::max( 0.01, partlength( part_main ) ) };
    auto const endlength { partlength( part_end ) };

    chunk run;
    std::vector<std::pair<glm::dvec3, double>> runbeams;
    auto const close_run = [&]() {
        if( runbeams.empty() ) {
            return;
        }
        glm::dvec3 centre { 0.0 };
        for( auto const &runbeam : runbeams ) {
            centre += runbeam.first;
        }
        centre /= static_cast<double>( runbeams.size() );
        auto radius { 0.0 };
        for( auto const &runbeam : runbeams ) {
            radius = std::max( radius, glm::length( runbeam.first - centre ) + runbeam.second );
        }
        run.centre = glm::vec3 { centre };
        run.radius = static_cast<float>( radius + 0.3 );
        for( int part = 0; part < part_count; ++part ) {
            run.last[ part ] = static_cast<std::uint32_t>( laid.instances[ part ].size() );
        }
        run.wholelast = static_cast<std::uint32_t>( laid.whole.size() );
        run.plainlast = static_cast<std::uint32_t>( laid.plain.size() );
        laid.chunks.push_back( run );
        runbeams.clear();
    };

    for( auto const &beam : beams ) {
        if( runbeams.empty() ) {
            for( int part = 0; part < part_count; ++part ) {
                run.first[ part ] = static_cast<std::uint32_t>( laid.instances[ part ].size() );
            }
            run.wholefirst = static_cast<std::uint32_t>( laid.whole.size() );
            run.plainfirst = static_cast<std::uint32_t>( laid.plain.size() );
        }
        auto const length { glm::length( beam.to - beam.from ) };
        if( length < 0.1 ) {
            continue;
        }
        // the frame of the beam: x along it, y up, z along the tracks
        glm::dvec3 const x { ( beam.to - beam.from ) / length };
        auto z { glm::normalize( beam.along - x * glm::dot( beam.along, x ) ) };
        auto y { glm::cross( z, x ) };
        if( y.y < 0.0 ) {
            z = -z;
            y = -y;
        }
        auto const middle { 0.5 * ( beam.from + beam.to ) - Track.m_origin };
        glm::mat4 frame { 1.f };
        frame[ 0 ] = glm::vec4 { glm::vec3 { x }, 0.f };
        frame[ 1 ] = glm::vec4 { glm::vec3 { y }, 0.f };
        frame[ 2 ] = glm::vec4 { glm::vec3 { z }, 0.f };
        frame[ 3 ] = glm::vec4 { glm::vec3 { middle }, 1.f };
        // a part stretched along x by the scale, and moved so its x lands at the given place along the beam
        auto const put = [&]( int const Part, double const Scale, double const From, double const Place ) {
            auto const shift { Place - Scale * From };
            laid.instances[ Part ].emplace_back(
                glm::scale(
                    glm::translate( frame, glm::vec3 { static_cast<float>( shift ), -parts.seat, 0.f } ),
                    glm::vec3 { static_cast<float>( Scale ), 1.f, 1.f } ) );
        };
        auto const half { 0.5 * length };
        auto const between { length - beginlength - endlength };
        if( beam.plain && laid.plains != nullptr ) {
            // the whole sleeper at its own length, its middle under the middle of the track
            auto const &plain { *laid.plains };
            laid.plain.emplace_back(
                glm::translate( frame, glm::vec3 { -0.5f * ( plain.span.x + plain.span.y ), -plain.seat, 0.f } ) );
        }
        else if( false == parts.stretchable ) {
            // a timber of a model which can't be put together, the whole of it stretched to the length
            auto const scale { length / static_cast<double>( parts.length() ) };
            laid.whole.emplace_back(
                glm::scale(
                    glm::translate( frame, glm::vec3 { static_cast<float>( -scale * 0.5 * ( parts.span.x + parts.span.y ) ), -parts.seat, 0.f } ),
                    glm::vec3 { static_cast<float>( scale ), 1.f, 1.f } ) );
        }
        else if( between > 0.01 ) {
            // the ends at their own size, and as many pieces between them as fit closest to their own size
            put( part_end, 1.0, parts.spans[ part_end ].x, -half );
            put( part_begin, 1.0, parts.spans[ part_begin ].y, half );
            auto const pieces { std::max( 1, static_cast<int>( std::round( between / mainlength ) ) ) };
            auto const piece { between / pieces };
            for( int index = 0; index < pieces; ++index ) {
                put( part_main, piece / mainlength, parts.spans[ part_main ].x, -half + endlength + index * piece );
            }
        }
        else {
            // a beam shorter than its ends: just the ends, squeezed
            auto const scale { length / std::max( 0.01, beginlength + endlength ) };
            put( part_end, scale, parts.spans[ part_end ].x, -half );
            put( part_begin, scale, parts.spans[ part_begin ].y, half );
        }
        runbeams.emplace_back( middle, half );
        if( runbeams.size() >= chunk_beams ) {
            close_run();
        }
    }
    close_run();
}

layout_check check() {

    layout_check result;
    result.digest = 1469598103934665603ull;
    settle_background();
    faults() = {};
    // laid anew, for the counts of what's wrong with them
    outdate_layouts();
    // the beams of all the tracks in plan, as boxes, with the grid of where they are
    std::vector<beam_box> boxes;
    std::vector<TTrack const *> owners;
    std::vector<TTrack *> tracks;
    for( auto *track : simulation::Paths.sequence() ) {
        if( track == nullptr || track->m_editorremoved || false == track->m_sleepers3d.active ) {
            continue;
        }
        tracks.push_back( track );
        if( auto const panel { panel_off_plane( *track ) }; panel.first > coplanar_tolerance ) {
            ++result.panels;
            if( result.panels <= 4 ) result.examples.push_back(
                "switch " + track->name() + " " + to_string( panel.first * 1000.0, 1 ) + " mm off one plane at " + to_string( panel.second, 2 ) + " m" );
        }
        for( auto const &beam : place( *track ) ) {
            for( auto const &vector : { beam.from, beam.to, beam.along } ) {
                for( int axis = 0; axis < 3; ++axis ) {
                    result.digest = ( result.digest ^ static_cast<std::uint64_t>( std::llround( vector[ axis ] * 1e4 ) ) ) * 1099511628211ull;
                }
            }
            result.digest = ( result.digest ^ ( beam.plain ? 1u : 2u ) ) * 1099511628211ull;
            auto const span { plan( beam.to - beam.from ) };
            auto const length { glm::length( span ) };
            if( length < 0.1 ) {
                continue;
            }
            boxes.push_back( box_of( beam ) );
            owners.push_back( track );
            result.longest = std::max( result.longest, length );
            if( length > length_cap ) {
                ++result.toolong;
            }
        }
    }
    result.beams = boxes.size();
    result.offplane = faults().offplane;
    result.clipped = faults().clipped;
    result.dropped = faults().dropped;
    result.slowest = faults().slowest;
    result.biggest = faults().biggest;
    // each box in every cell its span touches
    plan_grid<std::size_t> grid { 3.0 };
    for( std::size_t index = 0; index < boxes.size(); ++index ) {
        grid.insert( boxes[ index ].centre, boxes[ index ].half + half_width, index );
    }
    std::set<std::pair<std::size_t, std::size_t>> overlaps;
    for( auto const &cell : grid.cells() ) {
        auto const &indices { cell.second };
        for( std::size_t left = 0; left < indices.size(); ++left ) {
            for( std::size_t right = left + 1; right < indices.size(); ++right ) {
                auto const pair { std::minmax( indices[ left ], indices[ right ] ) };
                if( overlaps.count( pair ) || apart( boxes[ pair.first ], boxes[ pair.second ] ) ) {
                    continue;
                }
                overlaps.insert( pair );
                if( overlaps.size() <= 16 ) {
                    auto const &box { boxes[ pair.first ] };
                    result.examples.push_back(
                        "overlap " + owners[ pair.first ]->name() + "/" + owners[ pair.second ]->name() + " at " + to_string( box.centre.x, 2 ) + " " + to_string( box.centre.y, 2 ) );
                }
            }
        }
    }
    result.overlaps = overlaps.size();
    // the rails of the tracks, followed for how far apart the beams under them are
    for( auto *track : tracks ) {
        for( int path = 0; path < path_count( *track ); ++path ) {
            auto const *segment { path_segment( *track, path ) };
            if( segment == nullptr ) {
                continue;
            }
            auto const length { segment->GetLength() };
            auto const steps { static_cast<int>( std::ceil( length / 0.05 ) ) };
            for( auto const side : { -1.0, 1.0 } ) {
                // from the first beam under the rail to the last, how long it runs with none under it
                auto last { -1.0 };
                auto first { -1.0 };
                for( int step = 0; step <= steps; ++step ) {
                    auto const distance { length * step / std::max( 1, steps ) };
                    glm::dvec3 point, across;
                    segment->plan_frame( distance, point, across );
                    auto const rail { plan( point + across * ( side * 0.5 * std::abs( track->fTrackWidth ) ) ) };
                    auto const *cell { grid.at( rail ) };
                    if( cell == nullptr || std::none_of( cell->begin(), cell->end(), [&]( std::size_t const Index ) { return holds( boxes[ Index ], rail ); } ) ) {
                        continue;
                    }
                    if( last >= 0.0 && distance - last > 0.06 ) {
                        // the spacing of the beams, middle to middle, by the stretch without one
                        auto const spacing { distance - last + 2.0 * half_width };
                        if( spacing > result.widest ) {
                            result.widest = spacing;
                            result.widestat = track->name() + " " + to_string( distance, 2 );
                        }
                        if( spacing > 1.25 * 1.08 * pitch ) {
                            ++result.gaps;
                            if( result.examples.size() < 24 ) {
                                result.examples.push_back( "gap " + to_string( spacing, 2 ) + " m under " + track->name() + " path " + std::to_string( path ) + " at " + to_string( distance, 2 ) );
                            }
                        }
                    }
                    if( first < 0.0 ) {
                        first = distance;
                    }
                    last = distance;
                }
                if( first < 0.0 ) {
                    ++result.bare;
                    if( result.examples.size() < 80 ) {
                        result.examples.push_back( "no beams under " + track->name() + " path " + std::to_string( path ) );
                    }
                }
            }
        }
    }
    return result;
}

void unsettle_around( TTrack &Track ) {

    outdate_layouts();
    tracks_near( Track, []( TTrack *Other ) {
        if( Other->m_sleepers3d.active ) {
            Other->m_sleepers3d.dirty = true;
        } } );
}

} // namespace track_sleepers
