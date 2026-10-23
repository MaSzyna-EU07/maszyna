/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"

#include "world/SleeperFastenings.h"
#include "world/SleeperGeometry.h"
#include "world/SleeperPaths.h"
#include "world/SleeperRules.h"
#include "world/TrackRails.h"
#include "world/Track.h"

namespace track_sleepers {

fastened_rails::role fastened_rails::rail::at( double const Distance ) const {

    auto const within = [&]( glm::dvec2 const &Stretch ) {
        return Stretch.y > Stretch.x && Distance >= Stretch.x && Distance <= Stretch.y; };
    if( within( blade ) ) {
        return role::blade;
    }
    if( within( stock ) ) {
        return role::stock;
    }
    if( std::any_of( checks.begin(), checks.end(), within ) ) {
        return role::checked;
    }
    return role::plain;
}

fastened_rails::fastened_rails( TTrack &Track ) {

    std::vector<TTrack *> tracks;
    tracks_near( Track, [&]( TTrack *Other ) {
        if( bedded( *Other ) && std::find( tracks.begin(), tracks.end(), Other ) == tracks.end() ) {
            tracks.push_back( Other );
        } } );
    for( auto *track : tracks ) {
        auto const middle { 0.5 * std::abs( track->fTrackWidth ) + track->rail_dimensions().web };
        auto const cut { track_rails::cut_rails( *track ) };
        auto const switched { track->eType == tt_Switch && track->SwitchExtension != nullptr };
        // the side of the blade of the path, and the stretch along it from the tip to the heel
        auto const bladeside = [&]( int const Path ) {
            return ( track->SwitchExtension->RightSwitch == ( Path == 0 ) ? 1 : -1 ); };
        auto const bladestretch = [&]( int const Path ) {
            auto const &shape { track->blade( Path ) };
            return glm::dvec2 { shape.valid ? shape.tip : 0.0, track_rails::blade_length( *track, Path ) }; };
        for( int path = 0; path < path_count( *track ); ++path ) {
            auto const *segment { path_segment( *track, path ) };
            if( segment == nullptr ) {
                continue;
            }
            auto const length { segment->GetLength() };
            auto const count { std::max( 1, static_cast<int>( std::ceil( length / fastening_step ) ) ) };
            for( auto const side : { -1, 1 } ) {
                rail line;
                line.track = track;
                line.path = path;
                line.side = side;
                line.low = glm::dvec2 { std::numeric_limits<double>::max() };
                line.high = glm::dvec2 { std::numeric_limits<double>::lowest() };
                for( int step = 0; step <= count; ++step ) {
                    auto const distance { length * step / count };
                    glm::dvec3 point, across;
                    segment->plan_frame( distance, point, across );
                    auto const flat { plan( across ) };
                    auto const inward { glm::length( flat ) > 1e-9 ? glm::normalize( flat ) * static_cast<double>( -side ) : glm::dvec2 { 1.0, 0.0 } };
                    auto const position { plan( point ) - inward * middle };
                    line.points.push_back( position );
                    line.heights.push_back( point.y );
                    line.inward.push_back( inward );
                    line.distances.push_back( distance );
                    line.low = glm::min( line.low, position );
                    line.high = glm::max( line.high, position );
                }
                if( switched ) {
                    if( side == bladeside( path ) ) {
                        line.blade = { 0.0, bladestretch( path ).y };
                    }
                    if( side == bladeside( 1 - path ) ) {
                        auto const stretch { bladestretch( 1 - path ) };
                        line.stock = { std::max( 0.0, stretch.x - chair_lead ), stretch.y };
                    }
                }
                if( cut ) {
                    for( auto const &check : track_rails::rail_checks( *track )[ path ][ side > 0 ? 1 : 0 ] ) {
                        line.checks.emplace_back( check.from, check.to );
                    }
                }
                m_rails.emplace_back( std::move( line ) );
            }
        }
    }
}

std::vector<fastening> fastened_rails::on( beam const &Beam, beam_model const &Model ) const {

    std::vector<fastening> result;
    auto const span { Beam.to - Beam.from };
    auto const length { glm::length( plan( span ) ) };
    if( length < 0.1 ) {
        return result;
    }
    auto const start { plan( Beam.from ) };
    auto const axis { plan( span ) / length };
    auto const low { glm::min( start, plan( Beam.to ) ) };
    auto const high { glm::max( start, plan( Beam.to ) ) };
    std::vector<hit> hits;
    for( auto const &line : m_rails ) {
        if( line.high.x < low.x || line.low.x > high.x || line.high.y < low.y || line.low.y > high.y ) {
            continue;
        }
        auto const last { line.points.size() - 1 };
        for( std::size_t index = 1; index <= last; ++index ) {
            auto const from { line.points[ index - 1 ] };
            auto const chord { line.points[ index ] - from };
            double u, v;
            if( false == track_rails::meet( start, plan( span ), from, chord, 1e-12, u, v ) ) {
                continue;
            }
            if( u < 0.0 || u > 1.0 || v < 0.0 || v > 1.0 || ( v == 1.0 && index < last ) ) {
                continue;
            }
            auto const direction { glm::normalize( chord ) };
            if( std::abs( cross( axis, direction ) ) < crossing_sine_min ) {
                continue;
            }
            auto const height { glm::mix( line.heights[ index - 1 ], line.heights[ index ], v ) };
            if( std::abs( height - glm::mix( Beam.from.y, Beam.to.y, u ) ) > level_tolerance ) {
                continue;
            }
            hits.push_back( {
                u * length,
                line.at( glm::mix( line.distances[ index - 1 ], line.distances[ index ], v ) ),
                glm::normalize( glm::mix( line.inward[ index - 1 ], line.inward[ index ], v ) ) } );
        }
    }
    std::sort( hits.begin(), hits.end(), []( hit const &Left, hit const &Right ) { return Left.at < Right.at; } );
    // one rail where several lie on one another, the stock rail before the rail checked, before a plain one; a blade with no
    // other rail there slides on the chair of its stock rail
    auto const rank = []( role const Kind ) {
        switch( Kind ) {
            case role::stock: { return 3; }
            case role::checked: { return 2; }
            case role::plain: { return 1; }
            default: { return 0; }
        } };
    std::vector<hit> rails;
    for( std::size_t index = 0; index < hits.size(); ) {
        auto best { hits[ index ] };
        auto next { index + 1 };
        for( ; next < hits.size() && hits[ next ].at - hits[ next - 1 ].at < fastening_merge; ++next ) {
            if( rank( hits[ next ].kind ) > rank( best.kind ) ) {
                best = hits[ next ];
            }
        }
        if( best.kind != role::blade ) {
            rails.push_back( best );
        }
        index = next;
    }
    auto const point = [&]( double const At ) {
        return Beam.from + span * ( At / length ); };
    auto const has = [&]( int const Part ) {
        return Model.parts[ Part ] != nullptr; };
    for( std::size_t index = 0; index < rails.size(); ++index ) {
        auto const &rail { rails[ index ] };
        if( index + 1 < rails.size() && rails[ index + 1 ].at - rail.at < fastening_pair && rail.kind != role::stock && rails[ index + 1 ].kind != role::stock ) {
            auto const &other { rails[ index + 1 ] };
            if( other.at - rail.at < fastening_nose ) {
                result.push_back( { part_plate, point( 0.5 * ( rail.at + other.at ) ), rail.inward } );
            }
            else if( has( part_check ) ) {
                result.push_back( { part_check, point( rail.at ), axis } );
            }
            else {
                result.push_back( { part_plate, point( rail.at ), rail.inward } );
                result.push_back( { part_plate, point( other.at ), other.inward } );
            }
            ++index;
            continue;
        }
        switch( rail.kind ) {
            case role::stock: {
                result.push_back( { has( part_chair ) ? part_chair : part_plate, point( rail.at ), -rail.inward } );
                break;
            }
            case role::checked: {
                result.push_back( { has( part_check ) ? part_check : part_plate, point( rail.at ), rail.inward } );
                break;
            }
            default: {
                result.push_back( { part_plate, point( rail.at ), rail.inward } );
                break;
            }
        }
    }
    return result;
}

} // namespace track_sleepers
