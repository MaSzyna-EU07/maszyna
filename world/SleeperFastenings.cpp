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
#include "simulation/simulation.h"

#include <algorithm>
#include <numeric>
#include <optional>

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
        auto const size { track->rail_dimensions() };
        auto const middle { 0.5 * std::abs( track->fTrackWidth ) + size.web };
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
                line.foot = size.web - size.low;
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
                glm::normalize( glm::mix( line.inward[ index - 1 ], line.inward[ index ], v ) ),
                line.foot } );
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
            // a rail of the two with its check rail on the side away from the other one in the fastening with the check rail,
            // the other one in a plate of its own
            auto const away = [&]( hit const &Rail, double const Toward ) {
                return Rail.kind == role::checked && glm::dot( Rail.inward, axis ) * Toward < 0.0; };
            auto const awayfirst { away( rail, 1.0 ) };
            auto const awaysecond { away( other, -1.0 ) };
            if( has( part_check ) && ( awayfirst || awaysecond ) ) {
                auto const joined { other.at - rail.at < fastening_nose };
                for( auto const &[ one, checked ] : { std::make_pair( &rail, awayfirst ), std::make_pair( &other, awaysecond ) } ) {
                    if( checked ) {
                        result.push_back( { part_check, point( one->at ), one->inward, 1.f, static_cast<float>( 0.5 * check_seats ) } );
                    }
                    else if( false == joined ) {
                        result.push_back( { part_plate, point( one->at ), one->inward } );
                    }
                }
            }
            else if( other.at - rail.at < fastening_nose ) {
                result.push_back( { part_plate, point( 0.5 * ( rail.at + other.at ) ), rail.inward } );
            }
            else if( has( part_check ) ) {
                result.push_back( { part_check, point( 0.5 * ( rail.at + other.at - check_seats ) ), axis, 1.f, static_cast<float>( 0.5 * check_seats ) } );
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
                if( has( part_slide ) ) {
                    // the slide plate from the edge of the foot of the stock rail on, not under it
                    auto const &slide { Model.spans[ part_slide ] };
                    auto const edge { std::min<double>( slide.y, -rail.foot ) };
                    result.push_back( { part_slide, point( rail.at ), -rail.inward, static_cast<float>( ( edge - slide.x ) / ( slide.y - slide.x ) ), slide.y, static_cast<float>( edge - slide.y ) } );
                }
                break;
            }
            case role::checked: {
                result.push_back( { has( part_check ) ? part_check : part_plate, point( rail.at ), rail.inward, 1.f, has( part_check ) ? static_cast<float>( 0.5 * check_seats ) : 0.f } );
                break;
            }
            default: {
                result.push_back( { part_plate, point( rail.at ), rail.inward } );
                break;
            }
        }
    }
    fit( result, Beam, Model );
    return result;
}

void fastened_rails::fit( std::vector<fastening> &Fastenings, beam const &Beam, beam_model const &Model ) {

    auto const start { plan( Beam.from ) };
    auto const axis { glm::normalize( plan( Beam.to - Beam.from ) ) };
    auto const at = [&]( fastening const &Fastening ) {
        return glm::dot( plan( Fastening.point ) - start, axis ); };
    // how far it reaches along the timber before and after where it is, at the scale
    auto const reach = [&]( fastening const &Fastening, double const Scale ) {
        auto const &span { Model.spans[ Fastening.part ] };
        auto const turn { glm::dot( Fastening.across, axis ) };
        auto const a { turn * Fastening.placed( span.x, Scale ) };
        auto const b { turn * Fastening.placed( span.y, Scale ) };
        return glm::dvec2 { -std::min( a, b ), std::max( a, b ) }; };
    auto const least = [&]( fastening const &Fastening ) {
        switch( Fastening.part ) {
            case part_plate: { return plate_least; }
            case part_slide: { return slide_least; }
            case part_check: { return check_least; }
            default: { return 1.0; }
        } };
    // those of one rail together, the rails in the order along the timber
    std::vector<std::size_t> order( Fastenings.size() );
    std::iota( order.begin(), order.end(), std::size_t { 0 } );
    std::stable_sort( order.begin(), order.end(), [&]( std::size_t const Left, std::size_t const Right ) {
        return at( Fastenings[ Left ] ) < at( Fastenings[ Right ] ) - 1e-6; } );
    std::vector<std::pair<std::size_t, std::size_t>> rails;
    for( std::size_t index = 0; index < order.size(); ) {
        auto next { index + 1 };
        while( next < order.size() && at( Fastenings[ order[ next ] ] ) - at( Fastenings[ order[ index ] ] ) < 1e-6 ) {
            ++next;
        }
        rails.emplace_back( index, next );
        index = next;
    }
    for( std::size_t index = 1; index < rails.size(); ++index ) {
        auto const [ firstfrom, firstto ] { rails[ index - 1 ] };
        auto const [ secondfrom, secondto ] { rails[ index ] };
        auto const room { at( Fastenings[ order[ secondfrom ] ] ) - at( Fastenings[ order[ firstfrom ] ] ) - fastening_gap };
        // the ones of both reaching less toward the other as they're narrowed; with them narrowed by the share, how far the two
        // lie over each other
        std::vector<std::size_t> narrowed;
        for( auto member { firstfrom }; member < secondto; ++member ) {
            auto const &fastening { Fastenings[ order[ member ] ] };
            auto const side { member < firstto ? 1 : 0 };
            if( reach( fastening, std::min<double>( fastening.scale, least( fastening ) ) )[ side ] < reach( fastening, fastening.scale )[ side ] - 1e-6 ) {
                narrowed.push_back( order[ member ] );
            }
        }
        auto const over = [&]( double const Share ) {
            auto after { 0.0 };
            auto before { 0.0 };
            for( auto member { firstfrom }; member < secondto; ++member ) {
                auto const &fastening { Fastenings[ order[ member ] ] };
                auto const shrinks { std::find( narrowed.begin(), narrowed.end(), order[ member ] ) != narrowed.end() };
                auto const scale { shrinks ? glm::mix<double>( fastening.scale, least( fastening ), Share ) : fastening.scale };
                if( member < firstto ) {
                    after = std::max( after, reach( fastening, scale ).y );
                }
                else {
                    before = std::max( before, reach( fastening, scale ).x );
                }
            }
            return after + before - room; };
        if( narrowed.empty() || over( 0.0 ) <= 0.0 ) {
            continue;
        }
        auto share { 1.0 };
        if( over( 1.0 ) < 0.0 ) {
            auto low { 0.0 };
            for( int step = 0; step < 20; ++step ) {
                auto const middle { 0.5 * ( low + share ) };
                ( over( middle ) > 0.0 ? low : share ) = middle;
            }
        }
        for( auto const index : narrowed ) {
            auto &fastening { Fastenings[ index ] };
            fastening.scale = static_cast<float>( glm::mix<double>( fastening.scale, least( fastening ), share ) );
        }
    }
    // a rail still lying on the fastening of the rail beside it goes without one of its own, unless it's a chair
    std::vector<bool> dropped( Fastenings.size(), false );
    auto const chair = [&]( std::pair<std::size_t, std::size_t> const &Rail ) {
        for( auto member { Rail.first }; member < Rail.second; ++member ) {
            if( Fastenings[ order[ member ] ].part == part_chair || Fastenings[ order[ member ] ].part == part_slide ) {
                return true;
            }
        }
        return false; };
    auto const plates = [&]( std::pair<std::size_t, std::size_t> const &Rail ) {
        for( auto member { Rail.first }; member < Rail.second; ++member ) {
            if( Fastenings[ order[ member ] ].part != part_plate ) {
                return false;
            }
        }
        return true; };
    auto const extent = [&]( std::pair<std::size_t, std::size_t> const &Rail ) {
        glm::dvec2 result { 0.0 };
        for( auto member { Rail.first }; member < Rail.second; ++member ) {
            if( false == dropped[ order[ member ] ] ) {
                result = glm::max( result, reach( Fastenings[ order[ member ] ], Fastenings[ order[ member ] ].scale ) );
            }
        }
        return result; };
    auto kept { std::size_t { 0 } };
    for( std::size_t index = 1; index < rails.size(); ++index ) {
        auto const &first { rails[ kept ] };
        auto const &second { rails[ index ] };
        auto const apart { at( Fastenings[ order[ second.first ] ] ) - at( Fastenings[ order[ first.first ] ] ) };
        auto const overlaps = [&]() {
            return extent( first ).y + extent( second ).x - apart > fastening_gap; };
        if( false == overlaps() ) {
            kept = index;
            continue;
        }
        auto const firstreach { extent( first ) };
        auto const secondreach { extent( second ) };
        auto const drop = [&]( std::pair<std::size_t, std::size_t> const &Rail ) {
            for( auto member { Rail.first }; member < Rail.second; ++member ) {
                dropped[ order[ member ] ] = true;
            } };
        // the brace of a chair where the rail beside comes too close for it, its slide plate kept
        auto const brace = [&]( std::pair<std::size_t, std::size_t> const &Rail, int const Side ) {
            for( auto member { Rail.first }; member < Rail.second; ++member ) {
                auto const &fastening { Fastenings[ order[ member ] ] };
                if( fastening.part == part_chair && reach( fastening, fastening.scale )[ Side ] > fastening_gap ) {
                    dropped[ order[ member ] ] = true;
                    return true;
                }
            }
            return false; };
        if( false == chair( second ) && apart <= firstreach.y ) {
            drop( second );
            continue;
        }
        if( false == chair( first ) && apart <= secondreach.x ) {
            drop( first );
        }
        else {
            brace( first, 1 );
            if( overlaps() ) {
                brace( second, 0 );
            }
        }
        // a rail in a plate still lying over the fastening of a rail beside held with more than a plate goes without its own
        if( overlaps() ) {
            if( plates( second ) && false == plates( first ) ) {
                drop( second );
                continue;
            }
            if( plates( first ) && false == plates( second ) ) {
                drop( first );
            }
        }
        kept = index;
    }
    std::vector<fastening> fitted;
    for( std::size_t index = 0; index < Fastenings.size(); ++index ) {
        if( false == dropped[ index ] ) {
            fitted.push_back( Fastenings[ index ] );
        }
    }
    Fastenings = std::move( fitted );
}

std::vector<fastening_row> check_fastenings() {

    std::vector<fastening_row> result;
    for( auto *track : simulation::Paths.sequence() ) {
        if( track == nullptr || track->m_editorremoved || false == track->m_sleepers3d.active ) {
            continue;
        }
        auto const *model { timber_model( *track ) };
        if( model == nullptr || false == model->fastened() ) {
            continue;
        }
        fastening_row row;
        row.track = track->name();
        std::optional<fastened_rails> rails;
        for( auto const &beam : place( *track ) ) {
            if( beam.plain ) {
                continue;
            }
            if( false == rails.has_value() ) {
                rails.emplace( *track );
            }
            ++row.beams;
            auto const start { plan( beam.from ) };
            auto const axis { glm::normalize( plan( beam.to - beam.from ) ) };
            struct footprint {
                double from;
                double to;
                int part;
                glm::dvec3 point;
            };
            std::vector<footprint> footprints;
            for( auto const &fastening : rails->on( beam, *model ) ) {
                ++row.parts[ fastening.part ];
                auto const at { glm::dot( plan( fastening.point ) - start, axis ) };
                auto const turn { glm::dot( fastening.across, axis ) };
                auto const a { at + turn * fastening.placed( model->spans[ fastening.part ].x, fastening.scale ) };
                auto const b { at + turn * fastening.placed( model->spans[ fastening.part ].y, fastening.scale ) };
                footprints.push_back( { std::min( a, b ), std::max( a, b ), fastening.part, fastening.point } );
            }
            std::sort( footprints.begin(), footprints.end(), []( footprint const &Left, footprint const &Right ) { return Left.from < Right.from; } );
            for( std::size_t index = 1; index < footprints.size(); ++index ) {
                auto const over { footprints[ index - 1 ].to - footprints[ index ].from };
                if( over <= fastening_overlap ) {
                    continue;
                }
                ++row.overlaps;
                row.cases.push_back( { { footprints[ index - 1 ].part, footprints[ index ].part }, glm::length( plan( footprints[ index ].point - footprints[ index - 1 ].point ) ), over, footprints[ index ].point } );
                if( over > row.worst ) {
                    row.worst = over;
                    row.worstat = footprints[ index ].point;
                    row.worstparts = std::string { part_names[ footprints[ index - 1 ].part ] } + "/" + part_names[ footprints[ index ].part ];
                }
            }
        }
        if( row.beams > 0 ) {
            result.push_back( std::move( row ) );
        }
    }
    return result;
}

} // namespace track_sleepers
