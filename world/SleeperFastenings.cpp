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
#include <functional>
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

bool fastened_rails::rail::guarded( double const Distance ) const {

    return std::any_of( guards.begin(), guards.end(), [&]( glm::dvec2 const &Stretch ) { return Distance >= Stretch.x && Distance <= Stretch.y; } );
}

fastened_rails::fastened_rails( TTrack &Track ) {

    std::vector<TTrack *> tracks;
    tracks_near( Track, [&]( TTrack *Other ) {
        if( bedded( *Other ) && std::find( tracks.begin(), tracks.end(), Other ) == tracks.end() ) {
            tracks.push_back( Other );
        } } );
    auto &kept { fastened_rails::kept() };
    for( auto *track : tracks ) {
        auto &rails { kept[ track ] };
        if( rails == nullptr ) {
            rails = std::make_shared<std::vector<rail> const>( rails_of( *track ) );
        }
        m_kept.push_back( rails );
        m_rails.insert( m_rails.end(), rails->begin(), rails->end() );
    }
}

fastened_rails::kept_rails &fastened_rails::kept() {

    static kept_rails tracks;
    static unsigned int stamp { 0 };
    if( stamp != track_rails::memo_stamp() ) {
        tracks.clear();
        stamp = track_rails::memo_stamp();
    }
    return tracks;
}

void forget_rails( TTrack const &Track ) {

    fastened_rails::kept().erase( &Track );
}

std::vector<fastened_rails::rail> fastened_rails::rails_of( TTrack &Track ) {

    std::vector<rail> result;
    auto const size { Track.rail_dimensions() };
    auto const middle { 0.5 * std::abs( Track.fTrackWidth ) + size.web };
    auto const cut { track_rails::cut_rails( Track ) };
    auto const switched { Track.eType == tt_Switch && Track.SwitchExtension != nullptr };
    auto const rise { wing_rise( Track ) };
    // what the rail is along the path: a blade, the stock rail of a blade, checked
    auto const add = [&]( rail &&Line, int const Path, int const Side ) {
        if( switched ) {
            if( auto const heel { blade_heel( Track, Path, Side ) }; heel > 0.0 ) {
                Line.blade = { 0.0, heel };
            }
            Line.stock = stock_stretch( Track, Path, Side );
        }
        if( cut ) {
            for( auto const &check : track_rails::rail_checks( Track )[ Path ][ Side > 0 ? 1 : 0 ] ) {
                Line.checks.emplace_back( check.from, check.to );
            }
            Line.guards = Track.m_railguards[ Path ][ Side > 0 ? 1 : 0 ];
        }
        result.emplace_back( std::move( Line ) ); };
    // the rail of the side of the path from and to along it, by points the given way out from the middle of the path
    auto const sampled = [&]( int const Path, int const Side, double const From, double const To, double const Middle, knots::line const &Offsets ) {
        auto const *segment { path_segment( Track, Path ) };
        auto const count { std::max( 1, static_cast<int>( std::ceil( ( To - From ) / fastening_step ) ) ) };
        rail line;
        line.track = &Track;
        line.path = Path;
        line.side = Side;
        line.foot = size.web - size.low;
        line.low = glm::dvec2 { std::numeric_limits<double>::max() };
        line.high = glm::dvec2 { std::numeric_limits<double>::lowest() };
        for( int step = 0; step <= count; ++step ) {
            auto const distance { From + ( To - From ) * step / count };
            glm::dvec3 point, across;
            segment->plan_frame( distance, point, across );
            auto const flat { plan( across ) };
            auto const inward { glm::length( flat ) > 1e-9 ? glm::normalize( flat ) * static_cast<double>( -Side ) : glm::dvec2 { 1.0, 0.0 } };
            auto const position { plan( point ) - inward * ( Middle + knots::sample( Offsets, distance, 0.0 ) ) };
            line.points.push_back( position );
            line.heights.push_back( point.y );
            line.inward.push_back( inward );
            line.distances.push_back( distance );
            line.low = glm::min( line.low, position );
            line.high = glm::max( line.high, position );
        }
        return line; };
    for( int path = 0; path < path_count( Track ); ++path ) {
        auto const *segment { path_segment( Track, path ) };
        if( segment == nullptr ) {
            continue;
        }
        auto const length { segment->GetLength() };
        for( auto const side : { -1, 1 } ) {
            // the rail in its parts between the cuts at the crossings, as they're laid: a wing rail bent out from the path; on over the
            // gap at an obtuse crossing, where its check rail lies
            auto cuts { cut ? Track.m_railplans[ path ][ side > 0 ].cuts : std::vector<track_rails::rail_cut> {} };
            std::stable_sort( cuts.begin(), cuts.end(), []( track_rails::rail_cut const &A, track_rails::rail_cut const &B ) { return A.at < B.at; } );
            for( std::size_t part = 0; part <= cuts.size(); ++part ) {
                auto const *before { part > 0 ? &cuts[ part - 1 ] : nullptr };
                auto const *after { part < cuts.size() ? &cuts[ part ] : nullptr };
                auto const from { std::max( 0.0, before != nullptr ? before->start : 0.0 ) };
                auto const to { std::min( length, after != nullptr ? std::max( after->end, after->start ) : length ) };
                if( to - from < fastening_merge ) {
                    continue;
                }
                auto line { sampled( path, side, from, to, middle, track_rails::part_offsets( before, after ) ) };
                line.rises = track_rails::part_rises( rise > 0.0 ? track_rails::wing_shares( before, after, stock_stretch( Track, path, side ) ) : knots::line {}, rise, blade_heel( Track, path, side ) );
                add( std::move( line ), path, side );
            }
            // the guard rails of the crossings at wide angles inside the rail, the flangeway from it
            for( auto const &guard : cut ? Track.m_railguards[ path ][ side > 0 ] : std::vector<glm::dvec2> {} ) {
                auto const from { std::max( 0.0, guard.x ) };
                auto const to { std::min( length, guard.y ) };
                if( to - from < fastening_merge ) {
                    continue;
                }
                auto line { sampled( path, side, from, to, middle - ( track_rails::flangeway + size.head ), {} ) };
                line.guard = true;
                result.emplace_back( std::move( line ) );
            }
        }
    }
    return result;
}

std::vector<fastened_rails::hit> fastened_rails::rails_on( beam const &Beam ) const {

    auto const span { Beam.to - Beam.from };
    auto const length { glm::length( plan( span ) ) };
    if( length < 0.1 ) {
        return {};
    }
    auto const start { plan( Beam.from ) };
    auto const axis { plan( span ) / length };
    auto const low { glm::min( start, plan( Beam.to ) ) };
    auto const high { glm::max( start, plan( Beam.to ) ) };
    std::vector<hit> hits;
    for( rail const &line : m_rails ) {
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
            auto const distance { glm::mix( line.distances[ index - 1 ], line.distances[ index ], v ) };
            hits.push_back( {
                u * length,
                line.at( distance ),
                glm::normalize( glm::mix( line.inward[ index - 1 ], line.inward[ index ], v ) ),
                line.foot,
                knots::sample( line.rises, distance, 0.0 ) } );
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
        auto rise { best.rise };
        for( ; next < hits.size() && hits[ next ].at - hits[ next - 1 ].at < fastening_merge; ++next ) {
            if( rank( hits[ next ].kind ) > rank( best.kind ) ) {
                best = hits[ next ];
            }
            rise = std::max( rise, hits[ next ].rise );
        }
        best.rise = rise;
        if( best.kind != role::blade ) {
            rails.push_back( best );
        }
        index = next;
    }
    return rails;
}

std::vector<fastening> fastened_rails::on( beam const &Beam, beam_model const &Model ) const {

    std::vector<fastening> result;
    auto const span { Beam.to - Beam.from };
    auto const length { glm::length( plan( span ) ) };
    if( length < 0.1 ) {
        return result;
    }
    auto const axis { plan( span ) / length };
    auto const rails { rails_on( Beam ) };
    auto const point = [&]( double const At ) {
        return Beam.from + span * ( At / length ); };
    auto const has = [&]( int const Part ) {
        return Model.parts[ Part ] != nullptr; };
    // a fastening of two rails with its seats at the origin the way given, holding more rails than its seats, as the rails of a frog
    // or two rails with a check rail between: stretched under the rails from the first to the last and their check rails, its ends with
    // the clips past the feet of the outer ones, short of the feet of the rails beside
    auto const whole = [&]( std::size_t const First, std::size_t const Last, double const Origin, double const Turn ) {
        auto const &reach { Model.spans[ part_check ] };
        auto low { std::numeric_limits<double>::max() };
        auto high { std::numeric_limits<double>::lowest() };
        for( auto index { First }; index <= Last; ++index ) {
            auto const &rail { rails[ index ] };
            for( auto const at : { rail.at, rail.kind == role::checked ? rail.at + check_seats / glm::dot( rail.inward, axis ) : rail.at } ) {
                low = std::min( low, at - rail.foot - fastening_clip );
                high = std::max( high, at + rail.foot + fastening_clip );
            }
        }
        if( First > 0 ) {
            low = std::max( low, rails[ First - 1 ].at + rails[ First - 1 ].foot + fastening_gap );
        }
        if( Last + 1 < rails.size() ) {
            high = std::min( high, rails[ Last + 1 ].at - rails[ Last + 1 ].foot - fastening_gap );
        }
        low = std::min( low, Origin + std::min( Turn * reach.x, Turn * reach.y ) );
        high = std::max( high, Origin + std::max( Turn * reach.x, Turn * reach.y ) );
        auto const scale { ( high - low ) / ( reach.y - reach.x ) };
        return fastening { part_check, point( Origin ), axis * Turn, static_cast<float>( scale ), 0.f,
            static_cast<float>( Turn * ( 0.5 * ( low + high ) - Origin ) - scale * 0.5 * ( reach.x + reach.y ) ), true, true }; };
    // the rails from the first to the last under fastenings of two rails, each with its seats under two rails side by side and over as
    // many of the rails around as it reaches: the one holding the most first, the rails before and after it in turn
    std::function<void( std::size_t, std::size_t )> crowd = [&]( std::size_t const First, std::size_t const Last ) {
        auto const &reach { Model.spans[ part_check ] };
        std::size_t covered { 0 };
        std::size_t from { First };
        auto origin { 0.0 };
        auto turn { 1.0 };
        for( auto seat { First }; seat < Last; ++seat ) {
            for( auto const direction : { 1.0, -1.0 } ) {
                auto const at { 0.5 * ( rails[ seat ].at + rails[ seat + 1 ].at - direction * check_seats ) };
                auto const low { at + std::min( direction * reach.x, direction * reach.y ) };
                auto const high { at + std::max( direction * reach.x, direction * reach.y ) };
                if( rails[ seat ].at < low || rails[ seat + 1 ].at > high ) {
                    continue;
                }
                auto start { seat };
                while( start > First && rails[ start - 1 ].at >= low ) {
                    --start;
                }
                auto end { seat + 1 };
                while( end < Last && rails[ end + 1 ].at <= high ) {
                    ++end;
                }
                if( end - start + 1 > covered ) {
                    covered = end - start + 1;
                    from = start;
                    origin = at;
                    turn = direction;
                }
            }
        }
        if( covered < 2 ) {
            for( auto rail { First }; rail <= Last; ++rail ) {
                result.push_back( { part_plate, point( rails[ rail ].at ), rails[ rail ].inward } );
            }
            return;
        }
        result.push_back( covered > 2 ?
            whole( from, from + covered - 1, origin, turn ) :
            fastening { part_check, point( origin ), axis * turn, 1.f, static_cast<float>( 0.5 * check_seats ), 0.f, true } );
        if( from > First ) {
            crowd( First, from - 1 );
        }
        if( from + covered <= Last ) {
            crowd( from + covered, Last );
        }
    };
    // the rails and the check rails along the timber, each with its foot over the foot of the one beside, with a check rail between two
    // of them, as at the obtuse crossings of the slips, none a stock rail: from the first rail to the last, held by one fastening. a
    // check rail where a rail lies is that rail
    std::vector<std::size_t> runs( rails.size(), rails.size() ); // of the first rail of each, the last
    std::vector<bool> run( rails.size(), false );
    if( has( part_check ) ) {
        struct line {
            double at;
            double foot;
            std::size_t rail;
            bool check;
        };
        std::vector<line> lines;
        for( std::size_t index = 0; index < rails.size(); ++index ) {
            auto const &rail { rails[ index ] };
            lines.push_back( { rail.at, rail.foot, index, false } );
            if( rail.kind != role::checked ) {
                continue;
            }
            auto const at { rail.at + check_seats / glm::dot( rail.inward, axis ) };
            if( std::none_of( rails.begin(), rails.end(), [&]( hit const &Other ) { return std::abs( Other.at - at ) < fastening_nose; } ) ) {
                lines.push_back( { at, rail.foot, index, true } );
            }
        }
        std::stable_sort( lines.begin(), lines.end(), []( line const &Left, line const &Right ) { return Left.at < Right.at; } );
        for( std::size_t first = 0; first < lines.size(); ) {
            auto last { first };
            while( last + 1 < lines.size() && lines[ last + 1 ].at - lines[ last ].at < lines[ last ].foot + lines[ last + 1 ].foot ) {
                ++last;
            }
            auto const between { last > first + 1 && std::any_of( lines.begin() + first + 1, lines.begin() + last, []( line const &Line ) { return Line.check; } ) };
            auto low { rails.size() };
            auto high { std::size_t { 0 } };
            auto stock { false };
            for( auto index { first }; index <= last; ++index ) {
                low = std::min( low, lines[ index ].rail );
                high = std::max( high, lines[ index ].rail );
                stock = stock || rails[ lines[ index ].rail ].kind == role::stock;
            }
            if( between && false == stock && false == run[ low ] ) {
                runs[ low ] = high;
                std::fill( run.begin() + low, run.begin() + high + 1, true );
            }
            first = last + 1;
        }
    }
    for( std::size_t index = 0; index < rails.size(); ++index ) {
        auto const &rail { rails[ index ] };
        if( runs[ index ] < rails.size() ) {
            auto const &reach { Model.spans[ part_check ] };
            auto const last { runs[ index ] };
            result.push_back( whole( index, last, 0.5 * ( rail.at + rails[ last ].at - reach.x - reach.y ), 1.0 ) );
            index = last;
            continue;
        }
        // three rails or more close beside one another, as at the crossings of the slips, none a stock rail or with a check rail
        auto last { index };
        while( last + 1 < rails.size() && rails[ last + 1 ].at - rails[ last ].at < fastening_crowd && rails[ last ].kind == role::plain && rails[ last + 1 ].kind == role::plain && false == run[ last + 1 ] ) {
            ++last;
        }
        if( last >= index + 2 && has( part_check ) ) {
            crowd( index, last );
            index = last;
            continue;
        }
        if( index + 1 < rails.size() && rails[ index + 1 ].at - rail.at < fastening_pair && rail.kind != role::stock && rails[ index + 1 ].kind != role::stock && false == run[ index + 1 ] ) {
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
                result.push_back( { part_check, point( 0.5 * ( rail.at + other.at - check_seats ) ), axis, 1.f, static_cast<float>( 0.5 * check_seats ), 0.f, true } );
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
                if( has( part_chair ) ) {
                    // the rail on a plate as wide as its foot, between the brace and the slide plate
                    auto const &plate { Model.spans[ part_plate ] };
                    auto const low { -rail.foot };
                    auto const high { std::min<double>( rail.foot, Model.spans[ part_chair ].x ) };
                    auto const scale { ( high - low ) / ( plate.y - plate.x ) };
                    result.push_back( { part_plate, point( rail.at ), -rail.inward, static_cast<float>( scale ), 0.f, static_cast<float>( low - scale * plate.x ) } );
                }
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
    lift( result, rails, Beam, Model );
    along( result, Beam, Model );
    return result;
}

void fastened_rails::along( std::vector<fastening> &Fastenings, beam const &Beam, beam_model const &Model ) const {

    auto const span { Beam.to - Beam.from };
    auto const length { glm::length( plan( span ) ) };
    auto const start { plan( Beam.from ) };
    auto const axis { plan( span ) / length };
    auto const side { glm::normalize( plan( Beam.along ) ) };
    auto const low { glm::min( start, plan( Beam.to ) ) - 2.0 * half_width };
    auto const high { glm::max( start, plan( Beam.to ) ) + 2.0 * half_width };
    // where the rail lies over the timber, along it: a stretch of a chord of the rail
    struct stretch {
        rail const *line;
        std::size_t index; // of the end of the chord
        double from; // along the timber
        double to;
    };
    std::vector<stretch> stretches;
    for( rail const &line : m_rails ) {
        // a guard rail along the timber lies in the fastening of its running rail
        if( line.guard || line.high.x < low.x || line.low.x > high.x || line.high.y < low.y || line.low.y > high.y ) {
            continue;
        }
        for( std::size_t index = 1; index < line.points.size(); ++index ) {
            auto const chord { line.points[ index ] - line.points[ index - 1 ] };
            if( glm::length( chord ) < 1e-6 || std::abs( cross( axis, glm::normalize( chord ) ) ) >= crossing_sine_min ) {
                continue;
            }
            if( std::abs( line.heights[ index ] - glm::mix( Beam.from.y, Beam.to.y, std::clamp( glm::dot( line.points[ index ] - start, axis ) / length, 0.0, 1.0 ) ) ) > level_tolerance ) {
                continue;
            }
            // the share of the chord over the middle of the timber or its side toward the next one, and over its length
            auto const u0 { glm::dot( line.points[ index - 1 ] - start, axis ) };
            auto const u1 { glm::dot( line.points[ index ] - start, axis ) };
            auto const w0 { glm::dot( line.points[ index - 1 ] - start, side ) };
            auto const w1 { glm::dot( line.points[ index ] - start, side ) };
            auto from { 0.0 };
            auto to { 1.0 };
            auto const clip = [&]( double const A, double const B, double const Low, double const High ) {
                if( std::abs( B - A ) < 1e-9 ) {
                    if( A <= Low || A > High ) {
                        to = -1.0;
                    }
                    return;
                }
                auto const a { ( Low - A ) / ( B - A ) };
                auto const b { ( High - A ) / ( B - A ) };
                from = std::max( from, std::min( a, b ) );
                to = std::min( to, std::max( a, b ) ); };
            clip( w0, w1, -half_width + 2.0 * fastening_gap, half_width + 2.0 * fastening_gap );
            clip( u0, u1, 0.0, length );
            if( to <= from ) {
                continue;
            }
            auto const a { glm::mix( u0, u1, from ) };
            auto const b { glm::mix( u0, u1, to ) };
            stretches.push_back( { &line, index, std::min( a, b ), std::max( a, b ) } );
        }
    }
    if( stretches.empty() ) {
        return;
    }
    // kept clear of the fastenings of the rails across the timber, and of its ends
    std::vector<glm::dvec2> taken;
    for( auto const &fastening : Fastenings ) {
        auto const at { glm::dot( plan( fastening.point ) - start, axis ) };
        auto const turn { glm::dot( fastening.across, axis ) };
        auto const &reach { Model.spans[ fastening.part ] };
        auto const a { at + turn * fastening.placed( reach.x, fastening.scale ) };
        auto const b { at + turn * fastening.placed( reach.y, fastening.scale ) };
        taken.emplace_back( std::min( a, b ) - 0.5 * fastening_length - fastening_gap, std::max( a, b ) + 0.5 * fastening_length + fastening_gap );
    }
    auto const end { 0.5 * fastening_length + fastening_gap };
    std::sort( stretches.begin(), stretches.end(), []( stretch const &Left, stretch const &Right ) { return Left.line != Right.line ? Left.line < Right.line : Left.from < Right.from; } );
    for( std::size_t first = 0; first < stretches.size(); ) {
        // the stretches of one rail going on one from another
        auto last { first };
        auto to { stretches[ first ].to };
        while( last + 1 < stretches.size() && stretches[ last + 1 ].line == stretches[ first ].line && stretches[ last + 1 ].from <= to + 0.01 ) {
            to = std::max( to, stretches[ ++last ].to );
        }
        std::vector<glm::dvec2> free { glm::dvec2 { std::max( stretches[ first ].from, end ), std::min( to, length - end ) } };
        for( auto const &blocked : taken ) {
            std::vector<glm::dvec2> left;
            for( auto const &room : free ) {
                if( blocked.y <= room.x || blocked.x >= room.y ) {
                    left.push_back( room );
                    continue;
                }
                if( blocked.x > room.x ) {
                    left.emplace_back( room.x, blocked.x );
                }
                if( blocked.y < room.y ) {
                    left.emplace_back( blocked.y, room.y );
                }
            }
            free = std::move( left );
        }
        auto const plate = [&]( double const At ) {
            for( auto index { first }; index <= last; ++index ) {
                auto const &piece { stretches[ index ] };
                if( At < piece.from - 1e-6 || At > piece.to + 1e-6 ) {
                    continue;
                }
                auto const &line { *piece.line };
                auto const u0 { glm::dot( line.points[ piece.index - 1 ] - start, axis ) };
                auto const u1 { glm::dot( line.points[ piece.index ] - start, axis ) };
                auto const share { std::abs( u1 - u0 ) > 1e-9 ? std::clamp( ( At - u0 ) / ( u1 - u0 ), 0.0, 1.0 ) : 0.0 };
                auto const offset { glm::dot( glm::mix( line.points[ piece.index - 1 ], line.points[ piece.index ], share ) - start, side ) };
                auto const inward { glm::normalize( glm::mix( line.inward[ piece.index - 1 ], line.inward[ piece.index ], share ) ) };
                auto const point { Beam.from + span * ( At / length ) + glm::dvec3 { side.x, 0.0, side.y } * offset };
                // with its check rail, or the guard rail of the crossing, in the fastening of both
                auto const distance { glm::mix( line.distances[ piece.index - 1 ], line.distances[ piece.index ], share ) };
                if( Model.parts[ part_check ] != nullptr && ( line.at( distance ) == role::checked || line.guarded( distance ) ) ) {
                    Fastenings.push_back( { part_check, point, inward, 1.f, static_cast<float>( 0.5 * check_seats ) } );
                }
                else {
                    Fastenings.push_back( { part_plate, point, inward } );
                }
                return;
            } };
        for( auto const &room : free ) {
            if( room.y < room.x ) {
                continue;
            }
            if( room.y - room.x < 0.5 * pitch ) {
                plate( 0.5 * ( room.x + room.y ) );
                continue;
            }
            auto const count { static_cast<int>( std::ceil( ( room.y - room.x ) / pitch ) ) + 1 };
            for( int index = 0; index < count; ++index ) {
                plate( glm::mix( room.x, room.y, static_cast<double>( index ) / ( count - 1 ) ) );
            }
        }
        first = last + 1;
    }
}

std::vector<fastened_rails::hit> fastened_rails::lying( beam const &Beam ) const {

    return rails_on( Beam );
}

std::vector<glm::dvec2> fastened_rails::bare( beam const &Beam, beam_model const &Model, std::vector<fastening> const &Fastenings ) const {

    auto const span { plan( Beam.to - Beam.from ) };
    auto const length { glm::length( span ) };
    std::vector<glm::dvec2> result;
    for( auto const &rail : rails_on( Beam ) ) {
        auto const at { plan( Beam.from ) + span * ( rail.at / length ) };
        auto const under { std::any_of( Fastenings.begin(), Fastenings.end(), [&]( fastening const &Fastening ) {
            auto const along { glm::dot( at - plan( Fastening.point ), Fastening.across ) };
            auto const a { Fastening.placed( Model.spans[ Fastening.part ].x, Fastening.scale ) };
            auto const b { Fastening.placed( Model.spans[ Fastening.part ].y, Fastening.scale ) };
            return along >= std::min( a, b ) - 0.01 && along <= std::max( a, b ) + 0.01; } ) };
        if( false == under ) {
            result.push_back( at );
        }
    }
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
            case part_check: { return Fastening.whole ? static_cast<double>( Fastening.scale ) : check_least; }
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
    auto const checks = [&]( std::pair<std::size_t, std::size_t> const &Rail ) {
        for( auto member { Rail.first }; member < Rail.second; ++member ) {
            auto const &fastening { Fastenings[ order[ member ] ] };
            if( fastening.part == part_check && false == fastening.both ) {
                return true;
            }
        }
        return false; };
    auto const faces = [&]( std::pair<std::size_t, std::size_t> const &Rail, double const Toward ) {
        for( auto member { Rail.first }; member < Rail.second; ++member ) {
            auto const &fastening { Fastenings[ order[ member ] ] };
            if( fastening.part == part_check && false == fastening.both && glm::dot( fastening.across, axis ) * Toward > 0.0 ) {
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
        auto firstreach { extent( first ) };
        auto secondreach { extent( second ) };
        auto const drop = [&]( std::pair<std::size_t, std::size_t> const &Rail ) {
            for( auto member { Rail.first }; member < Rail.second; ++member ) {
                dropped[ order[ member ] ] = true;
            } };
        // the plates of the rail with their end toward the rail beside Room from it, the other end kept where it is
        auto const clip = [&]( std::pair<std::size_t, std::size_t> const &Rail, double const Toward, double const Room ) {
            if( Room < 0.0 ) {
                return false;
            }
            for( auto member { Rail.first }; member < Rail.second; ++member ) {
                auto &fastening { Fastenings[ order[ member ] ] };
                auto const &span { Model.spans[ fastening.part ] };
                auto const turn { glm::dot( fastening.across, axis ) };
                auto const toward { turn * Toward > 0.0 ? span.y : span.x };
                auto const away { turn * Toward > 0.0 ? span.x : span.y };
                // the other end where the narrowing left it, or as the plate was if that leaves too little of it
                auto stays { fastening.placed( away, fastening.scale ) };
                auto scale { ( Toward * turn * Room - stays ) / ( toward - away ) };
                if( scale < clip_least ) {
                    stays = fastening.placed( away, 1.0 );
                    scale = ( Toward * turn * Room - stays ) / ( toward - away );
                }
                if( scale < clip_least ) {
                    return false;
                }
                fastening.pivot = static_cast<float>( away );
                fastening.shift = static_cast<float>( stays - away );
                fastening.scale = static_cast<float>( scale );
            }
            return true; };
        // the fastenings of the rail made plates of their own
        auto const replate = [&]( std::pair<std::size_t, std::size_t> const &Rail ) -> std::pair<std::size_t, std::size_t> const & {
            for( auto member { Rail.first }; member < Rail.second; ++member ) {
                auto &fastening { Fastenings[ order[ member ] ] };
                fastening.part = part_plate;
                fastening.scale = 1.f;
                fastening.pivot = 0.f;
                fastening.shift = 0.f;
            }
            return Rail; };
        // the brace of a chair where the rail beside comes too close for it, its slide plate kept
        auto const brace = [&]( std::pair<std::size_t, std::size_t> const &Rail, int const Side, double const Reaching = fastening_gap ) {
            for( auto member { Rail.first }; member < Rail.second; ++member ) {
                auto const &fastening { Fastenings[ order[ member ] ] };
                if( fastening.part == part_chair && false == dropped[ order[ member ] ] && reach( fastening, fastening.scale )[ Side ] > Reaching ) {
                    dropped[ order[ member ] ] = true;
                    return true;
                }
            }
            return false; };
        // a rail lying on the brace of a chair beside takes the brace away
        if( brace( first, 1, apart ) || brace( second, 0, apart ) ) {
            if( false == overlaps() ) {
                kept = index;
                continue;
            }
            firstreach = extent( first );
            secondreach = extent( second );
        }
        // of two rails with check rails, the one with its check rail toward the other one, lying on the fastening of the other
        // one then, goes in a plate cut short at that fastening, or without its own, the other one keeping its check rail seated
        if( checks( first ) && checks( second ) && faces( first, 1.0 ) != faces( second, -1.0 ) ) {
            if( faces( first, 1.0 ) && apart - check_seats <= secondreach.x ) {
                if( false == clip( replate( first ), 1.0, apart - secondreach.x - fastening_gap ) ) {
                    drop( first );
                }
                kept = index;
                continue;
            }
            if( faces( second, -1.0 ) && apart - check_seats <= firstreach.y ) {
                if( false == clip( replate( second ), -1.0, apart - firstreach.y - fastening_gap ) ) {
                    drop( second );
                }
                continue;
            }
        }
        if( false == chair( second ) && apart <= firstreach.y && ( false == checks( second ) || checks( first ) ) ) {
            drop( second );
            continue;
        }
        if( false == chair( first ) && apart <= secondreach.x && ( false == checks( first ) || checks( second ) ) ) {
            drop( first );
        }
        else {
            brace( first, 1 );
            if( overlaps() ) {
                brace( second, 0 );
            }
        }
        // a rail in a plate still lying over the fastening of a rail beside held with more than a plate gets its plate cut short
        // at the edge of that fastening, as on one plate with it, or goes without its own if the plate would no longer be under it
        if( overlaps() ) {
            if( plates( second ) && false == plates( first ) ) {
                if( false == clip( second, -1.0, apart - extent( first ).y - fastening_gap ) ) {
                    drop( second );
                    continue;
                }
            }
            else if( plates( first ) && false == plates( second ) ) {
                if( false == clip( first, 1.0, apart - extent( second ).x - fastening_gap ) ) {
                    drop( first );
                }
            }
        }
        // the plates of two rails still lying over each other, as of the stock rails back to back, one plate under both
        auto const plate = [&]( std::pair<std::size_t, std::size_t> const &Rail ) {
            for( auto member { Rail.first }; member < Rail.second; ++member ) {
                if( Fastenings[ order[ member ] ].part == part_plate && false == dropped[ order[ member ] ] ) {
                    return order[ member ];
                }
            }
            return Fastenings.size(); };
        auto const firstplate { plate( first ) };
        auto const secondplate { plate( second ) };
        if( firstplate < Fastenings.size() && secondplate < Fastenings.size() ) {
            auto &one { Fastenings[ firstplate ] };
            auto const &other { Fastenings[ secondplate ] };
            auto const from { -reach( one, one.scale ).x };
            auto const to { apart + reach( other, other.scale ).y };
            if( reach( one, one.scale ).y + reach( other, other.scale ).x > apart ) {
                auto const &span { Model.spans[ part_plate ] };
                auto const turn { glm::dot( one.across, axis ) };
                auto const scale { ( to - from ) / ( span.y - span.x ) };
                one.pivot = 0.f;
                one.scale = static_cast<float>( scale );
                one.shift = static_cast<float>( ( turn > 0.0 ? from : -to ) - scale * span.x );
                dropped[ secondplate ] = true;
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

void fastened_rails::lift( std::vector<fastening> &Fastenings, std::vector<hit> const &Rails, beam const &Beam, beam_model const &Model ) {

    if( std::none_of( Rails.begin(), Rails.end(), []( hit const &Rail ) { return Rail.rise > 0.0; } ) ) {
        return;
    }
    auto const start { plan( Beam.from ) };
    auto const axis { glm::normalize( plan( Beam.to - Beam.from ) ) };
    auto const padpart { Model.parts[ part_slide ] != nullptr ? part_slide : part_plate };
    auto const &padspan { Model.spans[ padpart ] };
    auto const &padheights { Model.heights[ padpart ] };
    // a pad on the timber under the fastening, along its across from Low to High, up from Bottom to Top in the model
    auto const pad = [&]( fastening const &Over, double const Low, double const High, float const Bottom, float const Top ) {
        fastening result { padpart, Over.point, Over.across };
        result.scale = static_cast<float>( ( High - Low ) / ( padspan.y - padspan.x ) );
        result.shift = static_cast<float>( Low - result.scale * padspan.x );
        result.lift = Bottom - padheights.x;
        result.height = ( Top - Bottom ) / std::max( 0.001f, padheights.y - padheights.x );
        result.pad = true;
        return result; };
    // where along the timber the fastening reaches, from and to
    auto const footprint = [&]( fastening const &Fastening ) {
        auto const at { glm::dot( plan( Fastening.point ) - start, axis ) };
        auto const turn { glm::dot( Fastening.across, axis ) };
        auto const &span { Model.spans[ Fastening.part ] };
        auto const a { at + turn * Fastening.placed( span.x, Fastening.scale ) };
        auto const b { at + turn * Fastening.placed( span.y, Fastening.scale ) };
        return glm::dvec2 { std::min( a, b ), std::max( a, b ) }; };
    auto const holds = [&]( fastening const &Fastening, hit const &Rail ) {
        auto const reach { footprint( Fastening ) };
        return Fastening.part != part_chair && Rail.at >= reach.x && Rail.at <= reach.y; };
    std::vector<fastening> pads;
    for( auto &fastening : Fastenings ) {
        if( fastening.pad || fastening.part == part_chair ) {
            continue;
        }
        // moved up on a pad as wide as it as far as the least raised of the raised rails it holds, a rail not raised sunk in it
        auto lift { 0.0 };
        for( auto const &rail : Rails ) {
            if( rail.rise >= pad_least && holds( fastening, rail ) ) {
                lift = ( lift > 0.0 ? std::min( lift, rail.rise ) : rail.rise );
            }
        }
        if( lift <= 0.0 ) {
            continue;
        }
        auto const &span { Model.spans[ fastening.part ] };
        auto const a { fastening.placed( span.x, fastening.scale ) };
        auto const b { fastening.placed( span.y, fastening.scale ) };
        pads.push_back( pad( fastening, std::min( a, b ), std::max( a, b ), Model.bed, Model.heights[ fastening.part ].x + static_cast<float>( lift ) ) );
        fastening.lift = static_cast<float>( lift );
    }
    // a rail raised more than its fastening on a pad of its own on it, as wide as its foot, short of the rails beside
    for( std::size_t index = 0; index < Rails.size(); ++index ) {
        auto const &rail { Rails[ index ] };
        auto const under { std::find_if( Fastenings.begin(), Fastenings.end(), [&]( fastening const &Fastening ) {
            return false == Fastening.pad && holds( Fastening, rail ); } ) };
        if( under == Fastenings.end() || rail.rise - under->lift < pad_least ) {
            continue;
        }
        auto low { rail.foot };
        auto high { rail.foot };
        if( index > 0 ) {
            low = std::min( low, 0.5 * ( rail.at - Rails[ index - 1 ].at ) - fastening_gap );
        }
        if( index + 1 < Rails.size() ) {
            high = std::min( high, 0.5 * ( Rails[ index + 1 ].at - rail.at ) - fastening_gap );
        }
        if( low + high < pad_narrowest ) {
            continue;
        }
        fastening const at { padpart, Beam.from + ( Beam.to - Beam.from ) * ( rail.at / glm::length( plan( Beam.to - Beam.from ) ) ), axis };
        pads.push_back( pad( at, -low, high, Model.seat + under->lift, Model.seat + static_cast<float>( rail.rise ) ) );
    }
    Fastenings.insert( Fastenings.end(), pads.begin(), pads.end() );
}

std::vector<fastening_row> check_fastenings() {

    std::vector<fastening_row> result;
    for( auto *track : simulation::Paths.sequence() ) {
        if( track == nullptr || track->m_editorremoved || false == track->m_sleepers3d.active ) {
            continue;
        }
        auto const beams { place( *track ) };
        auto const *model { timber_model( *track, beams ) };
        if( model == nullptr || false == model->fastened() ) {
            continue;
        }
        fastening_row row;
        row.track = track->name();
        std::optional<fastened_rails> rails;
        for( auto const &beam : beams ) {
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
            auto const fastenings { rails->on( beam, *model ) };
            for( auto const &at : rails->bare( beam, *model, fastenings ) ) {
                row.bares.push_back( at );
            }
            for( auto const &fastening : fastenings ) {
                if( fastening.pad ) {
                    continue;
                }
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

std::vector<check_rail_gap> check_rail_seats() {

    // the check rails, by points along their middle
    struct check_line {
        check_rail_gap gap;
        std::vector<glm::dvec2> points;
        glm::dvec2 low { std::numeric_limits<double>::max() };
        glm::dvec2 high { std::numeric_limits<double>::lowest() };
    };
    std::vector<check_line> lines;
    for( auto *track : simulation::Paths.sequence() ) {
        if( track == nullptr || track->m_editorremoved || false == bedded( *track ) || false == track_rails::cut_rails( *track ) ) {
            continue;
        }
        auto const size { track->rail_dimensions() };
        auto const middle { 0.5 * std::abs( track->fTrackWidth ) + size.web };
        auto const &checks { track_rails::rail_checks( *track ) };
        for( int path = 0; path < path_count( *track ); ++path ) {
            auto const *segment { path_segment( *track, path ) };
            if( segment == nullptr ) {
                continue;
            }
            auto const length { segment->GetLength() };
            for( auto const side : { -1, 1 } ) {
                // the check rails and the guard rails of the crossings at wide angles
                auto spans { track->m_railguards[ path ][ side > 0 ? 1 : 0 ] };
                for( auto const &check : checks[ path ][ side > 0 ? 1 : 0 ] ) {
                    spans.emplace_back( check.from, check.to );
                }
                for( auto const &check : spans ) {
                    check_line line { { track->name(), path, side, check.x, check.y } };
                    auto const from { std::max( 0.0, check.x ) };
                    auto const to { std::min( length, check.y ) };
                    auto const count { static_cast<int>( std::ceil( ( to - from ) / fastening_step ) ) };
                    for( int step = 0; step <= count && to > from; ++step ) {
                        glm::dvec3 point, across;
                        segment->plan_frame( from + ( to - from ) * step / std::max( 1, count ), point, across );
                        auto const flat { plan( across ) };
                        if( glm::length( flat ) < 1e-9 ) {
                            continue;
                        }
                        auto const inward { glm::normalize( flat ) * static_cast<double>( -side ) };
                        auto const position { plan( point ) - inward * ( middle - check_seats ) };
                        line.points.push_back( position );
                        line.low = glm::min( line.low, position );
                        line.high = glm::max( line.high, position );
                    }
                    if( line.points.size() > 1 ) {
                        lines.push_back( std::move( line ) );
                    }
                }
            }
        }
    }
    // where each timber with the fastenings crosses a check rail, a seat of a fastening of the timber there
    std::vector<std::size_t> timbers( lines.size(), 0 );
    for( auto *track : simulation::Paths.sequence() ) {
        if( track == nullptr || track->m_editorremoved || false == track->m_sleepers3d.active ) {
            continue;
        }
        auto const beams { place( *track ) };
        auto const *model { timber_model( *track, beams ) };
        if( model == nullptr || false == model->fastened() ) {
            continue;
        }
        std::optional<fastened_rails> rails;
        for( auto const &beam : beams ) {
            if( beam.plain ) {
                continue;
            }
            if( false == rails.has_value() ) {
                rails.emplace( *track );
            }
            auto const start { plan( beam.from ) };
            auto const span { plan( beam.to - beam.from ) };
            auto const low { glm::min( start, start + span ) };
            auto const high { glm::max( start, start + span ) };
            std::vector<glm::dvec2> seats;
            auto const fastenings { rails->on( beam, *model ) };
            auto const under = [&]( glm::dvec2 const &Point, bool const Whole ) {
                return std::any_of( fastenings.begin(), fastenings.end(), [&]( fastening const &Fastening ) {
                    if( Whole && false == Fastening.whole ) {
                        return false;
                    }
                    auto const along { glm::dot( Point - plan( Fastening.point ), Fastening.across ) };
                    auto const a { Fastening.placed( model->spans[ Fastening.part ].x, Fastening.scale ) };
                    auto const b { Fastening.placed( model->spans[ Fastening.part ].y, Fastening.scale ) };
                    return along >= std::min( a, b ) && along <= std::max( a, b ); } ); };
            for( auto const &fastening : fastenings ) {
                if( fastening.pad ) {
                    continue;
                }
                seats.push_back( plan( fastening.point ) + fastening.across * fastening.placed( 0.0, fastening.scale ) );
                if( fastening.part == part_check ) {
                    seats.push_back( plan( fastening.point ) + fastening.across * fastening.placed( check_seats, fastening.scale ) );
                }
            }
            for( std::size_t index = 0; index < lines.size(); ++index ) {
                auto &line { lines[ index ] };
                if( line.high.x < low.x || line.low.x > high.x || line.high.y < low.y || line.low.y > high.y ) {
                    continue;
                }
                for( std::size_t point = 1; point < line.points.size(); ++point ) {
                    auto const chord { line.points[ point ] - line.points[ point - 1 ] };
                    double u, v;
                    if( false == track_rails::meet( start, span, line.points[ point - 1 ], chord, 1e-12, u, v ) || u < 0.0 || u > 1.0 || v < 0.0 || v >= 1.0 ) {
                        continue;
                    }
                    auto const at { start + span * u };
                    ++timbers[ index ];
                    // in a seat, or on a fastening holding all the rails over it
                    auto const seated { under( at, true ) || std::any_of( seats.begin(), seats.end(), [&]( glm::dvec2 const &Seat ) {
                        return glm::length( Seat - at ) < 0.03; } ) };
                    if( false == seated ) {
                        if( line.gap.bare == 0.0 ) {
                            line.gap.at = at;
                        }
                        line.gap.bare += 1.0;
                        if( false == under( at, false ) ) {
                            ++line.gap.unsupported;
                        }
                    }
                    break;
                }
            }
        }
    }
    std::vector<check_rail_gap> result;
    for( std::size_t index = 0; index < lines.size(); ++index ) {
        if( lines[ index ].gap.bare > 0.0 ) {
            lines[ index ].gap.timbers = timbers[ index ];
            result.push_back( lines[ index ].gap );
        }
    }
    return result;
}

} // namespace track_sleepers
