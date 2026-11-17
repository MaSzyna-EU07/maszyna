/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"

#include "world/TrackRails.h"

namespace track_rails {

namespace {

// where the outline of the profile crosses the middle of its height, how far the web reaches across, and from where up the head widens out
// of it
void measure_web( gfx::vertex_array const &Profile, std::size_t const Count, double const Top, rail_size &Result ) {

    auto const middle { Top - 0.5 * Result.height };
    auto webfrom { 1e9 };
    auto webto { -1e9 };
    for( std::size_t i = 0; i + 1 < Count; ++i ) {
        auto const &a { Profile[ i ].position };
        auto const &b { Profile[ i + 1 ].position };
        if( a.y != b.y && ( a.y - middle ) * ( b.y - middle ) <= 0.0 ) {
            auto const x { a.x + ( b.x - a.x ) * ( middle - a.y ) / ( b.y - a.y ) };
            webfrom = std::min( webfrom, x );
            webto = std::max( webto, x );
        }
    }
    if( false == ( webfrom < webto ) ) {
        return;
    }
    Result.web = 0.5 * ( webfrom + webto );
    Result.neck = Top;
    for( std::size_t i = 0; i < Count; ++i ) {
        auto const &point { Profile[ i ].position };
        if( point.y > middle && ( point.x < webfrom - 0.0005 || point.x > webto + 0.0005 ) ) {
            Result.neck = std::min( Result.neck, static_cast<double>( point.y ) );
        }
    }
}

// how far the head reaches out, and how far in from its outer edge its top is rounded off
void measure_head( gfx::vertex_array const &Profile, std::size_t const Count, double const Top, rail_size &Result ) {

    Result.head = 0.0;
    for( std::size_t i = 0; i < Count; ++i ) {
        auto const &point { Profile[ i ].position };
        if( point.y > Top - 0.3 * Result.height ) {
            Result.head = std::max( Result.head, static_cast<double>( point.x ) );
        }
    }
    Result.tread = Result.head;
    auto flat { -1e9 };
    for( std::size_t i = 0; i < Count; ++i ) {
        auto const &point { Profile[ i ].position };
        if( point.y > Top - 0.0005 ) {
            flat = std::max( flat, static_cast<double>( point.x ) );
        }
    }
    Result.shoulder = std::clamp( Result.head - flat, 0.0, 0.5 * Result.head );
}

// the lip of a grooved rail across the groove from the head: how far it reaches and how high, and where the tops of the head and the lip
// end toward the groove
void measure_lip( gfx::vertex_array const &Profile, std::vector<std::size_t> const &Heads, std::vector<std::size_t> const &Lips, double const Top, rail_size &Result ) {

    Result.lip = -1e9;
    Result.liptop = -1e9;
    for( auto const i : Lips ) {
        Result.lip = std::max( Result.lip, static_cast<double>( Profile[ i ].position.x ) );
        Result.liptop = std::max( Result.liptop, static_cast<double>( Profile[ i ].position.y ) );
    }
    Result.tread = -1e9;
    for( auto const i : Heads ) {
        if( Profile[ i ].position.y > Top - 0.0005 ) {
            Result.tread = std::max( Result.tread, static_cast<double>( Profile[ i ].position.x ) );
        }
    }
    Result.ledge = 1e9;
    for( auto const i : Lips ) {
        if( Profile[ i ].position.y > Result.liptop - 0.0005 ) {
            Result.ledge = std::min( Result.ledge, static_cast<double>( Profile[ i ].position.x ) );
        }
    }
}

// how far across the profile reaches above the bottom of the groove, on either side
void measure_reach( gfx::vertex_array const &Profile, std::size_t const Count, double const Groove, rail_size &Result ) {

    Result.rim = 1e9;
    Result.crown = -1e9;
    auto const reach = [ &Result ]( double const X ) {
        Result.rim = std::min( Result.rim, X );
        Result.crown = std::max( Result.crown, X ); };
    for( std::size_t i = 0; i < Count; ++i ) {
        auto const &point { Profile[ i ].position };
        if( point.y > Groove + 1e-4 ) {
            reach( point.x );
        }
        if( i + 1 >= Count ) {
            continue;
        }
        if( auto const &next { Profile[ i + 1 ].position };
            ( point.y - Groove ) * ( next.y - Groove ) < 0.0 ) {
            reach( point.x + ( next.x - point.x ) * ( Groove - point.y ) / ( next.y - point.y ) );
        }
    }
}

// the groove of a tram rail, between the head and the lip beside it: left as none where the profile has none
void measure_groove( gfx::vertex_array const &Profile, std::size_t const Count, double const Top, rail_size &Result ) {

    std::vector<std::size_t> heads;
    std::vector<std::size_t> lips;
    for( std::size_t i = 0; i < Count; ++i ) {
        auto const &point { Profile[ i ].position };
        if( point.y > Top - 0.015 ) {
            ( point.x < -0.02 ? lips : heads ).push_back( i );
        }
    }
    if( heads.empty() || lips.empty() ) {
        return;
    }
    auto const first { heads.back() < lips.front() ? heads.back() : lips.back() };
    auto const last { heads.back() < lips.front() ? lips.front() : heads.front() };
    if( first >= last || ( heads.back() > lips.front() && lips.back() > heads.front() ) ) {
        return;
    }
    auto groove { 1e9 };
    for( auto i { first + 1 }; i < last; ++i ) {
        groove = std::min( groove, static_cast<double>( Profile[ i ].position.y ) );
    }
    if( groove > Top ) {
        return;
    }
    Result.grooved = true;
    Result.groove = groove;
    measure_lip( Profile, heads, lips, Top, Result );
    measure_reach( Profile, Count, groove, Result );
}

} // namespace

rail_size measure_rail( gfx::vertex_array const &Profile ) {

    rail_size result;
    auto const count { Profile.size() / 2 };
    if( count < 2 ) {
        return result;
    }
    auto top { -1e9 };
    auto bottom { 1e9 };
    result.low = 1e9;
    result.high = -1e9;
    for( std::size_t i = 0; i < count; ++i ) {
        auto const &point { Profile[ i ].position };
        top = std::max( top, static_cast<double>( point.y ) );
        bottom = std::min( bottom, static_cast<double>( point.y ) );
        result.low = std::min( result.low, static_cast<double>( point.x ) );
        result.high = std::max( result.high, static_cast<double>( point.x ) );
    }
    result.height = top - bottom;
    result.crest = top;
    measure_web( Profile, count, top, result );
    measure_head( Profile, count, top, result );
    measure_groove( Profile, count, top, result );
    return result;
}

} // namespace track_rails
