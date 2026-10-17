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
    auto const middle { top - 0.5 * result.height };
    auto webfrom { 1e9 };
    auto webto { -1e9 };
    for( std::size_t i = 0; i + 1 < count; ++i ) {
        auto const &a { Profile[ i ].position };
        auto const &b { Profile[ i + 1 ].position };
        if( a.y != b.y && ( a.y - middle ) * ( b.y - middle ) <= 0.0 ) {
            auto const x { a.x + ( b.x - a.x ) * ( middle - a.y ) / ( b.y - a.y ) };
            webfrom = std::min( webfrom, x );
            webto = std::max( webto, x );
        }
    }
    if( webfrom < webto ) {
        result.web = 0.5 * ( webfrom + webto );
        result.neck = top;
        for( std::size_t i = 0; i < count; ++i ) {
            auto const &point { Profile[ i ].position };
            if( point.y > middle && ( point.x < webfrom - 0.0005 || point.x > webto + 0.0005 ) ) {
                result.neck = std::min( result.neck, static_cast<double>( point.y ) );
            }
        }
    }
    result.head = 0.0;
    for( std::size_t i = 0; i < count; ++i ) {
        auto const &point { Profile[ i ].position };
        if( point.y > top - 0.3 * result.height ) {
            result.head = std::max( result.head, static_cast<double>( point.x ) );
        }
    }
    result.tread = result.head;
    auto flat { -1e9 };
    for( std::size_t i = 0; i < count; ++i ) {
        auto const &point { Profile[ i ].position };
        if( point.y > top - 0.0005 ) {
            flat = std::max( flat, static_cast<double>( point.x ) );
        }
    }
    result.shoulder = std::clamp( result.head - flat, 0.0, 0.5 * result.head );
    std::vector<std::size_t> heads, lips;
    for( std::size_t i = 0; i < count; ++i ) {
        auto const &point { Profile[ i ].position };
        if( point.y > top - 0.015 ) {
            ( point.x < -0.02 ? lips : heads ).push_back( i );
        }
    }
    if( heads.empty() || lips.empty() ) {
        return result;
    }
    auto const first { heads.back() < lips.front() ? heads.back() : lips.back() };
    auto const last { heads.back() < lips.front() ? lips.front() : heads.front() };
    if( first >= last || ( heads.back() > lips.front() && lips.back() > heads.front() ) ) {
        return result;
    }
    auto groove { 1e9 };
    for( auto i { first + 1 }; i < last; ++i ) {
        groove = std::min( groove, static_cast<double>( Profile[ i ].position.y ) );
    }
    if( groove > top ) {
        return result;
    }
    result.grooved = true;
    result.groove = groove;
    result.lip = -1e9;
    result.liptop = -1e9;
    for( auto const i : lips ) {
        result.lip = std::max( result.lip, static_cast<double>( Profile[ i ].position.x ) );
        result.liptop = std::max( result.liptop, static_cast<double>( Profile[ i ].position.y ) );
    }
    result.tread = -1e9;
    for( auto const i : heads ) {
        if( Profile[ i ].position.y > top - 0.0005 ) {
            result.tread = std::max( result.tread, static_cast<double>( Profile[ i ].position.x ) );
        }
    }
    result.ledge = 1e9;
    for( auto const i : lips ) {
        if( Profile[ i ].position.y > result.liptop - 0.0005 ) {
            result.ledge = std::min( result.ledge, static_cast<double>( Profile[ i ].position.x ) );
        }
    }
    result.rim = 1e9;
    result.crown = -1e9;
    auto const reach = [&]( double const X ) {
        result.rim = std::min( result.rim, X );
        result.crown = std::max( result.crown, X ); };
    for( std::size_t i = 0; i < count; ++i ) {
        auto const &point { Profile[ i ].position };
        if( point.y > groove + 1e-4 ) {
            reach( point.x );
        }
        if( i + 1 < count ) {
            auto const &next { Profile[ i + 1 ].position };
            if( ( point.y - groove ) * ( next.y - groove ) < 0.0 ) {
                reach( point.x + ( next.x - point.x ) * ( groove - point.y ) / ( next.y - point.y ) );
            }
        }
    }
    return result;
}

} // namespace track_rails
