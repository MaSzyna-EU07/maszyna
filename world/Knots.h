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
#include <vector>
#include <glm/glm.hpp>

namespace knots {

using line = std::vector<glm::dvec2>;

inline bool by_position( glm::dvec2 const &A, glm::dvec2 const &B ) {

    return A.x < B.x;
}

inline void sort( line &Line ) {

    std::sort( Line.begin(), Line.end(), by_position );
}

inline double sample( line const &Line, double const Position, double const Default ) {

    if( Line.empty() ) {
        return Default;
    }
    if( Position <= Line.front().x ) {
        return Line.front().y;
    }
    // the first knot at or past the position, after the first one
    auto const next { std::lower_bound( Line.begin() + 1, Line.end(), Position, []( glm::dvec2 const &Knot, double const At ) { return Knot.x < At; } ) };
    if( next == Line.end() ) {
        return Line.back().y;
    }
    auto const &before { *( next - 1 ) };
    return glm::mix( before.y, next->y, ( Position - before.x ) / std::max( next->x - before.x, 1e-6 ) );
}

inline line shifted( line Line, double const Value ) {

    for( auto &knot : Line ) {
        knot.y += Value;
    }
    return Line;
}

inline std::vector<double> stations( line const &A, line const &B ) {

    std::vector<double> result;
    for( auto const *each : { &A, &B } ) {
        for( auto const &knot : *each ) {
            result.push_back( knot.x );
        }
    }
    std::sort( result.begin(), result.end() );
    return result;
}

inline std::vector<double> meetings( line const &A, line const &B ) {

    std::vector<double> result;
    if( A.empty() || B.empty() ) {
        return result;
    }
    auto const all { stations( A, B ) };
    for( std::size_t i = 1; i < all.size(); ++i ) {
        auto const before { sample( A, all[ i - 1 ], 0.0 ) - sample( B, all[ i - 1 ], 0.0 ) };
        auto const after { sample( A, all[ i ], 0.0 ) - sample( B, all[ i ], 0.0 ) };
        if( before * after < 0.0 ) {
            result.push_back( all[ i - 1 ] + ( all[ i ] - all[ i - 1 ] ) * before / ( before - after ) );
        }
    }
    return result;
}

inline line bound( line const &A, line const &B, bool const Lower ) {

    if( A.empty() || B.empty() ) {
        return ( A.empty() ? B : A );
    }
    auto all { stations( A, B ) };
    auto const crossed { meetings( A, B ) };
    all.insert( all.end(), crossed.begin(), crossed.end() );
    std::sort( all.begin(), all.end() );
    line result;
    for( auto const position : all ) {
        auto const a { sample( A, position, 0.0 ) };
        auto const b { sample( B, position, 0.0 ) };
        result.emplace_back( position, Lower ? std::min( a, b ) : std::max( a, b ) );
    }
    return result;
}

inline line middle( line const &A, line const &B ) {

    line result;
    for( auto const position : stations( A, B ) ) {
        result.emplace_back( position, 0.5 * ( sample( A, position, 0.0 ) + sample( B, position, 0.0 ) ) );
    }
    return result;
}

// both lines added up, each going on flat past its ends
inline line sum( line const &A, line const &B ) {

    if( A.empty() || B.empty() ) {
        return ( A.empty() ? B : A );
    }
    line result;
    for( auto const position : stations( A, B ) ) {
        if( false == result.empty() && position - result.back().x < 1e-9 ) {
            continue;
        }
        result.emplace_back( position, sample( A, position, 0.0 ) + sample( B, position, 0.0 ) );
    }
    return result;
}

// the line without the knots lying on the straight between the knots kept around them, within Tolerance
inline line simplified( line const &Line, double const Tolerance ) {

    if( Line.size() < 3 ) {
        return Line;
    }
    line result { Line.front() };
    std::size_t kept { 0 };
    for( std::size_t i = 2; i < Line.size(); ++i ) {
        auto const &a { Line[ kept ] };
        auto const &b { Line[ i ] };
        for( std::size_t j = kept + 1; j < i; ++j ) {
            auto const t { ( Line[ j ].x - a.x ) / std::max( b.x - a.x, 1e-9 ) };
            if( std::abs( glm::mix( a.y, b.y, t ) - Line[ j ].y ) > Tolerance ) {
                kept = i - 1;
                result.push_back( Line[ kept ] );
                break;
            }
        }
    }
    result.push_back( Line.back() );
    return result;
}

} // namespace knots
