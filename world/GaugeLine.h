/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "world/Segment.h"

namespace track_rails {

inline glm::dvec2 plan( glm::dvec3 const &V ) {

    return glm::dvec2{ V.x, V.z };
}

inline double cross( glm::dvec2 const &A, glm::dvec2 const &B ) {

    return A.x * B.y - A.y * B.x;
}

inline bool meet( glm::dvec2 const &A0, glm::dvec2 const &A, glm::dvec2 const &B0, glm::dvec2 const &B, double const Parallel, double &U, double &V ) {

    auto const denominator { cross( A, B ) };
    if( std::abs( denominator ) < Parallel ) {
        return false;
    }
    U = cross( B0 - A0, B ) / denominator;
    V = cross( B0 - A0, A ) / denominator;
    return true;
}

// gauge line of a rail: the running edge of its head, along a path and on past its ends along the tangents
struct gauge_line {
    TSegment const *segment { nullptr };
    double offset { 0.0 }; // across the path, to the running edge
    double from { 0.0 };
    std::vector<glm::dvec3> points;
    std::vector<double> distances;
    glm::dvec2 low { 0.0 };
    glm::dvec2 high { 0.0 };

    void at( double const Distance, glm::dvec3 &Point, glm::dvec3 &Tangent ) const {
        auto const length { segment->GetLength() };
        auto const local = [&]( double const D ) {
            glm::dvec3 point, across;
            segment->plan_frame( D, point, across );
            return point + across * offset; };
        auto const inside { std::clamp( Distance, 0.0, length ) };
        auto const delta { std::min( 0.05, 0.5 * length ) };
        auto const a { std::max( inside - delta, 0.0 ) };
        auto const b { std::min( inside + delta, length ) };
        Tangent = local( b ) - local( a );
        Tangent = ( glm::length2( Tangent ) > 0.0 ? glm::normalize( Tangent ) : glm::dvec3{ 1.0, 0.0, 0.0 } );
        Point = local( inside ) + Tangent * ( Distance - inside );
    }
    glm::dvec3 normal( double const Distance, int const Side ) const {
        glm::dvec3 point, across;
        segment->plan_frame( std::clamp( Distance, 0.0, segment->GetLength() ), point, across );
        return across * static_cast<double>( -Side );
    }
};

gauge_line make_gauge_line( TSegment const &Segment, double const Offset, double const From );
double lateral( gauge_line const &Line, int const Side, glm::dvec3 const &Point, double &Distance );
double nearest_distance( gauge_line const &Line, glm::dvec3 const &Point );

} // namespace track_rails
