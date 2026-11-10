/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"

#include "world/GaugeLine.h"

namespace track_rails {

gauge_line make_gauge_line( TSegment const &Segment, double const Offset, double const From ) {

    gauge_line line;
    line.segment = &Segment;
    line.offset = Offset;
    line.from = From;
    auto const length { Segment.GetLength() };
    auto const add = [&]( double const Distance ) {
        glm::dvec3 point, tangent;
        line.at( Distance, point, tangent );
        line.points.push_back( point );
        line.distances.push_back( Distance ); };
    auto const count { std::max( { 1, Segment.RaSegCount(), static_cast<int>( std::ceil( ( length - From ) / 5.0 ) ) } ) };
    for( int i = 0; i <= count; ++i ) {
        add( From + ( length - From ) * i / count );
    }
    line.low = line.high = plan( line.points.front() );
    for( auto const &point : line.points ) {
        line.low = glm::min( line.low, plan( point ) );
        line.high = glm::max( line.high, plan( point ) );
    }
    return line;
}

double lateral( gauge_line const &Line, int const Side, glm::dvec3 const &Point, double &Distance ) {

    glm::dvec3 point, tangent;
    for( int step = 0; step < 8; ++step ) {
        Line.at( Distance, point, tangent );
        Distance += glm::dot( plan( Point - point ), glm::normalize( plan( tangent ) ) );
    }
    Line.at( Distance, point, tangent );
    return -glm::dot( plan( Point - point ), plan( Line.normal( Distance, Side ) ) );
}

double nearest_distance( gauge_line const &Line, glm::dvec3 const &Point ) {

    auto result { Line.distances.front() };
    auto nearest { std::numeric_limits<double>::max() };
    for( std::size_t i = 0; i < Line.points.size(); ++i ) {
        auto const gap { glm::length( plan( Line.points[ i ] - Point ) ) };
        if( gap < nearest ) {
            nearest = gap;
            result = Line.distances[ i ];
        }
    }
    return result;
}

} // namespace track_rails
