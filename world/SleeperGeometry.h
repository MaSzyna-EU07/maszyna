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
#include <map>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "world/SleeperRules.h"
#include "world/TrackSleepers.h"

// geometry in plan, x and z of the world, the sleepers are laid out in
namespace track_sleepers {

inline glm::dvec2 plan( glm::dvec3 const &Vector ) {
    return { Vector.x, Vector.z };
}

inline double cross( glm::dvec2 const &Left, glm::dvec2 const &Right ) {
    return Left.x * Right.y - Left.y * Right.x;
}

// the angle between two directions in plan, either way along them
inline double angle_between( glm::dvec2 const &Left, glm::dvec2 const &Right ) {

    return std::acos( std::min( 1.0, std::abs( glm::dot( Left, Right ) ) ) );
}

// the direction turned by the angle, counterclockwise from x to z
inline glm::dvec2 rotated( glm::dvec2 const &Direction, double const Angle ) {

    return { Direction.x * std::cos( Angle ) - Direction.y * std::sin( Angle ), Direction.x * std::sin( Angle ) + Direction.y * std::cos( Angle ) };
}

// the angle the direction is turned off the forward way, either way along it
inline double turn_off( glm::dvec2 const &Forward, glm::dvec2 Direction ) {

    if( glm::dot( Forward, Direction ) < 0.0 ) {
        Direction = -Direction;
    }
    return std::atan2( cross( Forward, Direction ), glm::dot( Forward, Direction ) );
}

// a beam in plan, by its middle, the unit vector along it and half its length
struct beam_box {
    glm::dvec2 centre { 0.0 };
    glm::dvec2 axis { 1.0, 0.0 };
    double half { 0.0 };
};

inline beam_box box_of( beam const &Beam ) {

    auto const span { plan( Beam.to - Beam.from ) };
    auto const length { glm::length( span ) };
    return { plan( 0.5 * ( Beam.from + Beam.to ) ), length > 1e-9 ? span / length : glm::dvec2 { 1.0, 0.0 }, 0.5 * length };
}

// whether two beams keep clear of each other, with a little room left between them: an axis of either keeps them apart
inline bool apart( beam_box const &Left, beam_box const &Right ) {

    auto const offset { Right.centre - Left.centre };
    for( auto const &axis : { Left.axis, glm::dvec2 { -Left.axis.y, Left.axis.x }, Right.axis, glm::dvec2 { -Right.axis.y, Right.axis.x } } ) {
        auto const reach = [&]( beam_box const &Box ) {
            return Box.half * std::abs( glm::dot( Box.axis, axis ) ) + ( half_width - 0.01 ) * std::abs( cross( Box.axis, axis ) ); };
        if( std::abs( glm::dot( offset, axis ) ) > reach( Left ) + reach( Right ) ) {
            return true;
        }
    }
    return false;
}

// whether the point lies on the beam
inline bool holds( beam_box const &Box, glm::dvec2 const &Point ) {

    auto const offset { Point - Box.centre };
    return std::abs( glm::dot( offset, Box.axis ) ) <= Box.half && std::abs( cross( Box.axis, offset ) ) <= half_width;
}

// values put in the square cells of a grid in plan that a square about a point reaches into, to find those near a place
template <typename Value>
class plan_grid {

public:
    using cell = std::pair<std::int64_t, std::int64_t>;

    explicit plan_grid( double const Size ) :
        m_size( Size ) {}

    // the value in every cell the square of the reach about the centre touches
    void insert( glm::dvec2 const &Centre, double const Reach, Value const &Inserted ) {

        auto const low { key( Centre - glm::dvec2 { Reach } ) };
        auto const high { key( Centre + glm::dvec2 { Reach } ) };
        for( auto x { low.first }; x <= high.first; ++x ) {
            for( auto z { low.second }; z <= high.second; ++z ) {
                m_cells[ { x, z } ].push_back( Inserted );
            }
        }
    }
    // the values in the cell of the point, null where there are none
    std::vector<Value> const *at( glm::dvec2 const &Point ) const {

        auto const found { m_cells.find( key( Point ) ) };
        return ( found != m_cells.end() ? &found->second : nullptr );
    }
    // whether the test holds for every value in the cells the square of the reach about the centre touches
    template <typename Test>
    bool all_about( glm::dvec2 const &Centre, double const Reach, Test const &Holds ) const {

        auto const low { key( Centre - glm::dvec2 { Reach } ) };
        auto const high { key( Centre + glm::dvec2 { Reach } ) };
        for( auto x { low.first }; x <= high.first; ++x ) {
            for( auto z { low.second }; z <= high.second; ++z ) {
                auto const found { m_cells.find( { x, z } ) };
                if( found != m_cells.end() && false == std::all_of( found->second.begin(), found->second.end(), Holds ) ) {
                    return false;
                }
            }
        }
        return true;
    }
    std::map<cell, std::vector<Value>> const &cells() const {
        return m_cells;
    }

private:
    cell key( glm::dvec2 const &Point ) const {
        return { static_cast<std::int64_t>( std::floor( Point.x / m_size ) ), static_cast<std::int64_t>( std::floor( Point.y / m_size ) ) };
    }

    double m_size;
    std::map<cell, std::vector<Value>> m_cells;
};


} // namespace track_sleepers
