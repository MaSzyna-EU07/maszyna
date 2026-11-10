/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"

#include "world/SleeperFeatures.h"
#include "world/SleeperGeometry.h"
#include "world/SleeperRules.h"
#include "world/Track.h"

namespace track_sleepers {

namespace {

// how far past the timbers the sleepers turn back square to the tracks, by the angle the timbers are turned off them
double turn_length( double const Angle ) {

    return ( Angle <= turn_max ? 0.05 * pitch : pitch * std::ceil( Angle / turn_max ) );
}

// the frog of the switch: where its paths are a gauge apart, from the samples of the diverging path against the main one
void find_frog( sampled_path const &Main, sampled_path const &Diverging, std::vector<feature> &Features ) {

    auto previousgap { 0.0 };
    for( std::size_t index = 1; index < Diverging.points.size(); ++index ) {
        double gap;
        glm::dvec2 direction;
        if( false == beside( Main, Diverging.points[ index ], Main.gauge + 1.0, gap, direction ) ) {
            return;
        }
        if( gap < Main.gauge ) {
            previousgap = gap;
            continue;
        }
        // between the samples, where the paths are a gauge apart
        auto const share { ( Main.gauge - previousgap ) / std::max( 1e-6, gap - previousgap ) };
        auto const distance { glm::mix( Diverging.distances[ index - 1 ], Diverging.distances[ index ], share ) };
        glm::dvec3 point;
        glm::vec3 angles;
        Diverging.segment->RaPositionGet( distance, point, angles );
        glm::dvec2 mainpoint, maindirection;
        auto const mainat { nearest_on( Main, plan( point ), mainpoint, maindirection ) };
        auto const divergingdirection { heading( Diverging, distance ) };
        if( glm::dot( maindirection, divergingdirection ) < 0.0 ) {
            maindirection = -maindirection;
        }
        auto const halfangle { 0.5 * angle_between( maindirection, divergingdirection ) };
        if( halfangle < 0.002 ) {
            return;
        }
        feature frog;
        // the frog lies midway between the paths
        frog.point = 0.5 * ( mainpoint + plan( point ) );
        frog.bisector = glm::normalize( maindirection + divergingdirection );
        frog.sine = std::sin( halfangle );
        frog.spacing = Main.gauge;
        frog.from = -frog_ramp;
        frog.full = 0.0;
        // till the plain sleepers of the paths going apart wouldn't touch any more
        frog.until = std::max( 0.0, ( Main.half + Diverging.half + end_gap - Main.gauge ) / ( 2.0 * frog.sine ) );
        frog.back = frog.until + turn_length( halfangle );
        frog.paths = { &Main, &Diverging };
        frog.at = { mainat, distance };
        frog.frog = true;
        Features.push_back( frog );
        return;
    }
}

// where two paths of different tracks cross, from their samples
void find_crossing( sampled_path const &Left, sampled_path const &Right, std::vector<feature> &Features ) {

    if( Left.low.x > Right.high.x || Right.low.x > Left.high.x || Left.low.y > Right.high.y || Right.low.y > Left.high.y ) {
        return;
    }
    for( std::size_t left = 1; left < Left.points.size(); ++left ) {
        auto const a0 { plan( Left.points[ left - 1 ] ) };
        auto const a { plan( Left.points[ left ] ) - a0 };
        if( std::max( a0.x, a0.x + a.x ) < Right.low.x || std::min( a0.x, a0.x + a.x ) > Right.high.x
         || std::max( a0.y, a0.y + a.y ) < Right.low.y || std::min( a0.y, a0.y + a.y ) > Right.high.y ) {
            continue;
        }
        for( std::size_t right = 1; right < Right.points.size(); ++right ) {
            auto const b0 { plan( Right.points[ right - 1 ] ) };
            auto const b { plan( Right.points[ right ] ) - b0 };
            auto const denominator { cross( a, b ) };
            if( std::abs( denominator ) < 1e-9 ) {
                continue;
            }
            auto const u { cross( b0 - a0, b ) / denominator };
            auto const v { cross( b0 - a0, a ) / denominator };
            if( u < 0.0 || u >= 1.0 || v < 0.0 || v >= 1.0 ) {
                continue;
            }
            auto const leftat { glm::mix( Left.distances[ left - 1 ], Left.distances[ left ], u ) };
            auto const rightat { glm::mix( Right.distances[ right - 1 ], Right.distances[ right ], v ) };
            if( leftat < 0.05 || leftat > Left.length - 0.05 || rightat < 0.05 || rightat > Right.length - 0.05 ) {
                // tracks joined end to end, or one ending on the other
                continue;
            }
            auto const levels { glm::mix( Left.points[ left - 1 ].y, Left.points[ left ].y, u ) - glm::mix( Right.points[ right - 1 ].y, Right.points[ right ].y, v ) };
            if( std::abs( levels ) > level_tolerance ) {
                continue;
            }
            auto const leftdirection { glm::normalize( a ) };
            auto rightdirection { glm::normalize( b ) };
            if( glm::dot( leftdirection, rightdirection ) < 0.0 ) {
                rightdirection = -rightdirection;
            }
            auto const angle { angle_between( leftdirection, rightdirection ) };
            if( angle < 0.01 ) {
                continue;
            }
            feature crossing;
            crossing.point = a0 + a * u;
            crossing.bisector = glm::normalize( leftdirection + rightdirection );
            crossing.sine = std::sin( 0.5 * angle );
            crossing.until = std::max( touch_spacing( Left, Right, angle ), touch_spacing( Right, Left, angle ) ) / ( 2.0 * crossing.sine );
            crossing.back = crossing.until + turn_length( 0.5 * angle );
            crossing.from = -crossing.back;
            crossing.full = -crossing.until;
            crossing.paths = { &Left, &Right };
            crossing.at = { leftat, rightat };
            crossing.grid = ( angle >= grid_angle );
            Features.push_back( crossing );
        }
    }
}

// whether another path goes on from the end of the path, so the beams there go on too
bool goes_on( sampled_path const &Path, glm::dvec3 const &End, std::vector<sampled_path> const &Around ) {

    for( auto const &other : Around ) {
        if( other.track == Path.track ) {
            continue;
        }
        if( glm::length( plan( other.points.front() - End ) ) < 0.05 || glm::length( plan( other.points.back() - End ) ) < 0.05 ) {
            return true;
        }
    }
    return false;
}

// the stretch of the timbers square to the bisector ends where either path ends with nothing going on from it
void clamp_to_ends( feature &Feature, std::vector<sampled_path> const &Around ) {

    for( auto const *path : Feature.paths ) {
        for( auto const *end : { &path->points.front(), &path->points.back() } ) {
            if( goes_on( *path, *end, Around ) ) {
                continue;
            }
            auto const along { glm::dot( plan( *end ) - Feature.point, Feature.bisector ) };
            if( along > 0.0 ) {
                Feature.until = std::max( Feature.full, std::min( Feature.until, along ) );
            }
            else if( false == Feature.frog ) {
                Feature.full = std::min( Feature.until, std::max( Feature.full, along ) );
            }
        }
    }
    auto const turn { turn_length( std::asin( Feature.sine ) ) };
    Feature.back = Feature.until + turn;
    if( false == Feature.frog ) {
        Feature.from = Feature.full - turn;
    }
}

} // namespace

// the frogs of the switches and the crossings of the paths around
std::vector<feature> find_features( std::vector<sampled_path> const &Around ) {

    std::vector<feature> result;
    for( std::size_t index = 0; index < Around.size(); ++index ) {
        auto const &path { Around[ index ] };
        if( path.track->eType == tt_Switch && path.path == 0 && index + 1 < Around.size() && Around[ index + 1 ].track == path.track ) {
            find_frog( path, Around[ index + 1 ], result );
        }
        for( std::size_t other = index + 1; other < Around.size(); ++other ) {
            if( Around[ other ].track != path.track ) {
                find_crossing( path, Around[ other ], result );
            }
        }
    }
    for( auto &feature : result ) {
        clamp_to_ends( feature, Around );
    }
    // the frogs of the switches of a crossover between tracks running alongside each other: the diverging tracks of both lie on one
    // line, the main ones side by side
    auto const way = []( sampled_path const &Path, double const Distance ) {
        return heading( Path, Distance ); };
    for( auto &left : result ) {
        for( auto &right : result ) {
            if( &left == &right || false == left.frog || false == right.frog || left.paths[ 0 ]->track == right.paths[ 0 ]->track ) {
                continue;
            }
            auto const leftmain { way( *left.paths[ 0 ], left.at[ 0 ] ) };
            auto const rightmain { way( *right.paths[ 0 ], right.at[ 0 ] ) };
            auto const leftdiverging { way( *left.paths[ 1 ], left.at[ 1 ] ) };
            auto const rightdiverging { way( *right.paths[ 1 ], right.at[ 1 ] ) };
            auto const apart { std::abs( cross( leftmain, right.point - left.point ) ) };
            if( angle_between( leftmain, rightmain ) < 0.035 && angle_between( leftdiverging, rightdiverging ) < 0.035 && apart > 3.0 && apart < 8.0 ) {
                left.crossover = true;
                right.crossover = true;
            }
        }
    }
    return result;
}

// which way the beam at the point lies square to: the path laying it, turned toward the bisectors of the frogs and crossings
// about the point. this depends only on where the point is, so the beams of paths laid one after another turn smoothly
glm::dvec2 beam_direction( glm::dvec3 const &Point, glm::dvec2 const &Forward, std::vector<feature> const &Features ) {

    auto const point { plan( Point ) };
    auto weight { 0.0 };
    glm::dvec2 bisectors { 0.0 };
    for( auto const &feature : Features ) {
        if( feature.grid || feature.crossover ) {
            continue;
        }
        auto const offset { point - feature.point };
        auto const along { glm::dot( offset, feature.bisector ) };
        if( along <= feature.from || along >= feature.back ) {
            continue;
        }
        auto turned {
            along < feature.full ? glm::smoothstep( feature.from, feature.full, along ) :
            along > feature.until ? 1.0 - glm::smoothstep( feature.until, feature.back, along ) :
            1.0 };
        // only the beams under the paths of the frog or the crossing turn, not those of the tracks beside them
        auto const spacing { feature.spacing + 2.0 * feature.sine * ( feature.frog ? std::max( 0.0, along ) : std::abs( along ) ) };
        auto const reach { 0.5 * spacing + feature.paths[ 0 ]->reach + 0.3 };
        auto const lateral { std::abs( cross( feature.bisector, offset ) ) };
        turned *= 1.0 - glm::smoothstep( reach, reach + 0.5, lateral );
        if( turned <= 0.0 ) {
            continue;
        }
        weight = std::max( weight, turned );
        bisectors += turned * ( glm::dot( feature.bisector, Forward ) >= 0.0 ? feature.bisector : -feature.bisector );
    }
    if( weight <= 0.0 || glm::length( bisectors ) < 1e-9 ) {
        return Forward;
    }
    return glm::normalize( Forward * ( 1.0 - weight ) + glm::normalize( bisectors ) * weight );
}


} // namespace track_sleepers
