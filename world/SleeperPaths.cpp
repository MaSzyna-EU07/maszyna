/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"

#include "world/SleeperPaths.h"
#include "world/SleeperGeometry.h"
#include "world/SleeperRules.h"
#include "world/TrackSleepers.h"
#include "world/Track.h"
#include "simulation/simulation.h"
#include "scene/scene.h"

namespace track_sleepers {

int path_count( TTrack const &Track ) {
    return ( Track.eType == tt_Switch ? 2 : 1 );
}

TSegment const *path_segment( TTrack const &Track, int const Path ) {
    return ( Track.eType == tt_Switch ? Track.SwitchExtension->Segments[ Path ].get() : Track.Segment.get() );
}

// how long the line of tracks joined end to end the plain track is part of, so a line keeps laying the timbers where it runs
// past the tracks of another, from one track of it to the next. the line goes on through the main path of a switch, and ends
// where it comes to its diverging path
namespace {

double line_length( TTrack const &Track ) {

    if( Track.eType != tt_Normal || Track.Segment == nullptr ) {
        return 0.0;
    }
    auto result { Track.Segment->GetLength() };
    for( auto *next : { Track.trPrev, Track.trNext } ) {
        TTrack const *previous { &Track };
        auto const *current { next };
        for( int count = 0; current != nullptr && current != &Track && count < 1000; ++count ) {
            TTrack const *following { nullptr };
            if( current->eType == tt_Normal && current->Segment != nullptr ) {
                result += current->Segment->GetLength();
                following = ( current->trPrev == previous ? current->trNext : current->trPrev );
            }
            else if( current->eType == tt_Switch && current->SwitchExtension != nullptr && current->SwitchExtension->Segments[ 0 ] ) {
                auto const &links { *current->SwitchExtension };
                if( links.pNexts[ 0 ] == previous ) {
                    following = links.pPrevs[ 0 ];
                }
                else if( links.pPrevs[ 0 ] == previous || links.pPrevs[ 1 ] == previous ) {
                    following = links.pNexts[ 0 ];
                }
                else {
                    // come to the diverging path
                    break;
                }
                result += links.Segments[ 0 ]->GetLength();
            }
            else {
                break;
            }
            previous = current;
            current = following;
        }
    }
    return result;
}

} // namespace

sampled_path sample( TTrack &Track, int const Path ) {

    sampled_path result;
    result.track = &Track;
    result.path = Path;
    result.segment = path_segment( Track, Path );
    if( result.segment == nullptr ) {
        return result;
    }
    result.length = result.segment->GetLength();
    result.gauge = std::abs( Track.fTrackWidth );
    result.rails = 0.5 * result.gauge + rail_reach;
    result.reach = result.rails + overhang;
    result.half = 0.5 * plain_length( Track );
    result.line = line_length( Track );
    auto const count { std::max( 1, static_cast<int>( std::ceil( result.length / sample_step ) ) ) };
    result.low = glm::dvec2 { std::numeric_limits<double>::max() };
    result.high = glm::dvec2 { std::numeric_limits<double>::lowest() };
    for( int step = 0; step <= count; ++step ) {
        auto const distance { result.length * step / count };
        glm::dvec3 point;
        glm::vec3 angles;
        result.segment->RaPositionGet( distance, point, angles );
        if( std::abs( angles.x ) > 1e-6 || ( false == result.points.empty() && std::abs( point.y - result.points.front().y ) > 1e-4 ) ) {
            result.level = false;
        }
        result.points.emplace_back( point );
        result.distances.emplace_back( distance );
        result.low = glm::min( result.low, plan( point ) );
        result.high = glm::max( result.high, plan( point ) );
    }
    result.feet = result.points.front().y - Track.rail_height();
    return result;
}

// the direction of the path in plan by its samples about the distance
glm::dvec2 heading( sampled_path const &Path, double const Distance ) {

    auto const count { Path.points.size() };
    auto index { static_cast<std::size_t>( std::upper_bound( Path.distances.begin(), Path.distances.end(), Distance ) - Path.distances.begin() ) };
    index = std::clamp<std::size_t>( index, 1, count - 1 );
    auto const direction { plan( Path.points[ index ] - Path.points[ index - 1 ] ) };
    auto const length { glm::length( direction ) };
    return ( length > 1e-9 ? direction / length : glm::dvec2 { 0.0, 1.0 } );
}

// which of two paths lays the beams where they're merged: a switch before a plain track, the main path of a switch
// before the diverging one, a plain track of a longer line before one of a shorter, the longer before the shorter, and
// then by the name
bool before( sampled_path const &Left, sampled_path const &Right ) {

    auto const leftswitch { Left.track->eType == tt_Switch };
    auto const rightswitch { Right.track->eType == tt_Switch };
    if( leftswitch != rightswitch ) {
        return leftswitch;
    }
    if( Left.track == Right.track ) {
        return Left.path < Right.path;
    }
    if( Left.path != Right.path ) {
        return Left.path < Right.path;
    }
    if( std::abs( Left.line - Right.line ) > 0.001 ) {
        return Left.line > Right.line;
    }
    if( std::abs( Left.length - Right.length ) > 0.001 ) {
        return Left.length > Right.length;
    }
    if( Left.track->name() != Right.track->name() ) {
        return Left.track->name() < Right.track->name();
    }
    return Left.track < Right.track;
}

bool near_bounds( sampled_path const &Path, glm::dvec3 const &Point, double const Distance ) {

    return Point.x > Path.low.x - Distance && Point.x < Path.high.x + Distance
        && Point.z > Path.low.y - Distance && Point.z < Path.high.y + Distance;
}

// how far the point is from the path in plan and which way the path runs there, when it's beside the path closer than the
// distance and at about the same level. a point past either end of the path isn't beside it, so a track going on from the end
// of another doesn't count as running beside it
bool beside( sampled_path const &Path, glm::dvec3 const &Point, double const Distance, double &Gap, glm::dvec2 &Direction ) {

    if( Path.points.size() < 2 || false == near_bounds( Path, Point, Distance ) ) {
        return false;
    }
    auto nearest { Distance * Distance };
    auto found { false };
    auto past { false }; // the nearest place of the path is its end, with the point beyond it
    auto const last { Path.points.size() - 2 };
    for( std::size_t index = 0; index <= last; ++index ) {
        auto const &from { Path.points[ index ] };
        auto const &to { Path.points[ index + 1 ] };
        auto const chord { plan( to - from ) };
        auto const offset { plan( Point - from ) };
        auto const chordlength2 { glm::dot( chord, chord ) };
        auto const along { chordlength2 > 0.0 ? glm::dot( offset, chord ) / chordlength2 : 0.0 };
        auto const clamped { std::clamp( along, 0.0, 1.0 ) };
        auto const gap { offset - chord * clamped };
        auto const gap2 { glm::dot( gap, gap ) };
        if( gap2 >= nearest ) {
            continue;
        }
        auto const level { from.y + ( to.y - from.y ) * clamped };
        if( std::abs( level - Point.y ) > level_tolerance ) {
            continue;
        }
        nearest = gap2;
        found = true;
        past = ( ( index == 0 && along < 1e-6 ) || ( index == last && along > 1.0 - 1e-6 ) );
        if( chordlength2 > 0.0 ) {
            Direction = chord / std::sqrt( chordlength2 );
        }
    }
    Gap = std::sqrt( nearest );
    return found && false == past;
}

// the place of the path nearest to the point in plan: its distance along the path, where it is and which way the path runs
double nearest_on( sampled_path const &Path, glm::dvec2 const &Point, glm::dvec2 &Position, glm::dvec2 &Direction ) {

    auto nearest { std::numeric_limits<double>::max() };
    auto result { 0.0 };
    for( std::size_t index = 1; index < Path.points.size(); ++index ) {
        auto const from { plan( Path.points[ index - 1 ] ) };
        auto const chord { plan( Path.points[ index ] ) - from };
        auto const chordlength2 { glm::dot( chord, chord ) };
        if( chordlength2 <= 0.0 ) {
            continue;
        }
        auto const along { std::clamp( glm::dot( Point - from, chord ) / chordlength2, 0.0, 1.0 ) };
        auto const position { from + chord * along };
        auto const gap2 { glm::dot( Point - position, Point - position ) };
        if( gap2 < nearest ) {
            nearest = gap2;
            result = glm::mix( Path.distances[ index - 1 ], Path.distances[ index ], along );
            Position = position;
            Direction = chord / std::sqrt( chordlength2 );
        }
    }
    return result;
}

// where the feet of the rails of the path are at the distance along it: the middle between them, the unit vector across to the
// rail on the right, tilted with the cant, and the unit vector along the path, with its grade
void rail_frame( sampled_path const &Path, double const Distance, glm::dvec3 &Middle, glm::dvec3 &Across, glm::dvec3 &Along ) {

    glm::vec3 angles;
    Path.segment->RaPositionGet( Distance, Middle, angles );
    glm::dvec3 behind, ahead;
    glm::vec3 unused;
    Path.segment->RaPositionGet( std::max( 0.0, Distance - 0.1 ), behind, unused );
    Path.segment->RaPositionGet( std::min( Path.length, Distance + 0.1 ), ahead, unused );
    Along = ahead - behind;
    if( glm::length( Along ) < 1e-6 ) {
        Along = { 0.0, 0.0, 1.0 };
    }
    Along = glm::normalize( Along );
    auto flat { glm::dvec3 { -Along.z, 0.0, Along.x } };
    flat = ( glm::length( flat ) > 1e-6 ? glm::normalize( flat ) : glm::dvec3 { 1.0, 0.0, 0.0 } );
    auto const roll { static_cast<double>( angles.x ) };
    Across = flat * std::cos( roll ) - glm::dvec3 { 0.0, 1.0, 0.0 } * std::sin( roll );
    Middle.y -= Path.track->rail_height();
}

// how far the feet of the rails of the path at the point stand off the plane of the feet of the rails of the other path where
// it's nearest the point: a timber under both lies flat under all of them only if they're in one plane
double off_plane( sampled_path const &Path, glm::dvec3 const &Point, sampled_path const &Other ) {

    if( Path.level && Other.level ) {
        return std::abs( Path.feet - Other.feet );
    }
    glm::dvec2 position, direction;
    auto const at { nearest_on( Path, plan( Point ), position, direction ) };
    auto const otherat { nearest_on( Other, plan( Point ), position, direction ) };
    glm::dvec3 middle, across, along, othermiddle, otheracross, otheralong;
    rail_frame( Path, at, middle, across, along );
    rail_frame( Other, otherat, othermiddle, otheracross, otheralong );
    auto const normal { glm::normalize( glm::cross( otheracross, otheralong ) ) };
    auto result { 0.0 };
    for( auto const side : { -0.5, 0.5 } ) {
        result = std::max( result, std::abs( glm::dot( middle + across * ( side * Path.gauge ) - othermiddle, normal ) ) );
    }
    return result;
}

// how far apart the middles of two paths running at the angle are where their plain sleepers would just touch: the one square to
// the other reaches out by half its length, the other by its half length foreshortened and half its width, with the gap
// between their ends. closer, the paths share the timbers
double touch_spacing( sampled_path const &Path, sampled_path const &Other, double const Angle ) {

    return Other.half + Path.half * std::cos( Angle ) + half_width * std::sin( Angle ) + end_gap;
}

// how close two paths run where their sleepers can't be laid apart, so they share the timbers whether they're in one plane or not
double shared_spacing( sampled_path const &Path, sampled_path const &Other ) {

    return Path.rails + Other.rails + 2.0 * overhang_min + end_gap;
}

// whether the path at the point, running the given way, is carried by the beams of a path before it, which it runs beside so
// close that their plain sleepers would touch. where the paths cross so wide that one timber can't carry both, the sleepers
// of the path are only left out where they'd meet the timbers of the other. the paths share timbers only while their rails lie
// in one plane, or run so close they can't be parted: where one rises, falls or cants apart, each lays its own
bool shares_with( sampled_path const &Path, glm::dvec3 const &Point, glm::dvec2 const &Direction, sampled_path const &Other ) {

    double gap;
    glm::dvec2 direction { Direction };
    if( false == beside( Other, Point, Path.half + Other.half + half_width + end_gap, gap, direction ) ) {
        return false;
    }
    auto const angle { angle_between( Direction, direction ) };
    if( angle >= grid_angle ) {
        return gap < Other.half + half_width + end_gap;
    }
    if( gap >= touch_spacing( Path, Other, angle ) ) {
        return false;
    }
    return gap < shared_spacing( Path, Other ) || off_plane( Path, Point, Other ) <= coplanar_tolerance;
}

// the paths of the track, and of the tracks with beams around it; or of all those with the ballast the simulator makes, whether
// they get beams or not
std::vector<sampled_path> paths_around( TTrack &Track, bool const Bedded ) {

    std::vector<sampled_path> result;
    for( int path = 0; path < path_count( Track ); ++path ) {
        result.emplace_back( sample( Track, path ) );
    }
    std::vector<TTrack *> others;
    tracks_near( Track, [&]( TTrack *Other ) {
        if( Other == &Track || Other->m_editorremoved ) {
            return;
        }
        if( std::find( others.begin(), others.end(), Other ) != others.end() || false == ( Bedded ? bedded( *Other ) : lays( *Other ) ) ) {
            return;
        }
        others.push_back( Other ); } );
    for( auto *other : others ) {
        for( int path = 0; path < path_count( *other ); ++path ) {
            result.emplace_back( sample( *other, path ) );
        }
    }
    result.erase(
        std::remove_if( result.begin(), result.end(), []( sampled_path const &Path ) { return Path.points.size() < 2; } ),
        result.end() );
    return result;
}


void tracks_near( TTrack &Track, std::function<void( TTrack * )> const &Visit ) {

    auto const sections { simulation::Region->sections( Track.location(), static_cast<float>( Track.radius() + 50.0 ) ) };
    for( auto *section : sections ) {
        for( auto const &cell : section->m_cells ) {
            if( glm::length( cell.area().center - Track.location() ) > cell.area().radius + Track.radius() + around_margin ) {
                continue;
            }
            for( auto *other : cell.m_directories.paths ) {
                if( other == nullptr || glm::length( other->location() - Track.location() ) > other->radius() + Track.radius() + around_margin ) {
                    continue;
                }
                Visit( other );
            }
        }
    }
}

std::vector<glm::dvec2> stretches_where( sampled_path const &Path, std::function<bool( glm::dvec3 const &, glm::dvec2 const & )> const &Test ) {

    std::vector<glm::dvec2> result;
    auto const holds = [&]( double const Distance ) {
        auto const distance { std::clamp( Distance, std::min( 0.01, 0.5 * Path.length ), std::max( Path.length - 0.01, 0.5 * Path.length ) ) };
        glm::dvec3 point;
        glm::vec3 angles;
        Path.segment->RaPositionGet( distance, point, angles );
        return Test( point, heading( Path, distance ) ); };
    auto const count { std::max( 1, static_cast<int>( std::ceil( Path.length / ( 0.5 * sample_step ) ) ) ) };
    auto was { holds( 0.0 ) };
    auto start { 0.0 };
    auto previous { 0.0 };
    for( int step = 1; step <= count; ++step ) {
        auto const distance { Path.length * step / count };
        auto const is { holds( distance ) };
        if( is != was ) {
            auto low { previous };
            auto high { distance };
            for( int narrow = 0; narrow < 8; ++narrow ) {
                auto const middle { 0.5 * ( low + high ) };
                ( holds( middle ) == was ? low : high ) = middle;
            }
            auto const boundary { 0.5 * ( low + high ) };
            if( was ) {
                result.emplace_back( start, boundary );
            }
            start = boundary;
            was = is;
        }
        previous = distance;
    }
    if( was ) {
        result.emplace_back( start, Path.length );
    }
    return result;
}

} // namespace track_sleepers
