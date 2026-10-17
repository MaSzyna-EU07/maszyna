/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"

#include "world/TrackSleepers.h"
#include "world/TrackFeatures.h"
#include "world/Track.h"
#include "model/Model3d.h"
#include "model/MdlMngr.h"
#include "rendering/renderer.h"
#include "simulation/simulation.h"
#include "scene/scene.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/utilities.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <chrono>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <map>
#include <memory>
#include <optional>
#include <set>

namespace track_sleepers {

namespace {

// how far apart the beams are along the track, and how far closer or further they may go to put one where a pin is
double constexpr pitch { 0.6 };
double constexpr pitch_min { 0.83 * pitch };
double constexpr pitch_max { 1.08 * pitch };
// how far out from the middle of a track its rail reaches past half the gauge: the foot of the rail outside the gauge line
double constexpr rail_reach { 0.1115 };
// how far a beam reaches out past the foot of the rail, 2.6 m sleepers under the standard gauge, and how far at least
double constexpr overhang { 0.471 };
double constexpr overhang_min { 0.25 };
// how far the feet of the rails a timber carries may stand off one plane
double constexpr coplanar_tolerance { 0.01 };
// the gap left between the ends of two beams on one line
double constexpr end_gap { 0.05 };
// half the width of a beam, to keep the sleepers of a track crossing another at a wide angle clear of the timbers of the other
double constexpr half_width { 0.13 };
// a timber longer than this is a fault of the layout
double constexpr length_cap { 8.5 }; // the longest prestressed concrete bearer of EN 13230-4, under both tracks of a scissors crossing about 5 m apart
// the rails of a path crossing the line of a beam at a lower angle than this aren't laid on it
double constexpr crossing_sine_min { 0.2588 }; // 15 degrees
// two paths crossing at this angle or wider keep the beams square to each, laid as a grid
double constexpr grid_angle { 1.2915 }; // 74 degrees
// the timbers turn toward the bisector of a frog over as many beams before it
double constexpr frog_ramp { 6.0 * pitch };
// past where the tracks go apart the sleepers lie square to them at once, unless the last timber turns so far from them that
// the ends of the next sleepers would meet it: then they turn back by at most so much a beam
double constexpr turn_max { 0.2415 }; // 13.8 degrees: a sleeper 1.3 m out from the track keeps clear of one turned so far a pitch away
// how far apart the samples of a path are, to tell how far the paths around run from it
double constexpr sample_step { 0.5 };
// two paths over each other, on a bridge, aren't merged
double constexpr level_tolerance { 1.0 };
// how far around a track the tracks are taken in to lay its beams: those whose beams it carries, or which carry its own, and
// those which set where the beams of these go
double constexpr around_margin { 20.0 };
// how many beams make a run drawn or left out together
std::size_t constexpr chunk_beams { 16 };

// the models of the beams found so far, by their names and skins and seats
std::map<std::string, beam_model> &known_models() {
    static std::map<std::string, beam_model> models;
    return models;
}

// the vertices of the submodel in the space of the model, through its own transform
template <typename Visit>
void vertices_of( TSubModel *Submodel, Visit Visitor ) {

    glm::mat4 transform { 1.f };
    if( ( Submodel->iFlags & 0xC000 ) && ( Submodel->GetMatrix() != nullptr ) ) {
        transform = glm::make_mat4( Submodel->GetMatrix()->readArray() );
    }
    auto const &vertices { Submodel->m_geometry.handle != null_handle ? GfxRenderer->Vertices( Submodel->m_geometry.handle ) : Submodel->Vertices };
    for( auto const &vertex : vertices ) {
        Visitor( glm::vec3 { transform * glm::vec4 { vertex.position, 1.f } } );
    }
}

// the model of the beams of the name and skin, loaded and measured on the first use; null when it can't be loaded. the feet of
// the rails go to the given seat in it, or to its top where the seat is below zero
beam_model const *find_model( std::string const &Name, std::string const &Skinname, float const Seat ) {

    if( Name.empty() ) {
        return nullptr;
    }
    auto const Skin { Skinname == "none" ? std::string {} : Skinname };
    auto const key { Name + '|' + Skin + '|' + std::to_string( Seat ) };
    auto &models { known_models() };
    auto found { models.find( key ) };
    if( found == models.end() ) {
        auto &model { models[ key ] };
        model.name = Name;
        model.skinname = Skin;
        model.model = TModelsManager::GetModel( Name, false );
        if( model.model == nullptr ) {
            ErrorLog( "Bad sleepers3d: model \"" + Name + "\" failed to load" );
            return nullptr;
        }
        if( false == Skin.empty() ) {
            model.skin = GfxRenderer->Fetch_Material( Skin );
        }
        // a timber put together from its parts, with a part between the ends stretched and repeated to the length
        std::array<char const *, part_count> const names { "begin", "main", "end" };
        model.stretchable = true;
        for( int part = 0; part < part_count; ++part ) {
            model.parts[ part ] = model.model->GetFromName( names[ part ] );
            model.stretchable = model.stretchable && model.parts[ part ] != nullptr;
        }
        auto top { std::numeric_limits<float>::lowest() };
        model.span = { std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest() };
        if( model.stretchable ) {
            for( int part = 0; part < part_count; ++part ) {
                glm::vec2 span { std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest() };
                vertices_of( model.parts[ part ], [&]( glm::vec3 const &Position ) {
                    span = { std::min( span.x, Position.x ), std::max( span.y, Position.x ) };
                    top = std::max( top, Position.y ); } );
                model.spans[ part ] = span;
                model.span = { std::min( model.span.x, span.x ), std::max( model.span.y, span.y ) };
            }
            model.stretchable = model.spans[ part_main ].y > model.spans[ part_main ].x;
        }
        else {
            // a whole model, measured on its most detailed submodels, the others being the same beam seen from further
            model.parts = {};
            for( auto *submodel { model.model->Root }; submodel != nullptr; submodel = submodel->Next ) {
                if( submodel->fSquareMinDist > 0.f ) {
                    continue;
                }
                vertices_of( submodel, [&]( glm::vec3 const &Position ) {
                    model.span = { std::min( model.span.x, Position.x ), std::max( model.span.y, Position.x ) };
                    top = std::max( top, Position.y ); } );
            }
        }
        // the levels of detail, by the distances they're seen from
        for( auto *submodel { model.model->Root }; submodel != nullptr; submodel = submodel->Next ) {
            if( std::find( model.bands.begin(), model.bands.end(), submodel->fSquareMinDist ) == model.bands.end() ) {
                model.bands.push_back( submodel->fSquareMinDist );
            }
        }
        std::sort( model.bands.begin(), model.bands.end() );
        // the rails sit on the seat of the sleeper, below the fastenings sticking up over it
        model.seat = ( Seat >= 0.f ? Seat : top );
        if( model.length() < 0.5f ) {
            ErrorLog( "Bad sleepers3d: model \"" + Name + "\" has no beam along x" );
            model.model = nullptr;
            return nullptr;
        }
        WriteLog(
            "sleepers3d: model \"" + Name + "\"" + ( model.stretchable ? std::string { " of parts" } : std::string { " whole" } )
            + ", " + to_string( model.span.x, 3 ) + ".." + to_string( model.span.y, 3 )
            + ( model.stretchable ?
                ", main " + to_string( model.spans[ part_main ].x, 3 ) + ".." + to_string( model.spans[ part_main ].y, 3 ) :
                std::string {} )
            + ", seat " + to_string( model.seat, 3 ) );
        return &model;
    }
    return ( found->second.model != nullptr ? &found->second : nullptr );
}

int path_count( TTrack const &Track ) {
    return ( Track.eType == tt_Switch ? 2 : 1 );
}

TSegment const *path_segment( TTrack const &Track, int const Path ) {
    return ( Track.eType == tt_Switch ? Track.SwitchExtension->Segments[ Path ].get() : Track.Segment.get() );
}

glm::dvec2 plan( glm::dvec3 const &Vector ) {
    return { Vector.x, Vector.z };
}

double cross( glm::dvec2 const &Left, glm::dvec2 const &Right ) {
    return Left.x * Right.y - Left.y * Right.x;
}

// a path of a track by samples taken along it
struct sampled_path {
    TTrack *track { nullptr };
    int path { 0 };
    TSegment const *segment { nullptr };
    double length { 0.0 };
    double gauge { 1.435 };
    double rails { 0.829 }; // how far out from the middle of the path its rails reach
    double reach { 1.3 }; // half the length of the sleepers of the path alone
    double half { 1.3 }; // half the length of the plain sleepers of the track, of the model it lays them of
    double line { 0.0 }; // the length of the line of plain tracks joined end to end the path is part of
    bool level { true }; // the path runs level and without cant, its rails all at the one height
    double feet { 0.0 }; // the height of the feet of its rails where it's level
    std::vector<glm::dvec3> points;
    std::vector<double> distances;
    glm::dvec2 low { 0.0 }; // the bounds of the samples in plan
    glm::dvec2 high { 0.0 };
};

// how long the line of tracks joined end to end the plain track is part of, so a line keeps laying the timbers where it runs
// past the tracks of another, from one track of it to the next. the line goes on through the main path of a switch, and ends
// where it comes to its diverging path
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

// the angle between two directions in plan, either way along them
double angle_between( glm::dvec2 const &Left, glm::dvec2 const &Right ) {

    return std::acos( std::min( 1.0, std::abs( glm::dot( Left, Right ) ) ) );
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

// what was wrong with the beams laid since the counts were taken last
struct fault_counts {
    std::size_t offplane { 0 }; // timbers shared by rails out of one plane, where the tracks run too close to part them
    std::size_t clipped { 0 }; // beam ends cut shorter than the least overhang, to keep clear of the beams of other tracks
    std::size_t dropped { 0 }; // beams left out where they'd lie over one laid before
    double slowest { 0.0 }; // the longest a cluster took to lay, in ms
    std::size_t biggest { 0 }; // the most tracks in a cluster
};

fault_counts &faults() {
    static fault_counts counts;
    return counts;
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
std::vector<sampled_path> paths_around( TTrack &Track, bool const Bedded = false ) {

    std::vector<sampled_path> result;
    for( int path = 0; path < path_count( Track ); ++path ) {
        result.emplace_back( sample( Track, path ) );
    }
    std::vector<TTrack *> others;
    auto const sections { simulation::Region->sections( Track.location(), static_cast<float>( Track.radius() + 50.0 ) ) };
    for( auto *section : sections ) {
        for( auto const &cell : section->m_cells ) {
            if( glm::length( cell.area().center - Track.location() ) > cell.area().radius + Track.radius() + around_margin ) {
                continue;
            }
            for( auto *other : cell.m_directories.paths ) {
                if( other == nullptr || other == &Track || other->m_editorremoved ) {
                    continue;
                }
                if( glm::length( other->location() - Track.location() ) > other->radius() + Track.radius() + around_margin ) {
                    continue;
                }
                if( std::find( others.begin(), others.end(), other ) != others.end() || false == ( Bedded ? bedded( *other ) : lays( *other ) ) ) {
                    continue;
                }
                others.push_back( other );
            }
        }
    }
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

// where two paths meet at an angle, the timbers which carry both lie square to the bisector of the angle: past the frog of
// a switch, where the paths go apart, and about the point where two paths cross. the timbers turn to the bisector before
// and back square to the tracks after, by the distance along the bisector from the point
struct feature {
    glm::dvec2 point; // the frog, or where the paths cross, in plan
    glm::dvec2 bisector; // unit
    double sine { 0.0 }; // of half the angle between the paths
    double spacing { 0.0 }; // how far apart the paths run at the point
    double from { 0.0 }; // along the bisector, where the timbers start turning to it
    double full { 0.0 }; // where they're square to it
    double until { 0.0 }; // till where they stay so
    double back { 0.0 }; // where they're square to the tracks again
    std::array<sampled_path const *, 2> paths {};
    std::array<double, 2> at {}; // the distance of the point along each path
    bool frog { false }; // the paths go apart from the point, rather than cross there
    bool grid { false }; // the paths cross so wide the timbers keep square to each
    bool crossover { false }; // the frog of a switch whose diverging track leads to another switch of a track alongside: the timbers keep square to the tracks
};

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

// a place along a path the beams are spread from: a pin with a beam at it, a free end of the track with the first beam half
// a step from it, so the beams keep the pitch over the joints, or a timber of another path which carries the rails of this
// one past it, with the first beam of this one a step from it
struct anchor {
    double at { 0.0 };
    double offset { 0.5 }; // how many steps the first beam is from the anchor: 0 pin, 0.5 free end, 1 timber of another path
};

// the beams spread evenly between two anchors, the pin at the first one included and the one at the last left to the next stretch
void spread( anchor const &From, anchor const &To, std::vector<double> &Stations ) {

    auto const span { To.at - From.at };
    if( span <= 0.0 ) {
        return;
    }
    auto count { static_cast<int>( std::round( span / pitch + 1.0 - From.offset - To.offset ) ) };
    if( From.offset == 0.5 && To.offset == 0.5 ) {
        // a track between two others gets a beam however short it is
        count = std::max( 1, count );
    }
    // the rails are carried at most so far apart, the beams pulled closer where it takes one more
    while( span / ( count - 1 + From.offset + To.offset ) > pitch_max ) {
        ++count;
    }
    auto const intervals { count - 1 + From.offset + To.offset };
    if( count < 1 || intervals <= 0.0 ) {
        if( From.offset == 0.0 ) {
            Stations.push_back( From.at );
        }
        return;
    }
    auto const step { span / intervals };
    auto const last { To.offset == 0.0 ? count - 1 : count };
    for( int index = 0; index < last; ++index ) {
        Stations.push_back( From.at + ( From.offset + index ) * step );
    }
}

// where another path crosses the line of a beam
struct crossing {
    sampled_path const *path { nullptr };
    glm::dvec3 point;
    glm::dvec2 direction; // of the path, in plan
    double at { 0.0 }; // the distance of the point along the path
    double across { 0.0 }; // along the beam, from the path laying it
    double sine { 1.0 }; // of the angle between the path and the beam
    bool carried { false };
};

void find_crossings( sampled_path const &Path, glm::dvec3 const &Point, glm::dvec2 const &Along, glm::dvec2 const &Across, double const Reach, std::vector<sampled_path> const &Around, std::vector<crossing> &Crossings ) {

    for( auto const &other : Around ) {
        if( &other == &Path || false == near_bounds( other, Point, Reach ) ) {
            continue;
        }
        for( std::size_t index = 0; index + 1 < other.points.size(); ++index ) {
            auto const &from { other.points[ index ] };
            auto const &to { other.points[ index + 1 ] };
            auto const fromside { glm::dot( plan( from - Point ), Along ) };
            auto const toside { glm::dot( plan( to - Point ), Along ) };
            if( ( fromside < 0.0 ) == ( toside < 0.0 ) ) {
                continue;
            }
            auto const share { fromside / ( fromside - toside ) };
            auto const point { from + ( to - from ) * share };
            auto const across { glm::dot( plan( point - Point ), Across ) };
            if( std::abs( across ) > Reach || std::abs( point.y - Point.y ) > level_tolerance ) {
                continue;
            }
            auto const direction { plan( to - from ) };
            if( glm::dot( direction, direction ) < 1e-12 ) {
                continue;
            }
            auto const unit { glm::normalize( direction ) };
            auto const at { glm::mix( other.distances[ index ], other.distances[ index + 1 ], share ) };
            Crossings.push_back( { &other, point, unit, at, across, std::abs( glm::dot( unit, Along ) ), false } );
        }
    }
}

// the paths whose rails the line of the beam through the point of the path takes, marked among the paths of its cluster crossing
// the line: those whose plain sleepers would meet the sleepers of the path or of any the line takes already, at a narrow enough
// angle to the line, and in one plane with the path unless they run too close to be parted
// the timber takes the rails of a path whose plain sleepers would meet those of a path it takes already; where Tight, only of one so
// close that their sleepers couldn't be laid apart even cut short
void carry( sampled_path const &Path, glm::dvec2 const &Forward, glm::dvec2 const &Along, std::vector<crossing> &Crossings, bool const Tight, bool const Count = true ) {

    struct carried_path {
        double across;
        double half;
        double rails;
        double sine;
    };
    std::vector<carried_path> carried { { 0.0, Path.half, Path.rails, std::max( crossing_sine_min, std::abs( glm::dot( Forward, Along ) ) ) } };
    for( auto added { true }; added; ) {
        added = false;
        for( auto &crossing : Crossings ) {
            if( crossing.carried || crossing.sine < crossing_sine_min || angle_between( crossing.direction, Forward ) >= grid_angle ) {
                continue;
            }
            auto const close { std::any_of( carried.begin(), carried.end(), [&]( carried_path const &Carried ) {
                auto const reach { Tight ? crossing.path->rails + Carried.rails + 2.0 * overhang_min + end_gap : crossing.path->half + Carried.half + end_gap };
                return std::abs( crossing.across - Carried.across ) * std::min( crossing.sine, Carried.sine ) < reach; } ) };
            if( false == close ) {
                continue;
            }
            if( off_plane( *crossing.path, crossing.point, Path ) > coplanar_tolerance ) {
                if( std::abs( crossing.across ) * crossing.sine >= shared_spacing( *crossing.path, Path ) ) {
                    continue;
                }
                // too close to the path to be parted, out of the plane of its rails
                if( Count ) {
                    ++faults().offplane;
                }
            }
            crossing.carried = true;
            added = true;
            carried.push_back( { crossing.across, crossing.path->half, crossing.path->rails, crossing.sine } );
        }
    }
    // the line doesn't carry the rails of a track past those of one it doesn't, that one lays its own beams there
    auto const blocks = [&]( crossing const &Other ) {
        return false == Other.carried && Other.sine >= crossing_sine_min && angle_between( Other.direction, Forward ) < grid_angle; };
    for( auto demoted { true }; demoted; ) {
        demoted = false;
        for( auto &crossing : Crossings ) {
            if( false == crossing.carried ) {
                continue;
            }
            if( std::any_of( Crossings.begin(), Crossings.end(), [&]( struct crossing const &Other ) {
                    return blocks( Other ) && Other.across * crossing.across > 0.0 && std::abs( Other.across ) < std::abs( crossing.across ); } ) ) {
                crossing.carried = false;
                demoted = true;
            }
        }
    }
}

// the stretch of a beam line the rails of a path take, and how far the beam may reach past them
struct band {
    double from { 0.0 };
    double to { 0.0 };
    double overhang { 0.0 };
    double overhangmin { 0.0 };
    bool carried { true }; // the rails are laid on the line, else the beam keeps clear of them and of the sleepers under them
    sampled_path const *path { nullptr };
    double centre { 0.0 }; // where the middle of the path crosses the line
    glm::dvec2 direction { 0.0 }; // of the path there, in plan
    double sine { 1.0 }; // of the angle between the path and the line
};

band band_of( sampled_path const &Path, double const Across, double const Sine, glm::dvec2 const &Direction, bool const Carried = true ) {

    return { Across - Path.rails / Sine, Across + Path.rails / Sine, overhang / Sine, overhang_min / Sine, Carried, &Path, Across, Direction, Sine };
}

// a beam on a line, by where its ends are along the line
struct piece {
    double from { 0.0 };
    double to { 0.0 };
    bool clipped { false }; // an end cut short to keep clear of the rails of another track
    double corefrom { 0.0 }; // the stretch the feet of the rails it carries take
    double coreto { 0.0 };
    band const *single { nullptr }; // the rails of the only track it carries, null where it carries more
};

// the beams on the line through the point: one timber under all the rails it carries, the ends keeping clear of the rails of the
// tracks the line doesn't carry and of the sleepers under them. the timber is parted between the rails of two tracks where each
// keeps at least its least overhang past its rails, unless it's whole: about a crossing of the tracks, as in a scissors crossing
void emit_line( std::vector<band> &Bands, std::vector<piece> &Pieces, bool const Whole ) {

    std::sort( Bands.begin(), Bands.end(), []( band const &Left, band const &Right ) { return Left.from < Right.from; } );
    auto open { false };
    piece current;
    auto end { 0.0 }, endoverhang { 0.0 }, endoverhangmin { 0.0 };
    // the far side of the last rails not carried, and of the sleepers under them
    auto fenced { false };
    auto fencerails { 0.0 }, fencereach { 0.0 };
    for( auto const &next : Bands ) {
        if( false == next.carried ) {
            if( open ) {
                auto const kept { std::max( 0.0, std::clamp( next.from - next.overhang - end - end_gap, std::min( endoverhangmin, next.from - end - end_gap ), endoverhang ) ) };
                current.clipped = current.clipped || kept < endoverhang;
                if( kept < endoverhangmin ) {
                    ++faults().clipped;
                }
                current.to = end + kept;
                current.coreto = end;
                Pieces.push_back( current );
                open = false;
            }
            if( false == fenced || next.to > fencerails ) {
                fencerails = next.to;
                fencereach = next.to + next.overhang;
            }
            fenced = true;
            continue;
        }
        if( false == open ) {
            current = piece();
            auto startoverhang { next.overhang };
            if( fenced ) {
                startoverhang = std::max( 0.0, std::clamp( next.from - fencereach - end_gap, std::min( next.overhangmin, next.from - fencerails - end_gap ), next.overhang ) );
                current.clipped = startoverhang < next.overhang;
                if( startoverhang < next.overhangmin ) {
                    ++faults().clipped;
                }
            }
            current.from = next.from - startoverhang;
            current.corefrom = next.from;
            current.single = &next;
            end = next.to;
            endoverhang = next.overhang;
            endoverhangmin = next.overhangmin;
            open = true;
            continue;
        }
        if( false == Whole && next.from - end >= endoverhangmin + next.overhangmin + end_gap ) {
            // the room between them shared by their overhangs
            auto const room { next.from - end - end_gap };
            auto const ours { std::min( endoverhang, room * endoverhang / ( endoverhang + next.overhang ) ) };
            auto const theirs { std::min( next.overhang, room - ours ) };
            current.to = end + ours;
            current.coreto = end;
            current.clipped = current.clipped || ours < endoverhang;
            Pieces.push_back( current );
            current = piece();
            current.from = next.from - theirs;
            current.corefrom = next.from;
            current.clipped = theirs < next.overhang;
            current.single = &next;
            end = next.to;
            endoverhang = next.overhang;
            endoverhangmin = next.overhangmin;
            continue;
        }
        // one timber under the rails of all the tracks it carries
        current.single = nullptr;
        if( next.to > end ) {
            end = next.to;
            endoverhang = next.overhang;
            endoverhangmin = next.overhangmin;
        }
    }
    if( open ) {
        current.to = end + endoverhang;
        current.coreto = end;
        Pieces.push_back( current );
    }
}

// a timber longer than the cap cut in two where the rails of the tracks it carries are furthest apart, the ends sharing the
// room between them, till every piece is short enough or there's nowhere to cut
void cut_long( std::vector<band> const &Bands, std::vector<piece> &Pieces ) {

    for( std::size_t index = 0; index < Pieces.size(); ) {
        auto &whole { Pieces[ index ] };
        if( whole.to - whole.from <= length_cap ) {
            ++index;
            continue;
        }
        // the widest gap between the rails under the timber
        band const *left { nullptr };
        band const *right { nullptr };
        band const *last { nullptr };
        auto widest { 0.0 };
        auto end { std::numeric_limits<double>::lowest() };
        for( auto const &band : Bands ) {
            if( false == band.carried || band.from < whole.from || band.to > whole.to ) {
                continue;
            }
            if( last != nullptr && band.from - end > widest ) {
                widest = band.from - end;
                left = last;
                right = &band;
            }
            if( band.to > end ) {
                end = band.to;
                last = &band;
            }
        }
        if( left == nullptr ) {
            ++index;
            continue;
        }
        auto const room { std::max( 0.0, widest - end_gap ) };
        auto const ours { std::min( left->overhang, room * left->overhang / ( left->overhang + right->overhang ) ) };
        auto const theirs { std::min( right->overhang, room - ours ) };
        if( ours < left->overhangmin || theirs < right->overhangmin ) {
            ++faults().clipped;
        }
        piece second { right->from - theirs, whole.to, true, right->from, whole.coreto, nullptr };
        whole.to = left->to + ours;
        whole.coreto = left->to;
        whole.clipped = true;
        Pieces.insert( Pieces.begin() + index + 1, second );
    }
}

// how far the paths of the switch stand off one plane where they share the timbers of its panel, and where along the diverging
// path; the switch itself is out of shape for its timbers where it's more than the tolerance
std::pair<double, double> panel_off_plane( TTrack &Track ) {

    std::pair<double, double> result { 0.0, 0.0 };
    if( Track.eType != tt_Switch ) {
        return result;
    }
    auto const main { sample( Track, 0 ) };
    auto const diverging { sample( Track, 1 ) };
    if( main.points.size() < 2 || diverging.points.size() < 2 ) {
        return result;
    }
    auto alongside { false };
    for( std::size_t index = 0; index < diverging.points.size(); ++index ) {
        double gap;
        glm::dvec2 direction;
        if( false == beside( main, diverging.points[ index ], shared_spacing( diverging, main ), gap, direction ) ) {
            if( alongside ) {
                // apart far enough to lay their own
                break;
            }
            // at the start, which both paths share
            continue;
        }
        alongside = true;
        auto const off { off_plane( diverging, diverging.points[ index ], main ) };
        if( off > result.first ) {
            result = { off, diverging.distances[ index ] };
        }
    }
    return result;
}

} // namespace

beam_model const *timber_model() {

    if( false == track_features::sleepers ) {
        return nullptr;
    }
    return find_model( Global.Sleepers3D, std::string {}, -1.f );
}

beam_model const *plain_model() {

    return find_model( Global.Sleepers3DPlain, Global.Sleepers3DPlainSkin, Global.Sleepers3DPlainSeat );
}

beam_model const *timber_model( TTrack const &Track ) {

    if( false == Track.m_timber_model_name.empty() ) {
        if( auto const *model { find_model( Track.m_timber_model_name, Track.m_timber_skin_name, -1.f ) }; model != nullptr ) {
            return model;
        }
    }
    return timber_model();
}

beam_model const *plain_model( TTrack const &Track ) {

    if( Track.m_sleeper_enabled && false == Track.m_sleeper_model_name.empty() ) {
        // the sleepers of the track drawn as they are in the model, their tops at the feet of the rails
        if( auto const *model { find_model( Track.m_sleeper_model_name, Track.m_sleeper_skin_name, -1.f ) }; model != nullptr ) {
            return model;
        }
    }
    return plain_model();
}

double plain_length( TTrack const &Track ) {

    if( auto const *model { plain_model( Track ) }; model != nullptr ) {
        return model->length();
    }
    return 2.0 * ( 0.5 * std::abs( Track.fTrackWidth ) + rail_reach + overhang );
}

bool bedded( TTrack const &Track ) {

    if( ( Track.iCategoryFlag & 15 ) != 1 || Track.m_editorremoved ) {
        return false;
    }
    switch( Track.eType ) {
        case tt_Normal: {
            if( Track.m_material2 == null_handle || Track.Segment == nullptr ) {
                return false;
            }
            break;
        }
        case tt_Switch: {
            // the switch trackbed takes the ballast of a plain track joined to it
            auto const ballast = [&]( TTrack const *Other ) {
                return Other != nullptr && Other->eType == tt_Normal && Other->m_material2 != null_handle; };
            if( false == Global.CreateSwitchTrackbeds || Track.SwitchExtension == nullptr
             || ( Track.SwitchExtension->m_material3 == null_handle && false == ballast( Track.trPrev ) && false == ballast( Track.trNext ) ) ) {
                return false;
            }
            break;
        }
        default: {
            return false;
        }
    }
    return true;
}

bool lays( TTrack const &Track ) {

    return timber_model() != nullptr && bedded( Track );
}

std::vector<std::pair<double, double>> shared_zones( TTrack const &Track, int const Path ) {

    std::vector<std::pair<double, double>> result;
    if( false == bedded( Track ) || Path < 0 || Path >= path_count( Track ) ) {
        return result;
    }
    auto const around { paths_around( const_cast<TTrack &>( Track ), true ) };
    auto const found { std::find_if( around.begin(), around.end(), [&]( sampled_path const &Sampled ) { return Sampled.track == &Track && Sampled.path == Path; } ) };
    if( found == around.end() ) {
        return result;
    }
    auto const &path { *found };
    // whether another path runs so near at the distance that the timbers carry both
    auto const shared = [&]( double const Distance ) {
        auto const distance { std::clamp( Distance, std::min( 0.01, 0.5 * path.length ), std::max( path.length - 0.01, 0.5 * path.length ) ) };
        glm::dvec3 point;
        glm::vec3 angles;
        path.segment->RaPositionGet( distance, point, angles );
        auto const direction { heading( path, distance ) };
        return std::any_of( around.begin(), around.end(), [&]( sampled_path const &Other ) {
            return &Other != &path && shares_with( path, point, direction, Other ); } ); };
    auto const count { std::max( 1, static_cast<int>( std::ceil( path.length / ( 0.5 * sample_step ) ) ) ) };
    auto was { shared( 0.0 ) };
    auto start { 0.0 };
    auto previous { 0.0 };
    for( int step = 1; step <= count; ++step ) {
        auto const distance { path.length * step / count };
        auto const is { shared( distance ) };
        if( is != was ) {
            auto low { previous };
            auto high { distance };
            for( int narrow = 0; narrow < 8; ++narrow ) {
                auto const middle { 0.5 * ( low + high ) };
                ( shared( middle ) == was ? low : high ) = middle;
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
        result.emplace_back( start, path.length );
    }
    return result;
}

std::vector<pack_model> const &pack_models() {

    static std::vector<pack_model> models;
    static auto scanned { false };
    if( scanned ) {
        return models;
    }
    scanned = true;
    namespace fs = std::filesystem;
    std::error_code error;
    auto const root { fs::path { paths::models } };
    for( fs::recursive_directory_iterator entry { root, fs::directory_options::follow_directory_symlink | fs::directory_options::skip_permission_denied, error }, end;
         entry != end; entry.increment( error ) ) {
        if( error ) {
            break;
        }
        if( false == entry->is_regular_file( error ) ) {
            continue;
        }
        auto const extension { ToLower( entry->path().extension().string() ) };
        if( extension != ".t3d" && extension != ".e3d" ) {
            continue;
        }
        auto name { ToLower( fs::relative( entry->path(), root, error ).replace_extension().generic_string() ) };
        if( error || name.empty() ) {
            continue;
        }
        if( name.find( "podklad" ) == std::string::npos && name.find( "podrozjazd" ) == std::string::npos
         && name.find( "sleeper" ) == std::string::npos && name.find( "timber" ) == std::string::npos ) {
            continue;
        }
        // a timber model has the parts it's put together from, told by their names in the file
        std::ifstream file { entry->path(), std::ios::binary };
        std::string const content { std::istreambuf_iterator<char>( file ), std::istreambuf_iterator<char>() };
        auto const has = [&]( std::string const &Part ) {
            return (
                extension == ".t3d" ?
                    content.find( "Name: " + Part + "\n" ) != std::string::npos || content.find( "Name: " + Part + " " ) != std::string::npos
                 || content.find( "Name: " + Part + "\r" ) != std::string::npos :
                    content.find( std::string { '\0' } + Part + '\0' ) != std::string::npos ); };
        auto const timber { has( "begin" ) && has( "main" ) && has( "end" ) };
        if( std::none_of( models.begin(), models.end(), [&]( pack_model const &Model ) { return Model.name == name; } ) ) {
            models.push_back( { name, timber } );
        }
    }
    std::sort( models.begin(), models.end(), []( pack_model const &Left, pack_model const &Right ) {
        return ( Left.timber != Right.timber ? Left.timber : Left.name < Right.name ); } );
    return models;
}

std::pair<double, double> shared_zone( TTrack const &Track, int const Path ) {

    auto const zones { shared_zones( Track, Path ) };
    if( zones.empty() ) {
        return { 0.0, 0.0 };
    }
    return { zones.front().first, zones.back().second };
}

namespace {

// the version of the layout of the beams, moved on whenever a track changes, so the beams worked out for a cluster hold till then
std::atomic<std::uint64_t> &layout_epoch() {
    static std::atomic<std::uint64_t> epoch { 1 };
    return epoch;
}

// the beams of a cluster of tracks laid together, by the track each is laid with
struct cluster_beams {
    std::uint64_t epoch { 0 };
    std::map<TTrack const *, std::vector<beam>> beams;
};

std::map<TTrack const *, std::shared_ptr<cluster_beams>> &known_clusters() {
    static std::map<TTrack const *, std::shared_ptr<cluster_beams>> clusters;
    return clusters;
}

std::mutex &clusters_mutex() {
    static std::mutex mutex;
    return mutex;
}

// how many tracks a cluster takes at most
std::size_t constexpr cluster_cap { 256 };
// how far the lines of the timbers turn from one to the next at most, so the far ends of the long ones keep their spacing, and how
// far the plain sleepers turn back square to the track from one to the next
double constexpr timber_turn { 0.0175 }; // a degree
double constexpr sleeper_turn { 0.06 }; // three and a half degrees

// whether the plain sleepers of the two paths would meet anywhere, so they lie on timbers together there
bool share_anywhere( sampled_path const &Left, sampled_path const &Right ) {

    auto const margin { Left.half + Right.half + half_width + end_gap };
    if( Left.low.x > Right.high.x + margin || Right.low.x > Left.high.x + margin || Left.low.y > Right.high.y + margin || Right.low.y > Left.high.y + margin ) {
        return false;
    }
    for( auto const &pair : { std::make_pair( &Left, &Right ), std::make_pair( &Right, &Left ) } ) {
        auto const &path { *pair.first };
        for( std::size_t index = 0; index < path.points.size(); ++index ) {
            if( shares_with( path, path.points[ index ], heading( path, path.distances[ index ] ), *pair.second ) ) {
                return true;
            }
        }
    }
    return false;
}

// the tracks whose timbers are laid together with those of the track: all those reached from it through paths whose plain
// sleepers would meet, in the order of their names
std::vector<TTrack *> cluster_of( TTrack &Track ) {

    std::vector<TTrack *> result { &Track };
    for( std::size_t index = 0; index < result.size() && result.size() < cluster_cap; ++index ) {
        auto const around { paths_around( *result[ index ] ) };
        for( auto const &path : around ) {
            if( path.track != result[ index ] ) {
                continue;
            }
            for( auto const &other : around ) {
                if( other.track == result[ index ] || std::find( result.begin(), result.end(), other.track ) != result.end() ) {
                    continue;
                }
                if( share_anywhere( path, other ) ) {
                    result.push_back( other.track );
                }
            }
        }
    }
    std::sort( result.begin(), result.end(), []( TTrack const *Left, TTrack const *Right ) {
        return ( Left->name() != Right->name() ? Left->name() < Right->name() : Left < Right ); } );
    return result;
}

// the stretches of the path where another path of the cluster runs so near that their plain sleepers would meet
std::vector<glm::dvec2> zones_of( sampled_path const &Path, std::vector<sampled_path> const &Paths, std::vector<feature> const &Crossings ) {

    std::vector<glm::dvec2> result;
    auto const shared = [&]( double const Distance ) {
        auto const distance { std::clamp( Distance, std::min( 0.01, 0.5 * Path.length ), std::max( Path.length - 0.01, 0.5 * Path.length ) ) };
        glm::dvec3 point;
        glm::vec3 angles;
        Path.segment->RaPositionGet( distance, point, angles );
        auto const direction { heading( Path, distance ) };
        return std::any_of( Paths.begin(), Paths.end(), [&]( sampled_path const &Other ) {
            if( &Other == &Path ) {
                return false;
            }
            // tracks crossing share the timbers where their sleepers would meet, others only where they're too close to part them
            auto const crossed { std::any_of( Crossings.begin(), Crossings.end(), [&]( feature const &Feature ) {
                return ( Feature.paths[ 0 ] == &Path && Feature.paths[ 1 ] == &Other ) || ( Feature.paths[ 0 ] == &Other && Feature.paths[ 1 ] == &Path ); } ) };
            if( crossed ) {
                return shares_with( Path, point, direction, Other );
            }
            double gap;
            glm::dvec2 otherdirection { direction };
            return beside( Other, point, shared_spacing( Path, Other ), gap, otherdirection ) && gap < shared_spacing( Path, Other ); } ); };
    auto const count { std::max( 1, static_cast<int>( std::ceil( Path.length / ( 0.5 * sample_step ) ) ) ) };
    auto was { shared( 0.0 ) };
    auto start { 0.0 };
    auto previous { 0.0 };
    for( int step = 1; step <= count; ++step ) {
        auto const distance { Path.length * step / count };
        auto const is { shared( distance ) };
        if( is != was ) {
            auto low { previous };
            auto high { distance };
            for( int narrow = 0; narrow < 8; ++narrow ) {
                auto const middle { 0.5 * ( low + high ) };
                ( shared( middle ) == was ? low : high ) = middle;
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

// the stretches left of the first ones once the second ones are taken out, those shorter than the least length left out too
std::vector<glm::dvec2> without( std::vector<glm::dvec2> const &Stretches, std::vector<glm::dvec2> const &Taken, double const Least ) {

    std::vector<glm::dvec2> result { Stretches };
    for( auto const &taken : Taken ) {
        std::vector<glm::dvec2> left;
        for( auto const &stretch : result ) {
            if( stretch.y <= taken.x || stretch.x >= taken.y ) {
                left.push_back( stretch );
                continue;
            }
            if( stretch.x < taken.x ) {
                left.emplace_back( stretch.x, taken.x );
            }
            if( stretch.y > taken.y ) {
                left.emplace_back( taken.y, stretch.y );
            }
        }
        result = std::move( left );
    }
    result.erase( std::remove_if( result.begin(), result.end(), [&]( glm::dvec2 const &Stretch ) { return Stretch.y - Stretch.x < Least; } ), result.end() );
    return result;
}

// the beams of a cluster of tracks, laid together: lines of timbers led along the paths near other paths one after another, each
// line across all the rails of the cluster it reaches, and the plain sleepers of every path where no line carries it, in step
// with the last timber carrying it
class cluster_layout {

public:
    cluster_layout( std::vector<TTrack *> const &Tracks ) :
        tracks( Tracks ) {}

    std::shared_ptr<cluster_beams> lay() {

        auto result { std::make_shared<cluster_beams>() };
        result->epoch = layout_epoch();
        for( auto *track : tracks ) {
            result->beams[ track ];
            for( int path = 0; path < path_count( *track ); ++path ) {
                paths.emplace_back( sample( *track, path ) );
            }
        }
        paths.erase( std::remove_if( paths.begin(), paths.end(), []( sampled_path const &Path ) { return Path.points.size() < 2; } ), paths.end() );
        for( auto *track : tracks ) {
            for( auto &other : paths_around( *track ) ) {
                if( std::find( tracks.begin(), tracks.end(), other.track ) != tracks.end()
                 || std::any_of( context.begin(), context.end(), [&]( sampled_path const &Known ) { return Known.track == other.track && Known.path == other.path; } ) ) {
                    continue;
                }
                context.emplace_back( std::move( other ) );
            }
        }
        features = find_features( paths );
        for( auto const &feature : features ) {
            if( false == feature.frog && false == feature.grid ) {
                crossings.push_back( feature );
            }
        }
        zones.resize( paths.size() );
        records.resize( paths.size() );
        done.resize( paths.size() );
        for( std::size_t index = 0; index < paths.size(); ++index ) {
            zones[ index ] = ( tracks.size() > 1 || paths.size() > 1 ? zones_of( paths[ index ], paths, crossings ) : std::vector<glm::dvec2> {} );
        }
        // the lines of timbers, led along the path first by priority with a stretch near another path no line carries yet
        for( std::size_t round = 0; round < 1000; ++round ) {
            auto best { paths.size() };
            glm::dvec2 stretch { 0.0 };
            for( std::size_t index = 0; index < paths.size(); ++index ) {
                auto const left { without( zones[ index ], covered( index ), 0.3 ) };
                if( left.empty() ) {
                    continue;
                }
                if( best == paths.size() || before( paths[ index ], paths[ best ] ) ) {
                    best = index;
                    stretch = *std::max_element( left.begin(), left.end(), []( glm::dvec2 const &Left, glm::dvec2 const &Right ) { return Left.y - Left.x < Right.y - Right.x; } );
                }
            }
            if( best == paths.size() ) {
                break;
            }
            lead( best, stretch );
            done[ best ].push_back( stretch );
        }
        // the plain sleepers of each path, between the timbers carrying it and its ends
        for( std::size_t index = 0; index < paths.size(); ++index ) {
            lay_plain( index );
        }
        keep_apart();
        fill_gaps();
        keep_apart();
        for( auto const &laid : beams ) {
            result->beams[ laid.owner ].push_back( laid.value );
        }
        return result;
    }

private:
    struct owned {
        TTrack const *owner;
        beam value;
    };

    std::vector<TTrack *> const &tracks;
    std::vector<sampled_path> paths; // of the tracks of the cluster
    std::vector<sampled_path> context; // of the tracks about them, whose rails and sleepers the beams keep clear of
    std::vector<feature> features;
    std::vector<feature> crossings; // the features where tracks cross, without the frogs
    std::vector<std::vector<glm::dvec2>> zones; // of each path, where its plain sleepers would meet those of another one
    // a line of timbers carrying a path: where along the path, and which way the line runs in plan
    struct record {
        double at;
        glm::dvec2 along;
    };
    std::vector<std::vector<record>> records; // the lines of timbers carrying each path
    std::vector<std::vector<glm::dvec2>> done; // the stretches of each path lines were led along
    std::vector<owned> beams;

    // the stretches of the path the lines laid so far carry it on, and those lines were led along
    std::vector<glm::dvec2> covered( std::size_t const Index ) const {

        auto result { done[ Index ] };
        auto const at { distances( Index ) };
        if( at.empty() ) {
            return result;
        }
        auto start { at.front() };
        auto last { at.front() };
        for( std::size_t index = 1; index <= at.size(); ++index ) {
            if( index < at.size() && at[ index ] - last <= 1.6 * pitch ) {
                last = at[ index ];
                continue;
            }
            result.emplace_back( start - 0.5 * pitch, last + 0.5 * pitch );
            if( index < at.size() ) {
                start = last = at[ index ];
            }
        }
        return result;
    }

    // the pins of the lines led along the path: the rails of a path crossing this one so wide it's laid on a grid of beams; elsewhere
    // the lines are spread evenly
    std::vector<double> pins( sampled_path const &Path ) const {

        std::vector<double> result;
        for( auto const &feature : features ) {
            if( false == feature.grid ) {
                continue;
            }
            for( int side = 0; side < 2; ++side ) {
                if( feature.paths[ side ] != &Path ) {
                    continue;
                }
                // under the rails of the other path, which runs across the beams
                auto const *other { feature.paths[ 1 - side ] };
                auto const sine { std::max( 0.5, std::sin( 2.0 * std::asin( feature.sine ) ) ) };
                result.push_back( feature.at[ side ] - 0.5 * other->gauge / sine );
                result.push_back( feature.at[ side ] + 0.5 * other->gauge / sine );
            }
        }
        std::sort( result.begin(), result.end() );
        return result;
    }

    // the line of the timbers carrying the path nearest to the distance on the side, between the distance and the reach from it
    std::optional<record> record_near( std::size_t const Index, double const Distance, double const Side, double const Reach ) const {

        std::optional<record> result;
        for( auto const &line : records[ Index ] ) {
            auto const along { ( line.at - Distance ) * Side };
            if( along < -0.25 * pitch || along > Reach ) {
                continue;
            }
            if( false == result.has_value() || std::abs( line.at - Distance ) < std::abs( result->at - Distance ) ) {
                result = line;
            }
        }
        return result;
    }

    // where along the path the lines of timbers carry it, in order
    std::vector<double> distances( std::size_t const Index ) const {

        std::vector<double> result;
        for( auto const &line : records[ Index ] ) {
            result.push_back( line.at );
        }
        std::sort( result.begin(), result.end() );
        result.erase( std::unique( result.begin(), result.end(), []( double const Left, double const Right ) { return Right - Left < 0.05; } ), result.end() );
        return result;
    }

    // where the path is at the distance and which way it runs in plan; false where it can't tell
    bool frame_at( sampled_path const &Path, double const Distance, glm::dvec3 &Point, glm::vec3 &Angles, glm::dvec2 &Forward ) const {

        Path.segment->RaPositionGet( Distance, Point, Angles );
        glm::dvec3 behind, ahead;
        glm::vec3 unused;
        Path.segment->RaPositionGet( std::max( 0.0, Distance - 0.1 ), behind, unused );
        Path.segment->RaPositionGet( std::min( Path.length, Distance + 0.1 ), ahead, unused );
        Forward = plan( ahead - behind );
        if( glm::length( Forward ) < 1e-6 ) {
            return false;
        }
        Forward = glm::normalize( Forward );
        return true;
    }

    // the lines at the distances along the path, turned as the frogs and crossings about them ask, but by no more than the rate
    // from one line to the next, nor from the lines carrying the path at either end, so the ends of long timbers keep their spacing
    void lay_lines( std::size_t const Index, std::vector<double> const &Stations, std::optional<record> const &First, std::optional<record> const &Last, double const Rate, bool const Led ) {

        auto const &path { paths[ Index ] };
        std::vector<double> distances;
        std::vector<glm::dvec2> forwards;
        std::vector<double> turns; // of the lines off square to the path, in plan
        auto const turn = []( glm::dvec2 const &Forward, glm::dvec2 Along ) {
            if( glm::dot( Forward, Along ) < 0.0 ) {
                Along = -Along;
            }
            return std::atan2( cross( Forward, Along ), glm::dot( Forward, Along ) ); };
        for( auto const station : Stations ) {
            glm::dvec3 point;
            glm::vec3 angles;
            glm::dvec2 forward;
            if( false == frame_at( path, station, point, angles, forward ) ) {
                continue;
            }
            distances.push_back( station );
            forwards.push_back( forward );
            // the plain sleepers lie square to their track, turned only to meet the timbers next to them
            auto along { Led ? beam_direction( point, forward, features ) : forward };
            if( Led ) {
                // the timbers about a crossing lie square to the mean way of the tracks they carry, the frogs about it don't turn them:
                // square to the main tracks of a scissors crossing, to the bisector of a diamond
                if( auto const mean { whole_at( path, point, forward, along ) }; mean.has_value() ) {
                    along = *mean;
                }
            }
            turns.push_back( turn( forward, along ) );
        }
        if( distances.empty() ) {
            return;
        }
        auto const limit = [&]( std::size_t const From, std::size_t const To, std::optional<record> const &Seed ) {
            auto const step { To > From ? 1 : -1 };
            std::optional<double> previous;
            auto rate { Rate };
            if( Seed.has_value() ) {
                previous = turn( forwards[ From ], Seed->along );
                if( false == Led ) {
                    // square after three sleepers
                    rate = std::max( 0.004, std::abs( *previous ) / 3.0 );
                }
            }
            for( auto index { From }; ; index += step ) {
                if( previous.has_value() ) {
                    turns[ index ] = std::clamp( turns[ index ], *previous - rate, *previous + rate );
                }
                previous = turns[ index ];
                if( index == To ) {
                    break;
                }
            } };
        limit( 0, distances.size() - 1, First );
        limit( distances.size() - 1, 0, Last );
        for( std::size_t index = 0; index < distances.size(); ++index ) {
            auto const &forward { forwards[ index ] };
            auto const angle { turns[ index ] };
            lay_line( Index, distances[ index ], Led, glm::dvec2 { forward.x * std::cos( angle ) - forward.y * std::sin( angle ), forward.x * std::sin( angle ) + forward.y * std::cos( angle ) } );
        }
    }

    // the lines of timbers led along the stretch of the path, spread between the lines carrying it at either end, or the ends of the
    // stretch, and the pins on it
    void lead( std::size_t const Index, glm::dvec2 const &Stretch ) {

        auto const &path { paths[ Index ] };
        auto const back { record_near( Index, Stretch.x, -1.0, 2.0 * pitch ) };
        auto const fore { record_near( Index, Stretch.y, 1.0, 2.0 * pitch ) };
        anchor const start { back.has_value() ? anchor { back->at, 1.0 } : anchor { Stretch.x <= 0.01 ? 0.0 : Stretch.x, 0.5 } };
        anchor const end { fore.has_value() ? anchor { fore->at, 1.0 } : anchor { Stretch.y >= path.length - 0.01 ? path.length : Stretch.y, 0.5 } };
        if( end.at - start.at <= 0.0 ) {
            return;
        }
        std::vector<anchor> anchors { start };
        for( auto const pin : pins( path ) ) {
            if( pin > start.at + 0.5 * pitch_min && pin < end.at - 0.5 * pitch_min && pin - anchors.back().at >= pitch_min ) {
                anchors.push_back( { pin, 0.0 } );
            }
        }
        anchors.push_back( end );
        std::vector<double> stations;
        for( std::size_t index = 1; index < anchors.size(); ++index ) {
            spread( anchors[ index - 1 ], anchors[ index ], stations );
        }
        lay_lines( Index, stations, back, fore, timber_turn, true );
    }

    // where the timber led along the path through the point, lying the given way, carries both tracks of a crossing: the way it lies
    // then, square to the middle between the ways of the tracks it carries turned furthest either way
    std::optional<glm::dvec2> whole_at( sampled_path const &Path, glm::dvec3 const &Point, glm::dvec2 const &Forward, glm::dvec2 const &Along ) const {

        if( crossings.empty() ) {
            return std::nullopt;
        }
        std::vector<crossing> members;
        find_crossings( Path, Point, Along, glm::dvec2 { -Along.y, Along.x }, 0.5 * length_cap + Path.reach, paths, members );
        carry( Path, Forward, Along, members, false, false );
        if( false == whole_with( Path, members ) ) {
            return std::nullopt;
        }
        // the outermost tracks it carries, either way along it: the main tracks of a scissors crossing, the crossing ones of a diamond
        std::pair<double, glm::dvec2> low { 0.0, Forward };
        std::pair<double, glm::dvec2> high { 0.0, Forward };
        for( auto const &crossing : members ) {
            if( false == crossing.carried ) {
                continue;
            }
            if( crossing.across < low.first ) {
                low = { crossing.across, crossing.direction };
            }
            if( crossing.across > high.first ) {
                high = { crossing.across, crossing.direction };
            }
        }
        auto const angle_of = [&]( glm::dvec2 Direction ) {
            if( glm::dot( Direction, Forward ) < 0.0 ) {
                Direction = -Direction;
            }
            return std::atan2( cross( Forward, Direction ), glm::dot( Forward, Direction ) ); };
        auto const angle { 0.5 * ( angle_of( low.second ) + angle_of( high.second ) ) };
        return glm::dvec2 { Forward.x * std::cos( angle ) - Forward.y * std::sin( angle ), Forward.x * std::sin( angle ) + Forward.y * std::cos( angle ) };
    }

    bool whole_with( sampled_path const &Path, std::vector<crossing> const &Members ) const {

        auto const carries = [&]( sampled_path const *Carried ) {
            return Carried == &Path || std::any_of( Members.begin(), Members.end(), [&]( crossing const &Crossing ) { return Crossing.carried && Crossing.path == Carried; } ); };
        return std::any_of( crossings.begin(), crossings.end(), [&]( feature const &Feature ) {
            return carries( Feature.paths[ 0 ] ) && carries( Feature.paths[ 1 ] ); } );
    }

    // the plain sleepers of the path on the stretches no line of timbers carries it, in step with the timbers at their ends
    void lay_plain( std::size_t const Index ) {

        auto const &path { paths[ Index ] };
        auto const at { distances( Index ) };
        std::vector<std::pair<anchor, anchor>> stretches;
        if( at.empty() ) {
            stretches.push_back( { { 0.0, 0.5 }, { path.length, 0.5 } } );
        }
        else {
            stretches.push_back( { { 0.0, 0.5 }, { at.front(), 1.0 } } );
            for( std::size_t index = 1; index < at.size(); ++index ) {
                if( at[ index ] - at[ index - 1 ] > 1.6 * pitch ) {
                    stretches.push_back( { { at[ index - 1 ], 1.0 }, { at[ index ], 1.0 } } );
                }
            }
            stretches.push_back( { { at.back(), 1.0 }, { path.length, 0.5 } } );
        }
        for( auto const &stretch : stretches ) {
            auto const span { stretch.second.at - stretch.first.at };
            if( span <= 0.0 || span < ( stretch.first.offset + stretch.second.offset - 1.0 ) * pitch + 0.5 * pitch_min ) {
                continue;
            }
            std::vector<double> stations;
            spread( stretch.first, stretch.second, stations );
            // turned back square to the path from the timbers at the ends of the stretch
            auto const first { stretch.first.offset == 1.0 ? record_near( Index, stretch.first.at, 1.0, 0.05 ) : std::optional<record> {} };
            auto const last { stretch.second.offset == 1.0 ? record_near( Index, stretch.second.at, -1.0, 0.05 ) : std::optional<record> {} };
            lay_lines( Index, stations, first, last, sleeper_turn, false );
        }
    }

    // the beams on the line through the point of the path at the distance: the timber of the line led along it, taking the rails of
    // the paths of the cluster it reaches, or the plain sleeper of the path alone
    void lay_line( std::size_t const Index, double const Distance, bool const Led, glm::dvec2 const &Along ) {

        auto const &path { paths[ Index ] };
        glm::dvec3 point;
        glm::vec3 angles;
        glm::dvec2 forward;
        if( false == frame_at( path, Distance, point, angles, forward ) ) {
            return;
        }
        auto const along { Along };
        glm::dvec2 const across { -along.y, along.x };
        auto const reach { 0.5 * length_cap + path.reach };
        std::vector<crossing> members, others;
        find_crossings( path, point, along, across, reach, paths, members );
        find_crossings( path, point, along, across, reach, context, others );
        if( Led ) {
            // about a crossing the timbers take all the tracks whose sleepers would meet, elsewhere only those too close to part
            carry( path, forward, along, members, false );
            if( false == whole_with( path, members ) ) {
                for( auto &crossing : members ) {
                    crossing.carried = false;
                }
                carry( path, forward, along, members, true );
            }
        }
        std::vector<band> bands { band_of( path, 0.0, std::max( crossing_sine_min, std::abs( glm::dot( forward, along ) ) ), forward ) };
        for( auto const *crossings : { &members, &others } ) {
            for( auto const &crossing : *crossings ) {
                if( crossing.carried ) {
                    bands.push_back( band_of( *crossing.path, crossing.across, crossing.sine, crossing.direction ) );
                }
                else if( crossing.sine >= crossing_sine_min && angle_between( crossing.direction, forward ) < grid_angle ) {
                    bands.push_back( band_of( *crossing.path, crossing.across, crossing.sine, crossing.direction, false ) );
                }
            }
        }
        // a timber carrying both tracks of a crossing lies whole under all the tracks it carries
        auto const whole { Led && whole_with( path, members ) };
        std::vector<piece> pieces;
        emit_line( bands, pieces, whole );
        cut_long( bands, pieces );
        // the parts of a timber parted between tracks going apart turn square to their own track, the more the further apart the
        // tracks run, till square where their plain sleepers wouldn't touch any more
        std::vector<glm::dvec2> turned( pieces.size(), along );
        for( std::size_t index = 0; index < pieces.size() && pieces.size() > 1; ++index ) {
            auto const *own { pieces[ index ].single };
            if( own == nullptr ) {
                continue;
            }
            band const *nearest { nullptr };
            for( auto const &other : bands ) {
                if( &other != own && other.carried && ( nearest == nullptr || std::abs( other.centre - own->centre ) < std::abs( nearest->centre - own->centre ) ) ) {
                    nearest = &other;
                }
            }
            if( nearest == nullptr ) {
                continue;
            }
            auto const spacing { std::abs( nearest->centre - own->centre ) * std::min( own->sine, nearest->sine ) };
            auto const parted { shared_spacing( *own->path, *nearest->path ) };
            auto const apart { own->path->half + nearest->path->half + end_gap };
            auto const share { apart > parted ? std::clamp( ( spacing - parted ) / ( apart - parted ), 0.0, 1.0 ) : 1.0 };
            auto target { own->direction };
            if( glm::dot( target, along ) < 0.0 ) {
                target = -target;
            }
            auto const angle { share * std::atan2( cross( along, target ), glm::dot( along, target ) ) };
            turned[ index ] = { along.x * std::cos( angle ) - along.y * std::sin( angle ), along.x * std::sin( angle ) + along.y * std::cos( angle ) };
        }
        auto const turned_for = [&]( sampled_path const *Path ) {
            for( std::size_t index = 0; index < pieces.size(); ++index ) {
                if( pieces[ index ].single != nullptr && pieces[ index ].single->path == Path ) {
                    return turned[ index ];
                }
            }
            return along; };
        if( Led ) {
            records[ Index ].push_back( { Distance, turned_for( &path ) } );
            for( auto const &crossing : members ) {
                if( crossing.carried ) {
                    records[ crossing.path - paths.data() ].push_back( { crossing.at, turned_for( crossing.path ) } );
                }
            }
        }
        // the beam lies tilted with the roll of the path, as the ballast is
        glm::dvec3 const flatacross { across.x, 0.0, across.y };
        glm::dvec3 const up { 0.0, 1.0, 0.0 };
        auto const roll { static_cast<double>( angles.x ) };
        auto const rolledacross { flatacross * std::cos( roll ) - up * std::sin( roll ) };
        auto const rolledup { flatacross * std::sin( roll ) + up * std::cos( roll ) };
        auto const top { point - rolledup * static_cast<double>( path.track->rail_height() ) };
        // a sleeper of a plain track running alone and square to it, the rest are the timbers of the switches and crossings
        auto const carried { std::count_if( bands.begin(), bands.end(), []( band const &Band ) { return Band.carried; } ) };
        auto const plain { path.track->eType == tt_Normal && carried == 1 && pieces.size() == 1 && false == pieces.front().clipped && std::abs( glm::dot( forward, along ) ) > 0.9998 };
        for( std::size_t index = 0; index < pieces.size(); ++index ) {
            auto const &piece { pieces[ index ] };
            auto const &way { turned[ index ] };
            if( way == along ) {
                beams.push_back( { path.track, { top + rolledacross * piece.from, top + rolledacross * piece.to, glm::dvec3 { along.x, 0.0, along.y }, plain, piece.corefrom - piece.from, piece.coreto - piece.from } } );
                continue;
            }
            // turned about where the middle of its track crosses the line
            auto const centre { top + rolledacross * piece.single->centre };
            glm::dvec3 const turnedacross { glm::dvec3 { -way.y, 0.0, way.x } * std::cos( roll ) - up * std::sin( roll ) };
            beams.push_back( { path.track, { centre + turnedacross * ( piece.from - piece.single->centre ), centre + turnedacross * ( piece.to - piece.single->centre ), glm::dvec3 { way.x, 0.0, way.y }, false, piece.corefrom - piece.from, piece.coreto - piece.from } } );
        }
    }

    // a sleeper of the path put in where the rails of it go further without a beam under them than the pitch lets them, turned
    // as the beams about it are: where beams were left out or lines of timbers led along different paths meet
    void fill_gaps() {

        auto const count { beams.size() };
        // the beams by the cells of a grid they reach into
        double constexpr cellsize { 2.0 };
        auto const key = []( glm::dvec2 const &Point ) {
            return std::make_pair( static_cast<std::int64_t>( std::floor( Point.x / cellsize ) ), static_cast<std::int64_t>( std::floor( Point.y / cellsize ) ) ); };
        std::map<std::pair<std::int64_t, std::int64_t>, std::vector<std::size_t>> grid;
        for( std::size_t laid = 0; laid < count; ++laid ) {
            auto const &beam { beams[ laid ].value };
            auto const centre { plan( 0.5 * ( beam.from + beam.to ) ) };
            auto const reach { 0.5 * glm::length( plan( beam.to - beam.from ) ) + half_width };
            auto const low { key( centre - glm::dvec2 { reach } ) };
            auto const high { key( centre + glm::dvec2 { reach } ) };
            for( auto x { low.first }; x <= high.first; ++x ) {
                for( auto z { low.second }; z <= high.second; ++z ) {
                    grid[ { x, z } ].push_back( laid );
                }
            }
        }
        for( std::size_t index = 0; index < paths.size(); ++index ) {
            auto const &path { paths[ index ] };
            // where along the path beams lie under each of its rails, and which way they run
            auto const steps { std::max( 2, static_cast<int>( std::ceil( path.length / 0.05 ) ) ) };
            std::vector<double> gaps;
            std::vector<glm::dvec2> turns;
            for( auto const side : { -1.0, 1.0 } ) {
                std::optional<std::pair<double, glm::dvec2>> last;
                for( int step = 0; step <= steps; ++step ) {
                    auto const distance { path.length * step / steps };
                    glm::dvec3 point, across;
                    path.segment->plan_frame( distance, point, across );
                    auto const rail { plan( point + across * ( side * 0.5 * path.gauge ) ) };
                    std::optional<glm::dvec2> found;
                    auto const cell { grid.find( key( rail ) ) };
                    if( cell == grid.end() ) {
                        continue;
                    }
                    for( auto const laid : cell->second ) {
                        if( found.has_value() ) {
                            break;
                        }
                        auto const &beam { beams[ laid ].value };
                        auto const span { plan( beam.to - beam.from ) };
                        auto const length { glm::length( span ) };
                        if( length < 0.1 ) {
                            continue;
                        }
                        auto const axis { span / length };
                        auto const offset { rail - plan( 0.5 * ( beam.from + beam.to ) ) };
                        if( std::abs( glm::dot( offset, axis ) ) <= 0.5 * length && std::abs( cross( axis, offset ) ) <= half_width ) {
                            found = glm::dvec2 { -axis.y, axis.x };
                        }
                    }
                    if( false == found.has_value() ) {
                        continue;
                    }
                    if( last.has_value() && distance - last->first > 0.06 && distance - last->first + 2.0 * half_width > 1.25 * pitch_max ) {
                        // the middle of the stretch without one, and the way the beams at its ends run
                        gaps.push_back( 0.5 * ( distance + last->first ) );
                        auto along { last->second + ( glm::dot( last->second, *found ) < 0.0 ? -*found : *found ) };
                        turns.push_back( glm::length( along ) > 1e-9 ? glm::normalize( along ) : heading( path, gaps.back() ) );
                    }
                    last = std::make_pair( distance, *found );
                }
            }
            // a sleeper in each, those of both rails about one place put in once
            for( std::size_t gap = 0; gap < gaps.size(); ++gap ) {
                if( std::any_of( gaps.begin(), gaps.begin() + gap, [&]( double const Other ) { return std::abs( Other - gaps[ gap ] ) < 0.3; } ) ) {
                    continue;
                }
                lay_line( index, gaps[ gap ], false, turns[ gap ] );
            }
        }
    }

    // a beam over one laid before it is left out: where tracks cross too wide for one timber to take both, or the lines of two
    // stretches meet at an angle
    void keep_apart() {

        struct box {
            glm::dvec2 centre;
            glm::dvec2 axis;
            double half;
        };
        auto const box_of = []( beam const &Beam ) {
            auto const span { plan( Beam.to - Beam.from ) };
            auto const length { glm::length( span ) };
            return box { plan( 0.5 * ( Beam.from + Beam.to ) ), length > 1e-9 ? span / length : glm::dvec2 { 1.0, 0.0 }, 0.5 * length }; };
        auto const apart = []( box const &Left, box const &Right ) {
            auto const offset { Right.centre - Left.centre };
            for( auto const &axis : { Left.axis, glm::dvec2 { -Left.axis.y, Left.axis.x }, Right.axis, glm::dvec2 { -Right.axis.y, Right.axis.x } } ) {
                auto const reach = [&]( box const &Box ) {
                    return Box.half * std::abs( glm::dot( Box.axis, axis ) ) + ( half_width - 0.01 ) * std::abs( cross( Box.axis, axis ) ); };
                if( std::abs( glm::dot( offset, axis ) ) > reach( Left ) + reach( Right ) ) {
                    return true;
                }
            }
            return false; };
        double constexpr cellsize { 3.0 };
        auto const key = []( glm::dvec2 const &Point ) {
            return std::make_pair( static_cast<std::int64_t>( std::floor( Point.x / cellsize ) ), static_cast<std::int64_t>( std::floor( Point.y / cellsize ) ) ); };
        std::map<std::pair<std::int64_t, std::int64_t>, std::vector<box>> grid;
        std::vector<owned> kept;
        auto const clear_of_laid = [&]( struct box const &Box ) {
            auto const low { key( Box.centre - glm::dvec2 { Box.half + half_width } ) };
            auto const high { key( Box.centre + glm::dvec2 { Box.half + half_width } ) };
            for( auto x { low.first }; x <= high.first; ++x ) {
                for( auto z { low.second }; z <= high.second; ++z ) {
                    auto const found { grid.find( { x, z } ) };
                    if( found != grid.end()
                     && false == std::all_of( found->second.begin(), found->second.end(), [&]( struct box const &Other ) { return apart( Box, Other ); } ) ) {
                        return false;
                    }
                }
            }
            return true; };
        for( auto laid : beams ) {
            auto const length { glm::length( laid.value.to - laid.value.from ) };
            if( length < 0.1 ) {
                continue;
            }
            if( false == clear_of_laid( box_of( laid.value ) ) ) {
                // shortened from either end or both, down to the least overhang past the rails it carries, till it's clear
                auto const direction { ( laid.value.to - laid.value.from ) / length };
                auto const most { std::max( 0.0, laid.value.corefrom - overhang_min ) }; // off the from end
                auto const mostto { std::max( 0.0, length - laid.value.coreto - overhang_min ) };
                auto found { false };
                for( int step = 1; step <= 20 && false == found; ++step ) {
                    auto const share { step / 20.0 };
                    for( auto const ends : { glm::dvec2 { 0.0, mostto * share }, glm::dvec2 { most * share, 0.0 }, glm::dvec2 { most * share, mostto * share } } ) {
                        auto shortened { laid.value };
                        shortened.from = laid.value.from + direction * ends.x;
                        shortened.to = laid.value.to - direction * ends.y;
                        shortened.corefrom -= ends.x;
                        shortened.coreto -= ends.x;
                        if( clear_of_laid( box_of( shortened ) ) ) {
                            shortened.plain = false;
                            laid.value = shortened;
                            found = true;
                            break;
                        }
                    }
                }
                if( false == found ) {
                    ++faults().dropped;
                    continue;
                }
                ++faults().clipped;
            }
            auto const box { box_of( laid.value ) };
            auto const low { key( box.centre - glm::dvec2 { box.half + half_width } ) };
            auto const high { key( box.centre + glm::dvec2 { box.half + half_width } ) };
            for( auto x { low.first }; x <= high.first; ++x ) {
                for( auto z { low.second }; z <= high.second; ++z ) {
                    grid[ { x, z } ].push_back( box );
                }
            }
            kept.push_back( laid );
        }
        beams = std::move( kept );
    }
};

} // namespace

namespace {

// the beams of the cluster of the track worked out already for the current layout, null where they aren't
std::shared_ptr<cluster_beams> cached_cluster( TTrack const &Track ) {

    std::lock_guard<std::mutex> lock { clusters_mutex() };
    auto const found { known_clusters().find( &Track ) };
    return ( found != known_clusters().end() && found->second->epoch == layout_epoch() ? found->second : nullptr );
}

// the beams of the cluster of the track worked out, and kept for each track of it unless the tracks changed meanwhile
std::shared_ptr<cluster_beams> work_out( TTrack &Track ) {

    auto const start { std::chrono::steady_clock::now() };
    auto const tracks { cluster_of( Track ) };
    auto const laid { cluster_layout( tracks ).lay() };
    faults().slowest = std::max( faults().slowest, std::chrono::duration<double, std::milli>( std::chrono::steady_clock::now() - start ).count() );
    faults().biggest = std::max( faults().biggest, tracks.size() );
    std::lock_guard<std::mutex> lock { clusters_mutex() };
    if( laid->epoch == layout_epoch() ) {
        for( auto *track : tracks ) {
            known_clusters()[ track ] = laid;
        }
    }
    return laid;
}

std::vector<beam> beams_of( cluster_beams const &Cluster, TTrack const &Track ) {

    auto const beams { Cluster.beams.find( &Track ) };
    return ( beams != Cluster.beams.end() ? beams->second : std::vector<beam> {} );
}

// the clusters of the tracks coming near worked out in a thread of their own, one after another
class background_worker {

public:
    ~background_worker() {
        {
            std::lock_guard<std::mutex> lock { mutex };
            stop = true;
            queue.clear();
        }
        wake.notify_all();
        if( thread.joinable() ) {
            thread.join();
        }
    }

    bool enabled { true };

    void request( TTrack &Track ) {

        if( false == resolved ) {
            // the models the sleepers are made of loaded here, the thread can't load them
            resolved = true;
            for( auto *track : simulation::Paths.sequence() ) {
                if( track != nullptr && bedded( *track ) ) {
                    plain_model( *track );
                    timber_model( *track );
                }
            }
        }
        {
            std::lock_guard<std::mutex> lock { mutex };
            if( queued.count( &Track ) ) {
                return;
            }
            queued.insert( &Track );
            queue.push_back( &Track );
            if( false == thread.joinable() ) {
                thread = std::thread( [ this ]() { run(); } );
            }
        }
        wake.notify_one();
    }

    // nothing left to work out nor being worked out, the waiting ones dropped
    void settle() {

        std::unique_lock<std::mutex> lock { mutex };
        queue.clear();
        queued.clear();
        idle.wait( lock, [ this ]() { return false == busy; } );
    }

private:
    void run() {

        while( true ) {
            TTrack *track { nullptr };
            {
                std::unique_lock<std::mutex> lock { mutex };
                wake.wait( lock, [ this ]() { return stop || false == queue.empty(); } );
                if( stop ) {
                    return;
                }
                track = queue.front();
                queue.pop_front();
                busy = true;
            }
            if( cached_cluster( *track ) == nullptr ) {
                work_out( *track );
            }
            {
                std::lock_guard<std::mutex> lock { mutex };
                queued.erase( track );
                busy = false;
            }
            idle.notify_all();
        }
    }

    std::mutex mutex;
    std::condition_variable wake;
    std::condition_variable idle;
    std::deque<TTrack *> queue;
    std::set<TTrack const *> queued;
    std::thread thread;
    bool stop { false };
    bool busy { false };
    bool resolved { false };
};

background_worker &worker() {
    static background_worker worker;
    return worker;
}

} // namespace

std::vector<beam> place( TTrack &Track ) {

    auto cluster { cached_cluster( Track ) };
    if( cluster == nullptr ) {
        cluster = work_out( Track );
    }
    return beams_of( *cluster, Track );
}

void lay_in_background( bool const Background ) {

    if( false == Background ) {
        worker().settle();
    }
    worker().enabled = Background;
}

void prefetch( TTrack &Track ) {

    if( worker().enabled && cached_cluster( Track ) == nullptr ) {
        worker().request( Track );
    }
}

void lay( TTrack &Track ) {

    auto &laid { Track.m_sleepers3d };
    std::vector<beam> placed;
    if( laid.active && worker().enabled && timber_model( Track ) != nullptr ) {
        // worked out in the background; till then the track keeps what it has, and tries again
        auto const cluster { cached_cluster( Track ) };
        if( cluster == nullptr ) {
            worker().request( Track );
            return;
        }
        placed = beams_of( *cluster, Track );
    }
    laid.dirty = false;
    for( auto &instances : laid.instances ) {
        instances.clear();
    }
    laid.whole.clear();
    laid.plain.clear();
    laid.beams.clear();
    laid.chunks.clear();
    laid.timbers = timber_model( Track );
    laid.plains = plain_model( Track );
    if( false == laid.active || laid.timbers == nullptr ) {
        return;
    }
    auto const &parts { *laid.timbers };
    if( auto const panel { panel_off_plane( Track ) }; panel.first > coplanar_tolerance ) {
        WriteLog(
            "sleepers3d: switch \"" + Track.name() + "\" has its paths " + to_string( panel.first * 1000.0, 1 ) + " mm off one plane "
            + to_string( panel.second, 2 ) + " m along, where they share the timbers of its panel" );
    }
    laid.laying = true;
    laid.beams = ( worker().enabled ? std::move( placed ) : place( Track ) );
    laid.laying = false;
    auto const &beams { laid.beams };

    auto const partlength = [&]( int const Part ) {
        return static_cast<double>( parts.spans[ Part ].y - parts.spans[ Part ].x ); };
    auto const beginlength { partlength( part_begin ) };
    auto const mainlength { std::max( 0.01, partlength( part_main ) ) };
    auto const endlength { partlength( part_end ) };

    chunk run;
    std::vector<std::pair<glm::dvec3, double>> runbeams;
    auto const close_run = [&]() {
        if( runbeams.empty() ) {
            return;
        }
        glm::dvec3 centre { 0.0 };
        for( auto const &runbeam : runbeams ) {
            centre += runbeam.first;
        }
        centre /= static_cast<double>( runbeams.size() );
        auto radius { 0.0 };
        for( auto const &runbeam : runbeams ) {
            radius = std::max( radius, glm::length( runbeam.first - centre ) + runbeam.second );
        }
        run.centre = glm::vec3 { centre };
        run.radius = static_cast<float>( radius + 0.3 );
        for( int part = 0; part < part_count; ++part ) {
            run.last[ part ] = static_cast<std::uint32_t>( laid.instances[ part ].size() );
        }
        run.wholelast = static_cast<std::uint32_t>( laid.whole.size() );
        run.plainlast = static_cast<std::uint32_t>( laid.plain.size() );
        laid.chunks.push_back( run );
        runbeams.clear();
    };

    for( auto const &beam : beams ) {
        if( runbeams.empty() ) {
            for( int part = 0; part < part_count; ++part ) {
                run.first[ part ] = static_cast<std::uint32_t>( laid.instances[ part ].size() );
            }
            run.wholefirst = static_cast<std::uint32_t>( laid.whole.size() );
            run.plainfirst = static_cast<std::uint32_t>( laid.plain.size() );
        }
        auto const length { glm::length( beam.to - beam.from ) };
        if( length < 0.1 ) {
            continue;
        }
        // the frame of the beam: x along it, y up, z along the tracks
        glm::dvec3 const x { ( beam.to - beam.from ) / length };
        auto z { glm::normalize( beam.along - x * glm::dot( beam.along, x ) ) };
        auto y { glm::cross( z, x ) };
        if( y.y < 0.0 ) {
            z = -z;
            y = -y;
        }
        auto const middle { 0.5 * ( beam.from + beam.to ) - Track.m_origin };
        glm::mat4 frame { 1.f };
        frame[ 0 ] = glm::vec4 { glm::vec3 { x }, 0.f };
        frame[ 1 ] = glm::vec4 { glm::vec3 { y }, 0.f };
        frame[ 2 ] = glm::vec4 { glm::vec3 { z }, 0.f };
        frame[ 3 ] = glm::vec4 { glm::vec3 { middle }, 1.f };
        // a part stretched along x by the scale, and moved so its x lands at the given place along the beam
        auto const put = [&]( int const Part, double const Scale, double const From, double const Place ) {
            auto const shift { Place - Scale * From };
            laid.instances[ Part ].emplace_back(
                glm::scale(
                    glm::translate( frame, glm::vec3 { static_cast<float>( shift ), -parts.seat, 0.f } ),
                    glm::vec3 { static_cast<float>( Scale ), 1.f, 1.f } ) );
        };
        auto const half { 0.5 * length };
        auto const between { length - beginlength - endlength };
        if( beam.plain && laid.plains != nullptr ) {
            // the whole sleeper at its own length, its middle under the middle of the track
            auto const &plain { *laid.plains };
            laid.plain.emplace_back(
                glm::translate( frame, glm::vec3 { -0.5f * ( plain.span.x + plain.span.y ), -plain.seat, 0.f } ) );
        }
        else if( false == parts.stretchable ) {
            // a timber of a model which can't be put together, the whole of it stretched to the length
            auto const scale { length / static_cast<double>( parts.length() ) };
            laid.whole.emplace_back(
                glm::scale(
                    glm::translate( frame, glm::vec3 { static_cast<float>( -scale * 0.5 * ( parts.span.x + parts.span.y ) ), -parts.seat, 0.f } ),
                    glm::vec3 { static_cast<float>( scale ), 1.f, 1.f } ) );
        }
        else if( between > 0.01 ) {
            // the ends at their own size, and as many pieces between them as fit closest to their own size
            put( part_end, 1.0, parts.spans[ part_end ].x, -half );
            put( part_begin, 1.0, parts.spans[ part_begin ].y, half );
            auto const pieces { std::max( 1, static_cast<int>( std::round( between / mainlength ) ) ) };
            auto const piece { between / pieces };
            for( int index = 0; index < pieces; ++index ) {
                put( part_main, piece / mainlength, parts.spans[ part_main ].x, -half + endlength + index * piece );
            }
        }
        else {
            // a beam shorter than its ends: just the ends, squeezed
            auto const scale { length / std::max( 0.01, beginlength + endlength ) };
            put( part_end, scale, parts.spans[ part_end ].x, -half );
            put( part_begin, scale, parts.spans[ part_begin ].y, half );
        }
        runbeams.emplace_back( middle, half );
        if( runbeams.size() >= chunk_beams ) {
            close_run();
        }
    }
    close_run();
}

layout_check check() {

    layout_check result;
    worker().settle();
    faults() = {};
    // laid anew, for the counts of what's wrong with them
    ++layout_epoch();
    // the beams of all the tracks in plan, as boxes, with the grid of where they are
    struct box {
        glm::dvec2 centre;
        glm::dvec2 axis; // along the beam
        double half { 0.0 };
        TTrack const *track { nullptr };
    };
    std::vector<box> boxes;
    std::vector<TTrack *> tracks;
    for( auto *track : simulation::Paths.sequence() ) {
        if( track == nullptr || track->m_editorremoved || false == track->m_sleepers3d.active ) {
            continue;
        }
        tracks.push_back( track );
        if( auto const panel { panel_off_plane( *track ) }; panel.first > coplanar_tolerance ) {
            ++result.panels;
            if( result.panels <= 4 ) result.examples.push_back(
                "switch " + track->name() + " " + to_string( panel.first * 1000.0, 1 ) + " mm off one plane at " + to_string( panel.second, 2 ) + " m" );
        }
        for( auto const &beam : place( *track ) ) {
            auto const span { plan( beam.to - beam.from ) };
            auto const length { glm::length( span ) };
            if( length < 0.1 ) {
                continue;
            }
            boxes.push_back( { plan( 0.5 * ( beam.from + beam.to ) ), span / length, 0.5 * length, track } );
            result.longest = std::max( result.longest, length );
            if( length > length_cap ) {
                ++result.toolong;
            }
        }
    }
    result.beams = boxes.size();
    result.offplane = faults().offplane;
    result.clipped = faults().clipped;
    result.dropped = faults().dropped;
    result.slowest = faults().slowest;
    result.biggest = faults().biggest;
    double constexpr cellsize { 3.0 };
    auto const key = []( glm::dvec2 const &Point ) {
        return std::make_pair( static_cast<std::int64_t>( std::floor( Point.x / cellsize ) ), static_cast<std::int64_t>( std::floor( Point.y / cellsize ) ) ); };
    std::map<std::pair<std::int64_t, std::int64_t>, std::vector<std::size_t>> grid;
    for( std::size_t index = 0; index < boxes.size(); ++index ) {
        // each box in every cell its span touches
        auto const &box { boxes[ index ] };
        auto const low { key( box.centre - glm::dvec2 { box.half + half_width } ) };
        auto const high { key( box.centre + glm::dvec2 { box.half + half_width } ) };
        for( auto x { low.first }; x <= high.first; ++x ) {
            for( auto z { low.second }; z <= high.second; ++z ) {
                grid[ { x, z } ].push_back( index );
            }
        }
    }
    // two beams overlap when no axis of either keeps them apart, with a little room left between them
    auto const apart = []( box const &Left, box const &Right ) {
        auto const offset { Right.centre - Left.centre };
        for( auto const &axis : { Left.axis, glm::dvec2 { -Left.axis.y, Left.axis.x }, Right.axis, glm::dvec2 { -Right.axis.y, Right.axis.x } } ) {
            auto const reach = [&]( box const &Box ) {
                return Box.half * std::abs( glm::dot( Box.axis, axis ) ) + ( half_width - 0.01 ) * std::abs( cross( Box.axis, axis ) ); };
            if( std::abs( glm::dot( offset, axis ) ) > reach( Left ) + reach( Right ) ) {
                return true;
            }
        }
        return false; };
    std::set<std::pair<std::size_t, std::size_t>> overlaps;
    for( auto const &cell : grid ) {
        auto const &indices { cell.second };
        for( std::size_t left = 0; left < indices.size(); ++left ) {
            for( std::size_t right = left + 1; right < indices.size(); ++right ) {
                auto const pair { std::minmax( indices[ left ], indices[ right ] ) };
                if( overlaps.count( pair ) || apart( boxes[ pair.first ], boxes[ pair.second ] ) ) {
                    continue;
                }
                overlaps.insert( pair );
                if( overlaps.size() <= 16 ) {
                    auto const &box { boxes[ pair.first ] };
                    result.examples.push_back(
                        "overlap " + box.track->name() + "/" + boxes[ pair.second ].track->name() + " at " + to_string( box.centre.x, 2 ) + " " + to_string( box.centre.y, 2 ) );
                }
            }
        }
    }
    result.overlaps = overlaps.size();
    // the rails of the tracks, followed for how far apart the beams under them are
    for( auto *track : tracks ) {
        for( int path = 0; path < path_count( *track ); ++path ) {
            auto const *segment { path_segment( *track, path ) };
            if( segment == nullptr ) {
                continue;
            }
            auto const length { segment->GetLength() };
            auto const steps { static_cast<int>( std::ceil( length / 0.05 ) ) };
            for( auto const side : { -1.0, 1.0 } ) {
                // from the first beam under the rail to the last, how long it runs with none under it
                auto last { -1.0 };
                auto first { -1.0 };
                for( int step = 0; step <= steps; ++step ) {
                    auto const distance { length * step / std::max( 1, steps ) };
                    glm::dvec3 point, across;
                    segment->plan_frame( distance, point, across );
                    auto const rail { plan( point + across * ( side * 0.5 * std::abs( track->fTrackWidth ) ) ) };
                    auto const found { grid.find( key( rail ) ) };
                    auto supported { false };
                    if( found != grid.end() ) {
                        for( auto const index : found->second ) {
                            auto const &box { boxes[ index ] };
                            auto const offset { rail - box.centre };
                            if( std::abs( glm::dot( offset, box.axis ) ) <= box.half && std::abs( cross( box.axis, offset ) ) <= half_width ) {
                                supported = true;
                                break;
                            }
                        }
                    }
                    if( false == supported ) {
                        continue;
                    }
                    if( last >= 0.0 && distance - last > 0.06 ) {
                        // the spacing of the beams, middle to middle, by the stretch without one
                        auto const spacing { distance - last + 2.0 * half_width };
                        if( spacing > result.widest ) {
                            result.widest = spacing;
                            result.widestat = track->name() + " " + to_string( distance, 2 );
                        }
                        if( spacing > 1.25 * 1.08 * pitch ) {
                            ++result.gaps;
                            if( result.examples.size() < 24 ) {
                                result.examples.push_back( "gap " + to_string( spacing, 2 ) + " m under " + track->name() + " path " + std::to_string( path ) + " at " + to_string( distance, 2 ) );
                            }
                        }
                    }
                    if( first < 0.0 ) {
                        first = distance;
                    }
                    last = distance;
                }
                if( first < 0.0 ) {
                    ++result.bare;
                    if( result.examples.size() < 80 ) {
                        result.examples.push_back( "no beams under " + track->name() + " path " + std::to_string( path ) );
                    }
                }
            }
        }
    }
    return result;
}

void unsettle_around( TTrack &Track ) {

    ++layout_epoch();
    auto const sections { simulation::Region->sections( Track.location(), static_cast<float>( Track.radius() + 50.0 ) ) };
    for( auto *section : sections ) {
        for( auto &cell : section->m_cells ) {
            if( glm::length( cell.area().center - Track.location() ) > cell.area().radius + Track.radius() + around_margin ) {
                continue;
            }
            for( auto *other : cell.m_directories.paths ) {
                if( other == nullptr || false == other->m_sleepers3d.active ) {
                    continue;
                }
                if( glm::length( other->location() - Track.location() ) > other->radius() + Track.radius() + around_margin ) {
                    continue;
                }
                other->m_sleepers3d.dirty = true;
            }
        }
    }
}

} // namespace track_sleepers
