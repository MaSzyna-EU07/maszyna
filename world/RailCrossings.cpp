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
#include "world/TrackFeatures.h"
#include "simulation/simulation.h"
#include "scene/scene.h"

using namespace track_rails;

namespace {

bool joined( TTrack const &Track, TTrack const &Other ) {

    auto const next = []( TTrack const &From, TTrack const &To ) {
        if( From.trPrev == &To || From.trNext == &To ) {
            return true;
        }
        if( From.eType == tt_Switch && From.SwitchExtension ) {
            for( int i = 0; i < 2; ++i ) {
                if( From.SwitchExtension->pPrevs[ i ] == &To || From.SwitchExtension->pNexts[ i ] == &To ) {
                    return true;
                }
            }
        }
        return false; };
    return next( Track, Other ) || next( Other, Track );
}

struct rails_memo {
    std::optional<gauge_line> lines[ 2 ][ 2 ][ 2 ]; // path, right side, from the blades
    std::optional<std::vector<track_rails::rail_crossing>> own;
    std::optional<std::vector<track_rails::rail_crossing>> joined;
    std::optional<track_rails::check_rail_set> checks;
    std::optional<track_rails::switch_blade> blades[ 2 ];
};
std::unordered_map<TTrack const *, rails_memo> memos;
unsigned int memostamp { 0 };
unsigned int stamp { 1 };

rails_memo &memo_of( TTrack const &Track ) {

    if( memostamp != stamp ) {
        memos.clear();
        memostamp = stamp;
    }
    return memos[ &Track ];
}

} // namespace

namespace track_rails {

void forget() {

    ++stamp;
}

unsigned int memo_stamp() {

    return stamp;
}


double wing_reach( double const Slope ) {

    return 0.15 / Slope + 0.3;
}

double movable_nose_length( double const Slope ) {

    return 0.4 / Slope;
}

double check_length( double const Slope ) {

    return std::clamp( 0.5 / Slope, 3.6, 9.0 );
}

// the rails of the track are cut where they cross other rails: plain tracks, and switches whose frog is made from the rails
bool cut_rails( TTrack const &Track ) {

    return track_features::crossings
        && ( Track.iCategoryFlag & 15 ) == 1
        && Track.m_material1 != null_handle
        && false == Track.m_editorremoved
        && ( Track.eType == tt_Normal ? Track.Segment != nullptr :
             Track.eType == tt_Switch && Track.SwitchExtension && Track.SwitchExtension->m_frogs && Track.SwitchExtension->Segments[ 0 ] && Track.SwitchExtension->Segments[ 1 ] );
}

int blade_steps( TTrack const &Track, int const Path ) {

    return static_cast<int>( std::ceil( Track.SwitchExtension->Segments[ Path ]->RaSegCount() * 0.65 ) );
}

double blade_length( TTrack const &Track, int const Path ) {

    if( auto const &shape { Track.blade( Path ) }; shape.valid ) {
        return std::min( shape.heel, Track.SwitchExtension->Segments[ Path ]->GetLength() );
    }
    auto const &segment { *Track.SwitchExtension->Segments[ Path ] };
    return segment.GetLength() * blade_steps( Track, Path ) / segment.RaSegCount();
}

TTrack const *runon_track( TTrack const &Switch, int const Path ) {

    auto const *next { path_neighbour( Switch, Path, 1 ) };
    if( next == nullptr || next->eType != tt_Normal || next->Segment == nullptr || next == &Switch ) {
        return nullptr;
    }
    if( glm::distance( Switch.SwitchExtension->Segments[ Path ]->FastGetPoint_1(), next->Segment->FastGetPoint_0() ) > 0.01 ) {
        return nullptr;
    }
    // only as in a double switch: the plain track leads on to the start of the next switch, not to the end of the other half of a slip
    auto const *after { next->trNext };
    if( after == nullptr || after->eType != tt_Switch || after->SwitchExtension == nullptr
     || glm::distance( after->SwitchExtension->Segments[ 0 ]->FastGetPoint_0(), next->Segment->FastGetPoint_1() ) > 0.01 ) {
        return nullptr;
    }
    return next;
}

double blade_runon( TTrack const &Track, int const Side ) {

    auto const *owner { Track.trPrev };
    if( Track.eType != tt_Normal || Track.Segment == nullptr || owner == nullptr || owner->eType != tt_Switch || false == cut_rails( *owner ) ) {
        return 0.0;
    }
    for( int path = 0; path < 2; ++path ) {
        if( runon_track( *owner, path ) != &Track || ( owner->SwitchExtension->RightSwitch == ( path == 0 ) ? 1 : -1 ) != Side ) {
            continue;
        }
        auto const &shape { owner->blade( path ) };
        auto const length { owner->SwitchExtension->Segments[ path ]->GetLength() };
        if( shape.valid && shape.heel > length ) {
            return std::min( shape.heel - length, Track.Segment->GetLength() );
        }
    }
    return 0.0;
}

TSegment const &path_segment( TTrack const &Track, int const Path ) {

    return ( Track.eType == tt_Switch ? *Track.SwitchExtension->Segments[ Path ] : *Track.Segment );
}

double knuckle_bend( double const Into ) {

    return std::abs( Into ) / ( 1.0 + std::sqrt( std::max( 0.0, 1.0 - Into * Into ) ) );
}

// a check rail starting right beside the end of a wing rail running along the same rail, which the wing rail goes on in place of; the far end of it
bool replaced_check( std::vector<check_rail> const &Checks, double const Wingend, double const Direction, double &Far ) {

    for( auto const &check : Checks ) {
        auto const nearend { Direction > 0.0 ? check.from : check.to };
        auto const farend { Direction > 0.0 ? check.to : check.from };
        auto const gap { ( nearend - Wingend ) * Direction };
        if( gap > -0.5 && gap < 1.0 && ( farend - Wingend ) * Direction > 0.5 ) {
            Far = farend;
            return true;
        }
    }
    return false;
}

bool replaced_by_wing( check_rail const &Check, knots::line const &Wingends ) {

    auto farend { 0.0 };
    return std::any_of( Wingends.begin(), Wingends.end(), [&]( glm::dvec2 const &Wing ) { return replaced_check( { Check }, Wing.x, Wing.y, farend ); } );
}

// gauge line of a rail of a track, along a path of a switch from past its blades when Blades
gauge_line const &rail_line( TTrack const &Track, int const Path, int const Side, bool const Blades ) {

    auto const blades { Blades && Track.eType == tt_Switch };
    auto &line { memo_of( Track ).lines[ Path ][ Side ][ blades ] };
    if( false == line.has_value() ) {
        line = make_gauge_line( path_segment( Track, Path ), ( Side ? 1.0 : -1.0 ) * ( 0.5 * std::abs( Track.fTrackWidth ) ), blades ? blade_length( Track, Path ) : 0.0 );
    }
    return *line;
}

track_rails::check_rail_set const &rail_checks( TTrack &Track ) {

    auto &checks { memo_of( Track ).checks };
    if( false == checks.has_value() ) {
        checks = Track.check_rails( Track.rail_crossings() );
    }
    return *checks;
}

glm::dvec3 rail_point( TTrack const &Track, int const Path, int const Side, double const Distance, double const Lateral ) {

    auto const line { make_gauge_line( path_segment( Track, Path ), Side * 0.5 * std::abs( Track.fTrackWidth ), 0.0 ) };
    glm::dvec3 point, tangent;
    line.at( Distance, point, tangent );
    return point - line.normal( Distance, Side ) * Lateral;
}

void rail_offset( TTrack const &Track, int const Path, int const Side, glm::dvec3 const &Point, double &Distance, double &Lateral ) {

    auto const line { make_gauge_line( path_segment( Track, Path ), Side * 0.5 * std::abs( Track.fTrackWidth ), 0.0 ) };
    Distance = nearest_distance( line, Point );
    Lateral = lateral( line, Side, Point, Distance );
}

int path_count( TTrack const &Track ) {

    return ( Track.eType == tt_Switch ? 2 : 1 );
}

TTrack *path_neighbour( TTrack const &Track, int const Path, int const End ) {

    if( Track.eType == tt_Switch ) {
        return ( End ? Track.SwitchExtension->pNexts[ Path ] : Track.SwitchExtension->pPrevs[ Path ] );
    }
    return ( End ? Track.trNext : Track.trPrev );
}

} // namespace track_rails

namespace {

struct rail_run {
    TTrack *track { nullptr };
    int path { 0 };
    int side { 0 };
    gauge_line const *line { nullptr };
    double from { 0.0 };
    double length { 0.0 };
};

std::vector<TTrack *> nearby_rails( TTrack &Track ) {

    std::vector<TTrack *> others;
    auto const sections { simulation::Region->sections( Track.location(), static_cast<float>( Track.radius() + 50.0 ) ) };
    for( auto *section : sections ) {
        for( auto const &cell : section->m_cells ) {
            if( glm::length( cell.area().center - Track.location() ) > cell.area().radius + Track.radius() + 1.0 ) {
                continue;
            }
            for( auto *other : cell.m_directories.paths ) {
                if( other == nullptr || other == &Track || false == cut_rails( *other ) ) {
                    continue;
                }
                if( glm::length( other->location() - Track.location() ) > other->radius() + Track.radius() + 1.0 ) {
                    continue;
                }
                if( std::find( others.begin(), others.end(), other ) == others.end() ) {
                    others.push_back( other );
                }
            }
        }
    }
    return others;
}

double line_bend( gauge_line const &Line, double const At ) {

    glm::dvec3 point, before, after;
    Line.at( At - 1.0, point, before );
    Line.at( At + 1.0, point, after );
    return std::abs( cross( plan( before ), plan( after ) ) );
}

double heading( glm::dvec2 const &Tangent ) {

    auto const angle { std::atan2( Tangent.y, Tangent.x ) };
    return ( angle < 0.0 ? angle + M_PI : angle );
}

bool overlap( gauge_line const &A, gauge_line const &B ) {

    return A.low.x <= B.high.x && B.low.x <= A.high.x && A.low.y <= B.high.y && B.low.y <= A.high.y;
}

bool same_place( rail_crossing const &Crossing, int const Path, int const Side, double const At ) {

    return Crossing.path == Path && Crossing.side == Side && std::abs( Crossing.at - At ) < 0.05;
}

void refine_meeting( gauge_line const &Ours, gauge_line const &Theirs, double &S, double &T ) {

    glm::dvec3 p, tp, q, tq;
    for( int step = 0; step < 6; ++step ) {
        Ours.at( S, p, tp );
        Theirs.at( T, q, tq );
        auto const d { cross( plan( tp ), plan( tq ) ) };
        if( std::abs( d ) < 1e-9 ) {
            break;
        }
        auto const offset { plan( q ) - plan( p ) };
        S += cross( offset, plan( tq ) ) / d;
        T += cross( offset, plan( tp ) ) / d;
    }
}

rail_crossing make_crossing( rail_run const &Ours, rail_run const &Theirs, double const S, double const T, glm::dvec2 const &Tangent, glm::dvec2 const &Othertangent, double const Angle ) {

    auto const normal { glm::normalize( plan( Ours.line->normal( S, Ours.side ) ) ) };
    auto const othernormal { glm::normalize( plan( Theirs.line->normal( T, Theirs.side ) ) ) };
    rail_crossing crossing;
    crossing.track = Theirs.track;
    crossing.path = Ours.path;
    crossing.side = Ours.side;
    crossing.at = S;
    crossing.angle = Angle;
    crossing.into = glm::dot( Tangent, othernormal );
    crossing.facing = glm::dot( normal, othernormal );
    auto const ourbend { line_bend( *Ours.line, S ) };
    auto const theirbend { line_bend( *Theirs.line, T ) };
    crossing.nose = ( std::abs( ourbend - theirbend ) > 1e-4 ? ourbend < theirbend : heading( Tangent ) < heading( Othertangent ) );
    crossing.otherpath = Theirs.path;
    crossing.otherside = Theirs.side;
    crossing.otherat = T;
    crossing.aligned = glm::dot( Tangent, Othertangent ) > 0.0;
    return crossing;
}

void meet_rails( rail_run const &Ours, rail_run const &Theirs, std::vector<rail_crossing> &Result ) {

    auto const &line { *Ours.line };
    auto const &theirs { *Theirs.line };
    if( false == overlap( line, theirs ) ) {
        return;
    }
    for( std::size_t i = 1; i < line.points.size(); ++i ) {
        auto const a0 { plan( line.points[ i - 1 ] ) };
        auto const a { plan( line.points[ i ] ) - a0 };
        auto const alow { glm::min( a0, a0 + a ) - 0.02 * glm::length( a ) - 1e-6 };
        auto const ahigh { glm::max( a0, a0 + a ) + 0.02 * glm::length( a ) + 1e-6 };
        if( alow.x > theirs.high.x + 1.0 || theirs.low.x - 1.0 > ahigh.x || alow.y > theirs.high.y + 1.0 || theirs.low.y - 1.0 > ahigh.y ) {
            continue;
        }
        for( std::size_t j = 1; j < theirs.points.size(); ++j ) {
            auto const b0 { plan( theirs.points[ j - 1 ] ) };
            auto const b { plan( theirs.points[ j ] ) - b0 };
            auto const reach { 0.02 * glm::length( b ) };
            if( alow.x > std::max( b0.x, b0.x + b.x ) + reach || std::min( b0.x, b0.x + b.x ) - reach > ahigh.x
             || alow.y > std::max( b0.y, b0.y + b.y ) + reach || std::min( b0.y, b0.y + b.y ) - reach > ahigh.y ) {
                continue;
            }
            double u, v;
            if( false == meet( a0, a, b0, b, 1e-12, u, v ) ) {
                continue;
            }
            if( u < -0.01 || u > 1.01 || v < -0.01 || v > 1.01 ) {
                continue;
            }
            auto s { glm::mix( line.distances[ i - 1 ], line.distances[ i ], u ) };
            auto t { glm::mix( theirs.distances[ j - 1 ], theirs.distances[ j ], v ) };
            refine_meeting( line, theirs, s, t );
            glm::dvec3 p, tp, q, tq;
            line.at( s, p, tp );
            theirs.at( t, q, tq );
            if( glm::length( plan( p ) - plan( q ) ) > 0.01
             || s < 0.0 || s > Ours.length
             || t < Theirs.from || t > Theirs.length ) {
                continue;
            }
            if( std::abs( p.y - q.y ) > 0.5 ) {
                continue;
            }
            if( ( s < 0.5 || s > Ours.length - 0.5 ) && ( t < 0.5 || t > Theirs.length - 0.5 ) && joined( *Ours.track, *Theirs.track ) ) {
                continue;
            }
            auto const tangent { glm::normalize( plan( tp ) ) };
            auto const othertangent { glm::normalize( plan( tq ) ) };
            auto const angle { std::acos( std::min( 1.0, std::abs( glm::dot( tangent, othertangent ) ) ) ) };
            if( angle < 0.03 ) {
                continue;
            }
            if( std::any_of( Result.begin(), Result.end(), [&]( rail_crossing const &Crossing ) { return same_place( Crossing, Ours.path, Ours.side, s ); } ) ) {
                continue;
            }
            Result.push_back( make_crossing( Ours, Theirs, s, t, tangent, othertangent, angle ) );
        }
    }
}

std::vector<rail_crossing> own_crossings( TTrack &Track ) {

    std::vector<rail_crossing> result;
    for( auto *other : nearby_rails( Track ) ) {
        for( int otherpath = 0; otherpath < path_count( *other ); ++otherpath ) {
            rail_run theirs { other, otherpath };
            theirs.from = ( other->eType == tt_Switch ? blade_length( *other, otherpath ) : 0.0 );
            theirs.length = path_segment( *other, otherpath ).GetLength();
            for( int otherside = 0; otherside < 2; ++otherside ) {
                theirs.side = ( otherside ? 1 : -1 );
                theirs.line = &rail_line( *other, otherpath, otherside, true );
                for( int path = 0; path < path_count( Track ); ++path ) {
                    auto const length { path_segment( Track, path ).GetLength() };
                    for( int side = 0; side < 2; ++side ) {
                        meet_rails( { &Track, path, side ? 1 : -1, &rail_line( Track, path, side, false ), 0.0, length }, theirs, result );
                    }
                }
            }
        }
    }
    return result;
}

// how far the blades of a part of a double slip may reach: up to the check rails of the crossings of the rails of the diamond, found
// with the blades of the other parts left out of the account, as they're yet to be made as short; at the latest short of their throats,
// the check rails then cut to begin past the heels
// crossings of the rails of the track with the rails of the tracks around, with the blades of switches left out of the account
std::vector<rail_crossing> unbladed_crossings( TTrack const &Track ) {

    auto &track { const_cast<TTrack &>( Track ) };
    std::vector<rail_crossing> found;
    for( auto *other : nearby_rails( track ) ) {
        for( int otherpath = 0; otherpath < path_count( *other ); ++otherpath ) {
            rail_run theirs { other, otherpath };
            theirs.length = path_segment( *other, otherpath ).GetLength();
            for( int otherside = 0; otherside < 2; ++otherside ) {
                theirs.side = ( otherside ? 1 : -1 );
                theirs.line = &rail_line( *other, otherpath, otherside, false );
                for( int path = 0; path < path_count( track ); ++path ) {
                    for( int side = 0; side < 2; ++side ) {
                        meet_rails( { &track, path, side ? 1 : -1, &rail_line( track, path, side, false ), 0.0, path_segment( track, path ).GetLength() }, theirs, found );
                    }
                }
            }
        }
    }
    return found;
}

std::pair<double, double> slip_blade_reach( TTrack const &Track ) {

    auto reach { 1e9 };
    auto latest { 1e9 };
    for( auto const &crossing : unbladed_crossings( Track ) ) {
        if( crossing.angle <= middle_angle ) {
            auto const slope { std::tan( crossing.angle ) };
            reach = std::min( reach, crossing.at - 0.25 - 0.5 * check_length( slope ) - 0.3 );
            latest = std::min( latest, crossing.at - flangeway / slope - 0.6 );
        }
    }
    return { reach, latest };
}

// how far the blade of a switch may reach: short of the throat of a frog of another track on its rail, as in a double switch with the middle frog right past the heels,
// where the wing rail has to bend out of the line of the rail coming from the blade
double blade_reach( TTrack const &Track, int const Path, int const Side ) {

    auto reach { 1e9 };
    for( auto const &crossing : unbladed_crossings( Track ) ) {
        if( crossing.path == Path && crossing.side == Side && crossing.angle <= middle_angle ) {
            reach = std::min( reach, crossing.at - flangeway / std::tan( crossing.angle ) - 0.05 );
        }
    }
    return reach;
}

// crossings of the tracks going on from the ends, near enough for their check rails and wings to reach this one
void add_neighbour_crossings( TTrack &Track, std::vector<rail_crossing> &Result ) {

    auto const extension { 10.0 };
    auto const owncount { Result.size() };
    for( int path = 0; path < path_count( Track ); ++path ) {
        auto const &segment { path_segment( Track, path ) };
        auto const length { segment.GetLength() };
        for( int end = 0; end < 2; ++end ) {
            auto *neighbour { path_neighbour( Track, path, end ) };
            if( neighbour == nullptr || neighbour == &Track || false == cut_rails( *neighbour ) ) {
                continue;
            }
            auto const joint { end ? segment.FastGetPoint_1() : segment.FastGetPoint_0() };
            auto const theirs { neighbour->rail_crossings( false ) };
            for( int neighbourpath = 0; neighbourpath < path_count( *neighbour ); ++neighbourpath ) {
                auto const &neighboursegment { path_segment( *neighbour, neighbourpath ) };
                auto const neighbourlength { neighboursegment.GetLength() };
                auto const tostart { glm::length( neighboursegment.FastGetPoint_0() - joint ) };
                auto const toend { glm::length( neighboursegment.FastGetPoint_1() - joint ) };
                if( std::min( tostart, toend ) > 0.05 ) {
                    continue;
                }
                auto const atstart { tostart <= toend };
                auto const same { atstart == ( end == 1 ) };
                for( auto const &crossing : theirs ) {
                    if( crossing.path != neighbourpath || crossing.track == &Track ) {
                        continue;
                    }
                    auto const along { atstart ? crossing.at : neighbourlength - crossing.at };
                    if( along > extension ) {
                        continue;
                    }
                    auto mapped { crossing };
                    mapped.path = path;
                    mapped.at = ( end ? length + along : -along );
                    mapped.owner = neighbour;
                    if( false == same ) {
                        mapped.side = -crossing.side;
                        mapped.into = -crossing.into;
                        mapped.aligned = !crossing.aligned;
                    }
                    if( std::none_of( Result.begin(), Result.begin() + owncount, [&]( rail_crossing const &Crossing ) { return same_place( Crossing, mapped.path, mapped.side, mapped.at ); } ) ) {
                        Result.push_back( mapped );
                    }
                }
            }
        }
    }
}

struct planned_check {
    double from { 0.0 };
    double to { 0.0 };
    bool flarefrom { true };
    bool flareto { true };
    double shortest { 1.5 };
    double flare { 0.02 };
    double frombend { 0.0 };
    double tobend { 0.0 };
    bool acute { false }; // opposite an acute crossing, it keeps clear of the rails of the tracks crossing this one
};
using planned_check_set = std::array<std::array<std::vector<planned_check>, 2>, 2>; // path, right side

void add_acute_check( std::vector<planned_check> &Checks, double const At, double const Toward, double const Slope, double const Flare ) {

    auto const half { 0.5 * check_length( Slope ) };
    Checks.push_back( { At + Toward * 0.25 - half, At + Toward * 0.25 + half, true, true, 1.5, Flare, 0.0, 0.0, true } );
}

planned_check_set planned_checks( TTrack const &Track, std::vector<rail_crossing> const &Crossings ) {

    planned_check_set plans;
    for( auto const &crossing : Crossings ) {
        if( crossing.angle > middle_angle || crossing.track->rail_dimensions().grooved ) {
            continue;
        }
        if( crossing.facing < 0.0 ) {
            add_acute_check( plans[ crossing.path ][ crossing.side < 0 ], crossing.at, crossing.into > 0.0 ? 1.0 : -1.0, std::tan( crossing.angle ), 0.02 );
        }
        else if( crossing.into > 0.0 ) {
            plans[ crossing.path ][ crossing.side > 0 ].push_back( { crossing.at, crossing.at + 1.2, false, true, 0.5, 0.02, -knuckle_bend( crossing.into ), 0.0 } );
        }
        else {
            plans[ crossing.path ][ crossing.side > 0 ].push_back( { crossing.at - 1.2, crossing.at, true, false, 0.5, 0.02, 0.0, knuckle_bend( crossing.into ) } );
        }
    }
    if( Track.eType == tt_Switch ) {
        auto const frog { Track.frog() };
        // the movable nose leaves no gap for the wheels, the frog has no check rails
        if( frog.valid && false == Track.SwitchExtension->m_movablenose ) {
            for( int path = 0; path < 2; ++path ) {
                add_acute_check( plans[ path ][ frog.side[ path ] < 0 ], frog.at[ path ], -1.0, std::tan( frog.angle ), 0.04 );
            }
        }
    }
    return plans;
}

void merge_checks( std::vector<planned_check> &Checks ) {

    std::sort( Checks.begin(), Checks.end(), []( planned_check const &A, planned_check const &B ) { return A.from < B.from; } );
    for( std::size_t i = 1; i < Checks.size(); ++i ) {
        if( Checks[ i ].from < Checks[ i - 1 ].to + 2.0 ) {
            if( Checks[ i ].to > Checks[ i - 1 ].to ) {
                Checks[ i - 1 ].to = Checks[ i ].to;
                Checks[ i - 1 ].flareto = Checks[ i ].flareto;
                Checks[ i - 1 ].tobend = Checks[ i ].tobend;
            }
            Checks[ i - 1 ].shortest = std::min( Checks[ i - 1 ].shortest, Checks[ i ].shortest );
            Checks[ i - 1 ].flare = std::max( Checks[ i - 1 ].flare, Checks[ i ].flare );
            Checks[ i - 1 ].acute = Checks[ i - 1 ].acute || Checks[ i ].acute;
            Checks.erase( Checks.begin() + i-- );
        }
    }
}

bool blocked_by( std::vector<gauge_line> const &Others, glm::dvec2 const &Gauge, glm::dvec2 const &Inward, double const Clearance ) {

    auto blocked { false };
    for( auto const &other : Others ) {
        for( std::size_t i = 1; i < other.points.size() && false == blocked; ++i ) {
            auto const a { plan( other.points[ i - 1 ] ) };
            auto const edge { plan( other.points[ i ] ) - a };
            double t, u;
            if( false == meet( Gauge, Inward, a, edge, 1e-9, t, u ) ) {
                continue;
            }
            blocked = ( u > -1e-6 && u < 1.0 + 1e-6 && t > -0.05 && t < Clearance );
        }
    }
    return blocked;
}

// check rails keep clear of the blades and the rails of the other path of a switch running inside the rail, here and in a switch joined at an end, and of the blade tips;
// those opposite acute crossings also of the rails of the Crossing tracks, as where the slip curves of a double slip cross the diamond
knots::line check_blocks( TTrack const &Track, int const Path, int const Side, std::vector<planned_check> const &Checks, double const Clearance, std::vector<gauge_line> const &Crossing ) {

    auto const halfwidth { 0.5 * std::abs( Track.fTrackWidth ) };
    auto const &segment { path_segment( Track, Path ) };
    auto const length { segment.GetLength() };
    knots::line blocks;
    auto const start { plan( segment.FastGetPoint_0() ) };
    auto const end { plan( segment.FastGetPoint_1() ) };
    std::vector<gauge_line> others;
    auto const add_path = [&]( TSegment const &Other ) {
        others.push_back( make_gauge_line( Other, -halfwidth, 0.0 ) );
        others.push_back( make_gauge_line( Other, halfwidth, 0.0 ) ); };
    if( Track.eType == tt_Switch ) {
        // past the heels of the blades
        auto heel { 0.0 };
        for( int path = 0; path < 2; ++path ) {
            auto const &blade { Track.blade( path ) };
            if( blade.valid ) {
                heel = std::max( heel, blade.heel );
            }
        }
        blocks.emplace_back( -1e9, heel + 0.3 );
        add_path( path_segment( Track, 1 - Path ) );
    }
    for( auto const *neighbour : { Track.trPrev, Track.trNext } ) {
        if( neighbour == nullptr || neighbour == &Track || neighbour->eType != tt_Switch || neighbour->SwitchExtension == nullptr ) {
            continue;
        }
        auto const tip { plan( neighbour->SwitchExtension->Segments[ 0 ]->FastGetPoint_0() ) };
        if( glm::distance( start, tip ) < 0.1 ) {
            blocks.emplace_back( -1e9, 0.3 );
            continue;
        }
        if( glm::distance( end, tip ) < 0.1 ) {
            blocks.emplace_back( length - 0.3, 1e9 );
            continue;
        }
        for( int otherpath = 0; otherpath < 2; ++otherpath ) {
            auto const &other { *neighbour->SwitchExtension->Segments[ otherpath ] };
            auto const continues = [&]( glm::dvec2 const &Point ) {
                return glm::distance( Point, plan( other.FastGetPoint_0() ) ) < 0.1 || glm::distance( Point, plan( other.FastGetPoint_1() ) ) < 0.1; };
            if( false == continues( start ) && false == continues( end ) ) {
                add_path( other );
            }
        }
    }
    if( false == others.empty() || false == Crossing.empty() ) {
        auto const ours { make_gauge_line( segment, Side * halfwidth, 0.0 ) };
        for( auto const &check : Checks ) {
            for( auto s { check.from }; s <= check.to; s += 0.1 ) {
                glm::dvec3 point, tangent;
                ours.at( s, point, tangent );
                auto const inward { plan( ours.normal( s, Side ) ) };
                if( blocked_by( others, plan( point ), inward, Clearance )
                 || ( check.acute && blocked_by( Crossing, plan( point ), inward, Clearance ) ) ) {
                    blocks.emplace_back( s - 0.3, s + 0.3 );
                }
            }
        }
    }
    knots::sort( blocks );
    return blocks;
}

void cut_check( planned_check const &Check, knots::line const &Blocks, std::vector<check_rail> &Result ) {

    auto const add = [&]( double const From, double const To ) {
        Result.push_back( {
            From, To,
            ( From > Check.from || Check.flarefrom ) ? Check.flare : 0.0,
            ( To < Check.to || Check.flareto ) ? Check.flare : 0.0,
            From > Check.from ? 0.0 : Check.frombend,
            To < Check.to ? 0.0 : Check.tobend } ); };
    auto from { Check.from };
    for( auto const &block : Blocks ) {
        if( block.y <= from || block.x >= Check.to ) {
            continue;
        }
        if( block.x - from >= Check.shortest ) {
            add( from, block.x );
        }
        from = std::max( from, block.y );
    }
    if( Check.to - from >= Check.shortest ) {
        add( from, Check.to );
    }
}

} // namespace

std::vector<track_rails::rail_crossing> TTrack::rail_crossings( bool const Joined ) {

    if( false == cut_rails( *this ) || simulation::Region == nullptr ) {
        return {};
    }
    auto &memo { memo_of( *this ) };
    if( Joined && memo.joined.has_value() ) {
        return *memo.joined;
    }
    if( false == memo.own.has_value() ) {
        memo.own = own_crossings( *this );
    }
    auto result { *memo.own };
    if( false == Joined ) {
        return result;
    }
    add_neighbour_crossings( *this, result );
    memo.joined = result;
    return result;
}

track_rails::check_rail_set TTrack::check_rails( std::vector<rail_crossing> const &Crossings ) {

    check_rail_set result;
    auto const size { rail_dimensions() };
    if( size.grooved ) {
        return result;
    }
    auto plans { planned_checks( *this, Crossings ) };
    auto const clearance { 0.046 + 0.04 * size.head / 0.0735 + size.head };
    std::vector<gauge_line> crossing;
    std::vector<TTrack const *> crossers;
    for( auto const &entry : Crossings ) {
        if( entry.track == nullptr || entry.track == this || std::find( crossers.begin(), crossers.end(), entry.track ) != crossers.end() ) {
            continue;
        }
        crossers.push_back( entry.track );
        for( int otherpath = 0; otherpath < path_count( *entry.track ); ++otherpath ) {
            for( auto const side : { -1.0, 1.0 } ) {
                crossing.push_back( make_gauge_line( path_segment( *entry.track, otherpath ), side * 0.5 * std::abs( entry.track->fTrackWidth ), 0.0 ) );
            }
        }
    }
    for( int path = 0; path < path_count( *this ); ++path ) {
        for( int right = 0; right < 2; ++right ) {
            auto &checks { plans[ path ][ right ] };
            if( checks.empty() ) {
                continue;
            }
            merge_checks( checks );
            auto const blocks { check_blocks( *this, path, right ? 1 : -1, checks, clearance, crossing ) };
            for( auto const &check : checks ) {
                cut_check( check, blocks, result[ path ][ right ] );
            }
        }
    }
    return result;
}

track_rails::switch_frog TTrack::frog() const {

    switch_frog result;
    if( eType != tt_Switch || false == cut_rails( *this ) ) {
        return result;
    }
    result.side[ 0 ] = blade_side( 0 );
    result.side[ 1 ] = -result.side[ 0 ];
    auto const halfwidth { 0.5 * std::abs( fTrackWidth ) };
    std::vector<glm::dvec2> lines[ 2 ];
    std::vector<double> distances[ 2 ];
    for( int path = 0; path < 2; ++path ) {
        auto const &segment { *SwitchExtension->Segments[ path ] };
        auto const length { segment.GetLength() };
        result.blades[ path ] = blade_length( *this, path );
        auto const count { std::max( 1, static_cast<int>( std::ceil( length / 0.25 ) ) ) };
        for( int i = 0; i <= count; ++i ) {
            auto const distance { length * i / count };
            glm::dvec3 point, across;
            segment.plan_frame( distance, point, across );
            auto const gauge { point + across * ( result.side[ path ] * halfwidth ) };
            lines[ path ].emplace_back( gauge.x, gauge.z );
            distances[ path ].push_back( distance );
        }
    }
    for( std::size_t i = 1; i < lines[ 0 ].size() && false == result.valid; ++i ) {
        if( distances[ 0 ][ i ] <= result.blades[ 0 ] ) {
            continue;
        }
        auto const a0 { lines[ 0 ][ i - 1 ] };
        auto const a { lines[ 0 ][ i ] - a0 };
        for( std::size_t j = 1; j < lines[ 1 ].size(); ++j ) {
            if( distances[ 1 ][ j ] <= result.blades[ 1 ] ) {
                continue;
            }
            auto const b0 { lines[ 1 ][ j - 1 ] };
            auto const b { lines[ 1 ][ j ] - b0 };
            double u, v;
            if( false == meet( a0, a, b0, b, 1e-12, u, v ) ) {
                continue;
            }
            if( u < 0.0 || u > 1.0 || v < 0.0 || v > 1.0 ) {
                continue;
            }
            result.at[ 0 ] = glm::mix( distances[ 0 ][ i - 1 ], distances[ 0 ][ i ], u );
            result.at[ 1 ] = glm::mix( distances[ 1 ][ j - 1 ], distances[ 1 ][ j ], v );
            result.valid = true;
            break;
        }
    }
    if( false == result.valid ) {
        return result;
    }
    glm::dvec3 point, across0, across1;
    SwitchExtension->Segments[ 0 ]->plan_frame( result.at[ 0 ], point, across0 );
    SwitchExtension->Segments[ 1 ]->plan_frame( result.at[ 1 ], point, across1 );
    result.angle = std::acos( std::min( 1.0, std::abs( glm::dot( across0, across1 ) ) ) );
    if( result.angle < 0.01 || result.angle > 0.5 ) {
        result.valid = false;
        return result;
    }
    for( int path = 0; path < 2; ++path ) {
        auto const throat { result.at[ path ] - flangeway / std::tan( result.angle ) };
        if( throat <= result.blades[ path ] || result.at[ path ] >= SwitchExtension->Segments[ path ]->GetLength() ) {
            result.valid = false;
        }
    }
    return result;
}

double TTrack::blades_throw() const {

    return ( eType == tt_Switch && SwitchExtension && SwitchExtension->m_bladethrow > 0.0 ? SwitchExtension->m_bladethrow : blade_throw );
}

track_rails::switch_blade const &TTrack::blade( int const Path ) const {

    auto &cached { memo_of( *this ).blades[ Path ] };
    if( cached.has_value() ) {
        return *cached;
    }
    cached = switch_blade {};
    auto &result { *cached };
    if( eType != tt_Switch || false == cut_rails( *this ) || m_profiles.empty() ) {
        return result;
    }
    auto const side { blade_side( Path ) };
    auto const &ours { rail_line( *this, Path, side > 0, false ) };
    auto const &theirs { rail_line( *this, 1 - Path, side > 0, false ) };
    auto const size { rail_dimensions() };
    // a path ending short of the other one with a plain track going on from it, as the first switch of a double one has, lets the blades run on
    // into that track, taken straight on from the end, short of the track past it
    auto const runs = [&]( int const P ) {
        auto const *next { track_rails::runon_track( *this, P ) };
        return SwitchExtension->Segments[ P ]->GetLength() + ( next != nullptr ? next->Segment->GetLength() - blade_runon_margin : 0.0 ); };
    auto const shortest { std::min( SwitchExtension->Segments[ Path ]->GetLength(), SwitchExtension->Segments[ 1 - Path ]->GetLength() ) };
    auto const length { std::max( shortest, std::min( {
        std::max( SwitchExtension->Segments[ Path ]->GetLength(), SwitchExtension->Segments[ 1 - Path ]->GetLength() ),
        runs( Path ),
        runs( 1 - Path ) } ) ) };
    auto const count { std::max( 2, static_cast<int>( std::ceil( length / 0.1 ) ) ) };
    auto &gauge { result.gauge };
    glm::dvec3 point, tangent;
    ours.at( 0.0, point, tangent );
    auto otherat { nearest_distance( theirs, point ) };
    for( int i = 0; i <= count; ++i ) {
        auto const distance { length * i / count };
        ours.at( distance, point, tangent );
        lateral( theirs, side, point, otherat );
        glm::dvec3 line;
        theirs.at( otherat, line, tangent );
        auto at { distance };
        auto const gap { lateral( ours, side, line, at ) };
        gauge.emplace_back( at, gap );
    }
    std::sort( gauge.begin(), gauge.end(), []( glm::dvec2 const &A, glm::dvec2 const &B ) { return A.x < B.x; } );
    if( std::abs( gauge.front().y ) > 0.002 ) {
        return result;
    }
    auto const reach = [&]( double const Width ) {
        for( std::size_t i = 1; i < gauge.size(); ++i ) {
            if( gauge[ i ].y >= Width ) {
                auto const &a { gauge[ i - 1 ] };
                auto const &b { gauge[ i ] };
                return glm::mix( a.x, b.x, std::clamp( ( Width - a.y ) / std::max( b.y - a.y, 1e-9 ), 0.0, 1.0 ) );
            }
        }
        return -1.0; };
    auto const level = [&]( double const Distance ) {
        for( std::size_t i = 1; i < gauge.size(); ++i ) {
            if( gauge[ i ].x >= Distance ) {
                auto const &a { gauge[ i - 1 ] };
                auto const &b { gauge[ i ] };
                return glm::mix( a.y, b.y, std::clamp( ( Distance - a.x ) / std::max( b.x - a.x, 1e-9 ), 0.0, 1.0 ) );
            }
        }
        return gauge.back().y; };
    auto const turn = [&]( double const Distance ) {
        glm::dvec3 here, heading, line, along;
        ours.at( Distance, here, heading );
        auto there { nearest_distance( theirs, here ) };
        lateral( theirs, side, here, there );
        theirs.at( there, line, along );
        return std::abs( glm::cross( glm::normalize( glm::dvec3 { heading.x, 0.0, heading.z } ), glm::normalize( glm::dvec3 { along.x, 0.0, along.z } ) ).y ); };
    auto const thin { reach( blade_point ) };
    auto const thick { reach( blade_nose ) };
    auto angled { false };
    if( thin >= 0.0 && thick > thin ) {
        auto const at { turn( thin ) };
        auto const bending { ( turn( thick ) - at ) / ( thick - thin ) };
        angled = std::sqrt( std::max( 0.0, at * at - 2.0 * bending * blade_point ) ) >= blade_tangent;
    }
    result.tip = ( angled || thick < 0.0 ? thin : std::max( thin, thick - blade_planed ) );
    for( auto const path : { Path, 1 - Path } ) {
        auto const &segment { *SwitchExtension->Segments[ path ] };
        auto mark { -1.0 };
        for( std::size_t piece = 1; piece < segment.PieceCount(); ++piece ) {
            auto const at { segment.PieceStart( piece ) };
            auto const apart { level( at ) };
            if( apart > blade_mark ) {
                break;
            }
            if( angled || apart >= blade_gap ) {
                mark = at;
            }
        }
        if( mark >= 0.0 ) {
            result.tip = std::max( mark, reach( blade_gap ) );
            break;
        }
    }
    result.full = reach( size.head + blade_gap );
    auto clear { reach( size.head + blade_clearance ) };
    auto const foot { reach( size.high + blade_gap ) };
    auto limit { length > shortest + 1e-3 ? length : 0.9 * length };
    // the blades of a double slip with them outside of the diamond end before the check rails of its crossings, shorter than they'd bend;
    // with the throat of an acute frog of the diamond nearer than the blade gets clear of the stock rail, it ends short of the throat with the head clear of it,
    // the frog over the blade wouldn't be made at all
    auto capped { false };
    if( DoubleSlip() && clear >= 0.0 ) {
        auto const [ reach, latest ] { slip_blade_reach( *this ) };
        auto const cap { reach >= clear ? reach : latest };
        if( cap < limit && ( cap >= clear || ( result.full >= 0.0 && cap > result.full ) ) ) {
            limit = cap;
            capped = true;
            clear = std::min( clear, cap );
        }
    }
    // the blade of a switch with a frog of another track right past the heel keeps as long as it may, it doesn't bend as far as it would
    auto reached { false };
    if( false == DoubleSlip() && clear >= 0.0 ) {
        auto const cap { blade_reach( *this, Path, side ) };
        if( cap >= clear && cap < limit ) {
            limit = cap;
            capped = true;
            reached = true;
        }
    }
    if( result.tip < 0.0 || result.full < 0.0 || clear < 0.0 || clear > limit ) {
        return result;
    }
    auto const bend = [&]( double const Distance, double const Heel ) {
        auto const u { std::clamp( ( Heel - Distance ) / std::max( Heel - result.tip, 1e-6 ), 0.0, 1.0 ) };
        return ( result.rigid ? u : u * u * ( 3.0 - u ) * 0.5 ); };
    auto const opening = [&]( double const Heel, double &At ) {
        auto least { 1e9 };
        for( auto const &sample : gauge ) {
            if( sample.x < result.tip || sample.x > Heel ) {
                continue;
            }
            auto const gap { std::max( 0.0, sample.y - size.head ) + blades_throw() * bend( sample.x, Heel ) };
            if( gap < least ) {
                least = gap;
                At = sample.x;
            }
        }
        return least; };
    auto at { 0.0 };
    auto short_blade { false };
    auto const given { SwitchExtension->m_bladeheel };
    if( given > 0.0 ) {
        // the template of the switch says where its blades end; they bend stiff about the heel where the flangeway is too narrow for them to flex
        result.heel = std::clamp( given, result.tip + blade_spacing, limit );
        result.rigid = ( opening( result.heel, at ) < blade_clearance );
    }
    else {
        if( opening( limit, at ) < blade_clearance ) {
            result.rigid = true;
            if( opening( limit, at ) < blade_clearance || reached ) {
                if( false == capped ) {
                    return result;
                }
                short_blade = true;
            }
        }
        auto low { clear };
        auto high { limit };
        for( int step = 0; step < 40 && false == short_blade; ++step ) {
            auto const middle { 0.5 * ( low + high ) };
            ( opening( middle, at ) >= blade_clearance ? high : low ) = middle;
        }
        result.heel = high;
        // the blade goes on until the rail behind it stands clear of the stock rail with its whole foot, so it moves out of the place of the rail
        // of the other path all along; only the frog of another track or the crossings of a slip end it sooner
        auto const apart { reach( size.high - size.low + blade_gap ) };
        if( apart > result.heel && result.heel < limit ) {
            result.heel = std::min( apart, limit );
            result.rigid = false;
            result.rigid = ( opening( result.heel, at ) < blade_clearance );
        }
    }
    result.opening = opening( result.heel, result.openingat );
    for( std::size_t i = 1; i < gauge.size() && gauge[ i ].x <= result.heel; ++i ) {
        if( gauge[ i ].y < gauge[ i - 1 ].y - 1e-4 ) {
            return result;
        }
    }
    auto const &profile { track_rail_profile( m_profile1.second ) };
    auto const half { profile.size() / 2 };
    auto bladetop { -1e9 };
    for( auto i { half }; i < profile.size(); ++i ) {
        bladetop = std::max( bladetop, static_cast<double>( profile[ i ].position.y ) );
    }
    result.drop = ( half > 0 ? std::max( 0.0, size.crest - bladetop ) : 0.0 );
    auto const lowered { reach( blade_lowered + blade_gap ) };
    auto const cutto { std::min( result.heel, foot >= 0.0 ? foot + blade_spacing : result.heel ) };
    for( auto distance { result.tip }; distance < cutto + 0.5 * blade_spacing; distance += blade_spacing ) {
        auto const at { std::min( distance, cutto ) };
        auto const width { level( at ) - blade_gap };
        result.extents.emplace_back( at, width );
        if( result.drop > 0.0 && ( lowered < 0.0 || at <= lowered + blade_spacing ) ) {
            result.drops.emplace_back( at, result.drop * std::clamp( 1.0 - width / blade_lowered, 0.0, 1.0 ) );
        }
    }
    auto const steps { std::max( 1, static_cast<int>( std::ceil( ( result.heel - result.tip ) / blade_spacing ) ) ) };
    for( int i = 0; i <= steps; ++i ) {
        auto const distance { glm::mix( result.tip, result.heel, static_cast<double>( i ) / steps ) };
        result.bends.emplace_back( distance, bend( distance, result.heel ) );
    }
    result.valid = true;
    return result;
}
