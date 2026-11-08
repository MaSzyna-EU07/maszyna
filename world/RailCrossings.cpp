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
#include "utilities/Globals.h"
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
    path_sides<std::array<std::optional<gauge_line>, 2>> lines; // path, right side, from the blades
    std::optional<std::vector<track_rails::rail_crossing>> own;
    std::optional<std::vector<track_rails::rail_crossing>> joined;
    std::optional<track_rails::check_rail_set> checks;
    std::array<std::optional<track_rails::switch_blade>, 2> blades;
};
// the stamp what's kept of the rails of the tracks was kept with, and the one it holds with: forget() changes it
struct rails_stamps {
    unsigned int memostamp { 0 };
    unsigned int stamp { 1 };
};

// the stamps of what's kept of the rails; nothing to tear down at the end, as the tracks still forget as they go then
rails_stamps &kept_stamps() {

    static rails_stamps stamps;
    return stamps;
}

// what's kept of the rails of each track
std::unordered_map<TTrack const *, rails_memo> &kept_memos() {

    static std::unordered_map<TTrack const *, rails_memo> memos;
    return memos;
}

rails_memo &memo_of( TTrack const &Track ) {

    auto &memos { kept_memos() };
    if( auto &stamps { kept_stamps() }; stamps.memostamp != stamps.stamp ) {
        memos.clear();
        stamps.memostamp = stamps.stamp;
    }
    return memos[ &Track ];
}

} // namespace

namespace track_rails {

void forget() {

    ++kept_stamps().stamp;
}

unsigned int memo_stamp() {

    return kept_stamps().stamp;
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

// the rails of the track are cut where they cross other rails: plain tracks, and switches whose frog is made from the rails;
// frogs in eu07.ini turns it on
bool cut_rails( TTrack const &Track ) {

    return Global.Frogs
        && ( Track.iCategoryFlag & 15 ) == 1
        && Track.m_material1 != null_handle
        && false == Track.m_editorremoved
        && ( Track.eType == tt_Normal ? Track.Segment != nullptr :
             Track.eType == tt_Switch && Track.SwitchExtension && Track.SwitchExtension->m_frogs && Track.SwitchExtension->Segments[ 0 ] && Track.SwitchExtension->Segments[ 1 ] );
}

// a track with rails laid, cut or whole
bool railed( TTrack const &Track ) {

    return ( Track.iCategoryFlag & 15 ) == 1
        && Track.m_material1 != null_handle
        && false == Track.m_editorremoved
        && ( Track.eType == tt_Normal ? Track.Segment != nullptr :
             Track.eType == tt_Switch && Track.SwitchExtension && Track.SwitchExtension->Segments[ 0 ] && Track.SwitchExtension->Segments[ 1 ] );
}

bool shaped_blades( TTrack const &Track ) {

    if( cut_rails( Track ) ) {
        return true;
    }
    return track_features::blades
        && ( Track.iCategoryFlag & 15 ) == 1
        && Track.m_material1 != null_handle
        && false == Track.m_editorremoved
        && Track.eType == tt_Switch && Track.SwitchExtension && Track.SwitchExtension->m_frogs && Track.SwitchExtension->Segments[ 0 ] && Track.SwitchExtension->Segments[ 1 ];
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
    if( auto const *after { next->trNext };
        after == nullptr || after->eType != tt_Switch || after->SwitchExtension == nullptr
     || glm::distance( after->SwitchExtension->Segments[ 0 ]->FastGetPoint_0(), next->Segment->FastGetPoint_1() ) > 0.01 ) {
        return nullptr;
    }
    return next;
}

double blade_runon( TTrack const &Track, int const Side ) {

    auto const *owner { Track.trPrev };
    if( Track.eType != tt_Normal || Track.Segment == nullptr || owner == nullptr || owner->eType != tt_Switch || false == shaped_blades( *owner ) ) {
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
    return std::ranges::any_of( Wingends, [ &Check, &farend ]( glm::dvec2 const &Wing ) { return replaced_check( { Check }, Wing.x, Wing.y, farend ); } );
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
    glm::dvec3 point;
    glm::dvec3 tangent;
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

// a rail of a path of a track, looked at from a distance along it up to its length; const for the track whose crossings are looked for,
// as the crossings found name only the track they cross
template <typename Track_>
struct rail_run {
    Track_ *track { nullptr };
    int path { 0 };
    int side { 0 };
    gauge_line const *line { nullptr };
    double from { 0.0 };
    double length { 0.0 };
};
using our_run = rail_run<TTrack const>;
using their_run = rail_run<TTrack>;

// the tracks with rails of a cell around the track, each added once: the ones whose rails are cut, or with Whole all of them
void add_cell_rails( TTrack const &Track, scene::basic_cell const &Cell, bool const Whole, std::vector<TTrack *> &Others ) {

    for( auto *other : Cell.m_directories.paths ) {
        if( other == nullptr || other == &Track || false == ( Whole ? railed( *other ) : cut_rails( *other ) ) ) {
            continue;
        }
        if( glm::length( other->location() - Track.location() ) > other->radius() + Track.radius() + 1.0 ) {
            continue;
        }
        if( std::ranges::find( Others, other ) == Others.end() ) {
            Others.push_back( other );
        }
    }
}

// the tracks with rails around the track: the ones whose rails are cut, or with Whole all of them
std::vector<TTrack *> nearby_rails( TTrack const &Track, bool const Whole = false ) {

    std::vector<TTrack *> others;
    // a list of its own, not the one the region shares
    std::vector<scene::basic_section *> sections;
    simulation::Region->sections( Track.location(), static_cast<float>( Track.radius() + 50.0 ), sections );
    for( auto const *section : sections ) {
        for( auto const &cell : section->m_cells ) {
            if( glm::length( cell.area().center - Track.location() ) > cell.area().radius + Track.radius() + 1.0 ) {
                continue;
            }
            add_cell_rails( Track, cell, Whole, others );
        }
    }
    return others;
}

double line_bend( gauge_line const &Line, double const At ) {

    glm::dvec3 point;
    glm::dvec3 before;
    glm::dvec3 after;
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

    glm::dvec3 p;
    glm::dvec3 tp;
    glm::dvec3 q;
    glm::dvec3 tq;
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

rail_crossing make_crossing( our_run const &Ours, their_run const &Theirs, double const S, double const T, glm::dvec2 const &Tangent, glm::dvec2 const &Othertangent, double const Angle ) {

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

// a piece of the gauge line of our rail between two of its points, in plan, and the box around it a little wider
struct gauge_piece {
    std::size_t index { 0 }; // of the point the piece ends at
    glm::dvec2 start { 0.0 };
    glm::dvec2 along { 0.0 };
    glm::dvec2 low { 0.0 };
    glm::dvec2 high { 0.0 };
};

// where a piece of our rail crosses the piece of their rail ending at their point J, added unless a crossing is found there already
void meet_pieces( our_run const &Ours, their_run const &Theirs, gauge_piece const &Piece, std::size_t const J, std::vector<rail_crossing> &Result ) {

    auto const &line { *Ours.line };
    auto const &theirs { *Theirs.line };
    auto const b0 { plan( theirs.points[ J - 1 ] ) };
    auto const b { plan( theirs.points[ J ] ) - b0 };
    if( auto const reach { 0.02 * glm::length( b ) };
        Piece.low.x > std::max( b0.x, b0.x + b.x ) + reach || std::min( b0.x, b0.x + b.x ) - reach > Piece.high.x
     || Piece.low.y > std::max( b0.y, b0.y + b.y ) + reach || std::min( b0.y, b0.y + b.y ) - reach > Piece.high.y ) {
        return;
    }
    double u;
    double v;
    if( false == meet( Piece.start, Piece.along, b0, b, 1e-12, u, v ) ) {
        return;
    }
    if( u < -0.01 || u > 1.01 || v < -0.01 || v > 1.01 ) {
        return;
    }
    auto s { glm::mix( line.distances[ Piece.index - 1 ], line.distances[ Piece.index ], u ) };
    auto t { glm::mix( theirs.distances[ J - 1 ], theirs.distances[ J ], v ) };
    refine_meeting( line, theirs, s, t );
    glm::dvec3 p;
    glm::dvec3 tp;
    glm::dvec3 q;
    glm::dvec3 tq;
    line.at( s, p, tp );
    theirs.at( t, q, tq );
    if( glm::length( plan( p ) - plan( q ) ) > 0.01
     || s < 0.0 || s > Ours.length
     || t < Theirs.from || t > Theirs.length ) {
        return;
    }
    if( std::abs( p.y - q.y ) > 0.5 ) {
        return;
    }
    if( ( s < 0.5 || s > Ours.length - 0.5 ) && ( t < 0.5 || t > Theirs.length - 0.5 ) && joined( *Ours.track, *Theirs.track ) ) {
        return;
    }
    auto const tangent { glm::normalize( plan( tp ) ) };
    auto const othertangent { glm::normalize( plan( tq ) ) };
    auto const angle { std::acos( std::min( 1.0, std::abs( glm::dot( tangent, othertangent ) ) ) ) };
    if( angle < 0.03 ) {
        return;
    }
    if( std::ranges::any_of( Result, [ &Ours, s ]( rail_crossing const &Crossing ) { return same_place( Crossing, Ours.path, Ours.side, s ); } ) ) {
        return;
    }
    Result.push_back( make_crossing( Ours, Theirs, s, t, tangent, othertangent, angle ) );
}

void meet_rails( our_run const &Ours, their_run const &Theirs, std::vector<rail_crossing> &Result ) {

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
        gauge_piece const piece { i, a0, a, alow, ahigh };
        for( std::size_t j = 1; j < theirs.points.size(); ++j ) {
            meet_pieces( Ours, Theirs, piece, j, Result );
        }
    }
}

// the crossings of the rails of the track with a rail of another one
void meet_track( TTrack const &Track, their_run const &Theirs, std::vector<rail_crossing> &Result ) {

    for( int path = 0; path < path_count( Track ); ++path ) {
        auto const length { path_segment( Track, path ).GetLength() };
        for( int side = 0; side < 2; ++side ) {
            meet_rails( { &Track, path, side ? 1 : -1, &rail_line( Track, path, side, false ), 0.0, length }, Theirs, Result );
        }
    }
}

std::vector<rail_crossing> own_crossings( TTrack const &Track ) {

    std::vector<rail_crossing> result;
    for( auto *other : nearby_rails( Track ) ) {
        for( int otherpath = 0; otherpath < path_count( *other ); ++otherpath ) {
            their_run theirs { other, otherpath };
            theirs.from = ( other->eType == tt_Switch ? blade_length( *other, otherpath ) : 0.0 );
            theirs.length = path_segment( *other, otherpath ).GetLength();
            for( int otherside = 0; otherside < 2; ++otherside ) {
                theirs.side = ( otherside ? 1 : -1 );
                theirs.line = &rail_line( *other, otherpath, otherside, true );
                meet_track( Track, theirs, result );
            }
        }
    }
    return result;
}

// how far the blades of a part of a double slip may reach: up to the check rails of the crossings of the rails of the diamond, found
// with the blades of the other parts left out of the account, as they're yet to be made as short; at the latest short of their throats,
// the check rails then cut to begin past the heels
// crossings of the rails of the track with the rails of the tracks around, with the blades of switches left out of the account
// Whole: the rails of the tracks around taken whole too, where they aren't cut
std::vector<rail_crossing> unbladed_crossings( TTrack const &Track, bool const Whole = false ) {

    std::vector<rail_crossing> found;
    for( auto *other : nearby_rails( Track, Whole ) ) {
        for( int otherpath = 0; otherpath < path_count( *other ); ++otherpath ) {
            their_run theirs { other, otherpath };
            theirs.length = path_segment( *other, otherpath ).GetLength();
            for( int otherside = 0; otherside < 2; ++otherside ) {
                theirs.side = ( otherside ? 1 : -1 );
                theirs.line = &rail_line( *other, otherpath, otherside, false );
                meet_track( Track, theirs, found );
            }
        }
    }
    return found;
}

std::pair<double, double> slip_blade_reach( TTrack const &Track ) {

    auto reach { 1e9 };
    auto latest { 1e9 };
    // the crossings of the diamond end the blades whether its rails are cut there or laid whole
    for( auto const &crossing : unbladed_crossings( Track, true ) ) {
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

// a path of a track going on from an end of a path of the track, joined to it there
struct joined_path {
    TTrack const *track { nullptr }; // the track the crossings of the neighbour are mapped onto
    TTrack *neighbour { nullptr };
    int path { 0 }; // of the track
    int end { 0 }; // of the path of the track the neighbour goes on from
    double length { 0.0 }; // of the path of the track
    int neighbourpath { 0 };
    double neighbourlength { 0.0 };
    bool atstart { false }; // the path of the neighbour starts at the joint
    bool same { false }; // the path of the neighbour runs on the same way as the path of the track
};

// the crossings of a joined path near enough to the joint, mapped onto the path of the track, where it has none of its own
void add_joined_crossings( joined_path const &Joined, std::vector<rail_crossing> const &Theirs, std::size_t const Owncount, std::vector<rail_crossing> &Result ) {

    auto const extension { 10.0 };
    for( auto const &crossing : Theirs ) {
        if( crossing.path != Joined.neighbourpath || crossing.track == Joined.track ) {
            continue;
        }
        auto const along { Joined.atstart ? crossing.at : Joined.neighbourlength - crossing.at };
        if( along > extension ) {
            continue;
        }
        auto mapped { crossing };
        mapped.path = Joined.path;
        mapped.at = ( Joined.end ? Joined.length + along : -along );
        mapped.owner = Joined.neighbour;
        if( false == Joined.same ) {
            mapped.side = -crossing.side;
            mapped.into = -crossing.into;
            mapped.aligned = !crossing.aligned;
        }
        if( std::none_of( Result.begin(), Result.begin() + Owncount, [ &mapped ]( rail_crossing const &Crossing ) { return same_place( Crossing, mapped.path, mapped.side, mapped.at ); } ) ) {
            Result.push_back( mapped );
        }
    }
}

// the crossings of the track going on from an end of a path of the track, near enough for their check rails and wings to reach this one
void add_end_crossings( TTrack const &Track, int const Path, int const End, double const Length, std::size_t const Owncount, std::vector<rail_crossing> &Result ) {

    auto const &segment { path_segment( Track, Path ) };
    auto *neighbour { path_neighbour( Track, Path, End ) };
    if( neighbour == nullptr || neighbour == &Track || false == cut_rails( *neighbour ) ) {
        return;
    }
    auto const joint { End ? segment.FastGetPoint_1() : segment.FastGetPoint_0() };
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
        joined_path const joinedpath {
            .track = &Track,
            .neighbour = neighbour,
            .path = Path,
            .end = End,
            .length = Length,
            .neighbourpath = neighbourpath,
            .neighbourlength = neighbourlength,
            .atstart = atstart,
            .same = ( atstart == ( End == 1 ) ) };
        add_joined_crossings( joinedpath, theirs, Owncount, Result );
    }
}

// crossings of the tracks going on from the ends, near enough for their check rails and wings to reach this one
void add_neighbour_crossings( TTrack const &Track, std::vector<rail_crossing> &Result ) {

    auto const owncount { Result.size() };
    for( int path = 0; path < path_count( Track ); ++path ) {
        auto const length { path_segment( Track, path ).GetLength() };
        for( int end = 0; end < 2; ++end ) {
            add_end_crossings( Track, path, end, length, owncount, Result );
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
    Checks.emplace_back( At + Toward * 0.25 - half, At + Toward * 0.25 + half, true, true, 1.5, Flare, 0.0, 0.0, true );
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
            plans[ crossing.path ][ crossing.side > 0 ].emplace_back( crossing.at, crossing.at + 1.2, false, true, 0.5, 0.02, -knuckle_bend( crossing.into ), 0.0 );
        }
        else {
            plans[ crossing.path ][ crossing.side > 0 ].emplace_back( crossing.at - 1.2, crossing.at, true, false, 0.5, 0.02, 0.0, knuckle_bend( crossing.into ) );
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

    std::ranges::sort( Checks, []( planned_check const &A, planned_check const &B ) { return A.from < B.from; } );
    std::size_t i { 1 };
    while( i < Checks.size() ) {
        if( Checks[ i ].from < Checks[ i - 1 ].to + 2.0 ) {
            if( Checks[ i ].to > Checks[ i - 1 ].to ) {
                Checks[ i - 1 ].to = Checks[ i ].to;
                Checks[ i - 1 ].flareto = Checks[ i ].flareto;
                Checks[ i - 1 ].tobend = Checks[ i ].tobend;
            }
            Checks[ i - 1 ].shortest = std::min( Checks[ i - 1 ].shortest, Checks[ i ].shortest );
            Checks[ i - 1 ].flare = std::max( Checks[ i - 1 ].flare, Checks[ i ].flare );
            Checks[ i - 1 ].acute = Checks[ i - 1 ].acute || Checks[ i ].acute;
            // the merged one is gone, the next one comes in its place
            Checks.erase( Checks.begin() + static_cast<std::ptrdiff_t>( i ) );
        }
        else {
            ++i;
        }
    }
}

bool blocked_by( std::vector<gauge_line> const &Others, glm::dvec2 const &Gauge, glm::dvec2 const &Inward, double const Clearance ) {

    auto blocked { false };
    for( auto const &other : Others ) {
        for( std::size_t i = 1; i < other.points.size() && false == blocked; ++i ) {
            auto const a { plan( other.points[ i - 1 ] ) };
            auto const edge { plan( other.points[ i ] ) - a };
            double t;
            double u;
            if( false == meet( Gauge, Inward, a, edge, 1e-9, t, u ) ) {
                continue;
            }
            blocked = ( u > -1e-6 && u < 1.0 + 1e-6 && t > -0.05 && t < Clearance );
        }
    }
    return blocked;
}

// how far the heels of the blades of a switch reach, the furthest of them
double heels_reach( TTrack const &Track ) {

    auto heel { 0.0 };
    for( int path = 0; path < 2; ++path ) {
        auto const &blade { Track.blade( path ) };
        if( blade.valid ) {
            heel = std::max( heel, blade.heel );
        }
    }
    return heel;
}

// the places along a check rail, every 0.1, where a rail inside it comes nearer than the Clearance: one of the Others, or with an acute check one of the Crossing ones
void block_check( gauge_line const &Ours, int const Side, planned_check const &Check, std::vector<gauge_line> const &Others, std::vector<gauge_line> const &Crossing, double const Clearance, knots::line &Blocks ) {

    auto s { Check.from };
    while( s <= Check.to ) {
        glm::dvec3 point;
        glm::dvec3 tangent;
        Ours.at( s, point, tangent );
        if( auto const inward { plan( Ours.normal( s, Side ) ) };
            blocked_by( Others, plan( point ), inward, Clearance )
         || ( Check.acute && blocked_by( Crossing, plan( point ), inward, Clearance ) ) ) {
            Blocks.emplace_back( s - 0.3, s + 0.3 );
        }
        s += 0.1;
    }
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
        blocks.emplace_back( -1e9, heels_reach( Track ) + 0.3 );
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
            block_check( ours, Side, check, others, Crossing, Clearance, blocks );
        }
    }
    knots::sort( blocks );
    return blocks;
}

void cut_check( planned_check const &Check, knots::line const &Blocks, std::vector<check_rail> &Result ) {

    auto const add = [ &Check, &Result ]( double const From, double const To ) {
        Result.emplace_back(
            From, To,
            ( From > Check.from || Check.flarefrom ) ? Check.flare : 0.0,
            ( To < Check.to || Check.flareto ) ? Check.flare : 0.0,
            From > Check.from ? 0.0 : Check.frombend,
            To < Check.to ? 0.0 : Check.tobend ); };
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

// where the gauge lines of the crossing rails of the two paths of a switch meet past the blades, from their points in plan and the distances
// along the paths: true where they do, with the distances of the meeting put in the frog
bool meet_frog_lines( std::array<std::vector<glm::dvec2>, 2> const &Lines, std::array<std::vector<double>, 2> const &Distances, switch_frog &Frog ) {

    for( std::size_t i = 1; i < Lines[ 0 ].size(); ++i ) {
        if( Distances[ 0 ][ i ] <= Frog.blades[ 0 ] ) {
            continue;
        }
        auto const a0 { Lines[ 0 ][ i - 1 ] };
        auto const a { Lines[ 0 ][ i ] - a0 };
        for( std::size_t j = 1; j < Lines[ 1 ].size(); ++j ) {
            if( Distances[ 1 ][ j ] <= Frog.blades[ 1 ] ) {
                continue;
            }
            auto const b0 { Lines[ 1 ][ j - 1 ] };
            auto const b { Lines[ 1 ][ j ] - b0 };
            double u;
            double v;
            if( false == meet( a0, a, b0, b, 1e-12, u, v ) ) {
                continue;
            }
            if( u < 0.0 || u > 1.0 || v < 0.0 || v > 1.0 ) {
                continue;
            }
            Frog.at[ 0 ] = glm::mix( Distances[ 0 ][ i - 1 ], Distances[ 0 ][ i ], u );
            Frog.at[ 1 ] = glm::mix( Distances[ 1 ][ j - 1 ], Distances[ 1 ][ j ], v );
            return true;
        }
    }
    return false;
}

// how far along the blade the gap to the rail of the other path gets as wide as the Width; -1 where it doesn't
double gauge_reach( std::vector<glm::dvec2> const &Gauge, double const Width ) {

    for( std::size_t i = 1; i < Gauge.size(); ++i ) {
        if( Gauge[ i ].y >= Width ) {
            auto const &a { Gauge[ i - 1 ] };
            auto const &b { Gauge[ i ] };
            return glm::mix( a.x, b.x, std::clamp( ( Width - a.y ) / std::max( b.y - a.y, 1e-9 ), 0.0, 1.0 ) );
        }
    }
    return -1.0;
}

// how wide the gap to the rail of the other path is at a distance along the blade
double gauge_level( std::vector<glm::dvec2> const &Gauge, double const Distance ) {

    for( std::size_t i = 1; i < Gauge.size(); ++i ) {
        if( Gauge[ i ].x >= Distance ) {
            auto const &a { Gauge[ i - 1 ] };
            auto const &b { Gauge[ i ] };
            return glm::mix( a.y, b.y, std::clamp( ( Distance - a.x ) / std::max( b.x - a.x, 1e-9 ), 0.0, 1.0 ) );
        }
    }
    return Gauge.back().y;
}

// how far the rail of the other path turns away from the rail of the blade at a distance along it: the sine of the angle between them in plan
double gauge_turn( gauge_line const &Ours, gauge_line const &Theirs, int const Side, double const Distance ) {

    glm::dvec3 here;
    glm::dvec3 ourtangent;
    glm::dvec3 line;
    glm::dvec3 along;
    Ours.at( Distance, here, ourtangent );
    auto there { nearest_distance( Theirs, here ) };
    lateral( Theirs, Side, here, there );
    Theirs.at( there, line, along );
    return std::abs( glm::cross( glm::normalize( glm::dvec3 { ourtangent.x, 0.0, ourtangent.z } ), glm::normalize( glm::dvec3 { along.x, 0.0, along.z } ) ).y );
}

// the gap between the gauge line of the rail of the blade and the one of the rail of the other path, sampled along the Length from the tip,
// in order along the blade
void sample_gauge( gauge_line const &Ours, gauge_line const &Theirs, int const Side, double const Length, std::vector<glm::dvec2> &Gauge ) {

    auto const count { std::max( 2, static_cast<int>( std::ceil( Length / 0.1 ) ) ) };
    glm::dvec3 point;
    glm::dvec3 tangent;
    Ours.at( 0.0, point, tangent );
    auto otherat { nearest_distance( Theirs, point ) };
    for( int i = 0; i <= count; ++i ) {
        auto const distance { Length * i / count };
        Ours.at( distance, point, tangent );
        lateral( Theirs, Side, point, otherat );
        glm::dvec3 line;
        Theirs.at( otherat, line, tangent );
        auto at { distance };
        auto const gap { lateral( Ours, Side, line, at ) };
        Gauge.emplace_back( at, gap );
    }
    std::ranges::sort( Gauge, []( glm::dvec2 const &A, glm::dvec2 const &B ) { return A.x < B.x; } );
}

// the blade meets the stock rail at an angle too steep for its point to be planed along, from where it's Thin to where it's as Thick as the nose
bool angled_blade( gauge_line const &Ours, gauge_line const &Theirs, int const Side, double const Thin, double const Thick ) {

    auto angled { false };
    if( Thin >= 0.0 && Thick > Thin ) {
        auto const at { gauge_turn( Ours, Theirs, Side, Thin ) };
        auto const bending { ( gauge_turn( Ours, Theirs, Side, Thick ) - at ) / ( Thick - Thin ) };
        angled = std::sqrt( std::max( 0.0, at * at - 2.0 * bending * blade_point ) ) >= blade_tangent;
    }
    return angled;
}

// the last joint of the pieces of a path, short of where the blade stands apart by the mark, at which it stands apart enough for its tip, or at all
// with an Angled blade; -1 where there's none
double tip_mark( TSegment const &Segment, std::vector<glm::dvec2> const &Gauge, bool const Angled ) {

    auto mark { -1.0 };
    for( std::size_t piece = 1; piece < Segment.PieceCount(); ++piece ) {
        auto const at { Segment.PieceStart( piece ) };
        auto const apart { gauge_level( Gauge, at ) };
        if( apart > blade_mark ) {
            break;
        }
        if( Angled || apart >= blade_gap ) {
            mark = at;
        }
    }
    return mark;
}

// how far the heel of a blade may go: from where the blade stands clear of the stock rail up to the limit
struct heel_bounds {
    double clear { 0.0 };
    double limit { 0.0 };
    bool capped { false }; // the limit is cut short: the crossings of a slip or the frog of another track end the blade
    bool reached { false }; // the frog of another track ends the blade
};

// the bounds of the heel of the blade of a path, cut short by the crossings past it
void cap_heel( TTrack const &Track, int const Path, int const Side, double const Full, heel_bounds &Bounds ) {

    // the blades of a double slip with them outside of the diamond end before the check rails of its crossings, shorter than they'd bend;
    // with the throat of an acute frog of the diamond nearer than the blade gets clear of the stock rail, it ends short of the throat with the head clear of it,
    // the frog over the blade wouldn't be made at all
    if( Track.DoubleSlip() && Bounds.clear >= 0.0 ) {
        auto const [ slipreach, sliplatest ] { slip_blade_reach( Track ) };
        auto const cap { slipreach >= Bounds.clear ? slipreach : sliplatest };
        if( cap < Bounds.limit && ( cap >= Bounds.clear || ( Full >= 0.0 && cap > Full ) ) ) {
            Bounds.limit = cap;
            Bounds.capped = true;
            Bounds.clear = std::min( Bounds.clear, cap );
        }
    }
    // the blade of a switch with a frog of another track right past the heel keeps as long as it may, it doesn't bend as far as it would
    if( false == Track.DoubleSlip() && Bounds.clear >= 0.0 ) {
        auto const cap { blade_reach( Track, Path, Side ) };
        if( cap >= Bounds.clear && cap < Bounds.limit ) {
            Bounds.limit = cap;
            Bounds.capped = true;
            Bounds.reached = true;
        }
    }
}

// how far over the blade bends at a distance along it: nothing at the Heel, all the way at the tip
double blade_bend( switch_blade const &Blade, double const Distance, double const Heel ) {

    auto const u { std::clamp( ( Heel - Distance ) / std::max( Heel - Blade.tip, 1e-6 ), 0.0, 1.0 ) };
    return ( Blade.rigid ? u : u * u * ( 3.0 - u ) * 0.5 );
}

// the narrowest gap the blade thrown over by the Throw leaves to the stock rail with its Head, from the tip to the Heel, and where it is
double blade_opening( switch_blade const &Blade, double const Head, double const Throw, double const Heel, double &At ) {

    auto least { 1e9 };
    for( auto const &sample : Blade.gauge ) {
        if( sample.x < Blade.tip || sample.x > Heel ) {
            continue;
        }
        auto const gap { std::max( 0.0, sample.y - Head ) + Throw * blade_bend( Blade, sample.x, Heel ) };
        if( gap < least ) {
            least = gap;
            At = sample.x;
        }
    }
    return least;
}

// the heel of a blade the template of the switch doesn't set: as near the tip as the blade thrown over still opens clear of the stock rail, then
// on to where the rail behind it stands clear of it; false where the blade can't be made
bool find_heel( switch_blade &Blade, heel_bounds const &Bounds, rail_size const &Size, double const Throw ) {

    auto at { 0.0 };
    auto short_blade { false };
    if( blade_opening( Blade, Size.head, Throw, Bounds.limit, at ) < blade_clearance ) {
        Blade.rigid = true;
        if( blade_opening( Blade, Size.head, Throw, Bounds.limit, at ) < blade_clearance || Bounds.reached ) {
            if( false == Bounds.capped ) {
                return false;
            }
            short_blade = true;
        }
    }
    auto low { Bounds.clear };
    auto high { Bounds.limit };
    for( int step = 0; step < 40 && false == short_blade; ++step ) {
        auto const middle { 0.5 * ( low + high ) };
        ( blade_opening( Blade, Size.head, Throw, middle, at ) >= blade_clearance ? high : low ) = middle;
    }
    Blade.heel = high;
    // the blade goes on until the rail behind it stands clear of the stock rail with its whole foot, so it moves out of the place of the rail
    // of the other path all along; only the frog of another track or the crossings of a slip end it sooner
    if( auto const apart { gauge_reach( Blade.gauge, Size.high - Size.low + blade_gap ) };
        apart > Blade.heel && Blade.heel < Bounds.limit ) {
        Blade.heel = std::min( apart, Bounds.limit );
        Blade.rigid = false;
        Blade.rigid = ( blade_opening( Blade, Size.head, Throw, Blade.heel, at ) < blade_clearance );
    }
    return true;
}

// the gap to the rail of the other path doesn't narrow anywhere from the start of the blade to its heel
bool opens_steadily( std::vector<glm::dvec2> const &Gauge, double const Heel ) {

    for( std::size_t i = 1; i < Gauge.size() && Gauge[ i ].x <= Heel; ++i ) {
        if( Gauge[ i ].y < Gauge[ i - 1 ].y - 1e-4 ) {
            return false;
        }
    }
    return true;
}

// how high the top of the blade is: the highest of the second half of the points of the rail profile, those of the blade
double blade_top( gfx::vertex_array const &Profile ) {

    auto bladetop { -1e9 };
    for( auto i { Profile.size() / 2 }; i < Profile.size(); ++i ) {
        bladetop = std::max( bladetop, static_cast<double>( Profile[ i ].position.y ) );
    }
    return bladetop;
}

// how wide the blade is from the tip to where it's cut to, every blade spacing, and how far it's dropped below the stock rail where it's narrow
void blade_extents( switch_blade &Blade, double const Cutto, double const Lowered ) {

    auto distance { Blade.tip };
    while( distance < Cutto + 0.5 * blade_spacing ) {
        auto const at { std::min( distance, Cutto ) };
        auto const width { gauge_level( Blade.gauge, at ) - blade_gap };
        Blade.extents.emplace_back( at, width );
        if( Blade.drop > 0.0 && ( Lowered < 0.0 || at <= Lowered + blade_spacing ) ) {
            Blade.drops.emplace_back( at, Blade.drop * std::clamp( 1.0 - width / blade_lowered, 0.0, 1.0 ) );
        }
        distance += blade_spacing;
    }
}

// how far over the blade bends along it, from the tip to the heel
void blade_bends( switch_blade &Blade ) {

    auto const steps { std::max( 1, static_cast<int>( std::ceil( ( Blade.heel - Blade.tip ) / blade_spacing ) ) ) };
    for( int i = 0; i <= steps; ++i ) {
        auto const distance { glm::mix( Blade.tip, Blade.heel, static_cast<double>( i ) / steps ) };
        Blade.bends.emplace_back( distance, blade_bend( Blade, distance, Blade.heel ) );
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

track_rails::check_rail_set TTrack::check_rails( std::vector<rail_crossing> const &Crossings ) const {

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
        if( entry.track == nullptr || entry.track == this || std::ranges::find( crossers, entry.track ) != crossers.end() ) {
            continue;
        }
        crossers.push_back( entry.track );
        for( int otherpath = 0; otherpath < track_rails::path_count( *entry.track ); ++otherpath ) {
            for( auto const side : { -1.0, 1.0 } ) {
                crossing.push_back( make_gauge_line( track_rails::path_segment( *entry.track, otherpath ), side * 0.5 * std::abs( entry.track->fTrackWidth ), 0.0 ) );
            }
        }
    }
    for( int path = 0; path < track_rails::path_count( *this ); ++path ) {
        for( int rightside = 0; rightside < 2; ++rightside ) {
            auto &checks { plans[ path ][ rightside ] };
            if( checks.empty() ) {
                continue;
            }
            merge_checks( checks );
            auto const blocks { check_blocks( *this, path, rightside ? 1 : -1, checks, clearance, crossing ) };
            for( auto const &check : checks ) {
                cut_check( check, blocks, result[ path ][ rightside ] );
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
    std::array<std::vector<glm::dvec2>, 2> lines;
    std::array<std::vector<double>, 2> distances;
    for( int path = 0; path < 2; ++path ) {
        auto const &segment { *SwitchExtension->Segments[ path ] };
        auto const length { segment.GetLength() };
        result.blades[ path ] = blade_length( *this, path );
        auto const count { std::max( 1, static_cast<int>( std::ceil( length / 0.25 ) ) ) };
        for( int i = 0; i <= count; ++i ) {
            auto const distance { length * i / count };
            glm::dvec3 point;
            glm::dvec3 across;
            segment.plan_frame( distance, point, across );
            auto const gauge { point + across * ( result.side[ path ] * halfwidth ) };
            lines[ path ].emplace_back( gauge.x, gauge.z );
            distances[ path ].push_back( distance );
        }
    }
    result.valid = meet_frog_lines( lines, distances, result );
    if( false == result.valid ) {
        return result;
    }
    glm::dvec3 point;
    glm::dvec3 across0;
    glm::dvec3 across1;
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
    if( eType != tt_Switch || false == shaped_blades( *this ) || m_profiles.empty() ) {
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
    auto const &gauge { result.gauge };
    sample_gauge( ours, theirs, side, length, result.gauge );
    if( std::abs( gauge.front().y ) > 0.002 ) {
        return result;
    }
    auto const thin { gauge_reach( gauge, blade_point ) };
    auto const thick { gauge_reach( gauge, blade_nose ) };
    auto const angled { angled_blade( ours, theirs, side, thin, thick ) };
    result.tip = ( angled || thick < 0.0 ? thin : std::max( thin, thick - blade_planed ) );
    for( auto const path : { Path, 1 - Path } ) {
        if( auto const mark { tip_mark( *SwitchExtension->Segments[ path ], gauge, angled ) }; mark >= 0.0 ) {
            result.tip = std::max( mark, gauge_reach( gauge, blade_gap ) );
            break;
        }
    }
    result.full = gauge_reach( gauge, size.head + blade_gap );
    heel_bounds bounds;
    bounds.clear = gauge_reach( gauge, size.head + blade_clearance );
    auto const foot { gauge_reach( gauge, size.high + blade_gap ) };
    bounds.limit = ( length > shortest + 1e-3 ? length : 0.9 * length );
    cap_heel( *this, Path, side, result.full, bounds );
    if( result.tip < 0.0 || result.full < 0.0 || bounds.clear < 0.0 || bounds.clear > bounds.limit ) {
        return result;
    }
    auto const bladethrow { blades_throw() };
    if( auto const given { SwitchExtension->m_bladeheel }; given > 0.0 ) {
        // the template of the switch says where its blades end; they bend stiff about the heel where the flangeway is too narrow for them to flex
        result.heel = std::clamp( given, result.tip + blade_spacing, bounds.limit );
        auto at { 0.0 };
        result.rigid = ( blade_opening( result, size.head, bladethrow, result.heel, at ) < blade_clearance );
    }
    else if( false == find_heel( result, bounds, size, bladethrow ) ) {
        return result;
    }
    result.opening = blade_opening( result, size.head, bladethrow, result.heel, result.openingat );
    if( false == opens_steadily( gauge, result.heel ) ) {
        return result;
    }
    auto const &profile { track_rail_profile( m_profile1.second ) };
    auto const half { profile.size() / 2 };
    auto const bladetop { blade_top( profile ) };
    result.drop = ( half > 0 ? std::max( 0.0, size.crest - bladetop ) : 0.0 );
    auto const lowered { gauge_reach( gauge, blade_lowered + blade_gap ) };
    auto const cutto { std::min( result.heel, foot >= 0.0 ? foot + blade_spacing : result.heel ) };
    blade_extents( result, cutto, lowered );
    blade_bends( result );
    result.valid = true;
    return result;
}
