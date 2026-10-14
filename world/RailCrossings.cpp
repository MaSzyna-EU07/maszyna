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
#include "simulation/simulation.h"
#include "scene/scene.h"

using namespace track_rails;

namespace {

bool cut_profile( std::string Profile ) {

    std::transform( Profile.begin(), Profile.end(), Profile.begin(), []( unsigned char const Char ) { return static_cast<char>( std::tolower( Char ) ); } );
    return Profile.empty() || Profile == "default" || Profile == "s60" || Profile == "uic60" || Profile == "s39";
}

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


double wing_reach( double const Slope ) {

    return 0.15 / Slope + 0.3;
}

double check_length( double const Slope ) {

    return std::clamp( 0.5 / Slope, 3.6, 9.0 );
}

// the rails of the track are cut where they cross other rails: plain tracks, and switches whose frog is made from the rails
bool cut_rails( TTrack const &Track ) {

    return ( Track.iCategoryFlag & 15 ) == 1
        && Track.m_material1 != null_handle
        && false == Track.m_editorremoved
        && cut_profile( Track.m_profile1.first )
        && ( Track.eType == tt_Normal ? Track.Segment != nullptr :
             Track.eType == tt_Switch && Track.SwitchExtension && Track.SwitchExtension->m_frogs && Track.SwitchExtension->Segments[ 0 ] && Track.SwitchExtension->Segments[ 1 ] );
}

int blade_steps( TTrack const &Track ) {

    return static_cast<int>( std::ceil( Track.SwitchExtension->Segments[ 0 ]->RaSegCount() * 0.65 ) );
}

double blade_length( TTrack const &Track, int const Path ) {

    auto const &segment { *Track.SwitchExtension->Segments[ Path ] };
    return segment.GetLength() * blade_steps( Track ) / segment.RaSegCount();
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

} // namespace track_rails

std::vector<track_rails::rail_crossing> TTrack::rail_crossings( bool const Joined ) {

    std::vector<rail_crossing> result;
    if( false == cut_rails( *this ) || simulation::Region == nullptr ) {
        return result;
    }
    auto &memo { memo_of( *this ) };
    if( Joined && memo.joined.has_value() ) {
        return *memo.joined;
    }
    auto const extension { 10.0 };
    auto const paths { eType == tt_Switch ? 2 : 1 };
    if( memo.own.has_value() ) {
        result = *memo.own;
    }
    else {
        auto const sections { simulation::Region->sections( location(), static_cast<float>( radius() + 50.0 ) ) };
        std::vector<TTrack *> others;
        for( auto *section : sections ) {
            for( auto const &cell : section->m_cells ) {
                if( glm::length( cell.area().center - location() ) > cell.area().radius + radius() + 1.0 ) {
                    continue;
                }
                for( auto *other : cell.m_directories.paths ) {
                    if( other == nullptr || other == this || false == cut_rails( *other ) ) {
                        continue;
                    }
                    if( glm::length( other->location() - location() ) > other->radius() + radius() + 1.0 ) {
                        continue;
                    }
                    if( std::find( others.begin(), others.end(), other ) == others.end() ) {
                        others.push_back( other );
                    }
                }
            }
        }
        auto const bend = [&]( gauge_line const &Line, double const At ) {
            glm::dvec3 point, before, after;
            Line.at( At - 1.0, point, before );
            Line.at( At + 1.0, point, after );
            return std::abs( cross( plan( before ), plan( after ) ) ); };
        auto const heading = []( glm::dvec2 const &Tangent ) {
            auto const angle { std::atan2( Tangent.y, Tangent.x ) };
            return ( angle < 0.0 ? angle + M_PI : angle ); };
        auto const overlap = []( gauge_line const &A, gauge_line const &B ) {
            return A.low.x <= B.high.x && B.low.x <= A.high.x && A.low.y <= B.high.y && B.low.y <= A.high.y; };
        for( auto *other : others ) {
            auto const otherpaths { other->eType == tt_Switch ? 2 : 1 };
            for( int otherpath = 0; otherpath < otherpaths; ++otherpath ) {
                auto const &othersegment { path_segment( *other, otherpath ) };
                auto const otherfrom { other->eType == tt_Switch ? blade_length( *other, otherpath ) : 0.0 };
                auto const otherlength { othersegment.GetLength() };
                for( int otherside = 0; otherside < 2; ++otherside ) {
                    auto const &theirs { rail_line( *other, otherpath, otherside, true ) };
                    for( int path = 0; path < paths; ++path ) {
                        auto const length { path_segment( *this, path ).GetLength() };
                        for( int side = 0; side < 2; ++side ) {
                            auto const &line { rail_line( *this, path, side, false ) };
                            if( false == overlap( line, theirs ) ) {
                                continue;
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
                                    glm::dvec3 p, tp, q, tq;
                                    for( int step = 0; step < 6; ++step ) {
                                        line.at( s, p, tp );
                                        theirs.at( t, q, tq );
                                        auto const d { cross( plan( tp ), plan( tq ) ) };
                                        if( std::abs( d ) < 1e-9 ) {
                                            break;
                                        }
                                        auto const offset { plan( q ) - plan( p ) };
                                        s += cross( offset, plan( tq ) ) / d;
                                        t += cross( offset, plan( tp ) ) / d;
                                    }
                                    line.at( s, p, tp );
                                    theirs.at( t, q, tq );
                                    if( glm::length( plan( p ) - plan( q ) ) > 0.01
                                     || s < 0.0 || s > length
                                     || t < otherfrom || t > otherlength ) {
                                        continue;
                                    }
                                    if( std::abs( p.y - q.y ) > 0.5 ) {
                                        continue;
                                    }
                                    if( ( s < 0.5 || s > length - 0.5 ) && ( t < 0.5 || t > otherlength - 0.5 ) && joined( *this, *other ) ) {
                                        continue;
                                    }
                                    auto const tangent { glm::normalize( plan( tp ) ) };
                                    auto const othertangent { glm::normalize( plan( tq ) ) };
                                    auto const angle { std::acos( std::min( 1.0, std::abs( glm::dot( tangent, othertangent ) ) ) ) };
                                    if( angle < 0.03 ) {
                                        continue;
                                    }
                                    auto const sidevalue { side ? 1 : -1 };
                                    if( std::any_of( result.begin(), result.end(), [&]( rail_crossing const &Crossing ) {
                                            return Crossing.path == path && Crossing.side == sidevalue && std::abs( Crossing.at - s ) < 0.05; } ) ) {
                                        continue;
                                    }
                                    auto const normal { glm::normalize( plan( line.normal( s, sidevalue ) ) ) };
                                    auto const othernormal { glm::normalize( plan( theirs.normal( t, otherside ? 1 : -1 ) ) ) };
                                    rail_crossing crossing;
                                    crossing.track = other;
                                    crossing.path = path;
                                    crossing.side = sidevalue;
                                    crossing.at = s;
                                    crossing.angle = angle;
                                    crossing.into = glm::dot( tangent, othernormal );
                                    crossing.facing = glm::dot( normal, othernormal );
                                    auto const ourbend { bend( line, s ) };
                                    auto const theirbend { bend( theirs, t ) };
                                    crossing.nose = ( std::abs( ourbend - theirbend ) > 1e-4 ? ourbend < theirbend : heading( tangent ) < heading( othertangent ) );
                                    crossing.otherpath = otherpath;
                                    crossing.otherside = ( otherside ? 1 : -1 );
                                    crossing.otherat = t;
                                    crossing.aligned = glm::dot( tangent, othertangent ) > 0.0;
                                    result.push_back( crossing );
                                }
                            }
                        }
                    }
                }
            }
        }
        memo.own = result;
    }
    if( false == Joined ) {
        return result;
    }
    // crossings of the tracks going on from the ends, near enough for their check rails and wings to reach this one
    auto const owncount { result.size() };
    for( int path = 0; path < paths; ++path ) {
        auto const &segment { path_segment( *this, path ) };
        auto const length { segment.GetLength() };
        for( int end = 0; end < 2; ++end ) {
            auto *neighbour { eType == tt_Switch ? ( end ? SwitchExtension->pNexts[ path ] : SwitchExtension->pPrevs[ path ] ) : ( end ? trNext : trPrev ) };
            if( neighbour == nullptr || neighbour == this || false == cut_rails( *neighbour ) ) {
                continue;
            }
            auto const joint { end ? segment.FastGetPoint_1() : segment.FastGetPoint_0() };
            auto const theirs { neighbour->rail_crossings( false ) };
            for( int neighbourpath = 0; neighbourpath < ( neighbour->eType == tt_Switch ? 2 : 1 ); ++neighbourpath ) {
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
                    if( crossing.path != neighbourpath || crossing.track == this ) {
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
                    if( std::none_of( result.begin(), result.begin() + owncount, [&]( rail_crossing const &Crossing ) {
                            return Crossing.path == mapped.path && Crossing.side == mapped.side && std::abs( Crossing.at - mapped.at ) < 0.05; } ) ) {
                        result.push_back( mapped );
                    }
                }
            }
        }
    }
    memo.joined = result;
    return result;
}

track_rails::check_rail_set TTrack::check_rails( std::vector<rail_crossing> const &Crossings ) {

    struct planned {
        double from { 0.0 };
        double to { 0.0 };
        bool flarefrom { true };
        bool flareto { true };
        double shortest { 1.5 };
        double flare { 0.02 };
        double frombend { 0.0 };
        double tobend { 0.0 };
    };
    std::vector<planned> plans[ 2 ][ 2 ];
    check_rail_set result;
    auto const size { rail_dimensions() };
    if( size.grooved ) {
        return result;
    }
    auto const acute = [&]( std::vector<planned> &Other, double const At, double const Toward, double const Slope, double const Flare ) {
        auto const half { 0.5 * check_length( Slope ) };
        Other.push_back( { At + Toward * 0.25 - half, At + Toward * 0.25 + half, true, true, 1.5, Flare } );
    };
    for( auto const &crossing : Crossings ) {
        if( crossing.angle > middle_angle || crossing.track->rail_dimensions().grooved ) {
            continue;
        }
        if( crossing.facing < 0.0 ) {
            acute( plans[ crossing.path ][ crossing.side < 0 ], crossing.at, crossing.into > 0.0 ? 1.0 : -1.0, std::tan( crossing.angle ), 0.02 );
        }
        else if( crossing.into > 0.0 ) {
            plans[ crossing.path ][ crossing.side > 0 ].push_back( { crossing.at, crossing.at + 1.2, false, true, 0.5, 0.02, -knuckle_bend( crossing.into ), 0.0 } );
        }
        else {
            plans[ crossing.path ][ crossing.side > 0 ].push_back( { crossing.at - 1.2, crossing.at, true, false, 0.5, 0.02, 0.0, knuckle_bend( crossing.into ) } );
        }
    }
    if( eType == tt_Switch ) {
        auto const frog { this->frog() };
        if( frog.valid ) {
            for( int path = 0; path < 2; ++path ) {
                acute( plans[ path ][ frog.side[ path ] < 0 ], frog.at[ path ], -1.0, std::tan( frog.angle ), 0.04 );
            }
        }
    }
    auto const halfwidth { 0.5 * std::abs( fTrackWidth ) };
    auto const clearance { 0.046 + 0.04 * size.head / 0.0735 + size.head };
    for( int path = 0; path < ( eType == tt_Switch ? 2 : 1 ); ++path ) {
        auto const &segment { path_segment( *this, path ) };
        auto const length { segment.GetLength() };
        for( int right = 0; right < 2; ++right ) {
            auto const side { right ? 1 : -1 };
            auto &checks { plans[ path ][ right ] };
            if( checks.empty() ) {
                continue;
            }
            std::sort( checks.begin(), checks.end(), []( planned const &A, planned const &B ) { return A.from < B.from; } );
            for( std::size_t i = 1; i < checks.size(); ++i ) {
                if( checks[ i ].from < checks[ i - 1 ].to + 2.0 ) {
                    if( checks[ i ].to > checks[ i - 1 ].to ) {
                        checks[ i - 1 ].to = checks[ i ].to;
                        checks[ i - 1 ].flareto = checks[ i ].flareto;
                        checks[ i - 1 ].tobend = checks[ i ].tobend;
                    }
                    checks[ i - 1 ].shortest = std::min( checks[ i - 1 ].shortest, checks[ i ].shortest );
                    checks[ i - 1 ].flare = std::max( checks[ i - 1 ].flare, checks[ i ].flare );
                    checks.erase( checks.begin() + i-- );
                }
            }
            // check rails keep clear of the blades and the rails of the other path of a switch running inside the rail, here and in a switch joined at an end, and of the blade tips
            std::vector<glm::dvec2> blocks;
            auto const start { plan( segment.FastGetPoint_0() ) };
            auto const end { plan( segment.FastGetPoint_1() ) };
            std::vector<gauge_line> others;
            auto const add_path = [&]( TSegment const &Other ) {
                others.push_back( make_gauge_line( Other, -halfwidth, 0.0 ) );
                others.push_back( make_gauge_line( Other, halfwidth, 0.0 ) ); };
            if( eType == tt_Switch ) {
                blocks.emplace_back( -1e9, 0.3 );
                add_path( path_segment( *this, 1 - path ) );
            }
            for( auto const *neighbour : { trPrev, trNext } ) {
                if( neighbour == nullptr || neighbour == this || neighbour->eType != tt_Switch || neighbour->SwitchExtension == nullptr ) {
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
            if( false == others.empty() ) {
                auto const ours { make_gauge_line( segment, side * halfwidth, 0.0 ) };
                for( auto const &check : checks ) {
                    for( auto s { check.from }; s <= check.to; s += 0.1 ) {
                        glm::dvec3 point, tangent;
                        ours.at( s, point, tangent );
                        auto const gauge { plan( point ) };
                        auto const inward { plan( ours.normal( s, side ) ) };
                        auto blocked { false };
                        for( auto const &other : others ) {
                            for( std::size_t i = 1; i < other.points.size() && false == blocked; ++i ) {
                                auto const a { plan( other.points[ i - 1 ] ) };
                                auto const edge { plan( other.points[ i ] ) - a };
                                double t, u;
                                if( false == meet( gauge, inward, a, edge, 1e-9, t, u ) ) {
                                    continue;
                                }
                                blocked = ( u > -1e-6 && u < 1.0 + 1e-6 && t > -0.05 && t < clearance );
                            }
                        }
                        if( blocked ) {
                            blocks.emplace_back( s - 0.3, s + 0.3 );
                        }
                    }
                }
            }
            std::sort( blocks.begin(), blocks.end(), []( glm::dvec2 const &A, glm::dvec2 const &B ) { return A.x < B.x; } );
            for( auto const &check : checks ) {
                auto const add = [&]( double const From, double const To ) {
                    result[ path ][ right ].push_back( {
                        From, To,
                        ( From > check.from || check.flarefrom ) ? check.flare : 0.0,
                        ( To < check.to || check.flareto ) ? check.flare : 0.0,
                        From > check.from ? 0.0 : check.frombend,
                        To < check.to ? 0.0 : check.tobend } ); };
                auto from { check.from };
                for( auto const &block : blocks ) {
                    if( block.y <= from || block.x >= check.to ) {
                        continue;
                    }
                    if( block.x - from >= check.shortest ) {
                        add( from, block.x );
                    }
                    from = std::max( from, block.y );
                }
                if( check.to - from >= check.shortest ) {
                    add( from, check.to );
                }
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
