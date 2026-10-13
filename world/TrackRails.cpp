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
#include "world/Segment.h"
#include "simulation/simulation.h"
#include "scene/scene.h"
#include "rendering/renderer.h"

namespace {

glm::dvec2 plan( glm::dvec3 const &V ) {

    return glm::dvec2{ V.x, V.z };
}

double cross( glm::dvec2 const &A, glm::dvec2 const &B ) {

    return A.x * B.y - A.y * B.x;
}

bool meet( glm::dvec2 const &A0, glm::dvec2 const &A, glm::dvec2 const &B0, glm::dvec2 const &B, double const Parallel, double &U, double &V ) {

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

// rails of the profiles the frogs, check rails and guard rails are made for; others are laid whole
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

TSegment const &path_segment( TTrack const &Track, int const Path ) {

    return ( Track.eType == tt_Switch ? *Track.SwitchExtension->Segments[ Path ] : *Track.Segment );
}

// a check rail starting right beside the end of a wing rail running along the same rail, which the wing rail goes on in place of; the far end of it
bool replaced_check( std::vector<TTrack::check_rail> const &Checks, double const Wingend, double const Direction, double &Far ) {

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

struct rails_memo {
    std::optional<gauge_line> lines[ 2 ][ 2 ][ 2 ]; // path, right side, from the blades
    std::optional<std::vector<TTrack::rail_crossing>> own;
    std::optional<std::vector<TTrack::rail_crossing>> joined;
    std::optional<TTrack::check_rail_set> checks;
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

material_handle cut_material( material_handle const Material ) {

    static auto const unrolled { std::getenv( "EU07_EDITOR_UNROLLED" ) != nullptr };
    return unrolled ? GfxRenderer->Fetch_Material( "editor_unrolled" ) : GfxRenderer->Matte_Material( Material );
}

// in the colour of the top of the rail foot
void cap_loft( gfx::vertex_array const &Strip, std::size_t const Ring, bool const Start, bool const End, gfx::vertex_array &Triangles ) {

    if( Ring < 3 || Strip.size() < 2 * Ring ) {
        return;
    }
    std::vector<std::pair<std::vector<gfx::basic_vertex>, glm::vec3>> caps;
    if( Start ) {
        std::vector<gfx::basic_vertex> ring;
        for( std::size_t j = 0; j < Ring; ++j ) {
            ring.emplace_back( Strip[ 2 * j ] );
        }
        caps.emplace_back( ring, Strip[ 0 ].position - Strip[ 1 ].position );
    }
    if( End ) {
        auto const base { Strip.size() - 2 * Ring };
        std::vector<gfx::basic_vertex> ring;
        for( std::size_t j = 0; j < Ring; ++j ) {
            ring.emplace_back( Strip[ base + 2 * j + 1 ] );
        }
        caps.emplace_back( ring, Strip[ base + 1 ].position - Strip[ base ].position );
    }
    for( auto const &cap : caps ) {
        auto const &ring { cap.first };
        if( glm::length2( cap.second ) == 0.f ) {
            continue;
        }
        auto const outward { glm::normalize( cap.second ) };
        auto across { glm::vec3{ -outward.z, 0.f, outward.x } };
        if( glm::length2( across ) == 0.f ) {
            continue;
        }
        across = glm::normalize( across );
        std::vector<glm::dvec2> plan;
        auto bottom { 0.0 }, top { 0.0 };
        for( auto const &vertex : ring ) {
            auto const offset { vertex.position - ring[ 0 ].position };
            plan.emplace_back( glm::dot( offset, across ), offset.y );
            bottom = std::min<double>( bottom, offset.y );
            top = std::max<double>( top, offset.y );
        }
        auto web { ring[ 0 ].texture };
        auto nearest { std::numeric_limits<double>::max() };
        for( std::size_t j = 0; j < Ring / 2; ++j ) {
            auto const distance { std::abs( plan[ j ].y - 0.5 * ( bottom + top ) ) };
            if( distance < nearest ) {
                nearest = distance;
                web = ring[ j ].texture;
            }
        }
        auto lowest { std::numeric_limits<double>::max() };
        for( std::size_t j = 0; j < Ring / 2; ++j ) {
            if( ring[ j ].normal.y > 0.5f && plan[ j ].y < lowest ) {
                lowest = plan[ j ].y;
                web = ring[ j ].texture;
            }
        }
        auto const cross = []( glm::dvec2 const &A, glm::dvec2 const &B, glm::dvec2 const &C ) {
            return ( B.x - A.x ) * ( C.y - A.y ) - ( B.y - A.y ) * ( C.x - A.x ); };
        double area { 0.0 };
        for( std::size_t j = 0; j < plan.size(); ++j ) {
            auto const &a { plan[ j ] };
            auto const &b { plan[ ( j + 1 ) % plan.size() ] };
            area += a.x * b.y - b.x * a.y;
        }
        auto const orientation { area < 0.0 ? -1.0 : 1.0 };
        std::vector<std::size_t> left( Ring );
        for( std::size_t j = 0; j < Ring; ++j ) {
            left[ j ] = j;
        }
        std::vector<std::array<std::size_t, 3>> triangles;
        while( left.size() > 3 ) {
            auto ear { left.size() };
            for( std::size_t j = 0; j < left.size() && ear == left.size(); ++j ) {
                auto const &a { plan[ left[ ( j + left.size() - 1 ) % left.size() ] ] };
                auto const &b { plan[ left[ j ] ] };
                auto const &c { plan[ left[ ( j + 1 ) % left.size() ] ] };
                auto const turn { cross( a, b, c ) * orientation };
                if( turn < -1e-12 ) {
                    continue;
                }
                auto inside { false };
                if( turn > 1e-12 ) {
                    for( auto const k : left ) {
                        auto const &p { plan[ k ] };
                        if( p == a || p == b || p == c ) {
                            continue;
                        }
                        if( cross( a, b, p ) * orientation > 1e-12 && cross( b, c, p ) * orientation > 1e-12 && cross( c, a, p ) * orientation > 1e-12 ) {
                            inside = true;
                            break;
                        }
                    }
                }
                if( false == inside ) {
                    ear = j;
                }
            }
            if( ear == left.size() ) {
                ear = 0;
            }
            triangles.push_back( { left[ ( ear + left.size() - 1 ) % left.size() ], left[ ear ], left[ ( ear + 1 ) % left.size() ] } );
            left.erase( left.begin() + ear );
        }
        triangles.push_back( { left[ 0 ], left[ 1 ], left[ 2 ] } );
        for( auto const &triangle : triangles ) {
            for( auto const &order : { std::array<int, 3>{ 0, 1, 2 }, std::array<int, 3>{ 0, 2, 1 } } ) {
                for( auto const k : order ) {
                    auto const index { triangle[ k ] };
                    Triangles.emplace_back( ring[ index ].position, outward, web );
                }
            }
        }
    }
}

gfx::userdata_array worn_userdata( std::size_t const Count ) {

    return gfx::userdata_array( Count, gfx::vertex_userdata{ glm::vec4{ 1.f, 0.f, 0.f, 0.f } } );
}

void cut_faces( gfx::vertex_array const &Strip, std::size_t const Ring, bool const Start, bool const End, gfx::geometrybank_handle const &Bank, TTrack::geometryhandle_sequence &Geometry ) {

    gfx::vertex_array triangles;
    cap_loft( Strip, Ring, Start, End, triangles );
    if( false == triangles.empty() ) {
        auto userdata { worn_userdata( triangles.size() ) };
        Geometry.emplace_back( GfxRenderer->Insert( triangles, userdata, Bank, GL_TRIANGLES ) );
    }
}

glm::dvec3 rail_point( TTrack const &Track, int const Path, int const Side, double const Distance, double const Lateral ) {

    auto const line { make_gauge_line( path_segment( Track, Path ), Side * 0.5 * std::abs( Track.fTrackWidth ), 0.0 ) };
    glm::dvec3 point, tangent;
    line.at( Distance, point, tangent );
    return point - line.normal( Distance, Side ) * Lateral;
}

void rail_offset( TTrack const &Track, int const Path, int const Side, glm::dvec3 const &Point, double &Distance, double &Lateral ) {

    auto const line { make_gauge_line( path_segment( Track, Path ), Side * 0.5 * std::abs( Track.fTrackWidth ), 0.0 ) };
    Distance = line.distances.front();
    auto nearest { std::numeric_limits<double>::max() };
    for( std::size_t i = 0; i < line.points.size(); ++i ) {
        auto const gap { glm::length( plan( line.points[ i ] - Point ) ) };
        if( gap < nearest ) {
            nearest = gap;
            Distance = line.distances[ i ];
        }
    }
    glm::dvec3 point, tangent;
    for( int step = 0; step < 8; ++step ) {
        line.at( Distance, point, tangent );
        Distance += glm::dot( plan( Point - point ), glm::normalize( plan( tangent ) ) );
    }
    line.at( Distance, point, tangent );
    Lateral = -glm::dot( plan( Point - point ), plan( line.normal( Distance, Side ) ) );
}

} // namespace track_rails

using namespace track_rails;

namespace {

// gauge line of a rail of a track, along a path of a switch from past its blades when Blades
gauge_line const &rail_line( TTrack const &Track, int const Path, int const Side, bool const Blades ) {

    auto const blades { Blades && Track.eType == tt_Switch };
    auto &line { memo_of( Track ).lines[ Path ][ Side ][ blades ] };
    if( false == line.has_value() ) {
        line = make_gauge_line( path_segment( Track, Path ), ( Side ? 1.0 : -1.0 ) * ( 0.5 * std::abs( Track.fTrackWidth ) ), blades ? blade_length( Track, Path ) : 0.0 );
    }
    return *line;
}

TTrack::check_rail_set const &rail_checks( TTrack &Track ) {

    auto &checks { memo_of( Track ).checks };
    if( false == checks.has_value() ) {
        checks = Track.check_rails( Track.rail_crossings() );
    }
    return *checks;
}

} // namespace

std::vector<TTrack::rail_crossing> TTrack::rail_crossings( bool const Joined ) {

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

TTrack::check_rail_set TTrack::check_rails( std::vector<rail_crossing> const &Crossings ) {

    struct planned {
        double from { 0.0 };
        double to { 0.0 };
        bool flarefrom { true };
        bool flareto { true };
        double shortest { 1.5 };
        double flare { 0.02 };
    };
    std::vector<planned> plans[ 2 ][ 2 ];
    check_rail_set result;
    auto const size { rail_dimensions() };
    auto const acute = [&]( std::vector<planned> &Other, double const At, double const Toward, double const Slope, double const Flare ) {
        auto const half { 0.5 * check_length( Slope ) };
        Other.push_back( { At + Toward * 0.25 - half, At + Toward * 0.25 + half, true, true, 1.5, Flare } );
    };
    for( auto const &crossing : Crossings ) {
        if( crossing.angle > middle_angle ) {
            continue;
        }
        if( crossing.facing < 0.0 ) {
            acute( plans[ crossing.path ][ crossing.side < 0 ], crossing.at, crossing.into > 0.0 ? 1.0 : -1.0, std::tan( crossing.angle ), 0.02 );
        }
        else if( crossing.into > 0.0 ) {
            plans[ crossing.path ][ crossing.side > 0 ].push_back( { crossing.at - 0.03, crossing.at + 1.2, false, true, 0.5, 0.02 } );
        }
        else {
            plans[ crossing.path ][ crossing.side > 0 ].push_back( { crossing.at - 1.2, crossing.at + 0.03, true, false, 0.5, 0.02 } );
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
                        ( To < check.to || check.flareto ) ? check.flare : 0.0 } ); };
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

TTrack::switch_frog TTrack::frog() const {

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

namespace {

// a wing rail ending right beside a check rail of the rail it runs along goes on in its place up to the far end of it, past the end of the wing
std::vector<glm::dvec2> wing_extension( TTrack &Track, TTrack::check_rail_set const &Checks, int const Path, int const Side, double const At, double const Toward, double const Slope, TTrack &Partner, int const Partnerpath, int const Partnerside, double const Partnerat, double const Partnerdirection ) {

    std::vector<glm::dvec2> knots;
    auto const partnerchecks { &Partner == &Track ? Checks : rail_checks( Partner ) };
    auto const wingend { Partnerat + Partnerdirection * wing_reach( Slope ) };
    auto farend { 0.0 };
    if( false == replaced_check( partnerchecks[ Partnerpath ][ Partnerside > 0 ], wingend, Partnerdirection, farend ) ) {
        return knots;
    }
    auto const theirs { make_gauge_line( path_segment( Partner, Partnerpath ), Partnerside * ( 0.5 * std::abs( Partner.fTrackWidth ) - flangeway ), 0.0 ) };
    auto const ours { make_gauge_line( path_segment( Track, Path ), Side * ( 0.5f * std::abs( Track.fTrackWidth ) ), 0.0 ) };
    auto s { At - Toward * wing_reach( Slope ) };
    auto const count { std::max( 1, static_cast<int>( std::ceil( std::abs( farend - wingend ) / 0.5 ) ) ) };
    for( int i = 1; i <= count; ++i ) {
        glm::dvec3 target, tangent, point;
        theirs.at( glm::mix( wingend, farend, static_cast<double>( i ) / count ), target, tangent );
        for( int step = 0; step < 4; ++step ) {
            ours.at( s, point, tangent );
            s += glm::dot( plan( target - point ), glm::normalize( plan( tangent ) ) );
        }
        ours.at( s, point, tangent );
        knots.emplace_back( s, -glm::dot( plan( target - point ), plan( ours.normal( s, Side ) ) ) );
    }
    return knots;
}

// acute frog: the rail coming from the throat bends out as a wing rail, the rail going on behind the gap is a leg of the nose
void acute( TTrack::rail_plan &Plan, double const At, double const Toward, double const Slope, bool const Nose, std::vector<glm::dvec2> const &Extension, TTrack::rail_size const &Size ) {

    auto const railhead { Size.head };
    auto const apart { Size.high + 0.1 };
    auto const throat { At + Toward * flangeway / Slope };
    auto const flare { At - Toward * wing_reach( Slope ) };
    auto const tip { Nose ? nose_tip : railhead + 0.002 };
    auto const nose { At - Toward * tip / Slope };
    auto const blunt { At - Toward * apart / Slope };
    auto const reach = [&]( double const Apart ) {
        return Nose ? Apart : Apart - railhead + 0.002; };
    std::vector<glm::dvec2> wing { { throat, 0.0 }, { flare, std::abs( flare - throat ) * Slope } };
    wing.insert( wing.end(), Extension.begin(), Extension.end() );
    auto const wingend { Extension.empty() ? flare : Extension.back().x };
    std::vector<glm::dvec2> rolled;
    for( auto const &knot : wing ) {
        rolled.emplace_back( knot.x, rolled_band - knot.y );
    }
    std::sort( rolled.begin(), rolled.end(), []( glm::dvec2 const &A, glm::dvec2 const &B ) { return A.x < B.x; } );
    TTrack::rail_cut cut;
    cut.at = At;
    if( Toward < 0.0 ) {
        cut.end = wingend;
        cut.endoffsets = wing;
        cut.endrolled = rolled;
        cut.start = nose;
        cut.startextents = { { nose, reach( tip ) }, { blunt, reach( apart ) } };
    }
    else {
        cut.end = nose;
        cut.endextents = { { blunt, reach( apart ) }, { nose, reach( tip ) } };
        cut.start = wingend;
        cut.startoffsets = wing;
        cut.startrolled = rolled;
    }
    Plan.cuts.push_back( cut );
}

// obtuse crossing: the rail ends where it meets the other one and starts again pointed past its flangeway, a check rail beside the point guards the gap
void obtuse( TTrack::rail_plan &Plan, double const At, double const Into, TTrack::rail_size const &Size ) {

    auto const gap { flangeway / std::abs( Into ) };
    auto const full { Size.high + 0.0135 };
    auto const taper { ( flangeway + full ) / std::abs( Into ) };
    TTrack::rail_cut cut;
    cut.at = At;
    if( Into > 0.0 ) {
        cut.end = At;
        cut.start = At + gap;
        cut.startextents = { { At + gap, 0.006 }, { At + taper, full } };
    }
    else {
        cut.end = At - gap;
        cut.endextents = { { At - taper, full }, { At - gap, 0.006 } };
        cut.start = At;
    }
    Plan.cuts.push_back( cut );
}

// crossing at a wide angle: running rails are broken for the flanges of the other track, the ones of the track not going through also where the head of the other rail lies; guard rails give way to the running rails and flangeways of the other track, the ones of the track not going through also to its guard rails
void wide( TTrack::rail_plan &Plan, double const At, double const Into, double const Facing, double const From, double const To, double const Inner, double const Outer ) {

    auto const low { ( From + std::min( Inner * Facing, Outer * Facing ) - 0.005 ) / Into };
    auto const high { ( To + std::max( Inner * Facing, Outer * Facing ) + 0.005 ) / Into };
    TTrack::rail_cut cut;
    cut.at = At;
    cut.end = At + std::min( low, high );
    cut.start = At + std::max( low, high );
    cut.endmiter = { At + ( Into > 0.0 ? From - 0.005 : To + 0.005 ) / Into, Facing / Into };
    cut.startmiter = { At + ( Into > 0.0 ? To + 0.005 : From - 0.005 ) / Into, Facing / Into };
    Plan.cuts.push_back( cut );
}

bool goes_through( TTrack const &Track, TTrack const &Other ) {

    if( Track.name() != Other.name() ) {
        return Track.name() < Other.name();
    }
    auto const here { Track.location() };
    auto const there { Other.location() };
    return ( here.x != there.x ? here.x < there.x : here.z < there.z );
}

// crossings at a middle angle: guard rails along the sides of the quadrilateral between the tracks, glued at its corners to the guard rails of the other track
struct frame_corner {
    TTrack *track { nullptr };
    int path { 0 };
    int side { 0 };
    int otherpath { 0 };
    double at { 0.0 };
    double into { 0.0 };
    double facing { 0.0 };
};

} // namespace

TTrack::rail_layout TTrack::plan_rails( bool const Crossings ) {

    rail_layout layout;
    auto const size { rail_dimensions() };
    auto const railhead { size.head };
    std::vector<rail_crossing> crossings;
    if( Crossings ) {
        crossings = rail_crossings();
    }
    layout.checks = ( Crossings ? rail_checks( *this ) : check_rails( crossings ) );
    auto const &checks { layout.checks };
    auto &plans { layout.plans };
    auto &guards { layout.guards };
    auto &wingends { layout.wingends };
    auto const guardoffset { -( flangeway + railhead ) };
    std::vector<frame_corner> corners;
    if( Crossings ) {
        for( auto const &crossing : crossings ) {
            auto &plan { plans[ crossing.path ][ crossing.side > 0 ] };
            if( crossing.angle > middle_angle && crossing.angle <= wide_angle ) {
                corners.push_back( { crossing.track, crossing.path, crossing.side, crossing.otherpath, crossing.at, crossing.into, crossing.facing } );
            }
            if( crossing.angle > wide_angle ) {
                auto const other { crossing.track->rail_dimensions() };
                auto const through { goes_through( *this, *crossing.track ) };
                wide( plan, crossing.at, crossing.into, crossing.facing, through ? 0.0 : -other.head, flangeway, 0.0, railhead );
                auto const reach { ( flangeway + railhead + 0.3 ) / std::abs( crossing.into ) };
                auto &spans { guards[ crossing.path ][ crossing.side > 0 ] };
                TTrack::guard_rail guard { crossing.at - reach, crossing.at + reach };
                wide( guard.plan, crossing.at, crossing.into, crossing.facing, -other.head, through ? flangeway : flangeway + other.head, guardoffset, guardoffset + railhead );
                auto merged { false };
                for( auto &span : spans ) {
                    if( guard.from < span.to + 2.0 && guard.to > span.from - 2.0 ) {
                        span.from = std::min( span.from, guard.from );
                        span.to = std::max( span.to, guard.to );
                        span.plan.cuts.insert( span.plan.cuts.end(), guard.plan.cuts.begin(), guard.plan.cuts.end() );
                        merged = true;
                        break;
                    }
                }
                if( false == merged ) {
                    spans.push_back( guard );
                }
            }
            else if( crossing.facing < 0.0 ) {
                auto const toward { crossing.into > 0.0 ? 1.0 : -1.0 };
                auto const slope { std::tan( crossing.angle ) };
                wingends[ crossing.path ][ crossing.side > 0 ].emplace_back( crossing.at - toward * wing_reach( slope ), -toward );
                acute( plan, crossing.at, toward, slope, crossing.nose,
                    wing_extension( *this, checks, crossing.path, crossing.side, crossing.at, toward, slope, *crossing.track, crossing.otherpath, crossing.otherside, crossing.otherat, crossing.aligned ? -toward : toward ), size );
            }
            else {
                obtuse( plan, crossing.at, crossing.into, size );
            }
            if( std::find( layout.crossed.begin(), layout.crossed.end(), crossing.track ) == layout.crossed.end() ) {
                layout.crossed.push_back( crossing.track );
            }
        }
        for( auto &pathguards : guards ) {
            std::vector<glm::dvec2> spans[ 2 ];
            for( int side = 0; side < 2; ++side ) {
                for( auto const &guard : pathguards[ side ] ) {
                    spans[ side ].emplace_back( guard.from, guard.to );
                }
            }
            for( int side = 0; side < 2; ++side ) {
                for( auto &guard : pathguards[ side ] ) {
                    for( auto const &opposite : spans[ 1 - side ] ) {
                        if( opposite.x < guard.to && opposite.y > guard.from ) {
                            guard.from = std::min( guard.from, opposite.x );
                            guard.to = std::max( guard.to, opposite.y );
                        }
                    }
                }
            }
        }
        std::sort( corners.begin(), corners.end(), []( frame_corner const &A, frame_corner const &B ) { return A.at < B.at; } );
        auto const glued = [&]( frame_corner const &Corner ) {
            return Corner.at + ( flangeway + std::max( guardoffset * Corner.facing, ( guardoffset + railhead ) * Corner.facing ) ) / Corner.into; };
        auto const gluedmiter = [&]( frame_corner const &Corner ) {
            return glm::dvec2 { Corner.at + flangeway / Corner.into, Corner.facing / Corner.into }; };
        for( std::size_t i = 0; i < corners.size(); ++i ) {
            auto const &start { corners[ i ] };
            if( start.into <= 0.0 ) {
                continue;
            }
            for( std::size_t j = i + 1; j < corners.size(); ++j ) {
                auto const &end { corners[ j ] };
                if( end.track != start.track || end.path != start.path || end.side != start.side || end.otherpath != start.otherpath ) {
                    continue;
                }
                if( end.into < 0.0 ) {
                    TTrack::guard_rail guard { glued( start ), glued( end ) };
                    guard.flarefrom = false;
                    guard.flareto = false;
                    guard.frommiter = gluedmiter( start );
                    guard.tomiter = gluedmiter( end );
                    for( auto const *corner : { &start, &end } ) {
                        if( corner->facing < 0.0 ) {
                            continue;
                        }
                        auto const forward { corner == &end };
                        auto reach { forward ? guard.to : guard.from };
                        for( auto const &opposite : corners ) {
                            if( opposite.track != start.track || opposite.path != start.path || opposite.side != -start.side || opposite.otherpath != start.otherpath ) {
                                continue;
                            }
                            auto const margin { ( flangeway + railhead + 0.3 ) / std::abs( opposite.into ) };
                            reach = forward ? std::max( reach, opposite.at + margin ) : std::min( reach, opposite.at - margin );
                        }
                        if( reach == ( forward ? guard.to : guard.from ) ) {
                            continue;
                        }
                        wide( guard.plan, corner->at, corner->into, corner->facing, -corner->track->rail_dimensions().head, flangeway, guardoffset, guardoffset + railhead );
                        auto &cut { guard.plan.cuts.back() };
                        if( forward ) {
                            cut.end = guard.to;
                            cut.endmiter = guard.tomiter;
                            guard.to = reach;
                            guard.tomiter = {};
                            guard.flareto = true;
                        }
                        else {
                            cut.start = guard.from;
                            cut.startmiter = guard.frommiter;
                            guard.from = reach;
                            guard.frommiter = {};
                            guard.flarefrom = true;
                        }
                    }
                    if( guard.to - guard.from > 0.1 ) {
                        guards[ start.path ][ start.side > 0 ].push_back( guard );
                    }
                }
                break;
            }
        }
    }
    if( eType == tt_Switch ) {
        auto const frog { this->frog() };
        if( frog.valid ) {
            auto const slope { std::tan( frog.angle ) };
            for( int path = 0; path < 2; ++path ) {
                wingends[ path ][ frog.side[ path ] > 0 ].emplace_back( frog.at[ path ] + wing_reach( slope ), 1.0 );
                acute( plans[ path ][ frog.side[ path ] > 0 ], frog.at[ path ], -1.0, slope, path == 0,
                    wing_extension( *this, checks, path, frog.side[ path ], frog.at[ path ], -1.0, slope, *this, 1 - path, frog.side[ 1 - path ], frog.at[ 1 - path ], 1.0 ), size );
            }
        }
    }
    return layout;
}

TTrack::rail_drawing TTrack::make_rail_drawing( gfx::geometrybank_handle const &Bank ) {

    rail_drawing drawing;
    drawing.bank = Bank;
    drawing.size = rail_dimensions();
    drawing.halfwidth = 0.5f * std::abs( fTrackWidth );
    // check rail: a narrow head facing the running rail across the flangeway, on a thin web with a foot turned away from it, sized after the rail
    gfx::vertex_array checkshape {
        {{ -0.041f, -0.170f, 0.f }, {  1.000f,  0.000f, 0.f }, { 0.15f, 0.f }},
        {{ -0.041f, -0.010f, 0.f }, {  0.707f,  0.707f, 0.f }, { 0.40f, 0.f }},
        {{ -0.051f, -0.000f, 0.f }, {  0.000f,  1.000f, 0.f }, { 0.45f, 0.f }},
        {{ -0.071f, -0.000f, 0.f }, {  0.000f,  1.000f, 0.f }, { 0.55f, 0.f }},
        {{ -0.081f, -0.010f, 0.f }, { -0.707f,  0.707f, 0.f }, { 0.60f, 0.f }},
        {{ -0.081f, -0.045f, 0.f }, { -0.707f, -0.707f, 0.f }, { 0.65f, 0.f }},
        {{ -0.066f, -0.060f, 0.f }, { -0.707f, -0.707f, 0.f }, { 0.75f, 0.f }},
        {{ -0.066f, -0.150f, 0.f }, { -0.707f,  0.707f, 0.f }, { 0.85f, 0.f }},
        {{ -0.101f, -0.170f, 0.f }, { -1.000f,  0.000f, 0.f }, { 1.00f, 0.f }} };
    for( auto &point : checkshape ) {
        point.position.x = -0.041f + ( point.position.x + 0.041f ) * static_cast<float>( drawing.size.head / 0.0735 );
        point.position.y *= static_cast<float>( drawing.size.height / 0.172 );
    }
    create_track_rail_profile( drawing.checkshapes[ 0 ], drawing.checkshapes[ 1 ], &checkshape );
    for( auto &shape : drawing.checkshapes ) {
        shape.resize( checkshape.size() );
    }
    return drawing;
}

bool TTrack::lay_rail( rail_layout const &Layout, rail_drawing const &Drawing, int const Path, gfx::vertex_array const &Profile, int const Side, float const Texturelength, double const From, bool const Openstart, bool const Openend, geometryhandle_sequence &Geometry, geometryhandle_sequence &Cuts ) {

    gfx::userdata_array empty_userdata;
    auto const &size { Drawing.size };
    auto const halfwidth { Drawing.halfwidth };
    auto const railhead { size.head };
    auto const guardoffset { -( flangeway + railhead ) };
    auto const &segment { path_segment( *this, Path ) };
    auto const &plan { Layout.plans[ Path ][ Side > 0 ] };
    auto const length { segment.GetLength() };
    // parts no wheel rolls on go in the matte material of the cut faces
    auto const piece = [&]( double const Start, double const End, std::vector<glm::dvec2> const &Offsets, std::vector<glm::dvec2> const &Extents, bool const Capstart, bool const Capend, gfx::vertex_array const &Shape, bool const Rolled, std::vector<glm::dvec2> const &Inner, std::vector<glm::dvec2> const &Ramp = {} ) {
        gfx::vertex_array vertices;
        gfx::userdata_array userdata;
        if( Shape.size() > 1 && segment.RenderLoftSpan( vertices, m_origin, Shape, Side, halfwidth, Texturelength, Start, End, Offsets, Extents, 0.f, Inner, Rolled ? nullptr : &userdata, Ramp, Ramp.empty() ? 0.0 : blend ) ) {
            cut_faces( vertices, Shape.size(), Capstart, Capend, Drawing.bank, Cuts );
            gfx::vertex_array triangles;
            gfx::userdata_array triangleuserdata;
            auto const ring { 2 * Shape.size() };
            for( std::size_t block = 0; block + ring <= vertices.size(); block += ring ) {
                for( std::size_t j = 0; j + 1 < Shape.size(); ++j ) {
                    for( auto const index : { 0, 1, 2, 1, 3, 2 } ) {
                        auto const vertex { block + 2 * j + index };
                        triangles.push_back( vertices[ vertex ] );
                        if( false == Rolled ) {
                            triangleuserdata.push_back( userdata[ vertex ] );
                        }
                    }
                }
            }
            ( Rolled ? Geometry : Cuts ).emplace_back( GfxRenderer->Insert( triangles, Rolled ? empty_userdata : triangleuserdata, Drawing.bank, GL_TRIANGLES ) );
        }
    };
    auto const level = []( std::vector<glm::dvec2> const &Knots, double const Distance ) {
        if( Distance <= Knots.front().x ) {
            return Knots.front().y;
        }
        for( std::size_t i = 1; i < Knots.size(); ++i ) {
            if( Distance <= Knots[ i ].x ) {
                return glm::mix( Knots[ i - 1 ].y, Knots[ i ].y, ( Distance - Knots[ i - 1 ].x ) / std::max( Knots[ i ].x - Knots[ i - 1 ].x, 1e-6 ) );
            }
        }
        return Knots.back().y; };
    auto const lowest = [&]( std::vector<glm::dvec2> const &A, std::vector<glm::dvec2> const &B ) {
        if( A.empty() || B.empty() ) {
            return ( A.empty() ? B : A );
        }
        std::vector<double> stations;
        for( auto const *knots : { &A, &B } ) {
            for( auto const &knot : *knots ) {
                stations.push_back( knot.x );
            }
        }
        std::sort( stations.begin(), stations.end() );
        auto const count { stations.size() };
        for( std::size_t i = 1; i < count; ++i ) {
            auto const a { stations[ i - 1 ] };
            auto const b { stations[ i ] };
            auto const before { level( A, a ) - level( B, a ) };
            auto const after { level( A, b ) - level( B, b ) };
            if( before * after < 0.0 ) {
                stations.push_back( a + ( b - a ) * before / ( before - after ) );
            }
        }
        std::sort( stations.begin(), stations.end() );
        std::vector<glm::dvec2> result;
        for( auto const station : stations ) {
            result.emplace_back( station, std::min( level( A, station ), level( B, station ) ) );
        }
        return result; };
    // the running surface on top of the head is split along the edge of the wheel treads, the part of it out of their reach goes matte; the rest of the profile and the end faces are laid whole
    std::size_t topfirst { Profile.size() };
    std::size_t toplast { 0 };
    for( std::size_t j = 0; j < Profile.size(); ++j ) {
        if( Profile[ j ].position.y > static_cast<float>( -0.087 * size.height ) ) {
            topfirst = std::min( topfirst, j );
            toplast = std::max( toplast, j );
        }
    }
    auto const strip = [&]( std::size_t const First, std::size_t const Last ) {
        return gfx::vertex_array( Profile.begin() + First, Profile.begin() + Last + 1 ); };
    auto const running = [&]( double const Start, double const End, std::vector<glm::dvec2> const &Offsets, std::vector<glm::dvec2> const &Extents, std::vector<glm::dvec2> const &Rolled, bool const Capstart, bool const Capend ) {
        gfx::vertex_array vertices;
        if( segment.RenderLoftSpan( vertices, m_origin, Profile, Side, halfwidth, Texturelength, Start, End, Offsets, Extents ) ) {
            cut_faces( vertices, Profile.size(), Capstart, Capend, Drawing.bank, Cuts );
        }
        piece( Start, End, Offsets, Extents, false, false, strip( 0, topfirst ), true, {} );
        piece( Start, End, Offsets, Extents, false, false, strip( toplast, Profile.size() - 1 ), true, {} );
        auto const top { strip( topfirst, toplast ) };
        auto low { 1e9 };
        auto high { -1e9 };
        for( auto const &point : top ) {
            low = std::min( low, static_cast<double>( Side * point.position.x - halfwidth ) );
            high = std::max( high, static_cast<double>( Side * point.position.x - halfwidth ) );
        }
        std::vector<glm::dvec2> inner;
        for( auto const &knot : Rolled ) {
            inner.emplace_back( knot.x, knot.y - blend );
        }
        std::vector<double> splits { Start, End };
        for( auto const edge : { low, high, low + blend, high + blend } ) {
            for( std::size_t i = 1; i < Rolled.size(); ++i ) {
                auto const &a { Rolled[ i - 1 ] };
                auto const &b { Rolled[ i ] };
                if( ( a.y - edge ) * ( b.y - edge ) < 0.0 ) {
                    auto const distance { a.x + ( edge - a.y ) / ( b.y - a.y ) * ( b.x - a.x ) };
                    if( distance > Start && distance < End ) {
                        splits.push_back( distance );
                    }
                }
            }
        }
        std::sort( splits.begin(), splits.end() );
        auto const shiny { lowest( Extents, inner ) };
        auto const blended { lowest( Extents, Rolled ) };
        for( std::size_t i = 1; i < splits.size(); ++i ) {
            auto const reach { level( Rolled, 0.5 * ( splits[ i - 1 ] + splits[ i ] ) ) };
            if( reach - blend > low ) {
                piece( splits[ i - 1 ], splits[ i ], Offsets, shiny, false, false, top, true, {} );
            }
            if( reach > low && reach - blend < high ) {
                piece( splits[ i - 1 ], splits[ i ], Offsets, blended, false, false, top, false, inner, Rolled );
            }
            if( reach < high ) {
                piece( splits[ i - 1 ], splits[ i ], Offsets, Extents, false, false, top, false, Rolled );
            }
        }
    };
    for( auto const &guard : Layout.guards[ Path ][ Side > 0 ] ) {
        auto const flare { std::min( 0.3, 0.25 * ( guard.to - guard.from ) ) };
        std::vector<glm::dvec2> offsets;
        if( guard.flarefrom ) {
            offsets.insert( offsets.end(), { { guard.from, guardoffset - 0.02 }, { guard.from + flare, guardoffset } } );
        }
        else {
            offsets.push_back( { guard.from, guardoffset } );
        }
        if( guard.flareto ) {
            offsets.insert( offsets.end(), { { guard.to - flare, guardoffset }, { guard.to, guardoffset - 0.02 } } );
        }
        else {
            offsets.push_back( { guard.to, guardoffset } );
        }
        auto const top { flangeway + railhead + size.low - 0.002 };
        auto const mitered = [&]( glm::dvec2 const &Miter, double const Square, bool const Latest ) {
            if( Miter.y == 0.0 ) {
                return Square;
            }
            auto const inside { Miter.x + Miter.y * ( size.low + guardoffset ) };
            auto const outside { Miter.x + Miter.y * ( top + guardoffset ) };
            return Latest ? std::max( inside, outside ) : std::min( inside, outside ); };
        auto const envelope = [&]( std::vector<glm::dvec2> const &Lines, bool const Upper, double const A, double const B ) {
            std::vector<glm::dvec2> knots;
            if( Lines.empty() ) {
                return knots;
            }
            std::vector<double> stations { A, B };
            for( std::size_t i = 0; i < Lines.size(); ++i ) {
                for( std::size_t j = i + 1; j < Lines.size(); ++j ) {
                    if( std::abs( Lines[ i ].x - Lines[ j ].x ) > 1e-12 ) {
                        auto const d { ( Lines[ j ].y - Lines[ i ].y ) / ( Lines[ i ].x - Lines[ j ].x ) };
                        if( d > A && d < B ) {
                            stations.push_back( d );
                        }
                    }
                }
            }
            std::sort( stations.begin(), stations.end() );
            for( auto const d : stations ) {
                auto value { Upper ? 1e6 : -1e6 };
                for( auto const &line : Lines ) {
                    value = Upper ? std::min( value, line.x * d + line.y ) : std::max( value, line.x * d + line.y );
                }
                knots.emplace_back( d, value );
            }
            return knots; };
        auto cuts { guard.plan.cuts };
        std::sort( cuts.begin(), cuts.end(), []( rail_cut const &A, rail_cut const &B ) { return A.at < B.at; } );
        auto start { guard.from };
        auto startmiter { guard.frommiter };
        for( std::size_t i = 0; i <= cuts.size(); ++i ) {
            auto const end { i < cuts.size() ? cuts[ i ].end : guard.to };
            auto const endmiter { i < cuts.size() ? cuts[ i ].endmiter : guard.tomiter };
            auto const a { std::max( { mitered( startmiter, start, false ), From, 0.0 } ) };
            auto const b { std::min( { mitered( endmiter, end, true ), length, mitered( guard.tomiter, guard.to, true ) } ) };
            if( b - a > 0.01 ) {
                std::vector<glm::dvec2> uppers { { 0.0, top } };
                std::vector<glm::dvec2> lowers;
                for( auto const &[ miter, ending ] : { std::pair{ startmiter, false }, std::pair{ endmiter, true } } ) {
                    if( miter.y != 0.0 ) {
                        ( ending == ( miter.y < 0.0 ) ? uppers : lowers ).emplace_back( 1.0 / miter.y, -miter.x / miter.y - guardoffset );
                    }
                }
                piece( a, b, offsets, envelope( uppers, true, a, b ), true, true, Profile, false, envelope( lowers, false, a, b ) );
            }
            if( i < cuts.size() && cuts[ i ].start > start ) {
                start = cuts[ i ].start;
                startmiter = cuts[ i ].startmiter;
            }
        }
    }
    auto const &shape { Drawing.checkshapes[ Side > 0 ? 0 : 1 ] };
    for( auto const &check : Layout.checks[ Path ][ Side > 0 ] ) {
        auto const replaced { std::any_of( Layout.wingends[ Path ][ Side > 0 ].begin(), Layout.wingends[ Path ][ Side > 0 ].end(), [&]( glm::dvec2 const &Wing ) {
            auto farend { 0.0 };
            return replaced_check( { check }, Wing.x, Wing.y, farend ); } ) };
        auto const from { check.from };
        auto const to { check.to };
        auto const a { std::max( from, From ) };
        auto const b { std::min( to, length ) };
        if( replaced || b - a <= 0.01 ) {
            continue;
        }
        auto const flare { std::min( 0.6, 0.4 * ( to - from ) ) };
        std::vector<glm::dvec2> offsets;
        if( check.flarefrom > 0.0 ) {
            offsets.insert( offsets.end(), { { from, -check.flarefrom }, { from + flare, 0.0 } } );
        }
        else {
            offsets.push_back( { from, 0.0 } );
        }
        if( check.flareto > 0.0 ) {
            offsets.insert( offsets.end(), { { to - flare, 0.0 }, { to, -check.flareto } } );
        }
        else {
            offsets.push_back( { to, 0.0 } );
        }
        piece( a, b, offsets, {}, a > 0.0 || from >= 0.0, to <= length, shape, false, {} );
    }
    if( plan.cuts.empty() ) {
        return false;
    }
    auto cuts { plan.cuts };
    std::sort( cuts.begin(), cuts.end(), []( rail_cut const &A, rail_cut const &B ) { return A.at < B.at; } );
    auto const byposition = []( glm::dvec2 const &A, glm::dvec2 const &B ) { return A.x < B.x; };
    auto start { -1e9 };
    std::vector<glm::dvec2> rolledfrom;
    std::vector<glm::dvec2> offsets, extents;
    for( std::size_t i = 0; i <= cuts.size(); ++i ) {
        auto const end { i < cuts.size() ? cuts[ i ].end : 1e9 };
        auto pieceoffsets { offsets };
        auto pieceextents { extents };
        if( i < cuts.size() ) {
            pieceoffsets.insert( pieceoffsets.end(), cuts[ i ].endoffsets.begin(), cuts[ i ].endoffsets.end() );
            pieceextents.insert( pieceextents.end(), cuts[ i ].endextents.begin(), cuts[ i ].endextents.end() );
        }
        std::sort( pieceoffsets.begin(), pieceoffsets.end(), byposition );
        std::sort( pieceextents.begin(), pieceextents.end(), byposition );
        auto const a { std::max( start, From ) };
        auto const b { std::min( end, length ) };
        auto const rolled { i < cuts.size() ? lowest( rolledfrom, cuts[ i ].endrolled ) : rolledfrom };
        auto const capstart { start > From ? true : Openstart };
        auto const capend { end < length ? true : Openend };
        if( b - a > 0.01 ) {
            if( rolled.empty() || topfirst >= toplast ) {
                piece( a, b, pieceoffsets, pieceextents, capstart, capend, Profile, true, {} );
            }
            else {
                running( a, b, pieceoffsets, pieceextents, rolled, capstart, capend );
            }
        }
        if( i < cuts.size() ) {
            rolledfrom = cuts[ i ].startrolled;
            start = cuts[ i ].start;
            offsets = cuts[ i ].startoffsets;
            extents = cuts[ i ].startextents;
        }
    }
    return true;
}
