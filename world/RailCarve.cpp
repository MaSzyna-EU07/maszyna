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
#include "utilities/Logs.h"

using namespace track_rails;

// the parts of the rails at crossings at small angles, as designed by the frogs, are carved out of what lies around them. A part keeps its head
// out of the flangeways of the rails around where their wheels run (F), then the heads lying over each other share the room between them by
// what the wheels of each roll on (O), and the ends of the parts go where what's left of their heads ends as declared: wing rails in a point,
// the legs of a nose and the points of obtuse crossings blunt
namespace {

double constexpr coarse_step { 0.25 }; // how far apart the rails around are looked for along a part
double constexpr fine_step { 0.02 }; // how far apart the part is carved where they're near, on stations at whole steps along the path
double constexpr window { 0.25 }; // how far around a station with a rail near the part is carved
double constexpr nearby { 0.4 }; // how far along past the ends of a rail around it's near enough to be looked at closer
double constexpr across_margin { 0.05 }; // and how far across from its reach, more where it runs at an angle
double constexpr outside { 10.0 }; // how far past the ends of the track a part is looked at for where it ends
double constexpr slack { 0.0002 }; // how far a head reaches into a flangeway or another head unnoticed, and how far two heads planed against each other are kept apart
double constexpr point_width { 0.001 }; // a wing rail ends where less than this is left of its head
double constexpr blunt_width { 0.5 * nose_tip }; // the legs of a nose and the points of an obtuse crossing begin where this much is left of their heads
double constexpr runon_bend { 1.0 / 6.0 }; // how much more the end of a wing rail run on past its own end bends out
double constexpr far_end { 1e8 };
double constexpr heel_margin { 0.1 }; // how far past the heels of the blades of a switch its rails on a side lie at each other as made

int faults { 0 };

double const cos_wide { std::cos( wide_angle ) };
double const unbounded { std::numeric_limits<double>::infinity() };

// EU07_CARVE_DEBUG=track/path+side logs how the parts of that rail are carved
std::string const &debugged() {

    static std::string const name { std::getenv( "EU07_CARVE_DEBUG" ) != nullptr ? std::getenv( "EU07_CARVE_DEBUG" ) : "" };
    return name;
}

int blade_side( TTrack const &Track, int const Path ) {

    return ( Track.SwitchExtension->RightSwitch == ( Path == 0 ) ? 1 : -1 );
}

// a part of a rail between two cuts, as designed
struct carve_part {
    double from { -far_end }; // where the cuts put it
    double to { far_end };
    bool cut[ 2 ] { false, false }; // a cut at its start, at its end
    rail_end kinds[ 2 ] { rail_end::square, rail_end::square };
    double anchors[ 2 ] { 0.0, 0.0 };
    double runons[ 2 ] { 0.0, 0.0 };
    bool obtuse[ 2 ] { false, false };
    glm::dvec2 marks[ 2 ] { glm::dvec2 { 0.0 }, glm::dvec2 { 0.0 } };
    knots::line offsets; // as laid
    knots::line carve; // as carved
    knots::line rolled;
    bool throwable { false };
    bool carved { false };
    // kept out of the flangeways around: how far out and from how far in the head and the rest of the profile reach, out from the gauge face as carved
    bool trimmed { false };
    knots::line upper, lower;
    double trimfrom { 0.0 }; // where it's looked at for that, the lines say nothing past
    double trimto { -1.0 };
};

struct carve_rail {
    TTrack *track { nullptr };
    int path { 0 };
    int side { 0 };
    gauge_line const *line { nullptr };
    double length { 0.0 };
    double laidfrom { 0.0 };
    double heel { 0.0 }; // past the blades of both paths of a switch
    glm::dvec2 low { 0.0 };
    glm::dvec2 high { 0.0 };
    glm::dvec3 ends[ 2 ];
    glm::dvec2 directions[ 2 ];
    rail_size size;
    std::vector<carve_part> parts;
    double inmost { 0.0 }; // the least and the most the parts are carved out with
    double outmost { 0.0 };
};

std::string rail_name( carve_rail const &Rail ) {

    return Rail.track->name() + "/" + std::to_string( Rail.path ) + ( Rail.side > 0 ? "+1" : "-1" );
}

bool debugging( carve_rail const &Rail ) {

    return false == debugged().empty() && debugged() == rail_name( Rail );
}

struct carve_track {
    rail_size size;
    std::vector<carve_rail> rails;
    bool designed { false };
    bool planned { false }; // the layout designed here, as the track is laid out
    rail_layout layout;
    bool aroundknown { false };
    std::vector<TTrack *> around;
};

std::unordered_map<TTrack const *, carve_track> tracks;
unsigned int tracksstamp { 0 };

carve_track &track_of( TTrack &Track ) {

    if( tracksstamp != memo_stamp() ) {
        tracks.clear();
        tracksstamp = memo_stamp();
    }
    auto &track { tracks[ &Track ] };
    if( false == track.rails.empty() ) {
        return track;
    }
    track.size = Track.rail_dimensions();
    auto const heel { Track.eType == tt_Switch ? std::max( blade_length( Track, 0 ), blade_length( Track, 1 ) ) : 0.0 };
    for( int path = 0; path < path_count( Track ); ++path ) {
        for( auto const side : { -1, 1 } ) {
            carve_rail rail;
            rail.track = &Track;
            rail.path = path;
            rail.side = side;
            rail.line = &rail_line( Track, path, side > 0, false );
            rail.length = path_segment( Track, path ).GetLength();
            rail.laidfrom = ( Track.eType == tt_Switch ? ( side == blade_side( Track, path ) ? blade_length( Track, path ) : 0.0 ) : blade_runon( Track, side ) );
            rail.heel = heel;
            rail.low = rail.line->low;
            rail.high = rail.line->high;
            rail.size = track.size;
            for( int end = 0; end < 2; ++end ) {
                glm::dvec3 tangent;
                rail.line->at( end ? rail.length : 0.0, rail.ends[ end ], tangent );
                rail.directions[ end ] = glm::normalize( plan( tangent ) );
            }
            track.rails.push_back( rail );
        }
    }
    return track;
}

// the tracks whose rails the rails of the track are carved against: itself, the tracks crossing it and those going on from its ends
std::vector<TTrack *> const &around_of( TTrack &Track ) {

    auto &carved { track_of( Track ) };
    if( carved.aroundknown ) {
        return carved.around;
    }
    carved.aroundknown = true;
    std::vector<TTrack *> around;
    auto const add = [&]( TTrack *Other ) {
        if( Other == nullptr || std::find( around.begin(), around.end(), Other ) != around.end() || false == cut_rails( *Other ) ) {
            return;
        }
        if( false == track_of( *Other ).size.grooved ) {
            around.push_back( Other );
        } };
    add( &Track );
    // the rails crossing ours go on past their joints, near crossings at those too
    auto const tracks_on = [&]( TTrack *Other ) {
        add( Other );
        if( Other == nullptr ) {
            return;
        }
        for( int path = 0; path < path_count( *Other ); ++path ) {
            for( int end = 0; end < 2; ++end ) {
                add( path_neighbour( *Other, path, end ) );
            }
        } };
    for( auto const &crossing : Track.rail_crossings() ) {
        tracks_on( crossing.track );
        tracks_on( crossing.owner );
    }
    for( int path = 0; path < path_count( Track ); ++path ) {
        for( int end = 0; end < 2; ++end ) {
            add( path_neighbour( Track, path, end ) );
        }
    }
    carved.around = around;
    return carved.around;
}

// the parts of the rails of the track as designed, from the layout given or planned without carving
void design( TTrack &Track, rail_layout const *Layout ) {

    auto &carved { track_of( Track ) };
    if( carved.designed ) {
        return;
    }
    carved.designed = true;
    if( Layout == nullptr ) {
        carved.layout = Track.plan_rails( true, false );
        carved.planned = true;
        Layout = &carved.layout;
    }
    knots::line const none;
    for( auto &rail : carved.rails ) {
        auto cuts { Layout->plans[ rail.path ][ rail.side > 0 ].cuts };
        std::stable_sort( cuts.begin(), cuts.end(), []( rail_cut const &A, rail_cut const &B ) { return A.at < B.at; } );
        for( std::size_t j = 0; j <= cuts.size(); ++j ) {
            auto const *before { j > 0 ? &cuts[ j - 1 ] : nullptr };
            auto const *after { j < cuts.size() ? &cuts[ j ] : nullptr };
            carve_part part;
            if( before != nullptr ) {
                part.from = before->start;
                part.cut[ 0 ] = true;
                part.kinds[ 0 ] = before->startkind;
                part.anchors[ 0 ] = before->startanchor;
                part.runons[ 0 ] = before->startrunon;
                part.obtuse[ 0 ] = before->obtuse;
                part.marks[ 0 ] = before->mark;
                part.throwable = before->throwable;
            }
            if( after != nullptr ) {
                part.to = after->end;
                part.cut[ 1 ] = true;
                part.kinds[ 1 ] = after->endkind;
                part.anchors[ 1 ] = after->endanchor;
                part.runons[ 1 ] = after->endrunon;
                part.obtuse[ 1 ] = after->obtuse;
                part.marks[ 1 ] = after->mark;
            }
            part.carved = ( before != nullptr && before->carved ) || ( after != nullptr && after->carved );
            part.offsets = knots::sum( before ? before->startoffsets : none, after ? after->endoffsets : none );
            part.carve = ( part.throwable ? knots::sum( before->startcarve, after ? after->endoffsets : none ) : part.offsets );
            part.rolled = knots::bound( before ? before->startrolled : none, after ? after->endrolled : none, true );
            for( auto const &knot : part.carve ) {
                rail.inmost = std::min( rail.inmost, knot.y );
                rail.outmost = std::max( rail.outmost, knot.y );
            }
            rail.parts.push_back( part );
        }
    }
}

// the rail goes on from an end of the other one: the same rail past a joint, or a path of a switch going on from it
bool continues( carve_rail const &A, carve_rail const &B ) {

    if( A.track == B.track ) {
        return false;
    }
    for( int a = 0; a < 2; ++a ) {
        for( int b = 0; b < 2; ++b ) {
            if( glm::length( plan( A.ends[ a ] - B.ends[ b ] ) ) < 0.05 && std::abs( glm::dot( A.directions[ a ], B.directions[ b ] ) ) > 0.99 ) {
                return true;
            }
        }
    }
    return false;
}

// the wheels running along the other rail keep the head of ours out of its flangeway: not the wheels of our own path, nor those of a rail ours goes on
// from, nor of the other path of our own switch for a part moving with it
bool flangeway_counts( carve_rail const &Ours, carve_part const &Part, carve_rail const &Theirs ) {

    if( Theirs.track == Ours.track && ( Theirs.path == Ours.path || Part.throwable ) ) {
        return false;
    }
    return false == continues( Ours, Theirs );
}

// the heads of two parts share the room between them: not the parts of the rails of one path, nor of the rails going on from each other
bool bodies_count( carve_rail const &Ours, carve_rail const &Theirs ) {

    if( Theirs.track == Ours.track && Theirs.path == Ours.path ) {
        return false;
    }
    return false == continues( Ours, Theirs );
}

// nor a part moving with a switch and the wing rails of its own switch beside, nor two parts meeting at an obtuse crossing, which are designed to meet
bool bodies_count( carve_rail const &Ours, carve_part const &Part, carve_rail const &Theirs, carve_part const &Other ) {

    if( Theirs.track == Ours.track && Part.throwable != Other.throwable ) {
        return false;
    }
    for( int a = 0; a < 2; ++a ) {
        for( int b = 0; b < 2; ++b ) {
            if( Part.obtuse[ a ] && Other.obtuse[ b ] && glm::length( Part.marks[ a ] - Other.marks[ b ] ) < 0.1 ) {
                return false;
            }
        }
    }
    return true;
}

// where the gauge line of the rail lies at a distance along it, how it runs on and the rail outwards there: along the path from where it is
// across, past its ends as it runs on
void frame_of( carve_rail const &Rail, double const Distance, glm::dvec3 &Point, glm::dvec2 &Along, glm::dvec2 &Outwards ) {

    auto const &line { *Rail.line };
    if( Distance >= 0.0 && Distance <= line.segment->GetLength() ) {
        glm::dvec3 across;
        line.segment->plan_frame( Distance, Point, across );
        Point += across * line.offset;
        Outwards = plan( across ) * static_cast<double>( Rail.side );
        Along = { across.z, -across.x };
        return;
    }
    glm::dvec3 tangent;
    line.at( Distance, Point, tangent );
    Along = glm::normalize( plan( tangent ) );
    Outwards = -glm::normalize( plan( line.normal( Distance, Rail.side ) ) );
}

// how far out from the gauge line of the rail a point lies, where along it, from a distance near given; the rail outwards there, and on
double project( carve_rail const &Rail, glm::dvec3 const &Point, double &Distance, glm::dvec2 &Outwards, glm::dvec2 *Along = nullptr ) {

    glm::dvec3 point;
    glm::dvec2 along;
    for( int step = 0; step < 10; ++step ) {
        frame_of( Rail, Distance, point, along, Outwards );
        auto const ahead { glm::dot( plan( Point - point ), along ) };
        Distance += ahead;
        if( std::abs( ahead ) < 1e-6 ) {
            break;
        }
    }
    frame_of( Rail, Distance, point, along, Outwards );
    if( Along != nullptr ) {
        *Along = along;
    }
    return glm::dot( plan( Point - point ), Outwards );
}

struct station {
    bool known { false };
    glm::dvec3 point { 0.0 }; // on the gauge line
    glm::dvec2 outwards { 0.0 };
    double offset { 0.0 }; // as carved
    bool runon { false }; // past the end of a wing rail run on
};

glm::dvec3 across( station const &Station, double const Lateral ) {

    return { Station.point.x + Station.outwards.x * Lateral, Station.point.y, Station.point.z + Station.outwards.y * Lateral };
}

// the stations along a part where it's looked at
class part_scan {

public:
    part_scan( carve_rail const &Rail, carve_part const &Part ) :
        m_rail { Rail },
        m_part { Part } {
        auto const ahead { Part.cut[ 0 ] && Part.kinds[ 0 ] == rail_end::point ? Part.runons[ 0 ] : 0.0 };
        auto const behind { Part.cut[ 1 ] && Part.kinds[ 1 ] == rail_end::point ? Part.runons[ 1 ] : 0.0 };
        from = ( Part.cut[ 0 ] ? std::max( Part.from - ahead, -outside ) : Rail.laidfrom );
        to = ( Part.cut[ 1 ] ? std::min( Part.to + behind, Rail.length + outside ) : Rail.length );
        first = static_cast<long>( std::ceil( from / fine_step - 1e-9 ) );
        last = static_cast<long>( std::floor( to / fine_step + 1e-9 ) );
        if( last >= first ) {
            m_stations.resize( last - first + 1 );
        }
    }
    bool empty() const {
        return m_stations.empty(); }
    std::size_t count() const {
        return m_stations.size(); }
    static double position( long const K ) {
        return K * fine_step; }
    // the middle of the part as scanned, where its end nearer to it decides how it's planed
    double middle() const {
        return 0.5 * ( std::max( m_part.from, from ) + std::min( m_part.to, to ) ); }
    station frame( double const Distance ) const {
        station result;
        glm::dvec2 along;
        frame_of( m_rail, Distance, result.point, along, result.outwards );
        result.offset = offset( Distance, result.runon );
        result.known = true;
        return result; }
    station const &at( long const K ) {
        auto &result { m_stations[ K - first ] };
        if( false == result.known ) {
            result = frame( position( K ) );
        }
        return result; }
    // how far out the part is carved: past the end of a wing rail run on as it bends on, more
    double offset( double const Distance, bool &Runon ) const {
        auto const &line { m_part.carve };
        Runon = false;
        if( m_part.cut[ 1 ] && m_part.kinds[ 1 ] == rail_end::point && m_part.runons[ 1 ] > 0.0 && Distance > m_part.to ) {
            Runon = true;
            if( line.size() >= 2 && Distance > line.back().x ) {
                auto const &last { line.back() };
                auto const &previous { line[ line.size() - 2 ] };
                auto const course { ( last.y - previous.y ) / std::max( last.x - previous.x, 1e-6 ) };
                return last.y + ( course + runon_bend ) * ( Distance - last.x );
            }
        }
        else if( m_part.cut[ 0 ] && m_part.kinds[ 0 ] == rail_end::point && m_part.runons[ 0 ] > 0.0 && Distance < m_part.from ) {
            Runon = true;
            if( line.size() >= 2 && Distance < line.front().x ) {
                auto const &first { line.front() };
                auto const &next { line[ 1 ] };
                auto const course { ( next.y - first.y ) / std::max( next.x - first.x, 1e-6 ) };
                return first.y + ( runon_bend - course ) * ( first.x - Distance );
            }
        }
        return knots::sample( line, Distance, 0.0 ); }
    // the stations near the other rail: where the middle of the head of ours lies across its gauge line within what Across gives where along it,
    // along it within Along
    template <typename Reach>
    std::vector<std::pair<long, long>> stations_near( carve_rail const &Other, Reach const &Across, glm::dvec2 const &Along, double const Spread ) const {
        std::vector<std::pair<long, long>> result;
        if( empty() ) {
            return result;
        }
        auto const low { Other.low - Spread };
        auto const high { Other.high + Spread };
        auto distance { 0.0 };
        auto tracking { false };
        if( m_coarse.empty() ) {
            for( auto s { from }; ; s = std::min( s + coarse_step, to ) ) {
                auto const here { frame( s ) };
                m_coarse.push_back( { s, here.outwards, across( here, here.offset + 0.5 * m_rail.size.head ) } );
                if( s >= to ) {
                    break;
                }
            }
        }
        for( auto const &[ s, outwards_here, centre ] : m_coarse ) {
            auto const point { plan( centre ) };
            if( point.x < low.x || point.y < low.y || point.x > high.x || point.y > high.y ) {
                tracking = false;
            }
            else {
                if( false == tracking ) {
                    distance = nearest_distance( *Other.line, centre );
                    tracking = true;
                }
                glm::dvec2 outwards;
                auto const lateral { project( Other, centre, distance, outwards ) };
                auto const facing { glm::dot( outwards_here, outwards ) };
                // the stations between those looked at here lie off it this much more at most, running at an angle
                auto const margin { across_margin + coarse_step * std::sqrt( std::max( 0.0, 1.0 - facing * facing ) ) };
                if( distance < Along.x || distance > Along.y || std::abs( facing ) < cos_wide - 0.05 ) {
                    continue;
                }
                auto const across { Across( distance ) };
                if( lateral >= across.x - margin && lateral <= across.y + margin ) {
                    auto const a { std::max( first, static_cast<long>( std::floor( ( s - window ) / fine_step ) ) ) };
                    auto const b { std::min( last, static_cast<long>( std::ceil( ( s + window ) / fine_step ) ) ) };
                    if( false == result.empty() && a <= result.back().second + 1 ) {
                        result.back().second = std::max( result.back().second, b );
                    }
                    else if( a <= b ) {
                        result.emplace_back( a, b );
                    }
                }
            }
        }
        return result; }
    // a line of how far the profile reaches out from its gauge face as carved, or from where in, from how far out from the gauge line it can reach
    // at each station: Limit and past where it planes nothing
    // where a run of planing ends between stations, the line goes on as it ran out to Reach, where it stops planing what's left of the head
    knots::line local_line( std::vector<double> const &Values, bool const Upper, double const Limit, double const Clear, double const Reach ) {
        std::vector<double> local( Values.size(), Clear );
        std::vector<char> planed( Values.size(), 0 );
        for( std::size_t i = 0; i < Values.size(); ++i ) {
            if( std::isinf( Values[ i ] ) ) {
                continue;
            }
            auto const value { Values[ i ] - at( first + static_cast<long>( i ) ).offset };
            if( Upper ? value < Limit : value > Limit ) {
                local[ i ] = value;
                planed[ i ] = 1;
            }
        }
        knots::line line;
        auto const freed = [&]( std::size_t const I, std::size_t const Before, double const Direction ) {
            auto const slope { ( local[ I ] - local[ Before ] ) * Direction / fine_step };
            auto const left { Reach - local[ I ] };
            if( ( Upper ? slope > 0.0 && left > 0.0 : slope < 0.0 && left < 0.0 ) && left / slope < fine_step ) {
                line.emplace_back( position( first + static_cast<long>( I ) ) + Direction * left / slope, Reach );
            } };
        for( std::size_t i = 0; i < Values.size(); ++i ) {
            if( planed[ i ] && i > 0 && false == planed[ i - 1 ] && i + 1 < Values.size() && planed[ i + 1 ] ) {
                freed( i, i + 1, -1.0 );
            }
            if( planed[ i ] || ( i > 0 && planed[ i - 1 ] ) || ( i + 1 < Values.size() && planed[ i + 1 ] ) ) {
                line.emplace_back( position( first + static_cast<long>( i ) ), local[ i ] );
            }
            if( planed[ i ] && i > 0 && planed[ i - 1 ] && i + 1 < Values.size() && false == planed[ i + 1 ] ) {
                freed( i, i - 1, 1.0 );
            }
        }
        return knots::simplified( line, 0.5 * slack ); }
// members
    double from { 0.0 };
    double to { 0.0 };
    long first { 0 };
    long last { -1 };

private:
    struct coarse {
        double at;
        glm::dvec2 outwards;
        glm::dvec3 centre; // of the head
    };
    carve_rail const &m_rail;
    carve_part const &m_part;
    std::vector<station> m_stations;
    mutable std::vector<coarse> m_coarse; // the stations the rails around are looked for at
};

// how a bound made of a line and a limit runs on at a point: along the line where it bounds there, not across to where the other one does
template <typename Value>
double run_of( Value const &Bound, double const At ) {

    bool bounding, before, after;
    auto const at { Bound( At, bounding ) };
    auto const b { Bound( At - 0.5 * fine_step, before ) };
    auto const f { Bound( At + 0.5 * fine_step, after ) };
    if( before == after ) {
        return ( f - b ) / fine_step;
    }
    return ( before == bounding ? ( at - b ) : ( f - at ) ) / ( 0.5 * fine_step );
}

// how a run of stations along a part, where it runs into a flangeway or a head, planes it: from the side its head leaves that at the end of the
// run nearer the middle of the part; running into it all the way, from the side taking less of it where it takes least
template <typename Hit, typename Apply>
void settle_runs( std::vector<Hit> const &Hits, long const First, double const Middle, Apply const &Applied ) {

    auto const upper_at = [&]( std::size_t const I ) {
        return Hits[ I ].removed[ 0 ] <= Hits[ I ].removed[ 1 ]; };
    std::size_t i { 0 };
    while( i < Hits.size() ) {
        if( false == Hits[ i ].in ) {
            ++i;
            continue;
        }
        auto j { i };
        while( j + 1 < Hits.size() && Hits[ j + 1 ].in ) {
            ++j;
        }
        auto const leftstart { i > 0 };
        auto const leftend { j + 1 < Hits.size() };
        bool upper;
        if( leftstart && leftend ) {
            auto const a { part_scan::position( First + static_cast<long>( i ) ) };
            auto const b { part_scan::position( First + static_cast<long>( j ) ) };
            upper = upper_at( std::abs( a - Middle ) <= std::abs( b - Middle ) ? i : j );
        }
        else if( leftstart || leftend ) {
            upper = upper_at( leftstart ? i : j );
        }
        else {
            auto least { i };
            for( auto k { i }; k <= j; ++k ) {
                if( std::min( Hits[ k ].removed[ 0 ], Hits[ k ].removed[ 1 ] ) < std::min( Hits[ least ].removed[ 0 ], Hits[ least ].removed[ 1 ] ) ) {
                    least = k;
                }
            }
            upper = upper_at( least );
        }
        for( auto k { i }; k <= j; ++k ) {
            Applied( k, upper );
        }
        i = j + 1;
    }
}

// the head of ours in the flangeway of another rail, out from our gauge line: where it'd have to end, or start, to keep out
struct band_hit {
    bool in { false };
    double upper { 0.0 };
    double lower { 0.0 };
    double removed[ 2 ] {};
};

band_hit band_at( station const &Station, carve_rail const &Ours, carve_rail const &Theirs, double &Distance ) {

    band_hit hit;
    auto const centre { Station.offset + 0.5 * Ours.size.head };
    glm::dvec2 outwards;
    auto const lateral { project( Theirs, across( Station, centre ), Distance, outwards ) };
    if( Distance < 0.0 || Distance > Theirs.length ) {
        return hit;
    }
    // where the blades of a switch are, the rails of both paths on a side lie at each other as they're made to
    if( Theirs.track == Ours.track && Theirs.side == Ours.side && Distance < Theirs.heel + heel_margin ) {
        return hit;
    }
    auto const facing { glm::dot( Station.outwards, outwards ) };
    if( std::abs( facing ) < cos_wide ) {
        return hit;
    }
    auto const a { centre + ( -flangeway - lateral ) / facing };
    auto const b { centre - lateral / facing };
    auto const low { std::min( a, b ) };
    auto const high { std::max( a, b ) };
    auto const head0 { Station.offset };
    auto const head1 { Station.offset + Ours.size.head };
    if( head1 <= low + slack || head0 >= high - slack ) {
        return hit;
    }
    hit.in = true;
    hit.upper = low;
    hit.lower = high;
    hit.removed[ 0 ] = head1 - low;
    hit.removed[ 1 ] = high - head0;
    return hit;
}

// the part kept out of the flangeways of the rails around
void trim( carve_rail &Rail, carve_part &Part ) {

    if( Part.trimmed ) {
        return;
    }
    Part.trimmed = true;
    if( false == Part.carved ) {
        return;
    }
    part_scan scan { Rail, Part };
    if( scan.empty() ) {
        return;
    }
    auto const &size { Rail.size };
    std::vector<double> upper( scan.count(), unbounded );
    std::vector<double> lower( scan.count(), -unbounded );
    auto const around { around_of( *Rail.track ) };
    for( auto *other : around ) {
        for( auto &theirs : track_of( *other ).rails ) {
            if( false == flangeway_counts( Rail, Part, theirs ) ) {
                continue;
            }
            auto const windows { scan.stations_near( theirs, [&]( double ) { return glm::dvec2 { -flangeway - 0.5 * size.head, 0.5 * size.head }; }, { -nearby, theirs.length + nearby }, flangeway + size.head + nearby ) };
            for( auto const &[ a, b ] : windows ) {
                if( debugging( Rail ) ) {
                    WriteLog( "CARVE F " + rail_name( Rail ) + " part " + std::to_string( Part.from ) + "-" + std::to_string( Part.to ) + " vs " + rail_name( theirs ) + " window " + std::to_string( part_scan::position( a ) ) + "-" + std::to_string( part_scan::position( b ) ) );
                }
                std::vector<band_hit> hits;
                auto distance { nearest_distance( *theirs.line, across( scan.at( a ), scan.at( a ).offset ) ) };
                for( auto k { a }; k <= b; ++k ) {
                    hits.push_back( band_at( scan.at( k ), Rail, theirs, distance ) );
                }
                settle_runs( hits, a, scan.middle(), [&]( std::size_t const I, bool const Upper ) {
                    auto const index { static_cast<std::size_t>( a - scan.first ) + I };
                    if( debugging( Rail ) ) {
                        WriteLog( "CARVE F   at " + std::to_string( part_scan::position( a + static_cast<long>( I ) ) ) + ( Upper ? " upper " : " lower " ) + std::to_string( Upper ? hits[ I ].upper : hits[ I ].lower ) + " offset " + std::to_string( scan.at( a + static_cast<long>( I ) ).offset ) );
                    }
                    if( Upper ) {
                        upper[ index ] = std::min( upper[ index ], hits[ I ].upper );
                    }
                    else {
                        lower[ index ] = std::max( lower[ index ], hits[ I ].lower );
                    } } );
            }
        }
    }
    Part.upper = scan.local_line( upper, true, size.high, size.high + 0.01, size.head );
    Part.lower = scan.local_line( lower, false, size.low, size.low - 0.01, 0.0 );
    Part.trimfrom = scan.from;
    Part.trimto = scan.to;
}

// the head of ours lying over the head of another part, out from our gauge line
struct body_hit {
    bool in { false };
    bool heads { false }; // or only the feet of two rails back to back
    bool faces { false }; // the feet of two rails face to face across a flangeway, ourfoot and theirfoot how far in they reach
    double removed[ 2 ] {};
    glm::dvec2 ours { 0.0 };
    glm::dvec2 theirs { 0.0 };
    glm::dvec2 ourneed { 1.0, 0.0 }; // what of each the wheels of its own path roll on, none where the start is past the end
    glm::dvec2 theirneed { 1.0, 0.0 };
    double facing { 0.0 };
    double ourshoulder { 0.0 };
    double theirshoulder { 0.0 };
    double ourfoot { 0.0 }; // how far out the profile of ours reaches, and from where theirs does, back to back
    double theirfoot { 0.0 };
    double ourcrown { 0.0 }; // where the top of the head of each ends on its outer side, the rounded shoulder past it
    double theircrown { 0.0 };
    bool rolledon { false }; // the wheels of theirs roll on, out to
    double theirrolled { 0.0 };
    // how square the bounds of the heads run to ours: what of a step out across ours is a step away from them
    glm::dvec2 ourslant { 1.0 };
    glm::dvec2 theirslant { 1.0 };
    glm::dvec2 footslant { 1.0 }; // of ours and theirs
};

// the bounds of ours where its head lies under theirs, Above, or over it: the two meet in the middle of where they lie over each other, as far as
// the wheels of each leave room; a head giving way runs on past the meeting over the rounded shoulder of the other one, facing it with its outer edge.
// Two rails back to back meet with their feet in the middle between them too, and the wheels of theirs roll on ours planed flush against it;
// two face to face, as a wing rail and the leg of the nose, with their feet in the middle of the flangeway, the heads left whole
void settle( body_hit const &Hit, bool const Above, double &Upper, double &Lower, double &Worn ) {

    auto const slant { std::max( std::abs( Hit.facing ), cos_wide ) };
    auto const needed = []( glm::dvec2 const &Need ) {
        return Need.x <= Need.y; };
    // a bound across ours and how square it runs to ours: the lines along ours run square, those along theirs slanted
    struct bound {
        double at;
        double slant;
    };
    auto const higher = []( bound const &A, bound const &B ) {
        return A.at >= B.at ? A : B; };
    auto const lower = []( bound const &A, bound const &B ) {
        return A.at <= B.at ? A : B; };
    auto const clamped = [&]( bound const &Value, bound const &Low, bound const &High ) {
        return lower( higher( Value, Low ), High ); };
    // as far from both bounds, whichever way they run: the same line from either rail
    auto const between = []( bound const &A, bound const &B ) {
        return bound { ( A.at * A.slant + B.at * B.slant ) / ( A.slant + B.slant ), 1.0 }; };
    if( Hit.faces ) {
        auto const feet { between( { Hit.ourfoot, Hit.footslant.x }, { Hit.theirfoot, Hit.footslant.y } ).at };
        Lower = std::max( Lower, std::min( feet + slack, Hit.ours.x - slack ) );
        return;
    }
    auto const a { higher( { Hit.ours.x, Hit.ourslant.x }, { Hit.theirs.x, Hit.theirslant.x } ) };
    auto const b { lower( { Hit.ours.y, Hit.ourslant.y }, { Hit.theirs.y, Hit.theirslant.y } ) };
    auto const overlap { Hit.heads && b.at > a.at };
    auto const middle { between( a, b ).at };
    auto const half { std::max( 0.5 * ( b.at - a.at ), 1e-9 ) };
    // where the wheels of each need the head, run along ours, along theirs, or the bounds of the heads
    auto const ourneed = [&]( double const Need, double const Bound, double const Slant ) {
        return bound { Need, Need == Bound ? Slant : 1.0 }; };
    auto const theirneed = [&]( double const Need, double const Bound, double const Slant ) {
        return bound { Need, Need == Bound ? Slant : slant }; };
    if( Above ) {
        auto ouredge { Hit.ours.y };
        auto theiredge { Hit.theirs.x };
        if( overlap ) {
            auto const ours { needed( Hit.ourneed ) && Hit.ourneed.y > a.at ? clamped( ourneed( Hit.ourneed.y, Hit.ours.y, Hit.ourslant.y ), a, b ) : a };
            auto const theirs { needed( Hit.theirneed ) && Hit.theirneed.x < b.at ? clamped( theirneed( Hit.theirneed.x, Hit.theirs.x, Hit.theirslant.x ), a, b ) : b };
            auto const meet { ours.at <= theirs.at ? std::clamp( middle, ours.at, theirs.at ) : between( ours, theirs ).at };
            auto const given { ( meet - middle ) / half }; // below zero ours gives way
            // over the shoulder of the other head only, never past it into the head
            auto const edge { given < 0.0 && Hit.facing < 0.0 ? std::max( meet, std::min( meet - given * Hit.theirshoulder / slant, Hit.theircrown ) ) : meet };
            Upper = std::min( Upper, edge - slack );
            ouredge = std::min( ouredge, edge );
            theiredge = std::max( theiredge, given > 0.0 ? std::min( meet, std::max( meet - given * Hit.ourshoulder, Hit.ourcrown ) ) : meet );
        }
        if( Hit.facing < 0.0 && Hit.ourfoot > Hit.theirfoot + slack ) {
            auto const feet { between( { Hit.ourfoot, Hit.footslant.x }, { Hit.theirfoot, Hit.footslant.y } ).at };
            Upper = std::min( Upper, std::max( ouredge, std::min( feet, theiredge ) ) - slack );
        }
        if( Hit.facing < 0.0 && Hit.rolledon && std::min( Hit.ours.y, Upper ) > Hit.theirrolled + slack ) {
            Worn = std::min( Worn, Hit.theirrolled );
        }
    }
    else if( overlap ) {
        auto const ours { needed( Hit.ourneed ) && Hit.ourneed.x < b.at ? clamped( ourneed( Hit.ourneed.x, Hit.ours.x, Hit.ourslant.x ), a, b ) : b };
        auto const theirs { needed( Hit.theirneed ) && Hit.theirneed.y > a.at ? clamped( theirneed( Hit.theirneed.y, Hit.theirs.y, Hit.theirslant.y ), a, b ) : a };
        auto const meet { theirs.at <= ours.at ? std::clamp( middle, theirs.at, ours.at ) : between( ours, theirs ).at };
        auto const given { ( meet - middle ) / half }; // above zero ours gives way
        auto const edge { given > 0.0 && Hit.facing > 0.0 ? std::min( meet, std::max( meet - given * Hit.theirshoulder / slant, Hit.theircrown ) ) : meet };
        Lower = std::max( Lower, edge + slack );
    }
}

struct carved_lines {
    knots::line offsets, extents, inner, rolled, worn;
    double from { 0.0 };
    double to { 0.0 };
    material_handle wornby { null_handle };
};

// the part carved against the heads of the parts around, its ends put where they're declared to be
carved_lines carve( carve_rail &Rail, carve_part &Part ) {

    carved_lines result { Part.offsets, {}, {}, Part.rolled, {}, Part.from, Part.to, null_handle };
    trim( Rail, Part );
    part_scan scan { Rail, Part };
    if( scan.empty() ) {
        return result;
    }
    auto const &size { Rail.size };
    auto const clearup { size.high + 0.01 };
    auto const clearlow { size.low - 0.01 };
    std::vector<double> upper( scan.count(), unbounded );
    std::vector<double> lower( scan.count(), -unbounded );
    std::vector<double> worn( scan.count(), unbounded );
    std::vector<material_handle> wornby( scan.count(), null_handle );
    auto const around { around_of( *Rail.track ) };
    for( auto *other : around ) {
        design( *other, nullptr );
        for( auto &theirs : track_of( *other ).rails ) {
            if( &theirs == &Rail || false == bodies_count( Rail, theirs ) ) {
                continue;
            }
            auto const &theirsize { theirs.size };
            auto const reach { std::max( std::abs( theirs.inmost ), std::abs( theirs.outmost ) ) + theirsize.high + nearby + 0.1 };
            // the blades of a switch are in the way as they lie closed, the stock rails of their own switch aside
            // where the parts of theirs lie, as they bend out on to the stations around
            auto const spread = [&]( double const Distance ) {
                glm::dvec2 offsets { unbounded, -unbounded };
                for( auto const &part : theirs.parts ) {
                    if( Distance < part.from - coarse_step || Distance > part.to + coarse_step ) {
                        continue;
                    }
                    for( auto const at : { Distance - coarse_step, Distance, Distance + coarse_step } ) {
                        auto const offset { knots::sample( part.carve, at, 0.0 ) };
                        offsets = { std::min( offsets.x, offset ), std::max( offsets.y, offset ) };
                    }
                }
                if( offsets.x > offsets.y ) {
                    offsets = { 0.0, 0.0 };
                }
                return glm::dvec2 { offsets.x + theirsize.low - 0.1, offsets.y + theirsize.high + 0.1 }; };
            auto const windows { scan.stations_near( theirs, spread, { -nearby, theirs.length + nearby }, reach ) };
            for( auto const &[ a, b ] : windows ) {
                if( debugging( Rail ) ) {
                    WriteLog( "CARVE O " + rail_name( Rail ) + " part " + std::to_string( Part.from ) + "-" + std::to_string( Part.to ) + " vs " + rail_name( theirs ) + " window " + std::to_string( part_scan::position( a ) ) + "-" + std::to_string( part_scan::position( b ) ) );
                }
                std::vector<std::vector<body_hit>> hits( theirs.parts.size(), std::vector<body_hit>( static_cast<std::size_t>( b - a + 1 ) ) );
                auto distance { nearest_distance( *theirs.line, across( scan.at( a ), scan.at( a ).offset ) ) };
                for( auto k { a }; k <= b; ++k ) {
                    auto const &here { scan.at( k ) };
                    auto const s { part_scan::position( k ) };
                    auto const up { knots::sample( Part.upper, s, clearup ) };
                    auto const low { knots::sample( Part.lower, s, clearlow ) };
                    glm::dvec2 const ours { here.offset + std::max( 0.0, low ), here.offset + std::min( size.head, up ) };
                    if( ours.y - ours.x < slack ) {
                        continue;
                    }
                    // how the bounds of ours run on, along the lines bounding them here
                    auto const ourbound = [&]( knots::line const &Line, double const Clear, double const Reach, bool const Upper ) {
                        return [&, Clear, Reach, Upper]( double const At, bool &Bounding ) {
                            bool runon;
                            auto const value { knots::sample( Line, At, Clear ) };
                            Bounding = ( Upper ? value < Reach : value > Reach );
                            return scan.offset( At, runon ) + ( Bounding ? value : Reach ); }; };
                    auto const square = []( double const Slope ) {
                        return 1.0 / std::sqrt( 1.0 + Slope * Slope ); };
                    auto const centre { here.offset + 0.5 * size.head };
                    glm::dvec2 outwards, along;
                    auto const lateral { project( theirs, across( here, centre ), distance, outwards, &along ) };
                    auto const facing { glm::dot( here.outwards, outwards ) };
                    if( std::abs( facing ) < cos_wide ) {
                        continue;
                    }
                    // a step out across ours goes this far along theirs, too: where a line of theirs running slanted meets ours, out from it
                    auto const drift { glm::dot( here.outwards, along ) };
                    auto const mapped = [&]( double const V ) {
                        return centre + ( V - lateral ) / facing; };
                    auto const meets = [&]( auto const &Line ) {
                        bool bounding;
                        auto at { mapped( Line( distance, bounding ) ) };
                        for( int step = 0; step < 6; ++step ) {
                            auto const next { mapped( Line( distance + ( at - centre ) * drift, bounding ) ) };
                            auto const done { std::abs( next - at ) < 1e-5 };
                            at = next;
                            if( done ) {
                                break;
                            }
                        }
                        return at; };
                    // and how square to ours a line of theirs running on there is, where it meets ours At
                    auto const slant = [&]( auto const &Line, double const At ) {
                        auto const slope { run_of( Line, distance + ( At - centre ) * drift ) };
                        return std::abs( facing - slope * drift ) / std::sqrt( 1.0 + slope * slope ); };
                    auto const route { distance >= 0.0 && distance <= theirs.length };
                    // how the bounds of ours run, once something of theirs is in their way
                    glm::dvec2 ourslant { 1.0 };
                    auto footslant { 1.0 };
                    auto innerslant { 1.0 };
                    auto slanted { false };
                    if( theirs.track == Rail.track && theirs.side == Rail.side && distance < theirs.heel + heel_margin ) {
                        continue;
                    }
                    for( std::size_t j = 0; j < theirs.parts.size(); ++j ) {
                        auto &part { theirs.parts[ j ] };
                        if( distance < std::max( part.from, 0.0 ) || distance > std::min( part.to, theirs.length ) || false == bodies_count( Rail, Part, theirs, part ) ) {
                            continue;
                        }
                        trim( theirs, part );
                        // the head of theirs and how far its profile reaches, out from its gauge line: the bound Which (from, to, out to, the
                        // crown, in to) and how it runs on
                        auto const trimmed { distance >= part.trimfrom && distance <= part.trimto };
                        auto const theirclearup { theirsize.high + 0.01 };
                        auto const theirclearlow { theirsize.low - 0.01 };
                        auto const bound = [&]( int const Which ) {
                            return [&, Which]( double const At, bool &Bounding ) {
                                auto const offset { knots::sample( part.carve, At, 0.0 ) };
                                if( Which == 0 || Which == 4 ) {
                                    auto const reach { Which == 0 ? 0.0 : theirsize.low };
                                    auto const theirlow { trimmed ? knots::sample( part.lower, At, theirclearlow ) : theirclearlow };
                                    Bounding = theirlow > reach;
                                    return offset + std::max( reach, theirlow );
                                }
                                auto const reach { Which == 1 ? theirsize.head : Which == 2 ? theirsize.high : theirsize.head - theirsize.shoulder };
                                auto const theirup { trimmed ? knots::sample( part.upper, At, theirclearup ) : theirclearup };
                                Bounding = theirup < reach;
                                return offset + std::min( reach, theirup ); }; };
                        bool bounding;
                        auto const v0 { bound( 0 )( distance, bounding ) };
                        auto const v1 { bound( 1 )( distance, bounding ) };
                        if( v1 - v0 < slack ) {
                            continue;
                        }
                        // the same rail laid along another path: both lie as one
                        {
                            auto const theiroffset { knots::sample( part.carve, distance, 0.0 ) };
                            if( facing > 0.99 && std::abs( mapped( theiroffset ) - here.offset ) < 2.0 * slack && std::abs( mapped( theiroffset + theirsize.head ) - ( here.offset + size.head ) ) < 2.0 * slack ) {
                                continue;
                            }
                        }
                        auto const m0 { meets( bound( 0 ) ) };
                        auto const m1 { meets( bound( 1 ) ) };
                        glm::dvec2 const heads { std::min( m0, m1 ), std::max( m0, m1 ) };
                        auto const overlap { std::min( ours.y, heads.y ) - std::max( ours.x, heads.x ) };
                        auto const above { heads.x + heads.y > ours.x + ours.y };
                        auto const backs { facing < 0.0 && above };
                        auto const faces { facing < 0.0 && false == above && overlap <= 2.0 * slack };
                        auto const ourfoot { faces ? here.offset + std::max( size.low, low ) : here.offset + std::min( size.high, up ) };
                        auto const theirfoot { faces ? meets( bound( 4 ) ) : ( overlap > 2.0 * slack || backs ) ? meets( bound( 2 ) ) : 0.0 };
                        auto const feet { faces ? theirfoot > ourfoot + 2.0 * slack : backs && ourfoot > theirfoot + 2.0 * slack };
                        if( overlap <= 2.0 * slack && false == feet ) {
                            continue;
                        }
                        if( false == slanted ) {
                            ourslant = { square( run_of( ourbound( Part.lower, clearlow, 0.0, false ), s ) ), square( run_of( ourbound( Part.upper, clearup, size.head, true ), s ) ) };
                            footslant = square( run_of( ourbound( Part.upper, clearup, size.high, true ), s ) );
                            innerslant = square( run_of( ourbound( Part.lower, clearlow, size.low, false ), s ) );
                            slanted = true;
                        }
                        auto &hit { hits[ j ][ static_cast<std::size_t>( k - a ) ] };
                        hit.ours = ours;
                        hit.theirs = heads;
                        hit.ourslant = ourslant;
                        hit.theirslant = ( m0 <= m1 ? glm::dvec2 { slant( bound( 0 ), m0 ), slant( bound( 1 ), m1 ) } : glm::dvec2 { slant( bound( 1 ), m1 ), slant( bound( 0 ), m0 ) } );
                        hit.facing = facing;
                        hit.ourshoulder = size.shoulder;
                        hit.theirshoulder = theirsize.shoulder;
                        hit.ourfoot = ourfoot;
                        hit.theirfoot = theirfoot;
                        hit.footslant = ( faces ? glm::dvec2 { innerslant, slant( bound( 4 ), theirfoot ) } : glm::dvec2 { footslant, slant( bound( 2 ), theirfoot ) } );
                        hit.faces = faces;
                        hit.ourcrown = here.offset + std::min( size.head - size.shoulder, up );
                        hit.theircrown = meets( bound( 3 ) );
                        hit.heads = ( overlap > 2.0 * slack );
                        hit.in = true;
                        if( hit.heads ) {
                            auto const a { std::max( ours.x, hit.theirs.x ) };
                            auto const b { std::min( ours.y, hit.theirs.y ) };
                            hit.removed[ 0 ] = ours.y - a;
                            hit.removed[ 1 ] = b - ours.x;
                        }
                        else {
                            hit.removed[ 0 ] = std::abs( hit.ourfoot - hit.theirfoot );
                            hit.removed[ 1 ] = unbounded;
                        }
                        if( false == here.runon ) {
                            hit.ourneed = { std::max( 0.0, ours.x ), std::min( rolled_band, ours.y ) };
                        }
                        if( route ) {
                            auto const n0 { mapped( 0.0 ) };
                            auto const n1 { mapped( rolled_band ) };
                            hit.theirneed = { std::max( std::min( n0, n1 ), hit.theirs.x ), std::min( std::max( n0, n1 ), hit.theirs.y ) };
                            hit.rolledon = true;
                            hit.theirrolled = n1;
                        }
                    }
                }
                for( std::size_t j = 0; j < hits.size(); ++j ) {
                    auto const &parthits { hits[ j ] };
                    settle_runs( parthits, a, scan.middle(), [&]( std::size_t const I, bool const Above ) {
                        auto const index { static_cast<std::size_t>( a - scan.first ) + I };
                        // heads back to back lie over each other only with their gauge faces apart, each planed off its outer side
                        auto const was { worn[ index ] };
                        settle( parthits[ I ], Above || parthits[ I ].facing < 0.0, upper[ index ], lower[ index ], worn[ index ] );
                        if( worn[ index ] < was ) {
                            wornby[ index ] = theirs.track->rail_material( theirs.path );
                        }
                        if( debugging( Rail ) ) {
                            auto const &hit { parthits[ I ] };
                            WriteLog( "CARVE O   at " + std::to_string( part_scan::position( a + static_cast<long>( I ) ) ) + " their part " + std::to_string( j ) + ( Above || hit.facing < 0.0 ? " above" : " below" ) + ( hit.heads ? " heads" : " feet" )
                                + " ours " + std::to_string( hit.ours.x ) + ".." + std::to_string( hit.ours.y ) + " theirs " + std::to_string( hit.theirs.x ) + ".." + std::to_string( hit.theirs.y )
                                + " need " + std::to_string( hit.ourneed.x ) + ".." + std::to_string( hit.ourneed.y ) + "/" + std::to_string( hit.theirneed.x ) + ".." + std::to_string( hit.theirneed.y )
                                + " facing " + std::to_string( hit.facing ) + " slants " + std::to_string( hit.ourslant.x ) + "," + std::to_string( hit.ourslant.y ) + "/" + std::to_string( hit.theirslant.x ) + "," + std::to_string( hit.theirslant.y ) + " -> upper " + std::to_string( upper[ index ] ) + " lower " + std::to_string( lower[ index ] ) );
                        } } );
                }
            }
        }
    }
    auto extents { knots::bound( Part.upper, scan.local_line( upper, true, size.high, clearup, size.head ), true ) };
    auto inner { knots::bound( Part.lower, scan.local_line( lower, false, size.low, clearlow, 0.0 ), false ) };
    auto const wornline { scan.local_line( worn, true, size.head, clearup, size.head ) };
    auto const width = [&]( double const S ) {
        return std::min( size.head, knots::sample( extents, S, clearup ) ) - std::max( 0.0, knots::sample( inner, S, clearlow ) ); };
    // where the head gets to Width first, going from Start in Step, between stations; from nothing of it at Start, Open, where the rails cross
    // and the flangeway of the other one only just reaches the gauge face
    auto const reaches = [&]( double const Start, int const Step, double const Width, bool const Wider, double &At, bool const Open = false ) {
        auto const first { Start + ( Open ? Step * 0.005 : 0.0 ) };
        auto k { Step > 0 ? static_cast<long>( std::ceil( first / fine_step - 1e-9 ) ) : static_cast<long>( std::floor( first / fine_step + 1e-9 ) ) };
        auto previous { Start };
        auto before { Open ? 0.0 : width( Start ) };
        if( Wider ? before >= Width : before < Width ) {
            At = Start;
            return true;
        }
        for( ; k >= scan.first && k <= scan.last; k += Step ) {
            auto const s { part_scan::position( k ) };
            auto const now { width( s ) };
            if( Wider ? now >= Width : now < Width ) {
                At = glm::mix( previous, s, std::clamp( ( Width - before ) / ( now - before ), 0.0, 1.0 ) );
                return true;
            }
            previous = s;
            before = now;
        }
        return false; };
    auto from { Part.from };
    auto to { Part.to };
    auto faulty { false };
    for( int end = 0; end < 2; ++end ) {
        if( false == Part.cut[ end ] ) {
            continue;
        }
        auto const step { end ? -1 : 1 }; // into the part
        auto &at { end ? to : from };
        auto const nominal { at };
        switch( Part.kinds[ end ] ) {
            case rail_end::point: {
                // run on past its own end only into the head of another rail, not along a flangeway
                auto const anchor { std::clamp( Part.anchors[ end ], scan.from, scan.to ) };
                auto found { 0.0 };
                if( reaches( anchor, -step, point_width, false, found ) ) {
                    auto const inside { ( found - nominal ) * step >= 0.0 };
                    auto const kept { std::min( size.head, knots::sample( Part.upper, found, clearup ) ) - std::max( 0.0, knots::sample( Part.lower, found, clearlow ) ) };
                    if( inside || kept >= 3.0 * point_width ) {
                        at = found;
                    }
                }
                break;
            }
            case rail_end::blunt: {
                auto found { 0.0 };
                if( nominal >= scan.from && nominal <= scan.to ) {
                    if( reaches( nominal, step, blunt_width, true, found, true ) ) {
                        at = found;
                    }
                    else {
                        faulty = true;
                    }
                }
                break;
            }
            default: {
                break;
            }
        }
    }
    // a wing rail run on past its own end into what planes it to its point
    if( Part.cut[ 1 ] && to > Part.to + 1e-6 ) {
        bool runon;
        auto const out { scan.offset( to, runon ) };
        result.offsets.emplace_back( to, out );
        result.rolled.emplace_back( to, rolled_band - out );
    }
    if( Part.cut[ 0 ] && from < Part.from - 1e-6 ) {
        bool runon;
        auto const out { scan.offset( from, runon ) };
        result.offsets.insert( result.offsets.begin(), { from, out } );
        result.rolled.insert( result.rolled.begin(), { from, rolled_band - out } );
    }
    knots::sort( result.offsets );
    knots::sort( result.rolled );
    // nothing left of the head somewhere along the part, past its sharp ends getting wider
    auto start { std::max( from, Rail.laidfrom ) + 2.0 * fine_step };
    auto end { std::min( to, Rail.length ) - 2.0 * fine_step };
    for( int side = 0; side < 2; ++side ) {
        if( Part.cut[ side ] && Part.kinds[ side ] == rail_end::sharp ) {
            auto found { 0.0 };
            if( reaches( side ? to : from, side ? -1 : 1, point_width, true, found ) ) {
                ( side ? end : start ) = found + ( side ? -2.0 : 2.0 ) * fine_step;
            }
        }
    }
    for( auto k { static_cast<long>( std::ceil( start / fine_step ) ) }; k <= static_cast<long>( std::floor( end / fine_step ) ) && false == faulty; ++k ) {
        faulty = ( width( part_scan::position( k ) ) < point_width );
    }
    if( faulty ) {
        ++faults;
    }
    if( debugging( Rail ) ) {
        std::string names;
        for( auto *other : around ) {
            names += " " + other->name();
        }
        WriteLog( "CARVE " + rail_name( Rail ) + " part " + std::to_string( Part.from ) + "-" + std::to_string( Part.to ) + " scanned " + std::to_string( scan.from ) + "-" + std::to_string( scan.to ) + " ends " + std::to_string( from ) + "-" + std::to_string( to ) + ( faulty ? " FAULTY" : "" ) + " around" + names );
    }
    result.from = from;
    result.to = to;
    result.extents = knots::simplified( extents, 0.5 * slack );
    result.inner = knots::simplified( inner, 0.5 * slack );
    result.worn = wornline;
    // the wheels of the rails beside polish the head as bright as the brightest of them
    auto const band { Rail.track->tread_band() };
    for( auto const material : wornby ) {
        if( material != null_handle && material != result.wornby
         && ( result.wornby == null_handle || tread_brightness( material, band ) > tread_brightness( result.wornby, band ) ) ) {
            result.wornby = material;
        }
    }
    return result;
}

} // namespace

namespace track_rails {

void carve_plans( TTrack &Track, rail_layout &Layout ) {

    if( false == cut_rails( Track ) ) {
        return;
    }
    auto any { false };
    for( auto &paths : Layout.plans ) {
        for( auto &plan : paths ) {
            std::stable_sort( plan.cuts.begin(), plan.cuts.end(), []( rail_cut const &A, rail_cut const &B ) { return A.at < B.at; } );
            any = any || std::any_of( plan.cuts.begin(), plan.cuts.end(), []( rail_cut const &Cut ) { return Cut.carved; } );
        }
    }
    if( false == any || track_of( Track ).size.grooved ) {
        return;
    }
    design( Track, &Layout );
    for( auto &rail : track_of( Track ).rails ) {
        auto &cuts { Layout.plans[ rail.path ][ rail.side > 0 ].cuts };
        if( rail.parts.size() != cuts.size() + 1 ) {
            continue;
        }
        for( std::size_t j = 0; j < rail.parts.size(); ++j ) {
            auto &part { rail.parts[ j ] };
            if( false == part.carved ) {
                continue;
            }
            auto const lines { carve( rail, part ) };
            auto *before { j > 0 && cuts[ j - 1 ].carved ? &cuts[ j - 1 ] : nullptr };
            auto *after { j < cuts.size() && cuts[ j ].carved ? &cuts[ j ] : nullptr };
            if( before != nullptr ) {
                before->start = lines.from;
                before->startoffsets = lines.offsets;
                before->startextents = lines.extents;
                before->startinner = lines.inner;
                before->startrolled = lines.rolled;
                before->startworn = lines.worn;
                before->startwornby = lines.wornby;
            }
            if( after != nullptr ) {
                after->end = lines.to;
                after->endoffsets = lines.offsets;
                after->endextents = lines.extents;
                after->endinner = lines.inner;
                after->endrolled = lines.rolled;
                after->endworn = lines.worn;
                after->endwornby = lines.wornby;
            }
        }
    }
}

rail_layout const *designed_layout( TTrack &Track ) {

    auto const &carved { track_of( Track ) };
    return ( carved.planned ? &carved.layout : nullptr );
}

int carve_faults() {

    return faults;
}

} // namespace track_rails
