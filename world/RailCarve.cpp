/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"

#include <format>
#include <span>

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

// how many parts carving left with nothing of the head somewhere along them, since the start
int &fault_count() {

    static int faults { 0 };
    return faults;
}

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

// where the rail of the path on the side is laid from: on a switch past its blade on that side, on a plain track past a blade run on into it
double laid_from( TTrack const &Track, int const Path, int const Side ) {

    if( Track.eType != tt_Switch ) {
        return blade_runon( Track, Side );
    }
    return ( Side == blade_side( Track, Path ) ? blade_length( Track, Path ) : 0.0 );
}

// a part of a rail between two cuts, as designed
struct carve_part {
    double from { -far_end }; // where the cuts put it
    double to { far_end };
    std::array<bool, 2> cut { false, false }; // a cut at its start, at its end
    std::array<rail_end, 2> kinds { rail_end::square, rail_end::square };
    std::array<double, 2> anchors { 0.0, 0.0 };
    std::array<double, 2> runons { 0.0, 0.0 };
    std::array<bool, 2> obtuse { false, false };
    std::array<glm::dvec2, 2> marks { glm::dvec2 { 0.0 }, glm::dvec2 { 0.0 } };
    knots::line offsets; // as laid
    knots::line carve; // as carved
    knots::line rolled;
    bool throwable { false };
    bool carved { false };
    // kept out of the flangeways around: how far out and from how far in the head and the rest of the profile reach, out from the gauge face as carved
    bool trimmed { false };
    knots::line upper;
    knots::line lower;
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
    std::array<glm::dvec3, 2> ends;
    std::array<glm::dvec2, 2> directions;
    rail_size size;
    std::vector<carve_part> parts;
    double inmost { 0.0 }; // the least and the most the parts are carved out with
    double outmost { 0.0 };
};

std::string rail_name( carve_rail const &Rail ) {

    return std::format( "{}/{}{}", Rail.track->name(), Rail.path, ( Rail.side > 0 ? "+1" : "-1" ) );
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

// the tracks as carved, kept as long as what's kept of the rails around holds
struct carve_memo {
    std::unordered_map<TTrack const *, carve_track> tracks;
    unsigned int stamp { 0 };
};

carve_memo &carved_tracks() {

    static carve_memo memo;
    return memo;
}

carve_track &track_of( TTrack &Track ) {

    auto &memo { carved_tracks() };
    if( memo.stamp != memo_stamp() ) {
        memo.tracks.clear();
        memo.stamp = memo_stamp();
    }
    auto &track { memo.tracks[ &Track ] };
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
            rail.laidfrom = laid_from( Track, path, side );
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
    auto const add = [ &around ]( TTrack *Other ) {
        if( Other == nullptr || std::ranges::find( around, Other ) != around.end() || false == cut_rails( *Other ) ) {
            return;
        }
        if( false == track_of( *Other ).size.grooved ) {
            around.push_back( Other );
        } };
    add( &Track );
    // the rails crossing ours go on past their joints, near crossings at those too
    auto const tracks_on = [ &add ]( TTrack *Other ) {
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

// the part of a rail between two cuts as designed, from the cut before it and the cut after it, none of either at the ends of the rail
carve_part design_part( rail_cut const *Before, rail_cut const *After ) {

    knots::line const nothing;
    carve_part part;
    if( Before != nullptr ) {
        part.from = Before->start;
        part.cut[ 0 ] = true;
        part.kinds[ 0 ] = Before->starting.kind;
        part.anchors[ 0 ] = Before->starting.anchor;
        part.runons[ 0 ] = Before->starting.runon;
        part.obtuse[ 0 ] = Before->obtuse;
        part.marks[ 0 ] = Before->mark;
        part.throwable = Before->throwable;
    }
    if( After != nullptr ) {
        part.to = After->end;
        part.cut[ 1 ] = true;
        part.kinds[ 1 ] = After->ending.kind;
        part.anchors[ 1 ] = After->ending.anchor;
        part.runons[ 1 ] = After->ending.runon;
        part.obtuse[ 1 ] = After->obtuse;
        part.marks[ 1 ] = After->mark;
    }
    part.carved = ( Before != nullptr && Before->carved ) || ( After != nullptr && After->carved );
    auto const &endoffsets { After ? After->ending.offsets : nothing };
    part.offsets = knots::sum( Before ? Before->starting.offsets : nothing, endoffsets );
    // only the cut before makes the part throwable
    part.carve = ( Before != nullptr && part.throwable ? knots::sum( Before->starting.carve, endoffsets ) : part.offsets );
    part.rolled = knots::bound( Before ? Before->starting.rolled : nothing, After ? After->ending.rolled : nothing, true );
    return part;
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
    for( auto &rail : carved.rails ) {
        auto cuts { Layout->plans[ rail.path ][ rail.side > 0 ].cuts };
        std::ranges::stable_sort( cuts, []( rail_cut const &A, rail_cut const &B ) { return A.at < B.at; } );
        for( std::size_t j = 0; j <= cuts.size(); ++j ) {
            auto const *before { j > 0 ? &cuts[ j - 1 ] : nullptr };
            auto const *after { j < cuts.size() ? &cuts[ j ] : nullptr };
            auto part { design_part( before, after ) };
            for( auto const &knot : part.carve ) {
                rail.inmost = std::min( rail.inmost, knot.y );
                rail.outmost = std::max( rail.outmost, knot.y );
            }
            rail.parts.push_back( std::move( part ) );
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
        m_from = ( Part.cut[ 0 ] ? std::max( Part.from - ahead, -outside ) : Rail.laidfrom );
        m_to = ( Part.cut[ 1 ] ? std::min( Part.to + behind, Rail.length + outside ) : Rail.length );
        m_first = static_cast<long>( std::ceil( m_from / fine_step - 1e-9 ) );
        m_last = static_cast<long>( std::floor( m_to / fine_step + 1e-9 ) );
        if( m_last >= m_first ) {
            m_stations.resize( static_cast<std::size_t>( m_last - m_first + 1 ) );
        }
    }
    bool empty() const {
        return m_stations.empty(); }
    std::size_t count() const {
        return m_stations.size(); }
    // where along the path the part is scanned from and to
    double from() const {
        return m_from; }
    double to() const {
        return m_to; }
    // the first and the last of its stations
    long first() const {
        return m_first; }
    long last() const {
        return m_last; }
    static double position( long const K ) {
        return static_cast<double>( K ) * fine_step; }
    // the middle of the part as scanned, where its end nearer to it decides how it's planed
    double middle() const {
        return 0.5 * ( std::max( m_part.from, m_from ) + std::min( m_part.to, m_to ) ); }
    station frame( double const Distance ) const {
        station result;
        glm::dvec2 along;
        frame_of( m_rail, Distance, result.point, along, result.outwards );
        result.offset = offset( Distance, result.runon );
        result.known = true;
        return result; }
    station const &at( long const K ) {
        auto &result { m_stations[ static_cast<std::size_t>( K - m_first ) ] };
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
                auto const &lastknot { line.back() };
                auto const &previous { line[ line.size() - 2 ] };
                auto const course { ( lastknot.y - previous.y ) / std::max( lastknot.x - previous.x, 1e-6 ) };
                return lastknot.y + ( course + runon_bend ) * ( Distance - lastknot.x );
            }
        }
        else if( m_part.cut[ 0 ] && m_part.kinds[ 0 ] == rail_end::point && m_part.runons[ 0 ] > 0.0 && Distance < m_part.from ) {
            Runon = true;
            if( line.size() >= 2 && Distance < line.front().x ) {
                auto const &firstknot { line.front() };
                auto const &next { line[ 1 ] };
                auto const course { ( next.y - firstknot.y ) / std::max( next.x - firstknot.x, 1e-6 ) };
                return firstknot.y + ( runon_bend - course ) * ( firstknot.x - Distance );
            }
        }
        return knots::sample( line, Distance, 0.0 ); }
    // the stations near the other rail: where the middle of the head of ours lies across its gauge line within what Across gives where along it,
    // along it within Along
    template <typename Reach>
    std::vector<std::pair<long, long>> stations_near( carve_rail const &Other, Reach const &Across, glm::dvec2 const &Along, double const Spread ) {
        std::vector<std::pair<long, long>> result;
        if( empty() ) {
            return result;
        }
        auto const low { Other.low - Spread };
        auto const high { Other.high + Spread };
        auto distance { 0.0 };
        auto tracking { false };
        for( auto const &[ s, outwards_here, centre ] : coarse_stations() ) {
            if( auto const point { plan( centre ) }; point.x < low.x || point.y < low.y || point.x > high.x || point.y > high.y ) {
                tracking = false;
                continue;
            }
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
            if( auto const within { Across( distance ) }; lateral >= within.x - margin && lateral <= within.y + margin ) {
                add_window( result, s );
            }
        }
        return result; }
    // a line of how far the profile reaches out from its gauge face as carved, or from where in, from how far out from the gauge line it can reach
    // at each station: Limit and past where it planes nothing
    // where a run of planing ends between stations, the line goes on as it ran out to Reach, where it stops planing what's left of the head
    knots::line local_line( std::vector<double> const &Values, bool const Upper, double const Limit, double const Clear, double const Reach ) {
        std::vector local( Values.size(), Clear );
        std::vector<char> planed( Values.size(), 0 );
        plane( Values, Upper, Limit, local, planed );
        knots::line line;
        for( std::size_t i = 0; i < Values.size(); ++i ) {
            auto const here { planed[ i ] != 0 };
            auto const before { i > 0 && planed[ i - 1 ] };
            auto const after { i + 1 < Values.size() && planed[ i + 1 ] };
            if( here && i > 0 && false == before && after ) {
                freed( line, local, i, i + 1, -1.0, Upper, Reach );
            }
            if( here || before || after ) {
                line.emplace_back( position( m_first + static_cast<long>( i ) ), local[ i ] );
            }
            if( here && before && i + 1 < Values.size() && false == after ) {
                freed( line, local, i, i - 1, 1.0, Upper, Reach );
            }
        }
        return knots::simplified( line, 0.5 * slack ); }

private:
    struct coarse {
        double at;
        glm::dvec2 outwards;
        glm::dvec3 centre; // of the head
    };
    // the stations the rails around are looked for at, from where the part is scanned from on in steps to where it's scanned to
    std::vector<coarse> const &coarse_stations() {
        if( m_coarse.empty() ) {
            auto s { m_from };
            while( true ) {
                auto const here { frame( s ) };
                m_coarse.emplace_back( s, here.outwards, across( here, here.offset + 0.5 * m_rail.size.head ) );
                if( s >= m_to ) {
                    break;
                }
                s = std::min( s + coarse_step, m_to );
            }
        }
        return m_coarse; }
    // the stations around one looked at At carved, joined on to the last of the Windows where they reach it
    void add_window( std::vector<std::pair<long, long>> &Windows, double const At ) const {
        auto const a { std::max( m_first, static_cast<long>( std::floor( ( At - window ) / fine_step ) ) ) };
        auto const b { std::min( m_last, static_cast<long>( std::ceil( ( At + window ) / fine_step ) ) ) };
        if( false == Windows.empty() && a <= Windows.back().second + 1 ) {
            Windows.back().second = std::max( Windows.back().second, b );
        }
        else if( a <= b ) {
            Windows.emplace_back( a, b );
        }
    }
    // how far the profile reaches out at each station from the part as carved, Local, where it's planed short of Limit, Upper, or past it
    void plane( std::vector<double> const &Values, bool const Upper, double const Limit, std::vector<double> &Local, std::vector<char> &Planed ) {
        for( std::size_t i = 0; i < Values.size(); ++i ) {
            if( std::isinf( Values[ i ] ) ) {
                continue;
            }
            auto const value { Values[ i ] - at( m_first + static_cast<long>( i ) ).offset };
            if( Upper ? value < Limit : value > Limit ) {
                Local[ i ] = value;
                Planed[ i ] = 1;
            }
        }
    }
    // where a run of planing ends between the station I and the one Before it, Direction on from it, the line going on as it ran out to Reach
    void freed( knots::line &Line, std::vector<double> const &Local, std::size_t const I, std::size_t const Before, double const Direction, bool const Upper, double const Reach ) const {
        auto const slope { ( Local[ I ] - Local[ Before ] ) * Direction / fine_step };
        auto const remaining { Reach - Local[ I ] };
        if( ( Upper ? slope > 0.0 && remaining > 0.0 : slope < 0.0 && remaining < 0.0 ) && remaining / slope < fine_step ) {
            Line.emplace_back( position( m_first + static_cast<long>( I ) ) + Direction * remaining / slope, Reach );
        }
    }
// members
    carve_rail const &m_rail;
    carve_part const &m_part;
    double m_from { 0.0 };
    double m_to { 0.0 };
    long m_first { 0 };
    long m_last { -1 };
    std::vector<station> m_stations;
    std::vector<coarse> m_coarse; // the stations the rails around are looked for at
};

// how a bound made of a line and a limit runs on at a point: along the line where it bounds there, not across to where the other one does
template <typename Value>
double run_of( Value const &Bound, double const At ) {

    bool bounding { false };
    bool before { false };
    bool after { false };
    auto const at { Bound( At, bounding ) };
    auto const b { Bound( At - 0.5 * fine_step, before ) };
    auto const f { Bound( At + 0.5 * fine_step, after ) };
    if( before == after ) {
        return ( f - b ) / fine_step;
    }
    return ( before == bounding ? ( at - b ) : ( f - at ) ) / ( 0.5 * fine_step );
}

// which side the run of stations from I to J planes the part from, as told for settle_runs below
template <typename Hit>
bool run_upper( std::vector<Hit> const &Hits, std::size_t const I, std::size_t const J, long const First, double const Middle ) {

    auto const upper_at = [ &Hits ]( std::size_t const K ) {
        return Hits[ K ].removed[ 0 ] <= Hits[ K ].removed[ 1 ]; };
    auto const leftstart { I > 0 };
    auto const leftend { J + 1 < Hits.size() };
    if( leftstart && leftend ) {
        auto const a { part_scan::position( First + static_cast<long>( I ) ) };
        auto const b { part_scan::position( First + static_cast<long>( J ) ) };
        return upper_at( std::abs( a - Middle ) <= std::abs( b - Middle ) ? I : J );
    }
    if( leftstart || leftend ) {
        return upper_at( leftstart ? I : J );
    }
    auto least { I };
    for( auto k { I }; k <= J; ++k ) {
        if( std::min( Hits[ k ].removed[ 0 ], Hits[ k ].removed[ 1 ] ) < std::min( Hits[ least ].removed[ 0 ], Hits[ least ].removed[ 1 ] ) ) {
            least = k;
        }
    }
    return upper_at( least );
}

// how a run of stations along a part, where it runs into a flangeway or a head, planes it: from the side its head leaves that at the end of the
// run nearer the middle of the part; running into it all the way, from the side taking less of it where it takes least
template <typename Hit, typename Apply>
void settle_runs( std::vector<Hit> const &Hits, long const First, double const Middle, Apply const &Applied ) {

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
        auto const upper { run_upper( Hits, i, j, First, Middle ) };
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
    std::array<double, 2> removed {};
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

// how far out from the gauge face of the part as carved its head can reach, and from how far in, at each station along it
struct trim_bounds {
    std::vector<double> upper;
    std::vector<double> lower;
};

// the stations of the part from A to B, near the rail of theirs, kept out of its flangeway
void trim_window( part_scan &Scan, carve_rail const &Rail, carve_part const &Part, carve_rail const &Theirs, long const A, long const B, trim_bounds &Bounds ) {

    if( debugging( Rail ) ) {
        WriteLog( std::format( "CARVE F {} part {:f}-{:f} vs {} window {:f}-{:f}", rail_name( Rail ), Part.from, Part.to, rail_name( Theirs ), part_scan::position( A ), part_scan::position( B ) ) );
    }
    std::vector<band_hit> hits;
    auto distance { nearest_distance( *Theirs.line, across( Scan.at( A ), Scan.at( A ).offset ) ) };
    for( auto k { A }; k <= B; ++k ) {
        hits.push_back( band_at( Scan.at( k ), Rail, Theirs, distance ) );
    }
    settle_runs( hits, A, Scan.middle(), [ &Scan, &Rail, &hits, &Bounds, A ]( std::size_t const I, bool const Upper ) {
        auto const index { static_cast<std::size_t>( A - Scan.first() ) + I };
        if( debugging( Rail ) ) {
            WriteLog( std::format( "CARVE F   at {:f}{}{:f} offset {:f}", part_scan::position( A + static_cast<long>( I ) ), ( Upper ? " upper " : " lower " ), ( Upper ? hits[ I ].upper : hits[ I ].lower ), Scan.at( A + static_cast<long>( I ) ).offset ) );
        }
        if( Upper ) {
            Bounds.upper[ index ] = std::min( Bounds.upper[ index ], hits[ I ].upper );
        }
        else {
            Bounds.lower[ index ] = std::max( Bounds.lower[ index ], hits[ I ].lower );
        } } );
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
    trim_bounds bounds { std::vector( scan.count(), unbounded ), std::vector( scan.count(), -unbounded ) };
    auto const &around { around_of( *Rail.track ) };
    for( auto *other : around ) {
        for( auto const &theirs : track_of( *other ).rails ) {
            if( false == flangeway_counts( Rail, Part, theirs ) ) {
                continue;
            }
            auto const windows { scan.stations_near( theirs, [ &size ]( double ) { return glm::dvec2 { -flangeway - 0.5 * size.head, 0.5 * size.head }; }, { -nearby, theirs.length + nearby }, flangeway + size.head + nearby ) };
            for( auto const &[ a, b ] : windows ) {
                trim_window( scan, Rail, Part, theirs, a, b, bounds );
            }
        }
    }
    Part.upper = scan.local_line( bounds.upper, true, size.high, size.high + 0.01, size.head );
    Part.lower = scan.local_line( bounds.lower, false, size.low, size.low - 0.01, 0.0 );
    Part.trimfrom = scan.from();
    Part.trimto = scan.to();
}

// the head of ours lying over the head of another part, out from our gauge line
struct body_hit {
    bool in { false };
    bool heads { false }; // or only the feet of two rails back to back
    std::array<double, 2> removed {};
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

// the wheels of a rail need some of its head: none where the start is past the end
bool needed( glm::dvec2 const &Need ) {

    return Need.x <= Need.y;
}

// a bound across ours and how square it runs to ours: the lines along ours run square, those along theirs slanted
struct head_bound {
    double at;
    double slant;
};

// the higher of two bounds
head_bound higher_of( head_bound const &A, head_bound const &B ) {

    return A.at >= B.at ? A : B;
}

// the lower of two bounds
head_bound lower_of( head_bound const &A, head_bound const &B ) {

    return A.at <= B.at ? A : B;
}

// a bound kept between two others
head_bound clamped( head_bound const &Value, head_bound const &Low, head_bound const &High ) {

    return lower_of( higher_of( Value, Low ), High );
}

// as far from both bounds, whichever way they run: the same line from either rail
head_bound between( head_bound const &A, head_bound const &B ) {

    return head_bound { ( A.at * A.slant + B.at * B.slant ) / ( A.slant + B.slant ), 1.0 };
}

// where the wheels of each need the head, run along ours, along theirs, or the bounds of the heads
head_bound need_bound( double const Need, double const Bound, double const Slant, double const Otherwise ) {

    return head_bound { Need, Need == Bound ? Slant : Otherwise };
}

// where the heads of ours and theirs lie over each other across ours, from a to b
struct heads_overlap {
    double slant; // how square theirs runs to ours, no less than at the widest angle looked at
    head_bound a;
    head_bound b;
    bool overlap; // the heads lie over each other at all
    double middle;
    double half; // of how far they do
};

// the bound of ours where its head lies under theirs, back to back with their feet too, and from where the wheels of theirs roll on it
void settle_above( body_hit const &Hit, heads_overlap const &Heads, double &Upper, double &Worn ) {

    auto const &a { Heads.a };
    auto const &b { Heads.b };
    auto ouredge { Hit.ours.y };
    auto theiredge { Hit.theirs.x };
    if( Heads.overlap ) {
        auto const ours { needed( Hit.ourneed ) && Hit.ourneed.y > a.at ? clamped( need_bound( Hit.ourneed.y, Hit.ours.y, Hit.ourslant.y, 1.0 ), a, b ) : a };
        auto const theirs { needed( Hit.theirneed ) && Hit.theirneed.x < b.at ? clamped( need_bound( Hit.theirneed.x, Hit.theirs.x, Hit.theirslant.x, Heads.slant ), a, b ) : b };
        auto const meet { ours.at <= theirs.at ? std::clamp( Heads.middle, ours.at, theirs.at ) : between( ours, theirs ).at };
        auto const given { ( meet - Heads.middle ) / Heads.half }; // below zero ours gives way
        // over the shoulder of the other head only, never past it into the head
        auto const edge { given < 0.0 && Hit.facing < 0.0 ? std::max( meet, std::min( meet - given * Hit.theirshoulder / Heads.slant, Hit.theircrown ) ) : meet };
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

// the bound of ours where its head lies over theirs and the two heads lie over each other
void settle_below( body_hit const &Hit, heads_overlap const &Heads, double &Lower ) {

    auto const &a { Heads.a };
    auto const &b { Heads.b };
    auto const ours { needed( Hit.ourneed ) && Hit.ourneed.x < b.at ? clamped( need_bound( Hit.ourneed.x, Hit.ours.x, Hit.ourslant.x, 1.0 ), a, b ) : b };
    auto const theirs { needed( Hit.theirneed ) && Hit.theirneed.y > a.at ? clamped( need_bound( Hit.theirneed.y, Hit.theirs.y, Hit.theirslant.y, Heads.slant ), a, b ) : a };
    auto const meet { theirs.at <= ours.at ? std::clamp( Heads.middle, theirs.at, ours.at ) : between( ours, theirs ).at };
    auto const given { ( meet - Heads.middle ) / Heads.half }; // above zero ours gives way
    auto const edge { given > 0.0 && Hit.facing > 0.0 ? std::min( meet, std::max( meet - given * Hit.theirshoulder / Heads.slant, Hit.theircrown ) ) : meet };
    Lower = std::max( Lower, edge + slack );
}

// the bounds of ours where its head lies under theirs, Above, or over it: the two meet in the middle of where they lie over each other, as far as
// the wheels of each leave room; a head giving way runs on past the meeting over the rounded shoulder of the other one, facing it with its outer edge.
// Two rails back to back meet with their feet in the middle between them too, and the wheels of theirs roll on ours planed flush against it
void settle( body_hit const &Hit, bool const Above, double &Upper, double &Lower, double &Worn ) {

    auto const slant { std::max( std::abs( Hit.facing ), cos_wide ) };
    auto const a { higher_of( { Hit.ours.x, Hit.ourslant.x }, { Hit.theirs.x, Hit.theirslant.x } ) };
    auto const b { lower_of( { Hit.ours.y, Hit.ourslant.y }, { Hit.theirs.y, Hit.theirslant.y } ) };
    heads_overlap const heads { slant, a, b, Hit.heads && b.at > a.at, between( a, b ).at, std::max( 0.5 * ( b.at - a.at ), 1e-9 ) };
    if( Above ) {
        settle_above( Hit, heads, Upper, Worn );
    }
    else if( heads.overlap ) {
        settle_below( Hit, heads, Lower );
    }
}

struct carved_lines {
    knots::line offsets;
    knots::line extents;
    knots::line inner;
    knots::line rolled;
    knots::line worn;
    double from { 0.0 };
    double to { 0.0 };
    material_handle wornby { null_handle };
};

// the blades of a switch are in the way as they lie closed, the stock rails of their own switch aside
// where the parts of theirs lie, as they bend out on to the stations around
glm::dvec2 spread_of( carve_rail const &Theirs, double const Distance ) {

    glm::dvec2 offsets { unbounded, -unbounded };
    for( auto const &part : Theirs.parts ) {
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
    return glm::dvec2 { offsets.x + Theirs.size.low - 0.1, offsets.y + Theirs.size.high + 0.1 };
}

// how square to ours a line runs at a slope to it
double square_of( double const Slope ) {

    return 1.0 / std::sqrt( 1.0 + Slope * Slope );
}

// how far out the head of theirs reaches at most for the bound Which: to its outer edge, the profile out to, the top of the head to
double bound_reach( rail_size const &Size, int const Which ) {

    if( Which == 1 ) {
        return Size.head;
    }
    return ( Which == 2 ? Size.high : Size.head - Size.shoulder );
}

// how the bounds of ours run on, along the lines bounding them here: from Clear where the line says nothing, out to Reach
struct our_bound {
    part_scan const &scan;
    knots::line const &line;
    double clear;
    double reach;
    bool upper;
    double operator()( double const At, bool &Bounding ) const {
        bool runon { false };
        auto const value { knots::sample( line, At, clear ) };
        Bounding = ( upper ? value < reach : value > reach );
        return scan.offset( At, runon ) + ( Bounding ? value : reach ); }
};

// the head of theirs and how far out its profile reaches, out from its gauge line: the bound Which (from, to, out to, the top of the head to)
// and how it runs on
struct their_bound {
    carve_part const &part;
    rail_size const &size;
    bool trimmed;
    int which;
    double operator()( double const At, bool &Bounding ) const {
        auto const offset { knots::sample( part.carve, At, 0.0 ) };
        if( which == 0 ) {
            auto const theirclearlow { size.low - 0.01 };
            auto const theirlow { trimmed ? knots::sample( part.lower, At, theirclearlow ) : theirclearlow };
            Bounding = theirlow > 0.0;
            return offset + std::max( 0.0, theirlow );
        }
        auto const reach { bound_reach( size, which ) };
        auto const theirclearup { size.high + 0.01 };
        auto const theirup { trimmed ? knots::sample( part.upper, At, theirclearup ) : theirclearup };
        Bounding = theirup < reach;
        return offset + std::min( reach, theirup ); }
    // where the bound lies at a point
    double operator()( double const At ) const {
        bool bounding { false };
        return ( *this )( At, bounding ); }
};

// a station of ours and how the rail of theirs runs across it there
struct station_crossing {
    station const &here;
    double s; // where along ours
    double up; // how far out the head of ours is left, kept out of the flangeways around
    glm::dvec2 ours; // where the head of ours is, out from its gauge line
    double centre; // of the head of ours
    double lateral; // how far out from the gauge line of theirs that lies
    double facing;
    double drift; // a step out across ours goes this far along theirs, too
    double distance; // along theirs
    bool route; // along theirs, not past its ends
    // how the bounds of ours run, once something of theirs is in their way
    glm::dvec2 ourslant { 1.0 };
    double footslant { 1.0 };
    bool slanted { false };
    // where a line of theirs, out from its gauge line, lies out from ours
    double mapped( double const V ) const {
        return centre + ( V - lateral ) / facing; }
    // where a line of theirs running slanted meets ours, out from it
    template <typename Line>
    double meets( Line const &Bound ) const {
        bool bounding { false };
        auto at { mapped( Bound( distance, bounding ) ) };
        for( int step = 0; step < 6; ++step ) {
            auto const next { mapped( Bound( distance + ( at - centre ) * drift, bounding ) ) };
            auto const done { std::abs( next - at ) < 1e-5 };
            at = next;
            if( done ) {
                break;
            }
        }
        return at; }
    // and how square to ours a line of theirs running on there is, where it meets ours At
    template <typename Line>
    double slant( Line const &Bound, double const At ) const {
        auto const slope { run_of( Bound, distance + ( At - centre ) * drift ) };
        return std::abs( facing - slope * drift ) / std::sqrt( 1.0 + slope * slope ); }
};

// the log of a station of ours planed against the head of the part of theirs
std::string settled_log( double const At, std::size_t const Part, bool const Above, body_hit const &Hit, double const Upper, double const Lower ) {

    return std::format(
        "CARVE O   at {:f} their part {}{}{} ours {:f}..{:f} theirs {:f}..{:f} need {:f}..{:f}/{:f}..{:f} facing {:f} slants {:f},{:f}/{:f},{:f} -> upper {:f} lower {:f}",
        At, Part, ( Above ? " above" : " below" ), ( Hit.heads ? " heads" : " feet" ),
        Hit.ours.x, Hit.ours.y, Hit.theirs.x, Hit.theirs.y,
        Hit.ourneed.x, Hit.ourneed.y, Hit.theirneed.x, Hit.theirneed.y,
        Hit.facing, Hit.ourslant.x, Hit.ourslant.y, Hit.theirslant.x, Hit.theirslant.y, Upper, Lower );
}

// the log of the part as carved: where it's scanned, where its ends are put, and the tracks around it's carved against
void log_carved( carve_rail const &Rail, carve_part const &Part, part_scan const &Scan, double const From, double const To, bool const Faulty, std::span<TTrack * const> const Around ) {

    std::string names;
    for( auto const *other : Around ) {
        names += std::format( " {}", other->name() );
    }
    WriteLog( std::format( "CARVE {} part {:f}-{:f} scanned {:f}-{:f} ends {:f}-{:f}{} around{}", rail_name( Rail ), Part.from, Part.to, Scan.from(), Scan.to(), From, To, ( Faulty ? " FAULTY" : "" ), names ) );
}

// a wing rail run on past its own end into what planes it to its point: its lines go on to where its ends are put, From and To
void run_on_ends( carved_lines &Result, part_scan const &Scan, carve_part const &Part, double const From, double const To ) {

    if( Part.cut[ 1 ] && To > Part.to + 1e-6 ) {
        bool runon { false };
        auto const out { Scan.offset( To, runon ) };
        Result.offsets.emplace_back( To, out );
        Result.rolled.emplace_back( To, rolled_band - out );
    }
    if( Part.cut[ 0 ] && From < Part.from - 1e-6 ) {
        bool runon { false };
        auto const out { Scan.offset( From, runon ) };
        Result.offsets.insert( Result.offsets.begin(), { From, out } );
        Result.rolled.insert( Result.rolled.begin(), { From, rolled_band - out } );
    }
}

// a part carved against the heads of the parts around: how far out its head can reach at each station, from how far in, and from where
// the wheels of a rail beside roll on it
class part_carver {

public:
    part_carver( carve_rail const &Rail, carve_part const &Part, part_scan &Scan ) :
        m_rail { Rail },
        m_part { Part },
        m_scan { Scan },
        m_size { Rail.size },
        m_clearup { Rail.size.high + 0.01 },
        m_clearlow { Rail.size.low - 0.01 },
        m_upper( Scan.count(), unbounded ),
        m_lower( Scan.count(), -unbounded ),
        m_worn( Scan.count(), unbounded ),
        m_wornby( Scan.count(), null_handle ) {
        // nothing carved against yet: the head is left as it's trimmed
    }
    // the part carved against the heads of the parts of the rail of theirs
    void against( carve_rail &Theirs ) {
        auto const &theirsize { Theirs.size };
        auto const reach { std::max( std::abs( Theirs.inmost ), std::abs( Theirs.outmost ) ) + theirsize.high + nearby + 0.1 };
        auto const windows { m_scan.stations_near( Theirs, [ &Theirs ]( double const Distance ) { return spread_of( Theirs, Distance ); }, { -nearby, Theirs.length + nearby }, reach ) };
        for( auto const &[ a, b ] : windows ) {
            carve_window( Theirs, a, b );
        }
    }
    // the lines the head is carved to, out and in, and the line of where the wheels of the rails beside roll on it
    void bound_head() {
        m_extents = knots::bound( m_part.upper, m_scan.local_line( m_upper, true, m_size.high, m_clearup, m_size.head ), true );
        m_inner = knots::bound( m_part.lower, m_scan.local_line( m_lower, false, m_size.low, m_clearlow, 0.0 ), false );
        m_wornline = m_scan.local_line( m_worn, true, m_size.head, m_clearup, m_size.head ); }
    knots::line const &extents() const {
        return m_extents; }
    knots::line const &inner() const {
        return m_inner; }
    knots::line const &worn() const {
        return m_wornline; }
    // the ends of the part put where they're declared to be, From and To: Faulty where a blunt one finds nothing to begin at
    void put_ends( double &From, double &To, bool &Faulty ) const {
        for( int end = 0; end < 2; ++end ) {
            if( false == m_part.cut[ end ] ) {
                continue;
            }
            put_end( end, end ? To : From, Faulty );
        }
    }
    // nothing left of the head somewhere along the part, past its sharp ends getting wider: Faulty already, or found from From to To
    bool headless( double const From, double const To, bool const Faulty ) const {
        auto start { std::max( From, m_rail.laidfrom ) + 2.0 * fine_step };
        auto end { std::min( To, m_rail.length ) - 2.0 * fine_step };
        for( int side = 0; side < 2; ++side ) {
            if( auto found { 0.0 }; sharp_end( side, side ? To : From, found ) ) {
                ( side ? end : start ) = found + ( side ? -2.0 : 2.0 ) * fine_step;
            }
        }
        auto faulty { Faulty };
        auto k { static_cast<long>( std::ceil( start / fine_step ) ) };
        while( k <= static_cast<long>( std::floor( end / fine_step ) ) && false == faulty ) {
            faulty = ( width( part_scan::position( k ) ) < point_width );
            ++k;
        }
        return faulty; }
    // the wheels of the rails beside polish the head as bright as the brightest of them
    material_handle worn_by() const {
        auto const band { m_rail.track->tread_band() };
        material_handle result { null_handle };
        for( auto const material : m_wornby ) {
            if( material != null_handle && material != result
             && ( result == null_handle || tread_brightness( material, band ) > tread_brightness( result, band ) ) ) {
                result = material;
            }
        }
        return result; }

private:
    // the stations of the part from A to B near the rail of theirs, carved against the heads of its parts
    void carve_window( carve_rail &Theirs, long const A, long const B ) {
        if( debugging( m_rail ) ) {
            WriteLog( std::format( "CARVE O {} part {:f}-{:f} vs {} window {:f}-{:f}", rail_name( m_rail ), m_part.from, m_part.to, rail_name( Theirs ), part_scan::position( A ), part_scan::position( B ) ) );
        }
        std::vector hits( Theirs.parts.size(), std::vector<body_hit>( static_cast<std::size_t>( B - A + 1 ) ) );
        auto distance { nearest_distance( *Theirs.line, across( m_scan.at( A ), m_scan.at( A ).offset ) ) };
        for( auto k { A }; k <= B; ++k ) {
            carve_station( Theirs, A, k, distance, hits );
        }
        for( std::size_t j = 0; j < hits.size(); ++j ) {
            settle_part( Theirs, A, j, hits[ j ] );
        }
    }
    // the heads of the parts of theirs in the way of ours at the station K of the window from A, found from Distance along theirs on
    void carve_station( carve_rail &Theirs, long const A, long const K, double &Distance, std::vector<std::vector<body_hit>> &Hits ) const {
        auto const &here { m_scan.at( K ) };
        auto const s { part_scan::position( K ) };
        auto const up { knots::sample( m_part.upper, s, m_clearup ) };
        auto const low { knots::sample( m_part.lower, s, m_clearlow ) };
        glm::dvec2 const ours { here.offset + std::max( 0.0, low ), here.offset + std::min( m_size.head, up ) };
        if( ours.y - ours.x < slack ) {
            return;
        }
        auto const centre { here.offset + 0.5 * m_size.head };
        glm::dvec2 outwards;
        glm::dvec2 along;
        auto const lateral { project( Theirs, across( here, centre ), Distance, outwards, &along ) };
        auto const facing { glm::dot( here.outwards, outwards ) };
        if( std::abs( facing ) < cos_wide ) {
            return;
        }
        // a step out across ours goes this far along theirs, too: where a line of theirs running slanted meets ours, out from it
        auto const drift { glm::dot( here.outwards, along ) };
        station_crossing crossing { here, s, up, ours, centre, lateral, facing, drift, Distance, Distance >= 0.0 && Distance <= Theirs.length };
        if( Theirs.track == m_rail.track && Theirs.side == m_rail.side && Distance < Theirs.heel + heel_margin ) {
            return;
        }
        for( std::size_t j = 0; j < Theirs.parts.size(); ++j ) {
            part_hit( Theirs, Theirs.parts[ j ], crossing, Hits[ j ][ static_cast<std::size_t>( K - A ) ] );
        }
    }
    // how the bounds of ours run at the station, once something of theirs is in their way
    void slant_ours( station_crossing &At ) const {
        At.ourslant = { square_of( run_of( our_bound { m_scan, m_part.lower, m_clearlow, 0.0, false }, At.s ) ), square_of( run_of( our_bound { m_scan, m_part.upper, m_clearup, m_size.head, true }, At.s ) ) };
        At.footslant = square_of( run_of( our_bound { m_scan, m_part.upper, m_clearup, m_size.high, true }, At.s ) );
        At.slanted = true;
    }
    // the head of the part of theirs lying over ours at the station, or their feet back to back: how, for the room between them to be shared
    void part_hit( carve_rail &Theirs, carve_part &Theirpart, station_crossing &At, body_hit &Hit ) const {
        auto const distance { At.distance };
        if( distance < std::max( Theirpart.from, 0.0 ) || distance > std::min( Theirpart.to, Theirs.length ) || false == bodies_count( m_rail, m_part, Theirs, Theirpart ) ) {
            return;
        }
        trim( Theirs, Theirpart );
        auto const &theirsize { Theirs.size };
        auto const trimmed { distance >= Theirpart.trimfrom && distance <= Theirpart.trimto };
        auto const bound = [ &Theirpart, &theirsize, trimmed ]( int const Which ) {
            return their_bound { Theirpart, theirsize, trimmed, Which }; };
        if( auto const v0 { bound( 0 )( distance ) }; bound( 1 )( distance ) - v0 < slack ) {
            return;
        }
        // the same rail laid along another path: both lie as one
        if( auto const theiroffset { knots::sample( Theirpart.carve, distance, 0.0 ) };
            At.facing > 0.99 && std::abs( At.mapped( theiroffset ) - At.here.offset ) < 2.0 * slack && std::abs( At.mapped( theiroffset + theirsize.head ) - ( At.here.offset + m_size.head ) ) < 2.0 * slack ) {
            return;
        }
        auto const m0 { At.meets( bound( 0 ) ) };
        auto const m1 { At.meets( bound( 1 ) ) };
        glm::dvec2 const heads { std::min( m0, m1 ), std::max( m0, m1 ) };
        auto const overlap { std::min( At.ours.y, heads.y ) - std::max( At.ours.x, heads.x ) };
        auto const above { heads.x + heads.y > At.ours.x + At.ours.y };
        auto const ourfoot { At.here.offset + std::min( m_size.high, At.up ) };
        auto const backs { At.facing < 0.0 && above };
        auto const theirfoot { ( overlap > 2.0 * slack || backs ) ? At.meets( bound( 2 ) ) : 0.0 };
        if( auto const feet { backs && ourfoot > theirfoot + 2.0 * slack }; overlap <= 2.0 * slack && false == feet ) {
            return;
        }
        if( false == At.slanted ) {
            slant_ours( At );
        }
        Hit.ours = At.ours;
        Hit.theirs = heads;
        Hit.ourslant = At.ourslant;
        Hit.theirslant = ( m0 <= m1 ? glm::dvec2 { At.slant( bound( 0 ), m0 ), At.slant( bound( 1 ), m1 ) } : glm::dvec2 { At.slant( bound( 1 ), m1 ), At.slant( bound( 0 ), m0 ) } );
        Hit.facing = At.facing;
        Hit.ourshoulder = m_size.shoulder;
        Hit.theirshoulder = theirsize.shoulder;
        Hit.ourfoot = ourfoot;
        Hit.theirfoot = theirfoot;
        Hit.footslant = { At.footslant, At.slant( bound( 2 ), theirfoot ) };
        Hit.ourcrown = At.here.offset + std::min( m_size.head - m_size.shoulder, At.up );
        Hit.theircrown = At.meets( bound( 3 ) );
        Hit.heads = ( overlap > 2.0 * slack );
        Hit.in = true;
        if( Hit.heads ) {
            auto const lapfrom { std::max( At.ours.x, Hit.theirs.x ) };
            auto const lapto { std::min( At.ours.y, Hit.theirs.y ) };
            Hit.removed[ 0 ] = At.ours.y - lapfrom;
            Hit.removed[ 1 ] = lapto - At.ours.x;
        }
        else {
            Hit.removed[ 0 ] = Hit.ourfoot - Hit.theirfoot;
            Hit.removed[ 1 ] = unbounded;
        }
        if( false == At.here.runon ) {
            Hit.ourneed = { std::max( 0.0, At.ours.x ), std::min( rolled_band, At.ours.y ) };
        }
        if( At.route ) {
            auto const n0 { At.mapped( 0.0 ) };
            auto const n1 { At.mapped( rolled_band ) };
            Hit.theirneed = { std::max( std::min( n0, n1 ), Hit.theirs.x ), std::min( std::max( n0, n1 ), Hit.theirs.y ) };
            Hit.rolledon = true;
            Hit.theirrolled = n1;
        }
    }
    // the room between the heads of ours and of the part J of theirs shared along the window from A, as they lie over each other
    void settle_part( carve_rail const &Theirs, long const A, std::size_t const J, std::vector<body_hit> const &Hits ) {
        settle_runs( Hits, A, m_scan.middle(), [ this, &Theirs, &Hits, A, J ]( std::size_t const I, bool const Above ) {
            auto const index { static_cast<std::size_t>( A - m_scan.first() ) + I };
            // heads back to back lie over each other only with their gauge faces apart, each planed off its outer side
            auto const was { m_worn[ index ] };
            settle( Hits[ I ], Above || Hits[ I ].facing < 0.0, m_upper[ index ], m_lower[ index ], m_worn[ index ] );
            if( m_worn[ index ] < was ) {
                m_wornby[ index ] = Theirs.track->rail_material( Theirs.path );
            }
            if( debugging( m_rail ) ) {
                WriteLog( settled_log( part_scan::position( A + static_cast<long>( I ) ), J, Above || Hits[ I ].facing < 0.0, Hits[ I ], m_upper[ index ], m_lower[ index ] ) );
            } } );
    }
    // how wide what's left of the head is
    double width( double const S ) const {
        return std::min( m_size.head, knots::sample( m_extents, S, m_clearup ) ) - std::max( 0.0, knots::sample( m_inner, S, m_clearlow ) ); }
    // where the head gets to Width first, going from Start in Step, between stations; from nothing of it at Start, Open, where the rails cross
    // and the flangeway of the other one only just reaches the gauge face
    bool reaches( double const Start, int const Step, double const Width, bool const Wider, double &At, bool const Open = false ) const {
        auto const first { Start + ( Open ? Step * 0.005 : 0.0 ) };
        auto k { Step > 0 ? static_cast<long>( std::ceil( first / fine_step - 1e-9 ) ) : static_cast<long>( std::floor( first / fine_step + 1e-9 ) ) };
        auto previous { Start };
        auto before { Open ? 0.0 : width( Start ) };
        if( Wider ? before >= Width : before < Width ) {
            At = Start;
            return true;
        }
        for( ; k >= m_scan.first() && k <= m_scan.last(); k += Step ) {
            auto const s { part_scan::position( k ) };
            auto const now { width( s ) };
            if( Wider ? now >= Width : now < Width ) {
                At = glm::mix( previous, s, std::clamp( ( Width - before ) / ( now - before ), 0.0, 1.0 ) );
                return true;
            }
            previous = s;
            before = now;
        }
        return false; }
    // where an end of the part goes, At as designed: a point where the head of another rail planes it off, not along a flangeway; a blunt end
    // where enough of the head is left, Faulty where it never is
    void put_end( int const End, double &At, bool &Faulty ) const {
        auto const step { End ? -1 : 1 }; // into the part
        auto const nominal { At };
        switch( m_part.kinds[ End ] ) {
            case rail_end::point: {
                // run on past its own end only into the head of another rail, not along a flangeway
                if( auto found { 0.0 }; reaches( std::clamp( m_part.anchors[ End ], m_scan.from(), m_scan.to() ), -step, point_width, false, found ) ) {
                    auto const inside { ( found - nominal ) * step >= 0.0 };
                    auto const kept { std::min( m_size.head, knots::sample( m_part.upper, found, m_clearup ) ) - std::max( 0.0, knots::sample( m_part.lower, found, m_clearlow ) ) };
                    if( inside || kept >= 3.0 * point_width ) {
                        At = found;
                    }
                }
                break;
            }
            case rail_end::blunt: {
                if( nominal >= m_scan.from() && nominal <= m_scan.to() ) {
                    if( auto found { 0.0 }; reaches( nominal, step, blunt_width, true, found, true ) ) {
                        At = found;
                    }
                    else {
                        Faulty = true;
                    }
                }
                break;
            }
            default: {
                break;
            }
        }
    }
    // the head of the part gets wider past its sharp end Side: where it's first wide enough going in from At
    bool sharp_end( int const Side, double const At, double &Found ) const {
        return m_part.cut[ Side ] && m_part.kinds[ Side ] == rail_end::sharp && reaches( At, Side ? -1 : 1, point_width, true, Found ); }
// members
    carve_rail const &m_rail;
    carve_part const &m_part;
    part_scan &m_scan;
    rail_size const &m_size;
    double m_clearup; // where the lines bounding the head say nothing, out and in
    double m_clearlow;
    // at each station: how far out from its gauge line the head can reach, from how far in, from where the wheels of a rail beside roll on it,
    // and what those polish it to
    std::vector<double> m_upper;
    std::vector<double> m_lower;
    std::vector<double> m_worn;
    std::vector<material_handle> m_wornby;
    // the lines the head is carved to, out and in, and where the wheels of the rails beside roll on it
    knots::line m_extents;
    knots::line m_inner;
    knots::line m_wornline;
};

// the part carved against the heads of the parts around, its ends put where they're declared to be
carved_lines carve( carve_rail &Rail, carve_part &Part ) {

    carved_lines result { Part.offsets, {}, {}, Part.rolled, {}, Part.from, Part.to, null_handle };
    trim( Rail, Part );
    part_scan scan { Rail, Part };
    if( scan.empty() ) {
        return result;
    }
    part_carver carver { Rail, Part, scan };
    auto const &around { around_of( *Rail.track ) };
    for( auto *other : around ) {
        design( *other, nullptr );
        for( auto &theirs : track_of( *other ).rails ) {
            if( &theirs == &Rail || false == bodies_count( Rail, theirs ) ) {
                continue;
            }
            carver.against( theirs );
        }
    }
    carver.bound_head();
    auto from { Part.from };
    auto to { Part.to };
    auto faulty { false };
    carver.put_ends( from, to, faulty );
    run_on_ends( result, scan, Part, from, to );
    knots::sort( result.offsets );
    knots::sort( result.rolled );
    faulty = carver.headless( from, to, faulty );
    if( faulty ) {
        ++fault_count();
    }
    if( debugging( Rail ) ) {
        log_carved( Rail, Part, scan, from, to, faulty, around );
    }
    result.from = from;
    result.to = to;
    result.extents = knots::simplified( carver.extents(), 0.5 * slack );
    result.inner = knots::simplified( carver.inner(), 0.5 * slack );
    result.worn = carver.worn();
    result.wornby = carver.worn_by();
    return result;
}

// the cuts of the rails of the layout put in the order they lie along each rail: whether any of them leaves the lines of its parts to carving
bool sort_cuts( rail_layout &Layout ) {

    auto any { false };
    for( auto &paths : Layout.plans ) {
        for( auto &sideplan : paths ) {
            std::ranges::stable_sort( sideplan.cuts, []( rail_cut const &A, rail_cut const &B ) { return A.at < B.at; } );
            any = any || std::ranges::any_of( sideplan.cuts, []( rail_cut const &Cut ) { return Cut.carved; } );
        }
    }
    return any;
}

// the lines of a part as carved, taken by the cut at its end
void take_lines( rail_cut_part &Cut, carved_lines const &Lines ) {

    Cut.offsets = Lines.offsets;
    Cut.extents = Lines.extents;
    Cut.inner = Lines.inner;
    Cut.rolled = Lines.rolled;
    Cut.worn = Lines.worn;
    Cut.wornby = Lines.wornby;
}

// the parts of the rail left to carving carved, the cuts between them put where the parts as carved end
void carve_cuts( carve_rail &Rail, std::vector<rail_cut> &Cuts ) {

    for( std::size_t j = 0; j < Rail.parts.size(); ++j ) {
        auto &part { Rail.parts[ j ] };
        if( false == part.carved ) {
            continue;
        }
        auto const lines { carve( Rail, part ) };
        auto *before { j > 0 && Cuts[ j - 1 ].carved ? &Cuts[ j - 1 ] : nullptr };
        auto *after { j < Cuts.size() && Cuts[ j ].carved ? &Cuts[ j ] : nullptr };
        if( before != nullptr ) {
            before->start = lines.from;
            take_lines( before->starting, lines );
        }
        if( after != nullptr ) {
            after->end = lines.to;
            take_lines( after->ending, lines );
        }
    }
}

} // namespace

namespace track_rails {

void carve_plans( TTrack &Track, rail_layout &Layout ) {

    if( false == cut_rails( Track ) ) {
        return;
    }
    if( false == sort_cuts( Layout ) || track_of( Track ).size.grooved ) {
        return;
    }
    design( Track, &Layout );
    for( auto &rail : track_of( Track ).rails ) {
        auto &cuts { Layout.plans[ rail.path ][ rail.side > 0 ].cuts };
        if( rail.parts.size() != cuts.size() + 1 ) {
            continue;
        }
        carve_cuts( rail, cuts );
    }
}

rail_layout const *designed_layout( TTrack &Track ) {

    auto const &carved { track_of( Track ) };
    return ( carved.planned ? &carved.layout : nullptr );
}

int carve_faults() {

    return fault_count();
}

} // namespace track_rails
