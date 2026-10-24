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

using namespace track_rails;

namespace {

// a wing rail ending right beside a check rail of the rail it runs along goes on in its place up to the far end of it, past the end of the wing
std::vector<glm::dvec2> wing_extension( TTrack &Track, track_rails::check_rail_set const &Checks, int const Path, int const Side, double const At, double const Toward, double const Slope, TTrack &Partner, int const Partnerpath, int const Partnerside, double const Partnerat, double const Partnerdirection ) {

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

// a wing rail bent out along the flangeway of the other rail of its frog, from where that closes in on ours on to Wingend
knots::line flangeway_wing( TTrack const &Track, track_rails::rail_crossing const &Crossing, double const Throat, double const Wingend ) {

    auto const &ours { rail_line( Track, Crossing.path, Crossing.side > 0, false ) };
    auto const &theirs { rail_line( *Crossing.track, Crossing.otherpath, Crossing.otherside > 0, false ) };
    auto const direction { Wingend > Throat ? 1.0 : -1.0 };
    auto const start { Throat - direction * 2.0 };
    auto const count { std::max( 2, static_cast<int>( std::ceil( std::abs( Wingend - start ) / 0.05 ) ) ) };
    glm::dvec3 point, tangent;
    ours.at( start, point, tangent );
    auto distance { nearest_distance( theirs, point ) };
    knots::line wing;
    for( int i = 0; i <= count; ++i ) {
        auto const at { glm::mix( start, Wingend, static_cast<double>( i ) / count ) };
        ours.at( at, point, tangent );
        auto const outwards { -glm::normalize( plan( ours.normal( at, Crossing.side ) ) ) };
        auto offset { 0.0 };
        for( int iteration = 0; iteration < 3; ++iteration ) {
            glm::dvec3 const shifted { point.x + outwards.x * offset, point.y, point.z + outwards.y * offset };
            auto const across { lateral( theirs, Crossing.otherside, shifted, distance ) };
            auto const facing { glm::dot( outwards, -glm::normalize( plan( theirs.normal( distance, Crossing.otherside ) ) ) ) };
            if( std::abs( facing ) < 0.1 ) {
                break;
            }
            offset += ( -flangeway - across ) / facing;
        }
        wing.emplace_back( at, offset );
    }
    // from where the flangeway of the other rail reaches the gauge line of ours
    knots::line result;
    std::size_t first { 0 };
    while( first + 1 < wing.size() && wing[ first + 1 ].y <= 0.0 ) {
        ++first;
    }
    if( wing[ first ].y <= 0.0 && first + 1 < wing.size() ) {
        auto const &a { wing[ first ] };
        auto const &b { wing[ first + 1 ] };
        result.emplace_back( glm::mix( a.x, b.x, a.y / ( a.y - b.y ) ), 0.0 );
        ++first;
    }
    else {
        result.emplace_back( start, 0.0 );
    }
    for( auto i { first }; i < wing.size(); ++i ) {
        result.emplace_back( wing[ i ].x, std::max( wing[ i ].y, 0.0 ) );
    }
    knots::sort( result );
    return knots::simplified( result, 0.0001 );
}

knots::line rolled_wing( knots::line const &Wing ) {

    knots::line rolled;
    for( auto const &knot : Wing ) {
        rolled.emplace_back( knot.x, rolled_band - knot.y );
    }
    return rolled;
}

// acute frog as designed for carving: the wing rail bent out along the flangeway of the other rail ends in a point, or run on a bit past its end into
// what planes it to one; the leg of the nose begins blunt past where the rails cross
void designed_acute( track_rails::rail_plan &Plan, TTrack const &Track, track_rails::rail_crossing const &Crossing, double const Toward, double const Slope, double const Wingend ) {

    auto const throat { Crossing.at + Toward * flangeway / Slope };
    auto const wing { flangeway_wing( Track, Crossing, throat, Wingend ) };
    track_rails::rail_cut cut;
    cut.at = Crossing.at;
    cut.carved = true;
    if( Toward < 0.0 ) {
        cut.end = Wingend;
        cut.endoffsets = wing;
        cut.endrolled = rolled_wing( wing );
        cut.endkind = track_rails::rail_end::point;
        cut.endanchor = throat;
        cut.endrunon = 1.0;
        cut.start = Crossing.at;
        cut.startkind = track_rails::rail_end::blunt;
    }
    else {
        cut.end = Crossing.at;
        cut.endkind = track_rails::rail_end::blunt;
        cut.start = Wingend;
        cut.startoffsets = wing;
        cut.startrolled = rolled_wing( wing );
        cut.startkind = track_rails::rail_end::point;
        cut.startanchor = throat;
        cut.startrunon = 1.0;
    }
    Plan.cuts.push_back( cut );
}

// obtuse crossing as designed for carving: the rail ends mitred where it meets the other one, the knuckle, and starts again where its point keeps
// out of the flangeway of the other rail
void designed_obtuse( track_rails::rail_plan &Plan, double const At, double const Into, glm::dvec2 const &Mark ) {

    auto const bend { knuckle_bend( Into ) };
    track_rails::rail_cut cut;
    cut.at = At;
    cut.end = At;
    cut.start = At;
    cut.carved = true;
    cut.obtuse = true;
    cut.mark = Mark;
    if( Into > 0.0 ) {
        cut.endbend = -bend;
        cut.endkind = track_rails::rail_end::mitre;
        cut.startkind = track_rails::rail_end::blunt;
    }
    else {
        cut.endkind = track_rails::rail_end::blunt;
        cut.startbend = bend;
        cut.startkind = track_rails::rail_end::mitre;
    }
    Plan.cuts.push_back( cut );
}

// a guard rail of a crossing at a middle angle gives way to the running rail and the flangeway of the other track
struct wide_cut {
    double from { 0.0 };
    double to { 0.0 };
    double inner { 0.0 };
    double outer { 0.0 };
};

void wide( track_rails::rail_plan &Plan, track_rails::rail_crossing const &Crossing, wide_cut const &Cut ) {

    auto const at { Crossing.at };
    auto const into { Crossing.into };
    auto const facing { Crossing.facing };
    auto const low { ( Cut.from + std::min( Cut.inner * facing, Cut.outer * facing ) - 0.005 ) / into };
    auto const high { ( Cut.to + std::max( Cut.inner * facing, Cut.outer * facing ) + 0.005 ) / into };
    track_rails::rail_cut cut;
    cut.at = at;
    cut.end = at + std::min( low, high );
    cut.start = at + std::max( low, high );
    cut.endmiter = { at + ( into > 0.0 ? Cut.from - 0.005 : Cut.to + 0.005 ) / into, facing / into };
    cut.startmiter = { at + ( into > 0.0 ? Cut.to + 0.005 : Cut.from - 0.005 ) / into, facing / into };
    Plan.cuts.push_back( cut );
}

track_rails::rail_cut corner_cut( track_rails::rail_crossing const &Crossing, double const Face, double const Toward, double const Head, double const Other ) {

    auto const mitre = [&]( double const Edge, double const Flange ) {
        auto const slope { Flange * Other / ( Toward * Head ) };
        return glm::dvec2 { Crossing.at + ( Edge - Face * slope ) / Crossing.into, ( slope + Crossing.facing ) / Crossing.into }; };
    auto const rail { mitre( 0.0, 1.0 ) };
    auto const guard { mitre( flangeway, -1.0 ) };
    auto const rising { Crossing.into > 0.0 };
    track_rails::rail_cut cut;
    cut.at = Crossing.at;
    cut.endmiter = ( rising ? rail : guard );
    cut.startmiter = ( rising ? guard : rail );
    cut.end = cut.endmiter.x + cut.endmiter.y * Face;
    cut.start = cut.startmiter.x + cut.startmiter.y * Face;
    cut.endbend = cut.endmiter.y;
    cut.startbend = cut.startmiter.y;
    return cut;
}

std::vector<track_rails::rail_fill> spacer_fills( track_rails::rail_fill const &Whole, double const From, double const To, knots::line const &Castings ) {

    knots::line pieces;
    for( auto const &casting : Castings ) {
        if( false == pieces.empty() && casting.x <= pieces.back().y ) {
            pieces.back().y = std::max( pieces.back().y, casting.y );
        }
        else {
            pieces.push_back( casting );
        }
    }
    knots::line spacers;
    auto const place = [&]( double const Start, double const End, double const Step ) {
        auto const count { static_cast<int>( std::floor( ( End - Start - 0.5 * spacer_length ) / spacer_pitch ) ) };
        for( int i = 1; i <= count; ++i ) {
            auto const at { ( Step > 0.0 ? Start : End ) + Step * i * spacer_pitch };
            spacers.emplace_back( at - 0.5 * spacer_length, at + 0.5 * spacer_length );
        } };
    auto const first { From + spacer_length };
    auto const last { To - spacer_length };
    if( pieces.empty() ) {
        place( first - spacer_pitch, last, 1.0 );
    }
    else {
        place( first, pieces.front().x, -1.0 );
        for( std::size_t i = 1; i < pieces.size(); ++i ) {
            auto const middle { 0.5 * ( pieces[ i - 1 ].y + pieces[ i ].x ) };
            place( pieces[ i - 1 ].y, middle, 1.0 );
            place( middle, pieces[ i ].x, -1.0 );
        }
        place( pieces.back().y, last, 1.0 );
    }
    pieces.insert( pieces.end(), spacers.begin(), spacers.end() );
    std::vector<track_rails::rail_fill> fills;
    for( auto const &piece : pieces ) {
        auto fill { Whole };
        fill.from = std::max( piece.x, From );
        fill.to = std::min( piece.y, To );
        fill.capfrom = true;
        fill.capto = true;
        if( fill.to - fill.from > 0.01 ) {
            fills.push_back( fill );
        }
    }
    return fills;
}

std::vector<track_rails::rail_fill> guard_fills( track_rails::guard_rail const &Guard, track_rails::rail_size const &Size ) {

    auto const offset { Size.web - ( flangeway + Size.head ) };
    track_rails::rail_fill whole;
    whole.web = true;
    whole.lift = ( Guard.through ? 0.0 : -0.001 );
    whole.inner = { { Guard.from, offset }, { Guard.to, offset } };
    whole.outer = { { Guard.from, Size.web }, { Guard.to, Size.web } };
    return spacer_fills( whole, Guard.from, Guard.to, Guard.castings );
}

std::vector<track_rails::rail_fill> check_fills( track_rails::check_rail const &Check, track_rails::rail_size const &Size ) {

    auto const from { Check.from + check_flare( Check ) };
    auto const to { Check.to - check_flare( Check ) };
    track_rails::rail_fill whole;
    whole.web = true;
    whole.inner = { { from, -check_flangeway }, { to, -check_flangeway } };
    whole.outer = { { from, Size.web }, { to, Size.web } };
    return spacer_fills( whole, from, to, {} );
}

bool goes_through( TTrack const &Track, TTrack const &Other ) {

    if( Track.name() != Other.name() ) {
        return Track.name() < Other.name();
    }
    auto const here { Track.location() };
    auto const there { Other.location() };
    return ( here.x != there.x ? here.x < there.x : here.z < there.z );
}

track_rails::rail_band groove_band( double const At, double const Into, double const Facing, bool const Through, track_rails::rail_size const &Other ) {

    auto const seam { 0.001 };
    track_rails::rail_band band { At, Into, Facing };
    band.whole = ( false == Through );
    band.from = ( Through ? 0.0 : seam - Other.tread );
    band.to = ( Through ? ( Other.grooved ? -Other.lip : flangeway ) : -Other.ledge - seam );
    return band;
}

void vee_fill( std::vector<track_rails::rail_fill> &Fills, double const At, double const Into, double const Facing, track_rails::rail_size const &Own, track_rails::rail_size const &Other, bool const Outwards ) {

    auto const seam { 0.001 };
    auto const sign { Outwards ? 1.0 : -1.0 };
    auto const edge { Outwards ? std::max( Own.head, Own.crown ) : Own.rim };
    auto const flat { Outwards ? Own.tread : Own.ledge };
    auto const inside { Outwards == ( Facing > 0.0 ) };
    auto const closest { inside ? -Other.rim : -std::max( Other.head, Other.crown ) };
    auto const rounded { inside ? Other.ledge - Other.rim : std::max( Other.head, Other.crown ) - Other.tread };
    auto const own { Outwards ? Own.high - edge : edge - Own.low };
    auto const other { inside ? Other.rim - Other.low : Other.high - std::max( Other.head, Other.crown ) };
    auto const slant { std::max( std::abs( Facing ), 0.1 ) };
    auto const gap { own + other / slant };
    auto const meets = [&]( double const Width ) {
        return At + ( closest + Facing * ( edge + sign * Width ) ) / Into; };
    auto const apex { meets( 0.0 ) };
    auto const end { meets( gap ) };
    auto const reach { seam + rounded / slant };
    track_rails::rail_fill fill;
    fill.from = std::min( apex, end );
    fill.to = std::max( apex, end );
    fill.capfrom = ( end < apex );
    fill.capto = ( end > apex );
    if( Own.grooved ) {
        fill.lift = std::min( Outwards ? Own.crest : Own.liptop, inside ? Other.liptop : Other.crest ) - Own.liptop;
    }
    std::vector<glm::dvec2> line { { apex, edge + sign * reach }, { end, edge + sign * ( gap + reach ) } };
    knots::sort( line );
    std::vector<glm::dvec2> fixed { { fill.from, flat - sign * seam }, { fill.to, flat - sign * seam } };
    fill.outer = ( Outwards ? line : fixed );
    fill.inner = ( Outwards ? fixed : line );
    Fills.push_back( fill );
}

track_rails::rail_tread wide_tread( track_rails::rail_crossing const &Crossing, material_handle const Material, double const Base, double const Head, double const Reach ) {

    auto const facing { std::abs( Crossing.facing ) > 0.02 ? Crossing.facing : std::copysign( 0.02, Crossing.facing ) };
    auto const slope { Crossing.into / facing };
    track_rails::rail_tread tread { 1e9, -1e9, Material };
    if( std::abs( slope ) > square_slope ) {
        for( auto const edge : { 0.0, -Reach } ) {
            auto const distance { Crossing.at + ( edge + Crossing.facing * ( Base + 0.5 * Head ) ) / Crossing.into };
            tread.from = std::min( tread.from, distance );
            tread.to = std::max( tread.to, distance );
        }
        return tread;
    }
    for( auto const across : { Base, Base + Head } ) {
        for( auto const edge : { tread_fade, -Reach - tread_fade } ) {
            auto const distance { Crossing.at + ( edge + Crossing.facing * across ) / Crossing.into };
            tread.from = std::min( tread.from, distance - 0.01 );
            tread.to = std::max( tread.to, distance + 0.01 );
        }
    }
    auto const line = [&]( double const Edge ) {
        auto const across = [&]( double const Distance ) {
            return ( ( Distance - Crossing.at ) * Crossing.into - Edge ) / facing - Base; };
        return knots::line { { tread.from, across( tread.from ) }, { tread.to, across( tread.to ) } }; };
    tread.gauge = line( 0.0 );
    tread.edge = line( -Reach );
    tread.below = ( facing > 0.0 );
    tread.fade = tread_fade * std::sqrt( 1.0 + slope * slope );
    return tread;
}

std::vector<glm::dvec2> tread_edge( TTrack const &Track, int const Path, int const Side, TTrack const &Other, int const Otherpath, int const Otherside, double const From, double const To, bool &Below, std::vector<glm::dvec2> &Gauge ) {

    auto const &ours { rail_line( Track, Path, Side > 0, false ) };
    auto const &theirs { rail_line( Other, Otherpath, Otherside > 0, false ) };
    glm::dvec3 point, tangent;
    ours.at( From, point, tangent );
    auto otherat { nearest_distance( theirs, point ) };
    std::vector<glm::dvec2> knots;
    auto below { 0 };
    auto const count { std::max( 2, static_cast<int>( std::ceil( ( To - From ) / 0.1 ) ) + 1 ) };
    for( int i = 0; i < count; ++i ) {
        auto distance { glm::mix( From, To, static_cast<double>( i ) / ( count - 1 ) ) };
        ours.at( distance, point, tangent );
        lateral( theirs, Otherside, point, otherat );
        glm::dvec3 line;
        theirs.at( otherat, line, tangent );
        auto const normal { theirs.normal( otherat, Otherside ) };
        auto at { distance };
        auto const gauge { lateral( ours, Side, line, at ) };
        Gauge.emplace_back( at, gauge );
        at = distance;
        auto const edge { lateral( ours, Side, line - normal * rolled_band, at ) };
        below += ( gauge < edge ? 1 : -1 );
        knots.emplace_back( at, edge );
    }
    knots::sort( knots );
    knots::sort( Gauge );
    Below = ( below > 0 );
    return knots;
}

} // namespace

namespace {

void merge_guard( std::vector<track_rails::guard_rail> &Guards, track_rails::guard_rail const &Guard ) {

    for( auto &span : Guards ) {
        if( Guard.from < span.to + 2.0 && Guard.to > span.from - 2.0 ) {
            span.from = std::min( span.from, Guard.from );
            span.to = std::max( span.to, Guard.to );
            span.plan.cuts.insert( span.plan.cuts.end(), Guard.plan.cuts.begin(), Guard.plan.cuts.end() );
            span.through = ( span.through && Guard.through );
            span.treads.insert( span.treads.end(), Guard.treads.begin(), Guard.treads.end() );
            span.castings.insert( span.castings.end(), Guard.castings.begin(), Guard.castings.end() );
            knots::sort( span.castings );
            return;
        }
    }
    Guards.push_back( Guard );
}

void join_opposite_guards( std::vector<track_rails::guard_rail> ( &Guards )[ 2 ] ) {

    knots::line spans[ 2 ];
    for( int side = 0; side < 2; ++side ) {
        for( auto const &guard : Guards[ side ] ) {
            spans[ side ].emplace_back( guard.from, guard.to );
        }
    }
    for( int side = 0; side < 2; ++side ) {
        for( auto &guard : Guards[ side ] ) {
            for( auto const &opposite : spans[ 1 - side ] ) {
                if( opposite.x < guard.to && opposite.y > guard.from ) {
                    guard.from = std::min( guard.from, opposite.x );
                    guard.to = std::max( guard.to, opposite.y );
                }
            }
        }
    }
}

bool framed( track_rails::rail_crossing const &Corner, track_rails::rail_crossing const &Other, int const Side ) {

    return Other.track == Corner.track && Other.path == Corner.path && Other.side == Side && Other.otherpath == Corner.otherpath;
}

} // namespace

class TTrack::rail_planner {

public:
// constructors
    rail_planner( TTrack &Track, bool const Crossings, bool const Carve );
// methods
    rail_layout lay_out();

private:
// methods
    void add_crossing( rail_crossing const &Crossing );
    void cross_grooved( rail_crossing const &Crossing, rail_size const &Other );
    void cross_wide( rail_crossing const &Crossing, rail_size const &Other );
    void acute_frog( rail_crossing const &Crossing, double const Toward );
    void movable_frog( rail_crossing const &Crossing, double const Push );
    void add_vee( rail_crossing const &Crossing, rail_size const &Other, bool const Filled );
    void frame_middle_crossings();
    track_rails::guard_rail frame_guard( rail_crossing const &Start, rail_crossing const &End ) const;
    void switch_frog();
    void add_tread( int const Path, int const Side, glm::dvec2 const &Span, TTrack const &Other, int const Otherpath, int const Anchor = 0, int const Otherside = 0 );
    glm::dvec2 shared_span( rail_crossing const &Crossing, rail_size const &Other ) const;
    rail_plan &plan_of( rail_crossing const &Crossing ) {
        return m_layout.plans[ Crossing.path ][ Crossing.side > 0 ]; }
// members
    TTrack &m_track;
    bool const m_crossings;
    bool const m_carve; // the parts of the rails at crossings carved out of what lies around them, or only designed
    rail_size const m_size;
    double const m_guardoffset;
    rail_layout m_layout;
    std::vector<rail_crossing> m_corners;
};

TTrack::rail_planner::rail_planner( TTrack &Track, bool const Crossings, bool const Carve ) :
    m_track { Track },
    m_crossings { Crossings },
    m_carve { Carve },
    m_size { Track.rail_dimensions() },
    m_guardoffset { -( flangeway + m_size.head ) }
{}

rail_layout TTrack::rail_planner::lay_out() {

    if( m_carve && m_crossings ) {
        if( auto const *designed { designed_layout( m_track ) } ) {
            m_layout = *designed;
            carve_plans( m_track, m_layout );
            return m_layout;
        }
    }
    std::vector<rail_crossing> crossings;
    if( m_crossings ) {
        crossings = m_track.rail_crossings();
    }
    m_layout.checks = ( m_crossings ? rail_checks( m_track ) : m_track.check_rails( crossings ) );
    if( m_crossings ) {
        for( auto const &crossing : crossings ) {
            add_crossing( crossing );
        }
        for( auto &pathguards : m_layout.guards ) {
            join_opposite_guards( pathguards );
        }
        for( int path = 0; path < 2; ++path ) {
            for( int side = 0; side < 2; ++side ) {
                for( auto const &guard : m_layout.guards[ path ][ side ] ) {
                    auto const fills { guard_fills( guard, m_size ) };
                    m_layout.fills[ path ][ side ].insert( m_layout.fills[ path ][ side ].end(), fills.begin(), fills.end() );
                }
            }
        }
        frame_middle_crossings();
    }
    if( m_track.eType == tt_Switch && cut_rails( m_track ) ) {
        for( int path = 0; path < 2; ++path ) {
            add_tread( path, -m_track.blade_side( path ), { 0.0, blade_length( m_track, 1 - path ) }, m_track, 1 - path, 0, -m_track.blade_side( path ) );
        }
    }
    if( m_track.eType == tt_Switch ) {
        switch_frog();
    }
    for( int path = 0; path < 2; ++path ) {
        for( int side = 0; side < 2; ++side ) {
            for( auto const &check : m_layout.checks[ path ][ side ] ) {
                if( false == replaced_by_wing( check, m_layout.wingends[ path ][ side ] ) ) {
                    auto const fills { check_fills( check, m_size ) };
                    m_layout.fills[ path ][ side ].insert( m_layout.fills[ path ][ side ].end(), fills.begin(), fills.end() );
                }
            }
        }
    }
    if( m_carve && m_crossings ) {
        carve_plans( m_track, m_layout );
    }
    return m_layout;
}

void TTrack::rail_planner::add_crossing( rail_crossing const &Crossing ) {

    if( std::find( m_layout.crossed.begin(), m_layout.crossed.end(), Crossing.track ) == m_layout.crossed.end() ) {
        m_layout.crossed.push_back( Crossing.track );
    }
    auto const theirs { Crossing.track->rail_dimensions() };
    if( m_size.grooved || theirs.grooved ) {
        cross_grooved( Crossing, theirs );
        return;
    }
    if( Crossing.angle > middle_angle && Crossing.angle <= wide_angle ) {
        m_corners.push_back( Crossing );
    }
    if( Crossing.angle > wide_angle ) {
        cross_wide( Crossing, theirs );
    }
    else if( Crossing.facing < 0.0 ) {
        acute_frog( Crossing, Crossing.into > 0.0 ? 1.0 : -1.0 );
    }
    else {
        glm::dvec3 point, tangent;
        rail_line( m_track, Crossing.path, Crossing.side > 0, false ).at( Crossing.at, point, tangent );
        designed_obtuse( plan_of( Crossing ), Crossing.at, Crossing.into, plan( point ) );
        auto const reach { ( rolled_band + m_size.head + tread_fade ) / std::max( std::tan( Crossing.angle ), 0.01 ) + 0.2 };
        add_tread( Crossing.path, Crossing.side, { Crossing.at - reach, Crossing.at + reach }, *Crossing.track, Crossing.otherpath, 0, Crossing.otherside );
    }
}

void TTrack::rail_planner::cross_grooved( rail_crossing const &Crossing, rail_size const &Other ) {

    auto const through { m_size.grooved != Other.grooved ? m_size.grooved : goes_through( Crossing.owner ? *Crossing.owner : m_track, *Crossing.track ) };
    m_layout.bands[ Crossing.path ][ Crossing.side > 0 ].push_back( groove_band( Crossing.at, Crossing.into, Crossing.facing, through, Other ) );
    if( m_size.grooved && Other.grooved && Crossing.angle <= middle_angle ) {
        add_vee( Crossing, Other, through );
    }
    else if( through ) {
        add_tread( Crossing.path, Crossing.side, shared_span( Crossing, Other ), *Crossing.track, Crossing.otherpath );
    }
}

void TTrack::rail_planner::cross_wide( rail_crossing const &Crossing, rail_size const &Other ) {

    plan_of( Crossing ).cuts.push_back( corner_cut( Crossing, 0.0, -1.0, m_size.head, Other.head ) );
    auto const material { m_track.shared_tread( Crossing.path, *Crossing.track, Crossing.otherpath ) };
    if( material != m_track.rail_material( Crossing.path ) ) {
        m_layout.treads[ Crossing.path ][ Crossing.side > 0 ].push_back( wide_tread( Crossing, material, 0.0, m_size.head, rolled_band ) );
    }
    auto const reach { ( flangeway + m_size.head + guard_reach ) / std::abs( Crossing.into ) };
    track_rails::guard_rail guard { Crossing.at - reach, Crossing.at + reach };
    guard.through = goes_through( m_track, *Crossing.track );
    auto const casting { ( flangeway + Other.head + casting_reach ) / std::abs( Crossing.into ) };
    guard.castings.emplace_back( Crossing.at - casting, Crossing.at + casting );
    guard.plan.cuts.push_back( corner_cut( Crossing, -flangeway, 1.0, m_size.head, Other.head ) );
    auto const theirs { Crossing.track->rail_material( Crossing.otherpath ) };
    guard.treads.push_back( wide_tread( Crossing, theirs != null_handle ? theirs : m_track.rail_material( Crossing.path ), m_guardoffset, m_size.head, rolled_band ) );
    guard.treads.back().fade = 0.0;
    merge_guard( m_layout.guards[ Crossing.path ][ Crossing.side > 0 ], guard );
}

void TTrack::rail_planner::acute_frog( rail_crossing const &Crossing, double const Toward ) {

    auto &plan { plan_of( Crossing ) };
    auto const slope { std::tan( Crossing.angle ) };
    m_layout.wingends[ Crossing.path ][ Crossing.side > 0 ].emplace_back( Crossing.at - Toward * wing_reach( slope ), -Toward );
    auto const extension { wing_extension( m_track, m_layout.checks, Crossing.path, Crossing.side, Crossing.at, Toward, slope, *Crossing.track, Crossing.otherpath, Crossing.otherside, Crossing.otherat, Crossing.aligned ? -Toward : Toward ) };
    designed_acute( plan, m_track, Crossing, Toward, slope, extension.empty() ? Crossing.at - Toward * wing_reach( slope ) : extension.back().x );
    // the nose is carved yet, it ends near where its legs get as wide as its tip
    auto const half { slope / ( 1.0 + std::sqrt( 1.0 + slope * slope ) ) };
    auto const nose { Crossing.at - Toward * ( 0.5 * nose_tip / half - 0.1 ) };
    auto const shared { Crossing.at - Toward * ( rolled_band + m_size.head + tread_fade ) / slope };
    add_tread( Crossing.path, Crossing.side, { std::min( nose, shared ), std::max( nose, shared ) }, *Crossing.track, Crossing.otherpath, Toward > 0.0 ? 1 : -1, Crossing.otherside );
}

void TTrack::rail_planner::add_vee( rail_crossing const &Crossing, rail_size const &Other, bool const Filled ) {

    std::vector<rail_fill> vee;
    vee_fill( vee, Crossing.at, Crossing.into, Crossing.facing, m_size, Other, true );
    vee_fill( vee, Crossing.at, Crossing.into, Crossing.facing, m_size, Other, false );
    glm::dvec2 const block { std::min( vee[ 0 ].from, vee[ 1 ].from ), std::max( vee[ 0 ].to, vee[ 1 ].to ) };
    m_layout.blocks[ Crossing.path ][ Crossing.side > 0 ].push_back( block );
    add_tread( Crossing.path, Crossing.side, block, *Crossing.track, Crossing.otherpath );
    if( Filled ) {
        auto &fills { m_layout.fills[ Crossing.path ][ Crossing.side > 0 ] };
        fills.insert( fills.end(), vee.begin(), vee.end() );
    }
}

// crossings at a middle angle: guard rails along the sides of the quadrilateral between the tracks, glued at its corners to the guard rails of the other track
void TTrack::rail_planner::frame_middle_crossings() {

    std::sort( m_corners.begin(), m_corners.end(), []( rail_crossing const &A, rail_crossing const &B ) { return A.at < B.at; } );
    for( std::size_t i = 0; i < m_corners.size(); ++i ) {
        auto const &start { m_corners[ i ] };
        if( start.into <= 0.0 ) {
            continue;
        }
        for( std::size_t j = i + 1; j < m_corners.size(); ++j ) {
            auto const &end { m_corners[ j ] };
            if( false == framed( start, end, start.side ) ) {
                continue;
            }
            if( end.into < 0.0 ) {
                auto const guard { frame_guard( start, end ) };
                if( guard.to - guard.from > 0.1 ) {
                    m_layout.guards[ start.path ][ start.side > 0 ].push_back( guard );
                }
            }
            break;
        }
    }
}

track_rails::guard_rail TTrack::rail_planner::frame_guard( rail_crossing const &Start, rail_crossing const &End ) const {

    auto const glued = [&]( rail_crossing const &Corner ) {
        return Corner.at + ( flangeway + std::max( m_guardoffset * Corner.facing, ( m_guardoffset + m_size.head ) * Corner.facing ) ) / Corner.into; };
    auto const gluedmiter = [&]( rail_crossing const &Corner ) {
        return glm::dvec2 { Corner.at + flangeway / Corner.into, Corner.facing / Corner.into }; };
    track_rails::guard_rail guard { glued( Start ), glued( End ) };
    guard.flarefrom = false;
    guard.flareto = false;
    guard.frommiter = gluedmiter( Start );
    guard.tomiter = gluedmiter( End );
    for( auto const *corner : { &Start, &End } ) {
        if( corner->facing < 0.0 ) {
            continue;
        }
        auto const forward { corner == &End };
        auto reach { forward ? guard.to : guard.from };
        for( auto const &opposite : m_corners ) {
            if( false == framed( Start, opposite, -Start.side ) ) {
                continue;
            }
            auto const margin { ( flangeway + m_size.head + 0.3 ) / std::abs( opposite.into ) };
            reach = forward ? std::max( reach, opposite.at + margin ) : std::min( reach, opposite.at - margin );
        }
        if( reach == ( forward ? guard.to : guard.from ) ) {
            continue;
        }
        wide( guard.plan, *corner, { -corner->track->rail_dimensions().head, flangeway, m_guardoffset, m_guardoffset + m_size.head } );
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
    return guard;
}

void TTrack::rail_planner::switch_frog() {

    auto const frog { m_track.frog() };
    if( false == frog.valid ) {
        return;
    }
    glm::dvec3 point, tangents[ 2 ];
    glm::dvec2 normals[ 2 ];
    for( int path = 0; path < 2; ++path ) {
        auto const &line { rail_line( m_track, path, frog.side[ path ] > 0, false ) };
        line.at( frog.at[ path ], point, tangents[ path ] );
        normals[ path ] = glm::normalize( plan( line.normal( frog.at[ path ], frog.side[ path ] ) ) );
    }
    auto const facing { glm::dot( normals[ 0 ], normals[ 1 ] ) };
    for( int path = 0; path < 2; ++path ) {
        rail_crossing crossing;
        crossing.track = &m_track;
        crossing.path = path;
        crossing.side = frog.side[ path ];
        crossing.at = frog.at[ path ];
        crossing.angle = frog.angle;
        crossing.into = glm::dot( glm::normalize( plan( tangents[ path ] ) ), normals[ 1 - path ] );
        crossing.facing = facing;
        crossing.otherpath = 1 - path;
        crossing.otherside = frog.side[ 1 - path ];
        crossing.otherat = frog.at[ 1 - path ];
        if( m_size.grooved ) {
            m_layout.bands[ path ][ crossing.side > 0 ].push_back( groove_band( crossing.at, crossing.into, facing, path == 0, m_size ) );
            add_vee( crossing, m_size, path == 0 );
            continue;
        }
        if( m_track.SwitchExtension->m_movablenose ) {
            auto const nosethrow { std::max( 0.0, m_track.SwitchExtension->m_nosethrow ) };
            movable_frog( crossing, path == 0 ? nosethrow : 1.0 - nosethrow );
            continue;
        }
        acute_frog( crossing, -1.0 );
    }
}

// the frog of a switch with a movable nose: the nose reaches back to the throat and lies against the wing rail of the track set,
// its leg of the other track pushed over the flangeway and bending back to its place towards the heel; no gap on the way of the wheels.
// Push: how much of the flangeway the leg of this path is pushed over, 0 when its track is set, 1 when the other one is
void TTrack::rail_planner::movable_frog( rail_crossing const &Crossing, double const Push ) {

    auto &plan { plan_of( Crossing ) };
    auto const slope { std::tan( Crossing.angle ) };
    auto const throat { Crossing.at - flangeway / slope };
    auto const length { movable_nose_length( slope ) };
    // how far the leg of the track not set is pushed over: the whole flangeway at the tip, nothing at the heel, smooth at both
    auto const pushed = [&]( double const At ) {
        auto const t { std::clamp( ( At - throat ) / length, 0.0, 1.0 ) };
        return flangeway * ( 1.0 - t * t * ( 3.0 - 2.0 * t ) ); };
    // the legs are carved as they lie with the first path set, the leg of the second one pushed over: the same whichever is set
    auto const flare { std::max( Crossing.at + wing_reach( slope ), throat + length + 0.5 ) };
    auto const wing { flangeway_wing( m_track, Crossing, throat, flare ) };
    knots::line carved, offsets;
    for( int i = 0; i <= static_cast<int>( std::ceil( length / 0.1 ) ); ++i ) {
        auto const at { std::min( throat + 0.1 * i, throat + length ) };
        carved.emplace_back( at, -pushed( at ) );
        offsets.emplace_back( at, -Push * pushed( at ) );
    }
    m_layout.wingends[ Crossing.path ][ Crossing.side > 0 ].emplace_back( flare, 1.0 );
    track_rails::rail_cut cut;
    cut.at = Crossing.at;
    cut.carved = true;
    cut.end = flare;
    cut.endoffsets = wing;
    cut.endrolled = rolled_wing( wing );
    cut.endkind = rail_end::point;
    cut.endanchor = throat;
    cut.start = throat;
    cut.startkind = rail_end::sharp;
    cut.throwable = true;
    if( Push > 0.0 ) {
        cut.startoffsets = offsets;
    }
    if( Crossing.path == 1 ) {
        cut.startcarve = carved;
    }
    plan.cuts.push_back( cut );
    if( Push <= 0.0 ) {
        auto const shared { Crossing.at + ( rolled_band + m_size.head + tread_fade ) / slope };
        add_tread( Crossing.path, Crossing.side, { throat, shared }, *Crossing.track, Crossing.otherpath, -1, Crossing.otherside );
    }
}

void TTrack::rail_planner::add_tread( int const Path, int const Side, glm::dvec2 const &Span, TTrack const &Other, int const Otherpath, int const Anchor, int const Otherside ) {

    auto const material { m_track.shared_tread( Path, Other, Otherpath ) };
    if( material == m_track.rail_material( Path ) || Span.y <= Span.x ) {
        return;
    }
    auto &treads { m_layout.treads[ Path ][ Side > 0 ] };
    if( Otherside == 0 ) {
        treads.push_back( { Span.x, Span.y, material, Anchor } );
        return;
    }
    rail_tread part { Span.x, Span.y, material, Anchor };
    part.edge = tread_edge( m_track, Path, Side, Other, Otherpath, Otherside, Span.x, Span.y, part.below, part.gauge );
    auto from { 1e9 };
    auto to { -1e9 };
    for( std::size_t i = 0; i < part.edge.size() && i < part.gauge.size(); ++i ) {
        auto const low { std::min( part.edge[ i ].y, part.gauge[ i ].y ) };
        auto const high { std::max( part.edge[ i ].y, part.gauge[ i ].y ) };
        if( low - tread_fade < m_size.head && high + tread_fade > 0.0 ) {
            from = std::min( from, part.edge[ i ].x );
            to = std::max( to, part.edge[ i ].x );
        }
    }
    part.from = std::max( Span.x, from - 0.1 );
    part.to = std::min( Span.y, to + 0.1 );
    if( part.to > part.from ) {
        treads.push_back( part );
    }
}

glm::dvec2 TTrack::rail_planner::shared_span( rail_crossing const &Crossing, rail_size const &Other ) const {

    glm::dvec2 span { 1e9, -1e9 };
    for( auto const theirs : { Other.grooved ? Other.ledge : 0.0, Other.grooved ? Other.tread : Other.head } ) {
        for( auto const ours : { m_size.grooved ? m_size.ledge : 0.0, m_size.grooved ? m_size.tread : m_size.head } ) {
            auto const distance { Crossing.at + ( Crossing.facing * ours - theirs ) / Crossing.into };
            span = { std::min( span.x, distance ), std::max( span.y, distance ) };
        }
    }
    return span;
}

track_rails::rail_layout TTrack::plan_rails( bool const Crossings, bool const Carve ) {

    return rail_planner( *this, Crossings, Carve ).lay_out();
}
