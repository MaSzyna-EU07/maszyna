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

// acute frog: the rail coming from the throat bends out as a wing rail, the rail going on behind the gap is a leg of the nose
void acute( track_rails::rail_plan &Plan, double const At, double const Toward, double const Slope, std::vector<glm::dvec2> const &Extension, std::vector<glm::dvec2> const &Legs ) {

    auto const throat { At + Toward * flangeway / Slope };
    auto const flare { At - Toward * wing_reach( Slope ) };
    auto const nose { Toward < 0.0 ? Legs.front().x : Legs.back().x };
    std::vector<glm::dvec2> wing { { throat, 0.0 }, { flare, std::abs( flare - throat ) * Slope } };
    wing.insert( wing.end(), Extension.begin(), Extension.end() );
    auto const wingend { Extension.empty() ? flare : Extension.back().x };
    std::vector<glm::dvec2> rolled;
    for( auto const &knot : wing ) {
        rolled.emplace_back( knot.x, rolled_band - knot.y );
    }
    std::sort( rolled.begin(), rolled.end(), []( glm::dvec2 const &A, glm::dvec2 const &B ) { return A.x < B.x; } );
    track_rails::rail_cut cut;
    cut.at = At;
    if( Toward < 0.0 ) {
        cut.end = wingend;
        cut.endoffsets = wing;
        cut.endrolled = rolled;
        cut.start = nose;
        cut.startextents = Legs;
    }
    else {
        cut.end = nose;
        cut.endextents = Legs;
        cut.start = wingend;
        cut.startoffsets = wing;
        cut.startrolled = rolled;
    }
    Plan.cuts.push_back( cut );
}

// obtuse crossing: the rail ends where it meets the other one and starts again pointed past its flangeway, a check rail beside the point guards the gap
void obtuse( track_rails::rail_plan &Plan, double const At, double const Into, track_rails::rail_size const &Size ) {

    auto const gap { flangeway / std::abs( Into ) };
    auto const full { Size.high + 0.0135 };
    auto const taper { ( flangeway + full ) / std::abs( Into ) };
    auto const bend { knuckle_bend( Into ) };
    track_rails::rail_cut cut;
    cut.at = At;
    if( Into > 0.0 ) {
        cut.end = At;
        cut.endbend = -bend;
        cut.start = At + gap;
        cut.startextents = { { At + gap, 0.006 }, { At + taper, full } };
    }
    else {
        cut.end = At - gap;
        cut.endextents = { { At - taper, full }, { At - gap, 0.006 } };
        cut.start = At;
        cut.startbend = bend;
    }
    Plan.cuts.push_back( cut );
}

// crossing at a wide angle: running rails are broken for the flanges of the other track, the ones of the track not going through also where the head of the other rail lies; guard rails give way to the running rails and flangeways of the other track, the ones of the track not going through also to its guard rails
void wide( track_rails::rail_plan &Plan, double const At, double const Into, double const Facing, double const From, double const To, double const Inner, double const Outer, std::vector<glm::dvec2> const &Fromline = {}, std::vector<glm::dvec2> const &Toline = {} ) {

    auto const low { ( From + std::min( Inner * Facing, Outer * Facing ) - 0.005 ) / Into };
    auto const high { ( To + std::max( Inner * Facing, Outer * Facing ) + 0.005 ) / Into };
    track_rails::rail_cut cut;
    cut.at = At;
    cut.end = At + std::min( low, high );
    cut.start = At + std::max( low, high );
    cut.endmiter = { At + ( Into > 0.0 ? From - 0.005 : To + 0.005 ) / Into, Facing / Into };
    cut.startmiter = { At + ( Into > 0.0 ? To + 0.005 : From - 0.005 ) / Into, Facing / Into };
    auto const glue = [&]( std::vector<glm::dvec2> const &Line, double const Edge, double &Position, glm::dvec2 &Miter, double &Bend, std::vector<glm::dvec2> &Fit, bool &Cap ) {
        if( Line.empty() ) {
            return;
        }
        Position = At + Edge / Into;
        Miter = {};
        Bend = Facing / Into;
        for( auto const &knot : Line ) {
            Fit.emplace_back( knot.x, ( knot.y - Edge ) / Into );
        }
        Cap = false; };
    glue( Into > 0.0 ? Fromline : Toline, Into > 0.0 ? From : To, cut.end, cut.endmiter, cut.endbend, cut.endfit, cut.endcap );
    glue( Into > 0.0 ? Toline : Fromline, Into > 0.0 ? To : From, cut.start, cut.startmiter, cut.startbend, cut.startfit, cut.startcap );
    Plan.cuts.push_back( cut );
}

std::vector<glm::dvec2> glue_line( std::vector<glm::dvec2> const &Outline, double const Base ) {

    std::vector<glm::dvec2> line;
    for( auto const &knot : Outline ) {
        line.emplace_back( knot.x, Base - knot.y );
    }
    return line;
}

track_rails::rail_fill guard_fill( track_rails::guard_rail const &Guard, track_rails::rail_size const &Size ) {

    auto const offset { Size.web - ( flangeway + Size.head ) };
    auto const flare { std::min( 0.3, 0.25 * ( Guard.to - Guard.from ) ) };
    track_rails::rail_fill fill;
    fill.web = true;
    fill.from = Guard.from;
    fill.to = Guard.to;
    fill.capfrom = true;
    fill.capto = true;
    fill.lift = ( Guard.through ? 0.0 : -0.001 );
    fill.inner = { { Guard.from, offset - ( Guard.flarefrom ? 0.02 : 0.0 ) }, { Guard.from + flare, offset }, { Guard.to - flare, offset }, { Guard.to, offset - ( Guard.flareto ? 0.02 : 0.0 ) } };
    fill.outer = { { Guard.from, Size.web }, { Guard.to, Size.web } };
    return fill;
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
    std::sort( line.begin(), line.end(), []( glm::dvec2 const &A, glm::dvec2 const &B ) { return A.x < B.x; } );
    std::vector<glm::dvec2> fixed { { fill.from, flat - sign * seam }, { fill.to, flat - sign * seam } };
    fill.outer = ( Outwards ? line : fixed );
    fill.inner = ( Outwards ? fixed : line );
    Fills.push_back( fill );
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
    auto const byposition = []( glm::dvec2 const &A, glm::dvec2 const &B ) { return A.x < B.x; };
    std::sort( knots.begin(), knots.end(), byposition );
    std::sort( Gauge.begin(), Gauge.end(), byposition );
    Below = ( below > 0 );
    return knots;
}

std::vector<glm::dvec2> nose_extents( TTrack const &Track, int const Path, int const Side, TTrack const &Other, int const Otherpath, int const Otherside, double const At, double const Toward, double const Slope, track_rails::rail_size const &Size ) {

    auto const half { Slope / ( 1.0 + std::sqrt( 1.0 + Slope * Slope ) ) };
    auto const tip { 0.5 * nose_tip };
    auto const apart { Size.high + 0.01 };
    auto const nose { At - Toward * tip / half };
    auto const blunt { At - Toward * apart / half };
    std::vector<glm::dvec2> straight { { std::min( nose, blunt ), Toward < 0.0 ? tip : apart }, { std::max( nose, blunt ), Toward < 0.0 ? apart : tip } };
    auto const &ours { rail_line( Track, Path, Side > 0, false ) };
    auto const &theirs { rail_line( Other, Otherpath, Otherside > 0, false ) };
    glm::dvec3 point, tangent;
    ours.at( nose, point, tangent );
    auto otherat { nearest_distance( theirs, point ) };
    auto extent { tip };
    auto const step { 0.1 };
    auto const limit { static_cast<int>( std::ceil( 3.0 * std::abs( blunt - nose ) / step ) ) };
    std::vector<glm::dvec2> knots;
    for( int i = 0; i <= limit; ++i ) {
        auto const distance { nose - Toward * step * i };
        ours.at( distance, point, tangent );
        auto const normal { ours.normal( distance, Side ) };
        for( int iteration = 0; iteration < 6; ++iteration ) {
            extent += 0.5 * ( lateral( theirs, Otherside, point - normal * extent, otherat ) - extent );
        }
        if( extent >= apart && false == knots.empty() ) {
            auto const &last { knots.back() };
            knots.emplace_back( glm::mix( last.x, distance, ( apart - last.y ) / ( extent - last.y ) ), apart );
            break;
        }
        knots.emplace_back( distance, std::max( extent, 0.0 ) );
    }
    if( knots.size() < 2 || knots.back().y < apart ) {
        return straight;
    }
    std::sort( knots.begin(), knots.end(), []( glm::dvec2 const &A, glm::dvec2 const &B ) { return A.x < B.x; } );
    return knots;
}

} // namespace

track_rails::rail_layout TTrack::plan_rails( bool const Crossings ) {

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
    auto const tread = [&]( int const Path, int const Side, glm::dvec2 const &Span, TTrack const &Other, int const Otherpath, int const Anchor = 0, int const Otherside = 0 ) {
        auto const material { shared_tread( Path, Other, Otherpath ) };
        if( material == rail_material( Path ) || Span.y <= Span.x ) {
            return;
        }
        if( Otherside == 0 ) {
            layout.treads[ Path ][ Side > 0 ].push_back( { Span.x, Span.y, material, Anchor } );
            return;
        }
        rail_tread part { Span.x, Span.y, material, Anchor };
        part.edge = tread_edge( *this, Path, Side, Other, Otherpath, Otherside, Span.x, Span.y, part.below, part.gauge );
        auto from { 1e9 };
        auto to { -1e9 };
        for( std::size_t i = 0; i < part.edge.size() && i < part.gauge.size(); ++i ) {
            auto const low { std::min( part.edge[ i ].y, part.gauge[ i ].y ) };
            auto const high { std::max( part.edge[ i ].y, part.gauge[ i ].y ) };
            if( low - tread_fade < size.head && high + tread_fade > 0.0 ) {
                from = std::min( from, part.edge[ i ].x );
                to = std::max( to, part.edge[ i ].x );
            }
        }
        part.from = std::max( Span.x, from - 0.1 );
        part.to = std::min( Span.y, to + 0.1 );
        if( part.to > part.from ) {
            layout.treads[ Path ][ Side > 0 ].push_back( part );
        }
    };
    auto const across = [&]( rail_crossing const &Crossing, rail_size const &Other ) {
        glm::dvec2 span { 1e9, -1e9 };
        for( auto const theirs : { Other.grooved ? Other.ledge : 0.0, Other.grooved ? Other.tread : Other.head } ) {
            for( auto const ours : { size.grooved ? size.ledge : 0.0, size.grooved ? size.tread : size.head } ) {
                auto const distance { Crossing.at + ( Crossing.facing * ours - theirs ) / Crossing.into };
                span = { std::min( span.x, distance ), std::max( span.y, distance ) };
            }
        }
        return span; };
    std::vector<frame_corner> corners;
    if( Crossings ) {
        for( auto const &crossing : crossings ) {
            auto &plan { plans[ crossing.path ][ crossing.side > 0 ] };
            if( std::find( layout.crossed.begin(), layout.crossed.end(), crossing.track ) == layout.crossed.end() ) {
                layout.crossed.push_back( crossing.track );
            }
            auto const theirs { crossing.track->rail_dimensions() };
            if( size.grooved || theirs.grooved ) {
                auto const through { size.grooved != theirs.grooved ? size.grooved : goes_through( crossing.owner ? *crossing.owner : *this, *crossing.track ) };
                layout.bands[ crossing.path ][ crossing.side > 0 ].push_back( groove_band( crossing.at, crossing.into, crossing.facing, through, theirs ) );
                if( size.grooved && theirs.grooved && crossing.angle <= middle_angle ) {
                    std::vector<rail_fill> vee;
                    vee_fill( vee, crossing.at, crossing.into, crossing.facing, size, theirs, true );
                    vee_fill( vee, crossing.at, crossing.into, crossing.facing, size, theirs, false );
                    glm::dvec2 const block { std::min( vee[ 0 ].from, vee[ 1 ].from ), std::max( vee[ 0 ].to, vee[ 1 ].to ) };
                    layout.blocks[ crossing.path ][ crossing.side > 0 ].push_back( block );
                    tread( crossing.path, crossing.side, block, *crossing.track, crossing.otherpath );
                    if( through ) {
                        auto &fills { layout.fills[ crossing.path ][ crossing.side > 0 ] };
                        fills.insert( fills.end(), vee.begin(), vee.end() );
                    }
                }
                else if( through ) {
                    tread( crossing.path, crossing.side, across( crossing, theirs ), *crossing.track, crossing.otherpath );
                }
                continue;
            }
            if( crossing.angle > middle_angle && crossing.angle <= wide_angle ) {
                corners.push_back( { crossing.track, crossing.path, crossing.side, crossing.otherpath, crossing.at, crossing.into, crossing.facing } );
            }
            if( crossing.angle > wide_angle ) {
                auto const other { crossing.track->rail_dimensions() };
                auto const through { goes_through( *this, *crossing.track ) };
                wide( plan, crossing.at, crossing.into, crossing.facing, through ? 0.0 : -other.head, flangeway, 0.0, railhead, through ? std::vector<glm::dvec2>{} : glue_line( other.field, 0.0 ) );
                if( through ) {
                    tread( crossing.path, crossing.side, across( crossing, other ), *crossing.track, crossing.otherpath );
                }
                auto const reach { ( flangeway + railhead + 0.3 ) / std::abs( crossing.into ) };
                auto &spans { guards[ crossing.path ][ crossing.side > 0 ] };
                track_rails::guard_rail guard { crossing.at - reach, crossing.at + reach };
                guard.through = through;
                wide( guard.plan, crossing.at, crossing.into, crossing.facing, -other.head, through ? flangeway : flangeway + other.head, guardoffset, guardoffset + railhead, glue_line( other.field, 0.0 ), through ? std::vector<glm::dvec2>{} : glue_line( other.gauge, flangeway + other.head ) );
                auto merged { false };
                for( auto &span : spans ) {
                    if( guard.from < span.to + 2.0 && guard.to > span.from - 2.0 ) {
                        span.from = std::min( span.from, guard.from );
                        span.to = std::max( span.to, guard.to );
                        span.plan.cuts.insert( span.plan.cuts.end(), guard.plan.cuts.begin(), guard.plan.cuts.end() );
                        span.through = ( span.through && guard.through );
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
                acute( plan, crossing.at, toward, slope,
                    wing_extension( *this, checks, crossing.path, crossing.side, crossing.at, toward, slope, *crossing.track, crossing.otherpath, crossing.otherside, crossing.otherat, crossing.aligned ? -toward : toward ),
                    nose_extents( *this, crossing.path, crossing.side, *crossing.track, crossing.otherpath, crossing.otherside, crossing.at, toward, slope, size ) );
                auto const nose { toward < 0.0 ? plan.cuts.back().start : plan.cuts.back().end };
                auto const shared { crossing.at - toward * ( rolled_band + size.head + tread_fade ) / slope };
                tread( crossing.path, crossing.side, { std::min( nose, shared ), std::max( nose, shared ) }, *crossing.track, crossing.otherpath, toward > 0.0 ? 1 : -1, crossing.otherside );
            }
            else {
                obtuse( plan, crossing.at, crossing.into, size );
                auto const reach { ( rolled_band + size.head + tread_fade ) / std::max( std::tan( crossing.angle ), 0.01 ) + 0.2 };
                tread( crossing.path, crossing.side, { crossing.at - reach, crossing.at + reach }, *crossing.track, crossing.otherpath, 0, crossing.otherside );
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
        for( int path = 0; path < 2; ++path ) {
            for( int side = 0; side < 2; ++side ) {
                for( auto const &guard : guards[ path ][ side ] ) {
                    layout.fills[ path ][ side ].push_back( guard_fill( guard, size ) );
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
                    track_rails::guard_rail guard { glued( start ), glued( end ) };
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
    if( eType == tt_Switch && cut_rails( *this ) ) {
        for( int path = 0; path < 2; ++path ) {
            tread( path, -blade_side( path ), { 0.0, blade_length( *this, 1 - path ) }, *this, 1 - path, 0, -blade_side( path ) );
        }
    }
    if( eType == tt_Switch ) {
        auto const frog { this->frog() };
        if( frog.valid ) {
            glm::dvec3 point, tangents[ 2 ];
            glm::dvec2 normals[ 2 ];
            for( int path = 0; path < 2; ++path ) {
                auto const &line { rail_line( *this, path, frog.side[ path ] > 0, false ) };
                line.at( frog.at[ path ], point, tangents[ path ] );
                normals[ path ] = glm::normalize( plan( line.normal( frog.at[ path ], frog.side[ path ] ) ) );
            }
            auto const facing { glm::dot( normals[ 0 ], normals[ 1 ] ) };
            auto const slope { std::tan( frog.angle ) };
            for( int path = 0; path < 2; ++path ) {
                auto const into { glm::dot( glm::normalize( plan( tangents[ path ] ) ), normals[ 1 - path ] ) };
                auto &fills { layout.fills[ path ][ frog.side[ path ] > 0 ] };
                if( size.grooved ) {
                    layout.bands[ path ][ frog.side[ path ] > 0 ].push_back( groove_band( frog.at[ path ], into, facing, path == 0, size ) );
                    std::vector<rail_fill> vee;
                    vee_fill( vee, frog.at[ path ], into, facing, size, size, true );
                    vee_fill( vee, frog.at[ path ], into, facing, size, size, false );
                    glm::dvec2 const block { std::min( vee[ 0 ].from, vee[ 1 ].from ), std::max( vee[ 0 ].to, vee[ 1 ].to ) };
                    layout.blocks[ path ][ frog.side[ path ] > 0 ].push_back( block );
                    tread( path, frog.side[ path ], block, *this, 1 - path );
                    if( path == 0 ) {
                        fills.insert( fills.end(), vee.begin(), vee.end() );
                    }
                    continue;
                }
                wingends[ path ][ frog.side[ path ] > 0 ].emplace_back( frog.at[ path ] + wing_reach( slope ), 1.0 );
                acute( plans[ path ][ frog.side[ path ] > 0 ], frog.at[ path ], -1.0, slope,
                    wing_extension( *this, checks, path, frog.side[ path ], frog.at[ path ], -1.0, slope, *this, 1 - path, frog.side[ 1 - path ], frog.at[ 1 - path ], 1.0 ),
                    nose_extents( *this, path, frog.side[ path ], *this, 1 - path, frog.side[ 1 - path ], frog.at[ path ], -1.0, slope, size ) );
                auto const nose { plans[ path ][ frog.side[ path ] > 0 ].cuts.back().start };
                auto const shared { frog.at[ path ] + ( rolled_band + size.head + tread_fade ) / slope };
                tread( path, frog.side[ path ], { std::min( nose, shared ), std::max( nose, shared ) }, *this, 1 - path, -1, frog.side[ 1 - path ] );
            }
        }
    }
    return layout;
}
