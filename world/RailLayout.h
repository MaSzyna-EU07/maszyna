/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <algorithm>
#include <array>
#include <vector>
#include <glm/glm.hpp>
#include "utilities/Classes.h"
#include "world/Knots.h"

class TTrack;

namespace track_rails {

double constexpr flangeway { 0.045 };
double constexpr blend { 0.015 };
double constexpr tread_fade { 0.02 };
double constexpr wheel_width { 0.14 };
double constexpr back_to_back { 1.36 };
double constexpr rolled_band { wheel_width - ( 1.435 - back_to_back ) / 2.0 };
double constexpr wheel_play { 0.005 }; // how far the wheels wander across each way, by the play of their flanges in the gauge
double constexpr nose_tip { 0.012 };
double constexpr middle_angle { 0.35 };
double constexpr wide_angle { 0.785 };
double constexpr guard_reach { 1.0 };
double constexpr guard_flare { 0.6 };
double constexpr casting_reach { 0.3 };
double constexpr square_slope { 3.0 };
double constexpr spacer_length { 0.12 };
double constexpr spacer_pitch { 0.5 };
double constexpr check_flangeway { 0.041 };
double constexpr guard_planing { 0.015 }; // how far the ends of guard rails are planed off their face toward the running rail
double constexpr guard_bevel { 1.7320508 }; // how far along the flared ends of guard rails are cut back across them, from their face toward the running rail: 30 degrees to the rail
double constexpr blade_throw { 0.160 };
double constexpr blade_clearance { 0.058 };
double constexpr blade_point { 0.003 };
double constexpr blade_nose { 0.005 };
double constexpr blade_planed { 0.125 };
double constexpr blade_tangent { 0.00175 };
double constexpr blade_mark { 0.010 };
double constexpr blade_gap { 0.0005 };
double constexpr blade_lowered { 0.050 };
double constexpr blade_spacing { 0.5 };
double constexpr blade_runon_margin { 0.2 }; // how far short of the end of the plain track a blade running on into it ends
double constexpr wing_low { 1.5 }; // how far before the throat the wing rails are of the low section of the blades, raised on their timbers as the blades
double constexpr wing_forged { 0.5 }; // how long the end of a wing rail forged to the plain rail is, going down to it
// the low section of the blades, Zu1-60 (60E1A1) against 60E1: its web 44 mm thick against 16.5, its foot 55 mm inwards and 85 mm outwards
// of the middle against 75 both ways, the edges of its foot 20 mm high against 11.5, its web from 36 mm up against 30.6
double constexpr low_web { 44.0 / 16.5 };
double constexpr low_inward { 55.0 / 75.0 };
double constexpr low_outward { 85.0 / 75.0 };
double constexpr low_edge { 20.0 / 11.5 };
double constexpr low_root { 36.0 / 30.6 };

// where the inner rails of the two paths of a switch cross, found from their gauge lines behind the blades
struct switch_frog {
    bool valid { false };
    int side[ 2 ] {}; // rail profile of the crossing rail of each path, 1 right, -1 left
    double at[ 2 ] {}; // distance of the crossing along each path
    double blades[ 2 ] {}; // distance the blades take along each path
    double angle { 0.0 };
};
struct switch_blade {
    bool valid { false };
    bool rigid { false };
    double tip { 0.0 };
    double full { 0.0 };
    double heel { 0.0 };
    double drop { 0.0 };
    double opening { 0.0 };
    double openingat { 0.0 };
    std::vector<glm::dvec2> gauge;
    std::vector<glm::dvec2> extents;
    std::vector<glm::dvec2> drops;
    std::vector<glm::dvec2> bends;
};
// where a rail of this track crosses a rail of another one, found from their gauge lines
struct rail_crossing {
    TTrack *track { nullptr };
    int path { 0 };
    int side { 0 }; // rail profile of the crossing rail, 1 right, -1 left
    double at { 0.0 }; // distance of the crossing along the path, past the ends where the track goes on
    double angle { 0.0 };
    double into { 0.0 }; // the path direction against the gauge normal of the other rail, above zero: going into the other track
    double facing { 0.0 }; // the gauge normals of both rails against each other, below zero: an acute frog
    bool nose { false }; // at an acute frog the rail goes on as the nose rail, the other one as the rail beside it
    int otherpath { 0 };
    int otherside { 0 }; // rail profile of the other rail, 1 right, -1 left
    double otherat { 0.0 }; // distance of the crossing along the other path
    bool aligned { true }; // both paths run the same way at the crossing
    TTrack *owner { nullptr };
};
// the size of the rail profile, out from the gauge face: positive outwards, the side of the head away from the track
struct rail_size {
    double head { 0.0735 }; // how far the head reaches out
    double shoulder { 0.005 }; // how far in from the outer edge of the head its top is rounded off
    double height { 0.172 };
    double low { -0.0385 }; // the reach of the profile inwards
    double high { 0.1115 }; // the reach of the profile outwards
    bool grooved { false };
    double groove { 0.0 };
    double lip { 0.0 };
    double rim { 0.0 };
    double crown { 0.0 };
    double liptop { 0.0 };
    double crest { 0.0 };
    double tread { 0.0735 };
    double ledge { 0.0 };
    double web { 0.0365 };
    double neck { -0.0467 };
};
// a check rail beside a rail, how far its ends are bent away from the running rail
struct check_rail {
    double from { 0.0 };
    double to { 0.0 };
    double flarefrom { 0.0 };
    double flareto { 0.0 };
    double frombend { 0.0 };
    double tobend { 0.0 };
};
using check_rail_set = std::array<std::array<std::vector<check_rail>, 2>, 2>; // path, right side
inline double check_flare( check_rail const &Check ) {

    return std::min( guard_flare, 0.4 * ( Check.to - Check.from ) );
}
// how the end of a part of a rail planed by the rails around it is made: square where it's put, a point where the head is planed down to nothing
// (a wing rail), blunt where it's wide enough for the wheels running at it (a leg of the nose, a point rail), sharp where it's put as it gets wider
// (a leg of a movable nose), mitred against the end of the other rail (an obtuse crossing)
enum class rail_end {
    square,
    point,
    blunt,
    sharp,
    mitre
};
// a rail broken where another one crosses it: the part coming from below ends at end, the part going on starts at start
struct rail_cut {
    double at { 0.0 };
    double end { 0.0 };
    double start { 0.0 };
    knots::line endoffsets, endextents, startoffsets, startextents;
    knots::line endrolled, startrolled; // how far out from the gauge line the wheels roll on the head, of the part coming from below and of the part going on
    glm::dvec2 endmiter { 0.0 };
    glm::dvec2 startmiter { 0.0 };
    double endbend { 0.0 };
    double startbend { 0.0 };
    knots::line endinner, startinner;
    bool endcap { true };
    bool startcap { true };
    knots::line endworn, startworn; // from how far out from the gauge line the wheels of a rail beside roll on the head, where it's planed flush against that rail
    material_handle endwornby { 0 }; // what the wheels rolling on it from beside polish the head to, the brightest of them
    material_handle startwornby { 0 };
    // what carving the parts against the rails around takes from the design of the crossing
    bool carved { false }; // the lines of the parts are left to carving
    rail_end endkind { rail_end::square };
    rail_end startkind { rail_end::square };
    double endanchor { 0.0 }; // from where on a part planed down to its point can end
    double startanchor { 0.0 };
    double endrunon { 0.0 }; // how far past its end a wing rail can go on to be planed down to its point
    double startrunon { 0.0 };
    knots::line endcarve, startcarve; // offsets the parts are carved with where they aren't those they're laid with: the legs of a movable nose, as they lie with the first path set
    bool throwable { false }; // the part going on moves with the switch: a leg of a movable nose
    glm::dvec2 mark { 0.0 }; // where the rails cross, in plan: the parts at an obtuse crossing aren't carved against the other rail's there, they're designed to meet
    bool obtuse { false };
};
struct rail_plan {
    std::vector<rail_cut> cuts;
};
// how far out a part of a rail between two cuts is laid: as it goes on from the one and comes to the other, once where both are carved, which
// gives both its lines
inline knots::line part_offsets( rail_cut const *Before, rail_cut const *After ) {

    knots::line result { Before != nullptr ? Before->startoffsets : knots::line {} };
    if( After != nullptr && false == ( Before != nullptr && Before->carved && After->carved ) ) {
        result.insert( result.end(), After->endoffsets.begin(), After->endoffsets.end() );
    }
    knots::sort( result );
    return result;
}
// how far the foot of a part of a rail between two cuts is raised, Rise along a wing rail of the low section: from wing_low before the throat on,
// down to the plain rail past it along its forged end; the rail whole along Stock, the stock rail of a blade there
inline knots::line part_rises( rail_cut const *Before, rail_cut const *After, double const Rise, glm::dvec2 const &Stock ) {

    knots::line result;
    auto const going { Before != nullptr && Before->startkind == rail_end::point };
    auto const coming { After != nullptr && After->endkind == rail_end::point };
    if( Rise <= 0.0 || ( false == going && false == coming ) ) {
        return result;
    }
    auto const lowto { going ? Before->startanchor + wing_low : 0.0 };
    auto const lowfrom { coming ? After->endanchor - wing_low : 0.0 };
    auto const through { going && coming && lowto + 2.0 * wing_forged >= lowfrom };
    auto const stock { Stock.y > Stock.x };
    // the share of the rise: whole along the low section, none along the stock rail, between along the forged ends
    auto const share = [&]( double const At ) {
        auto const low { through ? 1.0 : std::max(
            going ? std::clamp( ( lowto + wing_forged - At ) / wing_forged, 0.0, 1.0 ) : 0.0,
            coming ? std::clamp( ( At - lowfrom + wing_forged ) / wing_forged, 0.0, 1.0 ) : 0.0 ) };
        auto const whole { stock ? std::clamp( std::max( Stock.x - At, At - Stock.y ) / wing_forged, 0.0, 1.0 ) : 1.0 };
        return std::min( low, whole ); };
    std::array<double, 8> places { lowto, lowto + wing_forged, lowfrom - wing_forged, lowfrom, Stock.x - wing_forged, Stock.x, Stock.y, Stock.y + wing_forged };
    std::sort( places.begin(), places.end() );
    for( auto const at : places ) {
        if( result.empty() || at > result.back().x + 1e-6 ) {
            result.emplace_back( at, Rise * share( at ) );
        }
    }
    return result;
}
struct rail_tread {
    double from { 0.0 };
    double to { 0.0 };
    material_handle material { 0 };
    int anchor { 0 };
    knots::line edge;
    knots::line gauge;
    bool below { false };
    double fade { tread_fade };
};
// guard rails of a crossing at a wide angle: the rail profile laid inside the running rail across the flangeway, broken like the rails
struct guard_rail {
    double from { 0.0 };
    double to { 0.0 };
    rail_plan plan;
    bool flarefrom { true };
    bool flareto { true };
    glm::dvec2 frommiter { 0.0 };
    glm::dvec2 tomiter { 0.0 };
    bool through { true };
    std::vector<rail_tread> treads;
    knots::line castings;
};
struct rail_band {
    double at { 0.0 };
    double into { 0.0 };
    double facing { 0.0 };
    double from { 0.0 };
    double to { 0.0 };
    bool whole { true };
};
struct rail_fill {
    double from { 0.0 };
    double to { 0.0 };
    knots::line inner, outer;
    bool capfrom { false };
    bool capto { false };
    double lift { 0.0 };
    bool web { false };
};
struct rail_layout {
    rail_plan plans[ 2 ][ 2 ]; // path, right side
    std::vector<rail_band> bands[ 2 ][ 2 ];
    std::vector<rail_fill> fills[ 2 ][ 2 ];
    knots::line blocks[ 2 ][ 2 ];
    std::vector<rail_tread> treads[ 2 ][ 2 ];
    std::vector<guard_rail> guards[ 2 ][ 2 ];
    knots::line wingends[ 2 ][ 2 ]; // ends of the wing rails of other rails running along each rail, and which way they run there
    check_rail_set checks;
    std::vector<TTrack *> crossed; // tracks whose rails cross the rails of this one
};

} // namespace track_rails
