/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <array>
#include <numbers>
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
double constexpr guard_bevel { std::numbers::sqrt3 }; // how far along the flared ends of guard rails are cut back across them, from their face toward the running rail: 30 degrees to the rail
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

// a value for each rail of a track: path, right side
template <typename Type_>
using path_sides = std::array<std::array<Type_, 2>, 2>;

// where the inner rails of the two paths of a switch cross, found from their gauge lines behind the blades
struct switch_frog {
    bool valid { false };
    std::array<int, 2> side {}; // rail profile of the crossing rail of each path, 1 right, -1 left
    std::array<double, 2> at {}; // distance of the crossing along each path
    std::array<double, 2> blades {}; // distance the blades take along each path
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
using check_rail_set = path_sides<std::vector<check_rail>>;
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
// one of the two parts of a rail broken where another one crosses it, at the break: the part coming from below as it ends there, or the part
// going on as it starts there
struct rail_cut_part {
    knots::line offsets;
    knots::line extents;
    knots::line inner;
    knots::line rolled; // how far out from the gauge line the wheels roll on the head
    knots::line worn; // from how far out from the gauge line the wheels of a rail beside roll on the head, where it's planed flush against that rail
    material_handle wornby { 0 }; // what the wheels rolling on it from beside polish the head to, the brightest of them
    glm::dvec2 miter { 0.0 };
    double bend { 0.0 };
    bool cap { true };
    // what carving the part against the rails around takes from the design of the crossing
    rail_end kind { rail_end::square };
    double anchor { 0.0 }; // from where on a part planed down to its point can end
    double runon { 0.0 }; // how far past its end a wing rail can go on to be planed down to its point
    knots::line carve; // offsets the part is carved with where they aren't those it's laid with: a leg of a movable nose, as it lies with the first path set
};
// a rail broken where another one crosses it: the part coming from below ends at end, the part going on starts at start
struct rail_cut {
    double at { 0.0 };
    double end { 0.0 };
    double start { 0.0 };
    rail_cut_part ending; // the part coming from below
    rail_cut_part starting; // the part going on
    bool carved { false }; // the lines of the parts are left to carving
    bool throwable { false }; // the part going on moves with the switch: a leg of a movable nose
    glm::dvec2 mark { 0.0 }; // where the rails cross, in plan: the parts at an obtuse crossing aren't carved against the other rail's there, they're designed to meet
    bool obtuse { false };
};
struct rail_plan {
    std::vector<rail_cut> cuts;
};
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
    knots::line inner;
    knots::line outer;
    bool capfrom { false };
    bool capto { false };
    double lift { 0.0 };
    bool web { false };
};
// which rail of the track is laid and how: its path and side, the length of its texture, from how far along the path on, its ends left open
struct rail_course {
    int path { 0 };
    int side { 0 };
    float texturelength { 0.f };
    double from { 0.0 };
    bool openstart { false };
    bool openend { false };
};
struct rail_layout {
    path_sides<rail_plan> plans;
    path_sides<std::vector<rail_band>> bands;
    path_sides<std::vector<rail_fill>> fills;
    path_sides<knots::line> blocks;
    path_sides<std::vector<rail_tread>> treads;
    path_sides<std::vector<guard_rail>> guards;
    path_sides<knots::line> wingends; // ends of the wing rails of other rails running along each rail, and which way they run there
    check_rail_set checks;
    std::vector<TTrack *> crossed; // tracks whose rails cross the rails of this one
};

} // namespace track_rails
