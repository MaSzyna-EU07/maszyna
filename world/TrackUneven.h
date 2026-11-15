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
#include <charconv>
#include <cmath>
#include <sstream>
#include <string>
#include <string_view>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

// Track unevenness: a displacement field over the plan of the scenery, so everything at one place moves together: joined tracks, the
// paths of a switch, sleepers, axles. Waves 5-42 m long, shares after the German track spectrum with the long waves cut down; up and down to 83 m.
// The "uneven" value is the peak deviation of a rail on a 10 m chord, as in PLK Id-14 appendix 1; see test_track_uneven.cpp.
namespace track_uneven {

inline constexpr float most { 100.f }; // largest value of the attributes, mm
inline constexpr double stray_per_mm { 0.00145 }; // field amplitude per mm of "uneven", m
inline constexpr double cross_share { 0.53 }; // share of the field amplitude the rails settle apart by
inline constexpr double fade { 20.0 }; // length over which a track goes from the value at its end to its own, m
inline constexpr double dip_share { 0.4 }; // joint dip depth per mm of "uneven", mm
inline constexpr double dip_reach { 3.0 }; // reach of the dip on each side of a joint, m

// the value of each component of the unevenness, mm
struct amounts {
    double up { 0.0 };
    double side { 0.0 };
    double cross { 0.0 };
};

inline amounts least( amounts const &Left, amounts const &Right ) {

    return { std::min( Left.up, Right.up ), std::min( Left.side, Right.side ), std::min( Left.cross, Right.cross ) };
}

// which of the other sources of the cab shake the ride on a track stands in for
struct shake_sources {
    bool sway { false }; // from the lateral acceleration
    bool jolts { false }; // drawn at random
};

// sideways only with something going sideways. Cross level turns the body, and the dips at the joints come and go: neither makes up for any of them
inline shake_sources replaced_shake( amounts const &Amount ) {

    return { Amount.side > 0.0, Amount.up > 0.0 || Amount.side > 0.0 };
}

struct wave {
    double x;
    double z;
    double phase;
    double share;
};

// the numbers of an entry like "uneven 14;35", negative where left out
template <std::size_t Count>
std::array<float, Count> read_fields( std::string_view const Text ) {

    std::array<float, Count> result;
    result.fill( -1.f );
    auto rest { Text };
    for( auto &field : result ) {
        auto const end { std::min( rest.find( ';' ), rest.size() ) };
        if( auto value { 0.f }; end > 0 && std::from_chars( rest.data(), rest.data() + end, value ).ec == std::errc {} ) {
            field = std::clamp( value, 0.f, most );
        }
        if( end == rest.size() ) {
            break;
        }
        rest.remove_prefix( end + 1 );
    }
    return result;
}

// and back, with the ones left out at the end dropped
template <std::size_t Count>
std::string write_fields( std::array<float, Count> const &Fields ) {

    auto count { Count };
    while( count > 1 && Fields[ count - 1 ] < 0.f ) {
        --count;
    }
    std::ostringstream text;
    for( std::size_t index = 0; index < count; ++index ) {
        if( index > 0 ) {
            text << ';';
        }
        if( Fields[ index ] >= 0.f ) {
            text << Fields[ index ];
        }
    }
    return text.str();
}

// the values of the attribute, mm, negative where not given. Cross level and the dip at joints for plain tracks, the path p3-p4 for switches
struct attribute {
    float value { 0.f };
    float up { -1.f };
    float side { -1.f };
    float cross { -1.f };
    float dip { -1.f };
    float branch { -1.f };
    float branchup { -1.f };
    float branchside { -1.f };
};

// a plain track: value;up;side;cross;dip. A switch: p1-p2;p3-p4;up of p1-p2;side of p1-p2;up of p3-p4;side of p3-p4
inline attribute read_attribute( std::string_view const Text, bool const Switch ) {

    auto const fields { read_fields<6>( Text ) };
    attribute result;
    result.value = std::max( fields[ 0 ], 0.f );
    if( Switch ) {
        result.branch = fields[ 1 ];
        result.up = fields[ 2 ];
        result.side = fields[ 3 ];
        result.branchup = fields[ 4 ];
        result.branchside = fields[ 5 ];
    }
    else {
        result.up = fields[ 1 ];
        result.side = fields[ 2 ];
        result.cross = fields[ 3 ];
        result.dip = fields[ 4 ];
    }
    return result;
}

// in the layout of the entry it goes into, which can be of another type than the track the values come from
inline std::string write_attribute( attribute const &Values, bool const Switch ) {

    auto const fields { Switch ?
        std::array<float, 6> { Values.value, Values.branch, Values.up, Values.side, Values.branchup, Values.branchside } :
        std::array<float, 6> { Values.value, Values.up, Values.side, Values.cross, Values.dip, -1.f } };
    if( Values.value <= 0.f && std::all_of( fields.begin() + 1, fields.end(), []( float const Field ) { return Field < 0.f; } ) ) {
        return {};
    }
    return "uneven " + write_fields( fields ) + ' ';
}

// what a path comes to from the values: what's not given takes the first value, p3-p4 those of p1-p2. Cross level on plain tracks only
inline amounts declared( attribute const &Values, bool const Plain, int const Path ) {

    auto const given = []( float const Part, double const Otherwise ) {
        return Part >= 0.f ? static_cast<double>( Part ) : Otherwise; };
    amounts result {
        given( Values.up, Values.value ),
        given( Values.side, Values.value ),
        Plain ? given( Values.cross, Values.value ) : 0.0 };
    if( Path == 1 ) {
        result.up = given( Values.branchup, given( Values.branch, result.up ) );
        result.side = given( Values.branchside, given( Values.branch, result.side ) );
    }
    return result;
}

template <std::size_t Count>
using wave_set = std::array<wave, Count>;

// the waves run in all directions of the plan, so no direction of a track is favoured
template <std::size_t Count>
wave_set<Count> make_waves( double const Turn, std::array<double, Count> const &Lengths, std::array<double, Count> const &Shares ) {

    auto const golden { glm::pi<double>() * ( 3.0 - std::sqrt( 5.0 ) ) };
    wave_set<Count> result {};
    for( std::size_t i = 0; i < Count; ++i ) {
        auto const number { glm::two_pi<double>() / Lengths[ i ] };
        auto const place { static_cast<double>( i ) };
        auto const heading { Turn + golden * place };
        result[ i ] = { number * std::cos( heading ), number * std::sin( heading ), golden * ( place + 1.0 ) * ( 1.0 + 3.0 * Turn ), Shares[ i ] };
    }
    return result;
}

inline constexpr std::array<double, 8> wave_lengths { 5.3, 7.1, 9.7, 13.1, 17.9, 23.9, 31.7, 41.9 };
inline constexpr std::array<double, 8> wave_shares { 0.0721, 0.0997, 0.1344, 0.1703, 0.1549, 0.1386, 0.1226, 0.1074 };

// fewer short ones than sideways: the rails come near the limit in the vertical more often, with the peak still at it.
// The long ones hardly count on the 10 m chord: the body rises and falls on them, slowly enough for a pantograph to keep up
inline wave_set<8> const up_waves { make_waves<8>( 0.0, { 8.3, 11.9, 16.7, 23.9, 37.1, 47.3, 61.9, 83.3 }, { 0.14, 0.19, 0.22, 0.22, 0.23, 0.23, 0.23, 0.23 } ) };
inline wave_set<8> const x_waves { make_waves( 1.1, wave_lengths, wave_shares ) };
inline wave_set<8> const z_waves { make_waves( 2.3, wave_lengths, wave_shares ) };
// the tilt along each axis of the plan. Long waves only: with the short ones the cross level reaching its limit would break the limit of twist
inline wave_set<3> const tilt_x_waves { make_waves<3>( 0.7, { 23.9, 31.7, 41.9 }, { 1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0 } ) };
inline wave_set<3> const tilt_z_waves { make_waves<3>( 2.9, { 23.9, 31.7, 41.9 }, { 1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0 } ) };

template <std::size_t Count>
double sum( wave_set<Count> const &Waves, glm::dvec3 const &Point ) {

    auto total { 0.0 };
    for( auto const &wave : Waves ) {
        total += wave.share * std::sin( wave.x * Point.x + wave.z * Point.z + wave.phase );
    }
    return total;
}

// displacement at the point for an amplitude of 1, sideways within +-1, up and down within +-1.69
inline glm::dvec3 field( glm::dvec3 const &Point ) {

    return { sum( x_waves, Point ), sum( up_waves, Point ), sum( z_waves, Point ) };
}

// how much lower the rail towards Across lies by settling apart, for an amplitude of 1. A tilt of the plan, so tracks laid towards each
// other meet at one cant. Follows the middle of a path, so plain tracks only
inline double level( glm::dvec3 const &Point, glm::dvec3 const &Across ) {

    return cross_share * ( Across.x * sum( tilt_x_waves, Point ) + Across.z * sum( tilt_z_waves, Point ) );
}

// roll made by the field for an amplitude of 1; positive lowers the rail towards Across, as the roll of a path
inline double roll( glm::dvec3 const &Point, glm::dvec3 const &Across, double const Gauge ) {

    return ( sum( up_waves, Point - Across * ( 0.5 * Gauge ) ) - sum( up_waves, Point + Across * ( 0.5 * Gauge ) ) ) / Gauge;
}

// value at a distance along a track: Full inside, exactly Atstart and Atend at its ends where it meets the neighbours
inline double along( double const Full, double const Atstart, double const Atend, double const Distance, double const Length ) {

    auto const smooth = []( double const Share ) {
        auto const share { std::clamp( Share, 0.0, 1.0 ) };
        return share * share * ( 3.0 - 2.0 * share ); };
    auto const ends { Length > 0.0 ? Atstart + ( Atend - Atstart ) * smooth( Distance / Length ) : Atstart };
    return ends + ( Full - ends ) * std::min( smooth( Distance / fade ), smooth( ( Length - Distance ) / fade ) );
}

// how much of the displacement a point of the profile takes, by its depth under the rail top: all of it down to the foot of the rail,
// none at the foot of the ballast, which stays on the ground as designed
inline double bed_share( double const Depth, double const Rail, double const Height ) {

    return Height > 0.0 ? std::clamp( 1.0 - ( Depth - Rail ) / Height, 0.0, 1.0 ) : 1.0;
}

// a displacement with the part along the track taken out, Across being a unit vector in the plan. Along the track
// it would only change the distance between coupled vehicles, and with it the forces between them
inline glm::dvec3 across_only( glm::dvec3 const &Offset, glm::dvec3 const &Across ) {

    return Across * glm::dot( Offset, Across ) + glm::dvec3 { 0.0, Offset.y, 0.0 };
}

// the unit vector across a direction in the plan, or across the chord if it has none; zero for a path of no length, which has neither
inline glm::dvec3 across_of( glm::dvec3 const &Direction, glm::dvec3 const &Chord ) {

    for( auto const &along : { Direction, Chord } ) {
        glm::dvec3 const across { -along.z, 0.0, along.x };
        if( glm::dot( across, across ) > 0.0 ) {
            return glm::normalize( across );
        }
    }
    return glm::dvec3 { 0.0 };
}

// the cant where two paths meet, theirs turned to the way of this one: a switch takes that of a plain track, alike ones meet halfway
inline float joint_roll( float const Own, float const Their, bool const Switch, bool const Theirswitch ) {

    if( Switch == Theirswitch ) {
        return 0.5f * ( Own + Their );
    }
    return Switch ? Their : Own;
}

// each component the same way
inline amounts along( amounts const &Full, amounts const &Atstart, amounts const &Atend, double const Distance, double const Length ) {

    return {
        along( Full.up, Atstart.up, Atend.up, Distance, Length ),
        along( Full.side, Atstart.side, Atend.side, Distance, Length ),
        along( Full.cross, Atstart.cross, Atend.cross, Distance, Length ) };
}

// distance from the joint behind the point
inline double rail_phase( double const Along, double const Rail ) {

    auto const phase { std::fmod( Along, Rail ) };
    return phase < 0.0 ? phase + Rail : phase;
}

// share of the dip depth at a distance from the joint
inline double dip( double const Fromjoint ) {

    if( Fromjoint >= dip_reach ) {
        return 0.0;
    }
    auto const height { std::cos( glm::half_pi<double>() * Fromjoint / dip_reach ) };
    return height * height;
}

} // namespace track_uneven
