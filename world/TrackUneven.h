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
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

// Track unevenness: a displacement field over the plan of the scenery, so everything at one place moves together: joined tracks, the
// paths of a switch, sleepers, axles. Waves 5-42 m long, shares after the German track spectrum with the long waves cut down.
// The "uneven" value is the peak deviation of a rail on a 10 m chord, as in PLK Id-14 appendix 1; see test_track_uneven.cpp.
namespace track_uneven {

inline constexpr float most { 100.f }; // largest value of the attributes, mm
inline constexpr double stray_per_mm { 0.00145 }; // field amplitude per mm of "uneven", m
inline constexpr double cross_share { 0.6 }; // share of the field amplitude the rails settle apart by
inline constexpr double fade { 20.0 }; // length over which a track goes from the value at its end to its own, m
inline constexpr double dip_share { 0.4 }; // joint dip depth per mm of "uneven", mm
inline constexpr double dip_reach { 3.0 }; // reach of the dip on each side of a joint, m

// the value of each component of the unevenness, mm
struct amounts {
    double up { 0.0 };
    double side { 0.0 };
    double cross { 0.0 };
};

struct wave {
    double x;
    double z;
    double phase;
    double share;
};

template <std::size_t Count>
using wave_set = std::array<wave, Count>;

// the waves run in all directions of the plan, so no direction of a track is favoured
template <std::size_t Count>
wave_set<Count> make_waves( double const Turn, std::array<double, Count> const &Lengths, std::array<double, Count> const &Shares ) {

    auto const golden { glm::pi<double>() * ( 3.0 - std::sqrt( 5.0 ) ) };
    wave_set<Count> result {};
    for( std::size_t i = 0; i < Count; ++i ) {
        auto const number { glm::two_pi<double>() / Lengths[ i ] };
        auto const heading { Turn + golden * i };
        result[ i ] = { number * std::cos( heading ), number * std::sin( heading ), golden * ( i + 1 ) * ( 1.0 + 3.0 * Turn ), Shares[ i ] };
    }
    return result;
}

inline constexpr std::array<double, 8> wave_lengths { 5.3, 7.1, 9.7, 13.1, 17.9, 23.9, 31.7, 41.9 };
inline constexpr std::array<double, 8> wave_shares { 0.0721, 0.0997, 0.1344, 0.1703, 0.1549, 0.1386, 0.1226, 0.1074 };

inline wave_set<8> const up_waves { make_waves( 0.0, wave_lengths, wave_shares ) };
inline wave_set<8> const x_waves { make_waves( 1.1, wave_lengths, wave_shares ) };
inline wave_set<8> const z_waves { make_waves( 2.3, wave_lengths, wave_shares ) };
// long waves only: with the short ones the cross level reaching its limit would break the limit of twist
inline wave_set<3> const cross_waves { make_waves<3>( 0.7, { 23.9, 31.7, 41.9 }, { 1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0 } ) };

template <std::size_t Count>
double sum( wave_set<Count> const &Waves, glm::dvec3 const &Point ) {

    auto total { 0.0 };
    for( auto const &wave : Waves ) {
        total += wave.share * std::sin( wave.x * Point.x + wave.z * Point.z + wave.phase );
    }
    return total;
}

// displacement at the point for an amplitude of 1, each axis within +-1
inline glm::dvec3 field( glm::dvec3 const &Point ) {

    return { sum( x_waves, Point ), sum( up_waves, Point ), sum( z_waves, Point ) };
}

// how much lower the right rail lies by settling apart, for an amplitude of 1. Follows the middle of a path, so plain tracks only
inline double level( glm::dvec3 const &Point ) {

    return cross_share * sum( cross_waves, Point );
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
