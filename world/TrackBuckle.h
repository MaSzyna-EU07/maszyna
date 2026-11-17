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
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <string>
#include <string_view>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

// Lateral buckling of continuous welded rail: in the heat the rails, held from growing longer, press on the track along it with
// E*A*alpha*dT, and where the bed holds it too weakly sideways the track jumps out to the side. The track of the scenery takes it as
// a displacement across it (TSegment::stray()), drawn and followed by the axles as the unevenness of TrackUneven.h is.
// Sources: Kish, Samavedam, "Risk analysis based CWR track buckling safety evaluations" (Volpe, about 2001); FRA, "Dynamic buckling
// test analyses of a high degree CWR track"; PKP PLK ST-T1-A8 (2021); the answers of the ministry to interpellations 3875 (2026) and
// 2575 (2025). The thresholds and the shape are fitted to the numbers there, see test_track_buckle.cpp.
namespace track_buckle {

// the shape: a main lobe and a side lobe of the other way on each side of it, as the buckled shapes of Kerr and Kish have it. A
// cosine of a period twice the main lobe under a window cos^2 over three lobes: the side lobes come to a quarter of the main one, and
// the curvature runs on smoothly all along, into the straight track at the ends too, as a bent rail has it
// straight track jumps out far at once, a curve goes out step by step and less far, to the outside of the curve (Kish, Samavedam)
inline constexpr double straight_amplitude { 0.4 }; // m
inline constexpr double straight_length { 14.0 }; // m, main lobe
inline constexpr double curve_amplitude { 0.2 }; // m
inline constexpr double curve_length { 11.0 }; // m
inline constexpr double curve_radius { 1500.0 }; // m, below which a track counts as a curve for the shape
inline constexpr double end_margin { 2.0 }; // m between the buckle and the ends of the track
inline constexpr double most_amplitude { 1.0 }; // m
inline constexpr double least_length { 4.0 }; // m
inline constexpr double most_length { 60.0 }; // m
// how far the side lobes reach from the centre, in lengths of the main lobe
inline constexpr double reach_share { 1.5 };

struct shape {
    double amplitude { 0.0 }; // of the main lobe, m: positive to the right of the path looking from its start to its end, negative to the left
    double length { 0.0 }; // of the main lobe between the points it crosses the line, m; each side lobe is as long
    double centre { 0.0 }; // of the main lobe, distance from the start of the path, m
};

inline bool active( shape const &Shape ) {

    return Shape.amplitude != 0.0 && Shape.length > 0.0;
}

// the wave and the window at a distance from the centre, with their derivatives
struct parts {
    double wave, wave1, wave2;
    double window, window1, window2;
};

inline parts parts_at( shape const &Shape, double const Distance ) {

    auto const from { Distance - Shape.centre };
    auto const k { glm::pi<double>() / Shape.length };
    // cos^2(b x) = ( 1 + cos( 2 b x ) ) / 2 reaching 0 at the ends of the side lobes
    auto const b { glm::pi<double>() / ( 2.0 * reach_share * Shape.length ) };
    return {
        std::cos( k * from ), -k * std::sin( k * from ), -k * k * std::cos( k * from ),
        0.5 * ( 1.0 + std::cos( 2.0 * b * from ) ), -b * std::sin( 2.0 * b * from ), -2.0 * b * b * std::cos( 2.0 * b * from ) };
}

inline bool within( shape const &Shape, double const Distance ) {

    return active( Shape ) && std::abs( Distance - Shape.centre ) < reach_share * Shape.length;
}

// displacement across the path at a distance along it, m
inline double offset( shape const &Shape, double const Distance ) {

    if( false == within( Shape, Distance ) ) {
        return 0.0;
    }
    auto const p { parts_at( Shape, Distance ) };
    return Shape.amplitude * p.wave * p.window;
}

// its first derivative along the path
inline double slope( shape const &Shape, double const Distance ) {

    if( false == within( Shape, Distance ) ) {
        return 0.0;
    }
    auto const p { parts_at( Shape, Distance ) };
    return Shape.amplitude * ( p.wave1 * p.window + p.wave * p.window1 );
}

// and the second: the curvature the buckle adds to the path, 1/m, positive bending to the right
inline double bend( shape const &Shape, double const Distance ) {

    if( false == within( Shape, Distance ) ) {
        return 0.0;
    }
    auto const p { parts_at( Shape, Distance ) };
    return Shape.amplitude * ( p.wave2 * p.window + 2.0 * p.wave1 * p.window1 + p.wave * p.window2 );
}

// the mean curvature over a stretch around a distance, as a bogie over its wheelbase feels it: the difference of the slopes at its ends
// over its length
inline double mean_bend( shape const &Shape, double const Distance, double const Base ) {

    if( Base <= 0.0 ) {
        return bend( Shape, Distance );
    }
    return ( slope( Shape, Distance + 0.5 * Base ) - slope( Shape, Distance - 0.5 * Base ) ) / Base;
}

// and as a body on two bogies: it goes sideways by the mean of its pivots, so its lateral acceleration is v^2 times the mean of the
// curvatures at the pivots, each over the wheelbase of the bogie (without bogies: the pivots are the axles, with no base)
inline double body_bend( shape const &Shape, double const Distance, double const Pivots, double const Base ) {

    return 0.5 * ( mean_bend( Shape, Distance - 0.5 * Pivots, Base ) + mean_bend( Shape, Distance + 0.5 * Pivots, Base ) );
}

// how far from the centre the buckle reaches on each side
inline double reach( shape const &Shape ) {

    return active( Shape ) ? reach_share * Shape.length : 0.0;
}

// the largest curvature of the buckle, 1/m, in the middle of the main lobe: pi^2 a / L^2 of the wave and 2/9 of it from the window
inline double peak_bend( double const Amplitude, double const Length ) {

    return ( 1.0 + 2.0 / ( 4.0 * reach_share * reach_share ) ) * glm::pi<double>() * glm::pi<double>() * std::abs( Amplitude ) / ( Length * Length );
}

// the heat

inline constexpr double steel_modulus { 210e9 }; // Pa
inline constexpr double steel_expansion { 1.15e-5 }; // 1/K
inline constexpr double sun_heating { 17.0 }; // K a rail in full sun is warmer than the air: 30 F (Kish, Samavedam)
inline constexpr float neutral_default { 26.f }; // C, the rail stress-free temperature PLK lays its tracks at since 2025
inline constexpr double area_60e1 { 76.70e-4 }; // m2
inline constexpr double area_49e1 { 62.92e-4 };
inline constexpr double area_s42 { 54.2e-4 };

// the rail section by the name of the rail profile, 60E1 if it tells nothing
inline double rail_area( std::string_view const Profile ) {

    auto const has = [ Profile ]( std::string_view const Part ) {
        return std::search( Profile.begin(), Profile.end(), Part.begin(), Part.end(), []( char const Left, char const Right ) {
            return std::tolower( static_cast<unsigned char>( Left ) ) == std::tolower( static_cast<unsigned char>( Right ) ); } ) != Profile.end(); };
    if( has( "49" ) ) {
        return area_49e1;
    }
    if( has( "s42" ) ) {
        return area_s42;
    }
    return area_60e1;
}

// the force along a rail held from growing longer, N, pressing for Rise above the neutral temperature
inline double thermal_force( double const Area, double const Rise ) {

    return steel_modulus * Area * steel_expansion * Rise;
}

// the temperature of a rail, C, from the air, the height of the sun over the horizon in degrees and the clouds (0 clear, 1 overcast;
// above 1 it rains, which cools as much)
inline double rail_temperature( double const Air, double const Sun, double const Overcast ) {

    auto const sun { std::max( 0.0, std::sin( glm::radians( Sun ) ) ) };
    return Air + sun_heating * sun * ( 1.0 - std::clamp( Overcast, 0.0, 1.0 ) );
}

// the state of the track, from the scenery
namespace grade {
inline constexpr int good { 0 }; // concrete sleepers, full ballast shoulder, consolidated
inline constexpr int average { 1 };
inline constexpr int weak { 2 }; // timber sleepers, ballast missing, or the bed disturbed by tamping or sleeper renewal not yet consolidated
}

struct cwr {
    float neutral { neutral_default }; // C
    int grade { grade::average };
    bool worked { false }; // works on the bed lately: weak whatever the grade
};

// the rise above the neutral temperature at which the track buckles: from low on under a passing train, which lifts it between the
// bogies and takes away part of its hold sideways, at high and above by itself
struct limits {
    double low { 0.0 }; // K
    double high { 0.0 };
};

inline constexpr std::array<limits, 3> grade_limits { { { 40.0, 55.0 }, { 32.0, 45.0 }, { 22.0, 32.0 } } };

// for the grade of the track, lowered in a curve, which presses outwards, and by an uneven line, which gives it a start
inline limits limits_of( cwr const &Track, double const Radius, double const Side ) {

    auto const kind { Track.worked ? grade::weak : std::clamp( Track.grade, grade::good, grade::weak ) };
    auto result { grade_limits[ kind ] };
    auto less { 0.0 };
    auto const radius { std::abs( Radius ) };
    if( radius > 0.0 && radius < 300.0 ) {
        less += 6.0;
    }
    else if( radius > 0.0 && radius < 600.0 ) {
        less += 3.0;
    }
    if( Side > 15.0 ) {
        less += 3.0;
    }
    result.low -= less;
    result.high -= less;
    return result;
}

// the share of the passes of a train which buckle the track at the rise: none below low, all at high
inline double chance( double const Rise, limits const &Limits ) {

    if( Limits.high <= Limits.low ) {
        return Rise >= Limits.high ? 1.0 : 0.0;
    }
    return std::clamp( ( Rise - Limits.low ) / ( Limits.high - Limits.low ), 0.0, 1.0 );
}

enum class verdict {
    none,
    under_train,
    by_itself
};

// with Train the track has a train going over it hard enough, Draw is the draw of the pass, 0 to 1
inline verdict decide( double const Rise, limits const &Limits, bool const Train, double const Draw ) {

    if( Rise >= Limits.high ) {
        return verdict::by_itself;
    }
    if( Train && Rise >= Limits.low && Draw < chance( Rise, Limits ) ) {
        return verdict::under_train;
    }
    return verdict::none;
}

// a train stirs the track up when it goes fast or brakes on it
inline constexpr double stirring_speed { 40.0 }; // km/h
inline bool stirs( double const Speed, bool const Braking ) {

    return Speed > stirring_speed || ( Braking && Speed > 5.0 );
}

// draws, the same for the same track and seed

// FNV-1a, which std::hash isn't on every compiler
inline std::uint64_t name_hash( std::string_view const Name ) {

    std::uint64_t hash { 14695981039346656037ULL };
    for( auto const character : Name ) {
        hash ^= static_cast<unsigned char>( character );
        hash *= 1099511628211ULL;
    }
    return hash;
}

// the splitmix64 finalizer, so close numbers give far apart draws
inline std::uint64_t scramble( std::uint64_t Value ) {

    Value += 0x9e3779b97f4a7c15ULL;
    Value = ( Value ^ ( Value >> 30 ) ) * 0xbf58476d1ce4e5b9ULL;
    Value = ( Value ^ ( Value >> 27 ) ) * 0x94d049bb133111ebULL;
    return Value ^ ( Value >> 31 );
}

// what a draw is for, so the draws for one track don't go together
enum purpose : std::uint64_t {
    pass = 1,
    site = 2,
    place = 3,
    way = 4
};

// a fraction 0 to 1
inline double draw( std::uint64_t const Name, std::uint32_t const Seed, std::uint64_t const Purpose, std::uint64_t const Number = 0 ) {

    auto const value { scramble( scramble( scramble( Name ^ Purpose ) + Seed ) + Number ) };
    // the 53 high bits
    return static_cast<double>( value >> 11 ) / 9007199254740992.0;
}

// where the track buckles by the temperature on its own: one place in so many metres of track, and the tracks worked on lately.
// Elsewhere the bed holds far beyond the limits of the grade (Sweden: 200 buckles in the hot summer of 2018 on 14 000 km)
inline constexpr double site_spacing { 10000.0 }; // m

inline bool weak_spot( std::uint64_t const Name, std::uint32_t const Seed, double const Length, bool const Worked ) {

    return Worked || draw( Name, Seed, site ) < Length / site_spacing;
}

// the entries in the scenery

// buckle <amplitude mm>;<length of the main lobe m>;<centre m>[;<side -1 left, 1 right, 0 out of the curve>]
struct declared {
    float amplitude { 0.f }; // mm
    float length { -1.f }; // m, negative: by the shape of the track
    float centre { -1.f }; // m, negative: drawn
    int side { 0 };
};

inline bool given( declared const &Values ) {

    return Values.amplitude > 0.f;
}

// the numbers of an entry like "400;18;60", NaN where left out
template <std::size_t Count>
std::array<float, Count> read_numbers( std::string_view const Text ) {

    std::array<float, Count> result;
    result.fill( std::numeric_limits<float>::quiet_NaN() );
    auto rest { Text };
    for( auto &field : result ) {
        auto const end { std::min( rest.find( ';' ), rest.size() ) };
        if( auto value { 0.f }; end > 0 && std::from_chars( rest.data(), rest.data() + end, value ).ec == std::errc {} && std::isfinite( value ) ) {
            field = value;
        }
        if( end == rest.size() ) {
            break;
        }
        rest.remove_prefix( end + 1 );
    }
    return result;
}

inline declared read_buckle( std::string_view const Text ) {

    auto const fields { read_numbers<4>( Text ) };
    declared result;
    if( std::isfinite( fields[ 0 ] ) ) {
        result.amplitude = std::clamp( fields[ 0 ], 0.f, static_cast<float>( most_amplitude * 1000.0 ) );
    }
    if( std::isfinite( fields[ 1 ] ) && fields[ 1 ] > 0.f ) {
        result.length = std::clamp( fields[ 1 ], static_cast<float>( least_length ), static_cast<float>( most_length ) );
    }
    if( std::isfinite( fields[ 2 ] ) && fields[ 2 ] >= 0.f ) {
        result.centre = fields[ 2 ];
    }
    if( std::isfinite( fields[ 3 ] ) ) {
        result.side = ( fields[ 3 ] > 0.f ? 1 : fields[ 3 ] < 0.f ? -1 : 0 );
    }
    return result;
}

inline std::string write_buckle( declared const &Values ) {

    if( false == given( Values ) ) {
        return {};
    }
    std::ostringstream text;
    text << "buckle " << Values.amplitude << ';';
    if( Values.length > 0.f ) {
        text << Values.length;
    }
    text << ';';
    if( Values.centre >= 0.f ) {
        text << Values.centre;
    }
    if( Values.side != 0 ) {
        text << ';' << Values.side;
    }
    text << ' ';
    return text.str();
}

// cwr <neutral temperature C>;<grade 0 good, 1 average, 2 weak>[;<worked lately 0/1>]
inline cwr read_cwr( std::string_view const Text ) {

    auto const fields { read_numbers<3>( Text ) };
    cwr result;
    if( std::isfinite( fields[ 0 ] ) ) {
        result.neutral = std::clamp( fields[ 0 ], -20.f, 60.f );
    }
    if( std::isfinite( fields[ 1 ] ) ) {
        result.grade = std::clamp( static_cast<int>( std::lround( fields[ 1 ] ) ), 0, 2 );
    }
    if( std::isfinite( fields[ 2 ] ) ) {
        result.worked = fields[ 2 ] > 0.f;
    }
    return result;
}

inline std::string write_cwr( cwr const &Values ) {

    if( Values.neutral == neutral_default && Values.grade == grade::average && false == Values.worked ) {
        return {};
    }
    std::ostringstream text;
    text << "cwr " << Values.neutral << ';' << Values.grade;
    if( Values.worked ) {
        text << ";1";
    }
    text << ' ';
    return text.str();
}

// the buckle of the values on a track of the length, which curves to the side Outward (-1 left, 1 right, 0 straight), with radius
// Radius (0 straight); None for one which doesn't fit into the track. Place and Way are draws for the centre and the side not given
inline shape resolve( declared const &Values, double const Length, int const Outward, double const Radius, double const Place, double const Way ) {

    if( false == given( Values ) ) {
        return {};
    }
    auto const curve { Outward != 0 && Radius > 0.0 && Radius < curve_radius };
    shape result;
    result.length = Values.length > 0.f ? static_cast<double>( Values.length ) : ( curve ? curve_length : straight_length );
    auto const first { reach_share * result.length + end_margin };
    auto const last { Length - reach_share * result.length - end_margin };
    if( last < first ) {
        return {};
    }
    result.centre = Values.centre >= 0.f ? std::clamp( static_cast<double>( Values.centre ), first, last ) : first + ( last - first ) * Place;
    auto const side { Values.side != 0 ? Values.side : Outward != 0 ? Outward : ( Way < 0.5 ? -1 : 1 ) };
    result.amplitude = side * 0.001 * Values.amplitude;
    return result;
}

// the buckle the temperature makes on a track: as large as a curve or a straight one makes
inline declared natural( bool const Curve ) {

    declared result;
    result.amplitude = static_cast<float>( 1000.0 * ( Curve ? curve_amplitude : straight_amplitude ) );
    result.length = static_cast<float>( Curve ? curve_length : straight_length );
    return result;
}

} // namespace track_buckle
