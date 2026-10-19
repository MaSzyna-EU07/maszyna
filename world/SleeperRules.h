/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstddef>

// the rules the sleepers and timbers are laid by, and the record of where the layout had to break them
namespace track_sleepers {

// how far apart the beams are along the track, and how far closer or further they may go to put one where a pin is
double constexpr pitch { 0.6 };
double constexpr pitch_min { 0.83 * pitch };
double constexpr pitch_max { 1.08 * pitch };
// how far out from the middle of a track its rail reaches past half the gauge: the foot of the rail outside the gauge line
double constexpr rail_reach { 0.1115 };
// how far a beam reaches out past the foot of the rail, 2.6 m sleepers under the standard gauge, and how far at least
double constexpr overhang { 0.471 };
double constexpr overhang_min { 0.25 };
// how far the feet of the rails a timber carries may stand off one plane
double constexpr coplanar_tolerance { 0.01 };
// the gap left between the ends of two beams on one line
double constexpr end_gap { 0.05 };
// half the width of a beam, to keep the sleepers of a track crossing another at a wide angle clear of the timbers of the other
double constexpr half_width { 0.13 };
// a timber longer than this is a fault of the layout
double constexpr length_cap { 8.5 }; // the longest prestressed concrete bearer of EN 13230-4, under both tracks of a scissors crossing about 5 m apart
// the rails of a path crossing the line of a beam at a lower angle than this aren't laid on it
double constexpr crossing_sine_min { 0.2588 }; // 15 degrees
// two paths crossing at this angle or wider keep the beams square to each, laid as a grid
double constexpr grid_angle { 1.2915 }; // 74 degrees
// the timbers turn toward the bisector of a frog over as many beams before it
double constexpr frog_ramp { 6.0 * pitch };
// past where the tracks go apart the sleepers lie square to them at once, unless the last timber turns so far from them that
// the ends of the next sleepers would meet it: then they turn back by at most so much a beam
double constexpr turn_max { 0.2415 }; // 13.8 degrees: a sleeper 1.3 m out from the track keeps clear of one turned so far a pitch away
// how far apart the samples of a path are, to tell how far the paths around run from it
double constexpr sample_step { 0.5 };
// two paths over each other, on a bridge, aren't merged
double constexpr level_tolerance { 1.0 };
// how far around a track the tracks are taken in to lay its beams: those whose beams it carries, or which carry its own, and
// those which set where the beams of these go
double constexpr around_margin { 20.0 };
// how many beams make a run drawn or left out together
std::size_t constexpr chunk_beams { 16 };

// how many tracks a cluster takes at most
std::size_t constexpr cluster_cap { 256 };
// how far the lines of the timbers turn from one to the next at most, so the far ends of the long ones keep their spacing, and how
// far the plain sleepers turn back square to the track from one to the next
double constexpr timber_turn { 0.0175 }; // a degree
double constexpr sleeper_turn { 0.06 }; // three and a half degrees

// what was wrong with the beams laid since the counts were taken last
struct fault_counts {
    std::size_t offplane { 0 }; // timbers shared by rails out of one plane, where the tracks run too close to part them
    std::size_t clipped { 0 }; // beam ends cut shorter than the least overhang, to keep clear of the beams of other tracks
    std::size_t dropped { 0 }; // beams left out where they'd lie over one laid before
    double slowest { 0.0 }; // the longest a cluster took to lay, in ms
    std::size_t biggest { 0 }; // the most tracks in a cluster
};

inline fault_counts &faults() {
    static fault_counts counts;
    return counts;
}


} // namespace track_sleepers
