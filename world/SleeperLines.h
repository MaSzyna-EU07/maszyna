/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <vector>

#include <glm/glm.hpp>

#include "world/SleeperPaths.h"

// the lines of beams across the tracks: where the beams go along the paths, which rails a line takes and where the beams on it
// part and end
namespace track_sleepers {

// a place along a path the beams are spread from: a pin with a beam at it, a free end of the track with the first beam half
// a step from it, so the beams keep the pitch over the joints, or a timber of another path which carries the rails of this
// one past it, with the first beam of this one a step from it
struct anchor {
    double at { 0.0 };
    double offset { 0.5 }; // how many steps the first beam is from the anchor: 0 pin, 0.5 free end, 1 timber of another path
};

// where another path crosses the line of a beam
struct crossing {
    sampled_path const *path { nullptr };
    glm::dvec3 point;
    glm::dvec2 direction; // of the path, in plan
    double at { 0.0 }; // the distance of the point along the path
    double across { 0.0 }; // along the beam, from the path laying it
    double sine { 1.0 }; // of the angle between the path and the beam
    bool carried { false };
};

// the stretch of a beam line the rails of a path take, and how far the beam may reach past them
struct band {
    double from { 0.0 };
    double to { 0.0 };
    double overhang { 0.0 };
    double overhangmin { 0.0 };
    bool carried { true }; // the rails are laid on the line, else the beam keeps clear of them and of the sleepers under them
    sampled_path const *path { nullptr };
    double centre { 0.0 }; // where the middle of the path crosses the line
    glm::dvec2 direction { 0.0 }; // of the path there, in plan
    double sine { 1.0 }; // of the angle between the path and the line
};

// a beam on a line, by where its ends are along the line
struct piece {
    double from { 0.0 };
    double to { 0.0 };
    bool clipped { false }; // an end cut short to keep clear of the rails of another track
    double corefrom { 0.0 }; // the stretch the feet of the rails it carries take
    double coreto { 0.0 };
    band const *single { nullptr }; // the rails of the only track it carries, null where it carries more
};

// the beams spread evenly between two anchors, the pin at the first one included and the one at the last left to the next stretch
void spread( anchor const &From, anchor const &To, std::vector<double> &Stations );
void find_crossings( sampled_path const &Path, glm::dvec3 const &Point, glm::dvec2 const &Along, glm::dvec2 const &Across, double const Reach, std::vector<sampled_path> const &Around, std::vector<crossing> &Crossings );
// the paths whose rails the line of the beam through the point of the path takes, marked among the paths of its cluster crossing
// the line: those whose plain sleepers would meet the sleepers of the path or of any the line takes already, at a narrow enough
// angle to the line, and in one plane with the path unless they run too close to be parted
// the timber takes the rails of a path whose plain sleepers would meet those of a path it takes already; where Tight, only of one so
// close that their sleepers couldn't be laid apart even cut short
void carry( sampled_path const &Path, glm::dvec2 const &Forward, glm::dvec2 const &Along, std::vector<crossing> &Crossings, bool const Tight, bool const Count = true );
band band_of( sampled_path const &Path, double const Across, double const Sine, glm::dvec2 const &Direction, bool const Carried = true );
// the beams on the line through the point: one timber under all the rails it carries, the ends keeping clear of the rails of the
// tracks the line doesn't carry and of the sleepers under them. the timber is parted between the rails of two tracks where each
// keeps at least its least overhang past its rails, unless it's whole: about a crossing of the tracks, as in a scissors crossing
void emit_line( std::vector<band> &Bands, std::vector<piece> &Pieces, bool const Whole );
// a timber longer than the cap cut in two where the rails of the tracks it carries are furthest apart, the ends sharing the
// room between them, till every piece is short enough or there's nowhere to cut
void cut_long( std::vector<band> const &Bands, std::vector<piece> &Pieces );

} // namespace track_sleepers
