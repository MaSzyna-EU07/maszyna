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
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

class TTrack;
class TModel3d;
class TSubModel;

// sleepers and switch timbers laid under the rails of the tracks whose ballast the simulator makes, when a model of them
// is set with sleepers3d in eu07.ini. A timber is put together from the parts of the model: its two ends and the piece between
// them, repeated and stretched to the length of the beam, so one model makes the timbers of any length; a model without such
// parts is drawn whole, stretched. The sleepers of plain tracks are a whole model of their own, set with sleepers3dplain, drawn
// at their own length. A track can have models of its own: of its sleepers with sleepermodel, of its timbers with timbermodel.
namespace track_sleepers {

// the parts of the model a beam is put together from, by the names of the submodels
enum part : int {
    part_begin, // the end of the beam on its +x side
    part_main, // the piece between the ends
    part_end, // the end on the -x side
    part_count
};

// a sleeper or a timber, by the middle of its top face at both ends
struct beam {
    glm::dvec3 from;
    glm::dvec3 to;
    glm::dvec3 along; // unit vector across the beam, along the tracks it carries
    bool plain { false }; // a sleeper of a track running alone, square to it, not a timber of a switch or a crossing
    double corefrom { 0.0 }; // the stretch the feet of the rails it carries take, along it from its from end
    double coreto { 0.0 };
};

// a model sleepers or timbers are made of, as found in it
struct beam_model {
    std::string name;
    std::string skinname;
    TModel3d *model { nullptr };
    int skin { 0 }; // the replacable skin, 0 for the textures of the model
    // put together from its submodels begin, main and end, the one between stretched and repeated to the length of a timber; a
    // model without these is drawn whole, a plain sleeper at its own length and a timber stretched to its length
    bool stretchable { false };
    std::array<TSubModel *, part_count> parts {};
    std::array<glm::vec2, part_count> spans {}; // from and to along x, of the parts
    glm::vec2 span { -1.3f, 1.3f }; // from and to along x, of the most detailed submodels of the whole model
    float seat { 0.f }; // the height in the model the feet of the rails go to
    std::vector<float> bands; // the squared distances the levels of detail of the model are seen from, the nearest first
    float length() const { return span.y - span.x; }
};

// a run of beams of a track drawn or left out together, by the sphere around them
struct chunk {
    glm::vec3 centre; // relative to the origin of the track
    float radius { 0.f };
    std::array<std::uint32_t, part_count> first {};
    std::array<std::uint32_t, part_count> last {};
    std::uint32_t wholefirst { 0 }; // the timbers of the run made of a whole model
    std::uint32_t wholelast { 0 };
    std::uint32_t plainfirst { 0 }; // the sleepers of the run made of the model of the plain sleepers
    std::uint32_t plainlast { 0 };
};

// the beams laid under a track, as the instances of the parts of the model
struct laid {
    bool active { false }; // the track gets beams and its ballast is lowered under them; set as its geometry is made
    bool dirty { true }; // the beams are to be laid again before they're drawn
    bool laying { false }; // the beams are being placed, so the tracks about it work out its beams themselves meanwhile
    beam_model const *timbers { nullptr }; // the model the timbers are made of
    beam_model const *plains { nullptr }; // the model of the plain sleepers, null where they're made of that of the timbers
    std::array<std::vector<glm::mat4>, part_count> instances; // transforms of the parts relative to the origin of the track
    std::vector<glm::mat4> whole; // transforms of the timbers of a model without parts, drawn whole
    std::vector<glm::mat4> plain; // transforms of the plain sleepers, whole models, relative to the origin of the track
    std::vector<chunk> chunks;
    std::vector<beam> beams; // as they're laid
};

// how far the ballast of a track with beams is lowered, so they stick out of it
double constexpr bed_drop { 0.06 };

// the model the timbers are made of where a track has none of its own, set with sleepers3d in eu07.ini and loaded on the first
// use; null when there's none, and no beams are laid
beam_model const *timber_model();
// the model of the sleepers of the tracks running alone, set with sleepers3dplain; null when there's none
beam_model const *plain_model();
// the models the track lays its beams of: those set for it with timbermodel and sleepermodel, else the ones of eu07.ini
beam_model const *timber_model( TTrack const &Track );
beam_model const *plain_model( TTrack const &Track );
// how long the plain sleepers of the track are, of the model it lays them of; 2.6 m at the standard gauge where there's none
double plain_length( TTrack const &Track );
// whether the track has the ballast the simulator makes, which the beams are laid on when there's a model of them
bool bedded( TTrack const &Track );
// whether the track gets beams: a rail track with the ballast made by the simulator, and the model is there
bool lays( TTrack const &Track );
// where along the path of the track its rails lie on timbers shared with other tracks: where another track runs so near that
// the plain sleepers of the two would touch, of the lengths of the models they'd be made of. this goes by the shape of the
// tracks alone, so it holds whether the beams are drawn or not. the stretches in the order along the path, none where there
// are no shared timbers
std::vector<std::pair<double, double>> shared_zones( TTrack const &Track, int const Path );
// from the start of the first of these stretches to the end of the last, both zero where there's none
std::pair<double, double> shared_zone( TTrack const &Track, int const Path );
// a model of sleepers or timbers in the scenery data, by the name it's loaded by
struct pack_model {
    std::string name;
    bool timber { false }; // has the submodels begin, main and end a timber is put together from
};
// the models of sleepers and timbers in the scenery data, found once in models/ by the names of their files telling sleepers
// or timbers: podklad, podrozjazd, sleeper, timber. the timber ones first, then by the name
std::vector<pack_model> const &pack_models();
// the beams of the track placed anew, as instances of the parts of the model
void lay( TTrack &Track );
// the beams of the tracks around the track to be placed again, as they depend on where it runs
void unsettle_around( TTrack &Track );
// the beams worked out in a thread of their own as the tracks come near, while the simulation runs; off in the editor, where the
// tracks change and the beams are worked out as they're needed. turning it off waits for the beams being worked out
void lay_in_background( bool const Background );
// the beams of the track worked out ahead, before it comes in range
void prefetch( TTrack &Track );

// what's wrong with the layout of the beams of all the tracks: beams over each other, rails with too few beams under them,
// timbers shared by rails out of one plane
struct layout_check {
    std::size_t beams { 0 };
    std::size_t overlaps { 0 };
    std::size_t gaps { 0 }; // stretches of rail with the beams under it further apart than the pitch allows
    std::size_t bare { 0 }; // rails with no beams under them at all
    std::size_t toolong { 0 };
    std::size_t offplane { 0 }; // timbers shared by rails out of one plane, where the tracks run too close to part them
    std::size_t clipped { 0 }; // beam ends cut shorter than the least overhang to keep clear of the beams of other tracks
    std::size_t dropped { 0 }; // beams left out where they'd lie over one laid before
    double slowest { 0.0 }; // the longest a cluster of tracks took to lay, in ms
    std::size_t biggest { 0 }; // the most tracks in a cluster
    std::size_t panels { 0 }; // switches with their paths out of one plane where they share the timbers of the panel
    std::uint64_t digest { 0 }; // of where all the beams lie, to tell whether a change of the code moved any
    double longest { 0.0 };
    double widest { 0.0 }; // the widest spacing of the beams under a rail
    std::string widestat;
    std::vector<std::string> examples;
};
layout_check check();

// the beams laid with the track: the tracks whose plain sleepers would meet make a cluster laid together, lines of timbers
// across all of their rails turned to the bisectors of frogs and crossings, and the plain sleepers square to each track where no
// timber carries it, in step with the timbers. every beam is laid with one track of the cluster
std::vector<beam> place( TTrack &Track );

} // namespace track_sleepers
