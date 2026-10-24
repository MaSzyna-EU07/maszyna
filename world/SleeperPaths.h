/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <functional>
#include <vector>

#include <glm/glm.hpp>

class TTrack;
class TSegment;

// the paths of the tracks as the sleepers are laid out along them: sampled in plan, with how close they run to each other and
// whether the beams of one carry the rails of another
namespace track_sleepers {

// a path of a track by samples taken along it
struct sampled_path {
    TTrack *track { nullptr };
    int path { 0 };
    TSegment const *segment { nullptr };
    double length { 0.0 };
    double gauge { 1.435 };
    double rails { 0.829 }; // how far out from the middle of the path its rails reach
    double reach { 1.3 }; // half the length of the sleepers of the path alone
    double half { 1.3 }; // half the length of the plain sleepers of the track, of the model it lays them of
    double line { 0.0 }; // the length of the line of plain tracks joined end to end the path is part of
    bool level { true }; // the path runs level and without cant, its rails all at the one height
    double feet { 0.0 }; // the height of the feet of its rails where it's level
    std::vector<glm::dvec3> points;
    std::vector<double> distances;
    glm::dvec2 low { 0.0 }; // the bounds of the samples in plan
    glm::dvec2 high { 0.0 };
};

int path_count( TTrack const &Track );
TSegment const *path_segment( TTrack const &Track, int const Path );
sampled_path sample( TTrack &Track, int const Path );
// the direction of the path in plan by its samples about the distance
glm::dvec2 heading( sampled_path const &Path, double const Distance );
// which of two paths lays the beams where they're merged: a switch before a plain track, the main path of a switch
// before the diverging one, a plain track of a longer line before one of a shorter, the longer before the shorter, and
// then by the name
bool before( sampled_path const &Left, sampled_path const &Right );
bool near_bounds( sampled_path const &Path, glm::dvec3 const &Point, double const Distance );
// how far the point is from the path in plan and which way the path runs there, when it's beside the path closer than the
// distance and at about the same level. a point past either end of the path isn't beside it, so a track going on from the end
// of another doesn't count as running beside it
bool beside( sampled_path const &Path, glm::dvec3 const &Point, double const Distance, double &Gap, glm::dvec2 &Direction );
// the place of the path nearest to the point in plan: its distance along the path, where it is and which way the path runs
double nearest_on( sampled_path const &Path, glm::dvec2 const &Point, glm::dvec2 &Position, glm::dvec2 &Direction );
// where the feet of the rails of the path are at the distance along it: the middle between them, the unit vector across to the
// rail on the right, tilted with the cant, and the unit vector along the path, with its grade
void rail_frame( sampled_path const &Path, double const Distance, glm::dvec3 &Middle, glm::dvec3 &Across, glm::dvec3 &Along );
// how far the feet of the rails of the path at the point stand off the plane of the feet of the rails of the other path where
// it's nearest the point: a timber under both lies flat under all of them only if they're in one plane
double off_plane( sampled_path const &Path, glm::dvec3 const &Point, sampled_path const &Other );
// how far apart the middles of two paths running at the angle are where their plain sleepers would just touch: the one square to
// the other reaches out by half its length, the other by its half length foreshortened and half its width, with the gap
// between their ends. closer, the paths share the timbers
double touch_spacing( sampled_path const &Path, sampled_path const &Other, double const Angle );
// how close two paths run where their sleepers can't be laid apart, so they share the timbers whether they're in one plane or not
double shared_spacing( sampled_path const &Path, sampled_path const &Other );
// whether the path at the point, running the given way, is carried by the beams of a path before it, which it runs beside so
// close that their plain sleepers would touch. where the paths cross so wide that one timber can't carry both, the sleepers
// of the path are only left out where they'd meet the timbers of the other. the paths share timbers only while their rails lie
// in one plane, or run so close they can't be parted: where one rises, falls or cants apart, each lays its own
bool shares_with( sampled_path const &Path, glm::dvec3 const &Point, glm::dvec2 const &Direction, sampled_path const &Other );
// the paths of the track, and of the tracks with beams around it; or of all those with the ballast the simulator makes, whether
// they get beams or not
std::vector<sampled_path> paths_around( TTrack &Track, bool const Bedded = false );
// the tracks of the region near enough the track to take part in laying its beams, itself among them, some more than once
void tracks_near( TTrack &Track, std::function<void( TTrack * )> const &Visit );
// the stretches of the path where the test holds of the place at a distance along it and the way the path runs there, tested at
// half the step of the samples, with the ends narrowed down between
std::vector<glm::dvec2> stretches_where( sampled_path const &Path, std::function<bool( glm::dvec3 const &, glm::dvec2 const & )> const &Test );

} // namespace track_sleepers
