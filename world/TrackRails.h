/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "world/Track.h"
#include "world/GaugeLine.h"

namespace track_rails {

void forget();
// changes with every forget(): what's kept of the rails around holds as long as it doesn't
unsigned int memo_stamp();
rail_size measure_rail( gfx::vertex_array const &Profile );
// the noses, wing rails and point rails at crossings at small angles are carved out of what lies around them, all by the same rules: no rail
// reaches into the flangeway of another one where its wheels run, and two heads lying over each other share the room between them by what
// the wheels of each roll on
void carve_plans( TTrack &Track, rail_layout &Layout );
// the layout of the track designed already to carve the tracks around against, before carving
rail_layout const *designed_layout( TTrack &Track );
// how many parts carving left with nothing of the head somewhere along them, since the start
int carve_faults();
// how far a wing rail reaches past the crossing point of the rails
double wing_reach( double const Slope );
double check_length( double const Slope );
// how far from the tip the nose of a movable frog bends over to the wing rail
double movable_nose_length( double const Slope );
// the rails of the track are cut where they cross other rails: plain tracks, and switches whose frog is made from the rails
bool cut_rails( TTrack const &Track );
int blade_steps( TTrack const &Track, int const Path );
double blade_length( TTrack const &Track, int const Path );
// the plain track a path of a switch goes on into at its end, laid the same way, which the blade of the path may run on into
TTrack const *runon_track( TTrack const &Switch, int const Path );
// how far the blade of the switch the track goes on from runs on into its rail of Side
double blade_runon( TTrack const &Track, int const Side );
material_handle cut_material( material_handle const Material );
float tread_brightness( material_handle const Material, glm::vec2 const &Band );
glm::vec3 tread_colour( material_handle const Material, glm::vec2 const &Band );
material_handle tint_material( material_handle const Material, glm::vec3 const &Colour );
// triangles closing the first and/or the last cross-section of a lofted strip, made of pairs of points for each of Ring profile points, both sides of each
void cap_loft( gfx::vertex_array const &Strip, std::size_t const Ring, bool const Start, bool const End, gfx::vertex_array &Triangles );
gfx::userdata_array worn_userdata( std::size_t const Count );
// the ends of rails left open: at track ends with nothing joined, blade tips, the gap of a frog
void cut_faces( gfx::vertex_array const &Strip, std::size_t const Ring, bool const Start, bool const End, gfx::geometrybank_handle const &Bank, TTrack::geometryhandle_sequence &Geometry );

// the geometry of a track laid while one is alive goes into the chunks of the pool given, of the same type, before new ones
class reused_chunks {
public:
    reused_chunks( chunk_pool *Pool );
    ~reused_chunks();
private:
    chunk_pool *m_previous { nullptr };
};
gfx::geometry_handle insert_chunk( gfx::vertex_array &Vertices, gfx::userdata_array &Userdata, gfx::geometrybank_handle const &Bank, int const Type );

TSegment const &path_segment( TTrack const &Track, int const Path );
int path_count( TTrack const &Track );
TTrack *path_neighbour( TTrack const &Track, int const Path, int const End );
double knuckle_bend( double const Into );
bool replaced_check( std::vector<check_rail> const &Checks, double const Wingend, double const Direction, double &Far );
bool replaced_by_wing( check_rail const &Check, knots::line const &Wingends );
gauge_line const &rail_line( TTrack const &Track, int const Path, int const Side, bool const Blades );
check_rail_set const &rail_checks( TTrack &Track );
glm::dvec3 rail_point( TTrack const &Track, int const Path, int const Side, double const Distance, double const Lateral );
void rail_offset( TTrack const &Track, int const Path, int const Side, glm::dvec3 const &Point, double &Distance, double &Lateral );

} // namespace track_rails
