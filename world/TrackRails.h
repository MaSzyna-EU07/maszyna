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

namespace track_rails {

double constexpr flangeway { 0.045 };
double constexpr blend { 0.015 };
double constexpr tread_fade { 0.02 };
double constexpr wheel_width { 0.14 };
double constexpr back_to_back { 1.36 };
double constexpr rolled_band { wheel_width - ( 1.435 - back_to_back ) / 2.0 };
double constexpr nose_tip { 0.012 };
double constexpr middle_angle { 0.35 };
double constexpr wide_angle { 0.785 };

void forget();
// how far a wing rail reaches past the crossing point of the rails
double wing_reach( double const Slope );
double check_length( double const Slope );
// the rails of the track are cut where they cross other rails: plain tracks, and switches whose frog is made from the rails
bool cut_rails( TTrack const &Track );
int blade_steps( TTrack const &Track );
double blade_length( TTrack const &Track, int const Path );
material_handle cut_material( material_handle const Material );
float tread_brightness( material_handle const Material, glm::vec2 const &Band );
glm::vec3 tread_colour( material_handle const Material, glm::vec2 const &Band );
material_handle tint_material( material_handle const Material, glm::vec3 const &Colour );
// triangles closing the first and/or the last cross-section of a lofted strip, made of pairs of points for each of Ring profile points, both sides of each
void cap_loft( gfx::vertex_array const &Strip, std::size_t const Ring, bool const Start, bool const End, gfx::vertex_array &Triangles );
gfx::userdata_array worn_userdata( std::size_t const Count );
// the ends of rails left open: at track ends with nothing joined, blade tips, the gap of a frog
void cut_faces( gfx::vertex_array const &Strip, std::size_t const Ring, bool const Start, bool const End, gfx::geometrybank_handle const &Bank, TTrack::geometryhandle_sequence &Geometry );

glm::dvec3 rail_point( TTrack const &Track, int const Path, int const Side, double const Distance, double const Lateral );
void rail_offset( TTrack const &Track, int const Path, int const Side, glm::dvec3 const &Point, double &Distance, double &Lateral );

} // namespace track_rails
