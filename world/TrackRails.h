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
double constexpr rolled_band { 0.1 };
double constexpr nose_tip { 0.012 };
double constexpr middle_angle { 0.35 };
double constexpr wide_angle { 0.785 };

// how far a wing rail reaches past the crossing point of the rails
double wing_reach( double const Slope );
double check_length( double const Slope );
// the rails of the track are cut where they cross other rails: plain tracks, and switches whose frog is made from the rails
bool cut_rails( TTrack const &Track );
int blade_steps( TTrack const &Track );
double blade_length( TTrack const &Track, int const Path );
material_handle cut_material( material_handle const Material );
// triangles closing the first and/or the last cross-section of a lofted strip, made of pairs of points for each of Ring profile points, both sides of each
void cap_loft( gfx::vertex_array const &Strip, std::size_t const Ring, bool const Start, bool const End, gfx::vertex_array &Triangles );
gfx::userdata_array worn_userdata( std::size_t const Count );
// the ends of rails left open: at track ends with nothing joined, blade tips, the gap of a frog
void cut_faces( gfx::vertex_array const &Strip, std::size_t const Ring, bool const Start, bool const End, gfx::geometrybank_handle const &Bank, TTrack::geometryhandle_sequence &Geometry );

} // namespace track_rails
