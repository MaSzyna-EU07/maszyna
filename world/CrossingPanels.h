/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <string>
#include <vector>

#include "model/vertex.h"

// the crossing panels of PKP PLK made of their drawings, for a sweep (world/Sweep.h) to lay along a track in the place of a model, bent
// along it, leaning with its cant: the concrete panels CBP of Id-111 (PKP PLK, Wymagania techniczne i badania prefabrykowanych
// wielkogabarytowych płyt żelbetowych do nawierzchni przejazdów kolejowych, 2010), some 80% of the level crossings in Poland. Three
// pieces, each 3000 mm long: the inner middle panel PW and the inner end panel PWS between the rails, the corners of PWS cut at its
// outer end for the wheels to come on, and the outer panel PZ outside a rail. A sweep names one in the place of its model:
//   "@cbp/<piece><height>-<gauge>[x<count>]"
//   piece: pw, pws, pz a single panel; in the inner row, a PWS at either end with the cut outwards and PW between them (2 at least);
//   out the outer row, PZ one after another
//   height: 180 (for the rail 60E1) or 140 (for 49E1); gauge: 1435 or 1520 (the inner ones 1300 or 1380 mm wide); count: of the row;
//   s<slope>: an outer one bent, its top rising by the slope (in 0.1 per mille) per metre away from the head over the plane of the heads
//   (from outer_gap off its edge by the rail),
//   to lie on the surface between two tracks, which goes from the plane of the heads of one to the plane of the heads of the other
// e.g. "@cbp/in180-1435x3", "@cbp/out180-1435x3". A piece lies in the space of a model: along z from 0 to its length, an inner one
// across x about 0, an outer one from its edge by the rail at x 0 to 0.64 m at +x (a sweep turns the -x side of a model to its curve),
// its top at y 0. What isn't on the drawings is said so ([N]): an editor choice
namespace crossing_panels
{

// Id-111 ch. 3: PW 3000 x 1300 (1380) x 140/180, PWS the same with the corners cut on one side, PZ 3000 x 640 x 140/180
double constexpr length{3.0};
double constexpr outer_width{0.640};
// 1.300, 1.380 m for the gauge of 1520 mm
double inner_width(int const Gauge);
// Id-111, annex 1, fig. 1-3 (PWS): the top narrows from 1300 (1380) to 1000 (1080) mm at the end over 260 mm, 150 x 260 a corner
double constexpr corner_across{0.150};
double constexpr corner_along{0.260};
// the edges: the steel angle L50x50x5 along both edges of PW and PWS and along the edge of PZ by the rail (Id-111 ch. 4); under it the
// side goes straight down 110 mm (the 180 mm panels, variant I, fig. 2, 5, 8) or 70 mm (the 140 mm ones, fig. 1, 4, 7), then slopes
// 150 mm in to the bottom
double constexpr angle_leg{0.050};
double constexpr edge_slope{0.150};
double edge_drop(int const Height);
// the joint of two panels: the top of each end set back by 5 mm, 10 mm between two panels [N]: Id-111 gives none, the panels of the
// game are 10 mm apart
double constexpr joint{0.005};
// the outer panel from the field side of the head [N]: the sets of the game have 10-65 mm
double constexpr outer_gap{0.030};
// Id-111 ch. 3: on straights and on curves of R 600 m and more
double constexpr minimum_radius{600.0};
// the sockets under the panels 600 mm apart, a half one at either end (Id-111, annex 1): the joints over the sleepers, which at a crossing
// of the panels are 0.60 m apart (STWiORB D.11.01.01, PKP PLK)
double constexpr sleeper_spacing{0.600};
int constexpr most{12};
// the texture of the panels of the game (models/przejazdy/cbp-*)
extern char const *const texture;

enum class piece
{
	pw,
	pws,
	pz,
	inner_row,
	outer_row
};
struct spec
{
	piece kind{piece::pw};
	int height{180};
	int gauge{1435};
	int count{1};
	double slope{0.0}; // of an outer one, see above
};
// true if the name is one of the panels; Spec receives what it says
bool parse(std::string const &Name, spec &Spec);
std::string name(spec const &Spec);
// the height of the panels for a rail as high as given (m): 180 mm for 160 mm and more (60E1 is 172 mm), 140 below (49E1, 149 mm).
// Id-111 names the two heights and the two rails without pairing them; paired by the height of the rail, the top of a panel lying at
// the top of the head with its bottom on the sleeper
int height_for(double const Rail);
// the length of the piece along the track
double piece_length(spec const &Spec);
// the triangles of the piece in the space of a model, the texture of the game laid on: the concrete of the top, the steel of the angles
std::vector<world_vertex> geometry(spec const &Spec);

} // namespace crossing_panels
