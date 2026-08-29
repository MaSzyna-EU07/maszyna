/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/turnouts.h"

#include "maj0sted/domain/geometry/turnout.hpp"

#include <cmath>

namespace editor
{

namespace
{

// the blade ends where the gap to the stock rail takes the przekladka in its heel.
// this is what fixes the length of a curved blade, so it is said once here.
// TODO: dlugosci iglic z tablic Id-1, gdy beda pod reka
double const heel_offset{0.136}; // metres
// 49E1 railhead. the blade is planed from its tip until it is this wide
double const railtop_width{0.070};

// a turnout with an iglica lukowa styczna do opornicy o scietym ostrzu (Koc, rys. 5.9).
// the curve is theoretically tangent to the opornica at PR, but the blade cannot begin
// there - it begins where there is metal enough, at the ostrze, and its planed nose (d)
// runs on to the point where the rail is u thick. what lies between PR and the ostrze is
// the odcinek przediglicowy: the curve is already there, the vehicle is still on the
// through track. Koc's tablica 5.1 gives a, d and u per type, and they obey
//     a + d = the run to a gap of u, taken on the blade's curve R + s/2
// so a is derived from u and d here rather than copied. The drawing dimensions it on the
// rail and a piece length is on the axis, so what goes in is the run scaled by R/(R+s/2);
// against the table itself that is a tenth of a millimetre at 1200-1:18,5, six at 190-1:9.
//
// b is the rear tangent from the drawing: a rozjazd podstawowy (krzyzownica lukowa) has
// b = R*tg(alfa/2) and no straight at all, while a straight crossing carries the arc out
// on a prosta krzyzownicowa of b - R*tg(alfa/2). Passing 0 asks for the podstawowy.
turnout_preset curved_tangent(std::string name, double crossing_n, double radius, double tip_thickness, double nose, double back_tangent = 0.0)
{
	auto const alfa{std::atan(1.0 / crossing_n)};
	auto const tangent{radius * std::tan(0.5 * alfa)};
	auto const b{back_tangent > 0.0 ? back_tangent : tangent};
	auto const lead{maj0sted::domain::blade_run_to_offset(radius, 0.0, tip_thickness) - nose};
	auto const blade{maj0sted::domain::blade_run_to_offset(radius, 0.0, heel_offset) - lead};

	turnout_preset preset{std::move(name), crossing_n, tangent + b, tip_thickness, nose, railtop_width, {}};
	preset.pieces.push_back({0, lead, radius, radius, 0.0});
	preset.pieces.push_back({1, blade, radius, radius, 0.0});
	preset.pieces.push_back({2, radius * alfa - lead - blade, radius, radius, 0.0});
	if (b - tangent > 1e-6)
	{
		preset.pieces.push_back({3, b - tangent, 0.0, 0.0, 0.0});
	}
	return preset;
}

} // namespace

std::vector<turnout_preset> const &turnout_presets()
{
	// the standard PKP rozjazdy zwyczajne. u and d come from Koc, tablica 5.1 (iglice
	// styczne do opornicy o scietym ostrzu); the rear tangents from the catalogue drawings:
	// rys. 5.13 gives Rz 49E1-190-1:9 as 27138 = 10523 + 16615 (krzyzownica prosta) and
	// rys. 5.15 gives Rz 49E1-300-1:9 as 33230 = 16615 + 16615 (lukowa, rozjazd podstawowy).
	// krzyzownice proste sa w 49E1-190-1:9 i 49E1-500-1:14, lukowe w 300-1:9, 500-1:12 i
	// 1200-1:18,5. TODO: 1:7,5 R190 i 1:14 R500 czekaja na wlasne u/d i styczne z Id-1
	static std::vector<turnout_preset> const presets{
	    curved_tangent("Rz 1:7,5 R190", 7.5, 190.0, 0.0049, 0.1250),
	    curved_tangent("Rz 1:9 R190", 9.0, 190.0, 0.0049, 0.1250, 16.615),
	    curved_tangent("Rz 1:9 R300", 9.0, 300.0, 0.0044, 0.1250),
	    curved_tangent("Rz 1:12 R500", 12.0, 500.0, 0.0053, 0.1250),
	    curved_tangent("Rz 1:14 R500", 14.0, 500.0, 0.0053, 0.1250, 24.544),
	    curved_tangent("Rz 1:14 R760", 14.0, 760.0, 0.0050, 0.1250),
	    curved_tangent("Rz 1:18,5 R1200", 18.5, 1200.0, 0.0050, 0.1304)};
	return presets;
}

} // namespace editor
