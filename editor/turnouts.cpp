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

namespace editor
{

std::vector<turnout_preset> const &turnout_presets()
{
	// the standard PKP rozjazdy zwyczajne (49E1 / 60E1). crossing mark, radius and catalogue length
	// (PR→KR along the through track). the crossing angles check out as atan(1/n); the lengths follow
	// the type tables (metres). the diverging leaves not tangent but at the blade angle (beta) after a
	// straight pre-blade lead, then arcs by (alfa - beta) to the frog.
	//
	// blade_angle and pre_blade are NOT yet from Id-1 - left at 0 (a tangent arc from PR) until the
	// catalogue figures are in. Fill them per type when we have them; the geometry already supports it.
	static std::vector<turnout_preset> const presets{
	    {"Rz 1:7,5 R190", 7.5, 190.0, 25.222, 0.0, 0.0},
	    {"Rz 1:9 R190", 9.0, 190.0, 27.138, 0.0, 0.0},
	    {"Rz 1:9 R300", 9.0, 300.0, 33.230, 0.0, 0.0},
	    {"Rz 1:12 R500", 12.0, 500.0, 41.594, 0.0, 0.0},
	    {"Rz 1:14 R500", 14.0, 500.0, 42.371, 0.0, 0.0},
	    {"Rz 1:14 R760", 14.0, 760.0, 54.216, 0.0, 0.0},
	    {"Rz 1:18,5 R1200", 18.5, 1200.0, 64.818, 0.0, 0.0}};
	return presets;
}

} // namespace editor
