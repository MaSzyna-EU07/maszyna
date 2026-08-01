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
	// the everyday PKP rozjazdy zwyczajne. the crossing mark and radius are the defining pair;
	// the length the switch spans follows from them and is shown in the editor
	static std::vector<turnout_preset> const presets{
	    {"Rz 1:9 R190", 9.0, 190.0},
	    {"Rz 1:9 R300", 9.0, 300.0},
	    {"Rz 1:12 R300", 12.0, 300.0},
	    {"Rz 1:14 R500", 14.0, 500.0},
	    {"Rz 1:18,5 R1200", 18.5, 1200.0},
	    {"Rz 1:26,5 R2500", 26.5, 2500.0}};
	return presets;
}

} // namespace editor
