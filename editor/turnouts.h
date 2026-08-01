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

namespace editor
{

// a standard turnout template: its designation and the defining pair a catalogue entry is built on -
// the crossing mark 1:n (skos) and the radius of the diverging arc. the actual length the switch
// works out to is computed geometrically from these (arc = R * atan(1/n)), shown next to it; the
// exact catalogue length can be pinned later if the Id-1 tables call for a different figure
struct turnout_preset
{
	std::string name;
	double crossing_n;
	double radius;
};

// typical Polish turnouts (rozjazdy zwyczajne). the (skos, R) pairs are the recognised standard;
// confirm against Id-1 before treating the derived length as the catalogue length
std::vector<turnout_preset> const &turnout_presets();

} // namespace editor
