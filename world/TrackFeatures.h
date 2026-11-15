/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// features of the tracks switched on or off in the build as a whole, whatever the scenery or eu07.ini sets
namespace track_features {

// on: the blades of the switches with the key frogs shaped as their templates have them, from the tip their planed nose ends at to
// the heel. that's whether frogs in eu07.ini is on or off: with it off only the frogs aren't cut, the rails are left whole there and
// the blades shaped still. off: the blades laid from the start of the switch as before, for the switches with the key too
bool constexpr blades { true };

} // namespace track_features
