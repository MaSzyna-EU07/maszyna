/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// features of the tracks built into the simulator but switched off in it as a whole, whatever the scenery or eu07.ini sets
namespace track_features {

// the blades of the switches with the key frogs shaped as their templates have them, from the tip their planed nose ends at to the heel,
// with the rails left whole where the frogs aren't cut (frogs in eu07.ini off); off with the frogs too, they're laid from the start of
// the switch as before
bool constexpr blades { true };

} // namespace track_features
