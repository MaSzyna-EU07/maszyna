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

// the rails cut where they cross other rails into frogs, wing rails and check rails; off, the rails of plain tracks and of
// switches with the key frogs are laid whole, as before
bool constexpr crossings { false };
// the 3D sleepers and switch timbers of sleepers3d; off, the tracks keep their ballast whatever eu07.ini sets
bool constexpr sleepers { false };
// the blades of the switches with the key frogs shaped as their templates have them, from the tip their planed nose ends at to the heel,
// with the rails left whole where the crossings are off; off with the crossings too, they're laid from the start of the switch as before
bool constexpr blades { true };

} // namespace track_features
