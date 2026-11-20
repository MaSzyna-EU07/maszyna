/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cctype>
#include <string_view>

// the pullee mode ("jazda ciągniona", MaSzyna-EU07/maszyna#64): a vehicle prepared for being hauled dead in a train,
// TMoverParameters::PulleeModeSwitch(). a scenery asks for it with the parameter H among the ones written after the
// coupling of the vehicle, split by dots: "3.H", or with others, "3.BG.H"
//   node -1 0 EU160-001 dynamic pkp/vectron_v1 1293-107 wariant_7w 0 nobody 3.H 0 enddynamic
namespace pullee_mode {

// whether the parameters of a vehicle, the text after the first dot of its coupling entry, ask for the mode. a
// parameter is told by its first letter, as the others are (B brakes, W wheels, T temperatures, L load), in either
// case; the letters after the H are left for later
inline bool requested(std::string_view const Parameters) {
	auto first{true}; // of a parameter
	for (auto const character : Parameters) {
		if (first && std::toupper(static_cast<unsigned char>(character)) == 'H') {
			return true;
		}
		first = character == '.';
	}
	return false;
}

} // namespace pullee_mode
