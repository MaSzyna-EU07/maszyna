/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <string_view>

// cab controls with more than one submodel (eu07/maszyna#527): a definition of a control in the .mmd marked
// "duplicate: yes" adds a copy of the control already defined in the cab, instead of replacing it
namespace gauge_duplicates {

// whether the value of the duplicate: key marks the definition as a duplicate
inline bool is_duplicate(std::string_view const Value) {
	return Value == "yes" || Value == "true" || Value == "1";
}

// whether a definition adds a copy to the control rather than replacing it: it is marked as a duplicate, and the
// control is already loaded from one of the 3d models of the cab being loaded (Cab, Lowpoly). a control left over
// from another cab, or not loaded at all (Loadedfrom null), has nothing in this cab to add the copy to
template <typename Model_> bool adds_copy(bool const Duplicate, Model_ const *Loadedfrom, Model_ const *Cab, Model_ const *Lowpoly) {
	return Duplicate && Loadedfrom != nullptr && (Loadedfrom == Cab || Loadedfrom == Lowpoly);
}

} // namespace gauge_duplicates
