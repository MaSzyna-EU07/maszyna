/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "editor/layout/layoutCommands.hpp"

#include <string>
#include <vector>

// A track laid alongside a line of the track geometry (editor-tracks-domain.md §8, "Odsuń tor równoległy"): a new line at a constant
// spacing, its straights offset and its arcs concentric with the line's, the radius less on the inside of a curve and more on the outside,
// the transition curves recomputed so the arcs stay concentric: the shift of the arc from its tangents taken so its centre is the line's.
// In a curve the spacing is widened, where the line's curve has transitions to take it, by the widening of the gauge of both tracks on the
// sides facing each other (PLK tom II annex 1, editorGauge.hpp): the norms of the track geometry have no spacing of their own yet. The new
// line is a create_line command, checked by the norms when it's carried out.
namespace layout
{

struct parallel_plan
{
	bool valid{false};
	create_line command;
	std::vector<std::string> errors; // why there's none
	std::vector<std::string> notes; // a curve the spacing isn't widened in, and why
	std::vector<double> spacings; // m, in the arc of each curve, the widening with it
};

// the line alongside the line at the offset, m, above 0 to its left as it runs (left_of), its spacing in the curves widened where Widen
parallel_plan parallel_line(network const &Network, line_id const Line, double const Offset, bool const Widen = true);

} // namespace layout
