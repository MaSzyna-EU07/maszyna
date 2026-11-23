/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "editor/layout/layoutGenerator.hpp"
#include "editor/project/projectFile.hpp"

#include <cstdint>
#include <functional>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <vector>
#include <glm/glm.hpp>

// The nodes of the simulator the generator wrote, kept in the project file by their names with the checksums of their text, and what the
// scenery files hold of them now (editor-tracks-domain.md §9): a track changed outside the editor, or taken away, marks its line changed
// by hand, and the editor doesn't write over it until the author decides.
namespace project
{

// FNV-1a, 64 bits, of the text as the simulator reads it: the words joined by one blank each, the comments left out, so the line ends
// and the blanks another tool writes don't count as a change
std::uint64_t checksum(std::string_view const Text);

// the tracks of the line as the generator made them, written into the file, take the place of the ones the project had of the line
void record_generated(document &Project, layout::generated_line const &Line, std::string const &File);
// the switch track of the turnout as the generator made it, written into the file, takes the place of the one the project had of it; it's
// kept by the main track of the turnout (Project.network has the turnout), which a change of it by hand marks
void record_generated(document &Project, layout::generated_turnout const &Turnout, std::string const &File);

struct hand_change
{
	layout::line_id line{layout::line_id::none};
	std::string name;
	std::string file;
	bool missing{false}; // the file has no node of the name any more, or the file can't be read
};
// the generated nodes against the text of the files now: Files gives the text of a file by its name in the project, none where it can't be
// read. The ones whose text isn't the one written, in the order of the project
std::vector<hand_change> find_hand_changes(document const &Project, std::function<std::optional<std::string>(std::string const &)> const &Files);
// their lines marked as changed by hand in the project; the lines newly marked
std::set<layout::line_id> mark_hand_changes(document &Project, std::vector<hand_change> const &Changes);
// the editor may write the tracks of the line again: none of them was changed by hand, or the author decided to write over them
bool may_overwrite(document const &Project, layout::line_id const Line);

// the entry of a track the generator wrote, a plain track or a switch track, as written into a file of another placement: the points of its
// paths less Shift (the origin of the file the entry goes into less the one it was written for), the control vectors as they are,
// relative to them; the words joined by one blank each. None where it isn't an entry of a track of the generator's
std::optional<std::string> moved_entry(std::string_view const Text, glm::dvec3 const &Shift);

} // namespace project
