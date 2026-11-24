/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "editor/layout/layoutAnchors.hpp"
#include "editor/layout/layoutNetwork.hpp"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <vector>

// The project file of the track geometry (editor-tracks-domain.md §9): the domain of the editor written next to the scenery, the
// .scn and .scm files staying what the simulator reads. One file per map, shared by its scenarios: <map>.eu07proj next to the file of
// the map. Plain text, a line a record, read and written here only; the schema version on the first line, a file of a newer one refused.
//
// Version 1:
//   eu07proj 1
//   next node <n> line <n> vertex <n> turnout <n>  the identifiers handed out next; none is handed out again
//   norms <profile>                                the profile of the scenery: plk, tsi, en13803, ch
//   area <name> <profile> <count> <x z>...         a profile for an area outlined in the plan
//   lineprofile <name> <profile>                   a profile for a line by name
//   class <n> vmax <km/h> vmin <km/h> traffic <passenger|mixed|freight> track <main|side> load <Tg> features <flags>
//   node <id> at <x y z> heading <x z> curvature <1/m> cant <mm> grade <ratio> ends <count> {<line> <start|end> <sense 1|-1>}
//   line <id> from <node> to <node> class <n> chainage <m> transition <clothoid|parabola|bloss> ramp <linear|bloss> continues <0|1> <0|1>
//        turnout <id, 0 for a line of its own> profile <profile, - for the one of its place>
//     vertex <id> at <x z> radius <m> in <m> out <m> cant <mm>
//     grade <chainage> <elevation> <vertical radius>
//     old <chainage> <name> reversed <0|1> continued <0|1> surface <words> attributes <words>   an old track the line was taken over from
//                                                  (E6): where its stretch begins, its name, whether it ran against the line, whether it's
//                                                  a part a split cut off, the words of its entry before the path and after it
//   end
//   turnouts
//     turnout <id> type <name> hand <1|-1> direction <1|-1> start <node> main <line> <node> diverging <line> <node> chainage <m>
//        radii <main> <diverging> cant <mm> speeds <main> <diverging> velocity <km/h> movable <0|1>
//        [second <node> <main track on> <diverging track> <node at its end>]   a double switch: its second switch (E5)
//        [crossing <line> <node> <node> slip <line, 0 for none>]                a slip or a crossing: the crossing track, the second slip
//                                                                                curve; the diverging track 0 0 of a crossing (E5)
//   end                                            the turnouts (E2b), their tracks lines of them; the ones they lay laid by the turnouts
//   anchors                                        the anchors of the lineside objects (E6)
//     anchor <id> <kind> <name> <key> group <n> at <x y z> yaw <deg> line <id> chainage <m> offset <m> height <m> turn <deg>
//        confidence <0..1> points <n> {<chainage> <offset> <height>} poles <n> {<anchor>} [why <reasons, | between them>]
//                                                  the object as it was found (its kind, name, key, category as editorInfra numbers
//                                                  it, where it stood and its yaw), where it stands along the line (the offset to the
//                                                  left, below 0 to the right), the points of a wire, the poles it hangs on
//   end
//   generated
//     track <name> line <id> file <file> sum <16 hex digits>   a track written by the generator, the checksum of its text
//     switch <name> line <id> file <file> sum <16 hex digits>  the switch track of a turnout, by its main track
//     hand <line>                                  a line whose tracks were changed outside the editor, not written over
//   end
//   legacy <mark> <file> line <id> [why <the rest of the line>]   //$ lines read when the map was first opened, as they were
//     | <the line>
//   end
// Lines starting with # are comments. The numbers are written in the shortest form which reads back to the same double; names and files
// with %20 for a blank and %25 for a percent sign.
namespace project
{

inline constexpr int schema_version{1};
inline constexpr std::string_view extension{".eu07proj"};

// a node of the simulator written by the generator: the name, the line it's made of (the main track of a turnout for its switch track),
// the file it's written into and the checksum of its text (checksum()), to tell a change made outside the editor
struct generated_node
{
	std::string kind{"track"};
	std::string name;
	layout::line_id line{layout::line_id::none};
	std::string file; // relative to the folder of the scenery, as the includes name it
	std::uint64_t sum{0};

	bool operator==(generated_node const &) const = default;
};

// the //$ lines of a kind read from a scenery file when the map was first opened (§9: they stay in the file until the author removes
// them): an //$h or an //$p record with the line it became, if it became one, and why not; the lines of the other kinds as a whole
struct legacy_record
{
	char mark{'h'};
	std::string file;
	layout::line_id line{layout::line_id::none};
	std::string why;
	std::vector<std::string> lines; // the mark taken off

	bool operator==(legacy_record const &) const = default;
};

struct document
{
	layout::network network; // the lines, the nodes and the turnouts, the profiles of the norms by place, the identifiers handed out
	std::vector<generated_node> generated;
	std::set<layout::line_id> hand_changed;
	std::vector<layout::anchor> anchors; // of the lineside objects
	std::vector<legacy_record> legacy;
};

// the same contents: the lines and the nodes, the norms, the identifiers handed out and the rest
bool same(document const &A, document const &B);

std::string write(document const &Project);

struct read_result
{
	std::optional<document> project;
	std::string error; // why there's none: not a project file, a newer schema, a line it can't read
	std::vector<std::string> notes; // the lines which can't be laid as they are written
};
// the lines are laid between their nodes as read; the nodes keep what's written of them
read_result read(std::string_view const Text);

// <map>.eu07proj next to the file of the map
std::filesystem::path path_of(std::filesystem::path const &Map);
read_result load(std::filesystem::path const &Path);
// written whole, through a file next to it put in its place, so a failed write leaves the old one; false with the reason in Error
bool save(document const &Project, std::filesystem::path const &Path, std::string &Error);

// the words of the file: a name with no blanks, %20 for a blank, %25 for a percent sign; - for none
std::string word(std::string_view const Text);
std::string unword(std::string_view const Word);

} // namespace project
