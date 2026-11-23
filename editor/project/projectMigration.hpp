/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "editor/project/projectFile.hpp"
#include "scene/scenelayers.h"

#include <filesystem>
#include <map>
#include <string>
#include <string_view>
#include <vector>

// The //$ lines of an old scenery taken into the project file when its map is first opened (editor-tracks-domain.md §9, §10): what the
// domain has a place for becomes its lines and nodes, the rest is kept as it was written; the lines stay in the scenery files, untouched.
//
//   //$h  an alignment designed: a line between two nodes, its vertices with their radii, transitions and cants, the shape of its
//         transitions, its chainage at the start (the origin) and its speed as the class of the line. Not one the domain lays otherwise:
//         a compound curve, two transitions with no arc between them, cant ramps of its own, a curve turning the longer way, a design kept
//         for a part of its line, one the domain lays to another length. The ends of two lines at one place join at a node. The record is
//         kept as it was, with the line it became: the points clicked, the old norms, the pieces, the fit have no place in the domain.
//   //$p  a grade line designed along such a line: its breaks with their vertical curves as the breaks of the line, the heights of its
//         ends as those of the nodes. Not one with a break with no vertical curve, or along no line of an alignment. Kept as it was too.
//   //$r  a switch, a double switch of the editor on a line made of an alignment, of a type of the catalogue (its type, rail, radius, ratio,
//         a and b), along the line or against it, bent as its record says the line is: a turnout laid into the line, as the command lays
//         it, the norms not refusing what the old line breaks already; its record kept on its own with the main track of the turnout.
//         Not a slip nor a crossing (their records don't tell the side of the crossing track), a connection (the solver of the
//         connections lays it anew), a lifted one, one of no type of the catalogue or on no line: kept in the block of the file, why with it.
//   //$k  the kilometrage of the railway lines, //$t the tracks fitted, //$w the tracks sharing a grade line, //$b the objects bound to
//         the tracks (E6), //$s the stations (E7): no place in the domain yet, kept as they were.
//   //$c, //$o, //$z  the camera, the orthophoto and the area of the downloads: what the editor keeps of itself in the scenery, not the
//         track geometry; not taken.
namespace project
{

// the //$ lines of a scenery file by the letter of their mark, the mark taken off as the editor takes it (scene::node_layers::scan()):
// the lines other tools put after the same marks left out, those of the old letters of the marks under the new ones
std::map<char, std::vector<std::string>> marked_lines(std::string_view const Text);

struct marked_file
{
	std::string file; // relative to the scenery folder
	scene::layer_context context; // the placement in effect where the lines are, their coordinates are those of the file
	std::map<char, std::vector<std::string>> lines;
};

struct migration
{
	document project;
	std::vector<std::string> notes; // what was kept as it was and why, the ends joined at the nodes
};
// the project of the //$ lines of the files, in the order of the files; the norms by place as given
migration migrate(std::vector<marked_file> const &Files, norms::area_map const &Areas = {});

// the files of the map (Map relative to the scenery folder, its includes followed) with their //$ lines and the placement in effect at
// their includes
std::vector<marked_file> marked_files(std::filesystem::path const &Folder, std::string const &Map);

struct opened
{
	read_result project;
	bool migrated{false}; // there was no project file: it was made of the //$ lines and written
	std::vector<std::string> notes;
};
// the project of the map: its file read, or made of the //$ lines of the files of the map where there's none yet and written next to it;
// the scenery files aren't written
opened open_project(std::filesystem::path const &Folder, std::string const &Map, norms::area_map const &Areas = {});

} // namespace project
