/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "scene/scenelayers.h"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

// The text of the scenery files read the way the simulator reads them (cParser, state_serializer), with no scene: the words, the
// statements and the includes followed. For what the editor finds in the files without loading them: the text of a track it wrote
// (the checksums of the project file), the files of a map with the //$ lines in them, the scenarios using a map and what they name in it.
namespace scenery_text
{

struct token
{
	std::string text; // as written, the parameters of the include put in place of (p1), (p2)...
	std::size_t file{0}; // of the files read (scenery::files), 0 where a text was split on its own
	std::size_t line{0}; // from 1
	std::size_t begin{0}; // the bytes of the file it takes, before the parameters were put in
	std::size_t end{0};
};

// the words of the text as cParser splits it: blanks, line ends and semicolons between them, the comments (// to the end of the line,
// /* to */) left out, a text in quotes one word with its quotes; the letters as they are written
std::vector<token> split(std::string_view const Text);
// the words after the include words of the text, the files it includes as written, for a quick look with no statements read: the
// comments not told apart, so it may name more files than the simulator includes, never fewer
std::vector<std::string> includes_in(std::string_view const Text);
// the letters A to Z made small, the rest as it is, as the parser does it with the names
std::string lowered(std::string_view const Text);
// the words joined by one blank each: the text of a statement as the simulator reads it, whatever the blanks and the comments
std::string joined(std::vector<token> const &Tokens, std::size_t const From = 0, std::size_t const To = static_cast<std::size_t>(-1));

// the node of the name and its text from the word node to the word ending it, the words joined (joined()); none where there's none
std::optional<std::string> node_text(std::string_view const Text, std::string_view const Name);
// the word ending a node of the type (endtrack for a track, end for an event launcher), empty for a type which isn't known
std::string_view node_end(std::string_view const Type);
// the word ending a statement other than a node (endevent for an event), empty for one which has none
std::string_view statement_end(std::string_view const Keyword);

// a file read: the root, or one an include names
struct included_file
{
	std::string name; // relative to the scenery folder, with forward slashes, as the include names it
	std::size_t parent{static_cast<std::size_t>(-1)}; // the file with the include, none for the root
	std::size_t line{0}; // of the include in it
	scene::layer_context context; // the placement in effect at the include (origin, rotate, scale)
	std::vector<std::string> parameters;
	bool map{false}; // the file of the map, or one it includes
	bool missing{false}; // it can't be read
	bool skipped{false}; // a terrain file passed over for the binary terrain of the scenario (.sbt), as the simulator does
};

// a statement of the scenery: its first word made small and the words after it up to the one ending it, that one included; a node with
// the words from the range on (<range max> <range min> <name> <type> ...)
struct statement
{
	std::string keyword;
	std::vector<token> words;
	std::size_t file{0};
	std::size_t line{0};
	bool map{false}; // it's in the files of the map

	// the word made small, empty past the last
	std::string word(std::size_t const Index) const { return Index < words.size() ? lowered(words[Index].text) : std::string{}; }
};

struct read_options
{
	// the file of the map, relative to the scenery folder: the statements of it and of the files it includes are marked as the map's
	std::string map;
	// the terrain files (*_ter.scm) passed over, as the simulator passes them over when the scenario has its binary terrain (.sbt)
	bool skip_terrain{false};
	// which statements are kept, by the keyword and the type of a node; all of them where there's none
	std::function<bool(std::string_view const Keyword, std::string_view const Type)> keep;
};

struct scenery
{
	std::vector<included_file> files; // the root first, then each include in the order they're met
	std::vector<statement> statements;
	std::vector<std::string> problems; // includes which can't be read, statements cut off by the end of the files
	bool uses_map() const;
};

// the scenery from the file Root, relative to the folder, with all its includes
scenery read(std::filesystem::path const &Folder, std::string const &Root, read_options const &Options = {});

// the same names of files, the case and the slashes aside
bool same_file(std::string_view const A, std::string_view const B);

} // namespace scenery_text
