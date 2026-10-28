#pragma once

#include <string>
#include <string_view>
#include <vector>

class cParser;

// what the game's vehicle loader (TMoverParameters::LoadFIZ_Parse) makes of the lines of a vehicle definition,
// for the test definitions to give it the same and for checking they do
namespace test::fiz {

// a section of the definition, opened by a line starting with its name and separator. a list section is
// followed by rows, read until the line closing them (an empty line for the Cntrl brake table) and as
// long as no other list closes first, see reading()
struct section {
	std::string_view name;
	char separator;
	unsigned list{0}; // the bit of the list its rows go to, none for a section of values only
	char const *end{nullptr}; // the line closing the rows
};

// the section of the given name, or none for a name the loader doesn't know
section const *named(std::string_view Name);
// the section the line opens, or none
section const *opened_by(std::string_view Line);

// the lines of a definition as the loader takes them: without comments, leading spaces and line ends
std::vector<std::string> lines(cParser &Parser);

// the keys of the values in the line, in the order of their first appearance: the words followed by =,
// which extract_value() finds
std::vector<std::string> keys(std::string const &Line);

// the value of the key in the line, as extract_value() gives it, empty for none (where it would throw too)
std::string value(std::string const &Key, std::string const &Line);
// the words of a row, as the readers of the lists get them from cParser
std::vector<std::string> words(std::string const &Line);

// what the loader reads from the lines, in order: every section line with the values it gives, every row
// with the lists reading it. two definitions reading the same load the same vehicle
std::vector<std::string> reading(std::vector<std::string> const &Lines);

} // namespace test::fiz
