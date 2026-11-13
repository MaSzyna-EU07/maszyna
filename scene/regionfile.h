/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstdint>
#include <functional>
#include <iosfwd>
#include <string>

// the parts of the region file (<scenery>.sbt, the static geometry of a scenery cached by basic_region) which need
// nothing of the scene: checking how a file is laid out, and writing it so that an interrupted write leaves no
// cut-short file behind. kept apart so the unit tests can check them
namespace scene::region_file {

// layout of the file: the header (two uint32: the file type and its version), the section count (uint32), then for
// each section its index (uint32), its size (uint32, counted from the start of the index, so it covers the index,
// the size itself and the section data) and the section data. all numbers little endian

// size of the index and the size of a section, the least a section can take
std::uint32_t constexpr section_header_size { 2 * sizeof( std::uint32_t ) };

// checks the layout of the file held by the stream, read from its start: the header has to match the provided type
// and version, the sections have to have indices below Sectionlimit, each one no more than once, and their sizes have
// to add up to the length of the file. the section data itself isn't read. returns: the empty string if the layout
// holds, the reason why not otherwise
std::string
    check_layout( std::istream &Input, std::uint32_t const Header, std::uint32_t const Version, std::uint32_t const Sectionlimit );

// reads the index and the size of the next section, and checks them: the index has to be below Sectionlimit, and the
// section no smaller than its own index and size. returns: the empty string if both were read and are sound, the
// reason why not otherwise
std::string
    read_section_header( std::istream &Input, std::uint32_t const Sectionlimit, std::uint32_t &Index, std::uint32_t &Size );

// writes a file with the provided function into a temporary file next to it (the name with .tmp added), which then
// replaces the file in one go. if the writing fails or is cut short, the previous file (or none) stays in place,
// rather than a partial one; the temporary file is removed when the failure is noticed. returns: the empty string on
// success, the reason of the failure otherwise
std::string
    write_through_temporary( std::string const &Filename, std::function<void( std::ostream & )> const &Write );

} // scene::region_file
