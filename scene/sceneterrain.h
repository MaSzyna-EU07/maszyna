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
#include <string>

namespace scene
{

class basic_region;
class basic_section;
struct terrain_block;

// binary terrain file (.btf): static geometry of a text terrain file (.txtf), arranged by the sections
// of the scene so each can be loaded on its own, when it comes into use.
// the binary file records length and checksum of the text it was made from, and is made anew when these don't match
class terrain_file
{
  public:
	// types
	// what a scenery is left with for a terrain file it refers to
	enum class state
	{
		binary, // binary file is in place and up to date
		text, // binary file can't be had, the text file has to be loaded the way any other scenery file is
		missing // there's nothing to load
	};
	// methods
	// calculates checksum (crc-32) of provided data
	static std::uint32_t checksum(char const *Data, std::size_t Size);
	// converts specified text terrain file to binary format. returns: true on success, false otherwise.
	// description of the result or of the failure is stored in Message, if provided
	static bool convert(std::string const &Textfile, std::string const &Binaryfile, std::string *Message = nullptr);
	// makes sure binary version of specified text terrain file is in place and matches the text, as far as the settings ask for it
	static state prepare(std::string const &Textfile, std::string const &Binaryfile);
	// makes content of specified binary terrain file a part of provided region. the geometry is loaded right away,
	// or left for the sections of the region to load when they're about to be used. returns: true on success, false otherwise
	static bool attach(std::string const &Binaryfile, basic_region &Region, bool Deferred);
	// loads specified piece of a binary terrain file into provided section
	static void load(basic_section &Section, terrain_block const &Block);

  private:
	// converts provided content of a text terrain file to binary format and stores it in specified file. returns: true on success, false otherwise
	static bool write(std::string const &Text, std::string const &Binaryfile, std::string &Message);
};

} // namespace scene

//---------------------------------------------------------------------------
