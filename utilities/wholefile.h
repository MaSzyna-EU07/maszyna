/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstddef>
#include <memory>
#include <string>

// the content of a whole file, kept in memory
struct whole_file
{
	std::unique_ptr<char[]> data;
	std::size_t size{0};
};

// reads the whole file with one large read. a file stream reads a file which isn't in the system's cache yet from a
// hard disk in small pieces, slower (2026-11, .e3d files of a scenario: about 24 MB/s against 38 MB/s for one read).
// a file which can't be opened or is empty gives no data; a read which stops early gives what was read
whole_file read_whole_file(std::string const &Path);
