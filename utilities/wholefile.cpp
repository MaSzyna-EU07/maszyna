/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "utilities/wholefile.h"

#include <cstdio>

whole_file read_whole_file(std::string const &Path)
{
	whole_file result;

	auto *const file{std::fopen(Path.c_str(), "rb")};
	if (file == nullptr)
	{
		return result;
	}
	// without a buffer of its own the stream hands the read to the system in one piece, straight into the memory
	std::setvbuf(file, nullptr, _IONBF, 0);
#ifdef _WIN32
	_fseeki64(file, 0, SEEK_END);
	auto const length{_ftelli64(file)};
	_fseeki64(file, 0, SEEK_SET);
#else
	fseeko(file, 0, SEEK_END);
	auto const length{ftello(file)};
	fseeko(file, 0, SEEK_SET);
#endif
	if (length > 0)
	{
		result.data.reset(new char[static_cast<std::size_t>(length)]);
		result.size = std::fread(result.data.get(), 1, static_cast<std::size_t>(length), file);
	}
	std::fclose(file);

	return result;
}
