/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <string_view>

// whether a catenary file made in the editor can be named so: the save writes it into the folder of the scenery and
// includes it with "catenary <file> endcatenary", which the parser reads as one token. So no white space, no ';' (both
// end a token), no quotes, no ':' (a drive), no empty part of the path (a leading or a doubled separator, which is also
// the start of a comment) and no ".." (out of the folder of the scenery). A subfolder of it is fine
inline bool catenary_file_name_valid(std::string_view const Name)
{
	if (Name.empty())
		return false;
	for (auto const character : Name)
	{
		auto const code{static_cast<unsigned char>(character)};
		if (code <= ' ' || code == 127 || character == ';' || character == '"' || character == ':')
			return false;
	}
	std::size_t start{0};
	while (start <= Name.size())
	{
		auto end{Name.find_first_of("/\\", start)};
		if (end == std::string_view::npos)
			end = Name.size();
		if (auto const part{Name.substr(start, end - start)}; part.empty() || part == "..")
			return false;
		start = end + 1;
	}
	return true;
}
