/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <string>
#include <string_view>

// names of the scenery written in plain ASCII.
// a name with letters outside of it is read as UTF-8 when it's valid UTF-8, otherwise as Windows-1250,
// the code page of the most of the sceneries
namespace ascii_name
{

// true if the name holds nothing but ASCII
bool plain(std::string_view Name);
// the name in ASCII, in lower case as the parser gives the names: the letters with marks lose them (ł -> l, ß -> ss),
// whatever else isn't ASCII becomes '_'
std::string fold(std::string_view Name);
// the name as UTF-8, to be shown or logged: a name which isn't valid UTF-8 is taken for Windows-1250
std::string readable(std::string_view Name);

} // namespace ascii_name
