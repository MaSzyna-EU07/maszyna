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
#include <vector>

// the settings changed in the game written back into eu07.ini: only the values of the keys given change, the rest of the file
// (the other keys, the comments, the layout, the encoding) stays byte for byte

namespace ini_settings
{

struct value
{
	std::string key;
	std::string text; // the value as it's written after the key
};

// Text with the values given in place of those its keys have. a key which is there more than once gets the value in each place,
// one which isn't is added in a line of its own before endconfig (after which nothing is read) or at the end. the lines added end
// as the lines of Text do
std::string with_values(std::string_view Text, std::vector<value> const &Values);

// whether Text is valid UTF-8; the files of the game may be in Windows-1250 instead
bool is_utf8(std::string_view Text);

} // namespace ini_settings
