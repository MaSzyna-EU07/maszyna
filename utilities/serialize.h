/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/
#pragma once

#include "scene/sn_utils.h"

//---------------------------------------------------------------------------

inline void deserialize_float32(std::istream &s, glm::mat4 &matrix)
{
	for (int column = 0; column < 4; ++column)
		for (int row = 0; row < 4; ++row)
			matrix[column][row] = sn_utils::ld_float32(s);
}

inline void deserialize_float64(std::istream &s, glm::mat4 &matrix)
{
	for (int column = 0; column < 4; ++column)
		for (int row = 0; row < 4; ++row)
			matrix[column][row] = static_cast<float>(sn_utils::ld_float64(s));
}

inline void serialize_float32(std::ostream &s, const glm::mat4 &matrix)
{
	for (int column = 0; column < 4; ++column)
		for (int row = 0; row < 4; ++row)
			sn_utils::ls_float32(s, matrix[column][row]);
}
