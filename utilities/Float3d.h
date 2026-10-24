/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cmath>
//---------------------------------------------------------------------------

inline glm::quat SafeNormalize(const glm::quat &q)
{
	const float len2 = glm::dot(q, q);

	if (len2 == 0.0f)
		return glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

	return glm::normalize(q);
}

inline void InitialRotateMatrix(glm::mat4 &matrix)
{
	for (int i = 0; i < 4; ++i)
	{
		matrix[i][0] = -matrix[i][0]; // zmiana znaku X

		const float f = matrix[i][1];
		matrix[i][1] = matrix[i][2];
		matrix[i][2] = f; // zamiana Y i Z
	}
}

void deserialize_float32(std::istream &s, glm::mat4 &matrix);

void deserialize_float64(std::istream &s, glm::mat4 &matrix);

void serialize_float32(std::ostream &s, const glm::mat4 &matrix);