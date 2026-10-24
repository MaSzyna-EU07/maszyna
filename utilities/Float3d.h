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

using float3 = glm::vec3;

inline float3 SafeNormalize(const float3 &v)
{ // bezpieczna normalizacja (wektor długości 1.0)
    auto const l = glm::length(v);
    float3 retVal;
    if (l == 0)
    {
        retVal.x = 0;
        retVal.y = 0;
        retVal.z = 0;
    }
    else
        retVal = v / l;
    return retVal;
};

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


// From code in Graphics Gems; p. 766
inline float Det2x2(float a, float b, float c, float d)
{ // obliczenie wyznacznika macierzy 2×2
    return a * d - b * c;
};

inline float Det3x3(float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2,
                    float c3)
{ // obliczenie wyznacznika macierzy 3×3
    return +a1 * Det2x2(b2, b3, c2, c3) - b1 * Det2x2(a2, a3, c2, c3) + c1 * Det2x2(a2, a3, b2, b3);
};