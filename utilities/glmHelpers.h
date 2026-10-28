#pragma once
#include <glm/glm.hpp>

inline glm::dmat4 BasisChange(glm::dvec3 u, glm::dvec3 v, glm::dvec3 n)
{
	return glm::dmat4{glm::dvec4(u.x, v.x, n.x, 0.0), glm::dvec4(u.y, v.y, n.y, 0.0), glm::dvec4(u.z, v.z, n.z, 0.0), glm::dvec4(0.0, 0.0, 0.0, 1.0)}; // 4 columns; first rows: u, v, n
}

inline glm::dmat4 BasisChange(glm::dvec3 v, glm::dvec3 n)
{
	glm::dvec3 u = glm::cross(v, n);
	return BasisChange(u, v, n);
}

template <typename T> inline glm::vec<3, T> RotateY(const glm::vec<3, T> &v, T angle)
{
	T s = std::sin(angle);
	T c = std::cos(angle);

	return glm::vec<3, T>(c * v.x + s * v.z, v.y, c * v.z - s * v.x);
}

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