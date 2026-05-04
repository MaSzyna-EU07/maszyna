#pragma once
#include <glm/glm.hpp>

namespace glmHelpers
{

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

inline void InitialRotate(glm::mat4 &m)
{
	for (int col = 0; col < 4; ++col)
	{
		m[col].x = -m[col].x;

		float tmp = m[col].y;
		m[col].y = m[col].z;
		m[col].z = tmp;
	}
}
} // namespace GlmHelpers