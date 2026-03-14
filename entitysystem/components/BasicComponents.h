/*
 *	Basic components used by almost everything
 */
#ifndef EU07_BASICCOMPONENTS_H
#define EU07_BASICCOMPONENTS_H
#include "entt/entity/entity.hpp"
#include "glm/vec3.hpp"
#include "glm/gtc/quaternion.hpp"
#include <string>
#include <cstdint>

namespace ECSComponent
{
///< summary>
/// Basic component for storing transform of entities
///</summary>
struct Transform
{
	glm::vec3 Position{0.f}; // object position
	glm::quat Rotation{1.f, 0.f, 0.f, 0.f}; // object rotation
	glm::vec3 Scale{1.f}; // object scale
};

///< summary>
/// Basic component for naming entities
/// in future for scenery hierarchy
///</summary>
struct Identification
{
	std::string Name; // object name - may contain slashes for editor hierarchy "directories"
	std::uint64_t Id{0}; // id in scene
};

///< summary>
/// Basic component for parent-child relationships between entities
///</summary>
struct Parent
{
	entt::entity value{entt::null};
};

///< summary>
/// Empty  component for entities that are disabled and should not be processed by systems
///</summary>
struct Disabled
{
};
} // namespace ECSComponent

#endif // EU07_BASICCOMPONENTS_H
