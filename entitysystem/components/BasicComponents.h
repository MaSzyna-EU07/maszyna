/*
 *	Basic components used by almost everything
 */
#ifndef EU07_BASICCOMPONENTS_H
#define EU07_BASICCOMPONENTS_H
#include "audio/sound.h"
#include "entt/entity/entity.hpp"
#include "glm/vec3.hpp"
#include "glm/gtc/quaternion.hpp"
#include "registry/FName.h"
#include "utils/uuid.hpp"
#include <cstdint>
#include <string>

namespace ECSComponent
{
///< summary>
/// Basic component for storing transform of entities
///</summary>
struct Transform
{
	glm::dvec3 Position{0.f}; // object position
	glm::quat Rotation{1.f, 0.f, 0.f, 0.f}; // object rotation
	glm::vec3 Scale{1.f}; // object scale
};

struct Velocity
{
	glm::vec3 Value{0.f}; // object velocity
};

///< summary>
/// Basic component for naming entities
/// in future for scenery hierarchy
///</summary>
struct Identification
{
	FName Name; // object name - may contain slashes for editor hierarchy "directories"
	std::uint64_t Id{0}; // id in scene
};

///< summary>
/// Basic component for parent-child relationships between entities
///</summary>
struct Parent
{
	entt::entity value{entt::null};
};

struct SoundComponent
{
	sound_source sound;   // sound source
	float volume = 1.0f; // 0–1
	float pitch  = 1.0f; // speed / pitch
	float range = 1.0f; // range
	bool loop = false;
	bool playOnStart = false;
	bool isPlaying = false;
};
///< summary>
/// Empty  component for entities that are disabled and should not be processed by systems
///</summary>
struct Disabled
{
};
} // namespace ECSComponent

#endif // EU07_BASICCOMPONENTS_H
