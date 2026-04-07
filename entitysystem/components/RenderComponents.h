#ifndef EU07_RENDERCOMPONENTS_H
#define EU07_RENDERCOMPONENTS_H

#include "registry/FName.h"
#include "rendering/geometrybank.h"

namespace ECSComponent
{
/// <summary>
/// Component for entities that can be rendered.
/// </summary>
/// <remarks>Currently empty
/// TODO: Add component members
/// </remarks>
struct MeshRenderer
{
	gfx::geometry_handle meshHandle;
	bool visible = true;
};

/// <summary>
/// Component for entities that can cast shadows.
/// </summary>
/// <remarks>Currently empty
/// TODO: Add component members
/// </remarks>
struct SpotLight
{
	glm::vec3 Color{ 1.0f, 1.0f, 1.0f };

    float Intensity  = 1.0f;    
    float Range = 25.0f;  

    float InnerAngle = 15.0f;  
    float OuterAngle = 25.0f;  

    bool CastShadows = false;
    bool Enabled = true;
};

// TODO: AreaLight like blender 
// TODO: SunLight for scene


struct ParticleEmitter
{
    FName particleEffectPath = ("");
    bool active = true;
};

struct Decal
{
    FName decalTexturePath = ("");
    float size = 1.0f;
    bool active = true;
};

struct Billboard
{
    FName texturePath = ("");
    float size = 1.0f;
    bool active = true;
};

struct Line {
    glm::vec3 start{ 0.0f };
    glm::vec3 end{ 1.0f, 0.0f, 0.0f };
    glm::vec3 color{ 1.0f, 1.0f, 1.0f };
    float thickness = 1.0f;
    bool active = true;
};
 


/// <summary>
/// Component for entities that can be rendered with LOD
/// </summary>
struct LODController
{
	double RangeMin;
	double RangeMax;
};

} // namespace ECSComponent
#endif // EU07_RENDERCOMPONENTS_H
