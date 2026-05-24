#ifndef EU07_RENDERCOMPONENTS_H
#define EU07_RENDERCOMPONENTS_H

#include "registry/FName.h"
#include "rendering/geometrybank.h"

class TAnimModel;

namespace ECSComponent
{
/// <summary>
/// Component for entities that can be rendered.
/// </summary>
struct MeshRenderer
{
	gfx::geometry_handle meshHandle;
	TAnimModel* modelInstance = nullptr;
	bool visible = true;
};

/// <summary>
/// Component for entities that can cast shadows.
/// </summary>
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

/// <summary>
/// 
///</summary>
struct AreaLight
{
	glm::vec3 Color{ 1.0f, 1.0f, 1.0f };
	float Intensity = 1.0f;
	float Width = 1.0f;
	float Height = 1.0f;
	bool CastShadows = false;
	bool Enabled = true;
};

struct SunLight
{
	glm::vec3 Color{ 1.0f, 1.0f, 1.0f };
	float Intensity = 1.0f;
	bool CastShadows = false;
	bool Enabled = true;
};


struct Particle {
	glm::vec3 position;
	glm::vec3 velocity;
	glm::vec4 color;
	float size;
	float age;
	float maxAge;
};

struct ParticleEmitter {

	// --- Kontener i Stan ---
	std::vector<Particle> particles;
	uint32_t maxParticles = 1000;
	bool isActive = true;
	float spawnAccumulator = 0.0f;

	// --- Konfiguracja Emisji ---
	float spawnRate = 20.0f;           // cząstek na sekundę
	float particleLifetime = 2.0f;     // bazowy czas życia
	glm::vec3 gravity{ 0.0f, 0.0f, 0.0f }; // np. dym: 0.5, deszcz: -9.81
	float airResistance = 0.1f;        // tłumienie prędkości

	// --- Zakresy (losowanie) ---
	glm::vec3 minStartVelocity{ -0.5f, 1.0f, -0.5f };
	glm::vec3 maxStartVelocity{ 0.5f, 2.0f, 0.5f };

	// --- Ewolucja w czasie (Modyfikatory) ---
	float sizeGrowth = 0.5f;           // zmiana rozmiaru na sekundę
	glm::vec4 colorFade = { 0.0f, 0.0f, 0.0f, -0.5f }; // np. znikanie (alpha)

	// --- Pozycja emitera ---
	glm::vec3 emitterLocation{ 0.0f, 0.0f, 0.0f };

	// --- Flag Funkcjonalnych ---
	bool hasCollision = false;         // czy sprawdzać kolizje
	float bounceFactor = 0.3f;         // energia po odbiciu (0.0 - 1.0)
};

/// <summary>
/// Component for entities that can be rendered as decals (e.g. graffiti on wagons, dirties on wall).
/// </summary>
struct Decal
{
    FName decalTexturePath = ("");
    float size = 1.0f;
    bool active = true;
};


/// <summary>
///	Component for entities that can be rendered as billboards (e.g. particles, sprites).
///  </summary>
struct Billboard
{
    FName texturePath = ("");
    float size = 1.0f;
    bool active = true;
};

/// <summary>
/// Component for entities that can be rendered as lines.
/// </summary>
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
