#pragma once

#include "BaseSystem.h"

// Manages ECS light components (SpotLight, AreaLight, SunLight).
// Updates light position/direction from Transform each tick.
// Actual renderer integration is handled by the rendering pipeline reading these components.
class LightSystem : public BaseSystem
{
public:
    void Update(ECWorld& world, float dt) override;
    void Render(ECWorld& world) override;
};
