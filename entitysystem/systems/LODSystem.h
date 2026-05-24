#pragma once

#include "BaseSystem.h"

// Reads LODController + Transform on each entity and toggles MeshRenderer.visible
// based on distance to camera. Runs every Update tick.
class LODSystem : public BaseSystem
{
public:
    void Update(ECWorld& world, float dt) override;
};
