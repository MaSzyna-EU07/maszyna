#pragma once

#include "BaseSystem.h"

// Keeps ECS Transform in sync with the vehicle's actual world position each frame.
// Must run before all other systems that read Transform (e.g. LOD, Sound, Hierarchy).
class VehicleSyncSystem : public BaseSystem
{
public:
    void Update(ECWorld& world, float dt) override;
};
