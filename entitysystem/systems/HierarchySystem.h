#pragma once

#include "BaseSystem.h"

// Propagates parent world Transform to children.
// Runs after MovementSystem so children always inherit the parent's
// already-updated position in the same frame.
// Child's Transform.Position is stored as a LOCAL offset relative to
// the parent and this system overwrites it with the computed world position.
class HierarchySystem : public BaseSystem
{
public:
    void Update(ECWorld& world, float dt) override;
};
