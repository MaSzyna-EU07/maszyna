#pragma once

#include "BaseSystem.h"

class MeshRenderSystem : public BaseSystem
{
public:
    void Render(ECWorld& world) override;
};
