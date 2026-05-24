#pragma once

#include "BaseSystem.h"
#include "rendering/renderer.h"
#include <unordered_map>
#include <string>

// Renders Billboard components as camera-facing textured quads.
// Instances sharing the same texture are batched into one draw call.
class BillboardSystem : public BaseSystem
{
public:
    BillboardSystem();
    ~BillboardSystem();
    void Render(ECWorld& world) override;

private:
    struct BillboardInstance {
        glm::vec3 position; // camera-relative
        float     size;
    };

    struct RenderState;
    std::unique_ptr<RenderState> m_renderState;
};
