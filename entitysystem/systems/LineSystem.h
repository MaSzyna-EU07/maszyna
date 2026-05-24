#pragma once

#include "BaseSystem.h"

// Renders Line components as GL_LINES using camera-relative world positions.
// Uses vbocolor.vert + ecs_line.frag.
class LineSystem : public BaseSystem
{
public:
    LineSystem();
    ~LineSystem();
    void Render(ECWorld& world) override;

private:
    struct LineVertex {
        glm::vec3 position; // camera-relative world position
        glm::vec3 color;
    };

    struct RenderState;
    std::unique_ptr<RenderState> m_renderState;
};
