//
// Created by Daniu
//

#ifndef EU07_PARTICLESSYSTEM_H
#define EU07_PARTICLESSYSTEM_H

#include "BaseSystem.h"
#include "entitysystem/ECWorld.h"
#include "entitysystem/components/RenderComponents.h"

class ParticlesSystem : public BaseSystem
{
public:
    ParticlesSystem();
    ~ParticlesSystem();
    void Update(ECWorld& world, float dt) override;
    void Render(ECWorld& world) override;

private:
    void SpawnParticle(ECSComponent::ParticleEmitter& emitter);

    // Per-instance data uploaded to GPU (one entry per live particle)
    struct GPUInstanceData {
        glm::vec3 position; // camera-relative world position
        float     size;
        glm::vec4 color;
    };

    // OpenGL resources — allocated lazily on first Render() call
    struct RenderState;
    std::unique_ptr<RenderState> m_renderState;
};

#endif // EU07_PARTICLESSYSTEM_H
