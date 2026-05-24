//
// Created by Daniu
//

#include "stdafx.h"
#include "ParticlesSystem.h"

#include "entitysystem/components/BasicComponents.h"
#include "entitysystem/components/RenderComponents.h"
#include "utilities/Globals.h"

#include "gl/buffer.h"
#include "gl/vao.h"
#include "gl/shader.h"

#include "utilities/Logs.h"
#include <algorithm>
#include <random>

// ---- OpenGL resource bundle (pimpl) ----------------------------------------

struct ParticlesSystem::RenderState
{
    std::optional<gl::buffer>          instanceBuffer;
    std::optional<gl::vao>             vao;
    std::unique_ptr<gl::program>       shader;
    std::size_t                        bufferCapacity { 0 };
};

ParticlesSystem::ParticlesSystem() = default;
ParticlesSystem::~ParticlesSystem() = default;

// ---- Update ----------------------------------------------------------------

void ParticlesSystem::Update(ECWorld &world, float dt)
{
    world.Each<ECSComponent::ParticleEmitter>(
        entt::exclude<ECSComponent::Disabled>,
        [&](entt::entity entity, ECSComponent::ParticleEmitter& emitter)
    {
        if (!emitter.isActive && emitter.particles.empty()) return;

        for (size_t i = 0; i < emitter.particles.size(); )
        {
            auto& p = emitter.particles[i];
            p.age += dt;

            if (p.age >= p.maxAge || p.color.a <= 0.0f) {
                p = emitter.particles.back();
                emitter.particles.pop_back();
                continue;
            }

            p.velocity += emitter.gravity * dt;
            p.velocity *= (1.0f - emitter.airResistance * dt);

            glm::vec3 nextPos = p.position + p.velocity * dt;
            if (emitter.hasCollision && nextPos.y < 0.0f) {
                p.velocity.y *= -emitter.bounceFactor;
                p.velocity.x *= 0.8f;
                nextPos.y = 0.01f;
            }
            p.position = nextPos;

            p.size  += emitter.sizeGrowth * dt;
            p.color += emitter.colorFade  * dt;

            ++i;
        }

        if (emitter.isActive) {
            emitter.spawnAccumulator += dt;
            float spawnInterval = 1.0f / emitter.spawnRate;

            while (emitter.spawnAccumulator >= spawnInterval) {
                if (emitter.particles.size() < emitter.maxParticles)
                    SpawnParticle(emitter);
                emitter.spawnAccumulator -= spawnInterval;
            }
        }
    });
}

void ParticlesSystem::SpawnParticle(ECSComponent::ParticleEmitter& emitter)
{
    static thread_local std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);

    ECSComponent::Particle p{};
    p.position = emitter.emitterLocation;
    p.velocity.x = emitter.minStartVelocity.x + dist(gen) * (emitter.maxStartVelocity.x - emitter.minStartVelocity.x);
    p.velocity.y = emitter.minStartVelocity.y + dist(gen) * (emitter.maxStartVelocity.y - emitter.minStartVelocity.y);
    p.velocity.z = emitter.minStartVelocity.z + dist(gen) * (emitter.maxStartVelocity.z - emitter.minStartVelocity.z);
    p.color  = glm::vec4(1.0f);
    p.size   = 0.1f;
    p.age    = 0.0f;
    p.maxAge = emitter.particleLifetime;

    emitter.particles.push_back(p);
}

// ---- Render ----------------------------------------------------------------

void ParticlesSystem::Render(ECWorld& world)
{
    // Collect instance data for every live particle across all emitters
    std::vector<GPUInstanceData> instances;
    instances.reserve(4096);

    const glm::vec3 camPos{
        static_cast<float>(Global.pCamera.Pos.x),
        static_cast<float>(Global.pCamera.Pos.y),
        static_cast<float>(Global.pCamera.Pos.z)
    };

    world.Each<ECSComponent::ParticleEmitter>(
        entt::exclude<ECSComponent::Disabled>,
        [&](entt::entity, ECSComponent::ParticleEmitter& emitter)
    {
        for (const auto& p : emitter.particles) {
            if (p.color.a <= 0.0f) continue;
            instances.push_back({
                p.position - camPos,   // camera-relative world position
                std::max(p.size, 0.01f),
                p.color
            });
        }
    });

    if (instances.empty()) return;

    // Lazy-init OpenGL resources
    if (!m_renderState)
        m_renderState = std::make_unique<RenderState>();

    auto& rs = *m_renderState;

    // Grow VBO if needed
    if (!rs.instanceBuffer)
        rs.instanceBuffer.emplace();

    if (instances.size() > rs.bufferCapacity) {
        rs.bufferCapacity = instances.size() + 1024; // headroom
        rs.instanceBuffer->allocate(
            gl::buffer::ARRAY_BUFFER,
            static_cast<int>(rs.bufferCapacity * sizeof(GPUInstanceData)),
            GL_DYNAMIC_DRAW);
        // VAO needs rebuild after buffer reallocation
        rs.vao.reset();
    }

    rs.instanceBuffer->upload(
        gl::buffer::ARRAY_BUFFER,
        instances.data(), 0,
        static_cast<int>(instances.size() * sizeof(GPUInstanceData)));

    // Build VAO once (or after buffer realloc)
    if (!rs.vao) {
        rs.vao.emplace();
        constexpr int stride = sizeof(GPUInstanceData);
        rs.vao->setup_attrib(*rs.instanceBuffer, 0, 3, GL_FLOAT, stride, 0);   // position
        glVertexAttribDivisor(0, 1);
        rs.vao->setup_attrib(*rs.instanceBuffer, 1, 1, GL_FLOAT, stride, 12);  // size
        glVertexAttribDivisor(1, 1);
        rs.vao->setup_attrib(*rs.instanceBuffer, 2, 4, GL_FLOAT, stride, 16);  // color
        glVertexAttribDivisor(2, 1);
        rs.vao->unbind();
        rs.instanceBuffer->unbind(gl::buffer::ARRAY_BUFFER);
    }

    // Load shader once
    if (!rs.shader) {
        try {
            gl::shader vert("ecs_particle.vert");
            gl::shader frag("ecs_particle.frag");
            rs.shader = std::unique_ptr<gl::program>(new gl::program({vert, frag}));
        } catch (const gl::shader_exception& e) {
            WriteLog("[ECS] ParticlesSystem: shader compile failed: " + std::string(e.what()));
            return;
        }
    }

    // Render
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);

    rs.shader->bind();
    rs.vao->bind();
    glDrawArraysInstanced(GL_TRIANGLES, 0, 6, static_cast<GLsizei>(instances.size()));
    rs.vao->unbind();
    gl::program::unbind();

    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}
