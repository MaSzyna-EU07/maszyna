#include "stdafx.h"
#include "BillboardSystem.h"

#include "entitysystem/ECWorld.h"
#include "entitysystem/components/BasicComponents.h"
#include "entitysystem/components/RenderComponents.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"

#include "gl/buffer.h"
#include "gl/vao.h"
#include "gl/shader.h"

struct BillboardSystem::RenderState
{
    std::optional<gl::buffer>    instanceBuffer;
    std::optional<gl::vao>       vao;
    std::unique_ptr<gl::program> shader;
    std::size_t                  bufferCapacity { 0 };
};

BillboardSystem::BillboardSystem() = default;
BillboardSystem::~BillboardSystem() = default;

void BillboardSystem::Render(ECWorld& world)
{
    // Group billboard instances by texture path
    std::unordered_map<std::string, std::vector<BillboardInstance>> groups;

    const glm::vec3 camPos{
        static_cast<float>(Global.pCamera.Pos.x),
        static_cast<float>(Global.pCamera.Pos.y),
        static_cast<float>(Global.pCamera.Pos.z)
    };

    world.Each<ECSComponent::Billboard, ECSComponent::Transform>(
        entt::exclude<ECSComponent::Disabled>,
        [&](entt::entity, ECSComponent::Billboard& bill, ECSComponent::Transform& t)
        {
            if (!bill.active) return;
            std::string key = bill.texturePath.ToString();
            groups[key].push_back({
                glm::vec3(t.Position) - camPos,
                bill.size
            });
        });

    if (groups.empty()) return;

    // Lazy-init shared GL resources
    if (!m_renderState)
        m_renderState = std::make_unique<RenderState>();

    auto& rs = *m_renderState;

    if (!rs.instanceBuffer)
        rs.instanceBuffer.emplace();

    if (!rs.shader) {
        try {
            gl::shader vert("ecs_billboard.vert");
            gl::shader frag("ecs_billboard.frag");
            rs.shader = std::unique_ptr<gl::program>(new gl::program({vert, frag}));
        } catch (const gl::shader_exception& e) {
            WriteLog("[ECS] BillboardSystem: shader compile failed: " + std::string(e.what()));
            return;
        }
    }

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    rs.shader->bind();

    for (auto& [texPath, instances] : groups) {
        // Grow VBO if needed
        if (instances.size() > rs.bufferCapacity) {
            rs.bufferCapacity = instances.size() + 64;
            rs.instanceBuffer->allocate(
                gl::buffer::ARRAY_BUFFER,
                static_cast<int>(rs.bufferCapacity * sizeof(BillboardInstance)),
                GL_DYNAMIC_DRAW);
            rs.vao.reset(); // rebuild after realloc
        }

        rs.instanceBuffer->upload(
            gl::buffer::ARRAY_BUFFER,
            instances.data(), 0,
            static_cast<int>(instances.size() * sizeof(BillboardInstance)));

        if (!rs.vao) {
            rs.vao.emplace();
            constexpr int stride = sizeof(BillboardInstance);
            rs.vao->setup_attrib(*rs.instanceBuffer, 0, 3, GL_FLOAT, stride, 0);  // position
            glVertexAttribDivisor(0, 1);
            rs.vao->setup_attrib(*rs.instanceBuffer, 1, 1, GL_FLOAT, stride, 12); // size
            glVertexAttribDivisor(1, 1);
            rs.vao->unbind();
            rs.instanceBuffer->unbind(gl::buffer::ARRAY_BUFFER);
        }

        // Bind texture (fallback: skip group if path is empty)
        if (!texPath.empty()) {
            auto texHandle = GfxRenderer->Fetch_Texture(texPath, true);
            GfxRenderer->Bind_Texture(0, texHandle);
        }

        rs.vao->bind();
        glDrawArraysInstanced(GL_TRIANGLES, 0, 6, static_cast<GLsizei>(instances.size()));
        rs.vao->unbind();
    }

    gl::program::unbind();
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}
