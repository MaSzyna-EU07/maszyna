#include "stdafx.h"
#include "LineSystem.h"

#include "entitysystem/ECWorld.h"
#include "entitysystem/components/BasicComponents.h"
#include "entitysystem/components/RenderComponents.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"

#include "gl/buffer.h"
#include "gl/vao.h"
#include "gl/shader.h"

struct LineSystem::RenderState
{
    std::optional<gl::buffer>    vertexBuffer;
    std::optional<gl::vao>       vao;
    std::unique_ptr<gl::program> shader;
    std::size_t                  bufferCapacity { 0 };
};

LineSystem::LineSystem() = default;
LineSystem::~LineSystem() = default;

void LineSystem::Render(ECWorld& world)
{
    std::vector<LineVertex> vertices;
    vertices.reserve(256);

    const glm::vec3 camPos{
        static_cast<float>(Global.pCamera.Pos.x),
        static_cast<float>(Global.pCamera.Pos.y),
        static_cast<float>(Global.pCamera.Pos.z)
    };

    world.Each<ECSComponent::Line, ECSComponent::Transform>(
        entt::exclude<ECSComponent::Disabled>,
        [&](entt::entity, ECSComponent::Line& line, ECSComponent::Transform& t)
        {
            if (!line.active) return;

            glm::vec3 worldOrigin = glm::vec3(t.Position);
            vertices.push_back({ worldOrigin + line.start - camPos, line.color });
            vertices.push_back({ worldOrigin + line.end   - camPos, line.color });
        });

    if (vertices.empty()) return;

    if (!m_renderState)
        m_renderState = std::make_unique<RenderState>();

    auto& rs = *m_renderState;

    if (!rs.vertexBuffer)
        rs.vertexBuffer.emplace();

    if (vertices.size() > rs.bufferCapacity) {
        rs.bufferCapacity = vertices.size() + 64;
        rs.vertexBuffer->allocate(
            gl::buffer::ARRAY_BUFFER,
            static_cast<int>(rs.bufferCapacity * sizeof(LineVertex)),
            GL_DYNAMIC_DRAW);
        rs.vao.reset();
    }

    rs.vertexBuffer->upload(
        gl::buffer::ARRAY_BUFFER,
        vertices.data(), 0,
        static_cast<int>(vertices.size() * sizeof(LineVertex)));

    if (!rs.vao) {
        rs.vao.emplace();
        constexpr int stride = sizeof(LineVertex);
        rs.vao->setup_attrib(*rs.vertexBuffer, 0, 3, GL_FLOAT, stride, 0);   // position
        rs.vao->setup_attrib(*rs.vertexBuffer, 1, 3, GL_FLOAT, stride, 12);  // color
        rs.vao->unbind();
        rs.vertexBuffer->unbind(gl::buffer::ARRAY_BUFFER);
    }

    if (!rs.shader) {
        try {
            gl::shader vert("vbocolor.vert");
            gl::shader frag("ecs_line.frag");
            rs.shader = std::unique_ptr<gl::program>(new gl::program({vert, frag}));
        } catch (const gl::shader_exception& e) {
            WriteLog("[ECS] LineSystem: shader compile failed: " + std::string(e.what()));
            return;
        }
    }

    rs.shader->bind();
    rs.vao->bind();
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(vertices.size()));
    rs.vao->unbind();
    gl::program::unbind();
}
