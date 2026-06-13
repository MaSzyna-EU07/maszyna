#pragma once

#include <eu07/scene/node/header.hpp>
#include <eu07/scene/node/types.hpp>
#include <eu07/scene/runtime/transform.hpp>
#include <eu07/scene/runtime/types.hpp>

#include <cmath>
#include <vector>

namespace eu07::scene::bake {

[[nodiscard]] inline runtime::RuntimeScratchpad scratchFromHeader(const NodeHeader& header) {
    runtime::RuntimeScratchpad scratch;
    scratch.location.offsetStack.push_back(header.originOffset);
    scratch.location.rotation = header.rotation;
    scratch.location.scaleStack.push_back(header.scaleFactor);
    return scratch;
}

[[nodiscard]] inline runtime::WorldVertex meshVertexToWorld(const MeshVertex& v) {
    runtime::WorldVertex out;
    out.position = {v.x, v.y, v.z};
    out.normal = {v.nx, v.ny, v.nz};
    out.u = v.u;
    out.v = v.v;
    return out;
}

[[nodiscard]] inline bool degenerateTriangle(
    const runtime::Vec3& a,
    const runtime::Vec3& b,
    const runtime::Vec3& c) {
    const runtime::Vec3 ab{b.x - a.x, b.y - a.y, b.z - a.z};
    const runtime::Vec3 ac{c.x - a.x, c.y - a.y, c.z - a.z};
    const runtime::Vec3 cross{
        ab.y * ac.z - ab.z * ac.y,
        ab.z * ac.x - ab.x * ac.z,
        ab.x * ac.y - ab.y * ac.x};
    const double len2 = cross.x * cross.x + cross.y * cross.y + cross.z * cross.z;
    return len2 < 1e-12;
}

inline void pushTriangle(
    std::vector<runtime::WorldVertex>& out,
    const runtime::WorldVertex& v0,
    const runtime::WorldVertex& v1,
    const runtime::WorldVertex& v2) {
    if (degenerateTriangle(v0.position, v1.position, v2.position)) {
        return;
    }
    out.push_back(v0);
    out.push_back(v1);
    out.push_back(v2);
}

[[nodiscard]] inline std::vector<runtime::WorldVertex> triangulateTriangles(
    const std::vector<MeshVertex>& source) {
    std::vector<runtime::WorldVertex> out;
    out.reserve(source.size());
    for (std::size_t i = 0; i + 2 < source.size(); i += 3) {
        pushTriangle(
            out,
            meshVertexToWorld(source[i]),
            meshVertexToWorld(source[i + 1]),
            meshVertexToWorld(source[i + 2]));
    }
    return out;
}

[[nodiscard]] inline std::vector<runtime::WorldVertex> triangulateTriangleFan(
    const std::vector<MeshVertex>& source) {
    std::vector<runtime::WorldVertex> out;
    if (source.size() < 3) {
        return out;
    }
    const runtime::WorldVertex v0 = meshVertexToWorld(source[0]);
    runtime::WorldVertex v1 = meshVertexToWorld(source[1]);
    for (std::size_t i = 2; i < source.size(); ++i) {
        const runtime::WorldVertex v2 = meshVertexToWorld(source[i]);
        pushTriangle(out, v0, v1, v2);
        v1 = v2;
    }
    return out;
}

[[nodiscard]] inline std::vector<runtime::WorldVertex> triangulateTriangleStrip(
    const std::vector<MeshVertex>& source) {
    std::vector<runtime::WorldVertex> out;
    if (source.size() < 3) {
        return out;
    }
    runtime::WorldVertex v0 = meshVertexToWorld(source[0]);
    runtime::WorldVertex v1 = meshVertexToWorld(source[1]);
    for (std::size_t i = 2; i < source.size(); ++i) {
        const runtime::WorldVertex v2 = meshVertexToWorld(source[i]);
        if (i % 2 == 0) {
            pushTriangle(out, v0, v1, v2);
        } else {
            pushTriangle(out, v1, v0, v2);
        }
        v0 = v1;
        v1 = v2;
    }
    return out;
}

inline void computeBounds(runtime::RuntimeShapeNode& shape) {
    if (shape.vertices.empty()) {
        return;
    }
    runtime::Vec3 center{};
    for (const runtime::WorldVertex& v : shape.vertices) {
        center.x += v.position.x;
        center.y += v.position.y;
        center.z += v.position.z;
    }
    const double inv = 1.0 / static_cast<double>(shape.vertices.size());
    center.x *= inv;
    center.y *= inv;
    center.z *= inv;
    shape.node.area.center = center;

    double radiusSq = 0.0;
    for (const runtime::WorldVertex& v : shape.vertices) {
        const double dx = v.position.x - center.x;
        const double dy = v.position.y - center.y;
        const double dz = v.position.z - center.z;
        radiusSq = std::max(radiusSq, dx * dx + dy * dy + dz * dz);
    }
    shape.node.area.radius = static_cast<float>(std::sqrt(radiusSq));
}

inline void computeLineBounds(runtime::RuntimeLinesNode& lines) {
    if (lines.vertices.empty()) {
        return;
    }
    runtime::Vec3 center{};
    for (const runtime::WorldVertex& v : lines.vertices) {
        center.x += v.position.x;
        center.y += v.position.y;
        center.z += v.position.z;
    }
    const double inv = 1.0 / static_cast<double>(lines.vertices.size());
    center.x *= inv;
    center.y *= inv;
    center.z *= inv;
    lines.node.area.center = center;

    double radiusSq = 0.0;
    for (const runtime::WorldVertex& v : lines.vertices) {
        const double dx = v.position.x - center.x;
        const double dy = v.position.y - center.y;
        const double dz = v.position.z - center.z;
        radiusSq = std::max(radiusSq, dx * dx + dy * dy + dz * dz);
    }
    lines.node.area.radius = static_cast<float>(std::sqrt(radiusSq));
}

[[nodiscard]] inline runtime::LightingData bakeLighting(const NodeMaterial& material) {
    runtime::LightingData lit;
    lit.ambient = {
        static_cast<float>(material.ambient.r / 255.0),
        static_cast<float>(material.ambient.g / 255.0),
        static_cast<float>(material.ambient.b / 255.0),
        1.f};
    lit.diffuse = {
        static_cast<float>(material.diffuse.r / 255.0),
        static_cast<float>(material.diffuse.g / 255.0),
        static_cast<float>(material.diffuse.b / 255.0),
        1.f};
    lit.specular = {
        static_cast<float>(material.specular.r / 255.0),
        static_cast<float>(material.specular.g / 255.0),
        static_cast<float>(material.specular.b / 255.0),
        1.f};
    return lit;
}

[[nodiscard]] inline runtime::LightingData defaultShapeLighting() {
    return runtime::LightingData{};
}

[[nodiscard]] inline bool guessTranslucent(const std::string& texturePath) {
    return texturePath.find('@') != std::string::npos;
}

} // namespace eu07::scene::bake
