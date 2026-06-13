#pragma once

#include <eu07/scene/bake/geometry.hpp>
#include <eu07/scene/bake/track.hpp>
#include <eu07/scene/node/triangle_fan.hpp>
#include <eu07/scene/node/triangle_strip.hpp>
#include <eu07/scene/node/triangles.hpp>
#include <eu07/scene/runtime/nodes.hpp>

#include <optional>
#include <string>

namespace eu07::scene::bake {

[[nodiscard]] inline runtime::RuntimeShapeNode bakeShape(
    const NodeHeader& header,
    const std::string& nodeType,
    const std::string& texturePath,
    const std::optional<NodeMaterial>& material,
    std::vector<runtime::WorldVertex> vertices) {
    runtime::RuntimeShapeNode shape;
    shape.node = bakeBasicNode(header, nodeType);
    shape.materialPath = texturePath;
    shape.translucent = guessTranslucent(texturePath);
    shape.lighting = material ? bakeLighting(*material) : defaultShapeLighting();
    shape.origin = {};

    runtime::RuntimeScratchpad scratch;
    scratch.location.offsetStack.push_back(header.originOffset);
    scratch.location.rotation = header.rotation;
    runtime::transformShapeVertices(vertices, scratch);

    shape.vertices = std::move(vertices);
    computeBounds(shape);
    return shape;
}

[[nodiscard]] inline runtime::RuntimeShapeNode bakeTriangles(const ParsedNodeTriangles& parsed) {
    const bool hasMaterial =
        parsed.material.ambient.r != 0.0 || parsed.material.ambient.g != 0.0 ||
        parsed.material.ambient.b != 0.0 || parsed.material.diffuse.r != 0.0 ||
        parsed.material.diffuse.g != 0.0 || parsed.material.diffuse.b != 0.0;
    return bakeShape(
        parsed.header,
        "triangles",
        parsed.texture,
        hasMaterial ? std::optional<NodeMaterial>{parsed.material} : std::nullopt,
        triangulateTriangles(parsed.vertices));
}

[[nodiscard]] inline runtime::RuntimeShapeNode bakeTriangleStrip(const ParsedNodeTriangleStrip& parsed) {
    return bakeShape(
        parsed.header,
        "triangle_strip",
        parsed.texture,
        std::nullopt,
        triangulateTriangleStrip(parsed.vertices));
}

[[nodiscard]] inline runtime::RuntimeShapeNode bakeTriangleFan(const ParsedNodeTriangleFan& parsed) {
    return bakeShape(
        parsed.header,
        "triangle_fan",
        parsed.texture,
        std::nullopt,
        triangulateTriangleFan(parsed.vertices));
}

} // namespace eu07::scene::bake
