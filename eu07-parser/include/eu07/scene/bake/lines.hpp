#pragma once

#include <eu07/scene/bake/geometry.hpp>
#include <eu07/scene/bake/track.hpp>
#include <eu07/scene/node/line_loop.hpp>
#include <eu07/scene/node/line_strip.hpp>
#include <eu07/scene/node/lines.hpp>
#include <eu07/scene/runtime/nodes.hpp>

#include <algorithm>

namespace eu07::scene::bake {

[[nodiscard]] inline runtime::RuntimeLinesNode bakeLinesNode(
    const NodeHeader& header,
    const std::string& nodeType,
    const MaterialRgb& color,
    const double thickness,
    std::vector<runtime::WorldVertex> vertices) {
    runtime::RuntimeLinesNode lines;
    lines.node = bakeBasicNode(header, nodeType);
    lines.lineWidth = static_cast<float>(std::min(30.0, thickness));
    lines.lighting.diffuse = {
        static_cast<float>(color.r / 255.0),
        static_cast<float>(color.g / 255.0),
        static_cast<float>(color.b / 255.0),
        1.f};
    lines.origin = {};

    runtime::RuntimeScratchpad scratch;
    scratch.location.offsetStack.push_back(header.originOffset);
    scratch.location.rotation = header.rotation;
    runtime::transformShapeVertices(vertices, scratch);

    lines.vertices = std::move(vertices);
    computeLineBounds(lines);
    return lines;
}

[[nodiscard]] inline runtime::WorldVertex pointVertex(const Vec3& p) {
    runtime::WorldVertex v;
    v.position = p;
    return v;
}

[[nodiscard]] inline runtime::RuntimeLinesNode bakeLines(const ParsedNodeLines& parsed) {
    std::vector<runtime::WorldVertex> vertices;
    vertices.reserve(parsed.segments.size() * 2);
    for (const LineSegment& seg : parsed.segments) {
        vertices.push_back(pointVertex(seg.a));
        vertices.push_back(pointVertex(seg.b));
    }
    return bakeLinesNode(parsed.header, "lines", parsed.color, parsed.thickness, std::move(vertices));
}

[[nodiscard]] inline runtime::RuntimeLinesNode bakeLineStrip(const ParsedNodeLineStrip& parsed) {
    std::vector<runtime::WorldVertex> vertices;
    if (parsed.points.size() > 1) {
        vertices.reserve((parsed.points.size() - 1) * 2);
        for (std::size_t i = 1; i < parsed.points.size(); ++i) {
            vertices.push_back(pointVertex(parsed.points[i - 1]));
            vertices.push_back(pointVertex(parsed.points[i]));
        }
    }
    return bakeLinesNode(parsed.header, "line_strip", parsed.color, parsed.thickness, std::move(vertices));
}

[[nodiscard]] inline runtime::RuntimeLinesNode bakeLineLoop(const ParsedNodeLineLoop& parsed) {
    std::vector<runtime::WorldVertex> vertices;
    if (parsed.points.size() > 1) {
        vertices.reserve(parsed.points.size() * 2);
        for (std::size_t i = 1; i < parsed.points.size(); ++i) {
            vertices.push_back(pointVertex(parsed.points[i - 1]));
            vertices.push_back(pointVertex(parsed.points[i]));
        }
        vertices.push_back(pointVertex(parsed.points.back()));
        vertices.push_back(pointVertex(parsed.points.front()));
    }
    return bakeLinesNode(parsed.header, "line_loop", parsed.color, parsed.thickness, std::move(vertices));
}

} // namespace eu07::scene::bake
