#pragma once

#include <algorithm>
#include <cstdint>

#include <eu07/scene/bake/geometry.hpp>
#include <eu07/scene/bake/track.hpp>
#include <eu07/scene/node/model.hpp>
#include <eu07/scene/runtime/nodes.hpp>

namespace eu07::scene::bake {

[[nodiscard]] inline runtime::RuntimeModelInstance bakeModel(const ParsedNodeModel& parsed) {
    runtime::RuntimeModelInstance model;
    model.node = bakeBasicNode(parsed.header, "model");
    model.isTerrain = parsed.header.rangeMin < 0.0;

    const runtime::RuntimeScratchpad scratch = scratchFromHeader(parsed.header);
    model.location = runtime::transformPoint(parsed.location, scratch);

    model.angles = parsed.header.rotation;
    model.angles.y += parsed.rotationY;
    if (parsed.angles) {
        model.angles.x += parsed.angles->x;
        model.angles.y += parsed.angles->y;
        model.angles.z += parsed.angles->z;
    }

    model.scale = parsed.header.scaleFactor;
    if (parsed.inlineScale) {
        model.scale.x *= parsed.inlineScale->x;
        model.scale.y *= parsed.inlineScale->y;
        model.scale.z *= parsed.inlineScale->z;
    }

    model.modelFile = parsed.modelPath;
    model.textureFile = parsed.replaceableSkin;
    model.transition = !parsed.noTransition;

    if (parsed.lightStates) {
        model.lightStates.reserve(parsed.lightStates->size());
        for (double v : *parsed.lightStates) {
            model.lightStates.push_back(static_cast<float>(v));
        }
    }

    if (parsed.lightColors) {
        model.lightColors.reserve(parsed.lightColors->size());
        for (const MaterialRgb& rgb : *parsed.lightColors) {
            const auto pack = [](double c) {
                return static_cast<std::uint32_t>(std::clamp(c, 0.0, 255.0));
            };
            const std::uint32_t r = pack(rgb.r);
            const std::uint32_t g = pack(rgb.g);
            const std::uint32_t b = pack(rgb.b);
            model.lightColors.push_back((r << 16) | (g << 8) | b);
        }
    }

    model.node.area.center = model.location;
    return model;
}

} // namespace eu07::scene::bake
