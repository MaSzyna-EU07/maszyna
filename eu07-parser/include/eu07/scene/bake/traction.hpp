#pragma once

#include <eu07/scene/bake/geometry.hpp>
#include <eu07/scene/bake/track.hpp>
#include <eu07/scene/node/traction.hpp>
#include <eu07/scene/runtime/nodes.hpp>

namespace eu07::scene::bake {

[[nodiscard]] inline runtime::TractionWireMaterial mapWireMaterial(const TractionMaterialKind kind) {
    switch (kind) {
    case TractionMaterialKind::None:
        return runtime::TractionWireMaterial::None;
    case TractionMaterialKind::Aluminum:
        return runtime::TractionWireMaterial::Aluminium;
    default:
        return runtime::TractionWireMaterial::Copper;
    }
}

[[nodiscard]] inline Vec3 offsetPoint(const Vec3& point, const Vec3& origin) {
    return {point.x + origin.x, point.y + origin.y, point.z + origin.z};
}

[[nodiscard]] inline runtime::RuntimeTraction bakeTraction(const ParsedNodeTraction& parsed) {
    runtime::RuntimeTraction traction;
    traction.node = bakeBasicNode(parsed.header, "traction");
    traction.powerSupplyName = parsed.powerSourceName;
    traction.nominalVoltage = static_cast<float>(parsed.nominalVoltage);
    traction.maxCurrent = static_cast<float>(parsed.maxCurrent);

    traction.resistivityLegacy = parsed.resistivity;
    double resistivity = parsed.resistivity;
    if (resistivity == 0.01) {
        resistivity = 0.075;
    }
    traction.resistivityOhmPerM = static_cast<float>(resistivity * 0.001);

    traction.materialRaw = parsed.materialRaw;
    traction.material = mapWireMaterial(parsed.materialKind);
    traction.wireThickness = static_cast<float>(parsed.wireThickness);
    traction.damageFlag = parsed.damageFlag;

    const Vec3& origin = parsed.header.originOffset;
    traction.wireP1 = offsetPoint(parsed.wireLowerStart, origin);
    traction.wireP2 = offsetPoint(parsed.wireLowerEnd, origin);
    traction.wireP3 = offsetPoint(parsed.wireUpperStart, origin);
    traction.wireP4 = offsetPoint(parsed.wireUpperEnd, origin);
    traction.minHeight = parsed.minHeight;
    traction.segmentLength = parsed.segmentLength;
    traction.wireCount = parsed.wires;
    traction.wireOffset = static_cast<float>(parsed.wireOffset);
    traction.parallelName = parsed.parallel;
    traction.node.visible = parsed.visibilityMode == NodeVisibilityMode::Visible;

    traction.node.area.center = {
        (traction.wireP2.x + traction.wireP1.x) * 0.5,
        (traction.wireP2.y + traction.wireP1.y) * 0.5,
        (traction.wireP2.z + traction.wireP1.z) * 0.5};

    return traction;
}

} // namespace eu07::scene::bake
