#pragma once

#include <eu07/scene/bake/geometry.hpp>
#include <eu07/scene/bake/track.hpp>
#include <eu07/scene/node/traction_power.hpp>
#include <eu07/scene/runtime/nodes.hpp>

namespace eu07::scene::bake {

[[nodiscard]] inline runtime::PowerSourceModifier mapPowerModifier(
    const ParsedNodeTractionPowerSource& parsed) {
    if (parsed.section) {
        return runtime::PowerSourceModifier::Section;
    }
    if (parsed.recuperation) {
        return runtime::PowerSourceModifier::Recuperation;
    }
    return runtime::PowerSourceModifier::None;
}

[[nodiscard]] inline runtime::RuntimeTractionPowerSource bakeTractionPower(
    const ParsedNodeTractionPowerSource& parsed) {
    runtime::RuntimeTractionPowerSource source;
    source.node = bakeBasicNode(parsed.header, "tractionpowersource");

    const runtime::RuntimeScratchpad scratch = scratchFromHeader(parsed.header);
    source.position = runtime::transformPoint(parsed.position, scratch);
    source.node.area.center = source.position;

    source.nominalVoltage = static_cast<float>(parsed.nominalVoltage);
    source.voltageFrequency = static_cast<float>(parsed.voltageFrequency);
    source.maxOutputCurrent = static_cast<float>(parsed.maxOutputCurrent);
    source.fastFuseTimeout = static_cast<float>(parsed.fastFuseTimeout);
    source.fastFuseRepetition = static_cast<float>(parsed.fastFuseRepetition);
    source.slowFuseTimeout = static_cast<float>(parsed.slowFuseTimeout);
    source.modifier = mapPowerModifier(parsed);

    source.internalResistanceLegacy = parsed.internalResistance;
    float internalResistance = static_cast<float>(parsed.internalResistance);
    if (internalResistance < 0.1f) {
        internalResistance = 0.2f;
    }
    source.internalResistance = internalResistance;

    return source;
}

} // namespace eu07::scene::bake
