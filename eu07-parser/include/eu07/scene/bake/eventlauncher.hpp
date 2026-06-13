#pragma once

#include <eu07/scene/bake/geometry.hpp>
#include <eu07/scene/bake/track.hpp>
#include <eu07/scene/node/eventlauncher.hpp>
#include <eu07/scene/runtime/nodes.hpp>

#include <cmath>

namespace eu07::scene::bake {

[[nodiscard]] inline double parseLauncherTime(const std::string& token) {
    try {
        return std::stod(token);
    } catch (...) {
        return -1.0;
    }
}

[[nodiscard]] inline runtime::RuntimeEventLauncher bakeEventlauncher(const ParsedNodeEventlauncher& parsed) {
    runtime::RuntimeEventLauncher launcher;
    launcher.node = bakeBasicNode(parsed.header, "eventlauncher");

    const runtime::RuntimeScratchpad scratch = scratchFromHeader(parsed.header);
    launcher.location = runtime::transformPoint(parsed.position, scratch);
    launcher.node.area.center = launcher.location;

    if (parsed.radius > 0.0) {
        launcher.radiusSquared = parsed.radius * parsed.radius;
    }

    launcher.activationKeyRaw = parsed.key;
    launcher.event1Name = parsed.event1;
    launcher.event2Name = parsed.event2.value_or("none");

    double delta = parseLauncherTime(parsed.time);
    if (delta > 0.0) {
        launcher.launchMinute = static_cast<int>(delta) % 100;
        launcher.launchHour = static_cast<int>(delta - launcher.launchMinute) / 100;
        launcher.deltaTime = 0.0;
    } else {
        launcher.deltaTime = delta < 0.0 ? -delta : delta;
    }

    if (parsed.condition) {
        runtime::EventLauncherCondition cond;
        cond.memcellName = parsed.condition->memcell;
        cond.compareText = parsed.condition->parameter;
        cond.compareValue1 = parsed.condition->value1.value_or(0.0);
        cond.compareValue2 = parsed.condition->value2.value_or(0.0);
        cond.checkMask = parsed.condition->checkMask;
        launcher.condition = cond;
    }

    launcher.trainTriggered = parsed.trainTriggered;
    return launcher;
}

} // namespace eu07::scene::bake
