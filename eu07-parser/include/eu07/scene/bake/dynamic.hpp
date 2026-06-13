#pragma once

#include <eu07/scene/bake/track.hpp>
#include <eu07/scene/node/dynamic.hpp>
#include <eu07/scene/runtime/nodes.hpp>

namespace eu07::scene::bake {

[[nodiscard]] inline runtime::RuntimeDynamicObject bakeDynamic(const ParsedNodeDynamic& parsed) {
    runtime::RuntimeDynamicObject vehicle;
    vehicle.node = bakeBasicNode(parsed.header, "dynamic");
    vehicle.dataFolder = parsed.datafolder;
    vehicle.skinFile = parsed.skinfile;
    vehicle.mmdFile = parsed.mmdfile;
    vehicle.trackName = parsed.pathname;
    vehicle.offset = parsed.vehicleOffset;
    vehicle.driverType = parsed.driverType;
    vehicle.coupling = parsed.coupling.type;
    vehicle.couplingRaw = parsed.coupling.raw;
    vehicle.couplingParams = parsed.coupling.params;
    vehicle.velocity = static_cast<float>(parsed.velocity);
    vehicle.loadCount = parsed.loadcount;
    if (parsed.loadtype) {
        vehicle.loadType = *parsed.loadtype;
    }
    vehicle.destination = parsed.destination;
    if (parsed.header.trainsetIndex) {
        vehicle.trainsetIndex = *parsed.header.trainsetIndex;
    }
    return vehicle;
}

} // namespace eu07::scene::bake
