#pragma once

#include <eu07/scene/cursor.hpp>
#include <eu07/scene/document.hpp>
#include <eu07/scene/match.hpp>
#include <eu07/scene/node/parse_util.hpp>
#include <eu07/scene/runtime/directives.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace eu07::scene::bake {

namespace detail {

[[nodiscard]] inline bool parseTrainsetHeader(
    const DirectiveBlock& block,
    runtime::RuntimeTrainset& trainset) {
    TokenStream stream(block.tokens);
    if (stream.empty() || !isKeyword(stream.peek().value, "trainset")) {
        return false;
    }
    stream.consume();

    std::vector<SourceToken> raw;
    std::string name;
    std::string track;
    double offset = 0.0;
    double velocity = 0.0;
    if (!node::io::takeString(stream, raw, name) || !node::io::takeString(stream, raw, track) ||
        !node::io::takeDouble(stream, raw, offset) ||
        !node::io::takeDouble(stream, raw, velocity)) {
        return false;
    }

    trainset.name = std::move(name);
    trainset.track = std::move(track);
    trainset.offset = static_cast<float>(offset);
    trainset.velocity = static_cast<float>(velocity);
    return true;
}

} // namespace detail

[[nodiscard]] inline std::vector<runtime::RuntimeTrainset> bakeTrainsets(
    const SceneDocument& document,
    const std::vector<runtime::RuntimeDynamicObject>& dynamics) {
    std::vector<runtime::RuntimeTrainset> trainsets;
    trainsets.reserve(document.trainset.size());

    for (std::size_t trainsetIdx = 0; trainsetIdx < document.trainset.size(); ++trainsetIdx) {
        runtime::RuntimeTrainset trainset;
        if (!detail::parseTrainsetHeader(document.trainset[trainsetIdx], trainset)) {
            continue;
        }

        for (std::size_t dynIdx = 0; dynIdx < dynamics.size(); ++dynIdx) {
            const runtime::RuntimeDynamicObject& vehicle = dynamics[dynIdx];
            if (vehicle.trainsetIndex != trainsetIdx) {
                continue;
            }
            trainset.vehicleIndices.push_back(dynIdx);
            trainset.couplings.push_back(vehicle.coupling);
            if (trainset.driverIndex == static_cast<std::size_t>(-1) &&
                !vehicle.driverType.empty() && vehicle.driverType != "0") {
                trainset.driverIndex = trainset.vehicleIndices.size() - 1;
            }
        }

        trainsets.push_back(std::move(trainset));
    }

    return trainsets;
}

} // namespace eu07::scene::bake
