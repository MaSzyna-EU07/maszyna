#pragma once

// scene/scene.h — scratch_data (kontekst parsowania, nie persystowany 1:1)

#include <eu07/scene/node/types.hpp>

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

namespace eu07::scene::runtime {

struct RuntimeScratchpad {
    struct LocationState {
        std::vector<Vec3> offsetStack;
        std::vector<Vec3> scaleStack;
        Vec3 rotation{};
    } location;

    struct TrainsetState {
        bool isOpen = false;
        std::string name;
        std::string track;
        float offset = 0.f;
        float velocity = 0.f;
        std::vector<std::size_t> vehicleIndices;
        std::vector<int> couplings;
        std::size_t driverIndex = static_cast<std::size_t>(-1);
        std::unordered_map<std::string, std::string> assignment;
    } trainset;

    std::string scenarioName;
    bool initialized = false;
    bool timeInitialized = false;
};

} // namespace eu07::scene::runtime
