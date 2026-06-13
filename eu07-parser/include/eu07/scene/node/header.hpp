#pragma once

// https://wiki.eu07.pl/index.php?title=Obiekt_node

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include <eu07/scene/node/shared.hpp>
#include <eu07/scene/node/types.hpp>

namespace eu07::scene {

struct NodeHeader {
    std::size_t line = 0;
    double rangeMax;
    double rangeMin;
    std::string name;
    Vec3 originOffset{};
    Vec3 scaleFactor{1.0, 1.0, 1.0};
    Vec3 rotation{};
    std::vector<Vec3> originStack;
    std::vector<Vec3> scaleStack;
    std::size_t groupStackDepth = 0;
    std::optional<std::size_t> groupHandle;
    std::optional<std::size_t> trainsetIndex;
    std::optional<TrainsetContext> trainset;
};

} // namespace eu07::scene
