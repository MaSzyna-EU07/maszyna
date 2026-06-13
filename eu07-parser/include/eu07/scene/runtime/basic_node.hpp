#pragma once

// scenenode.h — basic_node (wspolna baza TTrack, TAnimModel, TMemCell, …)

#include <eu07/scene/runtime/types.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace eu07::scene::runtime {

// Snapshot stosu origin/scale/rotate z momentu parsowania node'a (do detokenizacji).
struct TransformContext {
    std::vector<Vec3> originStack;
    std::vector<Vec3> scaleStack;
    Vec3 rotation{};
    std::size_t groupStackDepth = 0;
};

struct BasicNode {
    BoundingArea area;
    double rangeSquaredMin = 0.0;
    double rangeSquaredMax = 0.0;
    bool visible = true;
    std::string name;
    std::string nodeType;
    std::size_t groupHandle = 0;
    bool groupValid = false;
    TransformContext transform;
    std::string uuid;
    bool dirty = false;
};

[[nodiscard]] inline BasicNode makeBasicNode(const NodeData& data) {
    BasicNode node;
    node.name = (data.name == "none") ? std::string{} : data.name;
    node.nodeType = data.type;
    node.rangeSquaredMin = data.rangeMin * data.rangeMin;
    node.rangeSquaredMax =
        data.rangeMax >= 0.0 ? data.rangeMax * data.rangeMax : std::numeric_limits<double>::max();
    return node;
}

} // namespace eu07::scene::runtime
