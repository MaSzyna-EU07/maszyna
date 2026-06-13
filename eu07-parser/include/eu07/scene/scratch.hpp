#pragma once

#include <eu07/scene/node/types.hpp>

#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace eu07::scene {

struct SceneScratchpad {
    struct LocationState {
        std::vector<Vec3> originStack;
        std::vector<Vec3> scaleStack;
        Vec3 rotation{};
    } location;

    struct GroupState {
        std::vector<std::size_t> activeStack;
        std::size_t nextHandle = 1;
    } group;

    struct TrainsetState {
        bool isOpen = false;
        std::optional<std::size_t> docIndex;
        std::string name;
        std::string track;
        double offset = 0.0;
        double velocity = 0.0;
        std::vector<std::pair<std::string, std::string>> assignment;
    } trainset;

    [[nodiscard]] Vec3 currentOrigin() const noexcept {
        if (location.originStack.empty()) {
            return {};
        }
        return location.originStack.back();
    }

    void pushOrigin(const Vec3 offset) {
        const Vec3 parent = currentOrigin();
        location.originStack.push_back(
            {parent.x + offset.x, parent.y + offset.y, parent.z + offset.z});
    }

    void popOrigin() noexcept {
        if (!location.originStack.empty()) {
            location.originStack.pop_back();
        }
    }

    [[nodiscard]] Vec3 currentScale() const noexcept {
        if (location.scaleStack.empty()) {
            return {1.0, 1.0, 1.0};
        }
        return location.scaleStack.back();
    }

    void pushScale(const Vec3 factor) {
        const Vec3 parent = currentScale();
        location.scaleStack.push_back(
            {parent.x * factor.x, parent.y * factor.y, parent.z * factor.z});
    }

    void popScale() noexcept {
        if (!location.scaleStack.empty()) {
            location.scaleStack.pop_back();
        }
    }

    void setRotation(const Vec3 rotation) noexcept {
        location.rotation = rotation;
    }

    void openGroup() {
        if (group.activeStack.empty()) {
            group.activeStack.push_back(group.nextHandle++);
            return;
        }
        group.activeStack.push_back(group.activeStack.back());
    }

    void closeGroup() noexcept {
        if (!group.activeStack.empty()) {
            group.activeStack.pop_back();
        }
    }

    [[nodiscard]] std::optional<std::size_t> activeGroupHandle() const noexcept {
        if (group.activeStack.empty()) {
            return std::nullopt;
        }
        return group.activeStack.back();
    }

    void openTrainset() {
        trainset = TrainsetState{};
        trainset.isOpen = true;
    }

    void closeTrainset() noexcept {
        trainset.isOpen = false;
        trainset.docIndex.reset();
    }
};

} // namespace eu07::scene
