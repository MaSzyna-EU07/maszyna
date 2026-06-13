#pragma once

// Dyrektywy scenariusza poza `node`: trainset, event, transform stack.

#include <eu07/scene/runtime/nodes.hpp>

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

namespace eu07::scene::runtime {

// trainset / endtrainset
struct RuntimeTrainset {
    std::string name;
    std::string track;
    float offset = 0.f;
    float velocity = 0.f;
    std::unordered_map<std::string, std::string> assignment;
    std::vector<std::size_t> vehicleIndices;
    std::vector<int> couplings;
    std::size_t driverIndex = static_cast<std::size_t>(-1);
};

// event / endevent — world/Event.h basic_event + podklasy
enum class EventType : std::uint8_t {
    AddValues,
    UpdateValues,
    CopyValues,
    GetValues,
    PutValues,
    Whois,
    LogValues,
    Multiple,
    Switch,
    TrackVel,
    Sound,
    Texture,
    Animation,
    Lights,
    Voltage,
    Visible,
    Friction,
    Message,
    Unknown,
};

struct RuntimeEvent {
    std::string name;
    EventType type = EventType::Unknown;
    double delay = 0.0;
    std::vector<std::string> targets;
    double delayRandom = 0.0;
    double delayDeparture = 0.0;
    bool ignored = false;
    bool passive = false;
    // type-specific payload — union lub osobne structy w przyszlosci
    std::vector<std::pair<std::string, std::string>> payload;
};

enum class TransformOp : std::uint8_t {
    PushOrigin,
    PopOrigin,
    PushScale,
    PopScale,
    SetRotation,
    GroupBegin,
    GroupEnd,
};

struct TransformDirective {
    TransformOp op = TransformOp::PushOrigin;
    Vec3 vector{};
};

} // namespace eu07::scene::runtime
