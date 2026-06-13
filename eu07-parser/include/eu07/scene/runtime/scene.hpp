#pragma once

// Zbiorczy dokument runtime po bake.

#include <eu07/scene/runtime/directives.hpp>
#include <eu07/scene/runtime/scratch.hpp>

#include <cstdint>
#include <vector>

namespace eu07::scene::runtime {

// Stare szkice tagow — nieuzywane (runtime uzywa chunkow czteroliterowych w runtime_module.hpp).
enum class SectionTag : std::uint32_t {
    FileHeader = 0x45443742, // 'ED7B' placeholder
    Transform = 1,
    Trainset = 2,
    Event = 3,
    NodeModel = 10,
    NodeShape = 11,
    NodeLines = 12,
    NodeTrack = 13,
    NodeTraction = 14,
    NodePowerSource = 15,
    NodeMemCell = 16,
    NodeEventLauncher = 17,
    NodeDynamic = 18,
    NodeSound = 19,
};

struct RuntimeScene {
    RuntimeScratchpad scratch;

    std::vector<TransformDirective> transforms;
    std::vector<RuntimeTrainset> trainsets;
    std::vector<RuntimeEvent> events;

    std::vector<RuntimeModelInstance> models;
    std::vector<RuntimeShapeNode> shapes;
    std::vector<RuntimeLinesNode> lines;
    std::vector<RuntimeTrack> tracks;
    std::vector<RuntimeTraction> traction;
    std::vector<RuntimeTractionPowerSource> powerSources;
    std::vector<RuntimeMemCell> memcells;
    std::vector<RuntimeEventLauncher> eventLaunchers;
    std::vector<RuntimeDynamicObject> dynamics;
    std::vector<RuntimeSoundSource> sounds;

    // FirstInit — marker inicjalizacji scenariusza (bez payloadu)
    std::uint32_t firstInitCount = 0;
};

} // namespace eu07::scene::runtime
