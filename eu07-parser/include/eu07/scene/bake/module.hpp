#pragma once

// Jeden plik tekstowy (.scn/.inc) → jeden moduł runtime (RuntimeModule).

#include <eu07/scene/bake/dynamic.hpp>
#include <eu07/scene/bake/event.hpp>
#include <eu07/scene/bake/eventlauncher.hpp>
#include <eu07/scene/bake/lines.hpp>
#include <eu07/scene/bake/memcell.hpp>
#include <eu07/scene/bake/mesh.hpp>
#include <eu07/scene/bake/model.hpp>
#include <eu07/scene/bake/sound.hpp>
#include <eu07/scene/bake/track.hpp>
#include <eu07/scene/bake/traction.hpp>
#include <eu07/scene/bake/traction_power.hpp>
#include <eu07/scene/bake/trainset.hpp>
#include <eu07/scene/document.hpp>
#include <eu07/scene/include_placement.hpp>
#include <eu07/scene/include_types.hpp>
#include <eu07/scene/runtime/scene.hpp>

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace eu07::scene::bake {

struct ModuleInclude {
    std::uint32_t sourceLine = 0;
    std::string sourcePath;
    std::string binaryPath;
    std::vector<std::string> parameters;
    runtime::TransformContext siteTransform;
};

struct RuntimeModule {
    std::vector<ModuleInclude> includes;
    runtime::RuntimeScene scene;
    IncludePlacement includePlacement;
};

struct BakeModuleOptions {
    // Root + PACK: modele i tak pochodza z composePackModels.
    bool skipLocalModels = false;
};

[[nodiscard]] inline std::string textPathToBinaryPath(const std::string& textPath) {
    std::filesystem::path path{textPath};
    path.replace_extension(".eu7");
    return path.generic_string();
}

[[nodiscard]] inline RuntimeModule bakeModule(
    const SceneDocument& document,
    const BakeModuleOptions& options = {}) {
    RuntimeModule module;

    module.includes.reserve(document.include.size());
    for (const ParsedInclude& inc : document.include) {
        ModuleInclude ref;
        ref.sourceLine = static_cast<std::uint32_t>(inc.line);
        ref.sourcePath = inc.file;
        ref.binaryPath = textPathToBinaryPath(inc.file);
        ref.parameters = inc.parameters;
        ref.siteTransform = inc.siteTransform;
        module.includes.push_back(std::move(ref));
    }

    auto& scene = module.scene;

    scene.tracks.reserve(countTrackInstances(document));
    for (const ParsedTrackNormal& parsed : document.nodeTrackNormal) {
        scene.tracks.push_back(bakeTrackNormal(parsed));
    }
    for (const ParsedTrackSwitch& parsed : document.nodeTrackSwitch) {
        scene.tracks.push_back(bakeTrackSwitch(parsed));
    }
    for (const ParsedTrackRoad& parsed : document.nodeTrackRoad) {
        scene.tracks.push_back(bakeTrackRoad(parsed));
    }
    for (const ParsedTrackCross& parsed : document.nodeTrackCross) {
        scene.tracks.push_back(bakeTrackCross(parsed));
    }
    for (const ParsedTrackOther& parsed : document.nodeTrackOther) {
        scene.tracks.push_back(bakeTrackOther(parsed));
    }

    scene.traction.reserve(document.nodeTraction.size());
    for (const ParsedNodeTraction& parsed : document.nodeTraction) {
        scene.traction.push_back(bakeTraction(parsed));
    }

    scene.powerSources.reserve(document.nodeTractionPower.size());
    for (const ParsedNodeTractionPowerSource& parsed : document.nodeTractionPower) {
        scene.powerSources.push_back(bakeTractionPower(parsed));
    }

    const std::size_t shapeCount =
        document.nodeTriangles.size() + document.nodeTriangleStrip.size() +
        document.nodeTriangleFan.size();
    scene.shapes.reserve(shapeCount);
    for (const ParsedNodeTriangles& parsed : document.nodeTriangles) {
        scene.shapes.push_back(bakeTriangles(parsed));
    }
    for (const ParsedNodeTriangleStrip& parsed : document.nodeTriangleStrip) {
        scene.shapes.push_back(bakeTriangleStrip(parsed));
    }
    for (const ParsedNodeTriangleFan& parsed : document.nodeTriangleFan) {
        scene.shapes.push_back(bakeTriangleFan(parsed));
    }

    const std::size_t lineCount =
        document.nodeLines.size() + document.nodeLineStrip.size() + document.nodeLineLoop.size();
    scene.lines.reserve(lineCount);
    for (const ParsedNodeLines& parsed : document.nodeLines) {
        scene.lines.push_back(bakeLines(parsed));
    }
    for (const ParsedNodeLineStrip& parsed : document.nodeLineStrip) {
        scene.lines.push_back(bakeLineStrip(parsed));
    }
    for (const ParsedNodeLineLoop& parsed : document.nodeLineLoop) {
        scene.lines.push_back(bakeLineLoop(parsed));
    }

    if (!options.skipLocalModels) {
        scene.models.reserve(document.nodeModel.size());
        for (const ParsedNodeModel& parsed : document.nodeModel) {
            scene.models.push_back(bakeModel(parsed));
        }
    }

    scene.memcells.reserve(document.nodeMemcell.size());
    for (const ParsedNodeMemcell& parsed : document.nodeMemcell) {
        scene.memcells.push_back(bakeMemcell(parsed));
    }

    scene.eventLaunchers.reserve(document.nodeEventlauncher.size());
    for (const ParsedNodeEventlauncher& parsed : document.nodeEventlauncher) {
        scene.eventLaunchers.push_back(bakeEventlauncher(parsed));
    }

    scene.dynamics.reserve(document.nodeDynamic.size());
    for (const ParsedNodeDynamic& parsed : document.nodeDynamic) {
        scene.dynamics.push_back(bakeDynamic(parsed));
    }

    scene.sounds.reserve(document.nodeSound.size());
    for (const ParsedNodeSound& parsed : document.nodeSound) {
        scene.sounds.push_back(bakeSound(parsed));
    }

    scene.trainsets = bakeTrainsets(document, scene.dynamics);
    scene.events = bakeEvents(document);
    scene.firstInitCount = static_cast<std::uint32_t>(document.firstInit.size());

    if (const std::optional<IncludePlacement> placement = extractIncludePlacement(document)) {
        module.includePlacement = *placement;
    }

    return module;
}

} // namespace eu07::scene::bake
