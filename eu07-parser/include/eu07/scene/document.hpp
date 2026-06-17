#pragma once

#include <eu07/parser.hpp>
#include <eu07/scene/include_types.hpp>
#include <eu07/scene/node/dynamic.hpp>
#include <eu07/scene/node/eventlauncher.hpp>
#include <eu07/scene/node/line_loop.hpp>
#include <eu07/scene/node/line_strip.hpp>
#include <eu07/scene/node/lines.hpp>
#include <eu07/scene/node/memcell.hpp>
#include <eu07/scene/node/model.hpp>
#include <eu07/scene/node/sound.hpp>
#include <eu07/scene/node/track.hpp>
#include <eu07/scene/node/traction.hpp>
#include <eu07/scene/node/traction_power.hpp>
#include <eu07/scene/node/triangle_fan.hpp>
#include <eu07/scene/node/triangle_strip.hpp>
#include <eu07/scene/node/triangles.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace eu07::scene {

struct DirectiveBlock {
    std::size_t line = 0;
    std::vector<SourceToken> tokens;
};

struct UnknownEntry {
    std::size_t line = 0;
    std::string token;
};

struct SceneDocument {
    std::vector<DirectiveBlock> atmo;
    std::vector<DirectiveBlock> sky;
    std::vector<DirectiveBlock> time;
    std::vector<DirectiveBlock> firstInit;
    std::vector<DirectiveBlock> trainset;
    std::vector<DirectiveBlock> event;
    std::vector<ParsedInclude> include;
    std::vector<DirectiveBlock> camera;
    std::vector<DirectiveBlock> config;
    std::vector<DirectiveBlock> lua;

    std::vector<ParsedNodeDynamic> nodeDynamic;
    std::vector<ParsedNodeModel> nodeModel;
    std::vector<ParsedTrackNormal> nodeTrackNormal;
    std::vector<ParsedTrackSwitch> nodeTrackSwitch;
    std::vector<ParsedTrackRoad> nodeTrackRoad;
    std::vector<ParsedTrackCross> nodeTrackCross;
    std::vector<ParsedTrackOther> nodeTrackOther;
    std::vector<ParsedNodeTraction> nodeTraction;
    std::vector<ParsedNodeTractionPowerSource> nodeTractionPower;
    std::vector<ParsedNodeTriangles> nodeTriangles;
    std::vector<ParsedNodeTriangleStrip> nodeTriangleStrip;
    std::vector<ParsedNodeTriangleFan> nodeTriangleFan;
    std::vector<ParsedNodeLines> nodeLines;
    std::vector<ParsedNodeLineStrip> nodeLineStrip;
    std::vector<ParsedNodeLineLoop> nodeLineLoop;
    std::vector<ParsedNodeMemcell> nodeMemcell;
    std::vector<ParsedNodeEventlauncher> nodeEventlauncher;
    std::vector<ParsedNodeSound> nodeSound;

    std::vector<DirectiveBlock> origin;
    std::vector<DirectiveBlock> scale;
    std::vector<DirectiveBlock> rotate;
    std::vector<DirectiveBlock> group;
    std::vector<DirectiveBlock> isolated;
    std::vector<DirectiveBlock> area;
    std::vector<DirectiveBlock> terrain;

    std::vector<DirectiveBlock> description;
    std::vector<DirectiveBlock> light;
    std::vector<DirectiveBlock> test;

    std::vector<UnknownEntry> unknown;
};

struct SceneProcessResult {
    SceneDocument document;
};

[[nodiscard]] inline std::size_t countTrackInstances(const SceneDocument& doc) noexcept {
    return doc.nodeTrackNormal.size() + doc.nodeTrackSwitch.size() + doc.nodeTrackRoad.size() +
           doc.nodeTrackCross.size() + doc.nodeTrackOther.size();
}

[[nodiscard]] inline std::size_t countNodeInstances(const SceneDocument& doc) noexcept {
    return doc.nodeDynamic.size() + doc.nodeModel.size() + countTrackInstances(doc) +
           doc.nodeTraction.size() + doc.nodeTractionPower.size() + doc.nodeTriangles.size() +
           doc.nodeTriangleStrip.size() +
           doc.nodeTriangleFan.size() + doc.nodeLines.size() + doc.nodeLineStrip.size() +
           doc.nodeLineLoop.size() + doc.nodeMemcell.size() + doc.nodeEventlauncher.size() +
           doc.nodeSound.size();
}

[[nodiscard]] inline std::size_t countDirectiveInstances(const SceneDocument& doc) noexcept {
    return doc.atmo.size() + doc.sky.size() + doc.time.size() + doc.firstInit.size() +
           doc.trainset.size() + doc.event.size() + doc.include.size() + doc.camera.size() +
           doc.config.size() + doc.lua.size() + countNodeInstances(doc) + doc.origin.size() +
           doc.scale.size() + doc.rotate.size() + doc.group.size() + doc.isolated.size() +
           doc.area.size() + doc.terrain.size() + doc.description.size() + doc.light.size() +
           doc.test.size();
}

// Plaski plik flory: same wpisy node model, bez torow/siatek/include.
[[nodiscard]] inline bool isModelOnlyDocument(const SceneDocument& document) noexcept {
    if (document.nodeModel.empty()) {
        return false;
    }
    return document.nodeDynamic.empty() && document.nodeTrackNormal.empty() &&
           document.nodeTrackSwitch.empty() && document.nodeTrackRoad.empty() &&
           document.nodeTrackCross.empty() && document.nodeTrackOther.empty() &&
           document.nodeTraction.empty() && document.nodeTractionPower.empty() &&
           document.nodeTriangles.empty() && document.nodeTriangleStrip.empty() &&
           document.nodeTriangleFan.empty() && document.nodeLines.empty() &&
           document.nodeLineStrip.empty() && document.nodeLineLoop.empty() &&
           document.nodeMemcell.empty() && document.nodeEventlauncher.empty() &&
           document.nodeSound.empty() && document.include.empty() && document.unknown.empty();
}

// Drops parsed node payloads already converted to RuntimeModule / PackFileCacheEntry.
// Keeps include lists and lightweight directives. When release_models is false,
// nodeModel is retained for a pending PACK build (model-only flora child).
inline void releaseHeavySceneParseStorage(SceneDocument& document, bool const release_models) {
    if (release_models) {
        document.nodeModel.clear();
        document.nodeModel.shrink_to_fit();
    }
    document.nodeTriangles.clear();
    document.nodeTriangles.shrink_to_fit();
    document.nodeTriangleStrip.clear();
    document.nodeTriangleStrip.shrink_to_fit();
    document.nodeTriangleFan.clear();
    document.nodeTriangleFan.shrink_to_fit();
    document.nodeLines.clear();
    document.nodeLines.shrink_to_fit();
    document.nodeLineStrip.clear();
    document.nodeLineStrip.shrink_to_fit();
    document.nodeLineLoop.clear();
    document.nodeLineLoop.shrink_to_fit();
    document.nodeTrackNormal.clear();
    document.nodeTrackNormal.shrink_to_fit();
    document.nodeTrackSwitch.clear();
    document.nodeTrackSwitch.shrink_to_fit();
    document.nodeTrackRoad.clear();
    document.nodeTrackRoad.shrink_to_fit();
    document.nodeTrackCross.clear();
    document.nodeTrackCross.shrink_to_fit();
    document.nodeTrackOther.clear();
    document.nodeTrackOther.shrink_to_fit();
    document.nodeTraction.clear();
    document.nodeTraction.shrink_to_fit();
    document.nodeTractionPower.clear();
    document.nodeTractionPower.shrink_to_fit();
    document.nodeMemcell.clear();
    document.nodeMemcell.shrink_to_fit();
    document.nodeEventlauncher.clear();
    document.nodeEventlauncher.shrink_to_fit();
    document.nodeSound.clear();
    document.nodeSound.shrink_to_fit();
    document.nodeDynamic.clear();
    document.nodeDynamic.shrink_to_fit();
}

} // namespace eu07::scene
