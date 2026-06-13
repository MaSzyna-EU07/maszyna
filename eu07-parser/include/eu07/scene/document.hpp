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

} // namespace eu07::scene
