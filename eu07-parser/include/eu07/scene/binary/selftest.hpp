#pragma once

// Self-test: RuntimeModule → tokeny → processScene → bake → porównanie liczników.

#include <eu07/scene/bake/module.hpp>
#include <eu07/scene/binary/detokenize.hpp>
#include <eu07/scene/processor.hpp>

#include <format>
#include <string>
#include <vector>

namespace eu07::scene::binary::selftest {

struct ModuleSnapshot {
    std::size_t includes = 0;
    std::size_t tracks = 0;
    std::size_t traction = 0;
    std::size_t powerSources = 0;
    std::size_t shapes = 0;
    std::size_t lines = 0;
    std::size_t models = 0;
    std::size_t memcells = 0;
    std::size_t eventLaunchers = 0;
    std::size_t dynamics = 0;
    std::size_t sounds = 0;
    std::size_t trainsets = 0;
    std::size_t events = 0;
    std::uint32_t firstInit = 0;
    std::size_t meshVertices = 0;
    std::size_t lineVertices = 0;
};

[[nodiscard]] inline ModuleSnapshot snapshotModule(const bake::RuntimeModule& module) {
    ModuleSnapshot snap;
    snap.includes = module.includes.size();
    snap.tracks = module.scene.tracks.size();
    snap.traction = module.scene.traction.size();
    snap.powerSources = module.scene.powerSources.size();
    snap.shapes = module.scene.shapes.size();
    snap.lines = module.scene.lines.size();
    snap.models = module.scene.models.size();
    snap.memcells = module.scene.memcells.size();
    snap.eventLaunchers = module.scene.eventLaunchers.size();
    snap.dynamics = module.scene.dynamics.size();
    snap.sounds = module.scene.sounds.size();
    snap.trainsets = module.scene.trainsets.size();
    snap.events = module.scene.events.size();
    snap.firstInit = module.scene.firstInitCount;
    for (const runtime::RuntimeShapeNode& shape : module.scene.shapes) {
        snap.meshVertices += shape.vertices.size();
    }
    for (const runtime::RuntimeLinesNode& line : module.scene.lines) {
        snap.lineVertices += line.vertices.size();
    }
    return snap;
}

[[nodiscard]] inline std::string diffSnapshots(
    const ModuleSnapshot& expected,
    const ModuleSnapshot& actual) {
    std::string report;
    const auto field = [&](const char* label, const std::size_t a, const std::size_t b) {
        if (a != b) {
            report += std::format("  {}: {} -> {}\n", label, a, b);
        }
    };
    field("include", expected.includes, actual.includes);
    field("track", expected.tracks, actual.tracks);
    field("traction", expected.traction, actual.traction);
    field("power", expected.powerSources, actual.powerSources);
    field("mesh", expected.shapes, actual.shapes);
    field("mesh_vert", expected.meshVertices, actual.meshVertices);
    field("line", expected.lines, actual.lines);
    field("line_vert", expected.lineVertices, actual.lineVertices);
    field("model", expected.models, actual.models);
    field("memcell", expected.memcells, actual.memcells);
    field("launcher", expected.eventLaunchers, actual.eventLaunchers);
    field("dynamic", expected.dynamics, actual.dynamics);
    field("sound", expected.sounds, actual.sounds);
    field("trainset", expected.trainsets, actual.trainsets);
    field("event", expected.events, actual.events);
    field("firstInit", expected.firstInit, actual.firstInit);
    return report;
}

[[nodiscard]] inline bool snapshotsEqual(
    const ModuleSnapshot& expected,
    const ModuleSnapshot& actual) {
    return expected.includes == actual.includes && expected.tracks == actual.tracks &&
           expected.traction == actual.traction && expected.powerSources == actual.powerSources &&
           expected.shapes == actual.shapes && expected.meshVertices == actual.meshVertices &&
           expected.lines == actual.lines && expected.lineVertices == actual.lineVertices &&
           expected.models == actual.models && expected.memcells == actual.memcells &&
           expected.eventLaunchers == actual.eventLaunchers &&
           expected.dynamics == actual.dynamics && expected.sounds == actual.sounds &&
           expected.trainsets == actual.trainsets && expected.events == actual.events &&
           expected.firstInit == actual.firstInit;
}

struct RoundTripResult {
    bool passed = false;
    ModuleSnapshot original;
    ModuleSnapshot roundtrip;
    std::size_t tokenCount = 0;
    std::size_t unknownTokens = 0;
    std::string diff;
    std::vector<SourceToken> tokens;
};

[[nodiscard]] inline RoundTripResult runRoundTrip(const bake::RuntimeModule& module) {
    RoundTripResult result;
    result.original = snapshotModule(module);
    result.tokens = detokenize::runtimeModuleToTokens(module);
    result.tokenCount = result.tokens.size();

    eu07::ParseResult parsed;
    parsed.tokens = result.tokens;

    SceneProcessOptions options;
    options.expandIncludes = false;
    const SceneProcessResult processed = processScene(parsed, {}, options);

    result.unknownTokens = processed.document.unknown.size();
    const bake::RuntimeModule rebaked = bake::bakeModule(processed.document);
    result.roundtrip = snapshotModule(rebaked);

    result.diff = diffSnapshots(result.original, result.roundtrip);
    result.passed = snapshotsEqual(result.original, result.roundtrip) && result.unknownTokens == 0;
    return result;
}

} // namespace eu07::scene::binary::selftest
