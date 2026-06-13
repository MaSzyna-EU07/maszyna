#pragma once

// Runtime → strumień SourceToken (jak po eu07::parseFile) z odtworzonym stosem origin/scale/rotate.

#include <eu07/parser.hpp>
#include <eu07/scene/bake/module.hpp>
#include <eu07/scene/runtime/nodes.hpp>
#include <eu07/scene/runtime/transform.hpp>

#include <cmath>
#include <filesystem>
#include <format>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

namespace eu07::scene::binary::detokenize {

[[nodiscard]] inline std::string formatDouble(const double value) {
    std::string text = std::format("{:.4f}", value);
    if (const std::size_t dot = text.find('.'); dot != std::string::npos) {
        while (text.size() > dot + 1 && text.back() == '0') {
            text.pop_back();
        }
        if (text.back() == '.') {
            text.pop_back();
        }
    }
    return text;
}

class TokenWriter {
public:
    std::vector<SourceToken> tokens;

    void newline() { ++line_; }

    void push(std::string value) { tokens.push_back(SourceToken{std::move(value), line_}); }

    void push(std::string_view value) { push(std::string(value)); }

    void push(const char* value) { push(std::string_view(value)); }

    void push(const double value) { push(formatDouble(value)); }

    void push(const float value) { push(static_cast<double>(value)); }

    void push(const int value) { push(std::format("{}", value)); }

    void pushVec3(const runtime::Vec3& v) {
        push(v.x);
        push(v.y);
        push(v.z);
    }

private:
    std::size_t line_ = 0;
};

[[nodiscard]] inline std::string includeTextPath(const bake::ModuleInclude& include) {
    if (!include.sourcePath.empty()) {
        return include.sourcePath;
    }
    std::filesystem::path path{include.binaryPath};
    if (path.extension() == ".eu7") {
        path.replace_extension(".inc");
    }
    return path.generic_string();
}

[[nodiscard]] inline double rangeMaxFromNode(const runtime::BasicNode& node) {
    if (node.rangeSquaredMax >= std::numeric_limits<double>::max() * 0.5) {
        return -1.0;
    }
    return std::sqrt(node.rangeSquaredMax);
}

[[nodiscard]] inline double rangeMinFromNode(const runtime::BasicNode& node) {
    return std::sqrt(node.rangeSquaredMin);
}

[[nodiscard]] inline std::string nodeDisplayName(const runtime::BasicNode& node) {
    return node.name.empty() ? "none" : node.name;
}

inline void pushNodeHeader(TokenWriter& w, const runtime::BasicNode& node) {
    w.push("node");
    w.push(rangeMaxFromNode(node));
    w.push(rangeMinFromNode(node));
    w.push(nodeDisplayName(node));
}

struct TransformEmitState {
    std::vector<runtime::Vec3> originStack;
    std::vector<runtime::Vec3> scaleStack;
    runtime::Vec3 rotation{};
    std::size_t groupDepth = 0;
};

[[nodiscard]] inline runtime::Vec3 originPushDelta(
    const std::vector<runtime::Vec3>& stack,
    const std::size_t index) {
    const runtime::Vec3& cumulative = stack[index];
    if (index == 0) {
        return cumulative;
    }
    const runtime::Vec3& parent = stack[index - 1];
    return {cumulative.x - parent.x, cumulative.y - parent.y, cumulative.z - parent.z};
}

[[nodiscard]] inline runtime::Vec3 scalePushFactor(
    const std::vector<runtime::Vec3>& stack,
    const std::size_t index) {
    const runtime::Vec3& cumulative = stack[index];
    const runtime::Vec3 parent = index == 0 ? runtime::Vec3{1.0, 1.0, 1.0} : stack[index - 1];
    return {
        parent.x != 0.0 ? cumulative.x / parent.x : cumulative.x,
        parent.y != 0.0 ? cumulative.y / parent.y : cumulative.y,
        parent.z != 0.0 ? cumulative.z / parent.z : cumulative.z};
}

inline void syncTransformStack(TokenWriter& w, TransformEmitState& state, const runtime::TransformContext& target) {
    while (state.groupDepth > target.groupStackDepth) {
        w.push("endgroup");
        --state.groupDepth;
    }
    while (state.scaleStack.size() > target.scaleStack.size()) {
        w.push("endscale");
        state.scaleStack.pop_back();
    }
    while (state.originStack.size() > target.originStack.size()) {
        w.push("endorigin");
        state.originStack.pop_back();
    }

    while (state.originStack.size() < target.originStack.size()) {
        const std::size_t index = state.originStack.size();
        const runtime::Vec3 delta = originPushDelta(target.originStack, index);
        w.push("origin");
        w.pushVec3(delta);
        state.originStack.push_back(target.originStack[index]);
    }

    while (state.scaleStack.size() < target.scaleStack.size()) {
        const std::size_t index = state.scaleStack.size();
        const runtime::Vec3 factor = scalePushFactor(target.scaleStack, index);
        w.push("scale");
        w.pushVec3(factor);
        state.scaleStack.push_back(target.scaleStack[index]);
    }

    while (state.groupDepth < target.groupStackDepth) {
        w.push("group");
        ++state.groupDepth;
    }

    if (state.rotation.x != target.rotation.x || state.rotation.y != target.rotation.y ||
        state.rotation.z != target.rotation.z) {
        w.push("rotate");
        w.pushVec3(target.rotation);
        state.rotation = target.rotation;
    }
}

inline void emitNodeTransform(TokenWriter& w, TransformEmitState& state, const runtime::BasicNode& node) {
    syncTransformStack(w, state, node.transform);
}

[[nodiscard]] inline std::string_view trackEnvironmentName(const runtime::TrackEnvironment env) {
    switch (env) {
    case runtime::TrackEnvironment::Flat:
        return "flat";
    case runtime::TrackEnvironment::Mountains:
        return "mountains";
    case runtime::TrackEnvironment::Canyon:
        return "canyon";
    case runtime::TrackEnvironment::Tunnel:
        return "tunnel";
    case runtime::TrackEnvironment::Bridge:
        return "bridge";
    case runtime::TrackEnvironment::Bank:
        return "bank";
    default:
        return "flat";
    }
}

[[nodiscard]] inline std::string_view trackKindToken(const runtime::RuntimeTrack& track) {
    if (track.category == runtime::TrackCategory::Road) {
        return "road";
    }
    if (track.category == runtime::TrackCategory::Water) {
        return "river";
    }
    switch (track.trackType) {
    case runtime::TrackType::Switch:
        return "switch";
    case runtime::TrackType::Cross:
        return "cross";
    case runtime::TrackType::Table:
        return "turn";
    default:
        return "normal";
    }
}

inline void pushTrackCore(TokenWriter& w, const runtime::RuntimeTrack& track) {
    w.push(track.length);
    w.push(track.trackWidth);
    w.push(track.friction);
    w.push(track.soundDistance);
    w.push(track.qualityFlag);
    w.push(track.damageFlag);
    w.push(trackEnvironmentName(track.environment));
    if (track.visibility) {
        w.push("vis");
        w.push(track.visibility->material1);
        w.push(track.visibility->texLength);
        w.push(track.visibility->material2);
        w.push(track.visibility->texHeight1);
        w.push(track.visibility->texWidth);
        w.push(track.visibility->texSlope);
    } else {
        w.push("unvis");
    }
}

inline void pushTrackSegments(
    TokenWriter& w,
    const runtime::RuntimeTrack& track,
    const runtime::TransformContext& transform) {
    for (const runtime::SegmentPath& seg : track.paths) {
        w.pushVec3(runtime::subtractOriginOffset(seg.pStart, transform));
        w.push(seg.rollStart);
        w.pushVec3(seg.cpOut);
        w.pushVec3(seg.cpIn);
        w.pushVec3(runtime::subtractOriginOffset(seg.pEnd, transform));
        w.push(seg.rollEnd);
        w.push(seg.radius);
    }
    for (const auto& [key, value] : track.tailKeywords) {
        w.push(key);
        if (key == "sleepermodel") {
            std::size_t start = 0;
            while (start < value.size()) {
                const std::size_t end = value.find(' ', start);
                if (end == std::string::npos) {
                    w.push(value.substr(start));
                    break;
                }
                w.push(value.substr(start, end - start));
                start = end + 1;
            }
        } else {
            w.push(value);
        }
    }
}

inline void emitTrack(TokenWriter& w, TransformEmitState& state, const runtime::RuntimeTrack& track) {
    emitNodeTransform(w, state, track.node);
    pushNodeHeader(w, track.node);
    w.push("track");
    w.push(trackKindToken(track));
    pushTrackCore(w, track);
    pushTrackSegments(w, track, track.node.transform);
    w.push("endtrack");
    w.newline();
}

[[nodiscard]] inline std::string tractionResistivityToken(const runtime::RuntimeTraction& traction) {
    if (traction.resistivityLegacy != 0.0) {
        return formatDouble(traction.resistivityLegacy);
    }
    return formatDouble(traction.resistivityOhmPerM / 0.001);
}

[[nodiscard]] inline std::string tractionMaterialToken(const runtime::RuntimeTraction& traction) {
    if (!traction.materialRaw.empty()) {
        return traction.materialRaw;
    }
    switch (traction.material) {
    case runtime::TractionWireMaterial::Aluminium:
        return "al";
    case runtime::TractionWireMaterial::None:
        return "none";
    default:
        return "cu";
    }
}

inline void emitTraction(TokenWriter& w, TransformEmitState& state, const runtime::RuntimeTraction& traction) {
    emitNodeTransform(w, state, traction.node);
    const runtime::TransformContext& transform = traction.node.transform;
    pushNodeHeader(w, traction.node);
    w.push("traction");
    w.push(traction.powerSupplyName);
    w.push(traction.nominalVoltage);
    w.push(traction.maxCurrent);
    w.push(tractionResistivityToken(traction));
    w.push(tractionMaterialToken(traction));
    w.push(traction.wireThickness);
    w.push(traction.damageFlag);
    w.pushVec3(runtime::subtractOriginOffset(traction.wireP1, transform));
    w.pushVec3(runtime::subtractOriginOffset(traction.wireP2, transform));
    w.pushVec3(runtime::subtractOriginOffset(traction.wireP3, transform));
    w.pushVec3(runtime::subtractOriginOffset(traction.wireP4, transform));
    w.push(traction.minHeight);
    w.push(traction.segmentLength);
    w.push(traction.wireCount);
    w.push(traction.wireOffset);
    w.push(traction.node.visible ? "vis" : "unvis");
    if (traction.parallelName) {
        w.push("parallel");
        w.push(*traction.parallelName);
    }
    w.push("endtraction");
    w.newline();
}

inline void emitPowerSource(
    TokenWriter& w,
    TransformEmitState& state,
    const runtime::RuntimeTractionPowerSource& source) {
    emitNodeTransform(w, state, source.node);
    pushNodeHeader(w, source.node);
    w.push("tractionpowersource");
    w.pushVec3(runtime::inverseTransformPoint(source.position, source.node.transform));
    w.push(source.nominalVoltage);
    w.push(source.voltageFrequency);
    w.push(source.internalResistanceLegacy);
    w.push(source.maxOutputCurrent);
    w.push(source.fastFuseTimeout);
    w.push(source.fastFuseRepetition);
    w.push(source.slowFuseTimeout);
    if (source.modifier == runtime::PowerSourceModifier::Recuperation) {
        w.push("recuperation");
    } else if (source.modifier == runtime::PowerSourceModifier::Section) {
        w.push("section");
    }
    w.push("end");
    w.newline();
}

inline void emitShape(TokenWriter& w, TransformEmitState& state, const runtime::RuntimeShapeNode& shape) {
    emitNodeTransform(w, state, shape.node);
    std::vector<runtime::WorldVertex> localVertices = shape.vertices;
    runtime::inverseTransformShapeVertices(localVertices, shape.node.transform);
    pushNodeHeader(w, shape.node);
    const std::string_view shapeType =
        shape.node.nodeType.empty() ? std::string_view{"triangles"} : std::string_view{shape.node.nodeType};
    w.push(shapeType);
    w.push(shape.materialPath);
    const bool triangleList = shapeType == "triangles";
    for (std::size_t i = 0; i < localVertices.size(); ++i) {
        const runtime::WorldVertex& vert = localVertices[i];
        w.pushVec3(vert.position);
        w.pushVec3(vert.normal);
        w.push(vert.u);
        w.push(vert.v);
        // MaSzyna shape_node::import: po kazdym wierzcholku getToken() — oczekuje
        // "end" po 1. i 2. rogu kazdego trojkata (nie po 3.).
        if (triangleList && i % 3 != 2) {
            w.push("end");
        }
    }
    w.push("endtri");
    w.newline();
}

inline void emitLines(TokenWriter& w, TransformEmitState& state, const runtime::RuntimeLinesNode& lines) {
    emitNodeTransform(w, state, lines.node);
    std::vector<runtime::WorldVertex> localVertices = lines.vertices;
    runtime::inverseTransformShapeVertices(localVertices, lines.node.transform);
    pushNodeHeader(w, lines.node);
    w.push("lines");
    w.push(lines.lighting.diffuse.x * 255.0);
    w.push(lines.lighting.diffuse.y * 255.0);
    w.push(lines.lighting.diffuse.z * 255.0);
    w.push(lines.lineWidth);
    for (std::size_t i = 0; i + 1 < localVertices.size(); i += 2) {
        w.pushVec3(localVertices[i].position);
        w.pushVec3(localVertices[i + 1].position);
    }
    w.push("endlines");
    w.newline();
}

inline void emitModel(TokenWriter& w, TransformEmitState& state, const runtime::RuntimeModelInstance& model) {
    emitNodeTransform(w, state, model.node);
    const runtime::Vec3 localLocation =
        runtime::inverseTransformPoint(model.location, model.node.transform);
    const double localRotationY = model.angles.y - model.node.transform.rotation.y;
    pushNodeHeader(w, model.node);
    w.push("model");
    w.pushVec3(localLocation);
    w.push(localRotationY);
    w.push(model.modelFile);
    w.push(model.textureFile);
    w.push("endmodel");
    w.newline();
}

inline void emitMemcell(TokenWriter& w, TransformEmitState& state, const runtime::RuntimeMemCell& cell) {
    emitNodeTransform(w, state, cell.node);
    pushNodeHeader(w, cell.node);
    w.push("memcell");
    w.pushVec3(runtime::inverseTransformPoint(cell.node.area.center, cell.node.transform));
    w.push(cell.text);
    w.push(cell.value1);
    w.push(cell.value2);
    w.push(cell.trackName.value_or("none"));
    w.push("endmemcell");
    w.newline();
}

[[nodiscard]] inline std::string launcherTimeToken(const runtime::RuntimeEventLauncher& launcher) {
    if (launcher.launchHour != 0 || launcher.launchMinute != 0) {
        return std::format("{}", launcher.launchHour * 100 + launcher.launchMinute);
    }
    if (launcher.deltaTime != 0.0) {
        return formatDouble(-launcher.deltaTime);
    }
    return "0";
}

inline void emitLauncher(
    TokenWriter& w,
    TransformEmitState& state,
    const runtime::RuntimeEventLauncher& launcher) {
    emitNodeTransform(w, state, launcher.node);
    pushNodeHeader(w, launcher.node);
    w.push("eventlauncher");
    w.pushVec3(runtime::inverseTransformPoint(launcher.location, launcher.node.transform));
    w.push(std::sqrt(launcher.radiusSquared));
    w.push(launcher.activationKeyRaw);
    w.push(launcherTimeToken(launcher));
    w.push(launcher.event1Name);
    if (!launcher.event2Name.empty() && launcher.event2Name != "none") {
        w.push(launcher.event2Name);
    }
    if (launcher.condition) {
        w.push("condition");
        w.push(launcher.condition->memcellName);
        w.push(launcher.condition->compareText);
        if (launcher.condition->checkMask & 2) {
            w.push(launcher.condition->compareValue1);
        } else {
            w.push("*");
        }
        if (launcher.condition->checkMask & 4) {
            w.push(launcher.condition->compareValue2);
        } else {
            w.push("*");
        }
    }
    if (launcher.trainTriggered) {
        w.push("traintriggered");
    }
    w.push("endeventlauncher");
    w.newline();
}

inline void emitDynamic(
    TokenWriter& w,
    TransformEmitState& state,
    const runtime::RuntimeDynamicObject& vehicle,
    const bool inTrainset) {
    emitNodeTransform(w, state, vehicle.node);
    pushNodeHeader(w, vehicle.node);
    w.push("dynamic");
    w.push(vehicle.dataFolder);
    w.push(vehicle.skinFile);
    w.push(vehicle.mmdFile);
    if (!inTrainset) {
        w.push(vehicle.trackName);
    }
    w.push(vehicle.offset);
    w.push(vehicle.driverType);
    if (inTrainset) {
        w.push(vehicle.couplingRaw.empty() ? std::to_string(vehicle.coupling) : vehicle.couplingRaw);
    }
    if (!inTrainset) {
        w.push(vehicle.velocity);
    }
    w.push(vehicle.loadCount);
    if (!vehicle.loadType.empty()) {
        w.push(vehicle.loadType);
    }
    if (vehicle.destination) {
        w.push(*vehicle.destination);
    }
    w.push("enddynamic");
    w.newline();
}

inline void emitSound(TokenWriter& w, TransformEmitState& state, const runtime::RuntimeSoundSource& sound) {
    emitNodeTransform(w, state, sound.node);
    pushNodeHeader(w, sound.node);
    w.push("sound");
    w.pushVec3(runtime::inverseTransformPoint(sound.location, sound.node.transform));
    w.push(sound.wavFile);
    w.push("endsound");
    w.newline();
}

[[nodiscard]] inline std::string_view eventTypeName(const runtime::EventType type) {
    switch (type) {
    case runtime::EventType::AddValues:
        return "addvalues";
    case runtime::EventType::UpdateValues:
        return "updatevalues";
    case runtime::EventType::CopyValues:
        return "copyvalues";
    case runtime::EventType::GetValues:
        return "getvalues";
    case runtime::EventType::PutValues:
        return "putvalues";
    case runtime::EventType::Whois:
        return "whois";
    case runtime::EventType::LogValues:
        return "logvalues";
    case runtime::EventType::Multiple:
        return "multiple";
    case runtime::EventType::Switch:
        return "switch";
    case runtime::EventType::TrackVel:
        return "trackvel";
    case runtime::EventType::Sound:
        return "sound";
    case runtime::EventType::Texture:
        return "texture";
    case runtime::EventType::Animation:
        return "animation";
    case runtime::EventType::Lights:
        return "lights";
    case runtime::EventType::Voltage:
        return "voltage";
    case runtime::EventType::Visible:
        return "visible";
    case runtime::EventType::Friction:
        return "friction";
    case runtime::EventType::Message:
        return "message";
    default:
        return "unknown";
    }
}

[[nodiscard]] inline std::string joinTargets(const std::vector<std::string>& targets) {
    std::string joined;
    for (std::size_t i = 0; i < targets.size(); ++i) {
        if (i > 0) {
            joined += '|';
        }
        joined += targets[i];
    }
    return joined.empty() ? "none" : joined;
}

inline void emitEvent(TokenWriter& w, const runtime::RuntimeEvent& event) {
    w.push("event");
    w.push(event.name);
    w.push(eventTypeName(event.type));
    w.push(event.delay);
    w.push(joinTargets(event.targets));
    for (const auto& [key, value] : event.payload) {
        if (!key.empty()) {
            w.push(key);
        }
        if (!value.empty()) {
            w.push(value);
        }
    }
    if (event.delayRandom != 0.0) {
        w.push("randomdelay");
        w.push(event.delayRandom);
    }
    if (event.delayDeparture != 0.0) {
        w.push("departuredelay");
        w.push(event.delayDeparture);
    }
    if (event.passive) {
        w.push("passive");
    }
    w.push("endevent");
    w.newline();
}

inline void emitInclude(TokenWriter& w, const bake::ModuleInclude& include) {
    w.push("include");
    w.push(includeTextPath(include));
    for (const std::string& param : include.parameters) {
        w.push(param);
    }
    w.push("end");
    w.newline();
}

inline void emitTrainsetBlock(
    TokenWriter& w,
    TransformEmitState& state,
    const runtime::RuntimeTrainset& trainset,
    const std::vector<runtime::RuntimeDynamicObject>& dynamics) {
    w.push("trainset");
    w.push(trainset.name);
    w.push(trainset.track);
    w.push(trainset.offset);
    w.push(trainset.velocity);
    for (const std::size_t index : trainset.vehicleIndices) {
        if (index < dynamics.size()) {
            emitDynamic(w, state, dynamics[index], true);
        }
    }
    w.push("endtrainset");
    w.newline();
}

[[nodiscard]] inline std::vector<SourceToken> runtimeModuleToTokens(const bake::RuntimeModule& module) {
    TokenWriter w;
    TransformEmitState transformState;

    for (const bake::ModuleInclude& include : module.includes) {
        emitInclude(w, include);
    }

    for (const runtime::RuntimeTrack& track : module.scene.tracks) {
        emitTrack(w, transformState, track);
    }
    for (const runtime::RuntimeTraction& traction : module.scene.traction) {
        emitTraction(w, transformState, traction);
    }
    for (const runtime::RuntimeTractionPowerSource& source : module.scene.powerSources) {
        emitPowerSource(w, transformState, source);
    }
    for (const runtime::RuntimeShapeNode& shape : module.scene.shapes) {
        emitShape(w, transformState, shape);
    }
    for (const runtime::RuntimeLinesNode& lines : module.scene.lines) {
        emitLines(w, transformState, lines);
    }
    for (const runtime::RuntimeModelInstance& model : module.scene.models) {
        emitModel(w, transformState, model);
    }
    for (const runtime::RuntimeMemCell& cell : module.scene.memcells) {
        emitMemcell(w, transformState, cell);
    }
    for (const runtime::RuntimeEventLauncher& launcher : module.scene.eventLaunchers) {
        emitLauncher(w, transformState, launcher);
    }

    std::vector<bool> emittedInTrainset(module.scene.dynamics.size(), false);
    for (const runtime::RuntimeTrainset& trainset : module.scene.trainsets) {
        for (const std::size_t index : trainset.vehicleIndices) {
            if (index < emittedInTrainset.size()) {
                emittedInTrainset[index] = true;
            }
        }
    }
    for (std::size_t i = 0; i < module.scene.dynamics.size(); ++i) {
        if (!emittedInTrainset[i]) {
            emitDynamic(w, transformState, module.scene.dynamics[i], false);
        }
    }

    for (const runtime::RuntimeSoundSource& sound : module.scene.sounds) {
        emitSound(w, transformState, sound);
    }

    for (const runtime::RuntimeEvent& event : module.scene.events) {
        emitEvent(w, event);
    }

    for (std::uint32_t i = 0; i < module.scene.firstInitCount; ++i) {
        w.push("FirstInit");
        w.newline();
    }

    for (const runtime::RuntimeTrainset& trainset : module.scene.trainsets) {
        emitTrainsetBlock(w, transformState, trainset, module.scene.dynamics);
    }

    return w.tokens;
}

[[nodiscard]] inline bool endsDetokenizedLine(std::string_view token) noexcept {
    return token == "end" || token == "endtrack" || token == "endtraction" ||
           token == "endtri" || token == "endtriangles" || token == "endlines" || token == "endmodel" ||
           token == "endmemcell" || token == "endeventlauncher" || token == "enddynamic" ||
           token == "endsound" || token == "endtrainset" || token == "endevent" ||
           token == "endorigin" || token == "endscale" || token == "endgroup" ||
           token == "FirstInit";
}

[[nodiscard]] inline std::string tokensToText(const std::vector<SourceToken>& tokens) {
    std::string text;
    text.reserve(tokens.size() * 12);
    for (std::size_t i = 0; i < tokens.size(); ++i) {
        if (i > 0) {
            if (tokens[i].sourceLine != tokens[i - 1].sourceLine) {
                text.push_back('\n');
            } else {
                text.push_back(' ');
            }
        }
        text += tokens[i].value;
        if (endsDetokenizedLine(tokens[i].value)) {
            if (i + 1 >= tokens.size() || tokens[i + 1].sourceLine == tokens[i].sourceLine) {
                text.push_back('\n');
            }
        }
    }
    return text;
}

} // namespace eu07::scene::binary::detokenize
