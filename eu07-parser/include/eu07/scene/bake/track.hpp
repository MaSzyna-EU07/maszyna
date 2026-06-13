#pragma once

#include <eu07/scene/node/header.hpp>
#include <eu07/scene/node/track.hpp>
#include <eu07/scene/runtime/nodes.hpp>

#include <format>
#include <limits>
#include <optional>
#include <string>

namespace eu07::scene::bake {

namespace detail {

inline void appendTrackEventKeyword(
    std::vector<std::pair<std::string, std::string>>& tail,
    const std::string_view key,
    const std::optional<std::string>& value) {
    if (value && !value->empty()) {
        tail.emplace_back(std::string(key), *value);
    }
}

inline void appendTrackOptionals(
    std::vector<std::pair<std::string, std::string>>& tail,
    const TrackOptionals& optionals) {
    appendTrackEventKeyword(tail, "event0", optionals.events.occupiedStop);
    appendTrackEventKeyword(tail, "eventall0", optionals.events.anyStop);
    appendTrackEventKeyword(tail, "event1", optionals.events.occupiedEnterP1);
    appendTrackEventKeyword(tail, "eventall1", optionals.events.anyEnterP1);
    appendTrackEventKeyword(tail, "event2", optionals.events.occupiedEnterP2);
    appendTrackEventKeyword(tail, "eventall2", optionals.events.anyEnterP2);

    if (optionals.velocity) {
        tail.emplace_back("velocity", std::format("{}", *optionals.velocity));
    }
    for (const std::string& isolated : optionals.isolated) {
        tail.emplace_back("isolated", isolated);
    }
    if (optionals.overhead) {
        tail.emplace_back("overhead", std::format("{}", *optionals.overhead));
    }
    if (optionals.verticalRadius) {
        tail.emplace_back("vradius", std::format("{}", *optionals.verticalRadius));
    }
    if (optionals.railProfile) {
        tail.emplace_back("railprofile", *optionals.railProfile);
    }
    if (optionals.trackBed) {
        tail.emplace_back("trackbed", *optionals.trackBed);
    }
    if (optionals.frictionOverride) {
        tail.emplace_back("friction", std::format("{}", *optionals.frictionOverride));
    }
    if (optionals.fouling1) {
        tail.emplace_back("fouling1", *optionals.fouling1);
    }
    if (optionals.fouling2) {
        tail.emplace_back("fouling2", *optionals.fouling2);
    }
    if (optionals.sleeperModel) {
        const TrackSleeperModel& sleeper = *optionals.sleeperModel;
        tail.emplace_back(
            "sleepermodel",
            std::format(
                "{} {} {} {} {} {} {}",
                sleeper.spacing,
                sleeper.model,
                sleeper.texture,
                sleeper.offsetLateral,
                sleeper.offsetAlong,
                sleeper.offsetVertical,
                sleeper.bedLowering));
    }

    for (const auto& [key, value] : optionals.extraKeywords) {
        tail.emplace_back(key, value);
    }
}

inline void bakeTrackTailKeywords(std::vector<std::pair<std::string, std::string>>& tail, const TrackOptionals& optionals) {
    tail.clear();
    appendTrackOptionals(tail, optionals);
}

} // namespace detail

[[nodiscard]] inline runtime::BasicNode bakeBasicNode(const NodeHeader& header, const std::string& nodeType) {
    runtime::BasicNode node;
    node.name = (header.name == "none" || header.name.empty()) ? std::string{} : header.name;
    node.nodeType = nodeType;
    node.rangeSquaredMin = header.rangeMin * header.rangeMin;
    node.rangeSquaredMax =
        header.rangeMax >= 0.0 ? header.rangeMax * header.rangeMax
                               : std::numeric_limits<double>::max();
    node.visible = true;
    if (header.groupHandle) {
        node.groupHandle = *header.groupHandle;
        node.groupValid = true;
    }
    node.transform.originStack = header.originStack;
    node.transform.scaleStack = header.scaleStack;
    node.transform.rotation = header.rotation;
    node.transform.groupStackDepth = header.groupStackDepth;
    return node;
}

[[nodiscard]] inline runtime::TrackType mapTrackType(const std::string& kind) {
    if (kind == "normal") {
        return runtime::TrackType::Normal;
    }
    if (kind == "switch") {
        return runtime::TrackType::Switch;
    }
    if (kind == "turn" || kind == "table") {
        return runtime::TrackType::Table;
    }
    if (kind == "cross") {
        return runtime::TrackType::Cross;
    }
    if (kind == "tributary") {
        return runtime::TrackType::Tributary;
    }
    return runtime::TrackType::Unknown;
}

[[nodiscard]] inline runtime::TrackCategory mapTrackCategory(const std::string& kind) {
    if (kind == "road" || kind == "cross") {
        return runtime::TrackCategory::Road;
    }
    if (kind == "river" || kind == "tributary") {
        return runtime::TrackCategory::Water;
    }
    return runtime::TrackCategory::Rail;
}

[[nodiscard]] inline runtime::TrackEnvironment mapEnvironment(const std::string& env) {
    if (env == "flat") {
        return runtime::TrackEnvironment::Flat;
    }
    if (env == "mountains" || env == "mountain") {
        return runtime::TrackEnvironment::Mountains;
    }
    if (env == "canyon") {
        return runtime::TrackEnvironment::Canyon;
    }
    if (env == "tunnel") {
        return runtime::TrackEnvironment::Tunnel;
    }
    if (env == "bridge") {
        return runtime::TrackEnvironment::Bridge;
    }
    if (env == "bank") {
        return runtime::TrackEnvironment::Bank;
    }
    return runtime::TrackEnvironment::Unknown;
}

[[nodiscard]] inline runtime::SegmentPath bakeSegment(
    const TrackBezier& bez,
    const Vec3& originOffset) {
    runtime::SegmentPath seg;
    seg.pStart = {bez.p1.x + originOffset.x, bez.p1.y + originOffset.y, bez.p1.z + originOffset.z};
    seg.rollStart = bez.roll1;
    seg.cpOut = bez.cv1;
    seg.cpIn = bez.cv2;
    seg.pEnd = {bez.p2.x + originOffset.x, bez.p2.y + originOffset.y, bez.p2.z + originOffset.z};
    seg.rollEnd = bez.roll2;
    seg.radius = bez.radius;
    return seg;
}

[[nodiscard]] inline void appendSegments(
    runtime::RuntimeTrack& track,
    const std::vector<TrackBezier>& beziers,
    const Vec3& originOffset) {
    track.paths.reserve(track.paths.size() + beziers.size());
    for (const TrackBezier& bez : beziers) {
        track.paths.push_back(bakeSegment(bez, originOffset));
    }
}

[[nodiscard]] inline runtime::RuntimeTrack bakeTrackNormal(const ParsedTrackNormal& parsed) {
    runtime::RuntimeTrack track;
    track.node = bakeBasicNode(parsed.header, "track");
    track.trackType = mapTrackType(parsed.trajectoryKind);
    track.category = mapTrackCategory(parsed.trajectoryKind);
    track.length = static_cast<float>(parsed.length);
    track.trackWidth = static_cast<float>(parsed.gauge);
    track.friction = static_cast<float>(parsed.friction);
    track.soundDistance = static_cast<float>(parsed.soundDist);
    track.qualityFlag = parsed.quality;
    track.damageFlag = parsed.damageFlag;
    track.environment = mapEnvironment(parsed.environment);
    if (parsed.visibilityMode == TrackVisibilityMode::Visible && parsed.visibility) {
        runtime::TrackVisibility vis;
        vis.material1 = parsed.visibility->railTexture;
        vis.texLength = static_cast<float>(parsed.visibility->railTexLength);
        vis.material2 = parsed.visibility->ballastTexture;
        vis.texHeight1 = static_cast<float>(parsed.visibility->ballastHeight);
        vis.texWidth = static_cast<float>(parsed.visibility->ballastWidth);
        vis.texSlope = static_cast<float>(parsed.visibility->ballastSlope);
        track.visibility = vis;
    }
    appendSegments(track, parsed.beziers, parsed.header.originOffset);
    detail::bakeTrackTailKeywords(track.tailKeywords, parsed.optionals);
    return track;
}

[[nodiscard]] inline runtime::RuntimeTrack bakeTrackSwitch(const ParsedTrackSwitch& parsed) {
    runtime::RuntimeTrack track;
    track.node = bakeBasicNode(parsed.header, "track");
    track.trackType = runtime::TrackType::Switch;
    track.category = runtime::TrackCategory::Rail;
    track.length = static_cast<float>(parsed.length);
    track.trackWidth = static_cast<float>(parsed.gauge);
    track.friction = static_cast<float>(parsed.friction);
    track.soundDistance = static_cast<float>(parsed.soundDist);
    track.qualityFlag = parsed.quality;
    track.damageFlag = parsed.damageFlag;
    track.environment = mapEnvironment(parsed.environment);
    if (parsed.visibilityMode == TrackVisibilityMode::Visible && parsed.visibility) {
        runtime::TrackVisibility vis;
        vis.material1 = parsed.visibility->mainRailTexture;
        vis.texLength = static_cast<float>(parsed.visibility->mainRailTexLength);
        vis.material2 = parsed.visibility->switchRailTexture;
        vis.texHeight1 = static_cast<float>(parsed.visibility->unusedHeight);
        vis.texWidth = static_cast<float>(parsed.visibility->unusedWidth);
        vis.texSlope = static_cast<float>(parsed.visibility->unusedSlope);
        track.visibility = vis;
    }
    appendSegments(track, parsed.beziers, parsed.header.originOffset);
    detail::bakeTrackTailKeywords(track.tailKeywords, parsed.optionals);
    return track;
}

[[nodiscard]] inline runtime::RuntimeTrack bakeTrackRoad(const ParsedTrackRoad& parsed) {
    runtime::RuntimeTrack track;
    track.node = bakeBasicNode(parsed.header, "track");
    track.trackType = runtime::TrackType::Normal;
    track.category = runtime::TrackCategory::Road;
    track.length = static_cast<float>(parsed.length);
    track.trackWidth = static_cast<float>(parsed.roadWidth);
    track.friction = static_cast<float>(parsed.friction);
    track.soundDistance = static_cast<float>(parsed.soundDist);
    track.qualityFlag = parsed.quality;
    track.damageFlag = parsed.damageFlag;
    track.environment = mapEnvironment(parsed.environment);
    if (parsed.visibilityMode == TrackVisibilityMode::Visible && parsed.visibility) {
        runtime::TrackVisibility vis;
        vis.material1 = parsed.visibility->surfaceTexture;
        vis.texLength = static_cast<float>(parsed.visibility->surfaceTexLength);
        vis.material2 = parsed.visibility->shoulderTexture;
        vis.texHeight1 = static_cast<float>(parsed.visibility->shoulderHeight);
        vis.texWidth = static_cast<float>(parsed.visibility->shoulderWidth);
        vis.texSlope = static_cast<float>(parsed.visibility->shoulderSlope);
        track.visibility = vis;
    }
    appendSegments(track, parsed.beziers, parsed.header.originOffset);
    detail::bakeTrackTailKeywords(track.tailKeywords, parsed.optionals);
    return track;
}

[[nodiscard]] inline runtime::RuntimeTrack bakeTrackCross(const ParsedTrackCross& parsed) {
    runtime::RuntimeTrack track;
    track.node = bakeBasicNode(parsed.header, "track");
    track.trackType = runtime::TrackType::Cross;
    track.category = runtime::TrackCategory::Road;
    track.length = static_cast<float>(parsed.length);
    track.trackWidth = static_cast<float>(parsed.roadWidth);
    track.friction = static_cast<float>(parsed.friction);
    track.soundDistance = static_cast<float>(parsed.soundDist);
    track.qualityFlag = parsed.quality;
    track.damageFlag = parsed.damageFlag;
    track.environment = mapEnvironment(parsed.environment);
    if (parsed.visibilityMode == TrackVisibilityMode::Visible && parsed.visibility) {
        runtime::TrackVisibility vis;
        vis.material1 = parsed.visibility->surfaceTexture;
        vis.texLength = static_cast<float>(parsed.visibility->surfaceTexLength);
        vis.material2 = parsed.visibility->shoulderTexture;
        vis.texHeight1 = static_cast<float>(parsed.visibility->shoulderHeight);
        vis.texWidth = static_cast<float>(parsed.visibility->shoulderWidth);
        vis.texSlope = static_cast<float>(parsed.visibility->shoulderSlope);
        track.visibility = vis;
    }
    appendSegments(track, parsed.beziers, parsed.header.originOffset);
    detail::bakeTrackTailKeywords(track.tailKeywords, parsed.optionals);
    return track;
}

[[nodiscard]] inline runtime::RuntimeTrack bakeTrackOther(const ParsedTrackOther& parsed) {
    runtime::RuntimeTrack track;
    track.node = bakeBasicNode(parsed.header, "track");
    track.trackType = mapTrackType(parsed.trajectoryKind);
    track.category = mapTrackCategory(parsed.trajectoryKind);
    track.length = static_cast<float>(parsed.length);
    track.trackWidth = static_cast<float>(parsed.width);
    track.friction = static_cast<float>(parsed.friction);
    track.soundDistance = static_cast<float>(parsed.soundDist);
    track.qualityFlag = parsed.quality;
    track.damageFlag = parsed.damageFlag;
    track.environment = mapEnvironment(parsed.environment);
    if (parsed.visibilityMode == TrackVisibilityMode::Visible) {
        if (parsed.riverVisibility) {
            runtime::TrackVisibility vis;
            vis.material1 = parsed.riverVisibility->waterTexture;
            vis.texLength = static_cast<float>(parsed.riverVisibility->waterTexLength);
            vis.material2 = parsed.riverVisibility->bankTexture;
            vis.texHeight1 = static_cast<float>(parsed.riverVisibility->bankHeight);
            vis.texWidth = static_cast<float>(parsed.riverVisibility->leftBankWidth);
            vis.texSlope = static_cast<float>(parsed.riverVisibility->rightBankWidth);
            track.visibility = vis;
        } else if (parsed.railVisibility) {
            runtime::TrackVisibility vis;
            vis.material1 = parsed.railVisibility->railTexture;
            vis.texLength = static_cast<float>(parsed.railVisibility->railTexLength);
            vis.material2 = parsed.railVisibility->ballastTexture;
            vis.texHeight1 = static_cast<float>(parsed.railVisibility->ballastHeight);
            vis.texWidth = static_cast<float>(parsed.railVisibility->ballastWidth);
            vis.texSlope = static_cast<float>(parsed.railVisibility->ballastSlope);
            track.visibility = vis;
        }
    }
    appendSegments(track, parsed.beziers, parsed.header.originOffset);
    detail::bakeTrackTailKeywords(track.tailKeywords, parsed.optionals);
    return track;
}

} // namespace eu07::scene::bake
