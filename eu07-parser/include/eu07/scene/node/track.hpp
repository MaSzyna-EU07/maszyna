#pragma once

// https://wiki.eu07.pl/index.php?title=Obiekt_node::track

#include <eu07/scene/cursor.hpp>
#include <eu07/scene/match.hpp>
#include <eu07/scene/node/header.hpp>
#include <eu07/scene/node/parse_util.hpp>
#include <eu07/scene/node/types.hpp>

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace eu07::scene {

enum class TrackVisibilityMode {
    Visible,
    Hidden,
};

struct TrackBezier {
    Vec3 p1;
    double roll1 = 0.0;
    Vec3 cv1;
    Vec3 cv2;
    Vec3 p2;
    double roll2 = 0.0;
    double radius = 0.0;
};

struct TrackSleeperModel {
    double spacing = 0.0;
    std::string model;
    std::string texture;
    double offsetLateral = 0.0;
    double offsetAlong = 0.0;
    double offsetVertical = 0.0;
    double bedLowering = 0.0;
};

struct TrackEvents {
    std::optional<std::string> occupiedStop;
    std::optional<std::string> occupiedEnterP1;
    std::optional<std::string> occupiedEnterP2;
    std::optional<std::string> anyStop;
    std::optional<std::string> anyEnterP1;
    std::optional<std::string> anyEnterP2;
};

struct TrackOptionals {
    std::optional<double> velocity;
    std::optional<double> overhead;
    std::optional<std::string> railProfile;
    std::optional<std::string> trackBed;
    std::optional<double> frictionOverride;
    std::optional<double> verticalRadius;
    std::optional<double> angle1;
    std::optional<double> angle2;
    std::optional<std::string> fouling1;
    std::optional<std::string> fouling2;
    std::optional<TrackSleeperModel> sleeperModel;
    TrackEvents events;
    std::vector<std::string> isolated;
    std::vector<std::pair<std::string, std::string>> extraKeywords;
};

// normal / turn / table — szyny + podsypka
struct NormalRailVisibility {
    std::string railTexture;
    double railTexLength = 0.0;
    std::string ballastTexture;
    double ballastHeight = 0.0;
    double ballastWidth = 0.0;
    double ballastSlope = 0.0;
};

struct ParsedTrackNormal {
    NodeHeader header;
    std::string trajectoryKind;
    double length = 0.0;
    double gauge = 0.0;
    double friction = 0.0;
    double soundDist = 0.0;
    int quality = 0;
    int damageFlag = 0;
    std::string environment;
    TrackVisibilityMode visibilityMode = TrackVisibilityMode::Hidden;
    std::optional<NormalRailVisibility> visibility;
    std::vector<TrackBezier> beziers;
    TrackOptionals optionals;
    std::vector<SourceToken> raw;
};

// switch — szyny toru zasadniczego + toru zwrotnego
struct SwitchRailVisibility {
    std::string mainRailTexture;
    double mainRailTexLength = 0.0;
    std::string switchRailTexture;
    double unusedHeight = 0.0;
    double unusedWidth = 0.0;
    double unusedSlope = 0.0;
};

struct ParsedTrackSwitch {
    NodeHeader header;
    double length = 0.0;
    double gauge = 0.0;
    double friction = 0.0;
    double soundDist = 0.0;
    int quality = 0;
    int damageFlag = 0;
    std::string environment;
    TrackVisibilityMode visibilityMode = TrackVisibilityMode::Hidden;
    std::optional<SwitchRailVisibility> visibility;
    std::vector<TrackBezier> beziers;
    TrackOptionals optionals;
    std::vector<SourceToken> raw;
};

// road — nawierzchnia + pobocze/chodnik
struct RoadSurfaceVisibility {
    std::string surfaceTexture;
    double surfaceTexLength = 0.0;
    std::string shoulderTexture;
    double shoulderHeight = 0.0;
    double shoulderWidth = 0.0;
    double shoulderSlope = 0.0;
};

struct ParsedTrackRoad {
    NodeHeader header;
    double length = 0.0;
    double roadWidth = 0.0;
    double friction = 0.0;
    double soundDist = 0.0;
    int quality = 0;
    int damageFlag = 0;
    std::string environment;
    TrackVisibilityMode visibilityMode = TrackVisibilityMode::Hidden;
    std::optional<RoadSurfaceVisibility> visibility;
    std::vector<TrackBezier> beziers;
    TrackOptionals optionals;
    std::vector<SourceToken> raw;
};

// cross — jak droga, ale zwykle dwie krzywe
struct ParsedTrackCross {
    NodeHeader header;
    double length = 0.0;
    double roadWidth = 0.0;
    double friction = 0.0;
    double soundDist = 0.0;
    int quality = 0;
    int damageFlag = 0;
    std::string environment;
    TrackVisibilityMode visibilityMode = TrackVisibilityMode::Hidden;
    std::optional<RoadSurfaceVisibility> visibility;
    std::vector<TrackBezier> beziers;
    TrackOptionals optionals;
    std::vector<SourceToken> raw;
};

// river / tributary / turn / table / nieznany typ
struct RiverVisibility {
    std::string waterTexture;
    double waterTexLength = 0.0;
    std::string bankTexture;
    double bankHeight = 0.0;
    double leftBankWidth = 0.0;
    double rightBankWidth = 0.0;
};

struct ParsedTrackOther {
    NodeHeader header;
    std::string trajectoryKind;
    double length = 0.0;
    double width = 0.0;
    double friction = 0.0;
    double soundDist = 0.0;
    int quality = 0;
    int damageFlag = 0;
    std::string environment;
    TrackVisibilityMode visibilityMode = TrackVisibilityMode::Hidden;
    std::optional<RiverVisibility> riverVisibility;
    std::optional<NormalRailVisibility> railVisibility;
    std::vector<TrackBezier> beziers;
    TrackOptionals optionals;
    std::vector<SourceToken> raw;
};

namespace node_track {

inline constexpr std::string_view kSubtype = "track";
inline constexpr std::string_view kEndMarker = "endtrack";

struct CoreParams {
    double length = 0.0;
    double width = 0.0;
    double friction = 0.0;
    double soundDist = 0.0;
    int quality = 0;
    int damageFlag = 0;
    std::string environment;
    TrackVisibilityMode visibilityMode = TrackVisibilityMode::Hidden;
};

[[nodiscard]] inline bool isEnvironment(const std::string_view token) noexcept {
    return isKeywordIgnoreCase(token, "flat") || isKeywordIgnoreCase(token, "mountains") ||
           isKeywordIgnoreCase(token, "mountain") || isKeywordIgnoreCase(token, "canyon") ||
           isKeywordIgnoreCase(token, "tunnel") || isKeywordIgnoreCase(token, "bridge") ||
           isKeywordIgnoreCase(token, "bank");
}

[[nodiscard]] inline bool isTrackKeyword(const std::string_view token) noexcept {
    return isKeyword(token, "velocity") || isKeyword(token, "isolated") ||
           isKeyword(token, "overhead") || isKeyword(token, "railprofile") ||
           isKeyword(token, "trackbed") || isKeyword(token, "friction") ||
           isKeyword(token, "angle1") || isKeyword(token, "angle2") ||
           isKeyword(token, "fouling1") || isKeyword(token, "fouling2") ||
           isKeyword(token, "sleepermodel") || isKeyword(token, "vradius") ||
           isKeyword(token, "event0") || isKeyword(token, "event1") ||
           isKeyword(token, "event2") || isKeyword(token, "eventall0") ||
           isKeyword(token, "eventall1") || isKeyword(token, "eventall2");
}

[[nodiscard]] inline Vec3 addVec3(const Vec3& a, const Vec3& b) noexcept {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

// Wiki MaSzyna: cv1 wzgledem P1, cv2 wzgledem P2.
[[nodiscard]] inline bool takeBezier(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    TrackBezier& segment) {
    if (!node::io::takeVec3(stream, raw, segment.p1) ||
        !node::io::takeDouble(stream, raw, segment.roll1) ||
        !node::io::takeVec3(stream, raw, segment.cv1) ||
        !node::io::takeVec3(stream, raw, segment.cv2) ||
        !node::io::takeVec3(stream, raw, segment.p2) ||
        !node::io::takeDouble(stream, raw, segment.roll2) ||
        !node::io::takeDouble(stream, raw, segment.radius)) {
        return false;
    }

    // cv1/cv2 zostaja wzgledem P1/P2 (wiki MaSzyna) — jak terenAI CubicBezier.
    return true;
}

[[nodiscard]] inline bool takeSleeperModel(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    TrackSleeperModel& out) {
    return node::io::takeDouble(stream, raw, out.spacing) &&
           node::io::takeString(stream, raw, out.model) &&
           node::io::takeString(stream, raw, out.texture) &&
           node::io::takeDouble(stream, raw, out.offsetLateral) &&
           node::io::takeDouble(stream, raw, out.offsetAlong) &&
           node::io::takeDouble(stream, raw, out.offsetVertical) &&
           node::io::takeDouble(stream, raw, out.bedLowering);
}

[[nodiscard]] inline bool takeCoreParams(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    CoreParams& out) {
    std::vector<double> numerics;
    while (!stream.empty() && !isEnvironment(stream.peek().value)) {
        if (!node::io::parseDouble(stream.peek().value)) {
            break;
        }
        double value = 0.0;
        if (!node::io::takeDouble(stream, raw, value)) {
            return false;
        }
        numerics.push_back(value);
    }

    if (numerics.size() >= 1) {
        out.length = numerics[0];
    }
    if (numerics.size() >= 2) {
        out.width = numerics[1];
    }
    if (numerics.size() >= 3) {
        out.friction = numerics[2];
    }
    if (numerics.size() >= 4) {
        out.soundDist = numerics[3];
    }
    if (numerics.size() >= 5) {
        out.quality = static_cast<int>(numerics[4]);
    }
    if (numerics.size() >= 6) {
        out.damageFlag = static_cast<int>(numerics[5]);
    }

    if (!node::io::takeString(stream, raw, out.environment)) {
        return false;
    }

    std::string visToken;
    if (!node::io::takeString(stream, raw, visToken)) {
        return false;
    }
    if (isKeywordIgnoreCase(visToken, "unvis") || isKeywordIgnoreCase(visToken, "novis")) {
        out.visibilityMode = TrackVisibilityMode::Hidden;
        return true;
    }
    if (isKeywordIgnoreCase(visToken, "vis")) {
        out.visibilityMode = TrackVisibilityMode::Visible;
        return true;
    }
    return false;
}

[[nodiscard]] inline bool takeNormalVisibility(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    NormalRailVisibility& vis) {
    return node::io::takeString(stream, raw, vis.railTexture) &&
           node::io::takeDouble(stream, raw, vis.railTexLength) &&
           node::io::takeString(stream, raw, vis.ballastTexture) &&
           node::io::takeDouble(stream, raw, vis.ballastHeight) &&
           node::io::takeDouble(stream, raw, vis.ballastWidth) &&
           node::io::takeDouble(stream, raw, vis.ballastSlope);
}

[[nodiscard]] inline bool takeSwitchVisibility(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    SwitchRailVisibility& vis) {
    return node::io::takeString(stream, raw, vis.mainRailTexture) &&
           node::io::takeDouble(stream, raw, vis.mainRailTexLength) &&
           node::io::takeString(stream, raw, vis.switchRailTexture) &&
           node::io::takeDouble(stream, raw, vis.unusedHeight) &&
           node::io::takeDouble(stream, raw, vis.unusedWidth) &&
           node::io::takeDouble(stream, raw, vis.unusedSlope);
}

[[nodiscard]] inline bool takeRoadVisibility(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    RoadSurfaceVisibility& vis) {
    return node::io::takeString(stream, raw, vis.surfaceTexture) &&
           node::io::takeDouble(stream, raw, vis.surfaceTexLength) &&
           node::io::takeString(stream, raw, vis.shoulderTexture) &&
           node::io::takeDouble(stream, raw, vis.shoulderHeight) &&
           node::io::takeDouble(stream, raw, vis.shoulderWidth) &&
           node::io::takeDouble(stream, raw, vis.shoulderSlope);
}

[[nodiscard]] inline bool takeRiverVisibility(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    RiverVisibility& vis) {
    return node::io::takeString(stream, raw, vis.waterTexture) &&
           node::io::takeDouble(stream, raw, vis.waterTexLength) &&
           node::io::takeString(stream, raw, vis.bankTexture) &&
           node::io::takeDouble(stream, raw, vis.bankHeight) &&
           node::io::takeDouble(stream, raw, vis.leftBankWidth) &&
           node::io::takeDouble(stream, raw, vis.rightBankWidth);
}

[[nodiscard]] inline bool parseOptionalKeyword(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    const std::string_view key,
    TrackOptionals& out) {
    SourceToken kw;
    if (!node::io::takeToken(stream, raw, kw)) {
        return false;
    }

    if (isKeyword(key, "velocity")) {
        double vel = 0.0;
        if (!node::io::takeDouble(stream, raw, vel)) {
            return false;
        }
        out.velocity = vel;
        return true;
    }
    if (isKeyword(key, "isolated")) {
        std::string name;
        if (!node::io::takeString(stream, raw, name)) {
            return false;
        }
        out.isolated.push_back(std::move(name));
        return true;
    }
    if (isKeyword(key, "overhead")) {
        double value = 0.0;
        if (!node::io::takeDouble(stream, raw, value)) {
            return false;
        }
        out.overhead = value;
        return true;
    }
    if (isKeyword(key, "railprofile")) {
        std::string value;
        if (!node::io::takeString(stream, raw, value)) {
            return false;
        }
        out.railProfile = std::move(value);
        return true;
    }
    if (isKeyword(key, "trackbed")) {
        std::string value;
        if (!node::io::takeString(stream, raw, value)) {
            return false;
        }
        out.trackBed = std::move(value);
        return true;
    }
    if (isKeyword(key, "friction")) {
        double value = 0.0;
        if (!node::io::takeDouble(stream, raw, value)) {
            return false;
        }
        out.frictionOverride = value;
        return true;
    }
    if (isKeyword(key, "vradius")) {
        double value = 0.0;
        if (!node::io::takeDouble(stream, raw, value)) {
            return false;
        }
        out.verticalRadius = value;
        return true;
    }
    if (isKeyword(key, "angle1")) {
        double value = 0.0;
        if (!node::io::takeDouble(stream, raw, value)) {
            return false;
        }
        out.angle1 = value;
        return true;
    }
    if (isKeyword(key, "angle2")) {
        double value = 0.0;
        if (!node::io::takeDouble(stream, raw, value)) {
            return false;
        }
        out.angle2 = value;
        return true;
    }
    if (isKeyword(key, "fouling1")) {
        std::string value;
        if (!node::io::takeString(stream, raw, value)) {
            return false;
        }
        out.fouling1 = std::move(value);
        return true;
    }
    if (isKeyword(key, "fouling2")) {
        std::string value;
        if (!node::io::takeString(stream, raw, value)) {
            return false;
        }
        out.fouling2 = std::move(value);
        return true;
    }
    if (isKeyword(key, "sleepermodel")) {
        TrackSleeperModel sleeper;
        if (!takeSleeperModel(stream, raw, sleeper)) {
            return false;
        }
        out.sleeperModel = std::move(sleeper);
        return true;
    }
    if (isKeyword(key, "event0")) {
        std::string value;
        if (!node::io::takeString(stream, raw, value)) {
            return false;
        }
        out.events.occupiedStop = std::move(value);
        return true;
    }
    if (isKeyword(key, "event1")) {
        std::string value;
        if (!node::io::takeString(stream, raw, value)) {
            return false;
        }
        out.events.occupiedEnterP1 = std::move(value);
        return true;
    }
    if (isKeyword(key, "event2")) {
        std::string value;
        if (!node::io::takeString(stream, raw, value)) {
            return false;
        }
        out.events.occupiedEnterP2 = std::move(value);
        return true;
    }
    if (isKeyword(key, "eventall0")) {
        std::string value;
        if (!node::io::takeString(stream, raw, value)) {
            return false;
        }
        out.events.anyStop = std::move(value);
        return true;
    }
    if (isKeyword(key, "eventall1")) {
        std::string value;
        if (!node::io::takeString(stream, raw, value)) {
            return false;
        }
        out.events.anyEnterP1 = std::move(value);
        return true;
    }
    if (isKeyword(key, "eventall2")) {
        std::string value;
        if (!node::io::takeString(stream, raw, value)) {
            return false;
        }
        out.events.anyEnterP2 = std::move(value);
        return true;
    }

    std::string arg;
    if (!node::io::takeString(stream, raw, arg)) {
        return false;
    }
    out.extraKeywords.emplace_back(std::string(key), std::move(arg));
    return true;
}

[[nodiscard]] inline bool parseTail(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    std::vector<TrackBezier>& beziers,
    TrackOptionals& optionals) {
    while (!stream.empty() && !node::io::atEnd(stream, kSubtype, kEndMarker)) {
        const std::string_view value = stream.peek().value;

        if (isTrackKeyword(value)) {
            if (!parseOptionalKeyword(stream, raw, value, optionals)) {
                return false;
            }
            continue;
        }

        if (!node::io::parseDouble(stream.peek().value)) {
            break;
        }

        TrackBezier segment;
        if (!takeBezier(stream, raw, segment)) {
            return false;
        }
        beziers.push_back(segment);
    }

    return node::io::consumeEnd(stream, raw, kSubtype, kEndMarker);
}

[[nodiscard]] inline bool parseNormalBody(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    const NodeHeader& header,
    const std::string& trajectoryKind,
    ParsedTrackNormal& out) {
    out.header = header;
    out.trajectoryKind = trajectoryKind;

    CoreParams core;
    if (!takeCoreParams(stream, raw, core)) {
        return false;
    }
    out.length = core.length;
    out.gauge = core.width;
    out.friction = core.friction;
    out.soundDist = core.soundDist;
    out.quality = core.quality;
    out.damageFlag = core.damageFlag;
    out.environment = std::move(core.environment);
    out.visibilityMode = core.visibilityMode;

    if (out.visibilityMode == TrackVisibilityMode::Visible) {
        NormalRailVisibility vis;
        if (!takeNormalVisibility(stream, raw, vis)) {
            return false;
        }
        out.visibility = std::move(vis);
    }

    return parseTail(stream, raw, out.beziers, out.optionals);
}

[[nodiscard]] inline bool parseSwitchBody(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    const NodeHeader& header,
    ParsedTrackSwitch& out) {
    out.header = header;

    CoreParams core;
    if (!takeCoreParams(stream, raw, core)) {
        return false;
    }
    out.length = core.length;
    out.gauge = core.width;
    out.friction = core.friction;
    out.soundDist = core.soundDist;
    out.quality = core.quality;
    out.damageFlag = core.damageFlag;
    out.environment = std::move(core.environment);
    out.visibilityMode = core.visibilityMode;

    if (out.visibilityMode == TrackVisibilityMode::Visible) {
        SwitchRailVisibility vis;
        if (!takeSwitchVisibility(stream, raw, vis)) {
            return false;
        }
        out.visibility = std::move(vis);
    }

    return parseTail(stream, raw, out.beziers, out.optionals);
}

[[nodiscard]] inline bool parseRoadBody(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    const NodeHeader& header,
    ParsedTrackRoad& out) {
    out.header = header;

    CoreParams core;
    if (!takeCoreParams(stream, raw, core)) {
        return false;
    }
    out.length = core.length;
    out.roadWidth = core.width;
    out.friction = core.friction;
    out.soundDist = core.soundDist;
    out.quality = core.quality;
    out.damageFlag = core.damageFlag;
    out.environment = std::move(core.environment);
    out.visibilityMode = core.visibilityMode;

    if (out.visibilityMode == TrackVisibilityMode::Visible) {
        RoadSurfaceVisibility vis;
        if (!takeRoadVisibility(stream, raw, vis)) {
            return false;
        }
        out.visibility = std::move(vis);
    }

    return parseTail(stream, raw, out.beziers, out.optionals);
}

[[nodiscard]] inline bool parseCrossBody(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    const NodeHeader& header,
    ParsedTrackCross& out) {
    out.header = header;

    CoreParams core;
    if (!takeCoreParams(stream, raw, core)) {
        return false;
    }
    out.length = core.length;
    out.roadWidth = core.width;
    out.friction = core.friction;
    out.soundDist = core.soundDist;
    out.quality = core.quality;
    out.damageFlag = core.damageFlag;
    out.environment = std::move(core.environment);
    out.visibilityMode = core.visibilityMode;

    if (out.visibilityMode == TrackVisibilityMode::Visible) {
        RoadSurfaceVisibility vis;
        if (!takeRoadVisibility(stream, raw, vis)) {
            return false;
        }
        out.visibility = std::move(vis);
    }

    return parseTail(stream, raw, out.beziers, out.optionals);
}

[[nodiscard]] inline bool parseOtherBody(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    const NodeHeader& header,
    const std::string& trajectoryKind,
    ParsedTrackOther& out,
    const bool riverLike) {
    out.header = header;
    out.trajectoryKind = trajectoryKind;

    CoreParams core;
    if (!takeCoreParams(stream, raw, core)) {
        return false;
    }
    out.length = core.length;
    out.width = core.width;
    out.friction = core.friction;
    out.soundDist = core.soundDist;
    out.quality = core.quality;
    out.damageFlag = core.damageFlag;
    out.environment = std::move(core.environment);
    out.visibilityMode = core.visibilityMode;

    if (out.visibilityMode == TrackVisibilityMode::Visible) {
        if (riverLike) {
            RiverVisibility vis;
            if (!takeRiverVisibility(stream, raw, vis)) {
                return false;
            }
            out.riverVisibility = std::move(vis);
        } else {
            NormalRailVisibility vis;
            if (!takeNormalVisibility(stream, raw, vis)) {
                return false;
            }
            out.railVisibility = std::move(vis);
        }
    }

    return parseTail(stream, raw, out.beziers, out.optionals);
}

[[nodiscard]] inline bool isNormalLike(const std::string_view kind) noexcept {
    return isKeyword(kind, "normal") || isKeyword(kind, "turn") || isKeyword(kind, "table");
}

[[nodiscard]] inline bool isRiverLike(const std::string_view kind) noexcept {
    return isKeyword(kind, "river") || isKeyword(kind, "tributary");
}

} // namespace node_track

} // namespace eu07::scene
