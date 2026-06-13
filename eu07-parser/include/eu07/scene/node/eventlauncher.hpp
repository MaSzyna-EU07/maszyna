#pragma once

// https://wiki.eu07.pl/index.php?title=Obiekt_node::eventlauncher

#include <eu07/scene/cursor.hpp>
#include <eu07/scene/match.hpp>
#include <eu07/scene/node/header.hpp>
#include <eu07/scene/node/parse_util.hpp>
#include <eu07/scene/node/types.hpp>

#include <optional>
#include <string>
#include <vector>

namespace eu07::scene {

struct EventLauncherCondition {
    std::string memcell;
    std::string parameter;
    std::optional<double> value1;
    std::optional<double> value2;
    int checkMask = 0;
};

struct ParsedNodeEventlauncher {
    NodeHeader header;
    Vec3 position;
    double radius = 0.0;
    std::string key;
    std::string time;
    std::string event1;
    std::optional<std::string> event2;
    std::optional<EventLauncherCondition> condition;
    bool trainTriggered = false;
    std::vector<SourceToken> raw;
};

namespace node_eventlauncher {

inline constexpr std::string_view kSubtype = "eventlauncher";
inline constexpr std::string_view kEndMarker = "endeventlauncher";

[[nodiscard]] inline bool takeConditionField(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    const int flag,
    std::optional<double>& out,
    int& checkMask) {
    std::string token;
    if (!node::io::takeString(stream, raw, token)) {
        return false;
    }
    if (token == "*") {
        return true;
    }
    checkMask |= flag;
    const std::optional<double> value = node::io::parseDouble(token);
    if (!value) {
        return false;
    }
    out = *value;
    return true;
}

[[nodiscard]] inline bool consumeLauncherEnd(TokenStream& stream, std::vector<SourceToken>& raw) {
    return node::io::consumeEnd(stream, raw, kSubtype, kEndMarker);
}

[[nodiscard]] inline bool parseBody(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    const NodeHeader& header,
    ParsedNodeEventlauncher& out) {
    out.header = header;
    out.raw.clear();

    if (!node::io::takeVec3(stream, raw, out.position) ||
        !node::io::takeDouble(stream, raw, out.radius) || !node::io::takeString(stream, raw, out.key) ||
        !node::io::takeString(stream, raw, out.time) || !node::io::takeString(stream, raw, out.event1)) {
        return false;
    }

    if (stream.empty() || node::io::atEnd(stream, kSubtype, kEndMarker)) {
        return consumeLauncherEnd(stream, raw);
    }

    std::string token;
    if (!node::io::takeString(stream, raw, token)) {
        return false;
    }

    // MaSzyna (EvLaunch.cpp): "end" zamiast drugiego eventu lub endeventlauncher.
    if (isKeyword(token, "end") || isKeyword(token, "endeventlauncher")) {
        if (isKeyword(token, "endeventlauncher")) {
            return consumeLauncherEnd(stream, raw);
        }
        return true;
    }

    if (!isKeyword(token, "condition") && !isKeyword(token, "traintriggered")) {
        out.event2 = token;
        if (stream.empty() || node::io::atEnd(stream, kSubtype, kEndMarker)) {
            return consumeLauncherEnd(stream, raw);
        }
        if (!node::io::takeString(stream, raw, token)) {
            return false;
        }
        if (isKeyword(token, "end") || isKeyword(token, "endeventlauncher")) {
            if (isKeyword(token, "endeventlauncher")) {
                return consumeLauncherEnd(stream, raw);
            }
            return true;
        }
    }

    if (isKeyword(token, "condition")) {
        EventLauncherCondition cond;
        if (!node::io::takeString(stream, raw, cond.memcell) ||
            !node::io::takeString(stream, raw, cond.parameter)) {
            return false;
        }
        if (cond.parameter != "*") {
            cond.checkMask |= 1;
        }
        if (!takeConditionField(stream, raw, 2, cond.value1, cond.checkMask)) {
            return false;
        }
        if (!takeConditionField(stream, raw, 4, cond.value2, cond.checkMask)) {
            return false;
        }
        if (!stream.empty() && !node::io::atEnd(stream, kSubtype, kEndMarker)) {
            std::string closing;
            if (!node::io::takeString(stream, raw, closing)) {
                return false;
            }
            token = std::move(closing);
        }
        out.condition = std::move(cond);
        if (isKeyword(token, "end")) {
            return true;
        }
    }

    if (isKeyword(token, "traintriggered")) {
        out.trainTriggered = true;
        if (stream.empty()) {
            return true;
        }
        if (node::io::atEnd(stream, kSubtype, kEndMarker)) {
            return consumeLauncherEnd(stream, raw);
        }
        if (!node::io::takeString(stream, raw, token)) {
            return false;
        }
        if (isKeyword(token, "end")) {
            return true;
        }
    }

    if (stream.empty()) {
        return true;
    }

    return consumeLauncherEnd(stream, raw);
}

} // namespace node_eventlauncher

} // namespace eu07::scene
