#pragma once

#include <eu07/scene/cursor.hpp>
#include <eu07/scene/document.hpp>
#include <eu07/scene/match.hpp>
#include <eu07/scene/node/parse_util.hpp>
#include <eu07/scene/runtime/directives.hpp>

#include <cctype>
#include <string>
#include <vector>

namespace eu07::scene::bake {

[[nodiscard]] inline runtime::EventType parseEventType(std::string_view type) {
    if (isKeywordIgnoreCase(type, "addvalues")) {
        return runtime::EventType::AddValues;
    }
    if (isKeywordIgnoreCase(type, "updatevalues")) {
        return runtime::EventType::UpdateValues;
    }
    if (isKeywordIgnoreCase(type, "copyvalues")) {
        return runtime::EventType::CopyValues;
    }
    if (isKeywordIgnoreCase(type, "getvalues")) {
        return runtime::EventType::GetValues;
    }
    if (isKeywordIgnoreCase(type, "putvalues")) {
        return runtime::EventType::PutValues;
    }
    if (isKeywordIgnoreCase(type, "whois")) {
        return runtime::EventType::Whois;
    }
    if (isKeywordIgnoreCase(type, "logvalues")) {
        return runtime::EventType::LogValues;
    }
    if (isKeywordIgnoreCase(type, "multiple")) {
        return runtime::EventType::Multiple;
    }
    if (isKeywordIgnoreCase(type, "switch")) {
        return runtime::EventType::Switch;
    }
    if (isKeywordIgnoreCase(type, "trackvel")) {
        return runtime::EventType::TrackVel;
    }
    if (isKeywordIgnoreCase(type, "sound")) {
        return runtime::EventType::Sound;
    }
    if (isKeywordIgnoreCase(type, "texture")) {
        return runtime::EventType::Texture;
    }
    if (isKeywordIgnoreCase(type, "animation")) {
        return runtime::EventType::Animation;
    }
    if (isKeywordIgnoreCase(type, "lights")) {
        return runtime::EventType::Lights;
    }
    if (isKeywordIgnoreCase(type, "voltage")) {
        return runtime::EventType::Voltage;
    }
    if (isKeywordIgnoreCase(type, "visible")) {
        return runtime::EventType::Visible;
    }
    if (isKeywordIgnoreCase(type, "friction")) {
        return runtime::EventType::Friction;
    }
    if (isKeywordIgnoreCase(type, "message")) {
        return runtime::EventType::Message;
    }
    return runtime::EventType::Unknown;
}

namespace detail {

inline void splitTargetList(const std::string& text, std::vector<std::string>& out) {
    std::string current;
    for (const char ch : text) {
        if (ch == '|' || ch == ',') {
            if (!current.empty() && !isKeywordIgnoreCase(current, "none")) {
                out.push_back(current);
            }
            current.clear();
        } else if (!std::isspace(static_cast<unsigned char>(ch))) {
            current.push_back(ch);
        }
    }
    if (!current.empty() && !isKeywordIgnoreCase(current, "none")) {
        out.push_back(current);
    }
}

} // namespace detail

[[nodiscard]] inline runtime::RuntimeEvent bakeEvent(const DirectiveBlock& block) {
    runtime::RuntimeEvent event;
    TokenStream stream(block.tokens);
    if (stream.empty() || !isKeyword(stream.peek().value, "event")) {
        return event;
    }
    stream.consume();

    std::vector<SourceToken> raw;
    std::string name;
    std::string type;
    double delay = 0.0;
    if (!node::io::takeString(stream, raw, name) || !node::io::takeString(stream, raw, type) ||
        !node::io::takeDouble(stream, raw, delay)) {
        return event;
    }

    event.name = std::move(name);
    event.type = parseEventType(type);
    event.delay = delay;

    if (event.name.starts_with("none_")) {
        event.ignored = true;
    }

    if (!stream.empty() && !isKeyword(stream.peek().value, "endevent")) {
        std::string targets;
        if (node::io::takeString(stream, raw, targets)) {
            detail::splitTargetList(targets, event.targets);
        }
    }

    while (!stream.empty() && !isKeyword(stream.peek().value, "endevent")) {
        std::string token;
        if (!node::io::takeString(stream, raw, token)) {
            break;
        }
        if (isKeywordIgnoreCase(token, "randomdelay")) {
            double value = 0.0;
            if (node::io::takeDouble(stream, raw, value)) {
                event.delayRandom = value;
            }
            continue;
        }
        if (isKeywordIgnoreCase(token, "departuredelay")) {
            double value = 0.0;
            if (node::io::takeDouble(stream, raw, value)) {
                event.delayDeparture = value;
            }
            continue;
        }
        if (isKeywordIgnoreCase(token, "passive")) {
            event.passive = true;
            continue;
        }
        event.payload.emplace_back(std::move(token), "");
    }

    return event;
}

[[nodiscard]] inline std::vector<runtime::RuntimeEvent> bakeEvents(const SceneDocument& document) {
    std::vector<runtime::RuntimeEvent> events;
    events.reserve(document.event.size());
    for (const DirectiveBlock& block : document.event) {
        events.push_back(bakeEvent(block));
    }
    return events;
}

} // namespace eu07::scene::bake
