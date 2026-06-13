#pragma once

// Jedna tabela: płaski dispatch jak w simulationstateserializer.cpp (bez processNested).

#include <eu07/scene/area.hpp>
#include <eu07/scene/atmo.hpp>
#include <eu07/scene/boundaries.hpp>
#include <eu07/scene/camera.hpp>
#include <eu07/scene/config.hpp>
#include <eu07/scene/context.hpp>
#include <eu07/scene/cursor.hpp>
#include <eu07/scene/description.hpp>
#include <eu07/scene/document.hpp>
#include <eu07/scene/event.hpp>
#include <eu07/scene/first_init.hpp>
#include <eu07/scene/group.hpp>
#include <eu07/scene/include.hpp>
#include <eu07/scene/isolated.hpp>
#include <eu07/scene/light.hpp>
#include <eu07/scene/lua.hpp>
#include <eu07/scene/match.hpp>
#include <eu07/scene/node.hpp>
#include <eu07/scene/origin.hpp>
#include <eu07/scene/rotate.hpp>
#include <eu07/scene/scale.hpp>
#include <eu07/scene/sky.hpp>
#include <eu07/scene/terrain.hpp>
#include <eu07/scene/test.hpp>
#include <eu07/scene/time.hpp>
#include <eu07/scene/trainset.hpp>

#include <array>
#include <string_view>

namespace eu07::scene {

struct DirectiveEntry {
    std::string_view keyword;
    DirectiveParser parse;
};

namespace detail {

[[nodiscard]] inline bool matchOrigin(const std::string_view token) noexcept {
    return isKeyword(token, "origin") || isKeyword(token, "orgin");
}

[[nodiscard]] inline bool matchFirstInit(const std::string_view token) noexcept {
    return isKeywordIgnoreCase(token, "FirstInit");
}

[[nodiscard]] inline bool matchesEntry(const DirectiveEntry& entry, const std::string_view token) noexcept {
    if (entry.keyword == "origin") {
        return matchOrigin(token);
    }
    if (entry.keyword == "FirstInit") {
        return matchFirstInit(token);
    }
    return isKeyword(token, entry.keyword);
}

inline constexpr std::array<DirectiveEntry, 27> kDirectiveTable{{
    {"trainset", trainset::parse},
    {"endtrainset", endtrainset::parse},
    {"assignment", assignment::parse},
    {"include", scn_include::parse},
    {"FirstInit", first_init::parse},
    {"origin", origin::parse},
    {"endorigin", endorigin::parse},
    {"scale", scale::parse},
    {"endscale", endscale::parse},
    {"rotate", rotate::parse},
    {"group", group::parse},
    {"endgroup", endgroup::parse},
    {"test", test::parse},
    {"node", node::parse},
    {"event", event::parse},
    {"atmo", atmo::parse},
    {"sky", sky::parse},
    {"time", time::parse},
    {"camera", camera::parse},
    {"config", config::parse},
    {"lua", lua::parse},
    {"isolated", isolated::parse},
    {"area", area::parse},
    {"terrain", terrain::parse},
    {"description", description::parse},
    {"light", light::parse},
}};

[[nodiscard]] inline bool dispatchDirective(TokenStream& stream, ParseContext& context) {
    if (stream.empty()) {
        return false;
    }

    const std::string_view head = stream.peek().value;
    for (const DirectiveEntry& entry : kDirectiveTable) {
        if (!matchesEntry(entry, head)) {
            continue;
        }
        if (entry.parse(stream, context)) {
            return true;
        }
    }
    return false;
}

} // namespace detail

} // namespace eu07::scene
