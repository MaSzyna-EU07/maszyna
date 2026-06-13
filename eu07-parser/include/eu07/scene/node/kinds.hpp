#pragma once

#include <eu07/scene/document.hpp>
#include <eu07/scene/node/common.hpp>
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

#include <array>
#include <span>
#include <vector>

namespace eu07::scene::node::io {

inline void storeDynamic(SceneDocument& document, ParsedNodeDynamic&& node) {
    document.nodeDynamic.push_back(std::move(node));
}
inline void storeModel(SceneDocument& document, ParsedNodeModel&& node) {
    document.nodeModel.push_back(std::move(node));
}

inline bool parseTrack(
    TokenStream& stream,
    SceneDocument& document,
    const NodeHeader& header,
    std::vector<SourceToken>* embedRaw) {
    std::vector<SourceToken> raw;
    std::string kindToken;
    if (!node::io::takeString(stream, raw, kindToken)) {
        return false;
    }

    auto finalize = [&](auto& node) {
        node.raw = std::move(raw);
        if (embedRaw != nullptr) {
            embedRaw->insert(embedRaw->end(), node.raw.begin(), node.raw.end());
        }
    };

    if (node_track::isNormalLike(kindToken)) {
        ParsedTrackNormal node;
        if (!node_track::parseNormalBody(stream, raw, header, kindToken, node)) {
            return false;
        }
        finalize(node);
        document.nodeTrackNormal.push_back(std::move(node));
        return true;
    }
    if (isKeyword(kindToken, "switch")) {
        ParsedTrackSwitch node;
        if (!node_track::parseSwitchBody(stream, raw, header, node)) {
            return false;
        }
        finalize(node);
        document.nodeTrackSwitch.push_back(std::move(node));
        return true;
    }
    if (isKeyword(kindToken, "road")) {
        ParsedTrackRoad node;
        if (!node_track::parseRoadBody(stream, raw, header, node)) {
            return false;
        }
        finalize(node);
        document.nodeTrackRoad.push_back(std::move(node));
        return true;
    }
    if (isKeyword(kindToken, "cross")) {
        ParsedTrackCross node;
        if (!node_track::parseCrossBody(stream, raw, header, node)) {
            return false;
        }
        finalize(node);
        document.nodeTrackCross.push_back(std::move(node));
        return true;
    }

    ParsedTrackOther node;
    if (!node_track::parseOtherBody(
            stream, raw, header, kindToken, node, node_track::isRiverLike(kindToken))) {
        return false;
    }
    finalize(node);
    document.nodeTrackOther.push_back(std::move(node));
    return true;
}
inline void storeTraction(SceneDocument& document, ParsedNodeTraction&& node) {
    document.nodeTraction.push_back(std::move(node));
}
inline void storeTractionPower(SceneDocument& document, ParsedNodeTractionPowerSource&& node) {
    document.nodeTractionPower.push_back(std::move(node));
}
inline void storeTriangles(SceneDocument& document, ParsedNodeTriangles&& node) {
    document.nodeTriangles.push_back(std::move(node));
}
inline void storeTriangleStrip(SceneDocument& document, ParsedNodeTriangleStrip&& node) {
    document.nodeTriangleStrip.push_back(std::move(node));
}
inline void storeTriangleFan(SceneDocument& document, ParsedNodeTriangleFan&& node) {
    document.nodeTriangleFan.push_back(std::move(node));
}
inline void storeLines(SceneDocument& document, ParsedNodeLines&& node) {
    document.nodeLines.push_back(std::move(node));
}
inline void storeLineStrip(SceneDocument& document, ParsedNodeLineStrip&& node) {
    document.nodeLineStrip.push_back(std::move(node));
}
inline void storeLineLoop(SceneDocument& document, ParsedNodeLineLoop&& node) {
    document.nodeLineLoop.push_back(std::move(node));
}
inline void storeMemcell(SceneDocument& document, ParsedNodeMemcell&& node) {
    document.nodeMemcell.push_back(std::move(node));
}
inline void storeEventlauncher(SceneDocument& document, ParsedNodeEventlauncher&& node) {
    document.nodeEventlauncher.push_back(std::move(node));
}
inline void storeSound(SceneDocument& document, ParsedNodeSound&& node) {
    document.nodeSound.push_back(std::move(node));
}

template <typename Parsed, bool (*ParseBody)(TokenStream&, std::vector<SourceToken>&, const NodeHeader&, Parsed&), void (*Store)(SceneDocument&, Parsed&&)>
[[nodiscard]] inline bool parseKind(
    TokenStream& stream,
    SceneDocument& document,
    const NodeHeader& header,
    std::vector<SourceToken>* embedRaw) {
    std::vector<SourceToken> raw;
    Parsed parsed;
    if (!ParseBody(stream, raw, header, parsed)) {
        return false;
    }
    parsed.raw = std::move(raw);
    if (embedRaw != nullptr) {
        embedRaw->insert(embedRaw->end(), parsed.raw.begin(), parsed.raw.end());
    }
    Store(document, std::move(parsed));
    return true;
}

inline bool parseDynamic(
    TokenStream& stream,
    SceneDocument& document,
    const NodeHeader& header,
    std::vector<SourceToken>* embedRaw) {
    return parseKind<ParsedNodeDynamic, node_dynamic::parseBody, storeDynamic>(
        stream, document, header, embedRaw);
}

inline bool parseModel(
    TokenStream& stream,
    SceneDocument& document,
    const NodeHeader& header,
    std::vector<SourceToken>* embedRaw) {
    return parseKind<ParsedNodeModel, node_model::parseBody, storeModel>(
        stream, document, header, embedRaw);
}

inline bool parseTraction(
    TokenStream& stream,
    SceneDocument& document,
    const NodeHeader& header,
    std::vector<SourceToken>* embedRaw) {
    return parseKind<ParsedNodeTraction, node_traction::parseBody, storeTraction>(
        stream, document, header, embedRaw);
}

inline bool parseTractionPower(
    TokenStream& stream,
    SceneDocument& document,
    const NodeHeader& header,
    std::vector<SourceToken>* embedRaw) {
    return parseKind<ParsedNodeTractionPowerSource, node_traction_power::parseBody, storeTractionPower>(
        stream, document, header, embedRaw);
}

inline bool parseTriangles(
    TokenStream& stream,
    SceneDocument& document,
    const NodeHeader& header,
    std::vector<SourceToken>* embedRaw) {
    return parseKind<ParsedNodeTriangles, node_triangles::parseBody, storeTriangles>(
        stream, document, header, embedRaw);
}

inline bool parseTriangleStrip(
    TokenStream& stream,
    SceneDocument& document,
    const NodeHeader& header,
    std::vector<SourceToken>* embedRaw) {
    return parseKind<ParsedNodeTriangleStrip, node_triangle_strip::parseBody, storeTriangleStrip>(
        stream, document, header, embedRaw);
}

inline bool parseTriangleFan(
    TokenStream& stream,
    SceneDocument& document,
    const NodeHeader& header,
    std::vector<SourceToken>* embedRaw) {
    return parseKind<ParsedNodeTriangleFan, node_triangle_fan::parseBody, storeTriangleFan>(
        stream, document, header, embedRaw);
}

inline bool parseLines(
    TokenStream& stream,
    SceneDocument& document,
    const NodeHeader& header,
    std::vector<SourceToken>* embedRaw) {
    return parseKind<ParsedNodeLines, node_lines::parseBody, storeLines>(
        stream, document, header, embedRaw);
}

inline bool parseLineStrip(
    TokenStream& stream,
    SceneDocument& document,
    const NodeHeader& header,
    std::vector<SourceToken>* embedRaw) {
    return parseKind<ParsedNodeLineStrip, node_line_strip::parseBody, storeLineStrip>(
        stream, document, header, embedRaw);
}

inline bool parseLineLoop(
    TokenStream& stream,
    SceneDocument& document,
    const NodeHeader& header,
    std::vector<SourceToken>* embedRaw) {
    return parseKind<ParsedNodeLineLoop, node_line_loop::parseBody, storeLineLoop>(
        stream, document, header, embedRaw);
}

inline bool parseMemcell(
    TokenStream& stream,
    SceneDocument& document,
    const NodeHeader& header,
    std::vector<SourceToken>* embedRaw) {
    return parseKind<ParsedNodeMemcell, node_memcell::parseBody, storeMemcell>(
        stream, document, header, embedRaw);
}

inline bool parseEventlauncher(
    TokenStream& stream,
    SceneDocument& document,
    const NodeHeader& header,
    std::vector<SourceToken>* embedRaw) {
    return parseKind<ParsedNodeEventlauncher, node_eventlauncher::parseBody, storeEventlauncher>(
        stream, document, header, embedRaw);
}

inline bool parseSound(
    TokenStream& stream,
    SceneDocument& document,
    const NodeHeader& header,
    std::vector<SourceToken>* embedRaw) {
    return parseKind<ParsedNodeSound, node_sound::parseBody, storeSound>(
        stream, document, header, embedRaw);
}

inline constexpr std::array<NodeKindDesc, 14> kAllKinds{{
    {node_dynamic::kSubtype, node_dynamic::kEndMarker, &parseDynamic},
    {node_model::kSubtype, node_model::kEndMarker, &parseModel},
    {node_track::kSubtype, node_track::kEndMarker, &parseTrack},
    {node_traction::kSubtype, node_traction::kEndMarker, &parseTraction},
    {node_traction_power::kSubtype, node_traction_power::kEndMarker, &parseTractionPower},
    {node_triangles::kSubtype, node_triangles::kEndMarker, &parseTriangles},
    {node_triangle_strip::kSubtype, node_triangle_strip::kEndMarker, &parseTriangleStrip},
    {node_triangle_fan::kSubtype, node_triangle_fan::kEndMarker, &parseTriangleFan},
    {node_lines::kSubtype, node_lines::kEndMarker, &parseLines},
    {node_line_strip::kSubtype, node_line_strip::kEndMarker, &parseLineStrip},
    {node_line_loop::kSubtype, node_line_loop::kEndMarker, &parseLineLoop},
    {node_memcell::kSubtype, node_memcell::kEndMarker, &parseMemcell},
    {node_eventlauncher::kSubtype, node_eventlauncher::kEndMarker, &parseEventlauncher},
    {node_sound::kSubtype, node_sound::kEndMarker, &parseSound},
}};

[[nodiscard]] inline const NodeKindDesc* findKind(
    const std::string_view subtype,
    const std::span<const NodeKindDesc> kinds) noexcept {
    for (const NodeKindDesc& kind : kinds) {
        if (isKeyword(subtype, kind.subtype)) {
            return &kind;
        }
    }
    return nullptr;
}

} // namespace eu07::scene::node::io
