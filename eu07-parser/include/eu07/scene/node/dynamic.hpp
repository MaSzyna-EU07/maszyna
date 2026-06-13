#pragma once

// https://wiki.eu07.pl/index.php?title=Obiekt_node::dynamic

#include <eu07/scene/cursor.hpp>
#include <eu07/scene/match.hpp>
#include <eu07/scene/node/header.hpp>
#include <eu07/scene/node/parse_util.hpp>
#include <eu07/scene/node/shared.hpp>

#include <optional>
#include <string>
#include <vector>

namespace eu07::scene {

struct ParsedNodeDynamic {
    NodeHeader header;
    std::string datafolder;
    std::string skinfile;
    std::string mmdfile;
    std::string pathname;
    double vehicleOffset = -1.0;
    std::string driverType;
    CouplingSpec coupling;
    double velocity = 0.0;
    int loadcount = 0;
    std::optional<std::string> loadtype;
    std::optional<std::string> loadModifier;
    std::optional<std::string> destination;
    bool inTrainset = false;
    std::vector<SourceToken> raw;
};

namespace node_dynamic {

inline constexpr std::string_view kSubtype = "dynamic";
inline constexpr std::string_view kEndMarker = "enddynamic";

[[nodiscard]] inline bool parseBody(
    TokenStream& stream,
    std::vector<SourceToken>& raw,
    const NodeHeader& header,
    ParsedNodeDynamic& out) {
    out.header = header;
    out.raw.clear();
    out.inTrainset = header.trainset.has_value();

    if (!node::io::takeString(stream, raw, out.datafolder) ||
        !node::io::takeString(stream, raw, out.skinfile) ||
        !node::io::takeString(stream, raw, out.mmdfile)) {
        return false;
    }

    if (out.inTrainset) {
        out.pathname = header.trainset->track;
    } else if (!node::io::takeString(stream, raw, out.pathname)) {
        return false;
    }

    if (!node::io::takeDouble(stream, raw, out.vehicleOffset) ||
        !node::io::takeString(stream, raw, out.driverType)) {
        return false;
    }

    std::string couplingRaw;
    if (out.inTrainset) {
        if (!node::io::takeString(stream, raw, couplingRaw)) {
            return false;
        }
    } else {
        couplingRaw = "3";
    }
    out.coupling = parseCouplingSpec(couplingRaw);

    if (out.inTrainset) {
        out.velocity = header.trainset->velocity;
    } else if (!node::io::takeDouble(stream, raw, out.velocity)) {
        return false;
    }

    double loadcount = 0.0;
    if (!node::io::takeDouble(stream, raw, loadcount)) {
        return false;
    }
    out.loadcount = static_cast<int>(loadcount);

    if (out.loadcount > 0) {
        if (stream.empty()) {
            return false;
        }
        if (isKeyword(stream.peek().value, kEndMarker)) {
            out.loadcount = 0;
        } else {
            std::string loadtype;
            if (!node::io::takeString(stream, raw, loadtype)) {
                return false;
            }
            if (isKeyword(loadtype, kEndMarker)) {
                out.loadcount = 0;
            } else {
                out.loadtype = std::move(loadtype);
            }
        }
    }

    if (!stream.empty() && !node::io::atEnd(stream, kSubtype, kEndMarker)) {
        std::string token;
        if (!node::io::takeString(stream, raw, token)) {
            return false;
        }
        if (!isKeyword(token, kEndMarker)) {
            std::string dest;
            if (!node::io::takeString(stream, raw, dest)) {
                return false;
            }
            if (!isKeyword(dest, kEndMarker)) {
                out.destination = std::move(dest);
            }
        }
    }

    return node::io::consumeEnd(stream, raw, kSubtype, kEndMarker);
}

} // namespace node_dynamic

} // namespace eu07::scene
