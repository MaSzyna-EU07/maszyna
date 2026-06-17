#pragma once

// https://wiki.eu07.pl/index.php?title=Obiekt_node

#include <eu07/scene/context.hpp>
#include <eu07/scene/cursor.hpp>
#include <eu07/scene/node/common.hpp>
#include <eu07/scene/node/kinds.hpp>
#include <eu07/scene/node/shared.hpp>
#include <eu07/scene/scratch.hpp>

#include <string>
#include <vector>

namespace eu07::scene::node {

inline void applyScratchContext(NodeHeader& header, const SceneScratchpad& scratch) {
    header.originOffset = scratch.currentOrigin();
    header.scaleFactor = scratch.currentScale();
    header.rotation = scratch.location.rotation;
    header.originStack = scratch.location.originStack;
    header.scaleStack = scratch.location.scaleStack;
    header.groupStackDepth = scratch.group.activeStack.size();
    header.groupHandle = scratch.activeGroupHandle();
    if (scratch.trainset.isOpen && scratch.trainset.docIndex) {
        header.trainsetIndex = *scratch.trainset.docIndex;
        header.trainset = TrainsetContext{
            scratch.trainset.name,
            scratch.trainset.track,
            scratch.trainset.offset,
            scratch.trainset.velocity,
        };
    }
}

[[nodiscard]] inline bool parseInto(TokenStream& stream, ParseContext& context) {
    const std::size_t anchor = stream.checkpoint();

    NodeHeader header;
    std::string subtype;
    std::vector<SourceToken> raw;
    if (!io::consumeHeader(stream, header, subtype, raw)) {
        return false;
    }

    applyScratchContext(header, context.scratch);

    const io::NodeKindDesc* const kind = io::findKind(subtype, io::kAllKinds);
    if (kind == nullptr) {
        stream.rewind(anchor);
        return false;
    }

    if (context.packComposeLightweight) {
        std::vector<SourceToken> raw;
        while (!stream.empty() && !io::atEnd(stream, kind->subtype, kind->endMarker)) {
            stream.consume();
        }
        if (!io::consumeEnd(stream, raw, kind->subtype, kind->endMarker)) {
            stream.rewind(anchor);
            return false;
        }
        return true;
    }

    if (!kind->parseAndStore(stream, context.document, header, nullptr)) {
        stream.rewind(anchor);
        return false;
    }

    return true;
}

[[nodiscard]] inline bool parse(TokenStream& stream, ParseContext& context) {
    return parseInto(stream, context);
}

} // namespace eu07::scene::node
