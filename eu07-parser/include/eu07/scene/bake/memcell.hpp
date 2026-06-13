#pragma once

#include <eu07/scene/bake/geometry.hpp>
#include <eu07/scene/bake/track.hpp>
#include <eu07/scene/node/memcell.hpp>
#include <eu07/scene/runtime/nodes.hpp>

namespace eu07::scene::bake {

[[nodiscard]] inline runtime::RuntimeMemCell bakeMemcell(const ParsedNodeMemcell& parsed) {
    runtime::RuntimeMemCell cell;
    cell.node = bakeBasicNode(parsed.header, "memcell");
    cell.text = parsed.command;
    cell.value1 = parsed.value1;
    cell.value2 = parsed.value2;
    cell.trackName = parsed.trackName;

    const runtime::RuntimeScratchpad scratch = scratchFromHeader(parsed.header);
    cell.node.area.center = runtime::transformPoint(parsed.position, scratch);

    return cell;
}

} // namespace eu07::scene::bake
