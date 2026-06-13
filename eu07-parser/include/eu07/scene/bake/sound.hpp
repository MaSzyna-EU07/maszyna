#pragma once

#include <eu07/scene/bake/geometry.hpp>
#include <eu07/scene/bake/track.hpp>
#include <eu07/scene/node/sound.hpp>
#include <eu07/scene/runtime/nodes.hpp>

namespace eu07::scene::bake {

[[nodiscard]] inline runtime::RuntimeSoundSource bakeSound(const ParsedNodeSound& parsed) {
    runtime::RuntimeSoundSource sound;
    sound.node = bakeBasicNode(parsed.header, "sound");
    sound.wavFile = parsed.wavFile;

    const runtime::RuntimeScratchpad scratch = scratchFromHeader(parsed.header);
    sound.location = runtime::transformPoint(parsed.position, scratch);
    sound.node.area.center = sound.location;

    return sound;
}

} // namespace eu07::scene::bake
