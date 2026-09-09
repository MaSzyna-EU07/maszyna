#pragma once

// Macros cannot cross a module boundary, so they live in a plain header
// that consumers pull into their global module fragment. The bodies are
// expanded at the use site, where the imported names are in scope, so this
// header deliberately includes nothing.

#define MAKE_ID4(a, b, c, d) (((std::uint32_t)(d) << 24) | ((std::uint32_t)(c) << 16) | ((std::uint32_t)(b) << 8) | (std::uint32_t)(a))
