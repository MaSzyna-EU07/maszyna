#pragma once

#include <string>

#include "maj0sted/domain/map_project.hpp"

namespace maj0sted::io {

/// Serializes a whole map project (CRS + niwelety, each with its plan and
/// profile) to a self-contained textual format. The format is line-oriented and
/// dependency-free; @c deserialize round-trips it exactly.
[[nodiscard]] std::string serialize(const maj0sted::domain::MapProject& project);

/// Rebuilds a map project from text produced by @c serialize.
/// @throws std::runtime_error on malformed input; std::invalid_argument if the
///         reconstructed geometry violates a domain invariant.
[[nodiscard]] maj0sted::domain::MapProject deserialize(const std::string& text);

}  // namespace maj0sted::io
