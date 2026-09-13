#pragma once

#include <optional>
#include <string>

#include "maj0sted/editor/model.hpp"

namespace maj0sted::io {

/// The `.m0s` format, version 3.
///
/// Line-oriented text, one concept per token, every reference by id, and an
/// explicit version on the first line. Earlier versions are not read: they
/// described different models — anchored straights with mode-tagged fits between
/// them (1), then elements whose lengths were solved against construction lines
/// (2) — and translating either would have meant keeping those concepts alive in
/// the reader after they were removed from everything else.
///
/// Doubles go through std::to_chars/from_chars, so a round trip is exact and
/// does not depend on the locale.
[[nodiscard]] std::string serialize(const editor::Document& document);

/// Returns nothing when @p text is not a version 3 document or is malformed.
[[nodiscard]] std::optional<editor::Document> deserialize(const std::string& text);

[[nodiscard]] std::string default_project_path();
bool save(const editor::Document& document, const std::string& path = default_project_path());
[[nodiscard]] std::optional<editor::Document> load(
    const std::string& path = default_project_path());

}  // namespace maj0sted::io
