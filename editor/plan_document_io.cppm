/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/
module;
#include <optional>
#include <string>

export module eu07.editor.plan_document_io;
export import eu07.editor.plan_model;

export {

namespace editor::plan::io {

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
[[nodiscard]] std::string serialize(const plan::Document& document);

/// Returns nothing when @p text is not a version 3 document or is malformed.
[[nodiscard]] std::optional<plan::Document> deserialize(const std::string& text);

/// Where plans live: `editor/plan.m0s`, relative to the working directory — which
/// for the simulator is its own data directory, beside `scenery` and `dynamic`.
[[nodiscard]] std::string default_project_path();

/// The file a name means. A name with no directory in it is a plan, so it belongs
/// where plans live; anything else is taken as given. Use it to say where a plan
/// went, or came from, without guessing.
[[nodiscard]] std::string project_file(const std::string& path);
bool save(const plan::Document& document, const std::string& path = default_project_path());
[[nodiscard]] std::optional<plan::Document> load(
    const std::string& path = default_project_path());

}  // namespace editor::plan::io

}  // export
