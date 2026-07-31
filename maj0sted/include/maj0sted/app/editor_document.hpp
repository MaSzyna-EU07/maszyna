#pragma once

#include <optional>
#include <string>
#include <vector>

#include "maj0sted/domain/map_project.hpp"
#include "maj0sted/web/editor.hpp"  // NiweletaSpec (the editor's GUI model)

namespace maj0sted::app {

/// The editor's persistable document: the GUI model exactly as the user built
/// it — independent anchored straights plus the parametric fits between them.
///
/// This is the source of truth for save/restore because it is lossless: it keeps
/// the untrimmed straights and the fit *parameters* (radius, transition lengths,
/// compound arcs, ...), which the resolved domain geometry alone cannot express.
struct EditorDocument {
    std::vector<maj0sted::web::NiweletaSpec> niwelety;
};

/// Best-effort resolved-domain view of @p document: one Niweleta per editor
/// niweleta, keeping the anchored straights (a floating placeholder arc sits in
/// each gap so the alignment stays valid). Used for interop / inspection and so
/// the on-disk file embeds a real maj0sted project. Never throws.
[[nodiscard]] maj0sted::domain::MapProject to_map_project(const EditorDocument& document);

/// Imports a plain domain project into the editor model: every Straight in each
/// niweleta's plan becomes an editor straight (fits are left empty — resolved
/// geometry carries no fit parameters to recover).
[[nodiscard]] EditorDocument from_map_project(const maj0sted::domain::MapProject& project);

/// Serializes @p document to the library's textual format. The document embeds a
/// real maj0sted project (via maj0sted::io::serialize of to_map_project) followed
/// by an `editor` section that captures the straights and fit parameters
/// losslessly. @c deserialize_document restores it exactly.
[[nodiscard]] std::string serialize_document(const EditorDocument& document);

/// Rebuilds an EditorDocument from text produced by @c serialize_document.
/// @throws std::runtime_error on malformed input.
[[nodiscard]] EditorDocument deserialize_document(const std::string& text);

/// The default on-disk project path. Under Emscripten this lives beneath the
/// IDBFS mount point (see web/index.html), so it survives reloads once flushed.
[[nodiscard]] std::string default_project_path();

/// Serializes @p document and writes it to @p path (creating parent directories).
/// @returns true on success; never throws.
bool save_project(const EditorDocument& document,
                  const std::string& path = default_project_path());

/// Reads and deserializes the project at @p path, or std::nullopt if the file is
/// absent or unreadable/malformed. Never throws.
[[nodiscard]] std::optional<EditorDocument> load_project(
    const std::string& path = default_project_path());

}  // namespace maj0sted::app
