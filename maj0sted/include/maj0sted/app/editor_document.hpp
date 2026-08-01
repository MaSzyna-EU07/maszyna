#pragma once

#include <optional>
#include <string>
#include <vector>

#include "maj0sted/domain/map_project.hpp"
#include "maj0sted/editor/editor.hpp"

namespace maj0sted::app {

struct EditorDocument {
    std::vector<maj0sted::editor::NiweletaSpec> niwelety;
    std::vector<maj0sted::editor::Junction> junctions;
    double view_x{0.0};
    double view_y{0.0};
    double view_extent{0.0};
    bool origin_set{false};
    bool georeferenced{false};
    double origin_x{0.0};
    double origin_y{0.0};
};

[[nodiscard]] maj0sted::domain::MapProject to_map_project(const EditorDocument& document);
[[nodiscard]] EditorDocument from_map_project(const maj0sted::domain::MapProject& project);
[[nodiscard]] std::string serialize_document(const EditorDocument& document);
[[nodiscard]] EditorDocument deserialize_document(const std::string& text);
[[nodiscard]] std::string default_project_path();
bool save_project(const EditorDocument& document,
                  const std::string& path = default_project_path());
[[nodiscard]] std::optional<EditorDocument> load_project(
    const std::string& path = default_project_path());

}  // namespace maj0sted::app
