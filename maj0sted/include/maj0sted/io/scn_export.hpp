#pragma once

#include <ostream>
#include <string>
#include <vector>

#include "maj0sted/editor/editor.hpp"

namespace maj0sted::io {

struct ScnExportOptions {
    double origin_east{0.0};
    double origin_north{0.0};
    double rail_y{0.2};
    double max_arc_angle{1.5707963267948966};
};

struct ScnExportResult {
    int tracks{0};
    std::string first_track_name;
    double origin_east{0.0};
    double origin_north{0.0};
};

[[nodiscard]] ScnExportResult export_scn(
    const std::vector<maj0sted::editor::NiweletaPolys>& solved,
    const ScnExportOptions& options, std::ostream& out);

[[nodiscard]] ScnExportOptions resolve_scn_origin(
    const std::vector<maj0sted::editor::NiweletaPolys>& solved,
    ScnExportOptions options);

}  // namespace maj0sted::io
