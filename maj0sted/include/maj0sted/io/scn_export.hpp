#pragma once

#include <ostream>
#include <string>

#include "maj0sted/editor/solution.hpp"

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

/// Writes the solved layout as MaSzyna scenery. The axis comes straight out of
/// the solution: the exporter used to average the two drawn rails back together
/// to recover it, which put the whole geometry through the renderer and back.
///
/// Height is still a constant and both roll slots are still zero — the vertical
/// alignment is a separate piece of work, and nothing in the editor produces one
/// yet.
[[nodiscard]] ScnExportResult export_scn(const editor::Solution& solution,
                                         const ScnExportOptions& options,
                                         std::ostream& out);

[[nodiscard]] ScnExportOptions resolve_scn_origin(const editor::Solution& solution,
                                                  ScnExportOptions options);

}  // namespace maj0sted::io
