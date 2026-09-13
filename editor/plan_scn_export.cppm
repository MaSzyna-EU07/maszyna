/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/
module;
#include <ostream>
#include <string>
#include <vector>

export module eu07.editor.plan_scn_export;
export import eu07.editor.plan_model;
export import eu07.editor.plan_solution;

export {

namespace editor::plan::io {

struct ScnExportOptions {
    double origin_east{0.0};
    double origin_north{0.0};
    double rail_y{0.2};
    /// How far one cubic may be asked to turn. A cubic stands in for an arc to
    /// within about 2,7·10⁻⁴ R over a quarter circle, which is centimetres on a
    /// main-line radius; a quarter of that turn is a thousandth of a millimetre
    /// on anything a railway is built to.
    double max_arc_angle{0.4};
    /// Emit FirstInit and a lone locomotive on the first track written, so the
    /// scenery can be loaded and driven without editing it by hand.
    bool trainset{true};
    /// How far along that track it stands, metres. Far enough in that the
    /// vehicle is on the track and not hanging off its end.
    double trainset_offset{20.0};
    /// Ask each switch for the game's own sterowanie include: the lantern, the
    /// drive's sound and the events that throw it by name.
    bool switch_control{true};
    /// How near the blades one has to stand for `t` and `shift+t` to throw the
    /// switch, metres. Kept well under the distance between two of them, or one
    /// key would throw the lot.
    double switch_key_reach{12.0};
};

struct ScnExportResult {
    int tracks{0};    ///< `track normal` nodes written
    int switches{0};  ///< `track switch` nodes written
    std::string first_track_name;
    double origin_east{0.0};
    double origin_north{0.0};
    /// What the scenery does not say as well as the drawing did. The export is
    /// written anyway: these are read by whoever asked for it, never fixed up
    /// behind their back.
    std::vector<std::string> warnings;
};

/// Writes the solved layout as MaSzyna scenery.
///
/// The axis comes straight out of the solution: the exporter used to average the
/// two drawn rails back together to recover it, which put the whole geometry
/// through the renderer and back.
///
/// A turnout is a `track switch` node, which is two Bézier curves leaving one
/// point, so the through track is cut around it: what runs from PR to KR belongs
/// to the switch, and the rest of the track is written as ordinary pieces either
/// side. The simulator wires all of it together by nothing but the endpoints
/// standing within two centimetres of each other, so every piece is cut from the
/// same laid geometry and the shared points are written out identical.
///
/// The document is needed for what is authored and not derived: the names the
/// nodes are written under, and which track each turnout stands on.
///
/// Height is still a constant and both roll slots are still zero — the vertical
/// alignment is a separate piece of work, and nothing in the editor produces one
/// yet.
[[nodiscard]] ScnExportResult export_scn(const plan::Document& document,
                                         const plan::Solution& solution,
                                         const ScnExportOptions& options,
                                         std::ostream& out);

[[nodiscard]] ScnExportOptions resolve_scn_origin(const plan::Solution& solution,
                                                  ScnExportOptions options);

}  // namespace editor::plan::io

}  // export
