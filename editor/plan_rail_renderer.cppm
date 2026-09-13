/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/
module;
#include <cstdint>
#include <vector>

export module eu07.editor.plan_rail_renderer;
export import eu07.editor.plan_track_renderer;

export {

namespace editor::plan::render {

/// A drawable polyline produced by RailRenderer: one rail, carrying the logical
/// metadata of the centreline element it came from so the host can colour and
/// hit-test it.
struct RailPolyline {
    ElementKind kind{ElementKind::Straight};
    std::vector<Point> points;
    std::uint32_t element_id{0};
    std::uint32_t owner_id{0};
    bool from_turnout{false};
    double length{0.0};
    double radius_start{0.0};
    double radius_end{0.0};
};

/// Renders each centreline as two parallel rails a fixed gauge apart — the track
/// is drawn as its rails, not its axis. A concrete TrackRenderer; substituting a
/// different one (single-axis, SVG, ...) needs no change to the solver.
class RailRenderer : public TrackRenderer {
public:
    /// @param gauge_metres distance between the two rails (default 1.5 m).
    explicit RailRenderer(double gauge_metres = 1.5) noexcept;

    void add(const CentrelineElement& element) override;

    [[nodiscard]] const std::vector<RailPolyline>& rails() const noexcept {
        return rails_;
    }

private:
    double half_gauge_;
    std::vector<RailPolyline> rails_;
};

}  // namespace editor::plan::render

}  // export
