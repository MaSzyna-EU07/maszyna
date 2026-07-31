#pragma once

#include <vector>

#include "maj0sted/render/track_renderer.hpp"

namespace maj0sted::render {

/// A drawable polyline produced by RailRenderer: one rail, carrying the logical
/// metadata of the centreline element it came from so the host can colour and
/// hit-test it.
struct RailPolyline {
    ElementKind kind{ElementKind::Straight};
    std::vector<Point> points;
    int straight_index{-1};
    int gap{-1};
    int element_index{-1};
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

}  // namespace maj0sted::render
