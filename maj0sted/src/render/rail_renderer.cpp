#include "maj0sted/render/rail_renderer.hpp"

#include <cmath>
#include <cstddef>
#include <utility>

namespace maj0sted::render {

namespace {

// Offsets a centreline polyline sideways by @p dist metres (signed) along the
// per-vertex normal. The vertex tangent uses a central difference of the
// neighbours (forward/backward at the ends); the left normal of (dx, dy) is
// (-dy, dx), matching the project's (east=x, north=y) convention.
std::vector<Point> offset(const std::vector<Point>& pts, double dist) {
    const std::size_t m = pts.size();
    std::vector<Point> out(m);
    for (std::size_t i = 0; i < m; ++i) {
        double dx = 1.0, dy = 0.0;
        if (m >= 2) {
            if (i == 0) {
                dx = pts[1].x - pts[0].x;
                dy = pts[1].y - pts[0].y;
            } else if (i + 1 == m) {
                dx = pts[i].x - pts[i - 1].x;
                dy = pts[i].y - pts[i - 1].y;
            } else {
                dx = pts[i + 1].x - pts[i - 1].x;
                dy = pts[i + 1].y - pts[i - 1].y;
            }
        }
        const double len = std::hypot(dx, dy);
        const double nx = len > 0.0 ? -dy / len : 0.0;
        const double ny = len > 0.0 ? dx / len : 0.0;
        out[i] = Point{pts[i].x + nx * dist, pts[i].y + ny * dist};
    }
    return out;
}

}  // namespace

RailRenderer::RailRenderer(double gauge_metres) noexcept
    : half_gauge_{gauge_metres * 0.5} {}

void RailRenderer::add(const CentrelineElement& element) {
    for (const double side : {half_gauge_, -half_gauge_}) {
        RailPolyline rail;
        rail.kind = element.kind;
        rail.straight_index = element.straight_index;
        rail.gap = element.gap;
        rail.element_index = element.element_index;
        rail.length = element.length;
        rail.radius_start = element.radius_start;
        rail.radius_end = element.radius_end;
        rail.points = offset(element.points, side);
        rails_.push_back(std::move(rail));
    }
}

}  // namespace maj0sted::render
