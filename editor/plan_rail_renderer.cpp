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
#include <cmath>
#include <cstddef>
#include <utility>

module eu07.editor.plan_rail_renderer;

namespace editor::plan::render {

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
        rail.element_id = element.element_id;
        rail.owner_id = element.owner_id;
        rail.from_turnout = element.from_turnout;
        rail.length = element.length;
        rail.radius_start = element.radius_start;
        rail.radius_end = element.radius_end;
        rail.points = offset(element.points, side);
        rails_.push_back(std::move(rail));
    }
}

}  // namespace editor::plan::render
