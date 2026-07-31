#include "maj0sted/web/ribbon.hpp"

#include <cmath>

#include "maj0sted/domain/geometry/segment_layout.hpp"

namespace maj0sted::web {

using maj0sted::domain::geometry::layout_segment;
using maj0sted::domain::geometry::Pose;
using maj0sted::domain::geometry::XY;

std::vector<WebPolyline> solve_ribbon(const RibbonRequest& request) {
    std::vector<WebPolyline> result;
    result.reserve(request.elements.size());

    Pose pose{request.x0, request.y0, std::sin(request.az0), std::cos(request.az0)};

    for (const auto& element : request.elements) {
        double k0 = 0.0;
        double k1 = 0.0;
        double length = 0.0;
        int kind = element.kind;

        if (element.kind == 1) {  // arc
            const double sign = element.dir >= 0 ? 1.0 : -1.0;
            const double k = element.radius > 0.0 ? sign / element.radius : 0.0;
            k0 = k;
            k1 = k;
            length = element.radius * element.angle;
        } else if (element.kind == 2) {  // spiral (clothoid)
            const double sign = element.dir >= 0 ? 1.0 : -1.0;
            k0 = element.radius_start > 0.0 ? sign / element.radius_start : 0.0;
            k1 = element.radius > 0.0 ? sign / element.radius : 0.0;
            length = element.length;
        } else {  // line
            kind = 0;
            length = element.length;
        }

        std::vector<XY> points;
        pose = layout_segment(k0, k1, length, pose, &points);

        WebPolyline polyline;
        polyline.kind = kind;
        polyline.points.reserve(points.size());
        for (const auto& p : points) {
            polyline.points.push_back(WebPoint{p.x, p.y});
        }
        result.push_back(std::move(polyline));
    }
    return result;
}

}  // namespace maj0sted::web
