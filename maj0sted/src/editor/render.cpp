#include "maj0sted/editor/render.hpp"

#include <cstdint>
#include <utility>

#include "maj0sted/render/rail_renderer.hpp"

namespace maj0sted::editor {
namespace {

render::ElementKind kind_of(Kind kind) {
    switch (kind) {
        case Kind::Line: return render::ElementKind::Straight;
        case Kind::Arc: return render::ElementKind::Arc;
        case Kind::Clothoid: return render::ElementKind::Transition;
    }
    return render::ElementKind::Straight;
}

Kind kind_from(render::ElementKind kind) {
    switch (kind) {
        case render::ElementKind::Straight: return Kind::Line;
        case render::ElementKind::Arc: return Kind::Arc;
        case render::ElementKind::Transition: return Kind::Clothoid;
    }
    return Kind::Line;
}

double radius_of(double curvature) {
    return std::abs(curvature) > 1e-9 ? 1.0 / std::abs(curvature) : 0.0;
}

render::CentrelineElement centreline_of(Kind kind, double k0, double k1, double length,
                                        const std::vector<PlanPoint>& points,
                                        std::uint32_t element_id, std::uint32_t owner_id,
                                        bool from_turnout) {
    render::CentrelineElement element;
    element.kind = kind_of(kind);
    element.element_id = element_id;
    element.owner_id = owner_id;
    element.from_turnout = from_turnout;
    element.length = length;
    element.radius_start = radius_of(k0);
    element.radius_end = radius_of(k1);
    element.points.reserve(points.size());
    for (const auto& point : points) {
        element.points.push_back(render::Point{point.x, point.y});
    }
    return element;
}

Kind kind_of_segment(double k0, double k1) {
    if (std::abs(k0) < 1e-12 && std::abs(k1) < 1e-12) {
        return Kind::Line;
    }
    return std::abs(k1 - k0) < 1e-12 ? Kind::Arc : Kind::Clothoid;
}

}  // namespace

std::vector<PlanPolyline> render_rails(const Solution& solution, double gauge_metres) {
    render::RailRenderer rail_renderer{gauge_metres};
    render::TrackRenderer& renderer = rail_renderer;

    // Owner ids are recorded alongside so a rail can be traced back without
    // widening the render layer's vocabulary.
    for (const auto& track : solution.tracks) {
        for (const auto& element : track.elements) {
            if (element.points.size() < 2) {
                continue;
            }
            renderer.add(centreline_of(element.kind, element.k0, element.k1, element.length,
                                       element.points,
                                       static_cast<std::uint32_t>(element.id),
                                       static_cast<std::uint32_t>(track.id), false));
        }
    }
    for (const auto& turnout : solution.turnouts) {
        if (!turnout.valid) {
            continue;
        }
        std::uint32_t index = 0;
        for (const auto& segment : turnout.path) {
            if (segment.points.size() < 2) {
                ++index;
                continue;
            }
            renderer.add(centreline_of(kind_of_segment(segment.k0, segment.k1), segment.k0,
                                       segment.k1, segment.length, segment.points, index++,
                                       static_cast<std::uint32_t>(turnout.id), true));
        }
    }

    std::vector<PlanPolyline> out;
    out.reserve(rail_renderer.rails().size());
    for (const auto& rail : rail_renderer.rails()) {
        PlanPolyline polyline;
        polyline.kind = kind_from(rail.kind);
        polyline.length = rail.length;
        polyline.radius_start = rail.radius_start;
        polyline.radius_end = rail.radius_end;
        if (rail.from_turnout) {
            polyline.turnout = static_cast<TurnoutId>(rail.owner_id);
            polyline.piece = rail.element_id;
            const SolvedTurnout* owner = find_turnout(solution, polyline.turnout);
            if (owner != nullptr && polyline.piece < owner->path.size()) {
                polyline.part = static_cast<int>(owner->path[polyline.piece].part);
            }
        } else {
            polyline.element = static_cast<ElementId>(rail.element_id);
            polyline.track = static_cast<TrackId>(rail.owner_id);
        }
        polyline.points.reserve(rail.points.size());
        for (const auto& point : rail.points) {
            polyline.points.push_back(PlanPoint{point.x, point.y});
        }
        out.push_back(std::move(polyline));
    }
    return out;
}

}  // namespace maj0sted::editor
