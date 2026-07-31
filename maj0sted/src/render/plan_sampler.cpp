#include "maj0sted/render/plan_sampler.hpp"

#include <algorithm>
#include <cmath>
#include <variant>

#include "maj0sted/domain/geometry/segment_layout.hpp"
#include "maj0sted/maj0sted.hpp"

namespace maj0sted::render {

using namespace maj0sted::domain;

namespace {

template <class... Ts>
struct overloaded : Ts... {
    using Ts::operator()...;
};
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

struct Vec {
    double east;
    double north;
};

Vec direction_of(Azimuth azimuth) noexcept {
    return Vec{std::sin(azimuth.radians()), std::cos(azimuth.radians())};
}

// Samples a segment of linearly varying signed curvature (arc when k0 == k1,
// clothoid otherwise), starting at @p pos heading @p heading. Appends points to
// @p out and updates @p pos / @p heading to the segment's end.
// Samples a segment via the shared layout (identical to the fitting solver, so
// the drawn curve coincides exactly with the fitted tangent points).
void sample_segment(double k0, double k1, double length, Vec& pos, Vec& heading,
                    std::vector<Point>& out) {
    geometry::Pose start{pos.east, pos.north, heading.east, heading.north};
    std::vector<geometry::XY> points;
    const geometry::Pose end = geometry::layout_segment(k0, k1, length, start, &points);
    for (const auto& p : points) out.push_back({p.x, p.y});
    pos = Vec{end.x, end.y};
    heading = Vec{end.hx, end.hy};
}

void expand(Scene& scene, const Point& p) {
    if (scene.empty) {
        scene.min_x = scene.max_x = p.x;
        scene.min_y = scene.max_y = p.y;
        scene.empty = false;
        return;
    }
    scene.min_x = std::min(scene.min_x, p.x);
    scene.min_y = std::min(scene.min_y, p.y);
    scene.max_x = std::max(scene.max_x, p.x);
    scene.max_y = std::max(scene.max_y, p.y);
}

void sample_into(const HorizontalAlignment& plan, Scene& scene) {
    Vec pos{0.0, 0.0};
    Vec heading{1.0, 0.0};

    for (const auto& element : plan.elements()) {
        Polyline polyline;
        std::visit(
            overloaded{
                [&](const Straight& s) {
                    polyline.kind = ElementKind::Straight;
                    polyline.points = {{s.start().x(), s.start().y()},
                                       {s.end().x(), s.end().y()}};
                    pos = Vec{s.end().x(), s.end().y()};
                    heading = direction_of(s.azimuth());
                },
                [&](const CircularArc& a) {
                    polyline.kind = ElementKind::Arc;
                    const double sign = a.direction() == TurnDirection::Left ? 1.0 : -1.0;
                    const double k = sign / a.radius().metres();
                    const double length = a.length() ? a.length()->metres() : 0.0;
                    sample_segment(k, k, length, pos, heading, polyline.points);
                },
                [&](const TransitionCurve& t) {
                    polyline.kind = ElementKind::Transition;
                    const double sign = t.direction() == TurnDirection::Left ? 1.0 : -1.0;
                    const double k0 =
                        t.start_radius() ? sign / t.start_radius()->metres() : 0.0;
                    const double k1 =
                        t.end_radius() ? sign / t.end_radius()->metres() : 0.0;
                    sample_segment(k0, k1, t.length().metres(), pos, heading,
                                   polyline.points);
                },
            },
            element);

        for (const auto& p : polyline.points) expand(scene, p);
        scene.polylines.push_back(std::move(polyline));
    }
}

}  // namespace

Scene sample(const HorizontalAlignment& plan) {
    Scene scene;
    sample_into(plan, scene);
    return scene;
}

Scene sample(const MapProject& project) {
    Scene scene;
    for (const auto& niweleta : project.niwelety()) {
        sample_into(niweleta.plan(), scene);
    }
    return scene;
}

}  // namespace maj0sted::render
