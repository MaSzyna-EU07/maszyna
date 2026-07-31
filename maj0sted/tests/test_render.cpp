#include <cmath>

#include "check.hpp"
#include "maj0sted/io/svg_writer.hpp"
#include "maj0sted/maj0sted.hpp"
#include "maj0sted/render/plan_sampler.hpp"

using namespace maj0sted::domain;

namespace {

void samples_one_polyline_per_element() {
    Niweleta n{NiweletaId{1}};
    const StraightId a =
        n.add_straight(CartesianPosition{0.0, 0.0}, CartesianPosition{100.0, 0.0});
    const StraightId b = n.add_straight(CartesianPosition{100.0, 0.0},
                                        CartesianPosition{100.0, 100.0},
                                        JointContinuity::AzimuthBreak);
    const FitResult fit = FittingService::fit_between_straights(
        n, FitParameters{.entry = a,
                         .exit = b,
                         .radius = Radius::from_metres(40.0),
                         .transition_length = Length::from_metres(10.0)});
    n.apply_fit(fit);  // straight -> KP -> arc -> KP -> straight (5 elements)

    const auto scene = maj0sted::render::sample(n.plan());
    CHECK(scene.polylines.size() == n.plan().size());
    CHECK(scene.polylines.size() == 5);
    CHECK(!scene.empty);

    // First polyline is the (straight) entry, drawn from its anchored endpoints.
    const auto& first = scene.polylines.front();
    CHECK(first.kind == maj0sted::render::ElementKind::Straight);
    CHECK(first.points.size() == 2);
    CHECK(std::abs(first.points.front().x - 0.0) < 1e-9);

    // Curves are sampled into many points.
    CHECK(scene.polylines[2].kind == maj0sted::render::ElementKind::Arc);
    CHECK(scene.polylines[2].points.size() > 2);
}

void curve_geometry_is_continuous() {
    // The sampled points should join up (each element starts where the previous
    // ended) for a properly fitted, tangent-continuous plan.
    Niweleta n{NiweletaId{2}};
    const StraightId a =
        n.add_straight(CartesianPosition{0.0, 0.0}, CartesianPosition{100.0, 0.0});
    const StraightId b = n.add_straight(CartesianPosition{100.0, 0.0},
                                        CartesianPosition{100.0, 100.0},
                                        JointContinuity::AzimuthBreak);
    n.apply_fit(FittingService::fit_between_straights(
        n, FitParameters{.entry = a, .exit = b, .radius = Radius::from_metres(40.0)}));

    const auto scene = maj0sted::render::sample(n.plan());
    for (std::size_t i = 1; i < scene.polylines.size(); ++i) {
        const auto& prev_end = scene.polylines[i - 1].points.back();
        const auto& next_start = scene.polylines[i].points.front();
        CHECK(std::abs(prev_end.x - next_start.x) < 1e-6);
        CHECK(std::abs(prev_end.y - next_start.y) < 1e-6);
    }
}

void svg_is_produced() {
    Niweleta n{NiweletaId{3}};
    n.add_straight(CartesianPosition{0.0, 0.0}, CartesianPosition{50.0, 0.0});
    const auto svg = maj0sted::io::to_svg(maj0sted::render::sample(n.plan()));
    CHECK(svg.rfind("<svg", 0) == 0);
    CHECK(svg.find("<polyline") != std::string::npos);
}

}  // namespace

int main() {
    RUN(samples_one_polyline_per_element);
    RUN(curve_geometry_is_continuous);
    RUN(svg_is_produced);
    return REPORT();
}
