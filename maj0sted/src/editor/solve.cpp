#include "maj0sted/editor/solve.hpp"

#include <optional>

#include "maj0sted/maj0sted.hpp"
#include "maj0sted/render/plan_sampler.hpp"

namespace maj0sted::editor {

using namespace maj0sted::domain;

namespace {

std::optional<Length> optional_length(double metres) {
    if (metres > 0.0) return Length::from_metres(metres);
    return std::nullopt;
}

FitResult make_fit(const Niweleta& niweleta, StraightId entry, StraightId exit,
                   const SolveRequest& request) {
    if (request.mode == 2) {
        return FittingService::fit_compound(
            niweleta,
            CompoundFitParameters{
                .entry = entry,
                .exit = exit,
                .entry_transition = optional_length(request.entry_t),
                .exit_transition = optional_length(request.exit_t),
                .arcs = {CompoundArc{Radius::from_metres(request.r1),
                                     Length::from_metres(request.arc1_len),
                                     optional_length(request.between)},
                         CompoundArc{Radius::from_metres(request.r2)}}});
    }
    return FittingService::fit_between_straights(
        niweleta,
        FitParameters{.entry = entry,
                      .exit = exit,
                      .radius = Radius::from_metres(request.radius),
                      .transition_length = request.mode == 1
                                               ? optional_length(request.transition)
                                               : std::nullopt});
}

std::vector<PlanPolyline> to_polylines(const render::Scene& scene) {
    std::vector<PlanPolyline> result;
    result.reserve(scene.polylines.size());
    for (const auto& polyline : scene.polylines) {
        PlanPolyline out;
        out.kind = static_cast<int>(polyline.kind);
        out.points.reserve(polyline.points.size());
        for (const auto& p : polyline.points) {
            out.points.push_back(PlanPoint{p.x, p.y});
        }
        result.push_back(std::move(out));
    }
    return result;
}

}  // namespace

std::vector<PlanPolyline> solve_scene(const SolveRequest& request) {
    Niweleta niweleta{NiweletaId{1}};
    const CartesianPosition a{request.ax, request.ay};
    const CartesianPosition b{request.bx, request.by};
    const CartesianPosition c{request.cx, request.cy};

    StraightId entry;
    try {
        entry = niweleta.add_straight(a, b);
    } catch (...) {
        return {};  // degenerate entry straight
    }

    bool has_exit = false;
    StraightId exit;
    try {
        exit = niweleta.add_straight(b, c, JointContinuity::AzimuthBreak);
        has_exit = true;
    } catch (...) {
        try {
            exit = niweleta.add_straight(b, c, JointContinuity::Tangent);
            has_exit = true;
        } catch (...) {
        }
    }

    if (has_exit) {
        try {
            niweleta.apply_fit(make_fit(niweleta, entry, exit, request));
        } catch (...) {
            // Parameters don't admit a fit — show the bare straights instead.
        }
    }

    return to_polylines(render::sample(niweleta.plan()));
}

}  // namespace maj0sted::editor
