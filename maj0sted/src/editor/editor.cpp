#include "maj0sted/editor/editor.hpp"

#include <cmath>
#include <cstdint>
#include <optional>
#include <utility>
#include <variant>
#include <vector>

#include "maj0sted/domain/fitting/gap_fitter.hpp"
#include "maj0sted/domain/geometry/segment_layout.hpp"
#include "maj0sted/domain/geometry/turnout.hpp"
#include "maj0sted/maj0sted.hpp"
#include "maj0sted/render/rail_renderer.hpp"
#include "maj0sted/render/track_renderer.hpp"

// This file is orchestration only: it turns the editor's request DTOs into
// domain calls, samples the solved geometry into rendering-neutral centreline
// elements, and drives a render::TrackRenderer to produce the drawable output.
// All fitting business logic lives in maj0sted::domain::GapFitter; all rail
// geometry lives in maj0sted::render::RailRenderer. Nothing here decides how a
// curve is fitted or how it is drawn.
namespace maj0sted::editor {

using namespace maj0sted::domain;
using maj0sted::domain::geometry::layout_segment;
using maj0sted::domain::geometry::Pose;
using maj0sted::domain::geometry::XY;

namespace {

std::optional<Straight> make_straight(const StraightSpec& s, std::uint64_t id) {
    try {
        return Straight{StraightId{id}, CartesianPosition{s.x1, s.y1},
                        CartesianPosition{s.x2, s.y2}};
    } catch (...) {
        return std::nullopt;  // degenerate (zero length)
    }
}

// Maps an editor gap-fit DTO onto a domain connection request. Returns nullopt
// for "no fit" (mode 0) or an unknown mode.
std::optional<GapConnectionRequest> to_request(const GapFit& fit) {
    GapConnectionRequest request;
    switch (fit.mode) {
        case 1:  // plain arc
        // Legacy explicit nawrót (mode 4): treated as a plain-arc request; a
        // genuine 180° reversal is then detected from the geometry in GapFitter.
        case 4:
            request.mode = GapConnectionMode::Arc;
            request.radius = fit.radius;
            return request;
        case 2:  // legacy "łuk + klotoidy" -> compound with a single eased arc
            request.mode = GapConnectionMode::Compound;
            request.compound.entry_transition = fit.transition;
            request.compound.exit_transition = fit.transition;
            request.compound.arcs.push_back(CompoundGapArc{fit.radius, 0.0, 0.0});
            return request;
        case 3:  // unified compound: any number of arcs eased by clothoids
            request.mode = GapConnectionMode::Compound;
            request.compound.entry_transition = fit.entry_t;
            request.compound.exit_transition = fit.exit_t;
            if (!fit.arcs.empty()) {
                for (const auto& a : fit.arcs) {
                    request.compound.arcs.push_back(
                        CompoundGapArc{a.radius, a.length, a.transition_to_next});
                }
            } else if (fit.r2 > 0.0) {  // legacy two-arc basket
                request.compound.arcs.push_back(
                    CompoundGapArc{fit.r1, fit.arc1_len, fit.between});
                request.compound.arcs.push_back(CompoundGapArc{fit.r2, 0.0, 0.0});
            } else {  // legacy single arc
                request.compound.arcs.push_back(CompoundGapArc{fit.r1, 0.0, 0.0});
            }
            return request;
        default:
            return std::nullopt;
    }
}

double radius_of(double curvature) {
    return std::abs(curvature) > 1e-12 ? 1.0 / std::abs(curvature) : 0.0;
}

// Describes one domain plan element as sampling parameters for layout_segment,
// returning its render kind. A circular arc has constant curvature; a clothoid
// varies linearly; a straight has zero curvature.
render::ElementKind sample_params(const PlanElement& element, double& k0, double& k1,
                                  double& length) {
    if (const auto* arc = std::get_if<CircularArc>(&element)) {
        const double sign = arc->direction() == TurnDirection::Left ? 1.0 : -1.0;
        k0 = k1 = sign / arc->radius().metres();
        length = arc->length() ? arc->length()->metres() : 0.0;
        return render::ElementKind::Arc;
    }
    const auto& transition = std::get<TransitionCurve>(element);
    const double sign = transition.direction() == TurnDirection::Left ? 1.0 : -1.0;
    k0 = transition.start_radius() ? sign / transition.start_radius()->metres() : 0.0;
    k1 = transition.end_radius() ? sign / transition.end_radius()->metres() : 0.0;
    length = transition.length().metres();
    return render::ElementKind::Transition;
}

// Samples the solved niweleta (trimmed straights + fitted curves) into
// rendering-neutral centreline elements, and records the trimmed straights.
std::vector<render::CentrelineElement> sample_centrelines(
    const NiweletaSpec& spec, const std::vector<std::optional<Straight>>& straights,
    const std::vector<std::optional<FitResult>>& fits,
    std::vector<StraightSpec>& rendered_straights) {
    std::vector<render::CentrelineElement> axis;
    const std::size_t n = straights.size();
    for (std::size_t i = 0; i < n; ++i) {
        if (!straights[i]) continue;

        // Trim the straight to the tangent points of its adjacent fits.
        CartesianPosition a = straights[i]->start();
        CartesianPosition b = straights[i]->end();
        if (i > 0 && fits[i - 1]) a = fits[i - 1]->tangent_out;
        if (i < fits.size() && fits[i]) b = fits[i]->tangent_in;
        rendered_straights[i] =
            StraightSpec{a.x(), a.y(), b.x(), b.y(), spec.straights[i].hidden};

        if (!spec.straights[i].hidden) {
            render::CentrelineElement straight_element;
            straight_element.kind = render::ElementKind::Straight;
            straight_element.straight_index = static_cast<int>(i);
            straight_element.length = std::hypot(b.x() - a.x(), b.y() - a.y());
            straight_element.points = {render::Point{a.x(), a.y()},
                                       render::Point{b.x(), b.y()}};
            axis.push_back(std::move(straight_element));
        }

        if (i < fits.size() && fits[i]) {
            Pose pose{fits[i]->tangent_in.x(), fits[i]->tangent_in.y(),
                      std::sin(straights[i]->azimuth().radians()),
                      std::cos(straights[i]->azimuth().radians())};
            int element_index = 0;
            for (const auto& element : fits[i]->curve) {
                double k0 = 0.0, k1 = 0.0, length = 0.0;
                render::CentrelineElement curve_element;
                curve_element.kind = sample_params(element, k0, k1, length);
                std::vector<XY> pts;
                pose = layout_segment(k0, k1, length, pose, &pts);
                curve_element.points.reserve(pts.size());
                for (const auto& p : pts) {
                    curve_element.points.push_back(render::Point{p.x, p.y});
                }
                curve_element.gap = static_cast<int>(i);
                curve_element.element_index = element_index++;
                curve_element.length = length;
                curve_element.radius_start = radius_of(k0);
                curve_element.radius_end = radius_of(k1);
                axis.push_back(std::move(curve_element));
            }
        }
    }
    return axis;
}

// Renders the centreline elements to PlanPolylines through the renderer
// interface. A different render::TrackRenderer could be dropped in here without
// touching anything above.
std::vector<PlanPolyline> render_rails(
    const std::vector<render::CentrelineElement>& axis) {
    render::RailRenderer rail_renderer;
    render::TrackRenderer& renderer = rail_renderer;
    for (const auto& element : axis) renderer.add(element);

    std::vector<PlanPolyline> polylines;
    polylines.reserve(rail_renderer.rails().size());
    for (const auto& rail : rail_renderer.rails()) {
        PlanPolyline poly;
        poly.kind = static_cast<int>(rail.kind);
        poly.straight_index = rail.straight_index;
        poly.gap = rail.gap;
        poly.element_index = rail.element_index;
        poly.length = rail.length;
        poly.radius_start = rail.radius_start;
        poly.radius_end = rail.radius_end;
        poly.points.reserve(rail.points.size());
        for (const auto& p : rail.points) poly.points.push_back(PlanPoint{p.x, p.y});
        polylines.push_back(std::move(poly));
    }
    return polylines;
}

NiweletaPolys build(const NiweletaSpec& spec) {
    const std::size_t n = spec.straights.size();
    std::vector<std::optional<Straight>> straights(n);
    for (std::size_t i = 0; i < n; ++i) {
        straights[i] = make_straight(spec.straights[i], i + 1);
    }

    NiweletaPolys result;
    result.rendered_straights = spec.straights;

    // 1. Business: connect each requested gap. The domain decides the curve type
    //    and the radius actually used; a gap that admits no fit is simply left
    //    out (the library, not the host, turns a failed fit into "no fit"). Only
    //    fits that succeeded are echoed in applied_fits for the host to mirror.
    std::vector<std::optional<FitResult>> fits(n == 0 ? 0 : n - 1);
    for (const auto& gap : spec.fits) {
        const int g = gap.gap;
        if (g < 0 || static_cast<std::size_t>(g) + 1 >= n) continue;
        if (!straights[g] || !straights[g + 1]) continue;
        const auto request = to_request(gap);
        if (!request) continue;
        const auto connection =
            GapFitter::connect(*straights[g], *straights[g + 1], *request);
        if (!connection) continue;
        fits[g] = connection->fit;
        GapFit applied = gap;
        applied.radius = connection->applied_radius;
        result.applied_fits.push_back(applied);
    }

    // 2. Geometry: sample trimmed straights and fitted curves into a neutral,
    //    renderer-agnostic centreline.
    const auto axis =
        sample_centrelines(spec, straights, fits, result.rendered_straights);

    // Flatten the axis into one polyline for station lookups, dropping the join
    // points each element repeats at its start.
    for (const auto& element : axis) {
        for (const auto& p : element.points) {
            if (!result.centreline.empty()) {
                const auto& last = result.centreline.back();
                if (std::hypot(p.x - last.x, p.y - last.y) < 1e-9) continue;
            }
            result.centreline.push_back(PlanPoint{p.x, p.y});
        }
    }

    // 3. Presentation: hand the centreline to a renderer.
    result.polylines = render_rails(axis);
    return result;
}

// The pose (position + unit heading) at arc length @p station along a centreline
// polyline. Clamped to the ends; heading comes from the segment the station falls
// on. Returns false only for a centreline too short to have a direction.
bool pose_at(const std::vector<PlanPoint>& centreline, double station, Pose& out) {
    if (centreline.size() < 2) return false;

    double travelled = 0.0;
    for (std::size_t i = 0; i + 1 < centreline.size(); ++i) {
        const auto& a = centreline[i];
        const auto& b = centreline[i + 1];
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double seg = std::hypot(dx, dy);
        if (seg < 1e-12) continue;

        // the last segment also catches any station beyond the centreline's end
        if (station <= travelled + seg || i + 2 == centreline.size()) {
            const double t = std::clamp((station - travelled) / seg, 0.0, 1.0);
            out = Pose{a.x + dx * t, a.y + dy * t, dx / seg, dy / seg};
            return true;
        }
        travelled += seg;
    }
    return false;
}

// Translates the editor's switch DTO into a domain Turnout: a plain-arc curve
// (mode 1) or a basket of arcs eased by clothoids (mode 3). All the turnout
// business — turning to the crossing angle, closing the last arc, the frog rail —
// lives in the domain; nothing here decides geometry.
domain::Turnout to_turnout(const Junction& junction) {
    domain::DivergingCurve curve;
    if (junction.curve.mode == 3 && junction.curve.arcs.size() >= 2) {
        for (const auto& arc : junction.curve.arcs) {
            curve.arcs.push_back(
                domain::TurnoutArc{arc.radius, arc.length, arc.transition_to_next});
        }
        curve.entry_transition = junction.curve.entry_t;
        curve.exit_transition = junction.curve.exit_t;
    } else {
        curve.arcs.push_back(domain::TurnoutArc{junction.curve.radius, 0.0, 0.0});
    }
    return domain::Turnout{
        domain::CrossingMark{junction.crossing_n},
        junction.side == 0 ? domain::DivergeSide::Left : domain::DivergeSide::Right,
        std::move(curve), junction.length};
}

// The drawable geometry of one switch. @p station is the switch's start on the
// through track (początek rozjazdu) — the point the user clicks. The diverging
// path and frog come from the domain; here we only place them on the through
// track and sample them into rails.
JunctionGeom solve_junction(const Junction& junction,
                            const std::vector<NiweletaPolys>& solved) {
    JunctionGeom geom;
    if (junction.through < 0 ||
        static_cast<std::size_t>(junction.through) >= solved.size()) {
        return geom;
    }

    Pose start{};
    if (!pose_at(solved[junction.through].centreline, junction.station, start)) {
        return geom;
    }
    // a trailing switch opens against the running direction: flip the heading
    if (!junction.facing) {
        start.hx = -start.hx;
        start.hy = -start.hy;
    }

    const auto turnout = domain::lay_turnout(start, to_turnout(junction));
    if (!turnout.valid) return geom;

    // sample the domain segments into drawable rails
    std::vector<render::CentrelineElement> axis;
    Pose pose = start;
    for (const auto& segment : turnout.path) {
        std::vector<XY> pts;
        render::CentrelineElement element;
        element.kind =
            std::abs(segment.k1 - segment.k0) >= 1e-12 ? render::ElementKind::Transition
            : std::abs(segment.k0) >= 1e-12            ? render::ElementKind::Arc
                                                       : render::ElementKind::Straight;
        pose = layout_segment(segment.k0, segment.k1, segment.length, pose, &pts);
        element.points.reserve(pts.size());
        for (const auto& p : pts) element.points.push_back(render::Point{p.x, p.y});
        element.length = segment.length;
        element.radius_start = radius_of(segment.k0);
        element.radius_end = radius_of(segment.k1);
        axis.push_back(std::move(element));
    }

    geom.valid = true;
    geom.px = start.x;
    geom.py = start.y;
    geom.fx = turnout.frog.x;
    geom.fy = turnout.frog.y;
    geom.fhx = turnout.frog.hx;
    geom.fhy = turnout.frog.hy;
    geom.tangent_front = turnout.tangent_front;
    geom.tangent_back = turnout.tangent_back;
    geom.length = turnout.diverging_length;
    geom.polylines = render_rails(axis);
    return geom;
}

}  // namespace

std::vector<NiweletaPolys> solve_project(const std::vector<NiweletaSpec>& niwelety) {
    std::vector<NiweletaPolys> result;
    result.reserve(niwelety.size());
    for (const auto& spec : niwelety) result.push_back(build(spec));
    return result;
}

LayoutSolution solve_layout(std::vector<NiweletaSpec> niwelety,
                            const std::vector<Junction>& junctions) {
    // Pass 1: solve unpinned, to read the through centrelines the frogs sit on.
    auto solved = solve_project(niwelety);

    // Lock each branch's leaving straight to the frog: it starts at the frog and
    // runs along the frog heading, so the branch is tangent to the switch curve
    // no matter what the user draws. Only its length is theirs — the projection of
    // wherever they dragged its far end onto the frog heading. Because the first
    // straight is pinned tangent, any fit in the branch's first gap (a plain arc
    // or a łuk koszowy) meets the switch tangentially by construction.
    for (const auto& junction : junctions) {
        const auto geom = solve_junction(junction, solved);
        if (!geom.valid) continue;
        if (junction.branch < 0 ||
            static_cast<std::size_t>(junction.branch) >= niwelety.size()) {
            continue;
        }
        auto& branch = niwelety[junction.branch];
        if (branch.straights.empty()) continue;
        auto& first = branch.straights.front();
        double length = (first.x2 - geom.fx) * geom.fhx + (first.y2 - geom.fy) * geom.fhy;
        if (!(length > 1.0)) length = 1.0;  // keep a usable minimum, also catches NaN
        first.x1 = geom.fx;
        first.y1 = geom.fy;
        first.x2 = geom.fx + geom.fhx * length;
        first.y2 = geom.fy + geom.fhy * length;
    }

    // Pass 2: solve with the branches pinned, then read the switches off the
    // final centrelines (the through track may itself be a pinned branch).
    LayoutSolution out;
    out.niwelety = solve_project(niwelety);
    out.junctions.reserve(junctions.size());
    for (const auto& junction : junctions) {
        out.junctions.push_back(solve_junction(junction, out.niwelety));
    }
    return out;
}

}  // namespace maj0sted::editor
