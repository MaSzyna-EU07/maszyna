#include "maj0sted/web/editor.hpp"

#include <cmath>
#include <cstdint>
#include <optional>
#include <utility>
#include <variant>
#include <vector>

#include "maj0sted/domain/fitting/gap_fitter.hpp"
#include "maj0sted/domain/geometry/segment_layout.hpp"
#include "maj0sted/maj0sted.hpp"
#include "maj0sted/render/rail_renderer.hpp"
#include "maj0sted/render/track_renderer.hpp"

// This file is orchestration only: it turns the editor's request DTOs into
// domain calls, samples the solved geometry into rendering-neutral centreline
// elements, and drives a render::TrackRenderer to produce the drawable output.
// All fitting business logic lives in maj0sted::domain::GapFitter; all rail
// geometry lives in maj0sted::render::RailRenderer. Nothing here decides how a
// curve is fitted or how it is drawn.
namespace maj0sted::web {

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

// Renders the centreline elements to WebPolylines through the renderer
// interface. A different render::TrackRenderer could be dropped in here without
// touching anything above.
std::vector<WebPolyline> render_rails(
    const std::vector<render::CentrelineElement>& axis) {
    render::RailRenderer rail_renderer;
    render::TrackRenderer& renderer = rail_renderer;
    for (const auto& element : axis) renderer.add(element);

    std::vector<WebPolyline> polylines;
    polylines.reserve(rail_renderer.rails().size());
    for (const auto& rail : rail_renderer.rails()) {
        WebPolyline poly;
        poly.kind = static_cast<int>(rail.kind);
        poly.straight_index = rail.straight_index;
        poly.gap = rail.gap;
        poly.element_index = rail.element_index;
        poly.length = rail.length;
        poly.radius_start = rail.radius_start;
        poly.radius_end = rail.radius_end;
        poly.points.reserve(rail.points.size());
        for (const auto& p : rail.points) poly.points.push_back(WebPoint{p.x, p.y});
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
            result.centreline.push_back(WebPoint{p.x, p.y});
        }
    }

    // 3. Presentation: hand the centreline to a renderer.
    result.polylines = render_rails(axis);
    return result;
}

// The pose (position + unit heading) at arc length @p station along a centreline
// polyline. Clamped to the ends; heading comes from the segment the station falls
// on. Returns false only for a centreline too short to have a direction.
bool pose_at(const std::vector<WebPoint>& centreline, double station, Pose& out) {
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

// Samples a fitted curve chain into centreline elements, starting from @p start.
// Same layout the gap fits go through, so the switch curve is drawn identically.
std::vector<render::CentrelineElement> sample_curve(
    const std::vector<PlanElement>& curve, Pose start) {
    std::vector<render::CentrelineElement> axis;
    Pose pose = start;
    int element_index = 0;
    for (const auto& element : curve) {
        double k0 = 0.0, k1 = 0.0, length = 0.0;
        render::CentrelineElement out;
        out.kind = sample_params(element, k0, k1, length);
        std::vector<XY> pts;
        pose = layout_segment(k0, k1, length, pose, &pts);
        out.points.reserve(pts.size());
        for (const auto& p : pts) out.points.push_back(render::Point{p.x, p.y});
        out.element_index = element_index++;
        out.length = length;
        out.radius_start = radius_of(k0);
        out.radius_end = radius_of(k1);
        axis.push_back(std::move(out));
    }
    return axis;
}

// The diverging curve and frog of one switch, from the solved through niweleta.
// The switch's theoretical point sits at @p station on the through track; two
// tangent lines cross there (the through heading and that heading turned by the
// crossing angle), and the internal curve is fitted to round the corner between
// them — a plain arc, or a compound basket, whatever the switch asks for. The
// branch begins where the curve ends.
JunctionGeom solve_junction(const Junction& junction,
                            const std::vector<NiweletaPolys>& solved) {
    JunctionGeom geom;
    if (junction.through < 0 ||
        static_cast<std::size_t>(junction.through) >= solved.size()) {
        return geom;
    }
    if (junction.crossing_n <= 0.0) return geom;

    Pose theoretical{};
    if (!pose_at(solved[junction.through].centreline, junction.station,
                 theoretical)) {
        return geom;
    }

    // a trailing switch opens against the running direction: flip the heading
    double thx = theoretical.hx;
    double thy = theoretical.hy;
    if (!junction.facing) {
        thx = -thx;
        thy = -thy;
    }

    // the diverging tangent line: the through heading turned by the crossing
    // angle, left (positive) or right (negative)
    const double angle =
        (junction.side == 0 ? 1.0 : -1.0) * std::atan(1.0 / junction.crossing_n);
    const double dhx = thx * std::cos(angle) - thy * std::sin(angle);
    const double dhy = thx * std::sin(angle) + thy * std::cos(angle);

    // two long guide straights meeting at the theoretical point X; the fitter
    // rounds the corner and hands back the tangent points that become the switch
    // start and the frog. long enough that the tangent points land on them.
    const double guide = 5000.0;
    const CartesianPosition x{theoretical.x, theoretical.y};
    Straight entry{CartesianPosition{x.x() - thx * guide, x.y() - thy * guide}, x};
    Straight exit{x, CartesianPosition{x.x() + dhx * guide, x.y() + dhy * guide}};

    const auto request = to_request(junction.curve);
    if (!request) return geom;  // a switch with no curve is not a switch
    const auto connection = GapFitter::connect(entry, exit, *request);
    if (!connection) return geom;

    const FitResult& fit = connection->fit;
    const CartesianPosition s = fit.tangent_in;   // switch start
    const CartesianPosition f = fit.tangent_out;  // frog

    geom.valid = true;
    geom.px = s.x();
    geom.py = s.y();
    geom.fx = f.x();
    geom.fy = f.y();
    geom.fhx = dhx;
    geom.fhy = dhy;
    // theoretical tangent lengths: from the crossing point to each tangent point
    geom.tangent_front = std::hypot(s.x() - x.x(), s.y() - x.y());
    geom.tangent_back = std::hypot(f.x() - x.x(), f.y() - x.y());

    const auto axis = sample_curve(fit.curve, Pose{s.x(), s.y(), thx, thy});
    for (const auto& element : axis) geom.length += element.length;
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

}  // namespace maj0sted::web
