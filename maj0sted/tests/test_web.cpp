#include <cmath>
#include <numbers>

#include "check.hpp"
#include "maj0sted/web/editor.hpp"
#include "maj0sted/web/solve.hpp"

using namespace maj0sted::web;

namespace {

bool has_kind(const std::vector<WebPolyline>& polylines, int kind) {
    for (const auto& p : polylines) {
        if (p.kind == kind) return true;
    }
    return false;
}

void arc_with_transitions_produces_full_chain() {
    SolveRequest r{};
    r.ax = 0.0; r.ay = 0.0;
    r.bx = 300.0; r.by = 0.0;
    r.cx = 300.0; r.cy = 300.0;
    r.mode = 1;
    r.radius = 80.0;
    r.transition = 20.0;

    const auto polylines = solve_scene(r);
    CHECK(!polylines.empty());
    CHECK(has_kind(polylines, 0));  // straight
    CHECK(has_kind(polylines, 1));  // arc
    CHECK(has_kind(polylines, 2));  // transition
    // Each polyline has at least two points.
    for (const auto& p : polylines) CHECK(p.points.size() >= 2);
}

void compound_mode_produces_two_arcs() {
    SolveRequest r{};
    r.ax = 0.0; r.ay = 0.0;
    r.bx = 300.0; r.by = 0.0;
    r.cx = 300.0; r.cy = 300.0;
    r.mode = 2;
    r.r1 = 120.0; r.arc1_len = 60.0; r.between = 25.0; r.r2 = 80.0;
    r.entry_t = 30.0; r.exit_t = 30.0;

    const auto polylines = solve_scene(r);
    int arcs = 0;
    for (const auto& p : polylines) {
        if (p.kind == 1) ++arcs;
    }
    CHECK(arcs == 2);
    CHECK(has_kind(polylines, 2));  // transitions present
}

void degenerate_input_falls_back_to_straights() {
    // Collinear vertices -> no fit; still returns the straights, never throws.
    SolveRequest r{};
    r.ax = 0.0; r.ay = 0.0;
    r.bx = 100.0; r.by = 0.0;
    r.cx = 200.0; r.cy = 0.0;
    r.mode = 0;
    r.radius = 50.0;

    const auto polylines = solve_scene(r);
    CHECK(!polylines.empty());
    CHECK(has_kind(polylines, 0));   // straights
    CHECK(!has_kind(polylines, 1));  // no arc
}

NiweletaSpec short_corner(double radius, double transition = 0.0) {
    NiweletaSpec spec;
    spec.straights = {
        StraightSpec{0.0, 0.0, 10.0, 0.0},
        StraightSpec{10.0, 0.0, 10.0, 10.0},
    };
    spec.fits = {GapFit{.gap = 0,
                        .mode = transition > 0.0 ? 2 : 1,
                        .radius = radius,
                        .transition = transition}};
    return spec;
}

void editor_clamps_oversized_arc_to_fitting_radius() {
    const auto solved = solve_project({short_corner(60.0)});
    CHECK(solved.size() == 1);
    CHECK(solved[0].applied_fits.size() == 1);
    const double applied = solved[0].applied_fits[0].radius;
    CHECK(applied > 9.9);
    CHECK(applied < 10.1);
}

void editor_leaves_gap_unfitted_when_radius_below_floor() {
    // A radius too small for the requested transitions is NEVER enlarged (the
    // fitting rule only ever reduces). When nothing at or below the request fits,
    // the gap is simply left unconnected.
    NiweletaSpec spec;
    spec.straights = {
        StraightSpec{0.0, 0.0, 100.0, 0.0},
        StraightSpec{100.0, 0.0, 100.0, 100.0},
    };
    spec.fits = {GapFit{.gap = 0, .mode = 2, .radius = 1.0, .transition = 20.0}};

    const auto solved = solve_project({spec});
    CHECK(solved.size() == 1);
    CHECK(solved[0].applied_fits.empty());
    CHECK(!has_kind(solved[0].polylines, 1));  // no arc rendered
}

void editor_emits_two_rails_at_standard_gauge() {
    const auto solved = solve_project({short_corner(5.0)});
    CHECK(solved[0].polylines.size() == 6);
    CHECK(solved[0].rendered_straights.size() == 2);
    CHECK(std::abs(solved[0].rendered_straights[0].x2 - 5.0) < 1e-6);
    CHECK(std::abs(solved[0].rendered_straights[1].y1 - 5.0) < 1e-6);
    const auto& left = solved[0].polylines[0].points;
    const auto& right = solved[0].polylines[1].points;
    CHECK(left.size() == right.size());
    CHECK(std::abs(std::hypot(left[0].x - right[0].x, left[0].y - right[0].y) -
                   1.5) < 1e-9);
    CHECK(solved[0].polylines[0].straight_index == 0);
    CHECK(solved[0].polylines[0].gap == -1);
    CHECK(std::abs(solved[0].polylines[0].length - 5.0) < 1e-9);
    const WebPolyline* arc = nullptr;
    for (const auto& polyline : solved[0].polylines) {
        if (polyline.kind == 1) {
            arc = &polyline;
            break;
        }
    }
    CHECK(arc != nullptr);
    CHECK(arc->straight_index == -1);
    CHECK(arc->gap == 0);
    CHECK(arc->element_index == 0);
    CHECK(std::abs(arc->radius_start - 5.0) < 1e-9);
    CHECK(std::abs(arc->radius_end - 5.0) < 1e-9);
    CHECK(std::abs(arc->length - 5.0 * std::numbers::pi / 2.0) < 1e-9);
}

void editor_exposes_transition_radius_range() {
    const auto solved = solve_project({short_corner(5.0, 2.0)});
    const WebPolyline* transition = nullptr;
    for (const auto& polyline : solved[0].polylines) {
        if (polyline.kind == 2) {
            transition = &polyline;
            break;
        }
    }
    CHECK(transition != nullptr);
    CHECK(transition->gap == 0);
    CHECK(std::abs(transition->length - 2.0) < 1e-9);
    CHECK(transition->radius_start == 0.0);
    CHECK(std::abs(transition->radius_end - 5.0) < 1e-9);
}

void editor_leaves_parallel_offset_unfitted() {
    // Two parallel, laterally offset straights admit no plain arc (a single arc
    // cannot connect parallels) and reverse S-curves are never substituted, so the
    // gap stays unconnected rather than inventing a curve.
    NiweletaSpec spec;
    spec.straights = {
        StraightSpec{0.0, 0.0, 200.0, 0.0},
        StraightSpec{0.0, 20.0, 200.0, 20.0},
    };
    spec.fits = {GapFit{.gap = 0, .mode = 1, .radius = 50.0}};

    const auto solved = solve_project({spec});
    CHECK(solved[0].applied_fits.empty());
    CHECK(!has_kind(solved[0].polylines, 1));  // no arc, no reverse pair
}

void editor_honours_requested_radius_via_single_arc() {
    // A requested radius that lands on the straights (extending them toward the
    // corner if needed) is used verbatim as a single arc — never enlarged and
    // never swapped for a reverse pair.
    const double heading = std::numbers::pi / 6.0;
    const double tangent_x = 86.9615242271;
    const double tangent_y = 25.9807621135;
    const double dx = std::cos(heading);
    const double dy = std::sin(heading);
    NiweletaSpec spec;
    spec.straights = {
        StraightSpec{0.0, 0.0, 100.0, 0.0},
        StraightSpec{tangent_x - 30.0 * dx, tangent_y - 30.0 * dy,
                     tangent_x + 70.0 * dx, tangent_y + 70.0 * dy},
    };
    spec.fits = {GapFit{.gap = 0, .mode = 1, .radius = 30.0}};

    const auto solved = solve_project({spec});
    CHECK(solved[0].applied_fits.size() == 1);
    CHECK(std::abs(solved[0].applied_fits[0].radius - 30.0) < 1e-9);
    int arc_rails = 0;
    for (const auto& polyline : solved[0].polylines) {
        if (polyline.kind == 1) ++arc_rails;
    }
    CHECK(arc_rails == 2);  // a single arc (two rails), not a reverse pair
}

void hidden_tangent_guide_fits_curve_without_rendering_track() {
    NiweletaSpec spec = short_corner(5.0);
    spec.straights[1].hidden = true;

    const auto solved = solve_project({spec});
    CHECK(solved[0].applied_fits.size() == 1);
    CHECK(solved[0].rendered_straights[1].hidden);
    int straight_rails = 0;
    int arc_rails = 0;
    for (const auto& polyline : solved[0].polylines) {
        if (polyline.kind == 0) ++straight_rails;
        if (polyline.kind == 1) ++arc_rails;
    }
    CHECK(straight_rails == 2);  // only the visible entry straight
    CHECK(arc_rails == 2);
}

}  // namespace

int main() {
    RUN(arc_with_transitions_produces_full_chain);
    RUN(compound_mode_produces_two_arcs);
    RUN(degenerate_input_falls_back_to_straights);
    RUN(editor_clamps_oversized_arc_to_fitting_radius);
    RUN(editor_leaves_gap_unfitted_when_radius_below_floor);
    RUN(editor_emits_two_rails_at_standard_gauge);
    RUN(editor_exposes_transition_radius_range);
    RUN(editor_leaves_parallel_offset_unfitted);
    RUN(editor_honours_requested_radius_via_single_arc);
    RUN(hidden_tangent_guide_fits_curve_without_rendering_track);
    return REPORT();
}
