#include "maj0sted/app/basket_draft.hpp"

#include <algorithm>
#include <cmath>

#include "maj0sted/app/straight_constraints.hpp"

namespace maj0sted::app {
namespace {

using maj0sted::editor::CompoundArcSpec;
using maj0sted::editor::GapFit;
using maj0sted::editor::NiweletaPolys;
using maj0sted::editor::NiweletaSpec;
using maj0sted::editor::RibbonElement;
using maj0sted::editor::RibbonRequest;
using maj0sted::editor::StraightSpec;
using maj0sted::editor::PlanPolyline;

void sanitize_draft_arcs(BasketDraft& draft) {
    for (auto& arc : draft.arcs) {
        if (!(arc.radius >= 30.0)) {
            arc.radius = 30.0;
        }
        if (!(arc.length > 0.0)) {
            arc.length = 5.0;
        }
        if (arc.transition_to_next < 0.0) {
            arc.transition_to_next = 0.0;
        }
    }
    if (draft.entry_t < 0.0) {
        draft.entry_t = 0.0;
    }
    if (draft.exit_t < 0.0) {
        draft.exit_t = 0.0;
    }
}

bool draft_end_pose(const BasketDraft& draft,
                    const std::vector<PlanPolyline>& polys, double& out_x,
                    double& out_y, double& out_az) {
    if (!draft.active) {
        return false;
    }
    out_az = draft.az0;
    if (!polys.empty() && !polys.back().points.empty()) {
        const auto& pts = polys.back().points;
        out_x = pts.back().x;
        out_y = pts.back().y;
        if (pts.size() >= 2) {
            out_az = std::atan2(pts.back().x - pts[pts.size() - 2].x,
                                pts.back().y - pts[pts.size() - 2].y);
        }
        return true;
    }
    out_x = draft.x0;
    out_y = draft.y0;
    return true;
}

}  // namespace

RibbonRequest draft_to_ribbon(const BasketDraft& draft) {
    RibbonRequest req;
    req.x0 = draft.x0;
    req.y0 = draft.y0;
    req.az0 = draft.az0;
    if (draft.arcs.empty()) {
        return req;
    }
    const int dir = draft.dir >= 0 ? 1 : -1;
    if (draft.entry_t > 1e-6) {
        RibbonElement el;
        el.kind = 2;
        el.length = draft.entry_t;
        el.radius_start = 0.0;
        el.radius = draft.arcs.front().radius;
        el.dir = dir;
        req.elements.push_back(el);
    }
    for (std::size_t i = 0; i < draft.arcs.size(); ++i) {
        const auto& arc = draft.arcs[i];
        const double r = std::max(30.0, arc.radius);
        const double len = std::max(1e-3, arc.length);
        RibbonElement a;
        a.kind = 1;
        a.radius = r;
        a.angle = len / r;
        a.dir = dir;
        req.elements.push_back(a);
        if (arc.transition_to_next > 1e-6) {
            RibbonElement kp;
            kp.kind = 2;
            kp.length = arc.transition_to_next;
            kp.radius_start = r;
            if (i + 1 < draft.arcs.size()) {
                kp.radius = std::max(30.0, draft.arcs[i + 1].radius);
            } else {
                kp.radius = std::max(30.0, draft.preview_next_r);
            }
            kp.dir = dir;
            req.elements.push_back(kp);
        }
    }
    if (draft.exit_t > 1e-6) {
        RibbonElement el;
        el.kind = 2;
        el.length = draft.exit_t;
        el.radius_start = draft.arcs.back().radius;
        el.radius = 0.0;
        el.dir = dir;
        req.elements.push_back(el);
    }
    return req;
}

std::vector<PlanPolyline> preview_basket_draft(BasketDraft& draft,
                                              double fallback_next_r) {
    if (!draft.active || draft.arcs.empty()) {
        return {};
    }
    sanitize_draft_arcs(draft);
    draft.preview_next_r = std::max(30.0, fallback_next_r);
    return maj0sted::editor::solve_ribbon(draft_to_ribbon(draft));
}

void start_basket_draft_from_end(BasketDraft& draft, const StraightSpec& straight,
                                 int straight_index, int end, double default_r) {
    draft = {};
    draft.active = true;
    draft.attach_str = straight_index;
    draft.attach_end = end;
    draft.dir = 1;
    draft.preview_next_r = default_r;
    draft.reedit_gap = -1;
    draft.reedit_backup = {};
    if (end == 1) {
        draft.x0 = straight.x2;
        draft.y0 = straight.y2;
        draft.az0 = std::atan2(straight.x2 - straight.x1, straight.y2 - straight.y1);
    } else {
        draft.x0 = straight.x1;
        draft.y0 = straight.y1;
        draft.az0 = std::atan2(straight.x1 - straight.x2, straight.y1 - straight.y2);
    }
    draft.arcs.push_back({600.0, 60.0, 40.0});
    draft.arcs.push_back({std::max(30.0, default_r), 80.0, 0.0});
}

bool reopen_basket_draft(BasketDraft& draft, NiweletaSpec& niweleta,
                         std::size_t gap, const NiweletaPolys* solved) {
    if (gap + 1 >= niweleta.straights.size() || gap >= niweleta.fits.size()) {
        return false;
    }
    const auto& fit = niweleta.fits[gap];
    if (fit.mode != 3 || fit.arcs.empty()) {
        return false;
    }

    std::vector<double> solved_arc_lengths;
    if (solved != nullptr) {
        for (const auto& poly : solved->polylines) {
            if (poly.gap != static_cast<int>(gap) || poly.kind != 1 ||
                poly.element_index < 0) {
                continue;
            }
            bool seen = false;
            for (const auto& other : solved->polylines) {
                if (&other == &poly) {
                    break;
                }
                if (other.gap == poly.gap &&
                    other.element_index == poly.element_index) {
                    seen = true;
                    break;
                }
            }
            if (!seen) {
                solved_arc_lengths.push_back(std::max(5.0, poly.length));
            }
        }
    }

    const auto& entry = niweleta.straights[gap];
    const auto& exit = niweleta.straights[gap + 1];
    const double edx = entry.x2 - entry.x1;
    const double edy = entry.y2 - entry.y1;
    const double xdx = exit.x2 - exit.x1;
    const double xdy = exit.y2 - exit.y1;
    const double turn = edx * xdy - edy * xdx;

    draft = {};
    draft.active = true;
    draft.reedit_backup = fit;
    draft.reedit_gap = static_cast<int>(gap);
    draft.attach_str = static_cast<int>(gap);
    draft.attach_end = 1;
    draft.x0 = entry.x2;
    draft.y0 = entry.y2;
    draft.az0 = std::atan2(edx, edy);
    draft.dir = turn >= 0.0 ? 1 : -1;
    draft.entry_t = std::max(0.0, fit.entry_t);
    draft.exit_t = std::max(0.0, fit.exit_t);
    draft.arcs = fit.arcs;
    for (std::size_t i = 0; i < draft.arcs.size(); ++i) {
        if (!(draft.arcs[i].radius >= 30.0)) {
            draft.arcs[i].radius = 30.0;
        }
        if (draft.arcs[i].length <= 1e-6) {
            draft.arcs[i].length =
                i < solved_arc_lengths.size() ? solved_arc_lengths[i] : 60.0;
        }
        if (draft.arcs[i].transition_to_next < 0.0) {
            draft.arcs[i].transition_to_next = 0.0;
        }
    }
    if (!draft.arcs.empty()) {
        draft.arcs.back().transition_to_next = 0.0;
        draft.preview_next_r = draft.arcs.back().radius;
    }

    niweleta.fits[gap] = {};
    niweleta.fits[gap].gap = static_cast<int>(gap);
    niweleta.fits[gap].mode = 0;
    return true;
}

void discard_basket_draft(BasketDraft& draft, NiweletaSpec* niweleta) {
    if (draft.reedit_gap >= 0 && niweleta != nullptr &&
        static_cast<std::size_t>(draft.reedit_gap) < niweleta->fits.size()) {
        niweleta->fits[static_cast<std::size_t>(draft.reedit_gap)] =
            draft.reedit_backup;
    }
    draft = {};
}

bool anchor_basket_draft(BasketDraft& draft, NiweletaSpec& niweleta,
                         std::size_t gap) {
    if (!draft.active || draft.arcs.empty() || niweleta.straights.size() < 2) {
        return false;
    }
    if (gap + 1 >= niweleta.straights.size() || gap >= niweleta.fits.size()) {
        return false;
    }

    const GapFit previous = niweleta.fits[gap];
    const BasketDraft draft_backup = draft;

    GapFit fit{};
    fit.gap = static_cast<int>(gap);
    fit.mode = 3;
    fit.entry_t = std::max(0.0, draft.entry_t);
    fit.exit_t = std::max(0.0, draft.exit_t);
    fit.arcs = draft.arcs;
    for (std::size_t i = 0; i + 1 < fit.arcs.size(); ++i) {
        if (!(fit.arcs[i].radius >= 30.0)) {
            fit.arcs[i].radius = 30.0;
        }
        if (!(fit.arcs[i].length > 0.0)) {
            fit.arcs[i].length = 40.0;
        }
    }
    fit.arcs.back().length = 0.0;
    fit.arcs.back().transition_to_next = 0.0;
    if (!(fit.arcs.back().radius >= 30.0)) {
        fit.arcs.back().radius = 30.0;
    }
    for (std::size_t i = 1; i < fit.arcs.size(); ++i) {
        if (std::abs(fit.arcs[i].radius - fit.arcs[i - 1].radius) < 1.0) {
            fit.arcs[i].radius = std::max(30.0, fit.arcs[i - 1].radius * 0.75);
        }
    }

    niweleta.fits[gap] = fit;
    niweleta.straights[gap].rel_kind = 0;
    niweleta.straights[gap + 1].rel_kind = 0;

    const auto probe = maj0sted::editor::solve_project(
        std::vector<NiweletaSpec>{niweleta});
    bool applied = false;
    if (!probe.empty()) {
        for (const auto& af : probe.front().applied_fits) {
            if (af.gap == static_cast<int>(gap) && af.mode == 3) {
                applied = true;
                break;
            }
        }
    }
    if (!applied) {
        niweleta.fits[gap] = previous;
        draft = draft_backup;
        return false;
    }

    draft = {};
    return true;
}

FinishBasketDraftResult finish_basket_draft(
    BasketDraft& draft, NiweletaSpec& niweleta,
    std::vector<NiweletaSpec>& all_niwelety, int niweleta_index,
    double finish_straight_m) {
    FinishBasketDraftResult result;
    if (!draft.active || draft.arcs.empty() || draft.attach_str < 0 ||
        static_cast<std::size_t>(draft.attach_str) >= niweleta.straights.size()) {
        result.status = "brak łuku";
        return result;
    }

    if (draft.reedit_gap >= 0) {
        const auto gap = static_cast<std::size_t>(draft.reedit_gap);
        const GapFit backup = draft.reedit_backup;
        niweleta.fits[gap] = backup;
        if (!anchor_basket_draft(draft, niweleta, gap)) {
            draft.reedit_gap = static_cast<int>(gap);
            draft.reedit_backup = backup;
            result.status = "nie da się (kierunek / L+KP)";
            return result;
        }
        result.ok = true;
        result.status = "luka " + std::to_string(gap);
        return result;
    }

    auto polys = preview_basket_draft(draft);
    double tipx = 0.0, tipy = 0.0, tipaz = 0.0;
    if (!draft_end_pose(draft, polys, tipx, tipy, tipaz)) {
        result.status = "brak końca kosza";
        return result;
    }

    const NiweletaSpec backup = niweleta;
    const BasketDraft draft_backup = draft;
    const int attach = draft.attach_str;

    auto& entry = niweleta.straights[static_cast<std::size_t>(attach)];
    if (draft.attach_end == 0) {
        std::swap(entry.x1, entry.x2);
        std::swap(entry.y1, entry.y2);
        draft.attach_end = 1;
    }
    entry.x2 = draft.x0;
    entry.y2 = draft.y0;
    entry.rel_kind = 0;

    const double sl = std::max(20.0, finish_straight_m);
    StraightSpec exit_st{};
    exit_st.x1 = tipx;
    exit_st.y1 = tipy;
    exit_st.x2 = tipx + std::sin(tipaz) * sl;
    exit_st.y2 = tipy + std::cos(tipaz) * sl;

    const auto insert_at = static_cast<std::size_t>(attach + 1);
    for (auto& fit : niweleta.fits) {
        if (fit.gap >= attach) {
            ++fit.gap;
        }
    }
    auto bump_rel = [&](int delta) {
        for (std::size_t n = 0; n < all_niwelety.size(); ++n) {
            for (auto& st : all_niwelety[n].straights) {
                if (st.rel_kind != 0 && st.rel_niw == niweleta_index &&
                    st.rel_str >= static_cast<int>(insert_at)) {
                    st.rel_str += delta;
                }
            }
        }
    };
    bump_rel(+1);
    niweleta.straights.insert(
        niweleta.straights.begin() + static_cast<std::ptrdiff_t>(insert_at),
        exit_st);
    align_gap_fits(niweleta);

    if (!anchor_basket_draft(draft, niweleta, static_cast<std::size_t>(attach))) {
        bump_rel(-1);
        niweleta = backup;
        draft = draft_backup;
        result.status = "nie da się (kierunek / L+KP)";
        return result;
    }

    result.ok = true;
    result.status = "kosz + " + std::to_string(static_cast<int>(sl)) + " m";
    return result;
}

}  // namespace maj0sted::app
