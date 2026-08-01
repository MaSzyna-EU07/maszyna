#include "maj0sted/app/straight_constraints.hpp"

#include <algorithm>
#include <cmath>

#include "maj0sted/domain/geometry/plan_element.hpp"
#include "maj0sted/domain/parallelism/parallelism_service.hpp"
#include "maj0sted/domain/value_objects/cartesian_position.hpp"

namespace maj0sted::app {
namespace {

using maj0sted::domain::CartesianPosition;
using maj0sted::domain::ParallelismService;
using maj0sted::domain::Straight;
using maj0sted::editor::NiweletaSpec;
using maj0sted::editor::StraightSpec;

Straight to_straight(const StraightSpec& s) {
    return Straight{CartesianPosition{s.x1, s.y1}, CartesianPosition{s.x2, s.y2}};
}

}  // namespace

void align_gap_fits(NiweletaSpec& niweleta) {
    const auto gaps = niweleta.straights.empty() ? std::size_t{0}
                                                 : niweleta.straights.size() - 1;
    std::vector<maj0sted::editor::GapFit> slots(gaps);
    for (std::size_t gap = 0; gap < gaps; ++gap) {
        slots[gap].gap = static_cast<int>(gap);
    }
    for (const auto& fit : niweleta.fits) {
        if (fit.gap >= 0 && static_cast<std::size_t>(fit.gap) < gaps) {
            slots[static_cast<std::size_t>(fit.gap)] = fit;
        }
    }
    niweleta.fits = std::move(slots);
}

void apply_straight_constraints(std::vector<NiweletaSpec>& niwelety) {
    for (int pass = 0; pass < 3; ++pass) {
        bool changed = false;
        for (auto& niweleta : niwelety) {
            for (auto& st : niweleta.straights) {
                if (st.rel_kind == 0 || st.rel_niw < 0 || st.rel_str < 0) {
                    continue;
                }
                if (static_cast<std::size_t>(st.rel_niw) >= niwelety.size()) {
                    continue;
                }
                const auto& ref_niw = niwelety[static_cast<std::size_t>(st.rel_niw)];
                if (static_cast<std::size_t>(st.rel_str) >= ref_niw.straights.size()) {
                    continue;
                }
                const auto& ref = ref_niw.straights[static_cast<std::size_t>(st.rel_str)];
                if (&ref == &st) {
                    continue;
                }

                const double dx = ref.x2 - ref.x1;
                const double dy = ref.y2 - ref.y1;
                const double len = std::hypot(dx, dy);
                if (len < 1e-6) {
                    continue;
                }

                double x1 = st.x1, y1 = st.y1, x2 = st.x2, y2 = st.y2;
                if (st.rel_kind == 1) {
                    const Straight projected = ParallelismService::project_onto_offset_line(
                        to_straight(ref), st.rel_offset, to_straight(st));
                    x1 = projected.start().x();
                    y1 = projected.start().y();
                    x2 = projected.end().x();
                    y2 = projected.end().y();
                } else if (st.rel_kind == 2) {
                    const double a =
                        std::atan2(1.0, std::max(1e-9, st.rel_cot)) *
                        (st.rel_side < 0 ? -1.0 : 1.0);
                    const double ux = dx / len;
                    const double uy = dy / len;
                    const double ca = std::cos(a);
                    const double sa = std::sin(a);
                    const double rx = ux * ca - uy * sa;
                    const double ry = ux * sa + uy * ca;
                    double L = std::hypot(st.x2 - st.x1, st.y2 - st.y1);
                    if (L < 1.0) {
                        L = st.rel_length > 0.0 ? st.rel_length : len;
                    }
                    st.rel_length = L;
                    x1 = st.x1;
                    y1 = st.y1;
                    x2 = st.x1 + rx * L;
                    y2 = st.y1 + ry * L;
                }

                if (std::abs(st.x1 - x1) > 1e-9 || std::abs(st.y1 - y1) > 1e-9 ||
                    std::abs(st.x2 - x2) > 1e-9 || std::abs(st.y2 - y2) > 1e-9) {
                    st.x1 = x1;
                    st.y1 = y1;
                    st.x2 = x2;
                    st.y2 = y2;
                    changed = true;
                }
            }
        }
        if (!changed) {
            break;
        }
    }
}

}  // namespace maj0sted::app
