#include "maj0sted/domain/fitting/gap_fitter.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numeric>
#include <optional>
#include <utility>
#include <variant>
#include <vector>

#include "maj0sted/domain/fitting/fit_parameters.hpp"
#include "maj0sted/domain/fitting/fitting_service.hpp"

namespace maj0sted::domain {

namespace {

std::optional<Length> optional_length(double metres) {
    if (metres > 0.0) return Length::from_metres(metres);
    return std::nullopt;
}

// The two straights point in (nearly) opposite directions: the connection is a
// 180° reversal ("nawrót"), not an ordinary corner. Matches fit_uturn's own
// anti-parallel threshold.
bool is_reversal(const Straight& entry, const Straight& exit) {
    const double cos_between =
        std::cos(entry.azimuth().radians() - exit.azimuth().radians());
    return cos_between <= -1.0 + 1e-6;
}

// A tangent point may extend the drawn straight *toward the corner* as far as the
// geometry needs (filling the gap between the finite straight and the corner), but
// must never cross behind the outer, anchored end — that would flip the straight
// around and fling it off-screen. So we only bound the outer side: the point must
// lie on the ray from @p outer through @p corner (t >= 0), with no upper limit.
bool tangent_ok(CartesianPosition p, CartesianPosition outer,
                CartesianPosition corner) {
    const double dx = corner.x() - outer.x();
    const double dy = corner.y() - outer.y();
    const double len2 = dx * dx + dy * dy;
    if (len2 <= 0.0) return false;
    const double t = ((p.x() - outer.x()) * dx + (p.y() - outer.y()) * dy) / len2;
    return t >= -1e-3;
}

double deflection(const Straight& entry, const Straight& exit) {
    const double a1 = entry.azimuth().radians();
    const double a2 = exit.azimuth().radians();
    const double d1x = std::sin(a1), d1y = std::cos(a1);
    const double d2x = std::sin(a2), d2y = std::cos(a2);
    return std::abs(std::atan2(d1x * d2y - d1y * d2x, d1x * d2x + d1y * d2y));
}

// The single place the "lands on both finite straights" invariant is checked.
std::optional<FitResult> attempt_arc(const Straight& entry, const Straight& exit,
                                     double radius,
                                     std::optional<Length> transition) {
    if (!(radius > 0.0)) return std::nullopt;
    try {
        FitResult fit = FittingService::fit_between_straights(
            entry, exit, Radius::from_metres(radius), transition);
        // The corner is at entry.end() and exit.start() (that is where the two
        // straights meet through the fit); the anchored outer ends are the
        // opposite endpoints.
        if (tangent_ok(fit.tangent_in, entry.start(), entry.end()) &&
            tangent_ok(fit.tangent_out, exit.end(), exit.start())) {
            return fit;
        }
    } catch (...) {
    }
    return std::nullopt;
}

// Smallest radius the geometry admits: two symmetric clothoids consume the whole
// deflection at R = transition / delta, below which the arc angle turns
// negative; a plain arc has only a tiny numerical floor.
double smallest_radius(const Straight& entry, const Straight& exit,
                       std::optional<Length> transition) {
    constexpr double kTiny = 1e-3;
    if (!transition) return kTiny;
    const double delta = deflection(entry, exit);
    if (delta < 1e-3) return kTiny;
    return std::max(kTiny, transition->metres() / delta) * (1.0 + 1e-10);
}

// Bisects a fitting radius toward its non-fitting boundary and returns the fit
// at the fitting end. @p fitting fits, @p missing does not.
std::optional<FitResult> refine(const Straight& entry, const Straight& exit,
                                std::optional<Length> transition, double fitting,
                                double missing, double& used_radius) {
    for (int step = 0; step < 60; ++step) {
        const double mid = std::midpoint(fitting, missing);
        if (attempt_arc(entry, exit, mid, transition)) {
            fitting = mid;
        } else {
            missing = mid;
        }
    }
    used_radius = fitting;
    return attempt_arc(entry, exit, fitting, transition);
}

// The largest radius <= @p requested_radius whose arc fits both straights, or
// nullopt when none does. A fit NEVER enlarges the user's radius: if the request
// is too large to land on the (extendable) straights, we only ever reduce it to
// the largest radius that does. A radius that is smaller than the request already
// fits (a tighter arc just extends the straights toward the corner), so it is
// used verbatim upstream and this reduction path is not even reached.
std::optional<FitResult> fit_nearest_arc(const Straight& entry,
                                         const Straight& exit,
                                         double requested_radius,
                                         std::optional<Length> transition,
                                         double& used_radius) {
    const double floor = smallest_radius(entry, exit, transition);
    constexpr int kSamples = 4096;

    // Scan DOWN from the request for the largest radius that fits, then bisect.
    if (requested_radius > floor) {
        const double span = requested_radius - floor;
        double missing = requested_radius;
        for (int i = 1; i <= kSamples; ++i) {
            const double r = requested_radius - span * i / kSamples;
            if (attempt_arc(entry, exit, r, transition)) {
                return refine(entry, exit, transition, r, missing, used_radius);
            }
            missing = r;
        }
    }
    return std::nullopt;
}

std::optional<GapConnection> connect_uturn(const Straight& entry,
                                           const Straight& exit,
                                           std::optional<Length> transition,
                                           double requested_radius) {
    try {
        FitResult fit = FittingService::fit_uturn(entry, exit, transition);
        double applied = requested_radius;
        for (const auto& element : fit.curve) {
            if (const auto* arc = std::get_if<CircularArc>(&element)) {
                applied = arc->radius().metres();
                break;
            }
        }
        return GapConnection{std::move(fit), applied};
    } catch (...) {
        return std::nullopt;
    }
}

std::optional<GapConnection> connect_compound(const Straight& entry,
                                              const Straight& exit,
                                              const CompoundGapParameters& c) {
    if (c.arcs.empty()) return std::nullopt;
    try {
        // Every arc except the last carries its fixed length (which fixes its
        // angle) and an optional clothoid to the next; the last arc absorbs the
        // remaining deflection. One arc => "łuk + klotoidy", two or more => a
        // "łuk koszowy".
        std::vector<CompoundArc> arcs;
        arcs.reserve(c.arcs.size());
        for (std::size_t i = 0; i < c.arcs.size(); ++i) {
            const auto& a = c.arcs[i];
            if (i + 1 == c.arcs.size()) {
                arcs.push_back(CompoundArc{Radius::from_metres(a.radius)});
            } else {
                arcs.push_back(CompoundArc{Radius::from_metres(a.radius),
                                           Length::from_metres(a.length),
                                           optional_length(a.transition_to_next)});
            }
        }
        FitResult fit = FittingService::fit_compound(
            entry, exit,
            CompoundFitParameters{
                .entry_transition = optional_length(c.entry_transition),
                .exit_transition = optional_length(c.exit_transition),
                .arcs = std::move(arcs)});
        return GapConnection{std::move(fit), c.arcs.front().radius};
    } catch (...) {
        return std::nullopt;
    }
}

}  // namespace

std::optional<GapConnection> GapFitter::connect(
    const Straight& entry, const Straight& exit,
    const GapConnectionRequest& request) {
    // A compound curve is taken as requested (its arcs/clothoids are explicit).
    if (request.mode == GapConnectionMode::Compound) {
        return connect_compound(entry, exit, request.compound);
    }

    // Plain arc. A 180° reversal is geometry-driven and handled transparently.
    if (is_reversal(entry, exit)) {
        return connect_uturn(entry, exit, std::nullopt, request.radius);
    }

    // Use the requested radius verbatim when it fits (a tighter arc fits by
    // extending the straights toward the corner). Only when the request is too
    // large to land on them do we reduce it to the largest radius that fits — the
    // radius is never enlarged.
    if (auto fit = attempt_arc(entry, exit, request.radius, std::nullopt)) {
        return GapConnection{std::move(*fit), request.radius};
    }
    double applied = request.radius;
    if (auto fit = fit_nearest_arc(entry, exit, request.radius, std::nullopt, applied)) {
        return GapConnection{std::move(*fit), applied};
    }
    return std::nullopt;
}

}  // namespace maj0sted::domain
