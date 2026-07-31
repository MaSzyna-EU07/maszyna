#include "maj0sted/domain/fitting/fitting_service.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <variant>
#include <vector>

#include "maj0sted/domain/geometry/segment_layout.hpp"

namespace maj0sted::domain {
namespace {

constexpr double kEpsilon = 1e-9;

struct Vec {
    double east;
    double north;
};

// Unit direction of travel for an azimuth (east=x, north=y).
Vec direction_of(Azimuth azimuth) noexcept {
    return Vec{std::sin(azimuth.radians()), std::cos(azimuth.radians())};
}

const Straight& find_straight(const Niweleta& niweleta, StraightId id) {
    for (const auto& element : niweleta.plan().elements()) {
        if (const auto* straight = std::get_if<Straight>(&element)) {
            if (!straight->id().is_null() && straight->id() == id) {
                return *straight;
            }
        }
    }
    throw std::invalid_argument{"Straight with the given id not found in the niweleta"};
}

// A segment of the chain: signed curvature k0 -> k1 over `length`.
struct Seg {
    double k0;
    double k1;
    double length;
};

// Result of closing a chain onto the exit straight: the two tangent points that
// the entry/exit straights are trimmed to.
struct ChainClose {
    CartesianPosition tangent_in;
    CartesianPosition tangent_out;
};

double straight_length(const Straight& straight) noexcept {
    return straight.length().metres();
}

double parameter_on(const Straight& straight, CartesianPosition point) noexcept {
    const Vec direction = direction_of(straight.azimuth());
    return (point.x() - straight.start().x()) * direction.east +
           (point.y() - straight.start().y()) * direction.north;
}

bool valid_tangent_points(const Straight& entry, const Straight& exit,
                          const ChainClose& close) noexcept {
    constexpr double tolerance = 1e-6;
    const double entry_parameter = parameter_on(entry, close.tangent_in);
    const double exit_parameter = parameter_on(exit, close.tangent_out);
    // A zero-length trimmed straight cannot be represented by Straight.
    return entry_parameter > tolerance &&
           entry_parameter <= straight_length(entry) + tolerance &&
           exit_parameter >= -tolerance &&
           exit_parameter < straight_length(exit) - tolerance;
}

// Lays out `segments` head-to-tail from entry.start() heading along the entry
// azimuth, then slides the whole rigid chain along the entry line until its end
// lands on the exit line. Uses the SAME layout as the renderer, so the fitted
// tangent points coincide exactly with the drawn curve.
ChainClose solve_chain(const Straight& entry, const Straight& exit,
                       const std::vector<Seg>& segments) {
    const Vec d1 = direction_of(entry.azimuth());
    const Vec d2 = direction_of(exit.azimuth());

    geometry::Pose pose{entry.start().x(), entry.start().y(), d1.east, d1.north};
    for (const auto& s : segments) {
        pose = geometry::layout_segment(s.k0, s.k1, s.length, pose, nullptr);
    }

    const Vec exit_normal{-d2.north, d2.east};
    const double denominator = d1.east * exit_normal.east + d1.north * exit_normal.north;
    if (std::abs(denominator) < kEpsilon) {
        throw std::invalid_argument{"Straights are parallel; nothing to fit"};
    }
    const double offset = (pose.x - exit.start().x()) * exit_normal.east +
                          (pose.y - exit.start().y()) * exit_normal.north;
    const double shift = -offset / denominator;

    return ChainClose{
        CartesianPosition{entry.start().x() + shift * d1.east,
                          entry.start().y() + shift * d1.north},
        CartesianPosition{pose.x + shift * d1.east, pose.y + shift * d1.north}};
}

}  // namespace

FitResult FittingService::fit_between_straights(const Niweleta& niweleta,
                                                const FitParameters& params) {
    if (params.entry.is_null() || params.exit.is_null()) {
        throw std::invalid_argument{"Both straights must be identified"};
    }
    if (params.entry == params.exit) {
        throw std::invalid_argument{"Fitting needs two different straights"};
    }
    return fit_between_straights(find_straight(niweleta, params.entry),
                                 find_straight(niweleta, params.exit), params.radius,
                                 params.transition_length);
}

// ---------------------------------------------------------------------------
// Arc fit (tryb łuku)
//
// Rounds the corner between two supporting lines with a circular arc, optionally
// eased by a symmetric pair of clothoids (spiral–arc–spiral). The straights are
// treated as infinite lines: this routine finds the tangent points on those
// lines and returns the two straights trimmed to them. Whether those tangent
// points fall on the drawn finite segments is a separate, higher-level policy
// (see domain::GapFitter) — not this service's concern.
//
// The geometry is produced by laying the arc chain out with the SAME sampler the
// renderer uses (geometry::layout_segment via solve_chain) and sliding it onto
// the exit line. That guarantees the returned tangent points coincide with the
// drawn curve to the last bit, even for tight radii, so no lateral gap appears.
// ---------------------------------------------------------------------------
FitResult FittingService::fit_between_straights(const Straight& entry,
                                                const Straight& exit,
                                                Radius radius_value,
                                                std::optional<Length> transition_value) {
    const double radius = radius_value.metres();  // Radius guarantees radius > 0
    double transition = 0.0;
    if (transition_value) {
        transition = transition_value->metres();
        if (!(transition > 0.0)) {
            throw std::invalid_argument{"Transition length must be positive"};
        }
    }

    // Signed turn from the entry heading to the exit heading, in (-pi, pi]. Its
    // sign is the turn direction; its magnitude is the total deflection the curve
    // must absorb.
    const Vec d1 = direction_of(entry.azimuth());
    const Vec d2 = direction_of(exit.azimuth());
    const double deflection = std::atan2(d1.east * d2.north - d1.north * d2.east,
                                         d1.east * d2.east + d1.north * d2.north);
    const double total_turn = std::abs(deflection);
    if (total_turn < kEpsilon || std::abs(total_turn - std::numbers::pi) < kEpsilon) {
        throw std::invalid_argument{
            "Straights are parallel, collinear or anti-parallel; no arc fits"};
    }
    const TurnDirection direction =
        deflection > 0.0 ? TurnDirection::Left : TurnDirection::Right;
    const double curvature = (deflection > 0.0 ? 1.0 : -1.0) / radius;

    // A clothoid of length L run into a radius R bends the heading by L / (2R);
    // the two symmetric spirals therefore consume L / R of the turn, and the
    // circular arc between them takes whatever remains.
    const double arc_turn = total_turn - transition / radius;
    if (arc_turn < -kEpsilon) {
        throw std::invalid_argument{
            "Transition curves are too long for this radius and deflection"};
    }
    const double arc_length = radius * std::max(arc_turn, 0.0);

    // Chain the elements head-to-tail — clothoid in / arc / clothoid out — using
    // one definition for both the closure solve and the returned plan elements.
    std::vector<Seg> chain;
    std::vector<PlanElement> curve;
    chain.reserve(3);
    curve.reserve(3);
    if (transition > 0.0) {
        chain.push_back({0.0, curvature, transition});
        curve.push_back(TransitionCurve{Length::from_metres(transition),
                                        std::nullopt, radius_value, direction});
    }
    chain.push_back({curvature, curvature, arc_length});
    curve.push_back(
        CircularArc{radius_value, direction, Length::from_metres(arc_length)});
    if (transition > 0.0) {
        chain.push_back({curvature, 0.0, transition});
        curve.push_back(TransitionCurve{Length::from_metres(transition),
                                        radius_value, std::nullopt, direction});
    }

    const ChainClose close = solve_chain(entry, exit, chain);
    return FitResult{
        Straight{entry.id(), entry.start(), close.tangent_in},
        std::move(curve),
        Straight{exit.id(), close.tangent_out, exit.end()},
        close.tangent_in,
        close.tangent_out,
        direction,
    };
}

FitResult FittingService::fit_uturn(const Straight& entry, const Straight& exit,
                                    std::optional<Length> transition) {
    const Vec d1 = direction_of(entry.azimuth());
    const Vec d2 = direction_of(exit.azimuth());
    const double dot = d1.east * d2.east + d1.north * d2.north;
    if (dot > -1.0 + 1e-6) {
        throw std::invalid_argument{"A U-turn needs two anti-parallel straights"};
    }
    if (transition && !(transition->metres() > 0.0)) {
        throw std::invalid_argument{"Transition length must be positive"};
    }

    // Signed perpendicular offset from the entry line to the exit line.
    const Vec left{-d1.north, d1.east};
    const double offset = (exit.start().x() - entry.start().x()) * left.east +
                          (exit.start().y() - entry.start().y()) * left.north;
    if (std::abs(offset) < kEpsilon) {
        throw std::invalid_argument{"U-turn straights are collinear (zero offset)"};
    }
    const double sign = (offset > 0.0) ? 1.0 : -1.0;  // turn toward the exit line
    const TurnDirection direction =
        (offset > 0.0) ? TurnDirection::Left : TurnDirection::Right;

    // Plain 180° arc spans 2R across the offset, so R = |offset| / 2. (Transition
    // curves would change the span; solved numerically — not yet supported here.)
    const double radius = std::abs(offset) / 2.0;
    const Radius radius_vo = Radius::from_metres(radius);
    const double arc_length = radius * std::numbers::pi;

    // Start the reversal at the entry straight's end; the 180° arc lands on the
    // exit line at the diametrically opposite point.
    const CartesianPosition tangent_in = entry.end();
    const CartesianPosition tangent_out{tangent_in.x() + std::abs(offset) * sign * left.east,
                                        tangent_in.y() + std::abs(offset) * sign * left.north};

    std::vector<PlanElement> curve;
    curve.push_back(CircularArc{radius_vo, direction, Length::from_metres(arc_length)});
    (void)transition;  // clothoid U-turn: follow-up

    return FitResult{
        Straight{entry.id(), entry.start(), tangent_in},
        std::move(curve),
        Straight{exit.id(), tangent_out, exit.end()},
        tangent_in,
        tangent_out,
        direction,
    };
}

FitResult FittingService::fit_reverse(const Straight& entry, const Straight& exit,
                                      Radius radius_vo) {
    constexpr double parallel_tolerance = 1e-6;
    constexpr double angle_epsilon = 1e-5;
    const double radius = radius_vo.metres();
    const Vec d1 = direction_of(entry.azimuth());
    const Vec d2 = direction_of(exit.azimuth());
    const double cross = d1.east * d2.north - d1.north * d2.east;
    const double dot = d1.east * d2.east + d1.north * d2.north;

    const auto direction = [](double sign) {
        return sign > 0.0 ? TurnDirection::Left : TurnDirection::Right;
    };
    const auto opposite = [](TurnDirection value) {
        return value == TurnDirection::Left ? TurnDirection::Right
                                            : TurnDirection::Left;
    };
    const auto result_from = [&](const ChainClose& close, double first_sign,
                                 double first_angle, double second_angle) {
        const TurnDirection first_direction = direction(first_sign);
        std::vector<PlanElement> curve;
        curve.push_back(CircularArc{radius_vo, first_direction,
                                    Length::from_metres(radius * first_angle)});
        curve.push_back(CircularArc{radius_vo, opposite(first_direction),
                                    Length::from_metres(radius * second_angle)});
        return FitResult{
            Straight{entry.id(), entry.start(), close.tangent_in},
            std::move(curve),
            Straight{exit.id(), close.tangent_out, exit.end()},
            close.tangent_in,
            close.tangent_out,
            first_direction,
        };
    };

    // Same-direction parallel lines are the canonical S-curve case. Their
    // lateral displacement fixes the common arc angle for a requested radius.
    if (std::abs(cross) <= parallel_tolerance) {
        if (dot < 0.0) {
            throw std::invalid_argument{
                "Anti-parallel straights require a U-turn, not a reverse curve"};
        }
        const Vec left{-d1.north, d1.east};
        const double offset =
            (exit.start().x() - entry.start().x()) * left.east +
            (exit.start().y() - entry.start().y()) * left.north;
        if (std::abs(offset) <= kEpsilon) {
            throw std::invalid_argument{
                "Collinear straights do not need a reverse curve"};
        }

        const double cosine = 1.0 - std::abs(offset) / (2.0 * radius);
        if (cosine < -1.0 || cosine >= 1.0) {
            throw std::invalid_argument{
                "Requested radius cannot span the parallel-line offset"};
        }
        const double angle = std::acos(std::clamp(cosine, -1.0, 1.0));
        if (angle <= angle_epsilon) {
            throw std::invalid_argument{"Reverse-curve angle is too small"};
        }
        const double first_sign = offset > 0.0 ? 1.0 : -1.0;
        const std::vector<Seg> segments{
            {first_sign / radius, first_sign / radius, radius * angle},
            {-first_sign / radius, -first_sign / radius, radius * angle},
        };

        geometry::Pose pose{entry.start().x(), entry.start().y(), d1.east, d1.north};
        for (const auto& segment : segments) {
            pose = geometry::layout_segment(segment.k0, segment.k1, segment.length,
                                            pose, nullptr);
        }
        const double exit_at_zero =
            (pose.x - exit.start().x()) * d2.east +
            (pose.y - exit.start().y()) * d2.north;
        constexpr double length_epsilon = 1e-6;
        const double lower = std::max(length_epsilon, -exit_at_zero);
        const double upper =
            std::min(straight_length(entry),
                     straight_length(exit) - length_epsilon - exit_at_zero);
        if (lower > upper) {
            throw std::invalid_argument{
                "Reverse curve does not fit on the finite parallel straights"};
        }

        // Prefer tangent points near entry.end() and exit.start().
        const double unconstrained =
            0.5 * (straight_length(entry) - exit_at_zero);
        const double shift = std::clamp(unconstrained, lower, upper);
        const ChainClose close{
            CartesianPosition{entry.start().x() + shift * d1.east,
                              entry.start().y() + shift * d1.north},
            CartesianPosition{pose.x + shift * d1.east,
                              pose.y + shift * d1.north},
        };
        if (!valid_tangent_points(entry, exit, close)) {
            throw std::invalid_argument{
                "Reverse-curve tangent points lie outside the parallel straights"};
        }
        return result_from(close, first_sign, angle, angle);
    }

    const double deflection = std::atan2(cross, dot);
    const double delta = std::abs(deflection);
    if (std::abs(delta - std::numbers::pi) <= parallel_tolerance) {
        throw std::invalid_argument{
            "Anti-parallel straights require a U-turn, not a reverse curve"};
    }
    const double extra_max = std::numbers::pi - delta - angle_epsilon;
    if (extra_max <= angle_epsilon) {
        throw std::invalid_argument{"Not enough angle for a reverse curve"};
    }

    std::optional<FitResult> best;
    double best_length = 0.0;
    constexpr int samples = 720;
    for (const double first_sign : {1.0, -1.0}) {
        const double signed_delta = deflection / first_sign;
        const auto attempt = [&](double extra) -> std::optional<FitResult> {
            const double first_angle =
                signed_delta >= 0.0 ? extra + signed_delta : extra;
            const double second_angle =
                signed_delta >= 0.0 ? extra : extra - signed_delta;
            if (first_angle <= angle_epsilon || second_angle <= angle_epsilon ||
                first_angle >= std::numbers::pi ||
                second_angle >= std::numbers::pi) {
                return std::nullopt;
            }
            const std::vector<Seg> segments{
                {first_sign / radius, first_sign / radius, radius * first_angle},
                {-first_sign / radius, -first_sign / radius, radius * second_angle},
            };
            try {
                const ChainClose close = solve_chain(entry, exit, segments);
                if (!valid_tangent_points(entry, exit, close)) return std::nullopt;
                return result_from(close, first_sign, first_angle, second_angle);
            } catch (...) {
                return std::nullopt;
            }
        };

        double previous = angle_epsilon;
        for (int sample = 1; sample <= samples; ++sample) {
            const double extra =
                angle_epsilon +
                (extra_max - angle_epsilon) * static_cast<double>(sample) /
                    static_cast<double>(samples);
            auto candidate = attempt(extra);
            if (!candidate) {
                previous = extra;
                continue;
            }

            // Refine the first feasible angle boundary to avoid grid-shaped
            // jumps while dragging a straight in the editor.
            double low = previous;
            double high = extra;
            std::optional<FitResult> refined = std::move(candidate);
            for (int iteration = 0; iteration < 48; ++iteration) {
                const double middle = std::midpoint(low, high);
                if (auto middle_result = attempt(middle)) {
                    high = middle;
                    refined = std::move(middle_result);
                } else {
                    low = middle;
                }
            }
            const double total_length = radius * (delta + 2.0 * high);
            if (!best || total_length < best_length) {
                best = std::move(refined);
                best_length = total_length;
            }
            break;
        }
    }

    if (!best) {
        throw std::invalid_argument{
            "No equal-radius reverse curve fits the finite straights"};
    }
    return *best;
}

FitResult FittingService::fit_compound(const Niweleta& niweleta,
                                       const CompoundFitParameters& params) {
    if (params.entry.is_null() || params.exit.is_null()) {
        throw std::invalid_argument{"Both straights must be identified"};
    }
    if (params.entry == params.exit) {
        throw std::invalid_argument{"Fitting needs two different straights"};
    }
    return fit_compound(find_straight(niweleta, params.entry),
                        find_straight(niweleta, params.exit), params);
}

FitResult FittingService::fit_compound(const Straight& entry, const Straight& exit,
                                       const CompoundFitParameters& params) {
    if (params.arcs.empty()) {
        throw std::invalid_argument{"A compound curve needs at least one arc"};
    }
    for (std::size_t k = 1; k < params.arcs.size(); ++k) {
        if (params.arcs[k].radius == params.arcs[k - 1].radius) {
            throw std::invalid_argument{
                "Adjacent compound arcs must have different radii"};
        }
    }

    const Vec d1 = direction_of(entry.azimuth());
    const Vec d2 = direction_of(exit.azimuth());
    const double cross = d1.east * d2.north - d1.north * d2.east;
    const double dot = d1.east * d2.east + d1.north * d2.north;
    const double deflection = std::atan2(cross, dot);
    const double delta = std::abs(deflection);
    if (delta < kEpsilon || std::abs(delta - std::numbers::pi) < kEpsilon) {
        throw std::invalid_argument{
            "Straights are parallel or collinear; nothing to fit"};
    }
    const double sign = (deflection >= 0.0) ? 1.0 : -1.0;
    const TurnDirection direction =
        (deflection > 0.0) ? TurnDirection::Left : TurnDirection::Right;

    const std::size_t arc_count = params.arcs.size();
    const auto curvature = [&](std::size_t k) {
        return sign / params.arcs[k].radius.metres();
    };
    const auto require_positive = [](const std::optional<Length>& length,
                                     const char* message) -> double {
        const double value = length->metres();
        if (value <= kEpsilon) throw std::invalid_argument{message};
        return value;
    };

    // Heading (deflection) consumed by everything except the last arc's angle:
    // the fixed-length arcs plus every transition curve. The last arc absorbs
    // whatever remains.
    double consumed = 0.0;
    std::vector<double> arc_angle(arc_count, 0.0);
    for (std::size_t k = 0; k + 1 < arc_count; ++k) {
        if (!params.arcs[k].length) {
            throw std::invalid_argument{"Every arc except the last needs a length"};
        }
        const double angle =
            require_positive(params.arcs[k].length, "Arc length must be positive") /
            params.arcs[k].radius.metres();
        arc_angle[k] = angle;
        consumed += angle;
    }

    // Transition heading = mean curvature * length = (1/Ra + 1/Rb)/2 * L.
    const double entry_len =
        params.entry_transition
            ? require_positive(params.entry_transition, "Transition length must be positive")
            : 0.0;
    if (entry_len > 0.0) {
        consumed += 0.5 * (1.0 / params.arcs.front().radius.metres()) * entry_len;
    }
    const double exit_len =
        params.exit_transition
            ? require_positive(params.exit_transition, "Transition length must be positive")
            : 0.0;
    if (exit_len > 0.0) {
        consumed += 0.5 * (1.0 / params.arcs.back().radius.metres()) * exit_len;
    }
    std::vector<double> between(arc_count, 0.0);
    for (std::size_t k = 0; k + 1 < arc_count; ++k) {
        if (params.arcs[k].transition_to_next) {
            const double length = require_positive(params.arcs[k].transition_to_next,
                                                   "Transition length must be positive");
            between[k] = length;
            consumed += 0.5 *
                        (1.0 / params.arcs[k].radius.metres() +
                         1.0 / params.arcs[k + 1].radius.metres()) *
                        length;
        }
    }

    arc_angle[arc_count - 1] = delta - consumed;
    if (arc_angle[arc_count - 1] <= kEpsilon) {
        throw std::invalid_argument{
            "Arc lengths and transitions exceed the total deflection"};
    }

    // Build the chain of segments: entry KP?, arc, (KP?, arc)..., exit KP?.
    std::vector<Seg> segments;
    if (entry_len > 0.0) segments.push_back({0.0, curvature(0), entry_len});
    for (std::size_t k = 0; k < arc_count; ++k) {
        segments.push_back({curvature(k), curvature(k),
                            params.arcs[k].radius.metres() * arc_angle[k]});
        if (k + 1 < arc_count && between[k] > 0.0) {
            segments.push_back({curvature(k), curvature(k + 1), between[k]});
        }
    }
    if (exit_len > 0.0) segments.push_back({curvature(arc_count - 1), 0.0, exit_len});

    const ChainClose close = solve_chain(entry, exit, segments);
    const CartesianPosition tangent_in = close.tangent_in;
    const CartesianPosition tangent_out = close.tangent_out;

    // Build the plan elements to splice in, mirroring the segment chain.
    std::vector<PlanElement> curve;
    if (entry_len > 0.0) {
        curve.push_back(TransitionCurve{Length::from_metres(entry_len), std::nullopt,
                                        params.arcs.front().radius, direction});
    }
    for (std::size_t k = 0; k < arc_count; ++k) {
        curve.push_back(
            CircularArc{params.arcs[k].radius, direction,
                        Length::from_metres(params.arcs[k].radius.metres() * arc_angle[k])});
        if (k + 1 < arc_count && between[k] > 0.0) {
            curve.push_back(TransitionCurve{Length::from_metres(between[k]),
                                            params.arcs[k].radius,
                                            params.arcs[k + 1].radius, direction});
        }
    }
    if (exit_len > 0.0) {
        curve.push_back(TransitionCurve{Length::from_metres(exit_len),
                                        params.arcs.back().radius, std::nullopt, direction});
    }

    return FitResult{
        Straight{entry.id(), entry.start(), tangent_in},
        std::move(curve),
        Straight{exit.id(), tangent_out, exit.end()},
        tangent_in,
        tangent_out,
        direction,
    };
}

}  // namespace maj0sted::domain
