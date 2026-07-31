#pragma once

#include <optional>

#include "maj0sted/domain/fitting/fit_parameters.hpp"
#include "maj0sted/domain/fitting/fit_result.hpp"
#include "maj0sted/domain/niweleta.hpp"
#include "maj0sted/domain/value_objects/length.hpp"
#include "maj0sted/domain/value_objects/radius.hpp"

namespace maj0sted::domain {

/// Domain Service: fits a curve between two straights of a single niweleta.
class FittingService {
public:
    // --- Overloads that fit two standalone straights (no niweleta) ---

    /// Rounds the corner between two straight lines with an arc (and optional
    /// symmetric transition curves). Works for any deflection strictly between
    /// 0° and 180°. For a 180° reversal use fit_uturn instead.
    static FitResult fit_between_straights(
        const Straight& entry, const Straight& exit, Radius radius,
        std::optional<Length> transition = std::nullopt);

    /// Fits a compound curve between two standalone straight lines.
    static FitResult fit_compound(const Straight& entry, const Straight& exit,
                                  const CompoundFitParameters& params);

    /// Connects two (anti-)parallel straights with a 180° reversal ("nawrót").
    /// The radius is derived from the perpendicular offset between the two lines
    /// (a plain arc uses R = offset / 2). Optional symmetric transition curves
    /// are solved for numerically.
    /// @throws std::invalid_argument if the straights are not anti-parallel or
    ///         their offset is zero.
    static FitResult fit_uturn(const Straight& entry, const Straight& exit,
                               std::optional<Length> transition = std::nullopt);

    /// Fits an S-curve made of two tangent circular arcs with the same radius
    /// and opposite turn directions. Supports non-parallel straights and
    /// same-direction parallel straights with a non-zero lateral offset.
    /// Tangent points are required to lie on both finite straight segments.
    /// @throws std::invalid_argument if no equal-radius reverse curve fits.
    static FitResult fit_reverse(const Straight& entry, const Straight& exit,
                                 Radius radius);

    /// Computes the curve (and the trimmed straights) that connects the two
    /// straights named in @p params, both belonging to @p niweleta.
    ///
    /// @throws std::invalid_argument if a straight is missing, the two are the
    ///         same, the straights are parallel/collinear, or the transition
    ///         curves are too long for the chosen radius and deflection.
    static FitResult fit_between_straights(const Niweleta& niweleta,
                                           const FitParameters& params);

    /// Computes a compound curve ("łuk koszowy") — a chain of N arcs of
    /// different neighbouring radii, same direction — between the two straights
    /// named in @p params. Both straight lines stay fixed and are re-trimmed in
    /// length; each arc's length sets its central angle, and the last arc takes
    /// the remaining deflection.
    ///
    /// @throws std::invalid_argument if a straight is missing, the two are the
    ///         same, there are no arcs, neighbouring radii are equal, a non-last
    ///         arc has no length, the straights are parallel/collinear, or the
    ///         split lengths exceed the total deflection.
    static FitResult fit_compound(const Niweleta& niweleta,
                                  const CompoundFitParameters& params);
};

}  // namespace maj0sted::domain
