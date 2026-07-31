#pragma once

#include <optional>
#include <vector>

#include "maj0sted/domain/value_objects/length.hpp"
#include "maj0sted/domain/value_objects/radius.hpp"
#include "maj0sted/domain/value_objects/straight_id.hpp"

namespace maj0sted::domain {

/// Input parameters for fitting a curve between two straights of one niweleta.
///
/// These map directly onto a GUI form: the user picks the entry and exit
/// straights, types a radius, and optionally a (symmetric) transition-curve
/// length. The turn direction is NOT an input — it follows from the geometry of
/// the two straights (their deflection angle).
///
///   * transition_length absent  -> a plain circular arc is fitted;
///   * transition_length present -> a symmetric klotoida-arc-klotoida is fitted,
///                                  with that length on both ends.
struct FitParameters {
    StraightId entry{};                          ///< pierwsza (wejściowa) prosta
    StraightId exit{};                           ///< druga (wyjściowa) prosta
    Radius radius;                               ///< promień łuku kołowego
    std::optional<Length> transition_length{};   ///< symetryczna klotoida; brak = sam łuk
};

/// One arc of a compound curve.
struct CompoundArc {
    Radius radius;                               ///< promień łuku
    std::optional<Length> length{};              ///< długość łuku; wymagana OPRÓCZ ostatniego
    std::optional<Length> transition_to_next{};  ///< klotoida do następnego łuku (ignorowana dla ostatniego)
};

/// Input parameters for fitting a compound curve — a sequence of one or more
/// circular arcs (of different neighbouring radii, same direction) with optional
/// transition curves (klotoidy) between the arcs and at the ends — between two
/// straights. This single shape covers both a "łuk + klotoidy" (one arc eased by
/// symmetric clothoids) and a "łuk koszowy" (two or more arcs).
///
/// Both straight lines stay put (position and azimuth). Each arc except the last
/// carries its length, which fixes its central angle (`δ = L / R`); the last (or
/// only) arc takes whatever deflection remains after the other arcs and every
/// transition curve. The two straights are re-trimmed in length so the whole
/// chain closes.
struct CompoundFitParameters {
    StraightId entry{};                        ///< pierwsza (wejściowa) prosta
    StraightId exit{};                         ///< druga (wyjściowa) prosta
    std::optional<Length> entry_transition{};  ///< klotoida prosta -> pierwszy łuk
    std::optional<Length> exit_transition{};   ///< klotoida ostatni łuk -> prosta
    std::vector<CompoundArc> arcs{};           ///< >= 1 łuk; sąsiednie promienie różne
};

}  // namespace maj0sted::domain
