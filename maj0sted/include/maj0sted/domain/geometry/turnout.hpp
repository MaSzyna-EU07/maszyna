#pragma once

#include <vector>

#include "maj0sted/domain/geometry/segment_layout.hpp"  // Pose

namespace maj0sted::domain {

/// The crossing mark of a turnout — the skos 1:n. It is the tangent of the angle
/// between the through and diverging tracks, so it fixes the total angle the
/// diverging curve must turn through: a turnout is 1:9, 1:12, 1:18,5 and so on.
class CrossingMark {
public:
    explicit CrossingMark(double denominator) noexcept : denominator_{denominator} {}

    [[nodiscard]] double denominator() const noexcept { return denominator_; }
    /// The angle between the tracks, radians. Zero for a non-positive mark.
    [[nodiscard]] double angle() const noexcept;

private:
    double denominator_;
};

/// Which way the branch leaves the through track.
enum class DivergeSide { Left, Right };

/// One arc of a turnout's diverging curve: its radius and — unless it is the
/// closing arc — its fixed length and the clothoid easing it into the next arc.
struct TurnoutArc {
    double radius{0.0};
    double length{0.0};              ///< fixed length; ignored for the closing arc
    double transition_to_next{0.0};  ///< clothoid to the next arc (0 = none)
};

/// The shape of a turnout's diverging curve. One arc is the everyday turnout; two
/// or more make a łuk koszowy. Whatever the shape, the curve turns through exactly
/// the crossing angle — the closing (last) arc spends whatever deflection the
/// earlier arcs and clothoids leave.
struct DivergingCurve {
    std::vector<TurnoutArc> arcs;   ///< at least one; the last one closes the turn
    double entry_transition{0.0};   ///< clothoid from the through track into the first arc
    double exit_transition{0.0};    ///< clothoid from the last arc back toward tangent
};

/// A turnout (rozjazd), described the way a catalogue does: a crossing mark, the
/// side it opens to, the free diverging curve inside it, and the length it spans
/// from its start (początek rozjazdu) to its end (koniec rozjazdu) measured along
/// the through track. The curve turns to the crossing angle and a straight frog
/// rail carries the track on to that length.
struct Turnout {
    CrossingMark mark;
    DivergeSide side{DivergeSide::Left};
    DivergingCurve curve;
    double length{0.0};  ///< catalogue length along the through track; 0 = a tangent-length rail
};

/// One laid segment of the diverging path, as the constant/linear curvature the
/// renderer and the fitter already share (see segment_layout): a straight when
/// both curvatures are zero, an arc when they are equal, a clothoid otherwise.
struct TurnoutSegment {
    double k0{0.0};
    double k1{0.0};
    double length{0.0};
};

/// What a turnout works out to when laid from its start pose.
struct TurnoutGeometry {
    bool valid{false};
    std::vector<TurnoutSegment> path;  ///< diverging segments from the switch start to the frog
    geometry::Pose frog{};             ///< the end (koniec rozjazdu): position + heading at the crossing angle
    double tangent_front{0.0};         ///< theoretical tangent length, switch start to the crossing point
    double tangent_back{0.0};          ///< theoretical tangent length, crossing point to the frog
    double diverging_length{0.0};      ///< total length of the diverging path (curve + frog rail)
};

/// Lays a turnout forward from @p start — the switch start, tangent to the through
/// track — turning toward its side until the heading has swung by the crossing
/// angle, then running straight to the catalogue length. Never throws; returns
/// `valid == false` when the parameters admit no geometry: a non-positive radius,
/// or a curve whose fixed parts already overshoot the crossing angle before the
/// closing arc.
[[nodiscard]] TurnoutGeometry lay_turnout(const geometry::Pose& start,
                                          const Turnout& turnout);

}  // namespace maj0sted::domain
