#pragma once

#include <vector>

#include "maj0sted/domain/geometry/segment_layout.hpp"  // Pose

namespace maj0sted::domain {

/// The crossing mark of a turnout — the skos 1:n. It is the tangent of the angle
/// between the through and diverging tracks, so it fixes the total angle the
/// diverging curve must turn through: a turnout is 1:9, 1:12, 1:18,5 and so on.
/// Nothing is fitted to it any more: it is what the laid path is checked against.
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

/// Which part of the turnout a piece of the diverging path is. The role names the
/// piece in switch terms; it does not constrain its shape — a blade may be straight
/// (iglica prosta sieczna) or an arc (iglica łukowa styczna).
enum class TurnoutPart {
    PreBlade,  ///< odcinek przediglicowy: PR to the blade tip, along the through track
    Blade,     ///< iglica, from its (theoretical) tip to its heel
    Curve,     ///< the diverging curve between the blade heel and the frog rail
    FrogRail,  ///< the straight frog rail out to KR
};

/// One authored piece of the diverging path, in order from PR. Every length is
/// explicit — the catalogue says it, and nothing here is solved for. A piece is a
/// straight when both radii are zero, an arc when they are equal, a clothoid
/// otherwise; radii are unsigned, the hand comes from DivergeSide.
struct TurnoutPiece {
    TurnoutPart part{TurnoutPart::Curve};
    double length{0.0};
    double radius_start{0.0};  ///< 0 = straight
    double radius_end{0.0};    ///< != radius_start = clothoid
    /// Kąt nagięcia iglicy (beta): the heading break applied before this piece,
    /// radians, unsigned. Zero everywhere for a blade tangent to the stock rail.
    double turn_in{0.0};
};

/// Iglica — not the shape of the axis (that is a TurnoutPiece), but the two
/// dimensions of the blade rail itself that place its real tip. A blade cannot be
/// ground to nothing, so its tip is cut back to a thickness and the point where
/// the axis leaves the stock rail is only a theoretical one.
struct Blade {
    double tip_thickness{0.0};  ///< u: the blade's width where its planed nose ends
    double nose{0.0};           ///< d: the planed nose itself, from the tip to that point
    double railtop_width{0.0};  ///< w: railhead width, metres (49E1 ≈ 0,070)
};

/// Half the standard gauge. The gap the blade opens is taken between two rails,
/// and the blade's own curve is `R + s/2` where R is the diverging track's radius
/// (Koc, 5.2), so this is what turns a gap into an angle. Stations stay on the
/// axis: the rail radius converts, it does not measure.
inline constexpr double kHalfGauge = 0.7175;

/// A turnout (rozjazd), described the way a catalogue drawing is: the crossing
/// mark and length it is meant to work out to, and the list of pieces the
/// diverging path is actually laid from.
struct Turnout {
    CrossingMark mark;
    DivergeSide side{DivergeSide::Left};
    std::vector<TurnoutPiece> path;  ///< PR -> KR, in order; nothing implicit
    /// Catalogue length: a + b, the two tangent legs of the drawing — PR to the
    /// theoretical crossing point along the through track, and that point to KR
    /// along the diverging one. This is the L the type tables give (Rz 190-1:9:
    /// a 13,834 + b 13,304 = 27,138), not the projection on the through track.
    double length{0.0};
    Blade blade{};
};

/// One laid segment of the diverging path: a piece with its curvature signed for
/// the hand it diverges to, ready for layout_segment.
struct TurnoutSegment {
    double turn_in{0.0};  ///< signed heading break applied before laying this segment
    double k0{0.0};
    double k1{0.0};
    double length{0.0};
    TurnoutPart part{TurnoutPart::Curve};
};

/// What a turnout works out to when laid from its start pose.
struct TurnoutGeometry {
    bool valid{false};
    std::vector<TurnoutSegment> path;  ///< diverging segments from the switch start to the frog
    geometry::Pose frog{};             ///< the end (koniec rozjazdu): position + heading
    double tangent_front{0.0};         ///< theoretical tangent length, switch start to the crossing point
    double tangent_back{0.0};          ///< theoretical tangent length, crossing point to the frog
    double diverging_length{0.0};      ///< total length of the diverging path
    /// Where the turnout ends as measured on the through track: the frog projected
    /// back onto it. This is the point that closes the switch on the straight — or
    /// on the arc, when the turnout is bent.
    geometry::Pose through_end{};
    double through_length{0.0};

    // --- łukowanie ----------------------------------------------------------
    /// Signed curvature of the through track the turnout was laid against, and
    /// what the diverging path came out at. Both zero for a rozjazd zwyczajny;
    /// `diverging_curvature` is zero too when a two-sided bend straightens the
    /// branch out. Positive turns left.
    double bend{0.0};
    double diverging_curvature{0.0};
    /// Beta: how far the through track turns over the turnout. The bend is this
    /// angle as much as it is a radius, and it is what the crossing angle is
    /// measured from once the tor zasadniczy no longer runs straight.
    double bend_angle{0.0};

    // --- the blade, as dimensioned rather than as laid ----------------------
    // The curve is theoretically tangent to the stock rail at the switch start, but
    // the blade only begins where there is metal enough to begin it. What runs from
    // PR to that tip is the odcinek przediglicowy: the curve is already there, the
    // vehicle is still on the through track.
    bool has_blade{false};
    geometry::Pose blade_tip{};       ///< ostrze iglicy, where the blade piece starts
    geometry::Pose blade_nose_end{};  ///< A: where the planed nose ends and the blade is u thick
    geometry::Pose blade_planed{};    ///< where the blade reaches full railhead width
    double blade_station{0.0};        ///< a: PR -> the tip, the odcinek przediglicowy
    double nose_end{0.0};             ///< station of A, theoretically a + d
    double planing_length{0.0};       ///< struganie: the tip to full railhead width
    double planing_end{0.0};          ///< station where the planing ends
    double blade_angle{0.0};          ///< delta: kąt przylegania, the tangent angle at A
    /// (a + d) minus where the rail really stands `u` off the stock rail. Zero means
    /// the type's own a, d and u agree with the curve it is laid on.
    double blade_residual{0.0};

    // --- what the catalogue numbers say about all that ----------------------
    /// Total deflection of the path minus the crossing angle. Zero = the path
    /// closes on the mark; non-zero is reported, never fitted away.
    double angle_residual{0.0};
    /// The laid tangent legs (a + b) minus the catalogue length.
    double length_residual{0.0};
};

/// How far along an axis of radius @p radius, leaving the stock rail at angle
/// @p angle, the blade stands @p offset away from it: `R·(acos(cos β − offset/R1) − β)`
/// for a curved blade, `offset / sin β` for a straight one. The gap opens on the
/// blade's own curve `R1 = R + s/2`, but the answer is a station on the axis, which
/// is what every length here is measured in.
///
/// This is the relation the catalogue's own numbers obey. A drawing dimensions the
/// blade on the rail rather than on the axis, so what Koc's Tablica 5.1 gives as
/// `a + d` is this run scaled by `R1/R` — at 1200-1:18,5 the two agree to a tenth
/// of a millimetre. Zero when the curve neither turns nor breaks away, or when the
/// offset is out of its reach.
[[nodiscard]] double blade_run_to_offset(double radius, double angle, double offset);

/// Lays @p turnout from @p start. @p bend is the signed curvature (1/m, positive
/// turns left) of the through track under it: zero for a rozjazd zwyczajny, and
/// anything else for a rozjazd łukowy.
///
/// Bending is not adding curvature. Koc, 5.3.6: it rotates the triangle OBC about
/// the centre of the turnout, so the crossing angle alfa and the tangent
/// `t = R·tg(alfa/2)` come through untouched and every radius follows from them —
/// the through track turns by `beta = 2·arctg(t·bend)`, the diverging path by
/// `beta + alfa`, and each gets the radius `t/tg(its own angle/2)`. That is where
/// Koc's (5.7) and (5.8) come from, and it is why a Rz 49E1-190-1:9 bent
/// symmetrically is the catalogue's Rłs 49E1-380,292-1:9.
///
/// The switch and the frog are not re-cut, only bent: a straight krzyżownica stays
/// straight and the blade keeps its own dimensions, measured — as on the ground —
/// against the opornica that curves with it.
///
/// Invalid when the path has no arc to bend or its arcs disagree on a radius:
/// there is then no single triangle to rotate, and guessing one would be a lie.
[[nodiscard]] TurnoutGeometry lay_turnout(const geometry::Pose& start, const Turnout& turnout,
                                          double bend = 0.0);

}  // namespace maj0sted::domain
