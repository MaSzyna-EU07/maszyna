#pragma once

#include <cstdint>
#include <string>
#include <variant>
#include <vector>

namespace maj0sted::editor {

// ---------------------------------------------------------------------------
// Identity
// ---------------------------------------------------------------------------

/// Stable handles. Every reference in a document points at one of these, never
/// at a position in a vector: inserting or deleting anything must not silently
/// re-target a reference somewhere else. Ids are minted from one counter per
/// document (Document::next_id), so they are unique across all four spaces —
/// which makes a stray reference a lookup miss rather than a wrong hit.
enum class ElementId : std::uint32_t { none = 0 };
enum class TrackId : std::uint32_t { none = 0 };
enum class TurnoutId : std::uint32_t { none = 0 };

// ---------------------------------------------------------------------------
// The element
// ---------------------------------------------------------------------------

/// What an element is, geometrically. These are the three shapes
/// `geometry::layout_segment` lays: constant zero curvature, constant non-zero
/// curvature, linearly varying curvature.
enum class Kind {
    Line,
    Arc,
    Clothoid,
};

// ---------------------------------------------------------------------------
// What holds an element
// ---------------------------------------------------------------------------

/// Nothing holds it: its shape and its place are its own.
struct Free {};

/// Held parallel to another track's element at a signed offset (+ = left of
/// *that* element's travel, whichever way this one runs).
///
/// This is międzytorze, said once: a straight beside a straight, an arc
/// concentric with an arc at R∓d. It fixes two things — the shape (kind, radius,
/// hand, taken from the reference) and where the track runs across the
/// reference. It fixes nothing along it, so the track stays where it was put and
/// slides nowhere when the reference is edited.
///
/// A track that can move — one anchored at a pose of its own — is moved as one
/// rigid piece until the held element really lies at the offset. A track pinned
/// to a turnout's port cannot move: there the reference still fixes the shape,
/// and the offset that actually came out is reported rather than forced.
struct Parallel {
    ElementId ref{ElementId::none};
    double offset{0.0};
};

using Hold = std::variant<Free, Parallel>;

/// One element of a track. It starts where the previous element ends — by pose,
/// not by coordinates — so tangency is a property of the representation and not
/// a rule anyone has to check or restore.
///
/// Everything about it is stated outright: what shape it is, how long it is, and
/// how tightly it turns. Nothing here is left for a solver to decide — there is
/// no "run on until you reach that line". Laying track by clicking along the
/// ground still works, but the corner is worked out at the click and written
/// down as numbers, so what the document says is what stands on the ground.
struct Element {
    ElementId id{ElementId::none};
    Kind kind{Kind::Line};
    /// Arc: the radius, metres, > 0. Clothoid: the radius at its far end, with
    /// 0 meaning infinite (a clothoid running out to a straight); its near-end
    /// radius is whatever the previous element ends on. Line: ignored.
    double radius{0.0};
    /// +1 turns left, -1 turns right, 0 for a straight. Signed curvature is
    /// exactly `hand / radius`, and positive curvature turns left in
    /// `geometry::layout_segment` — so no sign is flipped anywhere downstream.
    int hand{0};
    /// Along the axis, metres, > 0.
    double length{0.0};
    /// What holds it to something else. Never changes its length.
    Hold hold{Free{}};
};

// ---------------------------------------------------------------------------
// Anchors
// ---------------------------------------------------------------------------

/// Start at an explicit pose. @c az is an azimuth: radians clockwise from north,
/// i.e. atan2(dx, dy), matching the rest of the editor.
struct AtPose {
    double x{0.0}, y{0.0}, az{0.0};
};
/// Which end of a turnout a track leaves from.
enum class Port {
    Frog,   ///< KR — koniec rozjazdu, the diverging track's end
    Start,  ///< PR — początek rozjazdu
};
/// Start at a turnout's port. This is what used to be `frog_of` plus the pinning
/// pass: the branch no longer gets moved back into place after the fact, it was
/// never anywhere else.
struct AtPort {
    TurnoutId turnout{TurnoutId::none};
    Port port{Port::Frog};
};

using Anchor = std::variant<AtPose, AtPort>;

// ---------------------------------------------------------------------------
// Track
// ---------------------------------------------------------------------------

/// A niweleta: an anchor plus an ordered chain of elements. Straights, arcs and
/// clothoids are equal citizens here — a chain may start with an arc, put two
/// arcs back to back, or simply stop without closing onto anything.
struct Track {
    TrackId id{TrackId::none};
    std::string name;
    Anchor anchor{AtPose{}};
    std::vector<Element> elements;
};

// ---------------------------------------------------------------------------
// Turnouts: catalogue type + placement
// ---------------------------------------------------------------------------

/// One piece of a catalogue turnout's diverging path. Mirrors
/// domain::TurnoutPiece; kept here so the document does not depend on the
/// geometry header.
struct TurnoutPieceSpec {
    int part{2};               ///< 0 przediglicowy, 1 iglica, 2 łuk, 3 krzyżownicowa
    double length{0.0};        ///< always explicit; nothing is fitted
    double radius_start{0.0};  ///< 0 = straight
    double radius_end{0.0};    ///< != radius_start = clothoid
    double turn_in{0.0};       ///< kąt nagięcia iglicy, radians, unsigned
};

/// The blade's own dimensions, which place its cut tip. Mirrors domain::Blade.
struct BladeSpec {
    double tip_thickness{0.0};  ///< u: width where the planed nose ends
    double nose{0.0};           ///< d: the planed nose, from the tip to that point
    double railtop_width{0.0};  ///< w: railhead width
};

/// A catalogue turnout: the list of pieces its diverging path is laid from, plus
/// the crossing mark and length the catalogue says it should work out to — which
/// the solver checks the list against and never fits it to. Every number that
/// belongs to the *type* lives here, so adding one is a catalogue change and
/// never touches the file format for placements.
struct TurnoutType {
    std::string name;
    double crossing_n{9.0};  ///< skos 1:n, checked against the laid deflection
    double length{0.0};      ///< catalogue length PR->KR along the through track, checked
    std::vector<TurnoutPieceSpec> pieces;
    BladeSpec blade{};
};

/// An instance of a type, laid on a through track. There is nothing here to
/// edit but where it sits — the geometry belongs to the type.
struct TurnoutPlacement {
    TurnoutId id{TurnoutId::none};
    std::string type;                ///< catalogue name
    TrackId on{TrackId::none};       ///< the through track, which is not split
    double station{0.0};             ///< PR, as arc length along the through centreline
    int hand{1};                     ///< +1 diverges left, -1 diverges right
    bool facing{true};               ///< opens toward increasing station

    /// Łukowanie. A turnout standing on a curve is bent onto it, and normally
    /// that is all there is to say: the tor zasadniczy is whatever the track does
    /// under the PR. Clearing this asks for a bend of its own instead — a rozjazd
    /// łukowy on a track drawn straight — and then `bend` is the signed curvature
    /// (1/m, positive turns left) the through track is taken to have. What the
    /// bend works out to is the solver's business and is never written back here.
    bool bend_from_track{true};
    double bend{0.0};
};

// ---------------------------------------------------------------------------
// Document
// ---------------------------------------------------------------------------

/// Everything that is authored. Nothing in here is ever written by the solver.
struct Document {
    std::vector<TurnoutType> turnout_types;
    std::vector<TurnoutPlacement> turnouts;
    std::vector<Track> tracks;

    double view_x{0.0}, view_y{0.0}, view_extent{0.0};
    bool origin_set{false};
    bool georeferenced{false};
    double origin_x{0.0}, origin_y{0.0};

    /// Next handle to hand out. One counter for every id space.
    std::uint32_t next_id{1};
};

/// Mints the next handle. The four overloads exist so a caller states which kind
/// of thing it is naming.
[[nodiscard]] ElementId mint_element(Document& document);
[[nodiscard]] TrackId mint_track(Document& document);
[[nodiscard]] TurnoutId mint_turnout(Document& document);

[[nodiscard]] const Track* find_track(const Document& document, TrackId id);
[[nodiscard]] Track* find_track(Document& document, TrackId id);
[[nodiscard]] const TurnoutPlacement* find_turnout(const Document& document, TurnoutId id);
[[nodiscard]] TurnoutPlacement* find_turnout(Document& document, TurnoutId id);
[[nodiscard]] const TurnoutType* find_type(const Document& document, const std::string& name);
/// The element itself, wherever it is; and the track it belongs to. Null when
/// unknown.
[[nodiscard]] const Element* find_element(const Document& document, ElementId id);
[[nodiscard]] const Track* find_element_track(const Document& document, ElementId id);

}  // namespace maj0sted::editor
