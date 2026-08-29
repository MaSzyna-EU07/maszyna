#pragma once

#include <string>
#include <vector>

#include "maj0sted/domain/geometry/segment_layout.hpp"  // Pose
#include "maj0sted/domain/geometry/turnout.hpp"         // TurnoutPart
#include "maj0sted/editor/model.hpp"

namespace maj0sted::editor {

/// A point in the project's CRS (metres for EPSG:2180).
struct PlanPoint {
    double x{0.0};
    double y{0.0};
};

// ---------------------------------------------------------------------------
// Solved geometry
// ---------------------------------------------------------------------------

/// What one authored element works out to on the ground. Curvatures rather than
/// radii, because that is what `geometry::layout_segment` takes and what makes a
/// straight (0, 0), an arc (k, k) and a clothoid (k0, k1) one shape.
struct SolvedElement {
    ElementId id{ElementId::none};
    Kind kind{Kind::Line};
    domain::geometry::Pose start{};  ///< pose the element is laid from
    domain::geometry::Pose end{};    ///< pose it ends on = the next element's start
    double k0{0.0};                  ///< signed curvature at the start (1/m)
    double k1{0.0};                  ///< signed curvature at the end
    double length{0.0};
    std::vector<PlanPoint> points;   ///< sampled axis, first point == start
};

/// A solved track: its elements in order, plus the axis flattened into one
/// polyline with no duplicated join points, which is what a station along the
/// track is resolved against.
struct SolvedTrack {
    TrackId id{TrackId::none};
    std::vector<SolvedElement> elements;
    std::vector<PlanPoint> centreline;
    domain::geometry::Pose start{};
    domain::geometry::Pose end{};
    double length{0.0};
    /// True when the chain was laid whole. False means it was cut short by a
    /// diagnostic — the elements up to the cut are still here and still drawable.
    bool complete{true};
};

/// One laid segment of a turnout's diverging path.
struct SolvedTurnoutSegment {
    domain::geometry::Pose start{};
    double k0{0.0}, k1{0.0}, length{0.0};
    domain::TurnoutPart part{domain::TurnoutPart::Curve};
    std::vector<PlanPoint> points;
};

/// A construction point of a turnout: where two of its parts meet, or a point
/// the catalogue drawing is dimensioned from. Reported rather than left to be
/// eyeballed off the drawn curve.
struct TurnoutMark {
    double x{0.0}, y{0.0};
    /// 0 PR, 1 ostrze iglicy (where the blade begins, the odcinek przediglicowy
    /// behind it), 2 pięta iglicy, 3 joint inside the curve, 4 end of curve /
    /// start of the frog rail, 5 KR, 6 punkt teoretyczny, 7 koniec dzioba iglicy
    /// (A, where the blade is u thick), 8 koniec strugania, 9 koniec rozjazdu on
    /// the through track (the frog projected back onto the straight).
    int kind{0};
    double station{0.0};  ///< along the diverging path from PR; 0 for the theoretical point
};

struct SolvedTurnout {
    TurnoutId id{TurnoutId::none};
    bool valid{false};
    domain::geometry::Pose start{};  ///< PR, on the through track
    domain::geometry::Pose frog{};   ///< KR, where a branch anchors
    double tangent_front{0.0}, tangent_back{0.0};
    double diverging_length{0.0};
    double through_length{0.0};  ///< PR to the frog's projection, along the through track
    std::vector<SolvedTurnoutSegment> path;
    std::vector<TurnoutMark> marks;

    /// The blade as dimensioned: where its tip stands (the odcinek przediglicowy
    /// is everything before it), how long the planed nose and the planing run,
    /// the kąt przylegania, and whether the type's own a/d/u agree with the curve.
    /// Łukowanie, as laid: the signed curvature of the through track under it, the
    /// one the branch came out at, and how far the through track turns over the
    /// turnout. All zero for a rozjazd zwyczajny.
    double bend{0.0};
    double diverging_curvature{0.0};
    double bend_angle{0.0};

    bool has_blade{false};
    double blade_station{0.0};
    double nose_end{0.0};
    double planing_length{0.0};
    double blade_angle{0.0};
    double blade_residual{0.0};
    /// What the laid path works out to, against what the catalogue promised:
    /// deflection minus the crossing angle, and through-length minus the
    /// catalogue length. Reported, never fitted away.
    double angle_residual{0.0};
    double length_residual{0.0};
};

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------

/// Why something could not be laid. Reported, never silently dropped: the user
/// gets the reason along with whatever part of the drawing did stand up.
enum class Code {
    UnknownReference,     ///< an anchor points at an id that is not there
    CyclicDependency,     ///< tracks/turnouts depend on each other in a circle
    DegenerateElement,    ///< non-positive radius or length, a clothoid that eases nothing
    TurnoutInvalid,       ///< catalogue numbers admit no geometry
    TurnoutNotClosed,     ///< the laid path misses the crossing angle or the catalogue length
    StationOffTrack,      ///< a turnout sits past the end of its through track
    TouchingStraights,    ///< two straights end to end, which is one straight
    HoldConflict,         ///< two elements of one movable track both hold it parallel
    OffsetNotHeld,        ///< a pinned track cannot move, so its offset is what it is
};

[[nodiscard]] const char* code_name(Code code);

struct Diagnostic {
    Code code{Code::DegenerateElement};
    TrackId track{TrackId::none};
    ElementId element{ElementId::none};
    TurnoutId turnout{TurnoutId::none};
    /// Human-readable, with the number that matters in it ("R=300 m nie mieści
    /// się, maksimum 214,4 m"). The host shows this; it does not compose it.
    std::string text;
};

// ---------------------------------------------------------------------------
// The whole solution
// ---------------------------------------------------------------------------

/// Everything derived. Rebuilt whole on each solve, never fed back into the
/// document. Vectors rather than maps so iteration order is the document's and
/// two solves of the same document compare equal.
struct Solution {
    std::vector<SolvedTrack> tracks;
    std::vector<SolvedTurnout> turnouts;
    std::vector<Diagnostic> diagnostics;
};

[[nodiscard]] const SolvedTrack* find_track(const Solution& solution, TrackId id);
[[nodiscard]] const SolvedTurnout* find_turnout(const Solution& solution, TurnoutId id);
[[nodiscard]] const SolvedElement* find_element(const Solution& solution, ElementId id);

/// Pose at @p station metres along a solved track's centreline. Clamped to the
/// ends. Returns false for an empty track.
[[nodiscard]] bool pose_at(const SolvedTrack& track, double station,
                           domain::geometry::Pose& out);

/// How much track of one steady curvature there is around @p station: @p behind
/// back to where the element begins, @p ahead on to where it ends, and
/// @p curvature what it runs at (signed, positive turns left; zero on a
/// straight). A turnout is laid as one piece of geometry, bent to the one radius
/// under it, so it belongs on a single element and has to fit on it whole.
/// False on a clothoid, where there is no one radius to bend to, and past the
/// end of the track. Nothing is written then.
[[nodiscard]] bool steady_room(const SolvedTrack& track, double station, double& curvature,
                               double& behind, double& ahead);

}  // namespace maj0sted::editor
