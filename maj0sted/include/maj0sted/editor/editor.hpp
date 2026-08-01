#pragma once

#include <string>
#include <vector>

#include "maj0sted/editor/solve.hpp"  // PlanPoint, PlanPolyline

namespace maj0sted::editor {

/// An anchored straight segment (two XY endpoints).
struct StraightSpec {
    double x1{0.0}, y1{0.0}, x2{0.0}, y2{0.0};
    /// Internal tangent guide used to solve a terminal curve. It participates
    /// in fitting but is not rendered as track or exposed as an editor handle.
    bool hidden{false};

    /// Live constraint the host reapplies before each solve (same model as the
    /// web editor's `par` / `skew`). 0 = none, 1 = parallel offset from another
    /// straight, 2 = skew 1:n from another straight.
    int rel_kind{0};
    int rel_niw{-1};       ///< index of the reference niweleta
    int rel_str{-1};       ///< index of the reference straight
    double rel_offset{0.0}; ///< parallel: signed metres (+ = left of travel)
    double rel_cot{0.0};    ///< skew: n of a 1:n turnout
    int rel_side{1};        ///< skew: +1 left / -1 right of the reference
    double rel_length{0.0}; ///< skew: length of the dependent arm
};

/// One arc of a compound fit: its radius, its fixed length (ignored for the last
/// arc, which absorbs the remaining deflection) and an optional clothoid to the
/// next arc.
struct CompoundArcSpec {
    double radius{0.0};
    double length{0.0};
    double transition_to_next{0.0};
};

/// A fit in the gap between straight @c gap and straight @c gap + 1.
struct GapFit {
    int gap{0};
    int mode{0};  ///< 0 none, 1 arc, 2 legacy arc+clothoids, 3 compound, 4 U-turn

    double radius{0.0};      ///< plain arc (mode 1)
    double transition{0.0};  ///< legacy mode-2 symmetric clothoid

    // Unified compound (mode 3): end clothoids plus one or more arcs.
    double entry_t{0.0}, exit_t{0.0};
    std::vector<CompoundArcSpec> arcs{};

    // Legacy fixed two-arc fields, kept so old saved projects still deserialize;
    // superseded by @c arcs. Used only when @c arcs is empty.
    double r1{0.0}, arc1_len{0.0}, between{0.0}, r2{0.0};
};

/// A niweleta as edited in the GUI: independent anchored straights plus fits
/// connecting consecutive straights. Because the straights are independent, a
/// gap can be a normal corner (straights meet at an intersection) OR a 180°
/// reversal (parallel, offset straights) — no 180° limitation.
struct NiweletaSpec {
    std::string name;
    std::vector<StraightSpec> straights;
    std::vector<GapFit> fits;
};

struct NiweletaPolys {
    std::vector<PlanPolyline> polylines;
    /// Centreline segments after adjacent fits trim the input straights. Kept
    /// index-aligned with NiweletaSpec::straights for editor hit handles.
    std::vector<StraightSpec> rendered_straights;
    /// The authoritative fit set after solving: exactly the requested fits that
    /// actually succeeded. A requested fit that could not be produced (bad
    /// parameters or geometry) is simply absent here — the library, not the host,
    /// decides that a failed fit becomes "no fit". The host mirrors this list.
    std::vector<GapFit> applied_fits;
    /// The solved axis as one polyline, in order, with no duplicate join points.
    /// Lets a station along the niweleta be resolved to a pose (see pose_at).
    std::vector<PlanPoint> centreline;
};

/// Builds each niweleta, fits every requested gap, and returns the sampled plan
/// per niweleta. Never throws: gaps whose parameters don't admit a fit are left
/// unconnected (the two straights simply stay as drawn).
[[nodiscard]] std::vector<NiweletaPolys> solve_project(
    const std::vector<NiweletaSpec>& niwelety);

/// A switch (rozjazd): the point where a branch leaves a through niweleta. It is
/// a relation, not a niweleta — the through track is not split. The switch is
/// described the surveyor's way: a crossing mark 1:n (skos) that fixes the angle
/// the branch leaves at, and an interior curve that is free to be anything — a
/// single arc, or a compound basket (łuk koszowy). @c station marks the switch's
/// theoretical point (środek rozjazdu, where the through and diverging tangent
/// lines cross); the curve is fitted around it and the branch niweleta is pinned
/// to the frog it ends on. The theoretical tangent lengths follow from the curve
/// (see JunctionGeom) — they are read off, not set.
struct Junction {
    int through{-1};        ///< index of the through niweleta (not modified)
    double station{0.0};    ///< theoretical point: arc length along the through centreline
    int side{0};            ///< 0 = diverges left, 1 = diverges right
    bool facing{true};      ///< true opens toward increasing station, false back
    double crossing_n{9.0}; ///< crossing mark 1:n (skos) → leaving angle atan(1/n)
    double length{0.0};     ///< catalogue length (PR→KR along the through track); 0 = just the curve
    GapFit curve;           ///< the internal diverging curve: arc / compound / basket
    int branch{-1};         ///< index of the branch niweleta, pinned to the frog
};

/// The solved geometry of one switch: the switch point where the branch leaves
/// the through track, the frog it ends on, the diverging curve between them and
/// the theoretical tangent lengths that curve works out to.
struct JunctionGeom {
    bool valid{false};
    double px{0.0}, py{0.0};   ///< switch point (diverging leaves the through here)
    double fx{0.0}, fy{0.0};   ///< frog: where the branch niweleta is pinned
    double fhx{0.0}, fhy{0.0};  ///< unit heading the branch leaves the frog on
    double tangent_front{0.0}; ///< theoretical tangent length before the curve
    double tangent_back{0.0};  ///< theoretical tangent length after the curve
    double length{0.0};        ///< diverging curve length (switch point to frog), metres
    /// The diverging curve, as rails ready to draw.
    std::vector<PlanPolyline> polylines;
};

/// A whole layout solved at once: the niwelety plus every switch. Returned by
/// solve_layout, which additionally pins each branch's start to its frog.
struct LayoutSolution {
    std::vector<NiweletaPolys> niwelety;
    std::vector<JunctionGeom> junctions;  ///< index-aligned with the input switches
};

/// Solves @p niwelety together with @p junctions: each branch niweleta's start is
/// pinned to its switch's frog before solving, and every switch's diverging arc
/// is computed from the through niweleta's centreline. Never throws; a switch
/// whose through/branch is out of range or whose geometry is degenerate comes
/// back with valid == false. Takes @p niwelety by value because it repins them.
[[nodiscard]] LayoutSolution solve_layout(std::vector<NiweletaSpec> niwelety,
                                          const std::vector<Junction>& junctions);

}  // namespace maj0sted::editor
