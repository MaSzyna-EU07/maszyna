/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/
module;
#include <variant>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <numbers>
#include <string>
#include <vector>

module eu07.editor.plan_layout;
import eu07.editor.plan_turnout_geometry;
import eu07.editor.plan_chain;
import eu07.editor.plan_sketch;

namespace editor::plan {
namespace {

constexpr double kEps = 1e-9;

using geometry::Pose;

std::string num(double value, int decimals = 1) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.*f", decimals, value);
    return buffer;
}

// ---------------------------------------------------------------------------
// The dependency graph
// ---------------------------------------------------------------------------

enum class NodeKind { Turnout, Track };

struct Node {
    NodeKind kind{NodeKind::Turnout};
    std::size_t index{0};
    std::vector<std::size_t> depends_on;
};

/// Everything a track needs laid before it can be laid itself: the turnout a
/// branch hangs off, and every track it holds an element parallel to — however
/// deep either goes. A hold that runs in a circle is caught as a cycle, the same
/// as anything else.
void track_dependencies(const Document& document, const Track& track,
                        const std::vector<std::size_t>& turnout_node,
                        const std::vector<std::size_t>& track_node, Node& node) {
    if (const auto* at_port = std::get_if<AtPort>(&track.anchor)) {
        for (std::size_t i = 0; i < document.turnouts.size(); ++i) {
            if (document.turnouts[i].id == at_port->turnout) {
                node.depends_on.push_back(turnout_node[i]);
                break;
            }
        }
    }
    for (const auto& element : track.elements) {
        const auto* parallel = std::get_if<Parallel>(&element.hold);
        if (parallel == nullptr) {
            continue;
        }
        for (std::size_t i = 0; i < document.tracks.size(); ++i) {
            if (document.tracks[i].id == track.id) {
                continue;
            }
            for (const auto& candidate : document.tracks[i].elements) {
                if (candidate.id == parallel->ref) {
                    node.depends_on.push_back(track_node[i]);
                    break;
                }
            }
        }
    }
}

/// Where the line leaving @p from meets @p track's centreline, as a station along it.
/// False when it never does — two tracks that do not converge have no przejście between
/// them, and saying so is better than putting the turnout somewhere plausible.
[[nodiscard]] bool meets_track(const SolvedTrack& track, const geometry::Pose& from,
                               double& station) {
    double walked = 0.0;
    double best = -1.0;
    double best_station = 0.0;
    for (std::size_t i = 1; i < track.centreline.size(); ++i) {
        const PlanPoint& a = track.centreline[i - 1];
        const PlanPoint& b = track.centreline[i];
        const double ex = b.x - a.x;
        const double ey = b.y - a.y;
        const double step = std::hypot(ex, ey);
        if (step <= kEps) {
            continue;
        }
        // from.x + t*from.hx == a.x + u*ex, same in y: solve for t along the ray and u
        // along this piece of the centreline
        const double denominator = from.hx * ey - from.hy * ex;
        if (std::abs(denominator) > 1e-12) {
            const double t = ((a.x - from.x) * ey - (a.y - from.y) * ex) / denominator;
            const double u = (from.hx * (a.y - from.y) - from.hy * (a.x - from.x)) / -denominator;
            if (t > kEps && u >= -1e-9 && u <= 1.0 + 1e-9 && (best < 0.0 || t < best)) {
                best = t;
                best_station = walked + step * std::clamp(u, 0.0, 1.0);
            }
        }
        walked += step;
    }
    if (best < 0.0) {
        return false;
    }
    station = best_station;
    return true;
}

// ---------------------------------------------------------------------------
// Holds: parallelism between elements
// ---------------------------------------------------------------------------

/// Where a held element has to be, and what shape it has to be, to lie at the
/// offset asked for. Everything is worked out from the reference as solved, so
/// this is geometry and not a guess.
struct Held {
    bool ok{false};
    Kind kind{Kind::Line};
    double radius{0.0};
    int hand{0};
    Pose target{};      ///< the pose the held element must start on
    double actual{0.0}; ///< the offset that came out, for a track that cannot move
    std::string why;    ///< why not, when !ok
};

/// The offset curve of @p reference at @p offset, taken where @p current is now.
///
/// Across the reference the answer is forced; along it, nothing is: the point
/// nearest to where the element already sits is the one taken, so holding a
/// track parallel never slides it sideways along the reference.
Held hold_against(const SolvedElement& reference, double offset, const Pose& current) {
    Held held;

    if (std::abs(reference.k1 - reference.k0) > kEps) {
        held.why =
            "równoległości nie da się trzymać do krzywej przejściowej — odsunięcie "
            "krzywej przejściowej nie jest krzywą przejściową";
        return held;
    }

    const Pose& base = reference.start;
    // Which way this element runs along the reference. The offset is always
    // measured to the left of the *reference's* travel, whichever way this one
    // goes, so a track laid against the run of another still sits where it looks.
    const double sense = (current.hx * base.hx + current.hy * base.hy) >= 0.0 ? 1.0 : -1.0;
    const double nx = -base.hy;
    const double ny = base.hx;
    const double k = reference.k0;

    if (std::abs(k) < kEps) {
        const double px = base.x + nx * offset;
        const double py = base.y + ny * offset;
        const double hx = sense * base.hx;
        const double hy = sense * base.hy;
        const double along = (current.x - px) * hx + (current.y - py) * hy;
        held.ok = true;
        held.kind = Kind::Line;
        held.target = Pose{px + hx * along, py + hy * along, hx, hy};
        held.actual = (current.x - base.x) * nx + (current.y - base.y) * ny;
        return held;
    }

    // Concentric with the reference: same centre, radius R∓d.
    const double radius = 1.0 / std::abs(k);
    const double cx = base.x + nx / k;
    const double cy = base.y + ny / k;
    const double curvature = k / (1.0 - offset * k);
    if (!std::isfinite(curvature) || std::abs(1.0 - offset * k) < kEps) {
        held.why = "odstęp " + num(offset, 2) + " m trafia w środek łuku R=" +
                   num(radius) + " m";
        return held;
    }

    double ux = current.x - cx;
    double uy = current.y - cy;
    const double reach = std::hypot(ux, uy);
    if (reach < kEps) {
        held.why = "odcinek zaczyna się w środku łuku, do którego ma trzymać "
                   "równoległość";
        return held;
    }
    ux /= reach;
    uy /= reach;

    const double held_radius = 1.0 / std::abs(curvature);
    const double turn = curvature > 0.0 ? 1.0 : -1.0;
    // The centre lies to the left of travel for a left-hand curve, so the
    // heading follows from where the point sits on the circle.
    const double hx = sense * turn * uy;
    const double hy = sense * turn * -ux;

    held.ok = true;
    held.kind = Kind::Arc;
    held.radius = held_radius;
    held.hand = (sense * turn) > 0.0 ? 1 : -1;
    held.target = Pose{cx + ux * held_radius, cy + uy * held_radius, hx, hy};
    // What offset this element actually keeps, as the drawing stands.
    held.actual = (k > 0.0 ? 1.0 : -1.0) * (radius - reach);
    return held;
}

/// Moves a whole chain, as one rigid piece, so that the pose @p from becomes the
/// pose @p to. Returns the anchor pose to lay it from again.
Pose rigid_move(const Pose& anchor, const Pose& from, const Pose& to) {
    const double cos_t = from.hx * to.hx + from.hy * to.hy;
    const double sin_t = from.hx * to.hy - from.hy * to.hx;
    const double dx = anchor.x - from.x;
    const double dy = anchor.y - from.y;
    return Pose{to.x + dx * cos_t - dy * sin_t, to.y + dx * sin_t + dy * cos_t,
                anchor.hx * cos_t - anchor.hy * sin_t,
                anchor.hx * sin_t + anchor.hy * cos_t};
}

// ---------------------------------------------------------------------------
// Turnouts
// ---------------------------------------------------------------------------

plan::TurnoutPart part_of(int part) {
    switch (part) {
        case 0: return plan::TurnoutPart::PreBlade;
        case 1: return plan::TurnoutPart::Blade;
        case 3: return plan::TurnoutPart::FrogRail;
        default: return plan::TurnoutPart::Curve;
    }
}

int mark_kind_for(plan::TurnoutPart part) {
    switch (part) {
        case plan::TurnoutPart::PreBlade: return 0;  // PR
        case plan::TurnoutPart::Blade: return 1;     // ostrze iglicy
        case plan::TurnoutPart::Curve: return 2;     // pięta iglicy
        case plan::TurnoutPart::FrogRail: return 4;  // koniec krzywej
    }
    return 3;
}

}  // namespace

// ---------------------------------------------------------------------------

plan::Turnout to_domain(const TurnoutType& type, int hand) {
    plan::Turnout turnout{
        plan::CrossingMark{type.crossing_n},
        hand >= 0 ? plan::DivergeSide::Left : plan::DivergeSide::Right,
        {},
        type.length,
        plan::Blade{type.blade.tip_thickness, type.blade.nose, type.blade.railtop_width},
    };
    turnout.path.reserve(type.pieces.size());
    for (const auto& piece : type.pieces) {
        turnout.path.push_back(plan::TurnoutPiece{part_of(piece.part), piece.length,
                                                    piece.radius_start, piece.radius_end,
                                                    piece.turn_in});
    }
    return turnout;
}

std::vector<TurnoutMark> turnout_marks(const plan::TurnoutGeometry& geometry,
                                       const geometry::Pose& start) {
    std::vector<TurnoutMark> marks;
    if (!geometry.valid) {
        return marks;
    }

    // PR stands whatever the path starts with, so it is said once here
    marks.push_back(TurnoutMark{start.x, start.y, 0, 0.0});

    geometry::Pose walker = start;
    double station = 0.0;
    plan::TurnoutPart previous = plan::TurnoutPart::PreBlade;
    bool first = true;
    for (const auto& segment : geometry.path) {
        if (std::abs(segment.turn_in) > kEps) {
            const double cos_t = std::cos(segment.turn_in);
            const double sin_t = std::sin(segment.turn_in);
            walker = geometry::Pose{walker.x, walker.y,
                                            walker.hx * cos_t - walker.hy * sin_t,
                                            walker.hx * sin_t + walker.hy * cos_t};
        }
        if ((first || segment.part != previous) &&
            !(first && segment.part == plan::TurnoutPart::PreBlade)) {
            marks.push_back(
                TurnoutMark{walker.x, walker.y, mark_kind_for(segment.part), station});
        }
        previous = segment.part;
        first = false;

        walker = geometry::layout_segment(segment.k0, segment.k1, segment.length,
                                                  walker, nullptr);
        station += segment.length;
    }

    marks.push_back(TurnoutMark{geometry.frog.x, geometry.frog.y, 5, station});
    marks.push_back(TurnoutMark{start.x + start.hx * geometry.tangent_front,
                                start.y + start.hy * geometry.tangent_front, 6, 0.0});
    // PR is on the through track already; this is where the switch ends on it, so the
    // straight carries both ends of the rozjazd
    marks.push_back(TurnoutMark{geometry.through_end.x, geometry.through_end.y, 9,
                                geometry.through_length});
    // the ostrze itself is marked with the blade piece; what the drawing dimensions on
    // top of it is where the planed nose ends (A, the rail u thick) and where the
    // planing does
    if (geometry.has_blade && geometry.nose_end > geometry.blade_station) {
        marks.push_back(TurnoutMark{geometry.blade_nose_end.x, geometry.blade_nose_end.y,
                                    7, geometry.nose_end});
    }
    if (geometry.has_blade && geometry.planing_length > 0.0) {
        marks.push_back(TurnoutMark{geometry.blade_planed.x, geometry.blade_planed.y, 8,
                                    geometry.planing_end});
    }
    return marks;
}

// ---------------------------------------------------------------------------

Solution solve(const Document& document) {
    Solution solution;

    const std::size_t turnout_count = document.turnouts.size();
    const std::size_t track_count = document.tracks.size();

    std::vector<Node> nodes;
    nodes.reserve(turnout_count + track_count);
    std::vector<std::size_t> turnout_node(turnout_count, 0);
    std::vector<std::size_t> track_node(track_count, 0);

    for (std::size_t i = 0; i < turnout_count; ++i) {
        turnout_node[i] = nodes.size();
        nodes.push_back(Node{NodeKind::Turnout, i, {}});
    }
    for (std::size_t i = 0; i < track_count; ++i) {
        track_node[i] = nodes.size();
        nodes.push_back(Node{NodeKind::Track, i, {}});
    }

    // Edges.
    for (std::size_t i = 0; i < turnout_count; ++i) {
        for (std::size_t k = 0; k < track_count; ++k) {
            if (document.tracks[k].id == document.turnouts[i].on) {
                nodes[turnout_node[i]].depends_on.push_back(track_node[k]);
                break;
            }
        }
        // a turnout standing against another one is laid after it: where it stands comes
        // off that one's frog
        if (document.turnouts[i].opposite != TurnoutId::none) {
            for (std::size_t k = 0; k < turnout_count; ++k) {
                if (document.turnouts[k].id == document.turnouts[i].opposite) {
                    nodes[turnout_node[i]].depends_on.push_back(turnout_node[k]);
                    break;
                }
            }
        }
    }
    for (std::size_t i = 0; i < track_count; ++i) {
        track_dependencies(document, document.tracks[i], turnout_node, track_node,
                           nodes[track_node[i]]);
    }

    // Topological order, Kahn's algorithm.
    std::vector<std::size_t> remaining(nodes.size(), 0);
    std::vector<std::vector<std::size_t>> dependents(nodes.size());
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        remaining[i] = nodes[i].depends_on.size();
        for (const std::size_t dependency : nodes[i].depends_on) {
            dependents[dependency].push_back(i);
        }
    }
    std::vector<std::size_t> order;
    order.reserve(nodes.size());
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        if (remaining[i] == 0) {
            order.push_back(i);
        }
    }
    for (std::size_t head = 0; head < order.size(); ++head) {
        for (const std::size_t next : dependents[order[head]]) {
            if (--remaining[next] == 0) {
                order.push_back(next);
            }
        }
    }

    // Whatever never came free is in a cycle.
    if (order.size() != nodes.size()) {
        for (std::size_t i = 0; i < nodes.size(); ++i) {
            if (remaining[i] == 0) {
                continue;
            }
            Diagnostic diagnostic;
            diagnostic.code = Code::CyclicDependency;
            switch (nodes[i].kind) {
                case NodeKind::Track:
                    diagnostic.track = document.tracks[nodes[i].index].id;
                    diagnostic.text = "tor „" + document.tracks[nodes[i].index].name +
                                      "” zależy sam od siebie w kółko";
                    break;
                case NodeKind::Turnout:
                    diagnostic.turnout = document.turnouts[nodes[i].index].id;
                    diagnostic.text = "rozjazd zależy sam od siebie w kółko";
                    break;
            }
            solution.diagnostics.push_back(std::move(diagnostic));
        }
    }

    // ---- walk the order ---------------------------------------------------
    for (const std::size_t index : order) {
        const Node& node = nodes[index];
        switch (node.kind) {
            case NodeKind::Turnout: {
                const TurnoutPlacement& placement = document.turnouts[node.index];
                SolvedTurnout solved;
                solved.id = placement.id;

                const TurnoutType* type = find_type(document, placement.type);
                if (type == nullptr) {
                    Diagnostic diagnostic;
                    diagnostic.code = Code::UnknownReference;
                    diagnostic.turnout = placement.id;
                    diagnostic.text = "rozjazd odwołuje się do typu „" + placement.type +
                                      "”, którego nie ma w katalogu";
                    solution.diagnostics.push_back(std::move(diagnostic));
                    solution.turnouts.push_back(std::move(solved));
                    break;
                }

                const SolvedTrack* through = find_track(solution, placement.on);

                // where it stands: what was authored, unless it stands against another
                // turnout - then it is worked out from that one's frog, so the two of them
                // move as the one piece of trackwork they are
                double station = placement.station;
                if (placement.opposite != TurnoutId::none) {
                    const SolvedTurnout* against = find_turnout(solution, placement.opposite);
                    double meeting = 0.0;
                    if (against == nullptr || !against->valid) {
                        Diagnostic diagnostic;
                        diagnostic.code = Code::UnknownReference;
                        diagnostic.turnout = placement.id;
                        diagnostic.text =
                            against == nullptr
                                ? "rozjazd stoi naprzeciw rozjazdu, którego nie ma"
                                : "rozjazd stoi naprzeciw rozjazdu, którego nie dało się położyć";
                        solution.diagnostics.push_back(std::move(diagnostic));
                        solution.turnouts.push_back(std::move(solved));
                        break;
                    }
                    if (through == nullptr || !meets_track(*through, against->frog, meeting)) {
                        Diagnostic diagnostic;
                        diagnostic.code = Code::StationOffTrack;
                        diagnostic.turnout = placement.id;
                        diagnostic.track = placement.on;
                        diagnostic.text =
                            "to, co wychodzi z iglicy naprzeciwka, nie spotyka tego toru: "
                            "przejście rozjazdowe nie ma gdzie stanąć";
                        solution.diagnostics.push_back(std::move(diagnostic));
                        solution.turnouts.push_back(std::move(solved));
                        break;
                    }
                    // the two frogs face each other across the międzytorze: this one has to
                    // stand where the line leaving the other one's frog passes, and its own
                    // frog sits off its track by as much as the other's does. so the station
                    // is settled by laying it and moving it along until its frog is on that
                    // line - which is one step on straight track and a few on a curve
                    const auto lay_at = [&](double where, plan::TurnoutGeometry& out, Pose& pr) {
                        if (!pose_at(*through, where, pr)) {
                            return false;
                        }
                        double here = 0.0;
                        double behind_here = 0.0;
                        double ahead_here = 0.0;
                        const bool steady_here = steady_room(*through, where, here, behind_here,
                                                             ahead_here);
                        double trial_bend = placement.bend_from_track
                                                ? (steady_here ? here : 0.0)
                                                : placement.bend;
                        Pose trial_at = pr;
                        if (!placement.facing) {
                            trial_at = Pose{trial_at.x, trial_at.y, -trial_at.hx, -trial_at.hy};
                            trial_bend = -trial_bend;
                        }
                        out = plan::lay_turnout(trial_at, to_domain(*type, placement.hand),
                                                trial_bend);
                        return out.valid;
                    };

                    plan::TurnoutGeometry trial;
                    Pose pr{};
                    if (!lay_at(meeting, trial, pr)) {
                        Diagnostic diagnostic;
                        diagnostic.code = Code::TurnoutInvalid;
                        diagnostic.turnout = placement.id;
                        diagnostic.text = "typ „" + type->name +
                                          "” nie daje się złożyć naprzeciw drugiego rozjazdu";
                        solution.diagnostics.push_back(std::move(diagnostic));
                        solution.turnouts.push_back(std::move(solved));
                        break;
                    }
                    // the first guess puts PR where the line crosses the track; from there the
                    // frog is walked onto the line itself
                    station = placement.facing ? meeting - trial.through_length
                                               : meeting + trial.through_length;
                    for (int pass = 0; pass < 6; ++pass) {
                        if (!lay_at(station, trial, pr)) {
                            break;
                        }
                        // how far the frog sits off the line leaving the other one's frog
                        const double off = (trial.frog.x - against->frog.x) * against->frog.hy -
                                           (trial.frog.y - against->frog.y) * against->frog.hx;
                        // and how much of that one metre along this track takes away
                        const double per_metre = pr.hx * against->frog.hy - pr.hy * against->frog.hx;
                        if (std::abs(per_metre) < 1e-9) {
                            break;
                        }
                        // its frog runs along the track whichever way the turnout opens, so
                        // the correction does not care about that either
                        const double step = off / per_metre;
                        station -= step;
                        if (std::abs(step) < 1e-9) {
                            break;
                        }
                    }
                }

                Pose at{};
                if (through == nullptr || !pose_at(*through, station, at)) {
                    Diagnostic diagnostic;
                    diagnostic.code = Code::StationOffTrack;
                    diagnostic.turnout = placement.id;
                    diagnostic.track = placement.on;
                    diagnostic.text = "rozjazd stoi na torze, którego nie da się położyć";
                    solution.diagnostics.push_back(std::move(diagnostic));
                    solution.turnouts.push_back(std::move(solved));
                    break;
                }
                // łukowanie: the turnout is bent onto whatever the track does under
                // it, unless it was asked to carry a bend of its own. Read the room
                // here too — both answers come off the same element
                double curvature = 0.0;
                double behind = 0.0;
                double ahead = 0.0;
                const bool steady = steady_room(*through, station, curvature, behind,
                                                ahead);
                double bend = placement.bend_from_track ? (steady ? curvature : 0.0)
                                                        : placement.bend;

                if (!placement.facing) {
                    at = Pose{at.x, at.y, -at.hx, -at.hy};
                    // laid against the way the track runs, a left curve is a right one
                    bend = -bend;
                }

                const plan::TurnoutGeometry geometry =
                    plan::lay_turnout(at, to_domain(*type, placement.hand), bend);
                if (!geometry.valid) {
                    Diagnostic diagnostic;
                    diagnostic.code = Code::TurnoutInvalid;
                    diagnostic.turnout = placement.id;
                    diagnostic.text =
                        bend != 0.0
                            ? "typu „" + type->name + "” nie da się wygiąć na łuk R " +
                                  num(std::abs(1.0 / bend)) +
                                  " m: łukowanie wymaga jednego promienia i szyn łączących, "
                                  "które je przyjmą"
                            : "typ „" + type->name +
                                  "” nie daje się złożyć: sprawdź skos, promienie i iglicę";
                    solution.diagnostics.push_back(std::move(diagnostic));
                    solution.turnouts.push_back(std::move(solved));
                    break;
                }

                // a turnout is one piece of geometry bent to the one radius under it, so it has to
                // stand on a single element and fit on it whole, or the drawing says something the
                // ground would not. facing turnouts eat the room ahead, trailing ones behind
                if (!steady) {
                    Diagnostic diagnostic;
                    diagnostic.code = Code::StationOffTrack;
                    diagnostic.turnout = placement.id;
                    diagnostic.track = placement.on;
                    diagnostic.text =
                        through->length > 0.0 && station > through->length + 1e-6
                            ? "rozjazd stoi " + num(station - through->length) +
                                  " m za końcem toru zasadniczego"
                            : "rozjazd stoi na krzywej przejściowej: nie ma jednego promienia, "
                              "na który dałoby się go wygiąć";
                    solution.diagnostics.push_back(std::move(diagnostic));
                } else if (const double room = placement.facing ? ahead : behind;
                           geometry.through_length > room + 1e-6) {
                    Diagnostic diagnostic;
                    diagnostic.code = Code::StationOffTrack;
                    diagnostic.turnout = placement.id;
                    diagnostic.track = placement.on;
                    diagnostic.text = "rozjazd wychodzi " + num(geometry.through_length - room) +
                                      " m poza odcinek toru, na którym stoi: potrzebuje " +
                                      num(geometry.through_length) + " m, a zostało " + num(room);
                    solution.diagnostics.push_back(std::move(diagnostic));
                }

                solved.valid = true;
                solved.station = station;
                solved.bend = geometry.bend;
                solved.diverging_curvature = geometry.diverging_curvature;
                solved.bend_angle = geometry.bend_angle;
                solved.start = at;
                solved.frog = geometry.frog;
                solved.tangent_front = geometry.tangent_front;
                solved.tangent_back = geometry.tangent_back;
                solved.diverging_length = geometry.diverging_length;
                solved.through_length = geometry.through_length;
                solved.has_blade = geometry.has_blade;
                solved.blade_station = geometry.blade_station;
                solved.nose_end = geometry.nose_end;
                solved.planing_length = geometry.planing_length;
                solved.blade_angle = geometry.blade_angle;
                solved.blade_residual = geometry.blade_residual;
                solved.angle_residual = geometry.angle_residual;
                solved.length_residual = geometry.length_residual;

                solved.marks = turnout_marks(geometry, at);

                Pose walker = at;
                for (const auto& segment : geometry.path) {
                    if (std::abs(segment.turn_in) > kEps) {
                        const double cos_t = std::cos(segment.turn_in);
                        const double sin_t = std::sin(segment.turn_in);
                        walker = Pose{walker.x, walker.y,
                                      walker.hx * cos_t - walker.hy * sin_t,
                                      walker.hx * sin_t + walker.hy * cos_t};
                    }

                    SolvedTurnoutSegment out_segment;
                    out_segment.start = walker;
                    out_segment.k0 = segment.k0;
                    out_segment.k1 = segment.k1;
                    out_segment.length = segment.length;
                    std::vector<geometry::XY> points;
                    walker = geometry::layout_segment(segment.k0, segment.k1,
                                                              segment.length, walker,
                                                              &points);
                    out_segment.part = segment.part;
                    out_segment.points.reserve(points.size());
                    for (const auto& point : points) {
                        out_segment.points.push_back(PlanPoint{point.x, point.y});
                    }
                    solved.path.push_back(std::move(out_segment));
                }

                // the list is laid as authored; whether it keeps the catalogue's
                // promises is said out loud rather than fitted away
                if (std::abs(geometry.angle_residual) > 1e-6 ||
                    (type->length > 0.0 && std::abs(geometry.length_residual) > 1e-3)) {
                    Diagnostic diagnostic;
                    diagnostic.code = Code::TurnoutNotClosed;
                    diagnostic.turnout = placement.id;
                    diagnostic.text =
                        "typ „" + type->name + "” nie domyka się: kąt o " +
                        num(geometry.angle_residual * 180.0 / std::numbers::pi) +
                        " st., długość o " + num(geometry.length_residual) + " m";
                    solution.diagnostics.push_back(std::move(diagnostic));
                }

                solution.turnouts.push_back(std::move(solved));
                break;
            }

            case NodeKind::Track: {
                const Track& track = document.tracks[node.index];
                Pose start{0.0, 0.0, 0.0, 1.0};
                bool anchored = true;

                if (const auto* pose = std::get_if<AtPose>(&track.anchor)) {
                    start = Pose{pose->x, pose->y, std::sin(pose->az), std::cos(pose->az)};
                } else if (const auto* at_port = std::get_if<AtPort>(&track.anchor)) {
                    const SolvedTurnout* turnout = find_turnout(solution, at_port->turnout);
                    if (turnout == nullptr || !turnout->valid) {
                        anchored = false;
                    } else {
                        start = at_port->port == Port::Frog ? turnout->frog : turnout->start;
                    }
                }

                if (!anchored) {
                    Diagnostic diagnostic;
                    diagnostic.code = Code::UnknownReference;
                    diagnostic.track = track.id;
                    diagnostic.text = "tor „" + track.name +
                                      "” jest zakotwiczony w czymś, czego nie ma "
                                      "albo czego nie dało się położyć";
                    solution.diagnostics.push_back(std::move(diagnostic));
                    SolvedTrack empty;
                    empty.id = track.id;
                    empty.complete = false;
                    solution.tracks.push_back(std::move(empty));
                    break;
                }

                // Laying is a walk, so a hold is resolved around it rather than
                // inside it: lay once to see which way the held element runs,
                // take its shape from the reference, lay again, and finally move
                // the whole chain onto the offset it is held at.
                const bool movable = std::holds_alternative<AtPose>(track.anchor);
                Track resolved = track;
                TrackOutcome outcome = lay_track(document, resolved, start);

                std::size_t held_index = resolved.elements.size();
                for (std::size_t e = 0; e < resolved.elements.size(); ++e) {
                    const auto* parallel = std::get_if<Parallel>(&resolved.elements[e].hold);
                    if (parallel == nullptr) {
                        continue;
                    }
                    const SolvedElement* reference =
                        find_element(solution, parallel->ref);
                    if (reference == nullptr || e >= outcome.track.elements.size()) {
                        Diagnostic diagnostic;
                        diagnostic.code = Code::UnknownReference;
                        diagnostic.track = track.id;
                        diagnostic.element = resolved.elements[e].id;
                        diagnostic.text =
                            "odcinek ma trzymać równoległość do czegoś, czego nie ma "
                            "albo czego nie dało się położyć";
                        solution.diagnostics.push_back(std::move(diagnostic));
                        continue;
                    }
                    const Held held = hold_against(*reference, parallel->offset,
                                                   outcome.track.elements[e].start);
                    if (!held.ok) {
                        Diagnostic diagnostic;
                        diagnostic.code = Code::DegenerateElement;
                        diagnostic.track = track.id;
                        diagnostic.element = resolved.elements[e].id;
                        diagnostic.text = held.why;
                        solution.diagnostics.push_back(std::move(diagnostic));
                        continue;
                    }
                    resolved.elements[e].kind = held.kind;
                    resolved.elements[e].radius = held.radius;
                    resolved.elements[e].hand = held.hand;
                    if (held_index == resolved.elements.size()) {
                        held_index = e;
                    } else if (movable) {
                        Diagnostic diagnostic;
                        diagnostic.code = Code::HoldConflict;
                        diagnostic.track = track.id;
                        diagnostic.element = resolved.elements[e].id;
                        diagnostic.text =
                            "tor „" + track.name +
                            "” trzyma równoległość dwoma odcinkami naraz; przystawiony "
                            "jest do pierwszego z nich";
                        solution.diagnostics.push_back(std::move(diagnostic));
                    }
                }

                if (held_index < resolved.elements.size()) {
                    outcome = lay_track(document, resolved, start);
                    if (held_index < outcome.track.elements.size()) {
                        const auto* parallel =
                            std::get_if<Parallel>(&resolved.elements[held_index].hold);
                        const SolvedElement* reference =
                            find_element(solution, parallel->ref);
                        const Held held =
                            hold_against(*reference, parallel->offset,
                                         outcome.track.elements[held_index].start);
                        if (held.ok && movable) {
                            const Pose moved =
                                rigid_move(start, outcome.track.elements[held_index].start,
                                           held.target);
                            outcome = lay_track(document, resolved, moved);
                        } else if (held.ok) {
                            // A track pinned to a turnout's port cannot be moved
                            // onto the offset, so the corner between the port and
                            // the held element has to take up the difference —
                            // the same fit as pulling a straight about by hand.
                            std::string why;
                            Skeleton skeleton;
                            Track fitted = resolved;
                            bool done = false;
                            if (resolved.elements[held_index].kind == Kind::Line &&
                                skeleton_of(resolved, outcome.track, skeleton, why)) {
                                std::size_t line_index = 0;
                                for (std::size_t e = 0; e < held_index; ++e) {
                                    if (resolved.elements[e].kind == Kind::Line) {
                                        ++line_index;
                                    }
                                }
                                if (line_index == 0) {
                                    why = "przed odcinkiem trzymającym równoległość nie ma "
                                          "łuku, który mógłby na nią wyprowadzić";
                                } else {
                                    skeleton.lines[line_index] =
                                        Support{held.target.x, held.target.y, held.target.hx,
                                                held.target.hy};
                                    if (line_index + 1 == skeleton.lines.size()) {
                                        skeleton.end_x = held.target.x;
                                        skeleton.end_y = held.target.y;
                                    }
                                    done = lay_along_skeleton(fitted, skeleton, why);
                                }
                            }
                            if (done) {
                                resolved = std::move(fitted);
                                outcome = lay_track(document, resolved, start);
                            } else if (std::abs(held.actual - parallel->offset) > 1e-3) {
                                Diagnostic diagnostic;
                                diagnostic.code = Code::OffsetNotHeld;
                                diagnostic.track = track.id;
                                diagnostic.element = resolved.elements[held_index].id;
                                diagnostic.text =
                                    "tor „" + track.name +
                                    "” jest przypięty do rozjazdu i nie da się go dopasować (" +
                                    why + "): odstęp wychodzi " + num(held.actual, 2) +
                                    " m zamiast " + num(parallel->offset, 2) + " m";
                                solution.diagnostics.push_back(std::move(diagnostic));
                            }
                        }
                    }
                }

                for (auto& diagnostic : outcome.diagnostics) {
                    solution.diagnostics.push_back(std::move(diagnostic));
                }
                solution.tracks.push_back(std::move(outcome.track));
                break;
            }
        }
    }

    // Anything left out by a cycle still gets an entry, so the host can index by
    // id without checking for holes.
    for (const auto& track : document.tracks) {
        if (find_track(solution, track.id) == nullptr) {
            SolvedTrack empty;
            empty.id = track.id;
            empty.complete = false;
            solution.tracks.push_back(std::move(empty));
        }
    }
    for (const auto& turnout : document.turnouts) {
        if (find_turnout(solution, turnout.id) == nullptr) {
            SolvedTurnout empty;
            empty.id = turnout.id;
            solution.turnouts.push_back(std::move(empty));
        }
    }

    return solution;
}

}  // namespace editor::plan
