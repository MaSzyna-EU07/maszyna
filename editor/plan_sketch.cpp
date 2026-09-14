/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/
module;
#include <algorithm>
#include <cmath>
#include <variant>
#include <cstdio>
#include <numbers>
#include <string>
#include <vector>

module eu07.editor.plan_sketch;

namespace editor::plan {
namespace {
constexpr double kEps = 1e-9;
}  // namespace

using geometry::Pose;


Corner fit_corner(double in_hx, double in_hy, double out_hx, double out_hy, double radius) {
    Corner corner;
    if (radius <= kEps) {
        return corner;
    }
    const double in_length = std::hypot(in_hx, in_hy);
    const double out_length = std::hypot(out_hx, out_hy);
    if (in_length < kEps || out_length < kEps) {
        return corner;
    }
    in_hx /= in_length;
    in_hy /= in_length;
    out_hx /= out_length;
    out_hy /= out_length;

    // On the short branch: a corner never turns further than it actually bends.
    const double deflection = std::atan2(in_hx * out_hy - in_hy * out_hx,
                                         in_hx * out_hx + in_hy * out_hy);
    const double turn = std::abs(deflection);
    if (turn < 1e-6 || turn > std::numbers::pi - 1e-6) {
        return corner;
    }

    corner.ok = true;
    corner.deflection = deflection;
    corner.hand = deflection > 0.0 ? 1 : -1;
    corner.tangent = radius * std::tan(0.5 * turn);
    corner.arc_length = radius * turn;
    return corner;
}

Element make_arc(Document& document, double radius, int hand, double length) {
    Element element;
    element.id = mint_element(document);
    element.kind = Kind::Arc;
    element.radius = radius;
    element.hand = hand >= 0 ? 1 : -1;
    element.length = length;
    return element;
}

Element make_line(Document& document, double length) {
    Element element;
    element.id = mint_element(document);
    element.kind = Kind::Line;
    element.length = length;
    return element;
}

Element make_clothoid(Document& document, double end_radius, int hand, double length) {
    Element element;
    element.id = mint_element(document);
    element.kind = Kind::Clothoid;
    element.radius = end_radius;
    element.hand = hand >= 0 ? 1 : -1;
    element.length = length;
    return element;
}


namespace {

std::string metres(double value) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.1f", value);
    return buffer;
}

/// Signed distance from a point to a line, positive to the left of travel.
double left_of(const Support& line, double x, double y) noexcept {
    return -(x - line.px) * line.dy + (y - line.py) * line.dx;
}

/// How far along a line a point sits.
double along(const Support& line, double x, double y) noexcept {
    return (x - line.px) * line.dx + (y - line.py) * line.dy;
}

}  // namespace

namespace {

/// What sits between two straights: one or more arcs turning the same way, with
/// transition curves easing into them, between them and out of them. One arc is
/// the everyday corner; several of them are a basket curve, and nothing here
/// treats that as a special case.
struct Group {
    std::vector<std::size_t> elements;  ///< in order, between the two straights
    std::vector<std::size_t> arcs;      ///< the arcs among them
    int hand{0};
};

/// Signed curvature an element ends on, the way the chain reads it.
double end_curvature(const Element& element) noexcept {
    if (element.kind == Kind::Line || element.radius <= kEps) {
        return 0.0;
    }
    return static_cast<double>(element.hand) / element.radius;
}

/// Cuts a chain into straights and the corners between them, or says why it is
/// not one. A corner may hang off either end, where there is no second straight
/// to fit it against: that is a track that starts or finishes mid-curve, which is
/// ordinary and is simply carried along as authored.
bool parse_chain(const Track& track, std::vector<std::size_t>& straights,
                 std::vector<Group>& groups, bool& has_head, std::string& why) {
    straights.clear();
    groups.clear();
    has_head = false;
    const std::size_t count = track.elements.size();
    if (count == 0) {
        why = "tor jest pusty";
        return false;
    }

    std::size_t i = 0;
    while (i < count) {
        if (track.elements[i].kind == Kind::Line) {
            if (!straights.empty() && straights.back() + 1 == i) {
                why = "dwie proste stykają się ze sobą — to jedna prosta";
                return false;
            }
            straights.push_back(i++);
            continue;
        }

        // A corner: everything up to the next straight, turning one way.
        Group group;
        const std::size_t first = i;
        while (i < count && track.elements[i].kind != Kind::Line) {
            if (track.elements[i].kind == Kind::Arc) {
                if (track.elements[i].hand == 0) {
                    why = "łuk nie mówi, w którą stronę skręca";
                    return false;
                }
                if (group.hand != 0 && track.elements[i].hand != group.hand) {
                    why = "łuki w jednym narożniku skręcają w różne strony — to nie jest "
                          "łuk koszowy, tylko esica, i nie ma jej co dopasowywać do dwóch "
                          "prostych";
                    return false;
                }
                group.hand = track.elements[i].hand;
                group.arcs.push_back(i);
            }
            group.elements.push_back(i++);
        }
        if (group.arcs.empty()) {
            why = "krzywa przejściowa nie wchodzi w żaden łuk — nie ma tu narożnika";
            return false;
        }

        // Every transition must ease to whatever it runs into: to the radius of
        // the arc after it, or out into the straight after it. Anything else is
        // a kink dressed up as an easement.
        for (const std::size_t at : group.elements) {
            if (track.elements[at].kind != Kind::Clothoid || at + 1 >= count) {
                continue;
            }
            const Element& next = track.elements[at + 1];
            if (next.kind == Kind::Line) {
                if (track.elements[at].radius > kEps) {
                    why = "krzywa przejściowa przed prostą musi wybiegać w prostą (R=0)";
                    return false;
                }
            } else if (std::abs(track.elements[at].radius - next.radius) > 1e-6 ||
                       track.elements[at].hand != next.hand) {
                why = "krzywa przejściowa nie schodzi się z łukiem: kończy się na R=" +
                      metres(track.elements[at].radius) + " m, a łuk ma R=" +
                      metres(next.radius) + " m";
                return false;
            }
        }

        if (first == 0) {
            // A corner before the first straight: it hangs off the anchor and is
            // carried along, not fitted.
            has_head = true;
        } else if (i >= count) {
            // And one after the last straight, which simply runs out.
            break;
        }
        groups.push_back(std::move(group));
    }

    if (straights.empty()) {
        why = "tor nie ma ani jednej prostej — nie ma na czym oprzeć dopasowania";
        return false;
    }
    return true;
}

/// Lays a corner on its own, from the origin heading along +x, and reports where
/// it ends up. The arc lengths are given; everything else is as authored.
Pose lay_group(const Track& track, const Group& group, const std::vector<double>& arc_length) {
    Pose pose{0.0, 0.0, 1.0, 0.0};
    double k_prev = 0.0;
    std::size_t arc_at = 0;
    for (const std::size_t at : group.elements) {
        const Element& element = track.elements[at];
        const double k1 = end_curvature(element);
        const double k0 = element.kind == Kind::Clothoid ? k_prev : k1;
        const double length =
            element.kind == Kind::Arc ? arc_length[arc_at++] : element.length;
        pose = geometry::layout_segment(k0, k1, length, pose, nullptr);
        k_prev = k1;
    }
    return pose;
}

/// The arc lengths a corner comes out with, and where it puts the chain down.
struct Fitted {
    std::vector<double> arc_length;
    double leaves{0.0};   ///< along the line in, where the straight gives way
    double arrives{0.0};  ///< along the line out, where the straight picks up
};

/// Fits one corner between two lines. @p keep_at is where it sits when the lines
/// are parallel and nothing else decides.
bool fit_group(const Track& track, const Group& group, const Support& in,
               const Support& out_line, double keep_at, std::size_t ordinal, Fitted& fitted,
               std::string& why) {
    const double side = group.hand >= 0 ? 1.0 : -1.0;

    // What the corner was drawn as: the transitions turn what they turn, the arcs
    // turn the rest. Both are read in the hand's convention, so both come out
    // positive for a corner going the way its arcs go.
    double spent = 0.0;
    double authored = 0.0;
    double k_prev = 0.0;
    for (const std::size_t at : group.elements) {
        const Element& element = track.elements[at];
        const double k1 = end_curvature(element);
        if (element.kind == Kind::Clothoid) {
            spent += side * 0.5 * (k_prev + k1) * element.length;
        } else {
            if (element.radius <= kEps) {
                why = "łuk w narożniku " + std::to_string(ordinal) + " nie ma promienia";
                return false;
            }
            authored += element.length / element.radius;
        }
        k_prev = k1;
    }
    const double as_drawn = spent + authored;

    // How far it has to turn now. The angle between two lines says that only to
    // within a full circle, so the turn taken is the one nearest to what was
    // drawn: a corner drawn at 240° stays about 240°, and one drawn at 10° stays
    // about 10° instead of looping the long way round to 350°.
    const double raw = std::atan2(in.dx * out_line.dy - in.dy * out_line.dx,
                                  in.dx * out_line.dx + in.dy * out_line.dy);
    double turn = side * raw;
    turn += 2.0 * std::numbers::pi *
            std::round((as_drawn - turn) / (2.0 * std::numbers::pi));
    if (turn <= kEps) {
        why = "narożnik " + std::to_string(ordinal) +
              " musiałby skręcać w drugą stronę niż jego łuki";
        return false;
    }

    // The transitions are authored, so what they turn is theirs; the arcs share
    // out what is left, keeping the proportions they were drawn with. A basket
    // curve therefore stays a basket curve — the same curve, tighter or wider,
    // not one arc quietly swallowing the others.
    const double left_for_arcs = turn - spent;
    if (left_for_arcs <= kEps) {
        why = "krzywe przejściowe w narożniku " + std::to_string(ordinal) +
              " zużywają cały zwrot — na łuki nie zostaje nic";
        return false;
    }

    fitted.arc_length.clear();
    fitted.arc_length.reserve(group.arcs.size());
    for (const std::size_t at : group.arcs) {
        const Element& arc = track.elements[at];
        const double share = authored > kEps ? (arc.length / arc.radius) / authored
                                             : 1.0 / static_cast<double>(group.arcs.size());
        fitted.arc_length.push_back(arc.radius * left_for_arcs * share);
    }

    // The corner is rigid now, so all that is left is sliding it along the line it
    // leaves until it lands on the line it joins.
    const Pose local = lay_group(track, group, fitted.arc_length);
    const double reach_x = local.x * in.dx - local.y * in.dy;
    const double reach_y = local.x * in.dy + local.y * in.dx;
    const double determinant = in.dx * out_line.dy - in.dy * out_line.dx;

    double at_line = 0.0;
    if (std::abs(determinant) < 1e-12) {
        // Parallel lines: the half circle, and the full one. Their spacing fixes
        // the geometry outright and says nothing about where along them the
        // corner sits, so it stays where it is.
        at_line = keep_at;
        const double miss = left_of(out_line, in.px + in.dx * at_line + reach_x,
                                    in.py + in.dy * at_line + reach_y);
        if (std::abs(miss) > 1e-6) {
            why = "proste przy narożniku " + std::to_string(ordinal) +
                  " są równoległe, więc ich rozstaw wyznacza geometrię: tak dopasowany "
                  "narożnik mija prostą o " + metres(std::abs(miss)) + " m";
            return false;
        }
    } else {
        at_line = left_of(out_line, in.px + reach_x, in.py + reach_y) / determinant;
    }

    fitted.leaves = at_line;
    fitted.arrives = along(out_line, in.px + in.dx * at_line + reach_x,
                           in.py + in.dy * at_line + reach_y);
    return true;
}

}  // namespace

void sync_joint(Track& track, std::size_t at) {
    if (at >= track.elements.size()) {
        return;
    }
    const Element& edited = track.elements[at];

    if (edited.kind == Kind::Arc) {
        // The transition that eases into this arc is the one before it: its end
        // radius is this arc's radius. What comes after the arc eases into
        // something else, and is none of this arc's business.
        if (at == 0 || track.elements[at - 1].kind != Kind::Clothoid) {
            return;
        }
        track.elements[at - 1].radius = edited.radius;
        track.elements[at - 1].hand = edited.hand;
        return;
    }

    if (edited.kind == Kind::Clothoid && edited.radius > kEps) {
        // A transition that ends curved ends on an arc, and that arc is it.
        if (at + 1 >= track.elements.size() ||
            track.elements[at + 1].kind != Kind::Arc) {
            return;
        }
        track.elements[at + 1].radius = edited.radius;
        track.elements[at + 1].hand = edited.hand;
    }
}

bool skeleton_of(const Track& track, const SolvedTrack& solved, Skeleton& out,
                 std::string& why) {
    out.lines.clear();
    if (solved.elements.size() != track.elements.size()) {
        why = "tor nie jest położony";
        return false;
    }
    std::vector<std::size_t> straights;
    std::vector<Group> groups;
    bool has_head = false;
    if (!parse_chain(track, straights, groups, has_head, why)) {
        return false;
    }
    for (const std::size_t i : straights) {
        const Pose& start = solved.elements[i].start;
        out.lines.push_back(Support{start.x, start.y, start.hx, start.hy});
    }
    out.start_x = solved.elements[straights.front()].start.x;
    out.start_y = solved.elements[straights.front()].start.y;
    out.end_x = solved.elements[straights.back()].end.x;
    out.end_y = solved.elements[straights.back()].end.y;
    out.pinned = !std::holds_alternative<AtPose>(track.anchor);
    out.begins = solved.start;
    return true;
}

bool lay_along_skeleton(Track& track, const Skeleton& skeleton, std::string& why) {
    std::vector<std::size_t> straights;
    std::vector<Group> groups;
    bool has_head = false;
    if (!parse_chain(track, straights, groups, has_head, why)) {
        return false;
    }
    if (skeleton.lines.size() != straights.size()) {
        why = "proste nie pasują do łańcucha";
        return false;
    }

    std::vector<Support> lines = skeleton.lines;
    // Where each corner leaves the line before it and lands on the line after it,
    // as distances along those lines. Nothing is written until every one of them
    // works out.
    std::vector<double> leaves(straights.size(), 0.0);
    std::vector<double> arrives(straights.size(), 0.0);
    std::vector<Fitted> fitted(straights.size() > 0 ? straights.size() - 1 : 0);

    arrives[0] = along(lines[0], skeleton.start_x, skeleton.start_y);

    if (has_head && skeleton.pinned) {
        // The chain leaves a frog, so its first curve starts where it starts and
        // ends where it ends: the line under the straight behind that curve is not
        // the one it was drawn on but wherever the curve now comes out. What the
        // change costs is then taken up by that straight's length, which is the
        // one thing here free to give.
        std::vector<double> as_authored;
        for (const std::size_t at : groups.front().arcs) {
            as_authored.push_back(track.elements[at].length);
        }
        const Pose local = lay_group(track, groups.front(), as_authored);
        const Pose& from = skeleton.begins;
        lines[0] = Support{from.x + local.x * from.hx - local.y * from.hy,
                           from.y + local.x * from.hy + local.y * from.hx,
                           local.hx * from.hx - local.hy * from.hy,
                           local.hx * from.hy + local.hy * from.hx};
        arrives[0] = 0.0;
    }
    leaves[straights.size() - 1] =
        along(lines[straights.size() - 1], skeleton.end_x, skeleton.end_y);

    // The corners with a straight on either side are the ones that get fitted. A
    // head or a tail corner has nothing on its far side, so it keeps what it was
    // given and the chain simply walks through it.
    const std::size_t first_interior = has_head ? 1u : 0u;
    for (std::size_t i = 0; i + 1 < straights.size(); ++i) {
        const Group& group = groups[first_interior + i];
        const double keep_at = arrives[i] + track.elements[straights[i]].length;
        if (!fit_group(track, group, lines[i], lines[i + 1], keep_at, i + 1, fitted[i], why)) {
            return false;
        }
        leaves[i] = fitted[i].leaves;
        arrives[i + 1] = fitted[i].arrives;
    }

    std::vector<double> straight_length(straights.size(), 0.0);
    for (std::size_t i = 0; i < straights.size(); ++i) {
        const double left = leaves[i] - arrives[i];
        if (left <= kEps) {
            why = "narożniki zachodzą na siebie na prostej " + std::to_string(i + 1) +
                  ": brakuje " + metres(-left) + " m";
            return false;
        }
        straight_length[i] = left;
    }

    // Where the chain has to start for all of that to stand where it stands. With
    // a corner hanging off the front, that is worked out by laying it and putting
    // its far end on the first straight.
    const Pose head_end{lines[0].px + lines[0].dx * arrives[0],
                        lines[0].py + lines[0].dy * arrives[0], lines[0].dx, lines[0].dy};
    Pose begins = head_end;
    if (has_head) {
        std::vector<double> as_authored;
        for (const std::size_t at : groups.front().arcs) {
            as_authored.push_back(track.elements[at].length);
        }
        const Pose local = lay_group(track, groups.front(), as_authored);
        // Undo the corner: rotate its local end onto the pose it has to reach,
        // and carry its local start along.
        const double cos_t = local.hx * head_end.hx + local.hy * head_end.hy;
        const double sin_t = local.hx * head_end.hy - local.hy * head_end.hx;
        // the corner's local start heads along +x, so turning it by the same angle
        // gives the heading the chain has to set off on
        begins = Pose{head_end.x - (local.x * cos_t - local.y * sin_t),
                      head_end.y - (local.x * sin_t + local.y * cos_t),
                      cos_t, sin_t};
    }

    if (std::holds_alternative<AtPose>(track.anchor)) {
        track.anchor = AtPose{begins.x, begins.y, std::atan2(begins.hx, begins.hy)};
    } else if (std::hypot(begins.x - skeleton.begins.x, begins.y - skeleton.begins.y) > 1e-6) {
        why = "tor jest przypięty do rozjazdu — jego początku nie da się przesunąć";
        return false;
    }

    for (std::size_t i = 0; i < straights.size(); ++i) {
        track.elements[straights[i]].length = straight_length[i];
    }
    for (std::size_t i = 0; i + 1 < straights.size(); ++i) {
        const Group& group = groups[first_interior + i];
        for (std::size_t k = 0; k < group.arcs.size(); ++k) {
            track.elements[group.arcs[k]].length = fitted[i].arc_length[k];
        }
    }
    return true;
}


// ---------------------------------------------------------------------------
// Cutting a chain in two
// ---------------------------------------------------------------------------

namespace {

/// The shortest piece worth leaving behind. A cut closer than this to a joint is
/// taken to be that joint, and one this close to either end of the track is not a
/// cut at all.
constexpr double kMinPiece = 0.05;

/// Where a station falls in a chain: the element it lands in, and how far into
/// it. @c local == 0 means it landed on the joint in front of @c index, so that
/// element and everything after it moves whole and nothing is divided.
struct Cut {
    std::size_t index{0};
    double local{0.0};
    double station{0.0};
};

/// The stations a turnout standing on its through track covers, PR to KR. A
/// facing one eats the room ahead of its station, a trailing one the room behind.
void turnout_span(const Document& document, const Solution& solution,
                  const TurnoutPlacement& placement, double& lo, double& hi) {
    double through = 0.0;
    if (const SolvedTurnout* solved = find_turnout(solution, placement.id);
        solved != nullptr && solved->valid) {
        through = solved->through_length;
    } else if (const TurnoutType* type = find_type(document, placement.type); type != nullptr) {
        through = type->length;
    }
    lo = placement.facing ? placement.station : placement.station - through;
    hi = placement.facing ? placement.station + through : placement.station;
}

[[nodiscard]] bool locate_cut(const Track& track, const SolvedTrack& solved,
                             const Document& document, const Solution& solution,
                             double station, Cut& out, std::string& why) {
    if (solved.elements.empty()) {
        why = "tor jest pusty";
        return false;
    }
    if (!solved.complete) {
        why = "tor nie jest złożony do końca — najpierw popraw to, co mówią diagnostyki";
        return false;
    }
    if (station <= kMinPiece || station >= solved.length - kMinPiece) {
        why = "przeciąć można tor, nie jego koniec: wskaż punkt między początkiem a końcem";
        return false;
    }

    for (const TurnoutPlacement& placement : document.turnouts) {
        if (placement.on != track.id) {
            continue;
        }
        double lo = 0.0;
        double hi = 0.0;
        turnout_span(document, solution, placement, lo, hi);
        if (station > lo + kMinPiece && station < hi - kMinPiece) {
            why = "w tym miejscu stoi rozjazd „" + placement.type +
                  "”: rozjazd jest jedną geometrią na jednym elemencie, więc jego połowa nie "
                  "jest rozjazdem";
            return false;
        }
    }

    double walked = 0.0;
    std::size_t index = solved.elements.size() - 1;
    double local = solved.elements.back().length;
    for (std::size_t i = 0; i < solved.elements.size(); ++i) {
        const double length = solved.elements[i].length;
        if (station < walked + length) {
            index = i;
            local = station - walked;
            break;
        }
        walked += length;
    }
    // a cut all but on a joint is that joint: dividing an element into a piece
    // nobody can see is how a chain fills up with elements of zero length
    if (local <= kMinPiece) {
        local = 0.0;
    } else if (solved.elements[index].length - local <= kMinPiece) {
        ++index;
        local = 0.0;
    }

    // a chain always begins running straight — there is nowhere in the document to
    // say otherwise — so the far side may not start in the middle of a transition
    const bool starts_on_transition =
        local > 0.0 ? solved.elements[index].kind == Kind::Clothoid
                    : index < solved.elements.size() &&
                          solved.elements[index].kind == Kind::Clothoid &&
                          std::abs(solved.elements[index].k0) > kEps;
    if (starts_on_transition) {
        why = "tor nie umie zacząć się w środku krzywej przejściowej — przeciąć go można na "
              "prostej albo na łuku";
        return false;
    }

    out.index = index;
    out.local = local;
    out.station = station;
    return true;
}

/// Carries out a located cut. Everything past it lands in a new track, which is
/// put straight after the one it came off.
TrackId apply_cut(Document& document, const SolvedTrack& solved, TrackId track, const Cut& cut) {
    Track* authored = find_track(document, track);

    // the pose the far side starts from is laid, not read off the drawn polyline:
    // sampled points sit on chords, and a chord is a tenth of a millimetre short of
    // the arc it stands for — enough to leave a visible kink at the cut
    const SolvedElement& divided = solved.elements[std::min(cut.index, solved.elements.size() - 1)];
    Pose at = divided.start;
    if (cut.local > 0.0) {
        const double k_at_cut =
            divided.k0 + (divided.k1 - divided.k0) * (cut.local / divided.length);
        at = geometry::layout_segment(divided.k0, k_at_cut, cut.local, divided.start, nullptr);
    }

    Track second;
    second.id = mint_track(document);
    second.name = authored->name + "-2";
    second.anchor = AtPose{at.x, at.y, std::atan2(at.hx, at.hy)};

    if (cut.local > 0.0) {
        // the element the cut lands in gives up its tail to a copy of itself. an
        // arc and a straight are the same shape however long they are, and a
        // transition was refused, so there is nothing else to work out
        Element tail = authored->elements[cut.index];
        tail.id = mint_element(document);
        tail.length = authored->elements[cut.index].length - cut.local;
        second.elements.push_back(tail);
    }
    for (std::size_t i = cut.index + (cut.local > 0.0 ? 1 : 0); i < authored->elements.size(); ++i) {
        second.elements.push_back(authored->elements[i]);
    }

    if (cut.local > 0.0) {
        authored->elements[cut.index].length = cut.local;
        authored->elements.resize(cut.index + 1);
    } else {
        authored->elements.resize(cut.index);
    }

    // a turnout past the cut stands on the new track now, at the same place on the
    // ground: its station is measured from the new beginning
    const TrackId second_id = second.id;
    for (TurnoutPlacement& placement : document.turnouts) {
        if (placement.on == track && placement.station > cut.station) {
            placement.on = second_id;
            placement.station -= cut.station;
        }
    }

    const auto after = std::find_if(document.tracks.begin(), document.tracks.end(),
                                    [track](const Track& candidate) {
                                        return candidate.id == track;
                                    });
    document.tracks.insert(after + 1, std::move(second));
    return second_id;
}

}  // namespace

JointOutcome insert_joint(Document& document, const Solution& solution, TrackId track,
                         double station) {
    JointOutcome outcome;
    Track* authored = find_track(document, track);
    const SolvedTrack* solved = find_track(solution, track);
    if (authored == nullptr || solved == nullptr) {
        outcome.why = "nie ma takiego toru";
        return outcome;
    }
    if (solved->elements.empty()) {
        outcome.why = "tor jest pusty";
        return outcome;
    }
    if (!solved->complete) {
        outcome.why = "tor nie jest złożony do końca — najpierw popraw to, co mówią diagnostyki";
        return outcome;
    }
    if (station <= kMinPiece || station >= solved->length - kMinPiece) {
        outcome.why = "złącze idzie w tor, nie w jego koniec: wskaż punkt między początkiem a końcem";
        return outcome;
    }

    // a turnout is one piece of geometry laid on one element, so a joint under it would
    // leave it standing on two
    for (const TurnoutPlacement& placement : document.turnouts) {
        if (placement.on != track) {
            continue;
        }
        double lo = 0.0;
        double hi = 0.0;
        turnout_span(document, solution, placement, lo, hi);
        if (station > lo + kMinPiece && station < hi - kMinPiece) {
            outcome.why = "w tym miejscu stoi rozjazd „" + placement.type +
                          "”: leży na jednym odcinku i złącze pod nim rozcięłoby go na dwa";
            return outcome;
        }
    }

    double walked = 0.0;
    std::size_t index = 0;
    double local = 0.0;
    for (std::size_t i = 0; i < solved->elements.size(); ++i) {
        const double length = solved->elements[i].length;
        if (station < walked + length) {
            index = i;
            local = station - walked;
            break;
        }
        walked += length;
    }
    if (local <= kMinPiece || solved->elements[index].length - local <= kMinPiece) {
        outcome.why = "tu już jest złącze";
        return outcome;
    }
    if (index >= authored->elements.size()) {
        outcome.why = "tego odcinka nie ma w dokumencie";
        return outcome;
    }

    Element& near = authored->elements[index];
    if (near.kind == Kind::Line) {
        outcome.why = "dwie proste stykające się ze sobą to jedna prosta — złącze na prostej nic "
                      "nie mówi, a dopasowanie takiego łańcucha nie przeczyta";
        return outcome;
    }

    const SolvedElement& laid = solved->elements[index];
    Element far = near;
    far.id = mint_element(document);
    far.length = near.length - local;
    near.length = local;
    if (near.kind == Kind::Clothoid) {
        // a transition's radius is the radius it ends on: the near half now ends at the
        // curvature the curve had reached here, and the far half goes on to where the
        // whole one was going. the curvature at the new joint is the one that was
        // always there, so nothing on the ground moves
        const double k_at_joint = laid.k0 + (laid.k1 - laid.k0) * (local / laid.length);
        near.radius = std::abs(k_at_joint) > kEps ? 1.0 / std::abs(k_at_joint) : 0.0;
        near.hand = k_at_joint > kEps ? 1 : (k_at_joint < -kEps ? -1 : 0);
    }
    // a hold belongs to the element that was authored with it, and it is the near half
    // that keeps that element's id; the far half is new and stands free
    far.hold = Free{};
    authored->elements.insert(authored->elements.begin() + static_cast<std::ptrdiff_t>(index) + 1,
                              far);

    outcome.added = far.id;
    outcome.ok = true;
    return outcome;
}

CutOutcome cut_track(Document& document, const Solution& solution, TrackId track, double from,
                       double to) {
    CutOutcome outcome;
    const Track* authored = find_track(document, track);
    const SolvedTrack* solved = find_track(solution, track);
    if (authored == nullptr || solved == nullptr) {
        outcome.why = "nie ma takiego toru";
        return outcome;
    }
    if (from > to) {
        std::swap(from, to);
    }
    if (to - from <= kMinPiece) {
        outcome.why = "wycinany odcinek nie ma długości: wskaż dwa różne punkty";
        return outcome;
    }

    // both cuts are checked before either is made, so a refusal leaves the drawing
    // exactly as it was
    Cut far;
    Cut near;
    if (!locate_cut(*authored, *solved, document, solution, to, far, outcome.why) ||
        !locate_cut(*authored, *solved, document, solution, from, near, outcome.why)) {
        return outcome;
    }
    for (const TurnoutPlacement& placement : document.turnouts) {
        if (placement.on != track) {
            continue;
        }
        double lo = 0.0;
        double hi = 0.0;
        turnout_span(document, solution, placement, lo, hi);
        if (hi > from && lo < to) {
            outcome.why = "w wycinanym odcinku stoi rozjazd „" + placement.type +
                          "”: usuń go najpierw, sam z niczego nie zniknie";
            return outcome;
        }
    }

    // the far cut first: the near one is measured from the same beginning, and the
    // geometry in front of it does not move, so its place in the chain still holds
    const TrackId beyond = apply_cut(document, *solved, track, far);
    const TrackId middle = apply_cut(document, *solved, track, near);
    document.tracks.erase(std::find_if(document.tracks.begin(), document.tracks.end(),
                                       [middle](const Track& candidate) {
                                           return candidate.id == middle;
                                       }));

    outcome.second = beyond;
    outcome.ok = true;
    return outcome;
}

}  // namespace editor::plan
