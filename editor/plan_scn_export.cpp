/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/
module;
#include <ostream>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <sstream>
#include <string>
#include <vector>

module eu07.editor.plan_scn_export;

namespace editor::plan::io {
namespace {

using editor::plan::Document;
using editor::plan::Kind;
using editor::plan::PlanPoint;
using editor::plan::SolvedElement;
using editor::plan::SolvedTrack;
using editor::plan::SolvedTurnout;
using editor::plan::TurnoutPlacement;

/// What `TTrack::Equal` calls one point, and so what the simulator will wire
/// together. Everything the exporter writes has to land well inside it.
constexpr double kJoin = 0.02;
/// Shorter than this and there is no piece worth writing.
constexpr double kShortest = 0.05;

struct World {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

/// One path of a track node: two ends and the control offsets from them, in the
/// simulator's coordinates and already in the units the file wants.
struct Bezier {
    World p1{}, cv1{}, cv2{}, p2{};
    double radius{0.0};
};

/// A pose on a sampled axis: where it is and which way it runs.
struct Step {
    PlanPoint at{};
    double tx{1.0};
    double ty{0.0};
};

// ---------------------------------------------------------------------------
// Sampled axes
// ---------------------------------------------------------------------------

double poly_length(const std::vector<PlanPoint>& points) {
    double along = 0.0;
    for (std::size_t i = 1; i < points.size(); ++i) {
        along += std::hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
    }
    return along;
}

bool poly_step(const std::vector<PlanPoint>& points, double along, Step& out) {
    if (points.size() < 2) {
        return false;
    }
    double walked = 0.0;
    for (std::size_t i = 1; i < points.size(); ++i) {
        const double dx = points[i].x - points[i - 1].x;
        const double dy = points[i].y - points[i - 1].y;
        const double step = std::hypot(dx, dy);
        if (step < 1e-12) {
            continue;
        }
        if (walked + step >= along || i + 1 == points.size()) {
            const double t = std::clamp((along - walked) / step, 0.0, 1.0);
            out.at = PlanPoint{points[i - 1].x + dx * t, points[i - 1].y + dy * t};
            out.tx = dx / step;
            out.ty = dy / step;
            return true;
        }
        walked += step;
    }
    return false;
}

/// The turnout's diverging path as one axis. The pieces are laid end to end, so
/// each one begins where the last left off and the join is dropped rather than
/// written twice.
std::vector<PlanPoint> diverging_axis(const SolvedTurnout& turnout) {
    std::vector<PlanPoint> points;
    for (const auto& segment : turnout.path) {
        for (const auto& point : segment.points) {
            if (!points.empty() &&
                std::hypot(point.x - points.back().x, point.y - points.back().y) < 1e-9) {
                continue;
            }
            points.push_back(point);
        }
    }
    return points;
}

// ---------------------------------------------------------------------------
// Bézier
// ---------------------------------------------------------------------------

World to_scn(const PlanPoint& plan, const ScnExportOptions& options) {
    return {plan.x - options.origin_east, options.rail_y,
            -(plan.y - options.origin_north)};
}

/// A plan tangent as a control offset: the simulator's x is east and its z runs
/// south, and the file wants the offset from the endpoint rather than the point.
World offset_scn(double tx, double ty, double length) {
    const double norm = std::hypot(tx, ty);
    if (norm < 1e-12) {
        return {0.0, 0.0, 0.0};
    }
    return {tx / norm * length, 0.0, -ty / norm * length};
}

/// The control offset that puts a cubic on a circle of @p radius through @p phi.
double arc_handle(double radius, double phi) {
    const double turn = std::clamp(phi, 1e-6, std::numbers::pi);
    return (4.0 / 3.0) * std::max(radius, 1.0) * std::tan(turn * 0.25);
}

double deflection(const Step& from, const Step& to) {
    const double cross = from.tx * to.ty - from.ty * to.tx;
    const double dot = from.tx * to.tx + from.ty * to.ty;
    return std::abs(std::atan2(cross, dot));
}

/// A straight, or an arc of a known radius: the handles follow from the geometry
/// and nothing has to be fitted.
Bezier arc_bezier(const Step& from, const Step& to, double length, double radius,
                  const ScnExportOptions& options) {
    Bezier bezier;
    bezier.p1 = to_scn(from.at, options);
    bezier.p2 = to_scn(to.at, options);
    if (radius < 1.0) {
        return bezier;  // straight: no offsets, no radius
    }
    const double phi = std::clamp(length / radius, 1e-6, std::numbers::pi);
    const double handle = arc_handle(radius, phi);
    bezier.cv1 = offset_scn(from.tx, from.ty, handle);
    bezier.cv2 = offset_scn(-to.tx, -to.ty, handle);
    bezier.radius = radius;
    return bezier;
}

/// The handle lengths that put a cubic closest to a sampled axis, ends and
/// tangents being already fixed. Least squares in the two lengths, which is what
/// a clothoid — and a whole turnout, which is several pieces under one curve —
/// needs, there being no one radius to take them from.
bool fit_handles(const std::vector<PlanPoint>& points, const PlanPoint& p0,
                 const PlanPoint& p3, const Step& from, const Step& to, double& out_first,
                 double& out_last) {
    const double span = poly_length(points);
    if (span < 1e-6) {
        return false;
    }
    const int samples = std::max(8, static_cast<int>(std::ceil(span / 2.0)));
    double m00 = 0.0, m01 = 0.0, m11 = 0.0, b0 = 0.0, b1 = 0.0;
    for (int i = 0; i <= samples; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(samples);
        Step step{};
        if (!poly_step(points, span * t, step)) {
            continue;
        }
        const double omt = 1.0 - t;
        const double c0 = 3.0 * omt * omt * t;
        const double c1 = 3.0 * omt * t * t;
        const double ax = omt * omt * (1.0 + 2.0 * t) * p0.x + t * t * (3.0 - 2.0 * t) * p3.x;
        const double ay = omt * omt * (1.0 + 2.0 * t) * p0.y + t * t * (3.0 - 2.0 * t) * p3.y;
        const double rx = step.at.x - ax;
        const double ry = step.at.y - ay;
        const double a0x = c0 * from.tx, a0y = c0 * from.ty;
        const double a1x = -c1 * to.tx, a1y = -c1 * to.ty;
        m00 += a0x * a0x + a0y * a0y;
        m01 += a0x * a1x + a0y * a1y;
        m11 += a1x * a1x + a1y * a1y;
        b0 += a0x * rx + a0y * ry;
        b1 += a1x * rx + a1y * ry;
    }
    const double det = m00 * m11 - m01 * m01;
    if (std::abs(det) < 1e-12) {
        return false;
    }
    const double first = (m11 * b0 - m01 * b1) / det;
    const double last = (-m01 * b0 + m00 * b1) / det;
    if (first < 0.05 || last < 0.05) {
        return false;
    }
    out_first = std::clamp(first, 0.05, span);
    out_last = std::clamp(last, 0.05, span);
    return true;
}

PlanPoint bezier_at(const PlanPoint& p0, const PlanPoint& c1, const PlanPoint& c2,
                    const PlanPoint& p3, double t) {
    const double omt = 1.0 - t;
    const double w0 = omt * omt * omt;
    const double w1 = 3.0 * omt * omt * t;
    const double w2 = 3.0 * omt * t * t;
    const double w3 = t * t * t;
    return PlanPoint{w0 * p0.x + w1 * c1.x + w2 * c2.x + w3 * p3.x,
                     w0 * p0.y + w1 * c1.y + w2 * c2.y + w3 * p3.y};
}

double to_segment(const PlanPoint& point, const PlanPoint& a, const PlanPoint& b) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double square = dx * dx + dy * dy;
    const double t = square > 1e-18
                         ? std::clamp(((point.x - a.x) * dx + (point.y - a.y) * dy) / square,
                                      0.0, 1.0)
                         : 0.0;
    return std::hypot(point.x - (a.x + dx * t), point.y - (a.y + dy * t));
}

/// How far the cubic strays from the axis it stands for. A turnout is several
/// pieces of geometry — przediglicowy, iglica, łuk, szyna łącząca — under one
/// curve in the scenery, so what that costs is measured against the curve
/// itself rather than against samples of it, which would only measure how far
/// apart the samples were put.
double curve_error(const std::vector<PlanPoint>& points, const PlanPoint& p0,
                   const PlanPoint& p3, const Step& from, const Step& to, double first,
                   double last) {
    if (points.size() < 3) {
        return 0.0;
    }
    const PlanPoint c1{p0.x + from.tx * first, p0.y + from.ty * first};
    const PlanPoint c2{p3.x - to.tx * last, p3.y - to.ty * last};

    constexpr int kSamples = 96;
    std::vector<PlanPoint> curve;
    curve.reserve(kSamples + 1);
    for (int i = 0; i <= kSamples; ++i) {
        curve.push_back(bezier_at(p0, c1, c2, p3, static_cast<double>(i) / kSamples));
    }

    double worst = 0.0;
    for (const auto& point : points) {
        double nearest = std::numeric_limits<double>::max();
        for (std::size_t i = 1; i < curve.size(); ++i) {
            nearest = std::min(nearest, to_segment(point, curve[i - 1], curve[i]));
        }
        worst = std::max(worst, nearest);
    }
    return worst;
}

/// Least squares pairs the axis with the curve by parameter, and a cubic does
/// not run at an even rate, so the handles it lands on are near but not the
/// closest. This walks downhill from there on the thing that actually matters:
/// how far the curve ever gets from the axis.
void refine_handles(const std::vector<PlanPoint>& points, const PlanPoint& p0,
                    const PlanPoint& p3, const Step& from, const Step& to, double& first,
                    double& last, double& out_error) {
    double best = curve_error(points, p0, p3, from, to, first, last);
    const double span = poly_length(points);
    double step = std::max(0.25, span * 0.1);
    int budget = 400;
    while (step > 0.005 && budget > 0) {
        bool better = false;
        constexpr int kMoves[6][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, -1}};
        for (const auto& move : kMoves) {
            const double trial_first = std::max(0.05, first + move[0] * step);
            const double trial_last = std::max(0.05, last + move[1] * step);
            const double error =
                curve_error(points, p0, p3, from, to, trial_first, trial_last);
            --budget;
            if (error < best - 1e-6) {
                best = error;
                first = trial_first;
                last = trial_last;
                better = true;
                break;
            }
        }
        if (!better) {
            step *= 0.5;
        }
    }
    out_error = best;
}

/// A cubic through the ends of a sampled axis, tangent to it at both, fitted to
/// what runs between. Falls back on thirds, which is what a cubic with nothing
/// better to go on does anyway.
Bezier fitted_bezier(const std::vector<PlanPoint>& points, const Step& from, const Step& to,
                     double radius, const ScnExportOptions& options, double& out_error) {
    const double span = poly_length(points);
    double first = span / 3.0;
    double last = span / 3.0;
    if (!fit_handles(points, from.at, to.at, from, to, first, last)) {
        const double phi = deflection(from, to);
        if (radius >= 1.0 && phi > 1e-6) {
            first = last = arc_handle(radius, phi);
        }
    }
    refine_handles(points, from.at, to.at, from, to, first, last, out_error);

    Bezier bezier;
    bezier.p1 = to_scn(from.at, options);
    bezier.p2 = to_scn(to.at, options);
    bezier.cv1 = offset_scn(from.tx, from.ty, first);
    bezier.cv2 = offset_scn(-to.tx, -to.ty, last);
    bezier.radius = radius;
    return bezier;
}

// ---------------------------------------------------------------------------
// Writing
// ---------------------------------------------------------------------------

std::string fixed(double value, int decimals = 4) {
    std::ostringstream text;
    text.setf(std::ios::fixed);
    text.precision(decimals);
    text << value;
    return text.str();
}

void write_point(std::ostream& out, const World& point, const char* trailer) {
    out << fixed(point.x) << ' ' << fixed(point.y) << ' ' << fixed(point.z) << trailer
        << '\n';
}

/// One `node ... track` with as many paths as its type takes: one for a plain
/// track, two for a switch — the straight route first, which is the one the
/// simulator holds a switch in when nothing has thrown it.
///
/// The two material slots do not mean the same thing on both. On plain track the
/// second is the podsypka with its sleepers, laid under the rails. On a switch
/// there is no ballast to lay — that comes as a model beside it — and the slot
/// draws the rails of the second route instead, so it takes the rail material
/// again. Giving it a ballast texture there puts sleepers up the diverging rails,
/// which is what the game's own sceneries avoid by writing the rails twice.
void write_track_node(std::ostream& out, const std::string& name, const char* type,
                      double length, const std::vector<Bezier>& paths) {
    const bool points = std::string_view{type} == "switch";
    out << "node -1 0 " << name << " track " << type << ' ' << fixed(length, 3)
        << (points ? " 1.435 0.24 15.0 20 2 flat vis\n" : " 1.435 0.15 25.0 20 0 flat vis\n");
    out << (points ? " rail_screw_used1 6 rail_screw_used1 0.2 2.75 2.5\n"
                   : " rail_screw_used1 6 1435mm/tpbps-new2 0.2 0.5 1.1\n");
    for (const auto& path : paths) {
        write_point(out, path.p1, " 0");
        write_point(out, path.cv1, "");
        write_point(out, path.cv2, "");
        write_point(out, path.p2, " 0");
        out << fixed(path.radius) << '\n';
    }
    out << "endtrack\n\n";
}

// ---------------------------------------------------------------------------
// Names
// ---------------------------------------------------------------------------

/// Whatever the drawing called it, in what a scenery file can be asked to hold.
std::string sanitised(const std::string& name) {
    std::string out;
    out.reserve(name.size());
    for (const char letter : name) {
        const bool plain = (letter >= '0' && letter <= '9') ||
                           (letter >= 'a' && letter <= 'z') ||
                           (letter >= 'A' && letter <= 'Z') || letter == '_' ||
                           letter == '-';
        out.push_back(plain ? letter : '_');
    }
    while (!out.empty() && out.back() == '_') {
        out.pop_back();
    }
    return out;
}

/// Names have to be unique: an event or a dispatcher naming a track has to reach
/// one of them.
class Names {
  public:
    std::string mint(std::string wanted) {
        if (wanted.empty()) {
            wanted = "tor";
        }
        std::string candidate = wanted;
        int suffix = 1;
        while (std::find(m_taken.begin(), m_taken.end(), candidate) != m_taken.end()) {
            candidate = wanted + '_' + std::to_string(++suffix);
        }
        m_taken.push_back(candidate);
        return candidate;
    }

  private:
    std::vector<std::string> m_taken;
};

// ---------------------------------------------------------------------------
// Turnouts on their through track
// ---------------------------------------------------------------------------

/// A turnout as the through track sees it: the stretch of it the switch node
/// takes over, and which end of that stretch the blades stand at.
struct Span {
    const SolvedTurnout* turnout{nullptr};
    const TurnoutPlacement* placement{nullptr};
    double from{0.0};  ///< lower station of the stretch
    double to{0.0};    ///< upper station
    bool facing{true};
};

/// What the catalogue calls the turnout's radius: the one the longest curved
/// piece runs at, which is the łuk itself and not an easement into it.
double catalogue_radius(const plan::TurnoutType& type) {
    double longest = 0.0;
    double radius = 0.0;
    for (const auto& piece : type.pieces) {
        if (piece.radius_start > 0.0 && piece.length > longest) {
            longest = piece.length;
            radius = piece.radius_start;
        }
    }
    return radius;
}

/// The sterowanie a scenery hangs off a switch: the lantern that turns with it,
/// the drive's sound, and the events a dispatcher throws it by — `zwr01+` to the
/// straight route, `zwr01-` to the diverging one. The game keeps one include per
/// geometry and hand, so the nearest of them to what was drawn is asked for; a
/// MaSzyna without that file says so in its log and loads the rest.
std::string switch_include(double radius, int hand) {
    struct Family {
        double radius;
        const char* suffix;
    };
    static constexpr Family kFamilies[]{
        {190.0, "25r190"}, {300.0, "34r300"}, {500.0, "41r500"}, {1200.0, "65r1200"}};
    const Family* nearest = &kFamilies[0];
    for (const auto& family : kFamilies) {
        if (std::abs(family.radius - radius) < std::abs(nearest->radius - radius)) {
            nearest = &family;
        }
    }
    return std::string(hand >= 0 ? "zwrl" : "zwrp") + nearest->suffix + ".inc";
}

/// Which way the switch faces, in the degrees an include's `rotate` takes: zero
/// along the simulator's +z, turning towards +x.
double heading_degrees(const geometry::Pose& pose) {
    return std::atan2(pose.hx, -pose.hy) * 180.0 / std::numbers::pi;
}

/// The radius the diverging route runs at. Bent, it is the one the bend worked
/// out to; otherwise the curve piece's own.
double diverging_radius(const SolvedTurnout& turnout) {
    if (std::abs(turnout.diverging_curvature) > 1e-9) {
        return 1.0 / std::abs(turnout.diverging_curvature);
    }
    for (const auto& segment : turnout.path) {
        if (std::abs(segment.k0) > 1e-9) {
            return 1.0 / std::abs(segment.k0);
        }
    }
    return 0.0;
}

}  // namespace

// ---------------------------------------------------------------------------

ScnExportOptions resolve_scn_origin(const editor::plan::Solution& solution,
                                    ScnExportOptions options) {
    if (std::abs(options.origin_east) >= 1.0 || std::abs(options.origin_north) >= 1.0) {
        return options;
    }
    for (const auto& track : solution.tracks) {
        if (track.elements.empty() || track.elements.front().points.empty()) {
            continue;
        }
        const auto& point = track.elements.front().points.front();
        if (std::abs(point.x) > 10000.0 || std::abs(point.y) > 10000.0) {
            options.origin_east = point.x;
            options.origin_north = point.y;
        }
        break;
    }
    return options;
}

ScnExportResult export_scn(const Document& document,
                           const editor::plan::Solution& solution,
                           const ScnExportOptions& options, std::ostream& out) {
    const auto opt = resolve_scn_origin(solution, options);
    ScnExportResult result;
    result.origin_east = opt.origin_east;
    result.origin_north = opt.origin_north;

    Names names;
    // whatever was written first, for when nothing is long enough to stand a train on
    std::string spare_track;
    // every endpoint written, to be told afterwards which of them the simulator
    // will and will not join up
    struct Ending {
        PlanPoint at{};
        std::string node;
        bool is_switch{false};
    };
    std::vector<Ending> endings;

    out << "//$n plan\n";
    out << "//$d origin " << fixed(opt.origin_east, 3) << ' '
        << fixed(opt.origin_north, 3) << "\n\n";

    for (const auto& track : solution.tracks) {
        const auto* authored = find_track(document, track.id);
        const std::string base =
            names.mint(sanitised(authored != nullptr ? authored->name : std::string{}));

        // ---- where the switches sit on this track -------------------------
        std::vector<Span> spans;
        for (const auto& placement : document.turnouts) {
            if (placement.on != track.id) {
                continue;
            }
            const SolvedTurnout* turnout = find_turnout(solution, placement.id);
            if (turnout == nullptr || !turnout->valid || turnout->path.empty()) {
                result.warnings.push_back("rozjazd „" + placement.type +
                                          "” nie dał się położyć i nie wyszedł do scn");
                continue;
            }
            Span span;
            span.turnout = turnout;
            span.placement = &placement;
            span.facing = placement.facing;
            span.from = placement.facing ? placement.station
                                         : placement.station - turnout->through_length;
            span.to = placement.facing ? placement.station + turnout->through_length
                                       : placement.station;
            if (span.from < -kJoin || span.to > track.length + kJoin) {
                result.warnings.push_back(
                    "rozjazd „" + placement.type + "” na torze „" + base +
                    "” wychodzi poza tor: zwrotnica wyszła, ale tor jej nie podpiera");
            }
            spans.push_back(span);
        }
        std::sort(spans.begin(), spans.end(),
                  [](const Span& left, const Span& right) { return left.from < right.from; });
        for (std::size_t i = 1; i < spans.size(); ++i) {
            if (spans[i].from < spans[i - 1].to - kJoin) {
                result.warnings.push_back("dwa rozjazdy na torze „" + base +
                                          "” zachodzą na siebie: w scn nakładają się "
                                          "dwie zwrotnice");
            }
        }

        // ---- the plain track, in the gaps the switches leave --------------
        int piece = 0;
        const auto write_range = [&](double from, double to) {
            double walked = 0.0;
            for (const SolvedElement& element : track.elements) {
                const double span = poly_length(element.points);
                const double starts = walked;
                const double ends = walked + span;
                walked = ends;
                if (element.points.size() < 2 || span < kShortest) {
                    continue;
                }
                const double first = std::max(from, starts);
                const double last = std::min(to, ends);
                if (last - first < kShortest) {
                    continue;
                }

                const double radius =
                    element.kind == Kind::Arc && std::abs(element.k0) > 1e-9
                        ? 1.0 / std::abs(element.k0)
                        : 0.0;
                // an arc has to be cut into cubics no one of which turns too far
                // to stay a circle
                const int cuts =
                    radius >= 1.0
                        ? std::max(1, static_cast<int>(std::ceil((last - first) / radius /
                                                                opt.max_arc_angle)))
                        : 1;
                const double cut = (last - first) / static_cast<double>(cuts);
                for (int c = 0; c < cuts; ++c) {
                    const double s0 = first - starts + cut * static_cast<double>(c);
                    const double s1 = c + 1 == cuts ? last - starts : s0 + cut;
                    Step from_step{}, to_step{};
                    if (!poly_step(element.points, s0, from_step) ||
                        !poly_step(element.points, s1, to_step)) {
                        continue;
                    }
                    const std::string name = names.mint(base + "_n" + std::to_string(++piece));
                    Bezier bezier;
                    if (element.kind == Kind::Clothoid) {
                        std::vector<PlanPoint> sampled;
                        constexpr int kSamples = 24;
                        for (int i = 0; i <= kSamples; ++i) {
                            Step step{};
                            if (poly_step(element.points,
                                          s0 + (s1 - s0) * static_cast<double>(i) / kSamples,
                                          step)) {
                                sampled.push_back(step.at);
                            }
                        }
                        double strayed = 0.0;
                        bezier = fitted_bezier(sampled, from_step, to_step, 0.0, opt, strayed);
                    } else {
                        bezier = arc_bezier(from_step, to_step, s1 - s0, radius, opt);
                    }
                    write_track_node(out, name, "normal", s1 - s0, {bezier});
                    endings.push_back(Ending{from_step.at, name, false});
                    endings.push_back(Ending{to_step.at, name, false});
                    // the train stands a way in from the end of a track, so the first
                    // piece long enough to hold it is the one it is put on
                    if (result.first_track_name.empty() &&
                        s1 - s0 >= opt.trainset_offset + 1.0) {
                        result.first_track_name = name;
                    }
                    if (spare_track.empty()) {
                        spare_track = name;
                    }
                    ++result.tracks;
                }
            }
        };

        // ---- the switches themselves --------------------------------------
        const auto write_switch = [&](const Span& span) {
            const SolvedTurnout& turnout = *span.turnout;
            // PR: where the blades are, and where both routes have to start from
            // the same point or the simulator will not join them
            const Step blade{PlanPoint{turnout.start.x, turnout.start.y}, turnout.start.hx,
                             turnout.start.hy};

            const double exit_station = span.facing ? span.to : span.from;
            geometry::Pose exit_pose{};
            if (!pose_at(track, exit_station, exit_pose)) {
                result.warnings.push_back("rozjazd „" + span.placement->type + "” na torze „" +
                                          base + "” nie ma końca na torze zasadniczym");
                return;
            }
            const double sense = span.facing ? 1.0 : -1.0;
            const Step straight_end{PlanPoint{exit_pose.x, exit_pose.y},
                                    sense * exit_pose.hx, sense * exit_pose.hy};

            const double bend_radius =
                std::abs(turnout.bend) > 1e-9 ? 1.0 / std::abs(turnout.bend) : 0.0;
            const Bezier straight = arc_bezier(blade, straight_end, turnout.through_length,
                                               bend_radius, opt);

            const auto axis = diverging_axis(turnout);
            const Step diverging_start{
                axis.empty() ? blade.at : axis.front(),
                turnout.path.front().start.hx, turnout.path.front().start.hy};
            const Step frog{PlanPoint{turnout.frog.x, turnout.frog.y}, turnout.frog.hx,
                            turnout.frog.hy};
            const double radius = diverging_radius(turnout);
            double strayed = 0.0;
            Bezier diverging =
                fitted_bezier(axis, diverging_start, frog, radius, opt, strayed);
            // the two routes leave one point: the fit is over the laid path, but
            // what is written is the blade pose the straight route also starts on
            diverging.p1 = straight.p1;

            if (strayed > 0.05) {
                result.warnings.push_back(
                    "rozjazd „" + span.placement->type +
                    "”: jedna krzywa scn odbiega od położonego toru zwrotnego o " +
                    fixed(strayed, 3) + " m");
            }

            // zwr01, zwr02: what a scenery calls its switches, and what the sterowanie
            // includes hang their events off — `zwr01+` throws the one named zwr01
            char number[16];
            std::snprintf(number, sizeof(number), "zwr%02d", result.switches + 1);
            const std::string name = names.mint(number);
            // a switch is one curve per route in the scenery and a turnout is
            // four pieces of geometry, so what the shortcut costs is written
            // down beside it rather than left to be found by measuring
            out << "// " << span.placement->type << ", tor zwrotny R " << fixed(radius, 1)
                << " m, odchyłka od położonej osi " << fixed(strayed, 3) << " m\n";
            write_track_node(out, name, "switch", turnout.through_length,
                             {straight, diverging});
            if (opt.switch_control) {
                const auto* type = find_type(document, span.placement->type);
                const double nominal = type != nullptr ? catalogue_radius(*type) : radius;
                out << "include "
                    << switch_include(nominal, span.placement->hand) << ' ' << name << ' '
                    << fixed(straight.p1.x, 4) << " 0.0 " << fixed(straight.p1.z, 4) << ' '
                    << fixed(heading_degrees(turnout.start), 1) << " end\n";
                // t and shift+t at the blades: the game has no switch key of its own,
                // it looks for a launcher standing close enough to the camera. the
                // includes carry one commented out, so the scenery brings its own
                out << "node -1 0 " << name << "_kl eventlauncher " << fixed(straight.p1.x, 4)
                    << " 0.0 " << fixed(straight.p1.z, 4) << ' '
                    << fixed(opt.switch_key_reach, 1) << " t 0 " << name << '+' << ' ' << name
                    << "- end\n\n";
            }
            endings.push_back(Ending{blade.at, name, true});
            endings.push_back(Ending{straight_end.at, name, true});
            endings.push_back(Ending{frog.at, name, true});
            ++result.switches;
        };

        double cursor = 0.0;
        for (const auto& span : spans) {
            write_range(cursor, std::max(cursor, span.from));
            write_switch(span);
            cursor = std::max(cursor, span.to);
        }
        write_range(cursor, track.length);
    }

    // Nothing links nodes but their endpoints standing together, and only plain
    // track goes looking for a neighbour. Two switches meeting head on is a
    // drawing the scenery cannot hold, so it is said rather than left to be
    // found by driving into it.
    for (std::size_t i = 0; i < endings.size(); ++i) {
        if (!endings[i].is_switch) {
            continue;
        }
        for (std::size_t k = i + 1; k < endings.size(); ++k) {
            if (!endings[k].is_switch || endings[k].node == endings[i].node) {
                continue;
            }
            if (std::abs(endings[i].at.x - endings[k].at.x) < kJoin &&
                std::abs(endings[i].at.y - endings[k].at.y) < kJoin) {
                result.warnings.push_back(
                    "zwrotnice „" + endings[i].node + "” i „" + endings[k].node +
                    "” stykają się końcami: symulator łączy tylko przez zwykły tor, "
                    "wstaw między nie odcinek");
            }
        }
    }

    if (result.first_track_name.empty()) {
        result.first_track_name = spare_track;
    }
    if (opt.trainset) {
        out << "FirstInit\n\n";
        if (!result.first_track_name.empty()) {
            out << "trainset none " << result.first_track_name << ' '
                << fixed(opt.trainset_offset, 1) << " 0\n";
            out << "node -1 0 plan_eu07 dynamic PKP\\303E_V1 303E-EP-TV-424-HIST "
                   "303E-EP-TV 0 headdriver 35.WH25 0 enddynamic\n";
            out << "endtrainset\n";
        }
    }
    return result;
}

}  // namespace editor::plan::io
