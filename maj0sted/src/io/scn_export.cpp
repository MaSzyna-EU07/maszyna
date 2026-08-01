#include "maj0sted/io/scn_export.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <sstream>
#include <vector>

namespace maj0sted::io {
namespace {

using maj0sted::editor::NiweletaPolys;
using maj0sted::editor::PlanPoint;
using maj0sted::editor::PlanPolyline;

struct AxisElement {
    int kind{0};
    int straight_index{-1};
    int gap{-1};
    int element_index{-1};
    double length{0.0};
    double radius_start{0.0};
    double radius_end{0.0};
    std::vector<PlanPoint> points;
};

struct WorldXYZ {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

std::vector<AxisElement> recover_axis(const NiweletaPolys& solved) {
    std::vector<AxisElement> out;
    const auto& polys = solved.polylines;
    for (std::size_t i = 0; i < polys.size();) {
        const auto& a = polys[i];
        if (a.points.size() < 2) {
            ++i;
            continue;
        }
        if (i + 1 < polys.size()) {
            const auto& b = polys[i + 1];
            if (b.straight_index == a.straight_index && b.gap == a.gap &&
                b.element_index == a.element_index &&
                b.points.size() == a.points.size()) {
                AxisElement el;
                el.kind = a.kind;
                el.straight_index = a.straight_index;
                el.gap = a.gap;
                el.element_index = a.element_index;
                el.length = a.length;
                el.radius_start = a.radius_start;
                el.radius_end = a.radius_end;
                el.points.resize(a.points.size());
                for (std::size_t k = 0; k < a.points.size(); ++k) {
                    el.points[k] = {(a.points[k].x + b.points[k].x) * 0.5,
                                    (a.points[k].y + b.points[k].y) * 0.5};
                }
                out.push_back(std::move(el));
                i += 2;
                continue;
            }
        }
        AxisElement el;
        el.kind = a.kind;
        el.straight_index = a.straight_index;
        el.gap = a.gap;
        el.element_index = a.element_index;
        el.length = a.length;
        el.radius_start = a.radius_start;
        el.radius_end = a.radius_end;
        el.points = a.points;
        out.push_back(std::move(el));
        ++i;
    }
    return out;
}

WorldXYZ to_scn(double east, double north, double height, double ox, double oy) {
    return {east - ox, height, -(north - oy)};
}

void plan_tangent_to_scn(double de, double dn, double& out_x, double& out_z) {
    const double len = std::hypot(de, dn);
    if (len < 1e-12) {
        out_x = 0.0;
        out_z = 1.0;
        return;
    }
    out_x = de / len;
    out_z = -dn / len;
}

bool axis_pose_at(const std::vector<PlanPoint>& pts, double along, PlanPoint& out_p,
                  double& out_tx, double& out_ty) {
    if (pts.size() < 2) {
        return false;
    }
    double a = 0.0;
    for (std::size_t i = 1; i < pts.size(); ++i) {
        const double dx = pts[i].x - pts[i - 1].x;
        const double dy = pts[i].y - pts[i - 1].y;
        const double seg = std::hypot(dx, dy);
        if (a + seg >= along || i + 1 == pts.size()) {
            const double t = seg > 1e-12 ? std::clamp((along - a) / seg, 0.0, 1.0) : 0.0;
            out_p = {pts[i - 1].x + dx * t, pts[i - 1].y + dy * t};
            if (seg > 1e-12) {
                out_tx = dx / seg;
                out_ty = dy / seg;
            } else {
                out_tx = 1.0;
                out_ty = 0.0;
            }
            return true;
        }
        a += seg;
    }
    return false;
}

double axis_total_length(const std::vector<PlanPoint>& pts) {
    double along = 0.0;
    for (std::size_t i = 1; i < pts.size(); ++i) {
        along += std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
    }
    return along;
}

double circular_bezier_handle(double radius, double phi) {
    const double p = std::clamp(phi, 1e-6, std::numbers::pi);
    return (4.0 / 3.0) * std::max(radius, 1.0) * std::tan(p * 0.25);
}

bool fit_clothoid_bezier_handles(const std::vector<PlanPoint>& pts, double s0,
                                 double s1, const PlanPoint& p0, const PlanPoint& p3,
                                 double t0x, double t0y, double t1x, double t1y,
                                 double& out_l0, double& out_l1) {
    const double span = s1 - s0;
    if (span < 1e-6) {
        return false;
    }
    const int samples = std::max(8, static_cast<int>(std::ceil(span / 2.0)));
    double m00 = 0.0, m01 = 0.0, m11 = 0.0, b0 = 0.0, b1 = 0.0;
    for (int i = 0; i <= samples; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(samples);
        PlanPoint q{};
        double tx = 1.0, ty = 0.0;
        if (!axis_pose_at(pts, s0 + span * t, q, tx, ty)) {
            continue;
        }
        const double omt = 1.0 - t;
        const double c0 = 3.0 * omt * omt * t;
        const double c1 = 3.0 * omt * t * t;
        const double ax =
            omt * omt * (1.0 + 2.0 * t) * p0.x + t * t * (3.0 - 2.0 * t) * p3.x;
        const double ay =
            omt * omt * (1.0 + 2.0 * t) * p0.y + t * t * (3.0 - 2.0 * t) * p3.y;
        const double rx = q.x - ax;
        const double ry = q.y - ay;
        const double a0x = c0 * t0x, a0y = c0 * t0y;
        const double a1x = -c1 * t1x, a1y = -c1 * t1y;
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
    const double l0 = (m11 * b0 - m01 * b1) / det;
    const double l1 = (-m01 * b0 + m00 * b1) / det;
    if (l0 < 0.05 || l1 < 0.05) {
        return false;
    }
    out_l0 = std::clamp(l0, 0.05, span);
    out_l1 = std::clamp(l1, 0.05, span);
    return true;
}

void clothoid_handles_from_radii(double len, double phi, double r0, double r1,
                                 double& out_l0, double& out_l1) {
    auto handle = [&](double r) {
        if (r < 1.0) {
            return len / 3.0;
        }
        return circular_bezier_handle(r, phi);
    };
    out_l0 = handle(r0);
    out_l1 = handle(r1);
}

void write_scn_track(std::ostream& out, const std::string& name, const WorldXYZ& p1,
                     const WorldXYZ& cv1, const WorldXYZ& p2, const WorldXYZ& cv2,
                     double length, double radius) {
    out << "node -1 0 " << name << " track normal " << length
        << " 1.435 0.15 25.0 20 0 flat vis\n";
    out << " rail_screw_used1 6 1435mm/tpbps-new2 0.2 0.5 1.1\n";
    auto fmt = [](double v) {
        std::ostringstream s;
        s.setf(std::ios::fixed);
        s.precision(4);
        s << v;
        return s.str();
    };
    out << fmt(p1.x) << ' ' << fmt(p1.y) << ' ' << fmt(p1.z) << " 0\n";
    out << fmt(cv1.x) << ' ' << fmt(cv1.y) << ' ' << fmt(cv1.z) << '\n';
    out << fmt(cv2.x) << ' ' << fmt(cv2.y) << ' ' << fmt(cv2.z) << '\n';
    out << fmt(p2.x) << ' ' << fmt(p2.y) << ' ' << fmt(p2.z) << " 0\n";
    out << fmt(radius) << '\n';
    out << "endtrack\n\n";
}

}  // namespace

ScnExportOptions resolve_scn_origin(const std::vector<NiweletaPolys>& solved,
                                    ScnExportOptions options) {
    if (std::abs(options.origin_east) >= 1.0 ||
        std::abs(options.origin_north) >= 1.0) {
        return options;
    }
    for (const auto& s : solved) {
        const auto axis = recover_axis(s);
        if (!axis.empty() && !axis.front().points.empty()) {
            const auto& p = axis.front().points.front();
            if (std::abs(p.x) > 10000.0 || std::abs(p.y) > 10000.0) {
                options.origin_east = p.x;
                options.origin_north = p.y;
            }
            break;
        }
    }
    return options;
}

ScnExportResult export_scn(const std::vector<NiweletaPolys>& solved,
                           const ScnExportOptions& options, std::ostream& out) {
    const auto opt = resolve_scn_origin(solved, options);
    ScnExportResult result;
    result.origin_east = opt.origin_east;
    result.origin_north = opt.origin_north;

    out << "//$n plan\n";
    out << "//$d origin " << opt.origin_east << ' ' << opt.origin_north << "\n\n";

    const double kRailY = opt.rail_y;
    const double kMaxArcAngle = opt.max_arc_angle;

    for (std::size_t n = 0; n < solved.size(); ++n) {
        const auto axis = recover_axis(solved[n]);
        for (std::size_t ei = 0; ei < axis.size(); ++ei) {
            const auto& el = axis[ei];
            if (el.points.size() < 2) {
                continue;
            }
            const double total =
                el.length > 1e-6 ? el.length : axis_total_length(el.points);
            if (total < 0.05) {
                continue;
            }

            auto curvature_at = [&](double along) {
                const double k0 = el.radius_start > 1.0 ? 1.0 / el.radius_start : 0.0;
                const double k1 = el.radius_end > 1.0 ? 1.0 / el.radius_end : 0.0;
                const double t =
                    total > 1e-9 ? std::clamp(along / total, 0.0, 1.0) : 0.0;
                return k0 + (k1 - k0) * t;
            };
            auto radius_from_k = [](double k) {
                return std::abs(k) > 1e-9 ? 1.0 / std::abs(k) : 0.0;
            };

            auto emit_piece = [&](double s0, double s1, double piece_len,
                                  double radius, const char* tag) {
                PlanPoint p0{}, p1{};
                double t0x = 1.0, t0y = 0.0, t1x = 1.0, t1y = 0.0;
                if (!axis_pose_at(el.points, s0, p0, t0x, t0y) ||
                    !axis_pose_at(el.points, s1, p1, t1x, t1y)) {
                    return;
                }
                const auto w0 = to_scn(p0.x, p0.y, kRailY, opt.origin_east,
                                       opt.origin_north);
                const auto w1 = to_scn(p1.x, p1.y, kRailY, opt.origin_east,
                                       opt.origin_north);
                double tx0 = 0.0, tz0 = 0.0, tx1 = 0.0, tz1 = 0.0;
                plan_tangent_to_scn(t0x, t0y, tx0, tz0);
                plan_tangent_to_scn(t1x, t1y, tx1, tz1);

                WorldXYZ cv1{}, cv2{};
                double radius_out = 0.0;
                if (el.kind == 1 && radius >= 1.0) {
                    const double phi = std::clamp(piece_len / std::max(radius, 1.0),
                                                  1e-6, std::numbers::pi);
                    const double handle = circular_bezier_handle(radius, phi);
                    cv1 = {tx0 * handle, 0.0, tz0 * handle};
                    cv2 = {-tx1 * handle, 0.0, -tz1 * handle};
                    radius_out = radius;
                } else if (el.kind == 2) {
                    const double cross = t0x * t1y - t0y * t1x;
                    const double dot = t0x * t1x + t0y * t1y;
                    const double phi = std::clamp(std::abs(std::atan2(cross, dot)),
                                                  1e-6, std::numbers::pi);
                    const double r0 = radius_from_k(curvature_at(s0));
                    const double r1 = radius_from_k(curvature_at(s1));
                    double l0 = piece_len / 3.0;
                    double l1 = piece_len / 3.0;
                    if (!fit_clothoid_bezier_handles(el.points, s0, s1, p0, p1, t0x,
                                                     t0y, t1x, t1y, l0, l1)) {
                        clothoid_handles_from_radii(piece_len, phi, r0, r1, l0, l1);
                    }
                    cv1 = {tx0 * l0, 0.0, tz0 * l0};
                    cv2 = {-tx1 * l1, 0.0, -tz1 * l1};
                }

                std::ostringstream name;
                name << "plan_n" << n;
                if (el.straight_index >= 0) {
                    name << "_s" << el.straight_index;
                } else {
                    name << "_g" << el.gap << "_e" << el.element_index;
                }
                name << '_' << tag << '_' << result.tracks;
                const auto track_name = name.str();
                if (result.first_track_name.empty()) {
                    result.first_track_name = track_name;
                }
                write_scn_track(out, track_name, w0, cv1, w1, cv2, piece_len,
                                radius_out);
                ++result.tracks;
            };

            if (el.kind == 0 || (el.kind == 1 && el.radius_start < 1.0)) {
                emit_piece(0.0, total, total, 0.0, "str");
                continue;
            }
            if (el.kind == 2) {
                emit_piece(0.0, total, total, 0.0, "kp");
                continue;
            }

            const double r = std::max(el.radius_start, el.radius_end);
            const double phi = total / std::max(r, 1.0);
            const int pieces =
                std::max(1, static_cast<int>(std::ceil(phi / kMaxArcAngle)));
            const double piece_len = total / static_cast<double>(pieces);
            for (int p = 0; p < pieces; ++p) {
                const double s0 = piece_len * static_cast<double>(p);
                const double s1 =
                    p + 1 == pieces ? total : piece_len * static_cast<double>(p + 1);
                emit_piece(s0, s1, s1 - s0, r, "arc");
            }
        }
    }

    out << "FirstInit\n\n";
    if (!result.first_track_name.empty()) {
        out << "trainset none " << result.first_track_name << " 5 0\n";
        out << "node -1 0 plan_eu07 dynamic PKP\\303E_V1 303E-EP-TV-424-HIST "
               "303E-EP-TV 0 headdriver 35.WH25 0 enddynamic\n";
        out << "endtrainset\n";
    }
    return result;
}

}  // namespace maj0sted::io
