#pragma once

// Geometria trasy: próbkowanie co N metrów wzdłuż osi toru + odległość boczna (pas 1 m).

#include <eu07/geo/puwg1992.hpp>
#include <eu07/geo/spatial_grid.hpp>
#include <eu07/nmt/profile_types.hpp>
#include <eu07/scene/document.hpp>
#include <eu07/scene/node/track.hpp>
#include <eu07/scene/track_routes.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace eu07::nmt {

struct TrackLineSegment {
    scene::Vec3 p1{};
    scene::Vec3 p2{};
};

struct BezierSpan {
    scene::Vec3 p1;
    scene::Vec3 cv1;
    scene::Vec3 cv2;
    scene::Vec3 p2;
    double lengthM = 0.0;
    double chainageStartM = 0.0;
};

struct RouteGeometry {
    std::string label;
    double totalLengthM = 0.0;
    std::vector<BezierSpan> spans;
    std::vector<TrackLineSegment> segments;
    geo::SpatialGrid<25> grid;
};

struct RouteGeometryOptions {
    double distanceSegmentLengthM = 2.0;
    int minSegmentsPerBezier = 4;
    int maxSegmentsPerBezier = 300;
};

namespace detail {

inline double vecLength(const scene::Vec3& v) noexcept {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

inline scene::Vec3 vecAdd(const scene::Vec3& a, const scene::Vec3& b) noexcept {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

inline scene::Vec3 vecScale(const scene::Vec3& v, const double s) noexcept {
    return {v.x * s, v.y * s, v.z * s};
}

inline scene::Vec3 cubicBezier(
    const double t,
    const scene::Vec3& p1,
    const scene::Vec3& cv1,
    const scene::Vec3& cv2,
    const scene::Vec3& p2) {
    const scene::Vec3 c1 = vecAdd(p1, cv1);
    const scene::Vec3 c2 = vecAdd(p2, cv2);
    const double u = 1.0 - t;
    const double tt = t * t;
    const double uu = u * u;
    const double uuu = uu * u;
    const double ttt = tt * t;

    scene::Vec3 p = vecScale(p1, uuu);
    p = vecAdd(p, vecScale(c1, 3.0 * uu * t));
    p = vecAdd(p, vecScale(c2, 3.0 * u * tt));
    p = vecAdd(p, vecScale(p2, ttt));
    return p;
}

inline double approximateBezierLength(
    const scene::Vec3& p1,
    const scene::Vec3& cv1,
    const scene::Vec3& cv2,
    const scene::Vec3& p2) {
    const scene::Vec3 c1 = vecAdd(p1, cv1);
    const scene::Vec3 c2 = vecAdd(p2, cv2);
    return vecLength(cv1) + vecLength(vecAdd(c2, vecScale(c1, -1.0))) + vecLength(cv2);
}

inline void appendBezierSpansAndSegments(
    const scene::TrackBezier& bez,
    const scene::Vec3& originOffset,
    const RouteGeometryOptions& opts,
    const double chainageStart,
    std::vector<BezierSpan>& spans,
    std::vector<TrackLineSegment>& segments,
    double& outTotalLength) {
    BezierSpan span;
    span.p1 = scene::detail::track_routes::addOffset(bez.p1, originOffset);
    span.p2 = scene::detail::track_routes::addOffset(bez.p2, originOffset);
    span.cv1 = bez.cv1;
    span.cv2 = bez.cv2;
    span.lengthM = approximateBezierLength(span.p1, span.cv1, span.cv2, span.p2);
    span.chainageStartM = chainageStart;
    spans.push_back(span);
    outTotalLength += span.lengthM;

    int pieces = static_cast<int>(span.lengthM / opts.distanceSegmentLengthM);
    if (pieces < opts.minSegmentsPerBezier) {
        pieces = opts.minSegmentsPerBezier;
    }
    if (pieces > opts.maxSegmentsPerBezier) {
        pieces = opts.maxSegmentsPerBezier;
    }

    scene::Vec3 prev = span.p1;
    for (int i = 1; i <= pieces; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(pieces);
        const scene::Vec3 curr = cubicBezier(t, span.p1, span.cv1, span.cv2, span.p2);
        segments.push_back({prev, curr});
        prev = curr;
    }
}

inline const std::vector<scene::TrackBezier>* beziersForSegment(
    const scene::SceneDocument& doc,
    const scene::TrackSegmentEndpoints& segment) {
    switch (segment.ref.kind) {
    case scene::TrackSegmentKind::Normal:
        return &doc.nodeTrackNormal[segment.ref.index].beziers;
    case scene::TrackSegmentKind::Switch:
        return &doc.nodeTrackSwitch[segment.ref.index].beziers;
    case scene::TrackSegmentKind::Road:
        return &doc.nodeTrackRoad[segment.ref.index].beziers;
    case scene::TrackSegmentKind::Cross:
        return &doc.nodeTrackCross[segment.ref.index].beziers;
    case scene::TrackSegmentKind::Other:
        return &doc.nodeTrackOther[segment.ref.index].beziers;
    }
    return nullptr;
}

inline scene::Vec3 headerOffsetForSegment(
    const scene::SceneDocument& doc,
    const scene::TrackSegmentEndpoints& segment) {
    switch (segment.ref.kind) {
    case scene::TrackSegmentKind::Normal:
        return doc.nodeTrackNormal[segment.ref.index].header.originOffset;
    case scene::TrackSegmentKind::Switch:
        return doc.nodeTrackSwitch[segment.ref.index].header.originOffset;
    case scene::TrackSegmentKind::Road:
        return doc.nodeTrackRoad[segment.ref.index].header.originOffset;
    case scene::TrackSegmentKind::Cross:
        return doc.nodeTrackCross[segment.ref.index].header.originOffset;
    case scene::TrackSegmentKind::Other:
        return doc.nodeTrackOther[segment.ref.index].header.originOffset;
    }
    return {};
}

inline scene::Vec3 pointOnBezierSpan(const BezierSpan& span, const double localT) {
    return cubicBezier(localT, span.p1, span.cv1, span.cv2, span.p2);
}

} // namespace detail

[[nodiscard]] inline RouteGeometry buildRouteGeometry(
    const scene::SceneDocument& doc,
    const scene::TrackRouteBuildResult& routes,
    const std::size_t routeIndex,
    const RouteGeometryOptions& opts = {}) {
    RouteGeometry geometry;
    if (routeIndex >= routes.routes.size()) {
        return geometry;
    }

    const scene::TrackRoute& route = routes.routes[routeIndex];
    geometry.label = route.label;
    geometry.spans.reserve(route.links.size() * 4);
    geometry.segments.reserve(route.links.size() * 32);

    double chainage = 0.0;
    for (const scene::TrackRouteLink& link : route.links) {
        const scene::TrackSegmentEndpoints& segment = routes.segments[link.segmentIndex];
        const std::vector<scene::TrackBezier>* beziers = detail::beziersForSegment(doc, segment);
        if (beziers == nullptr || beziers->empty()) {
            continue;
        }

        const scene::Vec3 origin = detail::headerOffsetForSegment(doc, segment);
        const std::vector<scene::TrackBezier>& list = *beziers;
        if (link.reversed) {
            for (auto it = list.rbegin(); it != list.rend(); ++it) {
                scene::TrackBezier flipped = *it;
                std::swap(flipped.p1, flipped.p2);
                std::swap(flipped.cv1, flipped.cv2);
                detail::appendBezierSpansAndSegments(
                    flipped, origin, opts, chainage, geometry.spans, geometry.segments, chainage);
            }
        } else {
            for (const scene::TrackBezier& bez : list) {
                detail::appendBezierSpansAndSegments(
                    bez, origin, opts, chainage, geometry.spans, geometry.segments, chainage);
            }
        }
    }

    geometry.totalLengthM = chainage;

    for (std::size_t i = 0; i < geometry.segments.size(); ++i) {
        const TrackLineSegment& seg = geometry.segments[i];
        geo::SpatialGrid<25>::ItemBounds bounds{
            std::min(seg.p1.x, seg.p2.x),
            std::max(seg.p1.x, seg.p2.x),
            std::min(seg.p1.z, seg.p2.z),
            std::max(seg.p1.z, seg.p2.z),
        };
        geometry.grid.insert(i, bounds);
    }

    return geometry;
}

[[nodiscard]] inline RouteGeometry subsampleRouteGeometryForBand(
    const RouteGeometry& source,
    const std::size_t everyNthSegment = 4) {
    RouteGeometry out;
    out.label = source.label;
    out.totalLengthM = source.totalLengthM;
    if (source.segments.empty() || everyNthSegment == 0) {
        return out;
    }

    out.segments.reserve(source.segments.size() / everyNthSegment + 1);
    for (std::size_t i = 0; i < source.segments.size(); i += everyNthSegment) {
        out.segments.push_back(source.segments[i]);
    }

    for (std::size_t i = 0; i < out.segments.size(); ++i) {
        const TrackLineSegment& seg = out.segments[i];
        geo::SpatialGrid<25>::ItemBounds bounds{
            std::min(seg.p1.x, seg.p2.x),
            std::max(seg.p1.x, seg.p2.x),
            std::min(seg.p1.z, seg.p2.z),
            std::max(seg.p1.z, seg.p2.z),
        };
        out.grid.insert(i, bounds);
    }

    return out;
}

[[nodiscard]] inline std::vector<RouteChainageSample> sampleRouteCenterlineFromGeometry(
    const RouteGeometry& geometry,
    const geo::PuwgMasterOffset& master,
    const double stepM) {
    std::vector<RouteChainageSample> samples;
    if (geometry.spans.empty() || geometry.totalLengthM <= 0.0 || stepM <= 0.0) {
        return samples;
    }

    for (double chainage = 0.0; chainage <= geometry.totalLengthM + 1e-6; chainage += stepM) {
        const BezierSpan* span = nullptr;
        for (const BezierSpan& candidate : geometry.spans) {
            if (chainage >= candidate.chainageStartM &&
                chainage <= candidate.chainageStartM + candidate.lengthM + 1e-6) {
                span = &candidate;
                break;
            }
        }
        if (span == nullptr) {
            span = &geometry.spans.back();
        }

        const double local = span->lengthM > 0.0
            ? std::clamp((chainage - span->chainageStartM) / span->lengthM, 0.0, 1.0)
            : 0.0;
        const scene::Vec3 sim = detail::pointOnBezierSpan(*span, local);

        RouteChainageSample sample;
        sample.chainageM = chainage;
        sample.sim = sim;
        sample.trackHeight = sim.y;
        sample.geo = geo::simToGeo(sim, master);
        samples.push_back(sample);
    }

    return samples;
}

[[nodiscard]] inline std::vector<RouteChainageSample> sampleRouteCenterlineEvery(
    const scene::SceneDocument& doc,
    const scene::TrackRouteBuildResult& routes,
    const std::size_t routeIndex,
    const geo::PuwgMasterOffset& master,
    const double stepM) {
    std::vector<RouteChainageSample> samples;
    if (routeIndex >= routes.routes.size() || stepM <= 0.0) {
        return samples;
    }

    const scene::TrackRoute& route = routes.routes[routeIndex];
    std::vector<BezierSpan> spans;
    spans.reserve(route.links.size() * 4);

    double totalLength = 0.0;
    for (const scene::TrackRouteLink& link : route.links) {
        const scene::TrackSegmentEndpoints& segment = routes.segments[link.segmentIndex];
        const std::vector<scene::TrackBezier>* beziers = detail::beziersForSegment(doc, segment);
        if (beziers == nullptr || beziers->empty()) {
            continue;
        }

        const scene::Vec3 origin = detail::headerOffsetForSegment(doc, segment);
        const std::vector<scene::TrackBezier>& list = *beziers;
        if (link.reversed) {
            for (auto it = list.rbegin(); it != list.rend(); ++it) {
                scene::TrackBezier flipped = *it;
                std::swap(flipped.p1, flipped.p2);
                std::swap(flipped.cv1, flipped.cv2);
                BezierSpan span;
                span.p1 = scene::detail::track_routes::addOffset(flipped.p1, origin);
                span.p2 = scene::detail::track_routes::addOffset(flipped.p2, origin);
                span.cv1 = flipped.cv1;
                span.cv2 = flipped.cv2;
                span.lengthM =
                    detail::approximateBezierLength(span.p1, span.cv1, span.cv2, span.p2);
                span.chainageStartM = totalLength;
                spans.push_back(span);
                totalLength += span.lengthM;
            }
        } else {
            for (const scene::TrackBezier& bez : list) {
                BezierSpan span;
                span.p1 = scene::detail::track_routes::addOffset(bez.p1, origin);
                span.p2 = scene::detail::track_routes::addOffset(bez.p2, origin);
                span.cv1 = bez.cv1;
                span.cv2 = bez.cv2;
                span.lengthM = detail::approximateBezierLength(span.p1, span.cv1, span.cv2, span.p2);
                span.chainageStartM = totalLength;
                spans.push_back(span);
                totalLength += span.lengthM;
            }
        }
    }

    if (spans.empty() || totalLength <= 0.0) {
        return samples;
    }

    for (double chainage = 0.0; chainage <= totalLength + 1e-6; chainage += stepM) {
        const BezierSpan* span = nullptr;
        for (const BezierSpan& candidate : spans) {
            if (chainage >= candidate.chainageStartM &&
                chainage <= candidate.chainageStartM + candidate.lengthM + 1e-6) {
                span = &candidate;
                break;
            }
        }
        if (span == nullptr) {
            span = &spans.back();
        }

        const double local = span->lengthM > 0.0
            ? std::clamp((chainage - span->chainageStartM) / span->lengthM, 0.0, 1.0)
            : 0.0;
        const scene::Vec3 sim = detail::pointOnBezierSpan(*span, local);

        RouteChainageSample sample;
        sample.chainageM = chainage;
        sample.sim = sim;
        sample.trackHeight = sim.y;
        sample.geo = geo::simToGeo(sim, master);
        samples.push_back(sample);
    }

    return samples;
}

[[nodiscard]] inline double distancePointToSegmentXZ(
    const scene::Vec3& point,
    const TrackLineSegment& seg,
    double& outTrackY) noexcept {
    const double dx = seg.p2.x - seg.p1.x;
    const double dz = seg.p2.z - seg.p1.z;
    const double l2 = dx * dx + dz * dz;
    if (l2 <= 0.0) {
        outTrackY = seg.p1.y;
        const double px = point.x - seg.p1.x;
        const double pz = point.z - seg.p1.z;
        return std::sqrt(px * px + pz * pz);
    }

    double t = ((point.x - seg.p1.x) * dx + (point.z - seg.p1.z) * dz) / l2;
    t = std::max(0.0, std::min(1.0, t));

    const double projX = seg.p1.x + t * dx;
    const double projZ = seg.p1.z + t * dz;
    outTrackY = seg.p1.y + t * (seg.p2.y - seg.p1.y);

    const double qx = point.x - projX;
    const double qz = point.z - projZ;
    return std::sqrt(qx * qx + qz * qz);
}

[[nodiscard]] inline double minDistanceToRouteXZ(
    const scene::Vec3& point,
    const RouteGeometry& geometry,
    const double searchRadius,
    std::vector<std::size_t>& scratch) {
    geometry.grid.query(point, searchRadius, scratch);

    double best = searchRadius + 1.0;
    double trackY = 0.0;
    for (const std::size_t idx : scratch) {
        const double d = distancePointToSegmentXZ(point, geometry.segments[idx], trackY);
        if (d < best) {
            best = d;
        }
    }
    return best;
}

inline void corridorBboxPUWG(
    const RouteGeometry& geometry,
    const geo::PuwgMasterOffset& master,
    const double marginM,
    double& northMin,
    double& northMax,
    double& eastMin,
    double& eastMax) {
    if (geometry.segments.empty()) {
        northMin = northMax = eastMin = eastMax = 0.0;
        return;
    }

    northMin = 1e18;
    northMax = -1e18;
    eastMin = 1e18;
    eastMax = -1e18;

    for (const TrackLineSegment& seg : geometry.segments) {
        for (const scene::Vec3& sim : {seg.p1, seg.p2}) {
            const geo::PuwgPoint geo = geo::simToGeo(sim, master);
            northMin = std::min(northMin, geo.north);
            northMax = std::max(northMax, geo.north);
            eastMin = std::min(eastMin, geo.east);
            eastMax = std::max(eastMax, geo.east);
        }
    }

    northMin -= marginM;
    northMax += marginM;
    eastMin -= marginM;
    eastMax += marginM;
}

} // namespace eu07::nmt
