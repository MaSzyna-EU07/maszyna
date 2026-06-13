#pragma once

// Trasy z odcinków track: łączenie wyłącznie po geometrii końców, trasy jak najdłuższe.

#include <eu07/scene/document.hpp>
#include <eu07/scene/node/track.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <optional>
#include <ostream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace eu07::scene {

enum class TrackSegmentKind { Normal, Switch, Road, Cross, Other };

struct TrackSegmentRef {
    TrackSegmentKind kind = TrackSegmentKind::Normal;
    std::size_t index = 0;
};

struct TrackSegmentEndpoints {
    TrackSegmentRef ref;
    std::string name;
    std::size_t sourceLine = 0;
    Vec3 start{};
    Vec3 end{};
    double length = 0.0;
    bool hasGeometry = false;
};

struct TrackConnection {
    std::size_t segmentA = 0;
    std::size_t segmentB = 0;
    bool aAtStart = false;
    bool bAtStart = false;
    double distance = 0.0;
};

struct TrackRouteLink {
    std::size_t segmentIndex = 0;
    bool reversed = false;
};

struct TrackRoute {
    std::string label;
    std::vector<TrackRouteLink> links;
    bool closed = false;
    double lengthSum = 0.0;
};

struct TrackRouteBuildOptions {
    double endpointTolerance = 0.15;
    double gridCellSize = 25.0;
    bool includeRoads = false;
    bool includeCross = false;
    bool includeOther = false;
};

struct TrackRouteBuildResult {
    std::vector<TrackSegmentEndpoints> segments;
    std::vector<TrackConnection> connections;
    std::vector<TrackRoute> routes;
    std::size_t skippedWithoutGeometry = 0;
};

namespace detail::track_routes {

inline constexpr std::size_t kInvalidSlot = static_cast<std::size_t>(-1);

inline Vec3 addOffset(const Vec3 point, const Vec3 offset) noexcept {
    return {point.x + offset.x, point.y + offset.y, point.z + offset.z};
}

inline double distance(const Vec3& a, const Vec3& b) noexcept {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

inline std::optional<std::pair<Vec3, Vec3>> segmentWorldEnds(
    const std::vector<TrackBezier>& beziers,
    const Vec3& originOffset) {
    if (beziers.empty()) {
        return std::nullopt;
    }
    return std::pair{
        addOffset(beziers.front().p1, originOffset),
        addOffset(beziers.back().p2, originOffset),
    };
}

template <typename TrackNode>
inline void appendSegment(
    std::vector<TrackSegmentEndpoints>& out,
    const TrackSegmentKind kind,
    const std::size_t index,
    const TrackNode& node) {
    TrackSegmentEndpoints entry;
    entry.ref = {kind, index};
    entry.name = node.header.name;
    entry.sourceLine = node.header.line;
    entry.length = node.length;

    if (const std::optional<std::pair<Vec3, Vec3>> ends =
            segmentWorldEnds(node.beziers, node.header.originOffset)) {
        entry.start = ends->first;
        entry.end = ends->second;
        entry.hasGeometry = true;
    }

    out.push_back(std::move(entry));
}

struct EndpointSlot {
    std::size_t segmentIndex = 0;
    bool isStart = false;
};

struct OrientedStep {
    std::size_t segmentIndex = 0;
    bool reversed = false;
};

struct GridKey {
    int x = 0;
    int z = 0;

    [[nodiscard]] bool operator==(const GridKey& other) const noexcept {
        return x == other.x && z == other.z;
    }
};

struct GridKeyHash {
    [[nodiscard]] std::size_t operator()(const GridKey& key) const noexcept {
        return static_cast<std::size_t>(key.x * 73856093) ^
               static_cast<std::size_t>(key.z * 19349663);
    }
};

struct UnionFind {
    std::vector<std::size_t> parent;

    explicit UnionFind(const std::size_t size) : parent(size) {
        for (std::size_t i = 0; i < size; ++i) {
            parent[i] = i;
        }
    }

    [[nodiscard]] std::size_t find(std::size_t x) {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        return x;
    }

    void unite(const std::size_t a, const std::size_t b) {
        const std::size_t ra = find(a);
        const std::size_t rb = find(b);
        if (ra != rb) {
            parent[rb] = ra;
        }
    }
};

struct EndpointIndex {
    std::vector<EndpointSlot> slots;
    std::vector<std::size_t> slotAtStart;
    std::vector<std::size_t> slotAtEnd;
    std::vector<std::size_t> junctionOfSlot;
    std::vector<std::vector<std::size_t>> junctionMembers;
};

inline GridKey gridKeyFor(const Vec3& point, const double cellSize) noexcept {
    return {
        static_cast<int>(std::floor(point.x / cellSize)),
        static_cast<int>(std::floor(point.z / cellSize)),
    };
}

inline void indexEndpoint(
    std::unordered_map<GridKey, std::vector<std::size_t>, GridKeyHash>& grid,
    const Vec3& point,
    const std::size_t endpointIndex,
    const double cellSize) {
    const GridKey center = gridKeyFor(point, cellSize);
    for (int dz = -1; dz <= 1; ++dz) {
        for (int dx = -1; dx <= 1; ++dx) {
            grid[{center.x + dx, center.z + dz}].push_back(endpointIndex);
        }
    }
}

inline Vec3 endpointPoint(const TrackSegmentEndpoints& segment, const bool isStart) noexcept {
    return isStart ? segment.start : segment.end;
}

inline EndpointIndex buildEndpointIndex(
    const std::vector<TrackSegmentEndpoints>& segments,
    const double tolerance,
    const double cellSize) {
    EndpointIndex index;
    index.slotAtStart.resize(segments.size(), kInvalidSlot);
    index.slotAtEnd.resize(segments.size(), kInvalidSlot);

    for (std::size_t segmentIndex = 0; segmentIndex < segments.size(); ++segmentIndex) {
        if (!segments[segmentIndex].hasGeometry) {
            continue;
        }

        index.slotAtStart[segmentIndex] = index.slots.size();
        index.slots.push_back({segmentIndex, true});

        index.slotAtEnd[segmentIndex] = index.slots.size();
        index.slots.push_back({segmentIndex, false});
    }

    UnionFind uf(index.slots.size());
    std::unordered_map<GridKey, std::vector<std::size_t>, GridKeyHash> grid;

    for (std::size_t slotIdx = 0; slotIdx < index.slots.size(); ++slotIdx) {
        const EndpointSlot& slot = index.slots[slotIdx];
        indexEndpoint(
            grid,
            endpointPoint(segments[slot.segmentIndex], slot.isStart),
            slotIdx,
            cellSize);
    }

    for (std::size_t left = 0; left < index.slots.size(); ++left) {
        const Vec3 leftPoint =
            endpointPoint(segments[index.slots[left].segmentIndex], index.slots[left].isStart);
        const GridKey center = gridKeyFor(leftPoint, cellSize);

        for (int dz = -1; dz <= 1; ++dz) {
            for (int dx = -1; dx <= 1; ++dx) {
                const GridKey key{center.x + dx, center.z + dz};
                const auto bucket = grid.find(key);
                if (bucket == grid.end()) {
                    continue;
                }

                for (const std::size_t right : bucket->second) {
                    if (right <= left) {
                        continue;
                    }
                    const Vec3 rightPoint = endpointPoint(
                        segments[index.slots[right].segmentIndex],
                        index.slots[right].isStart);
                    if (distance(leftPoint, rightPoint) <= tolerance) {
                        uf.unite(left, right);
                    }
                }
            }
        }
    }

    index.junctionOfSlot.resize(index.slots.size());
    index.junctionMembers.resize(index.slots.size());
    for (std::size_t slotIdx = 0; slotIdx < index.slots.size(); ++slotIdx) {
        const std::size_t root = uf.find(slotIdx);
        index.junctionOfSlot[slotIdx] = root;
        index.junctionMembers[root].push_back(slotIdx);
    }

    return index;
}

inline std::size_t slotForExit(
    const EndpointIndex& index,
    const std::size_t segmentIndex,
    const bool exitAtStart) {
    return exitAtStart ? index.slotAtStart[segmentIndex] : index.slotAtEnd[segmentIndex];
}

inline std::vector<OrientedStep> neighborsAtExit(
    const EndpointIndex& endpointIndex,
    const std::size_t segmentIndex,
    const bool exitAtStart) {
    std::vector<OrientedStep> steps;

    const std::size_t slot = slotForExit(endpointIndex, segmentIndex, exitAtStart);
    if (slot == kInvalidSlot) {
        return steps;
    }

    const std::size_t junction = endpointIndex.junctionOfSlot[slot];
    for (const std::size_t memberSlot : endpointIndex.junctionMembers[junction]) {
        const EndpointSlot& member = endpointIndex.slots[memberSlot];
        if (member.segmentIndex == segmentIndex) {
            continue;
        }
        steps.push_back({member.segmentIndex, !member.isStart});
    }

    return steps;
}

inline std::size_t greedyChainLength(
    const std::size_t segmentIndex,
    const bool reversed,
    const EndpointIndex& endpointIndex,
    std::vector<bool>& used) {
    if (used[segmentIndex]) {
        return 0;
    }

    used[segmentIndex] = true;
    std::size_t best = 1;

    const bool exitAtStart = reversed;
    for (const OrientedStep& step : neighborsAtExit(endpointIndex, segmentIndex, exitAtStart)) {
        if (used[step.segmentIndex]) {
            continue;
        }
        best = std::max(
            best,
            1 + greedyChainLength(step.segmentIndex, step.reversed, endpointIndex, used));
    }

    used[segmentIndex] = false;
    return best;
}

inline void extendFromExit(
    std::vector<TrackRouteLink>& links,
    std::size_t segmentIndex,
    bool exitAtStart,
    const EndpointIndex& endpointIndex,
    std::vector<bool>& used,
    const bool prepend) {
    while (true) {
        std::size_t bestLength = 0;
        std::optional<OrientedStep> bestStep;

        for (const OrientedStep& step : neighborsAtExit(endpointIndex, segmentIndex, exitAtStart)) {
            if (used[step.segmentIndex]) {
                continue;
            }
            const std::size_t tail =
                greedyChainLength(step.segmentIndex, step.reversed, endpointIndex, used);
            if (tail > bestLength) {
                bestLength = tail;
                bestStep = step;
            }
        }

        if (!bestStep) {
            break;
        }

        used[bestStep->segmentIndex] = true;
        if (prepend) {
            links.insert(links.begin(), {bestStep->segmentIndex, bestStep->reversed});
        } else {
            links.push_back({bestStep->segmentIndex, bestStep->reversed});
        }

        segmentIndex = bestStep->segmentIndex;
        exitAtStart = bestStep->reversed;
    }
}

inline double routeLengthSum(
    const TrackRoute& route,
    const std::vector<TrackSegmentEndpoints>& segments) {
    double sum = 0.0;
    for (const TrackRouteLink& link : route.links) {
        sum += segments[link.segmentIndex].length;
    }
    return sum;
}

inline bool routeIsClosed(
    const TrackRoute& route,
    const std::vector<TrackSegmentEndpoints>& segments,
    const double tolerance) {
    if (route.links.size() < 2) {
        return false;
    }

    const TrackSegmentEndpoints& first = segments[route.links.front().segmentIndex];
    const TrackSegmentEndpoints& last = segments[route.links.back().segmentIndex];
    const Vec3 routeStart = route.links.front().reversed ? first.end : first.start;
    const Vec3 routeEnd = route.links.back().reversed ? last.start : last.end;
    return distance(routeStart, routeEnd) <= tolerance;
}

inline TrackRoute buildRouteFromSeed(
    const std::size_t seedIndex,
    const bool seedReversed,
    const EndpointIndex& endpointIndex,
    std::vector<bool>& used) {
    TrackRoute route;

    used[seedIndex] = true;
    route.links.push_back({seedIndex, seedReversed});

    extendFromExit(route.links, seedIndex, seedReversed, endpointIndex, used, false);
    extendFromExit(route.links, seedIndex, !seedReversed, endpointIndex, used, true);

    return route;
}

inline void buildLongestRoutes(
    const std::vector<TrackSegmentEndpoints>& segments,
    const EndpointIndex& endpointIndex,
    const double tolerance,
    std::vector<TrackRoute>& routesOut) {
    std::vector<bool> used(segments.size(), false);

    const auto hasUnused = [&]() {
        return std::ranges::any_of(used, [](const bool flag) { return !flag; });
    };

    while (hasUnused()) {
        TrackRoute bestRoute;

        for (std::size_t seedIndex = 0; seedIndex < segments.size(); ++seedIndex) {
            if (!segments[seedIndex].hasGeometry || used[seedIndex]) {
                continue;
            }

            for (const bool seedReversed : {false, true}) {
                std::vector<bool> trialUsed = used;
                TrackRoute trial =
                    buildRouteFromSeed(seedIndex, seedReversed, endpointIndex, trialUsed);

                if (trial.links.size() > bestRoute.links.size()) {
                    bestRoute = std::move(trial);
                } else if (
                    trial.links.size() == bestRoute.links.size() &&
                    routeLengthSum(trial, segments) > routeLengthSum(bestRoute, segments)) {
                    bestRoute = std::move(trial);
                }
            }
        }

        if (bestRoute.links.empty()) {
            for (std::size_t i = 0; i < segments.size(); ++i) {
                if (segments[i].hasGeometry && !used[i]) {
                    bestRoute.links.push_back({i, false});
                    used[i] = true;
                    break;
                }
            }
        } else {
            for (const TrackRouteLink& link : bestRoute.links) {
                used[link.segmentIndex] = true;
            }
        }

        if (!bestRoute.links.empty()) {
            bestRoute.label = segments[bestRoute.links.front().segmentIndex].name;
            bestRoute.lengthSum = routeLengthSum(bestRoute, segments);
            bestRoute.closed = routeIsClosed(bestRoute, segments, tolerance);
            routesOut.push_back(std::move(bestRoute));
        }
    }
}

inline void recordConnections(
    const EndpointIndex& endpointIndex,
    const std::vector<TrackSegmentEndpoints>& segments,
    std::vector<TrackConnection>& connectionsOut) {
    for (std::size_t left = 0; left < endpointIndex.slots.size(); ++left) {
        const std::size_t junctionLeft = endpointIndex.junctionOfSlot[left];
        for (std::size_t right = left + 1; right < endpointIndex.slots.size(); ++right) {
            if (endpointIndex.junctionOfSlot[right] != junctionLeft) {
                continue;
            }

            const EndpointSlot& a = endpointIndex.slots[left];
            const EndpointSlot& b = endpointIndex.slots[right];
            if (a.segmentIndex == b.segmentIndex) {
                continue;
            }

            const double gap = distance(
                endpointPoint(segments[a.segmentIndex], a.isStart),
                endpointPoint(segments[b.segmentIndex], b.isStart));

            connectionsOut.push_back(
                {a.segmentIndex, b.segmentIndex, a.isStart, b.isStart, gap});
        }
    }
}

} // namespace detail::track_routes

[[nodiscard]] inline TrackRouteBuildResult buildTrackRoutes(
    const SceneDocument& doc,
    const TrackRouteBuildOptions& options = {}) {
    TrackRouteBuildResult result;
    result.segments.reserve(countTrackInstances(doc));

    for (std::size_t i = 0; i < doc.nodeTrackNormal.size(); ++i) {
        detail::track_routes::appendSegment(
            result.segments, TrackSegmentKind::Normal, i, doc.nodeTrackNormal[i]);
    }
    for (std::size_t i = 0; i < doc.nodeTrackSwitch.size(); ++i) {
        detail::track_routes::appendSegment(
            result.segments, TrackSegmentKind::Switch, i, doc.nodeTrackSwitch[i]);
    }
    if (options.includeRoads) {
        for (std::size_t i = 0; i < doc.nodeTrackRoad.size(); ++i) {
            detail::track_routes::appendSegment(
                result.segments, TrackSegmentKind::Road, i, doc.nodeTrackRoad[i]);
        }
    }
    if (options.includeCross) {
        for (std::size_t i = 0; i < doc.nodeTrackCross.size(); ++i) {
            detail::track_routes::appendSegment(
                result.segments, TrackSegmentKind::Cross, i, doc.nodeTrackCross[i]);
        }
    }
    if (options.includeOther) {
        for (std::size_t i = 0; i < doc.nodeTrackOther.size(); ++i) {
            detail::track_routes::appendSegment(
                result.segments, TrackSegmentKind::Other, i, doc.nodeTrackOther[i]);
        }
    }

    for (const TrackSegmentEndpoints& segment : result.segments) {
        if (!segment.hasGeometry) {
            ++result.skippedWithoutGeometry;
        }
    }

    const detail::track_routes::EndpointIndex endpointIndex =
        detail::track_routes::buildEndpointIndex(
            result.segments, options.endpointTolerance, options.gridCellSize);

    detail::track_routes::recordConnections(endpointIndex, result.segments, result.connections);

    detail::track_routes::buildLongestRoutes(
        result.segments, endpointIndex, options.endpointTolerance, result.routes);

    std::ranges::sort(result.routes, [](const TrackRoute& a, const TrackRoute& b) {
        if (a.links.size() != b.links.size()) {
            return a.links.size() > b.links.size();
        }
        return a.lengthSum > b.lengthSum;
    });

    return result;
}

inline void writeTrackRoutesReport(std::ostream& out, const TrackRouteBuildResult& built) {
    out << "# trasy: laczenie wylacznie po geometrii, maksymalna dlugosc lancucha\n";
    out << "# segmenty=" << built.segments.size() << " polaczenia=" << built.connections.size()
        << " trasy=" << built.routes.size() << " bez_geometrii=" << built.skippedWithoutGeometry
        << '\n';

    for (std::size_t routeIndex = 0; routeIndex < built.routes.size(); ++routeIndex) {
        const TrackRoute& route = built.routes[routeIndex];
        out << "\n[trasa " << (routeIndex + 1) << "] label=" << route.label
            << " odcinki=" << route.links.size() << " len_sum=" << route.lengthSum
            << " closed=" << (route.closed ? "yes" : "no") << '\n';

        for (const TrackRouteLink& link : route.links) {
            const TrackSegmentEndpoints& segment = built.segments[link.segmentIndex];
            const char* kind = "other";
            switch (segment.ref.kind) {
            case TrackSegmentKind::Normal:
                kind = "normal";
                break;
            case TrackSegmentKind::Switch:
                kind = "switch";
                break;
            case TrackSegmentKind::Road:
                kind = "road";
                break;
            case TrackSegmentKind::Cross:
                kind = "cross";
                break;
            case TrackSegmentKind::Other:
                kind = "other";
                break;
            }

            out << "  " << kind << '[' << segment.ref.index << "] flat=" << link.segmentIndex
                << " name=" << segment.name << " L" << (segment.sourceLine + 1)
                << " rev=" << (link.reversed ? "yes" : "no") << " len=" << segment.length << '\n';
        }
    }
}

inline void writeTrackRoutesReport(
    const std::filesystem::path& outPath,
    const TrackRouteBuildResult& built) {
    std::ofstream out(outPath);
    if (!out) {
        throw std::runtime_error("Nie mozna zapisac: " + outPath.string());
    }
    writeTrackRoutesReport(out, built);
}

} // namespace eu07::scene
