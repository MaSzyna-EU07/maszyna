#pragma once

#include <vector>

namespace maj0sted::render {

/// A point in the project's CRS (metres for EPSG:2180).
struct Point {
    double x;
    double y;
};

/// Which kind of plan element a polyline came from (lets a GUI colour it).
enum class ElementKind { Straight, Arc, Transition };

/// A polyline approximating one plan element, in CRS coordinates.
struct Polyline {
    ElementKind kind;
    std::vector<Point> points;
};

/// A rendering-agnostic view of one or more plans: polylines plus a bounding box.
/// Any GUI can draw this — the library computes the geometry, the GUI only paints.
struct Scene {
    std::vector<Polyline> polylines;
    double min_x{0.0};
    double min_y{0.0};
    double max_x{0.0};
    double max_y{0.0};
    bool empty{true};
};

}  // namespace maj0sted::render
