#pragma once

namespace maj0sted::render {

/// A point in the project's CRS (metres for EPSG:2180).
struct Point {
    double x;
    double y;
};

/// Which kind of plan element a polyline came from (lets a GUI colour it).
enum class ElementKind { Straight, Arc, Transition };

}  // namespace maj0sted::render
