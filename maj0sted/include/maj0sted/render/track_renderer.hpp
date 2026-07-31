#pragma once

#include <vector>

#include "maj0sted/render/scene.hpp"  // Point, ElementKind

namespace maj0sted::render {

/// One sampled centreline element of a solved niweleta, tagged with its logical
/// origin so a UI can colour, label and hit-test it. Rendering-neutral: it holds
/// the track axis only — no rails, no styling. This is the contract the solver
/// hands to a renderer.
struct CentrelineElement {
    ElementKind kind{ElementKind::Straight};
    std::vector<Point> points;  ///< sampled axis in CRS metres
    int straight_index{-1};     ///< owning straight, or -1 for a curve element
    int gap{-1};                ///< owning gap, or -1 for a straight
    int element_index{-1};      ///< position within a gap's curve chain
    double length{0.0};         ///< element length (metres)
    double radius_start{0.0};   ///< 0 == infinite (straight / clothoid open end)
    double radius_end{0.0};
};

/// Interface every renderer must satisfy.
///
/// The solver emits rendering-neutral CentrelineElements; a renderer turns each
/// into concrete drawable geometry (a single axis line, two rails, an SVG path,
/// ...). A renderer holds NO business logic — it only shapes presentation, so
/// swapping renderers never changes what geometry the solver computed.
class TrackRenderer {
public:
    virtual ~TrackRenderer() = default;

    /// Called once per centreline element, in plan order.
    virtual void add(const CentrelineElement& element) = 0;
};

}  // namespace maj0sted::render
