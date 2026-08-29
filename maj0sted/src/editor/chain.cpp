#include "maj0sted/editor/chain.hpp"

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace maj0sted::editor {
namespace {

constexpr double kEps = 1e-9;

using domain::geometry::Pose;

Diagnostic make(Code code, TrackId track, ElementId element, std::string text) {
    Diagnostic diagnostic;
    diagnostic.code = code;
    diagnostic.track = track;
    diagnostic.element = element;
    diagnostic.text = std::move(text);
    return diagnostic;
}

Diagnostic degenerate(TrackId track, ElementId element, std::string text) {
    return make(Code::DegenerateElement, track, element, std::move(text));
}

/// Signed curvature an element ends on. A straight, and a clothoid running out
/// to an infinite radius, both end flat.
double end_curvature(const Element& element) noexcept {
    if (element.kind == Kind::Line || element.radius <= kEps) {
        return 0.0;
    }
    return static_cast<double>(element.hand) / element.radius;
}

}  // namespace

// ---------------------------------------------------------------------------

TrackOutcome lay_track(const Document& /*document*/, const Track& track, const Pose& start) {
    TrackOutcome out;
    out.track.id = track.id;
    out.track.start = start;
    out.track.end = start;

    // Two straights end to end are one straight: there is no corner between them,
    // so there is nothing to grab hold of and nothing an arc could re-fit against.
    // The chain is still laid — this is a rule about how track is written down,
    // not about whether the geometry closes.
    for (std::size_t i = 1; i < track.elements.size(); ++i) {
        if (track.elements[i].kind == Kind::Line &&
            track.elements[i - 1].kind == Kind::Line) {
            out.diagnostics.push_back(make(
                Code::TouchingStraights, track.id, track.elements[i].id,
                "dwie proste stykają się ze sobą — to jedna prosta o długości " +
                    std::to_string(static_cast<long long>(track.elements[i - 1].length +
                                                          track.elements[i].length)) +
                    " m; złącz je albo wstaw między nie łuk"));
        }
    }

    Pose walker = start;
    // curvature the chain is running at where the previous element ended, which
    // is what a clothoid eases away from. It carries across the whole chain: two
    // clothoids back to back are as ordinary as two arcs.
    double k_prev = 0.0;

    out.track.elements.reserve(track.elements.size());
    for (const Element& element : track.elements) {
        double k0 = 0.0;
        double k1 = 0.0;

        if (element.kind == Kind::Arc) {
            if (element.radius <= kEps) {
                out.diagnostics.push_back(
                    degenerate(track.id, element.id, "łuk musi mieć dodatni promień"));
                out.track.complete = false;
                break;
            }
            if (element.hand == 0) {
                out.diagnostics.push_back(degenerate(
                    track.id, element.id, "łuk musi skręcać w lewo albo w prawo"));
                out.track.complete = false;
                break;
            }
            k0 = k1 = static_cast<double>(element.hand) / element.radius;
        } else if (element.kind == Kind::Clothoid) {
            k0 = k_prev;
            k1 = end_curvature(element);
            if (std::abs(k1 - k0) < kEps) {
                out.diagnostics.push_back(degenerate(
                    track.id, element.id,
                    "krzywa przejściowa musi zmieniać krzywiznę — na jej końcu "
                    "jest ten sam promień, co na początku"));
                out.track.complete = false;
                break;
            }
        }

        if (element.length <= kEps) {
            out.diagnostics.push_back(
                degenerate(track.id, element.id, "długość musi być dodatnia"));
            out.track.complete = false;
            break;
        }

        SolvedElement solved;
        solved.id = element.id;
        solved.kind = element.kind;
        solved.start = walker;
        solved.k0 = k0;
        solved.k1 = k1;
        solved.length = element.length;

        std::vector<domain::geometry::XY> points;
        walker = domain::geometry::layout_segment(k0, k1, element.length, walker, &points);
        solved.end = walker;
        solved.points.reserve(points.size());
        for (const auto& point : points) {
            solved.points.push_back(PlanPoint{point.x, point.y});
        }

        out.track.length += element.length;
        out.track.elements.push_back(std::move(solved));
        k_prev = k1;
    }

    out.track.end = walker;

    for (const auto& element : out.track.elements) {
        for (std::size_t i = out.track.centreline.empty() ? 0 : 1; i < element.points.size();
             ++i) {
            out.track.centreline.push_back(element.points[i]);
        }
    }

    return out;
}

}  // namespace maj0sted::editor
