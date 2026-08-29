#include "maj0sted/editor/solution.hpp"

#include <algorithm>
#include <cmath>

namespace maj0sted::editor {

const char* code_name(Code code) {
    switch (code) {
        case Code::UnknownReference: return "UnknownReference";
        case Code::CyclicDependency: return "CyclicDependency";
        case Code::DegenerateElement: return "DegenerateElement";
        case Code::TurnoutInvalid: return "TurnoutInvalid";
        case Code::TurnoutNotClosed: return "TurnoutNotClosed";
        case Code::StationOffTrack: return "StationOffTrack";
        case Code::TouchingStraights: return "TouchingStraights";
        case Code::HoldConflict: return "HoldConflict";
        case Code::OffsetNotHeld: return "OffsetNotHeld";
    }
    return "Unknown";
}

const SolvedTrack* find_track(const Solution& solution, TrackId id) {
    if (id == TrackId::none) {
        return nullptr;
    }
    for (const auto& track : solution.tracks) {
        if (track.id == id) {
            return &track;
        }
    }
    return nullptr;
}

const SolvedTurnout* find_turnout(const Solution& solution, TurnoutId id) {
    if (id == TurnoutId::none) {
        return nullptr;
    }
    for (const auto& turnout : solution.turnouts) {
        if (turnout.id == id) {
            return &turnout;
        }
    }
    return nullptr;
}

const SolvedElement* find_element(const Solution& solution, ElementId id) {
    if (id == ElementId::none) {
        return nullptr;
    }
    for (const auto& track : solution.tracks) {
        for (const auto& element : track.elements) {
            if (element.id == id) {
                return &element;
            }
        }
    }
    return nullptr;
}

bool pose_at(const SolvedTrack& track, double station, domain::geometry::Pose& out) {
    const auto& line = track.centreline;
    if (line.empty()) {
        return false;
    }
    if (line.size() == 1) {
        out = domain::geometry::Pose{line[0].x, line[0].y, track.start.hx, track.start.hy};
        return true;
    }

    double walked = 0.0;
    for (std::size_t i = 1; i < line.size(); ++i) {
        const double dx = line[i].x - line[i - 1].x;
        const double dy = line[i].y - line[i - 1].y;
        const double step = std::hypot(dx, dy);
        if (step <= 0.0) {
            continue;
        }
        if (station <= walked + step) {
            const double t = std::clamp((station - walked) / step, 0.0, 1.0);
            out = domain::geometry::Pose{line[i - 1].x + dx * t, line[i - 1].y + dy * t,
                                         dx / step, dy / step};
            return true;
        }
        walked += step;
    }

    out = track.end;
    return true;
}

bool steady_room(const SolvedTrack& track, double station, double& curvature, double& behind,
                 double& ahead) {
    constexpr double kEps = 1e-6;
    if (station < -kEps) {
        return false;
    }
    double walked = 0.0;
    for (std::size_t i = 0; i < track.elements.size(); ++i) {
        const SolvedElement& element = track.elements[i];
        const double ends = walked + element.length;
        // a station sitting exactly on a joint belongs to the element ahead of it
        const bool last = i + 1 == track.elements.size();
        if (station < ends - kEps || (last && station <= ends + kEps)) {
            if (std::abs(element.k1 - element.k0) > 1e-12) {
                return false;  // a clothoid: no one radius to bend a turnout to
            }
            curvature = element.k0;
            behind = station - walked;
            ahead = ends - station;
            return true;
        }
        walked = ends;
    }
    return false;
}

}  // namespace maj0sted::editor
