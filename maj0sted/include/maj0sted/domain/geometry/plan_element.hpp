#pragma once

#include <cmath>
#include <optional>
#include <variant>

#include "maj0sted/domain/value_objects/azimuth.hpp"
#include "maj0sted/domain/value_objects/cartesian_position.hpp"
#include "maj0sted/domain/value_objects/length.hpp"
#include "maj0sted/domain/value_objects/radius.hpp"
#include "maj0sted/domain/value_objects/straight_id.hpp"

namespace maj0sted::domain {

/// Direction of a horizontal turn: left ("w lewo") or right ("w prawo").
enum class TurnDirection { Left, Right };

/// Krzywa w planie: odcinek prosty.
///
/// A straight is *anchored* by both of its XY endpoints; its azimuth and length
/// are computed from them. During fitting a straight keeps its supporting line
/// (position + azimuth) fixed — only its length may change as an endpoint slides
/// along that line. The curves between straights are positioned as a result of
/// that fitting, not the other way round.
///
/// A straight also carries an optional identity (StraightId). References between
/// straight segments — e.g. parallelism — point at that id directly. Straights
/// used purely as geometry may stay anonymous (null id).
class Straight {
public:
    /// Anonymous straight (null id).
    /// @throws std::invalid_argument if @p start and @p end coincide (a
    ///         zero-length straight has no defined azimuth).
    Straight(CartesianPosition start, CartesianPosition end)
        : Straight{StraightId{}, start, end} {}

    /// Identified straight.
    /// @throws std::invalid_argument if @p start and @p end coincide.
    Straight(StraightId id, CartesianPosition start, CartesianPosition end)
        : id_{id},
          start_{start},
          end_{end},
          azimuth_{Azimuth::of_direction(start, end)},
          length_{Length::from_metres(distance(start, end))} {}

    [[nodiscard]] StraightId id() const noexcept { return id_; }
    [[nodiscard]] CartesianPosition start() const noexcept { return start_; }
    [[nodiscard]] CartesianPosition end() const noexcept { return end_; }
    [[nodiscard]] Azimuth azimuth() const noexcept { return azimuth_; }
    [[nodiscard]] Length length() const noexcept { return length_; }

private:
    static double distance(CartesianPosition a, CartesianPosition b) noexcept {
        const double dx = b.x() - a.x();
        const double dy = b.y() - a.y();
        return std::sqrt(dx * dx + dy * dy);
    }

    StraightId id_;
    CartesianPosition start_;
    CartesianPosition end_;
    Azimuth azimuth_;
    Length length_;
};

/// Krzywa w planie: łuk kołowy o stałym promieniu.
///
/// A floating element: it is defined by its radius and turn direction. Its
/// length is not essential and is therefore optional — typically it becomes
/// known only as a resultant of fitting between the anchored straights.
class CircularArc {
public:
    CircularArc(Radius radius, TurnDirection direction,
                std::optional<Length> length = std::nullopt) noexcept
        : radius_{radius}, direction_{direction}, length_{length} {}

    [[nodiscard]] Radius radius() const noexcept { return radius_; }
    [[nodiscard]] TurnDirection direction() const noexcept { return direction_; }
    [[nodiscard]] std::optional<Length> length() const noexcept { return length_; }

private:
    Radius radius_;
    TurnDirection direction_;
    std::optional<Length> length_;
};

/// Krzywa w planie: krzywa przejściowa (klotoida).
///
/// A floating element defined by its length, the radii at each end and its turn
/// direction. A missing radius (nullopt) denotes an infinite radius — a tangent
/// to a straight — so one type covers straight->arc, arc->arc and arc->straight.
class TransitionCurve {
public:
    TransitionCurve(Length length, std::optional<Radius> start_radius,
                    std::optional<Radius> end_radius, TurnDirection direction) noexcept
        : length_{length},
          start_radius_{start_radius},
          end_radius_{end_radius},
          direction_{direction} {}

    [[nodiscard]] Length length() const noexcept { return length_; }
    [[nodiscard]] std::optional<Radius> start_radius() const noexcept { return start_radius_; }
    [[nodiscard]] std::optional<Radius> end_radius() const noexcept { return end_radius_; }
    [[nodiscard]] TurnDirection direction() const noexcept { return direction_; }

private:
    Length length_;
    std::optional<Radius> start_radius_;
    std::optional<Radius> end_radius_;
    TurnDirection direction_;
};

/// Any single element of the horizontal alignment.
using PlanElement = std::variant<Straight, CircularArc, TransitionCurve>;

/// Length of a plan element, if known. A circular arc may not carry one.
[[nodiscard]] inline std::optional<Length> length_of(const PlanElement& element) {
    if (const auto* straight = std::get_if<Straight>(&element)) {
        return straight->length();
    }
    if (const auto* transition = std::get_if<TransitionCurve>(&element)) {
        return transition->length();
    }
    return std::get<CircularArc>(element).length();
}

}  // namespace maj0sted::domain
