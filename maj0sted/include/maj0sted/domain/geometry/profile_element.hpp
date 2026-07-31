#pragma once

#include <variant>

#include "maj0sted/domain/value_objects/length.hpp"
#include "maj0sted/domain/value_objects/radius.hpp"

namespace maj0sted::domain {

/// Sense of a vertical curve: a crest ("wypukły" — hilltop) or a sag
/// ("wklęsły" — valley).
enum class VerticalCurvature { Crest, Sag };

/// Krzywa w profilu: pochylenie o stałym nachyleniu, wyrażonym w promilach (‰).
/// Wartość dodatnia oznacza wznoszenie, ujemna — opadanie.
class Grade {
public:
    Grade(Length length, double slope_permille) noexcept
        : length_{length}, slope_permille_{slope_permille} {}

    [[nodiscard]] Length length() const noexcept { return length_; }
    [[nodiscard]] double slope_permille() const noexcept { return slope_permille_; }

private:
    Length length_;
    double slope_permille_;
};

/// Krzywa w profilu: łuk pionowy o stałym promieniu, łączący dwa pochylenia.
class VerticalCurve {
public:
    VerticalCurve(Length length, Radius radius, VerticalCurvature curvature) noexcept
        : length_{length}, radius_{radius}, curvature_{curvature} {}

    [[nodiscard]] Length length() const noexcept { return length_; }
    [[nodiscard]] Radius radius() const noexcept { return radius_; }
    [[nodiscard]] VerticalCurvature curvature() const noexcept { return curvature_; }

private:
    Length length_;
    Radius radius_;
    VerticalCurvature curvature_;
};

/// Any single element of the vertical alignment.
using ProfileElement = std::variant<Grade, VerticalCurve>;

/// Length of a profile element, whatever its concrete kind.
[[nodiscard]] inline Length length_of(const ProfileElement& element) {
    return std::visit([](const auto& e) { return e.length(); }, element);
}

}  // namespace maj0sted::domain
