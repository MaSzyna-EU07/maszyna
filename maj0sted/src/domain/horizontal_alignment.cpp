#include "maj0sted/domain/geometry/horizontal_alignment.hpp"

#include <cmath>
#include <stdexcept>
#include <variant>

namespace maj0sted::domain {
namespace {

// Two anchored endpoints closer than this are treated as touching.
constexpr double kPositionToleranceM = 1e-6;
// Azimuths differing by less than this are treated as equal.
constexpr double kAzimuthToleranceRad = 1e-9;

double distance(CartesianPosition a, CartesianPosition b) noexcept {
    const double dx = b.x() - a.x();
    const double dy = b.y() - a.y();
    return std::sqrt(dx * dx + dy * dy);
}

bool same_arc(const CircularArc& a, const CircularArc& b) noexcept {
    return a.direction() == b.direction() && a.radius() == b.radius();
}

void validate_joint(const PlanElement& previous, const PlanElement& current,
                    JointContinuity joint) {
    // Rule: two circular arcs of the same radius and direction must not meet.
    if (const auto* a = std::get_if<CircularArc>(&previous)) {
        if (const auto* b = std::get_if<CircularArc>(&current)) {
            if (same_arc(*a, *b)) {
                throw std::logic_error{
                    "Adjacent circular arcs with the same radius and direction "
                    "are not allowed"};
            }
        }
    }

    const auto* previous_straight = std::get_if<Straight>(&previous);
    const auto* current_straight = std::get_if<Straight>(&current);

    if (joint == JointContinuity::AzimuthBreak) {
        if (previous_straight == nullptr || current_straight == nullptr) {
            throw std::logic_error{
                "An azimuth break is only allowed between two straights"};
        }
        if (distance(previous_straight->end(), current_straight->start()) >
            kPositionToleranceM) {
            throw std::logic_error{"Straights at an azimuth break must touch"};
        }
        if (previous_straight->azimuth().angular_distance(
                current_straight->azimuth()) <= kAzimuthToleranceRad) {
            throw std::logic_error{
                "An azimuth break requires the two straights to have different "
                "azimuths"};
        }
        return;
    }

    // Tangent joint. Only straight-to-straight can be verified now; joints that
    // involve a floating curve are checked after fitting.
    if (previous_straight != nullptr && current_straight != nullptr) {
        if (distance(previous_straight->end(), current_straight->start()) >
            kPositionToleranceM) {
            throw std::logic_error{"Tangent straights must touch"};
        }
        if (previous_straight->azimuth().angular_distance(
                current_straight->azimuth()) > kAzimuthToleranceRad) {
            throw std::logic_error{
                "Adjacent straights with different azimuths must be marked as an "
                "azimuth break"};
        }
    }
}

}  // namespace

void HorizontalAlignment::append(PlanElement element, JointContinuity joint) {
    if (!elements_.empty()) {
        validate_joint(elements_.back(), element, joint);
        joints_.push_back(joint);
    }
    elements_.push_back(std::move(element));
}

Length HorizontalAlignment::total_length() const {
    Length total = Length::zero();
    for (const auto& element : elements_) {
        if (const auto element_length = length_of(element)) {
            total = total + *element_length;
        }
    }
    return total;
}

}  // namespace maj0sted::domain
