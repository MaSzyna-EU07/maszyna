#pragma once

#include <cstddef>
#include <utility>
#include <vector>

#include "maj0sted/domain/geometry/plan_element.hpp"
#include "maj0sted/domain/value_objects/length.hpp"

namespace maj0sted::domain {

/// How two consecutive plan elements meet.
enum class JointContinuity {
    /// Uniform azimuth ("styczność") — the rule for every joint.
    Tangent,
    /// An explicitly marked azimuth discontinuity ("załamanie"). Allowed only
    /// between two straights that touch at different azimuths.
    AzimuthBreak,
};

/// "Niweleta w planie": a directional (ordered) list of plan elements.
///
/// Straights are anchored in XY; the curves float and are positioned by later
/// fitting. The list enforces the connection rules between consecutive elements:
///
///   * as a rule consecutive elements meet under a uniform azimuth (Tangent);
///   * two straights may meet at different azimuths only if the joint is
///     explicitly marked as an AzimuthBreak;
///   * two circular arcs with the same radius and the same direction may not
///     be adjacent (it would be a single arc).
///
/// Joints that involve a floating curve cannot be fully checked yet and are
/// accepted pending the fitting step.
class HorizontalAlignment {
public:
    /// Appends an element. The first element ignores @p joint; every later
    /// element is validated against the current last element.
    /// @throws std::logic_error if the joint violates a connection rule.
    void append(PlanElement element, JointContinuity joint = JointContinuity::Tangent);

    [[nodiscard]] const std::vector<PlanElement>& elements() const noexcept {
        return elements_;
    }
    /// Joint between element @p index and @p index + 1 (index < size() - 1).
    [[nodiscard]] JointContinuity joint(std::size_t index) const {
        return joints_.at(index);
    }

    [[nodiscard]] bool empty() const noexcept { return elements_.empty(); }
    [[nodiscard]] std::size_t size() const noexcept { return elements_.size(); }

    /// Sum of the known element lengths; arcs without a length are skipped.
    [[nodiscard]] Length total_length() const;

private:
    std::vector<PlanElement> elements_;
    std::vector<JointContinuity> joints_;  // size() == elements_.size() - 1
};

}  // namespace maj0sted::domain
