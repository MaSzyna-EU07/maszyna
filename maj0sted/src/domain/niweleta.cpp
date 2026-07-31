#include "maj0sted/domain/niweleta.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <variant>

namespace maj0sted::domain {

Niweleta::Niweleta(NiweletaId id, std::string name)
    : id_{id}, name_{std::move(name)} {}

StraightId Niweleta::next_straight_id() noexcept { return StraightId{++straight_seq_}; }

StraightId Niweleta::add_straight(CartesianPosition start, CartesianPosition end,
                                  JointContinuity joint) {
    const StraightId id = next_straight_id();
    plan_.append(Straight{id, start, end}, joint);
    return id;
}

void Niweleta::add_plan_element(PlanElement element, JointContinuity joint) {
    if (auto* straight = std::get_if<Straight>(&element); straight != nullptr) {
        if (straight->id().is_null()) {
            // A straight added without an identity gets one from the niweleta.
            element = Straight{next_straight_id(), straight->start(), straight->end()};
        } else if (straight->id().value() > straight_seq_) {
            // Keep the counter ahead of any explicitly supplied id (e.g. on load).
            straight_seq_ = straight->id().value();
        }
    }
    plan_.append(std::move(element), joint);
}

void Niweleta::apply_fit(const FitResult& result) {
    const StraightId entry_id = result.entry.id();
    const StraightId exit_id = result.exit.id();
    if (entry_id.is_null() || exit_id.is_null()) {
        throw std::invalid_argument{"apply_fit: trimmed straights must be identified"};
    }

    const auto& elements = plan_.elements();
    const auto index_of = [&](StraightId id) -> std::size_t {
        for (std::size_t k = 0; k < elements.size(); ++k) {
            const auto* straight = std::get_if<Straight>(&elements[k]);
            if (straight != nullptr && !straight->id().is_null() &&
                straight->id() == id) {
                return k;
            }
        }
        throw std::invalid_argument{"apply_fit: straight not found in plan"};
    };
    const std::size_t i = index_of(entry_id);
    const std::size_t j = index_of(exit_id);
    if (j != i + 1) {
        throw std::invalid_argument{
            "apply_fit: the two straights must be adjacent (entry then exit)"};
    }

    // Rebuild the plan, re-validating every joint via append().
    HorizontalAlignment rebuilt;
    const auto add = [&](PlanElement element, JointContinuity joint) {
        rebuilt.append(std::move(element), joint);
    };

    for (std::size_t k = 0; k < i; ++k) {
        add(elements[k], k == 0 ? JointContinuity::Tangent : plan_.joint(k - 1));
    }
    add(result.entry, i == 0 ? JointContinuity::Tangent : plan_.joint(i - 1));
    for (const auto& curve_element : result.curve) {
        add(curve_element, JointContinuity::Tangent);
    }
    add(result.exit, JointContinuity::Tangent);
    for (std::size_t k = j + 1; k < elements.size(); ++k) {
        add(elements[k], plan_.joint(k - 1));
    }

    plan_ = std::move(rebuilt);
}

void Niweleta::add_profile_element(ProfileElement element) {
    profile_.append(std::move(element));
}

bool Niweleta::lengths_consistent(double tolerance_m) const {
    return std::abs(plan_length().metres() - profile_length().metres()) <= tolerance_m;
}

}  // namespace maj0sted::domain
