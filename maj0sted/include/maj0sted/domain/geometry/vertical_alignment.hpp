#pragma once

#include <cstddef>
#include <utility>
#include <vector>

#include "maj0sted/domain/geometry/profile_element.hpp"
#include "maj0sted/domain/value_objects/length.hpp"

namespace maj0sted::domain {

/// Geometria w profilu (rzut pionowy toru): uporządkowany ciąg pochyleń i łuków
/// pionowych, gdzie każdy element zaczyna się tam, gdzie kończy się poprzedni.
class VerticalAlignment {
public:
    void append(ProfileElement element) { elements_.push_back(std::move(element)); }

    [[nodiscard]] const std::vector<ProfileElement>& elements() const noexcept {
        return elements_;
    }
    [[nodiscard]] bool empty() const noexcept { return elements_.empty(); }
    [[nodiscard]] std::size_t size() const noexcept { return elements_.size(); }

    /// Sumaryczna długość toru w profilu.
    [[nodiscard]] Length total_length() const;

private:
    std::vector<ProfileElement> elements_;
};

}  // namespace maj0sted::domain
