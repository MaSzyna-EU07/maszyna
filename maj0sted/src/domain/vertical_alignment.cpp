#include "maj0sted/domain/geometry/vertical_alignment.hpp"

namespace maj0sted::domain {

Length VerticalAlignment::total_length() const {
    Length total = Length::zero();
    for (const auto& element : elements_) {
        total = total + length_of(element);
    }
    return total;
}

}  // namespace maj0sted::domain
