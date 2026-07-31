#pragma once

#include <compare>
#include <cstddef>
#include <cstdint>
#include <functional>

namespace maj0sted::domain {

/// Value Object: identity of a straight segment. A straight that takes part in
/// references (e.g. parallelism) carries a non-null id; anonymous straights used
/// purely as geometry may leave it null. Value 0 denotes "no identity".
class StraightId {
public:
    using value_type = std::uint64_t;

    constexpr StraightId() noexcept = default;
    explicit constexpr StraightId(value_type value) noexcept : value_{value} {}

    [[nodiscard]] constexpr value_type value() const noexcept { return value_; }
    [[nodiscard]] constexpr bool is_null() const noexcept { return value_ == 0; }

    friend constexpr auto operator<=>(const StraightId&, const StraightId&) noexcept = default;

private:
    value_type value_{0};
};

}  // namespace maj0sted::domain

template <>
struct std::hash<maj0sted::domain::StraightId> {
    std::size_t operator()(const maj0sted::domain::StraightId& id) const noexcept {
        return std::hash<std::uint64_t>{}(id.value());
    }
};
