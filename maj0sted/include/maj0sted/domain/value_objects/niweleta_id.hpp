#pragma once

#include <compare>
#include <cstddef>
#include <cstdint>
#include <functional>

namespace maj0sted::domain {

/// Value Object: identity of a Niweleta aggregate. Cross-aggregate references
/// (e.g. parallelism) point at niwelety by this id, never by pointer. Value 0
/// denotes "no identity".
class NiweletaId {
public:
    using value_type = std::uint64_t;

    constexpr NiweletaId() noexcept = default;
    explicit constexpr NiweletaId(value_type value) noexcept : value_{value} {}

    [[nodiscard]] constexpr value_type value() const noexcept { return value_; }
    [[nodiscard]] constexpr bool is_null() const noexcept { return value_ == 0; }

    friend constexpr auto operator<=>(const NiweletaId&, const NiweletaId&) noexcept = default;

private:
    value_type value_{0};
};

}  // namespace maj0sted::domain

template <>
struct std::hash<maj0sted::domain::NiweletaId> {
    std::size_t operator()(const maj0sted::domain::NiweletaId& id) const noexcept {
        return std::hash<std::uint64_t>{}(id.value());
    }
};
