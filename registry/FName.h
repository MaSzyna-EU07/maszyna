#pragma once

#include <cstdint>
#include <string>
#include <string_view>

#include "NameRegistry.h"

struct FName
{
    using value_type = uint64_t;

    static constexpr value_type INVALID_ID = 0;

    value_type id = INVALID_ID;

    constexpr FName() noexcept = default;

    constexpr explicit FName(value_type value) noexcept
        : id(value)
    {
    }

    FName(const char* text)
        : id(text ? NameRegistry::GetOrCreate(text) : INVALID_ID)
    {
    }

    FName(const std::string& text)
        : id(text.empty() ? INVALID_ID : NameRegistry::GetOrCreate(text))
    {
    }

    FName(std::string_view text)
        : id(text.empty() ? INVALID_ID : NameRegistry::GetOrCreate(text))
    {
    }

    constexpr bool IsValid() const noexcept
    {
        return id != INVALID_ID;
    }

    constexpr value_type GetId() const noexcept
    {
        return id;
    }

    const std::string& ToString() const
    {
        return NameRegistry::GetString(id);
    }

    constexpr bool operator==(const FName& other) const noexcept
    {
        return id == other.id;
    }

    constexpr bool operator!=(const FName& other) const noexcept
    {
        return id != other.id;
    }
};