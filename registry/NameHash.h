#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

constexpr uint64_t FNV1A_OFFSET_BASIS_64 = 14695981039346656037ull;
constexpr uint64_t FNV1A_PRIME_64 = 1099511628211ull;

constexpr uint64_t HashLiteral(const char* str, std::size_t len) noexcept
{
    uint64_t hash = FNV1A_OFFSET_BASIS_64;
    for (std::size_t i = 0; i < len; ++i)
    {
        hash ^= static_cast<unsigned char>(str[i]);
        hash *= FNV1A_PRIME_64;
    }
    return hash;
}

constexpr uint64_t HashLiteral(const char* str) noexcept
{
    uint64_t hash = FNV1A_OFFSET_BASIS_64;
    while (*str)
    {
        hash ^= static_cast<unsigned char>(*str++);
        hash *= FNV1A_PRIME_64;
    }
    return hash;
}

inline uint64_t HashRuntime(std::string_view str) noexcept
{
    uint64_t hash = FNV1A_OFFSET_BASIS_64;
    for (char c : str)
    {
        hash ^= static_cast<unsigned char>(c);
        hash *= FNV1A_PRIME_64;
    }
    return hash;
}