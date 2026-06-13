#pragma once

#include <cctype>
#include <string_view>

namespace eu07::scene {

[[nodiscard]] inline bool asciiEquals(std::string_view a, std::string_view b) noexcept {
    return a == b;
}

[[nodiscard]] inline bool asciiEqualsIgnoreCase(std::string_view a, std::string_view b) noexcept {
    if (a.size() != b.size()) {
        return false;
    }
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (std::tolower(static_cast<unsigned char>(a[i])) !=
            std::tolower(static_cast<unsigned char>(b[i]))) {
            return false;
        }
    }
    return true;
}

[[nodiscard]] inline bool isKeyword(std::string_view token, std::string_view keyword) noexcept {
    return asciiEquals(token, keyword);
}

[[nodiscard]] inline bool isKeywordIgnoreCase(std::string_view token, std::string_view keyword) noexcept {
    return asciiEqualsIgnoreCase(token, keyword);
}

} // namespace eu07::scene
