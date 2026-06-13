#pragma once

// Pomocniki: po trafieniu dyrektywy kazdy token jawnie (peek -> decyzja -> consume).

#include <eu07/scene/cursor.hpp>
#include <eu07/scene/document.hpp>
#include <eu07/scene/match.hpp>

namespace eu07::scene::detail {

class ParseSession {
public:
    ParseSession(TokenStream& stream, DirectiveBlock& block, const std::size_t anchor) noexcept
        : stream_(stream)
        , block_(block)
        , anchor_(anchor) {}

    [[nodiscard]] bool empty() const noexcept { return stream_.empty(); }

    [[nodiscard]] const SourceToken& peek() const { return stream_.peek(); }

    [[nodiscard]] std::size_t line() const noexcept { return block_.line; }

    [[nodiscard]] bool at(const std::string_view keyword) const noexcept {
        return !empty() && isKeyword(peek().value, keyword);
    }

    [[nodiscard]] bool atIgnoreCase(const std::string_view keyword) const noexcept {
        return !empty() && isKeywordIgnoreCase(peek().value, keyword);
    }

    [[nodiscard]] bool onSameLine() const noexcept {
        return !empty() && peek().sourceLine == block_.line;
    }

    SourceToken take() {
        SourceToken token = stream_.consume();
        block_.tokens.push_back(token);
        return token;
    }

    [[nodiscard]] bool takeIf(const std::string_view keyword) {
        if (!at(keyword)) {
            return false;
        }
        take();
        return true;
    }

    [[nodiscard]] bool takeIfIgnoreCase(const std::string_view keyword) {
        if (!atIgnoreCase(keyword)) {
            return false;
        }
        take();
        return true;
    }

    void fail() noexcept {
        stream_.rewind(anchor_);
        block_.tokens.clear();
        block_.line = 0;
    }

private:
    TokenStream& stream_;
    DirectiveBlock& block_;
    std::size_t anchor_;
};

} // namespace eu07::scene::detail
