#pragma once

#include <eu07/parser.hpp>

#include <cstddef>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace eu07::scene {

class TokenStream {
public:
    explicit TokenStream(const std::vector<SourceToken>& tokens) noexcept
        : tokens_(&tokens) {}

    [[nodiscard]] bool empty() const noexcept { return index_ >= tokens_->size(); }

    [[nodiscard]] const SourceToken& peek() const {
        if (empty()) {
            throw std::runtime_error("TokenStream: koniec strumienia");
        }
        return (*tokens_)[index_];
    }

    // Returns a reference into the underlying token vector (which outlives the
    // stream), so callers that only read a field (e.g. .sourceLine) or bind to
    // const& pay no per-token std::string copy. Callers that need an owning
    // copy still get one via copy-initialisation, exactly as before.
    [[nodiscard]] const SourceToken& consume() {
        const SourceToken& token = peek();
        ++index_;
        return token;
    }

    void skip(const std::size_t count = 1) {
        if (index_ + count > tokens_->size()) {
            throw std::runtime_error("TokenStream: przekroczenie konca");
        }
        index_ += count;
    }

    [[nodiscard]] std::size_t index() const noexcept { return index_; }

    [[nodiscard]] std::size_t remaining() const noexcept {
        return tokens_->size() - index_;
    }

    [[nodiscard]] bool peekIs(const std::string_view keyword) const {
        return !empty() && peek().value == keyword;
    }

    [[nodiscard]] std::size_t checkpoint() const noexcept { return index_; }

    void rewind(const std::size_t position) { index_ = position; }

private:
    const std::vector<SourceToken>* tokens_;
    std::size_t index_ = 0;
};

} // namespace eu07::scene
