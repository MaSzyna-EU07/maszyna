#pragma once

#include <cstdint>
#include <memory>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace eu07::scene::binary {

class StringTable {
public:
    static constexpr std::uint32_t kEmpty = 0xFFFFFFFFu;

    StringTable() = default;
    StringTable(const StringTable&) = delete;
    StringTable& operator=(const StringTable&) = delete;
    StringTable(StringTable&&) noexcept = default;
    StringTable& operator=(StringTable&&) noexcept = default;

    [[nodiscard]] std::uint32_t intern(const std::string& text) {
        if (text.empty()) {
            return kEmpty;
        }
        {
            std::shared_lock read_lock(*mutex_);
            if (const auto found = index_.find(text); found != index_.end()) {
                return found->second;
            }
        }
        std::unique_lock write_lock(*mutex_);
        if (const auto found = index_.find(text); found != index_.end()) {
            return found->second;
        }
        const std::uint32_t id = static_cast<std::uint32_t>(strings_.size());
        strings_.push_back(text);
        index_.emplace(strings_.back(), id);
        return id;
    }

    [[nodiscard]] const std::string& resolve(const std::uint32_t id) const {
        static const std::string empty;
        if (id == kEmpty || id >= strings_.size()) {
            return empty;
        }
        return strings_[id];
    }

    [[nodiscard]] const std::vector<std::string>& strings() const noexcept {
        return strings_;
    }

    void load(std::vector<std::string> strings) {
        strings_ = std::move(strings);
        index_.clear();
        index_.reserve(strings_.size());
        for (std::uint32_t i = 0; i < strings_.size(); ++i) {
            index_.emplace(strings_[i], i);
        }
    }

    void mergeFrom(const StringTable& other) {
        for (const std::string& text : other.strings()) {
            intern(text);
        }
    }

private:
    std::unique_ptr<std::shared_mutex> mutex_ { std::make_unique<std::shared_mutex>() };
    std::vector<std::string> strings_;
    std::unordered_map<std::string, std::uint32_t> index_;
};

} // namespace eu07::scene::binary
