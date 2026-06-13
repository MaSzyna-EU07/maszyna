#pragma once

#include <chrono>
#include <cstddef>
#include <iostream>
#include <string>

namespace eu07::nmt {

class ProgressLine {
public:
    void begin(std::string label, const std::size_t total) {
        label_ = std::move(label);
        total_ = total > 0 ? total : 1;
        current_ = 0;
        lastPrint_ = std::chrono::steady_clock::now();
        render(true);
    }

    void set(const std::size_t current) {
        current_ = current > total_ ? total_ : current;
        render(false);
    }

    void advance(const std::size_t delta = 1) { set(current_ + delta); }

    void end() {
        set(total_);
        std::cerr << '\n';
    }

private:
    void render(const bool force) {
        const auto now = std::chrono::steady_clock::now();
        if (!force &&
            std::chrono::duration_cast<std::chrono::milliseconds>(now - lastPrint_).count() < 200) {
            return;
        }
        lastPrint_ = now;

        const int pct = static_cast<int>((100.0 * static_cast<double>(current_)) / static_cast<double>(total_));
        std::cerr << '\r' << label_ << ' ' << current_ << '/' << total_ << " (" << pct << "%)   "
                  << std::flush;
    }

    std::string label_;
    std::size_t total_ = 1;
    std::size_t current_ = 0;
    std::chrono::steady_clock::time_point lastPrint_{};
};

} // namespace eu07::nmt
