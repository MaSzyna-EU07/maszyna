#pragma once

#include <algorithm>
#include <thread>

namespace eu07::nmt {

[[nodiscard]] inline unsigned int workerThreadCount() {
    const unsigned int hw = std::thread::hardware_concurrency();
    if (hw <= 2) {
        return 1;
    }
    return hw - 1;
}

} // namespace eu07::nmt
