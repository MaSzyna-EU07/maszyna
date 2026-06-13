#pragma once

#include <eu07/nmt/profile_types.hpp>

namespace eu07::nmt {

[[nodiscard]] inline double profileSampleHeightZ(const RouteChainageSample& sample) {
    if (sample.nmtHeight) {
        return *sample.nmtHeight;
    }
    return sample.trackHeight;
}

} // namespace eu07::nmt
