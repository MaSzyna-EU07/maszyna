#pragma once
#ifdef _WIN32
#include <windows.h>   // before any import: its guard must be set here
#endif

#include <memory>

#include "nvrenderer/nvrenderer.h"

// TSubModel/TModel3d are owned by the simulation core; a forward
// declaration here would be a second, conflicting declaration.

#include <array>
#include <chrono>
#include <cstddef>
#include <deque>
#include <limits>
#include <mutex>
#include <ostream>
#include <sstream>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>
import eu07.simcore;
import eu07.glm;
namespace Rt {
struct IRtModel {
  virtual TSubModel const* Intersect(NvRenderer::Renderable const& renderable,
                                     glm::dvec3 const& ro,
                                     glm::dvec3 const& rd) const {
    return nullptr;
  }
  virtual ~IRtModel() = default;
};

std::shared_ptr<IRtModel> CreateRtModel(TModel3d const* src,
                                        NvRenderer const* owner);

}  // namespace Rt