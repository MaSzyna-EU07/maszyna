#pragma once

#include <memory>

#include "nvrenderer/nvrenderer.h"

// TSubModel/TModel3d are owned by the simulation core; a forward
// declaration here would be a second, conflicting declaration.

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