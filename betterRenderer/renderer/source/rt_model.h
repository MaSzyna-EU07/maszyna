#pragma once

#include <glm/vec3.hpp>
#include <memory>

#include "nvrenderer/nvrenderer.h"

class TSubModel;
class TModel3d;

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