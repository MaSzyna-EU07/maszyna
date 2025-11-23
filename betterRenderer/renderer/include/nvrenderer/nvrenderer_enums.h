#pragma once

namespace RendererEnums {

enum class RenderPassType {
  DepthOnly,
  Deferred,
  Forward,
  CubeMap,
  ShadowMap,
  Num,

  RendererWarmUp

};

enum class DrawType : uint8_t { Model, InstancedModel, Num };

}
