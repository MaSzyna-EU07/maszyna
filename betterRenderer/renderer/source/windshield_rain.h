#pragma once

#include "nvrenderer/nvrenderer.h"

struct WindshieldRain {
  void Init(NvRenderer *in_renderer);
  void Resize(int width, int height);

  uint32_t width;
  uint32_t height;

  [[nodiscard]] nvrhi::ITexture *GetTexture() const;

  void Render(NvRenderer::RenderPass const &pass);
  bool IsRainShader(NvRenderer::MaterialTemplate const *mt) const;

  size_t texture_handle_droplets;
  size_t texture_handle_wipermask;

  nvrhi::TextureHandle texture_droplets;

  nvrhi::SamplerHandle sampler;
  nvrhi::SamplerHandle sampler_point;

  void CreatePso(nvrhi::ICommandList *command_list);

  NvRenderer *renderer;
  nvrhi::FramebufferHandle framebuffer;
  nvrhi::TextureHandle tex_droplets;
  nvrhi::TextureHandle tex_depth;
  nvrhi::TextureHandle tex_depth_temp;

  nvrhi::ShaderHandle ps_rain_anim;
  nvrhi::ShaderHandle vs_rain_anim;
  nvrhi::ShaderHandle cs_max_depth;

  nvrhi::BindingSetHandle bindings;
  nvrhi::BindingSetHandle bindings_max_depth;

  nvrhi::GraphicsPipelineHandle pso_droplets;
  nvrhi::ComputePipelineHandle pso_max_depth;

  nvrhi::BindingLayoutHandle binding_layout;

  std::unordered_map<size_t, nvrhi::BindingSetHandle> binding_sets_per_material;
};