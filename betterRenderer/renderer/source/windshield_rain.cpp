#include "windshield_rain.h"

#include "gbuffer.h"
#include "gbufferblitpass.h"
#include "nvrendererbackend.h"
#include "nvrhi/utils.h"
#include "nvtexture.h"

void WindshieldRain::Init(NvRenderer* in_renderer) {
  renderer = in_renderer;
  {
    auto const src_desc = renderer->m_gbuffer->m_gbuffer_depth->getDesc();
    width = src_desc.width / 2;
    height = src_desc.height / 2;
  }
  auto backend = renderer->GetBackend();
  tex_droplets = backend->GetDevice()->createTexture(
      nvrhi::TextureDesc()
          .setDebugName("Raindrop buffer")
          .setWidth(width)
          .setHeight(height)
          .setFormat(nvrhi::Format::R32_FLOAT)
          .setIsRenderTarget(true)
          .setInitialState(nvrhi::ResourceStates::RenderTarget)
          .setClearValue(nvrhi::Color(0.))
          .setUseClearValue(true)
          .setKeepInitialState(true));
  tex_depth_temp = backend->GetDevice()->createTexture(
      nvrhi::TextureDesc()
          .setDebugName("Raindrop depths temp")
          .setWidth(width)
          .setHeight(height)
          .setFormat(nvrhi::Format::D32)
          .setIsUAV(true)
          .setInitialState(nvrhi::ResourceStates::UnorderedAccess)
          .setKeepInitialState(true));
  tex_depth = backend->GetDevice()->createTexture(
      nvrhi::TextureDesc()
          .setDebugName("Raindrop depths")
          .setWidth(width)
          .setHeight(height)
          .setFormat(nvrhi::Format::D32)
          .setIsRenderTarget(true)
          .setInitialState(nvrhi::ResourceStates::DepthWrite)
          .setClearValue(nvrhi::Color(0.))
          .setUseClearValue(true)
          .setKeepInitialState(true));
  framebuffer = backend->GetDevice()->createFramebuffer(
      nvrhi::FramebufferDesc()
          .addColorAttachment(tex_droplets)
          .setDepthAttachment(tex_depth));

  ps_rain_anim =
      backend->CreateShader("windshield_rain_anim", nvrhi::ShaderType::Pixel);

  vs_rain_anim =
      backend->CreateShader("default_vertex_no_jitter", nvrhi::ShaderType::Vertex);

  cs_max_depth =
      backend->CreateShader("max_depth_4x4", nvrhi::ShaderType::Compute);

  auto const texture_manager = renderer->GetTextureManager();

  texture_handle_droplets = texture_manager->FetchTexture(
      "textures/fx/raindrops_height", GL_R, 0, false);

  texture_handle_wipermask = texture_manager->FetchTexture(
      "dynamic/pkp/ep09_v1/104ec/szyby_wipermask", GL_SRGB, 0, false);

  texture_manager->RegisterExternalTexture("system/raindrops_buffer",
                                           tex_droplets);

  {
    nvrhi::SamplerHandle sampler = backend->GetDevice()->createSampler(
        nvrhi::SamplerDesc().setAllFilters(true).setAllAddressModes(
            nvrhi::SamplerAddressMode::Clamp));
    nvrhi::BindingLayoutHandle binding_layout;
    nvrhi::utils::CreateBindingSetAndLayout(
        backend->GetDevice(), nvrhi::ShaderType::Compute, 0,
        nvrhi::BindingSetDesc()
            .addItem(nvrhi::BindingSetItem::Texture_SRV(
                0, renderer->m_gbuffer->m_gbuffer_depth))
            .addItem(nvrhi::BindingSetItem::Texture_UAV(0, tex_depth_temp))
            .addItem(nvrhi::BindingSetItem::Sampler(0, sampler)),
        binding_layout, bindings_max_depth);
    pso_max_depth = backend->GetDevice()->createComputePipeline(
        nvrhi::ComputePipelineDesc()
            .addBindingLayout(binding_layout)
            .setComputeShader(cs_max_depth));
  }
}

void WindshieldRain::Resize(int width, int height) {}

nvrhi::ITexture* WindshieldRain::GetTexture() const { return tex_droplets; }

void WindshieldRain::Render(NvRenderer::RenderPass const& pass) {
  if (!pso_droplets) {
    CreatePso(pass.m_command_list_draw);
  }
  if (renderer->m_dynamic_with_kabina < renderer->m_dynamics.size()) {
    const auto& dynamic = renderer->m_dynamics[renderer->m_dynamic_with_kabina];
    pass.m_command_list_draw->beginMarker("Render cab rain buffer");
    pass.m_command_list_draw->clearTextureFloat(
        tex_droplets, nvrhi::AllSubresources, nvrhi::Color(0.));
    {
      nvrhi::ComputeState compute_state;
      compute_state.setPipeline(pso_max_depth);
      compute_state.addBindingSet(bindings_max_depth);
      pass.m_command_list_draw->setComputeState(compute_state);
      pass.m_command_list_draw->dispatch((width + 7) / 8, (height + 7) / 8);
    }
    pass.m_command_list_draw->copyTexture(tex_depth, nvrhi::TextureSlice(),
                                          tex_depth_temp, nvrhi::TextureSlice());
    for (auto const& item : dynamic.m_renderable_kabina.m_items) {
      auto const& material = renderer->m_material_cache[item.m_material - 1];
      if (!IsRainShader(material.m_template)) {
        continue;
      }

      nvrhi::GraphicsState gfx_state{};
      nvrhi::DrawArguments draw_arguments;
      bool indexed;

      if (!renderer->BindGeometry(item.m_geometry, pass, gfx_state,
                                  draw_arguments, indexed)) {
        continue;
      }

      auto transform = item.m_transform;
      transform[3] -= pass.m_origin;

      auto& binding_set = binding_sets_per_material[item.m_material];
      if (!binding_set) {
        auto const& material = renderer->m_material_cache[item.m_material - 1];
        auto texture_wipermask = renderer->GetTextureManager()->GetRhiTexture(
            material.m_texture_handles[2], pass.m_command_list_draw);
        binding_set = renderer->m_backend->GetDevice()->createBindingSet(
            nvrhi::BindingSetDesc()
                .addItem(nvrhi::BindingSetItem::ConstantBuffer(
                    0, renderer->m_drawconstant_buffer))
                .addItem(nvrhi::BindingSetItem::ConstantBuffer(
                    2, renderer->m_gbuffer_blit->m_draw_constants))
                .addItem(nvrhi::BindingSetItem::PushConstants(
                    1, sizeof(NvRenderer::PushConstantsDraw)))
                .addItem(
                    nvrhi::BindingSetItem::Texture_SRV(0, texture_droplets))
                .addItem(
                    nvrhi::BindingSetItem::Texture_SRV(1, texture_wipermask))
                .addItem(nvrhi::BindingSetItem::Sampler(0, sampler))
                .addItem(nvrhi::BindingSetItem::Sampler(1, sampler_point)),
            binding_layout);
      }

      gfx_state.setFramebuffer(framebuffer);
      gfx_state.setPipeline(pso_droplets);
      gfx_state.addBindingSet(binding_set);
      gfx_state.setViewport(nvrhi::ViewportState().addViewportAndScissorRect(
          framebuffer->getFramebufferInfo().getViewport()));

      pass.m_command_list_draw->beginMarker(item.m_name.data());

      pass.m_command_list_draw->setGraphicsState(gfx_state);

      {
        NvRenderer::PushConstantsDraw data{};
        data.m_modelview = static_cast<glm::mat3x4>(
            transpose(static_cast<glm::dmat4>(pass.m_transform) *
                      static_cast<glm::dmat4>(transform)));
        data.m_modelview_history = data.m_modelview;
        pass.m_command_list_draw->setPushConstants(&data, sizeof(data));
      }

      if (indexed)
        pass.m_command_list_draw->drawIndexed(draw_arguments);
      else
        pass.m_command_list_draw->draw(draw_arguments);
      pass.m_command_list_draw->endMarker();
    }
    pass.m_command_list_draw->endMarker();
  }
  std::erase_if(binding_sets_per_material, [this](auto const& kv) {
    auto const& material = renderer->m_material_cache[kv.first - 1];
    return renderer->GetCurrentFrame() - material.m_last_frame_requested > 100;
  });
}

bool WindshieldRain::IsRainShader(
    NvRenderer::MaterialTemplate const* mt) const {
  return mt->m_name == "windshield_rain";
}

void WindshieldRain::CreatePso(nvrhi::ICommandList* command_list) {
  auto backend = renderer->GetBackend();

  texture_droplets = renderer->GetTextureManager()->GetRhiTexture(
      texture_handle_droplets, command_list);

  sampler = backend->GetDevice()->createSampler(
      nvrhi::SamplerDesc()
          .setAllFilters(true)
          .setMipFilter(false)
          .setAllAddressModes(nvrhi::SamplerAddressMode::Repeat));

  sampler_point = backend->GetDevice()->createSampler(
      nvrhi::SamplerDesc().setAllFilters(false).setAllAddressModes(
          nvrhi::SamplerAddressMode::Repeat));

  binding_layout = backend->GetDevice()->createBindingLayout(
      nvrhi::BindingLayoutDesc()
          .setVisibility(nvrhi::ShaderType::AllGraphics)
          .setRegisterSpace(0)
          .addItem(nvrhi::BindingLayoutItem::ConstantBuffer(0))
          .addItem(nvrhi::BindingLayoutItem::VolatileConstantBuffer(2))
          .addItem(nvrhi::BindingLayoutItem::PushConstants(
              1, sizeof(NvRenderer::PushConstantsDraw)))
          .addItem(nvrhi::BindingLayoutItem::Texture_SRV(0))
          .addItem(nvrhi::BindingLayoutItem::Texture_SRV(1))
          .addItem(nvrhi::BindingLayoutItem::Sampler(0))
          .addItem(nvrhi::BindingLayoutItem::Sampler(1)));

  pso_droplets = backend->GetDevice()->createGraphicsPipeline(
      nvrhi::GraphicsPipelineDesc()
          .addBindingLayout(binding_layout)
          .setVertexShader(vs_rain_anim)
          .setInputLayout(renderer->m_input_layout[static_cast<size_t>(
              RendererEnums::DrawType::Model)])
          .setPixelShader(ps_rain_anim)
          .setRenderState(
              nvrhi::RenderState()
                  .setDepthStencilState(
                      nvrhi::DepthStencilState()
                          .enableDepthTest()
                          .enableDepthWrite()
                          .disableStencil()
                          .setDepthFunc(nvrhi::ComparisonFunc::Greater))
                  .setRasterState(nvrhi::RasterState()
                                      .setFillSolid()
                                      .enableDepthClip()
                                      .disableScissor()
                                      .setCullFront())
                  .setBlendState(nvrhi::BlendState().setRenderTarget(
                      0, nvrhi::BlendState::RenderTarget().disableBlend())))
          .setPrimType(nvrhi::PrimitiveType::TriangleList),
      framebuffer);
}
