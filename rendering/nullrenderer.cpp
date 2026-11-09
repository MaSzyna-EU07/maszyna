#include "stdafx.h"
#include "rendering/nullrenderer.h"

#include "application/application.h"

void null_imgui_renderer::BeginFrame()
{
    auto &io { ImGui::GetIO() };
    if( false == io.Fonts->IsBuilt() ) {
        unsigned char *pixels { nullptr };
        int width { 0 };
        int height { 0 };
        io.Fonts->GetTexDataAsAlpha8( &pixels, &width, &height );
    }
}

bool null_renderer::Render()
{
    // the ui frame begun by the application has to be ended, or imgui won't start the next one
    Application.render_ui();
    return true;
}

std::unique_ptr<gfx_renderer> null_renderer::create_func()
{
    return std::make_unique<null_renderer>();
}

bool null_renderer::renderer_register = gfx_renderer_factory::get_instance()->register_backend("null", null_renderer::create_func);
