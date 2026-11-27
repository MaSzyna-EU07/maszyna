#include "stdafx.h"
#include "rendering/nullrenderer.h"

#include "application/application.h"
#include "utilities/Logs.h"

namespace
{

// the null renderer draws nothing, but parts of the ui make textures with gl calls of their own. the window has no hints,
// so any context will do for them: a machine without a graphics driver (a test runner) has only the basic one of the system
bool null_after_window(GLFWwindow *Window)
{
    glfwMakeContextCurrent(Window);
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        ErrorLog("Bad init: failed to initialize glad");
        return false;
    }
    return true;
}

gfx_backend_desc null_backend_desc()
{
    gfx_backend_desc desc;
    desc.name = "null";
    desc.create = &null_renderer::create_func;
    desc.after_window = &null_after_window;
    desc.gl_context = true;
    return desc;
}

} // namespace

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

bool null_renderer::renderer_register = gfx_renderer_factory::get_instance()->register_backend("null", null_backend_desc());
