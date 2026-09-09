module;
#include <cstddef>
#include <string>
#include <functional>
#include <optional>
#include <stdexcept>
#include <vector>
#include <memory>

module eu07.rendering.nullrenderer;

std::unique_ptr<gfx_renderer> null_renderer::create_func()
{
    return std::unique_ptr<null_renderer>(new null_renderer());
}

bool null_renderer::renderer_register = gfx_renderer_factory::get_instance()->register_backend("null", null_renderer::create_func);
