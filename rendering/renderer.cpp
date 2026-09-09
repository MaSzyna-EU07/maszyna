module;
#include <array>
#include <chrono>
#include <deque>
#include <limits>
#include <mutex>
#include <ostream>
#include <sstream>
#include <type_traits>
#include <utility>
#include <vector>
#include <cstddef>
#include <functional>
#include <optional>
#include <unordered_map>
#include <memory>
#include <string>

module eu07.rendering.renderer;
import eu07.utilities.logs;
/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/


std::unique_ptr<gfx_renderer> GfxRenderer;

bool gfx_renderer_factory::register_backend(const std::string &backend, gfx_renderer_factory::create_method func)
{
    backends[backend] = func;
    return true;
}

std::unique_ptr<gfx_renderer> gfx_renderer_factory::create(const std::string &backend)
{
    auto it = backends.find(backend);
    if (it != backends.end())
        return it->second();

    ErrorLog("renderer \"" + backend + "\" not found!");
    return nullptr;
}

gfx_renderer_factory *gfx_renderer_factory::get_instance()
{
    if (!instance)
        instance = new gfx_renderer_factory();

    return instance;
}

gfx_renderer_factory *gfx_renderer_factory::instance;


//---------------------------------------------------------------------------
