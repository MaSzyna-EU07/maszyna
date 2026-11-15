/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/renderer.h"
#include "utilities/Logs.h"

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

material_handle gfx_renderer::Terrain_Material( material_handle const Reuse, std::vector<gfx::terrain_layer> const &Layers, int const Samples, std::uint8_t const *Weights, glm::vec3 const &Placement )
{
    // without blending the chunk gets the material which covers the most of it
    if( Layers.empty() ) {
        return null_handle;
    }
    std::size_t best { 0 };
    if( ( Weights != nullptr ) && ( Samples > 0 ) ) {
        auto const samples { static_cast<std::size_t>( Samples + 1 ) * ( Samples + 1 ) };
        std::uint64_t bestsum { 0 };
        for( std::size_t layer = 0; layer < Layers.size(); ++layer ) {
            std::uint64_t sum { 0 };
            for( std::size_t i = 0; i < samples; ++i ) {
                sum += Weights[ layer * samples + i ];
            }
            if( sum > bestsum ) {
                bestsum = sum;
                best = layer;
            }
        }
    }
    return Layers[ best ].material;
}


//---------------------------------------------------------------------------
