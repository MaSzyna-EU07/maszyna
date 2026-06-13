/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "scene/eu7/eu7_model_prefetch.h"

#include "model/AnimModel.h"
#include "model/MdlMngr.h"
#include "rendering/renderer.h"
#include "scene/eu7/eu7_pack_bench.h"
#include "utilities/utilities.h"
#include "vehicle/DynObj.h"

#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

namespace scene::eu7 {
namespace {

std::mutex g_pack_mesh_load_mutex;
std::unordered_set<std::string> g_session_warmed_textures;

[[nodiscard]] bool
pack_texture_usable( std::string texture_file ) {
    if( texture_file.empty() || texture_file == "none" || texture_file.front() == '*' ) {
        return false;
    }
    replace_slashes( texture_file );
    if( texture_file == "none" || texture_file.ends_with( "/none" ) ) {
        return false;
    }
    if( texture_file.find( "tr/none" ) != std::string::npos ) {
        return false;
    }
    return true;
}

void
preload_pack_model_file( std::string const &model_file ) {
    if( false == TModelsManager::IsModelCached( model_file ) ) {
        return;
    }

    if( auto *const mesh { TModelsManager::GetModel( model_file, false, false ) } ) {
        TAnimModel::warm_instanceable_cache( mesh );
    }
}

[[nodiscard]] bool
warm_one_pack_texture( std::string texture_file, std::unordered_set<std::string> &seen ) {
    if( false == pack_texture_usable( texture_file ) ) {
        return false;
    }
    replace_slashes( texture_file );
    if( false == seen.insert( texture_file ).second ) {
        return false;
    }
    if( false == g_session_warmed_textures.insert( texture_file ).second ) {
        return false;
    }

    auto const resolved { TextureTest( ToLower( texture_file ) ) };
    if( resolved.empty() ) {
        pack_bench_inc( &Eu7PackBench::main_texture_warm_miss );
        log_pack_texture_fail( texture_file );
        return false;
    }
    GfxRenderer->Fetch_Material( resolved );

    for( int skinindex { 1 }; skinindex <= 4; ++skinindex ) {
        auto const multi {
            TextureTest( ToLower( texture_file + "," + std::to_string( skinindex ) ) ) };
        if( multi.empty() ) {
            break;
        }
        GfxRenderer->Fetch_Material( multi );
    }
    return true;
}

} // namespace

void
reset_pack_texture_warm_cache() {
    g_session_warmed_textures.clear();
}

void
preload_pack_models( std::vector<Eu7Model> const &models ) {
    std::unordered_set<std::string> seen;
    seen.reserve( models.size() );

    for( auto const &model : models ) {
        auto model_file { model.model_file };
        if( model_file.empty() || model_file == "notload" ) {
            continue;
        }

        replace_slashes( model_file );
        if( false == seen.insert( model_file ).second ) {
            continue;
        }

        std::lock_guard<std::mutex> lock { g_pack_mesh_load_mutex };
        preload_pack_model_file( model_file );
    }
}

std::size_t
warm_pack_textures_main( Eu7Model const *const models, std::size_t const count ) {
    if( models == nullptr || count == 0 || GfxRenderer == nullptr ) {
        return 0;
    }

    std::unordered_set<std::string> seen;
    seen.reserve( std::min( count, std::size_t { 64 } ) );
    std::size_t warmed { 0 };

    for( std::size_t i { 0 }; i < count; ++i ) {
        if( warm_one_pack_texture( models[ i ].texture_file, seen ) ) {
            ++warmed;
        }
    }
    return warmed;
}

} // namespace scene::eu7
