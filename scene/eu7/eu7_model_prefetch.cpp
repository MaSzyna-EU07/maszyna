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

#include <list>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace scene::eu7 {
namespace {

constexpr std::size_t kMaxWarmedTextures { 2048 };

std::mutex g_pack_mesh_load_mutex;
std::unordered_set<std::string> g_session_warmed_textures;
std::list<std::string> g_session_warmed_textures_lru;
std::unordered_map<std::string, std::list<std::string>::iterator> g_session_warmed_textures_iters;

void
touch_warmed_texture_lru( std::string const &texture_file ) {
    auto const found { g_session_warmed_textures_iters.find( texture_file ) };
    if( found == g_session_warmed_textures_iters.end() ) {
        return;
    }
    g_session_warmed_textures_lru.splice(
        g_session_warmed_textures_lru.end(),
        g_session_warmed_textures_lru,
        found->second );
}

void
evict_warmed_texture_lru() {
    while(
        g_session_warmed_textures.size() >= kMaxWarmedTextures &&
        false == g_session_warmed_textures_lru.empty() ) {
        auto const oldest { g_session_warmed_textures_lru.front() };
        g_session_warmed_textures_lru.pop_front();
        g_session_warmed_textures_iters.erase( oldest );
        g_session_warmed_textures.erase( oldest );
        pack_bench_inc( &Eu7PackBench::stream_texture_cache_evictions );
    }
}

void
remember_warmed_texture( std::string texture_file ) {
    if( g_session_warmed_textures.contains( texture_file ) ) {
        touch_warmed_texture_lru( texture_file );
        return;
    }

    evict_warmed_texture_lru();
    g_session_warmed_textures.insert( texture_file );
    g_session_warmed_textures_lru.push_back( texture_file );
    g_session_warmed_textures_iters.emplace(
        std::move( texture_file ),
        std::prev( g_session_warmed_textures_lru.end() ) );
}

[[nodiscard]] bool
pack_texture_usable( std::string texture_file ) {
    if( texture_file.empty() || texture_file == "none" || texture_file.front() == '*' ) {
        return false;
    }
    if(
        texture_file.starts_with( "make:" ) ||
        texture_file.starts_with( "@" ) ||
        texture_file.starts_with( "none|" ) ) {
        return false;
    }
    if(
        texture_file.ends_with( ".e3d" ) ||
        texture_file.ends_with( ".t3d" ) ) {
        return false;
    }
    replace_slashes( texture_file );
    if(
        texture_file == "none" ||
        texture_file.ends_with( "/none" ) ||
        texture_file.ends_with( '/' ) ) {
        return false;
    }
    if(
        texture_file.find( "tr/none" ) != std::string::npos ||
        texture_file.find( '#' ) != std::string::npos ) {
        return false;
    }
    return true;
}

void
preload_pack_model_file( std::string const & ) {
    // Mesh warm runs on the main thread via ensure_pack_mesh_in_session_cache only.
    // GetModel mutates Global.asCurrentTexturePath and must not run on PACK workers.
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
    if( g_session_warmed_textures.contains( texture_file ) ) {
        return false;
    }

    auto const resolved { TextureTest( ToLower( texture_file ) ) };
    if( resolved.empty() ) {
        bool warmed { false };
        for( int skinindex { 1 }; skinindex <= 4; ++skinindex ) {
            auto const multi {
                TextureTest( ToLower( texture_file + "," + std::to_string( skinindex ) ) ) };
            if( multi.empty() ) {
                break;
            }
            GfxRenderer->Fetch_Material( multi );
            warmed = true;
        }
        if( false == warmed ) {
            pack_bench_inc( &Eu7PackBench::main_texture_warm_miss );
            log_pack_texture_fail( texture_file );
            return false;
        }
    }
    else {
        GfxRenderer->Fetch_Material( resolved );
    }

    remember_warmed_texture( texture_file );

    if( false == resolved.empty() ) {
        for( int skinindex { 1 }; skinindex <= 4; ++skinindex ) {
            auto const multi {
                TextureTest( ToLower( texture_file + "," + std::to_string( skinindex ) ) ) };
            if( multi.empty() ) {
                break;
            }
            GfxRenderer->Fetch_Material( multi );
        }
    }
    return true;
}

} // namespace

[[nodiscard]] bool
ensure_pack_mesh_in_session_cache(
    std::string model_file,
    std::unordered_map<std::string, TModel3d *> &session_cache ) {
    if( model_file.empty() || model_file == "notload" ) {
        return false;
    }
    replace_slashes( model_file );
    if( session_cache.contains( model_file ) ) {
        pack_bench_inc( &Eu7PackBench::stream_mesh_session_hit );
        return true;
    }

    auto const was_global_cached { TModelsManager::IsModelCached( model_file ) };
    TModel3d *mesh { nullptr };
    {
        std::lock_guard<std::mutex> lock { g_pack_mesh_load_mutex };
        mesh = TModelsManager::GetModel( model_file, false, false );
    }
    if( was_global_cached ) {
        pack_bench_inc( &Eu7PackBench::stream_mesh_global_hit );
    }
    else {
        pack_bench_inc( &Eu7PackBench::stream_mesh_disk_load );
    }
    if( mesh != nullptr ) {
        TAnimModel::warm_instanceable_cache( mesh );
    }
    session_cache.emplace( model_file, mesh );
    return true;
}

void
reset_pack_texture_warm_cache() {
    g_session_warmed_textures.clear();
    g_session_warmed_textures_lru.clear();
    g_session_warmed_textures_iters.clear();
}

void
preload_pack_model_paths( std::vector<std::string> const & ) {
}

void
preload_pack_models( std::vector<Eu7Model> const & ) {
}

void
preload_pack_models(
    std::vector<Eu7Model> const &,
    std::vector<std::string> const & ) {
}

std::size_t
warm_pack_texture_paths_main(
    std::string const *const paths,
    std::size_t const count,
    double const budget_ms,
    std::size_t *const processed_out ) {
    if( paths == nullptr || count == 0 || GfxRenderer == nullptr ) {
        if( processed_out != nullptr ) {
            *processed_out = 0;
        }
        return 0;
    }

    std::unordered_set<std::string> seen;
    seen.reserve( std::min( count, std::size_t { 64 } ) );
    std::size_t warmed { 0 };
    std::size_t processed { 0 };
    auto const started { std::chrono::steady_clock::now() };

    for( std::size_t i { 0 }; i < count; ++i ) {
        if(
            budget_ms > 0.0 &&
            processed > 0 ) {
            auto const elapsed_ms {
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - started ).count() };
            if( elapsed_ms >= budget_ms ) {
                break;
            }
        }
        if( warm_one_pack_texture( paths[ i ], seen ) ) {
            ++warmed;
        }
        ++processed;
    }

    if( processed_out != nullptr ) {
        *processed_out = processed;
    }
    return warmed;
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
