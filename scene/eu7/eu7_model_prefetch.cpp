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
#include "rendering/renderer.h"
#include "scene/eu7/eu7_pack_bench.h"
#include "utilities/Globals.h"
#include "utilities/utilities.h"
#include "vehicle/DynObj.h"

#include <algorithm>
#include <list>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace scene::eu7 {
namespace {

constexpr std::size_t kMaxWarmedTextures { 2048 };
constexpr std::size_t kMaxPackMaterialAssignCache { 4096 };

std::unordered_set<std::string> g_session_warmed_textures;
std::list<std::string> g_session_warmed_textures_lru;
std::unordered_map<std::string, std::list<std::string>::iterator> g_session_warmed_textures_iters;
std::unordered_map<std::string, material_data> g_pack_material_assign_cache;

[[nodiscard]] std::string
pack_material_cache_key(
    std::string const &model_file,
    std::string const &texture_file ) {
    return model_file + '\x1e' + texture_file;
}

void
evict_pack_material_assign_cache_if_needed() {
    while( g_pack_material_assign_cache.size() >= kMaxPackMaterialAssignCache ) {
        if( g_pack_material_assign_cache.empty() ) {
            break;
        }
        g_pack_material_assign_cache.erase( g_pack_material_assign_cache.begin() );
    }
}

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
    // GetModel + Init must run on the main thread (Global.asCurrentTexturePath + OpenGL).
}

[[nodiscard]] bool
pack_texture_path_is_rooted( std::string const &texture_file ) {
    return texture_file.starts_with( "dynamic/" ) ||
        texture_file.starts_with( "textures/" ) ||
        texture_file.starts_with( "scenery/" ) ||
        texture_file.starts_with( "models/" );
}

[[nodiscard]] std::string
pack_model_texture_dir( std::string model_file ) {
    if( model_file.empty() || model_file == "notload" ) {
        return {};
    }
    replace_slashes( model_file );
    erase_extension( model_file );
    auto const slash_pos { model_file.rfind( '/' ) };
    if( slash_pos == std::string::npos ) {
        return {};
    }
    return model_file.substr( 0, slash_pos + 1 );
}

void
append_pack_texture_candidate(
    std::vector<std::string> &candidates,
    std::string candidate ) {
    if( candidate.empty() ) {
        return;
    }
    replace_slashes( candidate );
    if(
        std::find( candidates.begin(), candidates.end(), candidate ) ==
        candidates.end() ) {
        candidates.emplace_back( std::move( candidate ) );
    }
}

[[nodiscard]] std::vector<std::string>
pack_texture_resolve_candidates(
    std::string const &model_file,
    std::string texture_file ) {
    std::vector<std::string> candidates;
    candidates.reserve( 4 );
    if( false == pack_texture_usable( texture_file ) ) {
        return candidates;
    }
    replace_slashes( texture_file );
    append_pack_texture_candidate( candidates, texture_file );

    auto const model_dir { pack_model_texture_dir( model_file ) };
    if( false == model_dir.empty() ) {
        append_pack_texture_candidate( candidates, model_dir + texture_file );
    }
    if( false == pack_texture_path_is_rooted( texture_file ) ) {
        append_pack_texture_candidate( candidates, paths::textures + texture_file );
    }
    return candidates;
}

class PackTexturePathScope {
public:
    explicit PackTexturePathScope( std::string const &model_file ) {
        m_saved_path = Global.asCurrentTexturePath;
        auto const model_dir { pack_model_texture_dir( model_file ) };
        if( false == model_dir.empty() && model_file.find( '/' ) != std::string::npos ) {
            Global.asCurrentTexturePath = model_dir;
            m_active = true;
        }
    }

    ~PackTexturePathScope() {
        if( m_active ) {
            Global.asCurrentTexturePath = std::move( m_saved_path );
        }
    }

    PackTexturePathScope( PackTexturePathScope const & ) = delete;
    PackTexturePathScope &operator=( PackTexturePathScope const & ) = delete;

private:
    std::string m_saved_path;
    bool m_active { false };
};

[[nodiscard]] bool
fetch_pack_texture_material( std::string const &texture_file ) {
    auto const resolved { TextureTest( ToLower( texture_file ) ) };
    if( false == resolved.empty() ) {
        GfxRenderer->Fetch_Material( resolved );
        return true;
    }

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
    return warmed;
}

[[nodiscard]] bool
warm_one_pack_texture(
    std::string texture_file,
    std::unordered_set<std::string> &seen,
    std::string const &model_file = {} ) {
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

    PackTexturePathScope const scope { model_file };
    for( auto const &candidate : pack_texture_resolve_candidates( model_file, texture_file ) ) {
        if( fetch_pack_texture_material( candidate ) ) {
            remember_warmed_texture( texture_file );
            return true;
        }
    }

    pack_bench_inc( &Eu7PackBench::main_texture_warm_miss );
    log_pack_texture_fail( texture_file );
    return false;
}

[[nodiscard]] std::string
find_pack_model_file_for_texture(
    Eu7Model const *const models,
    std::size_t const model_count,
    std::string texture_file ) {
    if( models == nullptr || model_count == 0 ) {
        return {};
    }
    replace_slashes( texture_file );
    for( std::size_t i { 0 }; i < model_count; ++i ) {
        auto candidate { models[ i ].texture_file };
        replace_slashes( candidate );
        if( candidate == texture_file ) {
            return models[ i ].model_file;
        }
    }
    return {};
}

} // namespace

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
    std::size_t *const processed_out,
    Eu7Model const *const models,
    std::size_t const model_count ) {
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
        if( warm_one_pack_texture(
                paths[ i ],
                seen,
                find_pack_model_file_for_texture( models, model_count, paths[ i ] ) ) ) {
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
warm_pack_textures_main(
    Eu7Model const *const models,
    std::size_t const count,
    double const budget_ms ) {
    if( models == nullptr || count == 0 || GfxRenderer == nullptr ) {
        return 0;
    }

    std::unordered_set<std::string> seen;
    seen.reserve( std::min( count, std::size_t { 64 } ) );
    std::size_t warmed { 0 };
    auto const started { std::chrono::steady_clock::now() };

    for( std::size_t i { 0 }; i < count; ++i ) {
        if(
            budget_ms > 0.0 &&
            i > 0 ) {
            auto const elapsed_ms {
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - started ).count() };
            if( elapsed_ms >= budget_ms ) {
                break;
            }
        }
        if( warm_one_pack_texture(
                models[ i ].texture_file,
                seen,
                models[ i ].model_file ) ) {
            ++warmed;
        }
    }
    return warmed;
}

[[nodiscard]] bool
assign_pack_texture(
    material_data &material,
    std::string const &model_file,
    std::string const &texture_file,
    std::string const &resolved_texture,
    std::uint32_t const textures_alpha ) {
    if( false == pack_texture_usable( texture_file ) ) {
        if( textures_alpha != 0 ) {
            material.textures_alpha = static_cast<int>( textures_alpha );
        }
        return true;
    }

    auto const cache_key {
        resolved_texture.empty() ?
            pack_material_cache_key( model_file, texture_file ) :
            pack_material_cache_key( model_file, resolved_texture ) };
    auto const cached { g_pack_material_assign_cache.find( cache_key ) };
    if(
        cached != g_pack_material_assign_cache.end() &&
        cached->second.replacable_skins[ 1 ] != null_handle ) {
        material = cached->second;
        if( textures_alpha != 0 ) {
            material.textures_alpha = static_cast<int>( textures_alpha );
        }
        return true;
    }

    if( false == resolved_texture.empty() ) {
        material = {};
        material.assign( resolved_texture );
        if( material.replacable_skins[ 1 ] != null_handle ) {
            if( textures_alpha != 0 ) {
                material.textures_alpha = static_cast<int>( textures_alpha );
            }
            evict_pack_material_assign_cache_if_needed();
            g_pack_material_assign_cache.emplace( cache_key, material );
            return true;
        }
    }

    PackTexturePathScope const scope { model_file };
    for( auto const &candidate : pack_texture_resolve_candidates( model_file, texture_file ) ) {
        material = {};
        material.assign( candidate );
        if( material.replacable_skins[ 1 ] != null_handle ) {
            if( textures_alpha != 0 ) {
                material.textures_alpha = static_cast<int>( textures_alpha );
            }
            evict_pack_material_assign_cache_if_needed();
            g_pack_material_assign_cache.emplace( cache_key, material );
            return true;
        }
    }

    pack_bench_inc( &Eu7PackBench::main_texture_assign_fail );
    log_pack_texture_fail( texture_file );
    return false;
}

} // namespace scene::eu7
