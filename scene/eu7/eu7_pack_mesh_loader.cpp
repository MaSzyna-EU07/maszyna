/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "scene/eu7/eu7_pack_mesh_loader.h"

#include "model/AnimModel.h"
#include "model/MdlMngr.h"
#include "scene/eu7/eu7_pack_bench.h"
#include "utilities/Globals.h"
#include "utilities/utilities.h"

#include <chrono>
#include <deque>
#include <mutex>
#include <queue>
#include <unordered_set>

namespace scene::eu7 {
namespace {

std::mutex g_pack_mesh_load_mutex;

struct PackMeshQueueEntry {
    int priority { 0 };
    std::uint64_t sequence { 0 };
    std::string model_file;
};

struct PackMeshQueueCompare {
    [[nodiscard]] bool
    operator()( PackMeshQueueEntry const &lhs, PackMeshQueueEntry const &rhs ) const {
        if( lhs.priority != rhs.priority ) {
            return lhs.priority > rhs.priority;
        }
        return lhs.sequence > rhs.sequence;
    }
};

struct PackMeshLoader {
    std::mutex mutex;
    std::priority_queue<
        PackMeshQueueEntry,
        std::vector<PackMeshQueueEntry>,
        PackMeshQueueCompare> queue;
    std::unordered_map<std::string, int> queued;
    std::unordered_map<std::string, TModel3d *> ready;
    std::uint64_t sequence { 0 };
    bool running { false };
};

PackMeshLoader g_loader;

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
pack_mesh_path_valid( std::string const &model_file ) {
    return false == model_file.empty() && model_file != "notload";
}

[[nodiscard]] bool
pack_mesh_pointer_usable( TModel3d *const mesh ) {
    return mesh != nullptr;
}

[[nodiscard]] bool
pack_mesh_cached_usable(
    std::unordered_map<std::string, TModel3d *> const &session_cache,
    std::string const &model_file ) {
    auto const found { session_cache.find( model_file ) };
    return found != session_cache.end() && pack_mesh_pointer_usable( found->second );
}

[[nodiscard]] bool
pack_mesh_globally_usable( std::string model_file ) {
    if( false == pack_mesh_path_valid( model_file ) ) {
        return false;
    }
    replace_slashes( model_file );
    if( false == TModelsManager::IsModelCached( model_file ) ) {
        return false;
    }

    TModel3d *mesh { nullptr };
    {
        std::lock_guard<std::mutex> lock { g_pack_mesh_load_mutex };
        mesh = TModelsManager::GetModel( model_file, false, false );
    }
    return pack_mesh_pointer_usable( mesh );
}

[[nodiscard]] bool
pop_pack_mesh_queue_entry( std::string &model_file ) {
    model_file.clear();
    while( false == g_loader.queue.empty() ) {
        auto entry { g_loader.queue.top() };
        g_loader.queue.pop();
        auto const found { g_loader.queued.find( entry.model_file ) };
        if(
            found == g_loader.queued.end() ||
            found->second != entry.priority ) {
            continue;
        }
        model_file = std::move( entry.model_file );
        return true;
    }
    return false;
}

void
load_pack_mesh_on_main_thread( std::string const &model_file ) {
    PackBenchTimer const load_timer { &Eu7PackBench::loader_thread_getmodel_ms };
    TModel3d *mesh { nullptr };
    {
        std::lock_guard<std::mutex> lock { g_pack_mesh_load_mutex };
        PackTexturePathScope const scope { model_file };
        mesh = TModelsManager::GetModel( model_file, false, false );
        if( pack_mesh_pointer_usable( mesh ) ) {
            TAnimModel::warm_instanceable_cache( mesh );
        }
    }
    pack_bench_inc( &Eu7PackBench::loader_thread_disk_loads );

    std::lock_guard<std::mutex> lock { g_loader.mutex };
    g_loader.ready.emplace( model_file, mesh );
    g_loader.queued.erase( model_file );
}

[[nodiscard]] std::size_t
pump_pack_mesh_loader_main(
    double const budget_ms,
    std::size_t const max_loads ) {
    if( false == g_loader.running ) {
        return 0;
    }

    auto const started { std::chrono::steady_clock::now() };
    std::size_t loaded { 0 };

    while( loaded < max_loads ) {
        if( budget_ms > 0.0 ) {
            auto const elapsed_ms {
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - started ).count() };
            if( elapsed_ms >= budget_ms ) {
                break;
            }
        }

        std::string model_file;
        {
            std::lock_guard<std::mutex> lock { g_loader.mutex };
            if( false == pop_pack_mesh_queue_entry( model_file ) ) {
                break;
            }
        }

        load_pack_mesh_on_main_thread( model_file );
        ++loaded;
    }

    return loaded;
}

[[nodiscard]] bool
try_consume_loader_ready(
    std::string const &model_file,
    std::unordered_map<std::string, TModel3d *> &session_cache ) {
    TModel3d *mesh { nullptr };
    {
        std::lock_guard<std::mutex> lock { g_loader.mutex };
        auto const found { g_loader.ready.find( model_file ) };
        if( found == g_loader.ready.end() ) {
            return false;
        }
        mesh = found->second;
        g_loader.ready.erase( found );
    }
    session_cache.emplace( model_file, mesh );
    pack_bench_inc( &Eu7PackBench::stream_mesh_async_ready );
    return pack_mesh_pointer_usable( mesh );
}

[[nodiscard]] bool
sync_resolve_global_cached_mesh(
    std::string const &model_file,
    std::unordered_map<std::string, TModel3d *> &session_cache ) {
    if( false == pack_mesh_globally_usable( model_file ) ) {
        return false;
    }

    TModel3d *mesh { nullptr };
    {
        PackBenchTimer const load_timer { &Eu7PackBench::main_cold_getmodel_ms };
        std::lock_guard<std::mutex> lock { g_pack_mesh_load_mutex };
        PackTexturePathScope const scope { model_file };
        mesh = TModelsManager::GetModel( model_file, false, false );
        pack_bench_inc( &Eu7PackBench::main_cold_getmodel_calls );
        if( pack_mesh_pointer_usable( mesh ) ) {
            TAnimModel::warm_instanceable_cache( mesh );
        }
    }
    pack_bench_inc( &Eu7PackBench::stream_mesh_global_hit );
    session_cache.emplace( model_file, mesh );
    return pack_mesh_pointer_usable( mesh );
}

void
enqueue_pack_mesh_load( std::string model_file, int const priority ) {
    if( false == pack_mesh_path_valid( model_file ) ) {
        return;
    }
    replace_slashes( model_file );

    {
        std::lock_guard<std::mutex> lock { g_loader.mutex };
        if( g_loader.ready.contains( model_file ) ) {
            return;
        }
        auto const found { g_loader.queued.find( model_file ) };
        if( found != g_loader.queued.end() ) {
            if( priority >= found->second ) {
                return;
            }
            found->second = priority;
        }
        else {
            g_loader.queued.emplace( model_file, priority );
        }
        g_loader.queue.push(
            PackMeshQueueEntry { priority, g_loader.sequence++, std::move( model_file ) } );
    }
    pack_bench_inc( &Eu7PackBench::stream_mesh_async_queued );
}

[[nodiscard]] bool
wait_for_loader_ready(
    std::string const &model_file,
    double const block_budget_ms ) {
    auto const deadline {
        block_budget_ms > 0.0 ?
            std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double, std::milli>( block_budget_ms ) ) :
            std::chrono::steady_clock::time_point::max() };

    while( true ) {
        {
            std::lock_guard<std::mutex> lock { g_loader.mutex };
            if( g_loader.ready.contains( model_file ) ) {
                return true;
            }
            if( g_loader.queued.contains( model_file ) == false ) {
                return false;
            }
        }

        if( block_budget_ms > 0.0 && std::chrono::steady_clock::now() >= deadline ) {
            return false;
        }

        auto remaining_ms { block_budget_ms };
        if( block_budget_ms > 0.0 ) {
            remaining_ms = std::chrono::duration<double, std::milli>(
                deadline - std::chrono::steady_clock::now() ).count();
            if( remaining_ms <= 0.0 ) {
                return false;
            }
        }

        (void)pump_pack_mesh_loader_main( remaining_ms, 1 );
    }
}

} // namespace

void
start_pack_mesh_loader() {
    stop_pack_mesh_loader();
    g_loader.running = true;
}

void
stop_pack_mesh_loader() {
    g_loader.running = false;

    std::lock_guard<std::mutex> lock { g_loader.mutex };
    g_loader.queue = decltype( g_loader.queue )();
    g_loader.queued.clear();
    g_loader.ready.clear();
}

void
reset_pack_mesh_loader() {
    stop_pack_mesh_loader();
}

void
request_pack_mesh_load( std::string const &model_file, int const priority ) {
    if( false == g_loader.running ) {
        return;
    }
    if( false == pack_mesh_path_valid( model_file ) ) {
        return;
    }
    if( pack_mesh_globally_usable( model_file ) ) {
        return;
    }
    enqueue_pack_mesh_load( model_file, priority );
}

void
request_pack_mesh_load_paths(
    std::vector<std::string> const &model_files,
    std::size_t const max_enqueue,
    int const priority ) {
    if( false == g_loader.running ) {
        return;
    }

    std::size_t enqueued { 0 };
    for( auto const &path : model_files ) {
        if( max_enqueue > 0 && enqueued >= max_enqueue ) {
            break;
        }
        if( false == pack_mesh_path_valid( path ) ) {
            continue;
        }
        if( pack_mesh_globally_usable( path ) ) {
            continue;
        }
        enqueue_pack_mesh_load( path, priority );
        ++enqueued;
    }
}

[[nodiscard]] std::size_t
pump_pack_mesh_loader(
    double const budget_ms,
    std::size_t const max_loads ) {
    return pump_pack_mesh_loader_main( budget_ms, max_loads );
}

[[nodiscard]] TModel3d *
session_cached_pack_mesh(
    std::unordered_map<std::string, TModel3d *> const &session_cache,
    std::string const &model_file ) {
    auto const found { session_cache.find( model_file ) };
    return found != session_cache.end() ? found->second : nullptr;
}

[[nodiscard]] TModel3d *
ensure_pack_mesh_in_session_cache(
    std::string model_file,
    std::unordered_map<std::string, TModel3d *> &session_cache,
    PackMeshLoadWait const wait,
    double const block_budget_ms ) {
    if( false == pack_mesh_path_valid( model_file ) ) {
        return nullptr;
    }
    replace_slashes( model_file );
    if( pack_mesh_cached_usable( session_cache, model_file ) ) {
        pack_bench_inc( &Eu7PackBench::stream_mesh_session_hit );
        return session_cached_pack_mesh( session_cache, model_file );
    }
    if( session_cache.contains( model_file ) ) {
        session_cache.erase( model_file );
    }

    if( try_consume_loader_ready( model_file, session_cache ) ) {
        return session_cached_pack_mesh( session_cache, model_file );
    }
    if( sync_resolve_global_cached_mesh( model_file, session_cache ) ) {
        return session_cached_pack_mesh( session_cache, model_file );
    }

    request_pack_mesh_load( model_file );

    if( wait == PackMeshLoadWait::BlockUntilReady ) {
        if( wait_for_loader_ready( model_file, block_budget_ms ) ) {
            if( try_consume_loader_ready( model_file, session_cache ) ) {
                return session_cached_pack_mesh( session_cache, model_file );
            }
        }
        pack_bench_inc( &Eu7PackBench::stream_mesh_async_wait_timeout );
        return nullptr;
    }

    return nullptr;
}

[[nodiscard]] std::size_t
pack_mesh_loader_queue_depth() {
    std::lock_guard<std::mutex> lock { g_loader.mutex };
    return g_loader.queued.size();
}

[[nodiscard]] bool
try_adopt_pack_mesh_for_slice(
    std::string model_file,
    std::unordered_map<std::string, TModel3d *> &session_cache ) {
    if( model_file.empty() || model_file == "notload" ) {
        return true;
    }
    replace_slashes( model_file );
    if( pack_mesh_cached_usable( session_cache, model_file ) ) {
        return true;
    }
    if( session_cache.contains( model_file ) ) {
        session_cache.erase( model_file );
    }
    if( try_consume_loader_ready( model_file, session_cache ) ) {
        return true;
    }
    return sync_resolve_global_cached_mesh( model_file, session_cache );
}

[[nodiscard]] std::size_t
drain_pack_mesh_loader_ready(
    std::unordered_map<std::string, TModel3d *> &session_cache,
    std::size_t const max_drain ) {
    std::vector<std::pair<std::string, TModel3d *>> drained;
    {
        std::lock_guard<std::mutex> lock { g_loader.mutex };
        drained.reserve( g_loader.ready.size() );
        for( auto it { g_loader.ready.begin() }; it != g_loader.ready.end(); ) {
            if( max_drain > 0 && drained.size() >= max_drain ) {
                break;
            }
            drained.emplace_back( it->first, it->second );
            it = g_loader.ready.erase( it );
        }
    }

    std::size_t adopted { 0 };
    for( auto & [path, mesh] : drained ) {
        session_cache.emplace( std::move( path ), mesh );
        pack_bench_inc( &Eu7PackBench::stream_mesh_async_ready );
        pack_bench_inc( &Eu7PackBench::stream_mesh_async_drained );
        if( pack_mesh_pointer_usable( mesh ) ) {
            ++adopted;
        }
    }
    return adopted;
}

[[nodiscard]] std::size_t
pack_mesh_loader_ready_count() {
    std::lock_guard<std::mutex> lock { g_loader.mutex };
    return g_loader.ready.size();
}

[[nodiscard]] std::size_t
pack_mesh_loader_worker_count() {
    return 0;
}

} // namespace scene::eu7
