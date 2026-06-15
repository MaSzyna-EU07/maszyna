/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

class TModel3d;

namespace scene::eu7 {

enum class PackMeshLoadWait {
    NonBlocking,
    BlockUntilReady,
};

void
start_pack_mesh_loader();

void
stop_pack_mesh_loader();

void
reset_pack_mesh_loader();

void
request_pack_mesh_load( std::string const &model_file, int priority = 0 );

void
request_pack_mesh_load_paths(
    std::vector<std::string> const &model_files,
    std::size_t max_enqueue = 0,
    int priority = 0 );

[[nodiscard]] TModel3d *
ensure_pack_mesh_in_session_cache(
    std::string model_file,
    std::unordered_map<std::string, TModel3d *> &session_cache,
    PackMeshLoadWait wait = PackMeshLoadWait::NonBlocking,
    double block_budget_ms = 0.0 );

[[nodiscard]] bool
try_adopt_pack_mesh_for_slice(
    std::string model_file,
    std::unordered_map<std::string, TModel3d *> &session_cache );

[[nodiscard]] std::size_t
drain_pack_mesh_loader_ready(
    std::unordered_map<std::string, TModel3d *> &session_cache,
    std::size_t max_drain = 0 );

[[nodiscard]] std::size_t
pack_mesh_loader_queue_depth();

[[nodiscard]] std::size_t
pack_mesh_loader_ready_count();

[[nodiscard]] std::size_t
pack_mesh_loader_worker_count();

[[nodiscard]] std::size_t
pump_pack_mesh_loader(
    double budget_ms = 0.0,
    std::size_t max_loads = 1 );

} // namespace scene::eu7
