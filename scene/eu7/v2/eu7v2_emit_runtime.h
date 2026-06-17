/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// ---------------------------------------------------------------------------
// RuntimeScene -> eu7v2 emitter. Lives in the eu07_bake target because it
// touches the parser's RuntimeModule types directly. It maps Runtime* records
// into the same dependency-free eu7v2 structs that scene_baker uses (one
// writer/reader per record), so the produced .eu7v2 matches what the engine
// loads via eu7v2::load_module. For the scenario root the flattened PACK model
// batches are deduplicated into prototypes + instances.
// ---------------------------------------------------------------------------

#include <eu07/scene/bake/module.hpp>
#include <eu07/scene/binary/runtime_codec.hpp>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace eu07::scene::bake {
class PackModelSpoolFile;
class ShapeSpoolFile;
}

namespace eu7v2 {

struct emit_outcome {
    bool ok { true };
    bool verified { false };
    bool verify_ok { true };
    std::string written_path;
    std::string message;
    std::size_t model_total { 0 };
    std::size_t byte_size { 0 };
    double build_ms { 0.0 };
    double write_ms { 0.0 };
    double verify_ms { 0.0 };
};

struct module_verify_spec {
    std::size_t includes { 0 };
    std::size_t placements { 0 };
    std::size_t models { 0 };
    std::size_t shapes { 0 };
    std::size_t lines { 0 };
    std::size_t tracks { 0 };
    std::size_t traction { 0 };
    std::size_t power { 0 };
    std::size_t memcells { 0 };
    std::size_t launchers { 0 };
    std::size_t events { 0 };
    std::size_t sounds { 0 };
    std::size_t dynamics { 0 };
    std::size_t trainsets { 0 };
};

// Builds the eu7v2 byte image for one baked module.
[[nodiscard]] std::vector<std::uint8_t>
emit_runtime_module_bytes(
    eu07::scene::bake::RuntimeModule const &module,
    bool is_root,
    std::vector<eu07::scene::binary::codec::ModelSectionBatch> const *pack_batches,
    eu07::scene::bake::PackModelSpoolFile const *pack_spool = nullptr,
    eu07::scene::bake::ShapeSpoolFile const *shape_spool = nullptr,
    std::filesystem::path const &text_path = {} );

// Emits the module, writes "<stem>.eu7v2" next to the text source, and (when
// verify is true) reloads the bytes and compares record counts against the
// source RuntimeModule, filling outcome.message with a PASS/FAIL report.
[[nodiscard]] emit_outcome
emit_runtime_module(
    eu07::scene::bake::RuntimeModule const &module,
    std::filesystem::path const &text_path,
    bool is_root,
    std::vector<eu07::scene::binary::codec::ModelSectionBatch> const *pack_batches,
    bool verify,
    eu07::scene::bake::PackModelSpoolFile const *pack_spool = nullptr,
    eu07::scene::bake::ShapeSpoolFile const *shape_spool = nullptr );

// Porownanie zapisanej eu7v2 z oczekiwanymi licznikami (do odroczonego verify).
[[nodiscard]] bool
verify_written_module(
    std::filesystem::path const &path,
    module_verify_spec const &spec,
    bool const is_root,
    std::size_t const pack_models,
    std::string *message_out = nullptr );

} // namespace eu7v2
