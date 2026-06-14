#pragma once

// EU7B runtime (version=5) — 1 plik tekstowy → 1 .eu7.
// Chunki: STRS, INCL, TRAK, TRAC, PWRS, TERR, MESH, LINE, MODL, MEMC, LAUN, DYNM, SOND, TRSET, EVNT, FINT.

#include <eu07/scene/bake/geometry.hpp>
#include <eu07/scene/bake/module.hpp>
#include <eu07/scene/binary/common.hpp>
#include <eu07/scene/binary/io.hpp>
#include <eu07/scene/binary/runtime_codec.hpp>
#include <eu07/scene/binary/string_table.hpp>
#include <eu07/scene/runtime/nodes.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <cstdint>
#include <limits>
#include <filesystem>
#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <vector>

namespace eu07::scene::binary {

inline constexpr std::array<char, 4> kChunkStrs{'S', 'T', 'R', 'S'};
inline constexpr std::array<char, 4> kChunkIncl{'I', 'N', 'C', 'L'};
inline constexpr std::array<char, 4> kChunkTrak{'T', 'R', 'A', 'K'};
inline constexpr std::array<char, 4> kChunkMesh{'M', 'E', 'S', 'H'};
inline constexpr std::array<char, 4> kChunkTerr{'T', 'E', 'R', 'R'};
inline constexpr std::array<char, 4> kChunkLine{'L', 'I', 'N', 'E'};
inline constexpr std::array<char, 4> kChunkModl{'M', 'O', 'D', 'L'};
inline constexpr std::array<char, 4> kChunkTrac{'T', 'R', 'A', 'C'};
inline constexpr std::array<char, 4> kChunkPwrs{'P', 'W', 'R', 'S'};
inline constexpr std::array<char, 4> kChunkMemc{'M', 'E', 'M', 'C'};
inline constexpr std::array<char, 4> kChunkLaun{'L', 'A', 'U', 'N'};
inline constexpr std::array<char, 4> kChunkDynm{'D', 'Y', 'N', 'M'};
inline constexpr std::array<char, 4> kChunkSond{'S', 'O', 'N', 'D'};
inline constexpr std::array<char, 4> kChunkTrset{'T', 'R', 'S', 'E'};
inline constexpr std::array<char, 4> kChunkEvnt{'E', 'V', 'N', 'T'};
inline constexpr std::array<char, 4> kChunkFint{'F', 'I', 'N', 'T'};
inline constexpr std::array<char, 4> kChunkPlac{'P', 'L', 'A', 'C'};
inline constexpr std::array<char, 4> kChunkPidx{'P', 'I', 'D', 'X'};
inline constexpr std::array<char, 4> kChunkPack{'P', 'A', 'C', 'K'};

// PACK section payload: v9 = UMES, v10 = UMES + CHNK (sub-chunk offset table).
inline constexpr std::uint8_t kPackSectionFormatV9 = 2;
inline constexpr std::uint8_t kPackSectionFormatV10 = 3;
inline constexpr std::size_t kPackSectionChunkModels = 512;

struct PackWriteStats {
    std::size_t strings_total = 0;
    std::size_t pack_entries = 0;
    std::size_t pack_bytes = 0;
    std::size_t pack_build_ms = 0;
    std::size_t write_file_ms = 0;
    unsigned pack_workers = 0;
};

inline void printPackWriteStats(const PackWriteStats& stats, std::ostream& out) {
    out << "[EU7]   PACK zapis:\n"
        << "    strs=" << stats.strings_total << " pidx=" << stats.pack_entries
        << " pack_B=" << stats.pack_bytes << '\n'
        << "    czas_ms: build=" << stats.pack_build_ms << " zapis_pliku=" << stats.write_file_ms
        << " watki_build=" << stats.pack_workers << '\n'
        << std::flush;
}

struct WriteRuntimeModuleOptions {
    // Root scenariusz: MODL → PIDX+PACK (EU7B v7) zamiast plaskiego MODL.
    bool emitPackModels = false;
    // Sekcje 1 km z compose — serializacja PACK po zbudowaniu STRS.
    const std::vector<codec::ModelSectionBatch>* pack_batches = nullptr;
    PackWriteStats* pack_write_stats = nullptr;
};

struct LoadedRuntimeModule {
    bake::RuntimeModule module;
    std::uint32_t version = 0;
    StringTable strings;
    struct ChunkSummary {
        std::array<char, 4> id{};
        std::uint32_t size = 0;
        bool recognized = false;
    };
    std::vector<ChunkSummary> chunks;
};

namespace detail {

inline void writeSegmentPath(std::ostream& out, const runtime::SegmentPath& seg) {
    io::writeVec3(out, seg.pStart);
    io::writeF64(out, seg.rollStart);
    io::writeVec3(out, seg.cpOut);
    io::writeVec3(out, seg.cpIn);
    io::writeVec3(out, seg.pEnd);
    io::writeF64(out, seg.rollEnd);
    io::writeF64(out, seg.radius);
}

inline void writeTrackVisibility(
    std::ostream& out,
    StringTable& table,
    const runtime::TrackVisibility& vis) {
    codec::writeStringId(out, table, vis.material1);
    io::writeF32(out, vis.texLength);
    codec::writeStringId(out, table, vis.material2);
    io::writeF32(out, vis.texHeight1);
    io::writeF32(out, vis.texWidth);
    io::writeF32(out, vis.texSlope);
}

inline void writeRuntimeTrack(std::ostream& out, StringTable& table, const runtime::RuntimeTrack& track) {
    codec::writeSlimNode(out, table, track.node, "track");
    io::writeU8(out, static_cast<std::uint8_t>(track.trackType));
    io::writeU8(out, static_cast<std::uint8_t>(track.category));
    io::writeF32(out, track.length);
    io::writeF32(out, track.trackWidth);
    io::writeF32(out, track.friction);
    io::writeF32(out, track.soundDistance);
    io::writeI32(out, track.qualityFlag);
    io::writeI32(out, track.damageFlag);
    io::writeU8(out, static_cast<std::uint8_t>(static_cast<int>(track.environment) + 1));
    io::writeU8(out, track.visibility.has_value() ? 1 : 0);
    if (track.visibility) {
        writeTrackVisibility(out, table, *track.visibility);
    }
    io::writeU32(out, static_cast<std::uint32_t>(track.paths.size()));
    for (const runtime::SegmentPath& seg : track.paths) {
        writeSegmentPath(out, seg);
    }
    io::writeU32(out, static_cast<std::uint32_t>(track.tailKeywords.size()));
    for (const auto& [key, value] : track.tailKeywords) {
        codec::writeTrackTailEntry(out, table, key, value);
    }
}

inline void writeRuntimeShape(std::ostream& out, StringTable& table, const runtime::RuntimeShapeNode& shape) {
    const std::uint8_t subtype = codec::meshSubtypeCode(
        shape.node.nodeType.empty() ? std::string_view{"triangles"} : shape.node.nodeType);
    io::writeU8(out, subtype);
    codec::writeSlimNode(out, table, shape.node, codec::meshSubtypeName(subtype));
    io::writeU8(out, shape.translucent ? 1 : 0);
    codec::writeStringId(out, table, shape.materialPath);
    codec::writeLightingTagged(out, shape.lighting);
    io::writeVec3(out, shape.origin);
    io::writeU32(out, static_cast<std::uint32_t>(shape.vertices.size()));
    for (const runtime::WorldVertex& vertex : shape.vertices) {
        codec::writePackedWorldVertex(out, vertex);
    }
}

inline void writeRuntimeLines(std::ostream& out, StringTable& table, const runtime::RuntimeLinesNode& lines) {
    const std::uint8_t subtype = codec::lineSubtypeCode(
        lines.node.nodeType.empty() ? std::string_view{"lines"} : lines.node.nodeType);
    io::writeU8(out, subtype);
    codec::writeSlimNode(out, table, lines.node, codec::lineSubtypeName(subtype));
    codec::writeLightingTagged(out, lines.lighting);
    io::writeF32(out, lines.lineWidth);
    io::writeVec3(out, lines.origin);
    io::writeU32(out, static_cast<std::uint32_t>(lines.vertices.size()));
    for (const runtime::WorldVertex& vertex : lines.vertices) {
        io::writeVec3(out, vertex.position);
    }
}

inline void writeRuntimeModel(std::ostream& out, StringTable& table, const runtime::RuntimeModelInstance& model) {
    codec::writeSlimNode(out, table, model.node, "model");
    io::writeU8(out, model.isTerrain ? 1 : 0);
    io::writeU8(out, model.transition ? 1 : 0);
    io::writeVec3(out, model.location);
    io::writeVec3(out, model.angles);
    io::writeVec3(out, model.scale);
    codec::writeStringId(out, table, model.modelFile);
    codec::writeStringId(out, table, model.textureFile);
    io::writeU32(out, static_cast<std::uint32_t>(model.lightStates.size()));
    for (float light : model.lightStates) {
        io::writeF32(out, light);
    }
    io::writeU32(out, static_cast<std::uint32_t>(model.lightColors.size()));
    for (std::uint32_t color : model.lightColors) {
        io::writeU32(out, color);
    }
}

inline void collectShapeStrings(StringTable& table, const runtime::RuntimeShapeNode& shape) {
    table.intern(shape.node.name);
    table.intern(shape.materialPath);
}

inline void collectLinesStrings(StringTable& table, const runtime::RuntimeLinesNode& lines) {
    table.intern(lines.node.name);
}

inline void collectModelStrings(StringTable& table, const runtime::RuntimeModelInstance& model) {
    table.intern(model.node.name);
    table.intern(model.modelFile);
    table.intern(model.textureFile);
}

inline void collectTrackStrings(StringTable& table, const runtime::RuntimeTrack& track) {
    table.intern(track.node.name);
    if (track.visibility) {
        table.intern(track.visibility->material1);
        table.intern(track.visibility->material2);
    }
    for (const auto& [key, value] : track.tailKeywords) {
        if (codec::trackTailKeywordCode(key) == 255) {
            table.intern(key);
        }
        table.intern(value);
    }
}

#include <eu07/scene/binary/runtime_chunks_sim.hpp>
#include <eu07/scene/binary/runtime_chunks_directives.hpp>

inline void collectModuleStrings(StringTable& table, const bake::RuntimeModule& module) {
    for (const bake::ModuleInclude& inc : module.includes) {
        table.intern(inc.sourcePath);
        table.intern(inc.binaryPath);
        for (const std::string& param : inc.parameters) {
            table.intern(param);
        }
    }
    for (const runtime::RuntimeTrack& track : module.scene.tracks) {
        collectTrackStrings(table, track);
    }
    for (const runtime::RuntimeShapeNode& shape : module.scene.shapes) {
        collectShapeStrings(table, shape);
    }
    for (const runtime::RuntimeLinesNode& lines : module.scene.lines) {
        collectLinesStrings(table, lines);
    }
    for (const runtime::RuntimeModelInstance& model : module.scene.models) {
        collectModelStrings(table, model);
    }
    for (const runtime::RuntimeTraction& traction : module.scene.traction) {
        collectTractionStrings(table, traction);
    }
    for (const runtime::RuntimeTractionPowerSource& source : module.scene.powerSources) {
        collectPowerSourceStrings(table, source);
    }
    for (const runtime::RuntimeMemCell& cell : module.scene.memcells) {
        collectMemcellStrings(table, cell);
    }
    for (const runtime::RuntimeEventLauncher& launcher : module.scene.eventLaunchers) {
        collectLauncherStrings(table, launcher);
    }
    for (const runtime::RuntimeDynamicObject& vehicle : module.scene.dynamics) {
        collectDynamicStrings(table, vehicle);
    }
    for (const runtime::RuntimeSoundSource& sound : module.scene.sounds) {
        collectSoundStrings(table, sound);
    }
    for (const runtime::RuntimeTrainset& trainset : module.scene.trainsets) {
        collectTrainsetStrings(table, trainset);
    }
    for (const runtime::RuntimeEvent& event : module.scene.events) {
        collectEventStrings(table, event);
    }
}

[[nodiscard]] inline std::vector<char> buildStrsPayload(const StringTable& table) {
    std::ostringstream out(std::ios::binary);
    io::writeU32(out, static_cast<std::uint32_t>(table.strings().size()));
    for (const std::string& text : table.strings()) {
        io::writeU32(out, static_cast<std::uint32_t>(text.size()));
        out.write(text.data(), static_cast<std::streamsize>(text.size()));
    }
    const std::string blob = out.str();
    return {blob.begin(), blob.end()};
}

[[nodiscard]] inline std::vector<char> buildInclPayloadV2(
    StringTable& table,
    const std::vector<bake::ModuleInclude>& includes) {
    std::ostringstream out(std::ios::binary);
    io::writeU32(out, static_cast<std::uint32_t>(includes.size()));
    for (const bake::ModuleInclude& inc : includes) {
        io::writeU32(out, inc.sourceLine);
        codec::writeStringId(out, table, inc.sourcePath);
        codec::writeStringId(out, table, inc.binaryPath);
        io::writeU32(out, static_cast<std::uint32_t>(inc.parameters.size()));
        for (const std::string& param : inc.parameters) {
            codec::writeStringId(out, table, param);
        }
        codec::detail::writeTransformContext(out, inc.siteTransform);
    }
    const std::string blob = out.str();
    return {blob.begin(), blob.end()};
}

[[nodiscard]] inline std::vector<char> buildTrakPayloadV2(
    StringTable& table,
    const std::vector<runtime::RuntimeTrack>& tracks) {
    std::ostringstream out(std::ios::binary);
    io::writeU32(out, static_cast<std::uint32_t>(tracks.size()));
    for (const runtime::RuntimeTrack& track : tracks) {
        writeRuntimeTrack(out, table, track);
    }
    const std::string blob = out.str();
    return {blob.begin(), blob.end()};
}

[[nodiscard]] inline std::vector<char> buildMeshPayloadV2(
    StringTable& table,
    const std::vector<runtime::RuntimeShapeNode>& shapes) {
    std::ostringstream out(std::ios::binary);
    io::writeU32(out, static_cast<std::uint32_t>(shapes.size()));
    for (const runtime::RuntimeShapeNode& shape : shapes) {
        writeRuntimeShape(out, table, shape);
    }
    const std::string blob = out.str();
    return {blob.begin(), blob.end()};
}

[[nodiscard]] inline std::vector<char> buildTerrPayloadV2(
    StringTable& table,
    const std::vector<runtime::RuntimeShapeNode>& shapes) {
    const std::uint8_t flags = codec::terrFlagsForShape(shapes.front());
    std::ostringstream out(std::ios::binary);
    io::writeU8(out, flags);
    codec::writeStringId(out, table, shapes.front().materialPath);
    if ((flags & codec::kTerrFlagNonDefaultLighting) != 0) {
        codec::writeLightingBlock(out, shapes.front().lighting);
    }
    io::writeU32(out, static_cast<std::uint32_t>(shapes.size()));
    for (const runtime::RuntimeShapeNode& shape : shapes) {
        if (shape.vertices.size() != codec::kTerrVertsPerRecord) {
            throw std::runtime_error("EU7B TERR: oczekiwano 3 wierzcholkow na rekord");
        }
        for (const runtime::WorldVertex& vertex : shape.vertices) {
            codec::writePackedWorldVertex(out, vertex);
        }
    }
    const std::string blob = out.str();
    return {blob.begin(), blob.end()};
}

[[nodiscard]] inline std::vector<char> buildTerrBatchedPayloadV2(
    StringTable& table,
    const std::vector<runtime::RuntimeShapeNode>& shapes) {
    const std::vector<codec::TerrSectionBatch> batches = codec::groupTerrShapesBySection(shapes);
    const std::uint8_t flags = codec::terrFlagsForShape(shapes.front(), true);
    std::ostringstream out(std::ios::binary);
    io::writeU8(out, flags);
    codec::writeStringId(out, table, shapes.front().materialPath);
    if ((flags & codec::kTerrFlagNonDefaultLighting) != 0) {
        codec::writeLightingBlock(out, shapes.front().lighting);
    }
    io::writeU32(out, static_cast<std::uint32_t>(batches.size()));
    for (const codec::TerrSectionBatch& batch : batches) {
        io::writeI32(out, batch.section.x);
        io::writeI32(out, batch.section.z);
        io::writeU32(out, static_cast<std::uint32_t>(batch.vertices.size()));
        for (const runtime::WorldVertex& vertex : batch.vertices) {
            codec::writePackedWorldVertex(out, vertex);
        }
    }
    const std::string blob = out.str();
    return {blob.begin(), blob.end()};
}

[[nodiscard]] inline std::vector<char> buildLinePayloadV2(
    StringTable& table,
    const std::vector<runtime::RuntimeLinesNode>& lines) {
    std::ostringstream out(std::ios::binary);
    io::writeU32(out, static_cast<std::uint32_t>(lines.size()));
    for (const runtime::RuntimeLinesNode& line : lines) {
        writeRuntimeLines(out, table, line);
    }
    const std::string blob = out.str();
    return {blob.begin(), blob.end()};
}

[[nodiscard]] inline std::vector<char> buildModlPayloadV2(
    StringTable& table,
    const std::vector<runtime::RuntimeModelInstance>& models) {
    std::ostringstream out(std::ios::binary);
    io::writeU32(out, static_cast<std::uint32_t>(models.size()));
    for (const runtime::RuntimeModelInstance& model : models) {
        writeRuntimeModel(out, table, model);
    }
    const std::string blob = out.str();
    return {blob.begin(), blob.end()};
}

struct PackSectionIndexEntry {
    std::uint16_t row = 0;
    std::uint16_t column = 0;
    std::uint32_t modelCount = 0;
    std::uint64_t packOffset = 0;
};

struct PackPayloadBuild {
    std::vector<char> packPayload;
    std::vector<PackSectionIndexEntry> entries;
};

[[nodiscard]] inline bool isLoadableMeshPath(const std::string_view path) {
    return !path.empty() && path != "notload";
}

[[nodiscard]] inline std::vector<std::uint32_t> collectUniqueMeshIds(
    StringTable& table,
    const std::vector<runtime::RuntimeModelInstance>& models,
    const std::vector<std::string_view>* prototype_mesh_paths = nullptr) {
    std::vector<std::string> paths;
    paths.reserve(models.size());

    for (const runtime::RuntimeModelInstance& model : models) {
        if (isLoadableMeshPath(model.modelFile)) {
            paths.emplace_back(model.modelFile);
        }
    }
    if (prototype_mesh_paths != nullptr) {
        for (const std::string_view path : *prototype_mesh_paths) {
            if (isLoadableMeshPath(path)) {
                paths.emplace_back(path);
            }
        }
    }

    std::stable_sort(paths.begin(), paths.end());
    paths.erase(std::unique(paths.begin(), paths.end()), paths.end());

    std::vector<std::uint32_t> ids;
    ids.reserve(paths.size());
    for (const std::string& path : paths) {
        ids.push_back(table.intern(path));
    }
    return ids;
}

[[nodiscard]] inline std::vector<char> buildPackModelBlob(
    StringTable& table,
    const std::vector<runtime::RuntimeModelInstance>& models,
    const std::size_t begin,
    const std::size_t end) {
    std::ostringstream out(std::ios::binary);
    for (std::size_t index = begin; index < end; ++index) {
        runtime::RuntimeModelInstance worldModel = models[index];
        worldModel.node.transform = {};
        writeRuntimeModel(out, table, worldModel);
    }
    const std::string blob = out.str();
    return {blob.begin(), blob.end()};
}

[[nodiscard]] inline std::vector<char> buildPackSectionPayloadV10(
    StringTable& table,
    const std::vector<runtime::RuntimeModelInstance>& models,
    const std::uint32_t inst_count = 0,
    const std::vector<std::string_view>* prototype_mesh_paths = nullptr) {
    const auto mesh_ids = collectUniqueMeshIds(table, models, prototype_mesh_paths);
    const std::uint32_t solo_count = static_cast<std::uint32_t>(models.size());

    std::vector<std::vector<char>> chunk_payloads;
    chunk_payloads.reserve((models.size() + kPackSectionChunkModels - 1) / kPackSectionChunkModels);
    for (std::size_t offset = 0; offset < models.size(); offset += kPackSectionChunkModels) {
        const std::size_t end =
            std::min(offset + kPackSectionChunkModels, models.size());
        chunk_payloads.push_back(buildPackModelBlob(table, models, offset, end));
    }
    if (chunk_payloads.empty()) {
        chunk_payloads.emplace_back();
    }

    const std::uint32_t chunk_count = static_cast<std::uint32_t>(chunk_payloads.size());
    const std::uint32_t header_size =
        1u + 4u + 4u + 4u + static_cast<std::uint32_t>(mesh_ids.size()) * 4u + 4u +
        chunk_count * 8u;

    std::vector<std::uint32_t> chunk_offsets(chunk_count);
    std::vector<std::uint32_t> chunk_model_counts(chunk_count);
    std::uint32_t payload_offset = header_size;
    for (std::uint32_t chunk_index = 0; chunk_index < chunk_count; ++chunk_index) {
        chunk_offsets[chunk_index] = payload_offset;
        const std::size_t model_begin =
            static_cast<std::size_t>(chunk_index) * kPackSectionChunkModels;
        const std::size_t model_end =
            std::min(model_begin + kPackSectionChunkModels, models.size());
        chunk_model_counts[chunk_index] =
            static_cast<std::uint32_t>(model_end > model_begin ? model_end - model_begin : 0);
        payload_offset += static_cast<std::uint32_t>(chunk_payloads[chunk_index].size());
    }

    std::ostringstream packOut(std::ios::binary);
    io::writeU8(packOut, kPackSectionFormatV10);
    io::writeU32(packOut, solo_count);
    io::writeU32(packOut, inst_count);
    io::writeU32(packOut, static_cast<std::uint32_t>(mesh_ids.size()));
    for (const std::uint32_t id : mesh_ids) {
        io::writeU32(packOut, id);
    }
    io::writeU32(packOut, chunk_count);
    for (std::uint32_t chunk_index = 0; chunk_index < chunk_count; ++chunk_index) {
        io::writeU32(packOut, chunk_model_counts[chunk_index]);
        io::writeU32(packOut, chunk_offsets[chunk_index]);
    }
    for (const std::vector<char>& payload : chunk_payloads) {
        packOut.write(payload.data(), static_cast<std::streamsize>(payload.size()));
    }
    const std::string blob = packOut.str();
    return {blob.begin(), blob.end()};
}

[[nodiscard]] inline PackPayloadBuild buildPackPayloadV7(
    StringTable& table,
    const std::vector<codec::ModelSectionBatch>& batches) {
    PackPayloadBuild result;
    if (batches.empty()) {
        return result;
    }

    for (const codec::ModelSectionBatch& batch : batches) {
        for (const runtime::RuntimeModelInstance& model : batch.models) {
            collectModelStrings(table, model);
        }
    }

    result.entries.resize(batches.size());
    std::vector<std::vector<char>> section_payloads(batches.size());

    const unsigned hw =
        std::thread::hardware_concurrency() == 0 ? 4u : std::thread::hardware_concurrency();
    const unsigned worker_count = std::max(1u, std::min<unsigned>(hw, static_cast<unsigned>(batches.size())));

    if (worker_count == 1 || batches.size() < 8) {
        for (std::size_t batch_index = 0; batch_index < batches.size(); ++batch_index) {
            const codec::ModelSectionBatch& batch = batches[batch_index];
            PackSectionIndexEntry& entry = result.entries[batch_index];
            entry.row = static_cast<std::uint16_t>(batch.section.z);
            entry.column = static_cast<std::uint16_t>(batch.section.x);
            entry.modelCount = static_cast<std::uint32_t>(batch.models.size());
            section_payloads[batch_index] = buildPackSectionPayloadV10(table, batch.models);
        }
        std::size_t total_size = 0;
        for (const std::vector<char>& payload : section_payloads) {
            total_size += payload.size();
        }
        result.packPayload.clear();
        result.packPayload.reserve(total_size);
        std::uint64_t offset = 0;
        for (std::size_t batch_index = 0; batch_index < batches.size(); ++batch_index) {
            result.entries[batch_index].packOffset = offset;
            const std::vector<char>& payload = section_payloads[batch_index];
            result.packPayload.insert(result.packPayload.end(), payload.begin(), payload.end());
            offset += static_cast<std::uint64_t>(payload.size());
        }
        return result;
    }

    std::atomic<std::size_t> next_batch { 0 };
    std::vector<std::thread> workers;
    workers.reserve(worker_count);
    for (unsigned worker = 0; worker < worker_count; ++worker) {
        workers.emplace_back([&]() {
            while (true) {
                const std::size_t batch_index = next_batch.fetch_add(1, std::memory_order_relaxed);
                if (batch_index >= batches.size()) {
                    return;
                }
                const codec::ModelSectionBatch& batch = batches[batch_index];
                PackSectionIndexEntry& entry = result.entries[batch_index];
                entry.row = static_cast<std::uint16_t>(batch.section.z);
                entry.column = static_cast<std::uint16_t>(batch.section.x);
                entry.modelCount = static_cast<std::uint32_t>(batch.models.size());

                section_payloads[batch_index] = buildPackSectionPayloadV10(table, batch.models);
            }
        });
    }
    for (std::thread& worker : workers) {
        if (worker.joinable()) {
            worker.join();
        }
    }

    std::size_t total_size = 0;
    for (const std::vector<char>& payload : section_payloads) {
        total_size += payload.size();
    }
    result.packPayload.clear();
    result.packPayload.reserve(total_size);
    std::uint64_t offset = 0;
    for (std::size_t batch_index = 0; batch_index < batches.size(); ++batch_index) {
        result.entries[batch_index].packOffset = offset;
        const std::vector<char>& payload = section_payloads[batch_index];
        result.packPayload.insert(result.packPayload.end(), payload.begin(), payload.end());
        offset += static_cast<std::uint64_t>(payload.size());
    }
    return result;
}

[[nodiscard]] inline PackPayloadBuild buildPackPayloadV7(
    StringTable& table,
    const std::vector<runtime::RuntimeModelInstance>& models) {
    return buildPackPayloadV7(table, codec::groupModelsBySection(models));
}

} // namespace detail

namespace detail {

[[nodiscard]] inline std::vector<char> buildPidxPayloadV7(
    const std::vector<PackSectionIndexEntry>& entries) {
    std::ostringstream out(std::ios::binary);
    io::writeU32(out, static_cast<std::uint32_t>(entries.size()));
    for (const PackSectionIndexEntry& entry : entries) {
        io::writeU16(out, entry.row);
        io::writeU16(out, entry.column);
        io::writeU32(out, entry.modelCount);
        io::writeU64(out, entry.packOffset);
    }
    const std::string blob = out.str();
    return {blob.begin(), blob.end()};
}

inline runtime::SegmentPath readSegmentPath(std::istream& in) {
    runtime::SegmentPath seg;
    seg.pStart = io::readVec3(in);
    seg.rollStart = io::readF64(in);
    seg.cpOut = io::readVec3(in);
    seg.cpIn = io::readVec3(in);
    seg.pEnd = io::readVec3(in);
    seg.rollEnd = io::readF64(in);
    seg.radius = io::readF64(in);
    return seg;
}

inline runtime::TrackVisibility readTrackVisibility(std::istream& in, const StringTable& table) {
    runtime::TrackVisibility vis;
    vis.material1 = table.resolve(io::readU32(in));
    vis.texLength = io::readF32(in);
    vis.material2 = table.resolve(io::readU32(in));
    vis.texHeight1 = io::readF32(in);
    vis.texWidth = io::readF32(in);
    vis.texSlope = io::readF32(in);
    return vis;
}

inline runtime::RuntimeTrack readRuntimeTrack(std::istream& in, const StringTable& table) {
    runtime::RuntimeTrack track;
    track.node = codec::readSlimNode(in, table, "track");
    track.trackType = static_cast<runtime::TrackType>(io::readU8(in));
    track.category = static_cast<runtime::TrackCategory>(io::readU8(in));
    track.length = io::readF32(in);
    track.trackWidth = io::readF32(in);
    track.friction = io::readF32(in);
    track.soundDistance = io::readF32(in);
    track.qualityFlag = io::readI32(in);
    track.damageFlag = io::readI32(in);
    track.environment = static_cast<runtime::TrackEnvironment>(static_cast<int>(io::readU8(in)) - 1);
    if (io::readU8(in) != 0) {
        track.visibility = readTrackVisibility(in, table);
    }
    const std::uint32_t pathCount = io::readU32(in);
    track.paths.reserve(pathCount);
    for (std::uint32_t i = 0; i < pathCount; ++i) {
        track.paths.push_back(readSegmentPath(in));
    }
    const std::uint32_t tailCount = io::readU32(in);
    track.tailKeywords.reserve(tailCount);
    for (std::uint32_t i = 0; i < tailCount; ++i) {
        track.tailKeywords.push_back(codec::readTrackTailEntry(in, table));
    }
    return track;
}

inline void readStrsChunk(std::istream& in, StringTable& table) {
    const std::uint32_t count = io::readU32(in);
    std::vector<std::string> strings;
    strings.reserve(count);
    for (std::uint32_t i = 0; i < count; ++i) {
        strings.push_back(io::readString(in));
    }
    table.load(std::move(strings));
}

inline void readInclChunkV2(
    std::istream& in,
    const StringTable& table,
    const std::uint32_t fileVersion,
    bake::RuntimeModule& module) {
    const std::uint32_t count = io::readU32(in);
    module.includes.reserve(count);
    for (std::uint32_t i = 0; i < count; ++i) {
        bake::ModuleInclude inc;
        inc.sourceLine = io::readU32(in);
        inc.sourcePath = table.resolve(io::readU32(in));
        inc.binaryPath = table.resolve(io::readU32(in));
        const std::uint32_t paramCount = io::readU32(in);
        inc.parameters.reserve(paramCount);
        for (std::uint32_t p = 0; p < paramCount; ++p) {
            inc.parameters.push_back(table.resolve(io::readU32(in)));
        }
        if (fileVersion >= kVersionRuntime) {
            inc.siteTransform = codec::detail::readTransformContext(in);
        }
        module.includes.push_back(std::move(inc));
    }
}

inline void readTrakChunkV2(std::istream& in, const StringTable& table, bake::RuntimeModule& module) {
    const std::uint32_t count = io::readU32(in);
    module.scene.tracks.reserve(count);
    for (std::uint32_t i = 0; i < count; ++i) {
        module.scene.tracks.push_back(readRuntimeTrack(in, table));
    }
}

inline runtime::RuntimeShapeNode readRuntimeShape(std::istream& in, const StringTable& table) {
    runtime::RuntimeShapeNode shape;
    const std::uint8_t subtype = io::readU8(in);
    shape.node = codec::readSlimNode(in, table, codec::meshSubtypeName(subtype));
    shape.translucent = io::readU8(in) != 0;
    shape.materialPath = table.resolve(io::readU32(in));
    shape.lighting = codec::readLightingTagged(in);
    shape.origin = io::readVec3(in);
    const std::uint32_t vertexCount = io::readU32(in);
    shape.vertices.reserve(vertexCount);
    for (std::uint32_t i = 0; i < vertexCount; ++i) {
        shape.vertices.push_back(codec::readPackedWorldVertex(in));
    }
    return shape;
}

inline runtime::RuntimeLinesNode readRuntimeLines(std::istream& in, const StringTable& table) {
    runtime::RuntimeLinesNode lines;
    const std::uint8_t subtype = io::readU8(in);
    lines.node = codec::readSlimNode(in, table, codec::lineSubtypeName(subtype));
    lines.lighting = codec::readLightingTagged(in);
    lines.lineWidth = io::readF32(in);
    lines.origin = io::readVec3(in);
    const std::uint32_t vertexCount = io::readU32(in);
    lines.vertices.reserve(vertexCount);
    for (std::uint32_t i = 0; i < vertexCount; ++i) {
        runtime::WorldVertex vertex;
        vertex.position = io::readVec3(in);
        lines.vertices.push_back(vertex);
    }
    return lines;
}

inline runtime::RuntimeModelInstance readRuntimeModel(std::istream& in, const StringTable& table) {
    runtime::RuntimeModelInstance model;
    model.node = codec::readSlimNode(in, table, "model");
    model.isTerrain = io::readU8(in) != 0;
    model.transition = io::readU8(in) != 0;
    model.location = io::readVec3(in);
    model.angles = io::readVec3(in);
    model.scale = io::readVec3(in);
    model.modelFile = table.resolve(io::readU32(in));
    model.textureFile = table.resolve(io::readU32(in));
    const std::uint32_t lightCount = io::readU32(in);
    model.lightStates.resize(lightCount);
    for (std::uint32_t i = 0; i < lightCount; ++i) {
        model.lightStates[i] = io::readF32(in);
    }
    const std::uint32_t colorCount = io::readU32(in);
    model.lightColors.resize(colorCount);
    for (std::uint32_t i = 0; i < colorCount; ++i) {
        model.lightColors[i] = io::readU32(in);
    }
    return model;
}

inline void readMeshChunkV2(std::istream& in, const StringTable& table, bake::RuntimeModule& module) {
    const std::uint32_t count = io::readU32(in);
    module.scene.shapes.reserve(module.scene.shapes.size() + count);
    for (std::uint32_t i = 0; i < count; ++i) {
        module.scene.shapes.push_back(readRuntimeShape(in, table));
    }
}

inline runtime::RuntimeShapeNode makeTerrShape(
    const std::string& material,
    const std::uint8_t flags,
    const runtime::LightingData& lighting) {
    runtime::RuntimeShapeNode shape;
    shape.node.nodeType = "triangles";
    shape.node.rangeSquaredMin = 0.0;
    shape.node.rangeSquaredMax = std::numeric_limits<double>::max();
    shape.node.visible = true;
    shape.materialPath = material;
    shape.translucent = (flags & codec::kTerrFlagTranslucent) != 0;
    shape.lighting = lighting;
    shape.origin = {};
    return shape;
}

inline void readTerrChunkV2(std::istream& in, const StringTable& table, bake::RuntimeModule& module) {
    const std::uint8_t flags = io::readU8(in);
    const std::string material = table.resolve(io::readU32(in));
    runtime::LightingData lighting;
    if ((flags & codec::kTerrFlagNonDefaultLighting) != 0) {
        lighting = codec::readLightingBlock(in);
    }

    if ((flags & codec::kTerrFlagBatched) != 0) {
        const std::uint32_t batchCount = io::readU32(in);
        module.scene.shapes.reserve(module.scene.shapes.size() + batchCount);
        for (std::uint32_t i = 0; i < batchCount; ++i) {
            (void)io::readI32(in);
            (void)io::readI32(in);
            runtime::RuntimeShapeNode shape = makeTerrShape(material, flags, lighting);
            const std::uint32_t vertexCount = io::readU32(in);
            if (vertexCount % codec::kTerrVertsPerRecord != 0) {
                throw std::runtime_error("EU7B TERR: liczba wierzcholkow w batchu musi byc wielokrotnoscia 3");
            }
            shape.vertices.resize(vertexCount);
            for (std::uint32_t v = 0; v < vertexCount; ++v) {
                shape.vertices[v] = codec::readPackedWorldVertex(in);
            }
            bake::computeBounds(shape);
            module.scene.shapes.push_back(std::move(shape));
        }
        return;
    }

    const std::uint32_t count = io::readU32(in);
    module.scene.shapes.reserve(module.scene.shapes.size() + count);
    for (std::uint32_t i = 0; i < count; ++i) {
        runtime::RuntimeShapeNode shape = makeTerrShape(material, flags, lighting);
        shape.vertices.resize(codec::kTerrVertsPerRecord);
        for (std::size_t v = 0; v < codec::kTerrVertsPerRecord; ++v) {
            shape.vertices[v] = codec::readPackedWorldVertex(in);
        }
        bake::computeBounds(shape);
        module.scene.shapes.push_back(std::move(shape));
    }
}

inline void readLineChunkV2(std::istream& in, const StringTable& table, bake::RuntimeModule& module) {
    const std::uint32_t count = io::readU32(in);
    module.scene.lines.reserve(count);
    for (std::uint32_t i = 0; i < count; ++i) {
        module.scene.lines.push_back(readRuntimeLines(in, table));
    }
}

inline void readModlChunkV2(std::istream& in, const StringTable& table, bake::RuntimeModule& module) {
    const std::uint32_t count = io::readU32(in);
    module.scene.models.reserve(count);
    for (std::uint32_t i = 0; i < count; ++i) {
        module.scene.models.push_back(readRuntimeModel(in, table));
    }
}

inline void readPlacChunk(std::istream& in, bake::RuntimeModule& module) {
    module.includePlacement.origin_x_param = io::readU8(in);
    module.includePlacement.origin_y_param = io::readU8(in);
    module.includePlacement.origin_z_param = io::readU8(in);
    module.includePlacement.rotation_y_param = io::readU8(in);
}

inline void readPidxChunkV7(std::istream& in, std::vector<PackSectionIndexEntry>& entries) {
    const std::uint32_t count = io::readU32(in);
    entries.clear();
    entries.reserve(count);
    for (std::uint32_t i = 0; i < count; ++i) {
        PackSectionIndexEntry entry;
        entry.row = io::readU16(in);
        entry.column = io::readU16(in);
        entry.modelCount = io::readU32(in);
        entry.packOffset = io::readU64(in);
        entries.push_back(entry);
    }
}

[[nodiscard]] inline bool isKnownRuntimeChunk(const std::array<char, 4>& id) {
    return id == kChunkStrs || id == kChunkIncl || id == kChunkPlac || id == kChunkTrak ||
           id == kChunkTrac || id == kChunkPwrs || id == kChunkTerr || id == kChunkMesh ||
           id == kChunkLine ||            id == kChunkModl || id == kChunkPidx || id == kChunkPack || id == kChunkMemc ||
           id == kChunkLaun || id == kChunkDynm || id == kChunkSond ||
           id == kChunkTrset || id == kChunkEvnt || id == kChunkFint;
}

} // namespace detail

inline void writeRuntimeModule(
    const std::filesystem::path& outPath,
    const bake::RuntimeModule& module,
    WriteRuntimeModuleOptions options = {}) {
    const auto write_begin = std::chrono::steady_clock::now();
    StringTable table;
    detail::collectModuleStrings(table, module);

    const bool usePackModels = options.emitPackModels &&
        (options.pack_batches != nullptr || !module.scene.models.empty());

    detail::PackPayloadBuild packBuild;
    if (usePackModels) {
        const auto pack_begin = std::chrono::steady_clock::now();
        if (options.pack_batches != nullptr) {
            packBuild = detail::buildPackPayloadV7(table, *options.pack_batches);
        } else {
            packBuild = detail::buildPackPayloadV7(table, module.scene.models);
        }
        if (options.pack_write_stats != nullptr) {
            const auto pack_end = std::chrono::steady_clock::now();
            options.pack_write_stats->pack_build_ms = static_cast<std::size_t>(
                std::chrono::duration_cast<std::chrono::milliseconds>(pack_end - pack_begin).count());
            options.pack_write_stats->pack_entries = packBuild.entries.size();
            options.pack_write_stats->pack_bytes = packBuild.packPayload.size();
            options.pack_write_stats->strings_total = table.strings().size();
            if (options.pack_batches != nullptr) {
                const unsigned hw = std::thread::hardware_concurrency() == 0
                    ? 4u
                    : std::thread::hardware_concurrency();
                options.pack_write_stats->pack_workers = std::max(
                    1u,
                    std::min<unsigned>(
                        hw, static_cast<unsigned>(options.pack_batches->size())));
            }
        }
    }

    const std::vector<char> strsPayload = detail::buildStrsPayload(table);
    const std::vector<char> inclPayload = detail::buildInclPayloadV2(table, module.includes);
    const std::vector<char> trakPayload = detail::buildTrakPayloadV2(table, module.scene.tracks);
    const bool useTerr = codec::canUseTerrEncoding(module.scene.shapes);
    const std::vector<char> terrPayload =
        useTerr ? detail::buildTerrBatchedPayloadV2(table, module.scene.shapes) : std::vector<char>{};
    const std::vector<char> meshPayload = useTerr ? std::vector<char>{}
                                                  : detail::buildMeshPayloadV2(table, module.scene.shapes);
    const std::vector<char> linePayload = detail::buildLinePayloadV2(table, module.scene.lines);
    const std::vector<char> modlPayload =
        usePackModels ? std::vector<char>{}
                      : detail::buildModlPayloadV2(table, module.scene.models);
    const std::vector<char> pidxPayload =
        usePackModels ? detail::buildPidxPayloadV7(packBuild.entries) : std::vector<char>{};
    const std::vector<char> tracPayload = detail::buildTracPayloadV2(table, module.scene.traction);
    const std::vector<char> pwrsPayload =
        detail::buildPwrsPayloadV2(table, module.scene.powerSources);
    const std::vector<char> memcPayload = detail::buildMemcPayloadV2(table, module.scene.memcells);
    const std::vector<char> launPayload =
        detail::buildLaunPayloadV2(table, module.scene.eventLaunchers);
    const std::vector<char> dynmPayload = detail::buildDynmPayloadV2(table, module.scene.dynamics);
    const std::vector<char> sondPayload = detail::buildSondPayloadV2(table, module.scene.sounds);
    const std::vector<char> trsetPayload =
        detail::buildTrsetPayloadV2(table, module.scene.trainsets);
    const std::vector<char> evntPayload = detail::buildEvntPayloadV2(table, module.scene.events);

    std::ofstream out(outPath, std::ios::binary | std::ios::trunc);
    if (!out) {
        throw std::runtime_error("Nie mozna zapisac: " + outPath.string());
    }

    out.write(kMagic.data(), 4);
    io::writeU32(out, usePackModels ? kVersionRuntimeV7 : kVersionRuntime);

    io::writeChunkHeader(out, kChunkStrs, 8 + static_cast<std::uint32_t>(strsPayload.size()));
    out.write(strsPayload.data(), static_cast<std::streamsize>(strsPayload.size()));

    io::writeChunkHeader(out, kChunkIncl, 8 + static_cast<std::uint32_t>(inclPayload.size()));
    out.write(inclPayload.data(), static_cast<std::streamsize>(inclPayload.size()));

    if (!module.includePlacement.empty()) {
        io::writeChunkHeader(out, kChunkPlac, 12);
        io::writeU8(out, module.includePlacement.origin_x_param);
        io::writeU8(out, module.includePlacement.origin_y_param);
        io::writeU8(out, module.includePlacement.origin_z_param);
        io::writeU8(out, module.includePlacement.rotation_y_param);
    }

    io::writeChunkHeader(out, kChunkTrak, 8 + static_cast<std::uint32_t>(trakPayload.size()));
    out.write(trakPayload.data(), static_cast<std::streamsize>(trakPayload.size()));

    if (!terrPayload.empty()) {
        io::writeChunkHeader(out, kChunkTerr, 8 + static_cast<std::uint32_t>(terrPayload.size()));
        out.write(terrPayload.data(), static_cast<std::streamsize>(terrPayload.size()));
    } else if (!meshPayload.empty()) {
        io::writeChunkHeader(out, kChunkMesh, 8 + static_cast<std::uint32_t>(meshPayload.size()));
        out.write(meshPayload.data(), static_cast<std::streamsize>(meshPayload.size()));
    }
    if (!module.scene.lines.empty()) {
        io::writeChunkHeader(out, kChunkLine, 8 + static_cast<std::uint32_t>(linePayload.size()));
        out.write(linePayload.data(), static_cast<std::streamsize>(linePayload.size()));
    }
    if (usePackModels) {
        io::writeChunkHeader(out, kChunkPidx, 8 + static_cast<std::uint32_t>(pidxPayload.size()));
        out.write(pidxPayload.data(), static_cast<std::streamsize>(pidxPayload.size()));
        io::writeChunkHeader(
            out, kChunkPack, 8 + static_cast<std::uint32_t>(packBuild.packPayload.size()));
        out.write(
            packBuild.packPayload.data(),
            static_cast<std::streamsize>(packBuild.packPayload.size()));
    } else if (!module.scene.models.empty()) {
        io::writeChunkHeader(out, kChunkModl, 8 + static_cast<std::uint32_t>(modlPayload.size()));
        out.write(modlPayload.data(), static_cast<std::streamsize>(modlPayload.size()));
    }
    if (!module.scene.traction.empty()) {
        io::writeChunkHeader(out, kChunkTrac, 8 + static_cast<std::uint32_t>(tracPayload.size()));
        out.write(tracPayload.data(), static_cast<std::streamsize>(tracPayload.size()));
    }
    if (!module.scene.powerSources.empty()) {
        io::writeChunkHeader(out, kChunkPwrs, 8 + static_cast<std::uint32_t>(pwrsPayload.size()));
        out.write(pwrsPayload.data(), static_cast<std::streamsize>(pwrsPayload.size()));
    }
    if (!module.scene.memcells.empty()) {
        io::writeChunkHeader(out, kChunkMemc, 8 + static_cast<std::uint32_t>(memcPayload.size()));
        out.write(memcPayload.data(), static_cast<std::streamsize>(memcPayload.size()));
    }
    if (!module.scene.eventLaunchers.empty()) {
        io::writeChunkHeader(out, kChunkLaun, 8 + static_cast<std::uint32_t>(launPayload.size()));
        out.write(launPayload.data(), static_cast<std::streamsize>(launPayload.size()));
    }
    if (!module.scene.dynamics.empty()) {
        io::writeChunkHeader(out, kChunkDynm, 8 + static_cast<std::uint32_t>(dynmPayload.size()));
        out.write(dynmPayload.data(), static_cast<std::streamsize>(dynmPayload.size()));
    }
    if (!module.scene.sounds.empty()) {
        io::writeChunkHeader(out, kChunkSond, 8 + static_cast<std::uint32_t>(sondPayload.size()));
        out.write(sondPayload.data(), static_cast<std::streamsize>(sondPayload.size()));
    }
    if (!module.scene.trainsets.empty()) {
        io::writeChunkHeader(out, kChunkTrset, 8 + static_cast<std::uint32_t>(trsetPayload.size()));
        out.write(trsetPayload.data(), static_cast<std::streamsize>(trsetPayload.size()));
    }
    if (!module.scene.events.empty()) {
        io::writeChunkHeader(out, kChunkEvnt, 8 + static_cast<std::uint32_t>(evntPayload.size()));
        out.write(evntPayload.data(), static_cast<std::streamsize>(evntPayload.size()));
    }
    if (module.scene.firstInitCount > 0) {
        io::writeChunkHeader(out, kChunkFint, 12);
        io::writeU32(out, module.scene.firstInitCount);
    }

    if (options.pack_write_stats != nullptr) {
        const auto write_end = std::chrono::steady_clock::now();
        options.pack_write_stats->write_file_ms = static_cast<std::size_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(write_end - write_begin).count());
    }
}

[[nodiscard]] inline LoadedRuntimeModule readRuntimeModule(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        throw std::runtime_error("Nie mozna otworzyc: " + path.string());
    }

    std::array<char, 4> magic{};
    in.read(magic.data(), 4);
    if (!in || magic != kMagic) {
        throw std::runtime_error("EU7B: zly magic");
    }

    const std::uint32_t version = io::readU32(in);
    if (!isSupportedEu7bVersion(version)) {
        throw std::runtime_error(
            "EU7B: nieobslugiwana wersja runtime " + std::to_string(version) +
            " (aktualna " + std::to_string(kVersionRuntime) + ")");
    }

    LoadedRuntimeModule loaded;
    loaded.version = version;
    loaded.module = bake::RuntimeModule{};

    while (in.peek() != EOF) {
        std::array<char, 4> chunkId{};
        in.read(chunkId.data(), 4);
        if (!in) {
            break;
        }
        const std::uint32_t chunkSize = io::readU32(in);
        if (chunkSize < 8) {
            throw std::runtime_error("EU7B: uszkodzony chunk");
        }
        const std::uint32_t payloadSize = chunkSize - 8;

        LoadedRuntimeModule::ChunkSummary summary;
        summary.id = chunkId;
        summary.size = chunkSize;

        const auto chunkStart = in.tellg();

        if (chunkId == kChunkStrs) {
            summary.recognized = true;
            detail::readStrsChunk(in, loaded.strings);
        } else if (chunkId == kChunkIncl) {
            summary.recognized = true;
            detail::readInclChunkV2(in, loaded.strings, loaded.version, loaded.module);
        } else if (chunkId == kChunkPlac) {
            summary.recognized = true;
            detail::readPlacChunk(in, loaded.module);
        } else if (chunkId == kChunkTrak) {
            summary.recognized = true;
            detail::readTrakChunkV2(in, loaded.strings, loaded.module);
        } else if (chunkId == kChunkTerr) {
            summary.recognized = true;
            detail::readTerrChunkV2(in, loaded.strings, loaded.module);
        } else if (chunkId == kChunkMesh) {
            summary.recognized = true;
            detail::readMeshChunkV2(in, loaded.strings, loaded.module);
        } else if (chunkId == kChunkLine) {
            summary.recognized = true;
            detail::readLineChunkV2(in, loaded.strings, loaded.module);
        } else if (chunkId == kChunkModl) {
            summary.recognized = true;
            detail::readModlChunkV2(in, loaded.strings, loaded.module);
        } else if (chunkId == kChunkPidx) {
            summary.recognized = true;
            std::vector<detail::PackSectionIndexEntry> packIndex;
            detail::readPidxChunkV7(in, packIndex);
        } else if (chunkId == kChunkPack) {
            summary.recognized = true;
            in.seekg(static_cast<std::streamoff>(payloadSize), std::ios::cur);
        } else if (chunkId == kChunkTrac) {
            summary.recognized = true;
            detail::readTracChunkV2(in, loaded.strings, loaded.module);
        } else if (chunkId == kChunkPwrs) {
            summary.recognized = true;
            detail::readPwrsChunkV2(in, loaded.strings, loaded.module);
        } else if (chunkId == kChunkMemc) {
            summary.recognized = true;
            detail::readMemcChunkV2(in, loaded.strings, loaded.module);
        } else if (chunkId == kChunkLaun) {
            summary.recognized = true;
            detail::readLaunChunkV2(in, loaded.strings, loaded.module);
        } else if (chunkId == kChunkDynm) {
            summary.recognized = true;
            detail::readDynmChunkV2(in, loaded.strings, loaded.module);
        } else if (chunkId == kChunkSond) {
            summary.recognized = true;
            detail::readSondChunkV2(in, loaded.strings, loaded.module);
        } else if (chunkId == kChunkTrset) {
            summary.recognized = true;
            detail::readTrsetChunkV2(in, loaded.strings, loaded.module);
        } else if (chunkId == kChunkEvnt) {
            summary.recognized = true;
            detail::readEvntChunkV2(in, loaded.strings, loaded.module);
        } else if (chunkId == kChunkFint) {
            summary.recognized = true;
            loaded.module.scene.firstInitCount = io::readU32(in);
        } else {
            in.seekg(static_cast<std::streamoff>(payloadSize), std::ios::cur);
        }

        if (detail::isKnownRuntimeChunk(chunkId)) {
            if (in.tellg() - chunkStart != static_cast<std::streamoff>(payloadSize)) {
                throw std::runtime_error("EU7B: uszkodzony chunk payload");
            }
        }

        loaded.chunks.push_back(summary);
    }

    return loaded;
}

} // namespace eu07::scene::binary
