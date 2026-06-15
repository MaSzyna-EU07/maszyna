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
#include <unordered_map>
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
inline constexpr std::array<char, 4> kChunkProt{'P', 'R', 'O', 'T'};

// PACK section payload: v9 = UMES, v10 = UMES + CHNK, v11 = UMES + UTEX + CHNK, v12 = v11 + PROT inst, v13 = v12 + mesh/tex indices + cell_id.
inline constexpr std::uint8_t kPackSectionFormatV9 = 2;
inline constexpr std::uint8_t kPackSectionFormatV10 = 3;
inline constexpr std::uint8_t kPackSectionFormatV11 = 4;
inline constexpr std::uint8_t kPackSectionFormatV12 = 5;
inline constexpr std::uint8_t kPackSectionFormatV13 = 6;
inline constexpr std::uint16_t kPackIndexEmpty = 0xFFFF;
inline constexpr std::int32_t kEu07CellSize = 250;
inline constexpr std::int32_t kEu07CellsPerSection = 4;
inline constexpr std::size_t kPackSectionChunkModels = 512;
inline constexpr std::size_t kPackSectionChunkModelsHeavy = 256;
inline constexpr std::size_t kPackSectionHeavyModelThreshold = 1024;

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
    std::vector<char> protPayload;
    std::vector<PackSectionIndexEntry> entries;
};

[[nodiscard]] inline bool isLoadableMeshPath(const std::string_view path) {
    return !path.empty() && path != "notload";
}

[[nodiscard]] inline bool isPackTexturePath(const std::string_view path) {
    if (path.empty() || path == "none" || path.front() == '*') {
        return false;
    }
    if (path.starts_with("make:") || path.starts_with("@") || path.starts_with("none|")) {
        return false;
    }
    if (path.ends_with(".e3d") || path.ends_with(".t3d")) {
        return false;
    }
    if (path == "none" || path.ends_with("/none") || path.ends_with('/')) {
        return false;
    }
    if (path.find("tr/none") != std::string_view::npos || path.find('#') != std::string_view::npos) {
        return false;
    }
    return true;
}

[[nodiscard]] inline std::string packModelTextureDir(std::string model_file) {
    if (model_file.empty() || model_file == "notload") {
        return {};
    }
    for (char& ch : model_file) {
        if (ch == '\\') {
            ch = '/';
        }
    }
    const auto dot = model_file.rfind('.');
    if (dot != std::string::npos) {
        model_file.resize(dot);
    }
    const auto slash_pos = model_file.rfind('/');
    if (slash_pos == std::string::npos) {
        return {};
    }
    return model_file.substr(0, slash_pos + 1);
}

[[nodiscard]] inline bool packTexturePathIsRooted(const std::string_view path) {
    return path.starts_with("dynamic/") || path.starts_with("textures/") ||
           path.starts_with("scenery/") || path.starts_with("models/");
}

[[nodiscard]] inline bool probePackTextureCandidate(const std::string& candidate) {
    static constexpr std::array<const char*, 8> kExtensions{
        ".mat", ".dds", ".tga", ".ktx", ".png", ".bmp", ".jpg", ".tex"};
    for (const char* ext : kExtensions) {
        const std::filesystem::path path(candidate + ext);
        if (std::filesystem::exists(path)) {
            return true;
        }
    }
    return false;
}

[[nodiscard]] inline std::string resolvePackTexturePath(
    const std::string& model_file,
    const std::string& texture_file) {
    if (!isPackTexturePath(texture_file)) {
        return {};
    }

    std::string normalized = texture_file;
    for (char& ch : normalized) {
        if (ch == '\\') {
            ch = '/';
        }
    }

    std::vector<std::string> candidates;
    candidates.reserve(4);
    auto append_candidate = [&](std::string candidate) {
        if (candidate.empty()) {
            return;
        }
        for (char& ch : candidate) {
            if (ch == '\\') {
                ch = '/';
            }
        }
        if (std::find(candidates.begin(), candidates.end(), candidate) == candidates.end()) {
            candidates.emplace_back(std::move(candidate));
        }
    };

    append_candidate(normalized);
    const auto model_dir = packModelTextureDir(model_file);
    if (!model_dir.empty()) {
        append_candidate(model_dir + normalized);
    }
    if (!packTexturePathIsRooted(normalized)) {
        append_candidate(std::string("textures/") + normalized);
    }

    for (const std::string& candidate : candidates) {
        if (probePackTextureCandidate(candidate)) {
            return candidate;
        }
    }
    return {};
}

[[nodiscard]] inline runtime::Vec3 packSectionCenter(
    const codec::TerrSectionKey section) noexcept {
    runtime::Vec3 center{};
    center.x =
        (static_cast<double>(section.x) -
         static_cast<double>(codec::kEu07RegionSideSectionCount) / 2.0 + 0.5) *
        static_cast<double>(codec::kEu07SectionSize);
    center.z =
        (static_cast<double>(section.z) -
         static_cast<double>(codec::kEu07RegionSideSectionCount) / 2.0 + 0.5) *
        static_cast<double>(codec::kEu07SectionSize);
    return center;
}

[[nodiscard]] inline std::uint8_t computePackCellId(
    const runtime::Vec3& location,
    const codec::TerrSectionKey section) noexcept {
    const runtime::Vec3 center = packSectionCenter(section);
    const int column = static_cast<int>(std::floor(
        (location.x - (center.x - static_cast<double>(codec::kEu07SectionSize) / 2.0)) /
        static_cast<double>(kEu07CellSize)));
    const int row = static_cast<int>(std::floor(
        (location.z - (center.z - static_cast<double>(codec::kEu07SectionSize) / 2.0)) /
        static_cast<double>(kEu07CellSize)));
    const int clamped_col = std::clamp(column, 0, kEu07CellsPerSection - 1);
    const int clamped_row = std::clamp(row, 0, kEu07CellsPerSection - 1);
    return static_cast<std::uint8_t>(clamped_row * kEu07CellsPerSection + clamped_col);
}

inline void sortPackSectionModelsForStreaming(
    std::vector<runtime::RuntimeModelInstance>& models,
    const codec::TerrSectionKey section) {
    if (models.size() < 2) {
        return;
    }

    std::unordered_map<std::string, std::uint32_t> mesh_frequency;
    mesh_frequency.reserve(models.size());
    for (const runtime::RuntimeModelInstance& model : models) {
        if (isLoadableMeshPath(model.modelFile)) {
            ++mesh_frequency[model.modelFile];
        }
    }

    const runtime::Vec3 section_center = packSectionCenter(section);
    const auto mesh_rank = [&](const std::string& path) -> std::uint32_t {
        const auto it = mesh_frequency.find(path);
        return it == mesh_frequency.end() ? 0u : it->second;
    };
    const auto distance_squared = [&](const runtime::RuntimeModelInstance& model) -> double {
        const double dx = model.location.x - section_center.x;
        const double dz = model.location.z - section_center.z;
        return dx * dx + dz * dz;
    };

    std::stable_sort(
        models.begin(),
        models.end(),
        [&](const runtime::RuntimeModelInstance& lhs, const runtime::RuntimeModelInstance& rhs) {
            const std::uint32_t lhs_rank = mesh_rank(lhs.modelFile);
            const std::uint32_t rhs_rank = mesh_rank(rhs.modelFile);
            if (lhs_rank != rhs_rank) {
                return lhs_rank > rhs_rank;
            }
            return distance_squared(lhs) < distance_squared(rhs);
        });
}

[[nodiscard]] inline std::vector<std::uint32_t> collectUniqueMeshIds(
    StringTable& table,
    const std::vector<runtime::RuntimeModelInstance>& models,
    const std::vector<std::string_view>* prototype_mesh_paths = nullptr) {
    std::unordered_map<std::string, std::uint32_t> mesh_frequency;
    mesh_frequency.reserve(models.size());

    for (const runtime::RuntimeModelInstance& model : models) {
        if (isLoadableMeshPath(model.modelFile)) {
            ++mesh_frequency[model.modelFile];
        }
    }

    std::vector<std::string> paths;
    paths.reserve(mesh_frequency.size());
    for (const auto& [path, frequency] : mesh_frequency) {
        if (frequency > 0) {
            paths.emplace_back(path);
        }
    }
    if (prototype_mesh_paths != nullptr) {
        for (const std::string_view path : *prototype_mesh_paths) {
            if (isLoadableMeshPath(path)) {
                const std::string text(path);
                if (false == mesh_frequency.contains(text)) {
                    paths.emplace_back(text);
                    mesh_frequency.emplace(text, 0u);
                }
            }
        }
    }

    std::stable_sort(
        paths.begin(),
        paths.end(),
        [&](const std::string& lhs, const std::string& rhs) {
            const std::uint32_t lhs_rank = mesh_frequency[lhs];
            const std::uint32_t rhs_rank = mesh_frequency[rhs];
            if (lhs_rank != rhs_rank) {
                return lhs_rank > rhs_rank;
            }
            return lhs < rhs;
        });

    std::vector<std::uint32_t> ids;
    ids.reserve(paths.size());
    for (const std::string& path : paths) {
        ids.push_back(table.intern(path));
    }
    return ids;
}

[[nodiscard]] inline std::vector<std::uint32_t> collectUniqueTextureIds(
    StringTable& table,
    const std::vector<runtime::RuntimeModelInstance>& models) {
    std::unordered_map<std::string, std::uint32_t> texture_frequency;
    texture_frequency.reserve(models.size());

    for (const runtime::RuntimeModelInstance& model : models) {
        if (isPackTexturePath(model.textureFile)) {
            ++texture_frequency[model.textureFile];
        }
    }

    std::vector<std::string> paths;
    paths.reserve(texture_frequency.size());
    for (const auto& [path, frequency] : texture_frequency) {
        if (frequency > 0) {
            paths.emplace_back(path);
        }
    }

    std::stable_sort(
        paths.begin(),
        paths.end(),
        [&](const std::string& lhs, const std::string& rhs) {
            const std::uint32_t lhs_rank = texture_frequency[lhs];
            const std::uint32_t rhs_rank = texture_frequency[rhs];
            if (lhs_rank != rhs_rank) {
                return lhs_rank > rhs_rank;
            }
            return lhs < rhs;
        });

    std::vector<std::uint32_t> ids;
    ids.reserve(paths.size());
    for (const std::string& path : paths) {
        ids.push_back(table.intern(path));
    }
    return ids;
}

[[nodiscard]] inline std::vector<std::string> collectUniqueMeshPaths(
    const std::vector<runtime::RuntimeModelInstance>& models,
    const std::vector<std::string_view>* prototype_mesh_paths = nullptr) {
    std::unordered_map<std::string, std::uint32_t> mesh_frequency;
    mesh_frequency.reserve(models.size());

    for (const runtime::RuntimeModelInstance& model : models) {
        if (isLoadableMeshPath(model.modelFile)) {
            ++mesh_frequency[model.modelFile];
        }
    }

    std::vector<std::string> paths;
    paths.reserve(mesh_frequency.size());
    for (const auto& [path, frequency] : mesh_frequency) {
        if (frequency > 0) {
            paths.emplace_back(path);
        }
    }
    if (prototype_mesh_paths != nullptr) {
        for (const std::string_view path : *prototype_mesh_paths) {
            if (isLoadableMeshPath(path)) {
                const std::string text(path);
                if (!mesh_frequency.contains(text)) {
                    paths.emplace_back(text);
                    mesh_frequency.emplace(text, 0u);
                }
            }
        }
    }

    std::stable_sort(
        paths.begin(),
        paths.end(),
        [&](const std::string& lhs, const std::string& rhs) {
            const std::uint32_t lhs_rank = mesh_frequency[lhs];
            const std::uint32_t rhs_rank = mesh_frequency[rhs];
            if (lhs_rank != rhs_rank) {
                return lhs_rank > rhs_rank;
            }
            return lhs < rhs;
        });
    return paths;
}

[[nodiscard]] inline std::vector<std::string> collectUniqueTexturePaths(
    const std::vector<runtime::RuntimeModelInstance>& models) {
    std::unordered_map<std::string, std::uint32_t> texture_frequency;
    texture_frequency.reserve(models.size());

    for (const runtime::RuntimeModelInstance& model : models) {
        if (isPackTexturePath(model.textureFile)) {
            ++texture_frequency[model.textureFile];
        }
    }

    std::vector<std::string> paths;
    paths.reserve(texture_frequency.size());
    for (const auto& [path, frequency] : texture_frequency) {
        if (frequency > 0) {
            paths.emplace_back(path);
        }
    }

    std::stable_sort(
        paths.begin(),
        paths.end(),
        [&](const std::string& lhs, const std::string& rhs) {
            const std::uint32_t lhs_rank = texture_frequency[lhs];
            const std::uint32_t rhs_rank = texture_frequency[rhs];
            if (lhs_rank != rhs_rank) {
                return lhs_rank > rhs_rank;
            }
            return lhs < rhs;
        });
    return paths;
}

[[nodiscard]] inline std::unordered_map<std::string, std::uint16_t> makePackPathIndexMap(
    const std::vector<std::string>& paths) {
    std::unordered_map<std::string, std::uint16_t> index_by_path;
    index_by_path.reserve(paths.size());
    for (std::size_t index = 0; index < paths.size(); ++index) {
        index_by_path.emplace(paths[index], static_cast<std::uint16_t>(index));
    }
    return index_by_path;
}

[[nodiscard]] inline std::uint16_t lookupPackPathIndex(
    const std::unordered_map<std::string, std::uint16_t>& index_by_path,
    const std::string& path) {
    const auto it = index_by_path.find(path);
    return it == index_by_path.end() ? kPackIndexEmpty : it->second;
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
    const std::size_t chunk_models =
        models.size() > kPackSectionHeavyModelThreshold ? kPackSectionChunkModelsHeavy
                                                        : kPackSectionChunkModels;

    std::vector<std::vector<char>> chunk_payloads;
    chunk_payloads.reserve((models.size() + chunk_models - 1) / chunk_models);
    for (std::size_t offset = 0; offset < models.size(); offset += chunk_models) {
        const std::size_t end = std::min(offset + chunk_models, models.size());
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
        const std::size_t model_begin = static_cast<std::size_t>(chunk_index) * chunk_models;
        const std::size_t model_end = std::min(model_begin + chunk_models, models.size());
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

[[nodiscard]] inline std::vector<char> buildPackSectionPayloadV11(
    StringTable& table,
    const std::vector<runtime::RuntimeModelInstance>& models,
    const std::uint32_t inst_count = 0,
    const std::vector<std::string_view>* prototype_mesh_paths = nullptr) {
    const auto mesh_ids = collectUniqueMeshIds(table, models, prototype_mesh_paths);
    const auto texture_ids = collectUniqueTextureIds(table, models);
    const std::uint32_t solo_count = static_cast<std::uint32_t>(models.size());
    const std::size_t chunk_models =
        models.size() > kPackSectionHeavyModelThreshold ? kPackSectionChunkModelsHeavy
                                                        : kPackSectionChunkModels;

    std::vector<std::vector<char>> chunk_payloads;
    chunk_payloads.reserve((models.size() + chunk_models - 1) / chunk_models);
    for (std::size_t offset = 0; offset < models.size(); offset += chunk_models) {
        const std::size_t end = std::min(offset + chunk_models, models.size());
        chunk_payloads.push_back(buildPackModelBlob(table, models, offset, end));
    }
    if (chunk_payloads.empty()) {
        chunk_payloads.emplace_back();
    }

    const std::uint32_t chunk_count = static_cast<std::uint32_t>(chunk_payloads.size());
    const std::uint32_t header_size =
        1u + 4u + 4u + 4u + static_cast<std::uint32_t>(mesh_ids.size()) * 4u + 4u +
        static_cast<std::uint32_t>(texture_ids.size()) * 4u + 4u + chunk_count * 8u;

    std::vector<std::uint32_t> chunk_offsets(chunk_count);
    std::vector<std::uint32_t> chunk_model_counts(chunk_count);
    std::uint32_t payload_offset = header_size;
    for (std::uint32_t chunk_index = 0; chunk_index < chunk_count; ++chunk_index) {
        chunk_offsets[chunk_index] = payload_offset;
        const std::size_t model_begin = static_cast<std::size_t>(chunk_index) * chunk_models;
        const std::size_t model_end = std::min(model_begin + chunk_models, models.size());
        chunk_model_counts[chunk_index] =
            static_cast<std::uint32_t>(model_end > model_begin ? model_end - model_begin : 0);
        payload_offset += static_cast<std::uint32_t>(chunk_payloads[chunk_index].size());
    }

    std::ostringstream packOut(std::ios::binary);
    io::writeU8(packOut, kPackSectionFormatV11);
    io::writeU32(packOut, solo_count);
    io::writeU32(packOut, inst_count);
    io::writeU32(packOut, static_cast<std::uint32_t>(mesh_ids.size()));
    for (const std::uint32_t id : mesh_ids) {
        io::writeU32(packOut, id);
    }
    io::writeU32(packOut, static_cast<std::uint32_t>(texture_ids.size()));
    for (const std::uint32_t id : texture_ids) {
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

struct PackPrototypeKey {
    std::string modelFile;
    std::string textureFile;
    double rangeSquaredMin = 0.0;
    double rangeSquaredMax = 0.0;
    bool visible = true;
    bool isTerrain = false;
    bool transition = true;

    [[nodiscard]] bool operator==(const PackPrototypeKey& other) const noexcept {
        return modelFile == other.modelFile && textureFile == other.textureFile &&
               rangeSquaredMin == other.rangeSquaredMin &&
               rangeSquaredMax == other.rangeSquaredMax && visible == other.visible &&
               isTerrain == other.isTerrain && transition == other.transition;
    }
};

struct PackPrototypeKeyHash {
    [[nodiscard]] std::size_t operator()(const PackPrototypeKey& key) const noexcept {
        std::size_t hash = std::hash<std::string>{}(key.modelFile);
        hash ^= std::hash<std::string>{}(key.textureFile) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        hash ^= std::hash<double>{}(key.rangeSquaredMin) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        hash ^= std::hash<double>{}(key.rangeSquaredMax) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        hash ^= static_cast<std::size_t>(key.visible) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        hash ^= static_cast<std::size_t>(key.isTerrain) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        hash ^= static_cast<std::size_t>(key.transition) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        return hash;
    }
};

[[nodiscard]] inline PackPrototypeKey makePackPrototypeKey(
    const runtime::RuntimeModelInstance& model) noexcept {
    PackPrototypeKey key;
    key.modelFile = model.modelFile;
    key.textureFile = model.textureFile;
    key.rangeSquaredMin = model.node.rangeSquaredMin;
    key.rangeSquaredMax = model.node.rangeSquaredMax;
    key.visible = model.node.visible;
    key.isTerrain = model.isTerrain;
    key.transition = model.transition;
    return key;
}

[[nodiscard]] inline bool modelMustBePackSolo(const runtime::RuntimeModelInstance& model) noexcept {
    if (model.isTerrain) {
        return true;
    }
    if (!model.lightStates.empty() || !model.lightColors.empty()) {
        return true;
    }
    if (!isLoadableMeshPath(model.modelFile)) {
        return true;
    }
    return false;
}

[[nodiscard]] inline bool modelCanBePackInstanced(const runtime::RuntimeModelInstance& model) noexcept {
    return !modelMustBePackSolo(model);
}

inline void writeRuntimePrototype(
    std::ostream& out,
    StringTable& table,
    const runtime::RuntimeModelInstance& model,
    const bool emit_v9_fields = false) {
    codec::writeSlimNode(out, table, model.node, "model");
    io::writeU8(out, model.isTerrain ? 1 : 0);
    io::writeU8(out, model.transition ? 1 : 0);
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
    if (emit_v9_fields) {
        const std::string resolved =
            resolvePackTexturePath(model.modelFile, model.textureFile);
        codec::writeStringId(out, table, resolved);
        const bool needs_full_load =
            !model.lightStates.empty() || !model.lightColors.empty() || !model.transition;
        std::uint8_t pack_flags = 0;
        if (needs_full_load) {
            pack_flags |= 1u;
        } else if (!model.isTerrain) {
            pack_flags |= 2u;
        }
        io::writeU8(out, pack_flags);
        io::writeU32(out, 0u);
        const float baked_range_min =
            static_cast<float>(std::sqrt(std::max(0.0, model.node.rangeSquaredMin)));
        const float baked_range_max =
            model.node.rangeSquaredMax >= std::numeric_limits<double>::max() * 0.5
                ? -1.f
                : static_cast<float>(std::sqrt(model.node.rangeSquaredMax));
        io::writeF32(out, baked_range_min);
        io::writeF32(out, baked_range_max);
    }
}

[[nodiscard]] inline std::vector<char> buildProtPayload(
    StringTable& table,
    const std::vector<runtime::RuntimeModelInstance>& prototypes,
    const bool emit_v9_fields = false) {
    if (emit_v9_fields) {
        for (const runtime::RuntimeModelInstance& proto : prototypes) {
            const std::string resolved =
                resolvePackTexturePath(proto.modelFile, proto.textureFile);
            if (!resolved.empty()) {
                table.intern(resolved);
            }
        }
    }
    std::ostringstream out(std::ios::binary);
    io::writeU32(out, static_cast<std::uint32_t>(prototypes.size()));
    for (const runtime::RuntimeModelInstance& proto : prototypes) {
        writeRuntimePrototype(out, table, proto, emit_v9_fields);
    }
    const std::string blob = out.str();
    return {blob.begin(), blob.end()};
}

struct PackPrototypeTable {
    std::vector<runtime::RuntimeModelInstance> prototypes;
    std::unordered_map<PackPrototypeKey, std::uint32_t, PackPrototypeKeyHash> key_to_id;
};

[[nodiscard]] inline PackPrototypeTable collectPackPrototypes(
    StringTable& table,
    const std::vector<codec::ModelSectionBatch>& batches) {
    std::unordered_map<PackPrototypeKey, std::uint32_t, PackPrototypeKeyHash> frequency;
    frequency.reserve(1024);

    for (const codec::ModelSectionBatch& batch : batches) {
        for (const runtime::RuntimeModelInstance& model : batch.models) {
            if (!modelCanBePackInstanced(model)) {
                continue;
            }
            ++frequency[makePackPrototypeKey(model)];
        }
    }

    PackPrototypeTable result;
    for (const codec::ModelSectionBatch& batch : batches) {
        for (const runtime::RuntimeModelInstance& model : batch.models) {
            if (!modelCanBePackInstanced(model)) {
                continue;
            }
            const PackPrototypeKey key = makePackPrototypeKey(model);
            const auto freq_it = frequency.find(key);
            if (freq_it == frequency.end() || freq_it->second < 2) {
                continue;
            }
            if (result.key_to_id.contains(key)) {
                continue;
            }
            const std::uint32_t proto_id =
                static_cast<std::uint32_t>(result.prototypes.size());
            result.key_to_id.emplace(key, proto_id);
            runtime::RuntimeModelInstance proto = model;
            proto.location = {};
            proto.angles = {};
            proto.scale = {1.0, 1.0, 1.0};
            proto.node.transform = {};
            collectModelStrings(table, proto);
            result.prototypes.push_back(std::move(proto));
        }
    }
    return result;
}

enum class PackSectionEntryKind : std::uint8_t { Solo, Inst };

struct PackSectionEntry {
    PackSectionEntryKind kind = PackSectionEntryKind::Solo;
    std::size_t model_index = 0;
    std::uint32_t proto_id = 0;
    std::uint8_t pack_cell_id = 0;
};

inline void writePackSoloRecordV13(
    std::ostream& out,
    StringTable& table,
    const runtime::RuntimeModelInstance& model,
    const std::unordered_map<std::string, std::uint16_t>& mesh_index,
    const std::unordered_map<std::string, std::uint16_t>& texture_index,
    const std::uint8_t pack_cell_id) {
    codec::writeSlimNode(out, table, model.node, "model");
    io::writeU8(out, model.isTerrain ? 1 : 0);
    io::writeU8(out, model.transition ? 1 : 0);
    io::writeVec3(out, model.location);
    io::writeVec3(out, model.angles);
    io::writeVec3(out, model.scale);
    io::writeU16(out, lookupPackPathIndex(mesh_index, model.modelFile));
    io::writeU16(out, lookupPackPathIndex(texture_index, model.textureFile));
    io::writeU32(out, static_cast<std::uint32_t>(model.lightStates.size()));
    for (float light : model.lightStates) {
        io::writeF32(out, light);
    }
    io::writeU32(out, static_cast<std::uint32_t>(model.lightColors.size()));
    for (std::uint32_t color : model.lightColors) {
        io::writeU32(out, color);
    }
    io::writeU8(out, pack_cell_id);
}

inline void writePackInstRecord(
    std::ostream& out,
    StringTable& table,
    const std::uint32_t proto_id,
    const runtime::RuntimeModelInstance& model,
    const std::uint8_t pack_cell_id = 0) {
    io::writeU32(out, proto_id);
    io::writeVec3(out, model.location);
    io::writeVec3(out, model.angles);
    io::writeVec3(out, model.scale);
    codec::writeStringId(out, table, model.node.name);
    io::writeU8(out, pack_cell_id);
}

[[nodiscard]] inline std::vector<char> buildPackSectionChunkBlobV12(
    StringTable& table,
    const std::vector<runtime::RuntimeModelInstance>& models,
    const std::vector<PackSectionEntry>& entries,
    const std::size_t begin,
    const std::size_t end) {
    // Reader decodes each chunk as a solo block followed by inst records (v8 layout).
    std::ostringstream out(std::ios::binary);
    for (std::size_t index = begin; index < end; ++index) {
        const PackSectionEntry& entry = entries[index];
        if (entry.kind != PackSectionEntryKind::Solo) {
            continue;
        }
        runtime::RuntimeModelInstance worldModel = models[entry.model_index];
        worldModel.node.transform = {};
        writeRuntimeModel(out, table, worldModel);
    }
    for (std::size_t index = begin; index < end; ++index) {
        const PackSectionEntry& entry = entries[index];
        if (entry.kind != PackSectionEntryKind::Inst) {
            continue;
        }
        writePackInstRecord(out, table, entry.proto_id, models[entry.model_index]);
    }
    const std::string blob = out.str();
    return {blob.begin(), blob.end()};
}

[[nodiscard]] inline std::vector<char> buildPackSectionPayloadV12(
    StringTable& table,
    const std::vector<runtime::RuntimeModelInstance>& models,
    const PackPrototypeTable& prototypes) {
    std::vector<PackSectionEntry> entries;
    entries.reserve(models.size());

    std::uint32_t solo_count = 0;
    std::uint32_t inst_count = 0;
    for (std::size_t model_index = 0; model_index < models.size(); ++model_index) {
        const runtime::RuntimeModelInstance& model = models[model_index];
        if (modelMustBePackSolo(model)) {
            entries.push_back({PackSectionEntryKind::Solo, model_index, 0});
            ++solo_count;
            continue;
        }
        const PackPrototypeKey key = makePackPrototypeKey(model);
        const auto proto_it = prototypes.key_to_id.find(key);
        if (proto_it == prototypes.key_to_id.end()) {
            entries.push_back({PackSectionEntryKind::Solo, model_index, 0});
            ++solo_count;
            continue;
        }
        entries.push_back({PackSectionEntryKind::Inst, model_index, proto_it->second});
        ++inst_count;
    }

    std::vector<std::string_view> prototype_mesh_paths;
    prototype_mesh_paths.reserve(prototypes.prototypes.size());
    for (const runtime::RuntimeModelInstance& proto : prototypes.prototypes) {
        prototype_mesh_paths.emplace_back(proto.modelFile);
    }

    const auto mesh_ids =
        collectUniqueMeshIds(table, models, &prototype_mesh_paths);
    const auto texture_ids = collectUniqueTextureIds(table, models);
    const std::size_t chunk_models =
        models.size() > kPackSectionHeavyModelThreshold ? kPackSectionChunkModelsHeavy
                                                        : kPackSectionChunkModels;

    std::vector<std::vector<char>> chunk_payloads;
    std::vector<std::uint32_t> chunk_solo_counts;
    std::vector<std::uint32_t> chunk_inst_counts;
    chunk_payloads.reserve((entries.size() + chunk_models - 1) / chunk_models);
    chunk_solo_counts.reserve(chunk_payloads.capacity());
    chunk_inst_counts.reserve(chunk_payloads.capacity());

    for (std::size_t offset = 0; offset < entries.size(); offset += chunk_models) {
        const std::size_t end = std::min(offset + chunk_models, entries.size());
        std::uint32_t chunk_solo = 0;
        std::uint32_t chunk_inst = 0;
        for (std::size_t index = offset; index < end; ++index) {
            if (entries[index].kind == PackSectionEntryKind::Solo) {
                ++chunk_solo;
            } else {
                ++chunk_inst;
            }
        }
        chunk_solo_counts.push_back(chunk_solo);
        chunk_inst_counts.push_back(chunk_inst);
        chunk_payloads.push_back(
            buildPackSectionChunkBlobV12(table, models, entries, offset, end));
    }
    if (chunk_payloads.empty()) {
        chunk_payloads.emplace_back();
        chunk_solo_counts.push_back(0);
        chunk_inst_counts.push_back(0);
    }

    const std::uint32_t chunk_count = static_cast<std::uint32_t>(chunk_payloads.size());
    const std::uint32_t header_size =
        1u + 4u + 4u + 4u + static_cast<std::uint32_t>(mesh_ids.size()) * 4u + 4u +
        static_cast<std::uint32_t>(texture_ids.size()) * 4u + 4u + chunk_count * 12u;

    std::vector<std::uint32_t> chunk_offsets(chunk_count);
    std::uint32_t payload_offset = header_size;
    for (std::uint32_t chunk_index = 0; chunk_index < chunk_count; ++chunk_index) {
        chunk_offsets[chunk_index] = payload_offset;
        payload_offset += static_cast<std::uint32_t>(chunk_payloads[chunk_index].size());
    }

    std::ostringstream packOut(std::ios::binary);
    io::writeU8(packOut, kPackSectionFormatV12);
    io::writeU32(packOut, solo_count);
    io::writeU32(packOut, inst_count);
    io::writeU32(packOut, static_cast<std::uint32_t>(mesh_ids.size()));
    for (const std::uint32_t id : mesh_ids) {
        io::writeU32(packOut, id);
    }
    io::writeU32(packOut, static_cast<std::uint32_t>(texture_ids.size()));
    for (const std::uint32_t id : texture_ids) {
        io::writeU32(packOut, id);
    }
    io::writeU32(packOut, chunk_count);
    for (std::uint32_t chunk_index = 0; chunk_index < chunk_count; ++chunk_index) {
        io::writeU32(packOut, chunk_solo_counts[chunk_index]);
        io::writeU32(packOut, chunk_inst_counts[chunk_index]);
        io::writeU32(packOut, chunk_offsets[chunk_index]);
    }
    for (const std::vector<char>& payload : chunk_payloads) {
        packOut.write(payload.data(), static_cast<std::streamsize>(payload.size()));
    }
    const std::string blob = packOut.str();
    return {blob.begin(), blob.end()};
}

[[nodiscard]] inline std::vector<char> buildPackSectionChunkBlobV13(
    StringTable& table,
    const std::vector<runtime::RuntimeModelInstance>& models,
    const std::vector<PackSectionEntry>& entries,
    const std::size_t begin,
    const std::size_t end,
    const std::unordered_map<std::string, std::uint16_t>& mesh_index,
    const std::unordered_map<std::string, std::uint16_t>& texture_index) {
    std::ostringstream out(std::ios::binary);
    for (std::size_t index = begin; index < end; ++index) {
        const PackSectionEntry& entry = entries[index];
        if (entry.kind != PackSectionEntryKind::Solo) {
            continue;
        }
        runtime::RuntimeModelInstance worldModel = models[entry.model_index];
        worldModel.node.transform = {};
        writePackSoloRecordV13(
            out, table, worldModel, mesh_index, texture_index, entry.pack_cell_id);
    }
    for (std::size_t index = begin; index < end; ++index) {
        const PackSectionEntry& entry = entries[index];
        if (entry.kind != PackSectionEntryKind::Inst) {
            continue;
        }
        writePackInstRecord(
            out, table, entry.proto_id, models[entry.model_index], entry.pack_cell_id);
    }
    const std::string blob = out.str();
    return {blob.begin(), blob.end()};
}

[[nodiscard]] inline std::vector<char> buildPackSectionPayloadV13(
    StringTable& table,
    const std::vector<runtime::RuntimeModelInstance>& models,
    const PackPrototypeTable& prototypes,
    const codec::TerrSectionKey section) {
    std::vector<PackSectionEntry> entries;
    entries.reserve(models.size());

    std::uint32_t solo_count = 0;
    std::uint32_t inst_count = 0;
    for (std::size_t model_index = 0; model_index < models.size(); ++model_index) {
        const runtime::RuntimeModelInstance& model = models[model_index];
        PackSectionEntry entry;
        entry.model_index = model_index;
        entry.pack_cell_id = computePackCellId(model.location, section);
        if (modelMustBePackSolo(model)) {
            entry.kind = PackSectionEntryKind::Solo;
            ++solo_count;
            entries.push_back(entry);
            continue;
        }
        const PackPrototypeKey key = makePackPrototypeKey(model);
        const auto proto_it = prototypes.key_to_id.find(key);
        if (proto_it == prototypes.key_to_id.end()) {
            entry.kind = PackSectionEntryKind::Solo;
            ++solo_count;
            entries.push_back(entry);
            continue;
        }
        entry.kind = PackSectionEntryKind::Inst;
        entry.proto_id = proto_it->second;
        ++inst_count;
        entries.push_back(entry);
    }

    std::stable_sort(
        entries.begin(),
        entries.end(),
        [](const PackSectionEntry& lhs, const PackSectionEntry& rhs) {
            return lhs.pack_cell_id < rhs.pack_cell_id;
        });

    std::vector<std::string_view> prototype_mesh_paths;
    prototype_mesh_paths.reserve(prototypes.prototypes.size());
    for (const runtime::RuntimeModelInstance& proto : prototypes.prototypes) {
        prototype_mesh_paths.emplace_back(proto.modelFile);
    }

    const auto mesh_paths = collectUniqueMeshPaths(models, &prototype_mesh_paths);
    const auto texture_paths = collectUniqueTexturePaths(models);
    const auto mesh_ids = collectUniqueMeshIds(table, models, &prototype_mesh_paths);
    const auto texture_ids = collectUniqueTextureIds(table, models);
    const auto mesh_index = makePackPathIndexMap(mesh_paths);
    const auto texture_index = makePackPathIndexMap(texture_paths);
    const std::size_t chunk_models =
        models.size() > kPackSectionHeavyModelThreshold ? kPackSectionChunkModelsHeavy
                                                        : kPackSectionChunkModels;

    std::vector<std::vector<char>> chunk_payloads;
    std::vector<std::uint32_t> chunk_solo_counts;
    std::vector<std::uint32_t> chunk_inst_counts;
    chunk_payloads.reserve((entries.size() + chunk_models - 1) / chunk_models);
    chunk_solo_counts.reserve(chunk_payloads.capacity());
    chunk_inst_counts.reserve(chunk_payloads.capacity());

    for (std::size_t offset = 0; offset < entries.size(); offset += chunk_models) {
        const std::size_t end = std::min(offset + chunk_models, entries.size());
        std::uint32_t chunk_solo = 0;
        std::uint32_t chunk_inst = 0;
        for (std::size_t index = offset; index < end; ++index) {
            if (entries[index].kind == PackSectionEntryKind::Solo) {
                ++chunk_solo;
            } else {
                ++chunk_inst;
            }
        }
        chunk_solo_counts.push_back(chunk_solo);
        chunk_inst_counts.push_back(chunk_inst);
        chunk_payloads.push_back(
            buildPackSectionChunkBlobV13(
                table, models, entries, offset, end, mesh_index, texture_index));
    }
    if (chunk_payloads.empty()) {
        chunk_payloads.emplace_back();
        chunk_solo_counts.push_back(0);
        chunk_inst_counts.push_back(0);
    }

    const std::uint32_t chunk_count = static_cast<std::uint32_t>(chunk_payloads.size());
    const std::uint32_t header_size =
        1u + 4u + 4u + 4u + static_cast<std::uint32_t>(mesh_ids.size()) * 4u + 4u +
        static_cast<std::uint32_t>(texture_ids.size()) * 4u + 4u + chunk_count * 12u;

    std::vector<std::uint32_t> chunk_offsets(chunk_count);
    std::uint32_t payload_offset = header_size;
    for (std::uint32_t chunk_index = 0; chunk_index < chunk_count; ++chunk_index) {
        chunk_offsets[chunk_index] = payload_offset;
        payload_offset += static_cast<std::uint32_t>(chunk_payloads[chunk_index].size());
    }

    std::ostringstream packOut(std::ios::binary);
    io::writeU8(packOut, kPackSectionFormatV13);
    io::writeU32(packOut, solo_count);
    io::writeU32(packOut, inst_count);
    io::writeU32(packOut, static_cast<std::uint32_t>(mesh_ids.size()));
    for (const std::uint32_t id : mesh_ids) {
        io::writeU32(packOut, id);
    }
    io::writeU32(packOut, static_cast<std::uint32_t>(texture_ids.size()));
    for (const std::uint32_t id : texture_ids) {
        io::writeU32(packOut, id);
    }
    io::writeU32(packOut, chunk_count);
    for (std::uint32_t chunk_index = 0; chunk_index < chunk_count; ++chunk_index) {
        io::writeU32(packOut, chunk_solo_counts[chunk_index]);
        io::writeU32(packOut, chunk_inst_counts[chunk_index]);
        io::writeU32(packOut, chunk_offsets[chunk_index]);
    }
    for (const std::vector<char>& payload : chunk_payloads) {
        packOut.write(payload.data(), static_cast<std::streamsize>(payload.size()));
    }
    const std::string blob = packOut.str();
    return {blob.begin(), blob.end()};
}

[[nodiscard]] inline PackPayloadBuild buildPackPayloadV12(
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

    const PackPrototypeTable prototypes = collectPackPrototypes(table, batches);
    result.protPayload = buildProtPayload(table, prototypes.prototypes, true);

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
            std::vector<runtime::RuntimeModelInstance> section_models = batch.models;
            sortPackSectionModelsForStreaming(section_models, batch.section);
            section_payloads[batch_index] =
                buildPackSectionPayloadV13(table, section_models, prototypes, batch.section);
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

                std::vector<runtime::RuntimeModelInstance> section_models = batch.models;
                sortPackSectionModelsForStreaming(section_models, batch.section);
                section_payloads[batch_index] =
                    buildPackSectionPayloadV13(table, section_models, prototypes, batch.section);
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
    const std::vector<codec::ModelSectionBatch>& batches) {
    return buildPackPayloadV12(table, batches);
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
           id == kChunkLine ||            id == kChunkModl || id == kChunkPidx || id == kChunkPack ||
           id == kChunkProt || id == kChunkMemc ||
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
    io::writeU32(out, usePackModels ? kVersionRuntimeV9 : kVersionRuntime);

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
            out, kChunkProt, 8 + static_cast<std::uint32_t>(packBuild.protPayload.size()));
        out.write(
            packBuild.protPayload.data(),
            static_cast<std::streamsize>(packBuild.protPayload.size()));
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
