#pragma once

// EU7B v1 — binarna scena (SCM/SCN + referencje include, bez rozwijania INC).
// Little-endian. Bez NMT, bez mesh/e3d.

#include <eu07/scene/document.hpp>
#include <eu07/scene/include_types.hpp>
#include <eu07/scene/node/track.hpp>

#include <eu07/scene/binary/common.hpp>
#include <eu07/scene/binary/io.hpp>

#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace eu07::scene::binary {

inline constexpr std::uint32_t kVersion = kVersionLegacy;

inline constexpr std::array<char, 4> kChunkIncl{'I', 'N', 'C', 'L'};
inline constexpr std::array<char, 4> kChunkTrak{'T', 'R', 'A', 'K'};

enum class TrackKind : std::uint8_t {
    Normal = 0,
    Switch = 1,
    Road = 2,
    Cross = 3,
    Other = 4,
};

struct LoadedSceneBinary {
    SceneDocument document;
    std::uint32_t version = 0;
    struct ChunkSummary {
        std::array<char, 4> id{};
        std::uint32_t size = 0;
        bool recognized = false;
    };
    std::vector<ChunkSummary> chunks;
};

namespace detail {

inline void writeBezier(std::ostream& out, const TrackBezier& bez) {
    io::writeVec3(out, bez.p1);
    io::writeVec3(out, bez.cv1);
    io::writeVec3(out, bez.cv2);
    io::writeVec3(out, bez.p2);
    io::writeF64(out, bez.roll1);
    io::writeF64(out, bez.roll2);
    io::writeF64(out, bez.radius);
}

inline void writeIncludeRecord(std::ostream& out, const ParsedInclude& inc) {
    io::writeU32(out, static_cast<std::uint32_t>(inc.line));
    io::writeU32(out, static_cast<std::uint32_t>(inc.file.size()));
    out.write(inc.file.data(), static_cast<std::streamsize>(inc.file.size()));
    io::writeU32(out, static_cast<std::uint32_t>(inc.parameters.size()));
    for (const std::string& param : inc.parameters) {
        io::writeU32(out, static_cast<std::uint32_t>(param.size()));
        out.write(param.data(), static_cast<std::streamsize>(param.size()));
    }
}

[[nodiscard]] inline TrackBezier readBezier(std::istream& in) {
    TrackBezier bez;
    bez.p1 = io::readVec3(in);
    bez.cv1 = io::readVec3(in);
    bez.cv2 = io::readVec3(in);
    bez.p2 = io::readVec3(in);
    bez.roll1 = io::readF64(in);
    bez.roll2 = io::readF64(in);
    bez.radius = io::readF64(in);
    return bez;
}

[[nodiscard]] inline ParsedInclude readIncludeRecord(std::istream& in) {
    ParsedInclude inc;
    inc.line = io::readU32(in);
    inc.file = io::readString(in);
    const std::uint32_t paramCount = io::readU32(in);
    inc.parameters.reserve(paramCount);
    for (std::uint32_t i = 0; i < paramCount; ++i) {
        inc.parameters.push_back(io::readString(in));
    }
    inc.expanded = false;
    return inc;
}

template <typename TrackNode>
inline void writeTrackRecord(std::ostream& out, const TrackKind kind, const TrackNode& node) {
    io::writeU8(out, static_cast<std::uint8_t>(kind));
    io::writeU32(out, static_cast<std::uint32_t>(node.header.name.size()));
    out.write(node.header.name.data(), static_cast<std::streamsize>(node.header.name.size()));
    io::writeVec3(out, node.header.originOffset);
    io::writeF64(out, node.length);
    io::writeU32(out, static_cast<std::uint32_t>(node.beziers.size()));
    for (const TrackBezier& bez : node.beziers) {
        writeBezier(out, bez);
    }
}

template <typename TrackNode, typename PushFn>
inline void readTrackBody(std::istream& in, PushFn push) {
    TrackNode node;
    node.header.name = io::readString(in);
    node.header.originOffset = io::readVec3(in);
    node.length = io::readF64(in);
    const std::uint32_t bezCount = io::readU32(in);
    node.beziers.reserve(bezCount);
    for (std::uint32_t i = 0; i < bezCount; ++i) {
        node.beziers.push_back(readBezier(in));
    }
    push(std::move(node));
}

[[nodiscard]] inline std::vector<char> buildInclPayload(const SceneDocument& doc) {
    std::ostringstream inclOut(std::ios::binary);
    io::writeU32(inclOut, static_cast<std::uint32_t>(doc.include.size()));
    for (const ParsedInclude& inc : doc.include) {
        writeIncludeRecord(inclOut, inc);
    }
    const std::string blob = inclOut.str();
    return {blob.begin(), blob.end()};
}

[[nodiscard]] inline std::vector<char> buildTrakPayload(const SceneDocument& doc) {
    std::ostringstream trakOut(std::ios::binary);
    for (const ParsedTrackNormal& node : doc.nodeTrackNormal) {
        writeTrackRecord(trakOut, TrackKind::Normal, node);
    }
    for (const ParsedTrackSwitch& node : doc.nodeTrackSwitch) {
        writeTrackRecord(trakOut, TrackKind::Switch, node);
    }
    for (const ParsedTrackRoad& node : doc.nodeTrackRoad) {
        writeTrackRecord(trakOut, TrackKind::Road, node);
    }
    for (const ParsedTrackCross& node : doc.nodeTrackCross) {
        writeTrackRecord(trakOut, TrackKind::Cross, node);
    }
    for (const ParsedTrackOther& node : doc.nodeTrackOther) {
        writeTrackRecord(trakOut, TrackKind::Other, node);
    }
    const std::string blob = trakOut.str();
    return {blob.begin(), blob.end()};
}

} // namespace detail

inline void writeSceneBinary(const std::filesystem::path& outPath, const SceneDocument& doc) {
    const std::vector<char> inclPayload = detail::buildInclPayload(doc);
    const std::vector<char> trakPayload = detail::buildTrakPayload(doc);

    std::ofstream out(outPath, std::ios::binary | std::ios::trunc);
    if (!out) {
        throw std::runtime_error("Nie mozna zapisac: " + outPath.string());
    }

    out.write(kMagic.data(), 4);
    io::writeU32(out, kVersion);

    io::writeChunkHeader(out, kChunkIncl, 8 + static_cast<std::uint32_t>(inclPayload.size()));
    out.write(inclPayload.data(), static_cast<std::streamsize>(inclPayload.size()));

    io::writeChunkHeader(out, kChunkTrak, 8 + static_cast<std::uint32_t>(trakPayload.size()));
    out.write(trakPayload.data(), static_cast<std::streamsize>(trakPayload.size()));
}

[[nodiscard]] inline LoadedSceneBinary readSceneBinary(const std::filesystem::path& path) {
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
    if (version != kVersion) {
        throw std::runtime_error("EU7B: nieobslugiwana wersja " + std::to_string(version));
    }

    LoadedSceneBinary loaded;
    loaded.document = SceneDocument{};
    loaded.version = version;

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

        LoadedSceneBinary::ChunkSummary chunkSummary;
        chunkSummary.id = chunkId;
        chunkSummary.size = chunkSize;

        if (chunkId == kChunkIncl) {
            chunkSummary.recognized = true;
            const auto chunkStart = in.tellg();
            const std::uint32_t count = io::readU32(in);
            loaded.document.include.reserve(count);
            for (std::uint32_t i = 0; i < count; ++i) {
                loaded.document.include.push_back(detail::readIncludeRecord(in));
            }
            if (in.tellg() - chunkStart != static_cast<std::streamoff>(payloadSize)) {
                throw std::runtime_error("EU7B: uszkodzony chunk INCL");
            }
            loaded.chunks.push_back(chunkSummary);
            continue;
        }

        if (chunkId == kChunkTrak) {
            chunkSummary.recognized = true;
            const auto chunkStart = in.tellg();
            while (in.tellg() - chunkStart < static_cast<std::streamoff>(payloadSize)) {
                switch (static_cast<TrackKind>(io::readU8(in))) {
                case TrackKind::Normal:
                    detail::readTrackBody<ParsedTrackNormal>(in, [&](ParsedTrackNormal node) {
                        loaded.document.nodeTrackNormal.push_back(std::move(node));
                    });
                    break;
                case TrackKind::Switch:
                    detail::readTrackBody<ParsedTrackSwitch>(in, [&](ParsedTrackSwitch node) {
                        loaded.document.nodeTrackSwitch.push_back(std::move(node));
                    });
                    break;
                case TrackKind::Road:
                    detail::readTrackBody<ParsedTrackRoad>(in, [&](ParsedTrackRoad node) {
                        loaded.document.nodeTrackRoad.push_back(std::move(node));
                    });
                    break;
                case TrackKind::Cross:
                    detail::readTrackBody<ParsedTrackCross>(in, [&](ParsedTrackCross node) {
                        loaded.document.nodeTrackCross.push_back(std::move(node));
                    });
                    break;
                case TrackKind::Other:
                    detail::readTrackBody<ParsedTrackOther>(in, [&](ParsedTrackOther node) {
                        loaded.document.nodeTrackOther.push_back(std::move(node));
                    });
                    break;
                default:
                    throw std::runtime_error("EU7B: nieznany typ toru");
                }
            }
            loaded.chunks.push_back(chunkSummary);
            continue;
        }

        in.seekg(static_cast<std::streamoff>(payloadSize), std::ios::cur);
        loaded.chunks.push_back(chunkSummary);
    }

    return loaded;
}

} // namespace eu07::scene::binary
