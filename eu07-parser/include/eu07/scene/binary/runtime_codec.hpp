#pragma once

// EU7B v5 — slim node, TERR terrain-lite, packed vertices, track tail enums.

#include <eu07/scene/binary/io.hpp>
#include <eu07/scene/binary/string_table.hpp>
#include <eu07/scene/match.hpp>
#include <eu07/scene/runtime/basic_node.hpp>
#include <eu07/scene/runtime/nodes.hpp>
#include <eu07/scene/runtime/types.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <limits>
#include <istream>
#include <ostream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace eu07::scene::binary::codec {

namespace detail {

inline constexpr float kLightingEps = 1e-4f;

inline constexpr std::uint8_t kNodeFlagHasName = 1u << 0;
inline constexpr std::uint8_t kNodeFlagHasRangeMin = 1u << 1;
inline constexpr std::uint8_t kNodeFlagHasRangeMax = 1u << 2;
inline constexpr std::uint8_t kNodeFlagHasBounds = 1u << 3;
inline constexpr std::uint8_t kNodeFlagHasGroup = 1u << 4;
inline constexpr std::uint8_t kNodeFlagHasTransform = 1u << 5;
inline constexpr std::uint8_t kNodeFlagNotVisible = 1u << 6;

inline constexpr std::uint8_t kTrackTailCustom = 255;

inline bool hasNonDefaultRangeMax(const double rangeSquaredMax) noexcept {
    return rangeSquaredMax < std::numeric_limits<double>::max();
}

inline bool hasNonEmptyTransform(const runtime::TransformContext& transform) noexcept {
    if (!transform.originStack.empty() || !transform.scaleStack.empty() || transform.groupStackDepth != 0) {
        return true;
    }
    return transform.rotation.x != 0.0 || transform.rotation.y != 0.0 || transform.rotation.z != 0.0;
}

inline void writeTransformContext(std::ostream& out, const runtime::TransformContext& transform) {
    if (transform.originStack.size() > 255 || transform.scaleStack.size() > 255) {
        throw std::runtime_error("EU7B: zbyt gleboki stos transformacji");
    }
    io::writeU8(out, static_cast<std::uint8_t>(transform.originStack.size()));
    for (const runtime::Vec3& offset : transform.originStack) {
        io::writeVec3(out, offset);
    }
    io::writeU8(out, static_cast<std::uint8_t>(transform.scaleStack.size()));
    for (const runtime::Vec3& scale : transform.scaleStack) {
        io::writeVec3(out, scale);
    }
    io::writeVec3(out, transform.rotation);
    io::writeU8(out, static_cast<std::uint8_t>(transform.groupStackDepth));
}

inline runtime::TransformContext readTransformContext(std::istream& in) {
    runtime::TransformContext transform;
    const std::uint8_t originCount = io::readU8(in);
    transform.originStack.reserve(originCount);
    for (std::uint8_t i = 0; i < originCount; ++i) {
        transform.originStack.push_back(io::readVec3(in));
    }
    const std::uint8_t scaleCount = io::readU8(in);
    transform.scaleStack.reserve(scaleCount);
    for (std::uint8_t i = 0; i < scaleCount; ++i) {
        transform.scaleStack.push_back(io::readVec3(in));
    }
    transform.rotation = io::readVec3(in);
    transform.groupStackDepth = io::readU8(in);
    return transform;
}

inline void writeStringId(std::ostream& out, StringTable& table, const std::string& text) {
    io::writeU32(out, table.intern(text));
}

inline std::int16_t floatToSnorm16(const float value) {
    const float clamped = std::clamp(value, -1.f, 1.f);
    return static_cast<std::int16_t>(std::lround(clamped * 32767.f));
}

inline float snorm16ToFloat(const std::int16_t value) {
    return static_cast<float>(value) * (1.f / 32767.f);
}

inline std::uint16_t floatToHalf(const float value) {
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));

    const std::uint32_t sign = (bits >> 16u) & 0x8000u;
    const std::uint32_t fExp = (bits >> 23u) & 0xFFu;
    std::uint32_t mantissa = bits & 0x7FFFFFu;

    if (fExp == 0xFFu) {
        return static_cast<std::uint16_t>(sign | 0x7C00u | (mantissa ? 0x0200u : 0u));
    }
    if (fExp == 0) {
        return static_cast<std::uint16_t>(sign);
    }

    std::int32_t halfExp = static_cast<std::int32_t>(fExp) - 127 + 15;
    if (halfExp >= 31) {
        return static_cast<std::uint16_t>(sign | 0x7C00u);
    }
    if (halfExp <= 0) {
        if (halfExp < -10) {
            return static_cast<std::uint16_t>(sign);
        }
        mantissa = (mantissa | 0x800000u) >> static_cast<std::uint32_t>(1 - halfExp);
        return static_cast<std::uint16_t>(sign | (mantissa >> 13u));
    }

    return static_cast<std::uint16_t>(
        sign | (static_cast<std::uint32_t>(halfExp) << 10u) | ((mantissa + 0x1000u) >> 13u));
}

inline float halfToFloat(const std::uint16_t value) {
    const std::uint32_t sign = static_cast<std::uint32_t>(value & 0x8000u) << 16u;
    const std::uint32_t exponent = (value & 0x7C00u) >> 10u;
    std::uint32_t mantissa = value & 0x03FFu;

    std::uint32_t bits = 0;
    if (exponent == 0) {
        if (mantissa == 0) {
            bits = sign;
        } else {
            std::uint32_t exp = 127 - 15 - 1;
            while ((mantissa & 0x400u) == 0) {
                mantissa <<= 1u;
                --exp;
            }
            mantissa &= 0x3FFu;
            bits = sign | (exp << 23u) | (mantissa << 13u);
        }
    } else if (exponent == 31) {
        bits = sign | 0x7F800000u | (mantissa << 13u);
    } else {
        bits = sign | ((exponent + 127 - 15) << 23u) | (mantissa << 13u);
    }

    float result = 0.f;
    std::memcpy(&result, &bits, sizeof(result));
    return result;
}

inline void writeHalf(std::ostream& out, const float value) {
    io::writeU16(out, floatToHalf(value));
}

inline float readHalf(std::istream& in) {
    return halfToFloat(io::readU16(in));
}

} // namespace detail

inline void writeStringId(std::ostream& out, StringTable& table, const std::string& text) {
    detail::writeStringId(out, table, text);
}

inline void writeSlimNode(
    std::ostream& out,
    StringTable& table,
    const runtime::BasicNode& node,
    const std::string_view impliedType) {
    std::uint8_t flags = 0;
    if (!node.name.empty()) {
        flags |= detail::kNodeFlagHasName;
    }
    if (node.rangeSquaredMin != 0.0) {
        flags |= detail::kNodeFlagHasRangeMin;
    }
    if (detail::hasNonDefaultRangeMax(node.rangeSquaredMax)) {
        flags |= detail::kNodeFlagHasRangeMax;
    }
    if (node.area.radius >= 0.f) {
        flags |= detail::kNodeFlagHasBounds;
    }
    if (node.groupValid) {
        flags |= detail::kNodeFlagHasGroup;
    }
    if (detail::hasNonEmptyTransform(node.transform)) {
        flags |= detail::kNodeFlagHasTransform;
    }
    if (!node.visible) {
        flags |= detail::kNodeFlagNotVisible;
    }

    io::writeU8(out, flags);
    if ((flags & detail::kNodeFlagHasName) != 0) {
        detail::writeStringId(out, table, node.name);
    }
    if ((flags & detail::kNodeFlagHasRangeMin) != 0) {
        io::writeF64(out, node.rangeSquaredMin);
    }
    if ((flags & detail::kNodeFlagHasRangeMax) != 0) {
        io::writeF64(out, node.rangeSquaredMax);
    }
    if ((flags & detail::kNodeFlagHasBounds) != 0) {
        io::writeVec3(out, node.area.center);
        io::writeF32(out, node.area.radius);
    }
    if ((flags & detail::kNodeFlagHasGroup) != 0) {
        io::writeU32(out, static_cast<std::uint32_t>(node.groupHandle));
    }
    if ((flags & detail::kNodeFlagHasTransform) != 0) {
        detail::writeTransformContext(out, node.transform);
    }
    (void)impliedType;
}

inline runtime::BasicNode readSlimNode(std::istream& in, const StringTable& table, const std::string_view impliedType) {
    runtime::BasicNode node;
    node.nodeType = std::string(impliedType);
    const std::uint8_t flags = io::readU8(in);
    if ((flags & detail::kNodeFlagHasName) != 0) {
        node.name = table.resolve(io::readU32(in));
    }
    if ((flags & detail::kNodeFlagHasRangeMin) != 0) {
        node.rangeSquaredMin = io::readF64(in);
    }
    if ((flags & detail::kNodeFlagHasRangeMax) != 0) {
        node.rangeSquaredMax = io::readF64(in);
    } else {
        node.rangeSquaredMax = std::numeric_limits<double>::max();
    }
    if ((flags & detail::kNodeFlagHasBounds) != 0) {
        node.area.center = io::readVec3(in);
        node.area.radius = io::readF32(in);
    }
    if ((flags & detail::kNodeFlagHasGroup) != 0) {
        node.groupValid = true;
        node.groupHandle = io::readU32(in);
    }
    if ((flags & detail::kNodeFlagHasTransform) != 0) {
        node.transform = detail::readTransformContext(in);
    }
    node.visible = (flags & detail::kNodeFlagNotVisible) == 0;
    return node;
}

[[nodiscard]] inline bool isDefaultLighting(const runtime::LightingData& lighting) noexcept {
    const auto near = [&](const float a, const float b) noexcept {
        return std::abs(a - b) <= detail::kLightingEps;
    };
    return near(lighting.diffuse.x, 0.8f) && near(lighting.diffuse.y, 0.8f) &&
           near(lighting.diffuse.z, 0.8f) && near(lighting.diffuse.w, 1.f) &&
           near(lighting.ambient.x, 0.2f) && near(lighting.ambient.y, 0.2f) &&
           near(lighting.ambient.z, 0.2f) && near(lighting.ambient.w, 1.f) &&
           near(lighting.specular.x, 0.f) && near(lighting.specular.y, 0.f) &&
           near(lighting.specular.z, 0.f) && near(lighting.specular.w, 1.f);
}

inline void writeLightingBlock(std::ostream& out, const runtime::LightingData& lighting) {
    io::writeF32(out, lighting.diffuse.x);
    io::writeF32(out, lighting.diffuse.y);
    io::writeF32(out, lighting.diffuse.z);
    io::writeF32(out, lighting.diffuse.w);
    io::writeF32(out, lighting.ambient.x);
    io::writeF32(out, lighting.ambient.y);
    io::writeF32(out, lighting.ambient.z);
    io::writeF32(out, lighting.ambient.w);
    io::writeF32(out, lighting.specular.x);
    io::writeF32(out, lighting.specular.y);
    io::writeF32(out, lighting.specular.z);
    io::writeF32(out, lighting.specular.w);
}

inline runtime::LightingData readLightingBlock(std::istream& in) {
    runtime::LightingData lighting;
    lighting.diffuse.x = io::readF32(in);
    lighting.diffuse.y = io::readF32(in);
    lighting.diffuse.z = io::readF32(in);
    lighting.diffuse.w = io::readF32(in);
    lighting.ambient.x = io::readF32(in);
    lighting.ambient.y = io::readF32(in);
    lighting.ambient.z = io::readF32(in);
    lighting.ambient.w = io::readF32(in);
    lighting.specular.x = io::readF32(in);
    lighting.specular.y = io::readF32(in);
    lighting.specular.z = io::readF32(in);
    lighting.specular.w = io::readF32(in);
    return lighting;
}

inline void writeLightingTagged(std::ostream& out, const runtime::LightingData& lighting) {
    io::writeU8(out, isDefaultLighting(lighting) ? 0 : 1);
    if (!isDefaultLighting(lighting)) {
        writeLightingBlock(out, lighting);
    }
}

inline runtime::LightingData readLightingTagged(std::istream& in) {
    if (io::readU8(in) == 0) {
        return runtime::LightingData{};
    }
    return readLightingBlock(in);
}

inline void writePackedWorldVertex(std::ostream& out, const runtime::WorldVertex& vertex) {
    io::writeVec3(out, vertex.position);
    io::writeI16(out, detail::floatToSnorm16(static_cast<float>(vertex.normal.x)));
    io::writeI16(out, detail::floatToSnorm16(static_cast<float>(vertex.normal.y)));
    io::writeI16(out, detail::floatToSnorm16(static_cast<float>(vertex.normal.z)));
    detail::writeHalf(out, static_cast<float>(vertex.u));
    detail::writeHalf(out, static_cast<float>(vertex.v));
}

inline runtime::WorldVertex readPackedWorldVertex(std::istream& in) {
    runtime::WorldVertex vertex;
    vertex.position = io::readVec3(in);
    vertex.normal.x = detail::snorm16ToFloat(io::readI16(in));
    vertex.normal.y = detail::snorm16ToFloat(io::readI16(in));
    vertex.normal.z = detail::snorm16ToFloat(io::readI16(in));
    vertex.u = detail::readHalf(in);
    vertex.v = detail::readHalf(in);
    return vertex;
}

[[nodiscard]] inline std::uint8_t trackTailKeywordCode(const std::string_view key) noexcept {
    if (isKeyword(key, "event0")) {
        return 1;
    }
    if (isKeyword(key, "eventall0")) {
        return 2;
    }
    if (isKeyword(key, "event1")) {
        return 3;
    }
    if (isKeyword(key, "eventall1")) {
        return 4;
    }
    if (isKeyword(key, "event2")) {
        return 5;
    }
    if (isKeyword(key, "eventall2")) {
        return 6;
    }
    if (isKeyword(key, "velocity")) {
        return 7;
    }
    if (isKeyword(key, "isolated")) {
        return 8;
    }
    if (isKeyword(key, "overhead")) {
        return 9;
    }
    if (isKeyword(key, "vradius")) {
        return 10;
    }
    if (isKeyword(key, "railprofile")) {
        return 11;
    }
    if (isKeyword(key, "trackbed")) {
        return 12;
    }
    if (isKeyword(key, "friction")) {
        return 13;
    }
    if (isKeyword(key, "fouling1")) {
        return 14;
    }
    if (isKeyword(key, "fouling2")) {
        return 15;
    }
    if (isKeyword(key, "sleepermodel")) {
        return 16;
    }
    if (isKeyword(key, "angle1")) {
        return 17;
    }
    if (isKeyword(key, "angle2")) {
        return 18;
    }
    return detail::kTrackTailCustom;
}

[[nodiscard]] inline std::string trackTailKeywordName(const std::uint8_t code) {
    switch (code) {
    case 1:
        return "event0";
    case 2:
        return "eventall0";
    case 3:
        return "event1";
    case 4:
        return "eventall1";
    case 5:
        return "event2";
    case 6:
        return "eventall2";
    case 7:
        return "velocity";
    case 8:
        return "isolated";
    case 9:
        return "overhead";
    case 10:
        return "vradius";
    case 11:
        return "railprofile";
    case 12:
        return "trackbed";
    case 13:
        return "friction";
    case 14:
        return "fouling1";
    case 15:
        return "fouling2";
    case 16:
        return "sleepermodel";
    case 17:
        return "angle1";
    case 18:
        return "angle2";
    default:
        return {};
    }
}

inline void writeTrackTailEntry(
    std::ostream& out,
    StringTable& table,
    const std::string& key,
    const std::string& value) {
    const std::uint8_t code = trackTailKeywordCode(key);
    io::writeU8(out, code);
    if (code == detail::kTrackTailCustom) {
        detail::writeStringId(out, table, key);
    }
    detail::writeStringId(out, table, value);
}

[[nodiscard]] inline std::uint8_t meshSubtypeCode(const std::string_view type) noexcept {
    if (isKeyword(type, "triangle_strip")) {
        return 1;
    }
    if (isKeyword(type, "triangle_fan")) {
        return 2;
    }
    return 0;
}

[[nodiscard]] inline std::string_view meshSubtypeName(const std::uint8_t code) noexcept {
    switch (code) {
    case 1:
        return "triangle_strip";
    case 2:
        return "triangle_fan";
    default:
        return "triangles";
    }
}

[[nodiscard]] inline std::uint8_t lineSubtypeCode(const std::string_view type) noexcept {
    if (isKeyword(type, "line_strip")) {
        return 1;
    }
    if (isKeyword(type, "line_loop")) {
        return 2;
    }
    return 0;
}

[[nodiscard]] inline std::string_view lineSubtypeName(const std::uint8_t code) noexcept {
    switch (code) {
    case 1:
        return "line_strip";
    case 2:
        return "line_loop";
    default:
        return "lines";
    }
}

inline std::pair<std::string, std::string> readTrackTailEntry(std::istream& in, const StringTable& table) {
    const std::uint8_t code = io::readU8(in);
    std::string key;
    if (code == detail::kTrackTailCustom) {
        key = table.resolve(io::readU32(in));
    } else {
        key = trackTailKeywordName(code);
    }
    return {std::move(key), table.resolve(io::readU32(in))};
}

// --- TERR: jednorodny teren trójkątowy (materiał/lighting w nagłówku chunka) ---

inline constexpr std::uint8_t kTerrFlagTranslucent = 1u << 0;
inline constexpr std::uint8_t kTerrFlagNonDefaultLighting = 1u << 1;
inline constexpr std::uint8_t kTerrFlagBatched = 1u << 2;
inline constexpr std::size_t kTerrVertsPerRecord = 3;

// basic_region::section() w maszyna-fresh/scene/scene.cpp
inline constexpr std::int32_t kEu07SectionSize = 1000;
inline constexpr std::int32_t kEu07RegionSideSectionCount = 500;

struct TerrSectionKey {
    std::int32_t x = 0;
    std::int32_t z = 0;

    [[nodiscard]] bool operator==(const TerrSectionKey& other) const noexcept {
        return x == other.x && z == other.z;
    }
};

struct TerrSectionKeyHash {
    [[nodiscard]] std::size_t operator()(const TerrSectionKey& key) const noexcept {
        return static_cast<std::size_t>(key.x) * 0x9E3779B9u ^
               static_cast<std::size_t>(key.z);
    }
};

struct TerrSectionBatch {
    TerrSectionKey section{};
    std::vector<runtime::WorldVertex> vertices;
};

[[nodiscard]] inline runtime::Vec3 terrTriangleCentroid(
    const std::array<runtime::WorldVertex, kTerrVertsPerRecord>& triangle) noexcept {
    runtime::Vec3 center{};
    for (const runtime::WorldVertex& vertex : triangle) {
        center.x += vertex.position.x;
        center.y += vertex.position.y;
        center.z += vertex.position.z;
    }
    const double inv = 1.0 / static_cast<double>(kTerrVertsPerRecord);
    center.x *= inv;
    center.y *= inv;
    center.z *= inv;
    return center;
}

[[nodiscard]] inline TerrSectionKey terrSectionKey(const runtime::Vec3& centroid) noexcept {
    TerrSectionKey key;
    key.x = static_cast<std::int32_t>(std::floor(
        centroid.x / static_cast<double>(kEu07SectionSize) +
        static_cast<double>(kEu07RegionSideSectionCount) / 2.0));
    key.z = static_cast<std::int32_t>(std::floor(
        centroid.z / static_cast<double>(kEu07SectionSize) +
        static_cast<double>(kEu07RegionSideSectionCount) / 2.0));
    return key;
}

[[nodiscard]] inline bool nearZeroVec3(const runtime::Vec3& v, const double eps = 1e-9) noexcept {
    return std::abs(v.x) <= eps && std::abs(v.y) <= eps && std::abs(v.z) <= eps;
}

[[nodiscard]] inline bool lightingEqual(
    const runtime::LightingData& a,
    const runtime::LightingData& b) noexcept {
    const auto near = [&](const float x, const float y) noexcept {
        return std::abs(x - y) <= detail::kLightingEps;
    };
    return near(a.diffuse.x, b.diffuse.x) && near(a.diffuse.y, b.diffuse.y) &&
           near(a.diffuse.z, b.diffuse.z) && near(a.diffuse.w, b.diffuse.w) &&
           near(a.ambient.x, b.ambient.x) && near(a.ambient.y, b.ambient.y) &&
           near(a.ambient.z, b.ambient.z) && near(a.ambient.w, b.ambient.w) &&
           near(a.specular.x, b.specular.x) && near(a.specular.y, b.specular.y) &&
           near(a.specular.z, b.specular.z) && near(a.specular.w, b.specular.w);
}

[[nodiscard]] inline bool terrMetadataMatches(
    const runtime::RuntimeShapeNode& a,
    const runtime::RuntimeShapeNode& b) noexcept {
    return a.materialPath == b.materialPath && a.translucent == b.translucent &&
           lightingEqual(a.lighting, b.lighting);
}

[[nodiscard]] inline bool isTerrTriangleLite(const runtime::RuntimeShapeNode& shape) noexcept {
    const std::string_view nodeType =
        shape.node.nodeType.empty() ? std::string_view{"triangles"} : std::string_view{shape.node.nodeType};
    if (meshSubtypeCode(nodeType) != 0) {
        return false;
    }
    if (shape.vertices.size() != kTerrVertsPerRecord) {
        return false;
    }
    if (!nearZeroVec3(shape.origin)) {
        return false;
    }
    if (!shape.node.name.empty()) {
        return false;
    }
    if (shape.node.rangeSquaredMin != 0.0) {
        return false;
    }
    if (detail::hasNonDefaultRangeMax(shape.node.rangeSquaredMax)) {
        return false;
    }
    if (!shape.node.visible) {
        return false;
    }
    if (shape.node.groupValid) {
        return false;
    }
    if (detail::hasNonEmptyTransform(shape.node.transform)) {
        return false;
    }
    return true;
}

[[nodiscard]] inline bool canUseTerrEncoding(
    const std::vector<runtime::RuntimeShapeNode>& shapes) noexcept {
    if (shapes.empty()) {
        return false;
    }
    if (!isTerrTriangleLite(shapes.front())) {
        return false;
    }
    for (std::size_t i = 1; i < shapes.size(); ++i) {
        if (!isTerrTriangleLite(shapes[i])) {
            return false;
        }
        if (!terrMetadataMatches(shapes.front(), shapes[i])) {
            return false;
        }
    }
    return true;
}

[[nodiscard]] inline std::uint8_t terrFlagsForShape(
    const runtime::RuntimeShapeNode& shape,
    const bool batched = false) noexcept {
    std::uint8_t flags = 0;
    if (shape.translucent) {
        flags |= kTerrFlagTranslucent;
    }
    if (!isDefaultLighting(shape.lighting)) {
        flags |= kTerrFlagNonDefaultLighting;
    }
    if (batched) {
        flags |= kTerrFlagBatched;
    }
    return flags;
}

[[nodiscard]] inline std::vector<TerrSectionBatch> groupTerrShapesBySection(
    const std::vector<runtime::RuntimeShapeNode>& shapes) {
    std::unordered_map<TerrSectionKey, std::size_t, TerrSectionKeyHash> batchIndex;
    std::vector<TerrSectionBatch> batches;
    batches.reserve(256);

    for (const runtime::RuntimeShapeNode& shape : shapes) {
        if (shape.vertices.size() != kTerrVertsPerRecord) {
            throw std::runtime_error("EU7B TERR: oczekiwano 3 wierzcholkow na trojkat");
        }
        std::array<runtime::WorldVertex, kTerrVertsPerRecord> triangle{};
        for (std::size_t i = 0; i < kTerrVertsPerRecord; ++i) {
            triangle[i] = shape.vertices[i];
        }
        const TerrSectionKey key = terrSectionKey(terrTriangleCentroid(triangle));
        const auto [it, inserted] = batchIndex.emplace(key, batches.size());
        if (inserted) {
            TerrSectionBatch batch;
            batch.section = key;
            batches.push_back(std::move(batch));
        }
        TerrSectionBatch& batch = batches[it->second];
        batch.vertices.insert(
            batch.vertices.end(),
            shape.vertices.begin(),
            shape.vertices.end());
    }

    std::sort(
        batches.begin(),
        batches.end(),
        [](const TerrSectionBatch& a, const TerrSectionBatch& b) noexcept {
            if (a.section.z != b.section.z) {
                return a.section.z < b.section.z;
            }
            return a.section.x < b.section.x;
        });
    return batches;
}

struct ModelSectionBatch {
    TerrSectionKey section{};
    std::vector<runtime::RuntimeModelInstance> models;
};

[[nodiscard]] inline TerrSectionKey clampTerrSectionKey(const TerrSectionKey key) noexcept {
    TerrSectionKey clamped = key;
    clamped.x = std::clamp(
        clamped.x,
        0,
        static_cast<std::int32_t>(kEu07RegionSideSectionCount) - 1);
    clamped.z = std::clamp(
        clamped.z,
        0,
        static_cast<std::int32_t>(kEu07RegionSideSectionCount) - 1);
    return clamped;
}

[[nodiscard]] inline std::vector<ModelSectionBatch> groupModelsBySection(
    const std::vector<runtime::RuntimeModelInstance>& models) {
    std::unordered_map<TerrSectionKey, std::size_t, TerrSectionKeyHash> batchIndex;
    std::vector<ModelSectionBatch> batches;
    batches.reserve(256);

    for (const runtime::RuntimeModelInstance& model : models) {
        const TerrSectionKey key = clampTerrSectionKey(terrSectionKey(model.location));
        const auto [it, inserted] = batchIndex.emplace(key, batches.size());
        if (inserted) {
            ModelSectionBatch batch;
            batch.section = key;
            batches.push_back(std::move(batch));
        }
        batches[it->second].models.push_back(model);
    }

    std::sort(
        batches.begin(),
        batches.end(),
        [](const ModelSectionBatch& a, const ModelSectionBatch& b) noexcept {
            if (a.section.z != b.section.z) {
                return a.section.z < b.section.z;
            }
            return a.section.x < b.section.x;
        });
    return batches;
}

} // namespace eu07::scene::binary::codec
