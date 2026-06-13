#pragma once

#include <eu07/scene/node/types.hpp>

#include <array>
#include <cstdint>
#include <cstring>
#include <istream>
#include <ostream>
#include <stdexcept>
#include <string>

namespace eu07::scene::binary::io {

inline void writeU8(std::ostream& out, const std::uint8_t v) {
    out.put(static_cast<char>(v));
}

inline void writeU16(std::ostream& out, const std::uint16_t v) {
    const auto b = std::array<std::uint8_t, 2>{
        static_cast<std::uint8_t>(v),
        static_cast<std::uint8_t>(v >> 8),
    };
    out.write(reinterpret_cast<const char*>(b.data()), 2);
}

inline void writeU32(std::ostream& out, const std::uint32_t v) {
    const auto b = std::array<std::uint8_t, 4>{
        static_cast<std::uint8_t>(v),
        static_cast<std::uint8_t>(v >> 8),
        static_cast<std::uint8_t>(v >> 16),
        static_cast<std::uint8_t>(v >> 24),
    };
    out.write(reinterpret_cast<const char*>(b.data()), 4);
}

inline void writeU64(std::ostream& out, const std::uint64_t v) {
    writeU32(out, static_cast<std::uint32_t>(v));
    writeU32(out, static_cast<std::uint32_t>(v >> 32));
}

inline void writeI32(std::ostream& out, const std::int32_t v) {
    writeU32(out, static_cast<std::uint32_t>(v));
}

inline void writeI16(std::ostream& out, const std::int16_t v) {
    writeU16(out, static_cast<std::uint16_t>(v));
}

inline void writeF32(std::ostream& out, const float v) {
    static_assert(sizeof(float) == 4);
    std::uint32_t bits = 0;
    std::memcpy(&bits, &v, 4);
    writeU32(out, bits);
}

// Skalary i Vec3 w EU7B v3+ — float32 na dysku (runtime w pamieci nadal double).
inline void writeF64(std::ostream& out, const double v) {
    writeF32(out, static_cast<float>(v));
}

inline void writeVec3(std::ostream& out, const Vec3& v) {
    writeF64(out, v.x);
    writeF64(out, v.y);
    writeF64(out, v.z);
}

inline void writeChunkHeader(std::ostream& out, const std::array<char, 4>& id, const std::uint32_t size) {
    out.write(id.data(), 4);
    writeU32(out, size);
}

[[nodiscard]] inline std::uint8_t readU8(std::istream& in) {
    const int ch = in.get();
    if (ch == EOF) {
        throw std::runtime_error("EU7B: nieoczekiwany koniec pliku");
    }
    return static_cast<std::uint8_t>(ch);
}

[[nodiscard]] inline std::uint32_t readU32(std::istream& in) {
    std::array<std::uint8_t, 4> b{};
    in.read(reinterpret_cast<char*>(b.data()), 4);
    if (!in) {
        throw std::runtime_error("EU7B: nieoczekiwany koniec pliku");
    }
    return static_cast<std::uint32_t>(b[0]) | (static_cast<std::uint32_t>(b[1]) << 8) |
           (static_cast<std::uint32_t>(b[2]) << 16) | (static_cast<std::uint32_t>(b[3]) << 24);
}

[[nodiscard]] inline std::uint64_t readU64(std::istream& in) {
    const std::uint64_t lo = readU32(in);
    const std::uint64_t hi = readU32(in);
    return lo | (hi << 32);
}

[[nodiscard]] inline std::int32_t readI32(std::istream& in) {
    return static_cast<std::int32_t>(readU32(in));
}

[[nodiscard]] inline std::uint16_t readU16(std::istream& in) {
    std::array<std::uint8_t, 2> b{};
    in.read(reinterpret_cast<char*>(b.data()), 2);
    if (!in) {
        throw std::runtime_error("EU7B: nieoczekiwany koniec pliku");
    }
    return static_cast<std::uint16_t>(b[0]) | (static_cast<std::uint16_t>(b[1]) << 8);
}

[[nodiscard]] inline std::int16_t readI16(std::istream& in) {
    return static_cast<std::int16_t>(readU16(in));
}

[[nodiscard]] inline float readF32(std::istream& in) {
    const std::uint32_t bits = readU32(in);
    float v = 0.f;
    std::memcpy(&v, &bits, 4);
    return v;
}

[[nodiscard]] inline double readF64(std::istream& in) {
    return static_cast<double>(readF32(in));
}

[[nodiscard]] inline Vec3 readVec3(std::istream& in) {
    return {readF64(in), readF64(in), readF64(in)};
}

[[nodiscard]] inline std::string readString(std::istream& in) {
    const std::uint32_t len = readU32(in);
    std::string text(len, '\0');
    if (len > 0) {
        in.read(text.data(), static_cast<std::streamsize>(len));
        if (!in) {
            throw std::runtime_error("EU7B: nieoczekiwany koniec pliku (string)");
        }
    }
    return text;
}

} // namespace eu07::scene::binary::io
