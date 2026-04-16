#pragma once
#include <array>
#include <random>
#include <sstream>
#include <iomanip>
#include <string>
#include <algorithm>
#include <cstdint>

class UID {
public:
    std::array<uint8_t,16> bytes;


    static UID random() {
        static thread_local std::mt19937_64 gen(std::random_device{}());
        UID u;
        uint64_t a = gen();
        uint64_t b = gen();
        for (int i = 0; i < 8; ++i) u.bytes[i] = uint8_t((a >> (i*8)) & 0xFF);
        for (int i = 0; i < 8; ++i) u.bytes[8 + i] = uint8_t((b >> (i*8)) & 0xFF);

        u.bytes[6] = (u.bytes[6] & 0x0F) | 0x40;
        u.bytes[8] = (u.bytes[8] & 0x3F) | 0x80;

        return u;
    }

    std::string to_string() const {
        std::ostringstream os;
        os << std::hex << std::setfill('0');
        auto put = [&](int i){
            os << std::setw(2) << static_cast<int>(bytes[i]);
        };
        // format 8-4-4-4-12
        for (int i = 0; i < 4; ++i) put(i);
        for (int i = 4; i < 6; ++i) put(i);
        os << '-';
        for (int i = 6; i < 8; ++i) put(i);
        os << '-';
        for (int i = 8; i < 10; ++i) put(i);
        os << '-';
        for (int i = 10; i < 12; ++i) put(i);
        os << '-';
        for (int i = 12; i < 16; ++i) put(i);
        return os.str();
    }

    static UID from_string(const std::string& str) {
        std::istringstream is(str);
        is >> std::hex;
        UID u;
        for (int i = 0; i < 16; ++i) {
            int byte;
            is >> byte;
            u.bytes[i] = static_cast<uint8_t>(byte);
            if (is.peek() == '-') is.get();
        }
        return u;
    }

    bool operator==(const UID &other) const noexcept {
        return bytes == other.bytes;
    }

    bool operator!=(const UID &other) const noexcept {
        return !(*this == other);
    }
};