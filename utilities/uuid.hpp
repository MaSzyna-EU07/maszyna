#pragma once
#include <array>
#include <random>
#include <sstream>
#include <iomanip>
#include <string>
#include <algorithm>
#include <cstdint>
#include "utilities/utilities.h"
#include "utilities/Globals.h"

class UID {
public:
    std::array<uint8_t,16> bytes;


    static UID random() {
        UID u;
        uint64_t a = Global.random_engine();
        uint64_t b = Global.random_engine();
        for (int i = 0; i < 8; ++i) u.bytes[i] = uint8_t(a >> (i * 8) & 0xFF);
        for (int i = 0; i < 8; ++i) u.bytes[8 + i] = uint8_t(b >> (i * 8) & 0xFF);

        u.bytes[6] = u.bytes[6] & 0x0F | 0x40;
        u.bytes[8] = u.bytes[8] & 0x3F | 0x80;

        return u;
    }

    std::string to_string() const {
        // NOTE: done by hand as the text is made for each model instance of a scenery, and a string stream took a few microseconds each
        static constexpr char digits[] = "0123456789abcdef";
        std::string text;
        text.reserve(36);
        // format 12-4-4-4-8, the one node names in scenery files were written with
        for (int i = 0; i < 16; ++i) {
            if (i == 6 || i == 8 || i == 10 || i == 12) text += '-';
            text += digits[bytes[i] >> 4];
            text += digits[bytes[i] & 0x0F];
        }
        return text;
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