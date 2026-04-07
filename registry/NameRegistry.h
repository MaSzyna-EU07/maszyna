#pragma once

#include <cstdint>
#include <mutex>
#include <string>
#include <string_view>
#include <unordered_map>

class NameRegistry
{
public:
    static uint64_t GetOrCreate(std::string_view name);
    static const std::string& GetString(uint64_t id);

private:
    static std::unordered_map<uint64_t, std::string>& GetMap();
    static std::mutex& GetMutex();
};