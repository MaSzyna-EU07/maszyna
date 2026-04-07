#include "NameRegistry.h"
#include "NameHash.h"

#include <cassert>

uint64_t NameRegistry::GetOrCreate(std::string_view name)
{
    if (name.empty())
        return 0;

    const uint64_t hash = HashRuntime(name);

    std::lock_guard<std::mutex> lock(GetMutex());
    auto& map = GetMap();

    auto [it, inserted] = map.emplace(hash, std::string(name));
    if (!inserted)
    {
        assert(it->second == name && "Name hash collision detected");
    }

    return hash;
}

const std::string& NameRegistry::GetString(uint64_t id)
{
    static const std::string Empty;

    if (id == 0)
        return Empty;

    std::lock_guard<std::mutex> lock(GetMutex());
    auto& map = GetMap();

    auto it = map.find(id);
    if (it != map.end())
        return it->second;

    return Empty;
}

std::unordered_map<uint64_t, std::string>& NameRegistry::GetMap()
{
    static std::unordered_map<uint64_t, std::string> map;
    return map;
}

std::mutex& NameRegistry::GetMutex()
{
    static std::mutex mutex;
    return mutex;
}