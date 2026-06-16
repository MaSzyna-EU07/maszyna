#pragma once
#include <string>

class ECWorld;

namespace ecs_persistence
{
    // Save all named entities to a JSON file. Returns number of entities saved, or -1 on error.
    int save(ECWorld &world, const std::string &path);
    // Load entities from a JSON file. Skips entities whose name already exists. Returns count loaded, or -1 on error.
    int load(ECWorld &world, const std::string &path);
}
