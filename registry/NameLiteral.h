#pragma once

#include <cstddef>

#include "FName.h"
#include "NameHash.h"

consteval FName operator"" _name(const char* str, std::size_t len)
{
    return FName{ HashLiteral(str, len) };
}