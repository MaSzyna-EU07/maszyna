#pragma once
#include <string>
enum class logtype : unsigned int {
	shader
};

inline void ErrorLog(const std::string &, logtype const)
{
}
