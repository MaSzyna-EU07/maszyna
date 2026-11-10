#pragma once
#include <iostream>
#include <string>

enum class logtype : unsigned int {
	shader
};

// gl::shader reports #texture/#param parse errors through ErrorLog; the validator counts them as failures
inline bool ErrorLogged = false;

inline void ErrorLog(const std::string &str, logtype const)
{
	std::cerr << str << std::endl;
	ErrorLogged = true;
}
