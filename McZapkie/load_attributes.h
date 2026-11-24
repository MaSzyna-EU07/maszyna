#pragma once

#include "core.h"

namespace mover
{
struct load_attributes
{
	std::string name; // name of the cargo
	float offset_min{0.f}; // offset applied to cargo model when load amount is 0

	load_attributes() = default;
	load_attributes(std::string const &Name, float const Offsetmin) : name(Name), offset_min(Offsetmin) {}
};

} // namespace mover
