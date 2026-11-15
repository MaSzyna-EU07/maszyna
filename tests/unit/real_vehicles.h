#pragma once

#include <string>
#include <vector>

namespace test {

// the real vehicle definitions of data/fiz, as paths for definition::fiz(): every .json file there, sorted.
// vehicles the game defines by including others are vehicles of their own too (201e-rn, included by 201e-w)
std::vector<std::string> const &real_vehicles();

} // namespace test
