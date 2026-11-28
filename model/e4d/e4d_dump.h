/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// the content of an E4D model as text, for people and for comparing two files

#include "model/e4d/e4d_model.h"

#include <string>

namespace e4d {

std::string dump(model const &Model);

std::string fourcc_name(std::uint32_t Id);

} // namespace e4d
