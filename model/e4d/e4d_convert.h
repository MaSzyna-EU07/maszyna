/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// the conversion of a model loaded from E3D or T3D into E4D, and the -e4d commands of the program

#include "model/e4d/e4d_model.h"

#include <string>
#include <vector>

class TModel3d;

namespace e4d {

// the E4D form of the loaded model, its geometry taken from the renderer's banks (as SaveToBinFile does). the name index
// and the bounds are filled in; the source hash is left to the caller. the faults of the source found on the way, which
// leave the result invalid, are added to Faults, each naming its submodel
model convert(TModel3d &Source, std::vector<std::string> &Faults);

// the -e4d commands: convert, dump, validate. returns the exit code of the program
int run_command_line(int Argc, char *Argv[]);

} // namespace e4d
