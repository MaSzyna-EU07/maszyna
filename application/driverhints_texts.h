#pragma once

// The hint texts are built by re-expanding the X-macro list with a different
// DRIVER_HINT_DEF. A module cannot do that on a consumer's behalf -- the macro
// state would not cross the boundary -- so this stays a plain header, included
// in the global module fragment of the single translation unit that needs it.

#define DRIVER_HINT_DEF(a, b) b,

const char *driver_hints_texts[] =
{
    #include "application/driverhints_def.h"
};

#undef DRIVER_HINT_DEF
