#pragma once

// Macros cannot cross a module boundary, so they live in a plain header that
// consumers pull into their global module fragment. The bodies expand at the use
// site, where the imported names are in scope, so this header includes nothing.

#define PyGetFloat(param) PyFloat_FromDouble(param)
#define PyGetBool(param) param ? Py_True : Py_False
