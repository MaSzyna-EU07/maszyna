#pragma once

// Macros cannot cross a module boundary, so they live in a plain header
// that consumers pull into their global module fragment. The bodies are
// expanded at the use site, where the imported names are in scope, so this
// header deliberately includes nothing.

#define STR(x) Translations.lookup_s(x, false)
#define STR_C(x) Translations.lookup_c(x, true)
#define STRN(x) x
