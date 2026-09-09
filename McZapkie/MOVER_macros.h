#pragma once

// Macros cannot cross a module boundary, so they live in a plain header
// that consumers pull into their global module fragment. The bodies are
// expanded at the use site, where the imported names are in scope, so this
// header deliberately includes nothing.

#define p_elengproblem (1e-02)
#define p_coupldmg (2e-03)
#define p_accn (1e-01)
#define p_slippdmg (1e-03)
