/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstddef>
#include <string>

namespace scene::eu7::bake_parser {

[[nodiscard]] bool
bake_scenario_tree(
    std::string const &TextScenarioPath,
    unsigned MaxThreads,
    std::string &ErrorOut );

// Result of a headless text -> eu7v2 bake (and optional roundtrip verify).
struct Eu7v2BakeReport {
    bool baked { false };          // bake tree completed without throwing
    bool verify_requested { false };
    bool verify_ok { true };       // all per-module record counts matched
    std::size_t module_count { 0 };
    std::size_t model_count { 0 };
    std::string root_binary_path;  // emitted root .eu7v2
    std::string error;
};

// Bakes the whole text scenery tree directly into .eu7v2 modules (no legacy
// .eu7). When Verify is set each emitted module is reloaded and its record
// counts are compared against the source RuntimeModule, with a PASS/FAIL
// report printed to stdout. This is the headless consistency guard.
// MemLimitGb: abort when process private memory exceeds this (0 = no limit).
// Default 50 when called from headless CLI.
// MaxConcurrentParses: cap parallel parseFile/processScene (0 = auto: 1 with MemLimitGb spool).
// MaxThreads 0 with spool = 1 (serial module bake); use --eu7v2-threads N to override.
[[nodiscard]] Eu7v2BakeReport
bake_scenario_tree_eu7v2(
    std::string const &TextScenarioPath,
    unsigned MaxThreads,
    bool Verify,
    unsigned MemLimitGb = 50u,
    unsigned MaxConcurrentParses = 0u,
    unsigned HeavyParseThresholdMb = 0u );

} // namespace scene::eu7::bake_parser
