/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/
#pragma once

// A tiny zero-dependency check framework so the test suite needs nothing but a
// C++20 compiler. Each test executable returns 0 on success, 1 on any failure,
// which is exactly what CTest interprets as pass/fail.

#include <cstdio>

namespace plantest {
inline int checks = 0;
inline int failures = 0;
}  // namespace plantest

// Variadic so that brace-init lists with commas, e.g. CHECK(Point{1, 2} == ...),
// are passed through as a single condition instead of several macro arguments.
#define CHECK(...)                                                           \
    do {                                                                     \
        ++::plantest::checks;                                                \
        if (!(__VA_ARGS__)) {                                                \
            ++::plantest::failures;                                          \
            std::printf("  [FAIL] %s:%d  CHECK(%s)\n", __FILE__, __LINE__,   \
                        #__VA_ARGS__);                                       \
        }                                                                    \
    } while (0)

#define CHECK_THROWS(...)                                                    \
    do {                                                                     \
        ++::plantest::checks;                                                \
        bool threw = false;                                                  \
        try {                                                                \
            (void)(__VA_ARGS__);                                             \
        } catch (...) {                                                      \
            threw = true;                                                    \
        }                                                                    \
        if (!threw) {                                                        \
            ++::plantest::failures;                                          \
            std::printf("  [FAIL] %s:%d  CHECK_THROWS(%s)\n", __FILE__,      \
                        __LINE__, #__VA_ARGS__);                             \
        }                                                                    \
    } while (0)

#define RUN(fn)                              \
    do {                                     \
        std::printf("[RUN ] %s\n", #fn);     \
        fn();                                \
    } while (0)

#define REPORT()                                                             \
    (std::printf("\n%d checks, %d failures\n", ::plantest::checks,           \
                 ::plantest::failures),                                      \
     ::plantest::failures == 0 ? 0 : 1)
