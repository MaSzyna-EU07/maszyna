#pragma once

// A tiny zero-dependency check framework so the test suite needs nothing but a
// C++20 compiler. Each test executable returns 0 on success, 1 on any failure,
// which is exactly what CTest interprets as pass/fail.

#include <cstdio>

namespace maj0test {
inline int checks = 0;
inline int failures = 0;
}  // namespace maj0test

// Variadic so that brace-init lists with commas, e.g. CHECK(Point{1, 2} == ...),
// are passed through as a single condition instead of several macro arguments.
#define CHECK(...)                                                           \
    do {                                                                     \
        ++::maj0test::checks;                                                \
        if (!(__VA_ARGS__)) {                                                \
            ++::maj0test::failures;                                          \
            std::printf("  [FAIL] %s:%d  CHECK(%s)\n", __FILE__, __LINE__,   \
                        #__VA_ARGS__);                                       \
        }                                                                    \
    } while (0)

#define CHECK_THROWS(...)                                                    \
    do {                                                                     \
        ++::maj0test::checks;                                                \
        bool threw = false;                                                  \
        try {                                                                \
            (void)(__VA_ARGS__);                                             \
        } catch (...) {                                                      \
            threw = true;                                                    \
        }                                                                    \
        if (!threw) {                                                        \
            ++::maj0test::failures;                                          \
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
    (std::printf("\n%d checks, %d failures\n", ::maj0test::checks,           \
                 ::maj0test::failures),                                      \
     ::maj0test::failures == 0 ? 0 : 1)
