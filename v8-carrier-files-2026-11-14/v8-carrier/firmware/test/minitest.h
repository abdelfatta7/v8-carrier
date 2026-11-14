// Tiny assert-based test harness shared by every firmware test executable.
// No external framework: a test is a function registered with TEST(name),
// CHECK()/CHECK_NEAR() record failures, and main() runs everything and prints
// one line per test in the "file:line:name:PASS|FAIL" format PlatformIO's
// `test_framework = custom` runner understands, so the same executables run
// under CTest here and under `pio test -e native` on the owner's machine.
#pragma once

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <string>
#include <vector>

namespace minitest {

struct Case {
    const char* name;
    const char* file;
    int line;
    std::function<void()> fn;
};

inline std::vector<Case>& cases() {
    static std::vector<Case> v;
    return v;
}

inline int& failures_in_current() {
    static int n = 0;
    return n;
}

struct Registrar {
    Registrar(const char* name, const char* file, int line, std::function<void()> fn) {
        cases().push_back({name, file, line, fn});
    }
};

inline void fail(const char* file, int line, const std::string& msg) {
    ++failures_in_current();
    std::printf("%s:%d: check failed: %s\n", file, line, msg.c_str());
}

// Runs every registered test. Returns the number of failed tests (so it is
// usable directly as the process exit code).
inline int run_all(const char* suite) {
    int failed = 0;
    for (const Case& c : cases()) {
        failures_in_current() = 0;
        c.fn();
        if (failures_in_current() == 0) {
            std::printf("%s:%d:%s:PASS\n", c.file, c.line, c.name);
        } else {
            std::printf("%s:%d:%s:FAIL\n", c.file, c.line, c.name);
            ++failed;
        }
    }
    std::printf("-----------------------\n%zu Tests %d Failures 0 Ignored\n%s\n", cases().size(), failed,
                failed == 0 ? "OK" : "FAIL");
    (void)suite;
    return failed;
}

}  // namespace minitest

#define TEST(name)                                                                               \
    static void name();                                                                          \
    static minitest::Registrar minitest_registrar_##name(#name, __FILE__, __LINE__, name);       \
    static void name()

#define CHECK(cond)                                                                   \
    do {                                                                              \
        if (!(cond)) minitest::fail(__FILE__, __LINE__, #cond);                       \
    } while (0)

#define CHECK_EQ(a, b)                                                                            \
    do {                                                                                          \
        if (!((a) == (b)))                                                                        \
            minitest::fail(__FILE__, __LINE__,                                                    \
                           std::string(#a " == " #b " (") + std::to_string(a) + " vs " +          \
                               std::to_string(b) + ")");                                          \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                                     \
    do {                                                                                          \
        if (!(std::fabs((double)(a) - (double)(b)) <= (double)(tol)))                             \
            minitest::fail(__FILE__, __LINE__,                                                    \
                           std::string(#a " ~= " #b " (") + std::to_string((double)(a)) + " vs " + \
                               std::to_string((double)(b)) + ")");                                \
    } while (0)

// Each test executable defines MINITEST_MAIN(suite) once.
#define MINITEST_MAIN(suite) \
    int main() { return minitest::run_all(suite); }
