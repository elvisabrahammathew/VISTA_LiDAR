#pragma once

#include <cmath>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace vista::test {

struct TestCase {
    std::string name;
    std::function<void()> function;
};

inline std::vector<TestCase>& registry() {
    static std::vector<TestCase> tests;
    return tests;
}

class Registrar {
public:
    Registrar(std::string name, std::function<void()> function) {
        registry().push_back({std::move(name), std::move(function)});
    }
};

inline void check(bool condition, const char* expression, const char* file, int line) {
    if (!condition) {
        std::ostringstream message;
        message << file << ':' << line << ": check failed: " << expression;
        throw std::runtime_error(message.str());
    }
}

}  // namespace vista::test

#define VISTA_TEST(name)                                      \
    static void name();                                       \
    static ::vista::test::Registrar registrar_##name(#name, name); \
    static void name()

#define VISTA_CHECK(expression) \
    ::vista::test::check(static_cast<bool>(expression), #expression, __FILE__, __LINE__)

#define VISTA_CHECK_NEAR(left, right, tolerance) \
    VISTA_CHECK(std::fabs((left) - (right)) <= (tolerance))

#define VISTA_CHECK_THROWS(expression)      \
    do {                                    \
        bool vista_did_throw = false;       \
        try {                               \
            (void)(expression);             \
        } catch (...) {                     \
            vista_did_throw = true;         \
        }                                   \
        VISTA_CHECK(vista_did_throw);       \
    } while (false)
