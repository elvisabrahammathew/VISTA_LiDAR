#include <exception>
#include <iostream>

#include "unittest/test.hpp"

int main() {
    std::size_t passed = 0;
    for (const auto& test : vista::test::registry()) {
        try {
            test.function();
            ++passed;
            std::cout << "[PASS] " << test.name << '\n';
        } catch (const std::exception& error) {
            std::cerr << "[FAIL] " << test.name << ": " << error.what() << '\n';
        } catch (...) {
            std::cerr << "[FAIL] " << test.name << ": unknown exception\n";
        }
    }

    std::cout << passed << '/' << vista::test::registry().size()
              << " tests passed\n";
    return passed == vista::test::registry().size() ? 0 : 1;
}
