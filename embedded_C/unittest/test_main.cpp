#include <exception>
#include <iostream>

#include "unittest/test.hpp"

int main(int argc, char** argv) {
    // Optional substring enables focused regressions without skipping the
    // normal full-suite run when no argument is supplied.
    const std::string filter=argc>1?argv[1]:"";
    std::size_t passed = 0;
    std::size_t selected = 0;
    for (const auto& test : vista::test::registry()) {
        if(!filter.empty() && test.name.find(filter)==std::string::npos)continue;
        ++selected;
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

    std::cout << passed << '/' << selected
              << " tests passed\n";
    if(selected==0){std::cerr<<"No tests matched: "<<filter<<'\n';return 2;}
    return passed == selected ? 0 : 1;
}
