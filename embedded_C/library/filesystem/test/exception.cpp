#include <ghc/filesystem.hpp>

#ifdef GHC_WITH_EXCEPTIONS
#error "This test must be compiled without exception support"
#endif

int main(int argc, char** argv)
{
    namespace fs = ghc::filesystem;

    if (argc != 2) {
        return 1;
    }

    const fs::path regular_file(argv[1]);
    const fs::path test_directory = regular_file.parent_path();
    std::error_code ec;
    const fs::path proximate = fs::proximate(test_directory, ec);
    if (ec || proximate.empty()) {
        return 2;
    }

    fs::directory_iterator invalid_iter(regular_file, ec);
    if (!ec || invalid_iter != fs::directory_iterator()) {
        return 3;
    }

    ec.clear();
    fs::recursive_directory_iterator invalid_recursive_iter(regular_file, ec);
    if (!ec || invalid_recursive_iter != fs::recursive_directory_iterator()) {
        return 4;
    }

    ec.clear();
    fs::directory_iterator iter(test_directory, ec);
    if (ec) {
        return 5;
    }
    if (iter != fs::directory_iterator()) {
        iter++;
    }
    for (; iter != fs::directory_iterator(); ++iter) {
    }

    for (const auto& entry : fs::directory_iterator(test_directory, ec)) {
        (void)entry;
    }
    if (ec) {
        return 6;
    }

    fs::recursive_directory_iterator recursive_iter(test_directory, ec);
    if (ec) {
        return 7;
    }
    if (recursive_iter != fs::recursive_directory_iterator()) {
        recursive_iter++;
    }
    for (; recursive_iter != fs::recursive_directory_iterator(); ++recursive_iter) {
    }

    for (const auto& entry : fs::recursive_directory_iterator(test_directory, ec)) {
        (void)entry;
    }
    if (ec) {
        return 8;
    }

    return 0;
}
