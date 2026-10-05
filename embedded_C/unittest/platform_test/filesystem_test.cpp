#include "1_Platform/compat/filesystem.hpp"
#include "unittest/test.hpp"
#include <chrono>

namespace {
struct TemporaryTree {
    vista::fs::path root = vista::fs::temp_directory_path() /
        ("vista-filesystem-test-" + std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count()));
    TemporaryTree() { vista::fs::create_directories(root); }
    ~TemporaryTree() { std::error_code ec; vista::fs::remove_all(root, ec); }
};
}

VISTA_TEST(filesystem_normalizes_paths_without_touching_disk) {
    VISTA_CHECK(vista::fs::path("maps/./tile/../room.pcd").lexically_normal() ==
                vista::fs::path("maps/room.pcd"));
    VISTA_CHECK(vista::fs::path("RoomMap_20261006_120000.pcd").stem() ==
                vista::fs::path("RoomMap_20261006_120000"));
}

VISTA_TEST(filesystem_streams_support_spaces_and_unicode) {
    TemporaryTree tree;
    const auto file = tree.root / vista::fs::u8path(u8"map space \u00e9.pcd");
    { vista::io::ofstream output(file, std::ios::binary); output << "geometry";
      output.close(); VISTA_CHECK(output.good()); }
    vista::io::ifstream input(file, std::ios::binary);
    std::string value; input >> value;
    VISTA_CHECK(value == "geometry");
    VISTA_CHECK(vista::fs::file_size(file) == 8);
}

VISTA_TEST(filesystem_rename_iteration_and_space_work) {
    TemporaryTree tree;
    const auto nested = tree.root / "tiles";
    vista::fs::create_directories(nested);
    const auto temporary = nested / "cells.tmp";
    { vista::io::ofstream output(temporary); output << "cell"; }
    const auto destination = nested / "cells.bin";
    vista::fs::rename(temporary, destination);
    VISTA_CHECK(!vista::fs::exists(temporary));
    std::size_t regular_files = 0;
    for (const auto& entry : vista::fs::recursive_directory_iterator(tree.root))
        if (entry.is_regular_file()) ++regular_files;
    VISTA_CHECK(regular_files == 1);
    std::error_code error;
    const auto capacity = vista::fs::space(tree.root, error);
    VISTA_CHECK(!error && capacity.capacity > 0);
}

VISTA_TEST(filesystem_error_code_handles_missing_files) {
    TemporaryTree tree;
    std::error_code error;
    VISTA_CHECK(!vista::fs::exists(tree.root / "missing.pcd", error));
    VISTA_CHECK(!error);
    VISTA_CHECK(!vista::fs::remove(tree.root / "missing.pcd", error));
    VISTA_CHECK(!error);
}
