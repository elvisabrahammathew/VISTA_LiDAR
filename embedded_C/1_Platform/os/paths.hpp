#pragma once

#include "1_Platform/compat/filesystem.hpp"
#include <string_view>

namespace vista::platform {

/// Absolute path of the running executable, independent of the working
/// directory, shortcuts and argv[0]. Throws if the OS cannot resolve it.
vista::fs::path executable_path();

/// Exclusively create a missing text file. Never open/truncate an existing
/// path (including symlinks). Returns true only if created; throws on I/O error.
bool create_text_file_if_missing(const vista::fs::path& path,
    std::string_view contents);

}  // namespace vista::platform
