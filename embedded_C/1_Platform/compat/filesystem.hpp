#pragma once
#include <fstream>

// One backend is selected by CMake for all VISTA targets. Keep it out of std
// and keep path types identical across application/core/test boundaries.
#if defined(VISTA_USE_PORTABLE_FILESYSTEM) && VISTA_USE_PORTABLE_FILESYSTEM
// Only API declarations reach callers; OS headers and implementation are
// compiled once in filesystem.cpp, avoiding Windows min/max/near/far leaks.
#include <ghc/fs_fwd.hpp>
namespace vista { namespace fs = ghc::filesystem; }
namespace vista { namespace io {
using ifstream = ghc::filesystem::ifstream;
using ofstream = ghc::filesystem::ofstream;
using fstream = ghc::filesystem::fstream;
} }
#else
#include <filesystem>
namespace vista { namespace fs = std::filesystem; }
namespace vista { namespace io {
using ifstream = std::ifstream;
using ofstream = std::ofstream;
using fstream = std::fstream;
} }
#endif
