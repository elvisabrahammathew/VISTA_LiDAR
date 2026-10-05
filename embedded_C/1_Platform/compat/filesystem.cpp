#if defined(VISTA_USE_PORTABLE_FILESYSTEM) && VISTA_USE_PORTABLE_FILESYSTEM
#ifdef _WIN32
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#endif
#include <ghc/fs_impl.hpp>
#endif
