#define GHC_USE_STD_FS
#include <ghc/fs_std_fwd.hpp>

bool fs_std_fwd_header_test()
{
    fs::fstream stream;
    return stream.is_open();
}
