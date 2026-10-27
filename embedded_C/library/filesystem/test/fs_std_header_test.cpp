#define GHC_USE_STD_FS
#include <ghc/fs_std.hpp>

bool fs_std_fwd_header_test();

int main()
{
    fs::ifstream stream;
    return stream.is_open() || fs_std_fwd_header_test() ? 1 : 0;
}
