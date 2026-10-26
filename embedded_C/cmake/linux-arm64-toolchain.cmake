# Cross-build ARM64 on Linux x86-64 using target-specific Ubuntu packages.
# Native ARM64 builds do not need this file.
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
set(CMAKE_FIND_ROOT_PATH /usr/aarch64-linux-gnu)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
find_program(VISTA_QEMU_AARCH64 qemu-aarch64)
if(VISTA_QEMU_AARCH64)
    set(CMAKE_CROSSCOMPILING_EMULATOR
        "${VISTA_QEMU_AARCH64};-cpu;cortex-a57;-L;/usr/aarch64-linux-gnu")
endif()
set(PKG_CONFIG_EXECUTABLE /bin/false CACHE FILEPATH
    "Do not let ARM64 dependencies use x86-64 pkg-config metadata")
