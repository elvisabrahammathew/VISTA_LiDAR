#ifdef _WIN32

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <array>
#include <string>

#include "1_Platform/threading/threading.hpp"

namespace vista::platform {

namespace {

constexpr std::array<int, 5> native_priorities{
    THREAD_PRIORITY_HIGHEST,
    THREAD_PRIORITY_ABOVE_NORMAL,
    THREAD_PRIORITY_NORMAL,
    THREAD_PRIORITY_BELOW_NORMAL,
    THREAD_PRIORITY_LOWEST,
};

}  // namespace

namespace detail {

PriorityStatus configure_current_thread(
    const std::string&,
    ThreadPriority requested) noexcept {
    PriorityStatus status{requested, false, {}, {}};
    const auto native = native_priorities.at(requested.level() - 1);
    if (SetThreadPriority(GetCurrentThread(), native) == 0) {
        status.error = "SetThreadPriority failed with error " +
                       std::to_string(GetLastError());
        return status;
    }

    const auto applied = GetThreadPriority(GetCurrentThread());
    if (applied == THREAD_PRIORITY_ERROR_RETURN) {
        status.error = "GetThreadPriority failed with error " +
                       std::to_string(GetLastError());
        return status;
    }

    status.applied = true;
    status.native = {"NORMAL_PRIORITY_CLASS", applied};
    return status;
}

}  // namespace detail

const char* platform_name() noexcept {
    return "Windows";
}

}  // namespace vista::platform

#endif
