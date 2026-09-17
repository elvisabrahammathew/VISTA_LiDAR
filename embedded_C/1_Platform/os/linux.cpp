#ifdef __linux__

#include <array>
#include <cerrno>
#include <cstring>
#include <pthread.h>
#include <sys/resource.h>

#include "1_Platform/threading/threading.hpp"

namespace vista::platform {

namespace {

constexpr std::array<int, 5> nice_levels{0, 2, 5, 10, 15};

}  // namespace

namespace detail {

PriorityStatus configure_current_thread(
    const std::string& name,
    ThreadPriority requested) noexcept {
    PriorityStatus status{requested, false, {}, {}};

    // Linux limits a pthread name to 15 visible bytes.
    const auto short_name = name.substr(0, 15);
    (void)pthread_setname_np(pthread_self(), short_name.c_str());

    const auto nice_value = nice_levels.at(requested.level() - 1);
    if (setpriority(PRIO_PROCESS, 0, nice_value) != 0) {
        status.error = std::strerror(errno);
        return status;
    }

    errno = 0;
    const auto applied = getpriority(PRIO_PROCESS, 0);
    if (errno != 0 || applied != nice_value) {
        status.error = errno != 0 ? std::strerror(errno)
                                  : "Linux applied a different nice value";
        return status;
    }

    status.applied = true;
    status.native = {"SCHED_OTHER", applied};
    return status;
}

}  // namespace detail

const char* platform_name() noexcept {
    return "Linux";
}

}  // namespace vista::platform

#endif
