#ifdef __linux__

#include <array>
#include <cerrno>
#include <cstring>
#include <pthread.h>
#include <sys/resource.h>
#include <unistd.h>
#include <fcntl.h>
#include <system_error>
#include <vector>

#include "1_Platform/os/paths.hpp"
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

bool create_text_file_if_missing(const vista::fs::path& path,
    std::string_view contents) {
    const auto descriptor = ::open(path.c_str(),
        O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
    if (descriptor < 0) {
        if (errno == EEXIST) return false;
        throw std::system_error(errno, std::generic_category(),
            "could not create configuration '" + path.u8string() + "'");
    }
    try {
        while (!contents.empty()) {
            const auto count = contents.size() > 1024 * 1024
                ? 1024 * 1024 : contents.size();
            const auto written = ::write(descriptor, contents.data(), count);
            if (written < 0) {
                if (errno == EINTR) continue;
                throw std::system_error(errno, std::generic_category(),
                    "could not write configuration '" + path.u8string() + "'");
            }
            if (written == 0) throw std::runtime_error(
                "incomplete configuration write: " + path.u8string());
            contents.remove_prefix(static_cast<std::size_t>(written));
        }
    } catch (...) {
        ::close(descriptor);
        throw;
    }
    if (::close(descriptor) != 0) {
        throw std::system_error(errno, std::generic_category(),
            "could not close configuration '" + path.u8string() + "'");
    }
    return true;
}

vista::fs::path executable_path() {
    std::vector<char> buffer(512);
    while (buffer.size() <= 1024 * 1024) {
        const auto length = ::readlink(
            "/proc/self/exe", buffer.data(), buffer.size());
        if (length < 0) {
            throw std::system_error(errno, std::generic_category(),
                "could not locate the running executable via /proc/self/exe");
        }
        if (static_cast<std::size_t>(length) < buffer.size()) {
            return vista::fs::path(std::string(buffer.data(),
                static_cast<std::size_t>(length)));
        }
        buffer.resize(buffer.size() * 2);
    }
    throw std::runtime_error("running executable path is too long");
}

}  // namespace vista::platform

#endif
