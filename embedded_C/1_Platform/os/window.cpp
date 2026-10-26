#ifdef _WIN32

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <array>
#include <string>
#include <system_error>
#include <vector>

#include "1_Platform/os/paths.hpp"
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

bool create_text_file_if_missing(const vista::fs::path& path,
    std::string_view contents) {
    const auto handle = CreateFileW(path.c_str(), GENERIC_WRITE, 0, nullptr,
        CREATE_NEW, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (handle == INVALID_HANDLE_VALUE) {
        const auto error = GetLastError();
        if (error == ERROR_FILE_EXISTS || error == ERROR_ALREADY_EXISTS) return false;
        throw std::system_error(static_cast<int>(error), std::system_category(),
            "could not create configuration '" + path.u8string() + "'");
    }
    try {
        while (!contents.empty()) {
            const auto count = static_cast<DWORD>(
                contents.size() > 1024 * 1024 ? 1024 * 1024 : contents.size());
            DWORD written = 0;
            if (!WriteFile(handle, contents.data(), count, &written, nullptr)) {
                throw std::system_error(static_cast<int>(GetLastError()),
                    std::system_category(), "could not write configuration '" +
                    path.u8string() + "'");
            }
            if (written == 0) throw std::runtime_error(
                "incomplete configuration write: " + path.u8string());
            contents.remove_prefix(written);
        }
    } catch (...) {
        CloseHandle(handle);
        throw;
    }
    if (!CloseHandle(handle)) {
        throw std::system_error(static_cast<int>(GetLastError()),
            std::system_category(), "could not close configuration '" +
            path.u8string() + "'");
    }
    return true;
}

vista::fs::path executable_path() {
    // The wide API preserves non-ASCII installation directories. Grow beyond
    // MAX_PATH rather than silently using a truncated filename.
    std::vector<wchar_t> buffer(512);
    while (buffer.size() <= 32768) {
        const auto length = GetModuleFileNameW(
            nullptr, buffer.data(), static_cast<DWORD>(buffer.size()));
        if (length == 0) {
            throw std::system_error(
                static_cast<int>(GetLastError()), std::system_category(),
                "could not locate the running executable");
        }
        if (length < buffer.size()) {
            return vista::fs::path(std::wstring(buffer.data(), length));
        }
        buffer.resize(buffer.size() * 2);
    }
    throw std::runtime_error("running executable path exceeds Windows limits");
}

}  // namespace vista::platform

#endif
