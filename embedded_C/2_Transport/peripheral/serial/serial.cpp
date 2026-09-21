#include "2_Transport/peripheral/serial/serial.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace vista::transport {

namespace {

#ifdef _WIN32

std::string windows_port_name(const std::string& port) {
    if (port.rfind("\\\\.\\", 0) == 0) {
        return port;
    }
    return "\\\\.\\" + port;
}

std::runtime_error serial_error(const std::string& action) {
    return std::runtime_error(
        action + " failed with Windows error " +
        std::to_string(GetLastError()));
}

#else

std::runtime_error serial_error(const std::string& action) {
    return std::runtime_error(action + " failed: " + std::strerror(errno));
}

speed_t serial_speed(std::uint32_t baud_rate) {
    switch (baud_rate) {
        case 9'600:
            return B9600;
        case 19'200:
            return B19200;
        case 38'400:
            return B38400;
        case 57'600:
            return B57600;
        case 115'200:
            return B115200;
#ifdef B230400
        case 230'400:
            return B230400;
#endif
#ifdef B460800
        case 460'800:
            return B460800;
#endif
#ifdef B921600
        case 921'600:
            return B921600;
#endif
#ifdef B1000000
        case 1'000'000:
            return B1000000;
#endif
#ifdef B2000000
        case 2'000'000:
            return B2000000;
#endif
#ifdef B3000000
        case 3'000'000:
            return B3000000;
#endif
#ifdef B4000000
        case 4'000'000:
            return B4000000;
#endif
        default:
            throw std::invalid_argument(
                "unsupported POSIX serial baud rate " +
                std::to_string(baud_rate));
    }
}

#endif

}  // namespace

struct SerialConnection::Impl {
#ifdef _WIN32
    explicit Impl(HANDLE value) : handle(value) {}
    ~Impl() {
        if (handle != INVALID_HANDLE_VALUE) {
            CloseHandle(handle);
        }
    }
    HANDLE handle{INVALID_HANDLE_VALUE};
#else
    Impl(int value, std::chrono::milliseconds timeout_value)
        : descriptor(value), timeout(timeout_value) {}
    ~Impl() {
        if (descriptor >= 0) {
            close(descriptor);
        }
    }
    int descriptor{-1};
    std::chrono::milliseconds timeout{};
#endif
};

SerialConnection::SerialConnection(std::unique_ptr<Impl> implementation)
    : implementation_(std::move(implementation)) {}

SerialConnection::SerialConnection(SerialConnection&&) noexcept = default;
SerialConnection& SerialConnection::operator=(SerialConnection&&) noexcept = default;
SerialConnection::~SerialConnection() = default;

SerialConnection SerialConnection::open(
    const std::string& port,
    std::uint32_t baud_rate,
    std::chrono::milliseconds read_timeout) {
    if (port.empty() || baud_rate == 0 || read_timeout.count() <= 0) {
        throw std::invalid_argument("invalid serial connection configuration");
    }

#ifdef _WIN32
    const auto native_port = windows_port_name(port);
    const auto handle = CreateFileA(
        native_port.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
        OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (handle == INVALID_HANDLE_VALUE) {
        throw serial_error("opening serial port '" + port + "'");
    }

    auto implementation = std::make_unique<Impl>(handle);
    DCB settings{};
    settings.DCBlength = sizeof(settings);
    if (!GetCommState(handle, &settings)) {
        throw serial_error("reading serial settings");
    }
    settings.BaudRate = baud_rate;
    settings.ByteSize = 8;
    settings.Parity = NOPARITY;
    settings.StopBits = ONESTOPBIT;
    settings.fBinary = TRUE;
    settings.fParity = FALSE;
    settings.fOutxCtsFlow = FALSE;
    settings.fOutxDsrFlow = FALSE;
    settings.fDtrControl = DTR_CONTROL_ENABLE;
    settings.fDsrSensitivity = FALSE;
    settings.fOutX = FALSE;
    settings.fInX = FALSE;
    settings.fRtsControl = RTS_CONTROL_ENABLE;
    if (!SetCommState(handle, &settings)) {
        throw serial_error("configuring serial port '" + port + "'");
    }

    COMMTIMEOUTS timeouts{};
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutConstant =
        static_cast<DWORD>(read_timeout.count());
    timeouts.WriteTotalTimeoutConstant =
        static_cast<DWORD>(read_timeout.count());
    if (!SetCommTimeouts(handle, &timeouts)) {
        throw serial_error("configuring serial timeouts");
    }
    PurgeComm(handle, PURGE_RXCLEAR | PURGE_TXCLEAR);
    return SerialConnection(std::move(implementation));
#else
    const auto descriptor =
        ::open(port.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC | O_NONBLOCK);
    if (descriptor < 0) {
        throw serial_error("opening serial port '" + port + "'");
    }

    auto implementation = std::make_unique<Impl>(descriptor, read_timeout);
    termios settings{};
    if (tcgetattr(descriptor, &settings) != 0) {
        throw serial_error("reading serial settings");
    }
    cfmakeraw(&settings);
    const auto speed = serial_speed(baud_rate);
    if (cfsetispeed(&settings, speed) != 0 ||
        cfsetospeed(&settings, speed) != 0) {
        throw serial_error("setting serial speed");
    }
    settings.c_cflag |= CLOCAL | CREAD;
    settings.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    settings.c_cflag &= static_cast<tcflag_t>(~PARENB);
#ifdef CRTSCTS
    settings.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
#endif
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;
    if (tcsetattr(descriptor, TCSANOW, &settings) != 0) {
        throw serial_error("configuring serial port '" + port + "'");
    }
    tcflush(descriptor, TCIOFLUSH);
    return SerialConnection(std::move(implementation));
#endif
}

std::size_t SerialConnection::read_some(
    std::uint8_t* output,
    std::size_t size) {
    if (size == 0) {
        return 0;
    }
#ifdef _WIN32
    DWORD received = 0;
    const auto request = static_cast<DWORD>(
        std::min<std::size_t>(size, static_cast<std::size_t>(MAXDWORD)));
    if (!ReadFile(implementation_->handle, output, request, &received, nullptr)) {
        throw serial_error("serial read");
    }
    if (received == 0) {
        throw std::runtime_error("serial read timed out");
    }
    return static_cast<std::size_t>(received);
#else
    pollfd descriptor{implementation_->descriptor, POLLIN, 0};
    const auto wait = poll(
        &descriptor, 1, static_cast<int>(implementation_->timeout.count()));
    if (wait == 0) {
        throw std::runtime_error("serial read timed out");
    }
    if (wait < 0) {
        throw serial_error("waiting for serial data");
    }
    const auto received = ::read(implementation_->descriptor, output, size);
    if (received < 0) {
        throw serial_error("serial read");
    }
    return static_cast<std::size_t>(received);
#endif
}

void SerialConnection::write_all(
    const std::uint8_t* data,
    std::size_t size) {
    std::size_t written = 0;
    while (written < size) {
#ifdef _WIN32
        DWORD count = 0;
        const auto request = static_cast<DWORD>(std::min<std::size_t>(
            size - written, static_cast<std::size_t>(MAXDWORD)));
        if (!WriteFile(
                implementation_->handle, data + written, request, &count,
                nullptr)) {
            throw serial_error("serial write");
        }
#else
        const auto count = ::write(
            implementation_->descriptor, data + written, size - written);
        if (count < 0) {
            throw serial_error("serial write");
        }
#endif
        if (count == 0) {
            throw std::runtime_error("serial write made no progress");
        }
        written += static_cast<std::size_t>(count);
    }
}

}  // namespace vista::transport
