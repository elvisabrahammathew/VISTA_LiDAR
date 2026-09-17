#include "2_Transport/peripheral/ethernet/ethernet.hpp"

#include <cstring>
#include <stdexcept>
#include <string>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace vista::transport {

namespace {

#ifdef _WIN32
using SocketHandle = SOCKET;
constexpr SocketHandle invalid_socket = INVALID_SOCKET;

struct WinsockRuntime {
    WinsockRuntime() {
        WSADATA data{};
        const auto result = WSAStartup(MAKEWORD(2, 2), &data);
        if (result != 0) {
            throw std::runtime_error("WSAStartup failed with error " + std::to_string(result));
        }
    }

    ~WinsockRuntime() {
        WSACleanup();
    }
};

void ensure_socket_runtime() {
    static WinsockRuntime runtime;
    (void)runtime;
}

int last_socket_error() {
    return WSAGetLastError();
}

int connection_timeout_error() {
    return WSAETIMEDOUT;
}

void close_socket(SocketHandle socket) {
    closesocket(socket);
}

void set_blocking(SocketHandle socket, bool blocking) {
    u_long value = blocking ? 0UL : 1UL;
    if (ioctlsocket(socket, FIONBIO, &value) != 0) {
        throw std::runtime_error("ioctlsocket failed with error " +
                                 std::to_string(last_socket_error()));
    }
}

bool wait_until_connected(SocketHandle socket, std::chrono::milliseconds timeout) {
    fd_set write_set;
    FD_ZERO(&write_set);
    FD_SET(socket, &write_set);
    timeval value{
        static_cast<long>(timeout.count() / 1000),
        static_cast<long>((timeout.count() % 1000) * 1000),
    };
    return select(0, nullptr, &write_set, nullptr, &value) > 0;
}
#else
using SocketHandle = int;
constexpr SocketHandle invalid_socket = -1;

void ensure_socket_runtime() {}

int last_socket_error() {
    return errno;
}

int connection_timeout_error() {
    return ETIMEDOUT;
}

void close_socket(SocketHandle socket) {
    close(socket);
}

void set_blocking(SocketHandle socket, bool blocking) {
    const auto flags = fcntl(socket, F_GETFL, 0);
    if (flags < 0 ||
        fcntl(socket, F_SETFL, blocking ? (flags & ~O_NONBLOCK) : (flags | O_NONBLOCK)) < 0) {
        throw std::runtime_error(std::string("fcntl failed: ") + std::strerror(errno));
    }
}

bool wait_until_connected(SocketHandle socket, std::chrono::milliseconds timeout) {
    pollfd descriptor{socket, POLLOUT, 0};
    return poll(&descriptor, 1, static_cast<int>(timeout.count())) > 0;
}
#endif

class SocketGuard {
public:
    explicit SocketGuard(SocketHandle socket) : socket_(socket) {}
    ~SocketGuard() {
        if (socket_ != invalid_socket) {
            close_socket(socket_);
        }
    }

    SocketHandle get() const noexcept { return socket_; }
    SocketHandle release() noexcept {
        const auto socket = socket_;
        socket_ = invalid_socket;
        return socket;
    }

private:
    SocketHandle socket_;
};

std::runtime_error socket_error(const std::string& action, int error) {
#ifdef _WIN32
    return std::runtime_error(action + " failed with socket error " + std::to_string(error));
#else
    return std::runtime_error(action + " failed: " + std::strerror(error));
#endif
}

SocketHandle connect_socket(
    const std::string& host,
    std::uint16_t port,
    std::chrono::milliseconds timeout) {
    ensure_socket_runtime();

    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    addrinfo* addresses = nullptr;
    const auto port_text = std::to_string(port);
    const auto lookup = getaddrinfo(host.c_str(), port_text.c_str(), &hints, &addresses);
    if (lookup != 0) {
        throw std::runtime_error("could not resolve Ethernet host '" + host + "'");
    }

    struct AddressListGuard {
        addrinfo* value;
        ~AddressListGuard() { freeaddrinfo(value); }
    } address_guard{addresses};

    int final_error = 0;
    for (auto* address = addresses; address != nullptr; address = address->ai_next) {
        SocketGuard socket(::socket(address->ai_family, address->ai_socktype, address->ai_protocol));
        if (socket.get() == invalid_socket) {
            final_error = last_socket_error();
            continue;
        }

        try {
            set_blocking(socket.get(), false);
        } catch (...) {
            final_error = last_socket_error();
            continue;
        }

        const auto result =
            ::connect(socket.get(), address->ai_addr, static_cast<int>(address->ai_addrlen));
        if (result != 0) {
            const auto connect_error = last_socket_error();
#ifdef _WIN32
            const bool in_progress =
                connect_error == WSAEWOULDBLOCK || connect_error == WSAEINPROGRESS;
#else
            const bool in_progress = connect_error == EINPROGRESS;
#endif
            if (!in_progress) {
                final_error = connect_error;
                continue;
            }
            if (!wait_until_connected(socket.get(), timeout)) {
                final_error = connection_timeout_error();
                continue;
            }
        }

        int pending_error = 0;
#ifdef _WIN32
        int error_size = sizeof(pending_error);
#else
        socklen_t error_size = sizeof(pending_error);
#endif
        if (getsockopt(
                socket.get(),
                SOL_SOCKET,
                SO_ERROR,
                reinterpret_cast<char*>(&pending_error),
                &error_size) != 0 ||
            pending_error != 0) {
            final_error = pending_error != 0 ? pending_error : last_socket_error();
            continue;
        }

        set_blocking(socket.get(), true);
        return socket.release();
    }

    const auto endpoint = host + ":" + std::to_string(port);
    if (final_error == connection_timeout_error()) {
        throw std::runtime_error(
            "Ethernet connection to " + endpoint + " timed out after " +
            std::to_string(timeout.count()) + " ms");
    }
    throw socket_error("Ethernet connect to " + endpoint, final_error);
}

void configure_socket(
    SocketHandle socket,
    std::chrono::milliseconds read_timeout) {
    const int enabled = 1;
    if (setsockopt(
            socket,
            IPPROTO_TCP,
            TCP_NODELAY,
            reinterpret_cast<const char*>(&enabled),
            sizeof(enabled)) != 0) {
        throw socket_error("TCP_NODELAY", last_socket_error());
    }

#ifdef _WIN32
    const auto timeout = static_cast<DWORD>(read_timeout.count());
    const char* timeout_data = reinterpret_cast<const char*>(&timeout);
    const int timeout_size = sizeof(timeout);
#else
    const timeval timeout{
        static_cast<time_t>(read_timeout.count() / 1000),
        static_cast<suseconds_t>((read_timeout.count() % 1000) * 1000),
    };
    const char* timeout_data = reinterpret_cast<const char*>(&timeout);
    const socklen_t timeout_size = sizeof(timeout);
#endif
    if (setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, timeout_data, timeout_size) != 0) {
        throw socket_error("SO_RCVTIMEO", last_socket_error());
    }
}

}  // namespace

struct EthernetConnection::Impl {
    explicit Impl(SocketHandle value) : socket(value) {}
    ~Impl() { close_socket(socket); }
    SocketHandle socket;
};

EthernetConnection::EthernetConnection(std::unique_ptr<Impl> implementation)
    : implementation_(std::move(implementation)) {}

EthernetConnection::EthernetConnection(EthernetConnection&&) noexcept = default;
EthernetConnection& EthernetConnection::operator=(EthernetConnection&&) noexcept = default;
EthernetConnection::~EthernetConnection() = default;

EthernetConnection EthernetConnection::connect(
    const std::string& host,
    std::uint16_t port,
    std::chrono::milliseconds connect_timeout,
    std::chrono::milliseconds read_timeout) {
    const auto socket = connect_socket(host, port, connect_timeout);
    SocketGuard guard(socket);
    configure_socket(socket, read_timeout);
    return EthernetConnection(std::make_unique<Impl>(guard.release()));
}

void EthernetConnection::read_exact(std::uint8_t* output, std::size_t size) {
    std::size_t received = 0;
    while (received < size) {
#ifdef _WIN32
        const auto count = recv(
            implementation_->socket,
            reinterpret_cast<char*>(output + received),
            static_cast<int>(size - received),
            0);
#else
        const auto count = recv(
            implementation_->socket,
            output + received,
            size - received,
            0);
#endif
        if (count == 0) {
            throw std::runtime_error("Ethernet peer closed the connection");
        }
        if (count < 0) {
            throw socket_error("Ethernet receive", last_socket_error());
        }
        received += static_cast<std::size_t>(count);
    }
}

}  // namespace vista::transport
