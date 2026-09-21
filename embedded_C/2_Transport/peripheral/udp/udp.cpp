#include "2_Transport/peripheral/udp/udp.hpp"

#include <stdexcept>
#include <string>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <netdb.h>
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
            throw std::runtime_error(
                "WSAStartup failed with error " + std::to_string(result));
        }
    }
    ~WinsockRuntime() { WSACleanup(); }
};

void ensure_socket_runtime() {
    static WinsockRuntime runtime;
    (void)runtime;
}

int last_socket_error() { return WSAGetLastError(); }
void close_socket(SocketHandle socket) { closesocket(socket); }

#else
using SocketHandle = int;
constexpr SocketHandle invalid_socket = -1;
void ensure_socket_runtime() {}
int last_socket_error() { return errno; }
void close_socket(SocketHandle socket) { close(socket); }
#endif

std::runtime_error socket_error(const std::string& action, int error) {
#ifdef _WIN32
    return std::runtime_error(
        action + " failed with socket error " + std::to_string(error));
#else
    return std::runtime_error(action + " failed: " + std::strerror(error));
#endif
}

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
        const auto result = socket_;
        socket_ = invalid_socket;
        return result;
    }

private:
    SocketHandle socket_;
};

sockaddr_in ipv4_endpoint(
    const std::string& ip,
    std::uint16_t port,
    const std::string& label) {
    sockaddr_in endpoint{};
    endpoint.sin_family = AF_INET;
    endpoint.sin_port = htons(port);
    if (inet_pton(AF_INET, ip.c_str(), &endpoint.sin_addr) != 1) {
        throw std::invalid_argument(
            label + " must be an IPv4 address, got '" + ip + "'");
    }
    return endpoint;
}

void set_receive_timeout(
    SocketHandle socket,
    std::chrono::milliseconds timeout) {
#ifdef _WIN32
    const auto value = static_cast<DWORD>(timeout.count());
    const auto* data = reinterpret_cast<const char*>(&value);
    const int size = sizeof(value);
#else
    const timeval value{
        static_cast<time_t>(timeout.count() / 1000),
        static_cast<suseconds_t>((timeout.count() % 1000) * 1000),
    };
    const auto* data = reinterpret_cast<const char*>(&value);
    const socklen_t size = sizeof(value);
#endif
    if (setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, data, size) != 0) {
        throw socket_error("setting UDP receive timeout", last_socket_error());
    }
}

}  // namespace

struct UdpConnection::Impl {
    explicit Impl(SocketHandle value) : socket(value) {}
    ~Impl() { close_socket(socket); }
    SocketHandle socket{invalid_socket};
};

UdpConnection::UdpConnection(std::unique_ptr<Impl> implementation)
    : implementation_(std::move(implementation)) {}

UdpConnection::UdpConnection(UdpConnection&&) noexcept = default;
UdpConnection& UdpConnection::operator=(UdpConnection&&) noexcept = default;
UdpConnection::~UdpConnection() = default;

UdpConnection UdpConnection::connect(
    const std::string& local_ip,
    std::uint16_t local_port,
    const std::string& remote_ip,
    std::uint16_t remote_port,
    std::chrono::milliseconds read_timeout) {
    if (local_port == 0 || remote_port == 0 || read_timeout.count() <= 0) {
        throw std::invalid_argument("invalid UDP connection configuration");
    }
    ensure_socket_runtime();
    SocketGuard socket(::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP));
    if (socket.get() == invalid_socket) {
        throw socket_error("creating UDP socket", last_socket_error());
    }

    const auto local = ipv4_endpoint(local_ip, local_port, "LocalIP");
    if (bind(
            socket.get(), reinterpret_cast<const sockaddr*>(&local),
            sizeof(local)) != 0) {
        throw socket_error(
            "binding UDP endpoint " + local_ip + ':' +
                std::to_string(local_port),
            last_socket_error());
    }

    const auto remote = ipv4_endpoint(remote_ip, remote_port, "SensorIP");
    if (::connect(
            socket.get(), reinterpret_cast<const sockaddr*>(&remote),
            sizeof(remote)) != 0) {
        throw socket_error(
            "connecting UDP endpoint " + remote_ip + ':' +
                std::to_string(remote_port),
            last_socket_error());
    }
    set_receive_timeout(socket.get(), read_timeout);
    return UdpConnection(std::make_unique<Impl>(socket.release()));
}

std::size_t UdpConnection::receive(
    std::uint8_t* output,
    std::size_t capacity) {
    if (capacity == 0) {
        return 0;
    }
#ifdef _WIN32
    const auto count = recv(
        implementation_->socket,
        reinterpret_cast<char*>(output),
        static_cast<int>(capacity),
        0);
#else
    const auto count = recv(implementation_->socket, output, capacity, 0);
#endif
    if (count < 0) {
        throw socket_error("UDP receive", last_socket_error());
    }
    return static_cast<std::size_t>(count);
}

void UdpConnection::send(const std::uint8_t* data, std::size_t size) {
#ifdef _WIN32
    const auto count = ::send(
        implementation_->socket,
        reinterpret_cast<const char*>(data),
        static_cast<int>(size),
        0);
#else
    const auto count = ::send(
        implementation_->socket, data, size, MSG_NOSIGNAL);
#endif
    if (count < 0 || static_cast<std::size_t>(count) != size) {
        throw socket_error("UDP send", last_socket_error());
    }
}

}  // namespace vista::transport
