#include "2_Transport/messaging/websocket_server.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <climits>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
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
bool would_block(int error) { return error == WSAEWOULDBLOCK; }
void close_socket(SocketHandle socket) { closesocket(socket); }

void set_nonblocking(SocketHandle socket, bool enabled) {
    u_long value = enabled ? 1UL : 0UL;
    if (ioctlsocket(socket, FIONBIO, &value) != 0) {
        throw std::runtime_error(
            "ioctlsocket failed with error " +
            std::to_string(last_socket_error()));
    }
}
#else
using SocketHandle = int;
constexpr SocketHandle invalid_socket = -1;

void ensure_socket_runtime() {}
int last_socket_error() { return errno; }
bool would_block(int error) { return error == EAGAIN || error == EWOULDBLOCK; }
void close_socket(SocketHandle socket) { close(socket); }

void set_nonblocking(SocketHandle socket, bool enabled) {
    const auto flags = fcntl(socket, F_GETFL, 0);
    if (flags < 0 ||
        fcntl(
            socket,
            F_SETFL,
            enabled ? (flags | O_NONBLOCK) : (flags & ~O_NONBLOCK)) < 0) {
        throw std::runtime_error(
            std::string("fcntl failed: ") + std::strerror(errno));
    }
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
        const auto value = socket_;
        socket_ = invalid_socket;
        return value;
    }

private:
    SocketHandle socket_;
};

std::runtime_error socket_error(const std::string& action, int error) {
#ifdef _WIN32
    return std::runtime_error(
        action + " failed with socket error " + std::to_string(error));
#else
    return std::runtime_error(action + " failed: " + std::strerror(error));
#endif
}

void set_socket_timeout(
    SocketHandle socket,
    int option,
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
    if (setsockopt(socket, SOL_SOCKET, option, data, size) != 0) {
        throw socket_error("setting socket timeout", last_socket_error());
    }
}

bool send_all(SocketHandle socket, const std::uint8_t* data, std::size_t size) {
    std::size_t sent{};
    while (sent < size) {
#ifdef _WIN32
        const auto remaining = std::min<std::size_t>(
            size - sent, static_cast<std::size_t>(INT_MAX));
        const auto count = send(
            socket,
            reinterpret_cast<const char*>(data + sent),
            static_cast<int>(remaining),
            0);
#else
        const auto count = send(socket, data + sent, size - sent, MSG_NOSIGNAL);
#endif
        if (count <= 0) {
            return false;
        }
        sent += static_cast<std::size_t>(count);
    }
    return true;
}

std::string lower_copy(std::string value) {
    std::transform(
        value.begin(), value.end(), value.begin(), [](unsigned char character) {
            return static_cast<char>(std::tolower(character));
        });
    return value;
}

std::string trim(std::string value) {
    const auto first = std::find_if_not(
        value.begin(), value.end(), [](unsigned char character) {
            return std::isspace(character) != 0;
        });
    const auto last = std::find_if_not(
        value.rbegin(), value.rend(), [](unsigned char character) {
            return std::isspace(character) != 0;
        }).base();
    return first < last ? std::string(first, last) : std::string{};
}

std::string websocket_key_from_request(const std::string& request) {
    std::size_t offset{};
    while (offset < request.size()) {
        const auto end = request.find("\r\n", offset);
        const auto line = request.substr(
            offset,
            end == std::string::npos ? std::string::npos : end - offset);
        const auto separator = line.find(':');
        if (separator != std::string::npos &&
            lower_copy(trim(line.substr(0, separator))) ==
                "sec-websocket-key") {
            return trim(line.substr(separator + 1));
        }
        if (end == std::string::npos) {
            break;
        }
        offset = end + 2;
    }
    return {};
}

std::string read_handshake_request(SocketHandle socket) {
    std::string request;
    request.reserve(2048);
    std::array<char, 1024> buffer{};
    while (request.find("\r\n\r\n") == std::string::npos) {
        if (request.size() >= 8192) {
            throw std::runtime_error("WebSocket handshake header is too large");
        }
#ifdef _WIN32
        const auto count = recv(socket, buffer.data(), static_cast<int>(buffer.size()), 0);
#else
        const auto count = recv(socket, buffer.data(), buffer.size(), 0);
#endif
        if (count <= 0) {
            throw std::runtime_error("WebSocket client closed during handshake");
        }
        request.append(buffer.data(), static_cast<std::size_t>(count));
    }
    return request;
}

std::array<std::uint8_t, 20> sha1(const std::string& text) {
    std::vector<std::uint8_t> bytes(text.begin(), text.end());
    const auto bit_length = static_cast<std::uint64_t>(bytes.size()) * 8U;
    bytes.push_back(0x80U);
    while ((bytes.size() % 64U) != 56U) {
        bytes.push_back(0U);
    }
    for (int shift = 56; shift >= 0; shift -= 8) {
        bytes.push_back(static_cast<std::uint8_t>((bit_length >> shift) & 0xFFU));
    }

    std::uint32_t h0 = 0x67452301U;
    std::uint32_t h1 = 0xEFCDAB89U;
    std::uint32_t h2 = 0x98BADCFEU;
    std::uint32_t h3 = 0x10325476U;
    std::uint32_t h4 = 0xC3D2E1F0U;

    for (std::size_t chunk = 0; chunk < bytes.size(); chunk += 64U) {
        std::array<std::uint32_t, 80> words{};
        for (std::size_t index = 0; index < 16U; ++index) {
            const auto offset = chunk + index * 4U;
            words[index] =
                (static_cast<std::uint32_t>(bytes[offset]) << 24U) |
                (static_cast<std::uint32_t>(bytes[offset + 1U]) << 16U) |
                (static_cast<std::uint32_t>(bytes[offset + 2U]) << 8U) |
                static_cast<std::uint32_t>(bytes[offset + 3U]);
        }
        for (std::size_t index = 16U; index < words.size(); ++index) {
            const auto value = words[index - 3U] ^ words[index - 8U] ^
                               words[index - 14U] ^ words[index - 16U];
            words[index] = (value << 1U) | (value >> 31U);
        }

        auto a = h0;
        auto b = h1;
        auto c = h2;
        auto d = h3;
        auto e = h4;
        for (std::size_t index = 0; index < words.size(); ++index) {
            std::uint32_t function{};
            std::uint32_t constant{};
            if (index < 20U) {
                function = (b & c) | ((~b) & d);
                constant = 0x5A827999U;
            } else if (index < 40U) {
                function = b ^ c ^ d;
                constant = 0x6ED9EBA1U;
            } else if (index < 60U) {
                function = (b & c) | (b & d) | (c & d);
                constant = 0x8F1BBCDCU;
            } else {
                function = b ^ c ^ d;
                constant = 0xCA62C1D6U;
            }
            const auto rotated = (a << 5U) | (a >> 27U);
            const auto temporary =
                rotated + function + e + constant + words[index];
            e = d;
            d = c;
            c = (b << 30U) | (b >> 2U);
            b = a;
            a = temporary;
        }
        h0 += a;
        h1 += b;
        h2 += c;
        h3 += d;
        h4 += e;
    }

    std::array<std::uint8_t, 20> digest{};
    const std::array<std::uint32_t, 5> hash{h0, h1, h2, h3, h4};
    for (std::size_t index = 0; index < hash.size(); ++index) {
        digest[index * 4U] = static_cast<std::uint8_t>(hash[index] >> 24U);
        digest[index * 4U + 1U] =
            static_cast<std::uint8_t>(hash[index] >> 16U);
        digest[index * 4U + 2U] =
            static_cast<std::uint8_t>(hash[index] >> 8U);
        digest[index * 4U + 3U] = static_cast<std::uint8_t>(hash[index]);
    }
    return digest;
}

std::string base64(const std::uint8_t* data, std::size_t size) {
    constexpr char alphabet[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string output;
    output.reserve(((size + 2U) / 3U) * 4U);
    for (std::size_t index = 0; index < size; index += 3U) {
        const auto remaining = size - index;
        const auto value =
            (static_cast<std::uint32_t>(data[index]) << 16U) |
            (remaining > 1U
                 ? static_cast<std::uint32_t>(data[index + 1U]) << 8U
                 : 0U) |
            (remaining > 2U ? static_cast<std::uint32_t>(data[index + 2U])
                            : 0U);
        output.push_back(alphabet[(value >> 18U) & 0x3FU]);
        output.push_back(alphabet[(value >> 12U) & 0x3FU]);
        output.push_back(remaining > 1U ? alphabet[(value >> 6U) & 0x3FU] : '=');
        output.push_back(remaining > 2U ? alphabet[value & 0x3FU] : '=');
    }
    return output;
}

std::vector<std::uint8_t> binary_frame_header(std::size_t payload_size) {
    std::vector<std::uint8_t> header;
    header.reserve(10);
    header.push_back(0x82U);  // FIN + binary opcode.
    if (payload_size <= 125U) {
        header.push_back(static_cast<std::uint8_t>(payload_size));
    } else if (payload_size <= 65'535U) {
        header.push_back(126U);
        header.push_back(static_cast<std::uint8_t>((payload_size >> 8U) & 0xFFU));
        header.push_back(static_cast<std::uint8_t>(payload_size & 0xFFU));
    } else {
        header.push_back(127U);
        const auto value = static_cast<std::uint64_t>(payload_size);
        for (int shift = 56; shift >= 0; shift -= 8) {
            header.push_back(
                static_cast<std::uint8_t>((value >> shift) & 0xFFU));
        }
    }
    return header;
}

SocketHandle create_listen_socket(const WebSocketServerConfig& config) {
    ensure_socket_runtime();
    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags = AI_PASSIVE;

    addrinfo* addresses{};
    const auto port = std::to_string(config.port);
    const char* host = config.bind_address == "*"
                           ? nullptr
                           : config.bind_address.c_str();
    if (getaddrinfo(host, port.c_str(), &hints, &addresses) != 0) {
        throw std::runtime_error(
            "could not resolve WebSocket bind address '" +
            config.bind_address + "'");
    }
    struct AddressGuard {
        addrinfo* value;
        ~AddressGuard() { freeaddrinfo(value); }
    } address_guard{addresses};

    int final_error{};
    for (auto* address = addresses; address; address = address->ai_next) {
        SocketGuard socket(
            ::socket(address->ai_family, address->ai_socktype, address->ai_protocol));
        if (socket.get() == invalid_socket) {
            final_error = last_socket_error();
            continue;
        }
        const int reuse = 1;
        setsockopt(
            socket.get(),
            SOL_SOCKET,
            SO_REUSEADDR,
            reinterpret_cast<const char*>(&reuse),
            sizeof(reuse));
        if (bind(
                socket.get(),
                address->ai_addr,
                static_cast<int>(address->ai_addrlen)) != 0) {
            final_error = last_socket_error();
            continue;
        }
        if (::listen(socket.get(), static_cast<int>(config.maximum_clients)) != 0) {
            final_error = last_socket_error();
            continue;
        }
        set_nonblocking(socket.get(), true);
        return socket.release();
    }
    throw socket_error(
        "WebSocket listen on " + config.bind_address + ':' +
            std::to_string(config.port),
        final_error);
}

}  // namespace

std::string make_websocket_accept_key(const std::string& client_key) {
    constexpr char websocket_guid[] =
        "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    const auto digest = sha1(client_key + websocket_guid);
    return base64(digest.data(), digest.size());
}

struct WebSocketServer::Impl {
    Impl(WebSocketServerConfig value, SocketHandle listener)
        : config(std::move(value)), listen_socket(listener) {}

    ~Impl() {
        for (const auto client : clients) {
            close_socket(client);
        }
        close_socket(listen_socket);
    }

    WebSocketServerConfig config;
    SocketHandle listen_socket{invalid_socket};
    std::vector<SocketHandle> clients;
};

WebSocketServer::WebSocketServer(std::unique_ptr<Impl> implementation)
    : implementation_(std::move(implementation)) {}

WebSocketServer WebSocketServer::listen(WebSocketServerConfig config) {
    if (config.bind_address.empty() || config.port == 0 ||
        config.maximum_clients == 0) {
        throw std::invalid_argument(
            "WebSocket bind address, port, and maximum clients are required");
    }
    const auto socket = create_listen_socket(config);
    return WebSocketServer(
        std::make_unique<Impl>(std::move(config), socket));
}

WebSocketServer::WebSocketServer(WebSocketServer&&) noexcept = default;
WebSocketServer& WebSocketServer::operator=(WebSocketServer&&) noexcept = default;
WebSocketServer::~WebSocketServer() = default;

std::size_t WebSocketServer::poll_accept() {
    std::size_t accepted{};
    for (;;) {
        SocketGuard client(::accept(
            implementation_->listen_socket, nullptr, nullptr));
        if (client.get() == invalid_socket) {
            const auto error = last_socket_error();
            if (would_block(error)) {
                break;
            }
            throw socket_error("accepting WebSocket client", error);
        }
        if (implementation_->clients.size() >=
            implementation_->config.maximum_clients) {
            continue;
        }

        try {
            set_nonblocking(client.get(), false);
            set_socket_timeout(
                client.get(),
                SO_RCVTIMEO,
                implementation_->config.handshake_timeout);
            set_socket_timeout(
                client.get(),
                SO_SNDTIMEO,
                implementation_->config.send_timeout);
            const auto request = read_handshake_request(client.get());
            const auto key = websocket_key_from_request(request);
            if (key.empty()) {
                continue;
            }
            const auto response =
                std::string("HTTP/1.1 101 Switching Protocols\r\n") +
                "Upgrade: websocket\r\n" +
                "Connection: Upgrade\r\n" +
                "Sec-WebSocket-Accept: " + make_websocket_accept_key(key) +
                "\r\n\r\n";
            if (!send_all(
                    client.get(),
                    reinterpret_cast<const std::uint8_t*>(response.data()),
                    response.size())) {
                continue;
            }
            implementation_->clients.push_back(client.release());
            ++accepted;
        } catch (const std::exception&) {
            // A malformed or abandoned browser handshake must not stop the
            // point-cloud stream or any sensor worker.
        }
    }
    return accepted;
}

std::size_t WebSocketServer::broadcast_binary(
    const std::uint8_t* data,
    std::size_t size) {
    const auto header = binary_frame_header(size);
    std::size_t deliveries{};
    auto& clients = implementation_->clients;
    for (auto iterator = clients.begin(); iterator != clients.end();) {
        const auto sent =
            send_all(*iterator, header.data(), header.size()) &&
            send_all(*iterator, data, size);
        if (!sent) {
            close_socket(*iterator);
            iterator = clients.erase(iterator);
        } else {
            ++deliveries;
            ++iterator;
        }
    }
    return deliveries;
}

std::size_t WebSocketServer::client_count() const noexcept {
    return implementation_->clients.size();
}

const WebSocketServerConfig& WebSocketServer::config() const noexcept {
    return implementation_->config;
}

}  // namespace vista::transport
