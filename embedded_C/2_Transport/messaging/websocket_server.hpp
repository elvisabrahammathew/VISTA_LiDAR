#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace vista::transport {

struct WebSocketServerConfig {
    std::string bind_address{"127.0.0.1"};
    std::uint16_t port{8765};
    std::size_t maximum_clients{4};
    std::chrono::milliseconds handshake_timeout{1'000};
    std::chrono::milliseconds send_timeout{250};
};
struct WebSocketTextMessage {
    std::uint64_t client_id{};
    std::string text;
};

/// Creates the RFC 6455 Sec-WebSocket-Accept value for one client key.
std::string make_websocket_accept_key(const std::string& client_key);

/// Minimal cross-platform WebSocket server for unmasked server-to-browser
/// binary frames. Networking stays in the transport layer; applications only
/// provide a binary payload.
class WebSocketServer {
public:
    static WebSocketServer listen(WebSocketServerConfig config);

    WebSocketServer(const WebSocketServer&) = delete;
    WebSocketServer& operator=(const WebSocketServer&) = delete;
    WebSocketServer(WebSocketServer&&) noexcept;
    WebSocketServer& operator=(WebSocketServer&&) noexcept;
    ~WebSocketServer();

    /// Accepts every currently pending browser connection without blocking.
    std::size_t poll_accept();

    /// Sends one binary WebSocket message and removes disconnected clients.
    /// Returns the number of clients that received the complete message.
    std::size_t broadcast_binary(const std::uint8_t* data, std::size_t size);

    /// Bounded, masked browser text frames; partial TCP reads are buffered.
    std::vector<WebSocketTextMessage> poll_text();
    std::vector<std::uint64_t> client_ids() const;
    bool send_binary(std::uint64_t client_id, const std::uint8_t* data, std::size_t size);
    bool send_text(std::uint64_t client_id, const std::string& text);

    std::size_t client_count() const noexcept;
    const WebSocketServerConfig& config() const noexcept;

private:
    struct Impl;
    explicit WebSocketServer(std::unique_ptr<Impl> implementation);
    std::unique_ptr<Impl> implementation_;
    bool send_frame(std::uint64_t client_id, std::uint8_t opcode,
        const std::uint8_t* data, std::size_t size);
};

}  // namespace vista::transport
