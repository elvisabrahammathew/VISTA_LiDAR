#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace vista::transport {

/// Owns a UDP endpoint bound to the PC and connected to one sensor endpoint.
class UdpConnection {
public:
    static UdpConnection connect(
        const std::string& local_ip,
        std::uint16_t local_port,
        const std::string& remote_ip,
        std::uint16_t remote_port,
        std::chrono::milliseconds read_timeout);

    UdpConnection(const UdpConnection&) = delete;
    UdpConnection& operator=(const UdpConnection&) = delete;
    UdpConnection(UdpConnection&&) noexcept;
    UdpConnection& operator=(UdpConnection&&) noexcept;
    ~UdpConnection();

    /// Receives one UDP datagram from the configured sensor.
    std::size_t receive(std::uint8_t* output, std::size_t capacity);

    /// Sends one UDP datagram to the configured sensor.
    void send(const std::uint8_t* data, std::size_t size);

private:
    struct Impl;
    explicit UdpConnection(std::unique_ptr<Impl> implementation);
    std::unique_ptr<Impl> implementation_;
};

}  // namespace vista::transport
