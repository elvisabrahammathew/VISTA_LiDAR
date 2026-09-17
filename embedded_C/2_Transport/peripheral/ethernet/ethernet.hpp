#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace vista::transport {

/// Owns one cross-platform TCP connection used by an Ethernet sensor.
class EthernetConnection {
public:
    static EthernetConnection connect(
        const std::string& host,
        std::uint16_t port,
        std::chrono::milliseconds connect_timeout,
        std::chrono::milliseconds read_timeout);

    EthernetConnection(const EthernetConnection&) = delete;
    EthernetConnection& operator=(const EthernetConnection&) = delete;
    EthernetConnection(EthernetConnection&&) noexcept;
    EthernetConnection& operator=(EthernetConnection&&) noexcept;
    ~EthernetConnection();

    /// Reads exactly the requested byte count, including across TCP segments.
    void read_exact(std::uint8_t* output, std::size_t size);

private:
    struct Impl;
    explicit EthernetConnection(std::unique_ptr<Impl> implementation);
    std::unique_ptr<Impl> implementation_;
};

}  // namespace vista::transport
