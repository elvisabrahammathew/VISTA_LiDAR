#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace vista::transport {

/// Owns one cross-platform serial connection used by a USB serial sensor.
class SerialConnection {
public:
    static SerialConnection open(
        const std::string& port,
        std::uint32_t baud_rate,
        std::chrono::milliseconds read_timeout);

    SerialConnection(const SerialConnection&) = delete;
    SerialConnection& operator=(const SerialConnection&) = delete;
    SerialConnection(SerialConnection&&) noexcept;
    SerialConnection& operator=(SerialConnection&&) noexcept;
    ~SerialConnection();

    /// Reads at most size bytes and throws when no byte arrives before timeout.
    std::size_t read_some(std::uint8_t* output, std::size_t size);

    /// Sends every byte, including across partial operating-system writes.
    void write_all(const std::uint8_t* data, std::size_t size);

private:
    struct Impl;
    explicit SerialConnection(std::unique_ptr<Impl> implementation);
    std::unique_ptr<Impl> implementation_;
};

}  // namespace vista::transport
