#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace vista::transport {

struct HttpResponse {
    int status_code{};
    std::string reason;
    std::string body;
};

using HttpHeaders = std::vector<std::pair<std::string, std::string>>;

/// Small HTTP/1.1 client for sending metrics to a local Grafana instance.
/// HTTPS is intentionally out of scope; deploy TLS through a reverse proxy.
class HttpClient {
public:
    HttpClient(
        std::string host,
        std::uint16_t port,
        std::chrono::milliseconds timeout);

    HttpResponse post(
        const std::string& path,
        const std::string& content_type,
        const std::string& body,
        const HttpHeaders& headers = {}) const;

private:
    std::string host_;
    std::uint16_t port_{};
    std::chrono::milliseconds timeout_{};
};

}  // namespace vista::transport
