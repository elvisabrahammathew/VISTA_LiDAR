#include "2_Transport/messaging/http.hpp"

#include <array>
#include <cctype>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "2_Transport/peripheral/ethernet/ethernet.hpp"

namespace vista::transport {

namespace {

constexpr std::size_t maximum_response_bytes = 1024U * 1024U;

void validate_header_value(const std::string& value, const char* label) {
    if (value.empty()) {
        throw std::invalid_argument(std::string(label) + " cannot be empty");
    }
    for (const auto character : value) {
        if (character == '\r' || character == '\n') {
            throw std::invalid_argument(
                std::string(label) + " cannot contain a line break");
        }
    }
}

void validate_header_name(const std::string& name) {
    if (name.empty()) {
        throw std::invalid_argument("HTTP header name cannot be empty");
    }
    for (const auto character : name) {
        const auto value = static_cast<unsigned char>(character);
        if (std::isalnum(value) == 0 && character != '-') {
            throw std::invalid_argument(
                "HTTP header name may contain only letters, digits, and '-'");
        }
    }
}

HttpResponse parse_response(const std::string& response) {
    const auto first_line_end = response.find("\r\n");
    if (first_line_end == std::string::npos) {
        throw std::runtime_error("HTTP response has no status line");
    }

    std::istringstream status_line(response.substr(0, first_line_end));
    std::string version;
    HttpResponse parsed;
    if (!(status_line >> version >> parsed.status_code) ||
        version.rfind("HTTP/", 0) != 0) {
        throw std::runtime_error("invalid HTTP response status line");
    }
    std::getline(status_line, parsed.reason);
    while (!parsed.reason.empty() &&
           std::isspace(static_cast<unsigned char>(parsed.reason.front())) != 0) {
        parsed.reason.erase(parsed.reason.begin());
    }

    const auto header_end = response.find("\r\n\r\n", first_line_end);
    if (header_end != std::string::npos) {
        parsed.body = response.substr(header_end + 4);
    }
    return parsed;
}

}  // namespace

HttpClient::HttpClient(
    std::string host,
    std::uint16_t port,
    std::chrono::milliseconds timeout)
    : host_(std::move(host)), port_(port), timeout_(timeout) {
    validate_header_value(host_, "HTTP host");
    if (port_ == 0) {
        throw std::invalid_argument("HTTP port cannot be zero");
    }
    if (timeout_.count() <= 0) {
        throw std::invalid_argument("HTTP timeout must be positive");
    }
}

HttpResponse HttpClient::post(
    const std::string& path,
    const std::string& content_type,
    const std::string& body,
    const HttpHeaders& headers) const {
    if (path.empty() || path.front() != '/') {
        throw std::invalid_argument("HTTP path must begin with '/'");
    }
    validate_header_value(path, "HTTP path");
    validate_header_value(content_type, "HTTP content type");
    for (const auto& header : headers) {
        validate_header_name(header.first);
        validate_header_value(header.second, "HTTP header value");
    }

    std::ostringstream request;
    request << "POST " << path << " HTTP/1.1\r\n"
            << "Host: " << host_ << ':' << port_ << "\r\n"
            << "User-Agent: vista-sensor-processing/0.1\r\n"
            << "Content-Type: " << content_type << "\r\n";
    for (const auto& header : headers) {
        request << header.first << ": " << header.second << "\r\n";
    }
    request << "Content-Length: " << body.size() << "\r\n"
            << "Connection: close\r\n\r\n"
            << body;
    const auto request_text = request.str();

    auto connection = EthernetConnection::connect(
        host_, port_, timeout_, timeout_);
    connection.write_all(
        reinterpret_cast<const std::uint8_t*>(request_text.data()),
        request_text.size());

    std::string response_text;
    std::array<std::uint8_t, 4096> buffer{};
    for (;;) {
        const auto count = connection.read_some(buffer.data(), buffer.size());
        if (count == 0) {
            break;
        }
        if (response_text.size() + count > maximum_response_bytes) {
            throw std::runtime_error("HTTP response exceeded 1 MiB");
        }
        response_text.append(
            reinterpret_cast<const char*>(buffer.data()), count);
    }
    return parse_response(response_text);
}

}  // namespace vista::transport
