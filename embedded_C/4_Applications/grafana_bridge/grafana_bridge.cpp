#include "4_Applications/grafana_bridge/grafana_bridge.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include "2_Transport/messaging/http.hpp"
#include "models/telemetry.hpp"
#include "models/topics.hpp"

namespace vista::application {
namespace {

using Clock = std::chrono::steady_clock;

std::uint64_t system_timestamp_ns() {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
}

std::string current_exception_message() {
    try { throw; }
    catch (const std::exception& error) { return error.what(); }
    catch (...) { return "unknown Grafana bridge failure"; }
}

std::optional<std::string> read_environment_variable(const char* name) {
#if defined(_WIN32)
    char* value = nullptr;
    std::size_t length = 0;
    if (_dupenv_s(&value, &length, name) != 0) {
        throw std::runtime_error("failed to read environment variable");
    }
    const std::unique_ptr<char, decltype(&std::free)> owned(value, &std::free);
    if (value == nullptr || length <= 1) {
        return std::nullopt;
    }
    return std::string(value);
#else
    const auto* value = std::getenv(name);
    if (value == nullptr || value[0] == '\0') {
        return std::nullopt;
    }
    return std::string(value);
#endif
}

std::string trim_copy(std::string value) {
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

std::optional<std::string> read_grafana_token_file() {
    std::ifstream input(grafana_secret_file_name);
    if (!input.is_open()) {
        return std::nullopt;
    }

    std::optional<std::string> token;
    std::string line;
    std::size_t line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        line = trim_copy(std::move(line));
        if (line.empty() || line.front() == '#') {
            continue;
        }
        const auto separator = line.find(':');
        if (separator == std::string::npos ||
            trim_copy(line.substr(0, separator)) != "GrafanaToken") {
            throw std::invalid_argument(
                std::string(grafana_secret_file_name) +
                " accepts only 'GrafanaToken: <glsa_...>' (line " +
                std::to_string(line_number) + ")");
        }
        if (token) {
            throw std::invalid_argument(
                std::string(grafana_secret_file_name) +
                " contains more than one GrafanaToken");
        }
        const auto value = trim_copy(line.substr(separator + 1));
        if (!value.empty()) {
            token = value;
        }
    }
    return token;
}

struct GrafanaAuthorization {
    std::string header;
    std::string source;
};

std::optional<GrafanaAuthorization> load_grafana_authorization() {
    const auto file_token = read_grafana_token_file();
    if (file_token) {
        return GrafanaAuthorization{
            make_grafana_bearer_authorization(*file_token),
            grafana_secret_file_name};
    }
    const auto token =
        read_environment_variable(grafana_token_environment_variable);
    if (token) {
        return GrafanaAuthorization{
            make_grafana_bearer_authorization(*token),
            grafana_token_environment_variable};
    }
    return std::nullopt;
}

std::string escape_influx_tag(const std::string& value) {
    std::string escaped;
    for (const auto character : value) {
        if (character == ',' || character == '=' || character == ' ') {
            escaped.push_back('\\');
        }
        escaped.push_back(character);
    }
    return escaped;
}

std::ostringstream line_stream() {
    std::ostringstream line;
    line.imbue(std::locale::classic());
    line << std::setprecision(9);
    return line;
}

double queue_fill_percent(std::size_t pending, std::size_t capacity) {
    return capacity == 0 ? 0.0
                         : 100.0 * static_cast<double>(pending) /
                               static_cast<double>(capacity);
}

double update_rate(
    std::uint64_t timestamp,
    std::uint64_t& previous_timestamp,
    double previous_rate) {
    auto result = previous_rate;
    if (previous_timestamp != 0 && timestamp > previous_timestamp) {
        result = 1'000'000'000.0 /
                 static_cast<double>(timestamp - previous_timestamp);
    }
    previous_timestamp = timestamp;
    return result;
}

PointCloudTelemetry summarize_point_cloud(
    const devices::LidarPointCloudMessage& message,
    std::size_t input_count,
    std::uint64_t drops,
    double fps) {
    PointCloudTelemetry value;
    value.lidar_id = message.lidar_id;
    value.timestamp_ns = system_timestamp_ns();
    value.input_point_count = input_count;
    value.point_count = message.payload.points.size();
    value.dropped_messages = drops;
    value.frames_per_second = fps;
    value.online = true;
    if (value.timestamp_ns >= message.received_timestamp_ns) {
        value.processing_ms = static_cast<double>(
            value.timestamp_ns - message.received_timestamp_ns) / 1'000'000.0;
    }
    if (!message.payload.points.empty()) {
        const auto& first = message.payload.points.front();
        value.has_bounds = true;
        value.min_x_m = value.max_x_m = first.x;
        value.min_y_m = value.max_y_m = first.y;
        value.min_z_m = value.max_z_m = first.z;
        for (const auto& point : message.payload.points) {
            value.min_x_m = std::min(value.min_x_m, point.x);
            value.max_x_m = std::max(value.max_x_m, point.x);
            value.min_y_m = std::min(value.min_y_m, point.y);
            value.max_y_m = std::max(value.max_y_m, point.y);
            value.min_z_m = std::min(value.min_z_m, point.z);
            value.max_z_m = std::max(value.max_z_m, point.z);
        }
    }
    return value;
}

PointCloudTelemetry offline_pointcloud(const std::string& lidar_id) {
    PointCloudTelemetry value;
    value.lidar_id = lidar_id.empty() ? "selected-lidar" : lidar_id;
    value.timestamp_ns = system_timestamp_ns();
    return value;
}

std::string format_system_health(const models::SystemHealthTelemetry& value) {
    auto line = line_stream();
    line << "system_health online=1i,uptime_seconds=" << value.uptime_seconds
         << ",cpu_percent=" << value.cpu_percent
         << ",memory_mb=" << value.memory_mb
         << ",memory_percent=" << value.memory_percent
         << ",worker_count=" << value.worker_count << 'i'
         << ",failed_workers=" << value.failed_workers << 'i'
         << ' ' << value.timestamp_ns;
    return line.str();
}

std::string format_worker_health(const models::WorkerHealthTelemetry& value) {
    auto line = line_stream();
    line << "worker_health,worker=" << escape_influx_tag(value.worker_name)
         << " running=" << (value.running ? "1i" : "0i")
         << ",failed=" << (value.failed ? "1i" : "0i")
         << ",priority=" << static_cast<unsigned int>(value.priority) << 'i'
         << ",uptime_seconds=" << value.uptime_seconds
         << ' ' << value.timestamp_ns;
    return line.str();
}

std::string format_storage_health(const models::StorageHealthTelemetry& value) {
    auto line = line_stream();
    line << "storage_health disk_free_gb=" << value.disk_free_gb
         << ",raw_bytes_written=" << value.raw_bytes_written << 'i'
         << ",raw_messages_written=" << value.raw_messages_written << 'i'
         << ",pcd_points_written=" << value.pcd_points_written << 'i'
         << ",pcd_frames_written=" << value.pcd_frames_written << 'i'
         << ",write_errors=" << value.write_errors << 'i'
         << ' ' << value.timestamp_ns;
    return line.str();
}

std::string format_power_thermal(const models::PowerThermalTelemetry& value) {
    auto line = line_stream();
    line << "power_thermal available=" << (value.available ? "1i" : "0i")
         << ",cpu_temperature_c=" << value.cpu_temperature_c
         << ",gpu_temperature_c=" << value.gpu_temperature_c
         << ",board_power_w=" << value.board_power_w
         << ",fan_rpm=" << value.fan_rpm
         << ' ' << value.timestamp_ns;
    return line.str();
}

struct PipelineTelemetry {
    std::uint64_t timestamp_ns{};
    bool online{};
    double raw_fps{}, decoded_fps{}, processed_fps{};
    double latency_ms{};
    double raw_fill{}, decoded_fill{}, processed_fill{};
    std::uint64_t drops{};
    double last_message_age_ms{};
};

std::string format_pipeline_health(const PipelineTelemetry& value) {
    auto line = line_stream();
    line << "pipeline_health online=" << (value.online ? "1i" : "0i")
         << ",raw_fps=" << value.raw_fps
         << ",decoded_fps=" << value.decoded_fps
         << ",processed_fps=" << value.processed_fps
         << ",end_to_end_latency_ms=" << value.latency_ms
         << ",raw_queue_fill_percent=" << value.raw_fill
         << ",decoded_queue_fill_percent=" << value.decoded_fill
         << ",processed_queue_fill_percent=" << value.processed_fill
         << ",dropped_messages=" << value.drops << 'i'
         << ",last_message_age_ms=" << value.last_message_age_ms
         << ' ' << value.timestamp_ns;
    return line.str();
}

class GrafanaPublisher {
public:
    GrafanaPublisher(GrafanaBridgeConfig config, GrafanaBridgeReport& report)
        : config_(std::move(config)),
          client_(config_.host, config_.port, config_.request_timeout),
          path_("/api/live/push/" + config_.name_space),
          report_(report) {
        try {
            const auto authorization = load_grafana_authorization();
            if (authorization) {
                headers_.emplace_back("Authorization", authorization->header);
                std::cout << "Grafana authentication: Bearer token loaded from "
                          << authorization->source << '\n';
            } else {
                std::cerr
                    << "Grafana authentication warning: neither "
                    << grafana_token_environment_variable << " nor "
                    << grafana_secret_file_name
                    << " contains a token; requests will be sent without one.\n";
            }
        } catch (const std::exception& error) {
            std::cerr << "Grafana authentication warning: environment variable "
                      << "or secret file is invalid (" << error.what()
                      << "); requests will be sent without a token.\n";
        }
    }

    void publish(std::vector<std::string> lines, std::size_t topic_count) {
        const auto now = Clock::now();
        if (now < next_retry_) return;
        lines.push_back(health_line(topic_count));
        std::string body;
        for (const auto& line : lines) body += line + '\n';
        try {
            const auto response = client_.post(
                path_, "text/plain; charset=utf-8", body, headers_);
            last_http_status_ = response.status_code;
            if (response.status_code < 200 || response.status_code >= 300) {
                auto message = "Grafana returned HTTP " +
                    std::to_string(response.status_code) + " " + response.reason;
                if (response.status_code == 401) {
                    message += " (set a valid ";
                    message += grafana_token_environment_variable;
                    message += " or ";
                    message += grafana_secret_file_name;
                    message += " token and restart the application)";
                }
                throw std::runtime_error(message);
            }
            report_.published_measurements += lines.size();
            last_success_ = now;
            next_retry_ = Clock::time_point{};
            if (retrying_) {
                std::cout << "Grafana connection restored: http://"
                          << config_.host << ':' << config_.port << path_ << '\n';
            }
            retrying_ = false;
        } catch (const std::exception& error) {
            ++report_.failed_publish_attempts;
            next_retry_ = now + config_.retry_interval;
            if (!retrying_) {
                std::cerr << "Grafana publish failed: " << error.what()
                          << ". Retrying in "
                          << static_cast<double>(config_.retry_interval.count()) /
                                 1000.0 << " second(s).\n";
            }
            retrying_ = true;
        }
    }

private:
    std::string health_line(std::size_t topic_count) const {
        double success_age_ms = -1.0;
        if (last_success_) {
            success_age_ms = std::chrono::duration<double, std::milli>(
                Clock::now() - *last_success_).count();
        }
        auto line = line_stream();
        line << "grafana_bridge_health published_measurements="
             << report_.published_measurements << 'i'
             << ",failed_publish_attempts="
             << report_.failed_publish_attempts << 'i'
             << ",last_http_status=" << last_http_status_ << 'i'
             << ",last_success_age_ms=" << success_age_ms
             << ",retrying=" << (retrying_ ? "1i" : "0i")
             << ",subscribed_topics=" << topic_count << 'i'
             << ' ' << system_timestamp_ns();
        return line.str();
    }

    GrafanaBridgeConfig config_;
    transport::HttpClient client_;
    transport::HttpHeaders headers_;
    std::string path_;
    GrafanaBridgeReport& report_;
    Clock::time_point next_retry_{};
    std::optional<Clock::time_point> last_success_;
    int last_http_status_{};
    bool retrying_{};
};

template <typename Message, typename Input, typename Handler>
void drain(Input& input, bool& closed, platform::TopicWaitSet::Mask ready, Handler handler) {
    if (closed || !input.is_ready(ready)) return;
    std::shared_ptr<const Message> message;
    for (;;) {
        const auto status = input.try_receive(message);
        if (status == platform::ReceiveStatus::message) handler(message);
        else {
            closed = status == platform::ReceiveStatus::closed;
            return;
        }
    }
}

}  // namespace

std::string make_grafana_bearer_authorization(const std::string& token) {
    if (token.empty()) {
        throw std::invalid_argument("Grafana bearer token cannot be empty");
    }
    if (std::any_of(token.begin(), token.end(), [](unsigned char character) {
            return std::isspace(character) != 0;
        })) {
        throw std::invalid_argument(
            "Grafana bearer token cannot contain whitespace");
    }
    if (token.rfind("glsa_", 0) != 0) {
        throw std::invalid_argument(
            "Grafana service-account token must begin with 'glsa_'");
    }
    return "Bearer " + token;
}

void validate_grafana_bridge_config(const GrafanaBridgeConfig& config) {
    if (config.host.empty() || config.port == 0) {
        throw std::invalid_argument("Grafana host and port must be configured");
    }
    if (config.name_space.empty() || config.name_space.size() > 64 ||
        !std::all_of(config.name_space.begin(), config.name_space.end(),
            [](unsigned char value) {
                return std::isalnum(value) != 0 || value == '_' || value == '-';
            })) {
        throw std::invalid_argument(
            "Grafana namespace must contain only letters, digits, '-' or '_'"
            " and be at most 64 characters");
    }
    if (config.publish_interval.count() <= 0 || config.retry_interval.count() <= 0 ||
        config.request_timeout.count() <= 0 || config.offline_timeout.count() <= 0) {
        throw std::invalid_argument("Grafana timing values must be positive");
    }
}

std::string format_pointcloud_measurement(const PointCloudTelemetry& value) {
    auto line = line_stream();
    line << "pointcloud,lidar_id=" << escape_influx_tag(value.lidar_id)
         << " online=" << (value.online ? "1i" : "0i")
         << ",input_point_count=" << value.input_point_count << 'i'
         << ",point_count=" << value.point_count << 'i'
         << ",has_points=" << (value.has_bounds ? "1i" : "0i")
         << ",fps=" << value.frames_per_second
         << ",processing_ms=" << value.processing_ms
         << ",dropped_messages=" << value.dropped_messages << 'i'
         << ",min_x_m=" << value.min_x_m << ",max_x_m=" << value.max_x_m
         << ",min_y_m=" << value.min_y_m << ",max_y_m=" << value.max_y_m
         << ",min_z_m=" << value.min_z_m << ",max_z_m=" << value.max_z_m
         << ' ' << value.timestamp_ns;
    return line.str();
}

platform::WorkerHandle spawn_grafana_bridge(
    platform::MessageBus& bus,
    platform::ThreadConfig thread_config,
    platform::StopToken stop,
    GrafanaBridgeConfig config,
    GrafanaBridgeCompletion on_complete) {
    validate_grafana_bridge_config(config);
    platform::WorkerTopicInputs inputs(thread_config.name);
    auto raw = inputs.subscribe<devices::LidarRawMessage>(bus, models::topics::lidar_raw);
    auto decoded = inputs.subscribe<devices::LidarPointCloudMessage>(bus, models::topics::pointcloud_decoded);
    auto processed = inputs.subscribe<devices::LidarPointCloudMessage>(bus, models::topics::pointcloud_processed);
    auto system = inputs.subscribe<models::SystemHealthTelemetry>(bus, models::topics::system_health);
    auto worker = inputs.subscribe<models::WorkerHealthTelemetry>(bus, models::topics::worker_health);
    auto storage = inputs.subscribe<models::StorageHealthTelemetry>(bus, models::topics::storage_health);
    auto power = inputs.subscribe<models::PowerThermalTelemetry>(bus, models::topics::power_thermal);

    return platform::spawn_worker(
        std::move(thread_config),
        [stop, inputs = std::move(inputs), raw = std::move(raw),
         decoded = std::move(decoded), processed = std::move(processed),
         system = std::move(system), worker = std::move(worker),
         storage = std::move(storage), power = std::move(power),
         config = std::move(config), on_complete = std::move(on_complete)]() mutable {
            try {
                GrafanaBridgeReport report;
                GrafanaPublisher publisher(config, report);
                std::unordered_map<std::uint64_t, std::size_t> decoded_counts;
                std::unordered_map<std::string, models::WorkerHealthTelemetry> workers;
                std::optional<models::SystemHealthTelemetry> system_value;
                std::optional<models::StorageHealthTelemetry> storage_value;
                std::optional<models::PowerThermalTelemetry> power_value;
                std::shared_ptr<const devices::LidarPointCloudMessage> pending_cloud;
                std::size_t input_count{};
                std::string lidar_id;
                std::uint64_t raw_time{}, decoded_time{}, processed_time{};
                double raw_fps{}, decoded_fps{}, processed_fps{}, latency_ms{};
                double raw_fill{}, decoded_fill{}, processed_fill{};
                const auto started = Clock::now();
                auto last_cloud = started;
                auto next_publish = started;
                bool raw_closed{}, decoded_closed{}, processed_closed{};
                bool system_closed{}, worker_closed{}, storage_closed{}, power_closed{};

                while (!stop.is_stop_requested()) {
                    const auto ready = inputs.wait_for(std::chrono::milliseconds(100));
                    if (raw.is_ready(ready)) raw_fill = queue_fill_percent(raw.pending_messages(), raw.capacity());
                    if (decoded.is_ready(ready)) decoded_fill = queue_fill_percent(decoded.pending_messages(), decoded.capacity());
                    if (processed.is_ready(ready)) processed_fill = queue_fill_percent(processed.pending_messages(), processed.capacity());

                    drain<devices::LidarRawMessage>(raw, raw_closed, ready, [&](const auto& message) {
                        ++report.received_raw_messages;
                        raw_fps = update_rate(message->received_timestamp_ns, raw_time, raw_fps);
                    });
                    drain<devices::LidarPointCloudMessage>(decoded, decoded_closed, ready, [&](const auto& message) {
                        ++report.received_decoded_messages;
                        decoded_fps = update_rate(message->received_timestamp_ns, decoded_time, decoded_fps);
                        decoded_counts[message->sequence] = message->payload.points.size();
                        if (decoded_counts.size() > 256) decoded_counts.clear();
                    });
                    drain<devices::LidarPointCloudMessage>(processed, processed_closed, ready, [&](const auto& message) {
                        ++report.received_processed_messages;
                        processed_fps = update_rate(message->received_timestamp_ns, processed_time, processed_fps);
                        const auto now_ns = system_timestamp_ns();
                        latency_ms = now_ns >= message->received_timestamp_ns
                            ? static_cast<double>(now_ns - message->received_timestamp_ns) / 1'000'000.0 : 0.0;
                        last_cloud = Clock::now();
                        lidar_id = message->lidar_id;
                        input_count = message->payload.points.size();
                        const auto found = decoded_counts.find(message->sequence);
                        if (found != decoded_counts.end()) {
                            input_count = found->second;
                            decoded_counts.erase(found);
                        }
                        pending_cloud = message;
                    });
                    drain<models::SystemHealthTelemetry>(system, system_closed, ready,
                        [&](const auto& message) { system_value = *message; });
                    drain<models::WorkerHealthTelemetry>(worker, worker_closed, ready,
                        [&](const auto& message) { workers[message->worker_name] = *message; });
                    drain<models::StorageHealthTelemetry>(storage, storage_closed, ready,
                        [&](const auto& message) { storage_value = *message; });
                    drain<models::PowerThermalTelemetry>(power, power_closed, ready,
                        [&](const auto& message) { power_value = *message; });

                    const auto now = Clock::now();
                    if (now >= next_publish) {
                        const auto online = now - last_cloud < config.offline_timeout;
                        const auto drops = raw.dropped_messages() + decoded.dropped_messages() + processed.dropped_messages();
                        std::vector<std::string> lines;
                        if (pending_cloud) {
                            lines.push_back(format_pointcloud_measurement(
                                summarize_point_cloud(*pending_cloud, input_count, drops, processed_fps)));
                            pending_cloud.reset();
                        } else if (!online) {
                            lines.push_back(format_pointcloud_measurement(offline_pointcloud(lidar_id)));
                        }
                        lines.push_back(format_pipeline_health(PipelineTelemetry{
                            system_timestamp_ns(), online,
                            online ? raw_fps : 0.0, online ? decoded_fps : 0.0,
                            online ? processed_fps : 0.0, online ? latency_ms : 0.0,
                            raw_fill, decoded_fill, processed_fill, drops,
                            std::chrono::duration<double, std::milli>(now - last_cloud).count()}));
                        if (system_value) lines.push_back(format_system_health(*system_value));
                        for (const auto& entry : workers) lines.push_back(format_worker_health(entry.second));
                        if (storage_value) lines.push_back(format_storage_health(*storage_value));
                        if (power_value) lines.push_back(format_power_thermal(*power_value));
                        publisher.publish(std::move(lines), inputs.topic_count());
                        next_publish = now + config.publish_interval;
                    }
                    if (raw_closed && decoded_closed && processed_closed && system_closed &&
                        worker_closed && storage_closed && power_closed) break;
                }
                report.dropped_input_messages = raw.dropped_messages() + decoded.dropped_messages() +
                    processed.dropped_messages() + system.dropped_messages() + worker.dropped_messages() +
                    storage.dropped_messages() + power.dropped_messages();
                on_complete(report, {});
            } catch (...) {
                stop.request_stop();
                on_complete(std::nullopt, current_exception_message());
            }
        });
}

}  // namespace vista::application
