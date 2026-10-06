#include "4_Applications/grafana_bridge/grafana_bridge.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
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
#include "models/mapping/localization.hpp"

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
    // The dashboard needs counts/rate/age, not XYZ bounds. Avoid scanning every
    // point here; the independent 3D WebSocket still receives the full geometry.
    return value;
}

PointCloudTelemetry offline_pointcloud(const std::string& lidar_id) {
    PointCloudTelemetry value;
    if(lidar_id.empty()) throw std::invalid_argument("offline LiDAR telemetry needs a stable device ID");
    value.lidar_id = lidar_id;
    value.timestamp_ns = system_timestamp_ns();
    return value;
}

ImuTelemetry summarize_imu(
    const models::ImuMessage& message,
    double sample_rate_hz) {
    ImuTelemetry value;
    value.lidar_id = message.source_id; // Preserve the existing Grafana wire label.
    value.timestamp_ns = system_timestamp_ns();
    value.sample_rate_hz = sample_rate_hz;
    value.online = true;
    value.sample = message.payload;
    return value;
}

ImuTelemetry offline_imu(const std::string& lidar_id) {
    ImuTelemetry value;
    value.lidar_id = lidar_id;
    value.timestamp_ns = system_timestamp_ns();
    return value;
}

std::string format_system_health(const models::SystemHealthTelemetry& value) {
    auto line = line_stream();
    line << "system_health cpu_percent=" << value.cpu_percent
         << ",memory_mb=" << value.memory_mb
         << ",worker_count=" << value.worker_count << 'i'
         << ",failed_workers=" << value.failed_workers << 'i'
         << ' ' << value.timestamp_ns;
    return line.str();
}

struct PipelineTelemetry {
    std::uint64_t timestamp_ns{};
    double raw_fps{};
    double raw_fill{}, processed_fill{};
    std::uint64_t drops{};
};

std::string format_pipeline_health(const PipelineTelemetry& value) {
    auto line = line_stream();
    line << "pipeline_health raw_fps=" << value.raw_fps
         << ",raw_queue_fill_percent=" << value.raw_fill
         << ",processed_queue_fill_percent=" << value.processed_fill
         << ",dropped_messages=" << value.drops << 'i'
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

std::vector<std::string> format_worker_health_measurements(
    std::vector<models::WorkerHealthTelemetry> workers,
    std::uint64_t publication_timestamp_ns) {
    std::vector<std::string> lines;
    if (workers.empty()) return lines;
    constexpr std::uint64_t millisecond_ns = 1'000'000;
    const auto end_ms = publication_timestamp_ns / millisecond_ns;
    if (workers.size() > end_ms) {
        throw std::invalid_argument("worker snapshot timestamp cannot accommodate its rows");
    }
    std::sort(workers.begin(), workers.end(), [](const auto& a, const auto& b) {
        return a.worker_name < b.worker_name;
    });
    lines.reserve(workers.size());
    for (std::size_t index = 0; index < workers.size(); ++index) {
        const auto& value = workers[index];
        if (value.worker_name.empty() ||
            value.worker_name.find_first_of("\r\n") != std::string::npos ||
            value.worker_name.find('\0') != std::string::npos ||
            !std::isfinite(value.uptime_seconds) || value.uptime_seconds < 0 ||
            (index && value.worker_name == workers[index - 1].worker_name)) {
            throw std::invalid_argument("invalid or duplicate worker snapshot row");
        }
        std::string escaped;
        for (const auto character : value.worker_name) {
            if (character == '\\' || character == '"') escaped.push_back('\\');
            escaped.push_back(character);
        }
        // All rows are at/before publication, with no same-ms collisions.
        // A new measurement avoids stale tag-dependent schemas in Grafana Live.
        const auto timestamp = (end_ms - (workers.size() - 1 - index)) * millisecond_ns;
        auto line = line_stream();
        line << "worker_health_rows worker=\"" << escaped << "\""
             << ",running=" << (value.running ? "1i" : "0i")
             << ",failed=" << (value.failed ? "1i" : "0i")
             << ",priority=" << static_cast<unsigned>(value.priority) << 'i'
             << ",uptime_seconds=" << value.uptime_seconds << ' ' << timestamp;
        lines.push_back(line.str());
    }
    return lines;
}

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
    if(config.selected_lidar_id.empty()) throw std::invalid_argument("Grafana selected LiDAR ID must not be empty");
}

namespace {
std::string format_pointcloud(const PointCloudTelemetry& value,bool current) {
    auto line = line_stream();
    if(current) {
        // Compact, tag-free channel used by the current dashboard. Legacy
        // formatting below remains available as an API, but is not published.
        line << "pointcloud_current online=" << (value.online ? "1i" : "0i")
             << ",input_point_count=" << value.input_point_count << 'i'
             << ",point_count=" << value.point_count << 'i'
             << ",data_age_ms=" << value.processing_ms
             << ",message_rate_hz=" << value.frames_per_second
             << ' ' << value.timestamp_ns;
        return line.str();
    }
    line << (current ? "pointcloud_current" : "pointcloud");
    if(!current) line << ",lidar_id=" << escape_influx_tag(value.lidar_id);
    line << " online=" << (value.online ? "1i" : "0i")
         << ",input_point_count=" << value.input_point_count << 'i'
         << ",point_count=" << value.point_count << 'i'
         << ",has_points=" << (value.has_bounds ? "1i" : "0i")
         << ",fps=" << value.frames_per_second
         << ",processing_ms=" << value.processing_ms
         << ",dropped_messages=" << value.dropped_messages << 'i'
         << ",min_x_m=" << value.min_x_m << ",max_x_m=" << value.max_x_m
         << ",min_y_m=" << value.min_y_m << ",max_y_m=" << value.max_y_m
         << ",min_z_m=" << value.min_z_m << ",max_z_m=" << value.max_z_m;
    line << ' ' << value.timestamp_ns;
    return line.str();
}
} // namespace
std::string format_pointcloud_measurement(const PointCloudTelemetry& value) {return format_pointcloud(value,false);}
std::string format_current_pointcloud_measurement(const PointCloudTelemetry& value) {return format_pointcloud(value,true);}
PointCloudTelemetry make_offline_pointcloud_telemetry(const std::string& lidar_id) {return offline_pointcloud(lidar_id);}

std::string format_imu_measurement(const ImuTelemetry& value) {
    auto line = line_stream();
    line << "imu,lidar_id=" << escape_influx_tag(value.lidar_id)
         << " online=" << (value.online ? "1i" : "0i")
         << ",sample_rate_hz=" << value.sample_rate_hz
         << ",orientation_x=" << value.sample.orientation_x
         << ",orientation_y=" << value.sample.orientation_y
         << ",orientation_z=" << value.sample.orientation_z
         << ",orientation_w=" << value.sample.orientation_w
         << ",angular_velocity_x_rad_s="
         << value.sample.angular_velocity_x_rad_s
         << ",angular_velocity_y_rad_s="
         << value.sample.angular_velocity_y_rad_s
         << ",angular_velocity_z_rad_s="
         << value.sample.angular_velocity_z_rad_s
         << ",linear_acceleration_x_m_s2="
         << value.sample.linear_acceleration_x_m_s2
         << ",linear_acceleration_y_m_s2="
         << value.sample.linear_acceleration_y_m_s2
         << ",linear_acceleration_z_m_s2="
         << value.sample.linear_acceleration_z_m_s2
         << ' ' << value.timestamp_ns;
    return line.str();
}

std::string format_localization_measurement(const models::LocalizationStatus& value) {
    auto line=line_stream();
    std::string reason;
    for(char c:value.reason){if(c=='\\' || c=='"')reason.push_back('\\');if(c!='\n' && c!='\r')reason.push_back(c);}
    line << "localization state_code=" << static_cast<unsigned int>(value.state) << 'i'
         << ",pose_valid=" << (value.pose_valid?1:0) << 'i'
         << ",position_x_m=" << value.position_m[0] << ",position_y_m=" << value.position_m[1] << ",position_z_m=" << value.position_m[2]
         << ",match_ratio=" << value.match_ratio << ",residual_m=" << value.residual_m
         << ",position_variance=" << value.position_variance << ",local_map_points=" << value.local_map_points << 'i'
         << ",rejected_scans=" << value.rejected_scans << 'i' << ",dropped_messages=" << value.dropped_messages << 'i'
         << ",reason=\"" << reason << "\" " << system_timestamp_ns();
    return line.str();
}

std::string format_ground_measurement(
    const models::GroundStatusMessage& message) {
    const auto& value=message.payload;
    auto line=line_stream();
    line << "ground_status,lidar_id=" << escape_influx_tag(message.source_id)
         << ",mode=" << escape_influx_tag(value.configured_mode)
         << " ground_tilt_deg=" << value.ground_tilt_deg
         << ",ground_height_error_m=" << value.ground_height_error_m
         << ",ground_inlier_ratio=" << value.ground_inlier_ratio
         << ",rms_residual_m=" << value.rms_residual_m
         << ",removed_ground_ratio=" << value.removed_ground_ratio
         << ' ' << system_timestamp_ns();
    return line.str();
}

std::string format_ground_state_measurement(
    const models::GroundStatusMessage& message) {
    auto line=line_stream();
    line << "ground_state state_code="
         << static_cast<unsigned int>(message.payload.state) << 'i'
         << ' ' << system_timestamp_ns();
    return line.str();
}

std::string format_room_map_measurement(const models::RoomMapStatus& status) {
    auto line = line_stream();
    line << "room_map_status state_code=" << static_cast<unsigned int>(status.state) << 'i'
         << ",point_count=" << status.point_count << 'i'
         << ",candidate_voxels=" << status.candidate_voxels << 'i'
         << ",cache_max_voxels=" << status.cache_max_voxels << 'i'
         << ",cache_voxels=" << status.cache_voxels << 'i'
         << ",cache_tiles=" << status.cache_tiles << 'i'
         << ",cache_evictions=" << status.cache_evictions << 'i'
         << ",disk_tiles=" << status.disk_tiles << 'i'
         << ",cache_misses=" << status.cache_misses << 'i'
         << ",tile_reads=" << status.tile_reads << 'i'
         << ",tile_writes=" << status.tile_writes << 'i'
         << ",lod_node_writes=" << status.lod_node_writes << 'i'
         << ",view_queries=" << status.view_queries << 'i'
         << ",integrated_frames=" << status.integrated_frames << 'i'
         << ",preview_queries=" << status.preview_queries << 'i'
         << ",last_integration_ms=" << status.last_integration_ms
         << ",max_integration_ms=" << status.max_integration_ms
         << ",last_map_lock_wait_ms=" << status.last_map_lock_wait_ms
         << ",last_checkpoint_ms=" << status.last_checkpoint_ms
         << ",last_view_ms=" << status.last_view_ms
         << ",last_view_lock_wait_ms=" << status.last_view_lock_wait_ms
         << ",tile_read_total_ms=" << status.tile_read_total_ms
         << ",tile_write_total_ms=" << status.tile_write_total_ms
         << ",lod_update_total_ms=" << status.lod_update_total_ms
         << ",dropped_input_messages=" << status.dropped_input_messages << 'i'
         << ",active_build_seconds=" << status.active_build_seconds
         << ",raycasts=" << status.raycasts << 'i'
         << ",ray_budget_skipped_points=" << status.ray_budget_skipped_points << 'i'
         << ",ray_traversal_steps=" << status.ray_traversal_steps << 'i'
         << ",free_space_checks=" << status.free_space_checks << 'i'
         << ",cleared_voxels=" << status.cleared_voxels << 'i'
         << ",missing_origin_messages=" << status.missing_origin_messages << 'i'
         << ' ' << status.timestamp_ns;
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
    // Live health/counts are independent of world-pose validity. Keep the
    // existing compact Grafana field/channel names for dashboard compatibility.
    auto processed = inputs.subscribe<devices::LidarPointCloudMessage>(bus, models::topics::pointcloud_cleaned_sensor);
    auto imu = inputs.subscribe<models::ImuMessage>(bus, models::topics::lidar_imu);
    auto ground = inputs.subscribe<models::GroundStatusMessage>(bus, models::topics::ground_status);
    auto room_map = inputs.subscribe<models::RoomMapStatus>(bus, models::topics::room_map_status);
    auto localization = inputs.subscribe<models::LocalizationStatus>(bus, models::topics::localization_status);
    auto system = inputs.subscribe<models::SystemHealthTelemetry>(bus, models::topics::system_health);
    auto worker = inputs.subscribe<models::WorkerHealthTelemetry>(bus, models::topics::worker_health);
    // Storage/power remain available on the internal bus, but the current
    // dashboard has no consumers for them. Avoid subscribing/draining/sending.

    return platform::spawn_worker(
        std::move(thread_config),
        [stop, inputs = std::move(inputs), raw = std::move(raw),
         decoded = std::move(decoded), processed = std::move(processed),
         imu = std::move(imu), ground = std::move(ground),
         room_map = std::move(room_map),localization=std::move(localization),
         system = std::move(system), worker = std::move(worker),
         config = std::move(config), on_complete = std::move(on_complete)]() mutable {
            try {
                GrafanaBridgeReport report;
                GrafanaPublisher publisher(config, report);
                std::unordered_map<std::uint64_t, std::size_t> decoded_counts;
                std::unordered_map<std::string, models::WorkerHealthTelemetry> workers;
                std::optional<models::SystemHealthTelemetry> system_value;
                 std::shared_ptr<const devices::LidarPointCloudMessage> pending_cloud;
                 std::shared_ptr<const models::ImuMessage> pending_imu;
                 std::shared_ptr<const models::GroundStatusMessage> pending_ground;
                 std::optional<models::RoomMapStatus> pending_room_map;
                 std::optional<models::LocalizationStatus> pending_localization;
                 bool localization_closed{};
                 std::size_t input_count{};
                 std::string lidar_id=config.selected_lidar_id;
                 std::string imu_lidar_id;
                 std::uint64_t raw_time{}, processed_time{};
                 std::uint64_t imu_time{};
                 double raw_fps{}, processed_fps{};
                 double imu_rate_hz{};
                double raw_fill{}, processed_fill{};
                const auto started = Clock::now();
                 auto last_cloud = started;
                 auto last_imu = started;
                 auto next_publish = started;
                 bool raw_closed{}, decoded_closed{}, processed_closed{}, imu_closed{}, ground_closed{};
                bool system_closed{}, worker_closed{};
                bool room_map_closed{};

                while (!stop.is_stop_requested()) {
                    const auto ready = inputs.wait_for(std::chrono::milliseconds(100));
                    if (raw.is_ready(ready)) raw_fill = queue_fill_percent(raw.pending_messages(), raw.capacity());
                    if (processed.is_ready(ready)) processed_fill = queue_fill_percent(processed.pending_messages(), processed.capacity());

                    drain<devices::LidarRawMessage>(raw, raw_closed, ready, [&](const auto& message) {
                        ++report.received_raw_messages;
                        raw_fps = update_rate(message->received_timestamp_ns, raw_time, raw_fps);
                    });
                    drain<devices::LidarPointCloudMessage>(decoded, decoded_closed, ready, [&](const auto& message) {
                        ++report.received_decoded_messages;
                        decoded_counts[message->sequence] = message->payload.points.size();
                        if (decoded_counts.size() > 256) decoded_counts.clear();
                    });
                     drain<devices::LidarPointCloudMessage>(processed, processed_closed, ready, [&](const auto& message) {
                        ++report.received_processed_messages;
                        processed_fps = update_rate(message->received_timestamp_ns, processed_time, processed_fps);
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
                     drain<models::ImuMessage>(imu, imu_closed, ready,
                         [&](const auto& message) {
                             ++report.received_imu_messages;
                             imu_rate_hz = update_rate(
                                 message->received_timestamp_ns,
                                 imu_time,
                                 imu_rate_hz);
                             last_imu = Clock::now();
                             imu_lidar_id = message->source_id;
                             pending_imu = message;
                         });
                    drain<models::GroundStatusMessage>(ground, ground_closed, ready,
                        [&](const auto& message) {
                            ++report.received_ground_messages;
                            pending_ground=message;
                        });
                    drain<models::SystemHealthTelemetry>(system, system_closed, ready,
                        [&](const auto& message) { system_value = *message; });
                    drain<models::RoomMapStatus>(room_map, room_map_closed, ready,
                        [&](const auto& message) { pending_room_map = *message; });
                    drain<models::LocalizationStatus>(localization,localization_closed,ready,
                        [&](const auto& message){pending_localization=*message;});
                    drain<models::WorkerHealthTelemetry>(worker, worker_closed, ready,
                        [&](const auto& message) { workers[message->worker_name] = *message; });

                    const auto now = Clock::now();
                    if (now >= next_publish) {
                        const auto online = report.received_processed_messages!=0 && now - last_cloud < config.offline_timeout;
                        const auto drops = raw.dropped_messages() + decoded.dropped_messages() + processed.dropped_messages();
                        std::vector<std::string> lines;
                        if (pending_cloud) {
                            const auto value=summarize_point_cloud(*pending_cloud,input_count,drops,processed_fps);
                            lines.push_back(format_current_pointcloud_measurement(value));
                            pending_cloud.reset();
                         } else if (!online) {
                             auto value=offline_pointcloud(lidar_id);
                             value.dropped_messages=drops; // Cumulative bridge drops do not reset on disconnect.
                             lines.push_back(format_current_pointcloud_measurement(value));
                         }
                         const auto imu_online =
                             now - last_imu < config.offline_timeout;
                         if (pending_imu) {
                             lines.push_back(format_imu_measurement(
                                 summarize_imu(*pending_imu, imu_rate_hz)));
                             pending_imu.reset();
                         } else if (!imu_lidar_id.empty() && !imu_online) {
                             lines.push_back(format_imu_measurement(
                                 offline_imu(imu_lidar_id)));
                         }
                        if (pending_ground) {
                            lines.push_back(format_ground_state_measurement(*pending_ground));
                            lines.push_back(format_ground_measurement(*pending_ground));
                            pending_ground.reset();
                        }
                        lines.push_back(format_pipeline_health(PipelineTelemetry{
                            system_timestamp_ns(), online ? raw_fps : 0.0,
                            raw_fill, processed_fill, drops}));
                        if (pending_room_map) {
                            lines.push_back(format_room_map_measurement(*pending_room_map));
                            pending_room_map.reset();
                        }
                        if (system_value) lines.push_back(format_system_health(*system_value));
                        if(pending_localization){lines.push_back(format_localization_measurement(*pending_localization));pending_localization.reset();}
                        std::vector<models::WorkerHealthTelemetry> worker_rows;
                        worker_rows.reserve(workers.size());
                        for (const auto& entry : workers) worker_rows.push_back(entry.second);
                        auto worker_lines = format_worker_health_measurements(
                            std::move(worker_rows), system_timestamp_ns());
                        for (auto& line : worker_lines) lines.push_back(std::move(line));
                        publisher.publish(std::move(lines), inputs.topic_count());
                        next_publish = now + config.publish_interval;
                    }
                     if (raw_closed && decoded_closed && processed_closed && imu_closed && ground_closed && system_closed &&
                        worker_closed && room_map_closed && localization_closed) break;
                }
                 report.dropped_input_messages = raw.dropped_messages() + decoded.dropped_messages() +
                     processed.dropped_messages() + imu.dropped_messages() + ground.dropped_messages() +
                     system.dropped_messages() + worker.dropped_messages() +
                    room_map.dropped_messages()+localization.dropped_messages();
                on_complete(report, {});
            } catch (...) {
                stop.request_stop();
                on_complete(std::nullopt, current_exception_message());
            }
        });
}

}  // namespace vista::application
