#include <chrono>
#include <filesystem>
#include <optional>
#include <string>

#include "1_Platform/message_bus/message_bus.hpp"
#include "4_Applications/monitoring/system_monitor.hpp"
#include "models/telemetry.hpp"
#include "models/topics.hpp"
#include "unittest/test.hpp"

VISTA_TEST(system_monitor_publishes_health_without_a_sensor) {
    vista::platform::StopToken stop;
    vista::platform::MessageBus bus;
    auto subscriber = bus.subscribe<vista::models::SystemHealthTelemetry>(
        vista::models::topics::system_health, "system-monitor-test");
    std::optional<vista::application::SystemMonitorReport> report;
    std::string error;

    auto worker = vista::application::spawn_system_monitor(
        bus,
        vista::platform::ThreadConfig("test-system-monitor", 4),
        stop,
        vista::application::SystemMonitorConfig{
            std::chrono::milliseconds(10),
            std::filesystem::temp_directory_path(),
        },
        [&](auto completed_report, auto completed_error) {
            report = std::move(completed_report);
            error = std::move(completed_error);
        });

    std::shared_ptr<const vista::models::SystemHealthTelemetry> message;
    const auto status = subscriber.receive_for(message, std::chrono::seconds(1));
    stop.request_stop();
    bus.close();
    worker.join();

    VISTA_CHECK(status == vista::platform::ReceiveStatus::message);
    VISTA_CHECK(message && message->timestamp_ns > 0);
    VISTA_CHECK(message->worker_count >= 1);
    VISTA_CHECK(error.empty());
    VISTA_CHECK(report && report->sample_count >= 1);
}
