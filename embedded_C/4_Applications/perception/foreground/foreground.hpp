#pragma once

#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include "1_Platform/message_bus/message_bus.hpp"
#include "1_Platform/threading/threading.hpp"
#include "4_Applications/pointcloud_websocket/pointcloud_websocket.hpp"
#include "models/perception/foreground.hpp"

namespace vista::application {

struct ForegroundConfig {
    bool enabled{false};
    float background_distance_m{0.15F};
    std::size_t maximum_input_points{12'000};
    std::chrono::milliseconds publish_interval{250}; // One sample/process/publication cadence.
    // Derived from RoomMap RAM budgets, not a whole-map point limit.
    std::size_t maximum_cached_points{250'000}, maximum_cached_regions{128};
    // Cold disk work is capped per frame; unresolved regions are UNKNOWN.
    std::size_t maximum_region_reads_per_frame{8};
    bool require_localization{false};
    // Same-scan spatial support only; never gates RoomMap or retains motion history.
    bool outlier_filter_enabled{true};
    float outlier_radius_m{0.15F};
    // Linear interpolation from base at start to max at end; sensor-relative range.
    float outlier_start_distance_m{1.0F}, outlier_end_distance_m{5.0F}, outlier_max_radius_m{0.30F};
    std::size_t outlier_minimum_neighbors{2}; // Distinct positions, excluding self.
};
void validate_foreground_config(const ForegroundConfig& config);

struct ForegroundScanFilterReport {
    std::size_t input_points{}, retained_points{}, isolated_points{}, unresolved_points{}, distance_checks{};
    std::size_t adaptive_points{}, missing_origin_points{};
};
class ForegroundScanFilter {
public:
    explicit ForegroundScanFilter(ForegroundConfig config);
    ~ForegroundScanFilter();
    ForegroundScanFilter(const ForegroundScanFilter&)=delete;
    ForegroundScanFilter& operator=(const ForegroundScanFilter&)=delete;
    // Mutates only the worker-local bounded sample; no shared input or map writes.
    // Missing/nonfinite origin uses the fixed base radius, never the world origin.
    ForegroundScanFilterReport filter(std::vector<models::ForegroundPoint>& points,
        std::optional<std::array<float,3>> sensor_origin_world_m = std::nullopt);
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

class ForegroundEngine {
public:
    struct PrefetchReport { std::size_t region_reads{},cache_points{},cache_regions{}; };
    explicit ForegroundEngine(ForegroundConfig config);
    ~ForegroundEngine();
    ForegroundEngine(const ForegroundEngine&)=delete;
    ForegroundEngine& operator=(const ForegroundEngine&)=delete;
    models::ForegroundFrame process(const models::ForegroundObservation& observation);
    // Same worker, bounded cold reads in idle time. Soft deadline checked between
    // reads/probes; one synchronous region read cannot be preempted.
    PrefetchReport prefetch(const models::ForegroundObservation& observation,
        std::chrono::steady_clock::time_point deadline,std::size_t maximum_reads=2);
    void reset();
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
struct ForegroundReport {
    std::uint64_t received_messages{}, published_frames{}, foreground_frames{}, foreground_points{}, dropped_input_messages{};
    std::uint64_t processed_frames{}; // Actual queries; input arrivals are intentionally coalesced.
    std::uint64_t background_points{}, unknown_points{}, query_failures{};
    std::uint64_t isolated_points{}, filter_unresolved_points{}, filter_distance_checks{};
    std::size_t peak_cache_points{}, peak_cache_regions{};
    double processing_ms{}, maximum_processing_ms{};
    std::uint64_t prefetch_region_reads{},prefetch_failures{};
    double prefetch_ms{};
};
using ForegroundCompletion=std::function<void(std::optional<ForegroundReport>,std::string)>;
platform::WorkerHandle spawn_foreground_worker(platform::MessageBus& bus,
    platform::ThreadConfig thread,platform::StopToken stop,ForegroundConfig config,ForegroundCompletion complete);
platform::WorkerHandle spawn_foreground_websocket(platform::MessageBus& bus,
    platform::ThreadConfig thread,platform::StopToken stop,PointCloudWebSocketConfig config,PointCloudWebSocketCompletion complete);

} // namespace vista::application
