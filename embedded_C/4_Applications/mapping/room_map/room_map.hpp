#pragma once

#include <chrono>
#include <cstddef>
#include "1_Platform/compat/filesystem.hpp"
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "1_Platform/message_bus/message_bus.hpp"
#include "1_Platform/threading/threading.hpp"
#include "models/lidars/pointcloud.hpp"
#include "models/room_map.hpp"

namespace vista::transport { class RoomMapSessionFile; }
namespace vista::application {

struct RoomMapConfig {
    bool enabled{false};
    float voxel_size_m{0.05F};
    std::size_t cache_max_voxels{250'000}; // RAM working set, NOT total map size.
    std::size_t cache_max_tiles{128};
    float tile_size_m{2.0F};
    std::size_t lod_points_per_node{128};
    std::size_t preview_max_points{100'000};
    vista::fs::path storage_directory;
    std::size_t minimum_observations{3};
    std::chrono::milliseconds observation_interval{100};
    std::chrono::milliseconds freeze_after{30'000};  // Zero keeps integrating.
    std::chrono::milliseconds publish_interval{1'000};
    bool load_existing{false};
    /// Loading: user's input path. Building: session's preferred output filename.
    vista::fs::path file;
    /// Only actual measured rays may remove voxels; absence/occlusion is not evidence.
    std::size_t free_space_minimum_observations{5};
    std::size_t raycast_maximum_rays_per_window{256};
    float raycast_maximum_range_m{30.0F};
    float raycast_surface_margin_m{0.10F};
};

/// Rejects unsafe memory limits and invalid timing/voxel settings before startup.
void validate_room_map_config(const RoomMapConfig& config);

/// Disk-backed world-coordinate map with bounded RAM. Hit/free evidence is counted in distinct windows;
/// confirmed occluded geometry is retained, while measured free space can clear it.
class RoomMapAccumulator {
public:
    explicit RoomMapAccumulator(RoomMapConfig config);
    ~RoomMapAccumulator();
    RoomMapAccumulator(const RoomMapAccumulator&) = delete;
    RoomMapAccumulator& operator=(const RoomMapAccumulator&) = delete;

    /// Elapsed time is monotonic active acquisition time, independent of sensor timestamps.
    bool integrate(const models::PointCloudFrame& cloud, std::chrono::milliseconds elapsed);
    /// Returns a bounded overview of confirmed geometry; save() exports the complete map.
    models::PointCloudFrame snapshot() const;
    /// Flushes changed tiles and exports every confirmed point, without a full-map vector.
    void save(transport::RoomMapSessionFile& output);
    void load(const vista::fs::path& input);
    void flush();
    /// Checkpoints changed pages once; the returned disk reader does not lock live RAM.
    std::shared_ptr<const models::RoomMapViewSource> view_source() const;
    /// Reports global confirmation progress and RAM/disk cache diagnostics for Grafana.
    models::RoomMapStatus status() const;
    /// Stops geometry updates without discarding the current map.
    void freeze();
    /// Continues acquisition and starts a fresh auto-freeze interval.
    void resume();
    /// Clears memory and restarts confirmation; does not delete the saved PCD.
    void reset();
    /// Restores a read-only reference map. It cannot be resumed into a building map.
    void restore(const models::PointCloudFrame& cloud);

private:
    class Impl;
    std::shared_ptr<Impl> impl_;
};

struct RoomMapReport {
    std::uint64_t received_messages{};
    std::uint64_t dropped_input_messages{};
    std::uint64_t published_snapshots{};
    std::size_t map_points{};
};

using RoomMapCompletion = std::function<void(std::optional<RoomMapReport>, std::string)>;

/// Owns its subscriptions/publications. Frozen snapshots are republished using the
/// same immutable shared object, allowing a late subscriber to obtain the map.
platform::WorkerHandle spawn_room_map_worker(
    platform::MessageBus& bus, platform::ThreadConfig thread_config,
    platform::StopToken stop, RoomMapConfig config, RoomMapCompletion on_complete);

}  // namespace vista::application
