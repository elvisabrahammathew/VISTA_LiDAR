#pragma once

#include <cstddef>
#include <cstdint>
#include <array>
#include <memory>
#include "models/lidars/pointcloud.hpp"

namespace vista::models {

// Stable numeric codes are consumed by the Grafana dashboard.
enum class RoomMapState : std::uint8_t {
    empty = 0, building = 1, frozen = 2, loaded = 3, disabled = 4, load_error = 5, storage_error = 6
};

/// Mapping diagnostics are separate from sensor connection and ground quality.
struct RoomMapStatus {
    std::uint64_t timestamp_ns{};
    RoomMapState state{RoomMapState::empty};
    std::size_t point_count{};
    std::size_t candidate_voxels{};
    std::size_t cache_max_voxels{};
    std::size_t cache_voxels{};
    std::size_t cache_tiles{};
    std::uint64_t cache_evictions{};
    std::uint64_t disk_tiles{};
    std::uint64_t cache_misses{}, tile_reads{}, tile_writes{}, lod_node_writes{};
    std::uint64_t view_queries{}, integrated_frames{}, preview_queries{};
    double last_integration_ms{}, max_integration_ms{}, last_map_lock_wait_ms{};
    double last_checkpoint_ms{}, last_view_ms{}, last_view_lock_wait_ms{};
    double tile_read_total_ms{},tile_write_total_ms{},lod_update_total_ms{};
    std::uint64_t received_messages{};
    std::uint64_t dropped_input_messages{};
    std::uint64_t rejected_new_voxels{};
    double active_build_seconds{};
    std::uint64_t raycasts{};
    std::uint64_t ray_budget_skipped_points{};
    std::uint64_t ray_traversal_steps{};
    std::uint64_t free_space_checks{};
    std::uint64_t cleared_voxels{};
    std::uint64_t missing_origin_messages{};
};

/// A bounded, camera-specific rendering request. Planes face into the frustum.
struct RoomMapViewRequest {
    std::size_t point_budget{250'000};
    double viewport_height{600};
    std::array<double,3> camera{};
    std::array<std::array<double,4>,6> planes{};
    bool has_camera{false};
};
/// Workers share a disk reader, NOT a full-map copy. Disk queries are serialized
/// with page replacement, but never hold the live accumulator update mutex.
class RoomMapViewSource {
public:
    virtual ~RoomMapViewSource() = default;
    virtual PointCloudFrame select_view(const RoomMapViewRequest&) const = 0;
    virtual std::array<double,6> bounds() const = 0;
};
struct RoomMapViewMessage {
    std::shared_ptr<const RoomMapViewSource> source;
    std::uint64_t revision{}, timestamp_ns{}, point_count{};
    RoomMapState state{RoomMapState::empty};
};

}  // namespace vista::models
