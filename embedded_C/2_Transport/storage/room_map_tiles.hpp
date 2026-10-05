#pragma once
#include <cstdint>
#include "1_Platform/compat/filesystem.hpp"
#include <functional>
#include <memory>
#include <unordered_map>
#include "models/room_map.hpp"
#include "2_Transport/storage/room_map_storage.hpp"

namespace vista::transport {
struct MapVoxelKey {
    std::int64_t x{},y{},z{};
    bool operator==(const MapVoxelKey& b) const { return x==b.x && y==b.y && z==b.z; }
};
struct MapVoxelHash { std::size_t operator()(const MapVoxelKey&) const noexcept; };
struct MapVoxelCell {
    models::PointXYZIRT point;
    std::size_t observations{};
    std::int64_t last_hit_window{-1};
    std::size_t free_observations{};
};
struct RoomMapTileMetrics {
    std::uint64_t cache_misses{}, tile_reads{}, tile_writes{}, lod_node_writes{};
    std::uint64_t view_queries{};
    double last_view_ms{}, last_view_lock_wait_ms{};
    double tile_read_total_ms{},tile_write_total_ms{},lod_update_total_ms{};
};
/// LRU RAM cache + persistent octree nodes. Eviction writes voxel evidence before
/// freeing memory; it NEVER discards confirmed geometry to make room.
class RoomMapTileStore {
public:
    RoomMapTileStore(vista::fs::path root, float voxel_size, float tile_size,
        std::size_t cache_voxels, std::size_t cache_tiles,
        std::size_t confirmation_threshold, std::size_t lod_points);
    ~RoomMapTileStore();
    RoomMapTileStore(const RoomMapTileStore&)=delete;
    /// Lookup alone does not dirty a tile. Mark it only after changing evidence.
    MapVoxelCell* find(const MapVoxelKey&);
    void mark_changed(const MapVoxelKey&);
    MapVoxelKey tile_key(const MapVoxelKey&) const;
    MapVoxelCell& insert(const MapVoxelKey&, const MapVoxelCell&);
    void erase(const MapVoxelKey&);
    std::size_t size() const;
    std::size_t prune_cached(std::int64_t window, std::int64_t age);
    void flush();
    /// Imported references use streaming point pages, preserving duplicate points.
    void set_reference();
    void append_reference(const models::PointXYZIRT&);
    void for_each_confirmed(const RoomMapPointVisitor&);
    models::PointCloudFrame select_view(const models::RoomMapViewRequest&);
    /// Flush at a mapper checkpoint, then expose disk-only queries. Queries never
    /// flush RAM or take the accumulator's update mutex.
    std::shared_ptr<const models::RoomMapViewSource> view_source(std::uint64_t timestamp_ns);
    RoomMapTileMetrics metrics() const;
    std::array<double,6> bounds() const;
    std::size_t resident_voxels() const;
    std::size_t resident_tiles() const;
    std::uint64_t evictions() const;
    std::uint64_t disk_tiles() const;
    const vista::fs::path& directory() const;
private:
    class Impl;
    std::shared_ptr<Impl> impl_;
};
} // namespace vista::transport
