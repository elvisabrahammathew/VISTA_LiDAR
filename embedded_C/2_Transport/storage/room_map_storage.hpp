#pragma once

#include <cstddef>
#include <filesystem>
#include <functional>
#include <limits>

#include "models/lidars/pointcloud.hpp"

namespace vista::transport {
using RoomMapPointVisitor = std::function<void(const models::PointXYZIRT&)>;
using RoomMapPointEnumerator = std::function<void(const RoomMapPointVisitor&)>;
/// Streams all points through a fixed-size working set. No entire-map vector.
void write_room_map_snapshot(const std::filesystem::path&, std::uint64_t,
                             const RoomMapPointEnumerator&);
std::uint64_t stream_room_map_snapshot(const std::filesystem::path&,
    const RoomMapPointVisitor&, std::uint64_t maximum_points = std::numeric_limits<std::uint64_t>::max());
void atomic_replace_map_file(const std::filesystem::path& temporary,
                             const std::filesystem::path& destination);

/// Claims a new PCD filename exclusively on the first save. A collision adds
/// _1, _2, ...; subsequent saves replace only this session's owned file.
/// No PCD is created before save(); a failed initial claim is cleaned up at destruction.
class RoomMapSessionFile {
public:
    explicit RoomMapSessionFile(std::filesystem::path preferred_path);
    ~RoomMapSessionFile();
    RoomMapSessionFile(const RoomMapSessionFile&) = delete;
    RoomMapSessionFile& operator=(const RoomMapSessionFile&) = delete;
    /// Saves atomically using the same claimed filename throughout the session.
    void save(const models::PointCloudFrame& frame);
    void save(std::uint64_t count, const RoomMapPointEnumerator& enumerate);
    const std::filesystem::path& path() const { return path_; }
private:
    void claim();
    std::filesystem::path preferred_path_, path_;
    bool claimed_{false};
    bool saved_{false};
};

/// Replaces one snapshot atomically, preserving the previous file if writing fails.
void save_room_map_snapshot(const std::filesystem::path& path,
                            const models::PointCloudFrame& frame);

/// Loads the ASCII XYZ+intensity PCD format produced by save_room_map_snapshot.
/// Bounds the number of points read so malformed files cannot grow memory indefinitely.
models::PointCloudFrame load_room_map_snapshot(const std::filesystem::path& path,
                                               std::size_t maximum_points);

}  // namespace vista::transport
