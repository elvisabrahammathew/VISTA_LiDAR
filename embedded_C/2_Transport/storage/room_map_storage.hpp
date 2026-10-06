#pragma once

#include <cstddef>
#include "1_Platform/compat/filesystem.hpp"
#include <functional>
#include <limits>

#include "models/mapping/room_map.hpp"

namespace vista::transport {
using RoomMapPointVisitor = std::function<void(const models::RoomMapPoint&)>;
using RoomMapPointEnumerator = std::function<void(const RoomMapPointVisitor&)>;
/// Streams all points through a fixed-size working set. No entire-map vector.
void write_room_map_snapshot(const vista::fs::path&, std::uint64_t,
                             const RoomMapPointEnumerator&);
std::uint64_t stream_room_map_snapshot(const vista::fs::path&,
    const RoomMapPointVisitor&, std::uint64_t maximum_points = std::numeric_limits<std::uint64_t>::max());
void atomic_replace_map_file(const vista::fs::path& temporary,
                             const vista::fs::path& destination);

/// Claims a new PCD filename exclusively on the first save. A collision adds
/// _1, _2, ...; subsequent saves replace only this session's owned file.
/// No PCD is created before save(); a failed initial claim is cleaned up at destruction.
class RoomMapSessionFile {
public:
    explicit RoomMapSessionFile(vista::fs::path preferred_path);
    ~RoomMapSessionFile();
    RoomMapSessionFile(const RoomMapSessionFile&) = delete;
    RoomMapSessionFile& operator=(const RoomMapSessionFile&) = delete;
    /// Saves atomically using the same claimed filename throughout the session.
    void save(const models::RoomMapFrame& frame);
    void save(std::uint64_t count, const RoomMapPointEnumerator& enumerate);
    const vista::fs::path& path() const { return path_; }
private:
    void claim();
    vista::fs::path preferred_path_, path_;
    bool claimed_{false};
    bool saved_{false};
};

/// Replaces one snapshot atomically, preserving the previous file if writing fails.
void save_room_map_snapshot(const vista::fs::path& path,
                            const models::RoomMapFrame& frame);

/// Loads the ASCII XYZ+intensity PCD format produced by save_room_map_snapshot.
/// Bounds the number of points read so malformed files cannot grow memory indefinitely.
models::RoomMapFrame load_room_map_snapshot(const vista::fs::path& path,
                                               std::size_t maximum_points);

}  // namespace vista::transport
