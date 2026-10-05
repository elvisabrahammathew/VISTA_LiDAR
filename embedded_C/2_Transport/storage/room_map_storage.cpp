#include "2_Transport/storage/room_map_storage.hpp"

#include <cmath>
#include <cerrno>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <fcntl.h>
#include <unistd.h>
#endif

namespace vista::transport {

RoomMapSessionFile::RoomMapSessionFile(std::filesystem::path preferred_path)
    : preferred_path_(std::move(preferred_path)), path_(preferred_path_) {
    if (preferred_path_.empty()) throw std::invalid_argument("room-map session path is empty");
}
RoomMapSessionFile::~RoomMapSessionFile() {
    if (claimed_ && !saved_) {
        // This empty claim belongs to this session, never to an older map.
        std::error_code ignored;
        std::filesystem::remove(path_, ignored);
    }
}
void RoomMapSessionFile::claim() {
    if (!preferred_path_.parent_path().empty())
        std::filesystem::create_directories(preferred_path_.parent_path());
    // Exclusive create closes the exists()/write race between simultaneous
    // sessions. A crash may leave a claim, which a later session safely skips.
    for (std::size_t suffix = 0; suffix < 10'000; ++suffix) {
        path_ = suffix == 0 ? preferred_path_ : preferred_path_.parent_path() /
            (preferred_path_.stem().native() + std::filesystem::path("_" + std::to_string(suffix)).native() +
             preferred_path_.extension().native());
        auto temporary = path_; temporary += ".tmp";
        if (std::filesystem::exists(temporary)) continue; // Preserve stale/foreign temporary output too.
#ifdef _WIN32
        const auto handle = CreateFileW(path_.c_str(), GENERIC_WRITE, 0, nullptr,
                                       CREATE_NEW, FILE_ATTRIBUTE_NORMAL, nullptr);
        if (handle == INVALID_HANDLE_VALUE) {
            const auto error = GetLastError();
            if (error == ERROR_FILE_EXISTS || error == ERROR_ALREADY_EXISTS) continue;
            throw std::runtime_error("cannot claim room-map filename; Windows error " + std::to_string(error));
        }
        claimed_ = true;
        if (!CloseHandle(handle)) throw std::runtime_error("cannot close room-map filename claim");
#else
        const auto descriptor = ::open(path_.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0666);
        if (descriptor == -1) {
            const auto error = errno;
            if (error == EEXIST) continue;
            throw std::runtime_error("cannot claim room-map filename; errno " + std::to_string(error));
        }
        claimed_ = true;
        if (::close(descriptor) != 0) throw std::runtime_error("cannot close room-map filename claim");
#endif
        return;
    }
    throw std::runtime_error("too many room-map filename collisions in this session second");
}
void RoomMapSessionFile::save(const models::PointCloudFrame& frame) {
    save(frame.points.size(), [&](const RoomMapPointVisitor& visit) {
        for (const auto& point : frame.points) visit(point);
    });
}
void RoomMapSessionFile::save(std::uint64_t count, const RoomMapPointEnumerator& enumerate) {
    if (!claimed_) claim();
    write_room_map_snapshot(path_, count, enumerate);
    saved_ = true;
}

void save_room_map_snapshot(const std::filesystem::path& path,
                            const models::PointCloudFrame& frame) {
    write_room_map_snapshot(path, frame.points.size(), [&](const RoomMapPointVisitor& visit) {
        for (const auto& point : frame.points) visit(point);
    });
}
void write_room_map_snapshot(const std::filesystem::path& path, std::uint64_t count,
                             const RoomMapPointEnumerator& enumerate) {
    if (path.empty()) throw std::invalid_argument("room-map file path is empty");
    if (!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path());
    auto temporary = path;
    temporary += ".tmp";
    {
        std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
        if (!output) throw std::runtime_error("cannot write room map: " + temporary.string());
        output << "# VISTA room map in world coordinates (meters)\n"
               << "VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 1\n"
               << "TYPE F F F U\nCOUNT 1 1 1 1\nWIDTH " << count
               << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << count
               << "\nDATA ascii\n" << std::setprecision(std::numeric_limits<float>::max_digits10);
        std::uint64_t written{};
        enumerate([&](const models::PointXYZIRT& point) {
            if (++written > count) throw std::runtime_error("room-map export count mismatch");
            output << point.x << ' ' << point.y << ' ' << point.z << ' '
                   << static_cast<unsigned int>(point.intensity) << '\n';
        });
        if (written != count) throw std::runtime_error("room-map export count mismatch");
        output.flush();
        if (!output) throw std::runtime_error("cannot flush room-map snapshot");
        output.close();
        if (!output) throw std::runtime_error("cannot close room-map snapshot");
    }
    atomic_replace_map_file(temporary, path);
}
void atomic_replace_map_file(const std::filesystem::path& temporary, const std::filesystem::path& path) {
#ifdef _WIN32
    if (!MoveFileExW(temporary.c_str(), path.c_str(),
                     MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) {
        throw std::runtime_error("cannot replace room-map snapshot; Windows error " +
                                 std::to_string(GetLastError()));
    }
#else
    std::filesystem::rename(temporary, path);
#endif
}

models::PointCloudFrame load_room_map_snapshot(const std::filesystem::path& path,
                                               std::size_t maximum_points) {
    models::PointCloudFrame frame;
    stream_room_map_snapshot(path, [&](const models::PointXYZIRT& point) { frame.points.push_back(point); }, maximum_points);
    return frame;
}
std::uint64_t stream_room_map_snapshot(const std::filesystem::path& path,
    const RoomMapPointVisitor& visit, std::uint64_t maximum_points) {
    std::ifstream input(path, std::ios::binary);
    if (!input) throw std::runtime_error("cannot read room map: " + path.string());
    std::string line;
    std::uint64_t declared_points{}, read_points{};
    bool fields_seen = false, points_seen = false, ascii_seen = false;
    while (std::getline(input, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        std::istringstream header(line);
        std::string key;
        header >> key;
        if (key == "FIELDS") {
            std::string fields;
            std::getline(header, fields);
            fields_seen = fields == " x y z intensity";
        } else if (key == "POINTS") {
            if (!(header >> declared_points) || declared_points > maximum_points)
                throw std::runtime_error("room-map snapshot exceeds configured voxel limit");
            points_seen = true;
        } else if (key == "DATA") {
            std::string format;
            header >> format;
            ascii_seen = format == "ascii";
            break;
        }
    }
    if (!fields_seen || !points_seen || !ascii_seen)
        throw std::runtime_error("unsupported room-map PCD; use a VISTA room-map snapshot");
    while (read_points < declared_points && std::getline(input, line)) {
        std::istringstream row(line);
        models::PointXYZIRT point;
        unsigned int intensity{};
        if (!(row >> point.x >> point.y >> point.z >> intensity) || intensity > 255 ||
            !std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
            throw std::runtime_error("invalid point in room-map snapshot");
        point.intensity = static_cast<std::uint8_t>(intensity);
        visit(point);
        ++read_points;
    }
    if (read_points != declared_points)
        throw std::runtime_error("room-map snapshot is truncated");
    return read_points;
}

}  // namespace vista::transport
