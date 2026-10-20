#pragma once
#include "1_Platform/compat/filesystem.hpp"
#include "1_Platform/message_bus/message_bus.hpp"
#include "1_Platform/threading/threading.hpp"
#include "models/mapping/gaussian_map.hpp"
#include <functional>
#include <optional>
namespace vista::application {
struct GaussianMapConfig { bool enabled{true}; vista::fs::path storage_directory; };
struct GaussianMapReport {
    std::uint64_t received_maps{},completed_revisions{},rebuilt_tiles{},failed_revisions{};
};
/// Full confirmed regions, changed-tile/halo rebuilds, immutable disk output.
class GaussianMapBuilder {
public:
    explicit GaussianMapBuilder(vista::fs::path storage_directory = {});
    ~GaussianMapBuilder();
    std::shared_ptr<const models::GaussianMapViewSource> update(
        const models::RoomMapSurfaceSnapshot&,const std::function<bool()>& cancelled = {});
    std::uint64_t rebuilt_tiles() const;
private:
    class Impl;std::unique_ptr<Impl> impl_;
};
std::vector<std::uint8_t> encode_gaussian_map(const std::vector<models::GaussianMapPoint>&,
    std::uint64_t revision,std::uint64_t timestamp_ns);
using GaussianMapCompletion=std::function<void(std::optional<GaussianMapReport>,std::string)>;
platform::WorkerHandle spawn_gaussian_map_worker(platform::MessageBus&,platform::ThreadConfig,
    platform::StopToken,GaussianMapConfig,GaussianMapCompletion);
}
