#include "4_Applications/mapping/room_map/room_map.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <unordered_set>

#include "2_Transport/storage/room_map_storage.hpp"
#include "2_Transport/storage/room_map_tiles.hpp"
#include "3_Devices/lidars/lidar.hpp"
#include "models/topics.hpp"

namespace vista::application {
namespace {

using VoxelKey=transport::MapVoxelKey;
using VoxelHash=transport::MapVoxelHash;
using Cell=transport::MapVoxelCell;

/// Guard integer conversion; malformed coordinates must not create unsafe ray walks.
template<typename Point>
std::optional<VoxelKey> voxel_key(const Point& point, float size) {
    const double x = std::floor(static_cast<double>(point.x) / size);
    const double y = std::floor(static_cast<double>(point.y) / size);
    const double z = std::floor(static_cast<double>(point.z) / size);
    constexpr double limit = 1.0e12;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        std::fabs(x) > limit || std::fabs(y) > limit || std::fabs(z) > limit) return std::nullopt;
    return VoxelKey{static_cast<std::int64_t>(x), static_cast<std::int64_t>(y), static_cast<std::int64_t>(z)};
}
std::uint64_t system_timestamp_ns() {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
}

}  // namespace

void validate_room_map_config(const RoomMapConfig& config) {
    if (!std::isfinite(config.voxel_size_m) || config.voxel_size_m < 0.001F)
        throw std::invalid_argument("RoomMapVoxelSizeMeters must be finite and >= 0.001");
    if (!config.cache_max_voxels || config.cache_max_voxels>2'000'000 || !config.cache_max_tiles || config.cache_max_tiles>512)
        throw std::invalid_argument("invalid RoomMapCacheMaxVoxels/RoomMapCacheMaxTiles RAM limits");
    if (!std::isfinite(config.tile_size_m) || config.tile_size_m<=0)
        throw std::invalid_argument("RoomMapTileSizeMeters must be positive and finite");
    const auto edge=std::ceil(config.tile_size_m/config.voxel_size_m);
    if(edge>64 || edge*edge*edge>static_cast<double>(config.cache_max_voxels))
        throw std::invalid_argument("one tile must fit the cache and contain at most 64 voxels per axis");
    if (!config.lod_points_per_node || config.lod_points_per_node>4096 ||
        !config.preview_max_points || config.preview_max_points>2'000'000)
        throw std::invalid_argument("invalid room-map LOD/preview point budgets");
    if (config.minimum_observations == 0 || config.minimum_observations > 100)
        throw std::invalid_argument("RoomMapMinObservations must be between 1 and 100");
    if (config.observation_interval.count() <= 0 || config.publish_interval.count() <= 0 ||
        config.freeze_after.count() < 0)
        throw std::invalid_argument("room-map intervals must be positive; freeze may be zero");
    if (config.free_space_minimum_observations < 2 || config.free_space_minimum_observations > 100)
        throw std::invalid_argument("RoomMapFreeSpaceMinObservations must be between 2 and 100");
    if (config.raycast_maximum_rays_per_window == 0 || config.raycast_maximum_rays_per_window > 10'000)
        throw std::invalid_argument("RoomMapRaycastMaxRaysPerWindow must be between 1 and 10000");
    if (!std::isfinite(config.raycast_maximum_range_m) || config.raycast_maximum_range_m <= 0.0F ||
        config.raycast_maximum_range_m > 200.0F)
        throw std::invalid_argument("RoomMapRaycastMaxRangeMeters must be finite, positive, and <= 200");
    if (!std::isfinite(config.raycast_surface_margin_m) || config.raycast_surface_margin_m < 0.0F ||
        config.raycast_surface_margin_m >= config.raycast_maximum_range_m)
        throw std::invalid_argument("RoomMapRaycastSurfaceMarginMeters must be nonnegative and below ray range");
}

class RoomMapAccumulator::Impl {
public:
    explicit Impl(RoomMapConfig value) : config(std::move(value)), freeze_deadline(config.freeze_after) {
        validate_room_map_config(config);
        make_store();
    }
    void make_store() {
        cells=std::make_unique<transport::RoomMapTileStore>(config.storage_directory,
            config.voxel_size_m,config.tile_size_m,config.cache_max_voxels,
            config.cache_max_tiles,config.minimum_observations,config.lod_points_per_node);
    }
    models::RoomMapFrame select_view(const models::RoomMapViewRequest& request) const {
        std::lock_guard<std::mutex> lock(mutex);
        ++preview_queries;
        auto cloud=cells->select_view(request);cloud.timestamp_ns=timestamp_ns;return cloud;
    }

    /// Spatial order avoids repeatedly evicting/reloading interleaved tile hits.
    /// Equal tiles keep frame order, preserving the first hit per voxel/window.
    static bool tile_less(const VoxelKey& a,const VoxelKey& b) {
        if(a.x!=b.x) return a.x<b.x;
        if(a.y!=b.y) return a.y<b.y;
        return a.z<b.z;
    }

    /// Commit only a completed time window. A later hit in that same window wins
    /// over free-space rays, avoiding deletion by neighboring beams/packet order.
    bool finish_free_window() {
        bool changed = false;
        std::vector<VoxelKey> keys(pending_free.begin(),pending_free.end());
        std::sort(keys.begin(),keys.end(),[&](const auto& a,const auto& b) {
            return tile_less(cells->tile_key(a),cells->tile_key(b));
        });
        for (const auto& key : keys) {
            auto* found = cells->find(key);
            if (!found || found->last_hit_window == current_window) continue;
            auto& cell = *found;
            ++free_space_checks;
            ++cell.free_observations;
            cells->mark_changed(key);
            if (cell.free_observations < config.free_space_minimum_observations) continue;
            if (cell.observations >= config.minimum_observations) {
                --confirmed;
                ++cleared_voxels;
                changed = true;
            }
            cells->erase(key);
        }
        pending_free.clear();
        return changed;
    }

    /// Conservative 3D DDA traversal: visit only voxels BEFORE a measured surface,
    /// never behind it. Empty cells are not allocated, keeping memory bounded.
    void trace_free_ray(const std::array<float, 3>& origin, const models::PointXYZIRT& hit) {
        constexpr std::size_t traversal_budget_per_window = 250'000;
        const std::array<double, 3> start{origin[0], origin[1], origin[2]};
        const std::array<double, 3> direction{
            static_cast<double>(hit.x) - start[0],
            static_cast<double>(hit.y) - start[1],
            static_cast<double>(hit.z) - start[2]};
        const double distance = std::sqrt(direction[0]*direction[0] + direction[1]*direction[1] + direction[2]*direction[2]);
        // Always preserve at least one voxel diagonal next to a hit surface.
        const double margin = std::max<double>(config.raycast_surface_margin_m, config.voxel_size_m * std::sqrt(3.0));
        const double length = std::min<double>(config.raycast_maximum_range_m, distance - margin);
        if (!std::isfinite(distance) || length <= 0.0 || distance <= 0.0) return;
        const auto start_key = voxel_key(models::RoomMapPoint{origin[0], origin[1], origin[2], 0}, config.voxel_size_m);
        const auto end_key = voxel_key(models::RoomMapPoint{
            static_cast<float>(start[0] + direction[0]*length/distance),
            static_cast<float>(start[1] + direction[1]*length/distance),
            static_cast<float>(start[2] + direction[2]*length/distance), 0}, config.voxel_size_m);
        if (!start_key || !end_key) return;
        std::array<std::int64_t, 3> position{start_key->x, start_key->y, start_key->z};
        const std::array<std::int64_t, 3> end{end_key->x, end_key->y, end_key->z};
        std::array<int, 3> step{};
        std::array<double, 3> next{}, delta{};
        for (std::size_t axis = 0; axis < 3; ++axis) {
            const auto d = direction[axis] / distance;
            step[axis] = d > 0.0 ? 1 : d < 0.0 ? -1 : 0;
            if (step[axis] == 0) {
                next[axis] = delta[axis] = std::numeric_limits<double>::infinity();
            } else {
                const double boundary = (static_cast<double>(position[axis]) + (step[axis] > 0 ? 1.0 : 0.0)) * config.voxel_size_m;
                next[axis] = std::max(0.0, (boundary - start[axis]) / d);
                delta[axis] = config.voxel_size_m / std::fabs(d);
            }
        }
        ++raycasts;
        ++rays_this_window;
        while (position != end && steps_this_window < traversal_budget_per_window) {
            ++steps_this_window;
            ++ray_traversal_steps;
            const VoxelKey key{position[0], position[1], position[2]};
            // Do not clear the sensor's own voxel or partially traversed end voxel.
            if (!(key == *start_key) && cells->find(key)) pending_free.insert(key);
            const auto distance_to_boundary = std::min({next[0], next[1], next[2]});
            if (!std::isfinite(distance_to_boundary) || distance_to_boundary > length) break;
            // Step all tied axes. Edge/corner-only contacts are not free-space evidence.
            for (std::size_t axis = 0; axis < 3; ++axis) {
                if (next[axis] <= distance_to_boundary + 1.0e-10) {
                    position[axis] += step[axis];
                    next[axis] += delta[axis];
                }
            }
        }
    }

    bool integrate(const models::PointCloudFrame& cloud, std::chrono::milliseconds elapsed) {
        if (state == models::RoomMapState::frozen || state == models::RoomMapState::loaded) return false;
        elapsed = std::max(elapsed, last_elapsed);
        last_elapsed = elapsed;
        const auto window = elapsed.count() / config.observation_interval.count();
        bool changed = false;
        if (window != current_window) {
            changed = finish_free_window();
            current_window = window;
            rays_this_window = steps_this_window = 0;
        }
        // Expire only tentative one-off returns. Occluded CONFIRMED geometry stays.
        if (elapsed >= next_prune) {
            cells->prune_cached(window,std::max<std::int64_t>(10,10'000/config.observation_interval.count()));
            next_prune = elapsed + std::chrono::seconds(1);
        }
        // Register ALL measured hits before tracing sampled rays. Centroids below
        // are storage only; rays always use actual current measurement endpoints.
        struct Hit { VoxelKey key,tile; const models::PointXYZIRT* point; };
        std::vector<Hit> hits;hits.reserve(cloud.points.size());
        for (const auto& point : cloud.points) {
            const auto key = voxel_key(point, config.voxel_size_m);
            if (!key) continue;
            hits.push_back({*key,cells->tile_key(*key),&point});
        }
        std::stable_sort(hits.begin(),hits.end(),[](const Hit& a,const Hit& b) {
            return tile_less(a.tile,b.tile);
        });
        for (const auto& hit : hits) {
            const auto& point=*hit.point;
            auto* found = cells->find(hit.key);
            if (!found) {
                // Explicit acquisition -> persistent geometry boundary. Ray
                // origins remain on the live input for free-space traversal.
                models::RoomMapPoint geometry{point.x,point.y,point.z,point.intensity};
                found=&cells->insert(hit.key,Cell{geometry,0,-1,0});
            }
            auto& cell = *found;
            if(cell.free_observations!=0) {
                cell.free_observations=0;cells->mark_changed(hit.key);
            }
            if (cell.last_hit_window == window) continue;
            cells->mark_changed(hit.key);
            cell.last_hit_window = window;
            const auto weight = static_cast<float>(std::min<std::size_t>(cell.observations, 29) + 1);
            cell.point.x += (point.x - cell.point.x) / weight;
            cell.point.y += (point.y - cell.point.y) / weight;
            cell.point.z += (point.z - cell.point.z) / weight;
            cell.point.intensity = point.intensity;
            const auto was_confirmed = cell.observations >= config.minimum_observations;
            if (cell.observations < 100) ++cell.observations;
            if (!was_confirmed && cell.observations >= config.minimum_observations) ++confirmed;
            changed = changed || cell.observations >= config.minimum_observations;
        }
        if (!cloud.points.empty()) {
            if (!cloud.sensor_origin_world_m ||
                !std::all_of(cloud.sensor_origin_world_m->begin(), cloud.sensor_origin_world_m->end(),
                    [](float value) { return std::isfinite(value); })) {
                ++missing_origin_messages; // Never assume origin (0,0,0) for a mounted sensor.
            } else {
                const auto rays_before = rays_this_window;
                const auto available = config.raycast_maximum_rays_per_window - rays_this_window;
                const auto sample_count = std::min(available, cloud.points.size());
                for (std::size_t sample = 0; sample < sample_count && steps_this_window < 250'000; ++sample) {
                    const auto index = (sample * cloud.points.size() / sample_count +
                        static_cast<std::size_t>(window) % cloud.points.size()) % cloud.points.size();
                    const auto& point=cloud.points[index];
                    const auto& origin=point.ray_origin_world_m ? *point.ray_origin_world_m : *cloud.sensor_origin_world_m;
                    if(std::all_of(origin.begin(),origin.end(),[](float value){return std::isfinite(value);}))
                        trace_free_ray(origin,point);
                }
                // This counter includes points not selected because of CPU sampling limits.
                ray_budget_skipped_points += cloud.points.size() - (rays_this_window - rays_before);
            }
        }
        state = cells->size()==0 ? models::RoomMapState::empty : models::RoomMapState::building;
        timestamp_ns = system_timestamp_ns(); // Map publication metadata uses the host clock.
        if (confirmed > 0 && config.freeze_after.count() > 0 && elapsed >= freeze_deadline) {
            // Do not commit the current incomplete free window at freeze.
            pending_free.clear();
            state = models::RoomMapState::frozen;
            changed = true;
        }
        return changed;
    }

    RoomMapConfig config;
    mutable std::mutex mutex;
    std::unique_ptr<transport::RoomMapTileStore> cells;
    std::unordered_set<VoxelKey, VoxelHash> pending_free;
    models::RoomMapState state{models::RoomMapState::empty};
    std::size_t confirmed{}, rays_this_window{}, steps_this_window{};
    mutable std::uint64_t preview_queries{};
    std::uint64_t integrated_frames{};
    double last_integration_ms{},max_integration_ms{},last_map_lock_wait_ms{};
    mutable double last_checkpoint_ms{};
    std::int64_t current_window{-1};
    std::uint64_t rejected{}, timestamp_ns{}, raycasts{}, ray_budget_skipped_points{}, ray_traversal_steps{};
    std::uint64_t free_space_checks{}, cleared_voxels{}, missing_origin_messages{};
    std::chrono::milliseconds last_elapsed{}, next_prune{}, freeze_deadline{};
};

RoomMapAccumulator::RoomMapAccumulator(RoomMapConfig config) : impl_(std::make_shared<Impl>(std::move(config))) {}
RoomMapAccumulator::~RoomMapAccumulator() = default;
bool RoomMapAccumulator::integrate(const models::PointCloudFrame& cloud, std::chrono::milliseconds elapsed) {
    const auto before_lock=std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(impl_->mutex);
    const auto started=std::chrono::steady_clock::now();
    impl_->last_map_lock_wait_ms=std::chrono::duration<double,std::milli>(started-before_lock).count();
    const auto changed=impl_->integrate(cloud, elapsed);
    impl_->last_integration_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
    impl_->max_integration_ms=std::max(impl_->max_integration_ms,impl_->last_integration_ms);
    ++impl_->integrated_frames;
    return changed;
}
models::RoomMapFrame RoomMapAccumulator::snapshot() const {
    models::RoomMapViewRequest request;
    request.point_budget=impl_->config.preview_max_points;
    return impl_->select_view(request); // A bounded preview, NOT the complete PCD.
}
models::RoomMapStatus RoomMapAccumulator::status() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    models::RoomMapStatus value;
    value.state = impl_->state;
    value.point_count = impl_->confirmed;
    value.candidate_voxels = impl_->cells->size() - impl_->confirmed;
    value.cache_max_voxels = impl_->config.cache_max_voxels;
    value.cache_voxels = impl_->cells->resident_voxels();
    value.cache_tiles = impl_->cells->resident_tiles();
    value.cache_evictions = impl_->cells->evictions();
    value.disk_tiles = impl_->cells->disk_tiles();
    const auto metrics=impl_->cells->metrics();
    value.cache_misses=metrics.cache_misses;value.tile_reads=metrics.tile_reads;
    value.tile_writes=metrics.tile_writes;value.lod_node_writes=metrics.lod_node_writes;
    value.view_queries=metrics.view_queries;value.last_view_ms=metrics.last_view_ms;
    value.last_view_lock_wait_ms=metrics.last_view_lock_wait_ms;
    value.tile_read_total_ms=metrics.tile_read_total_ms;value.tile_write_total_ms=metrics.tile_write_total_ms;
    value.lod_update_total_ms=metrics.lod_update_total_ms;
    value.integrated_frames=impl_->integrated_frames;value.preview_queries=impl_->preview_queries;
    value.last_integration_ms=impl_->last_integration_ms;value.max_integration_ms=impl_->max_integration_ms;
    value.last_map_lock_wait_ms=impl_->last_map_lock_wait_ms;value.last_checkpoint_ms=impl_->last_checkpoint_ms;
    value.rejected_new_voxels = impl_->rejected;
    value.active_build_seconds = static_cast<double>(impl_->last_elapsed.count()) / 1000.0;
    value.raycasts = impl_->raycasts;
    value.ray_budget_skipped_points = impl_->ray_budget_skipped_points;
    value.ray_traversal_steps = impl_->ray_traversal_steps;
    value.free_space_checks = impl_->free_space_checks;
    value.cleared_voxels = impl_->cleared_voxels;
    value.missing_origin_messages = impl_->missing_origin_messages;
    return value;
}
void RoomMapAccumulator::freeze() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->state != models::RoomMapState::loaded) {
        impl_->pending_free.clear();
        impl_->state = models::RoomMapState::frozen;
    }
}
void RoomMapAccumulator::resume() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->state == models::RoomMapState::loaded) return; // Loaded references are immutable.
    impl_->state = impl_->cells->size()==0 ? models::RoomMapState::empty : models::RoomMapState::building;
    impl_->freeze_deadline = impl_->last_elapsed + impl_->config.freeze_after;
}
void RoomMapAccumulator::reset() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->make_store(); impl_->pending_free.clear();
    impl_->confirmed = impl_->rays_this_window = impl_->steps_this_window = 0;
    impl_->rejected = impl_->timestamp_ns = impl_->raycasts = impl_->ray_budget_skipped_points = 0;
    impl_->ray_traversal_steps = impl_->free_space_checks = impl_->cleared_voxels = impl_->missing_origin_messages = 0;
    impl_->current_window = -1;
    impl_->state = models::RoomMapState::empty;
    impl_->last_elapsed = impl_->next_prune = std::chrono::milliseconds(0);
    impl_->freeze_deadline = impl_->config.freeze_after;
    impl_->preview_queries=impl_->integrated_frames=0;
    impl_->last_integration_ms=impl_->max_integration_ms=impl_->last_map_lock_wait_ms=impl_->last_checkpoint_ms=0;
}
void RoomMapAccumulator::restore(const models::RoomMapFrame& cloud) {
    if (cloud.points.empty()) throw std::invalid_argument("saved room map is empty");
    for (const auto& point : cloud.points)
        if (!voxel_key(point, impl_->config.voxel_size_m))
            throw std::invalid_argument("saved room map has invalid coordinates");
    reset();
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->cells->set_reference();
    for(const auto& point:cloud.points) impl_->cells->append_reference(point);
    impl_->confirmed=cloud.points.size();
    impl_->cells->flush();
    impl_->timestamp_ns = cloud.timestamp_ns;
    impl_->state = models::RoomMapState::loaded;
}
void RoomMapAccumulator::load(const vista::fs::path& file) {
    reset();
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->cells->set_reference();
    // Stream a large PCD into disk tiles instead of loading the whole file in RAM.
    transport::stream_room_map_snapshot(file,[&](const models::RoomMapPoint& point) {
        if(!voxel_key(point,impl_->config.voxel_size_m)) throw std::invalid_argument("invalid saved map coordinate");
        impl_->cells->append_reference(point);++impl_->confirmed;
    });
    if(!impl_->confirmed) throw std::invalid_argument("saved room map is empty");
    impl_->cells->flush();impl_->state=models::RoomMapState::loaded;
}
void RoomMapAccumulator::save(transport::RoomMapSessionFile& output) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    output.save(impl_->confirmed,[&](const transport::RoomMapPointVisitor& visitor) {
        impl_->cells->for_each_confirmed(visitor); // Every confirmed disk voxel, not just the preview.
    });
}
void RoomMapAccumulator::flush() {
    std::lock_guard<std::mutex> lock(impl_->mutex);impl_->cells->flush();
}
std::shared_ptr<const models::RoomMapViewSource> RoomMapAccumulator::view_source() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    const auto started=std::chrono::steady_clock::now();
    auto source=impl_->cells->view_source(impl_->timestamp_ns);
    impl_->last_checkpoint_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
    return source;
}

platform::WorkerHandle spawn_room_map_worker(platform::MessageBus& bus, platform::ThreadConfig thread_config,
    platform::StopToken stop, RoomMapConfig config, RoomMapCompletion on_complete) {
    validate_room_map_config(config);
    // Only building mode subscribes to live geometry. Loading/disabled modes
    // merely serve reference snapshots and status; they never reconstruct.
    std::optional<platform::TopicSubscriber<devices::LidarPointCloudMessage>> subscriber;
    if (config.enabled && !config.load_existing)
        subscriber.emplace(bus.subscribe<devices::LidarPointCloudMessage>(models::topics::pointcloud_cleaned, thread_config.name));
    auto publisher = bus.publisher<models::RoomMapMessage>(models::topics::room_map);
    auto status_publisher = bus.publisher<models::RoomMapStatus>(models::topics::room_map_status);
    auto view_publisher = bus.publisher<models::RoomMapViewMessage>(models::topics::room_map_view);
    return platform::spawn_worker(std::move(thread_config), [stop, config = std::move(config),
        subscriber = std::move(subscriber), publisher = std::move(publisher),
        status_publisher = std::move(status_publisher), view_publisher=std::move(view_publisher),
        on_complete = std::move(on_complete)]() mutable {
        try {
            RoomMapAccumulator map(config);
            RoomMapReport report;
            std::optional<models::RoomMapState> status_override;
            if (!config.enabled) status_override = models::RoomMapState::disabled;
            if (config.enabled && config.load_existing) {
                try {
                    map.load(config.file);
                    std::cout << "Room map loaded as read-only reference: " << config.file.string() << '\n';
                } catch (const std::exception& error) {
                    status_override = models::RoomMapState::load_error;
                    map.reset(); // A partially imported/corrupt PCD must not become a visible reference.
                    std::cerr << "Room map load failed: " << error.what()
                              << "; live capture continues; no rebuild or file overwrite\n";
                }
            }
            bool dirty = true;
            std::optional<transport::RoomMapSessionFile> output_file;
            if (config.enabled && !config.load_existing && !config.file.empty())
                output_file.emplace(config.file); // Lazy claim: no PCD without map data.
            bool ever_confirmed = false;
            std::uint64_t revision{};
            std::chrono::steady_clock::duration active_time{};
            std::optional<std::chrono::steady_clock::time_point> last_observation;
            std::shared_ptr<const models::RoomMapMessage> snapshot;
            std::shared_ptr<const models::RoomMapViewSource> view_source;
            std::optional<std::uint64_t> saved_revision;
            auto next_publish = std::chrono::steady_clock::now();
            // RoomMapPublishIntervalMilliseconds is the single checkpoint/view
            // publication cadence. Finish the disk checkpoint BEFORE announcing
            // its revision; no full-PCD rewrite is performed on this timer.
            // Freeze/shutdown can make a final checkpoint outside that cadence.
            const auto refresh_view = [&]() {
                if (!dirty && revision!=0) return;
                if (!status_override) view_source=map.view_source();
                snapshot.reset();
                ++revision;
                dirty=false;
            };
            const auto refresh_snapshot = [&]() {
                if (snapshot) return;
                const auto now_ns = system_timestamp_ns();
                auto cloud = map.snapshot();
                cloud.timestamp_ns = now_ns;
                snapshot = std::make_shared<const models::RoomMapMessage>(
                    revision, now_ns, std::move(cloud));
            };
            const auto save = [&]() {
                // Building sessions always save on freeze/clean shutdown. A loaded
                // reference is NEVER rewritten; there is no save enable/disable flag.
                if (!output_file) return;
                refresh_view();
                // Do not claim/create a PCD at no-data startup. But once a new
                // map existed, save an empty update if all its geometry was cleared.
                if ((!ever_confirmed && map.status().point_count==0) || saved_revision==revision) return;
                try {
                    map.save(*output_file);
                    saved_revision = revision;
                    std::cout << "Room map saved: " << output_file->path().string() << " ("
                              << map.status().point_count << " points)\n";
                } catch (const std::exception& error) {
                    status_override=models::RoomMapState::storage_error;
                    std::cerr << "Room map save warning: " << error.what() << '\n';
                }
            };
            bool closed = false;
            while (!stop.is_stop_requested() && !closed) {
                if (subscriber) {
                    std::shared_ptr<const devices::LidarPointCloudMessage> message;
                    auto received = subscriber->receive_for(message, std::chrono::milliseconds(50));
                    const auto batch_deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(20);
                    for (std::size_t batch = 0; batch < 64 && received == platform::ReceiveStatus::message; ++batch) {
                        ++report.received_messages;
                        if (!message->payload.points.empty() && map.status().state != models::RoomMapState::frozen) {
                            const auto now = std::chrono::steady_clock::now();
                            if (last_observation) active_time += std::min(now - *last_observation,
                                std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::seconds(1)));
                            last_observation = now;
                            dirty = map.integrate(message->payload,
                                std::chrono::duration_cast<std::chrono::milliseconds>(active_time)) || dirty;
                            ever_confirmed = ever_confirmed || map.status().point_count > 0;
                            if (map.status().state == models::RoomMapState::frozen) {
                                std::cout << "Room map frozen after " << map.status().active_build_seconds << " active seconds\n";
                                save();
                            }
                        }
                        // Never dequeue a frame and then abandon it. Yield only
                        // after completing this frame; one frame may exceed 20 ms.
                        if(stop.is_stop_requested() || std::chrono::steady_clock::now()>=batch_deadline) break;
                        if (batch + 1 < 64) received = subscriber->try_receive(message);
                    }
                    closed = received == platform::ReceiveStatus::closed;
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
                const auto now = std::chrono::steady_clock::now();
                if (now >= next_publish) {
                    refresh_view();
                    if(publisher.subscriber_count()!=0) {
                        refresh_snapshot();
                        publisher.publish_shared(snapshot); // Late/legacy consumers still obtain a preview.
                        ++report.published_snapshots;
                    }
                    auto status = map.status();
                    if (status_override) status.state = *status_override;
                    status.timestamp_ns = system_timestamp_ns();
                    status.received_messages = report.received_messages;
                    status.dropped_input_messages = subscriber ? subscriber->dropped_messages() : 0;
                    models::RoomMapViewMessage view;
                    if(!status_override) view.source=view_source;
                    view.revision=revision;view.timestamp_ns=status.timestamp_ns;
                    view.point_count=status.point_count;view.state=status.state;
                    // Unchanged revision heartbeats let a late-started worker
                    // discover the current map. The WebSocket does not resend
                    // unchanged geometry unless its client changes camera/reconnects.
                    view_publisher.publish(std::move(view));
                    status_publisher.publish(std::move(status));
                    next_publish = now + config.publish_interval;
                }
            }
            save();
            report.map_points = map.status().point_count;
            report.dropped_input_messages = subscriber ? subscriber->dropped_messages() : 0;
            on_complete(report, {});
        } catch (const std::exception& error) {
            models::RoomMapStatus status;status.state=models::RoomMapState::storage_error;
            status.timestamp_ns=system_timestamp_ns();status_publisher.publish(status);
            models::RoomMapViewMessage view;view.state=status.state;view.timestamp_ns=status.timestamp_ns;
            view_publisher.publish(std::move(view));
            on_complete(std::nullopt, error.what()); // Optional mapping must not stop capture.
        } catch (...) { on_complete(std::nullopt, "unknown room-map failure"); }
    });
}

}  // namespace vista::application
