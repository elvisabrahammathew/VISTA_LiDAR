#include "4_Applications/perception/foreground/foreground.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <unordered_map>
#include "models/mapping/localization.hpp"
#include "models/topics.hpp"
#include "3_Devices/lidars/lidar.hpp"

namespace vista::application {
namespace {
using Key=std::array<std::int64_t,3>;
struct Hash {
    std::size_t operator()(const Key& k) const {
        auto h=std::hash<std::int64_t>{}(k[0]);
        for(std::size_t i=1;i<3;++i) h^=std::hash<std::int64_t>{}(k[i])+0x9e3779b9U+(h<<6U)+(h>>2U);
        return h;
    }
};
std::optional<Key> key(double x,double y,double z,double size) {
    const std::array<double,3> v{std::floor(x/size),std::floor(y/size),std::floor(z/size)};
    for(const auto a:v) if(!std::isfinite(a) || std::abs(a)>1e12) return std::nullopt;
    return Key{static_cast<std::int64_t>(v[0]),static_cast<std::int64_t>(v[1]),static_cast<std::int64_t>(v[2])};
}
std::uint64_t steady_ns() {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}
constexpr std::uint64_t stale_ns=500'000'000ULL;
constexpr std::uint64_t pose_heartbeat_stale_ns=2'500'000'000ULL;
std::int64_t floor_div(std::int64_t a,std::int64_t b) {const auto q=a/b;return a%b<0?q-1:q;}
std::optional<Key> region_key(double x,double y,double z,double voxel,std::int64_t edge) {
    const auto k=key(x,y,z,voxel);if(!k) return std::nullopt;
    return Key{floor_div((*k)[0],edge),floor_div((*k)[1],edge),floor_div((*k)[2],edge)};
}
}

void validate_foreground_config(const ForegroundConfig& c) {
    if(!std::isfinite(c.background_distance_m) || c.background_distance_m<0.005F || c.background_distance_m>1.0F ||
       !c.maximum_input_points || c.maximum_input_points>50'000 || c.publish_interval.count()<50 || c.publish_interval.count()>500 ||
       !c.maximum_cached_points || c.maximum_cached_points>2'000'000 || !c.maximum_cached_regions || c.maximum_cached_regions>512 ||
       !c.maximum_region_reads_per_frame || c.maximum_region_reads_per_frame>64 ||
       !std::isfinite(c.outlier_radius_m) || c.outlier_radius_m<0.005F || c.outlier_radius_m>1.0F ||
       !std::isfinite(c.outlier_start_distance_m) || c.outlier_start_distance_m<0.0F ||
       !std::isfinite(c.outlier_end_distance_m) || c.outlier_end_distance_m<=c.outlier_start_distance_m || c.outlier_end_distance_m>1000.0F ||
       !std::isfinite(c.outlier_max_radius_m) || c.outlier_max_radius_m<0.005F || c.outlier_max_radius_m>1.0F ||
       !c.outlier_minimum_neighbors || c.outlier_minimum_neighbors>16)
        throw std::invalid_argument("invalid Foreground distance, cadence, cache or scan outlier settings");
}

struct ForegroundScanFilter::Impl {
    explicit Impl(ForegroundConfig c):config(std::move(c)) {validate_foreground_config(config);}
    ForegroundConfig config;
    std::unordered_map<Key,std::vector<std::size_t>,Hash> grid;
    std::vector<unsigned char> keep;
};
ForegroundScanFilter::ForegroundScanFilter(ForegroundConfig c):impl_(std::make_unique<Impl>(std::move(c))) {}
ForegroundScanFilter::~ForegroundScanFilter()=default;
ForegroundScanFilterReport ForegroundScanFilter::filter(std::vector<models::ForegroundPoint>& points,
    std::optional<std::array<float,3>> origin) {
    auto& s=*impl_;const auto& c=s.config;
    if(points.size()>c.maximum_input_points) throw std::invalid_argument("foreground scan sample exceeds point budget");
    ForegroundScanFilterReport report;report.input_points=points.size();
    if(!c.outlier_filter_enabled) {report.retained_points=points.size();return report;}
    // Rebuild from THIS sample only. Keep masks prevent order-dependent/in-place
    // pruning; neighbors later removed as isolated still belong to this scan.
    s.grid.clear();s.keep.assign(points.size(),0);s.grid.reserve(points.size());
    const bool have_origin=origin && std::all_of(origin->begin(),origin->end(),[](float v){return std::isfinite(v);});
    // Older configs may specify a base larger than the default max. Never shrink
    // their base radius: a max <= base is also the fixed-radius A/B mode.
    const double base=c.outlier_radius_m,maximum=std::max(base,double(c.outlier_max_radius_m));
    // One grid; at most +/-2 cells per axis (125 lookups). With the default
    // max=2*base, nearby points keep exactly the original base-grid query.
    const double cell_size=have_origin?std::max(base,maximum/2):base;
    for(std::size_t i=0;i<points.size();++i) {
        const auto& p=points[i];const auto cell=key(p.x,p.y,p.z,cell_size);
        if(cell) s.grid[*cell].push_back(i);
    }
    for(std::size_t i=0;i<points.size();++i) {
        const auto& p=points[i];const auto cell=key(p.x,p.y,p.z,cell_size);
        if(!cell) {++report.unresolved_points;continue;}
        double radius=base;
        if(have_origin) {
            const double x=double(p.x)-(*origin)[0],y=double(p.y)-(*origin)[1],z=double(p.z)-(*origin)[2];
            const double fraction=std::clamp((std::sqrt(x*x+y*y+z*z)-c.outlier_start_distance_m)/
                (double(c.outlier_end_distance_m)-c.outlier_start_distance_m),0.0,1.0);
            radius=base+(maximum-base)*fraction;
            if(radius>base) ++report.adaptive_points;
        } else ++report.missing_origin_points;
        const double radius2=radius*radius;
        std::array<models::ForegroundPoint,16> neighbors{};
        std::size_t count=0,checks=0;bool exhausted=false;
        const auto low=key(double(p.x)-radius,double(p.y)-radius,double(p.z)-radius,cell_size);
        const auto high=key(double(p.x)+radius,double(p.y)+radius,double(p.z)+radius,cell_size);
        if(!low || !high) {++report.unresolved_points;continue;}
        for(auto x=(*low)[0];x<=(*high)[0] && count<c.outlier_minimum_neighbors && !exhausted;++x)
        for(auto y=(*low)[1];y<=(*high)[1] && count<c.outlier_minimum_neighbors && !exhausted;++y)
        for(auto z=(*low)[2];z<=(*high)[2] && count<c.outlier_minimum_neighbors && !exhausted;++z) {
            const auto bucket=s.grid.find({x,y,z});
            if(bucket==s.grid.end()) continue;
            for(const auto j:bucket->second) {
                if(j==i) continue;
                // Pathological duplicate/dense buckets cannot cause quadratic work.
                if(checks>=512 || report.distance_checks>=1'000'000) {exhausted=true;break;}
                ++checks;++report.distance_checks;
                const auto& q=points[j];
                if(p.x==q.x && p.y==q.y && p.z==q.z) continue;
                const double a=static_cast<double>(p.x)-q.x,b=static_cast<double>(p.y)-q.y,d=static_cast<double>(p.z)-q.z;
                if(a*a+b*b+d*d>radius2) continue;
                bool duplicate=false;
                for(std::size_t n=0;n<count;++n)
                    if(neighbors[n].x==q.x && neighbors[n].y==q.y && neighbors[n].z==q.z) {duplicate=true;break;}
                if(duplicate) continue;
                neighbors[count++]=q;
                if(count>=c.outlier_minimum_neighbors) break;
            }
        }
        if(count>=c.outlier_minimum_neighbors) {s.keep[i]=1;++report.retained_points;}
        else if(exhausted) ++report.unresolved_points; // Not falsely declared proven noise.
        else ++report.isolated_points;
    }
    std::size_t write=0;
    for(std::size_t i=0;i<points.size();++i) if(s.keep[i]) points[write++]=points[i];
    points.resize(write);
    return report;
}

struct ForegroundEngine::Impl {
    struct Entry {
        std::shared_ptr<const models::RoomMapRegion> source;
        std::unordered_map<Key,std::vector<models::RoomMapPoint>,Hash> grid;
        std::size_t points{};
        std::uint64_t touch{};
    };
    explicit Impl(ForegroundConfig c):config(std::move(c)) {validate_foreground_config(config);}
    ForegroundConfig config;
    std::map<Key,Entry> cache;
    std::size_t cached_points{};
    std::uint64_t clock{},last{},epoch{};
    float tile_size{};
    float voxel_size{};
    std::int64_t tile_edge{};
    std::string source;
    void clear() {cache.clear();cached_points=0;last=0;}
    void erase(std::map<Key,Entry>::iterator it) {cached_points-=it->second.points;cache.erase(it);}
    bool prepare(const models::ForegroundObservation& in,bool prefetch=false) {
        if(!in.monotonic_ns || !in.background || !in.background->error.empty()){clear();return false;}
        const auto& background=*in.background;
        if(!std::isfinite(background.tile_size)||background.tile_size<=0 || !std::isfinite(background.voxel_size)||background.voxel_size<=0)
            throw std::runtime_error("invalid foreground background layout");
        if(last && (in.source!=source || in.epoch!=epoch || in.monotonic_ns<last || (!prefetch && in.monotonic_ns==last) ||
           in.monotonic_ns-last>1'000'000'000ULL || tile_size!=background.tile_size || voxel_size!=background.voxel_size))clear();
        const auto edge=std::round(static_cast<double>(background.tile_size)/background.voxel_size);
        if(edge<1||edge>64)throw std::runtime_error("invalid foreground tile edge");
        source=in.source;epoch=in.epoch;tile_size=background.tile_size;voxel_size=background.voxel_size;tile_edge=static_cast<std::int64_t>(edge);
        if(!prefetch)last=in.monotonic_ns;
        // Deleted/replaced immutable pages can never be used as current background.
        for(auto it=cache.begin();it!=cache.end();){const auto region=background.regions.find(it->first);
            if(region==background.regions.end()||region->second!=it->second.source){auto old=it++;erase(old);}else ++it;}
        return true;
    }
    const Entry* acquire(const Key& k,const std::shared_ptr<const models::RoomMapRegion>& region,
                         std::size_t& reads,std::size_t& read_points,std::size_t read_limit) {
        auto found=cache.find(k);
        if(found!=cache.end() && found->second.source==region) {
            found->second.touch=++clock;return &found->second;
        }
        if(found!=cache.end()) erase(found);
        if(!region || region->point_count>config.maximum_cached_points ||
           region->point_count>config.maximum_cached_points-read_points || reads>=read_limit) return nullptr;
        ++reads;read_points+=static_cast<std::size_t>(region->point_count);
        while(cache.size()>=config.maximum_cached_regions || cached_points+region->point_count>config.maximum_cached_points) {
            auto oldest=std::min_element(cache.begin(),cache.end(),[](const auto& a,const auto& b){return a.second.touch<b.second.touch;});
            if(oldest==cache.end()) return nullptr;
            erase(oldest);
        }
        Entry entry;entry.source=region;entry.touch=++clock;
        region->read([&](const models::RoomMapPoint& p) {
            if(entry.points>=region->point_count) throw std::runtime_error("foreground region exceeds declared size");
            const auto cell=key(p.x,p.y,p.z,config.background_distance_m);
            if(!cell) throw std::runtime_error("invalid foreground background coordinate");
            // The source page must belong to its declared spatial tile.
            const auto tile=region_key(p.x,p.y,p.z,voxel_size,tile_edge);
            if(!tile || *tile!=k) throw std::runtime_error("foreground region outside declared tile");
            entry.grid[*cell].push_back(p);++entry.points;
        });
        if(entry.points!=region->point_count) throw std::runtime_error("foreground region is incomplete");
        cached_points+=entry.points;
        return &cache.emplace(k,std::move(entry)).first->second;
    }
};
ForegroundEngine::ForegroundEngine(ForegroundConfig c):impl_(std::make_unique<Impl>(std::move(c))) {}
ForegroundEngine::~ForegroundEngine()=default;
void ForegroundEngine::reset(){impl_->clear();}

ForegroundEngine::PrefetchReport ForegroundEngine::prefetch(const models::ForegroundObservation& in,
    std::chrono::steady_clock::time_point deadline,std::size_t maximum_reads) {
    auto& s=*impl_;PrefetchReport report;
    if(std::chrono::steady_clock::now()>=deadline || !maximum_reads)return report;
    if(in.points.size()>s.config.maximum_input_points)throw std::invalid_argument("foreground prefetch sample exceeds point budget");
    if(!s.prepare(in,true))return report;
    const auto limit=std::min(maximum_reads,s.config.maximum_region_reads_per_frame);std::size_t read_points=0;
    const double r=s.config.background_distance_m;
    for(const auto& p:in.points){
        if(report.region_reads>=limit || std::chrono::steady_clock::now()>=deadline)break;
        const auto low=region_key(p.x-r,p.y-r,p.z-r,s.voxel_size,s.tile_edge),high=region_key(p.x+r,p.y+r,p.z+r,s.voxel_size,s.tile_edge);
        if(!low||!high||(*high)[0]-(*low)[0]>3||(*high)[1]-(*low)[1]>3||(*high)[2]-(*low)[2]>3)continue;
        for(auto x=(*low)[0];x<=(*high)[0] && report.region_reads<limit;++x)
        for(auto y=(*low)[1];y<=(*high)[1] && report.region_reads<limit;++y)
        for(auto z=(*low)[2];z<=(*high)[2] && report.region_reads<limit;++z){
            if(std::chrono::steady_clock::now()>=deadline)break;
            const Key k{x,y,z};const auto region=in.background->regions.find(k);
            if(region!=in.background->regions.end())s.acquire(k,region->second,report.region_reads,read_points,limit);
        }
    }
    report.cache_points=s.cached_points;report.cache_regions=s.cache.size();return report;
}

models::ForegroundFrame ForegroundEngine::process(const models::ForegroundObservation& in) {
    auto& s=*impl_;const auto& c=s.config;
    models::ForegroundFrame out;
    out.source=in.source;out.sequence=in.sequence;out.timestamp_ns=in.timestamp_ns;out.monotonic_ns=in.monotonic_ns;
    out.received_monotonic_ns=in.received_monotonic_ns;out.epoch=in.epoch;
    if(!s.prepare(in)){out.reason="confirmed background unavailable";return out;}
    const auto& background=*in.background;
    out.background_revision=background.geometry_revision;
    const auto stride=std::max<std::size_t>(1,(in.points.size()+c.maximum_input_points-1)/c.maximum_input_points);
    std::size_t read_points=0;
    const double radius=c.background_distance_m,radius2=radius*radius;
    // Rotate work order so a cold/budget-limited map region cannot starve forever.
    const auto count=(in.points.size()+stride-1)/stride;
    for(std::size_t n=0;n<count;++n) {
        const auto index=((n+in.sequence%std::max<std::size_t>(count,1))%count)*stride;
        const auto& p=in.points[index];
        const auto cell=key(p.x,p.y,p.z,radius);
        const auto low=region_key(p.x-radius,p.y-radius,p.z-radius,s.voxel_size,s.tile_edge);
        const auto high=region_key(p.x+radius,p.y+radius,p.z+radius,s.voxel_size,s.tile_edge);
        if(!cell || !low || !high) continue;
        ++out.input_points;
        bool matched=false,unknown=false,exhausted=false;
        std::size_t point_checks=0;
        if((*high)[0]-(*low)[0]>3 || (*high)[1]-(*low)[1]>3 || (*high)[2]-(*low)[2]>3) {
            ++out.unknown_points;continue;
        }
        for(auto x=(*low)[0];x<=(*high)[0] && !matched && !exhausted;++x)
        for(auto y=(*low)[1];y<=(*high)[1] && !matched && !exhausted;++y)
        for(auto z=(*low)[2];z<=(*high)[2] && !matched && !exhausted;++z) {
            const Key tile{x,y,z};const auto region=background.regions.find(tile);
            if(region==background.regions.end()) continue; // Complete catalog: this tile has no confirmed background.
            const auto* entry=s.acquire(tile,region->second,out.region_reads,read_points,c.maximum_region_reads_per_frame);
            if(!entry) {unknown=true;continue;}
            for(int dx=-1;dx<=1 && !matched && !exhausted;++dx) for(int dy=-1;dy<=1 && !matched && !exhausted;++dy) for(int dz=-1;dz<=1 && !matched && !exhausted;++dz) {
                const auto bucket=entry->grid.find({(*cell)[0]+dx,(*cell)[1]+dy,(*cell)[2]+dz});
                if(bucket==entry->grid.end()) continue;
                for(const auto& q:bucket->second) {
                    // Dense/duplicate imported PCDs must not turn one query into
                    // an unbounded nearest-neighbor scan. Budget misses are UNKNOWN.
                    if(point_checks>=512 || out.distance_checks>=1'000'000) {unknown=true;exhausted=true;break;}
                    ++point_checks;++out.distance_checks;
                    const double a=static_cast<double>(p.x)-q.x,b=static_cast<double>(p.y)-q.y,d=static_cast<double>(p.z)-q.z;
                    if(a*a+b*b+d*d<=radius2) {matched=true;break;}
                }
            }
        }
        if(matched) ++out.background_points;
        else if(unknown) ++out.unknown_points;
        else out.points.push_back(p);
    }
    out.cache_points=s.cached_points;out.cache_regions=s.cache.size();
    out.reason=out.unknown_points?"foreground; background cache warming/budget":"foreground; not a motion label";
    return out;
}

platform::WorkerHandle spawn_foreground_worker(platform::MessageBus& bus,platform::ThreadConfig thread,
    platform::StopToken stop,ForegroundConfig config,ForegroundCompletion complete) {
    validate_foreground_config(config);
    platform::WorkerTopicInputs inputs(thread.name);
    auto input=inputs.subscribe<devices::LidarPointCloudMessage>(bus,models::topics::pointcloud_cleaned);
    auto backgrounds=inputs.subscribe<models::RoomMapViewMessage>(bus,models::topics::room_map_view);
    std::optional<platform::WorkerTopicInput<models::LocalizationStatus>> poses;
    if(config.require_localization) poses.emplace(inputs.subscribe<models::LocalizationStatus>(bus,models::topics::localization_status));
    auto output=bus.publisher<models::ForegroundFrame>(models::topics::foreground_points);
    return platform::spawn_worker(std::move(thread),[inputs=std::move(inputs),input=std::move(input),backgrounds=std::move(backgrounds),
        poses=std::move(poses),output,stop,config,complete=std::move(complete)]() mutable {
        try {
            ForegroundEngine engine(config);ForegroundScanFilter scan_filter(config);ForegroundReport report;
            std::shared_ptr<const models::LocalizationStatus> pose;
            std::shared_ptr<const devices::LidarPointCloudMessage> latest;
            std::shared_ptr<const models::RoomMapViewMessage> background;
            std::uint64_t pose_arrival{},last_sequence{},last_epoch{},last_receipt{};
            std::string last_source,last_pause;bool have_sample=false,cleared=false,closed=false;
            auto next_sample=std::chrono::steady_clock::now()+config.publish_interval;
            std::size_t prefetch_tokens=4,prefetch_cursor=0;
            const auto publish=[&](models::ForegroundFrame frame) {
                ++report.published_frames;if(!frame.points.empty()) ++report.foreground_frames;
                report.foreground_points+=frame.points.size();report.background_points+=frame.background_points;report.unknown_points+=frame.unknown_points;
                report.peak_cache_points=std::max(report.peak_cache_points,frame.cache_points);
                report.peak_cache_regions=std::max(report.peak_cache_regions,frame.cache_regions);
                output.publish(std::move(frame));
            };
            const auto pause=[&](const std::string& reason,bool reset=true) {
                if(cleared && reason==last_pause) return;
                if(reset) engine.reset();
                models::ForegroundFrame frame;frame.reason=reason;publish(std::move(frame));
                cleared=true;last_pause=reason;
            };
            while(!stop.is_stop_requested() && !closed) {
                const auto wait=std::min(std::chrono::milliseconds(25),std::max(std::chrono::milliseconds(1),
                    std::chrono::duration_cast<std::chrono::milliseconds>(next_sample-std::chrono::steady_clock::now())));
                (void)inputs.wait_for(wait);
                for(std::size_t n=0;n<input.capacity();++n) {
                    std::shared_ptr<const devices::LidarPointCloudMessage> scan;
                    const auto status=input.try_receive(scan);
                    if(status!=platform::ReceiveStatus::message) {closed=status==platform::ReceiveStatus::closed;break;}
                    latest=std::move(scan);++report.received_messages;
                }
                for(std::size_t n=0;n<backgrounds.capacity();++n) {
                    std::shared_ptr<const models::RoomMapViewMessage> next;
                    if(backgrounds.try_receive(next)!=platform::ReceiveStatus::message) break;
                    background=std::move(next);
                }
                if(closed || stop.is_stop_requested()) break;
                const auto now_tp=std::chrono::steady_clock::now();
                const auto now=steady_ns();
                if(poses) {
                    std::shared_ptr<const models::LocalizationStatus> next;
                    for(std::size_t n=0;n<poses->capacity() && poses->try_receive(next)==platform::ReceiveStatus::message;++n) {pose=next;pose_arrival=now;}
                }
                const bool due=now_tp>=next_sample;
                if(due) {next_sample=now_tp+config.publish_interval;prefetch_tokens=4;} // No catch-up bursts after slow queries.
                const bool valid_pose=!config.require_localization || (pose && now-pose_arrival<pose_heartbeat_stale_ns &&
                    pose->pose_valid && pose->state==models::LocalizationState::tracking &&
                    std::isfinite(pose->position_variance) && pose->position_variance<0.01);
                const auto pose_problem=[&]() -> std::string {
                    if(!pose) return "waiting for localization status";
                    if(now-pose_arrival>=pose_heartbeat_stale_ns) return "localization status older than 2.5 seconds";
                    if(!pose->pose_valid || pose->state!=models::LocalizationState::tracking)
                        return "localization unavailable: "+pose->reason;
                    return "localization position variance must be below 0.01 m2";
                };
                if(!valid_pose) {pause(pose_problem());continue;}
                if(!latest || !latest->received_monotonic_ns || now<latest->received_monotonic_ns ||
                   now-latest->received_monotonic_ns>=stale_ns) {pause("no fresh valid world scan");continue;}
                if(!background || !background->source || background->state==models::RoomMapState::load_error ||
                   background->state==models::RoomMapState::storage_error || background->state==models::RoomMapState::disabled) {
                    pause("confirmed background unavailable");continue;
                }
                if(have_sample && latest->lidar_id==last_source && latest->sequence==last_sequence &&
                   latest->measurement_clock_epoch==last_epoch && latest->received_monotonic_ns==last_receipt) continue;
                if(!background->checkpoint_monotonic_ns || latest->received_monotonic_ns<=background->checkpoint_monotonic_ns) {
                    if(due)pause("waiting for a scan newer than the completed background checkpoint",false);
                    continue;
                }
                // Freeze only a shared immutable checkpoint pointer for this query.
                // No map-owner lock, mapper flush, or scan integration is requested.
                models::ForegroundObservation observation;
                observation.source=latest->lidar_id;observation.sequence=latest->sequence;
                observation.timestamp_ns=latest->received_timestamp_ns;
                observation.monotonic_ns=latest->measurement_timestamp_ns.value_or(latest->received_monotonic_ns);
                observation.received_monotonic_ns=latest->received_monotonic_ns;observation.epoch=latest->measurement_clock_epoch;
                const auto& points=latest->payload.points;
                if(!due){
                    if(!prefetch_tokens || points.empty() || next_sample-now_tp<std::chrono::milliseconds(8))continue;
                    const auto probes=std::min<std::size_t>(64,config.maximum_input_points);
                    const auto stride=std::max<std::size_t>(1,(points.size()+probes-1)/probes);
                    for(std::size_t n=0;n<probes && n<points.size();++n){const auto& p=points[(prefetch_cursor+n*stride)%points.size()];
                        if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z))observation.points.push_back({p.x,p.y,p.z,p.intensity});}
                    prefetch_cursor=(prefetch_cursor+1)%points.size();
                    const auto started=std::chrono::steady_clock::now();
                    try{observation.background=background->source->surface_snapshot();
                        const auto warm=engine.prefetch(observation,std::min(started+std::chrono::milliseconds(3),next_sample-std::chrono::milliseconds(5)),std::min<std::size_t>(2,prefetch_tokens));
                        prefetch_tokens-=warm.region_reads;report.prefetch_region_reads+=warm.region_reads;}
                    catch(const std::exception& error){++report.prefetch_failures;pause(std::string("background prefetch: ")+error.what());}
                    report.prefetch_ms+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
                    continue;
                }
                const auto stride=std::max<std::size_t>(1,(points.size()+config.maximum_input_points-1)/config.maximum_input_points);
                observation.points.reserve(std::min(points.size(),config.maximum_input_points));
                for(std::size_t i=0;i<points.size();i+=stride) {
                    const auto& p=points[i];if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z))
                        observation.points.push_back({p.x,p.y,p.z,p.intensity});
                }
                last_source=latest->lidar_id;last_sequence=latest->sequence;
                last_epoch=latest->measurement_clock_epoch;last_receipt=latest->received_monotonic_ns;have_sample=true;
                const auto started=std::chrono::steady_clock::now();models::ForegroundFrame frame;
                ++report.processed_frames;
                try {
                    observation.background=background->source->surface_snapshot();
                    const auto filtered=scan_filter.filter(observation.points,latest->payload.sensor_origin_world_m);
                    report.isolated_points+=filtered.isolated_points;
                    report.filter_unresolved_points+=filtered.unresolved_points;
                    report.filter_distance_checks+=filtered.distance_checks;
                    frame=engine.process(observation);
                    if(config.outlier_filter_enabled)
                        frame.reason+="; isolated_removed="+std::to_string(filtered.isolated_points)+
                            "; scan_filter_unresolved="+std::to_string(filtered.unresolved_points)+
                            "; adaptive_radius_points="+std::to_string(filtered.adaptive_points)+
                            "; missing_origin_points="+std::to_string(filtered.missing_origin_points);
                }
                catch(const std::exception& error) {engine.reset();++report.query_failures;frame.reason=error.what();}
                const auto elapsed=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
                report.processing_ms+=elapsed;report.maximum_processing_ms=std::max(report.maximum_processing_ms,elapsed);
                frame.source=observation.source;frame.sequence=observation.sequence;frame.timestamp_ns=observation.timestamp_ns;
                frame.monotonic_ns=observation.monotonic_ns;frame.received_monotonic_ns=observation.received_monotonic_ns;frame.epoch=observation.epoch;
                cleared=false;last_pause.clear();publish(std::move(frame));
            }
            output.publish(models::ForegroundFrame{});
            report.dropped_input_messages=input.dropped_messages();complete(report,{});
        } catch(const std::exception& error) {output.publish(models::ForegroundFrame{});complete(std::nullopt,error.what());}
    });
}
} // namespace vista::application
