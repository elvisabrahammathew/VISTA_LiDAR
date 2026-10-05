#include "2_Transport/storage/room_map_tiles.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>
#include <unordered_set>

namespace vista::transport {
std::size_t MapVoxelHash::operator()(const MapVoxelKey& key) const noexcept {
    auto h=std::hash<std::int64_t>{}(key.x);
    h^=std::hash<std::int64_t>{}(key.y)+0x9e3779b9U+(h<<6U)+(h>>2U);
    return h^(std::hash<std::int64_t>{}(key.z)+0x9e3779b9U+(h<<6U)+(h>>2U));
}
namespace {
constexpr unsigned tree_depth=24; // Eight signed roots; no total point-count cap.
constexpr std::int64_t root_tiles=std::int64_t{1}<<tree_depth;
std::int64_t floor_div(std::int64_t a,std::int64_t b) { auto q=a/b; return a%b<0?q-1:q; }
// Explicit primitives avoid ABI padding differences across Windows/Linux ARM/x64.
// Supported targets use IEEE754 floats and little-endian integer representation.
template<class T> void put(std::ostream& out,const T& v) {
    out.write(reinterpret_cast<const char*>(&v),sizeof(v));
    if(!out) throw std::runtime_error("room-map tile write failed (check disk space/permissions)");
}
template<class T> T get(std::istream& in) {
    T v{}; in.read(reinterpret_cast<char*>(&v),sizeof(v));
    if(!in) throw std::runtime_error("room-map tile is truncated/corrupt"); return v;
}
void put_point(std::ostream& out,const models::PointXYZIRT& p) {
    put(out,p.x);put(out,p.y);put(out,p.z);put(out,p.intensity);
    put(out,p.ring);put(out,p.return_id);put(out,p.timestamp_ns);
}
models::PointXYZIRT get_point(std::istream& in) {
    models::PointXYZIRT p; p.x=get<float>(in);p.y=get<float>(in);p.z=get<float>(in);
    p.intensity=get<std::uint8_t>(in);p.ring=get<std::uint8_t>(in);
    p.return_id=get<std::uint8_t>(in);p.timestamp_ns=get<std::uint64_t>(in);
    if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)) throw std::runtime_error("invalid tile coordinate");
    return p;
}
void put_cell(std::ostream& out,const MapVoxelKey& k,const MapVoxelCell& c) {
    put(out,k.x);put(out,k.y);put(out,k.z);put_point(out,c.point);
    put(out,static_cast<std::uint64_t>(c.observations));put(out,c.last_hit_window);
    put(out,static_cast<std::uint64_t>(c.free_observations));
}
std::pair<MapVoxelKey,MapVoxelCell> get_cell(std::istream& in) {
    MapVoxelKey k{get<std::int64_t>(in),get<std::int64_t>(in),get<std::int64_t>(in)};
    MapVoxelCell c; c.point=get_point(in);c.observations=static_cast<std::size_t>(get<std::uint64_t>(in));
    c.last_hit_window=get<std::int64_t>(in);c.free_observations=static_cast<std::size_t>(get<std::uint64_t>(in));
    return {k,c};
}
std::uint64_t mix(std::uint64_t x) {
    x=(x^(x>>30))*0xbf58476d1ce4e5b9ULL; x=(x^(x>>27))*0x94d049bb133111ebULL; return x^(x>>31);
}
void sample(std::vector<models::PointXYZIRT>& v,const models::PointXYZIRT& p,
            std::uint64_t count,std::size_t cap) {
    if(v.size()<cap) v.push_back(p);
    else { const auto index=mix(count)%count; if(index<cap) v[static_cast<std::size_t>(index)]=p; }
}
struct Node {
    std::filesystem::path path;
    MapVoxelKey origin;
    unsigned level{};
    std::uint64_t count{};
    std::vector<models::PointXYZIRT> points;
    bool full{},blocked{};
};
/// Records cumulative wall time including file decoding/callback work.
struct MeasureIo {
    std::atomic<std::uint64_t>& total;
    const std::chrono::steady_clock::time_point started{std::chrono::steady_clock::now()};
    ~MeasureIo() {
        total.fetch_add(static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now()-started).count()),std::memory_order_relaxed);
    }
};
}

class RoomMapTileStore::Impl {
public:
    struct Tile {
        MapVoxelKey key;
        std::unordered_map<MapVoxelKey,MapVoxelCell,MapVoxelHash> cells;
        std::vector<models::PointXYZIRT> reference;
        std::uint64_t touch{};
        bool dirty{},on_disk{};
    };
    Impl(std::filesystem::path preferred,float voxel,float tile,std::size_t voxels,
         std::size_t tiles,std::size_t threshold,std::size_t lod)
        : voxel_size(voxel),edge(static_cast<std::int64_t>(std::ceil(tile/voxel))),
          cache_limit(voxels),tile_limit(tiles),confirmation(threshold),lod_limit(lod) {
        static std::atomic<std::uint64_t> counter{};
        if(preferred.empty()) preferred=std::filesystem::temp_directory_path()/"vista-map-tiles";
        root=preferred; root+= "."+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+
            "."+std::to_string(++counter);
    }
    MapVoxelKey tile_key(const MapVoxelKey& k) const {
        MapVoxelKey t{floor_div(k.x,edge),floor_div(k.y,edge),floor_div(k.z,edge)};
        if(t.x < -root_tiles || t.x>=root_tiles || t.y < -root_tiles || t.y>=root_tiles || t.z < -root_tiles || t.z>=root_tiles)
            throw std::runtime_error("map coordinate exceeds octree spatial range");
        return t;
    }
    std::vector<Node> chain(const MapVoxelKey& t) const {
        const auto octant=(t.x>=0?1U:0U)|(t.y>=0?2U:0U)|(t.z>=0?4U:0U);
        Node node{root/std::to_string(octant),{t.x>=0?0:-root_tiles,t.y>=0?0:-root_tiles,t.z>=0?0:-root_tiles},tree_depth};
        std::vector<Node> result{node};
        for(unsigned level=tree_depth;level>0;--level) {
            const auto half=std::int64_t{1}<<(level-1);
            const auto child=(t.x>=node.origin.x+half?1U:0U)|(t.y>=node.origin.y+half?2U:0U)|(t.z>=node.origin.z+half?4U:0U);
            node.path/=std::to_string(child);node.level=level-1;
            if(child&1U) node.origin.x+=half;if(child&2U) node.origin.y+=half;if(child&4U) node.origin.z+=half;
            result.push_back(node);
        }
        return result;
    }
    Node read_node(Node n) const {
        const auto cached=node_cache.find(n.path);
        if(cached!=node_cache.end()) {
            cached->second.second=++node_clock;
            return cached->second.first;
        }
        const auto file=n.path/"lod.bin";
        if(!std::filesystem::exists(file)) return n;
        std::ifstream in(file,std::ios::binary);
        if(get<std::uint32_t>(in)!=0x31444f4cU) throw std::runtime_error("unsupported room-map LOD node");
        n.count=get<std::uint64_t>(in);const auto size=get<std::uint32_t>(in);
        if(size>lod_limit) throw std::runtime_error("room-map LOD node exceeds its budget");
        n.points.reserve(size);for(std::uint32_t i=0;i<size;++i) n.points.push_back(get_point(in));
        // Bound cached disk summaries independently of total map size. Empty
        // children are not cached, so they cannot evict useful LOD summaries.
        if(node_cache.size()>=512) {
            auto oldest=node_cache.begin();
            for(auto it=node_cache.begin();it!=node_cache.end();++it)
                if(it->second.second<oldest->second.second) oldest=it;
            node_cache.erase(oldest);
        }
        node_cache.emplace(n.path,std::make_pair(n,++node_clock));return n;
    }
    void write_node(const Node& n) {
        std::filesystem::create_directories(n.path);
        const auto file=n.path/"lod.bin"; auto temporary=file;temporary+=".tmp";
        {std::ofstream out(temporary,std::ios::binary|std::ios::trunc);
         put(out,std::uint32_t{0x31444f4cU});put(out,n.count);put(out,static_cast<std::uint32_t>(n.points.size()));
         for(const auto& p:n.points) put_point(out,p);out.flush();if(!out) throw std::runtime_error("cannot flush LOD node");}
        atomic_replace_map_file(temporary,file);
        node_cache.erase(n.path); // A later query must see the replacement summary.
        ++lod_node_writes;
    }
    std::vector<Node> children(const Node& n) const {
        std::vector<Node> result;
        const auto half=std::int64_t{1}<<(n.level-1);
        for(unsigned child=0;child<8;++child) {
            Node c{n.path/std::to_string(child),{n.origin.x+((child&1U)?half:0),n.origin.y+((child&2U)?half:0),n.origin.z+((child&4U)?half:0)},n.level-1};
            c=read_node(std::move(c));if(c.count) result.push_back(std::move(c));
        }
        return result;
    }
    void read_cells(const std::filesystem::path& file,const std::function<void(const MapVoxelKey&,const MapVoxelCell&)>& visit) const {
        if(!std::filesystem::exists(file)) return;
        ++tile_reads;
        MeasureIo timer{tile_read_ns};
        std::ifstream in(file,std::ios::binary);
        if(get<std::uint32_t>(in)!=0x31564d52U) throw std::runtime_error("unsupported room-map voxel page");
        const auto count=get<std::uint64_t>(in);
        for(std::uint64_t i=0;i<count;++i) {auto v=get_cell(in);visit(v.first,v.second);}
    }
    /// Leaf pages are saved before eviction. Shared ancestors are rebuilt once
    /// per batch instead of rewriting the same 24 ancestors for every leaf.
    /// This pending set is bounded independently of the map's total size.
    void flush_parents() {
        std::lock_guard<std::recursive_mutex> disk_lock(disk_mutex);
        if(pending_parents.empty()) return;
        MeasureIo timer{lod_update_ns};
        std::vector<Node> nodes;nodes.reserve(pending_parents.size());
        for(const auto& entry:pending_parents) nodes.push_back(entry.second);
        std::sort(nodes.begin(),nodes.end(),[](const auto& a,const auto& b){return a.level<b.level;});
        for(auto& parent:nodes) {
            std::uint64_t seen{};
            for(const auto& c:children(parent)) {
                parent.count+=c.count;
                for(const auto& p:c.points) sample(parent.points,p,++seen,lod_limit);
            }
            const auto previous=read_node(Node{parent.path,parent.origin,parent.level});
            if(previous.count!=parent.count || previous.points!=parent.points) write_node(parent);
        }
        pending_parents.clear();
    }
    void persist(Tile& tile) {
        if(!tile.dirty) return;
        // Disk readers never access RAM cache/evidence. Only file replacement
        // competes with a query, not ordinary integration into resident tiles.
        std::lock_guard<std::recursive_mutex> disk_lock(disk_mutex);
        const auto write_started=std::chrono::steady_clock::now();
        auto nodes=chain(tile.key);auto& leaf=nodes.back();
        const auto file=leaf.path/"cells.bin";auto temporary=file;temporary+=".tmp";
        std::filesystem::create_directories(leaf.path);
        std::uint64_t old_count{};
        if(reference_mode && tile.on_disk) {
            std::ifstream in(file,std::ios::binary);(void)get<std::uint32_t>(in);old_count=get<std::uint64_t>(in);
        }
        const auto count=reference_mode?old_count+tile.reference.size():tile.cells.size();
        Node summary=leaf;std::uint64_t observed{};
        {std::ofstream out(temporary,std::ios::binary|std::ios::trunc);
         put(out,std::uint32_t{0x31564d52U});put(out,static_cast<std::uint64_t>(count));
         const auto record=[&](const MapVoxelKey& k,const MapVoxelCell& c) {
             put_cell(out,k,c);if(c.observations>=confirmation) {++summary.count;sample(summary.points,c.point,++observed,lod_limit);}
         };
         if(reference_mode) {
             if(tile.on_disk) read_cells(file,record);
             for(const auto& p:tile.reference) record({0,0,0},{p,confirmation,-1,0});
         } else for(const auto& c:tile.cells) record(c.first,c.second);
         out.flush();if(!out) throw std::runtime_error("cannot flush room-map voxel page");}
        atomic_replace_map_file(temporary,file);
        ++tile_writes;
        tile_write_ns.fetch_add(static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now()-write_started).count()),std::memory_order_relaxed);
        // Hit/free counters may change without changing confirmed geometry.
        // Persist that evidence, but do not rewrite all 24 LOD ancestors unless
        // the leaf's count or representative points actually changed.
        const auto previous=read_node(leaf);
        if(previous.count!=summary.count || previous.points!=summary.points) {
            write_node(summary);
            for(std::size_t i=nodes.size()-1;i>0;--i)
                pending_parents.insert_or_assign(nodes[i-1].path,nodes[i-1]);
        }
        if(pending_parents.size()>=4096) flush_parents();
        if(!tile.on_disk) {++stored_tiles;tile.on_disk=true;}
        tile.dirty=false;
        if(reference_mode) {resident-=tile.reference.size();tile.reference.clear();tile.reference.shrink_to_fit();}
    }
    void evict(const MapVoxelKey* exclude=nullptr) {
        auto chosen=cache.end();
        for(auto it=cache.begin();it!=cache.end();++it) {
            if(exclude && it->first==*exclude) continue;
            if(chosen==cache.end()||it->second->touch<chosen->second->touch) chosen=it;
        }
        if(chosen==cache.end()) throw std::runtime_error("RAM tile budget cannot fit one tile; increase RoomMapCacheMaxVoxels");
        persist(*chosen->second);resident-=chosen->second->cells.size()+chosen->second->reference.size();cache.erase(chosen);++eviction_count;
    }
    Tile& acquire(const MapVoxelKey& key) {
        const auto t=tile_key(key);auto found=cache.find(t);
        if(found!=cache.end()) {found->second->touch=++clock;return *found->second;}
        ++cache_misses;
        while(cache.size()>=tile_limit) evict();
        auto tile=std::make_unique<Tile>();tile->key=t;tile->touch=++clock;
        const auto file=chain(t).back().path/"cells.bin";
        tile->on_disk=std::filesystem::exists(file);
        if(tile->on_disk && !reference_mode) {
            std::ifstream in(file,std::ios::binary);(void)get<std::uint32_t>(in);const auto count=get<std::uint64_t>(in);
            if(count>cache_limit) throw std::runtime_error("voxel tile exceeds RAM cache budget");
            while(resident+count>cache_limit) evict();
            read_cells(file,[&](const auto& k,const auto& c){tile->cells.emplace(k,c);});resident+=tile->cells.size();
            // Expire only stale tentative evidence when an evicted tile returns.
            // Confirmed, occluded geometry is never removed by age.
            if(prune_window>=0) for(auto it=tile->cells.begin();it!=tile->cells.end();) {
                if(it->second.observations<confirmation && prune_window-it->second.last_hit_window>prune_age) {
                    it=tile->cells.erase(it);--resident;--total;tile->dirty=true;
                } else ++it;
            }
        }
        auto* pointer=tile.get();cache.emplace(t,std::move(tile));return *pointer;
    }
    void note_bounds(const models::PointXYZIRT& p) {
        const double v[3]{p.x,p.y,p.z};
        for(std::size_t i=0;i<3;++i) {box[i]=std::min(box[i],v[i]);box[i+3]=std::max(box[i+3],v[i]);}
    }
    std::array<double,6> node_bounds(const Node& n) const {
        const auto width=static_cast<double>(edge)*voxel_size;
        const auto size=width*static_cast<double>(std::int64_t{1}<<n.level);
        return {n.origin.x*width,n.origin.y*width,n.origin.z*width,n.origin.x*width+size,n.origin.y*width+size,n.origin.z*width+size};
    }
    bool visible(const Node& n,const models::RoomMapViewRequest& request) const {
        if(!request.has_camera) return true;
        const auto b=node_bounds(n);
        for(const auto& p:request.planes) {
            if(p[0]*b[p[0]>=0?3:0]+p[1]*b[p[1]>=0?4:1]+p[2]*b[p[2]>=0?5:2]+p[3] < -1.0e-5) return false;
        }
        return true;
    }
    double score(const Node& n,const models::RoomMapViewRequest& request) const {
        if(!request.has_camera) return static_cast<double>(n.count)+static_cast<double>(n.level);
        const auto b=node_bounds(n);double d2{};
        for(std::size_t i=0;i<3;++i) {const auto d=(b[i]+b[i+3])*0.5-request.camera[i];d2+=d*d;}
        const auto radius=(b[3]-b[0])*0.866025403784;
        return request.viewport_height*radius/std::max(0.01,std::sqrt(d2)-radius);
    }
    /// Read the disk index only: no RAM flush, accumulator mutex, or cache eviction.
    models::PointCloudFrame select_disk_view(const models::RoomMapViewRequest&) const;
    mutable std::recursive_mutex disk_mutex;
    // std::map also supports the VS2019 filesystem implementation, whose
    // std::hash<path> specialization is unavailable on some toolset versions.
    mutable std::map<std::filesystem::path,std::pair<Node,std::uint64_t>> node_cache;
    mutable std::uint64_t node_clock{};
    mutable std::atomic<std::uint64_t> tile_reads{}, view_queries{};
    std::atomic<std::uint64_t> cache_misses{}, tile_writes{}, lod_node_writes{};
    mutable std::atomic<std::uint64_t> last_view_ns{}, last_view_wait_ns{};
    mutable std::atomic<std::uint64_t> tile_read_ns{};
    std::atomic<std::uint64_t> tile_write_ns{},lod_update_ns{};
    std::filesystem::path root;
    float voxel_size;
    std::int64_t edge;
    std::size_t cache_limit,tile_limit,confirmation,lod_limit,resident{},total{};
    std::uint64_t clock{},eviction_count{},stored_tiles{};
    bool reference_mode{};
    std::int64_t prune_window{-1},prune_age{};
    std::unordered_map<MapVoxelKey,std::unique_ptr<Tile>,MapVoxelHash> cache;
    std::unordered_set<MapVoxelKey,MapVoxelHash> missing_tiles; // Bounded negative lookup cache for free-space rays.
    std::map<std::filesystem::path,Node> pending_parents;
    std::array<double,6> box{INFINITY,INFINITY,INFINITY,-INFINITY,-INFINITY,-INFINITY};
};

RoomMapTileStore::RoomMapTileStore(std::filesystem::path root,float voxel,float tile,std::size_t voxels,
    std::size_t tiles,std::size_t threshold,std::size_t lod)
    :impl_(std::make_shared<Impl>(std::move(root),voxel,tile,voxels,tiles,threshold,lod)) {}
RoomMapTileStore::~RoomMapTileStore()=default;
MapVoxelCell* RoomMapTileStore::find(const MapVoxelKey& k) {
    const auto key=impl_->tile_key(k);
    if(impl_->cache.find(key)==impl_->cache.end()) {
        if(impl_->missing_tiles.count(key)) return nullptr;
        if(!std::filesystem::exists(impl_->chain(key).back().path/"cells.bin")) {
            if(impl_->missing_tiles.size()>=4096) impl_->missing_tiles.clear();
            impl_->missing_tiles.insert(key);return nullptr;
        }
    }
    auto& tile=impl_->acquire(k);auto it=tile.cells.find(k);
    if(it==tile.cells.end()) return nullptr;
    return &it->second;
}
void RoomMapTileStore::mark_changed(const MapVoxelKey& k) {
    // The caller marks immediately after mutation, before any later acquisition
    // can evict the tile. Read-only ray lookups do not call this function.
    const auto tile=impl_->cache.find(impl_->tile_key(k));
    if(tile==impl_->cache.end()) throw std::logic_error("changed voxel is not resident");
    tile->second->dirty=true;
}
MapVoxelKey RoomMapTileStore::tile_key(const MapVoxelKey& k) const {return impl_->tile_key(k);}
MapVoxelCell& RoomMapTileStore::insert(const MapVoxelKey& k,const MapVoxelCell& c) {
    const auto t=impl_->tile_key(k);auto& tile=impl_->acquire(k);
    impl_->missing_tiles.erase(t);
    while(impl_->resident>=impl_->cache_limit) impl_->evict(&t);
    auto result=tile.cells.emplace(k,c);tile.dirty=true;if(result.second) {++impl_->resident;++impl_->total;impl_->note_bounds(c.point);}
    return result.first->second;
}
void RoomMapTileStore::erase(const MapVoxelKey& k) {
    auto& tile=impl_->acquire(k);if(tile.cells.erase(k)) {--impl_->resident;--impl_->total;tile.dirty=true;}
}
std::size_t RoomMapTileStore::size() const {return impl_->total;}
std::size_t RoomMapTileStore::prune_cached(std::int64_t window,std::int64_t age) {
    impl_->prune_window=window;impl_->prune_age=age;
    std::size_t removed{};
    for(auto& t:impl_->cache) for(auto it=t.second->cells.begin();it!=t.second->cells.end();) {
        if(it->second.observations<impl_->confirmation && window-it->second.last_hit_window>age) {
            it=t.second->cells.erase(it);--impl_->resident;--impl_->total;++removed;t.second->dirty=true;
        } else ++it;
    }
    return removed;
}
void RoomMapTileStore::flush() {
    std::lock_guard<std::recursive_mutex> disk_lock(impl_->disk_mutex);
    for(auto& t:impl_->cache) impl_->persist(*t.second);
    impl_->flush_parents(); // All disk LOD queries see a coherent, current hierarchy.
}
void RoomMapTileStore::set_reference() {impl_->reference_mode=true;}
void RoomMapTileStore::append_reference(const models::PointXYZIRT& p) {
    const MapVoxelKey k{static_cast<std::int64_t>(std::floor(static_cast<double>(p.x)/impl_->voxel_size)),
        static_cast<std::int64_t>(std::floor(static_cast<double>(p.y)/impl_->voxel_size)),
        static_cast<std::int64_t>(std::floor(static_cast<double>(p.z)/impl_->voxel_size))};
    const auto t=impl_->tile_key(k);auto& tile=impl_->acquire(k);
    while(impl_->resident>=impl_->cache_limit) {
        if(impl_->cache.size()==1) impl_->persist(tile);else impl_->evict(&t);
    }
    tile.reference.push_back(p);tile.dirty=true;++impl_->resident;++impl_->total;impl_->note_bounds(p);
}
void RoomMapTileStore::for_each_confirmed(const RoomMapPointVisitor& visit) {
    flush();if(!std::filesystem::exists(impl_->root)) return;
    for(const auto& entry:std::filesystem::recursive_directory_iterator(impl_->root)) {
        if(entry.is_regular_file() && entry.path().filename()=="cells.bin")
            impl_->read_cells(entry.path(),[&](const auto&,const auto& c){if(c.observations>=impl_->confirmation) visit(c.point);});
    }
}
models::PointCloudFrame RoomMapTileStore::select_view(const models::RoomMapViewRequest& request) {
    flush();
    return impl_->select_disk_view(request);
}
models::PointCloudFrame RoomMapTileStore::Impl::select_disk_view(const models::RoomMapViewRequest& request) const {
    if(!request.point_budget || request.point_budget>2'000'000) throw std::invalid_argument("invalid room-map view point budget");
    const auto before_lock=std::chrono::steady_clock::now();
    std::lock_guard<std::recursive_mutex> disk_lock(disk_mutex);
    const auto started=std::chrono::steady_clock::now();
    last_view_wait_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(started-before_lock).count();
    models::PointCloudFrame result;std::vector<Node> selected;std::size_t points{};
    for(unsigned octant=0;octant<8;++octant) {
        auto n=read_node({root/std::to_string(octant),{(octant&1U)?0:-root_tiles,(octant&2U)?0:-root_tiles,(octant&4U)?0:-root_tiles},tree_depth});
        if(n.count && visible(n,request)) {points+=n.points.size();selected.push_back(std::move(n));}
    }
    // Replace coarse parents with children by projected screen size. Traversal
    // and cut-set sizes are bounded independently of the total map on disk.
    for(std::size_t visits=0;visits<8192 && selected.size()<4096;++visits) {
        auto index=selected.size();double priority=-1;
        for(std::size_t i=0;i<selected.size();++i) if(!selected[i].blocked && !selected[i].full) {
            const auto value=score(selected[i],request);
            if(value>priority) {priority=value;index=i;}
        }
        if(index==selected.size() || (request.has_camera && priority<2.0)) break;
        auto& node=selected[index];
        if(!node.level) {
            if(node.count<=request.point_budget && points-node.points.size()+node.count<=request.point_budget) {
                points=points-node.points.size()+static_cast<std::size_t>(node.count);node.full=true;
            } else node.blocked=true;
            continue;
        }
        auto descendants=children(node);std::size_t count{};
        descendants.erase(std::remove_if(descendants.begin(),descendants.end(),[&](const Node& c){return !visible(c,request);}),descendants.end());
        for(const auto& c:descendants) count+=c.points.size();
        if(points-node.points.size()+count>request.point_budget || selected.size()+descendants.size()>4096) {node.blocked=true;continue;}
        points=points-node.points.size()+count;
        selected.erase(selected.begin()+static_cast<std::ptrdiff_t>(index));
        for(auto& c:descendants) selected.push_back(std::move(c));
    }
    result.points.reserve(std::min(points,request.point_budget));std::uint64_t seen{};
    const auto emit=[&](const models::PointXYZIRT& p){
        if(request.has_camera) for(const auto& plane:request.planes)
            if(plane[0]*p.x+plane[1]*p.y+plane[2]*p.z+plane[3]<-1e-5) return;
        sample(result.points,p,++seen,request.point_budget);
    };
    for(const auto& n:selected) {
        if(n.full) read_cells(n.path/"cells.bin",[&](const auto&,const auto& c){if(c.observations>=confirmation) emit(c.point);});
        else for(const auto& p:n.points) emit(p);
    }
    last_view_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now()-started).count();
    ++view_queries;
    return result;
}
std::shared_ptr<const models::RoomMapViewSource> RoomMapTileStore::view_source(std::uint64_t timestamp_ns) {
    flush();
    class DiskView final : public models::RoomMapViewSource {
    public:
        DiskView(std::shared_ptr<Impl> storage,std::array<double,6> box,std::uint64_t stamp)
            :storage_(std::move(storage)),box_(box),stamp_(stamp) {}
        models::PointCloudFrame select_view(const models::RoomMapViewRequest& request) const override {
            auto cloud=storage_->select_disk_view(request);cloud.timestamp_ns=stamp_;return cloud;
        }
        std::array<double,6> bounds() const override {return box_;}
    private:
        std::shared_ptr<Impl> storage_;
        const std::array<double,6> box_;
        const std::uint64_t stamp_;
    };
    return std::make_shared<DiskView>(impl_,bounds(),timestamp_ns);
}
RoomMapTileMetrics RoomMapTileStore::metrics() const {
    return {impl_->cache_misses.load(),impl_->tile_reads.load(),impl_->tile_writes.load(),
        impl_->lod_node_writes.load(),impl_->view_queries.load(),
        impl_->last_view_ns.load()/1'000'000.0,impl_->last_view_wait_ns.load()/1'000'000.0,
        impl_->tile_read_ns.load()/1'000'000.0,impl_->tile_write_ns.load()/1'000'000.0,
        impl_->lod_update_ns.load()/1'000'000.0};
}
std::array<double,6> RoomMapTileStore::bounds() const {
    if(!std::isfinite(impl_->box[0])) return {-1,-1,-1,1,1,1};return impl_->box;
}
std::size_t RoomMapTileStore::resident_voxels() const {return impl_->resident;}
std::size_t RoomMapTileStore::resident_tiles() const {return impl_->cache.size();}
std::uint64_t RoomMapTileStore::evictions() const {return impl_->eviction_count;}
std::uint64_t RoomMapTileStore::disk_tiles() const {return impl_->stored_tiles;}
const std::filesystem::path& RoomMapTileStore::directory() const {return impl_->root;}
} // namespace vista::transport
