#include "4_Applications/mapping/gaussian_map/gaussian_map.hpp"
#include "2_Transport/storage/room_map_tiles.hpp"
#include "2_Transport/storage/session_directory.hpp"
#include "models/topics.hpp"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <set>
#include <unordered_map>
namespace vista::application {
namespace {
using Key=models::RoomMapRegionKey;
using Point=models::GaussianMapPoint;
using Voxel=transport::MapVoxelKey;
using Cells=std::unordered_map<Voxel,models::RoomMapPoint,transport::MapVoxelHash>;
void write_point(std::ostream& out,const Point& p) {
    std::array<char,41> record{};
    std::memcpy(record.data(),&p.center.x,4);std::memcpy(record.data()+4,&p.center.y,4);
    std::memcpy(record.data()+8,&p.center.z,4);record[12]=static_cast<char>(p.center.intensity);
    std::memcpy(record.data()+13,p.covariance.data(),24);std::memcpy(record.data()+37,&p.opacity,4);
    out.write(record.data(),record.size());if(!out) throw std::runtime_error("cannot write Gaussian map cache");
}
Point read_point(std::istream& in) {
    std::array<char,41> record{};in.read(record.data(),record.size());
    if(!in) throw std::runtime_error("truncated Gaussian map cache");
    Point p;std::memcpy(&p.center.x,record.data(),4);std::memcpy(&p.center.y,record.data()+4,4);
    std::memcpy(&p.center.z,record.data()+8,4);p.center.intensity=static_cast<std::uint8_t>(record[12]);
    std::memcpy(p.covariance.data(),record.data()+13,24);std::memcpy(&p.opacity,record.data()+37,4);return p;
}
Voxel voxel(const models::RoomMapPoint& p,float size) {
    const double x=std::floor(static_cast<double>(p.x)/size),y=std::floor(static_cast<double>(p.y)/size),
        z=std::floor(static_cast<double>(p.z)/size);
    if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z)||
       std::abs(x)>1e12||std::abs(y)>1e12||std::abs(z)>1e12)
        throw std::runtime_error("invalid Gaussian source coordinate");
    return {static_cast<std::int64_t>(x),static_cast<std::int64_t>(y),static_cast<std::int64_t>(z)};
}
// Symmetric Jacobi eigensolver: no ROS/GPU or new numerical dependency.
std::array<double,3> normal(double a[3][3],bool& planar,double scale) {
    double v[3][3]{{1,0,0},{0,1,0},{0,0,1}};
    for(unsigned step=0;step<16;++step) {
        unsigned p=0,q=1;
        if(std::abs(a[0][2])>std::abs(a[p][q])) {p=0;q=2;}
        if(std::abs(a[1][2])>std::abs(a[p][q])) {p=1;q=2;}
        if(std::abs(a[p][q])<1e-14) break;
        const double angle=.5*std::atan2(2*a[p][q],a[q][q]-a[p][p]);
        const double c=std::cos(angle),s=std::sin(angle),ap=a[p][p],aq=a[q][q],off=a[p][q];
        a[p][p]=c*c*ap-2*c*s*off+s*s*aq;a[q][q]=s*s*ap+2*c*s*off+c*c*aq;
        a[p][q]=a[q][p]=0;
        for(unsigned k=0;k<3;++k) {
            if(k!=p && k!=q) {const auto kp=a[k][p],kq=a[k][q];
                a[k][p]=a[p][k]=c*kp-s*kq;a[k][q]=a[q][k]=s*kp+c*kq;}
            const auto vp=v[k][p],vq=v[k][q];v[k][p]=c*vp-s*vq;v[k][q]=s*vp+c*vq;
        }
    }
    std::array<unsigned,3> indices{0,1,2};
    std::sort(indices.begin(),indices.end(),[&](auto i,auto j){return a[i][i]<a[j][j];});
    planar=a[indices[1]][indices[1]]>scale*scale*.04 && a[indices[0]][indices[0]]<.15*a[indices[1]][indices[1]];
    return {v[0][indices[0]],v[1][indices[0]],v[2][indices[0]]};
}
// One bounded padded lookup table per active tile. Neighborhood order and all
// fitting math stay identical, but 343 hash lookups per point become direct reads.
class DenseCells {
    Voxel low{};std::size_t nx{},ny{},nz{};
    std::vector<const models::RoomMapPoint*> entries;
public:
    DenseCells(const Cells& cells,const std::vector<models::RoomMapPoint>& centers,float size) {
        if(centers.empty())return;
        auto high=voxel(centers.front(),size);low=high;
        for(const auto& p:centers){const auto k=voxel(p,size);
            low.x=std::min(low.x,k.x);low.y=std::min(low.y,k.y);low.z=std::min(low.z,k.z);
            high.x=std::max(high.x,k.x);high.y=std::max(high.y,k.y);high.z=std::max(high.z,k.z);}
        low.x-=3;low.y-=3;low.z-=3;high.x+=3;high.y+=3;high.z+=3;
        if(high.x-low.x>=72 || high.y-low.y>=72 || high.z-low.z>=72)
            throw std::runtime_error("Gaussian tile lookup exceeded");
        nx=static_cast<std::size_t>(high.x-low.x+1);ny=static_cast<std::size_t>(high.y-low.y+1);nz=static_cast<std::size_t>(high.z-low.z+1);
        entries.assign(nx*ny*nz,nullptr);
        for(const auto& item:cells){const auto& k=item.first;
            if(k.x<low.x||k.y<low.y||k.z<low.z||k.x>high.x||k.y>high.y||k.z>high.z)continue;
            entries[(static_cast<std::size_t>(k.x-low.x)*ny+static_cast<std::size_t>(k.y-low.y))*nz+static_cast<std::size_t>(k.z-low.z)]=&item.second;}
    }
    const models::RoomMapPoint* find(const Voxel& k) const {
        const auto x=k.x-low.x,y=k.y-low.y,z=k.z-low.z;
        if(x<0||y<0||z<0||static_cast<std::uint64_t>(x)>=nx||static_cast<std::uint64_t>(y)>=ny||static_cast<std::uint64_t>(z)>=nz)return nullptr;
        return entries[(static_cast<std::size_t>(x)*ny+static_cast<std::size_t>(y))*nz+static_cast<std::size_t>(z)];
    }
};
Point fit(const models::RoomMapPoint& center,const DenseCells& cells,float size) {
    const auto key=voxel(center,size);
    double mean[3]{},moments[3][3]{};unsigned count{};
    const double radius2=static_cast<double>(size)*size*6.25;
    for(int x=-3;x<=3;++x) for(int y=-3;y<=3;++y) for(int z=-3;z<=3;++z) {
        const auto* found=cells.find({key.x+x,key.y+y,key.z+z});if(!found) continue;
        const auto& p=*found;const double d[3]{p.x-center.x,p.y-center.y,p.z-center.z};
        if(d[0]*d[0]+d[1]*d[1]+d[2]*d[2]>radius2) continue;
        ++count;
        for(unsigned i=0;i<3;++i) {mean[i]+=d[i];for(unsigned j=0;j<3;++j) moments[i][j]+=d[i]*d[j];}
    }
    Point result;result.center=center;
    const float fallback=size*size*.0625F;result.covariance={fallback,0,0,fallback,0,fallback};
    if(count<6) return result;
    for(unsigned i=0;i<3;++i) for(unsigned j=0;j<3;++j)
        moments[i][j]=moments[i][j]/count-mean[i]*mean[j]/(static_cast<double>(count)*count);
    bool planar{};const auto n=normal(moments,planar,size);if(!planar) return result;
    const double tangent=size*size*.3025,thickness=size*size*.0064,difference=thickness-tangent;
    result.covariance={static_cast<float>(tangent+difference*n[0]*n[0]),static_cast<float>(difference*n[0]*n[1]),
        static_cast<float>(difference*n[0]*n[2]),static_cast<float>(tangent+difference*n[1]*n[1]),
        static_cast<float>(difference*n[1]*n[2]),static_cast<float>(tangent+difference*n[2]*n[2])};
    return result;
}
struct Page {
    std::shared_ptr<transport::detail::SessionDirectory> directory;
    vista::fs::path file;std::uint64_t count{};std::array<double,6> bounds{};
    ~Page() {std::error_code ignored;vista::fs::remove(file,ignored);}
};
class View final : public models::GaussianMapViewSource {
public:
    std::map<Key,std::shared_ptr<const Page>> pages;
    std::array<double,6> box{-1,-1,-1,1,1,1};
    std::array<double,6> bounds() const override {return box;}
    std::vector<Point> select_splats(const models::RoomMapViewRequest& request) const override {
        if(!request.point_budget || request.point_budget>2'000'000) throw std::invalid_argument("invalid Gaussian view budget");
        std::vector<std::shared_ptr<const Page>> visible;std::uint64_t total{};
        for(const auto& entry:pages) {
            const auto& page=entry.second;bool outside=false;
            if(request.has_camera) for(const auto& plane:request.planes) {
                const auto& b=page->bounds;
                if(plane[0]*b[plane[0]>=0?3:0]+plane[1]*b[plane[1]>=0?4:1]+
                   plane[2]*b[plane[2]>=0?5:2]+plane[3]<0) {outside=true;break;}
            }
            if(!outside && page->count) {visible.push_back(page);total+=page->count;}
        }
        std::vector<Point> result;result.reserve(static_cast<std::size_t>(std::min<std::uint64_t>(total,request.point_budget)));
        // Fixed-width random access bounds disk reads by the view budget, not map size.
        const auto wanted=std::min<std::uint64_t>(total,request.point_budget);
        std::uint64_t base{},sample_index{};
        for(const auto& page:visible) {
            vista::io::ifstream in(page->file,std::ios::binary);
            std::uint64_t next_record{};
            while(sample_index<wanted) {
                const auto index=static_cast<std::uint64_t>((static_cast<long double>(sample_index)+.5L)*total/wanted);
                if(index>=base+page->count) break;
                if(index-base!=next_record)in.seekg(static_cast<std::streamoff>((index-base)*41));
                next_record=index-base+1;
                auto p=read_point(in);bool inside=true;
                if(request.has_camera) for(const auto& plane:request.planes)
                    if(plane[0]*p.center.x+plane[1]*p.center.y+plane[2]*p.center.z+plane[3]<0) {inside=false;break;}
                if(inside) result.push_back(p);
                ++sample_index;
            }
            base+=page->count;
        }
        return result;
    }
};
}
class GaussianMapBuilder::Impl {
public:
    std::shared_ptr<transport::detail::SessionDirectory> directory=std::make_shared<transport::detail::SessionDirectory>();
    vista::fs::path root;
    std::map<Key,std::shared_ptr<const models::RoomMapRegion>> processed;
    std::shared_ptr<View> view=std::make_shared<View>();
    std::uint64_t serial{},rebuilt{};float voxel_size{},tile_size{};
};
GaussianMapBuilder::GaussianMapBuilder(vista::fs::path root):impl_(std::make_unique<Impl>()) {
    static std::atomic<std::uint64_t> counter{};
    if(root.empty()) root=vista::fs::temp_directory_path()/"vista-gaussian";
    root+="."+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+"."+std::to_string(++counter);
    impl_->root=vista::fs::absolute(root).lexically_normal();
    impl_->directory->root=impl_->root;
    // Derived geometry is regenerable from PCD. Pages keep this lease alive
    // after builder shutdown until the last in-flight viewer releases them.
    impl_->directory->discard=true;
}
GaussianMapBuilder::~GaussianMapBuilder()=default;
std::uint64_t GaussianMapBuilder::rebuilt_tiles() const {return impl_->rebuilt;}
std::shared_ptr<const models::GaussianMapViewSource> GaussianMapBuilder::update(
    const models::RoomMapSurfaceSnapshot& source,const std::function<bool()>& cancelled) {
    if(!std::isfinite(source.voxel_size)||source.voxel_size<.001F||!std::isfinite(source.tile_size)||
       source.tile_size<source.voxel_size || std::ceil(source.tile_size/source.voxel_size)>65)
        throw std::invalid_argument("invalid Gaussian source tile dimensions");
    if(!source.error.empty()) throw std::runtime_error(source.error);
    std::set<Key> changed;
    const bool dimensions=impl_->voxel_size!=source.voxel_size || impl_->tile_size!=source.tile_size;
    for(const auto& entry:source.regions) {
        const auto old=impl_->processed.find(entry.first);
        if(dimensions || old==impl_->processed.end() || old->second!=entry.second) changed.insert(entry.first);
    }
    for(const auto& entry:impl_->processed) if(!source.regions.count(entry.first)) changed.insert(entry.first);
    if(changed.empty()) return impl_->view;
    auto next=std::make_shared<View>(*impl_->view);if(dimensions) next->pages.clear();
    std::set<Key> affected;
    const int reach=static_cast<int>(std::ceil(3*source.voxel_size/source.tile_size));
    for(const auto& k:changed) for(int x=-reach;x<=reach;++x) for(int y=-reach;y<=reach;++y) for(int z=-reach;z<=reach;++z)
    {
        const Key neighbor{k[0]+x,k[1]+y,k[2]+z};
        if(source.regions.count(neighbor) || next->pages.count(neighbor)) affected.insert(neighbor);
    }
    impl_->directory->ensure();
    for(const auto& k:affected) {
        if(cancelled && cancelled()) return {};
        const auto own=source.regions.find(k);
        if(own==source.regions.end()) {next->pages.erase(k);continue;}
        Cells cells;const double margin=3.01*source.voxel_size;
        const double lo[3]{k[0]*source.tile_size-margin,k[1]*source.tile_size-margin,k[2]*source.tile_size-margin};
        const double hi[3]{(k[0]+1)*source.tile_size+margin,(k[1]+1)*source.tile_size+margin,(k[2]+1)*source.tile_size+margin};
        for(int x=-reach;x<=reach;++x) for(int y=-reach;y<=reach;++y) for(int z=-reach;z<=reach;++z) {
            const auto region=source.regions.find({k[0]+x,k[1]+y,k[2]+z});if(region==source.regions.end()) continue;
            region->second->read([&](const auto& p) {
                if(p.x>=lo[0]&&p.x<=hi[0]&&p.y>=lo[1]&&p.y<=hi[1]&&p.z>=lo[2]&&p.z<=hi[2])
                {
                    cells.emplace(voxel(p,source.voxel_size),p);
                    if(cells.size()>72U*72U*72U) throw std::runtime_error("Gaussian tile working set exceeded");
                }
            });
        }
        if(cells.size()>72U*72U*72U) throw std::runtime_error("Gaussian tile working set exceeded");
        std::vector<models::RoomMapPoint> centers;
        // Source voxel keys determine ownership exactly, including negative tiles.
        const auto edge=static_cast<std::int64_t>(std::llround(source.tile_size/source.voxel_size));
        const auto divide=[&](std::int64_t a){auto q=a/edge;return a%edge<0?q-1:q;};
        for(const auto& c:cells) if(Key{divide(c.first.x),divide(c.first.y),divide(c.first.z)}==k) centers.push_back(c.second);
        std::sort(centers.begin(),centers.end(),[](const auto& a,const auto& b){
            if(a.x!=b.x) return a.x<b.x;
            if(a.y!=b.y) return a.y<b.y;
            return a.z<b.z;});
        const DenseCells lookup(cells,centers,source.voxel_size);
        auto page=std::make_shared<Page>();page->file=impl_->root/(std::to_string(++impl_->serial)+".gsurf");
        page->directory=impl_->directory;
        page->count=centers.size();
        page->bounds={k[0]*source.tile_size,k[1]*source.tile_size,k[2]*source.tile_size,
            (k[0]+1)*source.tile_size,(k[1]+1)*source.tile_size,(k[2]+1)*source.tile_size};
        vista::io::ofstream out(page->file,std::ios::binary|std::ios::trunc);
        for(std::size_t i=0;i<centers.size();++i) {
            if(i%256==0 && cancelled && cancelled()) return {};
            write_point(out,fit(centers[i],lookup,source.voxel_size));
        }
        out.flush();if(!out) throw std::runtime_error("cannot flush Gaussian cache");out.close();
        next->pages.insert_or_assign(k,std::move(page));++impl_->rebuilt;
    }
    if(!next->pages.empty()) {
        next->box={std::numeric_limits<double>::infinity(),std::numeric_limits<double>::infinity(),std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity()};
        for(const auto& entry:next->pages) for(unsigned i=0;i<3;++i) {
            next->box[i]=std::min(next->box[i],entry.second->bounds[i]);
            next->box[i+3]=std::max(next->box[i+3],entry.second->bounds[i+3]);
        }
    } else next->box={-1,-1,-1,1,1,1};
    impl_->view=next;impl_->processed=source.regions;
    impl_->voxel_size=source.voxel_size;impl_->tile_size=source.tile_size;
    return next;
}
std::vector<std::uint8_t> encode_gaussian_map(const std::vector<Point>& points,std::uint64_t revision,std::uint64_t stamp) {
    if(points.size()>2'000'000) throw std::invalid_argument("Gaussian packet exceeds view budget");
    std::vector<std::uint8_t> bytes(32+points.size()*44);
    const auto write=[&](std::size_t offset,const auto& value){std::memcpy(bytes.data()+offset,&value,sizeof(value));};
    std::memcpy(bytes.data(),"GSP1",4);write(4,std::uint32_t{32});write(8,static_cast<std::uint32_t>(points.size()));
    write(12,std::uint32_t{44});write(16,revision);write(24,stamp);
    std::size_t offset=32;
    for(const auto& p:points) {
        write(offset,p.center.x);write(offset+4,p.center.y);write(offset+8,p.center.z);
        write(offset+12,static_cast<float>(p.center.intensity)/255.F);
        for(unsigned i=0;i<6;++i) write(offset+16+4*i,p.covariance[i]);
        write(offset+40,p.opacity);offset+=44;
    }
    return bytes;
}
platform::WorkerHandle spawn_gaussian_map_worker(platform::MessageBus& bus,platform::ThreadConfig thread,
    platform::StopToken stop,GaussianMapConfig config,GaussianMapCompletion complete) {
    auto subscriber=bus.subscribe<models::RoomMapViewMessage>(models::topics::room_map_view,thread.name);
    auto publisher=bus.publisher<models::RoomMapViewMessage>(models::topics::gaussian_map_view);
    return platform::spawn_worker(std::move(thread),[stop,config=std::move(config),subscriber=std::move(subscriber),
        publisher=std::move(publisher),complete=std::move(complete)]() mutable {
        try {
            GaussianMapBuilder builder(config.storage_directory);GaussianMapReport report;
            std::shared_ptr<const models::RoomMapSurfaceSnapshot> previous;
            std::shared_ptr<const models::GaussianMapViewSource> view;std::uint64_t revision{};
            std::string last_error;
            while(!stop.is_stop_requested()) {
                std::shared_ptr<const models::RoomMapViewMessage> latest,extra;
                const auto received=subscriber.receive_for(latest,std::chrono::milliseconds(50));
                if(received==platform::ReceiveStatus::closed) break;
                if(received!=platform::ReceiveStatus::message) continue;
                ++report.received_maps;
                while(subscriber.try_receive(extra)==platform::ReceiveStatus::message) {latest=extra;++report.received_maps;}
                try {
                    const auto snapshot=latest->source?latest->source->surface_snapshot():nullptr;
                    if(snapshot && !snapshot->error.empty()) throw std::runtime_error(snapshot->error);
                    if(snapshot && (!previous || snapshot->geometry_revision!=previous->geometry_revision || snapshot->regions!=previous->regions)) {
                        auto updated=builder.update(*snapshot,[&]{return stop.is_stop_requested();});if(!updated) break;
                        view=std::move(updated);previous=snapshot;++revision;++report.completed_revisions;
                    } else if(!snapshot) {view.reset();previous.reset();++revision;}
                    publisher.publish(models::RoomMapViewMessage{view,revision,latest->timestamp_ns,latest->point_count,latest->state});
                    last_error.clear();
                } catch(const std::exception& error) {
                    ++report.failed_revisions;
                    if(last_error!=error.what()) {std::cerr<<"Gaussian map revision failed: "<<error.what()<<'\n';last_error=error.what();}
                    publisher.publish(models::RoomMapViewMessage{nullptr,++revision,latest->timestamp_ns,0,models::RoomMapState::storage_error});
                }
            }
            report.rebuilt_tiles=builder.rebuilt_tiles();complete(report,{});
        } catch(const std::exception& error) {complete(std::nullopt,error.what());}
          catch(...) {complete(std::nullopt,"unknown Gaussian mapping failure");}
    });
}
}
